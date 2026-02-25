/* sd_storage.c
 * SD Card storage backend implementation
 * Reads disk images from FAT filesystem on SD card via SDIO interface
 *
 * Based on working implementation from user_port_v9k project.
 */

#define __GNU_VISIBLE 1
#define _GNU_SOURCE 1

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"

// no-OS-FatFS includes
#include "ff.h"
#include "f_util.h"
#include "hw_config.h"

#include "sd_storage.h"
#include "storage.h"
#include "fatfs_guard.h"

// Set to 1 to enable debug printf during sector I/O (WARNING: breaks timing-critical DMA operations)
#define SD_DEBUG_PRINTF 0

#if SD_DEBUG_PRINTF
#define sd_printf(...) printf(__VA_ARGS__)
#else
#define sd_printf(...) ((void)0)
#endif

// Maximum number of disk image files to scan
#define MAX_IMG_FILES 8
#define FILENAME_MAX_LENGTH 64

// Per-target disk image state
typedef struct {
    bool mounted;
    bool read_only;
    char image_path[FILENAME_MAX_LENGTH];
    FIL file;
    uint32_t capacity_sectors;
} sd_target_t;

// SD card state
typedef struct {
    FATFS *fs;
    char file_names[MAX_IMG_FILES][FILENAME_MAX_LENGTH];
    int file_count;
    sd_target_t targets[STORAGE_MAX_TARGETS];
} sd_state_t;

static sd_state_t *sd_state = NULL;
static bool sd_initialized = false;

static bool sd_storage_is_fatal_error(FRESULT fr) {
    return fr == FR_DISK_ERR ||
           fr == FR_INT_ERR ||
           fr == FR_NOT_READY ||
           fr == FR_INVALID_OBJECT;
}

static void sd_storage_disable_target(uint8_t target_id, const char *op, FRESULT fr) {
    if (!sd_state || target_id >= STORAGE_MAX_TARGETS) {
        return;
    }

    sd_target_t *target = &sd_state->targets[target_id];
    if (target->mounted) {
        FRESULT close_fr = f_close(&target->file);
        if (close_fr != FR_OK) {
            printf("SD Storage: f_close after %s error failed: %s (%d)\n",
                   op, FRESULT_str(close_fr), close_fr);
        }
    }

    target->mounted = false;
    target->capacity_sectors = 0;
    printf("SD Storage: target %d disabled after %s error: %s (%d)\n",
           target_id, op, FRESULT_str(fr), fr);
}

// Forward declarations
static bool sd_storage_init(void);
static bool sd_storage_mount(uint8_t target_id, const char *image_path, bool read_only);
static bool sd_storage_unmount(uint8_t target_id);
static bool sd_storage_read_sector(uint8_t target_id, uint32_t lba, uint8_t *buffer, size_t len);
static bool sd_storage_write_sector(uint8_t target_id, uint32_t lba, const uint8_t *buffer, size_t len);
static bool sd_storage_sync(uint8_t target_id);
static uint32_t sd_storage_get_capacity(uint8_t target_id);
static bool sd_storage_is_mounted(uint8_t target_id);

// Storage operations table
static const storage_ops_t sd_ops = {
    .init = sd_storage_init,
    .mount = sd_storage_mount,
    .unmount = sd_storage_unmount,
    .read_sector = sd_storage_read_sector,
    .write_sector = sd_storage_write_sector,
    .sync = sd_storage_sync,
    .get_capacity = sd_storage_get_capacity,
    .is_mounted = sd_storage_is_mounted,
};

void sd_storage_register(void) {
    storage_register_backend(STORAGE_BACKEND_SDCARD, &sd_ops);
}

const char* sd_storage_get_image_path(uint8_t target_id) {
    if (!sd_state || target_id >= STORAGE_MAX_TARGETS) {
        return NULL;
    }
    return sd_state->targets[target_id].mounted ? sd_state->targets[target_id].image_path : NULL;
}

// Check if a filename matches disk image patterns
// Matches: *.img files (case insensitive)
static int matches_pattern(const char *filename) {
    size_t len = strlen(filename);
    if (len < 5) return 0;  // Minimum: "x.img"

    // Skip hidden files
    if (filename[0] == '.') return 0;

    // Check for .img extension (case insensitive)
    const char *ext = filename + len - 4;
    if (strcasecmp(ext, ".img") == 0) {
        return 1;
    }
    return 0;
}

// Get the number of discovered disk images
int sd_storage_get_image_count(void) {
    return sd_state ? sd_state->file_count : 0;
}

// Get the filename of a discovered disk image by index
const char* sd_storage_get_image_name(int index) {
    if (!sd_state || index < 0 || index >= sd_state->file_count) {
        return NULL;
    }
    return sd_state->file_names[index];
}

static bool sd_storage_init(void) {
    if (sd_initialized) {
        return true;
    }

    printf("SD Storage: Initializing...\n");

    // Allocate SD state
    sd_state = malloc(sizeof(sd_state_t));
    if (!sd_state) {
        printf("SD Storage: Failed to allocate sd_state\n");
        return false;
    }
    memset(sd_state, 0, sizeof(sd_state_t));

    // Allocate FATFS structure
    sd_state->fs = malloc(sizeof(FATFS));
    if (!sd_state->fs) {
        printf("SD Storage: Failed to allocate FATFS\n");
        free(sd_state);
        sd_state = NULL;
        return false;
    }

    // Mount the filesystem
    // Note: f_mount with opt=1 triggers disk_initialize() internally
    printf("SD Storage: Mounting filesystem...\n");
    fatfs_guard_init();
    fatfs_guard_lock();
    FRESULT fr = f_mount(sd_state->fs, "", 1);
    if (FR_OK != fr) {
        fatfs_guard_unlock();
        printf("SD Storage: f_mount error: %s (%d)\n", FRESULT_str(fr), fr);
        free(sd_state->fs);
        free(sd_state);
        sd_state = NULL;
        return false;
    }
    printf("SD Storage: Filesystem mounted successfully\n");

    // Scan root directory for disk image files
    printf("SD Storage: Scanning for disk images...\n");
    DIR dir;
    fr = f_opendir(&dir, "");
    if (FR_OK != fr) {
        printf("SD Storage: f_opendir error: %s (%d)\n", FRESULT_str(fr), fr);
        // Continue anyway - we can still mount images by explicit path
    } else {
        FILINFO fno;
        sd_state->file_count = 0;

        while (FR_OK == f_readdir(&dir, &fno)) {
            if (fno.fname[0] == 0 || sd_state->file_count >= MAX_IMG_FILES) {
                break;
            }

            if (matches_pattern(fno.fname)) {
                printf("SD Storage: Found disk image: %s\n", fno.fname);
                strncpy(sd_state->file_names[sd_state->file_count], fno.fname,
                        FILENAME_MAX_LENGTH - 1);
                sd_state->file_names[sd_state->file_count][FILENAME_MAX_LENGTH - 1] = '\0';
                sd_state->file_count++;
            }
        }
        f_closedir(&dir);

        printf("SD Storage: Found %d disk image(s)\n", sd_state->file_count);
    }
    fatfs_guard_unlock();

    sd_initialized = true;
    return true;
}

static bool sd_storage_mount(uint8_t target_id, const char *image_path, bool read_only) {
    if (!sd_initialized || !sd_state) {
        printf("SD Storage: not initialized\n");
        return false;
    }

    if (target_id >= STORAGE_MAX_TARGETS) {
        printf("SD Storage: invalid target ID %d\n", target_id);
        return false;
    }

    // Unmount if already mounted
    if (sd_state->targets[target_id].mounted) {
        sd_storage_unmount(target_id);
    }

    // Open the disk image file
    BYTE mode = FA_READ;
    if (!read_only) {
        mode |= FA_WRITE;
    }

    printf("SD Storage: Opening '%s'...\n", image_path);
    fatfs_guard_lock();
    FRESULT fr = f_open(&sd_state->targets[target_id].file, image_path, mode);
    if (FR_OK != fr) {
        fatfs_guard_unlock();
        printf("SD Storage: f_open(%s) error: %s (%d)\n", image_path, FRESULT_str(fr), fr);
        return false;
    }

    // Get file size and calculate capacity in sectors
    FSIZE_t file_size = f_size(&sd_state->targets[target_id].file);
    sd_state->targets[target_id].capacity_sectors = (uint32_t)(file_size / STORAGE_SECTOR_SIZE);

    // Store state
    strncpy(sd_state->targets[target_id].image_path, image_path,
            sizeof(sd_state->targets[target_id].image_path) - 1);
    sd_state->targets[target_id].image_path[sizeof(sd_state->targets[target_id].image_path) - 1] = '\0';
    sd_state->targets[target_id].read_only = read_only;
    sd_state->targets[target_id].mounted = true;

    printf("SD Storage: Mounted '%s' on target %d (%lu sectors, %s)\n",
           image_path, target_id, (unsigned long)sd_state->targets[target_id].capacity_sectors,
           read_only ? "read-only" : "read-write");
    fatfs_guard_unlock();

    return true;
}

static bool sd_storage_unmount(uint8_t target_id) {
    if (!sd_state || target_id >= STORAGE_MAX_TARGETS) {
        return false;
    }

    if (!sd_state->targets[target_id].mounted) {
        return true; // Already unmounted
    }

    fatfs_guard_lock();
    FRESULT fr = f_close(&sd_state->targets[target_id].file);
    if (FR_OK != fr) {
        printf("SD Storage: f_close error: %s (%d)\n", FRESULT_str(fr), fr);
        // Continue unmounting anyway
    }

    sd_state->targets[target_id].mounted = false;
    sd_state->targets[target_id].capacity_sectors = 0;
    printf("SD Storage: Unmounted target %d\n", target_id);
    fatfs_guard_unlock();

    return true;
}

static bool sd_storage_read_sector(uint8_t target_id, uint32_t lba, uint8_t *buffer, size_t len) {
    if (!sd_initialized || !sd_state || target_id >= STORAGE_MAX_TARGETS) {
        return false;
    }

    sd_target_t *target = &sd_state->targets[target_id];
    if (!target->mounted) {
        return false;
    }

    // Validate LBA
    if (lba >= target->capacity_sectors) {
        printf("SD Storage: LBA %lu out of range (max %lu)\n",
               (unsigned long)lba, (unsigned long)target->capacity_sectors);
        return false;
    }

    fatfs_guard_lock();

    // Seek to sector position
    FSIZE_t offset = (FSIZE_t)lba * STORAGE_SECTOR_SIZE;
    FRESULT fr = f_lseek(&target->file, offset);
    if (FR_OK != fr) {
        printf("SD Storage: f_lseek error: %s (%d)\n", FRESULT_str(fr), fr);
        if (sd_storage_is_fatal_error(fr)) {
            sd_storage_disable_target(target_id, "f_lseek(read)", fr);
        }
        fatfs_guard_unlock();
        return false;
    }

    // Read the sector
    UINT bytes_read;
    size_t read_len = (len > 0) ? len : STORAGE_SECTOR_SIZE;
    fr = f_read(&target->file, buffer, read_len, &bytes_read);
    if (FR_OK != fr) {
        printf("SD Storage: f_read error: %s (%d)\n", FRESULT_str(fr), fr);
        if (sd_storage_is_fatal_error(fr)) {
            sd_storage_disable_target(target_id, "f_read", fr);
        }
        fatfs_guard_unlock();
        return false;
    }

    fatfs_guard_unlock();

    if (bytes_read != read_len) {
        // Pad with zeros if short read at end of file
        memset(buffer + bytes_read, 0, read_len - bytes_read);
    }

    sd_printf("SD RD LBA %lu t %d\n",
           (unsigned long)lba, target_id);
    return true;
}

static bool sd_storage_write_sector(uint8_t target_id, uint32_t lba, const uint8_t *buffer, size_t len) {
    if (!sd_initialized || !sd_state || target_id >= STORAGE_MAX_TARGETS) {
        return false;
    }

    sd_target_t *target = &sd_state->targets[target_id];
    if (!target->mounted) {
        return false;
    }

    if (target->read_only) {
        printf("SD Storage: target %d is read-only\n", target_id);
        return false;
    }

    // Validate LBA
    if (lba >= target->capacity_sectors) {
        printf("SD Storage: LBA %lu out of range (max %lu)\n",
               (unsigned long)lba, (unsigned long)target->capacity_sectors);
        return false;
    }

    fatfs_guard_lock();

    // Seek to sector position
    FSIZE_t offset = (FSIZE_t)lba * STORAGE_SECTOR_SIZE;
    FRESULT fr = f_lseek(&target->file, offset);
    if (FR_OK != fr) {
        printf("SD Storage: f_lseek error: %s (%d)\n", FRESULT_str(fr), fr);
        if (sd_storage_is_fatal_error(fr)) {
            sd_storage_disable_target(target_id, "f_lseek(write)", fr);
        }
        fatfs_guard_unlock();
        return false;
    }

    // Write the sector
    UINT bytes_written;
    size_t write_len = (len > 0) ? len : STORAGE_SECTOR_SIZE;
    fr = f_write(&target->file, buffer, write_len, &bytes_written);
    if (FR_OK != fr) {
        printf("SD Storage: f_write error: %s (%d)\n", FRESULT_str(fr), fr);
        if (sd_storage_is_fatal_error(fr)) {
            sd_storage_disable_target(target_id, "f_write", fr);
        }
        fatfs_guard_unlock();
        return false;
    }

    fatfs_guard_unlock();

    if (bytes_written != write_len) {
        printf("SD Storage: short write: %u of %zu bytes\n", bytes_written, write_len);
        return false;
    }

    return true;
}

// Track consecutive sync failures per target.  Transient SDIO errors are
// common (SD card internal GC, busy states) and should not permanently
// disable the target.  Only disable after many consecutive failures which
// indicate a genuinely broken card or corrupted file handle.
#define SD_SYNC_MAX_RETRIES      3
#define SD_SYNC_RETRY_DELAY_MS   5
#define SD_SYNC_DISABLE_THRESHOLD 50

static uint32_t sd_sync_fail_count[STORAGE_MAX_TARGETS];

static bool sd_storage_sync(uint8_t target_id) {
    if (!sd_initialized || !sd_state || target_id >= STORAGE_MAX_TARGETS) {
        return false;
    }

    sd_target_t *target = &sd_state->targets[target_id];
    if (!target->mounted) {
        return false;
    }

    FRESULT fr = FR_DISK_ERR;
    for (int attempt = 0; attempt < SD_SYNC_MAX_RETRIES; attempt++) {
        fatfs_guard_lock();
        fr = f_sync(&target->file);
        fatfs_guard_unlock();
        if (fr == FR_OK) {
            break;
        }
        if (attempt + 1 < SD_SYNC_MAX_RETRIES) {
            sleep_ms(SD_SYNC_RETRY_DELAY_MS);
        }
    }

    if (fr == FR_OK) {
        sd_sync_fail_count[target_id] = 0;
        return true;
    }

    // Sync failed after retries - log but keep target mounted.
    // Transient errors are recoverable; the next write+sync may succeed.
    sd_sync_fail_count[target_id]++;
    if (sd_sync_fail_count[target_id] <= 3 ||
        (sd_sync_fail_count[target_id] % 25) == 0) {
        printf("SD Storage: f_sync error: %s (%d), target %d (count=%lu)\n",
               FRESULT_str(fr), fr, target_id,
               (unsigned long)sd_sync_fail_count[target_id]);
    }

    // Only disable after sustained consecutive failures indicating
    // a genuinely broken card or corrupted file handle.
    if (sd_sync_fail_count[target_id] >= SD_SYNC_DISABLE_THRESHOLD) {
        sd_storage_disable_target(target_id, "f_sync(sustained)", fr);
    }

    return false;
}

static uint32_t sd_storage_get_capacity(uint8_t target_id) {
    if (!sd_state || target_id >= STORAGE_MAX_TARGETS || !sd_state->targets[target_id].mounted) {
        return 0;
    }
    return sd_state->targets[target_id].capacity_sectors;
}

static bool sd_storage_is_mounted(uint8_t target_id) {
    if (!sd_state || target_id >= STORAGE_MAX_TARGETS) {
        return false;
    }
    return sd_state->targets[target_id].mounted;
}
