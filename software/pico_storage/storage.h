/* storage.h
 * Storage abstraction layer for Victor 9000 DMA board firmware
 * Provides a unified interface for different storage backends (FujiNet, SD card, etc.)
 */
#ifndef STORAGE_H
#define STORAGE_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// Storage backend types
typedef enum {
    STORAGE_BACKEND_NONE = 0,
    STORAGE_BACKEND_FUJINET,
    STORAGE_BACKEND_SDCARD,
} storage_backend_t;

// Storage operation result codes
typedef enum {
    STORAGE_OK = 0,
    STORAGE_ERR_NOT_INITIALIZED,
    STORAGE_ERR_NOT_MOUNTED,
    STORAGE_ERR_READ_FAILED,
    STORAGE_ERR_WRITE_FAILED,
    STORAGE_ERR_INVALID_SECTOR,
    STORAGE_ERR_FILE_NOT_FOUND,
    STORAGE_ERR_NO_BACKEND,
} storage_result_t;

// Maximum number of virtual disk targets (SASI allows 0-7)
#define STORAGE_MAX_TARGETS 8

// Sector size in bytes
#define STORAGE_SECTOR_SIZE 512

// Storage interface function pointers
typedef struct storage_ops {
    // Initialize the storage backend
    bool (*init)(void);

    // Mount a disk image for the given target ID
    // image_path is backend-specific (FujiNet slot, SD card file path, etc.)
    bool (*mount)(uint8_t target_id, const char *image_path, bool read_only);

    // Unmount the disk image for the given target ID
    bool (*unmount)(uint8_t target_id);

    // Read a sector from the given target
    bool (*read_sector)(uint8_t target_id, uint32_t lba, uint8_t *buffer, size_t len);

    // Write a sector to the given target
    bool (*write_sector)(uint8_t target_id, uint32_t lba, const uint8_t *buffer, size_t len);

    // Get disk capacity in sectors for the given target
    uint32_t (*get_capacity)(uint8_t target_id);

    // Check if target is mounted
    bool (*is_mounted)(uint8_t target_id);
} storage_ops_t;

// Initialize the storage system with the specified backend
bool storage_init(storage_backend_t backend);

// Get the current active backend type
storage_backend_t storage_get_backend(void);

// Mount a disk image for a target
bool storage_mount(uint8_t target_id, const char *image_path, bool read_only);

// Unmount a disk image
bool storage_unmount(uint8_t target_id);

// Read a sector (512 bytes by default)
bool storage_read_sector(uint8_t target_id, uint32_t lba, uint8_t *buffer, size_t len);

// Write a sector (512 bytes by default)
bool storage_write_sector(uint8_t target_id, uint32_t lba, const uint8_t *buffer, size_t len);

// Get disk capacity in sectors
uint32_t storage_get_capacity(uint8_t target_id);

// Check if a target is mounted
bool storage_is_mounted(uint8_t target_id);

// Register a storage backend (called by backend implementations)
void storage_register_backend(storage_backend_t type, const storage_ops_t *ops);

#ifdef __cplusplus
}
#endif

#endif // STORAGE_H
