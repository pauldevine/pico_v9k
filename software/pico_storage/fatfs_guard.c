#include "pico/mutex.h"
#include "pico/sync.h"

#include "fatfs_guard.h"

static mutex_t fatfs_mutex;
static bool fatfs_mutex_initialized = false;
static critical_section_t fatfs_guard_init_cs;
static bool fatfs_guard_init_cs_initialized = false;

static void fatfs_guard_init_once(void) {
    if (!fatfs_guard_init_cs_initialized) {
        critical_section_init(&fatfs_guard_init_cs);
        fatfs_guard_init_cs_initialized = true;
    }

    critical_section_enter_blocking(&fatfs_guard_init_cs);
    if (!fatfs_mutex_initialized) {
        mutex_init(&fatfs_mutex);
        fatfs_mutex_initialized = true;
    }
    critical_section_exit(&fatfs_guard_init_cs);
}

void fatfs_guard_init(void) {
    fatfs_guard_init_once();
}

void fatfs_guard_lock(void) {
    fatfs_guard_init_once();
    mutex_enter_blocking(&fatfs_mutex);
}

bool fatfs_guard_try_lock(void) {
    fatfs_guard_init_once();
    return mutex_try_enter(&fatfs_mutex, NULL);
}

void fatfs_guard_unlock(void) {
    if (!fatfs_mutex_initialized) {
        return;
    }
    mutex_exit(&fatfs_mutex);
}
