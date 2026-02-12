/* storage.c
 * Storage abstraction layer implementation
 * Dispatches storage operations to the active backend
 */

#include <stdio.h>
#include <string.h>
#include "storage.h"

// Storage backend registry
static const storage_ops_t *backend_ops[3] = {NULL, NULL, NULL};
static storage_backend_t active_backend = STORAGE_BACKEND_NONE;
static bool initialized = false;

void storage_register_backend(storage_backend_t type, const storage_ops_t *ops) {
    if (type > STORAGE_BACKEND_NONE && type <= STORAGE_BACKEND_SDCARD) {
        backend_ops[type] = ops;
    }
}

bool storage_init(storage_backend_t backend) {
    if (backend == STORAGE_BACKEND_NONE) {
        printf("Storage: no backend specified\n");
        return false;
    }

    const storage_ops_t *ops = backend_ops[backend];
    if (!ops) {
        printf("Storage: backend %d not registered\n", backend);
        return false;
    }

    if (!ops->init) {
        printf("Storage: backend %d has no init function\n", backend);
        return false;
    }

    if (ops->init()) {
        active_backend = backend;
        initialized = true;
        printf("Storage: initialized backend %d\n", backend);
        return true;
    }

    printf("Storage: failed to initialize backend %d\n", backend);
    return false;
}

storage_backend_t storage_get_backend(void) {
    return active_backend;
}

bool storage_mount(uint8_t target_id, const char *image_path, bool read_only) {
    if (!initialized || active_backend == STORAGE_BACKEND_NONE) {
        return false;
    }

    const storage_ops_t *ops = backend_ops[active_backend];
    if (!ops || !ops->mount) {
        return false;
    }

    return ops->mount(target_id, image_path, read_only);
}

bool storage_unmount(uint8_t target_id) {
    if (!initialized || active_backend == STORAGE_BACKEND_NONE) {
        return false;
    }

    const storage_ops_t *ops = backend_ops[active_backend];
    if (!ops || !ops->unmount) {
        return false;
    }

    return ops->unmount(target_id);
}

bool storage_read_sector(uint8_t target_id, uint32_t lba, uint8_t *buffer, size_t len) {
    if (!initialized || active_backend == STORAGE_BACKEND_NONE) {
        return false;
    }

    const storage_ops_t *ops = backend_ops[active_backend];
    if (!ops || !ops->read_sector) {
        return false;
    }

    return ops->read_sector(target_id, lba, buffer, len);
}

bool storage_write_sector(uint8_t target_id, uint32_t lba, const uint8_t *buffer, size_t len) {
    if (!initialized || active_backend == STORAGE_BACKEND_NONE) {
        return false;
    }

    const storage_ops_t *ops = backend_ops[active_backend];
    if (!ops || !ops->write_sector) {
        return false;
    }

    return ops->write_sector(target_id, lba, buffer, len);
}

bool storage_sync(uint8_t target_id) {
    if (!initialized || active_backend == STORAGE_BACKEND_NONE) {
        return false;
    }

    const storage_ops_t *ops = backend_ops[active_backend];
    if (!ops || !ops->sync) {
        return true;  // No sync needed for this backend
    }

    return ops->sync(target_id);
}

uint32_t storage_get_capacity(uint8_t target_id) {
    if (!initialized || active_backend == STORAGE_BACKEND_NONE) {
        return 0;
    }

    const storage_ops_t *ops = backend_ops[active_backend];
    if (!ops || !ops->get_capacity) {
        return 0;
    }

    return ops->get_capacity(target_id);
}

bool storage_is_mounted(uint8_t target_id) {
    if (!initialized || active_backend == STORAGE_BACKEND_NONE) {
        return false;
    }

    const storage_ops_t *ops = backend_ops[active_backend];
    if (!ops || !ops->is_mounted) {
        return false;
    }

    return ops->is_mounted(target_id);
}
