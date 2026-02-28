#ifndef FATFS_GUARD_H
#define FATFS_GUARD_H

#include <stdbool.h>

// Initialize shared FatFS lock.
// Safe to call multiple times.
void fatfs_guard_init(void);

// Acquire/release shared lock for FatFS API calls.
void fatfs_guard_lock(void);
void fatfs_guard_unlock(void);

// Try to acquire shared lock without blocking.
bool fatfs_guard_try_lock(void);

#endif  // FATFS_GUARD_H
