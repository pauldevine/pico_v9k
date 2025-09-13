#include "pico/multicore.h"
#include "pico/util/queue.h"
#include "pico/stdlib.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#define LOG_BUFFER_SIZE 1024
queue_t log_queue;

// Non-blocking log function for your timing-critical code
void fast_log(const char* format, ...) {
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    // Just drop the message if queue is full (non-blocking)
    queue_try_add(&log_queue, buffer);
}
