#ifndef JPEG_PREVIEW_GUARD_CLEANUP_H
#define JPEG_PREVIEW_GUARD_CLEANUP_H

#include <stdbool.h>
#include <stdint.h>

typedef bool (*jpeg_preview_guard_abort_fn_t)(void *context);
typedef void (*jpeg_preview_guard_action_fn_t)(void *context);

static inline bool jpeg_preview_guard_timeout_valid(uint32_t timeout_ms, uint32_t maximum_ms) {
    return (timeout_ms >= 1U) && (timeout_ms <= maximum_ms);
}

/* Never release DMA-referenced memory until the abort callback confirms stop. */
static inline bool jpeg_preview_guard_cleanup(void *context,
                                             jpeg_preview_guard_abort_fn_t abort_fn,
                                             jpeg_preview_guard_action_fn_t unbind_fn,
                                             jpeg_preview_guard_action_fn_t release_fn) {
    bool stopped = abort_fn(context);
    unbind_fn(context);

    if (!stopped) {
        return false;
    }

    release_fn(context);
    return true;
}

#endif
