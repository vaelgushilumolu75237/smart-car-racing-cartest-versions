#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "../overlay/ports/stm32/jpeg_preview_guard_cleanup.h"

typedef struct mock_context {
    bool abort_succeeds;
    char order[4];
    unsigned order_len;
    unsigned mock_uma_frees;
} mock_context_t;

static bool mock_hal_jpeg_abort(void *opaque) {
    mock_context_t *mock = opaque;
    mock->order[mock->order_len++] = 'A';
    return mock->abort_succeeds;
}

static void mock_hal_unregister_callbacks(void *opaque) {
    mock_context_t *mock = opaque;
    mock->order[mock->order_len++] = 'U';
}

static void mock_uma_release_owned_buffers(void *opaque) {
    mock_context_t *mock = opaque;
    mock->order[mock->order_len++] = 'F';
    mock->mock_uma_frees += 2;
}

int main(void) {
    assert(jpeg_preview_guard_timeout_valid(1, 1000));
    assert(jpeg_preview_guard_timeout_valid(1000, 1000));
    assert(!jpeg_preview_guard_timeout_valid(0, 1000));
    assert(!jpeg_preview_guard_timeout_valid(1001, 1000));

    mock_context_t success = { .abort_succeeds = true };
    bool result = jpeg_preview_guard_cleanup(&success, mock_hal_jpeg_abort,
                                            mock_hal_unregister_callbacks,
                                            mock_uma_release_owned_buffers);
    assert(result);
    assert(success.order_len == 3);
    assert(memcmp(success.order, "AUF", 3) == 0);
    assert(success.mock_uma_frees == 2);

    mock_context_t abort_failure = { .abort_succeeds = false };
    result = jpeg_preview_guard_cleanup(&abort_failure, mock_hal_jpeg_abort,
                                       mock_hal_unregister_callbacks,
                                       mock_uma_release_owned_buffers);
    assert(!result);
    assert(abort_failure.order_len == 2);
    assert(memcmp(abort_failure.order, "AU", 2) == 0);
    assert(abort_failure.mock_uma_frees == 0);
    return 0;
}
