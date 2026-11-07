#ifndef CARTEST_RUNTIME_DIAG_H
#define CARTEST_RUNTIME_DIAG_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
    CAR_RUNTIME_DIAG_MAGIC = 0x47414944u, /* Little-endian bytes "DIAG". */
    CAR_RUNTIME_DIAG_VERSION = 1u,
    CAR_RUNTIME_DIAG_SIZE = 64u,
    CAR_RUNTIME_DIAG_PERIOD_MS = 100u,
    CAR_RUNTIME_DIAG_FLAG_LOG_ENABLED = 1u << 0
};

#if defined(_MSC_VER)
#pragma pack(push, 1)
#define CAR_RUNTIME_DIAG_PACKED
#else
#define CAR_RUNTIME_DIAG_PACKED __attribute__((packed))
#endif

/* Little-endian Hot Plug ABI. The runtime log's independent 64-byte V1/V2
   header and records are not changed by this diagnostic snapshot. */
typedef struct CAR_RUNTIME_DIAG_PACKED {
    uint32_t magic;                   /* 0: CAR_RUNTIME_DIAG_MAGIC */
    uint16_t version;                 /* 4: CAR_RUNTIME_DIAG_VERSION */
    uint16_t size;                    /* 6: CAR_RUNTIME_DIAG_SIZE */
    uint32_t generation;              /* 8: even stable, odd writer active */
    uint32_t crc32;                   /* 12: CRC-32/ISO-HDLC, this field zero */
    uint32_t start_events;            /* 16: physical PE7 accepted starts */
    uint32_t stop_events;             /* 20: physical PE7 stop generation */
    uint32_t log_update_calls;        /* 24: actual logger API calls */
    uint32_t start_token;             /* 28: last PE7 token consumed by logger */
    uint32_t cpu_header_state;        /* 32: IDLE=0, ACTIVE=1, FROZEN=2 */
    uint32_t cpu_header_count;        /* 36: committed runtime-log records */
    uint32_t uptime_ms;               /* 40: HAL_GetTick() modulo 2^32 */
    uint32_t reset_flags;             /* 44: captured H743 RCC->RSR */
    int16_t left_output_permille;     /* 48: software-applied Follow output */
    int16_t right_output_permille;    /* 50: software-applied Follow output */
    uint16_t servo_pulse_us;          /* 52: applied software PWM compare */
    uint16_t flags;                   /* 54: bit 0 log enabled; rest zero */
    uint32_t header_session_id;       /* 56: mirrors runtime-log header */
    uint32_t reserved;                /* 60: zero in version 1 */
} car_runtime_diag_snapshot_t;

#if defined(_MSC_VER)
#pragma pack(pop)
#endif
#undef CAR_RUNTIME_DIAG_PACKED

/* Input to the bounded publisher; this is not part of the binary ABI. */
typedef struct {
    uint32_t start_events;
    uint32_t stop_events;
    uint32_t start_token;
    uint32_t cpu_header_state;
    uint32_t cpu_header_count;
    uint32_t uptime_ms;
    uint32_t reset_flags;
    uint32_t header_session_id;
    uint16_t left_output_permille;
    uint16_t right_output_permille;
    uint16_t servo_pulse_us;
    uint8_t log_enabled;
} car_runtime_diag_values_t;

extern car_runtime_diag_snapshot_t g_runtime_diag_snapshot;

/* Initializes generation and publishes a valid zero-output boot record. */
void car_runtime_diag_init(uint32_t reset_flags, uint8_t log_enabled);

/* Called immediately before every real car_runtime_log_update() invocation,
   including invocations made while the CPU header is IDLE. */
void car_runtime_diag_note_log_update_call(void);

/* Publish on state/count/session or PE7 event changes and at most every
   CAR_RUNTIME_DIAG_PERIOD_MS while otherwise idle. Returns 1 when published. */
uint8_t car_runtime_diag_update(const car_runtime_diag_values_t *values);

uint32_t car_runtime_diag_crc32(const void *bytes, uint32_t length);
uint32_t car_runtime_diag_snapshot_crc32(
    const car_runtime_diag_snapshot_t *snapshot);
uint32_t car_runtime_diag_next_even_generation(uint32_t stable_generation);

#if defined(__cplusplus)
static_assert(sizeof(car_runtime_diag_snapshot_t) ==
                  CAR_RUNTIME_DIAG_SIZE,
              "runtime diagnostic ABI must remain 64 bytes");
static_assert(offsetof(car_runtime_diag_snapshot_t, generation) == 8u &&
                  offsetof(car_runtime_diag_snapshot_t, crc32) == 12u &&
                  offsetof(car_runtime_diag_snapshot_t, start_events) == 16u &&
                  offsetof(car_runtime_diag_snapshot_t, stop_events) == 20u &&
                  offsetof(car_runtime_diag_snapshot_t, log_update_calls) == 24u &&
                  offsetof(car_runtime_diag_snapshot_t, start_token) == 28u &&
                  offsetof(car_runtime_diag_snapshot_t, cpu_header_state) == 32u &&
                  offsetof(car_runtime_diag_snapshot_t, cpu_header_count) == 36u &&
                  offsetof(car_runtime_diag_snapshot_t, uptime_ms) == 40u &&
                  offsetof(car_runtime_diag_snapshot_t, reset_flags) == 44u &&
                  offsetof(car_runtime_diag_snapshot_t, left_output_permille) == 48u &&
                  offsetof(car_runtime_diag_snapshot_t, right_output_permille) == 50u &&
                  offsetof(car_runtime_diag_snapshot_t, servo_pulse_us) == 52u &&
                  offsetof(car_runtime_diag_snapshot_t, flags) == 54u &&
                  offsetof(car_runtime_diag_snapshot_t, header_session_id) == 56u &&
                  offsetof(car_runtime_diag_snapshot_t, reserved) == 60u,
              "runtime diagnostic ABI field offsets changed");
#else
_Static_assert(sizeof(car_runtime_diag_snapshot_t) ==
                   CAR_RUNTIME_DIAG_SIZE,
               "runtime diagnostic ABI must remain 64 bytes");
_Static_assert(offsetof(car_runtime_diag_snapshot_t, generation) == 8u &&
                   offsetof(car_runtime_diag_snapshot_t, crc32) == 12u &&
                   offsetof(car_runtime_diag_snapshot_t, start_events) == 16u &&
                   offsetof(car_runtime_diag_snapshot_t, stop_events) == 20u &&
                   offsetof(car_runtime_diag_snapshot_t, log_update_calls) == 24u &&
                   offsetof(car_runtime_diag_snapshot_t, start_token) == 28u &&
                   offsetof(car_runtime_diag_snapshot_t, cpu_header_state) == 32u &&
                   offsetof(car_runtime_diag_snapshot_t, cpu_header_count) == 36u &&
                   offsetof(car_runtime_diag_snapshot_t, uptime_ms) == 40u &&
                   offsetof(car_runtime_diag_snapshot_t, reset_flags) == 44u &&
                   offsetof(car_runtime_diag_snapshot_t, left_output_permille) == 48u &&
                   offsetof(car_runtime_diag_snapshot_t, right_output_permille) == 50u &&
                   offsetof(car_runtime_diag_snapshot_t, servo_pulse_us) == 52u &&
                   offsetof(car_runtime_diag_snapshot_t, flags) == 54u &&
                   offsetof(car_runtime_diag_snapshot_t, header_session_id) == 56u &&
                   offsetof(car_runtime_diag_snapshot_t, reserved) == 60u,
               "runtime diagnostic ABI field offsets changed");
#endif

#ifdef __cplusplus
}
#endif

#endif /* CARTEST_RUNTIME_DIAG_H */
