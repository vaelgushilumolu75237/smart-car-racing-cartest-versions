#ifndef CARTEST_RUNTIME_LOG_H
#define CARTEST_RUNTIME_LOG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
    CAR_RUNTIME_LOG_VERSION = 1u,
    CAR_RUNTIME_LOG_HEADER_SIZE = 64u,
    CAR_RUNTIME_LOG_RECORD_SIZE = 64u,
    CAR_RUNTIME_LOG_CAPACITY = 6144u,
    CAR_RUNTIME_LOG_PERIOD_MS = 50u,
    CAR_RUNTIME_LOG_ALIGNMENT = 32u,
    CAR_RUNTIME_LOG_BUFFER_SIZE = CAR_RUNTIME_LOG_HEADER_SIZE +
                                  CAR_RUNTIME_LOG_RECORD_SIZE *
                                      CAR_RUNTIME_LOG_CAPACITY
};

typedef enum {
    CAR_RUNTIME_LOG_STATE_IDLE = 0u,
    CAR_RUNTIME_LOG_STATE_ACTIVE = 1u,
    CAR_RUNTIME_LOG_STATE_FROZEN = 2u
} car_runtime_log_state_t;

typedef enum {
    CAR_RUNTIME_LOG_SESSION_NONE = 0u,
    CAR_RUNTIME_LOG_SESSION_FOLLOW = 1u,
    CAR_RUNTIME_LOG_SESSION_BENCH = 2u
} car_runtime_log_session_kind_t;

typedef enum {
    CAR_RUNTIME_LOG_STOP_NONE = 0u,
    CAR_RUNTIME_LOG_STOP_MANUAL_RELEASE = 1u,
    CAR_RUNTIME_LOG_STOP_HARD_FAULT = 2u,
    CAR_RUNTIME_LOG_STOP_BENCH_COMPLETE = 3u,
    CAR_RUNTIME_LOG_STOP_SESSION_ENDED = 4u
} car_runtime_log_stop_reason_t;

enum {
    CAR_RUNTIME_LOG_FLAG_FULL = 1u << 0,
    CAR_RUNTIME_LOG_FLAG_TRUNCATED = 1u << 1
};

enum {
    CAR_RUNTIME_LOG_EVENT_PERIODIC = 1u << 0,
    CAR_RUNTIME_LOG_EVENT_START = 1u << 1,
    CAR_RUNTIME_LOG_EVENT_STOP = 1u << 2,
    CAR_RUNTIME_LOG_EVENT_FAULT = 1u << 3,
    CAR_RUNTIME_LOG_EVENT_MODE = 1u << 4,
    CAR_RUNTIME_LOG_EVENT_RECOVERY = 1u << 5,
    CAR_RUNTIME_LOG_EVENT_AUTO_RECOVERY = 1u << 6,
    CAR_RUNTIME_LOG_EVENT_HOLD_ENTER = 1u << 7,
    CAR_RUNTIME_LOG_EVENT_HOLD_EXIT = 1u << 8,
    CAR_RUNTIME_LOG_EVENT_HOLD_DEADLINE = 1u << 9,
    CAR_RUNTIME_LOG_EVENT_BUTTON_START = 1u << 10,
    CAR_RUNTIME_LOG_EVENT_BUTTON_STOP = 1u << 11,
    CAR_RUNTIME_LOG_EVENT_HOLD_CAUSE_SHIFT = 12u,
    CAR_RUNTIME_LOG_EVENT_HOLD_CAUSE_MASK = 7u << 12
};

enum {
    CAR_RUNTIME_LOG_VISION_LATEST_VALID = 1u << 0,
    CAR_RUNTIME_LOG_VISION_USABLE_FRESH = 1u << 1,
    CAR_RUNTIME_LOG_VISION_TIMEOUT = 1u << 2,
    CAR_RUNTIME_LOG_VISION_LATEST_FRAME_PRESENT = 1u << 3,
    CAR_RUNTIME_LOG_VISION_HOLD_ACTIVE = 1u << 4,
    CAR_RUNTIME_LOG_VISION_HOLD_EXPIRED = 1u << 5
};

#if defined(_MSC_VER)
#pragma pack(push, 1)
#define CAR_RUNTIME_LOG_PACKED
#else
#define CAR_RUNTIME_LOG_PACKED __attribute__((packed))
#endif

/* Little-endian RAM ABI. Keep field order and size fixed at 64 bytes. */
typedef struct CAR_RUNTIME_LOG_PACKED {
    uint32_t time_ms;                         /* 0 */
    uint32_t session_id;                      /* 4 */
    int64_t left_encoder_counts;              /* 8 */
    int64_t right_encoder_counts;             /* 16 */
    uint32_t fault_generation;                /* 24 */
    uint16_t servo_requested_us;              /* 28 */
    uint16_t servo_limited_us;                /* 30 */
    uint16_t servo_applied_us;                /* 32, software PWM snapshot */
    uint16_t motor_requested_permille;        /* 34 */
    uint16_t left_applied_permille;            /* 36, software PWM snapshot */
    uint16_t right_applied_permille;           /* 38, software PWM snapshot */
    uint16_t motor_ccr1_counts;                /* 40 */
    uint16_t motor_ccr2_counts;                /* 42 */
    uint16_t motor_ccr3_counts;                /* 44 */
    uint16_t motor_ccr4_counts;                /* 46 */
    int16_t vision_error_norm;                 /* 48, normalized / 1000 */
    uint16_t vision_sequence;                  /* 50 */
    uint32_t auto_recovery_count;              /* 52 */
    uint16_t event_flags;                      /* 56 */
    uint8_t mode;                              /* 58 */
    uint8_t fault;                             /* 59 */
    uint8_t vision_confidence;                 /* 60, 0..100 */
    uint8_t vision_flags;                      /* 61, CAR_RUNTIME_LOG_VISION_* */
    uint8_t recovery_frame_count;              /* 62 */
    uint8_t recovery_active;                   /* 63 */
} car_runtime_log_record_t;

/* Header CRC is CRC-32/ISO-HDLC over all 64 bytes with header_crc32 zeroed. */
typedef struct CAR_RUNTIME_LOG_PACKED {
    uint32_t magic;                            /* 0: bytes "RLOG" */
    uint16_t version;                          /* 4 */
    uint16_t header_size;                      /* 6 */
    uint16_t record_size;                      /* 8 */
    uint16_t capacity;                         /* 10 */
    uint16_t period_ms;                        /* 12 */
    uint8_t state;                             /* 14 */
    uint8_t flags;                             /* 15 */
    uint32_t count;                            /* 16, published last */
    uint32_t session_id;                       /* 20 */
    uint32_t start_ms;                         /* 24 */
    uint32_t stop_ms;                          /* 28 */
    uint32_t dropped_count;                    /* 32 */
    uint32_t records_crc32;                    /* 36, CRC over committed records */
    uint32_t header_crc32;                     /* 40 */
    uint8_t build_id[16];                      /* 44 */
    uint8_t stop_reason;                       /* 60 */
    uint8_t reserved[3];                       /* 61 */
} car_runtime_log_header_t;

/* ROM descriptor is read from the selected ELF; buffer address is its symbol. */
typedef struct CAR_RUNTIME_LOG_PACKED {
    uint32_t magic;                            /* 0: bytes "RLDS" */
    uint16_t descriptor_version;               /* 4 */
    uint16_t descriptor_size;                  /* 6 */
    uint16_t log_version;                      /* 8 */
    uint16_t header_size;                      /* 10 */
    uint16_t record_size;                      /* 12 */
    uint16_t capacity;                         /* 14 */
    uint16_t period_ms;                        /* 16 */
    uint16_t alignment;                        /* 18 */
    uint32_t buffer_size;                      /* 20 */
    uint32_t endian_tag;                       /* 24: 0x12345678 */
    uint8_t build_id[16];                      /* 28 */
    uint32_t reserved[5];                      /* 44, must be zero */
} car_runtime_log_descriptor_t;

typedef struct {
    uint32_t now_ms;
    int64_t left_encoder_counts;
    int64_t right_encoder_counts;
    uint32_t fault_generation;
    uint32_t auto_recovery_count;
    uint16_t servo_requested_us;
    uint16_t servo_limited_us;
    uint16_t servo_applied_us;
    uint16_t motor_requested_permille;
    uint16_t left_applied_permille;
    uint16_t right_applied_permille;
    uint16_t motor_ccr1_counts;
    uint16_t motor_ccr2_counts;
    uint16_t motor_ccr3_counts;
    uint16_t motor_ccr4_counts;
    int16_t vision_error_norm;
    uint16_t vision_sequence;
    uint8_t mode;
    uint8_t fault;
    uint8_t vision_confidence;
    uint8_t vision_flags;
    uint8_t recovery_frame_count;
    uint8_t recovery_active;
    uint8_t follow_session_active;
    uint8_t hard_fault_latched;
    uint8_t manual_run_request;
    uint8_t vision_hold_active;
    uint8_t vision_hold_expired;
    uint8_t vision_hold_exit_cause;
    uint16_t vision_hold_elapsed_ms;
    uint8_t vision_hold_valid_count;
    uint16_t application_event_flags;
} car_runtime_log_sample_t;

typedef struct CAR_RUNTIME_LOG_PACKED {
    car_runtime_log_header_t header;
    car_runtime_log_record_t records[CAR_RUNTIME_LOG_CAPACITY];
} car_runtime_log_storage_t;

#if defined(_MSC_VER)
#pragma pack(pop)
#endif
#undef CAR_RUNTIME_LOG_PACKED

extern car_runtime_log_storage_t car_runtime_log_storage;
extern const car_runtime_log_descriptor_t car_runtime_log_rom_descriptor;

void car_runtime_log_init(void);
/* Invalidates only the fixed-size header; the record area is left untouched. */
void car_runtime_log_invalidate(void);
/* start_kind is a one-loop edge captured after car_control_tick proves that
   the manual Follow or Bench session started. It is never inferred from vision. */
void car_runtime_log_update(const car_runtime_log_sample_t *sample,
                            car_runtime_log_session_kind_t start_kind);
uint8_t car_runtime_log_state(void);
uint32_t car_runtime_log_crc32(const void *bytes, uint32_t length);

#ifdef __cplusplus
}
#endif

#endif /* CARTEST_RUNTIME_LOG_H */
