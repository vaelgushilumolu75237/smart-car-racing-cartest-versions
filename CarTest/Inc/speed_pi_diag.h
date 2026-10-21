#ifndef CARTEST_SPEED_PI_DIAG_H
#define CARTEST_SPEED_PI_DIAG_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
    CAR_SPEED_PI_DIAG_MAGIC = 0x47414950u, /* Little-endian bytes "PIAG". */
    CAR_SPEED_PI_DIAG_VERSION = 2u,
    CAR_SPEED_PI_DIAG_SIZE = 128u,
    CAR_SPEED_PI_DIAG_V1_OUTPUT_CAP_PERMILLE = 700u,
    CAR_SPEED_PI_DIAG_OUTPUT_CAP_PERMILLE = 1000u,
    CAR_SPEED_PI_DIAG_TYPE_NONE = 0u,
    CAR_SPEED_PI_DIAG_TYPE_POSITION = 1u,
    CAR_SPEED_PI_DIAG_TYPE_INCREMENTAL = 2u,
    CAR_SPEED_PI_DIAG_FLAG_ENABLED = 1u << 0,
    CAR_SPEED_PI_DIAG_FLAG_ARMED = 1u << 1,
    CAR_SPEED_PI_DIAG_FLAG_ACTIVE = 1u << 2,
    CAR_SPEED_PI_DIAG_FLAG_LAST_SAMPLE_VALID = 1u << 3,
    CAR_SPEED_PI_DIAG_FLAG_FROZEN = 1u << 4,
    CAR_SPEED_PI_DIAG_FLAG_HOLD_ACTIVE = 1u << 5
};

#if defined(_MSC_VER)
#pragma pack(push, 1)
#define CAR_SPEED_PI_DIAG_PACKED
#else
#define CAR_SPEED_PI_DIAG_PACKED __attribute__((packed))
#endif

/* Little-endian, 128-byte RAM ABI. The global snapshot is 32-byte aligned
   and resides in the NOLOAD .speed_pi_diag linker section. */
typedef struct CAR_SPEED_PI_DIAG_PACKED {
    uint32_t magic;                  /* 0: CAR_SPEED_PI_DIAG_MAGIC */
    uint16_t version;                /* 4 */
    uint16_t size;                   /* 6: CAR_SPEED_PI_DIAG_SIZE */
    uint32_t generation;             /* 8: even stable, odd writer active */
    uint32_t crc32;                  /* 12: CRC-32/ISO-HDLC, this field zero */
    uint32_t start_token;            /* 16: PE7 manual-start token */
    uint32_t start_ms;               /* 20: session start tick */
    uint32_t snapshot_ms;            /* 24: most recent publication tick */
    uint32_t sample_ms;              /* 28: last valid observer sample tick */
    int32_t target_left_millirps;    /* 32 */
    int32_t target_right_millirps;   /* 36 */
    int32_t measured_left_millirps;  /* 40: last valid measured speed */
    int32_t measured_right_millirps; /* 44: last valid measured speed */
    uint16_t nominal_ff_left_permille;  /* 48 */
    uint16_t nominal_ff_right_permille; /* 50 */
    uint16_t requested_left_permille;   /* 52: last valid PI proposal */
    uint16_t requested_right_permille;  /* 54 */
    uint16_t applied_left_permille;     /* 56: current software-applied output */
    uint16_t applied_right_permille;    /* 58 */
    uint32_t status;                 /* 60: car_follow_speed_pi status value */
    uint32_t termination_fault;      /* 64: car fault value at session stop */
    uint32_t saturation_left_ms;     /* 68: time at versioned upper cap */
    uint32_t saturation_right_ms;    /* 72: same; 0-output does not count */
    uint32_t flags;                  /* 76: enabled/armed/active/sample/frozen/hold */
    uint32_t pi_type;                /* 80: CAR_SPEED_PI_DIAG_TYPE_* */
    uint32_t reserved[11];           /* 84: cap; 88..127 zero in version 2 */
} car_speed_pi_diag_snapshot_t;

#if defined(_MSC_VER)
#pragma pack(pop)
#endif
#undef CAR_SPEED_PI_DIAG_PACKED

/* Input to the publisher; this is not part of the binary snapshot ABI. */
typedef struct {
    uint32_t now_ms;
    uint32_t sample_ms;
    int32_t target_left_millirps;
    int32_t target_right_millirps;
    int32_t measured_left_millirps;
    int32_t measured_right_millirps;
    uint16_t nominal_ff_left_permille;
    uint16_t nominal_ff_right_permille;
    uint16_t requested_left_permille;
    uint16_t requested_right_permille;
    uint16_t applied_left_permille;
    uint16_t applied_right_permille;
    uint32_t status;
    uint32_t sample_elapsed_ms;
    uint8_t enabled;
    uint8_t armed;
    uint8_t new_speed_sample;
    uint8_t speed_sample_valid;
    uint8_t saturated_left;
    uint8_t saturated_right;
    uint8_t hold_active;
    uint8_t pi_proposal_valid;
} car_speed_pi_diag_values_t;

extern car_speed_pi_diag_snapshot_t g_speed_pi_diag_snapshot;

/* Publish an IDLE record after reset so an unstarted RAM region is detectable. */
void car_speed_pi_diag_init(void);

/* Start a new C Follow PI session and clear all previous-session values. */
void car_speed_pi_diag_begin(uint32_t start_token, uint32_t start_ms,
                             uint32_t pi_type);

/* Publish one controller update. A valid new sample replaces measured speed.
   Target, feed-forward, and PI request fields change only for a valid PI
   proposal while enabled, armed, and outside hold. Invalid samples and
   controller reset preserve those last-valid values; applied output/status
   and enabled/armed state are refreshed on every call. Saturation time adds
   sample_elapsed_ms only once per distinct sample_ms when the sample is valid,
   the PI proposal is valid, armed, and not held, the side is saturated, and
   its request reaches the cap recorded in reserved[0]. */
uint8_t car_speed_pi_diag_update(const car_speed_pi_diag_values_t *values);

/* Freeze the record before resetting PI state or changing session state. */
void car_speed_pi_diag_freeze(uint32_t now_ms, uint32_t termination_fault);

uint32_t car_speed_pi_diag_crc32(const void *bytes, uint32_t length);
uint32_t car_speed_pi_diag_snapshot_crc32(
    const car_speed_pi_diag_snapshot_t *snapshot);
uint32_t car_speed_pi_diag_next_even_generation(uint32_t stable_generation);

#if defined(__cplusplus)
static_assert(sizeof(car_speed_pi_diag_snapshot_t) == CAR_SPEED_PI_DIAG_SIZE,
              "speed PI diagnostic ABI must remain 128 bytes");
static_assert(offsetof(car_speed_pi_diag_snapshot_t, generation) == 8u &&
                  offsetof(car_speed_pi_diag_snapshot_t, crc32) == 12u &&
                  offsetof(car_speed_pi_diag_snapshot_t, start_token) == 16u &&
                  offsetof(car_speed_pi_diag_snapshot_t, target_left_millirps) == 32u &&
                  offsetof(car_speed_pi_diag_snapshot_t, measured_left_millirps) == 40u &&
                  offsetof(car_speed_pi_diag_snapshot_t, nominal_ff_left_permille) == 48u &&
                  offsetof(car_speed_pi_diag_snapshot_t, requested_left_permille) == 52u &&
                  offsetof(car_speed_pi_diag_snapshot_t, applied_left_permille) == 56u &&
                  offsetof(car_speed_pi_diag_snapshot_t, status) == 60u &&
                  offsetof(car_speed_pi_diag_snapshot_t, termination_fault) == 64u &&
                  offsetof(car_speed_pi_diag_snapshot_t, saturation_left_ms) == 68u &&
                  offsetof(car_speed_pi_diag_snapshot_t, flags) == 76u &&
                  offsetof(car_speed_pi_diag_snapshot_t, pi_type) == 80u &&
                  offsetof(car_speed_pi_diag_snapshot_t, reserved) == 84u,
              "speed PI diagnostic ABI field offsets changed");
#else
_Static_assert(sizeof(car_speed_pi_diag_snapshot_t) == CAR_SPEED_PI_DIAG_SIZE,
               "speed PI diagnostic ABI must remain 128 bytes");
_Static_assert(offsetof(car_speed_pi_diag_snapshot_t, generation) == 8u &&
                   offsetof(car_speed_pi_diag_snapshot_t, crc32) == 12u &&
                   offsetof(car_speed_pi_diag_snapshot_t, start_token) == 16u &&
                   offsetof(car_speed_pi_diag_snapshot_t, target_left_millirps) == 32u &&
                   offsetof(car_speed_pi_diag_snapshot_t, measured_left_millirps) == 40u &&
                   offsetof(car_speed_pi_diag_snapshot_t, nominal_ff_left_permille) == 48u &&
                   offsetof(car_speed_pi_diag_snapshot_t, requested_left_permille) == 52u &&
                   offsetof(car_speed_pi_diag_snapshot_t, applied_left_permille) == 56u &&
                   offsetof(car_speed_pi_diag_snapshot_t, status) == 60u &&
                   offsetof(car_speed_pi_diag_snapshot_t, termination_fault) == 64u &&
                   offsetof(car_speed_pi_diag_snapshot_t, saturation_left_ms) == 68u &&
                   offsetof(car_speed_pi_diag_snapshot_t, flags) == 76u &&
                   offsetof(car_speed_pi_diag_snapshot_t, pi_type) == 80u &&
                   offsetof(car_speed_pi_diag_snapshot_t, reserved) == 84u,
               "speed PI diagnostic ABI field offsets changed");
#endif

#ifdef __cplusplus
}
#endif

#endif /* CARTEST_SPEED_PI_DIAG_H */
