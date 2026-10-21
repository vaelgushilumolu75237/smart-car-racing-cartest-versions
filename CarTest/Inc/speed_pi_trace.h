#ifndef CARTEST_SPEED_PI_TRACE_H
#define CARTEST_SPEED_PI_TRACE_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
    CAR_SPEED_PI_TRACE_MAGIC = 0x43525450u, /* Little-endian "PTRC". */
    CAR_SPEED_PI_TRACE_VERSION = 1u,
    CAR_SPEED_PI_TRACE_HEADER_SIZE = 64u,
    CAR_SPEED_PI_TRACE_SAMPLE_SIZE = 48u,
    CAR_SPEED_PI_TRACE_CAPACITY = 2048u,
    CAR_SPEED_PI_TRACE_SECTION_SIZE = CAR_SPEED_PI_TRACE_HEADER_SIZE +
        CAR_SPEED_PI_TRACE_SAMPLE_SIZE * CAR_SPEED_PI_TRACE_CAPACITY,
    CAR_SPEED_PI_TRACE_STATE_IDLE = 0u,
    CAR_SPEED_PI_TRACE_STATE_ACTIVE = 1u,
    CAR_SPEED_PI_TRACE_STATE_FROZEN = 2u,
    CAR_SPEED_PI_TRACE_FLAG_SPEED_VALID = 1u << 0,
    CAR_SPEED_PI_TRACE_FLAG_ARMED = 1u << 1,
    CAR_SPEED_PI_TRACE_FLAG_HOLD = 1u << 2,
    CAR_SPEED_PI_TRACE_FLAG_PROPOSAL_VALID = 1u << 3
};

#if defined(_MSC_VER)
#pragma pack(push, 1)
#define CAR_SPEED_PI_TRACE_PACKED
#else
#define CAR_SPEED_PI_TRACE_PACKED __attribute__((packed))
#endif

typedef struct CAR_SPEED_PI_TRACE_PACKED {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    uint32_t generation;
    uint32_t header_crc32;
    uint32_t records_crc32;
    uint32_t start_token;
    uint32_t start_ms;
    uint32_t stop_ms;
    uint16_t capacity;
    uint16_t count;
    uint16_t next_index;
    uint8_t pi_type;
    uint8_t state;
    uint32_t total_samples;
    uint32_t lost_samples;
    uint8_t build_id[16];
} car_speed_pi_trace_header_t;

typedef struct CAR_SPEED_PI_TRACE_PACKED {
    uint32_t sample_index;
    uint32_t record_ms;              /* snapshot tick, includes applied output */
    uint32_t sample_ms;
    uint16_t vision_sequence;
    uint32_t vision_arrival_ms;
    uint8_t pi_type;
    uint8_t flags;
    int32_t target_left_millirps;
    int32_t target_right_millirps;
    int32_t measured_left_millirps;
    int32_t measured_right_millirps;
    uint16_t requested_left_permille;
    uint16_t requested_right_permille;
    uint16_t applied_left_permille;
    uint16_t applied_right_permille;
    uint32_t crc32;
} car_speed_pi_trace_sample_t;

typedef struct CAR_SPEED_PI_TRACE_PACKED {
    car_speed_pi_trace_header_t header;
    car_speed_pi_trace_sample_t samples[CAR_SPEED_PI_TRACE_CAPACITY];
} car_speed_pi_trace_t;

#if defined(_MSC_VER)
#pragma pack(pop)
#endif
#undef CAR_SPEED_PI_TRACE_PACKED

typedef struct {
    uint32_t record_ms;
    uint32_t sample_ms;
    uint16_t vision_sequence;
    uint32_t vision_arrival_ms;
    uint8_t pi_type;
    uint8_t speed_valid;
    uint8_t armed;
    uint8_t hold_active;
    uint8_t proposal_valid;
    int32_t target_left_millirps;
    int32_t target_right_millirps;
    int32_t measured_left_millirps;
    int32_t measured_right_millirps;
    uint16_t requested_left_permille;
    uint16_t requested_right_permille;
    uint16_t applied_left_permille;
    uint16_t applied_right_permille;
} car_speed_pi_trace_values_t;

extern car_speed_pi_trace_t g_speed_pi_trace;

void car_speed_pi_trace_init(void);
void car_speed_pi_trace_begin(uint32_t start_token, uint32_t start_ms,
                              uint8_t pi_type);
void car_speed_pi_trace_append(const car_speed_pi_trace_values_t *values);
void car_speed_pi_trace_freeze(uint32_t stop_ms);
uint32_t car_speed_pi_trace_crc32(const void *bytes, uint32_t length);
uint32_t car_speed_pi_trace_header_crc32(
    const car_speed_pi_trace_header_t *header);
uint32_t car_speed_pi_trace_sample_crc32(
    const car_speed_pi_trace_sample_t *sample);

#if defined(__cplusplus)
static_assert(sizeof(car_speed_pi_trace_header_t) ==
                  CAR_SPEED_PI_TRACE_HEADER_SIZE,
              "speed PI trace header ABI must remain 64 bytes");
static_assert(sizeof(car_speed_pi_trace_sample_t) ==
                  CAR_SPEED_PI_TRACE_SAMPLE_SIZE,
              "speed PI trace sample ABI must remain 40 bytes");
static_assert(sizeof(car_speed_pi_trace_t) == CAR_SPEED_PI_TRACE_SECTION_SIZE,
              "speed PI trace section ABI changed");
#else
_Static_assert(sizeof(car_speed_pi_trace_header_t) ==
                   CAR_SPEED_PI_TRACE_HEADER_SIZE,
               "speed PI trace header ABI must remain 64 bytes");
_Static_assert(sizeof(car_speed_pi_trace_sample_t) ==
                   CAR_SPEED_PI_TRACE_SAMPLE_SIZE,
               "speed PI trace sample ABI must remain 40 bytes");
_Static_assert(sizeof(car_speed_pi_trace_t) == CAR_SPEED_PI_TRACE_SECTION_SIZE,
               "speed PI trace section ABI changed");
#endif

#ifdef __cplusplus
}
#endif

#endif /* CARTEST_SPEED_PI_TRACE_H */
