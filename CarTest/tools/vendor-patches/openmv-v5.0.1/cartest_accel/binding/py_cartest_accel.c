/* Optional OpenMV/MicroPython binding for the Cartest topology C core. */

#include <stdint.h>

#include "py/obj.h"
#include "py/binary.h"
#include "py/gc.h"
#include "py/runtime.h"
#include "cartest_accel_core.h"

#include "board_config.h"

#if defined(OMV_JPEG_CODEC_ENABLE) && (OMV_JPEG_CODEC_ENABLE == 1)
#include "imlib.h"
#endif

_Static_assert(sizeof(int32_t) == 4, "cartest_accel needs 32-bit int32_t");
_Static_assert(sizeof(uint16_t) == 2, "cartest_accel needs 16-bit uint16_t");

static void get_buffer_type(mp_obj_t object, const mp_obj_type_t *type,
                            int typecode, mp_uint_t flags, size_t exact_len,
                            mp_buffer_info_t *buffer, const char *name) {
    if (!mp_obj_is_type(object, type)) {
        mp_raise_TypeError(MP_ERROR_TEXT("cartest_accel: wrong buffer type"));
    }
    mp_get_buffer_raise(object, buffer, flags);
    if (buffer->typecode != typecode || buffer->len != exact_len ||
        buffer->buf == NULL) {
        mp_raise_ValueError(MP_ERROR_TEXT("cartest_accel: buffer format/length"));
    }
    (void) name;
}

static int32_t get_int32(mp_obj_t object, int32_t minimum, int32_t maximum) {
    if (mp_obj_is_bool(object) || !mp_obj_is_int(object)) {
        mp_raise_TypeError(MP_ERROR_TEXT("cartest_accel: expected integer"));
    }
    mp_int_t value = mp_obj_get_int(object);
    if (value < minimum || value > maximum) {
        mp_raise_ValueError(MP_ERROR_TEXT("cartest_accel: integer out of range"));
    }
    return (int32_t) value;
}

static mp_obj_t py_cartest_accel_region_segments(size_t n_args,
                                                  const mp_obj_t *args) {
    enum { ARG_MASK, ARG_KNOWN, ARG_ROWS, ARG_FLOOR, ARG_TRUSTED,
           ARG_STARTS, ARG_ENDS, ARG_SEGMENT_COUNT };
    mp_buffer_info_t mask;
    mp_buffer_info_t known;
    mp_buffer_info_t rows;
    mp_buffer_info_t trusted;
    mp_buffer_info_t starts;
    mp_buffer_info_t ends;
    mp_buffer_info_t segment_count;
    int32_t floor;
    int32_t result;

    if (n_args != 8) {
        mp_raise_TypeError(MP_ERROR_TEXT(
            "region_segments expects 8 positional arguments"));
    }

    /* Collect and validate all Python object types and lengths before the
     * pure-C core reads or writes any buffer contents. */
    get_buffer_type(args[ARG_MASK], &mp_type_bytearray, BYTEARRAY_TYPECODE,
                    MP_BUFFER_READ,
                    CARTEST_ACCEL_PIXELS, &mask, "mask");
    get_buffer_type(args[ARG_KNOWN], &mp_type_bytearray, BYTEARRAY_TYPECODE,
                    MP_BUFFER_READ,
                    CARTEST_ACCEL_PIXELS, &known, "known");
    get_buffer_type(args[ARG_ROWS], &mp_type_bytearray, BYTEARRAY_TYPECODE,
                    MP_BUFFER_READ,
                    CARTEST_ACCEL_HEIGHT, &rows, "row_counts");
    floor = get_int32(args[ARG_FLOOR], 0, CARTEST_ACCEL_HEIGHT - 1);
    get_buffer_type(args[ARG_TRUSTED], &mp_type_bytearray, BYTEARRAY_TYPECODE,
                    MP_BUFFER_WRITE,
                    CARTEST_ACCEL_HEIGHT, &trusted, "trusted_rows");
    get_buffer_type(args[ARG_STARTS], &mp_type_bytearray, BYTEARRAY_TYPECODE,
                    MP_BUFFER_WRITE,
                    CARTEST_ACCEL_HEIGHT, &starts, "segment_starts");
    get_buffer_type(args[ARG_ENDS], &mp_type_bytearray, BYTEARRAY_TYPECODE,
                    MP_BUFFER_WRITE,
                    CARTEST_ACCEL_HEIGHT, &ends, "segment_ends");
    get_buffer_type(args[ARG_SEGMENT_COUNT], &mp_type_array, 'i',
                    MP_BUFFER_WRITE, 2 * sizeof(int32_t), &segment_count,
                    "segment_count_buffer");

    result = cartest_accel_region_segments(
        (const uint8_t *) mask.buf, mask.len,
        (const uint8_t *) known.buf, known.len,
        (const uint8_t *) rows.buf, rows.len,
        CARTEST_ACCEL_WIDTH, CARTEST_ACCEL_HEIGHT, floor,
        (uint8_t *) trusted.buf, trusted.len,
        (uint8_t *) starts.buf, starts.len,
        (uint8_t *) ends.buf, ends.len,
        (int32_t *) segment_count.buf, segment_count.len);
    if (result == CARTEST_ACCEL_INVALID) {
        mp_raise_ValueError(MP_ERROR_TEXT("cartest_accel invalid region buffers"));
    }
    return mp_obj_new_int(result);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(
    cartest_accel_region_segments_obj, 8, 8,
    py_cartest_accel_region_segments);

static mp_obj_t py_cartest_accel_internal_component(size_t n_args,
                                                     const mp_obj_t *args) {
    enum { ARG_MASK, ARG_VISITED, ARG_EPOCH, ARG_QUEUE, ARG_FLOOR,
           ARG_BAND_END, ARG_QUEUE_CAPACITY, ARG_MIN_AREA, ARG_MAX_AREA,
           ARG_MIN_EXTENT, ARG_MAX_EXTENT, ARG_MIN_HOLE_AREA };
    mp_buffer_info_t mask;
    mp_buffer_info_t visited;
    mp_buffer_info_t visit_epoch;
    mp_buffer_info_t queue;
    int32_t floor;
    int32_t band_end;
    int32_t queue_capacity;
    int32_t min_area;
    int32_t max_area;
    int32_t min_extent;
    int32_t max_extent;
    int32_t min_hole_area;
    int32_t result;

    if (n_args != 12) {
        mp_raise_TypeError(MP_ERROR_TEXT(
            "internal_component expects 12 positional arguments"));
    }

    /* All argument validation completes before the core examines epoch or
     * mutates the reusable visit map, epoch, or queue. */
    get_buffer_type(args[ARG_MASK], &mp_type_bytearray, BYTEARRAY_TYPECODE,
                    MP_BUFFER_READ,
                    CARTEST_ACCEL_PIXELS, &mask, "mask");
    get_buffer_type(args[ARG_VISITED], &mp_type_bytearray, BYTEARRAY_TYPECODE,
                    MP_BUFFER_WRITE,
                    CARTEST_ACCEL_PIXELS, &visited, "visited");
    get_buffer_type(args[ARG_EPOCH], &mp_type_array, 'i', MP_BUFFER_WRITE,
                    sizeof(int32_t), &visit_epoch, "visit_epoch");
    if (mp_obj_is_bool(args[ARG_QUEUE]) ||
        !mp_obj_is_type(args[ARG_QUEUE], &mp_type_array)) {
        mp_raise_TypeError(MP_ERROR_TEXT("cartest_accel: queue must be array"));
    }
    mp_get_buffer_raise(args[ARG_QUEUE], &queue, MP_BUFFER_WRITE);
    if (queue.typecode != 'H' ||
        queue.len < 2 * sizeof(uint16_t) ||
        queue.len > CARTEST_ACCEL_QUEUE_ENTRIES * sizeof(uint16_t) ||
        (queue.len % sizeof(uint16_t)) != 0 || queue.buf == NULL) {
        mp_raise_ValueError(MP_ERROR_TEXT("cartest_accel: queue format/length"));
    }

    floor = get_int32(args[ARG_FLOOR], 0, CARTEST_ACCEL_HEIGHT - 1);
    band_end = get_int32(args[ARG_BAND_END], floor + 1,
                         CARTEST_ACCEL_HEIGHT);
    queue_capacity = get_int32(args[ARG_QUEUE_CAPACITY], 2,
                               CARTEST_ACCEL_QUEUE_ENTRIES);
    min_area = get_int32(args[ARG_MIN_AREA], 1, CARTEST_ACCEL_PIXELS);
    max_area = get_int32(args[ARG_MAX_AREA], min_area, CARTEST_ACCEL_PIXELS);
    min_extent = get_int32(args[ARG_MIN_EXTENT], 1, CARTEST_ACCEL_WIDTH);
    max_extent = get_int32(args[ARG_MAX_EXTENT], min_extent,
                           CARTEST_ACCEL_WIDTH);
    min_hole_area = get_int32(args[ARG_MIN_HOLE_AREA], 1,
                              CARTEST_ACCEL_PIXELS);
    if ((size_t) queue_capacity > queue.len / sizeof(uint16_t)) {
        mp_raise_ValueError(MP_ERROR_TEXT("cartest_accel: queue capacity"));
    }

    result = cartest_accel_internal_component(
        (const uint8_t *) mask.buf, mask.len,
        (uint8_t *) visited.buf, visited.len,
        (int32_t *) visit_epoch.buf, visit_epoch.len,
        (uint16_t *) queue.buf, queue.len, (size_t) queue_capacity,
        CARTEST_ACCEL_WIDTH, CARTEST_ACCEL_HEIGHT, floor, band_end,
        min_area, max_area, min_extent, max_extent, min_hole_area);
    if (result == CARTEST_ACCEL_INVALID) {
        mp_raise_ValueError(MP_ERROR_TEXT(
            "cartest_accel invalid component buffers/settings"));
    }
    return mp_obj_new_int(result);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(
    cartest_accel_internal_component_obj, 12, 12,
    py_cartest_accel_internal_component);

static mp_obj_t cartest_accel_heap_stats(void) {
    gc_info_t info;
    mp_obj_t values[2];

    gc_info(&info);
    values[0] = mp_obj_new_int_from_uint((mp_uint_t) info.free);
    values[1] = mp_obj_new_int_from_uint((mp_uint_t) info.used);
    return mp_obj_new_tuple(2, values);
}
static MP_DEFINE_CONST_FUN_OBJ_0(cartest_accel_heap_stats_obj,
                                cartest_accel_heap_stats);

#if defined(OMV_JPEG_CODEC_ENABLE) && (OMV_JPEG_CODEC_ENABLE == 1)
static mp_obj_t cartest_accel_jpeg_encode_timeout_ms(size_t n_args,
                                                      const mp_obj_t *args) {
    uint32_t previous_timeout_ms = jpeg_get_encode_timeout_ms();

    if (n_args == 1) {
        int32_t requested_timeout_ms = get_int32(args[0], 1, 1000);
        if (!jpeg_set_encode_timeout_ms((uint32_t) requested_timeout_ms)) {
            mp_raise_ValueError(MP_ERROR_TEXT(
                "cartest_accel: JPEG timeout must be in 1..1000 ms"));
        }
    }
    return mp_obj_new_int_from_uint((mp_uint_t) previous_timeout_ms);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(
    cartest_accel_jpeg_encode_timeout_ms_obj, 0, 1,
    cartest_accel_jpeg_encode_timeout_ms);
#endif

static mp_obj_t cartest_accel_api_version(void) {
    return mp_obj_new_int(CARTEST_ACCEL_API_VERSION);
}
static MP_DEFINE_CONST_FUN_OBJ_0(cartest_accel_api_version_obj,
                                 cartest_accel_api_version);

static const mp_rom_map_elem_t cartest_accel_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_cartest_accel) },
    { MP_ROM_QSTR(MP_QSTR_API_VERSION),
      MP_ROM_INT(CARTEST_ACCEL_API_VERSION) },
    { MP_ROM_QSTR(MP_QSTR_HEAP_STATS_API), MP_ROM_INT(1) },
    { MP_ROM_QSTR(MP_QSTR_api_version),
      MP_ROM_PTR(&cartest_accel_api_version_obj) },
    { MP_ROM_QSTR(MP_QSTR_heap_stats),
      MP_ROM_PTR(&cartest_accel_heap_stats_obj) },
#if defined(OMV_JPEG_CODEC_ENABLE) && (OMV_JPEG_CODEC_ENABLE == 1)
    { MP_ROM_QSTR(MP_QSTR_PREVIEW_CODEC_GUARD_API), MP_ROM_INT(1) },
    { MP_ROM_QSTR(MP_QSTR_jpeg_encode_timeout_ms),
      MP_ROM_PTR(&cartest_accel_jpeg_encode_timeout_ms_obj) },
#endif
    { MP_ROM_QSTR(MP_QSTR_region_segments),
      MP_ROM_PTR(&cartest_accel_region_segments_obj) },
    { MP_ROM_QSTR(MP_QSTR_internal_component),
      MP_ROM_PTR(&cartest_accel_internal_component_obj) },
};
static MP_DEFINE_CONST_DICT(cartest_accel_globals,
                            cartest_accel_globals_table);

const mp_obj_module_t cartest_accel_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *) &cartest_accel_globals,
};

MP_REGISTER_MODULE(MP_QSTR_cartest_accel, cartest_accel_module);
