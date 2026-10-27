/* Optional OpenMV/MicroPython binding for the exact fast_path kernels. */

#include <stdint.h>

#include "py/obj.h"
#include "py/binary.h"
#include "py/runtime.h"
#include "cartest_path_accel_core.h"

#if BYTEARRAY_TYPECODE != 1
#error "cartest_path_accel requires bytearray BYTEARRAY_TYPECODE == 1"
#endif

_Static_assert(sizeof(int32_t) == 4, "path acceleration needs 32-bit int32_t");

static int32_t get_int(mp_obj_t object, int32_t minimum, int32_t maximum) {
    mp_int_t value;
    if (mp_obj_is_bool(object) || !mp_obj_is_int(object)) {
        mp_raise_TypeError(MP_ERROR_TEXT("cartest_path_accel: expected integer"));
    }
    value = mp_obj_get_int(object);
    if (value < minimum || value > maximum) {
        mp_raise_ValueError(MP_ERROR_TEXT(
            "cartest_path_accel: integer out of range"));
    }
    return (int32_t) value;
}

static void bytearray_buffer(mp_obj_t object, mp_buffer_info_t *buffer,
                             size_t expected, mp_uint_t flags) {
    if (!mp_obj_is_type(object, &mp_type_bytearray)) {
        mp_raise_TypeError(MP_ERROR_TEXT(
            "cartest_path_accel: expected bytearray"));
    }
    mp_get_buffer_raise(object, buffer, flags);
    if (buffer->typecode != BYTEARRAY_TYPECODE || buffer->len != expected ||
        buffer->buf == NULL) {
        mp_raise_ValueError(MP_ERROR_TEXT(
            "cartest_path_accel: bytearray format/length"));
    }
}

static void int_array_buffer(mp_obj_t object, mp_buffer_info_t *buffer,
                             size_t expected, mp_uint_t flags) {
    if (!mp_obj_is_type(object, &mp_type_array)) {
        mp_raise_TypeError(MP_ERROR_TEXT(
            "cartest_path_accel: expected array('i')"));
    }
    mp_get_buffer_raise(object, buffer, flags);
    if (buffer->typecode != 'i' || buffer->len != expected ||
        buffer->buf == NULL) {
        mp_raise_ValueError(MP_ERROR_TEXT(
            "cartest_path_accel: array format/length"));
    }
}

static mp_obj_t py_path_extract(size_t n_args, const mp_obj_t *args) {
    enum { A_LUMA, A_ROAD, A_EDGE, A_KNOWN, A_COUNTS, A_CANDIDATES,
           A_WIDTH, A_HEIGHT, A_FLOOR, A_MIN_WIDTH, A_MAX_WIDTH,
           A_MAX_PER_ROW };
    mp_buffer_info_t luma, road, edge, known, counts, candidates;
    int32_t width, height, floor, min_width, max_width, max_per_row;
    size_t layers, candidate_bytes;
    int32_t result;
    if (n_args != 12) {
        mp_raise_TypeError(MP_ERROR_TEXT(
            "extract_rows expects 12 positional arguments"));
    }
    width = get_int(args[A_WIDTH], 1, 320);
    height = get_int(args[A_HEIGHT], 1, 240);
    floor = get_int(args[A_FLOOR], 0, height - 1);
    min_width = get_int(args[A_MIN_WIDTH], 8, 70);
    max_width = get_int(args[A_MAX_WIDTH], min_width, 70);
    max_per_row = get_int(args[A_MAX_PER_ROW], 1,
                          CARTEST_PATH_ACCEL_MAX_PER_ROW);
    if (width != CARTEST_PATH_ACCEL_WIDTH ||
        height != CARTEST_PATH_ACCEL_HEIGHT) {
        mp_raise_ValueError(MP_ERROR_TEXT(
            "cartest_path_accel: unsupported geometry"));
    }
    layers = (size_t) (height - floor);
    candidate_bytes = layers * (size_t) max_per_row *
                      CARTEST_PATH_ACCEL_CANDIDATE_STRIDE * sizeof(int32_t);
    bytearray_buffer(args[A_LUMA], &luma, 80U * 60U, MP_BUFFER_READ);
    bytearray_buffer(args[A_ROAD], &road, 80U * 60U, MP_BUFFER_READ);
    bytearray_buffer(args[A_EDGE], &edge, 80U * 60U, MP_BUFFER_READ);
    bytearray_buffer(args[A_KNOWN], &known, 80U * 60U, MP_BUFFER_READ);
    bytearray_buffer(args[A_COUNTS], &counts, layers, MP_BUFFER_WRITE);
    int_array_buffer(args[A_CANDIDATES], &candidates, candidate_bytes,
                     MP_BUFFER_WRITE);
    result = cartest_path_accel_extract(
        (const uint8_t *) luma.buf, luma.len,
        (const uint8_t *) road.buf, road.len,
        (const uint8_t *) edge.buf, edge.len,
        (const uint8_t *) known.buf, known.len,
        (uint8_t *) counts.buf, counts.len,
        (int32_t *) candidates.buf, candidates.len,
        width, height, floor, min_width, max_width, max_per_row);
    if (result == CARTEST_PATH_ACCEL_INVALID) {
        mp_raise_ValueError(MP_ERROR_TEXT(
            "cartest_path_accel: invalid extract buffers"));
    }
    return mp_obj_new_int(result);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(
    cartest_path_accel_extract_obj, 12, 12, py_path_extract);

static mp_obj_t py_path_connect(size_t n_args, const mp_obj_t *args) {
    enum { A_COUNTS, A_CANDIDATES, A_STATES, A_WIDTH, A_HEIGHT, A_FLOOR,
           A_MAX_GAP, A_MAX_WIDTH_CHANGE, A_MAX_SLOPE, A_LINK_TOLERANCE,
           A_REFERENCE_Y, A_MAX_PER_ROW, A_STATE_CAPACITY };
    mp_buffer_info_t counts, candidates, states;
    int32_t width, height, floor, max_gap, max_width_change;
    int32_t max_slope, link_tolerance, reference_y, max_per_row;
    int32_t state_capacity;
    size_t layers, candidate_bytes, state_bytes;
    int32_t result;
    if (n_args != 13) {
        mp_raise_TypeError(MP_ERROR_TEXT(
            "connect_rows expects 13 positional arguments"));
    }
    width = get_int(args[A_WIDTH], 1, 320);
    height = get_int(args[A_HEIGHT], 1, 240);
    floor = get_int(args[A_FLOOR], 0, height - 1);
    max_gap = get_int(args[A_MAX_GAP], 0, 2);
    max_width_change = get_int(args[A_MAX_WIDTH_CHANGE], 1, 70);
    max_slope = get_int(args[A_MAX_SLOPE], 0, 80000);
    link_tolerance = get_int(args[A_LINK_TOLERANCE], 0, 80000);
    reference_y = get_int(args[A_REFERENCE_Y], 0,
                          CARTEST_PATH_ACCEL_HEIGHT * 1000 - 1);
    max_per_row = get_int(args[A_MAX_PER_ROW], 1,
                          CARTEST_PATH_ACCEL_MAX_PER_ROW);
    state_capacity = get_int(args[A_STATE_CAPACITY], 1, 10000);
    if (width != CARTEST_PATH_ACCEL_WIDTH ||
        height != CARTEST_PATH_ACCEL_HEIGHT) {
        mp_raise_ValueError(MP_ERROR_TEXT(
            "cartest_path_accel: unsupported geometry"));
    }
    layers = (size_t) (height - floor);
    candidate_bytes = layers * (size_t) max_per_row *
                      CARTEST_PATH_ACCEL_CANDIDATE_STRIDE * sizeof(int32_t);
    state_bytes = layers * (size_t) max_per_row * 2U *
                  CARTEST_PATH_ACCEL_STATE_STRIDE * sizeof(int32_t);
    bytearray_buffer(args[A_COUNTS], &counts, layers, MP_BUFFER_READ);
    int_array_buffer(args[A_CANDIDATES], &candidates, candidate_bytes,
                     MP_BUFFER_READ);
    int_array_buffer(args[A_STATES], &states, state_bytes, MP_BUFFER_WRITE);
    result = cartest_path_accel_connect(
        (const uint8_t *) counts.buf, counts.len,
        (const int32_t *) candidates.buf, candidates.len,
        (int32_t *) states.buf, states.len,
        width, height, floor, max_gap, max_width_change,
        max_slope, link_tolerance, reference_y, max_per_row,
        state_capacity);
    if (result == CARTEST_PATH_ACCEL_INVALID) {
        mp_raise_ValueError(MP_ERROR_TEXT(
            "cartest_path_accel: invalid connect buffers"));
    }
    return mp_obj_new_int(result);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(
    cartest_path_accel_connect_obj, 13, 13, py_path_connect);

static mp_obj_t cartest_path_accel_api_version(void) {
    return mp_obj_new_int(CARTEST_PATH_ACCEL_API_VERSION);
}
static MP_DEFINE_CONST_FUN_OBJ_0(
    cartest_path_accel_api_version_obj, cartest_path_accel_api_version);

static const mp_rom_map_elem_t cartest_path_accel_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),
      MP_ROM_QSTR(MP_QSTR_cartest_path_accel) },
    { MP_ROM_QSTR(MP_QSTR_API_VERSION),
      MP_ROM_INT(CARTEST_PATH_ACCEL_API_VERSION) },
    { MP_ROM_QSTR(MP_QSTR_api_version),
      MP_ROM_PTR(&cartest_path_accel_api_version_obj) },
    { MP_ROM_QSTR(MP_QSTR_extract_rows),
      MP_ROM_PTR(&cartest_path_accel_extract_obj) },
    { MP_ROM_QSTR(MP_QSTR_connect_rows),
      MP_ROM_PTR(&cartest_path_accel_connect_obj) },
};
static MP_DEFINE_CONST_DICT(cartest_path_accel_globals,
                            cartest_path_accel_globals_table);

const mp_obj_module_t cartest_path_accel_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *) &cartest_path_accel_globals,
};

MP_REGISTER_MODULE(MP_QSTR_cartest_path_accel, cartest_path_accel_module);
