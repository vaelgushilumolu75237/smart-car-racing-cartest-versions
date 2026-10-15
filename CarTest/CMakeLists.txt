# Set the minimum version of CMake that can be used
cmake_minimum_required(VERSION 3.20)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)
set(CMAKE_C_COMPILER_FORCED TRUE)
set(CMAKE_CXX_COMPILER_FORCED TRUE)

# Set target definition variables required by the toolchain file
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/target.cmake)

# Set the toolchain file for cross-compilation
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/gcc-arm-none-eabi.cmake)

# Project definition
project(CarTest LANGUAGES C CXX ASM)

set(PROJECT_NAME CarTest)

# Create an executable object type
add_executable(${PROJECT_NAME})

# Include additional CMake files for flags, components, and files
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/flags.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/components.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/files.cmake)

option(CARTEST_ENABLE_BENCH "Compile the isolated overhead single-motor diagnostic path" OFF)
option(CARTEST_ENABLE_FOLLOW "Compile the vision-follow runtime profile" OFF)
option(CARTEST_ENABLE_STEERING "Compile the motor-disabled vision steering profile" OFF)
option(CARTEST_HARDWARE_ACCEPTED "Permit motor and servo output; software permission only, not hardware acceptance" OFF)
option(CARTEST_RUNTIME_LOG_ENABLED "Record bounded runtime telemetry in RAM" ON)
set(CARTEST_FOLLOW_DIFF_PERMILLE "0" CACHE STRING
    "Follow steering differential level in permille (0, 50, or 100)")
set_property(CACHE CARTEST_FOLLOW_DIFF_PERMILLE PROPERTY STRINGS 0 50 100)

if(NOT CARTEST_FOLLOW_DIFF_PERMILLE MATCHES "^(0|50|100)$")
  message(FATAL_ERROR "CARTEST_FOLLOW_DIFF_PERMILLE must be 0, 50, or 100.")
endif()
if(NOT CARTEST_ENABLE_FOLLOW AND NOT CARTEST_FOLLOW_DIFF_PERMILLE STREQUAL "0")
  message(FATAL_ERROR "A nonzero Follow differential is legal only in a Follow profile.")
endif()

set(_cartest_profile_count 0)
foreach(_profile_option CARTEST_ENABLE_BENCH CARTEST_ENABLE_FOLLOW CARTEST_ENABLE_STEERING)
  if(${_profile_option})
    math(EXPR _cartest_profile_count "${_cartest_profile_count} + 1")
  endif()
endforeach()
if(_cartest_profile_count GREATER 1)
  message(FATAL_ERROR "Select at most one active CarTest profile; bench, follow, and vision-steering modes cannot be combined.")
endif()

# Embed a build ID that identifies the source inputs and compile profile. The
# RAM logger header is matched against this ROM descriptor before export; this
# is deliberately not an ELF self-hash.
file(GLOB_RECURSE _cartest_build_id_inputs CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/App/*.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/App/*.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/App/*.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/Inc/*.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/Src/*.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/Src/*.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/Src/*.S"
  "${CMAKE_CURRENT_SOURCE_DIR}/Src/*.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/*.cmake"
  "${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32H7xx_HAL_Driver/Src/*.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32H7xx_HAL_Driver/Inc/*.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32H7xx/Include/*.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Core/Include/*.h"
)
list(APPEND _cartest_build_id_inputs
  "${CMAKE_CURRENT_SOURCE_DIR}/CMakeLists.txt"
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/runtime_log_build_id.h.in"
  "${CMAKE_CURRENT_SOURCE_DIR}/stm32h743xi_flash.ld"
)
list(REMOVE_DUPLICATES _cartest_build_id_inputs)
list(SORT _cartest_build_id_inputs)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
             ${_cartest_build_id_inputs})
string(TOUPPER "${CMAKE_BUILD_TYPE}" _cartest_build_type_upper)
set(_cartest_build_id_material
  "profile=${CARTEST_ENABLE_BENCH},${CARTEST_ENABLE_FOLLOW},${CARTEST_ENABLE_STEERING},${CARTEST_HARDWARE_ACCEPTED}\n"
  "runtime_log_enabled=${CARTEST_RUNTIME_LOG_ENABLED}\n"
  "follow_diff_permille=${CARTEST_FOLLOW_DIFF_PERMILLE}\n"
  "follow_base_permille=500;follow_normal_cap_permille=600;follow_hard_cap_permille=700\n"
  "build_type=${CMAKE_BUILD_TYPE}\n"
  "c_compiler=${CMAKE_C_COMPILER_ID}-${CMAKE_C_COMPILER_VERSION}\n"
  "cxx_compiler=${CMAKE_CXX_COMPILER_ID}-${CMAKE_CXX_COMPILER_VERSION}\n"
  "c_flags=${CMAKE_C_FLAGS}\n"
  "cxx_flags=${CMAKE_CXX_FLAGS}\n"
  "asm_flags=${CMAKE_ASM_FLAGS}\n"
  "link_flags=${CMAKE_EXE_LINKER_FLAGS}\n"
  "c_flags_config=${CMAKE_C_FLAGS_${_cartest_build_type_upper}}\n"
  "cxx_flags_config=${CMAKE_CXX_FLAGS_${_cartest_build_type_upper}}\n"
  "asm_flags_config=${CMAKE_ASM_FLAGS_${_cartest_build_type_upper}}\n"
  "link_flags_config=${CMAKE_EXE_LINKER_FLAGS_${_cartest_build_type_upper}}\n"
)
foreach(_build_id_input IN LISTS _cartest_build_id_inputs)
  file(SHA256 "${_build_id_input}" _build_id_input_sha256)
  file(RELATIVE_PATH _build_id_relative "${CMAKE_CURRENT_SOURCE_DIR}"
       "${_build_id_input}")
  string(APPEND _cartest_build_id_material
         "${_build_id_relative}=${_build_id_input_sha256}\n")
endforeach()
string(SHA256 _cartest_build_id_sha256 "${_cartest_build_id_material}")
string(SUBSTRING "${_cartest_build_id_sha256}" 0 32
       CARTEST_RUNTIME_LOG_BUILD_ID_HEX)
set(CARTEST_RUNTIME_LOG_BUILD_ID_BYTES "")
foreach(_build_id_offset RANGE 0 30 2)
  string(SUBSTRING "${CARTEST_RUNTIME_LOG_BUILD_ID_HEX}"
         ${_build_id_offset} 2 _build_id_byte)
  if(NOT CARTEST_RUNTIME_LOG_BUILD_ID_BYTES STREQUAL "")
    string(APPEND CARTEST_RUNTIME_LOG_BUILD_ID_BYTES ", ")
  endif()
  string(APPEND CARTEST_RUNTIME_LOG_BUILD_ID_BYTES "0x${_build_id_byte}u")
endforeach()
set(_cartest_generated_dir "${CMAKE_CURRENT_BINARY_DIR}/generated")
file(MAKE_DIRECTORY "${_cartest_generated_dir}")
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/runtime_log_build_id.h.in"
  "${_cartest_generated_dir}/runtime_log_build_id.h"
  @ONLY
)

# Add sources to executable/library
target_sources(${PROJECT_NAME} PRIVATE
  # User defined sources
  "${CMAKE_CURRENT_SOURCE_DIR}/App/commissioning.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/App/follow_output.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/App/manual_run_button.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/App/servo_safety.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/App/uart_rx_service.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/App/reverse_control.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/App/car_vision.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/App/encoder_diag.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/App/runtime_log.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/App/run_block_reason.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/App/vision_protocol.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/Src/system_stm32h7xx.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_cortex.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_gpio.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim_ex.c"
)

# Add include directories
target_include_directories(${PROJECT_NAME} PRIVATE
  # User defined include directories
  "${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32H7xx_HAL_Driver/Inc"
  "${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32H7xx/Include"
  "${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Core/Include"
  "${_cartest_generated_dir}"
)

target_compile_definitions(${PROJECT_NAME} PRIVATE
  USE_HAL_DRIVER
  STM32H743xx
  CARTEST_BENCH_MODE_COMPILED=$<BOOL:${CARTEST_ENABLE_BENCH}>
  CARTEST_FOLLOW_MODE_COMPILED=$<BOOL:${CARTEST_ENABLE_FOLLOW}>
  CARTEST_STEERING_MODE_COMPILED=$<BOOL:${CARTEST_ENABLE_STEERING}>
  CARTEST_HARDWARE_ACCEPTED=$<BOOL:${CARTEST_HARDWARE_ACCEPTED}>
  CARTEST_RUNTIME_LOG_ENABLED=$<BOOL:${CARTEST_RUNTIME_LOG_ENABLED}>
  CARTEST_FOLLOW_DIFF_PERMILLE=${CARTEST_FOLLOW_DIFF_PERMILLE}
)

# Compiler options
target_compile_options(${PROJECT_NAME} PRIVATE
  # User defined compiler options
  $<$<COMPILE_LANGUAGE:CXX>:-std=gnu++17>
)

# Add additional defined symbols
target_compile_definitions(${PROJECT_NAME} PRIVATE
  # Add additional defined symbols here
)

# Linker options
target_link_options(${PROJECT_NAME} PRIVATE 
  # User defined linker options
)

# Add additional linker search directories
target_link_directories(${PROJECT_NAME} PRIVATE
  # Add additional linker search directories here
)

# Add additional linker libraries
target_link_libraries(${PROJECT_NAME} PRIVATE
  # Add additional linker libraries here
)
