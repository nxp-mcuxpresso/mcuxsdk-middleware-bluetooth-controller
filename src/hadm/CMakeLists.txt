cmake_minimum_required(VERSION 3.16)

# Set the toolchain file for IAR
set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/iar.cmake" CACHE FILEPATH "Toolchain file")

project(ble_hadm_library C)

set(SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../../..")

# Platform selection option
set(PLATFORM_TARGET "kw47_mcxw72x_nbu" CACHE STRING "Target platform")
set_property(CACHE PLATFORM_TARGET PROPERTY STRINGS "kw47_mcxw72x_nbu" "kw43_mcxw70_nbu")

# Validate platform selection
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${PLATFORM_TARGET}")
    message(FATAL_ERROR "Platform directory ${PLATFORM_TARGET} does not exist")
endif()

# Include platform-specific configuration if it exists
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${PLATFORM_TARGET}/platform.cmake")
    include("${CMAKE_CURRENT_SOURCE_DIR}/${PLATFORM_TARGET}/platform.cmake")
endif()

# Common source files (platform independent)
set(COMMON_SOURCES
    ble_hadm_hal_impl.c
    lcl_hadm_aes.c
    lcl_hadm_hartt.c
)

# Create the library
add_library(ble_hadm_library STATIC
    ${COMMON_SOURCES}
    ${PLATFORM_SOURCES}
)

# Set include directories
target_include_directories(ble_hadm_library
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/${PLATFORM_TARGET}
        ${PLATFORM_INCLUDES}
        ${SDK_DIR}/drivers/common
        ${SDK_DIR}/components/lists
        ${SDK_DIR}/components/osa
        ${SDK_DIR}/components/osa/config
)

# Warnings as errors
target_compile_options(ble_hadm_library PRIVATE --warnings_are_errors)

# Apply platform-specific compiler definitions if defined
if(DEFINED PLATFORM_DEFINES)
    target_compile_definitions(ble_hadm_library PRIVATE ${PLATFORM_DEFINES})
endif()

# Apply platform-specific compiler options if defined
if(DEFINED PLATFORM_COMPILE_OPTIONS)
    target_compile_options(ble_hadm_library PRIVATE ${PLATFORM_COMPILE_OPTIONS})
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_definitions(ble_hadm_library PRIVATE DEBUG)
endif()

if (VALIDATIONBUILDOPTIONS MATCHES "True")
    # Add validation build options if needed
    message("-- + HAL GPIOS enabled")
    target_compile_definitions(ble_hadm_library PRIVATE VALIDATIONBUILDOPTIONS)
endif()

# Set library properties
set_target_properties(ble_hadm_library PROPERTIES
    OUTPUT_NAME "ble_hadm_${PLATFORM_TARGET}"
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
)

# Print configuration information
message(STATUS "Building for platform: ${PLATFORM_TARGET}")
if(DEFINED PLATFORM_DEFINES)
    message(STATUS "Platform defines: ${PLATFORM_DEFINES}")
endif()

add_custom_command(TARGET ble_hadm_library
    POST_BUILD
    COMMAND cmake -E echo "Copying ble_hadm_${PLATFORM_TARGET}.a to ${SDK_DIR}/middleware/wireless/ble_controller/lib"
    COMMAND cmake -E copy ${CMAKE_BINARY_DIR}/lib/ble_hadm_${PLATFORM_TARGET}.a ${SDK_DIR}/middleware/wireless/ble_controller/lib
    VERBATIM
)
