# Platform-specific files for this platform
set(PLATFORM_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/lcl_hadm_measurement.c
    ${CMAKE_CURRENT_LIST_DIR}/lcl_hadm_utils.c
    ${CMAKE_CURRENT_LIST_DIR}/lcl_xcvr_hal.c
)

# Platform-specific include paths for this platform
set(PLATFORM_INCLUDES
    ${CMAKE_CURRENT_LIST_DIR}/../../KW4x
    ${CMAKE_CURRENT_LIST_DIR}/../../../lib/mll_inc
    ${CMAKE_CURRENT_LIST_DIR}/../../../lib/threadx_inc
    ${CMAKE_CURRENT_LIST_DIR}/../../../lib/threadx_inc/cortex_m33
    ${SDK_DIR}/arch/arm/CMSIS/Core/Include
    ${SDK_DIR}/devices/Wireless/KW/KW47B42ZB7
    ${SDK_DIR}/devices/Wireless/KW/KW47B42ZB7/drivers
    ${SDK_DIR}/devices/Wireless/KW/periph6
    ${SDK_DIR}/drivers/gpio
    ${SDK_DIR}/drivers/ltc
    ${SDK_DIR}/drivers/port
    ${SDK_DIR}/drivers/tpm
    ${SDK_DIR}/middleware/wireless/framework/Common
    ${SDK_DIR}/middleware/wireless/framework/platform/wireless_mcu
    ${SDK_DIR}/middleware/wireless/XCVR/drv
    ${SDK_DIR}/middleware/wireless/XCVR/drv/nb2p4ghz
    ${SDK_DIR}/middleware/wireless/XCVR/drv/nb2p4ghz/configs/gen47
)

# Platform-specific configuration for kw47_mcxw72x_nbu
set(PLATFORM_DEFINES
    CPU_KW47B42ZB7AFTA_cm33_core1
)

set(PLATFORM_COMPILE_OPTIONS
    --cpu Cortex-M33.no_dsp
    -e
    --thumb
    --header_context
)

set(PLATFORM_LINK_OPTIONS
    --cpu Cortex-M33.no_dsp
)
