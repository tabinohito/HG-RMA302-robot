cmake_minimum_required(VERSION 3.13)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
include(${CMAKE_CURRENT_LIST_DIR}/pico_sdk_import.cmake)
project(hg_cart_power C CXX ASM)
pico_sdk_init()
add_executable(pico_power_master src/power_master.c)
target_include_directories(pico_power_master PRIVATE ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/panel ${CMAKE_CURRENT_LIST_DIR}/power ${CMAKE_CURRENT_LIST_DIR}/src)
target_link_libraries(pico_power_master PRIVATE pico_stdlib hardware_uart hardware_irq
    hardware_i2c hardware_pio hardware_clocks hardware_dma hardware_watchdog)
pico_generate_pio_header(pico_power_master ${PICO_SDK_PATH}/src/rp2_common/pico_status_led/ws2812.pio)
target_compile_definitions(pico_power_master PRIVATE
    CFG_TUSB_CONFIG_FILE="${PICO_SDK_PATH}/src/rp2_common/pico_stdio_usb/include/tusb_config.h"
    PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS=0 PICO_STDIO_USB_STDOUT_TIMEOUT_US=0
    CFG_TUD_CDC_TX_BUFSIZE=2048 CFG_TUD_CDC_RX_BUFSIZE=256
    USBD_PRODUCT="pico_power_master")
pico_enable_stdio_usb(pico_power_master 1)
pico_enable_stdio_uart(pico_power_master 0)
pico_add_extra_outputs(pico_power_master)
