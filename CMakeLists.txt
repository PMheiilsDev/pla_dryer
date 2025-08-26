cmake_minimum_required(VERSION 3.17)
set(PICO_SDK_PATH "C:/RP2040/pico-sdk")
#include(${CMAKE_CURRENT_SOURCE_DIR}/../../../hw/bsp/family_support.cmake)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)

# Initialize the SDK
pico_sdk_init()

project(lcd_test C CXX ASM)

add_executable(lcd_test)

#target_compile_definitions( lcd_test PRIVATE CFG_TUSB_MCU=OPT_MCU_RP2040 )

target_link_libraries( lcd_test PUBLIC
        pico_stdlib
)


target_link_libraries(lcd_test PUBLIC
        #tinyusb_port
        #tinyusb_common
        hardware_adc
        hardware_pwm
        hardware_pio
        hardware_i2c
)

# Example source
target_sources(lcd_test PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/main.c
        ${CMAKE_CURRENT_SOURCE_DIR}/src/lcd/lcd.c
        )

# Example include
target_include_directories(lcd_test PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/src/lcd/include/
        )


pico_enable_stdio_uart(lcd_test 0)
pico_enable_stdio_usb(lcd_test 0)


pico_add_extra_outputs(lcd_test)
