cmake_minimum_required(VERSION 3.13)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)

# define the project to build
project(adxl345_test C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# simple adxl345 lib
add_library(adxl345_lib INTERFACE)
target_sources(adxl345_lib PUBLIC
    adxl345.c
    adxl345_bus.c
)

# define if you would like to use ADXL345 via I2C or SPI
set(ADXL345_BUS "I2C")

if (ADXL345_BUS  STREQUAL "I2C")
    target_compile_definitions(adxl345_lib INTERFACE ADXL345_I2C_BUS=1)
    target_link_libraries(adxl345_lib INTERFACE pico_stdlib hardware_i2c)

elseif(ADXL345_BUS STREQUAL "SPI")
    target_compile_definitions(adxl345_lib PRIVATE ADXL345_SPI_BUS=1)
message(FATAL_ERROR "Current selected BUS for ADXL345 not supported, please use I2C or SPI")
endif()


# example executable
add_executable(adxl345_exec
    main.c
)

target_link_libraries(adxl345_exec
    adxl345_lib
)

# enable usb output, disable uart output
pico_enable_stdio_usb(adxl345_exec 1)
pico_enable_stdio_uart(adxl345_exec 0)

# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(adxl345_exec)
