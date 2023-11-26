# Raspberry Pi Pico ADXL345 Quickstart
Simple example code to control the ADXL345 accelerometer via a Raspberry Pi Pico. Because there was no real repository available (for any MCU) which aimed to flash and play with interrupts, I have written my own.

## Installation
Please install the the [Raspberry Pi Pico C/C++ SDK](https://www.raspberrypi.com/documentation/microcontrollers/c_sdk.html#sdk-setup), clone the repository and build it as usually:

```bash
# create build directory and navigate into it
$ mkdir build && build
# generate cmake project
$ cmake ..
# build cmake project
$ make
```

## Running
Once flashed to the MCU, it will open a serial monitor via USB and outputs the current accel data in combination with movement active and inactive interrupts. The chosen baud rate is `115200`. 

```bash
Raspberry Pi Pico ADXL345 (I2C) Example Test Application
ADXL345 setup done
X: 0 Y: 0 Z: 0; combined: 0.000000
X: 32 Y: -32 Z: 896; combined: 897.142129
X: 32 Y: 0 Z: 896; combined: 896.571246
X: 32 Y: -32 Z: 928; combined: 929.102793
X: 32 Y: 0 Z: 960; combined: 960.533185
X: 32 Y: 0 Z: 960; combined: 960.533185
interrupt! reasons: 0x8b
We have back in stationary mode!!!!!
X: 32 Y: 0 Z: 928; combined: 928.551560
interrupt! reasons: 0x93
We have active moving!!!!!
X: 32 Y: -32 Z: 928; combined: 929.102793
X: 32 Y: -64 Z: 928; combined: 930.754533
X: 32 Y: 0 Z: 896; combined: 896.571246
X: -64 Y: -96 Z: 832; combined: 839.961904
X: 32 Y: -32 Z: 960; combined: 961.066075
interrupt! reasons: 0x8b
We have back in stationary mode!!!!!
X: 32 Y: -32 Z: 960; combined: 961.066075
```

## Current state
The example code does not support SPI and all interrupts yet. Please feel free to add the missing parts.
SPI implementation should be quite easy to implement, simply implement the public API functions defined in the file `adxl345_bus.h` for the SPI backend.
Interrupts can be also added easy.

There is also no supported yet to select which ADXL345 interrupt pin should be used and if RAISING or FALLING edge should be leveraged.

## Library
As stated in the main section, this project does not aim to provide a ADXL345 library. However, it should not be hard to use it as a library tough.
Please simply copy the files `adxl345.c/h` and `adxl345_bus.c/h` into your project and apply the following `CMakeLists.txt` update:
```cmake
# define library

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

# link the library with your target

target_link_libraries(<target_name>
    adxl345_lib
)
```
