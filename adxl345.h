#ifndef ADXL345_H
#define ADXL345_H

#include <stdint.h>
#include <math.h>

#define ADXL345_OK 0
#define ADXL345_ERR -1

#define BIT(x) ((uint16_t)1 << (x))

// ADXL345 I2C address (we use the alternative address)
#define ADXL345_I2C_ADDRESS 0x53

// ADXL345 registers
#define ADXL345_DEVICE_ID_REG 0x00
#define ADXL345_DEVICE_ID_VALUE 0b11100101

// ADXL345 THRESH_TAP register
#define ADXL345_THRESH_TAP_REG 0x1D

// ADXL345 DUR register
#define ADXL345_DUR_REG 0x21

// ADXL345 THRESH_ACT register
#define ADXL345_THRESH_ACT_REG 0x24

// ADXL345 THRESH_INACT register
#define ADXL345_THRESH_INACT_REG 0x25

// ADXL345 TIME_INACT register
#define ADXL345_TIME_INACT_REG 0x26

// ADXL345 ACT_INACT_CTL register
#define ADXL345_ACT_INACT_CTL_REG 0x27
#define ADXL345_ACT_INACT_CTL_ACT_DECOUPLE_MSK 0b10000000
#define ADXL345_ACT_INACT_CTL_ACT_AXIS_MSK 0b01110000
#define ADXL345_ACT_INACT_CTL_INACT_DECOUPLE_MSK 0b00001000
#define ADXL345_ACT_INACT_CTL_INACT_AXIS_MSK 0b00000111

// ADXL345 THRESH_FF register
#define ADXL345_THRESH_FF_REG 0x28
// ADXL345 TIME_FF register
#define ADXL345_TIME_FF_REG 0x29

// ADXL345 TAP_AXES register
#define ADXL345_TAP_AXES_REG 0x2A
#define ADXL345_TAP_AXES_TAP_AXIS_MSK 0b111

// ADXL345 BW_RATE register
#define ADXL345_BW_RATE_REG 0x2C
#define ADXL345_BW_RATE_RATE_MSK 0b1111

// ADXL345 POWER_CTL register
#define ADXL345_POWER_CTL_REG 0x2D
#define ADXL345_POWER_CTL_REG_MEASURE_BIT BIT(3)
#define ADXL345_POWER_CTL_LINK_BIT BIT(5)

// ADXL345 INT_ENABLE register
#define ADXL345_INT_ENABLE_REG 0x2E
#define ADXL345_INT_ENABLE_WATERMARK_BIT BIT(1)
#define ADXL345_INT_ENABLE_FREE_FALL_BIT BIT(2)
#define ADXL345_INT_ENABLE_INACTIVE_BIT BIT(3)
#define ADXL345_INT_ENABLE_ACTIVE_BIT BIT(4)
#define ADXL345_INT_ENABLE_DOUBLE_TAP_BIT BIT(5)
#define ADXL345_INT_ENABLE_SINGLE_TAP_BIT BIT(6)
#define ADXL345_INT_ENABLE_DATA_READY_BIT BIT(7)

// ADXL345 INT_SOURCE register
#define ADXL345_INT_SOURCE_REG 0x30
#define ADXL345_INT_SOURCE_WATERMARK_BIT BIT(1)
#define ADXL345_INT_SOURCE_FREE_FALL_BIT BIT(2)
#define ADXL345_INT_SOURCE_INACTIVE_BIT BIT(3)
#define ADXL345_INT_SOURCE_ACTIVE_BIT BIT(4)
#define ADXL345_INT_SOURCE_DOUBLE_TAP_BIT BIT(5)
#define ADXL345_INT_SOURCE_SINGLE_TAP_BIT BIT(6)
#define ADXL345_INT_SOURCE_DATA_READY_BIT BIT(7)

// ADXL345 DATA_FORMAT register
#define ADXL345_DATA_FORMAT_REG 0x31
#define ADXL345_DATA_FORMAT_REG_RANGE_MSK 0b11

// ADXL345 DATAX0, etc registers
#define ADXL345_X_AXIS_DATA_0_REG 0x32

// ADXL345 FIFO_CTL register
#define ADXL345_FIFO_CTL_REG 0x38
#define ADXL345_FIFO_CTL_REG_FIFO_MSK 0b11000000
#define ADXL345_FIFO_CTL_SAMPLE_MSK 0b11111

// ADXL345 FIFO_STATUS register
#define ADXL345_FIFO_STATUS 0x39

/**
 * @brief ADXL345 sampling frequency
 */
typedef enum
{
    ADXL345_ODR_6P25HZ = 0b0110,
    ADXL345_ODR_12P5HZ = 0b0111,
    ADXL345_ODR_25HZ = 0b1000,
    ADXL345_ODR_50HZ = 0b1001,
    ADXL345_ODR_100HZ = 0b1010,
    ADXL345_ODR_200HZ = 0b1100,
    ADXL345_ODR_400HZ = 0b1101,
} adxl345_odr_t;

/**
 * @brief ADXL345 acceleration range
 */
typedef enum
{
    ADXL345_2G_RANGE = 0b00,
    ADXL345_4G_RANGE = 0b01,
    ADXL345_8G_RANGE = 0b10,
    ADXL345_16G_RANGE = 0b11,
} adxl345_range_t;

typedef enum
{
    ADXL345_BYPASS = 0b00000000,
    ADXL345_FIFO = 0b01000000,
    ADXL345_STREAM = 0b10000000,
    ADXL345_TRIGGER = 0b11000000,
} adxl345_fifo_t;

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} adxl345_value_t;

typedef void (*adxl345_trigger_cb)(void);

/**
 * @brief Initializes the ADXL345 accelerometer with specified settings.
 *
 * This function initializes the ADXL345 accelerometer with the given parameters.
 *
 * @param[in] - frequency The output data rate of the accelerometer.
 * @param[in] - range The measurement range of the accelerometer.
 * @param[in] - fifo The FIFO mode of the accelerometer.
 * @return ADXL345_OK on success, ADXL345_ERR on failure.
 */
int adxl345_initialize(adxl345_odr_t frequency, adxl345_range_t range, adxl345_fifo_t fifo);

/**
 * @brief Reads the current acceleration values from the ADXL345 accelerometer.
 *
 * This function reads the acceleration values in mG from the ADXL345 accelerometer and stores them in the provided structure.
 *
 * @param[out] value - Pointer to the structure where acceleration values will be stored.
 * @return ADXL345_OK on success, ADXL345_ERR on failure.
 */
int adxl345_read(adxl345_value_t *value);

/**
 * @brief Reads the raw acceleration values from the ADXL345 accelerometer.
 *
 * This function reads the raw acceleration values from the ADXL345 accelerometer and stores them in the provided structure.
 *
 * @param[out] value - Pointer to the structure where raw acceleration values will be stored.
 * @return ADXL345_OK on success, ADXL345_ERR on failure.
 */
int adxl345_read_raw(adxl345_value_t *value);

/**
 * @brief Gets the size of the FIFO buffer in the ADXL345 accelerometer.
 *
 * This function retrieves the current size of the FIFO buffer in the ADXL345 accelerometer.
 *
 * @param[out] size - Pointer to the variable where the FIFO size will be stored.
 * @return ADXL345_OK on success, ADXL345_ERR on failure.
 */
int adxl345_get_fifo_size(uint8_t *size);

/**
 * @brief Enable interrupt processing on the specified GPIO pin. Please ensure that the correct interrupt PIN from the ADXL345 is connected to the enabled PIN.
 *
 * @param[in] gpio_pin - GPIO pin to listen for interrupts
 * @return ADXL345_OK on success, ADXL345_ERR on failure.
 */
int adxl345_enable_interrupt(uint8_t gpio_pin);

/**
 * @brief Gets the trigger source in the ADXL345 accelerometer.
 *
 * This function retrieves the current trigger source in the ADXL345 accelerometer.
 *
 * @param[out] trigger - Pointer to the variable where the trigger source will be stored.
 * @return ADXL345_OK on success, ADXL345_ERR on failure.
 */
int adxl345_get_trigger_source(uint8_t *trigger);

/**
 * @brief Registers a trigger for free fall detection in the ADXL345 accelerometer.
 *
 * This function registers a trigger for free fall detection with the specified threshold and time parameters.
 *
 * @param[in] threshold - The threshold value for free fall detection.
 * @param[in] time      - The time window for free fall detection in ms.
 * @param[in] cb        - Interrupt callback to execute
 * @return ADXL345_OK on success, ADXL345_ERR on failure.
 */
int adxl345_register_trigger_free_fall(uint16_t threshold, uint16_t time, adxl345_trigger_cb cb);

/**
 * @brief Registers a trigger for single tap detection in the ADXL345 accelerometer.
 *
 * This function registers a trigger for single tap detection with the specified threshold and time parameters.
 *
 * @param[in] threshold - The threshold value for single tap detection.
 * @param[in] time      - The time window for single tap detection in us.
 * @param[in] cb        - Interrupt callback to execute
 * @return ADXL345_OK on success, ADXL345_ERR on failure.
 */
int adxl345_register_trigger_single_tap(uint16_t threshold, uint32_t time, adxl345_trigger_cb cb);

/**
 * @brief Registers a trigger for data ready in the ADXL345 accelerometer.
 *
 * This function registers a trigger for data ready in the ADXL345 accelerometer.
 *
 * @param[in] cb        - Interrupt callback to execute
 * @return ADXL345_OK on success, ADXL345_ERR on failure.
 */
int adxl345_register_trigger_data_ready(adxl345_trigger_cb cb);

/**
 * @brief Registers a trigger for watermark in the ADXL345 accelerometer.
 *
 * This function registers a trigger for watermark in the ADXL345 accelerometer with the specified watermark value.
 *
 * @param[in] watermark - The watermark value for triggering.
 * @param[in] cb        - Interrupt callback to execute
 * @return ADXL345_OK on success, ADXL345_ERR on failure.
 */
int adxl345_register_trigger_watermark(uint8_t watermark, adxl345_trigger_cb cb);

/**
 * @brief Registers a trigger for active movement in the ADXL345 accelerometer.
 *
 * @param[in] threshold - The threshold value for detecting active movement.
 * @param[in] cb        - Interrupt callback to execute
 * @return ADXL345_OK on success, ADXL345_ERR on failure.
 */
int adxl345_register_trigger_active(uint16_t threshold, adxl345_trigger_cb cb);

/**
 * @brief Registers a trigger for inactive movement in the ADXL345 accelerometer.
 *
 * @param[in] threshold - The threshold value for detecting inactive movement.
 * @param[in] time_ms   - The time window for detecting inactive movement.
 * @param[in] cb        - Interrupt callback to execute
 * @return ADXL345_OK on success, ADXL345_ERR on failure.
 */
int adxl345_register_trigger_inactive(uint16_t threshold, uint32_t time_ms, adxl345_trigger_cb cb);

/**
 * @brief Link active and inactive movement in the ADXL345 accelerometer.
 * 
 * @return ADXL345_OK on success, ADXL345_ERR on failure.
*/
int adxl345_link_movement();

/**
 * @brief Compute the combined gravity force of the provided accel data
 *
 * @param[in] value - accel data
 * @return combined computed force
 */
inline double adxl345_force_vector(adxl345_value_t *value)
{
    return sqrt(value->x * value->x + value->y * value->y + value->z * value->z);
}

#endif