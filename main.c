#include <stdio.h>
#include "pico/stdlib.h"
#include <stdint.h>
#include <math.h>

#include "adxl345.h"

#define APP_OK 0
#define APP_ERR -1

#define CHECK_ERROR(_rc, ...) \
    if (APP_OK != _rc)        \
    {                         \
        printf(__VA_ARGS__);  \
        while (1)             \
        {                     \
            sleep_ms(100);    \
        };                    \
    }

/**
 * @brief Callback for watermark is set.
 * Read entire FIFO to empty and reset the watermark interrupt.
 */
void cb_watermark()
{
    uint8_t fifo_size;
    if (ADXL345_OK == adxl345_get_fifo_size(&fifo_size))
    {
        for (uint8_t i = 0; i < fifo_size; i++)
        {
            adxl345_value_t value;
            if (ADXL345_OK == adxl345_read(&value))
            {
                printf("X: %d Y: %d Z: %d\n", value.x, value.y, value.z);
            }
        }
    }
}

/**
 * @brief Callback if data is ready.
 * We need to read the data, otherwise the interrupt does not reset and all interrupts stop to work.
 */
void cb_data_ready()
{
    adxl345_value_t value;
    if (ADXL345_OK == adxl345_read(&value))
    {
        printf("X: %d Y: %d Z: %d; combined: %f\n", value.x, value.y, value.z, adxl345_force_vector(&value));
    }
}

/**
 * @brief Callback if we are in free fall
 */
void cb_free_fall()
{
    printf("We are in free fall!!!!!\n");
}

/**
 * @brief Callback if we have tapped the device once
 */
void cb_single_tap()
{
    printf("We have tapped the device once!!!!!\n");
}

/**
 * @brief Callback if we are actively moving
 */
void cb_active()
{
    printf("We have active moving!!!!!\n");
}

/**
 * @brief Callback if we have are in stationary mode
 */
void cb_inactive()
{
    printf("We have back in stationary mode!!!!!\n");
}

int main()
{
    uint8_t rc;
    stdio_init_all();
    sleep_ms(5000);
    printf("Raspberry Pi Pico ADXL345 (I2C) Example Test Application\n");

    rc = adxl345_initialize(ADXL345_ODR_12P5HZ, ADXL345_16G_RANGE, ADXL345_STREAM);
    CHECK_ERROR(rc, "Failed to initialize ADXL345 sensor\n");
    printf("ADXL345 setup done\n");

    /**
     * Advanced handling with interrupts
     * NOTE: There are not all interrupts implemented yet.
     * In addition, some interrupts functions could overlap and destroy the ADXL345 functionality.
     * Please ensure that if you do not acknowledge the received interrupts, ADXL345 will stop sending new one.
     * Acknowledge is done via either reading the INT_SOURCE register, or the DATA regs for DATA_READY, WATERFALL, etc.
     */

    // register some interrupts
    rc = adxl345_enable_interrupt(2);
    CHECK_ERROR(rc, "Failed to enable interrupts\n");

    /**
     * Enable DATA_READY interrupt. This interrupt identifies that that some data is ready.
     * Not sure why, but this data does not increment the FIFO count. In addition, you need to read the DATA, otherwise
     * the ADXL345 will not fire new interrupts.
     */
    // rc = adxl345_register_trigger_data_ready(&cb_data_ready);
    // CHECK_ERROR(rc, "Failed to enable ADXL345 data ready trigger\n");

    /**
     * Enable waterfall interrupt. In this case, we will reive an interrupt when there are equal, more than 10
     * samples in the FIFO. Please ensure to initiate the ADXL345 in FIFO mode.
     */
    // rc = adxl345_register_trigger_watermark(10, &cb_watermark);
    // CHECK_ERROR(rc, "Failed to enable ADXL345 watermark trigger\n");

    /**
     * Enable free fall interrupt.
     */
    // rc = adxl345_register_trigger_free_fall(800, 100, &cb_free_fall);
    // CHECK_ERROR(rc, "Failed to enable ADXL345 free fall trigger\n");

    /**
     * Enable Single Tap interrupt.
     */
    // rc = adxl345_register_trigger_single_tap(2000, 100000UL, &cb_single_tap);
    // CHECK_ERROR(rc, "Failed to enable ADXL345 single tap\n");

    /**
     * Enable active and inactive movement mode. In addition, we set the link bit, which prevents the triggering of
     * further ACTIVE interrupts until the INACTIVE comes. This setup allows to track movements.
     */
    rc = adxl345_register_trigger_active(3000, &cb_active);
    CHECK_ERROR(rc, "Failed to enable ADXL345 active movement detection\n");

    rc = adxl345_register_trigger_inactive(1500, 5000, &cb_inactive);
    CHECK_ERROR(rc, "Failed to enable ADXL345 inactive movement detection\n");

    rc = adxl345_link_movement();
    CHECK_ERROR(rc, "Failed to link ADXL345 active and inactive movement detection\n");

    /**
     * Basic handling, read the accel data and print it
     */
    while (true)
    {
        adxl345_value_t value;
        if (ADXL345_OK == adxl345_read(&value))
        {
            printf("X: %d Y: %d Z: %d; combined: %f\n", value.x, value.y, value.z, adxl345_force_vector(&value));
        }
        sleep_ms(1000);
    }
}