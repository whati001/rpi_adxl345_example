
#include "adxl345_bus.h"
#include "adxl345.h"
#include <stdio.h>
#include "pico/stdlib.h"

#if defined(ADXL345_I2C_BUS)

#include "hardware/i2c.h"

#define I2C_INSTANCE i2c_default
/**
 * @brief Use default 0 I2C instance with the pinout as specified here:
 * https://www.raspberrypi.com/documentation/microcontrollers/raspberry-pi-pico.html
 *
 * SDA PIN: GPIO04 -> PIN 6
 * SCL PIN: GPIO05 -> PIN 7
 */
int bus_initialize()
{
    i2c_init(I2C_INSTANCE, 100 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    return BUS_OK;
}

int bus_write_reg(uint8_t reg, uint8_t value)
{
    int rc;
    uint8_t buf[2] = {reg, value};
    rc = i2c_write_blocking(I2C_INSTANCE, ADXL345_I2C_ADDRESS, buf, sizeof(buf), false);
    if (PICO_ERROR_GENERIC == rc)
    {
        return BUS_ERR;
    }
    return BUS_OK;
}

int bus_write_reg_mask(uint8_t reg, uint8_t mask, uint8_t value)
{
    int rc;
    uint8_t tmp;
    rc = bus_read_reg(reg, &tmp);
    if (BUS_OK != rc)
    {
        return rc;
    }

    tmp &= ~mask;
    tmp |= value;

    return bus_write_reg(reg, tmp);
}

int bus_read(uint8_t reg, uint8_t *value, uint8_t length)
{
    int rc;
    rc = i2c_write_blocking(I2C_INSTANCE, ADXL345_I2C_ADDRESS, &reg, sizeof(reg), true);
    if (PICO_ERROR_GENERIC == rc)
    {
        return BUS_ERR;
    }
    rc = i2c_read_blocking(I2C_INSTANCE, ADXL345_I2C_ADDRESS, value, length, false);
    if (PICO_ERROR_GENERIC == rc)
    {
        return BUS_ERR;
    }

    return BUS_OK;
}

#elif defined(ADXL345_SPI_BUS)
#error "ADXL345 SPI not yet implemented"
#endif