/**
 * @brief The ADXL345 accelerator sensor supports I2C and SPI.
 * The following sections aims to abstract the underlying protocol stack by providing the following generic functions to the ADXL345 logic.
 */

#ifndef ADXL345_BUS_H
#define ADXL345_BUS_H

#include <stdint.h>

#define BUS_OK 0
#define BUS_ERR -1

/**
 * @brief Initialize bus system
 *
 * @return BUS_OK on success
 * @return BUS_ERR on error
 */
int bus_initialize();

/**
 * @brief Write value to remove ADXL345 register
 *
 * @param[in] reg   - remote ADXL345 register
 * @param[in] value - value to update
 * @return BUS_OK on success
 * @return BUS_ERR on error
 */
int bus_write_reg(uint8_t reg, uint8_t value);

/**
 * @brief Update remote ADXL345 register value by applying mask & value.
 * For example,
 *  remote register: 0b11100000
 *  mask:            0b00000011
 *  value:           0b00000001
 * The remove register will be set to 0b11100001
 *
 * @param[in] reg   - remote ADXL345 register
 * @param[in] mask  - and mask for update value
 * @param[in] value - value to update
 * @return BUS_OK on success
 * @return BUS_ERR on error
 */
int bus_write_reg_mask(uint8_t reg, uint8_t mask, uint8_t value);

/**
 * @brief Update remote ADXL345 register value by setting bit.
 * For example,
 *  remote register: 0b11100000
 *  bit:             0b00000001
 * The remove register will be set to 0b11100001
 *
 * @param[in] reg      - remote ADXL345 register
 * @param[in] bit_mask - bit value to update
 * @return BUS_OK on success
 * @return BUS_ERR on error
 */
inline int bus_write_reg_bit(uint8_t reg, uint8_t bit)
{
    return bus_write_reg_mask(reg, bit, bit);
}

/**
 * @brief Read value from remove ADXL345 register
 *
 * @param[in] reg    - remote ADXL345 register
 * @param[out] value - variable holding the read value
 * @param[in] length - length in bytes to read
 * @return BUS_OK on success
 * @return BUS_ERR on error
 */
int bus_read(uint8_t reg, uint8_t *value, uint8_t length);

/**
 * @brief Read single byte from remove ADXL345 register
 *
 * @param[in] reg    - remote ADXL345 register
 * @param[out] value - variable holding the read byte
 * @return BUS_OK on success
 * @return BUS_ERR on error
 */
inline int bus_read_reg(uint8_t reg, uint8_t *value)
{
    return bus_read(reg, value, 1);
}

#endif