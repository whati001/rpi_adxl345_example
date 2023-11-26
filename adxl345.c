#include "adxl345.h"
#include "adxl345_bus.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include <math.h>
#include <string.h>

#define CHECK_ERROR(_rc, ...) \
    if (ADXL345_OK != _rc)    \
    {                         \
        printf(__VA_ARGS__);  \
        return _rc;           \
    }

#define CHECK_ERROR_VOID(_rc, ...) \
    if (ADXL345_OK != _rc)         \
    {                              \
        printf(__VA_ARGS__);       \
        return;                    \
    }

static struct
{
    bool initialized;
    adxl345_odr_t frequency;
    adxl345_range_t range;
    adxl345_fifo_t fifo;

    uint8_t gpio_interrupt;
    adxl345_trigger_cb cb_data_ready;
    adxl345_trigger_cb cb_watermark;
    adxl345_trigger_cb cb_free_fall;
    adxl345_trigger_cb cb_single_tap;
    adxl345_trigger_cb cb_active;
    adxl345_trigger_cb cb_inactive;

    // TODO: implemented them
    bool full_resolution;
} adxl345_inst;

static void interrupt_handler(uint gpio, uint32_t event_mask)
{
    int rc;
    uint8_t trigger;
    rc = adxl345_get_trigger_source(&trigger);
    CHECK_ERROR_VOID(rc, "Failed to get ADXL345 trigger source\n");
    printf("interrupt! reasons: 0x%x\n", trigger);

    if ((trigger & ADXL345_INT_SOURCE_DATA_READY_BIT) == ADXL345_INT_SOURCE_DATA_READY_BIT)
    {
        if (NULL != adxl345_inst.cb_data_ready)
        {
            adxl345_inst.cb_data_ready();
        }
    }

    if ((trigger & ADXL345_INT_SOURCE_WATERMARK_BIT) == ADXL345_INT_SOURCE_WATERMARK_BIT)
    {
        if (NULL != adxl345_inst.cb_watermark)
        {
            adxl345_inst.cb_watermark();
        }
    }

    if ((trigger & ADXL345_INT_SOURCE_FREE_FALL_BIT) == ADXL345_INT_SOURCE_FREE_FALL_BIT)
    {
        if (NULL != adxl345_inst.cb_free_fall)
        {
            adxl345_inst.cb_free_fall();
        }
    }

    if ((trigger & ADXL345_INT_SOURCE_SINGLE_TAP_BIT) == ADXL345_INT_SOURCE_SINGLE_TAP_BIT)
    {
        if (NULL != adxl345_inst.cb_single_tap)
        {
            adxl345_inst.cb_single_tap();
        }
    }

    if ((trigger & ADXL345_INT_SOURCE_ACTIVE_BIT) == ADXL345_INT_SOURCE_ACTIVE_BIT)
    {
        if (NULL != adxl345_inst.cb_active)
        {
            adxl345_inst.cb_active();
        }
    }

    if ((trigger & ADXL345_INT_SOURCE_INACTIVE_BIT) == ADXL345_INT_SOURCE_INACTIVE_BIT)
    {
        if (NULL != adxl345_inst.cb_inactive)
        {
            adxl345_inst.cb_inactive();
        }
    }
}

int adxl345_initialize(adxl345_odr_t frequency, adxl345_range_t range, adxl345_fifo_t fifo)
{
    int rc;
    memset(&adxl345_inst, 0, sizeof(adxl345_inst));
    // setup bus
    rc = bus_initialize();
    CHECK_ERROR(rc, "Failed to initialize ADXL345 BUS\n");

    // read the part id
    uint8_t part_id;
    rc = bus_read_reg(ADXL345_DEVICE_ID_REG, &part_id);
    CHECK_ERROR(rc, "Failed to read ADXL345 part id\n");
    if (ADXL345_DEVICE_ID_VALUE != part_id)
    {
        printf("Received invalid partid: %d\n", part_id);
        return ADXL345_ERR;
    }

    // set the range
    rc = bus_write_reg_mask(ADXL345_DATA_FORMAT_REG, ADXL345_DATA_FORMAT_REG_RANGE_MSK, range);
    CHECK_ERROR(rc, "Failed to set ADXL345 range\n");
    adxl345_inst.range = range;

    // set the frequency
    rc = bus_write_reg_mask(ADXL345_BW_RATE_REG, ADXL345_BW_RATE_RATE_MSK, frequency);
    CHECK_ERROR(rc, "Failed to set ADXL345 frequency\n");
    adxl345_inst.frequency = frequency;

    // set the FIFO mode
    rc = bus_write_reg_mask(ADXL345_FIFO_CTL_REG, ADXL345_FIFO_CTL_REG_FIFO_MSK, fifo);
    CHECK_ERROR(rc, "Failed to set ADXL345 fifo\n");
    adxl345_inst.fifo = fifo;

    // enable measurement
    rc = bus_write_reg_bit(ADXL345_POWER_CTL_REG, ADXL345_POWER_CTL_REG_MEASURE_BIT);
    CHECK_ERROR(rc, "Failed to enable ADXL345 measurement\n");

    adxl345_inst.initialized = true;
    return ADXL345_OK;
}

int adxl345_read_raw(adxl345_value_t *value)
{
    int rc;
    uint8_t axis_data[6];
    adxl345_value_t tmp;

    rc = bus_read(ADXL345_X_AXIS_DATA_0_REG, axis_data, sizeof(axis_data));
    CHECK_ERROR(rc, "Failed to read ADXL345 data\n");

    value->x = axis_data[0] | (axis_data[1] << 8);
    value->y = axis_data[2] | (axis_data[3] << 8);
    value->z = axis_data[4] | (axis_data[5] << 8);

    return ADXL345_OK;
}

int adxl345_read(adxl345_value_t *value)
{
    int rc;
    adxl345_value_t raw_value;
    rc = adxl345_read_raw(&raw_value);
    CHECK_ERROR(rc, "Failed to read raw ADXL345 data");

    // get scale factor for current set resolution and range mode
    // TODO: add full resolution mode -> 4mg/LSB
    float scale_factor = 0;
    switch (adxl345_inst.range)
    {
    case ADXL345_2G_RANGE:
        scale_factor = 4;
        break;
    case ADXL345_4G_RANGE:
        scale_factor = 8;
        break;
    case ADXL345_8G_RANGE:
        scale_factor = 16;
        break;
    case ADXL345_16G_RANGE:
    default:
        scale_factor = 32;
        break;
    }

    value->x = ((float)raw_value.x * scale_factor);
    value->y = ((float)raw_value.y * scale_factor);
    value->z = ((float)raw_value.z * scale_factor);

    return ADXL345_OK;
}

int adxl345_enable_interrupt(uint8_t gpio_pin)
{
    gpio_set_irq_enabled_with_callback(gpio_pin, GPIO_IRQ_EDGE_RISE, true, &interrupt_handler);
    return ADXL345_OK;
}

int adxl345_get_trigger_source(uint8_t *trigger)
{
    int rc;
    uint8_t value;

    rc = bus_read_reg(ADXL345_INT_SOURCE_REG, &value);
    CHECK_ERROR(rc, "Failed to read ADXL345 trigger source\n");

    *trigger = value;
    return ADXL345_OK;
}

int adxl345_register_trigger_free_fall(uint16_t threshold, uint16_t time_ms, adxl345_trigger_cb cb)
{
    int rc;

    rc = bus_write_reg(ADXL345_THRESH_FF_REG, (threshold / 62.5));
    CHECK_ERROR(rc, "Failed to set free fall threshold value\n");

    rc = bus_write_reg(ADXL345_TIME_FF_REG, (time_ms / 5));
    CHECK_ERROR(rc, "Failed to set free fall time value\n");

    // TODO: add logic to define witch INT pin should be used
    adxl345_inst.cb_free_fall = cb;
    return bus_write_reg_bit(ADXL345_INT_ENABLE_REG, ADXL345_INT_ENABLE_FREE_FALL_BIT);
}

int adxl345_register_trigger_data_ready(adxl345_trigger_cb cb)
{
    // TODO: add logic to define witch INT pin should be used
    adxl345_inst.cb_data_ready = cb;
    return bus_write_reg_bit(ADXL345_INT_ENABLE_REG, ADXL345_INT_ENABLE_DATA_READY_BIT);
}

int adxl345_register_trigger_watermark(uint8_t watermark, adxl345_trigger_cb cb)
{
    int rc;

    rc = bus_write_reg_mask(ADXL345_FIFO_CTL_REG, ADXL345_FIFO_CTL_SAMPLE_MSK, watermark);
    CHECK_ERROR(rc, "Failed to set free fall threshold value\n");

    // TODO: add logic to define witch INT pin should be used
    adxl345_inst.cb_watermark = cb;
    return bus_write_reg_bit(ADXL345_INT_ENABLE_REG, ADXL345_INT_ENABLE_WATERMARK_BIT);
}

int adxl345_register_trigger_single_tap(uint16_t threshold, uint32_t time_us, adxl345_trigger_cb cb)
{
    int rc;

    rc = bus_write_reg(ADXL345_THRESH_TAP_REG, (threshold / 62.5));
    CHECK_ERROR(rc, "Failed to set threshold for single tap detection\n");

    rc = bus_write_reg(ADXL345_DUR_REG, (time_us / 625));
    CHECK_ERROR(rc, "Failed to set time for single tap detection\n");

    // TODO: add logic to select which axis should be enabled -> currently all axis are enabled
    rc = bus_write_reg_mask(ADXL345_TAP_AXES_REG, ADXL345_TAP_AXES_TAP_AXIS_MSK, 0b111);
    CHECK_ERROR(rc, "Failed to enable all axis for single tap detection\n");

    // TODO: add logic to define witch INT pin should be used
    adxl345_inst.cb_single_tap = cb;
    return bus_write_reg_bit(ADXL345_INT_ENABLE_REG, ADXL345_INT_ENABLE_SINGLE_TAP_BIT);
}

int adxl345_register_trigger_active(uint16_t threshold, adxl345_trigger_cb cb)
{
    int rc;

    rc = bus_write_reg(ADXL345_THRESH_ACT_REG, (threshold / 62.5));
    CHECK_ERROR(rc, "Failed to set threshold for active movement detection\n");

    // TODO: add logic to select which axis should be enabled -> currently all axis are enabled
    rc = bus_write_reg_mask(ADXL345_ACT_INACT_CTL_REG, ADXL345_ACT_INACT_CTL_ACT_AXIS_MSK, ADXL345_ACT_INACT_CTL_ACT_AXIS_MSK);
    CHECK_ERROR(rc, "Failed to enable all axis for active movement detection\n");

    // TODO: add logic to define witch INT pin should be used
    adxl345_inst.cb_active = cb;
    return bus_write_reg_bit(ADXL345_INT_ENABLE_REG, ADXL345_INT_ENABLE_ACTIVE_BIT);
}

int adxl345_register_trigger_inactive(uint16_t threshold, uint32_t time_ms, adxl345_trigger_cb cb)
{
    int rc;

    rc = bus_write_reg(ADXL345_THRESH_INACT_REG, (threshold / 62.5));
    CHECK_ERROR(rc, "Failed to set threshold for inactive movement detection\n");

    rc = bus_write_reg(ADXL345_TIME_INACT_REG, (time_ms / 1000));
    CHECK_ERROR(rc, "Failed to set threshold for inactive movement detection\n");

    // TODO: add logic to select which axis should be enabled -> currently all axis are enabled
    rc = bus_write_reg_mask(ADXL345_ACT_INACT_CTL_REG, ADXL345_ACT_INACT_CTL_INACT_AXIS_MSK, ADXL345_ACT_INACT_CTL_INACT_AXIS_MSK);
    CHECK_ERROR(rc, "Failed to enable all axis for inactive movement detection\n");

    // TODO: add logic to define witch INT pin should be used
    adxl345_inst.cb_inactive = cb;
    return bus_write_reg_bit(ADXL345_INT_ENABLE_REG, ADXL345_INT_ENABLE_INACTIVE_BIT);
}

int adxl345_link_movement()
{
    if (NULL == adxl345_inst.cb_active)
    {
        CHECK_ERROR(ADXL345_ERR, "Failed to link active and inactive movement, no active cb set\n");
    }
    if (NULL == adxl345_inst.cb_inactive)
    {
        CHECK_ERROR(ADXL345_ERR, "Failed to link active and inactive movement, no inactive cb set\n");
    }

    return bus_write_reg_bit(ADXL345_POWER_CTL_REG, ADXL345_POWER_CTL_LINK_BIT);
}

int adxl345_get_fifo_size(uint8_t *size)
{
    int rc;
    uint8_t tmp;
    rc = bus_read_reg(ADXL345_FIFO_STATUS, &tmp);
    CHECK_ERROR(rc, "Failed to read FIFO size\n");

    *size = (tmp & 0b111111);
    return rc;
}