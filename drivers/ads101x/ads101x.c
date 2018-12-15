/*
 * Copyright (C) 2017 OTA keys S.A.
 *               2018 Acutam Automation, LLC
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_ads101x
 * @{
 *
 * @file
 * @brief       ADS101x/111x ADC device driver
 *
 * @author      Vincent Dupont <vincent@otakeys.com>
 * @author      Matthew Blue <matthew.blue.neuro@gmail.com>
 * @}
 */

#include "assert.h"
#include "periph/i2c.h"
#include "periph/gpio.h"
#include "xtimer.h"

#include "ads101x.h"
#include "ads101x_params.h"
#include "ads101x_regs.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

#ifndef ADS101X_READ_DELAY
#define ADS101X_READ_DELAY (8 * US_PER_MS)    /* Compatible with 128SPS */
#endif

#define I2C (dev->params.i2c)
#define ADDR (dev->params.addr)

static int _ads101x_init_test(i2c_t i2c, uint8_t addr);

int ads101x_init(ads101x_t *dev, const ads101x_params_t *params)
{
    assert(dev && params);

    dev->params = *params;

    return _ads101x_init_test(I2C, ADDR);
}

int ads101x_alert_init(ads101x_alert_t *dev,
                       const ads101x_alert_params_t *params)
{
    assert(dev && params);

    dev->params = *params;
    dev->cb = NULL;
    dev->arg = NULL;

    /* Set up alerts */
    ads101x_set_alert_parameters(dev, dev->params.low_limit,
                                 dev->params.high_limit);

    return _ads101x_init_test(I2C, ADDR);
}

static int _ads101x_init_test(i2c_t i2c, uint8_t addr)
{
    uint8_t regs[2];

    i2c_acquire(i2c);

    /* Register read test */
    if (i2c_read_regs(i2c, addr, ADS101X_CONF_ADDR, &regs, 2, 0x0) < 0) {
        DEBUG("[ads101x] init - error: unable to read reg %x\n",
              ADS101X_CONF_ADDR);
        i2c_release(i2c);
        return ADS101X_NODEV;
    }

    regs[1] = (regs[1] & ~ADS101X_DATAR_MASK) | ADS101X_DATAR_3300;

    /* Register write test */
    if (i2c_write_regs(i2c, addr, ADS101X_CONF_ADDR, &regs, 2, 0x0) < 0) {
        DEBUG("[ads101x] init - error: unable to write reg %x\n",
              ADS101X_CONF_ADDR);
        i2c_release(i2c);
        return ADS101X_NODEV;
    }

    i2c_read_regs(i2c, addr, ADS101X_CONF_ADDR, &regs, 2, 0x0);

    i2c_release(i2c);

    /* Write should have actually written the register */
    if ((regs[1] & ADS101X_DATAR_MASK) != ADS101X_DATAR_3300) {
        DEBUG("[ads101x] init - error: unable to set reg (reg=%x)\n", regs[1]);
        return ADS101X_NODEV;
    }

    return ADS101X_OK;
}

int ads101x_set_mux_gain(const ads101x_t *dev, uint8_t mux_gain)
{
    uint8_t regs[2];

    i2c_acquire(I2C);

    i2c_read_regs(I2C, ADDR, ADS101X_CONF_ADDR, &regs, 2, 0x0);

    /* Zero mux and gain */
    regs[0] &= ~ADS101X_MUX_MASK;
    regs[0] &= ~ADS101X_PGA_MASK;

    /* Write mux and gain */
    regs[0] |= mux_gain;

    i2c_write_regs(I2C, ADDR, ADS101X_CONF_ADDR, &regs, 2, 0x0);

    i2c_release(I2C);

    return ADS101X_OK;
}

int ads101x_read_raw(const ads101x_t *dev, int16_t *raw)
{
    uint8_t regs[2];

    i2c_acquire(I2C);

    /* Read control register */
    i2c_read_regs(I2C, ADDR, ADS101X_CONF_ADDR, &regs, 2, 0x0);

    /* Tell the ADC to aquire a single-shot sample */
    regs[0] |= ADS101X_CONF_OS_CONV;
    i2c_write_regs(I2C, ADDR, ADS101X_CONF_ADDR, &regs, 2, 0x0);

    /* Wait for the sample to be aquired */
    xtimer_usleep(ADS101X_READ_DELAY);

    /* Read the sample */
    if (i2c_read_regs(I2C, ADDR, ADS101X_CONV_RES_ADDR, &regs, 2, 0x0) < 0) {
        i2c_release(I2C);
        return ADS101X_NODATA;
    }

    i2c_release(I2C);

    /* If all okay, change raw value */
    *raw = (int16_t)(regs[0] << 8) | (int16_t)(regs[1]);

    return ADS101X_OK;
}

int ads101x_enable_alert(ads101x_alert_t *dev,
                         ads101x_alert_cb_t cb, void *arg)
{
    uint8_t regs[2];

    if (dev->params.alert_pin == GPIO_UNDEF) {
        return ADS101X_OK;
    }

    /* Read control register */
    i2c_acquire(I2C);
    i2c_read_regs(I2C, ADDR, ADS101X_CONF_ADDR, &regs, 2, 0x0);

    /* Enable alert comparator */
    regs[1] &= ~ADS101X_CONF_COMP_DIS;
    i2c_write_regs(I2C, ADDR, ADS101X_CONF_ADDR, &regs, 2, 0x0);

    i2c_release(I2C);

    /* Enable interrupt */
    dev->arg = arg;
    dev->cb = cb;
    gpio_init_int(dev->params.alert_pin, GPIO_IN, GPIO_FALLING, cb, arg);

    return ADS101X_OK;
}

int ads101x_set_alert_parameters(const ads101x_alert_t *dev,
                                 int16_t low_limit, int16_t high_limit)
{
    uint8_t regs[2];

    i2c_acquire(I2C);

    /* Set up low_limit */
    regs[0] = (uint8_t)(low_limit >> 8);
    regs[1] = (uint8_t)low_limit;
    i2c_write_regs(I2C, ADDR, ADS101X_LOW_LIMIT_ADDR, &regs, 2, 0x0);

    /* Set up high_limit */
    regs[0] = (uint8_t)(high_limit >> 8);
    regs[1] = (uint8_t)high_limit;
    i2c_write_regs(I2C, ADDR, ADS101X_HIGH_LIMIT_ADDR, &regs, 2, 0x0);

    /* Read control register */
    i2c_read_regs(I2C, ADDR, ADS101X_CONF_ADDR, &regs, 2, 0x0);

    /* Set up window mode */
    if (low_limit != 0) {
        /* Enable window mode */
        regs[1] |= ADS101X_CONF_COMP_MODE_WIND;
    }
    else {
        /* Disable window mode */
        regs[1] &= ~ADS101X_CONF_COMP_MODE_WIND;
    }
    i2c_write_regs(I2C, ADDR, ADS101X_CONF_ADDR, &regs, 2, 0x0);

    i2c_release(I2C);

    return ADS101X_OK;
}

#if MODULE_EXTEND_ADC

int ads101x_adc_init(const ads101x_t *dev, adc_t chn)
{
    DEBUG("%s: dev %p, chn %u\n", __func__, dev, chn);

    assert(dev != NULL);
    assert(chn < ads101x_adc_channels(dev));

    return 0;
}

int ads101x_adc_sample(const ads101x_t *dev, adc_t chn, adc_res_t res)
{
    DEBUG("%s: dev %p, chn %u, res %u\n", __func__, dev, chn, res);

    assert(dev != NULL);
    assert(chn < ads101x_adc_channels(dev));

    /* 16-bit res cannot be supported by ADC API on platforms with 16 bit int */
    if (res == ADC_RES_16BIT && sizeof(int) == 2) {
        return -1;
    }

    /* use only the gain from parameters, mux is set accroding to given chn */
    uint8_t mux_gain = dev->params.mux_gain & ADS101X_PGA_MASK;
    
    switch (chn) {
        case 0: mux_gain |= ADS101X_AIN0_SINGM; break;
        case 1: mux_gain |= ADS101X_AIN1_SINGM; break;
        case 2: mux_gain |= ADS101X_AIN2_SINGM; break;
        case 3: mux_gain |= ADS101X_AIN3_SINGM; break;
        default: return -1;
    }

    /* set the channel for next conversion */
    if (ads101x_set_mux_gain(dev, mux_gain) != ADS101X_OK) {
        return -1;
    }

    int16_t raw;
    /* execute one conversion for the selected channel */
    if (ads101x_read_raw(dev, &raw) != 0) {
        return -1;
    }

    /**
     * since ADS101X/111X ADC returns the data as two's complement and
     * ADC peripheral/extension API only support values in the range from
     * 0 to 2^n-1, we have to transform the results to unsigned int values
     * in this range.
     */
    /* two's complement to unsigned int in range of 0 ... 2^n-1 */
    uint16_t tmp = raw + 0x8000;

    /* values are always 16-bit left aligned independent on resolution */
    switch (res) {
        case ADC_RES_6BIT:  return tmp >> 10;
        case ADC_RES_8BIT:  return tmp >> 8;
        case ADC_RES_10BIT: return tmp >> 6;
        case ADC_RES_12BIT: return tmp >> 4;

        case ADC_RES_14BIT: if (dev->params.device == ADS101X_DEV_ADS1113 ||
                                dev->params.device == ADS101X_DEV_ADS1114 ||
                                dev->params.device == ADS101X_DEV_ADS1115) {
                                return tmp >> 2;
                            }
                            else {
                                return -1;
                            }

        case ADC_RES_16BIT: if (dev->params.device == ADS101X_DEV_ADS1113 ||
                                dev->params.device == ADS101X_DEV_ADS1114 ||
                                dev->params.device == ADS101X_DEV_ADS1115) {
                                return tmp;
                            }
                            else {
                                return -1;
                            }
        default: return -1;
    }
}

unsigned int ads101x_adc_channels(const ads101x_t *dev)
{
    DEBUG("%s: dev %p\n", __func__, dev);

    assert(dev != NULL);
    assert(dev->params.device != ADS101X_DEV_UNKOWN);

    switch(dev->params.device) {
        case ADS101X_DEV_ADS1015:
        case ADS101X_DEV_ADS1115: return 4;
        default: return 1;
    }
}

#endif /* MODULE_EXTEND_ADC */
