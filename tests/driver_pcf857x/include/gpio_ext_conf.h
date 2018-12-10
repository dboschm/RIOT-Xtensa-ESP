/*
 * Copyright (C) 2018 Gunar Schorcht
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     tests
 * @brief       GPIO extension for Texas Instruments PCF857X I2C I/O expanders
 * @author      Gunar Schorcht <gunar@schorcht.net>
 * @file
 */

#ifndef GPIO_EXT_CONF_H
#define GPIO_EXT_CONF_H

#if MODULE_EXTEND_GPIO

#include <stddef.h>

#include "extend/gpio.h"
#include "pcf857x_params.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Reference to PCF857X device driver struct
 */
extern const gpio_ext_driver_t pcf857x_extend_gpio_driver;
/** @} */

/**
 * @brief   References to the PCF857X devices
 */
extern pcf857x_t pcf857x_dev[];

/**
 * @brief   GPIO extension list of PCF857X expanders
 */
static const gpio_ext_t gpio_ext_list[] =
{
    {
        .driver = &pcf857x_extend_gpio_driver,
        .dev = (void *)&pcf857x_dev[0],
    },
};

#ifdef __cplusplus
}
#endif

#endif /* MODULE_EXTEND_GPIO */

#endif /* GPIO_EXT_CONF_H */
/** @} */
