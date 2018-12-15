/*
 * Copyright (C) 2018 Gunar Schorcht
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     tests
 *
 * @{
 *
 * @file
 * @brief       ADC extension for ADS101x/ADS111x ADC with I2C interface
 *
 * @author      Gunar Schorcht <gunar@schorcht.net>
 *
 * @}
 */

#ifndef ADC_EXT_CONF_H
#define ADC_EXT_CONF_H

#include <stddef.h>

#include "extend/adc.h"
#include "ads101x_params.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Reference to ads101x device driver struct
 */
extern const adc_ext_driver_t ads101x_extend_adc_driver;
/** @} */

/**
 * @brief   References to the ads101x devices
 */
extern ads101x_t ads101x_dev[];

/**
 * @brief   DAC extension list of ads101x devices
 */
static const adc_ext_t adc_ext_list[] =
{
    {
        .driver = &ads101x_extend_adc_driver,
        .dev = (void *)&ads101x_dev[0],
    },
#if defined(ADS101X_PARAM_ADDR_2) && defined(ADS101X_PARAM_DEV_2)
    {
        .driver = &ads101x_extend_adc_driver,
        .dev = (void *)&ads101x_dev[1],
    },
#endif
};

#ifdef __cplusplus
}
#endif

#endif /* ADC_EXT_CONF_H */
/** @} */
