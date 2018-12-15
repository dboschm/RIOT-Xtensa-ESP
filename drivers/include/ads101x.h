/*
 * Copyright (C) 2017 OTA keys S.A.
 *               2018 Acutam Automation, LLC
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup   drivers_ads101x ADS101x/111x ADC device driver
 * @ingroup    drivers_sensors
 * @ingroup    drivers_saul
 * @brief      I2C Analog-to-Digital Converter device driver
 *
 * This driver works with ADS1013-5 and ADS1113-5.
 *
 * This driver provides @ref drivers_saul capabilities.
 * @{
 *
 * @file
 * @brief      ADS101x/111x ADC device driver
 *
 * ADC and alert functionality are separated into two devices to
 * prevent wasteful representations on muxed devices.
 *
 * @author     Vincent Dupont <vincent@otakeys.com>
 * @author     Matthew Blue <matthew.blue.neuro@gmail.com>
 */

#ifndef ADS101X_H
#define ADS101X_H

#ifdef __cplusplus
extern "C" {
#endif

#include "periph/i2c.h"
#include "periph/gpio.h"

#if MODULE_EXTEND_ADC || DOXYGEN
#include "periph/adc.h"
#endif

/**
 * @brief   ADS101x/111x default address
 *
 * Address pin tied to: GND (0x48), Vcc (0x49), SDA (0x50), SCL (0x51)
 */
#ifndef ADS101X_I2C_ADDRESS
#define ADS101X_I2C_ADDRESS    (0x48)
#endif

#if MODULE_EXTEND_ADC || DOXYGEN
/**
 * @brief   ADS101x/111x device variants required by the ADC extension API
 *
 * ADS101x/111x device variant is required by the ADC extension API to have
 * information about the number of channels and the ADC resolution. The
 * ADS101x/111x device variant has to be specified in configuration
 * parameters.
 */
typedef enum {
    ADS101X_DEV_UNKOWN = 0, /**< unknown device variant */
    ADS101X_DEV_ADS1013,    /**< 1 channel, 12-bit ADC */
    ADS101X_DEV_ADS1014,    /**< 1 channel, 12-bit ADC */
    ADS101X_DEV_ADS1015,    /**< 4 channel, 12-bit ADC */
    ADS101X_DEV_ADS1113,    /**< 1 channel, 16-bit ADC */
    ADS101X_DEV_ADS1114,    /**< 1 channel, 16-bit ADC */
    ADS101X_DEV_ADS1115,    /**< 4 channel, 16-bit ADC */
}  ads101x_device_t;
#endif

/**
 * @brief   Named return values
 */
enum {
    ADS101X_OK          =  0,       /**< everything was fine */
    ADS101X_NOI2C       = -1,       /**< I2C communication failed */
    ADS101X_NODEV       = -2,       /**< no ADS101X device found on the bus */
    ADS101X_NODATA      = -3        /**< no data available */
};

/**
 * @brief   ADS101x/111x params
 */
typedef struct ads101x_params {
    i2c_t i2c;              /**< i2c device */
    uint8_t addr;           /**< i2c address */
    uint8_t mux_gain;       /**< Mux and gain boolean settings */
#if MODULE_EXTEND_ADC || DOXYGEN
    ads101x_device_t device;  /**< ADS101x/111x device variant */
#endif
} ads101x_params_t;

/**
 * @brief   ADS101x/111x alert params
 */
typedef struct ads101x_alert_params {
    i2c_t i2c;              /**< i2c device */
    uint8_t addr;           /**< i2c address */
    gpio_t alert_pin;       /**< alert pin (GPIO_UNDEF if not connected) */
    int16_t low_limit;      /**< alert low value */
    int16_t high_limit;     /**< alert high value */
} ads101x_alert_params_t;

/**
 * @brief   ADS101x/111x device descriptor
 */
typedef struct ads101x {
    ads101x_params_t params;    /**< device driver configuration */
} ads101x_t;

/**
 * @brief   ADS101x/111x alert callback
 */
typedef void (*ads101x_alert_cb_t)(void *);

/**
 * @brief   ADS101x/111x alert device descriptor
 */
typedef struct ads101x_alert {
    ads101x_alert_params_t params;    /**< device driver configuration */
    ads101x_alert_cb_t cb;            /**< alert callback */
    void *arg;                        /**< alert callback param */
} ads101x_alert_t;

/**
 * @brief   Initialize an ADS101x/111x ADC device (ADC only)
 *
 * @param[in,out] dev  device descriptor
 * @param[in] params   device configuration
 *
 * @return zero on successful initialization, non zero on error
 */
int ads101x_init(ads101x_t *dev, const ads101x_params_t *params);

/**
 * @brief   Initialize an ADS101x/111x alert device
 *
 * @param[in,out] dev  device descriptor
 * @param[in] params   device configuration
 *
 * @return zero on successful initialization, non zero on error
 */
int ads101x_alert_init(ads101x_alert_t *dev,
                       const ads101x_alert_params_t *params);

/**
 * @brief   Set mux and gain
 *
 * Mux settings have no effect on ADS1013-4 and ADS1113-4.
 * Gain settings have no effect on ADS1013 and ADS1113.
 *
 * @param[in] dev       device descriptor
 * @param[in] mux_gain  mux and gain boolean values
 *
 * @return zero on successful read, non zero on error
 */
int ads101x_set_mux_gain(const ads101x_t *dev, uint8_t mux_gain);

/**
 * @brief   Read a raw ADC value
 *
 * @param[in] dev   device descriptor
 * @param[out] raw  read value
 *
 * @return zero on successful read, non zero on error
 */
int ads101x_read_raw(const ads101x_t *dev, int16_t *raw);

/**
 * @brief   Enable alert interrupt
 *
 * Alert settings have no effect on ADS1013 and ADS1113.
 *
 * @param[in] dev   device descriptor
 * @param[in] cb    callback called when the alert fires
 * @param[in] arg   callback argument
 *
 * @return zero on success, non zero on error
 */
int ads101x_enable_alert(ads101x_alert_t *dev,
                         ads101x_alert_cb_t cb, void *arg);

/**
 * @brief   Set the alert parameters
 *
 * Alert settings have no effect on ADS1013 and ADS1113.
 *
 * @param[in,out] dev      device descriptor
 * @param[in] low_limit    alert low limit
 * @param[in] high_limit   alert high limit
 *
 * @return zero on success, non zero on error
 */
int ads101x_set_alert_parameters(const ads101x_alert_t *dev,
                                 int16_t low_limit, int16_t high_limit);

#if MODULE_EXTEND_ADC || DOXYGEN
/**
 * @name    ADC extension API functions
 *
 * These functions are defined for compatibility with the ADC extension API.
 * They are required to use ADS101x/111x devices as ADC extensions.
 *
 * @{
 */

/**
 * @brief   #adc_init callback function for the ADC extension API
 *
 * @note This function is for use with the ADC Extension API only.
 *
 * This function is the implementation of of #adc_init function of the
 * ADC extension API. Since ADS101x/ADS111x channels have not to be
 * initialized, this function will not do anything.
 * 
 * @param[in] dev   device descriptor
 * @param[in] chn   channel to initialize
 *
 * @return  0 on success
 * @return  -1 on invalid ADC channel
 *
 * @see #adc_init
 */
int ads101x_adc_init(const ads101x_t *dev, adc_t chn);

/**
 * @brief   #adc_sample callback function for the ADC extension API
 *
 * @note This function is for use with the ADC Extension API only.
 *
 * The function selects the according ADS101x/ADS111x channel, starts a single
 * conversion and returns the result of the conversion or -1 in case of error.
 * The gain that is used for the conversion is defined in the
 * ads101x_params_t::mux_gain parameter. The conversion takes about 8 ms.
 *
 * While the ADC peripheral/extension API supports only positive sample values
 * in the range of 0 to 2^n-1, ADS101x/ADS111x devices provide raw data
 * as a two's complement in the range of -2^(n-1) to +2^(n-1)-1, where n is
 * the resolution. Therefore, raw data are converted to values ranging from
 * 0 to 2^n-1. Sample values then corresponds to the following raw data values:
 *
 * <center>
 * Raw data   | Sample value | Example for a FSR of +-2.048V with #ADC_RES_12BIT
 * -----------|--------------|------------------------------------------------------
 * -2^(n-1)   | 0            | -2.048 V = -FSR
 * 0          | 2^(n-1)      | 0 V 
 * +2^(n-1)-1 | 2^n - 1      | +2.047 V
 * </center><br>
 * 
 * @note ADS111x devices provide a resolution of 16-bit. Since only positive
 * sample values can be used, this resolution is not available on platforms
 * with 16-bit integer size. The function returns -1 if #ADC_RES_16BIT is used
 * on such platforms.
 *
 * @param[in] dev   device descriptor
 * @param[in] chn   channel to sample
 * @param[in] res   resolution to use for conversion
 *
 * @return          sampled value on success
 * @return          -1 if resolution is not applicable
 *
 * @see #adc_sample
 */
int ads101x_adc_sample(const ads101x_t *dev, adc_t chn, adc_res_t res);

/**
 * @brief   #adc_channels callback function for the ADC extension API
 *
 * @note This function is for use with the ADC Extension API only.
 *
 * Returns the number of channels of the ADS101x/ADS111x device.
 *
 * @param[in] dev   device descriptor
 * @return number of channels
 *
 * @see #adc_channels
 */
unsigned int ads101x_adc_channels(const ads101x_t *dev);

/** @} */
#endif /* MODULE_EXTEND_ADC || DOXYGEN */

#ifdef __cplusplus
}
#endif

#endif /* ADS101X_H */
/** @} */
