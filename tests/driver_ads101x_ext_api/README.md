# Test for TI ADS101x/ADS111x as ADC extension

## Overview

This test application demonstrates the use of the ADS101x/ADS111x devices
as ADC extension. It can be used with up to two ADS101x/ADS111x devices to
test each channel with shell commands using the ADC extension API.

## Usage

The shell commands `sample` allow to read a single ADS101x/ADS111x channel
with a specific resolution. The `loop` command can be used to read all
channels of all ADS101x/ADS111x devices and with all resolutions in an
infinite loop with a half-second period.

The parameters for the first ADS101x/ADS111x device are reused from the
`ads101x_params.h` file. If only one ADS101x/ADS111x device is used, it is
sufficient to compile the test with:

```
make flash -C tests/driver_ads101x_ext_api BOARD=...
```

If a second device is to be used, the address and variant of the device
have to be specified by `ADS101X_PARAM_ADDR_2` and `ADS101X_PARAM_DEV_2`,
for example at the make command line:

```
CFLAGS="-DADS101X_PARAM_ADDR_2=\(0x49\) -DADS101X_PARAM_DEV_2=\(ADS101X_DEV_ADS1013\)" \
make flash -C tests/driver_ads101x_ext_api BOARD=...
```

The gain and I2C bus parameters are the same as for the first device as
defined in the `ads101x_params.h` file.

**Please note:** ADS1115 devices can also be used to test all other device
variants. That is, the device command shown as example above can also be used to
test the ADC extension API with two ADS1115 devices.
