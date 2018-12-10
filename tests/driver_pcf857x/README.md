# Texas Instruments PCF857X I2C I/O expanders test application

## Overview

This test appliation demonstrates the usage of the PCF857X driver interface
and can be used to test each PCF857X expander I/O pin with shell commands.

The application bases on the test application for GPIO peripheral drivers
which is under following copyright:

     Copyright (C) 2014,2017 Freie Universität Berlin
     @author Hauke Petersen <hauke.petersen@fu-berlin.de>

## Usage

To use the test application, compile it with one or more of the pseudomodules
`pcf8574`, `pcf8574a` or `pcf8575` to enable the driver for your
expander modules. Please check the default configuration parameters in
`$(RIOTBASE)/drivers/pcf857x/include/pcf857x_params.h` and adopt them
if necessary.

     USEMODULE=pcf8575 make -C tests/driver_pcf857x BOARD=...

**Please note:** When no pseudomodule is given, `pcf8575` is used by default.

The usage of the PCF857X low-active open-drain interrupt signal is
recommended to be able to use external interrupts with the expander I/O
pins. Add module `periph_gpio_irq` for this purpose and define the
MCU interrupt pin by parameter `PCF857X_PARAM_INT_PIN`, e.g.

     CFLAGS="-DPCF857X_PARAM_INT_PIN=\(GPIO\(0,6\)\)" \
     USEMODULE="pcf8575 periph_gpio_irq" make -C tests/driver_pcf857x BOARD=...
