/*
 * Copyright (C) 2018 Gunar Schorcht
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     tests
 * @brief       Test for TI ADS101x/ADS111x as ADC extension
 * @author      Gunar Schorcht <gunar@schorcht.net>
 * @file
 *
 * This test application demonstrates the use of the ADS101x/ADS111x devices
 * as ADC extension. It can be used with up to two ADS101x/ADS111x devices to
 * test each channel with shell commands using the ADC extension API.
 *
 * The shell commands `sample` allow to read a single ADS101x/ADS111x channel
 * with a specific resolution. The `loop` command can be used to read all
 * channels of all ADS101x/ADS111x devices and with all resolutions in an
 * infinite loop with a half-second period.
 *
 * The parameters for the first ADS101x/ADS111x device are reused from the
 * `ads101x_params.h` file. If only one ADS101x/ADS111x device is used, it is
 * sufficient to compile the test with:
 *
 * ```
 * make flash -C tests/driver_ads101x_ext_api BOARD=...
 * ```
 *
 * If a second device is to be used, the address and variant of the device
 * have to be specified by `ADS101X_PARAM_ADDR_2` and `ADS101X_PARAM_DEV_2`,
 * for example at the make command line:
 *
 * ```
 * CFLAGS="-DADS101X_PARAM_ADDR_2=\(0x49\) -DADS101X_PARAM_DEV_2=\(ADS101X_DEV_ADS1013\)" \
 * make flash -C tests/driver_ads101x_ext_api BOARD=...
 * ```
 *
 * The gain and I2C bus parameters are the same as for the first device as
 * defined in the `ads101x_params.h` file.
 *
 * @note ADS1115 devices can also be used to test all other device variants.
 * That is, the device command shown as example above can also be used to
 * test the ADC extension API with two ADS1115 devices.
 */

#ifndef ADS101X_PARAM_ADDR_1
#define ADS101X_PARAM_ADDR_1    (ADS101X_I2C_ADDRESS)
#endif
#ifndef ADS101X_PARAM_DEV_1
#define ADS101X_PARAM_DEV_1     (ADS101X_PARAM_DEVICE)
#endif

#if defined(ADS101X_PARAM_ADDR_2) && defined(ADS101X_PARAM_DEV_2)
#define ADS101X_PARAMS  {                                          \
                            .i2c        = ADS101X_PARAM_I2C,       \
                            .addr       = ADS101X_PARAM_ADDR_1,    \
                            .mux_gain   = ADS101X_PARAM_MUX_GAIN,  \
                            .device     = ADS101X_PARAM_DEV_1      \
                        },                                         \
                        {                                          \
                            .i2c        = ADS101X_PARAM_I2C,       \
                            .addr       = ADS101X_PARAM_ADDR_2,    \
                            .mux_gain   = ADS101X_PARAM_MUX_GAIN,  \
                            .device     = ADS101X_PARAM_DEV_2      \
                        },
#else
#define ADS101X_PARAMS  {                                          \
                            .i2c        = ADS101X_PARAM_I2C,       \
                            .addr       = ADS101X_PARAM_ADDR_1,    \
                            .mux_gain   = ADS101X_PARAM_MUX_GAIN,  \
                            .device     = ADS101X_PARAM_DEV_1      \
                        },
#endif

#include <stdio.h>
#include <stdlib.h>

#include "ads101x.h"
#include "ads101x_params.h"

#include "assert.h"
#include "irq.h"
#include "shell.h"
#include "benchmark.h"

#include "extend/adc.h"
#include "adc_ext_conf.h"

#define SLEEP   (500 * US_PER_MS)
#define BENCH_RUNS_DEFAULT  (10UL * 100)

/* Number of configured ADS101x/ADS111x ADC devices */
#define ADS101X_NUMOF    (sizeof(ads101x_params) / sizeof(ads101x_params[0]))

/* ADS101x/ADS111x devices allocation */
ads101x_t ads101x_dev[ADS101X_NUMOF];

/* ADS101x/ADS111x ADC extension driver definition */
const adc_ext_driver_t ads101x_extend_adc_driver = {
    .init = (adc_ext_init_t)ads101x_adc_init,
    .sample = (adc_ext_sample_t)ads101x_adc_sample,
    .channels = (adc_ext_channels_t)ads101x_adc_channels
};

static int init(int argc, char **argv)
{
    if (argc < 3) {
        printf("usage: %s <dev> <chn>\n", argv[0]);
        return 1;
    }

    int dev = atoi(argv[1]);
    int chn = atoi(argv[2]);

    if (adc_init(ADC_EXT_LINE(dev, chn)) < 0) {
        printf("error: init ADC_EXT_LINE(%i, %02i) failed\n", dev, chn);
        return 1;
    }

    return 0;
}

static int sample(int argc, char **argv)
{
    if (argc < 4) {
        printf("usage: %s <dev> <chn> <res>\n", argv[0]);
        puts("\t\tres   0    6-bit");
        puts("\t\t      1    8-bit");
        puts("\t\t      2   10-bit");
        puts("\t\t      3   12-bit");
        puts("\t\t      4   14-bit");
        puts("\t\t      5   16-bit");
        return 1;
    }

    int sample = adc_sample(ADC_EXT_LINE(atoi(argv[1]), atoi(argv[2])),
                            atoi(argv[3]));
    printf("sample: %d\n", sample);

    return 0;
}

static int channels(int argc, char **argv)
{
    if (argc < 2) {
        printf("usage: %s <dev>\n", argv[0]);
        return 1;
    }

    uint8_t chn = adc_channels(ADC_EXT_LINE(atoi(argv[1]), 0));
    printf("channel_num: %u\n", chn);
    return 0;
}

static int loop(int argc, char **argv)
{
    if (argc < 1) {
        return 1;
    }

    while (1) {
        for (unsigned dev = 0; dev < ADS101X_NUMOF; dev++) {
            for (int res = 0; res <= ADC_RES_16BIT; res++) {
                printf("dev %d, ", dev); 
                switch (res) {
                    case ADC_RES_6BIT:  printf("res  6-bit"); break;
                    case ADC_RES_8BIT:  printf("res  8-bit"); break;
                    case ADC_RES_10BIT: printf("res 10-bit"); break;
                    case ADC_RES_12BIT: printf("res 12-bit"); break;
                    case ADC_RES_14BIT: printf("res 14-bit"); break;
                    case ADC_RES_16BIT: printf("res 16-bit"); break;
                    default: break;
                }
                for (unsigned chn = 0; chn < adc_channels(ADC_EXT_LINE(dev, 0)); chn++) {
                    adc_t line = ADC_EXT_LINE(dev, chn);
                    int sample = adc_sample(line, res);
                    if (sample != -1) {
                        printf(", chn %d: %6d ", chn, sample);
                    }
                    else {
                        printf(", chn %d:      - ", chn);
                    }
                }
                printf("\n");
            }
        }
        xtimer_usleep(SLEEP);
    }
    return 0;
}

static int bench(int argc, char **argv)
{
    if (argc < 4) {
        printf("usage: %s <dev> <chn> <res> [# of runs]\n", argv[0]);
        return 1;
    }

    unsigned long runs = BENCH_RUNS_DEFAULT;
    if (argc > 4) {
        runs = (unsigned long)atol(argv[4]);
    }

    puts("\nADC driver run-time performance benchmark\n");
    adc_t line = ADC_EXT_LINE(atoi(argv[1]), atoi(argv[2]));
    BENCHMARK_FUNC("nop loop", runs, __asm__ volatile("nop"));
    adc_init(line);
    BENCHMARK_FUNC("adc_sample", runs, adc_sample(line, atoi(argv[3])));
    puts("\n --- DONE ---");
    return 0;
}

static const shell_command_t shell_commands[] = {
    { "init", "init one channel of a device", init },
    { "sample", "read one sample from one channel of a device", sample },
    { "loop", "read samples from all channels of all devices in an endless loop", loop },
    { "channels", "number of channels of a device", channels },
    { "bench", "run a set of predefined benchmarks", bench },
    { NULL, NULL, NULL }
};

int main(void)
{
    /*
     * number of ADC extension list entries has correspond to the
     * number of configured ADS101x/ADS111x devices
     */
    assert(ADS101X_NUMOF == (sizeof(adc_ext_list) / sizeof(adc_ext_list[0])));

    puts("ADS101x/ADS111x ADC peripheral driver test\n");
    puts("Initializing ADS101x/ADS111x");

    /* initialize configured ADS101x/ADS111x devices */
    for (unsigned i = 0; i < ADS101X_NUMOF; i++) {
        if (ads101x_init(&ads101x_dev[i], &ads101x_params[i]) != ADS101X_OK) {
            puts("[Failed]");
            return 1;
        }
    }
    puts("[OK]\n");

    puts("In this test, ADC lines are specified by device and channel numbers.\n"
         "NOTE: make sure the values exist! The\n"
         "      behavior for not existing devices/channels is not defined!");

    /* start the shell */
    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);

    return 0;
}
