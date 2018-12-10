/*
 * Copyright (C) 2018 Gunar Schorcht
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     tests
 * @brief       Test application for Texas Instruments PCF857X I2C I/O expanders
 * @author      Gunar Schorcht <gunar@schorcht.net>
 * @file
 *
 * ## Overview
 *
 * This test appliation demonstrates the usage of the PCF857X driver interface
 * and can be used to test each PCF857X expander I/O pin with shell commands.
 *
 * The application bases on the test application for GPIO peripheral drivers
 * which is under following copyright:
 *
 *      Copyright (C) 2014,2017 Freie Universität Berlin
 *      @author Hauke Petersen <hauke.petersen@fu-berlin.de>
 *
 * ## Usage
 *
 * To use the test application, compile it with one or more of the pseudomodules
 * `pcf8574`, `pcf8574a` or `pcf8575` to enable the driver for your
 * expander modules. Please check the default configuration parameters in
 * `$(RIOTBASE)/drivers/pcf857x/include/pcf857x_params.h` and adopt them
 * if necessary.
 *
 *      USEMODULE=pcf8575 make -C tests/driver_pcf857x BOARD=...
 *
 * @note When no pseudomodule is given, `pcf8575` is used by default.
 *
 * The usage of the PCF857X low-active open-drain interrupt signal is
 * recommended to be able to use external interrupts with the expander I/O
 * pins. Add module `periph_gpio_irq` for this purpose and define the
 * MCU interrupt pin by parameter `PCF857X_PARAM_INT_PIN`, e.g.
 *
 *      CFLAGS="-DPCF857X_PARAM_INT_PIN=\(GPIO\(0,6\)\)" \
 *      USEMODULE="pcf8575 periph_gpio_irq" make -C tests/driver_pcf857x BOARD=...
 */

#include <stdio.h>
#include <stdlib.h>

#include "pcf857x.h"
#include "pcf857x_params.h"

#include "irq.h"
#include "shell.h"
#include "benchmark.h"

#if MODULE_EXTEND_GPIO
#include "extend/gpio.h"
#include "gpio_ext_conf.h"
#endif

#define BENCH_RUNS_DEFAULT      (100UL * 100)

/* Number of configured PCF857X I/O expander devices */
#define PCF857X_NUM    (sizeof(pcf857x_params) / sizeof(pcf857x_params[0]))

/* PCF857X devices allocation */
pcf857x_t pcf857x_dev[PCF857X_NUM];

#if MODULE_EXTEND_GPIO
/* PCF857X GPIO extension driver definition */
const gpio_ext_driver_t pcf857x_extend_gpio_driver = {
    .init = (gpio_ext_init_t)pcf857x_gpio_init,
    .init_int = (gpio_ext_init_int_t)pcf857x_gpio_init_int,
    .irq_enable = (gpio_ext_irq_enable_t)pcf857x_gpio_irq_enable,
    .irq_disable = (gpio_ext_irq_disable_t)pcf857x_gpio_irq_disable,
    .read = (gpio_ext_read_t)pcf857x_gpio_read,
    .set = (gpio_ext_set_t)pcf857x_gpio_set,
    .clear = (gpio_ext_clear_t)pcf857x_gpio_clear,
    .toggle = (gpio_ext_toggle_t)pcf857x_gpio_toggle,
    .write = (gpio_ext_write_t)pcf857x_gpio_write,
};
#endif /* MODULE_EXTEND_GPIO */

#ifdef MODULE_PERIPH_GPIO_IRQ
static void cb(void *arg)
{
    printf("INT: external interrupt from pin %i\n", (int)arg);
}
#endif

static int init_pin(int argc, char **argv, gpio_mode_t mode)
{
    if (argc < 3) {
        printf("usage: %s <port> <pin>\n", argv[0]);
        return 1;
    }

    int po = atoi(argv[1]);
    int pi = atoi(argv[2]);

#if MODULE_EXTEND_GPIO
    if (gpio_init(GPIO_EXT_PIN(po, pi), mode) < 0) {
        printf("error: init GPIO_EXT_PIN(%i, %02i) failed\n", po, pi);
#else
    if (pcf857x_gpio_init(&pcf857x_dev[po], PCF857X_GPIO_PIN(0, pi), mode) < 0) {
        printf("error: init PCF857X pin (dev %i, pin %02i) failed\n", po, pi);
#endif
        return 1;
    }

    return 0;
}

static int init_out(int argc, char **argv)
{
    return init_pin(argc, argv, GPIO_OUT);
}

static int init_in(int argc, char **argv)
{
    return init_pin(argc, argv, GPIO_IN);
}

static int init_in_pu(int argc, char **argv)
{
    return init_pin(argc, argv, GPIO_IN_PU);
}

static int init_od(int argc, char **argv)
{
    return init_pin(argc, argv, GPIO_OD);
}

static int init_od_pu(int argc, char **argv)
{
    return init_pin(argc, argv, GPIO_OD_PU);
}

#ifdef MODULE_PERIPH_GPIO_IRQ
static int init_int(int argc, char **argv)
{
    gpio_mode_t mode = GPIO_IN;
    gpio_flank_t flank;
    int fl;

    if (argc < 4) {
        printf("usage: %s <port> <pin> <flank>\n", argv[0]);
        puts("\tflank:\n"
             "\t0: falling\n"
             "\t1: rising\n"
             "\t2: both\n");
        return 1;
    }

    int po = atoi(argv[1]);
    int pi = atoi(argv[2]);

    fl = atoi(argv[3]);
    switch (fl) {
        case 0:
            flank = GPIO_FALLING;
            break;
        case 1:
            flank = GPIO_RISING;
            break;
        case 2:
            flank = GPIO_BOTH;
            break;
        default:
            puts("error: invalid value for active flank");
            return 1;
    }

#if MODULE_EXTEND_GPIO
    if (gpio_init_int(GPIO_EXT_PIN(po, pi), mode, flank, cb, (void *)pi) < 0) {
        printf("error: init_int GPIO_EXT_PIN(%i, %02i) failed\n", po, pi);
#else
    if (pcf857x_gpio_init_int(&pcf857x_dev[po], PCF857X_GPIO_PIN(0, pi),
                              mode, flank, cb, (void *)pi) < 0) {
        printf("error: init_int PCF857X pin (dev %i, pin %02i) failed\n", po, pi);

#endif
        return 1;
    }

    return 0;
}

static int enable_int(int argc, char **argv)
{
    int status;

    if (argc < 4) {
        printf("usage: %s <port> <pin> <status>\n", argv[0]);
        puts("\tstatus:\n"
             "\t0: disable\n"
             "\t1: enable\n");
        return 1;
    }

    int po = atoi(argv[1]);
    int pi = atoi(argv[2]);

    status = atoi(argv[3]);

    switch (status) {
#if MODULE_EXTEND_GPIO
        case 0:
            puts("disabling GPIO interrupt");
            gpio_irq_disable(GPIO_EXT_PIN(po, pi));
            break;
        case 1:
            puts("enabling GPIO interrupt");
            gpio_irq_enable(GPIO_EXT_PIN(po, pi));
            break;
#else /* MODULE_EXTEND_GPIO */
        case 0:
            puts("disabling GPIO interrupt");
            pcf857x_gpio_irq_disable(&pcf857x_dev[po], PCF857X_GPIO_PIN(0, pi));
            break;
        case 1:
            puts("enabling GPIO interrupt");
            pcf857x_gpio_irq_enable(&pcf857x_dev[po], PCF857X_GPIO_PIN(0, pi));
            break;
#endif /* MODULE_EXTEND_GPIO */
        default:
            puts("error: invalid status");
            return 1;
    }

    return 0;
}
#endif /* MODULE_PERIPH_GPIO_IRQ */

static int read(int argc, char **argv)
{
    if (argc < 3) {
        printf("usage: %s <port> <pin>\n", argv[0]);
        return 1;
    }

    int po = atoi(argv[1]);
    int pi = atoi(argv[2]);

#if MODULE_EXTEND_GPIO
    if (gpio_read(GPIO_EXT_PIN(po, pi))) {
        printf("GPIO_EXT_PIN(%i, %02i) is HIGH\n", po, pi);
    }
    else {
        printf("GPIO_EXT_PIN(%i, %02i) is LOW\n", po, pi);
    }
#else
    if (pcf857x_gpio_read(&pcf857x_dev[po], PCF857X_GPIO_PIN(0, pi))) {
        printf("PCF857X pin (dev %i, pin %02i) is HIGH\n", po, pi);
    }
    else {
        printf("PCF857X pin (dev %i, pin %02i) is LOW\n", po, pi);
    }
#endif

    return 0;
}

static int set(int argc, char **argv)
{
    if (argc < 3) {
        printf("usage: %s <port> <pin>\n", argv[0]);
        return 1;
    }

#if MODULE_EXTEND_GPIO
    gpio_set(GPIO_EXT_PIN(atoi(argv[1]), atoi(argv[2])));
#else
    pcf857x_gpio_set(&pcf857x_dev[atoi(argv[1])],
                     PCF857X_GPIO_PIN(0, atoi(argv[2])));
#endif

    return 0;
}

static int clear(int argc, char **argv)
{
    if (argc < 3) {
        printf("usage: %s <port> <pin>\n", argv[0]);
        return 1;
    }

#if MODULE_EXTEND_GPIO
    gpio_clear(GPIO_EXT_PIN(atoi(argv[1]), atoi(argv[2])));
#else
    pcf857x_gpio_clear(&pcf857x_dev[atoi(argv[1])],
                        PCF857X_GPIO_PIN(0, atoi(argv[2])));
#endif

    return 0;
}

static int toggle(int argc, char **argv)
{
    if (argc < 3) {
        printf("usage: %s <port> <pin>\n", argv[0]);
        return 1;
    }

#if MODULE_EXTEND_GPIO
    gpio_toggle(GPIO_EXT_PIN(atoi(argv[1]), atoi(argv[2])));
#else
    pcf857x_gpio_toggle(&pcf857x_dev[atoi(argv[1])],
                        PCF857X_GPIO_PIN(0, atoi(argv[2])));
#endif

    return 0;
}

static int bench(int argc, char **argv)
{
    if (argc < 3) {
        printf("usage: %s <port> <pin> [# of runs]\n", argv[0]);
        return 1;
    }

#if MODULE_EXTEND_GPIO
    gpio_t pi = GPIO_EXT_PIN(atoi(argv[1]), atoi(argv[2]));
#else
    gpio_t pi = PCF857X_GPIO_PIN(0, atoi(argv[2]));
    int po = atoi(argv[1]);
#endif


    unsigned long runs = BENCH_RUNS_DEFAULT;
    if (argc > 3) {
        runs = (unsigned long)atol(argv[3]);
    }

    puts("\nGPIO driver run-time performance benchmark\n");
#if MODULE_EXTEND_GPIO
    BENCHMARK_FUNC("nop loop", runs, __asm__ volatile("nop"));
    gpio_init(pi, GPIO_OUT);
    BENCHMARK_FUNC("gpio_set", runs, gpio_set(pi));
    BENCHMARK_FUNC("gpio_clear", runs, gpio_clear(pi));
    BENCHMARK_FUNC("gpio_toggle", runs, gpio_toggle(pi));
    gpio_init(pi, GPIO_IN);
    BENCHMARK_FUNC("gpio_read", runs, (void)gpio_read(pi));
    gpio_init(pi, GPIO_OUT);
    BENCHMARK_FUNC("gpio_write", runs, gpio_write(pi, 1));
#else
    pcf857x_t* dev = &pcf857x_dev[po];
    BENCHMARK_FUNC("nop loop", runs, __asm__ volatile("nop"));
    pcf857x_gpio_init(dev, pi, GPIO_OUT);
    BENCHMARK_FUNC("gpio_set", runs, pcf857x_gpio_set(dev, pi));
    BENCHMARK_FUNC("gpio_clear", runs, pcf857x_gpio_clear(dev, pi));
    BENCHMARK_FUNC("gpio_toggle", runs, pcf857x_gpio_toggle(dev, pi));
    pcf857x_gpio_init(dev, pi, GPIO_IN);
    BENCHMARK_FUNC("gpio_read", runs, (void)pcf857x_gpio_read(dev, pi));
    pcf857x_gpio_init(dev, pi, GPIO_OUT);
    BENCHMARK_FUNC("gpio_write", runs, pcf857x_gpio_write(dev, pi, 1));
#endif
    puts("\n --- DONE ---");
    return 0;
}

static const shell_command_t shell_commands[] = {
    { "init_out", "init as output (push-pull mode)", init_out },
    { "init_in", "init as input w/o pull resistor", init_in },
    { "init_in_pu", "init as input with pull-up", init_in_pu },
    { "init_od", "init as output (open-drain without pull resistor)", init_od },
    { "init_od_pu", "init as output (open-drain with pull-up)", init_od_pu },
#ifdef MODULE_PERIPH_GPIO_IRQ
    { "init_int", "init as external INT w/o pull resistor", init_int },
    { "enable_int", "enable or disable gpio interrupt", enable_int },
#endif
    { "read", "read pin status", read },
    { "set", "set pin to HIGH", set },
    { "clear", "set pin to LOW", clear },
    { "toggle", "toggle pin", toggle },
    { "bench", "run a set of predefined benchmarks", bench },
    { NULL, NULL, NULL }
};

int main(void)
{
#if MODULE_GPIO_EXTEND
    /*
     * number of GPIO extension list entries has correspond to the
     * number of configured PCF857X devices
     */
    assert(PCF857X_NUM == sizeof(gpio_ext_list[]) / sizeof(gpio_ext_list[0]));
#endif

    puts("PCF857X I/O expander GPIO peripheral driver test\n");
    puts("Initializing PCF857X");

    /* initialize configured PCF857X devices */
    for (unsigned i = 0; i < PCF857X_NUM; i++) {
        if (pcf857x_init(&pcf857x_dev[i], &pcf857x_params[i]) != PCF857X_OK) {
            puts("[Failed]");
            return 1;
        }
    }
    puts("[OK]\n");

    puts("In this test, pins are specified by integer port and pin numbers.\n"
         "PCF8574 has 8 I/O pins labeled P00...P07.\n"
         "PCF8575 has 16 I/O pins labeled P00...P07 and P10...P17\n"
         "Use port 0 and pin 0...15 in all commands to access them.\n\n"
         "NOTE: make sure the values you use exist! The\n"
         "      behavior for not existing ports/pins is not defined!");

    /* start the shell */
    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);

    return 0;
}
