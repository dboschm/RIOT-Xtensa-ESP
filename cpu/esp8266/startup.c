/*
 * Copyright (C) 2019 Gunar Schorcht
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     cpu_esp8266
 * @{
 *
 * @file
 * @brief       Implementation of the CPU initialization
 *
 * @author      Gunar Schorcht <gunar@schorcht.net>
 * @}
 */
#define ENABLE_DEBUG  0
#include "debug.h"

#include <stdint.h>
#include <stdio.h>
#include <stddef.h>
#include <stdarg.h>
#include <string.h>

#include "kernel_init.h"
#include "periph/init.h"
#include "periph/uart.h"

#include "board.h"

#include "esp/common_macros.h"
#include "esp_log.h"
#include "exceptions.h"
#include "syscalls.h"
#include "thread_arch.h"

#include "rom_functions.h"
#include "sdk/sdk.h"

#if MODULE_ESP_GDBSTUB
#include "gdbstub.h"
#endif

/* external esp function declarations */
extern uint32_t hwrand (void);

void esp_riot_init(void)
{
    /* enable cached read from flash */
    Cache_Read_Enable_New();

    /* initialize the ISR stack for usage measurements */
    thread_isr_stack_init();

#ifndef MCU_ESP8266
    /* initialize newlib system calls */
    syscalls_init ();
#endif

    /* set system frequency if not 80 MHz */
    if (ESP8266_CPU_FREQUENCY != 80) {
        system_update_cpu_freq(ESP8266_CPU_FREQUENCY);
    }

    ets_printf("\n");
    ets_printf("Starting ESP8266 CPU with ID: %08x\n", system_get_chip_id());
    ets_printf("ESP8266-RTOS-SDK Version %s\n\n", system_get_sdk_version());
    ets_printf("CPU clock frequency: %d MHz\n", system_get_cpu_freq());
    extern void heap_stats(void);
    heap_stats();
    ets_printf("\n");

    /* set exception handlers */
    init_exceptions ();

    /* systemwide UART initialization */
    extern void uart_system_init (void);
    uart_system_init();

#ifndef MCU_ESP8266
    /* initialize newlib data structure */
    extern void esp_reent_init(struct _reent* r);
    esp_reent_init(_GLOBAL_REENT);
    _GLOBAL_REENT->_stdin  = (FILE*) &__sf_fake_stdin;
    _GLOBAL_REENT->_stdout = (FILE*) &__sf_fake_stdout;
    _GLOBAL_REENT->_stderr = (FILE*) &__sf_fake_stderr;
#endif

    /* init watchdogs */
    system_wdt_init();

    /* init random number generator */
    srand(hwrand());

    #if defined(MODULE_NEWLIB_SYSCALLS_DEFAULT)
    /*
     * initialization as it should be called from newlibc (includes the
     * execution of stdio_init)
    */
    extern void _init(void);
    _init();
    #elif defined(MODULE_STDIO_UART) /* TODO remove it when #10806 is merged */
    extern void stdio_init(void);
    stdio_init();
    #endif

    #if MODULE_MTD
    /* init flash drive */
    extern void flash_drive_init (void);
    flash_drive_init();
    #endif

    /* trigger static peripheral initialization */
    periph_init();

    /* trigger board initialization */
    board_init();

    /* print the board config */
    board_print_config();

    /* initialize ESP system event loop */
    extern void esp_event_handler_init(void);
    esp_event_handler_init();

    /* activate software interrupt based context switch */
    extern void IRAM thread_yield_isr(void* arg);
    ets_isr_attach(ETS_SOFT_INUM, thread_yield_isr, NULL);
    ets_isr_unmask(BIT(ETS_SOFT_INUM));

    #ifdef MODULE_ESP_GDBSTUB
    gdbstub_init();
    #endif
}

void esp_riot_start(void)
{
    /* does not return */
    kernel_init();
}
