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
 * @brief       ESP function required by SDK
 *
 * This file is a collection of functions required by ESP8266 RTOS SDK.
 *
 * @author      Gunar Schorcht <gunar@schorcht.net>
 */

#define ENABLE_DEBUG (0)
#include "debug.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "assert.h"
#include "esp/xtensa_ops.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_libc.h"
#include "irq_arch.h"
#include "rom/ets_sys.h"

#include "xtensa/xtensa_api.h"

/* Just to satisfy the linker, lwIP from SDK is not used */
uint32_t LwipTimOutLim = 0;

#ifndef MODULE_LWIP_ETHERNET
const uint8_t ethbroadcast[] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
#endif

/* This function is not part on newlib API, it is defined in libc/stdio/local.h
 * There is no nice way to get __cleanup member populated while avoiding __sinit,
 * so extern declaration is used here.
 */
extern void _cleanup_r(struct _reent* r);

/**
 * This is the replacement for newlib's _REENT_INIT_PTR and __sinit.
 * The problem with __sinit is that it allocates three FILE structures
 * (stdin, stdout, stderr). Having individual standard streams for each task
 * is a bit too much on a small embedded system. So we point streams
 * to the streams of the global struct _reent, which are initialized in
 * startup code.
 */
void IRAM_ATTR esp_reent_init(struct _reent* r)
{
    memset(r, 0, sizeof(*r));
    r->_stdout = _GLOBAL_REENT->_stdout;
    r->_stderr = _GLOBAL_REENT->_stderr;
    r->_stdin  = _GLOBAL_REENT->_stdin;
    r->__cleanup = &_cleanup_r;
    r->__sdidinit = 1;
    r->__sglue._next = NULL;
    r->__sglue._niobs = 0;
    r->__sglue._iobs = NULL;
    r->_current_locale = "C";
}

void IRAM_ATTR HDL_MAC_SIG_IN_LV1_ISR(void)
{
    extern unsigned int ets_soft_int_type;
    ets_soft_int_type = ETS_SOFT_INT_HDL_MAC;
    WSR(BIT(ETS_SOFT_INUM), interrupt);
}

void __attribute__((noreturn)) _esp_error_check_failed(esp_err_t rc, const char *file, int line, const char *function, const char *expression)
{
    printf("ESP_ERROR_CHECK failed: esp_err_t 0x%x at %p\n", rc, __builtin_return_address(0));
    printf("file: \"%s\" line %d\nfunc: %s\nexpression: %s\n", file, line, function, expression);
    abort();
}

void IRAM_ATTR _xt_isr_attach(uint8_t i, xt_handler func, void* arg)
{
    DEBUG("%s %d %p\n", __func__, i, func);
    xt_set_interrupt_handler(i, func, arg);
}


unsigned int IRAM_ATTR _xt_isr_unmask(unsigned int mask)
{
    DEBUG("%s %08x\n", __func__, mask);
    return xt_ints_on(mask);
}

unsigned int IRAM_ATTR _xt_isr_mask(unsigned int mask)
{
    DEBUG("%s %08x\n", __func__, mask);
    return xt_ints_off(mask);
}

void IRAM_ATTR _xt_clear_ints(uint32_t mask)
{
    DEBUG("%s %08x\n", __func__, mask);
    xt_set_intclear(mask);
}

void IRAM_ATTR _xt_set_xt_ccompare_val(void)
{
    /* to figure out whether it is called at all, not yet implemented */
    assert(0);
    DEBUG("%s\n", __func__);
}

