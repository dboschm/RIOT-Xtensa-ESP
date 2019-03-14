/*
 * Copyright (C) 2019 Gunar Schorcht
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     cpu_esp8266_sdk
 * @{
 *
 * @file
 * @brief       ESP8266 ETS ROM functions
 *
 * @author      Gunar Schorcht <gunar@schorcht.net>
 * @}
 */

#include <string.h>

#include "sdk/sdk.h"

uint8_t ets_get_cpu_frequency(void)
{
    return system_get_cpu_freq();
}

void *ets_memcpy(void *dst, const void *src, size_t size)
{
    return memcpy(dst, src, size);
}

void ets_wdt_disable(void)
{
    /* TODO implement */
}

void ets_wdt_enable (void)
{
    /* TODO implement */
}

void ets_install_putc1(void (*p)(char c))
{
    os_install_putc1(p);
}
