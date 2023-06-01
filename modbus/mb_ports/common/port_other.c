/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "sys/lock.h"

#include "port_common.h"

/* ----------------------- Variables ----------------------------------------*/

/* ----------------------- Start implementation -----------------------------*/
__attribute__((always_inline)) int lock_obj(_lock_t *plock)
{
    _lock_acquire(plock);
    return 1;
}

__attribute__((always_inline)) void unlock_obj(_lock_t *plock)
{
    _lock_release(plock);
}
