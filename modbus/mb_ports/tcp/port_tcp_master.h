/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_event.h"          // for esp event loop

#include "mb_common.h"
#include "mb_frame.h"

#ifdef __cplusplus
extern "C" {
#endif

void mbm_port_tcp_set_conn_cb(mb_port_base_t *inst, void *conn_fp, void *arg);

typedef struct _slave_addr_info mb_slave_addr_info_t;

mb_slave_addr_info_t *mbm_port_tcp_find_slave_addr(mb_port_base_t *inst, uint8_t slave_addr);

#ifdef __cplusplus
}
#endif
