/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdbool.h>
#include <string.h>

#include "port_tcp_common.h"

typedef struct
{
    mb_port_base_t base;
    // TCP communication properties
    mb_tcp_opts_t tcp_opts;
    bool rx_state_en;
    bool tx_state_en;
    uint64_t transaction_cnt; 
    uint16_t recv_length;
    uint64_t send_time_stamp;
    uint64_t recv_time_stamp;
    uint32_t flags;
    TaskHandle_t task_handle;
} mb_tcp_port_t;

/* ----------------------- Static variables & functions ----------------------*/
static const char *TAG = "mb_port.tcp.slave";

mb_err_enum_t mbs_port_tcp_create(mb_tcp_opts_t *tcp_opts, mb_port_base_t **port_obj)
{
    mb_tcp_port_t *ptcp = NULL;
    ptcp = (mb_tcp_port_t*)calloc(1, sizeof(mb_tcp_port_t));
    MB_RETURN_ON_FALSE((ptcp && port_obj), MB_EILLSTATE, TAG, "mb tcp port creation error.");
    CRITICAL_SECTION_INIT(ptcp->base.lock);
    return MB_ENOERR;
}

void mbs_port_tcp_delete(mb_port_base_t *inst)
{
    mb_tcp_port_t *port_obj = __containerof(inst, mb_tcp_port_t, base);
    //vTaskDelete(port_obj->task_handle);
    CRITICAL_SECTION_CLOSE(inst->lock);
    free(port_obj);
}

void mbs_port_tcp_enable(mb_port_base_t *inst)
{
    mb_tcp_port_t *port_obj = __containerof(inst, mb_tcp_port_t, base);
}

void mbs_port_tcp_disable(mb_port_base_t *inst)
{
    mb_tcp_port_t *port_obj = __containerof(inst, mb_tcp_port_t, base);

    
}

bool mbs_port_tcp_recv_data(mb_port_base_t *inst, uint8_t **ppframe, uint16_t *plength)
{
    mb_tcp_port_t *port_obj = __containerof(inst, mb_tcp_port_t, base);

    return false;
}

bool mbs_port_tcp_send_data(mb_port_base_t *inst, uint8_t *pframe, uint16_t length)
{
    mb_tcp_port_t *port_obj = __containerof(inst, mb_tcp_port_t, base);

    return false;
}