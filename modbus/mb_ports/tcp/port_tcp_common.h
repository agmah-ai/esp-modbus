/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <stdbool.h>
#include <string.h>
#include "mb_config.h"
#include "mb_common.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MB_TCP_PORT_MAX_CONN            (CONFIG_FMB_TCP_PORT_MAX_CONN)
#define MB_TCP_DEFAULT_PORT             (502)
#define MB_FRAME_QUEUE_SZ               (10)
#define MB_TCP_CONNECTION_TIMEOUT_MS    (20)  // Connection timeout in mS
#define MB_TCP_RECONNECT_TIMEOUT        (5000000) // Connection timeout in uS

#define MB_EVENT_SEND_RCV_TOUT_MS (500)

#define MB_TCP_MBAP_GET_FIELD(buffer, field) ((uint16_t)((buffer[field] << 8U) | buffer[field + 1]))
#define MB_TCP_MBAP_SET_FIELD(buffer, field, val) { \
    buffer[(field)] = (uint8_t)((val) >> 8U); \
    buffer[(field) + 1] = (uint8_t)((val) & 0xFF); \
}

#define MB_SLAVE_FMT(fmt) "slave #%d, socket(#%d)(%s)" fmt

mb_err_enum_t mbm_port_tcp_create(mb_tcp_opts_t *tcp_opts, mb_port_base_t **port_obj);
void mbm_port_tcp_delete(mb_port_base_t *inst);
void mbm_port_tcp_enable(mb_port_base_t *inst);
void mbm_port_tcp_disable(mb_port_base_t *inst);
bool mbm_port_tcp_send_data(mb_port_base_t *inst, uint8_t address, uint8_t *pframe, uint16_t length);
bool mbm_port_tcp_recv_data(mb_port_base_t *inst, uint8_t **ppframe, uint16_t *plength);
bool mbm_port_tcp_add_slave_info(mb_port_base_t *inst, const uint16_t index, const char *ip_str, uint8_t slave_addr);

mb_err_enum_t mbs_port_tcp_create(mb_tcp_opts_t *tcp_opts, mb_port_base_t **port_obj);
void mbs_port_tcp_delete(mb_port_base_t *inst);
void mbs_port_tcp_enable(mb_port_base_t *inst);
void mbs_port_tcp_disable(mb_port_base_t *inst);
bool mbs_port_tcp_send_data(mb_port_base_t *inst, uint8_t *pframe, uint16_t length);
bool mbs_port_tcp_recv_data(mb_port_base_t *inst, uint8_t **ppframe, uint16_t *plength);

#ifdef __cplusplus
}
#endif