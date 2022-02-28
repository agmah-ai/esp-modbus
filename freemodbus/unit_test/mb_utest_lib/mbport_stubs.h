/*
 * SPDX-FileCopyrightText: 2018-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
 
#pragma once

#include <sdkconfig.h>
#include "esp_log.h"
#include "port.h"
#include "mbport.h"
#include "mbcrc.h"
#include "mb_m.h"
#include "esp_modbus_master.h"

#if CONFIG_MB_UTEST

#define MB_WRAPPER_LOG_GET(frame_addr, frame_length, frame_get_method) do { \
    UT_LOGW("UT", "Func wrapper called: %s.", __func__); \
    BOOL ret = frame_get_method(frame_addr, frame_length); \
    UT_RETURN_ON_FALSE((*frame_addr && frame_length), FALSE, TAG, "wrong get request pointers."); \
    esp_err_t err = ut_stream_capture_packet(inp_stream_id, (uint8_t*)*frame_addr, (uint32_t)*frame_length, 0); \
    UT_RETURN_ON_FALSE(err == ESP_OK, FALSE, TAG, "fail to override packet."); \
    return ret; \
} while (0)

#define MB_WRAPPER_LOG_SEND(frame_addr, frame_length, frame_send_method) do { \
    UT_LOGW("UT", "Func wrapper called: %s.", __func__); \
    esp_err_t err = ut_stream_capture_packet(out_stream_id, (uint8_t*)frame_addr, (uint32_t)frame_length, 0); \
    UT_RETURN_ON_FALSE((err == ESP_OK), FALSE, TAG, "fail to capture packet."); \
    return frame_send_method(frame_addr, frame_length); \
} while (0)

#define MB_WRAPPER_OVERRIDE_SEND(frame_addr, frame_length, frame_send_method) do { \
    UT_LOGW(__func__, "faddr: %p, flen: %d.", frame_addr, frame_length); \
    return frame_send_method(frame_addr, frame_length); \
} while (0)

//BOOL result = frame_get_method(frame_addr, frame_length);
#define MB_WRAPPER_OVERRIDE_GET(frame_addr, frame_length, frame_get_method) do { \
    UT_LOGW(__func__, "faddr: %p, flen: %d", *frame_addr, *frame_length); \
    return frame_get_method(frame_addr, frame_length); \
} while (0)

extern USHORT usMBCRC16( UCHAR * pucFrame, USHORT usLen );

extern USHORT __real_usMBMasterPortSerialRxPoll(size_t xEventSize);

extern void __real_vMBMasterErrorCBRespondTimeout(UCHAR ucDestAddress, const UCHAR* pucPDUData, USHORT ucPDULength );

extern BOOL __real_xMBMasterPortEventGet(eMBMasterEventType* eEvent);

extern esp_err_t __wrap_init_services(mb_tcp_addr_type_t ip_addr_type);

extern eMBMasterReqErrCode __real_eMBMasterWaitRequestFinish( void );

//extern esp_err_t __real_mbc_master_send_request(mb_param_request_t* request, void* data_ptr);

#endif
