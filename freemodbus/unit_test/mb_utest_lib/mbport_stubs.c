/*
 * SPDX-FileCopyrightText: 2018-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
 
#include "sdkconfig.h"
#include "port.h"
#include "esp_log.h"
#include "esp_err.h"

#include "ut_io.h"
#include "mbport_stubs.h"

#define TAG "MBPORT_STUB"

#if CONFIG_MB_UTEST

#ifdef __cplusplus
extern "C" {
#endif

#define MB_WRAPPER_LOG_GET(frame_addr, frame_length, frame_get_method) do { \
    UT_LOGI("UT", "Func wrapper called: %s.", __func__); \
    BOOL ret = frame_get_method(frame_addr, frame_length); \
    UT_RETURN_ON_FALSE((*frame_addr && frame_length), FALSE, TAG, "wrong get request pointers."); \
    esp_err_t err = ut_stream_capture_packet(DIRECTION_INPUT, (uint8_t*)*frame_addr, (uint32_t)*frame_length, 0); \
    UT_RETURN_ON_FALSE(err == ESP_OK, FALSE, TAG, "fail to override packet."); \
    return ret; \
} while (0)

#define MB_WRAPPER_LOG_SEND(frame_addr, frame_length, frame_send_method) do { \
    UT_LOGI("UT", "Func wrapper called: %s.", __func__); \
    esp_err_t err = ut_stream_capture_packet(DIRECTION_OUTPUT, (uint8_t*)frame_addr, (uint32_t)frame_length, 0); \
    UT_RETURN_ON_FALSE((err == ESP_OK), FALSE, TAG, "fail to capture packet."); \
    return frame_send_method(frame_addr, frame_length); \
} while (0)

#define MB_WRAPPER_OVERRIDE_SEND(frame_addr, frame_length, frame_send_method) do { \
    ESP_LOGW(__func__, "faddr: %p, flen: %d.", frame_addr, frame_length); \
    return frame_send_method(frame_addr, frame_length); \
} while (0)

#define MB_WRAPPER_OVERRIDE_GET(frame_addr, frame_length, frame_get_method) do { \
    BOOL result = frame_get_method(frame_addr, frame_length); \
    UT_LOGW(__func__, "faddr: %p, flen: ", *frame_addr, *frame_length); \
    return result; \
} while (0)

//    *frame_addr = input_buffer;
//    *frame_length = input_length;

#if 0

#define MB_WRAPPER_OVERRIDE_SEND(frame_addr, frame_length, frame_send_method) do { \
    UT_LOGI("UT", "Func wrapper called: %s.", __func__); \
    esp_err_t err = ut_stream_override_packet((uint8_t*)frame_addr, (uint32_t)frame_length); \
    UT_RETURN_ON_FALSE((err == ESP_OK), FALSE, TAG, "fail to override packet."); \
    return frame_send_method(frame_addr, frame_length); \
} while (0)

//return frame_get_method(frame_addr, frame_length);

#define MB_WRAPPER_OVERRIDE_GET(frame_addr, frame_length, frame_get_method) do { \
    UT_LOGI("UT", "Func wrapper called: %s.", __func__); \
    UT_RETURN_ON_FALSE((input_buffer && input_length), FALSE, TAG, "incorrect input data."); \
    BOOL result = frame_get_method(frame_addr, frame_length); \
    *frame_addr = input_buffer; \
    *frame_length = input_length; \
    return result; \
} while (0)

#endif

/*
void mb_master_serial_poll_cb(const UCHAR* pucPDUData, USHORT ucPDULength)
{
    //__real_usMBMasterPortSerialRxPoll(ucPDULength);
    ESP_EARLY_LOGI("UT", "Func wrapper called: %s.", __func__);
    input_buffer = (UCHAR*)pucPDUData;
    input_length = ucPDULength;
    vMBMasterSetCurTimerMode(MB_TMODE_T35);
    xMBMasterPortEventPost(EV_MASTER_FRAME_RECEIVED);
}
*/

void __wrap_vMBMasterErrorCBRespondTimeout( UCHAR ucDestAddress, const UCHAR* pucPDUData,
                                                USHORT ucPDULength )
{
#if CONFIG_MB_UTEST_LOG
    UT_LOGI("UT", "Func wrapper called: %s.", __func__);
    ut_stream_capture_packet(DIRECTION_INPUT, NULL, 0, 0);
    __real_vMBMasterErrorCBRespondTimeout(ucDestAddress, pucPDUData, ucPDULength);
#endif
}

BOOL __wrap_xMBMasterPortEventGet(eMBMasterEventType* eEvent)
{
    BOOL res = __real_xMBMasterPortEventGet(eEvent);
    if (res) {
        UT_LOGI("EVT", "Get event: %x", *eEvent);
    }
    return res;
}

eMBMasterReqErrCode __wrap_eMBMasterWaitRequestFinish( void )
{
    eMBMasterReqErrCode event = __real_eMBMasterWaitRequestFinish();
    UT_LOGI("REQ", "Request event get: %x", event);
    return event;
}

BOOL __wrap_xMBMasterTCPPortInit( USHORT usTCPPort )
{
    UT_LOGI("UT", "Func wrapper called: %s.", __func__);
    ut_init("mbm_tcp");
    return __real_xMBMasterTCPPortInit(usTCPPort);
}

BOOL __wrap_xMBMasterTCPPortSendRequest( UCHAR * pucMBTCPFrame, USHORT usTCPLength )
{
#if CONFIG_MB_UTEST_LOG
    MB_WRAPPER_LOG_SEND(pucMBTCPFrame, usTCPLength, __real_xMBMasterTCPPortSendRequest);
#elif CONFIG_MB_UTEST_OVERRIDE
    MB_WRAPPER_OVERRIDE_SEND(pucMBTCPFrame, usTCPLength, __real_xMBMasterTCPPortSendRequest);
#endif
}

BOOL __wrap_xMBMasterTCPPortGetResponse( UCHAR ** ppucMBTCPFrame, USHORT * usTCPLength )
{
#if CONFIG_MB_UTEST_LOG
    MB_WRAPPER_LOG_GET(ppucMBTCPFrame, usTCPLength, __real_xMBMasterTCPPortGetResponse);
#elif CONFIG_MB_UTEST_OVERRIDE
    MB_WRAPPER_OVERRIDE_GET(ppucMBTCPFrame, usTCPLength, __real_xMBMasterTCPPortGetResponse);
#endif
}

void __wrap_vMBMasterTCPPortClose(void) 
{
    UT_LOGI("UT", "Func wrapper called: %s.", __func__);
    ut_close();
    return __real_vMBMasterTCPPortClose();
}

BOOL __wrap_xMBPortSerialInit(UCHAR ucPORT, ULONG ulBaudRate,
                        UCHAR ucDataBits, eMBParity eParity)
{
    UT_LOGI("UT", "Func wrapper called: %s.", __func__);
    ut_init("mbs_serial");
    return __real_xMBPortSerialInit(ucPORT, ulBaudRate, ucDataBits, eParity);
}
                        
BOOL __wrap_xMBPortSerialPutByte(CHAR ucByte)
{
    UT_LOGI("UT","Func wrapper called: %s.", __func__);
    return __real_xMBPortSerialPutByte(ucByte);
}

BOOL __wrap_xMBPortSerialGetByte(CHAR* pucByte)
{
    UT_LOGI("UT", "Func wrapper called: %s.", __func__);
    return __real_xMBPortSerialGetByte(pucByte);
}

BOOL __wrap_xMBSerialPortGetRequest(UCHAR **ppucMBSerialFrame, USHORT * usSerialLength)
{
#if CONFIG_MB_UTEST_LOG
    MB_WRAPPER_LOG_GET(ppucMBSerialFrame, usSerialLength, __real_xMBSerialPortGetRequest);
#elif CONFIG_MB_UTEST_OVERRIDE
    MB_WRAPPER_OVERRIDE_GET(ppucMBSerialFrame, usSerialLength, __real_xMBSerialPortGetRequest);
#endif
}

BOOL __wrap_xMBSerialPortSendResponse(UCHAR *pucMBSerialFrame, USHORT usSerialLength)
{
#if CONFIG_MB_UTEST_LOG
    MB_WRAPPER_LOG_SEND(pucMBSerialFrame, usSerialLength, __real_xMBSerialPortSendResponse);
#elif CONFIG_MB_UTEST_OVERRIDE
    MB_WRAPPER_OVERRIDE_SEND(pucMBSerialFrame, usSerialLength, __real_xMBSerialPortSendResponse);
#endif
}

void __wrap_vMBPortSerialClose(void) 
{
    UT_LOGI("UT", "Func wrapper called: %s.", __func__);
    ut_close();
    return __real_vMBPortSerialClose();
}

BOOL __wrap_xMBMasterPortSerialInit( UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity )
{
    UT_LOGI("UT", "Func wrapper called: %s.", __func__);
    ut_init("mbm_serial"); // check driver
    return __real_xMBMasterPortSerialInit(ucPORT, ulBaudRate, ucDataBits, eParity);
}

BOOL __wrap_xMBMasterPortSerialPutByte(CHAR ucByte)
{
    UT_LOGI("UT", "Func wrapper called: %s.", __func__);
    return __real_xMBMasterPortSerialPutByte(ucByte);   
}

BOOL __wrap_xMBMasterPortSerialGetByte(CHAR* pucByte)
{
    UT_LOGI("UT", "Func wrapper called: %s.", __func__);
    BOOL res = FALSE;
#if CONFIG_MB_UTEST_LOG
    res = __real_xMBMasterPortSerialGetByte(pucByte);
#elif CONFIG_MB_UTEST_OVERRIDE
    #if 0
    if (input_buffer && pucByte) {
        *pucByte = *input_buffer++;
        UT_LOGI("UT", "Func wrapper called: %s, byte:0x%x.", __func__, *pucByte);
        res = TRUE;
    }
    #endif
#endif
    return res;
}

BOOL __wrap_xMBMasterSerialPortGetResponse(UCHAR **ppucMBSerialFrame, USHORT * usSerialLength)
{
#if CONFIG_MB_UTEST_LOG
    MB_WRAPPER_LOG_GET(ppucMBSerialFrame, usSerialLength, __real_xMBMasterSerialPortGetResponse);
#elif CONFIG_MB_UTEST_OVERRIDE
    MB_WRAPPER_OVERRIDE_GET(ppucMBSerialFrame, usSerialLength, __real_xMBMasterSerialPortGetResponse);
#endif
}

BOOL __wrap_xMBMasterSerialPortSendRequest(UCHAR *pucMBSerialFrame, USHORT usSerialLength)
{
#if CONFIG_MB_UTEST_LOG
    MB_WRAPPER_LOG_SEND(pucMBSerialFrame, usSerialLength, __real_xMBMasterSerialPortSendRequest);
#elif CONFIG_MB_UTEST_OVERRIDE
    MB_WRAPPER_OVERRIDE_SEND(pucMBSerialFrame, usSerialLength, __real_xMBMasterSerialPortSendRequest);
#endif
}

void __wrap_vMBMasterPortSerialClose(void) 
{
    UT_LOGI("UT", "Func wrapper called: %s.", __func__);
    ut_close();
    return __real_vMBMasterPortSerialClose();
}

BOOL __wrap_xMBTCPPortInit( USHORT usTCPPort )
{
    UT_LOGI("UT", "Func wrapper called: %s.", __func__);
    ut_init("mbs_tcp");
    return __real_xMBTCPPortInit(usTCPPort);
}

BOOL __wrap_xMBTCPPortSendResponse( UCHAR * pucMBTCPFrame, USHORT usTCPLength )
{
#if CONFIG_MB_UTEST_LOG
    MB_WRAPPER_LOG_SEND(pucMBTCPFrame, usTCPLength, __real_xMBTCPPortSendResponse);
#elif CONFIG_MB_UTEST_OVERRIDE
    MB_WRAPPER_OVERRIDE_SEND(pucMBTCPFrame, usTCPLength, __real_xMBTCPPortSendResponse);
#endif
}

BOOL __wrap_xMBTCPPortGetRequest( UCHAR ** ppucMBTCPFrame, USHORT * usTCPLength )
{
#if CONFIG_MB_UTEST_LOG
    MB_WRAPPER_LOG_GET(ppucMBTCPFrame, usTCPLength, __real_xMBTCPPortGetRequest);
#elif CONFIG_MB_UTEST_OVERRIDE
    MB_WRAPPER_OVERRIDE_GET(ppucMBTCPFrame, usTCPLength, __real_xMBTCPPortGetRequest);
#endif
}

void __wrap_vMBTCPPortClose(void) 
{
    UT_LOGI("UT", "Func wrapper called: %s.", __func__);
    ut_close();
    return __real_vMBTCPPortClose();
}

// Override the function definition in source file
err_t __wrap_xMBTCPPortMasterConnect(MbSlaveInfo_t* pxInfo)
{
    ESP_LOGE("UT", "Func wrapper called: %s.", __func__);
    return ERR_OK;
}

#ifdef __cplusplus
}
#endif

#endif
