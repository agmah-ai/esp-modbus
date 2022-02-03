#warning "include master test source"

//#include "port_tcp_master.h"            // required for modbus tcp port types (the header exposed by cmake file)
#include "esp_modbus_master.h"

#include "ut_io.h"
#include "mbport_stubs.h"

#include "portserial_m_test.h"

// Include the moddule under test to get access to its static functions/variables for test porpose
#include "portserial_m.c"

static int inp_stream_id = -1;
static int out_stream_id = -1;

BOOL __wrap_xMBMasterPortSerialInit( UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity )
{
    UT_PORT_INIT("mbm_serial");
    return __real_xMBMasterPortSerialInit(ucPORT, ulBaudRate, ucDataBits, eParity);
}

BOOL __wrap_xMBMasterPortSerialPutByte(CHAR ucByte)
{
    //UT_LOGI("UT", "Func wrapper called: %s.", __func__);
    BOOL res = FALSE;
#if CONFIG_MB_UTEST_LOG
    res = __real_xMBMasterPortSerialPutByte(ucByte);
#elif CONFIG_MB_UTEST_OVERRIDE
    uint8_t byte = 0;
    res = (ut_stream_get_data(STREAM_ID_OUTPUT, &byte, 1, 1) == 1);
#endif
    return res;  
}

BOOL __wrap_xMBMasterPortSerialGetByte(CHAR* pucByte)
{
    //UT_LOGI("UT", "Func wrapper called: %s.", __func__);
    BOOL res = FALSE;
#if CONFIG_MB_UTEST_LOG
    res = __real_xMBMasterPortSerialGetByte(pucByte);
#elif CONFIG_MB_UTEST_OVERRIDE
    //res = __real_xMBMasterPortSerialGetByte(pucByte);
    res = (ut_stream_get_data(STREAM_ID_INPUT, pucByte, 1, 1) == 1);
#endif
    return res;
}

BOOL __wrap_xMBMasterSerialPortGetResponse(UCHAR **ppucMBSerialFrame, USHORT * usSerialLength)
{
#if CONFIG_MB_UTEST_LOG
    MB_WRAPPER_LOG_GET(ppucMBSerialFrame, usSerialLength, __real_xMBMasterSerialPortGetResponse);
#elif CONFIG_MB_UTEST_OVERRIDE
    //MB_WRAPPER_OVERRIDE_GET(ppucMBSerialFrame, usSerialLength, __real_xMBMasterSerialPortGetResponse);
    return TRUE;
#endif
}

BOOL __wrap_xMBMasterSerialPortSendRequest(UCHAR *pucMBSerialFrame, USHORT usSerialLength)
{
#if CONFIG_MB_UTEST_LOG
    MB_WRAPPER_LOG_SEND(pucMBSerialFrame, usSerialLength, __real_xMBMasterSerialPortSendRequest);
#elif CONFIG_MB_UTEST_OVERRIDE
    ESP_LOGW(__func__, "buffer: %p, size: %d", pucMBSerialFrame, usSerialLength);
    esp_err_t err = ESP_ERR_INVALID_STATE;
    size_t out_size = 0;
    err = ut_stream_get_notification(STREAM_ID_OUTPUT, pdMS_TO_TICKS(100), &out_size);
    if (out_size != usSerialLength) {
        ESP_LOGE(__func__, "Input len: %d != out len: %d", out_size, usSerialLength);
    }
    err = ut_stream_send_notification(STREAM_ID_INPUT, pdMS_TO_TICKS(100), NULL);
    //err = ut_stream_buffer_reset(STREAM_ID_OUTPUT); // Do not check the output buffer
    return TRUE;
    //MB_WRAPPER_OVERRIDE_SEND(pucMBSerialFrame, usSerialLength, __real_xMBMasterSerialPortSendRequest);
#endif
}

void __wrap_vMBMasterPortSerialClose(void) 
{
    UT_LOGI("UT", "Func wrapper called: %s.", __func__);
    ut_close();
    return __real_vMBMasterPortSerialClose();
}

// BOOL xMBMasterPortSerialInputEvent(uart_event_t* pxEvent, ULONG xTimeout)
// {
//     ESP_LOGI(__func__, "timeout: %lu.", xTimeout);
//     pxEvent->type = UART_DATA;
//     pxEvent->timeout_flag = true;
//     pxEvent->size = 10;
//     return TRUE;
// }