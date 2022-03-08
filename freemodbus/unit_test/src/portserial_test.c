#warning "include slave test source"

#include "sdkconfig.h"

#include "esp_modbus_slave.h"

#include "ut_io.h"

#include "portserial_test.h"
// Include the moddule under test to get access to its static functions/variables for test porpose
#include "portserial.c"

#include "mbport_stubs.h"

static int inp_stream_id = -1;
static int out_stream_id = -1;

// This is the place to place internal interface test functions
// This code has access to all static functions of the port file

BOOL __wrap_xMBPortSerialInit(UCHAR ucPORT, ULONG ulBaudRate,
                              UCHAR ucDataBits, eMBParity eParity)
{
    UT_PORT_INIT("mbs_serial");
    BOOL result = TRUE;
    result = __real_xMBPortSerialInit(ucPORT, ulBaudRate, ucDataBits, eParity);
    esp_err_t err = ESP_ERR_INVALID_STATE;
#if CONFIG_MB_UTEST_LOG
    err = ut_stream_capture_packet(inp_stream_id, "SYNC", sizeof("SYNC"), 0);
    UT_RETURN_ON_FALSE(err == ESP_OK, FALSE, __func__, "fail to save init data.");
    err = ut_stream_capture_packet(out_stream_id, "SYNC", sizeof("SYNC"), 0);
    UT_RETURN_ON_FALSE(err == ESP_OK, FALSE, __func__, "fail to save init data.");
#elif CONFIG_MB_UTEST_OVERRIDE
    // err = ut_stream_get_notification(STREAM_ID_INPUT, pdMS_TO_TICKS(100), NULL);
    // UT_RETURN_ON_FALSE(err == ESP_OK, FALSE, __func__, "fail to get init data.");
    err = ut_stream_get_notification(STREAM_ID_OUTPUT, pdMS_TO_TICKS(100), NULL);
    //UT_RETURN_ON_FALSE(err == ESP_OK, FALSE, __func__, "fail to get init data.");
    err = ut_stream_send_notification(STREAM_ID_INPUT, pdMS_TO_TICKS(100), NULL);
    //UT_RETURN_ON_FALSE(err == ESP_OK, FALSE, __func__, "fail to get init data.");
#endif
    return result;
}

BOOL __wrap_xMBPortSerialPutByte(CHAR ucByte)
{
    BOOL res = FALSE;
#if CONFIG_MB_UTEST_LOG
    res = __real_xMBPortSerialPutByte(ucByte);
#elif CONFIG_MB_UTEST_OVERRIDE
    uint8_t byte = 0;
    res = (ut_stream_get_data(STREAM_ID_OUTPUT, &byte, 1, 1) == 1);
#endif
    return res;
}

BOOL __wrap_xMBPortSerialGetByte(CHAR *pucByte)
{
    BOOL res = FALSE;
    
#if CONFIG_MB_UTEST_LOG
    res = __real_xMBPortSerialGetByte(pucByte);
    UT_LOGW(__func__, "Get byte %d", *pucByte);
#elif CONFIG_MB_UTEST_OVERRIDE
    res = (ut_stream_get_data(STREAM_ID_INPUT, pucByte, 1, 1) == 1);
#endif
    return res;
}

BOOL __wrap_xMBSerialPortGetRequest(UCHAR **ppucMBSerialFrame, USHORT *usSerialLength)
{
#if CONFIG_MB_UTEST_LOG
    MB_WRAPPER_LOG_GET(ppucMBSerialFrame, usSerialLength, __real_xMBSerialPortGetRequest);
#elif CONFIG_MB_UTEST_OVERRIDE
    // MB_WRAPPER_OVERRIDE_GET(ppucMBSerialFrame, usSerialLength, __real_xMBSerialPortGetRequest);
    ESP_LOGI(__func__, "len: %d", *usSerialLength);
    ut_stream_send_notification(STREAM_ID_OUTPUT, pdMS_TO_TICKS(100), NULL);
    return TRUE;
#endif
}

BOOL __wrap_xMBSerialPortSendResponse(UCHAR *pucMBSerialFrame, USHORT usSerialLength)
{
#if CONFIG_MB_UTEST_LOG
    MB_WRAPPER_LOG_SEND(pucMBSerialFrame, usSerialLength, __real_xMBSerialPortSendResponse);
#elif CONFIG_MB_UTEST_OVERRIDE
    size_t out_size = 0;
    ESP_LOGI(__func__, "len: %d", usSerialLength);
    esp_err_t err = ut_stream_wait_notification(STREAM_ID_OUTPUT, pdMS_TO_TICKS(100), &out_size);
    if ((out_size != usSerialLength) || err)
    {
        ESP_LOGE(__func__, "err: %x, new len: %d, old len: %d", err, out_size, usSerialLength);
    }
    // MB_WRAPPER_OVERRIDE_SEND(pucMBSerialFrame, usSerialLength, __real_xMBSerialPortSendResponse);
    ut_stream_send_notification(STREAM_ID_INPUT, pdMS_TO_TICKS(100), NULL);
    return (err == ESP_OK);
#endif
}

void __wrap_vMBPortSerialClose(void)
{
    UT_LOGI("UT", "Func wrapper called: %s.", __func__);
    ut_close();
    return __real_vMBPortSerialClose();
}
