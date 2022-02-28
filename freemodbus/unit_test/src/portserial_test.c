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
    UT_LOGI("UT", "Func wrapper called: %s.", __func__);
    ut_init("mbs_serial");
    UT_RETURN_ON_FALSE((ut_stream_create("input", &inp_stream_id) == ESP_OK), 
                        FALSE, TAG, "Could not create input stream.");
    UT_RETURN_ON_FALSE((ut_stream_create("output", &out_stream_id) == ESP_OK), 
                        FALSE, TAG, "Could not create output stream.");
    ut_stream_send_notification(STREAM_ID_INPUT, pdMS_TO_TICKS(100), NULL);

    return __real_xMBPortSerialInit(ucPORT, ulBaudRate, ucDataBits, eParity);
}
                        
BOOL __wrap_xMBPortSerialPutByte(CHAR ucByte)
{
    //UT_LOGI("UT", "Func wrapper called: %s.", __func__);
    BOOL res = FALSE;
#if CONFIG_MB_UTEST_LOG
    res = __real_xMBPortSerialPutByte(ucByte);
#elif CONFIG_MB_UTEST_OVERRIDE
    uint8_t byte = 0;
    res = (ut_stream_get_data(STREAM_ID_OUTPUT, &byte, 1, 1) == 1);
#endif
    return res;
}

BOOL __wrap_xMBPortSerialGetByte(CHAR* pucByte)
{
    UT_LOGI("UT", "Func wrapper called: %s.", __func__);
    BOOL res = FALSE;
#if CONFIG_MB_UTEST_LOG
    res = __real_xMBPortSerialGetByte(pucByte);
#elif CONFIG_MB_UTEST_OVERRIDE
    res = (ut_stream_get_data(STREAM_ID_INPUT, pucByte, 1, 1) == 1);
#endif
    return res;
}

BOOL __wrap_xMBSerialPortGetRequest(UCHAR **ppucMBSerialFrame, USHORT * usSerialLength)
{
#if CONFIG_MB_UTEST_LOG
    MB_WRAPPER_LOG_GET(ppucMBSerialFrame, usSerialLength, __real_xMBSerialPortGetRequest);
#elif CONFIG_MB_UTEST_OVERRIDE
    //MB_WRAPPER_OVERRIDE_GET(ppucMBSerialFrame, usSerialLength, __real_xMBSerialPortGetRequest);
    return (ut_stream_send_notification(STREAM_ID_INPUT, pdMS_TO_TICKS(100), NULL) == ESP_OK);
#endif
}

BOOL __wrap_xMBSerialPortSendResponse(UCHAR *pucMBSerialFrame, USHORT usSerialLength)
{
#if CONFIG_MB_UTEST_LOG
    MB_WRAPPER_LOG_SEND(pucMBSerialFrame, usSerialLength, __real_xMBSerialPortSendResponse);
#elif CONFIG_MB_UTEST_OVERRIDE
    size_t out_size = 0;
    esp_err_t err = ut_stream_get_notification(STREAM_ID_OUTPUT, pdMS_TO_TICKS(100), &out_size);
    if (out_size != usSerialLength) {
        ESP_LOGE(__func__, "new len: %d !=  old len: %d", out_size, usSerialLength);
    }
    //MB_WRAPPER_OVERRIDE_SEND(pucMBSerialFrame, usSerialLength, __real_xMBSerialPortSendResponse);
    return TRUE;
#endif
}

void __wrap_vMBPortSerialClose(void) 
{
    UT_LOGI("UT", "Func wrapper called: %s.", __func__);
    ut_close();
    return __real_vMBPortSerialClose();
}
