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
#include "port_tcp_master.h"

#define TAG "MBPORT_STUB"

#if CONFIG_MB_UTEST

#ifdef __cplusplus
extern "C" {
#endif

/*
void __wrap_vMBMasterErrorCBRespondTimeout( UCHAR ucDestAddress, const UCHAR* pucPDUData,
                                                USHORT ucPDULength )
{
#if CONFIG_MB_UTEST_LOG
    UT_LOGI("UT", "Func wrapper called: %s.", __func__);
    ut_stream_capture_packet(out_stream_id, NULL, 0, 0);
    __real_vMBMasterErrorCBRespondTimeout(ucDestAddress, pucPDUData, ucPDULength);
#endif
}

*/
BOOL __wrap_xMBMasterPortEventGet(eMBMasterEventType* eEvent)
{
    BOOL res = __real_xMBMasterPortEventGet(eEvent);
    if (res) {
        UT_LOGI("EVT", "Get event: %.2x", *eEvent);
    }
    return res;
}


eMBMasterReqErrCode __wrap_eMBMasterWaitRequestFinish( void )
{
    eMBMasterReqErrCode event = __real_eMBMasterWaitRequestFinish();
    UT_LOGI("REQ", "Request event get: %x", event);
    return event;
}


#ifdef __cplusplus
}
#endif

#endif
