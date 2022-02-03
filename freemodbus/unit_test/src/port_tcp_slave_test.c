#warning "include slave test source"

#include "sdkconfig.h"

#include "esp_modbus_slave.h"

#include "ut_io.h"

#include "port_tcp_slave_test.h"
// Include the moddule under test to get access to its static functions/variables for test porpose
#include "port_tcp_slave.c"

#include "mbport_stubs.h"

static int inp_stream_id = -1;
static int out_stream_id = -1;

// This is the place to place internal interface test functions
// This code has access to all static functions of the port file

BOOL __wrap_xMBTCPPortInit( USHORT usTCPPort )
{
    UT_PORT_INIT("mbs_tcp");
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