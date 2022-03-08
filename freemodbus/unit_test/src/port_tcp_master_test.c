#warning "include master test source"

//#include "port_tcp_master.h"            // required for modbus tcp port types (the header exposed by cmake file)
#include "esp_modbus_master.h"

#include "ut_io.h"
//#include "mbport_stubs.h"

#include "port_tcp_master_test.h"

// Include the moddule under test to get access to its static functions/variables for test porpose
#include "port_tcp_master.c"

static int inp_stream_id = -1;
static int out_stream_id = -1;

BOOL __wrap_xMBMasterTCPPortInit( USHORT usTCPPort )
{
    UT_PORT_INIT("mbm_tcp");
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

err_t test_xMBTCPPortMasterConnect(MbSlaveInfo_t* pxInfo)
{
    MbSlaveInfo_t* pxTempInfo = calloc(1, sizeof(MbSlaveInfo_t));
    err_t err = xMBTCPPortMasterConnect(pxTempInfo);
    free(pxTempInfo);
    return err;
}

bool test_xMBTCPPortMasterCheckHost(const char* pcHostStr, ip_addr_t* pxHostAddr)
{
    const char* host_str = "192.168.1.3";
    ip_addr_t host_ip;
    IP4_ADDR(&host_ip.u_addr.ip4, 0,0,0,0);

    bool result = xMBTCPPortMasterCheckHost(host_str, &host_ip);
    MB_PORT_CHECK(result, FALSE, "FAIL to add slave IP address: [%s].", host_str);
    ip4_addr_t ipaddr ;
    IP4_ADDR(&ipaddr, 192,168,1,3);
    
    if (((uint32_t)host_ip.u_addr.ip4.addr == (uint32_t)ipaddr.addr)) {
        result = true;
    } else {
        result = false;
    }
    return result;
}