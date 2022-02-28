#warning"dependency injection for port_tcp_slave is working"
#pragma once
#include "sdkconfig.h"

#if CONFIG_MB_UTEST

#include "mbport_stubs.h"
#include "port.h"

#if CONFIG_MB_UTEST_OVERRIDE

#include "hw_subst.h"

#define example_disconnect __wrap_example_disconnect
#define esp_netif_deinit __wrap_esp_netif_deinit

#define lwip_freeaddrinfo __wrap_lwip_freeaddrinfo
#define lwip_getaddrinfo __wrap_lwip_getaddrinfo
#define recv __wrap_lwip_recv
#define send __wrap_lwip_send
#define socket __wrap_lwip_socket
#define send __wrap_lwip_send
#define select __wrap_select
#define fcntl __wrap_fcntl
#define getsockopt __wrap_lwip_getsockopt
#define bind __wrap_lwip_bind
#define listen __wrap_lwip_listen
#define setsockopt __wrap_lwip_setsockopt
#define accept __wrap_lwip_accept
#define shutdown __wrap_lwip_shutdown
#define close __wrap_lwip_close

#endif

extern BOOL __real_xMBTCPPortInit(USHORT usTCPPort);

extern BOOL __real_xMBTCPPortSendResponse(UCHAR * pucMBTCPFrame, USHORT usTCPLength);

extern BOOL __real_xMBTCPPortGetRequest(UCHAR ** ppucMBTCPFrame, USHORT * usTCPLength);

extern void __real_vMBTCPPortClose(void);

#else
#warning "dependency injection for port_tcp_slave skipped"
#endif