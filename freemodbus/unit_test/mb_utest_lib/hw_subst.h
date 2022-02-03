 /*
  * SPDX-FileCopyrightText: 2018-2022 Espressif Systems (Shanghai) CO LTD
  *
  * SPDX-License-Identifier: Apache-2.0
  */

#pragma once

#include "lwip/err.h"                   // needs just for exposing of the TCP stack port functions
#include "lwip/netdb.h"                 // for getaddrinfo
#include "lwip/sockets.h"               // for socket
#include "esp_wifi_types.h"             // for esp_wifi_set_ps()
#include "esp_netif.h"                  // for esp_netif_t
#include "lwip/ip_addr.h"               // for ip_addr_t
#include "esp_modbus_common.h"          // for common modbus defines
#include "ut_io.h"

#include "port.h"                       // for modbus port types

#include "sdkconfig.h"

// Example network common wrappers
esp_netif_t* __wrap_get_example_netif(void);
esp_err_t __wrap_esp_netif_init(void);
esp_err_t __wrap_esp_wifi_set_ps(wifi_ps_type_t type);
esp_err_t __wrap_example_connect(void);
esp_err_t __wrap_example_disconnect(void);
esp_err_t __wrap_esp_netif_deinit(void);
esp_err_t __wrap_init_services(mb_tcp_addr_type_t ip_addr_type);

// Wrappers for network layer API
void __wrap_lwip_freeaddrinfo(struct addrinfo *ai);
int __wrap_lwip_getaddrinfo(const char *nodename, const char *servname, const struct addrinfo *addrinfo, struct addrinfo **res);
int __wrap_lwip_socket(int domain, int type, int protocol);
int __wrap_lwip_connect(int s,const struct sockaddr *name,socklen_t namelen);
ssize_t __wrap_lwip_recv(int s, void* mem, size_t len, int flags);
ssize_t __wrap_lwip_send(int s, const void *dataptr, size_t size, int flags);
int __wrap_select(int maxfdp1, fd_set *readset, fd_set *writeset, fd_set *exceptset,
                struct timeval *timeout);
int __wrap_fcntl(int s, int cmd, ...);
int __wrap_lwip_getsockopt(int s, int level, int optname, void *optval, socklen_t *optlen);
int __wrap_lwip_bind(int s, const struct sockaddr *name, socklen_t namelen);
int __wrap_lwip_listen(int s, int backlog);
int __wrap_lwip_setsockopt(int s, int level, int optname, const void *optval, socklen_t optlen);
int __wrap_lwip_accept(int s, struct sockaddr *addr, socklen_t *addrlen);
int __wrap_lwip_shutdown(int s, int how);
int __wrap_lwip_close(int s);
extern int lwip_connect(int s,const struct sockaddr *name,socklen_t namelen);
esp_err_t __wrap_uart_wait_tx_done(uart_port_t uart_num, TickType_t ticks_to_wait);
int __wrap_uart_read_bytes(uart_port_t uart_num, void *buf, uint32_t length, TickType_t ticks_to_wait);
BOOL __wrap__xMBPortSerialWaitEvent(QueueHandle_t xQueueHandle, uart_event_t* pxEvent, ULONG xTimeout);
extern BOOL __real_xMBPortSerialWaitEvent(QueueHandle_t xQueueHandle, uart_event_t* pxEvent, ULONG xTimeout);