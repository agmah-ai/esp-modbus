/*
 * SPDX-FileCopyrightText: 2015-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "hw_subst.h"

static esp_netif_t* pnetif = NULL;

esp_netif_t *__wrap_get_example_netif(void)
{
    UT_LOGW("UT", "Func wrapper called: %s.", __func__);
    return pnetif;
}

esp_err_t __wrap_esp_netif_init(void)
{
    UT_LOGW("UT", "Func wrapper called: %s.", __func__);
    //pnetif = (esp_netif_t *)get_example_netif_from_desc("sta"); //esp_netif_init();
    esp_netif_config_t netif_config = ESP_NETIF_DEFAULT_WIFI_STA();
    pnetif = esp_netif_new(&netif_config);
    return pnetif ? ESP_OK : ESP_ERR_INVALID_STATE;
}

esp_err_t __wrap_esp_netif_deinit(void) 
{
    UT_LOGW("UT", "Func wrapper called: %s.", __func__);
    esp_netif_destroy(pnetif);
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t __wrap_esp_wifi_set_ps(wifi_ps_type_t type)
{
    UT_LOGW("UT", "Func wrapper called: %s.", __func__);
    return ESP_OK;
}

esp_err_t __wrap_example_connect(void)
{
    UT_LOGW("UT", "Func wrapper called: %s.", __func__);
    return ESP_OK;
}

esp_err_t __wrap_example_disconnect(void)
{
    UT_LOGW("UT", "Func wrapper called: %s.", __func__);
    return ESP_OK;
}

esp_err_t __wrap_init_services(mb_tcp_addr_type_t ip_addr_type)
{
    UT_LOGE("UT", "Func wrapper called: %s.", __func__);
    return ESP_OK;
}

void __wrap_lwip_freeaddrinfo(struct addrinfo *ai)
{
    UT_LOGW("UT", "Func wrapper called: %s.", __func__);
    free(ai->ai_addr);
    //free(ai->ai_canonname);
    free(ai);
}

int __wrap_lwip_getaddrinfo(const char *nodename,
       const char *servname,
       const struct addrinfo *addrinfo,
       struct addrinfo **res) 
{
    UT_LOGW("UT", "Func wrapper called: %s.", __func__);
    if (!nodename) {
        nodename = "127.0.0.1";
    }
    struct addrinfo* painfo = calloc(1, sizeof(struct addrinfo));
    painfo->ai_socktype = SOCK_DGRAM;
    painfo->ai_protocol = IPPROTO_TCP;
    painfo->ai_addrlen = sizeof(struct sockaddr_in);
    painfo->ai_family = AF_INET;
    *res = painfo;
    painfo->ai_addr = calloc(1, sizeof(struct sockaddr_in));
    struct sockaddr_in* psaddr = ((struct sockaddr_in *)(painfo->ai_addr));
    memset(psaddr, 0, sizeof(struct sockaddr_in));
    psaddr->sin_family = AF_INET;
    psaddr->sin_port = htons(502);
    psaddr->sin_len = sizeof(struct sockaddr_in);
    u32_t ipaddr = inet_addr(nodename);
    if (ipaddr != IPADDR_NONE) {
        psaddr->sin_addr.s_addr = ipaddr;
    } else {
        psaddr->sin_addr.s_addr = inet_addr("127.0.0.1");
    }
    painfo->ai_canonname = "test_host";
    UT_LOGW(__func__, "Set node addr: %s.", nodename);
    return ERR_OK;
}

int curr_socket = 55;

int __wrap_lwip_socket(int domain, int type, int protocol)
{
    UT_LOGW("UT", "Func wrapper called: %s, %d.", __func__, curr_socket);
    return curr_socket++; // curr_socket++
}

int __wrap_lwip_connect(int s,const struct sockaddr *name,socklen_t namelen)
{
    UT_LOGW("UT", "Func wrapper called: %s.", __func__);
    //lwip_connect(s, name, namelen);
    return ERR_OK;
}

ssize_t __wrap_lwip_recv(int s, void* mem, size_t len, int flags)
{
    UT_LOGW("UT", "%s, sock: %d, ptr:%p, len:%d", __func__, s, mem, len);   
    return ut_stream_get_data(STREAM_ID_INPUT, mem, len, 100);
}

ssize_t __wrap_lwip_send(int s, const void *dataptr, size_t size, int flags)
{
    UT_LOGW("UT", "Func wrapper called: %s.", __func__);
    ESP_LOG_BUFFER_HEXDUMP("UT_SEND_SRC", dataptr, size, ESP_LOG_WARN);
    void *temp_data = (void *)dataptr;
    int length = ut_stream_get_data(STREAM_ID_OUTPUT, temp_data, size, 100);
    return length;
}

int __wrap_select(int maxfdp1, fd_set *readset, fd_set *writeset, fd_set *exceptset,
                struct timeval *timeout)
{
    if (!timeout) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = ESP_FAIL;
    if (readset) {
        err = ut_stream_get_notification(STREAM_ID_INPUT, pdMS_TO_TICKS(get_time_us(*timeout) / 1000), NULL);
        if (err == ESP_OK) {
            FD_SET(curr_socket, readset);
            UT_LOGW("UT_READ", " %s.", __func__);
        }
    }
    if (writeset) {
        err = ut_stream_get_notification(STREAM_ID_OUTPUT, pdMS_TO_TICKS(get_time_us(*timeout) / 1000), NULL);
        if (err == ESP_OK) {
            FD_SET(curr_socket, writeset);
            UT_LOGW("UT_WRITE", " %s.", __func__);
        } else {
            UT_LOGE("UT_WRITE", "%s write error.", __func__);
        }
    }
    if (exceptset) {
        // do not handle errors yet
        FD_CLR((maxfdp1 - 1), exceptset);
    }
    return curr_socket;
}

int __wrap_fcntl(int s, int cmd, ...)
{
    UT_LOGW("UT", "Func wrapper called: %s.", __func__);
    return ERR_OK;
}

int __wrap_lwip_getsockopt(int s, int level, int optname, void *optval, socklen_t *optlen)
{
    UT_LOGW("UT", "Func wrapper called: %s.", __func__);
    if (optval) {
        *(int*)optval = 0;
    }
    return ERR_OK;
}

int __wrap_lwip_bind(int s, const struct sockaddr *name, socklen_t namelen)
{
    UT_LOGW("UT", "Func wrapper called: %s.", __func__);
    return 0;
}

int __wrap_lwip_listen(int s, int backlog)
{
    UT_LOGW("UT", "Func wrapper called: %s.", __func__);
    return 0;
}

int __wrap_lwip_setsockopt(int s, int level, int optname, const void *optval, socklen_t optlen)
{
    UT_LOGW("UT", "Func wrapper called: %s.", __func__);
    return 0;
}

int __wrap_lwip_accept(int s, struct sockaddr *addr, socklen_t *addrlen)
{
    UT_LOGW("UT", "Func wrapper called: %s.", __func__);
    if (!addr || !addrlen) {
        return -1;
    }
    struct sockaddr_in* psaddr = ((struct sockaddr_in *)(addr));
    psaddr->sin_addr.s_addr = inet_addr("127.0.0.1");
    psaddr->sin_addr.s_addr |= (uint32_t)(s & 0x03);
    
    addr->sa_family = PF_INET;
    addr->sa_len = *addrlen;

    return curr_socket;
}

int __wrap_lwip_shutdown(int s, int how)
{
    UT_LOGW("UT", "Func wrapper called: %s.", __func__);
    curr_socket = 55;
    return 0;
}

int __wrap_lwip_close(int s)
{
    UT_LOGW("UT", "Func wrapper called: %s.", __func__);
    curr_socket = 55;
    return 0;    
}

esp_err_t __wrap_uart_wait_tx_done(uart_port_t uart_num, TickType_t ticks_to_wait)
{
    ESP_LOGW(__func__, "Wait TX done UART %d, %u.", uart_num, ticks_to_wait);
    return (ESP_OK);
}

BOOL __wrap_xMBMasterPortSerialWaitEvent(uart_event_t* pxEvent, ULONG xTimeout)
{
    if (!pxEvent) {
        return FALSE;
    }
    esp_err_t err = ESP_FAIL;
    size_t event_length = 0;    
    err = ut_stream_get_ready(STREAM_ID_INPUT, xTimeout, &event_length);
    if ((err == ESP_OK) && (event_length > 0)) {
        if (pxEvent) {
            pxEvent->type = UART_DATA;
            pxEvent->timeout_flag = true;
            pxEvent->size = event_length;
        }
        UT_LOGW("UT_READ", " %s: %d bytes in buffer.", __func__, event_length);
        return TRUE;
    }
    return FALSE;
}

int __wrap_uart_read_bytes(uart_port_t uart_num, void *buf, uint32_t length, TickType_t ticks_to_wait)
{
    int ret = ut_stream_get_data(STREAM_ID_INPUT, buf, length, 100);
    UT_LOGW(__func__, "UART: %d, buf:%p, len:%d, tout:%u.", uart_num, buf, length, ticks_to_wait); 
    return ret;
}

int uart_write_bytes(uart_port_t uart_num, const void *src, size_t size)
{
    return size;
}