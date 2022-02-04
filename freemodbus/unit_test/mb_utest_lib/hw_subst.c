/*
 * SPDX-FileCopyrightText: 2015-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "hw_subst.h"

static esp_netif_t* pnetif = NULL;

esp_netif_t *__wrap_get_example_netif(void)
{
    ESP_LOGW("UT", "Func wrapper called: %s.", __func__);
    return pnetif;
}

esp_err_t __wrap_esp_netif_init(void)
{
    ESP_LOGW("UT", "Func wrapper called: %s.", __func__);
    //pnetif = (esp_netif_t *)get_example_netif_from_desc("sta"); //esp_netif_init();
    esp_netif_config_t netif_config = ESP_NETIF_DEFAULT_WIFI_STA();
    pnetif = esp_netif_new(&netif_config);
    return pnetif ? ESP_OK : ESP_ERR_INVALID_STATE;
}

esp_err_t __wrap_esp_netif_deinit(void) 
{
    ESP_LOGW("UT", "Func wrapper called: %s.", __func__);
    esp_netif_destroy(pnetif);
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t __wrap_esp_wifi_set_ps(wifi_ps_type_t type)
{
    ESP_LOGW("UT", "Func wrapper called: %s.", __func__);
    return ESP_OK;
}

esp_err_t __wrap_example_connect(void)
{
    ESP_LOGW("UT", "Func wrapper called: %s.", __func__);
    return ESP_OK;
}

esp_err_t __wrap_example_disconnect(void)
{
    ESP_LOGW("UT", "Func wrapper called: %s.", __func__);
    return ESP_OK;
}

esp_err_t __wrap_init_services(mb_tcp_addr_type_t ip_addr_type)
{
    ESP_LOGE("UT", "Func wrapper called: %s.", __func__);
    return ESP_OK;
}

void __wrap_lwip_freeaddrinfo(struct addrinfo *ai)
{
    ESP_LOGW("UT", "Func wrapper called: %s.", __func__);
    free(ai->ai_addr);
    //free(ai->ai_canonname);
    free(ai);
}

int __wrap_lwip_getaddrinfo(const char *nodename,
       const char *servname,
       const struct addrinfo *addrinfo,
       struct addrinfo **res) 
{
    ESP_LOGW("UT", "Func wrapper called: %s.", __func__);
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
    ESP_LOGW(__func__, "Set node addr: %s.", nodename);
    return ERR_OK;
}

int curr_socket = 55;

int __wrap_lwip_socket(int domain, int type, int protocol)
{
    ESP_LOGW("UT", "Func wrapper called: %s, %d.", __func__, curr_socket);
    return curr_socket++; // curr_socket++
}

int __wrap_lwip_connect(int s,const struct sockaddr *name,socklen_t namelen)
{
    ESP_LOGW("UT", "Func wrapper called: %s.", __func__);
    return ERR_OK;
}

ssize_t __wrap_lwip_recv(int s, void* mem, size_t len, int flags)
{
    ESP_LOGW("UT", "%s, sock: %d, ptr:%p, len:%d", __func__, s, mem, len);   
    return ut_get_stream_data(DIRECTION_INPUT, mem, len, 100);
}

ssize_t __wrap_lwip_send(int s, const void *dataptr, size_t size, int flags)
{
    ESP_LOGW("UT", "Func wrapper called: %s.", __func__);
    ESP_LOG_BUFFER_HEXDUMP("UT_SEND_SRC", dataptr, size, ESP_LOG_WARN);
    void *temp_data = (void *)dataptr;
    int length = ut_get_stream_data(DIRECTION_OUTPUT, temp_data, size, 100);
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
        err = ut_get_notification(DIRECTION_INPUT, *timeout);
        if (err == ESP_OK) {
            FD_SET(curr_socket, readset);
            ESP_LOGW("UT_READ", " %s.", __func__);
        }
    }
    if (writeset) {
        err = ut_get_notification(DIRECTION_OUTPUT, *timeout);
        if (err == ESP_OK) {
            FD_SET(curr_socket, writeset);
            ESP_LOGW("UT_WRITE", " %s.", __func__);
        } else {
            ESP_LOGE("UT_WRITE", "%s write error.", __func__);
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
    ESP_LOGW("UT", "Func wrapper called: %s.", __func__);
    return ERR_OK;
}

int __wrap_lwip_getsockopt(int s, int level, int optname, void *optval, socklen_t *optlen)
{
    ESP_LOGW("UT", "Func wrapper called: %s.", __func__);
    if (optval) {
        *(int*)optval = 0;
    }
    return ERR_OK;
}

int __wrap_lwip_bind(int s, const struct sockaddr *name, socklen_t namelen)
{
    ESP_LOGW("UT", "Func wrapper called: %s.", __func__);
    return 0;
}

int __wrap_lwip_listen(int s, int backlog)
{
    ESP_LOGW("UT", "Func wrapper called: %s.", __func__);
    return 0;
}

int __wrap_lwip_setsockopt(int s, int level, int optname, const void *optval, socklen_t optlen)
{
    ESP_LOGW("UT", "Func wrapper called: %s.", __func__);
    return 0;
}

int __wrap_lwip_accept(int s, struct sockaddr *addr, socklen_t *addrlen)
{
    ESP_LOGW("UT", "Func wrapper called: %s.", __func__);
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
    ESP_LOGW("UT", "Func wrapper called: %s.", __func__);
    curr_socket = 55;
    return 0;
}

int __wrap_lwip_close(int s)
{
    ESP_LOGW("UT", "Func wrapper called: %s.", __func__);
    curr_socket = 55;
    return 0;    
}
