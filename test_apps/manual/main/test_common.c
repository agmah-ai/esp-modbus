/*
 * SPDX-FileCopyrightText: 2018-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <string.h>
#include "unity.h"
#include "unity_test_runner.h"
#include "mbcontroller.h"   // for common Modbus defines
#include "string.h"
#include "esp_log.h"
#include "sdkconfig.h"

#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"

#if __has_include("esp_mac.h")
#include "esp_mac.h"
#endif

#include "protocol_examples_common.h"

#define MASTER_MAX_CIDS 2
#define MASTER_MAX_RETRY 10000
#define MASTER_PORT_NUM CONFIG_MB_UART_PORT_NUM
#define MASTER_SPEED CONFIG_MB_UART_BAUD_RATE
#define MB_UART_RXD_PIN CONFIG_MB_UART_RXD
#define MB_UART_TXD_PIN CONFIG_MB_UART_TXD
#define MB_UART_RTS_PIN CONFIG_MB_UART_RTS
#define MB_SERIAL_TASK_STACK_SIZE 4096
#define MB_SERIAL_TASK_PRIO 10

#define TAG "MODBUS_TEST_COMMON"

TEST_CASE("test modbus master serial RTU create - delete sequence", "[modbus]")
{
    // Initialization of device peripheral and objects
    // Initialize and start Modbus controller
    mb_communication_info_t comm = {
        .ser_opts.port = (MASTER_PORT_NUM),
        .ser_opts.mode = MB_RTU,
        .ser_opts.data_bits = UART_DATA_8_BITS,
        .ser_opts.stop_bits = UART_STOP_BITS_2,
        .ser_opts.baudrate = MASTER_SPEED,
        .ser_opts.parity = UART_PARITY_DISABLE
    };

    void *mbm_handle = NULL;
    ESP_LOGI(TAG, "Modbus master instance1 create...");

    TEST_ESP_OK(mbc_master_create_serial(&comm, &mbm_handle));
    TEST_ESP_OK(mbc_master_delete(mbm_handle));

    ESP_LOGI(TAG, "%p, Modbus master serial instance1 delete...", mbm_handle);

}

TEST_CASE("test modbus master serial ASCII create - delete sequence", "[modbus]")
{
    mb_communication_info_t comm_par = {
        .ser_opts.port = MASTER_PORT_NUM,
        .ser_opts.mode = MB_ASCII,
        .ser_opts.data_bits = UART_DATA_8_BITS,
        .ser_opts.stop_bits = UART_STOP_BITS_2,
        .ser_opts.baudrate = MASTER_SPEED,
        .ser_opts.parity = UART_PARITY_DISABLE
    };

    void *mbm_handle = NULL;

    ESP_LOGI(TAG, "Modbus master serial instance create...");

    TEST_ESP_OK(mbc_master_create_serial(&comm_par, &mbm_handle));
    TEST_ESP_OK(mbc_master_delete(mbm_handle));

    ESP_LOGI(TAG, "Modbus master serial instance delete...");

}

TEST_CASE("test modbus master TCP create - delete sequence", "[modbus]")
{
    // esp_err_t result = nvs_flash_init();
    // if (result == ESP_ERR_NVS_NO_FREE_PAGES || result == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    //   ESP_ERROR_CHECK(nvs_flash_erase());
    //   result = nvs_flash_init();
    // }
    
    // result = esp_netif_init();
    // MB_RETURN_ON_FALSE((result == ESP_OK), ;,
    //                         TAG,
    //                         "esp_netif_init fail, returns(0x%x).",
    //                         (uint16_t)result);
    
    // result = esp_event_loop_create_default();
    // MB_RETURN_ON_FALSE((result == ESP_OK), ;,
    //                         TAG,
    //                         "esp_event_loop_create_default fail, returns(0x%x).",
    //                         (uint16_t)result);
    // result = example_connect();
    // MB_RETURN_ON_FALSE((result == ESP_OK), ;,
    //                         TAG,
    //                         "example_connect fail, returns(0x%x).",
    //                         (uint16_t)result);
    
    const char *slave_ip_table[] = {
        "01;127.0.0.1;502",
        NULL
    };
  
    mb_communication_info_t tcp_par = {
        .tcp_opts.port = 502,
        .tcp_opts.mode = MB_TCP,
        .tcp_opts.addr_type = MB_IPV4,
        .tcp_opts.ip_addr_table = (void *)slave_ip_table,
        .tcp_opts.slave_addr = 0,
    };
    
    tcp_par.tcp_opts.ip_netif_ptr = (void *)get_example_netif();

    void *handle = NULL;
    ESP_LOGI(TAG, "Start modbus stack...");

    TEST_ESP_OK(mbc_master_create_tcp(&tcp_par, &handle));

    ESP_LOGI(TAG, "Destroy modbus stack...");

    TEST_ESP_OK(mbc_master_delete(handle));

    //TEST_ESP_OK(mbc_master_set_descriptor(handle, &characteristics[0], 1));
    //ESP_LOGI(TAG, "Modbus master stack is initialized...");
    // example_disconnect();
    // esp_event_loop_delete_default();
    // esp_netif_deinit();
    // nvs_flash_deinit();
}
