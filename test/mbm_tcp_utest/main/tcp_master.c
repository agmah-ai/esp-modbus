/*
 * SPDX-FileCopyrightText: 2018-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

// FreeModbus Master Example ESP32

#include "string.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "mbcontroller.h"
#include "sdkconfig.h"

#include "protocol_examples_common.h"
#include "ut_io.h"

//#include "port_tcp_master.c"

#define MASTER_TAG "MASTER_TCP"
#define MASTER_MAX_RETRY 10

#define STR(fieldname) ((const char*)( fieldname ))
#define OPTS(min_val, max_val, step_val) { .opt1 = min_val, .opt2 = max_val, .opt3 = step_val }

// Enumeration of modbus slave addresses accessed by master device
enum {
    MB_DEVICE_ADDR1 = 1
};

// Enumeration of all supported CIDs for device (used in parameter definition table)
enum {
    CID_DEV_REG0 = 0,
    CID_DEV_REG1
};

// Example Data (Object) Dictionary for Modbus parameters
const mb_parameter_descriptor_t device_parameters[] = {
    // CID, Name, Units, Modbus addr, register type, Modbus Reg Start Addr, Modbus Reg read length, 
    // Instance offset (NA), Instance type, Instance length (bytes), Options (NA), Permissions
    { CID_DEV_REG0, STR("MB_hold_reg-0"), STR("Data"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 2, 1,
                    0, PARAM_TYPE_U16, 2, OPTS( 0,0,0 ), PAR_PERMS_READ_WRITE_TRIGGER },
    { CID_DEV_REG1, STR("MB_hold_reg-1"), STR("Data"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 3, 1,
                    0, PARAM_TYPE_U16, 2, OPTS( 0,0,0 ), PAR_PERMS_READ_WRITE_TRIGGER }
};

// Calculate number of parameters in the table
const uint16_t num_device_parameters = (sizeof(device_parameters)/sizeof(device_parameters[0]));

char* slave_ip_address_table[ 2 ] = { 
        "192.168.1.239",
        NULL 
    };

static esp_err_t read_modbus_parameter(uint16_t cid, uint16_t *par_data)
{
    const mb_parameter_descriptor_t* param_descriptor = NULL;
    
    esp_err_t err = mbc_master_get_cid_info(cid, &param_descriptor);
    if ((err != ESP_ERR_NOT_FOUND) && (param_descriptor != NULL)) {
        uint8_t type = 0;
        err = mbc_master_get_parameter(cid, (char*)param_descriptor->param_key,
                                                        (uint8_t*)par_data, &type);
        if (err == ESP_OK) {
            ESP_LOGI(MASTER_TAG, "Characteristic #%d %s (%s) value = (0x%04x) parameter read successful.",
                                     param_descriptor->cid,
                                     (char*)param_descriptor->param_key,
                                     (char*)param_descriptor->param_units,
                                     *(uint16_t*)par_data);
        }
        else 
        {
            ESP_LOGE(MASTER_TAG, "Characteristic #%d %s (%s), parameter read fail.", 
                                    param_descriptor->cid,
                                    (char*)param_descriptor->param_key,
                                    (char*)param_descriptor->param_units);
        }
    }
    return err;  
}

static esp_err_t write_modbus_parameter(uint16_t cid, uint16_t *par_data)
{
    const mb_parameter_descriptor_t* param_descriptor = NULL;
    
    esp_err_t err = mbc_master_get_cid_info(cid, &param_descriptor);
    if ((err != ESP_ERR_NOT_FOUND) && (param_descriptor != NULL)) {
        uint8_t type = 0; // type of parameter from dictionary
        err = mbc_master_set_parameter(cid, (char*)param_descriptor->param_key,
                                                        (uint8_t*)par_data, &type);
        if (err == ESP_OK) {
            ESP_LOGI(MASTER_TAG, "Characteristic #%d %s (%s) value = (0x%04x), write successful.",
                                        param_descriptor->cid,
                                        (char*)param_descriptor->param_key,
                                        (char*)param_descriptor->param_units,
                                        *(uint16_t*)par_data);
        } else {
            ESP_LOGE(MASTER_TAG, "Characteristic #%d (%s) write fail, err = 0x%x (%s).",
                                    param_descriptor->cid,
                                    (char*)param_descriptor->param_key,
                                    (int)err,
                                    (char*)esp_err_to_name(err));
        }
    }
    return err;  
}

// This is user function to read and write modbus holding registers
static void master_read_write_func(void *arg)
{
    esp_err_t err = ESP_OK;
    uint16_t register_data = 0;
    
    ESP_LOGI(MASTER_TAG, "Start modbus test...");
    
    for(uint16_t retry = 0; retry <= MASTER_MAX_RETRY; retry++) {
        // Simply read your register here CID_DEV_REG0 - one register from address 0 (see device_parameters)
        err = read_modbus_parameter(CID_DEV_REG0, &register_data);
        // if read successful then increase value of the parameter
        // Insert your modbus read_write register functionality here
        register_data += 1;
        err = write_modbus_parameter(CID_DEV_REG0, &register_data);

        err = read_modbus_parameter(CID_DEV_REG1, &register_data);   
        register_data += 1;
        err = write_modbus_parameter(CID_DEV_REG1, &register_data);
    }
    ESP_ERROR_CHECK(mbc_master_destroy());
    ESP_LOGI(MASTER_TAG, "Modbus test is completed.");
}

esp_err_t init_services(mb_tcp_addr_type_t ip_addr_type)
{
    esp_err_t result = nvs_flash_init();
    if (result == ESP_ERR_NVS_NO_FREE_PAGES || result == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      result = nvs_flash_init();
    }
    UT_RETURN_ON_FALSE((result == ESP_OK), ESP_ERR_INVALID_STATE, MASTER_TAG,
                            "nvs_flash_init fail, returns(0x%x).",
                            (uint32_t)result);
    result = esp_netif_init();
    UT_RETURN_ON_FALSE((result == ESP_OK), ESP_ERR_INVALID_STATE, MASTER_TAG,
                            "esp_netif_init fail, returns(0x%x).",
                            (uint32_t)result);
    result = esp_event_loop_create_default();
    UT_RETURN_ON_FALSE((result == ESP_OK), ESP_ERR_INVALID_STATE, MASTER_TAG,
                            "esp_event_loop_create_default fail, returns(0x%x).",
                            (uint32_t)result);
    // This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
    // Read "Establishing Wi-Fi or Ethernet Connection" section in
    // examples/protocols/README.md for more information about this function.
    result = example_connect();
    UT_RETURN_ON_FALSE((result == ESP_OK), ESP_ERR_INVALID_STATE, MASTER_TAG,
                                "example_connect fail, returns(0x%x).",
                                (uint32_t)result);
#if CONFIG_EXAMPLE_CONNECT_WIFI
    result = esp_wifi_set_ps(WIFI_PS_NONE);
    UT_RETURN_ON_FALSE((result == ESP_OK), ESP_ERR_INVALID_STATE, MASTER_TAG,
                                    "esp_wifi_set_ps fail, returns(0x%x).",
                                    (uint32_t)result);
#endif
    return result;
}

// Modbus master initialization
static esp_err_t master_init(void)
{
    // Initialize and start Modbus controller

    mb_communication_info_t comm_info = {
        .ip_port = 502,
        .ip_addr_type = MB_IPV4,
        .ip_mode = MB_MODE_TCP,
        .ip_addr = (void*)slave_ip_address_table,

    };
    void* master_handler = NULL;
    UT_RETURN_ON_FALSE((init_services(comm_info.ip_addr_type) == ESP_OK), ESP_ERR_INVALID_STATE, MASTER_TAG,
                                "mb services initialization fail.");
                                
    comm_info.ip_netif_ptr = (void*)get_example_netif();

    esp_err_t err = mbc_master_init_tcp(&master_handler);
    UT_RETURN_ON_FALSE((master_handler != NULL), ESP_ERR_INVALID_STATE, MASTER_TAG,
                                "mb controller initialization fail.");
    UT_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, MASTER_TAG,
                            "mb controller initialization fail, returns(0x%x).",
                            (uint32_t)err);
    err = mbc_master_setup((void*)&comm_info);
    UT_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, MASTER_TAG,
                            "mb controller setup fail, returns(0x%x).", err);

    UT_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, MASTER_TAG,
            "mb serial set pin failure, uart_set_pin() returned (0x%x).", err);

    vTaskDelay(5);
    err = mbc_master_set_descriptor(&device_parameters[0], num_device_parameters);
    UT_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, MASTER_TAG,
                                "mb controller set descriptor fail, returns(0x%x).", err);
                                
    err = mbc_master_start();
    UT_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, MASTER_TAG,
                            "mb controller start fail, returns(0x%x).", err);
    ESP_LOGI(MASTER_TAG, "Modbus master stack initialized...");
    return err;
}

#include "esp_heap_trace.h"
#include "port_tcp_master_test.h"

#define NUM_RECORDS 200
static heap_trace_record_t trace_record[NUM_RECORDS]; // This buffer must be in internal RAM

void app_main(void)
{
    // Initialization of device peripheral and objects
    ESP_ERROR_CHECK(master_init());
    vTaskDelay(10);

    ESP_ERROR_CHECK( heap_trace_init_standalone(trace_record, NUM_RECORDS) );
    ESP_ERROR_CHECK( heap_trace_start(HEAP_TRACE_LEAKS) );

    uint8_t** pclientbuf = calloc(5, sizeof(uint8_t*));
    if (!pclientbuf) {
        ESP_LOGE("TEST", "TCP pclientbuf alloc failure.");
    }

    uint8_t* pdata = (uint8_t*)calloc(10, sizeof(uint8_t));
    pclientbuf[0] = pdata;
    pclientbuf[1] = pdata;
    pclientbuf[2] = pdata;

    ESP_LOGI("TEST", "pdata pointer= 0x%p", pdata);
    ESP_LOGI("TEST", "pclientbuf pointer= 0x%p(0x%p, 0x%p, 0x%p)", 
                            pclientbuf, *pclientbuf, pclientbuf[1], pclientbuf[2]);

    free(pclientbuf[0]);

    ESP_LOGI("TEST", "free pdata pointer= 0x%p", pdata);
    
    pdata = NULL;

    free(pclientbuf);
    ESP_LOGI("TEST", "free pclientbuf pointer= 0x%p", pclientbuf);
    pclientbuf = NULL;

    ESP_ERROR_CHECK( heap_trace_stop() );
    heap_trace_dump();
    ip_addr_t ipaddr;
    if (!test_xMBTCPPortMasterCheckHost("192.168.1.3", &ipaddr))
    {
        ESP_LOGE("APP_UT", "test_xMBTCPPortMasterCheckHost() test fail.");
    } else {
        ESP_LOGI("APP_UT", "test_xMBTCPPortMasterCheckHost() test passed.");
    }
    // Write registers to predefined state
    uint16_t register_data = 0x1234;
    write_modbus_parameter(CID_DEV_REG0, &register_data);
    register_data = 0x5678;
    write_modbus_parameter(CID_DEV_REG1, &register_data);

    master_read_write_func(NULL);
}



