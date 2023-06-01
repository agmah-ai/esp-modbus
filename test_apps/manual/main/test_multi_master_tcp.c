/*
 * SPDX-FileCopyrightText: 2018-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <string.h>
#include "unity.h"
#include "unity_test_runner.h"
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
#include "mbcontroller.h"   // for common Modbus defines

#define MASTER_MAX_CIDS             (2)
#define MASTER_MAX_RETRY            (10000)
#define MB_TCP_TASK_STACK_SIZE      (4096)
#define MB_TCP_TASK_PRIO            (10)
#define MB_TCP_TEST_CYCLE_LENGTH    (10)

#define TAG                         "MODBUS_TEST"
#define STR(fieldname)              ((const char *)( fieldname ))
#define OPTS(min_val, max_val, step_val) { .opt1 = min_val, .opt2 = max_val, .opt3 = step_val }

#define MASTER_CHECK(a, ret_val, str, log_tag, ...) \
    if (!(a)) { \
        ESP_LOGE(log_tag, "%s(%u): " str, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
        return (ret_val); \
    }

// Enumeration of modbus slave addresses accessed by master device
enum {
    MB_DEVICE_ADDR1 = 1,
    MB_DEVICE_ADDR2 = 200,
    MB_DEVICE_ADDR3 = 2,
};

// Enumeration of all supported CIDs for device (used in parameter definition table)
enum {
    CID_DEV_REG0 = 0,
    CID_DEV_REG1,
    CID_DEV_REG2,
    CID_DEV_REG3,
};

// Example Data (Object) Dictionary for Modbus parameters
static const mb_parameter_descriptor_t characteristics1[] = {
    // CID, Name, Units, Modbus addr, register type, Modbus Reg Start Addr, Modbus Reg read length, 
    // Instance offset (NA), Instance type, Instance length (bytes), Options (NA), Permissions
    { CID_DEV_REG0, STR("MB_hold_reg-00"), STR("Data"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x0010, 1,
                    0, PARAM_TYPE_U16, 2, OPTS( 0,0,0 ), PAR_PERMS_READ_WRITE_TRIGGER },
    { CID_DEV_REG1, STR("MB_hold_reg-01"), STR("Data"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x0011, 1,
                    0, PARAM_TYPE_U16, 2, OPTS( 0,0,0 ), PAR_PERMS_READ_WRITE_TRIGGER },
    { CID_DEV_REG2, STR("MB_hold_reg-02"), STR("Data"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x0012, 1,
                    0, PARAM_TYPE_U16, 2, OPTS( 0,0,0 ), PAR_PERMS_READ_WRITE_TRIGGER },
    { CID_DEV_REG3, STR("MB_hold_reg-03"), STR("Data"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x0013, 1,
                    0, PARAM_TYPE_U16, 2, OPTS( 0,0,0 ), PAR_PERMS_READ_WRITE_TRIGGER }
};

static const mb_parameter_descriptor_t characteristics2[] = {
    // CID, Name, Units, Modbus addr, register type, Modbus Reg Start Addr, Modbus Reg read length, 
    // Instance offset (NA), Instance type, Instance length (bytes), Options (NA), Permissions
    { CID_DEV_REG0, STR("MB_hold_reg-10"), STR("Data"), MB_DEVICE_ADDR2, MB_PARAM_HOLDING, 0x0010, 1,
                    0, PARAM_TYPE_U16, 2, OPTS( 0,0,0 ), PAR_PERMS_READ_WRITE_TRIGGER },
    { CID_DEV_REG1, STR("MB_hold_reg-11"), STR("Data"), MB_DEVICE_ADDR2, MB_PARAM_HOLDING, 0x0011, 1,
                    0, PARAM_TYPE_U16, 2, OPTS( 0,0,0 ), PAR_PERMS_READ_WRITE_TRIGGER },
    { CID_DEV_REG2, STR("MB_hold_reg-12"), STR("Data"), MB_DEVICE_ADDR2, MB_PARAM_HOLDING, 0x0012, 1, //MB_SLAVE_ADDR_PLACEHOLDER
                    0, PARAM_TYPE_U16, 2, OPTS( 0,0,0 ), PAR_PERMS_READ_WRITE_TRIGGER },
    { CID_DEV_REG3, STR("MB_hold_reg-13"), STR("Data"), MB_DEVICE_ADDR2, MB_PARAM_HOLDING, 0x0013, 1,
                    0, PARAM_TYPE_U16, 2, OPTS( 0,0,0 ), PAR_PERMS_READ_WRITE_TRIGGER }
};

// Calculate number of parameters in the table
const uint16_t num_characteristics1 = (sizeof(characteristics1)/sizeof(characteristics1[0]));
const uint16_t num_characteristics2 = (sizeof(characteristics2)/sizeof(characteristics2[0]));

// This table represents slave IP addresses that correspond to the short address field of the slave in device_parameters structure
// Modbus TCP stack shall use these addresses to be able to connect and read parameters from slave
const char *slave_ip_table1[] = {
    //"01;192.168.32.206;502",     // Address corresponds to MB_DEVICE_ADDR1 and set to predefined value by user
    "01;192.168.88.251;502",     // Corresponds to characteristic MB_DEVICE_ADDR2 "mb_slave_tcp_C8"
    "01;192.168.88.254;502",
    //"01;mb_slave_tcp_01;502",
    //"200;mb_slave_tcp_c8;502",
    //"0120;2001:0db8:85a3:0000:0000:8a2e:0370:7334;1502",     // Corresponds to characteristic MB_DEVICE_ADDR3 
    NULL             // End of table condition (must be included)
};

// This table represents slave IP addresses that correspond to the short address field of the slave in device_parameters structure
// Modbus TCP stack shall use these addresses to be able to connect and read parameters from slave
const char *slave_ip_table2[] = {
    "200;192.168.88.253;502",
    "01;192.168.88.251;502",
    //"01;mb_slave_tcp_01;502",
    //"200;mb_slave_tcp_c8;502",     // Address corresponds to MB_DEVICE_ADDR1 and set to predefined value by user
    //"mb_slave_tcp_01",     // Corresponds to characteristic MB_DEVICE_ADDR2
    NULL,     // Corresponds to characteristic MB_DEVICE_ADDR3
    NULL             // End of table condition (must be included)
};

const size_t slave_ip_table_sz1 = (size_t)(sizeof(slave_ip_table1) / sizeof(slave_ip_table1[0]));
const size_t slave_ip_table_sz2 = (size_t)(sizeof(slave_ip_table2) / sizeof(slave_ip_table2[0]));

static esp_err_t read_modbus_parameter(void *handle, uint16_t cid, uint16_t *par_data)
{
    const mb_parameter_descriptor_t *param_descriptor = NULL;
    
    esp_err_t err = mbc_master_get_cid_info(handle, cid, &param_descriptor);
    if ((err != ESP_ERR_NOT_FOUND) && (param_descriptor != NULL)) {
        uint8_t type = 0;
        // This is just for test the functionality to override UID of slave
        if (param_descriptor->mb_slave_addr == MB_SLAVE_ADDR_PLACEHOLDER) {
            err = mbc_master_get_parameter_with(handle, cid, MB_DEVICE_ADDR2, (uint8_t *)par_data, &type);
        } else {
            err = mbc_master_get_parameter(handle, cid, (uint8_t *)par_data, &type);
        }
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Read mbm_%p, #%d %s (%s) value = (0x%x) parameter read successful.",
                                     handle,
                                     param_descriptor->cid,
                                     (char *)param_descriptor->param_key,
                                     (char *)param_descriptor->param_units,
                                     *(uint16_t *)par_data);
        }
        else 
        {
            ESP_LOGE(TAG, "Read mbm_%p, #%d %s (%s), parameter read fail.", 
                                    handle,
                                    param_descriptor->cid,
                                    (char *)param_descriptor->param_key,
                                    (char *)param_descriptor->param_units);
        }
    }
    return err;  
}

static esp_err_t write_modbus_parameter(void *handle, uint16_t cid, uint16_t *par_data)
{
    const mb_parameter_descriptor_t *param_descriptor = NULL;
    
    esp_err_t err = mbc_master_get_cid_info(handle, cid, &param_descriptor);
    if ((err != ESP_ERR_NOT_FOUND) && (param_descriptor != NULL)) {
        uint8_t type = 0; // type of parameter from dictionary
        if (param_descriptor->mb_slave_addr == MB_SLAVE_ADDR_PLACEHOLDER) {
            err = mbc_master_set_parameter_with(handle, cid, MB_DEVICE_ADDR2, (uint8_t*)par_data, &type);
        } else {
            err = mbc_master_set_parameter(handle, cid, (uint8_t *)par_data, &type);
        }
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Write mbm_%p, #%d %s (%s) value = (0x%x), write successful.",
                                        (void *)handle,
                                        param_descriptor->cid,
                                        (char *)param_descriptor->param_key,
                                        (char *)param_descriptor->param_units,
                                        *(uint16_t *)par_data);
        } else {
            ESP_LOGE(TAG, "Write mbm_%p, #%d (%s) write fail, err = 0x%x (%s).",
                                    (void *)handle,
                                    param_descriptor->cid,
                                    (char *)param_descriptor->param_key,
                                    (int)err,
                                    (char *)esp_err_to_name(err));
        }
    }
    return err;  
}

static TaskHandle_t master_task_handle = NULL;

typedef enum {
    RT_HOLDING_RD,
    RT_HOLDING_WR
} mb_reg_access_t;

static uint16_t holding_registers[] = {
    0x1122, 0x1122, 0x1122, 0x1122,
    0x1122, 0x1122, 0x1122, 0x1122,
    0x1122, 0x1122, 0x1122, 0x1122,
    0x1122, 0x1122, 0x1122, 0x1122
};

static uint16_t input_registers[] = {
    0xAA55, 0xAA55, 0xAA55, 0xAA55,
    0xAA55, 0xAA55, 0xAA55, 0xAA55,
    0xAA55, 0xAA55, 0xAA55, 0xAA55,
    0xAA55, 0xAA55, 0xAA55, 0xAA55
};

static uint16_t coil_registers[] = {
    0xAA55,
    0xAA55,
    0xAA55,
    0xAA55
};

// This is user function to read and write modbus holding registers
static void master_task1(void *pmbm_handle)
{
    static mb_reg_access_t req_type = RT_HOLDING_RD;
    esp_err_t err = ESP_FAIL;

    ESP_LOGI(TAG, "Start modbus test task 1...");
    TEST_ASSERT_TRUE(pmbm_handle);

    TEST_ESP_OK(mbc_master_start(pmbm_handle));
    holding_registers[0] = 0x1111;
    holding_registers[1] = 0x2222;
    holding_registers[2] = 0x3333;
    holding_registers[3] = 0x4444;
    write_modbus_parameter(pmbm_handle, CID_DEV_REG0, &holding_registers[0]);
    write_modbus_parameter(pmbm_handle, CID_DEV_REG1, &holding_registers[1]);
    write_modbus_parameter(pmbm_handle, CID_DEV_REG2, &holding_registers[2]);
    write_modbus_parameter(pmbm_handle, CID_DEV_REG3, &holding_registers[3]);
    uint32_t cycle_counter = MB_TCP_TEST_CYCLE_LENGTH;
    while(cycle_counter--) // 
    {
        switch (req_type)
        {
        case RT_HOLDING_RD:
            err = read_modbus_parameter(pmbm_handle, CID_DEV_REG0, &holding_registers[0]);
            if (holding_registers[0] != 0x1111) {
                ESP_LOGE(TAG, "%p, REG0 = 0x%x, is incorrect", pmbm_handle, holding_registers[0]);
            }
            err |= read_modbus_parameter(pmbm_handle, CID_DEV_REG1, &holding_registers[1]);
            if (holding_registers[1] != 0x2222) {
                ESP_LOGE(TAG, "%p, REG1 = 0x%x, is incorrect", pmbm_handle, holding_registers[1]);
            }
            err |= read_modbus_parameter(pmbm_handle, CID_DEV_REG2, &holding_registers[2]);
            if (holding_registers[2] != 0x3333) {
                ESP_LOGE(TAG, "%p, REG2 = 0x%x, is incorrect", pmbm_handle, holding_registers[2]);
            }
            err |= read_modbus_parameter(pmbm_handle, CID_DEV_REG3, &holding_registers[3]);
            if (holding_registers[3] != 0x4444) {
                ESP_LOGE(TAG, "%p, REG3 = 0x%x, is incorrect", pmbm_handle, holding_registers[3]);
            }
            req_type = RT_HOLDING_WR;
            break;

        case RT_HOLDING_WR:

            err = write_modbus_parameter(pmbm_handle, CID_DEV_REG0, &holding_registers[0]);
            err |= write_modbus_parameter(pmbm_handle, CID_DEV_REG1, &holding_registers[1]);
            err |= write_modbus_parameter(pmbm_handle, CID_DEV_REG2, &holding_registers[2]);
            err |= write_modbus_parameter(pmbm_handle, CID_DEV_REG3, &holding_registers[3]);
            req_type = RT_HOLDING_RD;
            break;
        default:
            break;
        }
        vTaskDelay(1);
    }
    TEST_ESP_OK(mbc_master_stop(pmbm_handle));
    ESP_LOGI(TAG, "Modbus master stack destroy...");
    TEST_ESP_OK(mbc_master_delete(pmbm_handle));
    ESP_LOGI(TAG, "Modbus test task1 is completed.");
    vTaskDelete(NULL);
}

// This is user function to read and write modbus holding registers
static void master_task2(void *phandle)
{
    static mb_reg_access_t req_type = RT_HOLDING_RD;
    esp_err_t err = ESP_FAIL;

    ESP_LOGI(TAG, "Start modbus test task2...");
    TEST_ASSERT_TRUE(phandle);

    TEST_ESP_OK(mbc_master_start(phandle));
    holding_registers[2] = 0x3333;
    holding_registers[3] = 0x4444;
    write_modbus_parameter(phandle, CID_DEV_REG2, &holding_registers[2]);
    write_modbus_parameter(phandle, CID_DEV_REG3, &holding_registers[3]);
    uint32_t cycle_counter = MB_TCP_TEST_CYCLE_LENGTH;
    while(cycle_counter--) //cycle_counter--
    {
        switch (req_type)
        {
            case RT_HOLDING_RD:
                err = read_modbus_parameter(phandle, CID_DEV_REG2, &holding_registers[2]);
                err |= read_modbus_parameter(phandle, CID_DEV_REG3, &holding_registers[3]);
                req_type = RT_HOLDING_WR;
                break;
            
            case RT_HOLDING_WR:
                err = write_modbus_parameter(phandle, CID_DEV_REG2, &holding_registers[2]);
                err |= write_modbus_parameter(phandle, CID_DEV_REG3, &holding_registers[3]);
                req_type = RT_HOLDING_RD;
                break;

            default:
                break;
        }
        vTaskDelay(1);
    }
    TEST_ESP_OK(mbc_master_stop(phandle));
    ESP_LOGI(TAG, "Modbus master stack destroy...");
    TEST_ESP_OK(mbc_master_delete(phandle));
    ESP_LOGI(TAG, "Modbus test task2 is completed.");
    vTaskDelete(NULL);
}

void master_init(mb_communication_info_t *pcomm, void **pmbm_handle) 
{
    //static TaskHandle_t task_handle = NULL;

    void *handle = NULL;

    TEST_ESP_OK(mbc_master_create_tcp(pcomm, &handle));

    *pmbm_handle = handle;
}

static TaskHandle_t task_handle1 = NULL;
static TaskHandle_t task_handle2 = NULL;

TEST_CASE("test setup modbus multi master", "[modbus]")
{
    esp_err_t result = nvs_flash_init();
    if (result == ESP_ERR_NVS_NO_FREE_PAGES || result == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      result = nvs_flash_init();
    }
    
    MB_RETURN_ON_FALSE((result == ESP_OK), ;,
                            TAG,
                            "nvs_flash_init fail, returns(0x%x).",
                            (uint16_t)result);
    result = esp_netif_init();
    MB_RETURN_ON_FALSE((result == ESP_OK), ;,
                            TAG,
                            "esp_netif_init fail, returns(0x%x).",
                            (uint16_t)result);
    result = esp_event_loop_create_default();
    MB_RETURN_ON_FALSE((result == ESP_OK), ;,
                            TAG,
                            "esp_event_loop_create_default fail, returns(0x%x).",
                            (uint16_t)result);
    result = example_connect();
    MB_RETURN_ON_FALSE((result == ESP_OK), ;,
                            TAG,
                            "example_connect fail, returns(0x%x).",
                            (uint16_t)result);

#if CONFIG_EXAMPLE_CONNECT_WIFI
   result = esp_wifi_set_ps(WIFI_PS_NONE);
   MB_RETURN_ON_FALSE((result == ESP_OK), ;,
                            TAG, "esp_wifi_set_ps fail, returns(0x%x).", (uint16_t)result);
#endif

    mb_communication_info_t tcp_par_1 = {
        .tcp_opts.port = 502,
        .tcp_opts.mode = MB_TCP,
        .tcp_opts.addr_type = MB_IPV4,
        .tcp_opts.ip_addr_table = (void *)slave_ip_table1,
        .tcp_opts.slave_addr = 0,
    };
    tcp_par_1.tcp_opts.ip_netif_ptr = (void *)get_example_netif();

    void *mbm_handle1 = NULL;

    master_init(&tcp_par_1, &mbm_handle1);

    ESP_LOGI(TAG, "Modbus master stack2 is started...");

    TEST_ESP_OK(mbc_master_set_descriptor(mbm_handle1, &characteristics1[0], num_characteristics1));
    ESP_LOGI(TAG, "Modbus master stack is initialized...");

    xTaskCreatePinnedToCore(master_task1, "master_tcp_task1",
                                            MB_TCP_TASK_STACK_SIZE,
                                            mbm_handle1, (MB_TCP_TASK_PRIO - 1),
                                            &task_handle1, MB_PORT_TASK_AFFINITY);

    mb_communication_info_t tcp_par_2 = {
        .tcp_opts.port = 502,
        .tcp_opts.mode = MB_TCP,
        .tcp_opts.addr_type = MB_IPV4,
        .tcp_opts.ip_addr_table = (void *)slave_ip_table2,
        .tcp_opts.slave_addr = 0,
    };
    
    tcp_par_2.tcp_opts.ip_netif_ptr = (void *)get_example_netif();

    void *mbm_handle2 = NULL;

    master_init(&tcp_par_2, &mbm_handle2);

    ESP_LOGI(TAG, "Modbus master stack2 is started...");

    TEST_ESP_OK(mbc_master_set_descriptor(mbm_handle2, &characteristics2[0], num_characteristics2));
    ESP_LOGI(TAG, "Modbus master stack is initialized...");

    xTaskCreatePinnedToCore(master_task2, "master_tcp_task2",
                                            MB_TCP_TASK_STACK_SIZE,
                                            mbm_handle2, (MB_TCP_TASK_PRIO - 2),
                                            &task_handle2, MB_PORT_TASK_AFFINITY);
                    
}
