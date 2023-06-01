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

#define MASTER_MAX_CIDS             (2)
#define MASTER_MAX_RETRY            (10000)
#define MASTER_PORT_NUM             (CONFIG_MB_UART_PORT_NUM)
#define MASTER_SPEED                (CONFIG_MB_UART_BAUD_RATE)
#define MB_UART_RXD_PIN             (CONFIG_MB_UART_RXD)
#define MB_UART_TXD_PIN             (CONFIG_MB_UART_TXD)
#define MB_UART_RTS_PIN             (CONFIG_MB_UART_RTS)
#define MB_SERIAL_TASK_STACK_SIZE   (4096)
#define MB_SERIAL_TASK_PRIO         (10)

#define TAG                         "MODBUS_TEST"
#define STR(fieldname)              ((const char*)( fieldname ))
#define OPTS(min_val, max_val, step_val) { .opt1 = min_val, .opt2 = max_val, .opt3 = step_val }

#define MASTER_CHECK(a, ret_val, str, log_tag, ...) \
    if (!(a)) { \
        ESP_LOGE(log_tag, "%s(%u): " str, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
        return (ret_val); \
    }

// Enumeration of modbus slave addresses accessed by master device
enum {
    MB_DEVICE_ADDR1 = 1,
    MB_DEVICE_ADDR2 = 2,
};

// Enumeration of all supported CIDs for device (used in parameter definition table)
enum {
    CID_DEV_REG0 = 0,
    CID_DEV_REG1,
    CID_DEV_REG2,
    CID_DEV_REG3,

};

// Example Data (Object) Dictionary for Modbus parameters
static const mb_parameter_descriptor_t characteristics[] = {
    // CID, Name, Units, Modbus addr, register type, Modbus Reg Start Addr, Modbus Reg read length, 
    // Instance offset (NA), Instance type, Instance length (bytes), Options (NA), Permissions
    { CID_DEV_REG0, STR("MB_hold_reg-0"), STR("Data"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 2, 1,
                    0, PARAM_TYPE_U16, 2, OPTS( 0,0,0 ), PAR_PERMS_READ_WRITE_TRIGGER },
    { CID_DEV_REG1, STR("MB_hold_reg-1"), STR("Data"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 3, 1,
                    0, PARAM_TYPE_U16, 2, OPTS( 0,0,0 ), PAR_PERMS_READ_WRITE_TRIGGER },
    { CID_DEV_REG2, STR("MB_hold_reg-2"), STR("Data"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 10, 1,
                    0, PARAM_TYPE_U16, 2, OPTS( 0,0,0 ), PAR_PERMS_READ_WRITE_TRIGGER },
    { CID_DEV_REG3, STR("MB_hold_reg-3"), STR("Data"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 12, 1,
                    0, PARAM_TYPE_U16, 2, OPTS( 0,0,0 ), PAR_PERMS_READ_WRITE_TRIGGER }
};

// Calculate number of parameters in the table
const uint16_t num_characteristics = (sizeof(characteristics)/sizeof(characteristics[0]));

static esp_err_t read_modbus_parameter(void* handle, uint16_t cid, uint16_t *par_data)
{
    const mb_parameter_descriptor_t* param_descriptor = NULL;
    
    esp_err_t err = mbc_master_get_cid_info(handle, cid, &param_descriptor);
    if ((err != ESP_ERR_NOT_FOUND) && (param_descriptor != NULL)) {
        uint8_t type = 0;
        err = mbc_master_get_parameter(handle, cid, (uint8_t*)par_data, &type);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Read mbm_%p, #%d %s (%s) value = (0x%x) parameter read successful.",
                                handle,
                                param_descriptor->cid,
                                (char*)param_descriptor->param_key,
                                (char*)param_descriptor->param_units,
                                *(uint16_t*)par_data);
        } else {
            ESP_LOGE(TAG, "Read mbm_%p, #%d %s (%s), parameter read fail.", 
                            handle,
                            param_descriptor->cid,
                            (char*)param_descriptor->param_key,
                            (char*)param_descriptor->param_units);
        }
    }
    return err;  
}

static esp_err_t write_modbus_parameter(void* handle, uint16_t cid, uint16_t *par_data)
{
    const mb_parameter_descriptor_t* param_descriptor = NULL;
    
    esp_err_t err = mbc_master_get_cid_info(handle, cid, &param_descriptor);
    if ((err != ESP_ERR_NOT_FOUND) && (param_descriptor != NULL)) {
        uint8_t type = 0; // type of parameter from dictionary
        err = mbc_master_set_parameter(handle, cid, (uint8_t*)par_data, &type);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Write mbm_%p, #%d %s (%s) value = (0x%x), write successful.",
                                        handle,
                                        param_descriptor->cid,
                                        (char*)param_descriptor->param_key,
                                        (char*)param_descriptor->param_units,
                                        *(uint16_t*)par_data);
        } else {
            ESP_LOGE(TAG, "Write mbm_%p, #%d (%s) write fail, err = 0x%x (%s).",
                                    handle,
                                    param_descriptor->cid,
                                    (char*)param_descriptor->param_key,
                                    (int)err,
                                    (char*)esp_err_to_name(err));
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
    0xAA55, 0x1122, 0x1122, 0x1122,
    0x4455, 0x1122, 0x1122, 0x1122,
    0x1122, 0x1122, 0x1122, 0x1122,
    0x5566, 0x1122, 0x1122, 0x1122
};

static uint16_t input_registers[] = {
    0xAA55, 0xAA55, 0xAA55, 0xAA55,
    0x4455, 0xAA55, 0xAA55, 0xAA55,
    0x1122, 0xAA55, 0xAA55, 0xAA55,
    0x5566, 0xAA55, 0xAA55, 0xAA55
};

static uint16_t coil_registers[] = {
    0xAA55,
    0x4455,
    0x1122,
    0x5566
};

// This is user function to read and write modbus holding registers
static void master_task(void *phandle)
{
    static mb_reg_access_t req_type = RT_HOLDING_RD;
    esp_err_t err = ESP_FAIL;

    ESP_LOGI(TAG, "Start modbus test...");
    ESP_LOGI(TAG, "Modbus test is completed.");
    TEST_ASSERT_TRUE(phandle);

    TEST_ESP_OK(mbc_master_start(phandle));
    holding_registers[0] = 0x1111;
    holding_registers[1] = 0x2222;
    write_modbus_parameter(phandle, CID_DEV_REG0, &holding_registers[0]);
    write_modbus_parameter(phandle, CID_DEV_REG1, &holding_registers[1]);

    while(1)
    {
        switch (req_type)
        {
            case RT_HOLDING_RD:
                err = read_modbus_parameter(phandle, CID_DEV_REG0, &holding_registers[0]);
                if ((holding_registers[0] != 0x1111)) {
                    ESP_LOGE(TAG, "%p, modbus REG0 = 0x%x, read fail", phandle, holding_registers[0]);
                }
                err |= read_modbus_parameter(phandle, CID_DEV_REG1, &holding_registers[1]);
                if ((holding_registers[1] != 0x2222)) {
                    ESP_LOGE(TAG, "%p, modbus REG1 = 0x%x, read fail", phandle, holding_registers[1]);
                }
                req_type = RT_HOLDING_WR;
                break;
            case RT_HOLDING_WR:
                holding_registers[0] = 0x1111;
                holding_registers[1] = 0x2222;
                err = write_modbus_parameter(phandle, CID_DEV_REG0, &holding_registers[0]);
                err |= write_modbus_parameter(phandle, CID_DEV_REG1, &holding_registers[1]);
                req_type = RT_HOLDING_RD;
                vTaskDelay(1);
                break;
            default:
                break;
        }
        vTaskDelay(1);
    }
    TEST_ESP_OK(mbc_master_stop(phandle));
    ESP_LOGI(TAG, "Modbus master stack destroy...");
    TEST_ESP_OK(mbc_master_delete(phandle));
    vTaskDelete(NULL);
}

// This is user function to read and write modbus holding registers
static void master_task1(void *phandle)
{
    static mb_reg_access_t req_type = RT_HOLDING_RD;
    esp_err_t err = ESP_FAIL;

    ESP_LOGI(TAG, "Start modbus test...");
    ESP_LOGI(TAG, "Modbus test is completed.");
    TEST_ASSERT_TRUE(phandle);

    TEST_ESP_OK(mbc_master_start(phandle));
    holding_registers[2] = 0x3333;
    holding_registers[3] = 0x4444;
    write_modbus_parameter(phandle, CID_DEV_REG2, &holding_registers[2]);
    write_modbus_parameter(phandle, CID_DEV_REG3, &holding_registers[3]);
    while(1)
    {
        switch (req_type)
        {
        case RT_HOLDING_RD:
            err = read_modbus_parameter(phandle, CID_DEV_REG2, &holding_registers[2]);
            if ((holding_registers[2] != 0x3333)) {
                ESP_LOGE(TAG, "%p, modbus REG2 = 0x%x, read fail", phandle, holding_registers[2]);
            }
            err |= read_modbus_parameter(phandle, CID_DEV_REG3, &holding_registers[3]);
            if ((holding_registers[3] != 0x4444)) {
                ESP_LOGE(TAG, "%p, modbus REG3 = 0x%x, read fail", phandle, holding_registers[3]);
            }
            req_type = RT_HOLDING_WR;
            break;
        case RT_HOLDING_WR:
            // holding_registers[2] = 0x3333;
            // holding_registers[3] = 0x4444;
            // err = write_modbus_parameter(phandle, CID_DEV_REG2, &holding_registers[2]);
            // err |= write_modbus_parameter(phandle, CID_DEV_REG3, &holding_registers[3]);
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
    vTaskDelete(NULL);
}

static void master_init(mb_communication_info_t *pcomm, void **pmbm_handle) 
{
    //static TaskHandle_t task_handle = NULL;

    void *handle = NULL;

    TEST_ESP_OK(mbc_master_create_serial(pcomm, &handle));
    // Set UART pin numbers
    // TEST_ESP_OK(uart_set_pin(pcomm->ser_opts.port, MB_UART_TXD_PIN, MB_UART_RXD_PIN,
    //                             MB_UART_RTS_PIN, UART_PIN_NO_CHANGE));
    // Set driver mode to Half Duplex
    // TEST_ESP_OK(uart_set_mode(pcomm->ser_opts.port, UART_MODE_RS485_HALF_DUPLEX));
    TEST_ESP_OK(mbc_master_set_descriptor(handle, &characteristics[0], num_characteristics));
    ESP_LOGI(TAG, "Modbus master stack is initialized...");

    *pmbm_handle = handle;
}

static TaskHandle_t task_handle1 = NULL;
static TaskHandle_t task_handle2 = NULL;
static TaskHandle_t task_handle3 = NULL;

#define SPIN_LOCK_INIT(lock) do { spinlock_initialize(&lock); } while(0)
#define SPIN_LOCK_ENTER(lock) do { vPortEnterCriticalSafe(&lock); } while(0)
#define SPIN_LOCK_EXIT(lock) do { vPortExitCriticalSafe(&lock); } while(0)

#define MB_ERR_COUNT 300000

static void busy_task(void *phandle)
{
    esp_err_t err = ESP_FAIL;
    portMUX_TYPE spin_lock;
    SPIN_LOCK_INIT(spin_lock);

    while(1) {
        SPIN_LOCK_ENTER(spin_lock);
        for (int i = 0; i < MB_ERR_COUNT; i++){
            ;
        }
        SPIN_LOCK_EXIT(spin_lock);
        vTaskDelay(1);
        // err = read_modbus_parameter(phandle, CID_DEV_REG2, &holding_registers[2]);
        // if (err == ESP_OK) {
        //     ESP_LOGI(TAG, "%p, Busy task read, error = %d", phandle, err);
        //     vTaskDelay(1);
        // } else {
        //     ESP_LOGE(TAG, "%p, Busy task read, error = %d", phandle, err);
        // }
    }
}

TEST_CASE("test setup modbus multi master", "[modbus]")
{
    // Initialization of device peripheral and objects
    // Initialize and start Modbus controller
    mb_communication_info_t comm = {
        .ser_opts.port = (MASTER_PORT_NUM ^ 3),
        .ser_opts.mode = MB_RTU,
        .ser_opts.data_bits = UART_DATA_8_BITS,
        .ser_opts.stop_bits = UART_STOP_BITS_2,
        .ser_opts.baudrate = MASTER_SPEED,
        .ser_opts.parity = UART_PARITY_DISABLE
    };

    void *mbm_handle1 = NULL;

    master_init(&comm, &mbm_handle1);

    // Set UART pin numbers
    TEST_ESP_OK(uart_set_pin(comm.ser_opts.port, MB_UART_TXD_PIN, MB_UART_RXD_PIN,
                            MB_UART_RTS_PIN, UART_PIN_NO_CHANGE));
    // Set driver mode to Half Duplex
    TEST_ESP_OK(uart_set_mode(comm.ser_opts.port, UART_MODE_RS485_HALF_DUPLEX));
    ESP_LOGI(TAG, "%p, Modbus master stack1 is started...", mbm_handle1);

    xTaskCreatePinnedToCore(master_task, "master_task",
                                                MB_SERIAL_TASK_STACK_SIZE,
                                                mbm_handle1, (MB_SERIAL_TASK_PRIO - 2),
                                                &task_handle1, MB_PORT_TASK_AFFINITY);

    mb_communication_info_t comm_par_1 = {
        .ser_opts.port = MASTER_PORT_NUM,
        .ser_opts.mode = MB_RTU,
        .ser_opts.data_bits = UART_DATA_8_BITS,
        .ser_opts.stop_bits = UART_STOP_BITS_2,
        .ser_opts.baudrate = MASTER_SPEED,
        .ser_opts.parity = UART_PARITY_DISABLE
    };

    void *mbm_handle2 = NULL;

    master_init(&comm_par_1, &mbm_handle2);
    // Set UART pin numbers
    TEST_ESP_OK(uart_set_pin(comm_par_1.ser_opts.port, 19, 
                                21, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    // Set driver mode to Half Duplex
    //TEST_ESP_OK(uart_set_mode(comm_par_1.ser_opts.port, UART_MODE_RS485_HALF_DUPLEX));

    ESP_LOGI(TAG, "%p, Modbus master stack2 is started...", mbm_handle2);

    xTaskCreatePinnedToCore(master_task1, "master_task1",
                                            MB_SERIAL_TASK_STACK_SIZE,
                                            mbm_handle2, (MB_SERIAL_TASK_PRIO),
                                            &task_handle2, MB_PORT_TASK_AFFINITY);

    xTaskCreatePinnedToCore(busy_task, "busy_task",
                                        MB_SERIAL_TASK_STACK_SIZE,
                                        mbm_handle2, (22),
                                        &task_handle3, MB_PORT_TASK_AFFINITY);
}
