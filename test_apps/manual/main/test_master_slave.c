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

#define MASTER_MAX_CIDS 2
#define MASTER_MAX_RETRY 10000
#define MASTER_PORT_NUM CONFIG_MB_UART_PORT_NUM
#define MASTER_SPEED CONFIG_MB_UART_BAUD_RATE
#define MB_UART_RXD_PIN CONFIG_MB_UART_RXD
#define MB_UART_TXD_PIN CONFIG_MB_UART_TXD
#define MB_UART_RTS_PIN CONFIG_MB_UART_RTS
#define MB_SERIAL_TASK_STACK_SIZE 4096
#define MB_SERIAL_TASK_PRIO 10

#define TAG "MODBUS_TEST"
#define STR(fieldname) ((const char*)( fieldname ))
#define OPTS(min_val, max_val, step_val) { .opt1 = min_val, .opt2 = max_val, .opt3 = step_val }

#define MB_REG_START_AREA0 (0x0000)
#define MB_READ_MASK (MB_EVENT_HOLDING_REG_RD | MB_EVENT_INPUT_REG_RD | MB_EVENT_DISCRETE_RD | MB_EVENT_COILS_RD)
#define MB_WRITE_MASK (MB_EVENT_HOLDING_REG_WR | MB_EVENT_COILS_WR)
#define MB_READ_WRITE_MASK (MB_WRITE_MASK | MB_READ_MASK)

#define MB_PAR_INFO_GET_TOUT (10)

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
static const mb_parameter_descriptor_t device_parameters[] = {
    // CID, Name, Units, Modbus addr, register type, Modbus Reg Start Addr, Modbus Reg read length, 
    // Instance offset (NA), Instance type, Instance length (bytes), Options (NA), Permissions
    { CID_DEV_REG0, STR("MB_hold_reg-0"), STR("Data"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 2, 1,
                    0, PARAM_TYPE_U16, 2, OPTS( 0,0,0 ), PAR_PERMS_READ_WRITE_TRIGGER },
    { CID_DEV_REG1, STR("MB_hold_reg-1"), STR("Data"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 3, 1,
                    0, PARAM_TYPE_U16, 2, OPTS( 0,0,0 ), PAR_PERMS_READ_WRITE_TRIGGER }
};

// Calculate number of parameters in the table
const uint16_t num_device_parameters = (sizeof(device_parameters)/sizeof(device_parameters[0]));

uint16_t holding_registers[] = {
    0xAA55, 0x1122, 0x1122, 0x1122,
    0x4455, 0x1122, 0x1122, 0x1122,
    0x1122, 0x1122, 0x1122, 0x1122,
    0x5566, 0x1122, 0x1122, 0x1122
};

uint16_t input_registers[] = {
    0xAA55, 0xAA55, 0xAA55, 0xAA55,
    0x4455, 0xAA55, 0xAA55, 0xAA55,
    0x1122, 0xAA55, 0xAA55, 0xAA55,
    0x5566, 0xAA55, 0xAA55, 0xAA55
};

uint16_t coil_registers[] = {
    0xAA55,
    0x4455,
    0x1122,
    0x5566
};

const uint16_t holding_registers_counter = (sizeof(holding_registers)/sizeof(holding_registers[0]));
const uint16_t input_registers_counter = (sizeof(input_registers)/sizeof(input_registers[0]));
const uint16_t coil_registers_counter = (sizeof(coil_registers)/sizeof(coil_registers[0]));

static portMUX_TYPE param_lock = portMUX_INITIALIZER_UNLOCKED;

void* mbm_handle = NULL;

static esp_err_t read_modbus_parameter(void* handle, uint16_t cid, uint16_t *par_data)
{
    const mb_parameter_descriptor_t* param_descriptor = NULL;
    
    esp_err_t err = mbc_master_get_cid_info(handle, cid, &param_descriptor);
    if ((err != ESP_ERR_NOT_FOUND) && (param_descriptor != NULL)) {
        uint8_t type = 0;
        err = mbc_master_get_parameter(handle, cid, (uint8_t*)par_data, &type);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Characteristic #%d %s (%s) value = (0x%04x) parameter read successful.",
                                     param_descriptor->cid,
                                     (char*)param_descriptor->param_key,
                                     (char*)param_descriptor->param_units,
                                     *(uint16_t*)par_data);
        } else {
            ESP_LOGE(TAG, "Characteristic #%d %s (%s), parameter read fail.", 
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
            ESP_LOGI(TAG, "Characteristic #%d %s (%s) value = (0x%04x), write successful.",
                                        param_descriptor->cid,
                                        (char*)param_descriptor->param_key,
                                        (char*)param_descriptor->param_units,
                                        *(uint16_t*)par_data);
        } else {
            ESP_LOGE(TAG, "Characteristic #%d (%s) write fail, err = 0x%x (%s).",
                                    param_descriptor->cid,
                                    (char*)param_descriptor->param_key,
                                    (int)err,
                                    (char*)esp_err_to_name(err));
        }
    }
    return err;  
}

static TaskHandle_t slave_task_handle = NULL;
static TaskHandle_t slave_task_handle1 = NULL;
static TaskHandle_t master_task_handle = NULL;

typedef enum {
    RT_HOLDING_RD,
    RT_HOLDING_WR
} mb_access_t;

// This is user function to read and write modbus holding registers
static void master_task(void *arg)
{
    esp_err_t err = ESP_OK;
    int i = 0;
    static mb_access_t req_type = RT_HOLDING_RD;  

    // Initialization of device peripheral and objects
    // Initialize and start Modbus controller
    mb_communication_info_t comm = {
        .ser_opts.port = (MASTER_PORT_NUM ^ 3),
        .ser_opts.mode = MB_ASCII,
        .ser_opts.data_bits = UART_DATA_8_BITS,
        .ser_opts.stop_bits = UART_STOP_BITS_2,
        .ser_opts.baudrate = MASTER_SPEED,
        .ser_opts.parity = UART_PARITY_DISABLE
    };

    TEST_ESP_OK(mbc_master_create_serial(&comm, &mbm_handle));
    // Set UART pin numbers
    TEST_ESP_OK(uart_set_pin(comm.ser_opts.port, MB_UART_TXD_PIN, MB_UART_RXD_PIN,
                            MB_UART_RTS_PIN, UART_PIN_NO_CHANGE));
    // Set driver mode to Half Duplex
    TEST_ESP_OK(uart_set_mode(comm.ser_opts.port, UART_MODE_RS485_HALF_DUPLEX));
    TEST_ESP_OK(mbc_master_set_descriptor(mbm_handle, &device_parameters[0], num_device_parameters));
    ESP_LOGI(TAG, "Modbus master stack is initialized...");

    ESP_LOGI(TAG, "Start modbus test...");
    ESP_LOGI(TAG, "Modbus test is completed.");
    TEST_ESP_OK(mbc_master_start(mbm_handle));
    holding_registers[0] = 0x1122;
    holding_registers[1] = 0x4455;
    write_modbus_parameter(mbm_handle, CID_DEV_REG0, &holding_registers[0]);
    write_modbus_parameter(mbm_handle, CID_DEV_REG1, &holding_registers[1]);
    for (i = 0; i < 50; i++)
    {
        switch (req_type)
        {
        case RT_HOLDING_RD:
            err = read_modbus_parameter(mbm_handle, CID_DEV_REG0, &holding_registers[0]);
            err |= read_modbus_parameter(mbm_handle, CID_DEV_REG0, &holding_registers[1]);
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "Read multireg fail, error = %d.", err);
            }
            else
            {
                ESP_LOGW(TAG, "Read multireg OK.");
            }
            req_type = RT_HOLDING_WR;
            break;
        case RT_HOLDING_WR:

            err = write_modbus_parameter(mbm_handle, CID_DEV_REG0, &holding_registers[0]);
            err |= write_modbus_parameter(mbm_handle, CID_DEV_REG1, &holding_registers[1]);
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "Write multireg fail, error %d.", err);
            }
            else
            {
                ESP_LOGI(TAG, "Write multireg OK.");
            }
            req_type = RT_HOLDING_RD;
            break;
        default:
            break;
        }
        vTaskDelay(1);
    }
    TEST_ESP_OK(mbc_master_stop(mbm_handle));
    ESP_LOGI(TAG, "Modbus master stack destroy...");
    TEST_ESP_OK(mbc_master_delete(mbm_handle));
    vTaskDelete(NULL);
}

static void slave_task1(void *arg)
{
    mb_communication_info_t comm_par_1 = {
        .ser_opts.port = MASTER_PORT_NUM,
        .ser_opts.mode = MB_ASCII,
        .ser_opts.slave_addr = 1,
        .ser_opts.data_bits = UART_DATA_8_BITS,
        .ser_opts.stop_bits = UART_STOP_BITS_2,
        .ser_opts.baudrate = MASTER_SPEED,
        .ser_opts.parity = UART_PARITY_DISABLE
    };

    void* mbs_handle1 = NULL;

    TEST_ESP_OK(mbc_slave_create_serial(&comm_par_1, &mbs_handle1));
    // Set UART pin numbers
    // TEST_ESP_OK(uart_set_pin(comm_par_1.ser_opts.port, 19,
    //                             21, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    TEST_ESP_OK(uart_set_pin(comm_par_1.ser_opts.port, MB_UART_TXD_PIN, MB_UART_RXD_PIN,
                        MB_UART_RTS_PIN, UART_PIN_NO_CHANGE));
    // Set driver mode to Half Duplex
    TEST_ESP_OK(uart_set_mode(comm_par_1.ser_opts.port, UART_MODE_RS485_HALF_DUPLEX));
    mb_register_area_descriptor_t reg_area;
    mb_param_info_t reg_info; // keeps the Modbus registers access information
    reg_area.type = MB_PARAM_HOLDING;
    reg_area.start_offset = MB_REG_START_AREA0;
    reg_area.address = (void*)&holding_registers[0];
    reg_area.size = holding_registers_counter; 
    TEST_ESP_OK(mbc_slave_set_descriptor(mbs_handle1, reg_area));

    reg_area.type = MB_PARAM_INPUT;
    reg_area.start_offset = MB_REG_START_AREA0;
    reg_area.address = (void*)&input_registers[0];
    reg_area.size = input_registers_counter;
    TEST_ESP_OK(mbc_slave_set_descriptor(mbs_handle1, reg_area));

    reg_area.type = MB_PARAM_COIL;
    reg_area.start_offset = MB_REG_START_AREA0;
    reg_area.address = (void*)&coil_registers[0];
    reg_area.size = coil_registers_counter;
    TEST_ESP_OK(mbc_slave_set_descriptor(mbs_handle1, reg_area));
    TEST_ESP_OK(mbc_slave_start(mbs_handle1));

    while (1) {
        mb_event_group_t event = mbc_slave_check_event(mbs_handle1, MB_READ_WRITE_MASK);
        const char* rw_str = (event & MB_READ_MASK) ? "READ" : "WRITE";

        // Filter events and process them accordingly
        if(event & (MB_READ_WRITE_MASK)) {
            // Get parameter information from parameter queue
            TEST_ESP_OK(mbc_slave_get_param_info(mbs_handle1, &reg_info, MB_PAR_INFO_GET_TOUT));
            ESP_LOGI(TAG, "REG SLAVE1 %s (%" PRIu32 " us), ADDR:%u, TYPE:%u, INST_ADDR:0x%" PRIx32 ", SIZE:%u",
                    rw_str,
                    (uint32_t)reg_info.time_stamp,
                    reg_info.mb_offset,
                    reg_info.type,
                    (uint32_t)reg_info.address,
                    reg_info.size);
            if (reg_info.address == (uint8_t*)&holding_registers[2]) {
                portENTER_CRITICAL(&param_lock);
                holding_registers[2] += 1;
                portEXIT_CRITICAL(&param_lock);
            }
        }
        vTaskDelay(1);
    }
    TEST_ESP_OK(mbc_slave_delete(mbs_handle1));
    vTaskDelete(NULL);
}

static void slave_task2(void *arg)
{
    mb_communication_info_t comm_par_1 = {
        .ser_opts.port = (MASTER_PORT_NUM ^ 3),
        .ser_opts.mode = MB_ASCII,
        .ser_opts.slave_addr = 1,
        .ser_opts.data_bits = UART_DATA_8_BITS,
        .ser_opts.stop_bits = UART_STOP_BITS_2,
        .ser_opts.baudrate = MASTER_SPEED,
        .ser_opts.parity = UART_PARITY_DISABLE
    };

    void* mbs_handle2 = NULL;

    TEST_ESP_OK(mbc_slave_create_serial(&comm_par_1, &mbs_handle2));
    // Set UART pin numbers
    TEST_ESP_OK(uart_set_pin(comm_par_1.ser_opts.port, 19,
                                21, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    // Set driver mode to Half Duplex
    //TEST_ESP_OK(uart_set_mode(comm_par_1.ser_opts.port, UART_MODE_RS485_HALF_DUPLEX));
    mb_register_area_descriptor_t reg_area;
    mb_param_info_t reg_info; // keeps the Modbus registers access information
    reg_area.type = MB_PARAM_HOLDING;
    reg_area.start_offset = MB_REG_START_AREA0;
    reg_area.address = (void*)&holding_registers[0];
    reg_area.size = holding_registers_counter; 
    TEST_ESP_OK(mbc_slave_set_descriptor(mbs_handle2, reg_area));

    reg_area.type = MB_PARAM_INPUT;
    reg_area.start_offset = MB_REG_START_AREA0;
    reg_area.address = (void*)&input_registers[0];
    reg_area.size = input_registers_counter;
    TEST_ESP_OK(mbc_slave_set_descriptor(mbs_handle2, reg_area));

    reg_area.type = MB_PARAM_COIL;
    reg_area.start_offset = MB_REG_START_AREA0;
    reg_area.address = (void*)&coil_registers[0];
    reg_area.size = coil_registers_counter;
    TEST_ESP_OK(mbc_slave_set_descriptor(mbs_handle2, reg_area));
    TEST_ESP_OK(mbc_slave_start(mbs_handle2));

    while (1) {
        mb_event_group_t event = mbc_slave_check_event(mbs_handle2, MB_READ_WRITE_MASK);
        const char* rw_str = (event & MB_READ_MASK) ? "READ" : "WRITE";

        // Filter events and process them accordingly
        if(event & (MB_READ_WRITE_MASK)) {
            // Get parameter information from parameter queue
            TEST_ESP_OK(mbc_slave_get_param_info(mbs_handle2, &reg_info, MB_PAR_INFO_GET_TOUT));
            ESP_LOGI(TAG, "REG SLAVE2 %s (%" PRIu32 " us), ADDR:%u, TYPE:%u, INST_ADDR:0x%" PRIx32 ", SIZE:%u",
                     rw_str,
                     (uint32_t)reg_info.time_stamp,
                     reg_info.mb_offset,
                     reg_info.type,
                     (uint32_t)reg_info.address,
                     reg_info.size);
            if (reg_info.address == (uint8_t*)&holding_registers[2]) {
                portENTER_CRITICAL(&param_lock);
                holding_registers[2] += 1;
                portEXIT_CRITICAL(&param_lock);
            }
        }
        vTaskDelay(1);
    }
    TEST_ESP_OK(mbc_slave_delete(mbs_handle2));
    vTaskDelete(NULL);
}

TEST_CASE("test modbus multi slave", "[modbus]")
{
    BaseType_t status;

    status = xTaskCreatePinnedToCore(slave_task1, "slave_task",
                                                MB_SERIAL_TASK_STACK_SIZE,
                                                NULL, (MB_SERIAL_TASK_PRIO - 2),
                                                &slave_task_handle, MB_PORT_TASK_AFFINITY);
    status = xTaskCreatePinnedToCore(slave_task2, "slave_task2",
                                            MB_SERIAL_TASK_STACK_SIZE,
                                            NULL, (MB_SERIAL_TASK_PRIO - 2),
                                            &slave_task_handle1, MB_PORT_TASK_AFFINITY);
    // status = xTaskCreatePinnedToCore(master_task, "master_task",
    //                                             MB_SERIAL_TASK_STACK_SIZE,
    //                                             NULL, (MB_SERIAL_TASK_PRIO - 1),
    //                                             &master_task_handle, MB_PORT_TASK_AFFINITY);
    (void)status;
}
