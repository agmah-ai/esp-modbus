/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_timer.h"
#include "mb_common.h"
#include "port_common.h"
#include "mb_config.h"
#include "port_serial_common.h"

/* ----------------------- Defines ------------------------------------------*/
#define MB_SERIAL_RX_SEMA_TOUT_MS   (1000)
#define MB_SERIAL_RX_SEMA_TOUT      (pdMS_TO_TICKS(MB_SERIAL_RX_SEMA_TOUT_MS))
#define MB_SERIAL_RX_FLUSH_RETRY    (2)
#define MB_QUEUE_LENGTH             (20)
#define MB_SERIAL_TOUT              (3)
#define MB_SERIAL_TX_TOUT_TICKS     (pdMS_TO_TICKS(400))
#define MB_SERIAL_TASK_STACK_SIZE   (2048)     
#define MB_SERIAL_RX_TOUT_TICKS     (pdMS_TO_TICKS(100))

#define MB_SERIAL_MIN_PATTERN_INTERVAL  (9)
#define MB_SERIAL_MIN_POST_IDLE         (0)
#define MB_SERIAL_MIN_PRE_IDLE          (0)

typedef struct
{
    mb_port_base_t base;
    // serial communication properties
    mb_serial_opts_t ser_opts;
    bool rx_state_en;
    bool tx_state_en;
    uint16_t recv_length;
    uint64_t send_time_stamp;
    uint64_t recv_time_stamp;
    uint32_t flags;
    QueueHandle_t uart_queue;           // A queue to handle UART event.
    TaskHandle_t  task_handle;          // UART task to handle UART event.
    SemaphoreHandle_t rx_sema_handle;   // Rx blocking semaphore handle
} mb_ser_port_t;

/* ----------------------- Static variables & functions ----------------------*/
static const char *TAG = "mb_port.serial";

// Function returns time left for response processing according to response timeout
IRAM_ATTR static int64_t mb_port_ser_get_resp_time_left(mb_port_base_t *inst, int64_t time)
{
    int64_t time_left = 0;
    int64_t start_time = time;
    if (start_time <= 0) {
        start_time = esp_timer_get_time();
    }
    mb_ser_port_t *port_obj = __containerof(inst, mb_ser_port_t, base);
    int64_t time_stamp = start_time - port_obj->send_time_stamp;
    uint32_t response_time = mb_port_tmr_get_response_time_ms(inst);
    time_left = (time_stamp > (1000 * response_time)) ? 0 :
                                (response_time - (time_stamp / 1000) - 1);
    return time_left;
}

static bool mb_port_ser_rx_sema_init(mb_port_base_t *inst)
{
    mb_ser_port_t *port_obj = __containerof(inst, mb_ser_port_t, base);
    port_obj->rx_sema_handle = xSemaphoreCreateBinary();
    MB_RETURN_ON_FALSE((port_obj->rx_sema_handle), false , TAG, 
                        "%s: RX semaphore create failure.", __func__);
    return true;
}

static void mb_port_ser_rx_sema_close(mb_port_base_t *inst)
{
    mb_ser_port_t *port_obj = __containerof(inst, mb_ser_port_t, base);
    if (port_obj->rx_sema_handle) {
        vSemaphoreDelete(port_obj->rx_sema_handle);
        port_obj->rx_sema_handle = NULL;
    }
}

static bool mb_port_ser_rx_sema_take(mb_port_base_t *inst, uint32_t tm_ticks)
{
    BaseType_t status = pdTRUE;
    mb_ser_port_t *port_obj = __containerof(inst, mb_ser_port_t, base);
    status = xSemaphoreTake(port_obj->rx_sema_handle, tm_ticks );
    MB_RETURN_ON_FALSE((status == pdTRUE), false , TAG, 
                        "%p: rx semaphore take failure.", inst);
    ESP_LOGV(TAG,"%s: take RX semaphore (%" PRIu32" ticks).", __func__, tm_ticks);
    return true;
}

static void mb_port_ser_rx_sema_release(mb_port_base_t *inst)
{
    BaseType_t status = pdFALSE;
    mb_ser_port_t *port_obj = __containerof(inst, mb_ser_port_t, base);
    status = xSemaphoreGive(port_obj->rx_sema_handle);
    if (status != pdTRUE) {
        ESP_LOGD(TAG,"%p: rx semaphore is free.", inst);
    }
}

static bool mb_port_ser_rx_sema_is_busy(mb_port_base_t *inst)
{
    BaseType_t status = pdFALSE;
    mb_ser_port_t *port_obj = __containerof(inst, mb_ser_port_t, base);
    status = (uxSemaphoreGetCount(port_obj->rx_sema_handle) == 0) ? true : false;
    return status;
}

static void mb_port_ser_rx_flush(mb_port_base_t *inst)
{
    size_t size = 1;
    esp_err_t err = ESP_OK;
    mb_ser_port_t *port_obj = __containerof(inst, mb_ser_port_t, base);
    for (int cnt = 0; (cnt < MB_SERIAL_RX_FLUSH_RETRY) && size; cnt++) {
        err = uart_get_buffered_data_len(port_obj->ser_opts.port, &size);
        MB_RETURN_ON_FALSE((err == ESP_OK), ; , TAG, 
                                "%p, mb flush serial fail, error = 0x%x.", inst, (uint16_t)err);
        BaseType_t status = xQueueReset(port_obj->uart_queue);
        if (status) {
            err = uart_flush_input(port_obj->ser_opts.port);
            MB_RETURN_ON_FALSE((err == ESP_OK), ; , TAG, 
                                "%p, mb flush serial fail, error = 0x%x.", inst, (uint16_t)err);
        }
    }
}

void mb_port_ser_enable(mb_port_base_t *inst, bool rx_enable, bool tx_enable)
{
    mb_ser_port_t *port_obj = __containerof(inst, mb_ser_port_t, base);
    if (rx_enable) {
        port_obj->rx_state_en = true;
        mb_port_ser_rx_sema_release(inst);
        vTaskResume(port_obj->task_handle); // Resume receiver task
    } else {
        port_obj->rx_state_en = false;
        vTaskSuspend(port_obj->task_handle); // Block receiver task
    }
    CRITICAL_SECTION(inst->lock) {
        port_obj->tx_state_en = tx_enable;
    }
}

// UART receive event task
static void uart_queue_task(void *p_args)
{
    mb_ser_port_t *port_obj = __containerof(p_args, mb_ser_port_t, base);
    uart_event_t event;
    MB_RETURN_ON_FALSE(port_obj, ; , TAG, "%p, get serial instance fail.", port_obj);
    for(;;) {
        if (xQueueReceive(port_obj->uart_queue, (void *)&event, portMAX_DELAY)) {
            ESP_LOGD(TAG, "%p, UART[%d] event:", port_obj, port_obj->ser_opts.port);
            switch(event.type) {
                case UART_DATA:
                    ESP_LOGW(TAG,"%p, data event, len: %d.", port_obj, event.size);
                    // This flag set in the event means that no more
                    // data received during configured timeout and UART TOUT feature is triggered
                    if ((event.timeout_flag) /*&& (port_obj->mode == MB_RTU)*/) {
                        // Response is received but previous packet processing is pending
                        // Do not wait completion of processing and just discard received data as incorrect
                        if (mb_port_ser_rx_sema_is_busy(&port_obj->base)) {
                            mb_port_ser_rx_flush(&port_obj->base);
                            mb_port_ser_rx_sema_release(&port_obj->base);
                            break;
                        }
                        uart_get_buffered_data_len(port_obj->ser_opts.port, (unsigned int*)&event.size);
                        port_obj->recv_length = (event.size < MB_SERIAL_BUF_SIZE) ? event.size : MB_SERIAL_BUF_SIZE;
                        if (event.size <= MB_SER_PDU_SIZE_MIN) {
                            ESP_LOGW(TAG, "%p, drop short packet %d byte(s)", port_obj, event.size);
                            mb_port_ser_rx_flush(&port_obj->base);
                            mb_port_ser_rx_sema_release(&port_obj->base);
                            break;
                        }
                        // New frame is received, send an event to main FSM
                        mb_port_evt_post(&port_obj->base, EVENT(EV_FRAME_RECEIVED, port_obj->recv_length, NULL, 0));
                        ESP_LOGD(TAG, "%p, frame %d bytes is ready.", port_obj, port_obj->recv_length);
                    }
                    break;
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGD(TAG, "%p, hw fifo overflow.", port_obj);
                    xQueueReset(port_obj->uart_queue);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGD(TAG, "%p, ring buffer full.", port_obj);
                    xQueueReset(port_obj->uart_queue);
                    uart_flush_input(port_obj->ser_opts.port);
                    break;
                //Event of UART RX break detected
                case UART_BREAK:
                    ESP_LOGD(TAG, "%p, uart rx break.", port_obj);
                    break;
                //Event of UART parity check error
                case UART_PARITY_ERR:
                    ESP_LOGD(TAG, "%p, uart parity error.", port_obj);
                    xQueueReset(port_obj->uart_queue);
                    uart_flush_input(port_obj->ser_opts.port);
                    break;
                //Event of UART frame error
                case UART_FRAME_ERR:
                    ESP_LOGD(TAG, "%p, uart frame error.", port_obj);
                    xQueueReset(port_obj->uart_queue);
                    uart_flush_input(port_obj->ser_opts.port);
                    break;
                default:
                    ESP_LOGD(TAG, "%p, uart event type: %d.", port_obj, event.type);
                    break;
            }
        }
    }
    vTaskDelete(NULL);
}

mb_err_enum_t mb_port_ser_create(mb_serial_opts_t *ser_opts, mb_port_base_t **port_obj)
{
    mb_ser_port_t *pserial = NULL;
    esp_err_t err = ESP_OK;
    mb_err_enum_t ret = MB_EILLSTATE;
    pserial = (mb_ser_port_t*)calloc(1, sizeof(mb_ser_port_t));
    MB_RETURN_ON_FALSE((pserial && port_obj), MB_EILLSTATE, TAG, "mb serial port creation error.");
    CRITICAL_SECTION_INIT(pserial->base.lock);
    ser_opts->data_bits = ((ser_opts->data_bits > UART_DATA_5_BITS) 
                                && (ser_opts->data_bits < UART_DATA_BITS_MAX)) 
                                ? ser_opts->data_bits : UART_DATA_8_BITS;
    pserial->ser_opts = *ser_opts;
    // Configure serial communication parameters
    uart_config_t uart_cfg = {
        .baud_rate = ser_opts->baudrate,
        .data_bits = ser_opts->data_bits,
        .parity = ser_opts->parity,
        .stop_bits = ser_opts->stop_bits,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 2,
#if (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0))
        .source_clk = UART_SCLK_DEFAULT,
#else
        .source_clk = UART_SCLK_APB,
#endif
    };
    // Set UART config
    err = uart_param_config(pserial->ser_opts.port, &uart_cfg);
    MB_GOTO_ON_FALSE((err == ESP_OK), MB_EILLSTATE, error, TAG, 
                            "%p, mb config failure, uart_param_config() returned (0x%x).", pserial, (uint16_t)err);
    // Install UART driver, and get the queue.
    err = uart_driver_install(pserial->ser_opts.port, MB_SERIAL_BUF_SIZE, MB_SERIAL_BUF_SIZE,
                                    MB_QUEUE_LENGTH, &pserial->uart_queue, ESP_INTR_FLAG_IRAM);
    MB_GOTO_ON_FALSE((err == ESP_OK), MB_EILLSTATE, error, TAG,
                        "%p, mb serial driver failure, uart_driver_install() retuned (0x%x).", pserial, (uint16_t)err);
    err = uart_set_rx_timeout(pserial->ser_opts.port, MB_SERIAL_TOUT);
    MB_GOTO_ON_FALSE((err == ESP_OK), MB_EILLSTATE, error, TAG,
                        "mb serial set rx timeout failure, uart_set_rx_timeout() returned (0x%x).", (uint16_t)err);
    // Set always timeout flag to trigger timeout interrupt even after rx fifo full
    uart_set_always_rx_timeout(pserial->ser_opts.port, true);
    MB_GOTO_ON_FALSE((mb_port_ser_rx_sema_init(&pserial->base)), MB_EILLSTATE, error, TAG,
                                "mb serial RX semaphore create fail.");
    // Create a task to handle UART events
    BaseType_t status = xTaskCreatePinnedToCore(uart_queue_task, "uart_queue_task",
                                                    MB_SERIAL_TASK_STACK_SIZE,
                                                    &pserial->base, CONFIG_FMB_PORT_TASK_PRIO,
                                                    &pserial->task_handle, CONFIG_FMB_PORT_TASK_AFFINITY);
    // Force exit from function with failure
    MB_GOTO_ON_FALSE((status == pdPASS), MB_EILLSTATE, error, TAG,
                                "mb stack serial task creation error. xTaskCreate() returned (0x%x).",
                                (uint16_t)status);
    vTaskSuspend(pserial->task_handle); // Suspend serial task while stack is not started
    *port_obj = &(pserial->base);
    ESP_LOGD(TAG, "created object @%p", pserial);
    return MB_ENOERR;

error:
    if (pserial && pserial->task_handle) {
        vTaskDelete(pserial->task_handle);
    }
    if (pserial && pserial->ser_opts.port) {
        uart_driver_delete(pserial->ser_opts.port);
        CRITICAL_SECTION_CLOSE(pserial->base.lock);
        mb_port_ser_rx_sema_close(&pserial->base);
    }
    free(pserial);
    return MB_EILLSTATE;
}

void mb_port_ser_delete(mb_port_base_t *inst)
{
    mb_ser_port_t *port_obj = __containerof(inst, mb_ser_port_t, base);
    vTaskDelete(port_obj->task_handle);
    ESP_ERROR_CHECK(uart_driver_delete(port_obj->ser_opts.port));
    mb_port_ser_rx_sema_close(inst);
    CRITICAL_SECTION_CLOSE(inst->lock);
    free(port_obj);
}

bool mb_port_ser_recv_data(mb_port_base_t *inst, uint8_t **pp_ser_frame, uint16_t *p_ser_length)
{
    MB_RETURN_ON_FALSE((pp_ser_frame && p_ser_length), false, TAG, "mb serial get buffer failure.");
    mb_ser_port_t *port_obj = __containerof(inst, mb_ser_port_t, base);
    uint16_t counter = *p_ser_length ? *p_ser_length : port_obj->recv_length;
    bool status = false;

    status = mb_port_ser_rx_sema_take(inst, pdMS_TO_TICKS(mb_port_ser_get_resp_time_left(inst, 0)));
    if (status && counter && *pp_ser_frame) {
        // Read frame data from the ringbuffer of receiver
        counter = uart_read_bytes(port_obj->ser_opts.port, (uint8_t *)*pp_ser_frame,
                                    counter, pdMS_TO_TICKS(mb_port_ser_get_resp_time_left(inst, 0)));
        // The buffer is transferred into Modbus stack and is not needed here any more
        uart_flush_input(port_obj->ser_opts.port);
        ESP_LOGW(TAG, "%p, received data: %d bytes.", port_obj, counter);
        // Stop timer because the new data is received
        mb_port_tmr_disable(inst);
        //mb_port_tmr_convert_delay_enable(inst);
        // Store the timestamp of received frame
        port_obj->recv_time_stamp = esp_timer_get_time();
        ESP_LOG_BUFFER_HEX_LEVEL("RECEIVE", (void *)*pp_ser_frame, counter, ESP_LOG_WARN);
        int64_t time_delta = port_obj->recv_time_stamp - port_obj->send_time_stamp;
        ESP_LOGW(TAG, "%p, serial processing time[us] = %" PRId64, port_obj, time_delta);
        status = true;
        *p_ser_length = counter;
    } else {
        ESP_LOGE(TAG, "%s: junk data (%d bytes) received. ", __func__, (uint16_t)counter);
        mb_port_ser_rx_sema_release(inst);
    }
    *p_ser_length = counter;
    return status;
}

bool mb_port_ser_send_data(mb_port_base_t *inst, uint8_t *p_ser_frame, uint16_t ser_length)
{
    bool res = false;
    int count = 0;
    mb_ser_port_t *port_obj = __containerof(inst, mb_ser_port_t, base);

    mb_port_ser_enable(inst, false, true);

    if (p_ser_frame && ser_length && port_obj->tx_state_en) {
        // Flush buffer received from previous transaction
        mb_port_ser_rx_flush(inst);
        count = uart_write_bytes(port_obj->ser_opts.port, p_ser_frame, ser_length);
        ESP_LOGD(TAG, "MB_TX_buffer sent: (%d) bytes.", (uint16_t)(count - 1));
        // Waits while UART sending the packet
        esp_err_t status = uart_wait_tx_done(port_obj->ser_opts.port, MB_SERIAL_TX_TOUT_TICKS);
        mb_port_ser_enable(inst, true, false);
        (void)mb_port_evt_post(inst, EVENT(EV_FRAME_SENT));
        MB_RETURN_ON_FALSE((status == ESP_OK), false, TAG, "mb serial sent buffer failure.");
        // Print sent packet, the tag used is more clear to see
        ESP_LOG_BUFFER_HEX_LEVEL("SENT", (void *)p_ser_frame, ser_length, ESP_LOG_WARN);
        port_obj->send_time_stamp = esp_timer_get_time();
        res = true;
    } else {
        ESP_LOGE(TAG, "Send callback %d, %p, %d. ", port_obj->tx_state_en, p_ser_frame, ser_length);
    }
    return res;
}

bool mb_port_ser_put_byte(mb_port_base_t *inst, uint8_t byte_val)
{
    mb_ser_port_t *port_obj = __containerof(inst, mb_ser_port_t, base);
    uint8_t len = uart_write_bytes(port_obj->ser_opts.port, &byte_val, 1);
    return (len == 1);
}

bool mb_port_ser_get_byte(mb_port_base_t *inst, uint8_t *byte_buf)
{
    assert(byte_buf);
    mb_ser_port_t *port_obj = __containerof(inst, mb_ser_port_t, base);
    uint16_t len = uart_read_bytes(port_obj->ser_opts.port, (char *)byte_buf, 1, MB_SERIAL_RX_TOUT_TICKS);
    return (len == 1);
}
