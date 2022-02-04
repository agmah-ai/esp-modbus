/*
 * SPDX-FileCopyrightText: 2018-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
 
#pragma once

#include <stdio.h>
#include <stdio.h>
#include <string.h>
#include <esp_compiler.h>               // for unlikely macro
#include <sys/unistd.h>
#include <sys/stat.h>
#include <time.h>
#include <limits.h>
#include <sys/time.h>
#include <sys/queue.h>                  // for list
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"              // for task handling
#include "freertos/stream_buffer.h"     // for stream buffer
#include "freertos/queue.h"             // for queue

#include "esp_err.h"
#include "stdint.h"
#include "esp_timer.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "pcap.h"

#define UT_TASK_AFFINITY            0
#define UT_TASK_PRIORITY            7
#define UT_TASK_STACK_SIZE          4096
#define UT_STREAM_BUF_SIZE          255
#define UT_TASK_EVENT_TOUT_MS       300

#if __has_include("esp_idf_version.h")
#include "esp_idf_version.h"
#endif

#define LOG_COLOR_H LOG_COLOR(LOG_COLOR_BLUE)

#define UT_LOG_FORMAT(letter, format)  DRAM_STR(LOG_COLOR_ ## letter #letter "(%u):%s " format LOG_RESET_COLOR "\n")

#if CONFIG_MB_UTEST_DEBUG

#define UT_LOG(tag, format, log_level, log_tag_letter, ...) do { \
            ets_printf(UT_LOG_FORMAT(log_tag_letter, format), esp_log_early_timestamp(), tag, ##__VA_ARGS__); \
} while(0)
#else
#define UT_LOG(tag, format, log_level, log_tag_letter, ...)
#endif

#define UT_LOGE( tag, format, ... ) UT_LOG(tag, format, ESP_LOG_ERROR, E, ##__VA_ARGS__)
#define UT_LOGW( tag, format, ... ) UT_LOG(tag, format, ESP_LOG_WARN, W, ##__VA_ARGS__)
#define UT_LOGI( tag, format, ... ) UT_LOG(tag, format, ESP_LOG_INFO, H, ##__VA_ARGS__)
#define UT_LOGD( tag, format, ... ) UT_LOG(tag, format, ESP_LOG_DEBUG, D, ##__VA_ARGS__)
#define UT_LOGV( tag, format, ... ) UT_LOG(tag, format, ESP_LOG_VERBOSE, V, ##__VA_ARGS__)
#define UT_LOGH( tag, format, ... ) UT_LOG(tag, format, ESP_LOG_INFO, H, ##__VA_ARGS__)

#if __has_include("esp_check.h")

#include "esp_check.h"

#define UT_RETURN_ON_FALSE ESP_RETURN_ON_FALSE
#define UT_GOTO_ON_ERROR ESP_GOTO_ON_ERROR
#define UT_GOTO_ON_FALSE ESP_GOTO_ON_FALSE
#define UT_RETURN_ON_ERROR ESP_RETURN_ON_ERROR

#else

// can not include esp_check then override check macro

#define UT_RETURN_ON_FALSE(a, err_code, log_tag, format, ...) do {                                         \
        if (unlikely(!(a))) {                                                                              \
            UT_LOGE(log_tag, "%s(%d): " format, __FUNCTION__, __LINE__ __VA_OPT__(,) __VA_ARGS__);        \
            return err_code;                                                                               \
        }                                                                                                  \
} while(0)

#define UT_GOTO_ON_ERROR(x, goto_tag, log_tag, format, ...) do {                                \
        esp_err_t err_rc_ = (x);                                                                \
        if (unlikely(err_rc_ != ESP_OK)) {                                                      \
            ret = err_rc_;                                                                      \
            goto goto_tag;                                                                      \
        }                                                                                       \
} while(0)

#define UT_GOTO_ON_FALSE(a, err_code, goto_tag, log_tag, format, ...) do {                     \
        if (unlikely(!(a))) {                                                                   \
            ret = err_code;                                                                     \
            goto goto_tag;                                                                      \
        }                                                                                       \
} while (0)

#define UT_RETURN_ON_ERROR(x, log_tag, format, ...) do {                                                  \
        esp_err_t err_rc_ = (x);                                                                           \
        if (unlikely(err_rc_ != ESP_OK)) {                                                                 \
            UT_LOGE(log_tag, "%s(%d): " format, __FUNCTION__, __LINE__ __VA_OPT__(,) __VA_ARGS__);        \
            return err_rc_;                                                                                \
        }                                                                                                  \
} while(0)

#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    DIRECTION_NOT_SET,
    DIRECTION_INPUT,
    DIRECTION_OUTPUT
} direction_t;

typedef struct pack_data_entry_s {
    int file_pos;
    uint32_t packet_index;
    uint32_t seconds;
    uint32_t microseconds;
    uint32_t capture_length;
    uint32_t packet_length;
    uint16_t crc;
    void* pbuffer;
    LIST_ENTRY(pack_data_entry_s) entries;
} pack_data_entry_t;

typedef struct {
    int stream_id;
    bool is_opened;
    bool is_writing;
    bool link_type_set;
    char* filename;
    char* stream_name;
    direction_t direction;
    uint32_t packet_count;
    pcap_file_handle_t pcap_handle;
    pcap_link_type_t link_type;
    StreamBufferHandle_t stream_buffer_handle;
    QueueHandle_t queue_handle;
    int curr_index;
    pack_data_entry_t* pcur_item;
    esp_timer_handle_t timer_handle;
    LIST_HEAD(pack_entries_, pack_data_entry_s) pack_entries;
} stream_t;

typedef struct ut_lister_s {
    TaskHandle_t  ut_task_handle;
    //QueueHandle_t ut_queue_handle;
    //esp_timer_handle_t ut_timer_handler;
    stream_t stream_input;
    stream_t stream_output;
    uint32_t packet_index;
    QueueHandle_t notif_queue_handle;
} ut_lister_t;

esp_err_t ut_init(const char* port_prefix);
void ut_close(void);
esp_err_t ut_get_notification(direction_t direction, struct timeval time);
int ut_get_stream_data(direction_t direction, void* pdata, size_t data_length, uint32_t timeout_ms);

esp_err_t ut_stream_capture_packet(direction_t direction, void *payload, uint32_t length, uint16_t crc);
esp_err_t ut_stream_override_packet(void *payload, uint32_t length);
esp_err_t ut_stream_insert_packet(direction_t direction, int index, void *payload, uint32_t length);
void ut_print_list(direction_t direction);
esp_err_t ut_stream_set_packet_data(direction_t direction, int index, void *payload, uint32_t length);

//#undef uart_hal_read_rxfifo

//uart_hal_read_rxfifo_subst(uart_hal_context_t *hal, uint8_t *buf, int *inout_rd_len);

//#define uart_hal_read_rxfifo uart_hal_read_rxfifo_subst

//#define uart_hal_get_rx_tout_thr(hal) uart_ll_get_rx_tout_thr((hal)->dev)

#ifdef __cplusplus
}
#endif
