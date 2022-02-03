/*
 * SPDX-FileCopyrightText: 2018-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_spiffs.h"
#include "esp_timer.h"
#include "mbport_stubs.h" // for read callbacks
#include "ut_io.h"
#include "pcap.h"
#include "sdkconfig.h"

static const char *TAG = "UT_TAG";

#define UT_TASK_STACK_SIZE  4096
#define UT_TASK_PRIO        6
#define UT_TASK_AFFINITY    0
#define UT_TIME_DIFF_MAX_US 2000000
#define UT_TIME_MAX_US      (INT_MAX)
#define UT_MAX_FILES        5
#define UT_FS_BASE_PATH     ("/spiffs")

static ut_lister_t ut_lister = {
    .stream_input = { .is_opened = false,
                        .is_writing = false,
                        .link_type_set = false,
                        .filename = NULL,
                        .stream_name = "input",
                        .direction = DIRECTION_INPUT,
                        .pcap_handle = NULL,
                        .stream_buffer_handle = NULL,
                        .pcur_item = NULL,
                        .curr_index = 0,
                        .link_type = PCAP_LINK_TYPE_LOOPBACK
                     },
    .stream_output = { .is_opened = false,
                        .is_writing = false,
                        .link_type_set = false,
                        .filename = NULL,
                        .stream_name = "output",
                        .direction = DIRECTION_OUTPUT,
                        .pcap_handle = NULL,
                        .stream_buffer_handle = NULL,
                        .pcur_item = NULL,
                        .curr_index = 0,
                        .link_type = PCAP_LINK_TYPE_LOOPBACK
                      },
    .ut_task_handle = NULL,
    .packet_index = 0
};

// Jist for test now Todo: remove
/*
#include <sys/lock.h>
#include "soc/rtc.h"
#include "soc/rtc_cntl_reg.h"

static _lock_t s_boot_time_lock;

#define RTC_BOOT_TIME_LOW_REG   RTC_CNTL_STORE2_REG
#define RTC_BOOT_TIME_HIGH_REG  RTC_CNTL_STORE3_REG

void esp_time_impl_set_boot_time(uint64_t time_us)
{
    _lock_acquire(&s_boot_time_lock);
    REG_WRITE(RTC_BOOT_TIME_LOW_REG, (uint32_t) (time_us & 0xffffffff));
    REG_WRITE(RTC_BOOT_TIME_HIGH_REG, (uint32_t) (time_us >> 32));
    _lock_release(&s_boot_time_lock);
}
*/

static void IRAM_ATTR ut_timer_cb(void *param)
{
    //ESP_EARLY_LOGI("TIMER", "callback triggered: %llu", esp_timer_get_time());
    stream_t* pstream = (stream_t*)param;
    BaseType_t xStatus = xQueueSend(pstream->queue_handle,
                                    (const void*)&pstream, 
                                    pdMS_TO_TICKS(UT_TASK_EVENT_TOUT_MS));
    if ((xStatus == pdTRUE) && pstream) {
        if (pstream && pstream->pcur_item) {
            ESP_LOGI("TIMER", "Send notification timeout %p, %s[%d].", pstream, pstream->stream_name, pstream->curr_index);
        }
    } else {
        ESP_LOGE("TIMER", "Timer timeout for stream: [%s].", pstream->stream_name);
        
    }
}

static esp_err_t ut_stream_open(stream_t *pcap);

static double time_diff(struct timeval x, struct timeval y)
{
  double x_ms, y_ms;

  x_ms = (double)x.tv_sec * 1000000 + (double)x.tv_usec;
  y_ms = (double)y.tv_sec * 1000000 + (double)y.tv_usec;
//
  return abs(y_ms - x_ms);
}

static uint64_t get_time_us(struct timeval time)
{
    int64_t time_us = (int64_t)time.tv_sec * 1000000L + (int64_t)time.tv_usec;
    return time_us;
}

static int64_t get_system_time_stamp(void)
{
    int64_t time_stamp = esp_timer_get_time();
    return time_stamp;
}

static void set_time(struct timeval time) 
{ 
    struct tm tm;
    tm.tm_year = 2017 - 1900;
    tm.tm_mon = 11;
    tm.tm_mday = 8;
    tm.tm_hour = 19;
    tm.tm_min = 51;
    tm.tm_sec = 10;
    time_t t = mktime(&tm);
    printf("Setting time: %s", asctime(&tm));
    struct timeval now = { .tv_sec = t };
    settimeofday(&now, NULL);
}

// Search the packet entry in the list from absolute index with the minimal time difference and return the entry and time difference
// returns list pointer which corresponds to the condition or NULL if it is not found in the list
pack_data_entry_t* ut_find_packet_index(int index, stream_t *pcap, struct timeval tv_start, uint64_t* diff_us)
{
    UT_RETURN_ON_FALSE((diff_us && pcap), NULL, TAG, "%s invalid arguments.", __func__);
    struct timeval tm_temp;
    uint64_t time_start = get_time_us(tv_start);
    uint64_t time_cur = 0;
    uint64_t time_prev = 0;
    uint64_t time_diff_us = 0;
    pack_data_entry_t* it = NULL;
    LIST_FOREACH(it, &pcap->pack_entries, entries) {
        tm_temp.tv_sec = it->seconds;
        tm_temp.tv_usec = it->microseconds;
        time_cur = get_time_us(tm_temp);
        if (index == (int)it->packet_index) {
            time_diff_us = (time_cur > time_start) ? (time_cur - time_start) : 0;
            if (diff_us) {
                *diff_us = time_diff_us;
            }
            //ESP_LOGI(pcap->stream_name, "cur_item %llu, start_t: %llu, diff: %llu", time_cur, time_start, time_diff_us);
            return it;
        }
        time_prev = time_cur;
    }
    *diff_us = 0;
    return NULL;
}

static esp_err_t ut_read_buffer(stream_t *pcap, pack_data_entry_t* pitem)
{
    UT_RETURN_ON_FALSE(pcap && pitem, ESP_ERR_INVALID_ARG, TAG, "invalid input pointers.");
    // If current item is inserted and not exist in the file just return the buffer to its data
    if (pitem->pbuffer && (pitem->file_pos == -1)) {
        return ESP_OK;
    }
    pitem->pbuffer = calloc(1, pitem->capture_length);
    if (pitem->pbuffer) {
        fseek(pcap->pcap_handle->file, pitem->file_pos, SEEK_SET);
        size_t real_read = fread(pitem->pbuffer, sizeof(uint8_t), pitem->capture_length, pcap->pcap_handle->file);
        if (real_read != pitem->capture_length) {
            ESP_LOGE(TAG, "read buffer failed.");
            free(pitem->pbuffer);
            return ESP_ERR_INVALID_STATE;
        }
        return ESP_OK;
    }
    return ESP_ERR_NO_MEM;
}

/*
static esp_err_t ut_mbm_get_answer(int index, void *payload, uint32_t length)
{
    struct timeval tv_temp = {0, 0};
    uint64_t time_diff_us = 0;
    //gettimeofday(&tv_temp, NULL);
    uint16_t crc = usMBCRC16((uint8_t*)payload, (length - 2));
    pack_data_entry_t* pitem = ut_find_packet_index(index, &ut_lister.stream_output, tv_temp, &time_diff_us);
    if (pitem) {
        if (crc != pitem->crc) {
            ESP_LOGE(TAG, "Packet #%d, CRC:%u!=%u", pitem->packet_index, crc, pitem->crc);
        }
        UT_RETURN_ON_FALSE((ut_read_buffer(&ut_lister.stream_output, pitem) == ESP_OK),
                                    ESP_ERR_INVALID_STATE, TAG, "can not read packet data.");
        ESP_LOG_BUFFER_HEX_LEVEL("OUT", (void*)pitem->pbuffer, pitem->capture_length, ESP_LOG_INFO);
        tv_temp.tv_sec = pitem->seconds;
        tv_temp.tv_usec = pitem->microseconds;
        pitem = ut_find_packet_index(index, &ut_lister.stream_input, tv_temp, &time_diff_us);
        UT_RETURN_ON_FALSE(pitem, ESP_ERR_INVALID_STATE, TAG, "the input packet not found in the log.");
        ESP_LOGW(TAG, "Found packet index #%d, time_diff_us: %llu, file_pos:%d", pitem->packet_index, time_diff_us, pitem->file_pos);
        UT_RETURN_ON_FALSE((ut_read_buffer(&ut_lister.stream_input, pitem) == ESP_OK),
                            ESP_ERR_INVALID_STATE, TAG, "can not read packet data.");
        ut_lister.stream_input.pcur_item = pitem;
        ESP_LOG_BUFFER_HEX_LEVEL("GET", (void*)pitem->pbuffer, pitem->capture_length, ESP_LOG_INFO);
        UT_RETURN_ON_FALSE(ut_lister.stream_input.timer_handle,
                            ESP_ERR_INVALID_STATE, TAG, "incorrect timer handler.");
        UT_RETURN_ON_FALSE((esp_timer_stop(ut_lister.stream_input.timer_handle) == 0),
                            ESP_ERR_INVALID_STATE, TAG, "can not read packet data.");
        return ESP_OK;
    }
    return ESP_ERR_INVALID_STATE;
}
*/

/*

esp_err_t ut_stream_override_packet(void *payload, uint32_t length)
{
    // ESP_LOGI(TAG, "Search packet #%d, %p, %u", ut_lister.packet_index, payload, length);
    // ESP_LOG_BUFFER_HEX_LEVEL("SEND", (void*)payload, (uint16_t)length, ESP_LOG_INFO);
    // esp_err_t err = ut_mbm_get_answer(ut_lister.packet_index, payload, length);
    // ut_lister.packet_index++;
    return ESP_FAIL;
}
*/

// Set packet data of packet as defined by parameters
esp_err_t ut_stream_set_packet_data(direction_t direction, int index, void *payload, uint32_t length)
{
    pack_data_entry_t* pcur_item = NULL;
    uint64_t time_diff_us = 0;
    struct timeval tv_temp = {0, 0};
    stream_t stream = (direction != DIRECTION_INPUT) ? (direction == DIRECTION_OUTPUT) ?
                                    ut_lister.stream_output : ut_lister.stream_input : ut_lister.stream_input;
    pcur_item = ut_find_packet_index(index, &stream, tv_temp, &time_diff_us);
    if (pcur_item) {
        void* pdata = pcur_item->pbuffer;
        pcur_item->pbuffer = realloc(pdata, (size_t)length);
        UT_RETURN_ON_FALSE(pcur_item->pbuffer, ESP_ERR_NO_MEM, TAG, "no mem for packet data.");
        memcpy(pcur_item->pbuffer, payload, length);
        pcur_item->packet_index = index;
        pcur_item->capture_length = length;
        pcur_item->packet_length = length;
        pcur_item->file_pos = -1;
        return ESP_OK;
    } else {

    }
    return ESP_ERR_NOT_FOUND;
}

// Insert fake packet into packet list
esp_err_t ut_stream_insert_packet(direction_t direction, int index, void *payload, uint32_t length)
{
    pack_data_entry_t* pcur_item = NULL;
    pack_data_entry_t* it = NULL;
    struct timeval tv_temp = {0, 0};
    uint64_t time_diff_us = 0;
    struct timeval tv_now;

    stream_t stream = (direction != DIRECTION_INPUT) ? (direction == DIRECTION_OUTPUT) ?
                                    ut_lister.stream_output : ut_lister.stream_input : ut_lister.stream_input;
    pcur_item = ut_find_packet_index(index, &stream, tv_temp, &time_diff_us);
    UT_RETURN_ON_FALSE(pcur_item, ESP_ERR_INVALID_STATE, TAG, "the index %d not found in the list.", index);
    pack_data_entry_t* pack_entry = calloc(1, sizeof(pack_data_entry_t));
    UT_RETURN_ON_FALSE(pack_entry, ESP_ERR_NO_MEM, TAG, "no mem for packet entry.");
    gettimeofday(&tv_now, NULL);
    pack_entry->packet_index = index;
    pack_entry->seconds = tv_now.tv_sec;
    pack_entry->microseconds = tv_now.tv_usec;
    pack_entry->capture_length = length;
    pack_entry->packet_length = length;
    pack_entry->file_pos = -1; // indicates that packet is not indexed in the file stream
    void* pdata = calloc(1, length);
    UT_RETURN_ON_FALSE(pdata, ESP_ERR_NO_MEM, TAG, "no mem for packet data.");
    memcpy(pdata, payload, length);
    pack_entry->pbuffer = pdata;
    // capture the crc of the payload, shall be used to check packet correctness
    pack_entry->crc = usMBCRC16((uint8_t*)pdata, (length - 2));
    LIST_INSERT_AFTER(pcur_item, pack_entry, entries);
    for(it = LIST_NEXT(pack_entry, entries); it != NULL; it = LIST_NEXT(it, entries)) {
        it->packet_index++;
        if (it->pbuffer) {
            ESP_LOG_BUFFER_HEX_LEVEL("pack", (void*)it->pbuffer, (uint16_t)it->capture_length, ESP_LOG_INFO);
        }
    }
    stream.packet_count++;
    return ESP_OK;
}

static esp_err_t ut_stream_open(stream_t *pcap)
{
    esp_err_t ret = ESP_OK;
    // Create file to read/write, binary format
    FILE *fp = fopen(pcap->filename, "rb+");
    UT_GOTO_ON_FALSE(fp, ESP_FAIL, err, TAG, "Open file %s failed.", pcap->filename);
    pcap_config_t pcap_config = {
        .fp = fp,
        .major_version = PCAP_DEFAULT_VERSION_MAJOR,
        .minor_version = PCAP_DEFAULT_VERSION_MINOR,
        .time_zone = PCAP_DEFAULT_TIME_ZONE_GMT,
    };
    UT_GOTO_ON_ERROR(pcap_new_session(&pcap_config, &pcap->pcap_handle), err, TAG, "Pcap init failed.");
    pcap->is_opened = true;
    ESP_LOGI(TAG, "Open file %s successfully.", pcap->filename);
    pcap_file_header_t fh;
    ret = pcap_read_header(pcap->pcap_handle, &fh);
    if (ret) {
          rewind(fp);
          ret = pcap_write_header(pcap->pcap_handle, PCAP_LINK_TYPE_LOOPBACK);
          UT_GOTO_ON_ERROR(ret, err, TAG, "Pcap write filed.");
    }
    return ESP_OK;
err:
    if (fp) {
        fclose(fp);
    }
    return ret;
}

static esp_err_t ut_stream_close(stream_t *pcap)
{
    esp_err_t ret = ESP_OK;
    UT_GOTO_ON_FALSE(pcap->is_opened, ESP_ERR_INVALID_STATE, err, TAG, ".pcap file is already closed");
    UT_GOTO_ON_ERROR(pcap_del_session(pcap->pcap_handle) != ESP_OK, err, TAG, "stop pcap session failed");
    pcap->is_opened = false;
    pcap->link_type_set = false;
    pcap->pcap_handle = NULL;
err:
    return ret;
}

void ut_print_list(direction_t direction)
{
    pack_data_entry_t* it;
    pack_data_entry_t* it_temp;

    stream_t stream = (direction != DIRECTION_INPUT) ? (direction == DIRECTION_OUTPUT) ?
                                        ut_lister.stream_output : ut_lister.stream_input : ut_lister.stream_input;
    LIST_FOREACH_SAFE(it, &stream.pack_entries, entries, it_temp) {
        ESP_LOGI("pack", "i:%d, p:%p, s:%d, pos:%d", it->packet_index, it->pbuffer, it->capture_length, it->file_pos);
        if (it->pbuffer) {
            ESP_LOG_BUFFER_HEX_LEVEL("pack", (void*)it->pbuffer, (uint16_t)it->capture_length, ESP_LOG_INFO);
        }
    }
}

static void ut_free_packet_list(stream_t* stream) {
    pack_data_entry_t* it; 
    pack_data_entry_t* it_temp;

    LIST_FOREACH_SAFE(it, &stream->pack_entries, entries, it_temp) {
        if (it->pbuffer) {
            free(it->pbuffer);
            it->pbuffer = NULL;
        }
        LIST_REMOVE(it, entries);
        free(it);
    }
    return;
}

static esp_err_t ut_read_packet_list(stream_t* stream)
{
    pcap_file_header_t file_header;
    esp_err_t ret = ESP_OK;
    uint32_t save_index = 0;
    uint32_t index = 0;
    long size = 0;
    save_index = ftell(stream->pcap_handle->file);
    fseek(stream->pcap_handle->file, 0L, SEEK_END);
    size = ftell(stream->pcap_handle->file);
    ESP_LOGD("DEBUG", "File %s, cur_index = %d, size = %lu", stream->filename, save_index, size);
    rewind(stream->pcap_handle->file);
    size_t real_read = fread(&file_header, sizeof(pcap_file_header_t), 1, stream->pcap_handle->file);
    UT_RETURN_ON_FALSE(real_read == 1, ESP_FAIL, TAG, "read pcap file header failed");
    index += sizeof(pcap_file_header_t);
    uint32_t packet_num = 0;
    pcap_packet_header_t packet_header;
    pack_data_entry_t* pcur_item = LIST_FIRST(&stream->pack_entries); // NULL if list is empty
    while (index < size) {
        real_read = fread(&packet_header, sizeof(pcap_packet_header_t), 1, stream->pcap_handle->file);
        UT_GOTO_ON_FALSE(real_read == 1, ESP_FAIL, err, TAG, "read pcap packet header failed");
        index += sizeof(pcap_packet_header_t);
        // create packet entry
        pack_data_entry_t* pack_entry = calloc(1, sizeof(pack_data_entry_t));
        UT_GOTO_ON_FALSE(pack_entry, ESP_ERR_NO_MEM, err, TAG, "no mem for packet entry");
        pack_entry->packet_index = packet_num;
        pack_entry->seconds = packet_header.seconds;
        pack_entry->microseconds = packet_header.microseconds;
        pack_entry->capture_length = packet_header.capture_length;
        pack_entry->packet_length = packet_header.packet_length;
        pack_entry->file_pos = index;
        void* pdata = calloc(1, packet_header.capture_length);
        real_read = fread(pdata, sizeof(uint8_t), packet_header.capture_length, stream->pcap_handle->file);
        UT_GOTO_ON_FALSE(real_read == packet_header.capture_length, ESP_FAIL, err, TAG, "pcap read payload failed");
        // capture the crc of the payload, shall be used to check packet correctness
        pack_entry->crc = usMBCRC16((uint8_t*)pdata, (packet_header.capture_length - 2));
        free(pdata);
        pack_entry->pbuffer = NULL;
        index += packet_header.capture_length;
        if (!pcur_item) {
            LIST_INSERT_HEAD(&stream->pack_entries, pack_entry, entries);
        } else {
            LIST_INSERT_AFTER(pcur_item, pack_entry, entries);
        }
        pcur_item = pack_entry;
        packet_num++;
    }
    stream->packet_count = packet_num;
err:
    fseek(stream->pcap_handle->file, save_index, SEEK_SET);
    ESP_LOGI("PACKET", "File: %s, Found: %d packets.", stream->filename, packet_num);

    return ret;
}

esp_err_t ut_set_timer(stream_t* pstream, uint64_t time_diff)
{
    esp_err_t err = ESP_OK;
    err = esp_timer_stop(pstream->timer_handle);
    err = esp_timer_start_once(pstream->timer_handle, time_diff);
    return err;
}

esp_err_t ut_handle_data(stream_t* pstream)
{
    uint64_t time_diff_us = 0;
    struct timeval tv_temp = {pstream->pcur_item->seconds, pstream->pcur_item->microseconds};
    pack_data_entry_t* pitem = ut_find_packet_index(pstream->curr_index, pstream, tv_temp, &time_diff_us);
    
    UT_RETURN_ON_FALSE(pitem, ESP_ERR_INVALID_STATE, TAG, "the input packet not found in the log.");
    ESP_LOGW(pstream->stream_name, "Packet index #%d, time_diff_us: %llu, file_pos:%d", pitem->packet_index, time_diff_us, pitem->file_pos);
    UT_RETURN_ON_FALSE((ut_read_buffer(pstream, pitem) == ESP_OK),
                        ESP_ERR_INVALID_STATE, TAG, "can not read packet data.");
    ESP_LOG_BUFFER_HEX_LEVEL(pstream->stream_name, (void*)pitem->pbuffer, pitem->capture_length, ESP_LOG_WARN);
    //ESP_EARLY_LOGI(__func__, "Send %s buf:%p, len:%d", pstream->stream_name, pstream->pcur_item->pbuffer, pstream->pcur_item->capture_length);
    xStreamBufferSend(pstream->stream_buffer_handle, 
                        (void*)pitem->pbuffer, 
                        (size_t)pitem->capture_length, pdMS_TO_TICKS(UT_TASK_EVENT_TOUT_MS));
    //ESP_LOG_BUFFER_HEXDUMP("TIMER", pstream->pcur_item->pbuffer, pstream->pcur_item->capture_length, ESP_LOG_WARN);
    pstream->pcur_item = pitem;
    pstream->curr_index++;
    UT_RETURN_ON_FALSE(pstream->timer_handle, ESP_ERR_INVALID_STATE, TAG, "Timer handler is not correct." );
    esp_timer_stop(pstream->timer_handle);
    esp_timer_start_once(pstream->timer_handle, time_diff_us);
    return ESP_OK;
}

// Notify the event task about new read/write event
esp_err_t ut_get_notification(direction_t direction, struct timeval tv)
{
    const direction_t dir = direction;
    esp_err_t err = ESP_FAIL;
    stream_t* pstream = (direction != DIRECTION_INPUT) ? (direction == DIRECTION_OUTPUT) ?
                                        &ut_lister.stream_output : &ut_lister.stream_input : &ut_lister.stream_input;
    stream_t* pnotif_stream = NULL;
    BaseType_t status = xQueueReceive(pstream->queue_handle,
                                                (void*)&pnotif_stream, 
                                                pdMS_TO_TICKS(get_time_us(tv) / 1000)); //portMAX_DELAY
    err = ut_handle_data(pstream);
    if (status != pdTRUE) {
        err = ESP_ERR_TIMEOUT;
    }
    return err;
}

int ut_get_stream_data(direction_t direction, void* pdata, size_t data_length, uint32_t timeout_ms) 
{
    UT_RETURN_ON_FALSE(pdata, ESP_ERR_INVALID_STATE, TAG,
                        "Invalid buffer pointer to get data.");
    stream_t stream = (direction != DIRECTION_INPUT) ? (direction == DIRECTION_OUTPUT) ?
                                        ut_lister.stream_output : ut_lister.stream_input : ut_lister.stream_input;

    size_t length = xStreamBufferReceive(stream.stream_buffer_handle,
                                            (void*)pdata, data_length, pdMS_TO_TICKS(timeout_ms)); //portMAX_DELAY
    if (length > 0) {
        ESP_LOG_BUFFER_HEXDUMP("UT_STREAM", pdata, length, ESP_LOG_WARN);
    }
    return length;
}

static void ut_lister_task(void *parg)
{
    struct timeval tv_temp = {0, 0};
    uint64_t time_diff_inp_us = 0;
    uint64_t time_diff_out_us = 0;
    
    pack_data_entry_t* pinp_pitem = ut_find_packet_index(0, &ut_lister.stream_input, tv_temp, &time_diff_inp_us);
    pack_data_entry_t* pout_pitem = ut_find_packet_index(0, &ut_lister.stream_output, tv_temp, &time_diff_out_us);
    if (time_diff_inp_us <= time_diff_out_us) {
        ut_set_timer(&ut_lister.stream_input, 0);
        ut_set_timer(&ut_lister.stream_output, (time_diff_out_us - time_diff_inp_us));
    } else {
        ut_set_timer(&ut_lister.stream_output, 0);
        ut_set_timer(&ut_lister.stream_input, (time_diff_inp_us - time_diff_out_us));
    }
    ESP_LOGW("INIT", "Initialization of streams is done.");
    ut_lister.stream_input.pcur_item = pinp_pitem;
    ut_lister.stream_output.pcur_item = pout_pitem;
    /*
    pack_data_entry_t* pitem = (time_diff_out_us <= time_diff_inp_us) ? out_pitem : inp_pitem;
    struct timeval start_tm = { .tv_sec = pitem->seconds, .tv_usec = 0 };
    ESP_LOGI(TAG, "Time to set: %llu", get_time_us(start_tm));
    ESP_LOGI(TAG, "Time actual: %llu", get_time_us(tv_temp));
    //settimeofday(&start_tm, NULL);
    gettimeofday(&tv_temp, NULL);
    ESP_LOGI(TAG, "Time changed: %llu", get_time_us(tv_temp));
    //esp_sync_counters_rtc_and_frc();
    */
    while(1)
    {
        // stream_t* pstream = NULL;
        // BaseType_t status = xQueueReceive(ut_lister.ut_queue_handle, 
        //                                         (void*)&pstream, 
        //                                         pdMS_TO_TICKS(UT_TASK_EVENT_TOUT_MS)); //portMAX_DELAY
        // if (status != pdTRUE) {
        //     // Timeout reading getting events
        //     ESP_LOGE(TAG, "UT Event read timeout.");
        // } else {
        //     // Got en event from port task waiting for data
        //     if (pstream) { 
        //         // Ready to read data
        //         ESP_LOGW("UT_TASK", "Stream %s, got timer event.", pstream->stream_name); //, pstream->stream_name
        //         if (pstream && pstream->pcur_item) {
        //             ESP_EARLY_LOGI("UT_TASK", "Send buf:%p, len:%d", pstream->pcur_item->pbuffer, pstream->pcur_item->capture_length);
        //             xStreamBufferSend(pstream->stream_buffer_handle, 
        //                                 (void*)pstream->pcur_item->pbuffer, 
        //                                 (size_t)pstream->pcur_item->capture_length, pdMS_TO_TICKS(UT_TASK_EVENT_TOUT_MS));
        //             ESP_LOG_BUFFER_HEXDUMP("UT_TASK", pstream->pcur_item->pbuffer, pstream->pcur_item->capture_length, ESP_LOG_WARN);
        //             // Todo: add handling of timeout
        //         }
        //     }
        // }
        vTaskDelay(1);
    }
    vTaskDelete(NULL);
}

static esp_err_t ut_init_descriptors(const char* port_prefix)
{
    esp_err_t ret = ESP_OK;

    if (asprintf(&ut_lister.stream_input.filename, "/spiffs/%s_input.pcap", port_prefix) == -1) {
        abort();
    }

    if (asprintf(&ut_lister.stream_output.filename, "/spiffs/%s_output.pcap", port_prefix) == -1) {
        abort();
    }
    UT_GOTO_ON_ERROR(ut_stream_open(&ut_lister.stream_input) != ESP_OK, err, TAG, "open input pcap filed.");
    UT_GOTO_ON_ERROR(ut_stream_open(&ut_lister.stream_output) != ESP_OK, err, TAG, "open output pcap filed.");
#if CONFIG_MB_UTEST_LOG
    ret = pcap_write_header(ut_lister.stream_input.pcap_handle, PCAP_LINK_TYPE_LOOPBACK);
    UT_GOTO_ON_ERROR(ret, err, TAG, "Pcap init input header failed.");
    ret = pcap_write_header(ut_lister.stream_output.pcap_handle, PCAP_LINK_TYPE_LOOPBACK);
    UT_GOTO_ON_ERROR(ret, err, TAG, "Pcap init output header failed.");
#elif CONFIG_MB_UTEST_OVERRIDE
    rewind(ut_lister.stream_output.pcap_handle->file);
    rewind(ut_lister.stream_input.pcap_handle->file);
    LIST_INIT(&ut_lister.stream_output.pack_entries);
    LIST_INIT(&ut_lister.stream_input.pack_entries);
    UT_GOTO_ON_ERROR(ut_read_packet_list(&ut_lister.stream_output),
                        err, TAG, "%s pcap read packet list failed.", ut_lister.stream_output.filename);
    UT_GOTO_ON_ERROR(ut_read_packet_list(&ut_lister.stream_input),
                        err, TAG, "%s pcap read packet list failed.", ut_lister.stream_output.filename);
    esp_timer_create_args_t timer_conf = {
        .callback = ut_timer_cb,
        .arg = &ut_lister.stream_input,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "ut_tm_inp_poll"
    };
    // Create Modbus timer handlers for streams
    UT_GOTO_ON_ERROR(esp_timer_create(&timer_conf, &ut_lister.stream_input.timer_handle),
                            err, TAG, "create input stream timer failed.");
    timer_conf.callback = ut_timer_cb;
    timer_conf.arg = &ut_lister.stream_output;
    timer_conf.name = "ut_tm_out_poll";
    UT_GOTO_ON_ERROR(esp_timer_create(&timer_conf, &ut_lister.stream_output.timer_handle),
                            err, TAG, "create output stream timer failed.");
    // Create r/w event queue
    ut_lister.stream_output.queue_handle = xQueueCreate(2, sizeof(stream_t*));
    ut_lister.stream_input.queue_handle = xQueueCreate(2, sizeof(stream_t*));
    UT_RETURN_ON_FALSE((ut_lister.stream_output.queue_handle && ut_lister.stream_input.queue_handle), ESP_ERR_INVALID_STATE,
                                    TAG, "Could not create lister queue.");
    // Create stream buffers (one per IO stream).
    // These buffers will be used to transfer data to/from listener/sender
    ut_lister.stream_input.stream_buffer_handle = xStreamBufferCreate(UT_STREAM_BUF_SIZE, 4);
    ut_lister.stream_output.stream_buffer_handle = xStreamBufferCreate(UT_STREAM_BUF_SIZE, 4);
    UT_RETURN_ON_FALSE((ut_lister.stream_output.stream_buffer_handle && ut_lister.stream_input.stream_buffer_handle), ESP_ERR_INVALID_STATE,
                                    TAG, "Could not create stream buffer.");
    // Create task for packet processing
    BaseType_t err = xTaskCreatePinnedToCore(ut_lister_task,
                                    "ut_lister",
                                    UT_TASK_STACK_SIZE,
                                    NULL,
                                    UT_TASK_PRIORITY,
                                    &ut_lister.ut_task_handle,
                                    UT_TASK_AFFINITY);
    if (err != pdTRUE)
    {
        ESP_LOGE(TAG, "Could not create packet lister task");
        vTaskDelete(ut_lister.ut_task_handle);
    }
#endif

err:
    return ret;
}

static esp_err_t ut_close_descriptors(void)
{
    esp_err_t ret = ESP_OK;
    
    UT_GOTO_ON_ERROR(ut_stream_close(&ut_lister.stream_input) != ESP_OK, err, TAG,
                            "close input pcap (%s) filed.", ut_lister.stream_input.filename);
    UT_GOTO_ON_ERROR(ut_stream_close(&ut_lister.stream_output) != ESP_OK, err, TAG,
                            "close output pcap (%s) filed.", ut_lister.stream_output.filename);
#if CONFIG_MB_UTEST_OVERRIDE
    vTaskSuspend(ut_lister.ut_task_handle);
    ut_free_packet_list(&ut_lister.stream_input);
    ut_free_packet_list(&ut_lister.stream_output);
    if (ut_lister.stream_output.timer_handle) {
        esp_timer_stop(ut_lister.stream_output.timer_handle);
        esp_timer_delete(ut_lister.stream_output.timer_handle);
    }
    if (ut_lister.stream_input.timer_handle) {
        esp_timer_stop(ut_lister.stream_input.timer_handle);
        esp_timer_delete(ut_lister.stream_input.timer_handle);
    }
    if (ut_lister.stream_input.queue_handle) {
        vQueueDelete(ut_lister.stream_input.queue_handle);
    }
    if (ut_lister.stream_output.queue_handle) {
        vQueueDelete(ut_lister.stream_output.queue_handle);
    }
    vStreamBufferDelete(ut_lister.stream_input.stream_buffer_handle);
    vStreamBufferDelete(ut_lister.stream_output.stream_buffer_handle);
    vTaskDelete(ut_lister.ut_task_handle);
#endif
    // free the filename pointer
    free(ut_lister.stream_input.filename);
    free(ut_lister.stream_output.filename);
err:
    return ret;
}

esp_err_t ut_stream_capture_packet(direction_t direction, void *payload, uint32_t length, uint16_t crc)
{

    stream_t stream = (direction != DIRECTION_INPUT) ? (direction == DIRECTION_OUTPUT) ?
                                        ut_lister.stream_output : ut_lister.stream_input : ut_lister.stream_input;
    pcap_file_handle_t pcap_handle = stream.pcap_handle;
    uint32_t size = ftell(pcap_handle->file);
    ESP_LOGW("DEBUG", "%s: %p, %u, %u ", stream.stream_name, payload, length, size);
    struct timeval tv_now;
    gettimeofday(&tv_now, NULL);
    esp_err_t err = pcap_capture_packet(pcap_handle, payload, length, tv_now.tv_sec, tv_now.tv_usec);
    ESP_LOG_BUFFER_HEX_LEVEL(stream.stream_name, (void*)payload, length, ESP_LOG_INFO);
    UT_RETURN_ON_ERROR(err, TAG, ".pcap file is already closed");
    return err;
}

esp_err_t ut_init(const char* port_prefix)
{
    ESP_LOGI(TAG, "Initializing SPIFFS");

    esp_vfs_spiffs_conf_t conf = {
      .base_path = UT_FS_BASE_PATH,
      .partition_label = NULL,
      .max_files = UT_MAX_FILES,
      .format_if_mount_failed = false
    };

    // Use settings defined above to initialize and mount SPIFFS file system.
    // Note: esp_vfs_spiffs_register is an all-in-one convenience function.
    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format file system");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        goto err;
    }

    size_t total = 0, used = 0;
    UT_GOTO_ON_ERROR(esp_spiffs_info(NULL, &total, &used) != ESP_OK, err, TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
    ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    UT_GOTO_ON_ERROR(ut_init_descriptors(port_prefix) != ESP_OK, err, TAG, "Failed to open descriptors (%s)", esp_err_to_name(ret));
err:
    return ret;
}

void ut_close(void) 
{
    esp_err_t ret = ESP_OK;
    // close input and out packets pcap files first
    if (ut_close_descriptors() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to close file descriptors (%s)", esp_err_to_name(ret));
    }
    // All done, unmount partition and disable SPIFFS
    esp_vfs_spiffs_unregister(NULL);
    ESP_LOGI(TAG, "SPIFFS unmounted");
}
