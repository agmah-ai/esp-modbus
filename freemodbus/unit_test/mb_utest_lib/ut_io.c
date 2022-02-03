/*
 * SPDX-FileCopyrightText: 2018-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_spiffs.h"
#include "esp_timer.h"
//#include "mbport_stubs.h" // for read callbacks
#include "ut_io.h"
#include "pcap.h"
#include "sdkconfig.h"

static const char *TAG = "UT_TAG";

// Base unit test class instance
static ut_lister_t ut_lister = {
    .port_prefix = NULL,            // name prefix of the port to be tested
    .ut_task_handle = NULL,         // unit test task handle
    .notif_queue_handle = NULL,     // unit test notification quue handle
    .packet_index = 0,
    .pstreams = NULL,               // array of streams
    .streams_counter = 0            // number of opened streams
};


// Timer task to send notification on timeout expiration
static void IRAM_ATTR ut_timer_cb(void *param)
{
    stream_t* pstream = (stream_t*)param;
    BaseType_t status = xQueueSend(pstream->queue_handle,
                                    (const void*)&pstream, 
                                    pdMS_TO_TICKS(UT_TASK_EVENT_TOUT_MS));
    if ((status == pdTRUE) && pstream) {
        if (pstream->pcur_item) {
            UT_LOGW("TIMER", "%s[%d], send notification.", pstream->stream_name, (pstream->curr_index - 1));
        }
    } else {
        UT_LOGE("TIMER", "Could not notify stream: [%s].", pstream->stream_name);
    }
}

static esp_err_t ut_stream_open(stream_t *pstream);

static double time_diff(struct timeval x, struct timeval y)
{
  double x_ms, y_ms;

  x_ms = (double)x.tv_sec * 1000000 + (double)x.tv_usec;
  y_ms = (double)y.tv_sec * 1000000 + (double)y.tv_usec;
//
  return abs(y_ms - x_ms);
}

uint64_t get_time_us(struct timeval time)
{
    int64_t time_us = (int64_t)time.tv_sec * 1000000L + (int64_t)time.tv_usec;
    return time_us;
}

struct timeval get_time_val(uint64_t time_us)
{
    struct timeval time = { 0, 0 };
    time.tv_sec = time_us / 1000000L;
    time.tv_usec = time_us - time.tv_sec;
    return time;
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

stream_t* ut_stream_get_handle(int stream_id)
{
    UT_RETURN_ON_FALSE((stream_id <= ut_lister.streams_counter), NULL, TAG,
                    "Invalid stream identifier %d.", stream_id);
    stream_t* pstream = ut_lister.pstreams[stream_id];
    return pstream;
}

// Search the packet entry in the list from absolute index with the minimal time difference 
// returns the entry and time difference which corresponds to the condition or NULL if it is not found in the list
pack_data_entry_t* ut_find_packet_index(int index, stream_t* pstream, struct timeval tv_start, uint64_t* diff_us)
{
    UT_RETURN_ON_FALSE(pstream, NULL, TAG, "%s invalid arguments.", __func__);
    struct timeval tm_temp;
    uint64_t time_start = get_time_us(tv_start);
    uint64_t time_cur = 0;
    uint64_t time_prev = 0;
    uint64_t time_diff_us = 0;
    pack_data_entry_t* it = NULL;
    LIST_FOREACH(it, &pstream->pack_entries, entries) {
        tm_temp.tv_sec = it->seconds;
        tm_temp.tv_usec = it->microseconds;
        time_cur = get_time_us(tm_temp);
        if (index == (int)it->packet_index) {
            time_diff_us = (time_cur > time_start) ? (time_cur - time_start) : 0;
            if (diff_us) {
                *diff_us = time_diff_us;
            }
            return it;
        }
        time_prev = time_cur;
    }
    *diff_us = 0;
    return NULL;
}

static esp_err_t ut_stream_read_buffer(stream_t *pstream, pack_data_entry_t* pitem)
{
    UT_RETURN_ON_FALSE(pstream && pitem, ESP_ERR_INVALID_ARG, TAG, "invalid input pointers.");
    // If current item is inserted and not exist in the file just return the buffer to its data
    if (pitem->pbuffer && (pitem->file_pos == -1)) {
        return ESP_OK;
    }
    pitem->pbuffer = calloc(1, pitem->capture_length);
    if (pitem->pbuffer) {
        fseek(pstream->pcap_handle->file, pitem->file_pos, SEEK_SET);
        size_t real_read = fread(pitem->pbuffer, sizeof(uint8_t), pitem->capture_length, pstream->pcap_handle->file);
        if (real_read != pitem->capture_length) {
            UT_LOGE(TAG, "read buffer failed.");
            free(pitem->pbuffer);
            return ESP_ERR_INVALID_STATE;
        }
        return ESP_OK;
    }
    return ESP_ERR_NO_MEM;
}

// Set packet data of packet as defined by parameters
esp_err_t ut_stream_set_packet_data(int stream_id, int index, void *payload, uint32_t length)
{
    pack_data_entry_t* pcur_item = NULL;
    uint64_t time_diff_us = 0;
    struct timeval tv_temp = {0, 0};
    stream_t* pstream = ut_stream_get_handle(stream_id);
    UT_RETURN_ON_FALSE(pstream, ESP_ERR_INVALID_ARG, TAG,
                            "Stream %d is invalid.", stream_id);
    pcur_item = ut_find_packet_index(index, pstream, tv_temp, &time_diff_us);
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
esp_err_t ut_stream_insert_packet(int stream_id, int index, void *payload, uint32_t length)
{
    pack_data_entry_t* pcur_item = NULL;
    pack_data_entry_t* it = NULL;
    struct timeval tv_temp = {0, 0};
    uint64_t time_diff_us = 0;
    struct timeval tv_now;

    stream_t* pstream = ut_stream_get_handle(stream_id);

    UT_RETURN_ON_FALSE(pstream, ESP_ERR_INVALID_ARG, TAG,
                            "Stream %d is invalid.", stream_id);

    pcur_item = ut_find_packet_index(index, pstream, tv_temp, &time_diff_us);
    UT_RETURN_ON_FALSE(pcur_item, ESP_ERR_INVALID_STATE, TAG, "the index %d not found in the list.", index);
    pack_data_entry_t* pack_entry = calloc(1, sizeof(pack_data_entry_t));
    UT_RETURN_ON_FALSE(pack_entry, ESP_ERR_NO_MEM, TAG, "No memory for packet entry.");
    gettimeofday(&tv_now, NULL);
    pack_entry->packet_index = index;
    pack_entry->seconds = tv_now.tv_sec;
    pack_entry->microseconds = tv_now.tv_usec;
    pack_entry->capture_length = length;
    pack_entry->packet_length = length;
    pack_entry->file_pos = -1; // indicates that packet is not indexed in the file stream
    void* pdata = calloc(1, length);
    UT_RETURN_ON_FALSE(pdata, ESP_ERR_NO_MEM, TAG, "No memory for packet data.");
    memcpy(pdata, payload, length);
    pack_entry->pbuffer = pdata;
    // capture the crc of the payload, shall be used to check packet correctness
    //pack_entry->crc = usMBCRC16((uint8_t*)pdata, (length - 2));
    LIST_INSERT_AFTER(pcur_item, pack_entry, entries);
    for(it = LIST_NEXT(pack_entry, entries); it != NULL; it = LIST_NEXT(it, entries)) {
        it->packet_index++;
        if (it->pbuffer) {
            ESP_LOG_BUFFER_HEX_LEVEL("pack", (void*)it->pbuffer, (uint16_t)it->capture_length, ESP_LOG_INFO);
        }
    }
    pstream->packet_count++;
    return ESP_OK;
}

static esp_err_t ut_stream_open(stream_t *pstream)
{
    esp_err_t ret = ESP_OK;
    // Create file to read/write, binary format
    FILE *fp = fopen(pstream->filename, "rb+");
    UT_GOTO_ON_FALSE(fp, ESP_FAIL, err, TAG, "Open file %s failed.", pstream->filename);
    pcap_config_t pcap_config = {
        .fp = fp,
        .major_version = PCAP_DEFAULT_VERSION_MAJOR,
        .minor_version = PCAP_DEFAULT_VERSION_MINOR,
        .time_zone = PCAP_DEFAULT_TIME_ZONE_GMT,
    };
    UT_GOTO_ON_ERROR(pcap_new_session(&pcap_config, &pstream->pcap_handle), err, TAG, "PCAP init failed.");
    pstream->is_opened = true;
    UT_LOGI(TAG, "Open file %s successfully.", pstream->filename);
    pcap_file_header_t fh;
    ret = pcap_read_header(pstream->pcap_handle, &fh);
    if (ret) {
          rewind(fp);
          ret = pcap_write_header(pstream->pcap_handle, PCAP_LINK_TYPE_LOOPBACK);
          UT_GOTO_ON_ERROR(ret, err, TAG, "PCAP write filed.");
    }
    pstream->link_type_set = false;
    return ESP_OK;
err:
    if (fp) {
        fclose(fp);
    }
    return ret;
}

static esp_err_t ut_stream_close(stream_t *pstream)
{
    esp_err_t ret = ESP_OK;
    UT_GOTO_ON_FALSE(pstream->is_opened, ESP_ERR_INVALID_STATE, err, TAG, "PCAP file is already closed");
    UT_GOTO_ON_ERROR(pcap_del_session(pstream->pcap_handle) != ESP_OK, err, TAG, "stop PCAP session failed");
    pstream->is_opened = false;
    pstream->link_type_set = false;
    pstream->pcap_handle = NULL;
err:
    return ret;
}

void ut_stream_print_list(int stream_id)
{
    pack_data_entry_t* it;
    pack_data_entry_t* it_temp;
    stream_t* pstream = ut_stream_get_handle(stream_id);

    UT_RETURN_ON_FALSE(pstream, ; , TAG,
                    "Stream %d is invalid.", stream_id);
    LIST_FOREACH_SAFE(it, &pstream->pack_entries, entries, it_temp) {
        UT_LOGI("pack", "i:%d, p:%p, s:%d, pos:%d", it->packet_index, it->pbuffer, it->capture_length, it->file_pos);
        if (it->pbuffer) {
            ESP_LOG_BUFFER_HEX_LEVEL("pack", (void*)it->pbuffer, (uint16_t)it->capture_length, ESP_LOG_INFO);
        }
    }
}

static void ut_stream_free_packet_list(stream_t* pstream) {
    pack_data_entry_t* it; 
    pack_data_entry_t* it_temp;

    LIST_FOREACH_SAFE(it, &pstream->pack_entries, entries, it_temp) {
        if (it->pbuffer) {
            free(it->pbuffer);
            it->pbuffer = NULL;
        }
        LIST_REMOVE(it, entries);
        free(it);
    }
}

static esp_err_t ut_stream_read_entries(stream_t* pstream)
{
    pcap_file_header_t file_header;
    esp_err_t ret = ESP_OK;
    uint32_t save_index = 0;
    uint32_t index = 0;
    long size = 0;
    save_index = ftell(pstream->pcap_handle->file);
    fseek(pstream->pcap_handle->file, 0L, SEEK_END);
    size = ftell(pstream->pcap_handle->file);
    UT_LOGD("DEBUG", "File %s, cur_index = %d, size = %lu", pstream->filename, save_index, size);
    rewind(pstream->pcap_handle->file);
    size_t real_read = fread(&file_header, sizeof(pcap_file_header_t), 1, pstream->pcap_handle->file);
    UT_RETURN_ON_FALSE(real_read == 1, ESP_FAIL, TAG, "read pcap file header failed");
    index += sizeof(pcap_file_header_t);
    uint32_t packet_num = 0;
    pcap_packet_header_t packet_header;
    pack_data_entry_t* pcur_item = LIST_FIRST(&pstream->pack_entries); // NULL if list is empty
    while (index < size) {
        real_read = fread(&packet_header, sizeof(pcap_packet_header_t), 1, pstream->pcap_handle->file);
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
        real_read = fread(pdata, sizeof(uint8_t), packet_header.capture_length, pstream->pcap_handle->file);
        UT_GOTO_ON_FALSE(real_read == packet_header.capture_length, ESP_FAIL, err, TAG, "PCAP read payload failed");
        // capture the crc of the payload, shall be used to check packet correctness
        //pack_entry->crc = usMBCRC16((uint8_t*)pdata, (packet_header.capture_length - 2));
        free(pdata);
        pack_entry->pbuffer = NULL;
        index += packet_header.capture_length;
        if (!pcur_item) {
            LIST_INSERT_HEAD(&pstream->pack_entries, pack_entry, entries);
        } else {
            LIST_INSERT_AFTER(pcur_item, pack_entry, entries);
        }
        pcur_item = pack_entry;
        packet_num++;
    }
    pstream->packet_count = packet_num;
    // Set current pointer onto first packet
    pstream->pcur_item = LIST_FIRST(&pstream->pack_entries);
err:
    fseek(pstream->pcap_handle->file, save_index, SEEK_SET);
    UT_LOGI("PACKET", "File: %s, Found: %d packets.", pstream->filename, packet_num);
    return ret;
}

esp_err_t ut_stream_reset_data(int stream_id)
{
    esp_err_t err = ESP_OK;
    stream_t* pstream = ut_stream_get_handle(stream_id);

    UT_RETURN_ON_FALSE(pstream, ESP_ERR_INVALID_ARG, TAG,
                    "Stream %d is invalid.", stream_id);

    size_t data_len = xStreamBufferBytesAvailable(pstream->stream_buffer_handle);
    // Reset buffer is there is data
    if (data_len != 0) {
        err = xStreamBufferReset(pstream->stream_buffer_handle) == pdPASS ? ESP_OK : ESP_ERR_INVALID_STATE;
    }
    return err;
}

static esp_err_t ut_stream_set_timer(stream_t* pstream, uint64_t time_diff)
{
    esp_timer_stop(pstream->timer_handle);
    esp_err_t ret = esp_timer_start_once(pstream->timer_handle, time_diff);
    UT_RETURN_ON_FALSE((ret == ESP_OK),
                        ESP_ERR_INVALID_STATE, TAG, "Can not start timer for stream %s, , err=%x.",pstream->stream_name, ret);
    return ESP_OK;
}

static esp_err_t ut_stream_handle_data(stream_t* pstream)
{
    uint64_t time_diff_us = 0;
    struct timeval tv_temp = { pstream->pcur_item->seconds, pstream->pcur_item->microseconds };
    pack_data_entry_t* pitem = ut_find_packet_index(pstream->curr_index, pstream, tv_temp, &time_diff_us);
    
    UT_RETURN_ON_FALSE(pitem, ESP_ERR_INVALID_STATE, TAG, "the input packet [%d] not found in the log.", pstream->curr_index);
    UT_LOGW(pstream->stream_name, "Packet index #%d, time_diff_us: %llu, file_pos:%d", pitem->packet_index, time_diff_us, pitem->file_pos);
    UT_RETURN_ON_FALSE((ut_stream_read_buffer(pstream, pitem) == ESP_OK),
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
    ut_stream_set_timer(pstream, time_diff_us);
    return ESP_OK;
}

// Notify the event task about new read/write event then wait timer event and returns buffer length
esp_err_t ut_stream_wait_notification(int stream_id, uint32_t time_ticks, size_t* plength)
{
    esp_err_t err = ESP_OK;
    stream_t* pstream = ut_stream_get_handle(stream_id);

    UT_RETURN_ON_FALSE(pstream, ESP_ERR_INVALID_ARG, TAG,
                            "Stream %s is invalid.", pstream->stream_name);
    // Check data readiness notification from timer 
    stream_t* pnotif_stream = NULL;
    BaseType_t status = xQueueReceive(pstream->queue_handle,
                                        (void*)&pnotif_stream, 
                                        time_ticks);
    UT_RETURN_ON_FALSE((status && pnotif_stream), ESP_ERR_TIMEOUT, TAG,
                            "Stream %s get data timeout (%d).", pstream->stream_name, time_ticks);
    if (plength) {
        *plength = pnotif_stream->pcur_item->capture_length;
    }
    return err;
}


// Notify the event task about new read/write event
esp_err_t ut_stream_send_notification(int stream_id, uint32_t time_ticks, size_t* plength)
{
    esp_err_t err = ESP_ERR_INVALID_STATE;
    stream_t* pstream = ut_stream_get_handle(stream_id);

    UT_RETURN_ON_FALSE(pstream, ESP_ERR_INVALID_ARG, TAG,
                    "Stream %d is invalid.", stream_id);

    // Send notification into lister task to iterate to next data
    BaseType_t status = xQueueSend(ut_lister.notif_queue_handle,
                            (const void*)&pstream, 
                            pdMS_TO_TICKS(UT_TASK_EVENT_TOUT_MS));
    UT_RETURN_ON_FALSE(status, ESP_ERR_INVALID_STATE, TAG,
                            "Stream %d send notification fail.", stream_id);
    if (plength) {
        *plength = pstream->pcur_item->capture_length;
    }
    return err;
}

// Notify the event task about new read/write event then wait timer event and returns buffer length
esp_err_t ut_stream_get_notification(int stream_id, uint32_t time_ticks, size_t* plength)
{
    esp_err_t err = ESP_ERR_INVALID_STATE;
    stream_t* pstream = ut_stream_get_handle(stream_id);

    UT_RETURN_ON_FALSE(pstream, ESP_ERR_INVALID_ARG, TAG,
                    "Stream %d is invalid.", stream_id);

    size_t data_len = xStreamBufferBytesAvailable(pstream->stream_buffer_handle);
    // Do not wait notification if the data available in the buffer
    if (data_len == 0) {
        // Send notification into lister task to iterate to next data
        BaseType_t status = xQueueSend(ut_lister.notif_queue_handle,
                                (const void*)&pstream, 
                                pdMS_TO_TICKS(UT_TASK_EVENT_TOUT_MS));
        UT_RETURN_ON_FALSE(status, ESP_ERR_INVALID_STATE, TAG,
                                "Stream %d send notification fail.", stream_id);
        // Check data readiness notification from timer
        stream_t* pnotif_stream = NULL;
        status = xQueueReceive(pstream->queue_handle,
                                (void*)&pnotif_stream, 
                                time_ticks); //portMAX_DELAY
        UT_RETURN_ON_FALSE((status && pnotif_stream), ESP_ERR_TIMEOUT, TAG,
                                "Stream %s get data timeout (%d ticks).", pstream->stream_name, time_ticks);
        //err = ut_stream_handle_data(pstream); // handling of the data
        if (plength) {
            *plength = pnotif_stream->pcur_item->capture_length;
        }
    } else {
        if (plength) {
            *plength = data_len;
        }
    }
    return ESP_OK;
}

esp_err_t ut_stream_buffer_reset(int stream_id)
{
    stream_t* pstream = ut_stream_get_handle(stream_id);
    UT_RETURN_ON_FALSE(pstream, ESP_ERR_INVALID_STATE, TAG,
                        "Stream %d handle is invalid.", stream_id);
    BaseType_t ret = xStreamBufferReset(pstream->stream_buffer_handle); //portMAX_DELAY pdMS_TO_TICKS(timeout_ms)
    UT_RETURN_ON_FALSE((ret == pdPASS), ESP_ERR_INVALID_STATE, TAG,
                        "Could not reset stream %s buffer.", pstream->stream_name);
    return ESP_OK;
}

int ut_stream_get_data(int stream_id, void* pdata, size_t data_length, uint32_t timeout_ms)
{
    UT_RETURN_ON_FALSE(pdata, 0, TAG,
                        "Invalid buffer pointer to get data.");
    stream_t* pstream = ut_stream_get_handle(stream_id);

    UT_RETURN_ON_FALSE(pstream, 0, TAG,
                        "Stream %d handle is invalid.", stream_id);
    size_t data_len = xStreamBufferReceive(pstream->stream_buffer_handle,
                                            (void*)pdata, data_length, pdMS_TO_TICKS(timeout_ms));
    if (data_len > 0) {
        ESP_LOG_BUFFER_HEXDUMP(pstream->stream_name, pdata, data_len, ESP_LOG_WARN);
    }
    return data_len;
}

static void ut_lister_task(void *parg)
{      
    while(1)
    {
        stream_t* pstream = NULL;
        BaseType_t status = xQueueReceive(ut_lister.notif_queue_handle, 
                                                (void*)&pstream, 
                                                portMAX_DELAY); //pdMS_TO_TICKS(UT_TASK_EVENT_TOUT_MS)
        if (status != pdTRUE) {
            // Timeout reading getting events
            UT_LOGE(TAG, "UT Event read timeout.");
        } else {
            // Got en event from port task waiting for data
            if (pstream && pstream->pcur_item) { 
                // Ready to read data
                UT_LOGW("UT_TASK", "Stream %s, handle event.", pstream->stream_name);
                if (pstream->pcur_item) {
                    UT_LOGW("UT_TASK", "Handle stream (%s), buf:%p, len:%d", pstream->stream_name, pstream->pcur_item->pbuffer, pstream->pcur_item->capture_length);
                    // Handle stream data on event
                    esp_err_t err = ut_stream_handle_data(pstream);
                    if (err) {
                        UT_LOGW(TAG, "UT Lister is suspended.");
                    }
                }
            }
        }
        vTaskDelay(1);
    }
    vTaskDelete(NULL);
}

esp_err_t ut_stream_create(const char* stream_name, int* stream_id)
{
    esp_err_t ret = ESP_OK;

    stream_t* pstream = calloc(1, sizeof(stream_t));

    UT_RETURN_ON_FALSE((stream_name && pstream), 
                        ESP_ERR_INVALID_STATE,
                        TAG, "Could not create stream %s.", stream_name);

    pstream->stream_id = -1;
    pstream->stream_name = (char*) stream_name;
    pstream->is_opened = false;
    pstream->is_writing = false;
    pstream->pcur_item = NULL;
    pstream->curr_index = 0; 

    if (asprintf(&pstream->filename, "/spiffs/%s_%s.pcap", ut_lister.port_prefix, stream_name) == -1) {
        abort();
    }
    UT_GOTO_ON_ERROR(ut_stream_open(pstream), err, TAG, "Open file stream `%s`filed.", stream_name);

#if CONFIG_MB_UTEST_LOG
    // Initiate stream file handlers for writing new data
    ret = pcap_write_header(pstream->pcap_handle, PCAP_LINK_TYPE_LOOPBACK);
    UT_GOTO_ON_ERROR(ret, err, TAG, "Can not open stream file.");
#elif CONFIG_MB_UTEST_OVERRIDE
    // Start stream to initial position of stream data
    rewind(pstream->pcap_handle->file);
    // Initialize the stream data lists
    LIST_INIT(&pstream->pack_entries);
    // Read data from stream into data list
    UT_GOTO_ON_ERROR(ut_stream_read_entries(pstream),
                        err, TAG, "%s pcap read packet list failed.", pstream->filename);
    esp_timer_create_args_t timer_conf = {
        .callback = ut_timer_cb,
        .arg = pstream,
        .dispatch_method = ESP_TIMER_TASK,
        .name = pstream->stream_name
    };
    // Create Modbus timer handlers for streams
    UT_GOTO_ON_ERROR(esp_timer_create(&timer_conf, &pstream->timer_handle),
                            err, TAG, "create input stream timer failed.");
    // Create r/w event queue
    pstream->queue_handle = xQueueCreate(2, sizeof(stream_t*));
    UT_RETURN_ON_FALSE(pstream->queue_handle, 
                                    ESP_ERR_INVALID_STATE,
                                    TAG, "Could not create stream %s queue.", pstream->stream_name);
    // Create stream buffers (one per IO stream).
    // These buffers will be used to transfer data to/from listener/sender
    pstream->stream_buffer_handle = xStreamBufferCreate(UT_STREAM_BUF_SIZE, 4);
    UT_RETURN_ON_FALSE(pstream->stream_buffer_handle, ESP_ERR_INVALID_STATE,
                                    TAG, "Could not create stream buffer.");
#endif
    // Register stream in the UT lister
    pstream->stream_id = ut_lister.streams_counter++;
    ut_lister.pstreams[pstream->stream_id] = pstream;
    if (stream_id) {
        *stream_id = pstream->stream_id;
    }
err:
    return ret;
}

esp_err_t ut_stream_destroy(stream_t* pstream)
{

    esp_err_t ret = ESP_OK;

    UT_RETURN_ON_FALSE(pstream,
                        ESP_ERR_INVALID_ARG,
                        TAG, "Invalid stream handle.");

    UT_GOTO_ON_ERROR(ut_stream_close(pstream), err, TAG,
                            "Close %s stream (%s) filed.", pstream->stream_name, pstream->filename);
    pstream->pcap_handle = NULL;
#if CONFIG_MB_UTEST_OVERRIDE
    ut_stream_free_packet_list(pstream);
    if (pstream->timer_handle) {
        esp_timer_stop(pstream->timer_handle);
        esp_timer_delete(pstream->timer_handle);
        pstream->timer_handle = NULL;
    }
    if (pstream->queue_handle) {
        vQueueDelete(pstream->queue_handle);
        pstream->queue_handle = NULL;
    }
    vStreamBufferDelete(pstream->stream_buffer_handle);
    pstream->stream_buffer_handle = NULL;
#endif
    // free the filename pointer
    free(pstream->filename);
    pstream->filename = NULL;
    pstream->stream_name = NULL;
    pstream->pcur_item = NULL;
    free(pstream);
    if (ut_lister.streams_counter) {
        ut_lister.streams_counter--;
    }
err:
    return ret;
}

static esp_err_t ut_init_descriptors(const char* port_prefix)
{
    esp_err_t ret = ESP_OK;

    ut_lister.pstreams = calloc(UT_MAX_FILES, sizeof(stream_t*));
    ut_lister.streams_counter = 0;
    ut_lister.port_prefix = (char*)port_prefix;
    ut_lister.notif_queue_handle = xQueueCreate(4, sizeof(stream_t*));
    UT_RETURN_ON_FALSE(ut_lister.notif_queue_handle,
                                    ESP_ERR_INVALID_STATE,
                                    TAG, "Could not create lister queue.");
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
        UT_LOGE(TAG, "Could not create packet lister task");
        vTaskDelete(ut_lister.ut_task_handle);
    }

    return ret;
}

static esp_err_t ut_close_descriptors(void)
{
    esp_err_t ret = ESP_OK;

    vQueueDelete(ut_lister.notif_queue_handle);
    // Free streams memory if not free at destroy
    for(int idx = 0; ut_lister.streams_counter > 0; idx++) {
        if (ut_lister.pstreams[idx]) {
            ut_stream_destroy(ut_lister.pstreams[idx]);
            ut_lister.pstreams[idx] = NULL;
        }
    }
    free(ut_lister.pstreams);
    vTaskDelete(ut_lister.ut_task_handle);
    ut_lister.pstreams = NULL;
    ut_lister.streams_counter = 0;
    return ret;
}

esp_err_t ut_stream_capture_packet(int stream_id, void *payload, uint32_t length, uint16_t crc)
{

    stream_t* pstream = ut_stream_get_handle(stream_id);

    UT_RETURN_ON_FALSE(pstream, ESP_ERR_INVALID_ARG, TAG,
                        "Stream %d handle is invalid.", stream_id);
    pcap_file_handle_t pcap_handle = pstream->pcap_handle;
    uint32_t size = ftell(pcap_handle->file);
    UT_LOGW("DEBUG", "%s: %p, %u, %u ", pstream->stream_name, payload, length, size);
    struct timeval tv_now;
    gettimeofday(&tv_now, NULL);
    esp_err_t err = pcap_capture_packet(pcap_handle, payload, length, tv_now.tv_sec, tv_now.tv_usec);
    if (payload) {
        ESP_LOG_BUFFER_HEX_LEVEL(pstream->stream_name, (void*)payload, length, ESP_LOG_INFO);
    }
    UT_RETURN_ON_ERROR(err, TAG, ".pcap file is already closed");
    return err;
}

esp_err_t ut_init(const char* port_prefix)
{
    UT_LOGI(TAG, "Initializing SPIFFS system.");

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
            UT_LOGE(TAG, "Failed to find SPIFFS partition");
        } else {
            UT_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        goto err;
    }

    size_t total = 0, used = 0;
    UT_GOTO_ON_ERROR(esp_spiffs_info(NULL, &total, &used) != ESP_OK, err, TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
    UT_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    UT_GOTO_ON_ERROR(ut_init_descriptors(port_prefix) != ESP_OK, err, TAG, "Failed to open descriptors (%s)", esp_err_to_name(ret));
err:
    return ret;
}

void ut_close(void) 
{
    esp_err_t ret = ESP_OK;
    // close input and out packets pcap files first
    if (ut_close_descriptors() != ESP_OK) {
        UT_LOGE(TAG, "Failed to close file descriptors (%s)", esp_err_to_name(ret));
    }
    // All done, unmount partition and disable SPIFFS
    esp_vfs_spiffs_unregister(NULL);
    UT_LOGI(TAG, "SPIFFS unmounted");
}
