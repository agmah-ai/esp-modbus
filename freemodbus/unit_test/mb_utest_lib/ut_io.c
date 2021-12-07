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
    .pcap_rt_input = { .is_opened = false,
                        .is_writing = false,
                        .link_type_set = false,
                        .filename = NULL,
                        .stream_name = "input",
                        .pcap_handle = NULL,
                        .link_type = PCAP_LINK_TYPE_LOOPBACK
                     },
    .pcap_rt_output = { .is_opened = false,
                        .is_writing = false,
                        .link_type_set = false,
                        .filename = NULL,
                        .stream_name = "output",
                        .pcap_handle = NULL,
                        .link_type = PCAP_LINK_TYPE_LOOPBACK
                      },
    .ut_timer_handler = NULL,
    .pcur_item = NULL,
    .packet_index = 0
};

static void IRAM_ATTR ut_timer_cb(void *param)
{
    //ESP_EARLY_LOGI("TIMER", "callback triggered: %llu", esp_timer_get_time());
    if (ut_lister.pcur_item->pbuffer) {
        //ESP_EARLY_LOGI("TIMER", "buf:%p, len:%d", ut_lister.pcur_item->pbuffer, ut_lister.pcur_item->capture_length);
        mb_master_serial_poll_cb(ut_lister.pcur_item->pbuffer, ut_lister.pcur_item->capture_length);
    }
}

static esp_err_t ut_stream_open(pcap_runtime_t *pcap);

static double time_diff(struct timeval x, struct timeval y)
{
  double x_ms, y_ms;

  x_ms = (double)x.tv_sec * 1000000 + (double)x.tv_usec;
  y_ms = (double)y.tv_sec * 1000000 + (double)y.tv_usec;
//
  return y_ms - x_ms;
}

static uint64_t get_time_us(struct timeval time)
{
    int64_t time_us = (int64_t)time.tv_sec * 1000000L + (int64_t)time.tv_usec;
    return time_us;
}

// Search the packet entry in the list from absolute index with the minimal time difference and return the entry and time difference
// returns list pointer which corresponds to the condition or NULL if it is not found in the list
pack_data_entry_t* ut_find_packet_index(int index, pcap_runtime_t *pcap, struct timeval tv, uint64_t* diff_us)
{
    UT_RETURN_ON_FALSE((diff_us && pcap), NULL, TAG, "%s invalid arguments.", __func__);
    struct timeval tm_temp;
    uint64_t time_start = get_time_us(tv);
    uint64_t time_cur = 0;
    uint64_t time_prev = 0;
    pack_data_entry_t* it = NULL;
    LIST_FOREACH(it, &pcap->pack_entries, entries) {
        tm_temp.tv_sec = it->seconds;
        tm_temp.tv_usec = it->microseconds;
        time_cur = get_time_us(tm_temp);
        if (index == (int)it->packet_index) {
            *diff_us = (time_cur - time_start);
            return it;
        }
        time_prev = time_cur;
    }
    *diff_us = 0;
    return NULL;
}

static esp_err_t ut_read_buffer(pcap_runtime_t *pcap, pack_data_entry_t* pitem)
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

static esp_err_t ut_mbm_get_answer(int index, void *payload, uint32_t length)
{
    struct timeval tv_temp = {0, 0};
    uint64_t time_diff_us = 0;
    //gettimeofday(&tv_temp, NULL);
    uint16_t crc = usMBCRC16((uint8_t*)payload, (length - 2));
    pack_data_entry_t* pitem = ut_find_packet_index(index, &ut_lister.pcap_rt_output, tv_temp, &time_diff_us);
    if (pitem) {
        if (crc != pitem->crc) {
            ESP_LOGE(TAG, "Packet #%d, CRC:%u!=%u", pitem->packet_index, crc, pitem->crc);
        }
        UT_RETURN_ON_FALSE((ut_read_buffer(&ut_lister.pcap_rt_output, pitem) == ESP_OK),
                                    ESP_ERR_INVALID_STATE, TAG, "can not read packet data.");
        ESP_LOG_BUFFER_HEX_LEVEL("OUT", (void*)pitem->pbuffer, pitem->capture_length, ESP_LOG_INFO);
        tv_temp.tv_sec = pitem->seconds;
        tv_temp.tv_usec = pitem->microseconds;
        pitem = ut_find_packet_index(index, &ut_lister.pcap_rt_input, tv_temp, &time_diff_us);
        UT_RETURN_ON_FALSE(pitem, ESP_ERR_INVALID_STATE, TAG, "the input packet not found in the log.");
        ESP_LOGI(TAG, "Found packet index #%d, time_diff_us: %llu, file_pos:%d", pitem->packet_index, time_diff_us, pitem->file_pos);
        UT_RETURN_ON_FALSE((ut_read_buffer(&ut_lister.pcap_rt_input, pitem) == ESP_OK),
                            ESP_ERR_INVALID_STATE, TAG, "can not read packet data.");
        ut_lister.pcur_item = pitem;
        ESP_LOG_BUFFER_HEX_LEVEL("GET", (void*)pitem->pbuffer, pitem->capture_length, ESP_LOG_INFO);
        if (ut_lister.ut_timer_handler) {
            esp_timer_stop(ut_lister.ut_timer_handler);
            esp_timer_start_once(ut_lister.ut_timer_handler, time_diff_us);
        }
    }
    return ESP_OK;
}

esp_err_t ut_stream_override_packet(void *payload, uint32_t length)
{
    ESP_LOGI(TAG, "Search packet #%d, %p, %u", ut_lister.packet_index, payload, length);
    ESP_LOG_BUFFER_HEX_LEVEL("SEND", (void*)payload, (uint16_t)length, ESP_LOG_INFO);
    esp_err_t err = ut_mbm_get_answer(ut_lister.packet_index, payload, length);
    ut_lister.packet_index++;
    return err;
}

// Set packet data of packet as defined by parameters
esp_err_t ut_stream_set_packet_data(pcap_direction_t direction, int index, void *payload, uint32_t length)
{
    pack_data_entry_t* pcur_item = NULL;
    uint64_t time_diff_us = 0;
    struct timeval tv_temp = {0, 0};
    pcap_runtime_t pcap_runtime = (direction != PCAP_INPUT) ? (direction == PCAP_OUTPUT) ?
                                    ut_lister.pcap_rt_output : ut_lister.pcap_rt_input : ut_lister.pcap_rt_input;
    pcur_item = ut_find_packet_index(index, &pcap_runtime, tv_temp, &time_diff_us);
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
esp_err_t ut_stream_insert_packet(pcap_direction_t direction, int index, void *payload, uint32_t length)
{
    pack_data_entry_t* pcur_item = NULL;
    pack_data_entry_t* it = NULL;
    struct timeval tv_temp = {0, 0};
    uint64_t time_diff_us = 0;
    struct timeval tv_now;

    pcap_runtime_t pcap_runtime = (direction != PCAP_INPUT) ? (direction == PCAP_OUTPUT) ?
                                    ut_lister.pcap_rt_output : ut_lister.pcap_rt_input : ut_lister.pcap_rt_input;
    pcur_item = ut_find_packet_index(index, &pcap_runtime, tv_temp, &time_diff_us);
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
    pcap_runtime.packet_count++;
    return ESP_OK;
}

static esp_err_t ut_stream_open(pcap_runtime_t *pcap)
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

static esp_err_t ut_stream_close(pcap_runtime_t *pcap)
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

void ut_print_list(pcap_direction_t direction)
{
    pack_data_entry_t* it;
    pack_data_entry_t* it_temp;

    pcap_runtime_t pcap_runtime = (direction != PCAP_INPUT) ? (direction == PCAP_OUTPUT) ?
                                        ut_lister.pcap_rt_output : ut_lister.pcap_rt_input : ut_lister.pcap_rt_input;
    LIST_FOREACH_SAFE(it, &pcap_runtime.pack_entries, entries, it_temp) {
        ESP_LOGI("pack", "i:%d, p:%p, s:%d, pos:%d", it->packet_index, it->pbuffer, it->capture_length, it->file_pos);
        if (it->pbuffer) {
            ESP_LOG_BUFFER_HEX_LEVEL("pack", (void*)it->pbuffer, (uint16_t)it->capture_length, ESP_LOG_INFO);
        }
    }
}

static void ut_free_packet_list(pcap_runtime_t* pcap_runtime) {
    pack_data_entry_t* it; 
    pack_data_entry_t* it_temp;

    LIST_FOREACH_SAFE(it, &pcap_runtime->pack_entries, entries, it_temp) {
        if (it->pbuffer) {
            free(it->pbuffer);
            it->pbuffer = NULL;
        }
        LIST_REMOVE(it, entries);
        free(it);
    }
    return;
}

static esp_err_t ut_read_packet_list(pcap_runtime_t* pcap_runtime)
{
    pcap_file_header_t file_header;
    esp_err_t ret = ESP_OK;
    uint32_t save_index = 0;
    uint32_t index = 0;
    long size = 0;
    save_index = ftell(pcap_runtime->pcap_handle->file);
    fseek(pcap_runtime->pcap_handle->file, 0L, SEEK_END);
    size = ftell(pcap_runtime->pcap_handle->file);
    ESP_LOGD("DEBUG", "File %s, cur_index = %d, size = %lu", pcap_runtime->filename, save_index, size);
    rewind(pcap_runtime->pcap_handle->file);
    size_t real_read = fread(&file_header, sizeof(pcap_file_header_t), 1, pcap_runtime->pcap_handle->file);
    UT_RETURN_ON_FALSE(real_read == 1, ESP_FAIL, TAG, "read pcap file header failed");
    index += sizeof(pcap_file_header_t);
    uint32_t packet_num = 0;
    pcap_packet_header_t packet_header;
    pack_data_entry_t* pcur_item = LIST_FIRST(&pcap_runtime->pack_entries); // NULL if list is empty
    while (index < size) {
        real_read = fread(&packet_header, sizeof(pcap_packet_header_t), 1, pcap_runtime->pcap_handle->file);
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
        real_read = fread(pdata, sizeof(uint8_t), packet_header.capture_length, pcap_runtime->pcap_handle->file);
        UT_GOTO_ON_FALSE(real_read == packet_header.capture_length, ESP_FAIL, err, TAG, "pcap read payload failed");
        // capture the crc of the payload, shall be used to check packet correctness
        pack_entry->crc = usMBCRC16((uint8_t*)pdata, (packet_header.capture_length - 2));
        free(pdata);
        pack_entry->pbuffer = NULL;
        index += packet_header.capture_length;
        if (!pcur_item) {
            LIST_INSERT_HEAD(&pcap_runtime->pack_entries, pack_entry, entries);
        } else {
            LIST_INSERT_AFTER(pcur_item, pack_entry, entries);
        }
        pcur_item = pack_entry;
        packet_num++;
    }
    pcap_runtime->packet_count = packet_num;
err:
    fseek(pcap_runtime->pcap_handle->file, save_index, SEEK_SET);
    ESP_LOGI("PACKET", "File: %s, Found: %d packets.", pcap_runtime->filename, packet_num);

    return ret;
}

static esp_err_t ut_init_descriptors(const char* port_prefix)
{
    esp_err_t ret = ESP_OK;

    if (asprintf(&ut_lister.pcap_rt_input.filename, "/spiffs/%s_input.pcap", port_prefix) == -1) {
        abort();
    }

    if (asprintf(&ut_lister.pcap_rt_output.filename, "/spiffs/%s_output.pcap", port_prefix) == -1) {
        abort();
    }
    UT_GOTO_ON_ERROR(ut_stream_open(&ut_lister.pcap_rt_input) != ESP_OK, err, TAG, "open input pcap filed.");
    UT_GOTO_ON_ERROR(ut_stream_open(&ut_lister.pcap_rt_output) != ESP_OK, err, TAG, "open output pcap filed.");
#if CONFIG_MB_UTEST_LOG
    ret = pcap_write_header(ut_lister.pcap_rt_input.pcap_handle, PCAP_LINK_TYPE_LOOPBACK);
    UT_GOTO_ON_ERROR(ret, err, TAG, "Pcap init input header failed.");
    ret = pcap_write_header(ut_lister.pcap_rt_output.pcap_handle, PCAP_LINK_TYPE_LOOPBACK);
    UT_GOTO_ON_ERROR(ret, err, TAG, "Pcap init output header failed.");
#elif CONFIG_MB_UTEST_OVERRIDE
    rewind(ut_lister.pcap_rt_output.pcap_handle->file);
    rewind(ut_lister.pcap_rt_input.pcap_handle->file);
    LIST_INIT(&ut_lister.pcap_rt_output.pack_entries);
    LIST_INIT(&ut_lister.pcap_rt_input.pack_entries);
    UT_GOTO_ON_ERROR(ut_read_packet_list(&ut_lister.pcap_rt_output),
                        err, TAG, "%s pcap read packet list failed.", ut_lister.pcap_rt_output.filename);
    UT_GOTO_ON_ERROR(ut_read_packet_list(&ut_lister.pcap_rt_input),
                        err, TAG, "%s pcap read packet list failed.", ut_lister.pcap_rt_output.filename);
    esp_timer_create_args_t timer_conf = {
        .callback = ut_timer_cb,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "ut_tm_poll"
    };
    // Create Modbus timer
    UT_GOTO_ON_ERROR(esp_timer_create(&timer_conf, &ut_lister.ut_timer_handler),
                            err, TAG, "create poll timer failed.");

#endif

err:
    return ret;
}

static esp_err_t ut_close_descriptors(void)
{
    esp_err_t ret = ESP_OK;
    
    UT_GOTO_ON_ERROR(ut_stream_close(&ut_lister.pcap_rt_input) != ESP_OK, err, TAG,
                            "close input pcap (%s) filed.", ut_lister.pcap_rt_input.filename);
    UT_GOTO_ON_ERROR(ut_stream_close(&ut_lister.pcap_rt_output) != ESP_OK, err, TAG,
                            "close output pcap (%s) filed.", ut_lister.pcap_rt_output.filename);
#if CONFIG_MB_UTEST_OVERRIDE
    ut_free_packet_list(&ut_lister.pcap_rt_input);
    ut_free_packet_list(&ut_lister.pcap_rt_output);
    if (ut_lister.ut_timer_handler) {
        esp_timer_stop(ut_lister.ut_timer_handler);
        esp_timer_delete(ut_lister.ut_timer_handler);
    }
#endif
    // free the filename pointer
    free(ut_lister.pcap_rt_input.filename);
    free(ut_lister.pcap_rt_output.filename);
err:
    return ret;
}

esp_err_t ut_stream_capture_packet(pcap_direction_t direction, void *payload, uint32_t length, uint16_t crc)
{

    pcap_runtime_t pcap_runtime = (direction != PCAP_INPUT) ? (direction == PCAP_OUTPUT) ?
                                        ut_lister.pcap_rt_output : ut_lister.pcap_rt_input : ut_lister.pcap_rt_input;
    pcap_file_handle_t pcap_handle = pcap_runtime.pcap_handle;
    uint32_t size = ftell(pcap_handle->file);
    ESP_LOGI("DEBUG", "%s: %p, %u, %u ", pcap_runtime.stream_name, payload, length, size);
    struct timeval tv_now;
    gettimeofday(&tv_now, NULL);
    esp_err_t err = pcap_capture_packet(pcap_handle, payload, length, tv_now.tv_sec, tv_now.tv_usec);
    ESP_LOG_BUFFER_HEX_LEVEL(pcap_runtime.stream_name, (void*)payload, length, ESP_LOG_INFO);
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
