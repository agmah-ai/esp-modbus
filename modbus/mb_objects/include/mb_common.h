/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <stdint.h>

#include "mb_config.h"
#include "mb_frame.h"
#include "mb_types.h"
#include "port_common.h"
#include "mb_port_types.h"
#include "mb_callbacks.h"

#include "esp_log.h"

/* Common definitions */

#ifdef __cplusplus
extern "C" {
#endif

#if __has_include("esp_check.h")
#include "esp_check.h"

#define MB_RETURN_ON_FALSE(a, err_code, tag, format, ...) ESP_RETURN_ON_FALSE(a, err_code, tag, format __VA_OPT__(,) __VA_ARGS__)
#define MB_GOTO_ON_ERROR(x, goto_tag, log_tag, format, ...) ESP_GOTO_ON_ERROR(x, goto_tag, log_tag, format __VA_OPT__(,) __VA_ARGS__)
#define MB_GOTO_ON_FALSE(a, err_code, goto_tag, log_tag, format, ...) ESP_GOTO_ON_FALSE(a, err_code, goto_tag, log_tag, format __VA_OPT__(,) __VA_ARGS__)

#else

// if cannot include esp_check then use custom check macro

#define MB_RETURN_ON_FALSE(a, err_code, tag, format, ...) do {                                         \
        if (!(a)) {                                                                                    \
            ESP_LOGE(tag, "%s(%d): " format, __FUNCTION__, __LINE__ __VA_OPT__(,) __VA_ARGS__);        \
            return err_code;                                                                           \
        }                                                                                              \
} while(0)

#define MB_GOTO_ON_ERROR(x, goto_tag, log_tag, format, ...) do {                                           \
        esp_err_t err_rc_ = (x);                                                                           \
        if (err_rc_ != ESP_OK) {                                                                           \
            ESP_LOGE(log_tag, "%s(%d): " format, __FUNCTION__, __LINE__ __VA_OPT__(,) __VA_ARGS__);        \
            ret = err_rc_;                                                                                 \
            goto goto_tag;                                                                                 \
        }                                                                                                  \
    } while(0)

#define MB_GOTO_ON_FALSE(a, err_code, goto_tag, log_tag, format, ...) do {                                  \
        (void)log_tag;                                                                                      \
        if (!(a)) {                                                                                         \
            ESP_LOGE(log_tag, "%s(%d): " format, __FUNCTION__, __LINE__ __VA_OPT__(,) __VA_ARGS__);         \
            ret = (err_code);                                                                                 \
            goto goto_tag;                                                                                  \
        }                                                                                                   \
    } while (0) 

#endif

#define MB_BASE2PORT(inst) (__extension__( \
{ \
    assert(inst); \
    (((mb_base_t *)inst)->port_obj); \
} \
))

#define MB_IS_WITHIN_TRANSACTION(trans, time) (__extension__( \
{ \
    mb_transaction_t tr = (trans); \
    ((time && (time < tr.expected_ts) \
                        && (time > tr.start_ts)) ? true : false); \
} \
))

// Todo: better to create translation function to propogate the errors
#define MB_ERR_TO_IDF(mb_error) (__extension__( \
{                                               \
    esp_err_t error = ESP_OK;                   \
    switch(mb_error) {                          \
        case MB_ENOERR:                         \
            error = ESP_OK;                     \
            break;                              \
        case MB_ENOREG:                         \
            error = ESP_ERR_NOT_FOUND;      \
            break;                              \
        case MB_ETIMEDOUT:                      \
            error = ESP_ERR_TIMEOUT;            \
            break;                              \
        case MB_EINVAL:                         \
            error = ESP_ERR_INVALID_ARG;        \
            break;                              \
        case MB_ENORES:                         \
            error = ESP_ERR_NO_MEM;             \
            break;                              \
        case MB_EILLFUNC:                       \
            error = ESP_ERR_INVALID_RESPONSE;   \
            break;                              \
        case MB_EIO:                            \
        case MB_EPORTERR:                       \
            error = ESP_ERR_INVALID_STATE;      \
            break;                              \
        case MB_EBUSY:                          \
            error = ESP_ERR_NOT_FINISHED;       \
            break;                              \
        default:                                \
            error = ESP_FAIL;                   \
            break;                              \
    }                                           \
    error;                                      \
}                                               \
))

#define MB_ERR_FROM_IDF(idf_error) (__extension__( \
{                                               \
    mb_err_enum_t error = MB_ENOERR;            \
    switch(idf_error)                           \
{                                           \
        case ESP_OK:                            \
            error = MB_ENOERR;                  \
            break;                              \
        case ESP_ERR_NOT_FOUND:                 \
            error = MB_ENOREG;                  \
            break;                              \
        case ESP_ERR_TIMEOUT:                   \
            error = MB_ETIMEDOUT;               \
            break;                              \
        case ESP_ERR_INVALID_ARG:               \
            error = MB_EINVAL;                  \
            break;                              \
        case ESP_ERR_NO_MEM:                    \
            error = MB_ENORES;                  \
            break;                              \
        case ESP_ERR_INVALID_RESPONSE:          \
            error = MB_EILLFUNC;                \
            break;                              \
        case ESP_ERR_INVALID_STATE:             \
            error = MB_EIO;                     \
            break;                              \
        case ESP_ERR_NOT_SUPPORTED:             \
            error = MB_EPORTERR;                \
            break;                              \
        case MB_EBUSY:                          \
            error = ESP_ERR_NOT_FINISHED;       \
            break;                              \
        default:                                \
            error = ESP_FAIL;                   \
            break;                              \
    }                                           \
    error;                                      \
}                                               \
))

typedef struct {
    uint64_t start_ts;
    uint64_t expected_ts;
    uint64_t read_ts;
    uint64_t end_ts;
    mb_err_event_t err_type;
    bool is_active;
    bool timed_out;
} mb_transaction_t;

typedef struct mb_base_t mb_base_t;             /*!< Type of modbus object */
typedef struct mb_trans_base_t mb_trans_base_t;
typedef struct mb_port_base_t mb_port_base_t;

typedef mb_err_enum_t (*mb_delete_fp)(mb_base_t *inst);
typedef mb_err_enum_t (*mb_enable_fp)(mb_base_t *inst);
typedef mb_err_enum_t (*mb_disable_fp)(mb_base_t *inst);
typedef mb_err_enum_t (*mb_poll_fp)(mb_base_t *inst);
typedef void (*mb_set_addr_fp)(mb_base_t *inst, uint8_t dest_addr);
typedef uint8_t (*mb_get_addr_fp)(mb_base_t *inst);
typedef void (*mb_set_send_len_fp)(mb_base_t *inst, uint16_t len);
typedef uint16_t (*mb_get_send_len_fp)(mb_base_t *inst);
typedef void (*mb_get_send_buf_fp)(mb_base_t *inst, uint8_t **pbuf);

typedef enum
{
    STATE_ENABLED,
    STATE_DISABLED,
    STATE_NOT_INITIALIZED
} mb_state_enum_t;

struct mb_base_t
{
    void *parent;
    _lock_t lock;                   // base object lock
    mb_trans_base_t *transp_obj;
    mb_port_base_t  *port_obj;
    char *obj_name;

    uint8_t slave_id[MB_FUNC_OTHER_REP_SLAVEID_BUF];
    uint16_t slave_id_len;

    mb_delete_fp delete;
    mb_enable_fp enable;
    mb_disable_fp disable;
    mb_poll_fp poll;
    mb_set_addr_fp set_dest_addr;
    mb_get_addr_fp get_dest_addr;
    mb_set_send_len_fp set_send_len;
    mb_get_send_len_fp get_send_len;
    mb_get_send_buf_fp get_send_buf;

    mb_rw_callbacks_t rw_cbs;
};

typedef struct _port_serial_opts mb_serial_opts_t;
typedef struct _port_tcp_opts mb_tcp_opts_t;

mb_err_enum_t mbs_rtu_create(mb_serial_opts_t *ser_opts, mb_base_t **out_obj);
mb_err_enum_t mbs_ascii_create(mb_serial_opts_t *ser_opts, mb_base_t **out_obj);
mb_err_enum_t mbs_tcp_create(mb_tcp_opts_t *tcp_opts, mb_base_t **out_obj);

mb_err_enum_t mbs_delete(mb_base_t *inst);
mb_err_enum_t mbs_enable(mb_base_t *inst);
mb_err_enum_t mbs_disable(mb_base_t *inst);
mb_err_enum_t mbs_poll(mb_base_t *inst);
mb_err_enum_t mbs_set_slv_id(mb_base_t *inst, uint8_t slv_id, bool is_running, uint8_t const *slv_idstr, uint16_t slv_idstr_len);

mb_err_enum_t mbm_rtu_create(mb_serial_opts_t *ser_opts, mb_base_t **out_obj);
mb_err_enum_t mbm_ascii_create(mb_serial_opts_t *ser_opts, mb_base_t **out_obj);
mb_err_enum_t mbm_tcp_create(mb_tcp_opts_t *tcp_opts, mb_base_t **out_obj);

mb_err_enum_t mbm_delete(mb_base_t *inst);
mb_err_enum_t mbm_enable(mb_base_t *inst);
mb_err_enum_t mbm_disable(mb_base_t *inst);
mb_err_enum_t mbm_poll(mb_base_t *inst);

#ifdef __cplusplus
}
#endif