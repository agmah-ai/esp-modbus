/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mb_config.h"
#include "mb_common.h"
#include "mb_proto.h"
#include "mb_func.h"
#include "mb_master.h"
#include "transport_common.h"
#include "port_common.h"
#include "ascii_transport.h"
#include "rtu_transport.h"
#include "tcp_transport.h"

static const char *TAG = "mb_object.master";

#if MB_MASTER_ASCII_ENABLED || MB_MASTER_RTU_ENABLED || MB_MASTER_TCP_ENABLED

static const mb_fn_handler_t master_handlers[MB_FUNC_HANDLERS_MAX] =
    {
#if MB_FUNC_OTHER_REP_SLAVEID_ENABLED
        // TODO Add Master function define
        {MB_FUNC_OTHER_REPORT_SLAVEID, (void *)mb_fn_report_slv_id},
#endif
#if MB_FUNC_READ_INPUT_ENABLED
        {MB_FUNC_READ_INPUT_REGISTER, (void *)mbm_fn_read_inp_reg},
#endif
#if MB_FUNC_READ_HOLDING_ENABLED
        {MB_FUNC_READ_HOLDING_REGISTER, (void *)mbm_fn_read_holding_reg},
#endif
#if MB_FUNC_WRITE_MULTIPLE_HOLDING_ENABLED
        {MB_FUNC_WRITE_MULTIPLE_REGISTERS, (void *)mbm_fn_write_multi_holding_reg},
#endif
#if MB_FUNC_WRITE_HOLDING_ENABLED
        {MB_FUNC_WRITE_REGISTER, (void *)mbm_fn_write_holding_reg},
#endif
#if MB_FUNC_READWRITE_HOLDING_ENABLED
        {MB_FUNC_READWRITE_MULTIPLE_REGISTERS, (void *)mbm_fn_rw_multi_holding_regs},
#endif
#if MB_FUNC_READ_COILS_ENABLED
        {MB_FUNC_READ_COILS, (void *)mbm_fn_read_coils},
#endif
#if MB_FUNC_WRITE_COIL_ENABLED
        {MB_FUNC_WRITE_SINGLE_COIL, (void *)mbm_fn_write_coil},
#endif
#if MB_FUNC_WRITE_MULTIPLE_COILS_ENABLED
        {MB_FUNC_WRITE_MULTIPLE_COILS, (void *)mbm_fn_write_multi_coils},
#endif
#if MB_FUNC_READ_DISCRETE_INPUTS_ENABLED
        {MB_FUNC_READ_DISCRETE_INPUTS, (void *)mbm_fn_read_discrete_inputs},
#endif
};

typedef struct
{
    mb_base_t base;
    mb_comm_mode_t cur_mode;
    mb_state_enum_t cur_state;
    const mb_fn_handler_t *func_handlers;
    uint8_t *rcv_frame;
    uint8_t *snd_frame;
    uint16_t pdu_snd_len;
    uint8_t rcv_addr;
    uint16_t pdu_rcv_len;
    uint8_t func_code;
    mb_exception_t exception;
    uint8_t master_dst_addr;
    mb_transaction_t transaction;
} mbm_object_t;

typedef struct _port_serial_opts mb_serial_opts_t;

mb_err_enum_t mbm_rtu_create(mb_serial_opts_t *ser_opts, mb_base_t **out_obj);
mb_err_enum_t mbm_ascii_create(mb_serial_opts_t *ser_opts, mb_base_t **out_obj);
mb_err_enum_t mbm_tcp_create(mb_tcp_opts_t *tcp_opts, mb_base_t **out_obj);

mb_err_enum_t mbm_delete(mb_base_t *inst);
mb_err_enum_t mbm_enable(mb_base_t *inst);
mb_err_enum_t mbm_disable(mb_base_t *inst);
mb_err_enum_t mbm_poll(mb_base_t *inst);

static void mbm_set_pdu_send_length(mb_base_t *inst, uint16_t length);
static uint16_t mbm_get_pdu_send_length(mb_base_t *inst);
static void mbm_set_dest_addr(mb_base_t *inst, uint8_t dest_addr);
static uint8_t mbm_get_dest_addr(mb_base_t *inst);
static void mbm_get_pdu_send_buf(mb_base_t *inst, uint8_t **pbuf);

mb_err_enum_t mbm_rtu_create(mb_serial_opts_t *ser_opts, mb_base_t **out_obj)
{
    MB_RETURN_ON_FALSE(ser_opts, MB_EINVAL, TAG, "invalid options for the instance.");
    MB_RETURN_ON_FALSE((ser_opts->mode == MB_RTU), MB_EILLSTATE, TAG, "incorrect mode != RTU.");
    mb_err_enum_t ret = MB_ENOERR;
    mbm_object_t *mbm_obj = NULL;
    mbm_obj = (mbm_object_t *)calloc(1, sizeof(mbm_object_t));
    MB_GOTO_ON_FALSE((mbm_obj), MB_EILLSTATE, error, TAG, "no mem for mb master instance.");

    CRITICAL_SECTION_INIT(mbm_obj->base.lock);
    mbm_obj->cur_state = STATE_NOT_INITIALIZED;
    mbm_obj->base.delete = mbm_delete;
    mbm_obj->base.enable = mbm_enable;
    mbm_obj->base.disable = mbm_disable;
    mbm_obj->base.poll = mbm_poll;
    mbm_obj->base.set_dest_addr = mbm_set_dest_addr;
    mbm_obj->base.get_dest_addr = mbm_get_dest_addr;
    mbm_obj->base.set_send_len = mbm_set_pdu_send_length;
    mbm_obj->base.get_send_len = mbm_get_pdu_send_length;
    mbm_obj->base.get_send_buf = mbm_get_pdu_send_buf;
    mb_trans_base_t *transp_obj = NULL;
    ret = mbm_rtu_transp_create(ser_opts, &transp_obj);
    MB_GOTO_ON_FALSE((transp_obj && (ret == MB_ENOERR)), MB_EILLSTATE, error, 
                        TAG, "transport creation, err: %d", ret);
    mbm_obj->func_handlers = master_handlers;
    mbm_obj->cur_mode = ser_opts->mode;
    mbm_obj->cur_state = STATE_DISABLED;
    transp_obj->get_tx_frm(transp_obj, (uint8_t **)&mbm_obj->snd_frame);
    transp_obj->get_rx_frm(transp_obj, (uint8_t **)&mbm_obj->rcv_frame);
    mbm_obj->base.port_obj = transp_obj->port_obj;
    mbm_obj->base.transp_obj = transp_obj;
    if (asprintf(&mbm_obj->base.obj_name, "mbm_rtu#%p", mbm_obj) == -1) {
        abort();
    }
    *out_obj = &(mbm_obj->base);
    ESP_LOGD(TAG, "created %s object @%p", TAG, mbm_obj);
    return MB_ENOERR;

error:
    if (transp_obj) {
        mbm_rtu_transp_delete(transp_obj);
    }
    free(mbm_obj->base.obj_name);
    CRITICAL_SECTION_CLOSE(mbm_obj->base.lock);
    free(mbm_obj);
    return ret;
}

mb_err_enum_t mbm_ascii_create(mb_serial_opts_t *ser_opts, mb_base_t **out_obj)
{
    MB_RETURN_ON_FALSE(ser_opts, MB_EINVAL, TAG, "invalid options for the instance.");
    MB_RETURN_ON_FALSE((ser_opts->mode == MB_ASCII), MB_EILLSTATE, TAG, "incorrect option mode != ASCII.");
    mb_err_enum_t ret = MB_ENOERR;
    mbm_object_t *mbm_obj = NULL;
    mbm_obj = (mbm_object_t *)calloc(1, sizeof(mbm_object_t));
    MB_GOTO_ON_FALSE((mbm_obj), MB_EILLSTATE, error, TAG, "no mem for mb master instance.");
    CRITICAL_SECTION_INIT(mbm_obj->base.lock);
    mbm_obj->cur_state = STATE_NOT_INITIALIZED;
    mbm_obj->base.delete = mbm_delete;
    mbm_obj->base.enable = mbm_enable;
    mbm_obj->base.disable = mbm_disable;
    mbm_obj->base.poll = mbm_poll;
    mbm_obj->base.set_dest_addr = mbm_set_dest_addr;
    mbm_obj->base.get_dest_addr = mbm_get_dest_addr;
    mbm_obj->base.set_send_len = mbm_set_pdu_send_length;
    mbm_obj->base.get_send_len = mbm_get_pdu_send_length;
    mbm_obj->base.get_send_buf = mbm_get_pdu_send_buf;
    mb_trans_base_t *transp_obj = NULL;
    ret = mbm_ascii_transp_create(ser_opts, &transp_obj);
    MB_GOTO_ON_FALSE((transp_obj && (ret == MB_ENOERR)), MB_EILLSTATE, error,
                     TAG, "transport creation, err: %d", ret);
    mbm_obj->func_handlers = master_handlers;
    mbm_obj->cur_mode = ser_opts->mode;
    mbm_obj->cur_state = STATE_DISABLED;
    transp_obj->get_tx_frm(transp_obj, (uint8_t **)&mbm_obj->snd_frame);
    transp_obj->get_rx_frm(transp_obj, (uint8_t **)&mbm_obj->rcv_frame);
    mbm_obj->base.port_obj = transp_obj->port_obj; // binding of the modbus object with port objet
    mbm_obj->base.transp_obj = transp_obj;
    *out_obj = &(mbm_obj->base);
    if (asprintf(&mbm_obj->base.obj_name, "mbm_ascii#%p", mbm_obj) == -1)
    {
        abort();
    }
    ESP_LOGD(TAG, "created object @%p", mbm_obj);
    return MB_ENOERR;
error:
    if (transp_obj)
    {
        mbm_ascii_transp_delete(transp_obj);
    }
    free(mbm_obj->base.obj_name);
    CRITICAL_SECTION_CLOSE(mbm_obj->base.lock);
    free(mbm_obj);
    return ret;
}

mb_err_enum_t mbm_tcp_create(mb_tcp_opts_t *tcp_opts, mb_base_t **out_obj)
{
    MB_RETURN_ON_FALSE(tcp_opts, MB_EINVAL, TAG, "invalid options for the instance.");
    MB_RETURN_ON_FALSE((tcp_opts->mode == MB_TCP), MB_EILLSTATE, TAG, "incorrect option mode != TCP.");
    mb_err_enum_t ret = MB_ENOERR;
    mbm_object_t *mbm_obj = NULL;
    mbm_obj = (mbm_object_t *)calloc(1, sizeof(mbm_object_t));
    MB_RETURN_ON_FALSE(mbm_obj, MB_EILLSTATE, TAG, "no mem for mb master instance.");
    CRITICAL_SECTION_INIT(mbm_obj->base.lock);
    mbm_obj->cur_state = STATE_NOT_INITIALIZED;
    mbm_obj->base.delete = mbm_delete;
    mbm_obj->base.enable = mbm_enable;
    mbm_obj->base.disable = mbm_disable;
    mbm_obj->base.poll = mbm_poll;
    mbm_obj->base.set_dest_addr = mbm_set_dest_addr;
    mbm_obj->base.get_dest_addr = mbm_get_dest_addr;
    mbm_obj->base.set_send_len = mbm_set_pdu_send_length;
    mbm_obj->base.get_send_len = mbm_get_pdu_send_length;
    mbm_obj->base.get_send_buf = mbm_get_pdu_send_buf;
    mb_trans_base_t *transp_obj = NULL;
    ret = mbm_tcp_transp_create(tcp_opts, &transp_obj);
    MB_GOTO_ON_FALSE((transp_obj && (ret == MB_ENOERR)), MB_EILLSTATE, error,
                     TAG, "transport creation, err: %d", ret);
    mbm_obj->func_handlers = master_handlers;
    mbm_obj->cur_mode = tcp_opts->mode;
    mbm_obj->cur_state = STATE_DISABLED;
    transp_obj->get_tx_frm(transp_obj, (uint8_t **)&mbm_obj->snd_frame);
    transp_obj->get_rx_frm(transp_obj, (uint8_t **)&mbm_obj->rcv_frame);
    mbm_obj->base.port_obj = transp_obj->port_obj; // binding of the modbus object with port objet
    mbm_obj->base.transp_obj = transp_obj;
    *out_obj = &(mbm_obj->base);
    if (asprintf(&mbm_obj->base.obj_name, "mbm_tcp#%p", mbm_obj) == -1)
    {
        abort();
    }
    ESP_LOGD(TAG, "created object @%p", mbm_obj);
    return MB_ENOERR;
error:
    if (transp_obj)
    {
        mbm_tcp_transp_delete(transp_obj);
    }
    free(mbm_obj->base.obj_name);
    CRITICAL_SECTION_CLOSE(mbm_obj->base.lock);
    free(mbm_obj);
    return ret;
}

mb_err_enum_t mbm_delete(mb_base_t *inst)
{
    mbm_object_t *mbm_obj = __containerof(inst, mbm_object_t, base);
    mb_err_enum_t status = MB_ENOERR;
    CRITICAL_SECTION(inst->lock)
    {
        if (mbm_obj->cur_state == STATE_DISABLED)
        {
            if (mbm_obj->base.transp_obj->frm_delete)
            {
                // call destructor of the transport object
                mbm_obj->base.transp_obj->frm_delete(inst->transp_obj);
            }
            // delete the modbus instance
            free(mbm_obj->base.obj_name);
            free(inst);
            status = MB_ENOERR;
        }
        else
        {
            ESP_LOGD(TAG, "disable the instance %p first.", mbm_obj);
            status = MB_EILLSTATE;
        }
    }
    CRITICAL_SECTION_CLOSE(inst->lock);
    return status;
}

mb_err_enum_t mbm_enable(mb_base_t *inst)
{
    mbm_object_t *mbm_obj = __containerof(inst, mbm_object_t, base);
    mb_err_enum_t status = MB_ENOERR;
    CRITICAL_SECTION(inst->lock)
    {
        if (mbm_obj->cur_state == STATE_DISABLED)
        {
            /* Activate the protocol stack. */
            mbm_obj->base.transp_obj->frm_start(mbm_obj->base.transp_obj);
            mbm_obj->cur_state = STATE_ENABLED;
            status = MB_ENOERR;
        }
        else
        {
            status = MB_EILLSTATE;
        }
    }
    return status;
}

mb_err_enum_t mbm_disable(mb_base_t *inst)
{
    mb_err_enum_t status = MB_ENOERR;
    mbm_object_t *mbm_obj = __containerof(inst, mbm_object_t, base);
    CRITICAL_SECTION(inst->lock)
    {
        if (mbm_obj->cur_state == STATE_ENABLED)
        {
            mbm_obj->base.transp_obj->frm_stop(mbm_obj->base.transp_obj);
            mbm_obj->cur_state = STATE_DISABLED;
            status = MB_ENOERR;
        }
        else if (mbm_obj->cur_state == STATE_DISABLED)
        {
            status = MB_ENOERR;
        }
        else
        {
            status = MB_EILLSTATE;
        }
    }
    return status;
}

static void mbm_get_pdu_send_buf(mb_base_t *inst, uint8_t **pbuf)
{
    mbm_object_t *mbm_obj = __containerof(inst, mbm_object_t, base);
    mbm_obj->base.transp_obj->get_tx_frm(mbm_obj->base.transp_obj, pbuf);
}

static void mbm_get_pdu_recv_buf(mb_base_t *inst, uint8_t **pbuf)
{
    mbm_object_t *mbm_obj = __containerof(inst, mbm_object_t, base);
    mbm_obj->base.transp_obj->get_rx_frm(mbm_obj->base.transp_obj, pbuf);
}

static void mbm_set_pdu_send_length(mb_base_t *inst, uint16_t length)
{
    mbm_object_t *mbm_obj = __containerof(inst, mbm_object_t, base);
    CRITICAL_SECTION(inst->lock)
    {
        mbm_obj->pdu_snd_len = length;
    }
}

static uint16_t mbm_get_pdu_send_length(mb_base_t *inst)
{
    mbm_object_t *mbm_obj = __containerof(inst, mbm_object_t, base);
    return mbm_obj->pdu_snd_len;
}

static void mbm_set_dest_addr(mb_base_t *inst, uint8_t dest_addr)
{
    mbm_object_t *mbm_obj = __containerof(inst, mbm_object_t, base);
    CRITICAL_SECTION(inst->lock)
    {
        mbm_obj->master_dst_addr = dest_addr;
    }
}

static uint8_t mbm_get_dest_addr(mb_base_t *inst)
{
    mbm_object_t *mbm_obj = __containerof(inst, mbm_object_t, base);
    return mbm_obj->master_dst_addr;
}

void mbm_error_cb_respond_timeout(mb_base_t *inst, uint8_t dest_addr, const uint8_t *pdu_data, uint16_t pdu_length)
{
    mb_port_evt_set_resp_flag(MB_BASE2PORT(inst), EV_MASTER_ERROR_RESPOND_TIMEOUT);
    ESP_LOG_BUFFER_HEX_LEVEL(__func__, (void *)pdu_data, pdu_length, ESP_LOG_DEBUG);
}

void mbm_error_cb_receive_data(mb_base_t *inst, uint8_t dest_addr, const uint8_t *pdu_data, uint16_t pdu_length)
{
    mb_port_evt_set_resp_flag(MB_BASE2PORT(inst), EV_MASTER_ERROR_RECEIVE_DATA);
    ESP_LOG_BUFFER_HEX_LEVEL(__func__, (void *)pdu_data, pdu_length, ESP_LOG_DEBUG);
}

void mbm_error_cb_execute_function(mb_base_t *inst, uint8_t dest_address, const uint8_t *pdu_data, uint16_t pdu_length)
{
    mb_port_evt_set_resp_flag(MB_BASE2PORT(inst), EV_MASTER_ERROR_EXECUTE_FUNCTION);
    ESP_LOG_BUFFER_HEX_LEVEL(__func__, (void *)pdu_data, pdu_length, ESP_LOG_DEBUG);
}

void mbm_error_cb_request_success(mb_base_t *inst, uint8_t dest_address, const uint8_t *pdu_data, uint16_t pdu_length)
{
    mb_port_evt_set_resp_flag(MB_BASE2PORT(inst), EV_MASTER_PROCESS_SUCCESS);
    ESP_LOG_BUFFER_HEX_LEVEL(__func__, (void *)pdu_data, pdu_length, ESP_LOG_DEBUG);
}

mb_err_enum_t mbm_poll(mb_base_t *inst)
{
    mbm_object_t *mbm_obj = __containerof(inst, mbm_object_t, base);

    uint16_t length;
    mb_exception_t exception;
    mb_err_enum_t status = MB_ENOERR;
    mb_event_t event;
    mb_err_event_t error_type;

    /* Check if the protocol stack is ready. */
    if (mbm_obj->cur_state != STATE_ENABLED)
    {
        return MB_EILLSTATE;
    }

    /* Check if there is a event available. If not return control to caller.
     * Otherwise we will handle the event. */
    if (mb_port_evt_get(mbm_obj->base.port_obj, &event))
    {
        switch (event.code)
        {
            // In some cases it is possible that more than one event set
            // together (even from one subset mask) than process them consistently
            case EV_READY:
                ESP_LOGW(TAG, "%p:EV_READY", inst);
                mb_port_evt_res_release(inst->port_obj);
                break;

            case EV_FRAME_TRANSMIT:
                mbm_get_pdu_send_buf(inst, &mbm_obj->snd_frame);
                ESP_LOG_BUFFER_HEX_LEVEL("POLL transmit buffer", (void *)mbm_obj->snd_frame, mbm_obj->pdu_snd_len, ESP_LOG_WARN);
                status = inst->transp_obj->frm_send(inst->transp_obj, mbm_obj->master_dst_addr, mbm_obj->snd_frame, mbm_obj->pdu_snd_len);
                if (status != MB_ENOERR) {
                    mb_port_evt_set_err_type(inst->port_obj, EV_ERROR_RECEIVE_DATA);
                    (void)mb_port_evt_post(inst->port_obj, EVENT(EV_ERROR_PROCESS));
                    ESP_LOGE(TAG, "%p: frame send error. %d", inst, status);
                }
                // Initialize modbus transaction
                mbm_obj->transaction.is_active = true;
                mbm_obj->transaction.err_type = EV_ERROR_INIT;
                mbm_obj->transaction.start_ts = event.post_ts;
                mbm_obj->transaction.read_ts = 0;
                uint32_t resp_time = mb_port_tmr_get_response_time_ms(inst->port_obj);
                mbm_obj->transaction.expected_ts = mbm_obj->transaction.start_ts + (resp_time * 1000);
                break;

            case EV_FRAME_SENT:
                ESP_LOGD(TAG, "%p:EV_MASTER_FRAME_SENT", inst);
                break;

            case EV_FRAME_RECEIVED:
                if (MB_IS_WITHIN_TRANSACTION(mbm_obj->transaction, event.post_ts)) {
                    mbm_obj->pdu_rcv_len = event.length;
                    status = inst->transp_obj->frm_rcv(inst->transp_obj, &mbm_obj->rcv_addr, &mbm_obj->rcv_frame, &mbm_obj->pdu_rcv_len);
                    MB_RETURN_ON_FALSE(mbm_obj->snd_frame, MB_EILLSTATE, TAG, "Send buffer initialization fail.");
                    // Check if the frame is for us. If not ,send an error process event.
                    if ((status == MB_ENOERR) && ((mbm_obj->rcv_addr == mbm_obj->master_dst_addr) 
                                    || (mbm_obj->rcv_addr == MB_TCP_PSEUDO_ADDRESS))) {
                        if ((mbm_obj->rcv_frame[MB_PDU_FUNC_OFF] & ~MB_FUNC_ERROR) == (mbm_obj->snd_frame[MB_PDU_FUNC_OFF])) {
                            ESP_LOGD(TAG, "%p, frame data received successfully, (%u).", inst, status);
                            ESP_LOG_BUFFER_HEX_LEVEL("POLL receive buffer", (void *)mbm_obj->rcv_frame, (uint16_t)mbm_obj->pdu_rcv_len, ESP_LOG_WARN);
                            mbm_obj->transaction.read_ts = event.post_ts;
                            (void)mb_port_evt_post(inst->port_obj, EVENT(EV_EXECUTE));
                        } else {
                            ESP_LOGE(TAG, "%p, drop incorrect frame, receive_func(%u) != send_func(%u)",
                                    inst, mbm_obj->rcv_frame[MB_PDU_FUNC_OFF], mbm_obj->snd_frame[MB_PDU_FUNC_OFF]);
                            mb_port_evt_set_err_type(inst->port_obj, EV_ERROR_RECEIVE_DATA);
                            (void)mb_port_evt_post(inst->port_obj, EVENT(EV_ERROR_PROCESS));
                        }
                    } else {
                        mb_port_evt_set_err_type(inst->port_obj, EV_ERROR_RECEIVE_DATA);
                        (void)mb_port_evt_post(inst->port_obj, EVENT(EV_ERROR_PROCESS));
                        ESP_LOGD(TAG, "%p: packet data receive failed (addr=%u)(%u).",
                                inst, mbm_obj->rcv_addr, status);
                    }
                } else {
                    // Ignore the `EV_FRAME_RECEIVED` event because the respond timeout occurred
                    // and this is likely respond to previous transaction
                    ESP_LOGE(TAG, "%p, drop data received outside of transaction.", inst);
                    mb_port_evt_set_err_type(inst->port_obj, EV_ERROR_RESPOND_TIMEOUT);
                    (void)mb_port_evt_post(inst->port_obj, EVENT(EV_ERROR_PROCESS));
                }
                break;

            case EV_EXECUTE:
                if (MB_IS_WITHIN_TRANSACTION(mbm_obj->transaction, mbm_obj->transaction.read_ts)) {
                    MB_RETURN_ON_FALSE(mbm_obj->rcv_frame, MB_EILLSTATE, TAG, "%p, receive buffer initialization fail.", inst);
                    ESP_LOGD(TAG, "%p:EV_EXECUTE", inst);
                    mbm_obj->func_code = mbm_obj->rcv_frame[MB_PDU_FUNC_OFF];
                    exception = MB_EX_ILLEGAL_FUNCTION;
                    /* If receive frame has exception. The receive function code highest bit is 1.*/
                    if (mbm_obj->func_code & MB_FUNC_ERROR) {
                        exception = (mb_exception_t)mbm_obj->rcv_frame[MB_PDU_DATA_OFF];
                    } else {
                        for (int i = 0; i < MB_FUNC_HANDLERS_MAX; i++) {
                            /* No more function handlers registered. Abort. */
                            if (mbm_obj->func_handlers[i].func_code == 0) {
                                break;
                            }
                            if (mbm_obj->func_handlers[i].func_code == mbm_obj->func_code) {
                                /* If master request is broadcast,
                                * the master need execute function for all slave.
                                */
                                if (inst->transp_obj->frm_is_bcast(inst->transp_obj)) {
                                    length = mbm_obj->pdu_snd_len;
                                    for (int j = 1; j <= MB_MASTER_TOTAL_SLAVE_NUM; j++) {
                                        mbm_set_dest_addr(inst, j);
                                        exception = mbm_obj->func_handlers[i].handler(inst, mbm_obj->rcv_frame, &length);
                                    }
                                } else {
                                    exception = mbm_obj->func_handlers[i].handler(inst, mbm_obj->rcv_frame, &mbm_obj->pdu_rcv_len);
                                }
                                break;
                            }
                        }
                    }
                    /* If master has exception, will send error process event. Otherwise the master is idle.*/
                    if (exception != MB_EX_NONE) {
                        mb_port_evt_set_err_type(inst->port_obj, EV_ERROR_EXECUTE_FUNCTION);
                        (void)mb_port_evt_post(inst->port_obj, EVENT(EV_ERROR_PROCESS));
                    } else {
                        error_type = mb_port_evt_get_err_type(inst->port_obj);
                        if (error_type == EV_ERROR_INIT) {
                            mb_port_evt_set_err_type(inst->port_obj, EV_ERROR_OK);
                            ESP_LOGD(TAG, "%p: set event EV_ERROR_OK", inst);
                            (void)mb_port_evt_post(inst->port_obj, EVENT(EV_ERROR_PROCESS));
                        }
                    }
                } else {
                    mb_port_evt_set_err_type(inst->port_obj, EV_ERROR_EXECUTE_FUNCTION);
                    (void)mb_port_evt_post(inst->port_obj, EVENT(EV_ERROR_PROCESS));
                    ESP_LOGE(TAG, "%p: execution is expired.", inst);
                }
                break;

            case EV_ERROR_PROCESS:
                ESP_LOGD(TAG, "%p:EV_ERROR_PROCESS", inst);
                // stop timer and execute specified error process callback function.
                mb_port_tmr_disable(inst->port_obj);
                error_type = mb_port_evt_get_err_type(inst->port_obj);
                mbm_get_pdu_send_buf(inst, &mbm_obj->snd_frame);
                switch (error_type)
                {
                    case EV_ERROR_RESPOND_TIMEOUT:
                        mbm_error_cb_respond_timeout(inst, mbm_obj->master_dst_addr,
                                                    mbm_obj->snd_frame, mbm_obj->pdu_snd_len);
                        break;
                    case EV_ERROR_RECEIVE_DATA:
                        mbm_error_cb_receive_data(inst, mbm_obj->master_dst_addr,
                                                mbm_obj->snd_frame, mbm_obj->pdu_snd_len);
                        break;
                    case EV_ERROR_EXECUTE_FUNCTION:
                        mbm_error_cb_execute_function(inst, mbm_obj->master_dst_addr,
                                                    mbm_obj->snd_frame, mbm_obj->pdu_snd_len);
                        break;
                    case EV_ERROR_OK:
                        mbm_error_cb_request_success(inst, mbm_obj->master_dst_addr,
                                                    mbm_obj->snd_frame, mbm_obj->pdu_snd_len);
                        break;
                    default:
                        ESP_LOGE(TAG, "%p: incorrect error type = %d.", inst, error_type);
                        break;
                }
                mbm_obj->transaction.err_type = error_type;
                mb_port_evt_set_err_type(inst->port_obj, EV_ERROR_INIT);
                mbm_obj->transaction.is_active = false;
                mbm_obj->transaction.end_ts = event.post_ts;
                uint64_t time_div_us = mbm_obj->transaction.end_ts - mbm_obj->transaction.start_ts;
                ESP_LOGW(TAG, "%p, transaction processing time(us) = %" PRId64, inst, time_div_us);
                mb_port_evt_res_release(inst->port_obj);
                break;

            default:
                ESP_LOGE(TAG, "%p: Unexpected event triggered 0x%02x.", inst, (uint16_t)event.code);
                break;
        }
    } else {
        // Something went wrong and task unblocked but there are no any correct events set
        ESP_LOGE(TAG, "%p: unexpected event triggered 0x%02x.", inst, (uint16_t)event.code);
        status = MB_EILLSTATE;
    }
    return status;
}

#endif
