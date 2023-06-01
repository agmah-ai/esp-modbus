/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mb_common.h"
#include "mb_proto.h"
#include "mb_func.h"
#include "mb_slave.h"
#include "transport_common.h"
#include "port_common.h"
#include "ascii_transport.h"
#include "rtu_transport.h"
#include "tcp_transport.h"

static const char *TAG = "mb_object.slave";

static mb_fn_handler_t slave_handlers[MB_FUNC_HANDLERS_MAX] =
    {
#if MB_FUNC_OTHER_REP_SLAVEID_ENABLED
        {MB_FUNC_OTHER_REPORT_SLAVEID, (void *)mb_fn_report_slv_id},
#endif
#if MB_FUNC_READ_INPUT_ENABLED
        {MB_FUNC_READ_INPUT_REGISTER, (void *)mbs_fn_read_input_reg},
#endif
#if MB_FUNC_READ_HOLDING_ENABLED
        {MB_FUNC_READ_HOLDING_REGISTER, (void *)mbs_fn_read_holding_reg},
#endif
#if MB_FUNC_WRITE_MULTIPLE_HOLDING_ENABLED
        {MB_FUNC_WRITE_MULTIPLE_REGISTERS, (void *)mbs_fn_write_multi_holding_reg},
#endif
#if MB_FUNC_WRITE_HOLDING_ENABLED
        {MB_FUNC_WRITE_REGISTER, (void *)mbs_fn_write_holding_reg},
#endif
#if MB_FUNC_READWRITE_HOLDING_ENABLED
        {MB_FUNC_READWRITE_MULTIPLE_REGISTERS, (void *)mbs_fn_rw_multi_holding_reg},
#endif
#if MB_FUNC_READ_COILS_ENABLED
        {MB_FUNC_READ_COILS, (void *)mbs_fn_read_coils},
#endif
#if MB_FUNC_WRITE_COIL_ENABLED
        {MB_FUNC_WRITE_SINGLE_COIL, (void *)mbs_fn_write_coil},
#endif
#if MB_FUNC_WRITE_MULTIPLE_COILS_ENABLED
        {MB_FUNC_WRITE_MULTIPLE_COILS, (void *)mbs_fn_write_multi_coils},
#endif
#if MB_FUNC_READ_DISCRETE_INPUTS_ENABLED
        {MB_FUNC_READ_DISCRETE_INPUTS, (void *)mbs_fn_read_discrete_inp},
#endif
};

typedef struct
{
    mb_base_t base;
    // here are slave object properties and methods
    
    uint8_t mb_address;
    mb_comm_mode_t cur_mode;
    mb_state_enum_t cur_state;
    mb_fn_handler_t *func_handlers;
    uint8_t *frame;
    uint16_t length;
    uint8_t func_code;
    uint8_t rcv_addr;
    bool transaction_is_active;
    volatile uint16_t *pdu_snd_len;
    mb_transaction_t transaction;
} mbs_object_t;

typedef struct _port_serial_opts mb_serial_opts_t;

mb_err_enum_t mbs_rtu_create(mb_serial_opts_t *ser_opts, mb_base_t **out_obj);
mb_err_enum_t mbs_ascii_create(mb_serial_opts_t *ser_opts, mb_base_t **out_obj);
mb_err_enum_t mbs_tcp_create(mb_tcp_opts_t *tcp_opts, mb_base_t **out_obj);

mb_err_enum_t mbs_delete(mb_base_t *inst);
mb_err_enum_t mbs_enable(mb_base_t *inst);
mb_err_enum_t mbs_disable(mb_base_t *inst);
mb_err_enum_t mbs_poll(mb_base_t *inst);
mb_err_enum_t mbs_set_slv_id(mb_base_t *inst, uint8_t slv_id, bool is_running, uint8_t const *slv_idstr, uint16_t slv_idstr_len);

// todo: add registration of custom callback function

mb_err_enum_t mbs_rtu_create(mb_serial_opts_t *ser_opts, mb_base_t **out_obj)
{
    mb_err_enum_t ret = MB_ENOERR;
    MB_RETURN_ON_FALSE(ser_opts, MB_EINVAL, TAG, "invalid options for the instance.");
    MB_RETURN_ON_FALSE((ser_opts->mode == MB_RTU), MB_EILLSTATE, TAG, "incorrect mode != RTU.");
    mbs_object_t *mbs_obj = NULL;
    mbs_obj = (mbs_object_t*)calloc(1, sizeof(mbs_object_t));
    MB_GOTO_ON_FALSE((mbs_obj), MB_EILLSTATE, error, TAG, "no mem for mb slave instance.");
    CRITICAL_SECTION_INIT(mbs_obj->base.lock);
    mbs_obj->cur_state = STATE_NOT_INITIALIZED;
    mbs_obj->base.delete = mbs_delete;
    mbs_obj->base.enable = mbs_enable;
    mbs_obj->base.disable = mbs_disable;
    mbs_obj->base.poll = mbs_poll;
    mb_trans_base_t *transp_obj = NULL;
    ret = mbs_rtu_transp_create(ser_opts, &transp_obj);
    MB_GOTO_ON_FALSE((transp_obj && (ret == MB_ENOERR)), MB_EILLSTATE, error, 
                                TAG, "transport creation, err: %d", ret);
    mbs_obj->func_handlers = slave_handlers;
    mbs_obj->cur_mode = ser_opts->mode;
    mbs_obj->mb_address = ser_opts->slave_addr;
    mbs_obj->cur_state = STATE_DISABLED;
    transp_obj->get_tx_frm(transp_obj, (uint8_t **)&mbs_obj->frame);
    mbs_obj->base.port_obj = transp_obj->port_obj;
    mbs_obj->base.transp_obj = transp_obj;
    *out_obj = &(mbs_obj->base);
    if (asprintf(&mbs_obj->base.obj_name, "mbs_rtu#%p", mbs_obj) == -1) {
        abort();
    }
    ESP_LOGD(TAG, "created %s object @%p", TAG, mbs_obj);
    return MB_ENOERR;
    
error:
    if (transp_obj) {
        mbs_rtu_transp_delete(transp_obj);
    }
    free(mbs_obj->base.obj_name);
    CRITICAL_SECTION_CLOSE(mbs_obj->base.lock);
    free(mbs_obj);
    return ret;
}

mb_err_enum_t mbs_ascii_create(mb_serial_opts_t *ser_opts, mb_base_t **out_obj)
{
    mb_err_enum_t ret = MB_ENOERR;
    MB_RETURN_ON_FALSE(ser_opts, MB_EINVAL, TAG, "invalid options for %s instance.", TAG);
    MB_RETURN_ON_FALSE((ser_opts->mode == MB_ASCII), MB_EILLSTATE, TAG, "incorrect mode != ASCII.");
    mbs_object_t *mbs_obj = NULL;
    mbs_obj = (mbs_object_t*)calloc(1, sizeof(mbs_object_t));
    MB_GOTO_ON_FALSE((mbs_obj), MB_EILLSTATE, error, TAG, "no mem for mb slave instance.");
    CRITICAL_SECTION_INIT(mbs_obj->base.lock);
    mbs_obj->base.delete = mbs_delete;
    mbs_obj->base.enable = mbs_enable;
    mbs_obj->base.disable = mbs_disable;
    mbs_obj->base.poll = mbs_poll;
    mb_trans_base_t *transp_obj = NULL;
    ret = mbs_ascii_transp_create(ser_opts, &transp_obj);
    MB_GOTO_ON_FALSE((transp_obj && (ret == MB_ENOERR)), MB_EILLSTATE, error, 
                        TAG, "transport creation, err: %d", ret);
    mbs_obj->func_handlers = slave_handlers;
    mbs_obj->cur_mode = ser_opts->mode;
    mbs_obj->mb_address = ser_opts->slave_addr;
    mbs_obj->cur_state = STATE_DISABLED;
    transp_obj->get_tx_frm(transp_obj, (uint8_t **)&mbs_obj->frame);
    mbs_obj->base.port_obj = transp_obj->port_obj;
    mbs_obj->base.transp_obj = transp_obj;
    *out_obj = &(mbs_obj->base);
    if (asprintf(&mbs_obj->base.obj_name, "mbs_ascii#%p", mbs_obj) == -1) {
        abort();
    }
    ESP_LOGD(TAG, "created %s object @%p", TAG, mbs_obj);
    return MB_ENOERR;
error:
    if (transp_obj) {
        mbs_ascii_transp_delete(transp_obj);
    }
    free(mbs_obj->base.obj_name);
    CRITICAL_SECTION_CLOSE(mbs_obj->base.lock);
    free(mbs_obj);
    return ret;
}

mb_err_enum_t mbs_tcp_create(mb_tcp_opts_t *tcp_opts, mb_base_t **out_obj)
{
    return MB_EILLSTATE;
}

mb_err_enum_t mbs_delete(mb_base_t *inst)
{
    mbs_object_t *mbs_obj = __containerof(inst, mbs_object_t, base);
    mb_err_enum_t status = MB_ENOERR;
    CRITICAL_SECTION(inst->lock) {
        if (mbs_obj->cur_state == STATE_DISABLED) {
            if (mbs_obj->base.transp_obj->frm_delete) {
                // call destructor of the transport object
                mbs_obj->base.transp_obj->frm_delete(inst->transp_obj);
            }
            // delete the modbus instance
            free(mbs_obj->base.obj_name);
            free(inst);
            status = MB_ENOERR;
        } else {
            ESP_LOGD(TAG, " need to disable %" PRIx32 " object first.", (uint32_t)mbs_obj);
            status = MB_EILLSTATE;
        }
    }
    CRITICAL_SECTION_CLOSE(inst->lock);
    return status;
}

mb_err_enum_t mbs_enable(mb_base_t *inst)
{
    mbs_object_t *mbs_obj = __containerof(inst, mbs_object_t, base);
    mb_err_enum_t status = MB_ENOERR;
    CRITICAL_SECTION(inst->lock) {
        if (mbs_obj->cur_state == STATE_DISABLED) {
            /* Activate the protocol stack. */
            mbs_obj->base.transp_obj->frm_start(mbs_obj->base.transp_obj);
            mbs_obj->cur_state = STATE_ENABLED;
            status = MB_ENOERR;
        } else {
            status = MB_EILLSTATE;
        }
    }
    if (!mbs_obj->mb_address) {
        ESP_LOGD(TAG, "incorrect slave address in %" PRIx32 " object.", (uint32_t)mbs_obj);
        status = MB_EINVAL;
    }
    return status;
}

mb_err_enum_t mbs_disable(mb_base_t *inst)
{
    mb_err_enum_t status = MB_ENOERR;
    mbs_object_t *mbs_obj = __containerof(inst, mbs_object_t, base);
    CRITICAL_SECTION(inst->lock) {
        if (mbs_obj->cur_state == STATE_ENABLED) {
            mbs_obj->base.transp_obj->frm_stop(mbs_obj->base.transp_obj);
            mbs_obj->cur_state = STATE_DISABLED;
            status = MB_ENOERR;
        } else if (mbs_obj->cur_state == STATE_DISABLED) {
            status = MB_ENOERR;
        } else {
            status = MB_EILLSTATE;
        }
    }
    return status;
}

mb_err_enum_t mbs_poll(mb_base_t *inst)
{
    mbs_object_t *mbs_obj = __containerof(inst, mbs_object_t, base);

    mb_exception_t exception;
    mb_err_enum_t status = MB_ENOERR;
    mb_event_t event;

    /* Check if the protocol stack is ready. */
    if (mbs_obj->cur_state != STATE_ENABLED) {
        return MB_EILLSTATE;
    }

    /* Check if there is a event available. If not return control to caller. Otherwise we will handle the event. */
    if (mb_port_evt_get(mbs_obj->base.port_obj, &event)) {
        switch(event.code) {
            case EV_READY:
                ESP_LOGW(TAG, "%s:EV_READY", __func__);
                mb_port_evt_res_release(inst->port_obj);
                break;
                
            case EV_FRAME_RECEIVED:
                ESP_LOGD(TAG, "%s:EV_FRAME_RECEIVED", __func__);
                mbs_obj->transaction.is_active = true;
                mbs_obj->transaction.start_ts = event.post_ts;
                status = inst->transp_obj->frm_rcv(inst->transp_obj, &mbs_obj->rcv_addr, &mbs_obj->frame, &mbs_obj->length);
                // Check if the frame is for us. If not ,send an error process event.
                if (status == MB_ENOERR) {
                    /* Check if the frame is for us. If not ignore the frame. */
                    if((mbs_obj->rcv_addr == mbs_obj->mb_address) || (mbs_obj->rcv_addr == MB_ADDRESS_BROADCAST) 
                                                                    || (mbs_obj->rcv_addr == MB_TCP_PSEUDO_ADDRESS)) {
                        (void)mb_port_evt_post(inst->port_obj, EVENT(EV_EXECUTE));
                        ESP_LOG_BUFFER_HEX_LEVEL("Receive data", &mbs_obj->frame[MB_PDU_FUNC_OFF], (uint16_t)mbs_obj->length, ESP_LOG_WARN);
                    }
                }
                break;

            case EV_EXECUTE:
                MB_RETURN_ON_FALSE(mbs_obj->frame, MB_EILLSTATE, TAG, "receive buffer fail.");
                ESP_LOGD(TAG, "%s:EV_EXECUTE", __func__);
                mbs_obj->func_code = mbs_obj->frame[MB_PDU_FUNC_OFF];
                exception = MB_EX_ILLEGAL_FUNCTION;
                /* If receive frame has exception. The receive function code highest bit is 1.*/
                for (int i = 0; (i < MB_FUNC_HANDLERS_MAX); i++) {
                    /* No more function handlers registered. Abort. */
                    if (mbs_obj->func_handlers[i].func_code == 0) {
                        break;
                    }
                    if ((mbs_obj->func_handlers[i].func_code) == mbs_obj->func_code) {
                        exception = mbs_obj->func_handlers[i].handler(inst, mbs_obj->frame, &mbs_obj->length);
                        break;
                    }
                }
                /* If the request was not sent to the broadcast address, return a reply. */
                if ((mbs_obj->rcv_addr != MB_ADDRESS_BROADCAST) || (mbs_obj->cur_mode == MB_TCP)) {
                    if (exception != MB_EX_NONE) {
                        /* An exception occurred. Build an error frame. */
                        mbs_obj->length = 0;
                        mbs_obj->frame[mbs_obj->length++] = (uint8_t)(mbs_obj->func_code | MB_FUNC_ERROR);
                        mbs_obj->frame[mbs_obj->length++] = exception;
                    }
                    if ((mbs_obj->cur_mode == MB_ASCII) && MB_ASCII_TIMEOUT_WAIT_BEFORE_SEND_MS) {
                        mb_port_tmr_delay(inst->port_obj, MB_ASCII_TIMEOUT_WAIT_BEFORE_SEND_MS);
                    }
                    ESP_LOG_BUFFER_HEX_LEVEL("Send data", (void *)mbs_obj->frame, mbs_obj->length, ESP_LOG_WARN);
                    status = inst->transp_obj->frm_send(inst->transp_obj, mbs_obj->rcv_addr, mbs_obj->frame, mbs_obj->length);
                    if (status != MB_ENOERR) {
                        ESP_LOGE(TAG, "%s:Frame send error. %d", __func__, status);
                    }
                }
                break;

            case EV_FRAME_TRANSMIT:
                ESP_LOGD(TAG, "%s:EV_FRAME_TRANSMIT", __func__);
                break;

            case EV_FRAME_SENT:
                ESP_LOGD(TAG, "%s:EV_MASTER_FRAME_SENT", __func__);
                uint64_t time_div_us = 0;
                mbs_obj->transaction.is_active = false;
                mbs_obj->transaction.end_ts = event.post_ts;
                time_div_us = mbs_obj->transaction.end_ts - mbs_obj->transaction.start_ts;
                ESP_LOGW(TAG, "%p, transaction processing time(us) = %" PRId64, inst, time_div_us);
                break;

            default:
                ESP_LOGE(TAG, "%s: Unexpected event triggered 0x%02x.", __func__, (uint16_t)event.code);
                break;
        }
    } else {
        // Something went wrong and task unblocked but there are no any correct events set
        ESP_LOGE(TAG, "%s: Unexpected event triggered 0x%02x.", __func__, (uint16_t)event.code);
        status = MB_EILLSTATE;
    }
    return status;
}
