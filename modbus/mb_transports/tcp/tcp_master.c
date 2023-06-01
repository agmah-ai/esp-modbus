/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "tcp_transport.h"
#include "port_tcp_common.h"

static const char *TAG = "mb_transp.tcp_master";

typedef struct
{
    mb_trans_base_t base;
    mb_port_base_t *port_obj;
    uint8_t recv_buf[MB_TCP_BUF_SIZE];
    uint8_t send_buf[MB_TCP_BUF_SIZE];
    mb_tcp_state_enum_t state;
    uint16_t snd_pdu_len;
} mbm_tcp_transp_t;

/* ----------------------- Defines ------------------------------------------*/

/* ----------------------- Function prototypes ------------------------------*/
mb_err_enum_t mbm_tcp_transp_create(mb_tcp_opts_t *tcp_opts, mb_trans_base_t **out_inst);
static void mbm_tcp_transp_start(mb_trans_base_t *inst);
static void mbm_tcp_transp_stop(mb_trans_base_t *inst);
static mb_err_enum_t mbm_tcp_transp_receive(mb_trans_base_t *inst, uint8_t *rcv_addr_buf, uint8_t **frame_ptr_buf, uint16_t *len_buf);
static mb_err_enum_t mbm_tcp_transp_send(mb_trans_base_t *inst, uint8_t _unused, const uint8_t *pframe, uint16_t len);
static void mbm_tcp_transp_get_rcv_buf(mb_trans_base_t *inst, uint8_t **frame_ptr_buf);
static void mbm_tcp_transp_get_snd_buf(mb_trans_base_t *inst, uint8_t **frame_ptr_buf);
bool mbm_tcp_transp_delete(mb_trans_base_t *inst);
static bool mbm_tcp_transp_rq_is_bcast(mb_trans_base_t *inst);
static bool mbm_tcp_transp_tmr_expired(void *inst);

mb_err_enum_t mbm_tcp_transp_create(mb_tcp_opts_t *tcp_opts, mb_trans_base_t **out_inst)
{
    mb_err_enum_t ret = MB_ENOERR;
    mbm_tcp_transp_t *transp = NULL;
    transp = (mbm_tcp_transp_t *)calloc(1, sizeof(mbm_tcp_transp_t));
    MB_RETURN_ON_FALSE(transp, MB_EILLSTATE, TAG, "no mem for instance.");
    CRITICAL_SECTION_INIT(transp->base.lock);
    CRITICAL_SECTION_LOCK(transp->base.lock);
    transp->base.frm_rcv = mbm_tcp_transp_receive;
    transp->base.frm_send = mbm_tcp_transp_send;
    transp->base.frm_start = mbm_tcp_transp_start;
    transp->base.frm_stop = mbm_tcp_transp_stop;
    transp->base.get_rx_frm = mbm_tcp_transp_get_rcv_buf;
    transp->base.get_tx_frm = mbm_tcp_transp_get_snd_buf;
    transp->base.frm_delete = mbm_tcp_transp_delete;
    transp->base.frm_is_bcast = mbm_tcp_transp_rq_is_bcast;
    mb_port_base_t *port_obj = NULL;
    ret = mbm_port_tcp_create(tcp_opts, &port_obj);
    MB_GOTO_ON_FALSE((ret == MB_ENOERR), MB_EPORTERR, error, TAG, "port creation, err: %d", ret);
    ret = mb_port_tmr_create(port_obj, MB_TCP_TIMEOUT_MS * MB_TIMER_TICS_PER_MS);
    MB_GOTO_ON_FALSE((ret == MB_ENOERR), MB_EPORTERR, error, TAG, "timer port creation, err: %d", ret);
    // Override default response time if defined
    if (tcp_opts->response_tout_ms) {
        mb_port_tmr_set_response_time(port_obj, tcp_opts->response_tout_ms);
    }
    ret = mb_port_evt_create(port_obj);
    MB_GOTO_ON_FALSE((ret == MB_ENOERR), MB_EPORTERR, error, TAG, "event port creation, err: %d", ret);
    transp->base.port_obj = port_obj;

    transp->port_obj = port_obj;
    *out_inst = &(transp->base);
    ESP_LOGD(TAG, "created %s object @%p", TAG, transp);
    CRITICAL_SECTION_UNLOCK(transp->base.lock);
    return MB_ENOERR;
error:
    if (port_obj) {
        free(port_obj->event_obj);
        free(port_obj->timer_obj);
    }
    free(port_obj);
    CRITICAL_SECTION_CLOSE(transp->base.lock);
    free(transp);
    return ret;
}

bool mbm_tcp_transp_delete(mb_trans_base_t *inst)
{
    mbm_tcp_transp_t *transp = __containerof(inst, mbm_tcp_transp_t, base);
    // destroy method of port tcp master is here
    CRITICAL_SECTION(inst->lock) {
        mbm_port_tcp_delete(inst->port_obj);
        mb_port_tmr_delete(inst->port_obj);
        mb_port_evt_delete(inst->port_obj);
    }
    CRITICAL_SECTION_CLOSE(inst->lock);
    free(transp);
    return true;
}

static void mbm_tcp_transp_start(mb_trans_base_t *inst)
{
    mbm_tcp_transp_t *transp = __containerof(inst, mbm_tcp_transp_t, base);
    transp->state = MB_TCP_STATE_INIT;
    CRITICAL_SECTION(inst->lock) {
        mbm_port_tcp_enable(inst->port_obj);
        mb_port_tmr_enable(inst->port_obj);
    };
    /* No special startup required for TCP. */
    (void)mb_port_evt_post(inst->port_obj, EVENT(EV_READY));
}

static void mbm_tcp_transp_stop(mb_trans_base_t *inst)
{
    /* Make sure that no more clients are connected. */
    CRITICAL_SECTION(inst->lock) {
        mbm_port_tcp_disable(inst->port_obj);
        mb_port_tmr_disable(inst->port_obj);
    };
}

static mb_err_enum_t mbm_tcp_transp_receive(mb_trans_base_t *inst, uint8_t *rcv_addr, uint8_t **frame_ptr_buf, uint16_t *len_buf)
{
    mb_err_enum_t status = MB_EIO;
    uint8_t *frame_ptr;
    uint16_t len;
    uint16_t pid;

    if (mbm_port_tcp_recv_data(inst->port_obj, &frame_ptr, &len) != false) {
        pid = frame_ptr[MB_TCP_PID] << 8U;
        pid |= frame_ptr[MB_TCP_PID + 1];

        if (pid == MB_TCP_PROTOCOL_ID) {
            *frame_ptr_buf = &frame_ptr[MB_TCP_FUNC];
            *len_buf = len - MB_TCP_FUNC;
            status = MB_ENOERR;

            /* Get MBAP UID field if its support is enabled.
             * Otherwise just ignore this field.
             */
#if MB_TCP_UID_ENABLED
            *rcv_addr = frame_ptr[MB_TCP_UID];
#else
            *rcv_addr = MB_TCP_PSEUDO_ADDRESS;
#endif
        }
    } else {
        status = MB_EIO;
    }
    return status;
}

static mb_err_enum_t mbm_tcp_transp_send(mb_trans_base_t *inst, uint8_t address, const uint8_t *pframe, uint16_t len)
{
    mb_err_enum_t status = MB_ENOERR;
    uint8_t *frame_ptr = (uint8_t *)pframe - MB_TCP_FUNC;
    uint16_t tcp_len = len + MB_TCP_FUNC;

    /* The MBAP header is already initialized because the caller calls this
     * function with the buffer returned by the previous call. Therefore we
     * only have to update the length in the header. Note that the length
     * header includes the size of the Modbus PDU and the UID Byte. Therefore
     * the length is len plus one.
     */
    frame_ptr[MB_TCP_LEN] = (len + 1) >> 8U;
    frame_ptr[MB_TCP_LEN + 1] = (len + 1) & 0xFF;

    /* Set UID field in the MBAP if it is supported.
     * If the RTU over TCP is not supported, the UID = 0 or 0xFF.
     */
#if MB_TCP_UID_ENABLED
    frame_ptr[MB_TCP_UID] = address;
#else
    frame_ptr[MB_TCP_UID] = 0x00;
#endif
    if (mbm_port_tcp_send_data(inst->port_obj, address, frame_ptr, tcp_len) == false) {
        status = MB_EIO;
    }
    return status;
}

// __attribute__((always_inline)) 
// static bool mbm_tcp_transp_tmr_expired(void *inst)
// {
//     mbm_tcp_transp_t *transp = __containerof(inst, mbm_tcp_transp_t, base);
    
//     bool need_poll = false;
//     mb_tmr_mode_enum_t timer_mode = mb_port_get_cur_tmr_mode(transp->base.port_obj);

//     mb_port_tmr_disable(transp->base.port_obj);

//     switch(timer_mode) {
//         case MB_TMODE_T35:
//             need_poll = mb_port_evt_post(transp->base.port_obj, EVENT(EV_READY);
//             ESP_EARLY_LOGD(TAG, "EV_READY");
//             break;

//         case MB_TMODE_RESPOND_TIMEOUT:
//             mb_port_evt_set_err_type(transp->base.port_obj, EVENT(EV_ERROR_RESPOND_TIMEOUT);
//             need_poll = mb_port_evt_post(transp->base.port_obj, EVENT(EV_ERROR_PROCESS);
//             // need_poll = mb_port_evt_post(transp->base.port_obj, EVENT(EV_MASTER_ERROR_RESPOND_TIMEOUT));
//             ESP_EARLY_LOGD(TAG, "EV_ERROR_RESPOND_TIMEOUT");
//             break;

//         case MB_TMODE_CONVERT_DELAY:
//             /* If timer mode is convert delay, the master event then turns EV_MASTER_EXECUTE status. */
//             need_poll = mb_port_evt_post(transp->base.port_obj, EVENT(EV_EXECUTE);
//             ESP_EARLY_LOGD(TAG, "MB_TMODE_CONVERT_DELAY");
//             break;
//         default:
//             need_poll = mb_port_evt_post(transp->base.port_obj, EVENT(EV_READY);
//             break;
//     }
    
//     return need_poll;
// }

static void mbm_tcp_transp_get_rcv_buf(mb_trans_base_t *inst, uint8_t **frame_ptr_buf)
{
    mbm_tcp_transp_t *transp = __containerof(inst, mbm_tcp_transp_t, base);
    CRITICAL_SECTION(inst->lock) {
        *frame_ptr_buf = transp->recv_buf + MB_TCP_FUNC;
    }
}

static void mbm_tcp_transp_get_snd_buf(mb_trans_base_t *inst, uint8_t **frame_ptr_buf)
{
    mbm_tcp_transp_t *transp = __containerof(inst, mbm_tcp_transp_t, base);
    CRITICAL_SECTION(inst->lock) {
        *frame_ptr_buf = transp->send_buf + MB_TCP_FUNC;
    }
}

static bool mbm_tcp_transp_rq_is_bcast(mb_trans_base_t *inst)
{
    return false; //no broadcast packets on tcp
}

