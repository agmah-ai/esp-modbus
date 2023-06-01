/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <stdbool.h>
#include <string.h>

/*----------------------- Platform includes --------------------------------*/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/portmacro.h"

#include "sys/lock.h"
#include "mb_types.h"

#ifdef __cplusplus
extern "C" {
#endif

int lock_obj(_lock_t *plock);
void unlock_obj(_lock_t *plock);

#define CRITICAL_SECTION_INIT(lock) do { _lock_init((_lock_t *)&lock); } while(0)
#define CRITICAL_SECTION_CLOSE(lock) do { _lock_close((_lock_t *)&lock); } while(0)
#define CRITICAL_SECTION_LOCK(lock) do { lock_obj((_lock_t *)&lock); } while(0)
#define CRITICAL_SECTION_UNLOCK(lock) do { unlock_obj((_lock_t *)&lock); } while(0)
#define CRITICAL_SECTION(lock) for(int st = lock_obj((_lock_t *)&lock); (st > 0) ; unlock_obj((_lock_t *)&lock), st = -1)

#define SPIN_LOCK_INIT(lock) do { spinlock_initialize(&lock); } while(0)
#define SPIN_LOCK_ENTER(lock) do { vPortEnterCriticalSafe(&lock); } while(0)
#define SPIN_LOCK_EXIT(lock) do { vPortExitCriticalSafe(&lock); } while(0)

#define MB_EVENT_RESOURCE       (0x80)
#define MB_SER_PDU_SIZE_MIN     (4)

#define MB_TIMER_TICS_PER_MS    (20UL) // Define number of timer reloads per 1 mS
#define MB_TIMER_TICK_TIME_US   (1000 / MB_TIMER_TICS_PER_MS) // 50uS = one discreet for timer

#define MB_EVENT_REQ_MASK   (EventBits_t)(EV_MASTER_PROCESS_SUCCESS | \
                                            EV_MASTER_ERROR_RESPOND_TIMEOUT | \
                                            EV_MASTER_ERROR_RECEIVE_DATA | \
                                            EV_MASTER_ERROR_EXECUTE_FUNCTION)

#define MB_PORT_CHECK_EVENT(event, mask) (event & mask)
#define MB_PORT_CLEAR_EVENT(event, mask) \
    do                                   \
    {                                    \
        event &= ~mask;                  \
    } while (0)

//concatenation of the two arguments
#define PP_CAT2(_1, _2) PP_CAT_(_1, _2)
#define PP_CAT_(_1, _2) _1##_2

#define PP_VA_NUM_ARGS(...) PP_VA_NUM_ARGS_(__VA_ARGS__,4,3,2,1)
#define PP_VA_NUM_ARGS_(_1,_2,_3,_4,N,...) N

// Initialization event structure using variadic parameters
#define EVENT(...) PP_CAT2(EVENT_, PP_VA_NUM_ARGS(__VA_ARGS__))(__VA_ARGS__)

#define EVENT_1(_1) (mb_event_t){ .code = _1 }
#define EVENT_2(_1,_2) (mb_event_t){ .code = _1, .length = _2 }
#define EVENT_3(_1,_2,_3) (mb_event_t){ .code = _1, .length = _2, .pdata = _3 }
#define EVENT_4(_1,_2,_3,_4) (mb_event_t){ .code = _1, .length = _2, .pdata = _3, .type = _4 }

typedef bool (*mb_port_cb_fp)(void *arg);

//!< port callback table for interrupts
typedef struct
{
    mb_port_cb_fp byte_rcvd;
    mb_port_cb_fp tx_empty;
    mb_port_cb_fp tmr_expired;
} mb_port_cb_t;

typedef struct mb_port_event_t mb_port_event_t;

typedef struct mb_port_timer_t mb_port_timer_t;

struct mb_port_base_t
{
    mb_port_cb_t cb;        //!< Port callbacks.
    void *arg;              //!< CB arg pointer.
    
    _lock_t lock;

    mb_port_event_t *event_obj;
    mb_port_timer_t *timer_obj;
};

typedef struct mb_port_base_t mb_port_base_t;

mb_err_enum_t mb_port_evt_create(mb_port_base_t *port_obj);
bool mb_port_evt_post(mb_port_base_t *inst, mb_event_t event);
bool mb_port_evt_get(mb_port_base_t *inst, mb_event_t *event);
bool mb_port_evt_res_take(mb_port_base_t *inst, uint32_t timeout);
void mb_port_evt_res_release(mb_port_base_t *inst);
void mb_port_evt_set_resp_flag(mb_port_base_t *inst, mb_event_enum_t event_mask);
void mb_port_evt_set_err_type(mb_port_base_t *inst, mb_err_event_t event);
mb_err_event_t mb_port_evt_get_err_type(mb_port_base_t *inst);
void mb_port_evt_delete(mb_port_base_t *inst);
mb_err_enum_t mb_port_evt_wait_req_finish(mb_port_base_t *inst);

mb_err_enum_t mb_port_tmr_create(mb_port_base_t *inst, uint16_t t35_timer_ticks);
void mb_port_tmr_disable(mb_port_base_t *inst);
void mb_port_tmr_enable(mb_port_base_t *inst);
void mb_port_tmr_respond_timeout_enable(mb_port_base_t *inst);
void mb_port_tmr_convert_delay_enable(mb_port_base_t *inst);
void mb_port_set_cur_tmr_mode(mb_port_base_t *inst, mb_tmr_mode_enum_t tmr_mode);
mb_tmr_mode_enum_t mb_port_get_cur_tmr_mode(mb_port_base_t *inst);
void mb_port_tmr_set_response_time(mb_port_base_t *inst, uint32_t resp_time_ms);
uint32_t mb_port_tmr_get_response_time_ms(mb_port_base_t *inst);
void mb_port_tmr_delay(mb_port_base_t *inst, uint16_t timeout_ms);
void mb_port_tmr_delete(mb_port_base_t *inst);

#ifdef __cplusplus
}
#endif