/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once
#include "mb_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct mb_base_t mb_base_t;  /*!< Type of moddus object */

mb_err_enum_t mbm_rq_read_inp_reg(mb_base_t *inst, uint8_t snd_addr, uint16_t reg_addr, uint16_t reg_num, uint32_t tout);
mb_err_enum_t mbm_rq_write_holding_reg(mb_base_t *inst, uint8_t snd_addr, uint16_t reg_addr, uint16_t reg_data, uint32_t tout);
mb_err_enum_t mbm_rq_write_multi_holding_reg(mb_base_t *inst, uint8_t snd_addr, uint16_t reg_addr, uint16_t reg_wr_addr, uint16_t *data_ptr, uint32_t tout);
mb_err_enum_t mbm_rq_read_holding_reg(mb_base_t *inst, uint8_t snd_addr, uint16_t reg_addr, uint16_t reg_num, uint32_t tout);
mb_err_enum_t mbm_rq_rw_multi_holding_reg(mb_base_t *inst, uint8_t snd_addr, uint16_t rd_reg_addr, 
                                            uint16_t rd_reg_num, uint16_t *data_ptr, uint16_t wr_reg_addr, uint16_t wr_reg_num, uint32_t tout);
mb_err_enum_t mbm_rq_read_discrete_inputs(mb_base_t *inst, uint8_t snd_addr, uint16_t discrete_addr, uint16_t discrete_num, uint32_t tout);
mb_err_enum_t mbm_rq_read_coils(mb_base_t *inst, uint8_t snd_addr, uint16_t coil_addr, uint16_t coil_num, uint32_t tout);
mb_err_enum_t mbm_rq_write_coil(mb_base_t *inst, uint8_t snd_addr, uint16_t coil_addr, uint16_t coil_data, uint32_t tout);
mb_err_enum_t mbm_rq_write_multi_coils(mb_base_t *inst, uint8_t snd_addr, uint16_t coil_addr, uint16_t coil_num, uint8_t *data_ptr, uint32_t tout);

/* ----------------------- Callbacks -----------------------------------------*/
// void mbm_error_exec_fn_cb(mb_base_t *inst);
// void mbm_error_rcv_data_cb(mb_base_t *inst);
// void mbm_error_timeout_cb(mb_base_t *inst);
// void mbm_error_success_cb(mb_base_t *inst);

// mb_err_enum_t mbm_reg_discrete_cb(mb_base_t *inst, uint8_t *reg_buff, uint16_t reg_addr, uint16_t disc_num);
// mb_err_enum_t mbm_reg_input_cb(mb_base_t *inst, uint8_t *reg_buff, uint16_t reg_addr, uint16_t reg_num);
// mb_err_enum_t mbm_reg_holding_cb(mb_base_t *inst, uint8_t *reg_buff, uint16_t reg_addr, uint16_t reg_num, mb_reg_mode_enum_t mode);
// mb_err_enum_t mbm_reg_coils_cb(mb_base_t *inst, uint8_t *reg_buff, uint16_t reg_addr, uint16_t coil_num, mb_reg_mode_enum_t mode);

#ifdef __cplusplus
}
#endif