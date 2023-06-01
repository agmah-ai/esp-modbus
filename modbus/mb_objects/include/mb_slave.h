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

// mb_err_enum_t mbs_close(mb_base_t *inst);
// mb_err_enum_t mbs_enable(mb_base_t *inst);
// mb_err_enum_t mbs_disable(mb_base_t *inst);
// mb_err_enum_t mbs_poll(mb_base_t *inst);

mb_err_enum_t mb_set_slv_id(mb_base_t *inst, uint8_t slv_id, bool is_running, uint8_t const *slv_idstr, uint16_t slv_idstr_len);

// /* ----------------------- Callbacks -----------------------------------------*/
// mb_err_enum_t mbs_reg_input_cb(mb_base_t *inst, uint8_t *reg_buff, uint16_t reg_addr, uint16_t reg_num);
// mb_err_enum_t mbs_reg_holding_cb(mb_base_t *inst, uint8_t *reg_buff, uint16_t reg_addr, uint16_t reg_num, mb_reg_mode_enum_t mode);
// mb_err_enum_t mbs_reg_coils_cb(mb_base_t *inst, uint8_t *reg_buff, uint16_t reg_addr, uint16_t coil_num, mb_reg_mode_enum_t mode);
// mb_err_enum_t mbs_reg_discrete_cb(mb_base_t *inst, uint8_t *reg_buff, uint16_t reg_addr, uint16_t disc_num);

#ifdef __cplusplus
}
#endif