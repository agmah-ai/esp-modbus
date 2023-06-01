/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mb_common.h"
#include "mb_proto.h"
#include "mb_func.h"

/**
 * Modbus master input register callback function.
 *
 * @param reg_buff input register buffer
 * @param reg_addr input register address
 * @param reg_num input register number
 *
 * @return result
 */
// __attribute__ ((weak))
// mb_err_enum_t mbm_reg_input_cb(mb_base_t *inst, uint8_t *reg_buff, uint16_t reg_addr, uint16_t reg_num)
// {
//     mb_err_enum_t status = MB_ENOERR;
//     (void)inst;

//     return status;
// }

/**
 * Modbus master holding register callback function.
 *
 * @param reg_buff holding register buffer
 * @param reg_addr holding register address
 * @param reg_num holding register number
 * @param mode read or write
 *
 * @return result
 */
// __attribute__ ((weak))
// mb_err_enum_t mbm_reg_holding_cb(mb_base_t *inst, uint8_t *reg_buff, uint16_t reg_addr, uint16_t reg_num, mb_reg_mode_enum_t mode)
// {
//     mb_err_enum_t status = MB_ENOERR;

//     (void)inst;

//     ESP_LOGW(__func__, "default callback.");
//     return status;
// }

/**
 * Modbus master coils callback function.
 *
 * @param reg_buff coils buffer
 * @param reg_addr coils address
 * @param coil_num coils number
 * @param mode read or write
 *
 * @return result
 */
// __attribute__ ((weak))
// mb_err_enum_t mbm_reg_coils_cb(mb_base_t *inst, uint8_t *reg_buff, uint16_t reg_addr, uint16_t coil_num, mb_reg_mode_enum_t mode)
// {
//     (void)inst;
//     (void)reg_addr;

//     ESP_LOGW(__func__, "default callback.");

//     return MB_ENOREG;
// }

/**
 * Modbus master discrete callback function.
 *
 * @param reg_buff discrete buffer
 * @param reg_addr discrete address
 * @param disc_num discrete number
 *
 * @return result
 */
// __attribute__ ((weak))
// mb_err_enum_t mbm_reg_discrete_cb(mb_base_t *inst, uint8_t *reg_buff, uint16_t reg_addr, uint16_t disc_num)
// {
//     (void)inst;
//     (void)reg_buff;
//     (void)reg_addr;

//     ESP_LOGW(__func__, "default callback.");

//     return MB_ENOREG;
// }

// __attribute__ ((weak))
// mb_err_enum_t mbs_reg_input_cb(mb_base_t *inst, uint8_t *reg_buff, uint16_t reg_addr, uint16_t reg_num)
// {
//     (void)inst;
//     (void)reg_buff;
//     (void)reg_addr;
//     (void)reg_num;

//     ESP_LOGW(__func__, "default callback.");

//     return MB_ENOREG;
// }

// __attribute__ ((weak))
// mb_err_enum_t mbs_reg_holding_cb(mb_base_t *inst, uint8_t *reg_buff, uint16_t reg_addr, uint16_t reg_num, mb_reg_mode_enum_t mode)
// {
//     (void)inst;
//     (void)reg_buff;
//     (void)reg_addr;
//     (void)reg_num;
//     (void)mode;
//     ESP_LOGW(__func__, "default callback.");
//     return MB_ENOREG;
// }

// __attribute__ ((weak))
// mb_err_enum_t mbs_reg_coils_cb(mb_base_t *inst, uint8_t *reg_buff, uint16_t reg_addr, uint16_t coil_num, mb_reg_mode_enum_t mode)
// {
//     (void)inst;
//     (void)reg_buff;
//     (void)reg_addr;
//     (void)coil_num;
//     (void)mode;
//     return MB_ENOREG;
// }

// __attribute__ ((weak))
// mb_err_enum_t mbs_reg_discrete_cb(mb_base_t *inst, uint8_t *reg_buff, uint16_t reg_addr, uint16_t disc_num)
// {
//     (void)inst;
//     (void)reg_buff;
//     (void)reg_addr;
//     (void)disc_num;
//     ESP_LOGW(__func__, "default callback.");
//     return MB_ENOREG;
// }                                                  