/**
 \addtogroup INTERFACES
 \{
 \addtogroup BLE
 \{
 \addtogroup MANAGER
 \{
 */

/**
 ****************************************************************************************
 * @file ble_mgr_irb.h
 *
 * @brief BLE IRB definitions
 *
 * Copyright (C) 2015. Dialog Semiconductor, unpublished work. This computer
 * program includes Confidential, Proprietary Information and is a Trade Secret of
 * Dialog Semiconductor. All use, disclosure, and/or reproduction is prohibited
 * unless authorized in writing. All Rights Reserved.
 *
 * <black.orca.support@diasemi.com> and contributors.
 *
 ****************************************************************************************
 */

#ifndef BLE_MGR_IRB_COMMON_H_
#define BLE_MGR_IRB_COMMON_H_

#include "osal.h"
#include "ble_mgr_irb.h"

enum irb_ble_opcode_common {
        IRB_BLE_STACK_MSG = IRB_BLE_CAT_EVENT0(IRB_BLE_CAT_COMMON),
        IRB_BLE_REGISTER_CMD,
        IRB_BLE_ENABLE_CMD,
        IRB_BLE_RESET_CMD,
        IRB_BLE_READ_TX_POWER_CMD,
        // dummy event ID, needs to be always defined after all command IRBs
        IRB_BLE_LAST_COMMON,
};

void irb_ble_handler_stack_api_msg(OS_IRB *irb);

typedef struct {
        irb_ble_hdr_t   hdr;
        OS_TASK         task;
} irb_ble_register_cmd_t;

typedef struct {
        irb_ble_hdr_t   hdr;
        ble_error_t     status;
} irb_ble_register_rsp_t;

typedef struct {
        irb_ble_hdr_t   hdr;
} irb_ble_enable_cmd_t;

typedef struct {
        irb_ble_hdr_t   hdr;
        ad_ble_status_t status;
} irb_ble_enable_rsp_t;

typedef struct {
        irb_ble_hdr_t   hdr;
} irb_ble_reset_cmd_t;

typedef struct {
        irb_ble_hdr_t   hdr;
        ad_ble_status_t status;
} irb_ble_reset_rsp_t;

typedef struct {
        irb_ble_hdr_t         hdr;
        uint16_t              conn_idx;
        tx_power_level_type_t type;
} irb_ble_read_tx_power_cmd_t;

typedef struct {
        irb_ble_hdr_t   hdr;
        ble_error_t     status;
        uint8_t         tx_power_level;
} irb_ble_read_tx_power_rsp_t;

void irb_ble_handler_register_cmd(OS_IRB *irb);
void irb_ble_handler_enable_cmd(OS_IRB *irb);
void irb_ble_handler_reset_cmd(OS_IRB *irb);
void irb_ble_handler_read_tx_power(OS_IRB *irb);

#endif /* BLE_MGR_IRB_COMMON_H_ */
/**
 \}
 \}
 \}
 */
