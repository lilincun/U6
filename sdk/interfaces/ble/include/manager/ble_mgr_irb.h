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

#ifndef BLE_MGR_IRB_H_
#define BLE_MGR_IRB_H_

#include <stdbool.h>
#include "osal.h"

typedef void (* irb_ble_handler_t) (OS_IRB *irb);

/**
 * Common header for all BLE IRB
 */
typedef struct {
        uint16_t        op_code;
        uint16_t        msg_len;
        uint8_t         payload[0];
} irb_ble_hdr_t;

/**
 * BLE IRB categories
 */
enum irb_ble_category {
        IRB_BLE_CAT_COMMON      = 0x00,
        IRB_BLE_CAT_GAP         = 0x01,
        IRB_BLE_CAT_GATTS       = 0x02,
        IRB_BLE_CAT_GATTC       = 0x03,
        IRB_BLE_CAT_L2CAP       = 0x04,
        IRB_BLE_CAT_LAST,
};

#define IRB_BLE_CAT_EVENT0(CAT) (CAT << 8)

#define IRB_BLE_GET_CAT(OPCODE) (OPCODE >> 8)
#define IRB_BLE_GET_IDX(OPCODE) (OPCODE & 0xFF)

bool ble_irb_handle_msg(OS_IRB *irb);

#endif /* BLE_MGR_IRB_H_ */
/**
 \}
 \}
 \}
 */
