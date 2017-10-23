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
 * @file ble_mgr_irb_gatts.h
 *
 * @brief BLE IRB definitions (for GATTS)
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

#ifndef BLE_MGR_IRB_GATTS_H_
#define BLE_MGR_IRB_GATTS_H_

#include <stdint.h>
#include "osal.h"
#include "ble_mgr_irb.h"
#include "ble_att.h"
#include "ble_gatt.h"
#include "ble_gatts.h"

enum irb_ble_opcode_gatts {
        IRB_BLE_GATTS_SERVICE_ADD_CMD = IRB_BLE_CAT_EVENT0(IRB_BLE_CAT_GATTS),
        IRB_BLE_GATTS_SERVICE_INCLUDE_ADD_CMD,
        IRB_BLE_GATTS_SERVICE_CHARACTERISTIC_ADD_CMD,
        IRB_BLE_GATTS_SERVICE_DESCRIPTOR_ADD_CMD,
        IRB_BLE_GATTS_SERVICE_REGISTER_CMD,
        IRB_BLE_GATTS_SERVICE_ENABLE_CMD,
        IRB_BLE_GATTS_SERVICE_DISABLE_CMD,
        IRB_BLE_GATTS_SERVICE_CHARACTERISTIC_GET_PROP_CMD,
        IRB_BLE_GATTS_SERVICE_CHARACTERISTIC_SET_PROP_CMD,
        IRB_BLE_GATTS_GET_VALUE_CMD,
        IRB_BLE_GATTS_SET_VALUE_CMD,
        IRB_BLE_GATTS_READ_CFM_CMD,
        IRB_BLE_GATTS_WRITE_CFM_CMD,
        IRB_BLE_GATTS_PREPARE_WRITE_CFM_CMD,
        IRB_BLE_GATTS_SEND_EVENT_CMD,
        IRB_BLE_GATTS_SERVICE_CHANGED_IND_CMD,
        // dummy event ID, needs to be always defined after all command IRBs
        IRB_BLE_LAST_GATTS,
};

typedef struct {
        irb_ble_hdr_t           hdr;
        att_uuid_t              uuid;
        gatt_service_t          type;
        uint16_t                num_attrs;
} irb_ble_gatts_service_add_cmd_t;

typedef struct {
        irb_ble_hdr_t   hdr;
        ble_error_t     status;
} irb_ble_gatts_service_add_rsp_t;

void irb_ble_handler_gatts_service_add_cmd(OS_IRB *irb);

typedef struct {
        irb_ble_hdr_t   hdr;
        uint16_t        handle;
} irb_ble_gatts_service_add_include_cmd_t;

typedef struct {
        irb_ble_hdr_t   hdr;
        ble_error_t     status;
        uint16_t        h_offset;
} irb_ble_gatts_service_add_include_rsp_t;

void irb_ble_handler_gatts_service_add_include_cmd(OS_IRB *irb);

typedef struct {
        irb_ble_hdr_t   hdr;
        att_uuid_t      uuid;
        uint16_t        prop;
        uint16_t        perm;
        uint16_t        max_len;
        gatts_flag_t    flags;
} irb_ble_gatts_service_add_characteristic_cmd_t;

typedef struct {
        irb_ble_hdr_t   hdr;
        ble_error_t     status;
        uint16_t        h_offset;
        uint16_t        h_val_offset;
} irb_ble_gatts_service_add_characteristic_rsp_t;

void irb_ble_handler_gatts_service_add_characteristic_cmd(OS_IRB *irb);

typedef struct {
        irb_ble_hdr_t   hdr;
        att_uuid_t      uuid;
        uint16_t        perm;
        uint16_t        max_len;
        gatts_flag_t    flags;
} irb_ble_gatts_service_add_descriptor_cmd_t;

typedef struct {
        irb_ble_hdr_t   hdr;
        ble_error_t     status;
        uint16_t        h_offset;
} irb_ble_gatts_service_add_descriptor_rsp_t;

void irb_ble_handler_gatts_service_add_descriptor_cmd(OS_IRB *irb);

typedef struct {
        irb_ble_hdr_t   hdr;
} irb_ble_gatts_service_register_cmd_t;

typedef struct {
        irb_ble_hdr_t   hdr;
        ble_error_t     status;
        uint16_t        handle;
} irb_ble_gatts_service_register_rsp_t;

void irb_ble_handler_gatts_service_register_cmd(OS_IRB *irb);

typedef struct {
        irb_ble_hdr_t   hdr;
        uint16_t        handle;
} irb_ble_gatts_service_enable_cmd_t;

typedef struct {
        irb_ble_hdr_t   hdr;
        ble_error_t     status;
} irb_ble_gatts_service_enable_rsp_t;

void irb_ble_handler_gatts_service_enable_cmd(OS_IRB *irb);

typedef struct {
        irb_ble_hdr_t   hdr;
        uint16_t        handle;
} irb_ble_gatts_service_disable_cmd_t;

typedef struct {
        irb_ble_hdr_t   hdr;
        ble_error_t     status;
} irb_ble_gatts_service_disable_rsp_t;

void irb_ble_handler_gatts_service_disable_cmd(OS_IRB *irb);

typedef struct {
        irb_ble_hdr_t   hdr;
        uint16_t        handle;
} irb_ble_gatts_service_characteristic_get_prop_cmd_t;

typedef struct {
        irb_ble_hdr_t   hdr;
        ble_error_t     status;
        gatt_prop_t     prop;
        att_perm_t      perm;
} irb_ble_gatts_service_characteristic_get_prop_rsp_t;

void irb_ble_handler_gatts_service_characteristic_get_prop_cmd(OS_IRB *irb);

typedef struct {
        irb_ble_hdr_t   hdr;
        uint16_t        handle;
        gatt_prop_t     prop;
        att_perm_t      perm;
} irb_ble_gatts_service_characteristic_set_prop_cmd_t;

typedef struct {
        irb_ble_hdr_t   hdr;
        ble_error_t     status;
} irb_ble_gatts_service_characteristic_set_prop_rsp_t;

void irb_ble_handler_gatts_service_characteristic_set_prop_cmd(OS_IRB *irb);

typedef struct {
        irb_ble_hdr_t   hdr;
        uint16_t        handle;
        uint16_t        max_len;
} irb_ble_gatts_get_value_cmd_t;

typedef struct {
        irb_ble_hdr_t   hdr;
        ble_error_t     status;
        uint16_t        length;
        ble_error_t     value[0];
} irb_ble_gatts_get_value_rsp_t;

void irb_ble_handler_gatts_get_value_cmd(OS_IRB *irb);

typedef struct {
        irb_ble_hdr_t   hdr;
        uint16_t        handle;
        uint16_t        length;
        uint8_t         value[0];
} irb_ble_gatts_set_value_cmd_t;

typedef struct {
        irb_ble_hdr_t   hdr;
        ble_error_t     status;
} irb_ble_gatts_set_value_rsp_t;

void irb_ble_handler_gatts_set_value_cmd(OS_IRB *irb);

typedef struct {
        irb_ble_hdr_t   hdr;
        uint16_t        conn_idx;
        uint16_t        handle;
        ble_error_t     status;
        uint16_t        length;
        uint8_t         value[0];
} irb_ble_gatts_read_cfm_cmd_t;

typedef struct {
        irb_ble_hdr_t   hdr;
        ble_error_t     status;
} irb_ble_gatts_read_cfm_rsp_t;

void irb_ble_handler_gatts_read_cfm_cmd(OS_IRB *irb);

typedef struct {
        irb_ble_hdr_t   hdr;
        uint16_t        conn_idx;
        uint16_t        handle;
        uint8_t         status;
} irb_ble_gatts_write_cfm_cmd_t;

typedef struct {
        irb_ble_hdr_t   hdr;
        ble_error_t     status;
} irb_ble_gatts_write_cfm_rsp_t;

void irb_ble_handler_gatts_write_cfm_cmd(OS_IRB *irb);

typedef struct {
        irb_ble_hdr_t   hdr;
        uint16_t        conn_idx;
        uint16_t        handle;
        uint16_t        length;
        uint8_t         status;
} irb_ble_gatts_prepare_write_cfm_cmd_t;

typedef struct {
        irb_ble_hdr_t   hdr;
        ble_error_t     status;
} irb_ble_gatts_prepare_write_cfm_rsp_t;

void irb_ble_handler_gatts_prepare_write_cfm_cmd(OS_IRB *irb);

typedef struct {
        irb_ble_hdr_t   hdr;
        uint16_t        conn_idx;
        uint16_t        handle;
        gatt_event_t    type;
        uint16_t        length;
        uint8_t         value[0];
} irb_ble_gatts_send_event_cmd_t;

typedef struct {
        irb_ble_hdr_t   hdr;
        ble_error_t     status;
} irb_ble_gatts_send_event_rsp_t;

void irb_ble_handler_gatts_send_event_cmd(OS_IRB *irb);

typedef struct {
        irb_ble_hdr_t   hdr;
        uint16_t        conn_idx;
        uint16_t        start_handle;
        uint16_t        end_handle;
} irb_ble_gatts_service_changed_ind_cmd_t;

typedef struct {
        irb_ble_hdr_t   hdr;
        ble_error_t     status;
} irb_ble_gatts_service_changed_ind_rsp_t;

void irb_ble_handler_gatts_service_changed_ind_cmd(OS_IRB *irb);

void irb_ble_handler_gatts_read_value_req_evt(ble_gtl_msg_t *gtl);

void irb_ble_handler_gatts_write_value_req_evt(ble_gtl_msg_t *gtl);

void irb_ble_handler_gatts_prepare_write_req_evt(ble_gtl_msg_t *gtl);

void irb_ble_handler_gatts_event_sent_evt(ble_gtl_msg_t *gtl);

#endif /* BLE_MGR_IRB_GATTS_H_ */
/**
 \}
 \}
 \}
 */
