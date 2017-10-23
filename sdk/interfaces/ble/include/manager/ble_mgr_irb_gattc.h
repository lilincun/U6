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
 * @file ble_mgr_irb_gattc.h
 *
 * @brief BLE IRB definitions (for GATTC)
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

#ifndef BLE_MGR_IRB_GATTC_H_
#define BLE_MGR_IRB_GATTC_H_

#include <stdint.h>
#include "osal.h"
#include "ble_mgr_irb.h"
#include "ble_att.h"
#include "ble_gatt.h"
#include "ble_gattc.h"

enum irb_ble_opcode_gattc {
        IRB_BLE_GATTC_BROWSE_CMD = IRB_BLE_CAT_EVENT0(IRB_BLE_CAT_GATTC),
        IRB_BLE_GATTC_DISCOVER_SVC_CMD,
        IRB_BLE_GATTC_DISCOVER_INCLUDE_CMD,
        IRB_BLE_GATTC_DISCOVER_CHAR_CMD,
        IRB_BLE_GATTC_DISCOVER_DESC_CMD,
        IRB_BLE_GATTC_READ_CMD,
        IRB_BLE_GATTC_WRITE_GENERIC_CMD,
        IRB_BLE_GATTC_WRITE_EXECUTE_CMD,
        IRB_BLE_GATTC_EXCHANGE_MTU_CMD,
        // dummy event ID, needs to be always defined after all command IRBs
        IRB_BLE_LAST_GATTC,
};

typedef struct {
        irb_ble_hdr_t    hdr;
        uint16_t         conn_idx;
        const att_uuid_t *uuid;
} irb_ble_gattc_browse_cmd_t;

typedef struct {
        irb_ble_hdr_t    hdr;
        ble_error_t      status;
} irb_ble_gattc_browse_rsp_t;

void irb_ble_handler_gattc_browse_cmd(OS_IRB *irb);

typedef struct {
        irb_ble_hdr_t    hdr;
        uint16_t         conn_idx;
        const att_uuid_t *uuid;
} irb_ble_gattc_discover_svc_cmd_t;

typedef struct {
        irb_ble_hdr_t    hdr;
        ble_error_t      status;
} irb_ble_gattc_discover_svc_rsp_t;

void irb_ble_handler_gattc_discover_svc_cmd(OS_IRB *irb);

typedef struct {
        irb_ble_hdr_t    hdr;
        uint16_t         conn_idx;
        uint16_t         start_h;
        uint16_t         end_h;
} irb_ble_gattc_discover_include_cmd_t;

typedef struct {
        irb_ble_hdr_t    hdr;
        ble_error_t      status;
} irb_ble_gattc_discover_include_rsp_t;

void irb_ble_handler_gattc_discover_include_cmd(OS_IRB *irb);

typedef struct {
        irb_ble_hdr_t    hdr;
        uint16_t         conn_idx;
        uint16_t         start_h;
        uint16_t         end_h;
        const att_uuid_t *uuid;
} irb_ble_gattc_discover_char_cmd_t;

typedef struct {
        irb_ble_hdr_t    hdr;
        ble_error_t      status;
} irb_ble_gattc_discover_char_rsp_t;

void irb_ble_handler_gattc_discover_char_cmd(OS_IRB *irb);

typedef struct {
        irb_ble_hdr_t    hdr;
        uint16_t         conn_idx;
        uint16_t         start_h;
        uint16_t         end_h;
} irb_ble_gattc_discover_desc_cmd_t;

typedef struct {
        irb_ble_hdr_t    hdr;
        ble_error_t      status;
} irb_ble_gattc_discover_desc_rsp_t;

void irb_ble_handler_gattc_discover_desc_cmd(OS_IRB *irb);

typedef struct {
        irb_ble_hdr_t    hdr;
        uint16_t         conn_idx;
        uint16_t         handle;
        uint16_t         offset;
} irb_ble_gattc_read_cmd_t;

typedef struct {
        irb_ble_hdr_t    hdr;
        ble_error_t      status;
} irb_ble_gattc_read_rsp_t;

void irb_ble_handler_gattc_read_cmd(OS_IRB *irb);

typedef struct {
        irb_ble_hdr_t    hdr;
        uint16_t         conn_idx;
        uint16_t         handle;
        bool             no_response:1;
        bool             signed_write:1;
        bool             prepare:1;
        uint16_t         offset;
        uint16_t         length;
        const uint8_t    *value;
} irb_ble_gattc_write_generic_cmd_t;

typedef struct {
        irb_ble_hdr_t    hdr;
        ble_error_t      status;
} irb_ble_gattc_write_generic_rsp_t;

void irb_ble_handler_gattc_write_generic_cmd(OS_IRB *irb);

typedef struct {
        irb_ble_hdr_t    hdr;
        uint16_t         conn_idx;
        bool             commit;
} irb_ble_gattc_write_execute_cmd_t;

typedef struct {
        irb_ble_hdr_t    hdr;
        ble_error_t      status;
} irb_ble_gattc_write_execute_rsp_t;

void irb_ble_handler_gattc_write_execute_cmd(OS_IRB *irb);

typedef struct {
        irb_ble_hdr_t    hdr;
        uint16_t         conn_idx;
} irb_ble_gattc_exchange_mtu_cmd_t;

typedef struct {
        irb_ble_hdr_t    hdr;
        ble_error_t      status;
} irb_ble_gattc_exchange_mtu_rsp_t;

void irb_ble_handler_gattc_exchange_mtu_cmd(OS_IRB *irb);

// events start here

void irb_ble_handler_gattc_mtu_changed_ind_evt(ble_gtl_msg_t *gtl);

void irb_ble_handler_gattc_sdp_svc_ind_evt(ble_gtl_msg_t *gtl);

void irb_ble_handler_gattc_cmp__browse_evt(ble_gtl_msg_t *gtl);

void irb_ble_handler_gattc_disc_svc_ind_evt(ble_gtl_msg_t *gtl);

void irb_ble_handler_gattc_disc_svc_incl_ind_evt(ble_gtl_msg_t *gtl);

void irb_ble_handler_gattc_disc_char_ind_evt(ble_gtl_msg_t *gtl);

void irb_ble_handler_gattc_disc_char_desc_ind_evt(ble_gtl_msg_t *gtl);

void irb_ble_handler_gattc_cmp__discovery_evt(ble_gtl_msg_t *gtl);

void irb_ble_handler_gattc_read_ind_evt(ble_gtl_msg_t *gtl);

void irb_ble_handler_gattc_cmp__read_evt(ble_gtl_msg_t *gtl);

void irb_ble_handler_gattc_cmp__write_evt(ble_gtl_msg_t *gtl);

void irb_ble_handler_gattc_cmp__exec_write_evt(ble_gtl_msg_t *gtl);

void irb_ble_handler_gattc_event_ind_evt(ble_gtl_msg_t *gtl);

void irb_ble_handler_gattc_event_req_ind_evt(ble_gtl_msg_t *gtl);

void irb_ble_handler_gattc_svc_changed_cfg_ind_evt(ble_gtl_msg_t *gtl);

#endif /* BLE_MGR_IRB_GATTC_H_ */
/**
 \}
 \}
 \}
 */
