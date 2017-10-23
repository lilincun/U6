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
 * @file ble_mgr_irb_l2cap.h
 *
 * @brief BLE IRB definitions (for L2CAP CoC)
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

#ifndef BLE_MGR_IRB_L2CAP_H_
#define BLE_MGR_IRB_L2CAP_H_

#include <stdint.h>
#include <stdbool.h>
#include "osal.h"
#include "ble_mgr_irb.h"
#include "ble_mgr_gtl.h"
#include "ble_l2cap.h"

/** OP codes for L2CAP IRBs */
enum irb_ble_opcode_l2cap {
        IRB_BLE_L2CAP_LISTEN_CMD  = IRB_BLE_CAT_EVENT0(IRB_BLE_CAT_L2CAP),
        IRB_BLE_L2CAP_STOP_LISTEN_CMD,
        IRB_BLE_L2CAP_CONNECT_CMD,
        IRB_BLE_L2CAP_DISCONNECT_CMD,
        IRB_BLE_L2CAP_ADD_CREDITS_CMD,
        IRB_BLE_L2CAP_SEND_CMD,

        // dummy event ID, needs to be always defined after all command IRBs
        IRB_BLE_LAST_L2CAP,
};

typedef struct {
        irb_ble_hdr_t   hdr;
        uint16_t        conn_idx;
        uint16_t        psm;
        gap_sec_level_t sec_level;
        uint16_t        initial_credits;
} irb_ble_l2cap_listen_cmd_t;

typedef struct {
        irb_ble_hdr_t   hdr;
        ble_error_t     status;
        uint16_t        scid;
} irb_ble_l2cap_listen_rsp_t;

void irb_ble_handler_l2cap_listen_cmd(OS_IRB *irb);

typedef struct {
        irb_ble_hdr_t   hdr;
        uint16_t        conn_idx;
        uint16_t        scid;
} irb_ble_l2cap_stop_listen_cmd_t;

typedef struct {
        irb_ble_hdr_t   hdr;
        ble_error_t     status;
        uint16_t        scid;
} irb_ble_l2cap_stop_listen_rsp_t;

void irb_ble_handler_l2cap_stop_listen_cmd(OS_IRB *irb);

typedef struct {
        irb_ble_hdr_t   hdr;
        uint16_t        conn_idx;
        uint16_t        psm;
        uint16_t        initial_credits;
} irb_ble_l2cap_connect_cmd_t;

typedef struct {
        irb_ble_hdr_t   hdr;
        ble_error_t     status;
        uint16_t        scid;
} irb_ble_l2cap_connect_rsp_t;

void irb_ble_handler_l2cap_connect_cmd(OS_IRB *irb);

typedef struct {
        irb_ble_hdr_t   hdr;
        uint16_t        conn_idx;
        uint16_t        scid;
} irb_ble_l2cap_disconnect_cmd_t;

typedef struct {
        irb_ble_hdr_t   hdr;
        ble_error_t     status;
        uint16_t        scid;
} irb_ble_l2cap_disconnect_rsp_t;

void irb_ble_handler_l2cap_disconnect_cmd(OS_IRB *irb);

typedef struct {
        irb_ble_hdr_t   hdr;
        uint16_t        conn_idx;
        uint16_t        scid;
        uint16_t        credits;
} irb_ble_l2cap_add_credits_cmd_t;

typedef struct {
        irb_ble_hdr_t   hdr;
        ble_error_t     status;
        uint16_t        conn_idx;
        uint16_t        scid;
        uint16_t        credits;
} irb_ble_l2cap_add_credits_rsp_t;

void irb_ble_handler_l2cap_add_credits_cmd(OS_IRB *irb);

typedef struct {
        irb_ble_hdr_t   hdr;
        uint16_t        conn_idx;
        uint16_t        scid;
        uint16_t        length;
        uint8_t         data[0];
} irb_ble_l2cap_send_cmd_t;

typedef struct {
        irb_ble_hdr_t   hdr;
        ble_error_t     status;
} irb_ble_l2cap_send_rsp_t;

void irb_ble_handler_l2cap_send_cmd(OS_IRB *irb);


void irb_ble_handler_l2cap_connect_ind_evt(ble_gtl_msg_t *gtl);

void irb_ble_handler_l2cap_disconnect_ind_evt(ble_gtl_msg_t *gtl);

void irb_ble_handler_l2cap_connect_req_ind_evt(ble_gtl_msg_t *gtl);

void irb_ble_handler_l2cap_add_ind_evt(ble_gtl_msg_t *gtl);

void irb_ble_handler_l2cap_pdu_send_rsp_evt(ble_gtl_msg_t *gtl);

void irb_ble_handler_l2cap_lecnx_data_recv_ind_evt(ble_gtl_msg_t *gtl);

void irb_ble_handler_gapc_cmp__le_cb_connection_evt(ble_gtl_msg_t *gtl);

void l2cap_disconnect_ind(uint16_t conn_idx);

#endif /* BLE_MGR_IRB_L2CAP_H_ */
/**
 \}
 \}
 \}
 */
