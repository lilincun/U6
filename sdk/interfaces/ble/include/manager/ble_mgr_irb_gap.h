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
 * @file ble_mgr_irb_gap.h
 *
 * @brief BLE IRB definitions (for GAP)
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

#ifndef BLE_MGR_IRB_GAP_H_
#define BLE_MGR_IRB_GAP_H_

#include <stdint.h>
#include <stdbool.h>
#include "osal.h"
#include "ble_mgr_irb.h"
#include "ble_gap.h"

/** OP codes for GAP IRBs */
enum irb_ble_opcode_gap {
        IRB_BLE_GAP_ADDRESS_SET_CMD  = IRB_BLE_CAT_EVENT0(IRB_BLE_CAT_GAP),
        IRB_BLE_GAP_DEVICE_NAME_SET_CMD,
        IRB_BLE_GAP_APPEARANCE_SET_CMD,
        IRB_BLE_GAP_PPCP_SET_CMD,
        IRB_BLE_GAP_ADV_START_CMD,
        IRB_BLE_GAP_ADV_STOP_CMD,
        IRB_BLE_GAP_ADV_DATA_SET_CMD,
        IRB_BLE_GAP_SCAN_START_CMD,
        IRB_BLE_GAP_SCAN_STOP_CMD,
        IRB_BLE_GAP_CONNECT_CMD,
        IRB_BLE_GAP_CONNECT_CANCEL_CMD,
        IRB_BLE_GAP_DISCONNECT_CMD,
        IRB_BLE_GAP_CONN_RSSI_GET_CMD,
        IRB_BLE_GAP_ROLE_SET_CMD,
        IRB_BLE_GAP_MTU_SIZE_SET_CMD,
        IRB_BLE_GAP_CHANNEL_MAP_SET_CMD,
        IRB_BLE_GAP_CONN_PARAM_UPDATE_CMD,
        IRB_BLE_GAP_CONN_PARAM_UPDATE_REPLY_CMD,
        IRB_BLE_GAP_PAIR_CMD,
        IRB_BLE_GAP_PAIR_REPLY_CMD,
        IRB_BLE_GAP_PASSKEY_REPLY_CMD,
        IRB_BLE_GAP_UNPAIR_CMD,
        IRB_BLE_GAP_SET_SEC_LEVEL_CMD,
#if (dg_configBLE_SKIP_LATENCY_API == 1)
        IRB_BLE_GAP_SKIP_LATENCY_CMD,
#endif /* (dg_configBLE_SKIP_LATENCY_API == 1) */
        IRB_BLE_GAP_DATA_LENGTH_SET_CMD,
#if (dg_configBLE_SECURE_CONNECTIONS == 1)
        IRB_BLE_GAP_NUMERIC_REPLY_CMD,
#endif /* (dg_configBLE_SECURE_CONNECTIONS == 1) */
        // dummy event ID, needs to be always defined after all command IRBs
        IRB_BLE_LAST_GAP,
};

/** GAP address set command message structure */
typedef struct {
        irb_ble_hdr_t        hdr;
        const own_address_t  *address;
        uint16_t             renew_dur;
} irb_ble_gap_address_set_cmd_t;

/** GAP address set response message structure */
typedef struct {
        irb_ble_hdr_t   hdr;
        ble_error_t     status;
} irb_ble_gap_address_set_rsp_t;

void irb_ble_handler_gap_address_set_cmd(OS_IRB *irb);

/** GAP device name set command message structure */
typedef struct {
        irb_ble_hdr_t   hdr;
        const char      *name;
        att_perm_t      perm;
} irb_ble_gap_device_name_set_cmd_t;

/** GAP device name set response message structure */
typedef struct {
        irb_ble_hdr_t   hdr;
        ble_error_t     status;
} irb_ble_gap_device_name_set_rsp_t;

void irb_ble_handler_gap_device_name_set_cmd(OS_IRB *irb);

/** GAP appearance set command message structure */
typedef struct {
        irb_ble_hdr_t     hdr;
        gap_appearance_t  appearance;
        att_perm_t        perm;
} irb_ble_gap_appearance_set_cmd_t;

/** GAP appearance set response message structure */
typedef struct {
        irb_ble_hdr_t   hdr;
        ble_error_t     status;
} irb_ble_gap_appearance_set_rsp_t;

void irb_ble_handler_gap_appearance_set_cmd(OS_IRB *irb);

/** GAP peripheral preferred connection parameters set command message structure */
typedef struct {
        irb_ble_hdr_t            hdr;
        const gap_conn_params_t  *gap_ppcp;
} irb_ble_gap_ppcp_set_cmd_t;

/** GAP peripheral preferred connection parameters set response message structure */
typedef struct {
        irb_ble_hdr_t   hdr;
        ble_error_t     status;
} irb_ble_gap_ppcp_set_rsp_t;

void irb_ble_handler_gap_ppcp_set_cmd(OS_IRB *irb);

/** GAP advertising start command message structure */
typedef struct {
        irb_ble_hdr_t   hdr;
        gap_conn_mode_t adv_type;
} irb_ble_gap_adv_start_cmd_t;

/** GAP advertising start response message structure */
typedef struct {
        irb_ble_hdr_t   hdr;
        ad_ble_status_t status;
} irb_ble_gap_adv_start_rsp_t;

void irb_ble_handler_gap_adv_start_cmd(OS_IRB *irb);

/** GAP advertising stop command message structure */
typedef struct {
        irb_ble_hdr_t   hdr;
} irb_ble_gap_adv_stop_cmd_t;

/** GAP advertising stop response message structure */
typedef struct {
        irb_ble_hdr_t   hdr;
        ad_ble_status_t status;
} irb_ble_gap_adv_stop_rsp_t;

void irb_ble_handler_gap_adv_stop_cmd(OS_IRB *irb);

/** GAP set advertising data command message structure */
typedef struct {
        irb_ble_hdr_t   hdr;
        uint8_t         adv_data_len;
        const uint8_t   *adv_data;
        uint8_t         scan_rsp_data_len;
        const uint8_t   *scan_rsp_data;
} irb_ble_gap_adv_data_set_cmd_t;

/** GAP set advertising data response message structure */
typedef struct {
        irb_ble_hdr_t   hdr;
        ble_error_t     status;
} irb_ble_gap_adv_data_set_rsp_t;

void irb_ble_handler_gap_adv_data_set_cmd(OS_IRB *irb);

/** GAP scan start command message structure */
typedef struct {
        irb_ble_hdr_t    hdr;
        gap_scan_type_t  type;
        gap_scan_mode_t  mode;
        uint16_t         interval;
        uint16_t         window;
        bool             filt_wlist;
        bool             filt_dupl;
} irb_ble_gap_scan_start_cmd_t;

/** GAP scan start response message structure */
typedef struct {
        irb_ble_hdr_t   hdr;
        ble_error_t     status;
} irb_ble_gap_scan_start_rsp_t;

void irb_ble_handler_gap_scan_start_cmd(OS_IRB *irb);

/** GAP scan stop command message structure */
typedef struct {
        irb_ble_hdr_t    hdr;
} irb_ble_gap_scan_stop_cmd_t;

/** GAP scan stop response message structure */
typedef struct {
        irb_ble_hdr_t   hdr;
        ble_error_t     status;
} irb_ble_gap_scan_stop_rsp_t;

void irb_ble_handler_gap_scan_stop_cmd(OS_IRB *irb);

/** GAP connect command message structure */
typedef struct {
        irb_ble_hdr_t           hdr;
        const bd_address_t      *peer_addr;
        const gap_conn_params_t *conn_params;
} irb_ble_gap_connect_cmd_t;

/** GAP connect response message structure */
typedef struct {
        irb_ble_hdr_t   hdr;
        ble_error_t     status;
} irb_ble_gap_connect_rsp_t;

void irb_ble_handler_gap_connect_cmd(OS_IRB *irb);

/** GAP connect cancel command message structure */
typedef struct {
        irb_ble_hdr_t   hdr;
} irb_ble_gap_connect_cancel_cmd_t;

/** GAP connect cancel response message structure */
typedef struct {
        irb_ble_hdr_t   hdr;
        ble_error_t     status;
} irb_ble_gap_connect_cancel_rsp_t;

void irb_ble_handler_gap_connect_cancel_cmd(OS_IRB *irb);

/** GAP disconnect command message structure */
typedef struct {
        irb_ble_hdr_t   hdr;
        uint16_t        conn_idx;
        ble_hci_error_t reason;
} irb_ble_gap_disconnect_cmd_t;

/** GAP disconnect response message structure */
typedef struct {
        irb_ble_hdr_t   hdr;
        ble_error_t     status;
} irb_ble_gap_disconnect_rsp_t;

void irb_ble_handler_gap_disconnect_cmd(OS_IRB *irb);

/** GAP get connection RSSI command message structure */
typedef struct {
        irb_ble_hdr_t   hdr;
        uint16_t        conn_idx;
} irb_ble_gap_conn_rssi_get_cmd_t;

/** GAP get connection RSSI response message structure */
typedef struct {
        irb_ble_hdr_t   hdr;
        int8_t          conn_rssi;
        ble_error_t     status;
} irb_ble_gap_conn_rssi_get_rsp_t;

void irb_ble_handler_gap_conn_rssi_get_cmd(OS_IRB *irb);

/** GAP set role command message structure */
typedef struct {
        irb_ble_hdr_t   hdr;
        gap_role_t      role;
} irb_ble_gap_role_set_cmd_t;

/** GAP set role response message structure */
typedef struct {
        irb_ble_hdr_t   hdr;
        gap_role_t      new_role;
        gap_role_t      previous_role;
        ble_error_t     status;
} irb_ble_gap_role_set_rsp_t;

void irb_ble_handler_gap_role_set_cmd(OS_IRB *irb);

/** GAP set MTU size command message structure */
typedef struct {
        irb_ble_hdr_t   hdr;
        uint16_t        mtu_size;
} irb_ble_gap_mtu_size_set_cmd_t;

/** GAP set MTU size response message structure */
typedef struct {
        irb_ble_hdr_t   hdr;
        uint16_t        new_mtu_size;
        uint16_t        previous_mtu_size;
        ble_error_t     status;
} irb_ble_gap_mtu_size_set_rsp_t;

void irb_ble_handler_gap_mtu_size_set_cmd(OS_IRB *irb);

/** GAP set channel map command message structure */
typedef struct {
        irb_ble_hdr_t   hdr;
        const uint64_t  *chnl_map;
} irb_ble_gap_channel_map_set_cmd_t;

/** GAP set channel map response message structure */
typedef struct {
        irb_ble_hdr_t   hdr;
        ble_error_t     status;
} irb_ble_gap_channel_map_set_rsp_t;

void irb_ble_handler_gap_channel_map_set_cmd(OS_IRB *irb);

/** GAP connection parameter update command message */
typedef struct {
        irb_ble_hdr_t            hdr;
        uint16_t                 conn_idx;
        const gap_conn_params_t  *conn_params;
} irb_ble_gap_conn_param_update_cmd_t;

/** GAP connection parameter update response message */
typedef struct {
        irb_ble_hdr_t   hdr;
        ble_error_t     status;
} irb_ble_gap_conn_param_update_rsp_t;

void irb_ble_handler_gap_conn_param_update_cmd(OS_IRB *irb);

/** GAP connection parameter update reply command message */
typedef struct {
        irb_ble_hdr_t   hdr;
        uint16_t        conn_idx;
        bool            accept;
} irb_ble_gap_conn_param_update_reply_cmd_t;

/** GAP connection parameter update reply response message */
typedef struct {
        irb_ble_hdr_t   hdr;
        ble_error_t     status;
} irb_ble_gap_conn_param_update_reply_rsp_t;

void irb_ble_handler_gap_conn_param_update_reply_cmd(OS_IRB *irb);

/** GAP pair command message */
typedef struct {
        irb_ble_hdr_t   hdr;
        uint16_t        conn_idx;
        bool            bond;
} irb_ble_gap_pair_cmd_t;

/** GAP pair response message */
typedef struct {
        irb_ble_hdr_t   hdr;
        ble_error_t     status;
} irb_ble_gap_pair_rsp_t;

void irb_ble_handler_gap_pair_cmd(OS_IRB *irb);

/** GAP pair reply command message */
typedef struct {
        irb_ble_hdr_t   hdr;
        uint16_t        conn_idx;
        bool            accept;
        bool            bond;
} irb_ble_gap_pair_reply_cmd_t;

/** GAP pair reply response message */
typedef struct {
        irb_ble_hdr_t   hdr;
        ble_error_t     status;
} irb_ble_gap_pair_reply_rsp_t;

void irb_ble_handler_gap_pair_reply_cmd(OS_IRB *irb);

/** GAP passkey reply command message */
typedef struct {
        irb_ble_hdr_t   hdr;
        uint16_t        conn_idx;
        bool            accept;
        uint32_t        passkey;
} irb_ble_gap_passkey_reply_cmd_t;

/** GAP passkey reply response message */
typedef struct {
        irb_ble_hdr_t   hdr;
        ble_error_t     status;
} irb_ble_gap_passkey_reply_rsp_t;

void irb_ble_handler_gap_passkey_reply_cmd(OS_IRB *irb);

#if (dg_configBLE_SECURE_CONNECTIONS == 1)
/** GAP numeric comparison reply command message */
typedef struct {
        irb_ble_hdr_t   hdr;
        uint16_t        conn_idx;
        bool            accept;
} irb_ble_gap_numeric_reply_cmd_t;

/** GAP numeric comparison reply response message */
typedef struct {
        irb_ble_hdr_t   hdr;
        ble_error_t     status;
} irb_ble_gap_numeric_reply_rsp_t;

void irb_ble_handler_gap_numeric_reply_cmd(OS_IRB *irb);
#endif /* (dg_configBLE_SECURE_CONNECTIONS == 1) */

/** GAP unpair command message */
typedef struct {
        irb_ble_hdr_t   hdr;
        bd_address_t    addr;
} irb_ble_gap_unpair_cmd_t;

/** GAP unpair response message */
typedef struct {
        irb_ble_hdr_t   hdr;
        ble_error_t     status;
} irb_ble_gap_unpair_rsp_t;

void irb_ble_handler_gap_unpair_cmd(OS_IRB *irb);

/** GAP get security level command message */
typedef struct {
        irb_ble_hdr_t   hdr;
        uint16_t        conn_idx;
        gap_sec_level_t level;
} irb_ble_gap_set_sec_level_cmd_t;

/** GAP get security level response message */
typedef struct {
        irb_ble_hdr_t   hdr;
        ble_error_t     status;
} irb_ble_gap_set_sec_level_rsp_t;

void irb_ble_handler_gap_set_sec_level_cmd(OS_IRB *irb);

#if (dg_configBLE_SKIP_LATENCY_API == 1)
/** GAP skip latency command message */
typedef struct {
        irb_ble_hdr_t   hdr;
        uint16_t        conn_idx;
        bool            enable;
} irb_ble_gap_skip_latency_cmd_t;

/** GAP skip latency response message */
typedef struct {
        irb_ble_hdr_t   hdr;
        ble_error_t     status;
} irb_ble_gap_skip_latency_rsp_t;

void irb_ble_handler_gap_skip_latency_cmd(OS_IRB *irb);
#endif /* (dg_configBLE_SKIP_LATENCY_API == 1) */

/** GAP data length set command message */
typedef struct {
        irb_ble_hdr_t   hdr;
        uint16_t        conn_idx;
        uint16_t        tx_length;
        uint16_t        tx_time;
} irb_ble_gap_data_length_set_cmd_t;

/** GAP data length set response message */
typedef struct {
        irb_ble_hdr_t   hdr;
        ble_error_t     status;
} irb_ble_gap_data_length_set_rsp_t;

void irb_ble_handler_gap_data_length_set_cmd(OS_IRB *irb);

void irb_ble_handler_gap_dev_bdaddr_ind_evt(ble_gtl_msg_t *gtl);

void irb_ble_handler_gap_adv_report_evt(ble_gtl_msg_t *gtl);

void irb_ble_handler_gap_connected_evt(ble_gtl_msg_t *gtl);

void irb_ble_handler_gap_get_device_info_req_evt(ble_gtl_msg_t *gtl);

void irb_ble_handler_gap_set_device_info_req_evt(ble_gtl_msg_t *gtl);

void irb_ble_handler_gap_disconnected_evt(ble_gtl_msg_t *gtl);

void irb_ble_handler_gap_peer_version_ind_evt(ble_gtl_msg_t *gtl);

void irb_ble_handler_gap_peer_features_ind_evt(ble_gtl_msg_t *gtl);

void irb_ble_handler_gap_conn_param_update_req_evt(ble_gtl_msg_t *gtl);

void irb_ble_handler_gap_conn_param_updated_evt(ble_gtl_msg_t *gtl);

void irb_ble_handler_gapm_adv_cmp_evt(ble_gtl_msg_t *gtl);

void irb_ble_handler_gapm_scan_cmp_evt(ble_gtl_msg_t *gtl);

void irb_ble_handler_gapm_connect_cmp_evt(ble_gtl_msg_t *gtl);

void irb_ble_handler_gap_bond_req_evt(ble_gtl_msg_t *gtl);

void irb_ble_handler_gap_bond_ind_evt(ble_gtl_msg_t *gtl);

void irb_ble_handler_gap_security_ind_evt(ble_gtl_msg_t *gtl);

void irb_ble_handler_gap_sign_counter_ind_evt(ble_gtl_msg_t *gtl);

void irb_ble_handler_gap_encrypt_req_ind_evt(ble_gtl_msg_t *gtl);

void irb_ble_handler_gapc_cmp__disconnect_evt(ble_gtl_msg_t *gtl);

void irb_ble_handler_gapc_cmp__update_params_evt(ble_gtl_msg_t *gtl);

void irb_ble_handler_gapc_cmp__bond_evt(ble_gtl_msg_t *gtl);

void irb_ble_handler_gapc_cmp__encrypt_evt(ble_gtl_msg_t *gtl);

void irb_ble_handler_gap_encrypt_ind_evt(ble_gtl_msg_t *gtl);

void irb_ble_handler_gap_addr_solved_evt(ble_gtl_msg_t *gtl);

void irb_ble_handler_gap_le_pkt_size_ind_evt(ble_gtl_msg_t *gtl);

void irb_ble_handler_gap_cmp__data_length_set_evt(ble_gtl_msg_t *gtl);

#endif /* BLE_MGR_IRB_GAP_H_ */
/**
 \}
 \}
 \}
 */
