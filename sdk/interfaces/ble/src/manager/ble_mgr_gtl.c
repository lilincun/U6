/**
 ****************************************************************************************
 *
 * @file ble_mgr_gtl.c
 *
 * @brief Helper library for GTL handling in BLE Manager
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

#include <string.h>
#include "FreeRTOS.h"
#include "co_version.h"
#include "ble_mgr.h"
#include "ble_mgr_config.h"
#include "ble_mgr_gtl.h"
#include "ble_mgr_irb_common.h"
#include "ble_mgr_irb_gap.h"
#include "ble_mgr_irb_gatts.h"
#include "ble_mgr_irb_gattc.h"
#include "ble_mgr_irb_l2cap.h"

#include "gapc_task.h"
#include "gapm_task.h"
#include "gattc_task.h"
#include "l2cc_task.h"

#define WAITQUEUE_MAXLEN (5)

typedef struct {
        uint16_t                conn_idx;
        uint16_t                msg_id;
        uint16_t                ext_id;
        ble_gtl_waitqueue_cb_t  cb;
        OS_IRB                  *irb;
} waitqueue_element_t;

PRIVILEGED_DATA static struct {
        waitqueue_element_t     queue[WAITQUEUE_MAXLEN];
        uint8_t                 len;
} waitqueue;

void *ble_hci_alloc(uint8_t hci_msg_type, uint16_t len)
{
        irb_ble_stack_msg_t *blemsg = NULL;

        if((hci_msg_type > 0) && (hci_msg_type <= BLE_HCI_EVT_MSG)) {
                blemsg = OS_MALLOC(sizeof(irb_ble_stack_msg_t) + len - sizeof(uint8_t));
        }
        else {
                goto done;
        }

        blemsg->op_code = IRB_BLE_STACK_MSG;
        blemsg->msg_type = hci_msg_type;
        switch(hci_msg_type) {
        case BLE_HCI_CMD_MSG:
                blemsg->msg_size = HCI_CMD_HEADER_LENGTH + len;
                break;
        case BLE_HCI_ACL_MSG:
                blemsg->msg_size = HCI_ACL_HEADER_LENGTH + len;
                break;
        case BLE_HCI_SCO_MSG:
                blemsg->msg_size = HCI_SCO_HEADER_LENGTH + len;
                break;
        case BLE_HCI_EVT_MSG:
                blemsg->msg_size = HCI_EVT_HEADER_LENGTH + len;
                break;
        }

        memset(blemsg->msg.gtl.param, 0, len);
done:
        return blemsg;
}

void *ble_gtl_alloc(uint16_t msg_id, uint16_t dest_id, uint16_t len)
{
        irb_ble_stack_msg_t *blemsg = OS_MALLOC(sizeof(irb_ble_stack_msg_t) + len - sizeof(uint32_t));

        blemsg->op_code = IRB_BLE_STACK_MSG;
        blemsg->msg_type = BLE_GTL_MSG;
        blemsg->msg_size = GTL_MSG_HEADER_LENGTH + len;
        blemsg->msg.gtl.msg_id = msg_id;
        blemsg->msg.gtl.dest_id = dest_id;
        blemsg->msg.gtl.src_id = TASK_ID_GTL;
        blemsg->msg.gtl.param_length = len;

        memset(blemsg->msg.gtl.param, 0, len);

        return blemsg;
}

void ble_gtl_waitqueue_add(uint16_t conn_idx, uint16_t msg_id, uint16_t ext_id,
                                                             ble_gtl_waitqueue_cb_t cb, OS_IRB *irb)
{
        waitqueue_element_t *elem;

#if (BLE_MGR_DIRECT_ACCESS == 1)
        /* Acquire the waitqueue. */
        ble_mgr_waitqueue_acquire();
#endif /* (BLE_MGR_DIRECT_ACCESS == 1) */

        /* There should be still room in the queue before calling this function */
        OS_ASSERT(waitqueue.len < WAITQUEUE_MAXLEN);

        elem = &waitqueue.queue[waitqueue.len++];

        elem->conn_idx = conn_idx;
        elem->msg_id = msg_id;
        elem->ext_id = ext_id;
        elem->cb = cb;
        elem->irb = OS_MALLOC(sizeof(OS_IRB));
        memcpy(elem->irb, irb, sizeof(OS_IRB));

#if (BLE_MGR_DIRECT_ACCESS == 1)
        /* Release the waitqueue. */
        ble_mgr_waitqueue_release();
#endif /* (BLE_MGR_DIRECT_ACCESS == 1) */
}

bool ble_gtl_waitqueue_match(ble_gtl_msg_t *gtl)
{
        uint8_t idx;
        bool ret = false;

#if (BLE_MGR_DIRECT_ACCESS == 1)
        /* Acquire the waitqueue. */
        ble_mgr_waitqueue_acquire();
#endif /* (BLE_MGR_DIRECT_ACCESS == 1) */

        for (idx = 0; idx < waitqueue.len; idx++) {
                bool match = true;
                waitqueue_element_t *elem = &waitqueue.queue[idx];

                match = (elem->conn_idx == TASK_2_CONNIDX(gtl->src_id)) && (elem->msg_id == gtl->msg_id);
                if (!match) {
                        continue;
                }

                switch (elem->msg_id) {
                case GAPM_CMP_EVT:
                {
                        struct gapm_cmp_evt *evt = (void *) gtl->param;
                        match = (evt->operation == elem->ext_id);
                        break;
                }
                case GAPC_CMP_EVT:
                {
                        struct gapc_cmp_evt *evt = (void *) gtl->param;
                        match = (evt->operation == elem->ext_id);
                        break;
                }
                /* Add more events if other commands need more fine-grained matching */
                }

                if (match) {
                        ble_gtl_waitqueue_cb_t cb = elem->cb;
                        OS_IRB *irb = elem->irb;

                        /* Remove from queue by moving remaining elements up in queue */
                        waitqueue.len--;
                        memmove(elem, elem + 1, sizeof(waitqueue_element_t) * (waitqueue.len - idx));

                        /* Fire associated callback */
                        cb(gtl, irb);

                        /* Free allocated IRB */
                        OS_FREE(irb);

                        ret = true;

                        break;
                }
        }

#if (BLE_MGR_DIRECT_ACCESS == 1)
        /* Release the waitqueue. */
        ble_mgr_waitqueue_release();
#endif /* (BLE_MGR_DIRECT_ACCESS == 1) */

        return ret;
}

void ble_gtl_waitqueue_flush(uint16_t conn_idx)
{
        uint8_t idx;

#if (BLE_MGR_DIRECT_ACCESS == 1)
        /* Acquire the waitqueue. */
        ble_mgr_waitqueue_acquire();
#endif /* (BLE_MGR_DIRECT_ACCESS == 1) */

        for (idx = 0; idx < waitqueue.len; idx++) {
                bool match = false;
                waitqueue_element_t *elem = &waitqueue.queue[idx];

                match = (elem->conn_idx == conn_idx);
                if (!match) {
                        continue;
                }

                switch(elem->msg_id) {
                case GAPC_CMP_EVT:
                        if (elem->ext_id == GAPC_ENCRYPT) {
                                match = true;
                        }
                        break;
                case GAPC_CON_RSSI_IND:
                        match = true;
                        break;
                }

                if (match) {
                        ble_gtl_waitqueue_cb_t cb = elem->cb;
                        OS_IRB *irb = elem->irb;

                        /* Remove from queue by moving remaining elements up in queue */
                        waitqueue.len--;
                        memmove(elem, elem + 1, sizeof(waitqueue_element_t) * (waitqueue.len - idx));

                        /* Fire associated callback with NULL gtl pointer */
                        cb(NULL, irb);

                        /* Free allocated IRB */
                        OS_FREE(irb);
                }
        }

#if (BLE_MGR_DIRECT_ACCESS == 1)
        /* Release the waitqueue. */
        ble_mgr_waitqueue_release();
#endif /* (BLE_MGR_DIRECT_ACCESS == 1) */
}

static bool ble_gtl_handle_gapm_cmp_evt(ble_gtl_msg_t *gtl)
{
        struct gapm_cmp_evt *gevt = (void *) gtl->param;

        switch (gevt->operation) {
#if (dg_configBLE_PERIPHERAL == 1) || (dg_configBLE_BROADCASTER == 1)
        case GAPM_ADV_NON_CONN:
        case GAPM_ADV_UNDIRECT:
        case GAPM_ADV_DIRECT:
        case GAPM_ADV_DIRECT_LDC:
                irb_ble_handler_gapm_adv_cmp_evt(gtl);
                break;
        case GAPM_UPDATE_ADVERTISE_DATA:
                break;
#endif /* (dg_configBLE_PERIPHERAL == 1) || (dg_configBLE_BROADCASTER == 1) */
#if (dg_configBLE_CENTRAL == 1) || (dg_configBLE_OBSERVER == 1)
        case GAPM_SCAN_ACTIVE:
        case GAPM_SCAN_PASSIVE:
                irb_ble_handler_gapm_scan_cmp_evt(gtl);
                break;
#endif /* (dg_configBLE_CENTRAL == 1) || (dg_configBLE_OBSERVER == 1) */
#if (dg_configBLE_CENTRAL == 1)
        case GAPM_CONNECTION_DIRECT:
                irb_ble_handler_gapm_connect_cmp_evt(gtl);
                break;
        case GAPM_SET_CHANNEL_MAP:
                break;
#endif /* (dg_configBLE_CENTRAL == 1) */
#if (dg_configBLE_CENTRAL == 1) || (dg_configBLE_PERIPHERAL == 1)
        case GAPM_SET_SUGGESTED_DFLT_LE_DATA_LEN:
                irb_ble_handler_gap_cmp__data_length_set_evt(gtl);
#endif /* (dg_configBLE_CENTRAL == 1) || (dg_configBLE_PERIPHERAL == 1) */
                break;
        case GAPM_RESET:
        case GAPM_CANCEL:
        case GAPM_SET_DEV_CONFIG:
        case GAPM_GET_DEV_VERSION:
        case GAPM_GET_DEV_BDADDR:
        case GAPM_RESOLV_ADDR:
                break;
        default:
                return false;
        }

       return true;
}

static bool ble_gtl_handle_gapc_cmp_evt(ble_gtl_msg_t *gtl)
{
        struct gapc_cmp_evt *gevt = (void *) gtl->param;

        switch (gevt->operation) {
#if (dg_configBLE_CENTRAL == 1) || (dg_configBLE_PERIPHERAL == 1)
        case GAPC_DISCONNECT:
                irb_ble_handler_gapc_cmp__disconnect_evt(gtl);
                break;
        case GAPC_UPDATE_PARAMS:
                irb_ble_handler_gapc_cmp__update_params_evt(gtl);
                break;
        case GAPC_SET_LE_PKT_SIZE:
                irb_ble_handler_gap_cmp__data_length_set_evt(gtl);
                break;
        case GAPC_GET_PEER_VERSION:
        case GAPC_GET_PEER_FEATURES:
        case GAPC_GET_CON_RSSI:
                break;
#endif /* (dg_configBLE_CENTRAL == 1) || (dg_configBLE_PERIPHERAL == 1) */
#if (dg_configBLE_CENTRAL == 1)
        case GAPC_BOND:
                irb_ble_handler_gapc_cmp__bond_evt(gtl);
                break;
#endif /* (dg_configBLE_CENTRAL == 1) */
#if (dg_configBLE_PERIPHERAL == 1)
        case GAPC_SECURITY_REQ:
                break;
#endif /* (dg_configBLE_PERIPHERAL == 1) */
#if (dg_configBLE_L2CAP_COC == 1)
        case GAPC_LE_CB_CONNECTION:
                irb_ble_handler_gapc_cmp__le_cb_connection_evt(gtl);
                break;
#endif /* (dg_configBLE_L2CAP_COC == 1) */

        default:
                return false;
        }

       return true;
}

static bool ble_gtl_handle_gattc_cmp_evt(ble_gtl_msg_t *gtl)
{
        struct gattc_cmp_evt *gevt = (void *) gtl->param;

        switch (gevt->operation) {
#if (dg_configBLE_GATT_SERVER == 1)
        case GATTC_NOTIFY:
        case GATTC_INDICATE:
                irb_ble_handler_gatts_event_sent_evt(gtl);
                break;
        case GATTC_SVC_CHANGED:
                break;
#endif /* (dg_configBLE_GATT_SERVER == 1) */
#if (dg_configBLE_GATT_CLIENT == 1)
        case GATTC_SDP_DISC_SVC:
        case GATTC_SDP_DISC_SVC_ALL:
                irb_ble_handler_gattc_cmp__browse_evt(gtl);
                break;
        case GATTC_DISC_BY_UUID_SVC:
        case GATTC_DISC_BY_UUID_CHAR:
        case GATTC_DISC_ALL_SVC:
        case GATTC_DISC_ALL_CHAR:
        case GATTC_DISC_DESC_CHAR:
        case GATTC_DISC_INCLUDED_SVC:
                irb_ble_handler_gattc_cmp__discovery_evt(gtl);
                break;
        case GATTC_READ:
                irb_ble_handler_gattc_cmp__read_evt(gtl);
                break;
        case GATTC_WRITE:
        case GATTC_WRITE_NO_RESPONSE:
        case GATTC_EXEC_WRITE:
                irb_ble_handler_gattc_cmp__write_evt(gtl);
                break;
#endif /* (dg_configBLE_GATT_CLIENT == 1) */
#if (dg_configBLE_GATT_SERVER == 1) || (dg_configBLE_GATT_CLIENT == 1)
        case GATTC_MTU_EXCH:
                break;
#endif /* (dg_configBLE_GATT_SERVER == 1) || (dg_configBLE_GATT_CLIENT == 1) */
        default:
                return false;
        }

       return true;
}

bool ble_gtl_handle_event(ble_gtl_msg_t *gtl)
{
        switch (gtl->msg_id) {
        /* Complete events */
        case GAPM_CMP_EVT:
                return ble_gtl_handle_gapm_cmp_evt(gtl);
        case GAPC_CMP_EVT:
                return ble_gtl_handle_gapc_cmp_evt(gtl);
        case GATTC_CMP_EVT:
                return ble_gtl_handle_gattc_cmp_evt(gtl);

        /* GAPM events */
        case GAPM_DEV_BDADDR_IND:
                irb_ble_handler_gap_dev_bdaddr_ind_evt(gtl);
                break;
#if (dg_configBLE_CENTRAL == 1) || (dg_configBLE_OBSERVER == 1)
        case GAPM_ADV_REPORT_IND:
                irb_ble_handler_gap_adv_report_evt(gtl);
                break;
#endif /* (dg_configBLE_CENTRAL == 1) || (dg_configBLE_OBSERVER == 1) */
        case GAPM_ADDR_SOLVED_IND:
                irb_ble_handler_gap_addr_solved_evt(gtl);
                break;

        /* GAPC events */
#if (dg_configBLE_CENTRAL == 1) || (dg_configBLE_PERIPHERAL == 1)
        case GAPC_CONNECTION_REQ_IND:
                irb_ble_handler_gap_connected_evt(gtl);
                break;
        case GAPC_DISCONNECT_IND:
                irb_ble_handler_gap_disconnected_evt(gtl);
                break;
        case GAPC_PEER_VERSION_IND:
                irb_ble_handler_gap_peer_version_ind_evt(gtl);
                break;
        case GAPC_PEER_FEATURES_IND:
                irb_ble_handler_gap_peer_features_ind_evt(gtl);
                break;
#endif /* #if (dg_configBLE_CENTRAL == 1) || (dg_configBLE_PERIPHERAL == 1) */
        case GAPC_GET_DEV_INFO_REQ_IND:
                irb_ble_handler_gap_get_device_info_req_evt(gtl);
                break;
        case GAPC_SET_DEV_INFO_REQ_IND:
                irb_ble_handler_gap_set_device_info_req_evt(gtl);
                break;
#if (dg_configBLE_CENTRAL == 1) || (dg_configBLE_PERIPHERAL == 1)
        case GAPC_PARAM_UPDATE_REQ_IND:
                irb_ble_handler_gap_conn_param_update_req_evt(gtl);
                break;
        case GAPC_PARAM_UPDATED_IND:
                irb_ble_handler_gap_conn_param_updated_evt(gtl);
                break;
        case GAPC_BOND_REQ_IND:
                irb_ble_handler_gap_bond_req_evt(gtl);
                break;
        case GAPC_BOND_IND:
                irb_ble_handler_gap_bond_ind_evt(gtl);
                break;
        case GAPC_ENCRYPT_IND:
                irb_ble_handler_gap_encrypt_ind_evt(gtl);
                break;
        case GAPC_LE_PKT_SIZE_IND:
                irb_ble_handler_gap_le_pkt_size_ind_evt(gtl);
                break;
#endif /* #if (dg_configBLE_CENTRAL == 1) || (dg_configBLE_PERIPHERAL == 1) */
#if (dg_configBLE_CENTRAL == 1)
        case GAPC_SECURITY_IND:
                irb_ble_handler_gap_security_ind_evt(gtl);
                break;
#endif /* (dg_configBLE_CENTRAL == 1) */
        case GAPC_SIGN_COUNTER_IND:
                irb_ble_handler_gap_sign_counter_ind_evt(gtl);
                break;
#if (dg_configBLE_PERIPHERAL == 1)
        case GAPC_ENCRYPT_REQ_IND:
                irb_ble_handler_gap_encrypt_req_ind_evt(gtl);
                break;
#endif /* (dg_configBLE_PERIPHERAL == 1) */
#if (dg_configBLE_L2CAP_COC == 1)
        case GAPC_LECB_CONNECT_IND:
                irb_ble_handler_l2cap_connect_ind_evt(gtl);
                break;
        case GAPC_LECB_DISCONNECT_IND:
                irb_ble_handler_l2cap_disconnect_ind_evt(gtl);
                break;
        case GAPC_LECB_CONNECT_REQ_IND:
                irb_ble_handler_l2cap_connect_req_ind_evt(gtl);
                break;
        case GAPC_LECB_ADD_IND:
                irb_ble_handler_l2cap_add_ind_evt(gtl);
                break;
#endif /* (dg_configBLE_L2CAP_COC == 1) */

        /* GATTC events */
#if (dg_configBLE_GATT_SERVER == 1)
        case GATTC_READ_REQ_IND:
                irb_ble_handler_gatts_read_value_req_evt(gtl);
                break;
        case GATTC_WRITE_REQ_IND:
                irb_ble_handler_gatts_write_value_req_evt(gtl);
                break;
        case GATTC_ATT_INFO_REQ_IND:
                irb_ble_handler_gatts_prepare_write_req_evt(gtl);
                break;
        case GATTC_SVC_CHANGED_CFG_IND:
                irb_ble_handler_gattc_svc_changed_cfg_ind_evt(gtl);
                break;
#endif /* (dg_configBLE_GATT_SERVER == 1) */
        case GATTC_MTU_CHANGED_IND:
                irb_ble_handler_gattc_mtu_changed_ind_evt(gtl);
                break;
#if (dg_configBLE_GATT_CLIENT == 1)
        case GATTC_SDP_SVC_IND:
                irb_ble_handler_gattc_sdp_svc_ind_evt(gtl);
                break;
        case GATTC_DISC_SVC_IND:
                irb_ble_handler_gattc_disc_svc_ind_evt(gtl);
                break;
        case GATTC_DISC_SVC_INCL_IND:
                irb_ble_handler_gattc_disc_svc_incl_ind_evt(gtl);
                break;
        case GATTC_DISC_CHAR_IND:
                irb_ble_handler_gattc_disc_char_ind_evt(gtl);
                break;
        case GATTC_DISC_CHAR_DESC_IND:
                irb_ble_handler_gattc_disc_char_desc_ind_evt(gtl);
                break;
        case GATTC_READ_IND:
                irb_ble_handler_gattc_read_ind_evt(gtl);
                break;
        case GATTC_EVENT_IND:
                irb_ble_handler_gattc_event_ind_evt(gtl);
                break;
        case GATTC_EVENT_REQ_IND:
                irb_ble_handler_gattc_event_req_ind_evt(gtl);
                break;
#endif /* (dg_configBLE_GATT_CLIENT == 1) */
#if (dg_configBLE_GATT_SERVER == 1) || (dg_configBLE_GATT_CLIENT == 1)
        case GATTC_TRANSACTION_TO_ERROR_IND:
                break;
#endif
#if (dg_configBLE_L2CAP_COC == 1)
        // L2CC events:
        case L2CC_PDU_SEND_RSP:
                irb_ble_handler_l2cap_pdu_send_rsp_evt(gtl);
                break;
        case L2CC_LECNX_DATA_RECV_IND:
                irb_ble_handler_l2cap_lecnx_data_recv_ind_evt(gtl);
                break;
#endif /* (dg_configBLE_L2CAP_COC == 1) */
        default:
                return false;
        }

        return true;
}
