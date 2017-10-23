/**
 ****************************************************************************************
 *
 * @file ble_l2cap.c
 *
 * @brief BLE L2CAP Connection Oriented Channels API implementation
 *
 * Copyright (C) 2015. Dialog Semiconductor, unpublished work. This computer
 * program includes Confidential, Proprietary Information and is a Trade Secret of
 * Dialog Semiconductor.  All use, disclosure, and/or reproduction is prohibited
 * unless authorized in writing. All Rights Reserved.
 *
 * <black.orca.support@diasemi.com> and contributors.
 *
 ****************************************************************************************
 */

#include "FreeRTOS.h"
#include <stdint.h>
#include <string.h>
#include "ble_common.h"
#include "osal.h"
#include "ble_gap.h"
#include "ble_irb_helper.h"
#include "ble_l2cap.h"
#include "ble_mgr.h"
#include "ble_mgr_irb_l2cap.h"
#include "storage.h"

#if (dg_configBLE_L2CAP_COC == 1)
ble_error_t ble_l2cap_listen(uint16_t conn_idx, uint16_t psm, gap_sec_level_t sec_level,
                                                        uint16_t initial_credits, uint16_t *scid)
{
        irb_ble_l2cap_listen_cmd_t *cmd;
        irb_ble_l2cap_listen_rsp_t *rsp;
        ble_error_t ret = BLE_ERROR_FAILED;

        /* setup IRB with new message and fill it */
        cmd = alloc_ble_msg(IRB_BLE_L2CAP_LISTEN_CMD, sizeof(*cmd));
        cmd->conn_idx = conn_idx;
        cmd->psm = psm;
        cmd->sec_level = sec_level;
        cmd->initial_credits = initial_credits;

        if (!irb_execute(cmd, (void **) &rsp, irb_ble_handler_l2cap_listen_cmd)) {
                return BLE_ERROR_FAILED;
        }

        ret = rsp->status;

        if (ret == BLE_STATUS_OK && scid) {
                *scid = rsp->scid;
        }

        OS_FREE(rsp);

        return ret;
}

ble_error_t ble_l2cap_stop_listen(uint16_t conn_idx, uint16_t cid)
{
        irb_ble_l2cap_stop_listen_cmd_t *cmd;
        irb_ble_l2cap_stop_listen_rsp_t *rsp;
        ble_error_t ret = BLE_ERROR_FAILED;

        /* setup IRB with new message and fill it */
        cmd = alloc_ble_msg(IRB_BLE_L2CAP_STOP_LISTEN_CMD, sizeof(*cmd));
        cmd->conn_idx = conn_idx;
        cmd->scid = cid;

        if (!irb_execute(cmd, (void **) &rsp, irb_ble_handler_l2cap_stop_listen_cmd)) {
                return BLE_ERROR_FAILED;
        }

        ret = rsp->status;

        OS_FREE(rsp);

        return ret;
}

ble_error_t ble_l2cap_connect(uint16_t conn_idx, uint16_t psm, uint16_t initial_credits,
                                                                                uint16_t *scid)
{
        irb_ble_l2cap_connect_cmd_t *cmd;
        irb_ble_l2cap_connect_rsp_t *rsp;
        ble_error_t ret = BLE_ERROR_FAILED;

        /* setup IRB with new message and fill it */
        cmd = alloc_ble_msg(IRB_BLE_L2CAP_CONNECT_CMD, sizeof(*cmd));
        cmd->conn_idx = conn_idx;
        cmd->psm = psm;
        cmd->initial_credits = initial_credits;

        if (!irb_execute(cmd, (void **) &rsp, irb_ble_handler_l2cap_connect_cmd)) {
                return BLE_ERROR_FAILED;
        }

        ret = rsp->status;

        if (ret == BLE_STATUS_OK && scid) {
                *scid = rsp->scid;
        }

        OS_FREE(rsp);

        return ret;
}

ble_error_t ble_l2cap_disconnect(uint16_t conn_idx, uint16_t scid)
{
        irb_ble_l2cap_disconnect_cmd_t *cmd;
        irb_ble_l2cap_disconnect_rsp_t *rsp;
        ble_error_t ret = BLE_ERROR_FAILED;

        /* setup IRB with new message and fill it */
        cmd = alloc_ble_msg(IRB_BLE_L2CAP_DISCONNECT_CMD, sizeof(*cmd));
        cmd->conn_idx = conn_idx;
        cmd->scid = scid;

        if (!irb_execute(cmd, (void **) &rsp, irb_ble_handler_l2cap_disconnect_cmd)) {
                return BLE_ERROR_FAILED;
        }

        ret = rsp->status;

        OS_FREE(rsp);

        return ret;
}

ble_error_t ble_l2cap_add_credits(uint16_t conn_idx, uint16_t scid, uint16_t credits)
{
        irb_ble_l2cap_add_credits_cmd_t *cmd;
        irb_ble_l2cap_add_credits_rsp_t *rsp;
        ble_error_t ret = BLE_ERROR_FAILED;

        /* setup IRB with new message and fill it */
        cmd = alloc_ble_msg(IRB_BLE_L2CAP_ADD_CREDITS_CMD, sizeof(*cmd));
        cmd->conn_idx = conn_idx;
        cmd->scid = scid;
        cmd->credits = credits;

        if (!irb_execute(cmd, (void **) &rsp, irb_ble_handler_l2cap_add_credits_cmd)) {
                return BLE_ERROR_FAILED;
        }

        ret = rsp->status;

        OS_FREE(rsp);

        return ret;
}

ble_error_t ble_l2cap_send(uint16_t conn_idx, uint16_t cid, uint16_t length, const void *data)
{
        irb_ble_l2cap_send_cmd_t *cmd;
        irb_ble_l2cap_send_rsp_t *rsp;
        ble_error_t ret = BLE_ERROR_FAILED;

        /* setup IRB with new message and fill it */
        cmd = alloc_ble_msg(IRB_BLE_L2CAP_SEND_CMD, sizeof(*cmd) + length);
        cmd->conn_idx = conn_idx;
        cmd->scid = cid;
        cmd->length = length;
        memcpy(cmd->data, data, length);

        if (!irb_execute(cmd, (void **) &rsp, irb_ble_handler_l2cap_send_cmd)) {
                return BLE_ERROR_FAILED;
        }

        ret = rsp->status;

        OS_FREE(rsp);

        return ret;
}
#endif /* (dg_configBLE_L2CAP_COC == 1) */

ble_error_t ble_l2cap_conn_param_update(uint16_t conn_idx, const gap_conn_params_t *conn_params)
{
        /* Only ble_gap_conn_param_update() is supported (Deprecated). */

        return BLE_ERROR_FAILED;
}
