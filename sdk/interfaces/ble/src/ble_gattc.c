/**
 ****************************************************************************************
 *
 * @file ble_gattc.c
 *
* @brief BLE GATT Client API
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

#include <stdarg.h>
#include <string.h>
#include "FreeRTOS.h"
#include "ble_mgr.h"
#include "ble_mgr_irb.h"
#include "ble_mgr_irb_gattc.h"
#include "ble_irb_helper.h"
#include "ble_common.h"
#include "ble_gattc.h"
#include "storage.h"

#if (dg_configBLE_GATT_CLIENT == 1)
ble_error_t ble_gattc_browse(uint16_t conn_idx, const att_uuid_t *uuid)
{
        irb_ble_gattc_browse_cmd_t *cmd;
        irb_ble_gattc_browse_rsp_t *rsp;
        ble_error_t ret = BLE_ERROR_FAILED;

        /* setup IRB with new message and fill it */
        cmd = alloc_ble_msg(IRB_BLE_GATTC_BROWSE_CMD, sizeof(*cmd));
        cmd->conn_idx = conn_idx;
        cmd->uuid = uuid;

        if (!irb_execute(cmd, (void **) &rsp, irb_ble_handler_gattc_browse_cmd)) {
                return BLE_ERROR_FAILED;
        }

        ret = rsp->status;

        OS_FREE(rsp);
        return ret;
}

ble_error_t ble_gattc_discover_svc(uint16_t conn_idx, const att_uuid_t *uuid)
{
        irb_ble_gattc_discover_svc_cmd_t *cmd;
        irb_ble_gattc_discover_svc_rsp_t *rsp;
        ble_error_t ret = BLE_ERROR_FAILED;

        /* setup IRB with new message and fill it */
        cmd = alloc_ble_msg(IRB_BLE_GATTC_DISCOVER_SVC_CMD, sizeof(*cmd));
        cmd->conn_idx = conn_idx;
        cmd->uuid = uuid;

        if (!irb_execute(cmd, (void **) &rsp, irb_ble_handler_gattc_discover_svc_cmd)) {
                return BLE_ERROR_FAILED;
        }

        ret = rsp->status;

        OS_FREE(rsp);
        return ret;
}

ble_error_t ble_gattc_discover_include(uint16_t conn_idx, uint16_t start_h, uint16_t end_h)
{
        irb_ble_gattc_discover_include_cmd_t *cmd;
        irb_ble_gattc_discover_include_rsp_t *rsp;
        ble_error_t ret = BLE_ERROR_FAILED;

        /* setup IRB with new message and fill it */
        cmd = alloc_ble_msg(IRB_BLE_GATTC_DISCOVER_INCLUDE_CMD, sizeof(*cmd));
        cmd->conn_idx = conn_idx;
        cmd->start_h = start_h;
        cmd->end_h = end_h;

        if (!irb_execute(cmd, (void **) &rsp, irb_ble_handler_gattc_discover_include_cmd)) {
                return BLE_ERROR_FAILED;
        }

        ret = rsp->status;

        OS_FREE(rsp);
        return ret;
}

ble_error_t ble_gattc_discover_char(uint16_t conn_idx, uint16_t start_h, uint16_t end_h,
                                                                        const att_uuid_t *uuid)
{
        irb_ble_gattc_discover_char_cmd_t *cmd;
        irb_ble_gattc_discover_char_rsp_t *rsp;
        ble_error_t ret = BLE_ERROR_FAILED;

        /* setup IRB with new message and fill it */
        cmd = alloc_ble_msg(IRB_BLE_GATTC_DISCOVER_CHAR_CMD, sizeof(*cmd));
        cmd->conn_idx = conn_idx;
        cmd->start_h = start_h;
        cmd->end_h = end_h;
        cmd->uuid = uuid;

        if (!irb_execute(cmd, (void **) &rsp, irb_ble_handler_gattc_discover_char_cmd)) {
                return BLE_ERROR_FAILED;
        }

        ret = rsp->status;

        OS_FREE(rsp);
        return ret;
}

ble_error_t ble_gattc_discover_desc(uint16_t conn_idx, uint16_t start_h, uint16_t end_h)
{
        irb_ble_gattc_discover_desc_cmd_t *cmd;
        irb_ble_gattc_discover_desc_rsp_t *rsp;
        ble_error_t ret = BLE_ERROR_FAILED;

        /* setup IRB with new message and fill it */
        cmd = alloc_ble_msg(IRB_BLE_GATTC_DISCOVER_DESC_CMD, sizeof(*cmd));
        cmd->conn_idx = conn_idx;
        cmd->start_h = start_h;
        cmd->end_h = end_h;

        if (!irb_execute(cmd, (void **) &rsp, irb_ble_handler_gattc_discover_desc_cmd)) {
                return BLE_ERROR_FAILED;
        }

        ret = rsp->status;

        OS_FREE(rsp);
        return ret;
}

ble_error_t ble_gattc_read(uint16_t conn_idx, uint16_t handle, uint16_t offset)
{
        irb_ble_gattc_read_cmd_t *cmd;
        irb_ble_gattc_read_rsp_t *rsp;
        ble_error_t ret = BLE_ERROR_FAILED;

        /* setup IRB with new message and fill it */
        cmd = alloc_ble_msg(IRB_BLE_GATTC_READ_CMD, sizeof(*cmd));
        cmd->conn_idx = conn_idx;
        cmd->handle = handle;
        cmd->offset = offset;

        if (!irb_execute(cmd, (void **) &rsp, irb_ble_handler_gattc_read_cmd)) {
                return BLE_ERROR_FAILED;
        }

        ret = rsp->status;

        OS_FREE(rsp);
        return ret;
}

ble_error_t ble_gattc_write(uint16_t conn_idx, uint16_t handle, uint16_t offset, uint16_t length,
                                                                        const uint8_t *value)
{
        irb_ble_gattc_write_generic_cmd_t *cmd;
        irb_ble_gattc_write_generic_rsp_t *rsp;
        ble_error_t ret = BLE_ERROR_FAILED;

        /* setup IRB with new message and fill it */
        cmd = alloc_ble_msg(IRB_BLE_GATTC_WRITE_GENERIC_CMD, sizeof(*cmd));
        cmd->conn_idx = conn_idx;
        cmd->handle = handle;
        cmd->offset = offset;
        cmd->length = length;
        cmd->value = value;

        if (!irb_execute(cmd, (void **) &rsp, irb_ble_handler_gattc_write_generic_cmd)) {
                return BLE_ERROR_FAILED;
        }

        ret = rsp->status;

        OS_FREE(rsp);
        return ret;
}

ble_error_t ble_gattc_write_no_resp(uint16_t conn_idx, uint16_t handle, bool signed_write,
                                                        uint16_t length, const uint8_t *value)
{
        irb_ble_gattc_write_generic_cmd_t *cmd;
        irb_ble_gattc_write_generic_rsp_t *rsp;
        ble_error_t ret = BLE_ERROR_FAILED;

        /* setup IRB with new message and fill it */
        cmd = alloc_ble_msg(IRB_BLE_GATTC_WRITE_GENERIC_CMD, sizeof(*cmd));
        cmd->conn_idx = conn_idx;
        cmd->handle = handle;
        cmd->no_response = true;
        cmd->signed_write = signed_write;
        cmd->length = length;
        cmd->value = value;

        if (!irb_execute(cmd, (void **) &rsp, irb_ble_handler_gattc_write_generic_cmd)) {
                return BLE_ERROR_FAILED;
        }

        ret = rsp->status;

        OS_FREE(rsp);
        return ret;
}

ble_error_t ble_gattc_write_prepare(uint16_t conn_idx, uint16_t handle, uint16_t offset,
                                                        uint16_t length, const uint8_t *value)
{
        irb_ble_gattc_write_generic_cmd_t *cmd;
        irb_ble_gattc_write_generic_rsp_t *rsp = NULL;
        ble_error_t ret = BLE_ERROR_FAILED;

        if (!length || !value) {
                goto done;
        }

        /* setup IRB with new message and fill it */
        cmd = alloc_ble_msg(IRB_BLE_GATTC_WRITE_GENERIC_CMD, sizeof(*cmd));
        cmd->conn_idx = conn_idx;
        cmd->handle = handle;
        cmd->prepare = true;
        cmd->offset = offset;
        cmd->length = length;
        cmd->value = value;

        if (!irb_execute(cmd, (void **) &rsp, irb_ble_handler_gattc_write_generic_cmd)) {
                goto done;
        }

        ret = rsp->status;

done:
        OS_FREE(rsp);
        return ret;
}

ble_error_t ble_gattc_write_execute(uint16_t conn_idx, bool commit)
{
        irb_ble_gattc_write_execute_cmd_t *cmd;
        irb_ble_gattc_write_execute_rsp_t *rsp = NULL;
        ble_error_t ret = BLE_ERROR_FAILED;

        /* setup IRB with new message and fill it */
        cmd = alloc_ble_msg(IRB_BLE_GATTC_WRITE_EXECUTE_CMD, sizeof(*cmd));
        cmd->conn_idx = conn_idx;
        cmd->commit = commit;

        if (!irb_execute(cmd, (void **) &rsp, irb_ble_handler_gattc_write_execute_cmd)) {
                goto done;
        }

        ret = rsp->status;

done:
        OS_FREE(rsp);
        return ret;
}

ble_error_t ble_gattc_indication_cfm(uint16_t conn_idx, uint16_t handle)
{
        return BLE_STATUS_OK;
}
#endif /* (dg_configBLE_GATT_CLIENT == 1) */

#if (dg_configBLE_GATT_CLIENT == 1) || (dg_configBLE_GATT_SERVER == 1)
ble_error_t ble_gattc_get_mtu(uint16_t conn_idx, uint16_t *mtu)
{
        device_t *dev;

        storage_acquire();

        dev = find_device_by_conn_idx(conn_idx);
        if (!dev) {
                storage_release();
                return BLE_ERROR_FAILED;
        }

        *mtu = dev->mtu;

        storage_release();

        return BLE_STATUS_OK;
}

ble_error_t ble_gattc_exchange_mtu(uint16_t conn_idx)
{
        irb_ble_gattc_exchange_mtu_cmd_t *cmd;
        irb_ble_gattc_exchange_mtu_rsp_t *rsp;
        ble_error_t ret = BLE_ERROR_FAILED;

        /* setup IRB with new message and fill it */
        cmd = alloc_ble_msg(IRB_BLE_GATTC_EXCHANGE_MTU_CMD, sizeof(*cmd));
        cmd->conn_idx = conn_idx;

        if (!irb_execute(cmd, (void **) &rsp, irb_ble_handler_gattc_exchange_mtu_cmd)) {
                return BLE_ERROR_FAILED;
        }

        ret = rsp->status;

        OS_FREE(rsp);
        return ret;
}
#endif /* (dg_configBLE_GATT_CLIENT == 1) || (dg_configBLE_GATT_SERVER == 1) */
