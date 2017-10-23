/**
 ****************************************************************************************
 *
 * @file ble_gatts.c
 *
 * @brief tbd
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
#include "ble_mgr_irb_gatts.h"
#include "ble_irb_helper.h"
#include "ble_common.h"
#include "ble_gatts.h"

#if (dg_configBLE_GATT_SERVER == 1)
ble_error_t ble_gatts_add_service(const att_uuid_t *uuid, const gatt_service_t type, uint16_t num_attrs)
{
        irb_ble_gatts_service_add_cmd_t *cmd;
        irb_ble_gatts_service_add_rsp_t *rsp;
        ble_error_t ret = BLE_ERROR_FAILED;

        /* setup IRB with new message and fill it */
        cmd = alloc_ble_msg(IRB_BLE_GATTS_SERVICE_ADD_CMD, sizeof(*cmd));
        cmd->uuid = *uuid;
        cmd->type = type;
        cmd->num_attrs = num_attrs;

        if (!irb_execute(cmd, (void **) &rsp, irb_ble_handler_gatts_service_add_cmd)) {
                return BLE_ERROR_FAILED;
        }

        ret = rsp->status;
        OS_FREE(rsp);

        return ret;
}

ble_error_t ble_gatts_add_include(uint16_t handle, uint16_t *h_offset)
{
        irb_ble_gatts_service_add_include_cmd_t *cmd;
        irb_ble_gatts_service_add_include_rsp_t *rsp;
        ble_error_t ret = BLE_ERROR_FAILED;

        /* setup IRB with new message and fill it */
        cmd = alloc_ble_msg(IRB_BLE_GATTS_SERVICE_INCLUDE_ADD_CMD, sizeof(*cmd));
        cmd->handle = handle;

        if (!irb_execute(cmd, (void **) &rsp, irb_ble_handler_gatts_service_add_include_cmd)) {
                return ret;
        }

        ret = rsp->status;

        if (h_offset) {
                *h_offset = rsp->h_offset;
        }

        /* free message */
        OS_FREE(rsp);

        return ret;
}

ble_error_t ble_gatts_add_characteristic(const att_uuid_t *uuid, gatt_prop_t prop, att_perm_t perm,
                                                        uint16_t max_len, gatts_flag_t flags,
                                                        uint16_t *h_offset, uint16_t *h_val_offset)
{
        irb_ble_gatts_service_add_characteristic_cmd_t *cmd;
        irb_ble_gatts_service_add_characteristic_rsp_t *rsp;
        ble_error_t ret = BLE_ERROR_FAILED;

        /* setup IRB with new message and fill it */
        cmd = alloc_ble_msg(IRB_BLE_GATTS_SERVICE_CHARACTERISTIC_ADD_CMD, sizeof(*cmd));
        cmd->uuid = *uuid;
        cmd->prop = prop;
        cmd->perm = perm;
        cmd->max_len = max_len;
        cmd->flags = flags;

        if (!irb_execute(cmd, (void **) &rsp, irb_ble_handler_gatts_service_add_characteristic_cmd)) {
                return ret;
        }

        ret = rsp->status;

        if (h_offset) {
                *h_offset = rsp->h_offset;
        }

        if (h_val_offset) {
                *h_val_offset = rsp->h_val_offset;
        }

        /* free message */
        OS_FREE(rsp);

        return ret;
}

ble_error_t ble_gatts_add_descriptor(const att_uuid_t *uuid, att_perm_t perm, uint16_t max_len,
                                                        gatts_flag_t flags, uint16_t *h_offset)
{
        irb_ble_gatts_service_add_descriptor_cmd_t *cmd;
        irb_ble_gatts_service_add_descriptor_rsp_t *rsp;
        ble_error_t ret = BLE_ERROR_FAILED;

        /* setup IRB with new message and fill it */
        cmd = alloc_ble_msg(IRB_BLE_GATTS_SERVICE_DESCRIPTOR_ADD_CMD, sizeof(*cmd));
        cmd->uuid = *uuid;
        cmd->perm = perm;
        cmd->max_len = max_len;
        cmd->flags = flags;

        if (!irb_execute(cmd, (void **) &rsp, irb_ble_handler_gatts_service_add_descriptor_cmd)) {
                return ret;
        }

        ret = rsp->status;

        if (h_offset) {
                *h_offset = rsp->h_offset;
        }

        /* free message */
        OS_FREE(rsp);

        return ret;
}

ble_error_t ble_gatts_register_service(uint16_t *handle, ...)
{
        va_list ap;
        irb_ble_gatts_service_register_cmd_t *cmd;
        irb_ble_gatts_service_register_rsp_t *rsp;
        ble_error_t ret = BLE_ERROR_FAILED;

        /* setup IRB with new message and fill it */
        cmd = alloc_ble_msg(IRB_BLE_GATTS_SERVICE_REGISTER_CMD, sizeof(*cmd));

        if (!irb_execute(cmd, (void **) &rsp, irb_ble_handler_gatts_service_register_cmd)) {
                return ret;
        }

        ret = rsp->status;

        if (handle) {
                *handle = rsp->handle;
        }

        va_start(ap, handle);
        for (;;) {
                uint16_t *h = va_arg(ap, uint16_t*);

                if (!h) {
                        break;
                }

                *h = *h + rsp->handle;
        }
        va_end(ap);

        /* free message */
        OS_FREE(rsp);

        return ret;
}

ble_error_t ble_gatts_enable_service(uint16_t handle)
{
        irb_ble_gatts_service_enable_cmd_t *cmd;
        irb_ble_gatts_service_enable_rsp_t *rsp;
        ble_error_t ret = BLE_ERROR_FAILED;

        /* setup IRB with new message and fill it */
        cmd = alloc_ble_msg(IRB_BLE_GATTS_SERVICE_ENABLE_CMD, sizeof(*cmd));
        cmd->handle = handle;

        if (!irb_execute(cmd, (void **) &rsp, irb_ble_handler_gatts_service_enable_cmd)) {
                return ret;
        }

        ret = rsp->status;

        /* free message */
        OS_FREE(rsp);

        return ret;
}

ble_error_t ble_gatts_disable_service(uint16_t handle)
{
        irb_ble_gatts_service_disable_cmd_t *cmd;
        irb_ble_gatts_service_disable_rsp_t *rsp;
        ble_error_t ret = BLE_ERROR_FAILED;

        /* setup IRB with new message and fill it */
        cmd = alloc_ble_msg(IRB_BLE_GATTS_SERVICE_DISABLE_CMD, sizeof(*cmd));
        cmd->handle = handle;

        if (!irb_execute(cmd, (void **) &rsp, irb_ble_handler_gatts_service_disable_cmd)) {
                return ret;
        }

        ret = rsp->status;

        /* free message */
        OS_FREE(rsp);

        return ret;
}

ble_error_t ble_gatts_get_characteristic_prop(uint16_t handle, gatt_prop_t *prop, att_perm_t *perm)
{
        irb_ble_gatts_service_characteristic_get_prop_cmd_t *cmd;
        irb_ble_gatts_service_characteristic_get_prop_rsp_t *rsp;
        ble_error_t ret = BLE_ERROR_FAILED;

        /* setup IRB with new message and fill it */
        cmd = alloc_ble_msg(IRB_BLE_GATTS_SERVICE_CHARACTERISTIC_GET_PROP_CMD, sizeof(*cmd));
        cmd->handle = handle;

        if (!irb_execute(cmd, (void **) &rsp,
                                       irb_ble_handler_gatts_service_characteristic_get_prop_cmd)) {
                return ret;
        }

        ret = rsp->status;

        if (ret == BLE_STATUS_OK) {
                if (prop) {
                        *prop = rsp->prop;
                }

                if (perm) {
                        *perm = rsp->perm;
                }
        }

        /* free message */
        OS_FREE(rsp);

        return ret;
}

ble_error_t ble_gatts_set_characteristic_prop(uint16_t handle, gatt_prop_t prop, att_perm_t perm)
{
        irb_ble_gatts_service_characteristic_set_prop_cmd_t *cmd;
        irb_ble_gatts_service_characteristic_set_prop_rsp_t *rsp;
        ble_error_t ret = BLE_ERROR_FAILED;

        /* setup IRB with new message and fill it */
        cmd = alloc_ble_msg(IRB_BLE_GATTS_SERVICE_CHARACTERISTIC_SET_PROP_CMD, sizeof(*cmd));
        cmd->handle = handle;
        cmd->prop = prop;
        cmd->perm = perm;

        if (!irb_execute(cmd, (void **) &rsp,
                                       irb_ble_handler_gatts_service_characteristic_set_prop_cmd)) {
                return ret;
        }

        ret = rsp->status;

        /* free message */
        OS_FREE(rsp);

        return ret;
}

ble_error_t ble_gatts_get_value(uint16_t handle, uint16_t *length, void *value)
{
        irb_ble_gatts_get_value_cmd_t *cmd;
        irb_ble_gatts_get_value_rsp_t *rsp;
        ble_error_t ret = BLE_ERROR_FAILED;
        uint16_t copy_length;

        /* setup IRB with new message and fill it */
        cmd = alloc_ble_msg(IRB_BLE_GATTS_GET_VALUE_CMD, sizeof(*cmd));
        cmd->handle = handle;
        cmd->max_len = *length;
        copy_length = *length; // save for later

        if (!irb_execute(cmd, (void **) &rsp, irb_ble_handler_gatts_get_value_cmd)) {
                return ret;
        }

        /*
         * Update length with actual attribute value length.
         * Copy as much data as will fit in buffer or what was returned (if buffer provided).
         */
        *length = rsp->length;
        if (value) {
                if (rsp->length < copy_length) {
                        copy_length = rsp->length;
                }
                memcpy(value, rsp->value, copy_length);
        }

        ret = rsp->status;
        /* free message */
        OS_FREE(rsp);

        return ret;
}

ble_error_t ble_gatts_set_value(uint16_t handle, uint16_t length, const void *value)
{
        irb_ble_gatts_set_value_cmd_t *cmd;
        irb_ble_gatts_set_value_rsp_t *rsp;
        ble_error_t ret = BLE_ERROR_FAILED;

        /* setup IRB with new message and fill it */
        cmd = alloc_ble_msg(IRB_BLE_GATTS_SET_VALUE_CMD, sizeof(*cmd) + length);
        cmd->handle = handle;
        cmd->length = length;
        memcpy(cmd->value, value, length);

        if (!irb_execute(cmd, (void **) &rsp, irb_ble_handler_gatts_set_value_cmd)) {
                return BLE_ERROR_FAILED;
        }

        ret = rsp->status;
        OS_FREE(rsp);

        return ret;
}

ble_error_t ble_gatts_read_cfm(uint16_t conn_idx, uint16_t handle, att_error_t status, uint16_t length, const void *value)
{
        irb_ble_gatts_read_cfm_cmd_t *cmd;
        irb_ble_gatts_read_cfm_rsp_t *rsp;
        ble_error_t ret = BLE_ERROR_FAILED;

        if (length && !value) {
                return ret;
        }

        /* setup IRB with new message and fill it */
        cmd = alloc_ble_msg(IRB_BLE_GATTS_READ_CFM_CMD, sizeof(*cmd) + length);
        cmd->conn_idx = conn_idx;
        cmd->handle = handle;
        cmd->status = status;
        cmd->length = length;
        if (value) {
                memcpy(cmd->value, value, length);
        }

        if (!irb_execute(cmd, (void **) &rsp, irb_ble_handler_gatts_read_cfm_cmd)) {
                return BLE_ERROR_FAILED;
        }

        ret = rsp->status;
        OS_FREE(rsp);

        return ret;
}

ble_error_t ble_gatts_write_cfm(uint16_t conn_idx, uint16_t handle, att_error_t status)
{
        irb_ble_gatts_write_cfm_cmd_t *cmd;
        irb_ble_gatts_write_cfm_rsp_t *rsp;
        ble_error_t ret = BLE_ERROR_FAILED;

        /* setup IRB with new message and fill it */
        cmd = alloc_ble_msg(IRB_BLE_GATTS_WRITE_CFM_CMD, sizeof(*cmd));
        cmd->conn_idx = conn_idx;
        cmd->handle = handle;
        cmd->status = status;

        if (!irb_execute(cmd, (void **) &rsp, irb_ble_handler_gatts_write_cfm_cmd)) {
                return BLE_ERROR_FAILED;
        }

        ret = rsp->status;
        OS_FREE(rsp);

        return ret;
}

ble_error_t ble_gatts_prepare_write_cfm(uint16_t conn_idx, uint16_t handle, uint16_t length, att_error_t status)
{
        irb_ble_gatts_prepare_write_cfm_cmd_t *cmd;
        irb_ble_gatts_prepare_write_cfm_rsp_t *rsp;
        ble_error_t ret = BLE_ERROR_FAILED;

        /* setup IRB with new message and fill it */
        cmd = alloc_ble_msg(IRB_BLE_GATTS_PREPARE_WRITE_CFM_CMD, sizeof(*cmd));
        cmd->conn_idx = conn_idx;
        cmd->handle = handle;
        cmd->length = length;
        cmd->status = status;

        if (!irb_execute(cmd, (void **) &rsp, irb_ble_handler_gatts_prepare_write_cfm_cmd)) {
                return BLE_ERROR_FAILED;
        }

        ret = rsp->status;
        OS_FREE(rsp);

        return ret;
}

ble_error_t ble_gatts_send_event(uint16_t conn_idx, uint16_t handle, gatt_event_t type,
                                                                uint16_t length, const void *value)
{
        irb_ble_gatts_send_event_cmd_t *cmd;
        irb_ble_gatts_send_event_rsp_t *rsp;
        ble_error_t ret = BLE_ERROR_FAILED;

        /* setup IRB with new message and fill it */
        cmd = alloc_ble_msg(IRB_BLE_GATTS_SEND_EVENT_CMD, sizeof(*cmd) + length);
        cmd->conn_idx = conn_idx;
        cmd->handle = handle;
        cmd->type = type;
        cmd->length = length;
        memcpy(cmd->value, value, length);

        if (!irb_execute(cmd, (void **) &rsp, irb_ble_handler_gatts_send_event_cmd)) {
                return BLE_ERROR_FAILED;
        }

        ret = rsp->status;
        OS_FREE(rsp);

        return ret;
}

ble_error_t ble_gatts_service_changed_ind(uint16_t conn_idx, uint16_t start_handle,
                                                                        uint16_t end_handle)
{
        irb_ble_gatts_service_changed_ind_cmd_t *cmd;
        irb_ble_gatts_service_changed_ind_rsp_t *rsp;
        ble_error_t ret = BLE_ERROR_FAILED;

        /* setup IRB with new message and fill it */
        cmd = alloc_ble_msg(IRB_BLE_GATTS_SERVICE_CHANGED_IND_CMD, sizeof(*cmd));
        cmd->conn_idx = conn_idx;
        cmd->start_handle = start_handle;
        cmd->end_handle = end_handle;

        if (!irb_execute(cmd, (void **) &rsp, irb_ble_handler_gatts_service_changed_ind_cmd)) {
                return BLE_ERROR_FAILED;
        }

        ret = rsp->status;
        OS_FREE(rsp);

        return ret;
}
#endif /* (dg_configBLE_GATT_SERVER == 1) */
