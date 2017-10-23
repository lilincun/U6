/**
 ****************************************************************************************
 *
 * @file sample.c
 *
 * @brief Serial Port Service sample implementation
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

#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include "osal.h"
#include "ble_storage.h"
#include "ble_bufops.h"
#include "ble_gatts.h"
#include "ble_uuid.h"
#include "svc_defines.h"
#include "sample.h"

//#define UUID_SAMPLE                "0783b03e-8535-b5a0-7140-a304d2495cb7"
#define UUID_SAMPLE                "0783b03e-8535-b5a0-7140-a304d2495cc7"
#define UUID_SAMPLE_SERVER_TX      "0783b03e-8535-b5a0-7140-a304d2495cc9"//read
#define UUID_SAMPLE_SERVER_RX      "0783b03e-8535-b5a0-7140-a304d2495cc8"//write
#define UUID_SAMPLE_FLOW_CTRL      "0783b03e-8535-b5a0-7140-a304d2495cca"//flow

static const char sample_tx_desc[] = "Server TX Data";
static const char sample_rx_desc[] = "Server RX Data";
static const char sample_flow_control_desc[] = "Flow Control";

/* Serial Port Service reference application: Table 1: Size of characteristics */
static const uint16_t sample_server_tx_size = 160;
static const uint16_t sample_server_rx_size = 160;

typedef struct {
        ble_service_t svc;

        sample_callbacks_t *cb;

        uint16_t sample_tx_val_h;
        uint16_t sample_tx_ccc_h;

        uint16_t sample_rx_val_h;

        uint16_t sample_flow_ctrl_val_h;
        uint16_t sample_flow_ctrl_ccc_h;
} sp_service_t;


#if 1    ////////////////////////////////////

#if 1    /////////
        static void notify_flow_ctrl(sp_service_t *sample, uint16_t conn_idx, sample_flow_control_t value)
        {
                uint8_t flow_ctrl = value;

                ble_gatts_send_event(conn_idx, sample->sample_flow_ctrl_val_h, GATT_EVENT_NOTIFICATION,
                                                                        sizeof(flow_ctrl), &flow_ctrl);
        }
        //handle_write_req
        static att_error_t handle_flow_ctrl_ccc_write(sp_service_t *sample,  uint16_t conn_idx,
                                                uint16_t offset, uint16_t length, const uint8_t *value)
        {
                uint16_t ccc;

                if (offset) {
                        return ATT_ERROR_ATTRIBUTE_NOT_LONG;
                }

                if (length != sizeof(ccc)) {
                        return ATT_ERROR_APPLICATION_ERROR;
                }

                ccc = get_u16(value);

                ble_storage_put_u32(conn_idx, sample->sample_flow_ctrl_ccc_h, ccc, true);

                /* Send notification if client enabled notifications */
                if (ccc & GATT_CCC_NOTIFICATIONS) {
                        uint8_t flow_ctrl = SAMPLE_FLOW_CONTROL_OFF;

                        ble_storage_get_u8(conn_idx, sample->sample_flow_ctrl_val_h, &flow_ctrl);

                        notify_flow_ctrl(sample, conn_idx, flow_ctrl);
                }

                return ATT_ERROR_OK;
        }
#endif   //////////
//handle_write_req
static att_error_t handle_tx_ccc_write(sp_service_t *sample,  uint16_t conn_idx,
                                        uint16_t offset, uint16_t length, const uint8_t *value)
{
        uint16_t ccc;

        if (offset) {
                return ATT_ERROR_ATTRIBUTE_NOT_LONG;
        }

        if (length != sizeof(ccc)) {
                return ATT_ERROR_APPLICATION_ERROR;
        }

        ccc = get_u16(value);

        ble_storage_put_u32(conn_idx, sample->sample_tx_ccc_h, ccc, true);

        return ATT_ERROR_OK;
}
//handle_write_req
static att_error_t set_flow_control_req(sp_service_t *sample, uint16_t conn_idx,
                                        uint16_t offset, uint16_t length, const uint8_t *value)
{
        if (offset) {
                return ATT_ERROR_ATTRIBUTE_NOT_LONG;
        }

        if (length != sizeof(uint8_t)) {
                return ATT_ERROR_INVALID_VALUE_LENGTH;
        }

        if (value[0] != SAMPLE_FLOW_CONTROL_ON && value[0] != SAMPLE_FLOW_CONTROL_OFF) {
                return ATT_ERROR_APPLICATION_ERROR;
        }

        if (sample->cb && sample->cb->set_flow_control) {
                sample->cb->set_flow_control(&sample->svc, conn_idx, value[0]);
        }

        return ATT_ERROR_OK;
}
//handle_write_req
static att_error_t handle_rx_data(sp_service_t *sample, uint16_t conn_idx,
                                        uint16_t offset, uint16_t length, const uint8_t *value)
{
        if (sample->cb && sample->cb->rx_data) {
                sample->cb->rx_data(&sample->svc, conn_idx, value, length);
        }

        return ATT_ERROR_OK;
}

static void handle_write_req(ble_service_t *svc, const ble_evt_gatts_write_req_t *evt)
{
        sp_service_t *sample = (sp_service_t *) svc;
        att_error_t status = ATT_ERROR_ATTRIBUTE_NOT_FOUND;
        uint16_t handle = evt->handle;

        if (handle == sample->sample_tx_ccc_h) {//c9   tx  notifi
                printf("c9 evt->value = %d\r\n",evt->value[0]);
                printf("c9 evt->value = %d\r\n",evt->value[1]);
                printf("c9 evt->length = %d\r\n",evt->length);
                status = handle_tx_ccc_write(sample, evt->conn_idx, evt->offset, evt->length, evt->value);
        }
        if (handle == sample->sample_flow_ctrl_ccc_h) {//ca
                printf("ca ccc evt->value = %d\r\n",evt->value[0]);
                printf("ca ccc evt->value = %d\r\n",evt->value[1]);
                printf("ca ccc evt->length = %d\r\n",evt->length);
                status = handle_flow_ctrl_ccc_write(sample, evt->conn_idx, evt->offset, evt->length, evt->value);
        }

        if (handle == sample->sample_flow_ctrl_val_h) {
                printf("ca evt->value = %d\r\n",evt->value[0]);
                printf("ca evt->value = %d\r\n",evt->value[1]);
                printf("ca evt->length = %d\r\n",evt->length);
                status = set_flow_control_req(sample, evt->conn_idx, evt->offset, evt->length, evt->value);
        }

        if (handle == sample->sample_rx_val_h) {//c8  rx  write
                printf("c8 evt->value = %d\r\n",evt->value[0]);
                printf("c8 evt->length = %d\r\n",evt->length);
                status = handle_rx_data(sample, evt->conn_idx, evt->offset, evt->length, evt->value);
        }

        ble_gatts_write_cfm(evt->conn_idx, evt->handle, status);
        printf("handle_write_req\r\n");
}
#endif//////////////////////////////////////////////

static void handle_read_req(ble_service_t *svc, const ble_evt_gatts_read_req_t *evt)
{
        sp_service_t *sample = (sp_service_t *) svc;

        if (evt->handle == sample->sample_flow_ctrl_ccc_h || evt->handle == sample->sample_tx_ccc_h) {
                //uint16_t ccc = 0x0000;
                uint16_t ccc = 0x1111;
                if (evt->handle == sample->sample_flow_ctrl_ccc_h)
                {
                        ble_storage_get_u16(evt->conn_idx, sample->sample_flow_ctrl_ccc_h, &ccc);
                }
                else//evt->handle == sample->sample_tx_ccc_h
                {
                        //ble_storage_get_u16(evt->conn_idx, sample->sample_tx_ccc_h, &ccc);
                }
                // we're little-endian, ok to write directly from uint16_t
                ble_gatts_read_cfm(evt->conn_idx, evt->handle, ATT_ERROR_OK, sizeof(ccc), &ccc);
        } else {
                ble_gatts_read_cfm(evt->conn_idx, evt->handle, ATT_ERROR_READ_NOT_PERMITTED, 0, NULL);
        }
        printf("handle_read_req\r\n");
}


#if 1  /////////////////////
static uint16_t get_tx_length(sp_service_t *sample, uint16_t conn_idx)
{
        uint16_t length = 0x00;

        ble_storage_get_u16(conn_idx, sample->sample_tx_val_h, &length);

        return length;
}

static void set_tx_length(sp_service_t *sample, uint16_t conn_idx, uint16_t length)
{
        ble_storage_put_u32(conn_idx, sample->sample_tx_val_h, length, false);
}
static void handle_event_sent(ble_service_t *svc, const ble_evt_gatts_event_sent_t *evt)
{
        sp_service_t *sample = (sp_service_t *) svc;
        uint16_t length, conn_idx = evt->conn_idx;

        if (evt->handle == sample->sample_tx_val_h) {
                length = get_tx_length(sample, conn_idx);
                set_tx_length(sample, conn_idx, 0x00);

                sample->cb->tx_done(&sample->svc, conn_idx, length);
        }
        printf("handle_event_sent\r\n");
}
#endif     ///////////////////////////



ble_service_t *sample_init(sample_callbacks_t *cb)
{
        uint16_t num_attr, sample_tx_desc_h, sample_rx_desc_h, sample_flow_ctrl_desc_h;
        sp_service_t *sample;
        att_uuid_t uuid;

        sample = OS_MALLOC(sizeof(*sample));
        memset(sample, 0, sizeof(*sample));

        num_attr = ble_gatts_get_num_attr(0, 3, 5);

        ble_uuid_from_string(UUID_SAMPLE, &uuid);
        //////////////////////////////////////////////////////////////////////
        //添加一个服务
        ble_gatts_add_service(&uuid, GATT_SERVICE_PRIMARY, num_attr);
#if 1
        /* SAMPLE Server TX *///c9
        ble_uuid_from_string(UUID_SAMPLE_SERVER_TX, &uuid);
        ble_gatts_add_characteristic(&uuid,
                                     GATT_PROP_NOTIFY ,
                                     //GATT_PROP_NOTIFY | GATT_PROP_READ,
                                     ATT_PERM_NONE ,
                                     //ATT_PERM_READ ,
                                     sample_server_tx_size,
                                     0, NULL,
                                     &sample->sample_tx_val_h);

        ble_uuid_create16(UUID_GATT_CLIENT_CHAR_CONFIGURATION, &uuid);//0x2902
        ble_gatts_add_descriptor(&uuid, ATT_PERM_RW, 2, 0, &sample->sample_tx_ccc_h);

        ble_uuid_create16(UUID_GATT_CHAR_USER_DESCRIPTION, &uuid);//0x2901///////////////
        ble_gatts_add_descriptor(&uuid, ATT_PERM_READ, sizeof(sample_tx_desc), 0, &sample_tx_desc_h);////////////
#endif

#if 1
        /* SAMPLE Server RX *///c8
        ble_uuid_from_string(UUID_SAMPLE_SERVER_RX, &uuid);
        ble_gatts_add_characteristic(&uuid,
                                     GATT_PROP_WRITE_NO_RESP,
                                     ATT_PERM_WRITE,
                                     sample_server_rx_size,
                                     0, NULL,
                                     &sample->sample_rx_val_h);

        ble_uuid_create16(UUID_GATT_CHAR_USER_DESCRIPTION, &uuid);//0x2901////////////////////////
        ble_gatts_add_descriptor(&uuid, ATT_PERM_READ, sizeof(sample_rx_desc), 0, &sample_rx_desc_h);/////////////////
#endif

#if 1
        /* SAMPLE Flow Control *///ca
        ble_uuid_from_string(UUID_SAMPLE_FLOW_CTRL, &uuid);
        ble_gatts_add_characteristic(&uuid,
                                     GATT_PROP_WRITE_NO_RESP | GATT_PROP_NOTIFY,
                                     ATT_PERM_WRITE,
                                     1,
                                     0, NULL,
                                     &sample->sample_flow_ctrl_val_h);

        ble_uuid_create16(UUID_GATT_CLIENT_CHAR_CONFIGURATION, &uuid);//0x2902
        ble_gatts_add_descriptor(&uuid, ATT_PERM_RW, 2, 0, &sample->sample_flow_ctrl_ccc_h);

        ble_uuid_create16(UUID_GATT_CHAR_USER_DESCRIPTION, &uuid);//0x2901/////////////////
        ble_gatts_add_descriptor(&uuid, ATT_PERM_READ, sizeof(sample_flow_control_desc), 0, &sample_flow_ctrl_desc_h);////////

#endif

        /* Register SAMPLE Service */
        ble_gatts_register_service(&sample->svc.start_h,

                                   &sample->sample_tx_val_h,
                                   &sample->sample_tx_ccc_h,//2
                                   &sample_tx_desc_h,
                                   &sample->sample_rx_val_h,
                                   &sample_rx_desc_h,
                                   &sample->sample_flow_ctrl_val_h,
                                   &sample->sample_flow_ctrl_ccc_h,//2
                                   &sample_flow_ctrl_desc_h,
                                   0);

        /* Set value of Characteristic Descriptions */
        ble_gatts_set_value(sample_tx_desc_h, sizeof(sample_tx_desc), sample_tx_desc);
        ble_gatts_set_value(sample_rx_desc_h, sizeof(sample_rx_desc), sample_rx_desc);
        ble_gatts_set_value(sample_flow_ctrl_desc_h, sizeof(sample_flow_control_desc),
                                                     sample_flow_control_desc);

        sample->svc.end_h = sample->svc.start_h + num_attr;
        sample->svc.write_req = handle_write_req;
        sample->svc.read_req = handle_read_req;
        sample->svc.event_sent = handle_event_sent;
        sample->cb = cb;

        return &sample->svc;
}

void sample_set_flow_control(ble_service_t *svc, uint16_t conn_idx, sample_flow_control_t value)
{
        sp_service_t *sample = (sp_service_t *) svc;
        uint16_t ccc = 0x0000;

        ble_storage_put_u32(conn_idx, sample->sample_flow_ctrl_val_h, value, false);

        ble_storage_get_u16(conn_idx, sample->sample_flow_ctrl_ccc_h, &ccc);
        if (!(ccc & GATT_CCC_NOTIFICATIONS)) {
                return;
        }

        notify_flow_ctrl(sample, conn_idx, value);
}

#if 1  ///////////////////
static int calculate_length(int length)
{
        return length > 20 ? 20 : length;
}

static bool send_tx_data(sp_service_t *sample, uint16_t conn_idx, uint16_t length, uint8_t *data)
{
        uint8_t status;

        status = ble_gatts_send_event(conn_idx, sample->sample_tx_val_h, GATT_EVENT_NOTIFICATION,
                                                                                length, data);

        return status == BLE_STATUS_OK ? true : false;
}
void sample_tx_data(ble_service_t *svc, uint16_t conn_idx, uint8_t *data, uint16_t length)
{
        sp_service_t *sample = (sp_service_t *) svc;
        uint16_t ccc = 0x0000;
        uint8_t flow_ctrl = SAMPLE_FLOW_CONTROL_OFF;

        if (get_tx_length(sample, conn_idx) != 0x00) {
                return;
        }

        if (!sample->cb->tx_done) {
                return;
        }

        /* Check if remote client registered for TX data */
        ble_storage_get_u16(conn_idx, sample->sample_tx_ccc_h, &ccc);
        if (!(ccc & GATT_CCC_NOTIFICATIONS)) {
                return;
        }

        /* Check if flow control is enabled */
        ble_storage_get_u8(conn_idx, sample->sample_flow_ctrl_val_h, &flow_ctrl);
        if (flow_ctrl != SAMPLE_FLOW_CONTROL_ON) {
                return;
        }

        length = calculate_length(length);

        if (send_tx_data(sample, conn_idx, length, data)) {
                set_tx_length(sample, conn_idx, length);
        }
}

#endif /////////////////////
