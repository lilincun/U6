/**
 ****************************************************************************************
 *
 * @file ble_suota_task.c
 *
 * @brief BLE SUOTA Demo task
 *
 * Copyright (C) 2016. Dialog Semiconductor, unpublished work. This computer
 * program includes Confidential, Proprietary Information and is a Trade Secret of
 * Dialog Semiconductor.  All use, disclosure, and/or reproduction is prohibited
 * unless authorized in writing. All Rights Reserved.
 *
 * <black.orca.support@diasemi.com> and contributors.
 *
 ****************************************************************************************
 */

#include "osal.h"
#include "ble_common.h"
#include "ble_service.h"
#include "dis.h"
#include "dlg_suota.h"
#include "ble_l2cap.h"
#include "ble_gattc.h"

/*
 * Support for multiple concurrent connections
 *
 * If set to zero (default), application will restart advertising upon disconnection of existing
 * client making possible for only one client being connected at given time.
 * If set to non-zero, application will restart advertising immediately after it was stopped due
 * to incoming connected. This allows for multiple clients to be connected at the same time.
 */
#define CFG_MULTIPLE_CLIENTS (0)

/*
 * HRP advertising and scan response data
 *
 * As per HRP specification, sensor device should include HRS UUID in advertising data and local name
 * in either advertising data or scan response.
 */
static const uint8_t adv_data[] = {
        0x03, GAP_DATA_TYPE_UUID16_LIST_INC,
        0xF5, 0xFE, // = 0xFEF5 (DUALOG SUOTA UUID)
};

static const uint8_t scan_rsp[] = {
        0x0D, GAP_DATA_TYPE_LOCAL_NAME,
        'D', 'i', 'a', 'l', 'o', 'g', ' ', 'S', 'U', 'O', 'T', 'A'
};

/*
 * Device Information Service data
 *
 * Manufacturer Name String is mandatory for devices supporting HRP.
 */
static const dis_device_info_t dis_info = {
        .manufacturer = "Dialog Semiconductor",
        .model_number = "Dialog BLE",
        .serial_number = "123456",
        .hw_revision = "Rev. D",
        .fw_revision = "1.0",
        .sw_revision = "BL " "1.3",
};

/* Callback from SUOTA implementation */
static void suota_status_changed_cb(uint8_t status, uint8_t error_code);
static bool suota_ready_cb(void);

static const suota_callbacks_t suota_cb = {
        .suota_ready = suota_ready_cb,
        .suota_status = suota_status_changed_cb,
};

#if !CFG_MULTIPLE_CLIENTS
static void handle_evt_gap_disconnected(ble_evt_gap_disconnected_t *evt)
{
        ble_gap_adv_start(GAP_CONN_MODE_UNDIRECTED);
}
#else
static void handle_evt_gap_adv_completed(ble_evt_gap_adv_completed_t *evt)
{
        ble_gap_adv_start(GAP_CONN_MODE_UNDIRECTED);
}
#endif

static bool suota_ready_cb(void)
{
        /*
         * This callback is used so application can accept/block SUOTA.
         * Also, before SUOTA starts, user might want to do some actions
         * e.g. disable sleep mode.
         *
         * If true is returned, then advertising is stopped and SUOTA
         * is started. Otherwise SUOTA is canceled.
         *
         */
        return true;
}

static void suota_status_changed_cb(uint8_t status, uint8_t error_code)
{
}

void ble_suota_task(void *params)
{
        ble_service_t *suota;

        ble_peripheral_start();
        ble_register_app();

        /* Set maximum allowed MTU to increase SUOTA throughput */
        ble_gap_mtu_size_set(512);

        /*
         * Register SUOTA
         *
         * SUOTA instance should be registered in ble_service framework in order for events inside
         * service to be processed properly.
         */

        suota = suota_init(&suota_cb);
        /* Make sure suota is properly initialized */
        configASSERT(suota != NULL);
        ble_service_add(suota);

        /*
         * Register DIS
         *
         * DIS doesn't contain any dynamic data thus it doesn't need to be registered in ble_service
         * framework (but it's not an error to do so).
         */
        dis_init(NULL, &dis_info);

        /*
         * Set advertising data and scan response, then start advertising.
         */
        ble_gap_adv_data_set(sizeof(adv_data), adv_data, sizeof(scan_rsp), scan_rsp);
        ble_gap_adv_start(GAP_CONN_MODE_UNDIRECTED);

        for (;;) {
                BaseType_t ret;
                uint32_t notif;

                ret = OS_TASK_NOTIFY_WAIT(0, (uint32_t) -1, &notif, OS_TASK_NOTIFY_FOREVER);
                /* Guarantted to succeed since we're waiting forever for the notification */
                OS_ASSERT(ret == OS_TASK_NOTIFY_SUCCESS);

                /* Notified from BLE Manager? */
                if (notif & BLE_APP_NOTIFY_MASK) {
                        ble_evt_hdr_t *hdr;

                        /*
                         * No need to wait for event, should be already there since we were notified
                         * from manager.
                         */
                        hdr = ble_get_event(false);
                        if (!hdr) {
                                continue;
                        }

                        /*
                         * First, application needs to try pass event through ble_framework.
                         * Then it can handle it itself and finally pass to default event handler.
                         */
                        if (!ble_service_handle_event(hdr)) {
                                switch (hdr->evt_code) {
#if !CFG_MULTIPLE_CLIENTS
                                case BLE_EVT_GAP_DISCONNECTED:
                                        handle_evt_gap_disconnected((ble_evt_gap_disconnected_t *) hdr);
                                        break;
#else
                                case BLE_EVT_GAP_ADV_COMPLETED:
                                        handle_evt_gap_adv_completed((ble_evt_gap_adv_completed_t *) hdr);
                                        break;
#endif
#ifdef SUOTA_PSM
                                case BLE_EVT_L2CAP_CONNECTED:
                                case BLE_EVT_L2CAP_DISCONNECTED:
                                case BLE_EVT_L2CAP_DATA_IND:
                                        suota_l2cap_event(suota, hdr);
                                        break;
#endif
                                default:
                                        ble_handle_event_default(hdr);
                                        break;
                                }
                        }

                        /* Free event buffer (it's not needed anymore) */
                        OS_FREE(hdr);

                        /*
                         * If there are more events waiting in queue, application should process
                         * them now.
                         */
                        if (ble_has_event()) {
                                OS_TASK_NOTIFY(OS_GET_CURRENT_TASK(), BLE_APP_NOTIFY_MASK, OS_NOTIFY_SET_BITS);
                        }
                }
        }
}

