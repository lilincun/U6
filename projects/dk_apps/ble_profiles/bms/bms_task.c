/**
 ****************************************************************************************
 *
 * @file bms_task.c
 *
 * @brief BMS demo task
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
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <time.h>
#include <stdio.h>
#include "osal.h"
#include "timers.h"
#include "osal.h"
#include "sys_watchdog.h"
#include "ble_att.h"
#include "ble_common.h"
#include "ble_gap.h"
#include "ble_gatts.h"
#include "ble_service.h"
#include "ble_uuid.h"
#include "bms.h"
#include "util/queue.h"
#include "bms_config.h"

/*
 * BMS Delete all bond notif mask
 */
#define BMS_DELETE_ALL_BOND_NOTIF       (1 << 1)

/*
 * BMS Update connection parameters notif mask
 */
#define BMS_UPDATE_CONN_PARAM_NOTIF     (1 << 2)

/*
 * BMS Demo advertising data
 */
static const uint8_t adv_data[] = {
        0x10, GAP_DATA_TYPE_LOCAL_NAME,
        'D', 'i', 'a', 'l', 'o', 'g', ' ', 'B', 'M', 'S', ' ', 'D', 'e', 'm', 'o'
};

/*
 * TASK handle
 */
PRIVILEGED_DATA static OS_TASK current_task;

/*
 * Bond Management Service data
 */

PRIVILEGED_DATA static ble_service_t *bms;

static const bms_config_t bms_config = {
        .supported_delete_bond_op =     BMS_DELETE_BOND_REQ_DEV                 |
                                        BMS_DELETE_BOND_REQ_DEV_AUTH            |
                                        BMS_DELETE_BOND_ALL_DEV                 |
                                        BMS_DELETE_BOND_ALL_DEV_AUTH            |
                                        BMS_DELETE_BOND_ALL_EXCEPT_REQ_DEV      |
                                        BMS_DELETE_BOND_ALL_EXCEPT_REQ_DEV_AUTH,
};

typedef struct {
        void            *next;

        uint16_t        conn_idx; ///< Connection index
        bd_address_t    addr;     ///< Connected device
        bool            unpair;   ///< True if device should be unpaired when disconnected

        OS_TIMER        conn_pause_timer;
} conn_dev_t;

typedef struct {
        void            *next;

        uint16_t        conn_idx; ///< Connection index
} update_param_t;

static queue_t connections;;
static queue_t update_conn_param_q;

static void delete_bond_cb(bms_delete_bond_op_t op, uint16_t conn_idx, uint16_t length,
                                                                         const uint8_t *auth_code);

static const bms_callbacks_t bms_cb = {
        .delete_bond = delete_bond_cb,
};

/*
 * Debug functions
 */

static const char *format_bd_address(const bd_address_t *addr)
{
        static char buf[19];
        int i;

        for (i = 0; i < sizeof(addr->addr); i++) {
                int idx;

                // for printout, address should be reversed
                idx = sizeof(addr->addr) - i - 1;
                sprintf(&buf[i * 3], "%02X:", addr->addr[idx]);
        }

        buf[sizeof(buf) - 2] = '\0';

        return buf;
}

void print_connection_func(void *data, void *user_data)
{
        const conn_dev_t *conn_dev = data;
        int *num = user_data;

        (*num)++;

        printf("%2d | %5d | %17s | %d\r\n", *num, conn_dev->conn_idx,
                                                        format_bd_address(&conn_dev->addr),
                                                        conn_dev->unpair);
}

static void print_connections(void)
{
        int num = 0;

        printf("\r\n");
        printf("Active connections:\r\n");
        printf("Nr | Index | Address           | Pending unpair\r\n");
        queue_foreach(&connections, print_connection_func, &num);

        if (!num) {
                printf("(no active connections)\r\n");
        }

        printf("\r\n");
}

static void print_bonded_devices(void)
{
        uint8_t bonded_length;
        bd_address_t *addr_bonded = NULL;
        uint8_t i;

        ble_gap_get_bonded(&bonded_length, &addr_bonded);

        printf("\r\n");
        printf("Bonded devices:\r\n");
        printf("Nr | Address \r\n");

        if (!bonded_length) {
                printf("(no bonded devices)\r\n");
                printf("\r\n");
                return;
        }

        for (i = 0; i < bonded_length; ++i) {
                printf("%2d | %17s\r\n", (i + 1), format_bd_address(&addr_bonded[i]));
        }

        printf("\r\n");
        OS_FREE(addr_bonded);
}

static bool conn_dev_match_conn_idx(const void *elem, const void *ud)
{
        conn_dev_t *conn_dev = (conn_dev_t *) elem;
        uint16_t conn_idx = *((uint16_t *) ud);

        return conn_dev->conn_idx == conn_idx;
}

static bool conn_dev_match_addr(const void *elem, const void *ud)
{
        conn_dev_t *conn_dev = (conn_dev_t *) elem;
        bd_address_t *addr = (bd_address_t *) ud;

        return !memcmp(&conn_dev->addr, addr, sizeof(bd_address_t));
}

/*
 * BMS demo WKUP handler. It is called when user pressed the button.
 */
void bms_wkup_handler(void)
{
        if (current_task) {
                OS_TASK_NOTIFY_FROM_ISR(current_task, BMS_DELETE_ALL_BOND_NOTIF,
                                                                        OS_NOTIFY_SET_BITS);
        }
}

static void unpair(uint16_t skip_conn_idx, bool unpair_connected)
{
        conn_dev_t *conn_dev;
        uint8_t bonded_length;
        bd_address_t *addr_bonded = NULL;
        uint8_t i;
        bool show_bonded_devices = false;

        ble_gap_get_bonded(&bonded_length, &addr_bonded);

        if (!addr_bonded) {
                return;
        }

        for (i = 0; i < bonded_length; ++i) {
                conn_dev = queue_find(&connections, conn_dev_match_addr, &addr_bonded[i]);

                /* Unpair immediately if device is not connected */
                if (!conn_dev) {
                        ble_gap_unpair(&addr_bonded[i]);
                        show_bonded_devices = true;
                        continue;
                }

                /**
                 * If current connection index is equal skip_conn_idx, don't mark device
                 * for unpairing upon disconnection
                 * */
                if (conn_dev->conn_idx == skip_conn_idx) {
                        continue;
                }

                if (unpair_connected) {
                        /* Mark device for unpairing upon disconnection */
                        conn_dev->unpair = true;
                }
        }

        if (show_bonded_devices) {
                print_bonded_devices();
        }
}

static void unpair_all(uint16_t skip_conn_idx)
{
        unpair(skip_conn_idx, true);
}

static void unpair_disconnected(void)
{
        printf("Unpairing non-connected devices\r\n");

        unpair(BLE_CONN_IDX_INVALID, false);
}

static bool check_auth_code(uint16_t length, const uint8_t *auth_code)
{
        static const char demo_auth_code[] = CFG_AUTH_CODE;
        static size_t demo_auth_code_len = sizeof(demo_auth_code) - 1;

        if (length != demo_auth_code_len) {
                return false;
        }

        return !memcmp(auth_code, demo_auth_code, length);
}

static void delete_bond_cb(bms_delete_bond_op_t op, uint16_t conn_idx, uint16_t length,
                                                                          const uint8_t *auth_code)
{
        char auth_code_str[length + 1];
        conn_dev_t *conn_dev;
        bms_delete_bond_status_t status = BMS_DELETE_BOND_STATUS_OK;

        memcpy(auth_code_str, auth_code, length);
        auth_code_str[length] = '\0';

        printf("%s: conn_idx=%d op=%d auth_code=%s\r\n", __func__, conn_idx, op, auth_code_str);

        switch (op) {
        case BMS_DELETE_BOND_REQ_DEV_AUTH:
                if (!check_auth_code(length, auth_code)) {
                        status = BMS_DELETE_BOND_STATUS_INSUFFICIENT_AUTH;
                        break;
                }
                /* no break */
        case BMS_DELETE_BOND_REQ_DEV:
                /* Remove required bonded device */
                conn_dev = queue_find(&connections, conn_dev_match_conn_idx, &conn_idx);
                if (conn_dev) {
                        conn_dev->unpair = true;
                }
                break;

        case BMS_DELETE_BOND_ALL_DEV_AUTH:
                if (!check_auth_code(length, auth_code)) {
                        status = BMS_DELETE_BOND_STATUS_INSUFFICIENT_AUTH;
                        break;
                }
                /* no break */
        case BMS_DELETE_BOND_ALL_DEV:
                /* Remove all bonded device */
                unpair_all(BLE_CONN_IDX_INVALID);
                break;

        case BMS_DELETE_BOND_ALL_EXCEPT_REQ_DEV_AUTH:
                if (!check_auth_code(length, auth_code)) {
                        status = BMS_DELETE_BOND_STATUS_INSUFFICIENT_AUTH;
                        break;
                }
                /* no break */
        case BMS_DELETE_BOND_ALL_EXCEPT_REQ_DEV:
                /* Remove all bonded devices except current device */
                unpair_all(conn_idx);
                break;
        }

        bms_delete_bond_cfm(bms, conn_idx, status);

        print_connections();
}

static void update_conn_param(uint16_t conn_idx)
{
        gap_conn_params_t cp;
        ble_error_t ret;

        cp.interval_min = defaultBLE_PPCP_INTERVAL_MIN;
        cp.interval_max = defaultBLE_PPCP_INTERVAL_MAX;
        cp.slave_latency = defaultBLE_PPCP_SLAVE_LATENCY;
        cp.sup_timeout = defaultBLE_PPCP_SUP_TIMEOUT;

        ret = ble_gap_conn_param_update(conn_idx, &cp);
        if (ret != BLE_STATUS_OK) {
                printf("Failed to change connection parameters for conn_idx=%d (%d)\r\n",
                                                                                conn_idx, ret);
        }
}

static void dump_conn_param(const gap_conn_params_t *conn_params)
{
        uint16_t intv_min = conn_params->interval_min;
        uint16_t intv_max = conn_params->interval_max;

        printf("\tInterval Min: 0x%04x (%d.%02d ms)\r\n", intv_min,
                BLE_CONN_INTERVAL_TO_MS(intv_min), BLE_CONN_INTERVAL_TO_MS(intv_min * 100) % 100);
        printf("\tInterval Max: 0x%04x (%d.%02d ms)\r\n", intv_max,
                BLE_CONN_INTERVAL_TO_MS(intv_max), BLE_CONN_INTERVAL_TO_MS(intv_max * 100) % 100);
        printf("\tSlave Latency: 0x%04x (%d)\r\n", conn_params->slave_latency,
                                                        conn_params->slave_latency);
        printf("\tSupervision Timeout: 0x%04x (%d ms)\r\n", conn_params->sup_timeout,
                                        BLE_SUPERVISION_TMO_TO_MS(conn_params->sup_timeout));
}

static void conn_pause_timer_cb(TimerHandle_t timer)
{
        update_param_t *update_param;
        uint16_t conn_idx = (uint32_t) pvTimerGetTimerID(timer);

        update_param = OS_MALLOC(sizeof(*update_param));
        update_param->conn_idx = conn_idx;

        queue_push_back(&update_conn_param_q, update_param);

        OS_TASK_NOTIFY(current_task, BMS_UPDATE_CONN_PARAM_NOTIF, OS_NOTIFY_SET_BITS);
}

/*
 * Main code
 */

static void handle_evt_gap_connected(ble_evt_gap_connected_t *evt)
{
        conn_dev_t *conn_dev;

        printf("Device connected on conn_idx=%d (%s)\r\n", evt->conn_idx,
                                                        format_bd_address(&evt->peer_address));
        dump_conn_param(&evt->conn_params);
        printf("\r\n");

        conn_dev = OS_MALLOC(sizeof(*conn_dev));

        conn_dev->conn_idx = evt->conn_idx;
        conn_dev->unpair = false;
        memcpy(&conn_dev->addr, &evt->peer_address, sizeof(conn_dev->addr));

        conn_dev->conn_pause_timer = OS_TIMER_CREATE("conn_pause_peripheral", OS_MS_2_TICKS(5000),
                                OS_TIMER_FAIL, (uint32_t) conn_dev->conn_idx, conn_pause_timer_cb);

        OS_TIMER_START(conn_dev->conn_pause_timer, OS_TIMER_FOREVER);

        queue_push_back(&connections, conn_dev);

        ble_gap_set_sec_level(evt->conn_idx, GAP_SEC_LEVEL_4);

        print_connections();
}

static void handle_evt_gap_address_resolved(ble_evt_gap_address_resolved_t *evt)
{
        conn_dev_t *conn_dev;
        char addr[19];

        /*
         * we can't just call format_bd_address() twice in printf since it returns pointer to static
         * buffer so one address would overwrite another - instead, we format one of addresses into
         * temporary buffer and print from there
         */
        strcpy(addr, format_bd_address(&evt->address));

        printf("%s: conn_idx=%d address=%s resolved_address=%s\r\n", __func__,
                                evt->conn_idx, addr, format_bd_address(&evt->resolved_address));

        conn_dev = queue_find(&connections, conn_dev_match_conn_idx, &evt->conn_idx);
        if (conn_dev) {
                memcpy(&conn_dev->addr, &evt->resolved_address, sizeof(conn_dev->addr));
        }

        print_connections();
}

static void handle_evt_gap_disconnected(ble_evt_gap_disconnected_t *evt)
{
        conn_dev_t *conn_dev;

        printf("Device disconnected on conn_idx=%d (%s)\r\n", evt->conn_idx,
                                                                format_bd_address(&evt->address));
        printf("\tReason: 0x%02x\r\n", evt->reason);
        printf("\r\n");

        /*
         * If we had defaultBLE_MAX_CONNECTIONS devices connected, advertising has not been
         * restarted in advertising complete event, restart it here
         */
        if (queue_length(&connections) == defaultBLE_MAX_CONNECTIONS) {
                printf("Start advertising...\r\n");
                ble_gap_adv_start(GAP_CONN_MODE_UNDIRECTED);
        }

        conn_dev = queue_remove(&connections, conn_dev_match_conn_idx, &evt->conn_idx);

        if (conn_dev) {
                if (conn_dev->unpair) {
                        ble_gap_unpair(&conn_dev->addr);
                        printf("%s: device unpaired\r\n", __func__);
                        print_bonded_devices();
                } else {
                        printf("%s: device NOT unpaired\r\n", __func__);
                }

                OS_TIMER_DELETE(conn_dev->conn_pause_timer, OS_TIMER_FOREVER);

                OS_FREE(conn_dev);
        }

        print_connections();
}

static void handle_evt_gap_adv_completed(ble_evt_gap_adv_completed_t *evt)
{
        printf("%s: status=%d\r\n", __func__, evt->status);

        if (queue_length(&connections) < defaultBLE_MAX_CONNECTIONS) {
                printf("Start advertising...\r\n");
                // restart advertising so we can connect again
                ble_gap_adv_start(GAP_CONN_MODE_UNDIRECTED);
        }
}

static void handle_evt_gap_pair_req(ble_evt_gap_pair_req_t *evt)
{
        ble_error_t status;

        printf("%s: conn_idx=%d, bond=%i\r\n", __func__, evt->conn_idx, evt->bond);

        status = ble_gap_pair_reply(evt->conn_idx, true, true);
        if (status == BLE_ERROR_INS_RESOURCES) {
                printf("%s: cannot pair with conn_idx=%d, number of max bonded devices has"
                                                "been reached\r\n", __func__, evt->conn_idx);

                ble_gap_pair_reply(evt->conn_idx, false, false);
        }
}

static void handle_evt_gap_passkey_notify(ble_evt_gap_passkey_notify_t * evt)
{
        printf("%s: conn_idx=%d passkey=%06" PRIu32 "\r\n", __func__, evt->conn_idx, evt->passkey);
}

static void handle_evt_gap_pair_completed(ble_evt_gap_pair_completed_t *evt)
{
        printf("%s: conn_idx=%d status=%d bond=%d mitm=%d\r\n", __func__, evt->conn_idx,
                                                                evt->status, evt->bond, evt->mitm);

        if (evt->bond) {
                print_bonded_devices();
        }
}

static void handle_evt_gap_sec_level_changed(ble_evt_gap_sec_level_changed_t *evt)
{
        // numeric value of security level is +1 since enum value are numbered from 0
        int level = evt->level + 1;

        printf("Security Level Changed on conn_idx=%d\r\n", evt->conn_idx);
        printf("\tLevel: %d\r\n", level);
        printf("\r\n");
}

static void handle_evt_gap_conn_param_updated(ble_evt_gap_conn_param_updated_t *evt)
{
        printf("Connection Parameters Updated for conn_idx=%d\r\n", evt->conn_idx);
        dump_conn_param(&evt->conn_params);
        printf("\r\n");
}

void bms_task(void *params)
{
        int8_t wdog_id;

        /* register bms task to be monitored by watchdog */
        wdog_id = sys_watchdog_register(false);

        queue_init(&connections);
        queue_init(&update_conn_param_q);

        ble_peripheral_start();
        ble_register_app();

        current_task = OS_GET_CURRENT_TASK();

        /* set device name */
        ble_gap_device_name_set("Dialog BMS Demo", ATT_PERM_READ);

        /* register BMS */
        bms = bms_init(NULL, &bms_config, &bms_cb);
        ble_service_add(bms);

        ble_gap_set_io_cap(GAP_IO_CAP_DISP_ONLY);

        print_bonded_devices();

        ble_gap_adv_data_set(sizeof(adv_data), adv_data, 0, NULL);
        ble_gap_adv_start(GAP_CONN_MODE_UNDIRECTED);

        printf("Start advertising...\r\n");

        for (;;) {
                OS_BASE_TYPE ret;
                uint32_t notif;

                /* notify watchdog on each loop */
                sys_watchdog_notify(wdog_id);

                /* suspend watchdog while blocking on OS_TASK_NOTIFY_WAIT() */
                sys_watchdog_suspend(wdog_id);

                /*
                 * Wait on any of the notification bits, then clear them all
                 */
                ret = OS_TASK_NOTIFY_WAIT(0, OS_TASK_NOTIFY_ALL_BITS, &notif, OS_TASK_NOTIFY_FOREVER);
                /* Since it will block forever, it is guaranteed to have a valid notification when
                   it gets here */
                OS_ASSERT(ret == OS_OK);

                /* resume watchdog */
                sys_watchdog_notify_and_resume(wdog_id);

                /* notified from BLE manager, can get event */
                if (notif & BLE_APP_NOTIFY_MASK) {
                        ble_evt_hdr_t *hdr;

                        hdr = ble_get_event(false);
                        if (!hdr) {
                                goto no_event;
                        }

                        if (ble_service_handle_event(hdr)) {
                                goto handled;
                        }

                        switch (hdr->evt_code) {
                        case BLE_EVT_GAP_CONNECTED:
                                handle_evt_gap_connected((ble_evt_gap_connected_t *) hdr);
                                break;
                        case BLE_EVT_GAP_ADDRESS_RESOLVED:
                                handle_evt_gap_address_resolved((ble_evt_gap_address_resolved_t *) hdr);
                                break;
                        case BLE_EVT_GAP_DISCONNECTED:
                                handle_evt_gap_disconnected((ble_evt_gap_disconnected_t *) hdr);
                                break;
                        case BLE_EVT_GAP_ADV_COMPLETED:
                                handle_evt_gap_adv_completed((ble_evt_gap_adv_completed_t *) hdr);
                                break;
                        case BLE_EVT_GAP_PAIR_REQ:
                                handle_evt_gap_pair_req((ble_evt_gap_pair_req_t *) hdr);
                                break;
                        case BLE_EVT_GAP_PASSKEY_NOTIFY:
                                handle_evt_gap_passkey_notify((ble_evt_gap_passkey_notify_t *) hdr);
                                break;
                        case BLE_EVT_GAP_PAIR_COMPLETED:
                                handle_evt_gap_pair_completed((ble_evt_gap_pair_completed_t *) hdr);
                                break;
                        case BLE_EVT_GAP_SEC_LEVEL_CHANGED:
                                handle_evt_gap_sec_level_changed((ble_evt_gap_sec_level_changed_t *) hdr);
                                break;
                        case BLE_EVT_GAP_CONN_PARAM_UPDATED:
                                handle_evt_gap_conn_param_updated((ble_evt_gap_conn_param_updated_t *) hdr);
                                break;
                        default:
                                ble_handle_event_default(hdr);
                                break;
                        }

handled:
                        OS_FREE(hdr);

no_event:
                        // notify again if there are more events to process in queue
                        if (ble_has_event()) {
                                OS_TASK_NOTIFY(OS_GET_CURRENT_TASK(), BLE_APP_NOTIFY_MASK, eSetBits);
                        }
                }

                if (notif & BMS_DELETE_ALL_BOND_NOTIF) {
                        // Remove bond information of disconnected devices
                        unpair_disconnected();
                }

                if (notif & BMS_UPDATE_CONN_PARAM_NOTIF) {
                        update_param_t *update_param;

                        update_param = queue_pop_front(&update_conn_param_q);

                        if (update_param) {
                                update_conn_param(update_param->conn_idx);

                                OS_FREE(update_param);
                        }

                        if (queue_length(&update_conn_param_q)) {
                                OS_TASK_NOTIFY(current_task, BMS_UPDATE_CONN_PARAM_NOTIF,
                                                                               OS_NOTIFY_SET_BITS);
                        }
                }
        }
}
