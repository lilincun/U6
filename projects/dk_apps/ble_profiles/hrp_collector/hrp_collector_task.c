/**
 ****************************************************************************************
 *
 * @file hrp_collector_task.c
 *
 * @brief Heart Rate Collector task
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

#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "osal.h"
#include "cli.h"
#include "ble_bufops.h"
#include "ble_client.h"
#include "ble_uuid.h"
#include "hrs_client.h"
#include "dis_client.h"

/**
 * CLI notify mask
 */
#define CLI_NOTIF               (1 << 1)

#define MAX_FOUND_DEVICES       25

typedef enum {
        APP_STATE_IDLE,
        APP_STATE_SCANNING,
        APP_STATE_CONNECTING,
        APP_STATE_CONNECTED,
} app_state_t;

typedef enum {
        PENDING_ACTION_READ_SENSOR_LOCATION = (1 << 1),
        PENDING_ACTION_READ_MANUFACTURER = (1 << 2),
        PENDING_ACTION_READ_MODEL = (1 << 3),
        PENDING_ACTION_READ_FW_VERSION = (1 << 4),
        PENDING_ACTION_READ_SW_VERSION = (1 << 5),
        PENDING_ACTION_ENABLE_NOTIF = (1 << 6),
} pending_action_t;

typedef enum {
        AUTH_TYPE_ENCRYPT,
        AUTH_TYPE_PAIR,
        AUTH_TYPE_BOND,
} auth_type_t;

typedef struct {
        bd_address_t addr;
        bool name_found;
} found_device_t;

typedef struct {
        bd_address_t addr;
        uint16_t conn_idx;

        ble_client_t *hrs_client;
        ble_client_t *dis_client;

        bool busy_init;
        pending_action_t pending_init;

        bool busy_auth;
        bool pending_browse;
        bool notif_state;
} peer_info_t;

/* Application state */
__RETAINED_RW static app_state_t app_state = APP_STATE_IDLE;

/* Current peer information (only 1 peer can be connected) */
__RETAINED_RW static peer_info_t peer_info = {
        .conn_idx = BLE_CONN_IDX_INVALID,
};

/* Scanning state */
__RETAINED static struct {
        bool match_any;
        found_device_t devices[MAX_FOUND_DEVICES];
        size_t num_devices;
} scan_state;

static bool start_auth(auth_type_t auth_type, bool mitm);

static found_device_t *get_found_device(const bd_address_t *addr, size_t *index)
{
        size_t i;

        for (i = 0; i < scan_state.num_devices; i++) {
                found_device_t *dev = &scan_state.devices[i];

                if (ble_address_cmp(&dev->addr, addr)) {
                        *index = i + 1;
                        return dev;
                }
        }

        return NULL;
}

static inline found_device_t *add_found_device(const bd_address_t *addr, size_t *index)
{
        static found_device_t tmp_dev;
        found_device_t *dev;

        if (scan_state.num_devices >= MAX_FOUND_DEVICES) {
                dev = &tmp_dev;
                *index = 0;
        } else {
                dev = &scan_state.devices[scan_state.num_devices++];
                *index = scan_state.num_devices;
        }

        dev->addr = *addr;
        dev->name_found = false;

        return dev;
}

static bool set_notif_state(bool new_state)
{
        peer_info.notif_state = new_state;

        return hrs_client_set_event_state(peer_info.hrs_client,
                                        HRS_CLIENT_EVENT_HEART_RATE_MEASUREMENT_NOTIF, new_state);
}

#define pending_init_execute_and_check(FLAG, FUNCTION, ...) \
        ({                                                                      \
                if (peer_info.pending_init & FLAG) {                            \
                        peer_info.busy_init = FUNCTION(__VA_ARGS__);            \
                        if (!peer_info.busy_init) {                             \
                                /* Failed to execute action, clear bit */       \
                                peer_info.pending_init &= ~FLAG;                \
                        }                                                       \
                }                                                               \
                peer_info.busy_init;                                            \
        })

static void process_pending_actions(void)
{
        if (peer_info.busy_init) {
                return;
        }

        if (pending_init_execute_and_check(PENDING_ACTION_READ_SENSOR_LOCATION,
                                hrs_client_read_body_sensor_location, peer_info.hrs_client)) {
                return;
        }

        if (pending_init_execute_and_check(PENDING_ACTION_READ_MANUFACTURER, dis_client_read,
                                peer_info.dis_client, DIS_CLIENT_CAP_MANUFACTURER_NAME)) {
                return;
        }

        if (pending_init_execute_and_check(PENDING_ACTION_READ_MODEL, dis_client_read,
                                peer_info.dis_client, DIS_CLIENT_CAP_MODEL_NUMBER)) {
                return;
        }

        if (pending_init_execute_and_check(PENDING_ACTION_READ_FW_VERSION, dis_client_read,
                                peer_info.dis_client, DIS_CLIENT_CAP_FIRMWARE_REVISION)) {
                return;
        }

        if (pending_init_execute_and_check(PENDING_ACTION_READ_SW_VERSION, dis_client_read,
                                peer_info.dis_client, DIS_CLIENT_CAP_SOFTWARE_REVISION)) {
                return;
        }

        if (pending_init_execute_and_check(PENDING_ACTION_ENABLE_NOTIF, set_notif_state, true)) {
                return;
        }

        printf("Ready.\r\n");
}

static void add_pending_action(pending_action_t action)
{
        peer_info.pending_init |= action;

        process_pending_actions();
}

static void clear_pending_action(pending_action_t action, att_error_t error)
{
        /* Do nothing if we try to clear action which is not pending */
        if ((peer_info.pending_init & action) == 0) {
                return;
        }

        /* Try to authenticate if action failed due to unsufficient authentication/ecnryption */
        if ((error == ATT_ERROR_INSUFFICIENT_AUTHENTICATION) ||
                                                (error == ATT_ERROR_INSUFFICIENT_ENCRYPTION)) {
                peer_info.busy_init = false;
                start_auth(AUTH_TYPE_PAIR, false);
                return;
        }

        peer_info.busy_init = false;
        peer_info.pending_init &= ~action;

        process_pending_actions();
}

static void start_browse(void)
{
        printf("Browsing...\r\n");
        ble_gattc_browse(peer_info.conn_idx, NULL);
}

static bool start_auth(auth_type_t auth_type, bool mitm)
{
        gap_device_t gap_dev;

        if (peer_info.busy_auth) {
                /* We're already doing authentication, ignore */
                return false;
        }

        if (ble_gap_get_device_by_conn_idx(peer_info.conn_idx, &gap_dev) != BLE_STATUS_OK) {
                return false;
        }

        if ((auth_type == AUTH_TYPE_ENCRYPT) && !gap_dev.paired) {
                return false;
        }

        if (gap_dev.paired) {
                ble_gap_set_sec_level(peer_info.conn_idx, mitm ? GAP_SEC_LEVEL_3 : GAP_SEC_LEVEL_2);
        } else {
                ble_gap_pair(peer_info.conn_idx, auth_type == AUTH_TYPE_BOND);
        }

        peer_info.busy_auth = true;

        return true;
}

static void finish_auth(void)
{
        if (!peer_info.busy_auth) {
                return;
        }

        peer_info.busy_auth = false;

        if (peer_info.pending_browse) {
                start_browse();
                return;
        }

        /* Security is completed, check if there are pending init actions to complete */
        process_pending_actions();
}

static void clicmd_scan_usage(void)
{
        printf("usage: scan <start|stop> [any]\r\n");
        printf("\t\"any\" will disable filtering devices by HRS UUID, only valid for \"scan start\"\r\n");
}

static void clicmd_scan_handler(int argc, const char *argv[], void *user_data)
{
        if (argc < 2) {
                clicmd_scan_usage();
                return;
        }

        if (!strcasecmp("start", argv[1])) {
                if (app_state == APP_STATE_IDLE) {
                        scan_state.match_any = (argc > 2) && !strcmp(argv[2], "any");

                        scan_state.num_devices = 0;
                        ble_gap_scan_start(GAP_SCAN_ACTIVE, GAP_SCAN_OBSERVER_MODE,
                                                                BLE_SCAN_INTERVAL_FROM_MS(30),
                                                                BLE_SCAN_WINDOW_FROM_MS(30),
                                                                false, false);

                        printf("Scanning...\r\n");

                        app_state = APP_STATE_SCANNING;
                } else {
                        printf("ERROR: application has to be in idle state to start scanning\r\n");
                }
        } else if (!strcasecmp("stop", argv[1])) {
                if (app_state == APP_STATE_SCANNING) {
                        ble_gap_scan_stop();

                        printf("Scan stopped\r\n");

                        app_state = APP_STATE_IDLE;
                } else {
                        printf("ERROR: no scan session in progress\r\n");
                }
        } else {
                clicmd_scan_usage();
        }
}

static void clicmd_connect_usage(void)
{
        printf("usage: connect <address|index> [public|random]\r\n");
        printf("       connect cancel\r\n");
        printf("\tinstead of address, index of found device can be passed\r\n");
        printf("\tif not specified, public address is assumed\r\n");
        printf("\tuse 'connect cancel' to cancel any outgoing connection attempt\r\n");
}

static void clicmd_connect_handler(int argc, const char *argv[], void *user_data)
{
        static const gap_conn_params_t cp = {
                .interval_min = BLE_CONN_INTERVAL_FROM_MS(30),
                .interval_max = BLE_CONN_INTERVAL_FROM_MS(50),
                .slave_latency = 0,
                .sup_timeout = BLE_SUPERVISION_TMO_FROM_MS(1000),
        };

        bd_address_t addr;
        size_t dev_index;

        if (argc < 2) {
                clicmd_connect_usage();
                return;
        }

        if (!strcasecmp("cancel", argv[1])) {
                if (app_state != APP_STATE_CONNECTING) {
                        printf("ERROR: no active connection attempt to cancel\r\n");
                        return;
                }

                ble_gap_connect_cancel();
                return;
        }

        if (app_state != APP_STATE_IDLE) {
                printf("ERROR: application has to be in idle state to connect\r\n");
                return;
        }

        /*
         * If argument cannot be parsed to valid address, check if it can be used as index in
         * found devices cache.
         */
        if (!ble_address_from_string(argv[1], PUBLIC_ADDRESS, &addr)) {
                dev_index = atoi(argv[1]);
                if (dev_index < 1 || dev_index > scan_state.num_devices) {
                        clicmd_connect_usage();
                        return;
                }

                addr = scan_state.devices[dev_index - 1].addr;
        } else {
                if (argc > 2) {
                        /*
                         * If address type argument is present, check for "random" or leave "public"
                         * as set by default.
                         */

                        if (!strcasecmp("random", argv[2])) {
                                addr.addr_type = PRIVATE_ADDRESS;
                        }
                } else {
                        size_t i;

                        /*
                         * If address type is not present try to check for address in found devices
                         * cache, otherwise leave "public".
                         */

                        for (i = 0; i < scan_state.num_devices; i++) {
                                found_device_t *dev = &scan_state.devices[i];

                                if (!memcmp(&dev->addr.addr, &addr.addr, sizeof(addr.addr))) {
                                        addr.addr_type = dev->addr.addr_type;
                                        break;
                                }
                        }
                }
        }

        ble_gap_connect(&addr, &cp);

        printf("Connecting to %s ...\r\n", ble_address_to_string(&addr));

        app_state = APP_STATE_CONNECTING;
}

static void clicmd_disconnect_handler(int argc, const char *argv[], void *user_data)
{
        if (app_state != APP_STATE_CONNECTED) {
                printf("ERROR: application has to be in connected state to disconnect\r\n");
                return;
        }

        ble_gap_disconnect(peer_info.conn_idx, BLE_HCI_ERROR_REMOTE_USER_TERM_CON);
        app_state = APP_STATE_IDLE;

        printf("Disconnected from %s\r\n", ble_address_to_string(&peer_info.addr));
}

static void clicmd_notifications_usage(void)
{
        printf("usage: notifications <on|off>\r\n");
}

static void clicmd_notifications_handler(int argc, const char *argv[], void *user_data)
{
        if (argc < 2) {
                clicmd_notifications_usage();
                return;
        }

        if (app_state != APP_STATE_CONNECTED) {
                printf("ERROR: application has to be in connected state to set notifications\r\n");
                return;
        }

        if (!strcasecmp("on", argv[1])) {
                set_notif_state(true);
        } else if (!strcasecmp("off", argv[1])) {
                set_notif_state(false);
        } else {
                clicmd_notifications_usage();
        }
}

static void clicmd_reset_ee_handler(int argc, const char *argv[], void *user_data)
{
        if (app_state != APP_STATE_CONNECTED) {
                printf("ERROR: application has to be in connected state to reset energy expended\r\n");
                return;
        }

        if (!hrs_client_reset_energy_expended(peer_info.hrs_client)) {
                printf("ERROR: failed to reset energy expended\r\n");
                return;
        }
}

static void clicmd_default_handler(int argc, const char *argv[], void *user_data)
{
        printf("Valid commands:\r\n");
        printf("\tscan <start|stop> [any]\r\n");
        printf("\tconnect <address|index> [public|random]\r\n");
        printf("\tconnect cancel\r\n");
        printf("\tnotifications <on|off>\r\n");
        printf("\treset_ee\r\n");
        printf("\tdisconnect\r\n");
}

static const cli_command_t clicmd[] = {
        { .name = "scan",               .handler = clicmd_scan_handler, },
        { .name = "connect",            .handler = clicmd_connect_handler, },
        { .name = "notifications",      .handler = clicmd_notifications_handler, },
        { .name = "reset_ee",           .handler = clicmd_reset_ee_handler, },
        { .name = "disconnect",         .handler = clicmd_disconnect_handler, },
        {},
};

void hrs_heart_rate_measurement_notif_cb(ble_client_t *client, hrs_client_measurement_t *measurement)
{
        printf("Heart Rate Measurement notification received\r\n");

        printf("\tValue: %d bpm\r\n", measurement->bpm);

        if (measurement->contact_supported) {
                printf("\tSensor Contact: supported, %s\r\n",
                                        measurement->contact_detected ? "detected" : "not detected");
        } else {
                printf("\tSensor Contact: not supported\r\n");
        }

        if (measurement->has_energy_expended) {
                printf("\tEnergy Expended: %d kJ\r\n", measurement->energy_expended);
        }

        if (measurement->rr_num) {
                int i;

                printf("\tRR-Intervals: ");

                for (i = 0; i < measurement->rr_num; i++) {
                        printf("%d ", measurement->rr[i]);
                }

                printf("\r\n");
        }

        printf("\r\n");
        OS_FREE(measurement);
}

void hrs_set_event_state_completed_cb(ble_client_t *client, hrs_client_event_t event, att_error_t status)
{
        /*
         * This is the only event supported right now, but more can be added in future versions so
         * need to check here.
         */
        if (event != HRS_CLIENT_EVENT_HEART_RATE_MEASUREMENT_NOTIF) {
                return;
        }

        if (status == ATT_ERROR_OK) {
                printf("Heart Rate Measurement notifications %s\r\n", peer_info.notif_state ?
                                                                        "enabled" : "disabled");
        } else {
                printf("ERROR: failed to set notifications (0x%02x)\r\n", status);
        }

        clear_pending_action(PENDING_ACTION_ENABLE_NOTIF, status);
}

static char *body_sensor_location_to_string(hrs_client_body_sensor_location_t location)
{
        switch (location) {
        case HRS_CLIENT_BODY_SENSOR_LOC_OTHER:
                return "other";
        case HRS_CLIENT_BODY_SENSOR_LOC_CHEST:
                return "chest";
        case HRS_CLIENT_BODY_SENSOR_LOC_WRIST:
                return "wrist";
        case HRS_CLIENT_BODY_SENSOR_LOC_FINGER:
                return "finger";
        case HRS_CLIENT_BODY_SENSOR_LOC_HAND:
                return "hand";
        case HRS_CLIENT_BODY_SENSOR_LOC_EAR_LOBE:
                return "ear lobe";
        case HRS_CLIENT_BODY_SENSOR_LOC_FOOT:
                return "foot";
        }

        return "unknown";
}

void hrs_read_body_sensor_location_completed_cb(ble_client_t *client, att_error_t status,
                                                        hrs_client_body_sensor_location_t location)
{
        if (status == ATT_ERROR_OK) {
                printf("\tBody Sensor Location: %s\r\n", body_sensor_location_to_string(location));
        } else {
                printf("\tFailed to read Body Sensor Location (%#x)\r\n", status);
        }

        clear_pending_action(PENDING_ACTION_READ_SENSOR_LOCATION, status);
}

void hrs_reset_energy_expended_completed_cb(ble_client_t *client, att_error_t status)
{
        printf("%s: Reset Energy Expended - status: 0x%02X\r\n", __func__, status);
}

static const hrs_client_callbacks_t hrs_callbacks = {
        .heart_rate_measurement_notif = hrs_heart_rate_measurement_notif_cb,
        .set_event_state_completed = hrs_set_event_state_completed_cb,
        .read_body_sensor_location_completed = hrs_read_body_sensor_location_completed_cb,
        .reset_energy_expended_completed =  hrs_reset_energy_expended_completed_cb,
};

static void dis_read_completed_cb(ble_client_t *dis_client, att_error_t status,
                                                        dis_client_cap_t capability,
                                                        uint16_t length, const uint8_t *value)
{
        switch (capability) {
        case DIS_CLIENT_CAP_MANUFACTURER_NAME:
                if (status == ATT_ERROR_OK) {
                        printf("\tManufacturer: %.*s\r\n", length, value);
                } else {
                        printf("\tFailed to read Manufacturer information (%#x)\r\n", status);
                }
                clear_pending_action(PENDING_ACTION_READ_MANUFACTURER, status);
                break;
        case DIS_CLIENT_CAP_MODEL_NUMBER:
                if (status == ATT_ERROR_OK) {
                        printf("\tModel: %.*s\r\n", length, value);
                } else {
                        printf("\tFailed to read Model information (%#x)\r\n", status);
                }
                clear_pending_action(PENDING_ACTION_READ_MODEL, status);
                break;
        case DIS_CLIENT_CAP_FIRMWARE_REVISION:
                if (status == ATT_ERROR_OK) {
                        printf("\tFirmware version: %.*s\r\n", length, value);
                } else {
                        printf("\tFailed to read FW Version information (%#x)\r\n", status);
                }
                clear_pending_action(PENDING_ACTION_READ_FW_VERSION, status);
                break;
        case DIS_CLIENT_CAP_SOFTWARE_REVISION:
                if (status == ATT_ERROR_OK) {
                        printf("\tSoftware version: %.*s\r\n", length, value);
                } else {
                        printf("\tFailed to read SW Version information (%#x)\r\n", status);
                }
                clear_pending_action(PENDING_ACTION_READ_SW_VERSION, status);
                break;
        default:
                break;
        }
}

static const dis_client_callbacks_t dis_callbacks = {
        .read_completed = dis_read_completed_cb,
};

static void handle_evt_gap_adv_report(const ble_evt_gap_adv_report_t *evt)
{
        found_device_t *dev;
        size_t dev_index = 0;
        const uint8_t *p;
        uint8_t ad_len, ad_type;
        bool new_device = false;
        const char *dev_name = NULL;
        size_t dev_name_len = 0;

        dev = get_found_device(&evt->address, &dev_index);
        if (dev && dev->name_found) {
                return;
        }

        /* Add device if 'any' was specified as scan argument */
        if (!dev && scan_state.match_any) {
                new_device = true;
                dev = add_found_device(&evt->address, &dev_index);
        }

        for (p = evt->data; p < evt->data + evt->length; p += ad_len) {
                ad_len = (*p++) - 1; /* ad_len is length of value only, without type */
                ad_type = *p++;

                /* Device not found so we look for UUID */
                if (!dev && (ad_type == GAP_DATA_TYPE_UUID16_LIST ||
                                                        ad_type == GAP_DATA_TYPE_UUID16_LIST_INC)) {
                        size_t idx;

                        for (idx = 0; idx < ad_len; idx += sizeof(uint16_t)) {
                                if (get_u16(p + idx) == UUID_SERVICE_HRS) {
                                        new_device = true;
                                        dev = add_found_device(&evt->address, &dev_index);
                                        break;
                                }
                        }

                        continue;
                }

                /* Look for name and store it to use later, if proper UUID is found */
                if (ad_type == GAP_DATA_TYPE_SHORT_LOCAL_NAME ||
                                                        ad_type == GAP_DATA_TYPE_LOCAL_NAME) {
                        dev_name = (const char *) p;
                        dev_name_len = ad_len;

                        if (dev) {
                                /* Already have device, no need to look further */
                                break;
                        }
                }
        }

        /*
         * If we have both device and device name, print as new device found with name.
         * For new device and no name, just print address for now.
         */
        if (dev && dev_name) {
                dev->name_found = true;
                printf("[%02d] Device found: %s (%.*s)\r\n", dev_index,
                                                                ble_address_to_string(&evt->address),
                                                                dev_name_len, dev_name);
        } else if (new_device) {
                printf("[%02d] Device found: %s\r\n", dev_index, ble_address_to_string(&evt->address));
        }
}

static void handle_evt_gap_connected(const ble_evt_gap_connected_t *evt)
{
        OS_ASSERT(peer_info.conn_idx == BLE_CONN_IDX_INVALID);

        printf("Device connected\r\n");
        printf("\tAddress: %s\r\n", ble_address_to_string(&evt->peer_address));
        printf("\tConnection index: %d\r\n", evt->conn_idx);

        peer_info.addr = evt->peer_address;
        peer_info.conn_idx = evt->conn_idx;

        app_state = APP_STATE_CONNECTED;

        /*
         * Try to start encryption first. If cannot be started it means we are not bonded and can
         * got directly to browse.
         */
        if (start_auth(AUTH_TYPE_ENCRYPT, false)) {
                peer_info.pending_browse = true;
                return;
        }

        start_browse();
}

static void handle_evt_gap_connection_completed(const ble_evt_gap_connection_completed_t *evt)
{
        /* Successful connections are handled in separate event */
        if (evt->status == BLE_STATUS_OK) {
                return;
        }

        printf("Connection failed\r\n");
        printf("\tStatus: 0x%02x\r\n", evt->status);

        app_state = APP_STATE_IDLE;
}

static void handle_evt_gap_disconnected(const ble_evt_gap_disconnected_t * evt)
{
        printf("Device disconnected\r\n");
        printf("\tConnection index: %d\r\n", evt->conn_idx);
        printf("\tReason: 0x%02x\r\n", evt->reason);

        if (peer_info.hrs_client) {
                ble_client_cleanup(peer_info.hrs_client);
        }
        if (peer_info.dis_client) {
                ble_client_cleanup(peer_info.dis_client);
        }

        memset(&peer_info, 0, sizeof(peer_info));
        peer_info.conn_idx = BLE_CONN_IDX_INVALID;

        app_state = APP_STATE_IDLE;
}

static void handle_evt_gap_security_request(const ble_evt_gap_security_request_t *evt)
{
        OS_ASSERT(evt->conn_idx == peer_info.conn_idx);

        printf("Security Request received\r\n");

        start_auth(evt->bond ? AUTH_TYPE_BOND : AUTH_TYPE_PAIR, evt->mitm);
}

static void handle_evt_gap_passkey_notify(const ble_evt_gap_passkey_notify_t *evt)
{
        OS_ASSERT(evt->conn_idx == peer_info.conn_idx);

        printf("Passkey: %06" PRIu32 "\r\n", evt->passkey);
}

static void handle_evt_gap_pair_completed(const ble_evt_gap_pair_completed_t *evt)
{
        if (evt->status == BLE_STATUS_OK) {
                OS_ASSERT(evt->conn_idx == peer_info.conn_idx);
                printf("Pairing completed (%s, %s)\r\n", evt->bond ? "bond" : "no bond",
                                                                evt->mitm ? "MITM" : "no MITM");
        } else {
                printf("Pairing failed (0x%02x)\r\n", evt->status);
        }

        finish_auth();
}

static void handle_evt_gap_sec_level_changed(const ble_evt_gap_sec_level_changed_t *evt)
{
        OS_ASSERT(evt->conn_idx == peer_info.conn_idx);

        printf("Security Level changed to %d\r\n", evt->level + 1);

        finish_auth();
}

static void handle_evt_gap_set_sec_level_failed(const ble_evt_gap_set_sec_level_failed_t *evt)
{
        printf("Failed to set security level (0x%02x)\r\n", evt->status);

        finish_auth();
}

static void handle_evt_gattc_browse_svc(const ble_evt_gattc_browse_svc_t *evt)
{
        OS_ASSERT(evt->conn_idx == peer_info.conn_idx);

        /* We are not interested in any service with 128-bit UUID */
        if (evt->uuid.type != ATT_UUID_16) {
                return;
        }

        switch (evt->uuid.uuid16) {
        case UUID_SERVICE_HRS:
                if (peer_info.hrs_client) {
                        return;
                }

                peer_info.hrs_client = hrs_client_init(&hrs_callbacks, evt);
                if (!peer_info.hrs_client) {
                        return;
                }

                ble_client_add(peer_info.hrs_client);
                break;
        case UUID_SERVICE_DIS:
                if (peer_info.dis_client) {
                        return;
                }

                peer_info.dis_client = dis_client_init(&dis_callbacks, evt);
                if (!peer_info.dis_client) {
                        return;
                }

                ble_client_add(peer_info.dis_client);
                break;
        }
}

static void handle_evt_gattc_browse_completed(const ble_evt_gattc_browse_completed_t *evt)
{
        if (evt->status == BLE_STATUS_OK) {
                OS_ASSERT(evt->conn_idx == peer_info.conn_idx);
        }

        printf("Browse completed\r\n");

        if (!peer_info.hrs_client) {
                printf("\tHeart Rate service not found\r\n");

                printf("Disconnecting...\r\n");
                ble_gap_disconnect(evt->conn_idx, BLE_HCI_ERROR_REMOTE_USER_TERM_CON);
                return;
        }

        printf("\tHeart Rate service found\r\n");

        if (peer_info.dis_client) {
                dis_client_cap_t cap = dis_client_get_capabilities(peer_info.dis_client);

                printf("\tDevice Information Service found\r\n");

                /* Read manufacturer name (if supported by DIS server) */
                if (cap & DIS_CLIENT_CAP_MANUFACTURER_NAME) {
                        add_pending_action(PENDING_ACTION_READ_MANUFACTURER);
                }

                /* Read model number (if supported by DIS server) */
                if (cap & DIS_CLIENT_CAP_MODEL_NUMBER) {
                        add_pending_action(PENDING_ACTION_READ_MODEL);
                }

                /* Read firmware version (if supported by DIS server) */
                if (cap & DIS_CLIENT_CAP_FIRMWARE_REVISION) {
                        add_pending_action(PENDING_ACTION_READ_FW_VERSION);
                }

                /* Read software version (if supported by DIS server) */
                if (cap & DIS_CLIENT_CAP_SOFTWARE_REVISION) {
                        add_pending_action(PENDING_ACTION_READ_SW_VERSION);
                }
        }

        /* Read body sensor location */
        if (hrs_client_get_capabilities(peer_info.hrs_client)
                                                        & HRS_CLIENT_CAP_BODY_SENSOR_LOCATION) {
                add_pending_action(PENDING_ACTION_READ_SENSOR_LOCATION);
        }

        /* Enable measurement notifications */
        add_pending_action(PENDING_ACTION_ENABLE_NOTIF);

        printf("Querying device...\r\n");
}

void hrp_collector_task(void *params)
{
        cli_t cli;

        /* Register CLI */
        cli = cli_register(CLI_NOTIF, clicmd, clicmd_default_handler);

        /* Set device name */
        ble_gap_device_name_set("Black Orca HR Collector", ATT_PERM_READ);

        /* Setup application in BLE Manager */
        ble_register_app();
        ble_central_start();

        /*
         * We have keyboard support (for CLI) but since support for passkey entry is missing, let's
         * just declare "Display Only" capabilities for now.
         */
        ble_gap_set_io_cap(GAP_IO_CAP_DISP_ONLY);

        printf("HRP Collector ready.\r\n");

        for (;;) {
                OS_BASE_TYPE ret;
                uint32_t notif;

                ret = OS_TASK_NOTIFY_WAIT(0, (uint32_t) -1, &notif, OS_TASK_NOTIFY_FOREVER);
                OS_ASSERT(ret == OS_OK);

                /* Notified from BLE manager, can get event */
                if (notif & BLE_APP_NOTIFY_MASK) {
                        ble_evt_hdr_t *hdr;

                        /*
                         * No need to wait for event, should be already there since we were notified
                         * from manager
                         */
                        hdr = ble_get_event(false);

                        if (!hdr) {
                                goto no_event;
                        }

                        ble_client_handle_event(hdr);

                        switch (hdr->evt_code) {
                        case BLE_EVT_GAP_ADV_REPORT:
                                handle_evt_gap_adv_report((ble_evt_gap_adv_report_t *) hdr);
                                break;
                        case BLE_EVT_GAP_CONNECTED:
                                handle_evt_gap_connected((ble_evt_gap_connected_t *) hdr);
                                break;
                        case BLE_EVT_GAP_CONNECTION_COMPLETED:
                                handle_evt_gap_connection_completed((ble_evt_gap_connection_completed_t *) hdr);
                                break;
                        case BLE_EVT_GAP_DISCONNECTED:
                                handle_evt_gap_disconnected((ble_evt_gap_disconnected_t *) hdr);
                                break;
                        case BLE_EVT_GAP_SECURITY_REQUEST:
                                handle_evt_gap_security_request((ble_evt_gap_security_request_t *) hdr);
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
                        case BLE_EVT_GAP_SET_SEC_LEVEL_FAILED:
                                handle_evt_gap_set_sec_level_failed((ble_evt_gap_set_sec_level_failed_t *) hdr);
                                break;
                        case BLE_EVT_GATTC_BROWSE_SVC:
                                handle_evt_gattc_browse_svc((ble_evt_gattc_browse_svc_t *) hdr);
                                break;
                        case BLE_EVT_GATTC_BROWSE_COMPLETED:
                                handle_evt_gattc_browse_completed((ble_evt_gattc_browse_completed_t *) hdr);
                                break;
                        default:
                                ble_handle_event_default(hdr);
                                break;
                        }

                        /* Free event buffer (it's not needed anymore) */
                        OS_FREE(hdr);

no_event:
                        /*
                         * If there are more events waiting in queue, application should process
                         * them now.
                         */
                        if (ble_has_event()) {
                                OS_TASK_NOTIFY(OS_GET_CURRENT_TASK(), BLE_APP_NOTIFY_MASK,
                                                                                OS_NOTIFY_SET_BITS);
                        }
                }

                if (notif & CLI_NOTIF) {
                        cli_handle_notified(cli);
                }
        }
}
