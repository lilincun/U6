/**
 ****************************************************************************************
 *
 * @file cscs_collector_task.c
 *
 * @brief Cycling Speed and Cadence Collector task
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
#include <limits.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "osal.h"
#include "cli.h"
#include "ble_bufops.h"
#include "ble_client.h"
#include "ble_uuid.h"
#include "cscs_client.h"
#include "dis_client.h"
#include "cscp_collector_config.h"

/* Main task notification bits */
#define CP_OPERATION_TMO_NOTIF  (1 << 1)
#define CLI_NOTIF               (1 << 2)

#define CP_OPERATION_TMO_MS    (30000)
#define MAX_FOUND_DEVICES       25

typedef enum {
        APP_STATE_IDLE,
        APP_STATE_SCANNING,
        APP_STATE_CONNECTING,
        APP_STATE_CONNECTED,
} app_state_t;

typedef enum {
        PENDING_ACTION_READ_MANUFACTURER = (1 << 0),
        PENDING_ACTION_READ_MODEL = (1 << 1),
        PENDING_ACTION_READ_FW_VERSION = (1 << 2),
        PENDING_ACTION_READ_SW_VERSION = (1 << 3),
        PENDING_ACTION_READ_FEATURES = (1 << 4),
        PENDING_ACTION_READ_SENSOR_LOCATION = (1 << 5),
        PENDING_ACTION_ENABLE_CP = (1 << 6),
        PENDING_ACTION_ENABLE_MEASUREMENT_NOTIF = (1 << 7),
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

        ble_client_t *cscs_client;
        ble_client_t *dis_client;

        bool busy_init;
        pending_action_t pending_init;

        bool busy_auth;
        bool pending_browse;
        bool notif_state;

        uint16_t csc_feature;
        cscs_client_measurement_t last_measurement;
} peer_info_t;

/* Application state */
__RETAINED_RW static app_state_t app_state = APP_STATE_IDLE;

/* Current peer information (only 1 peer can be connected) */
__RETAINED_RW static peer_info_t peer_info = {
        .conn_idx = BLE_CONN_IDX_INVALID,
};

/* Timer for handling CP operations timeout */
__RETAINED static OS_TIMER cp_tmo_timer;

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

        return cscs_client_set_event_state(peer_info.cscs_client,
                                                CSCS_CLIENT_EVENT_CSC_MEASUREMENT_NOTIF, new_state);
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

        if (pending_init_execute_and_check(PENDING_ACTION_READ_FEATURES,
                                cscs_client_read_csc_features, peer_info.cscs_client)) {
                return;
        }

        if (pending_init_execute_and_check(PENDING_ACTION_READ_SENSOR_LOCATION,
                                cscs_client_read_sensor_location, peer_info.cscs_client)) {
                return;
        }

        if (pending_init_execute_and_check(PENDING_ACTION_ENABLE_CP,
                                                        cscs_client_set_sc_control_point_ind_state,
                                                        peer_info.cscs_client, true)) {
                return;
        }

        if (pending_init_execute_and_check(PENDING_ACTION_ENABLE_MEASUREMENT_NOTIF, set_notif_state, true)) {
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

static void clicmd_get_usage(void)
{
        printf("usage: get <parameter>\r\n");
        printf("available parameters (note: availability may depend on features supported by sensor)\r\n");
        printf("\tlocations  supported sensor locations\r\n");
        printf("\tlocation   current sensor location\r\n");
}

static void clicmd_get_handler(int argc, const char *argv[], void *user_data)
{
        if (argc < 2) {
                clicmd_get_usage();
                return;
        }

        if (app_state != APP_STATE_CONNECTED) {
                printf("ERROR: application has to be in connected state to get parameter\r\n");
                return;
        }

        if (!strcasecmp("locations", argv[1])) {
                if (!cscs_client_request_supported_sensor_locations(peer_info.cscs_client)) {
                        goto failed;
                }

                OS_TIMER_START(cp_tmo_timer, OS_TIMER_FOREVER);
        } else if (!strcasecmp("location", argv[1])) {
                if (!cscs_client_read_sensor_location(peer_info.cscs_client)) {
                        goto failed;
                }
        } else {
                clicmd_get_usage();
        }

        return;

failed:
        printf("ERROR: failed to start operation\r\n");
}

static void clicmd_set_usage(void)
{
        printf("usage: set <parameter> <value>\r\n");
        printf("available parameters (note: availability may depend on features supported by sensor)\r\n");
        printf("\twheelrev   cumulative wheel revolution data\r\n");
        printf("\tlocation   sensor location\r\n");
}

static void clicmd_set_handler(int argc, const char *argv[], void *user_data)
{
        bool ret;

        if (argc < 3) {
                clicmd_set_usage();
                return;
        }

        if (app_state != APP_STATE_CONNECTED) {
                printf("ERROR: application has to be in connected state to set parameter\r\n");
                return;
        }

        if (!strcasecmp("wheelrev", argv[1])) {
                ret = cscs_client_set_cumulative_value(peer_info.cscs_client, atoi(argv[2]));
        } else if (!strcasecmp("location", argv[1])) {
                ret = cscs_client_update_sensor_location(peer_info.cscs_client, atoi(argv[2]));
        } else {
                clicmd_set_usage();
                return;
        }

        if (ret) {
                OS_TIMER_START(cp_tmo_timer, OS_TIMER_FOREVER);
        } else {
                printf("ERROR: failed to start operation\r\n");
        }
}

static void clicmd_default_handler(int argc, const char *argv[], void *user_data)
{
        printf("Valid commands:\r\n");
        printf("\tscan <start|stop> [any]\r\n");
        printf("\tconnect <address|index> [public|random]\r\n");
        printf("\tconnect cancel\r\n");
        printf("\tnotifications <on|off>\r\n");
        printf("\tdisconnect\r\n");
        printf("\tget <parameter>\r\n");
        printf("\tset <parameter> <value>\r\n");
}

static const cli_command_t clicmd[] = {
        { .name = "scan",               .handler = clicmd_scan_handler, },
        { .name = "connect",            .handler = clicmd_connect_handler, },
        { .name = "disconnect",         .handler = clicmd_disconnect_handler, },
        { .name = "notifications",      .handler = clicmd_notifications_handler, },
        { .name = "get",                .handler = clicmd_get_handler, },
        { .name = "set",                .handler = clicmd_set_handler, },
        {},
};

static void timer_cb(OS_TIMER timer)
{
        OS_TASK app = OS_TIMER_GET_TIMER_ID(timer);

        OS_TASK_NOTIFY(app, CP_OPERATION_TMO_NOTIF, OS_NOTIFY_SET_BITS);
}

/* Return speed in [m / h] */
static unsigned calculate_speed(uint16_t evt_time, uint32_t rev)
{
        uint16_t time;
        uint32_t distance;
        unsigned speed;

        distance = rev - peer_info.last_measurement.cumulative_wheel_revolutions;
        time = evt_time - peer_info.last_measurement.last_wheel_event_time;

        if (time == 0) {
                return 0;
        }

        /* Distance in [mm] */
        distance *= CFG_WHEEL_CIRCUMFERENCE;
        /* Time in [ms] */
        time = time * 1000 / 1024;
        /* Speed in [m / h] */
        speed = distance * 3600 / time ;

        return speed;
}

/* Function returns cadence in [rpm] */
static unsigned calculate_cadence(uint16_t evt_time, uint16_t rev)
{
        uint16_t time;
        uint16_t rev_diff;
        unsigned cadence;

        rev_diff = rev - peer_info.last_measurement.cumulative_crank_revolutions;
        time = evt_time - peer_info.last_measurement.last_crank_event_time;

        if (time == 0) {
                return 0;
        }

        /* Time in [ms] */
        time = time * 1000 / 1024;
        /* Cadence in [rpm] */
        cadence = rev_diff * 1000 * 60 / time;

        return cadence;
}

static void cscs_csc_measurement_cb(ble_client_t *client, const cscs_client_measurement_t *measurement)
{
        unsigned speed = UINT_MAX;
        unsigned cadence = UINT_MAX;

        printf("CSC measurement notification received\r\n");

        if (measurement->wheel_revolution_data_present) {
                uint16_t time = measurement->last_wheel_event_time;
                uint32_t rev = measurement->cumulative_wheel_revolutions;

                printf("\tLast wheel event time: %" PRIu16 ".%03" PRIu16 " [s] (raw %" PRIu16 ")\r\n",
                                                time / 1024, time * 1000 / 1024 % 1000, time);
                printf("\tCumulative wheel revolutions: %" PRIu32 "\r\n", rev);

                if (peer_info.last_measurement.wheel_revolution_data_present &&
                                (peer_info.last_measurement.cumulative_wheel_revolutions <= rev)) {
                        speed = calculate_speed(time, rev);
                }

                peer_info.last_measurement.wheel_revolution_data_present = true;
                peer_info.last_measurement.cumulative_wheel_revolutions = rev;
                peer_info.last_measurement.last_wheel_event_time = time;
        }

        if (measurement->crank_revolution_data_present) {
                uint16_t time = measurement->last_crank_event_time;
                uint16_t rev = measurement->cumulative_crank_revolutions;

                printf("\tLast crank event time: %" PRIu16 ".%03" PRIu16 " [s] (raw %" PRIu16 ")\r\n",
                                                time / 1024, time * 1000 / 1024 % 1000, time);
                printf("\tCumulative crank revolutions: %" PRIu16 "\r\n", rev);

                if (peer_info.last_measurement.crank_revolution_data_present) {
                        cadence = calculate_cadence(time, rev);
                }

                peer_info.last_measurement.crank_revolution_data_present = true;
                peer_info.last_measurement.cumulative_crank_revolutions = rev;
                peer_info.last_measurement.last_crank_event_time = time;
        }

        if (speed < UINT_MAX) {
                printf("\tCalculated speed: %u.%03u [km/h]\r\n", speed / 1000, speed % 1000);
        }

        if (cadence < UINT_MAX) {
                printf("\tCalculated cadence: %u [rpm]\r\n", cadence);
        }
}

static void cscs_read_csc_features_completed_cb(ble_client_t *cscs_client, att_error_t status,
                                                                                uint16_t features)
{
        if (status == ATT_ERROR_OK) {
                peer_info.csc_feature = features;

                printf("\tFeatures: 0x%02" PRIx16 "\r\n", features);

                if (features & CSCS_CLIENT_FEATURE_WHEEL_REVOLUTION_DATA) {
                        printf("\t\twheel revolution data supported\r\n");

                        /* CP shall be supported if this feature is supported by peer */
                        add_pending_action(PENDING_ACTION_ENABLE_CP);
                } else {
                        printf("\t\twheel revolution data NOT SUPPORTED\r\n");
                }

                if (features & CSCS_CLIENT_FEATURE_CRANK_REVOLUTION_DATA) {
                        printf("\t\tcrank revolution data supported\r\n");
                } else {
                        printf("\t\tcrank revolution data NOT SUPPORTED\r\n");
                }

                if (features & CSCS_CLIENT_FEATURE_MULTIPLE_SENSOR_LOCATIONS) {
                        printf("\t\tmultiple sensor locations supported\r\n");

                        /* CP shall be supported if this feature is supported by peer */
                        add_pending_action(PENDING_ACTION_ENABLE_CP);
                } else {
                        printf("\t\tmultiple sensor locations NOT SUPPORTED\r\n");
                }
        } else {
                printf("\tFailed to read features (0x%02x)\r\n", status);
        }

        clear_pending_action(PENDING_ACTION_READ_FEATURES, status);
}

static char *sensor_location_to_string(cscs_client_sensor_location_t location)
{
        switch (location) {
        case CSCS_CLIENT_SENSOR_LOCATION_OTHER:
                return "other";
        case CSCS_CLIENT_SENSOR_LOCATION_TOP_OF_SHOE:
                return "top of shoe";
        case CSCS_CLIENT_SENSOR_LOCATION_IN_SHOE:
                return "in shoe";
        case CSCS_CLIENT_SENSOR_LOCATION_HIP:
                return "hip";
        case CSCS_CLIENT_SENSOR_LOCATION_FRONT_WHEEL:
                return "front wheel";
        case CSCS_CLIENT_SENSOR_LOCATION_LEFT_CRANK:
                return "left crank";
        case CSCS_CLIENT_SENSOR_LOCATION_RIGHT_CRANK:
                return "right crank";
        case CSCS_CLIENT_SENSOR_LOCATION_LEFT_PEDAL:
                return "left pedal";
        case CSCS_CLIENT_SENSOR_LOCATION_RIGHT_PEDAL:
                return "right pedal";
        case CSCS_CLIENT_SENSOR_LOCATION_FRONT_HUB:
                return "front hub";
        case CSCS_CLIENT_SENSOR_LOCATION_REAR_DROPOUT:
                return "rear dropout";
        case CSCS_CLIENT_SENSOR_LOCATION_CHAINSTAY:
                return "chainstay";
        case CSCS_CLIENT_SENSOR_LOCATION_REAR_WHEEL:
                return "rear wheel";
        case CSCS_CLIENT_SENSOR_LOCATION_REAR_HUB:
                return "rear hub";
        case CSCS_CLIENT_SENSOR_LOCATION_CHEST:
                return "chest";
        default:
                return "unknown";
        }
}

static void cscs_read_sensor_location_completed_cb(ble_client_t *client, att_error_t status,
                                                        cscs_client_sensor_location_t location)
{
        if (status == ATT_ERROR_OK) {
                printf("\tSensor Location: %s\r\n", sensor_location_to_string(location));
        } else {
                printf("\tFailed to read Sensor Location (%#x)\r\n", status);
        }

        clear_pending_action(PENDING_ACTION_READ_SENSOR_LOCATION, status);
}

static void cscs_set_event_state_completed_cb(ble_client_t *cscs_client, cscs_client_event_t event,
                                                                                att_error_t status)
{
        /*
         * This is the only event supported right now, but more can be added in future versions so
         * need to check here.
         */
        if (event != CSCS_CLIENT_EVENT_CSC_MEASUREMENT_NOTIF) {
                return;
        }

        if (status == ATT_ERROR_OK) {
                printf("Measurement notifications %s\r\n", peer_info.notif_state ?
                                                                        "enabled" : "disabled");
        } else {
                printf("ERROR: failed to set measurement notifications (0x%02x)\r\n", status);
        }

        clear_pending_action(PENDING_ACTION_ENABLE_MEASUREMENT_NOTIF, status);
}

static void cscs_set_sc_control_point_state_completed_cb(ble_client_t *cscs_client, att_error_t status)
{
        if (status == ATT_ERROR_OK) {
                /*
                 * We only request to enable CP as it does not make sense to explicitly disable it
                 * at any time, thus we can assume CP was enabled on success here.
                 */
                printf("Control Point enabled\r\n");
        } else {
                printf("ERROR: failed to setup control point (0x%02x)\r\n", status);
        }

        clear_pending_action(PENDING_ACTION_ENABLE_CP, status);
}

static void cscs_get_supported_sensor_location_completed_cb(ble_client_t *client,
                cscs_client_status_t status, uint8_t locations_count, const uint8_t *locations)
{
        int i;

        OS_TIMER_STOP(cp_tmo_timer, OS_TIMER_FOREVER);

        if (status != CSCS_CLIENT_STATUS_SUCCESS) {
                printf("Failed to read supported sensor locations (0x%02x)\r\n", status);
                return;
        }

        printf("Supported sensor locations:\r\n");

        for (i = 0; i < locations_count; i++) {
                printf("\t%s (0x%02x)\r\n", sensor_location_to_string(locations[i]), locations[i]);
        }
}

static void cscs_set_cumulative_value_completed_cb(ble_client_t *client, cscs_client_status_t status)
{
        OS_TIMER_STOP(cp_tmo_timer, OS_TIMER_FOREVER);

        if (status == CSCS_CLIENT_STATUS_SUCCESS) {
                printf("Cumulative wheel revolution data updated\r\n");
        } else {
                printf("Failed to update cumulative wheel revolution data (0x%02x)\r\n", status);
        }
}

static void cscs_update_sensor_location_completed_cb(ble_client_t *client, cscs_client_status_t status)
{
        OS_TIMER_STOP(cp_tmo_timer, OS_TIMER_FOREVER);

        if (status == CSCS_CLIENT_STATUS_SUCCESS) {
                printf("Sensor location updated\r\n");
        } else {
                printf("Failed to update sensor location (0x%02x)\r\n", status);
        }
}

static const cscs_client_callbacks_t cscs_callbacks = {
        .read_csc_features_completed = cscs_read_csc_features_completed_cb,
        .csc_measurement = cscs_csc_measurement_cb,
        .read_sensor_location_completed = cscs_read_sensor_location_completed_cb,
        .set_event_state_completed = cscs_set_event_state_completed_cb,
        .set_sc_control_point_state_completed = cscs_set_sc_control_point_state_completed_cb,
        .request_supported_sensor_locations_completed = cscs_get_supported_sensor_location_completed_cb,
        .update_sensor_location_completed = cscs_update_sensor_location_completed_cb,
        .set_cumulative_value_completed = cscs_set_cumulative_value_completed_cb,
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
                                if (get_u16(p + idx) == UUID_SERVICE_CSCS) {
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

        if (peer_info.cscs_client) {
                ble_client_cleanup(peer_info.cscs_client);
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
        OS_ASSERT(evt->conn_idx == peer_info.conn_idx);

        if (evt->status == BLE_STATUS_OK) {
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
        OS_ASSERT(evt->conn_idx == peer_info.conn_idx);

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
        case UUID_SERVICE_CSCS:
                if (peer_info.cscs_client) {
                        return;
                }

                peer_info.cscs_client = cscs_client_init(&cscs_callbacks, evt);
                if (!peer_info.cscs_client) {
                        return;
                }

                ble_client_add(peer_info.cscs_client);
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
        OS_ASSERT(evt->conn_idx == peer_info.conn_idx);

        printf("Browse completed\r\n");

        if (!peer_info.cscs_client) {
                printf("\tCycling Speed and Cadence service not found\r\n");

                printf("Disconnecting...\r\n");
                ble_gap_disconnect(evt->conn_idx, BLE_HCI_ERROR_REMOTE_USER_TERM_CON);
                return;
        }

        printf("\tCycling Speed and Cadence service found\r\n");

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

        /* Read features */
        add_pending_action(PENDING_ACTION_READ_FEATURES);

        /* Read sensor location */
        if (cscs_client_get_capabilities(peer_info.cscs_client) & CSCS_CLIENT_CAP_SENSOR_LOCATION) {
                add_pending_action(PENDING_ACTION_READ_SENSOR_LOCATION);
        }

        /* Enable measurement notifications */
        add_pending_action(PENDING_ACTION_ENABLE_MEASUREMENT_NOTIF);

        printf("Querying device...\r\n");
}

void cscp_collector_task(void *params)
{
        cli_t cli;

        /* Register CLI */
        cli = cli_register(CLI_NOTIF, clicmd, clicmd_default_handler);

        /* Set device name */
        ble_gap_device_name_set("Black Orca CSC Collector", ATT_PERM_READ);

        /* Setup application in BLE Manager */
        ble_register_app();
        ble_central_start();

        /*
         * We have keyboard support (for CLI) but since support for passkey entry is missing, let's
         * just declare "Display Only" capabilities for now.
         */
        ble_gap_set_io_cap(GAP_IO_CAP_DISP_ONLY);

        /*
         * Create timer to control CP operations timeout - do not start it now, it will be started when needed.
         */
        cp_tmo_timer = OS_TIMER_CREATE("cptmo", OS_MS_2_TICKS(CP_OPERATION_TMO_MS), OS_FAIL,
                                                OS_UINT_TO_PTR(OS_GET_CURRENT_TASK()), timer_cb);

        printf("CSCP Collector ready.\r\n");

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

                if (notif & CP_OPERATION_TMO_NOTIF) {
                        cscs_client_sc_control_point_timeout(peer_info.cscs_client);
                }
        }
}