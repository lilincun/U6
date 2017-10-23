/**
 ****************************************************************************************
 *
 * @file pxp_reporter_task.c
 *
 * @brief PXP profile app implementation
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
#include <string.h>
#include "osal.h"
#include "ble_att.h"
#include "ble_common.h"
#include "ble_gap.h"
#include "ble_gatts.h"
#include "ble_l2cap.h"
#include "util/list.h"
#include "bas.h"
#include "ias.h"
#include "lls.h"
#include "tps.h"

#include "test_r_w_n.h"

#include "sys_power_mgr.h"
#include "sys_socf.h"
#include "ad_nvparam.h"
#include "ad_battery.h"
#include "hw_breath.h"
#include "hw_gpio.h"
#include "hw_led.h"
#include "sys_watchdog.h"
#include "platform_nvparam.h"
#include "pxp_reporter_config.h"

#include "stdio.h"

//////////////////////////////////////////////////////////////
#include "sample.h"//sku  ble

#if dg_configSUOTA_SUPPORT
#include "dis.h"
#include "dlg_suota.h"
#include "sw_version.h"
#endif

#if dg_configSUOTA_SUPPORT
/*
 * Store information about ongoing SUOTA.
 */
INITIALISED_PRIVILEGED_DATA static bool suota_ongoing = false;
#endif

/*
 * Notification bits reservation
 *
 * Bit #0 is always assigned to BLE event queue notification.
 */
#define ALERT_TMO_NOTIF (1 << 1)
#define ADV_TMO_NOTIF   (1 << 2)
#define BAS_TMO_NOTIF   (1 << 3)

#define DATA_NOTIF   (1 << 5)
/*
 * PXP Update connection parameters notif mask
 */
#define PXP_UPDATE_CONN_PARAM_NOTIF     (1 << 4)

/*
 * The maximum length of name in scan response
 */
#define MAX_NAME_LEN    (BLE_SCAN_RSP_LEN_MAX - 2)

/*
 * PXP advertising and scan response data
 *
 * While not required, PXP specification states that PX reporter device using peripheral role can
 * advertise support for LLS. Device name is set in scan response to make it easily recognizable.
 */
///////////////////////////////////////////////////////////////////////////////////////
static const uint8_t adv_data[] = {
#if dg_configSUOTA_SUPPORT
        0x07, GAP_DATA_TYPE_UUID16_LIST_INC,
        0x03, 0x18, // = 0x1803 (LLS UUID)
        0x02, 0x18, // = 0x1802 (IAS UUID)
        0xF5, 0xFE, // = 0xFEF5 (DIALOG SUOTA UUID)
#else
        0x05, GAP_DATA_TYPE_UUID16_LIST_INC,
        0x03, 0x18, // = 0x1803 (LLS UUID)
        0x02, 0x18, // = 0x1802 (IAS UUID)
#endif
};

/*
 * PXP advertising interval values
 *
 * Recommended advertising interval values as defined by PXP specification. By default "fast connection"
 * is used.
 */


//////////////////////////////////////////////////////////////////////////
static const struct {
        uint16_t min;
        uint16_t max;
} adv_intervals[2] = {
        // "fast connection" interval values
        {
                .min = BLE_ADV_INTERVAL_FROM_MS(20),      // 20ms
                .max = BLE_ADV_INTERVAL_FROM_MS(30),      // 30ms
        },
        // "reduced power" interval values
        {
                .min = BLE_ADV_INTERVAL_FROM_MS(1000),    // 1000ms
                .max = BLE_ADV_INTERVAL_FROM_MS(1500),    // 1500ms
        }
};

typedef enum {
        ADV_INTERVAL_FAST = 0,
        ADV_INTERVAL_POWER = 1,
} adv_setting_t;

/* Battery Service instance */
PRIVILEGED_DATA static ble_service_t *bas;

//////////////////////////////////////////////////////////////////////////////
/* sample Service instance */
PRIVILEGED_DATA static ble_service_t *sample_service;

static void sample_rx_data_cb(ble_service_t*svc, uint16_t conn_idx, const uint8_t *value,uint16_t length)
{
        sample_tx_data(svc, conn_idx, value, length); // WILL NOTIFY WHAT VALUE HAS BEEN WRITTEN in the RX char.
}
static void sample_tx_done_cb(ble_service_t*svc, uint16_t conn_idx, uint16_t length)
{
        // The notification is already sent out
}
static void sample_set_flow_control_cb(ble_service_t*svc, uint16_t conn_idx, sample_flow_control_t value)
{
        sample_set_flow_control(svc, conn_idx, value); // Enable the flow control
}
static sample_callbacks_t sample_callbacks=
{       /** Data received from remote client */
        .rx_data = sample_rx_data_cb,//c8
        //Click on the Server RX Data characteristic and write a value

        /** Service finished TX transaction */
        .tx_done = sample_tx_done_cb,//c9
        //Click on the Server TX Data characteristic and click on LISTEN FOR NOTIFICATIONS

        .set_flow_control = sample_set_flow_control_cb,//ca
        //Click on the Flow control characteristic and write 0x01 in order to enable flow control.
};

//////////////////////////////////////////////////////////////////////////////

/* Timer used to switch from "fast connection" to "reduced power" advertising intervals */
PRIVILEGED_DATA static OS_TIMER adv_tim;

/* Timer used to disable alert after timeout */
PRIVILEGED_DATA static OS_TIMER alert_tim;

/* Timer used for battery monitoring */
//PRIVILEGED_DATA static OS_TIMER bas_tim;
OS_TIMER bas_tim;

struct device {
        struct device *next;
        bd_address_t addr;
};

typedef struct {
        void           *next;

        bool            expired;

        uint16_t        conn_idx; //< Connection index

        OS_TIMER        param_timer;
        OS_TASK         current_task;
} conn_dev_t;

/* List of devices pending reconnection */
PRIVILEGED_DATA static void *reconnection_list;

/* List of devices waiting for connection parameters update */
PRIVILEGED_DATA static void *param_connections;

/* Buffer must have length at least max_len + 1 */
static uint16_t read_name(uint16_t max_len, char *name_buf)
{
        uint16_t read_len = 0;

#if dg_configNVPARAM_ADAPTER
        nvparam_t param;
        param = ad_nvparam_open("ble_app");
        read_len = ad_nvparam_read(param, TAG_BLE_APP_NAME, max_len, name_buf);
        ad_nvparam_close(param);
#endif /* dg_configNVPARAM_ADAPTER */

        if (1)//read_len == 0À¶ÑÀÃû³ÆÏÔÊ¾ÓÐÎó
        {
                strcpy(name_buf, PX_REPORTER_DEFAULT_NAME);
                return strlen(PX_REPORTER_DEFAULT_NAME);
        }

        name_buf[read_len] = '\0';

        return read_len;
}

static bool device_match_addr(const void *elem, const void *ud)
{
        const struct device *dev = elem;
        const bd_address_t *addr = ud;

        return !memcmp(&dev->addr, addr, sizeof(*addr));
}

/* Advertising intervals change timeout timer callback */
static void adv_tim_cb(OS_TIMER timer)
{
        OS_TASK task = (OS_TASK) OS_TIMER_GET_TIMER_ID(timer);

        OS_TASK_NOTIFY(task, ADV_TMO_NOTIF, OS_NOTIFY_SET_BITS);
}

static void set_advertising_interval(adv_setting_t setting)
{
        uint16_t min = adv_intervals[setting].min;
        uint16_t max = adv_intervals[setting].max;

        ble_gap_adv_intv_set(min, max);
}

/* Alert timeout timer callback */
static void alert_tim_cb(OS_TIMER timer)
{
        OS_TASK task = (OS_TASK) OS_TIMER_GET_TIMER_ID(timer);

        OS_TASK_NOTIFY(task, ALERT_TMO_NOTIF, OS_NOTIFY_SET_BITS);
}

static void set_alerting(bool new_alerting)
{
        PRIVILEGED_DATA static bool alerting = false;

        /*
         * Breath timer used for alerting does not work in sleep mode thus device cannot go to sleep
         * if it's alerting. Also pm_stay_alive() and pm_resume_sleep() has to be always called the
         * same number of times so this adds required logic to do this.
         */

        if (new_alerting == alerting) {
                return;
        }

        alerting = new_alerting;

        if (alerting) {
                pm_stay_alive();
        } else {
                pm_resume_sleep();
        }
}

/* Configure alerting for given level */
static void do_alert(uint8_t level)
{

        breath_config config = {
                .dc_min = 0,
                .dc_max = 255,
                .freq_div = 255,
                .polarity = HW_BREATH_PWM_POL_POS
        };

        switch (level) {
        case 1:
                set_alerting(true);
                config.dc_step = 96;
                break;
        case 2:
                set_alerting(true);
                config.dc_step = 32;
                break;
        default:
                set_alerting(false);
                /* Simply disable breath timer and return */
                hw_breath_disable();
                return;
        }

        /* Configure and enable breath timer */
        hw_breath_init(&config);
        hw_led_set_led1_src(HW_LED_SRC1_BREATH);
        hw_led_enable_led1(true);
        hw_breath_enable();
}

/* Alert callback from IAS (peer has written new value) */
static void ias_alert_cb(uint16_t conn_idx, uint8_t level)
{
        do_alert(level);
}

/*static void hrs_alert_cb(uint16_t conn_idx, uint8_t level)
{
        do_alert(level);
}*/

/* Alert callback from LLS (link to peer was lost) */
static void lls_alert_cb(uint16_t conn_idx, const bd_address_t *address, uint8_t level)
{
        struct device *dev;

        if (level == 0) {
                return;
        }

        /* Add alerting device to reconnection list */
        dev = OS_MALLOC(sizeof(*dev));
        memcpy(&dev->addr, address, sizeof(dev->addr));
        list_add(&reconnection_list, dev);

        /* Trigger an alert */
        do_alert(level);

        /* (Re)start alert timeout timer */
        OS_TIMER_RESET(alert_tim, OS_TIMER_FOREVER);

        /*
         * Set interval values to "fast connect" and restart advertising timer. Stop advertising
         * so it can be restarted with new interval values.
         */
        OS_TIMER_RESET(adv_tim, OS_TIMER_FOREVER);
        set_advertising_interval(ADV_INTERVAL_FAST);
        ble_gap_adv_stop();
}

/* Match connection by connection index */
static bool conn_params_match(const void *elem, const void *ud)
{
        conn_dev_t *conn_dev = (conn_dev_t *) elem;
        uint16_t conn_idx = (uint16_t) (uint32_t) ud;

        return conn_dev->conn_idx == conn_idx;
}

/*
 * This timer callback notifies task that time for discovery, bonding and encryption
 * elapsed, and connection parameters can be changed to preferred ones.
 */
static void conn_params_timer_cb(OS_TIMER timer)
{
        conn_dev_t *conn_dev = (conn_dev_t *) OS_TIMER_GET_TIMER_ID(timer);;

        conn_dev = list_find(param_connections, conn_params_match,
                                                (const void *) (uint32_t) conn_dev->conn_idx);
        if (conn_dev) {
                conn_dev->expired = true;
                ////////////////////////////////////////
                //notification  sent to update the connection parameters
                OS_TASK_NOTIFY(conn_dev->current_task, PXP_UPDATE_CONN_PARAM_NOTIF,
                                                                        OS_NOTIFY_SET_BITS);
        }
}

#if !dg_configSUOTA_SUPPORT || PX_REPORTER_SUOTA_POWER_SAVING
/* Update connection parameters */
///////////////////////////////////////////////////////////////
static void conn_param_update(uint16_t conn_idx)
{
        gap_conn_params_t cp;

        cp.interval_min = defaultBLE_PPCP_INTERVAL_MIN;
        cp.interval_max = defaultBLE_PPCP_INTERVAL_MAX;
        cp.slave_latency = defaultBLE_PPCP_SLAVE_LATENCY;
        cp.sup_timeout = defaultBLE_PPCP_SUP_TIMEOUT;

        ble_gap_conn_param_update(conn_idx, &cp);
}
#endif

#if dg_configSUOTA_SUPPORT && PX_REPORTER_SUOTA_POWER_SAVING
/* Update connection parameters for SUOTA */
static void conn_param_update_for_suota(uint16_t conn_idx)
{
        gap_conn_params_t cp;

        cp.interval_min = BLE_CONN_INTERVAL_FROM_MS(20);    // 20ms
        cp.interval_max = BLE_CONN_INTERVAL_FROM_MS(60);    // 60ms
        cp.slave_latency = 0;
        cp.sup_timeout = BLE_SUPERVISION_TMO_FROM_MS(2000); // 2000ms

        ble_gap_conn_param_update(conn_idx, &cp);
}
#endif

#if dg_configUSE_SOC
static uint8_t read_battery_level(void)
{
        int16_t level;

        /*
         * The return value from soc_get_soc is from 0(0%) to 1000(100.0%).
         * The input parameter of bas_set_level is from 0(0%) to 100(100%).
         */
        level = (socf_get_soc() + 5) / 10;

        return level;
}
#else
/*
 * The values depend on the battery type.
 * MIN_BATTERY_LEVEL (in mVolts) must correspond to dg_configBATTERY_LOW_LEVEL (in ADC units)
 */
#define MAX_BATTERY_LEVEL 4200
#define MIN_BATTERY_LEVEL 2800

static uint8_t bat_level(uint16_t voltage)
{
        if (voltage >= MAX_BATTERY_LEVEL) {
                return 100;
        } else if (voltage <= MIN_BATTERY_LEVEL) {
                return 0;
        }

        /*
         * For demonstration purposes discharging (Volt vs Capacity) is approximated
         * by a linear function. The exact formula depends on the specific battery being used.
         */
        return (uint8_t) ((int) (voltage - MIN_BATTERY_LEVEL) * 100 /
                                                        (MAX_BATTERY_LEVEL - MIN_BATTERY_LEVEL));
}

static uint8_t read_battery_level(void)
{
        uint8_t level;

        battery_source bat = ad_battery_open();
        uint16_t bat_voltage = ad_battery_raw_to_mvolt(bat, ad_battery_read(bat));
        level = bat_level(bat_voltage);
        ad_battery_close(bat);

        return level;
}
#endif

static void bas_update(void)
{
#if 1
        static uint8_t level = 0;
        if(level >=100)
                level = 0;
        printf("bas_update level = %d\r\n",level);
        bas_set_level(bas, level, true);
        level+=10;
#else
        uint8_t level;
        level = read_battery_level();
        bas_set_level(bas, level, true);
#endif

}

static void data_update(void)
{
     data_updata_level(bas, true);
}
////////////////////////////////////////////////
static void handle_evt_gap_connected(ble_evt_gap_connected_t *evt)
{
        struct device *dev;
        conn_dev_t *conn_dev;
        printf("gap_connected 1\r\n");
        /* Start battery monitoring if not yet started, but first update current battery level */
        if (!OS_TIMER_IS_ACTIVE(bas_tim)) {
                bas_update();
                OS_TIMER_START(bas_tim, OS_TIMER_FOREVER);
        }

        /*
         * Add timer that when expired will renegotiate connection parameters.
         */
        conn_dev = OS_MALLOC(sizeof(*conn_dev));
        if (conn_dev) {
                conn_dev->conn_idx = evt->conn_idx;
                conn_dev->expired = false;
                conn_dev->current_task = OS_GET_CURRENT_TASK();
                //////////////////////////////////////////////
                //conn_params_timer_cb  callback function which will be triggered
                conn_dev->param_timer = OS_TIMER_CREATE("conn_param", OS_MS_2_TICKS(5000),
                                        OS_TIMER_FAIL, (uint32_t) conn_dev, conn_params_timer_cb);

                list_append(&param_connections, conn_dev);
                OS_TIMER_START(conn_dev->param_timer, OS_TIMER_FOREVER);
        }

        /*
         * Try to unlink device with same address from reconnection list - if found, this is
         * reconnection and we should stop alerting and also clear reconnection list and disable
         * timer.
         */
        dev = list_unlink(&reconnection_list, device_match_addr, &evt->peer_address);
        if (dev) {
                do_alert(0);

                list_free(&reconnection_list, NULL, NULL);

                OS_TIMER_STOP(alert_tim, OS_TIMER_FOREVER);

                /*
                 * Device is reconnected so set interval values immediately to "reduced power" and
                 * stop advertising timer. Stop advertising so it can be restarted with new interval
                 * values.
                 */
                OS_TIMER_STOP(adv_tim, OS_TIMER_FOREVER);
                set_advertising_interval(ADV_INTERVAL_POWER);
                ble_gap_adv_stop();
        }
}
extern uint8_t ble_up_data ;
extern uint8_t read_sections ;
static void handle_evt_gap_disconnected(ble_evt_gap_disconnected_t *evt)
{
        size_t num_connected;
        conn_dev_t *conn_dev = list_unlink(&param_connections, conn_params_match,
                                                        (const void *) (uint32_t) evt->conn_idx);

        /*
         * Device is still in the list if disconnect happened before timer expired.
         * In this case stop the timer and free memory.
         */
        if (conn_dev) {
                OS_TIMER_DELETE(conn_dev->param_timer, OS_TIMER_FOREVER);
                OS_FREE(conn_dev);
        }

        /* Switch back to fast advertising interval. */
        set_advertising_interval(ADV_INTERVAL_FAST);
        ble_gap_adv_stop();
        OS_TIMER_START(adv_tim, OS_TIMER_FOREVER);

        /*
         * Stop monitoring battery level if no one is connected.
         */
        ble_gap_get_devices(GAP_DEVICE_FILTER_CONNECTED, NULL, &num_connected, NULL);
        if (num_connected == 0) {
                OS_TIMER_STOP(bas_tim, OS_TIMER_FOREVER);

                ble_up_data = 1; //sku
                read_sections = 0;//sku
        }
}

static void handle_evt_gap_adv_completed(ble_evt_gap_adv_completed_t *evt)
{
        /*
         * If advertising is completed, just restart it. It's either because new client connected
         * or it was cancelled in order to change interval values.
         */
#if dg_configSUOTA_SUPPORT
        /* If SUOTA is ongoing don't start advertising. */
        if (suota_ongoing) {
                return;
        }
#endif
        ble_gap_adv_start(GAP_CONN_MODE_UNDIRECTED);
}

static void handle_evt_gap_pair_req(ble_evt_gap_pair_req_t *evt)
{
        ble_gap_pair_reply(evt->conn_idx, true, evt->bond);
}

static void handle_evt_gap_pair_completed(ble_evt_gap_pair_completed_t *evt)
{
}

static void handle_evt_gap_sec_level_changed(ble_evt_gap_sec_level_changed_t *evt)
{
}

#if dg_configSUOTA_SUPPORT

/* Callback from SUOTA implementation */
static bool suota_ready_cb(void)
{
#if PX_REPORTER_SUOTA_POWER_SAVING
        gap_device_t dev;
        size_t dev_num = 1;
#endif

        /*
         * This callback is used so application can accept/block SUOTA.
         * Also, before SUOTA starts, user might want to do some actions
         * e.g. disable sleep mode.
         *
         * If true is returned, then advertising is stopped and SUOTA
         * is started. Otherwise SUOTA is canceled.
         *
         */
        suota_ongoing = true;

#if PX_REPORTER_SUOTA_POWER_SAVING
        /*
         * We need to decrease connection interval for SUOTA so data can be transferred quickly.
         * At the moment SUOTA does not provide information about connection on which it was
         * started. but since it can be completed only when there is one device connected we
         * assume proper device is the first (and only) device connected.
         */
        ble_gap_get_devices(GAP_DEVICE_FILTER_CONNECTED, NULL, &dev_num, &dev);
        if (dev_num > 0) {
                conn_param_update_for_suota(dev.conn_idx);
        }
#endif

        return true;
}

static void suota_status_changed_cb(uint8_t status, uint8_t error_code)
{
#if PX_REPORTER_SUOTA_POWER_SAVING
        gap_device_t dev;
        size_t dev_num = 1;

        /*
         * In case SUOTA finished with an error, we just restore default connection parameters.
         */

        if (status != SUOTA_ERROR) {
                return;
        }

        ble_gap_get_devices(GAP_DEVICE_FILTER_CONNECTED, NULL, &dev_num, &dev);
        if (dev_num > 0) {
                conn_param_update(dev.conn_idx);
        }
#endif
}

static const suota_callbacks_t suota_cb = {
        .suota_ready = suota_ready_cb,
        .suota_status = suota_status_changed_cb,
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
        .hw_revision = "Rev.D",
        .fw_revision = "1.0",
        .sw_revision = BLACKORCA_SW_VERSION,
};
#endif

static void bas_tim_cb(OS_TIMER timer)
{
        OS_TASK task = (OS_TASK) OS_TIMER_GET_TIMER_ID(timer);

        OS_TASK_NOTIFY(task, BAS_TMO_NOTIF, OS_NOTIFY_SET_BITS);
}

void pxp_reporter_task(void *params)
{
        ble_service_t *svc;
        int8_t tx_power_level;
        int8_t wdog_id;
#if dg_configSUOTA_SUPPORT
        ble_service_t *suota;
#endif
        uint16_t name_len;
        char name_buf[MAX_NAME_LEN + 1];        /* 1 byte for '\0' character */
        uint8_t scan_rsp[BLE_SCAN_RSP_LEN_MAX] = {
                0x00, GAP_DATA_TYPE_LOCAL_NAME, /* Parameter length must be set after */
                /* Name will be put here */
        };

        /* register pxp task to be monitored by watchdog */
        wdog_id = sys_watchdog_register(false);

        ble_peripheral_start();
        ble_register_app();

#if dg_configSUOTA_SUPPORT
        /* Set maximum allowed MTU to increase SUOTA throughput */
        ble_gap_mtu_size_set(512);
#endif

        /* Get name from nvparam if exist or default otherwise */
        name_len = read_name(MAX_NAME_LEN, name_buf);

        /*
         * Set device name
         */
        ble_gap_device_name_set(name_buf, ATT_PERM_READ);

        /*
         * Set scan response data
         */
        scan_rsp[0] = name_len + 1; /* 1 byte for data type */
        memcpy(&scan_rsp[2], name_buf, name_len);

        /*
         * Register IAS and LLS
         *
         * Both IAS and LLS instances should be registered in ble_service framework in order for
         * events inside service to be processed properly.
         */
        svc = ias_init(ias_alert_cb);
        ble_service_add(svc);

        svc = lls_init(lls_alert_cb);
        ble_service_add(svc);

     //   svc = hrs_init(HRS_SENSOR_LOC_FINGER,);
     //   ble_service_add(svc);

        /*
         * Register TPS
         *
         * TPS doesn't contain any dynamic data thus it doesn't need to be registered in ble_service
         * framework (but it's not an error to do so). For now we have output power set to 0 dBm.
         */
        tx_power_level = 0;
        tps_init(tx_power_level);

        /*
         * Register BAS
         *
         * BAS is not included in PXP, but it can be so PXP monitor can also monitor out battery
         * level. This service should also be registered in ble_service framework.
         */
        bas = bas_init(NULL, NULL);
        ble_service_add(bas);

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // sample_service = sample_init(&sample_callbacks); //sps
        // ble_service_add(sample_service);

        //svc = test_r_w_n_init( );
        //ble_service_add(svc);

//////////////////////////////////////////////////////////////////////////////////////////////////////////////

#if dg_configSUOTA_SUPPORT
        /*
         * Register SUOTA
         *
         * SUOTA instance should be registered in ble_service framework in order for events inside
         * service to be processed properly.
         */
        suota = suota_init(&suota_cb);
        OS_ASSERT(suota != NULL);
        ble_service_add(suota);

        /*
         * Register DIS
         *
         * DIS doesn't contain any dynamic data thus it doesn't need to be registered in ble_service
         * framework (but it's not an error to do so).
         */
        dis_init(NULL, &dis_info);
#endif

        /*
         * Create timer for LLS which will be started to timeout alarm if no reconnection occured
         * within 15 seconds.
         */
        alert_tim = OS_TIMER_CREATE("lls", OS_MS_2_TICKS(15000), OS_TIMER_FAIL,
                                                (void *) OS_GET_CURRENT_TASK(), alert_tim_cb);

        /*
         * Create timer for switching from "fast connection" to "reduced power" advertising
         * intervals after 30 seconds.
         */
        adv_tim = OS_TIMER_CREATE("adv", OS_MS_2_TICKS(30000), OS_TIMER_FAIL,
                                                (void *) OS_GET_CURRENT_TASK(), adv_tim_cb);

        /*
         * Create timer for battery monitoring.
         */
        bas_tim = OS_TIMER_CREATE("bas", OS_MS_2_TICKS(PX_REPORTER_BATTERY_CHECK_INTERVAL), true,
                                                (void *) OS_GET_CURRENT_TASK(), bas_tim_cb);

        /*
         * Set advertising data and scan response, then start advertising.
         *
         * By default, interval values are set to "fast connect" and timer is started to change
         * them after
         */
        ///////////////////////////////////////////////////////////////////////////
        set_advertising_interval(ADV_INTERVAL_FAST);//fast
        ble_gap_adv_data_set(sizeof(adv_data), adv_data, name_len + 2, scan_rsp);
        ////////////////////////////////////////////////////////////////////////////////
        ble_gap_adv_start(GAP_CONN_MODE_UNDIRECTED);//start  adv   and mode

        OS_TIMER_START(adv_tim, OS_TIMER_FOREVER);

        //bas_update();
        printf("...into_pxp_reporter_task...");
        for (;;) {
                OS_BASE_TYPE ret __attribute__((unused));
                uint32_t notif;

                /* notify watchdog on each loop */
                sys_watchdog_notify(wdog_id);

                /* suspend watchdog while blocking on OS_TASK_NOTIFY_WAIT() */
                sys_watchdog_suspend(wdog_id);

                /*
                 * Wait on any of the notification bits, then clear them all
                 */

                ret = OS_TASK_NOTIFY_WAIT(0, OS_TASK_NOTIFY_ALL_BITS, &notif, OS_TASK_NOTIFY_FOREVER);
                /* Blocks forever waiting for the task notification. Therefore, the return value must
                 * always be OS_OK
                 */


                OS_ASSERT(ret == OS_OK);

                /* resume watchdog */
                sys_watchdog_notify_and_resume(wdog_id);

                /* Notified from BLE Manager? */
                if (notif & BLE_APP_NOTIFY_MASK) {
                        ble_evt_hdr_t *hdr;

                        hdr = ble_get_event(false);
                        if (!hdr) {
                                goto no_event;
                        }

                        /*
                         * First, application needs to try pass event through ble_framework.
                         * Then it can handle it itself and finally pass to default event handler.
                         */
                        if (!ble_service_handle_event(hdr)) {
                                switch (hdr->evt_code) {
                                case BLE_EVT_GAP_CONNECTED:
                                        handle_evt_gap_connected((ble_evt_gap_connected_t *) hdr);
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
                                case BLE_EVT_GAP_PAIR_COMPLETED:
                                        handle_evt_gap_pair_completed((ble_evt_gap_pair_completed_t *) hdr);
                                        break;
                                case BLE_EVT_GAP_SEC_LEVEL_CHANGED:
                                        handle_evt_gap_sec_level_changed((ble_evt_gap_sec_level_changed_t *) hdr);
                                        break;
#if dg_configSUOTA_SUPPORT && defined (SUOTA_PSM)
                                case BLE_EVT_L2CAP_CONNECTED:
                                case BLE_EVT_L2CAP_DISCONNECTED:
                                case BLE_EVT_L2CAP_DATA_IND:
                                        suota_l2cap_event(suota, hdr);
                                        break;
#endif
                                default:
                                  ////////////////////////////////////////////////////////
                                        ble_handle_event_default(hdr);
                                        break;
                                }
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

                /* Notified from LLS timer? */
                if (notif & ALERT_TMO_NOTIF) {
                        /* Stop alerting and clear reconnection list */
                        do_alert(0);
                        list_free(&reconnection_list, NULL, NULL);
                }

                /* Notified from advertising timer? */
                if (notif & ADV_TMO_NOTIF) {
                        /*
                         * Change interval values and stop advertising. Once it's stopped, it will
                         * start again with new parameters.
                         */
                        set_advertising_interval(ADV_INTERVAL_POWER);
                        ble_gap_adv_stop();
                }

                /* Notified battery timer? */
                if (notif & BAS_TMO_NOTIF) {
                        /*
                         * Read battery level, and notify clients if level changed.
                         */
                        //bas_update();
                        printf("prepare_into_data_update()");//add
                        data_update();
                }

                /* Notified data*/
                if (notif & DATA_NOTIF) {
                        /*
                         * Read battery level, and notify clients if level changed.
                         */
                        printf("2333333333333\r\n");
                        //data_update();
                }

                /* Timer for fast connection expired, try set reduce power connection params */
                if (notif & PXP_UPDATE_CONN_PARAM_NOTIF) {
                        conn_dev_t *conn_dev = param_connections;

                        if (conn_dev && conn_dev->expired) {
                                param_connections = conn_dev->next;

#if dg_configSUOTA_SUPPORT
                                /*
                                 * Ignore this if SUOTA is ongoing - it's possible to start SUOTA
                                 * before power-saving parameters are applied so this would switch
                                 * to long connection interval.
                                 */
                                if (!suota_ongoing) {
#endif
/////////////////////////////////////////////////////
#if !dg_configSUOTA_SUPPORT || PX_REPORTER_SUOTA_POWER_SAVING
                                //conn_param_update(conn_dev->conn_idx); //change  link gap
#endif
#if dg_configSUOTA_SUPPORT
                                }
#endif

                                OS_TIMER_DELETE(conn_dev->param_timer, OS_TIMER_FOREVER);
                                OS_FREE(conn_dev);

                                /*
                                 * If the queue is not empty reset bit and check if timer expired
                                 * next time
                                 */
                                if (param_connections) {
                                        OS_TASK_NOTIFY(OS_GET_CURRENT_TASK(),
                                                PXP_UPDATE_CONN_PARAM_NOTIF, OS_NOTIFY_SET_BITS);
                                }
                        }
                }
        }
}

