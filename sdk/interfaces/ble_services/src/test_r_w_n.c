/**
 ****************************************************************************************
 *
 * @file lls.c
 *
 * @brief Link Loss Service sample implementation
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
#include "util/queue.h"
#include "ble_att.h"
#include "ble_bufops.h"
#include "ble_common.h"
#include "ble_gatts.h"
#include "ble_uuid.h"
#include "test_r_w_n.h"


#define UUID_SERVICE_TEST_R_W_N                        (0xaaaa) // TEST_R_W_N  Service
#define UUID_R_W_N                                     (0x1233)
#define UUID_R_W_N_1                                   (0x1234)

typedef struct {
        ble_service_t svc;

        // handles
        uint16_t test_r_w_n_val_h;
        uint16_t test_r_w_n_val_1_h;

        // callbacks
        //lls_alert_cb_t cb;
        //queue_t levels;
} test_r_w_n_service_t;


static void handle_read_req(ble_service_t *svc, const ble_evt_gatts_read_req_t *evt)
{
        test_r_w_n_service_t *test_r_w_n = (test_r_w_n_service_t *) svc;
        if (evt->handle == test_r_w_n->test_r_w_n_val_h )
        {
                static uint8_t level = 0;
                /* Default alert level - 'No Alert' */
                level++;
                printf("test_r_w_n_val_h level = %d\r\n",level);

                ble_gatts_read_cfm(evt->conn_idx, evt->handle, ATT_ERROR_OK, sizeof(level), &level);
        }
        printf("test_r_w_n handle_read_req\r\n");
}

static void handle_write_req(ble_service_t *svc, const ble_evt_gatts_write_req_t *evt)
{
        test_r_w_n_service_t *test_r_w_n = (test_r_w_n_service_t *) svc;
        printf("evt->length = %d\r\n",evt->length);
        printf("evt->value = %d\r\n",evt->value[0]);
        printf("evt->value = %d\r\n",evt->value[1]);
        if (evt->handle == test_r_w_n->test_r_w_n_val_h )
        {       att_error_t err = ATT_ERROR_OK;
                uint8_t level = get_u8(evt->value);
                ble_gatts_write_cfm(evt->conn_idx, evt->handle, err);
        }
        printf("test_r_w_n handle_write_req\r\n");
}

ble_service_t *test_r_w_n_init(void)
{
        test_r_w_n_service_t *test_r_w_n;
        uint16_t num_attr;
        att_uuid_t uuid;

        test_r_w_n = OS_MALLOC(sizeof(*test_r_w_n));
        memset(test_r_w_n, 0, sizeof(*test_r_w_n));

        //test_r_w_n->svc.disconnected_evt = handle_disconnected_evt;
        test_r_w_n->svc.read_req = handle_read_req;
        test_r_w_n->svc.write_req = handle_write_req;
        //test_r_w_n->cb = alert_cb;

        num_attr = ble_gatts_get_num_attr(0, 2, 0);

        ble_uuid_create16(UUID_SERVICE_TEST_R_W_N, &uuid);
        ble_gatts_add_service(&uuid, GATT_SERVICE_PRIMARY, num_attr);

        ble_uuid_create16(UUID_R_W_N, &uuid);
        ble_gatts_add_characteristic(&uuid,
                                     GATT_PROP_READ | GATT_PROP_WRITE,
                                     ATT_PERM_RW,
                                     //sizeof(uint8_t),
                                     20,
                                     GATTS_FLAG_CHAR_READ_REQ,
                                     NULL,
                                     &test_r_w_n->test_r_w_n_val_h);

        ble_uuid_create16(UUID_R_W_N_1, &uuid);
        ble_gatts_add_characteristic(&uuid,
                                     GATT_PROP_NOTIFY,
                                     ATT_PERM_RW,
                                     //sizeof(uint8_t),
                                     20,
                                     GATTS_FLAG_CHAR_READ_REQ,
                                     NULL,
                                     &test_r_w_n->test_r_w_n_val_1_h);

        ble_gatts_register_service(&test_r_w_n->svc.start_h,
                                   &test_r_w_n->test_r_w_n_val_h,
                                   &test_r_w_n->test_r_w_n_val_1_h, 0);

        test_r_w_n->svc.end_h = test_r_w_n->svc.start_h + num_attr;
        test_r_w_n->svc.write_req = handle_write_req;
        test_r_w_n->svc.read_req = handle_read_req;

        return &test_r_w_n->svc;
}
