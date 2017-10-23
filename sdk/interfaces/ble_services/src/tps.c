/**
 ****************************************************************************************
 *
 * @file tps.c
 *
 * @brief Tx Power Service sample implementation
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
#include "osal.h"
#include "ble_att.h"
#include "ble_bufops.h"
#include "ble_common.h"
#include "ble_gatts.h"
#include "ble_uuid.h"
#include "tps.h"

#define UUID_TX_POWER_LEVEL     (0x2A07)

typedef struct {
        ble_service_t svc;
} tp_service_t;

ble_service_t *tps_init(int8_t level)
{
        tp_service_t *tps;
        uint16_t num_attr;
        uint16_t tpl_val_h;
        att_uuid_t uuid;

        tps = OS_MALLOC(sizeof(*tps));

        num_attr = ble_gatts_get_num_attr(0, 1, 0);

        ble_uuid_create16(UUID_SERVICE_TPS, &uuid);
        ble_gatts_add_service(&uuid, GATT_SERVICE_PRIMARY, num_attr);

        ble_uuid_create16(UUID_TX_POWER_LEVEL, &uuid);
        ble_gatts_add_characteristic(&uuid, GATT_PROP_READ, ATT_PERM_READ, sizeof(int8_t), 0,
                                                                                NULL, &tpl_val_h);

        ble_gatts_register_service(&tps->svc.start_h, &tpl_val_h, 0);

        ble_gatts_set_value(tpl_val_h, sizeof(level), &level);

        tps->svc.end_h = tps->svc.start_h + num_attr;

        return &tps->svc;
}
