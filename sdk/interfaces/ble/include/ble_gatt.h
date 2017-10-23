/**
 \addtogroup INTERFACES
 \{
 \addtogroup BLE
 \{
 \addtogroup API
 \{
 */

/**
 ****************************************************************************************
 *
 * @file ble_gatt.h
 *
 * @brief Common definitions for GATT API
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

#ifndef BLE_GATT_H_
#define BLE_GATT_H_

/** GATT service type */
typedef enum {
        GATT_SERVICE_PRIMARY,
        GATT_SERVICE_SECONDARY,
} gatt_service_t;

/** GATT event type */
typedef enum {
        GATT_EVENT_NOTIFICATION,
        GATT_EVENT_INDICATION,
} gatt_event_t;

/** GATT characteristic properties */
typedef enum {
        GATT_PROP_NONE                          = 0,
        GATT_PROP_BROADCAST                     = 0x0001,//01
        GATT_PROP_READ                          = 0x0002,//10
        GATT_PROP_WRITE_NO_RESP                 = 0x0004,//100
        GATT_PROP_WRITE                         = 0x0008,//1000
        GATT_PROP_NOTIFY                        = 0x0010,//10000
        GATT_PROP_INDICATE                      = 0x0020,
        GATT_PROP_WRITE_SIGNED                  = 0x0040,
        GATT_PROP_EXTENDED                      = 0x0080,
        GATT_PROP_EXTENDED_RELIABLE_WRITE       = 0x0100,
        GATT_PROP_EXTENDED_WRITABLE_AUXILIARIES = 0x0200,
} gatt_prop_t;

/** GATT Client Characteristic Configuration bitmask values */
typedef enum {
        GATT_CCC_NONE           = 0x0000,
        GATT_CCC_NOTIFICATIONS  = 0x0001,
        GATT_CCC_INDICATIONS    = 0x0002,
} gatt_ccc_t;

#endif /* BLE_GATT_H_ */
/**
 \}
 \}
 \}
 */
