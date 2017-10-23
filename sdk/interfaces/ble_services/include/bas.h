/**
 ****************************************************************************************
 *
 * @file bas.h
 *
 * @brief Battery Service sample implementation API
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

#ifndef BAS_H_
#define BAS_H_

#include <stdint.h>
#include "ble_service.h"

/**
 * Battery instance information
 *
 * This corresponds to contents of Content Presentation Format descriptor for Battery Level
 * characteristic. Other fields of this descriptor are set internally to proper values.
 */
typedef struct {
        uint8_t         namespace;      /**< namespace */
        uint16_t        descriptor;     /**< descriptor */
} bas_battery_info_t;

typedef struct
{
   uint16_t  dyear;
   uint8_t   dmouth;
   uint8_t   dday;
   uint8_t   dweek;
   uint8_t   dhour;
   uint8_t   dmin;
   uint8_t   dsec;
   uint16_t  dheart;
   uint16_t  drun;
   uint16_t  dfoot;
   uint16_t  dcarolie;
   uint16_t  dmileage;
}Display_Type;
extern   Display_Type  Displaymsg;//add

/**
 * Register Battery Service instance
 *
 * For application with single instance of BAS, \p info parameter is optional. Otherwise \p info
 * is mandatory as per BAS 1.0 specification since this will add Content Presentation Format
 * descriptor to Battery Level characteristic.
 *
 * \param [in] config   service configuration
 * \param [in] info     battery instance information
 *
 * \return service instance
 *
 */
ble_service_t *bas_init(const ble_service_config_t *config, const bas_battery_info_t *info);

/**
 * Notify client with battery level
 *
 * \param [in] svc      service instance
 * \param [in] conn_idx connection index
 *
 */
void bas_notify_level(ble_service_t *svc, uint16_t conn_idx);

/**
 * Set reported battery level
 *
 * \param [in] svc      service instance
 * \param [in] level    new battery level
 * \param [in] notify   true if clients shall be notified, false otherwise
 *
 */
void bas_set_level(ble_service_t *svc, uint8_t level, bool notify);

#endif /* BAS_H_ */
