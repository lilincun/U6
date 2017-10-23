/**
 \addtogroup INTERFACES
 \{
 \addtogroup BLE
 \{
 \addtogroup ADAPTER
 \{
 */

/**
 ****************************************************************************************
 * @file ad_ble_config.h
 *
 * @brief BLE Adapter configuration
 *
 * Copyright (C) 2015. Dialog Semiconductor, unpublished work. This computer
 * program includes Confidential, Proprietary Information and is a Trade Secret of
 * Dialog Semiconductor. All use, disclosure, and/or reproduction is prohibited
 * unless authorized in writing. All Rights Reserved.
 *
 * <black.orca.support@diasemi.com> and contributors.
 *
 ****************************************************************************************
 */

#ifndef AD_BLE_CONFIG_H
#define AD_BLE_CONFIG_H

#include "queue.h"

/**
 * \brief UP Queue size
 *
 * Defines the UP (adapter to manager) Queue length, in number of messages, if not defined
 *
 */
#ifndef AD_BLE_EVENT_QUEUE_LENGTH
#define AD_BLE_EVENT_QUEUE_LENGTH    8
#endif

/**
 * \brief DOWN Queue size
 *
 * Defines the DOWN (manager to adapter) Queue length, in number of messages, if not defined
 *
 */
#ifndef AD_BLE_COMMAND_QUEUE_LENGTH
#define AD_BLE_COMMAND_QUEUE_LENGTH  8
#endif


#endif /* AD_BLE_CONFIG_H */
/**
 \}
 \}
 \}
 */
