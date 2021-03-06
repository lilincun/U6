/**
 \addtogroup INTERFACES
 \{
 \addtogroup BLE
 \{
 \addtogroup MANAGER
 \{
 */

/**
 ****************************************************************************************
 * @file ble_mgr_config.h
 *
 * @brief BLE Manager Configuration
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

#ifndef BLE_MGR_CONFIG_H
#define BLE_MGR_CONFIG_H

#include "ble_config.h"

/**
 * \brief Directly call BLE manager handlers
 *
 * When #BLE_MGR_DIRECT_ACCESS is defined to 1, the respective BLE manager command handler will be
 * directly called by the BLE API calling task in the calling task's context and the BLE manager's
 * command queue will not be used to interface with the BLE framework.
 */
#ifndef BLE_MGR_DIRECT_ACCESS
#define BLE_MGR_DIRECT_ACCESS   1
#endif

/**
 * \brief DOWN queue size
 *
 * Defines the DOWN (client app to BLE manager) queue length, in number of messages
 *
 */
#ifndef BLE_MGR_COMMAND_QUEUE_LENGTH
#define BLE_MGR_COMMAND_QUEUE_LENGTH  1
#endif

/**
 * \brief UP queue size
 *
 * Defines the UP (BLE manager to application) queue length, in number of messages
 *
 */
#ifndef BLE_MGR_EVENT_QUEUE_LENGTH
#define BLE_MGR_EVENT_QUEUE_LENGTH    8
#endif

/**
 * \brief UP response queue size
 *
 * Defines the UP (BLE manager to application) response queue length, in number of messages
 *
 */
#define BLE_MGR_RESPONSE_QUEUE_LENGTH  1

#endif /* BLE_MGR_CONFIG_H */
/**
 \}
 \}
 \}
 */
