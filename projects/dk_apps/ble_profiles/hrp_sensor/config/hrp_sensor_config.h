/**
****************************************************************************************
*
* @file hrp_sensor_config.h
*
* @brief Application configuration.
*
* Copyright (C) 2016. Dialog Semiconductor, unpublished work. This computer
* program includes Confidential, Proprietary Information and is a Trade Secret of
* Dialog Semiconductor. All use, disclosure, and/or reproduction is prohibited
* unless authorized in writing. All Rights Reserved.
*
* <black.orca.support@diasemi.com> and contributors.
*
****************************************************************************************
*/

#ifndef CONFIG_HRP_SENSOR_CONFIG_H_
#define CONFIG_HRP_SENSOR_CONFIG_H_

/**
 * Port and pin configuration for 16-bit HRP measurement value trigger
 */
#define CFG_SEND_16_BIT_VALUE_TRIGGER_GPIO_PORT   (HW_GPIO_PORT_1)
#define CFG_SEND_16_BIT_VALUE_TRIGGER_GPIO_PIN    (HW_GPIO_PIN_0)

/**
 * Port and pin configuration for starting advertising if it is turned off
 */
#define CFG_START_ADVERTISING_TRIGGER_GPIO_PORT   (HW_GPIO_PORT_1)
#define CFG_START_ADVERTISING_TRIGGER_GPIO_PIN    (HW_GPIO_PIN_6)

#endif /* CONFIG_HRP_SENSOR_CONFIG_H_ */
