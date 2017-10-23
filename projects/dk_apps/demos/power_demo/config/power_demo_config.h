/**
****************************************************************************************
*
* @file power_demo_config.h
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

#ifndef POWER_DEMO_CONFIG_H_
#define POWER_DEMO_CONFIG_H_

/**
 * Set power configuration over CLI
 */
#define POWER_DEMO_CLI_CONFIGURATION    (1)

/**
 * Set power configuration through GPIO settings
 */
#define POWER_DEMO_GPIO_CONFIGURATION   (0)

/**
 * Set config console write timeout
 */
#define CONFIG_CONSOLE_WRITE_TIMEOUT    (0x20)

#if POWER_DEMO_CLI_CONFIGURATION && POWER_DEMO_GPIO_CONFIGURATION
#error "Set one configuration only, POWER_DEMO_CLI_CONFIGURATION or POWER_DEMO_GPIO_CONFIGURATION"
#endif

#endif /* POWER_DEMO_CONFIG_H_ */
