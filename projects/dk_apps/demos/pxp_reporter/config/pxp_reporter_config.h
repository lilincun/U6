/**
 ****************************************************************************************
 *
 * @file pxp_reporter_config.h
 *
 * @brief Proximity Reporter configuration
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

#ifndef PXP_REPORTER_CONFIG_H_
#define PXP_REPORTER_CONFIG_H_

/* Name can not be longer than 29 bytes (BLE_SCAN_RSP_LEN_MAX - 2 bytes)*/
#define PX_REPORTER_DEFAULT_NAME  "U6-2"//"Dialog PX Reporter"//

/* Interval (in ms) for checking battery level (can be configured externally as well) */
#ifndef PX_REPORTER_BATTERY_CHECK_INTERVAL
#define PX_REPORTER_BATTERY_CHECK_INTERVAL      1000//60000
#endif

/*
 * When set to non-zero, SUOTA build will request longer connection interval in order to reduce
 * power consumption when connection is active and change back to short connection interval before
 * starting SUOTA process. This however seem to cause interoperability problems with some devices.
 *
 * This parameter is not applicable for non-SUOTA builds since they will always use longer connection
 * interval when possible.
 */
#define PX_REPORTER_SUOTA_POWER_SAVING  (0)

#endif /* PXP_REPORTER_CONFIG_H_ */
