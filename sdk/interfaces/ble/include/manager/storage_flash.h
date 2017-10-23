/**
 ****************************************************************************************
 * @file storage_flash.h
 *
 * @brief BLE Manager flash storage interface
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

#ifndef STORAGE_FLASH_H_
#define STORAGE_FLASH_H_

/**
 * Initialize flash storage
 *
 * This should be called only once on platform startup.
 *
 */
void storage_flash_init(void);

/**
 * Load BLE data from flash storage
 *
 * Loads data from BLE storage partition and updates device list.
 *
 */
void storage_flash_load(void);

/**
 * Save BLE data to flash storage
 *
 * Saves bonded devices data to BLE storage partition. This should be called when bonded devices are
 * modified.
 *
 */
void storage_flash_save(void);

#endif /* STORAGE_FLASH_H_ */
