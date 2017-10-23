/**
 \addtogroup UTILITIES
 \{
 */

/**
 ****************************************************************************************
 * @file rf_tools_common.h
 *
 * @brief RF Tools common part
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

#ifndef RF_TOOLS_COMMON_H
#define RF_TOOLS_COMMON_H

typedef void (*systick_cb_t)(void);

void rf_tools_start_systick(systick_cb_t cb, uint32_t ticks);
void rf_tools_stop_systick(void);

#endif /* RF_TOOLS_COMMON_H */
/**
 \}
 */
