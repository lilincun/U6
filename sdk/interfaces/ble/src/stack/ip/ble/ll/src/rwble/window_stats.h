/**
 ****************************************************************************************
 *
 * @file window_stats.h
 *
 * @brief Statistics about the RX window. 
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

#ifndef WINDOW_STATS_H_
#define WINDOW_STATS_H_


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <stdint.h>
#include <stdbool.h>
#include "rwip_config.h"
#include "compiler.h"
#include "co_error.h"

/*
 * Defines
 ****************************************************************************************
 */
#define WINSTAT_LOG_THRESHOLD                   10

/*
 * VARIABLE DECLARATIONS
 ****************************************************************************************
 */
extern uint32_t mst_sca;
extern uint32_t slv_sca;
extern uint32_t sca_drift;
extern uint32_t diff_pos;
extern uint32_t diff_neg;
extern uint32_t diff_zero;
extern uint32_t max_pos_diff;
extern uint32_t max_neg_diff;
extern uint32_t diff_events;
extern uint32_t sync_errors;
extern uint32_t type_errors;
extern uint32_t len_errors;
extern uint32_t crc_errors;
extern uint32_t stat_runs;

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */
void rxwin_calculate_lag(void);
int32_t rxwin_calibrate(void);

#endif // WINDOW_STATS_H_
