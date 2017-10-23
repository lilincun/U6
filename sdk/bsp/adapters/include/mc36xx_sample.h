/*****************************************************************************
 *
 * Copyright (c) 2015 mCube, Inc.  All rights reserved.
 *
 * This source is subject to the mCube Software License.
 * This software is protected by Copyright and the information and source code
 * contained herein is confidential. The software including the source code
 * may not be copied and the information contained herein may not be used or
 * disclosed except with the written permission of mCube Inc.
 *
 * All other rights reserved.
 *****************************************************************************/

#ifndef MC36XX_SAMPLE_H
#define MC36XX_SAMPLE_H



#define SUPPORT_PEDOMETER         1
#define SUPPORT_SLEEPMETER        0
#define SUPPORT_LIFTARM           0
#define SUPPORT_LONGSIT           0
        

#define MC36XX_ACC_FIFO_POLLING

#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>


#if SUPPORT_PEDOMETER
#include "mcube_pedometer_hooks.h"	
#include "mcube_pedo_interface.h"
#endif 
#if SUPPORT_SLEEPMETER
#include "mcube_sleepmeter.h"
#endif 
#if SUPPORT_LIFTARM 	
#include "mcube_liftarm.h"
#endif 	
#if SUPPORT_LONGSIT
#include "mcube_longsit.h"
#endif
#if SUPPORT_SOFTTAP
#include "mcube_softtap.h"
#endif
///----------------------------------------------------------------------------
/// Macros
/// ---------------------------------------------------------------------------
#define FIFO_DEEP           32
#define FIFO_THRESHOLD      30

#if (FIFO_THRESHOLD > FIFO_DEEP)
#error "FIFO threshold is greater than FIFO deepth"
#endif

uint32_t cur_total_step              ;//__attribute__((section("retention_mem_area0"),zero_init));

/*******************************************************************************
 *** EXTERNAL FUNCTION
 *******************************************************************************/
///----------------------------------------------------------------------------
/// Public method decl
/// ---------------------------------------------------------------------------



#if SUPPORT_PEDOMETER
extern void enable_pedometer(void);
extern void disable_pedometer(void);
#endif 

#if SUPPORT_SLEEPMETER
extern void enable_sleepmeter(void);
extern void disable_sleepmeter(void);
#endif 

#if SUPPORT_LIFTARM
extern void enable_liftarm(void);
extern void disable_liftarm(void);
#endif

#if SUPPORT_LONGSIT
extern void enable_longsit(void);
extern void disable_longsit(void);
extern void mcube_longsit_timer_handle(void);

#endif

extern int gsensor_init(void);
extern void mcube_fifo_timer_handle(void);

#ifndef MC36XX_ACC_FIFO_POLLING
extern void gsensor_handle_int(void);
#endif


#ifdef __cplusplus
}
#endif
#endif    // END of MC36XX_SAMPLE_H
