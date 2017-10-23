/*****************************************************************************
 *
 * Copyright (c) 2016 mCube, Inc.  All rights reserved.
 *
 * This source is subject to the mCube Software License.
 * This software is protected by Copyright and the information and source code
 * contained herein is confidential. The software including the source code
 * may not be copied and the information contained herein may not be used or
 * disclosed except with the written permission of mCube Inc.
 *
 * All other rights reserved.
 *****************************************************************************/

#ifndef _M_DRV_MC36XX_H_
    #define _M_DRV_MC36XX_H_

/*******************************************************************************
 *** INCLUDE FILES
 *******************************************************************************/

/*******************************************************************************
 *** CONSTANT / DEFINE
 *******************************************************************************/
#define M_DRV_MC36XX_RETCODE_SUCCESS                 (0)
#define M_DRV_MC36XX_RETCODE_ERROR_BUS               (-1)
#define M_DRV_MC36XX_RETCODE_ERROR_NULL_POINTER      (-2)
#define M_DRV_MC36XX_RETCODE_ERROR_STATUS            (-3)
#define M_DRV_MC36XX_RETCODE_ERROR_SETUP             (-4)
#define M_DRV_MC36XX_RETCODE_ERROR_GET_DATA          (-5)
#define M_DRV_MC36XX_RETCODE_ERROR_IDENTIFICATION    (-6)
#define M_DRV_MC36XX_RETCODE_ERROR_NO_DATA           (-7)
#define M_DRV_MC36XX_RETCODE_ERROR_WRONG_ARGUMENT    (-8)

#define M_DRV_MC36XX_AXIS_X      0
#define M_DRV_MC36XX_AXIS_Y      1
#define M_DRV_MC36XX_AXIS_Z      2
#define M_DRV_MC36XX_AXES_NUM    3

#define M_DRV_MC36XX_FIFO_DEPTH    32
#define M_DRV_MC36XX_REG_MAP_SIZE  64

/*****************************************************************************
 *** DATA TYPE / STRUCTURE DEFINITION / ENUM
 *****************************************************************************/
typedef enum
{
    E_M_DRV_MC36XX_MODE_SLEEP = 0,
    E_M_DRV_MC36XX_MODE_STANDBY,
    E_M_DRV_MC36XX_MODE_SNIFF,
    E_M_DRV_MC36XX_MODE_FSNIFF,
    E_M_DRV_MC36XX_MODE_PWAKE,
    E_M_DRV_MC36XX_MODE_CWAKE,
    E_M_DRV_MC36XX_MODE_SWAKE,
    E_M_DRV_MC36XX_MODE_TRIG,
    E_M_DRV_MC36XX_MODE_END
}   E_M_DRV_MC36XX_MODE;

typedef enum
{
    E_M_DRV_MC36XX_RANGE_2G = 0,
    E_M_DRV_MC36XX_RANGE_4G,
    E_M_DRV_MC36XX_RANGE_8G,
    E_M_DRV_MC36XX_RANGE_16G,
    E_M_DRV_MC36XX_RANGE_12G,
    E_M_DRV_MC36XX_RANGE_24G,
    E_M_DRV_MC36XX_RANGE_END
}   E_M_DRV_MC36XX_RANGE;

typedef enum
{
    E_M_DRV_MC36XX_RESOLUTION_6BIT = 0,
    E_M_DRV_MC36XX_RESOLUTION_7BIT,
    E_M_DRV_MC36XX_RESOLUTION_8BIT,
    E_M_DRV_MC36XX_RESOLUTION_10BIT,
    E_M_DRV_MC36XX_RESOLUTION_12BIT,
    E_M_DRV_MC36XX_RESOLUTION_14BIT,
    E_M_DRV_MC36XX_RESOLUTION_END,
}   E_M_DRV_MC36XX_RESOLUTION;

typedef enum
{
	E_M_DRV_MC36XX_CWAKE_SR_UL_DUMMY_BASE = 0,                                       // DO NOT USE (mark)
    E_M_DRV_MC36XX_CWAKE_SR_UL_0p4Hz,
    E_M_DRV_MC36XX_CWAKE_SR_UL_0p8Hz,
    E_M_DRV_MC36XX_CWAKE_SR_UL_1p6Hz,
    E_M_DRV_MC36XX_CWAKE_SR_UL_6p5Hz,
    E_M_DRV_MC36XX_CWAKE_SR_UL_13Hz,
    E_M_DRV_MC36XX_CWAKE_SR_UL_26Hz,
    E_M_DRV_MC36XX_CWAKE_SR_UL_51Hz,
    E_M_DRV_MC36XX_CWAKE_SR_UL_100Hz,
    E_M_DRV_MC36XX_CWAKE_SR_UL_197Hz,
    E_M_DRV_MC36XX_CWAKE_SR_UL_389Hz,
    E_M_DRV_MC36XX_CWAKE_SR_UL_761Hz,
    E_M_DRV_MC36XX_CWAKE_SR_UL_1122Hz,
    E_M_DRV_MC36XX_CWAKE_SR_UL_DUMMY_END,

    E_M_DRV_MC36XX_CWAKE_SR_LP_DUMMY_BASE = E_M_DRV_MC36XX_CWAKE_SR_UL_DUMMY_END,    // DO NOT USE (mark)
    E_M_DRV_MC36XX_CWAKE_SR_LP_0p4Hz,
    E_M_DRV_MC36XX_CWAKE_SR_LP_0p8Hz,
    E_M_DRV_MC36XX_CWAKE_SR_LP_1p7Hz,
    E_M_DRV_MC36XX_CWAKE_SR_LP_7Hz,
    E_M_DRV_MC36XX_CWAKE_SR_LP_14Hz,
    E_M_DRV_MC36XX_CWAKE_SR_LP_27Hz,
    E_M_DRV_MC36XX_CWAKE_SR_LP_54Hz,
    E_M_DRV_MC36XX_CWAKE_SR_LP_106Hz,
    E_M_DRV_MC36XX_CWAKE_SR_LP_210Hz,
    E_M_DRV_MC36XX_CWAKE_SR_LP_411Hz,
    E_M_DRV_MC36XX_CWAKE_SR_LP_606Hz,
    E_M_DRV_MC36XX_CWAKE_SR_LP_DUMMY_END,

	E_M_DRV_MC36XX_CWAKE_SR_LOW_PR_DUMMY_BASE = E_M_DRV_MC36XX_CWAKE_SR_LP_DUMMY_END,    // DO NOT USE (mark)
    E_M_DRV_MC36XX_CWAKE_SR_LOW_PR_0p2Hz,
    E_M_DRV_MC36XX_CWAKE_SR_LOW_PR_0p9Hz,
    E_M_DRV_MC36XX_CWAKE_SR_LOW_PR_3p6Hz,
    E_M_DRV_MC36XX_CWAKE_SR_LOW_PR_7p3Hz,
    E_M_DRV_MC36XX_CWAKE_SR_LOW_PR_14Hz,
    E_M_DRV_MC36XX_CWAKE_SR_LOW_PR_28Hz,
    E_M_DRV_MC36XX_CWAKE_SR_LOW_PR_55Hz,
    E_M_DRV_MC36XX_CWAKE_SR_LOW_PR_109Hz,
    E_M_DRV_MC36XX_CWAKE_SR_LOW_PR_213Hz,
    E_M_DRV_MC36XX_CWAKE_SR_LOW_PR_315Hz,
    E_M_DRV_MC36XX_CWAKE_SR_LOW_PR_DUMMY_END,                                            // DO NOT USE (mark)

    E_M_DRV_MC36XX_CWAKE_SR_PR_DUMMY_BASE = E_M_DRV_MC36XX_CWAKE_SR_LOW_PR_DUMMY_END,    // DO NOT USE (mark)
    E_M_DRV_MC36XX_CWAKE_SR_PR_0p2Hz,
    E_M_DRV_MC36XX_CWAKE_SR_PR_0p4Hz,
    E_M_DRV_MC36XX_CWAKE_SR_PR_1p8Hz,
    E_M_DRV_MC36XX_CWAKE_SR_PR_3p7Hz,
    E_M_DRV_MC36XX_CWAKE_SR_PR_14Hz,
    E_M_DRV_MC36XX_CWAKE_SR_PR_28Hz,
    E_M_DRV_MC36XX_CWAKE_SR_PR_55Hz,
    E_M_DRV_MC36XX_CWAKE_SR_PR_109Hz,
    E_M_DRV_MC36XX_CWAKE_SR_PR_161Hz,
    E_M_DRV_MC36XX_CWAKE_SR_PR_DUMMY_END,                                                // DO NOT USE (mark)

 	E_M_DRV_MC36XX_CWAKE_SR_HIGH_PR_DUMMY_BASE = E_M_DRV_MC36XX_CWAKE_SR_PR_DUMMY_END,   // DO NOT USE (mark)
    E_M_DRV_MC36XX_CWAKE_SR_HIGH_PR_0p2Hz,
    E_M_DRV_MC36XX_CWAKE_SR_HIGH_PR_0p4Hz,
    E_M_DRV_MC36XX_CWAKE_SR_HIGH_PR_0p9Hz,
    E_M_DRV_MC36XX_CWAKE_SR_HIGH_PR_7p2Hz,
    E_M_DRV_MC36XX_CWAKE_SR_HIGH_PR_14Hz,
    E_M_DRV_MC36XX_CWAKE_SR_HIGH_PR_28Hz,
    E_M_DRV_MC36XX_CWAKE_SR_HIGH_PR_55Hz,
    E_M_DRV_MC36XX_CWAKE_SR_HIGH_PR_81Hz,
    E_M_DRV_MC36XX_CWAKE_SR_HIGH_PR_DUMMY_END,
}   E_M_DRV_MC36XX_CWAKE_SR;

typedef enum
{
    E_M_DRV_MC36XX_SNIFF_SR_DEFAULT_6Hz = 0,
    E_M_DRV_MC36XX_SNIFF_SR_0p4Hz,
    E_M_DRV_MC36XX_SNIFF_SR_0p8Hz,
    E_M_DRV_MC36XX_SNIFF_SR_2Hz,
    E_M_DRV_MC36XX_SNIFF_SR_6Hz,
    E_M_DRV_MC36XX_SNIFF_SR_13Hz,
    E_M_DRV_MC36XX_SNIFF_SR_26Hz,
    E_M_DRV_MC36XX_SNIFF_SR_50Hz,
    E_M_DRV_MC36XX_SNIFF_SR_100Hz,
    E_M_DRV_MC36XX_SNIFF_SR_200Hz,
    E_M_DRV_MC36XX_SNIFF_SR_400Hz,
    E_M_DRV_MC36XX_SNIFF_SR_END,
}   E_M_DRV_MC36XX_SNIFF_SR;

typedef enum
{
    E_M_DRV_MC36XX_FIFO_CONTROL_DISABLE = 0,
    E_M_DRV_MC36XX_FIFO_CONTROL_ENABLE,
    E_M_DRV_MC36XX_FIFO_CONTROL_END,
}   E_M_DRV_MC36XX_FIFO_CONTROL;

typedef enum
{
    E_M_DRV_MC36XX_FIFO_MODE_NORMAL = 0,
    E_M_DRV_MC36XX_FIFO_MODE_WATERMARK,
    E_M_DRV_MC36XX_FIFO_MODE_END,
}   E_M_DRV_MC36XX_FIFO_MODE;

typedef struct
{
    unsigned char    bWAKE;
    unsigned char    bACQ;
    unsigned char    bFIFO_EMPTY;
    unsigned char    bFIFO_FULL;
    unsigned char    bFIFO_THRESHOLD;
    unsigned char    bRESV;
    unsigned char 	 bSWAKE_SNIFF;
    unsigned char    baPadding[2];
}   S_M_DRV_MC36XX_InterruptEvent;

typedef enum
{
	E_M_DRV_MC36XX_WAKE_GAIN_HIGH= 0,
    E_M_DRV_MC36XX_WAKE_GAIN_MED,
    E_M_DRV_MC36XX_WAKE_GAIN_LOW,
    E_M_DRV_MC36XX_WAKE_GAIN_END,
}   E_M_DRV_MC36XX_WAKE_GAIN;

typedef enum
{
    E_M_DRV_MC36XX_SNIFF_GAIN_HIGH = 0,
    E_M_DRV_MC36XX_SNIFF_GAIN_MED,
    E_M_DRV_MC36XX_SNIFF_GAIN_LOW,
    E_M_DRV_MC36XX_SNIFF_GAIN_END,
}   E_M_DRV_MC36XX_SNIFF_GAIN;

/*******************************************************************************
 *** EXTERNAL FUNCTION
 *******************************************************************************/
extern int    M_DRV_MC36XX_Init(void);
extern int    M_DRV_MC36XX_SetMode(E_M_DRV_MC36XX_MODE eNextMode);
extern int    M_DRV_MC36XX_ConfigRegRngResCtrl(E_M_DRV_MC36XX_RANGE eCfgRange, E_M_DRV_MC36XX_RESOLUTION eCfgResolution);
extern int    M_DRV_MC36XX_SetSampleRate(E_M_DRV_MC36XX_CWAKE_SR eCwakeSR, E_M_DRV_MC36XX_SNIFF_SR eSniffSR);
extern int    M_DRV_MC36XX_EnableFIFO(E_M_DRV_MC36XX_FIFO_CONTROL eCtrl, E_M_DRV_MC36XX_FIFO_MODE eMode, unsigned char bThreshold);
extern int    M_DRV_MC36XX_ConfigINT(unsigned char bFifoThreshEnable,
                                     unsigned char bFifoFullEnable  ,
                                     unsigned char bFifoEmptyEnable ,
                                     unsigned char bACQEnable       ,
                                     unsigned char bWakeEnable       );

extern int    M_DRV_MC36XX_ReadData(float faOutput[M_DRV_MC36XX_FIFO_DEPTH][M_DRV_MC36XX_AXES_NUM], int nNumOfSample);
extern int    M_DRV_MC36XX_HandleINT(S_M_DRV_MC36XX_InterruptEvent *ptINT_Event);
extern int    M_DRV_MC36XX_SetGain(E_M_DRV_MC36XX_WAKE_GAIN eWakeGain, E_M_DRV_MC36XX_SNIFF_GAIN eSniffGain);

// for debug
extern int    M_DRV_MC36XX_ReadRegMap(unsigned char baRegMap[M_DRV_MC36XX_REG_MAP_SIZE]);
extern unsigned char    M_DRV_MC36XX_ReadReg(unsigned char bRegAddr);
																		 
extern void _M_DRV_MC36XX_LowPassFilter(signed short _saData[M_DRV_MC36XX_AXES_NUM]);

extern void M_DRV_MC36XX_ReadDoff_Dgain(void);
extern void M_DRV_MC36XX_ReadAofsp(void);
extern void M_DRV_MC36XX_SetD0ff4x_DGain4x(void);




#endif    // END of _M_DRV_MC36XX_H_

