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

/*******************************************************************************
 *** INFORMATION
 *******************************************************************************/
#define M_DRV_MC36XX_VERSION    "2.0.1"
#include "stdio.h"
/*******************************************************************************
 *** CONFIGURATION
 *******************************************************************************/
//#define M_DRV_MC36XX_CFG_BUS_I2C    // !!! DO NOT use both I2C and SPI at the same time
//#define M_DRV_MC36XX_CFG_BUS_SPI

//#if (!defined (M_DRV_MC36XX_CFG_BUS_SPI) && !defined (M_DRV_MC36XX_CFG_BUS_I2C))
//#error "MUST use one bus to access register!"
//#endif

//#if (defined (M_DRV_MC36XX_CFG_BUS_SPI) && defined (M_DRV_MC36XX_CFG_BUS_I2C))
//#error "DO NOT use both SPI and I2C simultaneously!"
//#endif
#include "mcube_custom_config.h"
#define M_PRINTF	mcube_printf

//#define M_DRV_MC36XX_CFG_I2C_ADDR    (0x4C)

#define M_DRV_MC36XX_CFG_SAMPLE_RATE_CWAKE_DEFAULT    E_M_DRV_MC36XX_CWAKE_SR_LP_54Hz
#define M_DRV_MC36XX_CFG_SAMPLE_RATE_SNIFF_DEFAULT    E_M_DRV_MC36XX_SNIFF_SR_6Hz
#define M_DRV_MC36XX_CFG_RANGE         				  E_M_DRV_MC36XX_RANGE_4G
#define M_DRV_MC36XX_CFG_RESOLUTION    				  E_M_DRV_MC36XX_RESOLUTION_12BIT
#define M_DRV_MC36XX_CFG_ORIENTATION_MAP    		  E_M_DRV_UTIL_ORIENTATION_TOP_RIGHT_UP
#define M_DRV_MC36XX_CFG_WAKE_GAIN_DEFAULT			  E_M_DRV_MC36XX_WAKE_GAIN_LOW
#define M_DRV_MC36XX_CFG_SNIFF_GAIN_DEFAULT			  E_M_DRV_MC36XX_SNIFF_GAIN_HIGH
#define M_DRV_MC36XX_CFG_WA_10

//#define M_DRV_MC36XX_OPERATE_MODE_WAKE_WHEN_READ
#define M_DRV_MC36XX_OPERATE_MODE_ALWAYS_WAKE
//#define M_DRV_MC36XX_SUPPORT_LPF

/*******************************************************************************
 *** INCLUDE FILES
 *******************************************************************************/
typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef signed short int16_t;
//typedef signed int int32_t;
//typedef unsigned int uint32_t;


//#include <stdint.h>

//#ifdef M_DRV_MC36XX_CFG_BUS_I2C
//#include "m_drv_i2c.h"    // hook by custom
//#else
//#include "m_drv_spi.h"    // hook by custom
//#endif

#include "m_drv_mc36xx.h"
//#include "m_drv_mc_utility.h"
#include "mcube_custom_config.h"

// Remove from formal release!!!
//#include "m_drv_uart.h"
//#include "m_drv_timer.h"

/*******************************************************************************
 *** CONSTANT / DEFINE
 *******************************************************************************/
//#define M_DRV_MC36XX_I2C_WRITE_ADDR    ((M_DRV_MC36XX_CFG_I2C_ADDR << 1) | 0x00)
//#define M_DRV_MC36XX_I2C_READ_ADDR     ((M_DRV_MC36XX_CFG_I2C_ADDR << 1) | 0x01)

//=============================================
#define M_DRV_MC36XX_INTR_C_IPP_MODE_OPEN_DRAIN    (0x00)
#define M_DRV_MC36XX_INTR_C_IPP_MODE_PUSH_PULL     (0x01)

#define M_DRV_MC36XX_INTR_C_IAH_ACTIVE_LOW     (0x00)
#define M_DRV_MC36XX_INTR_C_IAH_ACTIVE_HIGH    (0x02)

 //=============================================
 #define M_DRV_MC36XX_REG_EXT_STAT_1       (0x00)
 #define M_DRV_MC36XX_REG_EXT_STAT_2       (0x01)
 #define M_DRV_MC36XX_REG_XOUT_LSB         (0x02)
 #define M_DRV_MC36XX_REG_XOUT_MSB         (0x03)
 #define M_DRV_MC36XX_REG_YOUT_LSB         (0x04)
 #define M_DRV_MC36XX_REG_YOUT_MSB         (0x05)
 #define M_DRV_MC36XX_REG_ZOUT_LSB         (0x06)
 #define M_DRV_MC36XX_REG_ZOUT_MSB         (0x07)
 #define M_DRV_MC36XX_REG_STATUS_1         (0x08)
 #define M_DRV_MC36XX_REG_STATUS_2         (0x09)
 #define M_DRV_MC36XX_REG_FEATURE_C_1	   (0X0D)
 #define M_DRV_MC36XX_REG_FEATURE_C_2	   (0X0E)
 #define M_DRV_MC36XX_REG_PWR_CONTROL	   (0X0F)
 #define M_DRV_MC36XX_REG_MODE_C           (0x10)
 #define M_DRV_MC36XX_REG_WAKE_C           (0x11)
 #define M_DRV_MC36XX_REG_SNIFF_C          (0x12)
 #define M_DRV_MC36XX_REG_SNIFFTH_C        (0x13)
 #define M_DRV_MC36XX_REG_SNIFF_CFG        (0x14)
 #define M_DRV_MC36XX_REG_RANGE_C          (0x15)
 #define M_DRV_MC36XX_REG_FIFO_C           (0x16)
 #define M_DRV_MC36XX_REG_INTR_C           (0x17)
 #define M_DRV_MC36XX_REG_DMX              (0x20)
 #define M_DRV_MC36XX_REG_DMY              (0x21)
 #define M_DRV_MC36XX_REG_DMZ              (0x22)
 
 #define M_DRV_MC36XX_REG_SEC              (0x25)
 
 #define M_DRV_MC36XX_REG_XOFFL            (0x2A)
 #define M_DRV_MC36XX_REG_XOFFH            (0x2B)
 #define M_DRV_MC36XX_REG_YOFFL            (0x2C)
 #define M_DRV_MC36XX_REG_YOFFH            (0x2D)
 #define M_DRV_MC36XX_REG_ZOFFL            (0x2E)
 #define M_DRV_MC36XX_REG_ZOFFH            (0x2F)
 #define M_DRV_MC36XX_REG_XGAIN            (0x30)
 #define M_DRV_MC36XX_REG_YGAIN            (0x31)
 #define M_DRV_MC36XX_REG_ZGAIN            (0x32)

 #define M_DRV_MC36XX_REG_XAOFSP		   (0x35)
 #define M_DRV_MC36XX_REG_XAOFSN		   (0x36)
 #define M_DRV_MC36XX_REG_YAOFSP		   (0x37)
 #define M_DRV_MC36XX_REG_YAOFSN		   (0x38)
 #define M_DRV_MC36XX_REG_ZAOFSP		   (0x39)
 #define M_DRV_MC36XX_REG_ZAOFSN		   (0x3A)

 #define M_DRV_MC36XX_REG_OPT              (0x3B)
 #define M_DRV_MC36XX_REG_LOC_X            (0x3C)
 #define M_DRV_MC36XX_REG_LOC_Y            (0x3D)
 #define M_DRV_MC36XX_REG_LOT_dAOFSZ       (0x3E)
 #define M_DRV_MC36XX_REG_WAF_LOT          (0x3F)

#define M_DRV_MC36XX_NULL_ADDR    (0)

//Sniff OSR
#define M_DRV_MC36XX_SR_MODE_UNKNOWN            	  (0xFF)
#define M_DRV_MC36XX_SNIFF_SR_MODE_ULTRA_LOW_POWER    (0x03)
#define M_DRV_MC36XX_SNIFF_SR_MODE_LOW_POWER          (0x00)
#define M_DRV_MC36XX_SNIFF_SR_MODE_PRECISION          (0x02)
#define M_DRV_MC36XX_SNIFF_SR_MODE_LOW_PRECISION      (0x01)
#define M_DRV_MC36XX_SNIFF_SR_MODE_HIGH_PRECISION     (0x04)

//Wake OSR
#define M_DRV_MC36XX_WAKE_SR_MODE_ULTRA_LOW_POWER    (0x03)
#define M_DRV_MC36XX_WAKE_SR_MODE_LOW_POWER          (0x00)
#define M_DRV_MC36XX_WAKE_SR_MODE_PRECISION          (0x02)
#define M_DRV_MC36XX_WAKE_SR_MODE_LOW_PRECISION      (0x01)
#define M_DRV_MC36XX_WAKE_SR_MODE_HIGH_PRECISION     (0x04)

#ifdef M_DRV_MC36XX_SUPPORT_LPF
#define SAMPLE_RATE   28.0f//50 //(Hz) Sampling frequency
#define SAMPLE_INTERVAL   1/SAMPLE_RATE //Sampling interval
#define CUT_OFF_FREQUENCY 4 //(Hz) Cut-off frequency
#define PI	3.14
#define ALPHA SAMPLE_INTERVAL/( 1 / ((2 * PI * CUT_OFF_FREQUENCY) + SAMPLE_INTERVAL)) //LPF coefficient
#endif

/*******************************************************************************
 *** MACRO
 *******************************************************************************/
//#ifdef M_DRV_MC36XX_CFG_BUS_I2C
//#define _M_DRV_MC36XX_REG_WRITE(bRegAddr, pbDataBuf, bLength)    M_DRV_I2C_Write(M_DRV_MC36XX_I2C_WRITE_ADDR, bRegAddr, pbDataBuf, bLength)
//#define _M_DRV_MC36XX_REG_READ(bRegAddr, pbDataBuf, bLength)     M_DRV_I2C_Read(M_DRV_MC36XX_I2C_READ_ADDR, bRegAddr, pbDataBuf, bLength)
//#else
//#define _M_DRV_MC36XX_REG_WRITE(bRegAddr, pbDataBuf, bLength)    M_DRV_SPI_Write(bRegAddr, pbDataBuf, bLength)
//#define _M_DRV_MC36XX_REG_READ(bRegAddr, pbDataBuf, bLength)     M_DRV_SPI_Read((bRegAddr | 0x80), pbDataBuf, bLength)
//#endif

#define _M_DRV_MC36XX_REG_STATUS_1_MODE(bRegStatus1)           		(bRegStatus1 & 0x07)
#define _M_DRV_MC36XX_REG_STATUS_1_NEW_DATA(bRegStatus1)      		(bRegStatus1 & 0x08)
#define _M_DRV_MC36XX_REG_STATUS_1_FIFO_EMPTY(bRegStatus1)     		(bRegStatus1 & 0x10)
#define _M_DRV_MC36XX_REG_STATUS_1_FIFO_FULL(bRegStatus1)      		(bRegStatus1 & 0x20)
#define _M_DRV_MC36XX_REG_STATUS_1_FIFO_THRESH(bRegStatus1)    		(bRegStatus1 & 0x40)
#define _M_DRV_MC36XX_REG_STATUS_1_INT_PEND(bRegStatus1)       		(bRegStatus1 & 0x80)

#define _M_DRV_MC36XX_REG_STATUS_2_INT_WAKE(bRegStatus2)           ((bRegStatus2 >> 2) & 0x01)
#define _M_DRV_MC36XX_REG_STATUS_2_INT_ACQ(bRegStatus2)            ((bRegStatus2 >> 3) & 0x01)
#define _M_DRV_MC36XX_REG_STATUS_2_INT_FIFO_EMPTY(bRegStatus2)     ((bRegStatus2 >> 4) & 0x01)
#define _M_DRV_MC36XX_REG_STATUS_2_INT_FIFO_FULL(bRegStatus2)      ((bRegStatus2 >> 5) & 0x01)
#define _M_DRV_MC36XX_REG_STATUS_2_INT_FIFO_THRESH(bRegStatus2)    ((bRegStatus2 >> 6) & 0x01)
#define _M_DRV_MC36XX_REG_STATUS_2_INT_SWAKE_SNIFF(bRegStatus2)    ((bRegStatus2 >> 7) & 0x01)

#define _M_DRV_MC36XX_REG_MODE_C_MODE(bRegMODE_C)       			(bRegMODE_C & 0x07)
#define _M_DRV_MC36XX_REG_RANGE_C_RES(bRegRANGE_C)     				(bRegRANGE_C & 0x07)
#define _M_DRV_MC36XX_REG_RANGE_C_RANGE(bRegRANGE_C)    			((bRegRANGE_C >> 4) & 0x07)

#define _M_DRV_MC36XX_REG_FIFO_C_FIFO_EN(bRegFIFO_C)    			(bRegFIFO_C & 0x40)
#define _M_DRV_MC36XX_FIFO_VDD_EN(bRegPwrCtrl)    					(bRegPwrCtrl | 0x42);

#define _M_DRV_MC36XX_M_ABS(x)    (((x) < 0) ? (-(x)) : (x))

#ifdef M_DRV_MC36XX_SUPPORT_LPF
#define _M_DRV_MC36XX_SENSOR_FILTER(last_data, curr_data) (( ALPHA * (last_data)) + (( 1 - ALPHA) * (curr_data)))
#endif



/*******************************************************************************
 *** STATIC VARIABLE
 *******************************************************************************/
static float s_fMC3XXX_Sensitivity = 0.0f;    // unit: SI/LSB, SI: m/s^2

static E_M_DRV_MC36XX_RANGE s_eRange = M_DRV_MC36XX_CFG_RANGE;
static E_M_DRV_MC36XX_RESOLUTION s_eResolution = M_DRV_MC36XX_CFG_RESOLUTION;
static E_M_DRV_MC36XX_CWAKE_SR s_eSR_CWAKE = M_DRV_MC36XX_CFG_SAMPLE_RATE_CWAKE_DEFAULT;
static E_M_DRV_MC36XX_SNIFF_SR s_eSR_SNIFF = M_DRV_MC36XX_CFG_SAMPLE_RATE_SNIFF_DEFAULT;
static E_M_DRV_MC36XX_WAKE_GAIN s_eGAIN_WAKE = M_DRV_MC36XX_CFG_WAKE_GAIN_DEFAULT;
static E_M_DRV_MC36XX_SNIFF_GAIN s_eGAIN_SNIFF =  M_DRV_MC36XX_CFG_SNIFF_GAIN_DEFAULT;

static unsigned char s_bCfgRngResol = 0x00;
static unsigned char s_bCfgSniffThr = 0x00;
//static unsigned char s_bCfgSniffCfg = 0x00;
static unsigned char s_bCfgFifo = 0x00;
static unsigned char s_bCfgINT = 0x00;
static unsigned char s_bCfgFifoVdd = 0x42;
static unsigned char s_bCfgWakeSRMode = M_DRV_MC36XX_SR_MODE_UNKNOWN;
static unsigned char s_bCfgSniffSRMode = M_DRV_MC36XX_SR_MODE_UNKNOWN;

//static unsigned char s_debug=0x00;

#ifdef M_DRV_MC36XX_SUPPORT_LPF
static short _saLPFPrevData[M_DRV_MC36XX_AXES_NUM]={0};
//static short _saLPF1STPrevData[M_DRV_MC36XX_AXES_NUM]={0};
//static short _saLPF2NDPrevData[M_DRV_MC36XX_AXES_NUM]={0};
#endif

/*******************************************************************************
 *** STATIC FUNCTION
 *******************************************************************************/
/*********************************************************************
 *** _M_DRV_MC36XX_Delay
 *********************************************************************/
int _M_DRV_MC36XX_Delay(unsigned long dwMs) {
	// hook by custom
	mcube_delay_ms(dwMs);

	return M_DRV_MC36XX_RETCODE_SUCCESS; //(M_DRV_TIMER_DelayMs(dwMs));
}

void _M_DRV_MC36XX_REG_READ(uint8_t bRegAddr, uint8_t *pbDataBuf, uint8_t bLength)
{
	mcube_read_regs(bRegAddr, pbDataBuf, bLength);
}

void _M_DRV_MC36XX_REG_WRITE(uint8_t bRegAddr, uint8_t *pbDataBuf, uint8_t bLength)
{
	mcube_write_regs(bRegAddr, pbDataBuf, bLength);
}

/*********************************************************************
 *** _M_DRV_MC36XX_SetBusIF
 *********************************************************************/
static int _M_DRV_MC36XX_SetBusIF(void)
{
//	if(s_debug) M_PRINTF("[%s]", __func__);

	/*Orion I2C/SPI interface Setup */
	unsigned char _bRegIO_C = 0;
	_M_DRV_MC36XX_REG_READ(M_DRV_MC36XX_REG_FEATURE_C_1, &_bRegIO_C, 1); //0x0D

#ifdef M_DRV_MC36XX_CFG_BUS_I2C
	_bRegIO_C &= 0x3F;
	_bRegIO_C |= 0x40;
#else    //#ifdef M_DRV_MC36XX_CFG_BUS_SPI
	_bRegIO_C &= 0x3F;
	_bRegIO_C |= 0x80;
#endif

	_M_DRV_MC36XX_REG_WRITE(M_DRV_MC36XX_REG_FEATURE_C_1, &_bRegIO_C, 1);

	return (M_DRV_MC36XX_RETCODE_SUCCESS);
}

/*****************************************
 *** _M_DRV_MC36XX_SetSniffAGAIN
 *****************************************/
static int _M_DRV_MC36XX_SetSniffAGAIN(uint8_t SniffGain)
{
	//if(s_debug) M_PRINTF("[%s] SniffGain= 0x%02X", __func__, SniffGain);
	unsigned char _bRegAGain = 0x00;

	if(SniffGain > E_M_DRV_MC36XX_SNIFF_GAIN_END)
	{
		return (M_DRV_MC36XX_RETCODE_ERROR_WRONG_ARGUMENT);
		//M_PRINTF("[%s] SniffGain > %d", __func__, SniffGain);
	}
	_M_DRV_MC36XX_REG_WRITE(M_DRV_MC36XX_REG_DMX, &_bRegAGain, 1);
	//if(s_debug)  M_PRINTF("[%s] REG[0x%02X] 0x%02X", __func__, M_DRV_MC36XX_REG_DMX, _bRegAGain);

	_bRegAGain = 0x00;
	_M_DRV_MC36XX_REG_READ(M_DRV_MC36XX_REG_DMY, &_bRegAGain, 1);
	_bRegAGain &= 0x3F;
	_bRegAGain |= (SniffGain << 6 );

	_M_DRV_MC36XX_REG_WRITE(M_DRV_MC36XX_REG_DMY, &_bRegAGain, 1);
	//if(s_debug)  M_PRINTF("[%s] REG[0x%02X] 0x%02X", __func__, M_DRV_MC36XX_REG_DMY, _bRegAGain);

	return (M_DRV_MC36XX_RETCODE_SUCCESS);
}

/*****************************************
 *** _M_DRV_MC36XX_SetWakeAGAIN
 *****************************************/
static int _M_DRV_MC36XX_SetWakeAGAIN(uint8_t WakeGain)
{
	//if(s_debug) M_PRINTF("[%s] WakeGain= 0x%02X", __func__, WakeGain);
	unsigned char _bRegAGain = 0x01;

	if(WakeGain > E_M_DRV_MC36XX_WAKE_GAIN_END)
	{
		return (M_DRV_MC36XX_RETCODE_ERROR_WRONG_ARGUMENT);
		//M_PRINTF("[%s] WakeGain > %d", __func__, WakeGain);
	}

	_M_DRV_MC36XX_REG_WRITE(M_DRV_MC36XX_REG_DMX, &_bRegAGain, 1);
	//if(s_debug)  M_PRINTF("[%s] REG[0x%02X] 0x%02X", __func__, M_DRV_MC36XX_REG_DMX, _bRegAGain);

	_bRegAGain = 0x00;
	_M_DRV_MC36XX_REG_READ(M_DRV_MC36XX_REG_DMY, &_bRegAGain, 1);

	_bRegAGain &= 0x3F;
	_bRegAGain |= (WakeGain << 6);
	_M_DRV_MC36XX_REG_WRITE(M_DRV_MC36XX_REG_DMY, &_bRegAGain, 1);
	//if(s_debug)  M_PRINTF("[%s] REG[0x%02X] 0x%02X", __func__, M_DRV_MC36XX_REG_DMY, _bRegAGain);

	_bRegAGain = 0x00;
	_M_DRV_MC36XX_REG_WRITE(M_DRV_MC36XX_REG_DMX, &_bRegAGain, 1);
	//if(s_debug)  M_PRINTF("[%s] REG[0x%02X] 0x%02X", __func__, M_DRV_MC36XX_REG_DMX, _bRegAGain);

	return (M_DRV_MC36XX_RETCODE_SUCCESS);
}

/*****************************************
 *** _M_DRV_MC36XX_ResetChip
 *****************************************/
static void _M_DRV_MC36XX_ResetChip(void)
{
	//if(s_debug) M_PRINTF("[%s]", __func__);
	unsigned char _bRegData = 0x01;

	_M_DRV_MC36XX_REG_WRITE(0x10, &_bRegData, 1);

	_M_DRV_MC36XX_Delay(10);

	_bRegData = 0x40;
	_M_DRV_MC36XX_REG_WRITE(0x24, &_bRegData, 1);

	_M_DRV_MC36XX_Delay(50);
	
#if 1
    //_bRegData = 0x00;
	_bRegData = 0x40;
	//_M_DRV_MC36XX_REG_WRITE(0x09, &_bRegData, 1); //dummy write
           _M_DRV_MC36XX_REG_WRITE(0x0D, &_bRegData, 1);
	// Jerry add start for initial sequence after reset
	_bRegData = 0x42;
	_M_DRV_MC36XX_REG_WRITE(0x0F, &_bRegData, 1);
	_bRegData = 0x01;
	_M_DRV_MC36XX_REG_WRITE(0x20, &_bRegData, 1);
	_bRegData = 0x80;
	_M_DRV_MC36XX_REG_WRITE(0x21, &_bRegData, 1);
	_bRegData = 0x00;
	_M_DRV_MC36XX_REG_WRITE(0x28, &_bRegData, 1);
	_bRegData = 0x00;
	_M_DRV_MC36XX_REG_WRITE(0x1a, &_bRegData, 1);
	// Jerry add end
#endif 	

	_M_DRV_MC36XX_SetBusIF();

	_M_DRV_MC36XX_SetWakeAGAIN(E_M_DRV_MC36XX_WAKE_GAIN_LOW);
	_M_DRV_MC36XX_SetSniffAGAIN(E_M_DRV_MC36XX_SNIFF_GAIN_HIGH);

	_bRegData = 0x01;
	_M_DRV_MC36XX_REG_WRITE(0x10, &_bRegData, 1);

	_M_DRV_MC36XX_Delay(10);
}

/*****************************************
 *** _M_DRV_MC36XX_ValidateSensorIC
 *****************************************/
static int _M_DRV_MC36XX_ValidateSensorIC(void) 
{
	//if (s_debug)	M_PRINTF("[%s] ===================================", __func__);

	unsigned char _bRegData = 0;
     //   printf("_bRegData0 = %u \r",_bRegData);
        fflush(stdout);
	_M_DRV_MC36XX_REG_READ(0x18, &_bRegData, 1);
    //    printf("_bRegData1 = %u \r",_bRegData);
        fflush(stdout);
        //_bRegData = 0x71 ;
	if (0x71 == _bRegData) 
	{
	        //printf("0x71 \r");
	        fflush(stdout);
	        return (M_DRV_MC36XX_RETCODE_SUCCESS);
#if 0
	        _M_DRV_MC36XX_REG_READ(0x19, &_bRegData, 1);

		// STD or ULL mode
		if (0x00 == _bRegData) //Orion 2A
		{
			//if (s_debug) M_PRINTF("[%s] Orion 2A STD", __func__);
			return (M_DRV_MC36XX_RETCODE_SUCCESS);
		} else if (0x01 == _bRegData) 
		{
			//if (s_debug) M_PRINTF("[%s] Orion 2A ULL", __func__);
			return (M_DRV_MC36XX_RETCODE_SUCCESS);
		} else if (0x02 == _bRegData) //Orion 2B ULL
		{
			//if (s_debug) M_PRINTF("[%s] Orion 2B ULL", __func__);
			return (M_DRV_MC36XX_RETCODE_SUCCESS);
		}
#endif

	}

	return (M_DRV_MC36XX_RETCODE_ERROR_IDENTIFICATION);
}


/*****************************************
 *** _M_DRV_MC36XX_SetOTPSampleRate
 *****************************************/
static void    _M_DRV_MC36XX_SetOTPSampleRate(uint8_t mode, uint8_t bDesiredSRMode)
{
	//if(s_debug) M_PRINTF("[%s]", __func__);
	unsigned char    _bRegData = 0x00;

    _M_DRV_MC36XX_REG_READ(0x01, &_bRegData, 1);

    if (0x00 == (_bRegData & 0x10))
    {
        _bRegData = 0x97;
        _M_DRV_MC36XX_REG_WRITE(0x25, &_bRegData, 1);

        _bRegData = 0x5E;
        _M_DRV_MC36XX_REG_WRITE(0x25, &_bRegData, 1);

        _bRegData = 0xF1;
        _M_DRV_MC36XX_REG_WRITE(0x25, &_bRegData, 1);
    }

    _bRegData = 0x00;
    _M_DRV_MC36XX_REG_READ(0x3B, &_bRegData, 1);

    if(mode == E_M_DRV_MC36XX_MODE_SNIFF)
    {
    	_bRegData &= 0xF3;
    	_bRegData |= bDesiredSRMode;
    }
    else if(mode >= E_M_DRV_MC36XX_MODE_PWAKE && mode < E_M_DRV_MC36XX_MODE_END)
    {
    	_bRegData &= 0xFC;
    	_bRegData |= bDesiredSRMode;
    }
    //else
    //	M_PRINTF("[%s] Wrong mode", __func__);

    _M_DRV_MC36XX_REG_WRITE(0x3B, &_bRegData, 1);

}

/*****************************************
 *** _M_DRV_MC36XX_SetSniffOverSampleRate
 *****************************************/
static void _M_DRV_MC36XX_SetSniffOverSampleRate(uint8_t bDesiredSRMode)
{
	//if(s_debug) M_PRINTF("[%s]", __func__);
	unsigned char _bRegData = 0x00;

	if (s_bCfgSniffSRMode == bDesiredSRMode)
		return;

	_M_DRV_MC36XX_REG_READ(0x1C, &_bRegData, 1);
	_bRegData &= 0x8F;
	_bRegData |= (bDesiredSRMode<< 4);

	_M_DRV_MC36XX_REG_WRITE(0x1C, &_bRegData, 1);
	//if(s_debug) M_PRINTF("[%s] REG[0x1C] 0x%02X", __func__, _bRegData );
	s_bCfgSniffSRMode = bDesiredSRMode;

	_M_DRV_MC36XX_SetOTPSampleRate(E_M_DRV_MC36XX_MODE_SNIFF,s_bCfgSniffSRMode);
}

/*****************************************
 *** _M_DRV_MC36XX_SetWakeOverSampleRate
 *****************************************/
static void _M_DRV_MC36XX_SetWakeOverSampleRate(uint8_t bDesiredSRMode)
{
	//if(s_debug) M_PRINTF("[%s]", __func__);

	unsigned char _bRegData = 0x00;
	if (s_bCfgWakeSRMode == bDesiredSRMode)
		return;

	_M_DRV_MC36XX_REG_READ(0x1C, &_bRegData, 1);
	_bRegData &= 0xF8;
	_bRegData |= bDesiredSRMode;

	_M_DRV_MC36XX_REG_WRITE(0x1C, &_bRegData, 1);
	//if(s_debug) M_PRINTF("[%s] REG[0x1C] 0x%02X", __func__, _bRegData );
	s_bCfgWakeSRMode = bDesiredSRMode;

	_M_DRV_MC36XX_SetOTPSampleRate(E_M_DRV_MC36XX_MODE_CWAKE, s_bCfgWakeSRMode);
}

/*****************************************
 *** _M_DRV_MC36XX_SetCwakeSampleRateUltraLowPowerMode
 *****************************************/
static void _M_DRV_MC36XX_SetCwakeSampleRateUltraLowPowerMode(E_M_DRV_MC36XX_CWAKE_SR eSR)
{
	//if(s_debug) M_PRINTF("[%s]", __func__);
	unsigned char _bRegWAKE = eSR;

	_M_DRV_MC36XX_SetWakeOverSampleRate(M_DRV_MC36XX_WAKE_SR_MODE_ULTRA_LOW_POWER);
	_M_DRV_MC36XX_REG_WRITE(M_DRV_MC36XX_REG_WAKE_C, &_bRegWAKE, 1);
	//if(s_debug)  M_PRINTF("[%s] REG[0x%02X] 0x%02X", __func__, M_DRV_MC36XX_REG_WAKE_C, _bRegWAKE );
}

/*****************************************
 *** _M_DRV_MC36XX_SetCwakeSampleRateLowPowerMode
 *****************************************/
static void _M_DRV_MC36XX_SetCwakeSampleRateLowPowerMode(E_M_DRV_MC36XX_CWAKE_SR eSR)
{
	//if(s_debug) M_PRINTF("[%s]", __func__);
	unsigned char _bRegWAKE = (eSR - E_M_DRV_MC36XX_CWAKE_SR_LP_DUMMY_BASE);

	_M_DRV_MC36XX_SetWakeOverSampleRate(M_DRV_MC36XX_WAKE_SR_MODE_LOW_POWER);
	_M_DRV_MC36XX_REG_WRITE(M_DRV_MC36XX_REG_WAKE_C, &_bRegWAKE, 1);
	//if(s_debug)  M_PRINTF("[%s] REG[0x%02X] 0x%02X", __func__, M_DRV_MC36XX_REG_WAKE_C, _bRegWAKE );
}

/*****************************************
 *** _M_DRV_MC36XX_SetCwakeSampleRateLowPrecisionMode
 *****************************************/
static void _M_DRV_MC36XX_SetCwakeSampleRateLowPrecisionMode(E_M_DRV_MC36XX_CWAKE_SR eSR)
{
	//if(s_debug) M_PRINTF("[%s]", __func__);
	unsigned char _bRegWAKE = (eSR - E_M_DRV_MC36XX_CWAKE_SR_LOW_PR_DUMMY_BASE);

	_M_DRV_MC36XX_SetWakeOverSampleRate(M_DRV_MC36XX_WAKE_SR_MODE_LOW_PRECISION);
	_M_DRV_MC36XX_REG_WRITE(M_DRV_MC36XX_REG_WAKE_C, &_bRegWAKE, 1);
}

/*****************************************
 *** _M_DRV_MC36XX_SetCwakeSampleRatePrecisionMode
 *****************************************/
static void _M_DRV_MC36XX_SetCwakeSampleRatePrecisionMode(E_M_DRV_MC36XX_CWAKE_SR eSR)
{
	//if(s_debug) M_PRINTF("[%s]", __func__);
	unsigned char _bRegWAKE = (eSR - E_M_DRV_MC36XX_CWAKE_SR_PR_DUMMY_BASE);

	_M_DRV_MC36XX_SetWakeOverSampleRate(M_DRV_MC36XX_WAKE_SR_MODE_PRECISION);
	_M_DRV_MC36XX_REG_WRITE(M_DRV_MC36XX_REG_WAKE_C, &_bRegWAKE, 1);
	//if(s_debug)  M_PRINTF("[%s] REG[0x%02X] 0x%02X", __func__, M_DRV_MC36XX_REG_WAKE_C, _bRegWAKE );
}

/*****************************************
 *** _M_DRV_MC36XX_SetCwakeSampleRateHighPrecisionMode
 *****************************************/
static void _M_DRV_MC36XX_SetCwakeSampleRateHighPrecisionMode(E_M_DRV_MC36XX_CWAKE_SR eSR)
{
	//if(s_debug) M_PRINTF("[%s]", __func__);
	unsigned char _bRegWAKE = (eSR - E_M_DRV_MC36XX_CWAKE_SR_HIGH_PR_DUMMY_BASE);

	_M_DRV_MC36XX_SetWakeOverSampleRate(M_DRV_MC36XX_WAKE_SR_MODE_HIGH_PRECISION);
	_M_DRV_MC36XX_REG_WRITE(M_DRV_MC36XX_REG_WAKE_C, &_bRegWAKE, 1);
}


/*****************************************
 *** _M_DRV_MC36XX_SetCwakeSampleRate
 *****************************************/
static int _M_DRV_MC36XX_SetCwakeSampleRate(E_M_DRV_MC36XX_CWAKE_SR eSR)
{
	//if(s_debug) M_PRINTF("[%s]", __func__);
	if ((E_M_DRV_MC36XX_CWAKE_SR_UL_DUMMY_BASE < eSR) && (eSR < E_M_DRV_MC36XX_CWAKE_SR_UL_DUMMY_END))
	{
		_M_DRV_MC36XX_SetCwakeSampleRateUltraLowPowerMode(eSR);
	}
	else if ((E_M_DRV_MC36XX_CWAKE_SR_LP_DUMMY_BASE < eSR) && (eSR < E_M_DRV_MC36XX_CWAKE_SR_LP_DUMMY_END))
	{
		_M_DRV_MC36XX_SetCwakeSampleRateLowPowerMode(eSR);
	}
	else if ((E_M_DRV_MC36XX_CWAKE_SR_PR_DUMMY_BASE < eSR) && (eSR < E_M_DRV_MC36XX_CWAKE_SR_PR_DUMMY_END))
	{
		_M_DRV_MC36XX_SetCwakeSampleRatePrecisionMode(eSR);
	}
	else if ((E_M_DRV_MC36XX_CWAKE_SR_HIGH_PR_DUMMY_BASE < eSR) && (eSR < E_M_DRV_MC36XX_CWAKE_SR_HIGH_PR_DUMMY_END))
	{
		_M_DRV_MC36XX_SetCwakeSampleRateHighPrecisionMode(eSR);
	}
	else if ((E_M_DRV_MC36XX_CWAKE_SR_LOW_PR_DUMMY_BASE < eSR) && (eSR < E_M_DRV_MC36XX_CWAKE_SR_LOW_PR_DUMMY_END))
	{
		_M_DRV_MC36XX_SetCwakeSampleRateLowPrecisionMode(eSR);
	}
	else
	{
		return (M_DRV_MC36XX_RETCODE_ERROR_WRONG_ARGUMENT);
	}

	return (M_DRV_MC36XX_RETCODE_SUCCESS);
}

/*****************************************
 *** _M_DRV_MC36XX_CheckCwakeSampleRate
 *****************************************/
static int _M_DRV_MC36XX_CheckCwakeSampleRate(E_M_DRV_MC36XX_CWAKE_SR eSR)
{
	//if(s_debug) M_PRINTF("[%s]", __func__);

	if ((!((E_M_DRV_MC36XX_CWAKE_SR_UL_DUMMY_BASE < eSR) && (eSR < E_M_DRV_MC36XX_CWAKE_SR_UL_DUMMY_END)))
			&& (!((E_M_DRV_MC36XX_CWAKE_SR_LP_DUMMY_BASE < eSR) && (eSR < E_M_DRV_MC36XX_CWAKE_SR_LP_DUMMY_END)))
			&& (!((E_M_DRV_MC36XX_CWAKE_SR_PR_DUMMY_BASE < eSR)	&& (eSR < E_M_DRV_MC36XX_CWAKE_SR_PR_DUMMY_END)))
			&& (!((E_M_DRV_MC36XX_CWAKE_SR_HIGH_PR_DUMMY_BASE < eSR) && (eSR < E_M_DRV_MC36XX_CWAKE_SR_HIGH_PR_DUMMY_END)))
			&& (!((E_M_DRV_MC36XX_CWAKE_SR_LOW_PR_DUMMY_BASE < eSR)	&& (eSR < E_M_DRV_MC36XX_CWAKE_SR_LOW_PR_DUMMY_END))))
	{
		return (M_DRV_MC36XX_RETCODE_ERROR_WRONG_ARGUMENT);
	}

	return (M_DRV_MC36XX_RETCODE_SUCCESS);
}

/*****************************************
 *** _M_DRV_MC36XX_SetSniffSampleRate
 *****************************************/
static int _M_DRV_MC36XX_SetSniffSampleRate(E_M_DRV_MC36XX_SNIFF_SR eSR)
{
	//if(s_debug) M_PRINTF("[%s]", __func__);

	unsigned char _bRegWAKE = 0x00;
	unsigned char _bRegSNIFF = 0xC0;

	_M_DRV_MC36XX_REG_READ(M_DRV_MC36XX_REG_WAKE_C, &_bRegWAKE, 1);
	_M_DRV_MC36XX_SetSniffOverSampleRate(M_DRV_MC36XX_SNIFF_SR_MODE_LOW_POWER);

	_bRegSNIFF |= eSR;
	_M_DRV_MC36XX_REG_WRITE(M_DRV_MC36XX_REG_SNIFF_C, &_bRegSNIFF, 1);
	//if(s_debug)  M_PRINTF("[%s] REG[0x%02X] 0x%02X", __func__, M_DRV_MC36XX_REG_SNIFF_C, _bRegSNIFF );


	_bRegWAKE  &= 0x7F;
	_M_DRV_MC36XX_REG_WRITE(M_DRV_MC36XX_REG_WAKE_C, &_bRegWAKE, 1);
	//if(s_debug)  M_PRINTF("[%s] REG[0x%02X] 0x%02X", __func__, M_DRV_MC36XX_REG_WAKE_C, _bRegWAKE );

	return (M_DRV_MC36XX_RETCODE_SUCCESS);
}

#ifdef M_DRV_MC36XX_SUPPORT_LPF
/*****************************************
 *** _M_DRV_MC36XX_LowPassFilter
 *****************************************/
void _M_DRV_MC36XX_LowPassFilter(signed short _saData[M_DRV_MC36XX_AXES_NUM])
{
	//if(s_debug) M_PRINTF("[%s]", __func__);

	//if(s_debug) 
	//	M_PRINTF("[CurrData]     %d,    %d,    %d\r\n",
		//	_saData[M_DRV_MC36XX_AXIS_X], _saData[M_DRV_MC36XX_AXIS_Y], _saData[M_DRV_MC36XX_AXIS_Z]);

	//if(s_debug)
	//	M_PRINTF("[PreData]     %d,    %d,    %d\r\n",
		//	_saLPFPrevData[M_DRV_MC36XX_AXIS_X], _saLPFPrevData[M_DRV_MC36XX_AXIS_Y], _saLPFPrevData[M_DRV_MC36XX_AXIS_Z]);

	_saData[M_DRV_MC36XX_AXIS_X] = _M_DRV_MC36XX_SENSOR_FILTER(_saLPFPrevData[M_DRV_MC36XX_AXIS_X], _saData[M_DRV_MC36XX_AXIS_X]);
	_saData[M_DRV_MC36XX_AXIS_Y] = _M_DRV_MC36XX_SENSOR_FILTER(_saLPFPrevData[M_DRV_MC36XX_AXIS_Y], _saData[M_DRV_MC36XX_AXIS_Y]);
	_saData[M_DRV_MC36XX_AXIS_Z] = _M_DRV_MC36XX_SENSOR_FILTER(_saLPFPrevData[M_DRV_MC36XX_AXIS_Z], _saData[M_DRV_MC36XX_AXIS_Z]);

	_saLPFPrevData[M_DRV_MC36XX_AXIS_X] = _saData[M_DRV_MC36XX_AXIS_X];
	_saLPFPrevData[M_DRV_MC36XX_AXIS_Y] = _saData[M_DRV_MC36XX_AXIS_Y];
	_saLPFPrevData[M_DRV_MC36XX_AXIS_Z] = _saData[M_DRV_MC36XX_AXIS_Z];

	//if(s_debug)
		// M_PRINTF("[M_DRV_MC36XX_SUPPORT_LPF]     %d,    %d,    %d\r\n",
			//	_saData[M_DRV_MC36XX_AXIS_X], _saData[M_DRV_MC36XX_AXIS_Y], _saData[M_DRV_MC36XX_AXIS_Z]);
}
#endif    // END OF #ifdef M_DRV_MC36XX_SUPPORT_LPF

/*****************************************
 *** _M_DRV_MC36XX_SetSniffThreshold
 *****************************************/
static int _M_DRV_MC36XX_SetSniffThreshold(int axis, uint8_t sniff_thr )
{
	unsigned char _bRegSniff_addr = 0;
	//if(s_debug) M_PRINTF("[%s]", __func__);
	_M_DRV_MC36XX_REG_READ(M_DRV_MC36XX_REG_SNIFFTH_C, &s_bCfgSniffThr, 1);
//	unsigned char _bRegSniff_addr = 0;
	switch(axis)
	{
	case M_DRV_MC36XX_AXIS_X:
		_bRegSniff_addr = 0x01; //Put X-axis to active
		_M_DRV_MC36XX_REG_WRITE(M_DRV_MC36XX_REG_SNIFF_CFG, &_bRegSniff_addr, 1);
		break;
	case M_DRV_MC36XX_AXIS_Y: //Put Y-axis to active
		_bRegSniff_addr = 0x02;
		_M_DRV_MC36XX_REG_WRITE(M_DRV_MC36XX_REG_SNIFF_CFG, &_bRegSniff_addr, 1);
		break;
	case M_DRV_MC36XX_AXIS_Z: //Put Z-axis to active
		_bRegSniff_addr = 0x03;
		_M_DRV_MC36XX_REG_WRITE(M_DRV_MC36XX_REG_SNIFF_CFG, &_bRegSniff_addr, 1);
		break;
	default:
		break;
	}

	//if(s_debug)  M_PRINTF("[%s] REG[0x%02X] 0x%02X", __func__, M_DRV_MC36XX_REG_SNIFF_CFG, _bRegSniff_addr);

	s_bCfgSniffThr |= sniff_thr;
	_M_DRV_MC36XX_REG_WRITE(M_DRV_MC36XX_REG_SNIFFTH_C, &s_bCfgSniffThr, 1);

	//if(s_debug)  M_PRINTF("[%s] REG[0x%02X] 0x%02X", __func__, M_DRV_MC36XX_REG_SNIFFTH_C, s_bCfgSniffThr);

	return (M_DRV_MC36XX_RETCODE_SUCCESS);
}

/*********************************************************************
 *** _M_DRV_MC36XX_CfgSniff
 *********************************************************************/
static void _M_DRV_MC36XX_CfgSniff(void)
{
	//if(s_debug) M_PRINTF("[%s]", __func__);
	_M_DRV_MC36XX_ResetChip();
	_M_DRV_MC36XX_SetBusIF();

	_M_DRV_MC36XX_SetSniffAGAIN(E_M_DRV_MC36XX_SNIFF_GAIN_HIGH);
	_M_DRV_MC36XX_SetSniffSampleRate(s_eSR_SNIFF);

	/* Orion 2X SNIFF threshold*/
	_M_DRV_MC36XX_SetSniffThreshold(M_DRV_MC36XX_AXIS_X, 4);
	_M_DRV_MC36XX_SetSniffThreshold(M_DRV_MC36XX_AXIS_Y, 4);
	_M_DRV_MC36XX_SetSniffThreshold(M_DRV_MC36XX_AXIS_Z, 4);

	_M_DRV_MC36XX_REG_WRITE(0x15, &s_bCfgRngResol, 1);
	_M_DRV_MC36XX_REG_WRITE(0x16, &s_bCfgFifo, 1);
	_M_DRV_MC36XX_REG_WRITE(0x17, &s_bCfgINT, 1);
}
#if 0
/*********************************************************************
 *** _M_DRV_MC36XX_CfgWake
 *********************************************************************/
static void _M_DRV_MC36XX_CfgWake(void)
{
	//if(s_debug) M_PRINTF("[%s]", __func__);
	_M_DRV_MC36XX_SetBusIF();
	_M_DRV_MC36XX_SetWakeAGAIN(E_M_DRV_MC36XX_WAKE_GAIN_LOW);
	_M_DRV_MC36XX_SetCwakeSampleRate(s_eSR_CWAKE);
}
#endif 
/*********************************************************************
 *** _M_DRV_MC36XX_SetMode
 *********************************************************************/
static int _M_DRV_MC36XX_SetMode(E_M_DRV_MC36XX_MODE eNextMode)
{

	unsigned char _bCurrMode = 0;
	unsigned char _bRegMODE_C = 0;
	unsigned char _bGuard = 0;

	_M_DRV_MC36XX_REG_READ(M_DRV_MC36XX_REG_STATUS_1, &_bCurrMode, 1);

	if (eNextMode == _M_DRV_MC36XX_REG_STATUS_1_MODE(_bCurrMode))
		return (M_DRV_MC36XX_RETCODE_ERROR_STATUS);

	if (E_M_DRV_MC36XX_MODE_SNIFF == eNextMode)
	{
		//if(s_debug) M_PRINTF("[%s] MODE_SNIFF", __func__);
		_M_DRV_MC36XX_CfgSniff();
		_M_DRV_MC36XX_REG_WRITE(M_DRV_MC36XX_REG_PWR_CONTROL, &s_bCfgFifoVdd, 1);
	}
	else if (E_M_DRV_MC36XX_MODE_SLEEP == eNextMode)
	{
		//if(s_debug) M_PRINTF("[%s] MODE_SLEEP", __func__);
		_M_DRV_MC36XX_REG_WRITE(M_DRV_MC36XX_REG_PWR_CONTROL, &s_bCfgFifoVdd, 1);
	}
	else if (E_M_DRV_MC36XX_MODE_STANDBY == eNextMode)
	{
		//if(s_debug) M_PRINTF("[%s] MODE_STANDBY", __func__);
	}
	else
	{
		//if(s_debug) M_PRINTF("[%s] MODE WAKE", __func__);
		if (E_M_DRV_MC36XX_MODE_CWAKE == eNextMode)
		{
			//if(s_debug) M_PRINTF("[%s] MODE CWAKE", __func__);
			_M_DRV_MC36XX_SetCwakeSampleRate(s_eSR_CWAKE);
		}

	}

	_bRegMODE_C |= eNextMode;

	_M_DRV_MC36XX_REG_WRITE(M_DRV_MC36XX_REG_MODE_C, &_bRegMODE_C, 1);
	//if(s_debug)  M_PRINTF("[%s] REG[0x%02X] 0x%02X", __func__, M_DRV_MC36XX_REG_MODE_C, _bRegMODE_C);

	while (1)
	{
		_bGuard++;

		_M_DRV_MC36XX_Delay(1);

		_M_DRV_MC36XX_REG_READ(M_DRV_MC36XX_REG_STATUS_1, &_bCurrMode, 1);

		if (eNextMode == _M_DRV_MC36XX_REG_STATUS_1_MODE(_bCurrMode))
			break;

		if (_bGuard > 64)
			return (M_DRV_MC36XX_RETCODE_ERROR_SETUP);
	}

	return (M_DRV_MC36XX_RETCODE_SUCCESS);
}

#if 0
/*****************************************
 *** _M_DRV_MC36XX_SetSniffAndOrN
 *****************************************/
static int _M_DRV_MC36XX_SetSniffAndOrN(uint8_t LogicAndOr)
{
	//if(s_debug) M_PRINTF("[%s]", __func__);
	unsigned char _bRegAndOrN = 0x00;
	_M_DRV_MC36XX_REG_READ(M_DRV_MC36XX_REG_SNIFFTH_C, &_bRegAndOrN, 1);

	if(LogicAndOr == 1)
		_bRegAndOrN |= (LogicAndOr << 6);
	else if (LogicAndOr == 0)
		_bRegAndOrN &= 0xBF;
	else
		return M_DRV_MC36XX_RETCODE_ERROR_WRONG_ARGUMENT;

	_M_DRV_MC36XX_REG_WRITE(M_DRV_MC36XX_REG_DMY, &_bRegAndOrN, 1);
	return (M_DRV_MC36XX_RETCODE_SUCCESS);
}


/*****************************************
 *** _M_DRV_MC36XX_SetSniffDetectCount
 *****************************************/
static int _M_DRV_MC36XX_SetSniffDetectCount(uint8_t axis, uint8_t SniffCount)
{
	//if(s_debug) M_PRINTF("[%s]", __func__);

	_M_DRV_MC36XX_REG_READ(M_DRV_MC36XX_REG_SNIFFTH_C, &s_bCfgSniffThr, 1);
	_M_DRV_MC36XX_REG_READ(M_DRV_MC36XX_REG_SNIFF_CFG, &s_bCfgSniffCfg, 1);

	unsigned char _bRegSniff_Axis = 0;
	unsigned char _bRegSniff_Count = SniffCount; //unsigned SNIFF event count, 1 to 62 events, independent from other channels
	unsigned char _bRegSniff_Count_En = 0x08;
	switch(axis)
	{
	case M_DRV_MC36XX_AXIS_X:
		_bRegSniff_Axis = 0x05; //Select X detection count shadow register
		s_bCfgSniffCfg |= _bRegSniff_Axis;
		_M_DRV_MC36XX_REG_WRITE(M_DRV_MC36XX_REG_SNIFF_CFG, &s_bCfgSniffCfg, 1);

		break;
	case M_DRV_MC36XX_AXIS_Y:
		_bRegSniff_Axis = 0x06; //Select Y detection count shadow register
		s_bCfgSniffCfg |= _bRegSniff_Axis;
		_M_DRV_MC36XX_REG_WRITE(M_DRV_MC36XX_REG_SNIFF_CFG, &s_bCfgSniffCfg, 1);
		break;
	case M_DRV_MC36XX_AXIS_Z:
		_bRegSniff_Axis = 0x07; //Select Z detection count shadow register
		s_bCfgSniffCfg |= _bRegSniff_Axis;
		_M_DRV_MC36XX_REG_WRITE(M_DRV_MC36XX_REG_SNIFF_CFG, &s_bCfgSniffCfg, 1);
		break;
	default:
		break;
	}

	//if(s_debug)  M_PRINTF("[%s] REG[0x%02X] 0x%02X", __func__, M_DRV_MC36XX_REG_SNIFF_CFG, s_bCfgSniffCfg);

	/*Set detection count as (count +1) */
	s_bCfgSniffThr |= _bRegSniff_Count;
	_M_DRV_MC36XX_REG_WRITE(M_DRV_MC36XX_REG_SNIFFTH_C, &s_bCfgSniffThr, 1);
	//if(s_debug)  M_PRINTF("[%s] REG[0x%02X] 0x%02X", __func__, M_DRV_MC36XX_REG_SNIFFTH_C, s_bCfgSniffThr);


	/*Enable SNIFF detection counts, required for valid SNIFF wake-up*/
	s_bCfgSniffCfg |= _bRegSniff_Count_En;
	_M_DRV_MC36XX_REG_WRITE(M_DRV_MC36XX_REG_SNIFF_CFG, &s_bCfgSniffCfg, 1);
	//if(s_debug)  M_PRINTF("[%s] REG[0x%02X] 0x%02X", __func__, M_DRV_MC36XX_REG_SNIFF_CFG, s_bCfgSniffCfg);

	return (M_DRV_MC36XX_RETCODE_SUCCESS);
}

#endif 
/*********************************************************************
 *** _M_DRV_MC36XX_ReadData
 *********************************************************************/
#if 0
static int _M_DRV_MC36XX_ReadData(float faOutput[M_DRV_MC36XX_AXES_NUM])
{
	//if(s_debug) M_PRINTF("[%s] ", __func__);

	signed short _waRaw[M_DRV_MC36XX_AXES_NUM] = { 0 };
	unsigned char _baData[6] = { 0 };

	const S_M_DRV_MC_UTIL_OrientationReMap *_ptOrienMap = &g_MDrvUtilOrientationReMap[M_DRV_MC36XX_CFG_ORIENTATION_MAP];

	_M_DRV_MC36XX_REG_READ(M_DRV_MC36XX_REG_XOUT_LSB, _baData, 6);

	_waRaw[M_DRV_MC36XX_AXIS_X] = ((signed short) ((_baData[0])	| (_baData[1] << 8)));
	_waRaw[M_DRV_MC36XX_AXIS_Y] = ((signed short) ((_baData[2])	| (_baData[3] << 8)));
	_waRaw[M_DRV_MC36XX_AXIS_Z] = ((signed short) ((_baData[4])	| (_baData[5] << 8)));

#ifdef M_DRV_MC36XX_SUPPORT_LPF
	_M_DRV_MC36XX_LowPassFilter(_waRaw);
#endif

	faOutput[M_DRV_MC36XX_AXIS_X] =
			((float) (_ptOrienMap->bSign[M_DRV_MC36XX_AXIS_X] * _waRaw[_ptOrienMap->bMap[M_DRV_MC36XX_AXIS_X]]));
	faOutput[M_DRV_MC36XX_AXIS_Y] =
			((float) (_ptOrienMap->bSign[M_DRV_MC36XX_AXIS_Y] * _waRaw[_ptOrienMap->bMap[M_DRV_MC36XX_AXIS_Y]]));
	faOutput[M_DRV_MC36XX_AXIS_Z] =
			((float) (_ptOrienMap->bSign[M_DRV_MC36XX_AXIS_Z] * _waRaw[_ptOrienMap->bMap[M_DRV_MC36XX_AXIS_Z]]));

	faOutput[M_DRV_MC36XX_AXIS_X] = -faOutput[M_DRV_MC36XX_AXIS_X];
	faOutput[M_DRV_MC36XX_AXIS_Y] = -faOutput[M_DRV_MC36XX_AXIS_Y];

	faOutput[M_DRV_MC36XX_AXIS_X] = (faOutput[M_DRV_MC36XX_AXIS_X] * s_fMC3XXX_Sensitivity);
	faOutput[M_DRV_MC36XX_AXIS_Y] = (faOutput[M_DRV_MC36XX_AXIS_Y] * s_fMC3XXX_Sensitivity);
	faOutput[M_DRV_MC36XX_AXIS_Z] = (faOutput[M_DRV_MC36XX_AXIS_Z] * s_fMC3XXX_Sensitivity);

	return (M_DRV_MC36XX_RETCODE_SUCCESS);
}
#endif

/*******************************************************************************
 *** FUNCTION
 *******************************************************************************/

/*********************************************************************
 *** M_DRV_MC36XX_SetMode
 *********************************************************************/
int M_DRV_MC36XX_SetMode(E_M_DRV_MC36XX_MODE eNextMode)
{
	//if(s_debug) M_PRINTF("[%s]", __func__);
	_M_DRV_MC36XX_SetMode(E_M_DRV_MC36XX_MODE_STANDBY);
	_M_DRV_MC36XX_SetMode(eNextMode);
	return (M_DRV_MC36XX_RETCODE_SUCCESS);
}

/*****************************************
 *** M_DRV_MC36XX_ConfigRegRngResCtrl
 *****************************************/
int M_DRV_MC36XX_ConfigRegRngResCtrl(E_M_DRV_MC36XX_RANGE eCfgRange, E_M_DRV_MC36XX_RESOLUTION eCfgResolution)
{
	//if(s_debug) M_PRINTF("[%s]", __func__);
	unsigned char _bPreMode = 0;
	float _faRange[E_M_DRV_MC36XX_RANGE_END] = { 19.614f, 39.228f, 78.456f,	156.912f, 117.684f, 235.368f };
	float _faResolution[E_M_DRV_MC36XX_RESOLUTION_END] = { 32.0f, 64.0f, 128.0f, 512.0f, 2048.0f, 8192.0f };

	_M_DRV_MC36XX_REG_READ(M_DRV_MC36XX_REG_MODE_C, &_bPreMode, 1);
	_M_DRV_MC36XX_SetMode(E_M_DRV_MC36XX_MODE_STANDBY);

#ifdef M_DRV_MC36XX_CFG_WA_10
	if (E_M_DRV_MC36XX_RESOLUTION_12BIT < eCfgResolution)
	{
		s_bCfgFifo = 0x80;
		_M_DRV_MC36XX_REG_WRITE(M_DRV_MC36XX_REG_FIFO_C, &s_bCfgFifo, 1);
		s_bCfgFifo = 0x00;
		_M_DRV_MC36XX_REG_WRITE(M_DRV_MC36XX_REG_FIFO_C, &s_bCfgFifo, 1);
	}
#endif

	s_bCfgRngResol = (((eCfgRange << 4) & 0x70) | eCfgResolution);
	_M_DRV_MC36XX_REG_WRITE(M_DRV_MC36XX_REG_RANGE_C, &s_bCfgRngResol, 1);
	//if(s_debug) M_PRINTF("[%s] REG[0x%02X] 0x%02X", __func__, M_DRV_MC36XX_REG_RANGE_C, s_bCfgRngResol );

	s_fMC3XXX_Sensitivity =	(_faRange[eCfgRange] / _faResolution[eCfgResolution]);
	//if(s_debug) M_PRINTF("[%s] s_fMC3XXX_Sensitivity=%f", __func__, s_fMC3XXX_Sensitivity);
	_M_DRV_MC36XX_SetMode(_M_DRV_MC36XX_REG_MODE_C_MODE(_bPreMode));
	s_eRange = eCfgRange;
	s_eResolution = eCfgResolution;

	return (M_DRV_MC36XX_RETCODE_SUCCESS);
}

/*****************************************
 *** M_DRV_MC36XX_SetSampleRate
 *****************************************/
int M_DRV_MC36XX_SetSampleRate(E_M_DRV_MC36XX_CWAKE_SR eCwakeSR, E_M_DRV_MC36XX_SNIFF_SR eSniffSR)
{
	//if(s_debug) M_PRINTF("[%s]", __func__);
	unsigned char _bPreMode = 0;

	if (M_DRV_MC36XX_RETCODE_SUCCESS != _M_DRV_MC36XX_CheckCwakeSampleRate(eCwakeSR))
		return (M_DRV_MC36XX_RETCODE_ERROR_WRONG_ARGUMENT);

	s_eSR_CWAKE = eCwakeSR;
	s_eSR_SNIFF = eSniffSR;

	_M_DRV_MC36XX_REG_READ(M_DRV_MC36XX_REG_MODE_C, &_bPreMode, 1);
	_M_DRV_MC36XX_SetMode(E_M_DRV_MC36XX_MODE_STANDBY);
	_M_DRV_MC36XX_SetMode(_M_DRV_MC36XX_REG_MODE_C_MODE(_bPreMode));

	return (M_DRV_MC36XX_RETCODE_SUCCESS);
}



/*********************************************************************
 *** M_DRV_MC36XX_Init
 *********************************************************************/
int M_DRV_MC36XX_Init(void)
{
	//if(s_debug) M_PRINTF("[%s]", __func__);

	if (M_DRV_MC36XX_RETCODE_SUCCESS != _M_DRV_MC36XX_ValidateSensorIC())
		return (M_DRV_MC36XX_RETCODE_ERROR_IDENTIFICATION);

	_M_DRV_MC36XX_ResetChip();

	/* Config Bus Type either SPI or I2C */
    _M_DRV_MC36XX_SetBusIF();

	_M_DRV_MC36XX_REG_WRITE(M_DRV_MC36XX_REG_PWR_CONTROL, &s_bCfgFifoVdd, 1);

	/*Config Range and Resolution */
	M_DRV_MC36XX_ConfigRegRngResCtrl(s_eRange, s_eResolution);

	/*Config Sniff and CWake Sample Rate */
	_M_DRV_MC36XX_SetSniffSampleRate(s_eSR_SNIFF);
	_M_DRV_MC36XX_SetCwakeSampleRate(s_eSR_CWAKE);

	/* Config Sniff and CWake Analog Gain */
	_M_DRV_MC36XX_SetWakeAGAIN(s_eGAIN_WAKE);
	_M_DRV_MC36XX_SetSniffAGAIN(s_eGAIN_SNIFF);

#ifdef M_DRV_MC36XX_OPERATE_MODE_WAKE_WHEN_READ
	_M_DRV_MC36XX_SetMode(E_M_DRV_MC36XX_MODE_SLEEP);
#endif

#ifdef M_DRV_MC36XX_OPERATE_MODE_ALWAYS_WAKE
	_M_DRV_MC36XX_SetMode(E_M_DRV_MC36XX_MODE_CWAKE);
#endif

	_M_DRV_MC36XX_REG_READ(M_DRV_MC36XX_REG_FIFO_C, &s_bCfgFifo, 1);
	_M_DRV_MC36XX_REG_READ(M_DRV_MC36XX_REG_INTR_C, &s_bCfgINT, 1);

	return (M_DRV_MC36XX_RETCODE_SUCCESS);
}



/*********************************************************************
 *** M_DRV_MC36XX_SetGain
 *********************************************************************/
int M_DRV_MC36XX_SetGain(E_M_DRV_MC36XX_WAKE_GAIN eWakeGain, E_M_DRV_MC36XX_SNIFF_GAIN eSniffGain)
{
	//if(s_debug) M_PRINTF("[%s] ", __func__);

	 unsigned char _bPreMode = 0x00;

	 s_eGAIN_WAKE = eWakeGain;
	 s_eGAIN_SNIFF = eSniffGain;

	 _M_DRV_MC36XX_REG_READ(M_DRV_MC36XX_REG_MODE_C, &_bPreMode, 1);
	 _M_DRV_MC36XX_SetMode(E_M_DRV_MC36XX_MODE_STANDBY);
	 _M_DRV_MC36XX_SetMode(_M_DRV_MC36XX_REG_MODE_C_MODE(_bPreMode));

	 return (M_DRV_MC36XX_RETCODE_SUCCESS);
}

/*********************************************************************
 *** M_DRV_MC36XX_EnableFIFO
 *********************************************************************/
int M_DRV_MC36XX_EnableFIFO(
		E_M_DRV_MC36XX_FIFO_CONTROL eCtrl,
		E_M_DRV_MC36XX_FIFO_MODE eMode,
		unsigned char bThreshold)
{
	//if(s_debug) M_PRINTF("[%s] ", __func__);
	unsigned char _bPreMode = 0;

	if (eCtrl >= E_M_DRV_MC36XX_FIFO_CONTROL_END)
		return (M_DRV_MC36XX_RETCODE_ERROR_WRONG_ARGUMENT);

	if (eMode >= E_M_DRV_MC36XX_FIFO_MODE_END)
		return (M_DRV_MC36XX_RETCODE_ERROR_WRONG_ARGUMENT);

	if (bThreshold > 31)
		return (M_DRV_MC36XX_RETCODE_ERROR_WRONG_ARGUMENT);

	_M_DRV_MC36XX_REG_READ(M_DRV_MC36XX_REG_MODE_C, &_bPreMode, 1);
	_M_DRV_MC36XX_SetMode(E_M_DRV_MC36XX_MODE_STANDBY);

#ifdef M_DRV_MC36XX_CFG_WA_10
	{
		unsigned char _bRegRANGE_C = 0;
		_M_DRV_MC36XX_REG_READ(M_DRV_MC36XX_REG_RANGE_C, &_bRegRANGE_C, 1);

		if (E_M_DRV_MC36XX_RESOLUTION_12BIT < _M_DRV_MC36XX_REG_RANGE_C_RES(_bRegRANGE_C))
			M_DRV_MC36XX_ConfigRegRngResCtrl( _M_DRV_MC36XX_REG_RANGE_C_RANGE(_bRegRANGE_C), E_M_DRV_MC36XX_RESOLUTION_12BIT);
	}
#endif

	s_bCfgFifo = ((eCtrl << 6) | (eMode << 5) | bThreshold);
	_M_DRV_MC36XX_REG_WRITE(M_DRV_MC36XX_REG_FIFO_C, &s_bCfgFifo, 1);
	_M_DRV_MC36XX_SetMode(_M_DRV_MC36XX_REG_MODE_C_MODE(_bPreMode));

	return (M_DRV_MC36XX_RETCODE_SUCCESS);
}

/*********************************************************************
 *** M_DRV_MC36XX_ConfigINT
 *********************************************************************/
int M_DRV_MC36XX_ConfigINT(
		unsigned char bFifoThreshEnable,
		unsigned char bFifoFullEnable,
		unsigned char bFifoEmptyEnable,
		unsigned char bACQEnable,
		unsigned char bWakeEnable)
{

	//if(s_debug) M_PRINTF("[%s] ", __func__);

	unsigned char _bPreMode = 0;

	_M_DRV_MC36XX_REG_READ(M_DRV_MC36XX_REG_MODE_C, &_bPreMode, 1);
	_M_DRV_MC36XX_SetMode(E_M_DRV_MC36XX_MODE_STANDBY);

	s_bCfgINT = (((bFifoThreshEnable & 0x01) << 6)
			  | ((bFifoFullEnable & 0x01) << 5)
			  | ((bFifoEmptyEnable & 0x01) << 4)
			  | ((bACQEnable & 0x01) << 3)
			  | ((bWakeEnable & 0x01) << 2)
			  | M_DRV_MC36XX_INTR_C_IAH_ACTIVE_LOW
			  | M_DRV_MC36XX_INTR_C_IPP_MODE_PUSH_PULL);

	_M_DRV_MC36XX_REG_WRITE(M_DRV_MC36XX_REG_INTR_C, &s_bCfgINT, 1);
	_M_DRV_MC36XX_SetMode(_M_DRV_MC36XX_REG_MODE_C_MODE(_bPreMode));

	return (M_DRV_MC36XX_RETCODE_SUCCESS);
}

/*********************************************************************
 *** M_DRV_MC36XX_ReadData
 *********************************************************************/
#if 0
int M_DRV_MC36XX_ReadData(float faOutput[M_DRV_MC36XX_FIFO_DEPTH][M_DRV_MC36XX_AXES_NUM], int nNumOfSample)
{

	int _nDataCount = 0;
	unsigned char _bRegStatus1 = 0;
	unsigned char _bRegFIFO_C = 0;

	if ((M_DRV_MC36XX_NULL_ADDR == faOutput) || (0 == nNumOfSample))
		return (M_DRV_MC36XX_RETCODE_ERROR_WRONG_ARGUMENT);

#ifdef M_DRV_MC36XX_OPERATE_MODE_WAKE_WHEN_READ
	_M_DRV_MC36XX_SetMode(E_M_DRV_MC36XX_MODE_STANDBY);
	_M_DRV_MC36XX_SetCwakeSampleRate(s_eSR_CWAKE);
	_M_DRV_MC36XX_SetMode(E_M_DRV_MC36XX_MODE_CWAKE);
#endif

	_M_DRV_MC36XX_REG_READ(M_DRV_MC36XX_REG_STATUS_1, &_bRegStatus1, 1);
	_M_DRV_MC36XX_REG_READ(M_DRV_MC36XX_REG_FIFO_C, &_bRegFIFO_C, 1);

	/* FIFO Mode */
	if (_M_DRV_MC36XX_REG_FIFO_C_FIFO_EN(_bRegFIFO_C))
	{

		if (M_DRV_MC36XX_FIFO_DEPTH < nNumOfSample)
			nNumOfSample = M_DRV_MC36XX_FIFO_DEPTH;

		//if(s_debug) M_PRINTF("[%s] FIFO mode read data", __func__);

		if (_M_DRV_MC36XX_REG_STATUS_1_FIFO_EMPTY(_bRegStatus1))
			return (M_DRV_MC36XX_RETCODE_ERROR_NO_DATA);

		for (_nDataCount = 0; _nDataCount < nNumOfSample; _nDataCount++)
		{
			_M_DRV_MC36XX_ReadData(faOutput[_nDataCount]);
			_M_DRV_MC36XX_REG_READ(M_DRV_MC36XX_REG_STATUS_1, &_bRegStatus1, 1);

			if (_M_DRV_MC36XX_REG_STATUS_1_FIFO_EMPTY(_bRegStatus1))
			{
				_nDataCount++;
				break;
			}
		}
	}
	/* Normal Mode */
	else
	{
		//if(s_debug) M_PRINTF("[%s] Normal mode read data", __func__);
		/*
		 * 0: No new sample data has arrived since last read.
		 * 1: New sample data has arrived and has been written to FIFO/registers
		 */
		if (!_M_DRV_MC36XX_REG_STATUS_1_NEW_DATA(_bRegStatus1))
			return (M_DRV_MC36XX_RETCODE_ERROR_NO_DATA);

		_M_DRV_MC36XX_ReadData(faOutput[0]);
		_nDataCount = 1;
	}

	return (_nDataCount);
}
#endif

/*********************************************************************
 *** M_DRV_MC36XX_HandleINT
 *********************************************************************/
int M_DRV_MC36XX_HandleINT(S_M_DRV_MC36XX_InterruptEvent *ptINT_Event)
{
	unsigned char _bRegStatus2 = 0;

	_M_DRV_MC36XX_REG_READ(M_DRV_MC36XX_REG_STATUS_2, &_bRegStatus2, 1);

	ptINT_Event->bWAKE = _M_DRV_MC36XX_REG_STATUS_2_INT_WAKE(_bRegStatus2);
	ptINT_Event->bACQ = _M_DRV_MC36XX_REG_STATUS_2_INT_ACQ(_bRegStatus2);
	ptINT_Event->bFIFO_EMPTY = _M_DRV_MC36XX_REG_STATUS_2_INT_FIFO_EMPTY(_bRegStatus2);
	ptINT_Event->bFIFO_FULL = _M_DRV_MC36XX_REG_STATUS_2_INT_FIFO_FULL(_bRegStatus2);
	ptINT_Event->bFIFO_THRESHOLD = _M_DRV_MC36XX_REG_STATUS_2_INT_FIFO_THRESH(_bRegStatus2);
	ptINT_Event->bSWAKE_SNIFF = _M_DRV_MC36XX_REG_STATUS_2_INT_SWAKE_SNIFF(_bRegStatus2);
	
#ifdef	M_DRV_MC36XX_CFG_BUS_SPI  //clear int flag
	_M_DRV_MC36XX_REG_WRITE(M_DRV_MC36XX_REG_STATUS_2, &_bRegStatus2, 1);
#endif
	
	return (M_DRV_MC36XX_RETCODE_SUCCESS);
}

/*********************************************************************
 *** M_DRV_MC36XX_ReadRegMap
 *********************************************************************/
int M_DRV_MC36XX_ReadRegMap(unsigned char baRegMap[M_DRV_MC36XX_REG_MAP_SIZE])
{
	uint8_t _bIndex = 0;
	uint8_t _bRegData = 0;

	for (_bIndex = 0; _bIndex < M_DRV_MC36XX_REG_MAP_SIZE; _bIndex++)
	{
		_M_DRV_MC36XX_REG_READ(_bIndex, &_bRegData, 1);

		//M_PRINTF("REG[0x%02X] 0x%02X\r\n", _bIndex, _bRegData);

		

		if (0 != baRegMap)
			baRegMap[_bIndex] = _bRegData;
	}

	return (M_DRV_MC36XX_RETCODE_SUCCESS);
}

/*********************************************************************
 *** M_DRV_MC36XX_ReadReg
 *********************************************************************/
unsigned char M_DRV_MC36XX_ReadReg(unsigned char bRegAddr)
{
	uint8_t _bData = 0;
	_M_DRV_MC36XX_REG_READ(bRegAddr, &_bData, 1);

	//M_PRINTF("[%s] REG[0x%02X] 0x%02X", __func__, bRegAddr, _bData );
	return (_bData);
}
//#define ANALOG_GAIN_ENABLE

#ifdef ANALOG_GAIN_ENABLE
uint8_t buffer[6];
uint16_t xdoffh,ydoffh,zdoffh;
int16_t xdoff,ydoff,zdoff;
int16_t xdgain, ydgain, zdgain;
uint8_t xaofs, yaofs, zaofs;
int16_t xaofs10, yaofs10, zaofs10;
int16_t xaofs40, yaofs40, zaofs40;
int16_t xaofs11, yaofs11, zaofs11;
int16_t xaofs41, yaofs41, zaofs41;
uint16_t xmeasgain, ymeasgain, zmeasgain;
int32_t xrbmout1g1xgain, yrbmout1g1xgain, zrbmout1g1xgain;
int32_t xrbmout0g4xgain, yrbmout0g4xgain, zrbmout0g4xgain;
int32_t xrbmout1g4xgain, yrbmout1g4xgain, zrbmout1g4xgain;
int32_t xm4xrbmsens, ym4xrbmsens, zm4xrbmsens;
int32_t xm1rbmsens, ym1rbmsens, zm1rbmsens;
int32_t xrbmout0g1xgain,yrbmout0g1xgain, zrbmout0g1xgain;
int16_t xgain4x, ygain4x, zgain4x;
int16_t xoff4x, yoff4x, zoff4x;
#endif 

#ifdef ANALOG_GAIN_ENABLE

void M_DRV_MC36XX_ReadDoff_Dgain(void)
{
    unsigned char _bReg = 0;
	_bReg = 0x0B;
     //set to high odr first 
	_M_DRV_MC36XX_REG_WRITE(M_DRV_MC36XX_REG_WAKE_C, &_bReg, 1);
	
	//read 15-bit doffset
	_M_DRV_MC36XX_REG_READ(M_DRV_MC36XX_REG_XOFFH, &_bReg, 1);
	xdoffh = _bReg;
    _M_DRV_MC36XX_REG_READ(M_DRV_MC36XX_REG_XOFFL, &_bReg, 1); 
	xdoff = ((((xdoffh & 0x40) <<1) + xdoffh)<<8) + _bReg;
    M_PRINTF("xdoff = %d\r\n", xdoff);
    _M_DRV_MC36XX_REG_READ(M_DRV_MC36XX_REG_YOFFH, &_bReg, 1);
	ydoffh = _bReg;
    _M_DRV_MC36XX_REG_READ(M_DRV_MC36XX_REG_YOFFL, &_bReg, 1); 
	ydoff = ((((ydoffh & 0x40) <<1) + ydoffh)<<8) + _bReg;
	M_PRINTF("ydoff = %d\r\n", ydoff);
	_M_DRV_MC36XX_REG_READ(M_DRV_MC36XX_REG_ZOFFH, &_bReg, 1);
	zdoffh = _bReg;
    _M_DRV_MC36XX_REG_READ(M_DRV_MC36XX_REG_ZOFFL, &_bReg, 1); 
	zdoff = ((((zdoffh & 0x40) <<1) + zdoffh)<<8) + _bReg;
    M_PRINTF("zdoff = %d\r\n", zdoff); 
    //read 9-bit dgain; 
    _M_DRV_MC36XX_REG_READ(M_DRV_MC36XX_REG_XOFFH, &_bReg, 1);
	xdgain = (_bReg & 0x80) << 8;
	_M_DRV_MC36XX_REG_READ(M_DRV_MC36XX_REG_XGAIN, &_bReg, 1);
	xdgain = xdgain + _bReg;
    M_PRINTF("xdgain = %d\r\n", xdgain); 
    _M_DRV_MC36XX_REG_READ(M_DRV_MC36XX_REG_YOFFH, &_bReg, 1);
	ydgain = (_bReg & 0x80) << 8;
	_M_DRV_MC36XX_REG_READ(M_DRV_MC36XX_REG_YGAIN, &_bReg, 1);
	ydgain = ydgain + _bReg;
    M_PRINTF("ydgain = %d\r\n", ydgain); 
	_M_DRV_MC36XX_REG_READ(M_DRV_MC36XX_REG_ZOFFH, &_bReg, 1);
	zdgain = (_bReg & 0x80) << 8;
	_M_DRV_MC36XX_REG_READ(M_DRV_MC36XX_REG_ZGAIN, &_bReg, 1);
	zdgain = zdgain + _bReg;
	M_PRINTF("zdgain = %d\r\n", zdgain);
	
}

	
void M_DRV_MC36XX_ReadAofsp(void)
{
	unsigned char _bReg = 0;
	_M_DRV_MC36XX_REG_READ(M_DRV_MC36XX_REG_XAOFSP, &_bReg, 1);
	xaofs = _bReg;
	M_PRINTF("xaofsp = %d\r\n", xaofs);
	_M_DRV_MC36XX_REG_READ(M_DRV_MC36XX_REG_YAOFSP, &_bReg, 1);
	yaofs = _bReg;
	M_PRINTF("yaofsp = %d\r\n", yaofs);
	_M_DRV_MC36XX_REG_READ(M_DRV_MC36XX_REG_ZAOFSP, &_bReg, 1);
	zaofs = _bReg;
	M_PRINTF("zaofsp = %d\r\n", zaofs);
}

void M_DRV_MC36XX_SetD0ff4x_DGain4x(void)
{
	unsigned char _bReg = 0;
	_bReg = 0x01; //standby
	_M_DRV_MC36XX_REG_WRITE(M_DRV_MC36XX_REG_MODE_C, &_bReg ,1);

	 _M_DRV_MC36XX_REG_READ(M_DRV_MC36XX_REG_EXT_STAT_2, &_bReg, 1);	

     M_PRINTF("check rbm_reg0x01_beforeSE = 0x%X\r\n", _bReg);
	//enable RBM
	_bReg = 0x24; 
	_M_DRV_MC36XX_REG_WRITE(M_DRV_MC36XX_REG_SEC, &_bReg, 1);
    _bReg = 0x77; 
	_M_DRV_MC36XX_REG_WRITE(M_DRV_MC36XX_REG_SEC, &_bReg, 1);
	_bReg = 0xF2; 
	_M_DRV_MC36XX_REG_WRITE(M_DRV_MC36XX_REG_SEC, &_bReg ,1);
    _bReg = 0x03; //rmb & cwake again 
	_M_DRV_MC36XX_REG_WRITE(M_DRV_MC36XX_REG_DMX, &_bReg ,1);

    _M_DRV_MC36XX_REG_READ(M_DRV_MC36XX_REG_EXT_STAT_2, &_bReg, 1);	

     M_PRINTF("check rbm_reg0x01 = 0x%X\r\n", _bReg);

	_bReg = 0x80; //cwake again 1x 
	_M_DRV_MC36XX_REG_WRITE(M_DRV_MC36XX_REG_DMY, &_bReg ,1);
    _bReg = 0x0D; //cwake
	_M_DRV_MC36XX_REG_WRITE(M_DRV_MC36XX_REG_MODE_C, &_bReg ,1);
	
    mcube_delay_ms(200);
     //while(1)
   	{
	   //  _M_DRV_MC36XX_REG_READ(M_DRV_MC36XX_REG_STATUS_1, &_bReg ,1);
	   //  if(_bReg&0x08)
	    {
			_M_DRV_MC36XX_REG_READ(M_DRV_MC36XX_REG_XOUT_LSB, buffer, 6);
			xaofs10 = buffer[0] | ((unsigned short)buffer[1] << 8);
			yaofs10 = buffer[2] | ((unsigned short)buffer[3] << 8);
			zaofs10 = buffer[4] | ((unsigned short)buffer[5] << 8);
			//break;
	    }	
   }

     mcube_printf("RBM_Data10:%d, %d, %d\r\n", xaofs10, yaofs10, zaofs10);
    
    _bReg = 0x01; //standby
	_M_DRV_MC36XX_REG_WRITE(M_DRV_MC36XX_REG_MODE_C, &_bReg ,1);
	_bReg = 0x40; //cwake again 4x
	_M_DRV_MC36XX_REG_WRITE(M_DRV_MC36XX_REG_DMY, &_bReg ,1);
	_bReg = 0x05; //cwake
	_M_DRV_MC36XX_REG_WRITE(M_DRV_MC36XX_REG_MODE_C, &_bReg ,1);

	  mcube_delay_ms(200);
	 //while(1)
   	{
	   //  _M_DRV_MC36XX_REG_READ(M_DRV_MC36XX_REG_STATUS_1, &_bReg ,1);
	    // if(_bReg&0x08)
	    {
			_M_DRV_MC36XX_REG_READ(M_DRV_MC36XX_REG_XOUT_LSB, buffer, 6);
			xaofs40 = buffer[0] | ((unsigned short)buffer[1] << 8);
			yaofs40 = buffer[2] | ((unsigned short)buffer[3] << 8);
			zaofs40 = buffer[4] | ((unsigned short)buffer[5] << 8);
  			//break;
	    }	
   }	
	mcube_printf("RBM_Data40:%d, %d, %d\r\n", xaofs40, yaofs40, zaofs40);

	_bReg = 0x01; //standby
	_M_DRV_MC36XX_REG_WRITE(M_DRV_MC36XX_REG_MODE_C, &_bReg ,1);

	 //Switch AOFS P to be +1
	
	_bReg = xaofs+1;
	_M_DRV_MC36XX_REG_WRITE(M_DRV_MC36XX_REG_XAOFSP,&_bReg ,1);
	_bReg = yaofs+1;
	_M_DRV_MC36XX_REG_WRITE(M_DRV_MC36XX_REG_YAOFSP,&_bReg ,1);
	_bReg = zaofs+1;
	_M_DRV_MC36XX_REG_WRITE(M_DRV_MC36XX_REG_ZAOFSP,&_bReg ,1);


	_bReg = 0x80; //cwake again 1x 
	_M_DRV_MC36XX_REG_WRITE(M_DRV_MC36XX_REG_DMY, &_bReg ,1);
	_bReg = 0x05; //cwake
	_M_DRV_MC36XX_REG_WRITE(M_DRV_MC36XX_REG_MODE_C, &_bReg ,1);

	 mcube_delay_ms(200);
 	 //while(1)
   	{
	     //_M_DRV_MC36XX_REG_READ(M_DRV_MC36XX_REG_STATUS_1, &_bReg ,1);
	     //if(_bReg&0x08)
	    {
			_M_DRV_MC36XX_REG_READ(M_DRV_MC36XX_REG_XOUT_LSB, buffer, 6);
			xaofs11 = buffer[0] | ((unsigned short)buffer[1] << 8);
			yaofs11 = buffer[2] | ((unsigned short)buffer[3] << 8);
			zaofs11 = buffer[4] | ((unsigned short)buffer[5] << 8);
			//break;
	    }	
   }	
    mcube_printf("RBM_Data11:%d, %d, %d\r\n", xaofs11, yaofs11, zaofs11);

    _bReg = 0x01; //standby
	_M_DRV_MC36XX_REG_WRITE(M_DRV_MC36XX_REG_MODE_C, &_bReg ,1);
	_bReg = 0x40; //cwake again 4x
	_M_DRV_MC36XX_REG_WRITE(M_DRV_MC36XX_REG_DMY, &_bReg ,1);
	_bReg = 0x05; //cwake
	_M_DRV_MC36XX_REG_WRITE(M_DRV_MC36XX_REG_MODE_C, &_bReg ,1);
	 mcube_delay_ms(200);
  	 //while(1)
   	{
	    // _M_DRV_MC36XX_REG_READ(M_DRV_MC36XX_REG_STATUS_1, &_bReg ,1);
	     //if(_bReg&0x08)
	    {
			_M_DRV_MC36XX_REG_READ(M_DRV_MC36XX_REG_XOUT_LSB, buffer, 6);
			xaofs41 = buffer[0] | ((unsigned short)buffer[1] << 8);
			yaofs41 = buffer[2] | ((unsigned short)buffer[3] << 8);
			zaofs41 = buffer[4] | ((unsigned short)buffer[5] << 8);
			//break;
	    }	
   }	
    mcube_printf("RBM_Data41:%d, %d, %d\r\n", xaofs41, yaofs41, zaofs41);

	_bReg = 0x01; //standby
	_M_DRV_MC36XX_REG_WRITE(M_DRV_MC36XX_REG_MODE_C, &_bReg ,1);
	
	if ((xaofs10 == xaofs11) || (yaofs10 == yaofs11) || (zaofs10 == zaofs11))
		xmeasgain = ymeasgain = zmeasgain = 4;
	else
	{
		xmeasgain = (xaofs41 - xaofs40+1)/(xaofs11 - xaofs10);
		ymeasgain = (yaofs41 - yaofs40+1)/(yaofs11 - yaofs10);
		zmeasgain = (zaofs41 - zaofs40+1)/(zaofs11 - zaofs10);
	}

	mcube_printf("xmeasgain = %d\r\n", xmeasgain);
	mcube_printf("ymeasgain = %d\r\n", ymeasgain);
	mcube_printf("zmeasgain = %d\r\n", zmeasgain);
	
	if (xdgain == -40) 
					   xdgain = -39;
	   if (ydgain == -40) 
					   ydgain = -39;
	   if (zdgain == -40) 
					   zdgain = -39;
	   
	   xrbmout1g1xgain = -((xdgain + 40) * xdoff - (256*683))/ (2*xdgain + 80);
	   yrbmout1g1xgain = -((ydgain + 40) * ydoff - (256*683))/ (2*ydgain + 80);
	   zrbmout1g1xgain = -((zdgain + 40) * zdoff - (256*683))/ (2*zdgain + 80);
	   
	   
	   xm1rbmsens = (128*683)/(xdgain + 40);
	   ym1rbmsens = (128*683)/(ydgain + 40);
	   zm1rbmsens = (128*683)/(zdgain + 40);
	   
	   xrbmout0g1xgain = xrbmout1g1xgain - xm1rbmsens; 
	   yrbmout0g1xgain = yrbmout1g1xgain - ym1rbmsens;
	   zrbmout0g1xgain = zrbmout1g1xgain - zm1rbmsens;
	   
	   
	   xrbmout0g4xgain = xrbmout0g1xgain * xmeasgain;
	   yrbmout0g4xgain = yrbmout0g1xgain * ymeasgain;
	   zrbmout0g4xgain = zrbmout0g1xgain * zmeasgain;
	   
	   
	   xrbmout1g4xgain = xrbmout1g1xgain * xmeasgain;
	   yrbmout1g4xgain = yrbmout1g1xgain * ymeasgain;
	   zrbmout1g4xgain = zrbmout1g1xgain * zmeasgain;
	   
	   
	   xm4xrbmsens = xrbmout1g4xgain - xrbmout0g4xgain;
	   ym4xrbmsens = yrbmout1g4xgain - yrbmout0g4xgain;
	   zm4xrbmsens = zrbmout1g4xgain - zrbmout0g4xgain;
	   if (xm4xrbmsens == 0)   xm4xrbmsens = 1;
	   if (ym4xrbmsens == 0)   ym4xrbmsens = 1;
	   if (zm4xrbmsens == 0)	zm4xrbmsens = 1;
	   
	   xgain4x = (349568/xm4xrbmsens) - 40; 
	   ygain4x = (349568/ym4xrbmsens) - 40; 
	   zgain4x = (349568/zm4xrbmsens) - 40; 
	   
	   xoff4x = ((2731 /(40 + xgain4x))*128 - xrbmout1g4xgain) * 2; 
	   yoff4x = ((2731 /(40 + ygain4x))*128 - yrbmout1g4xgain) * 2; 
	   zoff4x = ((2731 /(40 + zgain4x))*128 - zrbmout1g4xgain) * 2; 
	   


	mcube_printf("xgain4x = 0x%x\r\n", xgain4x);
	mcube_printf("ygain4x = 0x%x\r\n", ygain4x);
	mcube_printf("zgain4x = 0x%x\r\n", zgain4x);

	mcube_printf("xoff4x = 0x%x\r\n", xoff4x);
	mcube_printf("yoff4x = 0x%x\r\n", yoff4x);
	mcube_printf("zoff4x = 0x%x\r\n", zoff4x);

    
	_bReg = 0x01; //standby
	_M_DRV_MC36XX_REG_WRITE(M_DRV_MC36XX_REG_MODE_C, &_bReg ,1);
    _bReg = xoff4x & 0xFF;
	_M_DRV_MC36XX_REG_WRITE(M_DRV_MC36XX_REG_XOFFL, &_bReg, 1); 
	_bReg = (xoff4x>>8) & 0x7F ;//+ (xgain4x>>1) & 0x80;
    _M_DRV_MC36XX_REG_WRITE(M_DRV_MC36XX_REG_XOFFH, &_bReg, 1); 

	_bReg = yoff4x & 0xFF;
	_M_DRV_MC36XX_REG_WRITE(M_DRV_MC36XX_REG_YOFFL, &_bReg, 1); 
	_bReg = (yoff4x>>8) & 0x7F;//+ ( ygain4x>>1) & 0x80;;
    _M_DRV_MC36XX_REG_WRITE(M_DRV_MC36XX_REG_YOFFH, &_bReg, 1); 

	_bReg = zoff4x & 0xFF;
	_M_DRV_MC36XX_REG_WRITE(M_DRV_MC36XX_REG_ZOFFL, &_bReg, 1); 
	_bReg = (zoff4x>>8) & 0x7F;// + (zgain4x>>1) & 0x80;
    _M_DRV_MC36XX_REG_WRITE(M_DRV_MC36XX_REG_ZOFFH, &_bReg, 1); 

    _bReg = xgain4x&0xFF; 
	_M_DRV_MC36XX_REG_WRITE(M_DRV_MC36XX_REG_XGAIN,&_bReg, 1); 
	_bReg = ygain4x&0xFF;
	_M_DRV_MC36XX_REG_WRITE(M_DRV_MC36XX_REG_YGAIN,&_bReg, 1); 
	_bReg = zgain4x&0xFF;
	_M_DRV_MC36XX_REG_WRITE(M_DRV_MC36XX_REG_ZGAIN,&_bReg, 1); 

   //disable RMB
   // _bReg = 0x40;
  //	_M_DRV_MC36XX_REG_WRITE(M_DRV_MC36XX_REG_DMX, &_bReg ,1);

	_bReg = 0x01; //cwake again
	_M_DRV_MC36XX_REG_WRITE(M_DRV_MC36XX_REG_DMX, &_bReg ,1);

	_bReg = xaofs;
	_M_DRV_MC36XX_REG_WRITE(M_DRV_MC36XX_REG_XAOFSP,&_bReg ,1);
	_bReg = yaofs;
	_M_DRV_MC36XX_REG_WRITE(M_DRV_MC36XX_REG_YAOFSP,&_bReg ,1);
	_bReg = zaofs;
	_M_DRV_MC36XX_REG_WRITE(M_DRV_MC36XX_REG_ZAOFSP,&_bReg ,1);

    _bReg = 0x24;
	_M_DRV_MC36XX_REG_WRITE(0x15, &_bReg, 1);  // with 4x gain, this is +/- 2g not 8g.  using 8g to get 2g output

	//_bReg = 0x05; //cwake
	//_M_DRV_MC36XX_REG_WRITE(M_DRV_MC36XX_REG_MODE_C, &_bReg ,1);
}
#endif



