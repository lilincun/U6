#ifndef __MCUBE_CUSTOM_CONFIG_H__
#define __MCUBE_CUSTOM_CONFIG_H__

#ifdef __cplusplus
extern "C" {
#endif

/***********************************************
 *** Platform related headers and macros
 ***********************************************/
//#include "common_uart.h"
//#define mcube_printf printf_string



#define M_DRV_MC36XX_CFG_BUS_I2C    // !!! DO NOT use both I2C and SPI simutaneously
//#define M_DRV_MC36XX_CFG_BUS_SPI

#ifdef M_DRV_MC36XX_CFG_BUS_SPI
 #define SPI_TWO_BYTE_ADDRESS  0
#endif 

#if (!defined (M_DRV_MC36XX_CFG_BUS_SPI) && !defined (M_DRV_MC36XX_CFG_BUS_I2C))
    #error "MUST use one bus to access register!"
#endif

#if (defined (M_DRV_MC36XX_CFG_BUS_SPI) && defined (M_DRV_MC36XX_CFG_BUS_I2C))
    #error "DO NOT use both SPI and I2C simutaneously!"
#endif

//#define M_DRV_MC36XX_SUPPORT_LPF

#define M_DRV_MC_UTL_AXIS_X      0
#define M_DRV_MC_UTL_AXIS_Y      1
#define M_DRV_MC_UTL_AXIS_Z      2
#define M_DRV_MC_UTL_AXES_NUM    3

/*****************************************************************************
 *** DATA TYPE / STRUCTURE DEFINITION / ENUM
 *****************************************************************************/
typedef struct
{
    signed char      bSign[M_DRV_MC_UTL_AXES_NUM];
    unsigned char    bMap[M_DRV_MC_UTL_AXES_NUM];
}   S_M_DRV_MC_UTIL_OrientationReMap;

typedef enum
{
    E_M_DRV_UTIL_ORIENTATION_TOP_LEFT_DOWN = 0,
    E_M_DRV_UTIL_ORIENTATION_TOP_RIGHT_DOWN,
    E_M_DRV_UTIL_ORIENTATION_TOP_RIGHT_UP,
    E_M_DRV_UTIL_ORIENTATION_TOP_LEFT_UP,
    E_M_DRV_UTIL_ORIENTATION_BOTTOM_LEFT_DOWN,
    E_M_DRV_UTIL_ORIENTATION_BOTTOM_RIGHT_DOWN,
    E_M_DRV_UTIL_ORIENTATION_BOTTOM_RIGHT_UP,
    E_M_DRV_UTIL_ORIENTATION_BOTTOM_LEFT_UP,
    E_M_DRV_UTIL_ORIENTATION_TOTAL_CONFIG
}   E_M_DRV_UTIL_OrientationConfig;

/*******************************************************************************
 *** GLOBAL VARIABLE
 *******************************************************************************/

extern const S_M_DRV_MC_UTIL_OrientationReMap    g_MDrvUtilOrientationReMap[E_M_DRV_UTIL_ORIENTATION_TOTAL_CONFIG];

#ifdef  M_DRV_MC36XX_CFG_BUS_SPI
//extern int dialog_spi_init(void);
#endif 
// Export functions to other files
extern void mcube_printf(const char *format, ...);
extern void mcube_delay_ms(unsigned int ms);
extern unsigned char mcube_write_regs(unsigned char register_address, unsigned char *value, unsigned char number_of_bytes);
extern unsigned char mcube_read_regs(unsigned char register_address, unsigned char *destination, unsigned char number_of_bytes);
extern unsigned long mcube_burst_read(unsigned long address, unsigned char *buf, unsigned long size);

#ifdef __cplusplus
}
#endif

#endif
