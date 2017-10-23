/**
\addtogroup BSP
\{
\addtogroup CONFIG
\{
\addtogroup CUSTOM
\{
*/

/**
****************************************************************************************
*
* @file custom_config_ram.h
*
* @brief Board Support Package. User Configuration file for execution from RAM.
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

#ifndef CUSTOM_CONFIG_RAM_H_
#define CUSTOM_CONFIG_RAM_H_

#include "bsp_definitions.h"

#define dg_configTEMPSENS_ADAPTER               1

#define dg_configGPADC_ADAPTER                  1

#define SERIAL_CONSOLE_RETARGET                 1
#define CONFIG_CUSTOM_PRINT
#define __HEAP_SIZE                             0x0A00

#define dg_configUSE_LP_CLK                     LP_CLK_32768
#define dg_configCODE_LOCATION                  NON_VOLATILE_IS_NONE
#define dg_configEXT_CRYSTAL_FREQ               EXT_CRYSTAL_IS_16M

#define dg_configIMAGE_SETUP                    DEVELOPMENT_MODE
#define dg_configEMULATE_OTP_COPY               (0)

#define dg_configUSER_CAN_USE_TIMER1            (0)

#define dg_configUSE_WDOG                       (0)


#define dg_configFLASH_CONNECTED_TO             (FLASH_CONNECTED_TO_1V8)
#define dg_configFLASH_POWER_DOWN               (0)

#define dg_configPOWER_1V8_ACTIVE               (1)
#define dg_configPOWER_1V8_SLEEP                (1)

#define dg_configBATTERY_TYPE                   (BATTERY_TYPE_LIMN2O4)
#define dg_configBATTERY_CHARGE_CURRENT         2       // 30mA
#define dg_configBATTERY_PRECHARGE_CURRENT      20      // 2.1mA
#define dg_configBATTERY_CHARGE_NTC             1       // disabled

#define dg_configUSE_USB                        0
#define dg_configUSE_USB_CHARGER                0
#define dg_configALLOW_CHARGING_NOT_ENUM        1

#define dg_configUSE_ProDK                      (1)

#define dg_configUSE_SW_CURSOR                  (1)


/*************************************************************************************************\
 * FreeRTOS specific config
 */
#define OS_FREERTOS                              /* Define this to use FreeRTOS */
#define configTOTAL_HEAP_SIZE                    14750   /* This is the FreeRTOS Total Heap Size */

/*************************************************************************************************\
 * Peripheral specific config
 */
#define dg_configUSE_HW_I2C                     (1)
#define dg_configUSE_HW_IRGEN                   (1)
#define dg_configUSE_HW_QUAD                    (1)
#define dg_configUSE_HW_RF                      (0)
#define dg_configUSE_HW_SPI                     (1)
#define dg_configUSE_HW_TIMER0                  (1)
#define dg_configUSE_HW_TIMER2                  (1)

#define dg_configI2C_ADAPTER                    (1)
#define dg_configSPI_ADAPTER                    (1)
#define dg_configUART_ADAPTER                   (1)
#define dg_configRF_ADAPTER                     (0)

/* Include bsp default values */
#include "bsp_defaults.h"

/*************************************************************************************************\
 * Memory layout configuration
 */
#if (dg_configCODE_LOCATION == NON_VOLATILE_IS_NONE)
                #define CODE_SIZE       ( 79 * 1024)

        #if (dg_configEXEC_MODE == MODE_IS_CACHED)
                #warning "RAM cached mode is not supported! Reset to RAM (mirrored) mode!"
                #undef dg_configEXEC_MODE
                #define dg_configEXEC_MODE      MODE_IS_RAM
        #endif

                /* DA14681-01
                 * CODE is first, RetRAM follows. RAM is last, always 16K.
                 *
                 * RetRAM uses all RAM5 block. RAM uses CACHE.
                 */
                #define RETRAM_FIRST    1

                #define RAM_SIZE        ( 16 * 1024)
                #define RETRAM_0_SIZE   (128 * 1024 - CODE_SIZE)
                #define RETRAM_1_SIZE   (  0 * 1024)

#else
        #error "Unknown configuration..."
#endif

#endif /* CUSTOM_CONFIG_RAM_H_ */

/**
\}
\}
\}
*/
