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
* @file custom_config_qspi.h
*
* @brief Board Support Package. User Configuration file for cached QSPI mode.
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

#ifndef CUSTOM_CONFIG_QSPI_H_
#define CUSTOM_CONFIG_QSPI_H_

#include "bsp_definitions.h"

#define CONFIG_USE_BLE

#define dg_configCACHEABLE_QSPI_AREA_LEN        (NVMS_PARAM_PART_start - MEMORY_QSPIF_BASE)



/* Chose UART that will be used in loader 0 - no UART, 1 or 2 */
#define LOADER_UART                             2

#if LOADER_UART
        #define CONFIG_RETARGET
        #if LOADER_UART == 2
                #define CONFIG_RETARGET_UART HW_UART2
        #elif LOADER_UART == 1
                #define CONFIG_RETARGET_UART HW_UART1
        #else
                #error "Invalid LOADER_UART configuration!"
        #endif
#endif


#define dg_configUSE_LP_CLK                     LP_CLK_32768
#define dg_configEXEC_MODE                      MODE_IS_CACHED
#define dg_configCODE_LOCATION                  NON_VOLATILE_IS_FLASH
#define dg_configEXT_CRYSTAL_FREQ               EXT_CRYSTAL_IS_16M

#define dg_configIMAGE_SETUP                    DEVELOPMENT_MODE
#define dg_configEMULATE_OTP_COPY               (0)

#define dg_configUSER_CAN_USE_TIMER1            (0)

/*
 * Controls the retRAM size used by the project.
 * 0: all RAM is retained
 * 1: retention memory size is optimal
 */
#define proj_configOPTIMAL_RETRAM                (0)

#if !defined(RELEASE_BUILD) && (proj_configOPTIMAL_RETRAM == 1)
        /* WARNING: retRAM optimizations are disabled in DEBUG builds! */
        #undef proj_configOPTIMAL_RETRAM
        #define proj_configOPTIMAL_RETRAM       (0)
#elif (dg_configEXEC_MODE != MODE_IS_CACHED)
        /* WARNING: retRAM optimizations are not applicable in MIRRORED mode! */
        #undef proj_configOPTIMAL_RETRAM
        #define proj_configOPTIMAL_RETRAM       (0)
#endif

#if (proj_configOPTIMAL_RETRAM == 0)
        #define dg_configMEM_RETENTION_MODE     (0x1F)
        #define dg_configSHUFFLING_MODE         (0x3)
#else
        // Only for dg_configEXEC_MODE == MODE_IS_CACHED!
                #define dg_configMEM_RETENTION_MODE             (0x14)
                #define dg_configSHUFFLING_MODE                 (0x2)
#endif

#define dg_configUSE_WDOG                       (1)

#define dg_configUSE_DCDC                       (1)

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
#define configTOTAL_HEAP_SIZE                    14336  /* This is the FreeRTOS Total Heap Size */

/*************************************************************************************************\
 * Peripheral specific config
 */
#define dg_configUSE_HW_TRNG                    1
#define dg_configFLASH_ADAPTER                  1
#define dg_configNVMS_ADAPTER                   1
#define dg_configNVMS_VES                       1

/*************************************************************************************************\
 * BLE specific config
 */
#define BLE_MAX_MISSES_ALLOWED                  (3)
#define BLE_MAX_DELAYS_ALLOWED                  (3)

#define dg_configDEBUG_TRACE                    0

/*
 * SUOTA loader configuration:
 * - To enable SUOTA over GATT only, set SUOTA_VERSION to any version >= SUOTA_VERSION_1_1
 *      and leave SUOTA_PSM undefined.
 * - To enable SUOTA over GATT and L2CAP CoC, set SUOTA_VERSION to any version >= SUOTA_VERSION_1_2
 *      and also define SUOTA_PSM to match the desired PSM. In this case the central device
 *      can use either of both according to its preference.
 */
#define SUOTA_VERSION                           SUOTA_VERSION_1_3
#define SUOTA_PSM                               0x81

/*
 * When enabled, a special button is detected as having been pressed or not during boot.
 * If the button is pressed, SUOTA service will be started without booting any flashed
 * application. This allows the user to force SUOTA service in certain circumstances.
 */
#define CFG_FORCE_SUOTA_GPIO                    (0)

#define USE_PARTITION_TABLE_1MB_WITH_SUOTA

/*************************************************************************************************\
 * BLE device config
 */
#define dg_configBLE_CENTRAL                    (0)
#define dg_configBLE_GATT_CLIENT                (0)
#define dg_configBLE_OBSERVER                   (0)
#define dg_configBLE_BROADCASTER                (0)
#ifndef SUOTA_PSM
#define dg_configBLE_L2CAP_COC                  (0)
#endif



/* Include bsp default values */
#include "bsp_defaults.h"

/*************************************************************************************************\
 * Memory layout configuration
 */
#if (dg_configCODE_LOCATION == NON_VOLATILE_IS_OTP)
        // CODE_SIZE cannot be more than 58K
        #define CODE_SIZE     ( 58 * 1024)

        #if (dg_configEXEC_MODE == MODE_IS_CACHED)
                        /* DA14681-01
                         * RAM goes first, RetRAM0 follows. RetRAM1 is added at the beginning when
                         * optimized RetRAM configuration is used (so that the IVT is preserved).
                         * RAM size should be defined such that it covers the whole empty space
                         * between RetRAM1, if it exists, and RetRAM0.
                         */
                        #define RETRAM_FIRST    0

                        #define RAM_SIZE        ( 64 * 1024)

                        #if (proj_configOPTIMAL_RETRAM == 0)
                                #define RETRAM_0_SIZE   ( 64 * 1024)
                                #define RETRAM_1_SIZE   (  0 * 1024)
                        #else
                                #define RETRAM_0_SIZE   ( 32 * 1024)
                                #define RETRAM_1_SIZE   ( 32 * 1024)
                        #endif
        #else // MIRRORED
                        /* DA14681-01
                         * CODE is first, RetRAM follows. RAM is last, always 16K.
                         *
                         * RetRAM uses all RAM5 block. RAM uses CACHE.
                         */
                        #define RETRAM_FIRST    1

                        #define RAM_SIZE        ( 16 * 1024)
                        #define RETRAM_0_SIZE   (128 * 1024 - CODE_SIZE)
                        #define RETRAM_1_SIZE   (  0 * 1024)
        #endif

        #if (CODE_SIZE > (58 * 1024))
                #error "maximum CODE size when OTP is used is 58K!"
        #endif

#elif (dg_configCODE_LOCATION == NON_VOLATILE_IS_FLASH)
        #define CODE_SIZE     (128 * 1024)

        #if (dg_configEXEC_MODE == MODE_IS_CACHED)
                        /* DA14681-01
                         * RAM goes first, RetRAM0 follows. RetRAM1 is added at the beginning when
                         * optimized RetRAM configuration is used (so that the IVT is preserved).
                         */
                        #define RETRAM_FIRST    0

                        #define RAM_SIZE        ( 64 * 1024)

                        #if (proj_configOPTIMAL_RETRAM == 0)
                                #define RETRAM_0_SIZE   ( 64 * 1024)
                                #define RETRAM_1_SIZE   (  0 * 1024)
                        #else
                                #define RETRAM_0_SIZE   ( 32 * 1024)
                                #define RETRAM_1_SIZE   ( 32 * 1024)
                        #endif
        #else // MIRRORED
                #error "QSPI mirrored mode is not supported!"
        #endif

#elif (dg_configCODE_LOCATION == NON_VOLATILE_IS_NONE)
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

#endif /* CUSTOM_CONFIG_QSPI_H_ */

/**
\}
\}
\}
*/
