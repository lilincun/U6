/**
 * \addtogroup BSP
 * \{
 * \addtogroup DEVICES
 * \{
 * \addtogroup FEM
 * \{
 * \brief Front End Module for SKYWORKS SKY66112-11
 */

/**
 *****************************************************************************************
 *
 * @file
 *
 * @brief FEM Driver for SKYWORKS SKY66112-11 Low Level Driver API.
 *
 * GPIOs used for controlling the FEM are configured using the following macros, that should be placed
 * in custom_config.h:
 *
 *      CSD   : dg_configFEM_SKY66112_11_CSD_PORT/PIN
 *      CPS   : dg_configFEM_SKY66112_11_CPS_PORT/PIN
 *      CRX   : dg_configFEM_SKY66112_11_CRX_PORT/PIN
 *      CTX   : dg_configFEM_SKY66112_11_CTX_PORT/PIN
 *      CHL   : dg_configFEM_SKY66112_11_CHL_PORT/PIN
 *      ANTSEL: dg_configFEM_SKY66112_11_ANTSEL_PORT/PIN
 *
 * In order to control an external PA, three GPIOs can be used. These are called RF_ANT_TRIMx, x=0,1,2. Each one of
 * these bits is enabled if the corresponding PORT/PIN macros are set:
 *
 *      RF_ANT_TRIM0 : dg_configFEM_SKY66112_11_ANT_TRIM_0_PORT/PIN
 *      RF_ANT_TRIM1 : dg_configFEM_SKY66112_11_ANT_TRIM_1_PORT/PIN
 *      RF_ANT_TRIM2 : dg_configFEM_SKY66112_11_ANT_TRIM_2_PORT/PIN
 *
 * The actual GPIO values for RF_ANT_TRIMx are set by the corresponding MAC, whenever it gains access by the arbiter
 * to the RF. This driver only handles the GPIO initialization.
 *
 * FEM BIAS Voltage control is enabled by the following macros:
 *
 *      V18            : dg_configFEM_SKY66112_11_FEM_BIAS_V18
 *      V18P           : dg_configFEM_SKY66112_11_FEM_BIAS_V18P
 *
 * If none of them is set, FEM BIAS will not be controlled by this driver.
 *
 * Copyright (C) 2015. Dialog Semiconductor, unpublished work. This computer
 * program includes Confidential, Proprietary Information and is a Trade Secret of
 * Dialog Semiconductor. All use, disclosure, and/or reproduction is prohibited
 * unless authorized in writing. All Rights Reserved.
 *
 * <black.orca.support@diasemi.com> and contributors.
 *
 *****************************************************************************************
 */

#ifndef HW_FEM_H_
#define HW_FEM_H_

#if dg_configFEM == FEM_SKY66112_11
#include <stdbool.h>

#if defined(dg_configFEM_SKY66112_11_FEM_BIAS_V18) && defined(dg_configFEM_SKY66112_11_FEM_BIAS_V18P)
#error Only one of dg_configFEM_SKY66112_11_FEM_BIAS_V18 and dg_configFEM_SKY66112_11_FEM_BIAS_V18P can be set at a time
#endif

/* Configuration/state structure (actually just a byte) */
typedef struct __attribute__ ((__packed__)) {
        uint8_t tx_power: 1;
        uint8_t tx_bypass: 1;
        uint8_t rx_bypass: 1;
        uint8_t antsel: 1;
        uint8_t started: 1;
} hw_fem_config;

/**
 * \brief Configures FEM TX Power
 *
 * \param [in] high Set true to enable high TX power
 *
 */
void hw_fem_set_txpower(bool high);

/**
 * \brief Configures FEM TX bypass mode
 *
 * \param [in] enable false: Use PA, true: bypass
 *
 */
void hw_fem_set_tx_bypass(bool enable);

/**
 * \brief Configures FEM RX bypass mode
 *
 * \param [in] enable false: Use LNA, true: bypass
 *
 */
void hw_fem_set_rx_bypass(bool enable);

/**
 * \brief Gets the TX Power setting
 *
 * \return true, if TX Power is high, false if low
 *
 */
bool hw_fem_get_txpower(void);

/**
 * \brief Gets the TX bypass mode
 *
 * \return false: No TX bypass, true: TX bypass
 *
 */
bool hw_fem_get_tx_bypass(void);

/**
 * \brief Gets the RX bypass mode
 *
 * \return false: No RX bypass, true: RX bypass
 *
 */
bool hw_fem_get_rx_bypass(void);



/**
 * \brief Configures FEM Antenna to use
 *
 * \param [in] one false: antenna 0, true: antenna 1
 *
 */
void hw_fem_set_antenna(bool one);

/**
 * \brief Gets the selected antenna
 *
 * \return false: antenna 1, true: antenna 2
 *
 */
bool hw_fem_get_antenna(void);

/**
 * \brief Sets the FEM Bias
 *
 * \param [in] voltage The voltage to set the FEM Bias to (mV)
 *
 * \return 0: success, -1: Out of range value entered, -2: FEM BIAS not supported on this board
 *
 */
int hw_fem_set_bias(uint16_t voltage);

/**
 * \brief Sets the 2nd FEM Bias
 *
 * \param [in] voltage The voltage to set the 2nd FEM Bias to (mV)
 *
 * \return 0: success, -1: Out of range value entered, -2: 2nd FEM BIAS not supported on this board
 *
 */
int hw_fem_set_bias2(uint16_t voltage);

/**
 * \brief Starts and configures FEM.
 *
 * Configures and sets GPIOs according to configuration set by hw_fem_set_config(). Also configures
 * DCF Timers.
 *
 *
 * \note To be called by the RF driver when RF is powered on
 *
 */
void hw_fem_start(void);

/**
 * \brief Stops FEM.
 *
 * Stops DCF timers and sets all FEM control signals to 0 to achieve the lowest possible power
 *
 * \note To be called by the RF driver when RF is powered off
 *
 */
void hw_fem_stop(void);

#endif /* dg_configFEM == FEM_SKY66112_11 */
#endif /* HW_FEM_H_ */

/**
 * \}
 * \}
 * \}
 */
