/**
 ****************************************************************************************
 *
 * @file gpio_setup.h
 *
 * @brief GPIO pin assignments for demo
 *
 * Copyright (C) 2015. Dialog Semiconductor Ltd, unpublished work. This computer
 * program includes Confidential, Proprietary Information and is a Trade Secret of
 * Dialog Semiconductor Ltd.  All use, disclosure, and/or reproduction is prohibited
 * unless authorized in writing. All Rights Reserved.
 *
 * <black.orca.support@diasemi.com> and contributors.
 *
 ****************************************************************************************
 */

/*
 * IMPORTANT!
 * this is sample configuration for peripherals demo application GPIO assignments
 * for changing configuration, it's recommended to copy this file to "config/gpio_setup.h" and make
 * changes there - it will be used instead of this file.
 */
#include "hw_gpio.h"
#include "platform_devices.h"

#ifndef DEMOBOARD
//#define DEMOBOARD
#endif

/* Breath timer */
#ifdef DEMOBOARD
#define CFG_GPIO_BREATH_PORT        (HW_GPIO_PORT_4)
#define CFG_GPIO_BREATH_PIN         (HW_GPIO_PIN_0)
#else
#define CFG_GPIO_BREATH_PORT        (HW_GPIO_PORT_3)
#define CFG_GPIO_BREATH_PIN         (HW_GPIO_PIN_5)
#endif

/* IR generator */
#define CFG_GPIO_IR_PORT                (HW_GPIO_PORT_4)
#define CFG_GPIO_IR_PIN                 (HW_GPIO_PIN_5)

/* Quadrature decoder */
#define CFG_GPIO_QUAD_XA_PORT           (HW_GPIO_PORT_4)
#define CFG_GPIO_QUAD_XA_PIN            (HW_GPIO_PIN_4)
#define CFG_GPIO_QUAD_XB_PORT           (HW_GPIO_PORT_4)
#define CFG_GPIO_QUAD_XB_PIN            (HW_GPIO_PIN_1)
#define CFG_GPIO_QUAD_YA_PORT           (HW_GPIO_PORT_4)
#define CFG_GPIO_QUAD_YA_PIN            (HW_GPIO_PIN_6)
#define CFG_GPIO_QUAD_YB_PORT           (HW_GPIO_PORT_4)
#define CFG_GPIO_QUAD_YB_PIN            (HW_GPIO_PIN_7)
#define CFG_GPIO_QUAD_ZA_PORT           (HW_GPIO_PORT_4)
#define CFG_GPIO_QUAD_ZA_PIN            (HW_GPIO_PIN_2)
#define CFG_GPIO_QUAD_ZB_PORT           (HW_GPIO_PORT_4)
#define CFG_GPIO_QUAD_ZB_PIN            (HW_GPIO_PIN_3)

/* Timer 0 */
#if   dg_configBLACK_ORCA_MB_REV == BLACK_ORCA_MB_REV_D

#ifdef DEMOBOARD
#       define CFG_GPIO_TIMER0_PORT     (HW_GPIO_PORT_1)
#       define CFG_GPIO_TIMER0_PIN      (HW_GPIO_PIN_5)
#else
#       define CFG_GPIO_TIMER0_PORT     (HW_GPIO_PORT_3)
#       define CFG_GPIO_TIMER0_PIN      (HW_GPIO_PIN_1)
#endif

#else
#       error "Unknown board!"
#endif

/* Timer 1 */
#if   dg_configBLACK_ORCA_MB_REV == BLACK_ORCA_MB_REV_D
#       define CFG_GPIO_TIMER1_PWM_PORT (HW_GPIO_PORT_1)
#       define CFG_GPIO_TIMER1_PWM_PIN  (HW_GPIO_PIN_6)
#else
#       error "Unknown board!"
#endif

/* Timer 2 */
#ifdef DEMOBOARD
#define CFG_GPIO_TIMER2_PWM2_PORT       (HW_GPIO_PORT_3)
#define CFG_GPIO_TIMER2_PWM2_PIN        (HW_GPIO_PIN_5)
#else
#define CFG_GPIO_TIMER2_PWM2_PORT       (HW_GPIO_PORT_1)
#define CFG_GPIO_TIMER2_PWM2_PIN        (HW_GPIO_PIN_7)
#endif

#define CFG_GPIO_TIMER2_PWM3_PORT       (HW_GPIO_PORT_3)
#define CFG_GPIO_TIMER2_PWM3_PIN        (HW_GPIO_PIN_6)
#define CFG_GPIO_TIMER2_PWM4_PORT       (HW_GPIO_PORT_3)
#define CFG_GPIO_TIMER2_PWM4_PIN        (HW_GPIO_PIN_7)

/* UART 2 */
#if dg_configBLACK_ORCA_MB_REV == BLACK_ORCA_MB_REV_D
#       define CFG_GPIO_UART2_TX_PORT   (HW_GPIO_PORT_4)
#       define CFG_GPIO_UART2_TX_PIN    (HW_GPIO_PIN_2)
#       define CFG_GPIO_UART2_RX_PORT   (HW_GPIO_PORT_4)
#       define CFG_GPIO_UART2_RX_PIN    (HW_GPIO_PIN_1)
#else
#       define CFG_GPIO_UART2_TX_PORT   (HW_GPIO_PORT_1)
#       define CFG_GPIO_UART2_TX_PIN    (HW_GPIO_PIN_2)
#       define CFG_GPIO_UART2_RX_PORT   (HW_GPIO_PORT_1)
#       define CFG_GPIO_UART2_RX_PIN    (HW_GPIO_PIN_3)
#endif

/* Wakeup timer */
#ifdef DEMOBOARD
#define CFG_GPIO_WKUP_1_PORT            (HW_GPIO_PORT_3)
#define CFG_GPIO_WKUP_1_PIN             (HW_GPIO_PIN_0)
#define CFG_GPIO_WKUP_2_PORT            (HW_GPIO_PORT_3)
#define CFG_GPIO_WKUP_2_PIN             (HW_GPIO_PIN_1)
#else
#define CFG_GPIO_WKUP_1_PORT            (HW_GPIO_PORT_1)
#define CFG_GPIO_WKUP_1_PIN             (HW_GPIO_PIN_2)
#define CFG_GPIO_WKUP_2_PORT            (HW_GPIO_PORT_1)
#define CFG_GPIO_WKUP_2_PIN             (HW_GPIO_PIN_5)
#endif

#define CFG_GPIO_WKUP_3_PORT            (HW_GPIO_PORT_3)
#define CFG_GPIO_WKUP_3_PIN             (HW_GPIO_PIN_2)

/* I2C */

//skn wu KX022
#if CFG_DEMO_HW_I2C || CFG_AD_I2C_1 //
/* I2C1 */
#ifdef DEMOBOARD
#define CFG_GPIO_I2C1_SCL_PORT          (HW_GPIO_PORT_3)
#define CFG_GPIO_I2C1_SCL_PIN           (HW_GPIO_PIN_0)
#define CFG_GPIO_I2C1_SDA_PORT          (HW_GPIO_PORT_3)
#define CFG_GPIO_I2C1_SDA_PIN           (HW_GPIO_PIN_1)
#else
#define CFG_GPIO_I2C1_SCL_PORT          (HW_GPIO_PORT_1)
#define CFG_GPIO_I2C1_SCL_PIN           (HW_GPIO_PIN_2)
#define CFG_GPIO_I2C1_SDA_PORT          (HW_GPIO_PORT_1)
#define CFG_GPIO_I2C1_SDA_PIN           (HW_GPIO_PIN_5)
#endif

#endif

// mc3630
#if CFG_AD_I2C_2
/* I2C2 */
#define CFG_GPIO_I2C2_SCL_PORT          (HW_GPIO_PORT_3)
#define CFG_GPIO_I2C2_SCL_PIN           (HW_GPIO_PIN_7)
#define CFG_GPIO_I2C2_SDA_PORT          (HW_GPIO_PORT_3)
#define CFG_GPIO_I2C2_SDA_PIN           (HW_GPIO_PIN_6)
#endif

#if CFG_AD_SPI_1
/* SPI1 */
#define CFG_GPIO_SPI1_CLK_PORT          (HW_GPIO_PORT_4)
#define CFG_GPIO_SPI1_CLK_PIN           (HW_GPIO_PIN_1)
#define CFG_GPIO_SPI1_DI_PORT           (HW_GPIO_PORT_4)
#define CFG_GPIO_SPI1_DI_PIN            (HW_GPIO_PIN_0)
#define CFG_GPIO_SPI1_DO_PORT           (HW_GPIO_PORT_3)
#define CFG_GPIO_SPI1_DO_PIN            (HW_GPIO_PIN_7)
#endif

#if CFG_DEMO_SENSOR_ADXL362
/* sensors board has only 1 device which is using SPI bus  */
#define CFG_GPIO_ADXL362_CS_PORT        (HW_GPIO_PORT_3)
#define CFG_GPIO_ADXL362_CS_PIN         (HW_GPIO_PIN_6)
#endif

/* SPI1 */
