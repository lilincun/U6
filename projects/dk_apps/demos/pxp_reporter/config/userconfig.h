/**
 ****************************************************************************************
 *
 * @file userconfig.h
 *
 * @brief User peripherals demos configuration header
 *
 * Copyright (C) 2016. Dialog Semiconductor Ltd, unpublished work. This computer
 * program includes Confidential, Proprietary Information and is a Trade Secret of
 * Dialog Semiconductor Ltd. All use, disclosure, and/or reproduction is prohibited
 * unless authorized in writing. All Rights Reserved.
 *
 * <black.orca.support@diasemi.com> and contributors.
 *
 ****************************************************************************************
 */

/*
 * IMPORTANT!
 * this is sample configuration for peripherals demo application
 * for changing configuration, it's recommended to copy this file to "config/userconfig.h" and make
 * changes there - it will be used instead of this file.
 */

/* enable to use "\r\n" as newline sequence, otherwise "\n" is used */
#define CFG_UART_USE_CRLF       (1)

/* define CFG_UART_USE_CRLF when using terminal which does not handle \n alone properly */
#if CFG_UART_USE_CRLF
#define NEWLINE        "\r\n"
#define NEWLINE_SIZE   (2)
#else
#define NEWLINE        "\n"
#define NEWLINE_SIZE   (1)
#endif
/*
 * below you can enable and disable demos for peripherals (as in symbol name)
 * note1: not all combinations are supported due to memory and GPIO constraints
 * note2: GPIO configuration can be adjusted using config/gpio_setup.h
 */
/**
 * \brief Enable Timer0 demo
 *
 * External hardware is not needed. The D2 LED located on the ProDK board is used to show how
 * Timer0 can be used by a user.
 *
 */
//#define CFG_DEMO_HW_TIMER0      (1)
#define CFG_DEMO_HW_TIMER0      (0) //skn wu
/**
 * \brief Enable Timer1 demo
 *
 * External hardware is required when the default configuration is used. The user has to connect
 * an external LED to pin P1.6 to show how Timer1 can be used.
 *
 * The D2 LED can be used for this purpose but in this case Timer0 has to be disabled and pin
 * P1.6 has to be changed to pin P1.5 in a config/default/gpio_setup.h file in Timer1 section.
 *
 */
//#define CFG_DEMO_HW_TIMER1      (1)
#define CFG_DEMO_HW_TIMER1      (0) //skn wu
/**
 * \brief Enable Timer2 demo
 *
 * External hardware is required when the default configuration is used. The user has to connect
 * an external RGB LED to the P3.5, P3.6 and P3.7 pins to show how Timer2 can be used.
 *
 * To check if PWM signals are generated on P3.5, P3.6 and P3.7 pins in proper way a user may use
 * an oscilloscope or a signal analyzer connecting to the pins and check how signals look like.
 *
 */
//#define CFG_DEMO_HW_TIMER2      (1)
#define CFG_DEMO_HW_TIMER2      (0)//skn wu
/**
 * \brief Enable Wake-up demo
 *
 * External hardware is not needed. A simple pin wire is needed to connect/disconnect wake-up
 * pins (P3.0, P3.1 or P3.2) to/from GND for generating an appropriate slope to trigger wake-up
 * action.
 *
 * \note The Wake-up demo conflicts with the power mode demo. At most one of them must be enabled.
 *
 */
//#define CFG_DEMO_HW_WKUP        (1) //skn wu
#define CFG_DEMO_HW_WKUP        (0)
/**
 * \brief Enable Breath timer demo
 *
 * External hardware is not needed. The D1 LED located on the ProDK board is used to
 * show how Breath timer can be used by a user.
 *
 */
#define CFG_DEMO_HW_BREATH      (1)
/**
 * \brief Enable Infrared (IR) generator demo
 *
 * External hardware is required when the default configuration is used. The user has to connect
 * an external IR LED to pin P4.5 to send some data to an external IR receiver using an infrared
 * communication.
 *
 * A sensor board is equipped with IR LED so it can be used for this demo.
 *
 */
#define CFG_DEMO_HW_IRGEN       (0)
/**
 * \brief Enable I2C interface demo
 *
 * External hardware is required when the default configuration is used. The user has to connect
 * an external EEPROM memory (24LC256) and Temperature sensor (FM75) to I2C pins (P1.2 and P1.4 by
 * default). With this configuration HW_GPADC and AD_GPADC demos have to be disabled to avoid pins'
 * conflicts.
 *
 */
#define CFG_DEMO_HW_I2C         (0)
//#define CFG_DEMO_HW_I2C         (1)//skn wu  hw i2c1
/**
 * \brief Enable I2C interface demo in asynchronous mode
 *
 * It has the same requirements as HW_I2C demo.
 *
 */
#define CFG_DEMO_HW_I2C_ASYNC   (0)
/**
 * \brief Enable General Purpose ADC demo
 *
 * External hardware is needed only if a user wants to measure voltage of external devices.
 * Then these external devices have to be connected to ADC pins (P0.7, P1.2 or P1.4).
 *
 */
//#define CFG_DEMO_HW_GPADC       (1)
#define CFG_DEMO_HW_GPADC       (0) //skn wu
/**
 * \brief Enable Quadrature decoder demo
 *
 * External hardware is required when the default configuration is used. The user has to connect
 * an external HID device to P4.1, P4.2, P4.3, P4.4 and P4.6, P4.7 pins. Each direction change of
 * the device will be captured and shown in a serial terminal.
 *
 * The UART demo has to be disabled when this demo is used.
 *
 */
#define CFG_DEMO_HW_QUAD        (0)
/**
 * \brief Enable Quad SPI (QSPI) demo
 *
 * External hardware is not needed. The FLASH memory is located on Daughter board.
 *
 */
#define CFG_DEMO_HW_QSPI        (1)
/**
 * \brief Enable UART demo
 *
 * External hardware is required when the default configuration is used. The user has to connect
 * an external USB <-> serial converter device to P4.1 (Tx) and P4.2 (Rx) pins to have access to
 * the second UART interface.
 *
 */
#define CFG_DEMO_AD_UART        (1)
/**
 * \brief Enable power mode demo
 *
 * External hardware is not needed. A simple pin wire is needed to connect pin P1.7 to GND.
 * Disconnecting it from ground will wake the chip from sleep mode and the event will be reported
 * to the serial terminal.
 *
 * \note The power mode demo conflicts with the Wake-up demo. At most one of them must be enabled.
 *
 */
#define CFG_DEMO_POWER_MODE     (0)
/**
 * \brief Enable General ADC demo using the adapter solution
 *
 * Measuring the temperature value doesn't need an external device because a temperature sensor is
 * embedded into the chip. The same holds for the battery level. Other devices like a light sensor
 * or an encoder device have to be connected externally to P1.2 or/and P1.4 pins.
 *
 */
//#define CFG_DEMO_AD_GPADC       (1)
#define CFG_DEMO_AD_GPADC       (0)
/**
 * \brief Enable Temperature sensor demo
 *
 * External hardware is not needed. The temperature values are read from the temperature sensor
 * which is embedded into the chip.
 *
 */
#define CFG_DEMO_AD_TEMPSENS    (1)
/**
 * \brief Enable SPI interface demo using the adapter solution
 *
 * External hardware is required to run the demo properly. The user has to connect an external
 * FLASH memory (AT45DB011D) to pins defined in a gpio_setup.h file in a SPI section.
 *
 */
#define CFG_DEMO_AD_SPI         (0)
/**
 * \brief Enable SPI and I2C interfaces demo using their adapters
 *
 * External hardware is required to run the demo properly. The user has to connect an external
 * FLASH memory (AT45DB011D) to pins defined in a gpio_setup.h file in a SPI section and an external
 * EEPROM memory (24xx256) to pins defined in a gpio_setup.h file in a I2C section.
 *
 */
#define CFG_DEMO_AD_SPI_I2C     (0)

/*
 * Below you can enable or disable devices demos for demo_sensors.
 * They require a sensor board.
 *
 * HW_GPADC, AD_GPADC and HW_TIMER2 demos should be disabled to use sensors demos which use I2C
 * interface for communication with the motherboard.
 */

#define CFG_DEMO_SENSOR_BH1750  (0)
#define CFG_DEMO_SENSOR_BME280  (0)
#define CFG_DEMO_SENSOR_ADXL362 (0)
#define CFG_DEMO_SENSOR_BMM150  (0)
#define CFG_DEMO_SENSOR_BMG160  (0)

//OLED1316  oled
//#define CFG_DEMO_SENSOR_OLED1316  (0) //SKN  WU

//KX022 TASK
//#define CFG_DEMO_SENSOR_KX022  (0) //SKN  WU

//MC3630 TASK
#define CFG_DEMO_SENSOR_MC3630  (1)//YOKO

//pps960 TASK
#define CFG_DEMO_SENSOR_PPS960  (1) //SKN  WU
#define CFG_SENSOR_PPS960_LIB  (1) //SKN  WU

//BLE  TASK
#define CFG_DEMO_PXP_BLE          (1) //SKN  WU


#if CFG_DEMO_SENSOR_BH1750 || CFG_DEMO_SENSOR_BME280 || CFG_DEMO_SENSOR_ADXL362 || \
        CFG_DEMO_SENSOR_BMM150 || CFG_DEMO_SENSOR_BMG160
#define CFG_DEMO_SENSOR_BOARD   (1)
#else
#define CFG_DEMO_SENSOR_BOARD   (0)
#endif

//控制 periph_setup.c 中 hw pin I2C_1的 配置
#if CFG_DEMO_SENSOR_KX022 || CFG_DEMO_SENSOR_PPS960 || CFG_DEMO_SENSOR_BOARD || CFG_DEMO_HW_I2C_ASYNC || CFG_DEMO_AD_SPI_I2C
#define CFG_AD_I2C_1            (1)
#endif

//控制 periph_setup.c 中 hw pin I2C_2  的 配置
#if CFG_DEMO_SENSOR_MC3630
#define CFG_AD_I2C_2            (1)
#endif

#if CFG_DEMO_SENSOR_BOARD || CFG_DEMO_AD_SPI || CFG_DEMO_AD_SPI_I2C
#define CFG_AD_SPI_1            (1)
#endif

#if CFG_DEMO_HW_WKUP && CFG_DEMO_POWER_MODE
# error "The Wake-up demo and the power mode demo cannot both be enabled at the same time!"
#endif
