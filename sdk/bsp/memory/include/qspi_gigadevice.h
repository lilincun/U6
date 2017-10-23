/**
 * \addtogroup BSP
 * \{
 * \addtogroup SYSTEM
 * \{
 * \addtogroup MEMORY
 *
 * \{
 */

/**
 ****************************************************************************************
 *
 * @file qspi_gigadevice.h
 *
 * @brief QSPI flash driver for Gigadevice flashes - common code
 *
 * Copyright (C) 2016. Dialog Semiconductor, unpublished work. This computer
 * program includes Confidential, Proprietary Information and is a Trade Secret of
 * Dialog Semiconductor.  All use, disclosure, and/or reproduction is prohibited
 * unless authorized in writing. All Rights Reserved.
 *
 * <black.orca.support@diasemi.com> and contributors.
 *
 ****************************************************************************************
 */
#ifndef _QSPI_GIGADEVICE_H_
#define _QSPI_GIGADEVICE_H_

#define GIGADEVICE_ID    0xC8

#if (FLASH_AUTODETECT == 1) || (dg_configFLASH_MANUFACTURER_ID == GIGADEVICE_ID)

#include "qspi_common.h"

#define GIGADEVICE_PERFORMANCE_MODE       1       // bootrom does not support Macronix performance mode

#define GD_ERASE_PROGRAM_SUSPEND        0x75
#define GD_ERASE_PROGRAM_RESUME         0x7A
#define GD_READ_STATUS_REGISTER_1       0x35
#define GD_WRITE_STATUS_REGISTER        0x01

#define GD_STATUS_QE_BIT                9 // Quad Enable
#define GD_STATUS_QE_MASK               (1 << GD_STATUS_QE_BIT)

/* Suspend */
#define GD_STATUS_SUS1_BIT             7 // Erase suspend bit
#define GD_STATUS_SUS1_MASK            (1 << GD_STATUS_SUS1_BIT)
#define GD_STATUS_SUS2_BIT             2 // Program suspend bit
#define GD_STATUS_SUS2_MASK            (1 << GD_STATUS_SUS2_BIT)

QSPI_SECTION __attribute__((unused)) static inline uint8_t flash_gd_read_status_register_1(void)
{
        __DBG_QSPI_VOLATILE__ uint8_t status;
        uint8_t cmd[] = { GD_READ_STATUS_REGISTER_1 };

        qspi_transact(cmd, 1, &status, 1);

        return status;
}

QSPI_SECTION __attribute__((unused)) static inline void flash_gd_write_status_register(uint16_t value)
{
        uint8_t cmd[] = { GD_WRITE_STATUS_REGISTER, value, value >> 8 };

        qspi_write(cmd, 3);

        /* Wait for the Flash to process the command */
        while (flash_is_busy());
}

QSPI_SECTION static inline void flash_gd_enable_quad_mode(void)
{
        uint16_t status;

        status = flash_read_status_register();
        status |= flash_gd_read_status_register_1() << 8;
        if (!(status & GD_STATUS_QE_MASK)) {
                flash_write_enable();
                flash_gd_write_status_register(status | GD_STATUS_QE_MASK);
        }
}

QSPI_SECTION static bool flash_gd_is_suspended(void)
{
        __DBG_QSPI_VOLATILE__ uint8_t status;

        status = flash_gd_read_status_register_1();
        return (status & (GD_STATUS_SUS1_MASK | GD_STATUS_SUS2_MASK)) != 0;
}

QSPI_SECTION static void flash_gd_deactivate_command_entry_mode(void)
{

}
#endif

#endif /* _QSPI_GIGADEVICE_H_ */
/**
 * \}
 * \}
 * \}
 */
