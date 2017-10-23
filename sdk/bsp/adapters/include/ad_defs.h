/**
 * \addtogroup BSP
 * \{
 * \addtogroup ADAPTERS
 * \{
 * \addtogroup ADAPTER_DEFINITIONS
 *
 * \brief Doxygen documentation is not yet available for this module.
 *        Please check the source code file(s)
 *
 * \{
 */

/**
 ****************************************************************************************
 * @file ad_defs.h
 *
 * @brief Common definitions for adapters
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

#ifndef AD_DEFS_H_
#define AD_DEFS_H_

typedef enum {
        IRB_PENDING   = 0,
        IRB_COMPLETED = 1,
        IRB_ERROR     = 2,
} IRB_Status_t;

typedef enum IRB_Class_ID {
        IRB_BLE  = 0,
        IRB_FTDF = 1,
        IRB_I2C  = 2,
        IRB_SPI  = 3,
        IRB_UART = 4,
} IRB_Class_ID_t;

typedef void * IRB_Pointer_t;

typedef struct IRB {
        IRB_Status_t   status;
        IRB_Class_ID_t class_id;
        IRB_Pointer_t  ptr_buf;
} IRB_t;

#endif /* AD_DEFS_H_ */

/**
 * \}
 * \}
 * \}
 */

