/**
 * \addtogroup BSP
 * \{
 * \addtogroup SYSTEM
 * \{
 * \addtogroup INTERRUPTS
 * 
 * \brief Interrupt priority configuration
 *
 * \{
 */

/**
 *****************************************************************************************
 *
 * @file interrupts.h
 *
 * @brief Interrupt priority configuration
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

#ifndef INTERRUPTS_H_
#define INTERRUPTS_H_

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief Setup interrupt priorities
 *
 * When CPU is reset all interrupts have some priority setup.
 * Reset     -3
 * NMI       -2
 * HardFault -1
 * All other interrupts have configurable priority that is set to 0.
 * If some interrupts should have priority other then default, this function should be called.
 * Argument \p prios can specify only those interrupts that need to have value other than default.
 * For memory efficiency table with priorities for each interrupt consist of interrupt priority
 * tag PRIORITY_x followed by interrupts that should have this priority, interrupts names are from
 * enum IRQn_Type.
 *
 * \note If interrupt priorities do not need to be changed dynamically at runtime, best way to
 * specify static configuration is to create table named __dialog_interrupt_priorities that will
 * be used automatically at startup.
 *
 * Most convenient way to prepare such table is to use macros like in example below:
 *
 * \code{.c}
 * INTERRUPT_PRIORITY_CONFIG_START(__dialog_interrupt_priorities)
 *      PRIORITY_0, // Start interrupts with priority 0 (highest)
 *               SVCall_IRQn,
 *               PendSV_IRQn,
 *               SysTick_IRQn,
 *      PRIORITY_1, // Start interrupts with priority 1
 *               BLE_WAKEUP_LP_IRQn,
 *               BLE_GEN_IRQn,
 *               FTDF_WAKEUP_IRQn,
 *               FTDF_GEN_IRQn,
 *      PRIORITY_2,
 *               SRC_IN_IRQn,
 *               SRC_OUT_IRQn,
 *      PRIORITY_3,
 *               UART_IRQn,
 *               UART2_IRQn,
 * INTERRUPT_PRIORITY_CONFIG_END
 * \endcode
 *
 * Table __dialog_interrupt_priorities can now be used to call this function.
 * Table can specify all interrupts or only those that need to be changed.
 *
 * \param [in] prios table with interrupts and priorities to setup
 *
 */
void set_interrupt_priorities(const int8_t prios[]);

/**
 * \brief Check whether running in interrupt context.
 *
 * \return true if the CPU is serving an interrupt.
 */
static inline bool in_interrupt(void)
{
        return (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0;
}


/**
 * \brief Default interrupt priorities table.
 *
 */
extern const int8_t __dialog_interrupt_priorities[];


        #define LAST_IRQn RESERVED31_IRQn

/*
 * Following macros allow easy way to build table with interrupt priorities.
 * See example in set_interrupt_priorities function description.
 */
#define INTERRUPT_PRIORITY_CONFIG_START(name) const int8_t name[] = {
#define PRIORITY_0      (LAST_IRQn + 1)
#define PRIORITY_1      (LAST_IRQn + 2)
#define PRIORITY_2      (LAST_IRQn + 3)
#define PRIORITY_3      (LAST_IRQn + 4)
#define PRIORITY_TABLE_END (LAST_IRQn + 5)
#define INTERRUPT_PRIORITY_CONFIG_END PRIORITY_TABLE_END };

#ifdef __cplusplus
extern }
#endif

#endif /* INTERRUPTS_H_ */

/**
 * \}
 * \}
 * \}
 */
