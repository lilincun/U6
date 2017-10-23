/**
 \addtogroup INTERFACES
 \{
 \addtogroup BLE
 \{
 \addtogroup IRB
 \{
 */

/**
 ****************************************************************************************
 * @file ble_irb_helper.h
 *
 * @brief IRB creation and handling
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

#ifndef BLE_IRB_H_
#define BLE_IRB_H_

#include <stdbool.h>
#include "osal.h"
#include "ble_mgr_irb.h"

/**
 * \brief Allocates new BLE message to use in IRB
 *
 * \param [in] op_code  message op code
 * \param [in] size     message size
 *
 * \return allocated message pointer
 *
 */
void *alloc_ble_msg(uint16_t op_code, uint16_t size);

/**
 * \brief Initialize IRB with new message
 *
 * Existing contents of IRB is overwritten.
 * IRB is set to pending state and new message is allocated and assigned.
 *
 * \param [in,out]  irb      IRB structure
 * \param [in]      op_code  message op code
 * \param [in]      size     message size
 *
 * \return allocated message pointer
 *
 */
void *irb_ble_init_msg(OS_IRB *irb, uint16_t op_code, uint16_t size);

/**
 * \brief Replace message assigned to IRB
 *
 * Existing message, if any, is freed and new one is allocated.
 *
 * \param [in,out]  irb      IRB structure
 * \param [in]      op_code  message op code
 * \param [in]      size     message size
 *
 * \return allocated message pointer
 *
 */
void *irb_ble_replace_msg(OS_IRB *irb, uint16_t op_code, uint16_t size);

/**
 * \brief Initialize IRB with new event
 *
 * \param [in,out]  irb      IRB structure
 * \param [in]      evt_code event code
 * \param [in]      size     event size
 *
 * \return allocated event buffer pointer
 */
void *irb_ble_init_evt(OS_IRB *irb, uint16_t evt_code, uint16_t size);

/**
 * \brief Mark IRB as completed
 *
 * \param [in,out]  irb      IRB structure
 * \param [in]      free_msg true if message assigned to IRB should be freed, false otherwise
 *
 */
void irb_ble_mark_completed(OS_IRB *irb, bool free_msg);

/**
 * \brief Mark IRB as error
 *
 * \param [in,out]  irb      IRB structure
 * \param [in]      free_msg true if message assigned to IRB should be freed, false otherwise
 *
 */
void irb_ble_mark_error(OS_IRB *irb, bool free_msg);

/**
 * \brief Free message associated with IRB
 *
 * \param [in]      irb      IRB structure
 *
 */
void irb_ble_free_msg(OS_IRB *irb);

/**
 * \brief Execute IRB
 *
 * This creates IRB with given message, calls manager's \p handler (if #BLE_MGR_DIRECT_ACCESS is
 * defined to 1) or sends it to manager's command queue (if #BLE_MGR_DIRECT_ACCESS is defined to 0)
 * and waits for response.
 * Buffer pointed by \p cmd is owned by manager after calling this function and should not be
 * accessed.
 * Buffer returned in \p rsp is owned by caller and should be freed there.
 *
 * \param [in]  cmd      command buffer
 * \param [out] rsp      response buffer
 * \param [in]  handler  command handler
  *
 * \return true if executed successfully, false otherwise
 *
 */
bool irb_execute(void *cmd, void **rsp, irb_ble_handler_t handler);

#endif /* BLE_IRB_H_ */
/**
 \}
 \}
 \}
 */
