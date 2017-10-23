/**
 ****************************************************************************************
 *
 * @file rf_tools_ble.c
 *
 * @brief RF Tools BLE specific code
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
#ifdef CONFIG_USE_BLE
#include <string.h>
#include <stdbool.h>
#include "ble_packers.h"
#include "osal.h"
#include "ble_mgr.h"
#include "ble_mgr_gtl.h"
#include "rf_tools_ble.h"


PRIVILEGED_DATA rf_tools_ble_evt_cbs_t ble_cbs;
PRIVILEGED_DATA xTaskHandle cli_task_handle;
static const ble_mgr_interface_t *ble_mgr_if;

void rf_tools_ble_init(xTaskHandle task_handle, rf_tools_ble_evt_cbs_t cbs)
{
        // Get BLE manager interface handles
        ble_mgr_if = ble_mgr_get_interface();
        ble_mgr_register_application(task_handle);
        cli_task_handle = task_handle;

        ble_cbs = cbs;
}

static void process_response(irb_ble_stack_msg_t *msg_rx)
{
        if (msg_rx->msg_type != HCI_EVT_MSG)
                return;
        if (msg_rx->msg.hci.evt.param_length < 3)
                /* Strange response. Can't extract opcode. ignore */
                return;
        uint16_t opcode = r16le(msg_rx->msg.hci.evt.param + 1);
        uint8_t *bp = (uint8_t *)msg_rx->msg.hci.evt.param + 3;

        switch (opcode) {
        case 0x201E:
                /* Pkt tx */
                ble_cbs.tx(r8le(bp));
                break;
        case 0x201F:
                /* stop test */
                ble_cbs.stop(r8le(bp), r16le(bp + 1));
                break;
        case 0xFC81:
                /* start Pkt rx stats */
                ble_cbs.rx_stats();
                break;
        case 0xFC82:
        {
                /* stop Pkt rx stats */
                uint16_t packets = r16le(bp);
                padvN(bp, 2);
                uint16_t sync = r16le(bp);
                padvN(bp, 2);
                uint16_t crc= r16le(bp);
                padvN(bp, 2);
                uint16_t rssi = r16le(bp);

                ble_cbs.stop_rx_stats(packets, sync, crc, rssi);
        }
        break;
        case 0xFC84:
                /* Start cont tx */
                ble_cbs.start_cont_tx();
                break;
        case 0xFC85:
                /* Stop cont tx */
                ble_cbs.stop_cont_tx();
                break;
        case 0xFC90:
                /* Pkt TX with interval */
                if (msg_rx->msg.hci.evt.event_code == 0xF)
                        ble_cbs.tx_intv(0, 0);
                else {
                        uint8_t status = r8le(bp);
                        ble_cbs.tx_intv(1, status);
                }
                break;

        }
}

void rf_tools_ble_handle_evt(uint32_t ulNotifiedValue)
{
        OS_IRB xIRB;
        if (uxQueueMessagesWaiting(ble_mgr_if->evt_q)) {

                // Look the message without extracting it from the queue
                OS_QUEUE_PEEK(ble_mgr_if->evt_q, &xIRB, 0);

                if (xIRB.status == IRB_COMPLETED) {
                        /* Command received from mgr. Just dequeue */
                        OS_QUEUE_GET(ble_mgr_if->evt_q, &xIRB, 0);
                }
                else if (xIRB.status == IRB_ERROR) {
                        /* Error in previously sent IRB */
                        // Send back to queue
                        xIRB.status = IRB_PENDING;
                        OS_QUEUE_PUT(ble_mgr_if->evt_q, &xIRB, OS_QUEUE_FOREVER);
                }
                else { /* Pending */
                        irb_ble_stack_msg_t *msg_rx = NULL;
                        // Get the message from the queue
                        OS_QUEUE_GET(ble_mgr_if->evt_q, &xIRB, 0);

                        msg_rx = (irb_ble_stack_msg_t *) xIRB.ptr_buf;
                        process_response(msg_rx);

                        // Free allocated IRB message buffer
                        OS_FREE(xIRB.ptr_buf);
                }
        }

        // Check if there are more messages waiting in the BLE manager's event queue
        if (uxQueueMessagesWaiting(ble_mgr_if->evt_q)) {
                OS_TASK_NOTIFY(cli_task_handle, mainBIT_BLE_MGR_EVT,
                        OS_NOTIFY_SET_BITS);
        }




}

/**
* \brief  Start continuous packetized ble tx
*
* \param  [in] freq: frequency channel, 0-39
* \param  [in] payload_lendth: the length of the packet payload to TX (0x0 - 0x25)
* \param  [in] payload_type: 0: PRBS9, 1: 11110000, 2: 10101010, 3: Vendor Specific
*
* \return void
*
*/
void rf_tools_ble_start_cont_pkt_tx(uint8_t freq, uint8_t payload_length,
                uint8_t payload_type)
{
        OS_IRB new_irb;
        irb_ble_stack_msg_t *p_msg_rx = ble_hci_alloc(HCI_CMD_MSG, 3);
        uint8_t *bp = (uint8_t *)&p_msg_rx->msg;

        w16le(bp, 0x201E);
        padv(bp, uint16_t);

        w8le(bp, 0x3);
        padv(bp, uint8_t);

        w8le(bp, freq);
        padv(bp, uint8_t);

        w8le(bp, payload_length);
        padv(bp, uint8_t);

        w8le(bp, payload_type);
        padv(bp, uint8_t);


        // Initialize status
        new_irb.status   = IRB_PENDING;
        // Fill-in Class ID
        new_irb.class_id = IRB_BLE;
        // Copy pointer to allocated message
        new_irb.ptr_buf  = p_msg_rx;

        ble_mgr_command_queue_send(&new_irb, OS_QUEUE_FOREVER);
}

/**
* \brief Stop TX
*
* \return void
*
*/
void rf_tools_ble_stop_test(void)
{
        OS_IRB new_irb;
        irb_ble_stack_msg_t *p_msg_rx = ble_hci_alloc(HCI_CMD_MSG, 0);
        uint8_t *bp = (uint8_t *)&p_msg_rx->msg;

        w16le(bp, 0x201F);
        padv(bp, uint16_t);

        w8le(bp, 0x0);
        padv(bp, uint8_t);


        // Initialize status
        new_irb.status   = IRB_PENDING;
        // Fill-in Class ID
        new_irb.class_id = IRB_BLE;
        // Copy pointer to allocated message
        new_irb.ptr_buf  = p_msg_rx;

        ble_mgr_command_queue_send(&new_irb, OS_QUEUE_FOREVER);
}

void rf_tools_ble_start_pkt_rx_stats(uint8_t freq)
{
        OS_IRB new_irb;
        irb_ble_stack_msg_t *p_msg_rx = ble_hci_alloc(HCI_CMD_MSG, 1);
        uint8_t *bp = (uint8_t *)&p_msg_rx->msg;

        w16le(bp, 0xFC81);
        padv(bp, uint16_t);

        w8le(bp, 0x1);
        padv(bp, uint8_t);

        w8le(bp, freq);
        padv(bp, uint8_t);

        // Initialize status
        new_irb.status   = IRB_PENDING;
        // Fill-in Class ID
        new_irb.class_id = IRB_BLE;
        // Copy pointer to allocated message
        new_irb.ptr_buf  = p_msg_rx;

        ble_mgr_command_queue_send(&new_irb, OS_QUEUE_FOREVER);


}

void rf_tools_ble_stop_pkt_rx_stats(void)
{
        OS_IRB new_irb;
        irb_ble_stack_msg_t *p_msg_rx = ble_hci_alloc(HCI_CMD_MSG, 0);
        uint8_t *bp = (uint8_t *)&p_msg_rx->msg;

        w16le(bp, 0xFC82);
        padv(bp, uint16_t);

        w8le(bp, 0x0);
        padv(bp, uint8_t);

        // Initialize status
        new_irb.status   = IRB_PENDING;
        // Fill-in Class ID
        new_irb.class_id = IRB_BLE;
        // Copy pointer to allocated message
        new_irb.ptr_buf  = p_msg_rx;

        ble_mgr_command_queue_send(&new_irb, OS_QUEUE_FOREVER);
}

/**
* \brief  Start continuous non-packetized ble tx
*
* \param  [in] freq: frequency channel, 0-39
* \param  [in] payload_type: 0: PRBS9, 1: 11110000, 2: 10101010, 3: Vendor Specific
*
* \return void
*
*/
void rf_tools_ble_start_cont_tx(uint8_t freq, uint8_t payload_type)
{
        OS_IRB new_irb;
        irb_ble_stack_msg_t *p_msg_rx = ble_hci_alloc(HCI_CMD_MSG, 2);
        uint8_t *bp = (uint8_t *)&p_msg_rx->msg;

        w16le(bp, 0xFC84);
        padv(bp, uint16_t);

        w8le(bp, 0x2);
        padv(bp, uint8_t);

        w8le(bp, freq);
        padv(bp, uint8_t);

        w8le(bp, payload_type);
        padv(bp, uint8_t);


        // Initialize status
        new_irb.status   = IRB_PENDING;
        // Fill-in Class ID
        new_irb.class_id = IRB_BLE;
        // Copy pointer to allocated message
        new_irb.ptr_buf  = p_msg_rx;

        ble_mgr_command_queue_send(&new_irb, OS_QUEUE_FOREVER);
}

void rf_tools_ble_stop_cont_tx(void)
{
        OS_IRB new_irb;
        irb_ble_stack_msg_t *p_msg_rx = ble_hci_alloc(HCI_CMD_MSG, 0);
        uint8_t *bp = (uint8_t *)&p_msg_rx->msg;

        w16le(bp, 0xFC85);
        padv(bp, uint16_t);

        w8le(bp, 0x0);
        padv(bp, uint8_t);

        // Initialize status
        new_irb.status   = IRB_PENDING;
        // Fill-in Class ID
        new_irb.class_id = IRB_BLE;
        // Copy pointer to allocated message
        new_irb.ptr_buf  = p_msg_rx;

        ble_mgr_command_queue_send(&new_irb, OS_QUEUE_FOREVER);
}

/**
* \brief  Start packetized ble tx with interval
*
* \param  [in] freq: frequency channel, 0-39
* \param  [in] payload_lendth: the length of the packet payload to TX (0x0 - 0x25)
* \param  [in] payload_type: 0: PRBS9, 1: 11110000, 2: 10101010, 3: Vendor Specific
* \param  [in] number of packets to transmit. For infinite, use 0
* \param  [in] inter-packet interval, in uS
*
* \return void
*
*/
void rf_tools_ble_start_pkt_tx_interval(uint8_t freq, uint8_t payload_length,
                uint8_t payload_type, uint16_t num, uint32_t intv)
{
        OS_IRB new_irb;
        irb_ble_stack_msg_t *p_msg_rx = ble_hci_alloc(HCI_CMD_MSG, 9);
        uint8_t *bp = (uint8_t *)&p_msg_rx->msg;

        w16le(bp, 0xFC90);
        padv(bp, uint16_t);

        w8le(bp, 0x3);
        padv(bp, uint8_t);

        w8le(bp, freq);
        padv(bp, uint8_t);

        w8le(bp, payload_length);
        padv(bp, uint8_t);

        w8le(bp, payload_type);
        padv(bp, uint8_t);

        w16le(bp, num);
        padv(bp, uint16_t);

        w32le(bp, intv);
        padv(bp, uint32_t);

        // Initialize status
        new_irb.status   = IRB_PENDING;
        // Fill-in Class ID
        new_irb.class_id = IRB_BLE;
        // Copy pointer to allocated message
        new_irb.ptr_buf  = p_msg_rx;

        ble_mgr_command_queue_send(&new_irb, OS_QUEUE_FOREVER);
}


#endif
