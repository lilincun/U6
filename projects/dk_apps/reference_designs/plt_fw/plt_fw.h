/*
 * plt_fw.h
 *
 *  Created on: Sep 11, 2015
 *      Author: akostop
 */

#ifndef PLT_FW_H_
#define PLT_FW_H_

#include "dgtl_msg.h"
#include "dgtl_pkt.h"

#define PLT_VERSION_STR "1.1"

/* Common event header */
typedef struct {
        dgtl_pkt_hci_evt_t hci_evt;
        uint8_t            num_hci_cmd_packets;
        uint16_t           opcode;
} __attribute__((packed)) plt_evt_hdr_t;

/* xtrim HCI command */
typedef struct {
        dgtl_pkt_hci_cmd_t hci_cmd;
        uint8_t            operation;
        uint16_t           value;
} __attribute__((packed)) plt_cmd_xtrim_t;

/* xtrim HCI event response */
typedef struct {
        plt_evt_hdr_t hdr;
        uint16_t      trim_value;
} __attribute__((packed)) plt_evt_xtrim_t;

/* hci_firmware_version_get HCI command */
typedef struct {
        dgtl_pkt_hci_cmd_t hci_cmd;
} __attribute__((packed)) plt_cmd_hci_firmware_version_get_t;

/* hci_firmware_version_get HCI event response */
typedef struct {
        plt_evt_hdr_t hdr;
        uint8_t       ble_version_length;
        uint8_t       app_version_length;
        char          ble_fw_version[32];
        char          app_fw_version[32];
} __attribute__((packed)) plt_evt_hci_firmware_version_get_t;

/* hci_custom_action HCI command */
typedef struct {
        dgtl_pkt_hci_cmd_t hci_cmd;
        uint8_t            custom_action;
} __attribute__((packed)) plt_cmd_hci_custom_action_t;

/* hci_custom_action HCI event response */
typedef struct {
        plt_evt_hdr_t hdr;
        uint8_t       custom_action;
} __attribute__((packed)) plt_evt_hci_custom_action_t;

/* hci_read_adc HCI command */
typedef struct {
        dgtl_pkt_hci_cmd_t hci_cmd;
        uint8_t            samples_nr;
        uint8_t            samples_period;
} __attribute__((packed)) plt_cmd_hci_read_adc_t;

/* hci_read_adc HCI event response */
typedef struct {
        plt_evt_hdr_t hdr;
        uint16_t      result;
} __attribute__((packed)) plt_evt_hci_read_adc_t;

void plt_parse_dgtl_msg(dgtl_msg_t *msg);

#endif /* PLT_FW_H_ */
