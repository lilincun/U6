/**
 ****************************************************************************************
 *
 * @file plt_fw.c
 *
 * @brief PLT Firmware core code.
 *
 * Copyright (C) 2015. Dialog Semiconductor, unpublished work. This computer
 * program includes Confidential, Proprietary Information and is a Trade Secret of
 * Dialog Semiconductor.  All use, disclosure, and/or reproduction is prohibited
 * unless authorized in writing. All Rights Reserved.
 *
 * <black.orca.support@diasemi.com> and contributors.
 *
 ****************************************************************************************
 */
#include <string.h>
#include <stdio.h>

#include "osal.h"
#include "hw_gpio.h"

#include "ble_mgr.h"
#include "ble_mgr_irb_common.h"
#include "co_version.h"

#include "packers.h"
#include "plt_fw.h"
#include "Xtal_TRIM.h"

#include "ad_gpadc.h"
#include <platform_devices.h>

#include "dgtl.h"
#include "dgtl_msg.h"
#include "dgtl_pkt.h"


#define MAX_TRIM 2047
#define MIN_TRIM 0

typedef void (*plt_cmd_handler)(dgtl_msg_t *msg);

void xtal_trim(dgtl_msg_t *msg);
void fw_version_get(dgtl_msg_t *msg);
void hci_custom_action(dgtl_msg_t *msg);
void hci_read_adc(dgtl_msg_t *msg);

plt_cmd_handler plt_cmd_handlers[] = {
		NULL,
		NULL,
		xtal_trim,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		fw_version_get,
		NULL,
		hci_custom_action, /* 0xFE0A */
		hci_read_adc       /* 0xFE0B */

};

dgtl_msg_t *init_response_evt(dgtl_pkt_hci_cmd_t *cmd, size_t length)
{
        dgtl_msg_t *msg_evt;
        plt_evt_hdr_t *evt;
        size_t param_len;

        param_len = length - sizeof(dgtl_pkt_hci_evt_t);

        msg_evt = dgtl_msg_prepare_hci_evt(NULL, 0x0E /* Command Complete Event */, param_len, NULL);
        evt = (plt_evt_hdr_t *) msg_evt;

        /* Clear parameters of event packet */
        memset(dgtl_msg_get_param_ptr(msg_evt, NULL), 0, param_len);

        evt->num_hci_cmd_packets = 1;
        evt->opcode = cmd->opcode;

        return msg_evt;
}

void plt_parse_dgtl_msg(dgtl_msg_t *msg)
{
        const dgtl_pkt_hci_cmd_t *pkt = (const dgtl_pkt_hci_cmd_t *) msg;

        /* Get only lower 8-bits of OCF (others are unused) */
        uint8_t cmd = pkt->opcode;

        if ((cmd < (sizeof(plt_cmd_handlers) / sizeof(plt_cmd_handler))) && plt_cmd_handlers[cmd]) {
                plt_cmd_handlers[cmd](msg);
        }

        dgtl_msg_free(msg);
}

static inline void enable_output_xtal(void)
{
	GPIO->GPIO_CLK_SEL = 0x3; /* Select XTAL16 clock */
	hw_gpio_set_pin_function(HW_GPIO_PORT_0, HW_GPIO_PIN_5, HW_GPIO_MODE_OUTPUT, HW_GPIO_FUNC_CLOCK);
}

static inline void disable_output_xtal(void)
{
	hw_gpio_set_pin_function(HW_GPIO_PORT_0, HW_GPIO_PIN_5, HW_GPIO_MODE_INPUT, HW_GPIO_FUNC_GPIO);
}

static inline uint16_t auto_xtal_trim(uint16_t gpio_input)
{

        int r;
        HW_GPIO_PORT port;
        HW_GPIO_PIN pin;
        HW_GPIO_MODE mode;
        HW_GPIO_FUNC function;

        int gpio = gpio_input & 0xFF;
        port = gpio / 10;
        pin = gpio % 10;

        /* Store pulse input gpio previous mode and function */
        hw_gpio_get_pin_function(port, pin, &mode, &function);

        r = auto_trim(gpio);

        /* Restore pulse input gpio previous mode and functions.
         * This is needed because they use the UART RX pin for
         * pulse input. It must be restored to resume UART operation
         */
        hw_gpio_set_pin_function(port, pin, mode, function);


        if (r < 0)
                return -r;
        else
                return 0;
}

void xtal_trim(dgtl_msg_t *msg)
{
        plt_cmd_xtrim_t *cmd = (plt_cmd_xtrim_t *) msg;
        dgtl_msg_t *msg_evt;
        plt_evt_xtrim_t *evt;

        msg_evt = init_response_evt(&cmd->hci_cmd, sizeof(*evt));
        evt = (plt_evt_xtrim_t *) msg_evt;

        switch (cmd->operation) {
        case 0: /* Read trim value */
                evt->trim_value = CRG_TOP->CLK_FREQ_TRIM_REG;
                break;
        case 1: /* Write trim value */
                CRG_TOP->CLK_FREQ_TRIM_REG = cmd->value;
                break;
        case 2: /* Enable output xtal on P05 */
                enable_output_xtal();
                break;
        case 3: /* Increase trim value by delta */
                CRG_TOP->CLK_FREQ_TRIM_REG = CRG_TOP->CLK_FREQ_TRIM_REG + cmd->value;
                break;
        case 4: /* Decrease trim value by delta */
                CRG_TOP->CLK_FREQ_TRIM_REG = CRG_TOP->CLK_FREQ_TRIM_REG - cmd->value;
                break;
        case 5: /* Disable output xtal on P05 */
                disable_output_xtal();
                break;
        case 6: /* Auto calibration test */
                evt->trim_value = auto_xtal_trim(cmd->value);
                break;
        }

        dgtl_send(msg_evt);
}

void fw_version_get(dgtl_msg_t *msg)
{
        plt_cmd_hci_firmware_version_get_t *cmd = (plt_cmd_hci_firmware_version_get_t *) msg;
        dgtl_msg_t *msg_evt;
        plt_evt_hci_firmware_version_get_t *evt;

        msg_evt = init_response_evt(&cmd->hci_cmd, sizeof(*evt));
        evt = (plt_evt_hci_firmware_version_get_t *) msg_evt;

        evt->ble_version_length = snprintf(evt->ble_fw_version, sizeof(evt->ble_fw_version),
                                                "%d.%d.%d.%d", RWBLE_SW_VERSION_MAJOR,
                                                RWBLE_SW_VERSION_MINOR, RWBLE_SW_VERSION_BUILD,
                                                RWBLE_SW_VERSION_SUB_BUILD ) + 1;

        evt->app_version_length = strlen(PLT_VERSION_STR) + 1;
        memcpy(evt->app_fw_version, PLT_VERSION_STR, evt->app_version_length);

        dgtl_send(msg_evt);
}

void hci_custom_action(dgtl_msg_t *msg)
{
        plt_cmd_hci_custom_action_t *cmd = (plt_cmd_hci_custom_action_t *) msg;
        dgtl_msg_t *msg_evt;
        plt_evt_hci_custom_action_t *evt;

        msg_evt = init_response_evt(&cmd->hci_cmd, sizeof(*evt));
        evt = (plt_evt_hci_custom_action_t *) msg_evt;

        evt->custom_action = cmd->custom_action;

        dgtl_send(msg_evt);
}

void hci_read_adc(dgtl_msg_t *msg)
{
        plt_cmd_hci_read_adc_t *cmd = (plt_cmd_hci_read_adc_t *) msg;
        dgtl_msg_t *msg_evt;
        plt_evt_hci_read_adc_t *evt;
        uint16_t adc_value;
        uint16_t adc_offs_p = 0x200;
        uint16_t adc_offs_n = 0x200;
        uint32_t sum = 0;
        int i;

        msg_evt = init_response_evt(&cmd->hci_cmd, sizeof(*evt));
        evt = (plt_evt_hci_read_adc_t *) msg_evt;

        /* Hack: We can't really use ADC adapter here, since it only
         * returns the 10-bit result, not the 16-bit oversampled one
         */
        resource_acquire(RES_MASK(RES_ID_GPADC), RES_WAIT_FOREVER);

        adc_offs_p = hw_gpadc_get_offset_positive();
        adc_offs_n = hw_gpadc_get_offset_negative();

        hw_gpadc_reset();

        hw_gpadc_set_input(HW_GPADC_INPUT_SE_VBAT);
        hw_gpadc_set_input_mode(HW_GPADC_INPUT_MODE_SINGLE_ENDED);
        hw_gpadc_set_ldo_constant_current(true);
        hw_gpadc_set_ldo_dynamic_current(true);
        hw_gpadc_adc_measure(); // dummy (fast) measurement
        hw_gpadc_set_sample_time(15);

        hw_gpadc_set_offset_positive(0x200);
        hw_gpadc_set_offset_negative(0x200);

        hw_gpadc_set_chopping(true);
        hw_gpadc_set_oversampling(7); // 128 samples
        for (volatile int i = 10; i > 0; i--)
                ; // Make sure 1usec has passed since the mode setting (VBAT).

        OS_ENTER_CRITICAL_SECTION();
        for (i = 0; i < cmd->samples_nr; i++) {
                hw_gpadc_adc_measure();
                adc_value = hw_gpadc_get_raw_value();
                sum += adc_value;
                if (cmd->samples_period > 0)
                        OS_DELAY_MS(cmd->samples_period);
        }
        OS_LEAVE_CRITICAL_SECTION();

        hw_gpadc_set_offset_positive(adc_offs_p);
        hw_gpadc_set_offset_negative(adc_offs_n);

        resource_release(RES_MASK(RES_ID_GPADC));

        evt->result = ((cmd->samples_nr != 0) ? sum / cmd->samples_nr : 0);

        dgtl_send(msg_evt);
}
