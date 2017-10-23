 /*
 ****************************************************************************************
 *
 * @file UsbApp.h
 *
 * @brief header for USB application for the DA1680 USB driver.
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

#ifndef USBAPP_H_
#define USBAPP_H_

#define USB_INTERRUPT_PRIORITY 	1
#define USB_INTERRPT_TASK_PRIO 	1
#define USB_ATTACH_PRIORITY		2


/**
 * \brief Initialize USB function and driver.
 *
 */
void usb_init(void);

#endif // USBAPP_H_
