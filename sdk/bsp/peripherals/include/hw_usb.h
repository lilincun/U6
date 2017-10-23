/*
 ****************************************************************************************
 *
 * @file hw_usb.h
 *
 * @brief header for low level DA1680 USB driver.
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
#ifndef USBDRIVER_H
#define USBDRIVER_H

#if dg_configUSE_HW_USB

/*========================== Include files ==================================*/

#include <hw_usb_ch9.h>

/*========================== Local macro definitions & typedefs =============*/

#define USB_INTERRUPT_PRIO      3
#define USB_EP_DEFAULT          0
#define USB_EP_MAX              7

/**
 * \brief USB event types.
 */
typedef enum {
        UBE_SUSPEND,  // 3 ms suspend detected.
        UBE_RWKUP_OK, // 5 ms suspend detected, remote wakeup allowed.
        UBE_RESUME,   // Resume detected.
        UBE_RESET,    // Reset detected.
        UBE_MAX
} usb_bus_event_type;

/**
 * \brief USB statistic counters.
 */
typedef struct {
        uint16 frame_nr;
        uint8 sd3;
        uint8 sd5;
        uint8 reset;
        uint8 resume;
        uint8 tx_ev;
        uint8 tx_ev0;
        uint8 nak;
        uint8 nak0;
        uint8 rx_ev;
        uint8 rx_ev0;
} usb_stat_type;

/*========================== Global variabeles ==============================*/

extern volatile usb_stat_type ud_stat;

/*========================== Global function prototypes =====================*/

/**
 * \brief Disables the USB interrupt.
 *
 */
static inline void hw_usb_disable_interrupt(void)
{
        // Disable interupt
        REG_CLR_BIT(USB, USB_MAMSK_REG, USB_M_INTR);

        NVIC_DisableIRQ(USB_IRQn);
}

/**
 * \brief Enables the USB interrupt.
 *
 */
static inline void hw_usb_enable_interrupt(void)
{
        // Enable interrupt.
        REG_SET_BIT(USB, USB_MAMSK_REG, USB_M_INTR);

        NVIC_SetPriority(USB_IRQn, USB_INTERRUPT_PRIO);
        NVIC_ClearPendingIRQ(USB_IRQn);
        NVIC_EnableIRQ(USB_IRQn);
}

/**
 * \brief Must be called when an USB interrupt is handled.
 *
 */
void hw_usb_interrupt_handler(void);

/**
 * \brief Initialize the USB device driver & chip.
 *
 */
void hw_usb_init(void);

/**
 * \brief Disable the USB device driver & chip.
 *
 */
void hw_usb_disable(void);

/**
 * \brief Attach to USB bus.
 *
 */
void hw_usb_bus_attach(void);

/**
 * \brief Detach from USB bus.
 *
 */
void hw_usb_bus_detach(void);

/**
 * \brief Restore interrupts masks after resume.
 *
 * \detail On suspend, all interrupts are masked except for ALTEV interrupts. So, when getting out
 *        of suspend (VBUS falling or USB Reset or Resume interrupt pending), the masks must be
 *        removed before continuing.
 */
void hw_usb_restore_int_mask_at_resume(void);

/**
 * \brief Enable 3ms Suspend detection (after enumeration is completed).
 *
 */
static inline void hw_usb_enable_suspend(void)
{
        REG_SET_BIT(USB, USB_ALTMSK_REG, USB_M_SD3);
        REG_SET_BIT(USB, USB_ALTMSK_REG, USB_M_RESUME);
}

/**
 * \brief Disable 3ms Suspend detection
 *
 */
static inline void hw_usb_disable_suspend(void)
{
        REG_CLR_BIT(USB, USB_ALTMSK_REG, USB_M_SD3);
        REG_CLR_BIT(USB, USB_ALTMSK_REG, USB_M_RESUME);
}

/**
 * \brief Issue resume signaling on USB bus.
 *
 */
void hw_usb_bus_resume(void);

/**
 * \brief Set new USB bus address.
 *
 * \param[in] address USB bus address.
 *
 */
void hw_usb_bus_address(uint8 address);

/**
 * \brief Bus event callback.
 *
 * \param[in] event The event that occurred.
 *
 */
void hw_usb_bus_event(usb_bus_event_type event);

/**
 * \brief Callback with current USB frame.
 *
 * \param[in] frame_nr Current USB frame number (0..7FF).
 *
 */
void hw_usb_bus_frame(uint16 frame_nr);

/**
 * \brief Configure an endpoint in the USB driver.
 *
 * \param[in] ep_nr The endpoint number to configure.
 * \param[in] zero_terminate Set TRUE to enable zero termination of transfers that are an exact
 *            multiple of the endpoint packet size.
 * \param[in] config Reference to the endpoint descriptor.
 *
 */
void hw_usb_ep_configure(uint8 ep_nr, bool zero_terminate, const usb_endpoint_descriptor_type* config);

/**
 * \brief Stall endpoint zero.
 *
 */
void hw_usb_ep0_stall(void);

/**
 * \brief Stall an endpoint.
 *
 * \param[in] ep_nr The endpoint number to stall.
 *
 */
void hw_usb_ep_stall(uint8 ep_nr);

/**
 * \brief Unstall an endpoint.
 *
 * \param[in] ep_nr The endpoint number to unstall.
 *
 */
void hw_usb_ep_unstall(uint8 ep_nr);

/**
 * \brief Check if an endpoint is currently stalled.
 *
 * \param[in] ep_nr The endpoint number to check.
 *
 * \return TRUE if the endpoint is stalled.
 *
 */
bool hw_usb_ep_is_stalled(uint8 ep_nr);

/**
 * \brief Enable receive for an endpoint.
 *
 * \param[in] ep_nr The endpoint number to enable.
 *
 */
void hw_usb_ep_rx_enable(uint8 ep_nr);

/**
 * \brief Callback function which should indicate if the USB driver should read Rx data from the
 *        fifo or if for instead the DMA is used to read the data.
 *
 * \param[in] ep_nr The endpoint number to enable.
 *
 * \return Needs to be true when the driver should read the data and false when the data is read by
 *        something else.
 *
 */
bool hw_usb_ep_rx_read_by_driver(uint8 ep_nr);

/**
 * \brief Callback to get buffer to receive data.
 *
 * \param[in] ep_nr The receiving endpoint number.
 * \param[in] is_setup TRUE if the packet received is a SETUP packet.
 * \param[in] buffer_size Output for specifying the maximum size to receive.
 *
 * \return Buffer for receive data.
 *
 */
uint8* hw_usb_ep_get_rx_buffer(uint8 ep_nr, bool is_setup, uint16* buffer_size);

/**
 * \brief Callback for receive complete.
 *
 * \param[in] ep_nr The endpoint number to check.
 * \param[in] buffer Buffer with received data.
 * \param[in] size Number of bytes placed in the Buffer.
 *
 * \return TRUE to re-enable receive on the endpoint.
 *
 */
bool hw_usb_ep_rx_done(uint8 ep_nr, uint8* buffer, uint16 size);

/**
 * \brief Transmit on an endpoint.
 *
 * \param[in] ep_nr The endpoint number to transmit on.
 * \param[in] buffer Data to transmit.
 * \param[in] size Number of bytes in Buffer.
 *
 */
void hw_usb_ep_tx_start(uint8 ep_nr, uint8* buffer, uint16 size);

/**
 * \brief Callback for transmit complete.
 *
 * \param[in] ep_nr The endpoint number to transmit on.
 * \param[in] buffer Data that was transmitted.
 *
 */
void hw_usb_ep_tx_done(uint8 ep_nr, uint8* buffer);

/**
 * \brief Disable an endpoint. Any pending RX/TX operations are cancelled.
 *
 * \param[in] ep_nr The endpoint number to disable.
 * \param[in] clear_toggle TRUE is toggle is to be cleared.
 *
 */
void hw_usb_ep_disable(uint8 ep_nr, bool clear_toggle);

/**
 * \brief Endpoint NAK control. Default enabled for EP0.
 *
 * \param[in] ep_nr The endpoint number to control.
 * \param[in] enable Set TRUE to generate NAK for the endpoint.
 *
 */
void hw_usb_ep_set_nak(uint8 ep_nr, bool enable);

/**
 * \brief Endpoint NAK callback.
 *
 * \param[in] ep_nr The endpoint number that a NAK was generated for.
 *
 */
void hw_usb_ep_nak(uint8 ep_nr);

/**
 * \brief Freeze USB
 *
 */
static inline void hw_usb_freeze(void)
{
        GPREG->SET_FREEZE_REG = 1 << REG_POS(GPREG, SET_FREEZE_REG, FRZ_USB);
}

/**
 * \brief Unfreeze USB
 *
 */
static inline void hw_usb_unfreeze(void)
{
        GPREG->RESET_FREEZE_REG = 1 << REG_POS(GPREG, SET_FREEZE_REG, FRZ_USB);
}

#endif  /* dg_configUSE_HW_USB */

/**
 * \brief Check if the USB i/f is active.
 *
 * \return true, if it is active else false
 *
 */
static inline bool hw_usb_active(void)
{
        return REG_GETF(USB, USB_MCTRL_REG, USBEN);
}

#endif /* USBDRIVER_H  */

// End of file.

