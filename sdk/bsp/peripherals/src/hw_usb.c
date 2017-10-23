/*
 ****************************************************************************************
 *
 * @file hw_usb.c
 *
 * @brief Low level DA1680 USB driver.
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

#if dg_configUSE_HW_USB

/*========================== Include files ==================================*/

#include <string.h>
#include "hw_usb.h"
#include "hw_usb_charger.h"
#include "sys_charger.h"
#include "core_cm0.h"
#include "hw_dma.h"

/*========================== Local macro definitions ========================*/

// Node Functional State Register.
typedef enum {
        NFSR_NODE_RESET, NFSR_NODE_RESUME, NFSR_NODE_OPERATIONAL, NFSR_NODE_SUSPEND
} nfsr_type;

// Endpoint Control Registers.
#define EPC_EP_MASK     USB_USB_EPC1_REG_USB_EP_Msk
#define EPC_EP_EN       USB_USB_EPC1_REG_USB_EP_EN_Msk          // Not in EPC0.
#define EPC_ISO         USB_USB_EPC1_REG_USB_ISO_Msk            // Not in EPC0.
#define EPC_DEF         USB_USB_EPC0_REG_USB_DEF_Msk            // Only in EPC0.
#define EPC_STALL       USB_USB_EPC0_REG_USB_STALL_Msk

// TX status registers.
#define TXS_TCOUNT_MASK USB_USB_TXS0_REG_USB_TCOUNT_Msk
#define TXS_TX_DONE     USB_USB_TXS0_REG_USB_TX_DONE_Msk
#define TXS_ACK_STAT    USB_USB_TXS0_REG_USB_ACK_STAT_Msk
#define TXS_TX_URUN     USB_USB_TXS1_REG_USB_TX_URUN_Msk        // Not in TXS0.

// Transmit Command Registers.
#define TXC_TX_EN       USB_USB_TXC1_REG_USB_TX_EN_Msk
#define TXC_LAST        USB_USB_TXC1_REG_USB_LAST_Msk
#define TXC_TOGGLE      USB_USB_TXC1_REG_USB_TOGGLE_TX_Msk
#define TXC_FLUSH       USB_USB_TXC1_REG_USB_FLUSH_Msk
#define TXC_RFF         USB_USB_TXC1_REG_USB_RFF_Msk
#define TXC_TFWL_MASK   USB_USB_TXC1_REG_USB_TFWL_Msk
#define TXC_TFWL_4      (0x1 << 5)
#define TXC_TFWL_8      (0x2 << 5)
#define TXC_TFWL_16     (0x3 << 5)
#define TXC_IGN_ISOMSK  USB_USB_TXC1_REG_USB_IGN_ISOMSK_Msk

// Receive Status Registers.
#define RXS_RCOUNT_MASK USB_USB_RXS1_REG_USB_RCOUNT_Msk
#define RXS_RX_LAST     USB_USB_RXS1_REG_USB_RX_LAST_Msk
#define RXS_TOGGLE      USB_USB_RXS1_REG_USB_TOGGLE_RX_Msk
#define RXS_SETUP       USB_USB_RXS1_REG_USB_SETUP_Msk
#define RXS_RX_ERR      USB_USB_RXS1_REG_USB_RX_ERR_Msk

// Receive Command Registers.
#define RXC_RX_EN       USB_USB_RXC0_REG_USB_RX_EN_Msk
#define RXC_IGN_OUT     USB_USB_RXC0_REG_USB_IGN_OUT_Msk
#define RXC_IGN_SETUP   USB_USB_RXC0_REG_USB_IGN_SETUP_Msk
#define RXC_FLUSH       USB_USB_RXC0_REG_USB_FLUSH_Msk
#define RXC_RFWL_MASK   USB_USB_RXC1_REG_USB_RFWL_Msk
#define RXC_RFWL_4      (0x1 << 5)
#define RXC_RFWL_8      (0x2 << 5)
#define RXC_RFWL_16     (0x3 << 5)

typedef struct {
        volatile uint16 *epc;
        volatile uint16 *txc;
        volatile uint16 *txs;
        volatile uint16 *txd;
        volatile uint16 *rxc;
        volatile uint16 *rxs;
        volatile uint16 *rxd;
} ep_regs_type;

typedef struct {
        uint8 type :2; // Control, iso, bulk or interrupt.
        uint8 zero_terminate :1;
        uint8 toggle :1;
        uint8 spare :3;
        uint8 tx_busy :1;
} ep_flags_type;

typedef struct {
        uint16 max_size;
        uint16 actual_size;
        uint8  packet_size;
        uint16 actual_size_txfill;
        uint8  packet_size_txfill;
        uint8 *buffer;
} ep_buffer_type;

typedef struct {
        ep_flags_type flags;
        uint8 mps;
        ep_buffer_type tx;
        ep_buffer_type rx;
} ep_data_type;

typedef struct {
        uint8 sof;    // One or more SOF's lost.
        uint8 rx_err; // RXS_RX_ERR
        uint8 tx_rff; // !TXS_ACK_STAT
        uint8 toggle; // Wrong toggle.
} ud_err_type;

/*========================== Global definitions =============================*/

extern void hw_charger_usb_cb(uint16_t status) __attribute__((weak));


/*========================== Local function prototypes ======================*/

/*========================== Local data definitions =========================*/

#define EPREGS(epc, txc, txs, txd, rxc, rxs, rxd) \
                (volatile uint16*)epc, (volatile uint16*)txc, (volatile uint16*)txs, \
                (volatile uint16*)txd, (volatile uint16*)rxc, (volatile uint16*)rxs, \
                (volatile uint16*)rxd

// Table for looking up endpoint registers.
static const ep_regs_type ep_regs[] = {
        { EPREGS(&(USB->USB_EPC0_REG),
                        &(USB->USB_TXC0_REG), &(USB->USB_TXS0_REG), &(USB->USB_TXD0_REG),
                        &(USB->USB_RXC0_REG), &(USB->USB_RXS0_REG), &(USB->USB_RXD0_REG)) },
        { EPREGS(&(USB->USB_EPC1_REG),
                        &(USB->USB_TXC1_REG), &(USB->USB_TXS1_REG), &(USB->USB_TXD1_REG),
                        0, 0, 0) },
        { EPREGS(&(USB->USB_EPC2_REG),
                        0, 0, 0,
                        &(USB->USB_RXC1_REG), &(USB->USB_RXS1_REG), &(USB->USB_RXD1_REG)) },
        { EPREGS(&(USB->USB_EPC3_REG),
                        &(USB->USB_TXC2_REG), &(USB->USB_TXS2_REG), &(USB->USB_TXD2_REG),
                        0, 0, 0) },
        { EPREGS(&(USB->USB_EPC4_REG),
                        0, 0, 0,
                        &(USB->USB_RXC2_REG), &(USB->USB_RXS2_REG), &(USB->USB_RXD2_REG)) },
        { EPREGS(&(USB->USB_EPC5_REG),
                        &(USB->USB_TXC3_REG), &(USB->USB_TXS3_REG), &(USB->USB_TXD3_REG),
                        0, 0, 0) },
        { EPREGS(&(USB->USB_EPC6_REG),
                        0, 0, 0,
                        &(USB->USB_RXC3_REG), &(USB->USB_RXS3_REG), &(USB->USB_RXD3_REG)) },
};

#undef EPREGS

static ep_data_type usb_endpoints[USB_EP_MAX];
volatile usb_stat_type ud_stat;
static ud_err_type ud_err;
static volatile nfsr_type ud_nfsr;
static uint16_t int_masks;

/*========================== Function definitions ===========================*/

/**
 * \brief Disable USB interrupt.
 *
 * \return The last USB interrupt state.
 *
 */
static uint8 save_usb_int(void)
{
        uint8 state;

        state = REG_GETF(USB, USB_MAMSK_REG, USB_M_INTR);

        REG_CLR_BIT(USB, USB_MAMSK_REG, USB_M_INTR);

        return state;
}

/**
 * \brief Restore USB interrupt to previous state.
 *
 * \param[in] state The previous USB interrupt state.
 *
 */
static void restore_usb_int(uint8 state)
{
        REG_SETF(USB, USB_MAMSK_REG, USB_M_INTR, state);
}

/**
 * \brief Check if TX is active for the endpoint and complete it.
 *
 * \param[in] ep_nr The endpoint number.
 * \param[in] ep The endpoint data.
 *
 */
static void tx_done(uint8 ep_nr, ep_data_type* ep)
{
        if (ep->flags.tx_busy) {
                ep->flags.tx_busy = 0;
                hw_usb_ep_tx_done(ep_nr, ep->tx.buffer);
        }
        else {
                /* Even though TX was not active, indicate TxDone anyway. Useful for isochronous
                 * transfers...
                 */
                hw_usb_ep_tx_done(ep_nr, NULL);
        }
}

/**
 * \brief Fill TX buffer for the endpoint.
 *
 * \param[in] ep_nr The endpoint number.
 *
 */
static void tx_fill(uint8 ep_nr)
{
        ep_data_type* ep = &usb_endpoints[ep_nr];
        volatile uint16* txc = ep_regs[ep_nr].txc;
        volatile uint16* txs = ep_regs[ep_nr].txs;
        volatile uint16* txd = ep_regs[ep_nr].txd;
        uint16 saved = ep->tx.max_size - ep->tx.actual_size;
        uint8 tcount = *txs & TXS_TCOUNT_MASK;
        uint8* pd = ep->tx.buffer + ep->tx.actual_size;
        uint16 remain;

        saved = MIN(saved, ep->mps);
        remain = saved;

        while (tcount && remain) {
                uint8 n = MIN(tcount, remain);

                tcount -= n;
                remain -= n;
                while (n--) {
                        *txd = *pd++;
                }
                tcount = *txs & TXS_TCOUNT_MASK;
        }
        saved -= remain;

        //save current values in case want to retry after NAK
        ep->tx.packet_size_txfill = ep->tx.packet_size;
        ep->tx.actual_size_txfill = ep->tx.actual_size;

        ep->tx.packet_size = saved;
        ep->tx.actual_size += saved;

        if (ep->flags.toggle) {
                *txc |= TXC_TOGGLE;
        }
        else {
                *txc &= ~TXC_TOGGLE;
        }

        if (ep_nr == 0) {
                if (*(ep_regs[0].rxc) & RXC_RX_EN) {
                        uint8 state;

                        state = save_usb_int();
                        *(ep_regs[0].rxc) &= ~RXC_RX_EN;
                        restore_usb_int(state);
                }
        }

        *txc |= TXC_LAST | TXC_TX_EN;
}

static void tx_fill_retry(uint8 ep_nr)
{
        ep_data_type* ep = &usb_endpoints[ep_nr];
        ep->tx.packet_size = ep->tx.packet_size_txfill;
        ep->tx.actual_size = ep->tx.actual_size_txfill;
        tx_fill(ep_nr);
}

/**
 * \brief Continue or complete TX for the endpoint.
 *
 * \param[in] ep_nr The endpoint number.
 *
 */
static void tx_ep(uint8 ep_nr)
{
        /*End point number should be lower than MAX EP*/
        ASSERT_WARNING(ep_nr < USB_EP_MAX);

        ep_data_type* ep = &usb_endpoints[ep_nr];
        const ep_regs_type* er = &ep_regs[ep_nr];
        uint8 txs = *(er->txs);

        if (txs & TXS_TX_DONE) {
                if ((txs & TXS_ACK_STAT) || ep->flags.type == USB_ENDPOINT_XFER_ISOC) {
                        ep->flags.toggle = !ep->flags.toggle;
                        if (ep->tx.actual_size < ep->tx.max_size) {
                                tx_fill(ep_nr);
                        }
                        else if (ep->flags.zero_terminate
                                        && ep->tx.packet_size == ep->mps) {
                                tx_fill(ep_nr);
                        }
                        else {
                                tx_done(ep_nr, ep);
                        }
                }
                else {
                        if (ep->flags.tx_busy) {
                                // If we didn't get an ACK, refill FIFO
                                ep->flags.tx_busy = 0;
                                tx_fill_retry(ep_nr);

                                ud_err.tx_rff++;
                        }
                }
        }
        else {
                if ((ep_nr == 0) && (ep->flags.tx_busy == 1)) {
                        ASSERT_WARNING(false);
                }
        }
}

/**
 * \brief Check for interrupt from TX endpoints.
 *
 */
static void tx_event(void)
{
        uint16 txev;

        txev = USB->USB_TXEV_REG;
        txev &= USB->USB_TXMSK_REG;

        if (txev & 0x0001) {
                tx_ep(1);
        }

        if (txev & 0x0002) {
                tx_ep(3);
        }

        if (txev & 0x0004) {
                tx_ep(5);
        }

        if (txev & 0x0008) {
                tx_ep(7);
        }

        if (txev & 0x0100) {
                tx_ep(9);
        }

        if (txev & 0x0200) {
                tx_ep(11);
        }

        if (txev & 0x0400) {
                tx_ep(13);
        }

        if (txev & 0x0800) {
                tx_ep(15);
        }
}

/**
 * \brief Check if RX is active for the endpoint and complete it.
 *
 * \param[in] ep_nr The endpoint number.
 * \param[in] ep The endpoint data.
 *
 * \return Always true.
 *
 */
static bool rx_done(uint8 ep_nr, ep_data_type* ep)
{
        bool reenable = true;

        if (ep->rx.max_size) {
                ep->rx.max_size = 0;
                reenable = hw_usb_ep_rx_done(ep_nr, ep->rx.buffer, ep->rx.actual_size);
                ep->rx.actual_size = 0;
        }

        return reenable;
}

/**
 * \brief Read RX data from endpoint FIFO.
 *
 * \param[in] ep_nr The endpoint number.
 * \param[in] setup Indication of SETUP packet type.
 *
 */
static void rx_ep_read(uint8 ep_nr, bool setup)
{
        bool reenable = true;
        volatile uint16* rxc = ep_regs[ep_nr].rxc;
        ep_data_type* ep = &usb_endpoints[ep_nr];

        if (hw_usb_ep_rx_read_by_driver(ep_nr)) {
                uint8 pktsize = 0;
                volatile uint16* rxs = ep_regs[ep_nr].rxs;
                volatile uint16* rxd = ep_regs[ep_nr].rxd;

                for (;;) {
                        uint8 rxsize = *rxs & RXS_RCOUNT_MASK;

                        if (rxsize == 0) {
                                break;
                        }

                        pktsize += rxsize;

                        while (rxsize) {
                                uint16 remain = ep->rx.max_size - ep->rx.actual_size;

                                if (remain) {
                                        uint8* pb = ep->rx.buffer + ep->rx.actual_size;
                                        uint8 n = MIN(remain, rxsize);

                                        ep->rx.actual_size += n;
                                        remain -= n;
                                        while (n--) {
                                                *pb++ = *rxd;
                                        }
                                }
                                else {
                                        reenable = rx_done(ep_nr, ep);
                                        ep->rx.buffer = hw_usb_ep_get_rx_buffer(ep_nr, setup, &ep->rx.max_size);

                                        if (ep->rx.buffer == NULL) {
                                                *rxc |= RXC_FLUSH;
                                                rxsize = 0;
                                        }

                                        ep->rx.actual_size = 0;
                                }
                                rxsize = *rxs & RXS_RCOUNT_MASK;
                        }
                }

                if (pktsize < ep->mps) {
                        reenable = rx_done(ep_nr, ep);
                }
                else {
                        if (ep->rx.actual_size == ep->rx.max_size) {
                                if (ep_nr != USB_EP_DEFAULT && ep->flags.zero_terminate) {
                                        reenable = true;        // Wait for zero length packet.
                                }
                                else {
                                        reenable = rx_done(ep_nr, ep);
                                }
                        }
                }
        }
        else {
                reenable = rx_done(ep_nr, ep);
        }

        if (reenable) {
                *rxc |= RXC_RX_EN;
        }
}

/**
 * \brief Receive on endpoint zero.
 *
 */
static void rx_ep0(void)
{
        ep_data_type* ep = usb_endpoints;
        uint8 rxs = USB->USB_RXS0_REG;

        if (rxs & RXS_RX_LAST) {
                if (rxs & RXS_SETUP) {
                        if ((rxs & RXS_RCOUNT_MASK) == ep->mps) {
                                USB->USB_EPC0_REG = USB->USB_EPC0_REG & (~EPC_STALL);
                                ep->flags.toggle = 1;
                                rx_ep_read(USB_EP_DEFAULT, true);
                        }
                        else {
                                hw_usb_ep0_stall();
                        }
                }
                else {
                        if (rxs & RXS_RCOUNT_MASK) {
                                rx_ep_read(USB_EP_DEFAULT, false);
                        }
                }
        }
}

/**
 * \brief Receive on endpoint.
 *
 * \param[in] ep_nr The endpoint number.
 *
 */
static uint8 rx_ep(uint8 ep_nr)
{
        /*End point number should be lower than MAX EP*/
        ASSERT_WARNING(ep_nr < USB_EP_MAX);

        ep_data_type* ep = &usb_endpoints[ep_nr];
        volatile uint16* rxc = ep_regs[ep_nr].rxc;
        uint8 rxs = *(ep_regs[ep_nr].rxs);

        if (rxs & RXS_RX_ERR) {
                *rxc |= RXC_FLUSH;
                ud_err.rx_err++;

                return rxs;
        }

        if (rxs & RXS_RX_LAST) {
                if (rxs & RXS_TOGGLE) {
                        if (ep->flags.toggle == 0) {
                                ud_err.toggle++;
                        }
                        ep->flags.toggle = 0;
                        rx_ep_read(ep_nr, false);
                }
                else {
                        if (ep->flags.toggle) {
                                ud_err.toggle++;
                        }
                        ep->flags.toggle = 1;
                        rx_ep_read(ep_nr, false);
                }
        }

        return rxs;
}

/**
 * \brief Check for interrupt from RX endpoints.
 *
 */
static void rx_event(void)
{
        uint16 rxev;

        rxev = USB->USB_RXEV_REG;
        rxev &= USB->USB_RXMSK_REG;

        if (rxev & 0x0001) {
                rx_ep(2);
        }

        if (rxev & 0x0002) {
                rx_ep(4);
        }

        if (rxev & 0x0004) {
                rx_ep(6);
        }

        if (rxev & 0x0008) {
                rx_ep(8);
        }

        if (rxev & 0x0100) {
                rx_ep(10);
        }

        if (rxev & 0x0200) {
                rx_ep(12);
        }

        if (rxev & 0x0400) {
                rx_ep(14);
        }

        if (rxev & 0x0800) {
                rx_ep(16);
        }
}

/**
 * \brief Check for NAK interrupt from endpoints 0.
 *
 */
static void nak_event_ep0(void)
{
        uint8 nak = USB->USB_EP0_NAK_REG;

        if (nak & 0x02) {
                ep_data_type* ep = &usb_endpoints[USB_EP_DEFAULT];

                if (ep->flags.tx_busy) {
                        hw_usb_ep_nak(0);
                }
        }
}

/**
 * \brief Check for NAK interrupt from all endpoints.
 *
 */
static void nak_event(void)
{
        uint16 nak;

        nak = USB->USB_NAKEV_REG;
        nak &= USB->USB_NAKMSK_REG;

        if (nak & 0x0001) {
                hw_usb_ep_nak(1);
        }

        if (nak & 0x0002) {
                hw_usb_ep_nak(3);
        }

        if (nak & 0x0004) {
                hw_usb_ep_nak(5);
        }

        if (nak & 0x0008) {
                hw_usb_ep_nak(7);
        }

        if (nak & 0x0100) {
                hw_usb_ep_nak(9);
        }

        if (nak & 0x0200) {
                hw_usb_ep_nak(11);
        }

        if (nak & 0x0400) {
                hw_usb_ep_nak(13);
        }

        if (nak & 0x0800) {
                hw_usb_ep_nak(15);
        }

        if (nak & 0x0010) {
                hw_usb_ep_nak(2);
        }

        if (nak & 0x0020) {
                hw_usb_ep_nak(4);
        }

        if (nak & 0x0040) {
                hw_usb_ep_nak(6);
        }

        if (nak & 0x0080) {
                hw_usb_ep_nak(8);
        }

        if (nak & 0x1000) {
                hw_usb_ep_nak(10);
        }

        if (nak & 0x2000) {
                hw_usb_ep_nak(12);
        }

        if (nak & 0x4000) {
                hw_usb_ep_nak(14);
        }

        if (nak & 0x8000) {
                hw_usb_ep_nak(16);
        }
}

/**
 * \brief Process SD3 interrupt.
 *
 */
static void sd3_event(void)
{
        REG_SET_BIT(USB, USB_ALTMSK_REG, USB_M_RESET);

        if (ud_nfsr == NFSR_NODE_OPERATIONAL) {
                ud_nfsr = NFSR_NODE_SUSPEND;
                USB->USB_NFSR_REG = ud_nfsr;

                REG_CLR_BIT(USB, USB_ALTMSK_REG, USB_M_SD3);
                int_masks = USB->USB_MAMSK_REG;
                USB->USB_MAMSK_REG = 0;
                REG_SET_BIT(USB, USB_MAMSK_REG, USB_M_ALT);
                REG_SET_BIT(USB, USB_MAMSK_REG, USB_M_INTR);

                hw_charger_setclk_ahb();
                hw_usb_bus_event(UBE_SUSPEND);
                hw_charger_suspended_cb();
        }
}

/**
 * \brief Process SD5 interrupt.
 *
 */
static void sd5_event(void)
{
}

/**
 * \brief Process reset interrupt.
 *
 */
static void reset_event(void)
{
        if (ud_nfsr == NFSR_NODE_SUSPEND) {
                if (dg_configUSB_SUSPEND_MODE == 2) {
                        hw_usb_restore_int_mask_at_resume();
                }
                hw_charger_setclk_pll();
                hw_charger_resumed_cb();
        }

        ud_nfsr = NFSR_NODE_RESET;
        USB->USB_NFSR_REG = ud_nfsr;
        REG_CLR_BIT(USB, USB_ALTMSK_REG, USB_M_RESET);
        ud_nfsr = NFSR_NODE_OPERATIONAL;
        USB->USB_NFSR_REG = ud_nfsr;
        hw_usb_bus_event(UBE_RESET);
}

/**
 * \brief Process resume interrupt.
 *
 */
static void resume_event(void)
{
        REG_SET_BIT(USB, USB_ALTMSK_REG, USB_M_RESET);

        if (ud_nfsr == NFSR_NODE_SUSPEND) {
                hw_charger_setclk_pll();
                ud_nfsr = NFSR_NODE_OPERATIONAL;
                USB->USB_NFSR_REG = ud_nfsr;
                hw_usb_bus_event(UBE_RESUME);
                hw_charger_resumed_cb();
                if (dg_configUSB_SUSPEND_MODE == 2) {
                        hw_usb_restore_int_mask_at_resume();
                }
        }
}

/**
 * \brief Process frame interrupt.
 *
 */
static void frame_event(void)
{
        uint16 frame;

        frame = USB->USB_FNL_REG;
        frame |= (REG_GETF(USB, USB_FNH_REG, USB_FN_10_8) << 8);

        if (frame != ud_stat.frame_nr) {
                ud_err.sof++;
        }

        hw_usb_bus_frame(frame);
        ud_stat.frame_nr = (frame + 1) & 0x7FF;
        REG_SET_BIT(USB, USB_ALTMSK_REG, USB_M_RESET);
}

void hw_usb_restore_int_mask_at_resume(void)
{
        REG_SET_BIT(USB, USB_ALTMSK_REG, USB_M_SD3);
        USB->USB_MAMSK_REG = int_masks;
}

void hw_usb_bus_attach(void)
{
        uint32_t reg;
        uint8 state;

        state = save_usb_int();

        REG_SET_BIT(USB, USB_MCTRL_REG, USB_NAT);

        reg = USB->USB_FAR_REG;

        REG_CLR_FIELD(USB, USB_FAR_REG, USB_AD, reg);
        REG_SET_FIELD(USB, USB_FAR_REG, USB_AD_EN, reg, 1);

        USB->USB_FAR_REG = reg;

        ud_nfsr = NFSR_NODE_RESET;
        USB->USB_NFSR_REG = ud_nfsr;

        reg = USB->USB_ALTEV_REG; // Clear pending interrupts

        reg = USB->USB_ALTMSK_REG;
        REG_CLR_FIELD(USB, USB_ALTMSK_REG, USB_M_RESUME, reg);
        REG_SET_FIELD(USB, USB_ALTMSK_REG, USB_M_RESET, reg, 1);
        REG_CLR_FIELD(USB, USB_ALTMSK_REG, USB_M_SD5, reg);
        REG_CLR_FIELD(USB, USB_ALTMSK_REG, USB_M_SD3, reg);
        USB->USB_ALTMSK_REG = reg;

        // Give control of USB pads to the USB core
        CRG_PER->USBPAD_REG = 0;

        restore_usb_int(state);
}

void hw_usb_bus_detach(void)
{
        uint32_t reg;
        uint8 state;

        state = save_usb_int();

        ud_nfsr = NFSR_NODE_RESET;
        USB->USB_NFSR_REG = ud_nfsr;

        REG_CLR_BIT(USB, USB_MCTRL_REG, USB_NAT);

        reg = USB->USB_ALTMSK_REG;
        REG_CLR_FIELD(USB, USB_ALTMSK_REG, USB_M_RESUME, reg);
        REG_CLR_FIELD(USB, USB_ALTMSK_REG, USB_M_RESET, reg);
        REG_CLR_FIELD(USB, USB_ALTMSK_REG, USB_M_SD5, reg);
        REG_CLR_FIELD(USB, USB_ALTMSK_REG, USB_M_SD3, reg);
        USB->USB_ALTMSK_REG = reg;

        restore_usb_int(state);
}

void hw_usb_bus_resume(void)
{
        ASSERT_WARNING(false);                          // not implemented yet;
}

void hw_usb_bus_address(uint8 address)
{
        uint32_t reg;
        uint8 state = save_usb_int();

        REG_SET_BIT(USB, USB_EPC0_REG, USB_DEF);

        reg = USB->USB_FAR_REG;
        REG_SET_FIELD(USB, USB_FAR_REG, USB_AD, reg, address);
        REG_SET_FIELD(USB, USB_FAR_REG, USB_AD_EN, reg, 1);
        USB->USB_FAR_REG = reg;

        restore_usb_int(state);
}

void hw_usb_ep_configure(uint8 ep_nr, bool zero_terminate,
                const usb_endpoint_descriptor_type* config)
{
        ep_data_type* ep = &usb_endpoints[ep_nr];
        volatile uint16* epc = ep_regs[ep_nr].epc;

        ep->flags.zero_terminate = zero_terminate;
        ep->flags.toggle = 0;

        if (config) {
                ep->flags.type = config->attributes & USB_ENDPOINT_XFERTYPE_MASK;
                ep->mps = config->max_packet_size;
                *epc = config->endpoint_address & EPC_EP_MASK;
                if (ep->flags.type == USB_ENDPOINT_XFER_ISOC) {
                        *epc |= EPC_ISO;
                        if ((config->endpoint_address & USB_ENDPOINT_DIR_MASK) == USB_HW_DIR_IN) {
                                *(ep_regs[ep_nr].txc) |= TXC_IGN_ISOMSK;
                        }
                }
        }
        else {
                ep->flags.type = USB_ENDPOINT_XFER_CONTROL;
                ep->mps = 8;
        }

        *epc |= EPC_EP_EN;
}

void hw_usb_ep0_stall(void)
{
        uint8 state;

        hw_usb_ep_stall(USB_EP_DEFAULT);
        hw_usb_ep_tx_start(USB_EP_DEFAULT, NULL, 0);

        state = save_usb_int();
        usb_endpoints[0].flags.tx_busy = 0;
        restore_usb_int(state);
}

void hw_usb_ep_stall(uint8 ep_nr)
{
        volatile uint16* epc = ep_regs[ep_nr].epc;
        uint8 state;

        state = save_usb_int();
        *epc |= EPC_STALL;
        restore_usb_int(state);
}

void hw_usb_ep_unstall(uint8 ep_nr)
{
        ep_data_type* ep = &usb_endpoints[ep_nr];
        volatile uint16* epc = ep_regs[ep_nr].epc;
        uint8 state;

        state = save_usb_int();
        *epc &= ~EPC_STALL;
        ep->flags.toggle = 0;
        restore_usb_int(state);
}

bool hw_usb_ep_is_stalled(uint8 ep_nr)
{
        return (bool) (*(ep_regs[ep_nr].epc) & EPC_STALL);
}

void hw_usb_ep_rx_enable(uint8 ep_nr)
{
        ep_data_type* ep = &usb_endpoints[ep_nr];
        volatile uint16* rxc = ep_regs[ep_nr].rxc;
        uint8 state;

        state = save_usb_int();
        if (ep->rx.max_size == 0) {
                if (ep_nr != USB_EP_DEFAULT) {
                        *rxc |= RXC_IGN_SETUP;
                }
                *rxc |= RXC_RX_EN;
        }
        restore_usb_int(state);
}

void hw_usb_ep_tx_start(uint8 ep_nr, uint8* buffer, uint16 size)
{
        ep_data_type* ep = &usb_endpoints[ep_nr];
        uint8 state;

        state = save_usb_int();

        //noise can cause loss of ACK/NAK IRQ for previous SETUP response, so avoid ASSERT for EP0
        if(ep_nr==0)
                ep->flags.tx_busy = 0;

        /* Previous USB-TX is not complete. Application code must wait until the TX is complete. before sending another buffer to USB
         upon USB-TX operation completion, the AppUSBTxDataDone(...) is called. Then the AppUSBTxData(...) is safe to be called for the next buffer.
         To call multiple concurrent USB-TX use queues for the buffers to avoid overwriting. The current implementation is single buffer.*/
        ASSERT_WARNING(ep->flags.tx_busy == 0);
        ep->tx.max_size = size;
        ep->tx.actual_size = 0;
        ep->tx.buffer = buffer;
        tx_fill(ep_nr);
        ep->flags.tx_busy = 1;

        restore_usb_int(state);
}

void hw_usb_ep_disable(uint8 ep_nr, bool clearToggle)
{
        ep_data_type* ep = &usb_endpoints[ep_nr];
        const ep_regs_type* er = &ep_regs[ep_nr];
        uint8 state;

        state = save_usb_int();

        if (er->txc) {
                *(er->txc) &= ~TXC_TX_EN;
                *(er->txc) |= TXC_FLUSH;
                if (*(er->txs)) {
                        *(er->txs) = 0;
                }
                tx_done(ep_nr, ep);
        }

        if (er->rxc) {
                *(er->rxc) &= ~RXC_RX_EN;
                *(er->rxc) |= RXC_FLUSH;
                if (*(er->rxs)) {
                        *(er->rxs) = 0;
                }
                rx_done(ep_nr, ep);
        }

        if (clearToggle) {
                ep->flags.toggle = 0;
        }

        restore_usb_int(state);
}

/**
 * \brief Endpoint NAK control. Default enabled for EP0.
 *
 * \param[in] ep_nr The endpoint number.
 * \param[in] enable Set to true to generate NAK for the endpoint.
 *
 */
void ud_ep_set_nak(uint8 ep_nr, bool enable)
{
        uint8 state;

        if (ep_nr == USB_EP_DEFAULT) {
                state = save_usb_int();

                if (enable) {
                        REG_SET_BIT(USB, USB_MAMSK_REG, USB_M_EP0_NAK);
                }
                else {
                        REG_CLR_BIT(USB, USB_MAMSK_REG, USB_M_EP0_NAK);
                }

                restore_usb_int(state);
        }
        else {
                uint16 mask = 0x0101 << (ep_nr - 1);

                state = save_usb_int();

                if (enable) {
                        USB->USB_NAKMSK_REG |= mask;
                }
                else {
                        USB->USB_NAKMSK_REG &= (~mask);
                }

                restore_usb_int(state);
        }
}

void hw_usb_interrupt_handler(void)
{
        uint16 maev;

        maev = USB->USB_MAEV_REG;
        maev &= USB->USB_MAMSK_REG;

        if (maev & USB_USB_MAEV_REG_USB_ALT_Msk) {
                uint8 altev;

                altev = USB->USB_ALTEV_REG;
                altev &= USB->USB_ALTMSK_REG;

                if (altev & USB_USB_ALTEV_REG_USB_SD3_Msk) {
                        ud_stat.sd3++;
                        sd3_event();
                }

                if (altev & USB_USB_ALTEV_REG_USB_SD5_Msk) {
                        ud_stat.sd5++;
                        sd5_event();
                }

                if (altev & USB_USB_ALTEV_REG_USB_RESET_Msk) {
                        ud_stat.reset++;
                        reset_event();
                }

                if (altev & USB_USB_ALTEV_REG_USB_RESUME_Msk) {
                        ud_stat.resume++;
                        resume_event();
                }
        }

        if (maev & USB_USB_MAEV_REG_USB_FRAME_Msk) {
                frame_event();
        }

        if (maev & USB_USB_MAEV_REG_USB_NAK_Msk) {
                ud_stat.nak++;
                nak_event();
        }

        if (maev & USB_USB_MAEV_REG_USB_TX_EV_Msk) {
                ud_stat.tx_ev++;
                tx_event();
        }

        if (maev & USB_USB_MAEV_REG_USB_RX_EV_Msk) {
                ud_stat.rx_ev++;
                rx_event();
        }

        if (maev & USB_USB_MAEV_REG_USB_EP0_NAK_Msk) {
                ud_stat.nak0++;
                nak_event_ep0();
        }

        if (maev & USB_USB_MAEV_REG_USB_EP0_TX_Msk) {
                ud_stat.tx_ev0++;
                tx_ep(0);
        }

        if (maev & USB_USB_MAEV_REG_USB_EP0_RX_Msk) {
                ud_stat.rx_ev0++;
                rx_ep0();
        }

        if ((dg_configUSE_USB == 1) && (dg_configUSE_USB_ENUMERATION == 1)) {
                if (maev & USB_USB_MAEV_REG_USB_CH_EV_Msk) {
                        uint16_t status = hw_charger_get_status();

                        if (hw_charger_usb_cb) {
                                hw_charger_usb_cb(status);
                        }
                }
        }
}

void hw_usb_init(void)
{
        hw_charger_setclk_pll();

        memset(usb_endpoints, 0, sizeof(usb_endpoints));
        // Power up USB hardware.
        REG_SET_BIT(USB, USB_MCTRL_REG, USBEN);

        // Configure interrupt sources.
        REG_SETF(USB, USB_TXMSK_REG, USB_M_TXFIFO31, 0x7);
        REG_SETF(USB, USB_RXMSK_REG, USB_M_RXFIFO31, 0x7);

        uint32_t reg = USB->USB_MAMSK_REG;
        REG_SET_FIELD(USB, USB_MAMSK_REG, USB_M_EP0_NAK, reg, 1);
        REG_SET_FIELD(USB, USB_MAMSK_REG, USB_M_EP0_RX, reg, 1);
        REG_SET_FIELD(USB, USB_MAMSK_REG, USB_M_EP0_TX, reg, 1);
        REG_SET_FIELD(USB, USB_MAMSK_REG, USB_M_RX_EV, reg, 1);
        REG_SET_FIELD(USB, USB_MAMSK_REG, USB_M_ULD, reg, 1);
        REG_SET_FIELD(USB, USB_MAMSK_REG, USB_M_NAK, reg, 1);
        REG_SET_FIELD(USB, USB_MAMSK_REG, USB_M_FRAME, reg, 1);
        REG_SET_FIELD(USB, USB_MAMSK_REG, USB_M_TX_EV, reg, 1);
        REG_SET_FIELD(USB, USB_MAMSK_REG, USB_M_ALT, reg, 1);
        USB->USB_MAMSK_REG = reg;
}

void hw_usb_disable(void) {
        hw_usb_disable_interrupt();
        REG_CLR_BIT(USB, USB_MCTRL_REG, USBEN);
}

#endif  /* dg_configUSE_HW_USB */

// End of file.
