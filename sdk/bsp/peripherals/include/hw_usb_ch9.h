/*
 ****************************************************************************************
 *
 * @file hw_usb_ch9.h
 *
 * @brief Header file with USB configuration info for DA1680 USB driver.
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
#ifndef USBCH9_H
#define USBCH9_H

/*========================== Include files ==================================*/

#include <sdk_defs.h>

/*========================== Local macro definitions & typedefs =============*/

#pragma pack(push,1)

/* CONTROL REQUEST SUPPORT */

/*
 * USB directions
 *
 * This bit flag is used in endpoint descriptors' bEndpointAddress field.
 * It's also one of three fields in control requests bRequestType.
 */
#define USB_HW_DIR_OUT                  0       /* to device */
#define USB_HW_DIR_IN                   0x80    /* to host */

/*
 * USB types, the second of three bRequestType fields
 */
#define USB_TYPE_MASK                   (0x03 << 5)
#define USB_TYPE_STANDARD               (0x00 << 5)
#define USB_TYPE_CLASS                  (0x01 << 5)
#define USB_TYPE_VENDOR                 (0x02 << 5)
#define USB_TYPE_RESERVED               (0x03 << 5)

/*
 * USB recipients, the third of three bRequestType fields
 */
#define USB_RECIP_MASK                  0x1f
#define USB_RECIP_DEVICE                0x00
#define USB_RECIP_INTERFACE             0x01
#define USB_RECIP_ENDPOINT              0x02
#define USB_RECIP_OTHER                 0x03

/*
 * Standard requests, for the bRequest field of a SETUP packet.
 *
 * These are qualified by the bRequestType field, so that for example
 * TYPE_CLASS or TYPE_VENDOR specific feature flags could be retrieved
 * by a GET_STATUS request.
 */
#define USB_REQ_GET_STATUS              0x00
#define USB_REQ_CLEAR_FEATURE           0x01
#define USB_REQ_SET_FEATURE             0x03
#define USB_REQ_SET_ADDRESS             0x05
#define USB_REQ_GET_DESCRIPTOR          0x06
#define USB_REQ_SET_DESCRIPTOR          0x07
#define USB_REQ_GET_CONFIGURATION       0x08
#define USB_REQ_SET_CONFIGURATION       0x09
#define USB_REQ_GET_INTERFACE           0x0A
#define USB_REQ_SET_INTERFACE           0x0B
#define USB_REQ_SYNCH_FRAME             0x0C

/*
 * USB feature flags are written using USB_REQ_{CLEAR,SET}_FEATURE, and
 * are read as a bit array returned by USB_REQ_GET_STATUS.  (So there
 * are at most sixteen features of each type.)
 */
#define USB_DEVICE_SELF_POWERED         0  /* (read only) */
#define USB_DEVICE_REMOTE_WAKEUP        1  /* dev may initiate wakeup */
#define USB_DEVICE_TEST_MODE            2  /* (high speed only) */
#define USB_DEVICE_B_HNP_ENABLE         3  /* dev may initiate HNP */
#define USB_DEVICE_A_HNP_SUPPORT        4  /* RH port supports HNP */
#define USB_DEVICE_A_ALT_HNP_SUPPORT    5  /* other RH port does */
#define USB_DEVICE_DEBUG_MODE           6  /* (special devices only) */

#define USB_ENDPOINT_HALT               0  /* IN/OUT will STALL */


/**
 * \struct usb_control_request_type
 *
 * \brief SETUP data for a USB device control request
 *
 *        This structure is used to send control requests to a USB device.  It matches
 *        the different fields of the USB 2.0 Spec section 9.3, table 9-2.  See the
 *        USB spec for a fuller description of the different fields, and what they are
 *        used for.
 *
 *        Note that the driver for any interface can issue control requests.
 *        For most devices, interfaces don't coordinate with each other, so
 *        such requests may be made at any time.
 */
typedef struct {
        uint8 request_type;     /**< matches the USB bmRequestType field */
        uint8 request;          /**< matches the USB bRequest field */
        uint16 value;           /**< matches the USB wValue field (le16 byte order) */
        uint16 index;           /**< matches the USB wValue field (le16 byte order) */
        uint16 length;          /**< matches the USB wLength field (le16 byte order) */
} usb_control_request_type;

/*-------------------------------------------------------------------------*/

/*
 * STANDARD DESCRIPTORS ... as returned by GET_DESCRIPTOR, or
 * (rarely) accepted by SET_DESCRIPTOR.
 *
 */

/*
 * Descriptor types ... USB 2.0 spec table 9.5
 */
#define USB_DT_DEVICE                   0x01
#define USB_DT_CONFIG                   0x02
#define USB_DT_STRING                   0x03
#define USB_DT_INTERFACE                0x04
#define USB_DT_ENDPOINT                 0x05
#define USB_DT_DEVICE_QUALIFIER         0x06
#define USB_DT_OTHER_SPEED_CONFIG       0x07
#define USB_DT_INTERFACE_POWER          0x08
/* these are from a minor usb 2.0 revision (ECN) */
#define USB_DT_OTG                      0x09
#define USB_DT_DEBUG                    0x0a
#define USB_DT_INTERFACE_ASSOCIATION    0x0b

/* conventional codes for class-specific descriptors */
#define USB_DT_CS_DEVICE                0x21
#define USB_DT_CS_CONFIG                0x22
#define USB_DT_CS_STRING                0x23
#define USB_DT_CS_INTERFACE             0x24
#define USB_DT_CS_ENDPOINT              0x25

/*  Standard Endpoint Descriptor (Continued) */
#define CONTROL                         0x00
#define ISO                             0x01
#define BULK                            0x02
#define INT                             0x03

#define NO_SYNC                         (0x00 << 2)
#define ASYNC                           (0x01 << 2)
#define ADAPTIVE                        (0x02 << 2)
#define SYNC                            (0x03 << 2)

#define DATAEND                         (0x00 << 4)
#define FEEDBACK                        (0x01 << 4)
#define IMP_FEEDB                       (0x02 << 4)
#define RESERVED                        (0x03 << 4)

/**
\brief All standard descriptors have these 2 fields at the beginning 
 */
typedef struct {
        uint8  length;
        uint8  descriptor_type;
} usb_descriptor_header_type;

/*-------------------------------------------------------------------------*/

/**
\brief Device descriptor 
 */
typedef struct {
        usb_descriptor_header_type header;

        uint16 cd_usb;
        uint8  device_class;
        uint8  device_sub_class;
        uint8  device_protocol;
        uint8  max_packet_size0;
        uint16 id_vendor;
        uint16 id_product;
        uint16 cd_device;
        uint8  manufacturer;
        uint8  product;
        uint8  serial_number;
        uint8  num_configurations;
} usb_device_descriptor_type;

#define USB_DT_DEVICE_SIZE              18


/*
 * Device and/or Interface Class codes
 * as found in bDeviceClass or bInterfaceClass
 * and defined by www.usb.org documents
 */
#define USB_CLASS_PER_INTERFACE         0  /* for DeviceClass */
#define USB_CLASS_AUDIO                 1
#define USB_CLASS_COMM                  2
#define USB_CLASS_HID                   3
#define USB_CLASS_PHYSICAL              5
#define USB_CLASS_STILL_IMAGE           6
#define USB_CLASS_PRINTER               7
#define USB_CLASS_MASS_STORAGE          8
#define USB_CLASS_HUB                   9
#define USB_CLASS_CDC_DATA              0x0a
#define USB_CLASS_CSCID                 0x0b  /* chip+ smart card */
#define USB_CLASS_CONTENT_SEC           0x0d  /* content security */
#define USB_CLASS_VIDEO                 0x0e
#define USB_CLASS_APP_SPEC              0xfe
#define USB_CLASS_VENDOR_SPEC           0xff

/*-------------------------------------------------------------------------*/

/**
\brief Configuration descriptor information.
       USB_DT_OTHER_SPEED_CONFIG is the same descriptor, except that the
       descriptor type is different.  Highspeed-capable devices can look
       different depending on what speed they're currently running.  Only
       devices with a USB_DT_DEVICE_QUALIFIER have any OTHER_SPEED_CONFIG
       descriptors.
 */
typedef struct {
        usb_descriptor_header_type header;

        uint16 total_length;
        uint8  num_interfaces;
        uint8  configuration_value;
        uint8  configuration;
        uint8  attributes;
        uint8  max_power;
} usb_config_descriptor_type;

#define USB_DT_CONFIG_SIZE              9

/* from config descriptor bmAttributes */
#define USB_CONFIG_ATT_ONE              (1 << 7)  /* must be set */
#define USB_CONFIG_ATT_SELFPOWER        (1 << 6)  /* self powered */
#define USB_CONFIG_ATT_WAKEUP           (1 << 5)  /* can wakeup */

/*-------------------------------------------------------------------------*/

/**
\brief Interface descriptor 
 */
typedef struct {
        usb_descriptor_header_type header;

        uint8  interface_number;
        uint8  alternate_setting;
        uint8  num_endpoints;
        uint8  interface_class;
        uint8  interface_subclass;
        uint8  interface_protocol;
        uint8  interface;
} usb_interface_descriptor_type;

#define USB_DT_INTERFACE_SIZE    9

/*-------------------------------------------------------------------------*/

/**
\brief Endpoint descriptor 
 */
typedef struct {
        usb_descriptor_header_type header;

        uint8  endpoint_address;
        uint8  attributes;
        uint16 max_packet_size;
        uint8  interval;

        // NOTE:  these two are _only_ in audio endpoints.
        // use USB_DT_ENDPOINT*_SIZE in bLength, not sizeof.
        //uint8  bRefresh;
        //uint8  bSynchAddress;
} usb_endpoint_descriptor_type;

#define USB_DT_ENDPOINT_SIZE            7
#define USB_DT_ENDPOINT_AUDIO_SIZE      9  /* Audio extension */


/*
 * Endpoints
 */
#define USB_ENDPOINT_NUMBER_MASK        0x0f  /* in bEndpointAddress */
#define USB_ENDPOINT_DIR_MASK           0x80

#define USB_ENDPOINT_XFERTYPE_MASK      0x03  /* in bmAttributes */
#define USB_ENDPOINT_XFER_CONTROL       0
#define USB_ENDPOINT_XFER_ISOC          1
#define USB_ENDPOINT_XFER_BULK          2
#define USB_ENDPOINT_XFER_INT           3

/*-------------------------------------------------------------------------*/

/**
\brief USB device state type.
 */
typedef enum {
        /* NOTATTACHED isn't in the USB spec, and this state acts
         * the same as ATTACHED ... but it's clearer this way.
         */
        USB_STATE_NOTATTACHED = 0,

        /* the chapter 9 device states */
        USB_STATE_ATTACHED,
        USB_STATE_POWERED,
        USB_STATE_DEFAULT,              /* limited function */
        USB_STATE_ADDRESS,
        USB_STATE_CONFIGURED,           /* most functions */

        USB_STATE_SUSPENDED

        /* NOTE:  there are actually four different SUSPENDED
         * states, returning to POWERED, DEFAULT, ADDRESS, or
         * CONFIGURED respectively when SOF tokens flow again.
         */
} usb_device_state_type;

#pragma pack(pop)

/*========================== Global variabeles ==============================*/

/*========================== Global function prototypes =====================*/

#endif /* FILENAME_H  */

// End of file.

