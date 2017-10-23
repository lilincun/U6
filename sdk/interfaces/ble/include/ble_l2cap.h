/**
 \addtogroup INTERFACES
 \{
 \addtogroup BLE
 \{
 \addtogroup API
 \{
 */

/**
 ****************************************************************************************
 *
 * @file ble_l2cap.h
 *
 * @brief LE L2CAP connection oriented channels API
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

#ifndef BLE_L2CAP_H_
#define BLE_L2CAP_H_

#include <stdint.h>
#include "ble_common.h"
#include "ble_gap.h"

/** LE L2CAP connection oriented channels events */
enum ble_evt_l2cap {
        /** Channel connected */
        BLE_EVT_L2CAP_CONNECTED = BLE_EVT_CAT_FIRST(BLE_EVT_CAT_L2CAP),
        /** Channel connection failed */
        BLE_EVT_L2CAP_CONNECTION_FAILED,
        /** Channel disconnected */
        BLE_EVT_L2CAP_DISCONNECTED,
        /** Available remote credits changed on channel */
        BLE_EVT_L2CAP_REMOTE_CREDITS_CHANGED,
        /** Data received on channel */
        BLE_EVT_L2CAP_DATA_IND,
        /** Data sent on channel */
        BLE_EVT_L2CAP_SENT,
};

/** Structure for ::BLE_EVT_L2CAP_CONNECTED event */
typedef struct {
        ble_evt_hdr_t   hdr;
        uint16_t        conn_idx;       /**< connection index */
        uint16_t        psm;            /**< LE protocol/service multiplexer */
        uint16_t        scid;           /**< source CID */
        uint16_t        dcid;           /**< destination CID */
        uint16_t        local_credits;  /**< local credits available */
        uint16_t        remote_credits; /**< remote credits available */
        uint16_t        mtu;            /**< negotiated MTU */
} ble_evt_l2cap_connected_t;

/** Structure for ::BLE_EVT_L2CAP_CONNECTION_FAILED event */
typedef struct {
        ble_evt_hdr_t   hdr;
        uint16_t        conn_idx;       /**< connection index */
        uint16_t        scid;           /**< source CID */
        ble_error_t     status;         /**< operation status */
} ble_evt_l2cap_connection_failed_t;

/** Structure for ::BLE_EVT_L2CAP_DISCONNECTED event */
typedef struct {
        ble_evt_hdr_t   hdr;
        uint16_t        conn_idx;       /**< connection index */
        uint16_t        scid;           /**< source CID */
        uint16_t        reason;         /**< disconnection reason */
} ble_evt_l2cap_disconnected_t;

/** Structure for ::BLE_EVT_L2CAP_REMOTE_CREDITS_CHANGED event */
typedef struct {
        ble_evt_hdr_t   hdr;
        uint16_t        conn_idx;       /**< connection index */
        uint16_t        scid;           /**< source CID */
        uint16_t        remote_credits; /**< remote credits available */
} ble_evt_l2cap_credit_changed_t;

/** Structure for ::BLE_EVT_L2CAP_DATA_IND event */
typedef struct {
        ble_evt_hdr_t   hdr;
        uint16_t        conn_idx;       /**< connection index */
        uint16_t        scid;           /**< source CID */
        uint16_t        local_credits_consumed; /**< local credits consumed by received data */
        uint16_t        length;         /**< length of data available */
        uint8_t         data[0];        /**< received data */
} ble_evt_l2cap_data_ind_t;

/** Structure for ::BLE_EVT_L2CAP_SENT event */
typedef struct {
        ble_evt_hdr_t   hdr;
        uint16_t        conn_idx;       /**< connection index */
        uint16_t        scid;           /**< source CID */
        uint16_t        remote_credits; /**< remote credits available */
        ble_error_t     status;          /**< operation status */
} ble_evt_l2cap_sent_t;

/**
 * \brief Create a connection oriented channel listening for incoming connections
 *
 * Incoming connection will be signaled using ::BLE_EVT_L2CAP_CONNECTED event.
 *
 * \note
 * It's possible to have only single connection using given PSM (either incoming or outgoing).
 *
 * \param [in]  conn_idx        Connection index
 * \param [in]  psm             LE Protocol/Service Multiplexer
 * \param [in]  sec_level       Channel security
 * \param [in]  initial_credits Initial credits on channel
 * \param [out] scid            Source CID for created channel
 *
 * \return result code
 *
 * \sa ble_l2cap_stop_listen()
 *
 */
ble_error_t ble_l2cap_listen(uint16_t conn_idx, uint16_t psm, gap_sec_level_t sec_level,
                                                        uint16_t initial_credits, uint16_t *scid);

/**
 * \brief Stop listening for incoming connections
 *
 * \p scid should identify channel previously created with ble_l2cap_listen().
 *
* \param [in]  conn_idx         Connection index
* \param [in]  scid             Source CID for channel to stop listening on
*
* \return result code
 *
 */
ble_error_t ble_l2cap_stop_listen(uint16_t conn_idx, uint16_t scid);

/**
 * \brief Connect a connection oriented channel to remote peer
 *
 * \param [in]  conn_idx        Connection index
 * \param [in]  psm             LE Protocol/Service Multiplexer
 * \param [in]  initial_credits Initial credits on channel
 * \param [out] scid            Source CID for created channel
 *
 * \return result code
 *
 * \sa ble_l2cap_disconnect()
 *
 */
ble_error_t ble_l2cap_connect(uint16_t conn_idx, uint16_t psm, uint16_t initial_credits,
                                                                                uint16_t *scid);

/**
 * \brief Disconnect channel
 *
 * \p scid should identify channel which was received in ::BLE_EVT_L2CAP_CONNECTED event.
 *
* \param [in]  conn_idx         Connection index
* \param [in]  scid             Source CID for channel to be disconnected
*
* \return result code
 *
 */
ble_error_t ble_l2cap_disconnect(uint16_t conn_idx, uint16_t scid);

/**
 * \brief Add local credits on channel
 *
 * Application should monitor contents of \p local_credits_consumed member of ::BLE_EVT_L2CAP_DATA_IND
 * event and give back appropriate number of local credits when processing of incoming data is
 * completed.
 *
 * \param [in]  conn_idx        Connection index
 * \param [in]  scid            Source CID
 * \param [in]  credits         Number of credits to be added
 *
 * \return result code
 *
 */
ble_error_t ble_l2cap_add_credits(uint16_t conn_idx, uint16_t scid, uint16_t credits);

/**
 * \brief Send data on channel
 *
 * \param [in]  conn_idx        Connection index
 * \param [in]  scid            Source CID
 * \param [in]  length          Length of data to be sent
 * \param [in]  data            Data to be sent
 *
 * \return result code
 *
 */
ble_error_t ble_l2cap_send(uint16_t conn_idx, uint16_t scid, uint16_t length, const void *data);



/**
 * \brief Initiate a connection parameter update over L2CAP
 *
 * This call can be used only by the slave of a connection to initiate a connection parameter
 * request to the master over L2CAP, instead of using ble_gap_conn_param_update() which triggers the
 * connection parameter request procedure. Successful connection parameters update will result in a
 * ::BLE_EVT_GAP_CONN_PARAM_UPDATED event message. Unsuccessful connection parameters update
 * (connection parameters rejected by the master) will not result in any event message to the
 * application.
 *
 * \param [in]  conn_idx       Connection index
 * \param [in]  conn_params    Pointer to the connection parameters
 *
 * \return result code
 *
 * \deprecated Switch to \sa ble_gap_conn_param_update(), which has the same prototype.
 */
DEPRECATED_MSG("API no longer supported, use ble_gap_conn_param_update() instead.")
ble_error_t ble_l2cap_conn_param_update(uint16_t conn_idx, const gap_conn_params_t *conn_params);

#endif /* BLE_L2CAP_H_ */

/**
 \}
 \}
 \}
 */
