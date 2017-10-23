/**
 ****************************************************************************************
 *
 * @file sample.h
 *
 * @brief Serial Port Service sample implementation API
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

#ifndef SAMPLE_H_
#define SAMPLE_H_

#include "ble_service.h"

/**
 * SAMPLE Flow Control flags values
 */
typedef enum {
        SAMPLE_FLOW_CONTROL_ON = 0x01,
        SAMPLE_FLOW_CONTROL_OFF = 0x02,
} sample_flow_control_t;

typedef void (* sample_set_flow_control_cb_t) (ble_service_t *svc, uint16_t conn_idx, sample_flow_control_t value);

typedef void (* sample_rx_data_cb_t) (ble_service_t *svc, uint16_t conn_idx, const uint8_t *value,
                                                                                uint16_t length);

typedef void (* sample_tx_done_cb_t) (ble_service_t *svc, uint16_t conn_idx, uint16_t length);

/**
 * SAMPLE application callbacks
 */
typedef struct {
        /** Remote client wrote new value of flow control characteristic */
        sample_set_flow_control_cb_t set_flow_control;
        /** Data received from remote client */
        sample_rx_data_cb_t          rx_data;
        /** Service finished TX transaction */
        sample_tx_done_cb_t          tx_done;
} sample_callbacks_t;

/**
 * \brief Register Serial Port Service instance
 *
 * Function registers SAMPLE instance
 *
 * \param [in] cb               application callbacks
 *
 * \return service instance
 *
 */
ble_service_t *sample_init(sample_callbacks_t *cb);

/**
 * \brief Set flow control value
 *
 * Function updates flow control value.
 *
 * \param [in] svc              service instance
 * \param [in] conn_idx         connection index
 * \param [in] value            flow control value
 *
 */
void sample_set_flow_control(ble_service_t *svc, uint16_t conn_idx, sample_flow_control_t value);

/**
 * \brief TX data available
 *
 * Function notifies new data is available for client. After sending data, service
 * will call tx_done callback.
 *
 * \param [in] svc              service instance
 * \param [in] conn_idx         connection index
 * \param [in] data             tx data
 * \param [in] length           tx data length
 *
 */
void sample_tx_data(ble_service_t *svc, uint16_t conn_idx, uint8_t *data, uint16_t length);

#endif /* SAMPLE_H_ */
