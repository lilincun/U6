/**
 ****************************************************************************************
 *
 * @file llc_task.h
 *
 * @brief LLM task header file
 *
 * Copyright (C) RivieraWaves 2009-2014
 *
 *
 ****************************************************************************************
 */

#ifndef LLC_TASK_H_
#define LLC_TASK_H_

/**
 ****************************************************************************************
 * @addtogroup LLCTASK LLCTASK
 * @ingroup LLC
 * @brief Link Layer Controller Task
 *
 * The LLC task is responsible for managing link layer actions related to a
 * specific connection with a peer (e.g. Encryption setup, Supervision timeout, etc.). It
 * implements the state machine related to these actions.
 *
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <stdint.h>
#include "co_bt.h"
#include "co_version.h"
#include "ke_task.h"

#if (BLE_PERIPHERAL || BLE_CENTRAL)
/*
 * INSTANCES
 ****************************************************************************************
 */
/// Maximum number of instances of the LLC task
#define LLC_IDX_MAX  BLE_CONNECTION_MAX

/*
 * STATES
 ****************************************************************************************
 */


/// Operation type
enum llc_op_type
{
    /// Parameters update operation
    LLC_OP_PARAM_UPD         = 0x00,

    /// Max number of operations
    LLC_OP_MAX
};

/// Possible states of the LLC task
enum llc_state_id
{
    /// IDLE state
    LLC_FREE,
    /// CONNECTED state
    LLC_CONNECTED,
    /// Wait for feature response
    LLC_FEAT_WAIT_RSP,
    /// Wait for peer version indication (when we initiated the procedure)
    LLC_VERS_WAIT_IND,
    /// Wait for completion of connection parameter update procedure
    LLC_CON_UPD_WAIT_INSTANT,
    /// Wait for completion of channel map update procedure
    LLC_MAP_UPD_WAIT_INSTANT,
    /// Wait for acknowledgment of terminate indication
    LLC_TERM_WAIT_ACK,
    /// Wait for completion of data traffic before encryption procedure
    LLC_ENC_PAUSING_TRAFFIC,
    /// Data traffic is now stopped, encryption procedure can start
    LLC_ENC_TRAFFIC_PAUSED,
    /// Wait for encryption response
    LLC_ENC_WAIT_RSP,
    /// Wait for start encryption request
    LLC_ENC_WAIT_START_REQ,
    /// Wait for start encryption response
    LLC_ENC_WAIT_START_RSP,
    /// Wait for Session Key generated by HW AES engine
    LLC_ENC_WAIT_SK,
    /// Wait for LTK from host
    LLC_ENC_WAIT_LTK,
    /// Encryption pause procedure is ongoing
    LLC_ENC_WAIT_PAUSE_RSP,
    /// Encryption pause procedure is complete
    LLC_ENC_PAUSED,
    /// Wait for encryption request (when a encryption pause has been completed)
    LLC_ENC_WAIT_REQ,
    /// PAUSED state
    LLC_PAUSED,
    ///WAIT Session key
    LLC_WAIT_SK,
    /// DISCONNECTING state
    LLC_DISC,
    /// STOPPING state
    LLC_STOPPING,
    /// WAIT_ACK state
    LLCP_WAIT_ACK,

    /// Wait for the peer response to parameter request
    LLC_CON_PARAM_REQ_WAIT_RSP,
    /// Wait for the peer response to parameter response
    LLC_CON_PARAM_RSP_WAIT_RSP,
    /// Wait for the host response to parameter request
    LLC_CON_PARAM_REQ_WAIT_HOST_RSP,

#if RWBLE_SW_VERSION_MAJOR >= 8
    /// Wait for the LLC_LENGTH_RSP
    LLC_LENGTH_RSP_WAIT,
#endif
    /// Number of states.
    LLC_STATE_MAX
};


/*
 * MESSAGES
 ****************************************************************************************
 */
/// Message API of the LLC task
enum LLC_MSG
{
    /*
     * ************** Msg HCI->LLC****************
     */
    ///Data packet from driver
    LLC_DATA_IND = KE_FIRST_MSG(TASK_ID_LLC),

    /*
     * ************** Msg LLC->LLC****************
     */
    LLC_LE_LINK_SUP_TO,
    LLC_LLCP_RSP_TO,
    LLC_LLCP_UNKNOWN_IND,
    LLC_LLCP_TX_CFM,
    LLC_VERSION_IND_SEND,
    LLC_UNKNOWN_RSP_SEND,
    LLC_AUTH_PAYL_NEARLY_TO,
    LLC_AUTH_PAYL_REAL_TO,

    /*
     * ************** LLCP messages **************
     */
    /// Connection update request
    LLCP_CONNECTION_UPDATE_REQ,
    /// Channel map request
    LLCP_CHANNEL_MAP_REQ,
    /// Termination indication
    LLCP_TERMINATE_IND,
    /// Encryption request
    LLCP_ENC_REQ,
    /// Encryption response
    LLCP_ENC_RSP,
    /// Start encryption request
    LLCP_START_ENC_REQ,
    /// Start encryption response
    LLCP_START_ENC_RSP,
    /// Unknown response
    LLCP_UNKNOWN_RSP,
    /// Feature request
    LLCP_FEATURE_REQ,
    /// Feature response
    LLCP_FEATURE_RSP,
    /// Pause encryption request
    LLCP_PAUSE_ENC_REQ,
    /// Pause encryption response
    LLCP_PAUSE_ENC_RSP,
    /// Version indication
    LLCP_VERSION_IND,
    /// Reject indication
    LLCP_REJECT_IND,
    /// Slave feature request
    LLCP_SLAVE_FEATURE_REQ,
    /// Connection parameters request
    LLCP_CONNECTION_PARAM_REQ,
    /// Connection parameters response
    LLCP_CONNECTION_PARAM_RSP,
    /// Reject indication extended
    LLCP_REJECT_IND_EXT,
    /// Ping request
    LLCP_PING_REQ,
    /// Ping response
    LLCP_PING_RSP,
#if RWBLE_SW_VERSION_MAJOR >= 8
    /// Length request
    LLCP_LENGTH_REQ,
    /// Length response
    LLCP_LENGTH_RSP
#endif
};

/*
 * ************** Local Defines****************
 */
/// type of tx power level
enum
{
    TX_LVL_CURRENT,
    TX_LVL_MAX,
    TX_LVL_LEN
};

/// different control state for the LLC
enum
{
    LLC_CNTL_STATE_IDLE,
    LLC_ENC_PAUSE_RESUME,
    LLC_ENC_START,
    LLC_UPDATE_CNX,
    LLC_CNTL_STATE_LEN

};
/*
 * ************** API low energy ****************
 */


/// llc le create connection request parameters structure description.
struct llc_create_con_req
{
    ///Connection handle
    uint16_t       conhdl;
    ///Scan interval
    uint16_t       scan_intv;
    ///Scan window size
    uint16_t       scan_window;
    ///Initiator filter policy
    uint8_t        init_filt_policy;
    ///Peer address type - 0=public/1=random
    uint8_t        peer_addr_type;
    ///Peer BD address
    struct bd_addr peer_addr;
    ///Own address type - 0=public/1=random
    uint8_t        own_addr_type;
    ///Minimum of connection interval
    uint16_t       con_intv_min;
    ///Maximum of connection interval
    uint16_t       con_intv_max;
    ///Connection latency
    uint16_t       con_latency;
    ///Link supervision timeout
    uint16_t       superv_to;
    ///Minimum CE length
    uint16_t       ce_len_min;
    ///Maximum CE length
    uint16_t       ce_len_max;
};

/// llc le create connection confirmation parameters structure description.
struct  llc_create_con_cfm
{
    /// status
    uint8_t         status;
    /// connection handle
    uint16_t        conhdl;
};

/// llc le create connection indication and cfm structure description.
struct  llc_create_con_req_ind
{
    /// connection interval
    uint16_t        con_int;
    /// connection latency
    uint16_t        con_lat;
    /// supervision time out
    uint16_t        sup_to;
    /// peer bd address
    struct bd_addr  peer_addr;
    /// peer bd address type
    uint8_t         peer_addr_type;
    /// hopping
    uint8_t         hop_inc;
    /// sleep accuracy
    uint8_t         sleep_clk_acc;
#if RWBLE_SW_VERSION_MAJOR >= 8
    /// Pointer on the rx descriptor
    struct co_buf_rx_desc *rx_desc;
#endif
};

/// llc data rx packet structure
struct llc_data_ind
{
    /// connection handle
    uint16_t    conhdl;
    /// broadcast and packet boundary flag
    uint8_t     pb_bc_flag;
    /// length of the data
    uint16_t    length;
    /// Handle of the descriptor containing RX Data
    uint8_t     rx_hdl;
};

///llc read rssi command parameters structure
struct llc_llcp_unknown_ind
{
    /// Unknown opcode
    uint8_t opcode;
};

///llc read rssi command parameters structure
struct llc_unknown_rsp_send
{
    /// Unknown opcode
    uint8_t opcode;
};

/*
 * ************** API LLCP ****************
 */

/// LL_VERS_IND structure
struct llcp_vers_ind
{
    /// opcode
    uint8_t     opcode;
    /// version
    uint8_t     vers;
    /// company id
    uint16_t    compid;
    /// sub version
    uint16_t    subvers;
};

/// LL_CONNECTION_UPDATE_REQ structure.
struct  llcp_con_up_req
{
    /// opcode
    uint8_t         opcode;
    /// window size
    uint8_t         win_size;
    /// window offset
    uint16_t        win_off;
    /// interval
    uint16_t        interv;
    /// latency
    uint16_t        latency;
    /// timeout
    uint16_t        timeout;
    /// instant
    uint16_t        instant;
};

/// LL_CONNECTION_PARAM_REQ structure.
struct  llcp_con_param_req
{
    /// opcode
    uint8_t         opcode;
    /// minimum value of connInterval
    uint16_t         interval_min;
    /// maximum value of connInterval
    uint16_t         interval_max;
    /// connSlaveLatency value
    uint16_t        latency;
    /// connSupervisionTimeout value
    uint16_t        timeout;
    /// preferred periodicity
    uint8_t        pref_period;
    /// ReferenceConnEventCount
    uint16_t        ref_con_event_count;
    /// Offset0
    uint16_t        offset0;
    /// Offset1
    uint16_t        offset1;
    /// Offset2
    uint16_t        offset2;
    /// Offset3
    uint16_t        offset3;
    /// Offset4
    uint16_t        offset4;
    /// Offset5
    uint16_t        offset5;
};

/// LL_CONNECTION_PARAM_RSP structure.
struct  llcp_con_param_rsp
{
    /// opcode
    uint8_t          opcode;
    /// minimum value of connInterval
    uint16_t         interval_min;
    /// maximum value of connInterval
    uint16_t         interval_max;
    /// connSlaveLatency value
    uint16_t         latency;
    /// connSupervisionTimeout value
    uint16_t         timeout;
    /// preferred periodicity
    uint8_t          pref_period;
    /// ReferenceConnEventCount
    uint16_t         ref_con_event_count;
    /// Offset0
    uint16_t         offset0;
    /// Offset1
    uint16_t         offset1;
    /// Offset2
    uint16_t         offset2;
    /// Offset3
    uint16_t         offset3;
    /// Offset4
    uint16_t         offset4;
    /// Offset5
    uint16_t         offset5;
};

/// LL_CHANNEL_MAP_REQ structure.
struct  llcp_channel_map_req
{
    /// opcode
    uint8_t            opcode;
    /// channel mapping
    struct le_chnl_map ch_map;
    /// instant
    uint16_t           instant;
};

/// LL_TERMINATE_IND structure.
struct  llcp_terminate_ind
{
    /// opcode
    uint8_t         opcode;
    /// termination code
    uint8_t         err_code;
};

/// LL_REJECT_IND structure.
struct  llcp_reject_ind
{
    /// opcode
    uint8_t         opcode;
    /// reject reason
    uint8_t         err_code;
};

/// LL_UNKNOWN_RSP structure.
struct  llcp_unknown_rsp
{
    /// opcode
    uint8_t         opcode;
    /// unknown type
    uint8_t         unk_type;
};
/// LL_PAUSE_ENC_REQ structure.
struct  llcp_pause_enc_req
{
    /// opcode
    uint8_t             opcode;
};

/// LL_PAUSE_ENC_RSP structure.
struct  llcp_pause_enc_rsp
{
    /// opcode
    uint8_t             opcode;
};

/// LL_START_ENC_REQ structure.
struct  llcp_start_enc_req
{
    /// opcode
    uint8_t             opcode;
};

/// LL_START_ENC_RSP structure.
struct  llcp_start_enc_rsp
{
    /// opcode
    uint8_t             opcode;
};
/// LL_ENC_REQ structure.
struct  llcp_enc_req
{
    /// opcode
    uint8_t             opcode;
    /// random value
    struct rand_nb      rand;
    /// ediv
    uint8_t             ediv[2];
    /// skdm
    struct sess_k_div_x   skdm;
    /// ivm
    struct init_vect    ivm;
};

/// LL_ENC_RSP structure.
struct  llcp_enc_rsp
{
    /// opcode
    uint8_t             opcode;
    /// skds
    struct sess_k_div_x   skds;
    /// ivs
    struct init_vect    ivs;
};

/// LL_FEATURE_REQ structure.
struct  llcp_feats_req
{
    /// opcode
    uint8_t             opcode;
    /// le features
    struct le_features  feats;
};

/// LL_FEATURE_RSP structure.
struct  llcp_feats_rsp
{
    /// opcode
    uint8_t             opcode;
    /// le features
    struct le_features  feats;
};

/// LL_SLAVE_FEATURE_REQ structure.
struct  llcp_slave_feature_req
{
    /// opcode
    uint8_t             opcode;
    /// le features
    struct le_features  feats;
};

/// LL_REJECT_IND structure.
struct  llcp_reject_ind_ext
{
    /// opcode
    uint8_t         opcode;
    /// rejected opcode
    uint8_t         rej_opcode;
    /// reject reason
    uint8_t         err_code;
};

/// LL_PING_REQ structure.
struct  llcp_ping_req
{
    /// opcode
    uint8_t             opcode;
};

/// LL_PING_RSP structure.
struct  llcp_ping_rsp
{
    /// opcode
    uint8_t             opcode;
};

#if RWBLE_SW_VERSION_MAJOR >= 8
/// LL_LENGTH_REQ structure.
struct llcp_length_req
{
    /// opcode
    uint8_t             opcode;
    uint16_t            connMaxRxOctets;
    uint16_t            connMaxRxTime;
    uint16_t            connMaxTxOctets;
    uint16_t            connMaxTxTime;
};

/// LL_LENGTH_RSP structure.
struct llcp_length_rsp
{
    /// opcode
    uint8_t             opcode;
    uint16_t            connMaxRxOctets;
    uint16_t            connMaxRxTime;
    uint16_t            connMaxTxOctets;
    uint16_t            connMaxTxTime;
};
#endif

/*
 * TASK DESCRIPTOR DECLARATIONS
 ****************************************************************************************
 */
extern const struct ke_state_handler llc_state_handler[LLC_STATE_MAX];
extern const struct ke_state_handler llc_default_handler;
extern ke_state_t llc_state[LLC_IDX_MAX];

#endif // #if (BLE_PERIPHERAL || BLE_CENTRAL)
/// @} LLCTASK

#endif // LLC_TASK_H_
