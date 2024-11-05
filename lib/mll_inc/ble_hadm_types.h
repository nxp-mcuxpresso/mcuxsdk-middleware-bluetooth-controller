/*
*                Copyright 2023, NXP
 *
 *  NXPCONFIDENTIAL
 *  The source code contained or described herein and all documents related to
 *  the source code ( Material ) are owned by NXP its
 *  suppliers or licensors. Title to the Material remains with NXP
 *  or its suppliers and licensors. The Material contains
 *  trade secrets and proprietary and confidential information of NXP its
 *  suppliers and licensors. The Material is protected by worldwide copyright
 *  and trade secret laws and treaty provisions. No part of the Material may be
 *  used, copied, reproduced, modified, published, uploaded, posted,
 *  transmitted, distributed, or disclosed in any way without NXPprior
 *  express written permission.
 *
 *  No license under any patent, copyright, trade secret or other intellectual
 *  property right is granted to or conferred upon you by disclosure or delivery
 *  of the Materials, either expressly, by implication, inducement, estoppel or
 *  otherwise. Any license under such intellectual property rights must be
 *  express and approved by NXP writing.
*/

/**
 *@file         hadm_types.h
 *
 *\brief        This header file holds HADM related types used by LL and NBU
 *
 */
#ifndef __HADM_TYPES_H_
#define __HADM_TYPES_H_

#ifdef __ICCARM__
#include "cmsis_compiler.h"
#endif

#include "ble_hadm_hal.h"

/* === Macros =============================================================== */

/* Security fields lengths */
#define CS_IV_LEN                   8
#define CS_IN_LEN                   4
#define CS_PV_LEN                   8

/* channel map lenght in bytes*/
#define BLE_HADM_CHMAP_LEN          (10U)

/* Channel list array lenght in bytes */
#define BLE_HADM_TEST_MODE_CHANLIST_LEN  (72U)

/* FAE table lenght in bytes */
#define BLE_HADM_FAE_TABLE_LEN      (72U)

/* Used to index local/remote tables */
typedef enum TBleHadmDeviceIdx_tag
{
    BLE_HADM_DEVICE_IDX_LOCAL,
    BLE_HADM_DEVICE_IDX_REMOTE,
    BLE_HADM_DEVICE_IDX_MAX
} TBleHadmDeviceIdx;

/* Input vector type for one device */
__PACKED_STRUCT TBleHadmDrbgInitVectorsIndividual_s
{
    uint8  uc_CS_IV[CS_IV_LEN];
    uint8  uc_CS_IN[CS_IN_LEN];
    uint8  uc_CS_PV[CS_PV_LEN];
};

/*
eConfigState

                      ┌────────┐ CONFIG_REMOVE
            ┌─────────┤REMOVING│◄──────────┬─────────────┐
            │         └────────┘           │             │
            │                              │             │
            │                              │             │
        ┌───▼───┐     ┌────────┐      ┌────┴─────┐   ┌───┴─────┐
        │INVALID│     │CREATING│      │READY_RESP│   │  READY  │
        └───┬───┘     └─▲───┬──┘      └───▲───┬──┘   └───▲─────┘
            │           │   │             │   │          │
            └───────────┘   └─────────────┘   └──────────┘
            CREATE_CONFIG    CONFIG_COMPLETE  SET_PROC_PARAMS



eProcState
                                                                                                     
            SET_ENABLE(0) || TERMINATE_REQ                                                           
      ┌──────────────────────┬─────────────────┐◄────────────────────────────┐                       
      │                      │                 │   Procedure_instance_done   │                       
      │                      │                 │   ┌───────────┐             │                       
      │                      │   Procedure done│   │           │             │                       
  ┌───▼────┐  ┌────────┐   ┌─┴───────┐      ┌──┴───▼──┐   ┌────┴─────┐   ┌───┴───────┐               
  │DISABLED│  │STARTING│   │ ENABLED │      │ ACTIVE  │   │ RUNNING  │   │TERMINATING│               
  └───┬────┘  └─▲───▲─┬┘   └─▲─────┬─┘      └──▲───┬──┘   └────▲──┬──┘   └───▲───────┘               
      │         │   │ │      │     │           │   │           │  │          │                       
      ├─────────┘   │ └──────┘     └───────────┘   └───────────┘  └──────────┘                       
      │ READY       │  SET_ENABLE   Time to        Scheduled       SET_ENABLE(0) || TERMINATE_REQ    
      │&& SET_ENABLE│  Complete     run                                                              
      │             │                                                                                
      └─────────────┘                                                                                
       READY_RESPONDER                                                                               
       && CS_REQ                                                                                     
                                                                                                     
*/

typedef enum TBleHadmConfigState_tag
{
    BLE_HADM_CFGSTATE_INVALID,          /* Pool entry is free */
    BLE_HADM_CFGSTATE_CREATING,         /* Create Configuration procedure in progress */
    BLE_HADM_CFGSTATE_REMOVING,         /* Remove Configuration procedure in progress */
    BLE_HADM_CFGSTATE_READY_RESPONDER,  /* Configuration is ready to operate as responder (can handle CS_REQ) */
    BLE_HADM_CFGSTATE_READY,            /* Configuration is ready to operate as requestor or responder */
} TBleHadmConfigState_t;

typedef enum TBleHadmProcState_tag
{
    BLE_HADM_PROCSTATE_DISABLED,    /* No procedure is running for this config */
    BLE_HADM_PROCSTATE_STARTING,    /* Start Procedure in progress */
    BLE_HADM_PROCSTATE_ENABLED,     /* Start procedure (enabled=1) is complete */
    BLE_HADM_PROCSTATE_ACTIVE,      /* The configuration is the one being served on the connection. Ready to be scheduled. */
    BLE_HADM_PROCSTATE_RUNNING,     /* BLE_LM_HADM_StartProcedureInstance has been called */
    BLE_HADM_PROCSTATE_TERMINATING, /* The procedure repetition is in the process of being stopped (completes current instance) */
} TBleHadmProcState_t;

typedef enum TBleHadmConnectionState_tag
{
    BLE_HADM_CONNSTATE_INVALID,     /* Pool entry is free */
    BLE_HADM_CONNSTATE_VALID,       /* Pool entry is being used */
    BLE_HADM_CONNSTATE_READY,       /* CS connection setup done (doesn't reflect CS config state) */
} TBleHadmConnectionState_t;

/* LL HADM CAPABILITIES REQ / HADM CAPABILITIES RSP message */
typedef __PACKED_STRUCT TBleHadmLLCapabilities_tag
{
    //@CS_Spec #LL_CS_CAPABILITIES_REQ #LL_CS_CAPABILITIES_RSP
    uint8                 ucModeTypes;                      //@CS_Spec #Mode_Types
    uint8                 ucRTTcapability;                  //@CS_Spec #RTT_Capability
    uint8                 ucRTTaaOnlyN;                     //@CS_Spec #RTT_AA_Only_N
    uint8                 ucRTTsoundingN;                   //@CS_Spec #RTT_Sounding_N
    uint8                 ucRTTrandomSequenceN;             //@CS_Spec #RTT_Random_Sequence_N
    uint16                uiNADMsoundingCapability;         //@CS_Spec #NADM_Sounding_Capability
    uint16                uiNADMrandomSequenceCapability;   //@CS_Spec #NADM_Random_Sequence_Capability
    uint8                 ucCSsYNCpHY;                      //@CS_Spec #CS_SYNC_PHY_Capability
    uint8                 ucNumAnt:4;                       //@CS_Spec #Num_Ant
    uint8                 ucMax_Antenna_Path:4;             //@CS_Spec #Max_Ant_Path
    uint8                 ucRole:2;                         //@CS_Spec #Role
#if defined (SUPPORT_INLINE_PHASE_RETURN)
    uint8                 ucInlinePhaseReturn:1;            //@CS_Spec #InlinePhaseReturn
#else
    uint8                 ucRFU0:1;                         //@CS_Spec #RFU
#endif
    uint8                 ucNoFAE:1;                        //@CS_Spec #No_FAE
    uint8                 ucChannelSel3c:1;                 //@CS_Spec #ChannelSelection #3c
    uint8                 ucSoundingPhaseBasedRanging:1;    //@CS_Spec #Sounding_PCT_Estimate
    uint8                 ucRFU1:2;                         //@CS_Spec #RFU
    uint8                 ucNumConfigs;                     //@CS_Spec #Num_Configs
    uint16                ucMaxProcedures;                  //@CS_Spec #Max_Procedures_Supported
    uint8                 ucT_SW;                           //@CS_Spec #T_SW_Time_Supported
    uint16                uiT_IP1_Capability;               //@CS_Spec #T_IP1_Capability
    uint16                uiT_IP2_Capability;               //@CS_Spec #T_IP2_Capability
    uint16                uiT_FCS_Capability;               //@CS_Spec #T_FCS_Capability
    uint16                uiT_PM_Capability;                //@CS_Spec #T_PM_Capability
    uint8                 ucRFU2:1;                         //@CS_Spec #RFU
    uint8                 ucTX_SNR:7;                       //@CS_Spec #TX_SNR_Capability

} TBleHadmLLCapabilities_t;

typedef struct TBleHadmConfigParams_tag
{
    uint8   ucMain_Mode_Type;                       //@CS_Spec #LL_CS_CONFIG_REQ #Main_Mode
    uint8   ucSub_Mode_Type;                        //@CS_Spec #LL_CS_CONFIG_REQ #Sub_Mode
    uint8   ucMain_Mode_Min_Steps;                  //@CS_Spec #LL_CS_CONFIG_REQ #ucMain_Mode_Min_Steps
    uint8   ucMain_Mode_Max_Steps;                  //@CS_Spec #LL_CS_CONFIG_REQ #Main_Mode_Max_Steps
    uint8   ucMain_Mode_Repetition;                 //@CS_Spec #LL_CS_CONFIG_REQ #Main_Mode_Repetition
    uint8   ucMode_0_Steps;                         //@CS_Spec #LL_CS_CONFIG_REQ #Mode_0_Steps
    uint8   ucRole;                                 //@CS_Spec #LL_CS_CONFIG_REQ #Role
    uint8   ucRTT_Types;                            //@CS_Spec #LL_CS_CONFIG_REQ #RTT_Type
    uint8   ucCS_Sync_Phy; /* 1=1Mbps 2=2Mbps */    //@CS_Spec #LL_CS_CONFIG_REQ #CS_SYNC_PHY
    uint8   ucT_IP1;                                //@CS_Spec #LL_CS_CONFIG_REQ #T_IP1
    uint8   ucT_IP2;                                //@CS_Spec #LL_CS_CONFIG_REQ #T_IP1
    uint8   ucT_FCS;                                //@CS_Spec #LL_CS_CONFIG_REQ #T_FCS
    uint8   ucT_PM;                                 //@CS_Spec #LL_CS_CONFIG_REQ #T_PM
    uint8   aucChannel_Map[BLE_HADM_CHMAP_LEN];     //@CS_Spec #LL_CS_CONFIG_REQ #ChM
    uint8   ucChannel_Map_Repetition;               //@CS_Spec #LL_CS_CONFIG_REQ #ChM_Repetition
    uint8   ucChannel_Selection_Type;               //@CS_Spec #LL_CS_CONFIG_REQ #ChSel
    uint8   ucCh3c_Shape;                           //@CS_Spec #LL_CS_CONFIG_REQ #Ch3cShape
    uint8   ucCh3c_Jump;                            //@CS_Spec #LL_CS_CONFIG_REQ #Ch3cJump
    uint8   ucReserved;                             //@CS_Spec #LL_CS_CONFIG_REQ #Reserved
    // uint8   ucRTT_Antenna_Selection;
} TBleHadmConfigParams_t;

//@CS_Spec #LL_CS_CONFIG_REQ
typedef struct TBleHadmConfiguration_tag
{
    uint16                  uiConnHandle;
    uint8                   ucConfigId;
    TBleHadmConfigState_t   eConfigState;    /* State of the configuration */
    TBleHadmProcState_t     eProcState;      /* State of the CS procedure running on thuis configuration */
    uint16                  uiFlags;         /* Utility flags for runtime operation */

    /* Parameters from CREATE_CONFIG */
    TBleHadmConfigParams_t  params;
    TBleHadmConfigParams_t  pendingParams;   /* the existing config is being modified by Host stack. New parameters are stored temporarily here */
    TBleHadmConfigState_t   ePrevConfigState;/* the current state of the config which is being changed or removed */

    /* parameters received via HCI Procedure Set Parameters */
    uint16  uiMaxProcDur;
    uint16  uiMinProcInt;
    uint16  uiMaxProcInt;
    uint32  ulMinSubeventLen;
    uint32  ulMaxSubeventLen;

    /* parameters received via HCI Procedure Set Parameters or CS_REQ */
    uint16  uiProcedureCnt;                         //@CS_Spec #LL_CS_REQ #Procedure_Count
    uint8   ucToneAntCfgSel;                        //@CS_Spec #LL_CS_IND #ACI
    int8    ucPhy;                                  //@CS_Spec #LL_CS_IND #PHY
    int8    cTxPowerDelta;                          //@CS_Spec #LE_CS_SET_PROCEDURE_PARAMS #Tx_Power_Delta
    int8    cPwrDelta;                              //@CS_Spec #LL_CS_REQ/LL_CS_RSP/LL_CS_IND #Pwr_Delta
    uint8   ucPreferred_Peer_Antennas;              //@CS_Spec #LL_CS_REQ #Preferred_Peer_Antennas
    uint8   ucSNR_Control_Initiator;                //@CS_Spec #LL_CS_REQ #TX_SNR_I
    uint8   ucSNR_Control_Reflector;                //@CS_Spec #LL_CS_REQ #TX_SNR_R

    /* Parameters passed to LL_CS_REQ */
    //@CS_Spec #LL_CS_REQ #Procedure_Interval
    uint16  uiProcedureInterval;    /* Number of ACL connection events between consecutive CS procedure anchor points, chosen by LL internally */

    /* Negociated parameters during LL_CS_REQ/RSP/IND roundtrip */
    uint32  ulOffsetMin;            /* Initially chosen by LL internally */
    uint32  ulOffsetMax;            /* Initially chosen by LL internally */
    //@CS_Spec #LL_CS_IND #connEventCount, #Offset
    uint32  ulStartOffset;          /* HADM event start offset in us from the anchor point, chosen by LL internally. Has to be lie in ulOffsetMin, ulOffsetMax negotiated range  */
    uint16  uiConnEventCount;       /* HADM event start event counter, initially chosen by LL internally */

    //@CS_Spec #LL_CS_IND #Event_Interval, #Subevents_Per_Event, #Subevent_Interval, #Subevent_Len
    uint16  uiEventInterval;        /* number of ACL connection events between consecutive CS event anchor points. Initially chosen by LL internally */
    uint16  uiSubeventInterval;     /* interval(625 us) between the start of two consecutive CS subevents of the same CS event, initially chosen by LL internally */
    uint32  ulSubeventLen;          /* max duration(us) of sub-event (T_SUBEVENT_LEN) [1250us,4s], initially chosen by LL in HCI range ulMinSubeventLen , ulMaxSubeventLen*/
    uint8   ucSubeventsPerEvent;    /* number of CS subevents that anchored off of the same LE anchor point initially chosen by LL internally */

    /* Derived configuration parameters */
    int8    cSelectedTxPower;       /* Derived from Tx_Power_Delta in CS_REQ procedure or from HCI */
    uint8   ucT_SW_time;            /* Derived from ACI in CS_REQ procedure or ucToneAntCfgSel from HCI */

} TBleHadmConfiguration_t;


typedef struct TBleHadmEvent_ 
{
    BLE_HADM_SubeventConfig_t      *config_p; /* pointer to HAL config buffer allocated for this subevent */
#if (HADM_HAL_VERSION == 1)
    BLE_HADM_SubeventResultsData_t *result_p; /* pointer to HAL config buffer provided for this subevent */
    BLE_HADM_STATUS_t status; /* Status of HAL measurement */
#endif

    /* Scheduling data */
    uint16 uiConnHandle;
    uint16 uiHadmHandle;      /* Handle for HADM link in the scheduler */
    uint8  ucConfigId;
    uint8  ucFlags;           /* Utility flags for runtime optimization */
    uint16 uiStartConnEvCnt;  /* ACL connection event counter at which the subevent is started */
    uint32 ulStartOffsetUs;   /* Offset to ACL anchor point [us] */
    uint32 ulActivityLen;     /* Length of subevent including pre-post preparation [us] */
    uint16 uiHalPrepareTimeUs;/* Time needed to HAL for event preparation before actual start of event */

    /* Buffering data */
    boolean pendingData;      /* Some data is waiting to be flushed via HCI */
    uint8 nextStepToReport;   /* Next stepId to report to HCI via SubeventResultEvent */
    uint8 lastStepToReport;   /* Last stepId available for reporting */
    uint16 subeventSteps;     /* Number of steps in the subevent */
} TBleHadmEvent;

typedef struct TBleHadmConnection_tag
{
    uint16  uiConnHandle;
    TBleHadmConnectionState_t eConnState;
    uint16  uiFlags;                        /*!<< Utility flags for runtime operation */
    uint8   ucStartProcedureConfigId;       /* Tracks configId of the current Start Procedure */
    uint8   ucTmpStatus;                    /* Placeholder to store current LLCP procedure status */

    /* Capabilities */
    //@CS_Spec #LL_CS_CAPABILITIES_REQ #LL_CS_CAPABILITIES_RSP
    TBleHadmLLCapabilities_t remoteCapabilities;

    /* Security */
    //@CS_Spec #LL_CS_SEC_REQ #LL_CS_SEC_RSP #CS_IV_C #CS_IN_C #CS_PV_C #CS_IV_P #CS_IN_P #CS_PV_P
    struct TBleHadmDrbgInitVectorsIndividual_s localIV;
    struct TBleHadmDrbgInitVectorsIndividual_s peerIV;

    /* Default settings */
    uint8  ucRoleEnable;
    uint8  ucSyncAntennaSelection;
    int8   cMaxTxPower;

   /* Channel Map update procedure */
   //@CS_Spec #LL_CS_CHANNEL_MAP_IND #ChM #Instant
    uint8  ucChMapActiveIdx[BLE_HADM_DEVICE_IDX_MAX];
    uint16 uiChMapInstant[BLE_HADM_DEVICE_IDX_MAX];
    uint8  ucChMapUpd[BLE_HADM_DEVICE_IDX_MAX][2U][BLE_HADM_CHMAP_LEN];

    /* FAE Table (Frequency Actuation Error) */
    uint8  ucFaeTable[BLE_HADM_FAE_TABLE_LEN]; //@CS_Spec #LL_CS_FAE_RSP #ChFAE

    /* Running CS procedure */
    TBleHadmConfiguration_t *pActiveConfig;     /* NULL when no active/running procedure */
    uint16  ulRepIdx;                           /* Index of procedure for repeat count down */
    uint8   ucSubeventIdx;                      /* Index of current subevent within the procedure */
    int16  iProcCountTerminateDiff;                      /* Peer ProcCount minus Local ProcCount stored during termination procedure */

    TBleHadmEvent subevent;                     /* Current subevent data */
#if defined(BT60_HADM)
    TBleHadmDrbgContext_t drbgContext;
#endif
} TBleHadmConnection_t;

typedef struct TBleHadmTestModeOverrides_tag
{
    uint16 usOverrideConfig;
    uint8 ucT_PM_ToneExt;
    uint8 ucToneAntennaPermutation;
    uint32 ulCS_SYNC_AA_Initiator;
    uint32 ulCS_SYNC_AA_Reflector;
    uint8 ucCS_SYNC_User_Payload[16U];
} TBleHadmTestModeOverrides_t;

#endif // #ifndef __HADM_TYPES_H_
