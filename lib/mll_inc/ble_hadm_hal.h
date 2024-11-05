/*
 * Copyright 2021-2022 NXP
 * All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 */
/*
 * This file is an abstract layer API that must be implemented by each
 * hardware platform.
 * Be carefull not using any environement-specific types such as uint8_t or uint8
 * as this file is shared bw C&S LL code and SDK code.
 */

/* Prevent double inclusion */
#ifndef _BLE_HADM_HAL_H_
#define _BLE_HADM_HAL_H_

#if (defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN == 470)) || (defined (KW47_FIX))
#define HADM_HAL_VERSION 2 /* For KW47 architecture */
#else
#define HADM_HAL_VERSION 1 /* For KW45 architecture */
#endif

/* === Includes ============================================================= */

/* === Macros =============================================================== */
/* Maximum number of CS procedure context that can be handled in parrallel by the HAL */
/* Contexts are used to store per-procedure persistent data that need to be retained between several subevents (AGC, phase consitency ...) */
#define HADM_MAX_NB_CONNECTIONS (2U)

/* Maximum number of CS subevents that can be handled in parrallel by the HAL */
#define HADM_MAX_NB_SIMULT_SUBEVENTS (2U)

#if (HADM_HAL_VERSION == 1)
#define HADM_MAX_NB_STEPS (128U) /*!< Max number of HADM steps within a subevent */
#define HADM_MAX_NB_STEPS_RTT (106U) /*!< Max number of HADM steps containing RTT packets within a subevent. RSM limitation */
#define HADM_MAX_NB_STEPS_RTT_RAND (10U) /*!< Max number of HADM steps containing RTT packets with 32 bits random payload within an subevent. To limit memory footprint impact */
#else
#define HADM_MAX_NB_STEPS (160U) /*!< Max number of HADM steps within a subevent */
#endif

/*! Min number of HADM steps within a subevent */
#define HADM_MIN_NB_STEPS (2U) 

/*! Max number of HADM steps mode 0 within a subevent*/
#define HADM_MAX_NB_STEPS_MODE0 (3U)

/*! Max number of antenna paths (N_AP) */
#define HADM_MAX_NB_ANTENNA_PATHS (4U)

/*! Max number of antennas */
#define HADM_MAX_NB_ANTENNAS      (4U)

/*! Max channel number in HADM channel map [0..78] */
#define HADM_MAX_CHANNELS (79U)

/*! Max Tx power (dB) */
#define HADM_TX_PWR_MAX    (4)

/*! Min Tx power (dB) */
#define HADM_TX_PWR_MIN    (-12)

/*! Antenna permutation index: N_AP=2*/
#define HADM_ANT_PERM_A1_A2        (0U)
#define HADM_ANT_PERM_A2_A1        (1U)

/*! Antenna permutation index: N_AP=3*/

#define HADM_ANT_PERM_A1_A2_A3     (0U)
#define HADM_ANT_PERM_A1_A3_A2     (1U)
#define HADM_ANT_PERM_A2_A1_A3     (2U)
#define HADM_ANT_PERM_A2_A3_A1     (3U)
#define HADM_ANT_PERM_A3_A1_A2     (4U)
#define HADM_ANT_PERM_A3_A2_A1     (5U)
/*! Antenna permutation index: N_AP=4*/

#define HADM_ANT_PERM_A1_A2_A3_A4  (0U)
#define HADM_ANT_PERM_A1_A2_A4_A3  (1U)
#define HADM_ANT_PERM_A1_A3_A4_A2  (2U)
#define HADM_ANT_PERM_A1_A3_A2_A4  (3U)
#define HADM_ANT_PERM_A1_A4_A2_A3  (4U)
#define HADM_ANT_PERM_A1_A4_A3_A2  (5U)
#define HADM_ANT_PERM_A2_A1_A3_A4  (6U)
#define HADM_ANT_PERM_A2_A1_A4_A3  (7U)
#define HADM_ANT_PERM_A2_A3_A4_A1  (8U)
#define HADM_ANT_PERM_A2_A3_A1_A4  (9U)
#define HADM_ANT_PERM_A2_A4_A1_A3  (10U)
#define HADM_ANT_PERM_A2_A4_A3_A1  (11U)
#define HADM_ANT_PERM_A3_A1_A2_A4  (12U)
#define HADM_ANT_PERM_A3_A1_A4_A2  (13U)
#define HADM_ANT_PERM_A3_A2_A4_A1  (14U)
#define HADM_ANT_PERM_A3_A2_A1_A4  (15U)
#define HADM_ANT_PERM_A3_A4_A1_A2  (16U)
#define HADM_ANT_PERM_A3_A4_A2_A1  (17U)
#define HADM_ANT_PERM_A4_A1_A3_A2  (18U)
#define HADM_ANT_PERM_A4_A1_A2_A3  (19U)
#define HADM_ANT_PERM_A4_A2_A3_A1  (20U)
#define HADM_ANT_PERM_A4_A2_A1_A3  (21U)
#define HADM_ANT_PERM_A4_A3_A1_A2  (22U)
#define HADM_ANT_PERM_A4_A3_A2_A1  (23U)

#define HADM_RTT_ANT_ROUND_ROBIN         (0xFE)
#define HADM_RTT_ANT_NO_RECOMMENDATION   (0xFF)

/*!< type flags for a CS subEvent */
#define HADM_SUBEVT_FIRST  (1U << 0U)  /*!< First SubEvent of CS procedure */
#define HADM_SUBEVT_LAST   (1U << 1U)  /*!< Last SubEvent of CS procedure */

/*!< Tone Quality Indicator values bit 0...3 */
#define HADM_TQI_GOOD    (0U)
#define HADM_TQI_MEDIUM  (1U)
#define HADM_TQI_LOW     (2U)
#define HADM_TQI_UNAVAIL (3U)

/*!< Tone Quality Indicator values bit 4...7 */
#define HADM_TQI_TONE_EXT_PRESENT     (2U)
#define HADM_TQI_TONE_EXT_NOT_PRESENT (1U)

#define HADM_NUM_SUBEVENTS_MAX (32U)  /*!< Maximum nb of CS subevents per CS procedure */

#define HADM_STEP_MODE0  (0U)
#define HADM_STEP_MODE1  (1U)
#define HADM_STEP_MODE2  (2U)
#define HADM_STEP_MODE3  (3U)

/* === Types ================================================================ */
typedef enum
{
    HADM_EVENT_EOS = 0U,
    HADM_EVENT_STEP_INT = 1U,
    HADM_EVENT_SYNC_DONE = 2U,
    HADM_EVENT_INVALID
} BLE_HADM_event_type_t;

typedef enum
{
    HADM_ROLE_INITIATOR = 0U,
    HADM_ROLE_REFLECTOR = 1U,
    HADM_ROLE_INVALID
} BLE_HADM_role_t;

/*! Phase continuity scheme accross CS SubEvents */
typedef enum
{
    HADM_SUBEVT_MISSION_MODE = 0U,          /*!< Mission mode */
    HADM_SUBEVT_TEST_MODE,                  /*!< CS Test mode enabled */
    HADM_SUBEVT_TEST_MODE_PHASE_STAB,       /*!< CS Test mode enabled for phase stability test */
} BLE_HADM_subevent_mode_t;
typedef enum 
{
    HADM_RTT_TYPE_CS_AA_ONLY_TIMING = 0U,/*  RTT CS Access Address only timing */
    HADM_RTT_TYPE_FRAC_32BITS_SS, /*  RTT Fractional with 32-bit Sounding Sequence  */
    HADM_RTT_TYPE_FRAC_96BITS_SS, /*  RTT Fractional with 96-bit Sounding Sequence  */
    HADM_RTT_TYPE_FRAC_32BITS_RS, /*  RTT Fractional with 32-bit Random Sequence    */
    HADM_RTT_TYPE_FRAC_64BITS_RS, /*  RTT Fractional with 64-bit Random Sequence    */
    HADM_RTT_TYPE_FRAC_96BITS_RS, /*  RTT Fractional with 96-bit Random Sequence    */
    HADM_RTT_TYPE_FRAC_128BITS_RS,/*  RTT Fractional with 128-bit Random Sequence   */
    HADM_RTT_TYPE_INVALID
} BLE_HADM_rttType_t;

typedef enum 
{
    HADM_RTT_PHY_1MBPS = 0U,
    HADM_RTT_PHY_2MBPS,
    HADM_RTT_PHY_MAX,
    HADM_RTT_PHY_INVALID = HADM_RTT_PHY_MAX
} BLE_HADM_rttPhyMode_t;

typedef enum 
{
    HADM_T_FCS_15  = 15U,
    HADM_T_FCS_40  = 40U,
    HADM_T_FCS_50  = 50U,
    HADM_T_FCS_80  = 80U,
    HADM_T_FCS_150 =150U,
    HADM_T_FCS_INVALID
} BLE_HADM_T_FCS_t;

typedef enum 
{
    HADM_T_IP_10  = 10U,
    HADM_T_IP_40  = 40U,
    HADM_T_IP_80  = 80U,
    HADM_T_IP_145 =145U,
    HADM_T_IP_INVALID
} BLE_HADM_T_IP_t;

typedef enum
{
    HADM_T_PM_10  = 10U,
    HADM_T_PM_20  = 20U,
    HADM_T_PM_40  = 40U,
    HADM_T_PM_INVALID
} BLE_HADM_T_PM_t;

typedef enum
{
    HADM_T_SW_0  = 0U, /*! No antenna switching, 1:1 config */
    HADM_T_SW_1  = 1U,
    HADM_T_SW_2  = 2U,
    HADM_T_SW_4  = 4U,
    HADM_T_SW_10  = 10U,
    HADM_T_SW_INVALID
} BLE_HADM_T_SW_t;

/*! Packet_AA_Quality:
0x00	HADM Access Address check is successful and all bits match the expected sequence
0x01	HADM Access Address check contains one or more-bit errors
0x02	HADM Access Address not found
*/
typedef enum 
{
    HADM_AA_QUALITY_SUCCESS    = 0U,
    HADM_AA_QUALITY_ERROR      = 1U,
    HADM_AA_QUALITY_NOT_FOUND  = 2U,
    HADM_AA_QUALITY_INVALID = 0xFFU
} BLE_HADM_AA_QUALITY_t;

/*! Phase continuity scheme accross CS SubEvents */
typedef enum
{
    HADM_PHASE_CONT_DISABLED = 0U,  /*!< No phase continuity ensures */
    HADM_PHASE_CONT_OVERRIDES,      /*!< Phase continuity ensured by keeping certain radio blocks ON btw subevents */
    HADM_PHASE_CONT_LOOPBACK,       /*!< Phase continuity ensured by correcting PCT after detecting phase ambiguities using loopback */
} BLE_HADM_phase_cont_t;

/*! Distance calibration modes */
typedef enum
{
    HADM_DIST_CAL_MODE_DISABLED = 0U,  /*!< Normal operation */
    HADM_DIST_CAL_MODE_ACTIVE = 1U,    /*!< Enable distance calibration mode (disable internal compensations during procedures) */
} BLE_HADM_distance_cal_mode_t;

/*! Status code */
typedef enum 
{
    HADM_HAL_SUCCESS            = 0U,
    HADM_HAL_FAIL               = 1U,
    HADM_HAL_INVALID_ARGS       = 2U,
    HADM_HAL_ABORTED            = 3U, /* Internal error */
    HADM_HAL_ABORTED_SYNC       = 4U, /* Sync error (mode0 Rx issue) */
    HADM_HAL_STOPPED            = 5U, /* Procedure stopped by user */
    HADM_HAL_COLLISION          = 6U, /* HADM and connection events collision */
    HADM_HAL_TIME_PASSED        = 7U, /* HADM start time has passed */
    HADM_HAL_MEMORY_FULL        = 8U, /* HADM HAL could not allocate memory */
    HADM_HAL_INVALID            = 0xFFU
} BLE_HADM_STATUS_t;

/*! PN sequence */
typedef struct 
{
    uint32 pn1;  /*!< PN sequence Init -> Refl */
    uint32 pn2;  /*!< PN sequence Refl -> Init */
} BLE_HADM_PN_list_t;

typedef struct 
{
    uint32 rand1;  /*!< PN random payload Init -> Refl */
    uint32 rand2;  /*!< PN random payload Refl -> Init */
} BLE_HADM_PN_rand_t;

/*! Antenna board to be used: */
typedef enum 
{
    HADM_ANT_BOARD_EVK                  = 0U,   /*!< EVK board, no diversity */
    HADM_ANT_BOARD_ANTDIV_SMA           = 1U,   /*!< X-FR ANTDIV board, EXT antennas (SMA) */
    HADM_ANT_BOARD_ANTDIV_PRINTED       = 2U,   /*!< X-FR ANTDIV board, printed antennas */
    HADM_ANT_BOARD_LOC_SMA              = 3U,   /*!< LOC board, EXT antennas (SMA) */
    HADM_ANT_BOARD_LOC_PRINTED          = 4U,   /*!< LOC board, printed antennas */
    HADM_ANT_BOARD_INVALID              = 5U,
} BLE_HADM_AntennaBoardType_t;

/*! mode, T_PM extension presence, antenna permutation index for one particular step */
typedef struct 
{
  uint16 channel  : 7;  /*!< channel (0..78), 7 bits */
  uint16 mode     : 2;  /*!< step mode (0..3), 2 bits */
  uint16 pm_ext   : 2;  /*!< PM extension present init | refl, 2 bits */
  uint16 ant_perm : 5;  /*!< Antenna permutation index (0..23), 5 bits */
} BLE_HADM_Chan_Mode_PmExt_AntPerm_t;

/*! 
Index	N_AP	Num ant Init  Num ant Refl	Configuration
0	   1	    1	      1	                1:1
1	   2	    2	      1	                2:1
2	   3	    3	      1	                3:1
3	   4	    4	      1	                4:1
4	   2	    1	      2	                1:2
5	   3	    1	      3	                1:3
6	   4	    1	      4	                1:4
7	   4	    2	      2	                2:2
*/
/*! Antenna configuration index: */
typedef enum 
{
    HADM_ANT_CFG_IDX_0 = 0U,
    HADM_ANT_CFG_IDX_1,
    HADM_ANT_CFG_IDX_2,
    HADM_ANT_CFG_IDX_3,
    HADM_ANT_CFG_IDX_4,
    HADM_ANT_CFG_IDX_5,
    HADM_ANT_CFG_IDX_6,
    HADM_ANT_CFG_IDX_7,
    HADM_ANT_CFG_IDX_INVALID
} BLE_HADM_AntennaConfigIndex_t;

/*! Computes the number of antennas and antenna paths based on the role and config index */
#define HADM_COMPUTE_NUM_ANTENNA(role, configIdx, numAnt, nAP) \
    if (configIdx == HADM_ANT_CFG_IDX_0) {numAnt = 0U; nAP = 1U;}\
    else if (configIdx == HADM_ANT_CFG_IDX_7) {numAnt = 2U; nAP = 4U;}\
    else { \
        if (HADM_ROLE_INITIATOR == role) {\
            if (configIdx <= HADM_ANT_CFG_IDX_3) {numAnt = nAP = configIdx + 1U;}\
            else {numAnt = 1U;  nAP = configIdx - 2U;}\
        }\
        else {\
            if (configIdx <= HADM_ANT_CFG_IDX_3) {numAnt = 1U;  nAP = configIdx + 1U;}\
            else {numAnt = nAP = configIdx - 2U;}\
        }\
    }

/*! Calibration parameters */
typedef struct BLE_HADM_ZeroDistanceCompensationData_tag
{
    int16 rttFineTuningHns[HADM_RTT_PHY_MAX];  /*! Fine RTT compensation value per PHY (will be substracted from ToA-ToD, added to ToD-ToA) */
} BLE_HADM_ZeroDistanceCompensationData_t;

/*! Storage for an HADM event data */
typedef struct BLE_HADM_SubeventResultsData_tag
{
    uint8 resultBufferUsed;            /*! 0: available, 1 if still in use by the LL */
    uint8 connIdx;                     /*!< HAL connection index */
    uint8 subeventIdx;                 /*!< index of CS subevent within the CS procedure */
    uint8 nbStepsCollected;            /*!< Number of CS steps present in Result buffer */
    uint8 firstStepCollected;          /*!< Index of first CS step present in Result buffer */
    int8 referencePwrLevel;            /*! reference power level (RPL) for the event (dBm) */
    int16 frequencyCompensation;       /*! Frequency compensation value in units of 0.01 ppm */
    uint16 syncDelayUs;                /*! elapsed time between subevent start trigger and reception of 1st bit of sync packet. Invalid if subevent failed */
    uint16 resultBufferSize;           /*! resultBuffer size in bytes */
    uint16 debugBufferSize;            /*! debugBuffer size in bytes */
    uint16 halBufOffset;               /* Offset to next octet to read in HAL data buffer */
    uint16 halDbgBufOffset;            /* Offset to next octet to read in HAL debug buffer */
    /* Result buffer */
    uint8 *resultBuffer;               /*! Where to store results. See format below */
    /* Debug buffer */
    uint8 *debugBuffer;                /*! Where to store debug data - allocated by HAL layer (HW-specific) */
} BLE_HADM_SubeventResultsData_t;

/*! Configuration for an HADM SubEvent */
typedef struct BLE_HADM_SubeventConfig_tag
{
    uint32 rxWindowUs;                 /*!< Duration of the Rx window [us] (mode0 timeout detection): if set to 0, the HAL will not program any timeout */
    uint8 configBufferUsed;            /*! 0: available, 1 if still in use by the HAL */
    uint8 connIdx;                     /*!< HAL connection index */
    uint8 subeventIdx;                 /*!< index of CS subevent within the CS procedure */
    uint8 typeFlags;                   /*!< SubEvent type flags (bitmap) (first/last) in CS procedure */
    BLE_HADM_subevent_mode_t mode;     /*!< SubEvent mode */
    uint8 debugFlags;                  /*!< Opaque flags passed from App to LL down to HAL */
    BLE_HADM_phase_cont_t phaseCont;   /*!< Configures phase continuity scheme */
    BLE_HADM_distance_cal_mode_t distanceCalMode;  /*!< Calibration mode to control if various compensations are applied */
    uint8 stepsNb;                     /*!< Number of steps in the SubEvent */
    uint8 pnSeqNb;                     /*!< Number of steps containing a RTT packet in the SubEvent (step modes 0,1 and 3) */
    uint8 mode0Nb;                     /*!< Number of steps mode 0 */
    uint8 inlinePhaseReturn;           /*!< KW47 only: 1 if inlinePhaseReturn is supported by both sides (capabilities), otherwise 0 */
    BLE_HADM_role_t role;              /*!< 0=initiator, 1=reflector */
    int8 txPwrLevel;                   /*! Tx power for the entire SubEvent (dBm) */
    BLE_HADM_rttType_t rttTypes;       /*!< Type of RTT: coarse, frac, frac+soundSeq */
    uint8 rttMode;                     /*!< Mode of RTT steps (1, 3 or none) */
    BLE_HADM_rttPhyMode_t rttPhy;      /*!< 0=1Mbps, 1=2Mbps */
    BLE_HADM_T_IP_t T_IP1_Time;        /*!< T_IP1 in us */
    BLE_HADM_T_IP_t T_IP2_Time;        /*!< T_IP2 in us */
    BLE_HADM_T_FCS_t T_FCS_Time;       /*!< T_FCS in us */
    BLE_HADM_T_PM_t T_PM_Time;         /*!< T_PM in us */
    BLE_HADM_T_SW_t T_SW_Time;         /*!< T_SW in us */
    uint8 rttAntennaID;                /*! Antenna ID for RTT steps */
    BLE_HADM_AntennaConfigIndex_t toneAntennaConfigIdx; /*! Antenna configuration index. 0: no diversity */
    BLE_HADM_Chan_Mode_PmExt_AntPerm_t *chModePmAntMap; /*!< Step configuration length=stepsNb */
#if (HADM_HAL_VERSION == 1)
    BLE_HADM_PN_list_t pnList[HADM_MAX_NB_STEPS_RTT]; /*!< PN sequence list. List index is the nth step containing a RTT packet. Length=pnSeqNb */
    BLE_HADM_PN_rand_t pnRand[HADM_MAX_NB_STEPS_RTT_RAND]; /*! PN random sequence (if used) */
#endif

} BLE_HADM_SubeventConfig_t;

/*! Storage for HAL properties */
typedef struct BLE_HADM_HalProperties_tag
{
    uint16 txWarmupUs;                 /*! time required for the radio to Tx 1st bit after expiration of LL_SCHED_HadmWaitAnchorOffset() */
    uint16 prepareMaxUs;               /*! maximum time required for the HAL to run up to LL_SCHED_HadmWaitAnchorOffset() given HW capabilities (max number of steps) */
    uint16 rxWarmupUs;                 /*! time required for the radio to be ready in Rx after expiration of LL_SCHED_HadmWaitAnchorOffset() */
} BLE_HADM_HalProperties_t;

/*! Storage for HAL Capabilities */
typedef struct BLE_HADM_HalCapabilities_tag
{
    uint8 stepModeSupported;
    uint8 numAntennasSupported;
    uint8 nNumAPSupported;
    uint8 RTT_Capability;
    uint8 RTT_Coarse_N;
    uint8 RTT_Sounding_N;
    uint8 RTT_Random_Sequence_N;
    uint16 NADM_Sounding_Capability;
    uint16 NADM_Random_Sequence_Capability;
    uint8 PHYSupported;
    uint8 T_SW_TimeSupported;
    uint8 FAErequired;
    uint8 InlinePhaseReturn;
    uint16 T_IP1_TimesSupported;
    uint16 T_IP2_TimesSupported;
    uint16 T_FCS_TimesSupported;
    uint16 T_PM_TimesSupported;
} BLE_HADM_HalCapabilities_t;


/*! Result buffer format: 
 *  For each step reported in [0...Num_Steps_Reported[:
 *        + Step_Mode,
 *        + Step_Channel,
 *        + Step_Data_Length,
 *        + Step_Data
 *        Step_Data is formatted according to HCI spec to speed-up HCI flush at LL level
 *           Mode 0: Step_Data_Length = 5 (Init) or 3 (Refl) bytes
 *   +	Packet_AA_Quality       (1 byte)
 *   +	Packet_RSSI             (1 byte, Range: -127 to +20 Units: dBm)
 *   +  Packet_Antenna          (1 byte)
 *            + Measured_Freq_Offset    (2 bytes, Measured CFO in unit of 0.01 ppm, signed integer). ** Initiator role only **
 *
 *           Mode 1: Step_Data_Length = 6 bytes
 *   +	Packet_AA_Quality       (1 byte)
 *            + Packet_NADM             (1 byte)
 *   +	Packet_RSSI             (1 byte, Range: -127 to +20 Units: dBm)
 *            + ToX-ToX                 (2 bytes, Time diff in ns, Q18.0 format, 0xFFFFFF if unavailable)
 *   +  Packet_Antenna          (1 byte)
 *
 *           Mode 2:  Step_Data_Length = (1+4x(N_AP+1) bytes)
 *   +	Antenna_Permutation_Index (1 byte)
 *            For each antenna path in [0...N_AP+1]:
 *              +   Tone_PCT               (3 bytes, PCT for antenna path #k (24 bits, including 12 LSBs for I sample and 12 MSBs for Q sample)
 *              +   Tone_Quality_Indicator (1 byte, TQI PCT for antenna path #k)
 *
 *           Mode 3:  Step_Data_Length = (7+4x(N_AP+1) bytes)
 *            + Packet_AA_Quality       (1 byte)
 *            + Packet_NADM             (1 byte)
 *            + Packet_RSSI             (1 byte, Range: -127 to +20 Units: dBm)
 *            + Packet_Antenna          (1 byte)
 *            + ToX-ToX                 (2 bytes, Time diff in ns, Q18.0 format, 0xFFFFFF if unavailable)
 *            + Antenna_Permutation_Index (1 byte)
 *            For each antenna path in [0...N_AP+1]:
 *              +   Tone_PCT               (3 bytes, PCT for antenna path #k (24 bits, including 12 LSBs for I sample and 12 MSBs for Q sample)
 *              +   Tone_Quality_Indicator (1 byte, TQI PCT for antenna path #k)
*/
/*! Step report size based on step mode and number of APs */
#define BLE_HADM_STEP0_REPORT_SIZE(role)   ((role) == 0 ? 5U : 3U)
#define BLE_HADM_STEP1_REPORT_SIZE      (6U)
#define BLE_HADM_STEP2_REPORT_SIZE(n_ap)   (1U + 4U * (1U + (n_ap)))
#define BLE_HADM_STEP3_REPORT_SIZE(n_ap)   (BLE_HADM_STEP2_REPORT_SIZE(n_ap) + BLE_HADM_STEP1_REPORT_SIZE)

#define BLE_HADM_REPORT_SIZE_MAX(nbStepMode0, nbStepNonMode0, ap) (((nbStepMode0) * BLE_HADM_STEP0_REPORT_SIZE(0)) + ((nbStepNonMode0) * BLE_HADM_STEP3_REPORT_SIZE(ap)))

/*! HADM report maximum size based on current HAL implementation */
#define BLE_HADM_HAL_REPORT_SIZE_MAX (BLE_HADM_REPORT_SIZE_MAX(HADM_MAX_NB_STEPS_MODE0, HADM_MAX_NB_STEPS - HADM_MAX_NB_STEPS_MODE0, HADM_MAX_NB_ANTENNA_PATHS))

/* === Externals ============================================================ */

/* === Prototypes =========================================================== */

/*!
 * API to notify LL about HADM HAL events.
 * Is called from an ISR.
 */
void BLE_HADM_NotifyLL(BLE_HADM_SubeventResultsData_t *result_p, BLE_HADM_event_type_t type, BLE_HADM_STATUS_t status);

#if (HADM_HAL_VERSION == 1)
/*!
 * API to wait for the active HADM Link to reach its synchronization point.
 */
uint32 LL_SCHED_HadmWaitAnchorOffset();
#endif

/*!
 * Abstract function that needs to be implemented by the HW-specific HAL implementation.
 * This API initializes HADM HAL resources.
 */
BLE_HADM_STATUS_t BLE_HADM_Init(void);

/*!
 * Abstract function that needs to be implemented by the HW-specific HAL implementation.
 * This API releases HADM HAL resources.
 */
//void BLE_HADM_DeInit(void);

/*!
 * Abstract function that needs to be implemented by the HW-specific HAL implementation.
 * This API is called before starting a new procedure on the given connection.
 */
BLE_HADM_STATUS_t BLE_HADM_ProcedureInit(uint8 connIdx);

/*!
 * Abstract function that needs to be implemented by the HW-specific HAL implementation.
 * This API allows the LL to configure antenna diversity logic table.
 * antBoardTable array size is MAX_ANTENNA (=4) + 1
 * value stored in antBoardTable[MAX_ANTENNA] corresponds to "all OFF" state
 */
BLE_HADM_STATUS_t BLE_HADM_SetAntennaType(uint8 *antBoardTable);

/*!
 * Abstract function that needs to be implemented by the HW-specific HAL implementation.
 * This API allows the LL to configure calibration parameters.
 */
void BLE_HADM_SetZeroDistanceCompensationData(BLE_HADM_ZeroDistanceCompensationData_t *compData);

/*!
 * Abstract function that needs to be implemented by the HW-specific HAL implementation.
 * This API allows the LL to configure hadm debug buffer size and address.
 */
void BLE_HADM_SetDmaDebugBuff(uint16 dma_debug_buff_size, uint32 dma_debug_buff_address);


/*!
 * Abstract function that needs to be implemented by the HW-specific HAL implementation.
 * This API allows the LL to retrieve HAL-specific constant characteristics.
 */
const BLE_HADM_HalProperties_t *BLE_HADM_GetProperties(void);

/*!
 * Abstract function that needs to be implemented by the HW-specific HAL implementation.
 * This API allows the LL to retrieve, for the given config, the preparation time needed
 * by the HAL before being able to launch the subevent (unit: us).
 */
uint16 BLE_HADM_GetPrepareTime(const BLE_HADM_SubeventConfig_t *hadm_config);

/*!
 * Abstract function that needs to be implemented by the HW-specific HAL implementation.
 * This API allows the LL to retrieve HAL-specific capabilities.
 */
const BLE_HADM_HalCapabilities_t *BLE_HADM_GetCapabilities(void);

/*!
 * Abstract function that needs to be implemented by the HW-specific HAL implementation.
 * This API checks configuration validity and returns a status accordingly
 */
BLE_HADM_STATUS_t BLE_HADM_SubeventCheckConfig(const BLE_HADM_SubeventConfig_t *config);

/*!
 * Abstract function that needs to be implemented by the HW-specific HAL implementation.
 * Enable HW calibrations to be performed before starting any HADM activity.
 */
BLE_HADM_STATUS_t BLE_HADM_Calibrate(BLE_HADM_rttPhyMode_t rate);

/*!
 * Abstract function that needs to be implemented by the HW-specific HAL implementation.
 * HADM_HAL_VERSION==1: Need to be coupled with BLE_HADM_SubeventStart() since it is disruptive in terms of HW programming.
 * HADM_HAL_VERSION==2: Can be called assynchrounously from LL since not disruptive on any HW module
 */
BLE_HADM_STATUS_t BLE_HADM_SubeventConfigApply(const BLE_HADM_SubeventConfig_t *config);

/*!
 * Abstract function that needs to be implemented by the HW-specific HAL implementation.
 * Enable HW so that it starts HADM subEvent based on applied config.
 */
BLE_HADM_STATUS_t BLE_HADM_SubeventStart(const BLE_HADM_SubeventConfig_t *config);

/*!
 * Stop Subevent in HAL and associated HW blocks if required (i.e. given subevent is active)
 * Also releases the associated HAL config buffer and all corresponding HAL results buffers
 */
void BLE_HADM_SubeventStop(const BLE_HADM_SubeventConfig_t *config);

/*!
 * Stop procedure in HAL and associated subevents
 * Also releases all corresponding HAL config & results buffers
 */
void BLE_HADM_ProcedureStop(uint8 connIdx);

/*! AES wrapper function used for DRBG */
void BLE_HADM_Drbg_AES_wrapper(uint8 *target, const uint8 *source, uint32 size_buff, const uint8 *key, const int cbc, const uint8 *iv );

#if (HADM_HAL_VERSION == 2)
/*!
 * API Used by HAL to request DRBG AA / Payload generation for one CS SYNC step
 * cs_sync_step_data buffer is filled as follows:
 * + AA[4] init->refl
 * + AA[4] refl->init
 * + Payload[4/8/12/16] init->refl
 * + Payload[4/8/12/16] refl->init
 * Payload type (random or soundind sequence) and size is determined by the rtt_type
 * returns number of 32bits words written @cs_sync_step_data
 * Note this format is compatible with PKT RAM format
 */
uint8 BLE_HADM_DRBG_Generate_CS_SYNC_step(uint8 hadmConnIdx, uint8 subeventIdx, uint8 stepCnt, BLE_HADM_rttType_t rtt_type, uint32 *cs_sync_step_data);
#endif
/*!
 * API Used by LL to release HAL results buffer
 */
void BLE_HADM_ReleaseResultsBuffer(BLE_HADM_SubeventResultsData_t **result_p);

/*!
 *
 * This API requests a configuration buffer to the HAL.
 * Returns NULL is no more buffer available
 */
BLE_HADM_SubeventConfig_t* BLE_HADM_GetConfigBuffer(void);
/*!
 * API Used by LL to release HAL config buffer
 */
void BLE_HADM_ReleaseConfigBuffer(BLE_HADM_SubeventConfig_t **config_p);

#endif /* _BLE_HADM_HAL_H_ */
