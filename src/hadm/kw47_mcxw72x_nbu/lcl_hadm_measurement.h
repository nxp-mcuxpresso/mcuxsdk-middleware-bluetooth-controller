/*
 * Copyright 2024 NXP
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* Prevent double inclusion */
#ifndef _LCL_HADM_MEASUREMENT_H_
#define _LCL_HADM_MEASUREMENT_H_

/*!
 * @addtogroup genfsk_isp GENFSK Narrowband Localization
 * @{
 */

/*! @file
 * Describe HADM measurement data structures.
 */

/* === Includes ============================================================= */
#include <stdbool.h>
#include "ll_types.h" /* maps LL types to SDK types */
#include "nxp_xcvr_lcl_ctrl.h"
#include "ble_hadm_hal.h"

/* === Macros =============================================================== */
#define HADM_PLL_CAL_INTERPOLATION

#define ABS(x) (((x) > 0) ? ((uint32_t)(x)) :((uint32_t) -(x)))

/* Approximation of complex magnitude (sqrt(i^2+q^2)). i and q need to be absolute values */
#define MAG(i,q) (MAX(i,q) + (3*(MIN(i,q)) >> 3U))

//#define RSM_DBG_IQ   /* turned on with care: all IQs received during rx-di-en will be dumped over DMA. Make sure dest buffer is large enough */

/* Size of an IQ record when reported over the air */
#define IQ_SIZE 3

/*! To convert from HADM channel to frequency in Mhz */
#define HADM_CHAN_NUM_TO_MHZ(channel) (2402U + channel)

#define RX_SAMPLING_RATE (4U)                /* 4 MHz @1Mbps */
#define TX_SAMPLING_RATE (8U)                /* 8 MHz @1Mbps */
#define SAMPLING_RATE_FACTOR_2MBPS (2U)      /* Sampling rate x 2 @2Mbps */

#define HADM_T_FM            (80U)      /* Frequency measurement, T_FM = 80 us */
#define HADM_T_RD            (5U)       /* Ramp-down, T_RD = 5 us */
#define HADM_T_GD            (10U)      /* Guard time, T_GD = 10 us */
#define HADM_T_SY(rate)      (rate == 0 ? 44U : 26U)  /* T_SY = 44 us using the LE 1M PHY and 26 us when using the LE 2M PHY */
   
#define HADM_NUM_IQ_PER_US(us, data_rate /* 0 or 1 */, avg_win)  ((RX_SAMPLING_RATE * us * (1+data_rate)) >> avg_win) /* Number of IQ's per us depending on modulation and averaging window */

/*! @name HADM debug flags
 * Proprietary debug flags are encoded in HCI RTT_Type 
 * @{
 */
#define HADM_DBG_FLG_IQ_DMA        (1 << 0)  /*< Export captured IQ to Host memory using DMA */
#define HADM_DBG_FLG_AVG_OFF       (1 << 1)  /*< Disable HW averaging during IQ capture */
#define HADM_DBG_FLG_DBG_INFO      (1 << 2)  /*< Add debug info to Event Result (non standard) */
#define HADM_DBG_FLG_CFO_COMP_DIS  (1 << 3)  /*< Disable CFO compensation when set */
/*! @}*/

/*! @name HADM error flags
 *  Flags used to report errors in flags fields, @see hadm_info_t::flag
 * @{
 */
#define FLAGS_HADM_PLL_ERROR                    0x0001          /*!< PLL lock error */
#define FLAGS_HADM_ABORT                        0x0002          /*!< HADM sequence was aborted */
#define FLAGS_HADM_AGC_NOT_FROZEN               0x0004          /*!< Error during AGC lock */
#define FLAGS_HADM_IQ_CAPTURE_NOT_COMPLETE      0x0008          /*!< Error during IQ capture */
#define FLAGS_HADM_NO_SIGNAL                    0x0010          /*!< RSSI measured became too low */
#define FLAGS_HADM_SYNC_ERROR                   0x0020          /*!< Error happened during synchronization phase */
#define FLAGS_HADM_RTT_TS_ERROR                 0x0040          /*!< Timestamp reading on one or more RTT packets failed */
#define FLAGS_HADM_SW_SCHED_ERROR               0x0080          /*!< SW scheduler detected a desynchronization with RSM HW scheduler */
#define FLAGS_HADM_XCVR_API_ERROR               0x0100          /*!< XCVR API reported an error */
#define FLAGS_HADM_CFO_TOO_LARGE                0x0200          /*!< CFO measured was too large and could not get compensated */
#define FLAGS_HADM_PHASE_AMBIGUITY_UNRESOLVED   0x0400          /*!< Phase ambiguity detected in loopback could not be resolved */
/*! @}*/

#define HADM_HAL_PKT_RAM_CONFIG_CIRC_BUFF_SIZE  (256U) /* In words */
#define HADM_HAL_PKT_RAM_RESULT_CIRC_BUFF_SIZE  (HADM_HAL_PKT_RAM_CONFIG_CIRC_BUFF_SIZE) /* same size */
   
#define HADM_HAL_PKT_RAM_NB_STEPS_MARGIN          (2U)   /* margin btw number of config steps actually programmed and IRQ steps  */
#define HADM_HAL_PKT_RAM_MAX_NB_STEPS_BEFORE_IRQ  (8U)  /* maximum number of CS steps before step interrupt */
#define HADM_HAL_PKT_RAM_NB_STEPS_CONFIG_INITIAL(nbMode0) (nbMode0 + 1U) /* number of CS steps loaded initially during applyConfig */
#define HADM_HAL_PKT_RAM_MAX_NB_STEPS_ENGAGED     (HADM_HAL_PKT_RAM_MAX_NB_STEPS_BEFORE_IRQ + HADM_HAL_PKT_RAM_NB_STEPS_MARGIN)
#define HADM_HAL_PKT_RAM_IN_FLIGHT_DATA_BUFFER_SIZE (HADM_HAL_PKT_RAM_NB_STEPS_CONFIG_INITIAL(3U) + (HADM_HAL_PKT_RAM_MAX_NB_STEPS_ENGAGED*2U))

/* === Types ================================================================ */
/*! Contains some register backup values captured after Mode0 phase */
typedef enum
{
    HADM_HAL_MEAS_STATE_IDLE = 0U,
    HADM_HAL_MEAS_STATE_CONFIGURED = 1U,
    HADM_HAL_MEAS_STATE_RUNNING = 2U,
    HADM_HAL_MEAS_STATE_POSTPROCESSING = 3U,
    HADM_HAL_MEAS_STATE_ABORTING = 4U,
} hadm_meas_state_t;

/*! Contains some register backup values captured after Mode0 phase */
typedef struct
{
    uint8_t valid;
    int8_t  rssi;
    uint8_t agc_idx;
    int32_t cfo;
} hadm_sync_info_t;

typedef struct
{
    uint16_t hpm_cal_val;  /*!< External HPM CAL value. */
} hadm_pll_cal_data_t;

/*! Information collected during HADM measurement - reported via HciLeHadmEventResultDebugEvent */
typedef PACKED_STRUCT
{
    uint16_t   rtt_dbg_buffer_nb;  /*!< Not used. For backward compatibilty with other platforms */
    uint16_t   mciq_dbg_buffer_nb; /*!< Not used. For backward compatibilty with other platforms */
    uint32_t   flags;              /*!< Error flags @see HADM_flags */
    int32_t    sync_cfo;           /*!< CFO [Hz] measured during during alignment phase*/
    uint8_t    sync_step_id;       /*!< Step number of mode 0 retained for the synch phase */
    int8_t     sync_rssi;          /*!< RSSI [dB] obtained during alignment phase */
    uint8_t    sync_rxgain;        /*!< RX gain (AGC index) set during alignment phase */
    uint8_t    xtal_trim;          /*!< Xtal trim value used after alignement phase */
    uint16_t   agc_delay;          /*!< AGC group delay corresponding to selected RX gain */
    int8_t     temperature;        /*!< Temperature of device during measurement */
    uint8_t    num_time_adj;       /*!< Number of time-grid adjustement in event */
} hadm_info_t;

typedef struct
{
    int8_t i_resid;
    int8_t q_resid;
} hadm_dc_resid_t;

typedef struct
{
    /* runtime */
    uint8_t  curr_page;        /*!< Current Buffer page */
    uint8_t  curr_step_idx;    /*!< Next step index to be written/read to/from circular buffer */
    /* pseudo static */
    uint8_t  max_step_size;    /*!< Maximum size of step in PKT RAM (in 4 bytes words) */
    /* static */
    uint8_t  ram_type;         /*!< PKT RAM type (0:TX, 1:RX) */
    uint16_t buff_len;         /*!< Buffer length (in 4 bytes words) */
    uint32_t *base_ptr;        /*!< Base pointer to PKT RAM */
} hadm_circ_buff_desc_t;

typedef struct
{
    uint8_t nb_steps_before_irq;         /*!< Number of steps before RSM step IRQ */
    uint8_t nb_irq_steps_handled;        /*!< Number of RSM step IRQ handled in subevent */
    uint32_t *config_write_ptr;          /*!< Write pointer to Circular buffer for CS Steps config */
    uint32_t *result_read_ptr;           /*!< Read pointer to Circular buffer for CS Steps results */
    hadm_circ_buff_desc_t step_config;   /*!< Circular buffer descriptor for CS Steps config */
    hadm_circ_buff_desc_t step_result;   /*!< Circular buffer descriptor for CS Steps results */
} hadm_pkt_ram_desc_t;

typedef struct
{
    uint8_t cs_sync_ant_id;             /*!< RTT antenna id */ 
    uint32_t aa_rx;                     /*!< AA used in Rx */ 
} hadm_data_in_flight_t;

/*! Working structure for a subEvent */
typedef struct
{
    const BLE_HADM_SubeventConfig_t *config_p;  /*!< pointer on HADM configuration */ 
    BLE_HADM_SubeventResultsData_t *result_p;   /*!< pointer on HADM results */
    hadm_pkt_ram_desc_t    pkt_ram;             /*!< PKT RAM descriptoprs (circular buffers) */
    hadm_meas_state_t      state;               /*!< HAL meas state */
    uint8_t                debug_flags;         /*!< debug flags, see @HADM debug flags */
    uint8_t                iq_avg_win;          /*!< averging window size (power of 2): valid values: 0 and  [2...8] */
    uint8_t                iq_capture_win;      /*!< capture window size in us, depends on T_PM config */
    uint8_t                num_ant;             /*!< number of antennas from diversity config [1...4] */
    uint8_t                n_ap;                /*!< number of antenna paths from diversity config [1...4] */
    uint8_t                rtt_antenna_id;      /*!< Antenna Id to be applied to CS SYNC packets [0...3] */
    uint16_t               iq_buff_size;        /*!< IQ buffer size for all IQs from mode 2 and 3's. Unit: number of IQ samples, hence in 4 bytes words */
    uint16_t               iq_buff_size_mode0;  /*!< IQ buffer size for mode 0's. Unit: number of samples */
    uint16_t               step_duration[4];    /*!< Step duration for modes 0,1,2,3 in us */
    uint32_t               ts_delay_hns;        /*!< RTT latency (half ns) to remove from ToX-ToX */
    uint32_t               ts_extra_delay_hns;  /*!< Extra RTT latency (half ns) to remove from ToX-ToX */
    xcvr_lcl_rsm_config_t  rsm_config;          /*!< RSM XCVR configuration for this CS subevent */
    hadm_info_t            info;                /*!< pointer to HADM measurement info struct */
    hadm_sync_info_t       sync_info[HADM_MAX_NB_STEPS_MODE0]; /*!< structure to store synchronization information */
    uint8_t                data_in_flight_w_idx;  /*!< write index for in flight data */
    uint8_t                data_in_flight_r_idx;  /*!< read index for in flight data */
    hadm_data_in_flight_t  pkt_ram_data_in_flight[HADM_HAL_PKT_RAM_IN_FLIGHT_DATA_BUFFER_SIZE]; /*!< Working buffer of Rx AA currently in flight in config PKT RAM buffer It is required to keep a copy for HARTT frac delay computation ... */
} hadm_meas_t;

/* HAL Data associated to a CS procedure */
typedef struct
{
    bool                   is_proc_init_done;   /*!< Set to TRUE if procedure intialization has been done  */
    uint8_t                cfo_channel;         /*!< Channel index of the retained mode 0 */
    uint8_t                agc_idx;             /*!< AGC idx for the procedure (0xFF means AGC unlocked) */
    int8_t                 ppm;                 /*!< ppm computed based on mode0 CFO */
    int32_t                cfo;                 /*!< CFO [Hz] measured during alignment phase */
} hadm_proc_t;

/* HAL Data associated to this device */
typedef struct
{
    hadm_meas_t            *active_meas_p;      /*!< pointer on active CS measurement from hadm_meas[]  */
    bool                   is_rsm_cal_done;     /*!< Set to 1 if PLL calibration was successful (done once for all)  */
    uint8_t                ant2gpio[HADM_MAX_NB_ANTENNAS]; /*!< pointer to the LUT performing mapping btw antenna index an GPIOs */
#ifdef HADM_PLL_CAL_INTERPOLATION
    /*! Store PLL calibration data for channel 40 */
    hadm_pll_cal_data_t    cal_ch40[HADM_RTT_PHY_MAX];
#else
     /*! Store PLL calibration data for each HADM channel */
    hadm_pll_cal_data_t    cal_data[HADM_RTT_PHY_MAX][HADM_MAX_CHANNELS];
#endif
    /*! Store zero distance compensation information */
    BLE_HADM_ZeroDistanceCompensationData_t zero_distance_comp;
    /*! RTT compensation delay computed from current temperature */
    int32_t rtt_temperature_comp_hns[HADM_RTT_PHY_MAX];
    int16_t current_temperature;
    uint16_t dma_debug_buff_size;
    uint32_t dma_debug_buff_address;
} hadm_device_t;

/* === Externals ============================================================ */

extern hadm_device_t hadm_device;

/* === Prototypes =========================================================== */

#ifdef __cplusplus
extern "C" {
#endif

BLE_HADM_STATUS_t lcl_hadm_init(void);
BLE_HADM_STATUS_t lcl_hadm_init_procedure(uint8 connIdx);
void lcl_hadm_handle_temperature_change(int32_t temperature);
void lcl_hadm_reset_tpm_count(void);
BLE_HADM_STATUS_t lcl_hadm_set_antenna_type(uint8 *antBoardTable);
void lcl_hadm_set_dma_debug_buffer(uint16 dma_debug_buff_size, uint32 dma_debug_buff_address);
BLE_HADM_STATUS_t lcl_hadm_calibrate_pll(BLE_HADM_rttPhyMode_t rate);
BLE_HADM_STATUS_t lcl_hadm_calibrate_dcoc(BLE_HADM_rttPhyMode_t rate);
const BLE_HADM_HalProperties_t *lcl_hadm_get_properties(void);
uint16 lcl_hadm_get_prepare_time(const BLE_HADM_SubeventConfig_t *hadm_config);
const BLE_HADM_HalCapabilities_t *lcl_hadm_get_capabilities(void);
BLE_HADM_STATUS_t lcl_hadm_check_config(const BLE_HADM_SubeventConfig_t *hadm_config);
BLE_HADM_STATUS_t lcl_hadm_configure(const BLE_HADM_SubeventConfig_t *hadm_config);
BLE_HADM_STATUS_t lcl_hadm_run_measurement(const BLE_HADM_SubeventConfig_t *config);
void lcl_hadm_stop_measurement(const BLE_HADM_SubeventConfig_t *config);
void lcl_hadm_stop_procedure(uint8 connIdx);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* _LCL_HADM_MEASUREMENT_H_ */

/* EOF */
