/**
 * @file lcl_hadm_measurement.c
 *
 * This file implements platform dependent HADM functionality
 *
 * For KW45, RSM block is used as HW scheduler. However a SW scheduler is required
 * to run in parallel (lcl_hadm_run_measurement) in order to perform additional actions.
 * The SW scheduder expects following IRQ to be raised during measurement:
 * - LCL_HAL_XCVR_RSM_IRQ_FC for all modes
 * - LCL_HAL_XCVR_RSM_IRQ_IP for XCVR_RSM_STEP_PK_PK & XCVR_RSM_STEP_PK_TN_TN_PK
 */
/*
 * Copyright 2021-2024 NXP
 * SPDX-License-Identifier: BSD-3-Clause
 */
 
/* === Includes ============================================================ */
#include <stdint.h>
#include <stdbool.h>
#include "fsl_os_abstraction.h"
#include "fsl_ltc.h" /* LTC driver for AES */
#include "EmbeddedTypes.h"
#include "lcl_hadm_measurement.h"
#include "lcl_xcvr_hal.h"
#include "lcl_hadm_utils.h"
#include "lcl_hadm_hartt.h"
#include "nxp2p4_xcvr.h"
#include "nxp_xcvr_lcl_ctrl.h"

#if !defined(KW45B41Z82_NBU_SERIES) && !defined(KW45B41Z83_NBU_SERIES)
#error this code is supposed to be compiled for NBU core
#endif


#ifdef HADM_RTT_TYPE_3
#if !defined(SUPPORT_RSM_LONG_PN) || (SUPPORT_RSM_LONG_PN == 0)
#error HADM_RTT_TYPE_3 requires SUPPORT_RSM_LONG_PN=1
#endif
#define HADM_RTT_TYPE_3_LEN(phy)       ((HADM_RTT_PREAMBLE_LEN_1MBPS << phy) + HADM_RTT_AA_LEN + HADM_RTT_TYPE_3_PAYLOAD_LEN + HADM_RTT_TRAILER_BITS) /* in bits */
#endif

/* === Macros ============================================================== */
#define HADM_CFO_COMP_PER_STEP /* Enable CFO compensation refinment on each step */

/* The following timings have been measured in debug mode with IAR 9.30.1 */

/*! Time needed to execute BLE_HADM_Calibrate()
 * in order to perform the radio calibrations (HPM and DCOC)
 */
#define HADM_HAL_HPM_CAL_US  (180U)
#define HADM_HAL_DCOC_CAL_US (410U)
#define HADM_HAL_RADIO_CAL_US (HADM_HAL_DCOC_CAL_US + HADM_HAL_HPM_CAL_US)

/*! Time needed by the HAL to execute BLE_HADM_EventConfigApply() based on the number of steps
 */
#define HADM_HAL_CONFIG_APPLY_US(nbSteps, nbPnSteps) (50U + (nbSteps*7U)/2U + nbPnSteps*2U)

/*! Time needed by the HAL to execute BLE_HADM_EventStart() up to LL_SCHED_HadmWaitAnchorOffset() call
 */
#define HADM_HAL_START_US (215U)

/*! Time needed by the HAL to execute BLE_HADM_EventConfigApply() + up to LL_SCHED_HadmWaitAnchorOffset() call
 *  Does not need to be exact, should cover longest execution path
 */

#define HADM_HAL_PREPARE_US(nbSteps, nbPnSteps) (HADM_HAL_START_US + HADM_HAL_CONFIG_APPLY_US(nbSteps, nbPnSteps))

/*! Duration of software path between the end of LL_SCHED_HadmWaitAnchorOffset() polling
 * and RSMGo. This path is supposed to be the shortest possible (no code should be added there).
 */
#ifdef HADM_ENABLE_DEBUG_PINS
#define HADM_HAL_SW_POLL_US (4U)
#else
#define HADM_HAL_SW_POLL_US (3U)
#endif

/* First RSM Rx window is a bit late.
 * In order to compensate that, we delay RSM on initiator side so that both RSM
 * can still be triggered at the same time from SW perspective.
 */
#define HADM_HAL_RSM_INITIATOR_TRIGGER_DELAY (5U)

#define HADM_RTT_ANT_ID_SHIFT (14U)
#define HADM_RTT_ANT_ID_MASK  (0xC000U)
#define HADM_RTT_TS_MASK      (0x3FFFU) /* 14 bits are enough to store TS diff in ticks (>500us) */

/* Contengency time to make sure mode 0 steps are done */
#define HADM_MODE0_TIMEOUT_MARGIN_US (500U)

/* === Types =============================================================== */

/* === Globals ============================================================= */

static xcvr_lcl_rsm_config_t hadm_rsm_config =
{
    .op_mode                    = XCVR_RSM_SQTE_MODE,
    .num_steps                  = 0U,
    .t_fc                       = 0U,
    .t_ip1                      = 0U,
    .t_ip2                      = 0U,
    .t_pm0                      = 0U,
    .rxdig_dly                  = 0U,
    .txdig_dly                  = 0U,
    .trig_sel                   = XCVR_RSM_TRIG_SEQ_SPARE3,
    .trig_delay                 = 0U,
    .num_ant_path               = 1U,
    .use_rsm_dma_mask           = false, /* disable dma_mask */
    .user_callback = NULLPTR,
#if defined(SUPPORT_RSM_LONG_PN) && (SUPPORT_RSM_LONG_PN == 1)
    .rtt_len                    = XCVR_RSM_SQTE_PN32,
#endif /* defined(SUPPORT_RSM_LONG_PN) && (SUPPORT_RSM_LONG_PN == 1) */
    .rate                       = XCVR_RSM_RATE_1MBPS,
    .averaging_win              = XCVR_RSM_AVG_WIN_DISABLED, //XCVR_RSM_AVG_WIN_4_SMPL,
    .rsm_dma_dly_pm             = 0U,
    .rsm_dma_dly_fm_ext         = 0U,
    .rsm_dma_dur_pm             = 0U,
    .rsm_dma_dur_fm_ext         = 0U,
 /* New fields JVM */
    .sniffer_mode_en            = false,
    .t_sw                       = lclTSw_0usec,
};

static BLE_HADM_HalProperties_t hadm_hal_properties;

/* This reflects the capabilities of our IC */
/* CS capabilities might be further refined by the controller based on external elements capabilities (e.g antenna system) */
static const BLE_HADM_HalCapabilities_t hadm_hal_capabilities = {
    .stepModeSupported          = 0U, /* step mode 3 not supported */
    .numAntennasSupported       = HADM_MAX_NB_ANTENNAS,
    .nNumAPSupported            = HADM_MAX_NB_ANTENNA_PATHS,
    .RTT_Capability             = 0x1, /* RTT_Coarse_N field refers to the 10ns precision requirement */
    .RTT_Coarse_N               = 10, /* Number of RTT steps to satisfy the precision requirement. */ 
    .RTT_Sounding_N             = 0, /* not supported */
    .RTT_Random_Sequence_N       = 0, /* not supported */
    .NADM_Sounding_Capability   = 0, /* NADM not supported */
    .NADM_Random_Sequence_Capability = 0, /* NADM not supported */
    .PHYSupported               = 1<<1, /* 2Mbps PHY supported (bit #1) */
    .T_SW_TimeSupported         = 2, /* 2us */
    .FAErequired                = 0, /* TBD */
    .InlinePhaseReturn          = 0U, /* HW does not support */
    /* note: mandatory timings are not included in capabilities */
    .T_IP1_TimesSupported       = 0x0048, /* T_IP1=80 or 40us */
    .T_IP2_TimesSupported       = 0x0048, /* T_IP2=80 or 40us */
    .T_FCS_TimesSupported       = 0x0050, /* T_FCS=80 or 50us */
    .T_PM_TimesSupported        = 0x0002, /* T_PM=20us */
};


/* Contains data associated to this device */
hadm_device_t hadm_device;
/* Contains data that needs to be maintained across several subevents of the same procedure.
 * Several procedures may run in parralele (multi-conneciton).
 */
static hadm_proc_t hadm_procs[HADM_MAX_NB_CONNECTIONS];
/* Contains data for current subevent */
static hadm_meas_t hadm_meas;

// FIXME: at a later stage, do not use temp buffers
static uint16_t rtt_ts_buffer[HADM_MAX_NB_STEPS];
static int8_t hadm_rssi_buffer[HADM_MAX_NB_STEPS];
static uint32_t capture_buffer[HADM_MAX_NB_STEPS * (1U + HADM_MAX_NB_ANTENNA_PATHS)];
static uint16_t tone_quality_buffer[HADM_MAX_NB_STEPS * (1U + HADM_MAX_NB_ANTENNA_PATHS)];

/* === Externals =========================================================== */

/* === Prototypes ========================================================== */

static void lcl_hadm_measurement_setup(hadm_proc_t *hadm_proc, const BLE_HADM_SubeventConfig_t *hadm_config);
static void lcl_hadm_measurement_teardown(const BLE_HADM_SubeventConfig_t *hadm_config);
static xcvrLclStatus_t lcl_hadm_measurement_post_processing(hadm_proc_t *hadm_proc, XCVR_RSM_RXTX_MODE_T rsm_mode, XCVR_RSM_SQTE_RATE_T rate, hadm_info_t *hadm_info_p);

/* === Implementation (private) ============================================= */
BLE_HADM_STATUS_t lcl_hadm_init_procedure(uint8 connIdx)
{
    hadm_proc_t *hadm_proc;

    assert(connIdx < HADM_MAX_NB_CONNECTIONS);
    if (connIdx >= HADM_MAX_NB_CONNECTIONS)
    {
        return HADM_HAL_INVALID_ARGS;
    }
    hadm_proc = &hadm_procs[connIdx];
    hadm_proc->ppm = 0;
    hadm_proc->cfo_channel = 0;
    hadm_proc->cfo = 0;
    hadm_proc->agc_idx = 0xFF;
    hadm_proc->is_proc_init_done = true;
    
    return HADM_HAL_SUCCESS;
}

/* === Implementation (public) ============================================= */

BLE_HADM_STATUS_t lcl_hadm_init(void)
{
    BLE_HADM_STATUS_t hal_status = HADM_HAL_SUCCESS;

    lcl_hal_xcvr_tof_setup_tpm_mux();
    lcl_hadm_init_tpms();
    lcl_hadm_utils_init_buffers();
    
    DEBUG_PIN0_CLR
    DEBUG_PIN1_CLR

    hadm_hal_properties.prepareMaxUs = HADM_HAL_PREPARE_US(HADM_MAX_NB_STEPS, HADM_MAX_NB_STEPS_RTT); /* maximum prepare time given RSM capabilities */

    /*
     * FAST_RX2TX_START_FC and FAST_TX2RX_START_FC are actually TSM indexes (us) where TSM rewinds to start an FCS phase.
     * As a consequence, they represent the elapsed time between RSM start and the begining of FCS phase for initiator and reflector respectively.
     * Note: FCS phase in the RSM does not include ramp down which occurs at the very end of a step in the TSM sequence.
     * Typical value is 25us (for both roles).
     * Adjusted by 1us as per sniffer measurement.
     */
    hadm_hal_properties.txWarmupUs = (uint8_t)((xcvr_lcl_tsm_generic_config.FAST_CTRL3 & XCVR_TSM_FAST_CTRL3_FAST_RX2TX_START_FC_MASK) >> XCVR_TSM_FAST_CTRL3_FAST_RX2TX_START_FC_SHIFT) - HADM_T_RD + 1U;
    hadm_hal_properties.rxWarmupUs = (uint8_t)((xcvr_lcl_tsm_generic_config.FAST_CTRL3 & XCVR_TSM_FAST_CTRL3_FAST_TX2RX_START_FC_MASK) >> XCVR_TSM_FAST_CTRL3_FAST_TX2RX_START_FC_SHIFT) - HADM_T_RD + 1U;

    hadm_hal_properties.txWarmupUs += ((xcvr_lcl_tsm_generic_config.WU_LATENCY & XCVR_TSM_WU_LATENCY_TX_DATAPATH_LATENCY_MASK) >> XCVR_TSM_WU_LATENCY_TX_DATAPATH_LATENCY_SHIFT);
    hadm_hal_properties.txWarmupUs += HADM_HAL_SW_POLL_US + HADM_HAL_RSM_INITIATOR_TRIGGER_DELAY;
    hadm_hal_properties.rxWarmupUs += HADM_HAL_SW_POLL_US + HADM_HAL_RSM_INITIATOR_TRIGGER_DELAY;

    hadm_meas.config_p = NULL;
    hadm_meas.result_p = NULL;
    hadm_meas.is_config_valid = false;
    hadm_meas.debug_flags = 0;
    hadm_meas.iq_avg_win = 0;
    hadm_meas.iq_capture_win = 0;
    hadm_meas.num_ant = 0;
    hadm_meas.n_ap = 0;

    hadm_meas.iq_buff_size = 0;
    hadm_meas.iq_buff_size_mode0 = 0;
    hadm_meas.time_elapsed = 0U;
    hadm_meas.time_to_adj = 0xFFFFFFFFU;
    hadm_meas.ts_buff_tmp_p = rtt_ts_buffer;
    hadm_meas.info = lcl_hadm_utils_get_info_ptr();
    for (int i = 0; i < HADM_MAX_NB_ANTENNAS + 1U; i++)
    {
        hadm_device.ant2gpio[i] = 0U;
    }

    /* Perform initial calibration */
    hadm_device.is_rsm_cal_done = false;
#ifndef SIMULATOR
    hal_status = lcl_hadm_calibrate_dcoc(HADM_RTT_PHY_1MBPS);
    if (hal_status == HADM_HAL_SUCCESS)
    {
        hal_status = lcl_hadm_calibrate_dcoc(HADM_RTT_PHY_2MBPS);
    }
    if (hal_status == HADM_HAL_SUCCESS)
    {
        hal_status = lcl_hadm_calibrate_pll(HADM_RTT_PHY_1MBPS);
    }
    if (hal_status == HADM_HAL_SUCCESS)
    {
        hal_status = lcl_hadm_calibrate_pll(HADM_RTT_PHY_2MBPS);
    }
    if (hal_status == HADM_HAL_SUCCESS)
#endif
    {
        hadm_device.is_rsm_cal_done = true;
    }
    /* Initialize temperature compensation (assume 20 degrees C in case the host does not inform NBU) */
    lcl_hadm_handle_temperature_change(20);

    /* Init LTC for DRBG */
    LTC_Init(LTC0);

    /* Backup TSM */
    lcl_hal_xcvr_hadm_backup();

    assert(hadm_device.is_rsm_cal_done == true);
    return hal_status;
}

/* This API is supposed to be called in the context of idle task */
void lcl_hadm_handle_temperature_change(int32_t temperature)
{
    OSA_InterruptDisable();
    hadm_device.current_temperature = (int16_t)temperature;
    lcl_hadm_utils_calc_rtt_temperature_delay(temperature, &hadm_device);
    OSA_InterruptEnable();
}

BLE_HADM_STATUS_t lcl_hadm_set_antenna_type(uint8 *antBoardTable)
{
    BLE_HADM_STATUS_t hal_status = HADM_HAL_SUCCESS;
    
    for (int i = 0; i < HADM_MAX_NB_ANTENNAS + 1U; i++)
    {
        if (antBoardTable[i] <= 15U) /* max is 4 bits */
        {
            hadm_device.ant2gpio[i] = antBoardTable[i];
        }
        else
        {
           hal_status = HADM_HAL_INVALID_ARGS;
           assert(0);
           break;
        }
    }
    
    if (hal_status == HADM_HAL_SUCCESS)
    {
        LCL_HAL_SET_ANTENNA_PORT(hadm_device.ant2gpio[0]);
    }
    
    return hal_status;
}

void lcl_hadm_set_dma_debug_buffer(uint16 dma_debug_buff_size, uint32 dma_debug_buff_address)
{
    hadm_device.dma_debug_buff_size = dma_debug_buff_size;
    hadm_device.dma_debug_buff_address = dma_debug_buff_address;
}

BLE_HADM_STATUS_t lcl_hadm_calibrate_dcoc(BLE_HADM_rttPhyMode_t rate)
{   
    xcvrLclStatus_t status;
    
    DEBUG_PIN0_SET
    DEBUG_PIN1_SET

    /* trigger calibration */
    XCVR_LCL_CalibrateDcocStart((XCVR_RSM_SQTE_RATE_T)rate);
    /* wait for results */
    status = XCVR_LCL_CalibrateDcocComplete();
    
    DEBUG_PIN1_CLR

    /* store values for future usage */
    lcl_xcvr_hal_store_dcoc_cal(rate);
    
    DEBUG_PIN0_CLR

    return (status == gXcvrLclStatusSuccess ? HADM_HAL_SUCCESS : HADM_HAL_FAIL);
}

BLE_HADM_STATUS_t lcl_hadm_calibrate_pll(BLE_HADM_rttPhyMode_t rate)
{   
    xcvrLclStatus_t status = gXcvrLclStatusSuccess;
    XCVR_RSM_SQTE_RATE_T xcvr_rate = (XCVR_RSM_SQTE_RATE_T) (rate & 0x1);
    BLE_HADM_STATUS_t hal_status = HADM_HAL_SUCCESS;

    DEBUG_PIN0_SET

    /* Backup PLL config */
    lcl_hal_xcvr_pll_settings_backup();

#ifdef HADM_PLL_CAL_INTERPOLATION
    uint16_t chan40_ovrd;
    /* Trigger manual calibration on channel 40 */
    status = XCVR_LCL_MakeChanNumFromHadmIndex(40 /* 2442 MHz */, &chan40_ovrd);
    DEBUG_PIN1_SET
    status += XCVR_LCL_CalibratePll(&chan40_ovrd, (xcvr_lcl_pll_cal_data_t *)&hadm_device.cal_ch40[rate], 1, false, xcvr_rate);
    DEBUG_PIN1_CLR
#else

    /* Trigger manual calibration on all channels */
    uint16_t chan;
    uint16_t fstep_chan_num_ovrd[HADM_MAX_CHANNELS] = {0U};
    for (chan=0; chan < HADM_MAX_CHANNELS; chan++)
    {
        status += XCVR_LCL_MakeChanNumFromHadmIndex(chan, &fstep_chan_num_ovrd[chan]);
    }
    DEBUG_PIN1_SET
    status += XCVR_LCL_CalibratePll(fstep_chan_num_ovrd, (xcvr_lcl_pll_cal_data_t *)&hadm_device.cal_data[rate], HADM_MAX_CHANNELS, false, xcvr_rate);
    DEBUG_PIN1_CLR
#endif
    
    /* Restore PLL config */
    lcl_hal_xcvr_pll_settings_restore();

    if (gXcvrLclStatusSuccess == status)
    {
        /* Mark calibration as available */
        hal_status = HADM_HAL_SUCCESS;
    }
    else
    {
        hal_status = HADM_HAL_FAIL;
    }

    DEBUG_PIN0_CLR

    return hal_status;
}

const BLE_HADM_HalProperties_t *lcl_hadm_get_properties(void)
{
    return &hadm_hal_properties;
}

uint16 lcl_hadm_get_prepare_time(const BLE_HADM_SubeventConfig_t *hadm_config)
{
   uint16 prepareTimeUs = HADM_HAL_PREPARE_US(hadm_config->stepsNb, hadm_config->pnSeqNb);
   assert(prepareTimeUs <= hadm_hal_properties.prepareMaxUs);

   return prepareTimeUs;
}

const BLE_HADM_HalCapabilities_t *lcl_hadm_get_capabilities(void)
{
    return &hadm_hal_capabilities;
}

BLE_HADM_STATUS_t lcl_hadm_check_config(const BLE_HADM_SubeventConfig_t *hadm_config)
{
    uint32_t num_ant, n_ap;

    HADM_COMPUTE_NUM_ANTENNA(hadm_config->role, hadm_config->toneAntennaConfigIdx, num_ant, n_ap);
    
    if((hadm_config->stepsNb > HADM_MAX_NB_STEPS) || (hadm_config->stepsNb < HADM_MIN_NB_STEPS))
        goto config_error;
    
    if ((hadm_config->pnSeqNb > HADM_MAX_NB_STEPS_RTT) || (hadm_config->pnSeqNb == 0U)
#ifdef HADM_RTT_TYPE_3
        || ((hadm_config->pnSeqNb > (HADM_MAX_NB_STEPS_RTT_RAND)) && (hadm_config->rttTypes == HADM_RTT_TYPE_FRAC_32BITS_RS))
#endif
       )
        goto config_error;

    if ((hadm_config->rttTypes != HADM_RTT_TYPE_CS_AA_ONLY_TIMING)
#ifdef HADM_RTT_TYPE_3
        && (hadm_config->rttTypes != HADM_RTT_TYPE_FRAC_32BITS_RS)
#endif
       )
        goto config_error;

    if (hadm_config->chModePmAntMap[0].mode != 0) /* First step shall be mode 0 */
        goto config_error;
    
    if (hadm_config->T_PM_Time > HADM_T_PM_40)
        goto config_error;
    
    if ((hadm_config->toneAntennaConfigIdx == HADM_ANT_CFG_IDX_0) && (hadm_config->T_SW_Time != HADM_T_SW_0)) /* N_AP=1 => T_SW=0 */
        goto config_error;
        
    if ((hadm_config->toneAntennaConfigIdx != HADM_ANT_CFG_IDX_0) && (hadm_config->T_SW_Time < HADM_T_SW_2)) /* only T_SW >1us is supported */
        goto config_error;

    if ((hadm_config->rttAntennaID == 0) || ((hadm_config->rttAntennaID > HADM_MAX_NB_ANTENNAS) && (hadm_config->rttAntennaID < 0xFE)))
        goto config_error;

    if (((((uint32_t)hadm_config->T_SW_Time + (uint32_t)hadm_config->T_PM_Time) * (n_ap + 1U)) % 10U) != 0) /* RSM limitation: 10us granularity... */
        goto config_error;

    if (num_ant > HADM_MAX_NB_ANTENNAS)
        goto config_error;

    if (hadm_config->subeventIdx > HADM_NUM_SUBEVENTS_MAX)
        goto config_error;

    return HADM_HAL_SUCCESS;
    
config_error:
    return HADM_HAL_INVALID_ARGS;
}

BLE_HADM_STATUS_t lcl_hadm_configure(const BLE_HADM_SubeventConfig_t *hadm_config)
{
    uint32_t i;
    xcvrLclStatus_t status;

    uint8_t *fstep_pram_ptr = (uint8_t *)RSM_FSTEP_RAM;

    DEBUG_PIN0_SET

    hadm_meas.is_config_valid = false;
    hadm_meas.debug_flags = hadm_config->debugFlags;

#ifdef RSM_DEBUG
    lcl_hadm_utils_debug_init(&hadm_meas);
#endif
    hadm_meas.config_p = hadm_config; /* save config ptr */
    hadm_meas.result_p = lcl_hadm_utils_get_result_buffer((hadm_meas.debug_flags & HADM_DBG_FLG_DBG_INFO) != 0);

    if (hadm_meas.result_p == NULL)
    {
        return HADM_HAL_MEMORY_FULL;
    }

    hadm_meas.result_p->connIdx = hadm_config->connIdx;
    hadm_meas.result_p->subeventIdx = hadm_config->subeventIdx; /* echo subeventIdx */
    hadm_meas.result_p->nbStepsCollected = hadm_config->stepsNb; /* no steps interrupt, all steps reported */
    hadm_meas.result_p->firstStepCollected = 0;

    hadm_meas.nb_rtt_valid = 0;
    
    /* Configure iq_capture_win accoding to T_PM: choose the closest smaller power of 2 */
    /* this will allow discard of transient + IQ averaging */
    switch (hadm_config->T_PM_Time)
    {
        case HADM_T_PM_40:
          hadm_meas.iq_avg_win = 5;
          break;
        case HADM_T_PM_20:
          hadm_meas.iq_avg_win = 4;
          break;
        case HADM_T_PM_10:
          hadm_meas.iq_avg_win = 3;
          break;
        default:
          goto config_error;
          break;
    }
    hadm_meas.iq_capture_win = (1 << hadm_meas.iq_avg_win); /* this will result in 4 IQ's / AP / step */
#ifndef RSM_DBG_IQ
    if (hadm_meas.debug_flags & HADM_DBG_FLG_AVG_OFF)
#endif
    {
        hadm_meas.iq_avg_win = 0;
    }
#ifndef RSM_DBG_IQ
    else if (hadm_config->rttPhy == HADM_RTT_PHY_2MBPS)
    {
        /* Double IQ averaging window in order to get the same number of final (averaged) samples whatever the data rate */
        hadm_meas.iq_avg_win ++;
   }
#endif

    HADM_COMPUTE_NUM_ANTENNA(hadm_config->role, hadm_config->toneAntennaConfigIdx, hadm_meas.num_ant, hadm_meas.n_ap);
    
    /* Compute IQ buffer size */
    lcl_hadm_utils_compute_iq_buff_size(hadm_config, &hadm_meas, (hadm_config->rttPhy == HADM_RTT_PHY_2MBPS) ? (RX_SAMPLING_RATE*SAMPLING_RATE_FACTOR_2MBPS):RX_SAMPLING_RATE);

    /* Compute step mode durations */
    lcl_hadm_utils_compute_step_duration(hadm_config, hadm_meas.n_ap, hadm_meas.step_duration);

    /* Clear info structs */
    hadm_meas.info->sync_cfo = 0;
    hadm_meas.info->sync_rxgain = 0xFF;
    hadm_meas.info->sync_rssi = -128;
    hadm_meas.info->flags = 0;
    hadm_meas.info->temperature = hadm_device.current_temperature;
    hadm_meas.info->sync_step_id = 0xFF;
    hadm_meas.info->num_time_adj = 0;
    hadm_meas.time_elapsed = 0U;
    hadm_meas.time_to_adj = 0xFFFFFFFFU;

    for (i=0; i < HADM_MAX_NB_STEPS_MODE0; i++)
    {
        hadm_meas.sync_info[i].valid = 0;
        hadm_meas.sync_info[i].rssi = -128;
        hadm_meas.sync_info[i].rssi_raw = -128;
        hadm_meas.sync_info[i].agc_idx = 0xFF;
        hadm_meas.sync_info[i].cfo = 0;
    }

    /* Check result buffers pointers */
    assert(hadm_meas.result_p->resultBuffer != NULL);

    /* Prepare TSM/RSM config */
    hadm_rsm_config.num_steps = hadm_meas.config_p->stepsNb;
    hadm_rsm_config.num_ant_path = hadm_meas.n_ap;
    hadm_rsm_config.use_rsm_dma_mask = 0;
    hadm_rsm_config.rate = (XCVR_RSM_SQTE_RATE_T)hadm_meas.config_p->rttPhy;
    hadm_rsm_config.rsm_dma_dly_fm_ext = (HADM_T_FM - hadm_meas.iq_capture_win) >> 1; /* center capture window inside T_FM */
    hadm_rsm_config.rsm_dma_dur_fm_ext = hadm_meas.iq_capture_win;
    if (hadm_meas.config_p->mode != HADM_SUBEVT_TEST_MODE_PHASE_STAB)
    {
        hadm_rsm_config.t_pm0 = ((hadm_meas.n_ap + 1U) * ((uint32_t)hadm_meas.config_p->T_PM_Time + (uint32_t)hadm_meas.config_p->T_SW_Time));
    }
    else
    {
        hadm_rsm_config.t_pm0 = LCL_HAL_T_PM_MEAS; /* Set to T_PM_MEAS for stable phase test */
        hadm_meas.iq_buff_size_mode0 = 0;
        hadm_meas.iq_buff_size = 0;
        hadm_meas.iq_avg_win = 0;
    }
    if (hadm_meas.iq_avg_win == 0)
    {
        hadm_rsm_config.averaging_win = XCVR_RSM_AVG_WIN_DISABLED;
    }
    else
    {
        hadm_rsm_config.averaging_win = (XCVR_RSM_AVG_WIN_LEN_T) (hadm_meas.iq_avg_win - 1);
    }
    /* For now, we use SW trigger for test mode and procedure mode */
    hadm_rsm_config.trig_sel =  XCVR_RSM_TRIG_SW;
    hadm_rsm_config.trig_delay = (hadm_config->role == HADM_ROLE_INITIATOR) ? HADM_HAL_RSM_INITIATOR_TRIGGER_DELAY:0U;
    
    /* CONNRF-1157 revised timing API for generic TSM */
    hadm_rsm_config.t_fc = hadm_config->T_FCS_Time;
    hadm_rsm_config.t_ip1 = hadm_config->T_IP1_Time;
    hadm_rsm_config.t_ip2 = hadm_config->T_IP2_Time;
#if (SUPPORT_RSM_LONG_PN == 1)
    hadm_rsm_config.rtt_len = XCVR_RSM_SQTE_PN32; /* Always start with 32bits AA on mode 0s */
#endif
    DEBUG_PIN1_SET
    status = XCVR_LCL_ValidateRsmSettings(&hadm_rsm_config);
    if (gXcvrLclStatusSuccess != status)
        goto config_error;
    DEBUG_PIN1_CLR
    /* Determine which calibration will be handled by RSM autonomously or not */
#if defined(CONNRF_1163_IF_COMP) && (CONNRF_1163_IF_COMP == 1)
#else
    uint8_t is_rsm_cal_auto = lcl_xcvr_hal_rsm_get_cal_mode(hadm_rsm_config.t_ip1_rsm_config);
    if (!(is_rsm_cal_auto & LCL_HAL_XCVR_RSM_CTUNE_CAL_IS_AUTO))
        goto config_error; /* for now, all TSM configs have CTUNE calibration executed on the fly */
#endif
    if (!hadm_device.is_rsm_cal_done)
    {
        goto config_error; /* at this point all calibration must have ben performed */
    }

     /* Prepare RSM config in packet RAM: PN sequences */
    DEBUG_PIN1_SET
#ifdef HADM_RTT_TYPE_3
    if (hadm_config->rttTypes == HADM_RTT_TYPE_FRAC_32BITS_RS)
    {
        uint32_t *pn_ram_ptr = (uint32_t *)RSM_PN_RAM;
        for (int i = 0; i < hadm_config->pnSeqNb; i++)
        {
            if (hadm_config->chModePmAntMap[i].mode == 0)
            {
                *pn_ram_ptr++ = hadm_config->pnList[i].pn1;
                *pn_ram_ptr++ = hadm_config->pnList[i].pn2;
            }
            else
            {
                *pn_ram_ptr++ = hadm_config->pnList[i].pn1;
                *pn_ram_ptr++ = hadm_config->pnRand[i].rand1;
                *pn_ram_ptr++ = hadm_config->pnList[i].pn2;
                *pn_ram_ptr++ = hadm_config->pnRand[i].rand2;
            }
        }
    }
    else
#endif
    {
        status = XCVR_LCL_SetPnRamShort((xcvr_lcl_pn32_config_t *)hadm_config->pnList, hadm_config->pnSeqNb);
    }
    if (gXcvrLclStatusSuccess != status)
        goto config_error;
    
    DEBUG_PIN1_CLR
    DEBUG_PIN1_SET

    /* Prepare RSM config in packet RAM: steps configuration */
    for (i=0; i < hadm_config->stepsNb; i++)
    {
        XCVR_RSM_FSTEP_TYPE_T step_mode = (XCVR_RSM_FSTEP_TYPE_T)hadm_config->chModePmAntMap[i].mode;
        uint16_t hpm_cal_val;

#ifdef HADM_PLL_CAL_INTERPOLATION
        if (step_mode == XCVR_RSM_STEP_TN_TN)
        {
            hpm_cal_val = hadm_device.cal_ch40[hadm_meas.config_p->rttPhy].hpm_cal_val; /* fixed HPM cal for tones */
        }
        else if ((step_mode == XCVR_RSM_STEP_PK_PK) || (step_mode == XCVR_RSM_STEP_FCS))
        {
            hpm_cal_val = lcl_hadm_get_hpm_cal_interpolation(hadm_config->chModePmAntMap[i].channel, hadm_device.cal_ch40[hadm_meas.config_p->rttPhy].hpm_cal_val); /* interpolated HPM cal for packets */
        }
        else
#else
        hpm_cal_val = hadm_device->cal_data[hadm_meas.config_p->rttPhy][hadm_meas.config_p->chModePmAntMap[i].channel].hpm_cal_val;
#endif
        if (step_mode == XCVR_RSM_STEP_PK_TN_TN_PK)
        {
            /* Step mode 3 are not supported */
            hpm_cal_val = 0;
            goto config_error;
        }

        /* Build RSM step in PKT RAM config area */
        LCL_HAL_BUILD_PKT_RAM_CONFIG_STEP(i, step_mode, fstep_pram_ptr, hpm_cal_val);

    } /* end for steps */


    if (gXcvrLclStatusSuccess != status)
        goto config_error;

    DEBUG_PIN1_CLR
      
    /* Enable required RSM interrupts for SW scheduler */
    XCVR_LCL_RsmIrqEnDis(LCL_HAL_RSM_XCVR_IRQ_MASK_FLAGS, true);

    /* Configure Antenna switching using LCL block */
    if (hadm_meas.config_p->mode != HADM_SUBEVT_TEST_MODE_PHASE_STAB)
    {
       lcl_hal_xcvr_configure_ant_switch(hadm_meas.n_ap, hadm_config->T_PM_Time, hadm_config->T_SW_Time, hadm_meas.iq_capture_win, hadm_meas.config_p->rttPhy);
    }
    /* Configure rttAntennaID to be used during first mode 0 */
    /* If no recommendation from the host then apply crossed roud robin in the controller since it gives better perf */
    if (hadm_config->rttAntennaID >= HADM_RTT_ANT_ROUND_ROBIN)
        hadm_meas.rtt_antenna_id = 0U; /* start round robin */
    else
        hadm_meas.rtt_antenna_id = hadm_config->rttAntennaID - 1U; /* fixed antenna */
    lcl_hadm_utils_set_CS_SYNC_antenna(&hadm_meas);
       
    hadm_meas.is_config_valid = true;

    DEBUG_PIN0_CLR

    return HADM_HAL_SUCCESS;
    
config_error:

    assert(0);
    return HADM_HAL_FAIL;
}


BLE_HADM_STATUS_t lcl_hadm_run_measurement(const BLE_HADM_SubeventConfig_t *hadm_config_p)
{
    uint32_t step_no, rsm_step_no, rsm_irq, num_iq_per_step_per_ap, timestamp1 = 0, timestamp2 = 0;
    BLE_HADM_STATUS_t hal_status = HADM_HAL_SUCCESS;
    xcvrLclStatus_t status = gXcvrLclStatusSuccess;
    uint8_t rtt_ant_id;
    bool proc_agc_locked;
    bool subevent_synced;
    hadm_sync_info_t *sync_info_p;
    XCVR_RSM_RXTX_MODE_T rsm_mode;
    hadm_info_t *hadm_info_p = hadm_meas.info;
    uint32_t *capture_buffer_p;
    uint16_t *tone_quality_buffer_p;
    int32_t time_adjustment = 0; /* in us */
    uint32_t dtest_page_backup;
    uint32_t rx_timeout;
    hadm_proc_t *hadm_proc;

    if (!hadm_meas.is_config_valid)
    {
        /* At this point, the config must be valid */
        assert(false);
        return HADM_HAL_INVALID_ARGS;
    }

    hadm_proc = &hadm_procs[hadm_config_p->connIdx];
    if ((hadm_config_p->connIdx >= HADM_MAX_NB_CONNECTIONS) || !hadm_proc->is_proc_init_done)
    {
        /* At this point, the procedure must have been initialized */
        assert(false);
        return HADM_HAL_INVALID_ARGS;
    }

    proc_agc_locked = (hadm_proc->agc_idx != 0xFF);
    subevent_synced = false;

    dtest_page_backup = lcl_hadm_utils_dtest_set_page(DTEST_RSM2);

    DEBUG_PIN0_SET
    DEBUG_PIN1_SET
    
    /* Determine rx timeout (for reflector devices) = mode 0 duration + margin + margin coming from window widening */
    rx_timeout = hadm_meas.step_duration[0] + HADM_MODE0_TIMEOUT_MARGIN_US + hadm_config_p->rxWindowUs;
    assert(rx_timeout < 0xFFFF);
    
    /* Determine first antenna that will be used by CS event */
    if (hadm_config_p->rttAntennaID >= HADM_RTT_ANT_ROUND_ROBIN)
        rtt_ant_id = 0U;
    else 
        rtt_ant_id = hadm_meas.rtt_antenna_id;

    rsm_mode = (hadm_config_p->role == HADM_ROLE_INITIATOR) ? XCVR_RSM_TX_MODE : XCVR_RSM_RX_MODE;
    
 #ifndef SIMULATOR
    DEBUG_PIN0_PULSE
      
    lcl_hadm_measurement_setup(hadm_proc, hadm_config_p);
    
    DEBUG_PIN0_PULSE

    capture_buffer_p = &capture_buffer[0];
    tone_quality_buffer_p = &tone_quality_buffer[0];

    num_iq_per_step_per_ap = HADM_NUM_IQ_PER_US(hadm_meas.iq_capture_win, hadm_config_p->rttPhy, hadm_meas.iq_avg_win);
    
    /* Configure RSM block */
    status = XCVR_LCL_RsmInit(&hadm_rsm_config);
    assert(gXcvrLclStatusSuccess == status);
#if (0)       /* Old API */
    status += XCVR_LCL_Set_TSM_FastStart(rsm_mode, &hadm_rsm_config);
#else
    status += XCVR_LCL_Set_TSM_FastStart(rsm_mode, &hadm_rsm_config);
#endif
    LCL_HAL_DISABLE_RX_SYNC

#ifdef RSM_WA_FOR_RX_SETLLING_LATENCY_KFOURWONE_2175
    lcl_hal_xcvr_increase_rx_settling_latency();
#endif
    LCL_HAL_SET_IQ_CAPTURE_POINT(LCL_OUT_FRAC_CORR_SEL);
      
    /* Start DMA if needed */
#ifndef RSM_DBG_IQ
#ifdef HADM_RTT_TYPE_3
    if (hadm_config_p->rttTypes == HADM_RTT_TYPE_FRAC_32BITS_RS)
    {
#if 0 /* enable to capture only the 32bits payload of RTT type 3 packets */
        lcl_hal_xcvr_configure_dma_capture(LCL_START_DMA_ON_FSK_AA_MATCH, HADM_AA_MATCH_DMA_LATENCY, RX_SAMPLING_RATE * HADM_RTT_TYPE_3_PAYLOAD_LEN, hadm_device.dma_debug_buff_size, hadm_device.dma_debug_buff_address);
#else
        lcl_hal_xcvr_configure_dma_capture(LCL_START_DMA_ON_TSM_RX_DIG_EN, HADM_RX_EN_DMA_LATENCY(hadm_config_p->rttPhy), RX_SAMPLING_RATE * HADM_RTT_TYPE_3_LEN(hadm_config_p->rttPhy), hadm_device.dma_debug_buff_size, hadm_device.dma_debug_buff_address);
#endif
        /* DMA will be started later, before first mode 1 */
    }
    else
#endif
    if ((hadm_meas.debug_flags & HADM_DBG_FLG_IQ_DMA) && ((hadm_meas.iq_buff_size > 0) || (hadm_meas.iq_buff_size_mode0 > 0)))
    {
        lcl_hal_xcvr_configure_dma_capture(LCL_START_DMA_ON_RSM_TRIGGER, 0, hadm_meas.iq_buff_size + hadm_meas.iq_buff_size_mode0, hadm_device.dma_debug_buff_size, hadm_device.dma_debug_buff_address);
        LCL_HAL_START_DMA(LCL_DMA_PAGE_RXDIGIQ);
    }
#else
    LCL_HAL_SET_IQ_CAPTURE_POINT(LCL_OUT_CH_FILTER_SEL);
    LCL_HAL_CLEAR_DMA_MASK_AND_AVG_WIN;
    lcl_hal_xcvr_configure_dma_capture(LCL_START_DMA_ON_TSM_RX_DIG_EN, 0, 0x2000, hadm_device.dma_debug_buff_size, hadm_device.dma_debug_buff_address);
    LCL_HAL_START_DMA(LCL_DMA_PAGE_RXDIGIQ);
#endif

    /* Start DBG RAM capture with RSM trigger if needed */
    if (hadm_meas.iq_buff_size_mode0 > 0)
    {
        lcl_hal_xcvr_start_dbg_ram_capture(LCL_DMA_PAGE_PHASE, LCL_START_DMA_ON_RSM_TRIGGER, hadm_meas.iq_buff_size_mode0 * LCL_HAL_XCVR_PHASE_SAMPLE_SIZE); /* capture phases for mode 0's */
    }
    else if (hadm_meas.iq_buff_size > 0)
    {
        lcl_hal_xcvr_start_dbg_ram_capture(LCL_DMA_PAGE_RXDIGIQ, LCL_START_DMA_ON_RSM_TRIGGER, num_iq_per_step_per_ap * (hadm_meas.n_ap + 1U) * LCL_HAL_XCVR_IQ_SAMPLE_SIZE);
    }
      
    /* Enable Antenna switching if needed */
    LCL_HAL_START_LCL;
      
    /* Fine synchronization (us) to trigger RSM at the right time in connected mode.
     * The RSM should be started so that the T_FCS period starts at the correct time
     * as computed by the LL.
     * LL_SCHED_HadmWaitAnchorOffset() is aware of the current HADM event and
     * has knowledge of all the timings.
     */
    if ((hadm_config_p->mode == HADM_SUBEVT_MISSION_MODE) || 
        (hadm_config_p->mode == HADM_SUBEVT_TEST_MODE) && (rsm_mode == XCVR_RSM_TX_MODE))
    {
        uint32 rc;
        DEBUG_PIN0_CLR
        rc = LL_SCHED_HadmWaitAnchorOffset();

        if (rc != 0)
        {
            assert(false);
            hal_status = HADM_HAL_TIME_PASSED;
            hadm_info_p->flags |= FLAGS_HADM_START_TIMES_PASSED;
        }
        DEBUG_PIN_ALL_TGL
    }
    /* Start RSM block */
    /* Note: first mode 0 will synchronize reflector with initiator by waiting for the AA found of the first packet */
    XCVR_LCL_RsmGo(rsm_mode, &hadm_rsm_config);
    DEBUG_PIN1_PULSE

    /* Reset TPM counter to capture elapsed time before receiving the first mode0 (captured via timestamp channel) */
    lcl_hadm_reset_tpm_count();

    /* Do not add code/introduce delay here, between RsmGo and start of TPM. */
    if ((hadm_config_p->mode == HADM_SUBEVT_MISSION_MODE) || (hadm_config_p->rxWindowUs != 0))
    {
        /* Start supervision timer to detect RSM synchronization timeout and capture mode0 arrival time */
        lcl_hadm_tpm_timer_start(rx_timeout);
    }
    else
    {
        lcl_hadm_tpm_timer_stop();
    }

    /* Run SW scheduler in // than RSM scheduler to perform actions that HW does not support */
    /* At this point, all interrupts on NBU except RSM and TPM2 ones are disabled */
    /* Do not add more code btw RSM start and first call to lcl_hadm_utils_wait_for_rsm_irq(), to ensure IRQs are polled ASAP */
    step_no = 0;
    while ((step_no < hadm_rsm_config.num_steps) && (hal_status == HADM_HAL_SUCCESS))
    {
        uint32_t timestamp;
        bool tpm_has_triggered;
        bool time_adjustment_done = false; /* time adj has been done in current step */

        XCVR_RSM_FSTEP_TYPE_T step_mode = (XCVR_RSM_FSTEP_TYPE_T)hadm_config_p->chModePmAntMap[step_no].mode;
        /* Wait for next IRQ (_WFI()) */
        rsm_irq = lcl_hadm_utils_wait_for_rsm_irq(&rsm_step_no, (uint32_t)rsm_mode);

        /* Clearing TPM trigger must happen before tx_dig_en, so we read timestamp
         * and clear trigger as early as possible, even if RTT is not requested.
         */
        tpm_has_triggered = lcl_hadm_flush_tpm_timestamp(&timestamp);

        /* Handle time grid adjustment must happem as early as possible */
        if (rsm_irq & LCL_HAL_XCVR_RSM_IRQ_FC)
        {
            hadm_meas.time_elapsed += (uint32_t)hadm_meas.step_duration[step_mode];
        }
        if (hadm_meas.time_elapsed >= hadm_meas.time_to_adj)
        {
            if (rsm_irq & LCL_HAL_XCVR_RSM_IRQ_FC)
            {
                lcl_hal_xcvr_apply_time_grid_shift(time_adjustment);
                hadm_meas.info->num_time_adj++;
            }
            if (rsm_irq & LCL_HAL_XCVR_RSM_IRQ_IP)
            {
                /* Time grid has been adjusted on previous T_FCS, restore it */
                lcl_hal_xcvr_apply_time_grid_shift(-time_adjustment);
                hadm_meas.time_elapsed = 0;
                if (hadm_meas.info->num_time_adj == 1)
                {
                    /* Subsequent time adjustments will happen after 1us drift accumulation */
                    hadm_meas.time_to_adj <<= 1;
                }
            }
            time_adjustment_done = true;
        }

        if (rsm_irq & LCL_HAL_XCVR_RSM_IRQ_FC)
        {
            /* Determine next step's mode */
            /* it's safe to access step_no + 1 because there's no IRQ_FC on last step, but an IRQ_EOS */
            XCVR_RSM_FSTEP_TYPE_T next_step_mode = (XCVR_RSM_FSTEP_TYPE_T)hadm_config_p->chModePmAntMap[step_no + 1].mode;

#ifdef HADM_CFO_COMP_PER_STEP
            /* Refine CFO compensation for next step channel */
            if ((hadm_config_p->role == HADM_ROLE_INITIATOR) && (step_mode != XCVR_RSM_STEP_FCS) && ((hadm_meas.debug_flags & HADM_DBG_FLG_CFO_COMP_DIS) == 0))
            {
                int32_t cfo = hadm_proc->cfo;
                cfo += ((int32_t)hadm_config_p->chModePmAntMap[step_no + 1].channel - (int32_t)hadm_proc->cfo_channel) * hadm_proc->ppm;
                if (gXcvrLclStatusSuccess != XCVR_LCL_RsmCompCfo(-cfo))
                {
                    hadm_info_p->flags |= FLAGS_HADM_CFO_TOO_LARGE; /* CFO was too large and could not get compensated */
                }
            }
#endif
            if ((next_step_mode ==  XCVR_RSM_STEP_FCS) || (next_step_mode ==  XCVR_RSM_STEP_PK_PK))
            {
                /* Configure rttAntennaID to be used for CS SYNC packets */
                rtt_ant_id = lcl_hadm_utils_set_CS_SYNC_antenna(&hadm_meas);
#ifdef HADM_RTT_TYPE_3
                if ((hadm_config_p->rttTypes == HADM_RTT_TYPE_FRAC_32BITS_RS) && (next_step_mode ==  XCVR_RSM_STEP_PK_PK))
                {
                    /* Prepare DMA to receive RTT packet IQ samples: disable averaging, dma mask and (re)start DMA */
                    LCL_HAL_CLEAR_DMA_MASK_AND_AVG_WIN;
                    LCL_HAL_STOP_DMA;
                    LCL_HAL_RESTART_DSB;
                    LCL_HAL_START_DMA(LCL_DMA_PAGE_RXDIGIQ);
                }
#endif

            }
            else if (next_step_mode ==  XCVR_RSM_STEP_TN_TN)
            {
                /* Prepare antenna switching pattern for next step */
                lcl_hadm_utils_handle_antenna_permutation(&hadm_meas, step_no + 1, LCL_HAL_XCVR_RSM_IRQ_FC);
                /* Prepare T_PM extension for next step (first half) */
                lcl_hadm_utils_handle_t_pm_ext(&hadm_meas, step_no + 1, LCL_HAL_XCVR_RSM_IRQ_FC);
#ifdef HADM_RTT_TYPE_3
                if (hadm_config_p->rttTypes == HADM_RTT_TYPE_FRAC_32BITS_RS)
                {
                    /* Restore DBG RAM config for tones sampling */
                    LCL_HAL_SET_DMA_MASK_AND_AVG_WIN(hadm_meas.iq_avg_win - 1);
                }
#endif
            }
        }

        DEBUG_PIN1_TGL
        DEBUG_PIN1_TGL
          
        if ((rsm_irq & LCL_HAL_XCVR_RSM_IRQ_ABORT) ||
            ((rsm_irq & LCL_HAL_XCVR_RSM_IRQ_EOS) && (step_no < (hadm_rsm_config.num_steps-1))))
        {
            hadm_info_p->flags |= FLAGS_HADM_ABORT;
            hal_status = HADM_HAL_ABORTED;
            break; // the loop
        }
        else if (rsm_irq & LCL_HAL_TPM_TIMER_IRQ)
        {
            /* Handle Timer expiry */
            hadm_info_p->flags |= FLAGS_HADM_SYNC_ERROR;
            lcl_hadm_tpm_timer_stop();
            hal_status = HADM_HAL_ABORTED_SYNC;
            break; // the loop
        }

#if 0
        /* Due to KFOURWONE-761, the assert below cannot be reliably implemented */
        if ((rsm_mode == XCVR_RSM_TX_MODE) && (rsm_irq & LCL_HAL_XCVR_RSM_IRQ_FC))
        {
            /* RSM has already incremented its step no when in initiator mode */
            if (rsm_step_no > 0) rsm_step_no--;
        }
        assert(step_no == rsm_step_no); // ensure SW keeps track
#endif
        /* Execute specific actions at the end of T_FC or T_IP1 intervals, depending on step mode */
        if (step_mode == XCVR_RSM_STEP_FCS)
        {
            /* Alignment phase ends when next step is not a mode 0 */
            bool  align_phase_end = (step_no + 1) >= hadm_config_p->mode0Nb;

            if (rsm_irq & LCL_HAL_XCVR_RSM_IRQ_FC)
            {
                sync_info_p = &hadm_meas.sync_info[step_no];
                sync_info_p->valid = lcl_xcvr_hal_rsm_rtt_is_valid(step_no);
                sync_info_p->agc_idx = lcl_hal_xcvr_get_rxgain();
                sync_info_p->cfo = lcl_xcvr_hal_get_cfo(hadm_rsm_config.rate);

                if (sync_info_p->valid)
                {
                    subevent_synced = true;

                    /* If procedure has not locked its AGC yet (didn't get a subevent with at least one valid mode0) */
                    /* look for smallest gain (highest RSSI) amongst valid mode 0's */
                    if (!proc_agc_locked && (hadm_proc->agc_idx > sync_info_p->agc_idx))
                    {
                        /* Record AGC index and keep the smallest */
                        hadm_proc->agc_idx = sync_info_p->agc_idx;
                        /* Record the estimated CFO and channel used for CFO estimation */
                        hadm_proc->cfo = sync_info_p->cfo;
                        hadm_proc->cfo_channel = hadm_config_p->chModePmAntMap[step_no].channel;
                        hadm_info_p->sync_step_id = step_no;
                    } /* end of processing on first synced subEvent */
                }

                if (align_phase_end)
                {
                    /* Detect if all mode0 packets have been missed and abort (would only happen on initiator) */
                    if (!subevent_synced)
                    {
                          hadm_info_p->flags |= FLAGS_HADM_SYNC_ERROR;
                          hal_status = HADM_HAL_ABORTED_SYNC;
                          break; // the loop
                    }

                    /* Freeze AGC according to the index from the retained mode 0 */
                    lcl_hal_xcvr_set_rxgain(hadm_proc->agc_idx);
                
                    /* Compensate for the CFO on the PLL and time-grid drift (Initiator only) */
                    if ((hadm_config_p->role == HADM_ROLE_INITIATOR) && ((hadm_meas.debug_flags & HADM_DBG_FLG_CFO_COMP_DIS) == 0))
                    {
                        int32_t cfo = hadm_proc->cfo;
                        /* Compute ppm based on measured CFO */
                        hadm_proc->ppm = (int8_t)(cfo / ((int32_t)2402 + hadm_proc->cfo_channel));
#ifdef HADM_CFO_COMP_PER_STEP
                        cfo += ((int32_t)hadm_config_p->chModePmAntMap[step_no+1].channel - (int32_t)hadm_proc->cfo_channel) * hadm_proc->ppm;
#endif
                        /* Initial CFO compensation after last mode 0 */
                        status = XCVR_LCL_RsmCompCfo(-cfo);

                        if (gXcvrLclStatusSuccess == status)
                        {
                            /* Compute time adjustment periodicity */
                            time_adjustment = lcl_hadm_utils_compute_time_adjustement_period((int32_t)hadm_proc->ppm, &hadm_meas);
                        }
                        else /* don't try to adjust the time grid is CFO unreliable */
                        {
                            hadm_info_p->flags |= FLAGS_HADM_CFO_TOO_LARGE; /* CFO was too large and could not get compensated */
                        }
                    }


                    /* If needed reconfigure IQ capture with RXDIGIQ page */
                    if ((hadm_meas.iq_buff_size_mode0 > 0) && (hadm_meas.iq_buff_size > 0))
                    {
                        /* DBG RAM was capturing phases for mode 0's. Reconfigure it to IQs */
                        lcl_hal_xcvr_start_dbg_ram_capture(LCL_DMA_PAGE_RXDIGIQ, LCL_NO_DMA_START_TRIG, num_iq_per_step_per_ap * (hadm_meas.n_ap + 1U) * LCL_HAL_XCVR_IQ_SAMPLE_SIZE);
                    }
#ifdef HADM_RTT_TYPE_3
                    if (hadm_config_p->rttTypes == HADM_RTT_TYPE_FRAC_32BITS_RS)
                    {
                        LCL_HAL_ENABLE_64_BITS_PN; /* Enable 64bits PNs for the rest of the subEvent */
                        XCVR_2P4GHZ_PHY->RTT_CTRL &= ~(GEN4PHY_RTT_CTRL_HA_RTT_THRESHOLD_MASK); /* TODO */
                    }
#endif
                    /* Ensure that AGC & CFO has been adjusted before RSM starts next step activity */
                    if (lcl_xcvr_hal_check_rsm_state(LCL_HAL_XCVR_RSM_STATE_FC, rsm_mode) == false)
                    {
                        assert(0);
                        hadm_info_p->flags |= FLAGS_HADM_SW_SCHED_ERROR;
                    }
                }
                  
                sync_info_p->rssi = lcl_hal_xcvr_get_nb_rssi();
                sync_info_p->rssi_raw = lcl_hal_xcvr_get_rssi_raw();
                hadm_rssi_buffer[step_no] = sync_info_p->rssi;
                
                /* first mode0 is done, stop supervision timer */
                if (step_no == 0)
                {
                    lcl_hadm_tpm_timer_stop();
                    lcl_hal_xcvr_setup_rssi_continuous(false); /* fallback to one shot RSSI after first mode 0 */
                }
            }

            else if (rsm_irq & LCL_HAL_XCVR_RSM_IRQ_IP)
            {
                if (step_no == 0)
                {
                    /* sync delay is HADM_HAL_SW_POLL_US + TPM timestamp - pkt_header_duration - aa_match_delay */
                    hadm_meas.result_p->syncDelayUs = HADM_RTT_TS_TO_US(timestamp);
                }
                hadm_meas.ts_buff_tmp_p[step_no] = ((uint16_t)rtt_ant_id << HADM_RTT_ANT_ID_SHIFT); /* store current RTT ant id */
#ifdef RSM_WA_FOR_RX_SETLLING_LATENCY_KFOURWONE_2175
                if (align_phase_end)
                {
                    lcl_hal_xcvr_restore_rsm_t_fc();
                }
#endif
            }
        }
        else 
        {
            if (step_mode == XCVR_RSM_STEP_PK_PK)
            {
                /* Handle ToF TS collection by SW sinc HW is not managing them in packet RAM ... */
                if (tpm_has_triggered)
                {
                    if (rsm_mode == XCVR_RSM_TX_MODE)
                    {
                        /* Initiator collects TX TS first (on T_IP1 or T_IP2 for PK_TN_TN_PK) and RX TS second (on T_FC) */
                        if (rsm_irq & LCL_HAL_XCVR_RSM_IRQ_IP)
                        {
                            timestamp1 = timestamp;
                            /* Fix RTT timestamp if needed (time adjustment has occured)  */
                            if (time_adjustment_done)
                            {
                                timestamp1 -= time_adjustment * 32;
                            }
                            hadm_meas.ts_buff_tmp_p[step_no] = ((uint16_t)rtt_ant_id << HADM_RTT_ANT_ID_SHIFT); /* store current RTT ant id */
                        }
                        else if (rsm_irq & (LCL_HAL_XCVR_RSM_IRQ_FC | LCL_HAL_XCVR_RSM_IRQ_EOS))
                        {
                            timestamp2 = timestamp;
                            hadm_rssi_buffer[step_no] = lcl_hal_xcvr_get_nb_rssi();
                        }
                    }
                    else
                    {
                        /* Reflector collects RX TS first (on T_IP1 or T_IP2 for PK_TN_TN_PK) and TX TS second (on T_FC) */
                        if (rsm_irq & LCL_HAL_XCVR_RSM_IRQ_IP)
                        {
                            timestamp1 = timestamp;
                            hadm_rssi_buffer[step_no] = lcl_hal_xcvr_get_nb_rssi();
                            hadm_meas.ts_buff_tmp_p[step_no] = ((uint16_t)rtt_ant_id << HADM_RTT_ANT_ID_SHIFT); /* store current RTT ant id */
                        }
                        else if (rsm_irq & (LCL_HAL_XCVR_RSM_IRQ_FC | LCL_HAL_XCVR_RSM_IRQ_EOS))
                        {
                            timestamp2 = timestamp;
                        }
                    }

                    assert(((rsm_irq & LCL_HAL_XCVR_RSM_IRQ_FC) && (rsm_irq & LCL_HAL_XCVR_RSM_IRQ_IP)) == false); // if both are set, something went wrong
                }
                else
                {
                    hadm_info_p->flags |= FLAGS_HADM_RTT_TS_ERROR;
                }
                
                /* RTT post-processing happens on T_FC interrupt because both TS needs to be available */
                if (rsm_irq & (LCL_HAL_XCVR_RSM_IRQ_FC | LCL_HAL_XCVR_RSM_IRQ_EOS))
                {
#ifdef RSM_DEBUG
                    lcl_hadm_utils_populate_rtt_dbg_buff(timestamp1, timestamp2);
#endif
                    /* End of step reached, compute TS difference and reset TS */
                    uint16_t ts_diff;
                    ts_diff = (uint16_t) (timestamp2 - timestamp1);
                    hadm_meas.ts_buff_tmp_p[step_no] |= (ts_diff & HADM_RTT_TS_MASK);

                    assert(((ts_diff & HADM_RTT_ANT_ID_MASK) == 0) || ((hadm_info_p->flags & FLAGS_HADM_RTT_TS_ERROR) != 0));
                    timestamp1 = 0;
                    timestamp2 = 0;
                }
            }
            if (step_mode == XCVR_RSM_STEP_TN_TN)
            {
                if (rsm_irq & LCL_HAL_XCVR_RSM_IRQ_IP)
                {
                    /* Prepare T_PM extension for 2nd half of current step */
                    lcl_hadm_utils_handle_t_pm_ext(&hadm_meas, step_no, LCL_HAL_XCVR_RSM_IRQ_IP);
                }
                
                /* IQ post-processing happens on T_IP interrupt for reflectors and T_FC for initiators */
                /* This provides more time (T_PM) before next step starts */
                if (((rsm_mode == XCVR_RSM_TX_MODE) && (rsm_irq & (LCL_HAL_XCVR_RSM_IRQ_FC | LCL_HAL_XCVR_RSM_IRQ_EOS))) || 
                   ((rsm_mode == XCVR_RSM_RX_MODE) && (rsm_irq & LCL_HAL_XCVR_RSM_IRQ_IP)))
                {
                     /* Capture for curent step must be finished */
                    if ((hadm_meas.iq_buff_size > 0) && (!lcl_hal_xcvr_is_dbg_ram_capture_finished()))
                    {
                        assert(0);
                        hadm_info_p->flags |= FLAGS_HADM_SW_SCHED_ERROR;
                    }
                    hadm_rssi_buffer[step_no] = lcl_hal_xcvr_get_nb_rssi();

                    /* Do some post-processing */
                    if (hadm_meas.iq_avg_win != 0)
                    {
                        lcl_hadm_utils_compute_pct_and_tqi(num_iq_per_step_per_ap, hadm_meas.n_ap + 1U, &capture_buffer_p, &tone_quality_buffer_p);
                    }
                    else /* IQ averaging is disabled, we don't have enough time to compute PCT and TQI */
                    {
                        lcl_hadm_utils_compute_pct(num_iq_per_step_per_ap, hadm_meas.n_ap + 1U, &capture_buffer_p, &tone_quality_buffer_p);
                    }
                }
            }
        }

#ifdef RSM_DEBUG
        lcl_hadm_utils_populate_rsm_dbg_buff(step_no, rsm_irq);
#endif

        if (rsm_irq & LCL_HAL_XCVR_RSM_IRQ_FC)
        {
            step_no++;

            /* Determine next step's mode */
            step_mode = (XCVR_RSM_FSTEP_TYPE_T)hadm_config_p->chModePmAntMap[step_no].mode;

            if (step_mode ==  XCVR_RSM_STEP_TN_TN)
            {
                /* Restart DBG_RAM capture if required (not last step) */
                lcl_hal_xcvr_start_dbg_ram_capture(LCL_DMA_PAGE_RXDIGIQ, LCL_NO_DMA_START_TRIG, num_iq_per_step_per_ap * (hadm_meas.n_ap + 1U) * LCL_HAL_XCVR_IQ_SAMPLE_SIZE);
            }
        }
        
        if (rsm_irq & LCL_HAL_XCVR_RSM_IRQ_EOS)
        {
            step_no++;
            break; /* end of sequence reached, exit the loop */
        }
    } /* end while step_no */
    
    DEBUG_PIN1_SET

    /* Restore dtest page */
    (void)lcl_hadm_utils_dtest_set_page(dtest_page_backup);

#ifdef RSM_DEBUG
    if ((hal_status == HADM_HAL_SUCCESS) && (lcl_xcvr_hal_check_rsm_state(LCL_HAL_XCVR_RSM_STATE_IDLE, rsm_mode) == false))
    {
        hadm_info_p->flags |= FLAGS_HADM_SW_SCHED_ERROR;
    }
#endif

    /* RSM block shutdown  */
    XCVR_LCL_RsmStopAbort((hal_status != HADM_HAL_SUCCESS));

    /* Stop IQ capture */
#ifndef RSM_DBG_IQ
    if ((hadm_meas.iq_buff_size > 0) || (hadm_meas.iq_buff_size_mode0 > 0))
#endif
    {
#ifndef RSM_DBG_IQ
        if ((hadm_meas.debug_flags & HADM_DBG_FLG_IQ_DMA)
#ifdef HADM_RTT_TYPE_3
            || (hadm_config_p->rttTypes == HADM_RTT_TYPE_FRAC_32BITS_RS)
#endif
           )
#endif
        {
            LCL_HAL_DISABLE_DMA;
        }
        LCL_HAL_STOP_DBGRAM;
    }
    /* Disable antenna switching */
    LCL_HAL_STOP_LCL;
    LCL_HAL_SET_ANTENNA_PORT(hadm_device.ant2gpio[0]);
    
    lcl_hadm_measurement_teardown(hadm_config_p);

#else /* SIMULATOR */
    step_no = hadm_rsm_config.num_steps;
    hadm_meas.result_p->syncDelayUs = HADM_1ST_BIT_TO_AA_MATCH_DURATION_US(hadm_meas.config_p->rttPhy) - HADM_HAL_SW_POLL_US + 1U;
#endif /* SIMULATOR */

    DEBUG_PIN0_PULSE

    /* Perform post-capture remaining actions:
     *   - Populate hadm_info fields related to mode 0
     *   - Read IQ buffer for MCIQ
     *   - final computation for syncDelayUs
     *   - Compute fractional delay and adjust timestamps diff for ToF (maybe do it on the fly later on?)
     *   - Populate debug flags
     */   
    if (hal_status == HADM_HAL_SUCCESS)
    {       
        assert(step_no == hadm_rsm_config.num_steps); // Ensure RSM sequence ran up to completion

        /* Run post-processing on timestamps and IQs */
        status += lcl_hadm_measurement_post_processing(hadm_proc, rsm_mode, hadm_rsm_config.rate, hadm_info_p);

        /* Compute final value for syncDelayUs which represents time starting from end of polling = RsmGo + HADM_HAL_SW_POLL_US */
        /* syncOffsetUs = HADM_HAL_SW_POLL_US + TPM timestamp - pkt_header_duration - aa_match_delay */
        assert((HADM_HAL_SW_POLL_US + hadm_meas.result_p->syncDelayUs) > (HADM_1ST_BIT_TO_AA_MATCH_DURATION_US(hadm_meas.config_p->rttPhy)));
        hadm_meas.result_p->syncDelayUs = HADM_HAL_SW_POLL_US + hadm_meas.result_p->syncDelayUs - HADM_1ST_BIT_TO_AA_MATCH_DURATION_US(hadm_meas.config_p->rttPhy);
#ifdef HADM_ENABLE_DEBUG_PINS
        /* Add one uS in case GPIO debug are enabled, spent between poll and TPM start */
        hadm_meas.result_p->syncDelayUs += 1U;
#endif
    }
    else
    {
        hadm_meas.result_p->syncDelayUs = 0;
    }

    /* Populate hadm info from retained mode 0 */
    if (hadm_info_p->sync_step_id < HADM_MAX_NB_STEPS_MODE0) 
    {
        sync_info_p = &hadm_meas.sync_info[hadm_info_p->sync_step_id];
        hadm_info_p->sync_rssi = sync_info_p->rssi;
        hadm_info_p->sync_cfo = hadm_proc->cfo;
        hadm_info_p->sync_rxgain = hadm_proc->agc_idx;
    }
   
    //hadm_info_p->xtal_trim = (int32_t)XCVR_GetXtalTrim();
    //avoid use XCVR_GetXtalTrim which use RFMC and requires CM33 wakeup
    hadm_info_p->xtal_trim = 0U;
    hadm_info_p->temperature = hadm_device.current_temperature;

    if (hadm_proc->agc_idx <= LCL_HAL_XCVR_AGC_INDEX_MAX)
        hadm_info_p->agc_delay = (uint16_t)lcl_hal_xcvr_tof_get_agc_delay(hadm_proc->agc_idx);
    else
        hadm_info_p->flags |= FLAGS_HADM_AGC_NOT_FROZEN;

    if (hadm_info_p->sync_rssi < -90)
        hadm_info_p->flags |= FLAGS_HADM_NO_SIGNAL;

    if(status != gXcvrLclStatusSuccess)
        hadm_info_p->flags |= FLAGS_HADM_XCVR_API_ERROR;

    if(lcl_hal_xcvr_get_pll_lock_error_flags() & (LCL_HAL_XCVR_PLL_CTFF_MASK | LCL_HAL_XCVR_PLL_CSFF_MASK | LCL_HAL_XCVR_PLL_FTFF_MASK))
        hadm_info_p->flags |= FLAGS_HADM_PLL_ERROR;
    
    if (hadm_config_p->typeFlags & HADM_SUBEVT_LAST)
    {
        /* End of procedure, clean context */
        hadm_proc->is_proc_init_done = false;
    }
    
    DEBUG_PIN1_CLR
    DEBUG_PIN0_CLR

    /* Notify the LL for EOS */
    BLE_HADM_NotifyLL(hadm_meas.result_p, HADM_EVENT_EOS, hal_status);

    return hal_status;
}

void lcl_hadm_stop_procedure(uint8 connIdx)
{
    if (connIdx == hadm_meas.config_p->connIdx)
    {
        lcl_hadm_stop_measurement(hadm_meas.config_p);
    }
}

void lcl_hadm_stop_measurement(const BLE_HADM_SubeventConfig_t *config)
{
    DEBUG_PIN0_SET

    /* Make sure all HW blocks are stopped */
    XCVR_LCL_RsmStopAbort(TRUE);
    LCL_HAL_STOP_LCL;
#ifndef RSM_DBG_IQ
    if ((hadm_meas.debug_flags & HADM_DBG_FLG_IQ_DMA)
#ifdef HADM_RTT_TYPE_3
        || (hadm_meas.config_p->rttTypes == HADM_RTT_TYPE_FRAC_32BITS_RS)
#endif
       )
#endif
    {
        LCL_HAL_DISABLE_DMA;
    }
    LCL_HAL_STOP_DBGRAM;

    lcl_hadm_measurement_teardown(hadm_meas.config_p);

    BLE_HADM_ReleaseResultsBuffer(&hadm_meas.result_p);
    BLE_HADM_ReleaseConfigBuffer((BLE_HADM_SubeventConfig_t **)&hadm_meas.config_p);

    hadm_procs[config->connIdx].is_proc_init_done = false;
    hadm_meas.is_config_valid = false;

    DEBUG_PIN0_CLR
}

/* === Implementation (static) ============================================= */
xcvrLclStatus_t lcl_hadm_measurement_post_processing(hadm_proc_t *hadm_proc, XCVR_RSM_RXTX_MODE_T rsm_mode, XCVR_RSM_SQTE_RATE_T rate, hadm_info_t *hadm_info_p)
{
    uint32_t step_no, m, step_mode0_no = 0, rtt_pkt_no = 0;
    xcvrLclStatus_t status = gXcvrLclStatusSuccess;
    xcvr_lcl_rtt_data_t rtt_data;
    uint32_t *rtt_ram_ptr = (uint32_t *)RSM_RTT_RAM;
    uint8_t *res_buff_in_p = hadm_meas.result_p->resultBuffer;
    uint8_t *res_buff_p = res_buff_in_p;
    uint32_t *capture_buffer_p = &capture_buffer[0];
    uint16_t *tone_quality_buffer_p = &tone_quality_buffer[0];
    uint32_t ts_delay_hns; /* rtt latency (half ns) to remove form timestamps difference */
    int8_t phase_population_idx = 0;

    /* RTT: Compute RTT local contributions */
    ts_delay_hns = lcl_hadm_utils_calc_ts_delay(hadm_meas.config_p, &hadm_device);

#ifndef SIMULATOR
    /* Determine phase population if necessary */
    if (hadm_meas.config_p->phaseCont == HADM_PHASE_CONT_LOOPBACK)
    {
        phase_population_idx = lcl_hadm_utils_determine_phase_population(&hadm_proc->dc_residual[hadm_meas.config_p->subeventIdx]);
        
        if (phase_population_idx == (-1))
        {
            phase_population_idx = 0;
            assert(0);
            hadm_info_p->flags |= FLAGS_HADM_PHASE_AMBIGUITY_UNRESOLVED;
        }
    }
#endif
    
    for (step_no = 0; step_no < hadm_rsm_config.num_steps; step_no++)
    {
        XCVR_RSM_FSTEP_TYPE_T step_mode = (XCVR_RSM_FSTEP_TYPE_T)hadm_meas.config_p->chModePmAntMap[step_no].mode;

        /* Store Step_Data "header" */
        *res_buff_p++ = step_mode; /* Step_Mode */
        *res_buff_p++ = hadm_meas.config_p->chModePmAntMap[step_no].channel; /* Step_Channel */

        /* Encode HCI Step_Data based on CS specification */
        if (step_mode == XCVR_RSM_STEP_FCS)
        {
            /*
              Mode0 Initiator:
              . Packet_Quality
              . Packet_RSSI
              . Packet_Antenna
              . Measured_Freq_Offset (as measured relative to the other device)
              Mode0 reflector:
              . Packet_Quality
              . Packet_RSSI
              . Packet_Antenna
            */
            bool vld = hadm_meas.sync_info[step_mode0_no].valid;
             /* (Hz*100) / MHz  => 0.01 ppm unit */
            int32_t ppm = (hadm_meas.sync_info[step_mode0_no].cfo * 100) / (int32_t)HADM_CHAN_NUM_TO_MHZ(hadm_meas.config_p->chModePmAntMap[step_mode0_no].channel);
            /* Store Step_Data_Len */
            *res_buff_p++ = BLE_HADM_STEP0_REPORT_SIZE(hadm_meas.config_p->role);
            /* Store Packet_AA_Quality (1 byte), Packet RSSI (1 byte), CFO (2 bytes, Measured CFO in 0.01 ppm), Packet_antenna (1 byte) */
            *res_buff_p++ = HADM_GET_RTT_AA_QUALITY(vld, 1);
            *res_buff_p++ = HADM_GET_RTT_RSSI(vld, hadm_meas.sync_info[step_mode0_no].rssi);
            /* Store Packet_antenna (1 byte) */
            *res_buff_p++ = (uint8_t)(hadm_meas.ts_buff_tmp_p[step_no] >> HADM_RTT_ANT_ID_SHIFT) + 1U;
            if (hadm_meas.config_p->role == HADM_ROLE_INITIATOR)
            {
                /* Store Measured_Freq_Offset (2 bytes) */
                HADM_SET_RTT_CFO(vld, ppm, res_buff_p);
                if (hadm_info_p->sync_step_id == step_mode0_no)
                {
                    /* This is the mode 0 retained for synch, copy CFO into frequencyCompensation */
                    hadm_meas.result_p->frequencyCompensation = (*(res_buff_p-1U) << 8U) | (*(res_buff_p-2U));
                }
            }
            rtt_ram_ptr++;
            step_mode0_no++;
            rtt_pkt_no++;
        }
        else 
        {
            if (step_mode == XCVR_RSM_STEP_PK_PK)
            {
                /*
                  RTT type0 data: mode1
                  . Packet_Quality
                  . Packet_NADM
                  . Packet_RSSI
                  . ToA_ToD_Initiator or ToD_ToA_Reflector
                  . Packet_Antenna
                */
                int32_t frac_delay = 0;
                int32_t rtt_ts = 0;
                status += XCVR_LCL_UnpackRttResult((xcvr_lcl_rtt_data_raw_t *)rtt_ram_ptr, &rtt_data, rate);
                /* TPM timestamp unit is 1/32MHz */
                if (rtt_data.rtt_vld && rtt_data.rtt_found)
                {
                    hadm_meas.nb_rtt_valid++;

                    DEBUG_PIN1_SET
#ifdef HADM_RTT_TYPE_3
                    if (hadm_meas.config_p->rttTypes == HADM_RTT_TYPE_FRAC_32BITS_RS)
                        frac_delay = 0;
                    else
#endif
                    {
                        uint32_t aa = (rsm_mode == XCVR_RSM_TX_MODE) ? hadm_meas.config_p->pnList[rtt_pkt_no].pn2 : hadm_meas.config_p->pnList[rtt_pkt_no].pn1;

                        /* Compute integer and fractional adjustment in ns */
                        frac_delay = lcl_hadm_hartt_compute_fractional_delay((uint32_t)hadm_rsm_config.rate, aa, rtt_data.p_delta, rtt_data.int_adj);
                        
                        if (rsm_mode == XCVR_RSM_RX_MODE)
                        {
                            frac_delay = -frac_delay; /* On reflector side, substract frac delay */
                        }
                    }

                    DEBUG_PIN1_CLR
                    rtt_ts = hadm_meas.ts_buff_tmp_p[step_no] & HADM_RTT_TS_MASK; /* in ticks */
                    rtt_ts = HADM_RTT_TS_TO_NS(rtt_ts); /* in ns */
                    /*  Compute RTT time diff */
                    rtt_ts = rtt_ts*2U - ts_delay_hns + frac_delay*2U; /* in half ns to keep ts_delay_hns precision */
                    if ((rtt_ts < -32768) || (rtt_ts > 32767)) {
                        assert(false);
                        rtt_data.rtt_vld = false;
                    }
                }
                /* Store Step_Data_Len */
                *res_buff_p++ = BLE_HADM_STEP1_REPORT_SIZE;
                /* Store Packet_AA_Quality (1 byte) */
                *res_buff_p++ = HADM_GET_RTT_AA_QUALITY(rtt_data.rtt_vld, rtt_data.rtt_found);
                /* Store NADM (1 byte) */
                *res_buff_p++ = 0xFF;
                /* Store Packet_RSSI (1 byte) */
                *res_buff_p++ = HADM_GET_RTT_RSSI(rtt_data.rtt_vld, hadm_rssi_buffer[step_no]);
                /*  Store RTT time diff (2 bytes) */
                HADM_SET_RTT_TS_DIFF(rtt_data.rtt_vld, rtt_ts, res_buff_p);
                /* Store Packet_Antenna (1 byte) */
                *res_buff_p++ = (uint8_t)(hadm_meas.ts_buff_tmp_p[step_no] >> HADM_RTT_ANT_ID_SHIFT) + 1U;
                rtt_ram_ptr++;
                rtt_pkt_no++;
            }
            if (step_mode == XCVR_RSM_STEP_TN_TN)
            {
                /*
                  Tone data: mode2
                  . Antenna_Permutation_Index
                  . Tone_PCT[k]
                  . Tone_Quality_Indicator[k]
                */
                uint8_t t_pm_ext = hadm_meas.config_p->chModePmAntMap[step_no].pm_ext;
                /* Keep pm extension bit corresponding to peer device role */
                if (hadm_meas.config_p->role == HADM_ROLE_REFLECTOR)
                {
                    t_pm_ext >>= 1U;
                }
                t_pm_ext &= 0x1U;
                /* Compute RPL once per event */
                if (hadm_meas.result_p->referencePwrLevel == HADM_INVALID_REFERENCE_POWER_LEVEL)
                {
                    hadm_meas.result_p->referencePwrLevel = lcl_hadm_utils_compute_rpl(*capture_buffer_p, hadm_rssi_buffer[step_no]);
                }

                /* Store Step_Data_Len */
                *res_buff_p++ = BLE_HADM_STEP2_REPORT_SIZE(hadm_meas.n_ap);
                /* Store Antenna_Permutation_Index - 1 byte*/
                *res_buff_p++ = hadm_meas.config_p->chModePmAntMap[step_no].ant_perm;
                /* Store PCT[m], 3 bytes each 22 significant bits  +  Tone Quality Indicator [m] (1 byte each) */
                for(m = 0; m <= hadm_meas.n_ap; m++)
                {
                    HADM_SET_RTP_PCT(*capture_buffer_p, res_buff_p, phase_population_idx);
                    capture_buffer_p++;
                    HADM_SET_RTP_TONE_QUALITY(*tone_quality_buffer_p, res_buff_p, m==hadm_meas.n_ap, t_pm_ext);
                    tone_quality_buffer_p++;
                }
            }
        }
    }

    assert(gXcvrLclStatusSuccess == status);
    assert((res_buff_p - res_buff_in_p) <= hadm_meas.result_p->resultBufferSize); /* check we did not write out of bounds */

    return status;
}

static void lcl_hadm_measurement_setup(hadm_proc_t *hadm_proc, const BLE_HADM_SubeventConfig_t *hadm_config)
{
    lcl_hal_xcvr_hadm_init(hadm_config, &hadm_proc->dc_residual[hadm_config->subeventIdx]);
    lcl_hadm_disable_interrupts(); /* lock IRQs for SW scheduler */
    lcl_hadm_start_tpms();
}

/* Note: PLL_OFFSET_CTRL is restored as part of XCVR_LCL_RsmPLLBackup */
static void lcl_hadm_measurement_teardown(const BLE_HADM_SubeventConfig_t *hadm_config)
{
    lcl_hal_xcvr_hadm_deinit(hadm_config);
    lcl_hadm_restore_interrupts();
    lcl_hadm_stop_tpms();
}

/* EOF */
