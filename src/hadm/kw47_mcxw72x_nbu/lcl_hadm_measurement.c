/**
 * @file lcl_hadm_measurement.c
 *
 * This file implements platform dependent HADM functionality
 *
 */
/*
 * Copyright 2024 NXP
 * SPDX-License-Identifier: BSD-3-Clause
 */
 
/* === Includes ============================================================ */
#include <stdint.h>
#include <stdbool.h>
#include "fsl_os_abstraction.h"
#include "fsl_ltc.h" /* LTC driver for AES */
#include "EmbeddedTypes.h"
#include "nxp_xcvr_lcl_ctrl.h"
#include "nxp_xcvr_lcl_step_mgr.h"
#include "lcl_hadm_measurement.h"
#include "lcl_xcvr_hal.h"
#include "lcl_hadm_utils.h"
#include "lcl_hadm_hartt.h"
#include "nxp2p4_xcvr.h"
#ifdef SIMULATOR
#include "lcl_xcvr_simu.h"
#endif


#if !defined(CPU_KW47B42Z83AFTA_cm33_core1) && !defined(CPU_KW47B42ZB7AFTA_cm33_core1) && !defined(CPU_MCXW727CMFTA_cm33_core1)
#error this code is supposed to be compiled on KW47 core1
#endif

/* === Macros ============================================================== */
#define T_SLOT_US  (625U)

/* TKT0598229: disabled due to HW issue on KW47 A0 */
//#define HADM_CFO_COMP_PER_STEP /* Enable CFO compensation refinment on each step */

/* The following timings have been measured in debug mode with IAR 9.30.1 */
/* TODO OJE: Exact HAL timings need to be measured on KW47 */
/*! Time needed to execute BLE_HADM_Calibrate()
 * in order to perform the radio calibrations (HPM and DCOC)
 */
#define HADM_HAL_HPM_CAL_US  (180U)
#define HADM_HAL_DCOC_CAL_US (410U)
#define HADM_HAL_RADIO_CAL_US (HADM_HAL_DCOC_CAL_US + HADM_HAL_HPM_CAL_US)

/*! Time needed by the HAL to execute BLE_HADM_EventConfigApply(). Measured */
#define HADM_HAL_CONFIG_APPLY_US (100U)

/*! Time needed by the HAL to execute BLE_HADM_EventStart(). Measured */
#define HADM_HAL_START_US (215U)

/*! Time needed by the HAL to execute BLE_HADM_EventConfigApply() + up to BLE_HADM_EventStart() call
 *  Does not need to be exact, should cover longest execution path
 */
#define HADM_HAL_PREPARE_US (MAX(HADM_HAL_START_US + HADM_HAL_CONFIG_APPLY_US, T_SLOT_US))

/*! Time time be programmed to RSM_TRIGGER_DELAY
 *  Does not need to be exact, should cover BLE_HADM_EventStart() execution
 */
#define HADM_HAL_RSM_TRIGGER_DELAY (HADM_HAL_START_US + 50U /* margin */)

/*! First RSM Rx window is a bit late.
 * In order to compensate that, we delay RSM on initiator side so that both RSM
 * can still be triggered at the same time from SW perspective.
 */
#define HADM_HAL_RSM_INITIATOR_TRIGGER_DELAY (5U)

/* Contengency time to make sure mode 0 AA is caught */
#define HADM_MODE0_TIMEOUT_MARGIN_US (5U)

/* === Types =============================================================== */

/* === Globals ============================================================= */

/* Define this to enable some hack that will cause mode0 to be transmitted on unexpected channel from time to time on each device */
//#define UT_CORRUPT_MODE0
#ifdef UT_CORRUPT_MODE0
int hadm_ut_config_cnt;
#endif

//#define RTT_DEBUG
#ifdef RTT_DEBUG
int32_t rtt_tpm_dbg_buffer[HADM_MAX_NB_STEPS];
int32_t rtt_frac_dbg_buffer[HADM_MAX_NB_STEPS];
int16_t rtt_p_delta_dbg_buffer[HADM_MAX_NB_STEPS];
int32_t rtt_int_adj_dbg_buffer[HADM_MAX_NB_STEPS];
int32_t rtt_common_stat_dbg_buffer[HADM_MAX_NB_STEPS];
#endif

static BLE_HADM_HalProperties_t hadm_hal_properties;

/* This reflects the capabilities of our IC */
/* CS capabilities might be further refined by the controller based on external elements capabilities (e.g antenna system) */
static const BLE_HADM_HalCapabilities_t hadm_hal_capabilities = {
    .stepModeSupported          = 1U, /* step mode 3 is supported */
    .numAntennasSupported       = HADM_MAX_NB_ANTENNAS,
    .nNumAPSupported            = HADM_MAX_NB_ANTENNA_PATHS,
    .RTT_Capability             = 0x5, /* RTT_Coarse_N field refers to the 10ns precision requirement (AA only and random seq) */
    .RTT_Coarse_N               = 10, /* Number of RTT steps to satisfy the precision requirement. */ 
    .RTT_Sounding_N             = 0, /* not supported */
    .RTT_Random_Sequence_N      = 10, /* Number of RTT steps to satisfy the precision requirement. */
    .NADM_Sounding_Capability   = 0, /* NADM not supported */
    .NADM_Random_Sequence_Capability = 1, /* NADM supported */
    .PHYSupported               = 1<<1, /* 2Mbps PHY supported (bit #1) */
    .T_SW_TimeSupported         = 2, /* 2us: OJE TODO confirm OK for ramp-up/down */
    .FAErequired                = 0, /* no FAE */
    .InlinePhaseReturn          = 0U, /* to enable by _LE_HADM_NXP_Config */
    /* note: mandatory timings are not included in capabilities */
    .T_IP1_TimesSupported       = 0x0048, /* T_IP1=80 or 40us */
    .T_IP2_TimesSupported       = 0x0048, /* T_IP2=80 or 40us */
    .T_FCS_TimesSupported       = 0x0050, /* T_FCS=80 or 50us */
    .T_PM_TimesSupported        = 0x0002, /* T_PM=20us */
};

/* Contains data associated to this device */
hadm_device_t hadm_device;
/* Contains data that needs to be maintained across several subevents of the same procedure.
 * Several procedures may run in parallele (multi-connection).
 */
static hadm_proc_t hadm_procs[HADM_MAX_NB_CONNECTIONS];
/* Contains data for current subevent */
static hadm_meas_t hadm_meas[HADM_MAX_NB_SIMULT_SUBEVENTS];

const uint8_t rtt_type_2_payload_size[7U] = {0U, 1U, 3U, 1U, 2U, 3U, 4U}; /* in 32 bits words */

/* === Externals =========================================================== */

/* === Prototypes ========================================================== */

static void lcl_hadm_measurement_shutdown(hadm_meas_t *hadm_meas_p, bool_t abort_subevent);
static void lcl_hadm_measurement_setup(hadm_proc_t *hadm_proc, hadm_meas_t *hadm_meas_p, const BLE_HADM_SubeventConfig_t *hadm_config);
static void lcl_hadm_measurement_teardown(const BLE_HADM_SubeventConfig_t *hadm_config);
static BLE_HADM_STATUS_t lcl_hadm_get_step_results(uint16 n_steps, hadm_meas_t *hadm_meas_p);
static BLE_HADM_STATUS_t lcl_hadm_set_steps_config(uint16 n_steps, hadm_meas_t *hadm_meas_p, bool_t update_rsm_ptr);
static hadm_meas_t *lcl_hadm_alloc_meas_instance(void);
static void lcl_hadm_free_meas_instance(hadm_meas_t *meas_p);
static hadm_meas_t *lcl_hadm_get_meas_instance(const BLE_HADM_SubeventConfig_t *config_p);
void lcl_hadm_irq_handler(void *userData, bool abort, uint32_t rsm_csr);

/* === Implementation (public) ============================================= */

BLE_HADM_STATUS_t lcl_hadm_init_procedure(uint8 connIdx)
{
    hadm_proc_t *hadm_proc;

    if (connIdx >= HADM_MAX_NB_CONNECTIONS)
    {
        assert(FALSE);
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

BLE_HADM_STATUS_t lcl_hadm_init(void)
{
    int i;
    BLE_HADM_STATUS_t hal_status = HADM_HAL_SUCCESS;

    lcl_hadm_utils_init_buffers();
    lcl_hadm_enable_lcl_interrupts();
    
    DEBUG_PIN0_CLR
    DEBUG_PIN1_CLR

    hadm_hal_properties.prepareMaxUs = HADM_HAL_PREPARE_US;

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
    hadm_hal_properties.txWarmupUs += HADM_HAL_RSM_TRIGGER_DELAY + HADM_HAL_RSM_INITIATOR_TRIGGER_DELAY;
    hadm_hal_properties.rxWarmupUs += HADM_HAL_RSM_TRIGGER_DELAY + HADM_HAL_RSM_INITIATOR_TRIGGER_DELAY;

    for (i = 0; i < HADM_MAX_NB_SIMULT_SUBEVENTS; i++)
    {
        /* Static initialization of PKT RAM buffers */
        hadm_meas[i].pkt_ram.step_config.ram_type = 0;
        hadm_meas[i].pkt_ram.step_config.buff_len = HADM_HAL_PKT_RAM_CONFIG_CIRC_BUFF_SIZE;
        hadm_meas[i].pkt_ram.step_config.base_ptr = (uint32_t *)TX_PACKET_RAM + (i * HADM_HAL_PKT_RAM_CONFIG_CIRC_BUFF_SIZE);
        hadm_meas[i].pkt_ram.step_result.ram_type = 1;
        hadm_meas[i].pkt_ram.step_result.buff_len = HADM_HAL_PKT_RAM_RESULT_CIRC_BUFF_SIZE;
        hadm_meas[i].pkt_ram.step_result.base_ptr = (uint32_t *)RX_PACKET_RAM + (i * HADM_HAL_PKT_RAM_RESULT_CIRC_BUFF_SIZE);

        lcl_hadm_free_meas_instance(&hadm_meas[i]);
        
        /* Static initialization of RSM XCVR config (will be updated by ConfigApply() API) */
        hadm_meas[i].rsm_config.sniffer_mode_en = 0;
        hadm_meas[i].rsm_config.trig_sel = (XCVR_RSM_TRIG_T)6U; /* 110b - nbu trigger */
        hadm_meas[i].rsm_config.rxdig_dly = 0U;
        hadm_meas[i].rsm_config.txdig_dly = 0U;
        hadm_meas[i].rsm_config.use_rsm_dma_mask = false, /* disable dma_mask */
#if defined(SUPPORT_RSM_LONG_PN) && (SUPPORT_RSM_LONG_PN == 1)
        hadm_meas[i].rsm_config.rtt_len = XCVR_RSM_SQTE_PN32,
#endif /* defined(SUPPORT_RSM_LONG_PN) && (SUPPORT_RSM_LONG_PN == 1) */
        hadm_meas[i].rsm_config.rsm_dma_dly_pm = 0U,
        hadm_meas[i].rsm_config.rsm_dma_dur_pm = 0U,
        hadm_meas[i].rsm_config.pct_averaging_win = XCVR_RSM_PCT_AVG_WIN_4_SMPL;
        hadm_meas[i].rsm_config.pa_ramp_time = XCVR_RSM_PA_RAMP_0_USEC; /* OJE TODO */
        hadm_meas[i].rsm_config.disable_rx_sync = false;
        hadm_meas[i].rsm_config.iq_out_sel = XCVR_RSM_IQ_OUT_FRAC_CORR;

        /* Clear working variables */
        hadm_meas[i].debug_flags = 0;
        hadm_meas[i].iq_avg_win = 0;
        hadm_meas[i].iq_capture_win = 0;
        hadm_meas[i].num_ant = 0;
        hadm_meas[i].n_ap = 0;
        hadm_meas[i].iq_buff_size = 0;
        hadm_meas[i].iq_buff_size_mode0 = 0;
    }

    hadm_device.active_meas_p = NULL;
    
    for (i = 0; i < HADM_MAX_NB_ANTENNAS; i++)
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
    
    /* Enable required RSM interrupts for SW FSM */
    LCL_HAL_RSM_SET_IRQ_ENABLE_MASK(LCL_HAL_RSM_XCVR_IRQ_ENABLE_MASK);
    
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
    
    for (int i = 0; i < HADM_MAX_NB_ANTENNAS; i++)
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
   (void)hadm_config;
   return hadm_hal_properties.prepareMaxUs;
}

const BLE_HADM_HalCapabilities_t *lcl_hadm_get_capabilities(void)
{
    return &hadm_hal_capabilities;
}

BLE_HADM_STATUS_t lcl_hadm_check_config(const BLE_HADM_SubeventConfig_t *hadm_config)
{   
    if((hadm_config->stepsNb > HADM_MAX_NB_STEPS) || (hadm_config->stepsNb < HADM_MIN_NB_STEPS))
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

    if (hadm_config->subeventIdx >= HADM_NUM_SUBEVENTS_MAX)
        goto config_error;
    
    if (hadm_config->connIdx >= HADM_MAX_NB_CONNECTIONS)
        goto config_error;

    return HADM_HAL_SUCCESS;
    
config_error:
    return HADM_HAL_INVALID_ARGS;
}

/* Make sure the execution of this function is not disruptive on any HW module (RSM, TSM, LCL, etc.) as it might be called while */
/* another measuremnt instance is running */
BLE_HADM_STATUS_t lcl_hadm_configure(const BLE_HADM_SubeventConfig_t *hadm_config)
{
    uint32_t i;
    xcvrLclStatus_t status;
    hadm_meas_t *hadm_meas_p = lcl_hadm_alloc_meas_instance();
    assert(NULL != hadm_meas_p);
    xcvr_lcl_rsm_config_t *rsm_config_p = &hadm_meas_p->rsm_config;

    DEBUG_PIN0_SET

#ifdef UT_CORRUPT_MODE0
    hadm_ut_config_cnt++;
#endif

    if (!hadm_device.is_rsm_cal_done)
    {
        goto config_error; /* at this point all calibration must have ben performed */
    }

    hadm_meas_p->config_p = hadm_config; /* save config ptr */
    
    hadm_meas_p->debug_flags = hadm_config->debugFlags;
    
    /* Configure iq_avg_win according to T_PM: choose the closest smaller power of 2 to generate 4 IQ avg windows whatever
     * the configuration.
     * iq_avg_win is the number of samples (power of 2) per averaging window, based on
     *  4 samples per us when XCVR clocked for 1 Mbps
     *  8 samples per us when XCVR clocked for 2 Mbps
     * For instance with T_PM=20us: (4us)*4=16us with 4us of transition (discarded at start/end of T_PM)
     * - at 1 Mbps: 16 samples per avg window (4us)
     * - at 2 Mbps: 32 samples per avg window (4us)
     */

    switch (hadm_config->T_PM_Time)
    {
        case HADM_T_PM_40:
          hadm_meas_p->iq_avg_win = 5;
          break;
        case HADM_T_PM_20:
          hadm_meas_p->iq_avg_win = 4;
          break;
        case HADM_T_PM_10:
          hadm_meas_p->iq_avg_win = 3;
          break;
        default:
          goto config_error;
          break;
    }
    /* Capture window size in us, this will always result in 4 averaged values / AP / step */
    hadm_meas_p->iq_capture_win = (1 << hadm_meas_p->iq_avg_win);
#ifndef RSM_DBG_IQ
    if (hadm_meas_p->debug_flags & HADM_DBG_FLG_AVG_OFF)
#endif
    {
        hadm_meas_p->iq_avg_win = 0;
    }
#ifndef RSM_DBG_IQ
    else if (hadm_config->rttPhy == HADM_RTT_PHY_2MBPS)
    {
        /* Double IQ averaging window in order to get the same number of final (averaged) samples whatever the data rate */
        hadm_meas_p->iq_avg_win ++;
   }
#endif
    HADM_COMPUTE_NUM_ANTENNA(hadm_config->role, hadm_config->toneAntennaConfigIdx, hadm_meas_p->num_ant, hadm_meas_p->n_ap);
    
    /* Compute IQ buffer size */
    if (hadm_meas_p->debug_flags & HADM_DBG_FLG_IQ_DMA)
    {
        lcl_hadm_utils_compute_iq_buff_size(hadm_config, hadm_meas_p, (hadm_config->rttPhy == HADM_RTT_PHY_2MBPS) ? (RX_SAMPLING_RATE*SAMPLING_RATE_FACTOR_2MBPS):RX_SAMPLING_RATE);
    }
    /* Compute step mode durations */
    lcl_hadm_utils_compute_step_duration(hadm_config, hadm_meas_p->n_ap, hadm_meas_p->step_duration);

    /* RTT: Compute RTT local contributions */
    lcl_hadm_utils_calc_ts_delay(hadm_meas_p, &hadm_device);

    /* Configure rttAntennaID to be used during first mode 0 */
    /* If no recommendation from the host then apply crossed roud robin in the controller since it gives better perf */
    if (hadm_config->rttAntennaID >= HADM_RTT_ANT_ROUND_ROBIN)
        hadm_meas_p->rtt_antenna_id = 0U; /* start round robin */
    else
        hadm_meas_p->rtt_antenna_id = hadm_config->rttAntennaID - 1U; /* fixed antenna */
    
    /* Clear info structs */
    hadm_meas_p->info.rtt_dbg_buffer_nb = 0;
    hadm_meas_p->info.mciq_dbg_buffer_nb = 0;
    hadm_meas_p->info.sync_cfo = 0;
    hadm_meas_p->info.sync_rxgain = 0xFF;
    hadm_meas_p->info.sync_rssi = -128;
    hadm_meas_p->info.sync_step_id = 0xFF;
    hadm_meas_p->info.flags = 0;
    hadm_meas_p->info.xtal_trim = 0U; /* = (int32_t)XCVR_GetXtalTrim() avoid use XCVR_GetXtalTrim which use RFMC and requires CM33 wakeup */
    hadm_meas_p->info.temperature = hadm_device.current_temperature;
    hadm_meas_p->info.num_time_adj = 0;
    
    hadm_meas_p->pkt_ram.config_write_ptr = hadm_meas_p->pkt_ram.step_config.base_ptr;
    hadm_meas_p->pkt_ram.result_read_ptr = hadm_meas_p->pkt_ram.step_result.base_ptr;
    hadm_meas_p->pkt_ram.step_config.curr_page = 0;
    hadm_meas_p->pkt_ram.step_config.curr_step_idx = 0;
    hadm_meas_p->pkt_ram.step_config.max_step_size = LCL_HAL_PKT_RAM_STEP_CONFIG_MODE13_SIZE + rtt_type_2_payload_size[hadm_meas_p->config_p->rttTypes] * 2U; /* taking mode1/3 into account */
    hadm_meas_p->pkt_ram.step_result.curr_page = 0;
    hadm_meas_p->pkt_ram.step_result.curr_step_idx = 0;
    hadm_meas_p->pkt_ram.step_result.max_step_size = (LCL_HAL_PKT_RAM_STEP_RESULT_MODE01_SIZE + hadm_meas_p->n_ap + 1U); /* taking mode3 into account */
    /* Compute nb_steps_before_irq based on larget circ buffer step size (+ contingency (/2)) */
    hadm_meas_p->pkt_ram.nb_steps_before_irq = (HADM_HAL_PKT_RAM_CONFIG_CIRC_BUFF_SIZE / (MAX(hadm_meas_p->pkt_ram.step_config.max_step_size, hadm_meas_p->pkt_ram.step_result.max_step_size))) / 2U;
    if (hadm_meas_p->pkt_ram.nb_steps_before_irq > HADM_HAL_PKT_RAM_MAX_NB_STEPS_BEFORE_IRQ)
    {
        hadm_meas_p->pkt_ram.nb_steps_before_irq = HADM_HAL_PKT_RAM_MAX_NB_STEPS_BEFORE_IRQ;
    }
    hadm_meas_p->pkt_ram.nb_irq_steps_handled = 0;
    for (i=0; i < HADM_MAX_NB_STEPS_MODE0; i++)
    {
        hadm_meas_p->sync_info[i].valid = 0;
    }
    hadm_meas_p->data_in_flight_w_idx = 0;
    hadm_meas_p->data_in_flight_r_idx = 0;


    /* Prepare RSM configuration in RAM */
    DEBUG_PIN1_SET

    rsm_config_p->num_steps = hadm_meas_p->config_p->stepsNb;
    rsm_config_p->num_ant_path = hadm_meas_p->n_ap;
    rsm_config_p->rate = (XCVR_RSM_SQTE_RATE_T)hadm_meas_p->config_p->rttPhy;
    rsm_config_p->rsm_dma_dly_fm_ext = (HADM_T_FM - hadm_meas_p->iq_capture_win) >> 1; /* center capture window inside T_FM */
    rsm_config_p->rsm_dma_dur_fm_ext = hadm_meas_p->iq_capture_win;
    rsm_config_p->averaging_win = (hadm_meas_p->iq_avg_win == 0) ? XCVR_RSM_AVG_WIN_DISABLED : (XCVR_RSM_AVG_WIN_LEN_T) (hadm_meas_p->iq_avg_win - 1);
    rsm_config_p->trig_delay = HADM_HAL_RSM_TRIGGER_DELAY + ((hadm_config->role == HADM_ROLE_INITIATOR) ? HADM_HAL_RSM_INITIATOR_TRIGGER_DELAY : 0U); /* RSM trig_delay must cover start API execution time */
    rsm_config_p->t_fc = hadm_config->T_FCS_Time;
    rsm_config_p->t_ip1 = hadm_config->T_IP1_Time;
    rsm_config_p->t_ip2 = hadm_config->T_IP2_Time;
    rsm_config_p->t_sw = (lclTSw_t)hadm_config->T_SW_Time;
    rsm_config_p->rtt_type = (XCVR_RSM_RTT_TYPE_T)hadm_meas_p->config_p->rttTypes;
    rsm_config_p->role = (hadm_config->role == HADM_ROLE_INITIATOR) ? XCVR_RSM_TX_MODE : XCVR_RSM_RX_MODE;
    rsm_config_p->enable_inpr = (bool)hadm_config->inlinePhaseReturn;
    
    if (hadm_meas_p->config_p->mode != HADM_SUBEVT_TEST_MODE_PHASE_STAB)
    {
        rsm_config_p->op_mode = XCVR_RSM_SQTE_MODE;
        rsm_config_p->t_pm0 = ((hadm_meas_p->n_ap + 1U) * ((uint32_t)hadm_meas_p->config_p->T_PM_Time + (uint32_t)hadm_meas_p->config_p->T_SW_Time));
    }
    else
    {
        rsm_config_p->op_mode = XCVR_RSM_SQTE_STABLE_PHASE_TEST_MODE;
        rsm_config_p->t_pm0 = LCL_HAL_T_PM_MEAS; /* Set to T_PM_MEAS for stable phase test */
        hadm_meas_p->iq_buff_size_mode0 = 0;
        hadm_meas_p->iq_buff_size = 0;
        hadm_meas_p->iq_avg_win = 0;
    }
    
    /* Determine and program rx timeout (for reflector devices) = mode 0 duration */
    /* if window widening is larger than a mode0 step, disable rx timeout */
    if ((hadm_meas_p->config_p->mode == HADM_SUBEVT_MISSION_MODE) &&
        (hadm_config->rxWindowUs < (hadm_meas_p->step_duration[0] - hadm_config->T_FCS_Time - HADM_T_SY(hadm_config->rttPhy) - HADM_MODE0_TIMEOUT_MARGIN_US)))
    {
        rsm_config_p->mode0_timeout_usec = hadm_meas_p->step_duration[0] - hadm_config->T_FCS_Time;
    }
    else
    {
        rsm_config_p->mode0_timeout_usec = 0; /* means no timeout */
    }
    
    status = XCVR_LCL_ValidateRsmSettings(rsm_config_p);
    if (gXcvrLclStatusSuccess != status)
        goto config_error;

    DEBUG_PIN1_CLR
    DEBUG_PIN1_SET 

    /* Build first configuration steps in PKT RAM: mode 0 steps + next one (design choice) without pushing RSM pointers */
    lcl_hadm_set_steps_config(HADM_HAL_PKT_RAM_NB_STEPS_CONFIG_INITIAL(hadm_config->mode0Nb), hadm_meas_p, FALSE);
    
    /* Program RSM to start on NBU trigger + DELAY ahead of time */
    LCL_HAL_PROGRAM_RSM_TRIGGER(rsm_config_p->trig_sel, rsm_config_p->trig_delay);
    
    hadm_meas_p->state = HADM_HAL_MEAS_STATE_CONFIGURED;
    
    DEBUG_PIN1_CLR
    DEBUG_PIN0_CLR
      
    return HADM_HAL_SUCCESS;
    
config_error:

    assert(0);
    return HADM_HAL_FAIL;
}


BLE_HADM_STATUS_t lcl_hadm_run_measurement(const BLE_HADM_SubeventConfig_t *hadm_config_p)
{
    BLE_HADM_STATUS_t hal_status = HADM_HAL_SUCCESS;
    xcvrLclStatus_t status;

    hadm_meas_t *hadm_meas_p = lcl_hadm_get_meas_instance(hadm_config_p);
    hadm_proc_t *hadm_proc = &hadm_procs[hadm_config_p->connIdx];
    xcvr_lcl_rsm_config_t *rsm_config_p = &hadm_meas_p->rsm_config;
    uint32_t rsm_state;
       
    DEBUG_PIN0_SET
      
#ifdef UT_HADM_TRIGGER
    /* RSM should have started on NBU trigger and be in DELAY state */
    rsm_state = LCL_HAL_XCVR_GET_RSM_STATE;
    assert(rsm_state != LCL_HAL_XCVR_RSM_STATE_IDLE);
#endif

    /* At this point, the config must be valid and the procedure must have been initialized */
    if ((hadm_meas_p->state != HADM_HAL_MEAS_STATE_CONFIGURED) || (!hadm_proc->is_proc_init_done))
    {
        assert(false);
        return HADM_HAL_INVALID_ARGS;
    }
    
    /* Alloc result buffer */
    hadm_meas_p->result_p = lcl_hadm_utils_get_result_buffer();
    if (hadm_meas_p->result_p == NULL)
    {
        assert(false);
        return HADM_HAL_MEMORY_FULL;
    }
    
    DEBUG_PIN1_SET
      
    hadm_device.active_meas_p = hadm_meas_p; /* becomes active measurement */
    hadm_meas_p->state = HADM_HAL_MEAS_STATE_RUNNING;
        
    hadm_meas_p->result_p->connIdx = hadm_config_p->connIdx;
    hadm_meas_p->result_p->subeventIdx = hadm_config_p->subeventIdx; /* echo subeventIdx */
    hadm_meas_p->result_p->syncDelayUs = 0;

    DEBUG_PIN1_PULSE
      
    lcl_hadm_measurement_setup(hadm_proc, hadm_meas_p, hadm_config_p);
    
    DEBUG_PIN1_PULSE
    
    /* Configure Antenna switching */
    lcl_hadm_utils_configure_antenna_switching(hadm_meas_p);
    /* Configure TQI and start LCL if needed */
    if (rsm_config_p->op_mode != XCVR_RSM_SQTE_STABLE_PHASE_TEST_MODE)
    {
       if ((hadm_meas_p->debug_flags & HADM_DBG_FLG_AVG_OFF) == 0)
       {
            lcl_hal_xcvr_program_tqi(hadm_meas_p);
       }
       LCL_HAL_START_LCL;
    }
    
    DEBUG_PIN1_PULSE

    /* Start DMA if needed */
#ifndef RSM_DBG_IQ
    if ((hadm_meas_p->debug_flags & HADM_DBG_FLG_IQ_DMA) && ((hadm_meas_p->iq_buff_size > 0) || (hadm_meas_p->iq_buff_size_mode0 > 0)))
    {
        lcl_hal_xcvr_configure_dma_capture(LCL_START_DMA_ON_RSM_TRIGGER, 0, hadm_meas_p->iq_buff_size + hadm_meas_p->iq_buff_size_mode0, hadm_device.dma_debug_buff_size, hadm_device.dma_debug_buff_address);
        LCL_HAL_START_DMA(LCL_DMA_PAGE_RXDIGIQ);
    }
#else
    LCL_HAL_SET_IQ_CAPTURE_POINT(LCL_OUT_CH_FILTER_SEL);
    LCL_HAL_CLEAR_DMA_MASK_AND_AVG_WIN;
    lcl_hal_xcvr_configure_dma_capture(LCL_START_DMA_ON_TSM_RX_DIG_EN, 0, 0x2000, hadm_device.dma_debug_buff_size, hadm_device.dma_debug_buff_address);
    LCL_HAL_START_DMA(LCL_DMA_PAGE_RXDIGIQ);
#endif

    DEBUG_PIN1_PULSE

    /* Configure RSM block. Will start on NBU HW trigger */
    status = XCVR_LCL_RsmInit(rsm_config_p);    
    status += XCVR_LCL_Set_TSM_FastStart(rsm_config_p->role, rsm_config_p);
    if (hadm_config_p->phaseCont != HADM_PHASE_CONT_DISABLED)
    {
        XCVR_LCL_EnaLpmClkSwitch(1);
        XCVR_LCL_EnaDividerSync(true);
    }
    assert(gXcvrLclStatusSuccess == status);
      
    LCL_HAL_ENABLE_TONE_OBS

    DEBUG_PIN1_PULSE

    /* Configure PKT RAM circular buffers to be used by RSM */
    lcl_hal_pkt_ram_config_circ_buffers(&hadm_meas_p->pkt_ram);
    
#ifdef SIMULATOR
     status +=  SIMU_LCL_RsmGo(rsm_config_p->role, rsm_config_p);
#else
    rsm_state = LCL_HAL_XCVR_GET_RSM_STATE;
    assert(rsm_state == LCL_HAL_XCVR_RSM_STATE_DELAY);
    (void)rsm_state;
#endif

    DEBUG_PIN1_CLR
    DEBUG_PIN0_CLR

    return hal_status;
}

/* Stop one subevent */
void lcl_hadm_stop_measurement(const BLE_HADM_SubeventConfig_t *config)
{
    DEBUG_PIN0_SET
    
    hadm_meas_t *hadm_meas_p = lcl_hadm_get_meas_instance(config);
    assert(hadm_meas_p != NULL);
    assert(hadm_meas_p->state != HADM_HAL_MEAS_STATE_IDLE);


    if (hadm_meas_p->state == HADM_HAL_MEAS_STATE_RUNNING)
    {
        hadm_meas_p->state = HADM_HAL_MEAS_STATE_ABORTING;
        /* Abort RSM, this will set ABORT flag, trigger EOS IRQ, wait for RSM to switch to IDLE */
        XCVR_LCL_RsmStopAbort(TRUE);
        /* Note: Various buffers will be freed as part of regular RSM EOS/Abort code path */
    }
    else
    {
        /* Just free config buffer */
        BLE_HADM_ReleaseConfigBuffer((BLE_HADM_SubeventConfig_t **)&hadm_meas_p->config_p);
    }

    if (config->typeFlags & HADM_SUBEVT_LAST)
    {
        /* End of procedure, clean context */
        hadm_procs[config->connIdx].is_proc_init_done = false;
    }

    DEBUG_PIN0_CLR
}

/* Stop a whole procedure */
void lcl_hadm_stop_procedure(uint8 connIdx)
{
    DEBUG_PIN0_SET
    
    hadm_meas_t *hadm_meas_p = hadm_device.active_meas_p;
    if (hadm_meas_p != NULL)
    {
        /* we have a subevent in HADM_HAL_MEAS_STATE_RUNNING state on the connIdx beeing shut down */
        if (hadm_meas_p->config_p->connIdx == connIdx)
        {
            hadm_meas_p->state = HADM_HAL_MEAS_STATE_ABORTING;
            /* Abort RSM, this will set ABORT flag, trigger EOS IRQ, wait for RSM to switch to IDLE */
            XCVR_LCL_RsmStopAbort(TRUE);
            /* Note: Various buffers will be freed as part of regular RSM EOS/Abort code path */
        }
    }
    else
    {
        /* Config has been created, but no subevent running yet */
        lcl_hadm_utils_free_config_buffer(connIdx);
    }

    /* End of procedure, clean context */
    hadm_procs[connIdx].is_proc_init_done = false;
    
    DEBUG_PIN0_CLR
}

/* === Implementation (static) ============================================= */

static void lcl_hadm_measurement_setup(hadm_proc_t *hadm_proc, hadm_meas_t *hadm_meas_p, const BLE_HADM_SubeventConfig_t *hadm_config)
{
#ifndef SIMULATOR
    lcl_hal_xcvr_hadm_init(hadm_meas_p, hadm_config);
#endif
}

static void lcl_hadm_measurement_teardown(const BLE_HADM_SubeventConfig_t *hadm_config)
{
    lcl_hal_xcvr_hadm_deinit(hadm_config);
}

static hadm_meas_t *lcl_hadm_alloc_meas_instance(void)
{
    hadm_meas_t *meas_p = NULL;
    for (int i=0; i<HADM_MAX_NB_SIMULT_SUBEVENTS ; i++)
    {
        if (hadm_meas[i].state == HADM_HAL_MEAS_STATE_IDLE)
        {
            meas_p = &hadm_meas[i];
            meas_p->config_p = NULL;
            meas_p->result_p = NULL;
            break;
        }
    }
    return meas_p;
}

static void lcl_hadm_free_meas_instance(hadm_meas_t *meas_p)
{
    meas_p->state = HADM_HAL_MEAS_STATE_IDLE;
    meas_p->config_p = NULL;
    meas_p->result_p = NULL;
}

static hadm_meas_t *lcl_hadm_get_meas_instance(const BLE_HADM_SubeventConfig_t *config_p)
{
    hadm_meas_t *meas_p = NULL;
    for (int i=0; i<HADM_MAX_NB_SIMULT_SUBEVENTS ; i++)
    {
        if (hadm_meas[i].config_p == config_p)
        {
            meas_p = &hadm_meas[i];
            break;
        }
    }
    return meas_p;
}

static void lcl_hadm_measurement_shutdown(hadm_meas_t *hadm_meas_p, bool_t abort_subevent)
{
#ifndef SIMULATOR
    /* Make sure all HW blocks are stopped */
    XCVR_LCL_RsmStopAbort(abort_subevent);
    LCL_HAL_STOP_LCL;
#ifndef RSM_DBG_IQ
    if (hadm_meas_p->debug_flags & HADM_DBG_FLG_IQ_DMA)
#endif
    {
        LCL_HAL_DISABLE_DMA;
    }
    lcl_hadm_measurement_teardown(hadm_meas_p->config_p);
#endif /* SIMULATOR */
}

/*!
 * This function is used to write subevent steps (#n_steps) configuration to PKT RAM Config circular buffer
 * This is called before subevent start (by BLE_HADM_SubeventConfigApply()) or by RSM step interrupt IRQ
 */
static BLE_HADM_STATUS_t lcl_hadm_set_steps_config(uint16 n_steps, hadm_meas_t *hadm_meas_p, bool_t update_rsm_ptr)
{
    uint32_t step_idx;
    hadm_circ_buff_desc_t *circ_buff_p = &hadm_meas_p->pkt_ram.step_config;
    BLE_HADM_Chan_Mode_PmExt_AntPerm_t *step_config_p = &hadm_meas_p->config_p->chModePmAntMap[circ_buff_p->curr_step_idx];
#ifdef HADM_CFO_COMP_PER_STEP
    hadm_proc_t *hadm_proc = &hadm_procs[hadm_meas_p->config_p->connIdx];
    bool_t compensate_cfo = (hadm_meas_p->config_p->role == HADM_ROLE_INITIATOR) && (hadm_proc->ppm != 0) && ((hadm_meas_p->debug_flags & HADM_DBG_FLG_CFO_COMP_DIS) == 0);
#endif
    int16_t cfo = 0;
    uint8_t aa_offset = 1U - hadm_meas_p->config_p->role; /* offset to Rx AA */
    uint8_t cs_sync_ant_id = 0;
    
    DEBUG_PIN1_SET

    assert(circ_buff_p->curr_step_idx < hadm_meas_p->config_p->stepsNb);
    n_steps = MIN(n_steps, hadm_meas_p->config_p->stepsNb - circ_buff_p->curr_step_idx);

    for (step_idx = 0; step_idx < n_steps; step_idx++, step_config_p++)
    {
        if ((hadm_meas_p->pkt_ram.config_write_ptr + circ_buff_p->max_step_size) > circ_buff_p->base_ptr + circ_buff_p->buff_len)
        {
            /* Need to wrap now: rewind write pointer to start and toggle page */
            hadm_meas_p->pkt_ram.config_write_ptr = circ_buff_p->base_ptr;
            circ_buff_p->curr_page ^= 1U;
        }
        
        /* Compute HPM cal factor */
        uint16_t hpm_cal_val;
#ifdef HADM_PLL_CAL_INTERPOLATION
        if (step_config_p->mode >= HADM_STEP_MODE2)
        {
            hpm_cal_val = hadm_device.cal_ch40[hadm_meas_p->config_p->rttPhy].hpm_cal_val; /* fixed HPM cal for tones */
        }
        else /* mode 0 or 1 */
        {
            hpm_cal_val = lcl_hadm_get_hpm_cal_interpolation(step_config_p->channel, hadm_device.cal_ch40[hadm_meas_p->config_p->rttPhy].hpm_cal_val); /* interpolated HPM cal for packets */
        }
#else
        hpm_cal_val = hadm_device->cal_data[hadm_meas_p->config_p->rttPhy][step_config_p->channel].hpm_cal_val;
#endif
#ifdef HADM_CFO_COMP_PER_STEP
        /* Compute CFO compensation */
        if (compensate_cfo && (step_config_p->mode != HADM_STEP_MODE0))
        {
            LCL_HAL_COMPUTE_STEP_CFO(cfo, hadm_proc->cfo_channel, step_config_p->channel, hadm_proc->ppm);

        }
#endif
        /* Compute CS_SYNC antenna ID to be used for this step */
        if (step_config_p->mode != HADM_STEP_MODE2)
        {
            cs_sync_ant_id = lcl_hadm_utils_get_CS_SYNC_antenna(hadm_meas_p);
        }
        
#ifdef UT_CORRUPT_MODE0
        /* HACK for Unit Test: force missed mode0 (unexpected channel) from time to time */
        if (((hadm_ut_config_cnt%3==2) && (hadm_meas_p->config_p->role == HADM_ROLE_INITIATOR)) ||
            ((hadm_ut_config_cnt%5==1) && (hadm_meas_p->config_p->role == HADM_ROLE_REFLECTOR)))
            step_config_p->channel = 10;
#endif

        /* Build common config header to PKT RAM circular buffer */
        LCL_HAL_BUILD_PKT_RAM_CONFIG_STEP(step_config_p, hadm_meas_p->pkt_ram.config_write_ptr, cfo, hpm_cal_val, cs_sync_ant_id, hadm_meas_p->config_p->role);

        if (step_config_p->mode != HADM_STEP_MODE2)
        {
            BLE_HADM_rttType_t rtt_type = (step_config_p->mode == HADM_STEP_MODE0) ? HADM_RTT_TYPE_CS_AA_ONLY_TIMING : hadm_meas_p->config_p->rttTypes;

            /* Build AAs and payloads to PKT RAM circular buffer */
            DEBUG_PIN1_SET
            uint32_t nb_words_written = BLE_HADM_DRBG_Generate_CS_SYNC_step(hadm_meas_p->config_p->connIdx, hadm_meas_p->config_p->subeventIdx,
                                                                            circ_buff_p->curr_step_idx,
                                                                            rtt_type, hadm_meas_p->pkt_ram.config_write_ptr);
            hadm_meas_p->pkt_ram_data_in_flight[hadm_meas_p->data_in_flight_w_idx].cs_sync_ant_id = cs_sync_ant_id;
            hadm_meas_p->pkt_ram_data_in_flight[hadm_meas_p->data_in_flight_w_idx].aa_rx = *(hadm_meas_p->pkt_ram.config_write_ptr + aa_offset);
#ifdef HADM_UT_MODE0_TIMEOUT
            if ((step_config_p->mode == HADM_STEP_MODE0) && (circ_buff_p->curr_step_idx < 2) && (hadm_meas_p->config_p->role == HADM_ROLE_REFLECTOR))
            {
                *(hadm_meas_p->pkt_ram.config_write_ptr) ^= 1;
            }
#endif
            hadm_meas_p->pkt_ram.config_write_ptr += nb_words_written;
            if (++hadm_meas_p->data_in_flight_w_idx >= HADM_HAL_PKT_RAM_IN_FLIGHT_DATA_BUFFER_SIZE)
            {
                hadm_meas_p->data_in_flight_w_idx = 0;
            }
            assert(hadm_meas_p->data_in_flight_w_idx != hadm_meas_p->data_in_flight_r_idx);
            DEBUG_PIN1_CLR
        }
        if (update_rsm_ptr)
        {
            LCL_HAL_UPDATE_PKT_RAM_CONFIG_STEP_PTR(circ_buff_p->curr_page, hadm_meas_p->pkt_ram.config_write_ptr);
        }
        circ_buff_p->curr_step_idx++;
    }
    
    DEBUG_PIN1_CLR
          
    return HADM_HAL_SUCCESS;
}

/*!
 * This function is used to perform specific actions on last mode 0 reception:
 * - lock AGC
 * - Compute ppm (CFO)
 */
static BLE_HADM_STATUS_t lcl_hadm_handle_last_mode0(hadm_meas_t *hadm_meas_p)
{
    uint32_t step_idx;
    BLE_HADM_Chan_Mode_PmExt_AntPerm_t *step_config_p = &hadm_meas_p->config_p->chModePmAntMap[0];
    hadm_info_t *hadm_info_p = &hadm_meas_p->info;
    hadm_proc_t *hadm_proc = &hadm_procs[hadm_meas_p->config_p->connIdx];
    uint8_t mode0Nb = hadm_meas_p->config_p->mode0Nb;
    bool_t proc_agc_locked = (hadm_proc->agc_idx != 0xFF);
    bool_t subevent_synced = FALSE;

    hadm_sync_info_t *sync_info_p = &hadm_meas_p->sync_info[0];
    BLE_HADM_STATUS_t status;
    
    DEBUG_PIN1_SET

    for (step_idx = 0; step_idx < mode0Nb; step_idx++, sync_info_p++)
    {        
        if (sync_info_p->valid)
        {
            subevent_synced = TRUE;
            
            /* If procedure has not locked its AGC yet (didn't get a subevent with at least one valid mode0) */
            /* look for smallest gain amongst valid mode 0's */
            if (!proc_agc_locked && (hadm_proc->agc_idx > sync_info_p->agc_idx))
            {
                hadm_info_p->sync_step_id = step_idx;
                /* Record AGC index and keep the smallest */
                hadm_proc->agc_idx = sync_info_p->agc_idx;
                /* Record the estimated CFO and channel used for CFO estimation */
                hadm_proc->cfo = sync_info_p->cfo;
                hadm_proc->cfo_channel = step_config_p[step_idx].channel;
                /* Compute ppm based on measured CFO */
                hadm_proc->ppm = (int8_t)(hadm_proc->cfo / ((int32_t)2402 /* MHz */ + hadm_proc->cfo_channel));
            }
        }
    }
    
    if (subevent_synced)
    {
        /* Freeze AGC according to the index from the retained mode 0 */
        lcl_hal_xcvr_set_rxgain(hadm_proc->agc_idx);
        
        /* fallback to one shot RSSI */
        lcl_hal_xcvr_setup_rssi_continuous(false); 

        /* Compute CFO & time grid adjustment */
        if (hadm_meas_p->config_p->role == HADM_ROLE_INITIATOR)
        {
            if ((hadm_meas_p->debug_flags & HADM_DBG_FLG_CFO_COMP_DIS) == 0)
            {
                /* Initial CFO compensation */
                XCVR_LCL_RsmCompCfo(-hadm_proc->cfo);

#ifdef HADM_CFO_COMP_PER_STEP
                /* Update CFO programmed for already-built config steps */
                /* At this point only 1 step has been prepared in addition of the mode 0 steps */
                hadm_circ_buff_desc_t *circ_buff_p = &hadm_meas_p->pkt_ram.step_result;
                int16_t cfo;
                step_idx = mode0Nb + 1U;

                LCL_HAL_COMPUTE_STEP_CFO(cfo, hadm_proc->cfo_channel, step_config_p[step_idx].channel, hadm_proc->ppm);
              
                circ_buff_p = &hadm_meas_p->pkt_ram.step_config;
                assert(circ_buff_p->curr_step_idx == step_idx);
                uint32_t *config_write_ptr = circ_buff_p->base_ptr + (mode0Nb * LCL_HAL_PKT_RAM_STEP_CONFIG_MODE0_SIZE); /* skip mode 0s */
                
                LCL_HAL_UPDATE_CFO_IN_PKT_RAM_CONFIG_STEP(config_write_ptr, cfo);
#endif
            }
            
            /* Program time-grid adjustment (Initiator only) */
            lcl_hal_xcvr_program_time_adjustement((int32_t)hadm_proc->ppm);
        }
        else
        {   /* Reflector Only */
            /*
             * In some PKT-Tone cases, cfo_est may be activated (depends on received AA) for tones, affecting badly the phase.
             * To workaround this effect, we force cfo_est to zero by enabling ovveride.
             * Affected register XCVR_RX_DIG->CTRL1 is part of XCVR backup/restore.
             */
            LCL_HAL_SETUP_PKT_TONE_RX();
        }
        status = HADM_HAL_SUCCESS;
    }
    else
    {
        status = HADM_HAL_ABORTED_SYNC;
    }
    DEBUG_PIN1_CLR

    return status;
}

/*!
 * This function is used to read available subevent steps results from PKT RAM Results circular buffer
 * and format results directly in HCI format into hadm_result_buff_desc buffer.
 * This is called on RSM step interrupt IRQ or RSM EOS
 * RSM overflow is checked
 * if n_steps_required==0, all available results will be read
 */
static BLE_HADM_STATUS_t lcl_hadm_get_step_results(uint16 n_steps_required, hadm_meas_t *hadm_meas_p)
{
    hadm_circ_buff_desc_t *circ_buff_p = &hadm_meas_p->pkt_ram.step_result;
    BLE_HADM_Chan_Mode_PmExt_AntPerm_t *step_config_p = &hadm_meas_p->config_p->chModePmAntMap[circ_buff_p->curr_step_idx];
    hadm_info_t *hadm_info_p = &hadm_meas_p->info;
    hadm_proc_t *hadm_proc_p = &hadm_procs[hadm_meas_p->config_p->connIdx];
    uint8_t *res_buff_in_p;
    uint8_t *res_buff_p;
    uint8_t nb_steps = 0;
    uint8_t step_mode0_no = 0;
    uint8_t rtt_pkt_no = 0;
    xcvr_lcl_rtt_data_t rtt_data;
    BLE_HADM_role_t role = hadm_meas_p->config_p->role;

    uint32_t common_stat;
    uint32_t nadm_error_rssi;
    uint32_t rtt_data_raw;
    uint32_t tpm;  /* TPM timestamp unit is 1/32MHz */
    int ap;
    int32_t iq[HADM_MAX_NB_ANTENNA_PATHS+1U];
       
    assert(circ_buff_p->curr_step_idx < hadm_meas_p->config_p->stepsNb);
    
    if (circ_buff_p->curr_step_idx > 0)
    {
        hadm_meas_p->result_p = lcl_hadm_utils_get_result_buffer();
        
        if (hadm_meas_p->result_p == NULL)
        {
            assert(FALSE);
            return HADM_HAL_MEMORY_FULL;
        }
    }
    DEBUG_PIN1_SET

    res_buff_in_p = hadm_meas_p->result_p->resultBuffer;
    res_buff_p = res_buff_in_p;
    hadm_meas_p->result_p->firstStepCollected = circ_buff_p->curr_step_idx;
    /* Copy CFO into frequencyCompensation from procedure */
    hadm_meas_p->result_p->frequencyCompensation = hadm_proc_p->ppm;

    /* Prepare for NADM metric calculations */
    BLE_HADM_rttPhyMode_t rate = hadm_meas_p->config_p->rttPhy;
    uint8_t fm_corr_target;
    uint8_t fm_corr_div;
    XCVR_LCL_GetNadmMetricCalFactors((XCVR_RSM_SQTE_RATE_T)rate, (XCVR_RSM_RTT_TYPE_T)hadm_meas_p->config_p->rttTypes, fm_corr_target, fm_corr_div);
	
    while (circ_buff_p->curr_step_idx < hadm_meas_p->config_p->stepsNb)
    {
        if ((hadm_meas_p->pkt_ram.result_read_ptr + circ_buff_p->max_step_size) > circ_buff_p->base_ptr + circ_buff_p->buff_len)
        {
            /* Need to wrap now: rewind write pointer to start and toggle page */
            hadm_meas_p->pkt_ram.result_read_ptr = circ_buff_p->base_ptr;
            circ_buff_p->curr_page ^= 1U;
        }
        
        /* Decode common status */
        common_stat = *hadm_meas_p->pkt_ram.result_read_ptr++;
        assert(circ_buff_p->curr_step_idx == (common_stat & COM_RES_HDR_STEP_ID_STEP_ID_MASK)); /* HW and SW step_id should be aligned! */
        
        if ((common_stat & 0x30000000U) != 0)
        {
            hadm_info_p->num_time_adj++; /* record time_adj for debug */
        }
        
        /* Decode packet status if present */
        if (step_config_p->mode != HADM_STEP_MODE2)
        {
            nadm_error_rssi = *hadm_meas_p->pkt_ram.result_read_ptr++;
            rtt_data_raw = *hadm_meas_p->pkt_ram.result_read_ptr;
            hadm_meas_p->pkt_ram.result_read_ptr += 2U; /* skip cfo_est */
            tpm = *hadm_meas_p->pkt_ram.result_read_ptr++;
#ifdef RTT_DEBUG
            rtt_tpm_dbg_buffer[circ_buff_p->curr_step_idx] = tpm;
            rtt_common_stat_dbg_buffer[circ_buff_p->curr_step_idx] = common_stat & 0x30000000U;
#endif
        }
        
         /* Decode tone status if present */
        if (step_config_p->mode >= HADM_STEP_MODE2)
        {
            for (ap = 0; ap <= hadm_meas_p->n_ap; ap++)
            {
                iq[ap] = *hadm_meas_p->pkt_ram.result_read_ptr++;
            }
        }       
        
        /* Store Step_Data "header" */
        *res_buff_p++ = step_config_p->mode; /* Step_Mode */
        *res_buff_p++ = step_config_p->channel; /* Step_Channel */
        
        /* Store Step_Data */
        switch (step_config_p->mode)
        {
            case HADM_STEP_MODE0:
            {
                static bool_t synch_done = FALSE;
                bool vld = hadm_meas_p->sync_info[step_mode0_no].valid;
                 /* (Hz*100) / MHz  => 0.01 ppm unit */
                int32_t ppm = (hadm_meas_p->sync_info[step_mode0_no].cfo * 100) / (int32_t)HADM_CHAN_NUM_TO_MHZ(step_config_p->channel);
                *res_buff_p++ = BLE_HADM_STEP0_REPORT_SIZE(role); /* Step_Data_Length */
                *res_buff_p++ = HADM_SET_RTT_AA_QUALITY(vld, 1); /* Packet_AA_Quality */
                *res_buff_p++ = HADM_SET_RTT_RSSI(vld, hadm_meas_p->sync_info[step_mode0_no].rssi); /* Packet RSSI */
                *res_buff_p++ = hadm_meas_p->pkt_ram_data_in_flight[hadm_meas_p->data_in_flight_r_idx].cs_sync_ant_id + 1U;  /* Packet_Antenna (1 byte) */
                if (role == HADM_ROLE_INITIATOR)
                {
                    HADM_SET_RTT_CFO(vld, ppm, res_buff_p); /* Measured_Freq_Offset (2 bytes) */
                }
                if (!synch_done && vld)
                {
                    /* Compute final value for syncDelayUs which represents time starting from RSM trigger */
                    /* syncOffsetUs = HADM_HAL_RSM_TRIGGER_DELAY + TPM timestamp - pkt_header_duration - aa_match_delay */
                    hadm_meas_p->result_p->syncDelayUs = HADM_RTT_TS_TO_US(tpm);
                    assert((HADM_HAL_RSM_TRIGGER_DELAY + hadm_meas_p->result_p->syncDelayUs) > (HADM_1ST_BIT_TO_AA_MATCH_DURATION_US(hadm_meas_p->config_p->rttPhy)));
                    hadm_meas_p->result_p->syncDelayUs = hadm_meas_p->result_p->syncDelayUs + HADM_HAL_RSM_TRIGGER_DELAY - HADM_1ST_BIT_TO_AA_MATCH_DURATION_US(hadm_meas_p->config_p->rttPhy);
                    hadm_meas_p->result_p->syncDelayUs -= (step_mode0_no * hadm_meas_p->step_duration[0]); /* cope for missed mode0s */
                    synch_done = TRUE;
                }
                step_mode0_no++;
                rtt_pkt_no++;
                break;
            }
            case HADM_STEP_MODE1:
            case HADM_STEP_MODE3:
            {
                int32_t frac_delay = 0;
                int32_t rtt_ts = 0;
                uint8_t nadm_metric = 0xFF; /* NADM not available by default */
                rtt_pkt_no++;
                XCVR_LCL_UnpackRttResult((xcvr_lcl_rtt_data_raw_t *)&rtt_data_raw, &rtt_data, (XCVR_RSM_SQTE_RATE_T)rate); /* OJE TODO: optimize...*/
                if (rtt_data.rtt_vld && rtt_data.rtt_found)
                {
                    DEBUG_PIN1_SET
                    /* Compute integer and fractional adjustment in ns */
                    frac_delay = lcl_hadm_hartt_compute_fractional_delay((uint32_t)rate, hadm_meas_p->pkt_ram_data_in_flight[hadm_meas_p->data_in_flight_r_idx].aa_rx, rtt_data.p_delta, rtt_data.int_adj);
#ifdef RTT_DEBUG
                    rtt_frac_dbg_buffer[circ_buff_p->curr_step_idx] = frac_delay;
                    rtt_p_delta_dbg_buffer[circ_buff_p->curr_step_idx] = rtt_data.p_delta;
                    rtt_int_adj_dbg_buffer[circ_buff_p->curr_step_idx] = rtt_data.int_adj;
#endif
                    if (role == HADM_ROLE_REFLECTOR)
                    {
                        frac_delay = -frac_delay; /* On reflector side, substract frac delay */
                    }
                    DEBUG_PIN1_CLR              

                    rtt_ts = HADM_RTT_TS_TO_NS(tpm) * 2U; /* in half ns */
                    rtt_ts = rtt_ts - hadm_meas_p->ts_delay_hns + frac_delay*2U; /* in half ns to keep ts_delay_hns precision */
                    if ((step_config_p->mode == HADM_STEP_MODE3) && ((step_config_p->pm_ext & 0x1) != 0))
                    {
                        /* Remove an extra T_PM + T_SW in mode 3 case if required */
                        rtt_ts -= hadm_meas_p->ts_extra_delay_hns;
                    }
                    if ((rtt_ts < -32768) || (rtt_ts > 32767)) 
                    {
#ifndef SIMULATOR
                        //assert(false);
#endif
                        //rtt_data.rtt_vld = false;
                        rtt_ts = 12345; /* OJE TODO */
                    }
                    if (hadm_meas_p->config_p->rttTypes != HADM_RTT_TYPE_CS_AA_ONLY_TIMING)
                    {
                        uint32_t nadm_fm_corr_value = ((nadm_error_rssi & COM_MODE_013_RES_BODY_NADM_ERROR_RSSI_RAW_NADM_FM_CORR_VALUE_MASK)>>COM_MODE_013_RES_BODY_NADM_ERROR_RSSI_RAW_NADM_FM_CORR_VALUE_SHIFT);
                        XCVR_LCL_CalcNadmMetric(nadm_fm_corr_value, fm_corr_target, fm_corr_div, nadm_metric);
                    }
                }
                if (step_config_p->mode == HADM_STEP_MODE1)
                {
                    *res_buff_p++ = BLE_HADM_STEP1_REPORT_SIZE; /* Step_Data_Length */
                }
                else
                {
                    *res_buff_p++ = BLE_HADM_STEP3_REPORT_SIZE(hadm_meas_p->n_ap); /* Step_Data_Length */
                }
                *res_buff_p++ = HADM_SET_RTT_AA_QUALITY(rtt_data.rtt_vld, rtt_data.rtt_found); /* Packet_AA_Quality (1 byte) */
                *res_buff_p++ = nadm_metric; /* Packet_NADM */
                *res_buff_p++ = HADM_SET_RTT_RSSI(rtt_data.rtt_vld, (uint8_t)(nadm_error_rssi & COM_MODE_013_RES_BODY_NADM_ERROR_RSSI_RSSI_NB_MASK)); /* Packet_RSSI (1 byte) */
                HADM_SET_RTT_TS_DIFF(rtt_data.rtt_vld, rtt_ts, res_buff_p); /* ToX-ToX time diff (2 bytes) */
                *res_buff_p++ = hadm_meas_p->pkt_ram_data_in_flight[hadm_meas_p->data_in_flight_r_idx].cs_sync_ant_id + 1U;  /* Packet_Antenna (1 byte) */

                if (step_config_p->mode == HADM_STEP_MODE1)
                {
                    break;
                }
                /* else mode 3, do not break: intentional drop trough */
            }
            case HADM_STEP_MODE2:
            {
                /* Keep pm extension bit corresponding to peer device role */
                uint8_t t_pm_ext = (step_config_p->pm_ext >> role) & 0x1U;

                /* Compute RPL once per event.*/
                if (hadm_meas_p->result_p->referencePwrLevel == HADM_INVALID_REFERENCE_POWER_LEVEL)
                {
                    /* The computation is not optimal in term of accuracy as the only RSSI available whatever the main mode is the one from mode0 step.
                     * Ideally, we would need an RSSI for the captured IQ data, but we do not have that from the HW.
                     * We use averaged mode0's RSSI as the best alternative.
                     */
                    hadm_sync_info_t *sync_info_p = &hadm_meas_p->sync_info[0];
                    int32_t mode0_rssi = 0, rssi_nb = 0;
                    for (int i = 0; i < hadm_meas_p->config_p->mode0Nb; i++, sync_info_p++)
                    {
                        if (sync_info_p->valid)
                        {
                            mode0_rssi += sync_info_p->rssi;
                            rssi_nb++;
                        }
                    }
                    assert(rssi_nb != 0);
                    mode0_rssi /= rssi_nb;
                    hadm_meas_p->result_p->referencePwrLevel = lcl_hadm_utils_compute_rpl(iq[0], (int8_t)mode0_rssi);
                }
                if (step_config_p->mode == HADM_STEP_MODE2)
                {
                    *res_buff_p++ = BLE_HADM_STEP2_REPORT_SIZE(hadm_meas_p->n_ap); /* Step_Data_Length */
                }
                *res_buff_p++ = step_config_p->ant_perm; /* Antenna_Permutation_Index */
                /* Store PCT[ap], 3 bytes each 22 significant bits  +  Tone Quality Indicator [ap] (1 byte each) */
                for(ap = 0; ap < hadm_meas_p->n_ap; ap++)
                {
                    HADM_SET_RTP_PCT(iq[ap], res_buff_p);
                    HADM_SET_RTP_TONE_QUALITY(iq[ap], res_buff_p, false, 0);
                }
                /* Determine if N_AP+1 PCT has beein received or not */
                if ((step_config_p->mode == HADM_STEP_MODE2) || (role == HADM_ROLE_REFLECTOR) || ((step_config_p->pm_ext & 0x1) != 0))
                {
                    HADM_SET_RTP_PCT(iq[ap], res_buff_p);
                    HADM_SET_RTP_TONE_QUALITY(iq[ap], res_buff_p, true, t_pm_ext);
                }
                else
                {
                    HADM_SET_RTP_PCT(0, res_buff_p);
                    HADM_SET_RTP_TONE_QUALITY(0x3, res_buff_p, true, 0);
                }
                break;
            }
            default:
                assert(FALSE);
                break;
        }
        
        if (step_config_p->mode != HADM_STEP_MODE2)
        {
            if (++hadm_meas_p->data_in_flight_r_idx >= HADM_HAL_PKT_RAM_IN_FLIGHT_DATA_BUFFER_SIZE)
            {
                hadm_meas_p->data_in_flight_r_idx = 0;
            }
        }
        circ_buff_p->curr_step_idx++;
        step_config_p++;
        nb_steps++;
        
        if (nb_steps == n_steps_required)
        {
            break;
        }
    }
    
    hadm_meas_p->result_p->nbStepsCollected = nb_steps;
    assert((res_buff_p - res_buff_in_p) <= hadm_meas_p->result_p->resultBufferSize); /* check we did not write out of bounds */
    
    LCL_HAL_UPDATE_PKT_RAM_RESULT_STEP_PTR(circ_buff_p->curr_page, hadm_meas_p->pkt_ram.result_read_ptr);
    
    DEBUG_PIN1_CLR

    return HADM_HAL_SUCCESS;
}

/* RSM_INT_IRQ Handler */
void RSM_INT_IRQHandler(void)
{
    BLE_HADM_event_type_t type = HADM_EVENT_INVALID;
    uint32_t irq_status = LCL_HAL_RSM_GET_IRQ_STATUS_FLAGS;
    BLE_HADM_STATUS_t hal_status = HADM_HAL_SUCCESS;
    hadm_meas_t *hadm_meas_p = hadm_device.active_meas_p;
    
    DEBUG_PIN0_SET

    assert(hadm_meas_p != NULL);
    assert(hadm_meas_p->config_p != NULL);
    assert(hadm_meas_p->result_p != NULL);
    assert((hadm_meas_p->state == HADM_HAL_MEAS_STATE_RUNNING) || (hadm_meas_p->state == HADM_HAL_MEAS_STATE_ABORTING));

    if (hadm_meas_p->state == HADM_HAL_MEAS_STATE_ABORTING)
    {
        /* Aborted by user */
        type = HADM_EVENT_EOS;
        hal_status = HADM_HAL_STOPPED;
    }
    else if (irq_status & LCL_HAL_XCVR_RSM_IRQ_ABORT)
    {
        /* RSM has aborted */
        type = HADM_EVENT_EOS;
        uint32_t abort_reason = XCVR_MISC->RSM_CSR & (XCVR_MISC_RSM_CSR_RSM_PLL_ABORT_MASK | XCVR_MISC_RSM_CSR_RSM_UNDR_ERR_MASK | XCVR_MISC_RSM_CSR_RSM_OVF_ERR_MASK | XCVR_MISC_RSM_CSR_RSM_TIMEOUT0_ABORT_MASK);
        
        if (abort_reason & XCVR_MISC_RSM_CSR_RSM_TIMEOUT0_ABORT_MASK)
        {
            hal_status = HADM_HAL_ABORTED_SYNC;
        }
        else /* should not occur */
        {
            assert(FALSE);
            hal_status = HADM_HAL_ABORTED;
        }
    }
    else if (irq_status & LCL_HAL_XCVR_RSM_IRQ_EOS)
    {
        /* End Of Sequence */
        type = HADM_EVENT_EOS;

        /* Pull remaining steps results */
        if (hadm_meas_p->pkt_ram.step_result.curr_step_idx < hadm_meas_p->config_p->stepsNb)
        {
            hal_status = lcl_hadm_get_step_results(0, hadm_meas_p);
            assert(hadm_meas_p->data_in_flight_w_idx == hadm_meas_p->data_in_flight_r_idx);
        }
    }
    else
    {
        if (irq_status & LCL_HAL_XCVR_RSM_IRQ_FM)
        {
            /* Mode 0 interrupt */
            hadm_circ_buff_desc_t *circ_buff_p = &hadm_meas_p->pkt_ram.step_result;
            uint32_t *rsm_result_ptr;
            bool_t aa_det;
            bool_t mode0_index = circ_buff_p->curr_step_idx;
            
            do
            {
                rsm_result_ptr = circ_buff_p->base_ptr + (mode0_index * LCL_HAL_PKT_RAM_STEP_RESULT_MODE01_SIZE);
                aa_det = lcl_hal_xcvr_decode_mode0_step(hadm_meas_p->sync_info, rsm_result_ptr, (uint8_t)hadm_meas_p->config_p->rttPhy);
                mode0_index++;
            } while ((hadm_meas_p->config_p->role == HADM_ROLE_REFLECTOR) && (aa_det == FALSE) && (circ_buff_p->curr_step_idx == 0)); /* first mode 0 was missed, go to next */
            circ_buff_p->curr_step_idx = mode0_index;

            if (circ_buff_p->curr_step_idx == hadm_meas_p->config_p->mode0Nb)
            {
                /* Last mode 0 has been received */
                hal_status = lcl_hadm_handle_last_mode0(hadm_meas_p);

                if (hal_status != HADM_HAL_SUCCESS)
                {
                    type = HADM_EVENT_EOS;
                    /* Abort Subevent */
                    lcl_hadm_measurement_shutdown(hadm_meas_p, TRUE);
                }
                else
                {
                    type = HADM_EVENT_SYNC_DONE;
                    if (hadm_meas_p->pkt_ram.step_config.curr_step_idx < hadm_meas_p->config_p->stepsNb)
                    {
                        /* Push more config steps if needed */
                        lcl_hadm_set_steps_config(hadm_meas_p->pkt_ram.nb_steps_before_irq + HADM_HAL_PKT_RAM_NB_STEPS_MARGIN, hadm_meas_p, TRUE);
                    }
                }
                circ_buff_p->curr_step_idx = 0; /* reset current read index to process results in next step interrupt */
            }
        }
        else if (irq_status & LCL_HAL_XCVR_RSM_IRQ_STEP)
        {
            /* Step interrupt */
            if (hadm_meas_p->pkt_ram.step_config.curr_step_idx < hadm_meas_p->config_p->stepsNb)
            {
                /* Push more config steps if needed */
                lcl_hadm_set_steps_config(hadm_meas_p->pkt_ram.nb_steps_before_irq, hadm_meas_p, TRUE);
            }
            
            /* Read available step results */
            hal_status = lcl_hadm_get_step_results(hadm_meas_p->pkt_ram.nb_steps_before_irq, hadm_meas_p);
            if (hal_status == HADM_HAL_SUCCESS)
            {
                type = HADM_EVENT_STEP_INT;
            }
            else
            {
                assert(false);
                type = HADM_EVENT_EOS;
            }
            hadm_meas_p->pkt_ram.nb_irq_steps_handled++;
        }
    }

    if (type == HADM_EVENT_EOS)
    {
        /* Stop or abort Subevent */
        lcl_hadm_measurement_shutdown(hadm_meas_p, hal_status != HADM_HAL_SUCCESS);
        
         /* Perform post-capture remaining actions:
         *   - Populate hadm_info fields related to mode 0
         *   - Populate debug flags
         */
        hadm_proc_t *hadm_proc_p = &hadm_procs[hadm_meas_p->config_p->connIdx];
        hadm_info_t *hadm_info_p = &hadm_meas_p->info;
        if (hadm_info_p->sync_step_id < HADM_MAX_NB_STEPS_MODE0) 
        {
            hadm_sync_info_t *sync_info_p = &hadm_meas_p->sync_info[hadm_info_p->sync_step_id];
            hadm_info_p->sync_rssi = sync_info_p->rssi;
            hadm_info_p->sync_cfo = hadm_proc_p->cfo;
            hadm_info_p->sync_rxgain = hadm_proc_p->agc_idx;
        }

        if (hadm_proc_p->agc_idx > LCL_HAL_XCVR_AGC_INDEX_MAX)
            hadm_info_p->flags |= FLAGS_HADM_AGC_NOT_FROZEN;

        if (hadm_info_p->sync_rssi < -93)
            hadm_info_p->flags |= FLAGS_HADM_NO_SIGNAL;
        
        if (hal_status == HADM_HAL_ABORTED_SYNC)
            hadm_info_p->flags |= FLAGS_HADM_SYNC_ERROR;
        else if (hal_status == HADM_HAL_ABORTED)
            hadm_info_p->flags |= FLAGS_HADM_ABORT;
    }
    
    if (((hadm_meas_p->debug_flags & HADM_DBG_FLG_DBG_INFO) != 0) &&
        (hadm_meas_p->pkt_ram.step_result.curr_step_idx == hadm_meas_p->config_p->stepsNb))
    {
        /* On KW47, just report hadm_info_t for debug. Done in the last reported event. */
        hadm_meas_p->result_p->debugBuffer = (uint8_t *)&hadm_meas_p->info;
        hadm_meas_p->result_p->debugBufferSize = sizeof(hadm_info_t);
    }

    /* CLear IRQ status flags */
    LCL_HAL_RSM_SET_IRQ_STATUS_FLAGS(irq_status);
    
    /* Notify LL */
    DEBUG_PIN1_SET
    if (HADM_EVENT_INVALID != type)
    {
        BLE_HADM_NotifyLL(hadm_meas_p->result_p, type, hal_status);
    }
    DEBUG_PIN1_CLR

    if (type == HADM_EVENT_EOS)
    {
        if (hadm_meas_p->config_p->typeFlags & HADM_SUBEVT_LAST)
        {
            /* End of procedure, clean context */
            hadm_procs[hadm_meas_p->config_p->connIdx].is_proc_init_done = false;
        }
        lcl_hadm_free_meas_instance(hadm_meas_p);
        hadm_device.active_meas_p = NULL;
    }

    DEBUG_PIN0_CLR
}

/* EOF */