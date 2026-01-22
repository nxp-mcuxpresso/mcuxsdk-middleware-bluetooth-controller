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
#include "nxp_xcvr_trim.h"
#include "lcl_hadm_measurement.h"
#include "lcl_xcvr_hal.h"
#include "lcl_hadm_utils.h"
#include "lcl_hadm_hartt.h"
#include "nxp2p4_xcvr.h"
#ifdef SIMULATOR
#include "lcl_xcvr_simu.h"
#endif
#include "board.h"
#include "nxp_xcvr_ext_ctrl.h"

#if !defined(CPU_KW47B42Z83AFTA_cm33_core1) && !defined(CPU_KW47B42ZB7AFTA_cm33_core1) && !defined(CPU_MCXW727CMFTA_cm33_core1) \
    && !defined(CPU_KW43B43ZC7MFPA_NBU) && !defined(CPU_KW43B43ZC7MFTA_NBU)
#error this code is supposed to be compiled on KW47/KW43 core1
#endif

/* === Macros ============================================================== */
#define T_SLOT_US  (625U)

/* The following timings have been measured in debug mode with IAR 9.30.1 */
/* TODO OJE: Exact HAL timings need to be measured on KW47 */
/*! Time needed to execute BLE_HADM_Calibrate()
 * in order to perform the radio calibrations (HPM and DCOC)
 */
#define HADM_HAL_HPM_CAL_US  (180U)
#define HADM_HAL_DCOC_CAL_US (410U)
#define HADM_HAL_RADIO_CAL_US (HADM_HAL_DCOC_CAL_US + HADM_HAL_HPM_CAL_US)

/*! Time needed by the HAL to execute BLE_HADM_EventConfigApply()
 *  Does not need to be exact, should cover longest execution path
 */
#define HADM_HAL_PREPARE_US (50U)

/* In some cases (low power), the SW INT is triggered with up to 9us later than HW trigger */
#define HADM_HAL_RSM_TRIGGER_DELAY_MARGIN (10U)

/* 
 * Time spent in the critical path (impacting radio) to set CS context (PLL, RSM, ...).
 * Does not need to be exact, should cover RSM SW initialization, includes:
 * - time from ISR start to 1st cycle counter capture (10us)
 * - RSM init section (protected by HADM_HAL_RSM_INIT_BUDGET) = between 2 cycle counter captures
 * - Margin to absorb slight ISR drift
 * Characterisation should be done on initiator as a largest value is needed.
 */
/* For BLE/CS transitions */
#define HADM_HAL_RSM_INIT_BUDGET (140U)
 /* Optimized version corresponds to CS/CS transition (no register restore/save for BLE config) */
#define HADM_HAL_RSM_INIT_BUDGET_OPTIM (30U)
/* Additional preparation time for BT=2.0 */
#define HADM_HAL_RSM_INIT_BUDGET_DELTA_BT2 (30U)

/* 2us between cdt_expiry and RSM FSM start */
#define HADM_HAL_RSM_TRIGGER_OFFSET (2U)

/* Contengency time to make sure mode 0 AA is caught */
#define HADM_MODE0_TIMEOUT_MARGIN_US (5U)

/* === Types =============================================================== */

/* === Globals ============================================================= */

#if defined(DEBUG) || (defined(gValidationBuildOptions) && (gValidationBuildOptions == 1))
#define HADM_TRACK_RSM_INIT
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
    .RTT_Capability             = 0x2D, /* AA only and random sequence for 1Mbps and 2Mbps */
    .RTT_Coarse_N               = 10, /* Number of RTT steps to satisfy the precision requirement. */ 
    .RTT_Sounding_N             = 0, /* not supported */
    .RTT_Random_Sequence_N      = 10, /* Number of RTT steps to satisfy the precision requirement. */
    .RTT_2M_Coarse_N            = 7, /* Number of RTT steps to satisfy the precision requirement. */
    .RTT_2M_Sounding_N          = 0, /* not supported */
    .RTT_2M_Random_Sequence_N   = 7, /* Number of RTT steps to satisfy the precision requirement. */
    .NADM_Sounding_Capability   = 0, /* NADM not supported */
    .NADM_Random_Sequence_Capability = 1, /* NADM supported */
    .PHYSupported               = 1<<1 | 1<<2 , /* 2Mbps PHY supported (bit #1) & 2Mbps 2BT PHY supported (bit #2) */
    .T_SW_TimeSupported         = 2, /* 2us: OJE TODO confirm OK for ramp-up/down */
    .FAErequired                = 0, /* no FAE */
    .InlinePhaseReturn          = 1U,
    /* note: mandatory timings are not included in capabilities */
    .T_IP1_TimesSupported       = 0x0048, /* T_IP1=80 or 40us */
    .T_IP2_TimesSupported       = 0x0048, /* T_IP2=80 or 40us */
    .T_FCS_TimesSupported       = 0x0050, /* T_FCS=80 or 50us */
    .T_PM_TimesSupported        = 0x0003, /* T_PM=20us or 10us */
    .TX_SNR                     = 0x0F, /* 18dB, 21 dB, 24 dB and 27dB supported */
    .T_IP2_IPT_TimesSupported   = 0x0040, /* T_IP2_IPT=80us (dummy) */
    .T_SW_IPT_TimeSupported     = 4, /* 4us (dummy)*/
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

static uint8_t fem_active = 0U;               /* 0U / 1U for FEM inactive (default) / active (by application) */
static xcvr_pa_fem_config_t pa_fem_config =   /* the default FEM config */
{
    XCVR_ANTX_DUAL_MODE, 0U, 1U, 0U, 0U,
    XCVR_FAD_TSM_GPIO, XCVR_FAD_TSM_GPIO, XCVR_FAD_TSM_GPIO, XCVR_FAD_TSM_GPIO,
    0U, 0U, 0U, 0U, XCVR_FAD_ACTIVE_HIGH, XCVR_FAD_ACTIVE_HIGH
};

/* === Externals =========================================================== */

/* === Prototypes ========================================================== */

static void lcl_hadm_consume_drbg(hadm_meas_t *hadm_meas_p);
static void lcl_hadm_measurement_shutdown(hadm_meas_t *hadm_meas_p, bool_t abort_subevent);
static void lcl_hadm_measurement_cleanup(hadm_meas_t *hadm_meas_p);
static void lcl_hadm_measurement_setup(hadm_proc_t *hadm_proc, hadm_meas_t *hadm_meas_p, const BLE_HADM_SubeventConfig_t *hadm_config);
static BLE_HADM_STATUS_t lcl_hadm_get_step_results(uint16 n_steps_required, hadm_meas_t *hadm_meas_p);
static BLE_HADM_STATUS_t lcl_hadm_set_steps_config(uint16 n_steps, hadm_meas_t *hadm_meas_p, bool_t update_rsm_ptr);
static hadm_meas_t *lcl_hadm_alloc_meas_instance(void);
static void lcl_hadm_free_meas_instance(hadm_meas_t *meas_p);
static hadm_meas_t *lcl_hadm_get_meas_instance(const BLE_HADM_SubeventConfig_t *config_p);

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
    hadm_proc->agc_idx2 = 0xFF;
    hadm_proc->is_proc_init_done = true;

    return HADM_HAL_SUCCESS;
}

BLE_HADM_STATUS_t lcl_hadm_init(void)
{
    uint32_t i;
    BLE_HADM_STATUS_t hal_status = HADM_HAL_SUCCESS;

    lcl_hadm_utils_init_buffers();
    lcl_hadm_enable_lcl_interrupts();
    
    DEBUG_PIN0_CLR
    DEBUG_PIN1_CLR

    hadm_hal_properties.prepareMaxUs = HADM_HAL_PREPARE_US;

    /*
     * FAST_RX2TX_START_FC and FAST_TX2RX_START_FC are actually TSM indexes (us) where TSM rewinds to start an FCS phase.
     * As a consequence, they represent the elapsed time between RSM start and the beginning of FCS phase for initiator and reflector respectively.
     * Note: FCS phase in the RSM does not include ramp down which occurs at the very end of a step in the TSM sequence.
     * Typical value is 25us (for both roles).
     */
    hadm_hal_properties.txWarmupUs = 21U + 2U; /* 21us RSM WU state duration + 2us digital latency for 1st bit to reach the air */
    hadm_hal_properties.txWarmupUs += (uint16_t)((xcvr_lcl_tsm_generic_config.WU_LATENCY & XCVR_TSM_WU_LATENCY_TX_DATAPATH_LATENCY_MASK) >> XCVR_TSM_WU_LATENCY_TX_DATAPATH_LATENCY_SHIFT);
    hadm_hal_properties.txWarmupUs += HADM_HAL_RSM_TRIGGER_OFFSET;

    hadm_hal_properties.rxWarmupUs = (uint16_t)((xcvr_lcl_tsm_generic_config.FAST_CTRL3 & XCVR_TSM_FAST_CTRL3_FAST_TX2RX_START_FC_MASK) >> XCVR_TSM_FAST_CTRL3_FAST_TX2RX_START_FC_SHIFT) + 1U; /* +1 as this is a count from 0 */
    hadm_hal_properties.rxWarmupUs += HADM_HAL_RSM_TRIGGER_OFFSET;

    for (i = 0; i < HADM_MAX_NB_SIMULT_SUBEVENTS; i++)
    {
        /* Static initialization of PKT RAM buffers */
        hadm_meas[i].pkt_ram.step_config.ram_type = 0;
        hadm_meas[i].pkt_ram.step_config.buff_len = HADM_HAL_PKT_RAM_CONFIG_CIRC_BUFF_SIZE;
        hadm_meas[i].pkt_ram.step_config.base_ptr = (uint32_t *)(void *)TX_PACKET_RAM + (i * HADM_HAL_PKT_RAM_CONFIG_CIRC_BUFF_SIZE);
        assert((hadm_meas[i].pkt_ram.step_config.base_ptr + hadm_meas[i].pkt_ram.step_config.buff_len) <= (((uint32_t *)(void *)TX_PACKET_RAM) + TX_PACKET_RAM_PACKET_RAM_COUNT));
        hadm_meas[i].pkt_ram.step_result.ram_type = 1;
        hadm_meas[i].pkt_ram.step_result.buff_len = HADM_HAL_PKT_RAM_RESULT_CIRC_BUFF_SIZE;
        hadm_meas[i].pkt_ram.step_result.base_ptr = (uint32_t *)(void *)RX_PACKET_RAM + (i * HADM_HAL_PKT_RAM_RESULT_CIRC_BUFF_SIZE);
        assert((hadm_meas[i].pkt_ram.step_result.base_ptr + hadm_meas[i].pkt_ram.step_result.buff_len) <= (((uint32_t *)(void *)RX_PACKET_RAM) + RX_PACKET_RAM_PACKET_RAM_COUNT));

        lcl_hadm_free_meas_instance(&hadm_meas[i]);
        
        /* Static initialization of RSM XCVR config (will be updated by ConfigApply() API) */
        hadm_meas[i].rsm_config.sniffer_mode_en = false;
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

    hadm_device.rccal_manual_override_needed = FALSE;
    hadm_device.rtt_static_comp.rttRCcal = HADM_RCCAL_CENTER;
    hadm_device.rtt_static_comp.rttCbpfAtt[HADM_RTT_PHY_1MBPS] = HADM_CBPF_ATTEN_CENTER_1MBPS;
    hadm_device.rtt_static_comp.rttCbpfAtt[HADM_RTT_PHY_2MBPS] = HADM_CBPF_ATTEN_CENTER_2MBPS;

    /* Set default PA ramping duration and antenna switching mode. Can be overwritten via vendor HCI command. */
    hadm_device.paRampingTime = XCVR_RSM_PA_RAMP_1_USEC;
    hadm_device.paRampingAntSwitchEnabled = false;

    /* Perform initial calibration */
    hadm_device.is_rsm_cal_done = false;

#ifndef SIMULATOR
    /* Calibration sequence for 1Mbps, 2Mbps BT0.5 and BT2.0 */

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
    /* Backup modulation related registers and program BT=2 modulation */
    if (hal_status == HADM_HAL_SUCCESS)
    {
        lcl_enable_BT2p0_modulation();
    }
    if (hal_status == HADM_HAL_SUCCESS)
    {
        hal_status = lcl_hadm_calibrate_dcoc(HADM_RTT_PHY_2MBPS_2BT);
    }
    if (hal_status == HADM_HAL_SUCCESS)
    {
        hal_status = lcl_hadm_calibrate_pll(HADM_RTT_PHY_2MBPS_2BT);
    }
    /* Restore prior modulation programming */
    if (hal_status == HADM_HAL_SUCCESS)
    {
        lcl_restore_prior_modulation();
    }

    /* Read CBPF filter data from IFR - needed to compute internal RTT delay */
    {
        rf_ifr_rtt_trim_t rtt_trim_values;

        if (XCVR_TRIM_ReadRttIfr(&rtt_trim_values))
        {
            if (rtt_trim_values.rf_rtt_tg_trim_rccal != RTT_TRIM_TG_RCCAL_WIDTH_MASK)
            {
                hadm_device.rccal_manual_override_needed = TRUE;
                hadm_device.rtt_static_comp.rttRCcal = rtt_trim_values.rf_rtt_tg_trim_rccal;
                hadm_device.rtt_static_comp.rttCbpfAtt[HADM_RTT_PHY_1MBPS] = rtt_trim_values.rf_rtt_tg_attenuation_1mbps;
            }
            if (rtt_trim_values.rf_rtt_tg_attenuation_2mbps != RTT_TRIM_TG_ATTEN_WIDTH_MASK)
            {
                hadm_device.rtt_static_comp.rttCbpfAtt[HADM_RTT_PHY_2MBPS] = rtt_trim_values.rf_rtt_tg_attenuation_2mbps;
            }
        }
        else
        {
            /* Default values will be used */
        }
    }

    if (hal_status == HADM_HAL_SUCCESS)
#endif // SIMULATOR
    {
        hadm_device.is_rsm_cal_done = true;
    }

    /* Initialize RTT compensation for device contributors */
    lcl_hadm_utils_calc_rtt_static_delay(&hadm_device);

    /* Initialize temperature compensation (assume 25 degrees C in case the host does not inform NBU) */
    lcl_hadm_handle_temperature_change(HADM_TEMPERATURE_CENTER);

    /* Initialize default value for phase rotation offset */
    lcl_hadm_init_phase_offset();

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
    OSA_DisableIRQGlobal();
    hadm_device.current_temperature = (int16_t)temperature;
    lcl_hadm_utils_calc_rtt_temperature_delay(temperature, &hadm_device);
    OSA_EnableIRQGlobal();
}

BLE_HADM_STATUS_t lcl_hadm_set_antenna_type(uint8 *antBoardTable)
{
    BLE_HADM_STATUS_t hal_status = HADM_HAL_SUCCESS;
    
    for (uint8_t i = 0; i < HADM_MAX_NB_ANTENNAS; i++)
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

BLE_HADM_STATUS_t lcl_hadm_set_fem_config(uint8 *fem_config_ptr, uint8_t config_len)
{
    /* check for consistency of parameter number w.r.t XCVR */
    if ((fem_config_ptr != NULL) && (config_len == sizeof(xcvr_pa_fem_config_t)))
    {
        pa_fem_config.op_mode                  = (XCVR_ANTX_MODE_T)fem_config_ptr[0];
        pa_fem_config.ant_sel_pins_enable      = fem_config_ptr[1];
        pa_fem_config.tx_rx_switch_pins_enable = fem_config_ptr[2];
        pa_fem_config.high_z_enable            = fem_config_ptr[3];
        pa_fem_config.use_fad_state_machine    = fem_config_ptr[4];
        pa_fem_config.ant_a_pad_control        = (XCVR_FAD_NOT_GPIO_MODE_T)fem_config_ptr[5];
        pa_fem_config.ant_b_pad_control        = (XCVR_FAD_NOT_GPIO_MODE_T)fem_config_ptr[6];
        pa_fem_config.tx_switch_pad_control    = (XCVR_FAD_NOT_GPIO_MODE_T)fem_config_ptr[7];
        pa_fem_config.rx_switch_pad_control    = (XCVR_FAD_NOT_GPIO_MODE_T)fem_config_ptr[8];
        pa_fem_config.pa_tx_wu                 = fem_config_ptr[9];
        pa_fem_config.pa_tx_wd                 = fem_config_ptr[10];
        pa_fem_config.lna_rx_wu                = fem_config_ptr[11];
        pa_fem_config.lna_rx_wd                = fem_config_ptr[12];
        pa_fem_config.tx_switch_pol_control    = (XCVR_RX_TX_POLARITY_MODE_T)fem_config_ptr[13];
        pa_fem_config.rx_switch_pol_control    = (XCVR_RX_TX_POLARITY_MODE_T)fem_config_ptr[14];

        fem_active = 1U;
        return HADM_HAL_SUCCESS;
    }
    else
    {
        return HADM_HAL_INVALID_ARGS;
    }
}

void lcl_hadm_set_dma_debug_buffer(uint16 dma_debug_buff_size, uint32 dma_debug_buff_address)
{
    hadm_device.dma_debug_buff_size = dma_debug_buff_size;
    hadm_device.dma_debug_buff_address = dma_debug_buff_address;
}

BLE_HADM_STATUS_t lcl_hadm_calibrate_dcoc(BLE_HADM_rttPhyMode_t rate)
{   
    xcvrLclStatus_t status;
    XCVR_RSM_SQTE_RATE_T xcvr_rate = (XCVR_RSM_SQTE_RATE_T)rate;
    
    DEBUG_PIN0_SET

    /* trigger calibration */
    XCVR_LCL_CalibrateDcocStart(xcvr_rate);
    /* wait for results */
    status = XCVR_LCL_CalibrateDcocCompleteFine(&hadm_device.dcoc_cal_results[rate]);

    DEBUG_PIN0_CLR

    return (status == gXcvrLclStatusSuccess ? HADM_HAL_SUCCESS : HADM_HAL_FAIL);
}

BLE_HADM_STATUS_t lcl_hadm_calibrate_pll(BLE_HADM_rttPhyMode_t rate)
{   
    xcvrLclStatus_t status = gXcvrLclStatusSuccess;
    XCVR_RSM_SQTE_RATE_T xcvr_rate = (XCVR_RSM_SQTE_RATE_T)rate;
    BLE_HADM_STATUS_t hal_status = HADM_HAL_SUCCESS;

    DEBUG_PIN0_SET

    /* Backup PLL config */
    lcl_hal_xcvr_pll_settings_backup();

#ifdef HADM_PLL_CAL_INTERPOLATION
    uint16_t chan40_ovrd;
    /* Trigger manual calibration on channel 40 */
    (void)XCVR_LCL_MakeChanNumFromHadmIndex(40 /* 2442 MHz */, &chan40_ovrd);
    DEBUG_PIN1_SET
    status = XCVR_LCL_CalibratePll((const channel_num_t *)&chan40_ovrd, (xcvr_lcl_pll_cal_data_t *)(void*)&hadm_device.cal_ch40[rate], 1U, false, xcvr_rate);
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

/* Because the time needed to program CS in the XCVR (RSM, TSM etc ...) is significant, we have to 
 * estimate its duration as precisely as possible in order to adjust the time lost between scheduler trigger
 * and actual over the air activity. Adjustement can be determined from the configuration.
 * Main contributors identified today:
 * - BT2.0 programming
 * - whether or not a BLE/CS transition will occur
 */
static uint16_t lcl_hadm_compute_rsm_trigger_delay(const BLE_HADM_SubeventConfig_t *hadm_config_p)
{
    uint16_t rsm_trigger_delay;

    if ((HADM_IS_RSM_OPTIM_INACTIVE(hadm_config_p)) ||
        (hadm_config_p->subeventIdx == 0U))
    {
        rsm_trigger_delay = HADM_HAL_RSM_INIT_BUDGET + HADM_HAL_RSM_TRIGGER_DELAY_MARGIN;
    }
    else
    {
        rsm_trigger_delay = HADM_HAL_RSM_INIT_BUDGET_OPTIM + HADM_HAL_RSM_TRIGGER_DELAY_MARGIN;
    }
    if (hadm_config_p->rttPhy == HADM_RTT_PHY_2MBPS_2BT)
    {
        rsm_trigger_delay += HADM_HAL_RSM_INIT_BUDGET_DELTA_BT2;
    }

    return rsm_trigger_delay;
}

void lcl_hadm_get_preparation_timings(const BLE_HADM_SubeventConfig_t *hadm_config_p,
                                      uint16_t *prepare_time,
                                      uint16_t *warmup_time,
                                      uint16_t *warmdown_time)
{
    uint16_t rsm_trigger_delay = lcl_hadm_compute_rsm_trigger_delay(hadm_config_p);

    /* prepare_time */
    *prepare_time = HADM_HAL_PREPARE_US;

    /* warmup_time */
    if (hadm_config_p->role == HADM_ROLE_INITIATOR)
    {
        *warmup_time = hadm_hal_properties.txWarmupUs;
    }
    else
    {
        *warmup_time = hadm_hal_properties.rxWarmupUs;
    }
    *warmup_time += rsm_trigger_delay;

    /* warmdown_time */
    *warmdown_time = 0;
}

const BLE_HADM_HalCapabilities_t *lcl_hadm_get_capabilities(void)
{
    return &hadm_hal_capabilities;
}

BLE_HADM_STATUS_t lcl_hadm_check_config(const BLE_HADM_SubeventConfig_t *hadm_config)
{   
    if((hadm_config->stepsNb > HADM_MAX_NB_STEPS) || (hadm_config->stepsNb < HADM_MIN_NB_STEPS))
    {   goto config_error; }

    if (hadm_config->chModePmAntMap[0].mode != 0U) /* First step shall be mode 0 */
    {   goto config_error; }
    
    if (hadm_config->T_PM_Time > HADM_T_PM_40)
    {   goto config_error; }
    
    if ((hadm_config->toneAntennaConfigIdx == HADM_ANT_CFG_IDX_0) && (hadm_config->T_SW_Time != HADM_T_SW_0)) /* N_AP=1 => T_SW=0 */
    {   goto config_error; }
        
    if ((hadm_config->toneAntennaConfigIdx != HADM_ANT_CFG_IDX_0) && (hadm_config->T_SW_Time < HADM_T_SW_2)) /* only T_SW >1us is supported */
    {   goto config_error; }

    if ((hadm_config->rttAntennaID == 0U) || ((hadm_config->rttAntennaID > HADM_MAX_NB_ANTENNAS) && (hadm_config->rttAntennaID < HADM_RTT_ANT_ROUND_ROBIN)))
    {   goto config_error; }

    if (hadm_config->subeventIdx >= HADM_NUM_SUBEVENTS_MAX)
    {   goto config_error; }
    
    if (hadm_config->connIdx >= HADM_MAX_NB_CONNECTIONS)
    {   goto config_error; }

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
    uint16_t mode0_timeout_usec;

    DEBUG_PIN0_SET

    if (!hadm_device.is_rsm_cal_done)
    {
        goto config_error; /* at this point all calibration must have ben performed */
    }

    hadm_device.sys_clock_freq = (uint16_t)(BOARD_GetSystemCoreClockFreq()/1000000U);

    hadm_meas_p->config_p = hadm_config; /* save config ptr */
    
    hadm_meas_p->mode0_complete = false;
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
          hadm_meas_p->iq_avg_win = 5U;
          break;
        case HADM_T_PM_20:
          hadm_meas_p->iq_avg_win = 4U;
          break;
        case HADM_T_PM_10:
          hadm_meas_p->iq_avg_win = 3U;
          break;
        default:
          hadm_meas_p->iq_avg_win = 0xFFU; /* invalid value allows error handling */
          break;
    }
    if (hadm_meas_p->iq_avg_win == 0xFFU) { goto config_error; }

    /* Capture window size in us, this will always result in 4 averaged values / AP / step */
    hadm_meas_p->iq_capture_win = (uint8_t)(1U << hadm_meas_p->iq_avg_win);
#ifndef RSM_DBG_IQ
    if ((hadm_meas_p->debug_flags & HADM_DBG_FLG_AVG_OFF) != 0U)
#endif
    {
        hadm_meas_p->iq_avg_win = 0U;
    }
#ifndef RSM_DBG_IQ
    else if (hadm_config->rttPhy != HADM_RTT_PHY_1MBPS)
    {
        /* Double IQ averaging window in order to get the same number of final (averaged) samples whatever the data rate */
        hadm_meas_p->iq_avg_win ++;
    }
    else
    { /* MISRA */ }
#endif
    HADM_COMPUTE_NUM_ANTENNA(hadm_config->role, (uint8_t)hadm_config->toneAntennaConfigIdx, hadm_meas_p->num_ant, hadm_meas_p->n_ap);
    
    /* Compute IQ buffer size */
    if ((hadm_meas_p->debug_flags & HADM_DBG_FLG_IQ_DMA) != 0U)
    {
      lcl_hadm_utils_compute_iq_buff_size(hadm_config, hadm_meas_p, (hadm_config->rttPhy == HADM_RTT_PHY_1MBPS) ? RX_SAMPLING_RATE:(RX_SAMPLING_RATE*SAMPLING_RATE_FACTOR_2MBPS));
    }
    /* Compute step mode durations */
    lcl_hadm_utils_compute_step_duration(hadm_config, hadm_meas_p->n_ap, hadm_meas_p->step_duration);

    /* RTT: Compute RTT local contributions */
    lcl_hadm_utils_calc_ts_delay(hadm_meas_p, &hadm_device);

    /* Configure rttAntennaID to be used during first mode 0 */
    /* If no recommendation from the host then apply crossed roud robin in the controller since it gives better perf */
    if (hadm_config->rttAntennaID >= HADM_RTT_ANT_ROUND_ROBIN)
    {   hadm_meas_p->rtt_antenna_id = 0U;  }   /* start round robin */
    else
    {   hadm_meas_p->rtt_antenna_id = hadm_config->rttAntennaID - 1U;  }   /* fixed antenna */
    
    /* Clear info structs */
    hadm_meas_p->info.rtt_dbg_buffer_nb = 0;
    hadm_meas_p->info.mciq_dbg_buffer_nb = 0;
    hadm_meas_p->info.sync_cfo = 0;
    hadm_meas_p->info.sync_rxgain = 0xFF;
    hadm_meas_p->info.sync_rssi = -128;
    hadm_meas_p->info.sync_step_id = 0xFF;
    hadm_meas_p->info.flags = 0;
    hadm_meas_p->info.xtal_trim = 0U; /* = (int32_t)XCVR_GetXtalTrim() avoid use XCVR_GetXtalTrim which use RFMC and requires CM33 wakeup */
    hadm_meas_p->info.temperature = (int8_t)hadm_device.current_temperature;
    hadm_meas_p->info.num_time_adj = 0;
    
    hadm_meas_p->pkt_ram.config_write_ptr = hadm_meas_p->pkt_ram.step_config.base_ptr;
    hadm_meas_p->pkt_ram.result_read_ptr = hadm_meas_p->pkt_ram.step_result.base_ptr;
    hadm_meas_p->pkt_ram.step_config.curr_page = 0;
    hadm_meas_p->pkt_ram.step_config.curr_step_idx = 0;
    hadm_meas_p->pkt_ram.step_config.max_step_size = LCL_HAL_PKT_RAM_STEP_CONFIG_MODE13_SIZE(hadm_meas_p->n_ap) + rtt_type_2_payload_size[hadm_meas_p->config_p->rttTypes] * 2U; /* taking mode1/3 into account */
    hadm_meas_p->pkt_ram.step_result.curr_page = 0;
    hadm_meas_p->pkt_ram.step_result.curr_step_idx = 0;
    hadm_meas_p->pkt_ram.step_result.max_step_size = LCL_HAL_PKT_RAM_STEP_RESULT_MODE3_SIZE(hadm_meas_p->n_ap); /* taking mode3 into account */
    if (hadm_meas_p->config_p->role == HADM_ROLE_SNIFFER)
    {
        hadm_meas_p->pkt_ram.step_result.max_step_size *= HADM_SNIFFER_DEVICE_NB;
    }
    /* Compute nb_steps_before_irq based on larget circ buffer step size (+ contingency (/2)) */
    hadm_meas_p->pkt_ram.nb_steps_before_irq = (uint8_t)((HADM_HAL_PKT_RAM_CONFIG_CIRC_BUFF_SIZE / (MAX(hadm_meas_p->pkt_ram.step_config.max_step_size, hadm_meas_p->pkt_ram.step_result.max_step_size))) / 2U);
    if (hadm_meas_p->pkt_ram.nb_steps_before_irq > HADM_HAL_PKT_RAM_MAX_NB_STEPS_BEFORE_IRQ)
    {
        hadm_meas_p->pkt_ram.nb_steps_before_irq = HADM_HAL_PKT_RAM_MAX_NB_STEPS_BEFORE_IRQ;
    }
    hadm_meas_p->pkt_ram.nb_irq_steps_handled = 0;
    hadm_meas_p->pkt_ram.nb_irq_steps_postponed = 0;
    for (i=0; i < HADM_MAX_NB_STEPS_MODE0; i++)
    {
        hadm_meas_p->sync_info[i].valid = 0U;
    }
    hadm_meas_p->data_in_flight_w_idx = 0;
    hadm_meas_p->data_in_flight_r_idx = 0;
    hadm_meas_p->rsm_trigger_delay = lcl_hadm_compute_rsm_trigger_delay(hadm_meas_p->config_p);

    /* Prepare RSM configuration in RAM */
    DEBUG_PIN1_SET 
    
    rsm_config_p->num_steps = hadm_meas_p->config_p->stepsNb;
    rsm_config_p->num_ant_path = hadm_meas_p->n_ap;
    rsm_config_p->rate = (hadm_meas_p->config_p->rttPhy == HADM_RTT_PHY_1MBPS) ? XCVR_RSM_RATE_1MBPS:XCVR_RSM_RATE_2MBPS;
    rsm_config_p->rsm_dma_dly_fm_ext = (HADM_T_FM - hadm_meas_p->iq_capture_win) >> 1; /* center capture window inside T_FM */
    rsm_config_p->rsm_dma_dur_fm_ext = hadm_meas_p->iq_capture_win;
    rsm_config_p->averaging_win = (hadm_meas_p->iq_avg_win == 0U) ? XCVR_RSM_AVG_WIN_DISABLED : (XCVR_RSM_AVG_WIN_LEN_T)(hadm_meas_p->iq_avg_win - 1U);
    /* RSM trig_delay must cover start API execution time */
    rsm_config_p->trig_delay = hadm_meas_p->rsm_trigger_delay;
    rsm_config_p->t_fc = (uint8_t)hadm_config->T_FCS_Time;
    rsm_config_p->t_ip1 = (uint8_t)hadm_config->T_IP1_Time;
    rsm_config_p->t_ip2 = (uint8_t)hadm_config->T_IP2_Time;
    rsm_config_p->t_sw = (lclTSw_t)hadm_config->T_SW_Time;
    rsm_config_p->rtt_type = (XCVR_RSM_RTT_TYPE_T)hadm_meas_p->config_p->rttTypes;
    rsm_config_p->role = (hadm_config->role == HADM_ROLE_INITIATOR) ? XCVR_RSM_TX_MODE : XCVR_RSM_RX_MODE;
    rsm_config_p->sniffer_mode_en = (hadm_config->role == HADM_ROLE_SNIFFER);
    rsm_config_p->enable_inpr = (bool)hadm_config->inlinePhaseReturn;
    rsm_config_p->hpm_cal_manual_val = hadm_device.cal_ch40[hadm_meas_p->config_p->rttPhy].hpm_cal_val;
    rsm_config_p->use_rccal_manual_override = hadm_device.rccal_manual_override_needed;
    rsm_config_p->manual_rccal_value = hadm_device.rtt_static_comp.rttRCcal;
    rsm_config_p->tx_snr_setting = (hadm_config->Tx_Snr < (uint8)XCVR_RSM_TX_SNR_DISABLED)?(XCVR_RSM_TX_SNR_T)hadm_config->Tx_Snr:XCVR_RSM_TX_SNR_DISABLED;
    rsm_config_p->pa_ramp_time = hadm_device.paRampingTime;

    if (hadm_meas_p->config_p->mode != HADM_SUBEVT_TEST_MODE_PHASE_STAB)
    {
        rsm_config_p->op_mode = XCVR_RSM_SQTE_MODE;
        rsm_config_p->t_pm0 = (uint16_t)(((uint32_t)hadm_meas_p->n_ap + 1U) * ((uint32_t)hadm_meas_p->config_p->T_PM_Time + (uint32_t)hadm_meas_p->config_p->T_SW_Time));
    }
    else
    {
        rsm_config_p->op_mode = XCVR_RSM_SQTE_STABLE_PHASE_TEST_MODE;
        rsm_config_p->t_pm0 = LCL_HAL_T_PM_MEAS; /* Set to T_PM_MEAS for stable phase test */
        hadm_meas_p->iq_buff_size_mode0 = 0;
        hadm_meas_p->iq_buff_size = 0;
        hadm_meas_p->iq_avg_win = 0;
    }
    
    /* Determine and program rx timeout (for reflector devices) = mode 0 duration in order to keep RX channel in sync with initiator */
    /* From HW spec: RSM timeout interval = mode0_timeout + T_FCS + T_RD + TX_DATA_FLUSH_DLY + 1 */
    mode0_timeout_usec = (uint16_t)((uint32_t)hadm_meas_p->step_duration[0] - ((uint32_t)hadm_config->T_FCS_Time + T_RD + ((hadm_config->rttPhy == HADM_RTT_PHY_1MBPS) ? TX_DATA_FLUSH_DLY_1MBPS:TX_DATA_FLUSH_DLY_2MBPS) + 1U));
    assert(mode0_timeout_usec > HADM_T_SY(hadm_config->rttPhy) + HADM_MODE0_TIMEOUT_MARGIN_US);
    /* If window widening is larger than the mode0 RX window minus packet duration, do not activate rx timeout */
    if ((hadm_meas_p->config_p->mode == HADM_SUBEVT_MISSION_MODE) &&
        (hadm_config->rxWindowUs < ((uint32_t)mode0_timeout_usec - HADM_T_SY(hadm_config->rttPhy) - HADM_MODE0_TIMEOUT_MARGIN_US)))
    {
        /* Program the mode0 timeout (the actual duration of RX period) */
        rsm_config_p->mode0_timeout_usec = mode0_timeout_usec;
    }
    else
    {
        rsm_config_p->mode0_timeout_usec = 0; /* means no timeout */
    }
	
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 475)
    rsm_config_p->phase_comp_sel = XCVR_RSM_PHASE_COMP_DISABLED, /* Disables the phase compensation by default */
#endif /* defned(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 475)  */
    
    status = XCVR_LCL_ValidateRsmSettings(rsm_config_p);
    if (gXcvrLclStatusSuccess != status)
    {   goto config_error; }

    DEBUG_PIN1_CLR
    DEBUG_PIN1_SET 

    /* Build first configuration steps in PKT RAM: mode 0 steps without pushing RSM pointers */
    (void)lcl_hadm_set_steps_config(hadm_config->mode0Nb, hadm_meas_p, FALSE);
    
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
    xcvr_lcl_rsm_config_t *rsm_config_p;
    uint32_t rsm_state;
#ifdef HADM_TRACK_RSM_INIT
    static uint32_t cyccnt_1;
    static int32_t elapsed_time;
#endif

    DEBUG_PIN0_SET

    /* Enable required RSM interrupts for SW FSM */
    LCL_HAL_RSM_SET_IRQ_ENABLE_MASK(LCL_HAL_RSM_XCVR_IRQ_ENABLE_MASK);

#ifdef HADM_TRACK_RSM_INIT
    DWT->CTRL |= 1;
    cyccnt_1 = DWT->CYCCNT;
#endif
    do
    {
        if (hadm_meas_p == NULL)
        {
            assert(false);
            hal_status = HADM_HAL_INVALID_ARGS;
            break;
        }
        rsm_config_p = &hadm_meas_p->rsm_config;

#ifdef UT_HADM_TRIGGER
        /* RSM should have started on NBU trigger and be in DELAY state */
        rsm_state = LCL_HAL_XCVR_GET_RSM_STATE();
        assert(rsm_state != LCL_HAL_XCVR_RSM_STATE_IDLE);
#endif

        /* At this point, the config must be valid and the procedure must have been initialized */
        if ((hadm_meas_p->state != HADM_HAL_MEAS_STATE_CONFIGURED) || (!hadm_proc->is_proc_init_done))
        {
            assert(false);
            hal_status = HADM_HAL_INVALID_ARGS;
            break;
        }
        
        /* Alloc result buffer */
        hadm_meas_p->result_p = lcl_hadm_utils_get_result_buffer();
        if (hadm_meas_p->result_p == NULL)
        {
#ifdef HAL_ENABLE_ASSERT_ON_STRESS
            assert(false);
#endif
            hal_status = HADM_HAL_MEMORY_FULL;
            break;
        }

        DEBUG_PIN1_SET

        hadm_device.active_meas_p = hadm_meas_p; /* becomes active measurement */
        hadm_meas_p->state = HADM_HAL_MEAS_STATE_RUNNING;

        hadm_meas_p->result_p->connIdx = hadm_config_p->connIdx;
        hadm_meas_p->result_p->subeventIdx = hadm_config_p->subeventIdx; /* echo subeventIdx */
        hadm_meas_p->result_p->syncDelayUs = 0;

        DEBUG_PIN1_PULSE

        if ((hadm_meas_p->config_p->mode != HADM_SUBEVT_TEST_MODE) || (hadm_meas_p->result_p->subeventIdx == 0U))
        {
            lcl_hadm_measurement_setup(hadm_proc, hadm_meas_p, hadm_config_p);
        }

        DEBUG_PIN1_PULSE

        /* Configure Antenna switching */
        lcl_hadm_utils_configure_antenna_switching(hadm_meas_p, hadm_device.paRampingAntSwitchEnabled);
        /* Configure TQI and start LCL if needed */
        if (rsm_config_p->op_mode != XCVR_RSM_SQTE_STABLE_PHASE_TEST_MODE)
        {
            if ((hadm_meas_p->debug_flags & HADM_DBG_FLG_AVG_OFF) == 0U)
            {
                lcl_hal_xcvr_program_tqi(hadm_meas_p);
            }
            LCL_HAL_START_LCL;
        }

        DEBUG_PIN1_PULSE

        /* Start DMA if needed */
#ifndef RSM_DBG_IQ
        if (((hadm_meas_p->debug_flags & HADM_DBG_FLG_IQ_DMA) != 0U) && ((hadm_meas_p->iq_buff_size > 0U) || (hadm_meas_p->iq_buff_size_mode0 > 0U)))
        {
            lcl_hal_xcvr_configure_dma_capture(LCL_START_DMA_ON_RSM_TRIGGER, 0, (uint32_t)hadm_meas_p->iq_buff_size + (uint32_t)hadm_meas_p->iq_buff_size_mode0, hadm_device.dma_debug_buff_size, hadm_device.dma_debug_buff_address);
            LCL_HAL_START_DMA(LCL_DMA_PAGE_RXDIGIQ);
        }
#else
        LCL_HAL_SET_IQ_CAPTURE_POINT(LCL_OUT_CH_FILTER_SEL);
        LCL_HAL_CLEAR_DMA_MASK_AND_AVG_WIN;
        lcl_hal_xcvr_configure_dma_capture(LCL_START_DMA_ON_TSM_RX_DIG_EN, 0, 0x2000, hadm_device.dma_debug_buff_size, hadm_device.dma_debug_buff_address);
        LCL_HAL_START_DMA(LCL_DMA_PAGE_RXDIGIQ);
#endif

        DEBUG_PIN1_PULSE

        if (HADM_IS_RSM_OPTIM_INACTIVE(hadm_config_p) ||
            (hadm_config_p->subeventIdx == 0U))
        {
            /* Configure RSM block. Will start on NBU HW trigger */
            status = XCVR_LCL_RsmInit(rsm_config_p);
            assert(gXcvrLclStatusSuccess == status);
            (void)status;

            status = XCVR_LCL_Set_TSM_FastStart(rsm_config_p->role, rsm_config_p);
            assert(gXcvrLclStatusSuccess == status);
            (void)status;

            if (fem_active == 1U)
            {
                /* FEM is active, apply to XCVR. Backup & restore of BLE's TIMING07, TIMING08 are done.
                   XCVR_ANALOG->LDO_1 & XCVR_MISC->FAD_CTRL are not changed */
                xcvrStatus_t xcvr_status = XCVR_ExternalFadPaFemInit(&pa_fem_config);
                assert(xcvr_status == gXcvrSuccess_c);
                (void)xcvr_status;
            }
        }
        else
        {
            uint32_t temp;
            /* For testmode, RSM config does not need to be reapplied, simply set number of steps */
            temp = XCVR_MISC->RSM_CTRL0 & ~XCVR_MISC_RSM_CTRL0_RSM_STEPS_MASK;
            temp |= XCVR_MISC_RSM_CTRL0_RSM_STEPS((uint32_t)rsm_config_p->num_steps);
            temp |= (rsm_config_p->role == XCVR_RSM_TX_MODE ? XCVR_MISC_RSM_CTRL0_RSM_TX_EN_MASK :
                                                            XCVR_MISC_RSM_CTRL0_RSM_RX_EN_MASK);
            XCVR_MISC->RSM_CTRL0 = temp;

        }

        /* Setup BT=2 modulation if needed */
        if (hadm_config_p->rttPhy == HADM_RTT_PHY_2MBPS_2BT)
        {
            DEBUG_PIN1_TGL
            lcl_enable_BT2p0_modulation();
            DEBUG_PIN1_TGL
        }

        assert(gXcvrLclStatusSuccess == status);
        (void)status;

        LCL_HAL_ENABLE_TONE_OBS

        DEBUG_PIN1_PULSE

        /* Configure PKT RAM circular buffers to be used by RSM */
        lcl_hal_pkt_ram_config_circ_buffers(&hadm_meas_p->pkt_ram);
        /* make sure PLL_OFFSET_CTRL is cleared before first mode 0 */
        (void)XCVR_LCL_RsmCompCfo(0);

        /* End of critical configuration section: past this point, the RSM is supposed to exit SM_STATE_DELAY */
        DEBUG_PIN0_PULSE

#ifdef HADM_TRACK_RSM_INIT
        /* compute elapsed time since function entry - wrap is handled */
        elapsed_time = (uint32_t)((int32_t)DWT->CYCCNT - (int32_t)cyccnt_1)/hadm_device.sys_clock_freq;
        assert(elapsed_time < (hadm_meas_p->rsm_trigger_delay - HADM_HAL_RSM_TRIGGER_DELAY_MARGIN));
        (void)elapsed_time;
#endif /* HADM_TRACK_RSM_INIT */

#ifdef SIMULATOR
        status +=  SIMU_LCL_RsmGo(rsm_config_p->role, rsm_config_p);
#else
        rsm_state = LCL_HAL_XCVR_GET_RSM_STATE();
#ifdef HAL_ENABLE_ASSERT_ON_STRESS
        assert(rsm_state == LCL_HAL_XCVR_RSM_STATE_DELAY);
#endif
        if (rsm_state != LCL_HAL_XCVR_RSM_STATE_DELAY)
        {
            hal_status = HADM_HAL_COLLISION;
            break;
        }
#endif

        /* Check that subevent has some main mode steps (may have zero non-mode0 steps if procedure reaches 256 steps) */
        if (hadm_meas_p->config_p->stepsNb > hadm_meas_p->config_p->mode0Nb)
        {
            /* Build first non-mode0 configuration step in PKT RAM while subevent starts */
            (void)lcl_hadm_set_steps_config(HADM_HAL_PKT_RAM_NB_STEPS_CONFIG_INITIAL, hadm_meas_p, TRUE);
        }
    } while(false);
#ifdef HADM_TRACK_RSM_INIT
    DWT->CTRL &= ~1;
#endif

    if ((hal_status != HADM_HAL_SUCCESS) && (hadm_meas_p != NULL))
    {
        /* Disable all RSM interrupts - synchronous cleanup */
        LCL_HAL_RSM_SET_IRQ_ENABLE_MASK(0);
        lcl_hadm_measurement_shutdown(hadm_meas_p, true);
        BLE_HADM_ReleaseResultsBuffer(&hadm_meas_p->result_p);
        ((BLE_HADM_SubeventConfig_t*)hadm_meas_p->config_p)->configBufferUsed = 0U;
        lcl_hadm_measurement_cleanup(hadm_meas_p);
    }
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
    assert(hadm_meas_p->config_p != NULL);
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
        /* No EOS IRQ will be triggered, simply free resources */
        ((BLE_HADM_SubeventConfig_t*)hadm_meas_p->config_p)->configBufferUsed = 0U;
        lcl_hadm_measurement_cleanup(hadm_meas_p);
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
            lcl_hadm_stop_measurement(hadm_meas_p->config_p);
            hadm_meas_p->state = HADM_HAL_MEAS_STATE_ABORTING;
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
    lcl_hal_xcvr_hadm_init(&hadm_device, hadm_meas_p, hadm_config);
#ifdef HADM_CFO_COMP_PER_STEP_VIA_FOM
    if (hadm_config->role != HADM_ROLE_REFLECTOR)
    {
        /* In this mode, an additional per-step IRQ is setup with higher priority than RSM IRQ, in order to program CFO via Fast Override Module */
        lcl_hadm_enable_interrupts_for_subevent(hadm_config->role == HADM_ROLE_INITIATOR);
        /* FOM override occurs on fom_tx/tx_en or fom_rx/rx_en which happens before TSM raises the IT, so will apply to the next step. */
        LCL_HAL_SET_PLL_OFFSET_FO_ENTRY();
    }
#endif /* HADM_CFO_COMP_PER_STEP_VIA_FOM */
#endif /* SIMULATOR */
}

static hadm_meas_t *lcl_hadm_alloc_meas_instance(void)
{
    hadm_meas_t *meas_p = NULL;
    for (int i=0; i<(int)HADM_MAX_NB_SIMULT_SUBEVENTS ; i++)
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
    for (int i=0; i<(int)HADM_MAX_NB_SIMULT_SUBEVENTS ; i++)
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
    if ((hadm_meas_p->debug_flags & HADM_DBG_FLG_IQ_DMA) != 0U)
#endif
    {
        LCL_HAL_DISABLE_DMA;
    }
    /* Do not restore XCVR state if RSM optim is active (except for the last subevent) */
    if (HADM_IS_RSM_OPTIM_INACTIVE(hadm_meas_p->config_p) ||
        ((hadm_meas_p->config_p->typeFlags & HADM_SUBEVT_LAST) != 0U))
    {
        lcl_hal_xcvr_hadm_deinit(hadm_meas_p->config_p);
    }
#ifdef HADM_CFO_COMP_PER_STEP_VIA_FOM
    if (hadm_meas_p->config_p->role != HADM_ROLE_REFLECTOR)
    {
        lcl_hadm_restore_interrupts_for_subevent();
    }
#endif /* HADM_CFO_COMP_PER_STEP_VIA_FOM */
    lcl_hadm_consume_drbg(hadm_meas_p);
#endif /* SIMULATOR */
}

/* Release measurement resources and reset device state for next measurement.
 * If this is the last subevent in a procedure, also clean procedure context.
 */
static void lcl_hadm_measurement_cleanup(hadm_meas_t *hadm_meas_p)
{
    if ((hadm_meas_p->config_p->typeFlags & HADM_SUBEVT_LAST) != 0U)
    {
        /* End of procedure, clean context */
        hadm_procs[hadm_meas_p->config_p->connIdx].is_proc_init_done = false;
    }
    lcl_hadm_free_meas_instance(hadm_meas_p);
}

/*!
 * This function is used to write subevent steps (#n_steps) configuration to PKT RAM Config circular buffer
 * This is called before subevent start (by BLE_HADM_SubeventConfigApply()) or by RSM step interrupt IRQ
 */
static BLE_HADM_STATUS_t lcl_hadm_set_steps_config(uint16 n_steps, hadm_meas_t *hadm_meas_p, bool_t update_rsm_ptr)
{
    uint32_t step_idx;
    hadm_circ_buff_desc_t *circ_buff_p = &hadm_meas_p->pkt_ram.step_config;
    BLE_HADM_Chan_Mode_PmExt_AntPerm_t *step_config_p;
#ifdef HADM_CFO_COMP_PER_STEP_VIA_PKTRAM
    hadm_proc_t *hadm_proc = &hadm_procs[hadm_meas_p->config_p->connIdx];
    bool_t compensate_cfo = (hadm_meas_p->config_p->role != HADM_ROLE_REFLECTOR) && (hadm_proc->ppm != 0) && ((hadm_meas_p->debug_flags & HADM_DBG_FLG_CFO_COMP_DIS) == 0U);
#endif
    int16_t cfo = 0;
    uint8_t aa_offset = 1U - ((uint8_t)hadm_meas_p->config_p->role & 0x1U); /* offset to Rx AA. & 0x1U to avoid wrong index with HADM_ROLE_SNIFFER */
    uint8_t cs_sync_ant_id = 0U;
    
    DEBUG_PIN1_SET

    assert(circ_buff_p->curr_step_idx < hadm_meas_p->config_p->stepsNb);
    n_steps = MIN(n_steps, (uint16)hadm_meas_p->config_p->stepsNb - (uint16)circ_buff_p->curr_step_idx);

    for (step_idx = 0; step_idx < (uint32_t)n_steps; step_idx++)
    {
        uint16_t hpm_cal_val;

        if ((hadm_meas_p->pkt_ram.config_write_ptr + circ_buff_p->max_step_size) > circ_buff_p->base_ptr + circ_buff_p->buff_len)
        {
            /* Need to wrap now: rewind write pointer to start and toggle page */
            hadm_meas_p->pkt_ram.config_write_ptr = circ_buff_p->base_ptr;
            circ_buff_p->curr_page ^= 1U;
        }
        
        step_config_p = &hadm_meas_p->config_p->chModePmAntMap[circ_buff_p->curr_step_idx];

        /* Compute HPM cal factor */
#ifdef HADM_PLL_CAL_INTERPOLATION
        if (step_config_p->mode == HADM_STEP_MODE2)
        {
            /* Per-step HPM cal not used for mode2 */
            hpm_cal_val = 0;
        }
        else /* mode 0, 1 or 3 */
        {
            /* Per-step HPM cal for packets - interpolated */
            hpm_cal_val = lcl_hadm_get_hpm_cal_interpolation((uint8_t)step_config_p->channel, hadm_device.cal_ch40[hadm_meas_p->config_p->rttPhy].hpm_cal_val);
        }
#else
        hpm_cal_val = hadm_device->cal_data[hadm_meas_p->config_p->rttPhy][step_config_p->channel].hpm_cal_val;
#endif
#ifdef HADM_CFO_COMP_PER_STEP_VIA_PKTRAM
        /* Compute CFO compensation */
        if (compensate_cfo && (step_config_p->mode != HADM_STEP_MODE0))
        {
            cfo = LCL_HAL_COMPUTE_STEP_CFO(hadm_proc->cfo_channel, step_config_p->channel, hadm_proc->ppm);
        }
#endif
        /* Compute CS_SYNC antenna ID to be used for this step */
        if (step_config_p->mode != HADM_STEP_MODE2)
        {
            cs_sync_ant_id = lcl_hadm_utils_get_CS_SYNC_antenna(hadm_meas_p);
        }

        /* RSM timing requires TONE_EXT to be set for correct Stable Phase test TX timing */
        if (hadm_meas_p->rsm_config.op_mode == XCVR_RSM_SQTE_STABLE_PHASE_TEST_MODE)
        {
          step_config_p->pm_ext = 0x3U;
        }
        /* Build common config header to PKT RAM circular buffer */
        LCL_HAL_BUILD_PKT_RAM_CONFIG_STEP(step_config_p, hadm_meas_p->pkt_ram.config_write_ptr, (uint16_t)cfo, hpm_cal_val, cs_sync_ant_id, hadm_meas_p->config_p->role);

        /* COMMON MODE 13 */
        if (step_config_p->mode != HADM_STEP_MODE2)
        {
            uint32_t nb_words_written;
            BLE_HADM_rttType_t rtt_type = (step_config_p->mode == HADM_STEP_MODE0) ? HADM_RTT_TYPE_CS_AA_ONLY_TIMING : hadm_meas_p->config_p->rttTypes;

            /* Build AAs and payloads to PKT RAM circular buffer */
            DEBUG_PIN1_SET
            if (step_config_p->mode == HADM_STEP_MODE0)
            {
                /* For mode0, AAs have been computed in advanced and are available in HAL config buffer */
                nb_words_written = 2U; /* pn1 and pn2 */
                *hadm_meas_p->pkt_ram.config_write_ptr = hadm_meas_p->config_p->pnList[step_idx].pn1;
                *(hadm_meas_p->pkt_ram.config_write_ptr + 1U) = hadm_meas_p->config_p->pnList[step_idx].pn2;
            }
            else
            {
                /* For non-mode0, AAs and payloads have to be computed on the flight */
                nb_words_written = BLE_HADM_DRBG_Generate_CS_SYNC_step(hadm_meas_p->config_p->connIdx, hadm_meas_p->config_p->subeventIdx,
                                                                                circ_buff_p->curr_step_idx,
                                                                                rtt_type, hadm_meas_p->pkt_ram.config_write_ptr);
            }
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
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN > 470)
        /* COMMON MODE 23 */
        if ((step_config_p->mode == HADM_STEP_MODE2) || (step_config_p->mode == HADM_STEP_MODE3))
        {
            /* Mode2-specific fields */
            *hadm_meas_p->pkt_ram.config_write_ptr++ = 0; /* phase_add_ap1-4 */
        }
#endif // NXP_RADIO_GEN == 470
        if (update_rsm_ptr)
        {
            LCL_HAL_UPDATE_PKT_RAM_CONFIG_STEP_PTR(circ_buff_p->curr_page, hadm_meas_p->pkt_ram.config_write_ptr);
        }
        circ_buff_p->curr_step_idx++;
    }
    
    DEBUG_PIN1_CLR
          
    return HADM_HAL_SUCCESS;
}

/*
 * Brute force to consume DRBG context to re-sync after a missed subevent.
 * All DRBG transactions are consumed when preparing the procedure, except those below which are consumed as the subevent runs.
 * - CS_SYNC AA
 * - CS_SYNC payloads
 * Note that, for CS_SYNC AA, there is no need to resync since they are processed by chunks of 128 bits, so taBitsUsed is always 128
 * which will triger generation of new bits on next invocation.
 */
static void lcl_hadm_consume_drbg(hadm_meas_t *hadm_meas_p)
{
    uint32_t step_idx;

    DEBUG_PIN1_SET

    for (step_idx = hadm_meas_p->pkt_ram.step_config.curr_step_idx; step_idx < hadm_meas_p->config_p->stepsNb; step_idx++)
    {
        BLE_HADM_Chan_Mode_PmExt_AntPerm_t * step_config_p = hadm_meas_p->config_p->chModePmAntMap + step_idx;
        if ((step_config_p->mode == HADM_STEP_MODE1) || (step_config_p->mode == HADM_STEP_MODE3))
        {
            /* Build AAs and payloads to PKT RAM circular buffer */
            (void) BLE_HADM_DRBG_Generate_CS_SYNC_step(hadm_meas_p->config_p->connIdx,
                                                        hadm_meas_p->config_p->subeventIdx,
                                                        (uint8)step_idx,
                                                        hadm_meas_p->config_p->rttTypes,
                                                        NULL);
        }
    }
    hadm_meas_p->pkt_ram.step_config.curr_step_idx = hadm_meas_p->config_p->stepsNb;

    DEBUG_PIN1_CLR
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
    bool_t proc_agc_locked = (hadm_proc->agc_idx != 0xFFU);
    hadm_sync_info_t *sync_info_p;
    BLE_HADM_STATUS_t status;
    
    DEBUG_PIN1_SET

    /* Select best mode0 (smallest AGC index to limit IQ saturation) */
    for (step_idx = 0; step_idx < mode0Nb; step_idx++)
    {
        sync_info_p = &hadm_meas_p->sync_info[step_idx];
        if (sync_info_p->valid != 0U)
        {
            hadm_meas_p->mode0_complete = true;

            /* If procedure has not locked its AGC yet (didn't get a subevent with at least one valid mode0) */
            /* look for smallest gain amongst valid mode 0's */
            if (!proc_agc_locked)
            {
                if (hadm_proc->agc_idx > sync_info_p->agc_idx)
                {
                    hadm_info_p->sync_step_id = (uint8_t)step_idx;
                    /* Record AGC index and keep the smallest */
                    hadm_proc->agc_idx = sync_info_p->agc_idx;
                    /* Record the estimated CFO and channel used for CFO estimation */
                    hadm_proc->cfo = sync_info_p->cfo;
                    hadm_proc->cfo_channel = (uint8_t)step_config_p[step_idx].channel;
                }
            }
        }
    }

    if (hadm_meas_p->config_p->role == HADM_ROLE_SNIFFER)
    {
        /* If procedure has not locked its AGC yet (didn't get a subevent with at least one valid mode0) */
        /* look for smallest gain amongst valid mode 0's on sniffer's reflector channel */
        if (!proc_agc_locked)
        {
            hadm_sync_info_t *sync_info2_p;
            for (step_idx = 0; step_idx < mode0Nb; step_idx++)
            {
                sync_info2_p = &hadm_meas_p->sync_info2[step_idx];
                if (hadm_proc->agc_idx2 > sync_info2_p->agc_idx)
                {
                    /* Record AGC index and keep the smallest */
                    hadm_proc->agc_idx2 = sync_info2_p->agc_idx;
                    /* Override the estimated CFO and channel used for CFO estimation.
                     * Sniffer will use CFO vs Reflector as the Initiator will compensate its own CFO vs Reflector.
                     */
                    hadm_proc->cfo = sync_info2_p->cfo;
                    hadm_proc->cfo_channel = (uint8_t)step_config_p[step_idx].channel;
                }
            }
        }
    }

    if (!proc_agc_locked)
    {
        /* Compute ppm based on measured CFO */
        hadm_proc->ppm = (int16_t)((hadm_proc->cfo * HADM_PPM_DIVIDER)/ ((int32_t)(uint32_t)HADM_CHAN_NUM_TO_MHZ((uint32_t)hadm_proc->cfo_channel)));
    }

    if (hadm_meas_p->mode0_complete)
    {
        /* Freeze AGC according to the index from the retained mode 0 */
        lcl_hal_xcvr_set_rxgain(hadm_proc->agc_idx);
        if (hadm_meas_p->config_p->role == HADM_ROLE_SNIFFER)
        {
            lcl_hal_xcvr_set_rxgain2(hadm_proc->agc_idx2);
        }

        /* Readjust DCOC calibration based on gain (DC residual impacts RTT 2Mbps dispersion in particular) */
        XCVR_LCL_OverrideDcoc(XCVR_LCL_CombineCoarseFineDc(&hadm_device.dcoc_cal_results[hadm_meas_p->config_p->rttPhy], hadm_proc->agc_idx), true);

        /* fallback to one shot RSSI */
        lcl_hal_xcvr_setup_rssi_continuous(false); 

        /* Compute CFO & time grid adjustment */
        if (hadm_meas_p->config_p->role != HADM_ROLE_REFLECTOR)
        {
            if ((hadm_meas_p->debug_flags & HADM_DBG_FLG_CFO_COMP_DIS) == 0U)
            {
#ifdef HADM_CFO_COMP_PER_STEP_VIA_FOM
                int32_t cfo;
                /* Enable FOM entry trigger for next step. TX trigger if initator, RX trigger otherwise. */
                LCL_HAL_ENABLE_FO_ENTRY(hadm_meas_p->config_p->role == HADM_ROLE_INITIATOR);
                /* Compute CFO compensation to apply at next step */
                /* Initiator or sniffer */
                cfo = LCL_HAL_COMPUTE_CHANNEL_CFO(hadm_meas_p->config_p->chModePmAntMap[mode0Nb].channel,
                                                  hadm_proc->ppm);
                (void)lcl_hadm_apply_cfo_per_step(cfo);
#else
                /* Initial CFO compensation */
                XCVR_LCL_RsmCompCfo(-hadm_proc->cfo);
#endif /* HADM_CFO_COMP_PER_STEP_VIA_FOM */

#ifdef HADM_CFO_COMP_PER_STEP_VIA_PKTRAM
                /* Update CFO programmed for already-built config steps */
                /* At this point only 1 step has been prepared in addition of the mode 0 steps */
                hadm_circ_buff_desc_t *circ_buff_p = &hadm_meas_p->pkt_ram.step_result;
                int16_t step_cfo;
                step_idx = mode0Nb + 1U;

                step_cfo = LCL_HAL_COMPUTE_STEP_CFO(hadm_proc->cfo_channel, step_config_p[step_idx].channel, hadm_proc->ppm);
              
                circ_buff_p = &hadm_meas_p->pkt_ram.step_config;
                assert(circ_buff_p->curr_step_idx == step_idx);
                uint32_t *config_write_ptr = circ_buff_p->base_ptr + (mode0Nb * LCL_HAL_PKT_RAM_STEP_CONFIG_MODE0_SIZE); /* skip mode 0s */
                
                LCL_HAL_UPDATE_CFO_IN_PKT_RAM_CONFIG_STEP(config_write_ptr, step_cfo);
#endif
            }
            
            /* Program time-grid adjustment (Initiator only) */
            lcl_hal_xcvr_program_time_adjustement((int32_t)hadm_proc->ppm/HADM_PPM_DIVIDER);
        }
        if (hadm_meas_p->config_p->role == HADM_ROLE_REFLECTOR)
        {   /* Reflector Only */
            /*
             * In some PKT-Tone cases, cfo_est may be non-null (depends on received AA) after PKT Rx, affecting badly tone phase.
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
 * Note: instead of passing the number of steps to process we should rather rely on PKTRAM circular buffer to detect end of available steps,
 * but this pointer is not reliable on KW47 as it is flushed right after EOS (HW bug fixed on next generations)
 */
static BLE_HADM_STATUS_t lcl_hadm_get_step_results(uint16 n_steps_required, hadm_meas_t *hadm_meas_p)
{
    hadm_circ_buff_desc_t *circ_buff_p = &hadm_meas_p->pkt_ram.step_result;
    BLE_HADM_Chan_Mode_PmExt_AntPerm_t *step_config_p = &hadm_meas_p->config_p->chModePmAntMap[circ_buff_p->curr_step_idx];
    hadm_info_t *hadm_info_p = &hadm_meas_p->info;
    hadm_proc_t *hadm_proc_p = &hadm_procs[hadm_meas_p->config_p->connIdx];
    uint8_t *res_buff1_p;
    uint8_t *res_buff1_in_p;
    uint8_t *res_buff2_in_p;
    uint8_t *res_buff2_p;
    uint8_t *res_buff_p;
    uint8_t nb_steps = 0;
    uint8_t step_mode0_no = 0;
    uint8_t rtt_pkt_no = 0;
    /* A sniffer will be reported as an initiator for both devices */
    BLE_HADM_role_t role = (hadm_meas_p->config_p->role == HADM_ROLE_SNIFFER) ? HADM_ROLE_INITIATOR : hadm_meas_p->config_p->role;
    bool_t synch_done = FALSE;
    uint32_t nb_iter;  /* Number of iterations on result records */

    uint32_t common_stat;
    uint32_t nadm_error = 0U;
    uint8_t rssi_nb = 0x7F; /* invalid value */
    uint32_t rtt_data_raw;
    uint32_t tpm = 0U;  /* TPM timestamp unit is 1/32MHz */
    int ap;
    uint32_t iq[HADM_MAX_NB_ANTENNA_PATHS+1U];

    assert(circ_buff_p->curr_step_idx < hadm_meas_p->config_p->stepsNb);
    
    if (circ_buff_p->curr_step_idx > 0U)
    {
        hadm_meas_p->result_p = lcl_hadm_utils_get_result_buffer();
        
        if (hadm_meas_p->result_p == NULL)
        {
#ifdef HAL_ENABLE_ASSERT_ON_STRESS
            assert(FALSE);
#endif
            return HADM_HAL_MEMORY_FULL;
        }
        hadm_meas_p->result_p->connIdx = hadm_meas_p->config_p->connIdx;
        hadm_meas_p->result_p->subeventIdx = hadm_meas_p->config_p->subeventIdx; /* echo subeventIdx */
    }
    hadm_meas_p->result_p->firstStepCollected = circ_buff_p->curr_step_idx;

    DEBUG_PIN1_SET

    /* Prepare for NADM metric calculations */
    BLE_HADM_rttPhyMode_t rate = hadm_meas_p->config_p->rttPhy;
    uint8_t fm_corr_target;
    uint8_t fm_corr_div;
    XCVR_LCL_GetNadmMetricCalFactors((XCVR_RSM_SQTE_RATE_T)rate, (XCVR_RSM_RTT_TYPE_T)hadm_meas_p->config_p->rttTypes, fm_corr_target, fm_corr_div);

    /* Copy CFO into frequencyCompensation from procedure if initiator (already set to fixed value otherwise) */
    if (role == HADM_ROLE_INITIATOR)
    {
        uint8_t *freq_comp_p = (uint8_t *)&hadm_meas_p->result_p->frequencyCompensation;
        HADM_SET_RTT_CFO(TRUE, hadm_proc_p->ppm, freq_comp_p);
    }

    /* Compute RPL once per subevent.*/
    if (hadm_meas_p->result_p->referencePwrLevel == HADM_INVALID_REFERENCE_POWER_LEVEL)
    {
        if (hadm_proc_p->agc_idx <= 11U)
        {
            /* see comment of lcl_hal_xcvr_compute_rpl */
            hadm_meas_p->result_p->referencePwrLevel = (int8_t)lcl_hal_xcvr_compute_rpl(11U - hadm_proc_p->agc_idx);
        }
        if (hadm_proc_p->agc_idx2 <= 11U)
        {
            /* agc_idx2 only set if sniffer mode is active */
            hadm_meas_p->result_p->referencePwrLevel2 = (int8_t)lcl_hal_xcvr_compute_rpl(11U -hadm_proc_p->agc_idx2);
        }
    }

    res_buff1_in_p = hadm_meas_p->result_p->resultBuffer;
    res_buff1_p = res_buff1_in_p;
    res_buff2_in_p = hadm_meas_p->result_p->resultBuffer2;
    res_buff2_p = res_buff2_in_p;

    res_buff_p = res_buff1_p;

    while (circ_buff_p->curr_step_idx < hadm_meas_p->config_p->stepsNb)
    {
        if ((hadm_meas_p->pkt_ram.result_read_ptr + circ_buff_p->max_step_size) > circ_buff_p->base_ptr + circ_buff_p->buff_len)
        {
            /* Need to wrap now: rewind write pointer to start and toggle page */
            hadm_meas_p->pkt_ram.result_read_ptr = circ_buff_p->base_ptr;
            circ_buff_p->curr_page ^= 1U;
        }

        /* Read one or two (sniffer) record */
        nb_iter = 0;
        do
        {
            /* Decode common status */
            common_stat = *hadm_meas_p->pkt_ram.result_read_ptr++;
            assert(circ_buff_p->curr_step_idx == (common_stat & COM_RES_HDR_STEP_ID_STEP_ID_MASK)); /* HW and SW step_id should be aligned! */

            if (LCL_HAL_GET_PKT_RAM_COM_RES_HDR_TIME_DRIFT(common_stat) != 0U)
            {
                hadm_info_p->num_time_adj++; /* record time_adj for debug */
            }

            /* Decode packet status if present */
            if (step_config_p->mode != HADM_STEP_MODE2)
            {
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN == 470)
                rssi_nb = (uint8_t)((hadm_meas_p->pkt_ram.result_read_ptr[0U] & COM_MODE_013_RES_BODY_NADM_ERROR_RSSI_RSSI_NB_MASK) >> COM_MODE_013_RES_BODY_NADM_ERROR_RSSI_RSSI_NB_SHIFT);
#else
                rssi_nb = (uint8_t)((hadm_meas_p->pkt_ram.result_read_ptr[2U] & 0xFF0000U) >> 16U);
#endif
                nadm_error = *hadm_meas_p->pkt_ram.result_read_ptr++;
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
                for (ap = 0; ap <= (int)hadm_meas_p->n_ap; ap++)
                {
                    iq[ap] = *hadm_meas_p->pkt_ram.result_read_ptr++;
                }
            }

            /* Store Step_Data "header" */
            *res_buff_p++ = (uint8_t)step_config_p->mode; /* Step_Mode */
            *res_buff_p++ = (uint8_t)step_config_p->channel; /* Step_Channel */

            /* Store Step_Data */
            if (step_config_p->mode == HADM_STEP_MODE0)
            {
                bool vld = (bool)hadm_meas_p->sync_info[step_mode0_no].valid;
                /* (Hz*100) / MHz  => 0.01 ppm unit */
                int32_t ppm = (hadm_meas_p->sync_info[step_mode0_no].cfo * HADM_PPM_DIVIDER) / (int32_t)(uint32_t)HADM_CHAN_NUM_TO_MHZ((uint32_t)step_config_p->channel);
                *res_buff_p++ = (uint8_t)BLE_HADM_STEP0_REPORT_SIZE(role); /* Step_Data_Length */
                *res_buff_p++ = (uint8_t)HADM_SET_RTT_AA_QUALITY(vld); /* Packet_AA_Quality */
                *res_buff_p++ = (uint8_t)HADM_SET_RTT_RSSI(vld, hadm_meas_p->sync_info[step_mode0_no].rssi); /* Packet RSSI */
                *res_buff_p++ = hadm_meas_p->pkt_ram_data_in_flight[hadm_meas_p->data_in_flight_r_idx].cs_sync_ant_id + 1U;  /* Packet_Antenna (1 byte) */
                if (role == HADM_ROLE_INITIATOR)
                {
                    HADM_SET_RTT_CFO(vld, ppm, res_buff_p); /* Measured_Freq_Offset (2 bytes) */
                }
                if (!synch_done && vld)
                {
                    /* Compute final value for syncDelayUs which represents time starting from RSM trigger */
                    /* syncOffsetUs = RSM_TRIGGER_DELAY + TPM timestamp - pkt_header_duration - aa_match_delay */
                    hadm_meas_p->result_p->syncDelayUs = (uint16_t)HADM_RTT_TS_TO_US(tpm);
                    assert((hadm_meas_p->rsm_trigger_delay + hadm_meas_p->result_p->syncDelayUs) > (HADM_1ST_BIT_TO_AA_MATCH_DURATION_US(hadm_meas_p->config_p->rttPhy)));
                    hadm_meas_p->result_p->syncDelayUs = hadm_meas_p->result_p->syncDelayUs + hadm_meas_p->rsm_trigger_delay - HADM_1ST_BIT_TO_AA_MATCH_DURATION_US(hadm_meas_p->config_p->rttPhy);
                    hadm_meas_p->result_p->syncDelayUs -= (step_mode0_no * hadm_meas_p->step_duration[0]); /* cope for missed mode0s */
                    synch_done = TRUE;
                }
                step_mode0_no++;
                rtt_pkt_no++;
            }
            else
            {
                assert(step_config_p->mode <= HADM_STEP_MODE3);

                if ((step_config_p->mode == HADM_STEP_MODE1) || (step_config_p->mode == HADM_STEP_MODE3))
                {
                    xcvr_lcl_rtt_data_t rtt_data;
                    int32_t frac_delay = 0;
                    int32_t rtt_ts = 0;
                    uint8_t nadm_metric = 0xFF; /* NADM not available by default */
                    uint8_t nadm_symb_err = 0x0U; 
                    rtt_pkt_no++;
                    (void)XCVR_LCL_UnpackRttResult((xcvr_lcl_rtt_data_raw_t *)(void *)&rtt_data_raw, &rtt_data, (XCVR_RSM_SQTE_RATE_T)rate); /* OJE TODO: optimize...*/
                    if (rtt_data.rtt_vld && rtt_data.rtt_found)
                    {
                        int32_t ffo_correction = 0;
                        DEBUG_PIN1_SET
                        /* Compute integer and fractional adjustment in ns */
                        frac_delay = lcl_hadm_hartt_compute_fractional_delay((uint32_t)rate, hadm_meas_p->pkt_ram_data_in_flight[hadm_meas_p->data_in_flight_r_idx].aa_rx,
                                                                             (int16_t)rtt_data.p_delta, (int32_t)rtt_data.int_adj);
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

                        /* in half ns to keep ts_delay_hns precision */
                        rtt_ts = ((int32_t)(uint32_t)HADM_RTT_TS_TO_NS(tpm) + frac_delay)*2;
                        rtt_ts = rtt_ts - (int32_t)hadm_meas_p->ts_hw_delay_hns;
                        if (role == HADM_ROLE_INITIATOR)
                        {
                            /* "Device A shall implement frequency-based timing compensation" */
                            /* ppm is expressed in 0.01 ppm */
                            ffo_correction = (int32_t)(((int64_t)rtt_ts * hadm_proc_p->ppm)/100000000);
                        }
                        rtt_ts = rtt_ts - (int32_t)hadm_meas_p->ts_nominal_delay_hns + ffo_correction;
                        if ((step_config_p->mode == HADM_STEP_MODE3) && ((step_config_p->pm_ext & 0x1U) != 0U))
                        {
                            /* Remove an extra T_PM + T_SW in mode 3 case if required */
                            rtt_ts -= (int32_t)hadm_meas_p->ts_extra_delay_hns;
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
                            uint32_t nadm_fm_corr_value = ((nadm_error & COM_MODE_013_RES_BODY_NADM_ERROR_RSSI_RAW_NADM_FM_CORR_VALUE_MASK)>>COM_MODE_013_RES_BODY_NADM_ERROR_RSSI_RAW_NADM_FM_CORR_VALUE_SHIFT);
                            XCVR_LCL_CalcNadmMetric(nadm_fm_corr_value, (int16_t)fm_corr_target, (int16_t)fm_corr_div, nadm_metric);
                            nadm_symb_err = (uint8_t)((nadm_error & COM_MODE_013_RES_BODY_NADM_ERROR_RSSI_RAW_NADM_FM_SYMB_ERR_VALUE_MASK)>>COM_MODE_013_RES_BODY_NADM_ERROR_RSSI_RAW_NADM_FM_SYMB_ERR_VALUE_SHIFT);
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
                    *res_buff_p++ = (((nadm_symb_err << 4U) & 0xF0U) | (uint8_t)HADM_SET_RTT_AA_QUALITY(rtt_data.rtt_vld)); /* Payload_errors[7:4] | Packet_AA_Quality[3:0] (1 byte) */
                    *res_buff_p++ = nadm_metric; /* Packet_NADM */
                    *res_buff_p++ = (uint8_t)HADM_SET_RTT_RSSI(rtt_data.rtt_vld, (int8_t)rssi_nb); /* Packet_RSSI (1 byte) */
                    HADM_SET_RTT_TS_DIFF(rtt_data.rtt_vld, rtt_ts, res_buff_p); /* ToX-ToX time diff (2 bytes) */
                    *res_buff_p++ = hadm_meas_p->pkt_ram_data_in_flight[hadm_meas_p->data_in_flight_r_idx].cs_sync_ant_id + 1U;  /* Packet_Antenna (1 byte) */
                }

                if (step_config_p->mode != HADM_STEP_MODE1)    /* MODE2 and MODE3 */
                {
                    uint32_t curr_iq;
                    uint8_t ant_id_seq[HADM_MAX_NB_ANTENNA_PATHS];
                    /* Keep pm extension bit corresponding to peer device role */
                    uint8_t t_pm_ext = ((uint8_t)step_config_p->pm_ext >> (uint8_t)role) & 0x1U;
                    if (step_config_p->mode == HADM_STEP_MODE2)
                    {
                        *res_buff_p++ = BLE_HADM_STEP2_REPORT_SIZE(hadm_meas_p->n_ap); /* Step_Data_Length */
                    }
                    *res_buff_p++ = (uint8_t)step_config_p->ant_perm; /* Antenna_Permutation_Index */
                    bool_t inpr_refl = (hadm_meas_p->config_p->inlinePhaseReturn == 1U) && (hadm_meas_p->config_p->role == HADM_ROLE_REFLECTOR);
                    /* Store PCT[ap], 3 bytes each 22 significant bits  +  Tone Quality Indicator [ap] (1 byte each) */
                    lcl_hadm_utils_get_antenna_id_sequence(hadm_meas_p, circ_buff_p->curr_step_idx, ant_id_seq);
                    for(ap = 0; ap < (int)hadm_meas_p->n_ap; ap++)
                    {
                        HADM_DECODE_HW_RTP_PCT(iq[ap], inpr_refl, curr_iq);
                        lcl_hadm_measurement_phase_rotation(&curr_iq, (uint8_t)step_config_p->channel, ant_id_seq[ap]);
                        HADM_ENCODE_HCI_RTP_PCT(curr_iq, res_buff_p);
                        HADM_SET_RTP_TONE_QUALITY(iq[ap], res_buff_p, 0U/*NA*/);
                    }
                    /* Determine if N_AP+1 PCT has been received or not */
                    if ((step_config_p->mode == HADM_STEP_MODE2) || (role == HADM_ROLE_REFLECTOR) || ((step_config_p->pm_ext & 0x1U) != 0U))
                    {
                        HADM_DECODE_HW_RTP_PCT(iq[ap], inpr_refl, curr_iq);
                        lcl_hadm_measurement_phase_rotation(&curr_iq, (uint8_t)step_config_p->channel, ant_id_seq[ap]);
                        HADM_ENCODE_HCI_RTP_PCT(curr_iq, res_buff_p);
                        HADM_SET_RTP_TONE_QUALITY_WITH_EXT_SLOT(iq[ap], res_buff_p, t_pm_ext);
                    }
                    else
                    {
                        curr_iq = 0;
                        HADM_ENCODE_HCI_RTP_PCT(curr_iq, res_buff_p);
                        HADM_SET_RTP_TONE_QUALITY_WITH_EXT_SLOT(0x3U, res_buff_p, 0U);
                    }
                }
            }
            if (hadm_meas_p->config_p->role == HADM_ROLE_SNIFFER)
            {
                /* Switch output result buffer */
                if (nb_iter == 0U)
                {
                    res_buff1_p = res_buff_p;
                    res_buff_p = res_buff2_p;
                    nb_iter = 1;
                }
                else
                {
                    res_buff2_p = res_buff_p;
                    res_buff_p = res_buff1_p;
                    nb_iter = HADM_SNIFFER_DEVICE_NB;
                }
            }
            else
            {
                res_buff1_p = res_buff_p;
                nb_iter = HADM_SNIFFER_DEVICE_NB;
            }
        } while (nb_iter < HADM_SNIFFER_DEVICE_NB);
        
        if (step_config_p->mode != HADM_STEP_MODE2)
        {
            if (hadm_meas_p->data_in_flight_r_idx >= (HADM_HAL_PKT_RAM_IN_FLIGHT_DATA_BUFFER_SIZE - 1U))
            {
                hadm_meas_p->data_in_flight_r_idx = 0;
            }
            else
            {
                hadm_meas_p->data_in_flight_r_idx++;
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
    assert((res_buff1_p - res_buff1_in_p) <= hadm_meas_p->result_p->resultBufferSize); /* check we did not write out of bounds */
    assert((res_buff2_p - res_buff2_in_p) <= hadm_meas_p->result_p->resultBufferSize); /* check we did not write out of bounds */
    
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
    uint32_t abort_reason = 0;
    
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
    else if ((irq_status & LCL_HAL_XCVR_RSM_IRQ_ABORT) != 0U)
    {
        /* RSM has aborted */
        type = HADM_EVENT_EOS;
        abort_reason = XCVR_MISC->RSM_CSR & (XCVR_MISC_RSM_CSR_RSM_PLL_ABORT_MASK | XCVR_MISC_RSM_CSR_RSM_UNDR_ERR_MASK | XCVR_MISC_RSM_CSR_RSM_OVF_ERR_MASK | XCVR_MISC_RSM_CSR_RSM_TIMEOUT0_ABORT_MASK);
        
        if ((abort_reason & XCVR_MISC_RSM_CSR_RSM_TIMEOUT0_ABORT_MASK) != 0U)
        {
            hal_status = HADM_HAL_ABORTED_SYNC;
        }
        else if (((abort_reason & XCVR_MISC_RSM_CSR_RSM_UNDR_ERR_MASK) != 0U) &&
                 (!hadm_meas_p->mode0_complete))
        {
            /* KW47 HW bug workaround on reflector: when mode0 retry is enabled, there's a small
             * probability that the RSM does not send an abort IRQ in case the last mode0 is
             * received (AA match) just after transitioning to 1st non_mode0 step.
             * In this case, the condition is detected by an RSM underrun on 2nd non-mode0 step.
             * This situation should be avoided by the LL ensuring that mode0 timeout
             * is only enabled when devices are properly synchronized (small window widening).
             * Note: when this occurs there's a dummy transmission for the 1st non-mode0 step.
             */
            hal_status = HADM_HAL_ABORTED_SYNC;
        }
        else if ((abort_reason & XCVR_MISC_RSM_CSR_RSM_UNDR_ERR_MASK) != 0U)
        {
            /* May occur in case of heavily loaded system causing IRQ drift (critical section, ...) */
            hal_status = HADM_HAL_ABORTED;
        }
        else  /* should not occur */
        {
            assert(FALSE);
            hal_status = HADM_HAL_ABORTED;
        }
    }
    else if ((irq_status & LCL_HAL_XCVR_RSM_IRQ_EOS) != 0U)
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
        if ((irq_status & LCL_HAL_XCVR_RSM_IRQ_FM) != 0U)
        {
            /* Mode 0 interrupt */
            hadm_circ_buff_desc_t *circ_buff_p = &hadm_meas_p->pkt_ram.step_result;
            uint32_t *rsm_result_ptr;
            bool_t aa_det;
            uint32_t mode0_index = circ_buff_p->curr_step_idx;
            uint32_t mode0_step_size = LCL_HAL_PKT_RAM_STEP_RESULT_MODE01_SIZE;
            if (hadm_meas_p->config_p->role == HADM_ROLE_SNIFFER)
            {
                /* 2nd result */
                mode0_step_size *= HADM_SNIFFER_DEVICE_NB;
            }

            do
            {
                rsm_result_ptr = circ_buff_p->base_ptr + (mode0_index * mode0_step_size);
                aa_det = lcl_hal_xcvr_decode_mode0_step(hadm_meas_p->sync_info, rsm_result_ptr, hadm_meas_p->config_p->rttPhy);
                if (hadm_meas_p->config_p->role == HADM_ROLE_SNIFFER)
                {
                    /* Read 2nd result */
                    rsm_result_ptr += LCL_HAL_PKT_RAM_STEP_RESULT_MODE01_SIZE;
                    (void)lcl_hal_xcvr_decode_mode0_step(hadm_meas_p->sync_info2, rsm_result_ptr, hadm_meas_p->config_p->rttPhy);
                }
                mode0_index++;
            } while ((hadm_meas_p->config_p->role == HADM_ROLE_REFLECTOR) && (aa_det == FALSE) && (circ_buff_p->curr_step_idx == 0U)); /* first mode 0 was missed, go to next */
            circ_buff_p->curr_step_idx = (uint8_t)mode0_index;

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
                        (void)lcl_hadm_set_steps_config(hadm_meas_p->pkt_ram.nb_steps_before_irq, hadm_meas_p, TRUE);
                    }
                }
                circ_buff_p->curr_step_idx = 0; /* reset current read index to process results in next step interrupt */
            }
        }
        else if ((irq_status & LCL_HAL_XCVR_RSM_IRQ_STEP) != 0U)
        {
            if (!hadm_meas_p->mode0_complete)
            {
                /* postpone mode0 processing as we need CFO and AGC to be frozen before reporting to LL */
                type = HADM_EVENT_INVALID;
                hadm_meas_p->pkt_ram.nb_irq_steps_postponed++;
            }
            else
            {
                uint8_t steps_to_process; /* HW workaround, see note in lcl_hadm_get_step_results */
                bool_t shorten_last_step_processing = FALSE;

                /* Step interrupt */
                if (hadm_meas_p->pkt_ram.step_config.curr_step_idx < hadm_meas_p->config_p->stepsNb)
                {
                    /* Push more config steps if needed */
                    (void)lcl_hadm_set_steps_config(hadm_meas_p->pkt_ram.nb_steps_before_irq, hadm_meas_p, TRUE);
                }
                else
                {
                    shorten_last_step_processing = TRUE;
                }
                DEBUG_PIN0_PULSE
                /* Read available step results */
                if ((hadm_meas_p->pkt_ram.nb_irq_steps_handled == 0U) &&
                    (hadm_meas_p->pkt_ram.nb_steps_before_irq < hadm_meas_p->config_p->mode0Nb))
                {
                    /* Process postponed IRQ plus current one */
                    steps_to_process = (hadm_meas_p->pkt_ram.nb_irq_steps_postponed + 1U) * hadm_meas_p->pkt_ram.nb_steps_before_irq;
                }
                else
                {
                    steps_to_process = hadm_meas_p->pkt_ram.nb_steps_before_irq;
                }
                hal_status = lcl_hadm_get_step_results((uint16_t)steps_to_process, hadm_meas_p);
                if (shorten_last_step_processing)
                {
                    /* We are close to EOS, set one step per interrupt to minimize EOS processing latency */
                    hadm_meas_p->pkt_ram.nb_steps_before_irq = 1;
                    lcl_hal_pkt_ram_config_rsm_int_nbstep(hadm_meas_p->pkt_ram.nb_steps_before_irq);
                }
                if (hal_status == HADM_HAL_SUCCESS)
                {
                    type = HADM_EVENT_STEP_INT;
                }
                else
                {
#ifdef HAL_ENABLE_ASSERT_ON_STRESS
                    assert(false);
#endif
                    type = HADM_EVENT_EOS;
                }
                hadm_meas_p->pkt_ram.nb_irq_steps_handled++;
            }
        }
        else
        {
          /* MISRA rule 15.7 */
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
        {   hadm_info_p->flags |= FLAGS_HADM_AGC_NOT_FROZEN; }

        if (hadm_info_p->sync_rssi < -93)
        {   hadm_info_p->flags |= FLAGS_HADM_NO_SIGNAL; }
        
        if (hal_status == HADM_HAL_ABORTED_SYNC)
        {   hadm_info_p->flags |= FLAGS_HADM_SYNC_ERROR; }
        else if (hal_status == HADM_HAL_ABORTED)
        {
            hadm_info_p->flags |= FLAGS_HADM_ABORT;
            hadm_info_p->flags |= (abort_reason << 2U) & FLAGS_HADM_RSM_ABORT_REASON;
        }
        else
        {
          /* MISRA rule 15.7 */
        }
    }
    
    if (((hadm_meas_p->debug_flags & HADM_DBG_FLG_DBG_INFO) != 0U) &&
        (hadm_meas_p->pkt_ram.step_result.curr_step_idx == hadm_meas_p->config_p->stepsNb))
    {
        /* On KW47, just report hadm_info_t for debug. Done in the last reported event. */
        hadm_meas_p->result_p->debugBuffer = (uint8_t *)&hadm_meas_p->info;
        hadm_meas_p->result_p->debugBufferSize = (uint16_t)sizeof(hadm_info_t);
    }

    /* CLear IRQ status flags */
    LCL_HAL_RSM_SET_IRQ_STATUS_FLAGS(irq_status);
    
    /* Notify LL */
    DEBUG_PIN1_SET
    if (HADM_EVENT_INVALID != type)
    {
        BLE_HADM_NotifyLL(hadm_meas_p->config_p->connIdx, hadm_meas_p->result_p, type, hal_status);
    }
    DEBUG_PIN1_CLR

    if (type == HADM_EVENT_EOS)
    {
        lcl_hadm_measurement_cleanup(hadm_meas_p);
        hadm_device.active_meas_p = NULL;
    }

    DEBUG_PIN0_CLR
}

#ifdef HADM_CFO_COMP_PER_STEP_VIA_FOM
void BRF_INT_IRQHandler(void)
{
    uint32_t irq_status = XCVR_MISC->XCVR_STATUS;

    if ((irq_status & XCVR_MISC_XCVR_STATUS_TSM_IRQ0_MASK) != 0U)
    {
        /* Write one to clear status bit */
        XCVR_MISC->XCVR_STATUS = XCVR_MISC_XCVR_STATUS_TSM_IRQ0_MASK;
        hadm_meas_t *hadm_meas_p = hadm_device.active_meas_p;
        hadm_proc_t *hadm_proc = &hadm_procs[hadm_meas_p->config_p->connIdx];
        bool_t compensate_cfo = (hadm_meas_p->config_p->role != HADM_ROLE_REFLECTOR) && (hadm_proc->ppm != 0) && ((hadm_meas_p->debug_flags & HADM_DBG_FLG_CFO_COMP_DIS) == 0U);

        if (compensate_cfo)
        {
            uint32_t rsm_curr_step = LCL_HAL_RSM_GET_CURRENT_STEP;
            /* Prepare CFO for next step */
            rsm_curr_step ++;
            if ((rsm_curr_step > hadm_meas_p->config_p->mode0Nb) && (rsm_curr_step < hadm_meas_p->config_p->stepsNb))
            {
                int32_t cfo;
                BLE_HADM_Chan_Mode_PmExt_AntPerm_t *step_config_p = &hadm_meas_p->config_p->chModePmAntMap[rsm_curr_step];
                /* Initiator or sniffer */
                cfo = LCL_HAL_COMPUTE_CHANNEL_CFO(step_config_p->channel, hadm_proc->ppm);
                (void)lcl_hadm_apply_cfo_per_step(cfo);
            }
        }
    }

}
#endif /* HADM_CFO_COMP_PER_STEP_VIA_FOM */

/* EOF */