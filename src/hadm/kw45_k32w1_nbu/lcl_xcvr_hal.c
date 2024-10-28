/**
 * @file lcl_xcvr_hal_gen4p5.c
 *
 * This file implements LCL XCVR HAL for gen4p5 radio.
 */
/*
 * Copyright 2021-2024 NXP
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* === Includes ============================================================ */
#include "fsl_os_abstraction.h"
#include "EmbeddedTypes.h"
#include "nxp2p4_xcvr.h"
#include "nxp_xcvr_lcl_ctrl.h"
#include "lcl_hadm_measurement.h"
#include "lcl_xcvr_hal.h"
#include "controller_api_ll.h"

#if (NXP_RADIO_GEN != 450)
#error LCL XCVR HAL is not compatible with radio version
#endif

/* === Types =============================================================== */
typedef struct soc_xcvr_settings_tag 
{
    uint32_t xcvr_rx_dig_agc_ctrl;
    uint32_t xcvr_rx_dig_agc_ctrl_stat;
    uint32_t xcvr_rx_dig_agc_ovrd;
    uint32_t xcvr_rx_dig_rssi_global_ctrl;
    uint32_t xcvr_rx_dig_nb_rssi_ctrl0;
    uint32_t xcvr_rx_dig_dcoc_ctrl_0;
    uint32_t xcvr_rx_dig_dcoc_ctrl_2;
    uint32_t xcvr_rx_dig_ctrl_1;
    uint32_t xcvr_radio_ctrl_rf_ctrl;

    /* used by XCVR RSM driver */
    rsm_reg_backup_t xcvr_regs_backup;
    xcvr_lcl_tsm_config_t tsm_regs_backup;
    //XCVR_TSM_Type tsm_regs_backup;
    
    /* FIXME: remove duplicates */
} lcl_xcvr_hal_xcvr_settings_t;

/* === Macros ============================================================== */

#define LCL_HAL_CLIP_RSSI(rssi_u16, rssi_s8) \
    rssi_u16 |= (((rssi_u16 & 0x0100U) == 0x0100U) ? 0xFE00U : 0x0U); /* Sign extend from 9 to 16 bits */ \
    if ((int16_t)rssi_u16 < INT8_MIN) \
        rssi_s8 = INT8_MIN; /* in LCL context, we don't expect to deal with RSSI values lower than -128dBm */\
    else \
        rssi_s8 = (int8_t)rssi_u16;

/* === Globals ============================================================= */
lcl_xcvr_hal_xcvr_settings_t  xcvr_settings;

static uint32_t dcoc_ctrl_2[HADM_RTT_PHY_MAX] = {0, 0}; /* used to store XCVR_RX_DIG->DCOC_CTRL2, typically after calibration for CS operation */

/* === Externals =========================================================== */

/* === Prototypes ========================================================== */

/* === Implementation ====================================================== */

/* *****************************RSM related **********************************/

/*
 *  0b10010..FC_RX2TX (Frequency Change RX2TX).
 *  0b10011..FC_TX2RX (Frequency Change TX2RX).
 *  0b10100..WD (Warmdown)
 */
bool_t lcl_xcvr_hal_check_rsm_state(uint32_t expected_state, XCVR_RSM_RXTX_MODE_T role)
{
    bool_t ok = FALSE;
    t_hadm_rsm_states rsm_state = (t_hadm_rsm_states)((XCVR_MISC->RSM_CSR & XCVR_MISC_RSM_CSR_RSM_STATE_MASK) >> XCVR_MISC_RSM_CSR_RSM_STATE_SHIFT);
    
    switch (expected_state)
    {
        case LCL_HAL_XCVR_RSM_STATE_FC:
        {
            if (((XCVR_RSM_TX_MODE == role) && (RSM_FC_RX2TX == rsm_state)) || 
                ((XCVR_RSM_RX_MODE == role) && (RSM_FC_TX2RX == rsm_state)))
            {
                ok = TRUE;
            }
            break;
        }
        case LCL_HAL_XCVR_RSM_STATE_IDLE:
        {
            if ((rsm_state == RSM_IDLE) || (rsm_state == RSM_WD))
            {
                ok = TRUE;
            }
            break;
        }
        case LCL_HAL_XCVR_RSM_STATE_RX_SYNC:
        {
            if (rsm_state == RSM_DT_RX_SYN)
            {
                ok = TRUE;
            }
            break;
        }
        default:
        {
            ok = FALSE;
            break;
        }
    }
    return ok;
}

/* uints are us */
void lcl_xcvr_hal_rsm_configure_dma_mask(uint32_t pm_dur, uint32_t pm_dly, uint32_t mode0_dur, uint32_t mode0_dly)
{
    uint32_t temp;
    
    temp = XCVR_MISC->RSM_CTRL3;
    temp &= ~(XCVR_MISC_RSM_CTRL3_RSM_DMA_DUR_MASK);
    temp |= XCVR_MISC_RSM_CTRL3_RSM_DMA_DUR(pm_dur);
    XCVR_MISC->RSM_CTRL3 = temp;
    
    temp = XCVR_MISC->RSM_CTRL4;
    temp &= ~(XCVR_MISC_RSM_CTRL4_RSM_DMA_DUR0_MASK | XCVR_MISC_RSM_CTRL4_RSM_DMA_DLY0_MASK | XCVR_MISC_RSM_CTRL4_RSM_DMA_DLY_MASK);
    temp |= XCVR_MISC_RSM_CTRL4_RSM_DMA_DUR0(mode0_dur) | XCVR_MISC_RSM_CTRL4_RSM_DMA_DLY0(mode0_dly) | XCVR_MISC_RSM_CTRL4_RSM_DMA_DLY(pm_dly);
    XCVR_MISC->RSM_CTRL4 = temp;
    
    /* Set DMA mask delay and ref period to zero in order to be used with RSM */ 
    XCVR_MISC->LCL_DMA_MASK_DELAY = XCVR_MISC_LCL_DMA_MASK_DELAY_DMA_MASK_DELAY_OFF(0) | XCVR_MISC_LCL_DMA_MASK_DELAY_DMA_MASK_DELAY(0);
    XCVR_MISC->LCL_DMA_MASK_PERIOD = 0;
}

/* Returns CFO in Hz as estimated by HARTT block */
int32_t lcl_xcvr_hal_get_cfo(XCVR_RSM_SQTE_RATE_T rate)
{
    int16_t cfo16;
    int32_t cfo32;
    /* format is sfix16En15 (15 bits frac, 1 bit sign). CFO=CFO_reg*R/2, where R is the bit rate. */
    cfo16 = ((int16_t) ((XCVR_2P4GHZ_PHY->RTT_STAT & GEN4PHY_RTT_STAT_RTT_CFO_MASK) >> GEN4PHY_RTT_STAT_RTT_CFO_SHIFT));
    cfo32 = (int32_t)cfo16;
    if (rate == XCVR_RSM_RATE_1MBPS)
        cfo32 *= 15; /* 1e6 / (2 * 2^15) */
    else
        cfo32 *= 30; /* 1e6 / 2^15 */
    return cfo32;
}

bool_t lcl_xcvr_hal_rsm_rtt_is_valid(uint32_t step_no)
{
    uint32_t rtt_res = *((uint32_t *)RSM_RTT_RAM + step_no);
    return ((rtt_res & (XCVR_RSM_RTT_VALID_MASK | XCVR_RSM_RTT_FOUND_MASK)) == (XCVR_RSM_RTT_VALID_MASK | XCVR_RSM_RTT_FOUND_MASK));
}

uint8_t lcl_xcvr_hal_rsm_get_cal_mode(const xcvr_lcl_rsm_reg_config_t *rsm_config_p)
{
    uint8_t cal_auto = 0;
    if (((rsm_config_p->RSM_CTRL3 & XCVR_MISC_RSM_CTRL3_RSM_CTUNE_MASK) >> XCVR_MISC_RSM_CTRL3_RSM_CTUNE_SHIFT) == 0)
      cal_auto |= LCL_HAL_XCVR_RSM_CTUNE_CAL_IS_AUTO;
    return cal_auto;
}

/**** DCOC control *****/

/* Store DCOC values from current ones */
void lcl_xcvr_hal_store_dcoc_cal(BLE_HADM_rttPhyMode_t rate)
{
    assert(rate < HADM_RTT_PHY_MAX);
    dcoc_ctrl_2[rate] = XCVR_LCL_SetupManualDcoc();
}

/* ******************************* Common ************************************/

static inline void lcl_hal_xcvr_setup_agc(void)
{
    uint32 temp;
    
    xcvr_settings.xcvr_rx_dig_agc_ovrd = XCVR_RX_DIG->AGC_OVRD;
    
    /* Enable AGC WBD mode */
    xcvr_settings.xcvr_rx_dig_agc_ctrl = XCVR_RX_DIG->AGC_CTRL;
    temp = xcvr_settings.xcvr_rx_dig_agc_ctrl;
    temp &= ~XCVR_RX_DIG_AGC_CTRL_AGC_WBD_EN_MASK;
    temp |= XCVR_RX_DIG_AGC_CTRL_AGC_WBD_EN(2); /*  *  0b10..AGC WBD step1 and step2 is enabled */
    XCVR_RX_DIG->AGC_CTRL = temp;
    
    xcvr_settings.xcvr_rx_dig_agc_ctrl_stat = XCVR_RX_DIG->AGC_CTRL_STAT;
    temp = xcvr_settings.xcvr_rx_dig_agc_ctrl_stat;
    
    /* Enable AGC store function to allow AGC index to be read after rx_dig_en has been de-asserted */
    temp &= ~XCVR_RX_DIG_AGC_CTRL_STAT_AGC_GAIN_IDX_STORE_MASK;
    temp |= XCVR_RX_DIG_AGC_CTRL_STAT_AGC_GAIN_IDX_STORE(3); /* 0b11..Store AGC gain index when AA matched */
    XCVR_RX_DIG->AGC_CTRL_STAT = temp;
}

static inline void lcl_hal_xcvr_setup_rssi(BLE_HADM_rttPhyMode_t rttPhy)
{
    uint32 temp;
    uint32_t win_n = 4U + (uint32_t)rttPhy;
    
    xcvr_settings.xcvr_rx_dig_rssi_global_ctrl = XCVR_RX_DIG->RSSI_GLOBAL_CTRL;
    /* WB and NB RSSI are enabled by default XCVR settings in XCVR_RX_DIG->RSSI_GLOBAL_CTRL */
    /* Configure NB-RSSI to start on rx_dig_en instead of AA match */
    XCVR_RX_DIG->RSSI_GLOBAL_CTRL = xcvr_settings.xcvr_rx_dig_rssi_global_ctrl | (XCVR_RX_DIG_RSSI_GLOBAL_CTRL_NB_RSSI_AA_MATCH_OVRD_MASK | XCVR_RX_DIG_RSSI_GLOBAL_CTRL_NB_RSSI_AA_MATCH_OVRD_EN_MASK);
    
    xcvr_settings.xcvr_rx_dig_nb_rssi_ctrl0 = XCVR_RX_DIG->NB_RSSI_CTRL0;
    temp = xcvr_settings.xcvr_rx_dig_nb_rssi_ctrl0;

    /* Configure WT time (= 2^6 in samples) to be 16us for 1Mbps and 8 us for 2Mbps */
    /* Configure RSSI averaging window to be 64 samples (16us) for 1Mbps and 128 samples (16us) for 2Mbps */
    /* RSSI window = 2^M * 2^N samples. N averages in magnitude, M in dB */
    temp &= ~(XCVR_RX_DIG_NB_RSSI_CTRL0_RSSI_IIR_WAIT_NB_MASK | XCVR_RX_DIG_NB_RSSI_CTRL0_RSSI_IIR_WT_NB_MASK |
              XCVR_RX_DIG_NB_RSSI_CTRL0_RSSI_M_WINDOW_NB_MASK | XCVR_RX_DIG_NB_RSSI_CTRL0_RSSI_N_WINDOW_NB_MASK);
    temp |= XCVR_RX_DIG_NB_RSSI_CTRL0_RSSI_IIR_WAIT_NB(6) |
            XCVR_RX_DIG_NB_RSSI_CTRL0_RSSI_IIR_WT_NB(1) |
            XCVR_RX_DIG_NB_RSSI_CTRL0_RSSI_N_WINDOW_NB(win_n) |
            XCVR_RX_DIG_NB_RSSI_CTRL0_RSSI_M_WINDOW_NB(2);
    XCVR_RX_DIG->NB_RSSI_CTRL0 = temp;
}

/* Make the RSSI measurement continuous (useful for first mode 0 on reflector side) or not */
void lcl_hal_xcvr_setup_rssi_continuous(bool_t continuous)
{
    if (continuous)
    {
        XCVR_RX_DIG->RSSI_GLOBAL_CTRL |= (XCVR_RX_DIG_RSSI_GLOBAL_CTRL_NB_CONT_MEAS_OVRD_EN_MASK | XCVR_RX_DIG_RSSI_GLOBAL_CTRL_NB_CONT_MEAS_OVRD_MASK);
    }
    else
    {
        XCVR_RX_DIG->RSSI_GLOBAL_CTRL &= ~(XCVR_RX_DIG_RSSI_GLOBAL_CTRL_NB_CONT_MEAS_OVRD_EN_MASK | XCVR_RX_DIG_RSSI_GLOBAL_CTRL_NB_CONT_MEAS_OVRD_MASK);
    }
}

void lcl_hal_xcvr_set_rxgain(uint8_t man_agc_idx)
{
    if(man_agc_idx <= LCL_HAL_XCVR_AGC_INDEX_MAX)
    {
        /* Use AGC manual index */
        XCVR_RX_DIG->AGC_OVRD &= ~(XCVR_RX_DIG_AGC_OVRD_AGC_GAIN_IDX_OVRD_MASK);
        XCVR_RX_DIG->AGC_OVRD |= (XCVR_RX_DIG_AGC_OVRD_AGC_GAIN_IDX_OVRD_EN_MASK | XCVR_RX_DIG_AGC_OVRD_AGC_GAIN_IDX_OVRD(man_agc_idx)); 
    }
    /* else : Invalid index, remain in automatic mode */
}

void lcl_hal_xcvr_set_tsm_state(xcvr_state_t state)
{
    switch(state)
    {
    default:
        XCVR_TSM->CTRL &= ~(XCVR_TSM_CTRL_FORCE_RX_EN_MASK | XCVR_TSM_CTRL_FORCE_TX_EN_MASK);
        break;
    case eXcvrTx:
        XCVR_TSM->CTRL |= XCVR_TSM_CTRL_FORCE_TX_EN(1);
        break;
    case eXcvrRx:
        XCVR_TSM->CTRL |= XCVR_TSM_CTRL_FORCE_RX_EN(1);
        break;
    }
}

uint8_t lcl_hal_xcvr_get_rxgain(void)
{
    return (uint8_t)((XCVR_RX_DIG->AGC_CTRL_STAT & XCVR_RX_DIG_AGC_CTRL_STAT_AGC_PREV_GAIN_IDX_MASK) >> XCVR_RX_DIG_AGC_CTRL_STAT_AGC_PREV_GAIN_IDX_SHIFT);
}

/*
 * WB-RSSI.
 */
inline int8_t lcl_hal_xcvr_get_wb_rssi(void)
{
    int8_t rssi8bits;
    uint16_t rssi_result = (uint16_t)(XCVR_RX_DIG->WB_RSSI_RES0 & XCVR_RX_DIG_WB_RSSI_RES0_RSSI_WB_MASK) >> XCVR_RX_DIG_WB_RSSI_RES0_RSSI_WB_SHIFT;
    LCL_HAL_CLIP_RSSI(rssi_result, rssi8bits)
    return (rssi8bits); 
}

/*
 * NB-RSSI.
 */
int8_t lcl_hal_xcvr_get_nb_rssi(void)
{
    int8_t rssi8bits;
    uint16_t rssi_result = (uint16_t)(XCVR_RX_DIG->NB_RSSI_RES0 & XCVR_RX_DIG_NB_RSSI_RES0_RSSI_NB_MASK) >> XCVR_RX_DIG_NB_RSSI_RES0_RSSI_NB_SHIFT;
    LCL_HAL_CLIP_RSSI(rssi_result, rssi8bits)
    return (rssi8bits);  
}

/*
 * Returns the magnitude of the signal relative to the ADC scale (in dB)
 */
int8_t lcl_hal_xcvr_get_rssi_raw(void)
{
    return (int8_t)((XCVR_RX_DIG->WB_RSSI_RES0 & XCVR_RX_DIG_WB_RSSI_RES0_RSSI_RAW_WB_MASK) >> XCVR_RX_DIG_WB_RSSI_RES0_RSSI_RAW_WB_SHIFT);
}

/* Returns cfo estimate (done by preamble detect); unit is 1953Hz */
/* Deprecated function: legacy CFO estimate */
inline int8_t lcl_hal_xcvr_get_cfo(void)
{
    /* field unit is 3906Hz */
    assert(0); 
    return (int8_t)((XCVR_2P4GHZ_PHY->STAT0 & GEN4PHY_STAT0_CFO_EST_MASK) >> (GEN4PHY_STAT0_CFO_EST_SHIFT - 1));
}

/*
 * TX timestamp is relative to tx_dig_en and RX timestamp relative to aa_match (network address match)
 * the difference can be compensated. It consists of the following components:
 * preamble padding, packet preamble, network address, frontend delay (board delays, PA delay, RX chain, TX chain)
 */
uint16_t lcl_hal_xcvr_calc_aa_delay(bool_t warmup)
{
    uint32_t delay;
    /* Indicates the TX warmup time, including PA ramp time */
    delay = 0x70; // (GENFSK->WARMUP_TIME & GENFSK_WARMUP_TIME_TX_WARMUP_MASK) >> GENFSK_WARMUP_TIME_TX_WARMUP_SHIFT;
    /* either start from the moment the sequence is initiated or from tx_dig_en */
    if(!warmup)
    {
        delay -= ((XCVR_TSM->TIMING35 & XCVR_TSM_TIMING14_TX_DIG_EN_TX_LO_MASK) >> XCVR_TSM_TIMING14_TX_DIG_EN_TX_LO_SHIFT);
    }
    delay += ((XCVR_TSM->WU_LATENCY & XCVR_TSM_WU_LATENCY_TX_DATAPATH_LATENCY_MASK) >> XCVR_TSM_WU_LATENCY_TX_DATAPATH_LATENCY_SHIFT);
    delay += (8 * (4 /* AA: PN 32 bits*/ + 1 /* preamble 1Mbps*/));

    return delay;
}

void lcl_hal_xcvr_iq_capture_ctrl(uint8_t page)
{
    XCVR_MISC->DBG_RAM_CTRL &= ~XCVR_MISC_DBG_RAM_CTRL_DBG_PAGE_MASK;
    XCVR_MISC->DBG_RAM_CTRL |= XCVR_MISC_DBG_RAM_CTRL_DBG_PAGE(page);
}

uint32_t lcl_hal_xcvr_get_pll_lock_error_flags(void)
{
    uint32_t lock_detect = XCVR_PLL_DIG->LOCK_DETECT;
    uint32_t flags = 0;
    
    if(lock_detect & XCVR_PLL_DIG_LOCK_DETECT_CTFF_MASK)
        flags |= LCL_HAL_XCVR_PLL_CTFF_MASK;
    if(lock_detect & XCVR_PLL_DIG_LOCK_DETECT_FTFF_MASK)
        flags |= LCL_HAL_XCVR_PLL_FTFF_MASK;
    
    if (flags != 0)
    {
        XCVR_PLL_DIG->LOCK_DETECT = lock_detect; /* Clear W1C error flags */
    }
    return flags;
}

//FIXME: no equivalent found in gen4p5
uint32_t lcl_hal_xcvr_get_rx_hazard_flags(void)
{
    return 0;
}

/* ******************************* ToF ************************************/
void lcl_hal_xcvr_tof_setup_tpm_mux(void)
{
    uint32_t tmp;
    
    /* Connect tof_timestamp_trig signal from radio to TPM2_CH0 (set as source) */
    /* Enable XTAL clock as source */
    tmp = RF_CMC1_TPM2_CFG_CLK_MUX_SEL(2) | RF_CMC1_TPM2_CFG_CH0_MUX_SEL(1U) | RF_CMC1_TPM2_CFG_CGC(1);
    RF_CMC1->TPM2_CFG = tmp;
}

int8_t lcl_hal_xcvr_tof_get_frac(void)
{
    int8_t frac = (int8_t)((XCVR_2P4GHZ_PHY->STAT0 & GEN4PHY_STAT0_FRAC_MASK) >> (GEN4PHY_STAT0_FRAC_SHIFT - 2));
    /* divide by 4 to compensate for signed alignment */
    frac /= 4;
    return frac;
}

/*
 * Delay (in ns) per AGC index (0 to 11).
 * Values are rounded to nearest Q3.
 * For example: for index 6, approximated AGC group delay is 2/2^3 = 0.25 ns
 */
// FIXME: AGC group delay characterization is TBD
static const uint8_t tof_agc_grp_delay[LCL_HAL_XCVR_AGC_INDEX_MAX+1] =
{
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
};

uint8_t lcl_hal_xcvr_tof_get_agc_delay(uint8_t agc_idx)
{
    return tof_agc_grp_delay[agc_idx];
}

/* ******************************* MCIQ ************************************/

void lcl_hal_xcvr_hadm_backup(void)
{
    /* Backup TSM registers modified during ranging for later restoration */
    (void)XCVR_LCL_GetTsmTimings(&xcvr_settings.tsm_regs_backup);
}

void lcl_hal_xcvr_hadm_init(const BLE_HADM_SubeventConfig_t *hadm_config, hadm_dc_resid_t *dc_res)
{
    xcvrLclStatus_t status;
    
    NbuPwrPeakReductionActivityStart();
    
    /* First, backup registers touched by custom DCOC setting and calibration */
    xcvr_settings.xcvr_rx_dig_ctrl_1 = XCVR_RX_DIG->CTRL1;
    xcvr_settings.xcvr_rx_dig_dcoc_ctrl_0 = XCVR_RX_DIG->DCOC_CTRL0;
    xcvr_settings.xcvr_rx_dig_dcoc_ctrl_2 = XCVR_RX_DIG->DCOC_CTRL2;
    xcvr_settings.xcvr_radio_ctrl_rf_ctrl = RADIO_CTRL->RF_CTRL;
    
    assert(dcoc_ctrl_2[hadm_config->rttPhy] != 0);
    XCVR_LCL_OverrideDcoc(dcoc_ctrl_2[hadm_config->rttPhy], true);
    
    if ((hadm_config->typeFlags & HADM_SUBEVT_FIRST) && (hadm_config->phaseCont == HADM_PHASE_CONT_OVERRIDES))
    {
        /* This is the first subevent of a procedure */
        XCVR_LCL_ContPhaseOvrd(); /* maintain some dividers during the whole procedure */
        /* Do it before XCVR_LCL_RsmRegBackup() which will backup OVRD registers */
    }
    else if (hadm_config->phaseCont == HADM_PHASE_CONT_LOOPBACK)
    {     
        /* Maintain some dividers during the CS event */
        XCVR_LCL_ContPhaseOvrd();
        /* Enable TX leakage */
        XCVR_LCL_EnablePhaseMeasure();
        /* Start loopback phase measurement */
        /* DCOC calibration should have been done at this point, and DCOC overrides configured */
        XCVR_LCL_CalibrateDcocStart((XCVR_RSM_SQTE_RATE_T)hadm_config->rttPhy);
    }
    
    /* Backup XCVR registers modified during ranging for later restoration */
    (void)XCVR_LCL_RsmRegBackup(&xcvr_settings.xcvr_regs_backup);
    
    lcl_hal_xcvr_setup_agc();
    lcl_hal_xcvr_setup_rssi(hadm_config->rttPhy);
    lcl_hal_xcvr_setup_rssi_continuous(TRUE);
    
    XCVR_LCL_RsmCompCfo(0); /* make sure PLL_OFFSET_CTRL is cleared before first mode 0 */
    
    XCVR_PLL_DIG->LOCK_DETECT |= 0xFF; // clear sticky error flags
    
    if (hadm_config->phaseCont == HADM_PHASE_CONT_LOOPBACK)
    {
        /* wait for DCOC to complete */
        status = XCVR_LCL_CalibrateDcocComplete();
        /* and retreive loopback phase meas results */
        uint32_t temp_resid = XCVR_RX_DIG->DCOC_DIG_CORR_RESULT;
        /* Read and strore DC residual for I and Q */
        status += XCVR_LCL_ProcessPhaseMeasure(&dc_res->i_resid, &dc_res->q_resid, temp_resid);
        assert(gXcvrLclStatusSuccess == status);
        (void)status;
    }
}

void lcl_hal_xcvr_hadm_deinit(const BLE_HADM_SubeventConfig_t *hadm_config)
{
    /* Restore all TSM/XCVR modified registers */
    (void)XCVR_LCL_RsmRegRestore(&xcvr_settings.xcvr_regs_backup);
    (void)XCVR_LCL_ReprogramTsmTimings(&xcvr_settings.tsm_regs_backup);
        
    XCVR_RX_DIG->CTRL1 = xcvr_settings.xcvr_rx_dig_ctrl_1;
    XCVR_RX_DIG->DCOC_CTRL0 = xcvr_settings.xcvr_rx_dig_dcoc_ctrl_0;
    XCVR_RX_DIG->DCOC_CTRL2 = xcvr_settings.xcvr_rx_dig_dcoc_ctrl_2;
    RADIO_CTRL->RF_CTRL = xcvr_settings.xcvr_radio_ctrl_rf_ctrl;
    
    XCVR_RX_DIG->RSSI_GLOBAL_CTRL = xcvr_settings.xcvr_rx_dig_rssi_global_ctrl;
    XCVR_RX_DIG->NB_RSSI_CTRL0 = xcvr_settings.xcvr_rx_dig_nb_rssi_ctrl0;

    XCVR_RX_DIG->AGC_CTRL = xcvr_settings.xcvr_rx_dig_agc_ctrl;
    XCVR_RX_DIG->AGC_OVRD = xcvr_settings.xcvr_rx_dig_agc_ovrd;
    XCVR_RX_DIG->AGC_CTRL_STAT = xcvr_settings.xcvr_rx_dig_agc_ctrl_stat;
    
    if ((hadm_config->typeFlags & HADM_SUBEVT_LAST) && (hadm_config->phaseCont == HADM_PHASE_CONT_OVERRIDES))
    {
        XCVR_LCL_AllPhaseRelease(); /* release dividers overrides at the end of CS procedure */
    }
    else if (hadm_config->phaseCont == HADM_PHASE_CONT_LOOPBACK)
    {       
        XCVR_LCL_AllPhaseRelease(); /* release dividers overrides at the end of CS event */
    }
    
    NbuPwrPeakReductionActivityStop();
}

uint16_t lcl_hal_xcvr_mciq_get_agc_status(void)
{
    /* AGC_CTRL_STAT can be read only if rx-dig-en=1 */
    uint16_t flags = 0;
    uint32_t agc_stat = (XCVR_RX_DIG->AGC_CTRL_STAT & XCVR_RX_DIG_AGC_CTRL_STAT_AGC_STATUS_MASK) >> XCVR_RX_DIG_AGC_CTRL_STAT_AGC_STATUS_SHIFT;
    
    if (agc_stat == 6)
        flags |= LCL_HAL_XCVR_AGC_FROZEN;

    return flags;
}

void lcl_hal_xcvr_start_dbg_ram_capture(t_hadm_dma_page_t page, t_hadm_trigger_t start_trigger, uint32_t nb_bytes)
{
    /* stop ongoing capture if any */
    XCVR_MISC->DBG_RAM_CTRL = 0;
    /* set last word to be written in TX_PACKET_RAM */
    XCVR_MISC->DBG_RAM_ADDR = XCVR_MISC_DBG_RAM_ADDR_DBG_RAM_LAST((nb_bytes-1) & (~0x3));
    /* Start DBG RAM with given trigger, raising edge */
    XCVR_MISC->DBG_RAM_CTRL = XCVR_MISC_DBG_RAM_CTRL_DBG_EN_MASK | XCVR_MISC_DBG_RAM_CTRL_DBG_START_TRG(start_trigger) | XCVR_MISC_DBG_RAM_CTRL_DBG_START_EDGE(0);
    XCVR_MISC->DBG_RAM_CTRL |= XCVR_MISC_DBG_RAM_CTRL_DBG_PAGE(page);
}

bool_t lcl_hal_xcvr_is_dbg_ram_capture_finished(void)
{
    return ((XCVR_MISC->DBG_RAM_CTRL & XCVR_MISC_DBG_RAM_CTRL_DBG_RAM_FULL_MASK) == XCVR_MISC_DBG_RAM_CTRL_DBG_RAM_FULL_MASK);
}

void lcl_hal_xcvr_configure_dma_capture(t_hadm_trigger_t start_trigger, uint32_t delay, uint32_t nb_words, uint16 m_hadmbuffer_size, uint32 m_hadmbuffer_start)
{
  if(m_hadmbuffer_start != NULL)
  {
    /* Initialize DSB */
    MRCC->MRCC_DATA_STREAM_2P4 = MRCC_MRCC_DATA_STREAM_2P4_RSTB_MASK | MRCC_MRCC_DATA_STREAM_2P4_CC(1U);
    /* Put DSB into reset to ensure clean state */
    DSB0->CSR |= DSB_CSR_SFTRST_MASK;
    /* Release DSB from reset */
    DSB0->CSR &= ~DSB_CSR_SFTRST_MASK;
    /* stop ongoing capture if any */
    XCVR_MISC->DMA_CTRL = 0;
    /* Configure DSB */
    DSB0->CSR |= (DSB_CSR_DSB_EN_MASK | DSB_CSR_DMA_EN_MASK | DSB_CSR_ERR_EN_MASK);
    DSB0->WMC = DSB_WMC_WMRK(1);
    DSB0->DADDR = DSB_DADDR_DADDR(m_hadmbuffer_start); /* destination buffer from CM33 system RAM - see m_hadmbuff_start in wireless_ranging_measuement.c */
    if (nb_words > m_hadmbuffer_size) nb_words = m_hadmbuffer_size;  /* destination buffer size CM33 system RAM - see m_hadmbuffer_size in wireless_ranging_measuement.c */
    DSB0->XCR = DSB_XCR_TCNT(nb_words);

    /* Start DMA with given trigger, raising edge */
    XCVR_MISC->DMA_CTRL = XCVR_MISC_DMA_CTRL_DMA_EN_MASK | 
                          XCVR_MISC_DMA_CTRL_DMA_START_TRG(start_trigger) | 
                          XCVR_MISC_DMA_CTRL_DMA_START_EDGE(0) |
                          XCVR_MISC_DMA_CTRL_DMA_START_DLY(delay);
  }
  else
  {
      assert(0);
  }

}

bool_t lcl_hal_xcvr_is_dma_capture_finished(void)
{
    return ((DSB0->INT & DSB_INT_DONE_MASK) == DSB_INT_DONE_MASK);
    /* no need to check for other status bits, they are never set by the HW, see KFOURWONE-755 */
}



#define T_CAPTURE_DELAY (17U) /* accounts for RX warmup delay + data path latency. Measured. */
#define T_TX_ANT_SW_DELAY (2U) /* to align lant_sw signals on RX and TX side. RX TX_PM state starts 2us earlier (TX_DATA_FLUSH_DLY). */

void lcl_hal_xcvr_configure_ant_switch(uint32_t n_ap, uint8_t t_pm, uint8_t t_sw, uint8_t t_capture, uint8_t rttPhy)
{ 
    uint32_t spint_us, temp, offset, sampling_rate_factor;
    
    /* Configure 2Mbps oversampling if needed */
    if (rttPhy == HADM_RTT_PHY_1MBPS)
        sampling_rate_factor = 1U;
    else
        sampling_rate_factor = SAMPLING_RATE_FACTOR_2MBPS;
    
    /* Configure SPINT according to T_PM duration in order not to overflow SPINT and HI/LO_PER 5bits values */
    if ((t_pm == HADM_T_PM_20) || (t_pm == HADM_T_PM_40))
        spint_us = 2;
    else
        spint_us = 1;

    /* RX/TX config */
    offset = (T_TX_ANT_SW_DELAY / spint_us);
    temp = XCVR_MISC_LCL_TX_CFG0_TX_DELAY(offset);
    offset = (T_TX_ANT_SW_DELAY % spint_us);
    temp |= XCVR_MISC_LCL_TX_CFG0_TX_DELAY_OFF(offset*TX_SAMPLING_RATE*sampling_rate_factor);
    
    XCVR_MISC->LCL_RX_CFG0 = 0; /* trigger delay = 0 */
    XCVR_MISC->LCL_TX_CFG0 = temp; /* trigger delay = T_TX_ANT_SW_DELAY */

    assert(((t_capture / spint_us) + ((t_pm + t_sw - t_capture) / spint_us)) * spint_us == (t_sw + t_pm)); /* ensure divisions do not have remainder */

    temp = XCVR_MISC_LCL_RX_CFG1_RX_ANT_TRIG_SEL(6) /* RSM trigger */ |
        XCVR_MISC_LCL_RX_CFG1_RX_HI_PER(t_capture / spint_us /* HI_PER: dma mask enable duration */) | 
        XCVR_MISC_LCL_RX_CFG1_RX_LO_PER((t_pm + t_sw - t_capture) / spint_us); /* LO_PER */

    /* TX and RX configs have same register mapping. SPINT is different due to sampling rate */
    XCVR_MISC->LCL_RX_CFG1 = temp | XCVR_MISC_LCL_RX_CFG1_RX_SPINT(RX_SAMPLING_RATE*sampling_rate_factor*spint_us-1U);
    XCVR_MISC->LCL_TX_CFG1 = temp | XCVR_MISC_LCL_TX_CFG1_TX_SPINT(TX_SAMPLING_RATE*sampling_rate_factor*spint_us-1U);

    /* Configure TX+RX mode, and duration. All other fields are reset value (0) */
    XCVR_MISC->LCL_CFG0 = XCVR_MISC_LCL_CFG0_CTE_DUR(n_ap) | XCVR_MISC_LCL_CFG0_RX_LCL_EN_MASK | XCVR_MISC_LCL_CFG0_TX_LCL_EN_MASK; /* account for T_PM ext */
    
    /* workaround for KFOURWONE-702 */
    offset = (T_CAPTURE_DELAY + t_pm + t_sw - t_capture ) / (2U * spint_us);
    temp = XCVR_MISC_LCL_DMA_MASK_DELAY_DMA_MASK_DELAY(offset); /* LO_PER/2 to get dma_mask centered inside T_PM */
    offset = (T_CAPTURE_DELAY + t_pm + t_sw - t_capture) % (2U * spint_us);
    temp |= XCVR_MISC_LCL_DMA_MASK_DELAY_DMA_MASK_DELAY_OFF(offset*RX_SAMPLING_RATE*sampling_rate_factor);
    XCVR_MISC->LCL_DMA_MASK_DELAY = temp;
    XCVR_MISC->LCL_DMA_MASK_PERIOD = XCVR_MISC_LCL_DMA_MASK_PERIOD_DMA_MASK_REF_PER(t_capture / spint_us); /* == HI_PER */
}

void lcl_hal_xcvr_configure_t_pm_ext(uint32_t n_ap, bool_t enable_tx_in_tpm_ext, uint8_t dummy_ant)
{
    uint32_t temp = XCVR_MISC->LCL_GPIO_CTRL0;
    n_ap <<= 2; /* 4*n_ap */
    temp &= ~(0xFU << n_ap);
    if (enable_tx_in_tpm_ext)
    {
        temp |= ((temp & (0xFU << (n_ap - 4U))) << 4); /* reuse N_AP-1 antenna */
    }
    else
    {
        temp |= (dummy_ant << n_ap); /* use dummy antenna */
    }
    XCVR_MISC->LCL_GPIO_CTRL0 = temp;
}

void lcl_hal_xcvr_configure_ant_switch_lut(uint32_t n_ap, uint8_t *toneAntennaIDs_p)
{
    uint32_t i, temp;
    
    /* this assumes no more than 4+1 AP can be configured */
    XCVR_MISC->LCL_GPIO_CTRL4 = XCVR_MISC_LCL_GPIO_CTRL4_LUT_WRAP_PTR(n_ap); /* account for T_PM ext */
    
    temp = toneAntennaIDs_p[0];
    for (i=1; i<n_ap; i++)
    {
        temp |= (toneAntennaIDs_p[i] << (4 * i));
    }
    XCVR_MISC->LCL_GPIO_CTRL0 = temp;
}

void lcl_hal_xcvr_pll_settings_backup(void)
{
    (void)XCVR_LCL_RsmPLLBackup(&xcvr_settings.xcvr_regs_backup);
}

void lcl_hal_xcvr_pll_settings_restore(void)
{
    (void)XCVR_LCL_RsmPLLRestore(&xcvr_settings.xcvr_regs_backup);
}

/* Shift RSM time grid by playing with TX delay */
/* If time_shift > 0, time grid is shrinked because TX delay is locally increased */
/* And vice-versa */
void lcl_hal_xcvr_apply_time_grid_shift(int32_t time_shift)
{
    uint32_t tx_delay, temp;
    
    temp = XCVR_TX_DIG->DATA_PADDING_CTRL_1;
    
    tx_delay = (temp & XCVR_TX_DIG_DATA_PADDING_CTRL_1_TX_DATA_FLUSH_DLY_MASK) >> XCVR_TX_DIG_DATA_PADDING_CTRL_1_TX_DATA_FLUSH_DLY_SHIFT;
    tx_delay += time_shift;
    
    temp &= ~XCVR_TX_DIG_DATA_PADDING_CTRL_1_TX_DATA_FLUSH_DLY_MASK;
    temp |= XCVR_TX_DIG_DATA_PADDING_CTRL_1_TX_DATA_FLUSH_DLY(tx_delay);

    XCVR_TX_DIG->DATA_PADDING_CTRL_1 = temp;
}

#ifdef RSM_WA_FOR_RX_SETLLING_LATENCY_KFOURWONE_2175
uint32_t t_fc_rsm;

void lcl_hal_xcvr_increase_rx_settling_latency(void)
{
    /* Set T_S = 15us, T_FCS-=5us */
    t_fc_rsm = (XCVR_MISC->RSM_CTRL1 & XCVR_MISC_RSM_CTRL1_RSM_T_FC_MASK) >> XCVR_MISC_RSM_CTRL1_RSM_T_FC_SHIFT;
    XCVR_MISC->RSM_CTRL1 &= ~(XCVR_MISC_RSM_CTRL1_RSM_T_FC_MASK | XCVR_MISC_RSM_CTRL1_RSM_T_S_MASK);
    XCVR_MISC->RSM_CTRL1 |= (XCVR_MISC_RSM_CTRL1_RSM_T_S(3) | XCVR_MISC_RSM_CTRL1_RSM_T_FC(t_fc_rsm-1));
    /* Set RX_SETTLING_LATENCY = 11us */
    XCVR_TSM->WU_LATENCY = XCVR_TSM_WU_LATENCY_RX_SETTLING_LATENCY(0xB);
}

void lcl_hal_xcvr_restore_rsm_t_fc(void)
{
    XCVR_MISC->RSM_CTRL1 &= ~XCVR_MISC_RSM_CTRL1_RSM_T_FC_MASK;
    XCVR_MISC->RSM_CTRL1 |= XCVR_MISC_RSM_CTRL1_RSM_T_FC(t_fc_rsm);
}
#endif

/* EOF */
