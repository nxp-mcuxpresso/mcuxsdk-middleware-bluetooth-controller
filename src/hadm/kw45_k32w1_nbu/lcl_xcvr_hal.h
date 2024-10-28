/**
 * @file lcl_xcvr_hal_.h
 *
 * This file implements LCL XCVR HAL
 */
/*
 * Copyright 2021-2024 NXP
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* Prevent double inclusion */
#ifndef _LCL_XCVR_HAL_H_
#define _LCL_XCVR_HAL_H_

/* === Includes ============================================================ */
#include "nxp_xcvr_lcl_ctrl.h"

/* === Types =============================================================== */
typedef enum xcvr_state_tag {
    eXcvrIdle = 0,
    eXcvrTx,
    eXcvrRx
} xcvr_state_t;

typedef enum _GENFSK_commands_tag
{
    NO_ACTION = 0U,  /*!< No action. */
    TX_START_NOW = 1U,  /*!< TX start now. */
    TX_START_T1 = 2U, /*!< TX start @T1 Timer Compare Match (EVENT_TMR = T1_CMP). */
    TX_START_T2 = 3U,  /*!< TX start @T2 Timer Compare Match (EVENT_TMR = T2_CMP). */
    TX_CANCEL = 4U,  /*!< TX cancel -- Cancels pending TX events but do not abort a TX-in-progress. */
    RX_START_NOW = 5U,  /*!< RX start now. */
    RX_START_T1 = 6U,  /*!< RX start @T1 Timer Compare Match (EVENT_TMR = T1_CMP). */
    RX_START_T2 = 7U,  /*!< RX start @T2 Timer Compare Match (EVENT_TMR = T2_CMP). */
    RX_STOP_T1 = 8U,  /*!< RX stop @T1 Timer Compare Match (EVENT_TMR = T1_CMP). */
    RX_STOP_T2 = 9U,  /*!< RX stop @T2 Timer Compare Match (EVENT_TMR = T2_CMP). */
    RX_CANCEL = 10U,  /*!< RX cancel -- Cancels pending RX events but do not abort a RX-in-progress. */
    ABORT_ALL = 11U,  /*!< Abort all -- Cancels all pending events and abort any sequence-in-progress. */
} GENFSK_commands_t;

/*! RSM_STATE - RSM_STATE */
typedef enum
{
    RSM_IDLE    = 0,    /* IDLE */
    RSM_DELAY,          /* DELAY. Used only for the trigger delay in SQTE */
    RSM_WU      = 4,    /* 0b00100..WU (Warmup). Used only for SQTE */
    RSM_DT_TX,          /* 0b00101..DT_TX (Packet TX). Used only for SQTE */
    RSM_DT_RX,          /* 0b00110..DT_RX (Packet RX). Used only for SQTE */
    RSM_DT_RX_SYN,      /* DT_RX_SYNC (Packet RX Sync). Used only for SQTE */
    RSM_FM_TX,          /* FM_TX (Frequency Measurement TX). Used only for SQTE */
    RSM_FM_RX,          /* FM_RX (Frequency Measurement RX). Used only for SQTE */
    RSM_PM_TX,          /* PM_TX (Phase Measurement TX). */
    RSM_PM_RX,          /* PM_RX (Phase Measurement RX). */
    RSM_IP1_RX2TX,      /* IP1_RX2TX (Interlude Period 1 RX2TX). Used only for SQTE */
    RSM_IP1_TX2RX,      /* IP1_TX2RX (Interlude Period 1 TX2RX). Used only for SQTE */
    RSM_S_RX2RX,        /* S_RX2RX (Short Period RX2RX). Used only for SQTE */
    RSM_S_TX2TX,        /* S_TX2TX (Short Period TX2TX). Used only for SQTE*/
    RSM_IP2_RX2TX,      /* IP2_RX2TX (Interlude Period 2 RX2TX). */
    RSM_IP2_TX2RX,      /* IP2_TX2RX (Interlude Period 2 TX2RX). */
    RSM_FC_RX2TX,       /* 0b10010..FC_RX2TX (Frequency Change RX2TX).*/
    RSM_FC_TX2RX,       /* 0b10011..FC_TX2RX (Frequency Change TX2RX).*/
    RSM_WD              /* 0b10100..WD (Warmdown) */
} t_hadm_rsm_states;

/*! DMA/DBG_RAM page selections. */
typedef enum
{
    LCL_DMA_PAGE_IDLE = 0,       /*!< IDLE page, used to stop DMA capture. */
    LCL_DMA_PAGE_RXDIGIQ,        /*!< RXDIG I and Q sample capture. Data is MSB aligned in 16bit portions of the 32bit word. */
    LCL_DMA_PAGE_RXDIGIQ_ALT,    /*!< RXDIG I and Q sample capture plus additional signals in unused bits. Data is MSB aligned in 16bit portions of the 32bit word. */
    LCL_DMA_PAGE_RAWADCIQ,       /*!< Raw ADC capture of I and Q channel. Newest bit is in the MS bit of each byte, and newest I/Q samples are in the upper half-word.*/
    LCL_DMA_PAGE_PHASE,          /*!< Wide-band phase, narrowband phase, or high-precision phase (as selected in RXDIG). Newest sample in most significant byte. Each byte contains a MSB-aligned phase sample.*/
    LCL_DMA_PAGE_RSSI_PHASE,     /*!< 8bit RSSI with 8bit high-resolution phase. Newest sample in most significant half-word.*/
    LCL_DMA_PAGE_MAG_PHASE,      /*!< RSSI magnitude with 8bit high-resolution phase.*/
    LCL_DMA_PAGE_PHY,            /*!< As selected via programming of the PHY, various signals can be output on this 32bit bus for DMA capture. */
} t_hadm_dma_page_t;

/*! DMA/DBG_RAM Start Trigger selections. */
typedef enum
{
    LCL_NO_DMA_START_TRIG = 0,                  /*!< Immediate Start Trigger, sample capture starts as soon as DMA Page is non-zero.  */
    LCL_START_DMA_ON_FSK_PREAMBLE_FOUND = 1,    /*!< Preamble Found Start Trigger.  */
    LCL_START_DMA_ON_FSK_AA_MATCH = 2,          /*!< Access Address Match Start Trigger.  */          
    LCL_START_DMA_ON_ZBDEMOD_PREAMBLE_FOUND = 3,/*!< ZBDEMOD Preamble Found Start Trigger.  */
    LCL_START_DMA_ON_ZBDEMOD_SFD_MATCH = 4,     /*!< ZBDEMOD SFD Match Start Trigger.  */
    LCL_START_DMA_ON_AGC_DCOC_GAIN_CHG = 5,     /*!< AGC DCOC gain change. */
    LCL_START_DMA_ON_TSM_RX_DIG_EN = 6,         /*!< RX Digital Enable Start Trigger (TSM signal when RX warmup starts).  */
    LCL_START_DMA_ON_TSM_SPARE2_EN = 7,         /*!< TSM SPARE2 Start Trigger (based on custom TSM timing signal for TSM_IRQ0). */
    LCL_START_DMA_ON_RBME_CRC_PASS = 8,         /*!< CRC Passing Start Trigger (asserted when CRC result is passing). */
    LCL_START_DMA_ON_RBME_CRC_CMPLT = 9,        /*!< CRC Processing Complete Start Trigger (asserted when CRC processing completes, no matter what the result). */
    LCL_START_DMA_ON_LCL_PATT_MATCH = 10,       /*!< Localization Pattern Match Start Trigger. */
    LCL_START_DMA_ON_GENLL_CTE_PRESENT = 11,    /*!< CTE Tone Present Start Trigger.  */
    LCL_START_DMA_ON_RSM_TRIGGER = 12,      /*!< RSM Module Triggers DMA capture.  */
} t_hadm_trigger_t;

/*! IQ capture output selections. */
typedef enum
{
    LCL_OUT_DISABLED_SEL        = 0,           /*!< Disabled output */
    LCL_OUT_IF_MIXER_SEL        = 1,           /*!< IF mixer I/Q output */
    LCL_OUT_CIC_SEL             = 2,                /*!< CIC output */
    LCL_OUT_CH_FILTER_SEL       = 3,          /*!< Channel filter I/Q output */
    LCL_OUT_SRC_SEL             = 4,                /*!< SRC I/Q output */
    LCL_OUT_CFO_MIXER_SEL       = 5,          /*!< CFO Mixer I/Q output */
    LCL_OUT_FRAC_CORR_SEL       = 6,          /*!< Frac Corr I/Q output */
    LCL_OUT_DC_RESID_IQ_CORR    = 7,       /*!< DC Residual IQ correction output */
} t_hadm_rsm_iq_output_sel_t;

/* === Macros ============================================================== */
#define RSM_DEBUG
//#define RSM_DEBUG_CAL
//#define RSM_DEBUG_DMA
#define RTT_DEBUG
//#define MCIQ_DEBUG


//#define RSM_WA_FOR_RX_SETLLING_LATENCY_KFOURWONE_2175

#define LCL_HAL_XCVR_IQ_SAMPLE_SIZE         (4U)    /* One IQ sample from XCVR is 4 bytes */
#define LCL_HAL_XCVR_PHASE_SAMPLE_SIZE      (1U)    /* One phase sample from XCVR is 1 bytes */
#define LCL_HAL_XCVR_PHASE_MAG_SAMPLE_SIZE  (4U)    /* One phase / magnitude sample from XCVR is 4 bytes */

/* XCVR AGC limit */
#define LCL_HAL_XCVR_AGC_INDEX_MAX      (11U)

/* AGC status flags */
#define LCL_HAL_XCVR_AGC_FROZEN         (0x1U)
#define LCL_HAL_XCVR_AGC_PEAK_DETECT    (0x2U)

/* PLL lock error flags */
#define LCL_HAL_XCVR_PLL_CTFF_MASK      (0x1U)  /* Coarse Tune Failure Flag */
#define LCL_HAL_XCVR_PLL_CSFF_MASK      (0x2U)  /* Cycle Slip Failure Flag */
#define LCL_HAL_XCVR_PLL_FTFF_MASK      (0x4U)  /* Frequency Target Failure Flag */

/* RX error flags */
#define LCL_HAL_XCVR_RX_HAZARD          (0x1U)

/* IRQ flags */
#define LCL_HAL_XCVR_RSM_IRQ_IP         ((XCVR_MISC_RSM_CSR_RSM_IRQ_IP1_MASK | XCVR_MISC_RSM_CSR_RSM_IRQ_IP2_MASK))
#define LCL_HAL_XCVR_RSM_IRQ_FC         (XCVR_MISC_RSM_CSR_RSM_IRQ_FC_MASK)
#define LCL_HAL_XCVR_RSM_IRQ_EOS        (XCVR_MISC_RSM_CSR_RSM_IRQ_EOS_MASK)
#define LCL_HAL_XCVR_RSM_IRQ_ABORT      (XCVR_MISC_RSM_CSR_RSM_IRQ_ABORT_MASK)
#define LCL_HAL_TPM_TIMER_IRQ           (0x400U) /* warning: must not conflict with any RSM IRQ mask */
#define LCL_HAL_TPM_IRQ_MAX             LCL_HAL_TPM_TIMER_IRQ

/* RSM states flags */
#define LCL_HAL_XCVR_RSM_STATE_FC       (0x01U)
#define LCL_HAL_XCVR_RSM_STATE_IDLE     (0x02U)
#define LCL_HAL_XCVR_RSM_STATE_RX_SYNC  (0x03U)

#define LCL_HAL_XCVR_IS_RSM_IDLE        ((XCVR_MISC->RSM_CSR & XCVR_MISC_RSM_CSR_RSM_STATE_MASK) == 0)

/* Config flag for RSM calibration (HPM and CTUNE) */
#define LCL_HAL_XCVR_RSM_CTUNE_CAL_IS_AUTO       (2U)  /* RSM will perform CTUNE calibration automatically */
#define LCL_HAL_XCVR_RSM_HPM_CAL_IS_AUTO         (1U)  /* RSM will perform HPM calibration automatically */

/* RSM helpers */
#define LCL_HAL_RSM_XCVR_IRQ_MASK_FLAGS \
    (XCVR_MISC_RSM_CSR_RSM_IRQ_ABORT_EN_MASK | XCVR_MISC_RSM_CSR_RSM_IRQ_EOS_EN_MASK | XCVR_MISC_RSM_CSR_RSM_IRQ_FC_EN_MASK | \
     XCVR_MISC_RSM_CSR_RSM_IRQ_IP1_EN_MASK | XCVR_MISC_RSM_CSR_RSM_IRQ_IP2_EN_MASK)

#define LCL_HAL_RSM_IRQ_STATUS_FLAGS    (XCVR_MISC_RSM_CSR_RSM_IRQ_ABORT_MASK | XCVR_MISC_RSM_CSR_RSM_IRQ_EOS_MASK | XCVR_MISC_RSM_CSR_RSM_IRQ_FC_MASK  | XCVR_MISC_RSM_CSR_RSM_IRQ_IP1_MASK | XCVR_MISC_RSM_CSR_RSM_IRQ_IP2_MASK)
#define LCL_HAL_RSM_GET_IRQ_STATUS_FLAGS (XCVR_MISC->RSM_CSR & LCL_HAL_RSM_IRQ_STATUS_FLAGS)
#define LCL_HAL_RSM_SET_IRQ_STATUS_FLAGS(flags) (XCVR_MISC->RSM_CSR |= flags)

#define LCL_HAL_RSM_GET_CURRENT_STEP     ((XCVR_MISC->RSM_CSR & XCVR_MISC_RSM_CSR_RSM_CURRENT_STEPS_MASK) >> XCVR_MISC_RSM_CSR_RSM_CURRENT_STEPS_SHIFT)

#define LCL_HAL_ALL_DMA_STATUS_BITS (DSB_INT_DONE_MASK | DSB_INT_DBE_MASK | DSB_INT_UNDR_MASK | DSB_INT_OVRF_MASK | DSB_INT_DRDY_MASK)

/* Following macros allows to disable/enable DMA mask and IQ averaging */
#define LCL_HAL_SET_DMA_MASK_AND_AVG_WIN(win)   (XCVR_RX_DIG->CTRL1 |= (XCVR_RX_DIG_CTRL1_RX_IQ_PH_OUTPUT_COND_MASK | XCVR_RX_DIG_CTRL1_RX_IQ_PH_AVG_WIN(win)))
#define LCL_HAL_CLEAR_DMA_MASK_AND_AVG_WIN      (XCVR_RX_DIG->CTRL1 &= ~(XCVR_RX_DIG_CTRL1_RX_IQ_PH_OUTPUT_COND_MASK | XCVR_RX_DIG_CTRL1_RX_IQ_PH_AVG_WIN_MASK))

/* DMA / DSB control */
#define LCL_HAL_START_DMA(page)     (XCVR_MISC->DMA_CTRL |= XCVR_MISC_DMA_CTRL_DMA_PAGE(page))
#define LCL_HAL_STOP_DMA            (XCVR_MISC->DMA_CTRL &= ~(XCVR_MISC_DMA_CTRL_DMA_PAGE_MASK))
#define LCL_HAL_RESTART_DSB         (DSB0->CSR |= DSB_CSR_DSB_EN_MASK)

#define LCL_HAL_START_LCL           (XCVR_MISC->LCL_CFG0 |= XCVR_MISC_LCL_CFG0_LCL_EN_MASK)
#define LCL_HAL_STOP_LCL            (XCVR_MISC->LCL_CFG0 &= ~XCVR_MISC_LCL_CFG0_LCL_EN_MASK)

#define LCL_HAL_STOP_DBGRAM         (XCVR_MISC->DBG_RAM_CTRL = 0)

#define LCL_HAL_DISABLE_DMA          \
                                     DSB0->CSR |= DSB_CSR_SFTRST_MASK; /* Reset DSB */\
                                     XCVR_MISC->DMA_CTRL = 0 /* Disable DMA */
                                       
#define LCL_HAL_SET_IQ_CAPTURE_POINT(out_sel) \
{ \
    uint32_t temp = XCVR_RX_DIG->DFT_CTRL;\
    temp &= ~XCVR_RX_DIG_DFT_CTRL_DFT_RX_IQ_OUT_SEL_MASK;\
    temp |= XCVR_RX_DIG_DFT_CTRL_DFT_RX_IQ_OUT_SEL(out_sel);\
    XCVR_RX_DIG->DFT_CTRL = temp;\
}

#ifdef RSM_DEBUG
#define LCL_HAL_DISABLE_RX_SYNC  \
    XCVR_MISC->RSM_CTRL3 |= XCVR_MISC_RSM_CTRL3_RSM_RX_PHY_EN_MASK_DIS_MASK; /* For tones observability using LA */
#else
#define LCL_HAL_DISABLE_RX_SYNC
#endif

/* Override GPIO output by writing to LUT[0] */
/* and alternative way might be to change the pinmux to gpio output */
#define LCL_HAL_SET_ANTENNA_PORT(port) (XCVR_MISC->LCL_GPIO_CTRL0 = XCVR_MISC_LCL_GPIO_CTRL0_LUT_0(port))

/* Configure 64 bit PN sequences to HARTT & RSM */
#define LCL_HAL_ENABLE_64_BITS_PN   (XCVR_2P4GHZ_PHY->RTT_CTRL |= GEN4PHY_RTT_CTRL_RTT_SEQ_LEN_MASK)

#define LCL_HAL_BUILD_PKT_RAM_CONFIG_STEP(step_idx, step_mode, fstep_pram_ptr, hpm_cal_val) \
    uint8_t mapped_chan_num = hadm_config->chModePmAntMap[i].channel >> 1U; /* divide by 2 to get the normal BLE channel index for format #2 HOP_TBL_CFG_OVRD */    \
    if ((hadm_config->chModePmAntMap[i].channel & 0x1U) == 0x1U) /* original HADM channel was an odd number */\
    {\
        *fstep_pram_ptr++ = (1U + mapped_chan_num); /* go to next channel up (2MHz higher) to allow -1MHz to hit the target channel */\
        *fstep_pram_ptr++ = (1U <<7U); /* Apply -1MHz */\
    }\
    else\
    {\
        *fstep_pram_ptr++ = mapped_chan_num;\
        *fstep_pram_ptr++ = 0U;\
    }\
    *fstep_pram_ptr++ = 0U;\
    *fstep_pram_ptr++ = (uint8_t)((hpm_cal_val & 0x1FEU)>>1U); /* Bits [8:1] */\
    *fstep_pram_ptr++ = ((XCVR_RSM_HPM_CAL_MSB(((hpm_cal_val & 0x01E00U)>>9U))) | /* Bits [12:9] */ (XCVR_RSM_STEP_FORMAT(step_mode)));

#define LCL_HAL_T_PM_MEAS (630U) /* T_PM_MEAS shall be 650us but RSM supports only 630us max */

/* === Globals ============================================================= */

/* === Externals =========================================================== */

/* === Prototypes ========================================================== */
#ifdef __cplusplus
extern "C" {
#endif

void lcl_hal_xcvr_iq_capture_ctrl(uint8_t page);
uint32_t lcl_hal_xcvr_get_pll_lock_error_flags(void);
uint32_t lcl_hal_xcvr_get_rx_hazard_flags(void);
void lcl_hal_xcvr_set_rxgain(uint8_t man_agc_idx);
void lcl_hal_xcvr_set_tsm_state(xcvr_state_t state);
uint8_t lcl_hal_xcvr_get_rxgain(void);
int8_t lcl_hal_xcvr_get_wb_rssi(void);
int8_t lcl_hal_xcvr_get_nb_rssi(void);
int8_t lcl_hal_xcvr_get_rssi_raw(void);
int8_t lcl_hal_xcvr_get_cfo(void);
uint16_t lcl_hal_xcvr_calc_aa_delay(bool_t warmup);
void lcl_hal_xcvr_setup_rssi_continuous(bool_t continuous);

void lcl_hal_xcvr_tof_setup_tpm_mux(void);
int8_t lcl_hal_xcvr_tof_get_frac(void);
uint8_t lcl_hal_xcvr_tof_get_agc_delay(uint8_t agc_idx);

void lcl_hal_xcvr_hadm_backup(void);
void lcl_hal_xcvr_hadm_init(const BLE_HADM_SubeventConfig_t *hadm_config, hadm_dc_resid_t *dc_res);
void lcl_hal_xcvr_hadm_deinit(const BLE_HADM_SubeventConfig_t *hadm_config);
void lcl_hal_xcvr_mciq_tsm_setup(void);
void lcl_hal_xcvr_mciq_tsm_override_tx_warmup(void);
void lcl_hal_xcvr_mciq_tsm_override_tx_warmdown(void);
void lcl_hal_xcvr_mciq_enable_tsm_override_t_ip(void);
void lcl_hal_xcvr_mciq_release_tsm_override_t_ip(void);
void lcl_hal_xcvr_mciq_tsm_teardown(void);
void lcl_hal_xcvr_mciq_tsm_override(bool_t freeze_pll, bool_t freeze_aux_pll);
uint16_t lcl_hal_xcvr_mciq_get_agc_status(void);

bool_t lcl_xcvr_hal_check_rsm_state(uint32_t expected_state, XCVR_RSM_RXTX_MODE_T role);
void lcl_xcvr_hal_rsm_configure_dma_mask(uint32_t pm_dur, uint32_t pm_dly, uint32_t mode0_dur, uint32_t mode0_dly);
int32_t lcl_xcvr_hal_get_cfo(XCVR_RSM_SQTE_RATE_T rate);
bool_t lcl_xcvr_hal_rsm_rtt_is_valid(uint32_t step_no);
uint8_t lcl_xcvr_hal_rsm_get_cal_mode(const xcvr_lcl_rsm_reg_config_t *rsm_config_p);
void lcl_xcvr_hal_store_dcoc_cal(BLE_HADM_rttPhyMode_t rate);
void lcl_hal_xcvr_start_dbg_ram_capture(t_hadm_dma_page_t dbg_ram_page, t_hadm_trigger_t start_trigger, uint32_t nb_bytes);
bool_t lcl_hal_xcvr_is_dbg_ram_capture_finished(void);
void lcl_hal_xcvr_configure_dma_capture(t_hadm_trigger_t start_trigger, uint32_t delay, uint32_t nb_words, uint16 m_hadmbuffer_size, uint32 m_hadmbuffer_start);
bool_t lcl_hal_xcvr_is_dma_capture_finished(void);
void lcl_hal_xcvr_configure_ant_switch(uint32_t n_ap, uint8_t t_pm, uint8_t t_sw, uint8_t t_capture, uint8_t rttPhy);
void lcl_hal_xcvr_configure_ant_switch_lut(uint32_t n_ap, uint8_t *toneAntennaIDs_p);
void lcl_hal_xcvr_configure_t_pm_ext(uint32_t n_ap, bool_t enable_tx_in_tpm_ext, uint8_t dummy_ant);
void lcl_hal_xcvr_pll_settings_backup(void);
void lcl_hal_xcvr_pll_settings_restore(void);
void lcl_hal_xcvr_apply_time_grid_shift(int32_t time_shift);
#ifdef RSM_WA_FOR_RX_SETLLING_LATENCY_KFOURWONE_2175
void lcl_hal_xcvr_increase_rx_settling_latency(void);
void lcl_hal_xcvr_restore_rsm_t_fc(void);
#endif

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* _LCL_XCVR_HAL_H_ */

/* EOF */
