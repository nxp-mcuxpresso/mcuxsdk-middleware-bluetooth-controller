/**
 * @file lcl_xcvr_hal.c
 *
 * This file implements LCL XCVR HAL for gen4p7 radio.
 */
/*
 * Copyright 2024 NXP
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* === Includes ============================================================ */
#include "fsl_os_abstraction.h"
#include "EmbeddedTypes.h"
#include "nxp2p4_xcvr.h"
#include "nxp_xcvr_lcl_ctrl.h"
#include "lcl_hadm_measurement.h"
#include "lcl_hadm_utils.h"
#include "lcl_xcvr_hal.h"
#include "controller_api_ll.h"

#if (NXP_RADIO_GEN != 470)
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
    uint32_t dtest_page_backup;

    /* used by XCVR RSM driver */
    rsm_reg_backup_t xcvr_regs_backup;
    xcvr_lcl_tsm_config_t tsm_regs_backup;
    
    /* FIXME: remove duplicates */
} lcl_xcvr_hal_xcvr_settings_t;

/* === Macros ============================================================== */


/* === Globals ============================================================= */
lcl_xcvr_hal_xcvr_settings_t  xcvr_settings;

static uint32_t dcoc_ctrl_2[HADM_RTT_PHY_MAX] = {0, 0}; /* used to store XCVR_RX_DIG->DCOC_CTRL2, typically after calibration for CS operation */

/* === Externals =========================================================== */

/* === Prototypes ========================================================== */

/* === Implementation ====================================================== */

/* *****************************RSM related **********************************/

/* PKT RAM configuration when subevent starts */
void lcl_hal_pkt_ram_config_circ_buffers(hadm_pkt_ram_desc_t *pkt_ram)
{
    uint32_t ptr_word = (uint32_t)pkt_ram->step_config.base_ptr >> 2U;
    /* Config circular buffer */
    XCVR_MISC->RSM_CONFIG_BUFF = XCVR_MISC_RSM_CONFIG_BUFF_RSM_CONFIG_BASE_ADDR(ptr_word) | 
                                 XCVR_MISC_RSM_CONFIG_BUFF_RSM_CONFIG_BUFF_LOC(pkt_ram->step_config.ram_type)  |
                                 XCVR_MISC_RSM_CONFIG_BUFF_RSM_CONFIG_DEPTH(pkt_ram->step_config.buff_len) |
                                 XCVR_MISC_RSM_CONFIG_BUFF_RSM_INT_NBSTEP(pkt_ram->nb_steps_before_irq); /* step IRQ */
    XCVR_MISC->RSM_CONFIG_PTR = XCVR_MISC_RSM_CONFIG_PTR_RSM_CONFIG_START_PTR(ptr_word) | 
                                XCVR_MISC_RSM_CONFIG_PTR_RSM_CONFIG_WR_PTR((uint32_t)pkt_ram->config_write_ptr >> 2U);
    
    /* Result circular buffer */
    ptr_word = (uint32_t)pkt_ram->step_result.base_ptr >> 2U;
    XCVR_MISC->RSM_RESULT_BUFF = XCVR_MISC_RSM_RESULT_BUFF_RSM_RESULT_BASE_ADDR(ptr_word) | 
                                 XCVR_MISC_RSM_RESULT_BUFF_RSM_RESULT_BUFF_LOC(pkt_ram->step_result.ram_type)  |
                                 XCVR_MISC_RSM_RESULT_BUFF_RSM_RESULT_DEPTH(pkt_ram->step_result.buff_len);
    XCVR_MISC->RSM_RESULT_PTR = XCVR_MISC_RSM_RESULT_PTR_RSM_RESULT_START_PTR(ptr_word) |
                                XCVR_MISC_RSM_RESULT_PTR_RSM_RESULT_RD_PTR(ptr_word) |
                                XCVR_MISC_RSM_RESULT_PTR_RSM_BUFFER_ABORT_EN_MASK; /* enable abort on underrun / overflow */
}

/* TQI configuration */
#define HADM_HAL_TQI_THRESHOLD_BAD  (32U)   /* in Q9 (0.063 in decimal) */
#define HADM_HAL_TQI_THRESHOLD_MEDIUM  (HADM_HAL_TQI_THRESHOLD_BAD)

void lcl_hal_xcvr_program_tqi(hadm_meas_t *hadm_meas_p)
{
    assert(hadm_meas_p->iq_avg_win >= 3U);
    /* TQI thresholds */
    XCVR_RX_DIG->TQI_THR = XCVR_RX_DIG_TQI_THR_T1(HADM_HAL_TQI_THRESHOLD_MEDIUM) | XCVR_RX_DIG_TQI_THR_T2(HADM_HAL_TQI_THRESHOLD_BAD);
    /* TQI controls */
    XCVR_RX_DIG->TQI_CTRL = XCVR_RX_DIG_TQI_CTRL_TQI_EN_MASK | 
                            XCVR_RX_DIG_TQI_CTRL_IQ_AVG_DPTH(hadm_meas_p->iq_avg_win - 3U) |  /* configure IQ averager to get 8 windows */
                            XCVR_RX_DIG_TQI_CTRL_MAG_AVG_DPTH(3U);  /* Magnitude averager uses 8 windows*/
}

bool_t lcl_hal_xcvr_decode_mode0_step(hadm_sync_info_t *sync_info_p, uint32_t *rsm_read_ptr, uint8 rate)
{
    uint8 step_idx;
    bool_t aa_det;
    uint32 temp;
    int16_t cfo16;
    int32_t cfo32;
    
    step_idx = (rsm_read_ptr[0U] & 0xFF);
    assert(step_idx < HADM_MAX_NB_STEPS_MODE0);
    
    aa_det = (bool_t)((rsm_read_ptr[0U] & 0x80000000) >> 31U);
    
    if (aa_det)
    {
        sync_info_p += step_idx;
        
        sync_info_p->agc_idx = (rsm_read_ptr[0U] & 0xF000) >> 12U;
        sync_info_p->rssi = (rsm_read_ptr[1U] & COM_MODE_013_RES_BODY_NADM_ERROR_RSSI_RSSI_NB_MASK) >> COM_MODE_013_RES_BODY_NADM_ERROR_RSSI_RSSI_NB_SHIFT;
        temp = rsm_read_ptr[2U];
        sync_info_p->valid = ((temp & (COM_MODE_013_RES_BODY_RTT_RESULT_RTT_VLD_MASK | COM_MODE_013_RES_BODY_RTT_RESULT_RTT_FOUND_MASK)) == (COM_MODE_013_RES_BODY_RTT_RESULT_RTT_VLD_MASK | COM_MODE_013_RES_BODY_RTT_RESULT_RTT_FOUND_MASK));

        /* format is sfix16En15 (15 bits frac, 1 bit sign). CFO=CFO_reg*R/2, where R is the bit rate. */
        cfo16 = ((int16_t) ((temp & COM_MODE_013_RES_BODY_RTT_RESULT_RTT_CFO_MASK) >> COM_MODE_013_RES_BODY_RTT_RESULT_RTT_CFO_SHIFT));
        cfo32 = (int32_t)cfo16;
        if (rate == XCVR_RSM_RATE_1MBPS)
            cfo32 *= 15; /* 1e6 / (2 * 2^15) */
        else
            cfo32 *= 30; /* 1e6 / 2^15 */
        sync_info_p->cfo = cfo32;
    }
    return aa_det;
}

/* For a given CFO, this function computes of much and how often we need to adjust the RSM time grid */
void lcl_hal_xcvr_program_time_adjustement(int32_t ppm)
{
    uint8_t tim_adj; /* +/- 1 us */
    uint32_t temp, time_to_adj;
    
    if (ppm > 0)
      tim_adj = 1U;
    else if (ppm < 0)
      tim_adj = 3U;
    else // ppm == 0, no adjustment required
      return;

    /* Given the measured ppm, compute the time adjustment interval for 1us adjustment */
    time_to_adj = (1000000U / (ABS(ppm))); /* in us */

#if 0 // UT
    tim_adj = 2;
    time_to_adj = 10U;
#endif
    
    temp = XCVR_MISC->RSM_CTRL7;
    temp &= ~(XCVR_MISC_RSM_CTRL7_RSM_TIME_CORR_MODE_MASK | XCVR_MISC_RSM_CTRL7_RSM_TIME_CORR_DELTA_MASK | XCVR_MISC_RSM_CTRL7_RSM_TIME_CORR_MASK);
    
    if (tim_adj != 0)
    {
        temp |= XCVR_MISC_RSM_CTRL7_RSM_TIME_CORR_MODE(1U) |/* enable 1us RSM time grid adjustment */
                XCVR_MISC_RSM_CTRL7_RSM_TIME_CORR_DELTA(tim_adj) |
                XCVR_MISC_RSM_CTRL7_RSM_TIME_CORR(time_to_adj);
    }
    XCVR_MISC->RSM_CTRL7 = temp;
}

//* ******************************* Common ************************************/

static uint32_t lcl_hal_xcvr_dtest_set_page(uint32_t page)
{
    uint32_t dtest_ctrl = RADIO_CTRL_DTEST_CTRL_DTEST_PAGE(RADIO_CTRL->DTEST_CTRL);
    RADIO_CTRL->DTEST_CTRL = (RADIO_CTRL->DTEST_CTRL & ~RADIO_CTRL_DTEST_CTRL_DTEST_PAGE_MASK ) | RADIO_CTRL_DTEST_CTRL_DTEST_EN(1) | RADIO_CTRL_DTEST_CTRL_DTEST_PAGE(page);
    return dtest_ctrl;
}

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
        uint32_t temp = XCVR_RX_DIG->AGC_OVRD;
        temp &= ~(XCVR_RX_DIG_AGC_OVRD_AGC_GAIN_IDX_OVRD_MASK);
        temp |= (XCVR_RX_DIG_AGC_OVRD_AGC_GAIN_IDX_OVRD_EN_MASK | XCVR_RX_DIG_AGC_OVRD_AGC_GAIN_IDX_OVRD(man_agc_idx));
        XCVR_RX_DIG->AGC_OVRD |= temp;
    }
    /* else : Invalid index, remain in automatic mode */
}

/* Store DCOC values from current ones */
void lcl_xcvr_hal_store_dcoc_cal(BLE_HADM_rttPhyMode_t rate)
{
    assert(rate < HADM_RTT_PHY_MAX);
    dcoc_ctrl_2[rate] = XCVR_LCL_SetupManualDcoc();
}

void lcl_hal_xcvr_hadm_backup(void)
{
    /* Backup TSM registers modified during ranging for later restoration */
    (void)XCVR_LCL_GetTsmTimings(&xcvr_settings.tsm_regs_backup);
}

void lcl_hal_xcvr_hadm_init(hadm_meas_t *hadm_meas_p, const BLE_HADM_SubeventConfig_t *hadm_config)
{  
    NbuPwrPeakReductionActivityStart();
    
    xcvr_settings.dtest_page_backup = lcl_hal_xcvr_dtest_set_page(DTEST_RSM3);
    
    /* First, backup registers touched by custom DCOC setting and calibration */
    xcvr_settings.xcvr_rx_dig_ctrl_1 = XCVR_RX_DIG->CTRL1;
    xcvr_settings.xcvr_rx_dig_dcoc_ctrl_0 = XCVR_RX_DIG->DCOC_CTRL0;
    xcvr_settings.xcvr_rx_dig_dcoc_ctrl_2 = XCVR_RX_DIG->DCOC_CTRL2;
    xcvr_settings.xcvr_radio_ctrl_rf_ctrl = RADIO_CTRL->RF_CTRL;
    
    assert(dcoc_ctrl_2[hadm_config->rttPhy] != 0);
    XCVR_LCL_OverrideDcoc(dcoc_ctrl_2[hadm_config->rttPhy], true);
       
    /* Backup XCVR registers modified during ranging for later restoration */
    (void)XCVR_LCL_RsmRegBackup(&xcvr_settings.xcvr_regs_backup);
    
    lcl_hal_xcvr_setup_agc();
    lcl_hal_xcvr_setup_rssi(hadm_config->rttPhy);
    lcl_hal_xcvr_setup_rssi_continuous(TRUE);
    
    XCVR_LCL_RsmCompCfo(0); /* make sure PLL_OFFSET_CTRL is cleared before first mode 0 */
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
       
    (void)lcl_hal_xcvr_dtest_set_page(xcvr_settings.dtest_page_backup);
    
    NbuPwrPeakReductionActivityStop();
}

void lcl_hal_xcvr_pll_settings_backup(void)
{
    (void)XCVR_LCL_RsmPLLBackup(&xcvr_settings.xcvr_regs_backup);
}

void lcl_hal_xcvr_pll_settings_restore(void)
{
    (void)XCVR_LCL_RsmPLLRestore(&xcvr_settings.xcvr_regs_backup);
}

/* ******************************* RTT ************************************/
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

/* ******************************* DMA  ************************************/

void lcl_hal_xcvr_configure_dma_capture(t_hadm_trigger_t start_trigger, uint32_t delay, uint32_t nb_words, uint16 m_hadmbuffer_size, uint32 m_hadmbuffer_start)
{
  if(m_hadmbuffer_start != NULL)
  {
    uint32 temp;
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
    
    /* enable DMA mask for T_PM and T_FM */
    temp = XCVR_MISC->DMA_MASK_CTRL;
    temp &= ~(XCVR_MISC_DMA_MASK_CTRL_DMA_SIGNAL_VALID_MASK_SEL_MASK);
    temp |= XCVR_MISC_DMA_MASK_CTRL_DMA_SIGNAL_VALID_MASK_SEL(1U); /* Use LCL legacy DMA mask */
    XCVR_MISC->DMA_MASK_CTRL = temp;

    /* Start DMA with given trigger, raising edge */
    temp = XCVR_MISC->DMA_CTRL = XCVR_MISC_DMA_CTRL_DMA_EN_MASK | XCVR_MISC_DMA_CTRL_DMA_SIGNAL_VALID_MASK_EN_MASK |
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

/* ******************************* LCL  ************************************/

#define T_CAPTURE_DELAY (17U) /* accounts for RX warmup delay + data path latency. Measured. */
#define T_TX_ANT_SW_DELAY (2U) /* to align lant_sw signals on RX and TX side. RX TX_PM state starts 2us earlier (TX_DATA_FLUSH_DLY). */

#if 0 /* Deprecated, see XCVR API */
/* On KW47, define SPINT to be always 1us */
void lcl_hal_xcvr_configure_LCL(hadm_meas_t *hadm_meas_p, uint8_t *toneAntennaIDs_p)
{ 
    uint32_t temp, offset, sampling_rate_factor;
    uint32_t t_capture = hadm_meas_p->iq_capture_win; /* HI_PER*/
    uint32_t lo_per = (uint32_t)hadm_meas_p->config_p->T_PM_Time + (uint32_t)hadm_meas_p->config_p->T_SW_Time - t_capture; /* LO_PER */
    
    /* Configure 2Mbps oversampling if needed */
    sampling_rate_factor = (hadm_meas_p->config_p->rttPhy == HADM_RTT_PHY_1MBPS) ? 1U : SAMPLING_RATE_FACTOR_2MBPS;

    /* RX/TX config */
    XCVR_MISC->LCL_RX_CFG0 = 0; /* trigger delay = 0 */
    XCVR_MISC->LCL_TX_CFG0 = XCVR_MISC_LCL_TX_CFG0_TX_DELAY(T_TX_ANT_SW_DELAY); /* trigger delay = T_TX_ANT_SW_DELAY */

    temp = XCVR_MISC_LCL_RX_CFG1_RX_ANT_TRIG_SEL(6) /* RSM trigger */ |
        XCVR_MISC_LCL_RX_CFG1_RX_HI_PER(t_capture /* HI_PER: dma mask enable duration */) | 
        XCVR_MISC_LCL_RX_CFG1_RX_LO_PER(lo_per); /* LO_PER */

    /* TX and RX configs have same register mapping. SPINT is different due to sampling rate */
    XCVR_MISC->LCL_RX_CFG1 = temp | XCVR_MISC_LCL_RX_CFG1_RX_SPINT(RX_SAMPLING_RATE*sampling_rate_factor-1U);
    XCVR_MISC->LCL_TX_CFG1 = temp | XCVR_MISC_LCL_TX_CFG1_TX_SPINT(TX_SAMPLING_RATE*sampling_rate_factor-1U);

    /* Configure TX+RX mode, and duration + automatic permutation. All other fields are reset value (0) */
    temp = XCVR_MISC_LCL_CFG0_CTE_DUR(hadm_meas_p->n_ap) | XCVR_MISC_LCL_CFG0_RX_LCL_EN_MASK | XCVR_MISC_LCL_CFG0_TX_LCL_EN_MASK; /* account for T_PM ext */
    temp |= XCVR_MISC_LCL_CFG0_AP_MAX(hadm_meas_p->n_ap-1U) | XCVR_MISC_LCL_CFG0_LCL_ANT_PERMUT_EN_MASK;
    XCVR_MISC->LCL_CFG0 = temp;
#if 0 /* OJE TODO: enable to get ramp down during antenna switching */
    XCVR_MISC->LCL_CFG1 = XCVR_MISC_LCL_CFG1_ANT_SW_RF_MASK | XCVR_MISC_LCL_CFG1_ANT_SW_MODE3_MASK;
#endif   
    /* workaround for KFOURWONE-702 OJE TODO: still required ? */
    offset = (T_CAPTURE_DELAY + lo_per) / 2U;
    temp = XCVR_MISC_LCL_DMA_MASK_DELAY_DMA_MASK_DELAY(offset); /* LO_PER/2 to get dma_mask centered inside T_PM */
    offset = (T_CAPTURE_DELAY + lo_per) % 2U;
    temp |= XCVR_MISC_LCL_DMA_MASK_DELAY_DMA_MASK_DELAY_OFF(offset*RX_SAMPLING_RATE*sampling_rate_factor);
    XCVR_MISC->LCL_DMA_MASK_DELAY = temp;
    XCVR_MISC->LCL_DMA_MASK_PERIOD = XCVR_MISC_LCL_DMA_MASK_PERIOD_DMA_MASK_REF_PER(t_capture); /* == HI_PER */

    /* Configure LCL LUT: this assumes no more than 4 AP can be configured */
    XCVR_MISC->LCL_GPIO_CTRL4 = XCVR_MISC_LCL_GPIO_CTRL4_LUT_WRAP_PTR(hadm_meas_p->n_ap-1U); /* T_PM repetition handled by HW */
    temp = toneAntennaIDs_p[0] | (toneAntennaIDs_p[1] << 4U) | (toneAntennaIDs_p[2] << 8U) | (toneAntennaIDs_p[3] << 12U);
    XCVR_MISC->LCL_GPIO_CTRL0 = temp;
}
#endif

/* EOF */
