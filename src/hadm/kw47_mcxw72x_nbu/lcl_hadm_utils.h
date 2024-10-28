/**
 * @file lcl_hadm_utils.h
 *
 * This file implements some utility functions for HADM
 */
/*
 * Copyright 2024 NXP
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* Prevent double inclusion */
#ifndef _LCL_HADM_UTILS_H_
#define _LCL_HADM_UTILS_H_

/* === Includes ============================================================ */
#include "ll_types.h" /* maps LL types to SDK types */
#include "ble_hadm_hal.h"

/* === Types =============================================================== */


/* === Macros ============================================================== */
#ifndef NDEBUG
#define HADM_ENABLE_DEBUG_PINS /* Disable in case of conflicts with PTD2-3 PINs */
#endif

#define HADM_TPM                   TPM2
#define HADM_TPM_IRQ               TPM2_INT_IRQn
#define HADM_TPM_TIMER_CHANNEL     kTPM_Chnl_1
#define HADM_TPM_MODULO            (0xFFFFFFFFU)

/* DEBUG_PIN0: low frequency debug pin
 * DEBUG_PIN1: high frequency debug pin
 */
#ifdef HADM_ENABLE_DEBUG_PINS
#define DEBUG_PIN0_SET {GPIOD->PSOR = 0x08;} // set PTD3
#define DEBUG_PIN0_CLR {GPIOD->PCOR = 0x08;} // clear PTD3
#define DEBUG_PIN0_TGL {GPIOD->PTOR = 0x08;} // toggle PTD3
#define DEBUG_PIN0_PULSE {DEBUG_PIN0_TGL DEBUG_PIN0_TGL}
#define DEBUG_PIN1_SET {GPIOD->PSOR = 0x04;} // set PTD2
#define DEBUG_PIN1_CLR {GPIOD->PCOR = 0x04;} // clear PTD2
#define DEBUG_PIN1_TGL {GPIOD->PTOR = 0x04;} // toggle PTD2
#define DEBUG_PIN1_PULSE {DEBUG_PIN1_TGL DEBUG_PIN1_TGL}
#define DEBUG_PIN_ALL_TGL {GPIOD->PTOR = 0x0C;} // toggle PTD2 and PTD3
#else
#define DEBUG_PIN0_SET
#define DEBUG_PIN0_CLR
#define DEBUG_PIN0_TGL
#define DEBUG_PIN0_PULSE
#define DEBUG_PIN1_SET
#define DEBUG_PIN1_CLR
#define DEBUG_PIN1_TGL
#define DEBUG_PIN1_PULSE
#define DEBUG_PIN_ALL_TGL
#endif

/* "If the reference power level value is not available during a subevent, then this value shall be set to 0x7F." */
#define HADM_INVALID_REFERENCE_POWER_LEVEL   (0x7F)

/* Fine tuning for RTT HW compensation */
#define HADM_RXTX_FINE_LATENCY_1MBPS_HNS     (-1720)
#define HADM_RXTX_FINE_LATENCY_2MBPS_HNS     (-3286)

#define HADM_TX_LATENCY_NS             (2000U)  /* tx_on (RSM DT_TX state to 1st bit over the air). Measured */

#define HADM_AA_MATCH_DMA_LATENCY      (3U) /* 3us latency btw AA match trigger and last AA bit. Measured...*/
#define HADM_AA_MATCH_LATENCY_1MBPS_US (7U) /* ~6.5us latency between end of AA over the air and internal aa_match signal */
#define HADM_AA_MATCH_LATENCY_2MBPS_US (4U) /* ~3.5us latency between end of AA over the air and internal aa_match signal */
#define HADM_RX_EN_DMA_LATENCY(phy)    (phy == 0 ? 12U : 8U) /* latency btw RX enable trigger and first preamble bit. Measured...12us @1Mbps, 8us @2Mbps*/

#define HADM_RTT_TRAILER_BITS          (4U)
#define HADM_RTT_AA_LEN                (32U)  /* 32 bits */
#define HADM_RTT_PREAMBLE_LEN_1MBPS    (8U)   /* 8 bits */
#define HADM_RTT_PREAMBLE_LEN_2MBPS    (16U)  /* 16 bits */
#define HADM_RTT_PREAMBLE_DURATION_US  (8U)   /* 8us whatever the phy */

/* Delay between 1st preamble bit arrival time and AA match signal (= preamble duration + AA duration + AA match delay) */
#define HADM_1ST_BIT_TO_AA_MATCH_DURATION_1MBPS_US (HADM_RTT_PREAMBLE_DURATION_US + HADM_RTT_AA_LEN + HADM_AA_MATCH_LATENCY_1MBPS_US)
#define HADM_1ST_BIT_TO_AA_MATCH_DURATION_2MBPS_US (HADM_RTT_PREAMBLE_DURATION_US + (HADM_RTT_AA_LEN>>1) + HADM_AA_MATCH_LATENCY_2MBPS_US)
#define HADM_1ST_BIT_TO_AA_MATCH_DURATION_US(phy)  (((phy) == HADM_RTT_PHY_2MBPS) ? HADM_1ST_BIT_TO_AA_MATCH_DURATION_2MBPS_US:HADM_1ST_BIT_TO_AA_MATCH_DURATION_1MBPS_US)

/*! Translates RSM RTT validity results to HADM AA quality */
#define HADM_SET_RTT_AA_QUALITY(valid, found) \
    (valid ? (found ? HADM_AA_QUALITY_SUCCESS : HADM_AA_QUALITY_NOT_FOUND) : HADM_AA_QUALITY_ERROR)

/*! Set packet RSSI */
#define HADM_SET_RTT_RSSI(valid, rssi) \
    (valid ? rssi : 0x7F);

/*! OJE TODO: Set packet NADM */
#define HADM_SET_RTT_NADM(raw_nadm) \
    (0xFF);

/*! Translates RSM RTT CFO result to HADM CFO (2 bytes, unit 0.01 ppm, SQ15.0 format) */
#define HADM_SET_RTT_CFO(valid, ppm_in, report_p) \
    do \
    { \
        if (valid) \
        { \
            /* Clip to 15bits signed max : -100 ppm (0xD8F0) to +100 ppm (0x2710)*/ \
            if (ppm_in > 10000) ppm_in = 10000; \
            else if (ppm_in < -10000) ppm_in = -10000; \
            *report_p++ = (ppm_in & 0x000000FF); \
            *report_p++ = (ppm_in & 0x0000FF00) >> 8U; \
        } \
        else \
        { \
            *report_p++ = 0x00; \
            *report_p++ = 0xC0; \
        } \
    } while (0)

/* Convert timestamps (32Mhz ticks) to ns = ts * 31.25 */
#define HADM_RTT_TS_TO_NS(ts) (((ts) * 125) >> 2U)
/* Convert timestamps (32Mhz ticks) to us = ts / 32 */
#define HADM_RTT_TS_TO_US(ts) ((ts) >> 5U)

/*! Translates RTT timestamp difference to HADM time difference (3 bytes, Time diff in half ns, Q16.0 format, 0x8000 if unavailable) */
#define HADM_SET_RTT_TS_DIFF(valid, ts_diff, report_p) \
    do \
    { \
        if (valid) \
        { \
            *report_p++ = (ts_diff & 0x00FF); \
            *report_p++ = (ts_diff & 0xFF00) >> 8U; \
        } \
        else \
        { \
            *report_p++ = 0x00; \
            *report_p++ = 0x80; \
        } \
    } while (0)

/*! Translates IQ sample to HADM PCT:
 *    IQ: 2x11bits with sign bit MSB aligned.
 *    Phase Correction Term (22 bits containing 11 least significant bits to indicate I sample and 11 most significant bits to indicate Q sample)
 * Note: HW output is :{rx_dft_iq_out_q[10:0], rx_if_mixer_idx[9:5], rx_dft_iq_out_i[10:0], rx_if_mixer_idx[4:0]}
 */
#define HADM_SET_RTP_PCT(iq_in, report_p) \
    do \
    {\
        uint32_t iq_out = ((iq_in & 0xfff0) >> 4U) | ((iq_in & 0xfff00000) >> 8U); /* store rx_dft_iq_out_i in LSB and rx_dft_iq_out_q in MSB (packed in 2x12bits) */ \
        *report_p++ = (iq_out & 0x0000FF); \
        *report_p++ = (iq_out & 0x00FF00) >> 8U; \
        *report_p++ = (iq_out & 0xFF0000) >> 16U; \
    } while (0)

/*! Encode Tone Quality Indicator */
#define HADM_SET_RTP_TONE_QUALITY(iq_in, report_p, ext_slot, ext_pres) \
    do \
    { \
        uint8_t qual_metric = iq_in & 0x3U;\
        if (ext_slot) \
        {\
            if (ext_pres)\
                qual_metric |= (HADM_TQI_TONE_EXT_PRESENT << 4U);\
            else \
                qual_metric |= (HADM_TQI_TONE_EXT_NOT_PRESENT << 4U);\
        } /* else MSBs already set to 0 */\
        *report_p++ = qual_metric; \
    } while (0)


/* === Globals ============================================================= */

/* === Externals =========================================================== */

/* === Prototypes ========================================================== */
#ifdef __cplusplus
extern "C" {
#endif

void lcl_hadm_utils_init_buffers(void);
BLE_HADM_SubeventConfig_t *lcl_hadm_utils_get_config_buffer();
BLE_HADM_SubeventResultsData_t *lcl_hadm_utils_get_result_buffer(void);
void lcl_hadm_utils_free_config_buffer(uint8 connIdx);
void lcl_hadm_utils_free_result_buffer(uint8 connIdx);
void lcl_hadm_enable_lcl_interrupts(void);
void lcl_hadm_disable_interrupts(void);
void lcl_hadm_restore_interrupts(void);
void lcl_hadm_init_tpms(void);
void lcl_hadm_start_tpms(void);
void lcl_hadm_stop_tpms(void);
void lcl_hadm_tpm_timer_start(uint16_t delay_us);
void lcl_hadm_tpm_timer_stop(void);
void lcl_hadm_utils_compute_iq_buff_size(const BLE_HADM_SubeventConfig_t *hadm_config_p, hadm_meas_t *hadm_meas_p, uint32_t sample_rate);
void lcl_hadm_utils_compute_step_duration(const BLE_HADM_SubeventConfig_t *hadm_config_p, uint32_t n_ap, uint16_t *mode_dur);
uint32_t lcl_hadm_utils_wait_for_rsm_irq(uint32_t *rsm_step_no, uint32_t rsm_mode);
void lcl_hadm_utils_calc_rtt_temperature_delay(int32_t temperature, hadm_device_t *hadm_device);
void lcl_hadm_utils_calc_ts_delay(hadm_meas_t *hadm_meas_p, hadm_device_t *hadm_device);
void lcl_hadm_utils_compute_pct_and_tqi(uint32_t num_iq_per_step_per_ap, uint32_t n_ap, uint32_t **out_buf_p, uint8_t **out_quality_buf_p);
void lcl_hadm_utils_compute_pct(uint32_t num_iq_per_step_per_ap, uint32_t n_ap, uint32_t **out_buf_p, uint8_t **out_quality_buf_p);
void lcl_hadm_utils_configure_antenna_switching(hadm_meas_t *hadm_meas_p);
int8_t lcl_hadm_utils_compute_rpl(uint32_t iq, int8_t rssi);
uint8_t lcl_hadm_utils_get_CS_SYNC_antenna(hadm_meas_t *hadm_meas_p);
uint16_t lcl_hadm_get_hpm_cal_interpolation(uint8_t chan, uint16_t ref_cal);

void lcl_hadm_AES_EncryptEcb_128(const uint32_t *key, const uint32_t *plaintext, uint32_t *ciphertext);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* _LCL_HADM_UTILS_H_ */

/* EOF */
