/**
 * @file lcl_hadm_utils.c
 *
 * This file implements some utility functions for HADM
 */
/*
 * Copyright 2021-2024 NXP
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* === Includes ============================================================ */
#include "fsl_os_abstraction.h"
#include "EmbeddedTypes.h"
#include "tx_api.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
#include "fsl_tpm.h"
#include "lcl_hadm_measurement.h"
#include "lcl_hadm_utils.h"
#include "lcl_xcvr_hal.h"
#include "math.h" /* for log() */
#include "fwk_platform_sensors.h"

/* === Types =============================================================== */

#define LCL_HADM_RTT_DBG_BUFF_SIZE (20)
#define LCL_HADM_MCIQ_DBG_BUFF_SIZE (20)
#define LCL_HADM_RSM_DBG_BUFF_SIZE ((2*HADM_MAX_NB_STEPS) + 10)

#ifdef RSM_DEBUG
#if (LCL_HAL_TPM_IRQ_MAX > 0x800)
#error "lcl_hadm_rsm_debug: irq field size not large enough"
#endif
/* Take care of alignement to minimize buffer size */
typedef struct
{
    uint8_t step_no;
    uint8_t step_format;
    uint16_t irq;
    t_hadm_rsm_states state; /* 1 byte */
#ifdef RSM_DEBUG_CAL
    uint8_t  ctune;
    uint16_t hpm_cal;
#elif defined (RSM_DEBUG_DMA)
    uint16_t ptr;
    uint16_t ccnt;
#else
    int8_t   rssi;
    uint16_t time;
#endif
} lcl_hadm_rsm_debug;

typedef PACKED_STRUCT
{
    uint8_t  ctune;
} lcl_hadm_mciq_debug;

typedef PACKED_STRUCT
{
    uint16_t t1;
    uint16_t t2;
    uint32_t hartt_stat;
    uint32_t stat0;
} lcl_hadm_rtt_debug;

#endif

/* Contiguous packed memory area used for reporting info via HciLeHadmEventResultDebugEvent */
typedef PACKED_STRUCT hadm_full_info_tag {
    uint16_t rtt_dbg_buffer_nb;
    uint16_t mciq_dbg_buffer_nb;
    hadm_info_t info;
#ifdef RTT_DEBUG
    lcl_hadm_rtt_debug rtt_dbg_buffer[LCL_HADM_RTT_DBG_BUFF_SIZE];
#endif
#ifdef MCIQ_DEBUG
    lcl_hadm_mciq_debug mciq_dbg_buffer[LCL_HADM_MCIQ_DBG_BUFF_SIZE];
#endif
} hadm_full_info_t;


/* === Macros ============================================================== */

/* === Globals ============================================================= */
uint32_t nvic_backup;

BLE_HADM_SubeventConfig_t  configBuffer[HADM_MAX_NB_CONNECTIONS];
BLE_HADM_SubeventResultsData_t resultsDataBuffer;
static uint8 gpHadmBuffer[BLE_HADM_HAL_REPORT_SIZE_MAX];
hadm_full_info_t hadm_full_info;

#ifdef RSM_DEBUG
uint32_t rsm_iter; // index in rsm_dbg array
lcl_hadm_rsm_debug rsm_dbg[LCL_HADM_RSM_DBG_BUFF_SIZE];
/* To save some context right after IRQ triggers */
uint32_t rsm_dbg_tpm_cnt;
uint32_t rsm_dbg_rsm_csr;
#endif

/*! Antenna permutation index: N_AP=4*/
const uint8_t hadm_ant_perm_n_ap[24][4] = {
    {0,1,2,3}, // A1,A2,A3,A4
    {1,0,2,3}, // A2,A1,A3,A4
    {0,2,1,3}, // A1,A3,A2,A4
    {2,0,1,3}, // A3,A1,A2,A4
    {2,1,0,3}, // A3,A2,A1,A4
    {1,2,0,3}, // A2,A3,A1,A4
    {0,1,3,2}, // A1,A2,A4,A3
    {1,0,3,2}, // A2,A1,A4,A3
    {0,3,1,2}, // A1,A4,A2,A3
    {3,0,1,2}, // A4,A1,A2,A3
    {3,1,0,2}, // A4,A2,A1,A3
    {1,3,0,2}, // A2,A4,A1,A3
    {0,3,2,1}, // A1,A4,A3,A2
    {3,0,2,1}, // A4,A1,A3,A2
    {0,2,3,1}, // A1,A3,A4,A2
    {2,0,3,1}, // A3,A1,A4,A2
    {2,3,0,1}, // A3,A4,A1,A2
    {3,2,0,1}, // A4,A3,A1,A2
    {3,1,2,0}, // A4,A2,A3,A1
    {1,3,2,0}, // A2,A4,A3,A1
    {3,2,1,0}, // A4,A3,A2,A1
    {2,3,1,0}, // A3,A4,A2,A1
    {2,1,3,0}, // A3,A2,A4,A1
    {1,2,3,0}, // A2,A3,A4,A1
};

/* Conversion LUT for 2x2: from permutation index to antenna index */
/* "A1 is assigned to 1:1, A2 is assigned to 1:2, A3 is assigned to 2:1 and A4 is assigned to 2:2" */
const uint8_t hadm_ant_perm_to_idx[2][HADM_MAX_NB_ANTENNA_PATHS] = 
{
  {0U,0U,1U,1U}, // Init
  {0U,1U,0U,1U}  // Refl
};

/* === Externals =========================================================== */

/* === Prototypes ========================================================== */

/* === Implementation ====================================================== */

/* Initialise HAL config & results buffer */
void lcl_hadm_utils_init_buffers(void)
{
    for (int i=0; i < HADM_MAX_NB_CONNECTIONS; i++)
    {
        configBuffer[i].configBufferUsed = 0;
    }
    resultsDataBuffer.resultBufferUsed = 0;
}

/* Allocate one HAL config buffer */
BLE_HADM_SubeventConfig_t *lcl_hadm_utils_get_config_buffer()
{
    BLE_HADM_SubeventConfig_t *config_p = NULL;
    
    for (int i=0; i < HADM_MAX_NB_CONNECTIONS; i++)
    {
        if (configBuffer[i].configBufferUsed == 0)
        {
            config_p = &configBuffer[i];
            config_p->configBufferUsed = 1;
            config_p->stepsNb = 0;
            config_p->role = HADM_ROLE_INVALID;
            break;
        }
    }
    
    return config_p;
}

/* Allocate one HAL result buffer */
BLE_HADM_SubeventResultsData_t *lcl_hadm_utils_get_result_buffer(bool_t with_debug)
{
    BLE_HADM_SubeventResultsData_t *result_p = &resultsDataBuffer;
    
    if (result_p->resultBufferUsed == 1U)
    {
        return NULL;
    }
    result_p->resultBufferUsed = 1;
    result_p->resultBuffer = gpHadmBuffer;
    result_p->resultBufferSize = sizeof(gpHadmBuffer);
    
    if (with_debug)
    {
         result_p->debugBuffer = (uint8_t *)&hadm_full_info;
         result_p->debugBufferSize = sizeof(hadm_full_info);;
    } 
    else 
    {
         result_p->debugBuffer = NULL;
         result_p->debugBufferSize = 0;
    }

    /* Do some inits */
    result_p->referencePwrLevel = HADM_INVALID_REFERENCE_POWER_LEVEL;
    result_p->frequencyCompensation = 0xC000;
    result_p->syncDelayUs = 0;
    result_p->firstStepCollected = 0; /* no step interrupt */
    
    return result_p;
}

hadm_info_t *lcl_hadm_utils_get_info_ptr(void) {
    return &hadm_full_info.info;
}

#ifdef HADM_PLL_CAL_INTERPOLATION
uint16_t lcl_hadm_get_hpm_cal_interpolation(uint8_t chan, uint16_t ref_cal)
{ 
    uint32_t temp_cal;
    /* This array contains Fref^3 / freq^3 for all HADM channels 0..78, stored in fixed-point Q7 format */
    /* with Fref = 2442 MHz (channel 40) */
    static const uint8_t hpm_interp_fact[HADM_MAX_CHANNELS] = 
    {
         135, 134, 134, 134, 134, 134, 133, 133, 133, 133, 133, 133, 133, 132, 132, 132, 132, 132, 132, 131, 131, 131, 131, 131, 131, 130, 130, 130, 130, 
         130, 130, 129, 129, 129, 129, 129, 129, 128, 128, 128, 128, 128, 128, 128, 127, 127, 127, 127, 127, 127, 126, 126, 126, 126, 126, 126, 126, 125, 
         125, 125, 125, 125, 125, 124, 124, 124, 124, 124, 124, 124, 123, 123, 123, 123, 123, 123, 123, 122, 122
    };
    temp_cal = (uint32_t)hpm_interp_fact[chan];
    temp_cal = (temp_cal * ref_cal) >> 7U; /* compensate for Q7 format */

    return (uint16_t)temp_cal;
}
#endif

/* Compute RTT latency for a given temperature.
 * Characterization has shown that the delay can be approximated as a polynomial of degree 2.
 * Curve is positioned on Y axis so that there's no delay at 20 degrees C which is the recommended
 * temperature to perform zero distance calibration.
 * This will also garantie normal behavior at 20 degrees C if the host does not transmit temperature information (hcibb).
 * For 1Mbps: delay (half ns) =  0.0012x2 + 0.0488x - 1
 * For 2Mbps: delay (half ns) =  0.0007x2 + 0.056x - 1
 * Fixed point conversions:
 *      0.0012  => 20*2^14
 *      0.0488  => 800*2^14
 *      0.0007  => 11*2^14
 *      0.056   => 917*2^14
 * Return: half ns unit
 */
#define HADM_TEMP_SCALE_FACTOR (1<<14)
#define HADM_CALC_RTT_TEMP_DELAY_1MBPS(_TEMP) (((20 * (_TEMP) + 800) * (_TEMP))/HADM_TEMP_SCALE_FACTOR - 1)
#define HADM_CALC_RTT_TEMP_DELAY_2MBPS(_TEMP) (((11 * (_TEMP) + 917) * (_TEMP))/HADM_TEMP_SCALE_FACTOR - 1)
void lcl_hadm_utils_calc_rtt_temperature_delay(int32_t temperature, hadm_device_t *hadm_device)
{
    hadm_device->rtt_temperature_comp_hns[HADM_RTT_PHY_1MBPS] = HADM_CALC_RTT_TEMP_DELAY_1MBPS(temperature);
    hadm_device->rtt_temperature_comp_hns[HADM_RTT_PHY_2MBPS] = HADM_CALC_RTT_TEMP_DELAY_2MBPS(temperature);
}

/* Compute latency that has to be removed from ToA-ToD values (resp. substracted from ToD-ToA) :
 *      - nominal offset
 *      - HW contribution (HW TPM triggers vs 1st preamble bit reference)
 * Return: half ns unit
 */
uint32_t lcl_hadm_utils_calc_ts_delay(const BLE_HADM_SubeventConfig_t *hadm_config_p, hadm_device_t *hadm_device)
{
    int32_t ts_nominal_delay;
    int32_t ts_hw_delay = 0;
    int32_t ts_temperature_delay = 0;

    assert(hadm_config_p->rttPhy < HADM_RTT_PHY_MAX);

    /* Remove "the known nominal offsets" as per spec (based on step duration) */
    if (hadm_config_p->role == HADM_ROLE_INITIATOR)
    {
        ts_nominal_delay = (HADM_T_SY(hadm_config_p->rttPhy) + HADM_T_RD + hadm_config_p->T_IP1_Time) * 1000;
    }
    else
    {
        /* Due to TSM implementation, tx_dig_en is asserted on the shortest T_IP. Need to take that into account */
        ts_nominal_delay = (HADM_T_SY(hadm_config_p->rttPhy) + HADM_T_RD + MIN(hadm_config_p->T_IP1_Time,hadm_config_p->T_IP2_Time)) * 1000;
    }
    /* Convert to half ns */
    ts_nominal_delay *= 2U;

    /* Coarse HW contribution:
     * TX: elapsed time between tx_dig_en (TPM trigger on TX) and 1st bit over the air
     * RX: elapsed time between last bit of AA and aa_match_to_ll (TPM trigger on RX) + duration of preamble and AA
     */
    ts_hw_delay = HADM_TX_LATENCY_NS + HADM_1ST_BIT_TO_AA_MATCH_DURATION_US(hadm_config_p->rttPhy) * 1000U;
    /* Convert to half ns */
    ts_hw_delay *= 2U;

    /* Finer HW compensation based on characterization (EVK boards) */
    ts_hw_delay += (hadm_config_p->rttPhy == HADM_RTT_PHY_1MBPS) ? HADM_RXTX_FINE_LATENCY_1MBPS_HNS : HADM_RXTX_FINE_LATENCY_2MBPS_HNS;

    /* Perform fine tuning correction (only in normal operation, skipped during distance calibration) */
    if (hadm_config_p->distanceCalMode == HADM_DIST_CAL_MODE_DISABLED)
    {
        ts_hw_delay += hadm_device->zero_distance_comp.rttFineTuningHns[hadm_config_p->rttPhy];
    }

    /* Latency due to temperature */
    if (hadm_config_p->distanceCalMode == HADM_DIST_CAL_MODE_DISABLED)
    {
        ts_temperature_delay = hadm_device->rtt_temperature_comp_hns[hadm_config_p->rttPhy];
    }

    if (hadm_config_p->role == HADM_ROLE_REFLECTOR) {
        ts_hw_delay = -ts_hw_delay;
        ts_temperature_delay = -ts_temperature_delay;
    }

    return (uint32_t)(int32_t)(ts_nominal_delay + ts_hw_delay + ts_temperature_delay);
}

/* Disable all interrupts for NVIC except RSM IRQ and global interrupt enable */
/* Note that TPM used for ToF does not need need IRQ */
void lcl_hadm_disable_interrupts(void)
{
    OSA_DisableIRQGlobal();
    nvic_backup = NVIC->ICER[0];
    NVIC->ICER[0] = 0xffffffff; // disable all interrupts
    
    NVIC_ClearPendingIRQ(RSM_INT_IRQn);
    EnableIRQ(RSM_INT_IRQn);
    NVIC_ClearPendingIRQ(TPM2_INT_IRQn);
    EnableIRQ(TPM2_INT_IRQn);
}

void lcl_hadm_restore_interrupts(void)
{
    NVIC->ISER[0] = nvic_backup;
    OSA_EnableIRQGlobal();
}

void lcl_hadm_init_tpms(void)
{
    tpm_config_t tpmInfo;
    TPM_GetDefaultConfig(&tpmInfo);
    
    TPM_Init(HADM_TPM, &tpmInfo);
    TPM_SetupInputCapture(HADM_TPM, HADM_TPM_TIMESTAMP_CHANNEL, kTPM_RisingEdge); // input capture mode
    TPM_SetupOutputCompare(HADM_TPM, HADM_TPM_TIMER_CHANNEL, kTPM_NoOutputSignal, HADM_TPM_MODULO); // software compare
    HADM_TPM->CONTROLS[HADM_TPM_TIMER_CHANNEL].CnV = HADM_TPM_MODULO;
    TPM_SetTimerPeriod(HADM_TPM, HADM_TPM_MODULO);
}

void lcl_hadm_start_tpms(void)
{    
    HADM_TPM->CNT = 0;
    TPM_ClearStatusFlags(HADM_TPM, (1<<HADM_TPM_TIMESTAMP_CHANNEL));
    TPM_StartTimer(HADM_TPM, kTPM_SystemClock);
}

void lcl_hadm_stop_tpms(void)
{
    TPM_StopTimer(HADM_TPM);
}

void lcl_hadm_reset_tpm_count(void)
{    
    HADM_TPM->CNT = 0;
}

bool_t lcl_hadm_flush_tpm_timestamp(uint32_t *timestamp)
{
    bool_t has_triggered = ((TPM_GetStatusFlags(HADM_TPM) & (1<<HADM_TPM_TIMESTAMP_CHANNEL)) != 0);

    *timestamp = HADM_TPM->CONTROLS[HADM_TPM_TIMESTAMP_CHANNEL].CnV;

    TPM_ClearStatusFlags(HADM_TPM, (1<<HADM_TPM_TIMESTAMP_CHANNEL));

    return has_triggered;
}

int8_t lcl_hadm_get_tpm_timestamp_frac_adjustment(BLE_HADM_rttPhyMode_t phyMode)
{
    /* Radio >= 3.5 supports a specific timing feature to adjust timestamp trigered by AA_fnd. */
    int8_t frac = lcl_hal_xcvr_tof_get_frac();

    /* divide by 4 for 1mbps or divide by 8 for 2mpbs */
    if (phyMode == HADM_RTT_PHY_1MBPS)
        frac /= 4;
    else
        frac /= 8;
    /* The fractional value represents adjustment between true peak an AA_found signal. It is a signed Q6.5.
    * We need to substract this value from timestamp (positive frac means event occured earlier than timestamp).
    * - Check the sign bit and compute 2's complement if negative (obtain abs value)
    * - Right shift the value by 3 or 2 bits based on PHY clock (8MHz or 4MHz, i.e. 2Mbps or 1Mbps)
    *   This is to convert the 4MHz frac to 32MHz ticks.
    *   Ex for 4MHz: TimeStampAdjust = (FRAC/2^5)*(32MHz/4MHz) = FRAC/4
    */
    return frac;
}

void lcl_hadm_tpm_timer_start(uint16_t delay_us)
{
    TPM_SetupOutputCompare(HADM_TPM, HADM_TPM_TIMER_CHANNEL, kTPM_NoOutputSignal, (delay_us << 5) & HADM_TPM_MODULO); // software compare
    TPM_EnableInterrupts(HADM_TPM, (uint32_t)(1U << HADM_TPM_TIMER_CHANNEL)); // enable interrupt for timer channel
    TPM_ClearStatusFlags(HADM_TPM, 1<<HADM_TPM_TIMER_CHANNEL);
    NVIC_ClearPendingIRQ(TPM2_INT_IRQn);
}

void lcl_hadm_tpm_timer_stop(void)
{
    TPM_DisableInterrupts(HADM_TPM, (uint32_t)(1U << HADM_TPM_TIMER_CHANNEL)); // enable interrupt for timer channel
    /* disable channel, no SDK API to do that... */
    HADM_TPM->CONTROLS[HADM_TPM_TIMER_CHANNEL].CnSC &= ~(TPM_CnSC_MSA_MASK | TPM_CnSC_MSB_MASK | TPM_CnSC_ELSA_MASK | TPM_CnSC_ELSB_MASK);
}

void lcl_hadm_utils_compute_iq_buff_size(const BLE_HADM_SubeventConfig_t *hadm_config_p, hadm_meas_t *hadm_meas_p, uint32_t sample_rate)
{

    hadm_meas_p->iq_buff_size_mode0 = 0;


    if (hadm_meas_p->iq_avg_win > 0)
    {
        assert((hadm_meas_p->iq_capture_win * sample_rate) >= (1 << hadm_meas_p->iq_avg_win));
        assert(((hadm_meas_p->iq_capture_win * sample_rate) &  ((1 << hadm_meas_p->iq_avg_win) - 1)) == 0);
        assert((hadm_meas_p->iq_capture_win * sample_rate) >= (1 << hadm_meas_p->iq_avg_win));
        assert(((hadm_meas_p->iq_capture_win * sample_rate) &  ((1 << hadm_meas_p->iq_avg_win) - 1)) == 0);
    }


    uint32 nb_samples_per_pm = HADM_NUM_IQ_PER_US(hadm_meas_p->iq_capture_win, hadm_config_p->rttPhy, hadm_meas_p->iq_avg_win);
    if (hadm_config_p->role == HADM_ROLE_INITIATOR)
    {
        hadm_meas_p->iq_buff_size_mode0 = nb_samples_per_pm * hadm_config_p->mode0Nb;
    }
    hadm_meas_p->iq_buff_size = nb_samples_per_pm * (hadm_meas_p->n_ap + 1U) * (hadm_config_p->stepsNb - hadm_config_p->pnSeqNb);

}

/* Compute HADM step duration in us for all modes */
void lcl_hadm_utils_compute_step_duration(const BLE_HADM_SubeventConfig_t *hadm_config_p, uint32_t n_ap, uint16_t *mode_dur)
{
    mode_dur[1U] = (uint16_t)(hadm_config_p->T_FCS_Time + 2U*HADM_T_SY(hadm_config_p->rttPhy) + 2U*HADM_T_RD + hadm_config_p->T_IP1_Time); /* T_FCS + 2*T_SY + 2*T_RD + T_IP1 */
    mode_dur[0U] = (uint16_t)(mode_dur[1U] + HADM_T_FM + HADM_T_GD); /* T_FCS + 2*T_SY + 2*T_RD + T_GD + T_FM + T_IP1 */
    mode_dur[2U] = (uint16_t)(hadm_config_p->T_FCS_Time + 2U*((uint16_t)hadm_config_p->T_PM_Time + (uint16_t)hadm_config_p->T_SW_Time)*(n_ap + 1U) + 
                              2U*HADM_T_RD + hadm_config_p->T_IP2_Time); /* T_FCS + 2*(T_SW+T_PM)*(N_AP+1) + 2*T_RD + T_IP2 */
    mode_dur[3U] = 0U; // not supported
}

/* For a given CFO, this function computes of much and how often we need to adjust the RSM time grid */
int32_t lcl_hadm_utils_compute_time_adjustement_period(int32_t ppm, hadm_meas_t *hadm_meas_p)
{
    int32_t tim_adj; /* +/- 1 us */
    if (ppm > 0)
      tim_adj = 1;
    else if (ppm < 0)
      tim_adj = -1;
    else // ppm == 0, no adjustment required
      return 0;

    /* Given the measured ppm, compute the time adjustment interval for 0.5us adjustment */
    hadm_meas_p->time_to_adj = (500000U / (ABS(ppm))); /* in us */

#if 0 // UT
    tim_adj = -1;
    hadm_meas_p->time_to_adj = 10U;
#endif
    
    return tim_adj;
}

/* This function processes IQ received during one HADM step in REAL TIME (i.e. while HADM event in on-going) 
   So processing should fit into a T_FCS */
void lcl_hadm_utils_compute_pct_and_tqi(uint32_t num_iq_per_step_per_ap, uint32_t n_ap, uint32_t **out_buf_p, uint16_t **out_quality_buf_p)
{
    int16_t *in_buf_p = (int16_t *)TX_PACKET_RAM_BASE;
    int16_t i, q;
    int32_t i_avg, q_avg;
    uint32_t ap, sample, mag, mag_max, mag_min, mag_avg, tqi;
    
    for(ap = 0; ap < n_ap; ap++)
    {
        i_avg = 0;
        q_avg = 0;
        mag_avg = 0;
        mag_max = 0;
        mag_min = 0xFFFFFFFF;

        /* Compute IQ average and IQ magnitude avg, min and max */
        for (sample = 0; sample < num_iq_per_step_per_ap; sample++)
        {
            i = *in_buf_p++;
            q = *in_buf_p++;
            
            i_avg += i;
            q_avg += q;
            
            mag = MAG(ABS(i), ABS(q));
            mag_avg += mag;
            if (mag > mag_max) mag_max = mag;
            if (mag < mag_min) mag_min = mag;
        }
        i_avg /= (int32_t)num_iq_per_step_per_ap; /* 32bits accumlulation is safe */
        q_avg /= (int32_t)num_iq_per_step_per_ap;
        mag_avg /= (uint32_t)num_iq_per_step_per_ap;
        /* Compute PCT = IQ average */
        *(*out_buf_p)++ = ((q_avg & 0xFFFF) << 16) | (i_avg & 0xFFFF);
        
        /* Compute Tone Quality Indicator metric based on the normalized difference between max and min magnitude values */
        tqi = ((mag_max - mag_min) << 10U) / mag_avg; /* Q2 value in Q10 */
        if (tqi > UINT16_MAX)
        {
            tqi = UINT16_MAX;
        }
        *(*out_quality_buf_p)++ = (uint16_t)tqi;
    }
}

void lcl_hadm_utils_compute_pct(uint32_t num_iq_per_step_per_ap, uint32_t n_ap, uint32_t **out_buf_p, uint16_t **out_quality_buf_p)
{
    int16_t *in_buf_p = (int16_t *)TX_PACKET_RAM_BASE;
    int16_t i, q;
    uint32_t ap, idx;
    
    for(ap = 0; ap < n_ap; ap++)
    {
        idx = num_iq_per_step_per_ap*ap + num_iq_per_step_per_ap/2; // sample in the middle
        i = in_buf_p[2*idx];
        q = in_buf_p[2*idx + 1];
        /* PCT */
        *(*out_buf_p)++ = ((q & 0xFFFF) << 16) | (i & 0xFFFF);
        /* Force TQI = 0 */
        *(*out_quality_buf_p)++ = 0;
    }
}

int8_t lcl_hadm_utils_compute_rpl(uint32_t iq, int8_t rssi)
{
    int32_t rpl;
    int16_t i,q;
    uint32_t mag;
    
    /* Compute IQ magnitude */
    i = ((int16_t)(iq & 0xFFFF)) / 32;
    q = ((int16_t)((iq >> 16U) & 0xFFFF)) / 32;
    mag = MAG(ABS(i), ABS(q));

    /* RPL = RSSI +20log(1024/mag) = RSSI + 60 - 20log(mag) */

    if (mag > 0)
    {
        rpl = rssi + 60 - (int32_t)(20 * log10(mag));
    }
    else
    {
        rpl = HADM_INVALID_REFERENCE_POWER_LEVEL;
    }
    return (int8_t)rpl;
}

uint32_t lcl_hadm_utils_wait_for_rsm_irq(uint32_t *rsm_step_no, uint32_t role)
{
    uint32_t rsm_irq_status;

    DEBUG_PIN1_CLR

    do
    {
        __WFI();
    } while  ((LCL_HAL_RSM_GET_IRQ_STATUS_FLAGS == 0)
              && ((TPM_GetStatusFlags(HADM_TPM) & (1<<HADM_TPM_TIMER_CHANNEL)) == 0)
              );

    DEBUG_PIN1_SET

    rsm_irq_status = LCL_HAL_RSM_GET_IRQ_STATUS_FLAGS; /* Fetch IRQ status */
    *rsm_step_no = LCL_HAL_RSM_GET_CURRENT_STEP;  /* Fetch RSM step */
#ifdef RSM_DEBUG
    rsm_dbg_rsm_csr = XCVR_MISC->RSM_CSR;
    rsm_dbg_tpm_cnt = HADM_TPM->CNT;
#endif

    if (rsm_irq_status != 0)
    {
        /* Clear IRQ status bits which have been read already */
        LCL_HAL_RSM_SET_IRQ_STATUS_FLAGS(rsm_irq_status);

        //FIXME: need to clear NVIC as well? 
        NVIC_ClearPendingIRQ(RSM_INT_IRQn);
    }

    if ((TPM_GetStatusFlags(HADM_TPM) & (1<<HADM_TPM_TIMER_CHANNEL)) != 0)
    {
        rsm_irq_status |= LCL_HAL_TPM_TIMER_IRQ;
        TPM_ClearStatusFlags(HADM_TPM, 1<<HADM_TPM_TIMER_CHANNEL);
        //FIXME: need to clear NVIC as well? 
        NVIC_ClearPendingIRQ(TPM2_INT_IRQn);
    }

    return rsm_irq_status;
}

//#define UT_HADM_ANT_PERM
void lcl_hadm_utils_handle_t_pm_ext(hadm_meas_t *hadm_meas_p, uint8_t step_id, uint32_t irq_flag)
{
    bool_t t_pm_ext;
    if (((hadm_meas_p->config_p->role == HADM_ROLE_INITIATOR) && (irq_flag & LCL_HAL_XCVR_RSM_IRQ_IP)) || 
        ((hadm_meas_p->config_p->role == HADM_ROLE_REFLECTOR) && (irq_flag & LCL_HAL_XCVR_RSM_IRQ_FC)))
    {
        t_pm_ext = 1U; /* Next half step is RX, always enable RX on T_PM ext */
    }
    else
    {
        /* Next half step is TX, check whether we should TX during T_PM ext */
#ifdef UT_HADM_ANT_PERM
        static uint8_t t_pm_ext_present = 1U;
        t_pm_ext = t_pm_ext_present;
        if (irq_flag & LCL_HAL_XCVR_RSM_IRQ_IP)
            t_pm_ext_present ^= 1U;
#else
        /* Keep pm extension bit corresponding to our role */
        t_pm_ext = hadm_meas_p->config_p->chModePmAntMap[step_id].pm_ext;
        if (hadm_meas_p->config_p->role == HADM_ROLE_INITIATOR)
        {
            t_pm_ext >>= 1U;
        }
        t_pm_ext &= 0x1U;
#endif
    }
    /* Patch the last LUT entry according to t_pm_ext */
    lcl_hal_xcvr_configure_t_pm_ext(hadm_meas_p->n_ap, t_pm_ext, hadm_device.ant2gpio[HADM_MAX_NB_ANTENNAS]);
}

void lcl_hadm_utils_handle_antenna_permutation(hadm_meas_t *hadm_meas_p, uint8_t step_id, uint32_t irq_flag)
{
    /* Configure antenna permutation pattern by reconfiguring LCL LUT on the fly... */
    uint8_t ant_gpio[HADM_MAX_NB_ANTENNA_PATHS];
    uint32_t perm_idx = hadm_meas_p->config_p->chModePmAntMap[step_id].ant_perm;
    
#ifdef UT_HADM_ANT_PERM
    if (hadm_meas_p->config_p->toneAntennaConfigIdx == HADM_ANT_CFG_IDX_7)
    {
        perm_idx = step_id % 24;
    }
#endif

    /* 1. Perform antenna permutation index to antenna index mapping */
    /* " Most antenna configurations described below are 1:X or X:1 orientations where X is in the set of 1 to 4. 
     * In these configurations, antenna path A1 is assigned to the 1:1 antenna combination, A2 is assigned to the 1:2 or 2:1 combination, 
     * A3 is assigned to the 1:3 or 3:1 combination and A4 is assigned to the 1:4 or 4:1 combination. 
     * The exception is the 2:2 configuration where A1 is assigned to 1:1, A2 is assigned to 1:2, A3 is assigned to 2:1 and A4 is assigned to 2:2. " */
    /* 2. Perform antenna index to LUT_GPIO mapping */
    /* 3. Re-Configure LUT */

    if ((hadm_meas_p->n_ap == 1U) || (hadm_meas_p->num_ant == 1U)) /* device is not switching antennas */
    {
        ant_gpio[0] = hadm_device.ant2gpio[0];  // Use first antenna from antenna array
        ant_gpio[1] = ant_gpio[2] = ant_gpio[3] = ant_gpio[0];
    }
    else
    {
        const uint8_t *ant_perm_p = hadm_ant_perm_n_ap[perm_idx];
        
        if (hadm_meas_p->config_p->toneAntennaConfigIdx == HADM_ANT_CFG_IDX_7)
        {
            uint32_t role = hadm_meas_p->config_p->role;
            ant_gpio[0] = hadm_device.ant2gpio[hadm_ant_perm_to_idx[role][ant_perm_p[0]]];
            ant_gpio[1] = hadm_device.ant2gpio[hadm_ant_perm_to_idx[role][ant_perm_p[1]]];
            ant_gpio[2] = hadm_device.ant2gpio[hadm_ant_perm_to_idx[role][ant_perm_p[2]]];
            ant_gpio[3] = hadm_device.ant2gpio[hadm_ant_perm_to_idx[role][ant_perm_p[3]]];
        }
        else /* direct map btw antenna permutation index and antenna index */
        {
            ant_gpio[0] = hadm_device.ant2gpio[ant_perm_p[0]];
            ant_gpio[1] = hadm_device.ant2gpio[ant_perm_p[1]];
            ant_gpio[2] = hadm_device.ant2gpio[ant_perm_p[2]];
            ant_gpio[3] = hadm_device.ant2gpio[ant_perm_p[3]];
        }
    }
    lcl_hal_xcvr_configure_ant_switch_lut(hadm_meas_p->n_ap, ant_gpio);
}

uint8_t lcl_hadm_utils_set_CS_SYNC_antenna(hadm_meas_t *hadm_meas_p)
{
    uint32_t ant_id;

    if (hadm_meas_p->config_p->rttAntennaID == HADM_RTT_ANT_NO_RECOMMENDATION)
    {
        ant_id = hadm_ant_perm_to_idx[hadm_meas_p->config_p->role][hadm_meas_p->rtt_antenna_id];
    }
    else
    {
        ant_id = hadm_meas_p->rtt_antenna_id;
    }
    LCL_HAL_SET_ANTENNA_PORT(hadm_device.ant2gpio[ant_id]);
    
    if (hadm_meas_p->config_p->rttAntennaID == HADM_RTT_ANT_NO_RECOMMENDATION)
    {
        /* Custom implementation to cross antennas on both devices */
        hadm_meas_p->rtt_antenna_id = (hadm_meas_p->rtt_antenna_id + 1) % hadm_meas_p->n_ap;
    }
    else if (hadm_meas_p->config_p->rttAntennaID == HADM_RTT_ANT_ROUND_ROBIN)
    {
        hadm_meas_p->rtt_antenna_id = (hadm_meas_p->rtt_antenna_id + 1) % hadm_meas_p->num_ant; /* round robin */
    }
    return (uint8_t)ant_id;
}

uint32_t lcl_hadm_utils_dtest_set_page(uint32_t page)
{
    uint32_t dtest_ctrl = RADIO_CTRL_DTEST_CTRL_DTEST_PAGE(RADIO_CTRL->DTEST_CTRL);
    RADIO_CTRL->DTEST_CTRL = (RADIO_CTRL->DTEST_CTRL & ~RADIO_CTRL_DTEST_CTRL_DTEST_PAGE_MASK ) | RADIO_CTRL_DTEST_CTRL_DTEST_EN(1) | RADIO_CTRL_DTEST_CTRL_DTEST_PAGE(page);
    return dtest_ctrl;
}

/* This function allows to distinguish btw 2 phase populations (phi and phi + PI)
   Q values are not meaningful and are ignored,
   It returns:
     - 0 if I_res > 0
     - 1 if I_res < 0
     - (-1) if inconclusive
*/
#define HADM_I_RESID_THRESHOLD (5)
int8_t lcl_hadm_utils_determine_phase_population(hadm_dc_resid_t *cur)
{
    int8_t det;
    
    if (ABS(cur->i_resid) > HADM_I_RESID_THRESHOLD)
    {
        if (cur->i_resid < 0)
            det = 1; /* polpulation 1 */
        else
            det = 0; /* polpulation 0 */
    }
    else
    {
        det = -1; /* inconclusive measurement */
    }
    return det;
}

#ifdef RSM_DEBUG
void lcl_hadm_utils_debug_init(hadm_meas_t *hadm_meas_p)
{
    rsm_iter = 0;
    hadm_full_info.rtt_dbg_buffer_nb = 0;
#ifdef MCIQ_DEBUG
    hadm_full_info.mciq_dbg_buffer_nb = LCL_HADM_MCIQ_DBG_BUFF_SIZE;
#else
    hadm_full_info.mciq_dbg_buffer_nb = 0;
#endif
}

void lcl_hadm_utils_populate_rtt_dbg_buff(uint16_t timestamp1, uint16_t timestamp2)
{
#ifdef RTT_DEBUG
    int step_no = hadm_full_info.rtt_dbg_buffer_nb;
    if (step_no < LCL_HADM_RTT_DBG_BUFF_SIZE)
    {
        hadm_full_info.rtt_dbg_buffer[step_no].t1 = timestamp1;
        hadm_full_info.rtt_dbg_buffer[step_no].t2 = timestamp2;
        hadm_full_info.rtt_dbg_buffer[step_no].stat0 = XCVR_2P4GHZ_PHY->STAT0;
        hadm_full_info.rtt_dbg_buffer[step_no].hartt_stat = XCVR_2P4GHZ_PHY->RTT_STAT;
        hadm_full_info.rtt_dbg_buffer_nb++;
    }
#endif
}

void lcl_hadm_utils_populate_rsm_dbg_buff(uint32_t step_no, uint32_t hal_irq)
{
    rsm_dbg[rsm_iter].step_no = step_no;
    rsm_dbg[rsm_iter].step_format = ((rsm_dbg_rsm_csr & XCVR_MISC_RSM_CSR_RSM_STEP_FORMAT_MASK) >> XCVR_MISC_RSM_CSR_RSM_STEP_FORMAT_SHIFT);
    rsm_dbg[rsm_iter].state = (t_hadm_rsm_states)((rsm_dbg_rsm_csr & XCVR_MISC_RSM_CSR_RSM_STATE_MASK) >> XCVR_MISC_RSM_CSR_RSM_STATE_SHIFT);
    rsm_dbg[rsm_iter].irq = (uint16_t)hal_irq;
#ifdef RSM_DEBUG_CAL
    rsm_dbg[rsm_iter].ctune = (uint8_t)((XCVR_PLL_DIG->CTUNE_RES&XCVR_PLL_DIG_CTUNE_RES_CTUNE_SELECTED_MASK)>>XCVR_PLL_DIG_CTUNE_RES_CTUNE_SELECTED_SHIFT);
    rsm_dbg[rsm_iter].hpm_cal = (uint16_t)((XCVR_PLL_DIG->HPMCAL_CTRL&XCVR_PLL_DIG_HPMCAL_CTRL_HPM_CAL_FACTOR_MASK)>>XCVR_PLL_DIG_HPMCAL_CTRL_HPM_CAL_FACTOR_SHIFT);
#elif defined (RSM_DEBUG_DMA)
    static uint16_t prev_cnt = 0;
    uint16_t curr_cnt = (uint16_t)(DSB0->XCR >> DSB_XCR_CCNT_SHIFT);
    rsm_dbg[rsm_iter].ccnt = curr_cnt - prev_cnt;
    prev_cnt = curr_cnt;
    rsm_dbg[rsm_iter].ptr = (uint16_t)DSB0->DADDR;
#else
    rsm_dbg[rsm_iter].rssi = lcl_hal_xcvr_get_nb_rssi();
    rsm_dbg[rsm_iter].time = (uint16_t)rsm_dbg_tpm_cnt;
#endif
    if (++rsm_iter == LCL_HADM_RSM_DBG_BUFF_SIZE) rsm_iter = 0;
}
#endif /* RSM_DEBUG */

/* EOF */
