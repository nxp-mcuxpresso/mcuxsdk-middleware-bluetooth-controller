/**
 * @file lcl_hadm_utils.c
 *
 * This file implements some utility functions for HADM
 */
/*
 * Copyright 2024 NXP
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* === Includes ============================================================ */
#include "fsl_os_abstraction.h"
#include "EmbeddedTypes.h"
#include "tx_api.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
#include "fsl_tpm.h"
#include "nxp_xcvr_lcl_ctrl.h"
#include "lcl_hadm_measurement.h"
#include "lcl_hadm_utils.h"
#include "lcl_xcvr_hal.h"
#include "math.h" /* for log() */
#include "fwk_platform_sensors.h"

/* === Types =============================================================== */

/* === Macros ============================================================== */

/* Size of a preallocted HAL result buffer. For now assuming that mode3/n_ap=4 is the largest step report */
#define HADM_HAL_BUFFER_SIZE (HADM_HAL_PKT_RAM_MAX_NB_STEPS_ENGAGED*BLE_HADM_STEP3_REPORT_SIZE(HADM_MAX_NB_ANTENNA_PATHS))

/* === Globals ============================================================= */
//uint32_t nvic_backup;

static BLE_HADM_SubeventConfig_t configBuffer[HADM_MAX_NB_HAL_CONFIG_BUFFERS];
static BLE_HADM_SubeventResultsData_t resultsDataBuffer[HADM_MAX_NB_SIMULT_RESULT_BUFFERS];
static uint8 gpHadmHalResultBuffer[HADM_MAX_NB_SIMULT_RESULT_BUFFERS*HADM_SNIFFER_DEVICE_NB*HADM_HAL_BUFFER_SIZE];

/* Conversion LUT for 2x2: from permutation index to antenna index */
/* "A1 is assigned to 1:1, A2 is assigned to 1:2, A3 is assigned to 2:1 and A4 is assigned to 2:2" */
static const uint8_t hadm_ant_perm_to_idx[2][HADM_MAX_NB_ANTENNA_PATHS] =
{
  {0U,0U,1U,1U}, // Init
  {0U,1U,0U,1U}  // Refl
};

/*! Antenna permutation index: N_AP=4*/
static const uint8_t hadm_ant_perm_to_ap[24][HADM_MAX_NB_ANTENNA_PATHS] =
{
    {0U, 1U, 2U, 3U}, // AP1, AP2, AP3, AP4
    {1U, 0U, 2U, 3U}, // AP2, AP1, AP3, AP4
    {0U, 2U, 1U, 3U}, // AP1, AP3, AP2, AP4
    {2U, 0U, 1U, 3U}, // AP3, AP1, AP2, AP4
    {2U, 1U, 0U, 3U}, // AP3, AP2, AP1, AP4
    {1U, 2U, 0U, 3U}, // AP2, AP3, AP1, AP4
    {0U, 1U, 3U, 2U}, // AP1, AP2, AP4, AP3
    {1U, 0U, 3U, 2U}, // AP2, AP1, AP4, AP3
    {0U, 3U, 1U, 2U}, // AP1, AP4, AP2, AP3
    {3U, 0U, 1U, 2U}, // AP4, AP1, AP2, AP3
    {3U, 1U, 0U, 2U}, // AP4, AP2, AP1, AP3
    {1U, 3U, 0U, 2U}, // AP2, AP4, AP1, AP3
    {0U, 3U, 2U, 1U}, // AP1, AP4, AP3, AP2
    {3U, 0U, 2U, 1U}, // AP4, AP1, AP3, AP2
    {0U, 2U, 3U, 1U}, // AP1, AP3, AP4, AP2
    {2U, 0U, 3U, 1U}, // AP3, AP1, AP4, AP2
    {2U, 3U, 0U, 1U}, // AP3, AP4, AP1, AP2
    {3U, 2U, 0U, 1U}, // AP4, AP3, AP1, AP2
    {3U, 1U, 2U, 0U}, // AP4, AP2, AP3, AP1
    {1U, 3U, 2U, 0U}, // AP2, AP4, AP3, AP1
    {3U, 2U, 1U, 0U}, // AP4, AP3, AP2, AP1
    {2U, 3U, 1U, 0U}, // AP3, AP4, AP2, AP1
    {2U, 1U, 3U, 0U}, // AP3, AP2, AP4, AP1
    {1U, 2U, 3U, 0U}, // AP2, AP3, AP4, AP1
};

static int32_t pct_sin_phase_offset[HADM_MAX_NB_ANTENNAS][HADM_MAX_CHANNELS];
static int32_t pct_cos_phase_offset[HADM_MAX_NB_ANTENNAS][HADM_MAX_CHANNELS];

/* === Externals =========================================================== */

/* === Prototypes ========================================================== */

/* === Implementation ====================================================== */

void lcl_hadm_utils_init_buffers(void)
{
    /* Initialise HAL config & results buffer */
    for (uint32_t i=0U; i < HADM_MAX_NB_HAL_CONFIG_BUFFERS; i++)
    {
        configBuffer[i].configBufferUsed = 0;
    }
    for (uint32_t i=0U; i < HADM_MAX_NB_SIMULT_RESULT_BUFFERS; i++)
    {
        resultsDataBuffer[i].resultBufferUsed = 0;
        resultsDataBuffer[i].resultBuffer = &gpHadmHalResultBuffer[(uint32_t)i*HADM_SNIFFER_DEVICE_NB*HADM_HAL_BUFFER_SIZE];
        resultsDataBuffer[i].resultBuffer2 = &gpHadmHalResultBuffer[((uint32_t)i*HADM_SNIFFER_DEVICE_NB+1U)*HADM_HAL_BUFFER_SIZE];
        resultsDataBuffer[i].resultBufferSize = HADM_HAL_BUFFER_SIZE;
    }
}

/* Allocate one HAL config buffer */
BLE_HADM_SubeventConfig_t *lcl_hadm_utils_get_config_buffer(void)
{
    BLE_HADM_SubeventConfig_t *config_p = NULL;
    
    for (uint32_t i=0U; i < HADM_MAX_NB_HAL_CONFIG_BUFFERS; i++)
    {
        if (configBuffer[i].configBufferUsed == 0U)
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

/* Free HAL config buffer(s) belonging to connIdx */
void lcl_hadm_utils_free_config_buffer(uint8 connIdx)
{
    for (uint32_t i=0U; i < HADM_MAX_NB_HAL_CONFIG_BUFFERS; i++)
    {
        if ((configBuffer[i].configBufferUsed == 1U) && (configBuffer[i].connIdx == connIdx))
        {
            configBuffer[i].configBufferUsed = 0;
        }
    }
}

/* Allocate one HAL result buffer */
BLE_HADM_SubeventResultsData_t *lcl_hadm_utils_get_result_buffer(void)
{
    BLE_HADM_SubeventResultsData_t *result_p = NULL;
    
    for (uint32_t i=0U; i < HADM_MAX_NB_SIMULT_RESULT_BUFFERS; i++)
    {
        if (resultsDataBuffer[i].resultBufferUsed == 0U)
        {
            result_p = &resultsDataBuffer[i];
            result_p->resultBufferUsed = 1;
            result_p->debugBuffer = NULL;
            result_p->debugBufferSize = 0;
            result_p->nbStepsCollected = 0;
            result_p->firstStepCollected = 0;

            /* Do some inits */
            result_p->referencePwrLevel = HADM_INVALID_REFERENCE_POWER_LEVEL;
            result_p->referencePwrLevel2 = HADM_INVALID_REFERENCE_POWER_LEVEL;
            result_p->frequencyCompensation = (int16)0xC000;
            result_p->syncDelayUs = 0;
            break;
        }
    }
    return result_p;
}

/* Free HAL config buffer(s) belonging to connIdx */
void lcl_hadm_utils_free_result_buffer(uint8 connIdx)
{
    for (uint32_t i=0U; i < HADM_MAX_NB_SIMULT_RESULT_BUFFERS; i++)
    {
        if ((resultsDataBuffer[i].resultBufferUsed == 1U) && (resultsDataBuffer[i].connIdx == connIdx))
        {
            resultsDataBuffer[i].resultBufferUsed = 0U;
        }
    }
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
 * Curve is positioned on Y axis (hence the -1) so that there's no delay at 25 degrees C which is the recommended
 * temperature to perform zero distance calibration.
 * This will also garantie normal behavior at 25 degrees C if the host does not transmit temperature information (hcibb).
 * For 1MBPS: delay (half ns) = 0.0014x^2 + 0.1436x – 4.6426
 * For 2MBPS: delay (half ns) = 0.0014x^2 + 0.1428x – 8.1838
 * Fixed point conversions:
 *      0.0014 => 22.9*2^14
 *      0.1436 => 2352.7*2^14
 *      0.1428 => 2339.6*2^14
 * Return: half ns unit
 */
#define HADM_RTT_SCALE_FACTOR      (0x4000)     /* (1<<14) */
#define HADM_RTT_SCALE_FACTOR_HALF (HADM_RTT_SCALE_FACTOR / 2)
#define HADM_RTT_SCALE_FACTOR_4    (0x10)       /* (1<<(5-1)) */
#define HADM_CALC_RTT_TEMP_DELAY_1MBPS(_TEMP) ((((23 * (_TEMP) + 2353) * (_TEMP)) / HADM_RTT_SCALE_FACTOR) - 5)
#define HADM_CALC_RTT_TEMP_DELAY_2MBPS(_TEMP) ((((23 * (_TEMP) + 2340) * (_TEMP)) / HADM_RTT_SCALE_FACTOR) - 8)

void lcl_hadm_utils_calc_rtt_temperature_delay(int32_t temperature, hadm_device_t *hadm_device_p)
{
    hadm_device_p->rtt_temperature_comp_hns[HADM_RTT_PHY_1MBPS] = HADM_CALC_RTT_TEMP_DELAY_1MBPS(temperature);
    hadm_device_p->rtt_temperature_comp_hns[HADM_RTT_PHY_2MBPS] = HADM_CALC_RTT_TEMP_DELAY_2MBPS(temperature);
}

/* Compute device-specific constant contributions to RTT delay */
void lcl_hadm_utils_calc_rtt_static_delay(hadm_device_t *hadm_device_p)
{
    int32_t static_delay_hns;

    /* 1Mbps*/
    /* RCCal delay in hns = -0.774*(rccal-17) ns * 2 (0.774 in fixed point s14 = 12681) */
    static_delay_hns = (((int32_t)hadm_device_p->rtt_static_comp.rttRCcal - HADM_RCCAL_CENTER) * (-12681)) / HADM_RTT_SCALE_FACTOR_HALF;

    /* CBPF attenuation delay in ns = 14.86(a-6.5). Since a is stored *100 => in hns = 2*14.86(a-6.5)/100. (14.86 in fixed point s5 = 475) */
    hadm_device_p->rtt_static_comp_hns[HADM_RTT_PHY_1MBPS] = static_delay_hns +
       ((((int32_t)hadm_device_p->rtt_static_comp.rttCbpfAtt[HADM_RTT_PHY_1MBPS] - HADM_CBPF_ATTEN_CENTER_1MBPS) * 475) / (100*HADM_RTT_SCALE_FACTOR_4));

    /* 2Mbps*/
    /* RCCal delay in hns = -0.339*(rccal-17) ns * 2 (0.339 in fixed point s14 = 5554) */
    static_delay_hns = (((int32_t)hadm_device_p->rtt_static_comp.rttRCcal - HADM_RCCAL_CENTER) * (-5554)) / HADM_RTT_SCALE_FACTOR_HALF;

     /* CBPF attenuation delay in ns = 16.015(a-9). Since a is stored *100 => in hns = 2*16.015(a-9)/100. (16.015 in fixed point s5 = 512) */
    hadm_device_p->rtt_static_comp_hns[HADM_RTT_PHY_2MBPS] = static_delay_hns +
       ((((int32_t)hadm_device_p->rtt_static_comp.rttCbpfAtt[HADM_RTT_PHY_2MBPS] - HADM_CBPF_ATTEN_CENTER_2MBPS) * 512) / (100*HADM_RTT_SCALE_FACTOR_4));
}

/* Compute latency that has to be removed from ToA-ToD values (resp. substracted from ToD-ToA) :
 *      - nominal offset
 *      - HW contribution (HW TPM triggers vs 1st preamble bit reference)
 * Return: half ns unit
 */
void lcl_hadm_utils_calc_ts_delay(hadm_meas_t *hadm_meas_p, hadm_device_t *hadm_device_p)
{
    const BLE_HADM_SubeventConfig_t *hadm_config_p = hadm_meas_p->config_p;
    uint32_t ts_nominal_delay;
    int32_t ts_hw_delay;
    uint16_t aa_match_dur;

    assert(hadm_config_p->rttPhy < HADM_RTT_PHY_MAX);

    /* When reporting round-trip time to the Host, each device excludes nominal known time offsets from the reported time. This time is equivalent to the time period
     * between the reception and transmission of the center of the CS_SYNC fields, T_SY_CENTER_DELTA */
    if (hadm_meas_p->config_p->rttMode == HADM_STEP_MODE3)
    {
        /* T_SY_CENTER_DELTA = TSY + TRD + 2 x TGD + 2 x (TSW + TPM ) x NAP + (TSW + TPM ) + TIP2 if *no* physical transmission is present in the reflector to initiator transmission extension slot. */
        ts_nominal_delay = (uint32_t)HADM_T_SY(hadm_config_p->rttPhy) + HADM_T_RD + (2U * HADM_T_GD)
                           + 2U * (((uint32_t)hadm_config_p->T_PM_Time + (uint32_t)hadm_config_p->T_SW_Time) * hadm_meas_p->n_ap)
                           + ((uint32_t)hadm_config_p->T_PM_Time + (uint32_t)hadm_config_p->T_SW_Time) + (uint32_t)hadm_config_p->T_IP2_Time;
        
        hadm_meas_p->ts_extra_delay_hns = ((uint32_t)hadm_meas_p->config_p->T_PM_Time + (uint32_t)hadm_meas_p->config_p->T_SW_Time) * 2000U;
    }
    else
    {
        /* T_SY_CENTER_DELTA = TSY + TRD + TIP1 */
        ts_nominal_delay = (uint32_t)HADM_T_SY(hadm_config_p->rttPhy) + (uint32_t)HADM_T_RD + (uint32_t)hadm_config_p->T_IP1_Time;
        hadm_meas_p->ts_extra_delay_hns = 0;
    }
    /* Take payload into account */
    ts_nominal_delay += (((uint32_t)rtt_type_2_payload_size[hadm_config_p->rttTypes] * 32U) >> ((uint8_t)hadm_config_p->rttPhy));
    
    /* Convert us to half ns */
    ts_nominal_delay *= 2000U;

    /* Coarse HW contribution:
     * TX: elapsed time between tx_dig_en (TPM trigger on TX) and 1st bit over the air
     * RX: elapsed time between last bit of AA and aa_match_to_ll (TPM trigger on RX) + duration of preamble and AA
     */
    aa_match_dur = HADM_1ST_BIT_TO_AA_MATCH_DURATION_US(hadm_config_p->rttPhy);
    ts_hw_delay = (int32_t)HADM_TX_LATENCY_NS + (int32_t)aa_match_dur * (int32_t)1000;
    /* Convert to half ns */
    ts_hw_delay *= 2;

    /* Add per-device static compensation */
    ts_hw_delay += hadm_device_p->rtt_static_comp_hns[hadm_config_p->rttPhy];

    /* Finer HW compensation based on characterization (EVK boards) */
    ts_hw_delay += (hadm_config_p->rttPhy == HADM_RTT_PHY_1MBPS) ? HADM_RXTX_FINE_LATENCY_1MBPS_HNS : HADM_RXTX_FINE_LATENCY_2MBPS_HNS;

    /* Perform fine tuning correction (only in normal operation, skipped during distance calibration) */
    if (hadm_config_p->distanceCalMode == HADM_DIST_CAL_MODE_DISABLED)
    {
        ts_hw_delay += hadm_device_p->zero_distance_comp.rttFineTuningHns[hadm_config_p->rttPhy];
    }

    /* Latency due to temperature */
    if (hadm_config_p->distanceCalMode == HADM_DIST_CAL_MODE_DISABLED)
    {
        ts_hw_delay += hadm_device_p->rtt_temperature_comp_hns[hadm_config_p->rttPhy];
    }

    if (hadm_config_p->role == HADM_ROLE_REFLECTOR)
    {
        hadm_meas_p->ts_hw_delay_hns = (uint32_t)(-ts_hw_delay);
    }
    else
    {
        hadm_meas_p->ts_hw_delay_hns = (uint32_t)ts_hw_delay;
    }

    hadm_meas_p->ts_nominal_delay_hns = ts_nominal_delay;
    assert(hadm_meas_p->ts_nominal_delay_hns + hadm_meas_p->ts_hw_delay_hns > 0);
}

void lcl_hadm_enable_lcl_interrupts(void)
{
    NVIC_ClearPendingIRQ(RSM_INT_IRQn);
    (void)EnableIRQ(RSM_INT_IRQn);
    NVIC_SetPriority(RSM_INT_IRQn, 1);
}

#ifdef HADM_CFO_COMP_PER_STEP_VIA_FOM
void lcl_hadm_enable_interrupts_for_subevent(bool trig_on_tx)
{
    NVIC_ClearPendingIRQ(BRF_INT_IRQn);
    (void)EnableIRQ(BRF_INT_IRQn);
    NVIC_SetPriority(BRF_INT_IRQn, 0);
    XCVR_TSM->CTRL |= XCVR_TSM_CTRL_TSM_IRQ0_EN_MASK;
    if (trig_on_tx)
    {
        XCVR_TSM->TIMING03 &= ~(XCVR_TSM_TIMING03_IRQ0_START_TRIG_TX_HI_MASK | XCVR_TSM_TIMING03_IRQ0_START_TRIG_TX_LO_MASK);
        XCVR_TSM->TIMING03 |= XCVR_TSM_TIMING03_IRQ0_START_TRIG_TX_HI(26) | XCVR_TSM_TIMING03_IRQ0_START_TRIG_TX_LO(27);
    }
    else
    {
        XCVR_TSM->TIMING03 &= ~(XCVR_TSM_TIMING03_IRQ0_START_TRIG_RX_HI_MASK | XCVR_TSM_TIMING03_IRQ0_START_TRIG_RX_LO_MASK);
        XCVR_TSM->TIMING03 |= XCVR_TSM_TIMING03_IRQ0_START_TRIG_RX_HI(26) | XCVR_TSM_TIMING03_IRQ0_START_TRIG_RX_LO(27);
    }
}

void lcl_hadm_restore_interrupts_for_subevent(void)
{
    (void)DisableIRQ(BRF_INT_IRQn);
    XCVR_TSM->CTRL &= ~XCVR_TSM_CTRL_TSM_IRQ0_EN_MASK;
    XCVR_TSM->TIMING03 |= (XCVR_TSM_TIMING03_IRQ0_START_TRIG_TX_HI_MASK | XCVR_TSM_TIMING03_IRQ0_START_TRIG_TX_LO_MASK |
                           XCVR_TSM_TIMING03_IRQ0_START_TRIG_RX_HI_MASK | XCVR_TSM_TIMING03_IRQ0_START_TRIG_RX_LO_MASK);
}

/* Programm FOM register values.
 * Those values do not have to be reset after use since they are not relevant
 * without a FOM trigger (cleared via LCL_HAL_DISABLE_FO_ENTRY).
 * Return  0/1 for success/error correspondingly.
 */
uint8_t lcl_hadm_apply_cfo_per_step(int32_t cfo)
{
    uint32_t temp = (cfo >= 0) ? (uint32_t)cfo : (uint32_t)(-cfo);

    if ((cfo > 262143) || (cfo < -262143))  /* check CFO limitation: (2^32 / 16384) - 1 */
    {
        return 1U;
    }
    /* Hz to 0.95Hz unit, see XCVR_LCL_RsmCompCfo() */
    temp = ((temp * 16384U) / 15625U);
    cfo = (cfo >= 0) ? (int32_t)temp : (-(int32_t)temp);
    /* 1Mbps PHY */
    XCVR_MISC->IPS_FO_DRS0_DATA[HADM_FO_ENTRY] = XCVR_PLL_DIG_PLL_OFFSET_CTRL_PLL_NUMERATOR_OFFSET(cfo);
    /* 2 Mbps PHY */
    XCVR_MISC->IPS_FO_DRS1_DATA[HADM_FO_ENTRY] = XCVR_PLL_DIG_PLL_OFFSET_CTRL_PLL_NUMERATOR_OFFSET(cfo);

    return 0U;
}
#endif /* HADM_CFO_COMP_PER_STEP_VIA_FOM */

#if 0 /* not used anymore */
/* Disable all interrupts for NVIC except RSM IRQ and global interrupt enable */
/* Note that TPM used for ToF does not need need IRQ */
void lcl_hadm_disable_interrupts(void)
{
    OSA_DisableIRQGlobal();
    nvic_backup = NVIC->ICER[0];
    NVIC->ICER[0] = 0xffffffff; // disable all interrupts
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
    TPM_SetupOutputCompare(HADM_TPM, HADM_TPM_TIMER_CHANNEL, kTPM_NoOutputSignal, HADM_TPM_MODULO); // software compare
    HADM_TPM->CONTROLS[HADM_TPM_TIMER_CHANNEL].CnV = HADM_TPM_MODULO;
    TPM_SetTimerPeriod(HADM_TPM, HADM_TPM_MODULO);
}

void lcl_hadm_start_tpms(void)
{    
    HADM_TPM->CNT = 0;
    TPM_ClearStatusFlags(HADM_TPM, (1<<HADM_TPM_TIMER_CHANNEL));
    TPM_StartTimer(HADM_TPM, kTPM_SystemClock);
}

void lcl_hadm_stop_tpms(void)
{
    TPM_StopTimer(HADM_TPM);
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
#endif

void lcl_hadm_utils_compute_iq_buff_size(const BLE_HADM_SubeventConfig_t *hadm_config_p, hadm_meas_t *hadm_meas_p, uint32_t sample_rate)
{
    uint32_t nb_samples_per_pm;

    if (hadm_meas_p->iq_avg_win > 0U)
    {
        assert((hadm_meas_p->iq_capture_win * sample_rate) >= (1 << hadm_meas_p->iq_avg_win));
        assert(((hadm_meas_p->iq_capture_win * sample_rate) &  ((1 << hadm_meas_p->iq_avg_win) - 1)) == 0);
        assert((hadm_meas_p->iq_capture_win * sample_rate) >= (1 << hadm_meas_p->iq_avg_win));
        assert(((hadm_meas_p->iq_capture_win * sample_rate) &  ((1 << hadm_meas_p->iq_avg_win) - 1)) == 0);
    }

    nb_samples_per_pm = HADM_NUM_IQ_PER_US((uint32_t)hadm_meas_p->iq_capture_win, (uint32_t)hadm_config_p->rttPhy, hadm_meas_p->iq_avg_win);
    if (hadm_config_p->role == HADM_ROLE_INITIATOR)
    {
        hadm_meas_p->iq_buff_size_mode0 = (uint16_t)(nb_samples_per_pm * hadm_config_p->mode0Nb);
    }
    else
    {
       hadm_meas_p->iq_buff_size_mode0 = 0;
    }
    hadm_meas_p->iq_buff_size = (uint16_t)(nb_samples_per_pm * ((uint32_t)hadm_meas_p->n_ap + 1U) * (hadm_config_p->stepsNb - hadm_config_p->mode0Nb));
}

/* Compute HADM step duration in us for all modes */
void lcl_hadm_utils_compute_step_duration(const BLE_HADM_SubeventConfig_t *hadm_config_p, uint32_t n_ap, uint16_t *mode_dur)
{
    mode_dur[0U] = (uint16_t)((uint32_t)hadm_config_p->T_FCS_Time + (uint32_t)2U*(uint32_t)HADM_T_SY(hadm_config_p->rttPhy) + (uint32_t)2U*(uint32_t)HADM_T_RD + (uint32_t)hadm_config_p->T_IP1_Time + (uint32_t)HADM_T_FM + (uint32_t)HADM_T_GD); /* T_FCS + 2*T_SY + 2*T_RD + T_IP1 + T_GD + T_FM */
    /* not used */
    mode_dur[1U] = 0;
    mode_dur[2U] = 0;
    mode_dur[3U] = 0;
}

#define INIT_ACI_SINGLE_ANT (1U<<6U | 1U<<5U | 1U<<4U | 1U<<0U) /* ACI of 6,5,4,0 all have single antenna for INIT role */
#define REFL_ACI_SINGLE_ANT (1U<<3U | 1U<<2U | 1U<<1U | 1U<<0U) /* ACI of 3,2,1,0 all have single antenna for REFL role */
static bool is_single_antenna_config(hadm_meas_t *hadm_meas_p)
{
  bool single_ant = false;
  uint8_t aci_bit = 1U<<(uint8_t)(hadm_meas_p->config_p->toneAntennaConfigIdx);
  /* ACI bits have role specific cases where a single antenna is needed */
  if (hadm_meas_p->config_p->role == HADM_ROLE_INITIATOR)
  {
    single_ant = ((INIT_ACI_SINGLE_ANT & aci_bit) == aci_bit);
  }
  else
  {
    single_ant = ((REFL_ACI_SINGLE_ANT & aci_bit) == aci_bit);
  }
  
  return single_ant;
}

void lcl_hadm_utils_configure_antenna_switching(hadm_meas_t *hadm_meas_p, bool ena_antsw_pa_ramping)
{
    uint8_t default_antenna_idx = 0U; /* index of default antenna for 1:1 (A1) */
    xcvrLclStatus_t status;

    /* Perform antenna permutation index to antenna index mapping */
    /* " Most antenna configurations described below are 1:X or X:1 orientations where X is in the set of 1 to 4.
     * In these configurations, antenna path AP1 is assigned to the 1:1 antenna combination, AP2 is assigned to the 1:2 or 2:1 combination,
     * AP3 is assigned to the 1:3 or 3:1 combination and AP4 is assigned to the 1:4 or 4:1 combination.
     * The exception is the 2:2 configuration where AP1 is assigned to 1:1, AP2 is assigned to 1:2, AP3 is assigned to 2:1 and AP4 is assigned to 2:2. " */
    if (is_single_antenna_config(hadm_meas_p))
    {
        uint8_t ant_gpio[HADM_MAX_NB_ANTENNA_PATHS];
        ant_gpio[0] = hadm_device.ant2gpio[default_antenna_idx];
        ant_gpio[1] = hadm_device.ant2gpio[default_antenna_idx];
        ant_gpio[2] = hadm_device.ant2gpio[default_antenna_idx];
        ant_gpio[3] = hadm_device.ant2gpio[default_antenna_idx];
        status = XCVR_LCL_ConfigLclBlock(&hadm_meas_p->rsm_config, (XCVR_RSM_T_CAPTURE_SEL_T)hadm_meas_p->config_p->T_PM_Time, ant_gpio, ena_antsw_pa_ramping);
    }
    else
    if (hadm_meas_p->config_p->toneAntennaConfigIdx == HADM_ANT_CFG_IDX_7)
    {
        uint8_t ant_gpio[HADM_MAX_NB_ANTENNA_PATHS];
        uint32_t role = (uint32_t)hadm_meas_p->config_p->role;
        ant_gpio[0] = hadm_device.ant2gpio[hadm_ant_perm_to_idx[role][0]];
        ant_gpio[1] = hadm_device.ant2gpio[hadm_ant_perm_to_idx[role][1]];
        ant_gpio[2] = hadm_device.ant2gpio[hadm_ant_perm_to_idx[role][2]];
        ant_gpio[3] = hadm_device.ant2gpio[hadm_ant_perm_to_idx[role][3]];
        status = XCVR_LCL_ConfigLclBlock(&hadm_meas_p->rsm_config, (XCVR_RSM_T_CAPTURE_SEL_T)hadm_meas_p->config_p->T_PM_Time, ant_gpio, ena_antsw_pa_ramping);
    }
    else /* direct map btw antenna path and antenna index */
    {
        status = XCVR_LCL_ConfigLclBlock(&hadm_meas_p->rsm_config, (XCVR_RSM_T_CAPTURE_SEL_T)hadm_meas_p->config_p->T_PM_Time, hadm_device.ant2gpio, ena_antsw_pa_ramping);
    }
    assert(status == gXcvrLclStatusSuccess);
    (void)status;
}

uint8_t lcl_hadm_utils_get_CS_SYNC_antenna(hadm_meas_t *hadm_meas_p)
{
    uint32_t ant_id; /* to be used for this CS step */

    if (hadm_meas_p->config_p->rttAntennaID == HADM_RTT_ANT_NO_RECOMMENDATION)
    {
        /* Custom implementation to cross antennas on both devices */
        ant_id = hadm_ant_perm_to_idx[hadm_meas_p->config_p->role][hadm_meas_p->rtt_antenna_id];
        /* Compute next */
        hadm_meas_p->rtt_antenna_id = (hadm_meas_p->rtt_antenna_id + 1U) % hadm_meas_p->n_ap;
    }
    else
    {
        ant_id = hadm_meas_p->rtt_antenna_id;
        if (hadm_meas_p->config_p->rttAntennaID == HADM_RTT_ANT_ROUND_ROBIN)
        {
            /* Compute next */
            hadm_meas_p->rtt_antenna_id = (hadm_meas_p->rtt_antenna_id + 1U) % hadm_meas_p->num_ant; /* round robin */
        }
    }
    return (uint8_t)ant_id;
}

#define PCT_FIXED_POINT_UNIT  14U
#define PCT_FIXED_POINT_ONE   0x4000U   /* unsigned 1 << PCT_FIXED_POINT_UNIT */

void lcl_hadm_measurement_phase_rotation(uint32_t *iq, uint8_t ch, uint8_t ant_id)
{
    int32_t i_out, q_out;
    uint32_t i = (*iq & 0x000fffU);
    uint32_t q = (*iq & 0xfff000U) >> 12U;

    /* convert i & q from s12.0 format to int32_t. Still do in uint32_t */
    i = i | (((i & 0x800U) != 0U) ? 0xfffff000U : 0U);
    q = q | (((q & 0x800U) != 0U) ? 0xfffff000U : 0U);
    /* phase offset in fixed point 18.14 */
    i_out = (((int32_t)i * pct_cos_phase_offset[ant_id][ch]) - ((int32_t)q * pct_sin_phase_offset[ant_id][ch])) / (int32_t)PCT_FIXED_POINT_ONE;
    q_out = (((int32_t)q * pct_cos_phase_offset[ant_id][ch]) + ((int32_t)i * pct_sin_phase_offset[ant_id][ch])) / (int32_t)PCT_FIXED_POINT_ONE;

    *iq = ((uint32_t)i_out & 0xfffU) | (((uint32_t)q_out & 0xfffU) << 12U);
}

/**
 * Initialize phase offset to default values.
 * Phase rotation happens but no rotation is applied.
 */
void lcl_hadm_init_phase_offset(void)
{
    for (uint32_t ant = 0U; ant < HADM_MAX_NB_ANTENNAS; ant++)
    {
        for (uint32_t ch = 0U; ch < HADM_MAX_CHANNELS; ch++) {
            pct_cos_phase_offset[ant][ch] = (int32_t)PCT_FIXED_POINT_ONE;
            pct_sin_phase_offset[ant][ch] = 0;
        }
    }
}

#define PI_4_FIXED_POINT      12868U  /* PI/4    * 2^14 */
#define PI_2_FIXED_POINT      25736U  /* PI/2    * 2^14 */
#define PI_FIXED_POINT        51472U  /* PI      * 2^14 */
#define TWO_PI_FIXED_POINT    102943U /* 2*PI    * 2^14 */

#define FACT_2_FIXED_POINT    8192U   /* 1/2    * 2^14 */
#define FACT_3_FIXED_POINT    2730U   /* 1/6    * 2^14 */
#define FACT_4_FIXED_POINT    682U    /* 1/24   * 2^14 */
#define FACT_5_FIXED_POINT    136U    /* 1/120  * 2^14  */
#define FACT_6_FIXED_POINT    23U     /* 1/720  * 2^14 */

/* Note: input and output should be in fixed point 18.14 format */
static uint32_t lcl_utils_sqrt_fixed_point(uint32_t x)
{
    uint32_t temp, guess;
    uint8_t msb_pos;

    /* x must be positive and not bigger than 1<<14 */
    if ((x == 0U) || (x > PCT_FIXED_POINT_ONE)) { return 0U; }
    /* sqrt(1) = 1 */
    if (x == PCT_FIXED_POINT_ONE) { return PCT_FIXED_POINT_ONE; }

    /* Find the position of the most significant bit */
    temp = x;
    msb_pos = 0U;
    while (temp > 1U)
    {
        temp >>= 1U;
        msb_pos++;
    }

    /* Initial guess based on bit position */
    guess = (uint32_t)1U << (msb_pos >> 1U);
    if ((msb_pos & 1U) != 0U)
    {
        /* Adjust for odd bit positions */
        guess += (guess >> 1U);
    }

    /* Ensure guess is in proper fixed-point range */
    if (guess < (1U << (PCT_FIXED_POINT_UNIT / 2U)))
    {
        /* Minimum reasonable guess */
        guess = (1U << (PCT_FIXED_POINT_UNIT / 2U));
    }

    /* Heron's iterations */
    for (int i = 0; i < 8; i++)
    {
        uint32_t quotient, prev_guess;
        int32_t diff;

        prev_guess = guess;

        /* Calculate x/guess. x is smaller than 1<<14 then left shift by 14 won't overflow uint32_t */
        quotient = (x << PCT_FIXED_POINT_UNIT) / guess;
        guess = (guess + quotient) >> 1U;

        /* Check for convergence */
        diff = (int32_t)(uint32_t)(guess - prev_guess);
        if (diff < 0)
        {
            diff = -diff;
        }
        if (diff < 2) { break; } /* Converged */
    }
    return guess;
}

/* Returned range [0, PI_2_FIXED_POINT] */
static uint32_t lcl_utils_normalize_angle(int32_t angle, int32_t *cos_sign, int32_t *sin_sign)
{
    uint32_t angle_norm;
    *cos_sign = 1;
    *sin_sign = 1;

    /* Normalize angle to [0, 2*PI] */
    while (angle >= (int32_t)TWO_PI_FIXED_POINT) { angle -= (int32_t)TWO_PI_FIXED_POINT; }
    while (angle < 0) { angle += (int32_t)TWO_PI_FIXED_POINT; }

    angle_norm = (uint32_t)angle;
    /* Use symmetry to reduce to [0, PI/2] */
    if (angle_norm > PI_FIXED_POINT) {
        angle_norm = TWO_PI_FIXED_POINT - angle_norm;
        *sin_sign = -1;
    }
    if (angle_norm > PI_2_FIXED_POINT) {
        angle_norm = PI_FIXED_POINT - angle_norm;
        *cos_sign = -1;
    }
    return angle_norm;
}

/* Note: input and output should be in fixed point 18.14 */
static uint32_t lcl_utils_compute_sine_fixed_point(uint32_t angle)
{
    uint32_t x, x2, x3, x5;

    if (angle > PI_4_FIXED_POINT) { return 0U; }

    /* Taylor series: sin(x) = x - x^3/3! + x^5/5! - x^7/7! */
    x = angle;
    x2 = (x * x)   >> PCT_FIXED_POINT_UNIT;
    x3 = (x2 * x)  >> PCT_FIXED_POINT_UNIT;
    x5 = (x3 * x2) >> PCT_FIXED_POINT_UNIT;

    return (x - ((x3 * FACT_3_FIXED_POINT) >> PCT_FIXED_POINT_UNIT) + ((x5 * FACT_5_FIXED_POINT >> PCT_FIXED_POINT_UNIT)));
}

static uint32_t lcl_utils_compute_cosine_fixed_point(uint32_t angle)
{
    uint32_t x, x2, x4, x6;

    assert(angle <= PI_4_FIXED_POINT);
    if (angle > PI_4_FIXED_POINT) { return 0U; }
   
    /* Taylor series: cos(x) = 1 - x^2/2! + x^4/4! -x^6/6! */
    x = angle;
    x2 = (x * x)   >> PCT_FIXED_POINT_UNIT;
    x4 = (x2 * x2) >> PCT_FIXED_POINT_UNIT;
    x6 = (x4 * x2) >> PCT_FIXED_POINT_UNIT;

    return (PCT_FIXED_POINT_ONE - ((x2 * FACT_2_FIXED_POINT) >> PCT_FIXED_POINT_UNIT) + ((x4 * FACT_4_FIXED_POINT) >> PCT_FIXED_POINT_UNIT) - ((x6 * FACT_6_FIXED_POINT) >> PCT_FIXED_POINT_UNIT));
}

void lcl_hadm_utils_calc_phase_rotation_offset(BLE_HADM_PCTPhaseRotation_t *phaseRotation)
{ 
    int32_t angle_fp;
    int32_t cos_sign, sin_sign;
    uint32_t norm_angle_fp, sine;

    for (uint32_t ant_id=0U; ant_id<HADM_MAX_NB_ANTENNAS; ant_id++)
    {
        angle_fp = phaseRotation->offset_table[ant_id];
        if(angle_fp != 0)
        {
            for (uint32_t chn=0U; chn<HADM_MAX_CHANNELS; chn++)
            {
                norm_angle_fp = lcl_utils_normalize_angle(angle_fp * (int32_t)chn, &cos_sign, &sin_sign);
                if (norm_angle_fp < PI_4_FIXED_POINT)
                {
                    sine = lcl_utils_compute_sine_fixed_point(norm_angle_fp);
                }
                else
                {
                    sine = lcl_utils_compute_cosine_fixed_point(PI_2_FIXED_POINT - norm_angle_fp);
                }
                pct_sin_phase_offset[ant_id][chn] = sin_sign * (int32_t)sine;
                pct_cos_phase_offset[ant_id][chn] = cos_sign * (int32_t)lcl_utils_sqrt_fixed_point((((uint32_t)1<<(2U*PCT_FIXED_POINT_UNIT)) - (sine*sine)) >> (uint32_t)PCT_FIXED_POINT_UNIT);
            }
        }
    }
}

void lcl_hadm_utils_get_antenna_id_sequence(hadm_meas_t *hadm_meas_p, uint8_t step_id, uint8_t *ant_id_seq)
{
    
    if (is_single_antenna_config(hadm_meas_p))
    {
        /* A single antenna configuration requires no permutation, only antenna_id 0 is used */
        (void)memset(ant_id_seq, 0, HADM_MAX_NB_ANTENNAS);
    }
    else
    {
        /* Perform antenna permutation index to antenna index mapping */
        uint8_t role = (uint8_t)hadm_meas_p->config_p->role;
        uint8_t perm_idx = (uint8_t)hadm_meas_p->config_p->chModePmAntMap[step_id].ant_perm;

        if (hadm_meas_p->config_p->toneAntennaConfigIdx == HADM_ANT_CFG_IDX_7) 
        {
            for (uint32_t i = 0U; i < HADM_MAX_NB_ANTENNAS; i++)
            {
                /* Map antenna paths for 2:2 configuration */
                ant_id_seq[i] = hadm_ant_perm_to_idx[role][hadm_ant_perm_to_ap[perm_idx][i]];
            }
        }
        else
        {
            /* Direct mapping btw antenna path and antenna id*/
            (void)memcpy(ant_id_seq, hadm_ant_perm_to_ap[perm_idx], HADM_MAX_NB_ANTENNAS);
        }
    }
}
/* EOF */
