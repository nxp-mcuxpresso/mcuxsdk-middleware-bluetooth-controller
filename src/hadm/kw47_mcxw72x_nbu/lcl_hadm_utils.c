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

/* === Globals ============================================================= */
uint32_t nvic_backup;

BLE_HADM_SubeventConfig_t  configBuffer[HADM_MAX_NB_SIMULT_SUBEVENTS];
BLE_HADM_SubeventResultsData_t resultsDataBuffer[HADM_MAX_NB_SIMULT_SUBEVENTS];
static uint8 gpHadmHalResultBuffer[HADM_MAX_NB_SIMULT_SUBEVENTS*HADM_HAL_PKT_RAM_MAX_NB_STEPS_ENGAGED*BLE_HADM_STEP3_REPORT_SIZE(HADM_MAX_NB_ANTENNA_PATHS)];

/* Conversion LUT for 2x2: from permutation index to antenna index */
/* "A1 is assigned to 1:1, A2 is assigned to 1:2, A3 is assigned to 2:1 and A4 is assigned to 2:2" */
const uint8_t hadm_ant_perm_to_idx[2][HADM_MAX_NB_ANTENNA_PATHS] = 
{
  {0U,0U,1U,1U}, // Init
  {0U,1U,0U,1U}  // Refl
};

extern const uint8_t rtt_type_2_payload_size[7U];

/* === Externals =========================================================== */

/* === Prototypes ========================================================== */

/* === Implementation ====================================================== */

void lcl_hadm_utils_init_buffers(void)
{
    /* Initialise HAL config & results buffer */
    for (int i=0; i < HADM_MAX_NB_SIMULT_SUBEVENTS; i++)
    {
        configBuffer[i].configBufferUsed = 0;

        resultsDataBuffer[i].resultBufferUsed = 0;
        resultsDataBuffer[i].resultBuffer = &gpHadmHalResultBuffer[i*HADM_HAL_PKT_RAM_MAX_NB_STEPS_ENGAGED*BLE_HADM_STEP3_REPORT_SIZE(HADM_MAX_NB_ANTENNA_PATHS)];
        resultsDataBuffer[i].resultBufferSize = sizeof(gpHadmHalResultBuffer) / HADM_MAX_NB_SIMULT_SUBEVENTS;
    }
}

/* Allocate one HAL config buffer */
BLE_HADM_SubeventConfig_t *lcl_hadm_utils_get_config_buffer()
{
    BLE_HADM_SubeventConfig_t *config_p = NULL;
    
    for (int i=0; i < HADM_MAX_NB_SIMULT_SUBEVENTS; i++)
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

/* Free HAL config buffer(s) belonging to connIdx */
void lcl_hadm_utils_free_config_buffer(uint8 connIdx)
{
    for (int i=0; i < HADM_MAX_NB_SIMULT_SUBEVENTS; i++)
    {
        if ((configBuffer[i].configBufferUsed == 1) && (configBuffer[i].connIdx == connIdx))
        {
            configBuffer[i].configBufferUsed = 0;
        }
    }
}

/* Allocate one HAL result buffer */
BLE_HADM_SubeventResultsData_t *lcl_hadm_utils_get_result_buffer(void)
{
    BLE_HADM_SubeventResultsData_t *result_p = NULL;
    
    for (int i=0; i < HADM_MAX_NB_SIMULT_SUBEVENTS; i++)
    {
        if (resultsDataBuffer[i].resultBufferUsed == 0)
        {
            result_p = &resultsDataBuffer[i];
            result_p->resultBufferUsed = 1;
            result_p->debugBuffer = NULL;
            result_p->debugBufferSize = 0;
            result_p->nbStepsCollected = 0;
            result_p->firstStepCollected = 0;

            /* Do some inits */
            result_p->referencePwrLevel = HADM_INVALID_REFERENCE_POWER_LEVEL;
            result_p->frequencyCompensation = 0xC000;
            result_p->syncDelayUs = 0;
            break;
        }
    }
    return result_p;
}

/* Free HAL config buffer(s) belonging to connIdx */
void lcl_hadm_utils_free_result_buffer(uint8 connIdx)
{
    for (int i=0; i < HADM_MAX_NB_SIMULT_SUBEVENTS; i++)
    {
        if ((resultsDataBuffer[i].resultBufferUsed == 1) && (resultsDataBuffer[i].connIdx == connIdx))
        {
            resultsDataBuffer[i].resultBufferUsed = 0;
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
void lcl_hadm_utils_calc_ts_delay(hadm_meas_t *hadm_meas_p, hadm_device_t *hadm_device)
{
    const BLE_HADM_SubeventConfig_t *hadm_config_p = hadm_meas_p->config_p;
    int32_t ts_nominal_delay;
    int32_t ts_hw_delay;
    int32_t ts_delay;

    assert(hadm_config_p->rttPhy < HADM_RTT_PHY_MAX);

    /* When reporting round-trip time to the Host, each device excludes nominal known time offsets from the reported time. This time is equivalent to the time period
     * between the reception and transmission of the center of the CS_SYNC fields, T_SY_CENTER_DELTA */
    if (hadm_meas_p->config_p->rttMode == HADM_STEP_MODE3)
    {
        /* T_SY_CENTER_DELTA = TSY + TRD + 2 x TGD + 2 x (TSW + TPM ) x NAP + (TSW + TPM ) + TIP2 if *no* physical transmission is present in the reflector to initiator transmission extension slot. */
        ts_nominal_delay = (HADM_T_SY(hadm_config_p->rttPhy) + HADM_T_RD + (2U * HADM_T_GD) + 2U * (((uint8_t)hadm_config_p->T_PM_Time + (uint8_t)hadm_config_p->T_SW_Time) * hadm_meas_p->n_ap) 
                         + ((uint8_t)hadm_config_p->T_PM_Time + (uint8_t)hadm_config_p->T_SW_Time) + (uint8_t)hadm_config_p->T_IP2_Time);
        
        hadm_meas_p->ts_extra_delay_hns = ((uint8_t)hadm_meas_p->config_p->T_PM_Time + (uint8_t)hadm_meas_p->config_p->T_SW_Time) * 2000U;
    }
    else
    {
        /* T_SY_CENTER_DELTA = TSY + TRD + TIP1 */
        ts_nominal_delay = (HADM_T_SY(hadm_config_p->rttPhy) + HADM_T_RD + hadm_config_p->T_IP1_Time);
        hadm_meas_p->ts_extra_delay_hns = 0;
    }
    /* Take payload into account */
    ts_nominal_delay += ((rtt_type_2_payload_size[hadm_config_p->rttTypes] * 32) >> ((uint8_t)hadm_config_p->rttPhy));
    
    /* Convert us to half ns */
    ts_nominal_delay *= 2000U;

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
        ts_hw_delay += hadm_device->rtt_temperature_comp_hns[hadm_config_p->rttPhy];
    }

    if (hadm_config_p->role == HADM_ROLE_REFLECTOR)
    {
        ts_delay = ts_nominal_delay - ts_hw_delay;
    }
    else
    {
        ts_delay = ts_nominal_delay + ts_hw_delay;
    }

    assert(ts_delay > 0);
    hadm_meas_p->ts_delay_hns = (uint32_t)ts_delay;
}

void lcl_hadm_enable_lcl_interrupts(void)
{
    NVIC_ClearPendingIRQ(RSM_INT_IRQn);
    EnableIRQ(RSM_INT_IRQn);
}

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

void lcl_hadm_reset_tpm_count(void)
{    
    HADM_TPM->CNT = 0;
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
    else
    {
       hadm_meas_p->iq_buff_size_mode0 = 0;
    }
    hadm_meas_p->iq_buff_size = nb_samples_per_pm * (hadm_meas_p->n_ap + 1U) * (hadm_config_p->stepsNb - hadm_config_p->mode0Nb);
}

/* Compute HADM step duration in us for all modes */
void lcl_hadm_utils_compute_step_duration(const BLE_HADM_SubeventConfig_t *hadm_config_p, uint32_t n_ap, uint16_t *mode_dur)
{
    mode_dur[0U] = (uint16_t)(hadm_config_p->T_FCS_Time + 2U*HADM_T_SY(hadm_config_p->rttPhy) + 2U*HADM_T_RD + hadm_config_p->T_IP1_Time + HADM_T_FM + HADM_T_GD); /* T_FCS + 2*T_SY + 2*T_RD + T_IP1 + T_GD + T_FM */
    /* not used */
    mode_dur[1U] = 0;
    mode_dur[2U] = 0;
    mode_dur[3U] = 0;
}

/* Note: 78 us exec time on CM33 */
int8_t lcl_hadm_utils_compute_rpl(uint32_t iq, int8_t rssi)
{
    int32_t rpl;
    int16_t i,q;
    uint32_t mag;
    
    /* Compute IQ magnitude. IQ are signed 12 bits MSB left aligned: need signed shift by 4 bits */
    i = ABS(((int16_t)(iq & 0xFFFF)) / 16);
    q = ABS(((int16_t)((iq >> 16U) & 0xFFFF)) / 16);
    mag = MAG(i, q);

    /*
     * Maximum magnitude is 2048 (12bits)
     * RPL = RSSI +20log(2048/mag) = RSSI + 66 - 20log(mag)
     */

    if (mag > 0)
    {
        rpl = rssi + 66 - (int32_t)(20 * log10(mag));
    }
    else
    {
        rpl = HADM_INVALID_REFERENCE_POWER_LEVEL;
    }
    return (int8_t)rpl;
}


void lcl_hadm_utils_configure_antenna_switching(hadm_meas_t *hadm_meas_p)
{
    bool ena_antsw_pa_ramping = false; /* OJE TODO, for now disable PA ramping */
    xcvrLclStatus_t status;
    
    /* Perform antenna permutation index to antenna index mapping */
    /* " Most antenna configurations described below are 1:X or X:1 orientations where X is in the set of 1 to 4. 
     * In these configurations, antenna path A1 is assigned to the 1:1 antenna combination, A2 is assigned to the 1:2 or 2:1 combination, 
     * A3 is assigned to the 1:3 or 3:1 combination and A4 is assigned to the 1:4 or 4:1 combination. 
     * The exception is the 2:2 configuration where A1 is assigned to 1:1, A2 is assigned to 1:2, A3 is assigned to 2:1 and A4 is assigned to 2:2. " */
    if (hadm_meas_p->config_p->toneAntennaConfigIdx == HADM_ANT_CFG_IDX_7)
    {
        uint8_t ant_gpio[HADM_MAX_NB_ANTENNA_PATHS];
        uint32_t role = hadm_meas_p->config_p->role;
        ant_gpio[0] = hadm_device.ant2gpio[hadm_ant_perm_to_idx[role][0]];
        ant_gpio[1] = hadm_device.ant2gpio[hadm_ant_perm_to_idx[role][1]];
        ant_gpio[2] = hadm_device.ant2gpio[hadm_ant_perm_to_idx[role][2]];
        ant_gpio[3] = hadm_device.ant2gpio[hadm_ant_perm_to_idx[role][3]];
        status = XCVR_LCL_ConfigLclBlock(&hadm_meas_p->rsm_config, (XCVR_RSM_T_CAPTURE_SEL_T)hadm_meas_p->config_p->T_PM_Time, ant_gpio, ena_antsw_pa_ramping);
    }
    else /* direct map btw antenna permutation index and antenna index */
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
        hadm_meas_p->rtt_antenna_id = (hadm_meas_p->rtt_antenna_id + 1) % hadm_meas_p->n_ap;
    }
    else
    {
        ant_id = hadm_meas_p->rtt_antenna_id;
        if (hadm_meas_p->config_p->rttAntennaID == HADM_RTT_ANT_ROUND_ROBIN)
        {
            /* Compute next */
            hadm_meas_p->rtt_antenna_id = (hadm_meas_p->rtt_antenna_id + 1) % hadm_meas_p->num_ant; /* round robin */
        }
    }
    return (uint8_t)ant_id;
}

/* EOF */
