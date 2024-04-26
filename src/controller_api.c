/*! *********************************************************************************
 * \defgroup CONTROLLER
 * @{
 ********************************************************************************** */
/*! *********************************************************************************
*
* Copyright 2021-2022 NXP
* All rights reserved.
*
* \file
*
* SPDX-License-Identifier: BSD-3-Clause
********************************************************************************** */

/************************************************************************************
*************************************************************************************
* Include
*************************************************************************************
************************************************************************************/
#include "fwk_platform_ics.h"
#include "fwk_platform.h"
#include "controller_api.h"
#include "RNG_Interface.h"

/************************************************************************************
*************************************************************************************
* Definitions
*************************************************************************************
************************************************************************************/

#define _GET8(p)  (*((uint8_t*)(p)))
#define _GET16(p) (((uint16_t)_GET8 ((p))) | ((uint16_t)_GET8 ((uint8_t*)(p)+1U) << 8U))
#define _GET32(p) (((uint32_t)_GET16((p))) | ((uint32_t)_GET16((uint8_t*)(p)+2U) << 16U))

// this enum must be aligned with the one defined in the controller
typedef enum
{
    API_Controller_SetTxPowerLevel,
    API_Controller_SetTxPowerLevelDbm,
    API_Controller_SetMaxTxPower,
    API_Controller_SetRandomSeed,
    API_Controller_ConfigureAdvCodingScheme,
    API_Controller_ConfigureConnCodingScheme,
    API_Controller_ConfigureInvalidPduHandling,
    API_Controller_SetConnNotificationMode,
    API_Controller_SetChannelSelectionAlgo2,
    API_Controller_GetTimestamp,
    API_Controller_GetTimestampEx,
    API_Controller_ConfigureFEM,
    API_Controller_GetEncryptionParam,
    API_Controller_ConfigureCOEX,
    API_Controller_SetRxMode,
    API_Controller_ConfigureSCA,
    API_Controller_ConfigureIDSSecurityEvent,
    API_Controller_ReadMemory,
    API_Last
} PLATFORM_NbuApiId;

/************************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
************************************************************************************/
osa_status_t Controller_SetTxPowerLevel(uint8_t level, txChannelType_t channel)
{
    osa_status_t api_return = KOSA_StatusSuccess;
    int32_t      ret        = 0;
    bool rpmsg_status;
    uint32_t tab[2U] = {(uint32_t)level, (uint32_t)channel};
    rpmsg_status = PLATFORM_NbuApiReq((uint8_t*)&ret, API_Controller_SetTxPowerLevel,
                                (const uint8_t*)"\x01\x01", tab, 4U);
    assert(rpmsg_status);
    (void)rpmsg_status;

    if ( ret!=0 )
    {
        api_return = KOSA_StatusError;
    }
    return api_return;
}

osa_status_t Controller_SetTxPowerLevelDbm(int8_t level_dbm, txChannelType_t channel)
{
    osa_status_t api_return = KOSA_StatusSuccess;
    int32_t      ret        = 0;
    bool rpmsg_status;
    uint32_t tab[2U] = {(uint32_t)level_dbm, (uint32_t)channel};
    rpmsg_status = PLATFORM_NbuApiReq((uint8_t*)&ret, API_Controller_SetTxPowerLevelDbm,
                                (const uint8_t*)"\x01\x01", tab, 4U);
    assert(rpmsg_status);
    (void)rpmsg_status;

    if ( ret!=0 )
    {
        api_return = KOSA_StatusError;
    }
    return api_return;
}

osa_status_t Controller_SetMaxTxPower(int8_t power_dBm, uint8_t ldo_ant_trim)
{
    osa_status_t api_return = KOSA_StatusSuccess;
    int32_t      ret        = 0;
    bool rpmsg_status;
    uint32_t tab[2U] = {(uint32_t)power_dBm, (uint32_t)ldo_ant_trim};
    rpmsg_status = PLATFORM_NbuApiReq((uint8_t*)&ret, API_Controller_SetMaxTxPower,
                                (const uint8_t*)"\x01\x01", tab, 4U);
    assert(rpmsg_status);
    (void)rpmsg_status;

    if ( ret!=0 )
    {
        api_return = KOSA_StatusError;
    }
    return api_return;
}

osa_status_t Controller_SetRandomSeed(void)
{
    osa_status_t api_return = KOSA_StatusSuccess;
    int32_t      ret        = 0;
    bool rpmsg_status;
    uint32_t seed[1];

#if !defined(MULTICORE_APP) || (MULTICORE_APP!=1)
    RNG_GetPseudoRandomNo((uint8_t*)seed, sizeof(seed), NULL);
#else
    RNG_GetTrueRandomNumber(seed);
#endif

    rpmsg_status = PLATFORM_NbuApiReq((uint8_t*)&ret, API_Controller_SetRandomSeed,
                                (const uint8_t*)"\x04", seed, 4U);
    assert(rpmsg_status);
    (void)rpmsg_status;

    if ( ret!=0 )
    {
        api_return = KOSA_StatusError;
    }
    return api_return;
}

osa_status_t Controller_ConfigureAdvCodingScheme(advCodingScheme_t codingSch)
{
    osa_status_t api_return    = KOSA_StatusSuccess;
    int32_t      ret           = 0;
    uint32_t     coding_scheme[1] = {(uint32_t)codingSch};
    bool rpmsg_status = PLATFORM_NbuApiReq((uint8_t*)&ret, API_Controller_ConfigureAdvCodingScheme,
                                (const uint8_t*)"\x01", coding_scheme, 4U);
    assert(rpmsg_status);
    (void)rpmsg_status;

    if ( ret!=0 )
    {
        api_return = KOSA_StatusError;
    }
    return api_return;
}

osa_status_t Controller_ConfigureConnCodingScheme(connCodingScheme_t codingSch)
{
    osa_status_t api_return = KOSA_StatusSuccess;
    int32_t      ret        = 0;
    uint32_t     coding_scheme[1] = {(uint32_t)codingSch};
    bool rpmsg_status = PLATFORM_NbuApiReq((uint8_t*)&ret, API_Controller_ConfigureConnCodingScheme,
                                (const uint8_t*)"\x01", coding_scheme, 4U);
    assert(rpmsg_status);
    (void)rpmsg_status;

    if ( ret!=0 )
    {
        api_return = KOSA_StatusError;
    }
    return api_return;
}

osa_status_t Controller_ConfigureInvalidPduHandling(uint32_t pdu_handling_type)
{
    osa_status_t api_return = KOSA_StatusSuccess;
    int32_t      ret        = 0;
    uint32_t     tab[1]     = {pdu_handling_type};
    bool rpmsg_status = PLATFORM_NbuApiReq((uint8_t*)&ret, API_Controller_ConfigureInvalidPduHandling,
                                (const uint8_t*)"\x01", tab, 4U);
    assert(rpmsg_status);
    (void)rpmsg_status;

    if ( ret!=0 )
    {
        api_return = KOSA_StatusError;
    }
    return api_return;
}

osa_status_t Controller_SetConnNotificationMode(uint32_t mode)
{
    osa_status_t api_return = KOSA_StatusSuccess;
    int32_t      ret        = 0;
    uint32_t     tab[1]     = {(uint32_t) mode};
    bool rpmsg_status = PLATFORM_NbuApiReq((uint8_t*)&ret, API_Controller_SetConnNotificationMode,
                                (const uint8_t*)"\x04", tab, 4U);
    assert(rpmsg_status);
    (void)rpmsg_status;

    if ( ret!=0 )
    {
        api_return = KOSA_StatusError;
    }
    return api_return;
}

osa_status_t Controller_SetChannelSelectionAlgo2(bool enable)
{
    osa_status_t api_return = KOSA_StatusSuccess;
    int32_t      ret        = 0;
    uint32_t     tab[1]     = {(uint32_t) enable};
    bool rpmsg_status = PLATFORM_NbuApiReq((uint8_t*)&ret, API_Controller_SetChannelSelectionAlgo2,
                                (const uint8_t*)"\x01", tab, 4U);
    assert(rpmsg_status);
    (void)rpmsg_status;

    if ( ret!=0 )
    {
        api_return = KOSA_StatusError;
    }
    return api_return;
}    
    
osa_status_t Controller_GetNextLinLayerEventUsUnsafe(uint32_t* remaining_time)
{
    osa_status_t api_return = KOSA_StatusError;
    * remaining_time = 0u; /* no event */

    return api_return;
}

osa_status_t Controller_GetTimestamp(uint32_t* timestamp)
{
    int32_t      ret        = 0;
    uint32_t     dummy[1]   = {0};
    bool         rpmsg_status = PLATFORM_NbuApiReq((uint8_t*)&ret, API_Controller_GetTimestamp,
                                (const uint8_t*)"\x01", dummy, 4U);
    *timestamp = ret;
    if (rpmsg_status)
    {
      return KOSA_StatusSuccess;
    }
    else
    {
      return KOSA_StatusError;
    }
}

osa_status_t Controller_GetTimestampEx(uint32_t* ll_timing_slot, uint16_t* ll_timing_us, uint64_t *tstmr)
{
    osa_status_t api_status;
    int32_t      ret[5];
    bool         rpmsg_status = PLATFORM_NbuApiReq((uint8_t*)&ret[0], API_Controller_GetTimestampEx,
                                (const uint8_t*)"", NULL, 20U);
    if (rpmsg_status)
    {
      api_status = (osa_status_t)ret[0];
      if( api_status == KOSA_StatusSuccess )
      {
        // slot
        *ll_timing_slot = ret[1];
        // slot offset in us
        *ll_timing_us   = (uint16_t)ret[2];
        // TSTMR
        *tstmr = (uint64_t)ret[3] + (((uint64_t)ret[4])<<32U);
      }
    }
    else
    {
      api_status = KOSA_StatusError;
    }
    return api_status;
}

/* Default FEM configuration which is xcvr_pa_fem_config_t + RF_GPO option */
const uint8_t default_FEM_config[FEM_CONFIG_SIZE] = {
    1U, /* dual mode  XCVR_ANTX_MODE_T op_mode;  operating mode for the PA/FEM interface */
    0U, /* Disabled   uint8_t ant_sel_pins_enable;  control whether ANT_A & ANT_B pads should be active (0=Disabled,
           1=Enabled) */
    1U, /* Enabled    uint8_t tx_rx_switch_pins_enable; control whether TX and RX_SWITCH pads should be active
           (0=Disabled, 1=Enabled) */
    0U, /* Disabled   uint8_t high_z_enable;  control whether FEM/PA  pads should use High Z (0=Disabled, =Enabled) */
    0U, /* Disabled   uint8_t use_fad_state_machine;  control whether FAD state machine is active (0=Disabled,
           1=Enabled) */
    0U, /* TSM GPIO   XCVR_FAD_NOT_GPIO_MODE_T ant_a_pad_control;  control whether ANT_A pad should be controlled by FAD
           state machine or TSM GPIO */
    0U, /* TSM GPIO   XCVR_FAD_NOT_GPIO_MODE_T ant_b_pad_control;  control whether ANT_B pad should be controlled by FAD
           state machine or TSM GPIO */
    0U, /* TSM GPIO   XCVR_FAD_NOT_GPIO_MODE_T tx_switch_pad_control; control whether TX_SWITCH pad should be controlled
           by FAD state machine or TSM GPIO  */
    0U, /* TSM GPIO   XCVR_FAD_NOT_GPIO_MODE_T rx_switch_pad_control; control whether RX_SWITCH pad should be
           controlled by FAD state machine or TSM GPIO  */
    0U, /* ?          uint8_t pa_tx_wu;  Number of usec to start external PA warmup ahead of internal PA warmup (ramp)
           start */
    0U, /* MUST=0     uint8_t pa_tx_wd;  Number of usec to start external PA warmdown ahead of internal PA warmdown
           (ramp) completion */
    0U, /* ?          uint8_t lna_rx_wu; Number of usec to start external LNA warmup ahead of internal LNA warmup start
         */
    0U, /* MUST=0     uint8_t lna_rx_wd; Number of usec to start external LNA warmdown ahead of internal LNA warmdown
           completion */
    0U, /* HIGH       XCVR_RX_TX_POLARITY_MODE_T tx_switch_pol_control;  control whether TX_SWITCH pad should be active
           high or low  (0=Active HIGH, 1=Active LOW) */
    0U, /* HIGH       XCVR_RX_TX_POLARITY_MODE_T rx_switch_pol_control;  control whether RX_SWITCH pad should be active
           high or low  (0=Active HIGH, 1=Active LOW) */
    1U, /* RF_GPO[7:4] uint8_t  control which RF_GPOs to use, [0,2]=RF_GPO[3:0], [1,3]=RF_GPO[7:4], 6=RF_GPO[11:8],
           otherwise invalid */
};

osa_status_t Controller_ConfigureFEM(const uint8_t *config_ptr, uint8_t config_len)
{
    osa_status_t api_return = KOSA_StatusSuccess;
    int32_t      ret        = 0;
    bool rpmsg_status;
    uint32_t tab[FEM_CONFIG_SIZE];
    const uint8_t *config;

    /* use the input configuration if provided and its length is correct, otherwise use the default */
    if (config_ptr != NULL)
    {
        if (config_len == FEM_CONFIG_SIZE)
        {
            config = config_ptr;
        }
        else
        {
            return KOSA_StatusError;
        }
    }
    else
    {
        config = default_FEM_config;
    }
    assert(config[2]==1); /* tx_rx_switch_pins_enable must enabled */

    for (int i=0; i<FEM_CONFIG_SIZE; i++)
    {
        tab[i] = (uint32_t)config[i];
    }
    rpmsg_status = PLATFORM_NbuApiReq((uint8_t*)&ret, API_Controller_ConfigureFEM,
                                (const uint8_t*)"\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01", tab, 4U);
    assert(rpmsg_status);
    (void)rpmsg_status;

    if ( ret!=0 )
    {
        api_return = KOSA_StatusError;
    }
    return api_return;
}

osa_status_t Controller_GetEncryptionParam(uint16_t conn_handle, 
                                           uint8_t  sk_or_skd_req, 
                                           uint8_t  sk_or_skd[16], 
                                           uint8_t  iv[8], 
                                           uint8_t  payload_counter_tx[5], 
                                           uint8_t  payload_counter_rx[5])
{
    osa_status_t api_status;
    int8_t       ret[38];
    uint32_t     tab[2]     = {conn_handle, sk_or_skd_req};
    bool         rpmsg_status = PLATFORM_NbuApiReq((uint8_t*)&ret[0], API_Controller_GetEncryptionParam,
                                (const uint8_t*)"\x02\x01", tab, 38U);
    if (rpmsg_status)
    {
      api_status = (osa_status_t)_GET32(ret);
      if( api_status == KOSA_StatusSuccess )
      {
        memcpy(sk_or_skd,          ret+4U,           16U);
        memcpy(iv,                 ret+4U+16U,       8U );
        memcpy(payload_counter_tx, ret+4U+16U+8U,    5U );
        memcpy(payload_counter_rx, ret+4U+16U+8U+5U, 5U );
      }
    }
    else
    {
      api_status = KOSA_StatusError;
    }
    return api_status;
}

/* Default COEX configuration including several XCVR structures + output/input signals setting + RF_GPO option.
   Details: all output signals activated on RF_GPO[6,5,4] (no inversion/muxing), TSM is source, high priority TX/RX,
            RF_ACTIVE aligned with RF_STATUS & RF_PRIO in RX but TX, RF_NOT_ALLOWED on PTA16 (no inversion).
   Remark: RF_NALLOWED is active high in KW45 (in contrary with RM) ! */
const uint8_t default_COEX_config[COEX_CONFIG_SIZE] = {
    // wiring configuration, RF_NOT_ALLOWED is active per default
    0x01, /* uint8 rf_active_used;      When set to 1, RF_ACTIVE signal will be active */
    0x01, /* rf_status_used;            When set to 1 RF_STATUS signal will be active */
    0x01, /* rf_prio_used;              0 if not used, 1 if only priority[0] used, 2 for priority[1:0] */
    0x00, /* rfmcGpoCoex;               [0,5]=RF_GPO[6:4]=PORTD[3,2,1]=(*)J14-[16,14,12], [1,4]=RF_GPO[2:0]=PORTA[20,19,18]=(*)J14-[8,6,4] for [RF_PRIO,RF_STATUS,RF_ACTIVE]. (*) for FPGA */
    0x01, /* tsm_controls_coex;         When set to 1, TSM will control output signals */
    //--------
    0x01, /* rfactSrc;                  coexRfactSrc_t 0..2 */
    //-------- coexRfSignalInvert_t:
    0x00, /* rfna_invert;               When set to 1, inverts the RF_NOT_ALLOWED signal in RFMC muxing logic */
    0x00, /* rfact_invert;              When set to 1, inverts the RF_ACTIVE signal in RFMC muxing logic */
    0x00, /* rfstat_invert;             When set to 1, inverts the RF_STATUS signal in RFMC muxing logic */
    0x00, /* rfpri_invert[0];           When set to 1, inverts the RF_PRIORITY signals in RFMC muxing logic */
    0x00, /* rfpri_invert[1];           When set to 1, inverts the RF_PRIORITY signals in RFMC muxing logic */
    //-------- coexRfNotAllowedConfig_t:
    0x01, /* rfna_pin_enable;           coexRfNotAllowPin_t 1=PTA16, 2=PTA17, 4=PTC7. N/A for FPGA: PTD6 J14-19 */
    0x01, /* link_layer_rfna_select;    Enable bits for RF_NOT_ALLOWED signal to LLs. Value is the OR'd together of any ::coexRfNotAllowLL_t entries 0,1,4,8. */
    //-------- coexRfActiveTsmConfig_t:
    0x37, /* rf_act_extend;             RF_ACTIVE remains asserted this many microseconds after the end of TSM RX or TX sequences. Must be <=255. */
    0x5D, /* rf_act_tx_advance;         RF_ACTIVE aserts this many microseconds before the end of TX warm up. If this value is > than the number of microseconds before end of TX warm  ramp up in the TSM timing sequence then an error will be returned. */
    0x5A, /* rf_act_rx_advance;         RF_ACTIVE aserts this many microseconds before the RX digital is enabled. If this value is > than the number of microseconds before end of RX warm  ramp up in the TSM timing sequence then an error will be returned. */
    //-------- coexRfActiveRfmcConfig_t:
    0x00, /* deassert_when_tsm_idle;    When set to 1, RFMS will deassert RF_ACTIVE if the TSM is idle. Otherwise deassertion is on next low power entry. */
    0x00, /* wakeup_delay;              RF_ACTIVE asserts this many 32KHz ref. clocks after the XO is enabled.  Must be less <= 63. */
    //-------- coexRfStatusConfig_t:
    0x5A, /* rf_stat_tx_advance;        RF_STATUS aserts this many microseconds before the end of TX warm up. Must < TX_WU=0x70. */
    //-------- coexRfPriorityConfig_t:
    0x5A, /* rf_pri_tx_advance;         RF_PRIORITY aserts this many microseconds before the end of TX warm up. If this value is > than the number of microseconds before end of TX warm  ramp up in the TSM timing sequence then an error will be returned. */
    0x5A, /* rf_pri_rx_advance;         RF_PRIORITY aserts this many microseconds before the RX digital is enabled. If this value is > than the number of microseconds before end of RX warm  ramp up in the TSM timing sequence then an error will be returned. */
    0x00, /* rf_pri_on_rf_stat;         When set to 1, RF_PRIORITY signal is muxed on  the RF_STATUS signal. */
    //--------
    0x01, /* rf_pri_rx;                 priority of RX. 0 LOW, 1 HIGH */
    0x01, /* rf_pri_tx;                 priority of TX. 0 LOW, 1 HIGH */
};

osa_status_t Controller_ConfigureCOEX(const uint8_t *config_ptr, uint8_t config_len)
{
    osa_status_t api_return = KOSA_StatusSuccess;
    int32_t      ret        = 0;
    bool rpmsg_status;
    uint32_t tab[COEX_CONFIG_SIZE];
    const uint8_t *config;

    /* use the input configuration if provided and its length is correct, otherwise use the default */
    if (config_ptr != NULL)
    {
        if (config_len == COEX_CONFIG_SIZE)
        {
            config = config_ptr;
        }
        else
        {
            return KOSA_StatusError;
        }
    }
    else
    {
        config = default_COEX_config;
    }

    for (int i=0; i<COEX_CONFIG_SIZE; i++)
    {
        tab[i] = (uint32_t)config[i];
    }
    rpmsg_status = PLATFORM_NbuApiReq((uint8_t*)&ret, API_Controller_ConfigureCOEX,
                                (const uint8_t*)"\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01", tab, 4U);
    assert(rpmsg_status);
    (void)rpmsg_status;

    if ( ret!=0 )
    {
        api_return = KOSA_StatusError;
    }
    return api_return;
}

osa_status_t Controller_Set_PLL_OFFSET_CTRL(int32_t cfo_in_hz)
{
    osa_status_t api_return = KOSA_StatusSuccess;
    uint32_t chan_amp, lpm_denom_selected;

    /* Make sure remote CPU is awake */
    PLATFORM_RemoteActiveReq();

    chan_amp = XCVR_PLL_DIG->CHAN_MAP;
    lpm_denom_selected = XCVR_PLL_DIG->LPM_SDM_RES2;
    /* verify that BTLE band is selected with appropriate LPM_DENOM value to ensure the correct Hertz-to-PLL_OFFSET_CTRL
       conversion. 1 LSB of cfo_in_hz corresponds to (2*REFCLK*1e6)/LPM_DENOM Hz = (2*32e6)/0x4000000 = 0.9536743 Hz */
    if (((chan_amp & XCVR_PLL_DIG_CHAN_MAP_BAND_SELECT_MASK) == 0U) &&
        (lpm_denom_selected == 0x4000000))
    {
        /* this part is duplicated from int32_t XCVR_LCL_RsmReadCfoComp(void) in XCVR/drv/nxp_xcvr_lcl_ctrl.c */
        if ((cfo_in_hz > 500000) || (cfo_in_hz < -500000)) /* Test CFO limitation to keep to ~20bits */
        {
            api_return = KOSA_StatusError;
        }
        else
        {
            /* Each bit in the PLL_OFFSET_CTRL register is about 0.9537 Hz of frequency change so convert by multiply by 67 and
             * dividing by 64 */
            int32_t itemp                 = cfo_in_hz;
            itemp                         = itemp * 67;
            itemp                         = itemp / 64;
            XCVR_PLL_DIG->PLL_OFFSET_CTRL = XCVR_PLL_DIG_PLL_OFFSET_CTRL_PLL_NUMERATOR_OFFSET(itemp);
        }
    }
    else
    {
        api_return = KOSA_StatusError;
    }
    /* Release wake up to other CPU */
    PLATFORM_RemoteActiveRel();
    return api_return;
}

/*! *********************************************************************************
 * \brief  Set Rx Pdu handling timing
 * \param[in]  0: normal mode
 *             1: delay Rx Pdu handling after TX.
 *
 * \return KOSA_StatusSuccess or KOSA_StatusError.
 ********************************************************************************** */
osa_status_t Controller_SetRxMode(uint32_t mode)
{
    osa_status_t api_return = KOSA_StatusSuccess;
    int32_t      ret        = 0;
    uint32_t     tab[1]     = {(uint32_t) mode};
    bool rpmsg_status = PLATFORM_NbuApiReq((uint8_t*)&ret, API_Controller_SetRxMode,
                                (const uint8_t*)"\x04", tab, 4U);
    assert(rpmsg_status);
    (void)rpmsg_status;

    if ( ret!=0 )
    {
        api_return = KOSA_StatusError;
    }
    return api_return;
}

/*! *********************************************************************************
 * \brief  Set sleep clock accuracy for the device.
 * \param[in]  sca: ranges from 0 to 7
 *
 * \return KOSA_StatusSuccess or KOSA_StatusError.
 ********************************************************************************** */
osa_status_t Controller_SetSleepClockAccuracy(uint32_t sca)
{
    osa_status_t api_return = KOSA_StatusSuccess;
    int32_t      ret        = 0;
    uint32_t     tab[1]     = {sca};
    bool rpmsg_status = PLATFORM_NbuApiReq((uint8_t*)&ret, API_Controller_ConfigureSCA,
                                (const uint8_t*)"\x01", tab, 4U);
    assert(rpmsg_status);
    (void)rpmsg_status;

    if ( ret!=0 )
    {
        api_return = KOSA_StatusError;
    }
    return api_return;
}

/*!
 * \brief Sets the security events to be monitored in LL and the callback to be called upon security event detection.
 *
 * \param[in] bitmask containing security events
 * Supported security events:
 * bit 0 set: enable gSecEvt_LlLenOverflow_c event reporting
 * \param[in] callback called when security events (enabled through mask) have been detected
 * \return KOSA_StatusSuccess or KOSA_StatusError.
 */
osa_status_t Controller_ConfigureIDSSecurityEvent(nbu_security_event_callback_t cb, uint32_t securityEventsBitmask)
{
    osa_status_t api_return = KOSA_StatusSuccess;
    int32_t      ret        = 0;
    uint32_t     tab[1]     = {securityEventsBitmask};
    bool rpmsg_status = PLATFORM_NbuApiReq((uint8_t*)&ret, API_Controller_ConfigureIDSSecurityEvent,
                                (const uint8_t*)"\x01", tab, 4U);
    assert(rpmsg_status);
    (void)rpmsg_status;
    
    if (cb != NULL)
    {
      PLATFORM_RegisterSecurityEventCb(cb);
    }

    if ((ret != 0) || (cb == NULL))
    {
        api_return = KOSA_StatusError;
    }
    return api_return;
}

/*! *********************************************************************************
 * \brief  Read the NBU memory
 * \param[in]  address: NBU memory address
 * \param[in]  size:    number of bytes to read, limited to 32 bytes maximum
 * \param[in]  pBuffer: buffer to save the data
 *
 * \return KOSA_StatusSuccess or KOSA_StatusError.
 ********************************************************************************** */
osa_status_t Controller_ReadMemory(uint32_t address, uint32_t size, void* pBuffer)
{
    osa_status_t api_status;
    int8_t       ret[36];
    uint32_t     tab[2]     = {address, size};
    
    if ( size > sizeof(ret) - 4U )
    {
        api_status = KOSA_StatusError;
    }
    else
    {
        bool rpmsg_status = PLATFORM_NbuApiReq((uint8_t*)&ret[0], API_Controller_ReadMemory,
                                    (const uint8_t*)"\x04\x04", tab, 4U + size);
        if (rpmsg_status)
        {
          api_status = (osa_status_t)_GET32(ret);
          if( api_status == KOSA_StatusSuccess )
          {
            memcpy(pBuffer, ret+4U, size);
          }
        }
        else
        {
          api_status = KOSA_StatusError;
        }
    }
    return api_status;
}
