/*! *********************************************************************************
 * \defgroup CONTROLLER
 * @{
 ********************************************************************************** */
/*! *********************************************************************************
*
* Copyright 2020-2025 NXP
*
* \file
*
* SPDX-License-Identifier: BSD-3-Clause
********************************************************************************** */

#if defined(gNbu_Hadm_d) && gNbu_Hadm_d == 1
#define BT60_HADM
#endif

#include "board.h"
#include "ble_general.h"
#include "controller_init.h"
#include "ll_types.h"
#include "controller_api_ll.h"
#include "fwk_debug_struct.h"

#include "nxp2p4_xcvr.h"
#include "nxp_xcvr_gfsk_bt_0p5_h_0p5_config.h"
#include "nxp_xcvr_coding_config.h"

#if defined(gMWS_Enabled_d) && (gMWS_Enabled_d)
#include "MWS.h"
#include "ble_mws.h"
#endif

#if defined(gPlatformEnableDcdcOnNbu_d) && (gPlatformEnableDcdcOnNbu_d == 1)
#include "fwk_platform_dcdc.h"
#endif

#if defined(FPGA_TARGET) && (FPGA_TARGET == 1)
#include "hdi.h"
#endif

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#ifndef gControllerXcvrInitRetryCount_c
#define gControllerXcvrInitRetryCount_c (10U)
#endif

#ifndef gNbuMaxTxPowerDbm_c
/* default max tx power setting to avoid API call at power on */
#define gNbuMaxTxPowerDbm_c                    10U
#define gNbuMaxTxPowerLdoTrim_c                15U
#endif

/************************************************************************************
*************************************************************************************
* Private prototypes
*************************************************************************************
************************************************************************************/

/************************************************************************************
*************************************************************************************
* Private memory declarations
*************************************************************************************
************************************************************************************/

/* LDO ANT TRIM value to be applied at each XCVR Init or mode change.
   This is updated by app core. Valid value 0 - 15.
   Set to invalid value by default */
uint8_t g_ldo_ant_trim = (XCVR_ANALOG_LDO_1_LDO_ANT_TRIM_MASK >>
                          XCVR_ANALOG_LDO_1_LDO_ANT_TRIM_SHIFT) + 1U;

/************************************************************************************
*************************************************************************************
* Private functions
*************************************************************************************
************************************************************************************/

void Controller_RestoreLdoAntTrim(void)
{
    if( g_ldo_ant_trim <= (XCVR_ANALOG_LDO_1_LDO_ANT_TRIM_MASK >> XCVR_ANALOG_LDO_1_LDO_ANT_TRIM_SHIFT) )
    {
        // Set LDO ANT Trim
        uint32_t temp_trim;
        temp_trim = XCVR_ANALOG->LDO_1;
        temp_trim &= ~(XCVR_ANALOG_LDO_1_LDO_ANT_TRIM_MASK);
        temp_trim |= XCVR_ANALOG_LDO_1_LDO_ANT_TRIM(g_ldo_ant_trim);
        XCVR_ANALOG->LDO_1 = temp_trim;
    }
}

/************************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
************************************************************************************/
/* LTC access protection */
void (* lock_LTC)(void) = NULL;
void (* unlock_LTC)(void) = NULL;

/*! *********************************************************************************
* \brief   Initializes the XCVR module
*
* \return uint32_t , 0 if successful, error if other values.
*
********************************************************************************** */

uint32_t Controller_RadioInit(void)
{
    xcvrStatus_t status = gXcvrSuccess_c;

    const xcvr_config_t *xcvrConfig         = &xcvr_gfsk_bt_0p5_h_0p5_1mbps_full_config;
    const xcvr_coding_config_t *rbmeConfig  = &xcvr_ble_coded_s8_config;

#ifndef SIMULATOR
    status = XCVR_Init(&xcvrConfig, &rbmeConfig);
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 475)
    /* disable capture and enable auto load */
    XCVR_MISC->XCVR_CTRL |= XCVR_MISC_XCVR_CTRL_LL_CFG_CAPT_DIS(1);
    RADIO_CTRL->BLE_AUTOLOAD = RADIO_CTRL_BLE_AUTOLOAD_BLE_AUTOLOAD_CNT_VAL(8U);
#endif
    /* update LDO trim after xcvr init as the default value is no always correct */
    Controller_RestoreLdoAntTrim();

    assert(status == gXcvrSuccess_c);

    {
      /*TODO: Temporary RSSI fix is put here until fix is integrated in next XCVr release*/
      uint32_t read_rssi_ctrl = XCVR_RX_DIG->NB_RSSI_CTRL0;

      read_rssi_ctrl &= ~(XCVR_RX_DIG_NB_RSSI_CTRL0_RSSI_M_WINDOW_NB_MASK | XCVR_RX_DIG_NB_RSSI_CTRL0_RSSI_N_WINDOW_NB_MASK);
      read_rssi_ctrl |= (XCVR_RX_DIG_NB_RSSI_CTRL0_RSSI_M_WINDOW_NB(0x3)| XCVR_RX_DIG_NB_RSSI_CTRL0_RSSI_N_WINDOW_NB(0x3));
      XCVR_RX_DIG->NB_RSSI_CTRL0 = read_rssi_ctrl;
    }
#endif

    return (uint32_t)status;
}

uint32_t Controller_SetNbuVersion(const uint8* repo_digest)
{
    /*MSB of SHA1 is stored in repo_digest[0]*/
    NBUDBG_SET_SHA(repo_digest[0]|(repo_digest[1] << 8U)|(repo_digest[2] << 16U)|(repo_digest[3]<<24U));
    return 0;
}

osa_status_t Controller_Init(const nbuIntf_t* nbuInterface)
{
#if defined(FPGA_TARGET) && (FPGA_TARGET == 1)
    /* Select BLE radio mode */
    (void)HDI_Init();
    HDI_Set_Mode_Ble();
#endif

    /* set BLE as active LL */
    RADIO_CTRL->LL_CTRL &= ~RADIO_CTRL_LL_CTRL_ACTIVE_LL_MASK;

    /* enable sleep clock for SWT TMR */
    CIU2->CIU2_LBC_CTRL |= CIU2_CIU2_LBC_CTRL_LBC_NCO_EN_MASK;

    /* Enable dejit on 3.2Kz clock - required for lowpower and active to avoid irregular duty cycle on the clock */
    CIU2->CIU2_LBC_CTRL |=  (CIU2_CIU2_LBC_CTRL_DEJIT_EN_MASK | CIU2_CIU2_LBC_CTRL_AUTO_DEJIT_MASK);

    NbuHosted_Config(nbuInterface);

#if defined(gMWS_Enabled_d) && (gMWS_Enabled_d)
    MWS_Register(gMWS_BLE_c, MWS_BLE_Callback);
    // When 15.4 Phy becomes active LL then BLE block should be clocked ...
    // otherways it will loose time sync and also crush when GetInactivity read BLE registers
    RADIO_CTRL->RF_CLK_CTRL |= RADIO_CTRL_RF_CLK_CTRL_BTLL_CLK_EN_OVRD(1);
#endif

#if defined(gNbu_Hadm_d) && gNbu_Hadm_d == 1
    // Sample code to set HADM events priority lower than connection events.
    // LL_API_SchedSetPriority(LL_SCHED_PRIO_CONN);
    // HADM events priority is higher than connection events (default after power on reset)
    // LL_API_SchedSetPriority(LL_SCHED_PRIO_DEFAULT);
#endif

#if defined(gNbuMaxTxPowerDbm_c)
    // set the max tx power to avoid LL API call at power on
#if !defined(FPGA_TARGET) || (FPGA_TARGET == 0)
    Controller_SetMaxTxPower(gNbuMaxTxPowerDbm_c, gNbuMaxTxPowerLdoTrim_c);
#endif
#endif

    /* low level initialization of the controller
     * must happen before kernel init */
    NB_API_PreKernelInit();

    return KOSA_StatusSuccess;
}

/* controller API called by the app core to update LDO ANT TRIM */
bleResult_t Controller_SetMaxTxPower(int8_t power_dBm, uint8_t ldo_ant_trim)
{
    /* default status is success */
    bleResult_t status = gBleSuccess_c;
    
    if(ldo_ant_trim <= (XCVR_ANALOG_LDO_1_LDO_ANT_TRIM_MASK >> XCVR_ANALOG_LDO_1_LDO_ANT_TRIM_SHIFT))
    {
        status = (bleResult_t)LL_API_SetMaxTxPower(power_dBm);
        if (gBleSuccess_c == status)
        {
            OSA_DisableIRQGlobal();

            g_ldo_ant_trim = ldo_ant_trim;

            // Set LDO ANT Trim 
            uint32_t temp_trim;
            temp_trim = XCVR_ANALOG->LDO_1;
            temp_trim &= ~(XCVR_ANALOG_LDO_1_LDO_ANT_TRIM_MASK);
            temp_trim |= XCVR_ANALOG_LDO_1_LDO_ANT_TRIM(g_ldo_ant_trim);
            XCVR_ANALOG->LDO_1 = temp_trim;

            OSA_EnableIRQGlobal();
        }
    }
    else
    {
        status = gBleInvalidParameter_c;
    }
#if defined(gPlatformEnableDcdcOnNbu_d) && (gPlatformEnableDcdcOnNbu_d == 1)
    /* Configure SPC high power mode depending the targeted tx power and if the application core is allowing it */
    PLATFORM_ConfigureSpcHighPowerMode(power_dBm);
#endif

    return status;
}

/*! *********************************************************************************
* @}
********************************************************************************** */
