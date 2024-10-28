/*! *********************************************************************************
 * \defgroup CONTROLLER
 * @{
 ********************************************************************************** */
/*! *********************************************************************************
*
* Copyright 2020-2024 NXP
*
* \file
*
* SPDX-License-Identifier: BSD-3-Clause
********************************************************************************** */

#if defined(gNbu_Hadm_d) && gNbu_Hadm_d == 1
#define BT60_HADM
#endif

#include "board.h"
//#include "soc.h"
#include "ble_general.h"
#include "controller_init.h"
#include "ll_types.h"
#include "controller_api_ll.h"

#if defined(gControllerPreserveXcvrDacTrimValue_d) && gControllerPreserveXcvrDacTrimValue_d
#include "fsl_adapter_flash.h"
#endif

#include "nxp2p4_xcvr.h"
#include "nxp_xcvr_gfsk_bt_0p5_h_0p5_config.h"
#include "nxp_xcvr_coding_config.h"

#if defined(gMWS_Enabled_d) && (gMWS_Enabled_d)
#include "MWS.h"
#include "ble_mws.h"
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

typedef struct
{
  uint8 mode;
  uint8 error_count;
  uint8 warning_count;
  uint8 reserved1;
  uint32 length;
  uint32 nbu_sha1;
}DEBUG_STRUCT_INFO;

static DEBUG_STRUCT_INFO* debug_info = (DEBUG_STRUCT_INFO*)(0xB0000000);

/************************************************************************************
*************************************************************************************
* Private prototypes
*************************************************************************************
************************************************************************************/



#if defined(gControllerPreserveXcvrDacTrimValue_d) && gControllerPreserveXcvrDacTrimValue_d
static void Controller_SaveXcvrDcocDacTrimToFlash(xcvr_DcocDacTrim_t *xcvrDacTrim);
static uint32_t Controller_RestoreXcvrDcocDacTrimFromFlash(xcvr_DcocDacTrim_t *xcvrDacTrim);
#endif


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

#if defined(gControllerPreserveXcvrDacTrimValue_d) && gControllerPreserveXcvrDacTrimValue_d
static void  Controller_SaveXcvrDcocDacTrimToFlash(xcvr_DcocDacTrim_t *xcvrDacTrim)
{
    NV_Init();
	hardwareParameters_t *pHWParams = NULL;
	(void) NV_ReadHWParameters(&pHWParams);
    FLib_MemCpy(&pHWParams->xcvrCal, xcvrDacTrim, sizeof(xcvr_DcocDacTrim_t));
    (void)NV_WriteHWParameters();
}

static uint32_t Controller_RestoreXcvrDcocDacTrimFromFlash(xcvr_DcocDacTrim_t *xcvrDacTrim)
{
    uint32_t status;

    if (FLib_MemCmpToVal(&pHWParams->xcvrCal, 0xFF, sizeof(xcvr_DcocDacTrim_t)))
    {
        status = 1;
    }
    else
    {
        status = 0;
        FLib_MemCpy(xcvrDacTrim, &pHWParams->xcvrCal, sizeof(xcvr_DcocDacTrim_t));
    }

    return status;
}
#endif

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

/*! *********************************************************************************
* \brief   Initializes the XCVR module
*
* \return uint32_t , 0 if successful, error if other values.
*
********************************************************************************** */

uint32_t Controller_RadioInit(void)
{
    xcvrStatus_t status;
#if defined(gControllerPreserveXcvrDacTrimValue_d) && gControllerPreserveXcvrDacTrimValue_d
    uint32_t count = 0;
#endif

#if defined(gControllerPreserveXcvrDacTrimValue_d) && gControllerPreserveXcvrDacTrimValue_d
    xcvr_DcocDacTrim_t  mXcvrDacTrim;
    uint32_t            trim_invalid;
#endif

    const xcvr_config_t *xcvrConfig         = &xcvr_gfsk_bt_0p5_h_0p5_1mbps_full_config;
    const xcvr_coding_config_t *rbmeConfig  = &xcvr_ble_coded_s8_config;

#if defined(gControllerPreserveXcvrDacTrimValue_d) && gControllerPreserveXcvrDacTrimValue_d
	hardwareParameters_t *pHWParams = NULL;
	(void)NV_ReadHWParameters(&pHWParams);
    /* check if reserved memory in HW parameters is at least sizeof xcvr_DcocDacTrim_t */
    assert(sizeof(pHWParams->xcvrCal) >= sizeof(xcvr_DcocDacTrim_t) );

    /* Check if XCVR Trim value is valid */
    trim_invalid = Controller_RestoreXcvrDcocDacTrimFromFlash(&mXcvrDacTrim);

    /* Initialize Radio without trim */
    status = XCVR_InitNoDacTrim(&xcvrConfig, &rbmeConfig);
    assert( status == gXcvrSuccess_c );

    if(trim_invalid != 0U)
    {
        /* XCVR Trim value is not valid. Calculate now. */
        do {
            count++;
            status = XCVR_CalculateDcocDacTrims(&mXcvrDacTrim);
        } while ((gXcvrSuccess_c != status) && (count < gControllerXcvrInitRetryCount_c));

        /* store calculated XCVR Trim value */
        (void)Controller_SaveXcvrDcocDacTrimToFlash(&mXcvrDacTrim);
    }
    else
    {
        /* XCVR Trim value is valid. Restore it. */
        status = XCVR_SetDcocDacTrims(&mXcvrDacTrim);
    }
#else
#ifndef SIMULATOR
    status = XCVR_Init(&xcvrConfig, &rbmeConfig);

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
#endif /* defined(gControllerPreserveXcvrDacTrimValue_d) && gControllerPreserveXcvrDacTrimValue_d */

    return (uint32_t)status;
}

uint32_t Controller_SetNbuVersion(const uint8* repo_digest)
{
  /*MSB of SHA1 is stored in repo_digest[0]*/
  debug_info->nbu_sha1 = repo_digest[0]|(repo_digest[1] << 8U)|(repo_digest[2] << 16U)|(repo_digest[3]<<24U);
  return 0;
}

osa_status_t Controller_Init(const nbuIntf_t* nbuInterface)
{
    /* set BLE as active LL */
    RADIO_CTRL->LL_CTRL &= ~RADIO_CTRL_LL_CTRL_ACTIVE_LL_MASK;

    /* enable sleep clock for SWT TMR */
    CIU2->CIU2_LBC_CTRL |= CIU2_CIU2_LBC_CTRL_LBC_NCO_EN_MASK;

    /* Enable dejit on 3.2Kz clock - required for lowpower and active to avoid irregular duty cycle on the clock */
    CIU2->CIU2_LBC_CTRL |=  (CIU2_CIU2_LBC_CTRL_DEJIT_EN_MASK | CIU2_CIU2_LBC_CTRL_AUTO_DEJIT_MASK);

    NbuHosted_Config(nbuInterface);

    /* TODO: Can not do it here yet, there are still some XBAR access from main_nbu_ll() init.
          will do it first time Idle task is called */
    //PLATFORM_RemoteActiveRel();
#if defined(gMWS_Enabled_d) && (gMWS_Enabled_d)
    MWS_Register(gMWS_BLE_c, MWS_BLE_Callback);
    // When 15.4 Phy becomes active LL then BLE block should be clocked ... 
    // otherways it will loose time sync and also crush when GetInactivity read BLE registers
    RADIO_CTRL->RF_CLK_CTRL |= RADIO_CTRL_RF_CLK_CTRL_BTLL_CLK_EN_OVRD(1);
#endif

#if defined(gNbu_Hadm_d) && gNbu_Hadm_d == 1
    // Set HADM events priority lower than connection events.
    // By default after power on reset, HADM events priority is higher than connection events
    // LL_API_SetHadmPriorityOverConnection(0);
#endif
    
#if defined(gNbuMaxTxPowerDbm_c)
    // set the max tx power to avoid LL API call at power on
    Controller_SetMaxTxPower(gNbuMaxTxPowerDbm_c, gNbuMaxTxPowerLdoTrim_c); 
#endif

    // function below never returns
    main_nbu_ll();

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
            OSA_InterruptDisable();
            
            g_ldo_ant_trim = ldo_ant_trim;

            // Set LDO ANT Trim 
            uint32_t temp_trim;
            temp_trim = XCVR_ANALOG->LDO_1;
            temp_trim &= ~(XCVR_ANALOG_LDO_1_LDO_ANT_TRIM_MASK);
            temp_trim |= XCVR_ANALOG_LDO_1_LDO_ANT_TRIM(g_ldo_ant_trim); 
            XCVR_ANALOG->LDO_1 = temp_trim; 
            
            OSA_InterruptEnable();
        }
    }
    else
    {
        status = gBleInvalidParameter_c;
    }
    return status;
}

/*! *********************************************************************************
* @}
********************************************************************************** */
