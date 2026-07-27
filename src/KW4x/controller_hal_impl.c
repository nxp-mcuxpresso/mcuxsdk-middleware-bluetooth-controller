/*
 * Copyright 2026 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
/*
 * Implementation of controller abstraction layer defined in LL controller_hal.h.
 */
/*
                                                                                                             
      ┌─────────────────────────┐                  ┌───────────────────────────────────────────────────┐     
      │                         │                  │                                                   │     
      │                         │                  │ ┌──────────────────────┐        ┌─────────────┐   │     
      │      LL library         │ controller_hal.h │ │                      ┼───────►│             │   │     
      │                         │──────────────────┼►│controller_hal_impl.c │        │Framework    │   │     
      │                         │                  │ │                      │        │             │   │     
      │                         │                  │ └─────────────────────┬┘        └─────────────┘   │     
      │                         │                  │                       │         ┌─────────────┐   │     
      │                         │                  │                       │         │             │   │     
      │                         │ controller_api_ll.h                      └────────►│ nbu_ble     │   │     
      │                         │◄─────────────────┼─────────────────────────────────┤             │   │     
      └─────────────────────────┘                  │                                 └─────────────┘   │     
                                                   │                                       NBU code    │     
                                                   │                                                   │     
                                                   └───────────────────────────────────────────────────┘     
                                                                                                             
                                                                                                             
*/
/*******************************************************************************
 * Incude
 ******************************************************************************/
#include "fsl_common.h"
#include "fsl_port.h"
#include "ll_types.h"
#include "controller_hal.h"
#include "fwk_debug_struct.h"
#include "fwk_platform.h"
#include "nxp2p4_xcvr.h"
#include "fsl_os_abstraction.h"


/*******************************************************************************
 * Types & defines
 ******************************************************************************/

#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470)
 /* Antenna switching context */
typedef struct {
    uint8_t state;                /* antenna switching state */
    uint8_t lant_ctrl;            /* localization antenna control */
    uint32_t  bkp_lcl_cfg0;       /* LCL_CFG0 backup */
    uint32_t  bkp_lcl_gpio_ctrl0; /* LCL_GPIO_CTRL0 backup */
} antenna_switching_t;

#define ANTENNA_SWITCH_STATE_DISABLED                   0U
#define ANTENNA_SWITCH_STATE_ENABLED_DEACTIVATED        1U        
#define ANTENNA_SWITCH_STATE_ENABLED_ACTIVATED          2U
#endif // defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470)

/*******************************************************************************
 * Variables
 ******************************************************************************/

#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470)
 /* antenna switching disabled by default */
static antenna_switching_t antenna_switching;
#endif // defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
extern void NbuGetRevCtrlStr(uint32_t *str_addr, uint32_t *str_len);

/*******************************************************************************
 * Functions
 ******************************************************************************/

/*!
 * return the location and size where LL can store its debug data.
 */
void Controller_GetDebugStructData(void **debug_struct_ptr, uint16 *debug_struct_size)
{
    *debug_struct_ptr = (void *)NBUDBG_BLE_STRUCT;
    *debug_struct_size = NBUDBG_BLE_STRUCT_SIZE;
}

/*!
 * return the DMEM base address and size.
 */
void Controller_GetDMemConfig(uint32_t *mem_start, uint32_t *mem_sz)
{
    PLATFORM_GetDMemConfig(mem_start, mem_sz);
}

/*!
 * return the Shared Memory (SMU) base address and size.
 */
void Controller_GetSharedMemConfig(uint32_t *mem_start, uint32_t *mem_sz)
{
    PLATFORM_GetSharedMemConfig(mem_start, mem_sz);
}

/*!
 * return the start address and size of the controller revision control string.
 */
void Controller_GetRevCtrlStr(uint32_t *str_addr, uint32_t *str_sz)
{
    NbuGetRevCtrlStr(str_addr, str_sz);
}

/*!
 * Configure the XCVR for RSSI measurement on the frequency configured by the LL
 * return in case of sucess a positive number for the radio warmup time in us
 *        otherwise -1 to indicate a failure
 */
int32_t Controller_RssiMeasStart(void)
{
    xcvrStatus_t status;
    uint32 wu_delay;
    status = XCVR_RssiEstimate(true);
    // return warmup delay in us if success
    wu_delay = (XCVR_TSM->END_OF_SEQ & XCVR_TSM_END_OF_SEQ_END_OF_RX_WU_MASK) >> XCVR_TSM_END_OF_SEQ_END_OF_RX_WU_SHIFT;
    return status == gXcvrSuccess_c ? (int32)wu_delay:-1;
}

/*!
 * Perform the RSSI measurement
 * return the RSSI measurement in dBm
 */
int16_t Controller_RssiMeas(void)
{
    return XCVR_GetRssiResult();
}

/*!
 * Restore the XCVR settings and warmdown the radio
 * return true if success, false if failure
 */
boolean Controller_RssiMeasStop(void)
{
    xcvrStatus_t status;
    status = XCVR_RssiEstimate(false);
    return status == gXcvrSuccess_c ? true:false;
}

#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470)
void Controller_AntennaSwitchingConfigure(uint8 enable, uint8 lant_ctrl)
{
    /* API called from rpmsg nbu req, with priority lower than LL, so 
       Controller_AntennaSwitchingActivate and Controller_AntennaSwitchingDeactivate 
       are never interrupted by this call. 
       But protection is necessary as this API can be interrupted by LL. */
    OSA_DisableIRQGlobal();
    if( antenna_switching.state == ANTENNA_SWITCH_STATE_ENABLED_ACTIVATED )
    {
        /* disabling first */
        Controller_AntennaSwitchingDeactivate();
        antenna_switching.state = ANTENNA_SWITCH_STATE_DISABLED;
    }
    
    if( enable )
    {
        antenna_switching.lant_ctrl = lant_ctrl;
        antenna_switching.state = ANTENNA_SWITCH_STATE_ENABLED_DEACTIVATED;
    }
    OSA_EnableIRQGlobal();
}
#endif /* defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470) */

void Controller_AntennaSwitchingActivate(uint8 antenna_number)
{
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470)
    if( antenna_switching.state == ANTENNA_SWITCH_STATE_ENABLED_DEACTIVATED )
    {
        uint32_t temp = XCVR_MISC->LCL_CFG0;
        
        /* backup LCL_CFG0 */
        antenna_switching.bkp_lcl_cfg0 = temp;
        
        /* set LCL_MODE and clear LCL_ANT_PERMUT_EN in LCL_CFG0 */
        temp |= XCVR_MISC_LCL_CFG0_LCL_MODE_MASK;
        temp &= ~XCVR_MISC_LCL_CFG0_LCL_ANT_PERMUT_EN_MASK;
        XCVR_MISC->LCL_CFG0 = temp;
        
        /* backup LCL_GPIO_CTRL0 */
        antenna_switching.bkp_lcl_gpio_ctrl0 = XCVR_MISC->LCL_GPIO_CTRL0;
        if ( antenna_number == 0U )
        {
            /* 1st antenna */
            XCVR_MISC->LCL_GPIO_CTRL0 = 0U;
        }
        else
        {
            /* second antenna */
            XCVR_MISC->LCL_GPIO_CTRL0 = antenna_switching.lant_ctrl;
        }
        antenna_switching.state = ANTENNA_SWITCH_STATE_ENABLED_ACTIVATED;
    }
#else
    (void)antenna_number;
#endif /* defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470) */
}

void Controller_AntennaSwitchingDeactivate(void)
{
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470)
    if( antenna_switching.state == ANTENNA_SWITCH_STATE_ENABLED_ACTIVATED )
    {
        // restore to previous values
        XCVR_MISC->LCL_GPIO_CTRL0 = antenna_switching.bkp_lcl_gpio_ctrl0;
        XCVR_MISC->LCL_CFG0 = antenna_switching.bkp_lcl_cfg0;
        antenna_switching.state = ANTENNA_SWITCH_STATE_ENABLED_DEACTIVATED;
    }
#endif /* defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470) */
}

