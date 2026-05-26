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
#include "ll_types.h"
#include "controller_hal.h"
#include "fwk_debug_struct.h"
#include "fwk_platform.h"
#include "nxp2p4_xcvr.h"

/*******************************************************************************
 * Types & defines
 ******************************************************************************/

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
