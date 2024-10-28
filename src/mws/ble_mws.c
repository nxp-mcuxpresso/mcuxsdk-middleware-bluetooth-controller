/*
 * Copyright 2022-2024 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "controller_init.h"
#include "ll_types.h"
#include "controller_api_ll.h"

#include "nxp2p4_xcvr.h"
#include "nxp_xcvr_gfsk_bt_0p5_h_0p5_config.h"
#include "nxp_xcvr_coding_config.h"

#include "MWS.h"
#include "ble_mws.h"

#if defined(gMWS_Enabled_d) && (gMWS_Enabled_d)

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* Active states used by hybrid mode */
#define MWS_STATE_INACTIVE                                      0x00
#define MWS_STATE_ACTIVE                                        0x01

#define MWS_EXTERNAL_ACQUIRED                                   0x00
#define MWS_BLE_ACQUIRED                                        0x01

#define MWS_BLE_IDLE                                            0x00
#define MWS_BLE_ACTIVE_ADV                                      0x01
#define MWS_BLE_ACTIVE_SCAN                                     0x02
#define MWS_BLE_ACTIVE_CONN                                     0x04
#define MWS_BLE_ACTIVE_DTM                                      0x08

#define MWS_BLE_ACQUIRE                                         0x00
#define MWS_BLE_FORCE_ACQUIRE                                   0x01

#define MWS_BLE_NO_ABORT                                        0x00
#define MWS_BLE_ABORTED                                         0x01


/*******************************************************************************
 * Prototypes
 ******************************************************************************/


/*******************************************************************************
 * Variables
 ******************************************************************************/
typedef struct
{
    volatile uint8_t active_status;
    volatile uint8_t mws_acquired;
    volatile uint8_t ble_active;
    volatile uint8_t force_acquire;
    volatile uint8_t ble_aborted;
    volatile uint8_t data_paused;
} PL_DEVICE_MWS_MGR;

volatile PL_DEVICE_MWS_MGR pf_mws_mgr =
{
    .active_status = MWS_STATE_INACTIVE,
    .mws_acquired = MWS_EXTERNAL_ACQUIRED,
    .ble_active = MWS_BLE_IDLE,
    .force_acquire = MWS_BLE_ACQUIRE,
    .ble_aborted = MWS_BLE_NO_ABORT,
    .data_paused = 0
};

/*******************************************************************************
 * Private functions
 ******************************************************************************/

static uint32_t MWS_SetBleEnable(uint32_t enabled, uint32_t mode)
{
    uint32_t status = gMWS_Success_c;

    if( pf_mws_mgr.active_status == MWS_STATE_ACTIVE )
    {
        OSA_InterruptDisable();

        if( enabled != 0U )
        {
            /* BLE activity is about to start */
            /* Set BLE status active */
            pf_mws_mgr.ble_active |= (uint8_t)mode;

            if (pf_mws_mgr.mws_acquired != MWS_BLE_ACQUIRED)
            {
                /* Try to acquire the transceiver. */
                status = MWS_Acquire(gMWS_BLE_c, TRUE);
                if (gMWS_Success_c == status)
                {
                    pf_mws_mgr.mws_acquired = MWS_BLE_ACQUIRED;
                    pf_mws_mgr.force_acquire = MWS_BLE_FORCE_ACQUIRE;
                }
            }
        }
        else
        {
            /* BLE activity has just stopped */
            /* Clear MWS status active */
            pf_mws_mgr.ble_active &= ~(uint8_t)mode;

            /* If BLE is idle, release the MWS interface */
            if ((MWS_BLE_ACQUIRED == pf_mws_mgr.mws_acquired) && (pf_mws_mgr.ble_active == MWS_BLE_IDLE))
            {
                pf_mws_mgr.mws_acquired = MWS_EXTERNAL_ACQUIRED;
                pf_mws_mgr.force_acquire = MWS_BLE_ACQUIRE;
                MWS_Release(gMWS_BLE_c);
            }
        }

        OSA_InterruptEnable();
    }

    return status;
}

/*******************************************************************************
 * Public functions
 ******************************************************************************/

uint32_t Controller_GetInactivityDuration(void)
{
    /* convert from half slot to slot (625us) */
    return LL_SCHED_GetSleepTime()>>1U;
}

uint32_t MWS_BLE_Callback (mwsEvents_t event)
{
    uint32_t status = gMWS_Success_c;

    switch(event)
    {
        case gMWS_Init_c:

            OSA_InterruptDisable();

            /* enable MWS for BLE */
            pf_mws_mgr.active_status = MWS_STATE_ACTIVE;

            OSA_InterruptEnable();
            break;

        case gMWS_Active_c:

            OSA_InterruptDisable();

            if ( (RADIO_CTRL_LL_CTRL_ACTIVE_LL_MASK & RADIO_CTRL->LL_CTRL) != 0 )
            {
                const xcvr_config_t *xcvrConfig = &xcvr_gfsk_bt_0p5_h_0p5_1mbps_full_config;
                const xcvr_coding_config_t *rbmeConfig = &xcvr_ble_coded_s8_config;

                /* Setup all of the XCVR registers */
                XCVR_RadioRegSetup(&xcvrConfig);
                XCVR_RBME_Configure(&rbmeConfig);

                /*
                 * In the process of dynamically changing modes between BLE and 15.4 Phy,
                 * 15.4 Phy Layer may change LDO_ANT_TRIM value
                 * restore LDO ANT TRIM to value maintained and expected by BLE
                 */
                extern void Controller_RestoreLdoAntTrim(void);
                Controller_RestoreLdoAntTrim();

                /* enable BLE LL */
                XCVR_SetActiveLL(XCVR_ACTIVE_LL_BTLE);
            }

            if (MWS_BLE_ABORTED == pf_mws_mgr.ble_aborted)
            {
                pf_mws_mgr.ble_aborted = MWS_BLE_NO_ABORT;
            }

            pf_mws_mgr.mws_acquired = MWS_BLE_ACQUIRED;

            if (pf_mws_mgr.data_paused)
            {
                pf_mws_mgr.data_paused = 0;
            }

            OSA_InterruptEnable();

            break;

        case gMWS_Idle_c:

            break;

        case gMWS_Abort_c:

            OSA_InterruptDisable();
            pf_mws_mgr.mws_acquired = MWS_EXTERNAL_ACQUIRED;
            pf_mws_mgr.ble_aborted = MWS_BLE_ABORTED;
            OSA_InterruptEnable();
            break;

        case gMWS_GetInactivityDuration_c:

            OSA_InterruptDisable();
            status = Controller_GetInactivityDuration();
            status = status * 625; // transform into microseconds.

            if (status)
            {
              if ( status >= 625*6+1)
              {
                status -= 625*6; // allow before at least 6 slots (625 uS) for ble
              }
              else
              {
                status = 1;
              }
            }

            OSA_InterruptEnable();
            break;

        case gMWS_Release_c:

            break;

        default:

            status = (uint32_t)(gMWS_InvalidParameter_c);
            break;
    }

    return status;
}

void MWS_BleReset(void)
{
    if (MWS_STATE_ACTIVE == pf_mws_mgr.active_status)
    {
        OSA_InterruptDisable();

        pf_mws_mgr.ble_active = MWS_BLE_IDLE;

        if (MWS_BLE_ACQUIRED == pf_mws_mgr.mws_acquired)
        {
            pf_mws_mgr.mws_acquired = MWS_EXTERNAL_ACQUIRED;
            pf_mws_mgr.force_acquire = MWS_BLE_ACQUIRE;
            MWS_Release(gMWS_BLE_c);
        }

        OSA_InterruptEnable();
    }
}

uint32_t MWS_BleSetAdvEnable(uint32_t enabled)
{
    return MWS_SetBleEnable(enabled, MWS_BLE_ACTIVE_ADV);
}

uint32_t MWS_BleSetScanEnable(uint32_t enabled)
{
    return MWS_SetBleEnable(enabled, MWS_BLE_ACTIVE_SCAN);
}

uint32_t MWS_BleSetConnEnable(uint32_t enabled)
{
    return MWS_SetBleEnable(enabled, MWS_BLE_ACTIVE_CONN);
}

#endif /* #if defined(gMWS_Enabled_d) && (gMWS_Enabled_d) */
