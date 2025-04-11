/* Copyright 2021-2025 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "controller_api.h"
#include "controller_interface.h"
#include "ble_controller.h"
#include "nxp_hoststack_adapter.h"

osa_status_t Controller_TaskInit(void)
{
    return KOSA_StatusSuccess;
}

osa_status_t Controller_Init(gHostHciRecvCallback_t callback)
{
    if (kBLEC_Success != BLEController_Init(NXPHoststackAdapter_hciControllerToHost,
        gAppMaxTxPowerDbm_c, NULL))
    {
        return KOSA_StatusError;
    }
    return KOSA_StatusSuccess;
}

osa_status_t Controller_SetTxPowerLevel(uint8_t level, txChannelType_t channel)
{
    blec_result_t result = kBLEC_Success;

    switch(channel)
    {
    case gAdvTxChannel_c:
        result = BLEController_SetTxPowerDbm(level);
        break;
    case gConnTxChannel_c:
        result = BLEController_SetConnectionInitialTxPowerDbm(level);
        break;
    default:
        break;
    }

    return (result == kBLEC_Success) ? KOSA_StatusSuccess : KOSA_StatusError;
}

bleResult_t Controller_SendPacketToController( hciPacketType_t packetType, void* pPacket,
                                        uint16_t packetSize)
{
    return NXPHoststackAdapter_hciHostToController(packetType, pPacket, packetSize);
}

osa_status_t Controller_SetRandomSeed()
{
    /* Not relevant for Synopsys LL */
    return KOSA_StatusSuccess;
}

osa_status_t Controller_GetTimestampEx(uint32_t* ll_timing_slot, uint16_t* ll_timing_us, uint64_t *tstmr)
{
    /* Not relevant for Synopsys LL */
    *ll_timing_slot=0;
    *ll_timing_us=0;
    *tstmr=0;
    return KOSA_StatusSuccess;
}

osa_status_t Controller_SetTxPowerLevelDbm(int8_t level_dbm, txChannelType_t channel)
{
    /* Not relevant for Synopsys LL */
    return KOSA_StatusSuccess;
}

osa_status_t Controller_SetMaxTxPower(int8_t power_dBm, uint8_t ldo_ant_trim)
{
    /* Not relevant for Synopsys LL */
    return KOSA_StatusSuccess;
}

osa_status_t Controller_ConfigureInvalidPduHandling(uint32_t pdu_handling_type)
{
    /* Not relevant for Synopsys LL */
    return KOSA_StatusSuccess;
}

osa_status_t Controller_ConfigureAdvCodingScheme(advCodingScheme_t codingSch, uint8_t handle)
{
    /* Not relevant for Synopsys LL */
    return KOSA_StatusSuccess;
}

osa_status_t Controller_SetConnNotificationMode(uint32_t mode)
{
    /* Not relevant for Synopsys LL */
    return KOSA_StatusSuccess;
}