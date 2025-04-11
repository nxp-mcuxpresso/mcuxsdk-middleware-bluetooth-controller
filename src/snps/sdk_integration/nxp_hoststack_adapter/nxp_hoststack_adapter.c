/*
 * Copyright 2023-2025 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "nxp_hoststack_adapter.h"
#include "ble_controller.h"
#include <fsl_debug_console.h>
#include <assert.h>

#ifdef DEBUG_HCI
#define debug_hci_printf PRINTF
#else
#define debug_hci_printf(...) \
    do                        \
    {                         \
    } while (0)
#endif /* DEBUG_HCI */

static blec_result_t ResultHToC(bleResult_t result)
{
    switch (result)
    {
    case gBleSuccess_c:
        return kBLEC_Success;
    case gHciUnknownHciCommand_c:
        return kBLEC_UnknownHCICommand;
    case gBleOverflow_c:
        return kBLEC_Overflow;
    case gBleInvalidParameter_c:
        return kBLEC_InvalidParameter;
    case gBleOutOfMemory_c:
        return kBLEC_OutOfMemory;
    default:
        return kBLEC_UnknownError;
    }
};

static bleResult_t ResultCToH(blec_result_t result)
{
    switch (result)
    {
    case kBLEC_Success:
        return gBleSuccess_c;
    case kBLEC_UnknownHCICommand:
        return gHciUnknownHciCommand_c;
    case kBLEC_Overflow:
        return gBleOverflow_c;
    case kBLEC_InvalidParameter:
        return gBleInvalidParameter_c;
    case kBLEC_OutOfMemory:
        return gBleOutOfMemory_c;
    default:
        return gBleUnexpectedError_c;
    }
};

static blec_hciPacketType_t PacketTypeHToC(hciPacketType_t hostPacketType)
{
    switch(hostPacketType)
    {
    case gHciCommandPacket_c:
        return kBLEC_HciCommandPacket;
    case gHciDataPacket_c:
        return kBLEC_HciDataPacket;
    case gHciSynchronousDataPacket_c:
        return kBLEC_HciSynchronousDataPacket;
    case gHciEventPacket_c:
        return kBLEC_HciEventPacket;
    default:
        return kBLEC_HciCommandPacket;
    }
};

static hciPacketType_t PacketTypeCToH(blec_hciPacketType_t controllerPacketType)
{
    switch(controllerPacketType)
    {
    case kBLEC_HciCommandPacket:
        return gHciCommandPacket_c;
    case kBLEC_HciDataPacket:
        return gHciDataPacket_c;
    case kBLEC_HciSynchronousDataPacket:
        return gHciSynchronousDataPacket_c;
    case kBLEC_HciEventPacket:
        return gHciEventPacket_c;
    default:
        return gHciCommandPacket_c;
    }
};

bleResult_t NXPHoststackAdapter_hciHostToController(hciPacketType_t packetType, void *pPacket, uint16_t packetSize)
{
    return ResultCToH(BLEController_ProcessHciPacket(
            PacketTypeHToC(packetType),
            pPacket, packetSize));
}

blec_result_t NXPHoststackAdapter_hciControllerToHost(blec_hciPacketType_t packetType,
    void* pPacket,
    uint16_t packetSize)
{
    bleResult_t result = Ble_HciRecv(
            PacketTypeCToH(packetType),
            pPacket, packetSize);

    if (result != gBleSuccess_c) {
        debug_hci_printf("\t Error reported by host. Code 0x%x (check ble_general.h)", result);
    }

    return ResultHToC(result);
}
