/*
 * Copyright 2023-2025 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _NXP_HOSTSTACK_ADAPTER_H_
#define _NXP_HOSTSTACK_ADAPTER_H_

/************************************************************************************
* Include
************************************************************************************/

#include "ble_general.h"
#include "ble_controller.h"

/************************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
************************************************************************************/

/*! *********************************************************************************
 * \brief  Host-to-Controller API implementation
 * \param[in]  packetType HCI packet Type
 * \param[in]  pPacket    data buffer
 * \param[in]  packetSize data buffer length
 *
 * \return bleResult_t
 ********************************************************************************** */
bleResult_t NXPHoststackAdapter_hciHostToController(hciPacketType_t packetType, void* pPacket,
        uint16_t packetSize);

/*! *********************************************************************************
 * \brief  Controller-to-Host API implementation
 * \param[in]  packetType HCI packet Type
 * \param[in]  pPacket    data buffer
 * \param[in]  packetSize data buffer length
 *
 * \return blec_result_t
 ********************************************************************************** */
blec_result_t NXPHoststackAdapter_hciControllerToHost(blec_hciPacketType_t packetType, void* pPacket,
        uint16_t packetSize);

#endif /*_NXP_HOSTSTACK_ADAPTER_H_*/
