/*
 * Copyright 2022-2024 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#ifndef _BLE_MWS_H_
#define _BLE_MWS_H_

uint32_t MWS_BLE_Callback (mwsEvents_t event);

mwsStatus_t MWS_Acquire     (mwsProtocols_t protocol, uint8 force);
mwsStatus_t MWS_Release     (mwsProtocols_t protocol);

void     MWS_BleReset(void);
uint32_t MWS_BleSetAdvEnable(uint32_t enabled);
uint32_t MWS_BleSetScanEnable(uint32_t enabled);
uint32_t MWS_BleSetConnEnable(uint32_t enabled);

#endif /* _BLE_MWS_H_ */
