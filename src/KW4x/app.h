/*
 * Copyright 2020-2024 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef _APP_H_
#define _APP_H_


#if defined(gUseIpcTransport_d) && (gUseIpcTransport_d == 1)
#include "fsl_component_serial_manager.h"
#endif

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*${macro:start}*/

/*${macro:end}*/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*${prototype:start}*/
void BOARD_InitHardware(void);

#if defined(gUseIpcTransport_d) && (gUseIpcTransport_d == 1)
extern SERIAL_MANAGER_HANDLE_DEFINE(g_IpcSerialHandle);
#endif
/*${prototype:end}*/

#endif /* _APP_H_ */
