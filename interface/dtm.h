/*!
 * Copyright 2021-2024 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "ble_general.h"
#include "fsl_component_serial_manager.h"

/*****************************************************************************
*****************************************************************************
* Public prototypes
*****************************************************************************
*****************************************************************************/
bleResult_t DTM_Init(serial_handle_t dtm_handle);
void DTM_Uninit(void);
