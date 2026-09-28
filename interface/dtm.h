/*!
 * Copyright 2021-2024 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __DTM_H__
#define __DTM_H__

#include "ble_general.h"
#include "fsl_component_serial_manager.h"

/*****************************************************************************
*****************************************************************************
* Public prototypes
*****************************************************************************
*****************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

bleResult_t DTM_Init(serial_handle_t dtm_handle);
void DTM_Uninit(void);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* __DTM_H__ */