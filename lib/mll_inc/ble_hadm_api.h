/*                Copyright 2021-2024, NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 *@file         hadm_apis.h
 *
 *\brief        This header file holds HADM prototypes mainly used by NBU.
 *
 */
#ifndef __HADM_API_H_
#define __HADM_API_H_

/* === Includes ============================================================= */

#include "ble_hadm_types.h"

/* === Variables  =========================================================== */


/* === Prototypes =========================================================== */

/*!
 * \brief API called before each event start to request if LL continues the event. 
 *        If HADM_HAL_ABORTED is returned, LL will not perform the event.
 * 
 * \param[in] pConfig             pointer of the event configuration
 * \param[in] startTimeHSlot      start time in half slot of the first event (unit 625us)
 * \param[in] startTimeOffsetUs   start time offset in the slot the first event (unit us)
 * 
 * \return    HADM_HAL_SUCCESS indicates LL continuing the event, HADM_HAL_ABORTED otherwise 
 */
BLE_HADM_STATUS_t BLE_HADM_SubeventContinue(BLE_HADM_SubeventConfig_t *pConfig, uint32 startTimeHSlot, uint32 startTimeOffsetUs);

/*!
 * \brief API called before each procedure instance start to request if LL continues the current procedure instance. 
 *        If HADM_HAL_ABORTED is returned, LL will not perform the procedure instance.
 * 
 * \param[in] pProc               pointer of the procedure configuration
 * \param[in] startTimeHSlot      start time in half slot of the first event (unit 625us)
 * \param[in] startTimeOffsetUs   start time offset in the slot the first event (unit us)
 * 
 * \return    HADM_HAL_SUCCESS indicates LL continuing the procedure, HADM_HAL_ABORTED otherwise 
 */
BLE_HADM_STATUS_t BLE_HADM_ProcedureContinue(const TBleHadmConnection_t *pProc, uint32 startTimeHSlot, uint32 startTimeOffsetUs);

#endif // #ifndef __HADM_API_H_
