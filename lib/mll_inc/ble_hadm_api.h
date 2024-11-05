/*
*                Copyright 2023, NXP
 *
 *  NXPCONFIDENTIAL
 *  The source code contained or described herein and all documents related to
 *  the source code ( Material ) are owned by NXP its
 *  suppliers or licensors. Title to the Material remains with NXP
 *  or its suppliers and licensors. The Material contains
 *  trade secrets and proprietary and confidential information of NXP its
 *  suppliers and licensors. The Material is protected by worldwide copyright
 *  and trade secret laws and treaty provisions. No part of the Material may be
 *  used, copied, reproduced, modified, published, uploaded, posted,
 *  transmitted, distributed, or disclosed in any way without NXPprior
 *  express written permission.
 *
 *  No license under any patent, copyright, trade secret or other intellectual
 *  property right is granted to or conferred upon you by disclosure or delivery
 *  of the Materials, either expressly, by implication, inducement, estoppel or
 *  otherwise. Any license under such intellectual property rights must be
 *  express and approved by NXP writing.
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
