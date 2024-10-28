/*
 * Copyright 2024 NXP
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* Prevent double inclusion */
#ifndef _LCL_XCVR_SIMU_H_
#define _LCL_XCVR_SIMU_H_

/* === Includes ============================================================ */
#include "nxp_xcvr_lcl_ctrl.h"
#include "nxp_xcvr_lcl_step_structs.h"

xcvrLclStatus_t SIMU_LCL_RsmGo(XCVR_RSM_RXTX_MODE_T role, const xcvr_lcl_rsm_config_t * rsm_settings_ptr);

#endif /* _LCL_XCVR_SIMU_H_ */

/* EOF */
