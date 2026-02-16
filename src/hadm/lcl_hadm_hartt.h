/**
 * @file lcl_hadm_hartt.h
 *
 * This file implements HARTT services for HADM
 */
/*
 * Copyright 2021-2024 NXP
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* Prevent double inclusion */
#ifndef _LCL_HADM_HARTT_H_
#define _LCL_HADM_HARTT_H_

/* === Includes ============================================================ */

/* === Types =============================================================== */

/* === Macros ============================================================== */

/* === Prototypes ========================================================== */
#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    LCL_HADM_RTT_PHY_1MBPS = 0U,
    LCL_HADM_RTT_PHY_2MBPS = 1U,
    LCL_HADM_RTT_PHY_2MBPS_2BT = 2U,
    LCL_HADM_RTT_PHY_MAX = 3U,
} LCL_HADM_rttPhy_t;

int32_t lcl_hadm_hartt_compute_fractional_delay(const LCL_HADM_rttPhy_t data_rate, const uint32_t pn_seq, int16_t  p_delta, const int32_t int_adj);
#ifdef HARTT_ENABLE_FLOAT
void lcl_hadm_hartt_enable_float(uint8_t en);
#endif

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* _LCL_HADM_HARTT_H_ */

/* EOF */
