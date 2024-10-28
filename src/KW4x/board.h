/*
 * Copyright 2020-2024 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _BOARD_H_
#define _BOARD_H_

#include <stdint.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @brief The board name */
#define BOARD_NAME "K32WB41EVK_NBU"

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/*******************************************************************************
 * API
 ******************************************************************************/

/**
 * @brief Updates SystemCoreClock global variable
 *
 */
void BOARD_SystemCoreClockUpdate(void);

/**
 * @brief Updates and returns current SystemCoreClock frequency
 *
 * @return uint32_t SystemCoreClock frequency
 */
uint32_t BOARD_GetSystemCoreClockFreq(void);

#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* _BOARD_H_ */
