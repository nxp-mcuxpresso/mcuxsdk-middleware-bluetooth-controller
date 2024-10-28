/*
 * Copyright 2020-2024 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*${header:start}*/
#include "pin_mux.h"
#include "board.h"
#include "fsl_component_serial_manager.h"

static const uint32_t froPostDivFreq[] = {
    16000000U, 24000000U, 32000000U, 48000000U, 64000000U
};

void BOARD_SystemCoreClockUpdate(void)
{
    uint32_t froPostDivSel = (FRO192M0->FROCCSR & FRO192M_FROCCSR_POSTDIV_SEL_MASK) >> FRO192M_FROCCSR_POSTDIV_SEL_SHIFT;

    SystemCoreClock = froPostDivFreq[froPostDivSel];
}

uint32_t BOARD_GetSystemCoreClockFreq(void)
{
    BOARD_SystemCoreClockUpdate();
    return SystemCoreClock;
}


/*${function:end}*/
