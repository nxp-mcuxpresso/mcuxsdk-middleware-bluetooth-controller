/*
 * Copyright 2020-2025 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*${header:start}*/
#include "board.h"
#include "fwk_platform_definitions.h"

#if !defined(FWK_KW43_MCXW70_NBU_FAMILIES) || (FWK_KW43_MCXW70_NBU_FAMILIES == 0)
static const uint32_t froPostDivFreq[] = {
    16000000U, 24000000U, 32000000U, 48000000U, 64000000U
};

uint32_t BOARD_GetSystemCoreClockSel(void)
{
    return (FRO192M0->FROCCSR & FRO192M_FROCCSR_POSTDIV_SEL_MASK) >> FRO192M_FROCCSR_POSTDIV_SEL_SHIFT;
}
#endif

void BOARD_SystemCoreClockUpdate(void)
{
#if !defined(FWK_KW43_MCXW70_NBU_FAMILIES) || (FWK_KW43_MCXW70_NBU_FAMILIES == 0)
    uint32_t froPostDivSel = BOARD_GetSystemCoreClockSel();

    if (froPostDivSel < (sizeof(froPostDivFreq) / sizeof(froPostDivFreq[0])))
    {
        SystemCoreClock = froPostDivFreq[froPostDivSel];
    }

#endif
}

uint32_t BOARD_GetSystemCoreClockFreq(void)
{
#if !defined(FWK_KW43_MCXW70_NBU_FAMILIES) || (FWK_KW43_MCXW70_NBU_FAMILIES == 0)
    BOARD_SystemCoreClockUpdate();
#endif
    return SystemCoreClock;
}

/*${function:end}*/
