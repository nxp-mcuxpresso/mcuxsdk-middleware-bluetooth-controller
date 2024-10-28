/*
 * Copyright 2021-2024 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef _APP_PREINCLUDE_H_
#define _APP_PREINCLUDE_H_

/*! *********************************************************************************
 *     Configuration
 ***********************************************************************************/

/* Enable MWS (Mobile Wireless System) coexistence at protocol level */
#define gMWS_Enabled_d          0

/*  Enables NbuDbg module (need to generate the project with --debug-mode)
 *  This will enable debug IO toggling by the LL , logging and dtest*/
#define gDbg_Enabled_d          0

/* Force disabling lowpower on CM3 - Even if set to 0, CM33 requires to enable Radio domain lowpower
    by gPLATFORM_DisableNbuLowpower_d to 0 on Cm33 project  */
#define gNbuDisableLowpower_d   0

/* Uncomment to avoid issue while debugging (disable Lowpower and WFI execution in idle task) */
//#define gNbuJtagCapability      1

#if defined(gNbu_Hadm_d) && (gNbu_Hadm_d == 1)
#undef gNbuJtagCapability 
#define gNbuJtagCapability 1
#endif

/* Disable clock management on NBU (supposed to be handled on host CPU) */
#define FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL (1)


#define gEnableCoverage                        0
#if (defined(gEnableCoverage) && (gEnableCoverage == 1))
/*Coverage does not need MemBuffer*/
#define PoolsDetails_c _block_set_(32, 1, 0) _eol_
#else
#define PoolsDetails_c _block_set_(64, 1, 0) _eol_ _block_set_(128, 1, 1) _eol_ _block_set_(256, 1, 1) _eol_
#endif

/* Extend Heap usage beyond the size defined by MinimalHeapSize_c*/
#define MinimalHeapSize_c        (uint32_t)1024
#define gMemManagerLightExtendHeapAreaUsage 0
#endif /* _APP_PREINCLUDE_H_ */
