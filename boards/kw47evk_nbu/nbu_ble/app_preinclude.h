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

/* Enable NbuDbg module (need to generate the project with --debug-mode)
 *  This will enable debug IO toggling by the LL , logging and dtest*/
#define gDbg_Enabled_d          0

/* Enable debug throught SWO trace */
#define gDbg_SwoEnabled_d       0

#if (defined(gDbg_SwoEnabled_d) && (gDbg_SwoEnabled_d == 1))

/* Define to 1 if you want to configure the DWT/ITM/TPIU-SWO via SW i.s.o.the probe */
#define DBG_SWO_INIT_VIA_SW 1

/* CoreSight Funnel SWO route Muxing configuration    */
/* Setting shall be done on Main Application Core PPB */
#define DBG_SWO_CORE_MAIN_CORE 1 // Main Core
#define DBG_SWO_CORE_NBU_CORE  2 // Nbu Core
#define DBG_SWO_FUNNEL_MUXING DBG_SWO_CORE_NBU_CORE

#endif

/* Enable dump of debug info into NBU RAM log if fwk_fault_handlers.c is added to the project (internal use only)
 * To add fwk_fault_handlers.c into NBU project, add the following Kconfig to project prj.conf file
 *  CONFIG_MCUX_COMPONENT_middleware.wireless.framework_private=y
 *  CONFIG_MCUX_COMPONENT_middleware.wireless.framework_private.dbg=y
 *  CONFIG_MCUX_COMPONENT_middleware.wireless.framework_private.dbg.fault_handlers=y */
//#define gDBG_LogInLinkLayerDebugStructEnabled_d 1

/* Force disabling lowpower on CM3 - Even if set to 0, CM33 requires to enable Radio domain lowpower
    by gPLATFORM_DisableNbuLowpower_d to 0 on Cm33 project  */
#define gNbuDisableLowpower_d   0

/* Uncomment to avoid issue while debugging (disable Lowpower and WFI execution in idle task) */
#ifndef NDEBUG
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
