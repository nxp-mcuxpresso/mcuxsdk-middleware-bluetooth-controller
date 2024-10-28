/** @file tx_port.h
*
*  @brief <<ToDo:UpdateMe>>
*  
*
*/
/*
 * Copyright 2017-2024 NXP
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */


#ifndef TX_PORT_H
#define TX_PORT_H

#if defined(BUILT_THREADX_6_4) || defined (CPU_KW45B41Z83AFPA_NBU) || defined (CPU_KW47B42ZB7AFTA_cm33_core1) || defined (CPU_MCXW727CMFTA_cm33_core1)
#if defined (TARGET_CPU_CORTEX_M3) || defined (CPU_KW45B41Z83AFPA_NBU)
#include "tx_port_6_4_cm3.h" /* CM3 version of tx_port.h from official Azure-Rtos Threadx version 6.4 */
#elif defined(TARGET_CPU_CORTEX_M33) || defined (CPU_KW47B42ZB7AFTA_cm33_core1) || defined (CPU_MCXW727CMFTA_cm33_core1)
#include "tx_port_6_4_cm33.h" /* CM33 version of tx_port.h from official Azure-Rtos Threadx version 6.4 */
#else
#error "Please define achitecture of your chip!"
#endif
#elif defined(BUILT_THREADX_5_5)
#include "tx_port_5_5.h"
#elif defined(BUILT_THREADX_5_3)
#include "tx_port_5_3.h"
#else
#error "Please define threadx version for your chip!"
#endif

#endif /*TX_PORT_H*/
