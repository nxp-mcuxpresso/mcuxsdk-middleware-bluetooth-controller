/*! *********************************************************************************
*
* Copyright 2026 NXP
*
* NXP Proprietary. 
* This software is owned or controlled by NXP and may only be used strictly in accordance 
* with the applicable license terms.  By expressly accepting such terms or by downloading, 
* installing, activating and/or otherwise using the software, you are agreeing that you 
* have read, and that you agree to comply with and are bound by, such license terms.  
* If you do not agree to be bound by the applicable license terms, then you may not retain, 
* install, activate or otherwise use the software.
********************************************************************************** */
/*!
    This file defines the Harware Abstraction Layer that the LL uses to access platform-specific services.
    It contains function prototypes that must be implemented by the platform (outside LL library).
*/

#ifndef CONTROLLER_HAL_H_
#define CONTROLLER_HAL_H_

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*!
 * The implementation of this API should return the location and size where LL can store its debug data.
 * The area of debug will be extracted by platform-specific mecanism (unknown from the LL), and should just 
 * be considered as a contiguous storage space by the LL.
 */
void Controller_GetDebugStructData(void **debug_struct_ptr, uint16 *debug_struct_size);

#endif // CONTROLLER_HAL_H_