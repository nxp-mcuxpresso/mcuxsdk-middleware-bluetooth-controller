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

/*!
 * Get the DMEM base address and size.
 * The implementation should return the platform-specific DMEM configuration.
 */
void Controller_GetDMemConfig(uint32 *mem_start, uint32 *mem_sz);

/*!
 * Get the Shared Memory (SMU) base address and size.
 * The implementation should return the platform-specific shared memory configuration.
 */
void Controller_GetSharedMemConfig(uint32 *mem_start, uint32 *mem_sz);

/*!
 * Get the revision control string from the controller.
 * Returns the start address and size of the controller's version string.
 */
void Controller_GetRevCtrlStr(uint32 *str_addr, uint32 *str_sz);

/*!
 * Configure the XCVR for RSSI measurement on the frequency configured by the LL
 * return in case of sucess a positive number equal to the radio warmup time in us
 *        otherwise failure
 */
int32 Controller_RssiMeasStart(void);

/*!
 * Perform the RSSI measurement after the call to Controller_RssiMeasStart()
 * return the RSSI measurement in dBm
 */
int16 Controller_RssiMeas(void);

/*!
 * Restore the XCVR settings and warmdown the radio
 * return true if success, false if failure
 */
boolean Controller_RssiMeasStop(void);

#endif // CONTROLLER_HAL_H_
