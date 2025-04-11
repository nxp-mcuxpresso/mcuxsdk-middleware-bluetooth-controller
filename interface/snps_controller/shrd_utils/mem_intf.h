/**
 ********************************************************************************
 * @file    mem_intf.h
 * @brief   This file contains all the functions prototypes for the mem_intf.c.
 ******************************************************************************
 * @copy
 *
 *COPYRIGHT 2025 SYNOPSYS, Inc. This Synopsys "product" and all associated documentation 
 *are proprietary to Synopsys, Inc. and may only be used pursuant to the terms and 
 *conditions of a written license agreement with Synopsys, Inc or with a licensee of Synopsys 
 *who has been granted by Synopsys the right to grant access to this Synopsys "product" 
 *and associated documentation to specified users. All other use, 
 *reproduction, modification, or distribution of the Synopsys "product" or the associated 
 *documentation is strictly prohibited.
 *
 *
 * THE ENTIRE NOTICE ABOVE MUST BE REPRODUCED ON ALL AUTHORIZED COPIES.
 *
 * <h2><center>&copy; (C) COPYRIGHT 2025 SYNOPSYS, INC.</center></h2>
 * <h2><center>&copy; 	ALL RIGHTS RESERVED</center></h2>
 *
 * \n\n<b>References</b>\n
 * -Documents folder .
 *
 * <b>Edit History For File</b>\n
 *  This section contains comments describing changes made to this file.\n
 *  Notice that changes are listed in reverse chronological order.\n
 * <table border>
 * <tr>
 *   <td><b> when </b></td>
 *   <td><b> who </b></td>
 *   <td><b> what, where, why </b></td>
 * </tr>
 * <tr>
 * </tr>
 * </table>\n
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MEM_INTF_H
#define MEM_INTF_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
/* Defination ----------------------------------------------------------------*/
/* Exported variables ------------------------------------------------------- */
/* Exported types ------------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

/** @addtogroup Memory_Interface_Exported_Functions
 * @{
 */

/**
 * @brief  Coping memory from position to another.
 * @param  ptr_dstntion	: pointer to the destination array where the content is to be copied.
 * @param  ptr_src	: pointer to the source of data to be copied.
 * @param  n 		: the number of bytes to be copied.
 * @retval pointer to destination.
 */
void *ble_memcpy(
	void *ptr_dstntion,
	const void *ptr_src,
	uint16_t n);

/**
 * @brief  Setting a certian block of memory with a certain value.
 * @param  ptr_mem	: pointer to the block of memory to fill.
 * @param  value	: the value to be set. The value is passed as an int.
 * @param  n 		: the number of bytes to be set to the value.
 * @retval pointer to destination.
 */
void *ble_memset(
	void *ptr_mem,
	uint8_t value,
	uint16_t n);

/**
 * @brief  comparing a certain block of memory with another.
 * @param  ptr_dstntion	: pointer to the destination array where the content is to be compared.
 * @param  ptr_src	: pointer to the source of data to be compared.
 * @param  n 		: the number of bytes to be compared.
 * @retval  	< 0 ptr_dstntion is less than ptr_src.
 * 		> 0 ptr_src is less than ptr_dstntion.
 * 		  0 ptr_dstntion is equal to ptr_src.
 */
int8_t ble_memcmp(
	const void *ptr_dstntion,
	const void *ptr_src,
	uint16_t n);

/**
 * @}
 */
#endif /* MEM_INTF_H */

/******************* (C) COPYRIGHT 2025 SYNOPSYS, INC. *****END OF FILE****/
