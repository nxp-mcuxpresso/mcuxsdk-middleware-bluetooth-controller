/**
 ********************************************************************************
 * @file    ll_mem.h.h
 * @brief   This file contains definitions for pool type used by the LL memory manager.
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

#ifndef INCLUDE_LL_MEM_H
#define INCLUDE_LL_MEM_H

#include "os_wrapper.h"

#ifndef USE_OS_MALLOC
os_Pool_Def_extern(queue_handle_t);
os_Pool_Def_extern(sw_timer_t);
os_Pool_Def_extern(pkt_buff_hdr_t);
os_Pool_Def_extern(hci_buffer_t);
void ll_mem_init(void);
#endif

void ll_mem_shared_init(void);
void ll_mem_shared_check(void);
uint32_t ble_get_hci_transport_evnt_count(void);

#endif
