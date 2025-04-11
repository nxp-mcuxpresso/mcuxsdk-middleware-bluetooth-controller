/**
 ********************************************************************************
 * @file    hci_transport.h
 * @brief   This component for interfacing the hci with bus interface .
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
 *   <td>  Create the file</td>
 * </tr>
 * </table>\n
 */

#ifndef SRC_HCI_TRANSPORT_H_
#define SRC_HCI_TRANSPORT_H_

#include <stdint.h>
#include "common_types.h"

void hci_transport_init(void);
int hci_transport_initialized();
void hci_transport_deinit();
void hci_transport_reset(void);
void hci_transport_send_pckt(ble_buff_hdr_t *ptr_evnt_hdr);
void hci_transport_controller_ready(void);

#endif /* SRC_HCI_TRANSPORT_H_ */
/**
 * @}
 */
/******************* (C) COPYRIGHT 2025 SYNOPSYS, INC. *****END OF FILE****/

