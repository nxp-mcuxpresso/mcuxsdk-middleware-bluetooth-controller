/**
 ********************************************************************************
 * @brief
 *
 *
 ******************************************************************************
 * @copy
 *
 *COPYRIGHT 2025 SYNOPSYS, Inc. This Synopsys "product" and all associated documentation
 *are proprieytary to Synopsys, Inc. and may only be used pursuant to the terms and
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
 *   <td><b> PLEASE UPDATE with when date </b></td>
 *   <td><b> PLEASE UPDATE with who made the changes </b></td>
 *   <td><b> PLEASE UPDATE WITH what, where, why the changes were made</b></td>
 * </tr>
 * </table>\n
 */
// Abstract: Configuration definition header file for DWC_ble154combo
#define Z_DATA_SYNC_STAGES 2  //not used
#define USE_SLPTMR_CAL 1      //not used
#define USE_SCAN_OUT_MUX 0    //different - not used
#define USE_RNG 1             //new - not used
#define USE_JTAG 1            //not used
#define Z_RESET_SYNC_STAGES 2 //not used
#define USE_AES 1             //not used
#define SW_ACT_HIGH 1         //not used
#define ANT 0                 //different - not used
#define BUS_CLK 32            //not used
#define PHY_TYPE 1            //not used
#define BLE_MAC_ROLE 2        //different - not used
#define BLE_LL_ROLE 1         //not used
#define ACT_CLK 16            //different - used
#define DF_NO_ANTENNAS 4      //used
#define LST_ADDR 7            //used
#define SEQ_RAM_ADDR_WIDTH 7  //not used
#define BLE_STANDARD 3        //not used
#define JTAG_CLK 16           //not used
#define DATA_RAM_ADDR_WIDTH 12//not used
#define POW_GUARD_TIME 5      //not used
