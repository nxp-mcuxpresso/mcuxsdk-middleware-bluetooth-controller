/*!
 * Copyright 2021 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "dtm.h"
#include "FunctionLib.h"
#include "hci_transport.h"
#include "hci_types.h"
#include "fsl_component_mem_manager.h"

/*****************************************************************************
*****************************************************************************
* Private macros
*****************************************************************************
*****************************************************************************/
#define gHciLeControllerCommands_c          0x08U
#define mHciLeTestEndCommand_c                  0x1FU
#define mHciLeReadMaxDataLengthCommand_c        0x2FU
#define gHciLeEnhancedReceiverTestCommand_c     0x33U
#define gHciLeEnhancedTransmitterTestCommand_c  0x34U

#define gHciControllerBasebandCommands_c        0x03U
#define mHciResetCommand_c                      0x03U
#define mHciHciResetCommandLength_c             (0U)

#define mHciLeEnhancedTransmitterTestCommandLength_c (4U)
#define mHciLeEnhancedReceiverTestCommandLength_c    (3U)
#define mHciLeReadMaxDataLengthCommandLength_c       (0U)
#define mHciLeTestEndCommandLength_c       (0U)

#define LE_TEST_SETUP_CMD       0U
#define LE_RECEIVER_TEST_CMD    1U
#define LE_TRANSMITTER_TEST_CMD 2U
#define LE_TEST_END_CMD         3U
#define LE_TEST_CONTROL_RESET      0U
#define LE_TEST_CONTROL_DATA_LEN   1U
#define LE_TEST_CONTROL_PHY        2U
#define LE_TEST_CONTROL_MOD_IDX    3U
#define LE_TEST_CONTROL_READ_SUPP_FEAT  4U
#define LE_TEST_CONTROL_READ_MAX_OCTETS 5U

#define HciCommand(opCodeGroup, opCodeCommand)\
    (((uint16_t)(opCodeGroup) & (uint16_t)0x3FU)<<(uint16_t)SHIFT10)|(uint16_t)((opCodeCommand) & 0x3FFU)

/*****************************************************************************
*****************************************************************************
* Private memory
*****************************************************************************
*****************************************************************************/
static uint32_t gDtmSerialReadHandle[((SERIAL_MANAGER_READ_HANDLE_SIZE + sizeof(uint32_t) - 1U) / sizeof(uint32_t))];
static uint32_t gDtmSerialWriteHandle[((SERIAL_MANAGER_WRITE_HANDLE_SIZE + sizeof(uint32_t) - 1U) / sizeof(uint32_t))];
static uint8_t read_max_oct_type = 0U;
static serial_handle_t g_dtmHandle;

/*****************************************************************************
*****************************************************************************
* Private prototypes
*****************************************************************************
*****************************************************************************/
static bleResult_t DTM_send_transmitter_test_hci_cmd(uint8_t tx_channel, uint8_t length, uint8_t payload, uint8_t phy);
static bleResult_t DTM_send_receiver_test_hci_cmd(uint8_t rx_channel, uint8_t phy, uint8_t modulation_idx);
static void DTM_send_test_status_event(uint16_t response, uint8_t status);
static void DTM_send_test_report_event(uint8_t cmd, uint16_t packet_count);
static bleResult_t DTM_send_test_end_hci_cmd(void);
static bleResult_t DTM_send_read_maximum_data_len_hci_cmd(void);
static bleResult_t DTM_send_hci_reset_hci_cmd(void);
static void DTM_serial_manager_tx_callback(void *callbackParam, serial_manager_callback_message_t *message, serial_manager_status_t status);
static void DTM_serial_manager_rx_callback(void *callbackParam, serial_manager_callback_message_t *message, serial_manager_status_t status);
static bleResult_t DTM_hci_recv(hciPacketType_t packetType, void* pHciPacket,uint16_t packetSize);

/*****************************************************************************
*****************************************************************************
* Public functions
*****************************************************************************
*****************************************************************************/
bleResult_t DTM_Init(serial_handle_t dtm_handle)
{
    bleResult_t result;
    g_dtmHandle = dtm_handle;
    /*open read handle*/
    (void)SerialManager_OpenReadHandle(dtm_handle, (serial_read_handle_t)gDtmSerialReadHandle);
    (void)SerialManager_InstallRxCallback((serial_read_handle_t)gDtmSerialReadHandle, DTM_serial_manager_rx_callback, NULL);
    /*open write handle*/
    (void)SerialManager_OpenWriteHandle(dtm_handle, (serial_write_handle_t)gDtmSerialWriteHandle);
    /* configure HCI callback for DTM */
    result = Hcit_Reconfigure(DTM_hci_recv);
    
    if (result != gBleSuccess_c)
    {
        result = DTM_send_hci_reset_hci_cmd();
    }
    return result;
}

void DTM_Uninit(void)
{
    SerialManager_CloseReadHandle(gDtmSerialReadHandle);
    SerialManager_CloseWriteHandle(gDtmSerialWriteHandle);
#if defined(gUseHciTransportDownward_d) && gUseHciTransportDownward_d
    /* restore normal HCI Transport callback */
    Hcit_Reconfigure(Ble_HciRecv);
#endif
}

/*****************************************************************************
*****************************************************************************
* Private functions
*****************************************************************************
*****************************************************************************/
static bleResult_t DTM_send_transmitter_test_hci_cmd(uint8_t tx_channel, uint8_t length, uint8_t payload, uint8_t phy)
{
    uint8_t aHciPacket[mHciLeEnhancedTransmitterTestCommandLength_c + gHciCommandPacketHeaderLength_c];
    uint16_t opcode = HciCommand(gHciLeControllerCommands_c, gHciLeEnhancedTransmitterTestCommand_c);
    bleResult_t result;

    /* Set HCI opcode */
    FLib_MemCpy((void*)aHciPacket, (const void*)&opcode, 2U);
    aHciPacket[2U] = mHciLeEnhancedTransmitterTestCommandLength_c;
    aHciPacket[3U] = tx_channel;
    aHciPacket[4U] = length;
    if (payload == 3U)
    {
      aHciPacket[5U] = 4U;
    }
    else
    {
      aHciPacket[5U] = payload;
    }
    aHciPacket[6U] = phy;
    result = Hcit_SendPacket(gHciCommandPacket_c, aHciPacket, gHciCommandPacketHeaderLength_c + mHciLeEnhancedTransmitterTestCommandLength_c);

    return result;
}

static bleResult_t DTM_send_receiver_test_hci_cmd(uint8_t rx_channel, uint8_t phy, uint8_t modulation_idx)
{
    uint8_t aHciPacket[mHciLeEnhancedReceiverTestCommandLength_c + gHciCommandPacketHeaderLength_c];
    uint16_t opcode = HciCommand(gHciLeControllerCommands_c, gHciLeEnhancedReceiverTestCommand_c);
    bleResult_t result;

    /* Set HCI opcode */
    FLib_MemCpy((void*)aHciPacket, (const void*)&opcode, 2U);
    aHciPacket[2U] = mHciLeEnhancedReceiverTestCommandLength_c;
    aHciPacket[3U] = rx_channel;
    if (phy == 4U)
    {
       aHciPacket[4U] = 3U;
    }
    else
    {
      aHciPacket[4U] = phy;
    }
    aHciPacket[5U] = modulation_idx;
    result = Hcit_SendPacket(gHciCommandPacket_c, aHciPacket, gHciCommandPacketHeaderLength_c + mHciLeEnhancedReceiverTestCommandLength_c);

    return result;
}

/* send DTM 2-wires status event */
static void DTM_send_test_status_event(uint16_t response, uint8_t status)
{
    uint8_t* pPacketBuffer;

    pPacketBuffer = (uint8_t *)MEM_BufferAlloc(2U);
    
    if( NULL != pPacketBuffer )
    {
        pPacketBuffer[0U] = ((response >> 7U) & 0x7FU);
        pPacketBuffer[1U] = status & 0x1U | (((response ) & 0x7FU) << 1U);
        (void)SerialManager_OpenWriteHandle((serial_handle_t)g_dtmHandle, (serial_write_handle_t)gDtmSerialWriteHandle);
        (void)SerialManager_InstallTxCallback(gDtmSerialWriteHandle, (serial_manager_callback_t)DTM_serial_manager_tx_callback, pPacketBuffer);
        if(kStatus_SerialManager_Success != SerialManager_WriteNonBlocking(gDtmSerialWriteHandle, pPacketBuffer, 2U))
        {
            (void)MEM_BufferFree(pPacketBuffer);
        }
    }
}

/* send DTM 2-wires test report */
static void DTM_send_test_report_event(uint8_t cmd, uint16_t packet_count)
{
    uint8_t* pPacketBuffer;

    pPacketBuffer = (uint8_t *)MEM_BufferAlloc(2);
    
    if( NULL != pPacketBuffer )
    {
        pPacketBuffer[0U] = 0x80U | ((packet_count >> 8U) & 0x7FU);
        pPacketBuffer[1U] = ((packet_count ) & 0xFFU);
        (void)SerialManager_OpenWriteHandle((serial_handle_t)g_dtmHandle, (serial_write_handle_t)gDtmSerialWriteHandle);
        (void)SerialManager_InstallTxCallback(gDtmSerialWriteHandle, (serial_manager_callback_t)DTM_serial_manager_tx_callback, pPacketBuffer);
        if(kStatus_SerialManager_Success != SerialManager_WriteNonBlocking(gDtmSerialWriteHandle, pPacketBuffer, 2U))
        {
            (void)MEM_BufferFree(pPacketBuffer);
        }
    }
}

static bleResult_t DTM_send_test_end_hci_cmd(void)
{
    uint8_t aHciPacket[mHciLeTestEndCommandLength_c + gHciCommandPacketHeaderLength_c];
    uint16_t opcode = HciCommand(gHciLeControllerCommands_c, mHciLeTestEndCommand_c);
    bleResult_t result;

    /* Set HCI opcode */
    FLib_MemCpy((void*)aHciPacket, (const void*)&opcode, 2U);
    aHciPacket[2] = mHciLeTestEndCommandLength_c;
    result = Hcit_SendPacket(gHciCommandPacket_c, aHciPacket, gHciCommandPacketHeaderLength_c + mHciLeTestEndCommandLength_c);

    return result;
}

static bleResult_t DTM_send_read_maximum_data_len_hci_cmd(void)
{
    uint8_t aHciPacket[mHciLeReadMaxDataLengthCommandLength_c + gHciCommandPacketHeaderLength_c];
    uint16_t opcode = HciCommand(gHciLeControllerCommands_c, mHciLeReadMaxDataLengthCommand_c);
    bleResult_t result;

    /* Set HCI opcode */
    FLib_MemCpy((void*)aHciPacket, (const void*)&opcode, 2U);
    aHciPacket[2] = mHciLeReadMaxDataLengthCommandLength_c;
    result = Hcit_SendPacket(gHciCommandPacket_c, aHciPacket, gHciCommandPacketHeaderLength_c + mHciLeReadMaxDataLengthCommandLength_c);

    return result;
}

static bleResult_t DTM_send_hci_reset_hci_cmd(void)
{
    uint8_t aHciPacket[mHciHciResetCommandLength_c + gHciCommandPacketHeaderLength_c];
    uint16_t opcode = HciCommand(gHciControllerBasebandCommands_c, mHciResetCommand_c);
    bleResult_t result;

    /* Set HCI opcode */
    FLib_MemCpy((void*)aHciPacket, (const void*)&opcode, 2U);
    aHciPacket[2] = mHciHciResetCommandLength_c;
    result = Hcit_SendPacket(gHciCommandPacket_c, aHciPacket, gHciCommandPacketHeaderLength_c + mHciHciResetCommandLength_c);
    return result;
}

static void DTM_serial_manager_tx_callback(void *callbackParam, serial_manager_callback_message_t *message, serial_manager_status_t status)
{
    (void)MEM_BufferFree(callbackParam);
}

/* UART RX callback for DTM 2-wires */
static void DTM_serial_manager_rx_callback(void *callbackParam,
                   serial_manager_callback_message_t *message,
                   serial_manager_status_t status)
{
    uint8_t  recvChar;
    uint32_t count = 0U;
    uint8_t msb_byte = 0U;
    uint8_t lsb_byte = 0U;
    static uint8_t is_msb_byte = 1U;
    static uint8_t cmd = 0U;
    static uint8_t test_freq = 0U;
    static uint8_t test_data_len = 0U;
    static uint8_t test_pkt = 0U;
    static uint8_t test_phy = 1U;
    static uint8_t test_mod_idx = 0U;
    static uint8_t control = 0U;
    static uint8_t parameter = 0U;
    SerialManager_TryRead( (serial_read_handle_t)gDtmSerialReadHandle, &recvChar, 1U, &count);

    if (count)
    {
        if (is_msb_byte)
        {
           msb_byte = recvChar;
           cmd = msb_byte >> 6U;
           if ((cmd == 1) || (cmd == 2U))
           {
              test_freq = msb_byte & 0x3FU;
           }
           else
           {
              control = msb_byte & 0x3FU;
           }
           is_msb_byte = 0U;
        }
        else
        {
          lsb_byte = recvChar;
          if (cmd != 0U)
          {
              test_data_len |= (lsb_byte >> 2U);
              test_pkt = lsb_byte & 0x3U;
              if (cmd == 1)
              {
                 bleResult_t result = DTM_send_receiver_test_hci_cmd(test_freq, test_phy, test_mod_idx);
                 if (result != gBleSuccess_c)
                 {
                   DTM_send_test_status_event(0U, 1U);
                 }
              }
              else if (cmd == 2)
              {
                 bleResult_t result = DTM_send_transmitter_test_hci_cmd(test_freq, test_data_len, test_pkt, test_phy);
                 if (result != gBleSuccess_c)
                 {
                   DTM_send_test_status_event(0U, 1U);
                 }
              }
              else
              {
                 bleResult_t result = DTM_send_test_end_hci_cmd();
                 if (result != gBleSuccess_c)
                 {
                   DTM_send_test_status_event(0U, 1U);
                 }
              }
          }
          else
          {
              parameter = lsb_byte >> 2;
              switch(control)
              {
                case LE_TEST_CONTROL_RESET:
                {
                  if (parameter == 0U)
                  {
                     test_phy = 1U;
                     test_mod_idx = 0U;
                     test_data_len = 0U;
                     DTM_send_test_status_event(0U,0U);
                  }
                  else
                  {
                     DTM_send_test_status_event(0U,1U);
                  }
                  break;
                }
                case LE_TEST_CONTROL_DATA_LEN:
                {
                  if (parameter < 4U)
                  {
                      test_data_len = parameter << 6;
                      DTM_send_test_status_event(0U,0U);
                  }
                  else
                  {
                      DTM_send_test_status_event(0U,1U);
                  }
                  break;
                }
                case LE_TEST_CONTROL_PHY:
                {
                  if ((parameter > 0U) && (parameter <5U))
                  {
                      test_phy = parameter;
                      DTM_send_test_status_event(0U,0U);
                  }
                  else
                  {
                      DTM_send_test_status_event(0U,1U);
                  }
                  break;
                }
                case LE_TEST_CONTROL_MOD_IDX:
                {
                  if (parameter < 2U)
                  {
                      test_mod_idx = parameter;
                      DTM_send_test_status_event(0U,0U);
                  }
                  else
                  {
                      DTM_send_test_status_event(0U,1U);
                  }
                  break;
                }
                case LE_TEST_CONTROL_READ_SUPP_FEAT:
                {
                  if (parameter == 0U)
                  {
                    /* generate event */
                    /* bit 1: Data Packet Length Extension supported, 
                       bit 2: 2M PHY supported,
                       bit 3: Transmitter Stable mouldation index supported 
                       bit 4: Coded PHY supported*/
                    uint16_t resp = 0xF; 
                    DTM_send_test_status_event(resp, 0U);
                  }
                  else
                  {
                     DTM_send_test_status_event(0U, 0U);
                  }
                  break;
                }
                case LE_TEST_CONTROL_READ_MAX_OCTETS:
                {
                  bleResult_t result = DTM_send_read_maximum_data_len_hci_cmd();
                  if (result != gBleSuccess_c)
                  {
                    DTM_send_test_status_event(0U, 1U);
                  }
                  read_max_oct_type = parameter;
                  break;
                }
                default:
                {
                    DTM_send_test_status_event(0U, 1U);
                }
              }
          }
          is_msb_byte = 1U;
        }
    }
}

/* HCI downlink Callback for DTM 2-wires */
static bleResult_t DTM_hci_recv(hciPacketType_t packetType, void* pHciPacket,uint16_t packetSize)
{
    uint8_t *pSerialPacket = (uint8_t *) pHciPacket;
    /* pSerialPacket[0] == 0x04, pSerialPacket[1] == 0x0E */
    uint8_t status = 0U;
    /* command status */
    if (pSerialPacket[5U]!= 0U)
    {
        status = 1U;
    }
    /* check command group */
    if (pSerialPacket[4U] == 0x20U)
    {
        /* command group */ 
        switch(pSerialPacket[3U])
        {
            case mHciLeTestEndCommand_c:
            {
                uint16_t no_of_packet_received = ((pSerialPacket[7U] << 8U )| pSerialPacket[6]);
                DTM_send_test_report_event(3,no_of_packet_received);
                break;
            }
            case gHciLeEnhancedReceiverTestCommand_c:
            case gHciLeEnhancedTransmitterTestCommand_c:
            {
                DTM_send_test_status_event(0U,status);
                break;
            }
            case mHciLeReadMaxDataLengthCommand_c:
            {
                if (pSerialPacket[5U] == 0U)
                {
                    uint16_t nb_octets = 0U;
                    /* Max <x> octets */
                    nb_octets = (pSerialPacket[6U + (read_max_oct_type * 2U)] << 8U ) + pSerialPacket[7 + (read_max_oct_type * 2U)];
                    DTM_send_test_status_event(nb_octets,0U);
                }
                else
                {
                    DTM_send_test_status_event(0U,1U);
                }
                break;
            }
        }
    }
    return gBleSuccess_c;
}
