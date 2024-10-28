/*! *********************************************************************************
 * \defgroup CONTROLLER - Controller Interface
 * @{
 ********************************************************************************** */
/*!
* Copyright (c) 2014, Freescale Semiconductor, Inc.
* Copyright 2016-2024 NXP
*
* SPDX-License-Identifier: BSD-3-Clause
*/

#ifndef _CONTROLLER_INTERFACE_H_
#define _CONTROLLER_INTERFACE_H_

/************************************************************************************
*************************************************************************************
* Include
*************************************************************************************
************************************************************************************/
#include "EmbeddedTypes.h"
#include "fsl_os_abstraction.h"
#include "ll_types.h"
#include "ble_general.h"
#include "controller_api_ll.h"

/************************************************************************************
*************************************************************************************
* Public macros
*************************************************************************************
************************************************************************************/

#define Controller_SetAdvertisingTxPowerLevel(level) \
    Controller_SetTxPowerLevel(level,gAdvTxChannel_c)

#define Controller_SetConnectionTxPowerLevel(level) \
    Controller_SetTxPowerLevel(level,gConnTxChannel_c)

/* WARNING: Do not change these defines */
#define gLlAdvSetCtxtSize                64U
#define gLlConnectionCtxtSizeBle5       256U    //for lib_ble_kw38a4_controller.a
#define gLlConnectionCtxtSizeBle4_2     200U    //for lib_ble_kw38a4_controller_ble_4_2.a
#define gLlQueueElmtSize                 12U
#define gLlQueueSizeWithoutAclQueue       4U
#define gLlHciTdQueueElmtSize             8U
#define gLlDupFilListInfoSizeBle5        20U
#define gLlDupFilListInfoSizeBle4_2      12U
#define gLlCmdBufferSizeBle5            256U
#define gLlCmdBufferSizeBle4_2           64U
#define gLlGenEventBufferSizeExtAdv     255U
#define gLlGenEventBufferSizeNoExtAdv    68U
#define gLlWlCtxtSize                     8U
#define gLlPeriodicWlCtxtSize             9U
#define gLlSyncInfoCtxtSize              36U
/************************************************************************************
*************************************************************************************
* Structures/Data Types
*************************************************************************************
************************************************************************************/

typedef enum txChannelType_tag {
    gAdvTxChannel_c,
    gConnTxChannel_c
} txChannelType_t;

typedef enum dtmBaudrate_tag {
    gDTM_BaudRate_1200_c = 0,
    gDTM_BaudRate_2400_c,
    gDTM_BaudRate_9600_c,
    gDTM_BaudRate_14400_c,
    gDTM_BaudRate_19200_c,
    gDTM_BaudRate_38400_c,
    gDTM_BaudRate_57600_c,
    gDTM_BaudRate_115200_c
} dtmBaudrate_t;
typedef struct bleCtrlNotificationEvent_tag
{
    uint16_t event_type; /*! bleNotificationEventType_t */
    uint16_t conn_handle;
    uint8_t  rssi;
    uint8_t  channel_index;
    uint16_t conn_ev_counter;
    uint16_t timestamp;
    uint8_t  adv_handle;
} bleCtrlNotificationEvent_t;

typedef void (*bleCtrlNotificationCallback_t)
(
    bleCtrlNotificationEvent_t *pNotificationEvent
);

/* Structure used to pass configuration parameter to LL. A runtime check will be done
 to make sure parameter are within the allowed range of the linked LL library. */
typedef struct
{
    /* Extended advertising related info */
    uint16_t    ext_adv_max_used_adv_data_length;
    uint8_t     ext_adv_max_used_set;
    uint8_t     adv_set_ctxt_size_used;

    uint8_t     max_sync_engine_used;
    uint8_t     sync_info_ctxt_size_used;
    uint8_t     periodic_wl_size_used;
    uint8_t     wl_size_used;
    uint8_t     wl_ctxt_size_used;
    uint8_t     periodic_wl_ctxt_size_used;
    /* Connection related info */
    uint16_t    connection_ctxt_size_used;
    uint8_t     max_connections_used;
    /* duplicated filtering info */
    uint8_t     gBleMaxDupFilInfo;
    uint16_t    dup_fil_ctxt_size_used;
    /* command buffer related info */
    uint16_t    max_used_cmd_param_length_bytes;
    uint8_t     max_used_generic_event_length_bytes;
    /* TX acl packet info */
    uint8_t     max_configured_ll_tx_acl_pkts;
    /* RX acl packet info */
    uint8_t     max_configured_ll_rx_acl_pkts;
    uint8_t     max_configured_num_event_buffers;
    /* feature related info */
    uint8_t     gBle5FeatureMask1_c;
    uint8_t     gBle5FeatureEnable1_c;
    /* other parameter */
    uint8_t     ll_mem_pool_id;
}
bleCtrlConfigParam;

/************************************************************************************
*************************************************************************************
* Public memory declarations
*************************************************************************************
************************************************************************************/
extern OSA_EVENT_HANDLE_DEFINE(mControllerTaskEvent);
extern bool_t gMCUSleepDuringBleEvents;
extern bool_t gEnableSingleAdvertisement;
#ifndef NXP_FIX
extern bool_t gActivateHybridMode;
extern void (*pfBLE_SignalFromISR)(void);
#endif /* NXP_FIX */
/************************************************************************************
*************************************************************************************
* Public prototypes
*************************************************************************************
************************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

/*! *********************************************************************************
 * \brief  Controller_TaskHandler
 * \param[in]
 *
 ********************************************************************************** */
void * Controller_TaskHandler(osa_task_param_t arg);

/*! *********************************************************************************
 * \brief  Controller_InterruptHandler
 * \param[in]
 *
 ********************************************************************************** */
void Controller_InterruptHandler(void);

/*! *********************************************************************************
 * \brief  Sets the BD_ADDR.
 * \param[in]  packetType HCI packet Type
 *
 * \return gBleSuccess_c, gBleInvalidState_c or gBleInvalidParameter_c
 ********************************************************************************** */
bleResult_t Controller_SetDeviceAddress(uint8_t* bdAddr);

/*! *********************************************************************************
 * \brief  Controller Get Next instance
 * \param[in]  pNextInstant Pointer to the calculated_instant CE instant
 *   for applying new channel map or for updating connection parameters.
 * \return uint16_t
 ********************************************************************************** */
uint16_t Controller_GetNextInstance(uint16_t* pNextInstant);

/*! *********************************************************************************
 * \brief  Controller Get Inactivity Duration
 *
 * \return uint32_t
 ********************************************************************************** */
uint32_t Controller_GetInactivityDuration(void);

/*! *********************************************************************************
 * \brief  Sets the TX Power on the advertising or connection channel.
 * \param[in]  level    Power level (range 0-X) as defined in the table bellow.
 *                      (X=15 for MKW40 and X=31 for MKW41 and MKW38)
 * \param[in]  channel  Advertising or connection channel.
 *
 * \return gBleSuccess_c or error.
 *
 * \remarks This function executes synchronously.
 *
 * \remarks For MKW40Z BLE controller there are 16 possible power levels 0 <= N <= 15
 * for which the output power is distributed evenly between minimum and maximum power levels.
 * For further details see the silicon datasheet.
 *
 * \remarks For MKW41Z BLE controller there are 32 possible power levels 0 <= N <= 31
 * for which the output power is distributed evenly between minimum and maximum power levels.
 * For further details see the silicon datasheet.
 *
 * \remarks For MKW38 BLE controller there are 32 possible power levels 0 <= N <= 31
 * for which the output power is distributed evenly between minimum and maximum power levels.
 * For further details see the silicon datasheet. For this latter platform, the preferred
 * interface to set the TX power is Controller_SetTxPowerLevelDbm (see below) which does not
 * require any knowledge of the power tables.
 ********************************************************************************** */
bleResult_t Controller_SetTxPowerLevel(uint8_t level, txChannelType_t channel);

/*! *********************************************************************************
 * \brief  Sets the TX Power on the advertising or connection channel.
  * \param[in]  level_dbm    Power level in dBm.
 * \param[in]  channel      Advertising or connection channel.
 *
 * \return gBleSuccess_c or error.
 *
 * \remarks This function executes synchronously.
 ********************************************************************************** */
bleResult_t Controller_SetTxPowerLevelDbm(int8_t level_dbm, txChannelType_t channel);

/*! *********************************************************************************
 * \brief  Sets the perodic whitelist size
 * \param[in]  size    number of whitelist entries
 *
 * \return gBleSuccess_c or error.
 *
 * \remarks This function executes synchronously.
 ********************************************************************************** */
bleResult_t Controller_SetPeriodicWhiteListSize(uint8_t size);

/*! *********************************************************************************
 * \brief  Configure enhanced notifications on advertising, scannning and connection events
 *         on the controller.
 * \param[in]  eventType       Event type selection as specified by bleNotificationEventType_t.
 * \param[in]  conn_handle     Connection handle of the peer, used only for connection events.
 *
 * \return gBleSuccess_c or error.
 ********************************************************************************** */
bleResult_t Controller_ConfigureEnhancedNotification
(
    uint16_t eventType,
    uint16_t conn_handle
);

/*! *********************************************************************************
 * \brief  Controller Register Enhanced Notification Event Callback
 * \param[in]  notificationCallback Callback to be executed in adv/scan/conn ISR context
 *
 * \return gBleSuccess_c or error.
 ********************************************************************************** */
bleResult_t Controller_RegisterEnhancedEventCallback
(
    bleCtrlNotificationCallback_t notificationCallback
);

/*! *********************************************************************************
* \brief   Reset, Save and restore Hardware Link Layer Initialize
*
********************************************************************************** */
void Controller_ResetLlhState(void);
void Controller_RestoreLlhState(void);
void Controller_RestoreLlhStateComplete(void);
void Controller_SaveLlhState(void);

/*! *********************************************************************************
* \brief   Reprogram the next ADV event time - shall be called during restore process
*            before calling Controller_RestoreLlhStateComplete();
*
********************************************************************************** */
void Controller_UpdateLlhAdvInstant(uint16_t next_instant);

/*! *********************************************************************************
* \brief   Check if power gating lowpower mode is allowed by Link layer
*
********************************************************************************** */
bool_t Controller_IsPowerGatingAllowed(void);

/*! *********************************************************************************
* \brief   Initialize the XCVR module
*
* \return uint32_t , 0 if successful, error if other values.
*
********************************************************************************** */
uint32_t Controller_RadioInit(void);

/*! *********************************************************************************
* \brief   Sets nbu version
*
* \return uint32_t , 0 if successful, error if other values.
*
********************************************************************************** */
uint32_t Controller_SetNbuVersion(const uint8* repo_digest);

#ifdef __cplusplus
}
#endif

/************************************************************************************
 ************************************************************************************
 *                  Common header section, for all platforms                        *
 ************************************************************************************
 ***********************************************************************************/

typedef bleResult_t (*gHostHciRecvCallback_t)
(
    hciPacketType_t packetType,
    void* pHciPacket,
    uint16_t hciPacketLength
);


/*! *********************************************************************************
 * \brief  Performs initialization of the Controller.
 * \param[in]  callback HCI Host Receive Callback
 *
 * \return osaStatus_Success or osaStatus_Error
 ********************************************************************************** */
osa_status_t Controller_Init(const nbuIntf_t* nbuInterface);

/*! *********************************************************************************
 * \brief  Controller Receive Interface
 * \param[in]  packetType HCI packet Type
 * \param[in]  pPacket    data buffer
 * \param[in]  packetSize data buffer length
 *
 * \return gBleSuccess_c, gBleOutOfMemory_c or gBleInvalidParameter_c
 ********************************************************************************** */
bleResult_t Hci_SendPacketToController( hciPacketType_t packetType, void* pPacket,
                                uint16_t packetSize);

/*! *********************************************************************************
 * \brief  Controller Set DTM Baudrate
 *
 * \return void
 ********************************************************************************** */
bleResult_t Controller_SetDTMBaudrate(dtmBaudrate_t baudrate);

/*! *********************************************************************************
 * \brief  Set LDO ANT TRIM value in XCVR_ANALOG register
 * \param[in]  power_dBm    maximum target transmit power in dBm
 * \param[in]  ldo_ant_trim LDO_ANT_TRIM value to apply
 *
 * \return gBleSuccess_c, gBleInvalidParameter_c
 ********************************************************************************** */
bleResult_t Controller_SetMaxTxPower(int8_t power_dBm, uint8_t ldo_ant_trim);

#endif /* _CONTROLLER_INTERFACE_H_ */

/*! *********************************************************************************
* @}
********************************************************************************** */

