/*                Copyright 2021-2024, NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
 
/* WARNING: DO NOT MODIFY THIS FILE directly as this file is opied from LL. */

#ifndef CONTROLLER_API_H_
#define CONTROLLER_API_H_

/*******************************************************************************
 * Definitions
 ******************************************************************************/

typedef void (*nbuHciIntf_t)(unsigned long packetType, void *pPacket, unsigned short packetSize);
typedef void (*nbuChannelSwitchIntf_t)(unsigned short channel);
typedef void (*nbuDbgIoSet_t)(unsigned long pin, unsigned short level);
typedef void (*nbuPhySwitchIntf_t)(unsigned char rate);
#ifndef EP_MEAS_CRITICAL_SECTION
typedef void (*nbuEnterCritical_t)(void);
#else /*EP_MEAS_CRITICAL_SECTION*/
typedef void (*nbuEnterCritical_t)(unsigned int source);
#define CR_SECT_LL_ID_MAX    73U
#endif /*EP_MEAS_CRITICAL_SECTION*/
typedef void (*nbuExitCritical_t)(void);

typedef struct nbuIntf_tag
{
    nbuHciIntf_t nbuHciIntf;
    nbuChannelSwitchIntf_t nbuChannelSwitchIntf;
    nbuDbgIoSet_t nbuDbgIoSet;
    nbuPhySwitchIntf_t nbuPhySwitchIntf;
    nbuEnterCritical_t nbuEnterCritical;
    nbuExitCritical_t nbuExitCritical;
} nbuIntf_t;

/* enum below should be aligned with ble_general.h on app side */
#ifndef BLE_GENERAL_H
#define  gAdvTxChannel_c            0U
#define  gConnTxChannel_c           1U
#if defined(BT54_TEST_TX_POWER)
#define  gTestTxChannel_c           3U
#endif
#define  gBleSuccess_c              0U
#define  gBleInvalidParameter_c     1U
#define  gBleFeatureNotSupported_c  4U
#define  gBleConnNotifAll_c         0U
#define  gBleConnNotifDataOnly_c    1U
#endif

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/* update tx power level 0-31 by channel */
uint32 LL_API_SetTxPowerLevel(uint8 level, uint8 channel);

/* update tx power level in dBm by channel. Capted to max if too big */
uint32 LL_API_SetTxPowerLevelDbm(int8 level_dbm, uint8 channel);

/* update tx power level in dBm immediately in HW */
void LL_API_SetTxPowerLevelDbmImmediate(int8 level_dbm);

uint32 LL_API_SetRandomSeed(uint32 seed);

/* convert dBm to PA N slice */
uint32 LL_API_ConvertDbmToPowerNSlice(int8 cInDbm);

/* convert PA N slice to dBm */
int8 LL_API_ConvertPowerNSliceToDbm(uint8 n_slice);

/* update tx power in N slice 0 to 63 */
uint32 LL_API_SetTxPowerNSlice(uint8 pa_n_slice);

/* update the maximum target tx power 0, 7 or 10dBm */
uint32 LL_API_SetMaxTxPower(int8 power_dBm);

/* get number of ongoing Ble connections */
uint32 LL_API_GetNumBleConn(void);

#ifdef BT50_ADV_EXTENSIONS
/* get number of ongoing Periodic Advertising */
uint32 LL_API_GetNumPerAdvEn(void);
#endif

/* set coding scheme for adv or connection */
uint32 LL_API_ConfigureCodingScheme(uint8 codingSch, uint8 handle, boolean forAdv);

/*selects how the PDU is handled when invalid parameter is provided*/
/* 0: ignore PDU, 1: disconnect link */
uint32 LL_API_ConfigureInvalidPduHandlingType(uint8 pdu_handling_type);

/*Enables security feature in LL. Implemented for temporary use (debugging, maturing of feature).
Shall be removed for final release.*/
uint32 LL_API_EnableSecurityFeature(void);

/*modify enhanced notification for RxPdu.
mode 0: all non-empty PDUs are notified
mode 1: only Data Pdu are notified
*/
uint32 LL_API_SetConnRxPduNotificationMode(uint32 mode);

/*modify new Rx Pdu handling time.
mode 0: no delay in Rx
mode 1: new rx pdu processed after TX
*/
uint32 LL_API_Controller_SetRxMode(uint32 mode);

/*enable/disable channel selection algo #2*/
/*implicitly enable/disable extended advertising*/
uint32 LL_API_SetChannelSelectionAlgo2(boolean enable);

/*!
 * \brief Retrieve the PDU encryption related parameters.
 *
 * \param[in] conn_handle connection handle
 * \param[in] sk_or_skd_req indicate if the session key (0) or session key diversifier (1) is required
 * \param[out] sk_or_skd session key or session key diversifier
 * \param[out] iv initialization vector
 * \param[out] pl_counter_tx payload counter for transmission
 * \param[out] pl_counter_rx payload counter for reception
 * \return uint32 API status 0 success, others failure
 */
uint32 LL_API_GetEncryptionParam(
     uint16 conn_handle,
     uint8 sk_or_skd_req,
     uint8 sk_or_skd[16],
     uint8 iv[16],
     uint8 pl_counter_tx[5],
     uint8 pl_counter_rx[5]);

/*!
 * \brief Sets the SCA (Sleep Clock Accuracy) of the device.
 *
 * \param[in] SCA value (shall be in the range 0 to 7)
 * SCA field encoding
 * SCA  |       Accuracy in ppm
 *   0  |       251 ppm to 500 ppm
 *   1  |       151 ppm to 250 ppm
 *   2  |       101 ppm to 150 ppm
 *   3  |        76 ppm to 100 ppm
 *   4  |        51 ppm to  75 ppm
 *   5  |        31 ppm to  50 ppm
 *   6  |        21 ppm to  30 ppm
 *   7  |         0 ppm to  20 ppm
 * \return uint32 API status 0 success, others failure
 */
uint32 LL_API_SetClockAccuracy(uint8 ucSCA);

/*!
 * \brief Sets the security events to be monitored in LL.
 *
 * \param[in] bitmaks containing security events
 * Supported security events:
 * bit 0 set: enable gSecEvt_LlLenOverflow_c event reporting
  * \return uint32 API status 0 for success, others for failure
  *                           1 if provided parameter is not valid (out of range for instance)
  *                           4 if feature is not supported
 */
uint32 LL_API_ConfigureIDSSecurityEvent(uint32 securityEventsBitmask);

/*!
 * \brief Get the anchor point timing of the current connection event. LL timing includes two parts: free running half slot counter in unit of 312.5us
 *        and 1/4 us offset in the half slot.
 *
 * \param[in]  uiConnHandle         connection handle
 * \param[out] pCurrentEvCnt        pointer to save the current connection event counter
 * \param[out] pAnchorHSlot         pointer to save the connection event start time half slot (312.5us)
 * \param[out] pAnchorOffsetQUs     pointer to save the connection event start time half slot offset in quater us
 */
#ifdef SUPPORT_HALF_SLOT_SCHED
void LL_API_GetConnAnchorTiming(uint16 uiConnHandle, uint16 *pCurrentEvCnt, uint32 *pAnchorHSlot, uint16 *pAnchorOffsetQUs);
#endif

extern unsigned char NbuGetCodedIndicator(void);
extern void NbuPwrPeakReductionActivityStart(void);
extern void NbuPwrPeakReductionActivityStop(void);
extern void NbuPwrPeakReductionDisable(void);

extern unsigned long long PLATFORM_GetTimestamp(void);

/* Get BLE native clock in half slot and the native clock offset in quater us */
void LL_API_GetBleTiming(uint32 *pNativeClock, uint16 *pNativeClockOffset);

/* Determine if LL state machine allows sleep mode to gate off clk */
uint8 LL_API_SCHED_IsSleepAllowed(void);

/* Get the delay in halfslot to the next activity */
uint32 LL_API_SCHED_GetSleepTime(void);

/* Get number of pending HCI commands */
uint8 LL_API_BLE_HCI_GetNofPendingCommand(void);

/*! API used to set the LL scheduler activity priority order. 
    The priority identifier can be:
      - 0x0000 to 0x0EFF: 1 connection, other connections, CS, advertising, scanning.
            The specified connection has higher priority versus remaining connections
      - 0xFF00: Priority order connections, CS, advertising, scan
            All connections have same priority
      - 0xFF10: Priority order CS, advertising, connections, scanning
            All connections have same priority
      - 0xFF20: Priority order CS, scanning, connections, advertising
            When the scan is given the highest priorioty, its duration
            should be kept short, or its scan window much smaller than the
            scan interval to avoid connection timeout.
      - 0xFFFF: Priority order CS, connections, advertising, scan (default)

  The API returns HCI_ERR_NO_ERROR (0x00) in case of success, otherwise returns an error code:
  - HCI_ERR_UNKNOWN_CONNECTION_IDENTIFIER (0x02)
  - HCI_ERR_INVALID_PARAMETERS (0x12)
  - HCI_ERR_CMD_DISALLOWED (0x0C)
 */
#define LL_SCHED_PRIO_RESERVED  0x0000U  // reserved for internal usage
#define LL_SCHED_PRIO_CONN      0xFF00U  // (1 connection,) connections, CS, advertising, scan
#define LL_SCHED_PRIO_ADVT      0xFF10U  // CS, advertising, connections, scanning
#define LL_SCHED_PRIO_SCAN      0xFF20U  // CS, scanning, connections, advertising
#define LL_SCHED_PRIO_DEFAULT   0xFFFFU  // CS, connections, advertising, scan (default)
uint8 LL_API_SchedSetPriority(uint16 priority_identifier);

typedef void (*TIMER_APP_CALLBACK)(uint32 ulUserData);
/* API for NBU wakeup or delayed callback
   - the callback is done inside the interrupt service handler so care must be taken to avoid too long processing delay
   - The API CANNOT BE CALLED INSIDE THE CALL BACK. The API will return an error (1) in this case.
   - ulUserData is provided in the call back call
   - the maximum delay is about 23 hours
   - no call back if the parameter callback is NULL
   - API returns 0 if success, != 0 if failure.
*/
uint32 LL_API_AppTimerSet(uint64 ullDelayUs, TIMER_APP_CALLBACK callback, uint32 ulUserData);
/* clear the app timer */
uint32 LL_API_AppTimerClear(void);

extern int PLATFORM_NotifyNbuMemFull(unsigned short poolId, uint16 bufferSize);
extern int PLATFORM_NotifyNbuIssue(void);
extern int PLATFORM_NotifySecurityEvents(uint32 securityEventBitmask);
void NbuHosted_Config(const nbuIntf_t* nbuIf);
void NbuHci_SendPktToController(unsigned long packetType, void *pPacket, unsigned short packetSize);

// workaround for too late native clock update after wakeup
void LL_API_UpdateLastNativeClkBeforeSleep(void);
void LL_API_WaitForClkUpdtFromLowPwr(void);

extern void main_nbu_ll(void);

#endif // CONTROLLER_API_H_
