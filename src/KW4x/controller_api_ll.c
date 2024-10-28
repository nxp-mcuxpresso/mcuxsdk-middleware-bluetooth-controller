/*
 * Copyright 2020-2024 NXP
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*******************************************************************************
 * Incude
 ******************************************************************************/

#include "fsl_common.h"
#include "ll_types.h"
#include "controller_init.h"
#include "controller_api_ll.h"

/*******************************************************************************
 * Types & defines
 ******************************************************************************/
#define _PUT32(p, i)                \
    do {                            \
      (p)[0] = (uint8)(i);          \
      (p)[1] = (uint8)((i) >> 8U);  \
      (p)[2] = (uint8)((i) >> 16U); \
      (p)[3] = (uint8)((i) >> 24U); \
    } while(FALSE)

#define _PUT24(p, i)                \
    do {                            \
      (p)[0] = (uint8)(i);          \
      (p)[1] = (uint8)((i) >> 8U);  \
      (p)[2] = (uint8)((i) >> 16U); \
    } while(0U!=0U)

#define _PUT16(p, i)                \
    do {                            \
      (p)[0] = (uint8)(i);          \
      (p)[1] = (uint8)((i) >> 8U);  \
    } while(FALSE)

#define _PUT8(p, i)             \
    do {                            \
      (p)[0] = (uint8)(i);          \
    } while(FALSE)


#define _GET8(p)  (*((uint8_t*)(p)))
#define _GET16(p) (((uint16_t)_GET8 ((p))) | ((uint16_t)_GET8 ((uint8_t*)(p)+1U) << 8U))
#define _GET32(p) (((uint32_t)_GET16((p))) | ((uint32_t)_GET16((uint8_t*)(p)+2U) << 16U))
      
typedef enum 
{
    API_Controller_SetTxPowerLevel,
    API_Controller_SetTxPowerLevelDbm,
    API_Controller_SetMaxTxPower,
    API_Controller_SetRandomSeed,
    API_Controller_ConfigureAdvCodingScheme,
    API_Controller_ConfigureConnCodingScheme,
    API_Controller_ConfigureInvalidPduHandling,
    API_Controller_SetConnNotificationMode,
    API_Controller_SetChannelSelectionAlgo2,
    API_Controller_GetTimestamp,
    API_Controller_GetTimestampEx,
    API_Controller_Reserve_1, /* place holding for new entry */
    API_Controller_GetEncryptionParam,
    API_Controller_Reserve_2, /* place holding for new entry */
    API_Controller_SetRxMode,
    API_Controller_ConfigureSCA,
    API_Controller_ConfigureIDSSecurityEvent,
    API_Controller_ReadMemory,
    API_Last
} PLATFORM_NbuApiId;

/*******************************************************************************
 * Variables
 ******************************************************************************/

// this table must be aligned with the one defined on the app core side
static const uint8_t api_param_lenth[] =
{
    2U,   /* Controller_SetTxPowerLevel */
    2U,   /* Controller_SetTxPowerLevelDbm */
    2U,   /* Controller_SetMaxTxPower */
    4U,   /* Controller_SetRandomSeed */
    2U,   /* Controller_ConfigureAdvCodingScheme */
    1U,   /* Controller_ConfigureConnCodingScheme */
    1U,   /* Controller_ConfigureInvalidPduHandling */
    4U,   /* Controller_ConfigureConnNotificationMode */
    1U,   /* Controller_SetChannelSelectionAlgo2 */
    1U,   /* Controller_GetTimestamp */
    0U,   /* Controller_GetTimestampEx */
    0U,   /* place holding for new entry */
    3U,   /* Controller_GetEncryptionParam */
    0U,   /* place holding for new entry */
    2U,   /* Controller_SetRxMode */
    1U,   /* Controller_ConfigureSCA */
    1U,   /* Controller_ConfigureIDSSecurityEvent */
    8U,   /* Controller_ReadMemory */ 
};

enum { assert_api_param_lenth = 1/(API_Last==sizeof(api_param_lenth)?1:0) };

/*******************************************************************************
 * Functions
 ******************************************************************************/

/* return the number of bytes to send to the app core. 0 if error */
uint32_t Controller_HandleNbuApiReq(uint8_t *api_return, uint8_t *data, uint32_t data_len)
{
    PLATFORM_NbuApiId req_id;
    uint32 api_status = gBleSuccess_c;
    uint32 nb_returns = 4U; /* by default 4 containing API status (uint32_t) */

    req_id = (PLATFORM_NbuApiId)((uint16_t)data[0U]+((uint16_t)data[1U]<<8U));
    data_len -= 2U;
    
    if ( req_id >= API_Last || api_param_lenth[req_id] != data_len )
    {
        /* invalid parameters length */
        nb_returns = 0U;
    }
    else
    {
        switch(req_id)
        {
            case API_Controller_SetTxPowerLevel:
                api_status = LL_API_SetTxPowerLevel((uint8_t)data[2], (uint8_t)data[3]);
                break;
            
            case API_Controller_SetTxPowerLevelDbm:
                api_status = LL_API_SetTxPowerLevelDbm((uint8_t)data[2], (uint8_t)data[3]);
                break;
                
            case API_Controller_SetMaxTxPower:
                api_status = Controller_SetMaxTxPower((int8_t)data[2], (uint8_t)data[3]);
                break;
                
            case API_Controller_SetRandomSeed:
            {
                uint32 seed = ((uint32)data[5] << 24U) | ((uint32)data[4] << 16U) | ((uint32)data[3] << 8U) | (uint32)data[2];
                api_status = LL_API_SetRandomSeed(seed);
                break;
            }
            
            case API_Controller_ConfigureAdvCodingScheme:
            {
                api_status = LL_API_ConfigureCodingScheme(data[2], data[3], TRUE);
                break;
            }
            case API_Controller_ConfigureConnCodingScheme:
            {
                api_status = LL_API_ConfigureCodingScheme(data[2], 0, FALSE);
                break;
            }
            case API_Controller_ConfigureInvalidPduHandling:
            {
                api_status = LL_API_ConfigureInvalidPduHandlingType(data[2]);
                break;
            }
            case API_Controller_SetConnNotificationMode:
            {
                api_status = LL_API_SetConnRxPduNotificationMode(data[2]);
                break;
            }
            case API_Controller_SetChannelSelectionAlgo2:
            {
                api_status = LL_API_SetChannelSelectionAlgo2(data[2]);
                break;
            }
            case API_Controller_GetTimestamp:
            {
                uint32_t clock;
                uint16_t qus;  
                
                // workaround for native clock value after wakeup
                LL_API_WaitForClkUpdtFromLowPwr();
                
                LL_API_GetBleTiming(&clock, &qus);
                api_status = (uint32_t)(((uint64_t)clock*625U*2U + (uint64_t)qus) / 4U);
                break;
            }
            case API_Controller_GetTimestampEx:
            {
                uint32_t hslot;
                uint16_t qus;
                uint64_t tstmr;
                
                // workaround for native clock value after wakeup
                LL_API_WaitForClkUpdtFromLowPwr();

                // use atomic section to have LL timing and TSTMR0 at the same time
                OSA_InterruptDisable();
                LL_API_GetBleTiming(&hslot, &qus);
                tstmr = *(uint64_t *)TSTMR0;
                OSA_InterruptEnable();

                if( (hslot&1U) != 0 )
                {
                  qus += 1250U;
                  hslot -= 1;
                }
                _PUT32(api_return+4U,  hslot >> 1U);
                _PUT32(api_return+8U,  qus >> 2U);
                _PUT32(api_return+12U, (uint32_t)tstmr);
                _PUT32(api_return+16U, (uint32_t)(tstmr>>32U));
                nb_returns += 16U;
                break;
            }
            case API_Controller_GetEncryptionParam:
            {
                api_status = LL_API_GetEncryptionParam(_GET16(&data[2]), data[4],
                                                       &api_return[4],   // sk_skd
                                                       &api_return[20],  // iv
                                                       &api_return[28],  // pl counter tx
                                                       &api_return[33]); // pl counter rx
                nb_returns = 4U+16U+8U+5U+5U;
                break;
            }
            case API_Controller_SetRxMode:
            {
                api_status = LL_API_Controller_SetRxMode(data[2]);
                break;
            }
            case API_Controller_ConfigureSCA:
            {
                api_status = LL_API_SetClockAccuracy(data[2]);
                break;
            }
            case API_Controller_ConfigureIDSSecurityEvent:
            {
                uint32 securityEvents = ((uint32)data[5] << 24U) | ((uint32)data[4] << 16U) | ((uint32)data[3] << 8U) | (uint32)data[2];
                api_status = LL_API_ConfigureIDSSecurityEvent(securityEvents);
                break;
            }
            case API_Controller_ReadMemory:
            {
                uint32_t ptr = _GET32(&data[2]);
                uint32_t size = _GET32(&data[6]);
                
                // exclude code reading
                const uint32_t code_start = 0x00000000U;
                const uint32_t code_end   = 0x0003FFFFU;
                if( size <= 32U && (ptr < code_start || ptr > code_end ) )
                {
                    if( (ptr&3U)==0U && (size&3U)==0U )
                    {
                        // 4-byte aligned read
                        for (uint32_t index=0U; index<size; index+=4U)
                        {
                            *((uint32_t*)(api_return+4U+index)) = *((uint32_t*)(ptr+index));
                        }
                    }
                    else
                    {
                        memcpy(api_return+4U, (void*)ptr, size);
                    }
                    nb_returns += size;
                    api_status = gBleSuccess_c;
                }
                else
                {
                    api_status = gBleInvalidParameter_c;
                }
                break;
            }
            default:
                /* invalid api id */
                nb_returns = 0U;
                break;
        }
    }
    assert(nb_returns != 0U);
    if(nb_returns != 0U)
    {
        // update API status
        _PUT32(&api_return[0], api_status);
    }
    return nb_returns;
}

bool Controller_EnableSecurityFeature()
{
  uint32 status = LL_API_EnableSecurityFeature();
  return ((status == 0U) ? true: false);
}
