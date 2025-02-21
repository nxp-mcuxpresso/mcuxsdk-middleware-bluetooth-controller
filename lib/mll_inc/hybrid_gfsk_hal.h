/*! *********************************************************************************
*
* Copyright 2024 NXP
*
* NXP Proprietary. 
* This software is owned or controlled by NXP and may only be used strictly in accordance 
* with the applicable license terms.  By expressly accepting such terms or by downloading, 
* installing, activating and/or otherwise using the software, you are agreeing that you 
* have read, and that you agree to comply with and are bound by, such license terms.  
* If you do not agree to be bound by the applicable license terms, then you may not retain, 
* install, activate or otherwise use the software.
********************************************************************************** */

#ifndef __HYBRID_GFSK_HAL_H__
#define __HYBRID_GFSK_HAL_H__

/* RX packet status bitmap */
#define HYBRID_GFSK_STATUS_CRC_ERROR         0x01U
#define HYBRID_GFSK_STATUS_LENGTH_ERROR      0x02U

typedef struct rx_packet_info_tag
{
    uint8  status;
    uint8  rssi;
    uint8  length;
    uint8  *pData;
} rx_packet_info_t;

typedef void (*hybrid_gfsk_callback)(rx_packet_info_t *cb);

/* return 0 if GFSK is not enabled in the HAL, 1 otherwise */
uint32 hybrid_gfsk_hal_enabled(void);

/* initialize GFSK module */
void   hybrid_gfsk_enable(hybrid_gfsk_callback callback);

/* disable GFSK including IRQ so core0 can use the GFSK */
void   hybrid_gfsk_disable(void);

/* start TX */
uint32 hybrid_gfsk_enter_tx(uint8 *pData, uint8 size);

/* start RX */
uint32 hybrid_gfsk_enter_rx(void);

/* abort ongoing GFSK TX/RX, set LL mode to BTLE */
void   hybrid_gfsk_cancel(void);

/* get the tx packet duration from preamble to CRC. packet_length does not includ preamble nor CRC */
uint32 hybrid_gfsk_get_tx_pkt_dur_us(uint32 packet_length);

/* get the rx packet minimum duration from preamble to CRC */
uint32 hybrid_gfsk_get_rx_pkt_min_dur_us(void);

#endif // __HYBRID_GFSK_HAL_H__