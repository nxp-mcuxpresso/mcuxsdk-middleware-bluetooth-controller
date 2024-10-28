/*
 * Copyright 2024 NXP
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "fsl_common.h"
#include "EmbeddedTypes.h"
#include "fsl_device_registers.h"
#include "nxp2p4_xcvr.h"
#include "ll_types.h"
#include "hybrid_gfsk_hal.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define GENFSK_IrqPriority_c            (0x40U)

// GENFSK states
#define GENFSK_STATE_OFF                0U      // GFSK not used at all, IRQ disabled, GENFSK can be used by core0
#define GENFSK_STATE_READY              1U      // IRQ handler installed
#define GENFSK_STATE_TX                 2U      // GENFSK doing TX
#define GENFSK_STATE_RX                 3U      // GENFSK doing RX

// GENFSK commands
#define GENFSK_CMD_START_TX_NOW         1U
#define GENFSK_CMD_START_RX_NOW         5U
#define GENFSK_CMD_ABORT_ALL            11U

// IRQ write 1 to clear mask
#define GENFSK_IRQ_CTRL_STATUS_W1C      0xF9FFU
#define GENFSK_IRQ_CTRL2_STATUS_W1C     0x007FU

uint32_t Controller_RadioInit(void);
/*******************************************************************************
 * Private variables
 ******************************************************************************/

// module context
static struct
{
    hybrid_gfsk_callback callback;
    uint8_t              state;
} hybrid_gfsk_ctxt;

static rx_packet_info_t rx_info;

/* BLE channel to channel index */
static const uint8_t hybrid_gfsk_ble_channel_to_index[40] =
{
    37, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
    38, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 
        21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 
        31, 32, 33, 34, 35, 36,
    39
};

/*******************************************************************************
 * Private functions
 ******************************************************************************/

static void hybrid_gfsk_to_btle(void)
{              
    // clear all IRQs
    GENFSK->IRQ_CTRL = GENFSK_IRQ_CTRL_STATUS_W1C;
    GENFSK->IRQ_CTRL2 = GENFSK_IRQ_CTRL2_STATUS_W1C;

    // reset the whitening init
    RBME->WHITEN_CFG = (RBME->WHITEN_CFG & ~RBME_WHITEN_CFG_WHITEN_INIT_MASK) | RBME_WHITEN_CFG_WHITEN_INIT(0U);

    // switch to BTLE mode
    XCVR_SetActiveLL(XCVR_ACTIVE_LL_BTLE);
    hybrid_gfsk_ctxt.state = GENFSK_STATE_READY;
}

static void hybrid_gfsk_whiten_init_set(void)
{
    uint16_t generic_channel = GENFSK->CHANNEL_NUM0;
    uint16_t whiten_init;

    if ( (generic_channel&1U) == 0U && generic_channel >= 42U && generic_channel <= 120U)
    {
        // compatible with BLE channel whitening seed: 0x40 + BLE channel index
        whiten_init = 0x40 + hybrid_gfsk_ble_channel_to_index[(generic_channel-42U)>>1U];
    }
    else
    {  
        // non BLE channe whitening seed: 0x40 + generic channel number
        whiten_init = 0x40 + generic_channel; 
    }

    RBME->WHITEN_CFG = (RBME->WHITEN_CFG & ~RBME_WHITEN_CFG_WHITEN_INIT_MASK) | RBME_WHITEN_CFG_WHITEN_INIT(whiten_init);
}

/*******************************************************************************
 * Public functions
 ******************************************************************************/
// IRQ handler
void GENLL_INT_IRQHandler(void)
{
    uint32_t irq_ctrl;

    /* Read & clear interrupt status. */
    irq_ctrl = GENFSK->IRQ_CTRL;
    GENFSK->IRQ_CTRL = irq_ctrl;
    
    /* TX interrupt */
    if( (irq_ctrl & GENFSK_IRQ_CTRL_TX_IRQ_MASK) != 0U )
    {
        assert(hybrid_gfsk_ctxt.state == GENFSK_STATE_TX);
        hybrid_gfsk_to_btle();
      
        if( hybrid_gfsk_ctxt.callback != NULL )
        {
            // NULL pointer for TX done
            hybrid_gfsk_ctxt.callback(NULL);
        }
    }

    /* RX interrupt */
    if ( (irq_ctrl & GENFSK_IRQ_CTRL_RX_IRQ_MASK) != 0U )
    {
        assert(hybrid_gfsk_ctxt.state == GENFSK_STATE_RX);
        hybrid_gfsk_to_btle();
        
        if( hybrid_gfsk_ctxt.callback != NULL )
        {
            rx_info.pData  = (uint8_t*)RX_PACKET_RAM_BASE;
            rx_info.length = (uint8_t)(((GENFSK->RX_WATERMARK & GENFSK_RX_WATERMARK_BYTE_COUNTER_MASK) >> GENFSK_RX_WATERMARK_BYTE_COUNTER_SHIFT));
            // remove CRC field from the length
            if( (RBME->CRCW_CFG & RBME_CRCW_CFG_CRCW_EN_MASK) != 0U )
            {
                rx_info.length -= (RBME->CRCW_CFG3 & RBME_CRCW_CFG3_CRC_SZ_MASK) >> RBME_CRCW_CFG3_CRC_SZ_SHIFT;
            }
            rx_info.rssi   = (uint8_t)((GENFSK->XCVR_STS & GENFSK_XCVR_STS_RSSI_MASK)>>GENFSK_XCVR_STS_RSSI_SHIFT);
            rx_info.status = (uint8_t)((irq_ctrl&GENFSK_IRQ_CTRL_CRC_VALID_MASK)==0U ? HYBRID_GFSK_STATUS_CRC_ERROR : 0);

            hybrid_gfsk_ctxt.callback(&rx_info);
        }
    }
}

uint32_t hybrid_gfsk_enter_tx(uint8_t *pData, uint8_t size)
{
    if( hybrid_gfsk_ctxt.state != GENFSK_STATE_READY )
    {
        // no irq handler installed
        assert(FALSE);
        return 0U;
    }

    XCVR_SetActiveLL(XCVR_ACTIVE_LL_GENFSK);
    hybrid_gfsk_ctxt.state = GENFSK_STATE_TX;
    
    memcpy((uint8_t*)TX_PACKET_RAM_BASE, pData, size);
    
    hybrid_gfsk_whiten_init_set();

    GENFSK->IRQ_CTRL = GENFSK_IRQ_CTRL_GENERIC_FSK_IRQ_EN_MASK | GENFSK_IRQ_CTRL_TX_IRQ_EN_MASK | GENFSK_IRQ_CTRL_STATUS_W1C;
    GENFSK->XCVR_CTRL = GENFSK_CMD_START_TX_NOW;
    
    return 1U;
}

uint32_t hybrid_gfsk_enter_rx(void)
{
    if( hybrid_gfsk_ctxt.state != GENFSK_STATE_READY )
    {
        assert(FALSE);
        return 0U;
    }
    
    XCVR_SetActiveLL(XCVR_ACTIVE_LL_GENFSK);    
    hybrid_gfsk_ctxt.state = GENFSK_STATE_RX;

    hybrid_gfsk_whiten_init_set();
    
    GENFSK->IRQ_CTRL = GENFSK_IRQ_CTRL_GENERIC_FSK_IRQ_EN_MASK | GENFSK_IRQ_CTRL_RX_IRQ_EN_MASK | GENFSK_IRQ_CTRL_STATUS_W1C;      
    GENFSK->XCVR_CTRL = GENFSK_CMD_START_RX_NOW;
    
    return 1U;
}

void hybrid_gfsk_cancel(void)
{
    if( hybrid_gfsk_ctxt.state == GENFSK_STATE_TX || hybrid_gfsk_ctxt.state == GENFSK_STATE_RX )
    {
        // abort ongoing operation and wait until XCVR is free
        GENFSK->XCVR_CTRL = GENFSK_CMD_ABORT_ALL;
        while( (GENFSK->XCVR_CTRL & GENFSK_XCVR_CTRL_XCVR_BUSY_MASK) != 0U) {}
   
        hybrid_gfsk_to_btle();
    }
    else
    {
        // remains in off or ready state
    }
}

void hybrid_gfsk_disable(void)
{
    if( hybrid_gfsk_ctxt.state != GENFSK_STATE_OFF )
    {
        hybrid_gfsk_cancel();
        hybrid_gfsk_ctxt.callback = NULL;
        hybrid_gfsk_ctxt.state = GENFSK_STATE_OFF;
        NVIC_ClearPendingIRQ(GENLL_INT_IRQn);
        NVIC_DisableIRQ(GENLL_INT_IRQn);
    }
}

void hybrid_gfsk_enable(hybrid_gfsk_callback callback)
{
    // stop TX or RX if ongoing
    hybrid_gfsk_cancel();

    if( hybrid_gfsk_ctxt.state == GENFSK_STATE_OFF )
    {
        /* enable GENFSK interrupt */
        NVIC_ClearPendingIRQ(GENLL_INT_IRQn);
        NVIC_EnableIRQ(GENLL_INT_IRQn);
        NVIC_SetPriority(GENLL_INT_IRQn, GENFSK_IrqPriority_c >> (8 - __NVIC_PRIO_BITS));
    }

    hybrid_gfsk_ctxt.callback = callback;
    hybrid_gfsk_ctxt.state = GENFSK_STATE_READY;
}

uint32_t hybrid_gfsk_get_tx_pkt_dur_us(uint32_t packet_length)
{    
    // preamble
    packet_length += ((GENFSK->XCVR_CFG & GENFSK_XCVR_CFG_PREAMBLE_SZ_MASK) >> GENFSK_XCVR_CFG_PREAMBLE_SZ_SHIFT) + 1U;

    // CRC
    if( (RBME->CRCW_CFG & RBME_CRCW_CFG_CRCW_EN_MASK) != 0U )
    {
        packet_length += (RBME->CRCW_CFG3 & RBME_CRCW_CFG3_CRC_SZ_MASK) >> RBME_CRCW_CFG3_CRC_SZ_SHIFT;
    }

    // for now only 1Mbps is supported
    return (uint32_t) packet_length * 8U;
}

uint32_t hybrid_gfsk_get_rx_pkt_min_dur_us(void)
{    
    uint32_t min_packet_length;
  
    // preamble
    min_packet_length = ((GENFSK->XCVR_CFG & GENFSK_XCVR_CFG_PREAMBLE_SZ_MASK) >> GENFSK_XCVR_CFG_PREAMBLE_SZ_SHIFT) + 1U;
    
    // access address
    min_packet_length += ((GENFSK->PACKET_CFG & GENFSK_PACKET_CFG_SYNC_ADDR_SZ_MASK) >> GENFSK_PACKET_CFG_SYNC_ADDR_SZ_SHIFT) + 1U;
    
    // header size
    min_packet_length += (((GENFSK->PACKET_CFG & GENFSK_PACKET_CFG_H0_SZ_MASK) >> GENFSK_PACKET_CFG_H0_SZ_SHIFT) + 
                          ((GENFSK->PACKET_CFG & GENFSK_PACKET_CFG_H1_SZ_MASK) >> GENFSK_PACKET_CFG_H1_SZ_SHIFT) + 
                          ((GENFSK->PACKET_CFG & GENFSK_PACKET_CFG_LENGTH_SZ_MASK) >> GENFSK_PACKET_CFG_LENGTH_SZ_SHIFT) +
                          7U ) >> 3U;

    // CRC
    if( (RBME->CRCW_CFG & RBME_CRCW_CFG_CRCW_EN_MASK) != 0U )
    {
        min_packet_length += (RBME->CRCW_CFG3 & RBME_CRCW_CFG3_CRC_SZ_MASK) >> RBME_CRCW_CFG3_CRC_SZ_SHIFT;
    }

    // for now only 1Mbps is supported
    return (uint32_t) min_packet_length * 8U;
}

uint32 hybrid_gfsk_hal_enabled(void)
{
    return 1U;
}
