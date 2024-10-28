/*
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#if defined(gAppEnableHybridGenfsk_d) && (gAppEnableHybridGenfsk_d!=0)

#include "fsl_common.h"
#include "fsl_device_registers.h"
#include "hybrid_gfsk.h"

/* BLE channels mapped to GENFSK radio channels */
static const uint8_t hybrid_gfsk_ble_channel_to_generic_channel[40] =
{
     44U,  46U,  48U,  50U,  52U,  54U,  56U,  58U,  60U,  62U,
     64U,  68U,  70U,  72U,  74U,  76U,  78U,  80U,  82U,  84U,
     86U,  88U,  90U,  92U,  94U,  96U,  98U, 100U, 102U, 104U,
    106U, 108U, 110U, 112U, 114U, 116U, 118U,  42U,  66U, 120U
};

void hybrid_gfsk_init(void)
{
    uint32_t temp;
    
    // enable or disable BTLL & GENLL clock overrides
    temp = RADIO_CTRL->RF_CLK_CTRL;
    temp |= RADIO_CTRL_RF_CLK_CTRL_BTLL_CLK_EN_OVRD(1U);
    temp |= RADIO_CTRL_RF_CLK_CTRL_GENLL_CLK_EN_OVRD(1U);

    RADIO_CTRL->RF_CLK_CTRL = temp;

    // set to default config
    hybrid_gfsk_set_default_config();
}

/* Check the reference manual for GENFSK / RBME register details.
   The GENFSK and RBME registers can be updated only if GENFSK is not enabed in the controller. */
void hybrid_gfsk_set_default_config(void)
{   
    /* Configure packet format. */
    GENFSK->PACKET_CFG = GENFSK_PACKET_CFG_LENGTH_SZ(8U) |
                         GENFSK_PACKET_CFG_LENGTH_BIT_ORD(0U) |
                         GENFSK_PACKET_CFG_SYNC_ADDR_SZ(3U) |
                         GENFSK_PACKET_CFG_AA_PLAYBACK_CNT(0U) |
                         GENFSK_PACKET_CFG_LL_FETCH_AA(0U) |
                         GENFSK_PACKET_CFG_H0_SZ(8U) |
                         GENFSK_PACKET_CFG_H1_SZ(0U);

    /* exclude CRC from the length in the header */
    GENFSK->LENGTH_ADJ = GENFSK_LENGTH_ADJ_LENGTH_ADJ(3U);

    /* disable H0 check */
    GENFSK->H0_CFG = GENFSK_H0_CFG_H0_MATCH(0U) |
                     GENFSK_H0_CFG_H0_MASK(0U);

    /* disable H1 check */
    GENFSK->H1_CFG = GENFSK_H1_CFG_H1_MATCH(0U) |
                     GENFSK_H1_CFG_H1_MASK(0U);

    /* accept packet with invalid CRC */
    GENFSK->CRC_CFG = GENFSK_CRC_CFG_CRC_IGNORE(1U);

    /* address 4 bytes, no error tolerence, enable address 0 */
    GENFSK->NTW_ADR_CTRL = GENFSK_NTW_ADR_CTRL_NTW_ADR_SZ(3U) |
                           GENFSK_NTW_ADR_CTRL_NTW_ADR_THR(0U) |
                           GENFSK_NTW_ADR_CTRL_NTW_ADR_EN(1U);

    /* preamble, whitening */
    GENFSK->XCVR_CFG = GENFSK_XCVR_CFG_PREAMBLE_SEL(0U) |
                       GENFSK_XCVR_CFG_GEN_PREAMBLE(0U) |
                       GENFSK_XCVR_CFG_PREAMBLE_SZ(0U) |
                       GENFSK_XCVR_CFG_SW_CRC_EN(0U) |
                       GENFSK_XCVR_CFG_RX_DEWHITEN_DIS(0U) |
                       GENFSK_XCVR_CFG_TX_WHITEN_DIS(0U);

    /* GLL mode, data rate configuration bank 0 */
    GENFSK->ENH_FEATURE = GENFSK_ENH_FEATURE_GENLL_MODE(0U) |
                          GENFSK_ENH_FEATURE_DATARATE_CONFIG_SEL(0U);

    /* set to max tx power */
    GENFSK->TX_POWER = GENFSK_TX_POWER_TX_POWER(62U); 

    /* enable CRC */
    RBME->CRCW_CFG = RBME_CRCW_CFG_CRCW_EN(1U);

    /* CRC 3 bytes */
    RBME->CRCW_CFG3 = RBME_CRCW_CFG3_CRC_SZ(3U) |
                      RBME_CRCW_CFG3_CRC_START_BYTE(4U) |
                      RBME_CRCW_CFG3_CRC_REF_IN(0U) |
                      RBME_CRCW_CFG3_CRC_REF_OUT(0U) |
                      RBME_CRCW_CFG3_CRC_BYTE_ORD(0U);

    /* CRC init and polynomial */
    RBME->CRC_INIT = RBME_CRC_INIT_CRC_SEED(0x55555500U);
    RBME->CRC_POLY = RBME_CRC_POLY_CRC_POLY(0x00065B00U);
    RBME->CRC_XOR_OUT = RBME_CRC_XOR_OUT_CRC_XOR_OUT(0x00U);

    /* WHITEN_INIT must be initialized to 0 and it is set by the controller: 
       - BLE channels: compatible with BLE spec for generic channels 42, 44, 46, ..., 120
       - non BLE channels: 0x40 + GENFSK->CHANNEL_NUM0 */
    RBME->WHITEN_CFG = RBME_WHITEN_CFG_WHITEN_START(1U) |
                       RBME_WHITEN_CFG_WHITEN_END(1U) |
                       RBME_WHITEN_CFG_WHITEN_B4_CRC(0U) |
                       RBME_WHITEN_CFG_WHITEN_POLY_TYPE(0U) |
                       RBME_WHITEN_CFG_WHITEN_REF_IN(0U) |
                       RBME_WHITEN_CFG_WHITEN_PAYLOAD_REINIT(0U) |
                       RBME_WHITEN_CFG_WHITEN_SIZE(7U) |
                       RBME_WHITEN_CFG_WHITEN_INIT(/*!!! DO NOT CHANGE !!!*/0U/*!!! DO NOT CHANGE !!!*/);
    
    RBME->WHITEN_POLY = RBME_WHITEN_POLY_WHITEN_POLY(0x0004);

    RBME->WHITEN_SZ_THR = RBME_WHITEN_SZ_THR_WHITEN_SZ_THR(0);

    /* set to BLE advertising channel */
    GENFSK->CHANNEL_NUM0 = hybrid_gfsk_ble_channel_to_generic_channel[37U];
    GENFSK->NTW_ADR_0 = 0x8E89BED6; /* Used by RX only */
}
#endif // defined(gAppEnableHybridGenfsk_d) && (gAppEnableHybridGenfsk_d!=0)
