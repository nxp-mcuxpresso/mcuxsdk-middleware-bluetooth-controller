/**
 * @file lcl_hadm_aes.c
 *
 * This file implements AES service for HADM
 * It is a much faster LTC driver for 128 AES ECB for HADM DRBG
 */
/*
 * Copyright 2024 NXP
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* === Includes ============================================================ */
#include "fsl_os_abstraction.h"
#include "EmbeddedTypes.h"
#include "fsl_ltc.h"

/* === Types =============================================================== */

/* === Macros ============================================================== */

#define HADM_DRBG_BLOCK_SIZE	(16U) /* 128 bits data block size */
#define HADM_DRBG_KEY_SIZE	(16U) /* 128 bits key */

/* === Globals ============================================================= */

/* === Externals =========================================================== */

/* === Prototypes ========================================================== */

/* === Implementation ====================================================== */

void lcl_hadm_AES_EncryptEcb_128(const uint32_t *key, const uint32_t *plaintext, uint32_t *ciphertext)
{
    /* Clear internal register states. */
    LTC0->CW = (uint32_t)(LTC_CW_CM_MASK | LTC_CW_CDS_MASK | LTC_CW_CICV_MASK | LTC_CW_CCR_MASK | LTC_CW_CKR_MASK | LTC_CW_COF_MASK | LTC_CW_CIF_MASK);

    /* Set byte swap on for several registers we will be reading and writing
     * user data to/from. */
    LTC0->CTL |= (uint32_t)(LTC_CTL_IFS_MASK | LTC_CTL_OFS_MASK | LTC_CTL_KIS_MASK | LTC_CTL_KOS_MASK | LTC_CTL_CIS_MASK | LTC_CTL_COS_MASK);

    /* Write the key in place. */
    LTC0->KEY[0u] = *key++;
    LTC0->KEY[1u] = *key++;
    LTC0->KEY[2u] = *key++;
    LTC0->KEY[3u] = *key;

    /* Write the key size.  This must be done after writing the key, and this
     * action locks the ability to modify the key registers. */
    LTC0->KS = HADM_DRBG_KEY_SIZE;
    
    /* Clear the 'done' interrupt. */
    LTC0->STA = (uint32_t)LTC_STA_DI_MASK;
    
    /* Set the proper block and algorithm mode. */
    LTC0->MD = (uint32_t)kLTC_AlgorithmAES | (uint32_t)kLTC_ModeEncrypt | (uint32_t)kLTC_ModeUpdate | (uint32_t)kLTC_ModeECB;
    
    /* Write the data size. */
    LTC0->DS = HADM_DRBG_BLOCK_SIZE;
    
    /* Write to input FIFO */
    LTC0->IFIFO = *plaintext++;
    LTC0->IFIFO = *plaintext++;
    LTC0->IFIFO = *plaintext++;
    LTC0->IFIFO = *plaintext++;

    /* Wait for first word to be ready, then read all the content of the output FIFO */
    while ((LTC0->FIFOSTA & LTC_FIFOSTA_OFL_MASK) < 4U);
    *ciphertext++ = LTC0->OFIFO;
    *ciphertext++ = LTC0->OFIFO;
    *ciphertext++ = LTC0->OFIFO;
    *ciphertext++ = LTC0->OFIFO;

}

/* EOF */
