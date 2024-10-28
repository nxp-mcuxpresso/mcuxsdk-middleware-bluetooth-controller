/*
 * Copyright 2024 NXP
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* Prevent double inclusion */
#ifndef _LCL_HADM_AES_H_
#define _LCL_HADM_AES_H_

/* === Includes ============================================================ */

/* === Types =============================================================== */

/* === Macros ============================================================== */

/* === Prototypes ========================================================== */

/* Simplified version of LTC driver to ECB encrypt 128 bits text using a 128 bits key */
void lcl_hadm_AES_EncryptEcb_128(const uint32_t *key, const uint32_t *plaintext, uint32_t *ciphertext);


#endif /* _LCL_HADM_AES_H_ */

/* EOF */
