/*
 * Copyright 2021-2024 NXP
 * SPDX-License-Identifier: BSD-3-Clause
 */
/*
 * This file implements HADM hw abstract layer for KW platforms.
 */

/* === Includes ============================================================= */
#include "assert.h"
#include "EmbeddedTypes.h"
#include "fsl_common.h"
#include "fsl_ltc.h" /* LTC driver for AES */
/* maps LL types to SDK types */
#include "ll_types.h"
#include "ble_hadm_hal.h"
#include "nxp_xcvr_lcl_ctrl.h"
#include "lcl_hadm_measurement.h"
#include "lcl_hadm_utils.h"
#include "lcl_hadm_aes.h"
#include "fwk_platform.h"
/* === Macros =============================================================== */

//#define HADM_UT

#ifdef HADM_UT
#define HADM_UT_AA_LIST_SIZE 71
static const uint32_t aa_test_list[HADM_UT_AA_LIST_SIZE] = {
    0x215EE636,
    0x41118D59,
    0x3B737102,
    0xFC411F30,
    0xF9295459,
    0xD7E3E66B,
    0xF15812CE,
    0xA21AC849,
    0x91387EE9,
    0x307B9C7F,
    0xFC8C4845,
    0x3509C3E1,
    0x920D97D0,
    0xF72E9B9B,
    0x3EAE53C9,
    0x85C2CD96,
    0x86B53E4E,
    0x26B856C0,
    0x7F44B0AC,
    0x47163490,
    0xA24B3C46,
    0x0D9EB247,
    0xBA824ADF,
    0xB604A113,
    0xD75CD2C0,
    0x85D8AF24,
    0xA6D70D36,
    0xEAC8290A,
    0xB94F6BF3,
    0xE7BBC1B0,
    0x0E66A345,
    0xB7916E24,
    0xCD53F677,
    0x6FB14460,
    0xA72D22BB,
    0xE3911893,
    0x53F20ABB,
    0xECD0BD68,
    0x32055E16,
    0xE7146B1B,
    0x940E2CF2,
    0xCE2C8D7F,
    0x0B870D20,
    0x82470ACD,
    0x1213518F,
    0xC0304390,
    0x68F86FAC,
    0xD59A3C45,
    0x73566BDE,
    0x7117462F,
    0x2C2657C6,
    0xA27FBCA8,
    0x1E572646,
    0x532D598B,
    0x9933E15F,
    0x61DB0CE2,
    0x7E66AF42,
    0x641DCB45,
    0xF8B0D9B5,
    0xCF4857B5,
    0x41BC320B,
    0x6C09A9FA,
    0x64A3EA7F,
    0x81CA61AB,
    0x2EEF6882,
    0x35098FA5,
    0x0D2571F8,
    0x6218D02C,
    0x220D7CCE,
    0x1E2561DF,
    0xC68A11A8,
};
#endif // HADM_UT

/* === Types ================================================================ */

/* === Externals ============================================================ */

/* === Globals ============================================================= */

/* === Prototypes =========================================================== */

/* === Implementation ====================================================== */
#ifdef HADM_UT
static void BLE_HADM_ut_apply_aa_test_list (BLE_HADM_SubeventConfig_t *config)
{
    config->pnSeqNb = 0;
    for (int i=0; i<config->stepsNb; i++) 
    {
        if (config->chModePmAntMap[i].mode != 2)
        {
            config->pnList[config->pnSeqNb].pn1 = aa_test_list[config->pnSeqNb % HADM_UT_AA_LIST_SIZE];
            config->pnList[config->pnSeqNb].pn2 = aa_test_list[config->pnSeqNb % HADM_UT_AA_LIST_SIZE];
            config->pnSeqNb++;
        }
    }
}
#endif // HADM_UT

BLE_HADM_STATUS_t BLE_HADM_ProcedureInit(uint8 connIdx)
{
    lcl_hadm_init_procedure(connIdx);

    return HADM_HAL_SUCCESS;
}

BLE_HADM_STATUS_t BLE_HADM_Calibrate(BLE_HADM_rttPhyMode_t rate)
{
    BLE_HADM_STATUS_t status = HADM_HAL_SUCCESS;
#ifndef SIMULATOR
    status = lcl_hadm_calibrate_dcoc(rate);

    if (HADM_HAL_SUCCESS == status)
        status = lcl_hadm_calibrate_pll(rate);
#endif
    return status;
}

void BLE_HADM_SetZeroDistanceCompensationData(BLE_HADM_ZeroDistanceCompensationData_t *compData)
{
    hadm_device.zero_distance_comp = *compData;
}
                                            
BLE_HADM_STATUS_t BLE_HADM_SubeventCheckConfig(const BLE_HADM_SubeventConfig_t *config)
{
    BLE_HADM_STATUS_t status;

    status = lcl_hadm_check_config(config);
        
    return status;
}

BLE_HADM_STATUS_t BLE_HADM_SubeventConfigApply(const BLE_HADM_SubeventConfig_t *config)
{
    BLE_HADM_STATUS_t status;
    
#ifdef HADM_UT
    BLE_HADM_ut_apply_aa_test_list(config);
#endif // HADM_UT

    status = lcl_hadm_configure(config);

    return status;
}

BLE_HADM_STATUS_t BLE_HADM_SubeventStart(const BLE_HADM_SubeventConfig_t *config)
{
    BLE_HADM_STATUS_t status;
    status = lcl_hadm_run_measurement(config);
    
    return status;
}

void BLE_HADM_ProcedureStop(uint8 connIdx)
{
    lcl_hadm_stop_procedure(connIdx);
}

void BLE_HADM_SubeventStop(const BLE_HADM_SubeventConfig_t *config)
{
    lcl_hadm_stop_measurement(config);
}

BLE_HADM_STATUS_t BLE_HADM_Init(void)
{
    return lcl_hadm_init();
}


const BLE_HADM_HalProperties_t *BLE_HADM_GetProperties(void)
{
    return lcl_hadm_get_properties();
}

uint16 BLE_HADM_GetPrepareTime(const BLE_HADM_SubeventConfig_t *hadm_config)
{
  return lcl_hadm_get_prepare_time(hadm_config);
}

const BLE_HADM_HalCapabilities_t *BLE_HADM_GetCapabilities(void)
{
    return lcl_hadm_get_capabilities();
}

BLE_HADM_STATUS_t BLE_HADM_SetAntennaType(uint8 *antBoardTable)
{
    return lcl_hadm_set_antenna_type(antBoardTable);
}

void BLE_HADM_SetDmaDebugBuff(uint16 dma_debug_buff_size, uint32 dma_debug_buff_address)
{
    lcl_hadm_set_dma_debug_buffer(dma_debug_buff_size, dma_debug_buff_address);
}

#define HADM_OPT_LTC_DRV
#define KEY_SIZE	16U
void BLE_HADM_Drbg_AES_wrapper(uint8 *target, const uint8 *source, uint32 size_buff, const uint8 *key, const int cbc, const uint8 *iv )
{
    status_t status = kStatus_Success;
#ifndef SIMULATOR
    if (cbc == 1) 
    {
        assert(iv != NULL);
        status = LTC_AES_EncryptCbc(LTC0, source, target, size_buff, iv, key, KEY_SIZE);
    }
    else 
    {
#ifdef HADM_OPT_LTC_DRV
        lcl_hadm_AES_EncryptEcb_128((const uint32 *)key, (const uint32 *)source, (uint32 *)target);
#else
        status = LTC_AES_EncryptEcb(LTC0, source, target, size_buff, key, KEY_SIZE);
#endif
    }
#endif
    assert(kStatus_Success == status);
    (void)status;
}


void BLE_HADM_ReleaseResultsBuffer(BLE_HADM_SubeventResultsData_t **result_p)
{
    if ((*result_p) != NULL)
    {
        (*result_p)->resultBufferUsed = 0;
        *result_p = NULL;
    }
}

void BLE_HADM_ReleaseConfigBuffer(BLE_HADM_SubeventConfig_t **config_p)
{
    if ((*config_p) != NULL)
    {
        (*config_p)->configBufferUsed = 0;
        *config_p = NULL;
    }
}

BLE_HADM_SubeventConfig_t * BLE_HADM_GetConfigBuffer(void)
{
    return lcl_hadm_utils_get_config_buffer();
}