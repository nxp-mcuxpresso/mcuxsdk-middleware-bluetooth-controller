/*
 * Copyright 2024 NXP
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "lcl_xcvr_simu.h"
#include "EmbeddedTypes.h"
#include "lcl_hadm_measurement.h"
#include "lcl_xcvr_hal.h"

/* From lcl_hadm_measurement.c */
void lcl_hadm_irq_handler(void *userData, bool abort, uint32_t rsm_csr);

static const uint8_t rtt_type_2_payload_size[7U] = {0U, 1U, 3U, 1U, 2U, 3U, 4U}; /* in 32 bits words */

xcvr_lcl_rsm_config_t SIMU_rsm_settings;

#define CONCAT_(A, B, C, D) A##B##C##D
#define CONCAT(A, B, C, D) CONCAT_(A, B, C, D)
// Get reg->field value
#define GET_REG_VAL(BASE, R, FIELD) ((uint32_t)((BASE->R & CONCAT(BASE##_, R##_, FIELD##_, MASK)) >> (CONCAT(BASE##_, R##_, FIELD##_, SHIFT))))

uint32_t get_config_step_size(uint32_t mode, uint32_t rtt_type)
{
    uint32_t size;
 
    switch(mode) {
    case 0:
        size = 5;
        break;
    case 1:
    case 3:
        size = 5 + 2*rtt_type_2_payload_size[rtt_type];
        break;
    case 2:
        size = 3;
        break;
    default:
        assert(0);
        break;
    }
    return size;
}

/* Buffer location: 0=Tx Pkt RAM, 1=RX PktRAM */
#define PKT_RAM_BASE_ADDR(buffLoc) (((buffLoc) == 0)?(uint32_t*)TX_PACKET_RAM:(uint32_t*)RX_PACKET_RAM)

  static uint32_t RSM_step_idx = 0;
  static uint32_t RSM_config_curr_page = 0;
  static uint32_t RSM_result_curr_page = 0;

/*
   Run RSM sequence for RSM_INT_NBSTEP steps.
   Return TRUE if sequence is complete.
 */
bool_t SIMU_LCL_RunSteps(bool start)
{
    /* Start a new RSM sequence */
    if(start)
    {
        RSM_step_idx = 0;
    }
    uint32_t num_steps_per_int = (uint32_t)GET_REG_VAL(XCVR_MISC, RSM_CONFIG_BUFF, RSM_INT_NBSTEP);
    uint32_t config_base_offset = (uint32_t)GET_REG_VAL(XCVR_MISC, RSM_CONFIG_BUFF, RSM_CONFIG_BASE_ADDR);
    uint32_t *config_base;
    uint32_t config_buff_len = GET_REG_VAL(XCVR_MISC, RSM_CONFIG_BUFF, RSM_CONFIG_DEPTH);
    uint32_t config_start_offset = (uint32_t)GET_REG_VAL(XCVR_MISC, RSM_CONFIG_PTR, RSM_CONFIG_START_PTR);
    uint32_t *config_start;
    uint32_t config_wr_ptr_offset = (uint32_t)GET_REG_VAL(XCVR_MISC, RSM_CONFIG_PTR, RSM_CONFIG_WR_PTR);
    uint32_t config_rd_ptr_offset = start ? 0:(uint32_t)GET_REG_VAL(XCVR_MISC, RSM_PTR, RSM_RD_PTR);   
    uint32_t result_base_offset = (uint32_t)GET_REG_VAL(XCVR_MISC, RSM_RESULT_BUFF, RSM_RESULT_BASE_ADDR);
    uint32_t *result_base;
    uint32_t result_buff_len = GET_REG_VAL(XCVR_MISC, RSM_RESULT_BUFF, RSM_RESULT_DEPTH);
    uint32_t result_start_offset = (uint32_t)GET_REG_VAL(XCVR_MISC, RSM_RESULT_PTR, RSM_RESULT_START_PTR);
    uint32_t *result_start;
    uint32_t result_wr_ptr_offset = start ? 0:(uint32_t)GET_REG_VAL(XCVR_MISC, RSM_PTR, RSM_WR_PTR);
    uint32_t config_wr_page = GET_REG_VAL(XCVR_MISC, RSM_CONFIG_PTR, RSM_CONFIG_WR_PAGE);
    uint32_t result_r_page = GET_REG_VAL(XCVR_MISC, RSM_RESULT_PTR, RSM_RESULT_RD_PAGE);
    uint32_t result_rd_ptr_offset = GET_REG_VAL(XCVR_MISC, RSM_RESULT_PTR, RSM_RESULT_RD_PTR);
    
    uint32_t config_max_step_size = LCL_HAL_PKT_RAM_STEP_CONFIG_MODE13_SIZE + rtt_type_2_payload_size[SIMU_rsm_settings.rtt_type] * 2U; /* taking mode1/3 into account */
    uint32_t result_max_step_size = (LCL_HAL_PKT_RAM_STEP_RESULT_MODE01_SIZE + SIMU_rsm_settings.num_ant_path + 1U); /* taking mode3 into account */

    config_base = ((uint32_t *)PKT_RAM_BASE_ADDR(GET_REG_VAL(XCVR_MISC, RSM_CONFIG_BUFF, RSM_CONFIG_BUFF_LOC))) + config_base_offset;
    result_base = ((uint32_t *)PKT_RAM_BASE_ADDR(GET_REG_VAL(XCVR_MISC, RSM_RESULT_BUFF, RSM_RESULT_BUFF_LOC))) + result_base_offset;
    
    config_start = config_base + config_start_offset;
    result_start = result_base + result_start_offset;

    uint32_t *step_config_p = config_start + config_rd_ptr_offset;
    uint32_t *step_result_p = result_start + result_wr_ptr_offset;
    
#ifdef SIMULATOR_PRINTF
    if(start)
    {
        printf("RSM config nbSteps=%d\n", SIMU_rsm_settings.num_steps);
        printf("PKT RAM config addr=%x depth=%d\n", config_start, config_buff_len);
        printf("PKT RAM result addr=%x depth=%d\n", result_start, result_buff_len);
    }
#endif

    uint32_t step_iter = 0;
    while(RSM_step_idx < SIMU_rsm_settings.num_steps)
    {

        /* Move to next config step */
        if ((step_config_p + config_max_step_size) > config_base + config_buff_len)
        {
            /* Need to wrap now: rewind write pointer to start and toggle page */
            step_config_p = config_base;
            RSM_config_curr_page ^= 1U;
        }

        /* Move to next result step */
        if ((step_result_p + result_max_step_size) > result_base + result_buff_len)
        {
            /* Need to wrap now: rewind write pointer to start and toggle page */
            step_result_p = result_base;
            RSM_result_curr_page ^= 1U;
        }

        /* Check for config underrun or results overflow */
        uint32_t *config_wr_ptr = config_base + config_wr_ptr_offset;
        uint32_t *result_r_ptr = result_base + result_rd_ptr_offset;
        if (((RSM_config_curr_page == config_wr_page) && (config_wr_ptr <= step_config_p)) ||
            ((RSM_config_curr_page != config_wr_page) && (config_wr_ptr >= step_config_p)) ||
            ((RSM_result_curr_page == result_r_page) && (step_result_p < result_r_ptr)) ||
            ((RSM_result_curr_page != result_r_page) && (step_result_p > result_r_ptr)))
        {
            XCVR_MISC->RSM_INT_STATUS = XCVR_MISC_RSM_INT_STATUS_RSM_IRQ_ABORT(1);
#ifdef SIMULATOR_PRINTF
            printf("RSM INT ABORT! (step=%d)\n", RSM_step_idx);
#endif
            RSM_INT_IRQHandler();
            return true;
        }
        
        /* Decode config & encode results */
        COM_MODE_013_CFG_HDR_Type *step_config_s = (COM_MODE_013_CFG_HDR_Type *)(void *)step_config_p;
        COM_RES_HDR_Type *step_result_hdr_s = (COM_RES_HDR_Type *)(void *)step_result_p;
        uint32_t mode = (step_config_s->STEP_CFG & COM_MODE_013_CFG_HDR_STEP_CFG_MODE_MASK) >> COM_MODE_013_CFG_HDR_STEP_CFG_MODE_SHIFT;
        uint32_t config_step_size = get_config_step_size(mode, SIMU_rsm_settings.rtt_type);
        uint32_t result_step_size = 0;
        uint16_t channel = (step_config_s->CHANNEL_NUM & COM_MODE_013_CFG_HDR_CHANNEL_NUM_CHANNEL_NUM_MASK) >> COM_MODE_013_CFG_HDR_CHANNEL_NUM_CHANNEL_NUM_SHIFT;
        uint32_t ant_perm = (step_config_s->STEP_CFG & COM_MODE_013_CFG_HDR_STEP_CFG_ANT_PERMUT_MASK) >> COM_MODE_013_CFG_HDR_STEP_CFG_ANT_PERMUT_SHIFT;
        
        /* Check for channel validity */
        if (channel & 0x8000)
        {
            channel = ((channel & 0x7FFF) * 2) - 1;
        }
        else
        {
            channel *= 2;
        }
        assert(channel < 79);
        
        /* Check for antenna permutation validity */
        assert(ant_perm < 24);

        step_result_hdr_s->STEP_ID = RSM_step_idx++;
        step_result_hdr_s->SIZE_AGC_IDX = 0;
        step_result_hdr_s->PBCD_CTUNE_AA_DET = 0xFFFF;
        result_step_size += sizeof(COM_RES_HDR_Type);
  
        switch (mode) {
        case 0:
        case 1:
        case 3:
            {
                //assert(step_config_s->AA_INIT == 0xDEADBEEF);
                //assert(step_config_s->AA_REFL == 0xCAFEFADE);
        
                COM_MODE_013_RES_BODY_Type *step_result_013_s = (COM_MODE_013_RES_BODY_Type *)(void *)(((uint8_t*)step_result_p) + result_step_size);
                step_result_013_s->NADM_ERROR_RSSI = COM_MODE_013_RES_BODY_NADM_ERROR_RSSI_RSSI_NB(0xCC) | COM_MODE_013_RES_BODY_NADM_ERROR_RSSI_RAW_NADM_PAY_ERR(0);
                step_result_013_s->RTT_RESULT =
                  COM_MODE_013_RES_BODY_RTT_RESULT_RTT_VLD(1) |
                  COM_MODE_013_RES_BODY_RTT_RESULT_RTT_FOUND(1) |
                  COM_MODE_013_RES_BODY_RTT_RESULT_RTT_CFO(0) |
                  COM_MODE_013_RES_BODY_RTT_RESULT_RTT_INT_ADJ(0) |
                  COM_MODE_013_RES_BODY_RTT_RESULT_RTT_HAM_DIST_SAT(0) |
                  COM_MODE_013_RES_BODY_RTT_RESULT_RTT_P_DELTA(0);
                step_result_013_s->CFO_EST = COM_MODE_013_RES_BODY_CFO_EST_CFO_EST(0);
                step_result_013_s->TIMESTAMP = COM_MODE_013_RES_BODY_TIMESTAMP_TPM(0x85B00);
                result_step_size += sizeof(COM_MODE_013_RES_BODY_Type);
            }
            break;
         case 2:
            break;
        default:
            assert(FALSE);
            break;
        }
        switch (mode) {
        case 2:
        case 3:
            {
                uint32_t *step_result_iq_s = (uint32_t *)(void *)(((uint8_t*)step_result_p) + result_step_size);
                for (int i=0; i<SIMU_rsm_settings.num_ant_path+1; i++)
                {
                    step_result_iq_s[i] = 0xAAA0BBB0 + i;
                }
                result_step_size += sizeof(uint32_t)*(SIMU_rsm_settings.num_ant_path+1);
            }
            break;
        case 0:
        case 1:
            break;
        default:
            assert(FALSE);
            break;
        }       
        assert(result_step_size%4 == 0);
        result_step_size /= 4;
        step_result_hdr_s->SIZE_AGC_IDX = COM_RES_HDR_SIZE_AGC_IDX_RESULT_SIZE(result_step_size);

        step_config_p += config_step_size;
        step_result_p += result_step_size;

        XCVR_MISC->RSM_PTR =
          XCVR_MISC_RSM_PTR_RSM_WR_PAGE(RSM_result_curr_page) |
          XCVR_MISC_RSM_PTR_RSM_WR_PTR(((uint32_t)step_result_p)>>2) |
          XCVR_MISC_RSM_PTR_RSM_RD_PAGE(RSM_config_curr_page) |
          XCVR_MISC_RSM_PTR_RSM_RD_PTR(((uint32_t)step_config_p)>>2);

        /* Stop reading config when RSM_INT_NBSTEP is reached */
        step_iter ++;
        if (step_iter == num_steps_per_int)
        {
            XCVR_MISC->RSM_INT_STATUS = XCVR_MISC_RSM_INT_STATUS_RSM_IRQ_STEP(1);
#ifdef SIMULATOR_PRINTF
            printf("RSM INT IRQ_STEP (%d)\n", num_steps_per_int);
#endif
            RSM_INT_IRQHandler();
            break;
        }

    }

#ifdef SIMULATOR_PRINTF
    printf("RSM has run to step_idx = %d\n", RSM_step_idx);
#endif
    
    if (RSM_step_idx >= SIMU_rsm_settings.num_steps)
    {
        XCVR_MISC->RSM_INT_STATUS = XCVR_MISC_RSM_INT_STATUS_RSM_IRQ_EOS(1);
#ifdef SIMULATOR_PRINTF
        printf("RSM INT EOS\n");
#endif
        RSM_INT_IRQHandler();
        return true;
    }
    else
    {
      return false;
    }
}


enum {
     SIMU_Idle,
     SIMU_Run,
} SIMU_Action = SIMU_Idle;

/* This is called to simulate HW trigger for the RSM */
xcvrLclStatus_t SIMU_LCL_RsmGo(XCVR_RSM_RXTX_MODE_T role, const xcvr_lcl_rsm_config_t * rsm_settings_ptr)
{
    SIMU_rsm_settings = *rsm_settings_ptr;
    SIMU_Action = SIMU_Run;
    return gXcvrLclStatusSuccess;
}

void SIMU_LCL_RunRsmSequence(void)
{
    bool_t done;
    bool start = TRUE;

    do {
        switch (SIMU_Action)
        {
        case SIMU_Idle:
            break;
        case SIMU_Run:
            {
                done = SIMU_LCL_RunSteps(start);
                start = FALSE;
            }
            break;
        default:
          break;
        }
    } while(!done);
    SIMU_Action = SIMU_Idle;
}
