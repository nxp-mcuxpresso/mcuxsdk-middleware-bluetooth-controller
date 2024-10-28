/*                Copyright 2021-2024, NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
 
/* WARNING: DO NOT MODIFY THIS FILE directly as this file is opied from LL. */

#ifndef BLE_DEBUG_STRUCT_H_
#define BLE_DEBUG_STRUCT_H_

#define NB_BUF_ENTRIES 23U
typedef struct
{
  /*1 x 32 bit*/
  uint32 reserved0; /*Reserved. Do not use.*/

  uint32    cfsr;
  union
  {
    uint32 mmfar;
    uint32 bfar;
  }u;
  uint32 pc;
  uint32 lr;
  uint32 psp;
  uint32 psr;
  uint32 r0;
  uint32 r1;
  uint32 r2;
  uint32 r3;
  uint32 r4;
  uint32 r5;
  uint32 r6;
  uint32 r7;
  uint32 r8;
  uint32 r9;
  uint32 r10;
  uint32 r11;
  uint32 r12;
}DEBUG_STRUCT_EXCEPTION;
typedef struct BLE_LM_RES_STATE_NBU_T
{
  // Scan BIT0;
  // Init BIT1;
  // Test BIT2;
  // Chas BIT3;
  // reserve:4;
  // Above order must match the enum definition
  uint8 ucCurState;
  uint8 StatesCnt[4]; //Store Total number of each state
} BLE_LM_RES_STATE_NBU;
typedef struct
{
  /*3 x 32 bit*/
  uint16 error_bitmask;
  uint16 exception_id;
  uint32 task_stack_overflow_mask;
  uint32 idle_task_free_running_counter;
  /*2 x 32 bit*/
  BLE_LM_RES_STATE_NBU lm_state_status;
  uint8 adv_sched_free_running_counter;
  uint8 scan_sched_free_running_counter;
  uint8 init_scan_sched_free_running_counter;

  union
  {
#ifdef BLE_DEBUG_LL
    DEBUG_STRUCT_LEGACY     dbg_legacy;
#endif /*BLE_DEBUG_LL*/
    DEBUG_STRUCT_EXCEPTION  dbg_exception;
  }u;
}DBG_STRUCT_CUST;

union info
{
#ifdef BLE_DEBUG_LL
  DBG_STRUCT_EP   ep;
#endif /*BLE_DEBUG_LL*/
  DBG_STRUCT_CUST cust;
  uint32 mem[NB_BUF_ENTRIES + 3];
};

  /*30 x 32 bit*/
typedef struct
{
  /*1 x 32 bit*/
  uint8 mode;
  uint8 error_count;
  uint8 warning_count;
  uint8 issue_triggered;

  /*1 x 32 bit*/
  uint32 length;
  /*1 x 32 bit*/
  uint32 nbu_sha1;

  /*26 x 32 bit maximum*/
  union info dbg_info;

  /*1 x 32 bit*/
  uint32 reserved2;
}DEBUG_STRUCT;
#endif // BLE_DEBUG_STRUCT