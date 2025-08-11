// -------------------------------------------------------------------------
//  @file:    nbu_ble_wrap.s
//  @purpose: This file wraps the CM3 SDK vector table to the NBU LL handlers
//            K32WB41Z83_NBU
//  @version: 1.0
//  @date:    2020-5-12
//  @build:   b200716
// -------------------------------------------------------------------------
//
// Copyright 2020-2024 NXP
//
// SPDX-License-Identifier: BSD-3-Clause
//
//
// Cortex-M version
//
  .syntax unified
  .arch armv8-m.main
  .section .text:CODE:REORDER:NOROOT(2)

  .extern     DispatchIRQ
  .extern     BleDispatchIRQ
  .extern     PendSV_Handler
  .extern     _tx_timer_interrupt
  .extern     _tx_thread_system_stack_ptr
  .extern     __vector_table
  
  .global     SVC_Handler
  .global     SysTick_Handler
  .global     BLE_INT0_IRQHandler
  .global     BLE_INT0_IRQHandler
  .global     BLE_INT1_IRQHandler
  .global     BLE_INT2_IRQHandler
  .global     BTU_FIQ_IRQHandler
  .global     BTU_INT_IRQHandler
  .global     CTI_IRQ0_IRQHandler
  .global     T2_INT_IRQHandler
  .global     T3_INT_IRQHandler
  .global     T4_INT_IRQHandler
  .global     SI_INT_IRQHandler
  .global     CTI_IRQ1_IRQHandler
  .global     _tx_initialize_low_level
// external Interrupts

  .align 1
  .thumb_func
  .type BLE_INT0_IRQHandler, %function
BLE_INT0_IRQHandler:
  MOV     R0, #0
  LDR     R1, =BleDispatchIRQ
  BX      R1
  .size BLE_INT0_IRQHandler, . - BLE_INT0_IRQHandler
  
  .align 1
  .thumb_func
  .type BLE_INT1_IRQHandler, %function
BLE_INT1_IRQHandler:
  MOV     R0, #1
  LDR     R1, =BleDispatchIRQ
  BX      R1
  .size BLE_INT1_IRQHandler, . - BLE_INT1_IRQHandler

  .align 1
  .thumb_func
  .type BLE_INT2_IRQHandler, %function
BLE_INT2_IRQHandler:
  MOV     R0, #2
  LDR     R1, =BleDispatchIRQ
  BX      R1
  .size BLE_INT2_IRQHandler, . - BLE_INT2_IRQHandler

  .align 1
  .thumb_func
BTU_FIQ_IRQHandler:
BTU_INT_IRQHandler:
CTI_IRQ0_IRQHandler:
T2_INT_IRQHandler:
T3_INT_IRQHandler:
T4_INT_IRQHandler:
SI_INT_IRQHandler:
CTI_IRQ1_IRQHandler:
  LDR     R1, =DispatchIRQ
  BX      R1

  .align 1
  .thumb_func
  .type SVC_Handler, %function
SVC_Handler:
  B     PendSV_Handler
  .size SVC_Handler, . - SVC_Handler
  
  .align 1
  .thumb_func
  .type SysTick_Handler, %function
SysTick_Handler:
  LDR     R1, =_tx_timer_interrupt
  BX      R1
  .size SysTick_Handler, . - SysTick_Handler

// ThreadX low level init
_tx_initialize_low_level:
  LDR r0, =_tx_thread_system_stack_ptr    // Build address of system stack pointer
  LDR r1, =__VECTOR_TABLE                 // Pickup address of vector table
  LDR r1, [r1]                            // Pickup reset stack pointer
  STR r1, [r0]                            // Save system stack pointer

  MOV r0, #0xE000E000                     // Build address of NVIC registers

  LDR r1, =0x00000010                     // DIV_0_TRP
  STR r1, [r0, #0xD14]                    // Configuration Control Register setting

  LDR r1, =0x00000000                     // Rsrv, UsgF, BusF, MemM
  STR r1, [r0, #0xD18]                    // Setup System Handlers 4-7 Priority Registers

  LDR r1, =0xFF000000                     // SVCl, Rsrv, Rsrv, Rsrv
  STR r1, [r0, #0xD1C]                    // Setup System Handlers 8-11 Priority Registers
                                          // Note: SVC must be lowest priority, which is 0xFF
  LDR r1, =0x40FF0000                     // SysT, PnSV, Rsrv, DbgM
  STR r1, [r0, #0xD20]                    // Setup System Handlers 12-15 Priority Registers
                                          // Note: PnSV must be lowest priority, which is 0xFF
  LDR r1, [r0, #0xD24]
  ORR r1, #0x70000
  STR r1, [r0, #0xD24]
//
//    /* Return to caller.  */
//    
  BX  lr 

//        END
