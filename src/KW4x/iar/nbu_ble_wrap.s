; -------------------------------------------------------------------------
;  @file:    nbu_ble_wrap.s
;  @purpose: This file wraps the CM3 SDK vector table to the NBU LL handlers
;            K32WB41Z83_NBU
;  @version: 1.0
;  @date:    2020-5-12
;  @build:   b200716
; -------------------------------------------------------------------------
;
; Copyright 2020-2024 NXP
;
; SPDX-License-Identifier: BSD-3-Clause
;
;
; Cortex-M version
;

  SECTION .text:CODE:REORDER:NOROOT(2)

  PUBLIC     SVC_Handler
  PUBLIC     SysTick_Handler
  PUBLIC     BLE_INT0_IRQHandler
  PUBLIC     BLE_INT0_IRQHandler
  PUBLIC     BLE_INT1_IRQHandler
  PUBLIC     BLE_INT2_IRQHandler
  PUBLIC     BTU_FIQ_IRQHandler
  PUBLIC     BTU_INT_IRQHandler
  PUBLIC     CTI_IRQ0_IRQHandler
  PUBLIC     T1_INT_IRQHandler
  PUBLIC     T2_INT_IRQHandler
  PUBLIC     T3_INT_IRQHandler
  PUBLIC     T4_INT_IRQHandler
  PUBLIC     SI_INT_IRQHandler
  PUBLIC     CTI_IRQ1_IRQHandler

  PUBLIC     _tx_initialize_low_level

  EXTERN     DispatchIRQ
  EXTERN     BleDispatchIRQ
  EXTERN     PendSV_Handler
  EXTERN     _tx_timer_interrupt
  EXTERN     _tx_thread_system_stack_ptr
  EXTERN     __vector_table

;External Interrupts

BLE_INT0_IRQHandler
  MOV     R0, #0
  LDR     R1, =BleDispatchIRQ
  BX      R1

BLE_INT1_IRQHandler
  MOV     R0, #1
  LDR     R1, =BleDispatchIRQ
  BX      R1

BLE_INT2_IRQHandler
  MOV     R0, #2
  LDR     R1, =BleDispatchIRQ
  BX      R1

BTU_FIQ_IRQHandler
BTU_INT_IRQHandler
CTI_IRQ0_IRQHandler
T1_INT_IRQHandler
T2_INT_IRQHandler
T3_INT_IRQHandler
T4_INT_IRQHandler
SI_INT_IRQHandler
CTI_IRQ1_IRQHandler
  LDR     R1, =DispatchIRQ
  BX      R1

SVC_Handler
  B     PendSV_Handler

SysTick_Handler
  LDR     R1, =_tx_timer_interrupt
  BX      R1

; ThreadX low level init
_tx_initialize_low_level
  LDR r0, =_tx_thread_system_stack_ptr    ; Build address of system stack pointer
  LDR r1, =__vector_table                 ; Pickup address of vector table
  LDR r1, [r1]                            ; Pickup reset stack pointer
  STR r1, [r0]                            ; Save system stack pointer

  MOV r0, #0xE000E000                     ; Build address of NVIC registers

  LDR r1, =0x00000000                     ; Rsrv, UsgF, BusF, MemM
  STR r1, [r0, #0xD18]                    ; Setup System Handlers 4-7 Priority Registers

  LDR r1, =0xFF000000                     ; SVCl, Rsrv, Rsrv, Rsrv
  STR r1, [r0, #0xD1C]                    ; Setup System Handlers 8-11 Priority Registers
                                          ; Note: SVC must be lowest priority, which is 0xFF
  LDR r1, =0x40FF0000                     ; SysT, PnSV, Rsrv, DbgM
  STR r1, [r0, #0xD20]                    ; Setup System Handlers 12-15 Priority Registers
                                          ; Note: PnSV must be lowest priority, which is 0xFF
  LDR r1, [r0, #0xD24]
  ORR r1, #0x70000
  STR r1, [r0, #0xD24]
;
;    /* Return to caller.  */
;    
  BX  lr 
        
        END
