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

  EXTERN     DispatchIRQ
  EXTERN     BleDispatchIRQ
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
        LDR     R0, =DispatchIRQ
        BX      R0
        
        END
