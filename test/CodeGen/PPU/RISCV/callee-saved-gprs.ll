; RUN: llc -mtriple=ppu -verify-machineinstrs < %s \
; RUN:   | FileCheck %s -check-prefix=RV32I
; RUN: llc -mtriple=ppu -mattr=+f -target-abi ilp32f -verify-machineinstrs < %s \
; RUN:   | FileCheck %s -check-prefix=RV32I
; RUN: llc -mtriple=ppu -mattr=+d -target-abi ilp32f -verify-machineinstrs < %s \
; RUN:   | FileCheck %s -check-prefix=RV32I
; RUN: llc -mtriple=ppu -mattr=+d -target-abi ilp32d -verify-machineinstrs < %s \
; RUN:   | FileCheck %s -check-prefix=RV32I
; RUN: llc -mtriple=ppu -verify-machineinstrs -frame-pointer=all < %s \
; RUN:   | FileCheck %s -check-prefix=RV32I-WITH-FP

@var = global [32 x i32] zeroinitializer

; This function tests that RISCVRegisterInfo::getCalleeSavedRegs returns
; something appropriate.

define void @callee() nounwind {
; RV32I-LABEL: callee:
; RV32I:       # %bb.0:
; RV32I-NEXT:    addi sp, sp, -80
; RV32I-NEXT:    sw s0, 76(sp)
; RV32I-NEXT:    sw s1, 72(sp)
; RV32I-NEXT:    sw s2, 68(sp)
; RV32I-NEXT:    sw s3, 64(sp)
; RV32I-NEXT:    sw s4, 60(sp)
; RV32I-NEXT:    sw s5, 56(sp)
; RV32I-NEXT:    sw s6, 52(sp)
; RV32I-NEXT:    sw s7, 48(sp)
; RV32I-NEXT:    sw s8, 44(sp)
; RV32I-NEXT:    sw s9, 40(sp)
; RV32I-NEXT:    sw s10, 36(sp)
; RV32I-NEXT:    sw s11, 32(sp)
; RV32I-NEXT:    lui a0, %hi(var)
; RV32I-NEXT:    addi a1, a0, %lo(var)
;
; RV32I-WITH-FP-LABEL: callee:
; RV32I-WITH-FP:       # %bb.0:
; RV32I-WITH-FP-NEXT:    addi sp, sp, -80
; RV32I-WITH-FP-NEXT:    sw ra, 76(sp)
; RV32I-WITH-FP-NEXT:    sw s0, 72(sp)
; RV32I-WITH-FP-NEXT:    sw s1, 68(sp)
; RV32I-WITH-FP-NEXT:    sw s2, 64(sp)
; RV32I-WITH-FP-NEXT:    sw s3, 60(sp)
; RV32I-WITH-FP-NEXT:    sw s4, 56(sp)
; RV32I-WITH-FP-NEXT:    sw s5, 52(sp)
; RV32I-WITH-FP-NEXT:    sw s6, 48(sp)
; RV32I-WITH-FP-NEXT:    sw s7, 44(sp)
; RV32I-WITH-FP-NEXT:    sw s8, 40(sp)
; RV32I-WITH-FP-NEXT:    sw s9, 36(sp)
; RV32I-WITH-FP-NEXT:    sw s10, 32(sp)
; RV32I-WITH-FP-NEXT:    sw s11, 28(sp)
; RV32I-WITH-FP-NEXT:    addi s0, sp, 80
; RV32I-WITH-FP-NEXT:    lui a0, %hi(var)
; RV32I-WITH-FP-NEXT:    addi a1, a0, %lo(var)
;
;
  %val = load [32 x i32], [32 x i32]* @var
  store volatile [32 x i32] %val, [32 x i32]* @var
  ret void
}

; This function tests that RISCVRegisterInfo::getCallPreservedMask returns
; something appropriate.

define void @caller() nounwind {
; RV32I-LABEL: caller:
; RV32I:         lui a0, %hi(var)
; RV32I-NEXT:    addi s1, a0, %lo(var)
; RV32I:         sw a0, 8(sp)
; RV32I-NEXT:    lw s2, 84(s1)
; RV32I-NEXT:    lw s3, 88(s1)
; RV32I-NEXT:    lw s4, 92(s1)
; RV32I-NEXT:    lw s5, 96(s1)
; RV32I-NEXT:    lw s6, 100(s1)
; RV32I-NEXT:    lw s7, 104(s1)
; RV32I-NEXT:    lw s8, 108(s1)
; RV32I-NEXT:    lw s9, 112(s1)
; RV32I-NEXT:    lw s10, 116(s1)
; RV32I-NEXT:    lw s11, 120(s1)
; RV32I-NEXT:    lw s0, 124(s1)
; RV32I-NEXT:    call callee
; RV32I-NEXT:    sw s0, 124(s1)
; RV32I-NEXT:    sw s11, 120(s1)
; RV32I-NEXT:    sw s10, 116(s1)
; RV32I-NEXT:    sw s9, 112(s1)
; RV32I-NEXT:    sw s8, 108(s1)
; RV32I-NEXT:    sw s7, 104(s1)
; RV32I-NEXT:    sw s6, 100(s1)
; RV32I-NEXT:    sw s5, 96(s1)
; RV32I-NEXT:    sw s4, 92(s1)
; RV32I-NEXT:    sw s3, 88(s1)
; RV32I-NEXT:    sw s2, 84(s1)
; RV32I-NEXT:    lw a0, 8(sp)
;
; RV32I-WITH-FP-LABEL: caller:
; RV32I-WITH-FP:         addi s0, sp, 144
; RV32I-WITH-FP-NEXT:    lui a0, %hi(var)
; RV32I-WITH-FP-NEXT:    addi s1, a0, %lo(var)
; RV32I-WITH-FP:         sw a0, -140(s0)
; RV32I-WITH-FP-NEXT:    lw s5, 88(s1)
; RV32I-WITH-FP-NEXT:    lw s6, 92(s1)
; RV32I-WITH-FP-NEXT:    lw s7, 96(s1)
; RV32I-WITH-FP-NEXT:    lw s8, 100(s1)
; RV32I-WITH-FP-NEXT:    lw s9, 104(s1)
; RV32I-WITH-FP-NEXT:    lw s10, 108(s1)
; RV32I-WITH-FP-NEXT:    lw s11, 112(s1)
; RV32I-WITH-FP-NEXT:    lw s2, 116(s1)
; RV32I-WITH-FP-NEXT:    lw s3, 120(s1)
; RV32I-WITH-FP-NEXT:    lw s4, 124(s1)
; RV32I-WITH-FP-NEXT:    call callee
; RV32I-WITH-FP-NEXT:    sw s4, 124(s1)
; RV32I-WITH-FP-NEXT:    sw s3, 120(s1)
; RV32I-WITH-FP-NEXT:    sw s2, 116(s1)
; RV32I-WITH-FP-NEXT:    sw s11, 112(s1)
; RV32I-WITH-FP-NEXT:    sw s10, 108(s1)
; RV32I-WITH-FP-NEXT:    sw s9, 104(s1)
; RV32I-WITH-FP-NEXT:    sw s8, 100(s1)
; RV32I-WITH-FP-NEXT:    sw s7, 96(s1)
; RV32I-WITH-FP-NEXT:    sw s6, 92(s1)
; RV32I-WITH-FP-NEXT:    sw s5, 88(s1)
; RV32I-WITH-FP-NEXT:    lw a0, -140(s0)
;
;
  %val = load [32 x i32], [32 x i32]* @var
  call void @callee()
  store volatile [32 x i32] %val, [32 x i32]* @var
  ret void
}
