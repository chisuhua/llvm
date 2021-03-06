; NOTE: Assertions have been autogenerated by utils/update_llc_test_checks.py
; RUN: llc -mtriple=ppu -mattr=+v -verify-machineinstrs < %s \
; RUN:   | FileCheck %s -check-prefix=RV32IV

declare i32 @llvm.ppu.setvl(i32)
declare <vscale x 1 x i32> @llvm.ppu.vlw(i32*, i32)
declare void @llvm.ppu.vsw(i32*, <vscale x 1 x i32>, i32)
declare <vscale x 1 x i32> @llvm.ppu.vadd(<vscale x 1 x i32>, <vscale x 1 x i32>, i32)

; R[0..n] = A[0..n]
define void @foo(i32 %n, i32* %A, i32* %R, i1 %cond) {
; RV32IV-LABEL: foo:
; RV32IV:       # %bb.0: # %bb1
; RV32IV:    vconfig 96
; RV32IV-NEXT:    vsetvl a0, a0
; RV32IV-NEXT:    vlw v0, 0(a1)
; RV32IV-NEXT:    vadd v2, v0, v0
; RV32IV-NEXT:    andi a1, a3, 1
; RV32IV-NEXT:    beqz a1, .LBB0_2
; RV32IV-NEXT:  # %bb.1: # %bb2
; RV32IV-NEXT:    csrr a1, vl
; RV32IV-NEXT:    csrr a3, vlmax
; RV32IV-NEXT:    vsetvl zero, a3
; RV32IV-NEXT:    vaddi v1, v2, 0
; RV32IV-NEXT:    j .LBB0_3
; RV32IV-NEXT:  .LBB0_2:
; RV32IV-NEXT:    csrr a1, vl
; RV32IV-NEXT:    csrr a3, vlmax
; RV32IV-NEXT:    vsetvl zero, a3
; RV32IV-NEXT:    vaddi v1, v0, 0
; RV32IV-NEXT:    vsetvl zero, a1
; RV32IV-NEXT:    csrr a1, vl
; RV32IV-NEXT:    csrr a3, vlmax
; RV32IV-NEXT:    vsetvl zero, a3
; RV32IV-NEXT:    vaddi v0, v2, 0
; RV32IV-NEXT:  .LBB0_3: # %bb3
; RV32IV-NEXT:    vsetvl zero, a1
; RV32IV-NEXT:    vsetvl zero, a0
; RV32IV-NEXT:    vsw v1, 0(a2)
; RV32IV-NEXT:    vsw v0, 0(a2)
; RV32IV-NEXT:    vconfig 1
; RV32IV-NEXT:  .cfi_def_cfa_offset 0
; RV32IV-NEXT:    ret
bb1:
	%vl = call i32 @llvm.ppu.setvl(i32 %n)
	%v1 = call <vscale x 1 x i32> @llvm.ppu.vlw(i32* %A, i32 %vl)
	%v2 = call <vscale x 1 x i32> @llvm.ppu.vlw(i32* %A, i32 %vl)
	%sum = call <vscale x 1 x i32> @llvm.ppu.vadd(<vscale x 1 x i32> %v1, <vscale x 1 x i32> %v2,i32 %vl)
	br i1 %cond, label %bb2, label %bb3
bb2:
	br label %bb3
bb3:
	%v3 = phi <vscale x 1 x i32> [%v1, %bb1], [%sum, %bb2]
	%v4 = phi <vscale x 1 x i32> [%sum, %bb1], [%v2, %bb2]
	call void @llvm.ppu.vsw(i32* %R, <vscale x 1 x i32> %v3, i32 %vl)
	call void @llvm.ppu.vsw(i32* %R, <vscale x 1 x i32> %v4, i32 %vl)
	ret void
}
