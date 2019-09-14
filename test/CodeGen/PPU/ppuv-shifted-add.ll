; NOTE: Assertions have been autogenerated by utils/update_llc_test_checks.py
; RUN: llc -mtriple=ppu -mattr=+v -verify-machineinstrs < %s \
; RUN:   | FileCheck %s -check-prefix=RV32IV

declare i32 @llvm.ppu.setvl(i32)
declare <vscale x 1 x i32> @llvm.ppu.vadd(<vscale x 1 x i32>, <vscale x 1 x i32>, i32)
declare <vscale x 1 x i32> @llvm.ppu.vlw(i32*)
declare void @llvm.ppu.vsw(i32*, <vscale x 1 x i32>)

; A[0..n]
; Add pairs of elements together
; [A_0 | A_1 | A_2 | ... | A_n] => [A_0 + A_1 | A_1 + A_2 | A_i + A_(i+1) | ... A_n]
define void @shifted_add(i32 %n.0, i32* %A.0) {
; RV32IV-LABEL: shifted_add:
; RV32IV:       # %bb.0: # %entry
; RV32IV-NEXT:    vconfig 96
; RV32IV-NEXT:    addi a2, zero, 2
; RV32IV-NEXT:    bltu a0, a2, .LBB0_2
; RV32IV-NEXT:  .LBB0_1: # %loop
; RV32IV-NEXT:    # =>This Inner Loop Header: Depth=1
; RV32IV-NEXT:    addi a2, a0, -1
; RV32IV-NEXT:    vsetvl a2, a2
; RV32IV-NEXT:    addi a3, a1, 4
; RV32IV-NEXT:    vlw v0, 0(a3)
; RV32IV-NEXT:    vlw v1, 0(a1)
; RV32IV-NEXT:    vadd v0, v1, v0
; RV32IV-NEXT:    vsw v0, 0(a1)
; RV32IV-NEXT:    slli a3, a2, 2
; RV32IV-NEXT:    add a1, a1, a3
; RV32IV-NEXT:    sub a0, a0, a2
; RV32IV-NEXT:    bnez a2, .LBB0_1
; RV32IV-NEXT:  .LBB0_2: # %exit
; RV32IV-NEXT:    vconfig 1
; RV32IV-NEXT:	.cfi_def_cfa_offset 0
; RV32IV-NEXT:    ret
entry:
	%shouldStartLoop = icmp ule i32 %n.0, 1
	br i1 %shouldStartLoop, label %exit, label %loop
loop:
	%n = phi i32 [%n.0, %entry], [%n.rem, %loop]
	%A = phi i32* [%A.0, %entry], [%A.rem, %loop]

	%n.minusOne = sub i32 %n, 1

	%vl = call i32 @llvm.ppu.setvl(i32 %n.minusOne)
	%v.A = call <vscale x 1 x i32> @llvm.ppu.vlw(i32* %A)

	%A.shift = getelementptr i32, i32* %A, i32 1
	%v.A_shift = call <vscale x 1 x i32> @llvm.ppu.vlw(i32* %A.shift)

	%v.sum = call <vscale x 1 x i32> @llvm.ppu.vadd(<vscale x 1 x i32> %v.A, <vscale x 1 x i32> %v.A_shift, i32 %vl)
	call void @llvm.ppu.vsw(i32* %A, <vscale x 1 x i32> %v.sum)

	%n.rem = sub i32 %n, %vl
	%A.rem = getelementptr i32, i32* %A, i32 %vl

	%again = icmp ne i32 %vl, 0
	br i1 %again, label %loop, label %exit
exit:
	ret void
}
