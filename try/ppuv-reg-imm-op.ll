; NOTE: Assertions have been autogenerated by utils/update_llc_test_checks.py
; RUN: llc -mtriple=ppu -mattr=+v -verify-machineinstrs < %s \
; RUN:   | FileCheck %s -check-prefix=RV32IV

declare i32 @llvm.ppu.setvl(i32)
declare <vscale x 1 x i32> @llvm.ppu.vadd(<vscale x 1 x i32>, <vscale x 1 x i32>, i32)
declare <vscale x 1 x i32> @llvm.ppu.vlw(i32*, i32)
declare void @llvm.ppu.vsw(i32*, <vscale x 1 x i32>, i32)

; R[0..n] = A[0..n]
define void @foo(i32 %n.0, i32* %R.0, i32* %A.0) {
; RV32IV-LABEL: foo:
; RV32IV:       # %bb.0: # %entry
; RV32IV-NEXT:    vconfig 96
; RV32IV-NEXT:  .LBB0_1: # %loop
; RV32IV-NEXT:    # =>This Inner Loop Header: Depth=1
; RV32IV-NEXT:    vsetvl a3, a0
; RV32IV-NEXT:    vlw v0, 0(a2)
; RV32IV-NEXT:    vaddi v0, v0, 2
; RV32IV-NEXT:    vsw v0, 0(a1)
; RV32IV-NEXT:    slli a4, a3, 2
; RV32IV-NEXT:    add a1, a1, a4
; RV32IV-NEXT:    add a2, a2, a4
; RV32IV-NEXT:    sub a0, a0, a3
; RV32IV-NEXT:    bnez a3, .LBB0_1
; RV32IV-NEXT:  # %bb.2: # %exit
; RV32IV-NEXT:    vconfig 1
; RV32IV-NEXT:    ret
entry:
	br label %loop
loop:
	%n = phi i32 [%n.0, %entry], [%n.rem, %loop]
	%A = phi i32* [%A.0, %entry], [%A.rem, %loop]
	%R = phi i32* [%R.0, %entry], [%R.rem, %loop]
	%vl = call i32 @llvm.ppu.setvl(i32 %n)
	%v.A = call <vscale x 1 x i32> @llvm.ppu.vlw(i32* %A, i32 %vl)

	%vimm.inserted = insertelement <vscale x 1 x i32> undef, i32 2, i32 0
	%vimm.splatted = shufflevector <vscale x 1 x i32> %vimm.inserted, <vscale x 1 x i32> undef, <vscale x 1 x i32> zeroinitializer

	%v.R1 = call <vscale x 1 x i32> @llvm.ppu.vadd(<vscale x 1 x i32> %v.A, <vscale x 1 x i32> %vimm.splatted, i32 %vl)

	call void @llvm.ppu.vsw(i32* %R, <vscale x 1 x i32> %v.R1, i32 %vl)
	%n.rem = sub i32 %n, %vl
	%A.rem = getelementptr i32, i32* %A, i32 %vl
	%R.rem = getelementptr i32, i32* %R, i32 %vl
	%again = icmp ne i32 %vl, 0
	br i1 %again, label %loop, label %exit
exit:
	ret void
}
