; NOTE: Assertions have been autogenerated by utils/update_llc_test_checks.py
; RUN: llc < %s -O1 -mtriple=x86_64-unknown-unknown -o - | FileCheck %s

; We used to assert on widening the SMULFIX/UMULFIX/SMULFIXSAT node result,
; so primiary goal with the test is to see that we support legalization for
; such vectors.

declare <4 x i16> @llvm.smul.fix.v4i16(<4 x i16>, <4 x i16>, i32 immarg)
declare <4 x i16> @llvm.umul.fix.v4i16(<4 x i16>, <4 x i16>, i32 immarg)
declare <4 x i16> @llvm.smul.fix.sat.v4i16(<4 x i16>, <4 x i16>, i32 immarg)

define <4 x i16> @smulfix(<4 x i16> %a) {
; CHECK-LABEL: smulfix:
; CHECK:       # %bb.0:
; CHECK-NEXT:    movdqa {{.*#+}} xmm1 = <1,2,3,4,u,u,u,u>
; CHECK-NEXT:    movdqa %xmm0, %xmm2
; CHECK-NEXT:    pmullw %xmm1, %xmm2
; CHECK-NEXT:    psrlw $15, %xmm2
; CHECK-NEXT:    pmulhw %xmm1, %xmm0
; CHECK-NEXT:    psllw $1, %xmm0
; CHECK-NEXT:    por %xmm2, %xmm0
; CHECK-NEXT:    retq
  %t = call <4 x i16> @llvm.smul.fix.v4i16(<4 x i16> <i16 1, i16 2, i16 3, i16 4>, <4 x i16> %a, i32 15)
  ret <4 x i16> %t
}

define <4 x i16> @umulfix(<4 x i16> %a) {
; CHECK-LABEL: umulfix:
; CHECK:       # %bb.0:
; CHECK-NEXT:    movdqa {{.*#+}} xmm1 = <1,2,3,4,u,u,u,u>
; CHECK-NEXT:    movdqa %xmm0, %xmm2
; CHECK-NEXT:    pmullw %xmm1, %xmm2
; CHECK-NEXT:    psrlw $15, %xmm2
; CHECK-NEXT:    pmulhuw %xmm1, %xmm0
; CHECK-NEXT:    psllw $1, %xmm0
; CHECK-NEXT:    por %xmm2, %xmm0
; CHECK-NEXT:    retq
  %t = call <4 x i16> @llvm.umul.fix.v4i16(<4 x i16> <i16 1, i16 2, i16 3, i16 4>, <4 x i16> %a, i32 15)
  ret <4 x i16> %t
}

define <4 x i16> @smulfixsat(<4 x i16> %a) {
; CHECK-LABEL: smulfixsat:
; CHECK:       # %bb.0:
; CHECK-NEXT:    pextrw $2, %xmm0, %eax
; CHECK-NEXT:    cwtl
; CHECK-NEXT:    leal (%rax,%rax,2), %ecx
; CHECK-NEXT:    movl %ecx, %edx
; CHECK-NEXT:    shrl $16, %edx
; CHECK-NEXT:    shldw $1, %cx, %dx
; CHECK-NEXT:    sarl $16, %ecx
; CHECK-NEXT:    cmpl $16383, %ecx # imm = 0x3FFF
; CHECK-NEXT:    movl $32767, %r8d # imm = 0x7FFF
; CHECK-NEXT:    cmovgl %r8d, %edx
; CHECK-NEXT:    cmpl $-16384, %ecx # imm = 0xC000
; CHECK-NEXT:    movl $32768, %ecx # imm = 0x8000
; CHECK-NEXT:    cmovll %ecx, %edx
; CHECK-NEXT:    pextrw $1, %xmm0, %esi
; CHECK-NEXT:    movswl %si, %edi
; CHECK-NEXT:    movl %edi, %eax
; CHECK-NEXT:    shrl $15, %eax
; CHECK-NEXT:    leal (%rdi,%rdi), %esi
; CHECK-NEXT:    shrdw $15, %ax, %si
; CHECK-NEXT:    sarl $15, %edi
; CHECK-NEXT:    cmpl $16383, %edi # imm = 0x3FFF
; CHECK-NEXT:    cmovgl %r8d, %esi
; CHECK-NEXT:    cmpl $-16384, %edi # imm = 0xC000
; CHECK-NEXT:    cmovll %ecx, %esi
; CHECK-NEXT:    movd %xmm0, %eax
; CHECK-NEXT:    cwtl
; CHECK-NEXT:    movl %eax, %edi
; CHECK-NEXT:    shrl $16, %edi
; CHECK-NEXT:    shldw $1, %ax, %di
; CHECK-NEXT:    sarl $16, %eax
; CHECK-NEXT:    cmpl $16383, %eax # imm = 0x3FFF
; CHECK-NEXT:    cmovgl %r8d, %edi
; CHECK-NEXT:    cmpl $-16384, %eax # imm = 0xC000
; CHECK-NEXT:    cmovll %ecx, %edi
; CHECK-NEXT:    pxor %xmm1, %xmm1
; CHECK-NEXT:    pinsrw $0, %edi, %xmm1
; CHECK-NEXT:    pinsrw $1, %esi, %xmm1
; CHECK-NEXT:    pinsrw $2, %edx, %xmm1
; CHECK-NEXT:    pextrw $3, %xmm0, %eax
; CHECK-NEXT:    cwtl
; CHECK-NEXT:    movl %eax, %edx
; CHECK-NEXT:    shrl $14, %edx
; CHECK-NEXT:    leal (,%rax,4), %esi
; CHECK-NEXT:    shrdw $15, %dx, %si
; CHECK-NEXT:    sarl $14, %eax
; CHECK-NEXT:    cmpl $16383, %eax # imm = 0x3FFF
; CHECK-NEXT:    cmovgl %r8d, %esi
; CHECK-NEXT:    cmpl $-16384, %eax # imm = 0xC000
; CHECK-NEXT:    cmovll %ecx, %esi
; CHECK-NEXT:    pinsrw $3, %esi, %xmm1
; CHECK-NEXT:    movdqa %xmm1, %xmm0
; CHECK-NEXT:    retq
  %t = call <4 x i16> @llvm.smul.fix.sat.v4i16(<4 x i16> <i16 1, i16 2, i16 3, i16 4>, <4 x i16> %a, i32 15)
  ret <4 x i16> %t
}


