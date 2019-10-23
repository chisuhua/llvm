; RUN: opt -S -reconvergecfg %s | FileCheck %s

define amdgpu_cs i32 @test1(i1 inreg %cc, i32 inreg %a, i32 inreg %b) {
; CHECK-LABEL: @test1(
; CHECK: A:
; CHECK:   br i1 %cc, label %flow, label %C
A:
  br i1 %cc, label %B, label %C

; CHECK: B:
; CHECK:   br label %D
B:
  %r.B = add i32 %a, %b
  br label %D

; CHECK: C:
; CHECK:   br label %flow
C:
  %r.C = mul i32 %a, %b
  br label %D

; CHECK: flow:
; CHECK:   %r2 = phi i32 [ undef, %A ], [ %r.C, %C ]
; CHECK:   %0 = phi i1 [ true, %A ], [ false, %C ]
; CHECK:   br i1 %0, label %B, label %D
;
; CHECK: D:
; CHECK:   %r1 = phi i32 [ %r2, %flow ], [ %r.B, %B ]
D:
  %r = phi i32 [ %r.B, %B ], [ %r.C, %C ]
  ret i32 %r
}
