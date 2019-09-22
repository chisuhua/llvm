; RUN: opt -S -reconvergecfg %s | FileCheck %s

; First test:
;
;    A
;    |
;  /-+
;  | B
;  | +-\
;  | | |
;  | C D
;  \-+ |
;    +-/
;    E
;
; A single flow block after the loop should have both D and E as successors.
; Two variants, with the order of C and D reversed in the terminator of B.
define void @test1(i1 %cc, i32 %count) {
; CHECK-LABEL: @test1(
; CHECK: A:
; CHECK:   br label %B
A:
  br label %B

; CHECK: B:
; CHECK:   br i1 %cc, label %C, label %flow
B:
  %ctr.B = phi i32 [ 0, %A ], [ %ctr.C, %C ]
  br i1 %cc, label %C, label %D

; CHECK: C:
; CHECK:   br i1 %cc.loop, label %B, label %flow
C:
  %ctr.C = add i32 %ctr.B, 1
  %cc.loop = icmp ule i32 %ctr.C, %count
  br i1 %cc.loop, label %B, label %E

; CHECK: flow:
; CHECK:   %0 = phi i1 [ true, %B ], [ false, %C ]
; CHECK:   br i1 %0, label %D, label %E
;
; CHECK: D:
; CHECK:   br label %E
D:
  br label %E

E:
  ret void
}

define void @test1_reverse(i1 %cc, i32 %count) {
; CHECK-LABEL: @test1_reverse(
; CHECK: A:
; CHECK:   br label %B
A:
  br label %B

; CHECK: B:
; CHECK:   br i1 %cc, label %flow, label %C
B:
  %ctr.B = phi i32 [ 0, %A ], [ %ctr.C, %C ]
  br i1 %cc, label %D, label %C

; CHECK: C:
; CHECK:   br i1 %cc.loop, label %B, label %flow
C:
  %ctr.C = add i32 %ctr.B, 1
  %cc.loop = icmp ule i32 %ctr.C, %count
  br i1 %cc.loop, label %B, label %E

; CHECK: flow:
; CHECK:   %0 = phi i1 [ true, %B ], [ false, %C ]
; CHECK:   br i1 %0, label %D, label %E
;
; CHECK: D:
; CHECK:   br label %E
D:
  br label %E

E:
  ret void
}

; Second test:
;
;   A
;   |
;   +----
;   B    \
;   |    |
; /-+    |
; | C    |
; | +--\ |
; | |  | |
; | D  E |
; \-+  +-/
;   +--/
;   F
;
define void @test2(i32 %cmp0, i32 %count) {
; CHECK-LABEL: @test2(
; CHECK: A:
; CHECK:   br label %B
A:
  br label %B

; CHECK: B:
; CHECK:   br label %C
B:
  %ctr.B = phi i32 [ 0, %A ], [ %ctr.E, %E ]
  br label %C

; CHECK: C:
; CHECK:   br i1 %cc.C, label %D, label %flow
C:
  %ctr.C = phi i32 [ %ctr.B, %B ], [ %ctr.D, %D ]
  %cc.C = icmp eq i32 %cmp0, %ctr.C
  br i1 %cc.C, label %D, label %E

; CHECK: D:
; CHECK:   br i1 %cc.D, label %C, label %flow
D:
  %ctr.D = add i32 %ctr.C, 1
  %cc.D = icmp slt i32 %ctr.D, %count
  br i1 %cc.D, label %C, label %F

; CHECK: flow:
; CHECK:   %0 = phi i1 [ true, %C ], [ false, %D ]
; CHECK:   br i1 %0, label %E, label %F
;
; CHECK: E:
; CHECK:   br i1 %cc.E, label %B, label %F
E:
  %ctr.E = add i32 %ctr.C, 1
  %cc.E = icmp slt i32 %ctr.E, %count
  br i1 %cc.E, label %B, label %F

F:
  ret void
}
