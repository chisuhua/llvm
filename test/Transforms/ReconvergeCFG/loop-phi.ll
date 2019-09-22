; RUN: opt -S -reconvergecfg %s | FileCheck %s

;      A
;      |
; /----+----\
; |    B    |
; | /--+--\ |
; | |     | |
; | C     D |
; \-+     +-/
;   \--+--/
;      E
define i32 @test1(i1 %cc, i32 %a, i32 %b) {
; CHECK-LABEL: @test1(
; CHECK: A:
; CHECK:   br label %B
A:
  br label %B

; CHECK: B:
; TODO: the following phi is redundant
; CHECK:   %r.E2 = phi i32 [ undef, %A ], [ %r.B, %C ], [ %r.D, %D ]
; CHECK:   %r.B = phi i32 [ 0, %A ], [ %r.B, %C ], [ %r.D, %D ]
; CHECK:   br i1 %cc.B, label %flow, label %D
B:
  %ctr = phi i32 [ 0, %A ], [ %ctr.upd, %C ], [ %ctr.upd, %D ]
  %r.B = phi i32 [ 0, %A ], [ %r.B, %C ], [ %r.D, %D ]
  %ctr.upd = add i32 %ctr, 1
  %cc.B = icmp ugt i32 %ctr, %a
  %cc.out = icmp ugt i32 %ctr, %b
  br i1 %cc.B, label %C, label %D

; CHECK: C:
; CHECK:   br i1 %cc.out, label %E, label %B
C:
  br i1 %cc.out, label %E, label %B

; CHECK: D:
; CHECK:   br i1 %cc.out, label %flow, label %B
D:
  %r.D = add i32 %r.B, 1
  br i1 %cc.out, label %E, label %B

; CHECK: flow:
; CHECK:   %r.E1 = phi i32 [ %r.E2, %B ], [ %r.D, %D ]
; CHECK:   %0 = phi i1 [ true, %B ], [ false, %D ]
; CHECK:   br i1 %0, label %C, label %E
;
; CHECK: E:
; CHECK:   %r.E34 = phi i32 [ %r.E1, %flow ], [ %r.B, %C ]
E:
  %r.E = phi i32 [ %r.B, %C ], [ %r.D, %D ]
  ret i32 %r.E
}

;    A
;    +--\
; /--+  |
; |  B  |
; +--+  |
; |  |  |
; |  C  D
; \--+  |
;    +--/
;    X
define i32 @test2(i1 %cc, i1 %cc2, i1 %cc3) {
; CHECK-LABEL: @test2(
; CHECK: A:
; CHECK:   br i1 %cc, label %flow, label %D
A:
  br i1 %cc, label %B, label %D

; CHECK: B:
; CHECK:   %r.B1 = phi i32 [ 0, %flow ], [ %r.C, %C ], [ %r.B1, %B ]
; CHECK:   br i1 %cc2, label %B, label %C
B:
  %r.B = phi i32 [ 0, %A ], [ %r.B, %B ], [ %r.C, %C ]
  br i1 %cc2, label %B, label %C

; CHECK: C:
; CHECK:   br i1 %cc3, label %B, label %X
C:
  %r.C = add i32 %r.B, 1
  br i1 %cc3, label %B, label %X

; CHECK: D:
; CHECK:   br label %flow
D:
  br label %X

; CHECK: flow:
; CHECK:   %r3 = phi i32 [ undef, %A ], [ 42, %D ]
; CHECK:   %0 = phi i1 [ true, %A ], [ false, %D ]
; CHECK:   br i1 %0, label %B, label %X
;
; CHECK: X:
; CHECK:   %r2 = phi i32 [ %r3, %flow ], [ %r.C, %C ]
X:
  %r = phi i32 [ %r.C, %C ], [ 42, %D ]
  ret i32 %r
}
