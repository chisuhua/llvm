; NOTE: Assertions have been autogenerated by utils/update_test_checks.py
; RUN: opt -S -gvn < %s | FileCheck %s

define i32 @test(i32* %p, i32 %v) {
; CHECK-LABEL: @test(
; CHECK-NEXT:    [[LOAD:%.*]] = load i32, i32* [[P:%.*]]
; CHECK-NEXT:    [[C:%.*]] = icmp eq i32 [[LOAD]], [[V:%.*]]
; CHECK-NEXT:    call void @llvm.assume(i1 [[C]])
; CHECK-NEXT:    ret i32 [[LOAD]]
;
  %load = load i32, i32* %p
  %c = icmp eq i32 %load, %v
  call void @llvm.assume(i1 %c)
  ret i32 %load
}

define i32 @reverse(i32* %p, i32 %v) {
; CHECK-LABEL: @reverse(
; CHECK-NEXT:    [[LOAD:%.*]] = load i32, i32* [[P:%.*]]
; CHECK-NEXT:    [[C:%.*]] = icmp eq i32 [[LOAD]], [[V:%.*]]
; CHECK-NEXT:    call void @llvm.assume(i1 [[C]])
; CHECK-NEXT:    ret i32 [[V]]
;
  %load = load i32, i32* %p
  %c = icmp eq i32 %load, %v
  call void @llvm.assume(i1 %c)
  ret i32 %v
}

define i32 @test2(i32* %p, i32 %v) {
; CHECK-LABEL: @test2(
; CHECK-NEXT:    [[LOAD:%.*]] = load i32, i32* [[P:%.*]]
; CHECK-NEXT:    [[C:%.*]] = icmp eq i32 [[LOAD]], [[V:%.*]]
; CHECK-NEXT:    call void @llvm.assume(i1 [[C]])
; CHECK-NEXT:    ret i32 [[LOAD]]
;
  %load = load i32, i32* %p
  %c = icmp eq i32 %load, %v
  call void @llvm.assume(i1 %c)
  %load2 = load i32, i32* %p
  ret i32 %load2
}

define i32 @test3(i32* %p, i32 %v) {
; CHECK-LABEL: @test3(
; CHECK-NEXT:    [[LOAD:%.*]] = load i32, i32* [[P:%.*]]
; CHECK-NEXT:    [[C:%.*]] = icmp eq i32 [[LOAD]], [[V:%.*]]
; CHECK-NEXT:    call void @llvm.assume(i1 [[C]])
; CHECK-NEXT:    br i1 undef, label [[TAKEN:%.*]], label [[MERGE:%.*]]
; CHECK:       taken:
; CHECK-NEXT:    br label [[MERGE]]
; CHECK:       merge:
; CHECK-NEXT:    ret i32 [[V]]
;
  %load = load i32, i32* %p
  %c = icmp eq i32 %load, %v
  call void @llvm.assume(i1 %c)
  br i1 undef, label %taken, label %merge
taken:
  br label %merge
merge:
  ret i32 %load
}


declare void @llvm.assume(i1)
