
; Make sure that boolean immediates are properly (zero) extended.
; CHECK: TEST 42 + 1 - .


define i32 @foo() #0 {
entry:
  tail call void asm sideeffect "#TEST 42 + ${0:c} - .\0A\09", "i,~{dirflag},~{fpsr},~{flags}"(i1 true) #0
  ret i32 1
}

attributes #0 = { nounwind }
