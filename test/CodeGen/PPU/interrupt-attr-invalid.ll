; RUN: not llc -mtriple ppu-unknown-elf -o - %s \
; RUN: 2>&1 | FileCheck %s
; RUN: 2>&1 | FileCheck %s

; CHECK: LLVM ERROR: Function interrupt attribute argument not supported!
define void @isr_user() #0 {
  ret void
}

attributes #0 = { "interrupt"="foo" }
