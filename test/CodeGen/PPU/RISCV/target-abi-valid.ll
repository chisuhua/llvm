; RUN: llc -mtriple=ppu < %s \
; RUN:   | FileCheck -check-prefix=CHECK-IMP %s
; RUN: llc -mtriple=ppu -target-abi ilp32 < %s \
; RUN:   | FileCheck -check-prefix=CHECK-IMP %s
; RUN: llc -mtriple=ppu -mattr=+f -target-abi ilp32 < %s \
; RUN:   | FileCheck -check-prefix=CHECK-IMP %s
; RUN: llc -mtriple=ppu -mattr=+d -target-abi ilp32 < %s \
; RUN:   | FileCheck -check-prefix=CHECK-IMP %s
; RUN:   | FileCheck -check-prefix=CHECK-IMP %s
; RUN:   | FileCheck -check-prefix=CHECK-IMP %s
; RUN:   | FileCheck -check-prefix=CHECK-IMP %s
; RUN:   | FileCheck -check-prefix=CHECK-IMP %s
; RUN: llc -mtriple=ppu -mattr=+f -target-abi ilp32f < %s 2>&1 \
; RUN:   | FileCheck -check-prefix=CHECK-IMP %s
; RUN: llc -mtriple=ppu -mattr=+d -target-abi ilp32f < %s 2>&1 \
; RUN:   | FileCheck -check-prefix=CHECK-IMP %s
; RUN: llc -mtriple=ppu -mattr=+d -target-abi ilp32d < %s 2>&1 \
; RUN:   | FileCheck -check-prefix=CHECK-IMP %s
; RUN:   | FileCheck -check-prefix=CHECK-IMP %s
; RUN:   | FileCheck -check-prefix=CHECK-IMP %s
; RUN:   | FileCheck -check-prefix=CHECK-IMP %s

define void @nothing() nounwind {
; CHECK-IMP-LABEL: nothing:
; CHECK-IMP:       # %bb.0:
; CHECK-IMP-NEXT:    ret
  ret void
}

; RUN: not llc -mtriple=ppu -target-abi ilp32e < %s 2>&1 \
; RUN:   | FileCheck -check-prefix=CHECK-UNIMP %s

; CHECK-UNIMP: LLVM ERROR: Don't know how to lower this ABI
