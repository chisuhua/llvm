; RUN: llc -mtriple=ppu -verify-machineinstrs < %s \
; RUN:   | FileCheck %s -check-prefix=RV32I
; RUN: llc -mtriple=ppu -mattr=+c -verify-machineinstrs < %s \
; RUN:   | FileCheck %s -check-prefix=RV32C

define void @foo() {
;RV32I: .p2align 2
;RV32I: foo:
;RV32C: .p2align 1
;RV32C: foo:
entry:
  ret void
}
