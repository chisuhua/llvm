; RUN: llc -mtriple=ppu -verify-machineinstrs < %s \
; RUN:   | FileCheck -check-prefix=RV32I %s
; RUN: llc -mtriple=ppu -mattr=+f -verify-machineinstrs < %s \
; RUN:   | FileCheck -check-prefix=RV32I %s
; RUN: llc -mtriple=ppu -mattr=+d -verify-machineinstrs < %s \
; RUN:   | FileCheck -check-prefix=RV32I %s
; RUN: llc -mtriple=ppu -mattr=+f -target-abi ilp32f -verify-machineinstrs < %s \
; RUN:   | FileCheck -check-prefix=RV32IF %s
; RUN: llc -mtriple=ppu -mattr=+d -target-abi ilp32d -verify-machineinstrs < %s \
; RUN:   | FileCheck -check-prefix=RV32ID %s


define void @testcase() nounwind {
; RV32I-LABEL: testcase:
; RV32I:      sw s1, {{[0-9]+}}(sp)
; RV32I-NEXT: sw s2, {{[0-9]+}}(sp)
; RV32I-NOT:  fsw fs0, {{[0-9]+}}(sp)
; RV32I-NOT:  fsd fs0, {{[0-9]+}}(sp)
;
;
; RV32IF-LABEL: testcase:
; RV32IF:      sw s1, {{[0-9]+}}(sp)
; RV32IF-NEXT: sw s2, {{[0-9]+}}(sp)
; RV32IF-NEXT: fsw fs0, {{[0-9]+}}(sp)
; RV32IF-NEXT: fsw fs1, {{[0-9]+}}(sp)
;
;
; RV32ID-LABEL: testcase:
; RV32ID:      sw s1, {{[0-9]+}}(sp)
; RV32ID-NEXT: sw s2, {{[0-9]+}}(sp)
; RV32ID-NEXT: fsd fs0, {{[0-9]+}}(sp)
; RV32ID-NEXT: fsd fs1, {{[0-9]+}}(sp)
;
  tail call void asm sideeffect "", "~{f8},~{f9},~{x9},~{x18}"()
  ret void
}
