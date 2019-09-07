; NOTE: Assertions have been autogenerated by utils/update_llc_test_checks.py
; RUN: llc -mtriple=ppu -global-isel -stop-after=irtranslator -verify-machineinstrs < %s \
; RUN:   | FileCheck -check-prefix=RV32I %s

define void @foo() {
  ; RV32I-LABEL: name: foo
  ; RV32I: bb.1.entry:
  ; RV32I-NEXT:   PseudoRET

entry:
  ret void
}
