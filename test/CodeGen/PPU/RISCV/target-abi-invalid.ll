; RUN: llc -mtriple=ppu -target-abi foo < %s 2>&1 \
; RUN:   | FileCheck -check-prefix=RV32I-FOO %s
; RUN: llc -mtriple=ppu -mattr=+f -target-abi ilp32foof < %s 2>&1 \
; RUN:   | FileCheck -check-prefix=RV32IF-ILP32FOOF %s

; RV32I-FOO: 'foo' is not a recognized ABI for this target (ignoring target-abi)
; RV32IF-ILP32FOOF: 'ilp32foof' is not a recognized ABI for this target (ignoring target-abi)



; RUN: llc -mtriple=ppu -target-abi lp64 < %s 2>&1 \
; RUN:   | FileCheck -check-prefix=RV32I-LP64 %s
; RUN: llc -mtriple=ppu -mattr=+f -target-abi lp64f < %s 2>&1 \
; RUN:   | FileCheck -check-prefix=RV32IF-LP64F %s
; RUN: llc -mtriple=ppu -mattr=+d -target-abi lp64d < %s 2>&1 \
; RUN:   | FileCheck -check-prefix=RV32IFD-LP64D %s

; RV32I-LP64: 64-bit ABIs are not supported for 32-bit targets (ignoring target-abi)
; RV32IF-LP64F: 64-bit ABIs are not supported for 32-bit targets (ignoring target-abi)
; RV32IFD-LP64D: 64-bit ABIs are not supported for 32-bit targets (ignoring target-abi)

; RUN: llc -mtriple=ppu -target-abi ilp32f < %s 2>&1 \
; RUN:   | FileCheck -check-prefix=RV32I-ILP32F %s

; RV32I-ILP32F: Hard-float 'f' ABI can't be used for a target that doesn't support the F instruction set extension (ignoring target-abi)

; RUN: llc -mtriple=ppu -target-abi ilp32d < %s 2>&1 \
; RUN:   | FileCheck -check-prefix=RV32I-ILP32D %s
; RUN: llc -mtriple=ppu -mattr=+f -target-abi ilp32d < %s 2>&1 \
; RUN:   | FileCheck -check-prefix=RV32IF-ILP32D %s

; RV32I-ILP32D: Hard-float 'd' ABI can't be used for a target that doesn't support the D instruction set extension (ignoring target-abi)
; RV32IF-ILP32D: Hard-float 'd' ABI can't be used for a target that doesn't support the D instruction set extension (ignoring target-abi)

define void @nothing() nounwind {
  ret void
}
