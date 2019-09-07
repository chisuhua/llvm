# RUN: llvm-mc -triple=ppu -target-abi foo < %s 2>&1 \
# RUN:   | FileCheck -check-prefix=RV32I-FOO %s
# RUN: llvm-mc -triple=ppu -mattr=+f -target-abi ilp32foof < %s 2>&1 \
# RUN:   | FileCheck -check-prefix=RV32IF-ILP32FOOF %s

# RV32I-FOO: 'foo' is not a recognized ABI for this target (ignoring target-abi)
# RV32IF-ILP32FOOF: 'ilp32foof' is not a recognized ABI for this target (ignoring target-abi)



# RUN: llvm-mc -triple=ppu -target-abi lp64 < %s 2>&1 \
# RUN:   | FileCheck -check-prefix=RV32I-LP64 %s
# RUN: llvm-mc -triple=ppu -mattr=+f -target-abi lp64f < %s 2>&1 \
# RUN:   | FileCheck -check-prefix=RV32IF-LP64F %s
# RUN: llvm-mc -triple=ppu -mattr=+d -target-abi lp64d < %s 2>&1 \
# RUN:   | FileCheck -check-prefix=RV32IFD-LP64D %s
# RUN: llvm-mc -triple=ppu -mattr=+e -target-abi lp64 < %s 2>&1 \
# RUN:   | FileCheck -check-prefix=RV32E-LP64 %s
# RUN: llvm-mc -triple=ppu -mattr=+e,+f -target-abi lp64f < %s 2>&1 \
# RUN:   | FileCheck -check-prefix=RV32EF-LP64F %s
# RUN: llvm-mc -triple=ppu -mattr=+e,+d -target-abi lp64f < %s 2>&1 \
# RUN:   | FileCheck -check-prefix=RV32EFD-LP64D %s

# RV32I-LP64: 64-bit ABIs are not supported for 32-bit targets (ignoring target-abi)
# RV32IF-LP64F: 64-bit ABIs are not supported for 32-bit targets (ignoring target-abi)
# RV32IFD-LP64D: 64-bit ABIs are not supported for 32-bit targets (ignoring target-abi)
# RV32E-LP64: 64-bit ABIs are not supported for 32-bit targets (ignoring target-abi)
# RV32EF-LP64F: 64-bit ABIs are not supported for 32-bit targets (ignoring target-abi)
# RV32EFD-LP64D: 64-bit ABIs are not supported for 32-bit targets (ignoring target-abi)

# RUN: llvm-mc -triple=ppu -target-abi ilp32f < %s 2>&1 \
# RUN:   | FileCheck -check-prefix=RV32I-ILP32F %s

# RV32I-ILP32F: Hard-float 'f' ABI can't be used for a target that doesn't support the F instruction set extension (ignoring target-abi)

# RUN: llvm-mc -triple=ppu -target-abi ilp32d < %s 2>&1 \
# RUN:   | FileCheck -check-prefix=RV32I-ILP32D %s
# RUN: llvm-mc -triple=ppu -mattr=+f -target-abi ilp32d < %s 2>&1 \
# RUN:   | FileCheck -check-prefix=RV32IF-ILP32D %s

# RV32I-ILP32D: Hard-float 'd' ABI can't be used for a target that doesn't support the D instruction set extension (ignoring target-abi)
# RV32IF-ILP32D: Hard-float 'd' ABI can't be used for a target that doesn't support the D instruction set extension (ignoring target-abi)

# RUN: llvm-mc -triple=ppu -mattr=+e -target-abi ilp32 < %s 2>&1 \
# RUN:   | FileCheck -check-prefix=RV32EF-ILP32F %s
# RUN: llvm-mc -triple=ppu -mattr=+e,+f -target-abi ilp32f < %s 2>&1 \
# RUN:   | FileCheck -check-prefix=RV32EF-ILP32F %s
# RUN: llvm-mc -triple=ppu -mattr=+e,+d -target-abi ilp32f < %s 2>&1 \
# RUN:   | FileCheck -check-prefix=RV32EFD-ILP32F %s
# RUN: llvm-mc -triple=ppu -mattr=+e,+d -target-abi ilp32d < %s 2>&1 \
# RUN:   | FileCheck -check-prefix=RV32EFD-ILP32D %s

# RV32E-ILP32: Only the ilp32e ABI is supported for RV32E (ignoring target-abi)
# RV32EF-ILP32F: Only the ilp32e ABI is supported for RV32E (ignoring target-abi)
# RV32EFD-ILP32F: Only the ilp32e ABI is supported for RV32E (ignoring target-abi)
# RV32EFD-ILP32D: Only the ilp32e ABI is supported for RV32E (ignoring target-abi)

nop
