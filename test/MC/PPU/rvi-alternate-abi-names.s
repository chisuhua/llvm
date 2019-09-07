# RUN: llvm-mc %s -triple=ppu \
# RUN:     | FileCheck %s
# RUN:     | FileCheck %s
# RUN: llvm-mc -filetype=obj -triple ppu < %s \
# RUN:     | llvm-objdump -d -r - \
# RUN:     | FileCheck %s
# RUN:     | llvm-objdump -d -r - \
# RUN:     | FileCheck %s

# 'fp' is an alternate ABI name for 's0' and it should be accepted in input.
# However, 's0' should be printed in preference.

# CHECK: addi s0, s0, -4
addi fp, fp, -4
