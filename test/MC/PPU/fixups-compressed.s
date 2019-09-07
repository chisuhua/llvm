# RUN: llvm-mc %s -triple ppu -mattr=+c -show-encoding \
# RUN:     | FileCheck -check-prefix=CHECK-FIXUP %s
# RUN: llvm-mc -triple ppu -filetype=obj -mattr=+c < %s \
# RUN:     | llvm-objdump -d -ppu-no-aliases - | FileCheck -check-prefix=CHECK-INSTR %s
# RUN: llvm-mc -filetype=obj -mattr=+c -triple=ppu %s \
# RUN:     | llvm-readobj -r | FileCheck %s -check-prefix=CHECK-REL

.LBB0_2:
# CHECK-FIXUP:   fixup A - offset: 0, value: .LBB0_2, kind: fixup_ppu_rvc_jump
# CHECK-INSTR: c.j     0
c.j     .LBB0_2
# CHECK:   fixup A - offset: 0, value: func1, kind: fixup_ppu_rvc_jump
# CHECK-INSTR: c.jal   6
c.jal   func1
# CHECK-FIXUP:   fixup A - offset: 0, value: .LBB0_2, kind: fixup_ppu_rvc_branch
# CHECK-INSTR: c.beqz  a3, -4
c.beqz  a3, .LBB0_2
# CHECK-FIXUP:   fixup A - offset: 0, value: .LBB0_2, kind: fixup_ppu_rvc_branch
# CHECK-INSTR: c.bnez  a5, -6
c.bnez  a5, .LBB0_2

func1:
  nop

# CHECK-REL-NOT: R_RISCV
