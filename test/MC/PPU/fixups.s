# RUN: llvm-mc -triple ppu -ppu-no-aliases < %s -show-encoding \
# RUN:     | FileCheck -check-prefix=CHECK-FIXUP %s
# RUN: llvm-mc -filetype=obj -triple ppu < %s \
# RUN:     | llvm-objdump -ppu-no-aliases -d - \
# RUN:     | FileCheck -check-prefix=CHECK-INSTR %s
# RUN: llvm-mc -filetype=obj -triple=ppu %s \
# RUN:     | llvm-readobj -r | FileCheck %s -check-prefix=CHECK-REL

# Checks that fixups that can be resolved within the same object file are
# applied correctly

.LBB0:
lui t1, %hi(val)
# CHECK-FIXUP: fixup A - offset: 0, value: %hi(val), kind: fixup_ppu_hi20
# CHECK-INSTR: lui t1, 74565

lw a0, %lo(val)(t1)
# CHECK-FIXUP: fixup A - offset: 0, value: %lo(val), kind: fixup_ppu_lo12_i
# CHECK-INSTR: lw a0, 1656(t1)
addi a1, t1, %lo(val)
# CHECK-FIXUP: fixup A - offset: 0, value: %lo(val), kind: fixup_ppu_lo12_i
# CHECK-INSTR: addi a1, t1, 1656
sw a0, %lo(val)(t1)
# CHECK-FIXUP: fixup A - offset: 0, value: %lo(val), kind: fixup_ppu_lo12_s
# CHECK-INSTR: sw a0, 1656(t1)

1:
auipc t1, %pcrel_hi(.LBB0)
# CHECK-FIXUP: fixup A - offset: 0, value: %pcrel_hi(.LBB0), kind: fixup_ppu_pcrel_hi20
# CHECK-INSTR: auipc t1, 0
addi t1, t1, %pcrel_lo(1b)
# CHECK-FIXUP: fixup A - offset: 0, value: %pcrel_lo(.Ltmp0), kind: fixup_ppu_pcrel_lo12_i
# CHECK-INSTR: addi t1, t1, -16
sw t1, %pcrel_lo(1b)(t1)
# CHECK-FIXUP: fixup A - offset: 0, value: %pcrel_lo(.Ltmp0), kind: fixup_ppu_pcrel_lo12_s
# CHECK-INSTR: sw t1, -16(t1)

jal zero, .LBB0
# CHECK-FIXUP: fixup A - offset: 0, value: .LBB0, kind: fixup_ppu_jal
# CHECK-INSTR: jal zero, -28
jal zero, .LBB2
# CHECK-FIXUP: fixup A - offset: 0, value: .LBB2, kind: fixup_ppu_jal
# CHECK-INSTR: jal zero, 330996
beq a0, a1, .LBB0
# CHECK-FIXUP: fixup A - offset: 0, value: .LBB0, kind: fixup_ppu_branch
# CHECK-INSTR: beq a0, a1, -36
blt a0, a1, .LBB1
# CHECK-FIXUP: fixup A - offset: 0, value: .LBB1, kind: fixup_ppu_branch
# CHECK-INSTR: blt a0, a1, 1108

.fill 1104

.LBB1:

.fill 329876
addi zero, zero, 0
.LBB2:

.set val, 0x12345678

# CHECK-REL-NOT: R_RISCV

# Testing the function call offset could resovled by assembler
# when the function and the callsite within the same compile unit
# and the linker relaxation is disabled.
func:
.fill 100
call func
# CHECK-FIXUP: fixup A - offset: 0, value: func, kind: fixup_ppu_call
# CHECK-INSTR: auipc   ra, 0
# CHECK-INSTR: jalr    ra, -100(ra)

.fill 10000
call func
# CHECK-FIXUP: fixup A - offset: 0, value: func, kind: fixup_ppu_call
# CHECK-INSTR: auipc   ra, 1048574
# CHECK-INSTR: jalr    ra, -1916(ra)

.fill 20888
call func
# CHECK-FIXUP: fixup A - offset: 0, value: func, kind: fixup_ppu_call
# CHECK-INSTR: auipc   ra, 1048568
# CHECK-INSTR: jalr    ra, 1764(ra)
