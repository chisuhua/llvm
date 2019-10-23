# RUN: llvm-mc -triple ppu -mattr=-relax -ppu-no-aliases < %s \
# RUN:     | FileCheck -check-prefix=CHECK-INST %s
# RUN: llvm-mc -filetype=obj -triple ppu < %s \
# RUN:     | llvm-readobj -r | FileCheck -check-prefix=CHECK-RELOC %s
# RUN: llvm-mc -triple ppu -filetype=obj < %s \
# RUN:     | llvm-objdump  -triple ppu -mattr=+c -d - \
# RUN:     | FileCheck -check-prefixes=CHECK-BYTES,CHECK-ALIAS %s

# RUN:     | FileCheck -check-prefix=CHECK-INST %s
# RUN:     | llvm-readobj -r | FileCheck -check-prefix=CHECK-RELOC %s
# RUN:     | FileCheck -check-prefixes=CHECK-BYTES,CHECK-ALIAS %s

# Test the operation of the push and pop assembler directives when
# using .option relax and .option rvc. Checks that using .option pop
# correctly restores all target features to their state at the point
# where .option pop was last used.

# CHECK-INST: call foo
# CHECK-RELOC: R_RISCV_CALL foo 0x0
# CHECK-RELOC-NOT: R_RISCV_RELAX - 0x0
call foo

# CHECK-INST: addi s0, sp, 1020
# CHECK-BYTES: 13 04 c1 3f
# CHECK-ALIAS: addi s0, sp, 1020
addi s0, sp, 1020

.option push    # Push relax=false, rvc=false
# CHECK-INST: .option push

.option relax
# CHECK-INST: .option relax
# CHECK-INST: call bar
# CHECK-RELOC-NEXT: R_RISCV_CALL bar 0x0
# CHECK-RELOC-NEXT: R_RISCV_RELAX - 0x0
call bar

.option push    # Push relax=true, rvc=false
# CHECK-INST: .option push

.option rvc
# CHECK-INST: .option rvc
# CHECK-INST: c.addi4spn s0, sp, 1020
# CHECK-BYTES: e0 1f
# CHECK-ALIAS: addi s0, sp, 1020
addi s0, sp, 1020

.option pop     # Pop relax=true, rvc=false
# CHECK-INST: .option pop
# CHECK-INST: addi s0, sp, 1020
# CHECK-BYTES: 13 04 c1 3f
# CHECK-ALIAS: addi s0, sp, 1020
addi s0, sp, 1020

# CHECK-INST: call bar
# CHECK-RELOC-NEXT: R_RISCV_CALL bar 0x0
# CHECK-RELOC-NEXT: R_RISCV_RELAX - 0x0
call bar

.option pop     # Pop relax=false, rvc=false
# CHECK-INST: .option pop
# CHECK-INST: call baz
# CHECK-RELOC: R_RISCV_CALL baz 0x0
# CHECK-RELOC-NOT: R_RISCV_RELAX - 0x0
call baz

# CHECK-INST: addi s0, sp, 1020
# CHECK-BYTES: 13 04 c1 3f
# CHECK-ALIAS: addi s0, sp, 1020
addi s0, sp, 1020
