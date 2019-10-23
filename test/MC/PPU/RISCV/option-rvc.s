# RUN: llvm-mc -triple ppu -show-encoding < %s \
# RUN: | FileCheck -check-prefixes=CHECK,CHECK-ALIAS %s
# RUN: llvm-mc -triple ppu -show-encoding \
# RUN: -ppu-no-aliases <%s | FileCheck -check-prefixes=CHECK,CHECK-INST %s
# RUN: llvm-mc -triple ppu -filetype=obj < %s \
# RUN: | llvm-objdump  -triple ppu -mattr=+c -d - \
# RUN: | FileCheck -check-prefixes=CHECK-BYTES,CHECK-ALIAS %s
# RUN: llvm-mc -triple ppu -filetype=obj < %s \
# RUN: | llvm-objdump  -triple ppu -mattr=+c -d -ppu-no-aliases - \
# RUN: | FileCheck -check-prefixes=CHECK-BYTES,CHECK-INST %s

# RUN: | FileCheck -check-prefixes=CHECK-ALIAS %s
# RUN: -ppu-no-aliases <%s | FileCheck -check-prefixes=CHECK-INST %s
# RUN: | FileCheck -check-prefixes=CHECK-BYTES,CHECK-ALIAS %s
# RUN: | FileCheck -check-prefixes=CHECK-BYTES,CHECK-INST %s

# CHECK-BYTES: 13 85 05 00
# CHECK-ALIAS: mv a0, a1
# CHECK-INST: addi a0, a1, 0
# CHECK: # encoding:  [0x13,0x85,0x05,0x00]
addi a0, a1, 0

# CHECK-BYTES: 13 04 c1 3f
# CHECK-ALIAS: addi s0, sp, 1020
# CHECK-INST: addi s0, sp, 1020
# CHECK: # encoding:  [0x13,0x04,0xc1,0x3f]
addi s0, sp, 1020


# CHECK: .option rvc
.option rvc
# CHECK-BYTES: 2e 85
# CHECK-ALIAS: add a0, zero, a1
# CHECK-INST: c.mv a0, a1
# CHECK: # encoding:  [0x2e,0x85]
addi a0, a1, 0

# CHECK-BYTES: e0 1f
# CHECK-ALIAS: addi s0, sp, 1020
# CHECK-INST: c.addi4spn s0, sp, 1020
# CHECK: # encoding:  [0xe0,0x1f]
addi s0, sp, 1020

# CHECK: .option norvc
.option norvc
# CHECK-BYTES: 13 85 05 00
# CHECK-ALIAS: mv a0, a1
# CHECK-INST: addi a0, a1, 0
# CHECK: # encoding:  [0x13,0x85,0x05,0x00]
addi a0, a1, 0

# CHECK-BYTES: 13 04 c1 3f
# CHECK-ALIAS: addi s0, sp, 1020
# CHECK-INST: addi s0, sp, 1020
# CHECK: # encoding:  [0x13,0x04,0xc1,0x3f]
addi s0, sp, 1020

# CHECK: .option rvc
.option rvc
# CHECK-BYTES: 2e 85
# CHECK-ALIAS: add a0, zero, a1
# CHECK-INST: c.mv a0, a1
# CHECK: # encoding:  [0x2e,0x85]
addi a0, a1, 0

# CHECK-BYTES: e0 1f
# CHECK-ALIAS: addi s0, sp, 1020
# CHECK-INST: c.addi4spn s0, sp, 1020
# CHECK: # encoding:  [0xe0,0x1f]
addi s0, sp, 1020

# CHECK: .option norvc
.option norvc
# CHECK-BYTES: 13 85 05 00
# CHECK-ALIAS: mv a0, a1
# CHECK-INST: addi a0, a1, 0
# CHECK: # encoding:  [0x13,0x85,0x05,0x00]
addi a0, a1, 0

# CHECK-BYTES: 13 04 c1 3f
# CHECK-ALIAS: addi s0, sp, 1020
# CHECK-INST: addi s0, sp, 1020
# CHECK: # encoding:  [0x13,0x04,0xc1,0x3f]
addi s0, sp, 1020
