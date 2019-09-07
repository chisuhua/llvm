# RUN: llvm-mc %s -triple=ppu -mattr=+c -ppu-no-aliases -show-encoding \
# RUN:     | FileCheck -check-prefixes=CHECK-ASM,CHECK-ASM-AND-OBJ %s
# RUN: llvm-mc -filetype=obj -triple=ppu -mattr=+c < %s \
# RUN:     | llvm-objdump -mattr=+c -ppu-no-aliases -d -r - \
# RUN:     | FileCheck -check-prefixes=CHECK-OBJ,CHECK-ASM-AND-OBJ %s
#
# RUN: not llvm-mc -triple ppu \
# RUN:     -ppu-no-aliases -show-encoding < %s 2>&1 \
# RUN:     | FileCheck -check-prefixes=CHECK-NO-EXT %s
# RUN:     -ppu-no-aliases -show-encoding < %s 2>&1 \
# RUN:     | FileCheck -check-prefixes=CHECK-NO-EXT %s

# FIXME: error message for c.jal with rv64c is misleading

# CHECK-ASM-AND-OBJ: c.jal 2046
# CHECK-ASM: encoding: [0xfd,0x2f]
# CHECK-NO-EXT: error: instruction use requires an option to be enabled
c.jal 2046
