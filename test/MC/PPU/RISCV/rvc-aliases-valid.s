# RUN: llvm-mc %s -triple=ppu -mattr=+c -ppu-no-aliases \
# RUN:     | FileCheck -check-prefixes=CHECK-EXPAND %s
# RUN:     | FileCheck -check-prefixes=CHECK-EXPAND %s
# RUN: llvm-mc -filetype=obj -triple ppu -mattr=+c < %s \
# RUN:     | llvm-objdump -ppu-no-aliases -d - \
# RUN:     | FileCheck -check-prefixes=CHECK-EXPAND %s
# RUN:     | llvm-objdump -ppu-no-aliases -d - \
# RUN:     | FileCheck -check-prefixes=CHECK-EXPAND %s

# CHECK-EXPAND: c.lw s0, 0(s1)
c.lw x8, (x9)
# CHECK-EXPAND: c.sw s0, 0(s1)
c.sw x8, (x9)
# CHECK-EXPAND: c.lwsp s0, 0(sp)
c.lwsp x8, (x2)
# CHECK-EXPAND: c.swsp s0, 0(sp)
c.swsp x8, (x2)
