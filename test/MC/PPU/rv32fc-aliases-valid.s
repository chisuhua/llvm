# RUN: llvm-mc %s -triple=ppu -mattr=+c,+f -ppu-no-aliases \
# RUN:     | FileCheck -check-prefixes=CHECK-EXPAND %s
# RUN: llvm-mc -filetype=obj -triple ppu -mattr=+c,+f < %s \
# RUN:     | llvm-objdump -mattr=+c,+f -ppu-no-aliases -d - \
# RUN:     | FileCheck -check-prefixes=CHECK-EXPAND %s

# CHECK-EXPAND: c.flw fs0, 0(s1)
c.flw f8, (x9)
# CHECK-EXPAND: c.fsw fs0, 0(s1)
c.fsw f8, (x9)
# CHECK-EXPAND: c.flwsp fs0, 0(sp)
c.flwsp f8, (x2)
# CHECK-EXPAND: c.fswsp fs0, 0(sp)
c.fswsp f8, (x2)
