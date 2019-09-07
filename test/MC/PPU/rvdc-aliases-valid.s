# RUN: llvm-mc %s -triple=ppu -mattr=+c,+d -ppu-no-aliases \
# RUN:     | FileCheck -check-prefixes=CHECK-EXPAND %s
# RUN:     | FileCheck -check-prefixes=CHECK-EXPAND %s
# RUN: llvm-mc -filetype=obj -triple ppu -mattr=+c,+d < %s \
# RUN:     | llvm-objdump -mattr=+c,+d -ppu-no-aliases -d - \
# RUN:     | FileCheck -check-prefixes=CHECK-EXPAND %s
# RUN:     | llvm-objdump -mattr=+c,+d -ppu-no-aliases -d - \
# RUN:     | FileCheck -check-prefixes=CHECK-EXPAND %s

c.fld f8, (x9)
# CHECK-EXPAND: c.fsd fs0, 0(s1)
c.fsd f8, (x9)
# CHECK-EXPAND: c.fldsp fs0, 0(sp)
c.fldsp f8, (x2)
# CHECK-EXPAND: c.fsdsp fs0, 0(sp)
c.fsdsp f8, (x2)
