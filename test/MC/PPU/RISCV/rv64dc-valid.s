# RUN:     | FileCheck -check-prefixes=CHECK-ASM,CHECK-ASM-AND-OBJ %s
# RUN:     | llvm-objdump -mattr=+c,+d -ppu-no-aliases -d -r - \
# RUN:     | FileCheck -check-prefixes=CHECK-OBJ,CHECK-ASM-AND-OBJ %s
#
# RUN:     -ppu-no-aliases -show-encoding < %s 2>&1 \
# RUN:     | FileCheck -check-prefixes=CHECK-NO-EXT %s
# RUN:     | FileCheck -check-prefixes=CHECK-NO-EXT %s

# CHECK-ASM-AND-OBJ: c.fldsp  fs0, 504(sp)
# CHECK-ASM: encoding: [0x7e,0x34]
# CHECK-NO-EXT:  error: instruction use requires an option to be enabled
c.fldsp  fs0, 504(sp)
# CHECK-ASM-AND-OBJ: c.fsdsp  fa7, 504(sp)
# CHECK-ASM: encoding: [0xc6,0xbf]
# CHECK-NO-EXT:  error: instruction use requires an option to be enabled
c.fsdsp  fa7, 504(sp)

# CHECK-ASM-AND-OBJ: c.fld  fa3, 248(a5)
# CHECK-ASM: encoding: [0xf4,0x3f]
# CHECK-NO-EXT:  error: instruction use requires an option to be enabled
c.fld  fa3, 248(a5)
# CHECK-ASM-AND-OBJ: c.fsd  fa2, 248(a1)
# CHECK-ASM: encoding: [0xf0,0xbd]
# CHECK-NO-EXT:  error: instruction use requires an option to be enabled
c.fsd  fa2, 248(a1)
