# RUN:     | FileCheck -check-prefixes=CHECK-ASM,CHECK-ASM-AND-OBJ %s
# RUN:     | llvm-objdump -ppu-no-aliases -d -r - \
# RUN:     | FileCheck -check-prefixes=CHECK-OBJ,CHECK-ASM-AND-OBJ %s

# CHECK-ASM-AND-OBJ: c.slli zero, 63
# CHECK-ASM: encoding: [0x7e,0x10]
c.slli x0, 63
