# RUN: llvm-mc -triple=ppu -filetype=obj < %s \
# RUN:   | llvm-readobj --file-headers - \
# RUN:   | FileCheck -check-prefix=CHECK-NONE %s
# RUN: llvm-mc -triple=ppu -target-abi ilp32 -filetype=obj < %s \
# RUN:   | llvm-readobj --file-headers - \
# RUN:   | FileCheck -check-prefix=CHECK-NONE %s
# RUN: llvm-mc -triple=ppu -mattr=+f -target-abi ilp32 -filetype=obj < %s \
# RUN:   | llvm-readobj --file-headers - \
# RUN:   | FileCheck -check-prefix=CHECK-NONE %s
# RUN: llvm-mc -triple=ppu -mattr=+d -target-abi ilp32 -filetype=obj < %s \
# RUN:   | llvm-readobj --file-headers - \
# RUN:   | FileCheck -check-prefix=CHECK-NONE %s
# RUN:   | llvm-readobj --file-headers - \
# RUN:   | FileCheck -check-prefix=CHECK-NONE %s
# RUN:   | llvm-readobj --file-headers - \
# RUN:   | FileCheck -check-prefix=CHECK-NONE %s
# RUN:   | llvm-readobj --file-headers - \
# RUN:   | FileCheck -check-prefix=CHECK-NONE %s
# RUN:   | llvm-readobj --file-headers - \
# RUN:   | FileCheck -check-prefix=CHECK-NONE %s

# RUN: llvm-mc -triple=ppu -mattr=+f -target-abi ilp32f -filetype=obj < %s \
# RUN:   | llvm-readobj --file-headers - \
# RUN:   | FileCheck -check-prefix=CHECK-FLOAT-SINGLE %s
# RUN: llvm-mc -triple=ppu -mattr=+d -target-abi ilp32f -filetype=obj < %s \
# RUN:   | llvm-readobj --file-headers - \
# RUN:   | FileCheck -check-prefix=CHECK-FLOAT-SINGLE %s
# RUN:   | llvm-readobj --file-headers - \
# RUN:   | FileCheck -check-prefix=CHECK-FLOAT-SINGLE %s
# RUN:   | llvm-readobj --file-headers - \
# RUN:   | FileCheck -check-prefix=CHECK-FLOAT-SINGLE %s

# RUN: llvm-mc -triple=ppu -mattr=+d -target-abi ilp32d -filetype=obj < %s \
# RUN:   | llvm-readobj --file-headers - \
# RUN:   | FileCheck -check-prefix=CHECK-FLOAT-DOUBLE %s
# RUN:   | llvm-readobj --file-headers - \
# RUN:   | FileCheck -check-prefix=CHECK-FLOAT-DOUBLE %s

# RUN: llvm-mc -triple=ppu -target-abi ilp32e -filetype=obj < %s \
# RUN:   | llvm-readobj --file-headers - \
# RUN:   | FileCheck -check-prefix=CHECK-RVE %s

# CHECK-NONE:               Flags [ (0x0)
# CHECK-NONE-NEXT:          ]

# CHECK-FLOAT-SINGLE:       Flags [ (0x2)
# CHECK-FLOAT-SINGLE-NEXT:    EF_RISCV_FLOAT_ABI_SINGLE (0x2)
# CHECK-FLOAT-SINGLE-NEXT:  ]

# CHECK-FLOAT-DOUBLE:       Flags [ (0x4)
# CHECK-FLOAT-DOUBLE-NEXT:    EF_RISCV_FLOAT_ABI_DOUBLE (0x4)
# CHECK-FLOAT-DOUBLE-NEXT:  ]

# CHECK-RVE:                Flags [ (0x8)
# CHECK-RVE-NEXT:             EF_RISCV_RVE (0x8)
# CHECK-RVE-NEXT:           ]

nop
