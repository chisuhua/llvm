# RUN: llvm-mc %s -triple=ppu -mattr=+d | FileCheck %s

# CHECK: .Lpcrel_hi0:
# CHECK: auipc a2, %pcrel_hi(a_symbol)
# CHECK: fld  fa2, %pcrel_lo(.Lpcrel_hi0)(a2)
fld fa2, a_symbol, a2

# CHECK: .Lpcrel_hi1:
# CHECK: auipc a3, %pcrel_hi(a_symbol)
# CHECK: fsd  fa2, %pcrel_lo(.Lpcrel_hi1)(a3)
fsd fa2, a_symbol, a3
