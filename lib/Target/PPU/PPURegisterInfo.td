//===-- PPURegisterInfo.td - PPU Register defs --------*- tablegen -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
//  Declarations that describe the PPU register files
//===----------------------------------------------------------------------===//

let Namespace = "PPU" in {
class PPUReg<bits<5> Enc, string n, list<string> alt = []> : Register<n> {
  let HWEncoding{4-0} = Enc;
  let AltNames = alt;
}

class PPUReg32<bits<5> Enc, string n, list<string> alt = []> : Register<n> {
  let HWEncoding{4-0} = Enc;
  let AltNames = alt;
}

// Because PPUReg64 register have AsmName and AltNames that alias with their
// 32-bit sub-register, PPUAsmParser will need to coerce a register number
// from a PPUReg32 to the equivalent PPUReg64 when appropriate.
def sub_32 : SubRegIndex<32>;
class PPUReg64<PPUReg32 subreg> : Register<""> {
  let HWEncoding{4-0} = subreg.HWEncoding{4-0};
  let SubRegs = [subreg];
  let SubRegIndices = [sub_32];
  let AsmName = subreg.AsmName;
  let AltNames = subreg.AltNames;
}

def ABIRegAltName : RegAltNameIndex;
} // Namespace = "PPU"

// Integer registers
// CostPerUse is set higher for registers that may not be compressible as they
// are not part of GPRC, the most restrictive register class used by the
// compressed instruction set. This will influence the greedy register
// allocator to reduce the use of registers that can't be encoded in 16 bit
// instructions. This affects register allocation even when compressed
// instruction isn't targeted, we see no major negative codegen impact.

let RegAltNameIndices = [ABIRegAltName] in {
  def X0  : PPUReg<0, "x0", ["zero"]>, DwarfRegNum<[0]>;
  let CostPerUse = 1 in {
  def X1  : PPUReg<1, "x1", ["ra"]>, DwarfRegNum<[1]>;
  def X2  : PPUReg<2, "x2", ["sp"]>, DwarfRegNum<[2]>;
  def X3  : PPUReg<3, "x3", ["gp"]>, DwarfRegNum<[3]>;
  def X4  : PPUReg<4, "x4", ["tp"]>, DwarfRegNum<[4]>;
  def X5  : PPUReg<5, "x5", ["t0"]>, DwarfRegNum<[5]>;
  def X6  : PPUReg<6, "x6", ["t1"]>, DwarfRegNum<[6]>;
  def X7  : PPUReg<7, "x7", ["t2"]>, DwarfRegNum<[7]>;
  }
  def X8  : PPUReg<8, "x8", ["s0", "fp"]>, DwarfRegNum<[8]>;
  def X9  : PPUReg<9, "x9", ["s1"]>, DwarfRegNum<[9]>;
  def X10 : PPUReg<10,"x10", ["a0"]>, DwarfRegNum<[10]>;
  def X11 : PPUReg<11,"x11", ["a1"]>, DwarfRegNum<[11]>;
  def X12 : PPUReg<12,"x12", ["a2"]>, DwarfRegNum<[12]>;
  def X13 : PPUReg<13,"x13", ["a3"]>, DwarfRegNum<[13]>;
  def X14 : PPUReg<14,"x14", ["a4"]>, DwarfRegNum<[14]>;
  def X15 : PPUReg<15,"x15", ["a5"]>, DwarfRegNum<[15]>;
  let CostPerUse = 1 in {
  def X16 : PPUReg<16,"x16", ["a6"]>, DwarfRegNum<[16]>;
  def X17 : PPUReg<17,"x17", ["a7"]>, DwarfRegNum<[17]>;
  def X18 : PPUReg<18,"x18", ["s2"]>, DwarfRegNum<[18]>;
  def X19 : PPUReg<19,"x19", ["s3"]>, DwarfRegNum<[19]>;
  def X20 : PPUReg<20,"x20", ["s4"]>, DwarfRegNum<[20]>;
  def X21 : PPUReg<21,"x21", ["s5"]>, DwarfRegNum<[21]>;
  def X22 : PPUReg<22,"x22", ["s6"]>, DwarfRegNum<[22]>;
  def X23 : PPUReg<23,"x23", ["s7"]>, DwarfRegNum<[23]>;
  def X24 : PPUReg<24,"x24", ["s8"]>, DwarfRegNum<[24]>;
  def X25 : PPUReg<25,"x25", ["s9"]>, DwarfRegNum<[25]>;
  def X26 : PPUReg<26,"x26", ["s10"]>, DwarfRegNum<[26]>;
  def X27 : PPUReg<27,"x27", ["s11"]>, DwarfRegNum<[27]>;
  def X28 : PPUReg<28,"x28", ["t3"]>, DwarfRegNum<[28]>;
  def X29 : PPUReg<29,"x29", ["t4"]>, DwarfRegNum<[29]>;
  def X30 : PPUReg<30,"x30", ["t5"]>, DwarfRegNum<[30]>;
  def X31 : PPUReg<31,"x31", ["t6"]>, DwarfRegNum<[31]>;
  }

// more SGPR registers
// foreach Index = 32-95 in {
//   def X#Index : PPUReg<Index, "x"#Index, ["x"#Index]>, DwarfRegNum<[Index]>;
// }


}

def XLenVT : ValueTypeByHwMode<[RV32, RV64, DefaultMode],
                               [i32,  i64,  i32]>;

// The order of registers represents the preferred allocation sequence.
// Registers are listed in the order caller-save, callee-save, specials.
def GPR : RegisterClass<"PPU", [XLenVT], 32, (add
    (sequence "X%u", 10, 17),
    (sequence "X%u", 5, 7),
    (sequence "X%u", 28, 31),
    (sequence "X%u", 8, 9),
    (sequence "X%u", 18, 27),
    (sequence "X%u", 0, 4)
  )> {
  let RegInfos = RegInfoByHwMode<
      [RV32,              RV64,              DefaultMode],
      [RegInfo<32,32,32>, RegInfo<64,64,64>, RegInfo<32,32,32>]>;
}

def GPRX0 : RegisterClass<"PPU", [XLenVT], 32, (add X0)> {
  let RegInfos = RegInfoByHwMode<
      [RV32,              RV64,              DefaultMode],
      [RegInfo<32,32,32>, RegInfo<64,64,64>, RegInfo<32,32,32>]>;
}

// The order of registers represents the preferred allocation sequence.
// Registers are listed in the order caller-save, callee-save, specials.
def GPRNoX0 : RegisterClass<"PPU", [XLenVT], 32, (add
    (sequence "X%u", 10, 17),
    (sequence "X%u", 5, 7),
    (sequence "X%u", 28, 31),
    (sequence "X%u", 8, 9),
    (sequence "X%u", 18, 27),
    (sequence "X%u", 1, 4)
  )> {
  let RegInfos = RegInfoByHwMode<
      [RV32,              RV64,              DefaultMode],
      [RegInfo<32,32,32>, RegInfo<64,64,64>, RegInfo<32,32,32>]>;
}

def GPRNoX0X2 : RegisterClass<"PPU", [XLenVT], 32, (add
    (sequence "X%u", 10, 17),
    (sequence "X%u", 5, 7),
    (sequence "X%u", 28, 31),
    (sequence "X%u", 8, 9),
    (sequence "X%u", 18, 27),
    X1, X3, X4
  )> {
  let RegInfos = RegInfoByHwMode<
      [RV32,              RV64,              DefaultMode],
      [RegInfo<32,32,32>, RegInfo<64,64,64>, RegInfo<32,32,32>]>;
}

def GPRC : RegisterClass<"PPU", [XLenVT], 32, (add
    (sequence "X%u", 10, 15),
    (sequence "X%u", 8, 9)
  )> {
  let RegInfos = RegInfoByHwMode<
      [RV32,              RV64,              DefaultMode],
      [RegInfo<32,32,32>, RegInfo<64,64,64>, RegInfo<32,32,32>]>;
}

// For indirect tail calls, we can't use callee-saved registers, as they are
// restored to the saved value before the tail call, which would clobber a call
// address.
def GPRTC : RegisterClass<"PPU", [XLenVT], 32, (add
    (sequence "X%u", 5, 7),
    (sequence "X%u", 10, 17),
    (sequence "X%u", 28, 31)
  )> {
  let RegInfos = RegInfoByHwMode<
      [RV32,              RV64,              DefaultMode],
      [RegInfo<32,32,32>, RegInfo<64,64,64>, RegInfo<32,32,32>]>;
}

def SP : RegisterClass<"PPU", [XLenVT], 32, (add X2)> {
  let RegInfos = RegInfoByHwMode<
      [RV32,              RV64,              DefaultMode],
      [RegInfo<32,32,32>, RegInfo<64,64,64>, RegInfo<32,32,32>]>;
}

// Floating point registers
let RegAltNameIndices = [ABIRegAltName] in {
  def F0_32  : PPUReg32<0, "f0", ["ft0"]>, DwarfRegNum<[32]>;
  def F1_32  : PPUReg32<1, "f1", ["ft1"]>, DwarfRegNum<[33]>;
  def F2_32  : PPUReg32<2, "f2", ["ft2"]>, DwarfRegNum<[34]>;
  def F3_32  : PPUReg32<3, "f3", ["ft3"]>, DwarfRegNum<[35]>;
  def F4_32  : PPUReg32<4, "f4", ["ft4"]>, DwarfRegNum<[36]>;
  def F5_32  : PPUReg32<5, "f5", ["ft5"]>, DwarfRegNum<[37]>;
  def F6_32  : PPUReg32<6, "f6", ["ft6"]>, DwarfRegNum<[38]>;
  def F7_32  : PPUReg32<7, "f7", ["ft7"]>, DwarfRegNum<[39]>;
  def F8_32  : PPUReg32<8, "f8", ["fs0"]>, DwarfRegNum<[40]>;
  def F9_32  : PPUReg32<9, "f9", ["fs1"]>, DwarfRegNum<[41]>;
  def F10_32 : PPUReg32<10,"f10", ["fa0"]>, DwarfRegNum<[42]>;
  def F11_32 : PPUReg32<11,"f11", ["fa1"]>, DwarfRegNum<[43]>;
  def F12_32 : PPUReg32<12,"f12", ["fa2"]>, DwarfRegNum<[44]>;
  def F13_32 : PPUReg32<13,"f13", ["fa3"]>, DwarfRegNum<[45]>;
  def F14_32 : PPUReg32<14,"f14", ["fa4"]>, DwarfRegNum<[46]>;
  def F15_32 : PPUReg32<15,"f15", ["fa5"]>, DwarfRegNum<[47]>;
  def F16_32 : PPUReg32<16,"f16", ["fa6"]>, DwarfRegNum<[48]>;
  def F17_32 : PPUReg32<17,"f17", ["fa7"]>, DwarfRegNum<[49]>;
  def F18_32 : PPUReg32<18,"f18", ["fs2"]>, DwarfRegNum<[50]>;
  def F19_32 : PPUReg32<19,"f19", ["fs3"]>, DwarfRegNum<[51]>;
  def F20_32 : PPUReg32<20,"f20", ["fs4"]>, DwarfRegNum<[52]>;
  def F21_32 : PPUReg32<21,"f21", ["fs5"]>, DwarfRegNum<[53]>;
  def F22_32 : PPUReg32<22,"f22", ["fs6"]>, DwarfRegNum<[54]>;
  def F23_32 : PPUReg32<23,"f23", ["fs7"]>, DwarfRegNum<[55]>;
  def F24_32 : PPUReg32<24,"f24", ["fs8"]>, DwarfRegNum<[56]>;
  def F25_32 : PPUReg32<25,"f25", ["fs9"]>, DwarfRegNum<[57]>;
  def F26_32 : PPUReg32<26,"f26", ["fs10"]>, DwarfRegNum<[58]>;
  def F27_32 : PPUReg32<27,"f27", ["fs11"]>, DwarfRegNum<[59]>;
  def F28_32 : PPUReg32<28,"f28", ["ft8"]>, DwarfRegNum<[60]>;
  def F29_32 : PPUReg32<29,"f29", ["ft9"]>, DwarfRegNum<[61]>;
  def F30_32 : PPUReg32<30,"f30", ["ft10"]>, DwarfRegNum<[62]>;
  def F31_32 : PPUReg32<31,"f31", ["ft11"]>, DwarfRegNum<[63]>;

  foreach Index = 0-31 in {
    def F#Index#_64 : PPUReg64<!cast<PPUReg32>("F"#Index#"_32")>,
      DwarfRegNum<[!add(Index, 32)]>;
  }
}

// The order of registers represents the preferred allocation sequence,
// meaning caller-save regs are listed before callee-save.
def FPR32 : RegisterClass<"PPU", [f32], 32, (add
    (sequence "F%u_32", 0, 7),
    (sequence "F%u_32", 10, 17),
    (sequence "F%u_32", 28, 31),
    (sequence "F%u_32", 8, 9),
    (sequence "F%u_32", 18, 27)
)>;

def FPR32C : RegisterClass<"PPU", [f32], 32, (add
  (sequence "F%u_32", 10, 15),
  (sequence "F%u_32", 8, 9)
)>;

// The order of registers represents the preferred allocation sequence,
// meaning caller-save regs are listed before callee-save.
def FPR64 : RegisterClass<"PPU", [f64], 64, (add
    (sequence "F%u_64", 0, 7),
    (sequence "F%u_64", 10, 17),
    (sequence "F%u_64", 28, 31),
    (sequence "F%u_64", 8, 9),
    (sequence "F%u_64", 18, 27)
)>;

def FPR64C : RegisterClass<"PPU", [f64], 64, (add
  (sequence "F%u_64", 10, 15),
  (sequence "F%u_64", 8, 9)
)>;

// Vector registers
let RegAltNameIndices = [ABIRegAltName] in {
  def V0  : PPUReg<0, "v0", ["v0"]>;
  def V1  : PPUReg<1, "v1", ["v1"]>;
  def V2  : PPUReg<2, "v2", ["v2"]>;
  def V3  : PPUReg<3, "v3", ["v3"]>;
  def V4  : PPUReg<4, "v4", ["v4"]>;
  def V5  : PPUReg<5, "v5", ["v5"]>;
  def V6  : PPUReg<6, "v6", ["v6"]>;
  def V7  : PPUReg<7, "v7", ["v7"]>;
  def V8  : PPUReg<8, "v8", ["v8"]>;
  def V9  : PPUReg<9, "v9", ["v9"]>;
  def V10 : PPUReg<10,"v10", ["v10"]>;
  def V11 : PPUReg<11,"v11", ["v11"]>;
  def V12 : PPUReg<12,"v12", ["v12"]>;
  def V13 : PPUReg<13,"v13", ["v13"]>;
  def V14 : PPUReg<14,"v14", ["v14"]>;
  def V15 : PPUReg<15,"v15", ["v15"]>;
  def V16 : PPUReg<16,"v16", ["v16"]>;
  def V17 : PPUReg<17,"v17", ["v17"]>;
  def V18 : PPUReg<18,"v18", ["v18"]>;
  def V19 : PPUReg<19,"v19", ["v19"]>;
  def V20 : PPUReg<20,"v20", ["v20"]>;
  def V21 : PPUReg<21,"v21", ["v21"]>;
  def V22 : PPUReg<22,"v22", ["v22"]>;
  def V23 : PPUReg<23,"v23", ["v23"]>;
  def V24 : PPUReg<24,"v24", ["v24"]>;
  def V25 : PPUReg<25,"v25", ["v25"]>;
  def V26 : PPUReg<26,"v26", ["v26"]>;
  def V27 : PPUReg<27,"v27", ["v27"]>;
  def V28 : PPUReg<28,"v28", ["v28"]>;
  def V29 : PPUReg<29,"v29", ["v29"]>;
  def V30 : PPUReg<30,"v30", ["v30"]>;
  def V31 : PPUReg<31,"v31", ["v31"]>;

// foreach Index = 32-255 in {
//   def V#Index : PPUReg <Index, "v"#Index, ["v"#Index]>;
// }

}

def VR : RegisterClass<"PPU", [nxv1i32], 32, (sequence "V%u", 0, 31)>;

// Active vector length register (vl)
// There is only one physical register, and it is not explicitly encoded
// as operand of in any instructions, but for various reasons it is easier
// to treat the active vector length as an ordinary virtual register
let RegAltNameIndices = [ABIRegAltName] in
def VL: PPUReg<0, "vl", ["vl"]>;

def VLR: RegisterClass<"PPU", [XLenVT], 32, (add VL)> {
  let RegInfos = RegInfoByHwMode<
    [RV32,              RV64,              DefaultMode],
    [RegInfo<32,32,32>, RegInfo<64,64,64>, RegInfo<32,32,32>]>;
}

// VCFG
def VCFG : PPUReg<0, "vcfg">;
def VCFGR : RegisterClass<"PPU", [XLenVT], 32, (add VCFG)> {
  let CopyCost = -1; // Don't allow copying
  let isAllocatable = 0; // Not allocatable
  let RegInfos = RegInfoByHwMode<
      [RV32,              RV64,              DefaultMode],
      [RegInfo<32,32,32>, RegInfo<64,64,64>, RegInfo<32,32,32>]>;
}

// VCC
def VCC : PPUReg<0, "vcc">;
def VCCR : RegisterClass<"PPU", [XLenVT], 32, (add VCC)> {
  let CopyCost = -1; // Don't allow copying
  let isAllocatable = 0; // Not allocatable
  let RegInfos = RegInfoByHwMode<
      [RV32,              RV64,              DefaultMode],
      [RegInfo<32,32,32>, RegInfo<64,64,64>, RegInfo<32,32,32>]>;
}

// VCC
def EXEC : PPUReg<0, "exec">;
def EXECR : RegisterClass<"PPU", [XLenVT], 32, (add EXEC)> {
  let CopyCost = -1; // Don't allow copying
  let isAllocatable = 0; // Not allocatable
  let RegInfos = RegInfoByHwMode<
      [RV32,              RV64,              DefaultMode],
      [RegInfo<32,32,32>, RegInfo<64,64,64>, RegInfo<32,32,32>]>;
}

// Pseudo-registers: Used as placeholders during isel and immediately
// replaced, never seeing the verifier.
def PRIVATE_RSRC_REG : PPUReg<0, "private_rsrc">;
def FP_REG : PPUReg<0, "fp">;
def SP_REG : PPUReg<0, "sp">;
// def SCRATCH_WAVE_OFFSET_REG : BIReg<"", 0>;


// 32-bit real registers, for MC only.
// May be used with both 32-bit and 64-bit operands.
// def SRC_VCCZ : PPUReg<0,"src_vccz">;
// def SRC_EXECZ : PPUReg<0, "src_execz">;
// def SRC_SCC : PPUReg<0, "src_scc">;

// 1-bit pseudo register, for codegen only.
// Should never be emitted.
def SCC : PPUReg<0, "scc">;
def SCC_CLASS : RegisterClass<"PPU", [i1], 1, (add SCC)> {
  let CopyCost = -1;
  let isAllocatable = 0;
  let RegInfos = RegInfoByHwMode<
      [RV32,              RV64,              DefaultMode],
      [RegInfo<32,32,32>, RegInfo<64,64,64>, RegInfo<32,32,32>]>;
}

def SReg_32_XM0 : RegisterClass<"PPU", [XLenVT], 32,
  (add GPR)> { // , FLAT_SCR, XNACK_MASK, TTMP_64, TBA, TMA)> {
  let CopyCost = 1;
  let AllocationPriority = 13;
  let RegInfos = RegInfoByHwMode<
      [RV32,              RV64,              DefaultMode],
      [RegInfo<32,32,32>, RegInfo<64,64,64>, RegInfo<32,32,32>]>;
}


def SReg_32_XEXEC : RegisterClass<"PPU", [XLenVT], 32,
  (add GPR, VCC)> { // , FLAT_SCR, XNACK_MASK, TTMP_64, TBA, TMA)> {
  let CopyCost = 1;
  let AllocationPriority = 13;
  let RegInfos = RegInfoByHwMode<
      [RV32,              RV64,              DefaultMode],
      [RegInfo<32,32,32>, RegInfo<64,64,64>, RegInfo<32,32,32>]>;
}

def SReg_32_XM0_XEXEC : RegisterClass<"PPU", [XLenVT], 32,
  (add GPR, VCC)> { // , FLAT_SCR, XNACK_MASK, TTMP_64, TBA, TMA)> {
  let CopyCost = 1;
  let AllocationPriority = 13;
  let RegInfos = RegInfoByHwMode<
      [RV32,              RV64,              DefaultMode],
      [RegInfo<32,32,32>, RegInfo<64,64,64>, RegInfo<32,32,32>]>;
}

def SReg_32 : RegisterClass<"PPU", [XLenVT, i32, f32, i1, v2i16, v2f16], 32,
  (add SReg_32_XEXEC, EXEC)> {
  let CopyCost = 1;
  let AllocationPriority = 13;
  let RegInfos = RegInfoByHwMode<
      [RV32,              RV64,              DefaultMode],
      [RegInfo<32,32,32>, RegInfo<64,64,64>, RegInfo<32,32,32>]>;

}

def SReg_1_XEXEC : RegisterClass<"PPU", [i1], 32,
  (add SReg_32_XEXEC)> {
  let CopyCost = 1;
  let isAllocatable = 0;
  let RegInfos = RegInfoByHwMode<
      [RV32,              RV64,              DefaultMode],
      [RegInfo<32,32,32>, RegInfo<64,64,64>, RegInfo<32,32,32>]>;

}

def SReg_1 : RegisterClass<"PPU", [i1], 32,
  (add SReg_1_XEXEC, EXEC)> {
  let CopyCost = 1;
  let isAllocatable = 0;
  let RegInfos = RegInfoByHwMode<
      [RV32,              RV64,              DefaultMode],
      [RegInfo<32,32,32>, RegInfo<64,64,64>, RegInfo<32,32,32>]>;

}


