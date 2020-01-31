//===-- PPURegisterInfo.td - PPU Register defs --------*- tablegen -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

// Ptr of addresspace
def p0 : PtrValueType<i64, 0>;
def p1 : PtrValueType<i64, 1>;
def p2 : PtrValueType<i32, 2>;
def p3 : PtrValueType<i32, 3>;
def p4 : PtrValueType<i64, 4>;
def p5 : PtrValueType<i32, 5>;
def p6 : PtrValueType<i32, 6>;


//===----------------------------------------------------------------------===//
//  Helpers
//===----------------------------------------------------------------------===//
let Namespace = "PPU" in {

foreach Index = 0-31 in {
  def sub#Index : SubRegIndex<32, !shl(Index, 5)>;
}

}

class getSubRegs<int size> {
  list<SubRegIndex> ret2 = [sub0, sub1];
  list<SubRegIndex> ret3 = [sub0, sub1, sub2];
  list<SubRegIndex> ret4 = [sub0, sub1, sub2, sub3];
  list<SubRegIndex> ret5 = [sub0, sub1, sub2, sub3, sub4];
  list<SubRegIndex> ret8 = [sub0, sub1, sub2, sub3, sub4, sub5, sub6, sub7];
  list<SubRegIndex> ret16 = [sub0, sub1, sub2, sub3,
                             sub4, sub5, sub6, sub7,
                             sub8, sub9, sub10, sub11,
                             sub12, sub13, sub14, sub15];
  list<SubRegIndex> ret32 = [sub0, sub1, sub2, sub3,
                             sub4, sub5, sub6, sub7,
                             sub8, sub9, sub10, sub11,
                             sub12, sub13, sub14, sub15,
                             sub16, sub17, sub18, sub19,
                             sub20, sub21, sub22, sub23,
                             sub24, sub25, sub26, sub27,
                             sub28, sub29, sub30, sub31];

  list<SubRegIndex> ret = !if(!eq(size, 2), ret2,
                              !if(!eq(size, 3), ret3,
                                  !if(!eq(size, 4), ret4,
                                      !if(!eq(size, 5), ret5,
                                          !if(!eq(size, 8), ret8,
                                              !if(!eq(size, 16), ret16, ret32))))));
}

// Generates list of sequential register tuple names.
// E.g. RegSeq<3,2,2,"s">.ret -> [ "s[0:1]", "s[2:3]" ]
class RegSeqNames<int last_reg, int stride, int size, string prefix,
                  int start = 0> {
  int next = !add(start, stride);
  int end_reg = !add(!add(start, size), -1);
  list<string> ret =
    !if(!le(end_reg, last_reg),
        !listconcat([prefix # "[" # start # ":" # end_reg # "]"],
                    RegSeqNames<last_reg, stride, size, prefix, next>.ret),
                    []);
}

// Generates list of dags for register tupless.
class RegSeqDags<RegisterClass RC, int last_reg, int stride, int size,
                int start = 0> {
  dag trunc_rc = (trunc RC,
                  !if(!and(!eq(stride, 1), !eq(start, 0)),
                      !add(!add(last_reg, 2), !mul(size, -1)),
                      !add(last_reg, 1)));
  list<dag> ret =
    !if(!lt(start, size),
        !listconcat([(add (decimate (shl trunc_rc, start), stride))],
                    RegSeqDags<RC, last_reg, stride, size, !add(start, 1)>.ret),
        []);
}

class PPURegisterTuples<list<SubRegIndex> Indices, RegisterClass RC,
                       int last_reg, int stride, int size, string prefix> :
  RegisterTuples<Indices,
                 RegSeqDags<RC, last_reg, stride, size>.ret,
                 RegSeqNames<last_reg, stride, size, prefix>.ret>;


//===----------------------------------------------------------------------===//
//  Declarations that describe the PPU register files
//===----------------------------------------------------------------------===//

let Namespace = "PPU" in {

class PPTReg <string n, bits<16> regIdx = 0> :
  Register<n>, DwarfRegNum<[!cast<int>(HWEncoding)]>
{

  // This is the not yet the complete register encoding. An additional
  // bit is set for VGPRs.
  let HWEncoding = regIdx;
}

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
/* def sub_32_rv : SubRegIndex<32>;
class PPUReg64<PPUReg32 subreg> : Register<""> {
  let HWEncoding{4-0} = subreg.HWEncoding{4-0};
  let SubRegs = [subreg];
  let SubRegIndices = [sub_32_rv];
  let AsmName = subreg.AsmName;
  let AltNames = subreg.AltNames;
}
*/

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
/*
  foreach Index = 0-31 in {
    def F#Index#_64 : PPUReg64<!cast<PPUReg32>("F"#Index#"_32")>,
      DwarfRegNum<[!add(Index, 32)]>;
  }
  */
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
/*
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
*/

// Vector registers
let RegAltNameIndices = [ABIRegAltName] in {
  // here we assuem Fload is not enable the DwarfRegNum is start from 32
  foreach Index = 0-15 in {
    def TPR#Index : PPUReg<Index, "t"#Index, ["t"#Index]>,
      DwarfRegNum<[!add(Index, 32)]>;
  }
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
}

def TPR : RegisterClass<"PPU", [nxv1i32], 32, (add
    (sequence "TPR%u", 0, 7),
    (sequence "TPR%u", 10, 15),
    (sequence "TPR%u", 8, 9)
)>;

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
def VCC : PPTReg<"vcc", 106>;
/*
def VCC : PPUReg<0, "vcc">;
def VCCR : RegisterClass<"PPU", [XLenVT], 32, (add VCC)> {
  let CopyCost = -1; // Don't allow copying
  let isAllocatable = 0; // Not allocatable
  let RegInfos = RegInfoByHwMode<
      [RV32,              RV64,              DefaultMode],
      [RegInfo<32,32,32>, RegInfo<64,64,64>, RegInfo<32,32,32>]>;
}
*/

// TMSK
def TMSK : PPTReg<"tmsk", 126>;
/*
def TMSK : PPUReg<0, "tmsk">;
def TMSKR : RegisterClass<"PPU", [XLenVT], 32, (add TMSK)> {
  let CopyCost = -1; // Don't allow copying
  let isAllocatable = 0; // Not allocatable
  let RegInfos = RegInfoByHwMode<
      [RV32,              RV64,              DefaultMode],
      [RegInfo<32,32,32>, RegInfo<64,64,64>, RegInfo<32,32,32>]>;
}
*/

// Pseudo-registers: Used as placeholders during isel and immediately
// replaced, never seeing the verifier.
def PRIVATE_RSRC_REG : PPTReg<"private_rsrc", 0>;
def FP_REG : PPTReg<"fp", 0>;
def SP_REG : PPTReg<"sp", 0>;
def SCRATCH_WAVE_OFFSET_REG : PPTReg<"scratch_wave_offset", 0>;

def SCRATCH_RSRC_REG : PPTReg<"scratch_rsr", 0>;



class FlatReg <Register lo, Register hi, bits<16> encoding> :
    RegisterWithSubRegs<"flat_scratch", [lo, hi]>,
    DwarfRegAlias<lo> {
  let Namespace = "PPU";
  let SubRegIndices = [sub0, sub1];
  let HWEncoding = encoding;
}
def FLAT_SCR_LO : PPTReg<"flat_scratch_lo", 0>;
def FLAT_SCR_HI : PPTReg<"flat_scratch_hi", 0>;
def FLAT_SCR : FlatReg<FLAT_SCR_LO, FLAT_SCR_HI, 0>;


// 32-bit real registers, for MC only.
// May be used with both 32-bit and 64-bit operands.
def SRC_VCCZ : PPTReg<"src_vccz", 251>;
def SRC_TMSKZ : PPTReg<"src_tmskz", 252>;
def SRC_SCC : PPTReg<"src_scc", 253>;
// 1-bit pseudo register, for codegen only.
// Should never be emitted.
def SCC : PPTReg<"scc", 0>;


def M0 : PPTReg<"m0", 124>;

def SPR_NULL : PPTReg<"null", 125>;

def SRC_SHARED_BASE : PPTReg<"src_shared_base", 235>;
def SRC_SHARED_LIMIT : PPTReg<"src_shared_limit", 236>;
def SRC_PRIVATE_BASE : PPTReg<"src_private_base", 237>;
def SRC_PRIVATE_LIMIT : PPTReg<"src_private_limit", 238>;
def SRC_POPS_EXITING_WAVE_ID : PPTReg<"src_pops_exiting_wave_id", 239>;

def LDS_DIRECT : PPTReg <"src_lds_direct", 254>;

// SPR registers
foreach Index = 0-105 in {
  def SPR#Index : PPTReg <"s"#Index, Index>;
}

// VPR registers
foreach Index = 0-255 in {
  def VPR#Index : PPTReg <"v"#Index, Index> {
    let HWEncoding{8} = 1;
  }
}

//===----------------------------------------------------------------------===//
//  Groupings using register classes and tuples
//===----------------------------------------------------------------------===//

def SCC_CLASS : RegisterClass<"PPU", [i1], 1, (add SCC)> {
  let CopyCost = -1;
  let isAllocatable = 0;
  let RegInfos = RegInfoByHwMode<
      [RV32,              RV64,              DefaultMode],
      [RegInfo<32,32,32>, RegInfo<64,64,64>, RegInfo<32,32,32>]>;
}

def M0_CLASS : RegisterClass<"PPU", [i32], 32, (add M0)> {
  let CopyCost = 1;
  let isAllocatable = 0;
  let RegInfos = RegInfoByHwMode<
      [RV32,              RV64,              DefaultMode],
      [RegInfo<32,32,32>, RegInfo<64,64,64>, RegInfo<32,32,32>]>;
}

// TODO: Do we need to set DwarfRegAlias on register tuples?

// SPR 32-bit registers
def SPR_32 : RegisterClass<"PPU", [i32, f32, i16, f16, v2i16, v2f16], 32,
    (add (sequence "SPR%u", 0, 105))> {

  // Give all SGPR classes higher priority than VGPR classes, because
  // we want to spill SGPRs to VGPRs.
  let AllocationPriority = 9;
}



// SPR 64-bit registers
def SPR_64Regs : PPURegisterTuples<getSubRegs<2>.ret, SPR_32, 105, 2, 2, "s">;

// SPR 96-bit registers. No operations use these, but for symmetry with 96-bit VGPRs.
def SPR_96Regs : PPURegisterTuples<getSubRegs<3>.ret, SPR_32, 105, 3, 3, "s">;

// SPR 128-bit registers
def SPR_128Regs : PPURegisterTuples<getSubRegs<4>.ret, SPR_32, 105, 4, 4, "s">;

// SPR 160-bit registers. No operations use these, but for symmetry with 160-bit VGPRs.
def SPR_160b : PPURegisterTuples<getSubRegs<5>.ret, SPR_32, 123, 4, 5, "s">;

// SPR 256-bit registers
def SPR_256b : PPURegisterTuples<getSubRegs<8>.ret, SPR_32, 123, 4, 8, "s">;

// SPR 512-bit registers
def SPR_512b : PPURegisterTuples<getSubRegs<16>.ret, SPR_32, 123, 4, 16, "s">;

// SPR 1024-bit registers
def SPR_1024b : PPURegisterTuples<getSubRegs<32>.ret, SPR_32, 123, 4, 32, "s">;

// VPR 32-bit registers
// i16/f16 only
def VPR_32 : RegisterClass<"PPU", [i32, f32, i16, f16, v2i16, v2f16, p2, p3, p5, p6], 32,
                            (add (sequence "VPR%u", 0, 255))> {
  let AllocationPriority = 1;
  let Size = 32;
}

// VGPR 64-bit registers
def VPR_64 : PPURegisterTuples<getSubRegs<2>.ret, VPR_32, 255, 1, 2, "v">;

// VGPR 96-bit registers
def VPR_96b : PPURegisterTuples<getSubRegs<3>.ret, VPR_32, 255, 1, 3, "v">;

// VGPR 128-bit registers
def VPR_128b : PPURegisterTuples<getSubRegs<4>.ret, VPR_32, 255, 1, 4, "v">;

// VGPR 160-bit registers
def VPR_160b : PPURegisterTuples<getSubRegs<5>.ret, VPR_32, 255, 1, 5, "v">;

// VGPR 256-bit registers
def VPR_256b : PPURegisterTuples<getSubRegs<8>.ret, VPR_32, 255, 1, 8, "v">;

// VGPR 512-bit registers
def VPR_512b : PPURegisterTuples<getSubRegs<16>.ret, VPR_32, 255, 1, 16, "v">;

// VGPR 1024-bit registers
// def VPR_1024b : PPURegisterTuples<getSubRegs<32>.ret, VPR_32, 255, 1, 32, "v">;


//===----------------------------------------------------------------------===//
//  Register classes used as source and destination
//===----------------------------------------------------------------------===//

def Pseudo_SReg_32 : RegisterClass<"PPU", [i32, f32, i16, f16, v2i16, v2f16], 32,
  (add FP_REG, SP_REG, SCRATCH_WAVE_OFFSET_REG)> {
  let isAllocatable = 0;
  let CopyCost = -1;
}

def Pseudo_SReg_64 : RegisterClass<"PPU", [v2i32, i64, f64], 32,
  (add PRIVATE_RSRC_REG)> {
  let isAllocatable = 0;
  let CopyCost = -1;
}

def LDS_DIRECT_CLASS : RegisterClass<"PPU", [i32, f32, i16, f16, v2i16, v2f16], 32,
  (add LDS_DIRECT)> {
  let isAllocatable = 0;
  let CopyCost = -1;
}


def SReg_32 : RegisterClass<"PPU", [i32, f32, i16, f16, i1, v2i16, v2f16], 32,
  (add SPR_32, VCC, SPR_NULL, SRC_VCCZ, SRC_TMSKZ, SRC_SCC, TMSK, M0_CLASS,
   FLAT_SCR_LO, FLAT_SCR_HI, SRC_SHARED_BASE, SRC_SHARED_LIMIT, SRC_PRIVATE_BASE,
   SRC_PRIVATE_LIMIT)> {
  // let CopyCost = 1;
  let AllocationPriority = 10;
  /*
  let RegInfos = RegInfoByHwMode<
      [RV32,              RV64,              DefaultMode],
      [RegInfo<32,32,32>, RegInfo<64,64,64>, RegInfo<32,32,32>]>;
      */
}

def SRegOrLds_32 : RegisterClass<"PPU", [i32, f32, i16, f16, v2i16, v2f16, i1], 32,
  (add SPR_32, VCC, SPR_NULL, SRC_VCCZ, SRC_TMSKZ, SRC_SCC,
   TMSK, M0_CLASS, LDS_DIRECT_CLASS)> {
  let isAllocatable = 0;
  /*
  let RegInfos = RegInfoByHwMode<
      [RV32,              RV64,              DefaultMode],
      [RegInfo<32,32,32>, RegInfo<64,64,64>, RegInfo<32,32,32>]>;
      */
}

def SPR_64 : RegisterClass<"PPU", [v2i32, i64, v2f32, f64, v4i16, v4f16], 32,
                            (add SPR_64Regs)> {
  let CopyCost = 1;
  let AllocationPriority = 11;
}

// CCR (call clobbered registers) SGPR 64-bit registers
def CCR_SPR_64 : RegisterClass<"PPU", SPR_64.RegTypes, 32,
                                (add (trunc SPR_64, 16))> {
  let CopyCost = SPR_64.CopyCost;
  let AllocationPriority = SPR_64.AllocationPriority;
}

def SReg_64 : RegisterClass<"PPU", [v2i32, i64, v2f32, f64, i1, v4i16, v4f16], 32,
  (add SPR_64, FLAT_SCR)> {
  let CopyCost = 1;
  let AllocationPriority = 13;
  /*
  let RegInfos = RegInfoByHwMode<
      [RV32,              RV64,              DefaultMode],
      [RegInfo<32,32,32>, RegInfo<64,64,64>, RegInfo<32,32,32>]>;
      */
}

def SReg_1 : RegisterClass<"PPU", [i1], 32,
  (add SReg_32, SReg_64, TMSK)> {
  let CopyCost = 1;
  let isAllocatable = 0;
  /*
  let RegInfos = RegInfoByHwMode<
      [RV32,              RV64,              DefaultMode],
      [RegInfo<32,32,32>, RegInfo<64,64,64>, RegInfo<32,32,32>]>;
      */

}

// Requires 2 s_mov_b64 to copy
// let RegInfo = RegInfoByHwMode<
//       [RV32,              RV64,              DefaultMode],
//      [RegInfo<32,32,32>, RegInfo<64,64,64>, RegInfo<32,32,32>]>
// in {

let CopyCost = 2 in {
// There are no 3-component scalar instructions, but this is needed
// for symmetry with VGPRs.

def SPR_96 : RegisterClass<"PPU", [v3i32, v3f32], 32,
  (add SPR_96Regs)> {
  let AllocationPriority = 14;
}

def SReg_96 : RegisterClass<"PPU", [v3i32, v3f32], 32,
  (add SPR_96)> {
  let AllocationPriority = 14;
}

def SPR_128 : RegisterClass<"PPU", [v4i32, v4f32, v2i64], 32,
  (add SPR_128Regs)> {
  let AllocationPriority = 15;
}

def SReg_128 : RegisterClass<"PPU", [v4i32, v4f32, v2i64, v2f64], 32,
  // (add SPR_128, TTMP_128)> {
  (add SPR_128)> {
  let AllocationPriority = 15;
}


} // End CopyCost = 2

// There are no 5-component scalar instructions, but this is needed
// for symmetry with VGPRs.
def SPR_160 : RegisterClass<"PPU", [v5i32, v5f32], 32,
                             (add SPR_160b)> {
  let AllocationPriority = 16;
}

def SReg_160 : RegisterClass<"PPU", [v5i32, v5f32], 32,
                             (add SPR_160)> {
  let AllocationPriority = 16;
}

def SPR_256 : RegisterClass<"PPU", [v8i32, v8f32], 32, (add SPR_256b)> {
  let AllocationPriority = 17;
}

def SReg_256 : RegisterClass<"PPU", [v8i32, v8f32], 32,
                             (add SPR_256)> {
  // Requires 4 s_mov_b64 to copy
  let CopyCost = 4;
  let AllocationPriority = 17;
}

def SPR_512 : RegisterClass<"PPU", [v16i32, v16f32], 32,
                             (add SPR_512b)> {
  let AllocationPriority = 18;
}

def SReg_512 : RegisterClass<"PPU", [v16i32, v16f32], 32,
                             (add SPR_512)> {
  // Requires 8 s_mov_b64 to copy
  let CopyCost = 8;
  let AllocationPriority = 18;
}

def VRegOrLds_32 : RegisterClass<"PPU", [i32, f32, i16, f16, v2i16, v2f16], 32,
                                 (add VPR_32, LDS_DIRECT_CLASS)> {
  let isAllocatable = 0;
}
/*
def SPR_1024 : RegisterClass<"PPU", [v32i32, v32f32], 32,
                              (add SPR_1024b)> {
  let AllocationPriority = 19;
}

def SReg_1024 : RegisterClass<"PPU", [v32i32, v32f32], 32,
                              (add SPR_1024)> {
  let CopyCost = 16;
  let AllocationPriority = 19;
}
*/

def VReg_32 : RegisterClass<"PPU", [i32, f32, i1, v2i16, v2f16], 32,
  (add VPR_32)> {
  let CopyCost = 1;
  let AllocationPriority = 13;
}

// Register class for all vector registers (VGPRs + Interploation Registers)
def VReg_64 : RegisterClass<"PPU", [i64, f64, v2i32, v2f32, v4f16, v4i16, p0, p1, p4], 32,
                            (add VPR_64)> {
  let Size = 64;

  // Requires 2 v_mov_b32 to copy
  let CopyCost = 2;
  let AllocationPriority = 2;
}

def VReg_96 : RegisterClass<"PPU", [v3i32, v3f32], 32, (add VPR_96b)> {
  let Size = 96;

  // Requires 3 v_mov_b32 to copy
  let CopyCost = 3;
  let AllocationPriority = 3;
}


def VReg_128 : RegisterClass<"PPU", [v4i32, v4f32, v2i64, v2f64], 32,
                             (add VPR_128b)> {
  let Size = 128;

  // Requires 4 v_mov_b32 to copy
  let CopyCost = 4;
  let AllocationPriority = 4;
}
def VReg_160 : RegisterClass<"PPU", [v5i32, v5f32], 32,
                             (add VPR_160b)> {
  let Size = 160;

  // Requires 5 v_mov_b32 to copy
  let CopyCost = 5;
  let AllocationPriority = 5;
}

def VReg_256 : RegisterClass<"PPU", [v8i32, v8f32], 32,
                             (add VPR_256b)> {
  let Size = 256;
  let CopyCost = 8;
  let AllocationPriority = 6;
}

def VReg_512 : RegisterClass<"PPU", [v16i32, v16f32], 32,
                             (add VPR_512b)> {
  let Size = 512;
  let CopyCost = 16;
  let AllocationPriority = 7;
}

/*
def VReg_1024 : RegisterClass<"PPU", [v32i32, v32f32], 32,
                              (add VPR_1024b)> {
  let Size = 1024;
  let CopyCost = 32;
  let AllocationPriority = 8;
}
*/

def VReg_1 : RegisterClass<"PPU", [i1], 32, (add VPR_32)> {
  let Size = 32;
}

def VS_32 : RegisterClass<"PPU", [i32, f32, i16, f16, v2i16, v2f16], 32,
                          (add VPR_32, SReg_32, LDS_DIRECT_CLASS)> {
  let isAllocatable = 0;
}

def VS_64 : RegisterClass<"PPU", [i64, f64], 32, (add VReg_64, SReg_64)> {
  let isAllocatable = 0;
}
// } RegInfo

//===----------------------------------------------------------------------===//
//  Register operands
//===----------------------------------------------------------------------===//
class RegImmMatcher<string name> : AsmOperandClass {
  let Name = name;
  let RenderMethod = "addRegOrImmOperands";
}

multiclass PPURegOperand <string rc, string MatchName, string opType> {
  let OperandNamespace = "PPU" in {
    def _b8 : RegisterOperand<!cast<RegisterClass>(rc#"_32")> {
      let OperandType = opType#"_INT8";
      let ParserMatchClass = RegImmMatcher<MatchName#"B8">;
      let DecoderMethod = "decodeOperand_VSrc8";
    }
    def _b16 : RegisterOperand<!cast<RegisterClass>(rc#"_32")> {
      let OperandType = opType#"_INT16";
      let ParserMatchClass = RegImmMatcher<MatchName#"B16">;
      let DecoderMethod = "decodeOperand_VSrc16";
    }

    def _f16 : RegisterOperand<!cast<RegisterClass>(rc#"_32")> {
      let OperandType = opType#"_FP16";
      let ParserMatchClass = RegImmMatcher<MatchName#"F16">;
      let DecoderMethod = "decodeOperand_VSrc_16";
    }

    def _b32 : RegisterOperand<!cast<RegisterClass>(rc#"_32")> {
      let OperandType = opType#"_INT32";
      let ParserMatchClass = RegImmMatcher<MatchName#"B32">;
      // let DecoderMethod = "decodeOperand_" # rc # rc_suffix;
    }

    def _f32 : RegisterOperand<!cast<RegisterClass>(rc#"_32")> {
      let OperandType = opType#"_FP32";
      let ParserMatchClass = RegImmMatcher<MatchName#"F32">;
      // let DecoderMethod = "decodeOperand_" # rc # rc_suffix;
    }

    def _b64 : RegisterOperand<!cast<RegisterClass>(rc#"_64")> {
      let OperandType = opType#"_INT64";
      let ParserMatchClass = RegImmMatcher<MatchName#"B64">;
    }

    def _f64 : RegisterOperand<!cast<RegisterClass>(rc#"_64")> {
      let OperandType = opType#"_FP64";
      let ParserMatchClass = RegImmMatcher<MatchName#"F64">;
    }

    def _v2b16 : RegisterOperand<!cast<RegisterClass>(rc#"_32")> {
      let OperandType = opType#"_V2INT16";
      let ParserMatchClass = RegImmMatcher<MatchName#"V2B16">;
      let DecoderMethod = "decodeOperand_VSrcV216";
    }

    def _v2f16 : RegisterOperand<!cast<RegisterClass>(rc#"_32")> {
      let OperandType = opType#"_V2FP16";
      let ParserMatchClass = RegImmMatcher<MatchName#"V2F16">;
      let DecoderMethod = "decodeOperand_VSrcV216";
    }

    def _v4f16 : RegisterOperand<!cast<RegisterClass>(rc#"_64")> {
      let OperandType = opType#"_V4FP16";
      let ParserMatchClass = RegImmMatcher<MatchName#"V4F16">;
      let DecoderMethod = "decodeOperand_VSrcV416";
    }

    def _v4b8 : RegisterOperand<!cast<RegisterClass>(rc#"_32")> {
      let OperandType = opType#"_V4INT8";
      let ParserMatchClass = RegImmMatcher<MatchName#"V4B8">;
      let DecoderMethod = "decodeOperand_VSrcV48";
    }

  }
}

// FIXME: 64-bit sources can sometimes use 32-bit constants.
multiclass RegImmOperand <string rc, string MatchName>
  : PPURegOperand<rc, MatchName, "OPERAND_REG_IMM">;

multiclass RegInlineOperand <string rc, string MatchName>
  : PPURegOperand<rc, MatchName, "OPERAND_REG_INLINE_C">;



//===----------------------------------------------------------------------===//
//  SSrc_* Operands with an SPR or a 32-bit immediate
defm SSrc : RegImmOperand<"SReg", "SSrc">;

def SSrcOrLds_b32 : RegisterOperand<SRegOrLds_32> {
  let OperandNamespace = "PPU";
  let OperandType = "OPERAND_REG_IMM_INT32";
  let ParserMatchClass = RegImmMatcher<"SSrcOrLdsB32">;
}


//===----------------------------------------------------------------------===//
//  SCSrc_* Operands with an SGPR or a inline constant

defm SCSrc : RegInlineOperand<"SReg", "SCSrc"> ;

//===----------------------------------------------------------------------===//
//  VSrc_* Operands with an SPR, VPR or a 32-bit immediate

defm VSrc : RegImmOperand<"VS", "VSrc">;

def VSrc_64 : RegisterOperand<VReg_64> {
  let DecoderMethod = "DecodeVS_64RegisterClass";
}

def VSrc_128 : RegisterOperand<VReg_128> {
  let DecoderMethod = "DecodeVS_128RegisterClass";
}

// This is for operands with the enum(9), VSrc encoding restriction,
// but only allows VGPRs.
def VRegSrc_32 : RegisterOperand<VPR_32> {
  //let ParserMatchClass = RegImmMatcher<"VRegSrc32">;
  let DecoderMethod = "DecodeVS_32RegisterClass";
}

//===----------------------------------------------------------------------===//
//  VCSrc_* Operands with an SGPR, VGPR or an inline constant
//===----------------------------------------------------------------------===//

defm VCSrc : RegInlineOperand<"VS", "VCSrc">;

//===----------------------------------------------------------------------===//
//  VCSrc_* Operands with a VGPR or an inline constant
//===----------------------------------------------------------------------===//
// defm VISrc : RegInlineOperand<"VPR", "VISrc", "_32">;
/*
let OperandNamespace = "PPU" in {
  def VCSrc_b32 : RegisterOperand<!cast<RegisterClass>("VReg_32")> {
    let OperandType = "OPERAND_REG_INLINE_C_INT32";
    let ParserMatchClass = RegImmMatcher<"VCSrcB32">;
    let DecoderMethod = "DecodeVS_32RegisterClass";
  }
}
*/

//===----------------------------------------------------------------------===//
//  Special Operands for corner cases
//===----------------------------------------------------------------------===//
