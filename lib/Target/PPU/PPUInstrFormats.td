//===-- PPUInstrFormats.td - PPU Instruction Formats ---*- tablegen -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
//
//  These instruction format definitions are structured to match the
//  description in the PPU User-Level ISA specification as closely as
//  possible. For instance, the specification describes instructions with the
//  MSB (31st bit) on the left and the LSB (0th bit) on the right. This is
//  reflected in the order of parameters to each instruction class.
//
//  One area of divergence is in the description of immediates. The
//  specification describes immediate encoding in terms of bit-slicing
//  operations on the logical value represented. The immediate argument to
//  these instruction formats instead represents the bit sequence that will be
//  inserted into the instruction. e.g. although JAL's immediate is logically
//  a 21-bit value (where the LSB is always zero), we describe it as an imm20
//  to match how it is encoded.
//
//===----------------------------------------------------------------------===//

// Format specifies the encoding used by the instruction. This is used by
// PPUMCCodeEmitter to determine which form of fixup to use. These
// definitions must be kept in-sync with PPUBaseInfo.h.
class InstFormat<bits<5> val> {
  bits<5> Value = val;
}
def InstFormatPseudo : InstFormat<0>;
def InstFormatR      : InstFormat<1>;
def InstFormatR4     : InstFormat<2>;
def InstFormatI      : InstFormat<3>;
def InstFormatS      : InstFormat<4>;
def InstFormatB      : InstFormat<5>;
def InstFormatU      : InstFormat<6>;
def InstFormatJ      : InstFormat<7>;
def InstFormatCR     : InstFormat<8>;
def InstFormatCI     : InstFormat<9>;
def InstFormatCSS    : InstFormat<10>;
def InstFormatCIW    : InstFormat<11>;
def InstFormatCL     : InstFormat<12>;
def InstFormatCS     : InstFormat<13>;
def InstFormatCA     : InstFormat<14>;
def InstFormatCB     : InstFormat<15>;
def InstFormatCJ     : InstFormat<16>;
def InstFormatOther  : InstFormat<17>;

// The following opcode names match those given in Table 19.1 in the
// PPU User-level ISA specification ("PPU base opcode map").
class PPUOpcode<bits<7> val> {
  bits<7> Value = val;
}
def OPC_LOAD      : PPUOpcode<0b0000011>;
def OPC_LOAD_FP   : PPUOpcode<0b0000111>;
def OPC_MISC_MEM  : PPUOpcode<0b0001111>;
def OPC_OP_IMM    : PPUOpcode<0b0010011>;
def OPC_AUIPC     : PPUOpcode<0b0010111>;
def OPC_OP_IMM_32 : PPUOpcode<0b0011011>;
def OPC_STORE     : PPUOpcode<0b0100011>;
def OPC_STORE_FP  : PPUOpcode<0b0100111>;
def OPC_AMO       : PPUOpcode<0b0101111>;
def OPC_OP        : PPUOpcode<0b0110011>;
def OPC_LUI       : PPUOpcode<0b0110111>;
def OPC_OP_32     : PPUOpcode<0b0111011>;
def OPC_MADD      : PPUOpcode<0b1000011>;
def OPC_MSUB      : PPUOpcode<0b1000111>;
def OPC_NMSUB     : PPUOpcode<0b1001011>;
def OPC_NMADD     : PPUOpcode<0b1001111>;
def OPC_OP_FP     : PPUOpcode<0b1010011>;
def OPC_OP_V      : PPUOpcode<0b1010111>; // TODO copied from rvv
def OPC_OP_T      : PPUOpcode<0b1011011>; // TODO copied from rvv
def OPC_BRANCH    : PPUOpcode<0b1100011>;
def OPC_JALR      : PPUOpcode<0b1100111>;
def OPC_JAL       : PPUOpcode<0b1101111>;
def OPC_SYSTEM    : PPUOpcode<0b1110011>;


class RVInst<dag outs, dag ins, string opcodestr, string argstr,
             list<dag> pattern, InstFormat format>
    // : PPUInst<outs, ins, opcodestr # "\t" # argstr, pattern>, PPUPredicateControl {
    : Instruction, PredicateControl {
  let SubtargetPredicate = IsNotPPT;
  list<Predicate> OtherPredicates = [];

  field bits<32> Inst;
  // SoftFail is a field the disassembler can use to provide a way for
  // instructions to not match without killing the whole decode process. It is
  // mainly used for ARM, but Tablegen expects this field to exist or it fails
  // to build the decode table.
  field bits<32> SoftFail = 0;
  let Size = 4;

  bits<7> Opcode = 0;

  let Inst{6-0} = Opcode;

  let Namespace = "PPU";

  dag OutOperandList = outs;
  dag InOperandList = ins;
  let AsmString = opcodestr # "\t" # argstr;
  let Pattern = pattern;

  let TSFlags{4-0} = format.Value;
}


// def isPPT : Predicate<"Subtarget->getGeneration() "
//                      ">= PPUSubtarget::PPT">,
//            AssemblerPredicate<"FeaturePPT">;


class Enc32 {
  field bits<32> Inst;
  int Size = 4;
}

class Enc64 {
  field bits<64> Inst;
  int Size = 8;
}


class PPTInst <dag outs, dag ins, string opcodestr = "", string argstr = "",
  list<dag> pattern = []> : PPUBaseInst<outs, ins, opcodestr # " " # argstr, pattern>, PredicateControl {
  let SubtargetPredicate = IsPPT;
  field bit isRegisterLoad = 0;
  field bit isRegisterStore = 0;

  list<Predicate> OtherPredicates = [];

  // Low bits - basic encoding information.
  field bit SALU = 0;
  field bit VALU = 0;

  // SALU instruction formats.
  field bit SOP1 = 0;
  field bit SOP2 = 0;
  field bit SOPC = 0;
  field bit SOPK = 0;
  field bit SOPP = 0;

  // VALU instruction formats.
  field bit VOP1 = 0;
  field bit VOP2 = 0;
  field bit VOPC = 0;
  field bit VOP3 = 0;
  field bit VOP3P = 0;
  field bit DPP = 0;

  // Memory instruction formats.
  field bit MUBUF = 0;
  field bit MTBUF = 0;
  field bit SMRD = 0;
  field bit MIMG = 0;
  field bit EXP = 0;
  field bit FLAT = 0;
  field bit DS = 0;


  // Pseudo instruction formats.
  field bit VPRSpill = 0;
  field bit SPRSpill = 0;
  field bit GPRSpill = 0;
  field bit TPRSpill = 0;

  // High bits - other information.
  field bit VM_CNT = 0;
  field bit EXP_CNT = 0;
  field bit LGKM_CNT = 0;

  // Whether WQM _must_ be enabled for this instruction.
  field bit WQM = 0;
  // Whether WQM _must_ be disabled for this instruction.
  field bit DisableWQM = 0;

  // Most sopk treat the immediate as a signed 16-bit, however some
  // use it as unsigned.
  field bit SOPKZext = 0;

  // This is an s_store_dword* instruction that requires a cache flush
  // on wave termination. It is necessary to distinguish from mayStore
  // SMEM instructions like the cache flush ones.
  field bit ScalarStore = 0;

  // Whether the operands can be ignored when computing the
  // instruction size.
  field bit FixedSize = 0;

  // This bit tells the assembler to use the 32-bit encoding in case it
  // is unable to infer the encoding from the operands.
  field bit VOPAsmPrefer32Bit = 0;

  // Is it possible for this instruction to be atomic?
  field bit maybeAtomic = 0;

  // This bit indicates that this has a floating point result type, so
  // the clamp modifier has floating point semantics.
  field bit FPClamp = 0;

  // This bit indicates that instruction may support integer clamping
  // which depends on GPU features.
  field bit IntClamp = 0;

  // This field indicates that the clamp applies to the low component
  // of a packed output register.
  field bit ClampLo = 0;

  // This field indicates that the clamp applies to the high component
  // of a packed output register.
  field bit ClampHi = 0;

  // This bit indicates that this is a packed VOP3P instruction
  field bit IsPacked = 0;

  // This bit indicates that this is a D16 buffer instruction.
  field bit D16Buf = 0;

  // This field indicates that FLAT instruction accesses FLAT_GLBL or
  // FLAT_SCRATCH segment. Must be 0 for non-FLAT instructions.
  field bit IsNonFlatSeg = 0;

  // This bit indicates that this uses the floating point double precision
  // rounding mode flags
  field bit FPDPRounding = 0;

  // Instruction is FP atomic.
  field bit FPAtomic = 0;

  // This bit indicates that this is one of MFMA instructions.
  field bit IsMAI = 0;



  let Namespace = "PPU";
  let OutOperandList = outs;
  let InOperandList = ins;
  let AsmString = opcodestr # "\t" # argstr;
  let Pattern = pattern;
  let Itinerary = NullALU;

  // SoftFail is a field the disassembler can use to provide a way for
  // instructions to not match without killing the whole decode process. It is
  // mainly used for ARM, but Tablegen expects this field to exist or it fails
  // to build the decode table.
  field bits<64> SoftFail = 0;

  let DecoderNamespace = Namespace;

  // These need to be kept in sync with the enum in PPUInstrFlags.
  let TSFlags{0} = SALU;
  let TSFlags{1} = VALU;
  let TSFlags{2} = SOP1;
  let TSFlags{3} = SOP2;
  let TSFlags{4} = SOPC;
  let TSFlags{5} = SOPK;
  let TSFlags{6} = SOPP;
  let TSFlags{7} = VOP1;
  let TSFlags{8} = VOP2;
  let TSFlags{9} = VOPC;
  let TSFlags{10} = VOP3;
  let TSFlags{12} = VOP3P;

  let TSFlags{16} = MUBUF;
  let TSFlags{17} = MTBUF;
  let TSFlags{18} = SMRD;
  let TSFlags{19} = MIMG;
  let TSFlags{20} = EXP;
  let TSFlags{21} = FLAT;
  let TSFlags{22} = DS;


  let TSFlags{23} = VPRSpill;
  let TSFlags{24} = SPRSpill;
  let TSFlags{25} = GPRSpill;
  let TSFlags{26} = TPRSpill;

  let TSFlags{32} = VM_CNT;
  let TSFlags{33} = EXP_CNT;
  let TSFlags{34} = LGKM_CNT;

  let TSFlags{38} = SOPKZext;
  let TSFlags{39} = ScalarStore;
  let TSFlags{40} = FixedSize;
  let TSFlags{41} = VOPAsmPrefer32Bit;

  let TSFlags{43} = maybeAtomic;

  let TSFlags{45} = FPClamp;
  let TSFlags{46} = IntClamp;
  let TSFlags{47} = ClampLo;
  let TSFlags{48} = ClampHi;

  let TSFlags{49} = IsPacked;
  let TSFlags{50} = D16Buf;
  let TSFlags{51} = IsNonFlatSeg;
  let TSFlags{52} = FPDPRounding;
  let TSFlags{53} = FPAtomic;
  let TSFlags{54} = IsMAI;

  let TSFlags{63} = isRegisterLoad;
  let TSFlags{62} = isRegisterStore;

  // let SchedRW = [Write32Bit];
  // FIXME schi why need this
  let mayLoad = 0;
  let mayStore = 0;
  let hasSideEffects = 0;
}


// Pseudo instructions
class PPTPseudo<dag outs, dag ins, list<dag> pattern = [], string opcodestr = "", string argstr = "">
    : PPTInst<outs, ins, opcodestr, argstr, pattern> {
  let isPseudo = 1;
  let isCodeGenOnly = 1;
  /*
  let hasSideEffects = 0;
  let mayLoad = 0;
  let mayStore = 0;
  */
}


// Pseudo instructions
class Pseudo<dag outs, dag ins, list<dag> pattern, string opcodestr = "", string argstr = "">
    : RVInst<outs, ins, opcodestr, argstr, pattern, InstFormatPseudo> {
  let isPseudo = 1;
  let isCodeGenOnly = 1;
}

// Pseudo load instructions.
class PseudoLoad<string opcodestr, RegisterClass rdty = GPR>
    : Pseudo<(outs rdty:$rd), (ins bare_symbol:$addr), [], opcodestr, "$rd, $addr"> {
  let hasSideEffects = 0;
  let mayLoad = 1;
  let mayStore = 0;
  let isCodeGenOnly = 0;
  let isAsmParserOnly = 1;
}

class PseudoFloatLoad<string opcodestr, RegisterClass rdty = GPR>
    : Pseudo<(outs rdty:$rd, GPR:$tmp), (ins bare_symbol:$addr), [], opcodestr, "$rd, $addr, $tmp"> {
  let hasSideEffects = 0;
  let mayLoad = 1;
  let mayStore = 0;
  let isCodeGenOnly = 0;
  let isAsmParserOnly = 1;
}

// Pseudo store instructions.
class PseudoStore<string opcodestr, RegisterClass rsty = GPR>
    : Pseudo<(outs rsty:$rs, GPR:$tmp), (ins bare_symbol:$addr), [], opcodestr, "$rs, $addr, $tmp"> {
  let hasSideEffects = 0;
  let mayLoad = 0;
  let mayStore = 1;
  let isCodeGenOnly = 0;
  let isAsmParserOnly = 1;
}

// Instruction formats are listed in the order they appear in the PPU
// instruction set manual (R, I, S, B, U, J) with sub-formats (e.g. RVInstR4,
// RVInstRAtomic) sorted alphabetically.

class RVInstR<bits<7> funct7, bits<3> funct3, PPUOpcode opcode, dag outs,
              dag ins, string opcodestr, string argstr>
    : RVInst<outs, ins, opcodestr, argstr, [], InstFormatR> {
  bits<5> rs2;
  bits<5> rs1;
  bits<5> rd;

  let Inst{31-25} = funct7;
  let Inst{24-20} = rs2;
  let Inst{19-15} = rs1;
  let Inst{14-12} = funct3;
  let Inst{11-7} = rd;
  let Opcode = opcode.Value;
}

class RVInstR4<bits<2> funct2, PPUOpcode opcode, dag outs, dag ins,
               string opcodestr, string argstr>
    : RVInst<outs, ins, opcodestr, argstr, [], InstFormatR4> {
  bits<5> rs3;
  bits<5> rs2;
  bits<5> rs1;
  bits<3> funct3;
  bits<5> rd;

  let Inst{31-27} = rs3;
  let Inst{26-25} = funct2;
  let Inst{24-20} = rs2;
  let Inst{19-15} = rs1;
  let Inst{14-12} = funct3;
  let Inst{11-7} = rd;
  let Opcode = opcode.Value;
}

class RVInstRAtomic<bits<5> funct5, bit aq, bit rl, bits<3> funct3,
                    PPUOpcode opcode, dag outs, dag ins, string opcodestr,
                    string argstr>
    : RVInst<outs, ins, opcodestr, argstr, [], InstFormatR> {
  bits<5> rs2;
  bits<5> rs1;
  bits<5> rd;

  let Inst{31-27} = funct5;
  let Inst{26} = aq;
  let Inst{25} = rl;
  let Inst{24-20} = rs2;
  let Inst{19-15} = rs1;
  let Inst{14-12} = funct3;
  let Inst{11-7} = rd;
  let Opcode = opcode.Value;
}

class RVInstRFrm<bits<7> funct7, PPUOpcode opcode, dag outs, dag ins,
                 string opcodestr, string argstr>
    : RVInst<outs, ins, opcodestr, argstr, [], InstFormatR> {
  bits<5> rs2;
  bits<5> rs1;
  bits<3> funct3;
  bits<5> rd;

  let Inst{31-25} = funct7;
  let Inst{24-20} = rs2;
  let Inst{19-15} = rs1;
  let Inst{14-12} = funct3;
  let Inst{11-7} = rd;
  let Opcode = opcode.Value;
}

class RVInstI<bits<3> funct3, PPUOpcode opcode, dag outs, dag ins,
              string opcodestr, string argstr>
    : RVInst<outs, ins, opcodestr, argstr, [], InstFormatI> {
  bits<12> imm12;
  bits<5> rs1;
  bits<5> rd;

  let Inst{31-20} = imm12;
  let Inst{19-15} = rs1;
  let Inst{14-12} = funct3;
  let Inst{11-7} = rd;
  let Opcode = opcode.Value;
}

class RVInstIShift<bit arithshift, bits<3> funct3, PPUOpcode opcode,
                   dag outs, dag ins, string opcodestr, string argstr>
    : RVInst<outs, ins, opcodestr, argstr, [], InstFormatI> {
  bits<6> shamt;
  bits<5> rs1;
  bits<5> rd;

  let Inst{31} = 0;
  let Inst{30} = arithshift;
  let Inst{29-26} = 0;
  let Inst{25-20} = shamt;
  let Inst{19-15} = rs1;
  let Inst{14-12} = funct3;
  let Inst{11-7} = rd;
  let Opcode = opcode.Value;
}

class RVInstIShiftW<bit arithshift, bits<3> funct3, PPUOpcode opcode,
                    dag outs, dag ins, string opcodestr, string argstr>
    : RVInst<outs, ins, opcodestr, argstr, [], InstFormatI> {
  bits<5> shamt;
  bits<5> rs1;
  bits<5> rd;

  let Inst{31} = 0;
  let Inst{30} = arithshift;
  let Inst{29-25} = 0;
  let Inst{24-20} = shamt;
  let Inst{19-15} = rs1;
  let Inst{14-12} = funct3;
  let Inst{11-7} = rd;
  let Opcode = opcode.Value;
}

class RVInstS<bits<3> funct3, PPUOpcode opcode, dag outs, dag ins,
              string opcodestr, string argstr>
    : RVInst<outs, ins, opcodestr, argstr, [], InstFormatS> {
  bits<12> imm12;
  bits<5> rs2;
  bits<5> rs1;

  let Inst{31-25} = imm12{11-5};
  let Inst{24-20} = rs2;
  let Inst{19-15} = rs1;
  let Inst{14-12} = funct3;
  let Inst{11-7} = imm12{4-0};
  let Opcode = opcode.Value;
}

class RVInstB<bits<3> funct3, PPUOpcode opcode, dag outs, dag ins,
              string opcodestr, string argstr>
    : RVInst<outs, ins, opcodestr, argstr, [], InstFormatB> {
  bits<12> imm12;
  bits<5> rs2;
  bits<5> rs1;

  let Inst{31} = imm12{11};
  let Inst{30-25} = imm12{9-4};
  let Inst{24-20} = rs2;
  let Inst{19-15} = rs1;
  let Inst{14-12} = funct3;
  let Inst{11-8} = imm12{3-0};
  let Inst{7} = imm12{10};
  let Opcode = opcode.Value;
}

class RVInstU<PPUOpcode opcode, dag outs, dag ins, string opcodestr,
              string argstr>
    : RVInst<outs, ins, opcodestr, argstr, [], InstFormatU> {
  bits<20> imm20;
  bits<5> rd;

  let Inst{31-12} = imm20;
  let Inst{11-7} = rd;
  let Opcode = opcode.Value;
}

class RVInstJ<PPUOpcode opcode, dag outs, dag ins, string opcodestr,
              string argstr>
    : RVInst<outs, ins, opcodestr, argstr, [], InstFormatJ> {
  bits<20> imm20;
  bits<5> rd;

  let Inst{31} = imm20{19};
  let Inst{30-21} = imm20{9-0};
  let Inst{20} = imm20{10};
  let Inst{19-12} = imm20{18-11};
  let Inst{11-7} = rd;
  let Opcode = opcode.Value;
}
