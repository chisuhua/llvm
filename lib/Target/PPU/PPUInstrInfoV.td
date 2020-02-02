//===-- PPUInstrInfoV.td - RISC-V 'V' instructions -------*- tablegen -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file describes the RISC-V instructions from the standard 'V',
// Vector instruction set extension.
//
// BOTH THE VECTOR ISA SPEC AND THIS CODE ARE EXTREMELY WIP
//
//===----------------------------------------------------------------------===//

include "PPUInstrFormatsV.td"

//===----------------------------------------------------------------------===//
// RISC-V vector extension specific DAG Nodes.
//===----------------------------------------------------------------------===//


def SDT_PPUSETVL : SDTypeProfile<2, 1, [SDTCisVT<0, i32>,
                                          SDTCisSameAs<0, 1>,
                                          SDTCisSameAs<0, 2>]>;

def PPUSETVL : SDNode<"PPUISD::SETVL", SDT_PPUSETVL>;

def SDT_PPUBROADCAST : SDTypeProfile<1, 1, [SDTCisVT<0, nxv1i32>,
                                              SDTCisVT<1, i32>]>;

def PPUBROADCAST : SDNode<"PPUISD::BROADCAST", SDT_PPUBROADCAST>;


def AMDGPUIfOp : SDTypeProfile<1, 2,
  [SDTCisVT<0, i1>, SDTCisVT<1, i1>, SDTCisVT<2, OtherVT>] >;

def AMDGPUElseOp : SDTypeProfile<1, 2,
  [SDTCisVT<0, i1>, SDTCisVT<1, i1>, SDTCisVT<2, OtherVT>] >;

def AMDGPULoopOp : SDTypeProfile<0, 2,
  [SDTCisVT<0, i1>, SDTCisVT<1, OtherVT>] >;

def AMDGPUIfBreakOp : SDTypeProfile<1, 2,
  [SDTCisVT<0, i1>, SDTCisVT<1, i1>, SDTCisVT<2, i1>] >;


def AMDGPUif : SDNode<"PPUISD::IF", AMDGPUIfOp, [SDNPHasChain]>;
def AMDGPUelse : SDNode<"PPUISD::ELSE", AMDGPUElseOp, [SDNPHasChain]>;
def AMDGPUloop : SDNode<"PPUISD::LOOP", AMDGPULoopOp, [SDNPHasChain]>;

//===----------------------------------------------------------------------===//
// Operand and SDNode transformation definitions.
//===----------------------------------------------------------------------===//

def simm3 : Operand<XLenVT>, ImmLeaf<XLenVT, [{return isInt<3>(Imm);}]> {
  let ParserMatchClass = SImmAsmOperand<3>;
  let EncoderMethod = "getImmOpValue";
  let DecoderMethod = "decodeSImmOperand<3>";
}

def simm8 : Operand<XLenVT>, ImmLeaf<XLenVT, [{return isInt<8>(Imm);}]> {
  let ParserMatchClass = SImmAsmOperand<8>;
  let EncoderMethod = "getImmOpValue";
  let DecoderMethod = "decodeSImmOperand<8>";
}

//===----------------------------------------------------------------------===//
// Instruction class templates
//===----------------------------------------------------------------------===//

let hasSideEffects = 0, mayLoad = 0, mayStore = 0 in
class VALU_rr<bits<7> funct7, bits<1> funct1, string opcodestr>
    : RVInstVrVr<funct7, funct1, RVV_Unmasked, OPC_OP_V,
               (outs TPR:$vd), (ins TPR:$vs1, TPR:$vs2, VLR:$vl),
               opcodestr, "$vd, $vs1, $vs2">
{
  let Uses = [VCFG];
}

let hasSideEffects = 0, mayLoad = 0, mayStore = 0 in
class VALU_VrGpr<bits<12> funct12, bits<1> funct1, string opcodestr>
    : RVInstVrGpr<funct12, funct1, RVV_Unmasked, OPC_OP_V,
               (outs GPR:$rd), (ins TPR:$vs1, VLR:$vl),
               opcodestr, "$rd, $vs1">
{
  let Uses = [VCFG];
}

let hasSideEffects = 0, mayLoad = 0, mayStore = 0 in
class VALU_vvi<bits<4> funct4, bits<1> funct1, string opcodestr>
    : RVInstVI<funct4, funct1, RVV_Unmasked, OPC_OP_V,
               (outs TPR:$vd), (ins TPR:$vs1, simm8:$imm8, VLR:$vl),
               opcodestr, "$vd, $vs1, $imm8">
{
  let Uses = [VCFG];
}

let hasSideEffects = 0, mayLoad = 1, mayStore = 0 in
class VLoad_UnitStride<bits<2> funct2, bits<3> funct3, string opcodestr>
    : RVInstVLoad<funct2, funct3, RVV_Unmasked, OPC_LOAD_FP,
                  (outs TPR:$vd), (ins GPR:$rs1, VLR:$vl, simm3:$imm3),
                  opcodestr, "$vd, ${imm3}(${rs1})">
{
  let Uses = [VCFG];
}

let hasSideEffects = 0, mayLoad = 0, mayStore = 1 in
class VStore_UnitStride<bits<3> funct3, bits<2> funct2, string opcodestr>
    : RVInstVStore<funct3, funct2, RVV_Unmasked, OPC_STORE_FP,
                   (outs), (ins GPR:$rs1, TPR:$vs3, VLR:$vl, simm3:$imm3),
                   opcodestr, "$vs3, ${imm3}(${rs1})">
{
  let Uses = [VCFG];
}
/*
// FIXME we need change outs GPR to out TPR
let hasSideEffects = 0, mayLoad = 0, mayStore = 0 in
class VALU_tsi<bits<4> funct4, bits<1> funct1, string opcodestr>
    : RVInstTSI<funct4, funct1, RVV_Unmasked, OPC_OP_T,
               (outs GPR:$vd), (ins GPR:$rs1, simm8:$imm8),
               opcodestr, "$vd, ${imm8}(${rs1})">
{
}
*/

//===----------------------------------------------------------------------===//
// Instructions
//===----------------------------------------------------------------------===//

let Predicates = [HasStdExtV] in {

def VADD : VALU_rr<0b1001000, 0b1, "vadd">;

def VMOV : VALU_rr<0b1001010, 0b1, "vmov">;  // tententive
def SMOV : VALU_rr<0b1001011, 0b1, "smov">;  // tententive

def VSUB : VALU_rr<0b1001001, 0b1, "vsub">;

def VADDI : VALU_vvi<0b0000, 0b1, "vaddi">;

//Instruction encoding table: https://github.com/ppu/ppu-v-spec/blob/master/inst-table.adoc

def VMUL : VALU_rr<0b1000100, 0b1, "vmul">;

def VAND : VALU_rr<0b1001110, 0b1, "vand">;

def VOR : VALU_rr<0b1001101, 0b1, "vor">;

def VXOR : VALU_rr<0b1100000, 0b1, "vxor">;

def VLW : VLoad_UnitStride<0b00, 0b110, "vlw">;

def VSW : VStore_UnitStride<0b110, 0b00, "vsw">;

def VMPOPC : VALU_VrGpr<0b101011100001, 0b1, "vmpop">;
def VMFIRST : VALU_VrGpr<0b101011100000, 0b1, "vmfirst">;

def VREADLANE : VALU_VrGpr<0b101011100010, 0b1, "vreadlane">;
def VREAD1stLANE : VALU_VrGpr<0b101011100011, 0b1, "vread1stlane">;
def VWRITELANE : VALU_VrGpr<0b101011100100, 0b1, "vwritelane">;

/*
// intrinsic
def NVVM_TID_x : VALU_tsi<0b0000, 0b1, "tid_x"> {
  let simm8 = 0;
  let rs1 = 0; // FIXME
}
*/

let hasSideEffects = 0, mayLoad = 0, mayStore = 0 in
def VSETVL: RVInst<(outs VLR:$vl, GPR:$rd), (ins GPR:$rs1),
                   "vsetvl", "$rd, $rs1", [], InstFormatOther>
{
  bits<5> rs1;
  bits<5> rd;

  let Inst{31-28} = 0b1111;
  let Inst{27-25} = 0b000;
  let Inst{24-20} = 0b00000;
  let Inst{19-15} = rs1;
  let Inst{14} = 0b1;
  let Inst{13-12} = 0b00;
  let Inst{11-7} = rd;
  let Opcode = OPC_OP_V.Value;

  let Uses = [VCFG];
}

let hasSideEffects = 0, mayLoad = 0, mayStore = 0 in
def VCONFIG : RVInst<(outs), (ins uimm8:$imm8),
                     "vconfig", "$imm8", [], InstFormatOther>
{
  bits<8> imm8;

  let Inst{31-28} = 0b1111;
  let Inst{27-20} = imm8;
  let Inst{19-15} = 0b00000;
  let Inst{14} = 0b1;
  let Inst{13-12} = 0b01;
  let Inst{11-7} = 0b00000;
  let Opcode = OPC_OP_V.Value;

  let Defs = [VCFG];
}

} // Predicates = [HasStdExtV]

//===----------------------------------------------------------------------===//
// Pseudo-instructions and codegen patterns
//===----------------------------------------------------------------------===//


let Predicates = [HasStdExtV] in {

let hasSideEffects = 1, mayLoad = 0, mayStore = 0,
    isCodeGenOnly = 1, isAsmParserOnly = 0 in {
  def PseudoCSRR_VL : Pseudo<(outs GPR:$rd), (ins VLR:$vl), []>,
                      PseudoInstExpansion<(CSRRS GPR:$rd, 0xCC0, X0)>
  {
    let Uses = [VCFG];
  }
}


class PatVrVr<SDPatternOperator OpNode, RVInst Inst>
    : Pat<(OpNode TPR:$rs1, TPR:$rs2, VLR:$vl), (Inst TPR:$rs1, TPR:$rs2, VLR:$vl)>;

class PatVrIm<SDPatternOperator OpNode, RVInst Inst>
    : Pat<(OpNode TPR:$rs1, (PPUBROADCAST simm8:$imm), VLR:$vl), (Inst TPR:$rs1, simm8:$imm, VLR:$vl)>; 

class PatVr<SDPatternOperator OpNode, RVInst Inst>
    : Pat<(OpNode TPR:$rs1, VLR:$vl), (Inst TPR:$rs1, VLR:$vl)>;

// class PatTrIm<SDPatternOperator OpNode, RVInst Inst>
//     : Pat<(OpNode (PPUBROADCAST GPR:$rs1), (PPUBROADCAST simm8:$imm)), (Inst GPR:$s1, simm8:$imm)>;

def : PatVrVr<int_ppu_vadd, VADD>;
def : PatVrVr<int_ppu_vsub, VSUB>;
def : PatVrVr<int_ppu_vmul, VMUL>;
def : PatVrVr<int_ppu_vand, VAND>;
def : PatVrVr<int_ppu_vor, VOR>;
def : PatVrVr<int_ppu_vxor, VXOR>;
def : PatVrIm<int_ppu_vaddi, VADDI>;

def : PatVr<int_ppu_vmpopcnt, VMPOPC>;
def : PatVr<int_ppu_vmfirst, VMFIRST>;

// intrinsic
// def : PatTrIm<int_ppu_nvvm_tid_x, NVVM_TID>;

// def : Pat<(int_ppu_nvvm_tid_x), (NVVM_TID_x GPR:$rs1, 0) >;

// TODO exploit immediate offset
def : Pat<(int_ppu_vlw GPR:$rs1, VLR:$vl),
          (VLW GPR:$rs1, VLR:$vl, 0)>;

def : Pat<(int_ppu_vsw GPR:$rs1, TPR:$rs2, VLR:$vl),
          (VSW GPR:$rs1, TPR:$rs2, VLR:$vl, 0)>;

def : Pat<(PPUSETVL GPR:$rs1), (VSETVL GPR:$rs1)>;

/*
// Branch on undef scc. Used to avoid intermediate copy from
// IMPLICIT_DEF to SCC.
// FIXME def SI_BR_UNDEF : SPseudoInst <(outs), (ins sopp_brtarget:$simm16)> {
def SI_BR_UNDEF : SPseudoInst <(outs), (ins brtarget:$simm16)> {
  let isTerminator = 1;
  let usesCustomInserter = 1;
  let isBranch = 1;
}

// SI pseudo instructions. These are used by the CFG structurizer pass
// and should be lowered to ISA instructions prior to codegen.

// Dummy terminator instruction to use after control flow instructions
// replaced with exec mask operations.
def SI_MASK_BRANCH : VPseudoInst <
  (outs), (ins brtarget:$target)> {
  let isBranch = 0;
  let isTerminator = 1;
  let isBarrier = 0;
  let SchedRW = [];
  let hasNoSchedulingInfo = 1;
  // let FixedSize = 1;
  let Size = 0;
}
*/
/*
def CBRANCH_VCCZ: VPseudoInst <
  (outs), (ins brtarget:$target)> {
  let isBranch = 0;
  let isTerminator = 1;
  let isBarrier = 0;
  let SchedRW = [];
  let hasNoSchedulingInfo = 1;
  // let FixedSize = 1;
  let Size = 0;
}

def CBRANCH_VCCNZ: VPseudoInst <
  (outs), (ins brtarget:$target)> {
  let isBranch = 0;
  let isTerminator = 1;
  let isBarrier = 0;
  let SchedRW = [];
  let hasNoSchedulingInfo = 1;
  // let FixedSize = 1;
  let Size = 0;
}

def CBRANCH_TMSKNZ: VPseudoInst <
  (outs), (ins brtarget:$target)> {
  let isBranch = 0;
  let isTerminator = 1;
  let isBarrier = 0;
  let SchedRW = [];
  let hasNoSchedulingInfo = 1;
  // let FixedSize = 1;
  let Size = 0;
}
*/

} // Predicates = [HasStdExtV]


