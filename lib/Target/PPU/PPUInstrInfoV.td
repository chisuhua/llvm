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
               (outs VR:$vd), (ins VR:$vs1, VR:$vs2, VLR:$vl),
               opcodestr, "$vd, $vs1, $vs2">
{
  let Uses = [VCFG];
}

let hasSideEffects = 0, mayLoad = 0, mayStore = 0 in
class VALU_VrGpr<bits<12> funct12, bits<1> funct1, string opcodestr>
    : RVInstVrGpr<funct12, funct1, RVV_Unmasked, OPC_OP_V,
               (outs GPR:$rd), (ins VR:$vs1, VLR:$vl),
               opcodestr, "$rd, $vs1">
{
  let Uses = [VCFG];
}

let hasSideEffects = 0, mayLoad = 0, mayStore = 0 in
class VALU_ri<bits<4> funct4, bits<1> funct1, string opcodestr>
    : RVInstVI<funct4, funct1, RVV_Unmasked, OPC_OP_V,
               (outs VR:$vd), (ins VR:$vs1, simm8:$imm8, VLR:$vl),
               opcodestr, "$vd, $vs1, $imm8">
{
  let Uses = [VCFG];
}

let hasSideEffects = 0, mayLoad = 1, mayStore = 0 in
class VLoad_UnitStride<bits<2> funct2, bits<3> funct3, string opcodestr>
    : RVInstVLoad<funct2, funct3, RVV_Unmasked, OPC_LOAD_FP,
                  (outs VR:$vd), (ins GPR:$rs1, VLR:$vl, simm3:$imm3),
                  opcodestr, "$vd, ${imm3}(${rs1})">
{
  let Uses = [VCFG];
}

let hasSideEffects = 0, mayLoad = 0, mayStore = 1 in
class VStore_UnitStride<bits<3> funct3, bits<2> funct2, string opcodestr>
    : RVInstVStore<funct3, funct2, RVV_Unmasked, OPC_STORE_FP,
                   (outs), (ins GPR:$rs1, VR:$vs3, VLR:$vl, simm3:$imm3),
                   opcodestr, "$vs3, ${imm3}(${rs1})">
{
  let Uses = [VCFG];
}

//===----------------------------------------------------------------------===//
// Instructions
//===----------------------------------------------------------------------===//

let Predicates = [HasStdExtV] in {

def VADD : VALU_rr<0b1001000, 0b1, "vadd">;

def VMOV : VALU_rr<0b1001010, 0b1, "vmov">;  // tententive
def SMOV : VALU_rr<0b1001011, 0b1, "smov">;  // tententive

def VSUB : VALU_rr<0b1001001, 0b1, "vsub">;

def VADDI : VALU_ri<0b0000, 0b1, "vaddi">;

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

def brtarget   : Operand<OtherVT>;
/*
def SoppBrTarget : AsmOperandClass {
  let Name = "SoppBrTarget";
  let ParserMethod = "parseSOppBrTarget";
}

def sopp_brtarget : Operand<OtherVT> {
  let EncoderMethod = "getSOPPBrEncoding";
  let DecoderMethod = "decodeSoppBrTarget";
  let OperandType = "OPERAND_PCREL";
  let ParserMatchClass = SoppBrTarget;
}
*/

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
    : Pat<(OpNode VR:$rs1, VR:$rs2, VLR:$vl), (Inst VR:$rs1, VR:$rs2, VLR:$vl)>;

class PatVrIm<SDPatternOperator OpNode, RVInst Inst>
    : Pat<(OpNode VR:$rs1, (PPUBROADCAST simm8:$imm), VLR:$vl), (Inst VR:$rs1, simm8:$imm, VLR:$vl)>; 

class PatVr<SDPatternOperator OpNode, RVInst Inst>
    : Pat<(OpNode VR:$rs1, VLR:$vl), (Inst VR:$rs1, VLR:$vl)>;

def : PatVrVr<int_ppu_vadd, VADD>;
def : PatVrVr<int_ppu_vsub, VSUB>;
def : PatVrVr<int_ppu_vmul, VMUL>;
def : PatVrVr<int_ppu_vand, VAND>;
def : PatVrVr<int_ppu_vor, VOR>;
def : PatVrVr<int_ppu_vxor, VXOR>;
def : PatVrIm<int_ppu_vadd, VADDI>;

def : PatVr<int_ppu_vmpopcnt, VMPOPC>;
def : PatVr<int_ppu_vmfirst, VMFIRST>;

// TODO exploit immediate offset
def : Pat<(int_ppu_vlw GPR:$rs1, VLR:$vl),
          (VLW GPR:$rs1, VLR:$vl, 0)>;

def : Pat<(int_ppu_vsw GPR:$rs1, VR:$rs2, VLR:$vl),
          (VSW GPR:$rs1, VR:$rs2, VLR:$vl, 0)>;

def : Pat<(PPUSETVL GPR:$rs1), (VSETVL GPR:$rs1)>;


let isTerminator = 1 in {
let Uses = [EXEC], Defs = [EXEC],
    OtherPredicates = [EnableNonUniformMIBranchPseudo] in {
  def SI_NON_UNIFORM_BRCOND_PSEUDO : CFPseudoInst <
      (outs),
      (ins SReg_32:$vcc, brtarget:$target),
      [(brcond i1:$vcc, bb:$target)]> {
     // let Size = 12;
  }
}

def SI_IF: CFPseudoInst <
  (outs SReg_1:$dst), (ins SReg_1:$vcc, brtarget:$target),
  [(set i1:$dst, (AMDGPUif i1:$vcc, bb:$target))], 1, 1> {
  let Constraints = "";
  let Size = 12;
  let hasSideEffects = 1;
}

def SI_ELSE : CFPseudoInst <
  (outs SReg_1:$dst),
  (ins SReg_1:$src, brtarget:$target, i1imm:$execfix), [], 1, 1> {
  let Size = 12;
  let hasSideEffects = 1;
}

def SI_LOOP : CFPseudoInst <
  (outs), (ins SReg_1:$saved, brtarget:$target),
  [(AMDGPUloop i1:$saved, bb:$target)], 1, 1> {
  let Size = 8;
  let isBranch = 1;
  let hasSideEffects = 1;
}
} // isTerminator

def SI_END_CF : CFPseudoInst <
  (outs), (ins SReg_1:$saved), [], 1, 1> {
  let Size = 4;
  let isAsCheapAsAMove = 1;
  let isReMaterializable = 1;
  let hasSideEffects = 1;
  let mayLoad = 1; // FIXME: Should not need memory flags
  let mayStore = 1;
}

def SI_IF_BREAK : CFPseudoInst <
  (outs SReg_1:$dst), (ins SReg_1:$vcc, SReg_1:$src), []> {
  let Size = 4;
  let isAsCheapAsAMove = 1;
  let isReMaterializable = 1;
}

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

} // Predicates = [HasStdExtV]


