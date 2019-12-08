//===-- PPUInstructions.td - Common instruction defs ---*- tablegen -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains instruction defs  base on AMDGPUInstructions.td and AMDGPUInstrInfo.td
// SIInstructions.td 

// the AMDGPUInstrInfo.td is partially reference
// 
// This file contains DAG node defintions for the AMDGPU target.
//
//===----------------------------------------------------------------------===//

////////////////////////////////////////////////////////////////////
//  below AMDGPUInstrInfo.td
//===----------------------------------------------------------------------===//
// PPU DAG Profiles
//===----------------------------------------------------------------------===//
def PPUDTIntTernaryOp : SDTypeProfile<1, 3, [
  SDTCisSameAs<0, 1>, SDTCisSameAs<0, 2>, SDTCisInt<0>, SDTCisInt<3>
]>;

def PPUTrigPreOp : SDTypeProfile<1, 2,
  [SDTCisSameAs<0, 1>, SDTCisFP<0>, SDTCisInt<2>]
>;

def PPULdExpOp : SDTypeProfile<1, 2,
  [SDTCisSameAs<0, 1>, SDTCisFP<0>, SDTCisInt<2>]
>;


def PPUFPClassOp : SDTypeProfile<1, 2,
  [SDTCisInt<0>, SDTCisFP<1>, SDTCisInt<2>]
>;

def PPUFPPackOp : SDTypeProfile<1, 2,
  [SDTCisFP<1>, SDTCisSameAs<1, 2>]
>;

def PPUIntPackOp : SDTypeProfile<1, 2,
  [SDTCisInt<1>, SDTCisSameAs<1, 2>]
>;

def PPUDivScaleOp : SDTypeProfile<2, 3,
  [SDTCisFP<0>, SDTCisInt<1>, SDTCisSameAs<0, 2>, SDTCisSameAs<0, 3>, SDTCisSameAs<0, 4>]
>;

// float, float, float, vcc
def PPUFmasOp : SDTypeProfile<1, 4,
  [SDTCisFP<0>, SDTCisSameAs<0, 1>, SDTCisSameAs<0, 2>, SDTCisSameAs<0, 3>, SDTCisInt<4>]
>;

def PPUKillSDT : SDTypeProfile<0, 1, [SDTCisInt<0>]>;

def PPUIfOp : SDTypeProfile<1, 2,
  [SDTCisVT<0, i1>, SDTCisVT<1, i1>, SDTCisVT<2, OtherVT>]
>;

def PPUElseOp : SDTypeProfile<1, 2,
  [SDTCisVT<0, i1>, SDTCisVT<1, i1>, SDTCisVT<2, OtherVT>]
>;

def PPULoopOp : SDTypeProfile<0, 2,
  [SDTCisVT<0, i1>, SDTCisVT<1, OtherVT>]
>;

def PPUIfBreakOp : SDTypeProfile<1, 2,
  [SDTCisVT<0, i1>, SDTCisVT<1, i1>, SDTCisVT<2, i1>]
>;


// AMDGPU DAG Nodes
//
def PPUif : SDNode<"PPUISD::IF", PPUIfOp, [SDNPHasChain]>;
def PPUelse : SDNode<"PPUISD::ELSE", PPUElseOp, [SDNPHasChain]>;
def PPUloop : SDNode<"PPUISD::LOOP", PPULoopOp, [SDNPHasChain]>;
/*
def callseq_start : SDNode<"ISD::CALLSEQ_START",
  SDCallSeqStart<[ SDTCisVT<0, i32>, SDTCisVT<1, i32> ]>,
  [SDNPHasChain, SDNPOutGlue]
>;

def callseq_end : SDNode<"ISD::CALLSEQ_END",
 SDCallSeqEnd<[ SDTCisVT<0, i32>, SDTCisVT<1, i32> ]>,
  [SDNPHasChain, SDNPOptInGlue, SDNPOutGlue]
>;
*/

def PPUcall : SDNode<"PPUISD::CALL",
  SDTypeProfile<0, -1, [SDTCisPtrTy<0>]>,
  [SDNPHasChain, SDNPOptInGlue, SDNPOutGlue,
  SDNPVariadic]
>;

def PPUtc_return: SDNode<"PPUISD::TC_RETURN",
  SDTypeProfile<0, 3, [SDTCisPtrTy<0>]>,
  [SDNPHasChain, SDNPOptInGlue, SDNPVariadic]
>;

def PPUtrap : SDNode<"PPUISD::TRAP",
  SDTypeProfile<0, -1, [SDTCisVT<0, i16>]>,
    [SDNPHasChain, SDNPVariadic, SDNPSideEffect, SDNPInGlue]
>;

def PPUconstdata_ptr : SDNode<
  "PPUISD::CONST_DATA_PTR", SDTypeProfile <1, 1, [SDTCisVT<0, iPTR>,
                                                     SDTCisVT<0, iPTR>]>
>;


def PPUcos : SDNode<"PPUISD::COS_HW", SDTFPUnaryOp>;
def PPUsin : SDNode<"PPUISD::SIN_HW", SDTFPUnaryOp>;

// out = a - floor(a)
def PPUfract : SDNode<"PPUISD::FRACT", SDTFPUnaryOp>;

// out = 1.0 / a
def PPUrcp : SDNode<"PPUISD::RCP", SDTFPUnaryOp>;

// out = 1.0 / sqrt(a)
def PPUrsq : SDNode<"PPUISD::RSQ", SDTFPUnaryOp>;

def PPUrcp_iflag : SDNode<"PPUISD::RCP_IFLAG", SDTFPUnaryOp>;

// out = 1.0 / sqrt(a) result clamped to +/- max_float.
def PPUrsq_clamp : SDNode<"PPUISD::RSQ_CLAMP", SDTFPUnaryOp>;

def PPUpkrtz_f16_f32 : SDNode<"PPUISD::CVT_PKRTZ_F16_F32", PPUFPPackOp>;
def PPUpknorm_i16_f32 : SDNode<"PPUISD::CVT_PKNORM_I16_F32", PPUFPPackOp>;
def PPUpknorm_u16_f32 : SDNode<"PPUISD::CVT_PKNORM_U16_F32", PPUFPPackOp>;
def PPUpk_i16_i32 : SDNode<"PPUISD::CVT_PK_I16_I32", PPUIntPackOp>;
def PPUpk_u16_u32 : SDNode<"PPUISD::CVT_PK_U16_U32", PPUIntPackOp>;
def PPUfp_to_f16 : SDNode<"PPUISD::FP_TO_FP16" , SDTFPToIntOp>;
def PPUfp16_zext : SDNode<"PPUISD::FP16_ZEXT" , SDTFPToIntOp>;

def PPUfp_class : SDNode<"PPUISD::FP_CLASS", PPUFPClassOp>;

// FIXME: TableGen doesn't like commutative instructions with more
// than 2 operands.
// out = max(a, b, c) a, b and c are floats
def PPUfmax3 : SDNode<"PPUISD::FMAX3", SDTFPTernaryOp,
  [/*SDNPCommutative, SDNPAssociative*/]
>;

// out = max(a, b, c) a, b, and c are signed ints
def PPUsmax3 : SDNode<"PPUISD::SMAX3", PPUDTIntTernaryOp,
  [/*SDNPCommutative, SDNPAssociative*/]
>;

// out = max(a, b, c) a, b and c are unsigned ints
def PPUumax3 : SDNode<"PPUISD::UMAX3", PPUDTIntTernaryOp,
  [/*SDNPCommutative, SDNPAssociative*/]
>;

// out = min(a, b, c) a, b and c are floats
def PPUfmin3 : SDNode<"PPUISD::FMIN3", SDTFPTernaryOp,
  [/*SDNPCommutative, SDNPAssociative*/]
>;

// out = min(a, b, c) a, b and c are signed ints
def PPUsmin3 : SDNode<"PPUISD::SMIN3", PPUDTIntTernaryOp,
  [/*SDNPCommutative, SDNPAssociative*/]
>;

// out = min(a, b) a and b are unsigned ints
def PPUumin3 : SDNode<"PPUISD::UMIN3", PPUDTIntTernaryOp,
  [/*SDNPCommutative, SDNPAssociative*/]
>;

// out = (src0 + src1 > 0xFFFFFFFF) ? 1 : 0
def PPUcarry : SDNode<"PPUISD::CARRY", SDTIntBinOp, []>;

// out = (src1 > src0) ? 1 : 0
def PPUborrow : SDNode<"PPUISD::BORROW", SDTIntBinOp, []>;

def PPUSetCCOp : SDTypeProfile<1, 3, [        // setcc
  SDTCisInt<0>, SDTCisSameAs<1, 2>, SDTCisVT<3, OtherVT>
]>;

def PPUsetcc : SDNode<"PPUISD::SETCC", PPUSetCCOp>;

def PPUSetRegOp :  SDTypeProfile<0, 2, [
  SDTCisInt<0>, SDTCisInt<1>
]>;

def PPUsetreg : SDNode<"PPUISD::SETREG", PPUSetRegOp, [
  SDNPHasChain, SDNPSideEffect, SDNPOptInGlue, SDNPOutGlue]>;

def PPUfma : SDNode<"PPUISD::FMA_W_CHAIN", SDTFPTernaryOp, [
   SDNPHasChain, SDNPOptInGlue, SDNPOutGlue]>;

def PPUmul : SDNode<"PPUISD::FMUL_W_CHAIN", SDTFPBinOp, [
  SDNPHasChain, SDNPOptInGlue, SDNPOutGlue]>;

def PPUcvt_f32_ubyte0 : SDNode<"PPUISD::CVT_F32_UBYTE0",
  SDTIntToFPOp, []>;
def PPUcvt_f32_ubyte1 : SDNode<"PPUISD::CVT_F32_UBYTE1",
  SDTIntToFPOp, []>;
def PPUcvt_f32_ubyte2 : SDNode<"PPUISD::CVT_F32_UBYTE2",
  SDTIntToFPOp, []>;
def PPUcvt_f32_ubyte3 : SDNode<"PPUISD::CVT_F32_UBYTE3",
  SDTIntToFPOp, []>;

// urecip - This operation is a helper for integer division, it returns the
// result of 1 / a as a fractional unsigned integer.
// out = (2^32 / a) + e
// e is rounding error
def PPUurecip : SDNode<"PPUISD::URECIP", SDTIntUnaryOp>;

// Special case divide preop and flags.
def PPUdiv_scale : SDNode<"PPUISD::DIV_SCALE", PPUDivScaleOp>;

//  Special case divide FMA with scale and flags (src0 = Quotient,
//  src1 = Denominator, src2 = Numerator).
def PPUdiv_fmas : SDNode<"PPUISD::DIV_FMAS", PPUFmasOp,
                            [SDNPOptInGlue]>;

// Single or double precision division fixup.
// Special case divide fixup and flags(src0 = Quotient, src1 =
// Denominator, src2 = Numerator).
def PPUdiv_fixup : SDNode<"PPUISD::DIV_FIXUP", SDTFPTernaryOp>;

def PPUfmad_ftz : SDNode<"PPUISD::FMAD_FTZ", SDTFPTernaryOp>;

// Look Up 2.0 / pi src0 with segment select src1[4:0]
def PPUtrig_preop : SDNode<"PPUISD::TRIG_PREOP", PPUTrigPreOp>;






// MSKOR instructions are atomic memory instructions used mainly for storing
// 8-bit and 16-bit values.  The definition is:
//
// MSKOR(dst, mask, src) MEM[dst] = ((MEM[dst] & ~mask) | src)
//
// src0: vec4(src, 0, 0, mask)
// src1: dst - rat offset (aka pointer) in dwords
def PPUstore_mskor : SDNode<"PPUISD::STORE_MSKOR",
                        SDTypeProfile<0, 2, []>,
                        [SDNPHasChain, SDNPMayStore, SDNPMemOperand]>;
// This argument to this node is a dword address.
def PPUdwordaddr : SDNode<"PPUISD::DWORDADDR", SDTIntUnaryOp>;

def PPUround : SDNode<"ISD::FROUND",
                         SDTypeProfile<1, 1, [SDTCisFP<0>, SDTCisSameAs<0,1>]>>;

def PPUbfe_u32 : SDNode<"PPUISD::BFE_U32", PPUDTIntTernaryOp>;
def PPUbfe_i32 : SDNode<"PPUISD::BFE_I32", PPUDTIntTernaryOp>;
def PPUbfi : SDNode<"PPUISD::BFI", PPUDTIntTernaryOp>;
def PPUbfm : SDNode<"PPUISD::BFM", SDTIntBinOp>;

def PPUffbh_u32 : SDNode<"PPUISD::FFBH_U32", SDTIntUnaryOp>;
def PPUffbh_i32 : SDNode<"PPUISD::FFBH_I32", SDTIntUnaryOp>;

def PPUffbl_b32 : SDNode<"PPUISD::FFBL_B32", SDTIntUnaryOp>;

// Signed and unsigned 24-bit multiply. The highest 8-bits are ignore
// when performing the mulitply. The result is a 32-bit value.
def PPUmul_u24 : SDNode<"PPUISD::MUL_U24", SDTIntBinOp,
  [SDNPCommutative, SDNPAssociative]
>;

def PPUmul_i24 : SDNode<"PPUISD::MUL_I24", SDTIntBinOp,
  [SDNPCommutative, SDNPAssociative]
>;

def PPUmulhi_u24 : SDNode<"PPUISD::MULHI_U24", SDTIntBinOp,
  [SDNPCommutative, SDNPAssociative]
>;
def PPUmulhi_i24 : SDNode<"PPUISD::MULHI_I24", SDTIntBinOp,
  [SDNPCommutative, SDNPAssociative]
>;

def PPUmad_u24 : SDNode<"PPUISD::MAD_U24", PPUDTIntTernaryOp,
  []
>;
def PPUmad_i24 : SDNode<"PPUISD::MAD_I24", PPUDTIntTernaryOp,
  []
>;

def PPUsmed3 : SDNode<"PPUISD::SMED3", PPUDTIntTernaryOp,
  []
>;

def PPUumed3 : SDNode<"PPUISD::UMED3", PPUDTIntTernaryOp,
  []
>;

def PPUfmed3 : SDNode<"PPUISD::FMED3", SDTFPTernaryOp, []>;

def PPUfdot2 : SDNode<"PPUISD::FDOT2",
                  SDTypeProfile<1, 4, [SDTCisSameAs<0, 3>, SDTCisSameAs<1, 2>,
                                       SDTCisFP<0>, SDTCisVec<1>,
                                       SDTCisInt<4>]>,
                  []>;

def PPUperm : SDNode<"PPUISD::PERM", PPUDTIntTernaryOp, []>;

def PPUinit_exec : SDNode<"PPUISD::INIT_TMSK",
                      SDTypeProfile<0, 1, [SDTCisInt<0>]>,
                      [SDNPHasChain, SDNPInGlue]>;

def PPUinit_exec_from_input : SDNode<"PPUISD::INIT_TMSK_FROM_INPUT",
                                 SDTypeProfile<0, 2,
                                 [SDTCisInt<0>, SDTCisInt<1>]>,
                                 [SDNPHasChain, SDNPInGlue]>;


def PPUkill : SDNode<"PPUISD::KILL", PPUKillSDT,
  [SDNPHasChain, SDNPSideEffect]>;

//===----------------------------------------------------------------------===//
// Flow Control Profile Types
//===----------------------------------------------------------------------===//
// def IL_brcond      : SDNode<"PPUISD::BRANCH_COND", SDTIL_BRCond, [SDNPHasChain]>;

//===----------------------------------------------------------------------===//
// Call/Return DAG Nodes
//===----------------------------------------------------------------------===//
def PPUendpgm : SDNode<"PPUISD::ENDPGM", SDTNone,
    [SDNPHasChain, SDNPOptInGlue]>;

def PPUreturn_to_epilog : SDNode<"PPUISD::RETURN_TO_EPILOG", SDTNone,
    [SDNPHasChain, SDNPOptInGlue, SDNPVariadic]>;

// TODO
def PPUret_flag : SDNode<"PPUISD::RET_FLAG", SDTypeProfile<0, 1, [SDTCisPtrTy<0>]>,
  [SDNPHasChain, SDNPOptInGlue, SDNPVariadic]
>;

///////////////////////////////////////////////////////////////////////
// below is base on AMDGPUInstructions.td

class AddressSpacesImpl {
  int Flat = 0;
  int Global = 1;
  int Region = 2;
  int Local = 3;
  int Constant = 4;
  int Private = 5;
}

def AddrSpaces : AddressSpacesImpl;

class PPUBaseInst <dag outs, dag ins, string asm = "",
  list<dag> pattern = []> : Instruction {
  field bit isRegisterLoad = 0;
  field bit isRegisterStore = 0;

  let Namespace = "PPU";
  let OutOperandList = outs;
  let InOperandList = ins;
  let AsmString = asm;
  let Pattern = pattern;
  let Itinerary = NullALU;

  // SoftFail is a field the disassembler can use to provide a way for
  // instructions to not match without killing the whole decode process. It is
  // mainly used for ARM, but Tablegen expects this field to exist or it fails
  // to build the decode table.
  field bits<64> SoftFail = 0;

  let DecoderNamespace = Namespace;

  let TSFlags{63} = isRegisterLoad;
  let TSFlags{62} = isRegisterStore;
}

def TruePredicate : Predicate<"">;

class PredicateControl {
  Predicate SubtargetPredicate = TruePredicate;
  list<Predicate> AssemblerPredicates = [];
  Predicate AssemblerPredicate = TruePredicate;
  Predicate WaveSizePredicate = TruePredicate;
  list<Predicate> OtherPredicates = [];
  list<Predicate> Predicates = !listconcat([SubtargetPredicate,
                                            AssemblerPredicate,
                                            WaveSizePredicate],
                                            AssemblerPredicates,
                                            OtherPredicates);
}

class PPUPat<dag pattern, dag result> : Pat<pattern, result>,
      PredicateControl;


def FP16Denormals : Predicate<"Subtarget->hasFP16Denormals()">;
def FP32Denormals : Predicate<"Subtarget->hasFP32Denormals()">;
def FP64Denormals : Predicate<"Subtarget->hasFP64Denormals()">;
def NoFP16Denormals : Predicate<"!Subtarget->hasFP16Denormals()">;
def NoFP32Denormals : Predicate<"!Subtarget->hasFP32Denormals()">;
def NoFP64Denormals : Predicate<"!Subtarget->hasFP64Denormals()">;
def UnsafeFPMath : Predicate<"TM.Options.UnsafeFPMath">;
def FMA : Predicate<"Subtarget->hasFMA()">;

def InstFlag : OperandWithDefaultOps <i32, (ops (i32 0))>;

def u16ImmTarget : AsmOperandClass {
  let Name = "U16Imm";
  let RenderMethod = "addImmOperands";
}

def s16ImmTarget : AsmOperandClass {
  let Name = "S16Imm";
  let RenderMethod = "addImmOperands";
}

let OperandType = "OPERAND_IMMEDIATE" in {

def u32imm : Operand<i32> {
  let PrintMethod = "printU32ImmOperand";
}

def u16imm : Operand<i16> {
  let PrintMethod = "printU16ImmOperand";
  let ParserMatchClass = u16ImmTarget;
}

def s16imm : Operand<i16> {
  let PrintMethod = "printU16ImmOperand";
  let ParserMatchClass = s16ImmTarget;
}

def u8imm : Operand<i8> {
  let PrintMethod = "printU8ImmOperand";
}

def u1imm : Operand<i1> {
  let PrintMethod = "printU1ImmOperand";
}

} // End OperandType = "OPERAND_IMMEDIATE"

//===--------------------------------------------------------------------===//
// Custom Operands
//===--------------------------------------------------------------------===//
def brtarget   : Operand<OtherVT>;

//===----------------------------------------------------------------------===//
// Misc. PatFrags
//===----------------------------------------------------------------------===//

class HasOneUseUnaryOp<SDPatternOperator op> : PatFrag<
  (ops node:$src0),
  (op $src0),
  [{ return N->hasOneUse(); }]
>;

class HasOneUseBinOp<SDPatternOperator op> : PatFrag<
  (ops node:$src0, node:$src1),
  (op $src0, $src1),
  [{ return N->hasOneUse(); }]
>;

class HasOneUseTernaryOp<SDPatternOperator op> : PatFrag<
  (ops node:$src0, node:$src1, node:$src2),
  (op $src0, $src1, $src2),
  [{ return N->hasOneUse(); }]
>;

let Properties = [SDNPCommutative, SDNPAssociative] in {
def smax_oneuse : HasOneUseBinOp<smax>;
def smin_oneuse : HasOneUseBinOp<smin>;
def umax_oneuse : HasOneUseBinOp<umax>;
def umin_oneuse : HasOneUseBinOp<umin>;

def fminnum_oneuse : HasOneUseBinOp<fminnum>;
def fmaxnum_oneuse : HasOneUseBinOp<fmaxnum>;

def fminnum_ieee_oneuse : HasOneUseBinOp<fminnum_ieee>;
def fmaxnum_ieee_oneuse : HasOneUseBinOp<fmaxnum_ieee>;


def and_oneuse : HasOneUseBinOp<and>;
def or_oneuse : HasOneUseBinOp<or>;
def xor_oneuse : HasOneUseBinOp<xor>;
} // Properties = [SDNPCommutative, SDNPAssociative]

def not_oneuse : HasOneUseUnaryOp<not>;

def add_oneuse : HasOneUseBinOp<add>;
def sub_oneuse : HasOneUseBinOp<sub>;

def srl_oneuse : HasOneUseBinOp<srl>;
def shl_oneuse : HasOneUseBinOp<shl>;

def select_oneuse : HasOneUseTernaryOp<select>;

def PPUmul_u24_oneuse : HasOneUseBinOp<PPUmul_u24>;
def PPUmul_i24_oneuse : HasOneUseBinOp<PPUmul_i24>;

def srl_16 : PatFrag<
  (ops node:$src0), (srl_oneuse node:$src0, (i32 16))
>;


def hi_i16_elt : PatFrag<
  (ops node:$src0), (i16 (trunc (i32 (srl_16 node:$src0))))
>;


def hi_f16_elt : PatLeaf<
  (vt), [{
  if (N->getOpcode() != ISD::BITCAST)
    return false;
  SDValue Tmp = N->getOperand(0);

  if (Tmp.getOpcode() != ISD::SRL)
    return false;
    if (const auto *RHS = dyn_cast<ConstantSDNode>(Tmp.getOperand(1))
      return RHS->getZExtValue() == 16;
    return false;
}]>;

//===----------------------------------------------------------------------===//
// PatLeafs for floating-point comparisons
//===----------------------------------------------------------------------===//

def COND_OEQ : PatFrags<(ops), [(OtherVT SETOEQ), (OtherVT SETEQ)]>;
def COND_ONE : PatFrags<(ops), [(OtherVT SETONE), (OtherVT SETNE)]>;
def COND_OGT : PatFrags<(ops), [(OtherVT SETOGT), (OtherVT SETGT)]>;
def COND_OGE : PatFrags<(ops), [(OtherVT SETOGE), (OtherVT SETGE)]>;
def COND_OLT : PatFrags<(ops), [(OtherVT SETOLT), (OtherVT SETLT)]>;
def COND_OLE : PatFrags<(ops), [(OtherVT SETOLE), (OtherVT SETLE)]>;
def COND_O   : PatFrags<(ops), [(OtherVT SETO)]>;
def COND_UO  : PatFrags<(ops), [(OtherVT SETUO)]>;

//===----------------------------------------------------------------------===//
// PatLeafs for unsigned / unordered comparisons
//===----------------------------------------------------------------------===//

def COND_UEQ : PatFrag<(ops), (OtherVT SETUEQ)>;
def COND_UNE : PatFrag<(ops), (OtherVT SETUNE)>;
def COND_UGT : PatFrag<(ops), (OtherVT SETUGT)>;
def COND_UGE : PatFrag<(ops), (OtherVT SETUGE)>;
def COND_ULT : PatFrag<(ops), (OtherVT SETULT)>;
def COND_ULE : PatFrag<(ops), (OtherVT SETULE)>;

// XXX - For some reason R600 version is preferring to use unordered
// for setne?
def COND_UNE_NE  : PatFrags<(ops), [(OtherVT SETUNE), (OtherVT SETNE)]>;

//===----------------------------------------------------------------------===//
// PatLeafs for signed comparisons
//===----------------------------------------------------------------------===//

def COND_SGT : PatFrag<(ops), (OtherVT SETGT)>;
def COND_SGE : PatFrag<(ops), (OtherVT SETGE)>;
def COND_SLT : PatFrag<(ops), (OtherVT SETLT)>;
def COND_SLE : PatFrag<(ops), (OtherVT SETLE)>;

//===----------------------------------------------------------------------===//
// PatLeafs for integer equality
//===----------------------------------------------------------------------===//

def COND_EQ : PatFrags<(ops), [(OtherVT SETEQ), (OtherVT SETUEQ)]>;
def COND_NE : PatFrags<(ops), [(OtherVT SETNE), (OtherVT SETUNE)]>;

// FIXME: Should not need code predicate
//def COND_NULL : PatLeaf<(OtherVT null_frag)>;
def COND_NULL : PatLeaf <
  (cond),
  [{(void)N; return false;}]
>;
//===----------------------------------------------------------------------===//
// Load/Store Pattern Fragments
//===----------------------------------------------------------------------===//
def atomic_cmp_swap_glue : SDNode <"ISD::ATOMIC_CMP_SWAP", SDTAtomic3,
  [SDNPHasChain, SDNPMayStore, SDNPMayLoad, SDNPMemOperand, SDNPInGlue]
>;

class AddressSpaceList<list<int> AS> {
  list<int> AddrSpaces = AS;
}

class Aligned<int Bytes> {
  int MinAlignment = Bytes;
}

class LoadFrag <SDPatternOperator op> : PatFrag<(ops node:$ptr), (op node:$ptr)>;

class StoreFrag<SDPatternOperator op> : PatFrag <
  (ops node:$value, node:$ptr), (op node:$value, node:$ptr)
>;

class StoreHi16<SDPatternOperator op> : PatFrag <
  (ops node:$value, node:$ptr), (op (srl node:$value, (i32 16)), node:$ptr)
>;

def LoadAddress_constant : AddressSpaceList<[  AddrSpaces.Constant ]>;
def LoadAddress_global : AddressSpaceList<[  AddrSpaces.Global, AddrSpaces.Constant ]>;
def StoreAddress_global : AddressSpaceList<[ AddrSpaces.Global ]>;

def LoadAddress_flat : AddressSpaceList<[  AddrSpaces.Flat,
                                           AddrSpaces.Global,
                                           AddrSpaces.Constant ]>;
def StoreAddress_flat : AddressSpaceList<[ AddrSpaces.Flat, AddrSpaces.Global ]>;

def LoadAddress_private : AddressSpaceList<[ AddrSpaces.Private ]>;
def StoreAddress_private : AddressSpaceList<[ AddrSpaces.Private ]>;

def LoadAddress_local : AddressSpaceList<[ AddrSpaces.Local ]>;
def StoreAddress_local : AddressSpaceList<[ AddrSpaces.Local ]>;

def LoadAddress_region : AddressSpaceList<[ AddrSpaces.Region ]>;
def StoreAddress_region : AddressSpaceList<[ AddrSpaces.Region ]>;

class GlobalLoadAddress : CodePatPred<[{
  auto AS = cast<MemSDNode>(N)->getAddressSpace();
  return AS == AMDGPUAS::GLOBAL_ADDRESS || AS == AMDGPUAS::CONSTANT_ADDRESS;
}]>;

class FlatLoadAddress : CodePatPred<[{
  const auto AS = cast<MemSDNode>(N)->getAddressSpace();
  return AS == AMDGPUAS::FLAT_ADDRESS ||
         AS == AMDGPUAS::GLOBAL_ADDRESS ||
         AS == AMDGPUAS::CONSTANT_ADDRESS;
}]>;

class GlobalAddress : CodePatPred<[{
  return cast<MemSDNode>(N)->getAddressSpace() == AMDGPUAS::GLOBAL_ADDRESS;
}]>;

class PrivateAddress : CodePatPred<[{
  return cast<MemSDNode>(N)->getAddressSpace() == AMDGPUAS::PRIVATE_ADDRESS;
}]>;

class LocalAddress : CodePatPred<[{
  return cast<MemSDNode>(N)->getAddressSpace() == AMDGPUAS::LOCAL_ADDRESS;
}]>;

class RegionAddress : CodePatPred<[{
  return cast<MemSDNode>(N)->getAddressSpace() == AMDGPUAS::REGION_ADDRESS;
}]>;

class FlatStoreAddress : CodePatPred<[{
  const auto AS = cast<MemSDNode>(N)->getAddressSpace();
  return AS == AMDGPUAS::FLAT_ADDRESS ||
         AS == AMDGPUAS::GLOBAL_ADDRESS;
}]>;

// TODO: Remove these when stores to new PatFrag format.
class PrivateStore <SDPatternOperator op> : StoreFrag <op>, PrivateAddress;
class LocalStore <SDPatternOperator op> : StoreFrag <op>, LocalAddress;
class RegionStore <SDPatternOperator op> : StoreFrag <op>, RegionAddress;
class GlobalStore <SDPatternOperator op> : StoreFrag<op>, GlobalAddress;
class FlatStore <SDPatternOperator op> : StoreFrag <op>, FlatStoreAddress;


foreach as = [ "global", "flat", "constant", "local", "private", "region" ] in {
let AddressSpaces = !cast<AddressSpaceList>("LoadAddress_"#as).AddrSpaces in {

def load_#as : PatFrag<(ops node:$ptr), (unindexedload node:$ptr)> {
  let IsLoad = 1;
  let IsNonExtLoad = 1;
}

def extloadi8_#as  : PatFrag<(ops node:$ptr), (extload node:$ptr)> {
  let IsLoad = 1;
  let MemoryVT = i8;
}

def extloadi16_#as : PatFrag<(ops node:$ptr), (extload node:$ptr)> {
  let IsLoad = 1;
  let MemoryVT = i16;
}

def sextloadi8_#as  : PatFrag<(ops node:$ptr), (sextload node:$ptr)> {
  let IsLoad = 1;
  let MemoryVT = i8;
}

def sextloadi16_#as : PatFrag<(ops node:$ptr), (sextload node:$ptr)> {
  let IsLoad = 1;
  let MemoryVT = i16;
}

def zextloadi8_#as  : PatFrag<(ops node:$ptr), (zextload node:$ptr)> {
  let IsLoad = 1;
  let MemoryVT = i8;
}

def zextloadi16_#as : PatFrag<(ops node:$ptr), (zextload node:$ptr)> {
  let IsLoad = 1;
  let MemoryVT = i16;
}

def atomic_load_32_#as : PatFrag<(ops node:$ptr), (atomic_load_32 node:$ptr)> {
  let IsAtomic = 1;
  let MemoryVT = i32;
}

def atomic_load_64_#as : PatFrag<(ops node:$ptr), (atomic_load_64 node:$ptr)> {
  let IsAtomic = 1;
  let MemoryVT = i64;
}

def store_#as : PatFrag<(ops node:$val, node:$ptr),
                    (unindexedstore node:$val, node:$ptr)> {
  let IsStore = 1;
  let IsTruncStore = 0;
}

// truncstore fragments.
def truncstore_#as : PatFrag<(ops node:$val, node:$ptr),
                             (unindexedstore node:$val, node:$ptr)> {
  let IsStore = 1;
  let IsTruncStore = 1;
}

// TODO: We don't really need the truncstore here. We can use
// unindexedstore with MemoryVT directly, which will save an
// unnecessary check that the memory size is less than the value type
// in the generated matcher table.
def truncstorei8_#as : PatFrag<(ops node:$val, node:$ptr),
                               (truncstore node:$val, node:$ptr)> {
  let IsStore = 1;
  let MemoryVT = i8;
}

def truncstorei16_#as : PatFrag<(ops node:$val, node:$ptr),
                                (truncstore node:$val, node:$ptr)> {
  let IsStore = 1;
  let MemoryVT = i16;
}

defm atomic_store_#as : binary_atomic_op<atomic_store>;

} // End let AddressSpaces = ...
} // End foreach AddrSpace

multiclass ret_noret_binary_atomic_op<SDNode atomic_op, bit IsInt = 1> {
  foreach as = [ "global", "flat", "constant", "local", "private", "region" ] in {
    let AddressSpaces = !cast<AddressSpaceList>("LoadAddress_"#as).AddrSpaces in {
      defm "_"#as : binary_atomic_op<atomic_op, IsInt>;

      let PredicateCode = [{return (SDValue(N, 0).use_empty());}] in {
        defm "_"#as#"_noret" : binary_atomic_op<atomic_op, IsInt>;
      }

      let PredicateCode = [{return !(SDValue(N, 0).use_empty());}] in {
        defm "_"#as#"_ret" : binary_atomic_op<atomic_op, IsInt>;
      }
    }
  }
}

defm atomic_swap : ret_noret_binary_atomic_op<atomic_swap>;
defm atomic_load_add : ret_noret_binary_atomic_op<atomic_load_add>;
defm atomic_load_and : ret_noret_binary_atomic_op<atomic_load_and>;
defm atomic_load_max : ret_noret_binary_atomic_op<atomic_load_max>;
defm atomic_load_min : ret_noret_binary_atomic_op<atomic_load_min>;
defm atomic_load_or : ret_noret_binary_atomic_op<atomic_load_or>;
defm atomic_load_sub : ret_noret_binary_atomic_op<atomic_load_sub>;
defm atomic_load_umax : ret_noret_binary_atomic_op<atomic_load_umax>;
defm atomic_load_umin : ret_noret_binary_atomic_op<atomic_load_umin>;
defm atomic_load_xor : ret_noret_binary_atomic_op<atomic_load_xor>;
defm atomic_load_fadd : ret_noret_binary_atomic_op<atomic_load_fadd, 0>;

def store_hi16_private : StoreHi16 <truncstorei16>, PrivateAddress;
def truncstorei8_hi16_private : StoreHi16<truncstorei8>, PrivateAddress;

def store_atomic_global : GlobalStore<atomic_store>;
def truncstorei8_hi16_global : StoreHi16 <truncstorei8>, GlobalAddress;
def truncstorei16_hi16_global : StoreHi16 <truncstorei16>, GlobalAddress;

def store_local_hi16 : StoreHi16 <truncstorei16>, LocalAddress;
def truncstorei8_local_hi16 : StoreHi16<truncstorei8>, LocalAddress;
def atomic_store_local : LocalStore <atomic_store>;


def load_align8_local : PatFrag <(ops node:$ptr), (load_local node:$ptr)> {
  let IsLoad = 1;
  let IsNonExtLoad = 1;
  let MinAlignment = 8;
}

def load_align16_local : PatFrag <(ops node:$ptr), (load_local node:$ptr)> {
  let IsLoad = 1;
  let IsNonExtLoad = 1;
  let MinAlignment = 16;
}

def store_align8_local: PatFrag<(ops node:$val, node:$ptr),
                                (store_local node:$val, node:$ptr)>, Aligned<8> {
  let IsStore = 1;
  let IsTruncStore = 0;
}

def store_align16_local: PatFrag<(ops node:$val, node:$ptr),
                                (store_local node:$val, node:$ptr)>, Aligned<16> {
  let IsStore = 1;
  let IsTruncStore = 0;
}


def atomic_store_flat  : FlatStore <atomic_store>;
def truncstorei8_hi16_flat  : StoreHi16<truncstorei8>, FlatStoreAddress;
def truncstorei16_hi16_flat : StoreHi16<truncstorei16>, FlatStoreAddress;


class local_binary_atomic_op<SDNode atomic_op> :
  PatFrag<(ops node:$ptr, node:$value),
    (atomic_op node:$ptr, node:$value), [{
  return cast<MemSDNode>(N)->getAddressSpace() == AMDGPUAS::LOCAL_ADDRESS;
}]>;

class region_binary_atomic_op<SDNode atomic_op> :
  PatFrag<(ops node:$ptr, node:$value),
    (atomic_op node:$ptr, node:$value), [{
  return cast<MemSDNode>(N)->getAddressSpace() == AMDGPUAS::REGION_ADDRESS;
}]>;


def mskor_global : PatFrag<(ops node:$val, node:$ptr),
                            (PPUstore_mskor node:$val, node:$ptr), [{
  return cast<MemSDNode>(N)->getAddressSpace() == AMDGPUAS::GLOBAL_ADDRESS;
}]>;

let AddressSpaces = StoreAddress_local.AddrSpaces in {
defm atomic_cmp_swap_local : ternary_atomic_op<atomic_cmp_swap>;
defm atomic_cmp_swap_local_m0 : ternary_atomic_op<atomic_cmp_swap_glue>;
}

let AddressSpaces = StoreAddress_region.AddrSpaces in {
defm atomic_cmp_swap_region : ternary_atomic_op<atomic_cmp_swap>;
defm atomic_cmp_swap_region_m0 : ternary_atomic_op<atomic_cmp_swap_glue>;
}

class global_binary_atomic_op_frag<SDNode atomic_op> : PatFrag<
    (ops node:$ptr, node:$value),
    (atomic_op node:$ptr, node:$value),
    [{return cast<MemSDNode>(N)->getAddressSpace() == AMDGPUAS::GLOBAL_ADDRESS;}]>;

//===----------------------------------------------------------------------===//
// Misc Pattern Fragments
//===----------------------------------------------------------------------===//

class Constants {
int TWO_PI = 0x40c90fdb;
int PI = 0x40490fdb;
int TWO_PI_INV = 0x3e22f983;
int FP_UINT_MAX_PLUS_1 = 0x4f800000;    // 1 << 32 in floating point encoding
int FP16_ONE = 0x3C00;
int FP16_NEG_ONE = 0xBC00;
int FP32_ONE = 0x3f800000;
int FP32_NEG_ONE = 0xbf800000;
int FP64_ONE = 0x3ff0000000000000;
int FP64_NEG_ONE = 0xbff0000000000000;
}
def CONST : Constants;

def FP_ZERO : PatLeaf <
  (fpimm),
  [{return N->getValueAPF().isZero();}]
>;

def FP_ONE : PatLeaf <
  (fpimm),
  [{return N->isExactlyValue(1.0);}]
>;

def FP_HALF : PatLeaf <
  (fpimm),
  [{return N->isExactlyValue(0.5);}]
>;

/* Generic helper patterns for intrinsics */
/* -------------------------------------- */

class POW_Common <PPUBaseInst log_ieee, PPUBaseInst exp_ieee, PPUBaseInst mul>
  : PPUPat <
  (fpow f32:$src0, f32:$src1),
  (exp_ieee (mul f32:$src1, (log_ieee f32:$src0)))
>;

/* Other helper patterns */
/* --------------------- */

/* Extract element pattern */
class Extract_Element <ValueType sub_type, ValueType vec_type, int sub_idx,
                       SubRegIndex sub_reg>
  : PPUPat<
  (sub_type (extractelt vec_type:$src, sub_idx)),
  (EXTRACT_SUBREG $src, sub_reg)
>;

/* Insert element pattern */
class Insert_Element <ValueType elem_type, ValueType vec_type,
                      int sub_idx, SubRegIndex sub_reg>
  : PPUPat <
  (insertelt vec_type:$vec, elem_type:$elem, sub_idx),
  (INSERT_SUBREG $vec, $elem, sub_reg)
>;

// XXX: Convert to new syntax and use COPY_TO_REG, once the DFAPacketizer
// can handle COPY instructions.
// bitconvert pattern
class BitConvert <ValueType dt, ValueType st, RegisterClass rc> : PPUPat <
  (dt (bitconvert (st rc:$src0))),
  (dt rc:$src0)
>;

// XXX: Convert to new syntax and use COPY_TO_REG, once the DFAPacketizer
// can handle COPY instructions.
class DwordAddrPat<ValueType vt, RegisterClass rc> : PPUPat <
  (vt (PPUdwordaddr (vt rc:$addr))),
  (vt rc:$addr)
>;

// BFI_INT patterns

multiclass BFIPatterns <Instruction BFI_INT,
                        Instruction LoadImm32,
                        RegisterClass RC64> {
  // Definition from ISA doc:
  // (y & x) | (z & ~x)
  def : PPUPat <
    (or (and i32:$y, i32:$x), (and i32:$z, (not i32:$x))),
    (BFI_INT $x, $y, $z)
  >;

  // 64-bit version
  def : PPUPat <
    (or (and i64:$y, i64:$x), (and i64:$z, (not i64:$x))),
    (REG_SEQUENCE RC64,
      (BFI_INT (i32 (EXTRACT_SUBREG $x, sub0)),
               (i32 (EXTRACT_SUBREG $y, sub0)),
               (i32 (EXTRACT_SUBREG $z, sub0))), sub0,
      (BFI_INT (i32 (EXTRACT_SUBREG $x, sub1)),
               (i32 (EXTRACT_SUBREG $y, sub1)),
               (i32 (EXTRACT_SUBREG $z, sub1))), sub1)
  >;

  // SHA-256 Ch function
  // z ^ (x & (y ^ z))
  def : PPUPat <
    (xor i32:$z, (and i32:$x, (xor i32:$y, i32:$z))),
    (BFI_INT $x, $y, $z)
  >;

  // 64-bit version
  def : PPUPat <
    (xor i64:$z, (and i64:$x, (xor i64:$y, i64:$z))),
    (REG_SEQUENCE RC64,
      (BFI_INT (i32 (EXTRACT_SUBREG $x, sub0)),
               (i32 (EXTRACT_SUBREG $y, sub0)),
               (i32 (EXTRACT_SUBREG $z, sub0))), sub0,
      (BFI_INT (i32 (EXTRACT_SUBREG $x, sub1)),
               (i32 (EXTRACT_SUBREG $y, sub1)),
               (i32 (EXTRACT_SUBREG $z, sub1))), sub1)
  >;

  def : PPUPat <
    (fcopysign f32:$src0, f32:$src1),
    (BFI_INT (LoadImm32 (i32 0x7fffffff)), $src0, $src1)
  >;

  def : PPUPat <
    (f32 (fcopysign f32:$src0, f64:$src1)),
    (BFI_INT (LoadImm32 (i32 0x7fffffff)), $src0,
             (i32 (EXTRACT_SUBREG $src1, sub1)))
  >;

  def : PPUPat <
    (f64 (fcopysign f64:$src0, f64:$src1)),
    (REG_SEQUENCE RC64,
      (i32 (EXTRACT_SUBREG $src0, sub0)), sub0,
      (BFI_INT (LoadImm32 (i32 0x7fffffff)),
               (i32 (EXTRACT_SUBREG $src0, sub1)),
               (i32 (EXTRACT_SUBREG $src1, sub1))), sub1)
  >;

  def : PPUPat <
    (f64 (fcopysign f64:$src0, f32:$src1)),
    (REG_SEQUENCE RC64,
      (i32 (EXTRACT_SUBREG $src0, sub0)), sub0,
      (BFI_INT (LoadImm32 (i32 0x7fffffff)),
               (i32 (EXTRACT_SUBREG $src0, sub1)),
               $src1), sub1)
  >;
}

// SHA-256 Ma patterns

// ((x & z) | (y & (x | z))) -> BFI_INT (XOR x, y), z, y
multiclass SHA256MaPattern <Instruction BFI_INT, Instruction XOR, RegisterClass RC64> {
  def : PPUPat <
    (or (and i32:$x, i32:$z), (and i32:$y, (or i32:$x, i32:$z))),
    (BFI_INT (XOR i32:$x, i32:$y), i32:$z, i32:$y)
  >;

  def : PPUPat <
    (or (and i64:$x, i64:$z), (and i64:$y, (or i64:$x, i64:$z))),
    (REG_SEQUENCE RC64,
      (BFI_INT (XOR (i32 (EXTRACT_SUBREG $x, sub0)),
                    (i32 (EXTRACT_SUBREG $y, sub0))),
               (i32 (EXTRACT_SUBREG $z, sub0)),
               (i32 (EXTRACT_SUBREG $y, sub0))), sub0,
      (BFI_INT (XOR (i32 (EXTRACT_SUBREG $x, sub1)),
                    (i32 (EXTRACT_SUBREG $y, sub1))),
               (i32 (EXTRACT_SUBREG $z, sub1)),
               (i32 (EXTRACT_SUBREG $y, sub1))), sub1)
  >;
}

// Bitfield extract patterns

def IMMZeroBasedBitfieldMask : PatLeaf <(imm), [{
  return isMask_32(N->getZExtValue());
}]>;

def IMMPopCount : SDNodeXForm<imm, [{
  return CurDAG->getTargetConstant(countPopulation(N->getZExtValue()), SDLoc(N),
                                   MVT::i32);
}]>;

multiclass BFEPattern <Instruction UBFE, Instruction SBFE, Instruction MOV> {
  def : PPUPat <
    (i32 (and (i32 (srl i32:$src, i32:$rshift)), IMMZeroBasedBitfieldMask:$mask)),
    (UBFE $src, $rshift, (MOV (i32 (IMMPopCount $mask))))
  >;

  // x & ((1 << y) - 1)
  def : PPUPat <
    (and i32:$src, (add_oneuse (shl_oneuse 1, i32:$width), -1)),
    (UBFE $src, (MOV (i32 0)), $width)
  >;

  // x & ~(-1 << y)
  def : PPUPat <
    (and i32:$src, (xor_oneuse (shl_oneuse -1, i32:$width), -1)),
    (UBFE $src, (MOV (i32 0)), $width)
  >;

  // x & (-1 >> (bitwidth - y))
  def : PPUPat <
    (and i32:$src, (srl_oneuse -1, (sub 32, i32:$width))),
    (UBFE $src, (MOV (i32 0)), $width)
  >;

  // x << (bitwidth - y) >> (bitwidth - y)
  def : PPUPat <
    (srl (shl_oneuse i32:$src, (sub 32, i32:$width)), (sub 32, i32:$width)),
    (UBFE $src, (MOV (i32 0)), $width)
  >;

  def : PPUPat <
    (sra (shl_oneuse i32:$src, (sub 32, i32:$width)), (sub 32, i32:$width)),
    (SBFE $src, (MOV (i32 0)), $width)
  >;
}

// rotr pattern
class ROTRPattern <Instruction BIT_ALIGN> : PPUPat <
  (rotr i32:$src0, i32:$src1),
  (BIT_ALIGN $src0, $src0, $src1)
>;

multiclass IntMed3Pat<Instruction med3Inst,
                 SDPatternOperator min,
                 SDPatternOperator max,
                 SDPatternOperator min_oneuse,
                 SDPatternOperator max_oneuse,
                 ValueType vt = i32> {

  // This matches 16 permutations of
  // min(max(a, b), max(min(a, b), c))
  def : PPUPat <
  (min (max_oneuse vt:$src0, vt:$src1),
       (max_oneuse (min_oneuse vt:$src0, vt:$src1), vt:$src2)),
  (med3Inst vt:$src0, vt:$src1, vt:$src2)
>;

  // This matches 16 permutations of
  // max(min(x, y), min(max(x, y), z))
  def : PPUPat <
  (max (min_oneuse vt:$src0, vt:$src1),
       (min_oneuse (max_oneuse vt:$src0, vt:$src1), vt:$src2)),
  (med3Inst $src0, $src1, $src2)
>;
}

// Special conversion patterns

def cvt_rpi_i32_f32 : PatFrag <
  (ops node:$src),
  (fp_to_sint (ffloor (fadd $src, FP_HALF))),
  [{ (void) N; return TM.Options.NoNaNsFPMath; }]
>;

def cvt_flr_i32_f32 : PatFrag <
  (ops node:$src),
  (fp_to_sint (ffloor $src)),
  [{ (void)N; return TM.Options.NoNaNsFPMath; }]
>;

let AddedComplexity = 2 in {
class IMad24Pat<Instruction Inst, bit HasClamp = 0> : PPUPat <
  (add (PPUmul_i24 i32:$src0, i32:$src1), i32:$src2),
  !if(HasClamp, (Inst $src0, $src1, $src2, (i1 0)),
                (Inst $src0, $src1, $src2))
>;

class UMad24Pat<Instruction Inst, bit HasClamp = 0> : PPUPat <
  (add (PPUmul_u24 i32:$src0, i32:$src1), i32:$src2),
  !if(HasClamp, (Inst $src0, $src1, $src2, (i1 0)),
                (Inst $src0, $src1, $src2))
>;
} // AddedComplexity.

class RcpPat<Instruction RcpInst, ValueType vt> : PPUPat <
  (fdiv FP_ONE, vt:$src),
  (RcpInst $src)
>;

class RsqPat<Instruction RsqInst, ValueType vt> : PPUPat <
  (PPUrcp (fsqrt vt:$src)),
  (RsqInst $src)
>;

// Instructions which select to the same v_min_f*
def fminnum_like : PatFrags<(ops node:$src0, node:$src1),
  [(fminnum_ieee node:$src0, node:$src1),
   (fminnum node:$src0, node:$src1)]
>;

// Instructions which select to the same v_max_f*
def fmaxnum_like : PatFrags<(ops node:$src0, node:$src1),
  [(fmaxnum_ieee node:$src0, node:$src1),
   (fmaxnum node:$src0, node:$src1)]
>;

def fminnum_like_oneuse : PatFrags<(ops node:$src0, node:$src1),
  [(fminnum_ieee_oneuse node:$src0, node:$src1),
   (fminnum_oneuse node:$src0, node:$src1)]
>;

def fmaxnum_like_oneuse : PatFrags<(ops node:$src0, node:$src1),
  [(fmaxnum_ieee_oneuse node:$src0, node:$src1),
   (fmaxnum_oneuse node:$src0, node:$src1)]
>;
