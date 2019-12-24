//===-- PPUAsmParser.cpp - Parse PPU assembly to MCInst instructions --===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
#include "PPU.h"
#include "PPUKernelCodeT.h"
#include "PPUDefines.h"
#include "PPUInstrInfo.h"
#include "Utils/PPUAsmUtils.h"
#include "Utils/PPUBaseInfo.h"
#include "Utils/PPUKernelCodeTUtils.h"
#include "llvm/ADT/APFloat.h"
#include "llvm/ADT/APInt.h"
#include "llvm/ADT/ArrayRef.h"
#include "llvm/ADT/SmallBitVector.h"
#include "llvm/ADT/SmallString.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/ADT/Twine.h"
#include "llvm/BinaryFormat/ELF.h"
#include "llvm/MC/MCAsmInfo.h"
#include "llvm/MC/MCInstrDesc.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCParser/MCAsmParser.h"
#include "llvm/MC/MCParser/MCAsmParserExtension.h"
#include "llvm/MC/MCSymbol.h"
#include "llvm/Support/PPUMetadata.h"
#include "llvm/Support/AMDHSAKernelDescriptor.h"
#include "llvm/Support/Casting.h"
#include "llvm/Support/Compiler.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/MachineValueType.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/Support/SMLoc.h"
#include "llvm/Support/TargetParser.h"
#include "llvm/Support/TargetRegistry.h"
#include "llvm/Support/raw_ostream.h"


#include "MCTargetDesc/PPUAsmBackend.h"
#include "MCTargetDesc/PPUMCExpr.h"
#include "MCTargetDesc/PPUMCTargetDesc.h"
#include "MCTargetDesc/PPUTargetStreamer.h"
#include "TargetInfo/PPUTargetInfo.h"
#include "Utils/PPUBaseInfo.h"
#include "Utils/PPUMatInt.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/StringSwitch.h"
#include "llvm/CodeGen/Register.h"
#include "llvm/MC/MCAssembler.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCInstBuilder.h"
#include "llvm/MC/MCObjectFileInfo.h"
#include "llvm/MC/MCParser/MCAsmLexer.h"
#include "llvm/MC/MCParser/MCParsedAsmOperand.h"
#include "llvm/MC/MCParser/MCTargetAsmParser.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/Support/Casting.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/Support/TargetRegistry.h"

#include <limits>

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <cstring>
#include <iterator>
#include <map>
#include <memory>
#include <string>

#include "PPUAsmParser.h"


using namespace llvm;
using namespace llvm::PPU;
using namespace llvm::amdhsa;


// TODO we don't use Riscv C
// Include the auto-generated portion of the compress emitter.
// #define GEN_COMPRESS_INSTR
// #include "PPUGenCompressInstEmitter.inc"

// May be called with integer type with equivalent bitwidth.
static const fltSemantics *getFltSemantics(unsigned Size) {
  switch (Size) {
  case 4:
    return &APFloat::IEEEsingle();
  case 8:
    return &APFloat::IEEEdouble();
  case 2:
    return &APFloat::IEEEhalf();
  default:
    llvm_unreachable("unsupported fp type");
  }
}

static const fltSemantics *getFltSemantics(MVT VT) {
  return getFltSemantics(VT.getSizeInBits() / 8);
}

static const fltSemantics *getOpFltSemantics(uint8_t OperandType) {
  switch (OperandType) {
  case PPU::OPERAND_REG_IMM_INT32:
  case PPU::OPERAND_REG_IMM_FP32:
  case PPU::OPERAND_REG_INLINE_C_INT32:
  case PPU::OPERAND_REG_INLINE_C_FP32:
  case PPU::OPERAND_REG_INLINE_AC_INT32:
  case PPU::OPERAND_REG_INLINE_AC_FP32:
    return &APFloat::IEEEsingle();
  case PPU::OPERAND_REG_IMM_INT64:
  case PPU::OPERAND_REG_IMM_FP64:
  case PPU::OPERAND_REG_INLINE_C_INT64:
  case PPU::OPERAND_REG_INLINE_C_FP64:
    return &APFloat::IEEEdouble();
  case PPU::OPERAND_REG_IMM_INT16:
  case PPU::OPERAND_REG_IMM_FP16:
  case PPU::OPERAND_REG_INLINE_C_INT16:
  case PPU::OPERAND_REG_INLINE_C_FP16:
  case PPU::OPERAND_REG_INLINE_C_V2INT16:
  case PPU::OPERAND_REG_INLINE_C_V2FP16:
  case PPU::OPERAND_REG_INLINE_AC_INT16:
  case PPU::OPERAND_REG_INLINE_AC_FP16:
  case PPU::OPERAND_REG_INLINE_AC_V2INT16:
  case PPU::OPERAND_REG_INLINE_AC_V2FP16:
  case PPU::OPERAND_REG_IMM_V2INT16:
  case PPU::OPERAND_REG_IMM_V2FP16:
    return &APFloat::IEEEhalf();
  default:
    llvm_unreachable("unsupported fp type");
  }
}

//===----------------------------------------------------------------------===//
// Operand
//===----------------------------------------------------------------------===//

static bool canLosslesslyConvertToFPType(APFloat &FPLiteral, MVT VT) {
  bool Lost;

  // Convert literal to single precision
  APFloat::opStatus Status = FPLiteral.convert(*getFltSemantics(VT),
                                               APFloat::rmNearestTiesToEven,
                                               &Lost);
  // We allow precision lost but not overflow or underflow
  if (Status != APFloat::opOK &&
      Lost &&
      ((Status & APFloat::opOverflow)  != 0 ||
       (Status & APFloat::opUnderflow) != 0)) {
    return false;
  }

  return true;
}

static bool isSafeTruncation(int64_t Val, unsigned Size) {
  return isUIntN(Size, Val) || isIntN(Size, Val);
}

// move from header file
  template <int N> bool PPUOperand::isBareSimmNLsb0() const {
    int64_t Imm;
    PPUMCExpr::VariantKind VK = PPUMCExpr::VK_PPU_None;
    if (!isImm())
      return false;
    bool IsConstantImm = evaluateConstantImm(getImm(), Imm, VK);
    bool IsValid;
    if (!IsConstantImm)
      IsValid = PPUAsmParser::classifySymbolRef(getImm(), VK, Imm);
    else
      IsValid = isShiftedInt<N - 1, 1>(Imm);
    return IsValid && VK == PPUMCExpr::VK_PPU_None;
  }

  bool PPUOperand::isBareSymbol() const {
    int64_t Imm;
    PPUMCExpr::VariantKind VK = PPUMCExpr::VK_PPU_None;
    // Must be of 'immediate' type but not a constant.
    if (!isImm() || evaluateConstantImm(getImm(), Imm, VK))
      return false;
    return PPUAsmParser::classifySymbolRef(getImm(), VK, Imm) &&
           VK == PPUMCExpr::VK_PPU_None;
  }

  bool PPUOperand::isCallSymbol() const {
    int64_t Imm;
    PPUMCExpr::VariantKind VK = PPUMCExpr::VK_PPU_None;
    // Must be of 'immediate' type but not a constant.
    if (!isImm() || evaluateConstantImm(getImm(), Imm, VK))
      return false;
    return PPUAsmParser::classifySymbolRef(getImm(), VK, Imm) &&
           (VK == PPUMCExpr::VK_PPU_CALL ||
            VK == PPUMCExpr::VK_PPU_CALL_PLT);
  }

  bool PPUOperand::isTPRelAddSymbol() const {
    int64_t Imm;
    PPUMCExpr::VariantKind VK = PPUMCExpr::VK_PPU_None;
    // Must be of 'immediate' type but not a constant.
    if (!isImm() || evaluateConstantImm(getImm(), Imm, VK))
      return false;
    return PPUAsmParser::classifySymbolRef(getImm(), VK, Imm) &&
           VK == PPUMCExpr::VK_PPU_TPREL_ADD;
  }

  bool PPUOperand::isSImm12() const {
    PPUMCExpr::VariantKind VK = PPUMCExpr::VK_PPU_None;
    int64_t Imm;
    bool IsValid;
    if (!isImm())
      return false;
    bool IsConstantImm = evaluateConstantImm(getImm(), Imm, VK);
    if (!IsConstantImm)
      IsValid = PPUAsmParser::classifySymbolRef(getImm(), VK, Imm);
    else
      IsValid = isInt<12>(Imm);
    return IsValid && ((IsConstantImm && VK == PPUMCExpr::VK_PPU_None) ||
                       VK == PPUMCExpr::VK_PPU_LO ||
                       VK == PPUMCExpr::VK_PPU_PCREL_LO ||
                       VK == PPUMCExpr::VK_PPU_TPREL_LO);
  }

  bool PPUOperand::isSImm10Lsb0000NonZero() const {
    if (!isImm())
      return false;
    int64_t Imm;
    PPUMCExpr::VariantKind VK = PPUMCExpr::VK_PPU_None;
    bool IsConstantImm = evaluateConstantImm(getImm(), Imm, VK);
    return IsConstantImm && (Imm != 0) && isShiftedInt<6, 4>(Imm) &&
           VK == PPUMCExpr::VK_PPU_None;
  }

  bool PPUOperand::isUImm20LUI() const {
    PPUMCExpr::VariantKind VK = PPUMCExpr::VK_PPU_None;
    int64_t Imm;
    bool IsValid;
    if (!isImm())
      return false;
    bool IsConstantImm = evaluateConstantImm(getImm(), Imm, VK);
    if (!IsConstantImm) {
      IsValid = PPUAsmParser::classifySymbolRef(getImm(), VK, Imm);
      return IsValid && (VK == PPUMCExpr::VK_PPU_HI ||
                         VK == PPUMCExpr::VK_PPU_TPREL_HI);
    } else {
      return isUInt<20>(Imm) && (VK == PPUMCExpr::VK_PPU_None ||
                                 VK == PPUMCExpr::VK_PPU_HI ||
                                 VK == PPUMCExpr::VK_PPU_TPREL_HI);
    }
  }

  bool PPUOperand::isUImm20AUIPC() const {
    PPUMCExpr::VariantKind VK = PPUMCExpr::VK_PPU_None;
    int64_t Imm;
    bool IsValid;
    if (!isImm())
      return false;
    bool IsConstantImm = evaluateConstantImm(getImm(), Imm, VK);
    if (!IsConstantImm) {
      IsValid = PPUAsmParser::classifySymbolRef(getImm(), VK, Imm);
      return IsValid && (VK == PPUMCExpr::VK_PPU_PCREL_HI ||
                         VK == PPUMCExpr::VK_PPU_GOT_HI ||
                         VK == PPUMCExpr::VK_PPU_TLS_GOT_HI ||
                         VK == PPUMCExpr::VK_PPU_TLS_GD_HI);
    } else {
      return isUInt<20>(Imm) && (VK == PPUMCExpr::VK_PPU_None ||
                                 VK == PPUMCExpr::VK_PPU_PCREL_HI ||
                                 VK == PPUMCExpr::VK_PPU_GOT_HI ||
                                 VK == PPUMCExpr::VK_PPU_TLS_GOT_HI ||
                                 VK == PPUMCExpr::VK_PPU_TLS_GD_HI);
    }
  }

  bool PPUOperand::isSImm21Lsb0JAL() const { return isBareSimmNLsb0<21>(); }

  bool PPUOperand::isImmZero() const {
    if (!isImm())
      return false;
    int64_t Imm;
    PPUMCExpr::VariantKind VK = PPUMCExpr::VK_PPU_None;
    bool IsConstantImm = evaluateConstantImm(getImm(), Imm, VK);
    return IsConstantImm && (Imm == 0) && VK == PPUMCExpr::VK_PPU_None;
  }


bool PPUOperand::isInlinableImm(MVT type) const {

  // This is a hack to enable named inline values like
  // shared_base with both 32-bit and 64-bit operands.
  // Note that these values are defined as
  // 32-bit operands only.
  if (isInlineValue()) {
    return true;
  }

  if (!isImmTy(ImmTyNone)) {
    // Only plain immediates are inlinable (e.g. "clamp" attribute is not)
    return false;
  }
  // TODO: We should avoid using host float here. It would be better to
  // check the float bit values which is what a few other places do.
  // We've had bot failures before due to weird NaN support on mips hosts.

  APInt Literal(64, Imm.Val.Int);

  if (Imm.IsFPImm) { // We got fp literal token
    if (type == MVT::f64 || type == MVT::i64) { // Expected 64-bit operand
      return PPU::isInlinableLiteral64(Imm.Val.Int,
                                          AsmParser->hasInv2PiInlineImm());
    }

    APFloat FPLiteral(APFloat::IEEEdouble(), APInt(64, Imm.Val.Int));
    if (!canLosslesslyConvertToFPType(FPLiteral, type))
      return false;

    if (type.getScalarSizeInBits() == 16) {
      return PPU::isInlinableLiteral16(
        static_cast<int16_t>(FPLiteral.bitcastToAPInt().getZExtValue()),
        AsmParser->hasInv2PiInlineImm());
    }

    // Check if single precision literal is inlinable
    return PPU::isInlinableLiteral32(
      static_cast<int32_t>(FPLiteral.bitcastToAPInt().getZExtValue()),
      AsmParser->hasInv2PiInlineImm());
  }

  // We got int literal token.
  if (type == MVT::f64 || type == MVT::i64) { // Expected 64-bit operand
    return PPU::isInlinableLiteral64(Imm.Val.Int,
                                        AsmParser->hasInv2PiInlineImm());
  }

  if (!isSafeTruncation(Imm.Val.Int, type.getScalarSizeInBits())) {
    return false;
  }

  if (type.getScalarSizeInBits() == 16) {
    return PPU::isInlinableLiteral16(
      static_cast<int16_t>(Literal.getLoBits(16).getSExtValue()),
      AsmParser->hasInv2PiInlineImm());
  }

  return PPU::isInlinableLiteral32(
    static_cast<int32_t>(Literal.getLoBits(32).getZExtValue()),
    AsmParser->hasInv2PiInlineImm());
}

bool PPUOperand::isLiteralImm(MVT type) const {
  // Check that this immediate can be added as literal
  if (!isImmTy(ImmTyNone)) {
    return false;
  }

  if (!Imm.IsFPImm) {
    // We got int literal token.

    if (type == MVT::f64 && hasFPModifiers()) {
      // Cannot apply fp modifiers to int literals preserving the same semantics
      // for VOP1/2/C and VOP3 because of integer truncation. To avoid ambiguity,
      // disable these cases.
      return false;
    }

    unsigned Size = type.getSizeInBits();
    if (Size == 64)
      Size = 32;

    // FIXME: 64-bit operands can zero extend, sign extend, or pad zeroes for FP
    // types.
    return isSafeTruncation(Imm.Val.Int, Size);
  }

  // We got fp literal token
  if (type == MVT::f64) { // Expected 64-bit fp operand
    // We would set low 64-bits of literal to zeroes but we accept this literals
    return true;
  }

  if (type == MVT::i64) { // Expected 64-bit int operand
    // We don't allow fp literals in 64-bit integer instructions. It is
    // unclear how we should encode them.
    return false;
  }

  // We allow fp literals with f16x2 operands assuming that the specified
  // literal goes into the lower half and the upper half is zero. We also
  // require that the literal may be losslesly converted to f16.
  MVT ExpectedType = (type == MVT::v2f16)? MVT::f16 :
                     (type == MVT::v2i16)? MVT::i16 : type;

  APFloat FPLiteral(APFloat::IEEEdouble(), APInt(64, Imm.Val.Int));
  return canLosslesslyConvertToFPType(FPLiteral, ExpectedType);
}

bool PPUOperand::isRegClass(unsigned RCID) const {
  return isRegKind() && AsmParser->getMRI()->getRegClass(RCID).contains(getReg());
}

bool PPUOperand::isSDWAOperand(MVT type) const {
  if (AsmParser->isVI())
    return isVReg32();
  else if (AsmParser->isGFX9() || AsmParser->isGFX10())
    return isRegClass(PPU::VS_32RegClassID) || isInlinableImm(type);
  else
    return false;
}

bool PPUOperand::isSDWAFP16Operand() const {
  return isSDWAOperand(MVT::f16);
}

bool PPUOperand::isSDWAFP32Operand() const {
  return isSDWAOperand(MVT::f32);
}

bool PPUOperand::isSDWAInt16Operand() const {
  return isSDWAOperand(MVT::i16);
}

bool PPUOperand::isSDWAInt32Operand() const {
  return isSDWAOperand(MVT::i32);
}

bool PPUOperand::isBoolReg() const {
  // return // (AsmParser->getFeatureBits()[PPU::FeatureWavefrontSize64] && isSCSrcB64()) ||
        //  (AsmParser->getFeatureBits()[PPU::FeatureWavefrontSize32] && isSCSrcB32());
    return isSCSrcB32();
}

uint64_t PPUOperand::applyInputFPModifiers(uint64_t Val, unsigned Size) const
{
  assert(isImmTy(ImmTyNone) && Imm.Mods.hasFPModifiers());
  assert(Size == 2 || Size == 4 || Size == 8);

  const uint64_t FpSignMask = (1ULL << (Size * 8 - 1));

  if (Imm.Mods.Abs) {
    Val &= ~FpSignMask;
  }
  if (Imm.Mods.Neg) {
    Val ^= FpSignMask;
  }

  return Val;
}

void PPUOperand::addImmOperands(MCInst &Inst, unsigned N, bool ApplyModifiers) const {
  if (PPU::isSISrcOperand(AsmParser->getMII()->get(Inst.getOpcode()),
                             Inst.getNumOperands())) {
    addLiteralImmOperand(Inst, Imm.Val.Int,
                         ApplyModifiers &
                         isImmTy(ImmTyNone) && Imm.Mods.hasFPModifiers());
  } else {
    assert(!isImmTy(ImmTyNone) || !hasModifiers());
    Inst.addOperand(MCOperand::createImm(Imm.Val.Int));
  }
}

void PPUOperand::addLiteralImmOperand(MCInst &Inst, int64_t Val, bool ApplyModifiers) const {
  const auto& InstDesc = AsmParser->getMII()->get(Inst.getOpcode());
  auto OpNum = Inst.getNumOperands();
  // Check that this operand accepts literals
  assert(PPU::isSISrcOperand(InstDesc, OpNum));

  if (ApplyModifiers) {
    assert(PPU::isSISrcFPOperand(InstDesc, OpNum));
    const unsigned Size = Imm.IsFPImm ? sizeof(double) : getOperandSize(InstDesc, OpNum);
    Val = applyInputFPModifiers(Val, Size);
  }

  APInt Literal(64, Val);
  uint8_t OpTy = InstDesc.OpInfo[OpNum].OperandType;

  if (Imm.IsFPImm) { // We got fp literal token
    switch (OpTy) {
    case PPU::OPERAND_REG_IMM_INT64:
    case PPU::OPERAND_REG_IMM_FP64:
    case PPU::OPERAND_REG_INLINE_C_INT64:
    case PPU::OPERAND_REG_INLINE_C_FP64:
      if (PPU::isInlinableLiteral64(Literal.getZExtValue(),
                                       AsmParser->hasInv2PiInlineImm())) {
        Inst.addOperand(MCOperand::createImm(Literal.getZExtValue()));
        return;
      }

      // Non-inlineable
      if (PPU::isSISrcFPOperand(InstDesc, OpNum)) { // Expected 64-bit fp operand
        // For fp operands we check if low 32 bits are zeros
        if (Literal.getLoBits(32) != 0) {
          const_cast<PPUAsmParser *>(AsmParser)->Warning(Inst.getLoc(),
          "Can't encode literal as exact 64-bit floating-point operand. "
          "Low 32-bits will be set to zero");
        }

        Inst.addOperand(MCOperand::createImm(Literal.lshr(32).getZExtValue()));
        return;
      }

      // We don't allow fp literals in 64-bit integer instructions. It is
      // unclear how we should encode them. This case should be checked earlier
      // in predicate methods (isLiteralImm())
      llvm_unreachable("fp literal in 64-bit integer instruction.");

    case PPU::OPERAND_REG_IMM_INT32:
    case PPU::OPERAND_REG_IMM_FP32:
    case PPU::OPERAND_REG_INLINE_C_INT32:
    case PPU::OPERAND_REG_INLINE_C_FP32:
    case PPU::OPERAND_REG_INLINE_AC_INT32:
    case PPU::OPERAND_REG_INLINE_AC_FP32:
    case PPU::OPERAND_REG_IMM_INT16:
    case PPU::OPERAND_REG_IMM_FP16:
    case PPU::OPERAND_REG_INLINE_C_INT16:
    case PPU::OPERAND_REG_INLINE_C_FP16:
    case PPU::OPERAND_REG_INLINE_C_V2INT16:
    case PPU::OPERAND_REG_INLINE_C_V2FP16:
    case PPU::OPERAND_REG_INLINE_AC_INT16:
    case PPU::OPERAND_REG_INLINE_AC_FP16:
    case PPU::OPERAND_REG_INLINE_AC_V2INT16:
    case PPU::OPERAND_REG_INLINE_AC_V2FP16:
    case PPU::OPERAND_REG_IMM_V2INT16:
    case PPU::OPERAND_REG_IMM_V2FP16: {
      bool lost;
      APFloat FPLiteral(APFloat::IEEEdouble(), Literal);
      // Convert literal to single precision
      FPLiteral.convert(*getOpFltSemantics(OpTy),
                        APFloat::rmNearestTiesToEven, &lost);
      // We allow precision lost but not overflow or underflow. This should be
      // checked earlier in isLiteralImm()

      uint64_t ImmVal = FPLiteral.bitcastToAPInt().getZExtValue();
      Inst.addOperand(MCOperand::createImm(ImmVal));
      return;
    }
    default:
      llvm_unreachable("invalid operand size");
    }

    return;
  }

  // We got int literal token.
  // Only sign extend inline immediates.
  switch (OpTy) {
  case PPU::OPERAND_REG_IMM_INT32:
  case PPU::OPERAND_REG_IMM_FP32:
  case PPU::OPERAND_REG_INLINE_C_INT32:
  case PPU::OPERAND_REG_INLINE_C_FP32:
  case PPU::OPERAND_REG_INLINE_AC_INT32:
  case PPU::OPERAND_REG_INLINE_AC_FP32:
  case PPU::OPERAND_REG_IMM_V2INT16:
  case PPU::OPERAND_REG_IMM_V2FP16:
    if (isSafeTruncation(Val, 32) &&
        PPU::isInlinableLiteral32(static_cast<int32_t>(Val),
                                     AsmParser->hasInv2PiInlineImm())) {
      Inst.addOperand(MCOperand::createImm(Val));
      return;
    }

    Inst.addOperand(MCOperand::createImm(Val & 0xffffffff));
    return;

  case PPU::OPERAND_REG_IMM_INT64:
  case PPU::OPERAND_REG_IMM_FP64:
  case PPU::OPERAND_REG_INLINE_C_INT64:
  case PPU::OPERAND_REG_INLINE_C_FP64:
    if (PPU::isInlinableLiteral64(Val, AsmParser->hasInv2PiInlineImm())) {
      Inst.addOperand(MCOperand::createImm(Val));
      return;
    }

    Inst.addOperand(MCOperand::createImm(Lo_32(Val)));
    return;

  case PPU::OPERAND_REG_IMM_INT16:
  case PPU::OPERAND_REG_IMM_FP16:
  case PPU::OPERAND_REG_INLINE_C_INT16:
  case PPU::OPERAND_REG_INLINE_C_FP16:
  case PPU::OPERAND_REG_INLINE_AC_INT16:
  case PPU::OPERAND_REG_INLINE_AC_FP16:
    if (isSafeTruncation(Val, 16) &&
        PPU::isInlinableLiteral16(static_cast<int16_t>(Val),
                                     AsmParser->hasInv2PiInlineImm())) {
      Inst.addOperand(MCOperand::createImm(Val));
      return;
    }

    Inst.addOperand(MCOperand::createImm(Val & 0xffff));
    return;

  case PPU::OPERAND_REG_INLINE_C_V2INT16:
  case PPU::OPERAND_REG_INLINE_C_V2FP16:
  case PPU::OPERAND_REG_INLINE_AC_V2INT16:
  case PPU::OPERAND_REG_INLINE_AC_V2FP16: {
    assert(isSafeTruncation(Val, 16));
    assert(PPU::isInlinableLiteral16(static_cast<int16_t>(Val),
                                        AsmParser->hasInv2PiInlineImm()));

    Inst.addOperand(MCOperand::createImm(Val));
    return;
  }
  default:
    llvm_unreachable("invalid operand size");
  }
}

template <unsigned Bitwidth>
void PPUOperand::addKImmFPOperands(MCInst &Inst, unsigned N) const {
  APInt Literal(64, Imm.Val.Int);

  if (!Imm.IsFPImm) {
    // We got int literal token.
    Inst.addOperand(MCOperand::createImm(Literal.getLoBits(Bitwidth).getZExtValue()));
    return;
  }

  bool Lost;
  APFloat FPLiteral(APFloat::IEEEdouble(), Literal);
  FPLiteral.convert(*getFltSemantics(Bitwidth / 8),
                    APFloat::rmNearestTiesToEven, &Lost);
  Inst.addOperand(MCOperand::createImm(FPLiteral.bitcastToAPInt().getZExtValue()));
}

void PPUOperand::addRegOperands(MCInst &Inst, unsigned N) const {
  Inst.addOperand(MCOperand::createReg(PPU::getMCReg(getReg(), AsmParser->getSTI())));
}

static bool isInlineValue(unsigned Reg) {
  switch (Reg) {
  case PPU::SRC_SHARED_BASE:
  case PPU::SRC_SHARED_LIMIT:
  case PPU::SRC_PRIVATE_BASE:
  case PPU::SRC_PRIVATE_LIMIT:
  case PPU::SRC_POPS_EXITING_WAVE_ID:
    return true;
  case PPU::SRC_VCCZ:
  case PPU::SRC_TMSKZ:
  case PPU::SRC_SCC:
    return true;
  default:
    return false;
  }
}

bool PPUOperand::isInlineValue() const {
  return isRegKind() && ::isInlineValue(getReg());
}

//===----------------------------------------------------------------------===//
// AsmParser
//===----------------------------------------------------------------------===//

static int getRegClass(RegisterKind Is, unsigned RegWidth) {
  if (Is == IS_VGPR) {
    switch (RegWidth) {
      default: return -1;
      case 1: return PPU::VPR_32RegClassID;
      case 2: return PPU::VReg_64RegClassID;
      /*
      case 3: return PPU::VReg_96RegClassID;
      case 4: return PPU::VReg_128RegClassID;
      case 5: return PPU::VReg_160RegClassID;
      case 8: return PPU::VReg_256RegClassID;
      case 16: return PPU::VReg_512RegClassID;
      case 32: return PPU::VReg_1024RegClassID;
      */
    }
  } else if (Is == IS_TTMP) {
      /*
    switch (RegWidth) {
      default: return -1;
      case 1: return PPU::TTMP_32RegClassID;
      case 2: return PPU::TTMP_64RegClassID;
      case 4: return PPU::TTMP_128RegClassID;
      case 8: return PPU::TTMP_256RegClassID;
      case 16: return PPU::TTMP_512RegClassID;
    }
      */
  } else if (Is == IS_SGPR) {
    switch (RegWidth) {
      default: return -1;
      case 1: return PPU::SPR_32RegClassID;
      case 2: return PPU::SPR_64RegClassID;
      /*
      case 4: return PPU::SGPR_128RegClassID;
      case 8: return PPU::SGPR_256RegClassID;
      case 16: return PPU::SGPR_512RegClassID;
      */
    }
    /*
  } else if (Is == IS_AGPR) {
    switch (RegWidth) {
      default: return -1;
      case 1: return PPU::AGPR_32RegClassID;
      case 2: return PPU::AReg_64RegClassID;
      case 4: return PPU::AReg_128RegClassID;
      case 16: return PPU::AReg_512RegClassID;
      case 32: return PPU::AReg_1024RegClassID;
    }
    */
  }
  return -1;
}

static unsigned getSpecialRegForName(StringRef RegName) {
  return StringSwitch<unsigned>(RegName)
    .Case("tmsk", PPU::TMSK)
    .Case("vcc", PPU::VCC)
    .Case("flat_scratch", PPU::FLAT_SCR)
    .Case("shared_base", PPU::SRC_SHARED_BASE)
    .Case("src_shared_base", PPU::SRC_SHARED_BASE)
    .Case("shared_limit", PPU::SRC_SHARED_LIMIT)
    .Case("src_shared_limit", PPU::SRC_SHARED_LIMIT)
    .Case("private_base", PPU::SRC_PRIVATE_BASE)
    .Case("src_private_base", PPU::SRC_PRIVATE_BASE)
    .Case("private_limit", PPU::SRC_PRIVATE_LIMIT)
    .Case("src_private_limit", PPU::SRC_PRIVATE_LIMIT)
    .Case("pops_exiting_wave_id", PPU::SRC_POPS_EXITING_WAVE_ID)
    .Case("src_pops_exiting_wave_id", PPU::SRC_POPS_EXITING_WAVE_ID)
    .Case("lds_direct", PPU::LDS_DIRECT)
    .Case("src_lds_direct", PPU::LDS_DIRECT)
    .Case("m0", PPU::M0)
    .Case("vccz", PPU::SRC_VCCZ)
    .Case("src_vccz", PPU::SRC_VCCZ)
    .Case("execz", PPU::SRC_TMSKZ)
    .Case("src_execz", PPU::SRC_TMSKZ)
    .Case("scc", PPU::SRC_SCC)
    .Case("src_scc", PPU::SRC_SCC)
    /*
    .Case("tba", PPU::TBA)
    .Case("tma", PPU::TMA)
    */
    .Case("flat_scratch_lo", PPU::FLAT_SCR_LO)
    .Case("flat_scratch_hi", PPU::FLAT_SCR_HI)
    /*
    .Case("tma_lo", PPU::TMA_LO)
    .Case("tma_hi", PPU::TMA_HI)
    .Case("tba_lo", PPU::TBA_LO)
    .Case("tba_hi", PPU::TBA_HI)
    */
    .Case("null", PPU::SPR_NULL)
    .Default(0);
}
/* logic move to ricv
bool PPUAsmParser::ParseRegister(unsigned &RegNo, SMLoc &StartLoc,
                                    SMLoc &EndLoc) {
  auto R = parseRegister();
  if (!R) return true;
  assert(R->isReg());
  RegNo = R->getReg();
  StartLoc = R->getStartLoc();
  EndLoc = R->getEndLoc();
  return false;
}
*/

bool PPUAsmParser::AddNextRegisterToList(unsigned &Reg, unsigned &RegWidth,
                                            RegisterKind RegKind, unsigned Reg1,
                                            unsigned RegNum) {
  switch (RegKind) {
  case IS_SPECIAL:
    if (Reg == PPU::FLAT_SCR_LO && Reg1 == PPU::FLAT_SCR_HI) {
      Reg = PPU::FLAT_SCR;
      RegWidth = 2;
      return true;
    }
    /*
    if (Reg == PPU::TBA_LO && Reg1 == PPU::TBA_HI) {
      Reg = PPU::TBA;
      RegWidth = 2;
      return true;
    }
    if (Reg == PPU::TMA_LO && Reg1 == PPU::TMA_HI) {
      Reg = PPU::TMA;
      RegWidth = 2;
      return true;
    }
    */
    return false;
  case IS_VGPR:
  case IS_SGPR:
  case IS_AGPR:
  case IS_TTMP:
    if (Reg1 != Reg + RegWidth) {
      return false;
    }
    RegWidth++;
    return true;
  default:
    llvm_unreachable("unexpected register kind");
  }
}

static const StringRef Registers[] = {
  { "v" },
  { "s" },
  { "ttmp" },
  { "acc" },
  { "a" },
};

bool
PPUAsmParser::isRegister(const AsmToken &Token,
                            const AsmToken &NextToken) const {

  // A list of consecutive registers: [s0,s1,s2,s3]
  if (Token.is(AsmToken::LBrac))
    return true;

  if (!Token.is(AsmToken::Identifier))
    return false;

  // A single register like s0 or a range of registers like s[0:1]

  StringRef RegName = Token.getString();

  for (StringRef Reg : Registers) {
    if (RegName.startswith(Reg)) {
      if (Reg.size() < RegName.size()) {
        unsigned RegNum;
        // A single register with an index: rXX
        if (!RegName.substr(Reg.size()).getAsInteger(10, RegNum))
          return true;
      } else {
        // A range of registers: r[XX:YY].
        if (NextToken.is(AsmToken::LBrac))
          return true;
      }
    }
  }

  return getSpecialRegForName(RegName);
}

bool
PPUAsmParser::isRegister()
{
  return isRegister(getToken(), peekToken());
}

bool PPUAsmParser::ParsePPURegister(RegisterKind &RegKind, unsigned &Reg,
                                          unsigned &RegNum, unsigned &RegWidth,
                                          unsigned *DwordRegIndex) {
  if (DwordRegIndex) { *DwordRegIndex = 0; }
  const MCRegisterInfo *TRI = getContext().getRegisterInfo();
  if (getLexer().is(AsmToken::Identifier)) {
    StringRef RegName = Parser.getTok().getString();
    if ((Reg = getSpecialRegForName(RegName))) {
      Parser.Lex();
      RegKind = IS_SPECIAL;
    } else {
      unsigned RegNumIndex = 0;
      if (RegName[0] == 'v') {
        RegNumIndex = 1;
        RegKind = IS_VGPR;
      } else if (RegName[0] == 's') {
        RegNumIndex = 1;
        RegKind = IS_SGPR;
      } else if (RegName[0] == 'a') {
        RegNumIndex = RegName.startswith("acc") ? 3 : 1;
        RegKind = IS_AGPR;
      } else if (RegName.startswith("ttmp")) {
        RegNumIndex = strlen("ttmp");
        RegKind = IS_TTMP;
      } else {
        return false;
      }
      if (RegName.size() > RegNumIndex) {
        // Single 32-bit register: vXX.
        if (RegName.substr(RegNumIndex).getAsInteger(10, RegNum))
          return false;
        Parser.Lex();
        RegWidth = 1;
      } else {
        // Range of registers: v[XX:YY]. ":YY" is optional.
        Parser.Lex();
        int64_t RegLo, RegHi;
        if (getLexer().isNot(AsmToken::LBrac))
          return false;
        Parser.Lex();

        if (getParser().parseAbsoluteExpression(RegLo))
          return false;

        const bool isRBrace = getLexer().is(AsmToken::RBrac);
        if (!isRBrace && getLexer().isNot(AsmToken::Colon))
          return false;
        Parser.Lex();

        if (isRBrace) {
          RegHi = RegLo;
        } else {
          if (getParser().parseAbsoluteExpression(RegHi))
            return false;

          if (getLexer().isNot(AsmToken::RBrac))
            return false;
          Parser.Lex();
        }
        RegNum = (unsigned) RegLo;
        RegWidth = (RegHi - RegLo) + 1;
      }
    }
  } else if (getLexer().is(AsmToken::LBrac)) {
    // List of consecutive registers: [s0,s1,s2,s3]
    Parser.Lex();
    if (!ParsePPURegister(RegKind, Reg, RegNum, RegWidth, nullptr))
      return false;
    if (RegWidth != 1)
      return false;
    RegisterKind RegKind1;
    unsigned Reg1, RegNum1, RegWidth1;
    do {
      if (getLexer().is(AsmToken::Comma)) {
        Parser.Lex();
      } else if (getLexer().is(AsmToken::RBrac)) {
        Parser.Lex();
        break;
      } else if (ParsePPURegister(RegKind1, Reg1, RegNum1, RegWidth1, nullptr)) {
        if (RegWidth1 != 1) {
          return false;
        }
        if (RegKind1 != RegKind) {
          return false;
        }
        if (!AddNextRegisterToList(Reg, RegWidth, RegKind1, Reg1, RegNum1)) {
          return false;
        }
      } else {
        return false;
      }
    } while (true);
  } else {
    return false;
  }
  switch (RegKind) {
  case IS_SPECIAL:
    RegNum = 0;
    RegWidth = 1;
    break;
  case IS_VGPR:
  case IS_SGPR:
  case IS_AGPR:
  case IS_TTMP:
  {
    unsigned Size = 1;
    if (RegKind == IS_SGPR || RegKind == IS_TTMP) {
      // SGPR and TTMP registers must be aligned. Max required alignment is 4 dwords.
      Size = std::min(RegWidth, 4u);
    }
    if (RegNum % Size != 0)
      return false;
    if (DwordRegIndex) { *DwordRegIndex = RegNum; }
    RegNum = RegNum / Size;
    int RCID = getRegClass(RegKind, RegWidth);
    if (RCID == -1)
      return false;
    const MCRegisterClass RC = TRI->getRegClass(RCID);
    if (RegNum >= RC.getNumRegs())
      return false;
    Reg = RC.getRegister(RegNum);
    break;
  }

  default:
    llvm_unreachable("unexpected register kind");
  }
/*
  if (!subtargetHasRegister(*TRI, Reg))
    return false;
    */
  return true;
}

Optional<StringRef>
PPUAsmParser::getGprCountSymbolName(RegisterKind RegKind) {
  switch (RegKind) {
  case IS_VGPR:
    return StringRef(".amdgcn.next_free_vgpr");
  case IS_SGPR:
    return StringRef(".amdgcn.next_free_sgpr");
  default:
    return None;
  }
}

void PPUAsmParser::initializeGprCountSymbol(RegisterKind RegKind) {
  auto SymbolName = getGprCountSymbolName(RegKind);
  assert(SymbolName && "initializing invalid register kind");
  MCSymbol *Sym = getContext().getOrCreateSymbol(*SymbolName);
  Sym->setVariableValue(MCConstantExpr::create(0, getContext()));
}

bool PPUAsmParser::updateGprCountSymbols(RegisterKind RegKind,
                                            unsigned DwordRegIndex,
                                            unsigned RegWidth) {
  // Symbols are only defined for GCN targets
  /*
  if (PPU::getIsaVersion(getSTI().getCPU()).Major < 6)
    return true;
    */

  auto SymbolName = getGprCountSymbolName(RegKind);
  if (!SymbolName)
    return true;
  MCSymbol *Sym = getContext().getOrCreateSymbol(*SymbolName);

  int64_t NewMax = DwordRegIndex + RegWidth - 1;
  int64_t OldCount;

  if (!Sym->isVariable())
    return !Error(getParser().getTok().getLoc(),
                  ".amdgcn.next_free_{v,s}gpr symbols must be variable");
  if (!Sym->getVariableValue(false)->evaluateAsAbsolute(OldCount))
    return !Error(
        getParser().getTok().getLoc(),
        ".amdgcn.next_free_{v,s}gpr symbols must be absolute expressions");

  if (OldCount <= NewMax)
    Sym->setVariableValue(MCConstantExpr::create(NewMax + 1, getContext()));

  return true;
}

std::unique_ptr<PPUOperand> PPUAsmParser::parseRegister() {
  const auto &Tok = Parser.getTok();
  SMLoc StartLoc = Tok.getLoc();
  SMLoc EndLoc = Tok.getEndLoc();
  RegisterKind RegKind;
  unsigned Reg, RegNum, RegWidth, DwordRegIndex;

  if (!ParsePPURegister(RegKind, Reg, RegNum, RegWidth, &DwordRegIndex)) {
    //FIXME: improve error messages (bug 41303).
    Error(StartLoc, "not a valid operand.");
    return nullptr;
  }
  if (PPU::IsaInfo::hasCodeObjectV3(&getSTI())) {
    if (!updateGprCountSymbols(RegKind, DwordRegIndex, RegWidth))
      return nullptr;
  }
  return PPUOperand::CreateReg(this, Reg, StartLoc, EndLoc);
}

OperandMatchResultTy
PPUAsmParser::parseImm(OperandVector &Operands, bool HasSP3AbsModifier) {
  // TODO: add syntactic sugar for 1/(2*PI)

  assert(!isRegister());
  assert(!isModifier());

  const auto& Tok = getToken();
  const auto& NextTok = peekToken();
  bool IsReal = Tok.is(AsmToken::Real);
  SMLoc S = getLoc();
  bool Negate = false;

  if (!IsReal && Tok.is(AsmToken::Minus) && NextTok.is(AsmToken::Real)) {
    lex();
    IsReal = true;
    Negate = true;
  }

  if (IsReal) {
    // Floating-point expressions are not supported.
    // Can only allow floating-point literals with an
    // optional sign.

    StringRef Num = getTokenStr();
    lex();

    APFloat RealVal(APFloat::IEEEdouble());
    auto roundMode = APFloat::rmNearestTiesToEven;
    if (RealVal.convertFromString(Num, roundMode) == APFloat::opInvalidOp) {
      return MatchOperand_ParseFail;
    }
    if (Negate)
      RealVal.changeSign();

    Operands.push_back(
      PPUOperand::CreateImm(this, RealVal.bitcastToAPInt().getZExtValue(), S,
                               PPUOperand::ImmTyNone, true));

    return MatchOperand_Success;

  } else {
    int64_t IntVal;
    const MCExpr *Expr;
    SMLoc S = getLoc();

    if (HasSP3AbsModifier) {
      // This is a workaround for handling expressions
      // as arguments of SP3 'abs' modifier, for example:
      //     |1.0|
      //     |-1|
      //     |1+x|
      // This syntax is not compatible with syntax of standard
      // MC expressions (due to the trailing '|').
      SMLoc EndLoc;
      if (getParser().parsePrimaryExpr(Expr, EndLoc))
        return MatchOperand_ParseFail;
    } else {
      if (Parser.parseExpression(Expr))
        return MatchOperand_ParseFail;
    }

    if (Expr->evaluateAsAbsolute(IntVal)) {
      Operands.push_back(PPUOperand::CreateImm(this, IntVal, S));
    } else {
      Operands.push_back(PPUOperand::CreateExpr(this, Expr, S));
    }

    return MatchOperand_Success;
  }

  return MatchOperand_NoMatch;
}

OperandMatchResultTy
PPUAsmParser::parseReg(OperandVector &Operands) {
  if (!isRegister())
    return MatchOperand_NoMatch;

  if (auto R = parseRegister()) {
    assert(R->isReg());
    Operands.push_back(std::move(R));
    return MatchOperand_Success;
  }
  return MatchOperand_ParseFail;
}

OperandMatchResultTy
PPUAsmParser::parseRegOrImm(OperandVector &Operands, bool HasSP3AbsMod) {
  auto res = parseReg(Operands);
  if (res != MatchOperand_NoMatch) {
    return res;
  } else if (isModifier()) {
    return MatchOperand_NoMatch;
  } else {
    return parseImm(Operands, HasSP3AbsMod);
  }
}

bool
PPUAsmParser::isNamedOperandModifier(const AsmToken &Token, const AsmToken &NextToken) const {
  if (Token.is(AsmToken::Identifier) && NextToken.is(AsmToken::LParen)) {
    const auto &str = Token.getString();
    return str == "abs" || str == "neg" || str == "sext";
  }
  return false;
}

bool
PPUAsmParser::isOpcodeModifierWithVal(const AsmToken &Token, const AsmToken &NextToken) const {
  return Token.is(AsmToken::Identifier) && NextToken.is(AsmToken::Colon);
}

bool
PPUAsmParser::isOperandModifier(const AsmToken &Token, const AsmToken &NextToken) const {
  return isNamedOperandModifier(Token, NextToken) || Token.is(AsmToken::Pipe);
}

bool
PPUAsmParser::isRegOrOperandModifier(const AsmToken &Token, const AsmToken &NextToken) const {
  return isRegister(Token, NextToken) || isOperandModifier(Token, NextToken);
}

// Check if this is an operand modifier or an opcode modifier
// which may look like an expression but it is not. We should
// avoid parsing these modifiers as expressions. Currently
// recognized sequences are:
//   |...|
//   abs(...)
//   neg(...)
//   sext(...)
//   -reg
//   -|...|
//   -abs(...)
//   name:...
// Note that simple opcode modifiers like 'gds' may be parsed as
// expressions; this is a special case. See getExpressionAsToken.
//
bool
PPUAsmParser::isModifier() {

  AsmToken Tok = getToken();
  AsmToken NextToken[2];
  peekTokens(NextToken);

  return isOperandModifier(Tok, NextToken[0]) ||
         (Tok.is(AsmToken::Minus) && isRegOrOperandModifier(NextToken[0], NextToken[1])) ||
         isOpcodeModifierWithVal(Tok, NextToken[0]);
}

// Check if the current token is an SP3 'neg' modifier.
// Currently this modifier is allowed in the following context:
//
// 1. Before a register, e.g. "-v0", "-v[...]" or "-[v0,v1]".
// 2. Before an 'abs' modifier: -abs(...)
// 3. Before an SP3 'abs' modifier: -|...|
//
// In all other cases "-" is handled as a part
// of an expression that follows the sign.
//
// Note: When "-" is followed by an integer literal,
// this is interpreted as integer negation rather
// than a floating-point NEG modifier applied to N.
// Beside being contr-intuitive, such use of floating-point
// NEG modifier would have resulted in different meaning
// of integer literals used with VOP1/2/C and VOP3,
// for example:
//    v_exp_f32_e32 v5, -1 // VOP1: src0 = 0xFFFFFFFF
//    v_exp_f32_e64 v5, -1 // VOP3: src0 = 0x80000001
// Negative fp literals with preceding "-" are
// handled likewise for unifomtity
//
bool
PPUAsmParser::parseSP3NegModifier() {

  AsmToken NextToken[2];
  peekTokens(NextToken);

  if (isToken(AsmToken::Minus) &&
      (isRegister(NextToken[0], NextToken[1]) ||
       NextToken[0].is(AsmToken::Pipe) ||
       isId(NextToken[0], "abs"))) {
    lex();
    return true;
  }

  return false;
}

OperandMatchResultTy
PPUAsmParser::parseRegOrImmWithFPInputMods(OperandVector &Operands,
                                              bool AllowImm) {
  bool Neg, SP3Neg;
  bool Abs, SP3Abs;
  SMLoc Loc;

  // Disable ambiguous constructs like '--1' etc. Should use neg(-1) instead.
  if (isToken(AsmToken::Minus) && peekToken().is(AsmToken::Minus)) {
    Error(getLoc(), "invalid syntax, expected 'neg' modifier");
    return MatchOperand_ParseFail;
  }

  SP3Neg = parseSP3NegModifier();

  Loc = getLoc();
  Neg = trySkipId("neg");
  if (Neg && SP3Neg) {
    Error(Loc, "expected register or immediate");
    return MatchOperand_ParseFail;
  }
  if (Neg && !skipToken(AsmToken::LParen, "expected left paren after neg"))
    return MatchOperand_ParseFail;

  Abs = trySkipId("abs");
  if (Abs && !skipToken(AsmToken::LParen, "expected left paren after abs"))
    return MatchOperand_ParseFail;

  Loc = getLoc();
  SP3Abs = trySkipToken(AsmToken::Pipe);
  if (Abs && SP3Abs) {
    Error(Loc, "expected register or immediate");
    return MatchOperand_ParseFail;
  }

  OperandMatchResultTy Res;
  if (AllowImm) {
    Res = parseRegOrImm(Operands, SP3Abs);
  } else {
    Res = parseReg(Operands);
  }
  if (Res != MatchOperand_Success) {
    return (SP3Neg || Neg || SP3Abs || Abs)? MatchOperand_ParseFail : Res;
  }

  if (SP3Abs && !skipToken(AsmToken::Pipe, "expected vertical bar"))
    return MatchOperand_ParseFail;
  if (Abs && !skipToken(AsmToken::RParen, "expected closing parentheses"))
    return MatchOperand_ParseFail;
  if (Neg && !skipToken(AsmToken::RParen, "expected closing parentheses"))
    return MatchOperand_ParseFail;

  PPUOperand::Modifiers Mods;
  Mods.Abs = Abs || SP3Abs;
  Mods.Neg = Neg || SP3Neg;

  if (Mods.hasFPModifiers()) {
    PPUOperand &Op = static_cast<PPUOperand &>(*Operands.back());
    if (Op.isExpr()) {
      Error(Op.getStartLoc(), "expected an absolute expression");
      return MatchOperand_ParseFail;
    }
    Op.setModifiers(Mods);
  }
  return MatchOperand_Success;
}

OperandMatchResultTy
PPUAsmParser::parseRegOrImmWithIntInputMods(OperandVector &Operands,
                                               bool AllowImm) {
  bool Sext = trySkipId("sext");
  if (Sext && !skipToken(AsmToken::LParen, "expected left paren after sext"))
    return MatchOperand_ParseFail;

  OperandMatchResultTy Res;
  if (AllowImm) {
    Res = parseRegOrImm(Operands);
  } else {
    Res = parseReg(Operands);
  }
  if (Res != MatchOperand_Success) {
    return Sext? MatchOperand_ParseFail : Res;
  }

  if (Sext && !skipToken(AsmToken::RParen, "expected closing parentheses"))
    return MatchOperand_ParseFail;

  PPUOperand::Modifiers Mods;
  Mods.Sext = Sext;

  if (Mods.hasIntModifiers()) {
    PPUOperand &Op = static_cast<PPUOperand &>(*Operands.back());
    if (Op.isExpr()) {
      Error(Op.getStartLoc(), "expected an absolute expression");
      return MatchOperand_ParseFail;
    }
    Op.setModifiers(Mods);
  }

  return MatchOperand_Success;
}

OperandMatchResultTy
PPUAsmParser::parseRegWithFPInputMods(OperandVector &Operands) {
  return parseRegOrImmWithFPInputMods(Operands, false);
}

OperandMatchResultTy
PPUAsmParser::parseRegWithIntInputMods(OperandVector &Operands) {
  return parseRegOrImmWithIntInputMods(Operands, false);
}

OperandMatchResultTy PPUAsmParser::parseVReg32OrOff(OperandVector &Operands) {
  auto Loc = getLoc();
  if (trySkipId("off")) {
    Operands.push_back(PPUOperand::CreateImm(this, 0, Loc,
                                                PPUOperand::ImmTyOff, false));
    return MatchOperand_Success;
  }

  if (!isRegister())
    return MatchOperand_NoMatch;

  std::unique_ptr<PPUOperand> Reg = parseRegister();
  if (Reg) {
    Operands.push_back(std::move(Reg));
    return MatchOperand_Success;
  }

  return MatchOperand_ParseFail;

}

unsigned PPUAsmParser::checkTargetMatchPredicate(MCInst &Inst) {
  uint64_t TSFlags = MII.get(Inst.getOpcode()).TSFlags;

  if ((getForcedEncodingSize() == 32 && (TSFlags & PPUInstrFlags::VOP3)) ||
      (getForcedEncodingSize() == 64 && !(TSFlags & PPUInstrFlags::VOP3))) 
    return Match_InvalidOperand;

  if ((TSFlags & PPUInstrFlags::VOP3) &&
      (TSFlags & PPUInstrFlags::VOPAsmPrefer32Bit) &&
      getForcedEncodingSize() != 64)
    return Match_PreferE32;

  return Match_Success;
}

// What asm variants we should check
ArrayRef<unsigned> PPUAsmParser::getMatchedVariants() const {
  if (getForcedEncodingSize() == 32) {
    static const unsigned Variants[] = {PPUAsmVariants::DEFAULT};
    return makeArrayRef(Variants);
  }

  if (isForcedVOP3()) {
    static const unsigned Variants[] = {PPUAsmVariants::VOP3};
    return makeArrayRef(Variants);
  }

  if (isForcedSDWA()) {
    static const unsigned Variants[] = {PPUAsmVariants::SDWA,
                                        PPUAsmVariants::SDWA9};
    return makeArrayRef(Variants);
  }

  if (isForcedDPP()) {
    static const unsigned Variants[] = {PPUAsmVariants::DPP};
    return makeArrayRef(Variants);
  }

  static const unsigned Variants[] = {
    PPUAsmVariants::DEFAULT, PPUAsmVariants::VOP3,
    PPUAsmVariants::SDWA, PPUAsmVariants::SDWA9, PPUAsmVariants::DPP
  };

  return makeArrayRef(Variants);
}

unsigned PPUAsmParser::findImplicitSGPRReadInVOP(const MCInst &Inst) const {
  const MCInstrDesc &Desc = MII.get(Inst.getOpcode());
  const unsigned Num = Desc.getNumImplicitUses();
  for (unsigned i = 0; i < Num; ++i) {
    unsigned Reg = Desc.ImplicitUses[i];
    switch (Reg) {
    case PPU::FLAT_SCR:
    case PPU::VCC:
    case PPU::M0:
    case PPU::SPR_NULL:
      return Reg;
    default:
      break;
    }
  }
  return PPU::NoRegister;
}

// NB: This code is correct only when used to check constant
// bus limitations because GFX7 support no f16 inline constants.
// Note that there are no cases when a GFX7 opcode violates
// constant bus limitations due to the use of an f16 constant.
bool PPUAsmParser::isInlineConstant(const MCInst &Inst,
                                       unsigned OpIdx) const {
  const MCInstrDesc &Desc = MII.get(Inst.getOpcode());

  if (!PPU::isSISrcOperand(Desc, OpIdx)) {
    return false;
  }

  const MCOperand &MO = Inst.getOperand(OpIdx);

  int64_t Val = MO.getImm();
  auto OpSize = PPU::getOperandSize(Desc, OpIdx);

  switch (OpSize) { // expected operand size
  case 8:
    return PPU::isInlinableLiteral64(Val, hasInv2PiInlineImm());
  case 4:
    return PPU::isInlinableLiteral32(Val, hasInv2PiInlineImm());
  case 2: {
    const unsigned OperandType = Desc.OpInfo[OpIdx].OperandType;
    if (OperandType == PPU::OPERAND_REG_INLINE_C_V2INT16 ||
        OperandType == PPU::OPERAND_REG_INLINE_C_V2FP16 ||
        OperandType == PPU::OPERAND_REG_INLINE_AC_V2INT16 ||
        OperandType == PPU::OPERAND_REG_INLINE_AC_V2FP16 ||
        OperandType == PPU::OPERAND_REG_IMM_V2INT16 ||
        OperandType == PPU::OPERAND_REG_IMM_V2FP16) {
      return PPU::isInlinableLiteralV216(Val, hasInv2PiInlineImm());
    } else {
      return PPU::isInlinableLiteral16(Val, hasInv2PiInlineImm());
    }
  }
  default:
    llvm_unreachable("invalid operand size");
  }
}

bool PPUAsmParser::usesConstantBus(const MCInst &Inst, unsigned OpIdx) {
  const MCOperand &MO = Inst.getOperand(OpIdx);
  if (MO.isImm()) {
    return !isInlineConstant(Inst, OpIdx);
  }
  return !MO.isReg() ||
         isSGPR(mc2PseudoReg(MO.getReg()), getContext().getRegisterInfo());
}

bool PPUAsmParser::validateConstantBusLimitations(const MCInst &Inst) {
  const unsigned Opcode = Inst.getOpcode();
  const MCInstrDesc &Desc = MII.get(Opcode);
  unsigned ConstantBusUseCount = 0;
  unsigned NumLiterals = 0;
  unsigned LiteralSize;

  if (Desc.TSFlags &
      (PPUInstrFlags::VOPC |
       PPUInstrFlags::VOP1 | PPUInstrFlags::VOP2 |
       PPUInstrFlags::VOP3 | PPUInstrFlags::VOP3P |
       PPUInstrFlags::SDWA)) {
    // Check special imm operands (used by madmk, etc)
    if (PPU::getNamedOperandIdx(Opcode, PPU::OpName::imm) != -1) {
      ++ConstantBusUseCount;
    }

    SmallDenseSet<unsigned> SGPRsUsed;
    unsigned SGPRUsed = findImplicitSGPRReadInVOP(Inst);
    if (SGPRUsed != PPU::NoRegister) {
      SGPRsUsed.insert(SGPRUsed);
      ++ConstantBusUseCount;
    }

    const int Src0Idx = PPU::getNamedOperandIdx(Opcode, PPU::OpName::src0);
    const int Src1Idx = PPU::getNamedOperandIdx(Opcode, PPU::OpName::src1);
    const int Src2Idx = PPU::getNamedOperandIdx(Opcode, PPU::OpName::src2);

    const int OpIndices[] = { Src0Idx, Src1Idx, Src2Idx };

    for (int OpIdx : OpIndices) {
      if (OpIdx == -1) break;

      const MCOperand &MO = Inst.getOperand(OpIdx);
      if (usesConstantBus(Inst, OpIdx)) {
        if (MO.isReg()) {
          const unsigned Reg = mc2PseudoReg(MO.getReg());
          // Pairs of registers with a partial intersections like these
          //   s0, s[0:1]
          //   flat_scratch_lo, flat_scratch
          //   flat_scratch_lo, flat_scratch_hi
          // are theoretically valid but they are disabled anyway.
          // Note that this code mimics PPUInstrInfo::verifyInstruction
          if (!SGPRsUsed.count(Reg)) {
            SGPRsUsed.insert(Reg);
            ++ConstantBusUseCount;
          }
        } else { // Expression or a literal

          if (Desc.OpInfo[OpIdx].OperandType == MCOI::OPERAND_IMMEDIATE)
            continue; // special operand like VINTERP attr_chan

          // An instruction may use only one literal.
          // This has been validated on the previous step.
          // See validateVOP3Literal.
          // This literal may be used as more than one operand.
          // If all these operands are of the same size,
          // this literal counts as one scalar value.
          // Otherwise it counts as 2 scalar values.
          // See "GFX10 Shader Programming", section 3.6.2.3.

          unsigned Size = PPU::getOperandSize(Desc, OpIdx);
          if (Size < 4) Size = 4;

          if (NumLiterals == 0) {
            NumLiterals = 1;
            LiteralSize = Size;
          } else if (LiteralSize != Size) {
            NumLiterals = 2;
          }
        }
      }
    }
  }
  ConstantBusUseCount += NumLiterals;

  if (isGFX10())
    return ConstantBusUseCount <= 2;

  return ConstantBusUseCount <= 1;
}

bool PPUAsmParser::validateEarlyClobberLimitations(const MCInst &Inst) {
  const unsigned Opcode = Inst.getOpcode();
  const MCInstrDesc &Desc = MII.get(Opcode);

  const int DstIdx = PPU::getNamedOperandIdx(Opcode, PPU::OpName::vdst);
  if (DstIdx == -1 ||
      Desc.getOperandConstraint(DstIdx, MCOI::EARLY_CLOBBER) == -1) {
    return true;
  }

  const MCRegisterInfo *TRI = getContext().getRegisterInfo();

  const int Src0Idx = PPU::getNamedOperandIdx(Opcode, PPU::OpName::src0);
  const int Src1Idx = PPU::getNamedOperandIdx(Opcode, PPU::OpName::src1);
  const int Src2Idx = PPU::getNamedOperandIdx(Opcode, PPU::OpName::src2);

  assert(DstIdx != -1);
  const MCOperand &Dst = Inst.getOperand(DstIdx);
  assert(Dst.isReg());
  const unsigned DstReg = mc2PseudoReg(Dst.getReg());

  const int SrcIndices[] = { Src0Idx, Src1Idx, Src2Idx };

  for (int SrcIdx : SrcIndices) {
    if (SrcIdx == -1) break;
    const MCOperand &Src = Inst.getOperand(SrcIdx);
    if (Src.isReg()) {
      const unsigned SrcReg = mc2PseudoReg(Src.getReg());
      if (isRegIntersect(DstReg, SrcReg, TRI)) {
        return false;
      }
    }
  }

  return true;
}

bool PPUAsmParser::validateIntClampSupported(const MCInst &Inst) {

  const unsigned Opc = Inst.getOpcode();
  const MCInstrDesc &Desc = MII.get(Opc);

  if ((Desc.TSFlags & PPUInstrFlags::IntClamp) != 0 && !hasIntClamp()) {
    int ClampIdx = PPU::getNamedOperandIdx(Opc, PPU::OpName::clamp);
    assert(ClampIdx != -1);
    return Inst.getOperand(ClampIdx).getImm() == 0;
  }

  return true;
}
/*
bool PPUAsmParser::validateMIMGDataSize(const MCInst &Inst) {

  const unsigned Opc = Inst.getOpcode();
  const MCInstrDesc &Desc = MII.get(Opc);

  if ((Desc.TSFlags & PPUInstrFlags::MIMG) == 0)
    return true;

  int VDataIdx = PPU::getNamedOperandIdx(Opc, PPU::OpName::vdata);
  int DMaskIdx = PPU::getNamedOperandIdx(Opc, PPU::OpName::dmask);
  int TFEIdx   = PPU::getNamedOperandIdx(Opc, PPU::OpName::tfe);

  assert(VDataIdx != -1);
  assert(DMaskIdx != -1);
  assert(TFEIdx != -1);

  unsigned VDataSize = PPU::getRegOperandSize(getMRI(), Desc, VDataIdx);
  unsigned TFESize = Inst.getOperand(TFEIdx).getImm()? 1 : 0;
  unsigned DMask = Inst.getOperand(DMaskIdx).getImm() & 0xf;
  if (DMask == 0)
    DMask = 1;

  unsigned DataSize =
    (Desc.TSFlags & PPUInstrFlags::Gather4) ? 4 : countPopulation(DMask);
  if (hasPackedD16()) {
    int D16Idx = PPU::getNamedOperandIdx(Opc, PPU::OpName::d16);
    if (D16Idx >= 0 && Inst.getOperand(D16Idx).getImm())
      DataSize = (DataSize + 1) / 2;
  }

  return (VDataSize / 4) == DataSize + TFESize;
}

bool PPUAsmParser::validateMIMGAddrSize(const MCInst &Inst) {
  const unsigned Opc = Inst.getOpcode();
  const MCInstrDesc &Desc = MII.get(Opc);

  if ((Desc.TSFlags & PPUInstrFlags::MIMG) == 0 || !isGFX10())
    return true;

  const PPU::MIMGInfo *Info = PPU::getMIMGInfo(Opc);
  const PPU::MIMGBaseOpcodeInfo *BaseOpcode =
      PPU::getMIMGBaseOpcodeInfo(Info->BaseOpcode);
  int VAddr0Idx = PPU::getNamedOperandIdx(Opc, PPU::OpName::vaddr0);
  int SrsrcIdx = PPU::getNamedOperandIdx(Opc, PPU::OpName::srsrc);
  int DimIdx = PPU::getNamedOperandIdx(Opc, PPU::OpName::dim);

  assert(VAddr0Idx != -1);
  assert(SrsrcIdx != -1);
  assert(DimIdx != -1);
  assert(SrsrcIdx > VAddr0Idx);

  unsigned Dim = Inst.getOperand(DimIdx).getImm();
  const PPU::MIMGDimInfo *DimInfo = PPU::getMIMGDimInfoByEncoding(Dim);
  bool IsNSA = SrsrcIdx - VAddr0Idx > 1;
  unsigned VAddrSize =
      IsNSA ? SrsrcIdx - VAddr0Idx
            : PPU::getRegOperandSize(getMRI(), Desc, VAddr0Idx) / 4;

  unsigned AddrSize = BaseOpcode->NumExtraArgs +
                      (BaseOpcode->Gradients ? DimInfo->NumGradients : 0) +
                      (BaseOpcode->Coordinates ? DimInfo->NumCoords : 0) +
                      (BaseOpcode->LodOrClampOrMip ? 1 : 0);
  if (!IsNSA) {
    if (AddrSize > 8)
      AddrSize = 16;
    else if (AddrSize > 4)
      AddrSize = 8;
  }

  return VAddrSize == AddrSize;
}

bool PPUAsmParser::validateMIMGAtomicDMask(const MCInst &Inst) {

  const unsigned Opc = Inst.getOpcode();
  const MCInstrDesc &Desc = MII.get(Opc);

  if ((Desc.TSFlags & PPUInstrFlags::MIMG) == 0)
    return true;
  if (!Desc.mayLoad() || !Desc.mayStore())
    return true; // Not atomic

  int DMaskIdx = PPU::getNamedOperandIdx(Opc, PPU::OpName::dmask);
  unsigned DMask = Inst.getOperand(DMaskIdx).getImm() & 0xf;

  // This is an incomplete check because image_atomic_cmpswap
  // may only use 0x3 and 0xf while other atomic operations
  // may use 0x1 and 0x3. However these limitations are
  // verified when we check that dmask matches dst size.
  return DMask == 0x1 || DMask == 0x3 || DMask == 0xf;
}

bool PPUAsmParser::validateMIMGGatherDMask(const MCInst &Inst) {

  const unsigned Opc = Inst.getOpcode();
  const MCInstrDesc &Desc = MII.get(Opc);

  if ((Desc.TSFlags & PPUInstrFlags::Gather4) == 0)
    return true;

  int DMaskIdx = PPU::getNamedOperandIdx(Opc, PPU::OpName::dmask);
  unsigned DMask = Inst.getOperand(DMaskIdx).getImm() & 0xf;

  // GATHER4 instructions use dmask in a different fashion compared to
  // other MIMG instructions. The only useful DMASK values are
  // 1=red, 2=green, 4=blue, 8=alpha. (e.g. 1 returns
  // (red,red,red,red) etc.) The ISA document doesn't mention
  // this.
  return DMask == 0x1 || DMask == 0x2 || DMask == 0x4 || DMask == 0x8;
}

bool PPUAsmParser::validateMIMGD16(const MCInst &Inst) {

  const unsigned Opc = Inst.getOpcode();
  const MCInstrDesc &Desc = MII.get(Opc);

  if ((Desc.TSFlags & PPUInstrFlags::MIMG) == 0)
    return true;

  int D16Idx = PPU::getNamedOperandIdx(Opc, PPU::OpName::d16);
  if (D16Idx >= 0 && Inst.getOperand(D16Idx).getImm()) {
    if (isCI() || isSI())
      return false;
  }

  return true;
}

bool PPUAsmParser::validateMIMGDim(const MCInst &Inst) {
  const unsigned Opc = Inst.getOpcode();
  const MCInstrDesc &Desc = MII.get(Opc);

  if ((Desc.TSFlags & PPUInstrFlags::MIMG) == 0)
    return true;

  int DimIdx = PPU::getNamedOperandIdx(Opc, PPU::OpName::dim);
  if (DimIdx < 0)
    return true;

  long Imm = Inst.getOperand(DimIdx).getImm();
  if (Imm < 0 || Imm >= 8)
    return false;

  return true;
}
*/

static bool IsRevOpcode(const unsigned Opcode)
{
  switch (Opcode) {
  case PPU::V_SUBREV_F32_e32:
  case PPU::V_SUBREV_F32_e64:
  case PPU::V_SUBREV_F32_e32_ppu:
  case PPU::V_SUBREV_F32_e64_ppu:

  case PPU::V_SUBREV_I32_e32:
  case PPU::V_SUBREV_I32_e64:
  // case PPU::V_SUBREV_I32_e32_ppu:
  // case PPU::V_SUBREV_I32_e64_ppu:

  case PPU::V_SUBBREV_U32_e32:
  case PPU::V_SUBBREV_U32_e64:
  // case PPU::V_SUBBREV_U32_e32_ppu:
  // case PPU::V_SUBBREV_U32_e64_ppu:

  case PPU::V_SUBREV_U32_e32:
  case PPU::V_SUBREV_U32_e64:
  // case PPU::V_SUBREV_U32_e32_ppu:
  // case PPU::V_SUBREV_U32_e64_ppu:

  case PPU::V_SUBREV_F16_e32:
  case PPU::V_SUBREV_F16_e64:
  case PPU::V_SUBREV_F16_e32_ppu:
  case PPU::V_SUBREV_F16_e64_ppu:

  case PPU::V_SUBREV_U16_e32:
  case PPU::V_SUBREV_U16_e64:
  // case PPU::V_SUBREV_U16_e32_ppu:
  // case PPU::V_SUBREV_U16_e64_ppu:

  // case PPU::V_SUBREV_CO_U32_e32_ppu:
  case PPU::V_SUBREV_CO_U32_e64_ppu:

  // case PPU::V_SUBBREV_CO_U32_e32_ppu:
  // case PPU::V_SUBBREV_CO_U32_e64_ppu:

  case PPU::V_SUBREV_NC_U32_e32_ppu:
  case PPU::V_SUBREV_NC_U32_e64_ppu:

  case PPU::V_SUBREV_CO_CI_U32_e32_ppu:
  case PPU::V_SUBREV_CO_CI_U32_e64_ppu:

  case PPU::V_LSHRREV_B32_e32:
  case PPU::V_LSHRREV_B32_e64:
  case PPU::V_LSHRREV_B32_e32_ppu:
  case PPU::V_LSHRREV_B32_e64_ppu:

  case PPU::V_ASHRREV_I32_e32:
  case PPU::V_ASHRREV_I32_e64:
  case PPU::V_ASHRREV_I32_e32_ppu:
  case PPU::V_ASHRREV_I32_e64_ppu:

  case PPU::V_LSHLREV_B32_e32:
  case PPU::V_LSHLREV_B32_e64:
  case PPU::V_LSHLREV_B32_e32_ppu:
  case PPU::V_LSHLREV_B32_e64_ppu:

  case PPU::V_LSHLREV_B16_e32:
  case PPU::V_LSHLREV_B16_e64:
  // case PPU::V_LSHLREV_B16_e32_ppu:
  // case PPU::V_LSHLREV_B16_e64_ppu:

  case PPU::V_LSHRREV_B16_e32:
  case PPU::V_LSHRREV_B16_e64:
  // case PPU::V_LSHRREV_B16_e32_ppu:
  // case PPU::V_LSHRREV_B16_e64_ppu:

  case PPU::V_ASHRREV_I16_e32:
  case PPU::V_ASHRREV_I16_e64:
  // case PPU::V_ASHRREV_I16_e32_ppu:
  // case PPU::V_ASHRREV_I16_e64_ppu:

  case PPU::V_LSHLREV_B64:
  // case PPU::V_LSHLREV_B64_ppu:

  case PPU::V_LSHRREV_B64:
  // case PPU::V_LSHRREV_B64_ppu:

  case PPU::V_ASHRREV_I64:
  // case PPU::V_ASHRREV_I64_ppu:

  case PPU::V_PK_LSHLREV_B16:
  // case PPU::V_PK_LSHLREV_B16_ppu:

  case PPU::V_PK_LSHRREV_B16:
  // case PPU::V_PK_LSHRREV_B16_ppu:
  case PPU::V_PK_ASHRREV_I16:
  // case PPU::V_PK_ASHRREV_I16_ppu:
    return true;
  default:
    return false;
  }
}

bool PPUAsmParser::validateLdsDirect(const MCInst &Inst) {

  using namespace PPUInstrFlags;
  const unsigned Opcode = Inst.getOpcode();
  const MCInstrDesc &Desc = MII.get(Opcode);

  // lds_direct register is defined so that it can be used
  // with 9-bit operands only. Ignore encodings which do not accept these.
  if ((Desc.TSFlags & (VOP1 | VOP2 | VOP3 | VOPC | VOP3P)) == 0)
    return true;

  const int Src0Idx = PPU::getNamedOperandIdx(Opcode, PPU::OpName::src0);
  const int Src1Idx = PPU::getNamedOperandIdx(Opcode, PPU::OpName::src1);
  const int Src2Idx = PPU::getNamedOperandIdx(Opcode, PPU::OpName::src2);

  const int SrcIndices[] = { Src1Idx, Src2Idx };

  // lds_direct cannot be specified as either src1 or src2.
  for (int SrcIdx : SrcIndices) {
    if (SrcIdx == -1) break;
    const MCOperand &Src = Inst.getOperand(SrcIdx);
    if (Src.isReg() && Src.getReg() == LDS_DIRECT) {
      return false;
    }
  }

  if (Src0Idx == -1)
    return true;

  const MCOperand &Src = Inst.getOperand(Src0Idx);
  if (!Src.isReg() || Src.getReg() != LDS_DIRECT)
    return true;

  // lds_direct is specified as src0. Check additional limitations.
  return !IsRevOpcode(Opcode);
}

SMLoc PPUAsmParser::getFlatOffsetLoc(const OperandVector &Operands) const {
  for (unsigned i = 1, e = Operands.size(); i != e; ++i) {
    PPUOperand &Op = ((PPUOperand &)*Operands[i]);
    if (Op.isFlatOffset())
      return Op.getStartLoc();
  }
  return getLoc();
}

bool PPUAsmParser::validateFlatOffset(const MCInst &Inst,
                                         const OperandVector &Operands) {
  uint64_t TSFlags = MII.get(Inst.getOpcode()).TSFlags;
  if ((TSFlags & PPUInstrFlags::FLAT) == 0)
    return true;

  auto Opcode = Inst.getOpcode();
  auto OpNum = PPU::getNamedOperandIdx(Opcode, PPU::OpName::offset);
  assert(OpNum != -1);

  const auto &Op = Inst.getOperand(OpNum);
  if (!hasFlatOffsets() && Op.getImm() != 0) {
    Error(getFlatOffsetLoc(Operands),
          "flat offset modifier is not supported on this GPU");
    return false;
  }

  // Address offset is 12-bit signed for GFX10, 13-bit for GFX9.
  // For FLAT segment the offset must be positive;
  // MSB is ignored and forced to zero.
  unsigned OffsetSize = isGFX9() ? 13 : 12;
  if (TSFlags & PPUInstrFlags::IsNonFlatSeg) {
    if (!isIntN(OffsetSize, Op.getImm())) {
      Error(getFlatOffsetLoc(Operands),
            isGFX9() ? "expected a 13-bit signed offset" :
                       "expected a 12-bit signed offset");
      return false;
    }
  } else {
    if (!isUIntN(OffsetSize - 1, Op.getImm())) {
      Error(getFlatOffsetLoc(Operands),
            isGFX9() ? "expected a 12-bit unsigned offset" :
                       "expected an 11-bit unsigned offset");
      return false;
    }
  }

  return true;
}

bool PPUAsmParser::validateSOPLiteral(const MCInst &Inst) const {
  unsigned Opcode = Inst.getOpcode();
  const MCInstrDesc &Desc = MII.get(Opcode);
  if (!(Desc.TSFlags & (PPUInstrFlags::SOP2 | PPUInstrFlags::SOPC)))
    return true;

  const int Src0Idx = PPU::getNamedOperandIdx(Opcode, PPU::OpName::src0);
  const int Src1Idx = PPU::getNamedOperandIdx(Opcode, PPU::OpName::src1);

  const int OpIndices[] = { Src0Idx, Src1Idx };

  unsigned NumLiterals = 0;
  uint32_t LiteralValue;

  for (int OpIdx : OpIndices) {
    if (OpIdx == -1) break;

    const MCOperand &MO = Inst.getOperand(OpIdx);
    if (MO.isImm() &&
        // Exclude special imm operands (like that used by s_set_gpr_idx_on)
        PPU::isSISrcOperand(Desc, OpIdx) &&
        !isInlineConstant(Inst, OpIdx)) {
      uint32_t Value = static_cast<uint32_t>(MO.getImm());
      if (NumLiterals == 0 || LiteralValue != Value) {
        LiteralValue = Value;
        ++NumLiterals;
      }
    }
  }

  return NumLiterals <= 1;
}

bool PPUAsmParser::validateOpSel(const MCInst &Inst) {
  const unsigned Opc = Inst.getOpcode();
  if (Opc == PPU::V_PERMLANE16_B32_gfx10 ||
      Opc == PPU::V_PERMLANEX16_B32_gfx10) {
    int OpSelIdx = PPU::getNamedOperandIdx(Opc, PPU::OpName::op_sel);
    unsigned OpSel = Inst.getOperand(OpSelIdx).getImm();

    if (OpSel & ~3)
      return false;
  }
  return true;
}

// Check if VCC register matches wavefront size
bool PPUAsmParser::validateVccOperand(unsigned Reg) const {
  // auto FB = getFeatureBits();
  // return (FB[PPU::FeatureWavefrontSize64] && Reg == PPU::VCC) ||
  //  (FB[PPU::FeatureWavefrontSize32] && Reg == PPU::VCC_LO);
  return PPU::VCC;
}

// VOP3 literal is only allowed in GFX10+ and only one can be used
bool PPUAsmParser::validateVOP3Literal(const MCInst &Inst) const {
  unsigned Opcode = Inst.getOpcode();
  const MCInstrDesc &Desc = MII.get(Opcode);
  if (!(Desc.TSFlags & (PPUInstrFlags::VOP3 | PPUInstrFlags::VOP3P)))
    return true;

  const int Src0Idx = PPU::getNamedOperandIdx(Opcode, PPU::OpName::src0);
  const int Src1Idx = PPU::getNamedOperandIdx(Opcode, PPU::OpName::src1);
  const int Src2Idx = PPU::getNamedOperandIdx(Opcode, PPU::OpName::src2);

  const int OpIndices[] = { Src0Idx, Src1Idx, Src2Idx };

  unsigned NumLiterals = 0;
  uint32_t LiteralValue;

  for (int OpIdx : OpIndices) {
    if (OpIdx == -1) break;

    const MCOperand &MO = Inst.getOperand(OpIdx);
    if (!MO.isImm() || !PPU::isSISrcOperand(Desc, OpIdx))
      continue;
/*
    if (OpIdx == Src2Idx && (Desc.TSFlags & PPUInstrFlags::IsMAI) &&
        getFeatureBits()[PPU::FeatureMFMAInlineLiteralBug])
      return false;
*/
    if (!isInlineConstant(Inst, OpIdx)) {
      uint32_t Value = static_cast<uint32_t>(MO.getImm());
      if (NumLiterals == 0 || LiteralValue != Value) {
        LiteralValue = Value;
        ++NumLiterals;
      }
    }
  }

  return !NumLiterals ||
         (NumLiterals == 1 && getFeatureBits()[PPU::FeatureVOP3Literal]);
}

bool PPUAsmParser::validateInstruction(const MCInst &Inst,
                                          const SMLoc &IDLoc,
                                          const OperandVector &Operands) {
  if (!validateLdsDirect(Inst)) {
    Error(IDLoc,
      "invalid use of lds_direct");
    return false;
  }
  if (!validateSOPLiteral(Inst)) {
    Error(IDLoc,
      "only one literal operand is allowed");
    return false;
  }
  if (!validateVOP3Literal(Inst)) {
    Error(IDLoc,
      "invalid literal operand");
    return false;
  }
  if (!validateConstantBusLimitations(Inst)) {
    Error(IDLoc,
      "invalid operand (violates constant bus restrictions)");
    return false;
  }
  if (!validateEarlyClobberLimitations(Inst)) {
    Error(IDLoc,
      "destination must be different than all sources");
    return false;
  }
  if (!validateIntClampSupported(Inst)) {
    Error(IDLoc,
      "integer clamping is not supported on this GPU");
    return false;
  }
  if (!validateOpSel(Inst)) {
    Error(IDLoc,
      "invalid op_sel operand");
    return false;
  }
  // For MUBUF/MTBUF d16 is a part of opcode, so there is nothing to validate.
  /*
  if (!validateMIMGD16(Inst)) {
    Error(IDLoc,
      "d16 modifier is not supported on this GPU");
    return false;
  }
  if (!validateMIMGDim(Inst)) {
    Error(IDLoc, "dim modifier is required on this GPU");
    return false;
  }
  if (!validateMIMGDataSize(Inst)) {
    Error(IDLoc,
      "image data size does not match dmask and tfe");
    return false;
  }
  if (!validateMIMGAddrSize(Inst)) {
    Error(IDLoc,
      "image address size does not match dim and a16");
    return false;
  }
  if (!validateMIMGAtomicDMask(Inst)) {
    Error(IDLoc,
      "invalid atomic image dmask");
    return false;
  }
  if (!validateMIMGGatherDMask(Inst)) {
    Error(IDLoc,
      "invalid image_gather dmask: only one bit must be set");
    return false;
  }
  */
  if (!validateFlatOffset(Inst, Operands)) {
    return false;
  }

  return true;
}

static std::string PPUMnemonicSpellCheck(StringRef S,
                                            const FeatureBitset &FBS,
                                            unsigned VariantID = 0);

bool PPUAsmParser::MatchAndEmitInstruction_ppt(SMLoc IDLoc, unsigned &Opcode,
                                              OperandVector &Operands,
                                              MCStreamer &Out,
                                              uint64_t &ErrorInfo,
                                              bool MatchingInlineAsm) {
  MCInst Inst;
  unsigned Result = Match_Success;
  for (auto Variant : getMatchedVariants()) {
    uint64_t EI;
    auto R = MatchInstructionImpl(Operands, Inst, EI, MatchingInlineAsm,
                                  Variant);
    // We order match statuses from least to most specific. We use most specific
    // status as resulting
    // Match_MnemonicFail < Match_InvalidOperand < Match_MissingFeature < Match_PreferE32
    if ((R == Match_Success) ||
        (R == Match_PreferE32) ||
        (R == Match_MissingFeature && Result != Match_PreferE32) ||
        (R == Match_InvalidOperand && Result != Match_MissingFeature
                                   && Result != Match_PreferE32) ||
        (R == Match_MnemonicFail   && Result != Match_InvalidOperand
                                   && Result != Match_MissingFeature
                                   && Result != Match_PreferE32)) {
      Result = R;
      ErrorInfo = EI;
    }
    if (R == Match_Success)
      break;
  }

  switch (Result) {
  default: break;
  case Match_Success:
    if (!validateInstruction(Inst, IDLoc, Operands)) {
      return true;
    }
    Inst.setLoc(IDLoc);
    Out.EmitInstruction(Inst, getSTI());
    return false;

  case Match_MissingFeature:
    return Error(IDLoc, "instruction not supported on this GPU");

  case Match_MnemonicFail: {
    FeatureBitset FBS = ComputeAvailableFeatures(getSTI().getFeatureBits());
    std::string Suggestion = PPUMnemonicSpellCheck(
        ((PPUOperand &)*Operands[0]).getToken(), FBS);
    return Error(IDLoc, "invalid instruction" + Suggestion,
                 ((PPUOperand &)*Operands[0]).getLocRange());
  }

  case Match_InvalidOperand: {
    SMLoc ErrorLoc = IDLoc;
    if (ErrorInfo != ~0ULL) {
      if (ErrorInfo >= Operands.size()) {
        return Error(IDLoc, "too few operands for instruction");
      }
      ErrorLoc = ((PPUOperand &)*Operands[ErrorInfo]).getStartLoc();
      if (ErrorLoc == SMLoc())
        ErrorLoc = IDLoc;
    }
    return Error(ErrorLoc, "invalid operand for instruction");
  }

  case Match_PreferE32:
    return Error(IDLoc, "internal error: instruction without _e64 suffix "
                        "should be encoded as e32");
  }
  llvm_unreachable("Implement any new match types added!");
}

bool PPUAsmParser::ParseAsAbsoluteExpression(uint32_t &Ret) {
  int64_t Tmp = -1;
  if (getLexer().isNot(AsmToken::Integer) && getLexer().isNot(AsmToken::Identifier)) {
    return true;
  }
  if (getParser().parseAbsoluteExpression(Tmp)) {
    return true;
  }
  Ret = static_cast<uint32_t>(Tmp);
  return false;
}

bool PPUAsmParser::ParseDirectiveMajorMinor(uint32_t &Major,
                                               uint32_t &Minor) {
  if (ParseAsAbsoluteExpression(Major))
    return TokError("invalid major version");

  if (getLexer().isNot(AsmToken::Comma))
    return TokError("minor version number required, comma expected");
  Lex();

  if (ParseAsAbsoluteExpression(Minor))
    return TokError("invalid minor version");

  return false;
}

bool PPUAsmParser::ParseDirectivePPUTarget() {
  if (getSTI().getTargetTriple().getArch() != Triple::amdgcn)
    return TokError("directive only supported for amdgcn architecture");

  std::string Target;

  SMLoc TargetStart = getTok().getLoc();
  if (getParser().parseEscapedString(Target))
    return true;
  SMRange TargetRange = SMRange(TargetStart, getTok().getLoc());

  std::string ExpectedTarget;
  raw_string_ostream ExpectedTargetOS(ExpectedTarget);
  IsaInfo::streamIsaVersion(&getSTI(), ExpectedTargetOS);

  if (Target != ExpectedTargetOS.str())
    return getParser().Error(TargetRange.Start, "target must match options",
                             TargetRange);

  getTargetStreamer().EmitDirectivePPUTarget(Target);
  return false;
}

bool PPUAsmParser::OutOfRangeError(SMRange Range) {
  return getParser().Error(Range.Start, "value out of range", Range);
}

bool PPUAsmParser::calculateGPRBlocks(
    const FeatureBitset &Features, bool VCCUsed, bool FlatScrUsed,
    bool XNACKUsed, Optional<bool> EnableWavefrontSize32, unsigned NextFreeVGPR,
    SMRange VGPRRange, unsigned NextFreeSGPR, SMRange SGPRRange,
    unsigned &VGPRBlocks, unsigned &SGPRBlocks) {
  // TODO(scott.linder): These calculations are duplicated from
  // PPUAsmPrinter::getSIProgramInfo and could be unified.
  IsaVersion Version = {0, 0, 0}; // getIsaVersion(getSTI().getCPU());

  unsigned NumVGPRs = NextFreeVGPR;
  unsigned NumSGPRs = NextFreeSGPR;

  if (Version.Major >= 10)
    NumSGPRs = 0;
  else {
    unsigned MaxAddressableNumSGPRs =
        IsaInfo::getAddressableNumSGPRs(&getSTI());

    //if (Version.Major >= 8 && !Features.test(FeatureSGPRInitBug) &&
    //    NumSGPRs > MaxAddressableNumSGPRs)
    if(NumSGPRs > MaxAddressableNumSGPRs)
      return OutOfRangeError(SGPRRange);

    NumSGPRs +=
        IsaInfo::getNumExtraSGPRs(&getSTI(), VCCUsed, FlatScrUsed, XNACKUsed);

    //if ((Version.Major <= 7 || Features.test(FeatureSGPRInitBug)) &&
    //    NumSGPRs > MaxAddressableNumSGPRs)
    if(NumSGPRs > MaxAddressableNumSGPRs)
      return OutOfRangeError(SGPRRange);

    //if (Features.test(FeatureSGPRInitBug))
    //  NumSGPRs = IsaInfo::FIXED_NUM_SGPRS_FOR_INIT_BUG;
  }

  VGPRBlocks =
      IsaInfo::getNumVPRBlocks(&getSTI(), NumVGPRs, EnableWavefrontSize32);
  SGPRBlocks = IsaInfo::getNumSGPRBlocks(&getSTI(), NumSGPRs);

  return false;
}

bool PPUAsmParser::ParseDirectivePPSKernel() {
  if (getSTI().getTargetTriple().getArch() != Triple::ppu)
    return TokError("directive only supported for amdgcn architecture");

  if (getSTI().getTargetTriple().getOS() != Triple::PPS)
    return TokError("directive only supported for amdhsa OS");

  StringRef KernelName;
  if (getParser().parseIdentifier(KernelName))
    return true;

  kernel_descriptor_t KD = getDefaultPPUKernelDescriptor(&getSTI());

  StringSet<> Seen;

  IsaVersion IVersion = {0, 0, 0}; // getIsaVersion(getSTI().getCPU());

  SMRange VGPRRange;
  uint64_t NextFreeVGPR = 0;
  SMRange SGPRRange;
  uint64_t NextFreeSGPR = 0;
  unsigned UserSGPRCount = 0;
  bool ReserveVCC = true;
  bool ReserveFlatScr = true;
  bool ReserveXNACK = hasXNACK();
  Optional<bool> EnableWavefrontSize32;

  while (true) {
    while (getLexer().is(AsmToken::EndOfStatement))
      Lex();

    if (getLexer().isNot(AsmToken::Identifier))
      return TokError("expected .amdhsa_ directive or .end_amdhsa_kernel");

    StringRef ID = getTok().getIdentifier();
    SMRange IDRange = getTok().getLocRange();
    Lex();

    if (ID == ".end_amdhsa_kernel")
      break;

    if (Seen.find(ID) != Seen.end())
      return TokError(".amdhsa_ directives cannot be repeated");
    Seen.insert(ID);

    SMLoc ValStart = getTok().getLoc();
    int64_t IVal;
    if (getParser().parseAbsoluteExpression(IVal))
      return true;
    SMLoc ValEnd = getTok().getLoc();
    SMRange ValRange = SMRange(ValStart, ValEnd);

    if (IVal < 0)
      return OutOfRangeError(ValRange);

    uint64_t Val = IVal;

#define PARSE_BITS_ENTRY(FIELD, ENTRY, VALUE, RANGE)                           \
  if (!isUInt<ENTRY##_WIDTH>(VALUE))                                           \
    return OutOfRangeError(RANGE);                                             \
  AMDHSA_BITS_SET(FIELD, ENTRY, VALUE);

    if (ID == ".amdhsa_group_segment_fixed_size") {
      if (!isUInt<sizeof(KD.group_segment_fixed_size) * CHAR_BIT>(Val))
        return OutOfRangeError(ValRange);
      KD.group_segment_fixed_size = Val;
    } else if (ID == ".amdhsa_private_segment_fixed_size") {
      if (!isUInt<sizeof(KD.private_segment_fixed_size) * CHAR_BIT>(Val))
        return OutOfRangeError(ValRange);
      KD.private_segment_fixed_size = Val;
    } else if (ID == ".amdhsa_user_sgpr_private_segment_buffer") {
      PARSE_BITS_ENTRY(KD.kernel_code_properties,
                       KERNEL_CODE_PROPERTY_ENABLE_SGPR_PRIVATE_SEGMENT_BUFFER,
                       Val, ValRange);
      if (Val)
        UserSGPRCount += 4;
    } else if (ID == ".amdhsa_user_sgpr_dispatch_ptr") {
      PARSE_BITS_ENTRY(KD.kernel_code_properties,
                       KERNEL_CODE_PROPERTY_ENABLE_SGPR_DISPATCH_PTR, Val,
                       ValRange);
      if (Val)
        UserSGPRCount += 2;
    } else if (ID == ".amdhsa_user_sgpr_queue_ptr") {
      PARSE_BITS_ENTRY(KD.kernel_code_properties,
                       KERNEL_CODE_PROPERTY_ENABLE_SGPR_QUEUE_PTR, Val,
                       ValRange);
      if (Val)
        UserSGPRCount += 2;
    } else if (ID == ".amdhsa_user_sgpr_kernarg_segment_ptr") {
      PARSE_BITS_ENTRY(KD.kernel_code_properties,
                       KERNEL_CODE_PROPERTY_ENABLE_SGPR_KERNARG_SEGMENT_PTR,
                       Val, ValRange);
      if (Val)
        UserSGPRCount += 2;
    } else if (ID == ".amdhsa_user_sgpr_dispatch_id") {
      PARSE_BITS_ENTRY(KD.kernel_code_properties,
                       KERNEL_CODE_PROPERTY_ENABLE_SGPR_DISPATCH_ID, Val,
                       ValRange);
      if (Val)
        UserSGPRCount += 2;
    } else if (ID == ".amdhsa_user_sgpr_flat_scratch_init") {
      PARSE_BITS_ENTRY(KD.kernel_code_properties,
                       KERNEL_CODE_PROPERTY_ENABLE_SGPR_FLAT_SCRATCH_INIT, Val,
                       ValRange);
      if (Val)
        UserSGPRCount += 2;
    } else if (ID == ".amdhsa_user_sgpr_private_segment_size") {
      PARSE_BITS_ENTRY(KD.kernel_code_properties,
                       KERNEL_CODE_PROPERTY_ENABLE_SGPR_PRIVATE_SEGMENT_SIZE,
                       Val, ValRange);
      if (Val)
        UserSGPRCount += 1;
    } else if (ID == ".amdhsa_wavefront_size32") {
      if (IVersion.Major < 10)
        return getParser().Error(IDRange.Start, "directive requires gfx10+",
                                 IDRange);
      EnableWavefrontSize32 = Val;
      PARSE_BITS_ENTRY(KD.kernel_code_properties,
                       KERNEL_CODE_PROPERTY_ENABLE_WAVEFRONT_SIZE32,
                       Val, ValRange);
    } else if (ID == ".amdhsa_system_sgpr_private_segment_wavefront_offset") {
      PARSE_BITS_ENTRY(
          KD.compute_pgm_rsrc2,
          COMPUTE_PGM_RSRC2_ENABLE_SGPR_PRIVATE_SEGMENT_WAVEFRONT_OFFSET, Val,
          ValRange);
    } else if (ID == ".amdhsa_system_sgpr_workgroup_id_x") {
      PARSE_BITS_ENTRY(KD.compute_pgm_rsrc2,
                       COMPUTE_PGM_RSRC2_ENABLE_SGPR_WORKGROUP_ID_X, Val,
                       ValRange);
    } else if (ID == ".amdhsa_system_sgpr_workgroup_id_y") {
      PARSE_BITS_ENTRY(KD.compute_pgm_rsrc2,
                       COMPUTE_PGM_RSRC2_ENABLE_SGPR_WORKGROUP_ID_Y, Val,
                       ValRange);
    } else if (ID == ".amdhsa_system_sgpr_workgroup_id_z") {
      PARSE_BITS_ENTRY(KD.compute_pgm_rsrc2,
                       COMPUTE_PGM_RSRC2_ENABLE_SGPR_WORKGROUP_ID_Z, Val,
                       ValRange);
    } else if (ID == ".amdhsa_system_sgpr_workgroup_info") {
      PARSE_BITS_ENTRY(KD.compute_pgm_rsrc2,
                       COMPUTE_PGM_RSRC2_ENABLE_SGPR_WORKGROUP_INFO, Val,
                       ValRange);
    } else if (ID == ".amdhsa_system_vgpr_workitem_id") {
      PARSE_BITS_ENTRY(KD.compute_pgm_rsrc2,
                       COMPUTE_PGM_RSRC2_ENABLE_VGPR_WORKITEM_ID, Val,
                       ValRange);
    } else if (ID == ".amdhsa_next_free_vgpr") {
      VGPRRange = ValRange;
      NextFreeVGPR = Val;
    } else if (ID == ".amdhsa_next_free_sgpr") {
      SGPRRange = ValRange;
      NextFreeSGPR = Val;
    } else if (ID == ".amdhsa_reserve_vcc") {
      if (!isUInt<1>(Val))
        return OutOfRangeError(ValRange);
      ReserveVCC = Val;
    } else if (ID == ".amdhsa_reserve_flat_scratch") {
      if (IVersion.Major < 7)
        return getParser().Error(IDRange.Start, "directive requires gfx7+",
                                 IDRange);
      if (!isUInt<1>(Val))
        return OutOfRangeError(ValRange);
      ReserveFlatScr = Val;
    } else if (ID == ".amdhsa_reserve_xnack_mask") {
      if (IVersion.Major < 8)
        return getParser().Error(IDRange.Start, "directive requires gfx8+",
                                 IDRange);
      if (!isUInt<1>(Val))
        return OutOfRangeError(ValRange);
      ReserveXNACK = Val;
    } else if (ID == ".amdhsa_float_round_mode_32") {
      PARSE_BITS_ENTRY(KD.compute_pgm_rsrc1,
                       COMPUTE_PGM_RSRC1_FLOAT_ROUND_MODE_32, Val, ValRange);
    } else if (ID == ".amdhsa_float_round_mode_16_64") {
      PARSE_BITS_ENTRY(KD.compute_pgm_rsrc1,
                       COMPUTE_PGM_RSRC1_FLOAT_ROUND_MODE_16_64, Val, ValRange);
    } else if (ID == ".amdhsa_float_denorm_mode_32") {
      PARSE_BITS_ENTRY(KD.compute_pgm_rsrc1,
                       COMPUTE_PGM_RSRC1_FLOAT_DENORM_MODE_32, Val, ValRange);
    } else if (ID == ".amdhsa_float_denorm_mode_16_64") {
      PARSE_BITS_ENTRY(KD.compute_pgm_rsrc1,
                       COMPUTE_PGM_RSRC1_FLOAT_DENORM_MODE_16_64, Val,
                       ValRange);
    } else if (ID == ".amdhsa_dx10_clamp") {
      PARSE_BITS_ENTRY(KD.compute_pgm_rsrc1,
                       COMPUTE_PGM_RSRC1_ENABLE_DX10_CLAMP, Val, ValRange);
    } else if (ID == ".amdhsa_ieee_mode") {
      PARSE_BITS_ENTRY(KD.compute_pgm_rsrc1, COMPUTE_PGM_RSRC1_ENABLE_IEEE_MODE,
                       Val, ValRange);
    } else if (ID == ".amdhsa_fp16_overflow") {
      if (IVersion.Major < 9)
        return getParser().Error(IDRange.Start, "directive requires gfx9+",
                                 IDRange);
      PARSE_BITS_ENTRY(KD.compute_pgm_rsrc1, COMPUTE_PGM_RSRC1_FP16_OVFL, Val,
                       ValRange);
    } else if (ID == ".amdhsa_workgroup_processor_mode") {
      if (IVersion.Major < 10)
        return getParser().Error(IDRange.Start, "directive requires gfx10+",
                                 IDRange);
      PARSE_BITS_ENTRY(KD.compute_pgm_rsrc1, COMPUTE_PGM_RSRC1_WGP_MODE, Val,
                       ValRange);
    } else if (ID == ".amdhsa_memory_ordered") {
      if (IVersion.Major < 10)
        return getParser().Error(IDRange.Start, "directive requires gfx10+",
                                 IDRange);
      PARSE_BITS_ENTRY(KD.compute_pgm_rsrc1, COMPUTE_PGM_RSRC1_MEM_ORDERED, Val,
                       ValRange);
    } else if (ID == ".amdhsa_forward_progress") {
      if (IVersion.Major < 10)
        return getParser().Error(IDRange.Start, "directive requires gfx10+",
                                 IDRange);
      PARSE_BITS_ENTRY(KD.compute_pgm_rsrc1, COMPUTE_PGM_RSRC1_FWD_PROGRESS, Val,
                       ValRange);
    } else if (ID == ".amdhsa_exception_fp_ieee_invalid_op") {
      PARSE_BITS_ENTRY(
          KD.compute_pgm_rsrc2,
          COMPUTE_PGM_RSRC2_ENABLE_EXCEPTION_IEEE_754_FP_INVALID_OPERATION, Val,
          ValRange);
    } else if (ID == ".amdhsa_exception_fp_denorm_src") {
      PARSE_BITS_ENTRY(KD.compute_pgm_rsrc2,
                       COMPUTE_PGM_RSRC2_ENABLE_EXCEPTION_FP_DENORMAL_SOURCE,
                       Val, ValRange);
    } else if (ID == ".amdhsa_exception_fp_ieee_div_zero") {
      PARSE_BITS_ENTRY(
          KD.compute_pgm_rsrc2,
          COMPUTE_PGM_RSRC2_ENABLE_EXCEPTION_IEEE_754_FP_DIVISION_BY_ZERO, Val,
          ValRange);
    } else if (ID == ".amdhsa_exception_fp_ieee_overflow") {
      PARSE_BITS_ENTRY(KD.compute_pgm_rsrc2,
                       COMPUTE_PGM_RSRC2_ENABLE_EXCEPTION_IEEE_754_FP_OVERFLOW,
                       Val, ValRange);
    } else if (ID == ".amdhsa_exception_fp_ieee_underflow") {
      PARSE_BITS_ENTRY(KD.compute_pgm_rsrc2,
                       COMPUTE_PGM_RSRC2_ENABLE_EXCEPTION_IEEE_754_FP_UNDERFLOW,
                       Val, ValRange);
    } else if (ID == ".amdhsa_exception_fp_ieee_inexact") {
      PARSE_BITS_ENTRY(KD.compute_pgm_rsrc2,
                       COMPUTE_PGM_RSRC2_ENABLE_EXCEPTION_IEEE_754_FP_INEXACT,
                       Val, ValRange);
    } else if (ID == ".amdhsa_exception_int_div_zero") {
      PARSE_BITS_ENTRY(KD.compute_pgm_rsrc2,
                       COMPUTE_PGM_RSRC2_ENABLE_EXCEPTION_INT_DIVIDE_BY_ZERO,
                       Val, ValRange);
    } else {
      return getParser().Error(IDRange.Start,
                               "unknown .amdhsa_kernel directive", IDRange);
    }

#undef PARSE_BITS_ENTRY
  }

  if (Seen.find(".amdhsa_next_free_vgpr") == Seen.end())
    return TokError(".amdhsa_next_free_vgpr directive is required");

  if (Seen.find(".amdhsa_next_free_sgpr") == Seen.end())
    return TokError(".amdhsa_next_free_sgpr directive is required");

  unsigned VGPRBlocks;
  unsigned SGPRBlocks;
  if (calculateGPRBlocks(getFeatureBits(), ReserveVCC, ReserveFlatScr,
                         ReserveXNACK, EnableWavefrontSize32, NextFreeVGPR,
                         VGPRRange, NextFreeSGPR, SGPRRange, VGPRBlocks,
                         SGPRBlocks))
    return true;

  if (!isUInt<COMPUTE_PGM_RSRC1_GRANULATED_WORKITEM_VGPR_COUNT_WIDTH>(
          VGPRBlocks))
    return OutOfRangeError(VGPRRange);
  AMDHSA_BITS_SET(KD.compute_pgm_rsrc1,
                  COMPUTE_PGM_RSRC1_GRANULATED_WORKITEM_VGPR_COUNT, VGPRBlocks);

  if (!isUInt<COMPUTE_PGM_RSRC1_GRANULATED_WAVEFRONT_SGPR_COUNT_WIDTH>(
          SGPRBlocks))
    return OutOfRangeError(SGPRRange);
  AMDHSA_BITS_SET(KD.compute_pgm_rsrc1,
                  COMPUTE_PGM_RSRC1_GRANULATED_WAVEFRONT_SGPR_COUNT,
                  SGPRBlocks);

  if (!isUInt<COMPUTE_PGM_RSRC2_USER_SGPR_COUNT_WIDTH>(UserSGPRCount))
    return TokError("too many user SGPRs enabled");
  AMDHSA_BITS_SET(KD.compute_pgm_rsrc2, COMPUTE_PGM_RSRC2_USER_SGPR_COUNT,
                  UserSGPRCount);

  getTargetStreamer().EmitAmdhsaKernelDescriptor(
      getSTI(), KernelName, KD, NextFreeVGPR, NextFreeSGPR, ReserveVCC,
      ReserveFlatScr, ReserveXNACK);
  return false;
}
/*
bool PPUAsmParser::ParseDirectiveHSACodeObjectVersion() {
  uint32_t Major;
  uint32_t Minor;

  if (ParseDirectiveMajorMinor(Major, Minor))
    return true;

  getTargetStreamer().EmitDirectiveHSACodeObjectVersion(Major, Minor);
  return false;
}

bool PPUAsmParser::ParseDirectiveHSACodeObjectISA() {
  uint32_t Major;
  uint32_t Minor;
  uint32_t Stepping;
  StringRef VendorName;
  StringRef ArchName;

  // If this directive has no arguments, then use the ISA version for the
  // targeted GPU.
  if (getLexer().is(AsmToken::EndOfStatement)) {
    PPU::IsaVersion ISA = {0, 0, 0}; // PPU::getIsaVersion(getSTI().getCPU());
    getTargetStreamer().EmitDirectiveHSACodeObjectISA(ISA.Major, ISA.Minor,
                                                      ISA.Stepping,
                                                      "AMD", "PPU");
    return false;
  }

  if (ParseDirectiveMajorMinor(Major, Minor))
    return true;

  if (getLexer().isNot(AsmToken::Comma))
    return TokError("stepping version number required, comma expected");
  Lex();

  if (ParseAsAbsoluteExpression(Stepping))
    return TokError("invalid stepping version");

  if (getLexer().isNot(AsmToken::Comma))
    return TokError("vendor name required, comma expected");
  Lex();

  if (getLexer().isNot(AsmToken::String))
    return TokError("invalid vendor name");

  VendorName = getLexer().getTok().getStringContents();
  Lex();

  if (getLexer().isNot(AsmToken::Comma))
    return TokError("arch name required, comma expected");
  Lex();

  if (getLexer().isNot(AsmToken::String))
    return TokError("invalid arch name");

  ArchName = getLexer().getTok().getStringContents();
  Lex();

  getTargetStreamer().EmitDirectiveHSACodeObjectISA(Major, Minor, Stepping,
                                                    VendorName, ArchName);
  return false;
}
*/

bool PPUAsmParser::ParseAMDKernelCodeTValue(StringRef ID,
                                               amd_kernel_code_t &Header) {
  // max_scratch_backing_memory_byte_size is deprecated. Ignore it while parsing
  // assembly for backwards compatibility.
  if (ID == "max_scratch_backing_memory_byte_size") {
    Parser.eatToEndOfStatement();
    return false;
  }

  SmallString<40> ErrStr;
  raw_svector_ostream Err(ErrStr);
  if (!parseAmdKernelCodeField(ID, getParser(), Header, Err)) {
    return TokError(Err.str());
  }
  Lex();
/*
  if (ID == "enable_wavefront_size32") {
    if (Header.code_properties & AMD_CODE_PROPERTY_ENABLE_WAVEFRONT_SIZE32) {
      if (!isGFX10())
        return TokError("enable_wavefront_size32=1 is only allowed on GFX10+");
      if (!getFeatureBits()[PPU::FeatureWavefrontSize32])
        return TokError("enable_wavefront_size32=1 requires +WavefrontSize32");
    } else {
      if (!getFeatureBits()[PPU::FeatureWavefrontSize64])
        return TokError("enable_wavefront_size32=0 requires +WavefrontSize64");
    }
  }

  if (ID == "wavefront_size") {
    if (Header.wavefront_size == 5) {
      if (!isGFX10())
        return TokError("wavefront_size=5 is only allowed on GFX10+");
      if (!getFeatureBits()[PPU::FeatureWavefrontSize32])
        return TokError("wavefront_size=5 requires +WavefrontSize32");
    } else if (Header.wavefront_size == 6) {
      if (!getFeatureBits()[PPU::FeatureWavefrontSize64])
        return TokError("wavefront_size=6 requires +WavefrontSize64");
    }
  }
  if (ID == "enable_wgp_mode") {
    if (G_00B848_WGP_MODE(Header.compute_pgm_resource_registers) && !isGFX10())
      return TokError("enable_wgp_mode=1 is only allowed on GFX10+");
  }

  if (ID == "enable_mem_ordered") {
    if (G_00B848_MEM_ORDERED(Header.compute_pgm_resource_registers) && !isGFX10())
      return TokError("enable_mem_ordered=1 is only allowed on GFX10+");
  }

  if (ID == "enable_fwd_progress") {
    if (G_00B848_FWD_PROGRESS(Header.compute_pgm_resource_registers) && !isGFX10())
      return TokError("enable_fwd_progress=1 is only allowed on GFX10+");
  }
*/

  return false;
}
/*
bool PPUAsmParser::ParseDirectiveAMDKernelCodeT() {
  amd_kernel_code_t Header;
  PPU::initDefaultPPUKernelCodeT(Header, &getSTI());

  while (true) {
    // Lex EndOfStatement.  This is in a while loop, because lexing a comment
    // will set the current token to EndOfStatement.
    while(getLexer().is(AsmToken::EndOfStatement))
      Lex();

    if (getLexer().isNot(AsmToken::Identifier))
      return TokError("expected value identifier or .end_amd_kernel_code_t");

    StringRef ID = getLexer().getTok().getIdentifier();
    Lex();

    if (ID == ".end_amd_kernel_code_t")
      break;

    if (ParseAMDKernelCodeTValue(ID, Header))
      return true;
  }

  getTargetStreamer().EmitAMDKernelCodeT(Header);

  return false;
}

bool PPUAsmParser::ParseDirectivePPUHsaKernel() {
  if (getLexer().isNot(AsmToken::Identifier))
    return TokError("expected symbol name");

  StringRef KernelName = Parser.getTok().getString();

  getTargetStreamer().EmitPPUSymbolType(KernelName,
                                           ELF::STT_PPU_HSA_KERNEL);
  Lex();
  if (!PPU::IsaInfo::hasCodeObjectV3(&getSTI()))
    KernelScope.initialize(getContext());
  return false;
}

bool PPUAsmParser::ParseDirectiveISAVersion() {
  if (getSTI().getTargetTriple().getArch() != Triple::amdgcn) {
    return Error(getParser().getTok().getLoc(),
                 ".amd_amdgpu_isa directive is not available on non-amdgcn "
                 "architectures");
  }

  auto ISAVersionStringFromASM = getLexer().getTok().getStringContents();

  std::string ISAVersionStringFromSTI;
  raw_string_ostream ISAVersionStreamFromSTI(ISAVersionStringFromSTI);
  IsaInfo::streamIsaVersion(&getSTI(), ISAVersionStreamFromSTI);

  if (ISAVersionStringFromASM != ISAVersionStreamFromSTI.str()) {
    return Error(getParser().getTok().getLoc(),
                 ".amd_amdgpu_isa directive does not match triple and/or mcpu "
                 "arguments specified through the command line");
  }

  getTargetStreamer().EmitISAVersion(ISAVersionStreamFromSTI.str());
  Lex();

  return false;
}
*/

bool PPUAsmParser::ParseDirectiveHSAMetadata() {
  const char *AssemblerDirectiveBegin;
  const char *AssemblerDirectiveEnd;
  std::tie(AssemblerDirectiveBegin, AssemblerDirectiveEnd) =
           std::make_tuple(HSAMD::V3::AssemblerDirectiveBegin,
                            HSAMD::V3::AssemblerDirectiveEnd);
/*
      PPU::IsaInfo::hasCodeObjectV3(&getSTI())
          ? std::make_tuple(HSAMD::V3::AssemblerDirectiveBegin,
                            HSAMD::V3::AssemblerDirectiveEnd)
          : std::make_tuple(HSAMD::AssemblerDirectiveBegin,
                            HSAMD::AssemblerDirectiveEnd);
*/
  if (getSTI().getTargetTriple().getOS() != Triple::PPS) {
    return Error(getParser().getTok().getLoc(),
                 (Twine(AssemblerDirectiveBegin) + Twine(" directive is "
                 "not available on non-amdhsa OSes")).str());
  }

  std::string HSAMetadataString;
  if (ParseToEndDirective(AssemblerDirectiveBegin, AssemblerDirectiveEnd,
                          HSAMetadataString))
    return true;

  //if (IsaInfo::hasCodeObjectV3(&getSTI())) {
    if (!getTargetStreamer().EmitHSAMetadataV3(HSAMetadataString))
      return Error(getParser().getTok().getLoc(), "invalid HSA metadata");
  //} else {
  //  if (!getTargetStreamer().EmitHSAMetadataV2(HSAMetadataString))
  //    return Error(getParser().getTok().getLoc(), "invalid HSA metadata");
  //}

  return false;
}

/// Common code to parse out a block of text (typically YAML) between start and
/// end directives.
bool PPUAsmParser::ParseToEndDirective(const char *AssemblerDirectiveBegin,
                                          const char *AssemblerDirectiveEnd,
                                          std::string &CollectString) {

  raw_string_ostream CollectStream(CollectString);

  getLexer().setSkipSpace(false);

  bool FoundEnd = false;
  while (!getLexer().is(AsmToken::Eof)) {
    while (getLexer().is(AsmToken::Space)) {
      CollectStream << getLexer().getTok().getString();
      Lex();
    }

    if (getLexer().is(AsmToken::Identifier)) {
      StringRef ID = getLexer().getTok().getIdentifier();
      if (ID == AssemblerDirectiveEnd) {
        Lex();
        FoundEnd = true;
        break;
      }
    }

    CollectStream << Parser.parseStringToEndOfStatement()
                  << getContext().getAsmInfo()->getSeparatorString();

    Parser.eatToEndOfStatement();
  }

  getLexer().setSkipSpace(true);

  if (getLexer().is(AsmToken::Eof) && !FoundEnd) {
    return TokError(Twine("expected directive ") +
                    Twine(AssemblerDirectiveEnd) + Twine(" not found"));
  }

  CollectStream.flush();
  return false;
}

/// Parse the assembler directive for new MsgPack-format PAL metadata.
/*
bool PPUAsmParser::ParseDirectivePALMetadataBegin() {
  std::string String;
  if (ParseToEndDirective(PPU::PALMD::AssemblerDirectiveBegin,
                          PPU::PALMD::AssemblerDirectiveEnd, String))
    return true;

  auto PALMetadata = getTargetStreamer().getPALMetadata();
  if (!PALMetadata->setFromString(String))
    return Error(getParser().getTok().getLoc(), "invalid PAL metadata");
  return false;
}

/// Parse the assembler directive for old linear-format PAL metadata.
bool PPUAsmParser::ParseDirectivePALMetadata() {
  if (getSTI().getTargetTriple().getOS() != Triple::AMDPAL) {
    return Error(getParser().getTok().getLoc(),
                 (Twine(PALMD::AssemblerDirective) + Twine(" directive is "
                 "not available on non-amdpal OSes")).str());
  }

  auto PALMetadata = getTargetStreamer().getPALMetadata();
  PALMetadata->setLegacy();
  for (;;) {
    uint32_t Key, Value;
    if (ParseAsAbsoluteExpression(Key)) {
      return TokError(Twine("invalid value in ") +
                      Twine(PALMD::AssemblerDirective));
    }
    if (getLexer().isNot(AsmToken::Comma)) {
      return TokError(Twine("expected an even number of values in ") +
                      Twine(PALMD::AssemblerDirective));
    }
    Lex();
    if (ParseAsAbsoluteExpression(Value)) {
      return TokError(Twine("invalid value in ") +
                      Twine(PALMD::AssemblerDirective));
    }
    PALMetadata->setRegister(Key, Value);
    if (getLexer().isNot(AsmToken::Comma))
      break;
    Lex();
  }
  return false;
}
*/

/// ParseDirectivePPULDS
///  ::= .amdgpu_lds identifier ',' size_expression [',' align_expression]
bool PPUAsmParser::ParseDirectivePPULDS() {
  if (getParser().checkForValidSection())
    return true;

  StringRef Name;
  SMLoc NameLoc = getLexer().getLoc();
  if (getParser().parseIdentifier(Name))
    return TokError("expected identifier in directive");

  MCSymbol *Symbol = getContext().getOrCreateSymbol(Name);
  if (parseToken(AsmToken::Comma, "expected ','"))
    return true;

  unsigned LocalMemorySize = PPU::IsaInfo::getLocalMemorySize(&getSTI());

  int64_t Size;
  SMLoc SizeLoc = getLexer().getLoc();
  if (getParser().parseAbsoluteExpression(Size))
    return true;
  if (Size < 0)
    return Error(SizeLoc, "size must be non-negative");
  if (Size > LocalMemorySize)
    return Error(SizeLoc, "size is too large");

  int64_t Align = 4;
  if (getLexer().is(AsmToken::Comma)) {
    Lex();
    SMLoc AlignLoc = getLexer().getLoc();
    if (getParser().parseAbsoluteExpression(Align))
      return true;
    if (Align < 0 || !isPowerOf2_64(Align))
      return Error(AlignLoc, "alignment must be a power of two");

    // Alignment larger than the size of LDS is possible in theory, as long
    // as the linker manages to place to symbol at address 0, but we do want
    // to make sure the alignment fits nicely into a 32-bit integer.
    if (Align >= 1u << 31)
      return Error(AlignLoc, "alignment is too large");
  }

  if (parseToken(AsmToken::EndOfStatement,
                 "unexpected token in '.amdgpu_lds' directive"))
    return true;

  Symbol->redefineIfPossible();
  if (!Symbol->isUndefined())
    return Error(NameLoc, "invalid symbol redefinition");

  getTargetStreamer().emitPPULDS(Symbol, Size, Align);
  return false;
}

bool PPUAsmParser::ParseDirective(AsmToken DirectiveID) {
  StringRef IDVal = DirectiveID.getString();

  if (PPU::IsaInfo::hasCodeObjectV3(&getSTI())) {
    if (IDVal == ".amdgcn_target")
      return ParseDirectivePPUTarget();

    if (IDVal == ".amdhsa_kernel")
      return ParseDirectivePPSKernel();

    // TODO: Restructure/combine with PAL metadata directive.
    if (IDVal == PPU::HSAMD::V3::AssemblerDirectiveBegin)
      return ParseDirectiveHSAMetadata();
      /*
  } else {
    if (IDVal == ".hsa_code_object_version")
      return ParseDirectiveHSACodeObjectVersion();

    if (IDVal == ".hsa_code_object_isa")
      return ParseDirectiveHSACodeObjectISA();

    if (IDVal == ".amd_kernel_code_t")
      return ParseDirectiveAMDKernelCodeT();

    if (IDVal == ".amdgpu_hsa_kernel")
      return ParseDirectivePPUHsaKernel();

    if (IDVal == ".amd_amdgpu_isa")
      return ParseDirectiveISAVersion();

    if (IDVal == PPU::HSAMD::AssemblerDirectiveBegin)
      return ParseDirectiveHSAMetadata();
      */
  }

  if (IDVal == ".amdgpu_lds")
    return ParseDirectivePPULDS();
/*
  if (IDVal == PALMD::AssemblerDirectiveBegin)
    return ParseDirectivePALMetadataBegin();

  if (IDVal == PALMD::AssemblerDirective)
    return ParseDirectivePALMetadata();
*/

  // riscv option
  // This returns false if this function recognizes the directive
  // regardless of whether it is successfully handles or reports an
  // error. Otherwise it returns true to give the generic parser a
  // chance at recognizing it.
  if (IDVal == ".option")
    return parseDirectiveOption();

  return true;
}
/*
bool PPUAsmParser::subtargetHasRegister(const MCRegisterInfo &MRI,
                                           unsigned RegNo) const {

  for (MCRegAliasIterator R(PPU::TTMP12_TTMP13_TTMP14_TTMP15, &MRI, true);
       R.isValid(); ++R) {
    if (*R == RegNo)
      return isGFX9() || isGFX10();
  }

  // GFX10 has 2 more SGPRs 104 and 105.
  for (MCRegAliasIterator R(PPU::SGPR104_SGPR105, &MRI, true);
       R.isValid(); ++R) {
    if (*R == RegNo)
      return hasSGPR104_SGPR105();
  }

  switch (RegNo) {
  case PPU::SRC_SHARED_BASE:
  case PPU::SRC_SHARED_LIMIT:
  case PPU::SRC_PRIVATE_BASE:
  case PPU::SRC_PRIVATE_LIMIT:
  case PPU::SRC_POPS_EXITING_WAVE_ID:
    return !isCI() && !isSI() && !isVI();
  case PPU::TBA:
  case PPU::TBA_LO:
  case PPU::TBA_HI:
  case PPU::TMA:
  case PPU::TMA_LO:
  case PPU::TMA_HI:
    return !isGFX9() && !isGFX10();
  case PPU::XNACK_MASK:
  case PPU::XNACK_MASK_LO:
  case PPU::XNACK_MASK_HI:
    return !isCI() && !isSI() && !isGFX10() && hasXNACK();
  case PPU::SGPR_NULL:
    return isGFX10();
  default:
    break;
  }

  if (isCI())
    return true;

  if (isSI() || isGFX10()) {
    // No flat_scr on SI.
    // On GFX10 flat scratch is not a valid register operand and can only be
    // accessed with s_setreg/s_getreg.
    switch (RegNo) {
    case PPU::FLAT_SCR:
    case PPU::FLAT_SCR_LO:
    case PPU::FLAT_SCR_HI:
      return false;
    default:
      return true;
    }
  }

  // VI only has 102 SGPRs, so make sure we aren't trying to use the 2 more that
  // SI/CI have.
  for (MCRegAliasIterator R(PPU::SGPR102_SGPR103, &MRI, true);
       R.isValid(); ++R) {
    if (*R == RegNo)
      return hasSGPR102_SGPR103();
  }

  return true;
}
*/

OperandMatchResultTy
PPUAsmParser::parseOperand_ppt(OperandVector &Operands, StringRef Mnemonic,
                              OperandMode Mode) {
  // Try to parse with a custom parser
  OperandMatchResultTy ResTy = MatchOperandParserImpl(Operands, Mnemonic);

  // If we successfully parsed the operand or if there as an error parsing,
  // we are done.
  //
  // If we are parsing after we reach EndOfStatement then this means we
  // are appending default values to the Operands list.  This is only done
  // by custom parser, so we shouldn't continue on to the generic parsing.
  if (ResTy == MatchOperand_Success || ResTy == MatchOperand_ParseFail ||
      getLexer().is(AsmToken::EndOfStatement))
    return ResTy;

  if (Mode == OperandMode_NSA && getLexer().is(AsmToken::LBrac)) {
    unsigned Prefix = Operands.size();
    SMLoc LBraceLoc = getTok().getLoc();
    Parser.Lex(); // eat the '['

    for (;;) {
      ResTy = parseReg(Operands);
      if (ResTy != MatchOperand_Success)
        return ResTy;

      if (getLexer().is(AsmToken::RBrac))
        break;

      if (getLexer().isNot(AsmToken::Comma))
        return MatchOperand_ParseFail;
      Parser.Lex();
    }

    if (Operands.size() - Prefix > 1) {
      Operands.insert(Operands.begin() + Prefix,
                      PPUOperand::CreateToken(this, "[", LBraceLoc));
      Operands.push_back(PPUOperand::CreateToken(this, "]",
                                                    getTok().getLoc()));
    }

    Parser.Lex(); // eat the ']'
    return MatchOperand_Success;
  }

  return parseRegOrImm(Operands);
}

StringRef PPUAsmParser::parseMnemonicSuffix(StringRef Name) {
  // Clear any forced encodings from the previous instruction.
  setForcedEncodingSize(0);
  setForcedDPP(false);
  setForcedSDWA(false);

  if (Name.endswith("_e64")) {
    setForcedEncodingSize(64);
    return Name.substr(0, Name.size() - 4);
  } else if (Name.endswith("_e32")) {
    setForcedEncodingSize(32);
    return Name.substr(0, Name.size() - 4);
  }
  return Name;
}

bool PPUAsmParser::ParseInstruction_ppt(ParseInstructionInfo &Info,
                                       StringRef Name,
                                       SMLoc NameLoc, OperandVector &Operands) {
  // Add the instruction mnemonic
  Name = parseMnemonicSuffix(Name);
  Operands.push_back(PPUOperand::CreateToken(this, Name, NameLoc));

  bool IsMIMG = Name.startswith("image_");

  while (!getLexer().is(AsmToken::EndOfStatement)) {
    OperandMode Mode = OperandMode_Default;
    if (IsMIMG && isGFX10() && Operands.size() == 2)
      Mode = OperandMode_NSA;
    OperandMatchResultTy Res = parseOperand_ppt(Operands, Name, Mode);

    // Eat the comma or space if there is one.
    if (getLexer().is(AsmToken::Comma))
      Parser.Lex();

    switch (Res) {
      case MatchOperand_Success: break;
      case MatchOperand_ParseFail:
        // FIXME: use real operand location rather than the current location.
        Error(getLexer().getLoc(), "failed parsing operand.");
        while (!getLexer().is(AsmToken::EndOfStatement)) {
          Parser.Lex();
        }
        return true;
      case MatchOperand_NoMatch:
        // FIXME: use real operand location rather than the current location.
        Error(getLexer().getLoc(), "not a valid operand.");
        while (!getLexer().is(AsmToken::EndOfStatement)) {
          Parser.Lex();
        }
        return true;
    }
  }

  return false;
}

//===----------------------------------------------------------------------===//
// Utility functions
//===----------------------------------------------------------------------===//

OperandMatchResultTy
PPUAsmParser::parseIntWithPrefix(const char *Prefix, int64_t &IntVal) {

  if (!trySkipId(Prefix, AsmToken::Colon))
    return MatchOperand_NoMatch;

  return parseExpr(IntVal) ? MatchOperand_Success : MatchOperand_ParseFail;
}

OperandMatchResultTy
PPUAsmParser::parseIntWithPrefix(const char *Prefix, OperandVector &Operands,
                                    PPUOperand::ImmTy ImmTy,
                                    bool (*ConvertResult)(int64_t&)) {
  SMLoc S = getLoc();
  int64_t Value = 0;

  OperandMatchResultTy Res = parseIntWithPrefix(Prefix, Value);
  if (Res != MatchOperand_Success)
    return Res;

  if (ConvertResult && !ConvertResult(Value)) {
    Error(S, "invalid " + StringRef(Prefix) + " value.");
  }

  Operands.push_back(PPUOperand::CreateImm(this, Value, S, ImmTy));
  return MatchOperand_Success;
}

OperandMatchResultTy
PPUAsmParser::parseOperandArrayWithPrefix(const char *Prefix,
                                             OperandVector &Operands,
                                             PPUOperand::ImmTy ImmTy,
                                             bool (*ConvertResult)(int64_t&)) {
  SMLoc S = getLoc();
  if (!trySkipId(Prefix, AsmToken::Colon))
    return MatchOperand_NoMatch;

  if (!skipToken(AsmToken::LBrac, "expected a left square bracket"))
    return MatchOperand_ParseFail;

  unsigned Val = 0;
  const unsigned MaxSize = 4;

  // FIXME: How to verify the number of elements matches the number of src
  // operands?
  for (int I = 0; ; ++I) {
    int64_t Op;
    SMLoc Loc = getLoc();
    if (!parseExpr(Op))
      return MatchOperand_ParseFail;

    if (Op != 0 && Op != 1) {
      Error(Loc, "invalid " + StringRef(Prefix) + " value.");
      return MatchOperand_ParseFail;
    }

    Val |= (Op << I);

    if (trySkipToken(AsmToken::RBrac))
      break;

    if (I + 1 == MaxSize) {
      Error(getLoc(), "expected a closing square bracket");
      return MatchOperand_ParseFail;
    }

    if (!skipToken(AsmToken::Comma, "expected a comma"))
      return MatchOperand_ParseFail;
  }

  Operands.push_back(PPUOperand::CreateImm(this, Val, S, ImmTy));
  return MatchOperand_Success;
}

OperandMatchResultTy
PPUAsmParser::parseNamedBit(const char *Name, OperandVector &Operands,
                               PPUOperand::ImmTy ImmTy) {
  int64_t Bit = 0;
  SMLoc S = Parser.getTok().getLoc();

  // We are at the end of the statement, and this is a default argument, so
  // use a default value.
  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    switch(getLexer().getKind()) {
      case AsmToken::Identifier: {
        StringRef Tok = Parser.getTok().getString();
        if (Tok == Name) {
          if (Tok == "r128" && isGFX9())
            Error(S, "r128 modifier is not supported on this GPU");
          if (Tok == "a16" && !isGFX9() && !isGFX10())
            Error(S, "a16 modifier is not supported on this GPU");
          Bit = 1;
          Parser.Lex();
        } else if (Tok.startswith("no") && Tok.endswith(Name)) {
          Bit = 0;
          Parser.Lex();
        } else {
          return MatchOperand_NoMatch;
        }
        break;
      }
      default:
        return MatchOperand_NoMatch;
    }
  }

  if (!isGFX10() && ImmTy == PPUOperand::ImmTyDLC)
    return MatchOperand_ParseFail;

  Operands.push_back(PPUOperand::CreateImm(this, Bit, S, ImmTy));
  return MatchOperand_Success;
}

static void addOptionalImmOperand(
  MCInst& Inst, const OperandVector& Operands,
  PPUAsmParser::OptionalImmIndexMap& OptionalIdx,
  PPUOperand::ImmTy ImmT,
  int64_t Default = 0) {
  auto i = OptionalIdx.find(ImmT);
  if (i != OptionalIdx.end()) {
    unsigned Idx = i->second;
    ((PPUOperand &)*Operands[Idx]).addImmOperands(Inst, 1);
  } else {
    Inst.addOperand(MCOperand::createImm(Default));
  }
}

OperandMatchResultTy
PPUAsmParser::parseStringWithPrefix(StringRef Prefix, StringRef &Value) {
  if (getLexer().isNot(AsmToken::Identifier)) {
    return MatchOperand_NoMatch;
  }
  StringRef Tok = Parser.getTok().getString();
  if (Tok != Prefix) {
    return MatchOperand_NoMatch;
  }

  Parser.Lex();
  if (getLexer().isNot(AsmToken::Colon)) {
    return MatchOperand_ParseFail;
  }

  Parser.Lex();
  if (getLexer().isNot(AsmToken::Identifier)) {
    return MatchOperand_ParseFail;
  }

  Value = Parser.getTok().getString();
  return MatchOperand_Success;
}

// dfmt and nfmt (in a tbuffer instruction) are parsed as one to allow their
// values to live in a joint format operand in the MCInst encoding.
OperandMatchResultTy
PPUAsmParser::parseDfmtNfmt(OperandVector &Operands) {
  SMLoc S = Parser.getTok().getLoc();
  int64_t Dfmt = 0, Nfmt = 0;
  // dfmt and nfmt can appear in either order, and each is optional.
  bool GotDfmt = false, GotNfmt = false;
  while (!GotDfmt || !GotNfmt) {
    if (!GotDfmt) {
      auto Res = parseIntWithPrefix("dfmt", Dfmt);
      if (Res != MatchOperand_NoMatch) {
        if (Res != MatchOperand_Success)
          return Res;
        if (Dfmt >= 16) {
          Error(Parser.getTok().getLoc(), "out of range dfmt");
          return MatchOperand_ParseFail;
        }
        GotDfmt = true;
        Parser.Lex();
        continue;
      }
    }
    if (!GotNfmt) {
      auto Res = parseIntWithPrefix("nfmt", Nfmt);
      if (Res != MatchOperand_NoMatch) {
        if (Res != MatchOperand_Success)
          return Res;
        if (Nfmt >= 8) {
          Error(Parser.getTok().getLoc(), "out of range nfmt");
          return MatchOperand_ParseFail;
        }
        GotNfmt = true;
        Parser.Lex();
        continue;
      }
    }
    break;
  }
  if (!GotDfmt && !GotNfmt)
    return MatchOperand_NoMatch;
  auto Format = Dfmt | Nfmt << 4;
  Operands.push_back(
      PPUOperand::CreateImm(this, Format, S, PPUOperand::ImmTyFORMAT));
  return MatchOperand_Success;
}

//===----------------------------------------------------------------------===//
// ds
//===----------------------------------------------------------------------===//

void PPUAsmParser::cvtDSOffset01(MCInst &Inst,
                                    const OperandVector &Operands) {
  OptionalImmIndexMap OptionalIdx;

  for (unsigned i = 1, e = Operands.size(); i != e; ++i) {
    PPUOperand &Op = ((PPUOperand &)*Operands[i]);

    // Add the register arguments
    if (Op.isReg()) {
      Op.addRegOperands(Inst, 1);
      continue;
    }

    // Handle optional arguments
    OptionalIdx[Op.getImmTy()] = i;
  }

  addOptionalImmOperand(Inst, Operands, OptionalIdx, PPUOperand::ImmTyOffset0);
  addOptionalImmOperand(Inst, Operands, OptionalIdx, PPUOperand::ImmTyOffset1);
  addOptionalImmOperand(Inst, Operands, OptionalIdx, PPUOperand::ImmTyGDS);

  Inst.addOperand(MCOperand::createReg(PPU::M0)); // m0
}

void PPUAsmParser::cvtDSImpl(MCInst &Inst, const OperandVector &Operands,
                                bool IsGdsHardcoded) {
  OptionalImmIndexMap OptionalIdx;

  for (unsigned i = 1, e = Operands.size(); i != e; ++i) {
    PPUOperand &Op = ((PPUOperand &)*Operands[i]);

    // Add the register arguments
    if (Op.isReg()) {
      Op.addRegOperands(Inst, 1);
      continue;
    }

    if (Op.isToken() && Op.getToken() == "gds") {
      IsGdsHardcoded = true;
      continue;
    }

    // Handle optional arguments
    OptionalIdx[Op.getImmTy()] = i;
  }

  PPUOperand::ImmTy OffsetType =
    (Inst.getOpcode() == PPU::DS_SWIZZLE_B32) ? PPUOperand::ImmTySwizzle :
                                                      PPUOperand::ImmTyOffset;

  addOptionalImmOperand(Inst, Operands, OptionalIdx, OffsetType);

  if (!IsGdsHardcoded) {
    addOptionalImmOperand(Inst, Operands, OptionalIdx, PPUOperand::ImmTyGDS);
  }
  Inst.addOperand(MCOperand::createReg(PPU::M0)); // m0
}

void PPUAsmParser::cvtExp(MCInst &Inst, const OperandVector &Operands) {
  OptionalImmIndexMap OptionalIdx;

  unsigned OperandIdx[4];
  unsigned EnMask = 0;
  int SrcIdx = 0;

  for (unsigned i = 1, e = Operands.size(); i != e; ++i) {
    PPUOperand &Op = ((PPUOperand &)*Operands[i]);

    // Add the register arguments
    if (Op.isReg()) {
      assert(SrcIdx < 4);
      OperandIdx[SrcIdx] = Inst.size();
      Op.addRegOperands(Inst, 1);
      ++SrcIdx;
      continue;
    }

    if (Op.isOff()) {
      assert(SrcIdx < 4);
      OperandIdx[SrcIdx] = Inst.size();
      Inst.addOperand(MCOperand::createReg(PPU::NoRegister));
      ++SrcIdx;
      continue;
    }

    if (Op.isImm() && Op.getImmTy() == PPUOperand::ImmTyExpTgt) {
      Op.addImmOperands(Inst, 1);
      continue;
    }

    if (Op.isToken() && Op.getToken() == "done")
      continue;

    // Handle optional arguments
    OptionalIdx[Op.getImmTy()] = i;
  }

  assert(SrcIdx == 4);

  bool Compr = false;
  if (OptionalIdx.find(PPUOperand::ImmTyExpCompr) != OptionalIdx.end()) {
    Compr = true;
    Inst.getOperand(OperandIdx[1]) = Inst.getOperand(OperandIdx[2]);
    Inst.getOperand(OperandIdx[2]).setReg(PPU::NoRegister);
    Inst.getOperand(OperandIdx[3]).setReg(PPU::NoRegister);
  }

  for (auto i = 0; i < SrcIdx; ++i) {
    if (Inst.getOperand(OperandIdx[i]).getReg() != PPU::NoRegister) {
      EnMask |= Compr? (0x3 << i * 2) : (0x1 << i);
    }
  }

  addOptionalImmOperand(Inst, Operands, OptionalIdx, PPUOperand::ImmTyExpVM);
  addOptionalImmOperand(Inst, Operands, OptionalIdx, PPUOperand::ImmTyExpCompr);

  Inst.addOperand(MCOperand::createImm(EnMask));
}

//===----------------------------------------------------------------------===//
// s_waitcnt
//===----------------------------------------------------------------------===//

static bool
encodeCnt(
  const PPU::IsaVersion ISA,
  int64_t &IntVal,
  int64_t CntVal,
  bool Saturate,
  unsigned (*encode)(const IsaVersion &Version, unsigned, unsigned),
  unsigned (*decode)(const IsaVersion &Version, unsigned))
{
  bool Failed = false;

  IntVal = encode(ISA, IntVal, CntVal);
  if (CntVal != decode(ISA, IntVal)) {
    if (Saturate) {
      IntVal = encode(ISA, IntVal, -1);
    } else {
      Failed = true;
    }
  }
  return Failed;
}

bool PPUAsmParser::parseCnt(int64_t &IntVal) {

  SMLoc CntLoc = getLoc();
  StringRef CntName = getTokenStr();

  if (!skipToken(AsmToken::Identifier, "expected a counter name") ||
      !skipToken(AsmToken::LParen, "expected a left parenthesis"))
    return false;

  int64_t CntVal;
  SMLoc ValLoc = getLoc();
  if (!parseExpr(CntVal))
    return false;

  PPU::IsaVersion ISA = {0, 0, 0}; // PPU::getIsaVersion(getSTI().getCPU());

  bool Failed = true;
  bool Sat = CntName.endswith("_sat");

  if (CntName == "vmcnt" || CntName == "vmcnt_sat") {
    Failed = encodeCnt(ISA, IntVal, CntVal, Sat, encodeVmcnt, decodeVmcnt);
    /*
  } else if (CntName == "expcnt" || CntName == "expcnt_sat") {
    Failed = encodeCnt(ISA, IntVal, CntVal, Sat, encodeExpcnt, decodeExpcnt);
    */
  } else if (CntName == "lgkmcnt" || CntName == "lgkmcnt_sat") {
    Failed = encodeCnt(ISA, IntVal, CntVal, Sat, encodeLgkmcnt, decodeLgkmcnt);
  } else {
    Error(CntLoc, "invalid counter name " + CntName);
    return false;
  }

  if (Failed) {
    Error(ValLoc, "too large value for " + CntName);
    return false;
  }

  if (!skipToken(AsmToken::RParen, "expected a closing parenthesis"))
    return false;

  if (trySkipToken(AsmToken::Amp) || trySkipToken(AsmToken::Comma)) {
    if (isToken(AsmToken::EndOfStatement)) {
      Error(getLoc(), "expected a counter name");
      return false;
    }
  }

  return true;
}

OperandMatchResultTy
PPUAsmParser::parseSWaitCntOps(OperandVector &Operands) {
  PPU::IsaVersion ISA = {0, 0, 0}; // PPU::getIsaVersion(getSTI().getCPU());
  int64_t Waitcnt = getWaitcntBitMask(ISA);
  SMLoc S = getLoc();

  // If parse failed, do not return error code
  // to avoid excessive error messages.
  if (isToken(AsmToken::Identifier) && peekToken().is(AsmToken::LParen)) {
    while (parseCnt(Waitcnt) && !isToken(AsmToken::EndOfStatement));
  } else {
    parseExpr(Waitcnt);
  }

  Operands.push_back(PPUOperand::CreateImm(this, Waitcnt, S));
  return MatchOperand_Success;
}

bool
PPUOperand::isSWaitCnt() const {
  return isImm();
}

//===----------------------------------------------------------------------===//
// hwreg
//===----------------------------------------------------------------------===//
/*
bool
PPUAsmParser::parseHwregBody(OperandInfoTy &HwReg,
                                int64_t &Offset,
                                int64_t &Width) {
  using namespace llvm::PPU::Hwreg;

  // The register may be specified by name or using a numeric code
  if (isToken(AsmToken::Identifier) &&
      (HwReg.Id = getHwregId(getTokenStr())) >= 0) {
    HwReg.IsSymbolic = true;
    lex(); // skip message name
  } else if (!parseExpr(HwReg.Id)) {
    return false;
  }

  if (trySkipToken(AsmToken::RParen))
    return true;

  // parse optional params
  return
    skipToken(AsmToken::Comma, "expected a comma or a closing parenthesis") &&
    parseExpr(Offset) &&
    skipToken(AsmToken::Comma, "expected a comma") &&
    parseExpr(Width) &&
    skipToken(AsmToken::RParen, "expected a closing parenthesis");
}

bool
PPUAsmParser::validateHwreg(const OperandInfoTy &HwReg,
                               const int64_t Offset,
                               const int64_t Width,
                               const SMLoc Loc) {

  using namespace llvm::PPU::Hwreg;

  if (HwReg.IsSymbolic && !isValidHwreg(HwReg.Id, getSTI())) {
    Error(Loc, "specified hardware register is not supported on this GPU");
    return false;
  } else if (!isValidHwreg(HwReg.Id)) {
    Error(Loc, "invalid code of hardware register: only 6-bit values are legal");
    return false;
  } else if (!isValidHwregOffset(Offset)) {
    Error(Loc, "invalid bit offset: only 5-bit values are legal");
    return false;
  } else if (!isValidHwregWidth(Width)) {
    Error(Loc, "invalid bitfield width: only values from 1 to 32 are legal");
    return false;
  }
  return true;
}

OperandMatchResultTy
PPUAsmParser::parseHwreg(OperandVector &Operands) {
  using namespace llvm::PPU::Hwreg;

  int64_t ImmVal = 0;
  SMLoc Loc = getLoc();

  // If parse failed, do not return error code
  // to avoid excessive error messages.
  if (trySkipId("hwreg", AsmToken::LParen)) {
    OperandInfoTy HwReg(ID_UNKNOWN_);
    int64_t Offset = OFFSET_DEFAULT_;
    int64_t Width = WIDTH_DEFAULT_;
    if (parseHwregBody(HwReg, Offset, Width) &&
        validateHwreg(HwReg, Offset, Width, Loc)) {
      ImmVal = encodeHwreg(HwReg.Id, Offset, Width);
    }
  } else if (parseExpr(ImmVal)) {
    if (ImmVal < 0 || !isUInt<16>(ImmVal))
      Error(Loc, "invalid immediate: only 16-bit values are legal");
  }

  Operands.push_back(PPUOperand::CreateImm(this, ImmVal, Loc, PPUOperand::ImmTyHwreg));
  return MatchOperand_Success;
}

*/
bool PPUOperand::isHwreg() const {
  return isImmTy(ImmTyHwreg);
}
/*
//===----------------------------------------------------------------------===//
// sendmsg
//===----------------------------------------------------------------------===//

bool
PPUAsmParser::parseSendMsgBody(OperandInfoTy &Msg,
                                  OperandInfoTy &Op,
                                  OperandInfoTy &Stream) {
  using namespace llvm::PPU::SendMsg;

  if (isToken(AsmToken::Identifier) && (Msg.Id = getMsgId(getTokenStr())) >= 0) {
    Msg.IsSymbolic = true;
    lex(); // skip message name
  } else if (!parseExpr(Msg.Id)) {
    return false;
  }

  if (trySkipToken(AsmToken::Comma)) {
    Op.IsDefined = true;
    if (isToken(AsmToken::Identifier) &&
        (Op.Id = getMsgOpId(Msg.Id, getTokenStr())) >= 0) {
      lex(); // skip operation name
    } else if (!parseExpr(Op.Id)) {
      return false;
    }

    if (trySkipToken(AsmToken::Comma)) {
      Stream.IsDefined = true;
      if (!parseExpr(Stream.Id))
        return false;
    }
  }

  return skipToken(AsmToken::RParen, "expected a closing parenthesis");
}

bool
PPUAsmParser::validateSendMsg(const OperandInfoTy &Msg,
                                 const OperandInfoTy &Op,
                                 const OperandInfoTy &Stream,
                                 const SMLoc S) {
  using namespace llvm::PPU::SendMsg;

  // Validation strictness depends on whether message is specified
  // in a symbolc or in a numeric form. In the latter case
  // only encoding possibility is checked.
  bool Strict = Msg.IsSymbolic;

  if (!isValidMsgId(Msg.Id, getSTI(), Strict)) {
    Error(S, "invalid message id");
    return false;
  } else if (Strict && (msgRequiresOp(Msg.Id) != Op.IsDefined)) {
    Error(S, Op.IsDefined ?
             "message does not support operations" :
             "missing message operation");
    return false;
  } else if (!isValidMsgOp(Msg.Id, Op.Id, Strict)) {
    Error(S, "invalid operation id");
    return false;
  } else if (Strict && !msgSupportsStream(Msg.Id, Op.Id) && Stream.IsDefined) {
    Error(S, "message operation does not support streams");
    return false;
  } else if (!isValidMsgStream(Msg.Id, Op.Id, Stream.Id, Strict)) {
    Error(S, "invalid message stream id");
    return false;
  }
  return true;
}

OperandMatchResultTy
PPUAsmParser::parseSendMsgOp(OperandVector &Operands) {
  using namespace llvm::PPU::SendMsg;

  int64_t ImmVal = 0;
  SMLoc Loc = getLoc();

  // If parse failed, do not return error code
  // to avoid excessive error messages.
  if (trySkipId("sendmsg", AsmToken::LParen)) {
    OperandInfoTy Msg(ID_UNKNOWN_);
    OperandInfoTy Op(OP_NONE_);
    OperandInfoTy Stream(STREAM_ID_NONE_);
    if (parseSendMsgBody(Msg, Op, Stream) &&
        validateSendMsg(Msg, Op, Stream, Loc)) {
      ImmVal = encodeMsg(Msg.Id, Op.Id, Stream.Id);
    }
  } else if (parseExpr(ImmVal)) {
    if (ImmVal < 0 || !isUInt<16>(ImmVal))
      Error(Loc, "invalid immediate: only 16-bit values are legal");
  }

  Operands.push_back(PPUOperand::CreateImm(this, ImmVal, Loc, PPUOperand::ImmTySendMsg));
  return MatchOperand_Success;
}

bool PPUOperand::isSendMsg() const {
  return isImmTy(ImmTySendMsg);
}
*/


//===----------------------------------------------------------------------===//
// exp
//===----------------------------------------------------------------------===//
/*
void PPUAsmParser::errorExpTgt() {
  Error(Parser.getTok().getLoc(), "invalid exp target");
}

OperandMatchResultTy PPUAsmParser::parseExpTgtImpl(StringRef Str,
                                                      uint8_t &Val) {
  if (Str == "null") {
    Val = 9;
    return MatchOperand_Success;
  }

  if (Str.startswith("mrt")) {
    Str = Str.drop_front(3);
    if (Str == "z") { // == mrtz
      Val = 8;
      return MatchOperand_Success;
    }

    if (Str.getAsInteger(10, Val))
      return MatchOperand_ParseFail;

    if (Val > 7)
      errorExpTgt();

    return MatchOperand_Success;
  }

  if (Str.startswith("pos")) {
    Str = Str.drop_front(3);
    if (Str.getAsInteger(10, Val))
      return MatchOperand_ParseFail;

    if (Val > 4 || (Val == 4 && !isGFX10()))
      errorExpTgt();

    Val += 12;
    return MatchOperand_Success;
  }

  if (isGFX10() && Str == "prim") {
    Val = 20;
    return MatchOperand_Success;
  }

  if (Str.startswith("param")) {
    Str = Str.drop_front(5);
    if (Str.getAsInteger(10, Val))
      return MatchOperand_ParseFail;

    if (Val >= 32)
      errorExpTgt();

    Val += 32;
    return MatchOperand_Success;
  }

  if (Str.startswith("invalid_target_")) {
    Str = Str.drop_front(15);
    if (Str.getAsInteger(10, Val))
      return MatchOperand_ParseFail;

    errorExpTgt();
    return MatchOperand_Success;
  }

  return MatchOperand_NoMatch;
}

OperandMatchResultTy PPUAsmParser::parseExpTgt(OperandVector &Operands) {
  uint8_t Val;
  StringRef Str = Parser.getTok().getString();

  auto Res = parseExpTgtImpl(Str, Val);
  if (Res != MatchOperand_Success)
    return Res;

  SMLoc S = Parser.getTok().getLoc();
  Parser.Lex();

  Operands.push_back(PPUOperand::CreateImm(this, Val, S,
                                              PPUOperand::ImmTyExpTgt));
  return MatchOperand_Success;
}
*/

//===----------------------------------------------------------------------===//
// parser helpers
//===----------------------------------------------------------------------===//

bool
PPUAsmParser::isId(const AsmToken &Token, const StringRef Id) const {
  return Token.is(AsmToken::Identifier) && Token.getString() == Id;
}

bool
PPUAsmParser::isId(const StringRef Id) const {
  return isId(getToken(), Id);
}

bool
PPUAsmParser::isToken(const AsmToken::TokenKind Kind) const {
  return getTokenKind() == Kind;
}

bool
PPUAsmParser::trySkipId(const StringRef Id) {
  if (isId(Id)) {
    lex();
    return true;
  }
  return false;
}

bool
PPUAsmParser::trySkipId(const StringRef Id, const AsmToken::TokenKind Kind) {
  if (isId(Id) && peekToken().is(Kind)) {
    lex();
    lex();
    return true;
  }
  return false;
}

bool
PPUAsmParser::trySkipToken(const AsmToken::TokenKind Kind) {
  if (isToken(Kind)) {
    lex();
    return true;
  }
  return false;
}

bool
PPUAsmParser::skipToken(const AsmToken::TokenKind Kind,
                           const StringRef ErrMsg) {
  if (!trySkipToken(Kind)) {
    Error(getLoc(), ErrMsg);
    return false;
  }
  return true;
}

bool
PPUAsmParser::parseExpr(int64_t &Imm) {
  return !getParser().parseAbsoluteExpression(Imm);
}

bool
PPUAsmParser::parseExpr(OperandVector &Operands) {
  SMLoc S = getLoc();

  const MCExpr *Expr;
  if (Parser.parseExpression(Expr))
    return false;

  int64_t IntVal;
  if (Expr->evaluateAsAbsolute(IntVal)) {
    Operands.push_back(PPUOperand::CreateImm(this, IntVal, S));
  } else {
    Operands.push_back(PPUOperand::CreateExpr(this, Expr, S));
  }
  return true;
}

bool
PPUAsmParser::parseString(StringRef &Val, const StringRef ErrMsg) {
  if (isToken(AsmToken::String)) {
    Val = getToken().getStringContents();
    lex();
    return true;
  } else {
    Error(getLoc(), ErrMsg);
    return false;
  }
}

AsmToken
PPUAsmParser::getToken() const {
  return Parser.getTok();
}

AsmToken
PPUAsmParser::peekToken() {
  return getLexer().peekTok();
}

void
PPUAsmParser::peekTokens(MutableArrayRef<AsmToken> Tokens) {
  auto TokCount = getLexer().peekTokens(Tokens);

  for (auto Idx = TokCount; Idx < Tokens.size(); ++Idx)
    Tokens[Idx] = AsmToken(AsmToken::Error, "");
}

AsmToken::TokenKind
PPUAsmParser::getTokenKind() const {
  return getLexer().getKind();
}

SMLoc
PPUAsmParser::getLoc() const {
  return getToken().getLoc();
}

StringRef
PPUAsmParser::getTokenStr() const {
  return getToken().getString();
}

void
PPUAsmParser::lex() {
  Parser.Lex();
}

//===----------------------------------------------------------------------===//
// swizzle
//===----------------------------------------------------------------------===//

LLVM_READNONE
static unsigned
encodeBitmaskPerm(const unsigned AndMask,
                  const unsigned OrMask,
                  const unsigned XorMask) {
  using namespace llvm::PPU::Swizzle;

  return BITMASK_PERM_ENC |
         (AndMask << BITMASK_AND_SHIFT) |
         (OrMask  << BITMASK_OR_SHIFT)  |
         (XorMask << BITMASK_XOR_SHIFT);
}

bool
PPUAsmParser::parseSwizzleOperands(const unsigned OpNum, int64_t* Op,
                                      const unsigned MinVal,
                                      const unsigned MaxVal,
                                      const StringRef ErrMsg) {
  for (unsigned i = 0; i < OpNum; ++i) {
    if (!skipToken(AsmToken::Comma, "expected a comma")){
      return false;
    }
    SMLoc ExprLoc = Parser.getTok().getLoc();
    if (!parseExpr(Op[i])) {
      return false;
    }
    if (Op[i] < MinVal || Op[i] > MaxVal) {
      Error(ExprLoc, ErrMsg);
      return false;
    }
  }

  return true;
}

bool
PPUAsmParser::parseSwizzleQuadPerm(int64_t &Imm) {
  using namespace llvm::PPU::Swizzle;

  int64_t Lane[LANE_NUM];
  if (parseSwizzleOperands(LANE_NUM, Lane, 0, LANE_MAX,
                           "expected a 2-bit lane id")) {
    Imm = QUAD_PERM_ENC;
    for (unsigned I = 0; I < LANE_NUM; ++I) {
      Imm |= Lane[I] << (LANE_SHIFT * I);
    }
    return true;
  }
  return false;
}

bool
PPUAsmParser::parseSwizzleBroadcast(int64_t &Imm) {
  using namespace llvm::PPU::Swizzle;

  SMLoc S = Parser.getTok().getLoc();
  int64_t GroupSize;
  int64_t LaneIdx;

  if (!parseSwizzleOperands(1, &GroupSize,
                            2, 32,
                            "group size must be in the interval [2,32]")) {
    return false;
  }
  if (!isPowerOf2_64(GroupSize)) {
    Error(S, "group size must be a power of two");
    return false;
  }
  if (parseSwizzleOperands(1, &LaneIdx,
                           0, GroupSize - 1,
                           "lane id must be in the interval [0,group size - 1]")) {
    Imm = encodeBitmaskPerm(BITMASK_MAX - GroupSize + 1, LaneIdx, 0);
    return true;
  }
  return false;
}

bool
PPUAsmParser::parseSwizzleReverse(int64_t &Imm) {
  using namespace llvm::PPU::Swizzle;

  SMLoc S = Parser.getTok().getLoc();
  int64_t GroupSize;

  if (!parseSwizzleOperands(1, &GroupSize,
      2, 32, "group size must be in the interval [2,32]")) {
    return false;
  }
  if (!isPowerOf2_64(GroupSize)) {
    Error(S, "group size must be a power of two");
    return false;
  }

  Imm = encodeBitmaskPerm(BITMASK_MAX, 0, GroupSize - 1);
  return true;
}

bool
PPUAsmParser::parseSwizzleSwap(int64_t &Imm) {
  using namespace llvm::PPU::Swizzle;

  SMLoc S = Parser.getTok().getLoc();
  int64_t GroupSize;

  if (!parseSwizzleOperands(1, &GroupSize,
      1, 16, "group size must be in the interval [1,16]")) {
    return false;
  }
  if (!isPowerOf2_64(GroupSize)) {
    Error(S, "group size must be a power of two");
    return false;
  }

  Imm = encodeBitmaskPerm(BITMASK_MAX, 0, GroupSize);
  return true;
}

bool
PPUAsmParser::parseSwizzleBitmaskPerm(int64_t &Imm) {
  using namespace llvm::PPU::Swizzle;

  if (!skipToken(AsmToken::Comma, "expected a comma")) {
    return false;
  }

  StringRef Ctl;
  SMLoc StrLoc = Parser.getTok().getLoc();
  if (!parseString(Ctl)) {
    return false;
  }
  if (Ctl.size() != BITMASK_WIDTH) {
    Error(StrLoc, "expected a 5-character mask");
    return false;
  }

  unsigned AndMask = 0;
  unsigned OrMask = 0;
  unsigned XorMask = 0;

  for (size_t i = 0; i < Ctl.size(); ++i) {
    unsigned Mask = 1 << (BITMASK_WIDTH - 1 - i);
    switch(Ctl[i]) {
    default:
      Error(StrLoc, "invalid mask");
      return false;
    case '0':
      break;
    case '1':
      OrMask |= Mask;
      break;
    case 'p':
      AndMask |= Mask;
      break;
    case 'i':
      AndMask |= Mask;
      XorMask |= Mask;
      break;
    }
  }

  Imm = encodeBitmaskPerm(AndMask, OrMask, XorMask);
  return true;
}

bool
PPUAsmParser::parseSwizzleOffset(int64_t &Imm) {

  SMLoc OffsetLoc = Parser.getTok().getLoc();

  if (!parseExpr(Imm)) {
    return false;
  }
  if (!isUInt<16>(Imm)) {
    Error(OffsetLoc, "expected a 16-bit offset");
    return false;
  }
  return true;
}

bool
PPUAsmParser::parseSwizzleMacro(int64_t &Imm) {
  using namespace llvm::PPU::Swizzle;

  if (skipToken(AsmToken::LParen, "expected a left parentheses")) {

    SMLoc ModeLoc = Parser.getTok().getLoc();
    bool Ok = false;

    if (trySkipId(IdSymbolic[ID_QUAD_PERM])) {
      Ok = parseSwizzleQuadPerm(Imm);
    } else if (trySkipId(IdSymbolic[ID_BITMASK_PERM])) {
      Ok = parseSwizzleBitmaskPerm(Imm);
    } else if (trySkipId(IdSymbolic[ID_BROADCAST])) {
      Ok = parseSwizzleBroadcast(Imm);
    } else if (trySkipId(IdSymbolic[ID_SWAP])) {
      Ok = parseSwizzleSwap(Imm);
    } else if (trySkipId(IdSymbolic[ID_REVERSE])) {
      Ok = parseSwizzleReverse(Imm);
    } else {
      Error(ModeLoc, "expected a swizzle mode");
    }

    return Ok && skipToken(AsmToken::RParen, "expected a closing parentheses");
  }

  return false;
}

OperandMatchResultTy
PPUAsmParser::parseSwizzleOp(OperandVector &Operands) {
  SMLoc S = Parser.getTok().getLoc();
  int64_t Imm = 0;

  if (trySkipId("offset")) {

    bool Ok = false;
    if (skipToken(AsmToken::Colon, "expected a colon")) {
      if (trySkipId("swizzle")) {
        Ok = parseSwizzleMacro(Imm);
      } else {
        Ok = parseSwizzleOffset(Imm);
      }
    }

    Operands.push_back(PPUOperand::CreateImm(this, Imm, S, PPUOperand::ImmTySwizzle));

    return Ok? MatchOperand_Success : MatchOperand_ParseFail;
  } else {
    // Swizzle "offset" operand is optional.
    // If it is omitted, try parsing other optional operands.
    return parseOptionalOpr(Operands);
  }
}

bool
PPUOperand::isSwizzle() const {
  return isImmTy(ImmTySwizzle);
}

//===----------------------------------------------------------------------===//
// VGPR Index Mode
//===----------------------------------------------------------------------===//

int64_t PPUAsmParser::parseGPRIdxMacro() {

  using namespace llvm::PPU::VGPRIndexMode;

  if (trySkipToken(AsmToken::RParen)) {
    return OFF;
  }

  int64_t Imm = 0;

  while (true) {
    unsigned Mode = 0;
    SMLoc S = Parser.getTok().getLoc();

    for (unsigned ModeId = ID_MIN; ModeId <= ID_MAX; ++ModeId) {
        /* TODO
      if (trySkipId(IdSymbolic[ModeId])) {
        Mode = 1 << ModeId;
        break;
      }
      */
    }

    if (Mode == 0) {
      Error(S, (Imm == 0)?
               "expected a VGPR index mode or a closing parenthesis" :
               "expected a VGPR index mode");
      break;
    }

    if (Imm & Mode) {
      Error(S, "duplicate VGPR index mode");
      break;
    }
    Imm |= Mode;

    if (trySkipToken(AsmToken::RParen))
      break;
    if (!skipToken(AsmToken::Comma,
                   "expected a comma or a closing parenthesis"))
      break;
  }

  return Imm;
}

OperandMatchResultTy
PPUAsmParser::parseVPRIdxMode(OperandVector &Operands) {

  int64_t Imm = 0;
  SMLoc S = Parser.getTok().getLoc();

  if (getLexer().getKind() == AsmToken::Identifier &&
      Parser.getTok().getString() == "gpr_idx" &&
      getLexer().peekTok().is(AsmToken::LParen)) {

    Parser.Lex();
    Parser.Lex();

    // If parse failed, trigger an error but do not return error code
    // to avoid excessive error messages.
    Imm = parseGPRIdxMacro();

  } else {
    if (getParser().parseAbsoluteExpression(Imm))
      return MatchOperand_NoMatch;
    if (Imm < 0 || !isUInt<4>(Imm)) {
      Error(S, "invalid immediate: only 4-bit values are legal");
    }
  }

  Operands.push_back(
      PPUOperand::CreateImm(this, Imm, S, PPUOperand::ImmTyGprIdxMode));
  return MatchOperand_Success;
}

bool PPUOperand::isVPRIdxMode() const {
  return isImmTy(ImmTyGprIdxMode);
}

//===----------------------------------------------------------------------===//
// sopp branch targets
//===----------------------------------------------------------------------===//

OperandMatchResultTy
PPUAsmParser::parseSOppBrTarget(OperandVector &Operands) {

  // Make sure we are not parsing something
  // that looks like a label or an expression but is not.
  // This will improve error messages.
  if (isRegister() || isModifier())
    return MatchOperand_NoMatch;

  if (parseExpr(Operands)) {

    PPUOperand &Opr = ((PPUOperand &)*Operands[Operands.size() - 1]);
    assert(Opr.isImm() || Opr.isExpr());
    SMLoc Loc = Opr.getStartLoc();

    // Currently we do not support arbitrary expressions as branch targets.
    // Only labels and absolute expressions are accepted.
    if (Opr.isExpr() && !Opr.isSymbolRefExpr()) {
      Error(Loc, "expected an absolute expression or a label");
    } else if (Opr.isImm() && !Opr.isS16Imm()) {
      Error(Loc, "expected a 16-bit signed jump offset");
    }
  }

  return MatchOperand_Success; // avoid excessive error messages
}

//===----------------------------------------------------------------------===//
// Boolean holding registers
//===----------------------------------------------------------------------===//

OperandMatchResultTy
PPUAsmParser::parseBoolReg(OperandVector &Operands) {
  return parseReg(Operands);
}

//===----------------------------------------------------------------------===//
// mubuf
//===----------------------------------------------------------------------===//

PPUOperand::Ptr PPUAsmParser::defaultDLC() const {
  return PPUOperand::CreateImm(this, 0, SMLoc(), PPUOperand::ImmTyDLC);
}

PPUOperand::Ptr PPUAsmParser::defaultGLC() const {
  return PPUOperand::CreateImm(this, 0, SMLoc(), PPUOperand::ImmTyGLC);
}

PPUOperand::Ptr PPUAsmParser::defaultSLC() const {
  return PPUOperand::CreateImm(this, 0, SMLoc(), PPUOperand::ImmTySLC);
}

void PPUAsmParser::cvtMubufImpl(MCInst &Inst,
                               const OperandVector &Operands,
                               bool IsAtomic,
                               bool IsAtomicReturn,
                               bool IsLds) {
  bool IsLdsOpcode = IsLds;
  bool HasLdsModifier = false;
  OptionalImmIndexMap OptionalIdx;
  assert(IsAtomicReturn ? IsAtomic : true);
  unsigned FirstOperandIdx = 1;

  for (unsigned i = FirstOperandIdx, e = Operands.size(); i != e; ++i) {
    PPUOperand &Op = ((PPUOperand &)*Operands[i]);

    // Add the register arguments
    if (Op.isReg()) {
      Op.addRegOperands(Inst, 1);
      // Insert a tied src for atomic return dst.
      // This cannot be postponed as subsequent calls to
      // addImmOperands rely on correct number of MC operands.
      if (IsAtomicReturn && i == FirstOperandIdx)
        Op.addRegOperands(Inst, 1);
      continue;
    }

    // Handle the case where soffset is an immediate
    if (Op.isImm() && Op.getImmTy() == PPUOperand::ImmTyNone) {
      Op.addImmOperands(Inst, 1);
      continue;
    }

    HasLdsModifier |= Op.isLDS();

    // Handle tokens like 'offen' which are sometimes hard-coded into the
    // asm string.  There are no MCInst operands for these.
    if (Op.isToken()) {
      continue;
    }
    assert(Op.isImm());

    // Handle optional arguments
    OptionalIdx[Op.getImmTy()] = i;
  }

  // This is a workaround for an llvm quirk which may result in an
  // incorrect instruction selection. Lds and non-lds versions of
  // MUBUF instructions are identical except that lds versions
  // have mandatory 'lds' modifier. However this modifier follows
  // optional modifiers and llvm asm matcher regards this 'lds'
  // modifier as an optional one. As a result, an lds version
  // of opcode may be selected even if it has no 'lds' modifier.
  if (IsLdsOpcode && !HasLdsModifier) {
      llvm_unreachable("fixme getMUBUFNoLdsInst ");
      /*
    int NoLdsOpcode = PPU::getMUBUFNoLdsInst(Inst.getOpcode());
    if (NoLdsOpcode != -1) { // Got lds version - correct it.
      Inst.setOpcode(NoLdsOpcode);
      IsLdsOpcode = false;
    }
    */
  }

  addOptionalImmOperand(Inst, Operands, OptionalIdx, PPUOperand::ImmTyOffset);
  if (!IsAtomic) { // glc is hard-coded.
    addOptionalImmOperand(Inst, Operands, OptionalIdx, PPUOperand::ImmTyGLC);
  }
  addOptionalImmOperand(Inst, Operands, OptionalIdx, PPUOperand::ImmTySLC);

  if (!IsLdsOpcode) { // tfe is not legal with lds opcodes
    addOptionalImmOperand(Inst, Operands, OptionalIdx, PPUOperand::ImmTyTFE);
  }

  if (isGFX10())
    addOptionalImmOperand(Inst, Operands, OptionalIdx, PPUOperand::ImmTyDLC);
}

void PPUAsmParser::cvtMtbuf(MCInst &Inst, const OperandVector &Operands) {
  OptionalImmIndexMap OptionalIdx;

  for (unsigned i = 1, e = Operands.size(); i != e; ++i) {
    PPUOperand &Op = ((PPUOperand &)*Operands[i]);

    // Add the register arguments
    if (Op.isReg()) {
      Op.addRegOperands(Inst, 1);
      continue;
    }

    // Handle the case where soffset is an immediate
    if (Op.isImm() && Op.getImmTy() == PPUOperand::ImmTyNone) {
      Op.addImmOperands(Inst, 1);
      continue;
    }

    // Handle tokens like 'offen' which are sometimes hard-coded into the
    // asm string.  There are no MCInst operands for these.
    if (Op.isToken()) {
      continue;
    }
    assert(Op.isImm());

    // Handle optional arguments
    OptionalIdx[Op.getImmTy()] = i;
  }

  addOptionalImmOperand(Inst, Operands, OptionalIdx,
                        PPUOperand::ImmTyOffset);
  addOptionalImmOperand(Inst, Operands, OptionalIdx, PPUOperand::ImmTyFORMAT);
  addOptionalImmOperand(Inst, Operands, OptionalIdx, PPUOperand::ImmTyGLC);
  addOptionalImmOperand(Inst, Operands, OptionalIdx, PPUOperand::ImmTySLC);
  addOptionalImmOperand(Inst, Operands, OptionalIdx, PPUOperand::ImmTyTFE);

  if (isGFX10())
    addOptionalImmOperand(Inst, Operands, OptionalIdx, PPUOperand::ImmTyDLC);
}

//===----------------------------------------------------------------------===//
// mimg
//===----------------------------------------------------------------------===//
/*
void PPUAsmParser::cvtMIMG(MCInst &Inst, const OperandVector &Operands,
                              bool IsAtomic) {
  unsigned I = 1;
  const MCInstrDesc &Desc = MII.get(Inst.getOpcode());
  for (unsigned J = 0; J < Desc.getNumDefs(); ++J) {
    ((PPUOperand &)*Operands[I++]).addRegOperands(Inst, 1);
  }

  if (IsAtomic) {
    // Add src, same as dst
    assert(Desc.getNumDefs() == 1);
    ((PPUOperand &)*Operands[I - 1]).addRegOperands(Inst, 1);
  }

  OptionalImmIndexMap OptionalIdx;

  for (unsigned E = Operands.size(); I != E; ++I) {
    PPUOperand &Op = ((PPUOperand &)*Operands[I]);

    // Add the register arguments
    if (Op.isReg()) {
      Op.addRegOperands(Inst, 1);
    } else if (Op.isImmModifier()) {
      OptionalIdx[Op.getImmTy()] = I;
    } else if (!Op.isToken()) {
      llvm_unreachable("unexpected operand type");
    }
  }

  bool IsGFX10 = isGFX10();

  addOptionalImmOperand(Inst, Operands, OptionalIdx, PPUOperand::ImmTyDMask);
  if (IsGFX10)
    addOptionalImmOperand(Inst, Operands, OptionalIdx, PPUOperand::ImmTyDim, -1);
  addOptionalImmOperand(Inst, Operands, OptionalIdx, PPUOperand::ImmTyUNorm);
  if (IsGFX10)
    addOptionalImmOperand(Inst, Operands, OptionalIdx, PPUOperand::ImmTyDLC);
  addOptionalImmOperand(Inst, Operands, OptionalIdx, PPUOperand::ImmTyGLC);
  addOptionalImmOperand(Inst, Operands, OptionalIdx, PPUOperand::ImmTySLC);
  addOptionalImmOperand(Inst, Operands, OptionalIdx, PPUOperand::ImmTyR128A16);
  addOptionalImmOperand(Inst, Operands, OptionalIdx, PPUOperand::ImmTyTFE);
  addOptionalImmOperand(Inst, Operands, OptionalIdx, PPUOperand::ImmTyLWE);
  if (!IsGFX10)
    addOptionalImmOperand(Inst, Operands, OptionalIdx, PPUOperand::ImmTyDA);
  addOptionalImmOperand(Inst, Operands, OptionalIdx, PPUOperand::ImmTyD16);
}

void PPUAsmParser::cvtMIMGAtomic(MCInst &Inst, const OperandVector &Operands) {
  cvtMIMG(Inst, Operands, true);
}
*/

//===----------------------------------------------------------------------===//
// smrd
//===----------------------------------------------------------------------===//

bool PPUOperand::isSMRDOffset8() const {
  return isImm() && isUInt<8>(getImmInt());
}

bool PPUOperand::isSMRDOffset20() const {
  return isImm() && isUInt<20>(getImmInt());
}

bool PPUOperand::isSMRDLiteralOffset() const {
  // 32-bit literals are only supported on CI and we only want to use them
  // when the offset is > 8-bits.
  return isImm() && !isUInt<8>(getImmInt()) && isUInt<32>(getImmInt());
}

PPUOperand::Ptr PPUAsmParser::defaultSMRDOffset8() const {
  return PPUOperand::CreateImm(this, 0, SMLoc(), PPUOperand::ImmTyOffset);
}

PPUOperand::Ptr PPUAsmParser::defaultSMRDOffset20() const {
  return PPUOperand::CreateImm(this, 0, SMLoc(), PPUOperand::ImmTyOffset);
}

PPUOperand::Ptr PPUAsmParser::defaultSMRDLiteralOffset() const {
  return PPUOperand::CreateImm(this, 0, SMLoc(), PPUOperand::ImmTyOffset);
}

PPUOperand::Ptr PPUAsmParser::defaultFlatOffset() const {
  return PPUOperand::CreateImm(this, 0, SMLoc(), PPUOperand::ImmTyOffset);
}

//===----------------------------------------------------------------------===//
// vop3
//===----------------------------------------------------------------------===//

static bool ConvertOmodMul(int64_t &Mul) {
  if (Mul != 1 && Mul != 2 && Mul != 4)
    return false;

  Mul >>= 1;
  return true;
}

static bool ConvertOmodDiv(int64_t &Div) {
  if (Div == 1) {
    Div = 0;
    return true;
  }

  if (Div == 2) {
    Div = 3;
    return true;
  }

  return false;
}

static bool ConvertBoundCtrl(int64_t &BoundCtrl) {
  if (BoundCtrl == 0) {
    BoundCtrl = 1;
    return true;
  }

  if (BoundCtrl == -1) {
    BoundCtrl = 0;
    return true;
  }

  return false;
}

// Note: the order in this table matches the order of operands in AsmString.
static const OptionalOperand PPUOptionalOperandTable[] = {
  {"offen",   PPUOperand::ImmTyOffen, true, nullptr},
  {"idxen",   PPUOperand::ImmTyIdxen, true, nullptr},
  {"addr64",  PPUOperand::ImmTyAddr64, true, nullptr},
  {"offset0", PPUOperand::ImmTyOffset0, false, nullptr},
  {"offset1", PPUOperand::ImmTyOffset1, false, nullptr},
  {"gds",     PPUOperand::ImmTyGDS, true, nullptr},
  {"lds",     PPUOperand::ImmTyLDS, true, nullptr},
  {"offset",  PPUOperand::ImmTyOffset, false, nullptr},
  {"inst_offset", PPUOperand::ImmTyInstOffset, false, nullptr},
  {"dlc",     PPUOperand::ImmTyDLC, true, nullptr},
  {"format",  PPUOperand::ImmTyFORMAT, false, nullptr},
  {"glc",     PPUOperand::ImmTyGLC, true, nullptr},
  {"slc",     PPUOperand::ImmTySLC, true, nullptr},
  {"tfe",     PPUOperand::ImmTyTFE, true, nullptr},
  {"d16",     PPUOperand::ImmTyD16, true, nullptr},
  {"high",    PPUOperand::ImmTyHigh, true, nullptr},
  {"clamp",   PPUOperand::ImmTyClampSI, true, nullptr},
  {"omod",    PPUOperand::ImmTyOModSI, false, ConvertOmodMul},
  {"unorm",   PPUOperand::ImmTyUNorm, true, nullptr},
  {"da",      PPUOperand::ImmTyDA,    true, nullptr},
  {"r128",    PPUOperand::ImmTyR128A16,  true, nullptr},
  {"a16",     PPUOperand::ImmTyR128A16,  true, nullptr},
  {"lwe",     PPUOperand::ImmTyLWE,   true, nullptr},
  {"d16",     PPUOperand::ImmTyD16,   true, nullptr},
  {"dmask",   PPUOperand::ImmTyDMask, false, nullptr},
  {"dim",     PPUOperand::ImmTyDim,   false, nullptr},
  {"row_mask",   PPUOperand::ImmTyDppRowMask, false, nullptr},
  {"bank_mask",  PPUOperand::ImmTyDppBankMask, false, nullptr},
  {"bound_ctrl", PPUOperand::ImmTyDppBoundCtrl, false, ConvertBoundCtrl},
  {"fi",         PPUOperand::ImmTyDppFi, false, nullptr},
  {"dst_sel",    PPUOperand::ImmTySdwaDstSel, false, nullptr},
  {"src0_sel",   PPUOperand::ImmTySdwaSrc0Sel, false, nullptr},
  {"src1_sel",   PPUOperand::ImmTySdwaSrc1Sel, false, nullptr},
  {"dst_unused", PPUOperand::ImmTySdwaDstUnused, false, nullptr},
  {"compr", PPUOperand::ImmTyExpCompr, true, nullptr },
  {"vm", PPUOperand::ImmTyExpVM, true, nullptr},
  {"op_sel", PPUOperand::ImmTyOpSel, false, nullptr},
  {"op_sel_hi", PPUOperand::ImmTyOpSelHi, false, nullptr},
  {"neg_lo", PPUOperand::ImmTyNegLo, false, nullptr},
  {"neg_hi", PPUOperand::ImmTyNegHi, false, nullptr},
  {"blgp", PPUOperand::ImmTyBLGP, false, nullptr},
  {"cbsz", PPUOperand::ImmTyCBSZ, false, nullptr},
  {"abid", PPUOperand::ImmTyABID, false, nullptr}
};

OperandMatchResultTy PPUAsmParser::parseOptionalOperand(OperandVector &Operands) {
  unsigned size = Operands.size();
  assert(size > 0);

  OperandMatchResultTy res = parseOptionalOpr(Operands);

  // This is a hack to enable hardcoded mandatory operands which follow
  // optional operands.
  //
  // Current design assumes that all operands after the first optional operand
  // are also optional. However implementation of some instructions violates
  // this rule (see e.g. flat/global atomic which have hardcoded 'glc' operands).
  //
  // To alleviate this problem, we have to (implicitly) parse extra operands
  // to make sure autogenerated parser of custom operands never hit hardcoded
  // mandatory operands.

  if (size == 1 || ((PPUOperand &)*Operands[size - 1]).isRegKind()) {

    // We have parsed the first optional operand.
    // Parse as many operands as necessary to skip all mandatory operands.

    for (unsigned i = 0; i < MAX_OPR_LOOKAHEAD; ++i) {
      if (res != MatchOperand_Success ||
          getLexer().is(AsmToken::EndOfStatement)) break;
      if (getLexer().is(AsmToken::Comma)) Parser.Lex();
      res = parseOptionalOpr(Operands);
    }
  }

  return res;
}

OperandMatchResultTy PPUAsmParser::parseOptionalOpr(OperandVector &Operands) {
  OperandMatchResultTy res;
  for (const OptionalOperand &Op : PPUOptionalOperandTable) {
    // try to parse any optional operand here
    if (Op.IsBit) {
      res = parseNamedBit(Op.Name, Operands, Op.Type);
    } else if (Op.Type == PPUOperand::ImmTyOModSI) {
      res = parseOModOperand(Operands);
      /*
    } else if (Op.Type == PPUOperand::ImmTySdwaDstSel ||
               Op.Type == PPUOperand::ImmTySdwaSrc0Sel ||
               Op.Type == PPUOperand::ImmTySdwaSrc1Sel) {
      res = parseSDWASel(Operands, Op.Name, Op.Type);
    } else if (Op.Type == PPUOperand::ImmTySdwaDstUnused) {
      res = parseSDWADstUnused(Operands);
      */
    } else if (Op.Type == PPUOperand::ImmTyOpSel ||
               Op.Type == PPUOperand::ImmTyOpSelHi ||
               Op.Type == PPUOperand::ImmTyNegLo ||
               Op.Type == PPUOperand::ImmTyNegHi) {
      res = parseOperandArrayWithPrefix(Op.Name, Operands, Op.Type,
                                        Op.ConvertResult);
      /*
    } else if (Op.Type == PPUOperand::ImmTyDim) {
      res = parseDim(Operands);
      */
    } else if (Op.Type == PPUOperand::ImmTyFORMAT && !isGFX10()) {
      res = parseDfmtNfmt(Operands);
    } else {
      res = parseIntWithPrefix(Op.Name, Operands, Op.Type, Op.ConvertResult);
    }
    if (res != MatchOperand_NoMatch) {
      return res;
    }
  }
  return MatchOperand_NoMatch;
}

OperandMatchResultTy PPUAsmParser::parseOModOperand(OperandVector &Operands) {
  StringRef Name = Parser.getTok().getString();
  if (Name == "mul") {
    return parseIntWithPrefix("mul", Operands,
                              PPUOperand::ImmTyOModSI, ConvertOmodMul);
  }

  if (Name == "div") {
    return parseIntWithPrefix("div", Operands,
                              PPUOperand::ImmTyOModSI, ConvertOmodDiv);
  }

  return MatchOperand_NoMatch;
}

void PPUAsmParser::cvtVOP3OpSel(MCInst &Inst, const OperandVector &Operands) {
  cvtVOP3P(Inst, Operands);

  int Opc = Inst.getOpcode();

  int SrcNum;
  const int Ops[] = { PPU::OpName::src0,
                      PPU::OpName::src1,
                      PPU::OpName::src2 };
  for (SrcNum = 0;
       SrcNum < 3 && PPU::getNamedOperandIdx(Opc, Ops[SrcNum]) != -1;
       ++SrcNum);
  assert(SrcNum > 0);

  int OpSelIdx = PPU::getNamedOperandIdx(Opc, PPU::OpName::op_sel);
  unsigned OpSel = Inst.getOperand(OpSelIdx).getImm();

  if ((OpSel & (1 << SrcNum)) != 0) {
    int ModIdx = PPU::getNamedOperandIdx(Opc, PPU::OpName::src0_modifiers);
    uint32_t ModVal = Inst.getOperand(ModIdx).getImm();
    Inst.getOperand(ModIdx).setImm(ModVal | PPUSrcMods::DST_OP_SEL);
  }
}

static bool isRegOrImmWithInputMods(const MCInstrDesc &Desc, unsigned OpNum) {
      // 1. This operand is input modifiers
  return Desc.OpInfo[OpNum].OperandType == PPU::OPERAND_INPUT_MODS
      // 2. This is not last operand
      && Desc.NumOperands > (OpNum + 1)
      // 3. Next operand is register class
      && Desc.OpInfo[OpNum + 1].RegClass != -1
      // 4. Next register is not tied to any other operand
      && Desc.getOperandConstraint(OpNum + 1, MCOI::OperandConstraint::TIED_TO) == -1;
}
/*
void PPUAsmParser::cvtVOP3Interp(MCInst &Inst, const OperandVector &Operands)
{
  OptionalImmIndexMap OptionalIdx;
  unsigned Opc = Inst.getOpcode();

  unsigned I = 1;
  const MCInstrDesc &Desc = MII.get(Inst.getOpcode());
  for (unsigned J = 0; J < Desc.getNumDefs(); ++J) {
    ((PPUOperand &)*Operands[I++]).addRegOperands(Inst, 1);
  }

  for (unsigned E = Operands.size(); I != E; ++I) {
    PPUOperand &Op = ((PPUOperand &)*Operands[I]);
    if (isRegOrImmWithInputMods(Desc, Inst.getNumOperands())) {
      Op.addRegOrImmWithFPInputModsOperands(Inst, 2);
    } else if (Op.isInterpSlot() ||
               Op.isInterpAttr() ||
               Op.isAttrChan()) {
      Inst.addOperand(MCOperand::createImm(Op.getImmInt()));
    } else if (Op.isImmModifier()) {
      OptionalIdx[Op.getImmTy()] = I;
    } else {
      llvm_unreachable("unhandled operand type");
    }
  }

  if (PPU::getNamedOperandIdx(Opc, PPU::OpName::high) != -1) {
    addOptionalImmOperand(Inst, Operands, OptionalIdx, PPUOperand::ImmTyHigh);
  }

  if (PPU::getNamedOperandIdx(Opc, PPU::OpName::clamp) != -1) {
    addOptionalImmOperand(Inst, Operands, OptionalIdx, PPUOperand::ImmTyClampSI);
  }

  if (PPU::getNamedOperandIdx(Opc, PPU::OpName::omod) != -1) {
    addOptionalImmOperand(Inst, Operands, OptionalIdx, PPUOperand::ImmTyOModSI);
  }
}
*/

void PPUAsmParser::cvtVOP3(MCInst &Inst, const OperandVector &Operands,
                              OptionalImmIndexMap &OptionalIdx) {
  unsigned Opc = Inst.getOpcode();

  unsigned I = 1;
  const MCInstrDesc &Desc = MII.get(Inst.getOpcode());
  for (unsigned J = 0; J < Desc.getNumDefs(); ++J) {
    ((PPUOperand &)*Operands[I++]).addRegOperands(Inst, 1);
  }

  if (PPU::getNamedOperandIdx(Opc, PPU::OpName::src0_modifiers) != -1) {
    // This instruction has src modifiers
    for (unsigned E = Operands.size(); I != E; ++I) {
      PPUOperand &Op = ((PPUOperand &)*Operands[I]);
      if (isRegOrImmWithInputMods(Desc, Inst.getNumOperands())) {
        Op.addRegOrImmWithFPInputModsOperands(Inst, 2);
      } else if (Op.isImmModifier()) {
        OptionalIdx[Op.getImmTy()] = I;
      } else if (Op.isRegOrImm()) {
        Op.addRegOrImmOperands(Inst, 1);
      } else {
        llvm_unreachable("unhandled operand type");
      }
    }
  } else {
    // No src modifiers
    for (unsigned E = Operands.size(); I != E; ++I) {
      PPUOperand &Op = ((PPUOperand &)*Operands[I]);
      if (Op.isMod()) {
        OptionalIdx[Op.getImmTy()] = I;
      } else {
        Op.addRegOrImmOperands(Inst, 1);
      }
    }
  }

  if (PPU::getNamedOperandIdx(Opc, PPU::OpName::clamp) != -1) {
    addOptionalImmOperand(Inst, Operands, OptionalIdx, PPUOperand::ImmTyClampSI);
  }

  if (PPU::getNamedOperandIdx(Opc, PPU::OpName::omod) != -1) {
    addOptionalImmOperand(Inst, Operands, OptionalIdx, PPUOperand::ImmTyOModSI);
  }

  // Special case v_mac_{f16, f32} and v_fmac_{f16, f32} (gfx906/gfx10+):
  // it has src2 register operand that is tied to dst operand
  // we don't allow modifiers for this operand in assembler so src2_modifiers
  // should be 0.
  /*
  if (Opc == PPU::V_MAC_F32_e64_ppu || Opc == PPU::V_MAC_F16_e64_ppu ||
      Opc == PPU::V_FMAC_F32_e64_ppu ||
      Opc == PPU::V_FMAC_F16_e64_ppu) {
    auto it = Inst.begin();
    std::advance(it, PPU::getNamedOperandIdx(Opc, PPU::OpName::src2_modifiers));
    it = Inst.insert(it, MCOperand::createImm(0)); // no modifiers for src2
    ++it;
    Inst.insert(it, Inst.getOperand(0)); // src2 = dst
  }
      */
}

void PPUAsmParser::cvtVOP3(MCInst &Inst, const OperandVector &Operands) {
  OptionalImmIndexMap OptionalIdx;
  cvtVOP3(Inst, Operands, OptionalIdx);
}

void PPUAsmParser::cvtVOP3P(MCInst &Inst,
                               const OperandVector &Operands) {
  OptionalImmIndexMap OptIdx;
  const int Opc = Inst.getOpcode();
  const MCInstrDesc &Desc = MII.get(Opc);

  const bool IsPacked = (Desc.TSFlags & PPUInstrFlags::IsPacked) != 0;

  cvtVOP3(Inst, Operands, OptIdx);

  if (PPU::getNamedOperandIdx(Opc, PPU::OpName::vdst_in) != -1) {
    assert(!IsPacked);
    Inst.addOperand(Inst.getOperand(0));
  }

  // FIXME: This is messy. Parse the modifiers as if it was a normal VOP3
  // instruction, and then figure out where to actually put the modifiers

  addOptionalImmOperand(Inst, Operands, OptIdx, PPUOperand::ImmTyOpSel);

  int OpSelHiIdx = PPU::getNamedOperandIdx(Opc, PPU::OpName::op_sel_hi);
  if (OpSelHiIdx != -1) {
    int DefaultVal = IsPacked ? -1 : 0;
    addOptionalImmOperand(Inst, Operands, OptIdx, PPUOperand::ImmTyOpSelHi,
                          DefaultVal);
  }

  int NegLoIdx = PPU::getNamedOperandIdx(Opc, PPU::OpName::neg_lo);
  if (NegLoIdx != -1) {
    assert(IsPacked);
    addOptionalImmOperand(Inst, Operands, OptIdx, PPUOperand::ImmTyNegLo);
    addOptionalImmOperand(Inst, Operands, OptIdx, PPUOperand::ImmTyNegHi);
  }

  const int Ops[] = { PPU::OpName::src0,
                      PPU::OpName::src1,
                      PPU::OpName::src2 };
  const int ModOps[] = { PPU::OpName::src0_modifiers,
                         PPU::OpName::src1_modifiers,
                         PPU::OpName::src2_modifiers };

  int OpSelIdx = PPU::getNamedOperandIdx(Opc, PPU::OpName::op_sel);

  unsigned OpSel = Inst.getOperand(OpSelIdx).getImm();
  unsigned OpSelHi = 0;
  unsigned NegLo = 0;
  unsigned NegHi = 0;

  if (OpSelHiIdx != -1) {
    OpSelHi = Inst.getOperand(OpSelHiIdx).getImm();
  }

  if (NegLoIdx != -1) {
    int NegHiIdx = PPU::getNamedOperandIdx(Opc, PPU::OpName::neg_hi);
    NegLo = Inst.getOperand(NegLoIdx).getImm();
    NegHi = Inst.getOperand(NegHiIdx).getImm();
  }

  for (int J = 0; J < 3; ++J) {
    int OpIdx = PPU::getNamedOperandIdx(Opc, Ops[J]);
    if (OpIdx == -1)
      break;

    uint32_t ModVal = 0;

    if ((OpSel & (1 << J)) != 0)
      ModVal |= PPUSrcMods::OP_SEL_0;

    if ((OpSelHi & (1 << J)) != 0)
      ModVal |= PPUSrcMods::OP_SEL_1;

    if ((NegLo & (1 << J)) != 0)
      ModVal |= PPUSrcMods::NEG;

    if ((NegHi & (1 << J)) != 0)
      ModVal |= PPUSrcMods::NEG_HI;

    int ModIdx = PPU::getNamedOperandIdx(Opc, ModOps[J]);

    Inst.getOperand(ModIdx).setImm(Inst.getOperand(ModIdx).getImm() | ModVal);
  }
}

//===----------------------------------------------------------------------===//
// dpp
//===----------------------------------------------------------------------===//
/*
bool PPUOperand::isDPP8() const {
  return isImmTy(ImmTyDPP8);
}

bool PPUOperand::isDPPCtrl() const {
  using namespace PPU::DPP;

  bool result = isImm() && getImmTy() == ImmTyDppCtrl && isUInt<9>(getImm());
  if (result) {
    int64_t Imm = getImm();
    return (Imm >= DppCtrl::QUAD_PERM_FIRST && Imm <= DppCtrl::QUAD_PERM_LAST) ||
           (Imm >= DppCtrl::ROW_SHL_FIRST && Imm <= DppCtrl::ROW_SHL_LAST) ||
           (Imm >= DppCtrl::ROW_SHR_FIRST && Imm <= DppCtrl::ROW_SHR_LAST) ||
           (Imm >= DppCtrl::ROW_ROR_FIRST && Imm <= DppCtrl::ROW_ROR_LAST) ||
           (Imm == DppCtrl::WAVE_SHL1) ||
           (Imm == DppCtrl::WAVE_ROL1) ||
           (Imm == DppCtrl::WAVE_SHR1) ||
           (Imm == DppCtrl::WAVE_ROR1) ||
           (Imm == DppCtrl::ROW_MIRROR) ||
           (Imm == DppCtrl::ROW_HALF_MIRROR) ||
           (Imm == DppCtrl::BCAST15) ||
           (Imm == DppCtrl::BCAST31) ||
           (Imm >= DppCtrl::ROW_SHARE_FIRST && Imm <= DppCtrl::ROW_SHARE_LAST) ||
           (Imm >= DppCtrl::ROW_XMASK_FIRST && Imm <= DppCtrl::ROW_XMASK_LAST);
  }
  return false;
}
*/
//===----------------------------------------------------------------------===//
// mAI
//===----------------------------------------------------------------------===//

bool PPUOperand::isBLGP() const {
  return isImm() && getImmTy() == ImmTyBLGP && isUInt<3>(getImmInt());
}

bool PPUOperand::isCBSZ() const {
  return isImm() && getImmTy() == ImmTyCBSZ && isUInt<3>(getImmInt());
}

bool PPUOperand::isABID() const {
  return isImm() && getImmTy() == ImmTyABID && isUInt<4>(getImmInt());
}

bool PPUOperand::isS16Imm() const {
  return isImm() && (isInt<16>(getImmInt()) || isUInt<16>(getImmInt()));
}

bool PPUOperand::isU16Imm() const {
  return isImm() && isUInt<16>(getImmInt());
}
/*
OperandMatchResultTy PPUAsmParser::parseDim(OperandVector &Operands) {
  if (!isGFX10())
    return MatchOperand_NoMatch;

  SMLoc S = Parser.getTok().getLoc();

  if (getLexer().isNot(AsmToken::Identifier))
    return MatchOperand_NoMatch;
  if (getLexer().getTok().getString() != "dim")
    return MatchOperand_NoMatch;

  Parser.Lex();
  if (getLexer().isNot(AsmToken::Colon))
    return MatchOperand_ParseFail;

  Parser.Lex();

  // We want to allow "dim:1D" etc., but the initial 1 is tokenized as an
  // integer.
  std::string Token;
  if (getLexer().is(AsmToken::Integer)) {
    SMLoc Loc = getLexer().getTok().getEndLoc();
    Token = getLexer().getTok().getString();
    Parser.Lex();
    if (getLexer().getTok().getLoc() != Loc)
      return MatchOperand_ParseFail;
  }
  if (getLexer().isNot(AsmToken::Identifier))
    return MatchOperand_ParseFail;
  Token += getLexer().getTok().getString();

  StringRef DimId = Token;
  if (DimId.startswith("SQ_RSRC_IMG_"))
    DimId = DimId.substr(12);

  const PPU::MIMGDimInfo *DimInfo = PPU::getMIMGDimInfoByAsmSuffix(DimId);
  if (!DimInfo)
    return MatchOperand_ParseFail;

  Parser.Lex();

  Operands.push_back(PPUOperand::CreateImm(this, DimInfo->Encoding, S,
                                              PPUOperand::ImmTyDim));
  return MatchOperand_Success;
}
*/
/*
OperandMatchResultTy PPUAsmParser::parseDPP8(OperandVector &Operands) {
  SMLoc S = Parser.getTok().getLoc();
  StringRef Prefix;

  if (getLexer().getKind() == AsmToken::Identifier) {
    Prefix = Parser.getTok().getString();
  } else {
    return MatchOperand_NoMatch;
  }

  if (Prefix != "dpp8")
    return parseDPPCtrl(Operands);
  if (!isGFX10())
    return MatchOperand_NoMatch;

  // dpp8:[%d,%d,%d,%d,%d,%d,%d,%d]

  int64_t Sels[8];

  Parser.Lex();
  if (getLexer().isNot(AsmToken::Colon))
    return MatchOperand_ParseFail;

  Parser.Lex();
  if (getLexer().isNot(AsmToken::LBrac))
    return MatchOperand_ParseFail;

  Parser.Lex();
  if (getParser().parseAbsoluteExpression(Sels[0]))
    return MatchOperand_ParseFail;
  if (0 > Sels[0] || 7 < Sels[0])
    return MatchOperand_ParseFail;

  for (size_t i = 1; i < 8; ++i) {
    if (getLexer().isNot(AsmToken::Comma))
      return MatchOperand_ParseFail;

    Parser.Lex();
    if (getParser().parseAbsoluteExpression(Sels[i]))
      return MatchOperand_ParseFail;
    if (0 > Sels[i] || 7 < Sels[i])
      return MatchOperand_ParseFail;
  }

  if (getLexer().isNot(AsmToken::RBrac))
    return MatchOperand_ParseFail;
  Parser.Lex();

  unsigned DPP8 = 0;
  for (size_t i = 0; i < 8; ++i)
    DPP8 |= (Sels[i] << (i * 3));

  Operands.push_back(PPUOperand::CreateImm(this, DPP8, S, PPUOperand::ImmTyDPP8));
  return MatchOperand_Success;
}

OperandMatchResultTy
PPUAsmParser::parseDPPCtrl(OperandVector &Operands) {
  using namespace PPU::DPP;

  SMLoc S = Parser.getTok().getLoc();
  StringRef Prefix;
  int64_t Int;

  if (getLexer().getKind() == AsmToken::Identifier) {
    Prefix = Parser.getTok().getString();
  } else {
    return MatchOperand_NoMatch;
  }

  if (Prefix == "row_mirror") {
    Int = DppCtrl::ROW_MIRROR;
    Parser.Lex();
  } else if (Prefix == "row_half_mirror") {
    Int = DppCtrl::ROW_HALF_MIRROR;
    Parser.Lex();
  } else {
    // Check to prevent parseDPPCtrlOps from eating invalid tokens
    if (Prefix != "quad_perm"
        && Prefix != "row_shl"
        && Prefix != "row_shr"
        && Prefix != "row_ror"
        && Prefix != "wave_shl"
        && Prefix != "wave_rol"
        && Prefix != "wave_shr"
        && Prefix != "wave_ror"
        && Prefix != "row_bcast"
        && Prefix != "row_share"
        && Prefix != "row_xmask") {
      return MatchOperand_NoMatch;
    }

    if (!isGFX10() && (Prefix == "row_share" || Prefix == "row_xmask"))
      return MatchOperand_NoMatch;

    if (!isVI() && !isGFX9() &&
        (Prefix == "wave_shl" || Prefix == "wave_shr" ||
         Prefix == "wave_rol" || Prefix == "wave_ror" ||
         Prefix == "row_bcast"))
      return MatchOperand_NoMatch;

    Parser.Lex();
    if (getLexer().isNot(AsmToken::Colon))
      return MatchOperand_ParseFail;

    if (Prefix == "quad_perm") {
      // quad_perm:[%d,%d,%d,%d]
      Parser.Lex();
      if (getLexer().isNot(AsmToken::LBrac))
        return MatchOperand_ParseFail;
      Parser.Lex();

      if (getParser().parseAbsoluteExpression(Int) || !(0 <= Int && Int <=3))
        return MatchOperand_ParseFail;

      for (int i = 0; i < 3; ++i) {
        if (getLexer().isNot(AsmToken::Comma))
          return MatchOperand_ParseFail;
        Parser.Lex();

        int64_t Temp;
        if (getParser().parseAbsoluteExpression(Temp) || !(0 <= Temp && Temp <=3))
          return MatchOperand_ParseFail;
        const int shift = i*2 + 2;
        Int += (Temp << shift);
      }

      if (getLexer().isNot(AsmToken::RBrac))
        return MatchOperand_ParseFail;
      Parser.Lex();
    } else {
      // sel:%d
      Parser.Lex();
      if (getParser().parseAbsoluteExpression(Int))
        return MatchOperand_ParseFail;

      if (Prefix == "row_shl" && 1 <= Int && Int <= 15) {
        Int |= DppCtrl::ROW_SHL0;
      } else if (Prefix == "row_shr" && 1 <= Int && Int <= 15) {
        Int |= DppCtrl::ROW_SHR0;
      } else if (Prefix == "row_ror" && 1 <= Int && Int <= 15) {
        Int |= DppCtrl::ROW_ROR0;
      } else if (Prefix == "wave_shl" && 1 == Int) {
        Int = DppCtrl::WAVE_SHL1;
      } else if (Prefix == "wave_rol" && 1 == Int) {
        Int = DppCtrl::WAVE_ROL1;
      } else if (Prefix == "wave_shr" && 1 == Int) {
        Int = DppCtrl::WAVE_SHR1;
      } else if (Prefix == "wave_ror" && 1 == Int) {
        Int = DppCtrl::WAVE_ROR1;
      } else if (Prefix == "row_bcast") {
        if (Int == 15) {
          Int = DppCtrl::BCAST15;
        } else if (Int == 31) {
          Int = DppCtrl::BCAST31;
        } else {
          return MatchOperand_ParseFail;
        }
      } else if (Prefix == "row_share" && 0 <= Int && Int <= 15) {
        Int |= DppCtrl::ROW_SHARE_FIRST;
      } else if (Prefix == "row_xmask" && 0 <= Int && Int <= 15) {
        Int |= DppCtrl::ROW_XMASK_FIRST;
      } else {
        return MatchOperand_ParseFail;
      }
    }
  }

  Operands.push_back(PPUOperand::CreateImm(this, Int, S, PPUOperand::ImmTyDppCtrl));
  return MatchOperand_Success;
}

PPUOperand::Ptr PPUAsmParser::defaultRowMask() const {
  return PPUOperand::CreateImm(this, 0xf, SMLoc(), PPUOperand::ImmTyDppRowMask);
}
*/

PPUOperand::Ptr PPUAsmParser::defaultEndpgmImmOperands() const {
  return PPUOperand::CreateImm(this, 0, SMLoc(), PPUOperand::ImmTyEndpgm);
}
/*
PPUOperand::Ptr PPUAsmParser::defaultBankMask() const {
  return PPUOperand::CreateImm(this, 0xf, SMLoc(), PPUOperand::ImmTyDppBankMask);
}

PPUOperand::Ptr PPUAsmParser::defaultBoundCtrl() const {
  return PPUOperand::CreateImm(this, 0, SMLoc(), PPUOperand::ImmTyDppBoundCtrl);
}

PPUOperand::Ptr PPUAsmParser::defaultFI() const {
  return PPUOperand::CreateImm(this, 0, SMLoc(), PPUOperand::ImmTyDppFi);
}

void PPUAsmParser::cvtDPP(MCInst &Inst, const OperandVector &Operands, bool IsDPP8) {
  OptionalImmIndexMap OptionalIdx;

  unsigned I = 1;
  const MCInstrDesc &Desc = MII.get(Inst.getOpcode());
  for (unsigned J = 0; J < Desc.getNumDefs(); ++J) {
    ((PPUOperand &)*Operands[I++]).addRegOperands(Inst, 1);
  }

  int Fi = 0;
  for (unsigned E = Operands.size(); I != E; ++I) {
    auto TiedTo = Desc.getOperandConstraint(Inst.getNumOperands(),
                                            MCOI::TIED_TO);
    if (TiedTo != -1) {
      assert((unsigned)TiedTo < Inst.getNumOperands());
      // handle tied old or src2 for MAC instructions
      Inst.addOperand(Inst.getOperand(TiedTo));
    }
    PPUOperand &Op = ((PPUOperand &)*Operands[I]);
    // Add the register arguments
    if (Op.isReg() && validateVccOperand(Op.getReg())) {
      // VOP2b (v_add_u32, v_sub_u32 ...) dpp use "vcc" token.
      // Skip it.
      continue;
    }

    if (IsDPP8) {
      if (Op.isDPP8()) {
        Op.addImmOperands(Inst, 1);
      } else if (isRegOrImmWithInputMods(Desc, Inst.getNumOperands())) {
        Op.addRegWithFPInputModsOperands(Inst, 2);
      } else if (Op.isFI()) {
        Fi = Op.getImm();
      } else if (Op.isReg()) {
        Op.addRegOperands(Inst, 1);
      } else {
        llvm_unreachable("Invalid operand type");
      }
    } else {
      if (isRegOrImmWithInputMods(Desc, Inst.getNumOperands())) {
        Op.addRegWithFPInputModsOperands(Inst, 2);
      } else if (Op.isDPPCtrl()) {
        Op.addImmOperands(Inst, 1);
      } else if (Op.isImm()) {
        // Handle optional arguments
        OptionalIdx[Op.getImmTy()] = I;
      } else {
        llvm_unreachable("Invalid operand type");
      }
    }
  }

  if (IsDPP8) {
    using namespace llvm::PPU::DPP;
    Inst.addOperand(MCOperand::createImm(Fi? DPP8_FI_1 : DPP8_FI_0));
  } else {
    addOptionalImmOperand(Inst, Operands, OptionalIdx, PPUOperand::ImmTyDppRowMask, 0xf);
    addOptionalImmOperand(Inst, Operands, OptionalIdx, PPUOperand::ImmTyDppBankMask, 0xf);
    addOptionalImmOperand(Inst, Operands, OptionalIdx, PPUOperand::ImmTyDppBoundCtrl);
    if (PPU::getNamedOperandIdx(Inst.getOpcode(), PPU::OpName::fi) != -1) {
      addOptionalImmOperand(Inst, Operands, OptionalIdx, PPUOperand::ImmTyDppFi);
    }
  }
}
*/

//===----------------------------------------------------------------------===//
// sdwa
//===----------------------------------------------------------------------===//
/*
OperandMatchResultTy
PPUAsmParser::parseSDWASel(OperandVector &Operands, StringRef Prefix,
                              PPUOperand::ImmTy Type) {
  using namespace llvm::PPU::SDWA;

  SMLoc S = Parser.getTok().getLoc();
  StringRef Value;
  OperandMatchResultTy res;

  res = parseStringWithPrefix(Prefix, Value);
  if (res != MatchOperand_Success) {
    return res;
  }

  int64_t Int;
  Int = StringSwitch<int64_t>(Value)
        .Case("BYTE_0", SdwaSel::BYTE_0)
        .Case("BYTE_1", SdwaSel::BYTE_1)
        .Case("BYTE_2", SdwaSel::BYTE_2)
        .Case("BYTE_3", SdwaSel::BYTE_3)
        .Case("WORD_0", SdwaSel::WORD_0)
        .Case("WORD_1", SdwaSel::WORD_1)
        .Case("DWORD", SdwaSel::DWORD)
        .Default(0xffffffff);
  Parser.Lex(); // eat last token

  if (Int == 0xffffffff) {
    return MatchOperand_ParseFail;
  }

  Operands.push_back(PPUOperand::CreateImm(this, Int, S, Type));
  return MatchOperand_Success;
}

OperandMatchResultTy
PPUAsmParser::parseSDWADstUnused(OperandVector &Operands) {
  using namespace llvm::PPU::SDWA;

  SMLoc S = Parser.getTok().getLoc();
  StringRef Value;
  OperandMatchResultTy res;

  res = parseStringWithPrefix("dst_unused", Value);
  if (res != MatchOperand_Success) {
    return res;
  }

  int64_t Int;
  Int = StringSwitch<int64_t>(Value)
        .Case("UNUSED_PAD", DstUnused::UNUSED_PAD)
        .Case("UNUSED_SEXT", DstUnused::UNUSED_SEXT)
        .Case("UNUSED_PRESERVE", DstUnused::UNUSED_PRESERVE)
        .Default(0xffffffff);
  Parser.Lex(); // eat last token

  if (Int == 0xffffffff) {
    return MatchOperand_ParseFail;
  }

  Operands.push_back(PPUOperand::CreateImm(this, Int, S, PPUOperand::ImmTySdwaDstUnused));
  return MatchOperand_Success;
}

void PPUAsmParser::cvtSdwaVOP1(MCInst &Inst, const OperandVector &Operands) {
  cvtSDWA(Inst, Operands, PPUInstrFlags::VOP1);
}

void PPUAsmParser::cvtSdwaVOP2(MCInst &Inst, const OperandVector &Operands) {
  cvtSDWA(Inst, Operands, PPUInstrFlags::VOP2);
}

void PPUAsmParser::cvtSdwaVOP2b(MCInst &Inst, const OperandVector &Operands) {
  cvtSDWA(Inst, Operands, PPUInstrFlags::VOP2, true);
}

void PPUAsmParser::cvtSdwaVOPC(MCInst &Inst, const OperandVector &Operands) {
  cvtSDWA(Inst, Operands, PPUInstrFlags::VOPC, isVI());
}

void PPUAsmParser::cvtSDWA(MCInst &Inst, const OperandVector &Operands,
                              uint64_t BasicInstType, bool skipVcc) {
  using namespace llvm::PPU::SDWA;

  OptionalImmIndexMap OptionalIdx;
  bool skippedVcc = false;

  unsigned I = 1;
  const MCInstrDesc &Desc = MII.get(Inst.getOpcode());
  for (unsigned J = 0; J < Desc.getNumDefs(); ++J) {
    ((PPUOperand &)*Operands[I++]).addRegOperands(Inst, 1);
  }

  for (unsigned E = Operands.size(); I != E; ++I) {
    PPUOperand &Op = ((PPUOperand &)*Operands[I]);
    if (skipVcc && !skippedVcc && Op.isReg() &&
        (Op.getReg() == PPU::VCC || Op.getReg() == PPU::VCC_LO)) {
      // VOP2b (v_add_u32, v_sub_u32 ...) sdwa use "vcc" token as dst.
      // Skip it if it's 2nd (e.g. v_add_i32_sdwa v1, vcc, v2, v3)
      // or 4th (v_addc_u32_sdwa v1, vcc, v2, v3, vcc) operand.
      // Skip VCC only if we didn't skip it on previous iteration.
      if (BasicInstType == PPUInstrFlags::VOP2 &&
          (Inst.getNumOperands() == 1 || Inst.getNumOperands() == 5)) {
        skippedVcc = true;
        continue;
      } else if (BasicInstType == PPUInstrFlags::VOPC &&
                 Inst.getNumOperands() == 0) {
        skippedVcc = true;
        continue;
      }
    }
    if (isRegOrImmWithInputMods(Desc, Inst.getNumOperands())) {
      Op.addRegOrImmWithInputModsOperands(Inst, 2);
    } else if (Op.isImm()) {
      // Handle optional arguments
      OptionalIdx[Op.getImmTy()] = I;
    } else {
      llvm_unreachable("Invalid operand type");
    }
    skippedVcc = false;
  }

  if (Inst.getOpcode() != PPU::V_NOP_sdwa_gfx10 &&
      Inst.getOpcode() != PPU::V_NOP_sdwa_gfx9 &&
      Inst.getOpcode() != PPU::V_NOP_sdwa_vi) {
    // v_nop_sdwa_sdwa_vi/gfx9 has no optional sdwa arguments
    switch (BasicInstType) {
    case PPUInstrFlags::VOP1:
      addOptionalImmOperand(Inst, Operands, OptionalIdx, PPUOperand::ImmTyClampSI, 0);
      if (PPU::getNamedOperandIdx(Inst.getOpcode(), PPU::OpName::omod) != -1) {
        addOptionalImmOperand(Inst, Operands, OptionalIdx, PPUOperand::ImmTyOModSI, 0);
      }
      addOptionalImmOperand(Inst, Operands, OptionalIdx, PPUOperand::ImmTySdwaDstSel, SdwaSel::DWORD);
      addOptionalImmOperand(Inst, Operands, OptionalIdx, PPUOperand::ImmTySdwaDstUnused, DstUnused::UNUSED_PRESERVE);
      addOptionalImmOperand(Inst, Operands, OptionalIdx, PPUOperand::ImmTySdwaSrc0Sel, SdwaSel::DWORD);
      break;

    case PPUInstrFlags::VOP2:
      addOptionalImmOperand(Inst, Operands, OptionalIdx, PPUOperand::ImmTyClampSI, 0);
      if (PPU::getNamedOperandIdx(Inst.getOpcode(), PPU::OpName::omod) != -1) {
        addOptionalImmOperand(Inst, Operands, OptionalIdx, PPUOperand::ImmTyOModSI, 0);
      }
      addOptionalImmOperand(Inst, Operands, OptionalIdx, PPUOperand::ImmTySdwaDstSel, SdwaSel::DWORD);
      addOptionalImmOperand(Inst, Operands, OptionalIdx, PPUOperand::ImmTySdwaDstUnused, DstUnused::UNUSED_PRESERVE);
      addOptionalImmOperand(Inst, Operands, OptionalIdx, PPUOperand::ImmTySdwaSrc0Sel, SdwaSel::DWORD);
      addOptionalImmOperand(Inst, Operands, OptionalIdx, PPUOperand::ImmTySdwaSrc1Sel, SdwaSel::DWORD);
      break;

    case PPUInstrFlags::VOPC:
      if (PPU::getNamedOperandIdx(Inst.getOpcode(), PPU::OpName::clamp) != -1)
        addOptionalImmOperand(Inst, Operands, OptionalIdx, PPUOperand::ImmTyClampSI, 0);
      addOptionalImmOperand(Inst, Operands, OptionalIdx, PPUOperand::ImmTySdwaSrc0Sel, SdwaSel::DWORD);
      addOptionalImmOperand(Inst, Operands, OptionalIdx, PPUOperand::ImmTySdwaSrc1Sel, SdwaSel::DWORD);
      break;

    default:
      llvm_unreachable("Invalid instruction type. Only VOP1, VOP2 and VOPC allowed");
    }
  }

}
*/

//===----------------------------------------------------------------------===//
// mAI
//===----------------------------------------------------------------------===//

PPUOperand::Ptr PPUAsmParser::defaultBLGP() const {
  return PPUOperand::CreateImm(this, 0, SMLoc(), PPUOperand::ImmTyBLGP);
}

PPUOperand::Ptr PPUAsmParser::defaultCBSZ() const {
  return PPUOperand::CreateImm(this, 0, SMLoc(), PPUOperand::ImmTyCBSZ);
}

PPUOperand::Ptr PPUAsmParser::defaultABID() const {
  return PPUOperand::CreateImm(this, 0, SMLoc(), PPUOperand::ImmTyABID);
}

/*
extern "C" void LLVMInitializePPUAsmParser() {
  RegisterMCAsmParser<PPUAsmParser> A(getThePPUTarget());
  RegisterMCAsmParser<PPUAsmParser> B(getTheGCNTarget());
}
*/
extern "C" void LLVMInitializePPUAsmParser() {
  RegisterMCAsmParser<PPUAsmParser> X(getThePPUTarget());
}

#define GET_REGISTER_MATCHER
#define GET_MATCHER_IMPLEMENTATION
#define GET_MNEMONIC_SPELL_CHECKER
#include "PPUGenAsmMatcher.inc"

#if 0
// This fuction should be defined after auto-generated include so that we have
// MatchClassKind enum defined
unsigned PPUAsmParser::validateTargetOperandClass(MCParsedAsmOperand &Op,
                                                     unsigned Kind) {
  // Tokens like "glc" would be parsed as immediate operands in ParseOperand().
  // But MatchInstructionImpl() expects to meet token and fails to validate
  // operand. This method checks if we are given immediate operand but expect to
  // get corresponding token.
  PPUOperand &Operand = (PPUOperand&)Op;
  switch (Kind) {
  case MCK_addr64:
    return Operand.isAddr64() ? Match_Success : Match_InvalidOperand;
  case MCK_gds:
    return Operand.isGDS() ? Match_Success : Match_InvalidOperand;
  case MCK_lds:
    return Operand.isLDS() ? Match_Success : Match_InvalidOperand;
  case MCK_glc:
    return Operand.isGLC() ? Match_Success : Match_InvalidOperand;
  case MCK_idxen:
    return Operand.isIdxen() ? Match_Success : Match_InvalidOperand;
  case MCK_offen:
    return Operand.isOffen() ? Match_Success : Match_InvalidOperand;
  case MCK_SSrcB32:
    // When operands have expression values, they will return true for isToken,
    // because it is not possible to distinguish between a token and an
    // expression at parse time. MatchInstructionImpl() will always try to
    // match an operand as a token, when isToken returns true, and when the
    // name of the expression is not a valid token, the match will fail,
    // so we need to handle it here.
    return Operand.isSSrcB32() ? Match_Success : Match_InvalidOperand;
  case MCK_SSrcF32:
    return Operand.isSSrcF32() ? Match_Success : Match_InvalidOperand;
  case MCK_SoppBrTarget:
    return Operand.isSoppBrTarget() ? Match_Success : Match_InvalidOperand;
  case MCK_VReg32OrOff:
    return Operand.isVReg32OrOff() ? Match_Success : Match_InvalidOperand;
  case MCK_InterpSlot:
    return Operand.isInterpSlot() ? Match_Success : Match_InvalidOperand;
  case MCK_Attr:
    return Operand.isInterpAttr() ? Match_Success : Match_InvalidOperand;
  case MCK_AttrChan:
    return Operand.isAttrChan() ? Match_Success : Match_InvalidOperand;
  default:
    return Match_InvalidOperand;
  }
}
#endif

//===----------------------------------------------------------------------===//
// endpgm
//===----------------------------------------------------------------------===//

OperandMatchResultTy PPUAsmParser::parseEndpgmOp(OperandVector &Operands) {
  SMLoc S = Parser.getTok().getLoc();
  int64_t Imm = 0;

  if (!parseExpr(Imm)) {
    // The operand is optional, if not present default to 0
    Imm = 0;
  }

  if (!isUInt<16>(Imm)) {
    Error(S, "expected a 16-bit value");
    return MatchOperand_ParseFail;
  }

  Operands.push_back(
      PPUOperand::CreateImm(this, Imm, S, PPUOperand::ImmTyEndpgm));
  return MatchOperand_Success;
}

bool PPUOperand::isEndpgm() const { return isImmTy(ImmTyEndpgm); }



// below is RISCV
// 
// Return the matching FPR64 register for the given FPR32.
// FIXME: Ideally this function could be removed in favour of using
// information from TableGen.
static Register convertFPR32ToFPR64(Register Reg) {
  switch (Reg) {
  default:
    llvm_unreachable("Not a recognised FPR32 register");
  case PPU::F0_32: return PPU::F0_64;
  case PPU::F1_32: return PPU::F1_64;
  case PPU::F2_32: return PPU::F2_64;
  case PPU::F3_32: return PPU::F3_64;
  case PPU::F4_32: return PPU::F4_64;
  case PPU::F5_32: return PPU::F5_64;
  case PPU::F6_32: return PPU::F6_64;
  case PPU::F7_32: return PPU::F7_64;
  case PPU::F8_32: return PPU::F8_64;
  case PPU::F9_32: return PPU::F9_64;
  case PPU::F10_32: return PPU::F10_64;
  case PPU::F11_32: return PPU::F11_64;
  case PPU::F12_32: return PPU::F12_64;
  case PPU::F13_32: return PPU::F13_64;
  case PPU::F14_32: return PPU::F14_64;
  case PPU::F15_32: return PPU::F15_64;
  case PPU::F16_32: return PPU::F16_64;
  case PPU::F17_32: return PPU::F17_64;
  case PPU::F18_32: return PPU::F18_64;
  case PPU::F19_32: return PPU::F19_64;
  case PPU::F20_32: return PPU::F20_64;
  case PPU::F21_32: return PPU::F21_64;
  case PPU::F22_32: return PPU::F22_64;
  case PPU::F23_32: return PPU::F23_64;
  case PPU::F24_32: return PPU::F24_64;
  case PPU::F25_32: return PPU::F25_64;
  case PPU::F26_32: return PPU::F26_64;
  case PPU::F27_32: return PPU::F27_64;
  case PPU::F28_32: return PPU::F28_64;
  case PPU::F29_32: return PPU::F29_64;
  case PPU::F30_32: return PPU::F30_64;
  case PPU::F31_32: return PPU::F31_64;
  }
}

unsigned PPUAsmParser::validateTargetOperandClass(MCParsedAsmOperand &AsmOp,
                                                    unsigned Kind) {
  PPUOperand &Op = static_cast<PPUOperand &>(AsmOp);
  if (!Op.isReg())
    return Match_InvalidOperand;

  Register Reg = Op.getReg();
  bool IsRegFPR32 =
      PPUMCRegisterClasses[PPU::FPR32RegClassID].contains(Reg);
  bool IsRegFPR32C =
      PPUMCRegisterClasses[PPU::FPR32CRegClassID].contains(Reg);

  // As the parser couldn't differentiate an FPR32 from an FPR64, coerce the
  // register from FPR32 to FPR64 or FPR32C to FPR64C if necessary.
  if ((IsRegFPR32 && Kind == MCK_FPR64) ||
      (IsRegFPR32C && Kind == MCK_FPR64C)) {
    Op.Reg.RegNum = convertFPR32ToFPR64(Reg);
    return Match_Success;
  }

// TODO i add amd part here
  // Tokens like "glc" would be parsed as immediate operands in ParseOperand().
  // But MatchInstructionImpl() expects to meet token and fails to validate
  // operand. This method checks if we are given immediate operand but expect to
  // get corresponding token.
  PPUOperand &Operand = (PPUOperand&)Op;
  switch (Kind) {
  // case MCK_addr64:
  //   return Operand.isAddr64() ? Match_Success : Match_InvalidOperand;
  case MCK_gds:
    return Operand.isGDS() ? Match_Success : Match_InvalidOperand;
  case MCK_lds:
    return Operand.isLDS() ? Match_Success : Match_InvalidOperand;
  case MCK_glc:
    return Operand.isGLC() ? Match_Success : Match_InvalidOperand;
  case MCK_idxen:
    return Operand.isIdxen() ? Match_Success : Match_InvalidOperand;
  case MCK_offen:
    return Operand.isOffen() ? Match_Success : Match_InvalidOperand;
  case MCK_SSrcB32:
    // When operands have expression values, they will return true for isToken,
    // because it is not possible to distinguish between a token and an
    // expression at parse time. MatchInstructionImpl() will always try to
    // match an operand as a token, when isToken returns true, and when the
    // name of the expression is not a valid token, the match will fail,
    // so we need to handle it here.
    return Operand.isSSrcB32() ? Match_Success : Match_InvalidOperand;
  case MCK_SSrcF32:
    return Operand.isSSrcF32() ? Match_Success : Match_InvalidOperand;
  case MCK_SoppBrTarget:
    return Operand.isSoppBrTarget() ? Match_Success : Match_InvalidOperand;
  // case MCK_VReg32OrOff:
  //   return Operand.isVReg32OrOff() ? Match_Success : Match_InvalidOperand;
  // case MCK_InterpSlot:
  //   return Operand.isInterpSlot() ? Match_Success : Match_InvalidOperand;
  // case MCK_Attr:
  //   return Operand.isInterpAttr() ? Match_Success : Match_InvalidOperand;
  // case MCK_AttrChan:
  //   return Operand.isAttrChan() ? Match_Success : Match_InvalidOperand;
  default:
    return Match_InvalidOperand;
  }

  return Match_InvalidOperand;
}

bool PPUAsmParser::generateImmOutOfRangeError(
    OperandVector &Operands, uint64_t ErrorInfo, int64_t Lower, int64_t Upper,
    Twine Msg = "immediate must be an integer in the range") {
  SMLoc ErrorLoc = ((PPUOperand &)*Operands[ErrorInfo]).getStartLoc();
  return Error(ErrorLoc, Msg + " [" + Twine(Lower) + ", " + Twine(Upper) + "]");
}

bool PPUAsmParser::MatchAndEmitInstruction(SMLoc IDLoc, unsigned &Opcode,
                                             OperandVector &Operands,
                                             MCStreamer &Out,
                                             uint64_t &ErrorInfo,
                                             bool MatchingInlineAsm) {
  MCInst Inst;

  // TODO mixed with amd
  if (isPPT()) {
      return MatchAndEmitInstruction_ppt(IDLoc, Opcode, Operands, Out, ErrorInfo, MatchingInlineAsm);
  }
  auto Result =
    MatchInstructionImpl(Operands, Inst, ErrorInfo, MatchingInlineAsm);
  switch (Result) {
  default:
    break;
  case Match_Success:
    return processInstruction(Inst, IDLoc, Operands, Out);
  case Match_MissingFeature:
    return Error(IDLoc, "instruction use requires an option to be enabled");
  case Match_MnemonicFail:
    return Error(IDLoc, "unrecognized instruction mnemonic");
  case Match_InvalidOperand: {
    SMLoc ErrorLoc = IDLoc;
    if (ErrorInfo != ~0U) {
      if (ErrorInfo >= Operands.size())
        return Error(ErrorLoc, "too few operands for instruction");

      ErrorLoc = ((PPUOperand &)*Operands[ErrorInfo]).getStartLoc();
      if (ErrorLoc == SMLoc())
        ErrorLoc = IDLoc;
    }
    return Error(ErrorLoc, "invalid operand for instruction");
  }
  }

  // Handle the case when the error message is of specific type
  // other than the generic Match_InvalidOperand, and the
  // corresponding operand is missing.
  if (Result > FIRST_TARGET_MATCH_RESULT_TY) {
    SMLoc ErrorLoc = IDLoc;
    if (ErrorInfo != ~0U && ErrorInfo >= Operands.size())
        return Error(ErrorLoc, "too few operands for instruction");
  }

  switch(Result) {
  default:
    break;
  case Match_InvalidImmXLenLI:
    if (isRV64()) {
      SMLoc ErrorLoc = ((PPUOperand &)*Operands[ErrorInfo]).getStartLoc();
      return Error(ErrorLoc, "operand must be a constant 64-bit integer");
    }
    return generateImmOutOfRangeError(Operands, ErrorInfo,
                                      std::numeric_limits<int32_t>::min(),
                                      std::numeric_limits<uint32_t>::max());
  case Match_InvalidImmZero: {
    SMLoc ErrorLoc = ((PPUOperand &)*Operands[ErrorInfo]).getStartLoc();
    return Error(ErrorLoc, "immediate must be zero");
  }
  case Match_InvalidUImmLog2XLen:
    if (isRV64())
      return generateImmOutOfRangeError(Operands, ErrorInfo, 0, (1 << 6) - 1);
    return generateImmOutOfRangeError(Operands, ErrorInfo, 0, (1 << 5) - 1);
  case Match_InvalidUImmLog2XLenNonZero:
    if (isRV64())
      return generateImmOutOfRangeError(Operands, ErrorInfo, 1, (1 << 6) - 1);
    return generateImmOutOfRangeError(Operands, ErrorInfo, 1, (1 << 5) - 1);
  case Match_InvalidUImm5:
    return generateImmOutOfRangeError(Operands, ErrorInfo, 0, (1 << 5) - 1);
  case Match_InvalidSImm6:
    return generateImmOutOfRangeError(Operands, ErrorInfo, -(1 << 5),
                                      (1 << 5) - 1);
  case Match_InvalidSImm6NonZero:
    return generateImmOutOfRangeError(
        Operands, ErrorInfo, -(1 << 5), (1 << 5) - 1,
        "immediate must be non-zero in the range");
  case Match_InvalidCLUIImm:
    return generateImmOutOfRangeError(
        Operands, ErrorInfo, 1, (1 << 5) - 1,
        "immediate must be in [0xfffe0, 0xfffff] or");
  case Match_InvalidUImm7Lsb00:
    return generateImmOutOfRangeError(
        Operands, ErrorInfo, 0, (1 << 7) - 4,
        "immediate must be a multiple of 4 bytes in the range");
  case Match_InvalidUImm8Lsb00:
    return generateImmOutOfRangeError(
        Operands, ErrorInfo, 0, (1 << 8) - 4,
        "immediate must be a multiple of 4 bytes in the range");
  case Match_InvalidUImm8Lsb000:
    return generateImmOutOfRangeError(
        Operands, ErrorInfo, 0, (1 << 8) - 8,
        "immediate must be a multiple of 8 bytes in the range");
  case Match_InvalidSImm9Lsb0:
    return generateImmOutOfRangeError(
        Operands, ErrorInfo, -(1 << 8), (1 << 8) - 2,
        "immediate must be a multiple of 2 bytes in the range");
  case Match_InvalidUImm9Lsb000:
    return generateImmOutOfRangeError(
        Operands, ErrorInfo, 0, (1 << 9) - 8,
        "immediate must be a multiple of 8 bytes in the range");
  case Match_InvalidUImm10Lsb00NonZero:
    return generateImmOutOfRangeError(
        Operands, ErrorInfo, 4, (1 << 10) - 4,
        "immediate must be a multiple of 4 bytes in the range");
  case Match_InvalidSImm10Lsb0000NonZero:
    return generateImmOutOfRangeError(
        Operands, ErrorInfo, -(1 << 9), (1 << 9) - 16,
        "immediate must be a multiple of 16 bytes and non-zero in the range");
  case Match_InvalidSImm12:
    return generateImmOutOfRangeError(
        Operands, ErrorInfo, -(1 << 11), (1 << 11) - 1,
        "operand must be a symbol with %lo/%pcrel_lo/%tprel_lo modifier or an "
        "integer in the range");
  case Match_InvalidSImm12Lsb0:
    return generateImmOutOfRangeError(
        Operands, ErrorInfo, -(1 << 11), (1 << 11) - 2,
        "immediate must be a multiple of 2 bytes in the range");
  case Match_InvalidSImm13Lsb0:
    return generateImmOutOfRangeError(
        Operands, ErrorInfo, -(1 << 12), (1 << 12) - 2,
        "immediate must be a multiple of 2 bytes in the range");
  case Match_InvalidUImm20LUI:
    return generateImmOutOfRangeError(Operands, ErrorInfo, 0, (1 << 20) - 1,
                                      "operand must be a symbol with "
                                      "%hi/%tprel_hi modifier or an integer in "
                                      "the range");
  case Match_InvalidUImm20AUIPC:
    return generateImmOutOfRangeError(
        Operands, ErrorInfo, 0, (1 << 20) - 1,
        "operand must be a symbol with a "
        "%pcrel_hi/%got_pcrel_hi/%tls_ie_pcrel_hi/%tls_gd_pcrel_hi modifier or "
        "an integer in the range");
  case Match_InvalidSImm21Lsb0JAL:
    return generateImmOutOfRangeError(
        Operands, ErrorInfo, -(1 << 20), (1 << 20) - 2,
        "immediate must be a multiple of 2 bytes in the range");
  case Match_InvalidCSRSystemRegister: {
    return generateImmOutOfRangeError(Operands, ErrorInfo, 0, (1 << 12) - 1,
                                      "operand must be a valid system register "
                                      "name or an integer in the range");
  }
  case Match_InvalidFenceArg: {
    SMLoc ErrorLoc = ((PPUOperand &)*Operands[ErrorInfo]).getStartLoc();
    return Error(
        ErrorLoc,
        "operand must be formed of letters selected in-order from 'iorw'");
  }
  case Match_InvalidFRMArg: {
    SMLoc ErrorLoc = ((PPUOperand &)*Operands[ErrorInfo]).getStartLoc();
    return Error(
        ErrorLoc,
        "operand must be a valid floating point rounding mode mnemonic");
  }
  case Match_InvalidBareSymbol: {
    SMLoc ErrorLoc = ((PPUOperand &)*Operands[ErrorInfo]).getStartLoc();
    return Error(ErrorLoc, "operand must be a bare symbol name");
  }
  case Match_InvalidCallSymbol: {
    SMLoc ErrorLoc = ((PPUOperand &)*Operands[ErrorInfo]).getStartLoc();
    return Error(ErrorLoc, "operand must be a bare symbol name");
  }
  case Match_InvalidTPRelAddSymbol: {
    SMLoc ErrorLoc = ((PPUOperand &)*Operands[ErrorInfo]).getStartLoc();
    return Error(ErrorLoc, "operand must be a symbol with %tprel_add modifier");
  }
  }

  llvm_unreachable("Unknown match type detected!");
}

// Attempts to match Name as a register (either using the default name or
// alternative ABI names), setting RegNo to the matching register. Upon
// failure, returns true and sets RegNo to 0. If IsRV32E then registers
// x16-x31 will be rejected.
static bool matchRegisterNameHelper(bool IsRV32E, Register &RegNo,
                                    StringRef Name) {
  RegNo = MatchRegisterName(Name);
  if (RegNo == 0)
    RegNo = MatchRegisterAltName(Name);
  if (IsRV32E && RegNo >= PPU::X16 && RegNo <= PPU::X31)
    RegNo = 0;
  return RegNo == 0;
}

bool PPUAsmParser::ParseRegister(unsigned &RegNo, SMLoc &StartLoc,
                                   SMLoc &EndLoc) {

    /* FIXME here is ricvc, I mixed amd to here */
  auto R = parseRegister();
  if (R) {
    assert(R->isReg());
    RegNo = R->getReg();
    StartLoc = R->getStartLoc();
    EndLoc = R->getEndLoc();
    return true;
  }


  const AsmToken &Tok = getParser().getTok();
  StartLoc = Tok.getLoc();
  EndLoc = Tok.getEndLoc();
  RegNo = 0;
  StringRef Name = getLexer().getTok().getIdentifier();

  if (matchRegisterNameHelper(isRV32E(), (Register&)RegNo, Name))
    return Error(StartLoc, "invalid register name");

  getParser().Lex(); // Eat identifier token.
  return false;
}

OperandMatchResultTy PPUAsmParser::parseRegister(OperandVector &Operands,
                                                   bool AllowParens) {
  SMLoc FirstS = getLoc();
  bool HadParens = false;
  AsmToken LParen;

  // If this is an LParen and a parenthesised register name is allowed, parse it
  // atomically.
  if (AllowParens && getLexer().is(AsmToken::LParen)) {
    AsmToken Buf[2];
    size_t ReadCount = getLexer().peekTokens(Buf);
    if (ReadCount == 2 && Buf[1].getKind() == AsmToken::RParen) {
      HadParens = true;
      LParen = getParser().getTok();
      getParser().Lex(); // Eat '('
    }
  }

  switch (getLexer().getKind()) {
  default:
    if (HadParens)
      getLexer().UnLex(LParen);
    return MatchOperand_NoMatch;
  case AsmToken::Identifier:
    StringRef Name = getLexer().getTok().getIdentifier();
    Register RegNo;
    matchRegisterNameHelper(isRV32E(), RegNo, Name);

    if (RegNo == 0) {
      if (HadParens)
        getLexer().UnLex(LParen);
      return MatchOperand_NoMatch;
    }
    if (HadParens)
      Operands.push_back(PPUOperand::createToken("(", FirstS, isRV64()));
    SMLoc S = getLoc();
    SMLoc E = SMLoc::getFromPointer(S.getPointer() - 1);
    getLexer().Lex();
    Operands.push_back(PPUOperand::createReg(RegNo, S, E, isRV64()));
  }

  if (HadParens) {
    getParser().Lex(); // Eat ')'
    Operands.push_back(PPUOperand::createToken(")", getLoc(), isRV64()));
  }

  return MatchOperand_Success;
}

OperandMatchResultTy
PPUAsmParser::parseCSRSystemRegister(OperandVector &Operands) {
  SMLoc S = getLoc();
  const MCExpr *Res;

  switch (getLexer().getKind()) {
  default:
    return MatchOperand_NoMatch;
  case AsmToken::LParen:
  case AsmToken::Minus:
  case AsmToken::Plus:
  case AsmToken::Exclaim:
  case AsmToken::Tilde:
  case AsmToken::Integer:
  case AsmToken::String: {
    if (getParser().parseExpression(Res))
      return MatchOperand_ParseFail;

    auto *CE = dyn_cast<MCConstantExpr>(Res);
    if (CE) {
      int64_t Imm = CE->getValue();
      if (isUInt<12>(Imm)) {
        auto SysReg = PPUSysReg::lookupSysRegByEncoding(Imm);
        // Accept an immediate representing a named or un-named Sys Reg
        // if the range is valid, regardless of the required features.
        Operands.push_back(PPUOperand::createSysReg(
            SysReg ? SysReg->Name : "", S, Imm, isRV64()));
        return MatchOperand_Success;
      }
    }

    Twine Msg = "immediate must be an integer in the range";
    Error(S, Msg + " [" + Twine(0) + ", " + Twine((1 << 12) - 1) + "]");
    return MatchOperand_ParseFail;
  }
  case AsmToken::Identifier: {
    StringRef Identifier;
    if (getParser().parseIdentifier(Identifier))
      return MatchOperand_ParseFail;

    auto SysReg = PPUSysReg::lookupSysRegByName(Identifier);
    // Accept a named Sys Reg if the required features are present.
    if (SysReg) {
      if (!SysReg->haveRequiredFeatures(getSTI().getFeatureBits())) {
        Error(S, "system register use requires an option to be enabled");
        return MatchOperand_ParseFail;
      }
      Operands.push_back(PPUOperand::createSysReg(
          Identifier, S, SysReg->Encoding, isRV64()));
      return MatchOperand_Success;
    }

    Twine Msg = "operand must be a valid system register name "
                "or an integer in the range";
    Error(S, Msg + " [" + Twine(0) + ", " + Twine((1 << 12) - 1) + "]");
    return MatchOperand_ParseFail;
  }
  case AsmToken::Percent: {
    // Discard operand with modifier.
    Twine Msg = "immediate must be an integer in the range";
    Error(S, Msg + " [" + Twine(0) + ", " + Twine((1 << 12) - 1) + "]");
    return MatchOperand_ParseFail;
  }
  }

  return MatchOperand_NoMatch;
}

OperandMatchResultTy PPUAsmParser::parseImmediate(OperandVector &Operands) {
  SMLoc S = getLoc();
  SMLoc E = SMLoc::getFromPointer(S.getPointer() - 1);
  const MCExpr *Res;

  switch (getLexer().getKind()) {
  default:
    return MatchOperand_NoMatch;
  case AsmToken::LParen:
  case AsmToken::Dot:
  case AsmToken::Minus:
  case AsmToken::Plus:
  case AsmToken::Exclaim:
  case AsmToken::Tilde:
  case AsmToken::Integer:
  case AsmToken::String:
  case AsmToken::Identifier:
    if (getParser().parseExpression(Res))
      return MatchOperand_ParseFail;
    break;
  case AsmToken::Percent:
    return parseOperandWithModifier(Operands);
  }

  Operands.push_back(PPUOperand::createImm(Res, S, E, isRV64()));
  return MatchOperand_Success;
}

OperandMatchResultTy
PPUAsmParser::parseOperandWithModifier(OperandVector &Operands) {
  SMLoc S = getLoc();
  SMLoc E = SMLoc::getFromPointer(S.getPointer() - 1);

  if (getLexer().getKind() != AsmToken::Percent) {
    Error(getLoc(), "expected '%' for operand modifier");
    return MatchOperand_ParseFail;
  }

  getParser().Lex(); // Eat '%'

  if (getLexer().getKind() != AsmToken::Identifier) {
    Error(getLoc(), "expected valid identifier for operand modifier");
    return MatchOperand_ParseFail;
  }
  StringRef Identifier = getParser().getTok().getIdentifier();
  PPUMCExpr::VariantKind VK = PPUMCExpr::getVariantKindForName(Identifier);
  if (VK == PPUMCExpr::VK_PPU_Invalid) {
    Error(getLoc(), "unrecognized operand modifier");
    return MatchOperand_ParseFail;
  }

  getParser().Lex(); // Eat the identifier
  if (getLexer().getKind() != AsmToken::LParen) {
    Error(getLoc(), "expected '('");
    return MatchOperand_ParseFail;
  }
  getParser().Lex(); // Eat '('

  const MCExpr *SubExpr;
  if (getParser().parseParenExpression(SubExpr, E)) {
    return MatchOperand_ParseFail;
  }

  const MCExpr *ModExpr = PPUMCExpr::create(SubExpr, VK, getContext());
  Operands.push_back(PPUOperand::createImm(ModExpr, S, E, isRV64()));
  return MatchOperand_Success;
}

OperandMatchResultTy PPUAsmParser::parseBareSymbol(OperandVector &Operands) {
  SMLoc S = getLoc();
  SMLoc E = SMLoc::getFromPointer(S.getPointer() - 1);
  const MCExpr *Res;

  if (getLexer().getKind() != AsmToken::Identifier)
    return MatchOperand_NoMatch;

  StringRef Identifier;
  AsmToken Tok = getLexer().getTok();

  if (getParser().parseIdentifier(Identifier))
    return MatchOperand_ParseFail;

  if (Identifier.consume_back("@plt")) {
    Error(getLoc(), "'@plt' operand not valid for instruction");
    return MatchOperand_ParseFail;
  }

  MCSymbol *Sym = getContext().getOrCreateSymbol(Identifier);

  if (Sym->isVariable()) {
    const MCExpr *V = Sym->getVariableValue(/*SetUsed=*/false);
    if (!isa<MCSymbolRefExpr>(V)) {
      getLexer().UnLex(Tok); // Put back if it's not a bare symbol.
      return MatchOperand_NoMatch;
    }
    Res = V;
  } else
    Res = MCSymbolRefExpr::create(Sym, MCSymbolRefExpr::VK_None, getContext());

  MCBinaryExpr::Opcode Opcode;
  switch (getLexer().getKind()) {
  default:
    Operands.push_back(PPUOperand::createImm(Res, S, E, isRV64()));
    return MatchOperand_Success;
  case AsmToken::Plus:
    Opcode = MCBinaryExpr::Add;
    break;
  case AsmToken::Minus:
    Opcode = MCBinaryExpr::Sub;
    break;
  }

  const MCExpr *Expr;
  if (getParser().parseExpression(Expr))
    return MatchOperand_ParseFail;
  Res = MCBinaryExpr::create(Opcode, Res, Expr, getContext());
  Operands.push_back(PPUOperand::createImm(Res, S, E, isRV64()));
  return MatchOperand_Success;
}

OperandMatchResultTy PPUAsmParser::parseCallSymbol(OperandVector &Operands) {
  SMLoc S = getLoc();
  SMLoc E = SMLoc::getFromPointer(S.getPointer() - 1);
  const MCExpr *Res;

  if (getLexer().getKind() != AsmToken::Identifier)
    return MatchOperand_NoMatch;

  // Avoid parsing the register in `call rd, foo` as a call symbol.
  if (getLexer().peekTok().getKind() != AsmToken::EndOfStatement)
    return MatchOperand_NoMatch;

  StringRef Identifier;
  if (getParser().parseIdentifier(Identifier))
    return MatchOperand_ParseFail;

  PPUMCExpr::VariantKind Kind = PPUMCExpr::VK_PPU_CALL;
  if (Identifier.consume_back("@plt"))
    Kind = PPUMCExpr::VK_PPU_CALL_PLT;

  MCSymbol *Sym = getContext().getOrCreateSymbol(Identifier);
  Res = MCSymbolRefExpr::create(Sym, MCSymbolRefExpr::VK_None, getContext());
  Res = PPUMCExpr::create(Res, Kind, getContext());
  Operands.push_back(PPUOperand::createImm(Res, S, E, isRV64()));
  return MatchOperand_Success;
}

OperandMatchResultTy PPUAsmParser::parseJALOffset(OperandVector &Operands) {
  // Parsing jal operands is fiddly due to the `jal foo` and `jal ra, foo`
  // both being acceptable forms. When parsing `jal ra, foo` this function
  // will be called for the `ra` register operand in an attempt to match the
  // single-operand alias. parseJALOffset must fail for this case. It would
  // seem logical to try parse the operand using parseImmediate and return
  // NoMatch if the next token is a comma (meaning we must be parsing a jal in
  // the second form rather than the first). We can't do this as there's no
  // way of rewinding the lexer state. Instead, return NoMatch if this operand
  // is an identifier and is followed by a comma.
  if (getLexer().is(AsmToken::Identifier) &&
      getLexer().peekTok().is(AsmToken::Comma))
    return MatchOperand_NoMatch;

  return parseImmediate(Operands);
}

OperandMatchResultTy
PPUAsmParser::parseMemOpBaseReg(OperandVector &Operands) {
  if (getLexer().isNot(AsmToken::LParen)) {
    Error(getLoc(), "expected '('");
    return MatchOperand_ParseFail;
  }

  getParser().Lex(); // Eat '('
  Operands.push_back(PPUOperand::createToken("(", getLoc(), isRV64()));

  if (parseRegister(Operands) != MatchOperand_Success) {
    Error(getLoc(), "expected register");
    return MatchOperand_ParseFail;
  }

  if (getLexer().isNot(AsmToken::RParen)) {
    Error(getLoc(), "expected ')'");
    return MatchOperand_ParseFail;
  }

  getParser().Lex(); // Eat ')'
  Operands.push_back(PPUOperand::createToken(")", getLoc(), isRV64()));

  return MatchOperand_Success;
}

OperandMatchResultTy PPUAsmParser::parseAtomicMemOp(OperandVector &Operands) {
  // Atomic operations such as lr.w, sc.w, and amo*.w accept a "memory operand"
  // as one of their register operands, such as `(a0)`. This just denotes that
  // the register (in this case `a0`) contains a memory address.
  //
  // Normally, we would be able to parse these by putting the parens into the
  // instruction string. However, GNU as also accepts a zero-offset memory
  // operand (such as `0(a0)`), and ignores the 0. Normally this would be parsed
  // with parseImmediate followed by parseMemOpBaseReg, but these instructions
  // do not accept an immediate operand, and we do not want to add a "dummy"
  // operand that is silently dropped.
  //
  // Instead, we use this custom parser. This will: allow (and discard) an
  // offset if it is zero; require (and discard) parentheses; and add only the
  // parsed register operand to `Operands`.
  //
  // These operands are printed with PPUInstPrinter::printAtomicMemOp, which
  // will only print the register surrounded by parentheses (which GNU as also
  // uses as its canonical representation for these operands).
  std::unique_ptr<PPUOperand> OptionalImmOp;

  if (getLexer().isNot(AsmToken::LParen)) {
    // Parse an Integer token. We do not accept arbritrary constant expressions
    // in the offset field (because they may include parens, which complicates
    // parsing a lot).
    int64_t ImmVal;
    SMLoc ImmStart = getLoc();
    if (getParser().parseIntToken(ImmVal,
                                  "expected '(' or optional integer offset"))
      return MatchOperand_ParseFail;

    // Create a PPUOperand for checking later (so the error messages are
    // nicer), but we don't add it to Operands.
    SMLoc ImmEnd = getLoc();
    OptionalImmOp =
        PPUOperand::createImm(MCConstantExpr::create(ImmVal, getContext()),
                                ImmStart, ImmEnd, isRV64());
  }

  if (getLexer().isNot(AsmToken::LParen)) {
    Error(getLoc(), OptionalImmOp ? "expected '(' after optional integer offset"
                                  : "expected '(' or optional integer offset");
    return MatchOperand_ParseFail;
  }
  getParser().Lex(); // Eat '('

  if (parseRegister(Operands) != MatchOperand_Success) {
    Error(getLoc(), "expected register");
    return MatchOperand_ParseFail;
  }

  if (getLexer().isNot(AsmToken::RParen)) {
    Error(getLoc(), "expected ')'");
    return MatchOperand_ParseFail;
  }
  getParser().Lex(); // Eat ')'

  // Deferred Handling of non-zero offsets. This makes the error messages nicer.
  if (OptionalImmOp && !OptionalImmOp->isImmZero()) {
    Error(OptionalImmOp->getStartLoc(), "optional integer offset must be 0",
          SMRange(OptionalImmOp->getStartLoc(), OptionalImmOp->getEndLoc()));
    return MatchOperand_ParseFail;
  }

  return MatchOperand_Success;
}

/// Looks at a token type and creates the relevant operand from this
/// information, adding to Operands. If operand was parsed, returns false, else
/// true.
bool PPUAsmParser::parseOperand(OperandVector &Operands, StringRef Mnemonic) {
  if (isPPT()) {
    OperandMatchResultTy Result =
        parseOperand_ppt(Operands, Mnemonic);
    if (Result == MatchOperand_Success)
        return false;
    if (Result == MatchOperand_ParseFail)
        return true;
  }
  // Check if the current operand has a custom associated parser, if so, try to
  // custom parse the operand, or fallback to the general approach.
  OperandMatchResultTy Result =
      MatchOperandParserImpl(Operands, Mnemonic, /*ParseForAllFeatures=*/true);
  if (Result == MatchOperand_Success)
    return false;
  if (Result == MatchOperand_ParseFail)
    return true;

  // Attempt to parse token as a register.
  if (parseRegister(Operands, true) == MatchOperand_Success)
    return false;

  // Attempt to parse token as an immediate
  if (parseImmediate(Operands) == MatchOperand_Success) {
    // Parse memory base register if present
    if (getLexer().is(AsmToken::LParen))
      return parseMemOpBaseReg(Operands) != MatchOperand_Success;
    return false;
  }

  // Finally we have exhausted all options and must declare defeat.
  Error(getLoc(), "unknown operand");
  return true;
}

bool PPUAsmParser::ParseInstruction(ParseInstructionInfo &Info,
                                      StringRef Name, SMLoc NameLoc,
                                      OperandVector &Operands) {

  if (isPPT()) return ParseInstruction_ppt(Info, Name, NameLoc, Operands);

  // Ensure that if the instruction occurs when relaxation is enabled,
  // relocations are forced for the file. Ideally this would be done when there
  // is enough information to reliably determine if the instruction itself may
  // cause relaxations. Unfortunately instruction processing stage occurs in the
  // same pass as relocation emission, so it's too late to set a 'sticky bit'
  // for the entire file.
  if (getSTI().getFeatureBits()[PPU::FeatureRelax]) {
    auto *Assembler = getTargetStreamer().getStreamer().getAssemblerPtr();
    if (Assembler != nullptr) {
      PPUAsmBackend &MAB =
          static_cast<PPUAsmBackend &>(Assembler->getBackend());
      MAB.setForceRelocs();
    }
  }

  // First operand is token for instruction
  Operands.push_back(PPUOperand::createToken(Name, NameLoc, isRV64()));

  // If there are no more operands, then finish
  if (getLexer().is(AsmToken::EndOfStatement))
    return false;

  // Parse first operand
  if (parseOperand(Operands, Name))
    return true;

  // Parse until end of statement, consuming commas between operands
  unsigned OperandIdx = 1;
  while (getLexer().is(AsmToken::Comma)) {
    // Consume comma token
    getLexer().Lex();

    // Parse next operand
    if (parseOperand(Operands, Name))
      return true;

    ++OperandIdx;
  }

  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    SMLoc Loc = getLexer().getLoc();
    getParser().eatToEndOfStatement();
    return Error(Loc, "unexpected token");
  }

  getParser().Lex(); // Consume the EndOfStatement.
  return false;
}


bool PPUAsmParser::classifySymbolRef(const MCExpr *Expr,
                                       PPUMCExpr::VariantKind &Kind,
                                       int64_t &Addend) {
  Kind = PPUMCExpr::VK_PPU_None;
  Addend = 0;

  if (const PPUMCExpr *RE = dyn_cast<PPUMCExpr>(Expr)) {
    Kind = RE->getKind();
    Expr = RE->getSubExpr();
  }

  // It's a simple symbol reference or constant with no addend.
  if (isa<MCConstantExpr>(Expr) || isa<MCSymbolRefExpr>(Expr))
    return true;

  const MCBinaryExpr *BE = dyn_cast<MCBinaryExpr>(Expr);
  if (!BE)
    return false;

  if (!isa<MCSymbolRefExpr>(BE->getLHS()))
    return false;

  if (BE->getOpcode() != MCBinaryExpr::Add &&
      BE->getOpcode() != MCBinaryExpr::Sub)
    return false;

  // We are able to support the subtraction of two symbol references
  if (BE->getOpcode() == MCBinaryExpr::Sub &&
      isa<MCSymbolRefExpr>(BE->getRHS()))
    return true;

  // See if the addend is a constant, otherwise there's more going
  // on here than we can deal with.
  auto AddendExpr = dyn_cast<MCConstantExpr>(BE->getRHS());
  if (!AddendExpr)
    return false;

  Addend = AddendExpr->getValue();
  if (BE->getOpcode() == MCBinaryExpr::Sub)
    Addend = -Addend;

  // It's some symbol reference + a constant addend
  return Kind != PPUMCExpr::VK_PPU_Invalid;
}
/*
bool PPUAsmParser::ParseDirective(AsmToken DirectiveID) {
  // This returns false if this function recognizes the directive
  // regardless of whether it is successfully handles or reports an
  // error. Otherwise it returns true to give the generic parser a
  // chance at recognizing it.
  StringRef IDVal = DirectiveID.getString();

  if (IDVal == ".option")
    return parseDirectiveOption();

  return true;
}
*/

bool PPUAsmParser::parseDirectiveOption() {
  MCAsmParser &Parser = getParser();
  // Get the option token.
  AsmToken Tok = Parser.getTok();
  // At the moment only identifiers are supported.
  if (Tok.isNot(AsmToken::Identifier))
    return Error(Parser.getTok().getLoc(),
                 "unexpected token, expected identifier");

  StringRef Option = Tok.getIdentifier();

  if (Option == "push") {
    getTargetStreamer().emitDirectiveOptionPush();

    Parser.Lex();
    if (Parser.getTok().isNot(AsmToken::EndOfStatement))
      return Error(Parser.getTok().getLoc(),
                   "unexpected token, expected end of statement");

    pushFeatureBits();
    return false;
  }

  if (Option == "pop") {
    SMLoc StartLoc = Parser.getTok().getLoc();
    getTargetStreamer().emitDirectiveOptionPop();

    Parser.Lex();
    if (Parser.getTok().isNot(AsmToken::EndOfStatement))
      return Error(Parser.getTok().getLoc(),
                   "unexpected token, expected end of statement");

    if (popFeatureBits())
      return Error(StartLoc, ".option pop with no .option push");

    return false;
  }

  if (Option == "rvc") {
    getTargetStreamer().emitDirectiveOptionRVC();

    Parser.Lex();
    if (Parser.getTok().isNot(AsmToken::EndOfStatement))
      return Error(Parser.getTok().getLoc(),
                   "unexpected token, expected end of statement");

    setFeatureBits(PPU::FeatureStdExtC, "c");
    return false;
  }

  if (Option == "norvc") {
    getTargetStreamer().emitDirectiveOptionNoRVC();

    Parser.Lex();
    if (Parser.getTok().isNot(AsmToken::EndOfStatement))
      return Error(Parser.getTok().getLoc(),
                   "unexpected token, expected end of statement");

    clearFeatureBits(PPU::FeatureStdExtC, "c");
    return false;
  }

  if (Option == "relax") {
    getTargetStreamer().emitDirectiveOptionRelax();

    Parser.Lex();
    if (Parser.getTok().isNot(AsmToken::EndOfStatement))
      return Error(Parser.getTok().getLoc(),
                   "unexpected token, expected end of statement");

    setFeatureBits(PPU::FeatureRelax, "relax");
    return false;
  }

  if (Option == "norelax") {
    getTargetStreamer().emitDirectiveOptionNoRelax();

    Parser.Lex();
    if (Parser.getTok().isNot(AsmToken::EndOfStatement))
      return Error(Parser.getTok().getLoc(),
                   "unexpected token, expected end of statement");

    clearFeatureBits(PPU::FeatureRelax, "relax");
    return false;
  }

  // Unknown option.
  Warning(Parser.getTok().getLoc(),
          "unknown option, expected 'push', 'pop', 'rvc', 'norvc', 'relax' or "
          "'norelax'");
  Parser.eatToEndOfStatement();
  return false;
}

void PPUAsmParser::emitToStreamer(MCStreamer &S, const MCInst &Inst) {
  MCInst CInst;
  // bool Res = compressInst(CInst, Inst, getSTI(), S.getContext());
  // FIXME how to emit new compress way: extend 0 is skip
  bool Res = false;
  CInst.setLoc(Inst.getLoc());
  S.EmitInstruction((Res ? CInst : Inst), getSTI());
}

void PPUAsmParser::emitLoadImm(Register DestReg, int64_t Value,
                                 MCStreamer &Out) {
  PPUMatInt::InstSeq Seq;
  PPUMatInt::generateInstSeq(Value, isRV64(), Seq);

  Register SrcReg = PPU::X0;
  for (PPUMatInt::Inst &Inst : Seq) {
    if (Inst.Opc == PPU::LUI) {
      emitToStreamer(
          Out, MCInstBuilder(PPU::LUI).addReg(DestReg).addImm(Inst.Imm));
    } else {
      emitToStreamer(
          Out, MCInstBuilder(Inst.Opc).addReg(DestReg).addReg(SrcReg).addImm(
                   Inst.Imm));
    }

    // Only the first instruction has X0 as its source.
    SrcReg = DestReg;
  }
}

void PPUAsmParser::emitAuipcInstPair(MCOperand DestReg, MCOperand TmpReg,
                                       const MCExpr *Symbol,
                                       PPUMCExpr::VariantKind VKHi,
                                       unsigned SecondOpcode, SMLoc IDLoc,
                                       MCStreamer &Out) {
  // A pair of instructions for PC-relative addressing; expands to
  //   TmpLabel: AUIPC TmpReg, VKHi(symbol)
  //             OP DestReg, TmpReg, %pcrel_lo(TmpLabel)
  MCContext &Ctx = getContext();

  MCSymbol *TmpLabel = Ctx.createTempSymbol(
      "pcrel_hi", /* AlwaysAddSuffix */ true, /* CanBeUnnamed */ false);
  Out.EmitLabel(TmpLabel);

  const PPUMCExpr *SymbolHi = PPUMCExpr::create(Symbol, VKHi, Ctx);
  emitToStreamer(
      Out, MCInstBuilder(PPU::AUIPC).addOperand(TmpReg).addExpr(SymbolHi));

  const MCExpr *RefToLinkTmpLabel =
      PPUMCExpr::create(MCSymbolRefExpr::create(TmpLabel, Ctx),
                          PPUMCExpr::VK_PPU_PCREL_LO, Ctx);

  emitToStreamer(Out, MCInstBuilder(SecondOpcode)
                          .addOperand(DestReg)
                          .addOperand(TmpReg)
                          .addExpr(RefToLinkTmpLabel));
}

void PPUAsmParser::emitLoadLocalAddress(MCInst &Inst, SMLoc IDLoc,
                                          MCStreamer &Out) {
  // The load local address pseudo-instruction "lla" is used in PC-relative
  // addressing of local symbols:
  //   lla rdest, symbol
  // expands to
  //   TmpLabel: AUIPC rdest, %pcrel_hi(symbol)
  //             ADDI rdest, rdest, %pcrel_lo(TmpLabel)
  MCOperand DestReg = Inst.getOperand(0);
  const MCExpr *Symbol = Inst.getOperand(1).getExpr();
  emitAuipcInstPair(DestReg, DestReg, Symbol, PPUMCExpr::VK_PPU_PCREL_HI,
                    PPU::ADDI, IDLoc, Out);
}

void PPUAsmParser::emitLoadAddress(MCInst &Inst, SMLoc IDLoc,
                                     MCStreamer &Out) {
  // The load address pseudo-instruction "la" is used in PC-relative and
  // GOT-indirect addressing of global symbols:
  //   la rdest, symbol
  // expands to either (for non-PIC)
  //   TmpLabel: AUIPC rdest, %pcrel_hi(symbol)
  //             ADDI rdest, rdest, %pcrel_lo(TmpLabel)
  // or (for PIC)
  //   TmpLabel: AUIPC rdest, %got_pcrel_hi(symbol)
  //             Lx rdest, %pcrel_lo(TmpLabel)(rdest)
  MCOperand DestReg = Inst.getOperand(0);
  const MCExpr *Symbol = Inst.getOperand(1).getExpr();
  unsigned SecondOpcode;
  PPUMCExpr::VariantKind VKHi;
  // FIXME: Should check .option (no)pic when implemented
  if (getContext().getObjectFileInfo()->isPositionIndependent()) {
    SecondOpcode = isRV64() ? PPU::LD : PPU::LW;
    VKHi = PPUMCExpr::VK_PPU_GOT_HI;
  } else {
    SecondOpcode = PPU::ADDI;
    VKHi = PPUMCExpr::VK_PPU_PCREL_HI;
  }
  emitAuipcInstPair(DestReg, DestReg, Symbol, VKHi, SecondOpcode, IDLoc, Out);
}

void PPUAsmParser::emitLoadTLSIEAddress(MCInst &Inst, SMLoc IDLoc,
                                          MCStreamer &Out) {
  // The load TLS IE address pseudo-instruction "la.tls.ie" is used in
  // initial-exec TLS model addressing of global symbols:
  //   la.tls.ie rdest, symbol
  // expands to
  //   TmpLabel: AUIPC rdest, %tls_ie_pcrel_hi(symbol)
  //             Lx rdest, %pcrel_lo(TmpLabel)(rdest)
  MCOperand DestReg = Inst.getOperand(0);
  const MCExpr *Symbol = Inst.getOperand(1).getExpr();
  unsigned SecondOpcode = isRV64() ? PPU::LD : PPU::LW;
  emitAuipcInstPair(DestReg, DestReg, Symbol, PPUMCExpr::VK_PPU_TLS_GOT_HI,
                    SecondOpcode, IDLoc, Out);
}

void PPUAsmParser::emitLoadTLSGDAddress(MCInst &Inst, SMLoc IDLoc,
                                          MCStreamer &Out) {
  // The load TLS GD address pseudo-instruction "la.tls.gd" is used in
  // global-dynamic TLS model addressing of global symbols:
  //   la.tls.gd rdest, symbol
  // expands to
  //   TmpLabel: AUIPC rdest, %tls_gd_pcrel_hi(symbol)
  //             ADDI rdest, rdest, %pcrel_lo(TmpLabel)
  MCOperand DestReg = Inst.getOperand(0);
  const MCExpr *Symbol = Inst.getOperand(1).getExpr();
  emitAuipcInstPair(DestReg, DestReg, Symbol, PPUMCExpr::VK_PPU_TLS_GD_HI,
                    PPU::ADDI, IDLoc, Out);
}

void PPUAsmParser::emitLoadStoreSymbol(MCInst &Inst, unsigned Opcode,
                                         SMLoc IDLoc, MCStreamer &Out,
                                         bool HasTmpReg) {
  // The load/store pseudo-instruction does a pc-relative load with
  // a symbol.
  //
  // The expansion looks like this
  //
  //   TmpLabel: AUIPC tmp, %pcrel_hi(symbol)
  //             [S|L]X    rd, %pcrel_lo(TmpLabel)(tmp)
  MCOperand DestReg = Inst.getOperand(0);
  unsigned SymbolOpIdx = HasTmpReg ? 2 : 1;
  unsigned TmpRegOpIdx = HasTmpReg ? 1 : 0;
  MCOperand TmpReg = Inst.getOperand(TmpRegOpIdx);
  const MCExpr *Symbol = Inst.getOperand(SymbolOpIdx).getExpr();
  emitAuipcInstPair(DestReg, TmpReg, Symbol, PPUMCExpr::VK_PPU_PCREL_HI,
                    Opcode, IDLoc, Out);
}

bool PPUAsmParser::checkPseudoAddTPRel(MCInst &Inst,
                                         OperandVector &Operands) {
  assert(Inst.getOpcode() == PPU::PseudoAddTPRel && "Invalid instruction");
  assert(Inst.getOperand(2).isReg() && "Unexpected second operand kind");
  if (Inst.getOperand(2).getReg() != PPU::X4) {
    SMLoc ErrorLoc = ((PPUOperand &)*Operands[3]).getStartLoc();
    return Error(ErrorLoc, "the second input operand must be tp/x4 when using "
                           "%tprel_add modifier");
  }

  return false;
}

bool PPUAsmParser::processInstruction(MCInst &Inst, SMLoc IDLoc,
                                        OperandVector &Operands,
                                        MCStreamer &Out) {
  Inst.setLoc(IDLoc);

  switch (Inst.getOpcode()) {
  default:
    break;
  case PPU::PseudoLI: {
    Register Reg = Inst.getOperand(0).getReg();
    const MCOperand &Op1 = Inst.getOperand(1);
    if (Op1.isExpr()) {
      // We must have li reg, %lo(sym) or li reg, %pcrel_lo(sym) or similar.
      // Just convert to an addi. This allows compatibility with gas.
      emitToStreamer(Out, MCInstBuilder(PPU::ADDI)
                              .addReg(Reg)
                              .addReg(PPU::X0)
                              .addExpr(Op1.getExpr()));
      return false;
    }
    int64_t Imm = Inst.getOperand(1).getImm();
    // On RV32 the immediate here can either be a signed or an unsigned
    // 32-bit number. Sign extension has to be performed to ensure that Imm
    // represents the expected signed 64-bit number.
    if (!isRV64())
      Imm = SignExtend64<32>(Imm);
    emitLoadImm(Reg, Imm, Out);
    return false;
  }
  case PPU::PseudoLLA:
    emitLoadLocalAddress(Inst, IDLoc, Out);
    return false;
  case PPU::PseudoLA:
    emitLoadAddress(Inst, IDLoc, Out);
    return false;
  case PPU::PseudoLA_TLS_IE:
    emitLoadTLSIEAddress(Inst, IDLoc, Out);
    return false;
  case PPU::PseudoLA_TLS_GD:
    emitLoadTLSGDAddress(Inst, IDLoc, Out);
    return false;
  case PPU::PseudoLB:
    emitLoadStoreSymbol(Inst, PPU::LB, IDLoc, Out, /*HasTmpReg=*/false);
    return false;
  case PPU::PseudoLBU:
    emitLoadStoreSymbol(Inst, PPU::LBU, IDLoc, Out, /*HasTmpReg=*/false);
    return false;
  case PPU::PseudoLH:
    emitLoadStoreSymbol(Inst, PPU::LH, IDLoc, Out, /*HasTmpReg=*/false);
    return false;
  case PPU::PseudoLHU:
    emitLoadStoreSymbol(Inst, PPU::LHU, IDLoc, Out, /*HasTmpReg=*/false);
    return false;
  case PPU::PseudoLW:
    emitLoadStoreSymbol(Inst, PPU::LW, IDLoc, Out, /*HasTmpReg=*/false);
    return false;
  case PPU::PseudoLWU:
    emitLoadStoreSymbol(Inst, PPU::LWU, IDLoc, Out, /*HasTmpReg=*/false);
    return false;
  case PPU::PseudoLD:
    emitLoadStoreSymbol(Inst, PPU::LD, IDLoc, Out, /*HasTmpReg=*/false);
    return false;
  case PPU::PseudoFLW:
    emitLoadStoreSymbol(Inst, PPU::FLW, IDLoc, Out, /*HasTmpReg=*/true);
    return false;
  case PPU::PseudoFLD:
    emitLoadStoreSymbol(Inst, PPU::FLD, IDLoc, Out, /*HasTmpReg=*/true);
    return false;
  case PPU::PseudoSB:
    emitLoadStoreSymbol(Inst, PPU::SB, IDLoc, Out, /*HasTmpReg=*/true);
    return false;
  case PPU::PseudoSH:
    emitLoadStoreSymbol(Inst, PPU::SH, IDLoc, Out, /*HasTmpReg=*/true);
    return false;
  case PPU::PseudoSW:
    emitLoadStoreSymbol(Inst, PPU::SW, IDLoc, Out, /*HasTmpReg=*/true);
    return false;
  case PPU::PseudoSD:
    emitLoadStoreSymbol(Inst, PPU::SD, IDLoc, Out, /*HasTmpReg=*/true);
    return false;
  case PPU::PseudoFSW:
    emitLoadStoreSymbol(Inst, PPU::FSW, IDLoc, Out, /*HasTmpReg=*/true);
    return false;
  case PPU::PseudoFSD:
    emitLoadStoreSymbol(Inst, PPU::FSD, IDLoc, Out, /*HasTmpReg=*/true);
    return false;
  case PPU::PseudoAddTPRel:
    if (checkPseudoAddTPRel(Inst, Operands))
      return true;
    break;
  }

  emitToStreamer(Out, Inst);
  return false;
}

/*
static bool PPUAsmParser_classifySymbolRef(const MCExpr *Expr,
                                PPUMCExpr::VariantKind &Kind,
                                int64_t &Addend)
{
    return PPUAsmParser::classifySymbolRef(const_cast<MCExpr*>(Expr), Kind, Addend);
}
*/

