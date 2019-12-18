//===-- PPUMCCodeEmitter.cpp - Convert PPU code to machine code -------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file implements the PPUMCCodeEmitter class.
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/PPUFixupKinds.h"
#include "MCTargetDesc/PPUMCExpr.h"
#include "MCTargetDesc/PPUMCTargetDesc.h"
#include "MCTargetDesc/PPUMCCodeEmitter.h"
#include "Utils/PPUBaseInfo.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/CodeGen/Register.h"
#include "llvm/MC/MCAsmInfo.h"
#include "llvm/MC/MCCodeEmitter.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCInstBuilder.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCSymbol.h"
#include "llvm/Support/Casting.h"
#include "llvm/Support/EndianStream.h"
#include "llvm/Support/raw_ostream.h"

using namespace llvm;

#define DEBUG_TYPE "mccodeemitter"

STATISTIC(MCNumEmitted, "Number of MC instructions emitted");
STATISTIC(MCNumFixups, "Number of MC fixups created");

namespace {
/*
class PPUMCCodeEmitter : public MCCodeEmitter {
  PPUMCCodeEmitter(const PPUMCCodeEmitter &) = delete;
  void operator=(const PPUMCCodeEmitter &) = delete;
  MCContext &Ctx;
  MCInstrInfo const &MCII;

public:
  PPUMCCodeEmitter(MCContext &ctx, MCInstrInfo const &MCII)
      : Ctx(ctx), MCII(MCII) {}

  ~PPUMCCodeEmitter() override {}

  void encodeInstruction(const MCInst &MI, raw_ostream &OS,
                         SmallVectorImpl<MCFixup> &Fixups,
                         const MCSubtargetInfo &STI) const override;

  void expandFunctionCall(const MCInst &MI, raw_ostream &OS,
                          SmallVectorImpl<MCFixup> &Fixups,
                          const MCSubtargetInfo &STI) const;

  void expandAddTPRel(const MCInst &MI, raw_ostream &OS,
                      SmallVectorImpl<MCFixup> &Fixups,
                      const MCSubtargetInfo &STI) const;

  /// TableGen'erated function for getting the binary encoding for an
  /// instruction.
  uint64_t getBinaryCodeForInstr(const MCInst &MI,
                                 SmallVectorImpl<MCFixup> &Fixups,
                                 const MCSubtargetInfo &STI) const;

  /// Return binary encoding of operand. If the machine operand requires
  /// relocation, record the relocation and return zero.
  unsigned getMachineOpValue(const MCInst &MI, const MCOperand &MO,
                             SmallVectorImpl<MCFixup> &Fixups,
                             const MCSubtargetInfo &STI) const;

  unsigned getImmOpValueAsr1(const MCInst &MI, unsigned OpNo,
                             SmallVectorImpl<MCFixup> &Fixups,
                             const MCSubtargetInfo &STI) const;

  unsigned getImmOpValue(const MCInst &MI, unsigned OpNo,
                         SmallVectorImpl<MCFixup> &Fixups,
                         const MCSubtargetInfo &STI) const;
};
*/
} // end anonymous namespace


// Expand PseudoCALL(Reg) and PseudoTAIL to AUIPC and JALR with relocation
// types. We expand PseudoCALL(Reg) and PseudoTAIL while encoding, meaning AUIPC
// and JALR won't go through PPU MC to MC compressed instruction
// transformation. This is acceptable because AUIPC has no 16-bit form and
// C_JALR have no immediate operand field.  We let linker relaxation deal with
// it. When linker relaxation enabled, AUIPC and JALR have chance relax to JAL.
// If C extension is enabled, JAL has chance relax to C_JAL.
void PPUBaseMCCodeEmitter::expandFunctionCall(const MCInst &MI, raw_ostream &OS,
                                            SmallVectorImpl<MCFixup> &Fixups,
                                            const MCSubtargetInfo &STI) const {
  MCInst TmpInst;
  MCOperand Func;
  Register Ra;
  if (MI.getOpcode() == PPU::PseudoTAIL) {
    Func = MI.getOperand(0);
    Ra = PPU::X6;
  } else if (MI.getOpcode() == PPU::PseudoCALLReg) {
    Func = MI.getOperand(1);
    Ra = MI.getOperand(0).getReg();
  } else {
    Func = MI.getOperand(0);
    Ra = PPU::X1;
  }
  uint32_t Binary;

  assert(Func.isExpr() && "Expected expression");

  const MCExpr *CallExpr = Func.getExpr();

  // Emit AUIPC Ra, Func with R_PPU_CALL relocation type.
  TmpInst = MCInstBuilder(PPU::AUIPC)
                .addReg(Ra)
                .addOperand(MCOperand::createExpr(CallExpr));
  Binary = getBinaryCodeForInstr(TmpInst, Fixups, STI);
  support::endian::write(OS, Binary, support::little);

  if (MI.getOpcode() == PPU::PseudoTAIL)
    // Emit JALR X0, X6, 0
    TmpInst = MCInstBuilder(PPU::JALR).addReg(PPU::X0).addReg(Ra).addImm(0);
  else
    // Emit JALR Ra, Ra, 0
    TmpInst = MCInstBuilder(PPU::JALR).addReg(Ra).addReg(Ra).addImm(0);
  Binary = getBinaryCodeForInstr(TmpInst, Fixups, STI);
  support::endian::write(OS, Binary, support::little);
}

// Expand PseudoAddTPRel to a simple ADD with the correct relocation.
void PPUBaseMCCodeEmitter::expandAddTPRel(const MCInst &MI, raw_ostream &OS,
                                        SmallVectorImpl<MCFixup> &Fixups,
                                        const MCSubtargetInfo &STI) const {
  MCOperand DestReg = MI.getOperand(0);
  MCOperand SrcReg = MI.getOperand(1);
  MCOperand TPReg = MI.getOperand(2);
  assert(TPReg.isReg() && TPReg.getReg() == PPU::X4 &&
         "Expected thread pointer as second input to TP-relative add");

  MCOperand SrcSymbol = MI.getOperand(3);
  assert(SrcSymbol.isExpr() &&
         "Expected expression as third input to TP-relative add");

  const PPUMCExpr *Expr = dyn_cast<PPUMCExpr>(SrcSymbol.getExpr());
  assert(Expr && Expr->getKind() == PPUMCExpr::VK_PPU_TPREL_ADD &&
         "Expected tprel_add relocation on TP-relative symbol");

  // Emit the correct tprel_add relocation for the symbol.
  Fixups.push_back(MCFixup::create(
      0, Expr, MCFixupKind(PPU::fixup_ppu_tprel_add), MI.getLoc()));

  // Emit fixup_ppu_relax for tprel_add where the relax feature is enabled.
  if (STI.getFeatureBits()[PPU::FeatureRelax]) {
    const MCConstantExpr *Dummy = MCConstantExpr::create(0, Ctx);
    Fixups.push_back(MCFixup::create(
        0, Dummy, MCFixupKind(PPU::fixup_ppu_relax), MI.getLoc()));
  }

  // Emit a normal ADD instruction with the given operands.
  MCInst TmpInst = MCInstBuilder(PPU::ADD)
                       .addOperand(DestReg)
                       .addOperand(SrcReg)
                       .addOperand(TPReg);
  uint32_t Binary = getBinaryCodeForInstr(TmpInst, Fixups, STI);
  support::endian::write(OS, Binary, support::little);
}

void PPUBaseMCCodeEmitter::encodeInstruction(const MCInst &MI, raw_ostream &OS,
                                           SmallVectorImpl<MCFixup> &Fixups,
                                           const MCSubtargetInfo &STI) const {
  const MCInstrDesc &Desc = MCII.get(MI.getOpcode());
  // Get byte count of instruction.
  unsigned Size = Desc.getSize();

  if (MI.getOpcode() == PPU::PseudoCALLReg ||
      MI.getOpcode() == PPU::PseudoCALL ||
      MI.getOpcode() == PPU::PseudoTAIL) {
    expandFunctionCall(MI, OS, Fixups, STI);
    MCNumEmitted += 2;
    return;
  }

  if (MI.getOpcode() == PPU::PseudoAddTPRel) {
    expandAddTPRel(MI, OS, Fixups, STI);
    MCNumEmitted += 1;
    return;
  }

  switch (Size) {
  default:
    llvm_unreachable("Unhandled encodeInstruction length!");
  case 2: {
    uint16_t Bits = getBinaryCodeForInstr(MI, Fixups, STI);
    support::endian::write<uint16_t>(OS, Bits, support::little);
    break;
  }
  case 4: {
    uint32_t Bits = getBinaryCodeForInstr(MI, Fixups, STI);
    support::endian::write(OS, Bits, support::little);
    break;
  }
  }

  ++MCNumEmitted; // Keep track of the # of mi's emitted.
}

unsigned
PPUBaseMCCodeEmitter::getImmOpValueAsr1(const MCInst &MI, unsigned OpNo,
                                      SmallVectorImpl<MCFixup> &Fixups,
                                      const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);

  if (MO.isImm()) {
    unsigned Res = MO.getImm();
    assert((Res & 1) == 0 && "LSB is non-zero");
    return Res >> 1;
  }

  return getImmOpValue(MI, OpNo, Fixups, STI);
}

unsigned PPUBaseMCCodeEmitter::getImmOpValue(const MCInst &MI, unsigned OpNo,
                                           SmallVectorImpl<MCFixup> &Fixups,
                                           const MCSubtargetInfo &STI) const {
  bool EnableRelax = STI.getFeatureBits()[PPU::FeatureRelax];
  const MCOperand &MO = MI.getOperand(OpNo);

  MCInstrDesc const &Desc = MCII.get(MI.getOpcode());
  unsigned MIFrm = Desc.TSFlags & PPUII::InstFormatMask;

  // If the destination is an immediate, there is nothing to do.
  if (MO.isImm())
    return MO.getImm();

  assert(MO.isExpr() &&
         "getImmOpValue expects only expressions or immediates");
  const MCExpr *Expr = MO.getExpr();
  MCExpr::ExprKind Kind = Expr->getKind();
  PPU::Fixups FixupKind = PPU::fixup_ppu_invalid;
  bool RelaxCandidate = false;
  if (Kind == MCExpr::Target) {
    const PPUMCExpr *RVExpr = cast<PPUMCExpr>(Expr);

    switch (RVExpr->getKind()) {
    case PPUMCExpr::VK_PPU_None:
    case PPUMCExpr::VK_PPU_Invalid:
    case PPUMCExpr::VK_PPU_32_PCREL:
      llvm_unreachable("Unhandled fixup kind!");
    case PPUMCExpr::VK_PPU_TPREL_ADD:
      // tprel_add is only used to indicate that a relocation should be emitted
      // for an add instruction used in TP-relative addressing. It should not be
      // expanded as if representing an actual instruction operand and so to
      // encounter it here is an error.
      llvm_unreachable(
          "VK_PPU_TPREL_ADD should not represent an instruction operand");
    case PPUMCExpr::VK_PPU_LO:
      if (MIFrm == PPUII::InstFormatI)
        FixupKind = PPU::fixup_ppu_lo12_i;
      else if (MIFrm == PPUII::InstFormatS)
        FixupKind = PPU::fixup_ppu_lo12_s;
      else
        llvm_unreachable("VK_PPU_LO used with unexpected instruction format");
      RelaxCandidate = true;
      break;
    case PPUMCExpr::VK_PPU_HI:
      FixupKind = PPU::fixup_ppu_hi20;
      RelaxCandidate = true;
      break;
    case PPUMCExpr::VK_PPU_PCREL_LO:
      if (MIFrm == PPUII::InstFormatI)
        FixupKind = PPU::fixup_ppu_pcrel_lo12_i;
      else if (MIFrm == PPUII::InstFormatS)
        FixupKind = PPU::fixup_ppu_pcrel_lo12_s;
      else
        llvm_unreachable(
            "VK_PPU_PCREL_LO used with unexpected instruction format");
      RelaxCandidate = true;
      break;
    case PPUMCExpr::VK_PPU_PCREL_HI:
      FixupKind = PPU::fixup_ppu_pcrel_hi20;
      RelaxCandidate = true;
      break;
    case PPUMCExpr::VK_PPU_GOT_HI:
      FixupKind = PPU::fixup_ppu_got_hi20;
      break;
    case PPUMCExpr::VK_PPU_TPREL_LO:
      if (MIFrm == PPUII::InstFormatI)
        FixupKind = PPU::fixup_ppu_tprel_lo12_i;
      else if (MIFrm == PPUII::InstFormatS)
        FixupKind = PPU::fixup_ppu_tprel_lo12_s;
      else
        llvm_unreachable(
            "VK_PPU_TPREL_LO used with unexpected instruction format");
      RelaxCandidate = true;
      break;
    case PPUMCExpr::VK_PPU_TPREL_HI:
      FixupKind = PPU::fixup_ppu_tprel_hi20;
      RelaxCandidate = true;
      break;
    case PPUMCExpr::VK_PPU_TLS_GOT_HI:
      FixupKind = PPU::fixup_ppu_tls_got_hi20;
      break;
    case PPUMCExpr::VK_PPU_TLS_GD_HI:
      FixupKind = PPU::fixup_ppu_tls_gd_hi20;
      break;
    case PPUMCExpr::VK_PPU_CALL:
      FixupKind = PPU::fixup_ppu_call;
      RelaxCandidate = true;
      break;
    case PPUMCExpr::VK_PPU_CALL_PLT:
      FixupKind = PPU::fixup_ppu_call_plt;
      RelaxCandidate = true;
      break;
    }
  } else if (Kind == MCExpr::SymbolRef &&
             cast<MCSymbolRefExpr>(Expr)->getKind() == MCSymbolRefExpr::VK_None) {
    if (Desc.getOpcode() == PPU::JAL) {
      FixupKind = PPU::fixup_ppu_jal;
    } else if (MIFrm == PPUII::InstFormatB) {
      FixupKind = PPU::fixup_ppu_branch;
    } else if (MIFrm == PPUII::InstFormatCJ) {
      FixupKind = PPU::fixup_ppu_rvc_jump;
    } else if (MIFrm == PPUII::InstFormatCB) {
      FixupKind = PPU::fixup_ppu_rvc_branch;
    }
  }

  assert(FixupKind != PPU::fixup_ppu_invalid && "Unhandled expression!");

  Fixups.push_back(
      MCFixup::create(0, Expr, MCFixupKind(FixupKind), MI.getLoc()));
  ++MCNumFixups;

  // Ensure an R_PPU_RELAX relocation will be emitted if linker relaxation is
  // enabled and the current fixup will result in a relocation that may be
  // relaxed.
  if (EnableRelax && RelaxCandidate) {
    const MCConstantExpr *Dummy = MCConstantExpr::create(0, Ctx);
    Fixups.push_back(
    MCFixup::create(0, Dummy, MCFixupKind(PPU::fixup_ppu_relax),
                    MI.getLoc()));
    ++MCNumFixups;
  }

  return 0;
}

namespace {

class PPUMCCodeEmitter : public  PPUBaseMCCodeEmitter {
  const MCRegisterInfo &MRI;

  /// Encode an fp or int literal
  uint32_t getLitEncoding(const MCOperand &MO, const MCOperandInfo &OpInfo,
                          const MCSubtargetInfo &STI) const;

public:
  PPUMCCodeEmitter(const MCInstrInfo &mcii, const MCRegisterInfo &mri,
                  MCContext &ctx)
      : PPUBaseMCCodeEmitter(ctx, mcii), MRI(mri) {}
  PPUMCCodeEmitter(const PPUMCCodeEmitter &) = delete;
  PPUMCCodeEmitter &operator=(const PPUMCCodeEmitter &) = delete;

  /// Encode the instruction and write it to the OS.
  void encodeInstruction(const MCInst &MI, raw_ostream &OS,
                         SmallVectorImpl<MCFixup> &Fixups,
                         const MCSubtargetInfo &STI) const override;

   uint64_t getBinaryCodeForInstr(const MCInst &MI,
                                 SmallVectorImpl<MCFixup> &Fixups,
                                 const MCSubtargetInfo &STI) const;


  /// \returns the encoding for an MCOperand.
  uint64_t getMachineOpValue(const MCInst &MI, const MCOperand &MO,
                             SmallVectorImpl<MCFixup> &Fixups,
                             const MCSubtargetInfo &STI) const override;

  /// Use a fixup to encode the simm16 field for SOPP branch
  ///        instructions.
  unsigned getSOPPBrEncoding(const MCInst &MI, unsigned OpNo,
                             SmallVectorImpl<MCFixup> &Fixups,
                             const MCSubtargetInfo &STI) const override;
/*
  unsigned getSDWASrcEncoding(const MCInst &MI, unsigned OpNo,
                              SmallVectorImpl<MCFixup> &Fixups,
                              const MCSubtargetInfo &STI) const override;

  unsigned getSDWAVopcDstEncoding(const MCInst &MI, unsigned OpNo,
                                  SmallVectorImpl<MCFixup> &Fixups,
                                  const MCSubtargetInfo &STI) const override;

  unsigned getAVOperandEncoding(const MCInst &MI, unsigned OpNo,
                                SmallVectorImpl<MCFixup> &Fixups,
                                const MCSubtargetInfo &STI) const override;
                                */

protected:
  FeatureBitset computeAvailableFeatures(const FeatureBitset &FB) const;
  void
  verifyInstructionPredicates(const MCInst &MI,
                              const FeatureBitset &AvailableFeatures) const;


};

} // end anonymous namespace

MCCodeEmitter *llvm::createPPUMCCodeEmitter(const MCInstrInfo &MCII,
                                              const MCRegisterInfo &MRI,
                                              MCContext &Ctx) {
  return new PPUMCCodeEmitter(MCII, MRI, Ctx);
}
/*
uint64_t PPUMCCodeEmitter::getMachineOpValue(const MCInst &MI, const MCOperand &MO,
                                      SmallVectorImpl<MCFixup> &Fixups,
                                      const MCSubtargetInfo &STI) const {

  if (MO.isReg())
    return Ctx.getRegisterInfo()->getEncodingValue(MO.getReg());

  if (MO.isImm())
    return static_cast<unsigned>(MO.getImm());

  llvm_unreachable("Unhandled expression!");
  return 0;
}
*/


// Returns the encoding value to use if the given integer is an integer inline
// immediate value, or 0 if it is not.
template <typename IntTy>
static uint32_t getIntInlineImmEncoding(IntTy Imm) {
  if (Imm >= 0 && Imm <= 64)
    return 128 + Imm;

  if (Imm >= -16 && Imm <= -1)
    return 192 + std::abs(Imm);

  return 0;
}

static uint32_t getLit16Encoding(uint16_t Val, const MCSubtargetInfo &STI) {
  uint16_t IntImm = getIntInlineImmEncoding(static_cast<int16_t>(Val));
  if (IntImm != 0)
    return IntImm;

  if (Val == 0x3800) // 0.5
    return 240;

  if (Val == 0xB800) // -0.5
    return 241;

  if (Val == 0x3C00) // 1.0
    return 242;

  if (Val == 0xBC00) // -1.0
    return 243;

  if (Val == 0x4000) // 2.0
    return 244;

  if (Val == 0xC000) // -2.0
    return 245;

  if (Val == 0x4400) // 4.0
    return 246;

  if (Val == 0xC400) // -4.0
    return 247;

  if (Val == 0x3118 && // 1.0 / (2.0 * pi)
      STI.getFeatureBits()[PPU::FeatureInv2PiInlineImm])
    return 248;

  return 255;
}

static uint32_t getLit32Encoding(uint32_t Val, const MCSubtargetInfo &STI) {
  uint32_t IntImm = getIntInlineImmEncoding(static_cast<int32_t>(Val));
  if (IntImm != 0)
    return IntImm;

  if (Val == FloatToBits(0.5f))
    return 240;

  if (Val == FloatToBits(-0.5f))
    return 241;

  if (Val == FloatToBits(1.0f))
    return 242;

  if (Val == FloatToBits(-1.0f))
    return 243;

  if (Val == FloatToBits(2.0f))
    return 244;

  if (Val == FloatToBits(-2.0f))
    return 245;

  if (Val == FloatToBits(4.0f))
    return 246;

  if (Val == FloatToBits(-4.0f))
    return 247;

  if (Val == 0x3e22f983 && // 1.0 / (2.0 * pi)
      STI.getFeatureBits()[PPU::FeatureInv2PiInlineImm])
    return 248;

  return 255;
}

static uint32_t getLit64Encoding(uint64_t Val, const MCSubtargetInfo &STI) {
  uint32_t IntImm = getIntInlineImmEncoding(static_cast<int64_t>(Val));
  if (IntImm != 0)
    return IntImm;

  if (Val == DoubleToBits(0.5))
    return 240;

  if (Val == DoubleToBits(-0.5))
    return 241;

  if (Val == DoubleToBits(1.0))
    return 242;

  if (Val == DoubleToBits(-1.0))
    return 243;

  if (Val == DoubleToBits(2.0))
    return 244;

  if (Val == DoubleToBits(-2.0))
    return 245;

  if (Val == DoubleToBits(4.0))
    return 246;

  if (Val == DoubleToBits(-4.0))
    return 247;

  if (Val == 0x3fc45f306dc9c882 && // 1.0 / (2.0 * pi)
      STI.getFeatureBits()[PPU::FeatureInv2PiInlineImm])
    return 248;

  return 255;
}

uint32_t PPUMCCodeEmitter::getLitEncoding(const MCOperand &MO,
                                         const MCOperandInfo &OpInfo,
                                         const MCSubtargetInfo &STI) const {
  int64_t Imm;
  if (MO.isExpr()) {
    const auto *C = dyn_cast<MCConstantExpr>(MO.getExpr());
    if (!C)
      return 255;

    Imm = C->getValue();
  } else {

    assert(!MO.isFPImm());

    if (!MO.isImm())
      return ~0;

    Imm = MO.getImm();
  }

  switch (OpInfo.OperandType) {
  case PPU::OPERAND_REG_IMM_INT32:
  case PPU::OPERAND_REG_IMM_FP32:
  case PPU::OPERAND_REG_INLINE_C_INT32:
  case PPU::OPERAND_REG_INLINE_C_FP32:
  case PPU::OPERAND_REG_INLINE_AC_INT32:
  case PPU::OPERAND_REG_INLINE_AC_FP32:
    return getLit32Encoding(static_cast<uint32_t>(Imm), STI);

  case PPU::OPERAND_REG_IMM_INT64:
  case PPU::OPERAND_REG_IMM_FP64:
  case PPU::OPERAND_REG_INLINE_C_INT64:
  case PPU::OPERAND_REG_INLINE_C_FP64:
    return getLit64Encoding(static_cast<uint64_t>(Imm), STI);

  case PPU::OPERAND_REG_IMM_INT16:
  case PPU::OPERAND_REG_IMM_FP16:
  case PPU::OPERAND_REG_INLINE_C_INT16:
  case PPU::OPERAND_REG_INLINE_C_FP16:
  case PPU::OPERAND_REG_INLINE_AC_INT16:
  case PPU::OPERAND_REG_INLINE_AC_FP16:
    // FIXME Is this correct? What do inline immediates do on SI for f16 src
    // which does not have f16 support?
    return getLit16Encoding(static_cast<uint16_t>(Imm), STI);

  case PPU::OPERAND_REG_IMM_V2INT16:
  case PPU::OPERAND_REG_IMM_V2FP16:
    if (!isUInt<16>(Imm) && STI.getFeatureBits()[PPU::FeatureVOP3Literal])
      return getLit32Encoding(static_cast<uint32_t>(Imm), STI);
    LLVM_FALLTHROUGH;
  case PPU::OPERAND_REG_INLINE_C_V2INT16:
  case PPU::OPERAND_REG_INLINE_C_V2FP16:
  case PPU::OPERAND_REG_INLINE_AC_V2INT16:
  case PPU::OPERAND_REG_INLINE_AC_V2FP16: {
    uint16_t Lo16 = static_cast<uint16_t>(Imm);
    uint32_t Encoding = getLit16Encoding(Lo16, STI);
    return Encoding;
  }
  default:
    llvm_unreachable("invalid operand size");
  }
}

void PPUMCCodeEmitter::encodeInstruction(const MCInst &MI, raw_ostream &OS,
                                       SmallVectorImpl<MCFixup> &Fixups,
                                       const MCSubtargetInfo &STI) const {
  verifyInstructionPredicates(MI,
                              computeAvailableFeatures(STI.getFeatureBits()));

  // call base
  PPUBaseMCCodeEmitter::encodeInstruction(MI, OS, Fixups, STI);

  uint64_t Encoding = getBinaryCodeForInstr(MI, Fixups, STI);
  const MCInstrDesc &Desc = MCII.get(MI.getOpcode());
  unsigned bytes = Desc.getSize();

  for (unsigned i = 0; i < bytes; i++) {
    OS.write((uint8_t) ((Encoding >> (8 * i)) & 0xff));
  }
/*
  // NSA encoding.
  if (PPU::isGFX10(STI) && Desc.TSFlags & SIInstrFlags::MIMG) {
    int vaddr0 = PPU::getNamedOperandIdx(MI.getOpcode(),
                                            PPU::OpName::vaddr0);
    int srsrc = PPU::getNamedOperandIdx(MI.getOpcode(),
                                           PPU::OpName::srsrc);
    assert(vaddr0 >= 0 && srsrc > vaddr0);
    unsigned NumExtraAddrs = srsrc - vaddr0 - 1;
    unsigned NumPadding = (-NumExtraAddrs) & 3;

    for (unsigned i = 0; i < NumExtraAddrs; ++i)
      OS.write((uint8_t)getMachineOpValue(MI, MI.getOperand(vaddr0 + 1 + i),
                                          Fixups, STI));
    for (unsigned i = 0; i < NumPadding; ++i)
      OS.write(0);
  }
*/
  if ((bytes > 8 && STI.getFeatureBits()[PPU::FeatureVOP3Literal]) ||
      (bytes > 4 && !STI.getFeatureBits()[PPU::FeatureVOP3Literal]))
    return;

  // Check for additional literals in SRC0/1/2 (Op 1/2/3)
  for (unsigned i = 0, e = Desc.getNumOperands(); i < e; ++i) {

    // Check if this operand should be encoded as [SV]Src
    if (!PPU::isSISrcOperand(Desc, i))
      continue;

    // Is this operand a literal immediate?
    const MCOperand &Op = MI.getOperand(i);
    if (getLitEncoding(Op, Desc.OpInfo[i], STI) != 255)
      continue;

    // Yes! Encode it
    int64_t Imm = 0;

    if (Op.isImm())
      Imm = Op.getImm();
    else if (Op.isExpr()) {
      if (const auto *C = dyn_cast<MCConstantExpr>(Op.getExpr()))
        Imm = C->getValue();

    } else if (!Op.isExpr()) // Exprs will be replaced with a fixup value.
      llvm_unreachable("Must be immediate or expr");

    for (unsigned j = 0; j < 4; j++) {
      OS.write((uint8_t) ((Imm >> (8 * j)) & 0xff));
    }

    // Only one literal value allowed
    break;
  }
}

unsigned PPUMCCodeEmitter::getSOPPBrEncoding(const MCInst &MI, unsigned OpNo,
                                            SmallVectorImpl<MCFixup> &Fixups,
                                            const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);

  if (MO.isExpr()) {
    const MCExpr *Expr = MO.getExpr();
    llvm_unreachable("getSOPPBrEncoding WIP");
    /*
    MCFixupKind Kind = (MCFixupKind)PPU::fixup_si_sopp_br;
    Fixups.push_back(MCFixup::create(0, Expr, Kind, MI.getLoc()));
    */
    return 0;
  }

  return getMachineOpValue(MI, MO, Fixups, STI);
}
/*
unsigned
PPUMCCodeEmitter::getSDWASrcEncoding(const MCInst &MI, unsigned OpNo,
                                    SmallVectorImpl<MCFixup> &Fixups,
                                    const MCSubtargetInfo &STI) const {
  using namespace PPU::SDWA;

  uint64_t RegEnc = 0;

  const MCOperand &MO = MI.getOperand(OpNo);

  if (MO.isReg()) {
    unsigned Reg = MO.getReg();
    RegEnc |= MRI.getEncodingValue(Reg);
    RegEnc &= SDWA9EncValues::SRC_VGPR_MASK;
    if (PPU::isSGPR(PPU::mc2PseudoReg(Reg), &MRI)) {
      RegEnc |= SDWA9EncValues::SRC_SGPR_MASK;
    }
    return RegEnc;
  } else {
    const MCInstrDesc &Desc = MCII.get(MI.getOpcode());
    uint32_t Enc = getLitEncoding(MO, Desc.OpInfo[OpNo], STI);
    if (Enc != ~0U && Enc != 255) {
      return Enc | SDWA9EncValues::SRC_SGPR_MASK;
    }
  }

  llvm_unreachable("Unsupported operand kind");
  return 0;
}

unsigned
PPUMCCodeEmitter::getSDWAVopcDstEncoding(const MCInst &MI, unsigned OpNo,
                                        SmallVectorImpl<MCFixup> &Fixups,
                                        const MCSubtargetInfo &STI) const {
  using namespace PPU::SDWA;

  uint64_t RegEnc = 0;

  const MCOperand &MO = MI.getOperand(OpNo);

  unsigned Reg = MO.getReg();
  if (Reg != PPU::VCC && Reg != PPU::VCC_LO) {
    RegEnc |= MRI.getEncodingValue(Reg);
    RegEnc &= SDWA9EncValues::VOPC_DST_SGPR_MASK;
    RegEnc |= SDWA9EncValues::VOPC_DST_VCC_MASK;
  }
  return RegEnc;
}

unsigned
PPUMCCodeEmitter::getAVOperandEncoding(const MCInst &MI, unsigned OpNo,
                                      SmallVectorImpl<MCFixup> &Fixups,
                                      const MCSubtargetInfo &STI) const {
  unsigned Reg = MI.getOperand(OpNo).getReg();
  uint64_t Enc = MRI.getEncodingValue(Reg);

  // VGPR and AGPR have the same encoding, but SrcA and SrcB operands of mfma
  // instructions use acc[0:1] modifier bits to distinguish. These bits are
  // encoded as a virtual 9th bit of the register for these operands.
  if (MRI.getRegClass(PPU::AGPR_32RegClassID).contains(Reg) ||
      MRI.getRegClass(PPU::AReg_64RegClassID).contains(Reg))
    Enc |= 512;

  return Enc;
}
*/

static bool needsPCRel(const MCExpr *Expr) {
  switch (Expr->getKind()) {
  case MCExpr::SymbolRef: {
    auto *SE = cast<MCSymbolRefExpr>(Expr);
    MCSymbolRefExpr::VariantKind Kind = SE->getKind();
    return Kind != MCSymbolRefExpr::VK_AMDGPU_ABS32_LO &&
           Kind != MCSymbolRefExpr::VK_AMDGPU_ABS32_HI;
  }
  case MCExpr::Binary: {
    auto *BE = cast<MCBinaryExpr>(Expr);
    if (BE->getOpcode() == MCBinaryExpr::Sub)
      return false;
    return needsPCRel(BE->getLHS()) || needsPCRel(BE->getRHS());
  }
  case MCExpr::Unary:
    return needsPCRel(cast<MCUnaryExpr>(Expr)->getSubExpr());
  case MCExpr::Target:
  case MCExpr::Constant:
    return false;
  }
  llvm_unreachable("invalid kind");
}


// this is AMD version, we comment out RISCV version
uint64_t PPUMCCodeEmitter::getMachineOpValue(const MCInst &MI,
                                            const MCOperand &MO,
                                       SmallVectorImpl<MCFixup> &Fixups,
                                       const MCSubtargetInfo &STI) const {
  if (MO.isReg())
    return MRI.getEncodingValue(MO.getReg());

  if (MO.isExpr() && MO.getExpr()->getKind() != MCExpr::Constant) {
    // FIXME: If this is expression is PCRel or not should not depend on what
    // the expression looks like. Given that this is just a general expression,
    // it should probably be FK_Data_4 and whatever is producing
    //
    //    s_add_u32 s2, s2, (extern_const_addrspace+16
    //
    // And expecting a PCRel should instead produce
    //
    // .Ltmp1:
    //   s_add_u32 s2, s2, (extern_const_addrspace+16)-.Ltmp1
    MCFixupKind Kind;
    if (needsPCRel(MO.getExpr()))
      Kind = FK_PCRel_4;
    else
      Kind = FK_Data_4;

    const MCInstrDesc &Desc = MCII.get(MI.getOpcode());
    uint32_t Offset = Desc.getSize();
    assert(Offset == 4 || Offset == 8);

    Fixups.push_back(
      MCFixup::create(Offset, MO.getExpr(), Kind, MI.getLoc()));
  }

  // Figure out the operand number, needed for isSrcOperand check
  unsigned OpNo = 0;
  for (unsigned e = MI.getNumOperands(); OpNo < e; ++OpNo) {
    if (&MO == &MI.getOperand(OpNo))
      break;
  }

  const MCInstrDesc &Desc = MCII.get(MI.getOpcode());
  if (PPU::isSISrcOperand(Desc, OpNo)) {
    uint32_t Enc = getLitEncoding(MO, Desc.OpInfo[OpNo], STI);
    if (Enc != ~0U &&
        (Enc != 255 || Desc.getSize() == 4 || Desc.getSize() == 8))
      return Enc;

  } else if (MO.isImm())
    return MO.getImm();

  llvm_unreachable("Encoding of this operand type is not supported yet.");
  return 0;
}

#define ENABLE_INSTR_PREDICATE_VERIFIER
#include "PPUGenMCCodeEmitter.inc"
