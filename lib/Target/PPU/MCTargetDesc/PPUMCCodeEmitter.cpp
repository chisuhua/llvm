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
} // end anonymous namespace

MCCodeEmitter *llvm::createPPUMCCodeEmitter(const MCInstrInfo &MCII,
                                              const MCRegisterInfo &MRI,
                                              MCContext &Ctx) {
  return new PPUMCCodeEmitter(Ctx, MCII);
}

// Expand PseudoCALL(Reg) and PseudoTAIL to AUIPC and JALR with relocation
// types. We expand PseudoCALL(Reg) and PseudoTAIL while encoding, meaning AUIPC
// and JALR won't go through PPU MC to MC compressed instruction
// transformation. This is acceptable because AUIPC has no 16-bit form and
// C_JALR have no immediate operand field.  We let linker relaxation deal with
// it. When linker relaxation enabled, AUIPC and JALR have chance relax to JAL.
// If C extension is enabled, JAL has chance relax to C_JAL.
void PPUMCCodeEmitter::expandFunctionCall(const MCInst &MI, raw_ostream &OS,
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
void PPUMCCodeEmitter::expandAddTPRel(const MCInst &MI, raw_ostream &OS,
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

void PPUMCCodeEmitter::encodeInstruction(const MCInst &MI, raw_ostream &OS,
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
PPUMCCodeEmitter::getMachineOpValue(const MCInst &MI, const MCOperand &MO,
                                      SmallVectorImpl<MCFixup> &Fixups,
                                      const MCSubtargetInfo &STI) const {

  if (MO.isReg())
    return Ctx.getRegisterInfo()->getEncodingValue(MO.getReg());

  if (MO.isImm())
    return static_cast<unsigned>(MO.getImm());

  llvm_unreachable("Unhandled expression!");
  return 0;
}

unsigned
PPUMCCodeEmitter::getImmOpValueAsr1(const MCInst &MI, unsigned OpNo,
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

unsigned PPUMCCodeEmitter::getImmOpValue(const MCInst &MI, unsigned OpNo,
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

#include "PPUGenMCCodeEmitter.inc"
