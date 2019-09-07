//===-- PPUMCExpr.cpp - PPU specific MC expression classes ------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the implementation of the assembly expression modifiers
// accepted by the PPU architecture (e.g. ":lo12:", ":gottprel_g1:", ...).
//
//===----------------------------------------------------------------------===//

#include "PPUMCExpr.h"
#include "MCTargetDesc/PPUAsmBackend.h"
#include "PPU.h"
#include "PPUFixupKinds.h"
#include "llvm/BinaryFormat/ELF.h"
#include "llvm/MC/MCAsmLayout.h"
#include "llvm/MC/MCAssembler.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/MC/MCSymbolELF.h"
#include "llvm/MC/MCValue.h"
#include "llvm/Support/ErrorHandling.h"

using namespace llvm;

#define DEBUG_TYPE "ppumcexpr"

const PPUMCExpr *PPUMCExpr::create(const MCExpr *Expr, VariantKind Kind,
                                       MCContext &Ctx) {
  return new (Ctx) PPUMCExpr(Expr, Kind);
}

void PPUMCExpr::printImpl(raw_ostream &OS, const MCAsmInfo *MAI) const {
  VariantKind Kind = getKind();
  bool HasVariant = ((Kind != VK_PPU_None) && (Kind != VK_PPU_CALL) &&
                     (Kind != VK_PPU_CALL_PLT));

  if (HasVariant)
    OS << '%' << getVariantKindName(getKind()) << '(';
  Expr->print(OS, MAI);
  if (Kind == VK_PPU_CALL_PLT)
    OS << "@plt";
  if (HasVariant)
    OS << ')';
}

const MCFixup *PPUMCExpr::getPCRelHiFixup() const {
  MCValue AUIPCLoc;
  if (!getSubExpr()->evaluateAsRelocatable(AUIPCLoc, nullptr, nullptr))
    return nullptr;

  const MCSymbolRefExpr *AUIPCSRE = AUIPCLoc.getSymA();
  if (!AUIPCSRE)
    return nullptr;

  const MCSymbol *AUIPCSymbol = &AUIPCSRE->getSymbol();
  const auto *DF = dyn_cast_or_null<MCDataFragment>(AUIPCSymbol->getFragment());

  if (!DF)
    return nullptr;

  uint64_t Offset = AUIPCSymbol->getOffset();
  if (DF->getContents().size() == Offset) {
    DF = dyn_cast_or_null<MCDataFragment>(DF->getNextNode());
    if (!DF)
      return nullptr;
    Offset = 0;
  }

  for (const MCFixup &F : DF->getFixups()) {
    if (F.getOffset() != Offset)
      continue;

    switch ((unsigned)F.getKind()) {
    default:
      continue;
    case PPU::fixup_ppu_got_hi20:
    case PPU::fixup_ppu_tls_got_hi20:
    case PPU::fixup_ppu_tls_gd_hi20:
    case PPU::fixup_ppu_pcrel_hi20:
      return &F;
    }
  }

  return nullptr;
}

bool PPUMCExpr::evaluatePCRelLo(MCValue &Res, const MCAsmLayout *Layout,
                                  const MCFixup *Fixup) const {
  // VK_PPU_PCREL_LO has to be handled specially.  The MCExpr inside is
  // actually the location of a auipc instruction with a VK_PPU_PCREL_HI fixup
  // pointing to the real target.  We need to generate an MCValue in the form of
  // (<real target> + <offset from this fixup to the auipc fixup>).  The Fixup
  // is pcrel relative to the VK_PPU_PCREL_LO fixup, so we need to add the
  // offset to the VK_PPU_PCREL_HI Fixup from VK_PPU_PCREL_LO to correct.

  // Don't try to evaluate if the fixup will be forced as a relocation (e.g.
  // as linker relaxation is enabled). If we evaluated pcrel_lo in this case,
  // the modified fixup will be converted into a relocation that no longer
  // points to the pcrel_hi as the linker requires.
  auto &RAB =
      static_cast<PPUAsmBackend &>(Layout->getAssembler().getBackend());
  if (RAB.willForceRelocations())
    return false;

  MCValue AUIPCLoc;
  if (!getSubExpr()->evaluateAsValue(AUIPCLoc, *Layout))
    return false;

  const MCSymbolRefExpr *AUIPCSRE = AUIPCLoc.getSymA();
  // Don't try to evaluate %pcrel_hi/%pcrel_lo pairs that cross fragment
  // boundries.
  if (!AUIPCSRE ||
      findAssociatedFragment() != AUIPCSRE->findAssociatedFragment())
    return false;

  const MCSymbol *AUIPCSymbol = &AUIPCSRE->getSymbol();
  if (!AUIPCSymbol)
    return false;

  const MCFixup *TargetFixup = getPCRelHiFixup();
  if (!TargetFixup)
    return false;

  if ((unsigned)TargetFixup->getKind() != PPU::fixup_ppu_pcrel_hi20)
    return false;

  MCValue Target;
  if (!TargetFixup->getValue()->evaluateAsValue(Target, *Layout))
    return false;

  if (!Target.getSymA() || !Target.getSymA()->getSymbol().isInSection())
    return false;

  if (&Target.getSymA()->getSymbol().getSection() !=
      findAssociatedFragment()->getParent())
    return false;

  uint64_t AUIPCOffset = AUIPCSymbol->getOffset();

  Res = MCValue::get(Target.getSymA(), nullptr,
                     Target.getConstant() + (Fixup->getOffset() - AUIPCOffset));
  return true;
}

bool PPUMCExpr::evaluateAsRelocatableImpl(MCValue &Res,
                                            const MCAsmLayout *Layout,
                                            const MCFixup *Fixup) const {
  if (Kind == VK_PPU_PCREL_LO && evaluatePCRelLo(Res, Layout, Fixup))
    return true;

  if (!getSubExpr()->evaluateAsRelocatable(Res, Layout, Fixup))
    return false;

  // Some custom fixup types are not valid with symbol difference expressions
  if (Res.getSymA() && Res.getSymB()) {
    switch (getKind()) {
    default:
      return true;
    case VK_PPU_LO:
    case VK_PPU_HI:
    case VK_PPU_PCREL_LO:
    case VK_PPU_PCREL_HI:
    case VK_PPU_GOT_HI:
    case VK_PPU_TPREL_LO:
    case VK_PPU_TPREL_HI:
    case VK_PPU_TPREL_ADD:
    case VK_PPU_TLS_GOT_HI:
    case VK_PPU_TLS_GD_HI:
      return false;
    }
  }

  return true;
}

void PPUMCExpr::visitUsedExpr(MCStreamer &Streamer) const {
  Streamer.visitUsedExpr(*getSubExpr());
}

PPUMCExpr::VariantKind PPUMCExpr::getVariantKindForName(StringRef name) {
  return StringSwitch<PPUMCExpr::VariantKind>(name)
      .Case("lo", VK_PPU_LO)
      .Case("hi", VK_PPU_HI)
      .Case("pcrel_lo", VK_PPU_PCREL_LO)
      .Case("pcrel_hi", VK_PPU_PCREL_HI)
      .Case("got_pcrel_hi", VK_PPU_GOT_HI)
      .Case("tprel_lo", VK_PPU_TPREL_LO)
      .Case("tprel_hi", VK_PPU_TPREL_HI)
      .Case("tprel_add", VK_PPU_TPREL_ADD)
      .Case("tls_ie_pcrel_hi", VK_PPU_TLS_GOT_HI)
      .Case("tls_gd_pcrel_hi", VK_PPU_TLS_GD_HI)
      .Default(VK_PPU_Invalid);
}

StringRef PPUMCExpr::getVariantKindName(VariantKind Kind) {
  switch (Kind) {
  default:
    llvm_unreachable("Invalid ELF symbol kind");
  case VK_PPU_LO:
    return "lo";
  case VK_PPU_HI:
    return "hi";
  case VK_PPU_PCREL_LO:
    return "pcrel_lo";
  case VK_PPU_PCREL_HI:
    return "pcrel_hi";
  case VK_PPU_GOT_HI:
    return "got_pcrel_hi";
  case VK_PPU_TPREL_LO:
    return "tprel_lo";
  case VK_PPU_TPREL_HI:
    return "tprel_hi";
  case VK_PPU_TPREL_ADD:
    return "tprel_add";
  case VK_PPU_TLS_GOT_HI:
    return "tls_ie_pcrel_hi";
  case VK_PPU_TLS_GD_HI:
    return "tls_gd_pcrel_hi";
  }
}

static void fixELFSymbolsInTLSFixupsImpl(const MCExpr *Expr, MCAssembler &Asm) {
  switch (Expr->getKind()) {
  case MCExpr::Target:
    llvm_unreachable("Can't handle nested target expression");
    break;
  case MCExpr::Constant:
    break;

  case MCExpr::Binary: {
    const MCBinaryExpr *BE = cast<MCBinaryExpr>(Expr);
    fixELFSymbolsInTLSFixupsImpl(BE->getLHS(), Asm);
    fixELFSymbolsInTLSFixupsImpl(BE->getRHS(), Asm);
    break;
  }

  case MCExpr::SymbolRef: {
    // We're known to be under a TLS fixup, so any symbol should be
    // modified. There should be only one.
    const MCSymbolRefExpr &SymRef = *cast<MCSymbolRefExpr>(Expr);
    cast<MCSymbolELF>(SymRef.getSymbol()).setType(ELF::STT_TLS);
    break;
  }

  case MCExpr::Unary:
    fixELFSymbolsInTLSFixupsImpl(cast<MCUnaryExpr>(Expr)->getSubExpr(), Asm);
    break;
  }
}

void PPUMCExpr::fixELFSymbolsInTLSFixups(MCAssembler &Asm) const {
  switch (getKind()) {
  default:
    return;
  case VK_PPU_TPREL_HI:
  case VK_PPU_TLS_GOT_HI:
  case VK_PPU_TLS_GD_HI:
    break;
  }

  fixELFSymbolsInTLSFixupsImpl(getSubExpr(), Asm);
}

bool PPUMCExpr::evaluateAsConstant(int64_t &Res) const {
  MCValue Value;

  if (Kind == VK_PPU_PCREL_HI || Kind == VK_PPU_PCREL_LO ||
      Kind == VK_PPU_GOT_HI || Kind == VK_PPU_TPREL_HI ||
      Kind == VK_PPU_TPREL_LO || Kind == VK_PPU_TPREL_ADD ||
      Kind == VK_PPU_TLS_GOT_HI || Kind == VK_PPU_TLS_GD_HI ||
      Kind == VK_PPU_CALL || Kind == VK_PPU_CALL_PLT)
    return false;

  if (!getSubExpr()->evaluateAsRelocatable(Value, nullptr, nullptr))
    return false;

  if (!Value.isAbsolute())
    return false;

  Res = evaluateAsInt64(Value.getConstant());
  return true;
}

int64_t PPUMCExpr::evaluateAsInt64(int64_t Value) const {
  switch (Kind) {
  default:
    llvm_unreachable("Invalid kind");
  case VK_PPU_LO:
    return SignExtend64<12>(Value);
  case VK_PPU_HI:
    // Add 1 if bit 11 is 1, to compensate for low 12 bits being negative.
    return ((Value + 0x800) >> 12) & 0xfffff;
  }
}
