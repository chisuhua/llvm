//===-- PPUELFObjectWriter.cpp - PPU ELF Writer -----------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/PPUFixupKinds.h"
#include "MCTargetDesc/PPUMCExpr.h"
#include "MCTargetDesc/PPUMCTargetDesc.h"
#include "llvm/MC/MCELFObjectWriter.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCFixup.h"
#include "llvm/MC/MCObjectWriter.h"
#include "llvm/MC/MCSymbol.h"
#include "llvm/MC/MCValue.h"
#include "llvm/Support/ErrorHandling.h"

using namespace llvm;

namespace {
class PPUELFObjectWriter : public MCELFObjectTargetWriter {
public:
  PPUELFObjectWriter(uint8_t OSABI, bool Is64Bit);

  ~PPUELFObjectWriter() override;

  // Return true if the given relocation must be with a symbol rather than
  // section plus offset.
  bool needsRelocateWithSymbol(const MCSymbol &Sym,
                               unsigned Type) const override {
    // TODO: this is very conservative, update once PPU psABI requirements
    //       are clarified.
    return true;
  }

protected:
  unsigned getRelocType(MCContext &Ctx, const MCValue &Target,
                        const MCFixup &Fixup, bool IsPCRel) const override;
};
}

PPUELFObjectWriter::PPUELFObjectWriter(uint8_t OSABI, bool Is64Bit)
    : MCELFObjectTargetWriter(Is64Bit, OSABI, ELF::EM_PPU,
                              /*HasRelocationAddend*/ true) {}

PPUELFObjectWriter::~PPUELFObjectWriter() {}

unsigned PPUELFObjectWriter::getRelocType(MCContext &Ctx,
                                            const MCValue &Target,
                                            const MCFixup &Fixup,
                                            bool IsPCRel) const {
/*
  if (const auto *SymA = Target.getSymA()) {
    // SCRATCH_RSRC_DWORD[01] is a special global variable that represents
    // the scratch buffer.
    if (SymA->getSymbol().getName() == "SCRATCH_RSRC_DWORD0" ||
        SymA->getSymbol().getName() == "SCRATCH_RSRC_DWORD1")
      return ELF::R_PPU_ABS32_LO;
  }
*/
  switch (Target.getAccessVariant()) {
  default:
    break;
  case MCSymbolRefExpr::VK_GOTPCREL:
    return ELF::R_PPU_GOTPCREL;
  case MCSymbolRefExpr::VK_AMDGPU_GOTPCREL32_LO:
    return ELF::R_PPU_GOTPCREL32_LO;
  case MCSymbolRefExpr::VK_AMDGPU_GOTPCREL32_HI:
    return ELF::R_PPU_GOTPCREL32_HI;
  case MCSymbolRefExpr::VK_AMDGPU_REL32_LO:
    return ELF::R_PPU_REL32_LO;
  case MCSymbolRefExpr::VK_AMDGPU_REL32_HI:
    return ELF::R_PPU_REL32_HI;
  case MCSymbolRefExpr::VK_AMDGPU_REL64:
    return ELF::R_PPU_REL64;
  }
  // above is from AMD


  const MCExpr *Expr = Fixup.getValue();
  // Determine the type of the relocation
  unsigned Kind = Fixup.getTargetKind();
  if (IsPCRel) {
    switch (Kind) {
    default:
      llvm_unreachable("invalid fixup kind!");
    case FK_Data_4:
    case FK_PCRel_4:
      return ELF::R_PPU_32_PCREL;
    case PPU::fixup_ppu_pcrel_hi20:
      return ELF::R_PPU_PCREL_HI20;
    case PPU::fixup_ppu_pcrel_lo12_i:
      return ELF::R_PPU_PCREL_LO12_I;
    case PPU::fixup_ppu_pcrel_lo12_s:
      return ELF::R_PPU_PCREL_LO12_S;
    case PPU::fixup_ppu_got_hi20:
      return ELF::R_PPU_GOT_HI20;
    case PPU::fixup_ppu_tls_got_hi20:
      return ELF::R_PPU_TLS_GOT_HI20;
    case PPU::fixup_ppu_tls_gd_hi20:
      return ELF::R_PPU_TLS_GD_HI20;
    case PPU::fixup_ppu_jal:
      return ELF::R_PPU_JAL;
    case PPU::fixup_ppu_branch:
      return ELF::R_PPU_BRANCH;
    case PPU::fixup_ppu_rvc_jump:
      return ELF::R_PPU_RVC_JUMP;
    case PPU::fixup_ppu_rvc_branch:
      return ELF::R_PPU_RVC_BRANCH;
    case PPU::fixup_ppu_call:
      return ELF::R_PPU_CALL;
    case PPU::fixup_ppu_call_plt:
      return ELF::R_PPU_CALL_PLT;
    }
  }

  switch (Kind) {
  default:
    llvm_unreachable("invalid fixup kind!");
  case FK_Data_4:
    if (Expr->getKind() == MCExpr::Target &&
        cast<PPUMCExpr>(Expr)->getKind() == PPUMCExpr::VK_PPU_32_PCREL)
      return ELF::R_PPU_32_PCREL;
    // return ELF::R_PPU_32;
    return ELF::R_PPU_ABS32;
  case FK_Data_8:
    // return ELF::R_PPU_64;
    return ELF::R_PPU_ABS64;
  case FK_Data_Add_1:
    return ELF::R_PPU_ADD8;
  case FK_Data_Add_2:
    return ELF::R_PPU_ADD16;
  case FK_Data_Add_4:
    return ELF::R_PPU_ADD32;
  case FK_Data_Add_8:
    return ELF::R_PPU_ADD64;
  case FK_Data_Add_6b:
    return ELF::R_PPU_SET6;
  case FK_Data_Sub_1:
    return ELF::R_PPU_SUB8;
  case FK_Data_Sub_2:
    return ELF::R_PPU_SUB16;
  case FK_Data_Sub_4:
    return ELF::R_PPU_SUB32;
  case FK_Data_Sub_8:
    return ELF::R_PPU_SUB64;
  case FK_Data_Sub_6b:
    return ELF::R_PPU_SUB6;
  case PPU::fixup_ppu_hi20:
    return ELF::R_PPU_HI20;
  case PPU::fixup_ppu_lo12_i:
    return ELF::R_PPU_LO12_I;
  case PPU::fixup_ppu_lo12_s:
    return ELF::R_PPU_LO12_S;
  case PPU::fixup_ppu_tprel_hi20:
    return ELF::R_PPU_TPREL_HI20;
  case PPU::fixup_ppu_tprel_lo12_i:
    return ELF::R_PPU_TPREL_LO12_I;
  case PPU::fixup_ppu_tprel_lo12_s:
    return ELF::R_PPU_TPREL_LO12_S;
  case PPU::fixup_ppu_tprel_add:
    return ELF::R_PPU_TPREL_ADD;
  case PPU::fixup_ppu_relax:
    return ELF::R_PPU_RELAX;
  case PPU::fixup_ppu_align:
    return ELF::R_PPU_ALIGN;
  // TODO schi these 4 case is from AMD 
  /*
  case FK_PCRel_4:
    return ELF::R_PPU_REL32;
  case FK_Data_4:
  case FK_SecRel_4:
    return ELF::R_PPU_ABS32;
  case FK_Data_8:
    return ELF::R_PPU_ABS64;
    */
  case FK_SecRel_4:
    return ELF::R_PPU_ABS32;
  }
}

std::unique_ptr<MCObjectTargetWriter>
llvm::createPPUELFObjectWriter(uint8_t OSABI, bool Is64Bit) {
  return std::make_unique<PPUELFObjectWriter>(OSABI, Is64Bit);
}
