//===-- PPUMCExpr.h - PPU specific MC expression classes ----*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file describes PPU-specific MCExprs, used for modifiers like
// "%hi" or "%lo" etc.,
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_PPU_MCTARGETDESC_PPUMCEXPR_H
#define LLVM_LIB_TARGET_PPU_MCTARGETDESC_PPUMCEXPR_H

#include "llvm/MC/MCExpr.h"

namespace llvm {

class StringRef;
class MCOperand;
class PPUMCExpr : public MCTargetExpr {
public:
  enum VariantKind {
    VK_PPU_None,
    VK_PPU_LO,
    VK_PPU_HI,
    VK_PPU_PCREL_LO,
    VK_PPU_PCREL_HI,
    VK_PPU_GOT_HI,
    VK_PPU_TPREL_LO,
    VK_PPU_TPREL_HI,
    VK_PPU_TPREL_ADD,
    VK_PPU_TLS_GOT_HI,
    VK_PPU_TLS_GD_HI,
    VK_PPU_CALL,
    VK_PPU_CALL_PLT,
    VK_PPU_32_PCREL,
    VK_PPU_Invalid
  };

private:
  const MCExpr *Expr;
  const VariantKind Kind;

  int64_t evaluateAsInt64(int64_t Value) const;

  bool evaluatePCRelLo(MCValue &Res, const MCAsmLayout *Layout,
                       const MCFixup *Fixup) const;

  explicit PPUMCExpr(const MCExpr *Expr, VariantKind Kind)
      : Expr(Expr), Kind(Kind) {}

public:
  static const PPUMCExpr *create(const MCExpr *Expr, VariantKind Kind,
                                   MCContext &Ctx);

  VariantKind getKind() const { return Kind; }

  const MCExpr *getSubExpr() const { return Expr; }

  /// Get the corresponding PC-relative HI fixup that a VK_PPU_PCREL_LO
  /// points to.
  ///
  /// \returns nullptr if this isn't a VK_PPU_PCREL_LO pointing to a
  /// known PC-relative HI fixup.
  const MCFixup *getPCRelHiFixup() const;

  void printImpl(raw_ostream &OS, const MCAsmInfo *MAI) const override;
  bool evaluateAsRelocatableImpl(MCValue &Res, const MCAsmLayout *Layout,
                                 const MCFixup *Fixup) const override;
  void visitUsedExpr(MCStreamer &Streamer) const override;
  MCFragment *findAssociatedFragment() const override {
    return getSubExpr()->findAssociatedFragment();
  }

  void fixELFSymbolsInTLSFixups(MCAssembler &Asm) const override;

  bool evaluateAsConstant(int64_t &Res) const;

  static bool classof(const MCExpr *E) {
    return E->getKind() == MCExpr::Target;
  }

  static bool classof(const PPUMCExpr *) { return true; }

  static VariantKind getVariantKindForName(StringRef name);
  static StringRef getVariantKindName(VariantKind Kind);
};

} // end namespace llvm.

#endif
