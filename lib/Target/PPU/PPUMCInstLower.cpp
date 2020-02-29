//===-- PPUMCInstLower.cpp - Convert PPU MachineInstr to an MCInst ------=//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains code to lower PPU MachineInstrs to their corresponding
// MCInst records.
//
//===----------------------------------------------------------------------===//

#include "PPU.h"
#include "PPUInstrInfo.h"
#include "PPUMCInstLower.h"
#include "MCTargetDesc/PPUMCExpr.h"
#include "llvm/CodeGen/AsmPrinter.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/MC/MCAsmInfo.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"

using namespace llvm;

// FIXME PPU need to merge Flag Kind with AMD getVariantKind
MCOperand PPUMCInstLower::lowerSymbolOperand(const MachineOperand &MO, MCSymbol *Sym) const {
  MCContext &Ctx = AP.OutContext;
  PPUMCExpr::VariantKind Kind;

  switch (MO.getTargetFlags()) {
  default:
    // llvm_unreachable("Unknown target flag on GV operand");
    // FIXME I merge AMD here
    return MCOperand::createExpr(
                getLongBranchBlockExpr(*MO.getParent()->getParent(), MO));
  case PPUII::MO_None:
    Kind = PPUMCExpr::VK_PPU_None;
    break;
  case PPUII::MO_CALL:
    Kind = PPUMCExpr::VK_PPU_CALL;
    break;
  case PPUII::MO_PLT:
    Kind = PPUMCExpr::VK_PPU_CALL_PLT;
    break;
  case PPUII::MO_LO:
    Kind = PPUMCExpr::VK_PPU_LO;
    break;
  case PPUII::MO_HI:
    Kind = PPUMCExpr::VK_PPU_HI;
    break;
  case PPUII::MO_PCREL_LO:
    Kind = PPUMCExpr::VK_PPU_PCREL_LO;
    break;
  case PPUII::MO_PCREL_HI:
    Kind = PPUMCExpr::VK_PPU_PCREL_HI;
    break;
  case PPUII::MO_GOT_HI:
    Kind = PPUMCExpr::VK_PPU_GOT_HI;
    break;
  case PPUII::MO_TPREL_LO:
    Kind = PPUMCExpr::VK_PPU_TPREL_LO;
    break;
  case PPUII::MO_TPREL_HI:
    Kind = PPUMCExpr::VK_PPU_TPREL_HI;
    break;
  case PPUII::MO_TPREL_ADD:
    Kind = PPUMCExpr::VK_PPU_TPREL_ADD;
    break;
  case PPUII::MO_TLS_GOT_HI:
    Kind = PPUMCExpr::VK_PPU_TLS_GOT_HI;
    break;
  case PPUII::MO_TLS_GD_HI:
    Kind = PPUMCExpr::VK_PPU_TLS_GD_HI;
    break;
  }

  const MCExpr *ME =
      MCSymbolRefExpr::create(Sym, MCSymbolRefExpr::VK_None, Ctx);

  if (!MO.isJTI() && !MO.isMBB() && MO.getOffset())
    ME = MCBinaryExpr::createAdd(
        ME, MCConstantExpr::create(MO.getOffset(), Ctx), Ctx);

  if (Kind != PPUMCExpr::VK_PPU_None)
    ME = PPUMCExpr::create(ME, Kind, Ctx);
  return MCOperand::createExpr(ME);
}

// from AMD
PPUMCInstLower::PPUMCInstLower(MCContext &ctx,
                                     const TargetSubtargetInfo &st,
                                     const AsmPrinter &ap):
  Ctx(ctx), ST(st), AP(ap) { }

static MCSymbolRefExpr::VariantKind getVariantKind(unsigned MOFlags) {
  switch (MOFlags) {
  default:
    return MCSymbolRefExpr::VK_None;
  case PPUInstrInfo::MO_GOTPCREL:
    return MCSymbolRefExpr::VK_GOTPCREL;
  case PPUInstrInfo::MO_GOTPCREL32_LO:
    return MCSymbolRefExpr::VK_AMDGPU_GOTPCREL32_LO;
  case PPUInstrInfo::MO_GOTPCREL32_HI:
    return MCSymbolRefExpr::VK_AMDGPU_GOTPCREL32_HI;
  case PPUInstrInfo::MO_REL32_LO:
    return MCSymbolRefExpr::VK_AMDGPU_REL32_LO;
  case PPUInstrInfo::MO_REL32_HI:
    return MCSymbolRefExpr::VK_AMDGPU_REL32_HI;
  case PPUInstrInfo::MO_ABS32_LO:
    return MCSymbolRefExpr::VK_AMDGPU_ABS32_LO;
  case PPUInstrInfo::MO_ABS32_HI:
    return MCSymbolRefExpr::VK_AMDGPU_ABS32_HI;
  }
}

// from AMD
const MCExpr *PPUMCInstLower::getLongBranchBlockExpr(
  const MachineBasicBlock &SrcBB,
  const MachineOperand &MO) const {
  const MCExpr *DestBBSym = MCSymbolRefExpr::create(MO.getMBB()->getSymbol(), Ctx);
  const MCExpr *SrcBBSym = MCSymbolRefExpr::create(SrcBB.getSymbol(), Ctx);

  // FIXME: The first half of this assert should be removed. This should
  // probably be PC relative instead of using the source block symbol, and
  // therefore the indirect branch expansion should use a bundle.
  assert(
      skipDebugInstructionsForward(SrcBB.begin(), SrcBB.end())->getOpcode() ==
          PPU::S_GETPC_B64 &&
      ST.getInstrInfo()->get(PPU::S_GETPC_B64).Size == 4);

  // s_getpc_b64 returns the address of next instruction.
  const MCConstantExpr *One = MCConstantExpr::create(4, Ctx);
  SrcBBSym = MCBinaryExpr::createAdd(SrcBBSym, One, Ctx);

  if (MO.getTargetFlags() == PPUInstrInfo::MO_LONG_BRANCH_FORWARD)
    return MCBinaryExpr::createSub(DestBBSym, SrcBBSym, Ctx);

  assert(MO.getTargetFlags() == PPUInstrInfo::MO_LONG_BRANCH_BACKWARD);
  return MCBinaryExpr::createSub(SrcBBSym, DestBBSym, Ctx);
}

bool PPUMCInstLower::lowerOperand(const MachineOperand &MO,
                                     MCOperand &MCOp)  const {
  switch (MO.getType()) {
  default:
    llvm_unreachable("PPUMCInstLower: unknown operand type");
  case MachineOperand::MO_Immediate:
    MCOp = MCOperand::createImm(MO.getImm());
    return true;
  case MachineOperand::MO_Register:
    // Ignore all implicit register operands. TODO PPU schi add from RISCV
    if (MO.isImplicit())
      return false;
    MCOp = MCOperand::createReg(PPU::getMCReg(MO.getReg(), ST));
    return true;
  case MachineOperand::MO_MachineBasicBlock: {
    MCOp = lowerSymbolOperand(MO, MO.getMBB()->getSymbol());
    /* TODO merged with RISCV
    if (MO.getTargetFlags() != 0) {
      MCOp = MCOperand::createExpr(
        getLongBranchBlockExpr(*MO.getParent()->getParent(), MO));
    } else {
      MCOp = MCOperand::createExpr(
        MCSymbolRefExpr::create(MO.getMBB()->getSymbol(), Ctx));
    }
    */

    return true;
  }
  case MachineOperand::MO_GlobalAddress: {
/* TODO PPU is below AMD is equal to RISCV
    const GlobalValue *GV = MO.getGlobal();
    SmallString<128> SymbolName;
    AP.getNameWithPrefix(SymbolName, GV);
    MCSymbol *Sym = Ctx.getOrCreateSymbol(SymbolName);
    const MCExpr *Expr =
      MCSymbolRefExpr::create(Sym, getVariantKind(MO.getTargetFlags()),Ctx);
    int64_t Offset = MO.getOffset();
    if (Offset != 0) {
      Expr = MCBinaryExpr::createAdd(Expr,
                                     MCConstantExpr::create(Offset, Ctx), Ctx);
    }
    MCOp = MCOperand::createExpr(Expr);
    */
    MCOp = lowerSymbolOperand(MO, AP.getSymbol(MO.getGlobal()));
    return true;
  }
  case MachineOperand::MO_ExternalSymbol: {
    MCSymbol *Sym = Ctx.getOrCreateSymbol(StringRef(MO.getSymbolName()));
    Sym->setExternal(true);
    const MCSymbolRefExpr *Expr = MCSymbolRefExpr::create(Sym, Ctx);
    MCOp = MCOperand::createExpr(Expr);
/* this is RISCV
    MCOp = lowerSymbolOperand(
        MO, AP.GetExternalSymbolSymbol(MO.getSymbolName()), AP);
        */
    return true;
  }
  case MachineOperand::MO_RegisterMask:
    // Regmasks are like implicit defs.
    return false;
  case MachineOperand::MO_ConstantPoolIndex:
    MCOp = lowerSymbolOperand(MO, AP.GetCPISymbol(MO.getIndex()));
    break;
  }
}
/* TODO Use AMD lowerOperand instead
bool llvm::LowerPPUMachineOperandToMCOperand(const MachineOperand &MO,
                                               MCOperand &MCOp,
                                               const AsmPrinter &AP) {
  switch (MO.getType()) {
  default:
    report_fatal_error("LowerPPUMachineInstrToMCInst: unknown operand type");
  case MachineOperand::MO_Register:
    // Ignore all implicit register operands.
    if (MO.isImplicit())
      return false;
    MCOp = MCOperand::createReg(MO.getReg());
    break;
  case MachineOperand::MO_RegisterMask:
    // Regmasks are like implicit defs.
    return false;
  case MachineOperand::MO_Immediate:
    MCOp = MCOperand::createImm(MO.getImm());
    break;
  case MachineOperand::MO_MachineBasicBlock:
    MCOp = lowerSymbolOperand(MO, MO.getMBB()->getSymbol(), AP);
    break;
  case MachineOperand::MO_GlobalAddress:
    MCOp = lowerSymbolOperand(MO, AP.getSymbol(MO.getGlobal()), AP);
    break;
  case MachineOperand::MO_BlockAddress:
    MCOp = lowerSymbolOperand(
        MO, AP.GetBlockAddressSymbol(MO.getBlockAddress()), AP);
    break;
  case MachineOperand::MO_ExternalSymbol:
    MCOp = lowerSymbolOperand(
        MO, AP.GetExternalSymbolSymbol(MO.getSymbolName()), AP);
    break;
  case MachineOperand::MO_ConstantPoolIndex:
    MCOp = lowerSymbolOperand(MO, AP.GetCPISymbol(MO.getIndex()), AP);
    break;
  }
  return true;
}
*/

void PPUMCInstLower::lower(const MachineInstr *MI, MCInst &OutMI) const {
  unsigned Opcode = MI->getOpcode();
  const auto *TII = static_cast<const PPUInstrInfo*>(ST.getInstrInfo());

  // FIXME: Should be able to handle this with emitPseudoExpansionLowering. We
  // need to select it to the subtarget specific version, and there's no way to
  // do that with a single pseudo source operation.
  if (Opcode == PPU::S_SETPC_B64_return)
    Opcode = PPU::S_SETPC_B64;
  else if (Opcode == PPU::SI_CALL) {
    // SI_CALL is just S_SWAPPC_B64 with an additional operand to track the
    // called function (which we need to remove here).
    OutMI.setOpcode(TII->pseudoToMCOpcode(PPU::S_SWAPPC_B64));
    MCOperand Dest, Src;
    lowerOperand(MI->getOperand(0), Dest);
    lowerOperand(MI->getOperand(1), Src);
    OutMI.addOperand(Dest);
    OutMI.addOperand(Src);
    return;
  } else if (Opcode == PPU::SI_TCRETURN) {
    // TODO: How to use branch immediate and avoid register+add?
    Opcode = PPU::S_SETPC_B64;
  }

  int MCOpcode = TII->pseudoToMCOpcode(Opcode);
  if (MCOpcode == -1) {
    LLVMContext &C = MI->getParent()->getParent()->getFunction().getContext();
    C.emitError("PPUMCInstLower::lower - Pseudo instruction doesn't have "
                "a target-specific version: " + Twine(MI->getOpcode()));
  }

  OutMI.setOpcode(MCOpcode);

  for (const MachineOperand &MO : MI->explicit_operands()) {
    MCOperand MCOp;
    lowerOperand(MO, MCOp);
    OutMI.addOperand(MCOp);
  }
}
/*
void llvm::LowerPPUMachineInstrToMCInst(const MachineInstr *MI, MCInst &OutMI,
                                          const AsmPrinter &AP) {
  OutMI.setOpcode(MI->getOpcode());

  for (const MachineOperand &MO : MI->operands()) {
    MCOperand MCOp;
    if (LowerPPUMachineOperandToMCOperand(MO, MCOp, AP))
      OutMI.addOperand(MCOp);
  }
}
*/
