//===-- PPUInstPrinter.cpp - Convert PPU MCInst to asm syntax ---------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This class prints an PPU MCInst to a .s file.
//
//===----------------------------------------------------------------------===//

#include "PPUInstPrinter.h"
#include "PPUDefines.h"
#include "MCTargetDesc/PPUMCExpr.h"
#include "MCTargetDesc/PPUMCTargetDesc.h"
#include "Utils/PPUAsmUtils.h"
#include "Utils/PPUBaseInfo.h"
#include "llvm/MC/MCAsmInfo.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCInstrDesc.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/MC/MCSymbol.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/FormattedStream.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/Support/raw_ostream.h"
using namespace llvm;

#define DEBUG_TYPE "asm-printer"

// Include the auto-generated portion of the assembly writer.
#define PRINT_ALIAS_INSTR
#include "PPUGenAsmWriter.inc"

// Include the auto-generated portion of the compress emitter.
// #define GEN_UNCOMPRESS_INSTR
// #include "PPUGenCompressInstEmitter.inc"

static cl::opt<bool>
    NoAliases("ppu-no-aliases",
              cl::desc("Disable the emission of assembler pseudo instructions"),
              cl::init(false), cl::Hidden);

void PPUInstPrinter::printInst(const MCInst *MI, raw_ostream &O,
                                 StringRef Annot, const MCSubtargetInfo &STI) {
  O.flush();
  bool Res = false;
  const MCInst *NewMI = MI;
  MCInst UncompressedMI;
  /* TODO uncompress
  if (!NoAliases)
    Res = uncompressInst(UncompressedMI, *MI, MRI, STI);
  if (Res)
    NewMI = const_cast<MCInst *>(&UncompressedMI);
    */
  if (NoAliases || !printAliasInstr(NewMI, STI, O))
    printInstruction(NewMI, STI, O);
  printAnnotation(O, Annot);
}

void PPUInstPrinter::printRegName(raw_ostream &O, unsigned RegNo) const {
  O << getRegisterName(RegNo);
}

    /* old RISCV
void PPUInstPrinter::printOperand(const MCInst *MI, unsigned OpNo,
                                    const MCSubtargetInfo &STI, raw_ostream &O,
                                    const char *Modifier) {
  assert((Modifier == 0 || Modifier[0] == 0) && "No modifiers supported");
  const MCOperand &MO = MI->getOperand(OpNo);

  if (MO.isReg()) {
    printRegName(O, MO.getReg());
    return;
  }

  if (MO.isImm()) {
    O << MO.getImm();
    return;
  }

  assert(MO.isExpr() && "Unknown operand kind in printOperand");
  MO.getExpr()->print(O, &MAI);
}
*/

void PPUInstPrinter::printCSRSystemRegister(const MCInst *MI, unsigned OpNo,
                                              const MCSubtargetInfo &STI,
                                              raw_ostream &O) {
  unsigned Imm = MI->getOperand(OpNo).getImm();
  auto SysReg = PPUSysReg::lookupSysRegByEncoding(Imm);
  if (SysReg && SysReg->haveRequiredFeatures(STI.getFeatureBits()))
    O << SysReg->Name;
  else
    O << Imm;
}

void PPUInstPrinter::printFenceArg(const MCInst *MI, unsigned OpNo,
                                     const MCSubtargetInfo &STI,
                                     raw_ostream &O) {
  unsigned FenceArg = MI->getOperand(OpNo).getImm();
  assert (((FenceArg >> 4) == 0) && "Invalid immediate in printFenceArg");

  if ((FenceArg & PPUFenceField::I) != 0)
    O << 'i';
  if ((FenceArg & PPUFenceField::O) != 0)
    O << 'o';
  if ((FenceArg & PPUFenceField::R) != 0)
    O << 'r';
  if ((FenceArg & PPUFenceField::W) != 0)
    O << 'w';
  if (FenceArg == 0)
    O << "unknown";
}

void PPUInstPrinter::printFRMArg(const MCInst *MI, unsigned OpNo,
                                   const MCSubtargetInfo &STI, raw_ostream &O) {
  auto FRMArg =
      static_cast<PPUFPRndMode::RoundingMode>(MI->getOperand(OpNo).getImm());
  O << PPUFPRndMode::roundingModeToString(FRMArg);
}

void PPUInstPrinter::printAtomicMemOp(const MCInst *MI, unsigned OpNo,
                                        const MCSubtargetInfo &STI,
                                        raw_ostream &O) {
  const MCOperand &MO = MI->getOperand(OpNo);

  assert(MO.isReg() && "printAtomicMemOp can only print register operands");
  O << "(";
  printRegName(O, MO.getReg());
  O << ")";
  return;
}

// below from AMD
void PPUInstPrinter::printU4ImmOperand(const MCInst *MI, unsigned OpNo,
                                          const MCSubtargetInfo &STI,
                                          raw_ostream &O) {
  O << formatHex(MI->getOperand(OpNo).getImm() & 0xf);
}

void PPUInstPrinter::printU8ImmOperand(const MCInst *MI, unsigned OpNo,
                                          raw_ostream &O) {
  O << formatHex(MI->getOperand(OpNo).getImm() & 0xff);
}

void PPUInstPrinter::printU16ImmOperand(const MCInst *MI, unsigned OpNo,
                                           const MCSubtargetInfo &STI,
                                           raw_ostream &O) {
  // It's possible to end up with a 32-bit literal used with a 16-bit operand
  // with ignored high bits. Print as 32-bit anyway in that case.
  int64_t Imm = MI->getOperand(OpNo).getImm();
  if (isInt<16>(Imm) || isUInt<16>(Imm))
    O << formatHex(static_cast<uint64_t>(Imm & 0xffff));
  else
    printU32ImmOperand(MI, OpNo, STI, O);
}

void PPUInstPrinter::printU4ImmDecOperand(const MCInst *MI, unsigned OpNo,
                                             raw_ostream &O) {
  O << formatDec(MI->getOperand(OpNo).getImm() & 0xf);
}

void PPUInstPrinter::printU8ImmDecOperand(const MCInst *MI, unsigned OpNo,
                                             raw_ostream &O) {
  O << formatDec(MI->getOperand(OpNo).getImm() & 0xff);
}

void PPUInstPrinter::printU16ImmDecOperand(const MCInst *MI, unsigned OpNo,
                                              raw_ostream &O) {
  O << formatDec(MI->getOperand(OpNo).getImm() & 0xffff);
}

void PPUInstPrinter::printU32ImmOperand(const MCInst *MI, unsigned OpNo,
                                           const MCSubtargetInfo &STI,
                                           raw_ostream &O) {
  O << formatHex(MI->getOperand(OpNo).getImm() & 0xffffffff);
}

void PPUInstPrinter::printNamedBit(const MCInst *MI, unsigned OpNo,
                                      raw_ostream &O, StringRef BitName) {
  if (MI->getOperand(OpNo).getImm()) {
    O << ' ' << BitName;
  }
}

void PPUInstPrinter::printOffen(const MCInst *MI, unsigned OpNo,
                                   raw_ostream &O) {
  printNamedBit(MI, OpNo, O, "offen");
}

void PPUInstPrinter::printIdxen(const MCInst *MI, unsigned OpNo,
                                   raw_ostream &O) {
  printNamedBit(MI, OpNo, O, "idxen");
}

void PPUInstPrinter::printAddr64(const MCInst *MI, unsigned OpNo,
                                    raw_ostream &O) {
  printNamedBit(MI, OpNo, O, "addr64");
}

void PPUInstPrinter::printMBUFOffset(const MCInst *MI, unsigned OpNo,
                                        raw_ostream &O) {
  if (MI->getOperand(OpNo).getImm()) {
    O << " offset:";
    printU16ImmDecOperand(MI, OpNo, O);
  }
}

void PPUInstPrinter::printOffset(const MCInst *MI, unsigned OpNo,
                                    const MCSubtargetInfo &STI,
                                    raw_ostream &O) {
  uint16_t Imm = MI->getOperand(OpNo).getImm();
  if (Imm != 0) {
    O << ((OpNo == 0)? "offset:" : " offset:");
    printU16ImmDecOperand(MI, OpNo, O);
  }
}

void PPUInstPrinter::printFlatOffset(const MCInst *MI, unsigned OpNo,
                                        const MCSubtargetInfo &STI,
                                        raw_ostream &O) {
    llvm_unreachable("printFlatOffset is WIP");
    /*
  uint16_t Imm = MI->getOperand(OpNo).getImm();
  if (Imm != 0) {
    O << ((OpNo == 0)? "offset:" : " offset:");

    const MCInstrDesc &Desc = MII.get(MI->getOpcode());
    bool IsFlatSeg = !(Desc.TSFlags & PPUInstrFlags::IsNonFlatSeg);

    if (IsFlatSeg) { // Unsigned offset
      printU16ImmDecOperand(MI, OpNo, O);
    } else {         // Signed offset
      if (PPU::isGFX10(STI)) {
        O << formatDec(SignExtend32<12>(MI->getOperand(OpNo).getImm()));
      } else {
        O << formatDec(SignExtend32<13>(MI->getOperand(OpNo).getImm()));
      }
    }
  }
  */
}

void PPUInstPrinter::printOffset0(const MCInst *MI, unsigned OpNo,
                                     const MCSubtargetInfo &STI,
                                     raw_ostream &O) {
  if (MI->getOperand(OpNo).getImm()) {
    O << " offset0:";
    printU8ImmDecOperand(MI, OpNo, O);
  }
}

void PPUInstPrinter::printOffset1(const MCInst *MI, unsigned OpNo,
                                     const MCSubtargetInfo &STI,
                                     raw_ostream &O) {
  if (MI->getOperand(OpNo).getImm()) {
    O << " offset1:";
    printU8ImmDecOperand(MI, OpNo, O);
  }
}

void PPUInstPrinter::printSMRDOffset8(const MCInst *MI, unsigned OpNo,
                                        const MCSubtargetInfo &STI,
                                        raw_ostream &O) {
  printU32ImmOperand(MI, OpNo, STI, O);
}

void PPUInstPrinter::printSMRDOffset20(const MCInst *MI, unsigned OpNo,
                                        const MCSubtargetInfo &STI,
                                        raw_ostream &O) {
  printU32ImmOperand(MI, OpNo, STI, O);
}

void PPUInstPrinter::printSMRDLiteralOffset(const MCInst *MI, unsigned OpNo,
                                               const MCSubtargetInfo &STI,
                                               raw_ostream &O) {
  printU32ImmOperand(MI, OpNo, STI, O);
}

void PPUInstPrinter::printGDS(const MCInst *MI, unsigned OpNo,
                                 const MCSubtargetInfo &STI, raw_ostream &O) {
  printNamedBit(MI, OpNo, O, "gds");
}

void PPUInstPrinter::printDLC(const MCInst *MI, unsigned OpNo,
                                 const MCSubtargetInfo &STI, raw_ostream &O) {
  //if (PPU::isGFX10(STI))
    printNamedBit(MI, OpNo, O, "dlc");
}

void PPUInstPrinter::printGLC(const MCInst *MI, unsigned OpNo,
                                 const MCSubtargetInfo &STI, raw_ostream &O) {
  printNamedBit(MI, OpNo, O, "glc");
}

void PPUInstPrinter::printSLC(const MCInst *MI, unsigned OpNo,
                                 const MCSubtargetInfo &STI, raw_ostream &O) {
  printNamedBit(MI, OpNo, O, "slc");
}

void PPUInstPrinter::printTFE(const MCInst *MI, unsigned OpNo,
                                 const MCSubtargetInfo &STI, raw_ostream &O) {
  printNamedBit(MI, OpNo, O, "tfe");
}

void PPUInstPrinter::printDMask(const MCInst *MI, unsigned OpNo,
                                   const MCSubtargetInfo &STI, raw_ostream &O) {
  if (MI->getOperand(OpNo).getImm()) {
    O << " dmask:";
    printU16ImmOperand(MI, OpNo, STI, O);
  }
}
/*
void PPUInstPrinter::printDim(const MCInst *MI, unsigned OpNo,
                                 const MCSubtargetInfo &STI, raw_ostream &O) {
  unsigned Dim = MI->getOperand(OpNo).getImm();
  O << " dim:SQ_RSRC_IMG_";

  const PPU::MIMGDimInfo *DimInfo = PPU::getMIMGDimInfoByEncoding(Dim);
  if (DimInfo)
    O << DimInfo->AsmSuffix;
  else
    O << Dim;
}
*/

void PPUInstPrinter::printUNorm(const MCInst *MI, unsigned OpNo,
                                   const MCSubtargetInfo &STI, raw_ostream &O) {
  printNamedBit(MI, OpNo, O, "unorm");
}

void PPUInstPrinter::printDA(const MCInst *MI, unsigned OpNo,
                                const MCSubtargetInfo &STI, raw_ostream &O) {
  printNamedBit(MI, OpNo, O, "da");
}

/*
void PPUInstPrinter::printR128A16(const MCInst *MI, unsigned OpNo,
                                  const MCSubtargetInfo &STI, raw_ostream &O) {
  if (STI.hasFeature(PPU::FeatureR128A16))
    printNamedBit(MI, OpNo, O, "a16");
  else
    printNamedBit(MI, OpNo, O, "r128");
}
*/

void PPUInstPrinter::printLWE(const MCInst *MI, unsigned OpNo,
                                 const MCSubtargetInfo &STI, raw_ostream &O) {
  printNamedBit(MI, OpNo, O, "lwe");
}

void PPUInstPrinter::printD16(const MCInst *MI, unsigned OpNo,
                                 const MCSubtargetInfo &STI, raw_ostream &O) {
  printNamedBit(MI, OpNo, O, "d16");
}

void PPUInstPrinter::printExpCompr(const MCInst *MI, unsigned OpNo,
                                      const MCSubtargetInfo &STI,
                                      raw_ostream &O) {
  if (MI->getOperand(OpNo).getImm())
    O << " compr";
}

void PPUInstPrinter::printExpVM(const MCInst *MI, unsigned OpNo,
                                   const MCSubtargetInfo &STI,
                                   raw_ostream &O) {
  if (MI->getOperand(OpNo).getImm())
    O << " vm";
}

void PPUInstPrinter::printFORMAT(const MCInst *MI, unsigned OpNo,
                                    const MCSubtargetInfo &STI,
                                    raw_ostream &O) {
  if (unsigned Val = MI->getOperand(OpNo).getImm()) {
    // if (PPU::isGFX10(STI))
      O << " format:" << Val;
    /*
    else {
      O << " dfmt:" << (Val & 15);
      O << ", nfmt:" << (Val >> 4);
    }
    */
  }
}

void PPUInstPrinter::printRegOperand(unsigned RegNo, raw_ostream &O,
                                        const MCRegisterInfo &MRI) {
#if !defined(NDEBUG)
  switch (RegNo) {
  case PPU::FP_REG:
  case PPU::SP_REG:
  case PPU::SCRATCH_WAVE_OFFSET_REG:
  case PPU::PRIVATE_RSRC_REG:
    llvm_unreachable("pseudo-register should not ever be emitted");
  case PPU::SCC:
    llvm_unreachable("pseudo scc should not ever be emitted");
  default:
    break;
  }
#endif

  O << getRegisterName(RegNo, PPU::NoRegAltName);
}

void PPUInstPrinter::printVOPDst(const MCInst *MI, unsigned OpNo,
                                    const MCSubtargetInfo &STI, raw_ostream &O) {
  if (OpNo == 0) {
    if (MII.get(MI->getOpcode()).TSFlags & PPUInstrFlags::VOP3)
      O << "_e64 ";
    else if (MII.get(MI->getOpcode()).TSFlags & PPUInstrFlags::DPP)
      O << "_dpp ";
    else if (MII.get(MI->getOpcode()).TSFlags & PPUInstrFlags::SDWA)
      O << "_sdwa ";
    else
      O << "_e32 ";
  }

  printOperand(MI, OpNo, STI, O);

  // Print default vcc/vcc_lo operand.
  switch (MI->getOpcode()) {
  default: break;

  case PPU::V_ADD_CO_CI_U32_e32_ppu:
  case PPU::V_SUB_CO_CI_U32_e32_ppu:
  case PPU::V_SUBREV_CO_CI_U32_e32_ppu:
  /*
  case PPU::V_ADD_CO_CI_U32_sdwa_ppu:
  case PPU::V_SUB_CO_CI_U32_sdwa_ppu:
  case PPU::V_SUBREV_CO_CI_U32_sdwa_ppu:
  case PPU::V_ADD_CO_CI_U32_dpp_gfx10:
  case PPU::V_SUB_CO_CI_U32_dpp_gfx10:
  case PPU::V_SUBREV_CO_CI_U32_dpp_gfx10:
  case PPU::V_ADD_CO_CI_U32_dpp8_gfx10:
  case PPU::V_SUB_CO_CI_U32_dpp8_gfx10:
  case PPU::V_SUBREV_CO_CI_U32_dpp8_gfx10:
  */
    printDefaultVccOperand(1, STI, O);
    break;
  }
}
/*
void PPUInstPrinter::printVINTRPDst(const MCInst *MI, unsigned OpNo,
                                       const MCSubtargetInfo &STI, raw_ostream &O) {
  if (PPU::isSI(STI) || PPU::isCI(STI))
    O << " ";
  else
    O << "_e32 ";

  printOperand(MI, OpNo, STI, O);
}
*/

void PPUInstPrinter::printImmediate16(uint32_t Imm,
                                         const MCSubtargetInfo &STI,
                                         raw_ostream &O) {
  int16_t SImm = static_cast<int16_t>(Imm);
  if (SImm >= -16 && SImm <= 64) {
    O << SImm;
    return;
  }

  if (Imm == 0x3C00)
    O<< "1.0";
  else if (Imm == 0xBC00)
    O<< "-1.0";
  else if (Imm == 0x3800)
    O<< "0.5";
  else if (Imm == 0xB800)
    O<< "-0.5";
  else if (Imm == 0x4000)
    O<< "2.0";
  else if (Imm == 0xC000)
    O<< "-2.0";
  else if (Imm == 0x4400)
    O<< "4.0";
  else if (Imm == 0xC400)
    O<< "-4.0";
  else if (Imm == 0x3118) {
    assert(STI.getFeatureBits()[PPU::FeatureInv2PiInlineImm]);
    O << "0.15915494";
  } else
    O << formatHex(static_cast<uint64_t>(Imm));
}

void PPUInstPrinter::printImmediateV216(uint32_t Imm,
                                           const MCSubtargetInfo &STI,
                                           raw_ostream &O) {
  uint16_t Lo16 = static_cast<uint16_t>(Imm);
  printImmediate16(Lo16, STI, O);
}

void PPUInstPrinter::printImmediate32(uint32_t Imm,
                                         const MCSubtargetInfo &STI,
                                         raw_ostream &O) {
  int32_t SImm = static_cast<int32_t>(Imm);
  if (SImm >= -16 && SImm <= 64) {
    O << SImm;
    return;
  }

  if (Imm == FloatToBits(0.0f))
    O << "0.0";
  else if (Imm == FloatToBits(1.0f))
    O << "1.0";
  else if (Imm == FloatToBits(-1.0f))
    O << "-1.0";
  else if (Imm == FloatToBits(0.5f))
    O << "0.5";
  else if (Imm == FloatToBits(-0.5f))
    O << "-0.5";
  else if (Imm == FloatToBits(2.0f))
    O << "2.0";
  else if (Imm == FloatToBits(-2.0f))
    O << "-2.0";
  else if (Imm == FloatToBits(4.0f))
    O << "4.0";
  else if (Imm == FloatToBits(-4.0f))
    O << "-4.0";
  else if (Imm == 0x3e22f983 &&
           STI.getFeatureBits()[PPU::FeatureInv2PiInlineImm])
    O << "0.15915494";
  else
    O << formatHex(static_cast<uint64_t>(Imm));
}

void PPUInstPrinter::printImmediate64(uint64_t Imm,
                                         const MCSubtargetInfo &STI,
                                         raw_ostream &O) {
  int64_t SImm = static_cast<int64_t>(Imm);
  if (SImm >= -16 && SImm <= 64) {
    O << SImm;
    return;
  }

  if (Imm == DoubleToBits(0.0))
    O << "0.0";
  else if (Imm == DoubleToBits(1.0))
    O << "1.0";
  else if (Imm == DoubleToBits(-1.0))
    O << "-1.0";
  else if (Imm == DoubleToBits(0.5))
    O << "0.5";
  else if (Imm == DoubleToBits(-0.5))
    O << "-0.5";
  else if (Imm == DoubleToBits(2.0))
    O << "2.0";
  else if (Imm == DoubleToBits(-2.0))
    O << "-2.0";
  else if (Imm == DoubleToBits(4.0))
    O << "4.0";
  else if (Imm == DoubleToBits(-4.0))
    O << "-4.0";
  else if (Imm == 0x3fc45f306dc9c882 &&
           STI.getFeatureBits()[PPU::FeatureInv2PiInlineImm])
    O << "0.15915494309189532";
  else {
    assert(isUInt<32>(Imm) || Imm == 0x3fc45f306dc9c882);

    // In rare situations, we will have a 32-bit literal in a 64-bit
    // operand. This is technically allowed for the encoding of s_mov_b64.
    O << formatHex(static_cast<uint64_t>(Imm));
  }
}

void PPUInstPrinter::printBLGP(const MCInst *MI, unsigned OpNo,
                                  const MCSubtargetInfo &STI,
                                  raw_ostream &O) {
  unsigned Imm = MI->getOperand(OpNo).getImm();
  if (!Imm)
    return;

  O << " blgp:" << Imm;
}

void PPUInstPrinter::printCBSZ(const MCInst *MI, unsigned OpNo,
                                  const MCSubtargetInfo &STI,
                                  raw_ostream &O) {
  unsigned Imm = MI->getOperand(OpNo).getImm();
  if (!Imm)
    return;

  O << " cbsz:" << Imm;
}

void PPUInstPrinter::printABID(const MCInst *MI, unsigned OpNo,
                                  const MCSubtargetInfo &STI,
                                  raw_ostream &O) {
  unsigned Imm = MI->getOperand(OpNo).getImm();
  if (!Imm)
    return;

  O << " abid:" << Imm;
}

void PPUInstPrinter::printDefaultVccOperand(unsigned OpNo,
                                               const MCSubtargetInfo &STI,
                                               raw_ostream &O) {
  if (OpNo > 0)
    O << ", ";
  printRegOperand( PPU::VCC, O, MRI);
  if (OpNo == 0)
    O << ", ";
}

void PPUInstPrinter::printOperand(const MCInst *MI, unsigned OpNo,
                                     const MCSubtargetInfo &STI, raw_ostream &O,
                                    const char *Modifier) {
  // Print default vcc/vcc_lo operand of VOPC.
  const MCInstrDesc &Desc = MII.get(MI->getOpcode());
  if (OpNo == 0 && (Desc.TSFlags & PPUInstrFlags::VOPC) &&
      (Desc.hasImplicitDefOfPhysReg(PPU::VCC)))
    printDefaultVccOperand(OpNo, STI, O);

  if (OpNo >= MI->getNumOperands()) {
    O << "/*Missing OP" << OpNo << "*/";
    return;
  }

  const MCOperand &Op = MI->getOperand(OpNo);
  if (Op.isReg()) {
    printRegOperand(Op.getReg(), O, MRI);
  } else if (Op.isImm()) {
    switch (Desc.OpInfo[OpNo].OperandType) {
    case PPU::OPERAND_REG_IMM_INT32:
    case PPU::OPERAND_REG_IMM_FP32:
    case PPU::OPERAND_REG_INLINE_C_INT32:
    case PPU::OPERAND_REG_INLINE_C_FP32:
    case PPU::OPERAND_REG_INLINE_AC_INT32:
    case PPU::OPERAND_REG_INLINE_AC_FP32:
    case MCOI::OPERAND_IMMEDIATE:
      printImmediate32(Op.getImm(), STI, O);
      break;
    case PPU::OPERAND_REG_IMM_INT64:
    case PPU::OPERAND_REG_IMM_FP64:
    case PPU::OPERAND_REG_INLINE_C_INT64:
    case PPU::OPERAND_REG_INLINE_C_FP64:
      printImmediate64(Op.getImm(), STI, O);
      break;
    case PPU::OPERAND_REG_INLINE_C_INT16:
    case PPU::OPERAND_REG_INLINE_C_FP16:
    case PPU::OPERAND_REG_INLINE_AC_INT16:
    case PPU::OPERAND_REG_INLINE_AC_FP16:
    case PPU::OPERAND_REG_IMM_INT16:
    case PPU::OPERAND_REG_IMM_FP16:
      printImmediate16(Op.getImm(), STI, O);
      break;
    case PPU::OPERAND_REG_IMM_V2INT16:
    case PPU::OPERAND_REG_IMM_V2FP16:
      if (!isUInt<16>(Op.getImm()) &&
          STI.getFeatureBits()[PPU::FeatureVOP3Literal]) {
        printImmediate32(Op.getImm(), STI, O);
        break;
      }
      LLVM_FALLTHROUGH;
    case PPU::OPERAND_REG_INLINE_C_V2FP16:
    case PPU::OPERAND_REG_INLINE_C_V2INT16:
    case PPU::OPERAND_REG_INLINE_AC_V2FP16:
    case PPU::OPERAND_REG_INLINE_AC_V2INT16:
      printImmediateV216(Op.getImm(), STI, O);
      break;
    case MCOI::OPERAND_UNKNOWN:
    case MCOI::OPERAND_PCREL:
      O << formatDec(Op.getImm());
      break;
    case MCOI::OPERAND_REGISTER:
      // FIXME: This should be removed and handled somewhere else. Seems to come
      // from a disassembler bug.
      O << "/*invalid immediate*/";
      break;
    default:
      llvm_unreachable("just TODO: unexpected immediate operand type");
      const MCOperand &MO = MI->getOperand(OpNo);
      O << MO.getImm();
      // We hit this for the immediate instruction bits that don't yet have a
      // custom printer.
    }
  } else if (Op.isFPImm()) {
    // We special case 0.0 because otherwise it will be printed as an integer.
    if (Op.getFPImm() == 0.0)
      O << "0.0";
    else {
      const MCInstrDesc &Desc = MII.get(MI->getOpcode());
      int RCID = Desc.OpInfo[OpNo].RegClass;
      unsigned RCBits = PPU::getRegBitWidth(MRI.getRegClass(RCID));
      if (RCBits == 32)
        printImmediate32(FloatToBits(Op.getFPImm()), STI, O);
      else if (RCBits == 64)
        printImmediate64(DoubleToBits(Op.getFPImm()), STI, O);
      else
        llvm_unreachable("Invalid register class size");
    }
  } else if (Op.isExpr()) {
    const MCExpr *Exp = Op.getExpr();
    Exp->print(O, &MAI);
  } else {
    O << "/*INV_OP*/";
  }

  // Print default vcc/vcc_lo operand of v_cndmask_b32_e32.
  switch (MI->getOpcode()) {
  default: break;

  case PPU::V_CNDMASK_B32_e32_ppu:
  case PPU::V_ADD_CO_CI_U32_e32_ppu:
  case PPU::V_SUB_CO_CI_U32_e32_ppu:
  case PPU::V_SUBREV_CO_CI_U32_e32_ppu:
           /*
  case PPU::V_ADD_CO_CI_U32_dpp_gfx10:
  case PPU::V_SUB_CO_CI_U32_dpp_gfx10:
  case PPU::V_SUBREV_CO_CI_U32_dpp_gfx10:
  case PPU::V_ADD_CO_CI_U32_dpp8_gfx10:
  case PPU::V_SUB_CO_CI_U32_dpp8_gfx10:
  case PPU::V_SUBREV_CO_CI_U32_dpp8_gfx10:

  case PPU::V_CNDMASK_B32_e32_gfx6_gfx7:
  case PPU::V_CNDMASK_B32_e32_vi:
  */
    if ((int)OpNo == PPU::getNamedOperandIdx(MI->getOpcode(),
                                                PPU::OpName::src1))
      printDefaultVccOperand(OpNo, STI, O);
    break;
  }
}

void PPUInstPrinter::printOperandAndFPInputMods(const MCInst *MI,
                                                   unsigned OpNo,
                                                   const MCSubtargetInfo &STI,
                                                   raw_ostream &O) {
  unsigned InputModifiers = MI->getOperand(OpNo).getImm();

  // Use 'neg(...)' instead of '-' to avoid ambiguity.
  // This is important for integer literals because
  // -1 is not the same value as neg(1).
  bool NegMnemo = false;

  if (InputModifiers & PPUSrcMods::NEG) {
    if (OpNo + 1 < MI->getNumOperands() &&
        (InputModifiers & PPUSrcMods::ABS) == 0) {
      const MCOperand &Op = MI->getOperand(OpNo + 1);
      NegMnemo = Op.isImm() || Op.isFPImm();
    }
    if (NegMnemo) {
      O << "neg(";
    } else {
      O << '-';
    }
  }

  if (InputModifiers & PPUSrcMods::ABS)
    O << '|';
  printOperand(MI, OpNo + 1, STI, O);
  if (InputModifiers & PPUSrcMods::ABS)
    O << '|';

  if (NegMnemo) {
    O << ')';
  }
}

void PPUInstPrinter::printOperandAndIntInputMods(const MCInst *MI,
                                                    unsigned OpNo,
                                                    const MCSubtargetInfo &STI,
                                                    raw_ostream &O) {
  unsigned InputModifiers = MI->getOperand(OpNo).getImm();
  if (InputModifiers & PPUSrcMods::SEXT)
    O << "sext(";
  printOperand(MI, OpNo + 1, STI, O);
  if (InputModifiers & PPUSrcMods::SEXT)
    O << ')';

  // Print default vcc/vcc_lo operand of VOP2b.
  /*
  switch (MI->getOpcode()) {
  default: break;

  case PPU::V_ADD_CO_CI_U32_sdwa_gfx10:
  case PPU::V_SUB_CO_CI_U32_sdwa_gfx10:
  case PPU::V_SUBREV_CO_CI_U32_sdwa_gfx10:
    if ((int)OpNo + 1 == PPU::getNamedOperandIdx(MI->getOpcode(),
                                                    PPU::OpName::src1))
      printDefaultVccOperand(OpNo, STI, O);
    break;
  }
  */
}

#if 0
void PPUInstPrinter::printDPP8(const MCInst *MI, unsigned OpNo,
                                  const MCSubtargetInfo &STI,
                                  raw_ostream &O) {
  if (!PPU::isGFX10(STI))
    llvm_unreachable("dpp8 is not supported on ASICs earlier than GFX10");

  unsigned Imm = MI->getOperand(OpNo).getImm();
  O << " dpp8:[" << formatDec(Imm & 0x7);
  for (size_t i = 1; i < 8; ++i) {
    O << ',' << formatDec((Imm >> (3 * i)) & 0x7);
  }
  O << ']';
}

void PPUInstPrinter::printDPPCtrl(const MCInst *MI, unsigned OpNo,
                                     const MCSubtargetInfo &STI,
                                     raw_ostream &O) {
  using namespace PPU::DPP;

  unsigned Imm = MI->getOperand(OpNo).getImm();
  if (Imm <= DppCtrl::QUAD_PERM_LAST) {
    O << " quad_perm:[";
    O << formatDec(Imm & 0x3)         << ',';
    O << formatDec((Imm & 0xc)  >> 2) << ',';
    O << formatDec((Imm & 0x30) >> 4) << ',';
    O << formatDec((Imm & 0xc0) >> 6) << ']';
  } else if ((Imm >= DppCtrl::ROW_SHL_FIRST) &&
             (Imm <= DppCtrl::ROW_SHL_LAST)) {
    O << " row_shl:";
    printU4ImmDecOperand(MI, OpNo, O);
  } else if ((Imm >= DppCtrl::ROW_SHR_FIRST) &&
             (Imm <= DppCtrl::ROW_SHR_LAST)) {
    O << " row_shr:";
    printU4ImmDecOperand(MI, OpNo, O);
  } else if ((Imm >= DppCtrl::ROW_ROR_FIRST) &&
             (Imm <= DppCtrl::ROW_ROR_LAST)) {
    O << " row_ror:";
    printU4ImmDecOperand(MI, OpNo, O);
  } else if (Imm == DppCtrl::WAVE_SHL1) {
    if (!PPU::isVI(STI) && !PPU::isGFX9(STI)) {
      O << " /* wave_shl is not supported starting from GFX10 */";
      return;
    }
    O << " wave_shl:1";
  } else if (Imm == DppCtrl::WAVE_ROL1) {
    if (!PPU::isVI(STI) && !PPU::isGFX9(STI)) {
      O << " /* wave_rol is not supported starting from GFX10 */";
      return;
    }
    O << " wave_rol:1";
  } else if (Imm == DppCtrl::WAVE_SHR1) {
    if (!PPU::isVI(STI) && !PPU::isGFX9(STI)) {
      O << " /* wave_shr is not supported starting from GFX10 */";
      return;
    }
    O << " wave_shr:1";
  } else if (Imm == DppCtrl::WAVE_ROR1) {
    if (!PPU::isVI(STI) && !PPU::isGFX9(STI)) {
      O << " /* wave_ror is not supported starting from GFX10 */";
      return;
    }
    O << " wave_ror:1";
  } else if (Imm == DppCtrl::ROW_MIRROR) {
    O << " row_mirror";
  } else if (Imm == DppCtrl::ROW_HALF_MIRROR) {
    O << " row_half_mirror";
  } else if (Imm == DppCtrl::BCAST15) {
    if (!PPU::isVI(STI) && !PPU::isGFX9(STI)) {
      O << " /* row_bcast is not supported starting from GFX10 */";
      return;
    }
    O << " row_bcast:15";
  } else if (Imm == DppCtrl::BCAST31) {
    if (!PPU::isVI(STI) && !PPU::isGFX9(STI)) {
      O << " /* row_bcast is not supported starting from GFX10 */";
      return;
    }
    O << " row_bcast:31";
  } else if ((Imm >= DppCtrl::ROW_SHARE_FIRST) &&
             (Imm <= DppCtrl::ROW_SHARE_LAST)) {
    if (!PPU::isGFX10(STI)) {
      O << " /* row_share is not supported on ASICs earlier than GFX10 */";
      return;
    }
    O << " row_share:";
    printU4ImmDecOperand(MI, OpNo, O);
  } else if ((Imm >= DppCtrl::ROW_XMASK_FIRST) &&
             (Imm <= DppCtrl::ROW_XMASK_LAST)) {
    if (!PPU::isGFX10(STI)) {
      O << " /* row_xmask is not supported on ASICs earlier than GFX10 */";
      return;
    }
    O << "row_xmask:";
    printU4ImmDecOperand(MI, OpNo, O);
  } else {
    O << " /* Invalid dpp_ctrl value */";
  }
}
#endif

void PPUInstPrinter::printRowMask(const MCInst *MI, unsigned OpNo,
                                     const MCSubtargetInfo &STI,
                                     raw_ostream &O) {
  O << " row_mask:";
  printU4ImmOperand(MI, OpNo, STI, O);
}

void PPUInstPrinter::printBankMask(const MCInst *MI, unsigned OpNo,
                                      const MCSubtargetInfo &STI,
                                      raw_ostream &O) {
  O << " bank_mask:";
  printU4ImmOperand(MI, OpNo, STI, O);
}

void PPUInstPrinter::printBoundCtrl(const MCInst *MI, unsigned OpNo,
                                       const MCSubtargetInfo &STI,
                                       raw_ostream &O) {
  unsigned Imm = MI->getOperand(OpNo).getImm();
  if (Imm) {
    O << " bound_ctrl:0"; // XXX - this syntax is used in sp3
  }
}


#if 0
void PPUInstPrinter::printFI(const MCInst *MI, unsigned OpNo,
                                const MCSubtargetInfo &STI,
                                raw_ostream &O) {
  using namespace llvm::PPU::DPP;
  unsigned Imm = MI->getOperand(OpNo).getImm();
  if (Imm == DPP_FI_1 || Imm == DPP8_FI_1) {
    O << " fi:1";
  }
}

void PPUInstPrinter::printSDWASel(const MCInst *MI, unsigned OpNo,
                                     raw_ostream &O) {
  using namespace llvm::PPU::SDWA;

  unsigned Imm = MI->getOperand(OpNo).getImm();
  switch (Imm) {
  case SdwaSel::BYTE_0: O << "BYTE_0"; break;
  case SdwaSel::BYTE_1: O << "BYTE_1"; break;
  case SdwaSel::BYTE_2: O << "BYTE_2"; break;
  case SdwaSel::BYTE_3: O << "BYTE_3"; break;
  case SdwaSel::WORD_0: O << "WORD_0"; break;
  case SdwaSel::WORD_1: O << "WORD_1"; break;
  case SdwaSel::DWORD: O << "DWORD"; break;
  default: llvm_unreachable("Invalid SDWA data select operand");
  }
}

void PPUInstPrinter::printSDWADstSel(const MCInst *MI, unsigned OpNo,
                                        const MCSubtargetInfo &STI,
                                        raw_ostream &O) {
  O << "dst_sel:";
  printSDWASel(MI, OpNo, O);
}

void PPUInstPrinter::printSDWASrc0Sel(const MCInst *MI, unsigned OpNo,
                                         const MCSubtargetInfo &STI,
                                         raw_ostream &O) {
  O << "src0_sel:";
  printSDWASel(MI, OpNo, O);
}

void PPUInstPrinter::printSDWASrc1Sel(const MCInst *MI, unsigned OpNo,
                                         const MCSubtargetInfo &STI,
                                         raw_ostream &O) {
  O << "src1_sel:";
  printSDWASel(MI, OpNo, O);
}

void PPUInstPrinter::printSDWADstUnused(const MCInst *MI, unsigned OpNo,
                                           const MCSubtargetInfo &STI,
                                           raw_ostream &O) {
  using namespace llvm::PPU::SDWA;

  O << "dst_unused:";
  unsigned Imm = MI->getOperand(OpNo).getImm();
  switch (Imm) {
  case DstUnused::UNUSED_PAD: O << "UNUSED_PAD"; break;
  case DstUnused::UNUSED_SEXT: O << "UNUSED_SEXT"; break;
  case DstUnused::UNUSED_PRESERVE: O << "UNUSED_PRESERVE"; break;
  default: llvm_unreachable("Invalid SDWA dest_unused operand");
  }
}

template <unsigned N>
void PPUInstPrinter::printExpSrcN(const MCInst *MI, unsigned OpNo,
                                     const MCSubtargetInfo &STI,
                                     raw_ostream &O) {
  unsigned Opc = MI->getOpcode();
  int EnIdx = PPU::getNamedOperandIdx(Opc, PPU::OpName::en);
  unsigned En = MI->getOperand(EnIdx).getImm();

  int ComprIdx = PPU::getNamedOperandIdx(Opc, PPU::OpName::compr);

  // If compr is set, print as src0, src0, src1, src1
  if (MI->getOperand(ComprIdx).getImm()) {
    if (N == 1 || N == 2)
      --OpNo;
    else if (N == 3)
      OpNo -= 2;
  }

  if (En & (1 << N))
    printRegOperand(MI->getOperand(OpNo).getReg(), O, MRI);
  else
    O << "off";
}

void PPUInstPrinter::printExpSrc0(const MCInst *MI, unsigned OpNo,
                                     const MCSubtargetInfo &STI,
                                     raw_ostream &O) {
  printExpSrcN<0>(MI, OpNo, STI, O);
}

void PPUInstPrinter::printExpSrc1(const MCInst *MI, unsigned OpNo,
                                     const MCSubtargetInfo &STI,
                                     raw_ostream &O) {
  printExpSrcN<1>(MI, OpNo, STI, O);
}

void PPUInstPrinter::printExpSrc2(const MCInst *MI, unsigned OpNo,
                                     const MCSubtargetInfo &STI,
                                     raw_ostream &O) {
  printExpSrcN<2>(MI, OpNo, STI, O);
}

void PPUInstPrinter::printExpSrc3(const MCInst *MI, unsigned OpNo,
                                     const MCSubtargetInfo &STI,
                                     raw_ostream &O) {
  printExpSrcN<3>(MI, OpNo, STI, O);
}

void PPUInstPrinter::printExpTgt(const MCInst *MI, unsigned OpNo,
                                    const MCSubtargetInfo &STI,
                                    raw_ostream &O) {
  // This is really a 6 bit field.
  uint32_t Tgt = MI->getOperand(OpNo).getImm() & ((1 << 6) - 1);

  if (Tgt <= 7)
    O << " mrt" << Tgt;
  else if (Tgt == 8)
    O << " mrtz";
  else if (Tgt == 9)
    O << " null";
  else if ((Tgt >= 12 && Tgt <= 15) || (Tgt == 16 && PPU::isGFX10(STI)))
    O << " pos" << Tgt - 12;
  else if (PPU::isGFX10(STI) && Tgt == 20)
    O << " prim";
  else if (Tgt >= 32 && Tgt <= 63)
    O << " param" << Tgt - 32;
  else {
    // Reserved values 10, 11
    O << " invalid_target_" << Tgt;
  }
}
#endif

static bool allOpsDefaultValue(const int* Ops, int NumOps, int Mod,
                               bool IsPacked, bool HasDstSel) {
  int DefaultValue = IsPacked && (Mod == PPUSrcMods::OP_SEL_1);

  for (int I = 0; I < NumOps; ++I) {
    if (!!(Ops[I] & Mod) != DefaultValue)
      return false;
  }

  if (HasDstSel && (Ops[0] & PPUSrcMods::DST_OP_SEL) != 0)
    return false;

  return true;
}

void PPUInstPrinter::printPackedModifier(const MCInst *MI,
                                            StringRef Name,
                                            unsigned Mod,
                                            raw_ostream &O) {
  unsigned Opc = MI->getOpcode();
  int NumOps = 0;
  int Ops[3];

  for (int OpName : { PPU::OpName::src0_modifiers,
                      PPU::OpName::src1_modifiers,
                      PPU::OpName::src2_modifiers }) {
    int Idx = PPU::getNamedOperandIdx(Opc, OpName);
    if (Idx == -1)
      break;

    Ops[NumOps++] = MI->getOperand(Idx).getImm();
  }

  const bool HasDstSel =
    NumOps > 0 &&
    Mod == PPUSrcMods::OP_SEL_0 &&
    MII.get(MI->getOpcode()).TSFlags & PPUInstrFlags::VOP3_OPSEL;

  const bool IsPacked =
    MII.get(MI->getOpcode()).TSFlags & PPUInstrFlags::IsPacked;

  if (allOpsDefaultValue(Ops, NumOps, Mod, IsPacked, HasDstSel))
    return;

  O << Name;
  for (int I = 0; I < NumOps; ++I) {
    if (I != 0)
      O << ',';

    O << !!(Ops[I] & Mod);
  }

  if (HasDstSel) {
    O << ',' << !!(Ops[0] & PPUSrcMods::DST_OP_SEL);
  }

  O << ']';
}

void PPUInstPrinter::printOpSel(const MCInst *MI, unsigned,
                                   const MCSubtargetInfo &STI,
                                   raw_ostream &O) {
  unsigned Opc = MI->getOpcode();
  if (Opc == PPU::V_PERMLANE16_B32_gfx10 ||
      Opc == PPU::V_PERMLANEX16_B32_gfx10) {
    auto FIN = PPU::getNamedOperandIdx(Opc, PPU::OpName::src0_modifiers);
    auto BCN = PPU::getNamedOperandIdx(Opc, PPU::OpName::src1_modifiers);
    unsigned FI = !!(MI->getOperand(FIN).getImm() & PPUSrcMods::OP_SEL_0);
    unsigned BC = !!(MI->getOperand(BCN).getImm() & PPUSrcMods::OP_SEL_0);
    if (FI || BC)
      O << " op_sel:[" << FI << ',' << BC << ']';
    return;
  }

  printPackedModifier(MI, " op_sel:[", PPUSrcMods::OP_SEL_0, O);
}

void PPUInstPrinter::printOpSelHi(const MCInst *MI, unsigned OpNo,
                                     const MCSubtargetInfo &STI,
                                     raw_ostream &O) {
  printPackedModifier(MI, " op_sel_hi:[", PPUSrcMods::OP_SEL_1, O);
}

void PPUInstPrinter::printNegLo(const MCInst *MI, unsigned OpNo,
                                   const MCSubtargetInfo &STI,
                                   raw_ostream &O) {
  printPackedModifier(MI, " neg_lo:[", PPUSrcMods::NEG, O);
}

void PPUInstPrinter::printNegHi(const MCInst *MI, unsigned OpNo,
                                   const MCSubtargetInfo &STI,
                                   raw_ostream &O) {
  printPackedModifier(MI, " neg_hi:[", PPUSrcMods::NEG_HI, O);
}

void PPUInstPrinter::printInterpSlot(const MCInst *MI, unsigned OpNum,
                                        const MCSubtargetInfo &STI,
                                        raw_ostream &O) {
  unsigned Imm = MI->getOperand(OpNum).getImm();
  switch (Imm) {
  case 0:
    O << "p10";
    break;
  case 1:
    O << "p20";
    break;
  case 2:
    O << "p0";
    break;
  default:
    O << "invalid_param_" << Imm;
  }
}

void PPUInstPrinter::printInterpAttr(const MCInst *MI, unsigned OpNum,
                                        const MCSubtargetInfo &STI,
                                        raw_ostream &O) {
  unsigned Attr = MI->getOperand(OpNum).getImm();
  O << "attr" << Attr;
}

void PPUInstPrinter::printInterpAttrChan(const MCInst *MI, unsigned OpNum,
                                        const MCSubtargetInfo &STI,
                                        raw_ostream &O) {
  unsigned Chan = MI->getOperand(OpNum).getImm();
  O << '.' << "xyzw"[Chan & 0x3];
}

void PPUInstPrinter::printVPRIndexMode(const MCInst *MI, unsigned OpNo,
                                           const MCSubtargetInfo &STI,
                                           raw_ostream &O) {
  using namespace llvm::PPU::VGPRIndexMode;
  unsigned Val = MI->getOperand(OpNo).getImm();

  if ((Val & ~ENABLE_MASK) != 0) {
    O << " " << formatHex(static_cast<uint64_t>(Val));
  } else {
    O << " gpr_idx(";
    bool NeedComma = false;
    for (unsigned ModeId = ID_MIN; ModeId <= ID_MAX; ++ModeId) {
      if (Val & (1 << ModeId)) {
        if (NeedComma)
          O << ',';
        /*
        O << IdSymbolic[ModeId];
        */
        NeedComma = true;
      }
    }
    O << ')';
  }
}

void PPUInstPrinter::printMemOperand(const MCInst *MI, unsigned OpNo,
                                        const MCSubtargetInfo &STI,
                                        raw_ostream &O) {
  printOperand(MI, OpNo, STI, O);
  O  << ", ";
  printOperand(MI, OpNo + 1, STI, O);
}

void PPUInstPrinter::printIfSet(const MCInst *MI, unsigned OpNo,
                                   raw_ostream &O, StringRef Asm,
                                   StringRef Default) {
  const MCOperand &Op = MI->getOperand(OpNo);
  assert(Op.isImm());
  if (Op.getImm() == 1) {
    O << Asm;
  } else {
    O << Default;
  }
}

void PPUInstPrinter::printIfSet(const MCInst *MI, unsigned OpNo,
                                   raw_ostream &O, char Asm) {
  const MCOperand &Op = MI->getOperand(OpNo);
  assert(Op.isImm());
  if (Op.getImm() == 1)
    O << Asm;
}

void PPUInstPrinter::printHigh(const MCInst *MI, unsigned OpNo,
                                  const MCSubtargetInfo &STI,
                                  raw_ostream &O) {
  if (MI->getOperand(OpNo).getImm())
    O << " high";
}

void PPUInstPrinter::printClampSI(const MCInst *MI, unsigned OpNo,
                                     const MCSubtargetInfo &STI,
                                     raw_ostream &O) {
  if (MI->getOperand(OpNo).getImm())
    O << " clamp";
}

void PPUInstPrinter::printOModSI(const MCInst *MI, unsigned OpNo,
                                    const MCSubtargetInfo &STI,
                                    raw_ostream &O) {
  int Imm = MI->getOperand(OpNo).getImm();
  if (Imm == PPUOutMods::MUL2)
    O << " mul:2";
  else if (Imm == PPUOutMods::MUL4)
    O << " mul:4";
  else if (Imm == PPUOutMods::DIV2)
    O << " div:2";
}
void PPUInstPrinter::printSendMsg(const MCInst *MI, unsigned OpNo,
                                     const MCSubtargetInfo &STI,
                                     raw_ostream &O) {
    O << "SendMsg is wip in InstPrinter";
/*
  using namespace llvm::PPU::SendMsg;

  const unsigned Imm16 = MI->getOperand(OpNo).getImm();

  uint16_t MsgId;
  uint16_t OpId;
  uint16_t StreamId;
  decodeMsg(Imm16, MsgId, OpId, StreamId);

  if (isValidMsgId(MsgId, STI) &&
      isValidMsgOp(MsgId, OpId) &&
      isValidMsgStream(MsgId, OpId, StreamId)) {
    O << "sendmsg(" << getMsgName(MsgId);
    if (msgRequiresOp(MsgId)) {
      O << ", " << getMsgOpName(MsgId, OpId);
      if (msgSupportsStream(MsgId, OpId)) {
        O << ", " << StreamId;
      }
    }
    O << ')';
  } else if (encodeMsg(MsgId, OpId, StreamId) == Imm16) {
    O << "sendmsg(" << MsgId << ", " << OpId << ", " << StreamId << ')';
  } else {
    O << Imm16; // Unknown imm16 code.
  }
  */
}

static void printSwizzleBitmask(const uint16_t AndMask,
                                const uint16_t OrMask,
                                const uint16_t XorMask,
                                raw_ostream &O) {
  using namespace llvm::PPU::Swizzle;

  uint16_t Probe0 = ((0            & AndMask) | OrMask) ^ XorMask;
  uint16_t Probe1 = ((BITMASK_MASK & AndMask) | OrMask) ^ XorMask;

  O << "\"";

  for (unsigned Mask = 1 << (BITMASK_WIDTH - 1); Mask > 0; Mask >>= 1) {
    uint16_t p0 = Probe0 & Mask;
    uint16_t p1 = Probe1 & Mask;

    if (p0 == p1) {
      if (p0 == 0) {
        O << "0";
      } else {
        O << "1";
      }
    } else {
      if (p0 == 0) {
        O << "p";
      } else {
        O << "i";
      }
    }
  }

  O << "\"";
}

void PPUInstPrinter::printSwizzle(const MCInst *MI, unsigned OpNo,
                                     const MCSubtargetInfo &STI,
                                     raw_ostream &O) {
  using namespace llvm::PPU::Swizzle;

  uint16_t Imm = MI->getOperand(OpNo).getImm();
  if (Imm == 0) {
    return;
  }

  O << " offset:";

  if ((Imm & QUAD_PERM_ENC_MASK) == QUAD_PERM_ENC) {

    O << "swizzle(" << IdSymbolic[ID_QUAD_PERM];
    for (unsigned I = 0; I < LANE_NUM; ++I) {
      O << ",";
      O << formatDec(Imm & LANE_MASK);
      Imm >>= LANE_SHIFT;
    }
    O << ")";

  } else if ((Imm & BITMASK_PERM_ENC_MASK) == BITMASK_PERM_ENC) {

    uint16_t AndMask = (Imm >> BITMASK_AND_SHIFT) & BITMASK_MASK;
    uint16_t OrMask  = (Imm >> BITMASK_OR_SHIFT)  & BITMASK_MASK;
    uint16_t XorMask = (Imm >> BITMASK_XOR_SHIFT) & BITMASK_MASK;

    if (AndMask == BITMASK_MAX &&
        OrMask == 0 &&
        countPopulation(XorMask) == 1) {

      O << "swizzle(" << IdSymbolic[ID_SWAP];
      O << ",";
      O << formatDec(XorMask);
      O << ")";

    } else if (AndMask == BITMASK_MAX &&
               OrMask == 0 && XorMask > 0 &&
               isPowerOf2_64(XorMask + 1)) {

      O << "swizzle(" << IdSymbolic[ID_REVERSE];
      O << ",";
      O << formatDec(XorMask + 1);
      O << ")";

    } else {

      uint16_t GroupSize = BITMASK_MAX - AndMask + 1;
      if (GroupSize > 1 &&
          isPowerOf2_64(GroupSize) &&
          OrMask < GroupSize &&
          XorMask == 0) {

        O << "swizzle(" << IdSymbolic[ID_BROADCAST];
        O << ",";
        O << formatDec(GroupSize);
        O << ",";
        O << formatDec(OrMask);
        O << ")";

      } else {
        O << "swizzle(" << IdSymbolic[ID_BITMASK_PERM];
        O << ",";
        printSwizzleBitmask(AndMask, OrMask, XorMask, O);
        O << ")";
      }
    }
  } else {
    printU16ImmDecOperand(MI, OpNo, O);
  }
}

void PPUInstPrinter::printWaitFlag(const MCInst *MI, unsigned OpNo,
                                      const MCSubtargetInfo &STI,
                                      raw_ostream &O) {
    O << "printWaitFlag WIP";
    /*
  PPU::IsaInfo::IsaVersion ISA = PPU::IsaInfo::getIsaVersion(&STI);

  unsigned SImm16 = MI->getOperand(OpNo).getImm();
  unsigned Vmcnt, Expcnt, Lgkmcnt;
  decodeWaitcnt(ISA, SImm16, Vmcnt, Expcnt, Lgkmcnt);

  bool NeedSpace = false;

  if (Vmcnt != getVmcntBitMask(ISA)) {
    O << "vmcnt(" << Vmcnt << ')';
    NeedSpace = true;
  }

  if (Expcnt != getExpcntBitMask(ISA)) {
    if (NeedSpace)
      O << ' ';
    O << "expcnt(" << Expcnt << ')';
    NeedSpace = true;
  }

  if (Lgkmcnt != getLgkmcntBitMask(ISA)) {
    if (NeedSpace)
      O << ' ';
    O << "lgkmcnt(" << Lgkmcnt << ')';
  }
  */
}
void PPUInstPrinter::printHwreg(const MCInst *MI, unsigned OpNo,
                                   const MCSubtargetInfo &STI, raw_ostream &O) {
  unsigned Id;
  unsigned Offset;
  unsigned Width;

  using namespace llvm::PPU::Hwreg;
  unsigned Val = MI->getOperand(OpNo).getImm();
  decodeHwreg(Val, Id, Offset, Width);
  StringRef HwRegName = getHwreg(Id, STI);

  O << "hwreg(";
  if (!HwRegName.empty()) {
    O << HwRegName;
  } else {
    O << Id;
  }
  if (Width != WIDTH_DEFAULT_ || Offset != OFFSET_DEFAULT_) {
    O << ", " << Offset << ", " << Width;
  }
  O << ')';
}

void PPUInstPrinter::printEndpgm(const MCInst *MI, unsigned OpNo,
                                    const MCSubtargetInfo &STI,
                                    raw_ostream &O) {
  uint16_t Imm = MI->getOperand(OpNo).getImm();
  if (Imm == 0) {
    return;
  }

  O << ' ' << formatDec(Imm);
}

// Include the auto-generated portion of the assembly writer.
// #define PRINT_ALIAS_INSTR
// #include "PPUGenAsmWriter.inc"
