//===-- PPUDisassembler.cpp - Disassembler for PPU --------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file implements the PPUDisassembler class.
//
//===----------------------------------------------------------------------===//
#include "Disassembler/PPUDisassembler.h"
#include "PPU.h"
#include "PPURegisterInfo.h"
#include "MCTargetDesc/PPUMCTargetDesc.h"
#include "PPUDefines.h"
#include "TargetInfo/PPUTargetInfo.h"
#include "Utils/PPUBaseInfo.h"
#include "llvm-c/Disassembler.h"
#include "llvm/ADT/APInt.h"
#include "llvm/ADT/ArrayRef.h"
#include "llvm/ADT/Twine.h"
#include "llvm/BinaryFormat/ELF.h"
#include "llvm/MC/MCAsmInfo.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCDisassembler/MCDisassembler.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCFixedLenDisassembler.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/Support/Endian.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/Support/TargetRegistry.h"
#include "llvm/Support/raw_ostream.h"
#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <tuple>
#include <vector>
#include "llvm/CodeGen/Register.h"
#include "llvm/MC/MCRegisterInfo.h"

using namespace llvm;

#define DEBUG_TYPE "ppu-disassembler"

#define SGPR_MAX PPU::EncValues::SGPR_MAX

typedef MCDisassembler::DecodeStatus DecodeStatus;

// namespace {
// class PPUDisassembler : public MCDisassembler {
// 
// public:
//   PPUDisassembler(const MCSubtargetInfo &STI, MCContext &Ctx)
//       : MCDisassembler(STI, Ctx) {}
// 
//   DecodeStatus getInstruction(MCInst &Instr, uint64_t &Size,
//                               ArrayRef<uint8_t> Bytes, uint64_t Address,
//                               raw_ostream &VStream,
//                               raw_ostream &CStream) const override;
// };
// } // end anonymous namespace
//
PPUDisassembler::PPUDisassembler(const MCSubtargetInfo &STI,
                                       MCContext &Ctx,
                                       MCInstrInfo const *MCII) :
  MCDisassembler(STI, Ctx), MCII(MCII), MRI(*Ctx.getRegisterInfo()),
  TargetMaxInstBytes(Ctx.getAsmInfo()->getMaxInstLength(&STI)) {

  // ToDo: PPUDisassembler supports only VI ISA.
  // if (!STI.getFeatureBits()[PPU::FeatureGCN3Encoding] && !isGFX10())
  //  report_fatal_error("Disassembly not yet supported for subtarget");
}

inline static MCDisassembler::DecodeStatus addOperand(MCInst &Inst, const MCOperand& Opnd) {
  Inst.addOperand(Opnd);
  return Opnd.isValid() ?
    MCDisassembler::Success :
    MCDisassembler::SoftFail;
}

static int insertNamedMCOperand(MCInst &MI, const MCOperand &Op,
                                uint16_t NameIdx) {
  int OpIdx = PPU::getNamedOperandIdx(MI.getOpcode(), NameIdx);
  if (OpIdx != -1) {
    auto I = MI.begin();
    std::advance(I, OpIdx);
    MI.insert(I, Op);
  }
  return OpIdx;
}

static DecodeStatus decodeSoppBrTarget(MCInst &Inst, unsigned Imm,
                                       uint64_t Addr, const void *Decoder) {
  auto DAsm = static_cast<const PPUDisassembler*>(Decoder);

  // Our branches take a simm16, but we need two extra bits to account for the
  // factor of 4.
  APInt SignedOffset(18, Imm * 4, true);
  int64_t Offset = (SignedOffset.sext(64) + 4 + Addr).getSExtValue();

  if (DAsm->tryAddingSymbolicOperand(Inst, Offset, Addr, true, 2, 2))
    return MCDisassembler::Success;
  return addOperand(Inst, MCOperand::createImm(Imm));
}

static DecodeStatus decodeBoolReg(MCInst &Inst, unsigned Val,
                                  uint64_t Addr, const void *Decoder) {
  auto DAsm = static_cast<const PPUDisassembler*>(Decoder);
  return addOperand(Inst, DAsm->decodeBoolReg(Val));
}

#define DECODE_OPERAND(StaticDecoderName, DecoderName) \
static DecodeStatus StaticDecoderName(MCInst &Inst, \
                                       unsigned Imm, \
                                       uint64_t /*Addr*/, \
                                       const void *Decoder) { \
  auto DAsm = static_cast<const PPUDisassembler*>(Decoder); \
  return addOperand(Inst, DAsm->DecoderName(Imm)); \
}

#define DECODE_OPERAND_REG(RegClass) \
DECODE_OPERAND(Decode##RegClass##RegisterClass, decodeOperand_##RegClass)

DECODE_OPERAND_REG(VPR_32)
DECODE_OPERAND_REG(VRegOrLds_32)
DECODE_OPERAND_REG(VS_32)
DECODE_OPERAND_REG(VS_64)
DECODE_OPERAND_REG(VS_128)

DECODE_OPERAND_REG(VReg_32)
DECODE_OPERAND_REG(VReg_64)
DECODE_OPERAND_REG(VReg_96)
DECODE_OPERAND_REG(VReg_128)

DECODE_OPERAND_REG(SReg_32)
DECODE_OPERAND_REG(SReg_32_XM0_XEXEC)
DECODE_OPERAND_REG(SReg_32_XEXEC_HI)
DECODE_OPERAND_REG(SRegOrLds_32)
DECODE_OPERAND_REG(SReg_64)
DECODE_OPERAND_REG(SReg_64_XEXEC)
DECODE_OPERAND_REG(SReg_128)
DECODE_OPERAND_REG(SReg_256)
DECODE_OPERAND_REG(SReg_512)

static DecodeStatus decodeOperand_VSrc16(MCInst &Inst,
                                         unsigned Imm,
                                         uint64_t Addr,
                                         const void *Decoder) {
  auto DAsm = static_cast<const PPUDisassembler*>(Decoder);
  return addOperand(Inst, DAsm->decodeOperand_VSrc16(Imm));
}

static DecodeStatus decodeOperand_VSrc_16(MCInst &Inst,
                                         unsigned Imm,
                                         uint64_t Addr,
                                         const void *Decoder) {
  auto DAsm = static_cast<const PPUDisassembler*>(Decoder);
  return addOperand(Inst, DAsm->decodeOperand_VSrc16(Imm));
}

static DecodeStatus decodeOperand_VSrcV216(MCInst &Inst,
                                         unsigned Imm,
                                         uint64_t Addr,
                                         const void *Decoder) {
  auto DAsm = static_cast<const PPUDisassembler*>(Decoder);
  return addOperand(Inst, DAsm->decodeOperand_VSrcV216(Imm));
}

static DecodeStatus decodeOperand_VS_16(MCInst &Inst,
                                        unsigned Imm,
                                        uint64_t Addr,
                                        const void *Decoder) {
  auto DAsm = static_cast<const PPUDisassembler*>(Decoder);
  return addOperand(Inst, DAsm->decodeOperand_VSrc16(Imm));
}

static DecodeStatus decodeOperand_VS_32(MCInst &Inst,
                                        unsigned Imm,
                                        uint64_t Addr,
                                        const void *Decoder) {
  auto DAsm = static_cast<const PPUDisassembler*>(Decoder);
  return addOperand(Inst, DAsm->decodeOperand_VS_32(Imm));
}

static DecodeStatus decodeOperand_AReg_128(MCInst &Inst,
                                           unsigned Imm,
                                           uint64_t Addr,
                                           const void *Decoder) {
  auto DAsm = static_cast<const PPUDisassembler*>(Decoder);
  return addOperand(Inst, DAsm->decodeSrcOp(PPUDisassembler::OPW128, Imm | 512));
}

static DecodeStatus decodeOperand_AReg_512(MCInst &Inst,
                                           unsigned Imm,
                                           uint64_t Addr,
                                           const void *Decoder) {
  auto DAsm = static_cast<const PPUDisassembler*>(Decoder);
  return addOperand(Inst, DAsm->decodeSrcOp(PPUDisassembler::OPW512, Imm | 512));
}

static DecodeStatus decodeOperand_AReg_1024(MCInst &Inst,
                                            unsigned Imm,
                                            uint64_t Addr,
                                            const void *Decoder) {
  auto DAsm = static_cast<const PPUDisassembler*>(Decoder);
  return addOperand(Inst, DAsm->decodeSrcOp(PPUDisassembler::OPW1024, Imm | 512));
}

static DecodeStatus decodeOperand_SReg_32(MCInst &Inst,
                                          unsigned Imm,
                                          uint64_t Addr,
                                          const void *Decoder) {
  auto DAsm = static_cast<const PPUDisassembler*>(Decoder);
  return addOperand(Inst, DAsm->decodeOperand_SReg_32(Imm));
}

static DecodeStatus decodeOperand_VGPR_32(MCInst &Inst,
                                         unsigned Imm,
                                         uint64_t Addr,
                                         const void *Decoder) {
  auto DAsm = static_cast<const PPUDisassembler*>(Decoder);
  return addOperand(Inst, DAsm->decodeSrcOp(PPUDisassembler::OPW32, Imm));
}


// below is riscv 

static const Register GPRDecoderTable[] = {
  PPU::X0,  PPU::X1,  PPU::X2,  PPU::X3,
  PPU::X4,  PPU::X5,  PPU::X6,  PPU::X7,
  PPU::X8,  PPU::X9,  PPU::X10, PPU::X11,
  PPU::X12, PPU::X13, PPU::X14, PPU::X15,
  PPU::X16, PPU::X17, PPU::X18, PPU::X19,
  PPU::X20, PPU::X21, PPU::X22, PPU::X23,
  PPU::X24, PPU::X25, PPU::X26, PPU::X27,
  PPU::X28, PPU::X29, PPU::X30, PPU::X31
};

static DecodeStatus DecodeGPRRegisterClass(MCInst &Inst, uint64_t RegNo,
                                           uint64_t Address,
                                           const void *Decoder) {
  const FeatureBitset &FeatureBits =
      static_cast<const MCDisassembler *>(Decoder)
          ->getSubtargetInfo()
          .getFeatureBits();
  bool IsRV32E = FeatureBits[PPU::FeatureRV32E];

  if (RegNo > array_lengthof(GPRDecoderTable) || (IsRV32E && RegNo > 15))
    return MCDisassembler::Fail;

  // We must define our own mapping from RegNo to register identifier.
  // Accessing index RegNo in the register class will work in the case that
  // registers were added in ascending order, but not in general.
  Register Reg = GPRDecoderTable[RegNo];
  Inst.addOperand(MCOperand::createReg(Reg));
  return MCDisassembler::Success;
}

static const Register FPR32DecoderTable[] = {
  PPU::F0_32,  PPU::F1_32,  PPU::F2_32,  PPU::F3_32,
  PPU::F4_32,  PPU::F5_32,  PPU::F6_32,  PPU::F7_32,
  PPU::F8_32,  PPU::F9_32,  PPU::F10_32, PPU::F11_32,
  PPU::F12_32, PPU::F13_32, PPU::F14_32, PPU::F15_32,
  PPU::F16_32, PPU::F17_32, PPU::F18_32, PPU::F19_32,
  PPU::F20_32, PPU::F21_32, PPU::F22_32, PPU::F23_32,
  PPU::F24_32, PPU::F25_32, PPU::F26_32, PPU::F27_32,
  PPU::F28_32, PPU::F29_32, PPU::F30_32, PPU::F31_32
};

static DecodeStatus DecodeFPR32RegisterClass(MCInst &Inst, uint64_t RegNo,
                                             uint64_t Address,
                                             const void *Decoder) {
  if (RegNo > array_lengthof(FPR32DecoderTable))
    return MCDisassembler::Fail;

  // We must define our own mapping from RegNo to register identifier.
  // Accessing index RegNo in the register class will work in the case that
  // registers were added in ascending order, but not in general.
  Register Reg = FPR32DecoderTable[RegNo];
  Inst.addOperand(MCOperand::createReg(Reg));
  return MCDisassembler::Success;
}

static DecodeStatus DecodeFPR32CRegisterClass(MCInst &Inst, uint64_t RegNo,
                                              uint64_t Address,
                                              const void *Decoder) {
  if (RegNo > 8) {
    return MCDisassembler::Fail;
  }
  Register Reg = FPR32DecoderTable[RegNo + 8];
  Inst.addOperand(MCOperand::createReg(Reg));
  return MCDisassembler::Success;
}
/*
static const Register FPR64DecoderTable[] = {
  PPU::F0_64,  PPU::F1_64,  PPU::F2_64,  PPU::F3_64,
  PPU::F4_64,  PPU::F5_64,  PPU::F6_64,  PPU::F7_64,
  PPU::F8_64,  PPU::F9_64,  PPU::F10_64, PPU::F11_64,
  PPU::F12_64, PPU::F13_64, PPU::F14_64, PPU::F15_64,
  PPU::F16_64, PPU::F17_64, PPU::F18_64, PPU::F19_64,
  PPU::F20_64, PPU::F21_64, PPU::F22_64, PPU::F23_64,
  PPU::F24_64, PPU::F25_64, PPU::F26_64, PPU::F27_64,
  PPU::F28_64, PPU::F29_64, PPU::F30_64, PPU::F31_64
};

static DecodeStatus DecodeFPR64RegisterClass(MCInst &Inst, uint64_t RegNo,
                                             uint64_t Address,
                                             const void *Decoder) {
  if (RegNo > array_lengthof(FPR64DecoderTable))
    return MCDisassembler::Fail;

  // We must define our own mapping from RegNo to register identifier.
  // Accessing index RegNo in the register class will work in the case that
  // registers were added in ascending order, but not in general.
  Register Reg = FPR64DecoderTable[RegNo];
  Inst.addOperand(MCOperand::createReg(Reg));
  return MCDisassembler::Success;
}

static DecodeStatus DecodeFPR64CRegisterClass(MCInst &Inst, uint64_t RegNo,
                                              uint64_t Address,
                                              const void *Decoder) {
  if (RegNo > 8) {
    return MCDisassembler::Fail;
  }
  Register Reg = FPR64DecoderTable[RegNo + 8];
  Inst.addOperand(MCOperand::createReg(Reg));
  return MCDisassembler::Success;
}
*/

static DecodeStatus DecodeGPRNoX0RegisterClass(MCInst &Inst, uint64_t RegNo,
                                               uint64_t Address,
                                               const void *Decoder) {
  if (RegNo == 0) {
    return MCDisassembler::Fail;
  }

  return DecodeGPRRegisterClass(Inst, RegNo, Address, Decoder);
}

static DecodeStatus DecodeGPRNoX0X2RegisterClass(MCInst &Inst, uint64_t RegNo,
                                                 uint64_t Address,
                                                 const void *Decoder) {
  if (RegNo == 2) {
    return MCDisassembler::Fail;
  }

  return DecodeGPRNoX0RegisterClass(Inst, RegNo, Address, Decoder);
}

/*
static DecodeStatus DecodeGPRCRegisterClass(MCInst &Inst, uint64_t RegNo,
                                            uint64_t Address,
                                            const void *Decoder) {
  if (RegNo > 8)
    return MCDisassembler::Fail;

  Register Reg = GPRDecoderTable[RegNo + 8];
  Inst.addOperand(MCOperand::createReg(Reg));
  return MCDisassembler::Success;
}
*/

// TODO schi copied from rvv
static const unsigned TPRDecoderTable[] = {
  PPU::TPR0,  PPU::TPR1,  PPU::TPR2,  PPU::TPR3,
  PPU::TPR4,  PPU::TPR5,  PPU::TPR6,  PPU::TPR7,
  PPU::TPR8,  PPU::TPR9,  PPU::TPR10, PPU::TPR11,
  PPU::TPR12, PPU::TPR13, PPU::TPR14, PPU::TPR15
};
/*
  PPU::V16, PPU::V17, PPU::V18, PPU::V19,
  PPU::V20, PPU::V21, PPU::V22, PPU::V23,
  PPU::V24, PPU::V25, PPU::V26, PPU::V27,
  PPU::V28, PPU::V29, PPU::V30, PPU::V31
};
*/

static DecodeStatus DecodeTPRRegisterClass(MCInst &Inst, uint64_t RegNo,
                                          uint64_t Address,
                                          const void *Decoder) {
  // FIXME(rkruppe) this should be >= and the same bug exists in the other register decoders
  if (RegNo > sizeof(TPRDecoderTable)) {
    return MCDisassembler::Fail;
  }

  // We must define our own mapping from RegNo to register identifier.
  // Accessing index RegNo in the register class will work in the case that
  // registers were added in ascending order, but not in general.
  unsigned Reg = TPRDecoderTable[RegNo];
  Inst.addOperand(MCOperand::createReg(Reg));
  return MCDisassembler::Success;
}

// Add implied SP operand for instructions *SP compressed instructions. The SP
// operand isn't explicitly encoded in the instruction.
static void addImplySP(MCInst &Inst, int64_t Address, const void *Decoder) {
  /*
  if (Inst.getOpcode() == PPU::C_LWSP || Inst.getOpcode() == PPU::C_SWSP ||
      Inst.getOpcode() == PPU::C_LDSP || Inst.getOpcode() == PPU::C_SDSP ||
      Inst.getOpcode() == PPU::C_FLWSP ||
      Inst.getOpcode() == PPU::C_FSWSP ||
      Inst.getOpcode() == PPU::C_FLDSP ||
      Inst.getOpcode() == PPU::C_FSDSP ||
      Inst.getOpcode() == PPU::C_ADDI4SPN) {
    DecodeGPRRegisterClass(Inst, 2, Address, Decoder);
  }
  if (Inst.getOpcode() == PPU::C_ADDI16SP) {
    DecodeGPRRegisterClass(Inst, 2, Address, Decoder);
    DecodeGPRRegisterClass(Inst, 2, Address, Decoder);
  }
  */
}

template <unsigned N>
static DecodeStatus decodeUImmOperand(MCInst &Inst, uint64_t Imm,
                                      int64_t Address, const void *Decoder) {
  assert(isUInt<N>(Imm) && "Invalid immediate");
  addImplySP(Inst, Address, Decoder);
  Inst.addOperand(MCOperand::createImm(Imm));
  return MCDisassembler::Success;
}

template <unsigned N>
static DecodeStatus decodeUImmNonZeroOperand(MCInst &Inst, uint64_t Imm,
                                             int64_t Address,
                                             const void *Decoder) {
  if (Imm == 0)
    return MCDisassembler::Fail;
  return decodeUImmOperand<N>(Inst, Imm, Address, Decoder);
}

template <unsigned N>
static DecodeStatus decodeSImmOperand(MCInst &Inst, uint64_t Imm,
                                      int64_t Address, const void *Decoder) {
  assert(isUInt<N>(Imm) && "Invalid immediate");
  addImplySP(Inst, Address, Decoder);
  // Sign-extend the number in the bottom N bits of Imm
  Inst.addOperand(MCOperand::createImm(SignExtend64<N>(Imm)));
  return MCDisassembler::Success;
}

template <unsigned N>
static DecodeStatus decodeSImmNonZeroOperand(MCInst &Inst, uint64_t Imm,
                                             int64_t Address,
                                             const void *Decoder) {
  if (Imm == 0)
    return MCDisassembler::Fail;
  return decodeSImmOperand<N>(Inst, Imm, Address, Decoder);
}

template <unsigned N>
static DecodeStatus decodeSImmOperandAndLsl1(MCInst &Inst, uint64_t Imm,
                                             int64_t Address,
                                             const void *Decoder) {
  assert(isUInt<N>(Imm) && "Invalid immediate");
  // Sign-extend the number in the bottom N bits of Imm after accounting for
  // the fact that the N bit immediate is stored in N-1 bits (the LSB is
  // always zero)
  Inst.addOperand(MCOperand::createImm(SignExtend64<N>(Imm << 1)));
  return MCDisassembler::Success;
}

static DecodeStatus decodeCLUIImmOperand(MCInst &Inst, uint64_t Imm,
                                         int64_t Address,
                                         const void *Decoder) {
  assert(isUInt<6>(Imm) && "Invalid immediate");
  if (Imm > 31) {
    Imm = (SignExtend64<6>(Imm) & 0xfffff);
  }
  Inst.addOperand(MCOperand::createImm(Imm));
  return MCDisassembler::Success;
}

static DecodeStatus decodeFRMArg(MCInst &Inst, uint64_t Imm,
                                 int64_t Address,
                                 const void *Decoder) {
  assert(isUInt<3>(Imm) && "Invalid immediate");
  if (!llvm::PPUFPRndMode::isValidRoundingMode(Imm))
    return MCDisassembler::Fail;

  Inst.addOperand(MCOperand::createImm(Imm));
  return MCDisassembler::Success;
}
/*
static DecodeStatus decodeRVCInstrSImm(MCInst &Inst, unsigned Insn,
                                       uint64_t Address, const void *Decoder);

static DecodeStatus decodeRVCInstrRdSImm(MCInst &Inst, unsigned Insn,
                                         uint64_t Address, const void *Decoder);

static DecodeStatus decodeRVCInstrRdRs1UImm(MCInst &Inst, unsigned Insn,
                                            uint64_t Address,
                                            const void *Decoder);

static DecodeStatus decodeRVCInstrRdRs2(MCInst &Inst, unsigned Insn,
                                        uint64_t Address, const void *Decoder);

static DecodeStatus decodeRVCInstrRdRs1Rs2(MCInst &Inst, unsigned Insn,
                                           uint64_t Address,
                                           const void *Decoder);
*/
#include "PPUGenDisassemblerTables.inc"
/*
static DecodeStatus decodeRVCInstrSImm(MCInst &Inst, unsigned Insn,
                                       uint64_t Address, const void *Decoder) {
  uint64_t SImm6 =
      fieldFromInstruction(Insn, 12, 1) << 5 | fieldFromInstruction(Insn, 2, 5);
  DecodeStatus Result = decodeSImmOperand<6>(Inst, SImm6, Address, Decoder);
  (void)Result;
  assert(Result == MCDisassembler::Success && "Invalid immediate");
  return MCDisassembler::Success;
}

static DecodeStatus decodeRVCInstrRdSImm(MCInst &Inst, unsigned Insn,
                                         uint64_t Address,
                                         const void *Decoder) {
  DecodeGPRRegisterClass(Inst, 0, Address, Decoder);
  uint64_t SImm6 =
      fieldFromInstruction(Insn, 12, 1) << 5 | fieldFromInstruction(Insn, 2, 5);
  DecodeStatus Result = decodeSImmOperand<6>(Inst, SImm6, Address, Decoder);
  (void)Result;
  assert(Result == MCDisassembler::Success && "Invalid immediate");
  return MCDisassembler::Success;
}

static DecodeStatus decodeRVCInstrRdRs1UImm(MCInst &Inst, unsigned Insn,
                                            uint64_t Address,
                                            const void *Decoder) {
  DecodeGPRRegisterClass(Inst, 0, Address, Decoder);
  Inst.addOperand(Inst.getOperand(0));
  uint64_t UImm6 =
      fieldFromInstruction(Insn, 12, 1) << 5 | fieldFromInstruction(Insn, 2, 5);
  DecodeStatus Result = decodeUImmOperand<6>(Inst, UImm6, Address, Decoder);
  (void)Result;
  assert(Result == MCDisassembler::Success && "Invalid immediate");
  return MCDisassembler::Success;
}

static DecodeStatus decodeRVCInstrRdRs2(MCInst &Inst, unsigned Insn,
                                        uint64_t Address, const void *Decoder) {
  unsigned Rd = fieldFromInstruction(Insn, 7, 5);
  unsigned Rs2 = fieldFromInstruction(Insn, 2, 5);
  DecodeGPRRegisterClass(Inst, Rd, Address, Decoder);
  DecodeGPRRegisterClass(Inst, Rs2, Address, Decoder);
  return MCDisassembler::Success;
}

static DecodeStatus decodeRVCInstrRdRs1Rs2(MCInst &Inst, unsigned Insn,
                                           uint64_t Address,
                                           const void *Decoder) {
  unsigned Rd = fieldFromInstruction(Insn, 7, 5);
  unsigned Rs2 = fieldFromInstruction(Insn, 2, 5);
  DecodeGPRRegisterClass(Inst, Rd, Address, Decoder);
  Inst.addOperand(Inst.getOperand(0));
  DecodeGPRRegisterClass(Inst, Rs2, Address, Decoder);
  return MCDisassembler::Success;
}
*/

//===----------------------------------------------------------------------===//
//
//===----------------------------------------------------------------------===//

template <typename T> static inline T eatBytes(ArrayRef<uint8_t>& Bytes) {
  assert(Bytes.size() >= sizeof(T));
  const auto Res = support::endian::read<T, support::endianness::little>(Bytes.data());
  Bytes = Bytes.slice(sizeof(T));
  return Res;
}

DecodeStatus PPUDisassembler::tryDecodeInst(const uint8_t* Table,
                                               MCInst &MI,
                                               uint64_t Inst,
                                               uint64_t Address) const {
  assert(MI.getOpcode() == 0);
  assert(MI.getNumOperands() == 0);
  MCInst TmpInst;
  HasLiteral = false;
  const auto SavedBytes = Bytes;
  if (decodeInstruction(Table, TmpInst, Inst, Address, this, STI)) {
    MI = TmpInst;
    return MCDisassembler::Success;
  }
  Bytes = SavedBytes;
  return MCDisassembler::Fail;
}

DecodeStatus PPUDisassembler::getInstruction(MCInst &MI, uint64_t &Size,
                                                ArrayRef<uint8_t> Bytes_,
                                                uint64_t Address,
                                                raw_ostream &WS,
                                                raw_ostream &CS) const {
  CommentStream = &CS;
  bool IsSDWA = false;

  unsigned MaxInstBytesNum = std::min((size_t)TargetMaxInstBytes, Bytes_.size());
  Bytes = Bytes_.slice(0, MaxInstBytesNum);

  DecodeStatus Res = MCDisassembler::Fail;
  do {
    // ToDo: better to switch encoding length using some bit predicate
    // but it is unknown yet, so try all we can

    // Try to decode DPP and SDWA first to solve conflict with VOP1 and VOP2
    // encodings
//    if (Bytes.size() >= 8) {
//      const uint64_t QW = eatBytes<uint64_t>(Bytes);

//      Res = tryDecodeInst(DecoderTableDPP864, MI, QW, Address);
//      if (Res && convertDPP8Inst(MI) == MCDisassembler::Success)
//        break;

      MI = MCInst(); // clear
/*
      Res = tryDecodeInst(DecoderTableDPP64, MI, QW, Address);
      if (Res) break;

      Res = tryDecodeInst(DecoderTableSDWA64, MI, QW, Address);
      if (Res) { IsSDWA = true;  break; }

      Res = tryDecodeInst(DecoderTableSDWA964, MI, QW, Address);
      if (Res) { IsSDWA = true;  break; }

      Res = tryDecodeInst(DecoderTableSDWA1064, MI, QW, Address);
      if (Res) { IsSDWA = true;  break; }
*/
      // Some GFX9 subtargets repurposed the v_mad_mix_f32, v_mad_mixlo_f16 and
      // v_mad_mixhi_f16 for FMA variants. Try to decode using this special
      // table first so we print the correct name.
/*
      if (STI.getFeatureBits()[PPU::FeatureFmaMixInsts]) {
        Res = tryDecodeInst(DecoderTableGFX9_DL64, MI, QW, Address);
        if (Res) break;
      }

      if (STI.getFeatureBits()[PPU::FeatureUnpackedD16VMem]) {
        Res = tryDecodeInst(DecoderTableGFX80_UNPACKED64, MI, QW, Address);
        if (Res)
          break;
      }

      // Some GFX9 subtargets repurposed the v_mad_mix_f32, v_mad_mixlo_f16 and
      // v_mad_mixhi_f16 for FMA variants. Try to decode using this special
      // table first so we print the correct name.
      if (STI.getFeatureBits()[PPU::FeatureFmaMixInsts]) {
        Res = tryDecodeInst(DecoderTableGFX9_DL64, MI, QW, Address);
        if (Res)
          break;
      }
    }
*/
    // Reinitialize Bytes as DPP64 could have eaten too much
    Bytes = Bytes_.slice(0, MaxInstBytesNum);

    // Try decode 32-bit instruction
    if (Bytes.size() < 4) break;
    const uint32_t DW = eatBytes<uint32_t>(Bytes);

    Res = tryDecodeInst(DecoderTablePPU32, MI, DW, Address);
    if (Res) break;

    if (Bytes.size() < 4) break;

    const uint64_t QW = ((uint64_t)eatBytes<uint32_t>(Bytes) << 32) | DW;

    Res = tryDecodeInst(DecoderTablePPU64, MI, QW, Address);
    if (Res) break;

  } while (false);

  if (Res && (MaxInstBytesNum - Bytes.size()) == 12 && (!HasLiteral ||
        !(MCII->get(MI.getOpcode()).TSFlags & PPUInstrFlags::VOP3))) {
    MaxInstBytesNum = 8;
    Bytes = Bytes_.slice(0, MaxInstBytesNum);
    eatBytes<uint64_t>(Bytes);
  }

  if (Res && (MI.getOpcode() == PPU::V_MAC_F32_e64_ppu ||
              // MI.getOpcode() == PPU::V_MAC_F16_e64_ppu ||
              // MI.getOpcode() == PPU::V_FMAC_F32_e64_ppu ||
              MI.getOpcode() == PPU::V_FMAC_F16_e64_ppu)) {
    // Insert dummy unused src2_modifiers.
    insertNamedMCOperand(MI, MCOperand::createImm(0),
                         PPU::OpName::src2_modifiers);
  }
/*
  if (Res && (MCII->get(MI.getOpcode()).TSFlags & SIInstrFlags::MIMG)) {
    int VAddr0Idx =
        PPU::getNamedOperandIdx(MI.getOpcode(), PPU::OpName::vaddr0);
    int RsrcIdx =
        PPU::getNamedOperandIdx(MI.getOpcode(), PPU::OpName::srsrc);
    unsigned NSAArgs = RsrcIdx - VAddr0Idx - 1;
    if (VAddr0Idx >= 0 && NSAArgs > 0) {
      unsigned NSAWords = (NSAArgs + 3) / 4;
      if (Bytes.size() < 4 * NSAWords) {
        Res = MCDisassembler::Fail;
      } else {
        for (unsigned i = 0; i < NSAArgs; ++i) {
          MI.insert(MI.begin() + VAddr0Idx + 1 + i,
                    decodeOperand_VGPR_32(Bytes[i]));
        }
        Bytes = Bytes.slice(4 * NSAWords);
      }
    }

    if (Res)
      Res = convertMIMGInst(MI);
  }
  */

//  if (Res && IsSDWA)
//    Res = convertSDWAInst(MI);

  int VDstIn_Idx = PPU::getNamedOperandIdx(MI.getOpcode(),
                                              PPU::OpName::vdst_in);
  if (VDstIn_Idx != -1) {
    int Tied = MCII->get(MI.getOpcode()).getOperandConstraint(VDstIn_Idx,
                           MCOI::OperandConstraint::TIED_TO);
    if (Tied != -1 && (MI.getNumOperands() <= (unsigned)VDstIn_Idx ||
         !MI.getOperand(VDstIn_Idx).isReg() ||
         MI.getOperand(VDstIn_Idx).getReg() != MI.getOperand(Tied).getReg())) {
      if (MI.getNumOperands() > (unsigned)VDstIn_Idx)
        MI.erase(&MI.getOperand(VDstIn_Idx));
      insertNamedMCOperand(MI,
        MCOperand::createReg(MI.getOperand(Tied).getReg()),
        PPU::OpName::vdst_in);
    }
  }

  // if the opcode was not recognized we'll assume a Size of 4 bytes
  // (unless there are fewer bytes left)
  Size = Res ? (MaxInstBytesNum - Bytes.size())
             : std::min((size_t)4, Bytes_.size());
  return Res;
}

/*
DecodeStatus PPUDisassembler::getInstruction(MCInst &MI, uint64_t &Size,
                                               ArrayRef<uint8_t> Bytes,
                                               uint64_t Address,
                                               raw_ostream &OS,
                                               raw_ostream &CS) const {
  // TODO: This will need modification when supporting instruction set
  // extensions with instructions > 32-bits (up to 176 bits wide).
  uint32_t Insn;
  DecodeStatus Result;

  // It's a 32 bit instruction if bit 0 and 1 are 1.
  if ((Bytes[0] & 0x3) == 0x3) {
    if (Bytes.size() < 4) {
      Size = 0;
      return MCDisassembler::Fail;
    }
    Insn = support::endian::read32le(Bytes.data());
    LLVM_DEBUG(dbgs() << "Trying PPU table :\n");
    Result = decodeInstruction(DecoderTable32, MI, Insn, Address, this, STI);
    Size = 4;
  } else {
    if (Bytes.size() < 2) {
      Size = 0;
      return MCDisassembler::Fail;
    }
    Insn = support::endian::read16le(Bytes.data());

    if (!STI.getFeatureBits()[PPU::Feature64Bit]) {
      LLVM_DEBUG(
          dbgs() << "Trying PPU Only_16 table (16-bit Instruction):\n");
      // Calling the auto-generated decoder function.
      Result = decodeInstruction(DecoderTablePPU32Only_16, MI, Insn, Address,
                                 this, STI);
      if (Result != MCDisassembler::Fail) {
        Size = 2;
        return Result;
      }
    }

    LLVM_DEBUG(dbgs() << "Trying PPU_C table (16-bit Instruction):\n");
    // Calling the auto-generated decoder function.
    Result = decodeInstruction(DecoderTable16, MI, Insn, Address, this, STI);
    Size = 2;
  }

  return Result;
}
*/
const char* PPUDisassembler::getRegClassName(unsigned RegClassID) const {
  return getContext().getRegisterInfo()->
    getRegClassName(&PPUMCRegisterClasses[RegClassID]);
}

inline
MCOperand PPUDisassembler::errOperand(unsigned V,
                                         const Twine& ErrMsg) const {
  *CommentStream << "Error: " + ErrMsg;

  // ToDo: add support for error operands to MCInst.h
  // return MCOperand::createError(V);
  return MCOperand();
}

inline
MCOperand PPUDisassembler::createRegOperand(unsigned int RegId) const {
  return MCOperand::createReg(PPU::getMCReg(RegId, STI));
}

inline
MCOperand PPUDisassembler::createRegOperand(unsigned RegClassID,
                                               unsigned Val) const {
  const auto& RegCl = PPUMCRegisterClasses[RegClassID];
  if (Val >= RegCl.getNumRegs())
    return errOperand(Val, Twine(getRegClassName(RegClassID)) +
                           ": unknown register " + Twine(Val));
  return createRegOperand(RegCl.getRegister(Val));
}

inline
MCOperand PPUDisassembler::createSRegOperand(unsigned SRegClassID,
                                                unsigned Val) const {
  // ToDo: SI/CI have 104 SGPRs, VI - 102
  // Valery: here we accepting as much as we can, let assembler sort it out
  int shift = 0;
  switch (SRegClassID) {
  case PPU::SPR_32RegClassID:
  // case PPU::TTMP_32RegClassID:
    break;
  case PPU::SPR_64RegClassID:
  // case PPU::TTMP_64RegClassID:
    shift = 1;
    break;
  case PPU::SPR_128RegClassID:
  // case PPU::TTMP_128RegClassID:
  // ToDo: unclear if s[100:104] is available on VI. Can we use VCC as SGPR in
  // this bundle?
  /*
  case PPU::SGPR_256RegClassID:
  case PPU::TTMP_256RegClassID:
    // ToDo: unclear if s[96:104] is available on VI. Can we use VCC as SGPR in
  // this bundle?
  case PPU::SGPR_512RegClassID:
  case PPU::TTMP_512RegClassID:
    shift = 2;
    break;
  */
  // ToDo: unclear if s[88:104] is available on VI. Can we use VCC as SGPR in
  // this bundle?
  default:
    llvm_unreachable("unhandled register class");
  }

  if (Val % (1 << shift)) {
    *CommentStream << "Warning: " << getRegClassName(SRegClassID)
                   << ": scalar reg isn't aligned " << Val;
  }

  return createRegOperand(SRegClassID, Val >> shift);
}

MCOperand PPUDisassembler::decodeOperand_VS_32(unsigned Val) const {
  return decodeSrcOp(OPW32, Val);
}

MCOperand PPUDisassembler::decodeOperand_VS_64(unsigned Val) const {
  return decodeSrcOp(OPW64, Val);
}

MCOperand PPUDisassembler::decodeOperand_VS_128(unsigned Val) const {
  return decodeSrcOp(OPW128, Val);
}

MCOperand PPUDisassembler::decodeOperand_VSrc16(unsigned Val) const {
  return decodeSrcOp(OPW16, Val);
}

MCOperand PPUDisassembler::decodeOperand_VSrcV216(unsigned Val) const {
  return decodeSrcOp(OPWV216, Val);
}

MCOperand PPUDisassembler::decodeOperand_VPR_32(unsigned Val) const {
  // Some instructions have operand restrictions beyond what the encoding
  // allows. Some ordinarily VSrc_32 operands are VGPR_32, so clear the extra
  // high bit.
  Val &= 255;

  return createRegOperand(PPU::VPR_32RegClassID, Val);
}

MCOperand PPUDisassembler::decodeOperand_VRegOrLds_32(unsigned Val) const {
  return decodeSrcOp(OPW32, Val);
}
/*
MCOperand PPUDisassembler::decodeOperand_AGPR_32(unsigned Val) const {
  return createRegOperand(PPU::AGPR_32RegClassID, Val & 255);
}

MCOperand PPUDisassembler::decodeOperand_AReg_128(unsigned Val) const {
  return createRegOperand(PPU::AReg_128RegClassID, Val & 255);
}

MCOperand PPUDisassembler::decodeOperand_AReg_512(unsigned Val) const {
  return createRegOperand(PPU::AReg_512RegClassID, Val & 255);
}

MCOperand PPUDisassembler::decodeOperand_AReg_1024(unsigned Val) const {
  return createRegOperand(PPU::AReg_1024RegClassID, Val & 255);
}

MCOperand PPUDisassembler::decodeOperand_AV_32(unsigned Val) const {
  return decodeSrcOp(OPW32, Val);
}

MCOperand PPUDisassembler::decodeOperand_AV_64(unsigned Val) const {
  return decodeSrcOp(OPW64, Val);
}
*/

MCOperand PPUDisassembler::decodeOperand_VReg_32(unsigned Val) const {
  return createRegOperand(PPU::VReg_32RegClassID, Val);
}

MCOperand PPUDisassembler::decodeOperand_VReg_64(unsigned Val) const {
  return createRegOperand(PPU::VReg_64RegClassID, Val);
}

MCOperand PPUDisassembler::decodeOperand_VReg_96(unsigned Val) const {
  return createRegOperand(PPU::VReg_96RegClassID, Val);
}

MCOperand PPUDisassembler::decodeOperand_VReg_128(unsigned Val) const {
  return createRegOperand(PPU::VReg_128RegClassID, Val);
}
/*
MCOperand PPUDisassembler::decodeOperand_VReg_256(unsigned Val) const {
  return createRegOperand(PPU::VReg_256RegClassID, Val);
}

MCOperand PPUDisassembler::decodeOperand_VReg_512(unsigned Val) const {
  return createRegOperand(PPU::VReg_512RegClassID, Val);
}
*/

MCOperand PPUDisassembler::decodeOperand_SReg_32(unsigned Val) const {
  // table-gen generated disassembler doesn't care about operand types
  // leaving only registry class so SSrc_32 operand turns into SReg_32
  // and therefore we accept immediates and literals here as well
  return decodeSrcOp(OPW32, Val);
}

MCOperand PPUDisassembler::decodeOperand_SReg_32_XM0_XEXEC(
  unsigned Val) const {
  // SReg_32_XM0 is SReg_32 without M0 or EXEC_LO/EXEC_HI
  return decodeOperand_SReg_32(Val);
}

MCOperand PPUDisassembler::decodeOperand_SReg_32_XEXEC_HI(
  unsigned Val) const {
  // SReg_32_XM0 is SReg_32 without EXEC_HI
  return decodeOperand_SReg_32(Val);
}

MCOperand PPUDisassembler::decodeOperand_SRegOrLds_32(unsigned Val) const {
  // table-gen generated disassembler doesn't care about operand types
  // leaving only registry class so SSrc_32 operand turns into SReg_32
  // and therefore we accept immediates and literals here as well
  return decodeSrcOp(OPW32, Val);
}

MCOperand PPUDisassembler::decodeOperand_SReg_64(unsigned Val) const {
  return decodeSrcOp(OPW64, Val);
}

MCOperand PPUDisassembler::decodeOperand_SReg_64_XEXEC(unsigned Val) const {
  return decodeSrcOp(OPW64, Val);
}

MCOperand PPUDisassembler::decodeOperand_SReg_128(unsigned Val) const {
  return decodeSrcOp(OPW128, Val);
}

MCOperand PPUDisassembler::decodeOperand_SReg_256(unsigned Val) const {
  return decodeDstOp(OPW256, Val);
}

MCOperand PPUDisassembler::decodeOperand_SReg_512(unsigned Val) const {
  return decodeDstOp(OPW512, Val);
}

MCOperand PPUDisassembler::decodeLiteralConstant() const {
  // For now all literal constants are supposed to be unsigned integer
  // ToDo: deal with signed/unsigned 64-bit integer constants
  // ToDo: deal with float/double constants
  if (!HasLiteral) {
    if (Bytes.size() < 4) {
      return errOperand(0, "cannot read literal, inst bytes left " +
                        Twine(Bytes.size()));
    }
    HasLiteral = true;
    Literal = eatBytes<uint32_t>(Bytes);
  }
  return MCOperand::createImm(Literal);
}

MCOperand PPUDisassembler::decodeIntImmed(unsigned Imm) {
  using namespace PPU::EncValues;

  assert(Imm >= INLINE_INTEGER_C_MIN && Imm <= INLINE_INTEGER_C_MAX);
  return MCOperand::createImm((Imm <= INLINE_INTEGER_C_POSITIVE_MAX) ?
    (static_cast<int64_t>(Imm) - INLINE_INTEGER_C_MIN) :
    (INLINE_INTEGER_C_POSITIVE_MAX - static_cast<int64_t>(Imm)));
      // Cast prevents negative overflow.
}

static int64_t getInlineImmVal32(unsigned Imm) {
  switch (Imm) {
  case 240:
    return FloatToBits(0.5f);
  case 241:
    return FloatToBits(-0.5f);
  case 242:
    return FloatToBits(1.0f);
  case 243:
    return FloatToBits(-1.0f);
  case 244:
    return FloatToBits(2.0f);
  case 245:
    return FloatToBits(-2.0f);
  case 246:
    return FloatToBits(4.0f);
  case 247:
    return FloatToBits(-4.0f);
  case 248: // 1 / (2 * PI)
    return 0x3e22f983;
  default:
    llvm_unreachable("invalid fp inline imm");
  }
}

static int64_t getInlineImmVal64(unsigned Imm) {
  switch (Imm) {
  case 240:
    return DoubleToBits(0.5);
  case 241:
    return DoubleToBits(-0.5);
  case 242:
    return DoubleToBits(1.0);
  case 243:
    return DoubleToBits(-1.0);
  case 244:
    return DoubleToBits(2.0);
  case 245:
    return DoubleToBits(-2.0);
  case 246:
    return DoubleToBits(4.0);
  case 247:
    return DoubleToBits(-4.0);
  case 248: // 1 / (2 * PI)
    return 0x3fc45f306dc9c882;
  default:
    llvm_unreachable("invalid fp inline imm");
  }
}

static int64_t getInlineImmVal16(unsigned Imm) {
  switch (Imm) {
  case 240:
    return 0x3800;
  case 241:
    return 0xB800;
  case 242:
    return 0x3C00;
  case 243:
    return 0xBC00;
  case 244:
    return 0x4000;
  case 245:
    return 0xC000;
  case 246:
    return 0x4400;
  case 247:
    return 0xC400;
  case 248: // 1 / (2 * PI)
    return 0x3118;
  default:
    llvm_unreachable("invalid fp inline imm");
  }
}

MCOperand PPUDisassembler::decodeFPImmed(OpWidthTy Width, unsigned Imm) {
  assert(Imm >= PPU::EncValues::INLINE_FLOATING_C_MIN
      && Imm <= PPU::EncValues::INLINE_FLOATING_C_MAX);

  // ToDo: case 248: 1/(2*PI) - is allowed only on VI
  switch (Width) {
  case OPW32:
  case OPW128: // splat constants
  case OPW512:
  case OPW1024:
    return MCOperand::createImm(getInlineImmVal32(Imm));
  case OPW64:
    return MCOperand::createImm(getInlineImmVal64(Imm));
  case OPW16:
  case OPWV216:
    return MCOperand::createImm(getInlineImmVal16(Imm));
  default:
    llvm_unreachable("implement me");
  }
}

unsigned PPUDisassembler::getVgprClassId(const OpWidthTy Width) const {
  using namespace PPU;

  assert(OPW_FIRST_ <= Width && Width < OPW_LAST_);
  switch (Width) {
  default: // fall
  case OPW32:
  case OPW16:
  case OPWV216:
    return VPR_32RegClassID;
  case OPW64: return VReg_64RegClassID;
  case OPW128: return VReg_128RegClassID;
  }
}
/*
unsigned PPUDisassembler::getAgprClassId(const OpWidthTy Width) const {
  using namespace PPU;

  assert(OPW_FIRST_ <= Width && Width < OPW_LAST_);
  switch (Width) {
  default: // fall
  case OPW32:
  case OPW16:
  case OPWV216:
    return AGPR_32RegClassID;
  case OPW64: return AReg_64RegClassID;
  case OPW128: return AReg_128RegClassID;
  case OPW512: return AReg_512RegClassID;
  case OPW1024: return AReg_1024RegClassID;
  }
}
*/

unsigned PPUDisassembler::getSgprClassId(const OpWidthTy Width) const {
  using namespace PPU;

  assert(OPW_FIRST_ <= Width && Width < OPW_LAST_);
  switch (Width) {
  default: // fall
  case OPW32:
  case OPW16:
  case OPWV216:
    return SPR_32RegClassID;
  case OPW64: return SPR_64RegClassID;
  case OPW128: return SPR_128RegClassID;
  /*
  case OPW256: return SPR_256RegClassID;
  case OPW512: return SPR_512RegClassID;
  */
  }
}
/*
unsigned PPUDisassembler::getTtmpClassId(const OpWidthTy Width) const {
  using namespace PPU;

  assert(OPW_FIRST_ <= Width && Width < OPW_LAST_);
  switch (Width) {
  default: // fall
  case OPW32:
  case OPW16:
  case OPWV216:
    return TTMP_32RegClassID;
  case OPW64: return TTMP_64RegClassID;
  case OPW128: return TTMP_128RegClassID;
  case OPW256: return TTMP_256RegClassID;
  case OPW512: return TTMP_512RegClassID;
  }
}

int PPUDisassembler::getTTmpIdx(unsigned Val) const {
  using namespace PPU::EncValues;

  unsigned TTmpMin =
      (isGFX9() || isGFX10()) ? TTMP_GFX9_GFX10_MIN : TTMP_VI_MIN;
  unsigned TTmpMax =
      (isGFX9() || isGFX10()) ? TTMP_GFX9_GFX10_MAX : TTMP_VI_MAX;

  return (TTmpMin <= Val && Val <= TTmpMax)? Val - TTmpMin : -1;
}
*/

MCOperand PPUDisassembler::decodeSrcOp(const OpWidthTy Width, unsigned Val) const {
  using namespace PPU::EncValues;

  assert(Val < 1024); // enum10

  bool IsAGPR = Val & 512;
  Val &= 511;

  if (VGPR_MIN <= Val && Val <= VGPR_MAX) {
    return createRegOperand(getVgprClassId(Width), Val - VGPR_MIN);
    /*return createRegOperand(IsAGPR ? getAgprClassId(Width)
                                   : getVgprClassId(Width), Val - VGPR_MIN);*/
  }
  if (Val <= SGPR_MAX) {
    assert(SGPR_MIN == 0); // "SGPR_MIN <= Val" is always true and causes compilation warning.
    return createSRegOperand(getSgprClassId(Width), Val - SGPR_MIN);
  }
/*
  int TTmpIdx = getTTmpIdx(Val);
  if (TTmpIdx >= 0) {
    return createSRegOperand(getTtmpClassId(Width), TTmpIdx);
  }
*/
  if (INLINE_INTEGER_C_MIN <= Val && Val <= INLINE_INTEGER_C_MAX)
    return decodeIntImmed(Val);

  if (INLINE_FLOATING_C_MIN <= Val && Val <= INLINE_FLOATING_C_MAX)
    return decodeFPImmed(Width, Val);

  if (Val == LITERAL_CONST)
    return decodeLiteralConstant();

  switch (Width) {
  case OPW32:
  case OPW16:
  case OPWV216:
    return decodeSpecialReg32(Val);
  case OPW64:
    return decodeSpecialReg64(Val);
  default:
    llvm_unreachable("unexpected immediate type");
  }
}

MCOperand PPUDisassembler::decodeDstOp(const OpWidthTy Width, unsigned Val) const {
  using namespace PPU::EncValues;

  assert(Val < 128);
  assert(Width == OPW256 || Width == OPW512);

  if (Val <= SGPR_MAX) {
    assert(SGPR_MIN == 0); // "SGPR_MIN <= Val" is always true and causes compilation warning.
    return createSRegOperand(getSgprClassId(Width), Val - SGPR_MIN);
  }
/*
  int TTmpIdx = getTTmpIdx(Val);
  if (TTmpIdx >= 0) {
    return createSRegOperand(getTtmpClassId(Width), TTmpIdx);
  }
*/
  llvm_unreachable("unknown dst register");
}

MCOperand PPUDisassembler::decodeSpecialReg32(unsigned Val) const {
  using namespace PPU;

  switch (Val) {
  case 102: return createRegOperand(FLAT_SCR_LO);
  case 103: return createRegOperand(FLAT_SCR_HI);
  // case 104: return createRegOperand(XNACK_MASK_LO);
  // case 105: return createRegOperand(XNACK_MASK_HI);
  case 106: return createRegOperand(VCC);
  /*
  case 108: return createRegOperand(TBA_LO);
  case 109: return createRegOperand(TBA_HI);
  case 110: return createRegOperand(TMA_LO);
  case 111: return createRegOperand(TMA_HI);
  */
  case 124: return createRegOperand(M0);
  case 125: return createRegOperand(SPR_NULL);
  case 126: return createRegOperand(TMSK);
  // case 127: return createRegOperand(EXEC_HI);
  case 235: return createRegOperand(SRC_SHARED_BASE);
  case 236: return createRegOperand(SRC_SHARED_LIMIT);
  case 237: return createRegOperand(SRC_PRIVATE_BASE);
  case 238: return createRegOperand(SRC_PRIVATE_LIMIT);
  case 239: return createRegOperand(SRC_POPS_EXITING_WAVE_ID);
  case 251: return createRegOperand(SRC_VCCZ);
  case 252: return createRegOperand(SRC_TMSKZ);
  case 253: return createRegOperand(SRC_SCC);
  case 254: return createRegOperand(LDS_DIRECT);
  default: break;
  }
  return errOperand(Val, "unknown operand encoding " + Twine(Val));
}

MCOperand PPUDisassembler::decodeSpecialReg64(unsigned Val) const {
  using namespace PPU;

  switch (Val) {
  case 102: return createRegOperand(FLAT_SCR);
  // case 104: return createRegOperand(XNACK_MASK);
  case 106: return createRegOperand(VCC);
  // case 108: return createRegOperand(TBA);
  // case 110: return createRegOperand(TMA);
  case 126: return createRegOperand(TMSK);
  case 235: return createRegOperand(SRC_SHARED_BASE);
  case 236: return createRegOperand(SRC_SHARED_LIMIT);
  case 237: return createRegOperand(SRC_PRIVATE_BASE);
  case 238: return createRegOperand(SRC_PRIVATE_LIMIT);
  case 239: return createRegOperand(SRC_POPS_EXITING_WAVE_ID);
  case 251: return createRegOperand(SRC_VCCZ);
  case 252: return createRegOperand(SRC_TMSKZ);
  case 253: return createRegOperand(SRC_SCC);
  default: break;
  }
  return errOperand(Val, "unknown operand encoding " + Twine(Val));
}
/*
MCOperand PPUDisassembler::decodeSDWASrc(const OpWidthTy Width,
                                            const unsigned Val) const {
  using namespace PPU::SDWA;
  using namespace PPU::EncValues;

  if (STI.getFeatureBits()[PPU::FeatureGFX9] ||
      STI.getFeatureBits()[PPU::FeatureGFX10]) {
    // XXX: cast to int is needed to avoid stupid warning:
    // compare with unsigned is always true
    if (int(SDWA9EncValues::SRC_VGPR_MIN) <= int(Val) &&
        Val <= SDWA9EncValues::SRC_VGPR_MAX) {
      return createRegOperand(getVgprClassId(Width),
                              Val - SDWA9EncValues::SRC_VGPR_MIN);
    }
    if (SDWA9EncValues::SRC_SGPR_MIN <= Val &&
        Val <= (isGFX10() ? SDWA9EncValues::SRC_SGPR_MAX_GFX10
                          : SDWA9EncValues::SRC_SGPR_MAX_SI)) {
      return createSRegOperand(getSgprClassId(Width),
                               Val - SDWA9EncValues::SRC_SGPR_MIN);
    }
    if (SDWA9EncValues::SRC_TTMP_MIN <= Val &&
        Val <= SDWA9EncValues::SRC_TTMP_MAX) {
      return createSRegOperand(getTtmpClassId(Width),
                               Val - SDWA9EncValues::SRC_TTMP_MIN);
    }

    const unsigned SVal = Val - SDWA9EncValues::SRC_SGPR_MIN;

    if (INLINE_INTEGER_C_MIN <= SVal && SVal <= INLINE_INTEGER_C_MAX)
      return decodeIntImmed(SVal);

    if (INLINE_FLOATING_C_MIN <= SVal && SVal <= INLINE_FLOATING_C_MAX)
      return decodeFPImmed(Width, SVal);

    return decodeSpecialReg32(SVal);
  } else if (STI.getFeatureBits()[PPU::FeatureVolcanicIslands]) {
    return createRegOperand(getVgprClassId(Width), Val);
  }
  llvm_unreachable("unsupported target");
}

MCOperand PPUDisassembler::decodeSDWASrc16(unsigned Val) const {
  return decodeSDWASrc(OPW16, Val);
}

MCOperand PPUDisassembler::decodeSDWASrc32(unsigned Val) const {
  return decodeSDWASrc(OPW32, Val);
}

MCOperand PPUDisassembler::decodeSDWAVopcDst(unsigned Val) const {
  using namespace PPU::SDWA;

  assert((STI.getFeatureBits()[PPU::FeatureGFX9] ||
          STI.getFeatureBits()[PPU::FeatureGFX10]) &&
         "SDWAVopcDst should be present only on GFX9+");

  bool IsWave64 = STI.getFeatureBits()[PPU::FeatureWavefrontSize64];

  if (Val & SDWA9EncValues::VOPC_DST_VCC_MASK) {
    Val &= SDWA9EncValues::VOPC_DST_SGPR_MASK;

    int TTmpIdx = getTTmpIdx(Val);
    if (TTmpIdx >= 0) {
      return createSRegOperand(getTtmpClassId(OPW64), TTmpIdx);
    } else if (Val > SGPR_MAX) {
      return IsWave64 ? decodeSpecialReg64(Val)
                      : decodeSpecialReg32(Val);
    } else {
      return createSRegOperand(getSgprClassId(IsWave64 ? OPW64 : OPW32), Val);
    }
  } else {
    return createRegOperand(IsWave64 ? PPU::VCC : PPU::VCC_LO);
  }
}
*/

MCOperand PPUDisassembler::decodeBoolReg(unsigned Val) const {
    /*
  return STI.getFeatureBits()[PPU::FeatureWavefrontSize64] ?
    decodeOperand_SReg_64(Val) : decodeOperand_SReg_32(Val);
    */
  return decodeOperand_SReg_32(Val);
}
bool PPUDisassembler::isPPT() const {
  return STI.getFeatureBits()[PPU::FeaturePPT];
}
/*
bool PPUDisassembler::isVI() const {
  return STI.getFeatureBits()[PPU::FeatureVolcanicIslands];
}

bool PPUDisassembler::isGFX9() const {
  return STI.getFeatureBits()[PPU::FeatureGFX9];
}

bool PPUDisassembler::isGFX10() const {
  return STI.getFeatureBits()[PPU::FeatureGFX10];
}
*/

//===----------------------------------------------------------------------===//
// PPUSymbolizer
//===----------------------------------------------------------------------===//

// Try to find symbol name for specified label
bool PPUSymbolizer::tryAddingSymbolicOperand(MCInst &Inst,
                                raw_ostream &/*cStream*/, int64_t Value,
                                uint64_t /*Address*/, bool IsBranch,
                                uint64_t /*Offset*/, uint64_t /*InstSize*/) {
  using SymbolInfoTy = std::tuple<uint64_t, StringRef, uint8_t>;
  using SectionSymbolsTy = std::vector<SymbolInfoTy>;

  if (!IsBranch) {
    return false;
  }

  auto *Symbols = static_cast<SectionSymbolsTy *>(DisInfo);
  if (!Symbols)
    return false;

  auto Result = std::find_if(Symbols->begin(), Symbols->end(),
                             [Value](const SymbolInfoTy& Val) {
                                return std::get<0>(Val) == static_cast<uint64_t>(Value)
                                    && std::get<2>(Val) == ELF::STT_NOTYPE;
                             });
  if (Result != Symbols->end()) {
    auto *Sym = Ctx.getOrCreateSymbol(std::get<1>(*Result));
    const auto *Add = MCSymbolRefExpr::create(Sym, Ctx);
    Inst.addOperand(MCOperand::createExpr(Add));
    return true;
  }
  return false;
}

void PPUSymbolizer::tryAddingPcLoadReferenceComment(raw_ostream &cStream,
                                                       int64_t Value,
                                                       uint64_t Address) {
  llvm_unreachable("unimplemented");
}


//===----------------------------------------------------------------------===//
// Initialization
//===----------------------------------------------------------------------===//

static MCSymbolizer *createPPUSymbolizer(const Triple &/*TT*/,
                              LLVMOpInfoCallback /*GetOpInfo*/,
                              LLVMSymbolLookupCallback /*SymbolLookUp*/,
                              void *DisInfo,
                              MCContext *Ctx,
                              std::unique_ptr<MCRelocationInfo> &&RelInfo) {
  return new PPUSymbolizer(*Ctx, std::move(RelInfo), DisInfo);
}


static MCDisassembler *createPPUDisassembler(const Target &T,
                                               const MCSubtargetInfo &STI,
                                               MCContext &Ctx) {
  return new PPUDisassembler(STI, Ctx, T.createMCInstrInfo());
}

extern "C" void LLVMInitializePPUDisassembler() {
  // Register the disassembler for each target.
  TargetRegistry::RegisterMCDisassembler(getThePPUTarget(),
                                         createPPUDisassembler);
  TargetRegistry::RegisterMCSymbolizer(getThePPUTarget(),
                                         createPPUSymbolizer);
}

