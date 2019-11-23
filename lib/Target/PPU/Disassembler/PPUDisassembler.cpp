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

#include "MCTargetDesc/PPUMCTargetDesc.h"
#include "TargetInfo/PPUTargetInfo.h"
#include "Utils/PPUBaseInfo.h"
#include "llvm/CodeGen/Register.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCDisassembler/MCDisassembler.h"
#include "llvm/MC/MCFixedLenDisassembler.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/Support/Endian.h"
#include "llvm/Support/TargetRegistry.h"

using namespace llvm;

#define DEBUG_TYPE "ppu-disassembler"

typedef MCDisassembler::DecodeStatus DecodeStatus;

namespace {
class PPUDisassembler : public MCDisassembler {

public:
  PPUDisassembler(const MCSubtargetInfo &STI, MCContext &Ctx)
      : MCDisassembler(STI, Ctx) {}

  DecodeStatus getInstruction(MCInst &Instr, uint64_t &Size,
                              ArrayRef<uint8_t> Bytes, uint64_t Address,
                              raw_ostream &VStream,
                              raw_ostream &CStream) const override;
};
} // end anonymous namespace

static MCDisassembler *createPPUDisassembler(const Target &T,
                                               const MCSubtargetInfo &STI,
                                               MCContext &Ctx) {
  return new PPUDisassembler(STI, Ctx);
}

extern "C" void LLVMInitializePPUDisassembler() {
  // Register the disassembler for each target.
  TargetRegistry::RegisterMCDisassembler(getThePPUTarget(),
                                         createPPUDisassembler);
}

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

static DecodeStatus DecodeGPRCRegisterClass(MCInst &Inst, uint64_t RegNo,
                                            uint64_t Address,
                                            const void *Decoder) {
  if (RegNo > 8)
    return MCDisassembler::Fail;

  Register Reg = GPRDecoderTable[RegNo + 8];
  Inst.addOperand(MCOperand::createReg(Reg));
  return MCDisassembler::Success;
}

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

#include "PPUGenDisassemblerTables.inc"

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
