//===-- PPURegisterInfo.cpp - PPU Register Information ------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the PPU implementation of the TargetRegisterInfo class.
//
//===----------------------------------------------------------------------===//

#include "PPURegisterInfo.h"
#include "PPU.h"
#include "PPUSubtarget.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/RegisterScavenging.h"
#include "llvm/CodeGen/TargetFrameLowering.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
#include "llvm/Support/ErrorHandling.h"

#define GET_REGINFO_TARGET_DESC
#include "PPUGenRegisterInfo.inc"

using namespace llvm;

PPURegisterInfo::PPURegisterInfo(unsigned HwMode)
    : PPUGenRegisterInfo(PPU::X1, /*DwarfFlavour*/0, /*EHFlavor*/0,
                           /*PC*/0, HwMode) {}

const MCPhysReg *
PPURegisterInfo::getCalleeSavedRegs(const MachineFunction *MF) const {
  auto &Subtarget = MF->getSubtarget<PPUSubtarget>();
  if (MF->getFunction().hasFnAttribute("interrupt")) {
    if (Subtarget.hasStdExtD())
      return CSR_XLEN_F64_Interrupt_SaveList;
    if (Subtarget.hasStdExtF())
      return CSR_XLEN_F32_Interrupt_SaveList;
    return CSR_Interrupt_SaveList;
  }

  switch (Subtarget.getTargetABI()) {
  default:
    llvm_unreachable("Unrecognized ABI");
  case PPUABI::ABI_ILP32:
  case PPUABI::ABI_LP64:
    return CSR_ILP32_LP64_SaveList;
  case PPUABI::ABI_ILP32F:
  case PPUABI::ABI_LP64F:
    return CSR_ILP32F_LP64F_SaveList;
  case PPUABI::ABI_ILP32D:
  case PPUABI::ABI_LP64D:
    return CSR_ILP32D_LP64D_SaveList;
  }
}

BitVector PPURegisterInfo::getReservedRegs(const MachineFunction &MF) const {
  const TargetFrameLowering *TFI = getFrameLowering(MF);
  BitVector Reserved(getNumRegs());

  // Use markSuperRegs to ensure any register aliases are also reserved
  markSuperRegs(Reserved, PPU::X0); // zero
  markSuperRegs(Reserved, PPU::X1); // ra
  markSuperRegs(Reserved, PPU::X2); // sp
  markSuperRegs(Reserved, PPU::X3); // gp
  markSuperRegs(Reserved, PPU::X4); // tp
  if (TFI->hasFP(MF))
    markSuperRegs(Reserved, PPU::X8); // fp
  assert(checkAllSuperRegsMarked(Reserved));
  return Reserved;
}

bool PPURegisterInfo::isConstantPhysReg(unsigned PhysReg) const {
  return PhysReg == PPU::X0;
}

const uint32_t *PPURegisterInfo::getNoPreservedMask() const {
  return CSR_NoRegs_RegMask;
}

void PPURegisterInfo::eliminateFrameIndex(MachineBasicBlock::iterator II,
                                            int SPAdj, unsigned FIOperandNum,
                                            RegScavenger *RS) const {
  assert(SPAdj == 0 && "Unexpected non-zero SPAdj value");

  MachineInstr &MI = *II;
  MachineFunction &MF = *MI.getParent()->getParent();
  MachineRegisterInfo &MRI = MF.getRegInfo();
  const PPUInstrInfo *TII = MF.getSubtarget<PPUSubtarget>().getInstrInfo();
  DebugLoc DL = MI.getDebugLoc();

  int FrameIndex = MI.getOperand(FIOperandNum).getIndex();
  unsigned FrameReg;
  int Offset =
      getFrameLowering(MF)->getFrameIndexReference(MF, FrameIndex, FrameReg) +
      MI.getOperand(FIOperandNum + 1).getImm();

  if (!isInt<32>(Offset)) {
    report_fatal_error(
        "Frame offsets outside of the signed 32-bit range not supported");
  }

  MachineBasicBlock &MBB = *MI.getParent();
  bool FrameRegIsKill = false;

  if (!isInt<12>(Offset)) {
    assert(isInt<32>(Offset) && "Int32 expected");
    // The offset won't fit in an immediate, so use a scratch register instead
    // Modify Offset and FrameReg appropriately
    Register ScratchReg = MRI.createVirtualRegister(&PPU::GPRRegClass);
    TII->movImm32(MBB, II, DL, ScratchReg, Offset);
    BuildMI(MBB, II, DL, TII->get(PPU::ADD), ScratchReg)
        .addReg(FrameReg)
        .addReg(ScratchReg, RegState::Kill);
    Offset = 0;
    FrameReg = ScratchReg;
    FrameRegIsKill = true;
  }

  MI.getOperand(FIOperandNum)
      .ChangeToRegister(FrameReg, false, false, FrameRegIsKill);
  MI.getOperand(FIOperandNum + 1).ChangeToImmediate(Offset);
}

Register PPURegisterInfo::getFrameRegister(const MachineFunction &MF) const {
  const TargetFrameLowering *TFI = getFrameLowering(MF);
  return TFI->hasFP(MF) ? PPU::X8 : PPU::X2;
}

const uint32_t *
PPURegisterInfo::getCallPreservedMask(const MachineFunction & MF,
                                        CallingConv::ID /*CC*/) const {
  auto &Subtarget = MF.getSubtarget<PPUSubtarget>();
  if (MF.getFunction().hasFnAttribute("interrupt")) {
    if (Subtarget.hasStdExtD())
      return CSR_XLEN_F64_Interrupt_RegMask;
    if (Subtarget.hasStdExtF())
      return CSR_XLEN_F32_Interrupt_RegMask;
    return CSR_Interrupt_RegMask;
  }

  switch (Subtarget.getTargetABI()) {
  default:
    llvm_unreachable("Unrecognized ABI");
  case PPUABI::ABI_ILP32:
  case PPUABI::ABI_LP64:
    return CSR_ILP32_LP64_RegMask;
  case PPUABI::ABI_ILP32F:
  case PPUABI::ABI_LP64F:
    return CSR_ILP32F_LP64F_RegMask;
  case PPUABI::ABI_ILP32D:
  case PPUABI::ABI_LP64D:
    return CSR_ILP32D_LP64D_RegMask;
  }
}
