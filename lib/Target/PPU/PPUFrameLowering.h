//===-- PPUFrameLowering.h - Define frame lowering for PPU -*- C++ -*--===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
/// \file
/// Interface to describe a layout of a stack frame on an PPU target.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_PPU_PPUFRAMELOWERING_H
#define LLVM_LIB_TARGET_PPU_PPUFRAMELOWERING_H

#include "llvm/CodeGen/TargetFrameLowering.h"

namespace llvm {

class PPUInstrInfo;
class PPUMachineFunctionInfo;
class PPURegisterInfo;
class PPUSubtarget;

class PPUFrameLowering : public TargetFrameLowering {
public:
  explicit PPUFrameLowering(const PPUSubtarget &STI,
                            StackDirection D = StackGrowsDown,
                            unsigned StackAl = /*StackALignment=*/16,
                            int LAO = /*LocalAreaOffset=*/0,
                            unsigned TransAl = 1)
    : TargetFrameLowering(D, StackAl, LAO, TransAl)
    , STI(STI) {}

  ~PPUFrameLowering() override = default;

  // AMD
  void emitEntryFunctionPrologue(MachineFunction &MF, MachineBasicBlock &MBB) const;
  void emitPrologue_compute(MachineFunction &MF, MachineBasicBlock &MBB) const ;
  void emitEpilogue_compute(MachineFunction &MF, MachineBasicBlock &MBB) const ;
  int getFrameIndexReference_compute(const MachineFunction &MF, int FI,
                             unsigned &FrameReg) const ;
  void determineCalleeSaves_compute(MachineFunction &MF, BitVector &SavedRegs,
                            RegScavenger *RS = nullptr) const ;
  void processFunctionBeforeFrameFinalized_compute(MachineFunction &MF,
                                           RegScavenger *RS = nullptr) const ;
  bool hasFP_compute(const MachineFunction &MF) const ;
  bool hasSP_compute(const MachineFunction &MF) const ;

  MachineBasicBlock::iterator eliminateCallFramePseudoInstr_compute(
          MachineFunction &MF, MachineBasicBlock &MBB,
          MachineBasicBlock::iterator MI) const ;
  // end AMD

  void emitPrologue(MachineFunction &MF, MachineBasicBlock &MBB) const override;
  void emitEpilogue(MachineFunction &MF, MachineBasicBlock &MBB) const override;
  int getFrameIndexReference(const MachineFunction &MF, int FI,
                             unsigned &FrameReg) const override;
  void determineCalleeSaves(MachineFunction &MF, BitVector &SavedRegs,
                            RegScavenger *RS = nullptr) const override;

  void processFunctionBeforeFrameFinalized(MachineFunction &MF,
                                           RegScavenger *RS = nullptr) const override;

  bool hasFP(const MachineFunction &MF) const override;


  MachineBasicBlock::iterator eliminateCallFramePseudoInstr(
          MachineFunction &MF, MachineBasicBlock &MBB,
          MachineBasicBlock::iterator MI) const override;

  bool hasReservedCallFrame(const MachineFunction &MF) const override;

protected:
  const PPUSubtarget &STI;

private:
  void determineFrameLayout(MachineFunction &MF) const;
  void adjustReg(MachineBasicBlock &MBB, MachineBasicBlock::iterator MBBI,
                 const DebugLoc &DL, Register DestReg, Register SrcReg,
                 int64_t Val, MachineInstr::MIFlag Flag) const;
  bool shouldEnableVectorUnit(MachineFunction &MF) const;

public: // from AMD
  /// \returns The number of 32-bit sub-registers that are used when storing
  /// values to the stack.
  unsigned getStackWidth(const MachineFunction &MF) const;

  void determineCalleeSavesSGPR(MachineFunction &MF, BitVector &SavedRegs,
                                RegScavenger *RS = nullptr) const;

  bool assignCalleeSavedSpillSlots(MachineFunction &MF,
                              const TargetRegisterInfo *TRI,
                              std::vector<CalleeSavedInfo> &CSI) const override;
  bool isSupportedStackID(TargetStackID::Value ID) const override;


private:
  void emitFlatScratchInit(const PPUSubtarget &ST,
                           MachineFunction &MF,
                           MachineBasicBlock &MBB) const;

  unsigned getReservedPrivateSegmentBufferReg(
    const PPUSubtarget &ST,
    const PPUInstrInfo *TII,
    const PPURegisterInfo *TRI,
    PPUMachineFunctionInfo *MFI,
    MachineFunction &MF) const;

  std::pair<unsigned, bool> getReservedPrivateSegmentWaveByteOffsetReg(
      const PPUSubtarget &ST, const PPUInstrInfo *TII, const PPURegisterInfo *TRI,
      PPUMachineFunctionInfo *MFI, MachineFunction &MF) const;

  // Emit scratch setup code for AMDPAL or Mesa, assuming ResourceRegUsed is set.
  void emitEntryFunctionScratchSetup(const PPUSubtarget &ST, MachineFunction &MF,
      MachineBasicBlock &MBB, PPUMachineFunctionInfo *MFI,
      MachineBasicBlock::iterator I, unsigned PreloadedPrivateBufferReg,
      unsigned ScratchRsrcReg) const;

};
}
#endif
