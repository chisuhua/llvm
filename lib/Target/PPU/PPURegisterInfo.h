//===-- PPURegisterInfo.h - PPU Register Information Impl ---*- C++ -*-===//
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

#ifndef LLVM_LIB_TARGET_PPU_PPUREGISTERINFO_H
#define LLVM_LIB_TARGET_PPU_PPUREGISTERINFO_H

#include "PPUDefines.h"
#include "PPURegisterBankInfo.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"

#define GET_REGINFO_HEADER
#include "PPUGenRegisterInfo.inc"

namespace llvm {

struct PPUBaseRegisterInfo : public PPUGenRegisterInfo {

  PPUBaseRegisterInfo(unsigned HwMode);

  const uint32_t *getCallPreservedMask(const MachineFunction &MF,
                                       CallingConv::ID) const override;

  const MCPhysReg *getCalleeSavedRegs(const MachineFunction *MF) const override;

  BitVector getReservedRegs(const MachineFunction &MF) const override;

  bool isConstantPhysReg(unsigned PhysReg) const override;

  const uint32_t *getNoPreservedMask() const override;

  void eliminateFrameIndex(MachineBasicBlock::iterator MI, int SPAdj,
                           unsigned FIOperandNum,
                           RegScavenger *RS = nullptr) const override;

  Register getFrameRegister(const MachineFunction &MF) const override;

  bool requiresRegisterScavenging(const MachineFunction &MF) const override {
    return true;
  }

  bool requiresFrameIndexScavenging(const MachineFunction &MF) const override {
    return true;
  }

  bool trackLivenessAfterRegAlloc(const MachineFunction &) const override {
    return true;
  }

  const TargetRegisterClass *
  getPointerRegClass(const MachineFunction &MF,
                     unsigned Kind = 0) const override {
    return &PPU::GPRRegClass;
  }

  unsigned getVCC() const;
};

class PPUSubtarget;
class LiveIntervals;
class MachineRegisterInfo;
class PPUMachineFunctionInfo;

class PPURegisterInfo final : public PPUBaseRegisterInfo {
private:
  const PPUSubtarget &ST;
  unsigned SGPRSetID;
  unsigned VGPRSetID;
  BitVector SGPRPressureSets;
  BitVector VGPRPressureSets;
  bool SpillSGPRToVGPR;
  bool SpillSGPRToSMEM;
  bool isWave32 {true};

  void classifyPressureSet(unsigned PSetID, unsigned Reg,
                           BitVector &PressureSets) const;
public:
  PPURegisterInfo(const PPUSubtarget &ST, unsigned HwMode);

  // TODO schi below two is copied from AMGGPURegisterInfo.h
  /// \returns the sub reg enum value for the given \p Channel
  /// (e.g. getSubRegFromChannel(0) -> AMDGPU::sub0)
  static unsigned getSubRegFromChannel(unsigned Channel);

  void reserveRegisterTuples(BitVector &, unsigned Reg) const;
  // 

  bool spillSGPRToVGPR() const {
    return SpillSGPRToVGPR;
  }

  bool spillSGPRToSMEM() const {
    return SpillSGPRToSMEM;
  }

  /// Return the end register initially reserved for the scratch buffer in case
  /// spilling is needed.
  unsigned reservedPrivateSegmentBufferReg(const MachineFunction &MF) const;

  /// Return the end register initially reserved for the scratch wave offset in
  /// case spilling is needed.
  unsigned reservedPrivateSegmentWaveByteOffsetReg(
    const MachineFunction &MF) const;

  BitVector getReservedRegs(const MachineFunction &MF) const override;

  // const MCPhysReg *getCalleeSavedRegs(const MachineFunction *MF) const override;
  const MCPhysReg *getCalleeSavedRegsViaCopy(const MachineFunction *MF) const;
  const uint32_t *getCallPreservedMask(const MachineFunction &MF,
                                       CallingConv::ID CC) const override
  {
      PPUBaseRegisterInfo::getCallPreservedMask(MF, CC);
  }

  // Stack access is very expensive. CSRs are also the high registers, and we
  // want to minimize the number of used registers.
  unsigned getCSRFirstUseCost() const override {
    return 100;
  }

  bool canRealignStack(const MachineFunction &MF) const override;
  bool requiresRegisterScavenging(const MachineFunction &Fn) const override;

  bool requiresFrameIndexScavenging(const MachineFunction &MF) const override;
  bool requiresFrameIndexReplacementScavenging(
    const MachineFunction &MF) const override;
  bool requiresVirtualBaseRegisters(const MachineFunction &Fn) const override;
  bool trackLivenessAfterRegAlloc(const MachineFunction &MF) const override;

  int64_t getMUBUFInstrOffset(const MachineInstr *MI) const;

  // int64_t getFrameIndexInstrOffset(const MachineInstr *MI,
  //                                 int Idx) const override;

  // bool needsFrameBaseReg(MachineInstr *MI, int64_t Offset) const override;

  void materializeFrameBaseRegister(MachineBasicBlock *MBB,
                                    unsigned BaseReg, int FrameIdx,
                                    int64_t Offset) const override;

  void resolveFrameIndex(MachineInstr &MI, unsigned BaseReg,
                         int64_t Offset) const override;

  // bool isFrameOffsetLegal(const MachineInstr *MI, unsigned BaseReg,
  //                        int64_t Offset) const override;

  const TargetRegisterClass *getPointerRegClass(
    const MachineFunction &MF, unsigned Kind = 0) const override;

  /// If \p OnlyToVGPR is true, this will only succeed if this
  bool spillSGPR(MachineBasicBlock::iterator MI,
                 int FI, RegScavenger *RS,
                 bool OnlyToVGPR = false) const;

  bool restoreSGPR(MachineBasicBlock::iterator MI,
                   int FI, RegScavenger *RS,
                   bool OnlyToVGPR = false) const;

  void eliminateFrameIndex(MachineBasicBlock::iterator MI, int SPAdj,
                           unsigned FIOperandNum,
                           RegScavenger *RS) const override;

  bool eliminateSGPRToVGPRSpillFrameIndex(MachineBasicBlock::iterator MI,
                                          int FI, RegScavenger *RS) const;

  StringRef getRegAsmName(unsigned Reg) const override;

  unsigned getHWRegIndex(unsigned Reg) const {
    return getEncodingValue(Reg) & 0xff;
  }

  /// Return the 'base' register class for this register.
  /// e.g. SGPR0 => SReg_32, VGPR => VGPR_32 SGPR0_SGPR1 -> SReg_32, etc.
  const TargetRegisterClass *getPhysRegClass(unsigned Reg) const;

  /// \returns true if this class contains only SGPR registers
  bool isSGPRClass(const TargetRegisterClass *RC) const {
    return !hasVGPRs(RC);
  }

  /// \returns true if this class ID contains only SGPR registers
  bool isSGPRClassID(unsigned RCID) const {
    return isSGPRClass(getRegClass(RCID));
  }

  bool isSGPRReg(const MachineRegisterInfo &MRI, unsigned Reg) const {
    const TargetRegisterClass *RC;
    if (Register::isVirtualRegister(Reg))
      RC = MRI.getRegClass(Reg);
    else
      RC = getPhysRegClass(Reg);
    return isSGPRClass(RC);
  }

  /// \returns true if this class contains VGPR registers.
  bool hasVGPRs(const TargetRegisterClass *RC) const;

  /// \returns true if this class contains any vector registers.
  bool hasVectorRegisters(const TargetRegisterClass *RC) const {
    return hasVGPRs(RC);
  }

  /// \returns A VGPR reg class with the same width as \p SRC
  const TargetRegisterClass *getEquivalentVGPRClass(
                                          const TargetRegisterClass *SRC) const;

  /// \returns A SGPR reg class with the same width as \p SRC
  const TargetRegisterClass *getEquivalentSGPRClass(
                                           const TargetRegisterClass *VRC) const;

  /// \returns The register class that is used for a sub-register of \p RC for
  /// the given \p SubIdx.  If \p SubIdx equals NoSubRegister, \p RC will
  /// be returned.
  const TargetRegisterClass *getSubRegClass(const TargetRegisterClass *RC,
                                            unsigned SubIdx) const;

  bool shouldRewriteCopySrc(const TargetRegisterClass *DefRC,
                            unsigned DefSubReg,
                            const TargetRegisterClass *SrcRC,
                            unsigned SrcSubReg) const override;

  /// \returns True if operands defined with this operand type can accept
  /// a literal constant (i.e. any 32-bit immediate).
  bool opCanUseLiteralConstant(unsigned OpType) const {
    // TODO: 64-bit operands have extending behavior from 32-bit literal.
    return OpType >= PPU::OPERAND_REG_IMM_FIRST &&
           OpType <= PPU::OPERAND_REG_IMM_LAST;
  }

  /// \returns True if operands defined with this operand type can accept
  /// an inline constant. i.e. An integer value in the range (-16, 64) or
  /// -4.0f, -2.0f, -1.0f, -0.5f, 0.0f, 0.5f, 1.0f, 2.0f, 4.0f.
  bool opCanUseInlineConstant(unsigned OpType) const;

  unsigned findUnusedRegister(const MachineRegisterInfo &MRI,
                              const TargetRegisterClass *RC,
                              const MachineFunction &MF) const;

  unsigned getSGPRPressureSet() const { return SGPRSetID; };
  unsigned getVGPRPressureSet() const { return VGPRSetID; };

  const TargetRegisterClass *getRegClassForReg(const MachineRegisterInfo &MRI,
                                               unsigned Reg) const;
  bool isVGPR(const MachineRegisterInfo &MRI, unsigned Reg) const;
  bool isVectorRegister(const MachineRegisterInfo &MRI, unsigned Reg) const {
    return isVGPR(MRI, Reg) ;
  }

  virtual bool
  isDivergentRegClass(const TargetRegisterClass *RC) const override {
    return !isSGPRClass(RC);
  }

  bool isSGPRPressureSet(unsigned SetID) const {
    return SGPRPressureSets.test(SetID) && !VGPRPressureSets.test(SetID);
  }
  bool isVGPRPressureSet(unsigned SetID) const {
    return VGPRPressureSets.test(SetID) && !SGPRPressureSets.test(SetID);
  }

  ArrayRef<int16_t> getRegSplitParts(const TargetRegisterClass *RC,
                                     unsigned EltSize) const;

  bool shouldCoalesce(MachineInstr *MI,
                      const TargetRegisterClass *SrcRC,
                      unsigned SubReg,
                      const TargetRegisterClass *DstRC,
                      unsigned DstSubReg,
                      const TargetRegisterClass *NewRC,
                      LiveIntervals &LIS) const override;

  unsigned getRegPressureLimit(const TargetRegisterClass *RC,
                               MachineFunction &MF) const override;

  unsigned getRegPressureSetLimit(const MachineFunction &MF,
                                  unsigned Idx) const override;

  const int *getRegUnitPressureSets(unsigned RegUnit) const override;

  unsigned getReturnAddressReg(const MachineFunction &MF) const;

  const TargetRegisterClass *
  getRegClassForSizeOnBank(unsigned Size,
                           const RegisterBank &Bank,
                           const MachineRegisterInfo &MRI) const;

  const TargetRegisterClass *
  getRegClassForTypeOnBank(LLT Ty,
                           const RegisterBank &Bank,
                           const MachineRegisterInfo &MRI) const {
    return getRegClassForSizeOnBank(Ty.getSizeInBits(), Bank, MRI);
  }

  const TargetRegisterClass *
  getConstrainedRegClassForOperand(const MachineOperand &MO,
                                 const MachineRegisterInfo &MRI) const override;

  const TargetRegisterClass *getBoolRC() const {
    // FIXME return &PPU::SReg_32_XM0RegClass
    return &PPU::GPRRegClass;
  }

  const TargetRegisterClass *getWaveMaskRegClass() const {
    // return &PPU::SReg_32_XM0_XEXECRegClass
    return &PPU::GPRRegClass;
  }

  unsigned getVCC() const;

  const TargetRegisterClass *getRegClass(unsigned RCID) const;

  // Find reaching register definition
  MachineInstr *findReachingDef(unsigned Reg, unsigned SubReg,
                                MachineInstr &Use,
                                MachineRegisterInfo &MRI,
                                LiveIntervals *LIS) const;

  const uint32_t *getAllVGPRRegMask() const;
  const uint32_t *getAllAllocatableSRegMask() const;

private:
  void buildSpillLoadStore(MachineBasicBlock::iterator MI,
                           unsigned LoadStoreOp,
                           int Index,
                           unsigned ValueReg,
                           bool ValueIsKill,
                           unsigned ScratchRsrcReg,
                           unsigned ScratchOffsetReg,
                           int64_t InstrOffset,
                           MachineMemOperand *MMO,
                           RegScavenger *RS) const;
};


}

#endif
