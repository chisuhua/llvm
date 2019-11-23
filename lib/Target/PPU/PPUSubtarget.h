//===-- PPUSubtarget.h - Define Subtarget for the PPU -------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file declares the PPU specific subclass of TargetSubtargetInfo.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_PPU_PPUSUBTARGET_H
#define LLVM_LIB_TARGET_PPU_PPUSUBTARGET_H

#include "PPUFrameLowering.h"
#include "PPUISelLowering.h"
#include "PPUInstrInfo.h"
#include "Utils/PPUBaseInfo.h"
#include "llvm/CodeGen/GlobalISel/CallLowering.h"
#include "llvm/CodeGen/GlobalISel/InstructionSelector.h"
#include "llvm/CodeGen/GlobalISel/LegalizerInfo.h"
#include "llvm/CodeGen/GlobalISel/RegisterBankInfo.h"
#include "llvm/CodeGen/SelectionDAGTargetInfo.h"
#include "llvm/CodeGen/TargetSubtargetInfo.h"
#include "llvm/IR/DataLayout.h"
#include "llvm/Target/TargetMachine.h"

#define GET_SUBTARGETINFO_HEADER
#include "PPUGenSubtargetInfo.inc"

namespace llvm {
class StringRef;


// base on AMDGPUSubtarget
class PPUBaseSubtarget : public PPUGenSubtargetInfo {
public:
  enum Generation {
    PPU = 1,
  };

private:
  Triple TargetTriple;

protected:
  // const FeatureBitset &SubtargetFeatureBits;
  bool Has16BitInsts {false};
  bool HasMadMixInsts {false};
  bool FP32Denormals {false};
  bool FPExceptions {false};
  bool HasSDWA {false};
  bool HasVOP3PInsts {false};
  bool HasMulI24 {true};
  bool HasMulU24 {true};
  bool HasInv2PiInlineImm {false};

  bool HasFminFmaxLegacy {true};


  // Used as options.
  /*
  bool EnablePromoteAlloca {false};
  bool EnablePPUScheduler {false};
  bool DumpCode {false};
  */

  // bool HasTrigReducedRange {false};
  unsigned MaxWavesPerEU {10};
  int LocalMemorySize {0};
  unsigned WavefrontSize {32};
  static unsigned CUJITThreadsPerBlock;

public:
  PPUBaseSubtarget(const Triple &TT, StringRef CPU, StringRef FS);
/* move to PPUSubTarget subclass
  static const PPUBaseSubtarget &get(const MachineFunction &MF);
  static const PPUBaseSubtarget &get(const TargetMachine &TM, const Function &F);
*/
  /// \returns Default range flat work group size for a calling convention.
  std::pair<unsigned, unsigned> getDefaultFlatWorkGroupSize(CallingConv::ID CC) const;
  /// \returns Subtarget's default pair of minimum/maximum flat work group sizes
  /// for function \p F, or minimum/maximum flat work group sizes explicitly
  /// requested using "amdgpu-flat-work-group-size" attribute attached to
  /// function \p F.
  ///
  /// \returns Subtarget's default values if explicitly requested values cannot
  /// be converted to integer, or violate subtarget's specifications.
  std::pair<unsigned, unsigned> getFlatWorkGroupSizes(const Function &F) const;

  /// \returns Subtarget's default pair of minimum/maximum number of waves per
  /// execution unit for function \p F, or minimum/maximum number of waves per
  /// execution unit explicitly requested using "amdgpu-waves-per-eu" attribute
  /// attached to function \p F.
  ///
  /// \returns Subtarget's default values if explicitly requested values cannot
  /// be converted to integer, violate subtarget's specifications, or are not
  /// compatible with minimum/maximum number of waves limited by flat work group
  /// size, register usage, and/or lds usage.
  std::pair<unsigned, unsigned> getWavesPerEU(const Function &F) const;

  /// Return the amount of LDS that can be used that will not restrict the
  /// occupancy lower than WaveCount.
  unsigned getMaxLocalMemSizeWithWaveCount(unsigned WaveCount,
                                           const Function &) const;

  /// Inverse of getMaxLocalMemWithWaveCount. Return the maximum wavecount if
  /// the given LDS memory size is the only constraint.
  unsigned getOccupancyWithLocalMemSize(uint32_t Bytes, const Function &) const;

  unsigned getOccupancyWithLocalMemSize(const MachineFunction &MF) const;

  bool isPPSOS() const {
    return TargetTriple.getOS() == Triple::PPS;
  }

  bool has16BitInsts() const { return Has16BitInsts;
  }

  bool hasMadMixInsts() const { return HasMadMixInsts; }

  bool hasFP32Denormals() const { return FP32Denormals; }

  bool hasFPExceptions() const { return FPExceptions; }

//   bool hasSDWA() const { return HasSDWA; }

  bool hasVOP3PInsts() const { return HasVOP3PInsts; }

  bool hasMulI24() const { return HasMulI24; }

  bool hasMulU24() const { return HasMulU24; }

  bool hasFminFmaxLegacy() const { return HasFminFmaxLegacy; }


  unsigned getWavefrontSize() const { return WavefrontSize; }

  int getLocalMemorySize() const { return LocalMemorySize; }

  unsigned getAlignmentForImplicitArgPtr() const {
    // FIXME return isAmdHsaOS() ? 8 : 4;
    return 8;
  }

  /// Returns the offset in bytes from the start of the input buffer
  ///        of the first explicit kernel argument.
  unsigned getExplicitKernelArgOffset(const Function &F) const {
    // FIXME return isAmdHsaOrMesa(F) ? 0 : 36;
    return 0;
  }

  /// \returns Maximum number of work groups per compute unit supported by the
  /// subtarget and limited by given \p FlatWorkGroupSize.
  unsigned getMaxWorkGroupsPerCU(unsigned FlatWorkGroupSize) const {
    return PPU::IsaInfo::getMaxWorkGroupsPerCU(this, FlatWorkGroupSize);
  }

  /// \returns Minimum flat work group size supported by the subtarget.
  unsigned getMinFlatWorkGroupSize() const {
    return PPU::IsaInfo::getMinFlatWorkGroupSize(this);
  }

  /// \returns Maximum flat work group size supported by the subtarget.
  unsigned getMaxFlatWorkGroupSize() const {
    return PPU::IsaInfo::getMaxFlatWorkGroupSize(this);
  }

  /// \returns Maximum number of waves per execution unit supported by the
  /// subtarget and limited by given \p FlatWorkGroupSize.
  unsigned getMaxWavesPerEU(unsigned FlatWorkGroupSize) const {
    return PPU::IsaInfo::getMaxWavesPerEU(this, FlatWorkGroupSize);
  }

  /// \returns Minimum number of waves per execution unit supported by the
  /// subtarget.
  unsigned getMinWavesPerEU() const {
    return PPU::IsaInfo::getMinWavesPerEU(this);
  }

  /// \returns Maximum number of waves per execution unit supported by the
  /// subtarget without any kind of limitation.
  unsigned getMaxWavesPerEU() const { return MaxWavesPerEU; }

  /// Creates value range metadata on an workitemid.* inrinsic call or load.
  bool makeLIDRangeMetadata(Instruction *I) const;

  /// \returns Number of bytes of arguments that are passed to a shader or
  /// kernel in addition to the explicit ones declared for the function.
  unsigned getImplicitArgNumBytes(const Function &F) const {
    return PPU::getIntegerAttribute(F, "ppu-implicitarg-num-bytes", 0);
  }
  uint64_t getExplicitKernArgSize(const Function &F, unsigned &MaxAlign) const;
  unsigned getKernArgSegmentSize(const Function &F, unsigned &MaxAlign) const;

  virtual ~PPUBaseSubtarget() {}
};


// Base on GCNSubtarget and RISCVSubtarget
class PPUSubtarget : public PPUBaseSubtarget
{
  using PPUBaseSubtarget::getExplicitKernArgSize;
  virtual void anchor();
  bool HasStdExtM = false;
  bool HasStdExtA = false;
  bool HasStdExtF = false;
  bool HasStdExtD = false;
  bool HasStdExtC = false;
  bool HasStdExtV = false;
  bool HasRV64 = false;
  bool IsRV32E = false;
  bool EnableLinkerRelax = false;
  bool EnableRVCHintInstrs = false;
  bool EnableReconvergeCFG = false;
  unsigned XLen = 32;
  MVT XLenVT = MVT::i32;
  PPUABI::ABI TargetABI = PPUABI::ABI_Unknown;
  PPUFrameLowering FrameLowering;
  PPUInstrInfo InstrInfo;
  PPURegisterInfo RegInfo;
  PPUTargetLowering TLInfo;
  SelectionDAGTargetInfo TSInfo;

  /// Initializes using the passed in CPU and feature strings so that we can
  /// use initializer lists for subtarget initialization.
  PPUSubtarget &initializeSubtargetDependencies(const Triple &TT,
                                                  StringRef CPU, StringRef FS,
                                                  StringRef ABIName);

public:
  // Initializes the data members to match that of the specified triple.
  PPUSubtarget(const Triple &TT, StringRef CPU, StringRef FS,
                 StringRef ABIName, const TargetMachine &TM);

  static const PPUSubtarget &get(const MachineFunction &MF);
  static const PPUSubtarget &get(const TargetMachine &TM, const Function &F);

  // Parses features string setting specified subtarget options. The
  // definition of this function is auto-generated by tblgen.
  void ParseSubtargetFeatures(StringRef CPU, StringRef FS);

  const PPUFrameLowering *getFrameLowering() const override {
    return &FrameLowering;
  }
  const PPUInstrInfo *getInstrInfo() const override { return &InstrInfo; }
  const PPURegisterInfo *getRegisterInfo() const override { return &RegInfo; }
  const PPUTargetLowering *getTargetLowering() const override { return &TLInfo; }
  const SelectionDAGTargetInfo *getSelectionDAGInfo() const override { return &TSInfo; }
  bool hasStdExtM() const { return HasStdExtM; }
  bool hasStdExtA() const { return HasStdExtA; }
  bool hasStdExtF() const { return HasStdExtF; }
  bool hasStdExtD() const { return HasStdExtD; }
  bool hasStdExtC() const { return HasStdExtC; }
  bool hasStdExtV() const { return HasStdExtV; }
  bool is64Bit() const { return HasRV64; }
  bool isRV32E() const { return IsRV32E; }
  bool enableLinkerRelax() const { return EnableLinkerRelax; }
  bool enableRVCHintInstrs() const { return EnableRVCHintInstrs; }
  bool enableReconvergeCFG() const { return EnableReconvergeCFG; }
  MVT getXLenVT() const { return XLenVT; }
  unsigned getXLen() const { return XLen; }
  PPUABI::ABI getTargetABI() const { return TargetABI; }

protected:
  // GlobalISel related APIs.
  std::unique_ptr<CallLowering> CallLoweringInfo;
  std::unique_ptr<InstructionSelector> InstSelector;
  std::unique_ptr<LegalizerInfo> Legalizer;
  std::unique_ptr<RegisterBankInfo> RegBankInfo;

public:
  const CallLowering *getCallLowering() const override;
  InstructionSelector *getInstructionSelector() const override;
  const LegalizerInfo *getLegalizerInfo() const override;
  const RegisterBankInfo *getRegBankInfo() const override;

// FIXME
// Below is copied from AMDGPUSubTarget.h
protected:
  // unsigned Gen;
  // TODO InstrItineraryData InstrItins;
  int LDSBankCount {0};
  unsigned MaxPrivateElementSize {0};

  // Possibly statically set by tablegen, but may want to be overridden.
  bool FastFMAF32 {false};
  bool HalfRate64Ops {false};

  // Dynamially set bits that enable features.
  bool FP64FP16Denormals {false};
  bool AutoWaitcntBeforeBarrier {false};
  bool UnalignedScratchAccess {false};
  bool UnalignedBufferAccess {false};
  bool HasApertureRegs {false};
  /*
  bool EnableXNACK;
  bool DoesNotSupportXNACK;
  bool EnableCuMode;
  bool TrapHandler;

  // Used as options.

  bool EnableLoadStoreOpt;
  bool EnableUnsafeDSOffsetFolding;
  */
  bool FlatForGlobal {false};
  bool CodeObjectV3 {false};
  bool EnablePromoteAlloca {false};
  bool EnablePPUScheduler {false};
  bool DumpCode {false};
  // bool EnableDS128;
  // bool EnablePRTStrictNull;
/*
  // Subtarget statically properties set by tablegen
  bool FP64;
  bool FMA;
  bool MIMG_R128;
  bool IsGCN;
  bool GCN3Encoding;
  bool CIInsts;
  bool GFX8Insts;
  bool GFX9Insts;
  bool GFX10Insts;
  bool GFX7GFX8GFX9Insts;
  bool SGPRInitBug;
  bool HasSMemRealTime;
  bool HasIntClamp;
  */
  bool HasFmaMixInsts {false};
  /*
  bool HasMovrel;
  bool HasVGPRIndexMode;
  bool HasScalarStores;
  bool HasScalarAtomics;
  bool HasSDWAOmod;
  bool HasSDWAScalar;
  bool HasSDWASdst;
  bool HasSDWAMac;
  bool HasSDWAOutModsVOPC;
  bool HasDPP;
  bool HasDPP8;
  bool HasR128A16;
  bool HasNSAEncoding;
  bool HasDLInsts;
  bool HasDot1Insts;
  bool HasDot2Insts;
  bool HasDot3Insts;
  bool HasDot4Insts;
  bool HasDot5Insts;
  bool HasDot6Insts;
  bool HasPkFmacF16Inst;
  bool HasAtomicFaddInsts;
  bool EnableSRAMECC;
  bool DoesNotSupportSRAMECC;
  bool HasNoSdstCMPX;
  bool HasVscnt;
  bool HasVOP3Literal;
  bool HasNoDataDepHazard;
  */
  bool HasRegisterBanking;
  bool HasMAIInsts;
  bool FlatAddressSpace;
  bool FlatInstOffsets;
  bool FlatGlobalInsts;
  /*
  bool FlatScratchInsts;
  bool ScalarFlatScratchInsts;
  bool AddNoCarryInsts;
  bool HasUnpackedD16VMem;
  bool R600ALUInst;
  bool CaymanISA;
  bool CFALUBug;
  bool LDSMisalignedBug;
  bool HasMFMAInlineLiteralBug;
  bool HasVertexCache;
  short TexVTXClauseSize;
  */
  bool ScalarizeGlobal;
/*
  bool HasVcmpxPermlaneHazard;
  bool HasVMEMtoScalarWriteHazard;
  bool HasSMEMtoVectorWriteHazard;
  bool HasInstFwdPrefetchBug;
  bool HasVcmpxExecWARHazard;
  bool HasLdsBranchVmemWARHazard;
  bool HasNSAtoVMEMBug;
  bool HasOffset3fBug;
  bool HasFlatSegmentOffsetBug;
*/
  // Dummy feature to use for assembler in tablegen.
  bool FeatureDisable;

private:

  // See COMPUTE_TMPRING_SIZE.WAVESIZE, 13-bit field in units of 256-dword.
  static const unsigned MaxWaveScratchSize = (256 * 4) * ((1 << 13) - 1);

public:
  ~PPUSubtarget() {};
/*
  const CallLowering *getCallLowering() const override {
    return CallLoweringInfo.get();
  }

  InstructionSelector *getInstructionSelector() const override {
    return InstSelector.get();
  }

  const LegalizerInfo *getLegalizerInfo() const override {
    return Legalizer.get();
  }

  const RegisterBankInfo *getRegBankInfo() const override {
    return RegBankInfo.get();
  }

  // Nothing implemented, just prevent crashes on use.
  const SelectionDAGTargetInfo *getSelectionDAGInfo() const override {
    return &TSInfo;
  }
  */
/*
  const InstrItineraryData *getInstrItineraryData() const override {
    return &InstrItins;
  }
*/

  // Generation getGeneration() const { return (Generation)Gen; }

  unsigned getWavefrontSizeLog2() const { return Log2_32(WavefrontSize); }

  /// Return the number of high bits known to be zero fror a frame index.
  unsigned getKnownHighZeroBitsForFrameIndex() const {
    return countLeadingZeros(MaxWaveScratchSize) + getWavefrontSizeLog2();
  }

  int getLDSBankCount() const { return LDSBankCount; }

  unsigned getMaxPrivateElementSize() const { return MaxPrivateElementSize; }
/*
  bool hasIntClamp() const { return HasIntClamp; }

  bool hasFP64() const { return FP64; }

  bool hasHWFP64() const { return FP64; }

  bool hasFastFMAF32() const { return FastFMAF32; }

  bool hasHalfRate64Ops() const { return HalfRate64Ops; }
  */

  bool hasAddr64() const { return false; }
/*
  bool hasBFE() const { return true; }

  bool hasBFI() const { return true; }

  bool hasBFM() const { return hasBFE(); }

  bool hasBCNT(unsigned Size) const { return true; }

  bool hasFFBL() const { return true; }

  bool hasFFBH() const { return true; }

  bool hasMed3_16() const { return true; }
  */

  bool hasFmaMixInsts() const { return HasFmaMixInsts; }

/*
  bool hasCARRY() const { return true; }

  bool hasFMA() const { return FMA; }
  TrapHandlerAbi getTrapHandlerAbi() const {
    return isAmdHsaOS() ? TrapHandlerAbiHsa : TrapHandlerAbiNone;
  }
  bool unsafeDSOffsetFoldingEnabled() const {
    return EnableUnsafeDSOffsetFolding;
  }
  */

  bool isPromoteAllocaEnabled() const { return EnablePromoteAlloca; }

  bool dumpCode() const { return DumpCode; }

  bool hasFP16Denormals() const { return FP64FP16Denormals; }

  /*
  bool hasFP64Denormals() const { return FP64FP16Denormals; }

  bool supportsMinMaxDenormModes() const { false; }
*/

  bool useFlatForGlobal() const { return FlatForGlobal; }

  /// \returns If MUBUF instructions always perform range checking, even for
  /// buffer resources used for private memory access.
  bool privateMemoryResourceIsRangeChecked() const { return true; }
/*
  bool hasAutoWaitcntBeforeBarrier() const { return AutoWaitcntBeforeBarrier; }
  */

  bool hasCodeObjectV3() const { return CodeObjectV3; }

  bool hasUnalignedBufferAccess() const { return UnalignedBufferAccess; }

  bool hasUnalignedScratchAccess() const { return UnalignedScratchAccess; }
  bool hasApertureRegs() const { return HasApertureRegs; }

/*
  bool isTrapHandlerEnabled() const { return TrapHandler; }
  */

  bool hasMAIInsts() const { return HasMAIInsts; }

  bool hasFlatAddressSpace() const { return FlatAddressSpace; }

  bool hasFlatScrRegister() const { return hasFlatAddressSpace(); }

  bool hasFlatInstOffsets() const { return FlatInstOffsets; }

  bool hasFlatGlobalInsts() const { return FlatGlobalInsts; }
/*

  bool hasFlatScratchInsts() const { return FlatScratchInsts; }

  bool hasScalarFlatScratchInsts() const { return ScalarFlatScratchInsts; }

  bool hasFlatSegmentOffsetBug() const { return HasFlatSegmentOffsetBug; }

  bool hasFlatLgkmVMemCountInOrder() const { return false; }

  bool hasAddNoCarry() const { return AddNoCarryInsts; }

  bool hasUnpackedD16VMem() const { return HasUnpackedD16VMem; }

  bool hasMad64_32() const { return false; }

  bool hasSDWAOutModsVOPC() const { return HasSDWAOutModsVOPC; }

  bool hasDLInsts() const { return HasDLInsts; }
*/

  bool hasRegisterBanking() const {
    return HasRegisterBanking;
  }
  // Scratch is allocated in 256 dword per wave blocks for the entire
  // wavefront. When viewed from the perspecive of an arbitrary workitem, this
  // is 4-byte aligned.
  //
  // Only 4-byte alignment is really needed to access anything. Transformations
  // on the pointer value itself may rely on the alignment / known low bits of
  // the pointer. Set this to something above the minimum to avoid needing
  // dynamic realignment in common cases.
  unsigned getStackAlignment() const { return 16; }

  bool enableMachineScheduler() const override
  {
      // FIXME set to true after Scheduler is ready
      return false;
  }

  bool enableSubRegLiveness() const override
  {
      return true;
  }

  void setScalarizeGlobalBehavior(bool b) { ScalarizeGlobal = b; }
  bool getScalarizeGlobalBehavior() const { return ScalarizeGlobal; }

  /// \returns Number of execution units per compute unit supported by the
  /// subtarget.
  unsigned getEUsPerCU() const {
    return PPU::IsaInfo::getEUsPerCU(this);
  }

  /// \returns Maximum number of waves per compute unit supported by the
  /// subtarget without any kind of limitation.
  unsigned getMaxWavesPerCU() const {
    return PPU::IsaInfo::getMaxWavesPerCU(this);
  }

  /// \returns Maximum number of waves per compute unit supported by the
  /// subtarget and limited by given \p FlatWorkGroupSize.
  unsigned getMaxWavesPerCU(unsigned FlatWorkGroupSize) const {
    return PPU::IsaInfo::getMaxWavesPerCU(this, FlatWorkGroupSize);
  }

  /// \returns Number of waves per work group supported by the subtarget and
  /// limited by given \p FlatWorkGroupSize.
  unsigned getWavesPerWorkGroup(unsigned FlatWorkGroupSize) const {
    return PPU::IsaInfo::getWavesPerWorkGroup(this, FlatWorkGroupSize);
  }

  // XXX - Why is this here if it isn't in the default pass set?
  bool enableEarlyIfConversion() const override {
    return true;
  }

  void overrideSchedPolicy(MachineSchedPolicy &Policy,
                           unsigned NumRegionInstrs) const override;

  unsigned getMaxNumUserSGPRs() const {
    return 16;
  }
/*
  bool hasSMemRealTime() const {
    return HasSMemRealTime;
  }
*/

  bool useVGPRIndexMode(bool UserEnable) const {
    return UserEnable;
    // return !hasMovrel() || (UserEnable && hasVGPRIndexMode());
  }

/*
  bool hasDPP() const {
    return HasDPP;
  }
*/
  bool hasMadF16() const;

  bool enablePPUScheduler() const {
    return EnablePPUScheduler;
  }
/*  // TODO

  bool loadStoreOptEnabled() const {
    return EnableLoadStoreOpt;
  }
*/
  bool has12DWordStoreHazard() const {
    return true;
  }

  /// Return the maximum number of waves per SIMD for kernels using \p SGPRs
  /// SGPRs
  unsigned getOccupancyWithNumSGPRs(unsigned SGPRs) const;

  /// Return the maximum number of waves per SIMD for kernels using \p VGPRs
  /// VGPRs
  unsigned getOccupancyWithNumVGPRs(unsigned VGPRs) const;

  /// Return occupancy for the given function. Used LDS and a number of
  /// registers if provided.
  /// Note, occupancy can be affected by the scratch allocation as well, but
  /// we do not have enough information to compute it.
  unsigned computeOccupancy(const MachineFunction &MF, unsigned LDSSize = 0,
                            unsigned NumSGPRs = 0, unsigned NumVGPRs = 0) const;

  /// \returns true if the flat_scratch register should be initialized with the
  /// pointer to the wave's scratch memory rather than a size and offset.
  bool flatScratchIsPointer() const {
      return true;
  }

  /// \returns SGPR allocation granularity supported by the subtarget.
  unsigned getSGPRAllocGranule() const {
    return PPU::IsaInfo::getSGPRAllocGranule(this);
  }

  /// \returns SGPR encoding granularity supported by the subtarget.
  unsigned getSGPREncodingGranule() const {
    return PPU::IsaInfo::getSGPREncodingGranule(this);
  }

  /// \returns Total number of SGPRs supported by the subtarget.
  unsigned getTotalNumSGPRs() const {
    return PPU::IsaInfo::getTotalNumSGPRs(this);
  }

  /// \returns Addressable number of SGPRs supported by the subtarget.
  unsigned getAddressableNumSGPRs() const {
    return PPU::IsaInfo::getAddressableNumSGPRs(this);
  }

  /// \returns Minimum number of SGPRs that meets the given number of waves per
  /// execution unit requirement supported by the subtarget.
  unsigned getMinNumSGPRs(unsigned WavesPerEU) const {
    return PPU::IsaInfo::getMinNumSGPRs(this, WavesPerEU);
  }

  /// \returns Maximum number of SGPRs that meets the given number of waves per
  /// execution unit requirement supported by the subtarget.
  unsigned getMaxNumSGPRs(unsigned WavesPerEU, bool Addressable) const {
    return PPU::IsaInfo::getMaxNumSGPRs(this, WavesPerEU, Addressable);
  }

  /// \returns Reserved number of SGPRs for given function \p MF.
  unsigned getReservedNumSGPRs(const MachineFunction &MF) const;

  /// \returns Maximum number of SGPRs that meets number of waves per execution
  /// unit requirement for function \p MF, or number of SGPRs explicitly
  /// requested using "amdgpu-num-sgpr" attribute attached to function \p MF.
  ///
  /// \returns Value that meets number of waves per execution unit requirement
  /// if explicitly requested value cannot be converted to integer, violates
  /// subtarget's specifications, or does not meet number of waves per execution
  /// unit requirement.
  unsigned getMaxNumSGPRs(const MachineFunction &MF) const;

  /// \returns VGPR allocation granularity supported by the subtarget.
  unsigned getVGPRAllocGranule() const {
    return PPU::IsaInfo::getVGPRAllocGranule(this);
  }
/*
  /// \returns VGPR encoding granularity supported by the subtarget.
  unsigned getVGPREncodingGranule() const {
    return PPU::IsaInfo::getVGPREncodingGranule(this);
  }
*/
  /// \returns Total number of VGPRs supported by the subtarget.
  unsigned getTotalNumVGPRs() const {
    return PPU::IsaInfo::getTotalNumVGPRs(this);
  }

  /// \returns Addressable number of VGPRs supported by the subtarget.
  unsigned getAddressableNumVGPRs() const {
    return PPU::IsaInfo::getAddressableNumVGPRs(this);
  }

  /// \returns Minimum number of VGPRs that meets given number of waves per
  /// execution unit requirement supported by the subtarget.
  unsigned getMinNumVGPRs(unsigned WavesPerEU) const {
    return PPU::IsaInfo::getMinNumVGPRs(this, WavesPerEU);
  }

  /// \returns Maximum number of VGPRs that meets given number of waves per
  /// execution unit requirement supported by the subtarget.
  unsigned getMaxNumVGPRs(unsigned WavesPerEU) const {
    return PPU::IsaInfo::getMaxNumVGPRs(this, WavesPerEU);
  }

  /// \returns Maximum number of VGPRs that meets number of waves per execution
  /// unit requirement for function \p MF, or number of VGPRs explicitly
  /// requested using "amdgpu-num-vgpr" attribute attached to function \p MF.
  ///
  /// \returns Value that meets number of waves per execution unit requirement
  /// if explicitly requested value cannot be converted to integer, violates
  /// subtarget's specifications, or does not meet number of waves per execution
  /// unit requirement.
  unsigned getMaxNumVGPRs(const MachineFunction &MF) const;

  void getPostRAMutations(
      std::vector<std::unique_ptr<ScheduleDAGMutation>> &Mutations)
      const override;

  bool isWave32() const {
    return WavefrontSize == 32;
  }
/*
  const TargetRegisterClass *getBoolRC() const {
    return getRegisterInfo()->getBoolRC();
  }
*/

};
} // End llvm namespace

#endif
