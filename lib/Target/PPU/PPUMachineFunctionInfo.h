//=- PPUMachineFunctionInfo.h - PPU machine function info -----*- C++ -*-=//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file declares PPU-specific per-machine-function information.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_PPU_PPUMACHINEFUNCTIONINFO_H
#define LLVM_LIB_TARGET_PPU_PPUMACHINEFUNCTIONINFO_H

#include "PPUArgumentUsageInfo.h"
#include "PPUMachineFunction.h"
#include "MCTargetDesc/PPUMCTargetDesc.h"
#include "PPUInstrInfo.h"
#include "PPURegisterInfo.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/ADT/Optional.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/SparseBitVector.h"
#include "llvm/CodeGen/MIRYamlMapping.h"
#include "llvm/CodeGen/PseudoSourceValue.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/Support/ErrorHandling.h"

// TODO copied from rvv
#include "llvm/CodeGen/TargetSubtargetInfo.h"
#include <memory>


namespace llvm {

/// PPUMachineFunctionInfo - This class is derived from MachineFunctionInfo
/// and contains private PPU-specific information for each MachineFunction.
class PPUBaseMachineFunctionInfo : public PPUMachineFunction {
private:
  MachineFunction &MF;
  /// FrameIndex for start of varargs area
  int VarArgsFrameIndex = 0;
  /// Size of the save area used for varargs
  int VarArgsSaveSize = 0;
  /// FrameIndex used for transferring values between 64-bit FPRs and a pair
  /// of 32-bit GPRs via the stack.
  int MoveF64FrameIndex = -1;

public:
  PPUBaseMachineFunctionInfo() = delete;

  PPUBaseMachineFunctionInfo(const MachineFunction &MF)
      : PPUMachineFunction(MF)
      , MF(*const_cast<MachineFunction*>(&MF))
  {
  }

  int getVarArgsFrameIndex() const { return VarArgsFrameIndex; }
  void setVarArgsFrameIndex(int Index) { VarArgsFrameIndex = Index; }

  unsigned getVarArgsSaveSize() const { return VarArgsSaveSize; }
  void setVarArgsSaveSize(int Size) { VarArgsSaveSize = Size; }

  int getMoveF64FrameIndex() {
    if (MoveF64FrameIndex == -1)
      MoveF64FrameIndex = MF.getFrameInfo().CreateStackObject(8, 8, false);
    return MoveF64FrameIndex;
  }
};



// below copied
class MachineFrameInfo;
class MachineFunction;
class TargetRegisterClass;

class PPUPseudoSourceValue : public PseudoSourceValue {
public:
  enum PPUPSVKind : unsigned {
    PSVBuffer = PseudoSourceValue::TargetCustom,
    PSVImage,
    GWSResource
  };

protected:
  PPUPseudoSourceValue(unsigned Kind, const TargetInstrInfo &TII)
      : PseudoSourceValue(Kind, TII) {}

public:
  bool isConstant(const MachineFrameInfo *) const override {
    // This should probably be true for most images, but we will start by being
    // conservative.
    return false;
  }

  bool isAliased(const MachineFrameInfo *) const override {
    return true;
  }

  bool mayAlias(const MachineFrameInfo *) const override {
    return true;
  }
};

class PPUBufferPseudoSourceValue final : public PPUPseudoSourceValue {
public:
  explicit PPUBufferPseudoSourceValue(const TargetInstrInfo &TII)
      : PPUPseudoSourceValue(PSVBuffer, TII) {}

  static bool classof(const PseudoSourceValue *V) {
    return V->kind() == PSVBuffer;
  }
};

class PPUImagePseudoSourceValue final : public PPUPseudoSourceValue {
public:
  // TODO: Is the img rsrc useful?
  explicit PPUImagePseudoSourceValue(const TargetInstrInfo &TII)
      : PPUPseudoSourceValue(PSVImage, TII) {}

  static bool classof(const PseudoSourceValue *V) {
    return V->kind() == PSVImage;
  }
};

class PPUGWSResourcePseudoSourceValue final : public PPUPseudoSourceValue {
public:
  explicit PPUGWSResourcePseudoSourceValue(const TargetInstrInfo &TII)
      : PPUPseudoSourceValue(GWSResource, TII) {}

  static bool classof(const PseudoSourceValue *V) {
    return V->kind() == GWSResource;
  }

  // These are inaccessible memory from IR.
  bool isAliased(const MachineFrameInfo *) const override {
    return false;
  }

  // These are inaccessible memory from IR.
  bool mayAlias(const MachineFrameInfo *) const override {
    return false;
  }

  void printCustom(raw_ostream &OS) const override {
    OS << "GWSResource";
  }
};

namespace yaml {

struct PPUArgument {
  bool IsRegister;
  union {
    StringValue RegisterName;
    unsigned StackOffset;
  };
  Optional<unsigned> Mask;

  // Default constructor, which creates a stack argument.
  PPUArgument() : IsRegister(false), StackOffset(0) {}
  PPUArgument(const PPUArgument &Other) {
    IsRegister = Other.IsRegister;
    if (IsRegister) {
      ::new ((void *)std::addressof(RegisterName))
          StringValue(Other.RegisterName);
    } else
      StackOffset = Other.StackOffset;
    Mask = Other.Mask;
  }
  PPUArgument &operator=(const PPUArgument &Other) {
    IsRegister = Other.IsRegister;
    if (IsRegister) {
      ::new ((void *)std::addressof(RegisterName))
          StringValue(Other.RegisterName);
    } else
      StackOffset = Other.StackOffset;
    Mask = Other.Mask;
    return *this;
  }
  ~PPUArgument() {
    if (IsRegister)
      RegisterName.~StringValue();
  }

  // Helper to create a register or stack argument.
  static inline PPUArgument createArgument(bool IsReg) {
    if (IsReg)
      return PPUArgument(IsReg);
    return PPUArgument();
  }

private:
  // Construct a register argument.
  PPUArgument(bool) : IsRegister(true), RegisterName() {}
};

template <> struct MappingTraits<PPUArgument> {
  static void mapping(IO &YamlIO, PPUArgument &A) {
    if (YamlIO.outputting()) {
      if (A.IsRegister)
        YamlIO.mapRequired("reg", A.RegisterName);
      else
        YamlIO.mapRequired("offset", A.StackOffset);
    } else {
      auto Keys = YamlIO.keys();
      if (is_contained(Keys, "reg")) {
        A = PPUArgument::createArgument(true);
        YamlIO.mapRequired("reg", A.RegisterName);
      } else if (is_contained(Keys, "offset"))
        YamlIO.mapRequired("offset", A.StackOffset);
      else
        YamlIO.setError("missing required key 'reg' or 'offset'");
    }
    YamlIO.mapOptional("mask", A.Mask);
  }
  static const bool flow = true;
};

struct PPUArgumentInfo {
  Optional<PPUArgument> PrivateSegmentBuffer;
  Optional<PPUArgument> DispatchPtr;
  Optional<PPUArgument> QueuePtr;
  Optional<PPUArgument> KernargSegmentPtr;
  Optional<PPUArgument> DispatchID;
  Optional<PPUArgument> FlatScratchInit;
  Optional<PPUArgument> PrivateSegmentSize;

  Optional<PPUArgument> WorkGroupIDX;
  Optional<PPUArgument> WorkGroupIDY;
  Optional<PPUArgument> WorkGroupIDZ;
  Optional<PPUArgument> WorkGroupInfo;
  Optional<PPUArgument> PrivateSegmentWaveByteOffset;

  Optional<PPUArgument> ImplicitArgPtr;
  Optional<PPUArgument> ImplicitBufferPtr;

  Optional<PPUArgument> WorkItemIDX;
  Optional<PPUArgument> WorkItemIDY;
  Optional<PPUArgument> WorkItemIDZ;
};

template <> struct MappingTraits<PPUArgumentInfo> {
  static void mapping(IO &YamlIO, PPUArgumentInfo &AI) {
    YamlIO.mapOptional("privateSegmentBuffer", AI.PrivateSegmentBuffer);
    YamlIO.mapOptional("dispatchPtr", AI.DispatchPtr);
    YamlIO.mapOptional("queuePtr", AI.QueuePtr);
    YamlIO.mapOptional("kernargSegmentPtr", AI.KernargSegmentPtr);
    YamlIO.mapOptional("dispatchID", AI.DispatchID);
    YamlIO.mapOptional("flatScratchInit", AI.FlatScratchInit);
    YamlIO.mapOptional("privateSegmentSize", AI.PrivateSegmentSize);

    YamlIO.mapOptional("workGroupIDX", AI.WorkGroupIDX);
    YamlIO.mapOptional("workGroupIDY", AI.WorkGroupIDY);
    YamlIO.mapOptional("workGroupIDZ", AI.WorkGroupIDZ);
    YamlIO.mapOptional("workGroupInfo", AI.WorkGroupInfo);
    YamlIO.mapOptional("privateSegmentWaveByteOffset",
                       AI.PrivateSegmentWaveByteOffset);

    YamlIO.mapOptional("implicitArgPtr", AI.ImplicitArgPtr);
    YamlIO.mapOptional("implicitBufferPtr", AI.ImplicitBufferPtr);

    YamlIO.mapOptional("workItemIDX", AI.WorkItemIDX);
    YamlIO.mapOptional("workItemIDY", AI.WorkItemIDY);
    YamlIO.mapOptional("workItemIDZ", AI.WorkItemIDZ);
  }
};

// Default to default mode for default calling convention.
struct PPUMode {
  bool IEEE = true;
  bool DX10Clamp = true;

  PPUMode() = default;


  PPUMode(const PPU::PPUModeRegisterDefaults &Mode) {
    IEEE = Mode.IEEE;
    DX10Clamp = Mode.DX10Clamp;
  }

  bool operator ==(const PPUMode Other) const {
    return IEEE == Other.IEEE && DX10Clamp == Other.DX10Clamp;
  }
};

template <> struct MappingTraits<PPUMode> {
  static void mapping(IO &YamlIO, PPUMode &Mode) {
    YamlIO.mapOptional("ieee", Mode.IEEE, true);
    YamlIO.mapOptional("dx10-clamp", Mode.DX10Clamp, true);
  }
};

struct PPUMachineFunctionInfo final : public yaml::MachineFunctionInfo {
  uint64_t ExplicitKernArgSize = 0;
  unsigned MaxKernArgAlign = 0;
  unsigned LDSSize = 0;
  bool IsEntryFunction = false;
  bool NoSignedZerosFPMath = false;
  bool MemoryBound = false;
  bool WaveLimiter = false;
  uint32_t HighBitsOf32BitAddress = 0;

  StringValue ScratchRSrcReg = "$private_rsrc_reg";
  StringValue ScratchWaveOffsetReg = "$scratch_wave_offset_reg";
  StringValue FrameOffsetReg = "$fp_reg";
  StringValue StackPtrOffsetReg = "$sp_reg";

  Optional<PPUArgumentInfo> ArgInfo;
  PPUMode Mode;

  PPUMachineFunctionInfo() = default;
  PPUMachineFunctionInfo(const llvm::PPUMachineFunctionInfo &,
                        const TargetRegisterInfo &TRI);

  void mappingImpl(yaml::IO &YamlIO) override;
  ~PPUMachineFunctionInfo() = default;
};

template <> struct MappingTraits<PPUMachineFunctionInfo> {
  static void mapping(IO &YamlIO, PPUMachineFunctionInfo &MFI) {
    YamlIO.mapOptional("explicitKernArgSize", MFI.ExplicitKernArgSize,
                       UINT64_C(0));
    YamlIO.mapOptional("maxKernArgAlign", MFI.MaxKernArgAlign, 0u);
    YamlIO.mapOptional("ldsSize", MFI.LDSSize, 0u);
    YamlIO.mapOptional("isEntryFunction", MFI.IsEntryFunction, false);
    YamlIO.mapOptional("noSignedZerosFPMath", MFI.NoSignedZerosFPMath, false);
    YamlIO.mapOptional("memoryBound", MFI.MemoryBound, false);
    YamlIO.mapOptional("waveLimiter", MFI.WaveLimiter, false);
    YamlIO.mapOptional("scratchRSrcReg", MFI.ScratchRSrcReg,
                       StringValue("$private_rsrc_reg"));
    YamlIO.mapOptional("scratchWaveOffsetReg", MFI.ScratchWaveOffsetReg,
                       StringValue("$scratch_wave_offset_reg"));
    YamlIO.mapOptional("frameOffsetReg", MFI.FrameOffsetReg,
                       StringValue("$fp_reg"));
    YamlIO.mapOptional("stackPtrOffsetReg", MFI.StackPtrOffsetReg,
                       StringValue("$sp_reg"));
    YamlIO.mapOptional("argumentInfo", MFI.ArgInfo);
    YamlIO.mapOptional("mode", MFI.Mode, PPUMode());
    YamlIO.mapOptional("highBitsOf32BitAddress",
                       MFI.HighBitsOf32BitAddress, 0u);
  }
};

} // end namespace yaml

/// This class keeps track of the SPI_SP_INPUT_ADDR config register, which
/// tells the hardware which interpolation parameters to load.
class PPUMachineFunctionInfo final : public PPUBaseMachineFunctionInfo {
  friend class PPUTargetMachine;

  unsigned TIDReg = PPU::NoRegister;

  // Registers that may be reserved for spilling purposes. These may be the same
  // as the input registers.
  unsigned ScratchRSrcReg = PPU::PRIVATE_RSRC_REG;
  unsigned ScratchWaveOffsetReg = PPU::SCRATCH_WAVE_OFFSET_REG;

  // This is the current function's incremented size from the kernel's scratch
  // wave offset register. For an entry function, this is exactly the same as
  // the ScratchWaveOffsetReg.
  unsigned FrameOffsetReg = PPU::FP_REG;  // FIXME PPU::X8

  // Top of the stack SGPR offset derived from the ScratchWaveOffsetReg.
  unsigned StackPtrOffsetReg = PPU::SP_REG;  // FIXME PPU::X2

  PPUFunctionArgInfo ArgInfo;

  // State of MODE register, assumed FP mode.
  // TODO FP mode field consider RISCV 
  PPU::PPUModeRegisterDefaults Mode;


  // Graphics info.
  // unsigned PSInputAddr = 0;
  // unsigned PSInputEnable = 0;

  /// Number of bytes of arguments this function has on the stack. If the callee
  /// is expected to restore the argument stack this should be a multiple of 16,
  /// all usable during a tail call.
  ///
  /// The alternative would forbid tail call optimisation in some cases: if we
  /// want to transfer control from a function with 8-bytes of stack-argument
  /// space to a function with 16-bytes then misalignment of this value would
  /// make a stack adjustment necessary, which could not be undone by the
  /// callee.
  unsigned BytesInStackArgArea = 0;

  bool ReturnsVoid = true;

  // A pair of default/requested minimum/maximum flat work group sizes.
  // Minimum - first, maximum - second.
  std::pair<unsigned, unsigned> FlatWorkGroupSizes = {0, 0};

  // A pair of default/requested minimum/maximum number of waves per execution
  // unit. Minimum - first, maximum - second.
  std::pair<unsigned, unsigned> WavesPerEU = {0, 0};

  DenseMap<const Value *,
           std::unique_ptr<const PPUBufferPseudoSourceValue>> BufferPSVs;
  DenseMap<const Value *,
           std::unique_ptr<const PPUImagePseudoSourceValue>> ImagePSVs;
  std::unique_ptr<const PPUGWSResourcePseudoSourceValue> GWSResourcePSV;

private:
  unsigned LDSWaveSpillSize = 0;
  unsigned NumUserSGPRs = 0;
  unsigned NumSystemSGPRs = 0;

  bool HasSpilledSGPRs = false;
  bool HasSpilledVGPRs = false;
  bool HasNonSpillStackObjects = false;
  bool IsStackRealigned = false;

  unsigned NumSpilledSGPRs = 0;
  unsigned NumSpilledVGPRs = 0;

  // Feature bits required for inputs passed in user SGPRs.
  bool PrivateSegmentBuffer : 1;
  bool DispatchPtr : 1;
  bool QueuePtr : 1;
  bool KernargSegmentPtr : 1;
  bool DispatchID : 1;
  bool FlatScratchInit : 1;

  // Feature bits required for inputs passed in system SGPRs.
  bool WorkGroupIDX : 1; // Always initialized.
  bool WorkGroupIDY : 1;
  bool WorkGroupIDZ : 1;
  bool WorkGroupInfo : 1;
  bool PrivateSegmentWaveByteOffset : 1;

  bool WorkItemIDX : 1; // Always initialized.
  bool WorkItemIDY : 1;
  bool WorkItemIDZ : 1;

  // Private memory buffer
  // Compute directly in sgpr[0:1]
  // Other shaders indirect 64-bits at sgpr[0:1]
  bool ImplicitBufferPtr : 1;

  // Pointer to where the ABI inserts special kernel arguments separate from the
  // user arguments. This is an offset from the KernargSegmentPtr.
  bool ImplicitArgPtr : 1;

  // The hard-wired high half of the address of the global information table
  // for AMDPAL OS type. 0xffffffff represents no hard-wired high half, since
  // current hardware only allows a 16 bit value.
  unsigned GITPtrHigh;

  unsigned HighBitsOf32BitAddress {0};
  unsigned GDSSize {0};

  // Current recorded maximum possible occupancy.
  unsigned Occupancy;

  MCPhysReg getNextUserSPR() const;

  MCPhysReg getNextSystemSGPR() const;

public:
  struct SpilledReg {
    unsigned VGPR = 0;
    int Lane = -1;

    SpilledReg() = default;
    SpilledReg(unsigned R, int L) : VGPR (R), Lane (L) {}

    bool hasLane() { return Lane != -1;}
    bool hasReg() { return VGPR != 0;}
  };

  struct SGPRSpillVGPRCSR {
    // VGPR used for SGPR spills
    unsigned VGPR;

    // If the VGPR is a CSR, the stack slot used to save/restore it in the
    // prolog/epilog.
    Optional<int> FI;

    SGPRSpillVGPRCSR(unsigned V, Optional<int> F) : VGPR(V), FI(F) {}
  };

  struct VGPRSpillToAGPR {
    SmallVector<MCPhysReg, 32> Lanes;
    bool FullyAllocated = false;
  };

  SparseBitVector<> WWMReservedRegs;

  void ReserveWWMRegister(unsigned reg) { WWMReservedRegs.set(reg); }

private:
  // SGPR->VGPR spilling support.
  using SpillRegMask = std::pair<unsigned, unsigned>;

  // Track VGPR + wave index for each subregister of the SGPR spilled to
  // frameindex key.
  DenseMap<int, std::vector<SpilledReg>> SGPRToVGPRSpills;
  unsigned NumVGPRSpillLanes = 0;
  SmallVector<SGPRSpillVGPRCSR, 2> SpillVGPRs;

  DenseMap<int, VGPRSpillToAGPR> VGPRToAGPRSpills;

  // AGPRs used for VGPR spills.
  SmallVector<MCPhysReg, 32> SpillAGPR;

  // VGPRs used for AGPR spills.
  SmallVector<MCPhysReg, 32> SpillVGPR;

public: // FIXME
  /// If this is set, an SGPR used for save/restore of the register used for the
  /// frame pointer.
  unsigned SGPRForFPSaveRestoreCopy = 0;
  Optional<int> FramePointerSaveIndex;

public:
  PPUMachineFunctionInfo(const MachineFunction &MF);

  bool initializeBaseYamlFields(const yaml::PPUMachineFunctionInfo &YamlMFI);

  ArrayRef<SpilledReg> getSGPRToVGPRSpills(int FrameIndex) const {
    auto I = SGPRToVGPRSpills.find(FrameIndex);
    return (I == SGPRToVGPRSpills.end()) ?
      ArrayRef<SpilledReg>() : makeArrayRef(I->second);
  }

  ArrayRef<SGPRSpillVGPRCSR> getSGPRSpillVGPRs() const {
    return SpillVGPRs;
  }

  ArrayRef<MCPhysReg> getAGPRSpillVGPRs() const {
    return SpillAGPR;
  }

  ArrayRef<MCPhysReg> getVGPRSpillAGPRs() const {
    return SpillVGPR;
  }

  MCPhysReg getVGPRToAGPRSpill(int FrameIndex, unsigned Lane) const {
    auto I = VGPRToAGPRSpills.find(FrameIndex);
    return (I == VGPRToAGPRSpills.end()) ? (MCPhysReg)PPU::NoRegister
                                         : I->second.Lanes[Lane];
  }

  PPU::PPUModeRegisterDefaults getMode() const {
    return Mode;
  }

  bool haveFreeLanesForSGPRSpill(const MachineFunction &MF, unsigned NumLane) const;
  bool allocateSGPRSpillToVGPR(MachineFunction &MF, int FI);
  bool allocateVGPRSpillToAGPR(MachineFunction &MF, int FI, bool isAGPRtoVGPR);
  void removeSGPRToVGPRFrameIndices(MachineFrameInfo &MFI);
  void removeDeadFrameIndices(MachineFrameInfo &MFI);

  bool hasCalculatedTID() const { return TIDReg != 0; };
  unsigned getTIDReg() const { return TIDReg; };
  void setTIDReg(unsigned Reg) { TIDReg = Reg; }

  unsigned getBytesInStackArgArea() const {
    return BytesInStackArgArea;
  }

  void setBytesInStackArgArea(unsigned Bytes) {
    BytesInStackArgArea = Bytes;
  }

  // Add user SGPRs.
  unsigned addPrivateSegmentBuffer(const PPURegisterInfo &TRI);
  unsigned addDispatchPtr(const PPURegisterInfo &TRI);
  unsigned addQueuePtr(const PPURegisterInfo &TRI);
  unsigned addKernargSegmentPtr(const PPURegisterInfo &TRI);
  unsigned addDispatchID(const PPURegisterInfo &TRI);
  unsigned addFlatScratchInit(const PPURegisterInfo &TRI);
  unsigned addImplicitBufferPtr(const PPURegisterInfo &TRI);

  // Add system SGPRs.
  unsigned addWorkGroupIDX() {
    ArgInfo.WorkGroupIDX = ArgDescriptor::createRegister(getNextSystemSGPR());
    NumSystemSGPRs += 1;
    return ArgInfo.WorkGroupIDX.getRegister();
  }

  unsigned addWorkGroupIDY() {
    ArgInfo.WorkGroupIDY = ArgDescriptor::createRegister(getNextSystemSGPR());
    NumSystemSGPRs += 1;
    return ArgInfo.WorkGroupIDY.getRegister();
  }

  unsigned addWorkGroupIDZ() {
    ArgInfo.WorkGroupIDZ = ArgDescriptor::createRegister(getNextSystemSGPR());
    NumSystemSGPRs += 1;
    return ArgInfo.WorkGroupIDZ.getRegister();
  }

  unsigned addWorkGroupInfo() {
    ArgInfo.WorkGroupInfo = ArgDescriptor::createRegister(getNextSystemSGPR());
    NumSystemSGPRs += 1;
    return ArgInfo.WorkGroupInfo.getRegister();
  }

  // Add special VGPR inputs
  void setWorkItemIDX(ArgDescriptor Arg) {
    ArgInfo.WorkItemIDX = Arg;
  }

  void setWorkItemIDY(ArgDescriptor Arg) {
    ArgInfo.WorkItemIDY = Arg;
  }

  void setWorkItemIDZ(ArgDescriptor Arg) {
    ArgInfo.WorkItemIDZ = Arg;
  }

  unsigned addPrivateSegmentWaveByteOffset() {
    ArgInfo.PrivateSegmentWaveByteOffset
      = ArgDescriptor::createRegister(getNextSystemSGPR());
    NumSystemSGPRs += 1;
    return ArgInfo.PrivateSegmentWaveByteOffset.getRegister();
  }

  void setPrivateSegmentWaveByteOffset(unsigned Reg) {
    ArgInfo.PrivateSegmentWaveByteOffset = ArgDescriptor::createRegister(Reg);
  }

  bool hasPrivateSegmentBuffer() const {
    return PrivateSegmentBuffer;
  }

  bool hasDispatchPtr() const {
    return DispatchPtr;
  }

  bool hasQueuePtr() const {
    return QueuePtr;
  }

  bool hasKernargSegmentPtr() const {
    return KernargSegmentPtr;
  }

  bool hasDispatchID() const {
    return DispatchID;
  }

  bool hasFlatScratchInit() const {
    return FlatScratchInit;
  }

  bool hasWorkGroupIDX() const {
    return WorkGroupIDX;
  }

  bool hasWorkGroupIDY() const {
    return WorkGroupIDY;
  }

  bool hasWorkGroupIDZ() const {
    return WorkGroupIDZ;
  }

  bool hasWorkGroupInfo() const {
    return WorkGroupInfo;
  }

  bool hasPrivateSegmentWaveByteOffset() const {
    return PrivateSegmentWaveByteOffset;
  }

  bool hasWorkItemIDX() const {
    return WorkItemIDX;
  }

  bool hasWorkItemIDY() const {
    return WorkItemIDY;
  }

  bool hasWorkItemIDZ() const {
    return WorkItemIDZ;
  }

  bool hasImplicitArgPtr() const {
    return ImplicitArgPtr;
  }

  bool hasImplicitBufferPtr() const {
    return ImplicitBufferPtr;
  }

  PPUFunctionArgInfo &getArgInfo() {
    return ArgInfo;
  }

  const PPUFunctionArgInfo &getArgInfo() const {
    return ArgInfo;
  }

  std::pair<const ArgDescriptor *, const TargetRegisterClass *>
  getPreloadedValue(PPUFunctionArgInfo::PreloadedValue Value) const {
    return ArgInfo.getPreloadedValue(Value);
  }

  Register getPreloadedReg(PPUFunctionArgInfo::PreloadedValue Value) const {
    auto Arg = ArgInfo.getPreloadedValue(Value).first;
    return Arg ? Arg->getRegister() : Register();
  }

  unsigned getGITPtrHigh() const {
    return GITPtrHigh;
  }

  uint32_t get32BitAddressHighBits() const {
    return HighBitsOf32BitAddress;
  }

  unsigned getGDSSize() const {
    return GDSSize;
  }

  unsigned getNumUserSGPRs() const {
    return NumUserSGPRs;
  }

  unsigned getNumPreloadedSGPRs() const {
    return NumUserSGPRs + NumSystemSGPRs;
  }

  unsigned getPrivateSegmentWaveByteOffsetSystemSGPR() const {
    return ArgInfo.PrivateSegmentWaveByteOffset.getRegister();
  }

  /// Returns the physical register reserved for use as the resource
  /// descriptor for scratch accesses.
  unsigned getScratchRSrcReg() const {
    return ScratchRSrcReg;
  }

  void setScratchRSrcReg(unsigned Reg) {
    assert(Reg != 0 && "Should never be unset");
    ScratchRSrcReg = Reg;
  }

  unsigned getScratchWaveOffsetReg() const {
    return ScratchWaveOffsetReg;
  }

  unsigned getFrameOffsetReg() const {
    return FrameOffsetReg;
  }

  void setFrameOffsetReg(unsigned Reg) {
    assert(Reg != 0 && "Should never be unset");
    FrameOffsetReg = Reg;
  }

  void setStackPtrOffsetReg(unsigned Reg) {
    assert(Reg != 0 && "Should never be unset");
    StackPtrOffsetReg = Reg;
  }

  // Note the unset value for this is PPU::SP_REG rather than
  // NoRegister. This is mostly a workaround for MIR tests where state that
  // can't be directly computed from the function is not preserved in serialized
  // MIR.
  unsigned getStackPtrOffsetReg() const {
    return StackPtrOffsetReg;
  }

  void setScratchWaveOffsetReg(unsigned Reg) {
    assert(Reg != 0 && "Should never be unset");
    ScratchWaveOffsetReg = Reg;
  }

  unsigned getQueuePtrUserSGPR() const {
    return ArgInfo.QueuePtr.getRegister();
  }

  unsigned getImplicitBufferPtrUserSGPR() const {
    return ArgInfo.ImplicitBufferPtr.getRegister();
  }

  bool hasSpilledSGPRs() const {
    return HasSpilledSGPRs;
  }

  void setHasSpilledSGPRs(bool Spill = true) {
    HasSpilledSGPRs = Spill;
  }

  bool hasSpilledVGPRs() const {
    return HasSpilledVGPRs;
  }

  void setHasSpilledVGPRs(bool Spill = true) {
    HasSpilledVGPRs = Spill;
  }

  bool hasNonSpillStackObjects() const {
    return HasNonSpillStackObjects;
  }

  void setHasNonSpillStackObjects(bool StackObject = true) {
    HasNonSpillStackObjects = StackObject;
  }

  bool isStackRealigned() const {
    return IsStackRealigned;
  }

  void setIsStackRealigned(bool Realigned = true) {
    IsStackRealigned = Realigned;
  }

  unsigned getNumSpilledSGPRs() const {
    return NumSpilledSGPRs;
  }

  unsigned getNumSpilledVGPRs() const {
    return NumSpilledVGPRs;
  }

  void addToSpilledSGPRs(unsigned num) {
    NumSpilledSGPRs += num;
  }

  void addToSpilledVGPRs(unsigned num) {
    NumSpilledVGPRs += num;
  }
/*
  unsigned getPSInputAddr() const {
    return PSInputAddr;
  }

  unsigned getPSInputEnable() const {
    return PSInputEnable;
  }

  bool isPSInputAllocated(unsigned Index) const {
    return PSInputAddr & (1 << Index);
  }

  void markPSInputAllocated(unsigned Index) {
    PSInputAddr |= 1 << Index;
  }

  void markPSInputEnabled(unsigned Index) {
    PSInputEnable |= 1 << Index;
  }
*/
  bool returnsVoid() const {
    return ReturnsVoid;
  }

  void setIfReturnsVoid(bool Value) {
    ReturnsVoid = Value;
  }

  /// \returns A pair of default/requested minimum/maximum flat work group sizes
  /// for this function.
  std::pair<unsigned, unsigned> getFlatWorkGroupSizes() const {
    return FlatWorkGroupSizes;
  }

  /// \returns Default/requested minimum flat work group size for this function.
  unsigned getMinFlatWorkGroupSize() const {
    return FlatWorkGroupSizes.first;
  }

  /// \returns Default/requested maximum flat work group size for this function.
  unsigned getMaxFlatWorkGroupSize() const {
    return FlatWorkGroupSizes.second;
  }

  /// \returns A pair of default/requested minimum/maximum number of waves per
  /// execution unit.
  std::pair<unsigned, unsigned> getWavesPerEU() const {
    return WavesPerEU;
  }

  /// \returns Default/requested minimum number of waves per execution unit.
  unsigned getMinWavesPerEU() const {
    return WavesPerEU.first;
  }

  /// \returns Default/requested maximum number of waves per execution unit.
  unsigned getMaxWavesPerEU() const {
    return WavesPerEU.second;
  }

  /// \returns SGPR used for \p Dim's work group ID.
  unsigned getWorkGroupIDSGPR(unsigned Dim) const {
    switch (Dim) {
    case 0:
      assert(hasWorkGroupIDX());
      return ArgInfo.WorkGroupIDX.getRegister();
    case 1:
      assert(hasWorkGroupIDY());
      return ArgInfo.WorkGroupIDY.getRegister();
    case 2:
      assert(hasWorkGroupIDZ());
      return ArgInfo.WorkGroupIDZ.getRegister();
    }
    llvm_unreachable("unexpected dimension");
  }

  unsigned getLDSWaveSpillSize() const {
    return LDSWaveSpillSize;
  }

  const PPUBufferPseudoSourceValue *getBufferPSV(const PPUInstrInfo &TII,
                                                    const Value *BufferRsrc) {
    assert(BufferRsrc);
    auto PSV = BufferPSVs.try_emplace(
      BufferRsrc,
      std::make_unique<PPUBufferPseudoSourceValue>(TII));
    return PSV.first->second.get();
  }

  const PPUImagePseudoSourceValue *getImagePSV(const PPUInstrInfo &TII,
                                                  const Value *ImgRsrc) {
    assert(ImgRsrc);
    auto PSV = ImagePSVs.try_emplace(
      ImgRsrc,
      std::make_unique<PPUImagePseudoSourceValue>(TII));
    return PSV.first->second.get();
  }

  const PPUGWSResourcePseudoSourceValue *getGWSPSV(const PPUInstrInfo &TII) {
    if (!GWSResourcePSV) {
      GWSResourcePSV =
          std::make_unique<PPUGWSResourcePseudoSourceValue>(TII);
    }

    return GWSResourcePSV.get();
  }

  unsigned getOccupancy() const {
    return Occupancy;
  }

  unsigned getMinAllowedOccupancy() const {
    if (!isMemoryBound() && !needsWaveLimiter())
      return Occupancy;
    return (Occupancy < 4) ? Occupancy : 4;
  }

  void limitOccupancy(const MachineFunction &MF);

  void limitOccupancy(unsigned Limit) {
    if (Occupancy > Limit)
      Occupancy = Limit;
  }

  void increaseOccupancy(const MachineFunction &MF, unsigned Limit) {
    if (Occupancy < Limit)
      Occupancy = Limit;
    limitOccupancy(MF);
  }
};


} // end namespace llvm

#endif // LLVM_LIB_TARGET_PPU_PPUMACHINEFUNCTIONINFO_H
