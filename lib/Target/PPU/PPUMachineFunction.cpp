//===-- PPUMachineFunctionInfo.cpp ---------------------------------------=//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "PPUMachineFunction.h"
#include "PPUMachineFunctionInfo.h"
#include "PPUSubtarget.h"
#include "PPUPerfHintAnalysis.h"
#include "llvm/CodeGen/MachineModuleInfo.h"

using namespace llvm;

#define MAX_LANES 32

PPUMachineFunction::PPUMachineFunction(const MachineFunction &MF) :
  MachineFunctionInfo(),
  LocalMemoryObjects(),
  ExplicitKernArgSize(0),
  MaxKernArgAlign(0),
  LDSSize(0),
  IsEntryFunction(PPU::isEntryFunctionCC(MF.getFunction().getCallingConv())),
  NoSignedZerosFPMath(MF.getTarget().Options.NoSignedZerosFPMath),
  MemoryBound(false),
  WaveLimiter(false) {
  // const PPUSubtarget &ST = PPUSubtarget::get(MF);
  const PPUBaseSubtarget *ST =  dynamic_cast<const PPUBaseSubtarget*>(&MF.getSubtarget<PPUSubtarget>());

  // FIXME: Should initialize KernArgSize based on ExplicitKernelArgOffset,
  // except reserved size is not correctly aligned.
  const Function &F = MF.getFunction();

  Attribute MemBoundAttr = F.getFnAttribute("ppu-memory-bound");
  MemoryBound = MemBoundAttr.isStringAttribute() &&
                MemBoundAttr.getValueAsString() == "true";

  Attribute WaveLimitAttr = F.getFnAttribute("ppu-wave-limiter");
  WaveLimiter = WaveLimitAttr.isStringAttribute() &&
                WaveLimitAttr.getValueAsString() == "true";

  CallingConv::ID CC = F.getCallingConv();
  if (CC == CallingConv::AMDGPU_KERNEL || CC == CallingConv::SPIR_KERNEL)
    ExplicitKernArgSize = ST->getExplicitKernArgSize(F, MaxKernArgAlign);
}

unsigned PPUMachineFunction::allocateLDSGlobal(const DataLayout &DL,
                                                  const GlobalValue &GV) {
  auto Entry = LocalMemoryObjects.insert(std::make_pair(&GV, 0));
  if (!Entry.second)
    return Entry.first->second;

  unsigned Align = GV.getAlignment();
  if (Align == 0)
    Align = DL.getABITypeAlignment(GV.getValueType());

  /// TODO: We should sort these to minimize wasted space due to alignment
  /// padding. Currently the padding is decided by the first encountered use
  /// during lowering.
  unsigned Offset = LDSSize = alignTo(LDSSize, Align);

  Entry.first->second = Offset;
  LDSSize += DL.getTypeAllocSize(GV.getValueType());

  return Offset;
}

    // Mode(MF.getFunction()),
PPUMachineFunctionInfo::PPUMachineFunctionInfo(const MachineFunction &MF)
  : PPUBaseMachineFunctionInfo(MF),
    PrivateSegmentBuffer(false),
    DispatchPtr(false),
    QueuePtr(false),
    KernargSegmentPtr(false),
    DispatchID(false),
    FlatScratchInit(false),
    WorkGroupIDX(false),
    WorkGroupIDY(false),
    WorkGroupIDZ(false),
    WorkGroupInfo(false),
    PrivateSegmentWaveByteOffset(false),
    WorkItemIDX(false),
    WorkItemIDY(false),
    WorkItemIDZ(false),
    ImplicitBufferPtr(false),
    ImplicitArgPtr(false),
    GITPtrHigh(0xffffffff)
{
  const PPUSubtarget &ST = MF.getSubtarget<PPUSubtarget>();
  const Function &F = MF.getFunction();
  FlatWorkGroupSizes = ST.getFlatWorkGroupSizes(F);
  WavesPerEU = ST.getWavesPerEU(F);

  Occupancy = ST.computeOccupancy(MF, getLDSSize());
  CallingConv::ID CC = F.getCallingConv();

  if (CC == CallingConv::AMDGPU_KERNEL || CC == CallingConv::SPIR_KERNEL) {
    if (!F.arg_empty())
      KernargSegmentPtr = true;
    WorkGroupIDX = true;
    WorkItemIDX = true;
  }

  if (!isEntryFunction()) {
    // Non-entry functions have no special inputs for now, other registers
    // required for scratch access.
    /* FIXME
    ScratchRSrcReg = PPU::SGPR0_SGPR1_SGPR2_SGPR3;
    ScratchWaveOffsetReg = PPU::SGPR33;
    */

    // TODO: Pick a high register, and shift down, similar to a kernel.
    FrameOffsetReg = PPU::FP_REG;
    StackPtrOffsetReg = PPU::SP_REG;

    /* FIXME
    ArgInfo.PrivateSegmentBuffer =
      ArgDescriptor::createRegister(ScratchRSrcReg);
    ArgInfo.PrivateSegmentWaveByteOffset =
      ArgDescriptor::createRegister(ScratchWaveOffsetReg);
      */

    if (F.hasFnAttribute("ppu-implicitarg-ptr"))
      ImplicitArgPtr = true;
  } else {
    if (F.hasFnAttribute("ppu-implicitarg-ptr")) {
      KernargSegmentPtr = true;
      MaxKernArgAlign = std::max(ST.getAlignmentForImplicitArgPtr(),
                                 MaxKernArgAlign);
    }
  }

  if (F.hasFnAttribute("ppu-work-group-id-x"))
    WorkGroupIDX = true;

  if (F.hasFnAttribute("ppu-work-group-id-y"))
    WorkGroupIDY = true;

  if (F.hasFnAttribute("ppu-work-group-id-z"))
    WorkGroupIDZ = true;

  if (F.hasFnAttribute("ppu-work-item-id-x"))
    WorkItemIDX = true;

  if (F.hasFnAttribute("ppu-work-item-id-y"))
    WorkItemIDY = true;

  if (F.hasFnAttribute("ppu-work-item-id-z"))
    WorkItemIDZ = true;

  const MachineFrameInfo &FrameInfo = MF.getFrameInfo();
  bool HasStackObjects = FrameInfo.hasStackObjects();

  if (isEntryFunction()) {
    // X, XY, and XYZ are the only supported combinations, so make sure Y is
    // enabled if Z is.
    if (WorkItemIDZ)
      WorkItemIDY = true;

    PrivateSegmentWaveByteOffset = true;
  }

  // bool isAmdHsaOrMesa = ST.isAmdHsaOrMesa(F);
  // if (isAmdHsaOrMesa) {
    PrivateSegmentBuffer = true;

    if (F.hasFnAttribute("ppu-dispatch-ptr"))
      DispatchPtr = true;

    if (F.hasFnAttribute("ppu-queue-ptr"))
      QueuePtr = true;

    if (F.hasFnAttribute("ppu-dispatch-id"))
      DispatchID = true;
  // } else if (ST.isMesaGfxShader(F)) {
  //  ImplicitBufferPtr = true;
  // }

  if (F.hasFnAttribute("ppu-kernarg-segment-ptr"))
    KernargSegmentPtr = true;

  /* FIXME
  if (ST.hasFlatAddressSpace() && isEntryFunction() && isAmdHsaOrMesa) {
    auto hasNonSpillStackObjects = [&]() {
      // Avoid expensive checking if there's no stack objects.
      if (!HasStackObjects)
        return false;
      for (auto OI = FrameInfo.getObjectIndexBegin(),
                OE = FrameInfo.getObjectIndexEnd(); OI != OE; ++OI)
        if (!FrameInfo.isSpillSlotObjectIndex(OI))
          return true;
      // All stack objects are spill slots.
      return false;
    };
    // TODO: This could be refined a lot. The attribute is a poor way of
    // detecting calls that may require it before argument lowering.
    if (hasNonSpillStackObjects() || F.hasFnAttribute("amdgpu-flat-scratch"))
      FlatScratchInit = true;
  }
  */

  Attribute A = F.getFnAttribute("ppu-git-ptr-high");
  StringRef S = A.getValueAsString();
  if (!S.empty())
    S.consumeInteger(0, GITPtrHigh);

  A = F.getFnAttribute("ppu-32bit-address-high-bits");
  S = A.getValueAsString();
  if (!S.empty())
    S.consumeInteger(0, HighBitsOf32BitAddress);

  S = F.getFnAttribute("ppu-gds-size").getValueAsString();
  if (!S.empty())
    S.consumeInteger(0, GDSSize);
}

void PPUMachineFunctionInfo::limitOccupancy(const MachineFunction &MF) {
  limitOccupancy(getMaxWavesPerEU());
  const PPUSubtarget& ST = MF.getSubtarget<PPUSubtarget>();
  limitOccupancy(ST.getOccupancyWithLocalMemSize(getLDSSize(),
                 MF.getFunction()));
}

unsigned PPUMachineFunctionInfo::addPrivateSegmentBuffer(
  const PPURegisterInfo &TRI) {
  // FIXME
  ArgInfo.PrivateSegmentBuffer =
    ArgDescriptor::createRegister(TRI.getMatchingSuperReg(
    getNextUserSGPR(), PPU::X0, &PPU::GPRRegClass));
  NumUserSGPRs += 4;
  return ArgInfo.PrivateSegmentBuffer.getRegister();
  return 1;
}

unsigned PPUMachineFunctionInfo::addDispatchPtr(const PPURegisterInfo &TRI) {
  // FIXME
  ArgInfo.DispatchPtr = ArgDescriptor::createRegister(TRI.getMatchingSuperReg(
    getNextUserSGPR(), PPU::X0, &PPU::GPRRegClass));
  NumUserSGPRs += 2;
  return ArgInfo.DispatchPtr.getRegister();
}

unsigned PPUMachineFunctionInfo::addQueuePtr(const PPURegisterInfo &TRI) {
  // FIXME
  ArgInfo.QueuePtr = ArgDescriptor::createRegister(TRI.getMatchingSuperReg(
    getNextUserSGPR(), PPU::X0, &PPU::GPRRegClass));
  NumUserSGPRs += 2;
  return ArgInfo.QueuePtr.getRegister();
}

unsigned PPUMachineFunctionInfo::addKernargSegmentPtr(const PPURegisterInfo &TRI) {
  ArgInfo.KernargSegmentPtr
    = ArgDescriptor::createRegister(TRI.getMatchingSuperReg(
    getNextUserSGPR(), PPU::X0, &PPU::GPRRegClass));
  NumUserSGPRs += 2;
  return ArgInfo.KernargSegmentPtr.getRegister();
}

unsigned PPUMachineFunctionInfo::addDispatchID(const PPURegisterInfo &TRI) {
  ArgInfo.DispatchID = ArgDescriptor::createRegister(TRI.getMatchingSuperReg(
    getNextUserSGPR(), PPU::X0, &PPU::GPRRegClass));
  NumUserSGPRs += 2;
  return ArgInfo.DispatchID.getRegister();
}

unsigned PPUMachineFunctionInfo::addFlatScratchInit(const PPURegisterInfo &TRI) {
  ArgInfo.FlatScratchInit = ArgDescriptor::createRegister(TRI.getMatchingSuperReg(
    getNextUserSGPR(), PPU::X0, &PPU::GPRRegClass));
  NumUserSGPRs += 2;
  return ArgInfo.FlatScratchInit.getRegister();
}

unsigned PPUMachineFunctionInfo::addImplicitBufferPtr(const PPURegisterInfo &TRI) {
  ArgInfo.ImplicitBufferPtr = ArgDescriptor::createRegister(TRI.getMatchingSuperReg(
    getNextUserSGPR(), PPU::X0, &PPU::GPRRegClass));
  NumUserSGPRs += 2;
  return ArgInfo.ImplicitBufferPtr.getRegister();
}

static bool isCalleeSavedReg(const MCPhysReg *CSRegs, MCPhysReg Reg) {
  for (unsigned I = 0; CSRegs[I]; ++I) {
    if (CSRegs[I] == Reg)
      return true;
  }

  return false;
}

/// \p returns true if \p NumLanes slots are available in VGPRs already used for
/// SGPR spilling.
//
// FIXME: This only works after processFunctionBeforeFrameFinalized
bool PPUMachineFunctionInfo::haveFreeLanesForSGPRSpill(const MachineFunction &MF,
                                                      unsigned NumNeed) const {
  const PPUSubtarget &ST = MF.getSubtarget<PPUSubtarget>();
  unsigned WaveSize = ST.getWavefrontSize();
  return NumVGPRSpillLanes + NumNeed <= WaveSize * SpillVGPRs.size();
}

/// Reserve a slice of a VGPR to support spilling for FrameIndex \p FI.
bool PPUMachineFunctionInfo::allocateSGPRSpillToVGPR(MachineFunction &MF,
                                                    int FI) {
  std::vector<SpilledReg> &SpillLanes = SGPRToVGPRSpills[FI];

  // This has already been allocated.
  if (!SpillLanes.empty())
    return true;

  const PPUSubtarget &ST = MF.getSubtarget<PPUSubtarget>();
  const PPURegisterInfo *TRI = ST.getRegisterInfo();
  MachineFrameInfo &FrameInfo = MF.getFrameInfo();
  MachineRegisterInfo &MRI = MF.getRegInfo();
  unsigned WaveSize = ST.getWavefrontSize();

  unsigned Size = FrameInfo.getObjectSize(FI);
  assert(Size >= 4 && Size <= 64 && "invalid sgpr spill size");
  assert(TRI->spillSGPRToVGPR() && "not spilling SGPRs to VGPRs");

  int NumLanes = Size / 4;

  const MCPhysReg *CSRegs = MRI.getCalleeSavedRegs();

  // Make sure to handle the case where a wide SGPR spill may span between two
  // VGPRs.
  for (int I = 0; I < NumLanes; ++I, ++NumVGPRSpillLanes) {
    unsigned LaneVGPR;
    unsigned VGPRIndex = (NumVGPRSpillLanes % WaveSize);

    if (VGPRIndex == 0) {
      LaneVGPR = TRI->findUnusedRegister(MRI, &PPU::VRRegClass, MF);
      if (LaneVGPR == PPU::NoRegister) {
        // We have no VGPRs left for spilling SGPRs. Reset because we will not
        // partially spill the SGPR to VGPRs.
        SGPRToVGPRSpills.erase(FI);
        NumVGPRSpillLanes -= I;
        return false;
      }

      Optional<int> CSRSpillFI;
      if ((FrameInfo.hasCalls() || !isEntryFunction()) && CSRegs &&
          isCalleeSavedReg(CSRegs, LaneVGPR)) {
        CSRSpillFI = FrameInfo.CreateSpillStackObject(4, 4);
      }

      SpillVGPRs.push_back(SGPRSpillVGPRCSR(LaneVGPR, CSRSpillFI));

      // Add this register as live-in to all blocks to avoid machine verifer
      // complaining about use of an undefined physical register.
      for (MachineBasicBlock &BB : MF)
        BB.addLiveIn(LaneVGPR);
    } else {
      LaneVGPR = SpillVGPRs.back().VGPR;
    }

    SpillLanes.push_back(SpilledReg(LaneVGPR, VGPRIndex));
  }

  return true;
}

/// Reserve AGPRs or VGPRs to support spilling for FrameIndex \p FI.
/// Either AGPR is spilled to VGPR to vice versa.
/// Returns true if a \p FI can be eliminated completely.
bool PPUMachineFunctionInfo::allocateVGPRSpillToAGPR(MachineFunction &MF,
                                                    int FI,
                                                    bool isAGPRtoVGPR) {
  MachineRegisterInfo &MRI = MF.getRegInfo();
  MachineFrameInfo &FrameInfo = MF.getFrameInfo();
  const PPUSubtarget &ST =  MF.getSubtarget<PPUSubtarget>();

  // FIXME assert(ST.hasMAIInsts() && FrameInfo.isSpillSlotObjectIndex(FI));

  auto &Spill = VGPRToAGPRSpills[FI];

  // This has already been allocated.
  if (!Spill.Lanes.empty())
    return Spill.FullyAllocated;

  unsigned Size = FrameInfo.getObjectSize(FI);
  unsigned NumLanes = Size / 4;
  Spill.Lanes.resize(NumLanes, PPU::NoRegister);

/* FIXME
  const TargetRegisterClass &RC =
      isAGPRtoVGPR ? PPU::VGPR_32RegClass : PPU::AGPR_32RegClass;
      */
  const TargetRegisterClass &RC = PPU::VRRegClass;

  auto Regs = RC.getRegisters();

  // FIXME auto &SpillRegs = isAGPRtoVGPR ? SpillAGPR : SpillVGPR;
  auto &SpillRegs = SpillVGPR;
  const PPURegisterInfo *TRI = ST.getRegisterInfo();
  Spill.FullyAllocated = true;

  // FIXME: Move allocation logic out of MachineFunctionInfo and initialize
  // once.
  BitVector OtherUsedRegs;
  OtherUsedRegs.resize(TRI->getNumRegs());

  const uint32_t *CSRMask =
      TRI->getCallPreservedMask(MF, MF.getFunction().getCallingConv());
  if (CSRMask)
    OtherUsedRegs.setBitsInMask(CSRMask);

  // TODO: Should include register tuples, but doesn't matter with current
  // usage.
  /* FIXME
  for (MCPhysReg Reg : SpillAGPR)
    OtherUsedRegs.set(Reg);
    */

  for (MCPhysReg Reg : SpillVGPR)
    OtherUsedRegs.set(Reg);

  SmallVectorImpl<MCPhysReg>::const_iterator NextSpillReg = Regs.begin();
  for (unsigned I = 0; I < NumLanes; ++I) {
    NextSpillReg = std::find_if(
        NextSpillReg, Regs.end(), [&MRI, &OtherUsedRegs](MCPhysReg Reg) {
          return MRI.isAllocatable(Reg) && !MRI.isPhysRegUsed(Reg) &&
                 !OtherUsedRegs[Reg];
        });

    if (NextSpillReg == Regs.end()) { // Registers exhausted
      Spill.FullyAllocated = false;
      break;
    }

    OtherUsedRegs.set(*NextSpillReg);
    SpillRegs.push_back(*NextSpillReg);
    Spill.Lanes[I] = *NextSpillReg++;
  }

  return Spill.FullyAllocated;
}

void PPUMachineFunctionInfo::removeDeadFrameIndices(MachineFrameInfo &MFI) {
  // The FP spill hasn't been inserted yet, so keep it around.
  for (auto &R : SGPRToVGPRSpills) {
    if (R.first != FramePointerSaveIndex)
      MFI.RemoveStackObject(R.first);
  }

  // All other SPGRs must be allocated on the default stack, so reset the stack
  // ID.
  for (int i = MFI.getObjectIndexBegin(), e = MFI.getObjectIndexEnd(); i != e;
       ++i)
    if (i != FramePointerSaveIndex)
      MFI.setStackID(i, TargetStackID::Default);

  for (auto &R : VGPRToAGPRSpills) {
    if (R.second.FullyAllocated)
      MFI.RemoveStackObject(R.first);
  }
}

MCPhysReg PPUMachineFunctionInfo::getNextUserSGPR() const {
  assert(NumSystemSGPRs == 0 && "System SGPRs must be added after user SGPRs");
  // FIXME
  return PPU::X0 + NumUserSGPRs;
}

MCPhysReg PPUMachineFunctionInfo::getNextSystemSGPR() const {
  // FIXME
  return PPU::X0 + NumUserSGPRs + NumSystemSGPRs;
}
/*
static yaml::StringValue regToString(unsigned Reg,
                                     const TargetRegisterInfo &TRI) {
  yaml::StringValue Dest;
  {
    raw_string_ostream OS(Dest.Value);
    OS << printReg(Reg, &TRI);
  }
  return Dest;
}

static Optional<yaml::SIArgumentInfo>
convertArgumentInfo(const PPUFunctionArgInfo &ArgInfo,
                    const TargetRegisterInfo &TRI) {
  yaml::SIArgumentInfo AI;

  auto convertArg = [&](Optional<yaml::SIArgument> &A,
                        const ArgDescriptor &Arg) {
    if (!Arg)
      return false;

    // Create a register or stack argument.
    yaml::SIArgument SA = yaml::SIArgument::createArgument(Arg.isRegister());
    if (Arg.isRegister()) {
      raw_string_ostream OS(SA.RegisterName.Value);
      OS << printReg(Arg.getRegister(), &TRI);
    } else
      SA.StackOffset = Arg.getStackOffset();
    // Check and update the optional mask.
    if (Arg.isMasked())
      SA.Mask = Arg.getMask();

    A = SA;
    return true;
  };

  bool Any = false;
  Any |= convertArg(AI.PrivateSegmentBuffer, ArgInfo.PrivateSegmentBuffer);
  Any |= convertArg(AI.DispatchPtr, ArgInfo.DispatchPtr);
  Any |= convertArg(AI.QueuePtr, ArgInfo.QueuePtr);
  Any |= convertArg(AI.KernargSegmentPtr, ArgInfo.KernargSegmentPtr);
  Any |= convertArg(AI.DispatchID, ArgInfo.DispatchID);
  Any |= convertArg(AI.FlatScratchInit, ArgInfo.FlatScratchInit);
  Any |= convertArg(AI.PrivateSegmentSize, ArgInfo.PrivateSegmentSize);
  Any |= convertArg(AI.WorkGroupIDX, ArgInfo.WorkGroupIDX);
  Any |= convertArg(AI.WorkGroupIDY, ArgInfo.WorkGroupIDY);
  Any |= convertArg(AI.WorkGroupIDZ, ArgInfo.WorkGroupIDZ);
  Any |= convertArg(AI.WorkGroupInfo, ArgInfo.WorkGroupInfo);
  Any |= convertArg(AI.PrivateSegmentWaveByteOffset,
                    ArgInfo.PrivateSegmentWaveByteOffset);
  Any |= convertArg(AI.ImplicitArgPtr, ArgInfo.ImplicitArgPtr);
  Any |= convertArg(AI.ImplicitBufferPtr, ArgInfo.ImplicitBufferPtr);
  Any |= convertArg(AI.WorkItemIDX, ArgInfo.WorkItemIDX);
  Any |= convertArg(AI.WorkItemIDY, ArgInfo.WorkItemIDY);
  Any |= convertArg(AI.WorkItemIDZ, ArgInfo.WorkItemIDZ);

  if (Any)
    return AI;

  return None;
}

yaml::PPUMachineFunctionInfo::PPUMachineFunctionInfo(
  const llvm::PPUMachineFunctionInfo& MFI,
  const TargetRegisterInfo &TRI)
  : ExplicitKernArgSize(MFI.getExplicitKernArgSize()),
    MaxKernArgAlign(MFI.getMaxKernArgAlign()),
    LDSSize(MFI.getLDSSize()),
    IsEntryFunction(MFI.isEntryFunction()),
    NoSignedZerosFPMath(MFI.hasNoSignedZerosFPMath()),
    MemoryBound(MFI.isMemoryBound()),
    WaveLimiter(MFI.needsWaveLimiter()),
    HighBitsOf32BitAddress(MFI.get32BitAddressHighBits()),
    ScratchRSrcReg(regToString(MFI.getScratchRSrcReg(), TRI)),
    ScratchWaveOffsetReg(regToString(MFI.getScratchWaveOffsetReg(), TRI)),
    FrameOffsetReg(regToString(MFI.getFrameOffsetReg(), TRI)),
    StackPtrOffsetReg(regToString(MFI.getStackPtrOffsetReg(), TRI)),
    ArgInfo(convertArgumentInfo(MFI.getArgInfo(), TRI)),
    Mode(MFI.getMode()) {}

void yaml::PPUMachineFunctionInfo::mappingImpl(yaml::IO &YamlIO) {
  MappingTraits<PPUMachineFunctionInfo>::mapping(YamlIO, *this);
}

bool PPUMachineFunctionInfo::initializeBaseYamlFields(
  const yaml::PPUMachineFunctionInfo &YamlMFI) {
  ExplicitKernArgSize = YamlMFI.ExplicitKernArgSize;
  MaxKernArgAlign = YamlMFI.MaxKernArgAlign;
  LDSSize = YamlMFI.LDSSize;
  HighBitsOf32BitAddress = YamlMFI.HighBitsOf32BitAddress;
  IsEntryFunction = YamlMFI.IsEntryFunction;
  NoSignedZerosFPMath = YamlMFI.NoSignedZerosFPMath;
  MemoryBound = YamlMFI.MemoryBound;
  WaveLimiter = YamlMFI.WaveLimiter;
  return false;
}
*/
