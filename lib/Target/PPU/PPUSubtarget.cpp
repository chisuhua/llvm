//===-- PPUSubtarget.cpp - PPU Subtarget Information ------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file implements the PPU specific subclass of TargetSubtargetInfo.
//
//===----------------------------------------------------------------------===//

#include "PPUSubtarget.h"
#include "PPU.h"
#include "PPUCallLowering.h"
#include "PPUFrameLowering.h"
#include "PPULegalizerInfo.h"
#include "PPURegisterBankInfo.h"
#include "PPUTargetMachine.h"
#include "PPURegisterInfo.h"
#include "PPUMachineFunction.h"
#include "PPUMachineFunctionInfo.h"
#include "llvm/Support/TargetRegistry.h"
#include "llvm/CodeGen/MachineScheduler.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/IR/MDBuilder.h"

using namespace llvm;

#define DEBUG_TYPE "ppu-subtarget"

#define GET_SUBTARGETINFO_TARGET_DESC
#define GET_SUBTARGETINFO_CTOR
#include "PPUGenSubtargetInfo.inc"

void PPUSubtarget::anchor() {}

PPUSubtarget &PPUSubtarget::initializeSubtargetDependencies(
    const Triple &TT, StringRef CPU, StringRef FS, StringRef ABIName) {
  // Determine default and user-specified characteristics
  bool Is64Bit = TT.isArch64Bit();
  std::string CPUName = CPU;
  if (CPUName.empty())
    CPUName = Is64Bit ? "generic-ppu64" : "generic-ppu";
  ParseSubtargetFeatures(CPUName, FS);
  if (Is64Bit) {
    XLenVT = MVT::i64;
    XLen = 64;
  }

  TargetABI = PPUABI::computeTargetABI(TT, getFeatureBits(), ABIName);
  PPUFeatures::validate(TT, getFeatureBits());
  return *this;
}

PPUBaseSubtarget::PPUBaseSubtarget(const Triple &TT, StringRef CPU, StringRef FS)
    : PPUGenSubtargetInfo(TT, CPU, FS)
    , TargetTriple(TT)
{}

std::pair<unsigned, unsigned>
PPUBaseSubtarget::getDefaultFlatWorkGroupSize(CallingConv::ID CC) const {
  switch (CC) {
  case CallingConv::AMDGPU_KERNEL:
  case CallingConv::SPIR_KERNEL:
    return std::make_pair(getWavefrontSize() * 2,
                          std::max(getWavefrontSize() * 4, 256u));
  default:
    return std::make_pair(1, 16 * getWavefrontSize());
  }
}

std::pair<unsigned, unsigned> PPUBaseSubtarget::getFlatWorkGroupSizes(
  const Function &F) const {
  // FIXME: 1024 if function.
  // Default minimum/maximum flat work group sizes.
  std::pair<unsigned, unsigned> Default = getDefaultFlatWorkGroupSize(F.getCallingConv());

  // Requested minimum/maximum flat work group sizes.
  std::pair<unsigned, unsigned> Requested = PPU::getIntegerPairAttribute(
    F, "ppu-flat-work-group-size", Default);

  // Make sure requested minimum is less than requested maximum.
  if (Requested.first > Requested.second)
    return Default;

  // Make sure requested values do not violate subtarget's specifications.
  if (Requested.first < getMinFlatWorkGroupSize())
    return Default;
  if (Requested.second > getMaxFlatWorkGroupSize())
    return Default;

  return Requested;
}

std::pair<unsigned, unsigned> PPUBaseSubtarget::getWavesPerEU(const Function &F) const {
  // Default minimum/maximum number of waves per execution unit.
  std::pair<unsigned, unsigned> Default(1, getMaxWavesPerEU());

  // Default/requested minimum/maximum flat work group sizes.
  std::pair<unsigned, unsigned> FlatWorkGroupSizes = getFlatWorkGroupSizes(F);

  // If minimum/maximum flat work group sizes were explicitly requested using
  // "ppu-flat-work-group-size" attribute, then set default minimum/maximum
  // number of waves per execution unit to values implied by requested
  // minimum/maximum flat work group sizes.
  unsigned MinImpliedByFlatWorkGroupSize =
    getMaxWavesPerEU(FlatWorkGroupSizes.second);
  bool RequestedFlatWorkGroupSize = false;

  if (F.hasFnAttribute("ppu-flat-work-group-size")) {
    Default.first = MinImpliedByFlatWorkGroupSize;
    RequestedFlatWorkGroupSize = true;
  }

  // Requested minimum/maximum number of waves per execution unit.
  std::pair<unsigned, unsigned> Requested = PPU::getIntegerPairAttribute(
    F, "ppu-waves-per-eu", Default, true);

  // Make sure requested minimum is less than requested maximum.
  if (Requested.second && Requested.first > Requested.second)
    return Default;

  // Make sure requested values do not violate subtarget's specifications.
  if (Requested.first < getMinWavesPerEU() ||
      Requested.first > getMaxWavesPerEU())
    return Default;
  if (Requested.second > getMaxWavesPerEU())
    return Default;

  // Make sure requested values are compatible with values implied by requested
  // minimum/maximum flat work group sizes.
  if (RequestedFlatWorkGroupSize &&
      Requested.first < MinImpliedByFlatWorkGroupSize)
    return Default;

  return Requested;
}

bool PPUBaseSubtarget::makeLIDRangeMetadata(Instruction *I) const {
  Function *Kernel = I->getParent()->getParent();
  unsigned MinSize = 0;
  unsigned MaxSize = getFlatWorkGroupSizes(*Kernel).second;
  bool IdQuery = false;

  // If reqd_work_group_size is present it narrows value down.
  if (auto *CI = dyn_cast<CallInst>(I)) {
    const Function *F = CI->getCalledFunction();
    if (F) {
      unsigned Dim = UINT_MAX;
      switch (F->getIntrinsicID()) {
      case Intrinsic::ppu_workitem_id_x:
        IdQuery = true;
        LLVM_FALLTHROUGH;
        break;
      case Intrinsic::ppu_workitem_id_y:
        IdQuery = true;
        LLVM_FALLTHROUGH;
        break;
      case Intrinsic::ppu_workitem_id_z:
        IdQuery = true;
        LLVM_FALLTHROUGH;
        break;
      default:
        break;
      }
      if (Dim <= 3) {
        if (auto Node = Kernel->getMetadata("reqd_work_group_size"))
          if (Node->getNumOperands() == 3)
            MinSize = MaxSize = mdconst::extract<ConstantInt>(
                                  Node->getOperand(Dim))->getZExtValue();
      }
    }
  }

  if (!MaxSize)
    return false;

  // Range metadata is [Lo, Hi). For ID query we need to pass max size
  // as Hi. For size query we need to pass Hi + 1.
  if (IdQuery)
    MinSize = 0;
  else
    ++MaxSize;

  MDBuilder MDB(I->getContext());
  MDNode *MaxWorkGroupSizeRange = MDB.createRange(APInt(32, MinSize),
                                                  APInt(32, MaxSize));
  I->setMetadata(LLVMContext::MD_range, MaxWorkGroupSizeRange);
  return true;
}

uint64_t PPUBaseSubtarget::getExplicitKernArgSize(const Function &F,
                                                 unsigned &MaxAlign) const {
  assert(F.getCallingConv() == CallingConv::AMDGPU_KERNEL ||
         F.getCallingConv() == CallingConv::SPIR_KERNEL);

  const DataLayout &DL = F.getParent()->getDataLayout();
  uint64_t ExplicitArgBytes = 0;
  MaxAlign = 1;

  for (const Argument &Arg : F.args()) {
    Type *ArgTy = Arg.getType();

    unsigned Align = DL.getABITypeAlignment(ArgTy);
    uint64_t AllocSize = DL.getTypeAllocSize(ArgTy);
    ExplicitArgBytes = alignTo(ExplicitArgBytes, Align) + AllocSize;
    MaxAlign = std::max(MaxAlign, Align);
  }

  return ExplicitArgBytes;
}

unsigned PPUBaseSubtarget::getKernArgSegmentSize(const Function &F,
                                                unsigned &MaxAlign) const {
  uint64_t ExplicitArgBytes = getExplicitKernArgSize(F, MaxAlign);

  unsigned ExplicitOffset = getExplicitKernelArgOffset(F);

  uint64_t TotalSize = ExplicitOffset + ExplicitArgBytes;
  unsigned ImplicitBytes = getImplicitArgNumBytes(F);
  if (ImplicitBytes != 0) {
    unsigned Alignment = getAlignmentForImplicitArgPtr();
    TotalSize = alignTo(ExplicitArgBytes, Alignment) + ImplicitBytes;
  }

  // Being able to dereference past the end is useful for emitting scalar loads.
  return alignTo(TotalSize, 4);
}

unsigned PPUBaseSubtarget::getMaxLocalMemSizeWithWaveCount(unsigned NWaves,
  const Function &F) const {
  if (NWaves == 1)
    return getLocalMemorySize();
  unsigned WorkGroupSize = getFlatWorkGroupSizes(F).second;
  unsigned WorkGroupsPerCu = getMaxWorkGroupsPerCU(WorkGroupSize);
  if (!WorkGroupsPerCu)
    return 0;
  unsigned MaxWaves = getMaxWavesPerEU();
  return getLocalMemorySize() * MaxWaves / WorkGroupsPerCu / NWaves;
}

unsigned PPUBaseSubtarget::getOccupancyWithLocalMemSize(uint32_t Bytes,
  const Function &F) const {
  unsigned WorkGroupSize = getFlatWorkGroupSizes(F).second;
  unsigned WorkGroupsPerCu = getMaxWorkGroupsPerCU(WorkGroupSize);
  if (!WorkGroupsPerCu)
    return 0;
  unsigned MaxWaves = getMaxWavesPerEU();
  unsigned Limit = getLocalMemorySize() * MaxWaves / WorkGroupsPerCu;
  unsigned NumWaves = Limit / (Bytes ? Bytes : 1u);
  NumWaves = std::min(NumWaves, MaxWaves);
  NumWaves = std::max(NumWaves, 1u);
  return NumWaves;
}

PPUSubtarget::PPUSubtarget(const Triple &TT, StringRef CPU, StringRef FS,
                               StringRef ABIName, const TargetMachine &TM)
    : PPUBaseSubtarget(TT, CPU, FS)
    , FrameLowering(initializeSubtargetDependencies(TT, CPU, FS, ABIName))
    , InstrInfo(*this), RegInfo(*this, getHwMode()), TLInfo(TM, *this) {
  CallLoweringInfo.reset(new PPUCallLowering(*getTargetLowering()));
  Legalizer.reset(new PPULegalizerInfo(*this));

  auto *RBI = new PPURegisterBankInfo(*getRegisterInfo());
  RegBankInfo.reset(RBI);
  InstSelector.reset(createPPUInstructionSelector(
      *static_cast<const PPUTargetMachine *>(&TM), *this, *RBI));
}

const CallLowering *PPUSubtarget::getCallLowering() const {
  return CallLoweringInfo.get();
}

InstructionSelector *PPUSubtarget::getInstructionSelector() const {
  return InstSelector.get();
}

const LegalizerInfo *PPUSubtarget::getLegalizerInfo() const {
  return Legalizer.get();
}

const RegisterBankInfo *PPUSubtarget::getRegBankInfo() const {
  return RegBankInfo.get();
}

void PPUSubtarget::overrideSchedPolicy(MachineSchedPolicy &Policy,
                                      unsigned NumRegionInstrs) const {
  // Track register pressure so the scheduler can try to decrease
  // pressure once register usage is above the threshold defined by
  // SIRegisterInfo::getRegPressureSetLimit()
  Policy.ShouldTrackPressure = true;

  // Enabling both top down and bottom up scheduling seems to give us less
  // register spills than just using one of these approaches on its own.
  Policy.OnlyTopDown = false;
  Policy.OnlyBottomUp = false;

  // Enabling ShouldTrackLaneMasks crashes the PPU Machine Scheduler.
  if (!enablePPUScheduler())
    Policy.ShouldTrackLaneMasks = true;
}

bool PPUSubtarget::hasMadF16() const {
  return InstrInfo.pseudoToMCOpcode(PPU::V_MAD_F16) != -1;
}

unsigned PPUSubtarget::getOccupancyWithNumSGPRs(unsigned SGPRs) const {
    return getMaxWavesPerEU();
}

unsigned PPUSubtarget::getOccupancyWithNumVGPRs(unsigned VGPRs) const {
  unsigned MaxWaves = getMaxWavesPerEU();
  unsigned Granule = getVGPRAllocGranule();
  if (VGPRs < Granule)
    return MaxWaves;
  unsigned RoundedRegs = ((VGPRs + Granule - 1) / Granule) * Granule;
  return std::min(getTotalNumVGPRs() / RoundedRegs, MaxWaves);
}

unsigned PPUSubtarget::getReservedNumSGPRs(const MachineFunction &MF) const {
  return 2; // VCC.
}

unsigned PPUSubtarget::computeOccupancy(const MachineFunction &MF,
                                        unsigned LDSSize,
                                        unsigned NumSGPRs,
                                        unsigned NumVGPRs) const {
  unsigned Occupancy = std::min(getMaxWavesPerEU(), getOccupancyWithLocalMemSize(LDSSize, MF.getFunction()));
  if (NumSGPRs)
    Occupancy = std::min(Occupancy, getOccupancyWithNumSGPRs(NumSGPRs));
  if (NumVGPRs)
    Occupancy = std::min(Occupancy, getOccupancyWithNumVGPRs(NumVGPRs));
  return Occupancy;
}

unsigned PPUSubtarget::getMaxNumSGPRs(const MachineFunction &MF) const {
  const Function &F = MF.getFunction();
  const PPUMachineFunctionInfo &MFI = *MF.getInfo<PPUMachineFunctionInfo>();
  // const PPUMachineFunctionInfo &MFI = *dynamic_cast<PPUMachineFunctionInfo*>(MF.getInfo<MachineFunctionInfo>());

  // Compute maximum number of SGPRs function can use using default/requested
  // minimum number of waves per execution unit.
  // std::pair<unsigned, unsigned> WavesPerEU = MFI.getWavesPerEU();
  std::pair<unsigned, unsigned> WavesPerEU = getWavesPerEU(F);
  unsigned MaxNumSGPRs = getMaxNumSGPRs(WavesPerEU.first, false);
  unsigned MaxAddressableNumSGPRs = getMaxNumSGPRs(WavesPerEU.first, true);

  // Check if maximum number of SGPRs was explicitly requested using
  // "ppu-num-sgpr" attribute.
  if (F.hasFnAttribute("ppu-num-sgpr")) {
    unsigned Requested = PPU::getIntegerAttribute(F, "ppu-num-sgpr", MaxNumSGPRs);

    // Make sure requested value does not violate subtarget's specifications.
    if (Requested && (Requested <= getReservedNumSGPRs(MF)))
      Requested = 0;

    // If more SGPRs are required to support the input user/system SGPRs,
    // increase to accommodate them.
    //
    // FIXME: This really ends up using the requested number of SGPRs + number
    // of reserved special registers in total. Theoretically you could re-use
    // the last input registers for these special registers, but this would
    // require a lot of complexity to deal with the weird aliasing.
    /*
    unsigned InputNumSGPRs = MFI.getNumPreloadedSGPRs();
    if (Requested && Requested < InputNumSGPRs)
      Requested = InputNumSGPRs;
      */

    // Make sure requested value is compatible with values implied by
    // default/requested minimum/maximum number of waves per execution unit.
    if (Requested && Requested > getMaxNumSGPRs(WavesPerEU.first, false))
      Requested = 0;
    if (WavesPerEU.second && Requested && Requested < getMinNumSGPRs(WavesPerEU.second))
      Requested = 0;

    if (Requested)
      MaxNumSGPRs = Requested;
  }

  return std::min(MaxNumSGPRs - getReservedNumSGPRs(MF),
                  MaxAddressableNumSGPRs);
}

unsigned PPUSubtarget::getMaxNumVGPRs(const MachineFunction &MF) const {
  const Function &F = MF.getFunction();
  const PPUMachineFunctionInfo &MFI = *MF.getInfo<PPUMachineFunctionInfo>();

  // Compute maximum number of VGPRs function can use using default/requested
  // minimum number of waves per execution unit.
  // std::pair<unsigned, unsigned> WavesPerEU = MFI.getWavesPerEU(F);
  std::pair<unsigned, unsigned> WavesPerEU = getWavesPerEU(F);
  unsigned MaxNumVGPRs = getMaxNumVGPRs(WavesPerEU.first);

  // Check if maximum number of VGPRs was explicitly requested using
  // "ppu-num-vgpr" attribute.
  if (F.hasFnAttribute("ppu-num-vgpr")) {
    unsigned Requested = PPU::getIntegerAttribute(
      F, "ppu-num-vgpr", MaxNumVGPRs);

    // Make sure requested value is compatible with values implied by
    // default/requested minimum/maximum number of waves per execution unit.
    if (Requested && Requested > getMaxNumVGPRs(WavesPerEU.first))
      Requested = 0;
    if (WavesPerEU.second &&
        Requested && Requested < getMinNumVGPRs(WavesPerEU.second))
      Requested = 0;

    if (Requested)
      MaxNumVGPRs = Requested;
  }

  return MaxNumVGPRs;
}

namespace {
struct MemOpClusterMutation : ScheduleDAGMutation {
  const PPUInstrInfo *TII;

  MemOpClusterMutation(const PPUInstrInfo *tii) : TII(tii) {}

  void apply(ScheduleDAGInstrs *DAG) override {
    SUnit *SUa = nullptr;
    // Search for two consequent memory operations and link them
    // to prevent scheduler from moving them apart.
    // In DAG pre-process SUnits are in the original order of
    // the instructions before scheduling.
    for (SUnit &SU : DAG->SUnits) {
      MachineInstr &MI2 = *SU.getInstr();
      if (!MI2.mayLoad() && !MI2.mayStore()) {
        SUa = nullptr;
        continue;
      }
      if (!SUa) {
        SUa = &SU;
        continue;
      }

      MachineInstr &MI1 = *SUa->getInstr();
      /* FIXME
      if ((TII->isVMEM(MI1) && TII->isVMEM(MI2)) ||
          (TII->isFLAT(MI1) && TII->isFLAT(MI2)) ||
          (TII->isSMRD(MI1) && TII->isSMRD(MI2)) ||
          (TII->isDS(MI1)   && TII->isDS(MI2))) {
        SU.addPredBarrier(SUa);

        for (const SDep &SI : SU.Preds) {
          if (SI.getSUnit() != SUa)
            SUa->addPred(SDep(SI.getSUnit(), SDep::Artificial));
        }

        if (&SU != &DAG->ExitSU) {
          for (const SDep &SI : SUa->Succs) {
            if (SI.getSUnit() != &SU)
              SI.getSUnit()->addPred(SDep(&SU, SDep::Artificial));
          }
        }
      }
      */

      SUa = &SU;
    }
  }
};

struct FillMFMAShadowMutation : ScheduleDAGMutation {
  const PPUInstrInfo *TII;

  ScheduleDAGMI *DAG;

  FillMFMAShadowMutation(const PPUInstrInfo *tii) : TII(tii) {}

  bool isSALU(const SUnit *SU) const {
    const MachineInstr *MI = SU->getInstr();
    return MI && TII->isSALU(*MI) && !MI->isTerminator();
  }

  bool canAddEdge(const SUnit *Succ, const SUnit *Pred) const {
    if (Pred->NodeNum < Succ->NodeNum)
      return true;

    SmallVector<const SUnit*, 64> Succs({Succ}), Preds({Pred});

    for (unsigned I = 0; I < Succs.size(); ++I) {
      for (const SDep &SI : Succs[I]->Succs) {
        const SUnit *SU = SI.getSUnit();
        if (SU != Succs[I] && llvm::find(Succs, SU) == Succs.end())
          Succs.push_back(SU);
      }
    }

    SmallPtrSet<const SUnit*, 32> Visited;
    while (!Preds.empty()) {
      const SUnit *SU = Preds.pop_back_val();
      if (llvm::find(Succs, SU) != Succs.end())
        return false;
      Visited.insert(SU);
      for (const SDep &SI : SU->Preds)
        if (SI.getSUnit() != SU && !Visited.count(SI.getSUnit()))
          Preds.push_back(SI.getSUnit());
    }

    return true;
  }

  // Link as much SALU intructions in chain as possible. Return the size
  // of the chain. Links up to MaxChain instructions.
  unsigned linkSALUChain(SUnit *From, SUnit *To, unsigned MaxChain,
                         SmallPtrSetImpl<SUnit *> &Visited) const {
    SmallVector<SUnit *, 8> Worklist({To});
    unsigned Linked = 0;

    while (!Worklist.empty() && MaxChain-- > 0) {
      SUnit *SU = Worklist.pop_back_val();
      if (!Visited.insert(SU).second)
        continue;

      LLVM_DEBUG(dbgs() << "Inserting edge from\n" ; DAG->dumpNode(*From);
                 dbgs() << "to\n"; DAG->dumpNode(*SU); dbgs() << '\n');

      if (SU->addPred(SDep(From, SDep::Artificial), false))
        ++Linked;

      for (SDep &SI : From->Succs) {
        SUnit *SUv = SI.getSUnit();
        if (SUv != From && TII->isVALU(*SUv->getInstr()) && canAddEdge(SUv, SU))
          SUv->addPred(SDep(SU, SDep::Artificial), false);
      }

      for (SDep &SI : SU->Succs) {
        SUnit *Succ = SI.getSUnit();
        if (Succ != SU && isSALU(Succ) && canAddEdge(From, Succ))
          Worklist.push_back(Succ);
      }
    }

    return Linked;
  }

  void apply(ScheduleDAGInstrs *DAGInstrs) override {
    const PPUSubtarget &ST = DAGInstrs->MF.getSubtarget<PPUSubtarget>();
    /*
    if (!ST.hasMAIInsts() || DisablePowerSched)
      return;
      */
    DAG = static_cast<ScheduleDAGMI*>(DAGInstrs);
    const TargetSchedModel *TSchedModel = DAGInstrs->getSchedModel();
    if (!TSchedModel || DAG->SUnits.empty())
      return;

    // Scan for MFMA long latency instructions and try to add a dependency
    // of available SALU instructions to give them a chance to fill MFMA
    // shadow. That is desirable to fill MFMA shadow with SALU instructions
    // rather than VALU to prevent power consumption bursts and throttle.
    auto LastSALU = DAG->SUnits.begin();
    auto E = DAG->SUnits.end();
    SmallPtrSet<SUnit*, 32> Visited;
    for (SUnit &SU : DAG->SUnits) {
      MachineInstr &MAI = *SU.getInstr();
      /* FIXME
      if (!TII->isMAI(MAI) ||
           MAI.getOpcode() == AMDGPU::V_ACCVGPR_WRITE_B32 ||
           MAI.getOpcode() == AMDGPU::V_ACCVGPR_READ_B32)
        continue;
        */

      unsigned Lat = TSchedModel->computeInstrLatency(&MAI) - 1;

      LLVM_DEBUG(dbgs() << "Found MFMA: "; DAG->dumpNode(SU);
                 dbgs() << "Need " << Lat
                        << " instructions to cover latency.\n");

      // Find up to Lat independent scalar instructions as early as
      // possible such that they can be scheduled after this MFMA.
      for ( ; Lat && LastSALU != E; ++LastSALU) {
        if (Visited.count(&*LastSALU))
          continue;

        if (!isSALU(&*LastSALU) || !canAddEdge(&*LastSALU, &SU))
          continue;

        Lat -= linkSALUChain(&SU, &*LastSALU, Lat, Visited);
      }
    }
  }
};
} // namespace

void PPUSubtarget::getPostRAMutations(
    std::vector<std::unique_ptr<ScheduleDAGMutation>> &Mutations) const {
  Mutations.push_back(std::make_unique<MemOpClusterMutation>(&InstrInfo));
  Mutations.push_back(std::make_unique<FillMFMAShadowMutation>(&InstrInfo));
}

const PPUSubtarget &PPUSubtarget::get(const MachineFunction &MF) {
  // if (MF.getTarget().getTargetTriple().getArch() == Triple::ppu)
    return static_cast<const PPUSubtarget&>(MF.getSubtarget<PPUSubtarget>());
}

const PPUSubtarget &PPUSubtarget::get(const TargetMachine &TM, const Function &F) {
  // if (TM.getTargetTriple().getArch() == Triple::ppu)
    return static_cast<const PPUSubtarget&>(TM.getSubtarget<PPUSubtarget>(F));
}
