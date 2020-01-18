//===-- PPUCallLowering.cpp - Call lowering -------------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
/// \file
/// This file implements the lowering of LLVM calls to machine code calls for
/// GlobalISel.
//
//===----------------------------------------------------------------------===//

#include "PPUCallLowering.h"
#include "PPU.h"
#include "PPUBaseISelLowering.h"
#include "PPUISelLowering.h"
#include "PPURegisterInfo.h"
#include "PPUMachineFunctionInfo.h"
#include "PPUSubtarget.h"
#include "PPURegisterInfo.h"
#include "llvm/CodeGen/Analysis.h"
#include "llvm/CodeGen/GlobalISel/MachineIRBuilder.h"
#include "llvm/CodeGen/CallingConvLower.h"
#include "llvm/CodeGen/GlobalISel/MachineIRBuilder.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/Support/LowLevelTypeImpl.h"

using namespace llvm;

#include "PPUGenCallingConv.inc"


PPUCallLowering::PPUCallLowering(const PPUTargetLowering &TLI)
    : CallLowering(&TLI) {}

bool PPUCallLowering::lowerReturn(MachineIRBuilder &MIRBuilder,
                                    const Value *Val,
                                    ArrayRef<Register> VRegs) const {

  CallingConv::ID CC = MIRBuilder.getMF().getFunction().getCallingConv();
  if (PPU::isCompute(CC))
      return lowerReturn_compute(MIRBuilder, Val, VRegs);

  MachineInstrBuilder Ret = MIRBuilder.buildInstrNoInsert(PPU::PseudoRET);

  if (Val != nullptr) {
    return false;
  }
  MIRBuilder.insertInstr(Ret);
  return true;
}

bool PPUCallLowering::lowerFormalArguments(
    MachineIRBuilder &MIRBuilder, const Function &F,
    ArrayRef<ArrayRef<Register>> VRegs) const {

  CallingConv::ID CC = F.getCallingConv();
  if (PPU::isCompute(CC))
      return lowerFormalArguments_compute(MIRBuilder, F, VRegs);


  if (F.arg_empty())
    return true;

  return false;
}

bool PPUCallLowering::lowerCall(MachineIRBuilder &MIRBuilder,
                                  CallLoweringInfo &Info) const {
  return false;
}

namespace {

struct OutgoingValueHandler : public CallLowering::ValueHandler {
  OutgoingValueHandler(MachineIRBuilder &MIRBuilder, MachineRegisterInfo &MRI,
                       MachineInstrBuilder MIB, CCAssignFn *AssignFn)
      : ValueHandler(MIRBuilder, MRI, AssignFn), MIB(MIB) {}

  MachineInstrBuilder MIB;

  Register getStackAddress(uint64_t Size, int64_t Offset,
                           MachinePointerInfo &MPO) override {
    llvm_unreachable("not implemented");
  }

  void assignValueToAddress(Register ValVReg, Register Addr, uint64_t Size,
                            MachinePointerInfo &MPO, CCValAssign &VA) override {
    llvm_unreachable("not implemented");
  }

  void assignValueToReg(Register ValVReg, Register PhysReg,
                        CCValAssign &VA) override {
    Register ExtReg;
    if (VA.getLocVT().getSizeInBits() < 32) {
      // 16-bit types are reported as legal for 32-bit registers. We need to
      // extend and do a 32-bit copy to avoid the verifier complaining about it.
      ExtReg = MIRBuilder.buildAnyExt(LLT::scalar(32), ValVReg).getReg(0);
    } else
      ExtReg = extendRegister(ValVReg, VA);

    MIRBuilder.buildCopy(PhysReg, ExtReg);
    MIB.addUse(PhysReg, RegState::Implicit);
  }

  bool assignArg(unsigned ValNo, MVT ValVT, MVT LocVT,
                 CCValAssign::LocInfo LocInfo,
                 const CallLowering::ArgInfo &Info,
                 CCState &State) override {
    return AssignFn(ValNo, ValVT, LocVT, LocInfo, Info.Flags, State);
  }
};

struct IncomingArgHandler : public CallLowering::ValueHandler {
  uint64_t StackUsed = 0;

  IncomingArgHandler(MachineIRBuilder &MIRBuilder, MachineRegisterInfo &MRI,
                     CCAssignFn *AssignFn)
    : ValueHandler(MIRBuilder, MRI, AssignFn) {}

  Register getStackAddress(uint64_t Size, int64_t Offset,
                           MachinePointerInfo &MPO) override {
    auto &MFI = MIRBuilder.getMF().getFrameInfo();
    int FI = MFI.CreateFixedObject(Size, Offset, true);
    MPO = MachinePointerInfo::getFixedStack(MIRBuilder.getMF(), FI);
    Register AddrReg = MRI.createGenericVirtualRegister(
      LLT::pointer(AMDGPUAS::PRIVATE_ADDRESS, 32));
    MIRBuilder.buildFrameIndex(AddrReg, FI);
    StackUsed = std::max(StackUsed, Size + Offset);
    return AddrReg;
  }

  void assignValueToReg(Register ValVReg, Register PhysReg,
                        CCValAssign &VA) override {
    markPhysRegUsed(PhysReg);

    if (VA.getLocVT().getSizeInBits() < 32) {
      // 16-bit types are reported as legal for 32-bit registers. We need to do
      // a 32-bit copy, and truncate to avoid the verifier complaining about it.
      auto Copy = MIRBuilder.buildCopy(LLT::scalar(32), PhysReg);
      MIRBuilder.buildTrunc(ValVReg, Copy);
      return;
    }

    switch (VA.getLocInfo()) {
    case CCValAssign::LocInfo::SExt:
    case CCValAssign::LocInfo::ZExt:
    case CCValAssign::LocInfo::AExt: {
      auto Copy = MIRBuilder.buildCopy(LLT{VA.getLocVT()}, PhysReg);
      MIRBuilder.buildTrunc(ValVReg, Copy);
      break;
    }
    default:
      MIRBuilder.buildCopy(ValVReg, PhysReg);
      break;
    }
  }

  void assignValueToAddress(Register ValVReg, Register Addr, uint64_t Size,
                            MachinePointerInfo &MPO, CCValAssign &VA) override {
    // FIXME: Get alignment
    auto MMO = MIRBuilder.getMF().getMachineMemOperand(
      MPO, MachineMemOperand::MOLoad | MachineMemOperand::MOInvariant, Size, 1);
    MIRBuilder.buildLoad(ValVReg, Addr, *MMO);
  }

  /// How the physical register gets marked varies between formal
  /// parameters (it's a basic-block live-in), and a call instruction
  /// (it's an implicit-def of the BL).
  virtual void markPhysRegUsed(unsigned PhysReg) = 0;

  // FIXME: What is the point of this being a callback?
  bool isIncomingArgumentHandler() const override { return true; }
};

struct FormalArgHandler : public IncomingArgHandler {
  FormalArgHandler(MachineIRBuilder &MIRBuilder, MachineRegisterInfo &MRI,
                   CCAssignFn *AssignFn)
    : IncomingArgHandler(MIRBuilder, MRI, AssignFn) {}

  void markPhysRegUsed(unsigned PhysReg) override {
    MIRBuilder.getMBB().addLiveIn(PhysReg);
  }
};

}

void PPUCallLowering::splitToValueTypes(
    const ArgInfo &OrigArg, SmallVectorImpl<ArgInfo> &SplitArgs,
    const DataLayout &DL, MachineRegisterInfo &MRI, CallingConv::ID CallConv,
    SplitArgTy PerformArgSplit) const {
  const PPUTargetLowering &TLI = *getTLI<PPUTargetLowering>();
  LLVMContext &Ctx = OrigArg.Ty->getContext();

  if (OrigArg.Ty->isVoidTy())
    return;

  SmallVector<EVT, 4> SplitVTs;
  ComputeValueVTs(TLI, DL, OrigArg.Ty, SplitVTs);

  assert(OrigArg.Regs.size() == SplitVTs.size());

  int SplitIdx = 0;
  for (EVT VT : SplitVTs) {
    unsigned NumParts = TLI.getNumRegistersForCallingConv(Ctx, CallConv, VT);
    Type *Ty = VT.getTypeForEVT(Ctx);



    if (NumParts == 1) {
      // No splitting to do, but we want to replace the original type (e.g. [1 x
      // double] -> double).
      SplitArgs.emplace_back(OrigArg.Regs[SplitIdx], Ty,
                             OrigArg.Flags, OrigArg.IsFixed);

      ++SplitIdx;
      continue;
    }

    LLT LLTy = getLLTForType(*Ty, DL);

    SmallVector<Register, 8> SplitRegs;

    EVT PartVT = TLI.getRegisterTypeForCallingConv(Ctx, CallConv, VT);
    Type *PartTy = PartVT.getTypeForEVT(Ctx);
    LLT PartLLT = getLLTForType(*PartTy, DL);

    // FIXME: Should we be reporting all of the part registers for a single
    // argument, and let handleAssignments take care of the repacking?
    for (unsigned i = 0; i < NumParts; ++i) {
      Register PartReg = MRI.createGenericVirtualRegister(PartLLT);
      SplitRegs.push_back(PartReg);
      SplitArgs.emplace_back(ArrayRef<Register>(PartReg), PartTy, OrigArg.Flags);
    }

    PerformArgSplit(SplitRegs, LLTy, PartLLT, SplitIdx);

    ++SplitIdx;
  }
}

// Get the appropriate type to make \p OrigTy \p Factor times bigger.
static LLT getMultipleType(LLT OrigTy, int Factor) {
  if (OrigTy.isVector()) {
    return LLT::vector(OrigTy.getNumElements() * Factor,
                       OrigTy.getElementType());
  }

  return LLT::scalar(OrigTy.getSizeInBits() * Factor);
}

// TODO: Move to generic code
static void unpackRegsToOrigType(MachineIRBuilder &MIRBuilder,
                                 ArrayRef<Register> DstRegs,
                                 Register SrcReg,
                                 LLT SrcTy,
                                 LLT PartTy) {
  assert(DstRegs.size() > 1 && "Nothing to unpack");

  MachineFunction &MF = MIRBuilder.getMF();
  MachineRegisterInfo &MRI = MF.getRegInfo();

  const unsigned SrcSize = SrcTy.getSizeInBits();
  const unsigned PartSize = PartTy.getSizeInBits();

  if (SrcTy.isVector() && !PartTy.isVector() &&
      PartSize > SrcTy.getElementType().getSizeInBits()) {
    // Vector was scalarized, and the elements extended.
    auto UnmergeToEltTy = MIRBuilder.buildUnmerge(SrcTy.getElementType(),
                                                  SrcReg);
    for (int i = 0, e = DstRegs.size(); i != e; ++i)
      MIRBuilder.buildAnyExt(DstRegs[i], UnmergeToEltTy.getReg(i));
    return;
  }

  if (SrcSize % PartSize == 0) {
    MIRBuilder.buildUnmerge(DstRegs, SrcReg);
    return;
  }

  const int NumRoundedParts = (SrcSize + PartSize - 1) / PartSize;

  LLT BigTy = getMultipleType(PartTy, NumRoundedParts);
  auto ImpDef = MIRBuilder.buildUndef(BigTy);

  Register BigReg = MRI.createGenericVirtualRegister(BigTy);
  MIRBuilder.buildInsert(BigReg, ImpDef.getReg(0), SrcReg, 0).getReg(0);

  int64_t Offset = 0;
  for (unsigned i = 0, e = DstRegs.size(); i != e; ++i, Offset += PartSize)
    MIRBuilder.buildExtract(DstRegs[i], BigReg, Offset);
}

/// Lower the return value for the already existing \p Ret. This assumes that
/// \p MIRBuilder's insertion point is correct.
bool PPUCallLowering::lowerReturnVal_compute(MachineIRBuilder &MIRBuilder,
                                        const Value *Val, ArrayRef<Register> VRegs,
                                        MachineInstrBuilder &Ret) const {
  if (!Val)
    return true;

  auto &MF = MIRBuilder.getMF();
  const auto &F = MF.getFunction();
  const DataLayout &DL = MF.getDataLayout();

  CallingConv::ID CC = F.getCallingConv();
  const PPUTargetLowering &TLI = *getTLI<PPUTargetLowering>();
  MachineRegisterInfo &MRI = MF.getRegInfo();

  ArgInfo OrigRetInfo(VRegs, Val->getType());
  setArgFlags(OrigRetInfo, AttributeList::ReturnIndex, DL, F);
  SmallVector<ArgInfo, 4> SplitRetInfos;

  splitToValueTypes(
    OrigRetInfo, SplitRetInfos, DL, MRI, CC,
    [&](ArrayRef<Register> Regs, LLT LLTy, LLT PartLLT, int VTSplitIdx) {
      unpackRegsToOrigType(MIRBuilder, Regs, VRegs[VTSplitIdx], LLTy, PartLLT);
    });

  CCAssignFn *AssignFn = TLI.CCAssignFnForReturn(CC, F.isVarArg());

  OutgoingValueHandler RetHandler(MIRBuilder, MF.getRegInfo(), Ret, AssignFn);
  return handleAssignments(MIRBuilder, SplitRetInfos, RetHandler);
}

bool PPUCallLowering::lowerReturn_compute(MachineIRBuilder &MIRBuilder,
                                     const Value *Val,
                                     ArrayRef<Register> VRegs) const {

  MachineFunction &MF = MIRBuilder.getMF();
  MachineRegisterInfo &MRI = MF.getRegInfo();
  PPUMachineFunctionInfo *MFI = MF.getInfo<PPUMachineFunctionInfo>();
  MFI->setIfReturnsVoid(!Val);

  assert(!Val == VRegs.empty() && "Return value without a vreg");

  CallingConv::ID CC = MIRBuilder.getMF().getFunction().getCallingConv();
  const bool IsKernel = PPU::isKernel(CC);
  const bool IsCompute = PPU::isCompute(CC);

  const bool IsWaveEnd = IsKernel || (IsCompute && MFI->returnsVoid());
  if (IsWaveEnd) {
    MIRBuilder.buildInstr(PPU::ENDPGM)
      .addImm(0);
    return true;
  }

  auto const &ST = MIRBuilder.getMF().getSubtarget<PPUSubtarget>();

  unsigned ReturnOpc = PPU::S_SETPC_B64_return;

  // delete some some from original code

  auto Ret = MIRBuilder.buildInstrNoInsert(ReturnOpc);
  Register ReturnAddrVReg;

  ReturnAddrVReg = MRI.createVirtualRegister(&PPU::CCR_SPR_64RegClass);
  Ret.addUse(ReturnAddrVReg);

  if (!lowerReturnVal_compute(MIRBuilder, Val, VRegs, Ret))
    return false;

  const PPURegisterInfo *TRI = ST.getRegisterInfo();
  Register LiveInReturn = MF.addLiveIn(TRI->getReturnAddressReg(MF),
                                         &PPU::SPR_64RegClass);
  MIRBuilder.buildCopy(ReturnAddrVReg, LiveInReturn);

  // TODO: Handle CalleeSavedRegsViaCopy.

  MIRBuilder.insertInstr(Ret);
  return true;
}

Register PPUCallLowering::lowerParameterPtr_compute(MachineIRBuilder &MIRBuilder,
                                               Type *ParamTy,
                                               uint64_t Offset) const {

  MachineFunction &MF = MIRBuilder.getMF();
  const PPUMachineFunctionInfo *MFI = MF.getInfo<PPUMachineFunctionInfo>();
  MachineRegisterInfo &MRI = MF.getRegInfo();
  const Function &F = MF.getFunction();
  const DataLayout &DL = F.getParent()->getDataLayout();
  PointerType *PtrTy = PointerType::get(ParamTy, AMDGPUAS::CONSTANT_ADDRESS);
  LLT PtrType = getLLTForType(*PtrTy, DL);
  Register DstReg = MRI.createGenericVirtualRegister(PtrType);
  Register KernArgSegmentPtr =
    MFI->getPreloadedReg(PPUFunctionArgInfo::KERNARG_SEGMENT_PTR);
  Register KernArgSegmentVReg = MRI.getLiveInVirtReg(KernArgSegmentPtr);

  Register OffsetReg = MRI.createGenericVirtualRegister(LLT::scalar(64));
  MIRBuilder.buildConstant(OffsetReg, Offset);

  MIRBuilder.buildGEP(DstReg, KernArgSegmentVReg, OffsetReg);

  return DstReg;
}

void PPUCallLowering::lowerParameter_compute(MachineIRBuilder &MIRBuilder,
                                        Type *ParamTy, uint64_t Offset,
                                        unsigned Align,
                                        Register DstReg) const {
  MachineFunction &MF = MIRBuilder.getMF();
  const Function &F = MF.getFunction();
  const DataLayout &DL = F.getParent()->getDataLayout();
  PointerType *PtrTy = PointerType::get(ParamTy, AMDGPUAS::CONSTANT_ADDRESS);
  MachinePointerInfo PtrInfo(UndefValue::get(PtrTy));
  unsigned TypeSize = DL.getTypeStoreSize(ParamTy);
  Register PtrReg = lowerParameterPtr_compute(MIRBuilder, ParamTy, Offset);

  MachineMemOperand *MMO =
      MF.getMachineMemOperand(PtrInfo, MachineMemOperand::MOLoad |
                                       MachineMemOperand::MODereferenceable |
                                       MachineMemOperand::MOInvariant,
                                       TypeSize, Align);

  MIRBuilder.buildLoad(DstReg, PtrReg, *MMO);
}

// Allocate special inputs passed in user SGPRs.
static void allocateHSAUserSGPRs(CCState &CCInfo,
                                 MachineIRBuilder &MIRBuilder,
                                 MachineFunction &MF,
                                 const PPURegisterInfo &TRI,
                                 PPUMachineFunctionInfo &Info) {
  // FIXME: How should these inputs interact with inreg / custom SGPR inputs?
  if (Info.hasPrivateSegmentBuffer()) {
    unsigned PrivateSegmentBufferReg = Info.addPrivateSegmentBuffer(TRI);
    MF.addLiveIn(PrivateSegmentBufferReg, &PPU::SPR_64RegClass);
    CCInfo.AllocateReg(PrivateSegmentBufferReg);
  }

  if (Info.hasDispatchPtr()) {
    unsigned DispatchPtrReg = Info.addDispatchPtr(TRI);
    MF.addLiveIn(DispatchPtrReg, &PPU::SPR_64RegClass);
    CCInfo.AllocateReg(DispatchPtrReg);
  }

  if (Info.hasQueuePtr()) {
    unsigned QueuePtrReg = Info.addQueuePtr(TRI);
    MF.addLiveIn(QueuePtrReg, &PPU::SPR_64RegClass);
    CCInfo.AllocateReg(QueuePtrReg);
  }

  if (Info.hasKernargSegmentPtr()) {
    MachineRegisterInfo &MRI = MF.getRegInfo();
    Register InputPtrReg = Info.addKernargSegmentPtr(TRI);
    const LLT P4 = LLT::pointer(AMDGPUAS::CONSTANT_ADDRESS, 64);
    Register VReg = MRI.createGenericVirtualRegister(P4);
    MRI.addLiveIn(InputPtrReg, VReg);
    MIRBuilder.getMBB().addLiveIn(InputPtrReg);
    MIRBuilder.buildCopy(VReg, InputPtrReg);
    CCInfo.AllocateReg(InputPtrReg);
  }

  if (Info.hasDispatchID()) {
    unsigned DispatchIDReg = Info.addDispatchID(TRI);
    MF.addLiveIn(DispatchIDReg, &PPU::SPR_32RegClass);
    CCInfo.AllocateReg(DispatchIDReg);
  }

  if (Info.hasFlatScratchInit()) {
    unsigned FlatScratchInitReg = Info.addFlatScratchInit(TRI);
    MF.addLiveIn(FlatScratchInitReg, &PPU::SPR_64RegClass);
    CCInfo.AllocateReg(FlatScratchInitReg);
  }

  // TODO: Add GridWorkGroupCount user SGPRs when used. For now with HSA we read
  // these from the dispatch pointer.
}

bool PPUCallLowering::lowerFormalArgumentsKernel(
    MachineIRBuilder &MIRBuilder, const Function &F,
    ArrayRef<ArrayRef<Register>> VRegs) const {
  MachineFunction &MF = MIRBuilder.getMF();
  const PPUSubtarget *Subtarget = &MF.getSubtarget<PPUSubtarget>();
  MachineRegisterInfo &MRI = MF.getRegInfo();
  PPUMachineFunctionInfo *Info = MF.getInfo<PPUMachineFunctionInfo>();
  const PPURegisterInfo *TRI = Subtarget->getRegisterInfo();
  const PPUTargetLowering &TLI = *getTLI<PPUTargetLowering>();

  const DataLayout &DL = F.getParent()->getDataLayout();

  SmallVector<CCValAssign, 16> ArgLocs;
  CCState CCInfo(F.getCallingConv(), F.isVarArg(), MF, ArgLocs, F.getContext());

  allocateHSAUserSGPRs(CCInfo, MIRBuilder, MF, *TRI, *Info);

  unsigned i = 0;
  const unsigned KernArgBaseAlign = 16;
  const unsigned BaseOffset = Subtarget->getExplicitKernelArgOffset(F);
  uint64_t ExplicitArgOffset = 0;

  // TODO: Align down to dword alignment and extract bits for extending loads.
  for (auto &Arg : F.args()) {
    Type *ArgTy = Arg.getType();
    unsigned AllocSize = DL.getTypeAllocSize(ArgTy);
    if (AllocSize == 0)
      continue;

    unsigned ABIAlign = DL.getABITypeAlignment(ArgTy);

    uint64_t ArgOffset = alignTo(ExplicitArgOffset, ABIAlign) + BaseOffset;
    ExplicitArgOffset = alignTo(ExplicitArgOffset, ABIAlign) + AllocSize;

    ArrayRef<Register> OrigArgRegs = VRegs[i];
    Register ArgReg =
      OrigArgRegs.size() == 1
      ? OrigArgRegs[0]
      : MRI.createGenericVirtualRegister(getLLTForType(*ArgTy, DL));
    unsigned Align = MinAlign(KernArgBaseAlign, ArgOffset);
    ArgOffset = alignTo(ArgOffset, DL.getABITypeAlignment(ArgTy));
    lowerParameter_compute(MIRBuilder, ArgTy, ArgOffset, Align, ArgReg);
    if (OrigArgRegs.size() > 1)
      unpackRegs(OrigArgRegs, ArgReg, ArgTy, MIRBuilder);
    ++i;
  }

  TLI.allocateSpecialEntryInputVGPRs(CCInfo, MF, *TRI, *Info);
  TLI.allocateSystemSGPRs(CCInfo, MF, *Info, F.getCallingConv(), false);
  return true;
}

// TODO: Move this to generic code
static void packSplitRegsToOrigType(MachineIRBuilder &MIRBuilder,
                                    ArrayRef<Register> OrigRegs,
                                    ArrayRef<Register> Regs,
                                    LLT LLTy,
                                    LLT PartLLT) {
  if (!LLTy.isVector() && !PartLLT.isVector()) {
    MIRBuilder.buildMerge(OrigRegs[0], Regs);
    return;
  }

  if (LLTy.isVector() && PartLLT.isVector()) {
    assert(LLTy.getElementType() == PartLLT.getElementType());

    int DstElts = LLTy.getNumElements();
    int PartElts = PartLLT.getNumElements();
    if (DstElts % PartElts == 0)
      MIRBuilder.buildConcatVectors(OrigRegs[0], Regs);
    else {
      // Deal with v3s16 split into v2s16
      assert(PartElts == 2 && DstElts % 2 != 0);
      int RoundedElts = PartElts * ((DstElts + PartElts - 1) / PartElts);

      LLT RoundedDestTy = LLT::vector(RoundedElts, PartLLT.getElementType());
      auto RoundedConcat = MIRBuilder.buildConcatVectors(RoundedDestTy, Regs);
      MIRBuilder.buildExtract(OrigRegs[0], RoundedConcat, 0);
    }

    return;
  }

  assert(LLTy.isVector() && !PartLLT.isVector());

  LLT DstEltTy = LLTy.getElementType();
  if (DstEltTy == PartLLT) {
    // Vector was trivially scalarized.
    MIRBuilder.buildBuildVector(OrigRegs[0], Regs);
  } else if (DstEltTy.getSizeInBits() > PartLLT.getSizeInBits()) {
    // Deal with vector with 64-bit elements decomposed to 32-bit
    // registers. Need to create intermediate 64-bit elements.
    SmallVector<Register, 8> EltMerges;
    int PartsPerElt = DstEltTy.getSizeInBits() / PartLLT.getSizeInBits();

    assert(DstEltTy.getSizeInBits() % PartLLT.getSizeInBits() == 0);

    for (int I = 0, NumElts = LLTy.getNumElements(); I != NumElts; ++I)  {
      auto Merge = MIRBuilder.buildMerge(DstEltTy,
                                         Regs.take_front(PartsPerElt));
      EltMerges.push_back(Merge.getReg(0));
      Regs = Regs.drop_front(PartsPerElt);
    }

    MIRBuilder.buildBuildVector(OrigRegs[0], EltMerges);
  } else {
    // Vector was split, and elements promoted to a wider type.
    LLT BVType = LLT::vector(LLTy.getNumElements(), PartLLT);
    auto BV = MIRBuilder.buildBuildVector(BVType, Regs);
    MIRBuilder.buildTrunc(OrigRegs[0], BV);
  }
}

bool PPUCallLowering::lowerFormalArguments_compute(
    MachineIRBuilder &MIRBuilder, const Function &F,
    ArrayRef<ArrayRef<Register>> VRegs) const {
  CallingConv::ID CC = F.getCallingConv();

  // The infrastructure for normal calling convention lowering is essentially
  // useless for kernels. We want to avoid any kind of legalization or argument
  // splitting.
  if (CC == CallingConv::AMDGPU_KERNEL)
    return lowerFormalArgumentsKernel(MIRBuilder, F, VRegs);

  const bool IsShader = !PPU::isKernel(CC);
  const bool IsEntryFunc = PPU::isEntryFunctionCC(CC);

  MachineFunction &MF = MIRBuilder.getMF();
  MachineBasicBlock &MBB = MIRBuilder.getMBB();
  MachineRegisterInfo &MRI = MF.getRegInfo();
  PPUMachineFunctionInfo *Info = MF.getInfo<PPUMachineFunctionInfo>();
  const PPUSubtarget &Subtarget = MF.getSubtarget<PPUSubtarget>();
  const PPURegisterInfo *TRI = Subtarget.getRegisterInfo();
  const DataLayout &DL = F.getParent()->getDataLayout();


  SmallVector<CCValAssign, 16> ArgLocs;
  CCState CCInfo(CC, F.isVarArg(), MF, ArgLocs, F.getContext());

  if (!IsEntryFunc) {
    Register ReturnAddrReg = TRI->getReturnAddressReg(MF);
    Register LiveInReturn = MF.addLiveIn(ReturnAddrReg,
                                         &PPU::SPR_64RegClass);
    MBB.addLiveIn(ReturnAddrReg);
    MIRBuilder.buildCopy(LiveInReturn, ReturnAddrReg);
  }

  if (Info->hasImplicitBufferPtr()) {
    Register ImplicitBufferPtrReg = Info->addImplicitBufferPtr(*TRI);
    MF.addLiveIn(ImplicitBufferPtrReg, &PPU::SPR_64RegClass);
    CCInfo.AllocateReg(ImplicitBufferPtrReg);
  }


  SmallVector<ArgInfo, 32> SplitArgs;
  unsigned Idx = 0;

  for (auto &Arg : F.args()) {
    if (DL.getTypeStoreSize(Arg.getType()) == 0)
      continue;

    const bool InReg = Arg.hasAttribute(Attribute::InReg);

    // SGPR arguments to functions not implemented.
    if (!IsShader && InReg)
      return false;

    if (Arg.hasAttribute(Attribute::SwiftSelf) ||
        Arg.hasAttribute(Attribute::SwiftError) ||
        Arg.hasAttribute(Attribute::Nest))
      return false;

    ArgInfo OrigArg(VRegs[Idx], Arg.getType());
    setArgFlags(OrigArg, Idx + AttributeList::FirstArgIndex, DL, F);

    splitToValueTypes(
      OrigArg, SplitArgs, DL, MRI, CC,
      // FIXME: We should probably be passing multiple registers to
      // handleAssignments to do this
      [&](ArrayRef<Register> Regs, LLT LLTy, LLT PartLLT, int VTSplitIdx) {
        packSplitRegsToOrigType(MIRBuilder, VRegs[Idx][VTSplitIdx], Regs,
                                LLTy, PartLLT);
      });

    ++Idx;
  }

  const PPUTargetLowering &TLI = *getTLI<PPUTargetLowering>();
  CCAssignFn *AssignFn = TLI.CCAssignFnForCall(CC, F.isVarArg());

  if (!MBB.empty())
    MIRBuilder.setInstr(*MBB.begin());

  FormalArgHandler Handler(MIRBuilder, MRI, AssignFn);
  if (!handleAssignments(CCInfo, ArgLocs, MIRBuilder, SplitArgs, Handler))
    return false;

  if (!IsEntryFunc) {
    // Special inputs come after user arguments.
    TLI.allocateSpecialInputVGPRs(CCInfo, MF, *TRI, *Info);
  }

  // Start adding system SGPRs.
  if (IsEntryFunc) {
    TLI.allocateSystemSGPRs(CCInfo, MF, *Info, CC, IsShader);
  } else {
    CCInfo.AllocateReg(Info->getScratchRSrcReg());
    CCInfo.AllocateReg(Info->getScratchWaveOffsetReg());
    CCInfo.AllocateReg(Info->getFrameOffsetReg());
    TLI.allocateSpecialInputSGPRs(CCInfo, MF, *TRI, *Info);
  }

  // Move back to the end of the basic block.
  MIRBuilder.setMBB(MBB);

  return true;
}

CCAssignFn *PPUCallLowering::CCAssignFnForCall(CallingConv::ID CC,
                                                  bool IsVarArg) {
  switch (CC) {
  case CallingConv::AMDGPU_CS:
    return CC_PPT;
  case CallingConv::C:
  case CallingConv::Fast:
  case CallingConv::Cold:
    return CC_PPT_Func;
  case CallingConv::AMDGPU_KERNEL:
  case CallingConv::SPIR_KERNEL:
  default:
    report_fatal_error("Unsupported calling convention for call");
  }
}

CCAssignFn *PPUCallLowering::CCAssignFnForReturn(CallingConv::ID CC,
                                                    bool IsVarArg) {
  switch (CC) {
  case CallingConv::AMDGPU_KERNEL:
  case CallingConv::SPIR_KERNEL:
    llvm_unreachable("kernels should not be handled here");
  case CallingConv::AMDGPU_CS:
    return RetCC_PPT_Compute;
  case CallingConv::C:
  case CallingConv::Fast:
  case CallingConv::Cold:
    return RetCC_PPT_Func;
  default:
    report_fatal_error("Unsupported calling convention.");
  }
}


