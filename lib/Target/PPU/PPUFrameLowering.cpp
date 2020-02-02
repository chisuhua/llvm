//===-- PPUFrameLowering.cpp - PPU Frame Information ------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the PPU implementation of TargetFrameLowering class.
//
//===----------------------------------------------------------------------===//

#include "PPUFrameLowering.h"
#include "PPUMachineFunctionInfo.h"
#include "PPUSubtarget.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/RegisterScavenging.h"
#include "llvm/MC/MCDwarf.h"

using namespace llvm;

#define DEBUG_TYPE "frame-info"

static ArrayRef<MCPhysReg> getAllSPR64(const PPUSubtarget &ST,
                                         const MachineFunction &MF) {
  return makeArrayRef(PPU::SPR_64RegClass.begin(),
                      ST.getMaxNumSGPRs(MF) / 2);
}

static ArrayRef<MCPhysReg> getAllSPR128(const PPUSubtarget &ST,
                                         const MachineFunction &MF) {
  return makeArrayRef(PPU::SReg_128RegClass.begin(),
                      ST.getMaxNumSGPRs(MF) / 4);

  // return makeArrayRef(PPU::SPR_64RegClass.begin(),
  //                    ST.getMaxNumSGPRs(MF) / 4);
}


static ArrayRef<MCPhysReg> getAllSPRs(const PPUSubtarget &ST,
                                       const MachineFunction &MF) {
  return makeArrayRef(PPU::SPR_32RegClass.begin(),
                      ST.getMaxNumSGPRs(MF));
}

// Find a scratch register that we can use at the start of the prologue to
// re-align the stack pointer. We avoid using callee-save registers since they
// may appear to be free when this is called from canUseAsPrologue (during
// shrink wrapping), but then no longer be free when this is called from
// emitPrologue.
//
// FIXME: This is a bit conservative, since in the above case we could use one
// of the callee-save registers as a scratch temp to re-align the stack pointer,
// but we would then have to make sure that we were in fact saving at least one
// callee-save register in the prologue, which is additional complexity that
// doesn't seem worth the benefit.
static unsigned findScratchNonCalleeSaveRegister(MachineRegisterInfo &MRI,
                                                 LivePhysRegs &LiveRegs,
                                                 const TargetRegisterClass &RC,
                                                 bool Unused = false) {
  // Mark callee saved registers as used so we will not choose them.
  const MCPhysReg *CSRegs = MRI.getCalleeSavedRegs();
  for (unsigned i = 0; CSRegs[i]; ++i)
    LiveRegs.addReg(CSRegs[i]);

  if (Unused) {
    // We are looking for a register that can be used throughout the entire
    // function, so any use is unacceptable.
    for (unsigned Reg : RC) {
      if (!MRI.isPhysRegUsed(Reg) && LiveRegs.available(MRI, Reg))
        return Reg;
    }
  } else {
    for (unsigned Reg : RC) {
      if (LiveRegs.available(MRI, Reg))
        return Reg;
    }
  }

  // If we require an unused register, this is used in contexts where failure is
  // an option and has an alternative plan. In other contexts, this must
  // succeed0.
  if (!Unused)
    report_fatal_error("failed to find free scratch register");

  return PPU::NoRegister;
}

static MCPhysReg findUnusedSGPRNonCalleeSaved(MachineRegisterInfo &MRI) {
  LivePhysRegs LiveRegs;
  LiveRegs.init(*MRI.getTargetRegisterInfo());
  return findScratchNonCalleeSaveRegister(
    MRI, LiveRegs, PPU::SReg_32RegClass, true);
}

// We need to specially emit stack operations here because a different frame
// register is used than in the rest of the function, as getFrameRegister would
// use.
static void buildPrologSpill(LivePhysRegs &LiveRegs, MachineBasicBlock &MBB,
                             MachineBasicBlock::iterator I,
                             const PPUInstrInfo *TII, unsigned SpillReg,
                             unsigned ScratchRsrcReg, unsigned SPReg, int FI) {
  MachineFunction *MF = MBB.getParent();
  MachineFrameInfo &MFI = MF->getFrameInfo();

  int64_t Offset = MFI.getObjectOffset(FI);

  MachineMemOperand *MMO = MF->getMachineMemOperand(
      MachinePointerInfo::getFixedStack(*MF, FI), MachineMemOperand::MOStore, 4,
      MFI.getObjectAlignment(FI));

  if (isUInt<12>(Offset)) {
    BuildMI(MBB, I, DebugLoc(), TII->get(PPU::BUFFER_STORE_DWORD_OFFSET))
      .addReg(SpillReg, RegState::Kill)
      .addReg(ScratchRsrcReg)
      .addReg(SPReg)
      .addImm(Offset)
      .addImm(0) // glc
      .addImm(0) // slc
      .addImm(0) // tfe
      .addImm(0) // dlc
      .addMemOperand(MMO);
    return;
  }

  MCPhysReg OffsetReg = findScratchNonCalleeSaveRegister(
    MF->getRegInfo(), LiveRegs, PPU::VPR_32RegClass);

  BuildMI(MBB, I, DebugLoc(), TII->get(PPU::V_MOV_B32_e32), OffsetReg)
    .addImm(Offset);

  BuildMI(MBB, I, DebugLoc(), TII->get(PPU::BUFFER_STORE_DWORD_OFFEN))
    .addReg(SpillReg, RegState::Kill)
    .addReg(OffsetReg, RegState::Kill)
    .addReg(ScratchRsrcReg)
    .addReg(SPReg)
    .addImm(0)
    .addImm(0) // glc
    .addImm(0) // slc
    .addImm(0) // tfe
    .addImm(0) // dlc
    .addMemOperand(MMO);
}

static void buildEpilogReload(LivePhysRegs &LiveRegs, MachineBasicBlock &MBB,
                              MachineBasicBlock::iterator I,
                              const PPUInstrInfo *TII, unsigned SpillReg,
                              unsigned ScratchRsrcReg, unsigned SPReg, int FI) {
  MachineFunction *MF = MBB.getParent();
  MachineFrameInfo &MFI = MF->getFrameInfo();
  int64_t Offset = MFI.getObjectOffset(FI);

  MachineMemOperand *MMO = MF->getMachineMemOperand(
      MachinePointerInfo::getFixedStack(*MF, FI), MachineMemOperand::MOLoad, 4,
      MFI.getObjectAlignment(FI));

  if (isUInt<12>(Offset)) {
    BuildMI(MBB, I, DebugLoc(),
            TII->get(PPU::BUFFER_LOAD_DWORD_OFFSET), SpillReg)
      .addReg(ScratchRsrcReg)
      .addReg(SPReg)
      .addImm(Offset)
      .addImm(0) // glc
      .addImm(0) // slc
      .addImm(0) // tfe
      .addImm(0) // dlc
      .addMemOperand(MMO);
    return;
  }

  MCPhysReg OffsetReg = findScratchNonCalleeSaveRegister(
    MF->getRegInfo(), LiveRegs, PPU::VPR_32RegClass);

  BuildMI(MBB, I, DebugLoc(), TII->get(PPU::V_MOV_B32_e32), OffsetReg)
    .addImm(Offset);

  BuildMI(MBB, I, DebugLoc(),
          TII->get(PPU::BUFFER_LOAD_DWORD_OFFEN), SpillReg)
    .addReg(OffsetReg, RegState::Kill)
    .addReg(ScratchRsrcReg)
    .addReg(SPReg)
    .addImm(0)
    .addImm(0) // glc
    .addImm(0) // slc
    .addImm(0) // tfe
    .addImm(0) // dlc
    .addMemOperand(MMO);
}

bool PPUFrameLowering::hasFP_compute(const MachineFunction &MF) const {
  const MachineFrameInfo &MFI = MF.getFrameInfo();
  if (MFI.hasCalls()) {
    // All offsets are unsigned, so need to be addressed in the same direction
    // as stack growth.

    // FIXME: This function is pretty broken, since it can be called before the
    // frame layout is determined or CSR spills are inserted.
    if (MFI.getStackSize() != 0)
      return true;

    // For the entry point, the input wave scratch offset must be copied to the
    // API SP if there are calls.
    if (MF.getInfo<PPUMachineFunctionInfo>()->isEntryFunction())
      return true;
  }

  return MFI.hasVarSizedObjects() || MFI.isFrameAddressTaken() ||
    MFI.hasStackMap() || MFI.hasPatchPoint() ||
    MF.getSubtarget<PPUSubtarget>().getRegisterInfo()->needsStackRealignment(MF) ||
    MF.getTarget().Options.DisableFramePointerElim(MF);
}
/*
bool PPUFrameLowering::hasSP_compute(const MachineFunction &MF) const {
  const PPURegisterInfo *TRI = MF.getSubtarget<PPUSubtarget>().getRegisterInfo();
  // All stack operations are relative to the frame offset SGPR.
  const MachineFrameInfo &MFI = MF.getFrameInfo();
  return MFI.hasCalls() || MFI.hasVarSizedObjects() ||
    TRI->needsStackRealignment(MF);
}
*/

bool PPUFrameLowering::hasFP(const MachineFunction &MF) const {
  if (PPU::isCompute(&MF))
      return hasFP_compute(MF);

  const TargetRegisterInfo *RegInfo = MF.getSubtarget().getRegisterInfo();

  const MachineFrameInfo &MFI = MF.getFrameInfo();
  return MF.getTarget().Options.DisableFramePointerElim(MF) ||
         RegInfo->needsStackRealignment(MF) || MFI.hasVarSizedObjects() ||
         MFI.isFrameAddressTaken();
}



// Determines the size of the frame and maximum call frame size.
void PPUFrameLowering::determineFrameLayout(MachineFunction &MF) const {
  MachineFrameInfo &MFI = MF.getFrameInfo();
  const PPURegisterInfo *RI = STI.getRegisterInfo();

  // Get the number of bytes to allocate from the FrameInfo.
  uint64_t FrameSize = MFI.getStackSize();

  // Get the alignment.
  unsigned StackAlign = getStackAlignment();
  if (RI->needsStackRealignment(MF)) {
    unsigned MaxStackAlign = std::max(StackAlign, MFI.getMaxAlignment());
    FrameSize += (MaxStackAlign - StackAlign);
    StackAlign = MaxStackAlign;
  }

  // Set Max Call Frame Size
  uint64_t MaxCallSize = alignTo(MFI.getMaxCallFrameSize(), StackAlign);
  MFI.setMaxCallFrameSize(MaxCallSize);

  // Make sure the frame is aligned.
  FrameSize = alignTo(FrameSize, StackAlign);

  // Update frame info.
  MFI.setStackSize(FrameSize);
}

// TODO schi copied from rvv
bool PPUFrameLowering::shouldEnableVectorUnit(MachineFunction &MF) const {
  auto &Subtarget = MF.getSubtarget<PPUSubtarget>();
  if (!Subtarget.hasStdExtV())
    return false;
  return true;
}

void PPUFrameLowering::adjustReg(MachineBasicBlock &MBB,
                                   MachineBasicBlock::iterator MBBI,
                                   const DebugLoc &DL, Register DestReg,
                                   Register SrcReg, int64_t Val,
                                   MachineInstr::MIFlag Flag) const {
  MachineRegisterInfo &MRI = MBB.getParent()->getRegInfo();
  const PPUInstrInfo *TII = STI.getInstrInfo();

  if (DestReg == SrcReg && Val == 0)
    return;

  if (isInt<12>(Val)) {
    BuildMI(MBB, MBBI, DL, TII->get(PPU::ADDI), DestReg)
        .addReg(SrcReg)
        .addImm(Val)
        .setMIFlag(Flag);
  } else if (isInt<32>(Val)) {
    unsigned Opc = PPU::ADD;
    bool isSub = Val < 0;
    if (isSub) {
      Val = -Val;
      Opc = PPU::SUB;
    }

    Register ScratchReg = MRI.createVirtualRegister(&PPU::GPRRegClass);
    TII->movImm32(MBB, MBBI, DL, ScratchReg, Val, Flag);
    BuildMI(MBB, MBBI, DL, TII->get(Opc), DestReg)
        .addReg(SrcReg)
        .addReg(ScratchReg, RegState::Kill)
        .setMIFlag(Flag);
  } else {
    report_fatal_error("adjustReg cannot yet handle adjustments >32 bits");
  }
}

/*
 move to MFI->getStackPtrOffsetReg and MFI->getFrameOffsetReg
StackPtrOffsetReg is PPU::FP_REG 
FrameOffsetReg is PPU::SP_REG
*/

// Returns the register used to hold the frame pointer.
static Register getFPReg(const PPUSubtarget &STI) { return PPU::X8; }

// Returns the register used to hold the stack pointer.
static Register getSPReg(const PPUSubtarget &STI) { return PPU::X2; }

void PPUFrameLowering::emitPrologue_compute(MachineFunction &MF,
                                   MachineBasicBlock &MBB) const {
  PPUMachineFunctionInfo *FuncInfo = MF.getInfo<PPUMachineFunctionInfo>();
  if (FuncInfo->isEntryFunction()) {
    emitEntryFunctionPrologue(MF, MBB);
    return;
  }

  const MachineFrameInfo &MFI = MF.getFrameInfo();
  MachineRegisterInfo &MRI = MF.getRegInfo();
  const PPUSubtarget &ST = MF.getSubtarget<PPUSubtarget>();
  const PPUInstrInfo *TII = ST.getInstrInfo();
  const PPURegisterInfo &TRI = TII->getRegisterInfo();

  unsigned StackPtrReg = FuncInfo->getStackPtrOffsetReg();
  unsigned FramePtrReg = FuncInfo->getFrameOffsetReg();
  LivePhysRegs LiveRegs;

  MachineBasicBlock::iterator MBBI = MBB.begin();
  DebugLoc DL;

  bool HasFP = false;
  uint32_t NumBytes = MFI.getStackSize();
  uint32_t RoundedSize = NumBytes;

  // To avoid clobbering VGPRs in lanes that weren't active on function entry,
  // turn on all lanes before doing the spill to memory.
  unsigned ScratchExecCopy = PPU::NoRegister;

  // Emit the copy if we need an FP, and are using a free SGPR to save it.
  if (FuncInfo->SGPRForFPSaveRestoreCopy != PPU::NoRegister) {
    BuildMI(MBB, MBBI, DL, TII->get(PPU::COPY), FuncInfo->SGPRForFPSaveRestoreCopy)
      .addReg(FramePtrReg)
      .setMIFlag(MachineInstr::FrameSetup);
  }

  for (const PPUMachineFunctionInfo::SGPRSpillVGPRCSR &Reg
         : FuncInfo->getSGPRSpillVGPRs()) {
    if (!Reg.FI.hasValue())
      continue;

    if (ScratchExecCopy == PPU::NoRegister) {
      if (LiveRegs.empty()) {
        LiveRegs.init(TRI);
        LiveRegs.addLiveIns(MBB);
        if (FuncInfo->SGPRForFPSaveRestoreCopy)
          LiveRegs.removeReg(FuncInfo->SGPRForFPSaveRestoreCopy);
      }

      ScratchExecCopy
        = findScratchNonCalleeSaveRegister(MRI, LiveRegs,
                                           *TRI.getWaveMaskRegClass());
      assert(FuncInfo->SGPRForFPSaveRestoreCopy != ScratchExecCopy);

      const unsigned OrSaveExec = PPU::S_OR_SAVETMSK_B32;
      BuildMI(MBB, MBBI, DL, TII->get(OrSaveExec),
              ScratchExecCopy)
        .addImm(-1);
    }

    buildPrologSpill(LiveRegs, MBB, MBBI, TII, Reg.VGPR,
                     FuncInfo->getScratchRSrcReg(),
                     StackPtrReg,
                     Reg.FI.getValue());
  }

  if (ScratchExecCopy != PPU::NoRegister) {
    // FIXME: Split block and make terminator.
    unsigned ExecMov = PPU::S_MOV_B32;
    unsigned Exec = PPU::TMSK;
    BuildMI(MBB, MBBI, DL, TII->get(ExecMov), Exec)
      .addReg(ScratchExecCopy, RegState::Kill);
    LiveRegs.addReg(ScratchExecCopy);
  }

  if (FuncInfo->FramePointerSaveIndex) {
    const int FI = FuncInfo->FramePointerSaveIndex.getValue();
    assert(!MFI.isDeadObjectIndex(FI) &&
           MFI.getStackID(FI) == TargetStackID::SGPRSpill);
    ArrayRef<PPUMachineFunctionInfo::SpilledReg> Spill
      = FuncInfo->getSGPRToVGPRSpills(FI);
    assert(Spill.size() == 1);

    // Save FP before setting it up.
    // FIXME: This should respect spillSGPRToVGPR;
    BuildMI(MBB, MBBI, DL, TII->getMCOpcodeFromPseudo(PPU::V_WRITELANE_B32),
            Spill[0].VGPR)
      .addReg(FramePtrReg)
      .addImm(Spill[0].Lane)
      .addReg(Spill[0].VGPR, RegState::Undef);
  }

  if (TRI.needsStackRealignment(MF)) {
    HasFP = true;
    const unsigned Alignment = MFI.getMaxAlignment();

    RoundedSize += Alignment;
    if (LiveRegs.empty()) {
      LiveRegs.init(TRI);
      LiveRegs.addLiveIns(MBB);
      LiveRegs.addReg(FuncInfo->SGPRForFPSaveRestoreCopy);
    }

    unsigned ScratchSPReg = findScratchNonCalleeSaveRegister(
        MRI, LiveRegs, PPU::SReg_32RegClass);
    assert(ScratchSPReg != PPU::NoRegister &&
           ScratchSPReg != FuncInfo->SGPRForFPSaveRestoreCopy);

    // s_add_u32 tmp_reg, s32, NumBytes
    // s_and_b32 s32, tmp_reg, 0b111...0000
    BuildMI(MBB, MBBI, DL, TII->get(PPU::S_ADD_U32), ScratchSPReg)
      .addReg(StackPtrReg)
      .addImm((Alignment - 1) * ST.getWavefrontSize())
      .setMIFlag(MachineInstr::FrameSetup);
    BuildMI(MBB, MBBI, DL, TII->get(PPU::S_AND_B32), FramePtrReg)
      .addReg(ScratchSPReg, RegState::Kill)
      .addImm(-Alignment * ST.getWavefrontSize())
      .setMIFlag(MachineInstr::FrameSetup);
    FuncInfo->setIsStackRealigned(true);
  } else if ((HasFP = hasFP(MF))) {
    // If we need a base pointer, set it up here. It's whatever the value of
    // the stack pointer is at this point. Any variable size objects will be
    // allocated after this, so we can still use the base pointer to reference
    // locals.
    BuildMI(MBB, MBBI, DL, TII->get(PPU::COPY), FramePtrReg)
      .addReg(StackPtrReg)
      .setMIFlag(MachineInstr::FrameSetup);
  }

  if (HasFP && RoundedSize != 0) {
    BuildMI(MBB, MBBI, DL, TII->get(PPU::S_ADD_U32), StackPtrReg)
      .addReg(StackPtrReg)
      .addImm(RoundedSize * ST.getWavefrontSize())
      .setMIFlag(MachineInstr::FrameSetup);
  }

  assert((!HasFP || (FuncInfo->SGPRForFPSaveRestoreCopy != PPU::NoRegister ||
                     FuncInfo->FramePointerSaveIndex)) &&
         "Needed to save FP but didn't save it anywhere");

  assert((HasFP || (FuncInfo->SGPRForFPSaveRestoreCopy == PPU::NoRegister &&
                    !FuncInfo->FramePointerSaveIndex)) &&
         "Saved FP but didn't need it");

}

void PPUFrameLowering::emitPrologue(MachineFunction &MF,
                                      MachineBasicBlock &MBB) const {
  if (PPU::isCompute(&MF))
      return emitPrologue_compute(MF, MBB);

  assert(&MF.front() == &MBB && "Shrink-wrapping not yet supported");

  MachineFrameInfo &MFI = MF.getFrameInfo();
  auto *RVFI = MF.getInfo<PPUMachineFunctionInfo>();
  const PPURegisterInfo *RI = STI.getRegisterInfo();
  const PPUInstrInfo *TII = STI.getInstrInfo();
  MachineBasicBlock::iterator MBBI = MBB.begin();

  if (RI->needsStackRealignment(MF) && MFI.hasVarSizedObjects()) {
    report_fatal_error(
        "RISC-V backend can't currently handle functions that need stack "
        "realignment and have variable sized objects");
  }

  Register FPReg = getFPReg(STI);
  Register SPReg = getSPReg(STI);

  // Debug location must be unknown since the first debug location is used
  // to determine the end of the prologue.
  DebugLoc DL;

  // Determine the correct frame layout
  determineFrameLayout(MF);

  // FIXME (note copied from Lanai): This appears to be overallocating.  Needs
  // investigation. Get the number of bytes to allocate from the FrameInfo.
  uint64_t StackSize = MFI.getStackSize();

  // Early exit if there is no need to allocate on the stack
  if (StackSize == 0 && !MFI.adjustsStack())
    return;

  // Allocate space on the stack if necessary.
  adjustReg(MBB, MBBI, DL, SPReg, SPReg, -StackSize, MachineInstr::FrameSetup);

  // Emit ".cfi_def_cfa_offset StackSize"
  unsigned CFIIndex = MF.addFrameInst(
      MCCFIInstruction::createDefCfaOffset(nullptr, -StackSize));
  BuildMI(MBB, MBBI, DL, TII->get(TargetOpcode::CFI_INSTRUCTION))
      .addCFIIndex(CFIIndex);

  // The frame pointer is callee-saved, and code has been generated for us to
  // save it to the stack. We need to skip over the storing of callee-saved
  // registers as the frame pointer must be modified after it has been saved
  // to the stack, not before.
  // FIXME: assumes exactly one instruction is used to save each callee-saved
  // register.
  const std::vector<CalleeSavedInfo> &CSI = MFI.getCalleeSavedInfo();
  std::advance(MBBI, CSI.size());

  // Iterate over list of callee-saved registers and emit .cfi_offset
  // directives.
  for (const auto &Entry : CSI) {
    int64_t Offset = MFI.getObjectOffset(Entry.getFrameIdx());
    Register Reg = Entry.getReg();
    unsigned CFIIndex = MF.addFrameInst(MCCFIInstruction::createOffset(
        nullptr, RI->getDwarfRegNum(Reg, true), Offset));
    BuildMI(MBB, MBBI, DL, TII->get(TargetOpcode::CFI_INSTRUCTION))
        .addCFIIndex(CFIIndex);
  }

  // Generate new FP.
  if (hasFP(MF)) {
    adjustReg(MBB, MBBI, DL, FPReg, SPReg,
              StackSize - RVFI->getVarArgsSaveSize(), MachineInstr::FrameSetup);

    // Emit ".cfi_def_cfa $fp, 0"
    unsigned CFIIndex = MF.addFrameInst(MCCFIInstruction::createDefCfa(
        nullptr, RI->getDwarfRegNum(FPReg, true), 0));
    BuildMI(MBB, MBBI, DL, TII->get(TargetOpcode::CFI_INSTRUCTION))
        .addCFIIndex(CFIIndex);

    // Realign Stack
    const PPURegisterInfo *RI = STI.getRegisterInfo();
    if (RI->needsStackRealignment(MF)) {
      unsigned MaxAlignment = MFI.getMaxAlignment();

      const PPUInstrInfo *TII = STI.getInstrInfo();
      if (isInt<12>(-(int)MaxAlignment)) {
        BuildMI(MBB, MBBI, DL, TII->get(PPU::ANDI), SPReg)
            .addReg(SPReg)
            .addImm(-(int)MaxAlignment);
      } else {
        unsigned ShiftAmount = countTrailingZeros(MaxAlignment);
        Register VR =
            MF.getRegInfo().createVirtualRegister(&PPU::GPRRegClass);
        BuildMI(MBB, MBBI, DL, TII->get(PPU::SRLI), VR)
            .addReg(SPReg)
            .addImm(ShiftAmount);
        BuildMI(MBB, MBBI, DL, TII->get(PPU::SLLI), SPReg)
            .addReg(VR)
            .addImm(ShiftAmount);
      }
    }
  }
}

void PPUFrameLowering::emitEpilogue_compute(MachineFunction &MF,
                                   MachineBasicBlock &MBB) const {
  const PPUMachineFunctionInfo *FuncInfo = MF.getInfo<PPUMachineFunctionInfo>();
  if (FuncInfo->isEntryFunction())
    return;

  const PPUSubtarget &ST = MF.getSubtarget<PPUSubtarget>();
  const PPUInstrInfo *TII = ST.getInstrInfo();
  MachineRegisterInfo &MRI = MF.getRegInfo();
  MachineBasicBlock::iterator MBBI = MBB.getFirstTerminator();
  LivePhysRegs LiveRegs;
  DebugLoc DL;

  const MachineFrameInfo &MFI = MF.getFrameInfo();
  uint32_t NumBytes = MFI.getStackSize();
  uint32_t RoundedSize = FuncInfo->isStackRealigned() ?
    NumBytes + MFI.getMaxAlignment() : NumBytes;

  if (RoundedSize != 0 && hasFP(MF)) {
    const unsigned StackPtrReg = FuncInfo->getStackPtrOffsetReg();
    BuildMI(MBB, MBBI, DL, TII->get(PPU::S_SUB_U32), StackPtrReg)
      .addReg(StackPtrReg)
      .addImm(RoundedSize * ST.getWavefrontSize())
      .setMIFlag(MachineInstr::FrameDestroy);
  }

  if (FuncInfo->SGPRForFPSaveRestoreCopy != PPU::NoRegister) {
    BuildMI(MBB, MBBI, DL, TII->get(PPU::COPY), FuncInfo->getFrameOffsetReg())
      .addReg(FuncInfo->SGPRForFPSaveRestoreCopy)
      .setMIFlag(MachineInstr::FrameSetup);
  }

  if (FuncInfo->FramePointerSaveIndex) {
    const int FI = FuncInfo->FramePointerSaveIndex.getValue();

    assert(!MF.getFrameInfo().isDeadObjectIndex(FI) &&
           MF.getFrameInfo().getStackID(FI) == TargetStackID::SGPRSpill);

    ArrayRef<PPUMachineFunctionInfo::SpilledReg> Spill
      = FuncInfo->getSGPRToVGPRSpills(FI);
    assert(Spill.size() == 1);
    BuildMI(MBB, MBBI, DL, TII->getMCOpcodeFromPseudo(PPU::V_READLANE_B32),
            FuncInfo->getFrameOffsetReg())
      .addReg(Spill[0].VGPR)
      .addImm(Spill[0].Lane);
  }


  unsigned ScratchExecCopy = PPU::NoRegister;
  for (const PPUMachineFunctionInfo::SGPRSpillVGPRCSR &Reg
         : FuncInfo->getSGPRSpillVGPRs()) {
    if (!Reg.FI.hasValue())
      continue;

    const PPURegisterInfo &TRI = TII->getRegisterInfo();
    if (ScratchExecCopy == PPU::NoRegister) {
      // See emitPrologue
      if (LiveRegs.empty()) {
        LiveRegs.init(*ST.getRegisterInfo());
        LiveRegs.addLiveOuts(MBB);
        LiveRegs.stepBackward(*MBBI);
      }

      ScratchExecCopy = findScratchNonCalleeSaveRegister(
          MRI, LiveRegs, *TRI.getWaveMaskRegClass());
      LiveRegs.removeReg(ScratchExecCopy);

      const unsigned OrSaveExec = PPU::S_OR_SAVETMSK_B32;

      BuildMI(MBB, MBBI, DL, TII->get(OrSaveExec), ScratchExecCopy)
        .addImm(-1);
    }

    buildEpilogReload(LiveRegs, MBB, MBBI, TII, Reg.VGPR,
                      FuncInfo->getScratchRSrcReg(),
                      FuncInfo->getStackPtrOffsetReg(), Reg.FI.getValue());
  }

  if (ScratchExecCopy != PPU::NoRegister) {
    // FIXME: Split block and make terminator.
    unsigned ExecMov = PPU::S_MOV_B32;
    unsigned Exec = PPU::TMSK;
    BuildMI(MBB, MBBI, DL, TII->get(ExecMov), Exec)
      .addReg(ScratchExecCopy, RegState::Kill);
  }
}

void PPUFrameLowering::emitEpilogue(MachineFunction &MF,
                                      MachineBasicBlock &MBB) const {
  if (PPU::isCompute(&MF))
      return emitEpilogue_compute(MF, MBB);

  MachineBasicBlock::iterator MBBI = MBB.getLastNonDebugInstr();
  const PPURegisterInfo *RI = STI.getRegisterInfo();
  MachineFrameInfo &MFI = MF.getFrameInfo();
  auto *RVFI = MF.getInfo<PPUMachineFunctionInfo>();
  DebugLoc DL = MBBI->getDebugLoc();
  const PPUInstrInfo *TII = STI.getInstrInfo();

  Register FPReg = getFPReg(STI);
  Register SPReg = getSPReg(STI);

  // Skip to before the restores of callee-saved registers
  // FIXME: assumes exactly one instruction is used to restore each
  // callee-saved register.
  auto LastFrameDestroy = std::prev(MBBI, MFI.getCalleeSavedInfo().size());

  uint64_t StackSize = MFI.getStackSize();
  uint64_t FPOffset = StackSize - RVFI->getVarArgsSaveSize();

  // Restore the stack pointer using the value of the frame pointer. Only
  // necessary if the stack pointer was modified, meaning the stack size is
  // unknown.
  if (RI->needsStackRealignment(MF) || MFI.hasVarSizedObjects()) {
    assert(hasFP(MF) && "frame pointer should not have been eliminated");
    adjustReg(MBB, LastFrameDestroy, DL, SPReg, FPReg, -FPOffset,
              MachineInstr::FrameDestroy);
  }

  if (hasFP(MF)) {
    // To find the instruction restoring FP from stack.
    for (auto &I = LastFrameDestroy; I != MBBI; ++I) {
      if (I->mayLoad() && I->getOperand(0).isReg()) {
        Register DestReg = I->getOperand(0).getReg();
        if (DestReg == FPReg) {
          // If there is frame pointer, after restoring $fp registers, we
          // need adjust CFA to ($sp - FPOffset).
          // Emit ".cfi_def_cfa $sp, -FPOffset"
          unsigned CFIIndex = MF.addFrameInst(MCCFIInstruction::createDefCfa(
              nullptr, RI->getDwarfRegNum(SPReg, true), -FPOffset));
          BuildMI(MBB, std::next(I), DL,
                  TII->get(TargetOpcode::CFI_INSTRUCTION))
              .addCFIIndex(CFIIndex);
          break;
        }
      }
    }
  }

  // Add CFI directives for callee-saved registers.
  const std::vector<CalleeSavedInfo> &CSI = MFI.getCalleeSavedInfo();
  // Iterate over list of callee-saved registers and emit .cfi_restore
  // directives.
  for (const auto &Entry : CSI) {
    Register Reg = Entry.getReg();
    unsigned CFIIndex = MF.addFrameInst(MCCFIInstruction::createRestore(
        nullptr, RI->getDwarfRegNum(Reg, true)));
    BuildMI(MBB, MBBI, DL, TII->get(TargetOpcode::CFI_INSTRUCTION))
        .addCFIIndex(CFIIndex);
  }

  // Deallocate stack
  adjustReg(MBB, MBBI, DL, SPReg, SPReg, StackSize, MachineInstr::FrameDestroy);

  if (shouldEnableVectorUnit(MF)) {
    BuildMI(MBB, MBBI, DL, STI.getInstrInfo()->get(PPU::VCONFIG))
        // vdisable = vconfig 0x01
        .addImm(0x01);
  }

  // After restoring $sp, we need to adjust CFA to $(sp + 0)
  // Emit ".cfi_def_cfa_offset 0"
  unsigned CFIIndex =
      MF.addFrameInst(MCCFIInstruction::createDefCfaOffset(nullptr, 0));
  BuildMI(MBB, MBBI, DL, TII->get(TargetOpcode::CFI_INSTRUCTION))
      .addCFIIndex(CFIIndex);
}

static bool allStackObjectsAreDead(const MachineFrameInfo &MFI) {
  for (int I = MFI.getObjectIndexBegin(), E = MFI.getObjectIndexEnd(); I != E;
    ++I) {
    if (!MFI.isDeadObjectIndex(I))
      return false;
  }

  return true;
}

#ifndef NDEBUG
static bool allSGPRSpillsAreDead(const MachineFrameInfo &MFI,
                                 Optional<int> FramePointerSaveIndex) {
  for (int I = MFI.getObjectIndexBegin(), E = MFI.getObjectIndexEnd();
       I != E; ++I) {
    if (!MFI.isDeadObjectIndex(I) &&
        MFI.getStackID(I) == TargetStackID::SGPRSpill &&
        FramePointerSaveIndex && I != FramePointerSaveIndex) {
      return false;
    }
  }

  return true;
}
#endif


int PPUFrameLowering::getFrameIndexReference_compute(const MachineFunction &MF, int FI,
                                            unsigned &FrameReg) const {
  const PPURegisterInfo *RI = MF.getSubtarget<PPUSubtarget>().getRegisterInfo();

  FrameReg = RI->getFrameRegister(MF);
  return MF.getFrameInfo().getObjectOffset(FI);
}


int PPUFrameLowering::getFrameIndexReference(const MachineFunction &MF,
                                               int FI,
                                               unsigned &FrameReg) const {
  if (PPU::isCompute(&MF))
      return getFrameIndexReference_compute(MF, FI, FrameReg);

  const MachineFrameInfo &MFI = MF.getFrameInfo();
  const TargetRegisterInfo *RI = MF.getSubtarget().getRegisterInfo();
  const auto *RVFI = MF.getInfo<PPUMachineFunctionInfo>();

  // Callee-saved registers should be referenced relative to the stack
  // pointer (positive offset), otherwise use the frame pointer (negative
  // offset).
  const std::vector<CalleeSavedInfo> &CSI = MFI.getCalleeSavedInfo();
  int MinCSFI = 0;
  int MaxCSFI = -1;

  int Offset = MFI.getObjectOffset(FI) - getOffsetOfLocalArea() +
               MFI.getOffsetAdjustment();

  if (CSI.size()) {
    MinCSFI = CSI[0].getFrameIdx();
    MaxCSFI = CSI[CSI.size() - 1].getFrameIdx();
  }

  if (FI >= MinCSFI && FI <= MaxCSFI) {
    FrameReg = PPU::X2;
    Offset += MF.getFrameInfo().getStackSize();
  } else if (RI->needsStackRealignment(MF)) {
    assert(!MFI.hasVarSizedObjects() &&
           "Unexpected combination of stack realignment and varsized objects");
    // If the stack was realigned, the frame pointer is set in order to allow
    // SP to be restored, but we still access stack objects using SP.
    FrameReg = PPU::X2;
    Offset += MF.getFrameInfo().getStackSize();
  } else {
    FrameReg = RI->getFrameRegister(MF);
    if (hasFP(MF))
      Offset += RVFI->getVarArgsSaveSize();
    else
      Offset += MF.getFrameInfo().getStackSize();
  }
  return Offset;
}

void PPUFrameLowering::processFunctionBeforeFrameFinalized_compute(
  MachineFunction &MF,
  RegScavenger *RS) const {
  MachineFrameInfo &MFI = MF.getFrameInfo();

  const PPUSubtarget &ST = MF.getSubtarget<PPUSubtarget>();
  const PPURegisterInfo *TRI = ST.getRegisterInfo();
  PPUMachineFunctionInfo *FuncInfo = MF.getInfo<PPUMachineFunctionInfo>();

  FuncInfo->removeDeadFrameIndices(MFI);
  assert(allSGPRSpillsAreDead(MFI, None) &&
         "SGPR spill should have been removed in SILowerSGPRSpills");

  // FIXME: The other checks should be redundant with allStackObjectsAreDead,
  // but currently hasNonSpillStackObjects is set only from source
  // allocas. Stack temps produced from legalization are not counted currently.
  if (!allStackObjectsAreDead(MFI)) {
    assert(RS && "RegScavenger required if spilling");

    if (FuncInfo->isEntryFunction()) {
      int ScavengeFI = MFI.CreateFixedObject(
        TRI->getSpillSize(PPU::SPR_32RegClass), 0, false);
      RS->addScavengingFrameIndex(ScavengeFI);
    } else {
      int ScavengeFI = MFI.CreateStackObject(
        TRI->getSpillSize(PPU::SPR_32RegClass),
        TRI->getSpillAlignment(PPU::SPR_32RegClass),
        false);
      RS->addScavengingFrameIndex(ScavengeFI);
    }
  }
}

void PPUFrameLowering::processFunctionBeforeFrameFinalized(
    MachineFunction &MF, RegScavenger *RS) const {

  if (PPU::isCompute(&MF))
      return processFunctionBeforeFrameFinalized_compute(MF, RS);

  const TargetRegisterInfo *RegInfo = MF.getSubtarget().getRegisterInfo();
  MachineFrameInfo &MFI = MF.getFrameInfo();
  const TargetRegisterClass *RC = &PPU::GPRRegClass;
  // estimateStackSize has been observed to under-estimate the final stack
  // size, so give ourselves wiggle-room by checking for stack size
  // representable an 11-bit signed field rather than 12-bits.
  // FIXME: It may be possible to craft a function with a small stack that
  // still needs an emergency spill slot for branch relaxation. This case
  // would currently be missed.
  if (!isInt<11>(MFI.estimateStackSize(MF))) {
    int RegScavFI = MFI.CreateStackObject(
        RegInfo->getSpillSize(*RC), RegInfo->getSpillAlignment(*RC), false);
    RS->addScavengingFrameIndex(RegScavFI);
  }
}


// Only report VGPRs to generic code.
void PPUFrameLowering::determineCalleeSaves_compute(MachineFunction &MF,
                                           BitVector &SavedVGPRs,
                                           RegScavenger *RS) const {
  TargetFrameLowering::determineCalleeSaves(MF, SavedVGPRs, RS);
  PPUMachineFunctionInfo *MFI = MF.getInfo<PPUMachineFunctionInfo>();

  if (MFI->isEntryFunction())
    return;

  const MachineFrameInfo &FrameInfo = MF.getFrameInfo();
  const PPUSubtarget &ST = MF.getSubtarget<PPUSubtarget>();
  const PPURegisterInfo *TRI = ST.getRegisterInfo();

  // Ignore the SGPRs the default implementation found.
  SavedVGPRs.clearBitsNotInMask(TRI->getAllVGPRRegMask());

  // hasFP only knows about stack objects that already exist. We're now
  // determining the stack slots that will be created, so we have to predict
  // them. Stack objects force FP usage with calls.
  //
  // Note a new VGPR CSR may be introduced if one is used for the spill, but we
  // don't want to report it here.
  //
  // FIXME: Is this really hasReservedCallFrame?
  const bool WillHaveFP =
      FrameInfo.hasCalls() &&
      (SavedVGPRs.any() || !allStackObjectsAreDead(FrameInfo));

  // VGPRs used for SGPR spilling need to be specially inserted in the prolog,
  // so don't allow the default insertion to handle them.
  for (auto SSpill : MFI->getSGPRSpillVGPRs())
    SavedVGPRs.reset(SSpill.VGPR);

  const bool HasFP = WillHaveFP || hasFP(MF);
  if (!HasFP)
    return;

  if (MFI->haveFreeLanesForSGPRSpill(MF, 1)) {
    int NewFI = MF.getFrameInfo().CreateStackObject(4, 4, true, nullptr,
                                                    TargetStackID::SGPRSpill);

    // If there is already a VGPR with free lanes, use it. We may already have
    // to pay the penalty for spilling a CSR VGPR.
    if (!MFI->allocateSGPRSpillToVGPR(MF, NewFI))
      llvm_unreachable("allocate SGPR spill should have worked");

    MFI->FramePointerSaveIndex = NewFI;

    LLVM_DEBUG(
      auto Spill = MFI->getSGPRToVGPRSpills(NewFI).front();
      dbgs() << "Spilling FP to  " << printReg(Spill.VGPR, TRI)
             << ':' << Spill.Lane << '\n');
    return;
  }

  MFI->SGPRForFPSaveRestoreCopy = findUnusedSGPRNonCalleeSaved(MF.getRegInfo());

  if (!MFI->SGPRForFPSaveRestoreCopy) {
    // There's no free lane to spill, and no free register to save FP, so we're
    // forced to spill another VGPR to use for the spill.
    int NewFI = MF.getFrameInfo().CreateStackObject(4, 4, true, nullptr,
                                                    TargetStackID::SGPRSpill);
    if (!MFI->allocateSGPRSpillToVGPR(MF, NewFI))
      llvm_unreachable("allocate SGPR spill should have worked");
    MFI->FramePointerSaveIndex = NewFI;

    LLVM_DEBUG(
      auto Spill = MFI->getSGPRToVGPRSpills(NewFI).front();
      dbgs() << "FP requires fallback spill to " << printReg(Spill.VGPR, TRI)
             << ':' << Spill.Lane << '\n';);
  } else {
    LLVM_DEBUG(dbgs() << "Saving FP with copy to " <<
               printReg(MFI->SGPRForFPSaveRestoreCopy, TRI) << '\n');
  }
}

void PPUFrameLowering::determineCalleeSaves(MachineFunction &MF,
                                              BitVector &SavedRegs,
                                              RegScavenger *RS) const {
  if (PPU::isCompute(&MF))
      return determineCalleeSaves_compute(MF, SavedRegs, RS);

  TargetFrameLowering::determineCalleeSaves(MF, SavedRegs, RS);
  // Unconditionally spill RA and FP only if the function uses a frame
  // pointer.
  if (hasFP(MF)) {
    SavedRegs.set(PPU::X1);
    SavedRegs.set(PPU::X8);
  }

  // If interrupt is enabled and there are calls in the handler,
  // unconditionally save all Caller-saved registers and
  // all FP registers, regardless whether they are used.
  MachineFrameInfo &MFI = MF.getFrameInfo();

  if (MF.getFunction().hasFnAttribute("interrupt") && MFI.hasCalls()) {

    static const MCPhysReg CSRegs[] = { PPU::X1,      /* ra */
      PPU::X5, PPU::X6, PPU::X7,                  /* t0-t2 */
      PPU::X10, PPU::X11,                           /* a0-a1, a2-a7 */
      PPU::X12, PPU::X13, PPU::X14, PPU::X15, PPU::X16, PPU::X17,
      PPU::X28, PPU::X29, PPU::X30, PPU::X31, 0 /* t3-t6 */
    };

    for (unsigned i = 0; CSRegs[i]; ++i)
      SavedRegs.set(CSRegs[i]);

    if (MF.getSubtarget<PPUSubtarget>().hasStdExtD() ||
        MF.getSubtarget<PPUSubtarget>().hasStdExtF()) {

      // If interrupt is enabled, this list contains all FP registers.
      const MCPhysReg * Regs = MF.getRegInfo().getCalleeSavedRegs();

      for (unsigned i = 0; Regs[i]; ++i)
        if (PPU::FPR32RegClass.contains(Regs[i]) /*||
            PPU::FPR64RegClass.contains(Regs[i])*/)
          SavedRegs.set(Regs[i]);
    }
  }
}

void PPUFrameLowering::determineCalleeSavesSGPR(MachineFunction &MF,
                                               BitVector &SavedRegs,
                                               RegScavenger *RS) const {
  TargetFrameLowering::determineCalleeSaves(MF, SavedRegs, RS);
  const PPUMachineFunctionInfo *MFI = MF.getInfo<PPUMachineFunctionInfo>();
  if (MFI->isEntryFunction())
    return;

  const PPUSubtarget &ST = MF.getSubtarget<PPUSubtarget>();
  const PPURegisterInfo *TRI = ST.getRegisterInfo();

  // The SP is specifically managed and we don't want extra spills of it.
  SavedRegs.reset(MFI->getStackPtrOffsetReg());
  SavedRegs.clearBitsInMask(TRI->getAllVGPRRegMask());
}

bool PPUFrameLowering::assignCalleeSavedSpillSlots(
    MachineFunction &MF, const TargetRegisterInfo *TRI,
    std::vector<CalleeSavedInfo> &CSI) const {
  if (CSI.empty())
    return true; // Early exit if no callee saved registers are modified!

  const PPUMachineFunctionInfo *FuncInfo = MF.getInfo<PPUMachineFunctionInfo>();
  if (!FuncInfo->SGPRForFPSaveRestoreCopy)
    return false;

  for (auto &CS : CSI) {
    if (CS.getReg() == FuncInfo->getFrameOffsetReg()) {
      if (FuncInfo->SGPRForFPSaveRestoreCopy != PPU::NoRegister)
        CS.setDstReg(FuncInfo->SGPRForFPSaveRestoreCopy);
      break;
    }
  }

  return false;
}

// Not preserve stack space within prologue for outgoing variables when the
// function contains variable size objects and let eliminateCallFramePseudoInstr
// preserve stack space for it.
bool PPUFrameLowering::hasReservedCallFrame(const MachineFunction &MF) const {
  return !MF.getFrameInfo().hasVarSizedObjects();
}

MachineBasicBlock::iterator PPUFrameLowering::eliminateCallFramePseudoInstr_compute(
  MachineFunction &MF,
  MachineBasicBlock &MBB,
  MachineBasicBlock::iterator I) const {
  int64_t Amount = I->getOperand(0).getImm();
  if (Amount == 0)
    return MBB.erase(I);

  const PPUSubtarget &ST = MF.getSubtarget<PPUSubtarget>();
  const PPUInstrInfo *TII = ST.getInstrInfo();
  const DebugLoc &DL = I->getDebugLoc();
  unsigned Opc = I->getOpcode();
  bool IsDestroy = Opc == TII->getCallFrameDestroyOpcode();
  uint64_t CalleePopAmount = IsDestroy ? I->getOperand(1).getImm() : 0;

  if (!hasReservedCallFrame(MF)) {
    unsigned Align = getStackAlignment();

    Amount = alignTo(Amount, Align);
    assert(isUInt<32>(Amount) && "exceeded stack address space size");
    const PPUMachineFunctionInfo *MFI = MF.getInfo<PPUMachineFunctionInfo>();
    unsigned SPReg = MFI->getStackPtrOffsetReg();

    unsigned Op = IsDestroy ? PPU::SUB : PPU::ADD;
    BuildMI(MBB, I, DL, TII->get(Op), SPReg)
      .addReg(SPReg)
      .addImm(Amount * ST.getWavefrontSize());
  } else if (CalleePopAmount != 0) {
    llvm_unreachable("is this used?");
  }

  return MBB.erase(I);
}

// Eliminate ADJCALLSTACKDOWN, ADJCALLSTACKUP pseudo instructions.
MachineBasicBlock::iterator PPUFrameLowering::eliminateCallFramePseudoInstr(
    MachineFunction &MF, MachineBasicBlock &MBB,
    MachineBasicBlock::iterator MI) const {

  if (PPU::isCompute(&MF))
      return eliminateCallFramePseudoInstr_compute(MF, MBB, MI);

  Register SPReg = PPU::X2;
  DebugLoc DL = MI->getDebugLoc();

  if (!hasReservedCallFrame(MF)) {
    // If space has not been reserved for a call frame, ADJCALLSTACKDOWN and
    // ADJCALLSTACKUP must be converted to instructions manipulating the stack
    // pointer. This is necessary when there is a variable length stack
    // allocation (e.g. alloca), which means it's not possible to allocate
    // space for outgoing arguments from within the function prologue.
    int64_t Amount = MI->getOperand(0).getImm();

    if (Amount != 0) {
      // Ensure the stack remains aligned after adjustment.
      Amount = alignSPAdjust(Amount);

      if (MI->getOpcode() == PPU::ADJCALLSTACKDOWN)
        Amount = -Amount;

      adjustReg(MBB, MI, DL, SPReg, SPReg, Amount, MachineInstr::NoFlags);
    }
  }

  return MBB.erase(MI);
}

// below is from AMDGPU

unsigned PPUFrameLowering::getStackWidth(const MachineFunction &MF) const {
  // XXX: Hardcoding to 1 for now.
  //
  // I think the StackWidth should stored as metadata associated with the
  // MachineFunction.  This metadata can either be added by a frontend, or
  // calculated by a R600 specific LLVM IR pass.
  //
  // The StackWidth determines how stack objects are laid out in memory.
  // For a vector stack variable, like: int4 stack[2], the data will be stored
  // in the following ways depending on the StackWidth.
  //
  // StackWidth = 1:
  //
  // T0.X = stack[0].x
  // T1.X = stack[0].y
  // T2.X = stack[0].z
  // T3.X = stack[0].w
  // T4.X = stack[1].x
  // T5.X = stack[1].y
  // T6.X = stack[1].z
  // T7.X = stack[1].w
  //
  // StackWidth = 2:
  //
  // T0.X = stack[0].x
  // T0.Y = stack[0].y
  // T1.X = stack[0].z
  // T1.Y = stack[0].w
  // T2.X = stack[1].x
  // T2.Y = stack[1].y
  // T3.X = stack[1].z
  // T3.Y = stack[1].w
  //
  // StackWidth = 4:
  // T0.X = stack[0].x
  // T0.Y = stack[0].y
  // T0.Z = stack[0].z
  // T0.W = stack[0].w
  // T1.X = stack[1].x
  // T1.Y = stack[1].y
  // T1.Z = stack[1].z
  // T1.W = stack[1].w
  return 1;
}


void PPUFrameLowering::emitFlatScratchInit(const PPUSubtarget &ST,
                                          MachineFunction &MF,
                                          MachineBasicBlock &MBB) const {

  const PPUInstrInfo *TII = ST.getInstrInfo();
  const PPURegisterInfo* TRI = &TII->getRegisterInfo();
  const PPUMachineFunctionInfo *MFI = MF.getInfo<PPUMachineFunctionInfo>();

  // We don't need this if we only have spills since there is no user facing
  // scratch.

  // TODO: If we know we don't have flat instructions earlier, we can omit
  // this from the input registers.
  //
  // TODO: We only need to know if we access scratch space through a flat
  // pointer. Because we only detect if flat instructions are used at all,
  // this will be used more often than necessary on VI.

  // Debug location must be unknown since the first debug location is used to
  // determine the end of the prologue.
  DebugLoc DL;
  MachineBasicBlock::iterator I = MBB.begin();

  Register FlatScratchInitReg =
      MFI->getPreloadedReg(PPUFunctionArgInfo::FLAT_SCRATCH_INIT);

  MachineRegisterInfo &MRI = MF.getRegInfo();
  MRI.addLiveIn(FlatScratchInitReg);
  MBB.addLiveIn(FlatScratchInitReg);

  Register FlatScrInitLo = TRI->getSubReg(FlatScratchInitReg, PPU::sub0);
  Register FlatScrInitHi = TRI->getSubReg(FlatScratchInitReg, PPU::sub1);

  unsigned ScratchWaveOffsetReg = MFI->getScratchWaveOffsetReg();

  // Do a 64-bit pointer add.
  if (ST.flatScratchIsPointer()) {
      /* FIXME use Gfx10 branch?
    if (ST.getGeneration() >= PPUSubtarget::GFX10) {
      BuildMI(MBB, I, DL, TII->get(PPU::S_ADD_U32), FlatScrInitLo)
        .addReg(FlatScrInitLo)
        .addReg(ScratchWaveOffsetReg);
      BuildMI(MBB, I, DL, TII->get(PPU::S_ADDC_U32), FlatScrInitHi)
        .addReg(FlatScrInitHi)
        .addImm(0);
      BuildMI(MBB, I, DL, TII->get(PPU::S_SETREG_B32)).
        addReg(FlatScrInitLo).
        addImm(int16_t(PPU::Hwreg::ID_FLAT_SCR_LO |
                       (31 << PPU::Hwreg::WIDTH_M1_SHIFT_)));
      BuildMI(MBB, I, DL, TII->get(PPU::S_SETREG_B32)).
        addReg(FlatScrInitHi).
        addImm(int16_t(PPU::Hwreg::ID_FLAT_SCR_HI |
                       (31 << PPU::Hwreg::WIDTH_M1_SHIFT_)));
      return;
    }
    */

    BuildMI(MBB, I, DL, TII->get(PPU::S_ADD_U32), PPU::FLAT_SCR_LO)
      .addReg(FlatScrInitLo)
      .addReg(ScratchWaveOffsetReg);
    BuildMI(MBB, I, DL, TII->get(PPU::S_ADDC_U32), PPU::FLAT_SCR_HI)
      .addReg(FlatScrInitHi)
      .addImm(0);

    return;
  }
  llvm_unreachable("FIXME on emitFlatScratchInit");
/* FIXME
  // Copy the size in bytes.
  BuildMI(MBB, I, DL, TII->get(PPU::COPY), PPU::FLAT_SCR_LO)
    .addReg(FlatScrInitHi, RegState::Kill);

  // Add wave offset in bytes to private base offset.
  // See comment in AMDKernelCodeT.h for enable_sgpr_flat_scratch_init.
  BuildMI(MBB, I, DL, TII->get(PPU::ADD_U32), FlatScrInitLo)
    .addReg(FlatScrInitLo)
    .addReg(ScratchWaveOffsetReg);

  // Convert offset to 256-byte units.
  BuildMI(MBB, I, DL, TII->get(PPU::LSHR_B32), PPU::FLAT_SCR_HI)
    .addReg(FlatScrInitLo, RegState::Kill)
    .addImm(8);
    */
}


// FIXME AllSPR128 for Rsrc? or AllSPR64

unsigned PPUFrameLowering::getReservedPrivateSegmentBufferReg(
  const PPUSubtarget &ST,
  const PPUInstrInfo *TII,
  const PPURegisterInfo *TRI,
  PPUMachineFunctionInfo *MFI,
  MachineFunction &MF) const {
  MachineRegisterInfo &MRI = MF.getRegInfo();

  // We need to insert initialization of the scratch resource descriptor.
  unsigned ScratchRsrcReg = MFI->getScratchRSrcReg();
  if (ScratchRsrcReg == PPU::NoRegister ||
      !MRI.isPhysRegUsed(ScratchRsrcReg))
    return PPU::NoRegister;

  if (ScratchRsrcReg != TRI->reservedPrivateSegmentBufferReg(MF))
    return ScratchRsrcReg;

  // We reserved the last registers for this. Shift it down to the end of those
  // which were actually used.
  //
  // FIXME: It might be safer to use a pseudoregister before replacement.

  // FIXME: We should be able to eliminate unused input registers. We only
  // cannot do this for the resources required for scratch access. For now we
  // skip over user SGPRs and may leave unused holes.

  // We find the resource first because it has an alignment requirement.

  unsigned NumPreloaded = (MFI->getNumPreloadedSGPRs() + 3) / 4;
  ArrayRef<MCPhysReg> AllSPR128s = getAllSPR128(ST, MF);
  AllSPR128s = AllSPR128s.slice(std::min(static_cast<unsigned>(AllSPR128s.size()), NumPreloaded));

  // Skip the last N reserved elements because they should have already been
  // reserved for VCC etc.
  for (MCPhysReg Reg : AllSPR128s) {
    // Pick the first unallocated one. Make sure we don't clobber the other
    // reserved input we needed.
    if (!MRI.isPhysRegUsed(Reg) && MRI.isAllocatable(Reg)) {
      MRI.replaceRegWith(ScratchRsrcReg, Reg);
      MFI->setScratchRSrcReg(Reg);
      return Reg;
    }
  }

  return ScratchRsrcReg;
}
// Shift down registers reserved for the scratch wave offset.
std::pair<unsigned, bool>
PPUFrameLowering::getReservedPrivateSegmentWaveByteOffsetReg(
    const PPUSubtarget &ST, const PPUInstrInfo *TII, const PPURegisterInfo *TRI,
    PPUMachineFunctionInfo *MFI, MachineFunction &MF) const {
  MachineRegisterInfo &MRI = MF.getRegInfo();
  unsigned ScratchWaveOffsetReg = MFI->getScratchWaveOffsetReg();

  assert(MFI->isEntryFunction());

  // No replacement necessary.
  if (ScratchWaveOffsetReg == PPU::NoRegister ||
      (!hasFP(MF) && !MRI.isPhysRegUsed(ScratchWaveOffsetReg))) {
    return std::make_pair(PPU::NoRegister, false);
  }

  unsigned NumPreloaded = MFI->getNumPreloadedSGPRs();

  ArrayRef<MCPhysReg> AllSPRs = getAllSPRs(ST, MF);
  if (NumPreloaded > AllSPRs.size())
    return std::make_pair(ScratchWaveOffsetReg, false);

  AllSPRs = AllSPRs.slice(NumPreloaded);

  // We need to drop register from the end of the list that we cannot use
  // for the scratch wave offset.
  // + 2 s102 and s103 do not exist on VI.
  // + 2 for vcc
  // + 2 for xnack_mask
  // + 2 for flat_scratch
  // + 4 for registers reserved for scratch resource register
  // + 1 for register reserved for scratch wave offset.  (By exluding this
  //     register from the list to consider, it means that when this
  //     register is being used for the scratch wave offset and there
  //     are no other free SGPRs, then the value will stay in this register.
  // + 1 if stack pointer is used.
  // ----
  //  13 (+1)
  unsigned ReservedRegCount = 13;

  if (AllSPRs.size() < ReservedRegCount)
    return std::make_pair(ScratchWaveOffsetReg, false);

  bool HandledScratchWaveOffsetReg =
    ScratchWaveOffsetReg != TRI->reservedPrivateSegmentWaveByteOffsetReg(MF);
  bool FPAdjusted = false;

  for (MCPhysReg Reg : AllSPRs.drop_back(ReservedRegCount)) {
    // Pick the first unallocated SGPR. Be careful not to pick an alias of the
    // scratch descriptor, since we haven’t added its uses yet.
    if (!MRI.isPhysRegUsed(Reg) && MRI.isAllocatable(Reg)) {
      if (!HandledScratchWaveOffsetReg) {
        HandledScratchWaveOffsetReg = true;

        MRI.replaceRegWith(ScratchWaveOffsetReg, Reg);
        if (MFI->getScratchWaveOffsetReg() == MFI->getStackPtrOffsetReg()) {
          assert(!hasFP(MF));
          MFI->setStackPtrOffsetReg(Reg);
        }

        MFI->setScratchWaveOffsetReg(Reg);
        MFI->setFrameOffsetReg(Reg);
        ScratchWaveOffsetReg = Reg;
        FPAdjusted = true;
        break;
      }
    }
  }

  return std::make_pair(ScratchWaveOffsetReg, FPAdjusted);
}


void PPUFrameLowering::emitEntryFunctionPrologue(MachineFunction &MF,
                                                MachineBasicBlock &MBB) const {
  assert(&MF.front() == &MBB && "Shrink-wrapping not yet supported");

  PPUMachineFunctionInfo *MFI = MF.getInfo<PPUMachineFunctionInfo>();

  // If we only have SGPR spills, we won't actually be using scratch memory
  // since these spill to VGPRs.
  //
  // FIXME: We should be cleaning up these unused SGPR spill frame indices
  // somewhere.

  const PPUSubtarget &ST = MF.getSubtarget<PPUSubtarget>();
  const PPUInstrInfo *TII = ST.getInstrInfo();
  const PPURegisterInfo *TRI = &TII->getRegisterInfo();
  MachineRegisterInfo &MRI = MF.getRegInfo();
  const Function &F = MF.getFunction();

  // We need to do the replacement of the private segment buffer and wave offset
  // register even if there are no stack objects. There could be stores to undef
  // or a constant without an associated object.

  // FIXME: We still have implicit uses on SGPR spill instructions in case they
  // need to spill to vector memory. It's likely that will not happen, but at
  // this point it appears we need the setup. This part of the prolog should be
  // emitted after frame indices are eliminated.

  if (MFI->hasFlatScratchInit())
    emitFlatScratchInit(ST, MF, MBB);

  unsigned ScratchRsrcReg
    = getReservedPrivateSegmentBufferReg(ST, TII, TRI, MFI, MF);

  unsigned ScratchWaveOffsetReg;
  bool FPAdjusted;
  std::tie(ScratchWaveOffsetReg, FPAdjusted) =
    getReservedPrivateSegmentWaveByteOffsetReg(ST, TII, TRI, MFI, MF);

  // We need to insert initialization of the scratch resource descriptor.
  unsigned PreloadedScratchWaveOffsetReg = MFI->getPreloadedReg(
    PPUFunctionArgInfo::PRIVATE_SEGMENT_WAVE_BYTE_OFFSET);

  unsigned PreloadedPrivateBufferReg = PPU::NoRegister;
  if (ST.isPPSOS(F)) {
    PreloadedPrivateBufferReg = MFI->getPreloadedReg(
      PPUFunctionArgInfo::PRIVATE_SEGMENT_BUFFER);
  }

  bool OffsetRegUsed = ScratchWaveOffsetReg != PPU::NoRegister &&
                       MRI.isPhysRegUsed(ScratchWaveOffsetReg);

  bool ResourceRegUsed = ScratchRsrcReg != PPU::NoRegister &&
                        MRI.isPhysRegUsed(ScratchRsrcReg);

  // FIXME: Hack to not crash in situations which emitted an error.
  if (PreloadedScratchWaveOffsetReg == PPU::NoRegister)
    return;

  // We added live-ins during argument lowering, but since they were not used
  // they were deleted. We're adding the uses now, so add them back.
  MRI.addLiveIn(PreloadedScratchWaveOffsetReg);
  MBB.addLiveIn(PreloadedScratchWaveOffsetReg);

  if (ResourceRegUsed && PreloadedPrivateBufferReg != PPU::NoRegister) {
    assert(ST.isPPSOS(F));
    MRI.addLiveIn(PreloadedPrivateBufferReg);
    MBB.addLiveIn(PreloadedPrivateBufferReg);
  }


  // Make the register selected live throughout the function.
  for (MachineBasicBlock &OtherBB : MF) {
    if (&OtherBB == &MBB)
      continue;

    if (OffsetRegUsed)
      OtherBB.addLiveIn(ScratchWaveOffsetReg);

    if (ResourceRegUsed)
      OtherBB.addLiveIn(ScratchRsrcReg);
  }

  DebugLoc DL;
  MachineBasicBlock::iterator I = MBB.begin();

  // If we reserved the original input registers, we don't need to copy to the
  // reserved registers.

  bool CopyBuffer = ResourceRegUsed &&
    PreloadedPrivateBufferReg != PPU::NoRegister &&
    ST.isPPSOS(F) &&
    ScratchRsrcReg != PreloadedPrivateBufferReg;

  // This needs to be careful of the copying order to avoid overwriting one of
  // the input registers before it's been copied to it's final
  // destination. Usually the offset should be copied first.
  bool CopyBufferFirst = TRI->isSubRegisterEq(PreloadedPrivateBufferReg,
                                              ScratchWaveOffsetReg);
  if (CopyBuffer && CopyBufferFirst) {
    BuildMI(MBB, I, DL, TII->get(PPU::COPY), ScratchRsrcReg)
      .addReg(PreloadedPrivateBufferReg, RegState::Kill);
  }

  unsigned SPReg = MFI->getStackPtrOffsetReg();
  assert(SPReg != PPU::SP_REG);

  // FIXME: Remove the isPhysRegUsed checks
  const bool HasFP = hasFP(MF);

  if (HasFP || OffsetRegUsed) {
    assert(ScratchWaveOffsetReg);
    BuildMI(MBB, I, DL, TII->get(PPU::COPY), ScratchWaveOffsetReg)
      .addReg(PreloadedScratchWaveOffsetReg, HasFP ? RegState::Kill : 0);
  }

  if (CopyBuffer && !CopyBufferFirst) {
    BuildMI(MBB, I, DL, TII->get(PPU::COPY), ScratchRsrcReg)
      .addReg(PreloadedPrivateBufferReg, RegState::Kill);
  }

  if (ResourceRegUsed) {
    emitEntryFunctionScratchSetup(ST, MF, MBB, MFI, I,
        PreloadedPrivateBufferReg, ScratchRsrcReg);
  }

  if (HasFP) {
    DebugLoc DL;
    const MachineFrameInfo &FrameInfo = MF.getFrameInfo();
    int64_t StackSize = FrameInfo.getStackSize();

    // On kernel entry, the private scratch wave offset is the SP value.
    if (StackSize == 0) {
      BuildMI(MBB, I, DL, TII->get(PPU::COPY), SPReg)
        .addReg(MFI->getScratchWaveOffsetReg());
    } else {
      BuildMI(MBB, I, DL, TII->get(PPU::S_ADD_U32), SPReg)
        .addReg(MFI->getScratchWaveOffsetReg())
        .addImm(StackSize * ST.getWavefrontSize());
    }
  }
}

// Emit scratch setup code for AMDPAL or Mesa, assuming ResourceRegUsed is set.
void PPUFrameLowering::emitEntryFunctionScratchSetup(const PPUSubtarget &ST,
      MachineFunction &MF, MachineBasicBlock &MBB, PPUMachineFunctionInfo *MFI,
      MachineBasicBlock::iterator I, unsigned PreloadedPrivateBufferReg,
      unsigned ScratchRsrcReg) const {
    return;
  // llvm_unreachable("Invalid TargetStackID::Value");
/*
  const SIInstrInfo *TII = ST.getInstrInfo();
  const SIRegisterInfo *TRI = &TII->getRegisterInfo();
  const Function &Fn = MF.getFunction();
  DebugLoc DL;

  if (ST.isAmdPalOS()) {
    // The pointer to the GIT is formed from the offset passed in and either
    // the amdgpu-git-ptr-high function attribute or the top part of the PC
    Register RsrcLo = TRI->getSubReg(ScratchRsrcReg, PPU::sub0);
    Register RsrcHi = TRI->getSubReg(ScratchRsrcReg, PPU::sub1);
    Register Rsrc01 = TRI->getSubReg(ScratchRsrcReg, PPU::sub0_sub1);

    const MCInstrDesc &SMovB32 = TII->get(PPU::S_MOV_B32);

    if (MFI->getGITPtrHigh() != 0xffffffff) {
      BuildMI(MBB, I, DL, SMovB32, RsrcHi)
        .addImm(MFI->getGITPtrHigh())
        .addReg(ScratchRsrcReg, RegState::ImplicitDefine);
    } else {
      const MCInstrDesc &GetPC64 = TII->get(PPU::S_GETPC_B64);
      BuildMI(MBB, I, DL, GetPC64, Rsrc01);
    }
    auto GitPtrLo = PPU::SGPR0; // Low GIT address passed in
    if (ST.hasMergedShaders()) {
      switch (MF.getFunction().getCallingConv()) {
        case CallingConv::AMDGPU_HS:
        case CallingConv::AMDGPU_GS:
          // Low GIT address is passed in s8 rather than s0 for an LS+HS or
          // ES+GS merged shader on gfx9+.
          GitPtrLo = PPU::SGPR8;
          break;
        default:
          break;
      }
    }
    MF.getRegInfo().addLiveIn(GitPtrLo);
    MBB.addLiveIn(GitPtrLo);
    BuildMI(MBB, I, DL, SMovB32, RsrcLo)
      .addReg(GitPtrLo)
      .addReg(ScratchRsrcReg, RegState::ImplicitDefine);

    // We now have the GIT ptr - now get the scratch descriptor from the entry
    // at offset 0 (or offset 16 for a compute shader).
    PointerType *PtrTy =
      PointerType::get(Type::getInt64Ty(MF.getFunction().getContext()),
                       AMDGPUAS::CONSTANT_ADDRESS);
    MachinePointerInfo PtrInfo(UndefValue::get(PtrTy));
    const MCInstrDesc &LoadDwordX4 = TII->get(PPU::S_LOAD_DWORDX4_IMM);
    auto MMO = MF.getMachineMemOperand(PtrInfo,
                                       MachineMemOperand::MOLoad |
                                       MachineMemOperand::MOInvariant |
                                       MachineMemOperand::MODereferenceable,
                                       16, 4);
    unsigned Offset = Fn.getCallingConv() == CallingConv::AMDGPU_CS ? 16 : 0;
    const GCNSubtarget &Subtarget = MF.getSubtarget<GCNSubtarget>();
    unsigned EncodedOffset = PPU::getSMRDEncodedOffset(Subtarget, Offset);
    BuildMI(MBB, I, DL, LoadDwordX4, ScratchRsrcReg)
      .addReg(Rsrc01)
      .addImm(EncodedOffset) // offset
      .addImm(0) // glc
      .addImm(0) // dlc
      .addReg(ScratchRsrcReg, RegState::ImplicitDefine)
      .addMemOperand(MMO);
    return;
  }
  if (ST.isMesaGfxShader(Fn)
      || (PreloadedPrivateBufferReg == PPU::NoRegister)) {
    assert(!ST.isAmdHsaOrMesa(Fn));
    const MCInstrDesc &SMovB32 = TII->get(PPU::S_MOV_B32);

    Register Rsrc2 = TRI->getSubReg(ScratchRsrcReg, PPU::sub2);
    Register Rsrc3 = TRI->getSubReg(ScratchRsrcReg, PPU::sub3);

    // Use relocations to get the pointer, and setup the other bits manually.
    uint64_t Rsrc23 = TII->getScratchRsrcWords23();

    if (MFI->hasImplicitBufferPtr()) {
      Register Rsrc01 = TRI->getSubReg(ScratchRsrcReg, PPU::sub0_sub1);

      if (PPU::isCompute(MF.getFunction().getCallingConv())) {
        const MCInstrDesc &Mov64 = TII->get(PPU::S_MOV_B64);

        BuildMI(MBB, I, DL, Mov64, Rsrc01)
          .addReg(MFI->getImplicitBufferPtrUserSGPR())
          .addReg(ScratchRsrcReg, RegState::ImplicitDefine);
      } else {
        const MCInstrDesc &LoadDwordX2 = TII->get(PPU::S_LOAD_DWORDX2_IMM);

        PointerType *PtrTy =
          PointerType::get(Type::getInt64Ty(MF.getFunction().getContext()),
                           AMDGPUAS::CONSTANT_ADDRESS);
        MachinePointerInfo PtrInfo(UndefValue::get(PtrTy));
        auto MMO = MF.getMachineMemOperand(PtrInfo,
                                           MachineMemOperand::MOLoad |
                                           MachineMemOperand::MOInvariant |
                                           MachineMemOperand::MODereferenceable,
                                           8, 4);
        BuildMI(MBB, I, DL, LoadDwordX2, Rsrc01)
          .addReg(MFI->getImplicitBufferPtrUserSGPR())
          .addImm(0) // offset
          .addImm(0) // glc
          .addImm(0) // dlc
          .addMemOperand(MMO)
          .addReg(ScratchRsrcReg, RegState::ImplicitDefine);

        MF.getRegInfo().addLiveIn(MFI->getImplicitBufferPtrUserSGPR());
        MBB.addLiveIn(MFI->getImplicitBufferPtrUserSGPR());
      }
    } else {
      Register Rsrc0 = TRI->getSubReg(ScratchRsrcReg, PPU::sub0);
      Register Rsrc1 = TRI->getSubReg(ScratchRsrcReg, PPU::sub1);

      BuildMI(MBB, I, DL, SMovB32, Rsrc0)
        .addExternalSymbol("SCRATCH_RSRC_DWORD0")
        .addReg(ScratchRsrcReg, RegState::ImplicitDefine);

      BuildMI(MBB, I, DL, SMovB32, Rsrc1)
        .addExternalSymbol("SCRATCH_RSRC_DWORD1")
        .addReg(ScratchRsrcReg, RegState::ImplicitDefine);

    }

    BuildMI(MBB, I, DL, SMovB32, Rsrc2)
      .addImm(Rsrc23 & 0xffffffff)
      .addReg(ScratchRsrcReg, RegState::ImplicitDefine);

    BuildMI(MBB, I, DL, SMovB32, Rsrc3)
      .addImm(Rsrc23 >> 32)
      .addReg(ScratchRsrcReg, RegState::ImplicitDefine);
  }
*/
}

bool PPUFrameLowering::isSupportedStackID(TargetStackID::Value ID) const {
  switch (ID) {
  case TargetStackID::Default:
  case TargetStackID::NoAlloc:
  case TargetStackID::SGPRSpill:
    return true;
  }
  llvm_unreachable("Invalid TargetStackID::Value");
}

