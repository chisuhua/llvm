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
#include "PPUInstrInfo.h"
#include "PPUMachineFunctionInfo.h"
#include "MCTargetDesc/PPUInstPrinter.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/RegisterScavenging.h"
#include "llvm/CodeGen/TargetFrameLowering.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
#include "llvm/Support/ErrorHandling.h"
// #include "llvm/IR/Function.h"

#define GET_REGINFO_TARGET_DESC
#include "PPUGenRegisterInfo.inc"

using namespace llvm;

PPUBaseRegisterInfo::PPUBaseRegisterInfo(unsigned HwMode)
    : PPUGenRegisterInfo(PPU::X1, /*DwarfFlavour*/0, /*EHFlavor*/0,
                           /*PC*/0, HwMode) {}

const MCPhysReg *
PPUBaseRegisterInfo::getCalleeSavedRegs(const MachineFunction *MF) const {
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

BitVector PPUBaseRegisterInfo::getReservedRegs(const MachineFunction &MF) const {
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
  markSuperRegs(Reserved, PPU::VCFG); // vcfg CSR
  assert(checkAllSuperRegsMarked(Reserved));
  return Reserved;
}

bool PPUBaseRegisterInfo::isConstantPhysReg(unsigned PhysReg) const {
  return PhysReg == PPU::X0;
}

const uint32_t *PPUBaseRegisterInfo::getNoPreservedMask() const {
  return CSR_NoRegs_RegMask;
}

void PPUBaseRegisterInfo::eliminateFrameIndex(MachineBasicBlock::iterator II,
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

Register PPUBaseRegisterInfo::getFrameRegister(const MachineFunction &MF) const {
  const TargetFrameLowering *TFI = getFrameLowering(MF);
  return TFI->hasFP(MF) ? PPU::X8 : PPU::X2;
}

const uint32_t *PPURegisterInfo::getAllVGPRRegMask() const {
  return CSR_PPU_AllVPRs_RegMask;
}

const uint32_t *
PPUBaseRegisterInfo::getCallPreservedMask(const MachineFunction & MF,
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

// unsigned PPUBaseRegisterInfo::getVCC() const {
//   return PPU::VCC;
// }

static bool hasPressureSet(const int *PSets, unsigned PSetID) {
  for (unsigned i = 0; PSets[i] != -1; ++i) {
    if (PSets[i] == (int)PSetID)
      return true;
  }
  return false;
}

void PPURegisterInfo::classifyPressureSet(unsigned PSetID, unsigned Reg,
                                         BitVector &PressureSets) const {
  for (MCRegUnitIterator U(Reg, this); U.isValid(); ++U) {
    const int *PSets = getRegUnitPressureSets(*U);
    if (hasPressureSet(PSets, PSetID)) {
      PressureSets.set(PSetID);
      break;
    }
  }
}

static cl::opt<bool> EnableSpillSGPRToSMEM(
  "ppu-spill-sgpr-to-smem",
  cl::desc("Use scalar stores to spill SGPRs if supported by subtarget"),
  cl::init(false));

static cl::opt<bool> EnableSpillSGPRToVGPR(
  "ppu-spill-sgpr-to-vgpr",
  cl::desc("Enable spilling VGPRs to SGPRs"),
  cl::ReallyHidden,
  cl::init(false));

// FIXME argument is not same as origina PPURegisterInfo
PPURegisterInfo::PPURegisterInfo(const PPUSubtarget &ST, unsigned HwMode) :
  PPUBaseRegisterInfo(HwMode),
  ST(ST),
  SGPRPressureSets(getNumRegPressureSets()),
  VGPRPressureSets(getNumRegPressureSets()),
  SpillSGPRToVGPR(false),
  SpillSGPRToSMEM(false),
  isWave32(ST.isWave32()) {
  if (EnableSpillSGPRToSMEM)
    SpillSGPRToSMEM = true;
  else if (EnableSpillSGPRToVGPR)
    SpillSGPRToVGPR = true;

  unsigned NumRegPressureSets = getNumRegPressureSets();

  SGPRSetID = NumRegPressureSets;
  VGPRSetID = NumRegPressureSets;

  for (unsigned i = 0; i < NumRegPressureSets; ++i) {
    classifyPressureSet(i, PPU::SPR0, SGPRPressureSets);
    classifyPressureSet(i, PPU::VPR0, VGPRPressureSets);
  }

  // Determine the number of reg units for each pressure set.
  std::vector<unsigned> PressureSetRegUnits(NumRegPressureSets, 0);
  for (unsigned i = 0, e = getNumRegUnits(); i != e; ++i) {
    const int *PSets = getRegUnitPressureSets(i);
    for (unsigned j = 0; PSets[j] != -1; ++j) {
      ++PressureSetRegUnits[PSets[j]];
    }
  }

  unsigned VGPRMax = 0, SGPRMax = 0;
  for (unsigned i = 0; i < NumRegPressureSets; ++i) {
    if (isVGPRPressureSet(i) && PressureSetRegUnits[i] > VGPRMax) {
      VGPRSetID = i;
      VGPRMax = PressureSetRegUnits[i];
      continue;
    }
    if (isSGPRPressureSet(i) && PressureSetRegUnits[i] > SGPRMax) {
      SGPRSetID = i;
      SGPRMax = PressureSetRegUnits[i];
    }
  }

  assert(SGPRSetID < NumRegPressureSets &&
         VGPRSetID < NumRegPressureSets);
}

unsigned PPURegisterInfo::getSubRegFromChannel(unsigned Channel) {
  static const unsigned SubRegs[] = {
      PPU::X0, PPU::X1
      /* FIXME
    PPU::sub0, PPU::sub1, PPU::sub2, PPU::sub3, PPU::sub4,
    PPU::sub5, PPU::sub6, PPU::sub7, PPU::sub8, PPU::sub9,
    PPU::sub10, PPU::sub11, PPU::sub12, PPU::sub13, PPU::sub14,
    PPU::sub15, PPU::sub16, PPU::sub17, PPU::sub18, PPU::sub19,
    PPU::sub20, PPU::sub21, PPU::sub22, PPU::sub23, PPU::sub24,
    PPU::sub25, PPU::sub26, PPU::sub27, PPU::sub28, PPU::sub29,
    PPU::sub30, PPU::sub31
    */
  };

  assert(Channel < array_lengthof(SubRegs));
  return SubRegs[Channel];
}

void PPURegisterInfo::reserveRegisterTuples(BitVector &Reserved, unsigned Reg) const {
  MCRegAliasIterator R(Reg, this, true);

  for (; R.isValid(); ++R)
    Reserved.set(*R);
}


unsigned PPURegisterInfo::reservedPrivateSegmentBufferReg(
  const MachineFunction &MF) const {

  const PPUSubtarget &ST = MF.getSubtarget<PPUSubtarget>();
  unsigned BaseIdx = alignDown(ST.getMaxNumSGPRs(MF), 4) - 4;
  unsigned BaseReg(PPU::SPR_32RegClass.getRegister(BaseIdx));
  return getMatchingSuperReg(BaseReg, PPU::sub0, &PPU::SReg_64RegClass);
  // return getMatchingSuperReg(BaseReg, PPU::sub0, &PPU::SReg_128RegClass);
  // return getMatchingSuperReg(BaseReg, PPU::X0, &PPU::GPRRegClass);
}

static unsigned findPrivateSegmentWaveByteOffsetRegIndex(unsigned RegCount) {
  unsigned Reg;

  // Try to place it in a hole after PrivateSegmentBufferReg.
  if (RegCount & 3) {
    // We cannot put the segment buffer in (Idx - 4) ... (Idx - 1) due to
    // alignment constraints, so we have a hole where can put the wave offset.
    Reg = RegCount - 1;
  } else {
    // We can put the segment buffer in (Idx - 4) ... (Idx - 1) and put the
    // wave offset before it.
    Reg = RegCount - 5;
  }

  return Reg;
}

unsigned PPURegisterInfo::reservedPrivateSegmentWaveByteOffsetReg(
  const MachineFunction &MF) const {
  const PPUSubtarget &ST = MF.getSubtarget<PPUSubtarget>();
  unsigned Reg = findPrivateSegmentWaveByteOffsetRegIndex(ST.getMaxNumSGPRs(MF));
  return PPU::SPR_32RegClass.getRegister(Reg);
  // return PPU::GPRRegClass.getRegister(Reg);
}

BitVector PPURegisterInfo::getReservedRegs(const MachineFunction &MF) const {
  BitVector Reserved(getNumRegs());

  // EXEC_LO and EXEC_HI could be allocated and used as regular register, but
  // this seems likely to result in bugs, so I'm marking them as reserved.
  reserveRegisterTuples(Reserved, PPU::TMSK);
/*
  reserveRegisterTuples(Reserved, PPU::FLAT_SCR);

  // M0 has to be reserved so that llvm accepts it as a live-in into a block.
  reserveRegisterTuples(Reserved, PPU::M0);

  // Reserve src_vccz, src_execz, src_scc.
  reserveRegisterTuples(Reserved, PPU::SRC_VCCZ);
  reserveRegisterTuples(Reserved, PPU::SRC_EXECZ);
  reserveRegisterTuples(Reserved, PPU::SRC_SCC);

  // Reserve the memory aperture registers.
  reserveRegisterTuples(Reserved, PPU::SRC_SHARED_BASE);
  reserveRegisterTuples(Reserved, PPU::SRC_SHARED_LIMIT);
  reserveRegisterTuples(Reserved, PPU::SRC_PRIVATE_BASE);
  reserveRegisterTuples(Reserved, PPU::SRC_PRIVATE_LIMIT);

  // Reserve src_pops_exiting_wave_id - support is not implemented in Codegen.
  reserveRegisterTuples(Reserved, PPU::SRC_POPS_EXITING_WAVE_ID);

  // Reserve xnack_mask registers - support is not implemented in Codegen.
  reserveRegisterTuples(Reserved, PPU::XNACK_MASK);

  // Reserve lds_direct register - support is not implemented in Codegen.
  reserveRegisterTuples(Reserved, PPU::LDS_DIRECT);

  // Reserve Trap Handler registers - support is not implemented in Codegen.
  reserveRegisterTuples(Reserved, PPU::TBA);
  reserveRegisterTuples(Reserved, PPU::TMA);
  reserveRegisterTuples(Reserved, PPU::TTMP0_TTMP1);
  reserveRegisterTuples(Reserved, PPU::TTMP2_TTMP3);
  reserveRegisterTuples(Reserved, PPU::TTMP4_TTMP5);
  reserveRegisterTuples(Reserved, PPU::TTMP6_TTMP7);
  reserveRegisterTuples(Reserved, PPU::TTMP8_TTMP9);
  reserveRegisterTuples(Reserved, PPU::TTMP10_TTMP11);
  reserveRegisterTuples(Reserved, PPU::TTMP12_TTMP13);
  reserveRegisterTuples(Reserved, PPU::TTMP14_TTMP15);

  // Reserve null register - it shall never be allocated
  reserveRegisterTuples(Reserved, PPU::SGPR_NULL);
*/
  // Disallow vcc_hi allocation in wave32. It may be allocated but most likely
  // will result in bugs.
  if (isWave32) {
    Reserved.set(PPU::VCC);
  }

  const PPUSubtarget &ST = MF.getSubtarget<PPUSubtarget>();

  unsigned MaxNumSGPRs = ST.getMaxNumSGPRs(MF);
  // unsigned TotalNumSGPRs = PPU::SGPR_32RegClass.getNumRegs();
  unsigned TotalNumSGPRs = PPU::SPR_32RegClass.getNumRegs();
  for (unsigned i = MaxNumSGPRs; i < TotalNumSGPRs; ++i) {
    // unsigned Reg = PPU::SGPR_32RegClass.getRegister(i);
    unsigned Reg = PPU::SPR_32RegClass.getRegister(i);
    reserveRegisterTuples(Reserved, Reg);
  }

  unsigned MaxNumVGPRs = ST.getMaxNumVGPRs(MF);
  // unsigned TotalNumVGPRs = PPU::VPR_32RegClass.getNumRegs();
  unsigned TotalNumVGPRs = PPU::VPR_32RegClass.getNumRegs();
  for (unsigned i = MaxNumVGPRs; i < TotalNumVGPRs; ++i) {
    // unsigned Reg = PPU::VPR_32RegClass.getRegister(i);
    unsigned Reg = PPU::VPR_32RegClass.getRegister(i);
    reserveRegisterTuples(Reserved, Reg);
  }

  const PPUMachineFunctionInfo *MFI = MF.getInfo<PPUMachineFunctionInfo>();

  /* TODO  enable scratch in future for per wave spill
  unsigned ScratchWaveOffsetReg = MFI->getScratchWaveOffsetReg();
  if (ScratchWaveOffsetReg != PPU::NoRegister) {
    // Reserve 1 SGPR for scratch wave offset in case we need to spill.
    reserveRegisterTuples(Reserved, ScratchWaveOffsetReg);
  }

  unsigned ScratchRSrcReg = MFI->getScratchRSrcReg();
  if (ScratchRSrcReg != PPU::NoRegister) {
    // Reserve 4 SGPRs for the scratch buffer resource descriptor in case we need
    // to spill.
    // TODO: May need to reserve a VGPR if doing LDS spilling.
    reserveRegisterTuples(Reserved, ScratchRSrcReg);
    assert(!isSubRegister(ScratchRSrcReg, ScratchWaveOffsetReg));
  }
  */

  // We have to assume the SP is needed in case there are calls in the function,
  // which is detected after the function is lowered. If we aren't really going
  // to need SP, don't bother reserving it.
  unsigned StackPtrReg = MFI->getStackPtrOffsetReg();

  /*
  if (StackPtrReg != PPU::NoRegister) {
    reserveRegisterTuples(Reserved, StackPtrReg);
    assert(!isSubRegister(ScratchRSrcReg, StackPtrReg));
  }

  unsigned FrameReg = MFI->getFrameOffsetReg();
  if (FrameReg != PPU::NoRegister) {
    reserveRegisterTuples(Reserved, FrameReg);
    assert(!isSubRegister(ScratchRSrcReg, FrameReg));
  }

  for (unsigned Reg : MFI->WWMReservedRegs) {
    reserveRegisterTuples(Reserved, Reg);
  }
  */

  return Reserved;
}

// Forced to be here by one .inc
const MCPhysReg *PPURegisterInfo::getCalleeSavedRegs(
  const MachineFunction *MF) const {
  CallingConv::ID CC = MF->getFunction().getCallingConv();

  if (PPU::isCompute(CC)) {
    // Dummy to not crash RegisterClassInfo.
    static const MCPhysReg NoCalleeSavedReg = PPU::NoRegister;
    return &NoCalleeSavedReg;
  }
  return PPUBaseRegisterInfo::getCalleeSavedRegs(MF);
}

const MCPhysReg * PPURegisterInfo::getCalleeSavedRegsViaCopy(const MachineFunction *MF) const {
  return nullptr;
}

Register PPURegisterInfo::getFrameRegister(const MachineFunction &MF) const {
  CallingConv::ID CC = MF.getFunction().getCallingConv();

  if (!PPU::isCompute(CC)) {
      return PPUBaseRegisterInfo::getFrameRegister(MF);
  }

  const PPUFrameLowering *TFI = MF.getSubtarget<PPUSubtarget>().getFrameLowering();
  const PPUMachineFunctionInfo *FuncInfo = MF.getInfo<PPUMachineFunctionInfo>();
  return TFI->hasFP(MF) ? FuncInfo->getFrameOffsetReg()
                        : FuncInfo->getStackPtrOffsetReg();
}

bool PPURegisterInfo::canRealignStack(const MachineFunction &MF) const {
  const PPUMachineFunctionInfo *Info = MF.getInfo<PPUMachineFunctionInfo>();
  // On entry, the base address is 0, so it can't possibly need any more
  // alignment.

  // FIXME: Should be able to specify the entry frame alignment per calling
  // convention instead.
  if (Info->isEntryFunction())
    return false;

  return TargetRegisterInfo::canRealignStack(MF);
}

bool PPURegisterInfo::requiresRegisterScavenging(const MachineFunction &Fn) const {
  const PPUMachineFunctionInfo *Info = Fn.getInfo<PPUMachineFunctionInfo>();
  if (Info->isEntryFunction()) {
    const MachineFrameInfo &MFI = Fn.getFrameInfo();
    return MFI.hasStackObjects() || MFI.hasCalls();
  }

  // May need scavenger for dealing with callee saved registers.
  return true;
}

bool PPURegisterInfo::requiresFrameIndexScavenging(
  const MachineFunction &MF) const {
  // Do not use frame virtual registers. They used to be used for SGPRs, but
  // once we reach PrologEpilogInserter, we can no longer spill SGPRs. If the
  // scavenger fails, we can increment/decrement the necessary SGPRs to avoid a
  // spill.
  return false;
}

bool PPURegisterInfo::requiresFrameIndexReplacementScavenging(
  const MachineFunction &MF) const {
  const MachineFrameInfo &MFI = MF.getFrameInfo();
  return MFI.hasStackObjects();
}

bool PPURegisterInfo::requiresVirtualBaseRegisters(
  const MachineFunction &) const {
  // There are no special dedicated stack or frame pointers.
  return true;
}

bool PPURegisterInfo::trackLivenessAfterRegAlloc(const MachineFunction &MF) const {
  // This helps catch bugs as verifier errors.
  return true;
}

/* FIXME
int64_t PPURegisterInfo::getMUBUFInstrOffset(const MachineInstr *MI) const {
  assert(PPUInstrInfo::isMUBUF(*MI));

  int OffIdx = PPU::getNamedOperandIdx(MI->getOpcode(),
                                          PPU::OpName::offset);
  return MI->getOperand(OffIdx).getImm();
}

int64_t PPURegisterInfo::getFrameIndexInstrOffset(const MachineInstr *MI,
                                                 int Idx) const {
  if (!PPUInstrInfo::isMUBUF(*MI))
    return 0;

  assert(Idx == PPU::getNamedOperandIdx(MI->getOpcode(),
                                           PPU::OpName::vaddr) &&
         "Should never see frame index on non-address operand");

  return getMUBUFInstrOffset(MI);
}
*/
/*
bool PPURegisterInfo::needsFrameBaseReg(MachineInstr *MI, int64_t Offset) const {
  if (!MI->mayLoadOrStore())
    return false;

  int64_t FullOffset = Offset + getMUBUFInstrOffset(MI);

  return !isUInt<12>(FullOffset);
}
*/

void PPURegisterInfo::materializeFrameBaseRegister(MachineBasicBlock *MBB,
                                                  unsigned BaseReg,
                                                  int FrameIdx,
                                                  int64_t Offset) const {
  MachineBasicBlock::iterator Ins = MBB->begin();
  DebugLoc DL; // Defaults to "unknown"

  if (Ins != MBB->end())
    DL = Ins->getDebugLoc();

  MachineFunction *MF = MBB->getParent();
  const PPUSubtarget &Subtarget = MF->getSubtarget<PPUSubtarget>();
  const PPUInstrInfo *TII = Subtarget.getInstrInfo();

  if (Offset == 0) {
    // FIXME BuildMI(*MBB, Ins, DL, TII->get(PPU::V_MOV_B32_e32), BaseReg)
    BuildMI(*MBB, Ins, DL, TII->get(PPU::VADD), BaseReg)
      .addFrameIndex(FrameIdx);
    return;
  }

  MachineRegisterInfo &MRI = MF->getRegInfo();
  // TODO Register OffsetReg = MRI.createVirtualRegister(&PPU::SReg_32_XM0RegClass);
  Register OffsetReg = MRI.createVirtualRegister(&PPU::SPR_32RegClass);

  // Register FIReg = MRI.createVirtualRegister(&PPU::VPR_32RegClass);
  Register FIReg = MRI.createVirtualRegister(&PPU::VPR_32RegClass);

  // FIXME BuildMI(*MBB, Ins, DL, TII->get(PPU::S_MOV_B32), OffsetReg)
  BuildMI(*MBB, Ins, DL, TII->get(PPU::ADD), OffsetReg)
    .addImm(Offset);
  // FIXME BuildMI(*MBB, Ins, DL, TII->get(PPU::V_MOV_B32_e32), FIReg)
  BuildMI(*MBB, Ins, DL, TII->get(PPU::VADD), FIReg)
    .addFrameIndex(FrameIdx);

  // TII->getAddNoCarry(*MBB, Ins, DL, BaseReg)
  TII->getAddNoCarry(*MBB, Ins, DL, BaseReg)
    .addReg(OffsetReg, RegState::Kill)
    .addReg(FIReg)
    .addImm(0); // clamp bit
}

void PPURegisterInfo::resolveFrameIndex(MachineInstr &MI, unsigned BaseReg,
                                       int64_t Offset) const {

  MachineBasicBlock *MBB = MI.getParent();
  MachineFunction *MF = MBB->getParent();
  const PPUSubtarget &Subtarget = MF->getSubtarget<PPUSubtarget>();
  const PPUInstrInfo *TII = Subtarget.getInstrInfo();

#ifndef NDEBUG
  // FIXME: Is it possible to be storing a frame index to itself?
  bool SeenFI = false;
  for (const MachineOperand &MO: MI.operands()) {
    if (MO.isFI()) {
      if (SeenFI)
        llvm_unreachable("should not see multiple frame indices");

      SeenFI = true;
    }
  }
#endif
/* FIXME
  MachineOperand *FIOp = TII->getNamedOperand(MI, PPU::OpName::vaddr);
  assert(FIOp && FIOp->isFI() && "frame index must be address operand");
  // FIXME assert(TII->isMUBUF(MI));
  assert(TII->getNamedOperand(MI, PPU::OpName::soffset)->getReg() ==
         MF->getInfo<PPUMachineFunctionInfo>()->getFrameOffsetReg() &&
         "should only be seeing frame offset relative FrameIndex");


  MachineOperand *OffsetOp = TII->getNamedOperand(MI, PPU::OpName::offset);
  int64_t NewOffset = OffsetOp->getImm() + Offset;
  assert(isUInt<12>(NewOffset) && "offset should be legal");

  FIOp->ChangeToRegister(BaseReg, false);
  OffsetOp->setImm(NewOffset);
  */
}
/*
bool PPURegisterInfo::isFrameOffsetLegal(const MachineInstr *MI,
                                        unsigned BaseReg,
                                        int64_t Offset) const {
  if (!PPUInstrInfo::isMUBUF(*MI))
    return false;

  int64_t NewOffset = Offset + getMUBUFInstrOffset(MI);

  return isUInt<12>(NewOffset);
}
*/

const TargetRegisterClass *PPURegisterInfo::getPointerRegClass(
  const MachineFunction &MF, unsigned Kind) const {
  // This is inaccurate. It depends on the instruction and address space. The
  // only place where we should hit this is for dealing with frame indexes /
  // private accesses, so this is correct in that case.
  return &PPU::VPR_32RegClass;
}

static unsigned getNumSubRegsForSpillOp(unsigned Op) {

  switch (Op) {
      /*
  case PPU::SI_SPILL_S1024_SAVE:
  case PPU::SI_SPILL_S1024_RESTORE:
  case PPU::SI_SPILL_V1024_SAVE:
  case PPU::SI_SPILL_V1024_RESTORE:
  case PPU::SI_SPILL_A1024_SAVE:
  case PPU::SI_SPILL_A1024_RESTORE:
    return 32;
  case PPU::SI_SPILL_S512_SAVE:
  case PPU::SI_SPILL_S512_RESTORE:
  case PPU::SI_SPILL_V512_SAVE:
  case PPU::SI_SPILL_V512_RESTORE:
  case PPU::SI_SPILL_A512_SAVE:
  case PPU::SI_SPILL_A512_RESTORE:
    return 16;
  case PPU::SI_SPILL_S256_SAVE:
  case PPU::SI_SPILL_S256_RESTORE:
  case PPU::SI_SPILL_V256_SAVE:
  case PPU::SI_SPILL_V256_RESTORE:
    return 8;
  case PPU::SI_SPILL_S160_SAVE:
  case PPU::SI_SPILL_S160_RESTORE:
  case PPU::SI_SPILL_V160_SAVE:
  case PPU::SI_SPILL_V160_RESTORE:
    return 5;
  case PPU::SI_SPILL_S128_SAVE:
  case PPU::SI_SPILL_S128_RESTORE:
  case PPU::SI_SPILL_V128_SAVE:
  case PPU::SI_SPILL_V128_RESTORE:
  case PPU::SI_SPILL_A128_SAVE:
  case PPU::SI_SPILL_A128_RESTORE:
    return 4;
  case PPU::SI_SPILL_S96_SAVE:
  case PPU::SI_SPILL_S96_RESTORE:
  case PPU::SI_SPILL_V96_SAVE:
  case PPU::SI_SPILL_V96_RESTORE:
    return 3;
  case PPU::SI_SPILL_S64_SAVE:
  case PPU::SI_SPILL_S64_RESTORE:
  case PPU::SI_SPILL_V64_SAVE:
  case PPU::SI_SPILL_V64_RESTORE:
  case PPU::SI_SPILL_A64_SAVE:
  case PPU::SI_SPILL_A64_RESTORE:
    return 2;
  case PPU::SI_SPILL_S32_SAVE:
  case PPU::SI_SPILL_S32_RESTORE:
  case PPU::SI_SPILL_V32_SAVE:
  case PPU::SI_SPILL_V32_RESTORE:
  case PPU::SI_SPILL_A32_SAVE:
  case PPU::SI_SPILL_A32_RESTORE:
  */
    return 1;
  default: llvm_unreachable("Invalid spill opcode");
  }
}
/*
static int getOffsetMUBUFStore(unsigned Opc) {
  switch (Opc) {
  case PPU::BUFFER_STORE_DWORD_OFFEN:
    return PPU::BUFFER_STORE_DWORD_OFFSET;
  case PPU::BUFFER_STORE_BYTE_OFFEN:
    return PPU::BUFFER_STORE_BYTE_OFFSET;
  case PPU::BUFFER_STORE_SHORT_OFFEN:
    return PPU::BUFFER_STORE_SHORT_OFFSET;
  case PPU::BUFFER_STORE_DWORDX2_OFFEN:
    return PPU::BUFFER_STORE_DWORDX2_OFFSET;
  case PPU::BUFFER_STORE_DWORDX4_OFFEN:
    return PPU::BUFFER_STORE_DWORDX4_OFFSET;
  case PPU::BUFFER_STORE_SHORT_D16_HI_OFFEN:
    return PPU::BUFFER_STORE_SHORT_D16_HI_OFFSET;
  case PPU::BUFFER_STORE_BYTE_D16_HI_OFFEN:
    return PPU::BUFFER_STORE_BYTE_D16_HI_OFFSET;
  default:
    return -1;
  }
}

static int getOffsetMUBUFLoad(unsigned Opc) {
  switch (Opc) {
  case PPU::BUFFER_LOAD_DWORD_OFFEN:
    return PPU::BUFFER_LOAD_DWORD_OFFSET;
  case PPU::BUFFER_LOAD_UBYTE_OFFEN:
    return PPU::BUFFER_LOAD_UBYTE_OFFSET;
  case PPU::BUFFER_LOAD_SBYTE_OFFEN:
    return PPU::BUFFER_LOAD_SBYTE_OFFSET;
  case PPU::BUFFER_LOAD_USHORT_OFFEN:
    return PPU::BUFFER_LOAD_USHORT_OFFSET;
  case PPU::BUFFER_LOAD_SSHORT_OFFEN:
    return PPU::BUFFER_LOAD_SSHORT_OFFSET;
  case PPU::BUFFER_LOAD_DWORDX2_OFFEN:
    return PPU::BUFFER_LOAD_DWORDX2_OFFSET;
  case PPU::BUFFER_LOAD_DWORDX4_OFFEN:
    return PPU::BUFFER_LOAD_DWORDX4_OFFSET;
  case PPU::BUFFER_LOAD_UBYTE_D16_OFFEN:
    return PPU::BUFFER_LOAD_UBYTE_D16_OFFSET;
  case PPU::BUFFER_LOAD_UBYTE_D16_HI_OFFEN:
    return PPU::BUFFER_LOAD_UBYTE_D16_HI_OFFSET;
  case PPU::BUFFER_LOAD_SBYTE_D16_OFFEN:
    return PPU::BUFFER_LOAD_SBYTE_D16_OFFSET;
  case PPU::BUFFER_LOAD_SBYTE_D16_HI_OFFEN:
    return PPU::BUFFER_LOAD_SBYTE_D16_HI_OFFSET;
  case PPU::BUFFER_LOAD_SHORT_D16_OFFEN:
    return PPU::BUFFER_LOAD_SHORT_D16_OFFSET;
  case PPU::BUFFER_LOAD_SHORT_D16_HI_OFFEN:
    return PPU::BUFFER_LOAD_SHORT_D16_HI_OFFSET;
  default:
    return -1;
  }
}
*/

/*
// This differs from buildSpillLoadStore by only scavenging a VGPR. It does not
// need to handle the case where an SGPR may need to be spilled while spilling.
static bool buildMUBUFOffsetLoadStore(const PPUInstrInfo *TII,
                                      MachineFrameInfo &MFI,
                                      MachineBasicBlock::iterator MI,
                                      int Index,
                                      int64_t Offset) {
  MachineBasicBlock *MBB = MI->getParent();
  const DebugLoc &DL = MI->getDebugLoc();
  bool IsStore = MI->mayStore();

  unsigned Opc = MI->getOpcode();
  int LoadStoreOp = IsStore ?
    getOffsetMUBUFStore(Opc) : getOffsetMUBUFLoad(Opc);
  if (LoadStoreOp == -1)
    return false;

  const MachineOperand *Reg = TII->getNamedOperand(*MI, PPU::OpName::vdata);
  if (spillVGPRtoAGPR(MI, Index, 0, Reg->getReg(), false).getInstr())
    return true;

  MachineInstrBuilder NewMI =
      BuildMI(*MBB, MI, DL, TII->get(LoadStoreOp))
          .add(*Reg)
          .add(*TII->getNamedOperand(*MI, PPU::OpName::srsrc))
          .add(*TII->getNamedOperand(*MI, PPU::OpName::soffset))
          .addImm(Offset)
          .addImm(0) // glc
          .addImm(0) // slc
          .addImm(0) // tfe
          .addImm(0) // dlc
          .cloneMemRefs(*MI);

  const MachineOperand *VDataIn = TII->getNamedOperand(*MI,
                                                       PPU::OpName::vdata_in);
  if (VDataIn)
    NewMI.add(*VDataIn);
  return true;
}
*/

void PPURegisterInfo::buildSpillLoadStore(MachineBasicBlock::iterator MI,
                                         unsigned LoadStoreOp,
                                         int Index,
                                         unsigned ValueReg,
                                         bool IsKill,
                                         unsigned ScratchRsrcReg,
                                         unsigned ScratchOffsetReg,
                                         int64_t InstOffset,
                                         MachineMemOperand *MMO,
                                         RegScavenger *RS) const {
  MachineBasicBlock *MBB = MI->getParent();
  MachineFunction *MF = MI->getParent()->getParent();
  const PPUSubtarget &ST =  MF->getSubtarget<PPUSubtarget>();
  const PPUInstrInfo *TII = ST.getInstrInfo();
  const MachineFrameInfo &MFI = MF->getFrameInfo();

  const MCInstrDesc &Desc = TII->get(LoadStoreOp);
  const DebugLoc &DL = MI->getDebugLoc();
  bool IsStore = Desc.mayStore();

  bool Scavenged = false;
  unsigned SOffset = ScratchOffsetReg;

  const unsigned EltSize = 4;
  const TargetRegisterClass *RC = getRegClassForReg(MF->getRegInfo(), ValueReg);
  // FIXME unsigned NumSubRegs = PPU::getRegBitWidth(RC->getID()) / (EltSize * CHAR_BIT);
  unsigned NumSubRegs = 0;
  
  unsigned Size = NumSubRegs * EltSize;
  int64_t Offset = InstOffset + MFI.getObjectOffset(Index);
  int64_t ScratchOffsetRegDelta = 0;

  unsigned Align = MFI.getObjectAlignment(Index);
  const MachinePointerInfo &BasePtrInfo = MMO->getPointerInfo();

  Register TmpReg =Register();

  assert((Offset % EltSize) == 0 && "unexpected VGPR spill offset");

  if (!isUInt<12>(Offset + Size - EltSize)) {
    SOffset = PPU::NoRegister;

    // We currently only support spilling VGPRs to EltSize boundaries, meaning
    // we can simplify the adjustment of Offset here to just scale with
    // WavefrontSize.
    Offset *= ST.getWavefrontSize();

    // We don't have access to the register scavenger if this function is called
    // during  PEI::scavengeFrameVirtualRegs().
    if (RS)
      SOffset = RS->scavengeRegister(&PPU::SPR_32RegClass, MI, 0, false);

    if (SOffset == PPU::NoRegister) {
      // There are no free SGPRs, and since we are in the process of spilling
      // VGPRs too.  Since we need a VGPR in order to spill SGPRs (this is true
      // on SI/CI and on VI it is true until we implement spilling using scalar
      // stores), we have no way to free up an SGPR.  Our solution here is to
      // add the offset directly to the ScratchOffset register, and then
      // subtract the offset after the spill to return ScratchOffset to it's
      // original value.
      SOffset = ScratchOffsetReg;
      ScratchOffsetRegDelta = Offset;
    } else {
      Scavenged = true;
    }

    BuildMI(*MBB, MI, DL, TII->get(PPU::ADD), SOffset)
      .addReg(ScratchOffsetReg)
      .addImm(Offset);

    Offset = 0;
  }

  for (unsigned i = 0, e = NumSubRegs; i != e; ++i, Offset += EltSize) {
    Register SubReg = NumSubRegs == 1
                          ? Register(ValueReg)
                          : getSubReg(ValueReg, getSubRegFromChannel(i));

    unsigned SOffsetRegState = 0;
    unsigned SrcDstRegState = getDefRegState(!IsStore);
    if (i + 1 == e) {
      SOffsetRegState |= getKillRegState(Scavenged);
      // The last implicit use carries the "Kill" flag.
      SrcDstRegState |= getKillRegState(IsKill);
    }

  }

  if (ScratchOffsetRegDelta != 0) {
    // Subtract the offset we added to the ScratchOffset register.
    BuildMI(*MBB, MI, DL, TII->get(PPU::SUB), ScratchOffsetReg)
        .addReg(ScratchOffsetReg)
        .addImm(ScratchOffsetRegDelta);
  }
}

static std::pair<unsigned, unsigned> getSpillEltSize(unsigned SuperRegSize,
                                                     bool Store) {
#if 0 
  if (SuperRegSize % 16 == 0) {
    // return { 16, Store ? PPU::S_BUFFER_STORE_DWORDX4_SGPR :
    //                      PPU::S_BUFFER_LOAD_DWORDX4_SGPR };
    return { 16, Store ? PPU::SWX4_GPR :
                         PPU::LWX4_GPR };
  }

  if (SuperRegSize % 8 == 0) {
    return { 8, Store ? PPU::SWX2_GPR :
                        PPU::SWX2_GPR };
  }
#endif
  return { 4, Store ? PPU::SW :
                      PPU::LW};
}

bool PPURegisterInfo::spillSGPR(MachineBasicBlock::iterator MI,
                               int Index,
                               RegScavenger *RS,
                               bool OnlyToVGPR) const {
  MachineBasicBlock *MBB = MI->getParent();
  MachineFunction *MF = MBB->getParent();
  PPUMachineFunctionInfo *MFI = MF->getInfo<PPUMachineFunctionInfo>();
  DenseSet<unsigned> SGPRSpillVGPRDefinedSet;

  ArrayRef<PPUMachineFunctionInfo::SpilledReg> VGPRSpills
    = MFI->getSGPRToVGPRSpills(Index);
  bool SpillToVGPR = !VGPRSpills.empty();
  if (OnlyToVGPR && !SpillToVGPR)
    return false;

  const PPUSubtarget &ST =  MF->getSubtarget<PPUSubtarget>();
  const PPUInstrInfo *TII = ST.getInstrInfo();

  Register SuperReg = MI->getOperand(0).getReg();
  bool IsKill = MI->getOperand(0).isKill();
  const DebugLoc &DL = MI->getDebugLoc();

  MachineFrameInfo &FrameInfo = MF->getFrameInfo();

  bool SpillToSMEM = spillSGPRToSMEM();
  if (SpillToSMEM && OnlyToVGPR)
    return false;

  Register FrameReg = getFrameRegister(*MF);

  assert(SpillToVGPR || (SuperReg != MFI->getStackPtrOffsetReg() &&
                         SuperReg != MFI->getFrameOffsetReg() &&
                         SuperReg != MFI->getScratchWaveOffsetReg()));

  assert(SuperReg != PPU::M0 && "m0 should never spill");

  unsigned OffsetReg = PPU::M0;
  unsigned M0CopyReg = PPU::NoRegister;

  if (SpillToSMEM) {
    if (RS->isRegUsed(PPU::M0)) {
      M0CopyReg = RS->scavengeRegister(&PPU::SReg_32RegClass, MI, 0, false);
      BuildMI(*MBB, MI, DL, TII->get(PPU::COPY), M0CopyReg)
        .addReg(PPU::M0);
    }
  }

  unsigned ScalarStoreOp;
  unsigned EltSize = 4;
  const TargetRegisterClass *RC = getPhysRegClass(SuperReg);
  if (SpillToSMEM && isSGPRClass(RC)) {
    // XXX - if private_element_size is larger than 4 it might be useful to be
    // able to spill wider vmem spills.
    std::tie(EltSize, ScalarStoreOp) =
          getSpillEltSize(getRegSizeInBits(*RC) / 8, true);
  }

  ArrayRef<int16_t> SplitParts = getRegSplitParts(RC, EltSize);
  unsigned NumSubRegs = SplitParts.empty() ? 1 : SplitParts.size();

  // Scavenged temporary VGPR to use. It must be scavenged once for any number
  // of spilled subregs.
  Register TmpVGPR;

  // SubReg carries the "Kill" flag when SubReg == SuperReg.
  unsigned SubKillState = getKillRegState((NumSubRegs == 1) && IsKill);
  for (unsigned i = 0, e = NumSubRegs; i < e; ++i) {
    Register SubReg =
        NumSubRegs == 1 ? SuperReg : getSubReg(SuperReg, SplitParts[i]);

    if (SpillToSMEM) {
      int64_t FrOffset = FrameInfo.getObjectOffset(Index);

      // The allocated memory size is really the wavefront size * the frame
      // index size. The widest register class is 64 bytes, so a 4-byte scratch
      // allocation is enough to spill this in a single stack object.
      //
      // FIXME: Frame size/offsets are computed earlier than this, so the extra
      // space is still unnecessarily allocated.

      unsigned Align = FrameInfo.getObjectAlignment(Index);
      MachinePointerInfo PtrInfo
        = MachinePointerInfo::getFixedStack(*MF, Index, EltSize * i);
      MachineMemOperand *MMO
        = MF->getMachineMemOperand(PtrInfo, MachineMemOperand::MOStore,
                                   EltSize, MinAlign(Align, EltSize * i));

      // SMEM instructions only support a single offset, so increment the wave
      // offset.

      int64_t Offset = (ST.getWavefrontSize() * FrOffset) + (EltSize * i);
      if (Offset != 0) {
        BuildMI(*MBB, MI, DL, TII->get(PPU::ADD), OffsetReg)
          .addReg(FrameReg)
          .addImm(Offset);
      } else {
        BuildMI(*MBB, MI, DL, TII->get(PPU::SMOV), OffsetReg)
          .addReg(FrameReg);
      }

      BuildMI(*MBB, MI, DL, TII->get(ScalarStoreOp))
        .addReg(SubReg, getKillRegState(IsKill)) // sdata
        .addReg(MFI->getScratchRSrcReg())        // sbase
        .addReg(OffsetReg, RegState::Kill)       // soff
        .addImm(0)                               // glc
        .addImm(0)                               // dlc
        .addMemOperand(MMO);

      continue;
    }

    if (SpillToVGPR) {
      PPUMachineFunctionInfo::SpilledReg Spill = VGPRSpills[i];

      // During SGPR spilling to VGPR, determine if the VGPR is defined. The
      // only circumstance in which we say it is undefined is when it is the
      // first spill to this VGPR in the first basic block.
      bool VGPRDefined = true;
      if (MBB == &MF->front())
        VGPRDefined = !SGPRSpillVGPRDefinedSet.insert(Spill.VGPR).second;

      // Mark the "old value of vgpr" input undef only if this is the first sgpr
      // spill to this specific vgpr in the first basic block.
      /* FIXME
      BuildMI(*MBB, MI, DL,
              TII->getMCOpcodeFromPseudo(PPU::VWRITELANE),
              Spill.VGPR)
        .addReg(SubReg, getKillRegState(IsKill))
        .addImm(Spill.Lane)
        .addReg(Spill.VGPR, VGPRDefined ? 0 : RegState::Undef);
        */

      // FIXME: Since this spills to another register instead of an actual
      // frame index, we should delete the frame index when all references to
      // it are fixed.
    } else {
      // XXX - Can to VGPR spill fail for some subregisters but not others?
      if (OnlyToVGPR)
        return false;

      // Spill SGPR to a frame index.
      // TODO: Should VI try to spill to VGPR and then spill to SMEM?
      if (!TmpVGPR.isValid())
        TmpVGPR = RS->scavengeRegister(&PPU::VPR_32RegClass, MI, 0);
      // TODO: Should VI try to spill to VGPR and then spill to SMEM?

      MachineInstrBuilder Mov
        = BuildMI(*MBB, MI, DL, TII->get(PPU::VMOV), TmpVGPR)
        .addReg(SubReg, SubKillState);

      // There could be undef components of a spilled super register.
      // TODO: Can we detect this and skip the spill?
      if (NumSubRegs > 1) {
        // The last implicit use of the SuperReg carries the "Kill" flag.
        unsigned SuperKillState = 0;
        if (i + 1 == e)
          SuperKillState |= getKillRegState(IsKill);
        Mov.addReg(SuperReg, RegState::Implicit | SuperKillState);
      }

      unsigned Align = FrameInfo.getObjectAlignment(Index);
      MachinePointerInfo PtrInfo
        = MachinePointerInfo::getFixedStack(*MF, Index, EltSize * i);
      MachineMemOperand *MMO
        = MF->getMachineMemOperand(PtrInfo, MachineMemOperand::MOStore,
                                   EltSize, MinAlign(Align, EltSize * i));
      /* FIXME  
      BuildMI(*MBB, MI, DL, TII->get(PPU::PPU_SPILL_V32_SAVE))
        .addReg(TmpVGPR, RegState::Kill)      // src
        .addFrameIndex(Index)                 // vaddr
        .addReg(MFI->getScratchRSrcReg())     // srrsrc
        .addReg(MFI->getStackPtrOffsetReg())  // soffset
        .addImm(i * 4)                        // offset
        .addMemOperand(MMO);
        */
    }
  }

  if (M0CopyReg != PPU::NoRegister) {
    BuildMI(*MBB, MI, DL, TII->get(PPU::COPY), PPU::M0)
      .addReg(M0CopyReg, RegState::Kill);
  }

  MI->eraseFromParent();
  MFI->addToSpilledSGPRs(NumSubRegs);
  return true;
}

bool PPURegisterInfo::restoreSGPR(MachineBasicBlock::iterator MI,
                                 int Index,
                                 RegScavenger *RS,
                                 bool OnlyToVGPR) const {
  MachineFunction *MF = MI->getParent()->getParent();
  MachineBasicBlock *MBB = MI->getParent();
  PPUMachineFunctionInfo *MFI = MF->getInfo<PPUMachineFunctionInfo>();

  ArrayRef<PPUMachineFunctionInfo::SpilledReg> VGPRSpills
    = MFI->getSGPRToVGPRSpills(Index);
  bool SpillToVGPR = !VGPRSpills.empty();
  if (OnlyToVGPR && !SpillToVGPR)
    return false;

  MachineFrameInfo &FrameInfo = MF->getFrameInfo();
  const PPUSubtarget &ST =  MF->getSubtarget<PPUSubtarget>();
  const PPUInstrInfo *TII = ST.getInstrInfo();
  const DebugLoc &DL = MI->getDebugLoc();

  Register SuperReg = MI->getOperand(0).getReg();
  bool SpillToSMEM = spillSGPRToSMEM();
  if (SpillToSMEM && OnlyToVGPR)
    return false;

  assert(SuperReg != PPU::M0 && "m0 should never spill");

  unsigned OffsetReg = PPU::M0;
  unsigned M0CopyReg = PPU::NoRegister;

  if (SpillToSMEM) {
    if (RS->isRegUsed(PPU::M0)) {
      M0CopyReg = RS->scavengeRegister(&PPU::SReg_32RegClass, MI, 0, false);
      BuildMI(*MBB, MI, DL, TII->get(PPU::COPY), M0CopyReg)
        .addReg(PPU::M0);
    }
  }

  unsigned EltSize = 4;
  unsigned ScalarLoadOp;

  Register FrameReg = getFrameRegister(*MF);

  const TargetRegisterClass *RC = getPhysRegClass(SuperReg);
  if (SpillToSMEM && isSGPRClass(RC)) {
    // XXX - if private_element_size is larger than 4 it might be useful to be
    // able to spill wider vmem spills.
    std::tie(EltSize, ScalarLoadOp) =
          getSpillEltSize(getRegSizeInBits(*RC) / 8, false);
  }

  ArrayRef<int16_t> SplitParts = getRegSplitParts(RC, EltSize);
  unsigned NumSubRegs = SplitParts.empty() ? 1 : SplitParts.size();

  // SubReg carries the "Kill" flag when SubReg == SuperReg.
  int64_t FrOffset = FrameInfo.getObjectOffset(Index);

  Register TmpVGPR;

  for (unsigned i = 0, e = NumSubRegs; i < e; ++i) {
    Register SubReg =
        NumSubRegs == 1 ? SuperReg : getSubReg(SuperReg, SplitParts[i]);

    if (SpillToSMEM) {
      // FIXME: Size may be > 4 but extra bytes wasted.
      unsigned Align = FrameInfo.getObjectAlignment(Index);
      MachinePointerInfo PtrInfo
        = MachinePointerInfo::getFixedStack(*MF, Index, EltSize * i);
      MachineMemOperand *MMO
        = MF->getMachineMemOperand(PtrInfo, MachineMemOperand::MOLoad,
                                   EltSize, MinAlign(Align, EltSize * i));

      // Add i * 4 offset
      int64_t Offset = (ST.getWavefrontSize() * FrOffset) + (EltSize * i);
      if (Offset != 0) {
        BuildMI(*MBB, MI, DL, TII->get(PPU::ADD), OffsetReg)
          .addReg(FrameReg)
          .addImm(Offset);
      } else {
        BuildMI(*MBB, MI, DL, TII->get(PPU::SMOV), OffsetReg)
          .addReg(FrameReg);
      }
      auto MIB =
        BuildMI(*MBB, MI, DL, TII->get(ScalarLoadOp), SubReg)
        .addReg(MFI->getScratchRSrcReg())  // sbase
        .addReg(OffsetReg, RegState::Kill) // soff
        .addImm(0)                         // glc
        .addImm(0)                         // dlc
        .addMemOperand(MMO);

      if (NumSubRegs > 1 && i == 0)
        MIB.addReg(SuperReg, RegState::ImplicitDefine);
      continue;
    }

    if (SpillToVGPR) {
      PPUMachineFunctionInfo::SpilledReg Spill = VGPRSpills[i];
      auto MIB =
        BuildMI(*MBB, MI, DL, TII->getMCOpcodeFromPseudo(PPU::VREADLANE),
                SubReg)
        .addReg(Spill.VGPR)
        .addImm(Spill.Lane);

      if (NumSubRegs > 1 && i == 0)
        MIB.addReg(SuperReg, RegState::ImplicitDefine);
    } else {
      if (OnlyToVGPR)
        return false;

      // Restore SGPR from a stack slot.
      // FIXME: We should use S_LOAD_DWORD here for VI.
      if (!TmpVGPR.isValid())
        TmpVGPR = RS->scavengeRegister(&PPU::VPR_32RegClass, MI, 0);
      unsigned Align = FrameInfo.getObjectAlignment(Index);

      MachinePointerInfo PtrInfo
        = MachinePointerInfo::getFixedStack(*MF, Index, EltSize * i);

      MachineMemOperand *MMO = MF->getMachineMemOperand(PtrInfo,
        MachineMemOperand::MOLoad, EltSize,
        MinAlign(Align, EltSize * i));
/* FIXME
      BuildMI(*MBB, MI, DL, TII->get(PPU::SI_SPILL_V32_RESTORE), TmpVGPR)
        .addFrameIndex(Index)                 // vaddr
        .addReg(MFI->getScratchRSrcReg())     // srsrc
        .addReg(MFI->getStackPtrOffsetReg())  // soffset
        .addImm(i * 4)                        // offset
        .addMemOperand(MMO);

      auto MIB =
        BuildMI(*MBB, MI, DL, TII->get(PPU::V_READFIRSTLANE_B32), SubReg)
        .addReg(TmpVGPR, RegState::Kill);
      if (NumSubRegs > 1)
        MIB.addReg(MI->getOperand(0).getReg(), RegState::ImplicitDefine);
*/
    }
  }

  if (M0CopyReg != PPU::NoRegister) {
    BuildMI(*MBB, MI, DL, TII->get(PPU::COPY), PPU::M0)
      .addReg(M0CopyReg, RegState::Kill);
  }

  MI->eraseFromParent();

  return true;
}

/// Special case of eliminateFrameIndex. Returns true if the SGPR was spilled to
/// a VGPR and the stack slot can be safely eliminated when all other users are
/// handled.
bool PPURegisterInfo::eliminateSGPRToVGPRSpillFrameIndex(
  MachineBasicBlock::iterator MI,
  int FI,
  RegScavenger *RS) const {
  switch (MI->getOpcode()) {
      /*
  case PPU::SI_SPILL_S1024_SAVE:
  case PPU::SI_SPILL_S512_SAVE:
  case PPU::SI_SPILL_S256_SAVE:
  case PPU::SI_SPILL_S160_SAVE:
  case PPU::SI_SPILL_S128_SAVE:
  case PPU::SI_SPILL_S96_SAVE:
  case PPU::SI_SPILL_S64_SAVE:
  case PPU::SI_SPILL_S32_SAVE:
    return spillSGPR(MI, FI, RS, true);
  case PPU::SI_SPILL_S1024_RESTORE:
  case PPU::SI_SPILL_S512_RESTORE:
  case PPU::SI_SPILL_S256_RESTORE:
  case PPU::SI_SPILL_S160_RESTORE:
  case PPU::SI_SPILL_S128_RESTORE:
  case PPU::SI_SPILL_S96_RESTORE:
  case PPU::SI_SPILL_S64_RESTORE:
  case PPU::SI_SPILL_S32_RESTORE:
    return restoreSGPR(MI, FI, RS, true);
    */
  default:
    llvm_unreachable("not an SGPR spill instruction");
  }
}

void PPURegisterInfo::eliminateFrameIndex(MachineBasicBlock::iterator MI,
                                        int SPAdj, unsigned FIOperandNum,
                                        RegScavenger *RS) const {
  MachineFunction *MF = MI->getParent()->getParent();
  MachineBasicBlock *MBB = MI->getParent();
  PPUMachineFunctionInfo *MFI = MF->getInfo<PPUMachineFunctionInfo>();
  MachineFrameInfo &FrameInfo = MF->getFrameInfo();
  const PPUSubtarget &ST =  MF->getSubtarget<PPUSubtarget>();
  const PPUInstrInfo *TII = ST.getInstrInfo();
  DebugLoc DL = MI->getDebugLoc();

  assert(SPAdj == 0 && "unhandled SP adjustment in call sequence?");

  MachineOperand &FIOp = MI->getOperand(FIOperandNum);
  int Index = MI->getOperand(FIOperandNum).getIndex();

  Register FrameReg = getFrameRegister(*MF);

  switch (MI->getOpcode()) {
    // SGPR register spill
    /*
    case PPU::SI_SPILL_S1024_SAVE:
    case PPU::SI_SPILL_S512_SAVE:
    case PPU::SI_SPILL_S256_SAVE:
    case PPU::SI_SPILL_S160_SAVE:
    case PPU::SI_SPILL_S128_SAVE:
    case PPU::SI_SPILL_S96_SAVE:
    case PPU::SI_SPILL_S64_SAVE:
    case PPU::SI_SPILL_S32_SAVE: {
      spillSGPR(MI, Index, RS);
      break;
    }

    // SGPR register restore
    case PPU::SI_SPILL_S1024_RESTORE:
    case PPU::SI_SPILL_S512_RESTORE:
    case PPU::SI_SPILL_S256_RESTORE:
    case PPU::SI_SPILL_S160_RESTORE:
    case PPU::SI_SPILL_S128_RESTORE:
    case PPU::SI_SPILL_S96_RESTORE:
    case PPU::SI_SPILL_S64_RESTORE:
    case PPU::SI_SPILL_S32_RESTORE: {
      restoreSGPR(MI, Index, RS);
      break;
    }

    // VGPR register spill
    case PPU::SI_SPILL_V1024_SAVE:
    case PPU::SI_SPILL_V512_SAVE:
    case PPU::SI_SPILL_V256_SAVE:
    case PPU::SI_SPILL_V160_SAVE:
    case PPU::SI_SPILL_V128_SAVE:
    case PPU::SI_SPILL_V96_SAVE:
    case PPU::SI_SPILL_V64_SAVE:
    case PPU::SI_SPILL_V32_SAVE:
    case PPU::SI_SPILL_A1024_SAVE:
    case PPU::SI_SPILL_A512_SAVE:
    case PPU::SI_SPILL_A128_SAVE:
    case PPU::SI_SPILL_A64_SAVE:
    case PPU::SI_SPILL_A32_SAVE: {
      const MachineOperand *VData = TII->getNamedOperand(*MI,
                                                         PPU::OpName::vdata);
      assert(TII->getNamedOperand(*MI, PPU::OpName::soffset)->getReg() ==
             MFI->getStackPtrOffsetReg());

      buildSpillLoadStore(MI, PPU::BUFFER_STORE_DWORD_OFFSET,
            Index,
            VData->getReg(), VData->isKill(),
            TII->getNamedOperand(*MI, PPU::OpName::srsrc)->getReg(),
            FrameReg,
            TII->getNamedOperand(*MI, PPU::OpName::offset)->getImm(),
            *MI->memoperands_begin(),
            RS);
      MFI->addToSpilledVGPRs(getNumSubRegsForSpillOp(MI->getOpcode()));
      MI->eraseFromParent();
      break;
    }
    case PPU::SI_SPILL_V32_RESTORE:
    case PPU::SI_SPILL_V64_RESTORE:
    case PPU::SI_SPILL_V96_RESTORE:
    case PPU::SI_SPILL_V128_RESTORE:
    case PPU::SI_SPILL_V160_RESTORE:
    case PPU::SI_SPILL_V256_RESTORE:
    case PPU::SI_SPILL_V512_RESTORE:
    case PPU::SI_SPILL_V1024_RESTORE:
    case PPU::SI_SPILL_A32_RESTORE:
    case PPU::SI_SPILL_A64_RESTORE:
    case PPU::SI_SPILL_A128_RESTORE:
    case PPU::SI_SPILL_A512_RESTORE:
    case PPU::SI_SPILL_A1024_RESTORE: {
      const MachineOperand *VData = TII->getNamedOperand(*MI,
                                                         PPU::OpName::vdata);
      assert(TII->getNamedOperand(*MI, PPU::OpName::soffset)->getReg() ==
             MFI->getStackPtrOffsetReg());

      buildSpillLoadStore(MI, PPU::BUFFER_LOAD_DWORD_OFFSET,
            Index,
            VData->getReg(), VData->isKill(),
            TII->getNamedOperand(*MI, PPU::OpName::srsrc)->getReg(),
            FrameReg,
            TII->getNamedOperand(*MI, PPU::OpName::offset)->getImm(),
            *MI->memoperands_begin(),
            RS);
      MI->eraseFromParent();
      break;
    }
  */

    default: {
      const DebugLoc &DL = MI->getDebugLoc();
      bool IsMUBUF = TII->isMUBUF(*MI);

      if (!IsMUBUF && !MFI->isEntryFunction()) {
        // Convert to an absolute stack address by finding the offset from the
        // scratch wave base and scaling by the wave size.
        //
        // In an entry function/kernel the offset is already the absolute
        // address relative to the frame register.

        Register TmpDiffReg =
          RS->scavengeRegister(&PPU::SReg_32RegClass, MI, 0, false);

        // If there's no free SGPR, in-place modify the FP
        Register DiffReg = TmpDiffReg.isValid() ? TmpDiffReg : FrameReg;

        bool IsCopy = MI->getOpcode() == PPU::VMOV;
        Register ResultReg = IsCopy ?
          MI->getOperand(0).getReg() :
          RS->scavengeRegister(&PPU::VPR_32RegClass, MI, 0);
/*
        BuildMI(*MBB, MI, DL, TII->get(PPU::SUB), DiffReg)
          .addReg(FrameReg)
          .addReg(MFI->getScratchWaveOffsetReg());
*/

        int64_t Offset = FrameInfo.getObjectOffset(Index);
        if (Offset == 0) {
          // XXX - This never happens because of emergency scavenging slot at 0?
          /*
          BuildMI(*MBB, MI, DL, TII->get(PPU::V_LSHRREV_B32_e64), ResultReg)
            .addImm(Log2_32(ST.getWavefrontSize()))
            .addReg(DiffReg);
            */
        } else {
          Register ScaledReg =
            RS->scavengeRegister(&PPU::VPR_32RegClass, MI, 0);

          // FIXME: Assusmed VGPR use.
          /*
          BuildMI(*MBB, MI, DL, TII->get(PPU::V_LSHRREV_B32_e64), ScaledReg)
            .addImm(Log2_32(ST.getWavefrontSize()))
            .addReg(DiffReg, RegState::Kill);
            */

          // TODO: Fold if use instruction is another add of a constant.
          /*
          if (PPU::isInlinableLiteral32(Offset, ST.hasInv2PiInlineImm())) {

            // FIXME: This can fail
            TII->getAddNoCarry(*MBB, MI, DL, ResultReg, *RS)
              .addImm(Offset)
              .addReg(ScaledReg, RegState::Kill)
              .addImm(0); // clamp bit
          } else {
            Register ConstOffsetReg =
              RS->scavengeRegister(&PPU::SReg_32_XM0RegClass, MI, 0, false);

            BuildMI(*MBB, MI, DL, TII->get(PPU::SMOV), ConstOffsetReg)
              .addImm(Offset);
            TII->getAddNoCarry(*MBB, MI, DL, ResultReg, *RS)
              .addReg(ConstOffsetReg, RegState::Kill)
              .addReg(ScaledReg, RegState::Kill)
              .addImm(0); // clamp bit
          }
          */
        }
/*
        if (!TmpDiffReg.isValid()) {
          // Restore the FP.
          BuildMI(*MBB, MI, DL, TII->get(PPU::ADD), FrameReg)
            .addReg(FrameReg)
            .addReg(MFI->getScratchWaveOffsetReg());
        }
        */

        // Don't introduce an extra copy if we're just materializing in a mov.
        if (IsCopy)
          MI->eraseFromParent();
        else
          FIOp.ChangeToRegister(ResultReg, false, false, true);
        return;
      }
/*
      if (IsMUBUF) {
        // Disable offen so we don't need a 0 vgpr base.
        assert(static_cast<int>(FIOperandNum) ==
               PPU::getNamedOperandIdx(MI->getOpcode(),
                                          PPU::OpName::vaddr));

        assert(TII->getNamedOperand(*MI, PPU::OpName::soffset)->getReg() ==
               MFI->getStackPtrOffsetReg());

        TII->getNamedOperand(*MI, PPU::OpName::soffset)->setReg(FrameReg);

        int64_t Offset = FrameInfo.getObjectOffset(Index);
        int64_t OldImm
          = TII->getNamedOperand(*MI, PPU::OpName::offset)->getImm();
        int64_t NewOffset = OldImm + Offset;

        if (isUInt<12>(NewOffset) &&
            buildMUBUFOffsetLoadStore(TII, FrameInfo, MI, Index, NewOffset)) {
          MI->eraseFromParent();
          return;
        }
      }
*/
      // If the offset is simply too big, don't convert to a scratch wave offset
      // relative index.

      int64_t Offset = FrameInfo.getObjectOffset(Index);
      FIOp.ChangeToImmediate(Offset);
      if (!TII->isImmOperandLegal(*MI, FIOperandNum, FIOp)) {
        Register TmpReg = RS->scavengeRegister(&PPU::VPR_32RegClass, MI, 0);
        BuildMI(*MBB, MI, DL, TII->get(PPU::VMOV), TmpReg)
          .addImm(Offset);
        FIOp.ChangeToRegister(TmpReg, false, false, true);
      }
    }
  }
}

StringRef PPURegisterInfo::getRegAsmName(unsigned Reg) const {
  return PPUInstPrinter::getRegisterName(Reg);
}

// FIXME: This is very slow. It might be worth creating a map from physreg to
// register class.
const TargetRegisterClass *PPURegisterInfo::getPhysRegClass(unsigned Reg) const {
  assert(!Register::isVirtualRegister(Reg));

  static const TargetRegisterClass *const BaseClasses[] = {
    &PPU::VPR_32RegClass,
    &PPU::SPR_32RegClass,
    &PPU::VReg_64RegClass,
    &PPU::SReg_64RegClass,
    /*
    &PPU::AReg_64RegClass,
    &PPU::VReg_96RegClass,
    &PPU::SReg_96RegClass,
    &PPU::VReg_128RegClass,
    &PPU::SReg_128RegClass,
    &PPU::AReg_128RegClass,
    &PPU::VReg_160RegClass,
    &PPU::SReg_160RegClass,
    &PPU::VReg_256RegClass,
    &PPU::SReg_256RegClass,
    &PPU::VReg_512RegClass,
    &PPU::SReg_512RegClass,
    &PPU::AReg_512RegClass,
    &PPU::SReg_1024RegClass,
    &PPU::VReg_1024RegClass,
    &PPU::AReg_1024RegClass,
    &PPU::SCC_CLASSRegClass,
    &PPU::Pseudo_SReg_32RegClass,
    &PPU::Pseudo_SReg_128RegClass,
    */
  };

  for (const TargetRegisterClass *BaseClass : BaseClasses) {
    if (BaseClass->contains(Reg)) {
      return BaseClass;
    }
  }
  return nullptr;
}

// TODO: It might be helpful to have some target specific flags in
// TargetRegisterClass to mark which classes are VGPRs to make this trivial.
bool PPURegisterInfo::hasVGPRs(const TargetRegisterClass *RC) const {
  unsigned Size = getRegSizeInBits(*RC);
  if (Size < 32)
    return false;
  switch (Size) {
  case 32:
    return getCommonSubClass(&PPU::VPR_32RegClass, RC) != nullptr;
  case 64:
    return getCommonSubClass(&PPU::VReg_64RegClass, RC) != nullptr;
    /*
  case 96:
    return getCommonSubClass(&PPU::VReg_96RegClass, RC) != nullptr;
  case 128:
    return getCommonSubClass(&PPU::VReg_128RegClass, RC) != nullptr;
  case 160:
    return getCommonSubClass(&PPU::VReg_160RegClass, RC) != nullptr;
  case 256:
    return getCommonSubClass(&PPU::VReg_256RegClass, RC) != nullptr;
  case 512:
    return getCommonSubClass(&PPU::VReg_512RegClass, RC) != nullptr;
  case 1024:
    return getCommonSubClass(&PPU::VReg_1024RegClass, RC) != nullptr;
    */
  default:
    llvm_unreachable("Invalid register class size");
  }
}

const TargetRegisterClass *PPURegisterInfo::getEquivalentVGPRClass(
                                         const TargetRegisterClass *SRC) const {
  switch (getRegSizeInBits(*SRC)) {
  case 32:
    return &PPU::VPR_32RegClass;
  case 64:
    return &PPU::VReg_64RegClass;
    /*
  case 96:
    return &PPU::VReg_96RegClass;
  case 128:
    return &PPU::VReg_128RegClass;
  case 160:
    return &PPU::VReg_160RegClass;
  case 256:
    return &PPU::VReg_256RegClass;
  case 512:
    return &PPU::VReg_512RegClass;
  case 1024:
    return &PPU::VReg_1024RegClass;
    */
  default:
    llvm_unreachable("Invalid register class size");
  }
}

const TargetRegisterClass *PPURegisterInfo::getEquivalentSGPRClass(
                                         const TargetRegisterClass *VRC) const {
  switch (getRegSizeInBits(*VRC)) {
  case 32:
    return &PPU::SPR_32RegClass;
    /*
  case 64:
    return &PPU::SReg_64RegClass;
  case 96:
    return &PPU::SReg_96RegClass;
  case 128:
    return &PPU::SReg_128RegClass;
  case 160:
    return &PPU::SReg_160RegClass;
  case 256:
    return &PPU::SReg_256RegClass;
  case 512:
    return &PPU::SReg_512RegClass;
  case 1024:
    return &PPU::SReg_1024RegClass;
    */
  default:
    llvm_unreachable("Invalid register class size");
  }
}

const TargetRegisterClass *PPURegisterInfo::getSubRegClass(
                         const TargetRegisterClass *RC, unsigned SubIdx) const {
  if (SubIdx == PPU::NoSubRegister)
    return RC;

  // We can assume that each lane corresponds to one 32-bit register.
  unsigned Count = getSubRegIndexLaneMask(SubIdx).getNumLanes();
  if (isSGPRClass(RC)) {
    switch (Count) {
    case 1:
      return &PPU::SPR_32RegClass;
      /*
    case 2:
      return &PPU::SReg_64RegClass;
    case 3:
      return &PPU::SReg_96RegClass;
    case 4:
      return &PPU::SReg_128RegClass;
    case 5:
      return &PPU::SReg_160RegClass;
    case 8:
      return &PPU::SReg_256RegClass;
    case 16:
      return &PPU::SReg_512RegClass;
    case 32: // fall-through
      */
    default:
      llvm_unreachable("Invalid sub-register class size");
    }
  } else {
    switch (Count) {
    case 1:
      return &PPU::VPR_32RegClass;
      /*
    case 2:
      return &PPU::VReg_64RegClass;
    case 3:
      return &PPU::VReg_96RegClass;
    case 4:
      return &PPU::VReg_128RegClass;
    case 5:
      return &PPU::VReg_160RegClass;
    case 8:
      return &PPU::VReg_256RegClass;
    case 16:
      return &PPU::VReg_512RegClass;
    case 32:
      */
    default:
      llvm_unreachable("Invalid sub-register class size");
    }
  }
}

bool PPURegisterInfo::opCanUseInlineConstant(unsigned OpType) const {
    /*
  if (OpType >= PPU::OPERAND_REG_INLINE_AC_FIRST &&
      OpType <= PPU::OPERAND_REG_INLINE_AC_LAST)
    return !ST.hasMFMAInlineLiteralBug();

  return OpType >= PPU::OPERAND_SRC_FIRST &&
         OpType <= PPU::OPERAND_SRC_LAST;
         */
    return false;
}

bool PPURegisterInfo::shouldRewriteCopySrc(
  const TargetRegisterClass *DefRC,
  unsigned DefSubReg,
  const TargetRegisterClass *SrcRC,
  unsigned SrcSubReg) const {
  // We want to prefer the smallest register class possible, so we don't want to
  // stop and rewrite on anything that looks like a subregister
  // extract. Operations mostly don't care about the super register class, so we
  // only want to stop on the most basic of copies between the same register
  // class.
  //
  // e.g. if we have something like
  // %0 = ...
  // %1 = ...
  // %2 = REG_SEQUENCE %0, sub0, %1, sub1, %2, sub2
  // %3 = COPY %2, sub0
  //
  // We want to look through the COPY to find:
  //  => %3 = COPY %0

  // Plain copy.
  return getCommonSubClass(DefRC, SrcRC) != nullptr;
}

/// Returns a register that is not used at any point in the function.
///        If all registers are used, then this function will return
//         PPU::NoRegister.
unsigned
PPURegisterInfo::findUnusedRegister(const MachineRegisterInfo &MRI,
                                   const TargetRegisterClass *RC,
                                   const MachineFunction &MF) const {

  for (unsigned Reg : *RC)
    if (MRI.isAllocatable(Reg) && !MRI.isPhysRegUsed(Reg))
      return Reg;
  return PPU::NoRegister;
}

ArrayRef<int16_t> PPURegisterInfo::getRegSplitParts(const TargetRegisterClass *RC,
                                                   unsigned EltSize) const {
    /*
  if (EltSize == 4) {
    static const int16_t Sub0_31[] = {
      PPU::sub0, PPU::sub1, PPU::sub2, PPU::sub3,
      PPU::sub4, PPU::sub5, PPU::sub6, PPU::sub7,
      PPU::sub8, PPU::sub9, PPU::sub10, PPU::sub11,
      PPU::sub12, PPU::sub13, PPU::sub14, PPU::sub15,
      PPU::sub16, PPU::sub17, PPU::sub18, PPU::sub19,
      PPU::sub20, PPU::sub21, PPU::sub22, PPU::sub23,
      PPU::sub24, PPU::sub25, PPU::sub26, PPU::sub27,
      PPU::sub28, PPU::sub29, PPU::sub30, PPU::sub31,
    };

    static const int16_t Sub0_15[] = {
      PPU::sub0, PPU::sub1, PPU::sub2, PPU::sub3,
      PPU::sub4, PPU::sub5, PPU::sub6, PPU::sub7,
      PPU::sub8, PPU::sub9, PPU::sub10, PPU::sub11,
      PPU::sub12, PPU::sub13, PPU::sub14, PPU::sub15,
    };

    static const int16_t Sub0_7[] = {
      PPU::sub0, PPU::sub1, PPU::sub2, PPU::sub3,
      PPU::sub4, PPU::sub5, PPU::sub6, PPU::sub7,
    };

    static const int16_t Sub0_4[] = {
      PPU::sub0, PPU::sub1, PPU::sub2, PPU::sub3, PPU::sub4,
    };

    static const int16_t Sub0_3[] = {
      PPU::sub0, PPU::sub1, PPU::sub2, PPU::sub3,
    };

    static const int16_t Sub0_2[] = {
      PPU::sub0, PPU::sub1, PPU::sub2,
    };

    static const int16_t Sub0_1[] = {
      PPU::sub0, PPU::sub1,
    };

    switch (PPU::getRegBitWidth(*RC->MC)) {
    case 32:
      return {};
    case 64:
      return makeArrayRef(Sub0_1);
    case 96:
      return makeArrayRef(Sub0_2);
    case 128:
      return makeArrayRef(Sub0_3);
    case 160:
      return makeArrayRef(Sub0_4);
    case 256:
      return makeArrayRef(Sub0_7);
    case 512:
      return makeArrayRef(Sub0_15);
    case 1024:
      return makeArrayRef(Sub0_31);
    default:
      llvm_unreachable("unhandled register size");
    }
  }

  if (EltSize == 8) {
    static const int16_t Sub0_31_64[] = {
      PPU::sub0_sub1, PPU::sub2_sub3,
      PPU::sub4_sub5, PPU::sub6_sub7,
      PPU::sub8_sub9, PPU::sub10_sub11,
      PPU::sub12_sub13, PPU::sub14_sub15,
      PPU::sub16_sub17, PPU::sub18_sub19,
      PPU::sub20_sub21, PPU::sub22_sub23,
      PPU::sub24_sub25, PPU::sub26_sub27,
      PPU::sub28_sub29, PPU::sub30_sub31
    };

    static const int16_t Sub0_15_64[] = {
      PPU::sub0_sub1, PPU::sub2_sub3,
      PPU::sub4_sub5, PPU::sub6_sub7,
      PPU::sub8_sub9, PPU::sub10_sub11,
      PPU::sub12_sub13, PPU::sub14_sub15
    };

    static const int16_t Sub0_7_64[] = {
      PPU::sub0_sub1, PPU::sub2_sub3,
      PPU::sub4_sub5, PPU::sub6_sub7
    };


    static const int16_t Sub0_3_64[] = {
      PPU::sub0_sub1, PPU::sub2_sub3
    };

    switch (PPU::getRegBitWidth(*RC->MC)) {
    case 64:
      return {};
    case 128:
      return makeArrayRef(Sub0_3_64);
    case 256:
      return makeArrayRef(Sub0_7_64);
    case 512:
      return makeArrayRef(Sub0_15_64);
    case 1024:
      return makeArrayRef(Sub0_31_64);
    default:
      llvm_unreachable("unhandled register size");
    }
  }

  if (EltSize == 16) {

    static const int16_t Sub0_31_128[] = {
      PPU::sub0_sub1_sub2_sub3,
      PPU::sub4_sub5_sub6_sub7,
      PPU::sub8_sub9_sub10_sub11,
      PPU::sub12_sub13_sub14_sub15,
      PPU::sub16_sub17_sub18_sub19,
      PPU::sub20_sub21_sub22_sub23,
      PPU::sub24_sub25_sub26_sub27,
      PPU::sub28_sub29_sub30_sub31
    };

    static const int16_t Sub0_15_128[] = {
      PPU::sub0_sub1_sub2_sub3,
      PPU::sub4_sub5_sub6_sub7,
      PPU::sub8_sub9_sub10_sub11,
      PPU::sub12_sub13_sub14_sub15
    };

    static const int16_t Sub0_7_128[] = {
      PPU::sub0_sub1_sub2_sub3,
      PPU::sub4_sub5_sub6_sub7
    };

    switch (PPU::getRegBitWidth(*RC->MC)) {
    case 128:
      return {};
    case 256:
      return makeArrayRef(Sub0_7_128);
    case 512:
      return makeArrayRef(Sub0_15_128);
    case 1024:
      return makeArrayRef(Sub0_31_128);
    default:
      llvm_unreachable("unhandled register size");
    }
  }

  assert(EltSize == 32 && "unhandled elt size");

  static const int16_t Sub0_31_256[] = {
    PPU::sub0_sub1_sub2_sub3_sub4_sub5_sub6_sub7,
    PPU::sub8_sub9_sub10_sub11_sub12_sub13_sub14_sub15,
    PPU::sub16_sub17_sub18_sub19_sub20_sub21_sub22_sub23,
    PPU::sub24_sub25_sub26_sub27_sub28_sub29_sub30_sub31
  };

  static const int16_t Sub0_15_256[] = {
    PPU::sub0_sub1_sub2_sub3_sub4_sub5_sub6_sub7,
    PPU::sub8_sub9_sub10_sub11_sub12_sub13_sub14_sub15
  };

  switch (PPU::getRegBitWidth(*RC->MC)) {
  case 256:
    return {};
  case 512:
    return makeArrayRef(Sub0_15_256);
  case 1024:
    return makeArrayRef(Sub0_31_256);
  default:
    llvm_unreachable("unhandled register size");
  }
  */
}

const TargetRegisterClass*
PPURegisterInfo::getRegClassForReg(const MachineRegisterInfo &MRI,
                                  unsigned Reg) const {
  if (Register::isVirtualRegister(Reg))
    return  MRI.getRegClass(Reg);

  return getPhysRegClass(Reg);
}

bool PPURegisterInfo::isVGPR(const MachineRegisterInfo &MRI,
                            unsigned Reg) const {
  const TargetRegisterClass * RC = getRegClassForReg(MRI, Reg);
  assert(RC && "Register class for the reg not found");
  return hasVGPRs(RC);
}


bool PPURegisterInfo::shouldCoalesce(MachineInstr *MI,
                                    const TargetRegisterClass *SrcRC,
                                    unsigned SubReg,
                                    const TargetRegisterClass *DstRC,
                                    unsigned DstSubReg,
                                    const TargetRegisterClass *NewRC,
                                    LiveIntervals &LIS) const {
  unsigned SrcSize = getRegSizeInBits(*SrcRC);
  unsigned DstSize = getRegSizeInBits(*DstRC);
  unsigned NewSize = getRegSizeInBits(*NewRC);

  // Do not increase size of registers beyond dword, we would need to allocate
  // adjacent registers and constraint regalloc more than needed.

  // Always allow dword coalescing.
  if (SrcSize <= 32 || DstSize <= 32)
    return true;

  return NewSize <= DstSize || NewSize <= SrcSize;
}

unsigned PPURegisterInfo::getRegPressureLimit(const TargetRegisterClass *RC,
                                             MachineFunction &MF) const {

  const PPUSubtarget &ST = MF.getSubtarget<PPUSubtarget>();
  const PPUMachineFunctionInfo *MFI = MF.getInfo<PPUMachineFunctionInfo>();

  unsigned Occupancy = ST.getOccupancyWithLocalMemSize(MFI->getLDSSize(),
                                                       MF.getFunction());
  switch (RC->getID()) {
  default:
    return PPUBaseRegisterInfo::getRegPressureLimit(RC, MF);
  case PPU::TPRRegClassID:
    return std::min(ST.getMaxNumVGPRs(Occupancy), ST.getMaxNumVGPRs(MF));
  case PPU::GPRRegClassID:
    return std::min(ST.getMaxNumSGPRs(Occupancy, true), ST.getMaxNumSGPRs(MF));
  }
}

unsigned PPURegisterInfo::getRegPressureSetLimit(const MachineFunction &MF,
                                                unsigned Idx) const {
  if (Idx == getVGPRPressureSet())
    return getRegPressureLimit(&PPU::VPR_32RegClass,
                               const_cast<MachineFunction &>(MF));

  if (Idx == getSGPRPressureSet())
    return getRegPressureLimit(&PPU::SPR_32RegClass,
                               const_cast<MachineFunction &>(MF));

  return PPUBaseRegisterInfo::getRegPressureSetLimit(MF, Idx);
}

const int *PPURegisterInfo::getRegUnitPressureSets(unsigned RegUnit) const {
  static const int Empty[] = { -1 };
/*
  if (hasRegUnit(PPU::M0, RegUnit))
    return Empty;
    */
  return PPUBaseRegisterInfo::getRegUnitPressureSets(RegUnit);
}

unsigned PPURegisterInfo::getReturnAddressReg(const MachineFunction &MF) const {
  // Not a callee saved register.
  /*
  return PPU::SGPR30_SGPR31;
  */
    return PPU::X1;
}

const TargetRegisterClass *
PPURegisterInfo::getRegClassForSizeOnBank(unsigned Size,
                                         const RegisterBank &RB,
                                         const MachineRegisterInfo &MRI) const {
  switch (Size) {
  case 1: {
    switch (RB.getID()) {
    case PPU::GPRRegBankID:
      return &PPU::SReg_32RegClass;
      /*
    case PPU::VRRegBankID:
      return &PPU::VPR_32RegClass;
    case PPU::VCCRegBankID:
      return &PPU::SReg_32_XM0_XEXECRegClass 
    case PPU::SCCRegBankID:
      // This needs to return an allocatable class, so don't bother returning
      // the dummy SCC class.
      return &PPU::SReg_32_XM0RegClass;
      */
    default:
      llvm_unreachable("unknown register bank");
    }
  }
    /*
  case 32:
    return RB.getID() == PPU::VRRegBankID ? &PPU::VPR_32RegClass :
                                                 &PPU::SReg_32_XM0RegClass;
  case 64:
    return RB.getID() == PPU::VGPRRegBankID ? &PPU::VReg_64RegClass :
                                                 &PPU::SReg_64_XEXECRegClass;
  case 96:
    return RB.getID() == PPU::VGPRRegBankID ? &PPU::VReg_96RegClass :
                                                 &PPU::SReg_96RegClass;
  case 128:
    return RB.getID() == PPU::VGPRRegBankID ? &PPU::VReg_128RegClass :
                                                 &PPU::SReg_128RegClass;
  case 160:
    return RB.getID() == PPU::VGPRRegBankID ? &PPU::VReg_160RegClass :
                                                 &PPU::SReg_160RegClass;
  case 256:
    return RB.getID() == PPU::VGPRRegBankID ? &PPU::VReg_256RegClass :
                                                 &PPU::SReg_256RegClass;
  case 512:
    return RB.getID() == PPU::VGPRRegBankID ? &PPU::VReg_512RegClass :
                                                 &PPU::SReg_512RegClass;
                                                 */
  default:
          /*
    if (Size < 32)
      return RB.getID() == PPU::VRRegBankID ? &PPU::VPR_32RegClass :
                                                   &PPU::SReg_32_XM0RegClass;
                                                   */
    return nullptr;
  }
}

const TargetRegisterClass *
PPURegisterInfo::getConstrainedRegClassForOperand(const MachineOperand &MO,
                                         const MachineRegisterInfo &MRI) const {
  if (const RegisterBank *RB = MRI.getRegBankOrNull(MO.getReg()))
    return getRegClassForTypeOnBank(MRI.getType(MO.getReg()), *RB, MRI);
  return nullptr;
}

unsigned PPURegisterInfo::getVCC() const {
  return PPU::VCC;
}

const TargetRegisterClass *
PPURegisterInfo::getRegClass(unsigned RCID) const {
  switch ((int)RCID) {
  case PPU::SReg_1RegClassID:
    return getBoolRC();
    /* FIXME
  case PPU::SReg_1_RegClassID:
    return &PPU::SReg_32RegClass;
    */
  case -1:
    return nullptr;
  default:
    return PPUBaseRegisterInfo::getRegClass(RCID);
  }
}

// Find reaching register definition
/*
MachineInstr *PPURegisterInfo::findReachingDef(unsigned Reg, unsigned SubReg,
                                              MachineInstr &Use,
                                              MachineRegisterInfo &MRI,
                                              LiveIntervals *LIS) const {
  auto &MDT = LIS->getAnalysis<MachineDominatorTree>();
  SlotIndex UseIdx = LIS->getInstructionIndex(Use);
  SlotIndex DefIdx;

  if (Register::isVirtualRegister(Reg)) {
    if (!LIS->hasInterval(Reg))
      return nullptr;
    LiveInterval &LI = LIS->getInterval(Reg);
    LaneBitmask SubLanes = SubReg ? getSubRegIndexLaneMask(SubReg)
                                  : MRI.getMaxLaneMaskForVReg(Reg);
    VNInfo *V = nullptr;
    if (LI.hasSubRanges()) {
      for (auto &S : LI.subranges()) {
        if ((S.LaneMask & SubLanes) == SubLanes) {
          V = S.getVNInfoAt(UseIdx);
          break;
        }
      }
    } else {
      V = LI.getVNInfoAt(UseIdx);
    }
    if (!V)
      return nullptr;
    DefIdx = V->def;
  } else {
    // Find last def.
    for (MCRegUnitIterator Units(Reg, this); Units.isValid(); ++Units) {
      LiveRange &LR = LIS->getRegUnit(*Units);
      if (VNInfo *V = LR.getVNInfoAt(UseIdx)) {
        if (!DefIdx.isValid() ||
            MDT.dominates(LIS->getInstructionFromIndex(DefIdx),
                          LIS->getInstructionFromIndex(V->def)))
          DefIdx = V->def;
      } else {
        return nullptr;
      }
    }
  }

  MachineInstr *Def = LIS->getInstructionFromIndex(DefIdx);

  if (!Def || !MDT.dominates(Def, &Use))
    return nullptr;

  assert(Def->modifiesRegister(Reg, this));

  return Def;
}
*/

