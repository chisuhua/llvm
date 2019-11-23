//===-- PPUInstrInfo.cpp - PPU Instruction Information ------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the PPU implementation of the TargetInstrInfo class.
//
//===----------------------------------------------------------------------===//

#include "PPUInstrInfo.h"
#include "PPU.h"
#include "PPUSubtarget.h"
#include "PPUTargetMachine.h"
#include "PPUMachineFunctionInfo.h"
#include "PPUArgumentUsageInfo.h"
#include "MCTargetDesc/PPUMCTargetDesc.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/RegisterScavenging.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/TargetRegistry.h"
#include "llvm/IR/DiagnosticInfo.h"

#define GET_INSTRINFO_CTOR_DTOR
#include "PPUGenInstrInfo.inc"

using namespace llvm;

PPUBaseInstrInfo::PPUBaseInstrInfo()
    : PPUGenInstrInfo(PPU::ADJCALLSTACKDOWN, PPU::ADJCALLSTACKUP) {}

unsigned PPUBaseInstrInfo::isLoadFromStackSlot(const MachineInstr &MI,
                                             int &FrameIndex) const {
  switch (MI.getOpcode()) {
  default:
    return 0;
  case PPU::LB:
  case PPU::LBU:
  case PPU::LH:
  case PPU::LHU:
  case PPU::LW:
  case PPU::FLW:
  case PPU::LWU:
  case PPU::LD:
  case PPU::FLD:
    break;
  }

  if (MI.getOperand(1).isFI() && MI.getOperand(2).isImm() &&
      MI.getOperand(2).getImm() == 0) {
    FrameIndex = MI.getOperand(1).getIndex();
    return MI.getOperand(0).getReg();
  }

  return 0;
}

unsigned PPUBaseInstrInfo::isStoreToStackSlot(const MachineInstr &MI,
                                            int &FrameIndex) const {
  switch (MI.getOpcode()) {
  default:
    return 0;
  case PPU::SB:
  case PPU::SH:
  case PPU::SW:
  case PPU::FSW:
  case PPU::SD:
  case PPU::FSD:
    break;
  }

  if (MI.getOperand(0).isFI() && MI.getOperand(1).isImm() &&
      MI.getOperand(1).getImm() == 0) {
    FrameIndex = MI.getOperand(0).getIndex();
    return MI.getOperand(2).getReg();
  }

  return 0;
}

void PPUBaseInstrInfo::copyPhysReg(MachineBasicBlock &MBB,
                                 MachineBasicBlock::iterator MBBI,
                                 const DebugLoc &DL, unsigned DstReg,
                                 unsigned SrcReg, bool KillSrc) const {
  if (PPU::GPRRegClass.contains(DstReg, SrcReg)) {
    BuildMI(MBB, MBBI, DL, get(PPU::ADDI), DstReg)
        .addReg(SrcReg, getKillRegState(KillSrc))
        .addImm(0);
    return;
  }

  // GPR -> VL via VSETVL
  if (PPU::GPRRegClass.contains(SrcReg) &&
      PPU::VLRRegClass.contains(DstReg)) {
    BuildMI(MBB, MBBI, DL, get(PPU::VSETVL), DstReg)
        .addDef(PPU::X0)
        .addReg(SrcReg, getKillRegState(KillSrc));
    return;
  }
  // VL -> GPR via CSR read
  if (PPU::VLRRegClass.contains(SrcReg) &&
      PPU::GPRRegClass.contains(DstReg)) {
    BuildMI(MBB, MBBI, DL, get(PPU::PseudoCSRR_VL), DstReg)
        .addReg(SrcReg, getKillRegState(KillSrc));
    return;
  }

  // VR -> VR copies
  if (PPU::TPRRegClass.contains(SrcReg) &&
      PPU::TPRRegClass.contains(DstReg)) {
    
    auto Scavenger = RegScavenger();
    Scavenger.enterBasicBlockEnd(MBB);
    unsigned SavedVL = Scavenger.scavengeRegisterBackwards(
      PPU::GPRRegClass, MBBI, false, 0);

    //Save current VL
    MachineInstr &MI = *BuildMI(MBB, MBBI, DL, get(PPU::CSRRS), SavedVL)
        .addImm(0xCC0)
        .addReg(PPU::X0);

    Scavenger.setRegUsed(SavedVL);

    //Save MAXVL in GPR
    unsigned MaxVL = Scavenger.scavengeRegisterBackwards(
      PPU::GPRRegClass, MachineBasicBlock::iterator(MI), false, 0);


    BuildMI(MBB, MBBI, DL, get(PPU::CSRRS), MaxVL)
        .addImm(0xCC1) //Couldn't find the actual CSR number - placeholder
        .addReg(PPU::X0);

    //Set VL to MAXVL
    BuildMI(MBB, MBBI, DL, get(PPU::VSETVL), PPU::VL)
        .addDef(PPU::X0)
        .addReg(MaxVL, getKillRegState(true));

    //Copy Vector
    BuildMI(MBB, MBBI, DL, get(PPU::VADDI), DstReg)
        .addReg(SrcReg, getKillRegState(KillSrc))
        .addImm(0)
        .addReg(PPU::VL);

    BuildMI(MBB, MBBI, DL, get(PPU::VSETVL), PPU::VL)
        .addDef(PPU::X0)
        .addReg(SavedVL, getKillRegState(true));

    return;
  }

  // FPR->FPR copies
  unsigned Opc;
  if (PPU::FPR32RegClass.contains(DstReg, SrcReg))
    Opc = PPU::FSGNJ_S;
  else if (PPU::FPR64RegClass.contains(DstReg, SrcReg))
    Opc = PPU::FSGNJ_D;
  else
    llvm_unreachable("Impossible reg-to-reg copy");

  BuildMI(MBB, MBBI, DL, get(Opc), DstReg)
      .addReg(SrcReg, getKillRegState(KillSrc))
      .addReg(SrcReg, getKillRegState(KillSrc));
}

void PPUBaseInstrInfo::storeRegToStackSlot(MachineBasicBlock &MBB,
                                         MachineBasicBlock::iterator I,
                                         unsigned SrcReg, bool IsKill, int FI,
                                         const TargetRegisterClass *RC,
                                         const TargetRegisterInfo *TRI) const {
  DebugLoc DL;
  if (I != MBB.end())
    DL = I->getDebugLoc();

  unsigned Opcode;

  if (PPU::GPRRegClass.hasSubClassEq(RC))
    Opcode = TRI->getRegSizeInBits(PPU::GPRRegClass) == 32 ?
             PPU::SW : PPU::SD;
  else if (PPU::FPR32RegClass.hasSubClassEq(RC))
    Opcode = PPU::FSW;
  else if (PPU::FPR64RegClass.hasSubClassEq(RC))
    Opcode = PPU::FSD;
  else
    llvm_unreachable("Can't store this register to stack slot");

  BuildMI(MBB, I, DL, get(Opcode))
      .addReg(SrcReg, getKillRegState(IsKill))
      .addFrameIndex(FI)
      .addImm(0);
}

void PPUBaseInstrInfo::loadRegFromStackSlot(MachineBasicBlock &MBB,
                                          MachineBasicBlock::iterator I,
                                          unsigned DstReg, int FI,
                                          const TargetRegisterClass *RC,
                                          const TargetRegisterInfo *TRI) const {
  DebugLoc DL;
  if (I != MBB.end())
    DL = I->getDebugLoc();

  unsigned Opcode;

  if (PPU::GPRRegClass.hasSubClassEq(RC))
    Opcode = TRI->getRegSizeInBits(PPU::GPRRegClass) == 32 ?
             PPU::LW : PPU::LD;
  else if (PPU::FPR32RegClass.hasSubClassEq(RC))
    Opcode = PPU::FLW;
  else if (PPU::FPR64RegClass.hasSubClassEq(RC))
    Opcode = PPU::FLD;
  else
    llvm_unreachable("Can't load this register from stack slot");

  BuildMI(MBB, I, DL, get(Opcode), DstReg).addFrameIndex(FI).addImm(0);
}

void PPUBaseInstrInfo::movImm32(MachineBasicBlock &MBB,
                              MachineBasicBlock::iterator MBBI,
                              const DebugLoc &DL, Register DstReg, uint64_t Val,
                              MachineInstr::MIFlag Flag) const {
  assert(isInt<32>(Val) && "Can only materialize 32-bit constants");

  // TODO: If the value can be materialized using only one instruction, only
  // insert a single instruction.

  uint64_t Hi20 = ((Val + 0x800) >> 12) & 0xfffff;
  uint64_t Lo12 = SignExtend64<12>(Val);
  BuildMI(MBB, MBBI, DL, get(PPU::LUI), DstReg)
      .addImm(Hi20)
      .setMIFlag(Flag);
  BuildMI(MBB, MBBI, DL, get(PPU::ADDI), DstReg)
      .addReg(DstReg, RegState::Kill)
      .addImm(Lo12)
      .setMIFlag(Flag);
}

// The contents of values added to Cond are not examined outside of
// PPUInstrInfo, giving us flexibility in what to push to it. For PPU, we
// push BranchOpcode, Reg1, Reg2.
static void parseCondBranch(MachineInstr &LastInst, MachineBasicBlock *&Target,
                            SmallVectorImpl<MachineOperand> &Cond) {
  // Block ends with fall-through condbranch.
  assert(LastInst.getDesc().isConditionalBranch() &&
         "Unknown conditional branch");
  Target = LastInst.getOperand(2).getMBB();
  Cond.push_back(MachineOperand::CreateImm(LastInst.getOpcode()));
  Cond.push_back(LastInst.getOperand(0));
  Cond.push_back(LastInst.getOperand(1));
}

static unsigned getOppositeBranchOpcode(int Opc) {
  switch (Opc) {
  default:
    llvm_unreachable("Unrecognized conditional branch");
  case PPU::BEQ:
    return PPU::BNE;
  case PPU::BNE:
    return PPU::BEQ;
  case PPU::BLT:
    return PPU::BGE;
  case PPU::BGE:
    return PPU::BLT;
  case PPU::BLTU:
    return PPU::BGEU;
  case PPU::BGEU:
    return PPU::BLTU;
  }
}

bool PPUBaseInstrInfo::analyzeBranch(MachineBasicBlock &MBB,
                                   MachineBasicBlock *&TBB,
                                   MachineBasicBlock *&FBB,
                                   SmallVectorImpl<MachineOperand> &Cond,
                                   bool AllowModify) const {
  TBB = FBB = nullptr;
  Cond.clear();

  // If the block has no terminators, it just falls into the block after it.
  MachineBasicBlock::iterator I = MBB.getLastNonDebugInstr();
  if (I == MBB.end() || !isUnpredicatedTerminator(*I))
    return false;

  // Count the number of terminators and find the first unconditional or
  // indirect branch.
  MachineBasicBlock::iterator FirstUncondOrIndirectBr = MBB.end();
  int NumTerminators = 0;
  for (auto J = I.getReverse(); J != MBB.rend() && isUnpredicatedTerminator(*J);
       J++) {
    NumTerminators++;
    if (J->getDesc().isUnconditionalBranch() ||
        J->getDesc().isIndirectBranch()) {
      FirstUncondOrIndirectBr = J.getReverse();
    }
  }

  // If AllowModify is true, we can erase any terminators after
  // FirstUncondOrIndirectBR.
  if (AllowModify && FirstUncondOrIndirectBr != MBB.end()) {
    while (std::next(FirstUncondOrIndirectBr) != MBB.end()) {
      std::next(FirstUncondOrIndirectBr)->eraseFromParent();
      NumTerminators--;
    }
    I = FirstUncondOrIndirectBr;
  }

  // We can't handle blocks that end in an indirect branch.
  if (I->getDesc().isIndirectBranch())
    return true;

  // We can't handle blocks with more than 2 terminators.
  if (NumTerminators > 2)
    return true;

  // Handle a single unconditional branch.
  if (NumTerminators == 1 && I->getDesc().isUnconditionalBranch()) {
    TBB = I->getOperand(0).getMBB();
    return false;
  }

  // Handle a single conditional branch.
  if (NumTerminators == 1 && I->getDesc().isConditionalBranch()) {
    parseCondBranch(*I, TBB, Cond);
    return false;
  }

  // Handle a conditional branch followed by an unconditional branch.
  if (NumTerminators == 2 && std::prev(I)->getDesc().isConditionalBranch() &&
      I->getDesc().isUnconditionalBranch()) {
    parseCondBranch(*std::prev(I), TBB, Cond);
    FBB = I->getOperand(0).getMBB();
    return false;
  }

  // Otherwise, we can't handle this.
  return true;
}

unsigned PPUBaseInstrInfo::removeBranch(MachineBasicBlock &MBB,
                                      int *BytesRemoved) const {
  if (BytesRemoved)
    *BytesRemoved = 0;
  MachineBasicBlock::iterator I = MBB.getLastNonDebugInstr();
  if (I == MBB.end())
    return 0;

  if (!I->getDesc().isUnconditionalBranch() &&
      !I->getDesc().isConditionalBranch())
    return 0;

  // Remove the branch.
  if (BytesRemoved)
    *BytesRemoved += getInstSizeInBytes(*I);
  I->eraseFromParent();

  I = MBB.end();

  if (I == MBB.begin())
    return 1;
  --I;
  if (!I->getDesc().isConditionalBranch())
    return 1;

  // Remove the branch.
  if (BytesRemoved)
    *BytesRemoved += getInstSizeInBytes(*I);
  I->eraseFromParent();
  return 2;
}

// Inserts a branch into the end of the specific MachineBasicBlock, returning
// the number of instructions inserted.
unsigned PPUBaseInstrInfo::insertBranch(
    MachineBasicBlock &MBB, MachineBasicBlock *TBB, MachineBasicBlock *FBB,
    ArrayRef<MachineOperand> Cond, const DebugLoc &DL, int *BytesAdded) const {
  if (BytesAdded)
    *BytesAdded = 0;

  // Shouldn't be a fall through.
  assert(TBB && "InsertBranch must not be told to insert a fallthrough");
  assert((Cond.size() == 3 || Cond.size() == 0) &&
         "PPU branch conditions have two components!");

  // Unconditional branch.
  if (Cond.empty()) {
    MachineInstr &MI = *BuildMI(&MBB, DL, get(PPU::PseudoBR)).addMBB(TBB);
    if (BytesAdded)
      *BytesAdded += getInstSizeInBytes(MI);
    return 1;
  }

  // Either a one or two-way conditional branch.
  unsigned Opc = Cond[0].getImm();
  MachineInstr &CondMI =
      *BuildMI(&MBB, DL, get(Opc)).add(Cond[1]).add(Cond[2]).addMBB(TBB);
  if (BytesAdded)
    *BytesAdded += getInstSizeInBytes(CondMI);

  // One-way conditional branch.
  if (!FBB)
    return 1;

  // Two-way conditional branch.
  MachineInstr &MI = *BuildMI(&MBB, DL, get(PPU::PseudoBR)).addMBB(FBB);
  if (BytesAdded)
    *BytesAdded += getInstSizeInBytes(MI);
  return 2;
}

unsigned PPUBaseInstrInfo::insertIndirectBranch(MachineBasicBlock &MBB,
                                              MachineBasicBlock &DestBB,
                                              const DebugLoc &DL,
                                              int64_t BrOffset,
                                              RegScavenger *RS) const {
  assert(RS && "RegScavenger required for long branching");
  assert(MBB.empty() &&
         "new block should be inserted for expanding unconditional branch");
  assert(MBB.pred_size() == 1);

  MachineFunction *MF = MBB.getParent();
  MachineRegisterInfo &MRI = MF->getRegInfo();
  const auto &TM = static_cast<const PPUTargetMachine &>(MF->getTarget());

  if (TM.isPositionIndependent())
    report_fatal_error("Unable to insert indirect branch");

  if (!isInt<32>(BrOffset))
    report_fatal_error(
        "Branch offsets outside of the signed 32-bit range not supported");

  // FIXME: A virtual register must be used initially, as the register
  // scavenger won't work with empty blocks (PPUInstrInfo::insertIndirectBranch
  // uses the same workaround).
  Register ScratchReg = MRI.createVirtualRegister(&PPU::GPRRegClass);
  auto II = MBB.end();

  MachineInstr &LuiMI = *BuildMI(MBB, II, DL, get(PPU::LUI), ScratchReg)
                             .addMBB(&DestBB, PPUII::MO_HI);
  BuildMI(MBB, II, DL, get(PPU::PseudoBRIND))
      .addReg(ScratchReg, RegState::Kill)
      .addMBB(&DestBB, PPUII::MO_LO);

  RS->enterBasicBlockEnd(MBB);
  unsigned Scav = RS->scavengeRegisterBackwards(PPU::GPRRegClass,
                                                LuiMI.getIterator(), false, 0);
  MRI.replaceRegWith(ScratchReg, Scav);
  MRI.clearVirtRegs();
  RS->setRegUsed(Scav);
  return 8;
}

bool PPUBaseInstrInfo::reverseBranchCondition(
    SmallVectorImpl<MachineOperand> &Cond) const {
  assert((Cond.size() == 3) && "Invalid branch condition!");
  Cond[0].setImm(getOppositeBranchOpcode(Cond[0].getImm()));
  return false;
}

MachineBasicBlock *
PPUBaseInstrInfo::getBranchDestBlock(const MachineInstr &MI) const {
  assert(MI.getDesc().isBranch() && "Unexpected opcode!");
  // The branch target is always the last operand.
  int NumOp = MI.getNumExplicitOperands();
  return MI.getOperand(NumOp - 1).getMBB();
}

bool PPUBaseInstrInfo::isBranchOffsetInRange(unsigned BranchOp,
                                           int64_t BrOffset) const {
  // Ideally we could determine the supported branch offset from the
  // PPUII::FormMask, but this can't be used for Pseudo instructions like
  // PseudoBR.
  switch (BranchOp) {
  default:
    llvm_unreachable("Unexpected opcode!");
  case PPU::BEQ:
  case PPU::BNE:
  case PPU::BLT:
  case PPU::BGE:
  case PPU::BLTU:
  case PPU::BGEU:
    return isIntN(13, BrOffset);
  case PPU::JAL:
  case PPU::PseudoBR:
    return isIntN(21, BrOffset);
  }
}

unsigned PPUBaseInstrInfo::getInstSizeInBytes(const MachineInstr &MI) const {
  unsigned Opcode = MI.getOpcode();

  switch (Opcode) {
  default: { return get(Opcode).getSize(); }
  case TargetOpcode::EH_LABEL:
  case TargetOpcode::IMPLICIT_DEF:
  case TargetOpcode::KILL:
  case TargetOpcode::DBG_VALUE:
    return 0;
  case PPU::PseudoCALLReg:
  case PPU::PseudoCALL:
  case PPU::PseudoTAIL:
  case PPU::PseudoLLA:
  case PPU::PseudoLA:
  case PPU::PseudoLA_TLS_IE:
  case PPU::PseudoLA_TLS_GD:
    return 8;
  case TargetOpcode::INLINEASM:
  case TargetOpcode::INLINEASM_BR: {
    const MachineFunction &MF = *MI.getParent()->getParent();
    const auto &TM = static_cast<const PPUTargetMachine &>(MF.getTarget());
    return getInlineAsmLength(MI.getOperand(0).getSymbolName(),
                              *TM.getMCAsmInfo());
  }
  }
}

bool PPUBaseInstrInfo::isAsCheapAsAMove(const MachineInstr &MI) const {
  const unsigned Opcode = MI.getOpcode();
  switch(Opcode) {
    default:
      break;
    case PPU::ADDI:
    case PPU::ORI:
    case PPU::XORI:
      return (MI.getOperand(1).isReg() && MI.getOperand(1).getReg() == PPU::X0);
  }
  return MI.isAsCheapAsAMove();
}

bool PPUBaseInstrInfo::isBranchOp(unsigned BranchOp) const {
  switch (BranchOp) {
  default:
    return false;
  case PPU::BEQ:
  case PPU::BNE:
  case PPU::BLT:
  case PPU::BGE:
  case PPU::BLTU:
  case PPU::BGEU:
    return true;
  case PPU::JAL:
  case PPU::PseudoBR:
    return true;
  }
}

// this is base on AMDGPUInstrInfo.cpp
// TODO: Should largely merge with AMDGPUTTIImpl::isSourceOfDivergence.
bool PPUInstrInfo::isUniformMMO(const MachineMemOperand *MMO) {
  const Value *Ptr = MMO->getValue();
  // UndefValue means this is a load of a kernel input.  These are uniform.
  // Sometimes LDS instructions have constant pointers.
  // If Ptr is null, then that means this mem operand contains a
  // PseudoSourceValue like GOT.
  if (!Ptr || isa<UndefValue>(Ptr) ||
      isa<Constant>(Ptr) || isa<GlobalValue>(Ptr))
    return true;

  if (MMO->getAddrSpace() == AMDGPUAS::CONSTANT_ADDRESS_32BIT)
    return true;

  if (const Argument *Arg = dyn_cast<Argument>(Ptr))
    return PPU::isArgPassedInSGPR(Arg);

  const Instruction *I = dyn_cast<Instruction>(Ptr);
  return I && I->getMetadata("ppu.uniform");
}


// below is base on SIInstrInfo.cpp
//
// Must be at least 4 to be able to branch over minimum unconditional branch
// code. This is only for making it possible to write reasonably small tests for
// long branches.
static cl::opt<unsigned>
BranchOffsetBits("ppu-s-branch-bits", cl::ReallyHidden, cl::init(16),
                 cl::desc("Restrict range of branch instructions (DEBUG)"));

PPUInstrInfo::PPUInstrInfo(const PPUSubtarget &ST)
    : PPUBaseInstrInfo()
    , ST(ST)
    , RI(*ST.getRegisterInfo())
    {
        // RI = ST.getRegisterInfo();
    }
  // : PPUGenInstrInfo(PPU::ADJCALLSTACKUP, PPU::ADJCALLSTACKDOWN),

//===----------------------------------------------------------------------===//
// TargetInstrInfo callbacks
//===----------------------------------------------------------------------===//

static unsigned getNumOperandsNoGlue(SDNode *Node) {
  unsigned N = Node->getNumOperands();
  while (N && Node->getOperand(N - 1).getValueType() == MVT::Glue)
    --N;
  return N;
}

/// Returns true if both nodes have the same value for the given
///        operand \p Op, or if both nodes do not have this operand.
static bool nodesHaveSameOperandValue(SDNode *N0, SDNode* N1, unsigned OpName) {
  unsigned Opc0 = N0->getMachineOpcode();
  unsigned Opc1 = N1->getMachineOpcode();

  int Op0Idx = PPU::getNamedOperandIdx(Opc0, OpName);
  int Op1Idx = PPU::getNamedOperandIdx(Opc1, OpName);

  if (Op0Idx == -1 && Op1Idx == -1)
    return true;


  if ((Op0Idx == -1 && Op1Idx != -1) ||
      (Op1Idx == -1 && Op0Idx != -1))
    return false;

  // getNamedOperandIdx returns the index for the MachineInstr's operands,
  // which includes the result as the first operand. We are indexing into the
  // MachineSDNode's operands, so we need to skip the result operand to get
  // the real index.
  --Op0Idx;
  --Op1Idx;

  return N0->getOperand(Op0Idx) == N1->getOperand(Op1Idx);
}

bool PPUInstrInfo::isReallyTriviallyReMaterializable(const MachineInstr &MI,
                                                    AliasAnalysis *AA) const {
  // TODO: The generic check fails for VALU instructions that should be
  // rematerializable due to implicit reads of exec. We really want all of the
  // generic logic for this except for this.
  switch (MI.getOpcode()) {
  case PPU::VMOV:
    // No implicit operands.
    return MI.getNumOperands() == MI.getDesc().getNumOperands();
  default:
    return false;
  }
}

bool PPUInstrInfo::areLoadsFromSameBasePtr(SDNode *Load0, SDNode *Load1,
                                          int64_t &Offset0,
                                          int64_t &Offset1) const {
    /* TODO
  if (!Load0->isMachineOpcode() || !Load1->isMachineOpcode())
    return false;

  unsigned Opc0 = Load0->getMachineOpcode();
  unsigned Opc1 = Load1->getMachineOpcode();

  // Make sure both are actually loads.
  if (!get(Opc0).mayLoad() || !get(Opc1).mayLoad())
    return false;

  if (isDS(Opc0) && isDS(Opc1)) {

    // FIXME: Handle this case:
    if (getNumOperandsNoGlue(Load0) != getNumOperandsNoGlue(Load1))
      return false;

    // Check base reg.
    if (Load0->getOperand(0) != Load1->getOperand(0))
      return false;

    // Skip read2 / write2 variants for simplicity.
    // TODO: We should report true if the used offsets are adjacent (excluded
    // st64 versions).
    int Offset0Idx = PPU::getNamedOperandIdx(Opc0, PPU::OpName::offset);
    int Offset1Idx = PPU::getNamedOperandIdx(Opc1, PPU::OpName::offset);
    if (Offset0Idx == -1 || Offset1Idx == -1)
      return false;

    // XXX - be careful of datalesss loads
    // getNamedOperandIdx returns the index for MachineInstrs.  Since they
    // include the output in the operand list, but SDNodes don't, we need to
    // subtract the index by one.
    Offset0Idx -= get(Opc0).NumDefs;
    Offset1Idx -= get(Opc1).NumDefs;
    Offset0 = cast<ConstantSDNode>(Load0->getOperand(Offset0Idx))->getZExtValue();
    Offset1 = cast<ConstantSDNode>(Load1->getOperand(Offset1Idx))->getZExtValue();
    return true;
  }

  if (isSMRD(Opc0) && isSMRD(Opc1)) {
    // Skip time and cache invalidation instructions.
    if (PPU::getNamedOperandIdx(Opc0, PPU::OpName::sbase) == -1 ||
        PPU::getNamedOperandIdx(Opc1, PPU::OpName::sbase) == -1)
      return false;

    assert(getNumOperandsNoGlue(Load0) == getNumOperandsNoGlue(Load1));

    // Check base reg.
    if (Load0->getOperand(0) != Load1->getOperand(0))
      return false;

    const ConstantSDNode *Load0Offset =
        dyn_cast<ConstantSDNode>(Load0->getOperand(1));
    const ConstantSDNode *Load1Offset =
        dyn_cast<ConstantSDNode>(Load1->getOperand(1));

    if (!Load0Offset || !Load1Offset)
      return false;

    Offset0 = Load0Offset->getZExtValue();
    Offset1 = Load1Offset->getZExtValue();
    return true;
  }

  // MUBUF and MTBUF can access the same addresses.
  if ((isMUBUF(Opc0) || isMTBUF(Opc0)) && (isMUBUF(Opc1) || isMTBUF(Opc1))) {

    // MUBUF and MTBUF have vaddr at different indices.
    if (!nodesHaveSameOperandValue(Load0, Load1, PPU::OpName::soffset) ||
        !nodesHaveSameOperandValue(Load0, Load1, PPU::OpName::vaddr) ||
        !nodesHaveSameOperandValue(Load0, Load1, PPU::OpName::srsrc))
      return false;

    int OffIdx0 = PPU::getNamedOperandIdx(Opc0, PPU::OpName::offset);
    int OffIdx1 = PPU::getNamedOperandIdx(Opc1, PPU::OpName::offset);

    if (OffIdx0 == -1 || OffIdx1 == -1)
      return false;

    // getNamedOperandIdx returns the index for MachineInstrs.  Since they
    // include the output in the operand list, but SDNodes don't, we need to
    // subtract the index by one.
    OffIdx0 -= get(Opc0).NumDefs;
    OffIdx1 -= get(Opc1).NumDefs;

    SDValue Off0 = Load0->getOperand(OffIdx0);
    SDValue Off1 = Load1->getOperand(OffIdx1);

    // The offset might be a FrameIndexSDNode.
    if (!isa<ConstantSDNode>(Off0) || !isa<ConstantSDNode>(Off1))
      return false;

    Offset0 = cast<ConstantSDNode>(Off0)->getZExtValue();
    Offset1 = cast<ConstantSDNode>(Off1)->getZExtValue();
    return true;
  }
    */

  return false;
}

static bool isStride64(unsigned Opc) {
  switch (Opc) {
      /* TODO
  case PPU::DS_READ2ST64_B32:
  case PPU::DS_READ2ST64_B64:
  case PPU::DS_WRITE2ST64_B32:
  case PPU::DS_WRITE2ST64_B64:
    return true;
    */
  default:
    return false;
  }
}

bool PPUInstrInfo::getMemOperandWithOffset(const MachineInstr &LdSt,
                                          const MachineOperand *&BaseOp,
                                          int64_t &Offset,
                                          const TargetRegisterInfo *TRI) const {
    /* TODO
  unsigned Opc = LdSt.getOpcode();

  if (isDS(LdSt)) {
    const MachineOperand *OffsetImm =
        getNamedOperand(LdSt, PPU::OpName::offset);
    if (OffsetImm) {
      // Normal, single offset LDS instruction.
      BaseOp = getNamedOperand(LdSt, PPU::OpName::addr);
      // TODO: ds_consume/ds_append use M0 for the base address. Is it safe to
      // report that here?
      if (!BaseOp)
        return false;

      Offset = OffsetImm->getImm();
      assert(BaseOp->isReg() && "getMemOperandWithOffset only supports base "
                                "operands of type register.");
      return true;
    }

    // The 2 offset instructions use offset0 and offset1 instead. We can treat
    // these as a load with a single offset if the 2 offsets are consecutive. We
    // will use this for some partially aligned loads.
    const MachineOperand *Offset0Imm =
        getNamedOperand(LdSt, PPU::OpName::offset0);
    const MachineOperand *Offset1Imm =
        getNamedOperand(LdSt, PPU::OpName::offset1);

    uint8_t Offset0 = Offset0Imm->getImm();
    uint8_t Offset1 = Offset1Imm->getImm();

    if (Offset1 > Offset0 && Offset1 - Offset0 == 1) {
      // Each of these offsets is in element sized units, so we need to convert
      // to bytes of the individual reads.

      unsigned EltSize;
      if (LdSt.mayLoad())
        EltSize = TRI->getRegSizeInBits(*getOpRegClass(LdSt, 0)) / 16;
      else {
        assert(LdSt.mayStore());
        int Data0Idx = PPU::getNamedOperandIdx(Opc, PPU::OpName::data0);
        EltSize = TRI->getRegSizeInBits(*getOpRegClass(LdSt, Data0Idx)) / 8;
      }

      if (isStride64(Opc))
        EltSize *= 64;

      BaseOp = getNamedOperand(LdSt, PPU::OpName::addr);
      Offset = EltSize * Offset0;
      assert(BaseOp->isReg() && "getMemOperandWithOffset only supports base "
                                "operands of type register.");
      return true;
    }

    return false;
  }

  if (isMUBUF(LdSt) || isMTBUF(LdSt)) {
    const MachineOperand *SOffset = getNamedOperand(LdSt, PPU::OpName::soffset);
    if (SOffset && SOffset->isReg())
      return false;

    const MachineOperand *AddrReg = getNamedOperand(LdSt, PPU::OpName::vaddr);
    if (!AddrReg)
      return false;

    const MachineOperand *OffsetImm =
        getNamedOperand(LdSt, PPU::OpName::offset);
    BaseOp = AddrReg;
    Offset = OffsetImm->getImm();

    if (SOffset) // soffset can be an inline immediate.
      Offset += SOffset->getImm();

    assert(BaseOp->isReg() && "getMemOperandWithOffset only supports base "
                              "operands of type register.");
    return true;
  }

  if (isSMRD(LdSt)) {
    const MachineOperand *OffsetImm =
        getNamedOperand(LdSt, PPU::OpName::offset);
    if (!OffsetImm)
      return false;

    const MachineOperand *SBaseReg = getNamedOperand(LdSt, PPU::OpName::sbase);
    BaseOp = SBaseReg;
    Offset = OffsetImm->getImm();
    assert(BaseOp->isReg() && "getMemOperandWithOffset only supports base "
                              "operands of type register.");
    return true;
  }

  if (isFLAT(LdSt)) {
    const MachineOperand *VAddr = getNamedOperand(LdSt, PPU::OpName::vaddr);
    if (VAddr) {
      // Can't analyze 2 offsets.
      if (getNamedOperand(LdSt, PPU::OpName::saddr))
        return false;

      BaseOp = VAddr;
    } else {
      // scratch instructions have either vaddr or saddr.
      BaseOp = getNamedOperand(LdSt, PPU::OpName::saddr);
    }

    Offset = getNamedOperand(LdSt, PPU::OpName::offset)->getImm();
    assert(BaseOp->isReg() && "getMemOperandWithOffset only supports base "
                              "operands of type register.");
    return true;
  }
  */

  return false;
}

static bool memOpsHaveSameBasePtr(const MachineInstr &MI1,
                                  const MachineOperand &BaseOp1,
                                  const MachineInstr &MI2,
                                  const MachineOperand &BaseOp2) {
  // Support only base operands with base registers.
  // Note: this could be extended to support FI operands.
  /*
  if (!BaseOp1.isReg() || !BaseOp2.isReg())
    return false;

  if (BaseOp1.isIdenticalTo(BaseOp2))
    return true;

  if (!MI1.hasOneMemOperand() || !MI2.hasOneMemOperand())
    return false;

  auto MO1 = *MI1.memoperands_begin();
  auto MO2 = *MI2.memoperands_begin();
  if (MO1->getAddrSpace() != MO2->getAddrSpace())
    return false;

  auto Base1 = MO1->getValue();
  auto Base2 = MO2->getValue();
  if (!Base1 || !Base2)
    return false;
  const MachineFunction &MF = *MI1.getParent()->getParent();
  const DataLayout &DL = MF.getFunction().getParent()->getDataLayout();
  Base1 = GetUnderlyingObject(Base1, DL);
  Base2 = GetUnderlyingObject(Base1, DL);

  if (isa<UndefValue>(Base1) || isa<UndefValue>(Base2))
    return false;

  return Base1 == Base2;
  */
    return false;
}

bool PPUInstrInfo::shouldClusterMemOps(const MachineOperand &BaseOp1,
                                      const MachineOperand &BaseOp2,
                                      unsigned NumLoads) const {
    /*
  const MachineInstr &FirstLdSt = *BaseOp1.getParent();
  const MachineInstr &SecondLdSt = *BaseOp2.getParent();

  if (!memOpsHaveSameBasePtr(FirstLdSt, BaseOp1, SecondLdSt, BaseOp2))
    return false;

  const MachineOperand *FirstDst = nullptr;
  const MachineOperand *SecondDst = nullptr;

  if ((isMUBUF(FirstLdSt) && isMUBUF(SecondLdSt)) ||
      (isMTBUF(FirstLdSt) && isMTBUF(SecondLdSt)) ||
      (isFLAT(FirstLdSt) && isFLAT(SecondLdSt))) {
    const unsigned MaxGlobalLoadCluster = 6;
    if (NumLoads > MaxGlobalLoadCluster)
      return false;

    FirstDst = getNamedOperand(FirstLdSt, PPU::OpName::vdata);
    if (!FirstDst)
      FirstDst = getNamedOperand(FirstLdSt, PPU::OpName::vdst);
    SecondDst = getNamedOperand(SecondLdSt, PPU::OpName::vdata);
    if (!SecondDst)
      SecondDst = getNamedOperand(SecondLdSt, PPU::OpName::vdst);
  } else if (isSMRD(FirstLdSt) && isSMRD(SecondLdSt)) {
    FirstDst = getNamedOperand(FirstLdSt, PPU::OpName::sdst);
    SecondDst = getNamedOperand(SecondLdSt, PPU::OpName::sdst);
  } else if (isDS(FirstLdSt) && isDS(SecondLdSt)) {
    FirstDst = getNamedOperand(FirstLdSt, PPU::OpName::vdst);
    SecondDst = getNamedOperand(SecondLdSt, PPU::OpName::vdst);
  }

  if (!FirstDst || !SecondDst)
    return false;

  // Try to limit clustering based on the total number of bytes loaded
  // rather than the number of instructions.  This is done to help reduce
  // register pressure.  The method used is somewhat inexact, though,
  // because it assumes that all loads in the cluster will load the
  // same number of bytes as FirstLdSt.

  // The unit of this value is bytes.
  // FIXME: This needs finer tuning.
  unsigned LoadClusterThreshold = 16;

  const MachineRegisterInfo &MRI =
      FirstLdSt.getParent()->getParent()->getRegInfo();

  const Register Reg = FirstDst->getReg();

  const TargetRegisterClass *DstRC = Register::isVirtualRegister(Reg)
                                         ? MRI.getRegClass(Reg)
                                         : RI.getPhysRegClass(Reg);

  return (NumLoads * (RI.getRegSizeInBits(*DstRC) / 8)) <= LoadClusterThreshold;
  */
    return false;
}

// FIXME: This behaves strangely. If, for example, you have 32 load + stores,
// the first 16 loads will be interleaved with the stores, and the next 16 will
// be clustered as expected. It should really split into 2 16 store batches.
//
// Loads are clustered until this returns false, rather than trying to schedule
// groups of stores. This also means we have to deal with saying different
// address space loads should be clustered, and ones which might cause bank
// conflicts.
//
// This might be deprecated so it might not be worth that much effort to fix.
bool PPUInstrInfo::shouldScheduleLoadsNear(SDNode *Load0, SDNode *Load1,
                                          int64_t Offset0, int64_t Offset1,
                                          unsigned NumLoads) const {
  assert(Offset1 > Offset0 &&
         "Second offset should be larger than first offset!");
  // If we have less than 16 loads in a row, and the offsets are within 64
  // bytes, then schedule together.

  // A cacheline is 64 bytes (for global memory).
  return (NumLoads <= 16 && (Offset1 - Offset0) < 64);
}

static void reportIllegalCopy(const PPUInstrInfo *TII, MachineBasicBlock &MBB,
                              MachineBasicBlock::iterator MI,
                              const DebugLoc &DL, unsigned DestReg,
                              unsigned SrcReg, bool KillSrc) {
    /*
  MachineFunction *MF = MBB.getParent();
  DiagnosticInfoUnsupported IllegalCopy(MF->getFunction(),
                                        "illegal SGPR to VGPR copy",
                                        DL, DS_Error);
  LLVMContext &C = MF->getFunction().getContext();
  C.diagnose(IllegalCopy);

  BuildMI(MBB, MI, DL, TII->get(PPU::SI_ILLEGAL_COPY), DestReg)
    .addReg(SrcReg, getKillRegState(KillSrc));
    */
}

void PPUInstrInfo::copyPhysReg(MachineBasicBlock &MBB,
                              MachineBasicBlock::iterator MI,
                              const DebugLoc &DL, unsigned DestReg,
                              unsigned SrcReg, bool KillSrc) const {
  const TargetRegisterClass *RC = RI.getPhysRegClass(DestReg);

  if (RC == &PPU::TPRRegClass) {
    assert(PPU::TPRRegClass.contains(SrcReg) ||
           PPU::GPRRegClass.contains(SrcReg));
    unsigned Opc = PPU::VMOV;
    BuildMI(MBB, MI, DL, get(Opc), DestReg)
      .addReg(SrcReg, getKillRegState(KillSrc));
    return;
  }

    /*
  if (RC == &PPU::SReg_32_XM0RegClass ||
      RC == &PPU::GPRRegClass) {
    if (SrcReg == PPU::SCC) {
      BuildMI(MBB, MI, DL, get(PPU::S_CSELECT_B32), DestReg)
          .addImm(-1)
          .addImm(0);
      return;
    }

    if (DestReg == PPU::VCC) {
      if (PPU::GPRRegClass.contains(SrcReg)) {
        BuildMI(MBB, MI, DL, get(PPU::SMOV), PPU::VCC)
          .addReg(SrcReg, getKillRegState(KillSrc));
      } else {
        // FIXME: Hack until VReg_1 removed.
        assert(PPU::TPRRegClass.contains(SrcReg));
        BuildMI(MBB, MI, DL, get(PPU::V_CMP_NE_U32_e32))
          .addImm(0)
          .addReg(SrcReg, getKillRegState(KillSrc));
      }

      return;
    }

    if (!PPU::GPRRegClass.contains(SrcReg)) {
      reportIllegalCopy(this, MBB, MI, DL, DestReg, SrcReg, KillSrc);
      return;
    }

    BuildMI(MBB, MI, DL, get(PPU::SMOV), DestReg)
            .addReg(SrcReg, getKillRegState(KillSrc));
    return;
  }
    */

/*
  if (DestReg == PPU::SCC) {
    assert(PPU::GPRRegClass.contains(SrcReg));
    BuildMI(MBB, MI, DL, get(PPU::S_CMP_LG_U32))
      .addReg(SrcReg, getKillRegState(KillSrc))
      .addImm(0);
    return;
  }
  */

  unsigned EltSize = 4;
  unsigned Opcode = PPU::VMOV;
  if (RI.isSGPRClass(RC)) {
    // TODO: Copy vec3/vec5 with s_mov_b64s then final s_mov_b32.
    Opcode = PPU::SMOV;
    EltSize = 4;

    if (!RI.isSGPRClass(RI.getPhysRegClass(SrcReg))) {
      reportIllegalCopy(this, MBB, MI, DL, DestReg, SrcReg, KillSrc);
      return;
    }
  }

  ArrayRef<int16_t> SubIndices = RI.getRegSplitParts(RC, EltSize);
  bool Forward = RI.getHWRegIndex(DestReg) <= RI.getHWRegIndex(SrcReg);

  for (unsigned Idx = 0; Idx < SubIndices.size(); ++Idx) {
    unsigned SubIdx;
    if (Forward)
      SubIdx = SubIndices[Idx];
    else
      SubIdx = SubIndices[SubIndices.size() - Idx - 1];

    if (Opcode == TargetOpcode::COPY) {
      copyPhysReg(MBB, MI, DL, RI.getSubReg(DestReg, SubIdx),
                  RI.getSubReg(SrcReg, SubIdx), KillSrc);
      continue;
    }

    MachineInstrBuilder Builder = BuildMI(MBB, MI, DL,
      get(Opcode), RI.getSubReg(DestReg, SubIdx));

    Builder.addReg(RI.getSubReg(SrcReg, SubIdx));

    if (Idx == 0)
      Builder.addReg(DestReg, RegState::Define | RegState::Implicit);

    bool UseKill = KillSrc && Idx == SubIndices.size() - 1;
    Builder.addReg(SrcReg, getKillRegState(UseKill) | RegState::Implicit);
  }
}

int PPUInstrInfo::commuteOpcode(unsigned Opcode) const {
    /*
  int NewOpc;

  // Try to map original to commuted opcode
  NewOpc = PPU::getCommuteRev(Opcode);
  if (NewOpc != -1)
    // Check if the commuted (REV) opcode exists on the target.
    return pseudoToMCOpcode(NewOpc) != -1 ? NewOpc : -1;

  // Try to map commuted to original opcode
  NewOpc = PPU::getCommuteOrig(Opcode);
  if (NewOpc != -1)
    // Check if the original (non-REV) opcode exists on the target.
    return pseudoToMCOpcode(NewOpc) != -1 ? NewOpc : -1;

  */
  return Opcode;
}

void PPUInstrInfo::materializeImmediate(MachineBasicBlock &MBB,
                                       MachineBasicBlock::iterator MI,
                                       const DebugLoc &DL, unsigned DestReg,
                                       int64_t Value) const {
  MachineRegisterInfo &MRI = MBB.getParent()->getRegInfo();
  const TargetRegisterClass *RegClass = MRI.getRegClass(DestReg);
  if (RegClass == &PPU::GPRRegClass ||
      RegClass == &PPU::SReg_32RegClass) {
    BuildMI(MBB, MI, DL, get(PPU::SMOV), DestReg)
      .addImm(Value);
    return;
  }

  if (RegClass == &PPU::TPRRegClass) {
    BuildMI(MBB, MI, DL, get(PPU::VMOV), DestReg)
      .addImm(Value);
    return;
  }

  unsigned EltSize = 4;
  unsigned Opcode = PPU::VMOV;
  if (RI.isSGPRClass(RegClass)) {
      Opcode = PPU::SMOV;
      EltSize = 4;
  }

  ArrayRef<int16_t> SubIndices = RI.getRegSplitParts(RegClass, EltSize);
  for (unsigned Idx = 0; Idx < SubIndices.size(); ++Idx) {
    int64_t IdxValue = Idx == 0 ? Value : 0;

    MachineInstrBuilder Builder = BuildMI(MBB, MI, DL,
      get(Opcode), RI.getSubReg(DestReg, Idx));
    Builder.addImm(IdxValue);
  }
}

const TargetRegisterClass *
PPUInstrInfo::getPreferredSelectRegClass(unsigned Size) const {
  return &PPU::TPRRegClass;
}

void PPUInstrInfo::insertVectorSelect(MachineBasicBlock &MBB,
                                     MachineBasicBlock::iterator I,
                                     const DebugLoc &DL, unsigned DstReg,
                                     ArrayRef<MachineOperand> Cond,
                                     unsigned TrueReg,
                                     unsigned FalseReg) const {
    /*
  MachineRegisterInfo &MRI = MBB.getParent()->getRegInfo();
  MachineFunction *MF = MBB.getParent();
  const PPUSubtarget &ST = MF->getSubtarget<PPUSubtarget>();
  const TargetRegisterClass *BoolXExecRC =
    RI.getRegClass(PPU::SReg_1_XEXECRegClassID);
  assert(MRI.getRegClass(DstReg) == &PPU::TPRRegClass &&
         "Not a VGPR32 reg");

  if (Cond.size() == 1) {
    Register SReg = MRI.createVirtualRegister(BoolXExecRC);
    BuildMI(MBB, I, DL, get(PPU::COPY), SReg)
      .add(Cond[0]);
    BuildMI(MBB, I, DL, get(PPU::V_CNDMASK_B32_e64), DstReg)
      .addImm(0)
      .addReg(FalseReg)
      .addImm(0)
      .addReg(TrueReg)
      .addReg(SReg);
  } else if (Cond.size() == 2) {
    assert(Cond[0].isImm() && "Cond[0] is not an immediate");
    switch (Cond[0].getImm()) {
    case PPUInstrInfo::SCC_TRUE: {
      Register SReg = MRI.createVirtualRegister(BoolXExecRC);
      BuildMI(MBB, I, DL, get(ST.isWave32() ? PPU::S_CSELECT_B32
                                            : PPU::S_CSELECT_B64), SReg)
        .addImm(-1)
        .addImm(0);
      BuildMI(MBB, I, DL, get(PPU::V_CNDMASK_B32_e64), DstReg)
        .addImm(0)
        .addReg(FalseReg)
        .addImm(0)
        .addReg(TrueReg)
        .addReg(SReg);
      break;
    }
    case PPUInstrInfo::SCC_FALSE: {
      Register SReg = MRI.createVirtualRegister(BoolXExecRC);
      BuildMI(MBB, I, DL, get(ST.isWave32() ? PPU::S_CSELECT_B32
                                            : PPU::S_CSELECT_B64), SReg)
        .addImm(0)
        .addImm(-1);
      BuildMI(MBB, I, DL, get(PPU::V_CNDMASK_B32_e64), DstReg)
        .addImm(0)
        .addReg(FalseReg)
        .addImm(0)
        .addReg(TrueReg)
        .addReg(SReg);
      break;
    }
    case PPUInstrInfo::VCCNZ: {
      MachineOperand RegOp = Cond[1];
      RegOp.setImplicit(false);
      Register SReg = MRI.createVirtualRegister(BoolXExecRC);
      BuildMI(MBB, I, DL, get(PPU::COPY), SReg)
        .add(RegOp);
      BuildMI(MBB, I, DL, get(PPU::V_CNDMASK_B32_e64), DstReg)
          .addImm(0)
          .addReg(FalseReg)
          .addImm(0)
          .addReg(TrueReg)
          .addReg(SReg);
      break;
    }
    case PPUInstrInfo::VCCZ: {
      MachineOperand RegOp = Cond[1];
      RegOp.setImplicit(false);
      Register SReg = MRI.createVirtualRegister(BoolXExecRC);
      BuildMI(MBB, I, DL, get(PPU::COPY), SReg)
        .add(RegOp);
      BuildMI(MBB, I, DL, get(PPU::V_CNDMASK_B32_e64), DstReg)
          .addImm(0)
          .addReg(TrueReg)
          .addImm(0)
          .addReg(FalseReg)
          .addReg(SReg);
      break;
    }
    case PPUInstrInfo::EXECNZ: {
      Register SReg = MRI.createVirtualRegister(BoolXExecRC);
      Register SReg2 = MRI.createVirtualRegister(RI.getBoolRC());
      BuildMI(MBB, I, DL, get(ST.isWave32() ? PPU::S_OR_SAVEEXEC_B32
                                            : PPU::S_OR_SAVEEXEC_B64), SReg2)
        .addImm(0);
      BuildMI(MBB, I, DL, get(ST.isWave32() ? PPU::S_CSELECT_B32
                                            : PPU::S_CSELECT_B64), SReg)
        .addImm(-1)
        .addImm(0);
      BuildMI(MBB, I, DL, get(PPU::V_CNDMASK_B32_e64), DstReg)
        .addImm(0)
        .addReg(FalseReg)
        .addImm(0)
        .addReg(TrueReg)
        .addReg(SReg);
      break;
    }
    case PPUInstrInfo::EXECZ: {
      Register SReg = MRI.createVirtualRegister(BoolXExecRC);
      Register SReg2 = MRI.createVirtualRegister(RI.getBoolRC());
      BuildMI(MBB, I, DL, get(ST.isWave32() ? PPU::S_OR_SAVEEXEC_B32
                                            : PPU::S_OR_SAVEEXEC_B64), SReg2)
        .addImm(0);
      BuildMI(MBB, I, DL, get(ST.isWave32() ? PPU::S_CSELECT_B32
                                            : PPU::S_CSELECT_B64), SReg)
        .addImm(0)
        .addImm(-1);
      BuildMI(MBB, I, DL, get(PPU::V_CNDMASK_B32_e64), DstReg)
        .addImm(0)
        .addReg(FalseReg)
        .addImm(0)
        .addReg(TrueReg)
        .addReg(SReg);
      llvm_unreachable("Unhandled branch predicate EXECZ");
      break;
    }
    default:
      llvm_unreachable("invalid branch predicate");
    }
  } else {
    llvm_unreachable("Can only handle Cond size 1 or 2");
  }
*/
}

unsigned PPUInstrInfo::insertEQ(MachineBasicBlock *MBB,
                               MachineBasicBlock::iterator I,
                               const DebugLoc &DL,
                               unsigned SrcReg, int Value) const {
    /*
  MachineRegisterInfo &MRI = MBB->getParent()->getRegInfo();
  Register Reg = MRI.createVirtualRegister(RI.getBoolRC());
  BuildMI(*MBB, I, DL, get(PPU::V_CMP_EQ_I32_e64), Reg)
    .addImm(Value)
    .addReg(SrcReg);

  return Reg;
  */
    return 0;
}

unsigned PPUInstrInfo::insertNE(MachineBasicBlock *MBB,
                               MachineBasicBlock::iterator I,
                               const DebugLoc &DL,
                               unsigned SrcReg, int Value) const {
    /*
  MachineRegisterInfo &MRI = MBB->getParent()->getRegInfo();
  Register Reg = MRI.createVirtualRegister(RI.getBoolRC());
  BuildMI(*MBB, I, DL, get(PPU::V_CMP_NE_I32_e64), Reg)
    .addImm(Value)
    .addReg(SrcReg);

  return Reg;
  */
    return 0;
}

unsigned PPUInstrInfo::getMovOpcode(const TargetRegisterClass *DstRC) const {

  if (RI.getRegSizeInBits(*DstRC) == 32) {
    return RI.isSGPRClass(DstRC) ? PPU::SMOV : PPU::VMOV;
    /*
  } else if (RI.getRegSizeInBits(*DstRC) == 64 && RI.isSGPRClass(DstRC)) {
    return PPU::S_MOV_B64;
  } else if (RI.getRegSizeInBits(*DstRC) == 64 && !RI.isSGPRClass(DstRC)) {
    return  PPU::V_MOV_B64_PSEUDO;
    */
  }
  return PPU::COPY;
}

static unsigned getSGPRSpillSaveOpcode(unsigned Size) {
  switch (Size) {
      /* TODO
  case 4:
    return PPU::SI_SPILL_S32_SAVE;
  case 8:
    return PPU::SI_SPILL_S64_SAVE;
  case 12:
    return PPU::SI_SPILL_S96_SAVE;
  case 16:
    return PPU::SI_SPILL_S128_SAVE;
  case 20:
    return PPU::SI_SPILL_S160_SAVE;
  case 32:
    return PPU::SI_SPILL_S256_SAVE;
  case 64:
    return PPU::SI_SPILL_S512_SAVE;
  case 128:
    return PPU::SI_SPILL_S1024_SAVE;
    */
  default:
    llvm_unreachable("unknown register size");
  }
}

static unsigned getVGPRSpillSaveOpcode(unsigned Size) {
  switch (Size) {
      /* TODO
  case 4:
    return PPU::SI_SPILL_V32_SAVE;
  case 8:
    return PPU::SI_SPILL_V64_SAVE;
  case 12:
    return PPU::SI_SPILL_V96_SAVE;
  case 16:
    return PPU::SI_SPILL_V128_SAVE;
  case 20:
    return PPU::SI_SPILL_V160_SAVE;
  case 32:
    return PPU::SI_SPILL_V256_SAVE;
  case 64:
    return PPU::SI_SPILL_V512_SAVE;
  case 128:
    return PPU::SI_SPILL_V1024_SAVE;
    */
  default:
    llvm_unreachable("unknown register size");
  }
}


void PPUInstrInfo::storeRegToStackSlot(MachineBasicBlock &MBB,
                                      MachineBasicBlock::iterator MI,
                                      unsigned SrcReg, bool isKill,
                                      int FrameIndex,
                                      const TargetRegisterClass *RC,
                                      const TargetRegisterInfo *TRI) const {
    /*
  MachineFunction *MF = MBB.getParent();
  PPUMachineFunctionInfo *MFI = MF->getInfo<PPUMachineFunctionInfo>();
  MachineFrameInfo &FrameInfo = MF->getFrameInfo();
  const DebugLoc &DL = MBB.findDebugLoc(MI);

  unsigned Size = FrameInfo.getObjectSize(FrameIndex);
  unsigned Align = FrameInfo.getObjectAlignment(FrameIndex);
  MachinePointerInfo PtrInfo
    = MachinePointerInfo::getFixedStack(*MF, FrameIndex);
  MachineMemOperand *MMO
    = MF->getMachineMemOperand(PtrInfo, MachineMemOperand::MOStore,
                               Size, Align);
  unsigned SpillSize = TRI->getSpillSize(*RC);

  if (RI.isSGPRClass(RC)) {
    MFI->setHasSpilledSGPRs();

    // We are only allowed to create one new instruction when spilling
    // registers, so we need to use pseudo instruction for spilling SGPRs.
    const MCInstrDesc &OpDesc = get(getSGPRSpillSaveOpcode(SpillSize));

    // The SGPR spill/restore instructions only work on number sgprs, so we need
    // to make sure we are using the correct register class.
    if (Register::isVirtualRegister(SrcReg) && SpillSize == 4) {
      MachineRegisterInfo &MRI = MF->getRegInfo();
      MRI.constrainRegClass(SrcReg, &PPU::GPRRegClass);
    }

    MachineInstrBuilder Spill = BuildMI(MBB, MI, DL, OpDesc)
      .addReg(SrcReg, getKillRegState(isKill)) // data
      .addFrameIndex(FrameIndex)               // addr
      .addMemOperand(MMO)
      .addReg(MFI->getScratchRSrcReg(), RegState::Implicit)
      .addReg(MFI->getStackPtrOffsetReg(), RegState::Implicit);
    // Add the scratch resource registers as implicit uses because we may end up
    // needing them, and need to ensure that the reserved registers are
    // correctly handled.
    if (RI.spillSGPRToVGPR())
      FrameInfo.setStackID(FrameIndex, TargetStackID::SGPRSpill);

    if (ST.hasScalarStores()) {
      // m0 is used for offset to scalar stores if used to spill.
      Spill.addReg(PPU::M0, RegState::ImplicitDefine | RegState::Dead);
    }

    return;
  }

  unsigned Opcode = getVGPRSpillSaveOpcode(SpillSize);
  MFI->setHasSpilledVGPRs();

  auto MIB = BuildMI(MBB, MI, DL, get(Opcode));
  MIB.addReg(SrcReg, getKillRegState(isKill)) // data
     .addFrameIndex(FrameIndex)               // addr
     .addReg(MFI->getScratchRSrcReg())        // scratch_rsrc
     .addReg(MFI->getStackPtrOffsetReg())     // scratch_offset
     .addImm(0)                               // offset
     .addMemOperand(MMO);
  */
}

static unsigned getSGPRSpillRestoreOpcode(unsigned Size) {
  switch (Size) {
      /*
  case 4:
    return PPU::SI_SPILL_S32_RESTORE;
  case 8:
    return PPU::SI_SPILL_S64_RESTORE;
  case 12:
    return PPU::SI_SPILL_S96_RESTORE;
  case 16:
    return PPU::SI_SPILL_S128_RESTORE;
  case 20:
    return PPU::SI_SPILL_S160_RESTORE;
  case 32:
    return PPU::SI_SPILL_S256_RESTORE;
  case 64:
    return PPU::SI_SPILL_S512_RESTORE;
  case 128:
    return PPU::SI_SPILL_S1024_RESTORE;
    */
  default:
    llvm_unreachable("unknown register size");
  }
}

static unsigned getVGPRSpillRestoreOpcode(unsigned Size) {
  switch (Size) {
      /*
  case 4:
    return PPU::SI_SPILL_V32_RESTORE;
  case 8:
    return PPU::SI_SPILL_V64_RESTORE;
  case 12:
    return PPU::SI_SPILL_V96_RESTORE;
  case 16:
    return PPU::SI_SPILL_V128_RESTORE;
  case 20:
    return PPU::SI_SPILL_V160_RESTORE;
  case 32:
    return PPU::SI_SPILL_V256_RESTORE;
  case 64:
    return PPU::SI_SPILL_V512_RESTORE;
  case 128:
    return PPU::SI_SPILL_V1024_RESTORE;
    */
  default:
    llvm_unreachable("unknown register size");
  }
}


void PPUInstrInfo::loadRegFromStackSlot(MachineBasicBlock &MBB,
                                       MachineBasicBlock::iterator MI,
                                       unsigned DestReg, int FrameIndex,
                                       const TargetRegisterClass *RC,
                                       const TargetRegisterInfo *TRI) const {
    /*
  MachineFunction *MF = MBB.getParent();
  PPUMachineFunctionInfo *MFI = MF->getInfo<PPUMachineFunctionInfo>();
  MachineFrameInfo &FrameInfo = MF->getFrameInfo();
  const DebugLoc &DL = MBB.findDebugLoc(MI);
  unsigned Align = FrameInfo.getObjectAlignment(FrameIndex);
  unsigned Size = FrameInfo.getObjectSize(FrameIndex);
  unsigned SpillSize = TRI->getSpillSize(*RC);

  MachinePointerInfo PtrInfo
    = MachinePointerInfo::getFixedStack(*MF, FrameIndex);

  MachineMemOperand *MMO = MF->getMachineMemOperand(
    PtrInfo, MachineMemOperand::MOLoad, Size, Align);

  if (RI.isSGPRClass(RC)) {
    MFI->setHasSpilledSGPRs();

    // FIXME: Maybe this should not include a memoperand because it will be
    // lowered to non-memory instructions.
    const MCInstrDesc &OpDesc = get(getSGPRSpillRestoreOpcode(SpillSize));
    if (Register::isVirtualRegister(DestReg) && SpillSize == 4) {
      MachineRegisterInfo &MRI = MF->getRegInfo();
      MRI.constrainRegClass(DestReg, &PPU::SReg_32_XM0RegClass);
    }

    if (RI.spillSGPRToVGPR())
      FrameInfo.setStackID(FrameIndex, TargetStackID::SGPRSpill);
    MachineInstrBuilder Spill = BuildMI(MBB, MI, DL, OpDesc, DestReg)
      .addFrameIndex(FrameIndex) // addr
      .addMemOperand(MMO)
      .addReg(MFI->getScratchRSrcReg(), RegState::Implicit)
      .addReg(MFI->getStackPtrOffsetReg(), RegState::Implicit);

    if (ST.hasScalarStores()) {
      // m0 is used for offset to scalar stores if used to spill.
      Spill.addReg(PPU::M0, RegState::ImplicitDefine | RegState::Dead);
    }

    return;
  }

  unsigned Opcode = getVGPRSpillRestoreOpcode(SpillSize);
  auto MIB = BuildMI(MBB, MI, DL, get(Opcode), DestReg);
  MIB.addFrameIndex(FrameIndex)        // vaddr
     .addReg(MFI->getScratchRSrcReg()) // scratch_rsrc
     .addReg(MFI->getStackPtrOffsetReg()) // scratch_offset
     .addImm(0)                           // offset
     .addMemOperand(MMO);
     */
}

/// \param @Offset Offset in bytes of the FrameIndex being spilled
unsigned PPUInstrInfo::calculateLDSSpillAddress(
    MachineBasicBlock &MBB, MachineInstr &MI, RegScavenger *RS, unsigned TmpReg,
    unsigned FrameOffset, unsigned Size) const {
  MachineFunction *MF = MBB.getParent();
  PPUMachineFunctionInfo *MFI = MF->getInfo<PPUMachineFunctionInfo>();
  const PPUSubtarget &ST = MF->getSubtarget<PPUSubtarget>();
  const DebugLoc &DL = MBB.findDebugLoc(MI);
  unsigned WorkGroupSize = MFI->getMaxFlatWorkGroupSize();
  unsigned WavefrontSize = ST.getWavefrontSize();

  unsigned TIDReg = MFI->getTIDReg();
  if (!MFI->hasCalculatedTID()) {
    MachineBasicBlock &Entry = MBB.getParent()->front();
    MachineBasicBlock::iterator Insert = Entry.front();
    const DebugLoc &DL = Insert->getDebugLoc();

    TIDReg = RI.findUnusedRegister(MF->getRegInfo(), &PPU::TPRRegClass,
                                   *MF);
    if (TIDReg == PPU::NoRegister)
      return TIDReg;

    if ( WorkGroupSize > WavefrontSize) {
      Register TIDIGXReg =
          MFI->getPreloadedReg(PPUFunctionArgInfo::WORKGROUP_ID_X);
      Register TIDIGYReg =
          MFI->getPreloadedReg(PPUFunctionArgInfo::WORKGROUP_ID_Y);
      Register TIDIGZReg =
          MFI->getPreloadedReg(PPUFunctionArgInfo::WORKGROUP_ID_Z);
      Register InputPtrReg =
          MFI->getPreloadedReg(PPUFunctionArgInfo::KERNARG_SEGMENT_PTR);
      for (unsigned Reg : {TIDIGXReg, TIDIGYReg, TIDIGZReg}) {
        if (!Entry.isLiveIn(Reg))
          Entry.addLiveIn(Reg);
      }

      RS->enterBasicBlock(Entry);
      // FIXME: Can we scavenge an SReg_64 and access the subregs?
      unsigned STmp0 = RS->scavengeRegister(&PPU::GPRRegClass, 0);
      unsigned STmp1 = RS->scavengeRegister(&PPU::GPRRegClass, 0);
      /* TODO
      BuildMI(Entry, Insert, DL, get(PPU::S_LOAD_DWORD_IMM), STmp0)
              .addReg(InputPtrReg)
              .addImm(SI::KernelInputOffsets::NGROUPS_Z);
      BuildMI(Entry, Insert, DL, get(PPU::S_LOAD_DWORD_IMM), STmp1)
              .addReg(InputPtrReg)
              .addImm(SI::KernelInputOffsets::NGROUPS_Y);
              */

      // NGROUPS.X * NGROUPS.Y
      BuildMI(Entry, Insert, DL, get(PPU::MUL), STmp1)
              .addReg(STmp1)
              .addReg(STmp0);
      // (NGROUPS.X * NGROUPS.Y) * TIDIG.X
      // BuildMI(Entry, Insert, DL, get(PPU::V_MUL_U32_U24_e32), TIDReg)
      BuildMI(Entry, Insert, DL, get(PPU::VMUL), TIDReg)
              .addReg(STmp1)
              .addReg(TIDIGXReg);
      // NGROUPS.Z * TIDIG.Y + (NGROUPS.X * NGROPUS.Y * TIDIG.X)
      /* TODO
      BuildMI(Entry, Insert, DL, get(PPU::V_MAD_U32_U24), TIDReg)
              .addReg(STmp0)
              .addReg(TIDIGYReg)
              .addReg(TIDReg);
              */
      // (NGROUPS.Z * TIDIG.Y + (NGROUPS.X * NGROPUS.Y * TIDIG.X)) + TIDIG.Z
      getAddNoCarry(Entry, Insert, DL, TIDReg)
        .addReg(TIDReg)
        .addReg(TIDIGZReg)
        .addImm(0); // clamp bit
    } else {
      // Get the wave id
      /* TODO
      BuildMI(Entry, Insert, DL, get(PPU::V_MBCNT_LO_U32_B32_e64),
              TIDReg)
              .addImm(-1)
              .addImm(0);

      BuildMI(Entry, Insert, DL, get(PPU::V_MBCNT_HI_U32_B32_e64),
              TIDReg)
              .addImm(-1)
              .addReg(TIDReg);
              */
    }
/* TODO
    BuildMI(Entry, Insert, DL, get(PPU::V_LSHLREV_B32_e32),
            TIDReg)
            .addImm(2)
            .addReg(TIDReg);
    MFI->setTIDReg(TIDReg);
    */
  }

  // Add FrameIndex to LDS offset
  unsigned LDSOffset = MFI->getLDSSize() + (FrameOffset * WorkGroupSize);
  getAddNoCarry(MBB, MI, DL, TmpReg)
    .addImm(LDSOffset)
    .addReg(TIDReg)
    .addImm(0); // clamp bit

  return TmpReg;
}

void PPUInstrInfo::insertWaitStates(MachineBasicBlock &MBB,
                                   MachineBasicBlock::iterator MI,
                                   int Count) const {
    /*
  DebugLoc DL = MBB.findDebugLoc(MI);
  while (Count > 0) {
    int Arg;
    if (Count >= 8)
      Arg = 7;
    else
      Arg = Count - 1;
    Count -= 8;
    BuildMI(MBB, MI, DL, get(PPU::NOP))
            .addImm(Arg);
  }
  */
}

void PPUInstrInfo::insertNoop(MachineBasicBlock &MBB,
                             MachineBasicBlock::iterator MI) const {
  insertWaitStates(MBB, MI, 1);
}

void PPUInstrInfo::insertReturn(MachineBasicBlock &MBB) const {
    /*
  auto MF = MBB.getParent();
  PPUMachineFunctionInfo *Info = MF->getInfo<PPUMachineFunctionInfo>();

  assert(Info->isEntryFunction());

  if (MBB.succ_empty()) {
    bool HasNoTerminator = MBB.getFirstTerminator() == MBB.end();
    if (HasNoTerminator) {
      if (Info->returnsVoid()) {
        BuildMI(MBB, MBB.end(), DebugLoc(), get(PPU::S_ENDPGM)).addImm(0);
      } else {
        BuildMI(MBB, MBB.end(), DebugLoc(), get(PPU::SI_RETURN_TO_EPILOG));
      }
    }
  }
  */
}

unsigned PPUInstrInfo::getNumWaitStates(const MachineInstr &MI) {
    /*
  switch (MI.getOpcode()) {
  default: return 1; // FIXME: Do wait states equal cycles?

  case PPU::S_NOP:
    return MI.getOperand(0).getImm() + 1;
  }
  */
    return 0;
}

bool PPUInstrInfo::expandPostRAPseudo(MachineInstr &MI) const {
  MachineBasicBlock &MBB = *MI.getParent();
  DebugLoc DL = MBB.findDebugLoc(MI);
  switch (MI.getOpcode()) {
  default: return TargetInstrInfo::expandPostRAPseudo(MI);
           /*
  case PPU::S_MOV_B64_term:
    // This is only a terminator to get the correct spill code placement during
    // register allocation.
    MI.setDesc(get(PPU::S_MOV_B64));
    break;

  case PPU::S_MOV_B32_term:
    // This is only a terminator to get the correct spill code placement during
    // register allocation.
    MI.setDesc(get(PPU::S_MOV_B32));
    break;

  case PPU::S_XOR_B64_term:
    // This is only a terminator to get the correct spill code placement during
    // register allocation.
    MI.setDesc(get(PPU::S_XOR_B64));
    break;

  case PPU::S_XOR_B32_term:
    // This is only a terminator to get the correct spill code placement during
    // register allocation.
    MI.setDesc(get(PPU::S_XOR_B32));
    break;

  case PPU::S_OR_B32_term:
    // This is only a terminator to get the correct spill code placement during
    // register allocation.
    MI.setDesc(get(PPU::S_OR_B32));
    break;

  case PPU::S_ANDN2_B64_term:
    // This is only a terminator to get the correct spill code placement during
    // register allocation.
    MI.setDesc(get(PPU::S_ANDN2_B64));
    break;

  case PPU::S_ANDN2_B32_term:
    // This is only a terminator to get the correct spill code placement during
    // register allocation.
    MI.setDesc(get(PPU::S_ANDN2_B32));
    break;

  case PPU::V_MOV_B64_PSEUDO: {
    Register Dst = MI.getOperand(0).getReg();
    Register DstLo = RI.getSubReg(Dst, PPU::sub0);
    Register DstHi = RI.getSubReg(Dst, PPU::sub1);

    const MachineOperand &SrcOp = MI.getOperand(1);
    // FIXME: Will this work for 64-bit floating point immediates?
    assert(!SrcOp.isFPImm());
    if (SrcOp.isImm()) {
      APInt Imm(64, SrcOp.getImm());
      BuildMI(MBB, MI, DL, get(PPU::V_MOV_B32_e32), DstLo)
        .addImm(Imm.getLoBits(32).getZExtValue())
        .addReg(Dst, RegState::Implicit | RegState::Define);
      BuildMI(MBB, MI, DL, get(PPU::V_MOV_B32_e32), DstHi)
        .addImm(Imm.getHiBits(32).getZExtValue())
        .addReg(Dst, RegState::Implicit | RegState::Define);
    } else {
      assert(SrcOp.isReg());
      BuildMI(MBB, MI, DL, get(PPU::V_MOV_B32_e32), DstLo)
        .addReg(RI.getSubReg(SrcOp.getReg(), PPU::sub0))
        .addReg(Dst, RegState::Implicit | RegState::Define);
      BuildMI(MBB, MI, DL, get(PPU::V_MOV_B32_e32), DstHi)
        .addReg(RI.getSubReg(SrcOp.getReg(), PPU::sub1))
        .addReg(Dst, RegState::Implicit | RegState::Define);
    }
    MI.eraseFromParent();
    break;
  }
  case PPU::V_SET_INACTIVE_B32: {
    unsigned NotOpc = ST.isWave32() ? PPU::S_NOT_B32 : PPU::S_NOT_B64;
    unsigned Exec = ST.isWave32() ? PPU::EXEC_LO : PPU::EXEC;
    BuildMI(MBB, MI, DL, get(NotOpc), Exec)
      .addReg(Exec);
    BuildMI(MBB, MI, DL, get(PPU::V_MOV_B32_e32), MI.getOperand(0).getReg())
      .add(MI.getOperand(2));
    BuildMI(MBB, MI, DL, get(NotOpc), Exec)
      .addReg(Exec);
    MI.eraseFromParent();
    break;
  }
  case PPU::V_SET_INACTIVE_B64: {
    unsigned NotOpc = ST.isWave32() ? PPU::S_NOT_B32 : PPU::S_NOT_B64;
    unsigned Exec = ST.isWave32() ? PPU::EXEC_LO : PPU::EXEC;
    BuildMI(MBB, MI, DL, get(NotOpc), Exec)
      .addReg(Exec);
    MachineInstr *Copy = BuildMI(MBB, MI, DL, get(PPU::V_MOV_B64_PSEUDO),
                                 MI.getOperand(0).getReg())
      .add(MI.getOperand(2));
    expandPostRAPseudo(*Copy);
    BuildMI(MBB, MI, DL, get(NotOpc), Exec)
      .addReg(Exec);
    MI.eraseFromParent();
    break;
  }
  case PPU::V_MOVRELD_B32_V1:
  case PPU::V_MOVRELD_B32_V2:
  case PPU::V_MOVRELD_B32_V4:
  case PPU::V_MOVRELD_B32_V8:
  case PPU::V_MOVRELD_B32_V16: {
    const MCInstrDesc &MovRelDesc = get(PPU::V_MOVRELD_B32_e32);
    Register VecReg = MI.getOperand(0).getReg();
    bool IsUndef = MI.getOperand(1).isUndef();
    unsigned SubReg = PPU::sub0 + MI.getOperand(3).getImm();
    assert(VecReg == MI.getOperand(1).getReg());

    MachineInstr *MovRel =
        BuildMI(MBB, MI, DL, MovRelDesc)
            .addReg(RI.getSubReg(VecReg, SubReg), RegState::Undef)
            .add(MI.getOperand(2))
            .addReg(VecReg, RegState::ImplicitDefine)
            .addReg(VecReg,
                    RegState::Implicit | (IsUndef ? RegState::Undef : 0));

    const int ImpDefIdx =
        MovRelDesc.getNumOperands() + MovRelDesc.getNumImplicitUses();
    const int ImpUseIdx = ImpDefIdx + 1;
    MovRel->tieOperands(ImpDefIdx, ImpUseIdx);

    MI.eraseFromParent();
    break;
  }
  case PPU::SI_PC_ADD_REL_OFFSET: {
    MachineFunction &MF = *MBB.getParent();
    Register Reg = MI.getOperand(0).getReg();
    Register RegLo = RI.getSubReg(Reg, PPU::sub0);
    Register RegHi = RI.getSubReg(Reg, PPU::sub1);

    // Create a bundle so these instructions won't be re-ordered by the
    // post-RA scheduler.
    MIBundleBuilder Bundler(MBB, MI);
    Bundler.append(BuildMI(MF, DL, get(PPU::S_GETPC_B64), Reg));

    // Add 32-bit offset from this instruction to the start of the
    // constant data.
    Bundler.append(BuildMI(MF, DL, get(PPU::S_ADD_U32), RegLo)
                       .addReg(RegLo)
                       .add(MI.getOperand(1)));

    MachineInstrBuilder MIB = BuildMI(MF, DL, get(PPU::S_ADDC_U32), RegHi)
                                  .addReg(RegHi);
    MIB.add(MI.getOperand(2));

    Bundler.append(MIB);
    finalizeBundle(MBB, Bundler.begin());

    MI.eraseFromParent();
    break;
  }
  case PPU::ENTER_WWM: {
    // This only gets its own opcode so that SIPreAllocateWWMRegs can tell when
    // WWM is entered.
    MI.setDesc(get(ST.isWave32() ? PPU::S_OR_SAVEEXEC_B32
                                 : PPU::S_OR_SAVEEXEC_B64));
    break;
  }
  case PPU::EXIT_WWM: {
    // This only gets its own opcode so that SIPreAllocateWWMRegs can tell when
    // WWM is exited.
    MI.setDesc(get(ST.isWave32() ? PPU::S_MOV_B32 : PPU::S_MOV_B64));
    break;
  }
  case TargetOpcode::BUNDLE: {
    if (!MI.mayLoad() || MI.hasUnmodeledSideEffects())
      return false;

    // If it is a load it must be a memory clause
    for (MachineBasicBlock::instr_iterator I = MI.getIterator();
         I->isBundledWithSucc(); ++I) {
      I->unbundleFromSucc();
      for (MachineOperand &MO : I->operands())
        if (MO.isReg())
          MO.setIsInternalRead(false);
    }

    MI.eraseFromParent();
    break;
  }
    */
  }
  return true;
}

bool PPUInstrInfo::swapSourceModifiers(MachineInstr &MI,
                                      MachineOperand &Src0,
                                      unsigned Src0OpName,
                                      MachineOperand &Src1,
                                      unsigned Src1OpName) const {
  MachineOperand *Src0Mods = getNamedOperand(MI, Src0OpName);
  if (!Src0Mods)
    return false;

  MachineOperand *Src1Mods = getNamedOperand(MI, Src1OpName);
  assert(Src1Mods &&
         "All commutable instructions have both src0 and src1 modifiers");

  int Src0ModsVal = Src0Mods->getImm();
  int Src1ModsVal = Src1Mods->getImm();

  Src1Mods->setImm(Src0ModsVal);
  Src0Mods->setImm(Src1ModsVal);
  return true;
}

static MachineInstr *swapRegAndNonRegOperand(MachineInstr &MI,
                                             MachineOperand &RegOp,
                                             MachineOperand &NonRegOp) {
  Register Reg = RegOp.getReg();
  unsigned SubReg = RegOp.getSubReg();
  bool IsKill = RegOp.isKill();
  bool IsDead = RegOp.isDead();
  bool IsUndef = RegOp.isUndef();
  bool IsDebug = RegOp.isDebug();

  if (NonRegOp.isImm())
    RegOp.ChangeToImmediate(NonRegOp.getImm());
  else if (NonRegOp.isFI())
    RegOp.ChangeToFrameIndex(NonRegOp.getIndex());
  else
    return nullptr;

  NonRegOp.ChangeToRegister(Reg, false, false, IsKill, IsDead, IsUndef, IsDebug);
  NonRegOp.setSubReg(SubReg);

  return &MI;
}

MachineInstr *PPUInstrInfo::commuteInstructionImpl(MachineInstr &MI, bool NewMI,
                                                  unsigned Src0Idx,
                                                  unsigned Src1Idx) const {
  assert(!NewMI && "this should never be used");
  /*

  unsigned Opc = MI.getOpcode();
  int CommutedOpcode = commuteOpcode(Opc);
  if (CommutedOpcode == -1)
    return nullptr;

  assert(PPU::getNamedOperandIdx(Opc, PPU::OpName::src0) ==
           static_cast<int>(Src0Idx) &&
         PPU::getNamedOperandIdx(Opc, PPU::OpName::src1) ==
           static_cast<int>(Src1Idx) &&
         "inconsistency with findCommutedOpIndices");

  MachineOperand &Src0 = MI.getOperand(Src0Idx);
  MachineOperand &Src1 = MI.getOperand(Src1Idx);

  MachineInstr *CommutedMI = nullptr;
  if (Src0.isReg() && Src1.isReg()) {
    if (isOperandLegal(MI, Src1Idx, &Src0)) {
      // Be sure to copy the source modifiers to the right place.
      CommutedMI
        = TargetInstrInfo::commuteInstructionImpl(MI, NewMI, Src0Idx, Src1Idx);
    }

  } else if (Src0.isReg() && !Src1.isReg()) {
    // src0 should always be able to support any operand type, so no need to
    // check operand legality.
    CommutedMI = swapRegAndNonRegOperand(MI, Src0, Src1);
  } else if (!Src0.isReg() && Src1.isReg()) {
    if (isOperandLegal(MI, Src1Idx, &Src0))
      CommutedMI = swapRegAndNonRegOperand(MI, Src1, Src0);
  } else {
    // FIXME: Found two non registers to commute. This does happen.
    return nullptr;
  }

  if (CommutedMI) {
    swapSourceModifiers(MI, Src0, PPU::OpName::src0_modifiers,
                        Src1, PPU::OpName::src1_modifiers);

    CommutedMI->setDesc(get(CommutedOpcode));
  }

  return CommutedMI;
  */
  MachineInstr *CommutedMI = nullptr;
  return CommutedMI;
}

// This needs to be implemented because the source modifiers may be inserted
// between the true commutable operands, and the base
// TargetInstrInfo::commuteInstruction uses it.
bool PPUInstrInfo::findCommutedOpIndices(MachineInstr &MI, unsigned &SrcOpIdx0,
                                        unsigned &SrcOpIdx1) const {
  return findCommutedOpIndices(MI.getDesc(), SrcOpIdx0, SrcOpIdx1);
}

bool PPUInstrInfo::findCommutedOpIndices(MCInstrDesc Desc, unsigned &SrcOpIdx0,
                                        unsigned &SrcOpIdx1) const {
    /*
  if (!Desc.isCommutable())
    return false;

  unsigned Opc = Desc.getOpcode();
  int Src0Idx = PPU::getNamedOperandIdx(Opc, PPU::OpName::src0);
  if (Src0Idx == -1)
    return false;

  int Src1Idx = PPU::getNamedOperandIdx(Opc, PPU::OpName::src1);
  if (Src1Idx == -1)
    return false;

  return fixCommutedOpIndices(SrcOpIdx0, SrcOpIdx1, Src0Idx, Src1Idx);
  */
    return false;
}

bool PPUInstrInfo::isBranchOffsetInRange(unsigned BranchOp,
                                        int64_t BrOffset) const {
    /*
  // BranchRelaxation should never have to check s_setpc_b64 because its dest
  // block is unanalyzable.
  assert(BranchOp != PPU::S_SETPC_B64);

  // Convert to dwords.
  BrOffset /= 4;

  // The branch instructions do PC += signext(SIMM16 * 4) + 4, so the offset is
  // from the next instruction.
  BrOffset -= 1;

  return isIntN(BranchOffsetBits, BrOffset);
  */
}

MachineBasicBlock *PPUInstrInfo::getBranchDestBlock(
  const MachineInstr &MI) const {
    /*
  if (MI.getOpcode() == PPU::S_SETPC_B64) {
    // This would be a difficult analysis to perform, but can always be legal so
    // there's no need to analyze it.
    return nullptr;
  }
  */

  return MI.getOperand(0).getMBB();
}

unsigned PPUInstrInfo::insertIndirectBranch(MachineBasicBlock &MBB,
                                           MachineBasicBlock &DestBB,
                                           const DebugLoc &DL,
                                           int64_t BrOffset,
                                           RegScavenger *RS) const {
  assert(RS && "RegScavenger required for long branching");
  assert(MBB.empty() &&
         "new block should be inserted for expanding unconditional branch");
  assert(MBB.pred_size() == 1);
/* FIXME
  MachineFunction *MF = MBB.getParent();
  MachineRegisterInfo &MRI = MF->getRegInfo();

  // FIXME: Virtual register workaround for RegScavenger not working with empty
  // blocks.
  Register PCReg = MRI.createVirtualRegister(&PPU::SReg_64RegClass);

  auto I = MBB.end();

  // We need to compute the offset relative to the instruction immediately after
  // s_getpc_b64. Insert pc arithmetic code before last terminator.
  MachineInstr *GetPC = BuildMI(MBB, I, DL, get(PPU::S_GETPC_B64), PCReg);

  // TODO: Handle > 32-bit block address.
  if (BrOffset >= 0) {
    BuildMI(MBB, I, DL, get(PPU::S_ADD_U32))
      .addReg(PCReg, RegState::Define, PPU::sub0)
      .addReg(PCReg, 0, PPU::sub0)
      .addMBB(&DestBB, MO_LONG_BRANCH_FORWARD);
    BuildMI(MBB, I, DL, get(PPU::S_ADDC_U32))
      .addReg(PCReg, RegState::Define, PPU::sub1)
      .addReg(PCReg, 0, PPU::sub1)
      .addImm(0);
  } else {
    // Backwards branch.
    BuildMI(MBB, I, DL, get(PPU::S_SUB_U32))
      .addReg(PCReg, RegState::Define, PPU::sub0)
      .addReg(PCReg, 0, PPU::sub0)
      .addMBB(&DestBB, MO_LONG_BRANCH_BACKWARD);
    BuildMI(MBB, I, DL, get(PPU::S_SUBB_U32))
      .addReg(PCReg, RegState::Define, PPU::sub1)
      .addReg(PCReg, 0, PPU::sub1)
      .addImm(0);
  }

  // Insert the indirect branch after the other terminator.
  BuildMI(&MBB, DL, get(PPU::S_SETPC_B64))
    .addReg(PCReg);

  // FIXME: If spilling is necessary, this will fail because this scavenger has
  // no emergency stack slots. It is non-trivial to spill in this situation,
  // because the restore code needs to be specially placed after the
  // jump. BranchRelaxation then needs to be made aware of the newly inserted
  // block.
  //
  // If a spill is needed for the pc register pair, we need to insert a spill
  // restore block right before the destination block, and insert a short branch
  // into the old destination block's fallthrough predecessor.
  // e.g.:
  //
  // s_cbranch_scc0 skip_long_branch:
  //
  // long_branch_bb:
  //   spill s[8:9]
  //   s_getpc_b64 s[8:9]
  //   s_add_u32 s8, s8, restore_bb
  //   s_addc_u32 s9, s9, 0
  //   s_setpc_b64 s[8:9]
  //
  // skip_long_branch:
  //   foo;
  //
  // .....
  //
  // dest_bb_fallthrough_predecessor:
  // bar;
  // s_branch dest_bb
  //
  // restore_bb:
  //  restore s[8:9]
  //  fallthrough dest_bb
  ///
  // dest_bb:
  //   buzz;

  RS->enterBasicBlockEnd(MBB);
  unsigned Scav = RS->scavengeRegisterBackwards(
    PPU::SReg_64RegClass,
    MachineBasicBlock::iterator(GetPC), false, 0);
  MRI.replaceRegWith(PCReg, Scav);
  MRI.clearVirtRegs();
  RS->setRegUsed(Scav);
*/
  return 4 + 8 + 4 + 4;
}

unsigned PPUInstrInfo::getBranchOpcode(PPUInstrInfo::BranchPredicate Cond) {
  switch (Cond) {
      /*
  case PPUInstrInfo::SCC_TRUE:
    return PPU::S_CBRANCH_SCC1;
  case PPUInstrInfo::SCC_FALSE:
    return PPU::S_CBRANCH_SCC0;
  case PPUInstrInfo::VCCNZ:
    return PPU::S_CBRANCH_VCCNZ;
  case PPUInstrInfo::VCCZ:
    return PPU::S_CBRANCH_VCCZ;
  case PPUInstrInfo::EXECNZ:
    return PPU::S_CBRANCH_EXECNZ;
  case PPUInstrInfo::EXECZ:
    return PPU::S_CBRANCH_EXECZ;
    */
  default:
    llvm_unreachable("invalid branch predicate");
  }
}

PPUInstrInfo::BranchPredicate PPUInstrInfo::getBranchPredicate(unsigned Opcode) {
  switch (Opcode) {
      /*
  case PPU::S_CBRANCH_SCC0:
    return SCC_FALSE;
  case PPU::S_CBRANCH_SCC1:
    return SCC_TRUE;
  case PPU::S_CBRANCH_VCCNZ:
    return VCCNZ;
  case PPU::S_CBRANCH_VCCZ:
    return VCCZ;
  case PPU::S_CBRANCH_EXECNZ:
    return EXECNZ;
  case PPU::S_CBRANCH_EXECZ:
    return EXECZ;
    */
  default:
    return INVALID_BR;
  }
}

bool PPUInstrInfo::analyzeBranchImpl(MachineBasicBlock &MBB,
                                    MachineBasicBlock::iterator I,
                                    MachineBasicBlock *&TBB,
                                    MachineBasicBlock *&FBB,
                                    SmallVectorImpl<MachineOperand> &Cond,
                                    bool AllowModify) const {
    /*
  if (I->getOpcode() == PPU::S_BRANCH) {
    // Unconditional Branch
    TBB = I->getOperand(0).getMBB();
    return false;
  }

  MachineBasicBlock *CondBB = nullptr;

  if (I->getOpcode() == PPU::SI_NON_UNIFORM_BRCOND_PSEUDO) {
    CondBB = I->getOperand(1).getMBB();
    Cond.push_back(I->getOperand(0));
  } else {
    BranchPredicate Pred = getBranchPredicate(I->getOpcode());
    if (Pred == INVALID_BR)
      return true;

    CondBB = I->getOperand(0).getMBB();
    Cond.push_back(MachineOperand::CreateImm(Pred));
    Cond.push_back(I->getOperand(1)); // Save the branch register.
  }
  ++I;

  if (I == MBB.end()) {
    // Conditional branch followed by fall-through.
    TBB = CondBB;
    return false;
  }

  if (I->getOpcode() == PPU::S_BRANCH) {
    TBB = CondBB;
    FBB = I->getOperand(0).getMBB();
    return false;
  }
  */

  return true;
}

bool PPUInstrInfo::analyzeBranch(MachineBasicBlock &MBB, MachineBasicBlock *&TBB,
                                MachineBasicBlock *&FBB,
                                SmallVectorImpl<MachineOperand> &Cond,
                                bool AllowModify) const {
    /*
  MachineBasicBlock::iterator I = MBB.getFirstTerminator();
  auto E = MBB.end();
  if (I == E)
    return false;

  // Skip over the instructions that are artificially terminators for special
  // exec management.
  while (I != E && !I->isBranch() && !I->isReturn() &&
         I->getOpcode() != PPU::SI_MASK_BRANCH) {
    switch (I->getOpcode()) {
    case PPU::SI_MASK_BRANCH:
    case PPU::S_MOV_B64_term:
    case PPU::S_XOR_B64_term:
    case PPU::S_ANDN2_B64_term:
    case PPU::S_MOV_B32_term:
    case PPU::S_XOR_B32_term:
    case PPU::S_OR_B32_term:
    case PPU::S_ANDN2_B32_term:
      break;
    case PPU::SI_IF:
    case PPU::SI_ELSE:
    case PPU::SI_KILL_I1_TERMINATOR:
    case PPU::SI_KILL_F32_COND_IMM_TERMINATOR:
      // FIXME: It's messy that these need to be considered here at all.
      return true;
    default:
      llvm_unreachable("unexpected non-branch terminator inst");
    }

    ++I;
  }

  if (I == E)
    return false;

  if (I->getOpcode() != PPU::SI_MASK_BRANCH)
    return analyzeBranchImpl(MBB, I, TBB, FBB, Cond, AllowModify);

  ++I;

  // TODO: Should be able to treat as fallthrough?
  if (I == MBB.end())
    return true;

  if (analyzeBranchImpl(MBB, I, TBB, FBB, Cond, AllowModify))
    return true;

  MachineBasicBlock *MaskBrDest = I->getOperand(0).getMBB();

  // Specifically handle the case where the conditional branch is to the same
  // destination as the mask branch. e.g.
  //
  // si_mask_branch BB8
  // s_cbranch_execz BB8
  // s_cbranch BB9
  //
  // This is required to understand divergent loops which may need the branches
  // to be relaxed.
  if (TBB != MaskBrDest || Cond.empty())
    return true;

  auto Pred = Cond[0].getImm();
  return (Pred != EXECZ && Pred != EXECNZ);
  */
    return false;
}

unsigned PPUInstrInfo::removeBranch(MachineBasicBlock &MBB,
                                   int *BytesRemoved) const {
    /*
  MachineBasicBlock::iterator I = MBB.getFirstTerminator();

  unsigned Count = 0;
  unsigned RemovedSize = 0;
  while (I != MBB.end()) {
    MachineBasicBlock::iterator Next = std::next(I);
    if (I->getOpcode() == PPU::SI_MASK_BRANCH) {
      I = Next;
      continue;
    }

    RemovedSize += getInstSizeInBytes(*I);
    I->eraseFromParent();
    ++Count;
    I = Next;
  }

  if (BytesRemoved)
    *BytesRemoved = RemovedSize;

  return Count;
  */
    return 0;
}

// Copy the flags onto the implicit condition register operand.
static void preserveCondRegFlags(MachineOperand &CondReg,
                                 const MachineOperand &OrigCond) {
  CondReg.setIsUndef(OrigCond.isUndef());
  CondReg.setIsKill(OrigCond.isKill());
}

unsigned PPUInstrInfo::insertBranch(MachineBasicBlock &MBB,
                                   MachineBasicBlock *TBB,
                                   MachineBasicBlock *FBB,
                                   ArrayRef<MachineOperand> Cond,
                                   const DebugLoc &DL,
                                   int *BytesAdded) const {
    /*
  if (!FBB && Cond.empty()) {
    BuildMI(&MBB, DL, get(PPU::S_BRANCH))
      .addMBB(TBB);
    if (BytesAdded)
      *BytesAdded = 4;
    return 1;
  }

  if(Cond.size() == 1 && Cond[0].isReg()) {
     BuildMI(&MBB, DL, get(PPU::SI_NON_UNIFORM_BRCOND_PSEUDO))
       .add(Cond[0])
       .addMBB(TBB);
     return 1;
  }

  assert(TBB && Cond[0].isImm());

  unsigned Opcode
    = getBranchOpcode(static_cast<BranchPredicate>(Cond[0].getImm()));

  if (!FBB) {
    Cond[1].isUndef();
    MachineInstr *CondBr =
      BuildMI(&MBB, DL, get(Opcode))
      .addMBB(TBB);

    // Copy the flags onto the implicit condition register operand.
    preserveCondRegFlags(CondBr->getOperand(1), Cond[1]);

    if (BytesAdded)
      *BytesAdded = 4;
    return 1;
  }

  assert(TBB && FBB);

  MachineInstr *CondBr =
    BuildMI(&MBB, DL, get(Opcode))
    .addMBB(TBB);
  BuildMI(&MBB, DL, get(PPU::S_BRANCH))
    .addMBB(FBB);

  MachineOperand &CondReg = CondBr->getOperand(1);
  CondReg.setIsUndef(Cond[1].isUndef());
  CondReg.setIsKill(Cond[1].isKill());

  if (BytesAdded)
      *BytesAdded = 8;

  return 2;
  */
    return 0;
}

bool PPUInstrInfo::reverseBranchCondition(
  SmallVectorImpl<MachineOperand> &Cond) const {
  if (Cond.size() != 2) {
    return true;
  }

  if (Cond[0].isImm()) {
    Cond[0].setImm(-Cond[0].getImm());
    return false;
  }

  return true;
}

bool PPUInstrInfo::canInsertSelect(const MachineBasicBlock &MBB,
                                  ArrayRef<MachineOperand> Cond,
                                  unsigned TrueReg, unsigned FalseReg,
                                  int &CondCycles,
                                  int &TrueCycles, int &FalseCycles) const {
  switch (Cond[0].getImm()) {
    /*
  case VCCNZ:
  case VCCZ: {
    const MachineRegisterInfo &MRI = MBB.getParent()->getRegInfo();
    const TargetRegisterClass *RC = MRI.getRegClass(TrueReg);
    assert(MRI.getRegClass(FalseReg) == RC);

    int NumInsts = PPU::getRegBitWidth(RC->getID()) / 32;
    CondCycles = TrueCycles = FalseCycles = NumInsts; // ???

    // Limit to equal cost for branch vs. N v_cndmask_b32s.
    return RI.hasVGPRs(RC) && NumInsts <= 6;
  }
  case SCC_TRUE:
  case SCC_FALSE: {
    // FIXME: We could insert for VGPRs if we could replace the original compare
    // with a vector one.
    const MachineRegisterInfo &MRI = MBB.getParent()->getRegInfo();
    const TargetRegisterClass *RC = MRI.getRegClass(TrueReg);
    assert(MRI.getRegClass(FalseReg) == RC);

    int NumInsts = PPU::getRegBitWidth(RC->getID()) / 32;

    // Multiples of 8 can do s_cselect_b64
    if (NumInsts % 2 == 0)
      NumInsts /= 2;

    CondCycles = TrueCycles = FalseCycles = NumInsts; // ???
    return RI.isSGPRClass(RC);
  }
  */
  default:
    return false;
  }
}

void PPUInstrInfo::insertSelect(MachineBasicBlock &MBB,
                               MachineBasicBlock::iterator I, const DebugLoc &DL,
                               unsigned DstReg, ArrayRef<MachineOperand> Cond,
                               unsigned TrueReg, unsigned FalseReg) const {
    /*
  BranchPredicate Pred = static_cast<BranchPredicate>(Cond[0].getImm());
  if (Pred == VCCZ || Pred == SCC_FALSE) {
    Pred = static_cast<BranchPredicate>(-Pred);
    std::swap(TrueReg, FalseReg);
  }

  MachineRegisterInfo &MRI = MBB.getParent()->getRegInfo();
  const TargetRegisterClass *DstRC = MRI.getRegClass(DstReg);
  unsigned DstSize = RI.getRegSizeInBits(*DstRC);

  if (DstSize == 32) {
    unsigned SelOp = Pred == SCC_TRUE ?
      PPU::S_CSELECT_B32 : PPU::V_CNDMASK_B32_e32;

    // Instruction's operands are backwards from what is expected.
    MachineInstr *Select =
      BuildMI(MBB, I, DL, get(SelOp), DstReg)
      .addReg(FalseReg)
      .addReg(TrueReg);

    preserveCondRegFlags(Select->getOperand(3), Cond[1]);
    return;
  }

  if (DstSize == 64 && Pred == SCC_TRUE) {
    MachineInstr *Select =
      BuildMI(MBB, I, DL, get(PPU::S_CSELECT_B64), DstReg)
      .addReg(FalseReg)
      .addReg(TrueReg);

    preserveCondRegFlags(Select->getOperand(3), Cond[1]);
    return;
  }

  static const int16_t Sub0_15[] = {
    PPU::sub0, PPU::sub1, PPU::sub2, PPU::sub3,
    PPU::sub4, PPU::sub5, PPU::sub6, PPU::sub7,
    PPU::sub8, PPU::sub9, PPU::sub10, PPU::sub11,
    PPU::sub12, PPU::sub13, PPU::sub14, PPU::sub15,
  };

  static const int16_t Sub0_15_64[] = {
    PPU::sub0_sub1, PPU::sub2_sub3,
    PPU::sub4_sub5, PPU::sub6_sub7,
    PPU::sub8_sub9, PPU::sub10_sub11,
    PPU::sub12_sub13, PPU::sub14_sub15,
  };

  unsigned SelOp = PPU::V_CNDMASK_B32_e32;
  const TargetRegisterClass *EltRC = &PPU::TPRRegClass;
  const int16_t *SubIndices = Sub0_15;
  int NElts = DstSize / 32;

  // 64-bit select is only available for SALU.
  // TODO: Split 96-bit into 64-bit and 32-bit, not 3x 32-bit.
  if (Pred == SCC_TRUE) {
    if (NElts % 2) {
      SelOp = PPU::S_CSELECT_B32;
      EltRC = &PPU::GPRRegClass;
    } else {
      SelOp = PPU::S_CSELECT_B64;
      EltRC = &PPU::SGPR_64RegClass;
      SubIndices = Sub0_15_64;
      NElts /= 2;
    }
  }

  MachineInstrBuilder MIB = BuildMI(
    MBB, I, DL, get(PPU::REG_SEQUENCE), DstReg);

  I = MIB->getIterator();

  SmallVector<unsigned, 8> Regs;
  for (int Idx = 0; Idx != NElts; ++Idx) {
    Register DstElt = MRI.createVirtualRegister(EltRC);
    Regs.push_back(DstElt);

    unsigned SubIdx = SubIndices[Idx];

    MachineInstr *Select =
      BuildMI(MBB, I, DL, get(SelOp), DstElt)
      .addReg(FalseReg, 0, SubIdx)
      .addReg(TrueReg, 0, SubIdx);
    preserveCondRegFlags(Select->getOperand(3), Cond[1]);
    fixImplicitOperands(*Select);

    MIB.addReg(DstElt)
       .addImm(SubIdx);
  }
  */
}

bool PPUInstrInfo::isFoldableCopy(const MachineInstr &MI) const {
  switch (MI.getOpcode()) {
  case PPU::VMOV:
  {
    // If there are additional implicit register operands, this may be used for
    // register indexing so the source register operand isn't simply copied.
    unsigned NumOps = MI.getDesc().getNumOperands() +
      MI.getDesc().getNumImplicitUses();

    return MI.getNumOperands() == NumOps;
  }
  case PPU::SMOV:
  /*
  case PPU::COPY:
  case PPU::V_ACCVGPR_WRITE_B32:
  case PPU::V_ACCVGPR_READ_B32:
  */
    return true;
  default:
    return false;
  }
}

unsigned PPUInstrInfo::getAddressSpaceForPseudoSourceKind(
    unsigned Kind) const {
  switch(Kind) {
  case PseudoSourceValue::Stack:
  case PseudoSourceValue::FixedStack:
    return AMDGPUAS::PRIVATE_ADDRESS;
  case PseudoSourceValue::ConstantPool:
  case PseudoSourceValue::GOT:
  case PseudoSourceValue::JumpTable:
  case PseudoSourceValue::GlobalValueCallEntry:
  case PseudoSourceValue::ExternalSymbolCallEntry:
  case PseudoSourceValue::TargetCustom:
    return AMDGPUAS::CONSTANT_ADDRESS;
  }
  return AMDGPUAS::FLAT_ADDRESS;
}

static void removeModOperands(MachineInstr &MI) {
    /*
  unsigned Opc = MI.getOpcode();
  int Src0ModIdx = PPU::getNamedOperandIdx(Opc,
                                              PPU::OpName::src0_modifiers);
  int Src1ModIdx = PPU::getNamedOperandIdx(Opc,
                                              PPU::OpName::src1_modifiers);
  int Src2ModIdx = PPU::getNamedOperandIdx(Opc,
                                              PPU::OpName::src2_modifiers);

  MI.RemoveOperand(Src2ModIdx);
  MI.RemoveOperand(Src1ModIdx);
  MI.RemoveOperand(Src0ModIdx);
  */
}

bool PPUInstrInfo::FoldImmediate(MachineInstr &UseMI, MachineInstr &DefMI,
                                unsigned Reg, MachineRegisterInfo *MRI) const {
    /*
  if (!MRI->hasOneNonDBGUse(Reg))
    return false;

  switch (DefMI.getOpcode()) {
  default:
    return false;
  
  case PPU::VMOV:
  case PPU::SMOV:
    break;
  }

  const MachineOperand *ImmOp = getNamedOperand(DefMI, PPU::OpName::src0);
  assert(ImmOp);
  // FIXME: We could handle FrameIndex values here.
  if (!ImmOp->isImm())
    return false;

  unsigned Opc = UseMI.getOpcode();
  if (Opc == PPU::COPY) {
    bool isVGPRCopy = RI.isVGPR(*MRI, UseMI.getOperand(0).getReg());
    unsigned NewOpc = isVGPRCopy ? PPU::VMOV : PPU::SMOV;
    UseMI.setDesc(get(NewOpc));
    UseMI.getOperand(1).ChangeToImmediate(ImmOp->getImm());
    UseMI.addImplicitDefUseOperands(*UseMI.getParent()->getParent());
    return true;
  }
  */
/*
  if (Opc == PPU::V_MAD_F32 || Opc == PPU::V_MAC_F32_e64 ||
      Opc == PPU::V_MAD_F16 || Opc == PPU::V_MAC_F16_e64 ||
      Opc == PPU::V_FMA_F32 || Opc == PPU::V_FMAC_F32_e64 ||
      Opc == PPU::V_FMA_F16 || Opc == PPU::V_FMAC_F16_e64) {
    // Don't fold if we are using source or output modifiers. The new VOP2
    // instructions don't have them.
    if (hasAnyModifiersSet(UseMI))
      return false;

    // If this is a free constant, there's no reason to do this.
    // TODO: We could fold this here instead of letting SIFoldOperands do it
    // later.
    MachineOperand *Src0 = getNamedOperand(UseMI, PPU::OpName::src0);

    // Any src operand can be used for the legality check.
    if (isInlineConstant(UseMI, *Src0, *ImmOp))
      return false;

    bool IsF32 = Opc == PPU::V_MAD_F32 || Opc == PPU::V_MAC_F32_e64 ||
                 Opc == PPU::V_FMA_F32 || Opc == PPU::V_FMAC_F32_e64;
    bool IsFMA = Opc == PPU::V_FMA_F32 || Opc == PPU::V_FMAC_F32_e64 ||
                 Opc == PPU::V_FMA_F16 || Opc == PPU::V_FMAC_F16_e64;
    MachineOperand *Src1 = getNamedOperand(UseMI, PPU::OpName::src1);
    MachineOperand *Src2 = getNamedOperand(UseMI, PPU::OpName::src2);

    // Multiplied part is the constant: Use v_madmk_{f16, f32}.
    // We should only expect these to be on src0 due to canonicalizations.
    if (Src0->isReg() && Src0->getReg() == Reg) {
      if (!Src1->isReg() || RI.isSGPRClass(MRI->getRegClass(Src1->getReg())))
        return false;

      if (!Src2->isReg() || RI.isSGPRClass(MRI->getRegClass(Src2->getReg())))
        return false;

      unsigned NewOpc =
        IsFMA ? (IsF32 ? PPU::V_FMAMK_F32 : PPU::V_FMAMK_F16)
              : (IsF32 ? PPU::V_MADMK_F32 : PPU::V_MADMK_F16);
      if (pseudoToMCOpcode(NewOpc) == -1)
        return false;

      // We need to swap operands 0 and 1 since madmk constant is at operand 1.

      const int64_t Imm = ImmOp->getImm();

      // FIXME: This would be a lot easier if we could return a new instruction
      // instead of having to modify in place.

      // Remove these first since they are at the end.
      UseMI.RemoveOperand(
          PPU::getNamedOperandIdx(Opc, PPU::OpName::omod));
      UseMI.RemoveOperand(
          PPU::getNamedOperandIdx(Opc, PPU::OpName::clamp));

      Register Src1Reg = Src1->getReg();
      unsigned Src1SubReg = Src1->getSubReg();
      Src0->setReg(Src1Reg);
      Src0->setSubReg(Src1SubReg);
      Src0->setIsKill(Src1->isKill());

      if (Opc == PPU::V_MAC_F32_e64 ||
          Opc == PPU::V_MAC_F16_e64 ||
          Opc == PPU::V_FMAC_F32_e64 ||
          Opc == PPU::V_FMAC_F16_e64)
        UseMI.untieRegOperand(
            PPU::getNamedOperandIdx(Opc, PPU::OpName::src2));

      Src1->ChangeToImmediate(Imm);

      removeModOperands(UseMI);
      UseMI.setDesc(get(NewOpc));

      bool DeleteDef = MRI->hasOneNonDBGUse(Reg);
      if (DeleteDef)
        DefMI.eraseFromParent();

      return true;
    }

    // Added part is the constant: Use v_madak_{f16, f32}.
    if (Src2->isReg() && Src2->getReg() == Reg) {
      // Not allowed to use constant bus for another operand.
      // We can however allow an inline immediate as src0.
      bool Src0Inlined = false;
      if (Src0->isReg()) {
        // Try to inline constant if possible.
        // If the Def moves immediate and the use is single
        // We are saving VGPR here.
        MachineInstr *Def = MRI->getUniqueVRegDef(Src0->getReg());
        if (Def && Def->isMoveImmediate() &&
          isInlineConstant(Def->getOperand(1)) &&
          MRI->hasOneUse(Src0->getReg())) {
          Src0->ChangeToImmediate(Def->getOperand(1).getImm());
          Src0Inlined = true;
        } else if ((Register::isPhysicalRegister(Src0->getReg()) &&
                    (ST.getConstantBusLimit(Opc) <= 1 &&
                     RI.isSGPRClass(RI.getPhysRegClass(Src0->getReg())))) ||
                   (Register::isVirtualRegister(Src0->getReg()) &&
                    (ST.getConstantBusLimit(Opc) <= 1 &&
                     RI.isSGPRClass(MRI->getRegClass(Src0->getReg())))))
          return false;
          // VGPR is okay as Src0 - fallthrough
      }

      if (Src1->isReg() && !Src0Inlined ) {
        // We have one slot for inlinable constant so far - try to fill it
        MachineInstr *Def = MRI->getUniqueVRegDef(Src1->getReg());
        if (Def && Def->isMoveImmediate() &&
            isInlineConstant(Def->getOperand(1)) &&
            MRI->hasOneUse(Src1->getReg()) &&
            commuteInstruction(UseMI)) {
            Src0->ChangeToImmediate(Def->getOperand(1).getImm());
        } else if ((Register::isPhysicalRegister(Src1->getReg()) &&
                    RI.isSGPRClass(RI.getPhysRegClass(Src1->getReg()))) ||
                   (Register::isVirtualRegister(Src1->getReg()) &&
                    RI.isSGPRClass(MRI->getRegClass(Src1->getReg()))))
          return false;
          // VGPR is okay as Src1 - fallthrough
      }

      unsigned NewOpc =
        IsFMA ? (IsF32 ? PPU::V_FMAAK_F32 : PPU::V_FMAAK_F16)
              : (IsF32 ? PPU::V_MADAK_F32 : PPU::V_MADAK_F16);
      if (pseudoToMCOpcode(NewOpc) == -1)
        return false;

      const int64_t Imm = ImmOp->getImm();

      // FIXME: This would be a lot easier if we could return a new instruction
      // instead of having to modify in place.

      // Remove these first since they are at the end.
      UseMI.RemoveOperand(
          PPU::getNamedOperandIdx(Opc, PPU::OpName::omod));
      UseMI.RemoveOperand(
          PPU::getNamedOperandIdx(Opc, PPU::OpName::clamp));

      if (Opc == PPU::V_MAC_F32_e64 ||
          Opc == PPU::V_MAC_F16_e64 ||
          Opc == PPU::V_FMAC_F32_e64 ||
          Opc == PPU::V_FMAC_F16_e64)
        UseMI.untieRegOperand(
            PPU::getNamedOperandIdx(Opc, PPU::OpName::src2));

      // ChangingToImmediate adds Src2 back to the instruction.
      Src2->ChangeToImmediate(Imm);

      // These come before src2.
      removeModOperands(UseMI);
      UseMI.setDesc(get(NewOpc));
      // It might happen that UseMI was commuted
      // and we now have SGPR as SRC1. If so 2 inlined
      // constant and SGPR are illegal.
      legalizeOperands(UseMI);

      bool DeleteDef = MRI->hasOneNonDBGUse(Reg);
      if (DeleteDef)
        DefMI.eraseFromParent();

      return true;
    }
  }
  */

  return false;
}

static bool offsetsDoNotOverlap(int WidthA, int OffsetA,
                                int WidthB, int OffsetB) {
  int LowOffset = OffsetA < OffsetB ? OffsetA : OffsetB;
  int HighOffset = OffsetA < OffsetB ? OffsetB : OffsetA;
  int LowWidth = (LowOffset == OffsetA) ? WidthA : WidthB;
  return LowOffset + LowWidth <= HighOffset;
}

bool PPUInstrInfo::checkInstOffsetsDoNotOverlap(const MachineInstr &MIa,
                                               const MachineInstr &MIb) const {
  const MachineOperand *BaseOp0, *BaseOp1;
  int64_t Offset0, Offset1;

  if (getMemOperandWithOffset(MIa, BaseOp0, Offset0, &RI) &&
      getMemOperandWithOffset(MIb, BaseOp1, Offset1, &RI)) {
    if (!BaseOp0->isIdenticalTo(*BaseOp1))
      return false;

    if (!MIa.hasOneMemOperand() || !MIb.hasOneMemOperand()) {
      // FIXME: Handle ds_read2 / ds_write2.
      return false;
    }
    unsigned Width0 = (*MIa.memoperands_begin())->getSize();
    unsigned Width1 = (*MIb.memoperands_begin())->getSize();
    if (offsetsDoNotOverlap(Width0, Offset0, Width1, Offset1)) {
      return true;
    }
  }

  return false;
}

bool PPUInstrInfo::areMemAccessesTriviallyDisjoint(const MachineInstr &MIa,
                                                  const MachineInstr &MIb,
                                                  AliasAnalysis *AA) const {
    /*
  assert((MIa.mayLoad() || MIa.mayStore()) &&
         "MIa must load from or modify a memory location");
  assert((MIb.mayLoad() || MIb.mayStore()) &&
         "MIb must load from or modify a memory location");

  if (MIa.hasUnmodeledSideEffects() || MIb.hasUnmodeledSideEffects())
    return false;

  // XXX - Can we relax this between address spaces?
  if (MIa.hasOrderedMemoryRef() || MIb.hasOrderedMemoryRef())
    return false;

  // TODO: Should we check the address space from the MachineMemOperand? That
  // would allow us to distinguish objects we know don't alias based on the
  // underlying address space, even if it was lowered to a different one,
  // e.g. private accesses lowered to use MUBUF instructions on a scratch
  // buffer.
  if (isDS(MIa)) {
    if (isDS(MIb))
      return checkInstOffsetsDoNotOverlap(MIa, MIb);

    return !isFLAT(MIb) || isSegmentSpecificFLAT(MIb);
  }

  if (isMUBUF(MIa) || isMTBUF(MIa)) {
    if (isMUBUF(MIb) || isMTBUF(MIb))
      return checkInstOffsetsDoNotOverlap(MIa, MIb);

    return !isFLAT(MIb) && !isSMRD(MIb);
  }

  if (isSMRD(MIa)) {
    if (isSMRD(MIb))
      return checkInstOffsetsDoNotOverlap(MIa, MIb);

    return !isFLAT(MIb) && !isMUBUF(MIa) && !isMTBUF(MIa);
  }

  if (isFLAT(MIa)) {
    if (isFLAT(MIb))
      return checkInstOffsetsDoNotOverlap(MIa, MIb);

    return false;
  }
  */

  return false;
}

static int64_t getFoldableImm(const MachineOperand* MO) {
  if (!MO->isReg())
    return false;
  const MachineFunction *MF = MO->getParent()->getParent()->getParent();
  const MachineRegisterInfo &MRI = MF->getRegInfo();
  auto Def = MRI.getUniqueVRegDef(MO->getReg());
  if (Def && Def->getOpcode() == PPU::VMOV &&
      Def->getOperand(1).isImm())
    return Def->getOperand(1).getImm();
  return PPU::NoRegister;
}

MachineInstr *PPUInstrInfo::convertToThreeAddress(MachineFunction::iterator &MBB,
                                                 MachineInstr &MI,
                                                 LiveVariables *LV) const {
    /*
  unsigned Opc = MI.getOpcode();
  bool IsF16 = false;
  bool IsFMA = Opc == PPU::V_FMAC_F32_e32 || Opc == PPU::V_FMAC_F32_e64 ||
               Opc == PPU::V_FMAC_F16_e32 || Opc == PPU::V_FMAC_F16_e64;

  switch (Opc) {
  default:
    return nullptr;
  case PPU::V_MAC_F16_e64:
  case PPU::V_FMAC_F16_e64:
    IsF16 = true;
    LLVM_FALLTHROUGH;
  case PPU::V_MAC_F32_e64:
  case PPU::V_FMAC_F32_e64:
    break;
  case PPU::V_MAC_F16_e32:
  case PPU::V_FMAC_F16_e32:
    IsF16 = true;
    LLVM_FALLTHROUGH;
  case PPU::V_MAC_F32_e32:
  case PPU::V_FMAC_F32_e32: {
    int Src0Idx = PPU::getNamedOperandIdx(MI.getOpcode(),
                                             PPU::OpName::src0);
    const MachineOperand *Src0 = &MI.getOperand(Src0Idx);
    if (!Src0->isReg() && !Src0->isImm())
      return nullptr;

    if (Src0->isImm() && !isInlineConstant(MI, Src0Idx, *Src0))
      return nullptr;

    break;
  }
  }

  const MachineOperand *Dst = getNamedOperand(MI, PPU::OpName::vdst);
  const MachineOperand *Src0 = getNamedOperand(MI, PPU::OpName::src0);
  const MachineOperand *Src0Mods =
    getNamedOperand(MI, PPU::OpName::src0_modifiers);
  const MachineOperand *Src1 = getNamedOperand(MI, PPU::OpName::src1);
  const MachineOperand *Src1Mods =
    getNamedOperand(MI, PPU::OpName::src1_modifiers);
  const MachineOperand *Src2 = getNamedOperand(MI, PPU::OpName::src2);
  const MachineOperand *Clamp = getNamedOperand(MI, PPU::OpName::clamp);
  const MachineOperand *Omod = getNamedOperand(MI, PPU::OpName::omod);

  if (!Src0Mods && !Src1Mods && !Clamp && !Omod &&
      // If we have an SGPR input, we will violate the constant bus restriction.
      (ST.getConstantBusLimit(Opc) > 1 ||
       !Src0->isReg() ||
       !RI.isSGPRReg(MBB->getParent()->getRegInfo(), Src0->getReg()))) {
    if (auto Imm = getFoldableImm(Src2)) {
      unsigned NewOpc =
         IsFMA ? (IsF16 ? PPU::V_FMAAK_F16 : PPU::V_FMAAK_F32)
               : (IsF16 ? PPU::V_MADAK_F16 : PPU::V_MADAK_F32);
      if (pseudoToMCOpcode(NewOpc) != -1)
        return BuildMI(*MBB, MI, MI.getDebugLoc(), get(NewOpc))
                 .add(*Dst)
                 .add(*Src0)
                 .add(*Src1)
                 .addImm(Imm);
    }
    unsigned NewOpc =
      IsFMA ? (IsF16 ? PPU::V_FMAMK_F16 : PPU::V_FMAMK_F32)
            : (IsF16 ? PPU::V_MADMK_F16 : PPU::V_MADMK_F32);
    if (auto Imm = getFoldableImm(Src1)) {
      if (pseudoToMCOpcode(NewOpc) != -1)
        return BuildMI(*MBB, MI, MI.getDebugLoc(), get(NewOpc))
                 .add(*Dst)
                 .add(*Src0)
                 .addImm(Imm)
                 .add(*Src2);
    }
    if (auto Imm = getFoldableImm(Src0)) {
      if (pseudoToMCOpcode(NewOpc) != -1 &&
          isOperandLegal(MI, PPU::getNamedOperandIdx(NewOpc,
                           PPU::OpName::src0), Src1))
        return BuildMI(*MBB, MI, MI.getDebugLoc(), get(NewOpc))
                 .add(*Dst)
                 .add(*Src1)
                 .addImm(Imm)
                 .add(*Src2);
    }
  }

  unsigned NewOpc = IsFMA ? (IsF16 ? PPU::V_FMA_F16 : PPU::V_FMA_F32)
                          : (IsF16 ? PPU::V_MAD_F16 : PPU::V_MAD_F32);
  if (pseudoToMCOpcode(NewOpc) == -1)
    return nullptr;

  return BuildMI(*MBB, MI, MI.getDebugLoc(), get(NewOpc))
      .add(*Dst)
      .addImm(Src0Mods ? Src0Mods->getImm() : 0)
      .add(*Src0)
      .addImm(Src1Mods ? Src1Mods->getImm() : 0)
      .add(*Src1)
      .addImm(0) // Src mods
      .add(*Src2)
      .addImm(Clamp ? Clamp->getImm() : 0)
      .addImm(Omod ? Omod->getImm() : 0);
  */
}

// It's not generally safe to move VALU instructions across these since it will
// start using the register as a base index rather than directly.
// XXX - Why isn't hasSideEffects sufficient for these?
static bool changesVGPRIndexingMode(const MachineInstr &MI) {
  switch (MI.getOpcode()) {
    /*
  case PPU::S_SET_GPR_IDX_ON:
  case PPU::S_SET_GPR_IDX_MODE:
  case PPU::S_SET_GPR_IDX_OFF:
    return true;
    */
  default:
    return false;
  }
}

bool PPUInstrInfo::isSchedulingBoundary(const MachineInstr &MI,
                                       const MachineBasicBlock *MBB,
                                       const MachineFunction &MF) const {
    /*
  // XXX - Do we want the SP check in the base implementation?

  // Target-independent instructions do not have an implicit-use of EXEC, even
  // when they operate on VGPRs. Treating EXEC modifications as scheduling
  // boundaries prevents incorrect movements of such instructions.
  return TargetInstrInfo::isSchedulingBoundary(MI, MBB, MF) ||
         MI.modifiesRegister(PPU::EXEC, &RI) ||
         MI.getOpcode() == PPU::S_SETREG_IMM32_B32 ||
         MI.getOpcode() == PPU::S_SETREG_B32 ||
         MI.getOpcode() == PPU::S_DENORM_MODE ||
         changesVGPRIndexingMode(MI);
         */
    return false;
}

bool PPUInstrInfo::isAlwaysGDS(uint16_t Opcode) const {
    /*
  return Opcode == PPU::DS_ORDERED_COUNT ||
         Opcode == PPU::DS_GWS_INIT ||
         Opcode == PPU::DS_GWS_SEMA_V ||
         Opcode == PPU::DS_GWS_SEMA_BR ||
         Opcode == PPU::DS_GWS_SEMA_P ||
         Opcode == PPU::DS_GWS_SEMA_RELEASE_ALL ||
         Opcode == PPU::DS_GWS_BARRIER;
         */
}

bool PPUInstrInfo::hasUnwantedEffectsWhenEXECEmpty(const MachineInstr &MI) const {
    /*
  unsigned Opcode = MI.getOpcode();

  if (MI.mayStore() && isSMRD(MI))
    return true; // scalar store or atomic

  // This will terminate the function when other lanes may need to continue.
  if (MI.isReturn())
    return true;

  // These instructions cause shader I/O that may cause hardware lockups
  // when executed with an empty EXEC mask.
  //
  // Note: exp with VM = DONE = 0 is automatically skipped by hardware when
  //       EXEC = 0, but checking for that case here seems not worth it
  //       given the typical code patterns.
  if (Opcode == PPU::S_SENDMSG || Opcode == PPU::S_SENDMSGHALT ||
      Opcode == PPU::EXP || Opcode == PPU::EXP_DONE ||
      Opcode == PPU::DS_ORDERED_COUNT || Opcode == PPU::S_TRAP ||
      Opcode == PPU::DS_GWS_INIT || Opcode == PPU::DS_GWS_BARRIER)
    return true;

  if (MI.isCall() || MI.isInlineAsm())
    return true; // conservative assumption

  // These are like SALU instructions in terms of effects, so it's questionable
  // whether we should return true for those.
  //
  // However, executing them with EXEC = 0 causes them to operate on undefined
  // data, which we avoid by returning true here.
  if (Opcode == PPU::V_READFIRSTLANE_B32 || Opcode == PPU::V_READLANE_B32)
    return true;
    */

  return false;
}

bool PPUInstrInfo::mayReadEXEC(const MachineRegisterInfo &MRI,
                              const MachineInstr &MI) const {
  if (MI.isMetaInstruction())
    return false;

  // This won't read exec if this is an SGPR->SGPR copy.
  if (MI.isCopyLike()) {
    if (!RI.isSGPRReg(MRI, MI.getOperand(0).getReg()))
      return true;

    // Make sure this isn't copying exec as a normal operand
    return MI.readsRegister(PPU::TMSK, &RI);
  }

  // Make a conservative assumption about the callee.
  if (MI.isCall())
    return true;

  // Be conservative with any unhandled generic opcodes.
  if (!isTargetSpecificOpcode(MI.getOpcode()))
    return true;

  return !isSALU(MI) || MI.readsRegister(PPU::TMSK, &RI);
}

bool PPUInstrInfo::isInlineConstant(const APInt &Imm) const {
  switch (Imm.getBitWidth()) {
      /*
  case 1: // This likely will be a condition code mask.
    return true;

  case 32:
    return PPU::isInlinableLiteral32(Imm.getSExtValue(),
                                        ST.hasInv2PiInlineImm());
  case 64:
    return PPU::isInlinableLiteral64(Imm.getSExtValue(),
                                        ST.hasInv2PiInlineImm());
  case 16:
    return ST.has16BitInsts() &&
           PPU::isInlinableLiteral16(Imm.getSExtValue(),
                                        ST.hasInv2PiInlineImm());
                                        */
  default:
    llvm_unreachable("invalid bitwidth");
  }
}

bool PPUInstrInfo::isInlineConstant(const MachineOperand &MO,
                                   uint8_t OperandType) const {
  if (!MO.isImm() ||
      OperandType < PPU::OPERAND_SRC_FIRST ||
      OperandType > PPU::OPERAND_SRC_LAST)
    return false;

  // MachineOperand provides no way to tell the true operand size, since it only
  // records a 64-bit value. We need to know the size to determine if a 32-bit
  // floating point immediate bit pattern is legal for an integer immediate. It
  // would be for any 32-bit integer operand, but would not be for a 64-bit one.

  int64_t Imm = MO.getImm();
  switch (OperandType) {
      /*
  case PPU::OPERAND_REG_IMM_INT32:
  case PPU::OPERAND_REG_IMM_FP32:
  case PPU::OPERAND_REG_INLINE_C_INT32:
  case PPU::OPERAND_REG_INLINE_C_FP32:
  case PPU::OPERAND_REG_INLINE_AC_INT32:
  case PPU::OPERAND_REG_INLINE_AC_FP32: {
    int32_t Trunc = static_cast<int32_t>(Imm);
    return PPU::isInlinableLiteral32(Trunc, ST.hasInv2PiInlineImm());
  }
  case PPU::OPERAND_REG_IMM_INT64:
  case PPU::OPERAND_REG_IMM_FP64:
  case PPU::OPERAND_REG_INLINE_C_INT64:
  case PPU::OPERAND_REG_INLINE_C_FP64:
    return PPU::isInlinableLiteral64(MO.getImm(),
                                        ST.hasInv2PiInlineImm());
  case PPU::OPERAND_REG_IMM_INT16:
  case PPU::OPERAND_REG_IMM_FP16:
  case PPU::OPERAND_REG_INLINE_C_INT16:
  case PPU::OPERAND_REG_INLINE_C_FP16:
  case PPU::OPERAND_REG_INLINE_AC_INT16:
  case PPU::OPERAND_REG_INLINE_AC_FP16: {
    if (isInt<16>(Imm) || isUInt<16>(Imm)) {
      // A few special case instructions have 16-bit operands on subtargets
      // where 16-bit instructions are not legal.
      // TODO: Do the 32-bit immediates work? We shouldn't really need to handle
      // constants in these cases
      int16_t Trunc = static_cast<int16_t>(Imm);
      return ST.has16BitInsts() &&
             PPU::isInlinableLiteral16(Trunc, ST.hasInv2PiInlineImm());
    }

    return false;
  }
  case PPU::OPERAND_REG_IMM_V2INT16:
  case PPU::OPERAND_REG_IMM_V2FP16:
  case PPU::OPERAND_REG_INLINE_C_V2INT16:
  case PPU::OPERAND_REG_INLINE_C_V2FP16:
  case PPU::OPERAND_REG_INLINE_AC_V2INT16:
  case PPU::OPERAND_REG_INLINE_AC_V2FP16: {
    uint32_t Trunc = static_cast<uint32_t>(Imm);
    return PPU::isInlinableLiteralV216(Trunc, ST.hasInv2PiInlineImm());
  }
  */
  default:
    llvm_unreachable("invalid bitwidth");
  }
}

bool PPUInstrInfo::isLiteralConstantLike(const MachineOperand &MO,
                                        const MCOperandInfo &OpInfo) const {
  switch (MO.getType()) {
      /*
  case MachineOperand::MO_Register:
    return false;
  case MachineOperand::MO_Immediate:
    return !isInlineConstant(MO, OpInfo);
  case MachineOperand::MO_FrameIndex:
  case MachineOperand::MO_MachineBasicBlock:
  case MachineOperand::MO_ExternalSymbol:
  case MachineOperand::MO_GlobalAddress:
  case MachineOperand::MO_MCSymbol:
    return true;
    */
  default:
    llvm_unreachable("unexpected operand type");
  }
}

static bool compareMachineOp(const MachineOperand &Op0,
                             const MachineOperand &Op1) {
  if (Op0.getType() != Op1.getType())
    return false;

  switch (Op0.getType()) {
  case MachineOperand::MO_Register:
    return Op0.getReg() == Op1.getReg();
  case MachineOperand::MO_Immediate:
    return Op0.getImm() == Op1.getImm();
  default:
    llvm_unreachable("Didn't expect to be comparing these operand types");
  }
}

bool PPUInstrInfo::isImmOperandLegal(const MachineInstr &MI, unsigned OpNo,
                                    const MachineOperand &MO) const {
    /*
  const MCInstrDesc &InstDesc = MI.getDesc();
  const MCOperandInfo &OpInfo = InstDesc.OpInfo[OpNo];

  assert(MO.isImm() || MO.isTargetIndex() || MO.isFI() || MO.isGlobal());

  if (OpInfo.OperandType == MCOI::OPERAND_IMMEDIATE)
    return true;

  if (OpInfo.RegClass < 0)
    return false;

  const MachineFunction *MF = MI.getParent()->getParent();
  const PPUSubtarget &ST = MF->getSubtarget<PPUSubtarget>();

  if (MO.isImm() && isInlineConstant(MO, OpInfo)) {
    if (isMAI(MI) && ST.hasMFMAInlineLiteralBug() &&
        OpNo ==(unsigned)PPU::getNamedOperandIdx(MI.getOpcode(),
                                                    PPU::OpName::src2))
      return false;
    return RI.opCanUseInlineConstant(OpInfo.OperandType);
  }

  if (!RI.opCanUseLiteralConstant(OpInfo.OperandType))
    return false;

  if (!isVOP3(MI) || !PPU::isSISrcOperand(InstDesc, OpNo))
    return true;

  return ST.hasVOP3Literal();
  */
    return false;
}

/*
bool PPUInstrInfo::hasVALU32BitEncoding(unsigned Opcode) const {
  int Op32 = PPU::getVOPe32(Opcode);
  if (Op32 == -1)
    return false;

  return pseudoToMCOpcode(Op32) != -1;
}
*/

bool PPUInstrInfo::hasModifiers(unsigned Opcode) const {
  // The src0_modifier operand is present on all instructions
  // that have modifiers.
  // return PPU::getNamedOperandIdx(Opcode, PPU::OpName::src0_modifiers) != -1;
  return false;
}

bool PPUInstrInfo::hasModifiersSet(const MachineInstr &MI,
                                  unsigned OpName) const {
  const MachineOperand *Mods = getNamedOperand(MI, OpName);
  return Mods && Mods->getImm();
}

/*
bool PPUInstrInfo::hasAnyModifiersSet(const MachineInstr &MI) const {
  return hasModifiersSet(MI, PPU::OpName::src0_modifiers) ||
         hasModifiersSet(MI, PPU::OpName::src1_modifiers) ||
         hasModifiersSet(MI, PPU::OpName::src2_modifiers) ||
         hasModifiersSet(MI, PPU::OpName::clamp) ||
         hasModifiersSet(MI, PPU::OpName::omod);
}
*/

bool PPUInstrInfo::canShrink(const MachineInstr &MI,
                            const MachineRegisterInfo &MRI) const {
    /*
  const MachineOperand *Src2 = getNamedOperand(MI, PPU::OpName::src2);
  // Can't shrink instruction with three operands.
  // FIXME: v_cndmask_b32 has 3 operands and is shrinkable, but we need to add
  // a special case for it.  It can only be shrunk if the third operand
  // is vcc, and src0_modifiers and src1_modifiers are not set.
  // We should handle this the same way we handle vopc, by addding
  // a register allocation hint pre-regalloc and then do the shrinking
  // post-regalloc.
  if (Src2) {
    switch (MI.getOpcode()) {
      default: return false;

      case PPU::V_ADDC_U32_e64:
      case PPU::V_SUBB_U32_e64:
      case PPU::V_SUBBREV_U32_e64: {
        const MachineOperand *Src1
          = getNamedOperand(MI, PPU::OpName::src1);
        if (!Src1->isReg() || !RI.isVGPR(MRI, Src1->getReg()))
          return false;
        // Additional verification is needed for sdst/src2.
        return true;
      }
      case PPU::V_MAC_F32_e64:
      case PPU::V_MAC_F16_e64:
      case PPU::V_FMAC_F32_e64:
      case PPU::V_FMAC_F16_e64:
        if (!Src2->isReg() || !RI.isVGPR(MRI, Src2->getReg()) ||
            hasModifiersSet(MI, PPU::OpName::src2_modifiers))
          return false;
        break;

      case PPU::V_CNDMASK_B32_e64:
        break;
    }
  }

  const MachineOperand *Src1 = getNamedOperand(MI, PPU::OpName::src1);
  if (Src1 && (!Src1->isReg() || !RI.isVGPR(MRI, Src1->getReg()) ||
               hasModifiersSet(MI, PPU::OpName::src1_modifiers)))
    return false;

  // We don't need to check src0, all input types are legal, so just make sure
  // src0 isn't using any modifiers.
  if (hasModifiersSet(MI, PPU::OpName::src0_modifiers))
    return false;

  // Can it be shrunk to a valid 32 bit opcode?
  if (!hasVALU32BitEncoding(MI.getOpcode()))
    return false;

  // Check output modifiers
  return !hasModifiersSet(MI, PPU::OpName::omod) &&
         !hasModifiersSet(MI, PPU::OpName::clamp);
         */
    return false;
}

// Set VCC operand with all flags from \p Orig, except for setting it as
// implicit.
static void copyFlagsToImplicitVCC(MachineInstr &MI,
                                   const MachineOperand &Orig) {

  for (MachineOperand &Use : MI.implicit_operands()) {
    if (Use.isUse() && Use.getReg() == PPU::VCC) {
      Use.setIsUndef(Orig.isUndef());
      Use.setIsKill(Orig.isKill());
      return;
    }
  }
}
/*
MachineInstr *PPUInstrInfo::buildShrunkInst(MachineInstr &MI,
                                           unsigned Op32) const {
  MachineBasicBlock *MBB = MI.getParent();;
  MachineInstrBuilder Inst32 =
    BuildMI(*MBB, MI, MI.getDebugLoc(), get(Op32));

  // Add the dst operand if the 32-bit encoding also has an explicit $vdst.
  // For VOPC instructions, this is replaced by an implicit def of vcc.
  int Op32DstIdx = PPU::getNamedOperandIdx(Op32, PPU::OpName::vdst);
  if (Op32DstIdx != -1) {
    // dst
    Inst32.add(MI.getOperand(0));
  } else {
    assert(((MI.getOperand(0).getReg() == PPU::VCC) ||
            (MI.getOperand(0).getReg() == PPU::VCC_LO)) &&
           "Unexpected case");
  }

  Inst32.add(*getNamedOperand(MI, PPU::OpName::src0));

  const MachineOperand *Src1 = getNamedOperand(MI, PPU::OpName::src1);
  if (Src1)
    Inst32.add(*Src1);

  const MachineOperand *Src2 = getNamedOperand(MI, PPU::OpName::src2);

  if (Src2) {
    int Op32Src2Idx = PPU::getNamedOperandIdx(Op32, PPU::OpName::src2);
    if (Op32Src2Idx != -1) {
      Inst32.add(*Src2);
    } else {
      // In the case of V_CNDMASK_B32_e32, the explicit operand src2 is
      // replaced with an implicit read of vcc. This was already added
      // during the initial BuildMI, so find it to preserve the flags.
      copyFlagsToImplicitVCC(*Inst32, *Src2);
    }
  }

  return Inst32;
}
*/

bool PPUInstrInfo::usesConstantBus(const MachineRegisterInfo &MRI,
                                  const MachineOperand &MO,
                                  const MCOperandInfo &OpInfo) const {
  // Literal constants use the constant bus.
  //if (isLiteralConstantLike(MO, OpInfo))
  // return true;
  if (MO.isImm())
    return !isInlineConstant(MO, OpInfo);

  if (!MO.isReg())
    return true; // Misc other operands like FrameIndex

  if (!MO.isUse())
    return false;

  if (Register::isVirtualRegister(MO.getReg()))
    return RI.isSGPRClass(MRI.getRegClass(MO.getReg()));

  // Null is free
  /*
  if (MO.getReg() == PPU::SGPR_NULL)
    return false;
    */

  // SGPRs use the constant bus
  if (MO.isImplicit()) {
    // return MO.getReg() == PPU::VCC;
    return false; // FIXME
  } else {
    return PPU::GPRRegClass.contains(MO.getReg());
  }
}

static unsigned findImplicitSGPRRead(const MachineInstr &MI) {
  for (const MachineOperand &MO : MI.implicit_operands()) {
    // We only care about reads.
    if (MO.isDef())
      continue;

    switch (MO.getReg()) {
        /* FIXME
    case PPU::VCC:
    case PPU::FLAT_SCR:
      return MO.getReg();
      */


    default:
      break;
    }
  }

  return PPU::NoRegister;
}

static bool shouldReadExec(const MachineInstr &MI) {
  if (PPUInstrInfo::isVALU(MI)) {
    switch (MI.getOpcode()) {
        /* FIXME
    case PPU::V_READLANE_B32:
    case PPU::V_WRITELANE_B32:
    */
      return false;
    }

    return true;
  }

  if (PPUInstrInfo::isGenericOpcode(MI.getOpcode()) ||
      PPUInstrInfo::isSALU(MI) ||
      PPUInstrInfo::isSMRD(MI))
    return false;

  return true;
}

/*
static bool isSubRegOf(const SIRegisterInfo &TRI,
                       const MachineOperand &SuperVec,
                       const MachineOperand &SubReg) {
  if (Register::isPhysicalRegister(SubReg.getReg()))
    return TRI.isSubRegister(SuperVec.getReg(), SubReg.getReg());

  return SubReg.getSubReg() != PPU::NoSubRegister &&
         SubReg.getReg() == SuperVec.getReg();
}
*/

bool PPUInstrInfo::verifyInstruction(const MachineInstr &MI,
                                    StringRef &ErrInfo) const {
    /*
  uint16_t Opcode = MI.getOpcode();
  if (PPUInstrInfo::isGenericOpcode(MI.getOpcode()))
    return true;

  const MachineFunction *MF = MI.getParent()->getParent();
  const MachineRegisterInfo &MRI = MF->getRegInfo();

  int Src0Idx = PPU::getNamedOperandIdx(Opcode, PPU::OpName::src0);
  int Src1Idx = PPU::getNamedOperandIdx(Opcode, PPU::OpName::src1);
  int Src2Idx = PPU::getNamedOperandIdx(Opcode, PPU::OpName::src2);

  // Make sure the number of operands is correct.
  const MCInstrDesc &Desc = get(Opcode);
  if (!Desc.isVariadic() &&
      Desc.getNumOperands() != MI.getNumExplicitOperands()) {
    ErrInfo = "Instruction has wrong number of operands.";
    return false;
  }

  if (MI.isInlineAsm()) {
    // Verify register classes for inlineasm constraints.
    for (unsigned I = InlineAsm::MIOp_FirstOperand, E = MI.getNumOperands();
         I != E; ++I) {
      const TargetRegisterClass *RC = MI.getRegClassConstraint(I, this, &RI);
      if (!RC)
        continue;

      const MachineOperand &Op = MI.getOperand(I);
      if (!Op.isReg())
        continue;

      Register Reg = Op.getReg();
      if (!Register::isVirtualRegister(Reg) && !RC->contains(Reg)) {
        ErrInfo = "inlineasm operand has incorrect register class.";
        return false;
      }
    }

    return true;
  }

  // Make sure the register classes are correct.
  for (int i = 0, e = Desc.getNumOperands(); i != e; ++i) {
    if (MI.getOperand(i).isFPImm()) {
      ErrInfo = "FPImm Machine Operands are not supported. ISel should bitcast "
                "all fp values to integers.";
      return false;
    }

    int RegClass = Desc.OpInfo[i].RegClass;

    switch (Desc.OpInfo[i].OperandType) {
    case MCOI::OPERAND_REGISTER:
      if (MI.getOperand(i).isImm() || MI.getOperand(i).isGlobal()) {
        ErrInfo = "Illegal immediate value for operand.";
        return false;
      }
      break;
    case PPU::OPERAND_REG_IMM_INT32:
    case PPU::OPERAND_REG_IMM_FP32:
      break;
    case PPU::OPERAND_REG_INLINE_C_INT32:
    case PPU::OPERAND_REG_INLINE_C_FP32:
    case PPU::OPERAND_REG_INLINE_C_INT64:
    case PPU::OPERAND_REG_INLINE_C_FP64:
    case PPU::OPERAND_REG_INLINE_C_INT16:
    case PPU::OPERAND_REG_INLINE_C_FP16:
    case PPU::OPERAND_REG_INLINE_AC_INT32:
    case PPU::OPERAND_REG_INLINE_AC_FP32:
    case PPU::OPERAND_REG_INLINE_AC_INT16:
    case PPU::OPERAND_REG_INLINE_AC_FP16: {
      const MachineOperand &MO = MI.getOperand(i);
      if (!MO.isReg() && (!MO.isImm() || !isInlineConstant(MI, i))) {
        ErrInfo = "Illegal immediate value for operand.";
        return false;
      }
      break;
    }
    case MCOI::OPERAND_IMMEDIATE:
    case PPU::OPERAND_KIMM32:
      // Check if this operand is an immediate.
      // FrameIndex operands will be replaced by immediates, so they are
      // allowed.
      if (!MI.getOperand(i).isImm() && !MI.getOperand(i).isFI()) {
        ErrInfo = "Expected immediate, but got non-immediate";
        return false;
      }
      LLVM_FALLTHROUGH;
    default:
      continue;
    }

    if (!MI.getOperand(i).isReg())
      continue;

    if (RegClass != -1) {
      Register Reg = MI.getOperand(i).getReg();
      if (Reg == PPU::NoRegister || Register::isVirtualRegister(Reg))
        continue;

      const TargetRegisterClass *RC = RI.getRegClass(RegClass);
      if (!RC->contains(Reg)) {
        ErrInfo = "Operand has incorrect register class.";
        return false;
      }
    }
  }

  // Verify SDWA
  if (isSDWA(MI)) {
    if (!ST.hasSDWA()) {
      ErrInfo = "SDWA is not supported on this target";
      return false;
    }

    int DstIdx = PPU::getNamedOperandIdx(Opcode, PPU::OpName::vdst);

    const int OpIndicies[] = { DstIdx, Src0Idx, Src1Idx, Src2Idx };

    for (int OpIdx: OpIndicies) {
      if (OpIdx == -1)
        continue;
      const MachineOperand &MO = MI.getOperand(OpIdx);

      if (!ST.hasSDWAScalar()) {
        // Only VGPRS on VI
        if (!MO.isReg() || !RI.hasVGPRs(RI.getRegClassForReg(MRI, MO.getReg()))) {
          ErrInfo = "Only VGPRs allowed as operands in SDWA instructions on VI";
          return false;
        }
      } else {
        // No immediates on GFX9
        if (!MO.isReg()) {
          ErrInfo = "Only reg allowed as operands in SDWA instructions on GFX9";
          return false;
        }
      }
    }

    if (!ST.hasSDWAOmod()) {
      // No omod allowed on VI
      const MachineOperand *OMod = getNamedOperand(MI, PPU::OpName::omod);
      if (OMod != nullptr &&
        (!OMod->isImm() || OMod->getImm() != 0)) {
        ErrInfo = "OMod not allowed in SDWA instructions on VI";
        return false;
      }
    }

    uint16_t BasicOpcode = PPU::getBasicFromSDWAOp(Opcode);
    if (isVOPC(BasicOpcode)) {
      if (!ST.hasSDWASdst() && DstIdx != -1) {
        // Only vcc allowed as dst on VI for VOPC
        const MachineOperand &Dst = MI.getOperand(DstIdx);
        if (!Dst.isReg() || Dst.getReg() != PPU::VCC) {
          ErrInfo = "Only VCC allowed as dst in SDWA instructions on VI";
          return false;
        }
      } else if (!ST.hasSDWAOutModsVOPC()) {
        // No clamp allowed on GFX9 for VOPC
        const MachineOperand *Clamp = getNamedOperand(MI, PPU::OpName::clamp);
        if (Clamp && (!Clamp->isImm() || Clamp->getImm() != 0)) {
          ErrInfo = "Clamp not allowed in VOPC SDWA instructions on VI";
          return false;
        }

        // No omod allowed on GFX9 for VOPC
        const MachineOperand *OMod = getNamedOperand(MI, PPU::OpName::omod);
        if (OMod && (!OMod->isImm() || OMod->getImm() != 0)) {
          ErrInfo = "OMod not allowed in VOPC SDWA instructions on VI";
          return false;
        }
      }
    }

    const MachineOperand *DstUnused = getNamedOperand(MI, PPU::OpName::dst_unused);
    if (DstUnused && DstUnused->isImm() &&
        DstUnused->getImm() == PPU::SDWA::UNUSED_PRESERVE) {
      const MachineOperand &Dst = MI.getOperand(DstIdx);
      if (!Dst.isReg() || !Dst.isTied()) {
        ErrInfo = "Dst register should have tied register";
        return false;
      }

      const MachineOperand &TiedMO =
          MI.getOperand(MI.findTiedOperandIdx(DstIdx));
      if (!TiedMO.isReg() || !TiedMO.isImplicit() || !TiedMO.isUse()) {
        ErrInfo =
            "Dst register should be tied to implicit use of preserved register";
        return false;
      } else if (Register::isPhysicalRegister(TiedMO.getReg()) &&
                 Dst.getReg() != TiedMO.getReg()) {
        ErrInfo = "Dst register should use same physical register as preserved";
        return false;
      }
    }
  }

  // Verify MIMG
  if (isMIMG(MI.getOpcode()) && !MI.mayStore()) {
    // Ensure that the return type used is large enough for all the options
    // being used TFE/LWE require an extra result register.
    const MachineOperand *DMask = getNamedOperand(MI, PPU::OpName::dmask);
    if (DMask) {
      uint64_t DMaskImm = DMask->getImm();
      uint32_t RegCount =
          isGather4(MI.getOpcode()) ? 4 : countPopulation(DMaskImm);
      const MachineOperand *TFE = getNamedOperand(MI, PPU::OpName::tfe);
      const MachineOperand *LWE = getNamedOperand(MI, PPU::OpName::lwe);
      const MachineOperand *D16 = getNamedOperand(MI, PPU::OpName::d16);

      // Adjust for packed 16 bit values
      if (D16 && D16->getImm() && !ST.hasUnpackedD16VMem())
        RegCount >>= 1;

      // Adjust if using LWE or TFE
      if ((LWE && LWE->getImm()) || (TFE && TFE->getImm()))
        RegCount += 1;

      const uint32_t DstIdx =
          PPU::getNamedOperandIdx(MI.getOpcode(), PPU::OpName::vdata);
      const MachineOperand &Dst = MI.getOperand(DstIdx);
      if (Dst.isReg()) {
        const TargetRegisterClass *DstRC = getOpRegClass(MI, DstIdx);
        uint32_t DstSize = RI.getRegSizeInBits(*DstRC) / 32;
        if (RegCount > DstSize) {
          ErrInfo = "MIMG instruction returns too many registers for dst "
                    "register class";
          return false;
        }
      }
    }
  }

  // Verify VOP*. Ignore multiple sgpr operands on writelane.
  if (Desc.getOpcode() != PPU::V_WRITELANE_B32
      && (isVOP1(MI) || isVOP2(MI) || isVOP3(MI) || isVOPC(MI) || isSDWA(MI))) {
    // Only look at the true operands. Only a real operand can use the constant
    // bus, and we don't want to check pseudo-operands like the source modifier
    // flags.
    const int OpIndices[] = { Src0Idx, Src1Idx, Src2Idx };

    unsigned ConstantBusCount = 0;
    unsigned LiteralCount = 0;

    if (PPU::getNamedOperandIdx(Opcode, PPU::OpName::imm) != -1)
      ++ConstantBusCount;

    SmallVector<unsigned, 2> SGPRsUsed;
    unsigned SGPRUsed = findImplicitSGPRRead(MI);
    if (SGPRUsed != PPU::NoRegister) {
      ++ConstantBusCount;
      SGPRsUsed.push_back(SGPRUsed);
    }

    for (int OpIdx : OpIndices) {
      if (OpIdx == -1)
        break;
      const MachineOperand &MO = MI.getOperand(OpIdx);
      if (usesConstantBus(MRI, MO, MI.getDesc().OpInfo[OpIdx])) {
        if (MO.isReg()) {
          SGPRUsed = MO.getReg();
          if (llvm::all_of(SGPRsUsed, [this, SGPRUsed](unsigned SGPR) {
                return !RI.regsOverlap(SGPRUsed, SGPR);
              })) {
            ++ConstantBusCount;
            SGPRsUsed.push_back(SGPRUsed);
          }
        } else {
          ++ConstantBusCount;
          ++LiteralCount;
        }
      }
    }
    const PPUSubtarget &ST = MF->getSubtarget<PPUSubtarget>();
    // v_writelane_b32 is an exception from constant bus restriction:
    // vsrc0 can be sgpr, const or m0 and lane select sgpr, m0 or inline-const
    if (ConstantBusCount > ST.getConstantBusLimit(Opcode) &&
        Opcode != PPU::V_WRITELANE_B32) {
      ErrInfo = "VOP* instruction violates constant bus restriction";
      return false;
    }

    if (isVOP3(MI) && LiteralCount) {
      if (LiteralCount && !ST.hasVOP3Literal()) {
        ErrInfo = "VOP3 instruction uses literal";
        return false;
      }
      if (LiteralCount > 1) {
        ErrInfo = "VOP3 instruction uses more than one literal";
        return false;
      }
    }
  }

  // Verify misc. restrictions on specific instructions.
  if (Desc.getOpcode() == PPU::V_DIV_SCALE_F32 ||
      Desc.getOpcode() == PPU::V_DIV_SCALE_F64) {
    const MachineOperand &Src0 = MI.getOperand(Src0Idx);
    const MachineOperand &Src1 = MI.getOperand(Src1Idx);
    const MachineOperand &Src2 = MI.getOperand(Src2Idx);
    if (Src0.isReg() && Src1.isReg() && Src2.isReg()) {
      if (!compareMachineOp(Src0, Src1) &&
          !compareMachineOp(Src0, Src2)) {
        ErrInfo = "v_div_scale_{f32|f64} require src0 = src1 or src2";
        return false;
      }
    }
  }

  if (isSOP2(MI) || isSOPC(MI)) {
    const MachineOperand &Src0 = MI.getOperand(Src0Idx);
    const MachineOperand &Src1 = MI.getOperand(Src1Idx);
    unsigned Immediates = 0;

    if (!Src0.isReg() &&
        !isInlineConstant(Src0, Desc.OpInfo[Src0Idx].OperandType))
      Immediates++;
    if (!Src1.isReg() &&
        !isInlineConstant(Src1, Desc.OpInfo[Src1Idx].OperandType))
      Immediates++;

    if (Immediates > 1) {
      ErrInfo = "SOP2/SOPC instruction requires too many immediate constants";
      return false;
    }
  }

  if (isSOPK(MI)) {
    auto Op = getNamedOperand(MI, PPU::OpName::simm16);
    if (Desc.isBranch()) {
      if (!Op->isMBB()) {
        ErrInfo = "invalid branch target for SOPK instruction";
        return false;
      }
    } else {
      uint64_t Imm = Op->getImm();
      if (sopkIsZext(MI)) {
        if (!isUInt<16>(Imm)) {
          ErrInfo = "invalid immediate for SOPK instruction";
          return false;
        }
      } else {
        if (!isInt<16>(Imm)) {
          ErrInfo = "invalid immediate for SOPK instruction";
          return false;
        }
      }
    }
  }

  if (Desc.getOpcode() == PPU::V_MOVRELS_B32_e32 ||
      Desc.getOpcode() == PPU::V_MOVRELS_B32_e64 ||
      Desc.getOpcode() == PPU::V_MOVRELD_B32_e32 ||
      Desc.getOpcode() == PPU::V_MOVRELD_B32_e64) {
    const bool IsDst = Desc.getOpcode() == PPU::V_MOVRELD_B32_e32 ||
                       Desc.getOpcode() == PPU::V_MOVRELD_B32_e64;

    const unsigned StaticNumOps = Desc.getNumOperands() +
      Desc.getNumImplicitUses();
    const unsigned NumImplicitOps = IsDst ? 2 : 1;

    // Allow additional implicit operands. This allows a fixup done by the post
    // RA scheduler where the main implicit operand is killed and implicit-defs
    // are added for sub-registers that remain live after this instruction.
    if (MI.getNumOperands() < StaticNumOps + NumImplicitOps) {
      ErrInfo = "missing implicit register operands";
      return false;
    }

    const MachineOperand *Dst = getNamedOperand(MI, PPU::OpName::vdst);
    if (IsDst) {
      if (!Dst->isUse()) {
        ErrInfo = "v_movreld_b32 vdst should be a use operand";
        return false;
      }

      unsigned UseOpIdx;
      if (!MI.isRegTiedToUseOperand(StaticNumOps, &UseOpIdx) ||
          UseOpIdx != StaticNumOps + 1) {
        ErrInfo = "movrel implicit operands should be tied";
        return false;
      }
    }

    const MachineOperand &Src0 = MI.getOperand(Src0Idx);
    const MachineOperand &ImpUse
      = MI.getOperand(StaticNumOps + NumImplicitOps - 1);
    if (!ImpUse.isReg() || !ImpUse.isUse() ||
        !isSubRegOf(RI, ImpUse, IsDst ? *Dst : Src0)) {
      ErrInfo = "src0 should be subreg of implicit vector use";
      return false;
    }
  }

  // Make sure we aren't losing exec uses in the td files. This mostly requires
  // being careful when using let Uses to try to add other use registers.
  if (shouldReadExec(MI)) {
    if (!MI.hasRegisterImplicitUseOperand(PPU::EXEC)) {
      ErrInfo = "VALU instruction does not implicitly read exec mask";
      return false;
    }
  }

  if (isSMRD(MI)) {
    if (MI.mayStore()) {
      // The register offset form of scalar stores may only use m0 as the
      // soffset register.
      const MachineOperand *Soff = getNamedOperand(MI, PPU::OpName::soff);
      if (Soff && Soff->getReg() != PPU::M0) {
        ErrInfo = "scalar stores must use m0 as offset register";
        return false;
      }
    }
  }

  if (isFLAT(MI) && !MF->getSubtarget<PPUSubtarget>().hasFlatInstOffsets()) {
    const MachineOperand *Offset = getNamedOperand(MI, PPU::OpName::offset);
    if (Offset->getImm() != 0) {
      ErrInfo = "subtarget does not support offsets in flat instructions";
      return false;
    }
  }

  if (isMIMG(MI)) {
    const MachineOperand *DimOp = getNamedOperand(MI, PPU::OpName::dim);
    if (DimOp) {
      int VAddr0Idx = PPU::getNamedOperandIdx(Opcode,
                                                 PPU::OpName::vaddr0);
      int SRsrcIdx = PPU::getNamedOperandIdx(Opcode, PPU::OpName::srsrc);
      const PPU::MIMGInfo *Info = PPU::getMIMGInfo(Opcode);
      const PPU::MIMGBaseOpcodeInfo *BaseOpcode =
          PPU::getMIMGBaseOpcodeInfo(Info->BaseOpcode);
      const PPU::MIMGDimInfo *Dim =
          PPU::getMIMGDimInfoByEncoding(DimOp->getImm());

      if (!Dim) {
        ErrInfo = "dim is out of range";
        return false;
      }

      bool IsNSA = SRsrcIdx - VAddr0Idx > 1;
      unsigned AddrWords = BaseOpcode->NumExtraArgs +
                           (BaseOpcode->Gradients ? Dim->NumGradients : 0) +
                           (BaseOpcode->Coordinates ? Dim->NumCoords : 0) +
                           (BaseOpcode->LodOrClampOrMip ? 1 : 0);

      unsigned VAddrWords;
      if (IsNSA) {
        VAddrWords = SRsrcIdx - VAddr0Idx;
      } else {
        const TargetRegisterClass *RC = getOpRegClass(MI, VAddr0Idx);
        VAddrWords = MRI.getTargetRegisterInfo()->getRegSizeInBits(*RC) / 32;
        if (AddrWords > 8)
          AddrWords = 16;
        else if (AddrWords > 4)
          AddrWords = 8;
        else if (AddrWords == 3 && VAddrWords == 4) {
          // CodeGen uses the V4 variant of instructions for three addresses,
          // because the selection DAG does not support non-power-of-two types.
          AddrWords = 4;
        }
      }

      if (VAddrWords != AddrWords) {
        ErrInfo = "bad vaddr size";
        return false;
      }
    }
  }

  const MachineOperand *DppCt = getNamedOperand(MI, PPU::OpName::dpp_ctrl);
  if (DppCt) {
    using namespace PPU::DPP;

    unsigned DC = DppCt->getImm();
    if (DC == DppCtrl::DPP_UNUSED1 || DC == DppCtrl::DPP_UNUSED2 ||
        DC == DppCtrl::DPP_UNUSED3 || DC > DppCtrl::DPP_LAST ||
        (DC >= DppCtrl::DPP_UNUSED4_FIRST && DC <= DppCtrl::DPP_UNUSED4_LAST) ||
        (DC >= DppCtrl::DPP_UNUSED5_FIRST && DC <= DppCtrl::DPP_UNUSED5_LAST) ||
        (DC >= DppCtrl::DPP_UNUSED6_FIRST && DC <= DppCtrl::DPP_UNUSED6_LAST) ||
        (DC >= DppCtrl::DPP_UNUSED7_FIRST && DC <= DppCtrl::DPP_UNUSED7_LAST) ||
        (DC >= DppCtrl::DPP_UNUSED8_FIRST && DC <= DppCtrl::DPP_UNUSED8_LAST)) {
      ErrInfo = "Invalid dpp_ctrl value";
      return false;
    }
    if (DC >= DppCtrl::WAVE_SHL1 && DC <= DppCtrl::WAVE_ROR1 &&
        ST.getGeneration() >= PPUSubtarget::GFX10) {
      ErrInfo = "Invalid dpp_ctrl value: "
                "wavefront shifts are not supported on GFX10+";
      return false;
    }
    if (DC >= DppCtrl::BCAST15 && DC <= DppCtrl::BCAST31 &&
        ST.getGeneration() >= PPUSubtarget::GFX10) {
      ErrInfo = "Invalid dpp_ctrl value: "
                "broadcasts are not supported on GFX10+";
      return false;
    }
    if (DC >= DppCtrl::ROW_SHARE_FIRST && DC <= DppCtrl::ROW_XMASK_LAST &&
        ST.getGeneration() < PPUSubtarget::GFX10) {
      ErrInfo = "Invalid dpp_ctrl value: "
                "row_share and row_xmask are not supported before GFX10";
      return false;
    }
  }
  */

  return true;
}

unsigned PPUInstrInfo::getVALUOp(const MachineInstr &MI) const {
  switch (MI.getOpcode()) {
  default: return PPU::INSTRUCTION_LIST_END;
           /*
  case PPU::REG_SEQUENCE: return PPU::REG_SEQUENCE;
  case PPU::COPY: return PPU::COPY;
  case PPU::PHI: return PPU::PHI;
  case PPU::INSERT_SUBREG: return PPU::INSERT_SUBREG;
  case PPU::WQM: return PPU::WQM;
  case PPU::SOFT_WQM: return PPU::SOFT_WQM;
  case PPU::WWM: return PPU::WWM;
  case PPU::S_MOV_B32: {
    const MachineRegisterInfo &MRI = MI.getParent()->getParent()->getRegInfo();
    return MI.getOperand(1).isReg() || PPU::VMOV;
  }
  case PPU::S_ADD_I32:
    return ST.hasAddNoCarry() ? PPU::V_ADD_U32_e64 : PPU::V_ADD_I32_e32;
  case PPU::S_ADDC_U32:
    return PPU::V_ADDC_U32_e32;
  case PPU::S_SUB_I32:
    return ST.hasAddNoCarry() ? PPU::V_SUB_U32_e64 : PPU::V_SUB_I32_e32;
    // FIXME: These are not consistently handled, and selected when the carry is
    // used.
  case PPU::S_ADD_U32:
    return PPU::V_ADD_I32_e32;
  case PPU::S_SUB_U32:
    return PPU::V_SUB_I32_e32;
  case PPU::S_SUBB_U32: return PPU::V_SUBB_U32_e32;
  case PPU::S_MUL_I32: return PPU::V_MUL_LO_U32;
  case PPU::S_MUL_HI_U32: return PPU::V_MUL_HI_U32;
  case PPU::S_MUL_HI_I32: return PPU::V_MUL_HI_I32;
  case PPU::S_AND_B32: return PPU::V_AND_B32_e64;
  case PPU::S_OR_B32: return PPU::V_OR_B32_e64;
  case PPU::S_XOR_B32: return PPU::V_XOR_B32_e64;
  case PPU::S_XNOR_B32:
    return ST.hasDLInsts() ? PPU::V_XNOR_B32_e64 : PPU::INSTRUCTION_LIST_END;
  case PPU::S_MIN_I32: return PPU::V_MIN_I32_e64;
  case PPU::S_MIN_U32: return PPU::V_MIN_U32_e64;
  case PPU::S_MAX_I32: return PPU::V_MAX_I32_e64;
  case PPU::S_MAX_U32: return PPU::V_MAX_U32_e64;
  case PPU::S_ASHR_I32: return PPU::V_ASHR_I32_e32;
  case PPU::S_ASHR_I64: return PPU::V_ASHR_I64;
  case PPU::S_LSHL_B32: return PPU::V_LSHL_B32_e32;
  case PPU::S_LSHL_B64: return PPU::V_LSHL_B64;
  case PPU::S_LSHR_B32: return PPU::V_LSHR_B32_e32;
  case PPU::S_LSHR_B64: return PPU::V_LSHR_B64;
  case PPU::S_SEXT_I32_I8: return PPU::V_BFE_I32;
  case PPU::S_SEXT_I32_I16: return PPU::V_BFE_I32;
  case PPU::S_BFE_U32: return PPU::V_BFE_U32;
  case PPU::S_BFE_I32: return PPU::V_BFE_I32;
  case PPU::S_BFM_B32: return PPU::V_BFM_B32_e64;
  case PPU::S_BREV_B32: return PPU::V_BFREV_B32_e32;
  case PPU::S_NOT_B32: return PPU::V_NOT_B32_e32;
  case PPU::S_NOT_B64: return PPU::V_NOT_B32_e32;
  case PPU::S_CMP_EQ_I32: return PPU::V_CMP_EQ_I32_e32;
  case PPU::S_CMP_LG_I32: return PPU::V_CMP_NE_I32_e32;
  case PPU::S_CMP_GT_I32: return PPU::V_CMP_GT_I32_e32;
  case PPU::S_CMP_GE_I32: return PPU::V_CMP_GE_I32_e32;
  case PPU::S_CMP_LT_I32: return PPU::V_CMP_LT_I32_e32;
  case PPU::S_CMP_LE_I32: return PPU::V_CMP_LE_I32_e32;
  case PPU::S_CMP_EQ_U32: return PPU::V_CMP_EQ_U32_e32;
  case PPU::S_CMP_LG_U32: return PPU::V_CMP_NE_U32_e32;
  case PPU::S_CMP_GT_U32: return PPU::V_CMP_GT_U32_e32;
  case PPU::S_CMP_GE_U32: return PPU::V_CMP_GE_U32_e32;
  case PPU::S_CMP_LT_U32: return PPU::V_CMP_LT_U32_e32;
  case PPU::S_CMP_LE_U32: return PPU::V_CMP_LE_U32_e32;
  case PPU::S_CMP_EQ_U64: return PPU::V_CMP_EQ_U64_e32;
  case PPU::S_CMP_LG_U64: return PPU::V_CMP_NE_U64_e32;
  case PPU::S_BCNT1_I32_B32: return PPU::V_BCNT_U32_B32_e64;
  case PPU::S_FF1_I32_B32: return PPU::V_FFBL_B32_e32;
  case PPU::S_FLBIT_I32_B32: return PPU::V_FFBH_U32_e32;
  case PPU::S_FLBIT_I32: return PPU::V_FFBH_I32_e64;
  case PPU::S_CBRANCH_SCC0: return PPU::S_CBRANCH_VCCZ;
  case PPU::S_CBRANCH_SCC1: return PPU::S_CBRANCH_VCCNZ;
                            */
  }
  llvm_unreachable(
      "Unexpected scalar opcode without corresponding vector one!");
}

const TargetRegisterClass *PPUInstrInfo::getOpRegClass(const MachineInstr &MI,
                                                      unsigned OpNo) const {
  const MachineRegisterInfo &MRI = MI.getParent()->getParent()->getRegInfo();
  const MCInstrDesc &Desc = get(MI.getOpcode());
  if (MI.isVariadic() || OpNo >= Desc.getNumOperands() ||
      Desc.OpInfo[OpNo].RegClass == -1) {
    Register Reg = MI.getOperand(OpNo).getReg();

    if (Register::isVirtualRegister(Reg))
      return MRI.getRegClass(Reg);
    return RI.getPhysRegClass(Reg);
  }

  unsigned RCID = Desc.OpInfo[OpNo].RegClass;
  return RI.getRegClass(RCID);
}

void PPUInstrInfo::legalizeOpWithMove(MachineInstr &MI, unsigned OpIdx) const {
    /*
  MachineBasicBlock::iterator I = MI;
  MachineBasicBlock *MBB = MI.getParent();
  MachineOperand &MO = MI.getOperand(OpIdx);
  MachineRegisterInfo &MRI = MBB->getParent()->getRegInfo();
  const PPURegisterInfo *TRI = static_cast<const PPURegisterInfo*>(MRI.getTargetRegisterInfo());
  unsigned RCID = get(MI.getOpcode()).OpInfo[OpIdx].RegClass;
  const TargetRegisterClass *RC = RI.getRegClass(RCID);
  unsigned Size = TRI->getRegSizeInBits(*RC);
  // unsigned Opcode = (Size == 64) ? PPU::V_MOV_B64_PSEUDO : PPU::V_MOV_B32_e32;
  unsigned Opcode = PPU::VMOV;
  if (MO.isReg())
    Opcode = PPU::COPY;
  else if (RI.isSGPRClass(RC))
    Opcode = PPU::SMOV;

  const TargetRegisterClass *VRC = RI.getEquivalentVGPRClass(RC);
  VRC = &PPU::TPRRegClass;

  Register Reg = MRI.createVirtualRegister(VRC);
  DebugLoc DL = MBB->findDebugLoc(I);
  BuildMI(*MI.getParent(), I, DL, get(Opcode), Reg).add(MO);
  MO.ChangeToRegister(Reg, false);
  */
}

unsigned PPUInstrInfo::buildExtractSubReg(MachineBasicBlock::iterator MI,
                                         MachineRegisterInfo &MRI,
                                         MachineOperand &SuperReg,
                                         const TargetRegisterClass *SuperRC,
                                         unsigned SubIdx,
                                         const TargetRegisterClass *SubRC)
                                         const {
  /*
  MachineBasicBlock *MBB = MI->getParent();
  DebugLoc DL = MI->getDebugLoc();
  Register SubReg = MRI.createVirtualRegister(SubRC);

  if (SuperReg.getSubReg() == PPU::NoSubRegister) {
    BuildMI(*MBB, MI, DL, get(TargetOpcode::COPY), SubReg)
      .addReg(SuperReg.getReg(), 0, SubIdx);
    return SubReg;
  }

  // Just in case the super register is itself a sub-register, copy it to a new
  // value so we don't need to worry about merging its subreg index with the
  // SubIdx passed to this function. The register coalescer should be able to
  // eliminate this extra copy.
  Register NewSuperReg = MRI.createVirtualRegister(SuperRC);

  BuildMI(*MBB, MI, DL, get(TargetOpcode::COPY), NewSuperReg)
    .addReg(SuperReg.getReg(), 0, SuperReg.getSubReg());

  BuildMI(*MBB, MI, DL, get(TargetOpcode::COPY), SubReg)
    .addReg(NewSuperReg, 0, SubIdx);

  return SubReg;
  */
  return false;
}

  /*
MachineOperand PPUInstrInfo::buildExtractSubRegOrImm(
  MachineBasicBlock::iterator MII,
  MachineRegisterInfo &MRI,
  MachineOperand &Op,
  const TargetRegisterClass *SuperRC,
  unsigned SubIdx,
  const TargetRegisterClass *SubRC) const {
  if (Op.isImm()) {
    if (SubIdx == PPU::sub0)
      return MachineOperand::CreateImm(static_cast<int32_t>(Op.getImm()));
    if (SubIdx == PPU::sub1)
      return MachineOperand::CreateImm(static_cast<int32_t>(Op.getImm() >> 32));

    llvm_unreachable("Unhandled register index for immediate");
  }

  unsigned SubReg = buildExtractSubReg(MII, MRI, Op, SuperRC,
                                       SubIdx, SubRC);
  return MachineOperand::CreateReg(SubReg, false);
}
  */

// Change the order of operands from (0, 1, 2) to (0, 2, 1)
void PPUInstrInfo::swapOperands(MachineInstr &Inst) const {
  assert(Inst.getNumExplicitOperands() == 3);
  MachineOperand Op1 = Inst.getOperand(1);
  Inst.RemoveOperand(1);
  Inst.addOperand(Op1);
}

bool PPUInstrInfo::isLegalRegOperand(const MachineRegisterInfo &MRI,
                                    const MCOperandInfo &OpInfo,
                                    const MachineOperand &MO) const {
  if (!MO.isReg())
    return false;

  Register Reg = MO.getReg();
  const TargetRegisterClass *RC = Register::isVirtualRegister(Reg)
                                      ? MRI.getRegClass(Reg)
                                      : RI.getPhysRegClass(Reg);

  const PPURegisterInfo *TRI = static_cast<const PPURegisterInfo*>(MRI.getTargetRegisterInfo());
  RC = TRI->getSubRegClass(RC, MO.getSubReg());

  // In order to be legal, the common sub-class must be equal to the
  // class of the current operand.  For example:
  //
  // v_mov_b32 s0 ; Operand defined as vsrc_b32
  //              ; RI.getCommonSubClass(s0,vsrc_b32) = sgpr ; LEGAL
  //
  // s_sendmsg 0, s0 ; Operand defined as m0reg
  //                 ; RI.getCommonSubClass(s0,m0reg) = m0reg ; NOT LEGAL

  return RI.getCommonSubClass(RC, RI.getRegClass(OpInfo.RegClass)) == RC;
}

bool PPUInstrInfo::isLegalVSrcOperand(const MachineRegisterInfo &MRI,
                                     const MCOperandInfo &OpInfo,
                                     const MachineOperand &MO) const {
  if (MO.isReg())
    return isLegalRegOperand(MRI, OpInfo, MO);

  // Handle non-register types that are treated like immediates.
  assert(MO.isImm() || MO.isTargetIndex() || MO.isFI() || MO.isGlobal());
  return true;
}

bool PPUInstrInfo::isOperandLegal(const MachineInstr &MI, unsigned OpIdx,
                                 const MachineOperand *MO) const {
    /*
  const MachineFunction &MF = *MI.getParent()->getParent();
  const MachineRegisterInfo &MRI = MF.getRegInfo();
  const MCInstrDesc &InstDesc = MI.getDesc();
  const MCOperandInfo &OpInfo = InstDesc.OpInfo[OpIdx];
  const PPUSubtarget &ST = MF.getSubtarget<PPUSubtarget>();
  const TargetRegisterClass *DefinedRC =
      OpInfo.RegClass != -1 ? RI.getRegClass(OpInfo.RegClass) : nullptr;
  if (!MO)
    MO = &MI.getOperand(OpIdx);

  int ConstantBusLimit = ST.getConstantBusLimit(MI.getOpcode());
  int VOP3LiteralLimit = ST.hasVOP3Literal() ? 1 : 0;
  if (isVALU(MI) && usesConstantBus(MRI, *MO, OpInfo)) {
    if (isVOP3(MI) && isLiteralConstantLike(*MO, OpInfo) && !VOP3LiteralLimit--)
      return false;

    SmallDenseSet<RegSubRegPair> SGPRsUsed;
    if (MO->isReg())
      SGPRsUsed.insert(RegSubRegPair(MO->getReg(), MO->getSubReg()));

    for (unsigned i = 0, e = MI.getNumOperands(); i != e; ++i) {
      if (i == OpIdx)
        continue;
      const MachineOperand &Op = MI.getOperand(i);
      if (Op.isReg()) {
        RegSubRegPair SGPR(Op.getReg(), Op.getSubReg());
        if (!SGPRsUsed.count(SGPR) &&
            usesConstantBus(MRI, Op, InstDesc.OpInfo[i])) {
          if (--ConstantBusLimit <= 0)
            return false;
          SGPRsUsed.insert(SGPR);
        }
      } else if (InstDesc.OpInfo[i].OperandType == PPU::OPERAND_KIMM32) {
        if (--ConstantBusLimit <= 0)
          return false;
      } else if (isVOP3(MI) && PPU::isSISrcOperand(InstDesc, i) &&
                 isLiteralConstantLike(Op, InstDesc.OpInfo[i])) {
        if (!VOP3LiteralLimit--)
          return false;
        if (--ConstantBusLimit <= 0)
          return false;
      }
    }
  }

  if (MO->isReg()) {
    assert(DefinedRC);
    return isLegalRegOperand(MRI, OpInfo, *MO);
  }

  // Handle non-register types that are treated like immediates.
  assert(MO->isImm() || MO->isTargetIndex() || MO->isFI() || MO->isGlobal());

  if (!DefinedRC) {
    // This operand expects an immediate.
    return true;
  }
  */

  return isImmOperandLegal(MI, OpIdx, *MO);

}

void PPUInstrInfo::legalizeOperandsVOP2(MachineRegisterInfo &MRI,
                                       MachineInstr &MI) const {
    /*
  unsigned Opc = MI.getOpcode();
  const MCInstrDesc &InstrDesc = get(Opc);

  int Src0Idx = PPU::getNamedOperandIdx(Opc, PPU::OpName::src0);
  MachineOperand &Src0 = MI.getOperand(Src0Idx);

  int Src1Idx = PPU::getNamedOperandIdx(Opc, PPU::OpName::src1);
  MachineOperand &Src1 = MI.getOperand(Src1Idx);

  // If there is an implicit SGPR use such as VCC use for v_addc_u32/v_subb_u32
  // we need to only have one constant bus use before GFX10.
  bool HasImplicitSGPR = findImplicitSGPRRead(MI) != PPU::NoRegister;
  if (HasImplicitSGPR && ST.getConstantBusLimit(Opc) <= 1 &&
      Src0.isReg() && (RI.isSGPRReg(MRI, Src0.getReg()) ||
       isLiteralConstantLike(Src0, InstrDesc.OpInfo[Src0Idx])))
    legalizeOpWithMove(MI, Src0Idx);

  // Special case: V_WRITELANE_B32 accepts only immediate or SGPR operands for
  // both the value to write (src0) and lane select (src1).  Fix up non-SGPR
  // src0/src1 with V_READFIRSTLANE.
  if (Opc == PPU::V_WRITELANE_B32) {
    const DebugLoc &DL = MI.getDebugLoc();
    if (Src0.isReg() && RI.isVGPR(MRI, Src0.getReg())) {
      Register Reg = MRI.createVirtualRegister(&PPU::SReg_32_XM0RegClass);
      BuildMI(*MI.getParent(), MI, DL, get(PPU::V_READFIRSTLANE_B32), Reg)
          .add(Src0);
      Src0.ChangeToRegister(Reg, false);
    }
    if (Src1.isReg() && RI.isVGPR(MRI, Src1.getReg())) {
      Register Reg = MRI.createVirtualRegister(&PPU::SReg_32_XM0RegClass);
      const DebugLoc &DL = MI.getDebugLoc();
      BuildMI(*MI.getParent(), MI, DL, get(PPU::V_READFIRSTLANE_B32), Reg)
          .add(Src1);
      Src1.ChangeToRegister(Reg, false);
    }
    return;
  }


  // VOP2 src0 instructions support all operand types, so we don't need to check
  // their legality. If src1 is already legal, we don't need to do anything.
  if (isLegalRegOperand(MRI, InstrDesc.OpInfo[Src1Idx], Src1))
    return;

  // Special case: V_READLANE_B32 accepts only immediate or SGPR operands for
  // lane select. Fix up using V_READFIRSTLANE, since we assume that the lane
  // select is uniform.
  if (Opc == PPU::V_READLANE_B32 && Src1.isReg() &&
      RI.isVGPR(MRI, Src1.getReg())) {
    Register Reg = MRI.createVirtualRegister(&PPU::SReg_32_XM0RegClass);
    const DebugLoc &DL = MI.getDebugLoc();
    BuildMI(*MI.getParent(), MI, DL, get(PPU::V_READFIRSTLANE_B32), Reg)
        .add(Src1);
    Src1.ChangeToRegister(Reg, false);
    return;
  }

  // We do not use commuteInstruction here because it is too aggressive and will
  // commute if it is possible. We only want to commute here if it improves
  // legality. This can be called a fairly large number of times so don't waste
  // compile time pointlessly swapping and checking legality again.
  if (HasImplicitSGPR || !MI.isCommutable()) {
    legalizeOpWithMove(MI, Src1Idx);
    return;
  }

  // If src0 can be used as src1, commuting will make the operands legal.
  // Otherwise we have to give up and insert a move.
  //
  // TODO: Other immediate-like operand kinds could be commuted if there was a
  // MachineOperand::ChangeTo* for them.
  if ((!Src1.isImm() && !Src1.isReg()) ||
      !isLegalRegOperand(MRI, InstrDesc.OpInfo[Src1Idx], Src0)) {
    legalizeOpWithMove(MI, Src1Idx);
    return;
  }

  int CommutedOpc = commuteOpcode(MI);
  if (CommutedOpc == -1) {
    legalizeOpWithMove(MI, Src1Idx);
    return;
  }

  MI.setDesc(get(CommutedOpc));

  Register Src0Reg = Src0.getReg();
  unsigned Src0SubReg = Src0.getSubReg();
  bool Src0Kill = Src0.isKill();

  if (Src1.isImm())
    Src0.ChangeToImmediate(Src1.getImm());
  else if (Src1.isReg()) {
    Src0.ChangeToRegister(Src1.getReg(), false, false, Src1.isKill());
    Src0.setSubReg(Src1.getSubReg());
  } else
    llvm_unreachable("Should only have register or immediate operands");

  Src1.ChangeToRegister(Src0Reg, false, false, Src0Kill);
  Src1.setSubReg(Src0SubReg);
  fixImplicitOperands(MI);
  */
}

// Legalize VOP3 operands. All operand types are supported for any operand
// but only one literal constant and only starting from GFX10.
void PPUInstrInfo::legalizeOperandsVOP3(MachineRegisterInfo &MRI,
                                       MachineInstr &MI) const {
    /*
  unsigned Opc = MI.getOpcode();

  int VOP3Idx[3] = {
    PPU::getNamedOperandIdx(Opc, PPU::OpName::src0),
    PPU::getNamedOperandIdx(Opc, PPU::OpName::src1),
    PPU::getNamedOperandIdx(Opc, PPU::OpName::src2)
  };

  if (Opc == PPU::V_PERMLANE16_B32 ||
      Opc == PPU::V_PERMLANEX16_B32) {
    // src1 and src2 must be scalar
    MachineOperand &Src1 = MI.getOperand(VOP3Idx[1]);
    MachineOperand &Src2 = MI.getOperand(VOP3Idx[2]);
    const DebugLoc &DL = MI.getDebugLoc();
    if (Src1.isReg() && !RI.isSGPRClass(MRI.getRegClass(Src1.getReg()))) {
      Register Reg = MRI.createVirtualRegister(&PPU::SReg_32_XM0RegClass);
      BuildMI(*MI.getParent(), MI, DL, get(PPU::V_READFIRSTLANE_B32), Reg)
        .add(Src1);
      Src1.ChangeToRegister(Reg, false);
    }
    if (Src2.isReg() && !RI.isSGPRClass(MRI.getRegClass(Src2.getReg()))) {
      Register Reg = MRI.createVirtualRegister(&PPU::SReg_32_XM0RegClass);
      BuildMI(*MI.getParent(), MI, DL, get(PPU::V_READFIRSTLANE_B32), Reg)
        .add(Src2);
      Src2.ChangeToRegister(Reg, false);
    }
  }

  // Find the one SGPR operand we are allowed to use.
  int ConstantBusLimit = ST.getConstantBusLimit(Opc);
  int LiteralLimit = ST.hasVOP3Literal() ? 1 : 0;
  SmallDenseSet<unsigned> SGPRsUsed;
  unsigned SGPRReg = findUsedSGPR(MI, VOP3Idx);
  if (SGPRReg != PPU::NoRegister) {
    SGPRsUsed.insert(SGPRReg);
    --ConstantBusLimit;
  }

  for (unsigned i = 0; i < 3; ++i) {
    int Idx = VOP3Idx[i];
    if (Idx == -1)
      break;
    MachineOperand &MO = MI.getOperand(Idx);

    if (!MO.isReg()) {
      if (!isLiteralConstantLike(MO, get(Opc).OpInfo[Idx]))
        continue;

      if (LiteralLimit > 0 && ConstantBusLimit > 0) {
        --LiteralLimit;
        --ConstantBusLimit;
        continue;
      }

      --LiteralLimit;
      --ConstantBusLimit;
      legalizeOpWithMove(MI, Idx);
      continue;
    }

    if (!RI.isSGPRClass(MRI.getRegClass(MO.getReg())))
      continue; // VGPRs are legal

    // We can use one SGPR in each VOP3 instruction prior to GFX10
    // and two starting from GFX10.
    if (SGPRsUsed.count(MO.getReg()))
      continue;
    if (ConstantBusLimit > 0) {
      SGPRsUsed.insert(MO.getReg());
      --ConstantBusLimit;
      continue;
    }

    // If we make it this far, then the operand is not legal and we must
    // legalize it.
    legalizeOpWithMove(MI, Idx);
  }
*/
}

unsigned PPUInstrInfo::readlaneVGPRToSGPR(unsigned SrcReg, MachineInstr &UseMI,
                                         MachineRegisterInfo &MRI) const {
    /*
  const TargetRegisterClass *VRC = MRI.getRegClass(SrcReg);
  const TargetRegisterClass *SRC = RI.getEquivalentSGPRClass(VRC);
  Register DstReg = MRI.createVirtualRegister(SRC);
  unsigned SubRegs = RI.getRegSizeInBits(*VRC) / 32;


  if (SubRegs == 1) {
    BuildMI(*UseMI.getParent(), UseMI, UseMI.getDebugLoc(),
            get(PPU::V_READFIRSTLANE_B32), DstReg)
        .addReg(SrcReg);
    return DstReg;
  }

  SmallVector<unsigned, 8> SRegs;
  for (unsigned i = 0; i < SubRegs; ++i) {
    Register SGPR = MRI.createVirtualRegister(&PPU::GPRRegClass);
    BuildMI(*UseMI.getParent(), UseMI, UseMI.getDebugLoc(),
            get(PPU::V_READFIRSTLANE_B32), SGPR)
        .addReg(SrcReg, 0, RI.getSubRegFromChannel(i));
    SRegs.push_back(SGPR);
  }

  MachineInstrBuilder MIB =
      BuildMI(*UseMI.getParent(), UseMI, UseMI.getDebugLoc(),
              get(PPU::REG_SEQUENCE), DstReg);
  for (unsigned i = 0; i < SubRegs; ++i) {
    MIB.addReg(SRegs[i]);
    MIB.addImm(RI.getSubRegFromChannel(i));
  }
  return DstReg;
  */
    return 0;
}

void PPUInstrInfo::legalizeOperandsSMRD(MachineRegisterInfo &MRI,
                                       MachineInstr &MI) const {
/*
  // If the pointer is store in VGPRs, then we need to move them to
  // SGPRs using v_readfirstlane.  This is safe because we only select
  // loads with uniform pointers to SMRD instruction so we know the
  // pointer value is uniform.
  MachineOperand *SBase = getNamedOperand(MI, PPU::OpName::sbase);
  if (SBase && !RI.isSGPRClass(MRI.getRegClass(SBase->getReg()))) {
    unsigned SGPR = readlaneVGPRToSGPR(SBase->getReg(), MI, MRI);
    SBase->setReg(SGPR);
  }
  MachineOperand *SOff = getNamedOperand(MI, PPU::OpName::soff);
  if (SOff && !RI.isSGPRClass(MRI.getRegClass(SOff->getReg()))) {
    unsigned SGPR = readlaneVGPRToSGPR(SOff->getReg(), MI, MRI);
    SOff->setReg(SGPR);
  }
*/
}

void PPUInstrInfo::legalizeGenericOperand(MachineBasicBlock &InsertMBB,
                                         MachineBasicBlock::iterator I,
                                         const TargetRegisterClass *DstRC,
                                         MachineOperand &Op,
                                         MachineRegisterInfo &MRI,
                                         const DebugLoc &DL) const {
    /*
  Register OpReg = Op.getReg();
  unsigned OpSubReg = Op.getSubReg();

  const TargetRegisterClass *OpRC = RI.getSubClassWithSubReg(
      RI.getRegClassForReg(MRI, OpReg), OpSubReg);

  // Check if operand is already the correct register class.
  if (DstRC == OpRC)
    return;

  Register DstReg = MRI.createVirtualRegister(DstRC);
  MachineInstr *Copy =
      BuildMI(InsertMBB, I, DL, get(PPU::COPY), DstReg).add(Op);

  Op.setReg(DstReg);
  Op.setSubReg(0);

  MachineInstr *Def = MRI.getVRegDef(OpReg);
  if (!Def)
    return;

  // Try to eliminate the copy if it is copying an immediate value.
  if (Def->isMoveImmediate())
    FoldImmediate(*Copy, *Def, OpReg, &MRI);

  bool ImpDef = Def->isImplicitDef();
  while (!ImpDef && Def && Def->isCopy()) {
    Def = MRI.getUniqueVRegDef(Def->getOperand(1).getReg());
    ImpDef = Def && Def->isImplicitDef();
  }
  if (!RI.isSGPRClass(DstRC) && !Copy->readsRegister(PPU::EXEC, &RI) &&
      !ImpDef)
    Copy->addOperand(MachineOperand::CreateReg(PPU::EXEC, false, true));
    */
}

// Emit the actual waterfall loop, executing the wrapped instruction for each
// unique value of \p Rsrc across all lanes. In the best case we execute 1
// iteration, in the worst case we execute 64 (once per lane).
static void
emitLoadSRsrcFromVGPRLoop(const PPUInstrInfo &TII, MachineRegisterInfo &MRI,
                          MachineBasicBlock &OrigBB, MachineBasicBlock &LoopBB,
                          const DebugLoc &DL, MachineOperand &Rsrc) {
    /*
  MachineFunction &MF = *OrigBB.getParent();
  const PPUSubtarget &ST = MF.getSubtarget<PPUSubtarget>();
  const SIRegisterInfo *TRI = ST.getRegisterInfo();
  unsigned Exec = ST.isWave32() ? PPU::EXEC_LO : PPU::EXEC;
  unsigned SaveExecOpc =
      ST.isWave32() ? PPU::S_AND_SAVEEXEC_B32 : PPU::S_AND_SAVEEXEC_B64;
  unsigned XorTermOpc =
      ST.isWave32() ? PPU::S_XOR_B32_term : PPU::S_XOR_B64_term;
  unsigned AndOpc =
      ST.isWave32() ? PPU::S_AND_B32 : PPU::S_AND_B64;
  const auto *BoolXExecRC = TRI->getRegClass(PPU::SReg_1_XEXECRegClassID);

  MachineBasicBlock::iterator I = LoopBB.begin();

  Register VRsrc = Rsrc.getReg();
  unsigned VRsrcUndef = getUndefRegState(Rsrc.isUndef());

  Register SaveExec = MRI.createVirtualRegister(BoolXExecRC);
  Register CondReg0 = MRI.createVirtualRegister(BoolXExecRC);
  Register CondReg1 = MRI.createVirtualRegister(BoolXExecRC);
  Register AndCond = MRI.createVirtualRegister(BoolXExecRC);
  Register SRsrcSub0 = MRI.createVirtualRegister(&PPU::GPRRegClass);
  Register SRsrcSub1 = MRI.createVirtualRegister(&PPU::GPRRegClass);
  Register SRsrcSub2 = MRI.createVirtualRegister(&PPU::GPRRegClass);
  Register SRsrcSub3 = MRI.createVirtualRegister(&PPU::GPRRegClass);
  Register SRsrc = MRI.createVirtualRegister(&PPU::SReg_128RegClass);

  // Beginning of the loop, read the next Rsrc variant.
  BuildMI(LoopBB, I, DL, TII.get(PPU::V_READFIRSTLANE_B32), SRsrcSub0)
      .addReg(VRsrc, VRsrcUndef, PPU::sub0);
  BuildMI(LoopBB, I, DL, TII.get(PPU::V_READFIRSTLANE_B32), SRsrcSub1)
      .addReg(VRsrc, VRsrcUndef, PPU::sub1);
  BuildMI(LoopBB, I, DL, TII.get(PPU::V_READFIRSTLANE_B32), SRsrcSub2)
      .addReg(VRsrc, VRsrcUndef, PPU::sub2);
  BuildMI(LoopBB, I, DL, TII.get(PPU::V_READFIRSTLANE_B32), SRsrcSub3)
      .addReg(VRsrc, VRsrcUndef, PPU::sub3);

  BuildMI(LoopBB, I, DL, TII.get(PPU::REG_SEQUENCE), SRsrc)
      .addReg(SRsrcSub0)
      .addImm(PPU::sub0)
      .addReg(SRsrcSub1)
      .addImm(PPU::sub1)
      .addReg(SRsrcSub2)
      .addImm(PPU::sub2)
      .addReg(SRsrcSub3)
      .addImm(PPU::sub3);

  // Update Rsrc operand to use the SGPR Rsrc.
  Rsrc.setReg(SRsrc);
  Rsrc.setIsKill(true);

  // Identify all lanes with identical Rsrc operands in their VGPRs.
  BuildMI(LoopBB, I, DL, TII.get(PPU::V_CMP_EQ_U64_e64), CondReg0)
      .addReg(SRsrc, 0, PPU::sub0_sub1)
      .addReg(VRsrc, 0, PPU::sub0_sub1);
  BuildMI(LoopBB, I, DL, TII.get(PPU::V_CMP_EQ_U64_e64), CondReg1)
      .addReg(SRsrc, 0, PPU::sub2_sub3)
      .addReg(VRsrc, 0, PPU::sub2_sub3);
  BuildMI(LoopBB, I, DL, TII.get(AndOpc), AndCond)
      .addReg(CondReg0)
      .addReg(CondReg1);

  MRI.setSimpleHint(SaveExec, AndCond);

  // Update EXEC to matching lanes, saving original to SaveExec.
  BuildMI(LoopBB, I, DL, TII.get(SaveExecOpc), SaveExec)
      .addReg(AndCond, RegState::Kill);

  // The original instruction is here; we insert the terminators after it.
  I = LoopBB.end();

  // Update EXEC, switch all done bits to 0 and all todo bits to 1.
  BuildMI(LoopBB, I, DL, TII.get(XorTermOpc), Exec)
      .addReg(Exec)
      .addReg(SaveExec);
  BuildMI(LoopBB, I, DL, TII.get(PPU::S_CBRANCH_EXECNZ)).addMBB(&LoopBB);
  */
}

// Build a waterfall loop around \p MI, replacing the VGPR \p Rsrc register
// with SGPRs by iterating over all unique values across all lanes.
static void loadSRsrcFromVGPR(const PPUInstrInfo &TII, MachineInstr &MI,
                              MachineOperand &Rsrc, MachineDominatorTree *MDT) {
    /*
  MachineBasicBlock &MBB = *MI.getParent();
  MachineFunction &MF = *MBB.getParent();
  const PPUSubtarget &ST = MF.getSubtarget<PPUSubtarget>();
  const SIRegisterInfo *TRI = ST.getRegisterInfo();
  MachineRegisterInfo &MRI = MF.getRegInfo();
  MachineBasicBlock::iterator I(&MI);
  const DebugLoc &DL = MI.getDebugLoc();
  unsigned Exec = ST.isWave32() ? PPU::EXEC_LO : PPU::EXEC;
  unsigned MovExecOpc = ST.isWave32() ? PPU::S_MOV_B32 : PPU::S_MOV_B64;
  const auto *BoolXExecRC = TRI->getRegClass(PPU::SReg_1_XEXECRegClassID);

  Register SaveExec = MRI.createVirtualRegister(BoolXExecRC);

  // Save the EXEC mask
  BuildMI(MBB, I, DL, TII.get(MovExecOpc), SaveExec).addReg(Exec);

  // Killed uses in the instruction we are waterfalling around will be
  // incorrect due to the added control-flow.
  for (auto &MO : MI.uses()) {
    if (MO.isReg() && MO.isUse()) {
      MRI.clearKillFlags(MO.getReg());
    }
  }

  // To insert the loop we need to split the block. Move everything after this
  // point to a new block, and insert a new empty block between the two.
  MachineBasicBlock *LoopBB = MF.CreateMachineBasicBlock();
  MachineBasicBlock *RemainderBB = MF.CreateMachineBasicBlock();
  MachineFunction::iterator MBBI(MBB);
  ++MBBI;

  MF.insert(MBBI, LoopBB);
  MF.insert(MBBI, RemainderBB);

  LoopBB->addSuccessor(LoopBB);
  LoopBB->addSuccessor(RemainderBB);

  // Move MI to the LoopBB, and the remainder of the block to RemainderBB.
  MachineBasicBlock::iterator J = I++;
  RemainderBB->transferSuccessorsAndUpdatePHIs(&MBB);
  RemainderBB->splice(RemainderBB->begin(), &MBB, I, MBB.end());
  LoopBB->splice(LoopBB->begin(), &MBB, J);

  MBB.addSuccessor(LoopBB);

  // Update dominators. We know that MBB immediately dominates LoopBB, that
  // LoopBB immediately dominates RemainderBB, and that RemainderBB immediately
  // dominates all of the successors transferred to it from MBB that MBB used
  // to dominate.
  if (MDT) {
    MDT->addNewBlock(LoopBB, &MBB);
    MDT->addNewBlock(RemainderBB, LoopBB);
    for (auto &Succ : RemainderBB->successors()) {
      if (MDT->dominates(&MBB, Succ)) {
        MDT->changeImmediateDominator(Succ, RemainderBB);
      }
    }
  }

  emitLoadSRsrcFromVGPRLoop(TII, MRI, MBB, *LoopBB, DL, Rsrc);

  // Restore the EXEC mask
  MachineBasicBlock::iterator First = RemainderBB->begin();
  BuildMI(*RemainderBB, First, DL, TII.get(MovExecOpc), Exec).addReg(SaveExec);
  */
}

// Extract pointer from Rsrc and return a zero-value Rsrc replacement.
    /*
static std::tuple<unsigned, unsigned>
extractRsrcPtr(const PPUInstrInfo &TII, MachineInstr &MI, MachineOperand &Rsrc) {
  MachineBasicBlock &MBB = *MI.getParent();
  MachineFunction &MF = *MBB.getParent();
  MachineRegisterInfo &MRI = MF.getRegInfo();

  // Extract the ptr from the resource descriptor.
  unsigned RsrcPtr =
      TII.buildExtractSubReg(MI, MRI, Rsrc, &PPU::VReg_128RegClass,
                             PPU::sub0_sub1, &PPU::VReg_64RegClass);

  // Create an empty resource descriptor
  Register Zero64 = MRI.createVirtualRegister(&PPU::SReg_64RegClass);
  Register SRsrcFormatLo = MRI.createVirtualRegister(&PPU::GPRRegClass);
  Register SRsrcFormatHi = MRI.createVirtualRegister(&PPU::GPRRegClass);
  Register NewSRsrc = MRI.createVirtualRegister(&PPU::SReg_128RegClass);
  uint64_t RsrcDataFormat = TII.getDefaultRsrcDataFormat();

  // Zero64 = 0
  BuildMI(MBB, MI, MI.getDebugLoc(), TII.get(PPU::S_MOV_B64), Zero64)
      .addImm(0);

  // SRsrcFormatLo = RSRC_DATA_FORMAT{31-0}
  BuildMI(MBB, MI, MI.getDebugLoc(), TII.get(PPU::S_MOV_B32), SRsrcFormatLo)
      .addImm(RsrcDataFormat & 0xFFFFFFFF);

  // SRsrcFormatHi = RSRC_DATA_FORMAT{63-32}
  BuildMI(MBB, MI, MI.getDebugLoc(), TII.get(PPU::S_MOV_B32), SRsrcFormatHi)
      .addImm(RsrcDataFormat >> 32);

  // NewSRsrc = {Zero64, SRsrcFormat}
  BuildMI(MBB, MI, MI.getDebugLoc(), TII.get(PPU::REG_SEQUENCE), NewSRsrc)
      .addReg(Zero64)
      .addImm(PPU::sub0_sub1)
      .addReg(SRsrcFormatLo)
      .addImm(PPU::sub2)
      .addReg(SRsrcFormatHi)
      .addImm(PPU::sub3);

  return std::make_tuple(RsrcPtr, NewSRsrc);
}
*/
/*
void PPUInstrInfo::legalizeOperands(MachineInstr &MI,
                                   MachineDominatorTree *MDT) const {
  MachineFunction &MF = *MI.getParent()->getParent();
  MachineRegisterInfo &MRI = MF.getRegInfo();

  // Legalize VOP2
  if (isVOP2(MI) || isVOPC(MI)) {
    legalizeOperandsVOP2(MRI, MI);
    return;
  }

  // Legalize VOP3
  if (isVOP3(MI)) {
    legalizeOperandsVOP3(MRI, MI);
    return;
  }

  // Legalize SMRD
  if (isSMRD(MI)) {
    legalizeOperandsSMRD(MRI, MI);
    return;
  }

  // Legalize REG_SEQUENCE and PHI
  // The register class of the operands much be the same type as the register
  // class of the output.
  if (MI.getOpcode() == PPU::PHI) {
    const TargetRegisterClass *RC = nullptr, *SRC = nullptr, *VRC = nullptr;
    for (unsigned i = 1, e = MI.getNumOperands(); i != e; i += 2) {
      if (!MI.getOperand(i).isReg() ||
          !Register::isVirtualRegister(MI.getOperand(i).getReg()))
        continue;
      const TargetRegisterClass *OpRC =
          MRI.getRegClass(MI.getOperand(i).getReg());
      if (RI.hasVectorRegisters(OpRC)) {
        VRC = OpRC;
      } else {
        SRC = OpRC;
      }
    }

    // If any of the operands are VGPR registers, then they all most be
    // otherwise we will create illegal VGPR->SGPR copies when legalizing
    // them.
    if (VRC || !RI.isSGPRClass(getOpRegClass(MI, 0))) {
      if (!VRC) {
        assert(SRC);
        VRC = RI.getEquivalentVGPRClass(SRC);
      }
      RC = VRC;
    } else {
      RC = SRC;
    }

    // Update all the operands so they have the same type.
    for (unsigned I = 1, E = MI.getNumOperands(); I != E; I += 2) {
      MachineOperand &Op = MI.getOperand(I);
      if (!Op.isReg() || !Register::isVirtualRegister(Op.getReg()))
        continue;

      // MI is a PHI instruction.
      MachineBasicBlock *InsertBB = MI.getOperand(I + 1).getMBB();
      MachineBasicBlock::iterator Insert = InsertBB->getFirstTerminator();

      // Avoid creating no-op copies with the same src and dst reg class.  These
      // confuse some of the machine passes.
      legalizeGenericOperand(*InsertBB, Insert, RC, Op, MRI, MI.getDebugLoc());
    }
  }

  // REG_SEQUENCE doesn't really require operand legalization, but if one has a
  // VGPR dest type and SGPR sources, insert copies so all operands are
  // VGPRs. This seems to help operand folding / the register coalescer.
  if (MI.getOpcode() == PPU::REG_SEQUENCE) {
    MachineBasicBlock *MBB = MI.getParent();
    const TargetRegisterClass *DstRC = getOpRegClass(MI, 0);
    if (RI.hasVGPRs(DstRC)) {
      // Update all the operands so they are VGPR register classes. These may
      // not be the same register class because REG_SEQUENCE supports mixing
      // subregister index types e.g. sub0_sub1 + sub2 + sub3
      for (unsigned I = 1, E = MI.getNumOperands(); I != E; I += 2) {
        MachineOperand &Op = MI.getOperand(I);
        if (!Op.isReg() || !Register::isVirtualRegister(Op.getReg()))
          continue;

        const TargetRegisterClass *OpRC = MRI.getRegClass(Op.getReg());
        const TargetRegisterClass *VRC = RI.getEquivalentVGPRClass(OpRC);
        if (VRC == OpRC)
          continue;

        legalizeGenericOperand(*MBB, MI, VRC, Op, MRI, MI.getDebugLoc());
        Op.setIsKill();
      }
    }

    return;
  }

  // Legalize INSERT_SUBREG
  // src0 must have the same register class as dst
  if (MI.getOpcode() == PPU::INSERT_SUBREG) {
    Register Dst = MI.getOperand(0).getReg();
    Register Src0 = MI.getOperand(1).getReg();
    const TargetRegisterClass *DstRC = MRI.getRegClass(Dst);
    const TargetRegisterClass *Src0RC = MRI.getRegClass(Src0);
    if (DstRC != Src0RC) {
      MachineBasicBlock *MBB = MI.getParent();
      MachineOperand &Op = MI.getOperand(1);
      legalizeGenericOperand(*MBB, MI, DstRC, Op, MRI, MI.getDebugLoc());
    }
    return;
  }

  // Legalize SI_INIT_M0
  if (MI.getOpcode() == PPU::SI_INIT_M0) {
    MachineOperand &Src = MI.getOperand(0);
    if (Src.isReg() && RI.hasVectorRegisters(MRI.getRegClass(Src.getReg())))
      Src.setReg(readlaneVGPRToSGPR(Src.getReg(), MI, MRI));
    return;
  }

  // Legalize MIMG and MUBUF/MTBUF for shaders.
  //
  // Shaders only generate MUBUF/MTBUF instructions via intrinsics or via
  // scratch memory access. In both cases, the legalization never involves
  // conversion to the addr64 form.
  if (isMIMG(MI) ||
      (PPU::isShader(MF.getFunction().getCallingConv()) &&
       (isMUBUF(MI) || isMTBUF(MI)))) {
    MachineOperand *SRsrc = getNamedOperand(MI, PPU::OpName::srsrc);
    if (SRsrc && !RI.isSGPRClass(MRI.getRegClass(SRsrc->getReg()))) {
      unsigned SGPR = readlaneVGPRToSGPR(SRsrc->getReg(), MI, MRI);
      SRsrc->setReg(SGPR);
    }

    MachineOperand *SSamp = getNamedOperand(MI, PPU::OpName::ssamp);
    if (SSamp && !RI.isSGPRClass(MRI.getRegClass(SSamp->getReg()))) {
      unsigned SGPR = readlaneVGPRToSGPR(SSamp->getReg(), MI, MRI);
      SSamp->setReg(SGPR);
    }
    return;
  }

  // Legalize MUBUF* instructions.
  int RsrcIdx =
      PPU::getNamedOperandIdx(MI.getOpcode(), PPU::OpName::srsrc);
  if (RsrcIdx != -1) {
    // We have an MUBUF instruction
    MachineOperand *Rsrc = &MI.getOperand(RsrcIdx);
    unsigned RsrcRC = get(MI.getOpcode()).OpInfo[RsrcIdx].RegClass;
    if (RI.getCommonSubClass(MRI.getRegClass(Rsrc->getReg()),
                             RI.getRegClass(RsrcRC))) {
      // The operands are legal.
      // FIXME: We may need to legalize operands besided srsrc.
      return;
    }

    // Legalize a VGPR Rsrc.
    //
    // If the instruction is _ADDR64, we can avoid a waterfall by extracting
    // the base pointer from the VGPR Rsrc, adding it to the VAddr, then using
    // a zero-value SRsrc.
    //
    // If the instruction is _OFFSET (both idxen and offen disabled), and we
    // support ADDR64 instructions, we can convert to ADDR64 and do the same as
    // above.
    //
    // Otherwise we are on non-ADDR64 hardware, and/or we have
    // idxen/offen/bothen and we fall back to a waterfall loop.

    MachineBasicBlock &MBB = *MI.getParent();

    MachineOperand *VAddr = getNamedOperand(MI, PPU::OpName::vaddr);
    if (VAddr && PPU::getIfAddr64Inst(MI.getOpcode()) != -1) {
      // This is already an ADDR64 instruction so we need to add the pointer
      // extracted from the resource descriptor to the current value of VAddr.
      Register NewVAddrLo = MRI.createVirtualRegister(&PPU::TPRRegClass);
      Register NewVAddrHi = MRI.createVirtualRegister(&PPU::TPRRegClass);
      Register NewVAddr = MRI.createVirtualRegister(&PPU::VReg_64RegClass);

      const auto *BoolXExecRC = RI.getRegClass(PPU::SReg_1_XEXECRegClassID);
      Register CondReg0 = MRI.createVirtualRegister(BoolXExecRC);
      Register CondReg1 = MRI.createVirtualRegister(BoolXExecRC);

      unsigned RsrcPtr, NewSRsrc;
      std::tie(RsrcPtr, NewSRsrc) = extractRsrcPtr(*this, MI, *Rsrc);

      // NewVaddrLo = RsrcPtr:sub0 + VAddr:sub0
      const DebugLoc &DL = MI.getDebugLoc();
      BuildMI(MBB, MI, DL, get(PPU::V_ADD_I32_e64), NewVAddrLo)
        .addDef(CondReg0)
        .addReg(RsrcPtr, 0, PPU::sub0)
        .addReg(VAddr->getReg(), 0, PPU::sub0)
        .addImm(0);

      // NewVaddrHi = RsrcPtr:sub1 + VAddr:sub1
      BuildMI(MBB, MI, DL, get(PPU::V_ADDC_U32_e64), NewVAddrHi)
        .addDef(CondReg1, RegState::Dead)
        .addReg(RsrcPtr, 0, PPU::sub1)
        .addReg(VAddr->getReg(), 0, PPU::sub1)
        .addReg(CondReg0, RegState::Kill)
        .addImm(0);

      // NewVaddr = {NewVaddrHi, NewVaddrLo}
      BuildMI(MBB, MI, MI.getDebugLoc(), get(PPU::REG_SEQUENCE), NewVAddr)
          .addReg(NewVAddrLo)
          .addImm(PPU::sub0)
          .addReg(NewVAddrHi)
          .addImm(PPU::sub1);

      VAddr->setReg(NewVAddr);
      Rsrc->setReg(NewSRsrc);
    } else if (!VAddr && ST.hasAddr64()) {
      // This instructions is the _OFFSET variant, so we need to convert it to
      // ADDR64.
      assert(MBB.getParent()->getSubtarget<PPUSubtarget>().getGeneration()
             < PPUSubtarget::VOLCANIC_ISLANDS &&
             "FIXME: Need to emit flat atomics here");

      unsigned RsrcPtr, NewSRsrc;
      std::tie(RsrcPtr, NewSRsrc) = extractRsrcPtr(*this, MI, *Rsrc);

      Register NewVAddr = MRI.createVirtualRegister(&PPU::VReg_64RegClass);
      MachineOperand *VData = getNamedOperand(MI, PPU::OpName::vdata);
      MachineOperand *Offset = getNamedOperand(MI, PPU::OpName::offset);
      MachineOperand *SOffset = getNamedOperand(MI, PPU::OpName::soffset);
      unsigned Addr64Opcode = PPU::getAddr64Inst(MI.getOpcode());

      // Atomics rith return have have an additional tied operand and are
      // missing some of the special bits.
      MachineOperand *VDataIn = getNamedOperand(MI, PPU::OpName::vdata_in);
      MachineInstr *Addr64;

      if (!VDataIn) {
        // Regular buffer load / store.
        MachineInstrBuilder MIB =
            BuildMI(MBB, MI, MI.getDebugLoc(), get(Addr64Opcode))
                .add(*VData)
                .addReg(NewVAddr)
                .addReg(NewSRsrc)
                .add(*SOffset)
                .add(*Offset);

        // Atomics do not have this operand.
        if (const MachineOperand *GLC =
                getNamedOperand(MI, PPU::OpName::glc)) {
          MIB.addImm(GLC->getImm());
        }
        if (const MachineOperand *DLC =
                getNamedOperand(MI, PPU::OpName::dlc)) {
          MIB.addImm(DLC->getImm());
        }

        MIB.addImm(getNamedImmOperand(MI, PPU::OpName::slc));

        if (const MachineOperand *TFE =
                getNamedOperand(MI, PPU::OpName::tfe)) {
          MIB.addImm(TFE->getImm());
        }

        MIB.cloneMemRefs(MI);
        Addr64 = MIB;
      } else {
        // Atomics with return.
        Addr64 = BuildMI(MBB, MI, MI.getDebugLoc(), get(Addr64Opcode))
                     .add(*VData)
                     .add(*VDataIn)
                     .addReg(NewVAddr)
                     .addReg(NewSRsrc)
                     .add(*SOffset)
                     .add(*Offset)
                     .addImm(getNamedImmOperand(MI, PPU::OpName::slc))
                     .cloneMemRefs(MI);
      }

      MI.removeFromParent();

      // NewVaddr = {NewVaddrHi, NewVaddrLo}
      BuildMI(MBB, Addr64, Addr64->getDebugLoc(), get(PPU::REG_SEQUENCE),
              NewVAddr)
          .addReg(RsrcPtr, 0, PPU::sub0)
          .addImm(PPU::sub0)
          .addReg(RsrcPtr, 0, PPU::sub1)
          .addImm(PPU::sub1);
    } else {
      // This is another variant; legalize Rsrc with waterfall loop from VGPRs
      // to SGPRs.
      loadSRsrcFromVGPR(*this, MI, *Rsrc, MDT);
    }
  }
}
*/

void PPUInstrInfo::moveToVALU(MachineInstr &TopInst,
                             MachineDominatorTree *MDT) const {
    /*
  SetVectorType Worklist;
  Worklist.insert(&TopInst);

  while (!Worklist.empty()) {
    MachineInstr &Inst = *Worklist.pop_back_val();
    MachineBasicBlock *MBB = Inst.getParent();
    MachineRegisterInfo &MRI = MBB->getParent()->getRegInfo();

    unsigned Opcode = Inst.getOpcode();
    unsigned NewOpcode = getVALUOp(Inst);

    // Handle some special cases
    switch (Opcode) {
    default:
      break;
    case PPU::S_ADD_U64_PSEUDO:
    case PPU::S_SUB_U64_PSEUDO:
      splitScalar64BitAddSub(Worklist, Inst, MDT);
      Inst.eraseFromParent();
      continue;
    case PPU::S_ADD_I32:
    case PPU::S_SUB_I32:
      // FIXME: The u32 versions currently selected use the carry.
      if (moveScalarAddSub(Worklist, Inst, MDT))
        continue;

      // Default handling
      break;
    case PPU::S_AND_B64:
      splitScalar64BitBinaryOp(Worklist, Inst, PPU::S_AND_B32, MDT);
      Inst.eraseFromParent();
      continue;

    case PPU::S_OR_B64:
      splitScalar64BitBinaryOp(Worklist, Inst, PPU::S_OR_B32, MDT);
      Inst.eraseFromParent();
      continue;

    case PPU::S_XOR_B64:
      splitScalar64BitBinaryOp(Worklist, Inst, PPU::S_XOR_B32, MDT);
      Inst.eraseFromParent();
      continue;

    case PPU::S_NAND_B64:
      splitScalar64BitBinaryOp(Worklist, Inst, PPU::S_NAND_B32, MDT);
      Inst.eraseFromParent();
      continue;

    case PPU::S_NOR_B64:
      splitScalar64BitBinaryOp(Worklist, Inst, PPU::S_NOR_B32, MDT);
      Inst.eraseFromParent();
      continue;

    case PPU::S_XNOR_B64:
      if (ST.hasDLInsts())
        splitScalar64BitBinaryOp(Worklist, Inst, PPU::S_XNOR_B32, MDT);
      else
        splitScalar64BitXnor(Worklist, Inst, MDT);
      Inst.eraseFromParent();
      continue;

    case PPU::S_ANDN2_B64:
      splitScalar64BitBinaryOp(Worklist, Inst, PPU::S_ANDN2_B32, MDT);
      Inst.eraseFromParent();
      continue;

    case PPU::S_ORN2_B64:
      splitScalar64BitBinaryOp(Worklist, Inst, PPU::S_ORN2_B32, MDT);
      Inst.eraseFromParent();
      continue;

    case PPU::S_NOT_B64:
      splitScalar64BitUnaryOp(Worklist, Inst, PPU::S_NOT_B32);
      Inst.eraseFromParent();
      continue;

    case PPU::S_BCNT1_I32_B64:
      splitScalar64BitBCNT(Worklist, Inst);
      Inst.eraseFromParent();
      continue;

    case PPU::S_BFE_I64:
      splitScalar64BitBFE(Worklist, Inst);
      Inst.eraseFromParent();
      continue;

    case PPU::S_LSHL_B32:
      if (ST.hasOnlyRevVALUShifts()) {
        NewOpcode = PPU::V_LSHLREV_B32_e64;
        swapOperands(Inst);
      }
      break;
    case PPU::S_ASHR_I32:
      if (ST.hasOnlyRevVALUShifts()) {
        NewOpcode = PPU::V_ASHRREV_I32_e64;
        swapOperands(Inst);
      }
      break;
    case PPU::S_LSHR_B32:
      if (ST.hasOnlyRevVALUShifts()) {
        NewOpcode = PPU::V_LSHRREV_B32_e64;
        swapOperands(Inst);
      }
      break;
    case PPU::S_LSHL_B64:
      if (ST.hasOnlyRevVALUShifts()) {
        NewOpcode = PPU::V_LSHLREV_B64;
        swapOperands(Inst);
      }
      break;
    case PPU::S_ASHR_I64:
      if (ST.hasOnlyRevVALUShifts()) {
        NewOpcode = PPU::V_ASHRREV_I64;
        swapOperands(Inst);
      }
      break;
    case PPU::S_LSHR_B64:
      if (ST.hasOnlyRevVALUShifts()) {
        NewOpcode = PPU::V_LSHRREV_B64;
        swapOperands(Inst);
      }
      break;

    case PPU::S_ABS_I32:
      lowerScalarAbs(Worklist, Inst);
      Inst.eraseFromParent();
      continue;

    case PPU::S_CBRANCH_SCC0:
    case PPU::S_CBRANCH_SCC1:
      // Clear unused bits of vcc
      if (ST.isWave32())
        BuildMI(*MBB, Inst, Inst.getDebugLoc(), get(PPU::S_AND_B32),
                PPU::VCC_LO)
            .addReg(PPU::EXEC_LO)
            .addReg(PPU::VCC_LO);
      else
        BuildMI(*MBB, Inst, Inst.getDebugLoc(), get(PPU::S_AND_B64),
                PPU::VCC)
            .addReg(PPU::EXEC)
            .addReg(PPU::VCC);
      break;

    case PPU::S_BFE_U64:
    case PPU::S_BFM_B64:
      llvm_unreachable("Moving this op to VALU not implemented");

    case PPU::S_PACK_LL_B32_B16:
    case PPU::S_PACK_LH_B32_B16:
    case PPU::S_PACK_HH_B32_B16:
      movePackToVALU(Worklist, MRI, Inst);
      Inst.eraseFromParent();
      continue;

    case PPU::S_XNOR_B32:
      lowerScalarXnor(Worklist, Inst);
      Inst.eraseFromParent();
      continue;

    case PPU::S_NAND_B32:
      splitScalarNotBinop(Worklist, Inst, PPU::S_AND_B32);
      Inst.eraseFromParent();
      continue;

    case PPU::S_NOR_B32:
      splitScalarNotBinop(Worklist, Inst, PPU::S_OR_B32);
      Inst.eraseFromParent();
      continue;

    case PPU::S_ANDN2_B32:
      splitScalarBinOpN2(Worklist, Inst, PPU::S_AND_B32);
      Inst.eraseFromParent();
      continue;

    case PPU::S_ORN2_B32:
      splitScalarBinOpN2(Worklist, Inst, PPU::S_OR_B32);
      Inst.eraseFromParent();
      continue;
    }

    if (NewOpcode == PPU::INSTRUCTION_LIST_END) {
      // We cannot move this instruction to the VALU, so we should try to
      // legalize its operands instead.
      legalizeOperands(Inst, MDT);
      continue;
    }

    // Use the new VALU Opcode.
    const MCInstrDesc &NewDesc = get(NewOpcode);
    Inst.setDesc(NewDesc);

    // Remove any references to SCC. Vector instructions can't read from it, and
    // We're just about to add the implicit use / defs of VCC, and we don't want
    // both.
    for (unsigned i = Inst.getNumOperands() - 1; i > 0; --i) {
      MachineOperand &Op = Inst.getOperand(i);
      if (Op.isReg() && Op.getReg() == PPU::SCC) {
        // Only propagate through live-def of SCC.
        if (Op.isDef() && !Op.isDead())
          addSCCDefUsersToVALUWorklist(Op, Inst, Worklist);
        Inst.RemoveOperand(i);
      }
    }

    if (Opcode == PPU::S_SEXT_I32_I8 || Opcode == PPU::S_SEXT_I32_I16) {
      // We are converting these to a BFE, so we need to add the missing
      // operands for the size and offset.
      unsigned Size = (Opcode == PPU::S_SEXT_I32_I8) ? 8 : 16;
      Inst.addOperand(MachineOperand::CreateImm(0));
      Inst.addOperand(MachineOperand::CreateImm(Size));

    } else if (Opcode == PPU::S_BCNT1_I32_B32) {
      // The VALU version adds the second operand to the result, so insert an
      // extra 0 operand.
      Inst.addOperand(MachineOperand::CreateImm(0));
    }

    Inst.addImplicitDefUseOperands(*Inst.getParent()->getParent());
    fixImplicitOperands(Inst);

    if (Opcode == PPU::S_BFE_I32 || Opcode == PPU::S_BFE_U32) {
      const MachineOperand &OffsetWidthOp = Inst.getOperand(2);
      // If we need to move this to VGPRs, we need to unpack the second operand
      // back into the 2 separate ones for bit offset and width.
      assert(OffsetWidthOp.isImm() &&
             "Scalar BFE is only implemented for constant width and offset");
      uint32_t Imm = OffsetWidthOp.getImm();

      uint32_t Offset = Imm & 0x3f; // Extract bits [5:0].
      uint32_t BitWidth = (Imm & 0x7f0000) >> 16; // Extract bits [22:16].
      Inst.RemoveOperand(2);                     // Remove old immediate.
      Inst.addOperand(MachineOperand::CreateImm(Offset));
      Inst.addOperand(MachineOperand::CreateImm(BitWidth));
    }

    bool HasDst = Inst.getOperand(0).isReg() && Inst.getOperand(0).isDef();
    unsigned NewDstReg = PPU::NoRegister;
    if (HasDst) {
      Register DstReg = Inst.getOperand(0).getReg();
      if (Register::isPhysicalRegister(DstReg))
        continue;

      // Update the destination register class.
      const TargetRegisterClass *NewDstRC = getDestEquivalentVGPRClass(Inst);
      if (!NewDstRC)
        continue;

      if (Inst.isCopy() &&
          Register::isVirtualRegister(Inst.getOperand(1).getReg()) &&
          NewDstRC == RI.getRegClassForReg(MRI, Inst.getOperand(1).getReg())) {
        // Instead of creating a copy where src and dst are the same register
        // class, we just replace all uses of dst with src.  These kinds of
        // copies interfere with the heuristics MachineSink uses to decide
        // whether or not to split a critical edge.  Since the pass assumes
        // that copies will end up as machine instructions and not be
        // eliminated.
        addUsersToMoveToVALUWorklist(DstReg, MRI, Worklist);
        MRI.replaceRegWith(DstReg, Inst.getOperand(1).getReg());
        MRI.clearKillFlags(Inst.getOperand(1).getReg());
        Inst.getOperand(0).setReg(DstReg);

        // Make sure we don't leave around a dead VGPR->SGPR copy. Normally
        // these are deleted later, but at -O0 it would leave a suspicious
        // looking illegal copy of an undef register.
        for (unsigned I = Inst.getNumOperands() - 1; I != 0; --I)
          Inst.RemoveOperand(I);
        Inst.setDesc(get(PPU::IMPLICIT_DEF));
        continue;
      }

      NewDstReg = MRI.createVirtualRegister(NewDstRC);
      MRI.replaceRegWith(DstReg, NewDstReg);
    }

    // Legalize the operands
    legalizeOperands(Inst, MDT);

    if (HasDst)
      addUsersToMoveToVALUWorklist(NewDstReg, MRI, Worklist);
  }
*/
}

// Add/sub require special handling to deal with carry outs.
bool PPUInstrInfo::moveScalarAddSub(SetVectorType &Worklist, MachineInstr &Inst,
                                   MachineDominatorTree *MDT) const {
    /*
  if (ST.hasAddNoCarry()) {
    // Assume there is no user of scc since we don't select this in that case.
    // Since scc isn't used, it doesn't really matter if the i32 or u32 variant
    // is used.

    MachineBasicBlock &MBB = *Inst.getParent();
    MachineRegisterInfo &MRI = MBB.getParent()->getRegInfo();

    Register OldDstReg = Inst.getOperand(0).getReg();
    Register ResultReg = MRI.createVirtualRegister(&PPU::TPRRegClass);

    unsigned Opc = Inst.getOpcode();
    assert(Opc == PPU::S_ADD_I32 || Opc == PPU::S_SUB_I32);

    unsigned NewOpc = Opc == PPU::S_ADD_I32 ?
      PPU::V_ADD_U32_e64 : PPU::V_SUB_U32_e64;

    assert(Inst.getOperand(3).getReg() == PPU::SCC);
    Inst.RemoveOperand(3);

    Inst.setDesc(get(NewOpc));
    Inst.addOperand(MachineOperand::CreateImm(0)); // clamp bit
    Inst.addImplicitDefUseOperands(*MBB.getParent());
    MRI.replaceRegWith(OldDstReg, ResultReg);
    legalizeOperands(Inst, MDT);

    addUsersToMoveToVALUWorklist(ResultReg, MRI, Worklist);
    return true;
  }
  */

  return false;
}

void PPUInstrInfo::lowerScalarAbs(SetVectorType &Worklist,
                                 MachineInstr &Inst) const {
    /*
  MachineBasicBlock &MBB = *Inst.getParent();
  MachineRegisterInfo &MRI = MBB.getParent()->getRegInfo();
  MachineBasicBlock::iterator MII = Inst;
  DebugLoc DL = Inst.getDebugLoc();

  MachineOperand &Dest = Inst.getOperand(0);
  MachineOperand &Src = Inst.getOperand(1);
  Register TmpReg = MRI.createVirtualRegister(&PPU::TPRRegClass);
  Register ResultReg = MRI.createVirtualRegister(&PPU::TPRRegClass);

  unsigned SubOp = ST.hasAddNoCarry() ?
    PPU::V_SUB_U32_e32 : PPU::V_SUB_I32_e32;

  BuildMI(MBB, MII, DL, get(SubOp), TmpReg)
    .addImm(0)
    .addReg(Src.getReg());

  BuildMI(MBB, MII, DL, get(PPU::V_MAX_I32_e64), ResultReg)
    .addReg(Src.getReg())
    .addReg(TmpReg);

  MRI.replaceRegWith(Dest.getReg(), ResultReg);
  addUsersToMoveToVALUWorklist(ResultReg, MRI, Worklist);
  */
}

void PPUInstrInfo::lowerScalarXnor(SetVectorType &Worklist,
                                  MachineInstr &Inst) const {
    /*
  MachineBasicBlock &MBB = *Inst.getParent();
  MachineRegisterInfo &MRI = MBB.getParent()->getRegInfo();
  MachineBasicBlock::iterator MII = Inst;
  const DebugLoc &DL = Inst.getDebugLoc();

  MachineOperand &Dest = Inst.getOperand(0);
  MachineOperand &Src0 = Inst.getOperand(1);
  MachineOperand &Src1 = Inst.getOperand(2);

  if (ST.hasDLInsts()) {
    Register NewDest = MRI.createVirtualRegister(&PPU::TPRRegClass);
    legalizeGenericOperand(MBB, MII, &PPU::TPRRegClass, Src0, MRI, DL);
    legalizeGenericOperand(MBB, MII, &PPU::TPRRegClass, Src1, MRI, DL);

    BuildMI(MBB, MII, DL, get(PPU::V_XNOR_B32_e64), NewDest)
      .add(Src0)
      .add(Src1);

    MRI.replaceRegWith(Dest.getReg(), NewDest);
    addUsersToMoveToVALUWorklist(NewDest, MRI, Worklist);
  } else {
    // Using the identity !(x ^ y) == (!x ^ y) == (x ^ !y), we can
    // invert either source and then perform the XOR. If either source is a
    // scalar register, then we can leave the inversion on the scalar unit to
    // acheive a better distrubution of scalar and vector instructions.
    bool Src0IsSGPR = Src0.isReg() &&
                      RI.isSGPRClass(MRI.getRegClass(Src0.getReg()));
    bool Src1IsSGPR = Src1.isReg() &&
                      RI.isSGPRClass(MRI.getRegClass(Src1.getReg()));
    MachineInstr *Xor;
    Register Temp = MRI.createVirtualRegister(&PPU::SReg_32_XM0RegClass);
    Register NewDest = MRI.createVirtualRegister(&PPU::SReg_32_XM0RegClass);

    // Build a pair of scalar instructions and add them to the work list.
    // The next iteration over the work list will lower these to the vector
    // unit as necessary.
    if (Src0IsSGPR) {
      BuildMI(MBB, MII, DL, get(PPU::S_NOT_B32), Temp).add(Src0);
      Xor = BuildMI(MBB, MII, DL, get(PPU::S_XOR_B32), NewDest)
      .addReg(Temp)
      .add(Src1);
    } else if (Src1IsSGPR) {
      BuildMI(MBB, MII, DL, get(PPU::S_NOT_B32), Temp).add(Src1);
      Xor = BuildMI(MBB, MII, DL, get(PPU::S_XOR_B32), NewDest)
      .add(Src0)
      .addReg(Temp);
    } else {
      Xor = BuildMI(MBB, MII, DL, get(PPU::S_XOR_B32), Temp)
        .add(Src0)
        .add(Src1);
      MachineInstr *Not =
          BuildMI(MBB, MII, DL, get(PPU::S_NOT_B32), NewDest).addReg(Temp);
      Worklist.insert(Not);
    }

    MRI.replaceRegWith(Dest.getReg(), NewDest);

    Worklist.insert(Xor);

    addUsersToMoveToVALUWorklist(NewDest, MRI, Worklist);
  }
  */
}

void PPUInstrInfo::splitScalarNotBinop(SetVectorType &Worklist,
                                      MachineInstr &Inst,
                                      unsigned Opcode) const {
    /*
  MachineBasicBlock &MBB = *Inst.getParent();
  MachineRegisterInfo &MRI = MBB.getParent()->getRegInfo();
  MachineBasicBlock::iterator MII = Inst;
  const DebugLoc &DL = Inst.getDebugLoc();

  MachineOperand &Dest = Inst.getOperand(0);
  MachineOperand &Src0 = Inst.getOperand(1);
  MachineOperand &Src1 = Inst.getOperand(2);

  Register NewDest = MRI.createVirtualRegister(&PPU::SReg_32_XM0RegClass);
  Register Interm = MRI.createVirtualRegister(&PPU::SReg_32_XM0RegClass);

  MachineInstr &Op = *BuildMI(MBB, MII, DL, get(Opcode), Interm)
    .add(Src0)
    .add(Src1);

  MachineInstr &Not = *BuildMI(MBB, MII, DL, get(PPU::S_NOT_B32), NewDest)
    .addReg(Interm);

  Worklist.insert(&Op);
  Worklist.insert(&Not);

  MRI.replaceRegWith(Dest.getReg(), NewDest);
  addUsersToMoveToVALUWorklist(NewDest, MRI, Worklist);
  */
}

void PPUInstrInfo::splitScalarBinOpN2(SetVectorType& Worklist,
                                     MachineInstr &Inst,
                                     unsigned Opcode) const {
    /*
  MachineBasicBlock &MBB = *Inst.getParent();
  MachineRegisterInfo &MRI = MBB.getParent()->getRegInfo();
  MachineBasicBlock::iterator MII = Inst;
  const DebugLoc &DL = Inst.getDebugLoc();

  MachineOperand &Dest = Inst.getOperand(0);
  MachineOperand &Src0 = Inst.getOperand(1);
  MachineOperand &Src1 = Inst.getOperand(2);

  Register NewDest = MRI.createVirtualRegister(&PPU::SReg_32_XM0RegClass);
  Register Interm = MRI.createVirtualRegister(&PPU::SReg_32_XM0RegClass);

  MachineInstr &Not = *BuildMI(MBB, MII, DL, get(PPU::S_NOT_B32), Interm)
    .add(Src1);

  MachineInstr &Op = *BuildMI(MBB, MII, DL, get(Opcode), NewDest)
    .add(Src0)
    .addReg(Interm);

  Worklist.insert(&Not);
  Worklist.insert(&Op);

  MRI.replaceRegWith(Dest.getReg(), NewDest);
  addUsersToMoveToVALUWorklist(NewDest, MRI, Worklist);
  */
}
/*
void PPUInstrInfo::addUsersToMoveToVALUWorklist(
  unsigned DstReg,
  MachineRegisterInfo &MRI,
  SetVectorType &Worklist) const {
  for (MachineRegisterInfo::use_iterator I = MRI.use_begin(DstReg),
         E = MRI.use_end(); I != E;) {
    MachineInstr &UseMI = *I->getParent();

    unsigned OpNo = 0;

    switch (UseMI.getOpcode()) {
    case PPU::COPY:
    case PPU::WQM:
    case PPU::SOFT_WQM:
    case PPU::WWM:
    case PPU::REG_SEQUENCE:
    case PPU::PHI:
    case PPU::INSERT_SUBREG:
      break;
    default:
      OpNo = I.getOperandNo();
      break;
    }

    if (!RI.hasVectorRegisters(getOpRegClass(UseMI, OpNo))) {
      Worklist.insert(&UseMI);

      do {
        ++I;
      } while (I != E && I->getParent() == &UseMI);
    } else {
      ++I;
    }
  }
}
*/

void PPUInstrInfo::movePackToVALU(SetVectorType &Worklist,
                                 MachineRegisterInfo &MRI,
                                 MachineInstr &Inst) const {
    /*
  Register ResultReg = MRI.createVirtualRegister(&PPU::TPRRegClass);
  MachineBasicBlock *MBB = Inst.getParent();
  MachineOperand &Src0 = Inst.getOperand(1);
  MachineOperand &Src1 = Inst.getOperand(2);
  const DebugLoc &DL = Inst.getDebugLoc();

  switch (Inst.getOpcode()) {
  case PPU::S_PACK_LL_B32_B16: {
    Register ImmReg = MRI.createVirtualRegister(&PPU::TPRRegClass);
    Register TmpReg = MRI.createVirtualRegister(&PPU::TPRRegClass);

    // FIXME: Can do a lot better if we know the high bits of src0 or src1 are
    // 0.
    BuildMI(*MBB, Inst, DL, get(PPU::V_MOV_B32_e32), ImmReg)
      .addImm(0xffff);

    BuildMI(*MBB, Inst, DL, get(PPU::V_AND_B32_e64), TmpReg)
      .addReg(ImmReg, RegState::Kill)
      .add(Src0);

    BuildMI(*MBB, Inst, DL, get(PPU::V_LSHL_OR_B32), ResultReg)
      .add(Src1)
      .addImm(16)
      .addReg(TmpReg, RegState::Kill);
    break;
  }
  case PPU::S_PACK_LH_B32_B16: {
    Register ImmReg = MRI.createVirtualRegister(&PPU::TPRRegClass);
    BuildMI(*MBB, Inst, DL, get(PPU::V_MOV_B32_e32), ImmReg)
      .addImm(0xffff);
    BuildMI(*MBB, Inst, DL, get(PPU::V_BFI_B32), ResultReg)
      .addReg(ImmReg, RegState::Kill)
      .add(Src0)
      .add(Src1);
    break;
  }
  case PPU::S_PACK_HH_B32_B16: {
    Register ImmReg = MRI.createVirtualRegister(&PPU::TPRRegClass);
    Register TmpReg = MRI.createVirtualRegister(&PPU::TPRRegClass);
    BuildMI(*MBB, Inst, DL, get(PPU::V_LSHRREV_B32_e64), TmpReg)
      .addImm(16)
      .add(Src0);
    BuildMI(*MBB, Inst, DL, get(PPU::V_MOV_B32_e32), ImmReg)
      .addImm(0xffff0000);
    BuildMI(*MBB, Inst, DL, get(PPU::V_AND_OR_B32), ResultReg)
      .add(Src1)
      .addReg(ImmReg, RegState::Kill)
      .addReg(TmpReg, RegState::Kill);
    break;
  }
  default:
    llvm_unreachable("unhandled s_pack_* instruction");
  }

  MachineOperand &Dest = Inst.getOperand(0);
  MRI.replaceRegWith(Dest.getReg(), ResultReg);
  addUsersToMoveToVALUWorklist(ResultReg, MRI, Worklist);
  */
}

void PPUInstrInfo::addSCCDefUsersToVALUWorklist(MachineOperand &Op,
                                               MachineInstr &SCCDefInst,
                                               SetVectorType &Worklist) const {
    /*
  // Ensure that def inst defines SCC, which is still live.
  assert(Op.isReg() && Op.getReg() == PPU::SCC && Op.isDef() &&
         !Op.isDead() && Op.getParent() == &SCCDefInst);
  // This assumes that all the users of SCC are in the same block
  // as the SCC def.
  for (MachineInstr &MI : // Skip the def inst itself.
       make_range(std::next(MachineBasicBlock::iterator(SCCDefInst)),
                  SCCDefInst.getParent()->end())) {
    // Check if SCC is used first.
    if (MI.findRegisterUseOperandIdx(PPU::SCC, false, &RI) != -1)
      Worklist.insert(&MI);
    // Exit if we find another SCC def.
    if (MI.findRegisterDefOperandIdx(PPU::SCC, false, false, &RI) != -1)
      return;
  }
  */
}

const TargetRegisterClass *PPUInstrInfo::getDestEquivalentVGPRClass(
  const MachineInstr &Inst) const {
    /*
  const TargetRegisterClass *NewDstRC = getOpRegClass(Inst, 0);

  switch (Inst.getOpcode()) {
  // For target instructions, getOpRegClass just returns the virtual register
  // class associated with the operand, so we need to find an equivalent VGPR
  // register class in order to move the instruction to the VALU.
  case PPU::COPY:
  case PPU::PHI:
  case PPU::REG_SEQUENCE:
  case PPU::INSERT_SUBREG:
  case PPU::WQM:
  case PPU::SOFT_WQM:
  case PPU::WWM: {
    const TargetRegisterClass *SrcRC = getOpRegClass(Inst, 1);
    if (RI.hasVGPRs(NewDstRC))
        return nullptr;

    NewDstRC = RI.getEquivalentVGPRClass(NewDstRC);
    if (!NewDstRC)
        return nullptr;

    return NewDstRC;
  }
  default:
    return NewDstRC;
  }
  */
}

// Find the one SGPR operand we are allowed to use.
unsigned PPUInstrInfo::findUsedSGPR(const MachineInstr &MI,
                                   int OpIndices[3]) const {
  const MCInstrDesc &Desc = MI.getDesc();

  // Find the one SGPR operand we are allowed to use.
  //
  // First we need to consider the instruction's operand requirements before
  // legalizing. Some operands are required to be SGPRs, such as implicit uses
  // of VCC, but we are still bound by the constant bus requirement to only use
  // one.
  //
  // If the operand's class is an SGPR, we can never move it.

  unsigned SGPRReg = findImplicitSGPRRead(MI);
  if (SGPRReg != PPU::NoRegister)
    return SGPRReg;

  unsigned UsedSGPRs[3] = { PPU::NoRegister };
  const MachineRegisterInfo &MRI = MI.getParent()->getParent()->getRegInfo();

  for (unsigned i = 0; i < 3; ++i) {
    int Idx = OpIndices[i];
    if (Idx == -1)
      break;

    const MachineOperand &MO = MI.getOperand(Idx);
    if (!MO.isReg())
      continue;

    // Is this operand statically required to be an SGPR based on the operand
    // constraints?
    const TargetRegisterClass *OpRC = RI.getRegClass(Desc.OpInfo[Idx].RegClass);
    bool IsRequiredSGPR = RI.isSGPRClass(OpRC);
    if (IsRequiredSGPR)
      return MO.getReg();

    // If this could be a VGPR or an SGPR, Check the dynamic register class.
    Register Reg = MO.getReg();
    const TargetRegisterClass *RegRC = MRI.getRegClass(Reg);
    if (RI.isSGPRClass(RegRC))
      UsedSGPRs[i] = Reg;
  }

  // We don't have a required SGPR operand, so we have a bit more freedom in
  // selecting operands to move.

  // Try to select the most used SGPR. If an SGPR is equal to one of the
  // others, we choose that.
  //
  // e.g.
  // V_FMA_F32 v0, s0, s0, s0 -> No moves
  // V_FMA_F32 v0, s0, s1, s0 -> Move s1

  // TODO: If some of the operands are 64-bit SGPRs and some 32, we should
  // prefer those.

  if (UsedSGPRs[0] != PPU::NoRegister) {
    if (UsedSGPRs[0] == UsedSGPRs[1] || UsedSGPRs[0] == UsedSGPRs[2])
      SGPRReg = UsedSGPRs[0];
  }

  if (SGPRReg == PPU::NoRegister && UsedSGPRs[1] != PPU::NoRegister) {
    if (UsedSGPRs[1] == UsedSGPRs[2])
      SGPRReg = UsedSGPRs[1];
  }

  return SGPRReg;
}

MachineOperand *PPUInstrInfo::getNamedOperand(MachineInstr &MI,
                                             unsigned OperandName) const {
  int Idx = PPU::getNamedOperandIdx(MI.getOpcode(), OperandName);
  if (Idx == -1)
    return nullptr;

  return &MI.getOperand(Idx);
}

uint64_t PPUInstrInfo::getDefaultRsrcDataFormat() const {
/*
  uint64_t RsrcDataFormat = PPU::RSRC_DATA_FORMAT;
  if (ST.isAmdHsaOS()) {
    // Set ATC = 1. GFX9 doesn't have this bit.
    if (ST.getGeneration() <= PPUSubtarget::VOLCANIC_ISLANDS)
      RsrcDataFormat |= (1ULL << 56);

    // Set MTYPE = 2 (MTYPE_UC = uncached). GFX9 doesn't have this.
    // BTW, it disables TC L2 and therefore decreases performance.
    if (ST.getGeneration() == PPUSubtarget::VOLCANIC_ISLANDS)
      RsrcDataFormat |= (2ULL << 59);
  }

  return RsrcDataFormat;
  */
    return 0;
}

uint64_t PPUInstrInfo::getScratchRsrcWords23() const {
    /*
  uint64_t Rsrc23 = getDefaultRsrcDataFormat() |
                    PPU::RSRC_TID_ENABLE |
                    0xffffffff; // Size;

  // GFX9 doesn't have ELEMENT_SIZE.
  if (ST.getGeneration() <= PPUSubtarget::VOLCANIC_ISLANDS) {
    uint64_t EltSizeValue = Log2_32(ST.getMaxPrivateElementSize()) - 1;
    Rsrc23 |= EltSizeValue << PPU::RSRC_ELEMENT_SIZE_SHIFT;
  }

  // IndexStride = 64 / 32.
  uint64_t IndexStride = ST.getWavefrontSize() == 64 ? 3 : 2;
  Rsrc23 |= IndexStride << PPU::RSRC_INDEX_STRIDE_SHIFT;

  // If TID_ENABLE is set, DATA_FORMAT specifies stride bits [14:17].
  // Clear them unless we want a huge stride.
  if (ST.getGeneration() >= PPUSubtarget::VOLCANIC_ISLANDS &&
      ST.getGeneration() <= PPUSubtarget::GFX9)
    Rsrc23 &= ~PPU::RSRC_DATA_FORMAT;

  return Rsrc23;
  */
    return 0;
}

bool PPUInstrInfo::isLowLatencyInstruction(const MachineInstr &MI) const {
  unsigned Opc = MI.getOpcode();

  return isSMRD(Opc);
}

bool PPUInstrInfo::isHighLatencyInstruction(const MachineInstr &MI) const {
  unsigned Opc = MI.getOpcode();

  return isMUBUF(Opc) || isMTBUF(Opc) || isMIMG(Opc);
}

unsigned PPUInstrInfo::isStackAccess(const MachineInstr &MI,
                                    int &FrameIndex) const {
    /*
  const MachineOperand *Addr = getNamedOperand(MI, PPU::OpName::vaddr);
  if (!Addr || !Addr->isFI())
    return PPU::NoRegister;

  assert(!MI.memoperands_empty() &&
         (*MI.memoperands_begin())->getAddrSpace() == AMDGPUAS::PRIVATE_ADDRESS);

  FrameIndex = Addr->getIndex();
  return getNamedOperand(MI, PPU::OpName::vdata)->getReg();
  */
    return 0;
}

unsigned PPUInstrInfo::isSGPRStackAccess(const MachineInstr &MI,
                                        int &FrameIndex) const {
    /*
  const MachineOperand *Addr = getNamedOperand(MI, PPU::OpName::addr);
  assert(Addr && Addr->isFI());
  FrameIndex = Addr->getIndex();
  return getNamedOperand(MI, PPU::OpName::data)->getReg();
  */
    return 0;
}

unsigned PPUInstrInfo::isLoadFromStackSlot(const MachineInstr &MI,
                                          int &FrameIndex) const {
  if (!MI.mayLoad())
    return PPU::NoRegister;

  if (isMUBUF(MI) || isVGPRSpill(MI))
    return isStackAccess(MI, FrameIndex);

  if (isSGPRSpill(MI))
    return isSGPRStackAccess(MI, FrameIndex);

  return PPU::NoRegister;
}

unsigned PPUInstrInfo::isStoreToStackSlot(const MachineInstr &MI,
                                         int &FrameIndex) const {
  if (!MI.mayStore())
    return PPU::NoRegister;

  if (isMUBUF(MI) || isVGPRSpill(MI))
    return isStackAccess(MI, FrameIndex);

  if (isSGPRSpill(MI))
    return isSGPRStackAccess(MI, FrameIndex);

  return PPU::NoRegister;
}

unsigned PPUInstrInfo::getInstBundleSize(const MachineInstr &MI) const {
  unsigned Size = 0;
  MachineBasicBlock::const_instr_iterator I = MI.getIterator();
  MachineBasicBlock::const_instr_iterator E = MI.getParent()->instr_end();
  while (++I != E && I->isInsideBundle()) {
    assert(!I->isBundle() && "No nested bundle!");
    Size += getInstSizeInBytes(*I);
  }

  return Size;
}

unsigned PPUInstrInfo::getInstSizeInBytes(const MachineInstr &MI) const {
    /*
  unsigned Opc = MI.getOpcode();
  const MCInstrDesc &Desc = getMCOpcodeFromPseudo(Opc);
  unsigned DescSize = Desc.getSize();

  // If we have a definitive size, we can use it. Otherwise we need to inspect
  // the operands to know the size.
  if (isFixedSize(MI))
    return DescSize;

  // 4-byte instructions may have a 32-bit literal encoded after them. Check
  // operands that coud ever be literals.
  if (isVALU(MI) || isSALU(MI)) {
    int Src0Idx = PPU::getNamedOperandIdx(Opc, PPU::OpName::src0);
    if (Src0Idx == -1)
      return DescSize; // No operands.

    if (isLiteralConstantLike(MI.getOperand(Src0Idx), Desc.OpInfo[Src0Idx]))
      return isVOP3(MI) ? 12 : (DescSize + 4);

    int Src1Idx = PPU::getNamedOperandIdx(Opc, PPU::OpName::src1);
    if (Src1Idx == -1)
      return DescSize;

    if (isLiteralConstantLike(MI.getOperand(Src1Idx), Desc.OpInfo[Src1Idx]))
      return isVOP3(MI) ? 12 : (DescSize + 4);

    int Src2Idx = PPU::getNamedOperandIdx(Opc, PPU::OpName::src2);
    if (Src2Idx == -1)
      return DescSize;

    if (isLiteralConstantLike(MI.getOperand(Src2Idx), Desc.OpInfo[Src2Idx]))
      return isVOP3(MI) ? 12 : (DescSize + 4);

    return DescSize;
  }

  // Check whether we have extra NSA words.
  if (isMIMG(MI)) {
    int VAddr0Idx = PPU::getNamedOperandIdx(Opc, PPU::OpName::vaddr0);
    if (VAddr0Idx < 0)
      return 8;

    int RSrcIdx = PPU::getNamedOperandIdx(Opc, PPU::OpName::srsrc);
    return 8 + 4 * ((RSrcIdx - VAddr0Idx + 2) / 4);
  }

  switch (Opc) {
  case TargetOpcode::IMPLICIT_DEF:
  case TargetOpcode::KILL:
  case TargetOpcode::DBG_VALUE:
  case TargetOpcode::EH_LABEL:
    return 0;
  case TargetOpcode::BUNDLE:
    return getInstBundleSize(MI);
  case TargetOpcode::INLINEASM:
  case TargetOpcode::INLINEASM_BR: {
    const MachineFunction *MF = MI.getParent()->getParent();
    const char *AsmStr = MI.getOperand(0).getSymbolName();
    return getInlineAsmLength(AsmStr, *MF->getTarget().getMCAsmInfo(),
                              &MF->getSubtarget());
  }
  default:
    return DescSize;
  }
  */
    return 0;
}

bool PPUInstrInfo::mayAccessFlatAddressSpace(const MachineInstr &MI) const {
  if (!isFLAT(MI))
    return false;

  if (MI.memoperands_empty())
    return true;

  for (const MachineMemOperand *MMO : MI.memoperands()) {
    if (MMO->getAddrSpace() == AMDGPUAS::FLAT_ADDRESS)
      return true;
  }
  return false;
}

bool PPUInstrInfo::isNonUniformBranchInstr(MachineInstr &Branch) const {
    /* FIXME
  return Branch.getOpcode() == PPU::SI_NON_UNIFORM_BRCOND_PSEUDO;
  */
}

void PPUInstrInfo::convertNonUniformIfRegion(MachineBasicBlock *IfEntry,
                                            MachineBasicBlock *IfEnd) const {
    /*
  MachineBasicBlock::iterator TI = IfEntry->getFirstTerminator();
  assert(TI != IfEntry->end());

  MachineInstr *Branch = &(*TI);
  MachineFunction *MF = IfEntry->getParent();
  MachineRegisterInfo &MRI = IfEntry->getParent()->getRegInfo();

  if (Branch->getOpcode() == PPU::SI_NON_UNIFORM_BRCOND_PSEUDO) {
    Register DstReg = MRI.createVirtualRegister(RI.getBoolRC());
    MachineInstr *SIIF =
        BuildMI(*MF, Branch->getDebugLoc(), get(PPU::SI_IF), DstReg)
            .add(Branch->getOperand(0))
            .add(Branch->getOperand(1));
    MachineInstr *SIEND =
        BuildMI(*MF, Branch->getDebugLoc(), get(PPU::SI_END_CF))
            .addReg(DstReg);

    IfEntry->erase(TI);
    IfEntry->insert(IfEntry->end(), SIIF);
    IfEnd->insert(IfEnd->getFirstNonPHI(), SIEND);
  }
  */
}

void PPUInstrInfo::convertNonUniformLoopRegion(
    MachineBasicBlock *LoopEntry, MachineBasicBlock *LoopEnd) const {
    /*
  MachineBasicBlock::iterator TI = LoopEnd->getFirstTerminator();
  // We expect 2 terminators, one conditional and one unconditional.
  assert(TI != LoopEnd->end());

  MachineInstr *Branch = &(*TI);
  MachineFunction *MF = LoopEnd->getParent();
  MachineRegisterInfo &MRI = LoopEnd->getParent()->getRegInfo();

  if (Branch->getOpcode() == PPU::SI_NON_UNIFORM_BRCOND_PSEUDO) {

    Register DstReg = MRI.createVirtualRegister(RI.getBoolRC());
    Register BackEdgeReg = MRI.createVirtualRegister(RI.getBoolRC());
    MachineInstrBuilder HeaderPHIBuilder =
        BuildMI(*(MF), Branch->getDebugLoc(), get(TargetOpcode::PHI), DstReg);
    for (MachineBasicBlock::pred_iterator PI = LoopEntry->pred_begin(),
                                          E = LoopEntry->pred_end();
         PI != E; ++PI) {
      if (*PI == LoopEnd) {
        HeaderPHIBuilder.addReg(BackEdgeReg);
      } else {
        MachineBasicBlock *PMBB = *PI;
        Register ZeroReg = MRI.createVirtualRegister(RI.getBoolRC());
        materializeImmediate(*PMBB, PMBB->getFirstTerminator(), DebugLoc(),
                             ZeroReg, 0);
        HeaderPHIBuilder.addReg(ZeroReg);
      }
      HeaderPHIBuilder.addMBB(*PI);
    }
    MachineInstr *HeaderPhi = HeaderPHIBuilder;
    MachineInstr *SIIFBREAK = BuildMI(*(MF), Branch->getDebugLoc(),
                                      get(PPU::SI_IF_BREAK), BackEdgeReg)
                                  .addReg(DstReg)
                                  .add(Branch->getOperand(0));
    MachineInstr *SILOOP =
        BuildMI(*(MF), Branch->getDebugLoc(), get(PPU::SI_LOOP))
            .addReg(BackEdgeReg)
            .addMBB(LoopEntry);

    LoopEntry->insert(LoopEntry->begin(), HeaderPhi);
    LoopEnd->erase(TI);
    LoopEnd->insert(LoopEnd->end(), SIIFBREAK);
    LoopEnd->insert(LoopEnd->end(), SILOOP);
  }
  */
}
/*
ArrayRef<std::pair<int, const char *>>
PPUInstrInfo::getSerializableTargetIndices() const {
  static const std::pair<int, const char *> TargetIndices[] = {
      {PPU::TI_CONSTDATA_START, "amdgpu-constdata-start"},
      {PPU::TI_SCRATCH_RSRC_DWORD0, "amdgpu-scratch-rsrc-dword0"},
      {PPU::TI_SCRATCH_RSRC_DWORD1, "amdgpu-scratch-rsrc-dword1"},
      {PPU::TI_SCRATCH_RSRC_DWORD2, "amdgpu-scratch-rsrc-dword2"},
      {PPU::TI_SCRATCH_RSRC_DWORD3, "amdgpu-scratch-rsrc-dword3"}};
  return makeArrayRef(TargetIndices);
}
*/

/// This is used by the post-RA scheduler (SchedulePostRAList.cpp).  The
/// post-RA version of misched uses CreateTargetMIHazardRecognizer.
/*
ScheduleHazardRecognizer *
PPUInstrInfo::CreateTargetPostRAHazardRecognizer(const InstrItineraryData *II,
                                            const ScheduleDAG *DAG) const {
  return new PPUHazardRecognizer(DAG->MF);
}

/// This is the hazard recognizer used at -O0 by the PostRAHazardRecognizer
/// pass.
ScheduleHazardRecognizer *
PPUInstrInfo::CreateTargetPostRAHazardRecognizer(const MachineFunction &MF) const {
  return new PPUHazardRecognizer(MF);
}
*/

std::pair<unsigned, unsigned>
PPUInstrInfo::decomposeMachineOperandsTargetFlags(unsigned TF) const {
  return std::make_pair(TF & MO_MASK, TF & ~MO_MASK);
}
/*
ArrayRef<std::pair<unsigned, const char *>>
PPUInstrInfo::getSerializableDirectMachineOperandTargetFlags() const {
  static const std::pair<unsigned, const char *> TargetFlags[] = {
    { MO_GOTPCREL, "amdgpu-gotprel" },
    { MO_GOTPCREL32_LO, "amdgpu-gotprel32-lo" },
    { MO_GOTPCREL32_HI, "amdgpu-gotprel32-hi" },
    { MO_REL32_LO, "amdgpu-rel32-lo" },
    { MO_REL32_HI, "amdgpu-rel32-hi" },
    { MO_ABS32_LO, "amdgpu-abs32-lo" },
    { MO_ABS32_HI, "amdgpu-abs32-hi" },
  };

  return makeArrayRef(TargetFlags);
}
*/

bool PPUInstrInfo::isBasicBlockPrologue(const MachineInstr &MI) const {
  return !MI.isTerminator() && MI.getOpcode() != PPU::COPY &&
         MI.modifiesRegister(PPU::TMSK, &RI);
}

MachineInstrBuilder
PPUInstrInfo::getAddNoCarry(MachineBasicBlock &MBB,
                           MachineBasicBlock::iterator I,
                           const DebugLoc &DL,
                           unsigned DestReg) const {
    /*
  if (ST.hasAddNoCarry())
    return BuildMI(MBB, I, DL, get(PPU::VADD), DestReg);
    */

  MachineRegisterInfo &MRI = MBB.getParent()->getRegInfo();
  Register UnusedCarry = MRI.createVirtualRegister(RI.getBoolRC());
  MRI.setRegAllocationHint(UnusedCarry, 0, RI.getVCC());

  return BuildMI(MBB, I, DL, get(PPU::VADD), DestReg)
           .addReg(UnusedCarry, RegState::Define | RegState::Dead);
}

MachineInstrBuilder PPUInstrInfo::getAddNoCarry(MachineBasicBlock &MBB,
                                               MachineBasicBlock::iterator I,
                                               const DebugLoc &DL,
                                               Register DestReg,
                                               RegScavenger &RS) const {
    /*
  if (ST.hasAddNoCarry())
    return BuildMI(MBB, I, DL, get(PPU::VADD), DestReg);
    */

  Register UnusedCarry = RS.scavengeRegister(RI.getBoolRC(), I, 0, false);
  // TODO: Users need to deal with this.
  if (!UnusedCarry.isValid())
    report_fatal_error("failed to scavenge unused carry-out SGPR");

  return BuildMI(MBB, I, DL, get(PPU::VADD), DestReg)
           .addReg(UnusedCarry, RegState::Define | RegState::Dead);
}

bool PPUInstrInfo::isKillTerminator(unsigned Opcode) {
  switch (Opcode) {
      /*
  case PPU::SI_KILL_F32_COND_IMM_TERMINATOR:
  case PPU::SI_KILL_I1_TERMINATOR:
    return true;
    */
  default:
    return false;
  }
}

const MCInstrDesc &PPUInstrInfo::getKillTerminatorFromPseudo(unsigned Opcode) const {
  switch (Opcode) {
      /*
  case PPU::SI_KILL_F32_COND_IMM_PSEUDO:
    return get(PPU::SI_KILL_F32_COND_IMM_TERMINATOR);
  case PPU::SI_KILL_I1_PSEUDO:
    return get(PPU::SI_KILL_I1_TERMINATOR);
    */
  default:
    llvm_unreachable("invalid opcode, expected SI_KILL_*_PSEUDO");
  }
}

void PPUInstrInfo::fixImplicitOperands(MachineInstr &MI) const {
    /*
  MachineBasicBlock *MBB = MI.getParent();
  MachineFunction *MF = MBB->getParent();
  const PPUSubtarget &ST = MF->getSubtarget<PPUSubtarget>();

  if (!ST.isWave32())
    return;

  for (auto &Op : MI.implicit_operands()) {
    if (Op.isReg() && Op.getReg() == PPU::VCC)
      Op.setReg(PPU::VCC_LO);
  }
  */
}

bool PPUInstrInfo::isBufferSMRD(const MachineInstr &MI) const {
    /*
  if (!isSMRD(MI))
    return false;

  // Check that it is using a buffer resource.
  int Idx = PPU::getNamedOperandIdx(MI.getOpcode(), PPU::OpName::sbase);
  if (Idx == -1) // e.g. s_memtime
    return false;

  const auto RCID = MI.getDesc().OpInfo[Idx].RegClass;
  return RCID == PPU::SReg_128RegClassID;
  */
    return false;
}

bool PPUInstrInfo::isLegalFLATOffset(int64_t Offset, unsigned AddrSpace,
                                    bool Signed) const {
    /*
  // TODO: Should 0 be special cased?
  if (!ST.hasFlatInstOffsets())
    return false;

  if (ST.hasFlatSegmentOffsetBug() && AddrSpace == AMDGPUAS::FLAT_ADDRESS)
    return false;

  return (Signed && isInt<13>(Offset)) ||
         (!Signed && isUInt<12>(Offset));
         */
}


// This must be kept in sync with the SIEncodingFamily class in PPUInstrInfo.td
enum PPUEncodingFamily {
  PPU = 0
};

static PPUEncodingFamily subtargetEncodingFamily(const PPUSubtarget &ST) {
  switch (ST.getGeneration()) {
  default:
    break;
  case PPUSubtarget::PPU:
    return PPUEncodingFamily::PPU;
  }
  llvm_unreachable("Unknown subtarget generation!");
}

int PPUInstrInfo::pseudoToMCOpcode(int Opcode) const {
    /*
  SIEncodingFamily Gen = subtargetEncodingFamily(ST);

  if ((get(Opcode).TSFlags & SIInstrFlags::renamedInGFX9) != 0 &&
    ST.getGeneration() == PPUSubtarget::GFX9)
    Gen = SIEncodingFamily::GFX9;

  // Adjust the encoding family to GFX80 for D16 buffer instructions when the
  // subtarget has UnpackedD16VMem feature.
  // TODO: remove this when we discard GFX80 encoding.
  if (ST.hasUnpackedD16VMem() && (get(Opcode).TSFlags & SIInstrFlags::D16Buf))
    Gen = SIEncodingFamily::GFX80;

  if (get(Opcode).TSFlags & SIInstrFlags::SDWA) {
    switch (ST.getGeneration()) {
    default:
      Gen = SIEncodingFamily::SDWA;
      break;
    case PPUSubtarget::GFX9:
      Gen = SIEncodingFamily::SDWA9;
      break;
    case PPUSubtarget::GFX10:
      Gen = SIEncodingFamily::SDWA10;
      break;
    }
  }

  int MCOp = PPU::getMCOpcode(Opcode, Gen);

  // -1 means that Opcode is already a native instruction.
  if (MCOp == -1)
    return Opcode;

  // (uint16_t)-1 means that Opcode is a pseudo instruction that has
  // no encoding in the given subtarget generation.
  if (MCOp == (uint16_t)-1)
    return -1;

  return MCOp;
  */
    return 0;
}

static
TargetInstrInfo::RegSubRegPair getRegOrUndef(const MachineOperand &RegOpnd) {
  assert(RegOpnd.isReg());
  return RegOpnd.isUndef() ? TargetInstrInfo::RegSubRegPair() :
                             llvm::PPU::getRegSubRegPair(RegOpnd);
}

TargetInstrInfo::RegSubRegPair
llvm::PPU::getRegSequenceSubReg(MachineInstr &MI, unsigned SubReg) {
  assert(MI.isRegSequence());
  for (unsigned I = 0, E = (MI.getNumOperands() - 1)/ 2; I < E; ++I)
    if (MI.getOperand(1 + 2 * I + 1).getImm() == SubReg) {
      auto &RegOp = MI.getOperand(1 + 2 * I);
      return getRegOrUndef(RegOp);
    }
  return TargetInstrInfo::RegSubRegPair();
}

// Try to find the definition of reg:subreg in subreg-manipulation pseudos
// Following a subreg of reg:subreg isn't supported
static bool followSubRegDef(MachineInstr &MI,
                            TargetInstrInfo::RegSubRegPair &RSR) {
    /*
  if (!RSR.SubReg)
    return false;
  switch (MI.getOpcode()) {
  default: break;
  case PPU::REG_SEQUENCE:
    RSR = getRegSequenceSubReg(MI, RSR.SubReg);
    return true;
  // EXTRACT_SUBREG ins't supported as this would follow a subreg of subreg
  case PPU::INSERT_SUBREG:
    if (RSR.SubReg == (unsigned)MI.getOperand(3).getImm())
      // inserted the subreg we're looking for
      RSR = getRegOrUndef(MI.getOperand(2));
    else { // the subreg in the rest of the reg
      auto R1 = getRegOrUndef(MI.getOperand(1));
      if (R1.SubReg) // subreg of subreg isn't supported
        return false;
      RSR.Reg = R1.Reg;
    }
    return true;
  }
  */
  return false;
}

MachineInstr *llvm::PPU::getVRegSubRegDef(const TargetInstrInfo::RegSubRegPair &P,
                                     MachineRegisterInfo &MRI) {
  assert(MRI.isSSA());
  if (!Register::isVirtualRegister(P.Reg))
    return nullptr;

  auto RSR = P;
  auto *DefInst = MRI.getVRegDef(RSR.Reg);
  while (auto *MI = DefInst) {
    DefInst = nullptr;
    switch (MI->getOpcode()) {
    case PPU::COPY:
    case PPU::VMOV: {
      auto &Op1 = MI->getOperand(1);
      if (Op1.isReg() && Register::isVirtualRegister(Op1.getReg())) {
        if (Op1.isUndef())
          return nullptr;
        RSR = getRegSubRegPair(Op1);
        DefInst = MRI.getVRegDef(RSR.Reg);
      }
      break;
    }
    default:
      if (followSubRegDef(*MI, RSR)) {
        if (!RSR.Reg)
          return nullptr;
        DefInst = MRI.getVRegDef(RSR.Reg);
      }
    }
    if (!DefInst)
      return MI;
  }
  return nullptr;
}

bool llvm::PPU::execMayBeModifiedBeforeUse(const MachineRegisterInfo &MRI,
                                      Register VReg,
                                      const MachineInstr &DefMI,
                                      const MachineInstr &UseMI) {
  assert(MRI.isSSA() && "Must be run on SSA");

  auto *TRI = MRI.getTargetRegisterInfo();
  auto *DefBB = DefMI.getParent();

  // Don't bother searching between blocks, although it is possible this block
  // doesn't modify exec.
  if (UseMI.getParent() != DefBB)
    return true;

  const int MaxInstScan = 20;
  int NumInst = 0;

  // Stop scan at the use.
  auto E = UseMI.getIterator();
  for (auto I = std::next(DefMI.getIterator()); I != E; ++I) {
    if (I->isDebugInstr())
      continue;

    if (++NumInst > MaxInstScan)
      return true;

    if (I->modifiesRegister(PPU::TMSK, TRI))
      return true;
  }

  return false;
}

bool llvm::PPU::execMayBeModifiedBeforeAnyUse(const MachineRegisterInfo &MRI,
                                         Register VReg,
                                         const MachineInstr &DefMI) {
  assert(MRI.isSSA() && "Must be run on SSA");
  /*

  auto *TRI = MRI.getTargetRegisterInfo();
  auto *DefBB = DefMI.getParent();

  const int MaxUseInstScan = 10;
  int NumUseInst = 0;

  for (auto &UseInst : MRI.use_nodbg_instructions(VReg)) {
    // Don't bother searching between blocks, although it is possible this block
    // doesn't modify exec.
    if (UseInst.getParent() != DefBB)
      return true;

    if (++NumUseInst > MaxUseInstScan)
      return true;
  }

  const int MaxInstScan = 20;
  int NumInst = 0;

  // Stop scan when we have seen all the uses.
  for (auto I = std::next(DefMI.getIterator()); ; ++I) {
    if (I->isDebugInstr())
      continue;

    if (++NumInst > MaxInstScan)
      return true;

    if (I->readsRegister(VReg))
      if (--NumUseInst == 0)
        return false;

    if (I->modifiesRegister(PPU::EXEC, TRI))
      return true;
  }
  */
}
