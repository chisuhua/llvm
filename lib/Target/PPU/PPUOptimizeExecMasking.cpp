//===-- PPUOptimizeExecMasking.cpp -----------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
///
//===----------------------------------------------------------------------===//

#include "PPU.h"
#include "PPUSubtarget.h"
#include "PPUInstrInfo.h"
#include "MCTargetDesc/PPUMCTargetDesc.h"
#include "llvm/ADT/SmallSet.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/Support/Debug.h"

using namespace llvm;

#define DEBUG_TYPE "ppu-optimize-exec-masking"

namespace {

class PPUOptimizeExecMasking : public MachineFunctionPass {
public:
  static char ID;

public:
  PPUOptimizeExecMasking() : MachineFunctionPass(ID) {
    initializePPUOptimizeExecMaskingPass(*PassRegistry::getPassRegistry());
  }

  bool runOnMachineFunction(MachineFunction &MF) override;

  StringRef getPassName() const override {
    return "PPU optimize exec mask operations";
  }

  void getAnalysisUsage(AnalysisUsage &AU) const override {
    AU.setPreservesCFG();
    MachineFunctionPass::getAnalysisUsage(AU);
  }
};

} // End anonymous namespace.

INITIALIZE_PASS_BEGIN(PPUOptimizeExecMasking, DEBUG_TYPE,
                      "PPU optimize exec mask operations", false, false)
INITIALIZE_PASS_DEPENDENCY(LiveIntervals)
INITIALIZE_PASS_END(PPUOptimizeExecMasking, DEBUG_TYPE,
                    "PPU optimize exec mask operations", false, false)

char PPUOptimizeExecMasking::ID = 0;

char &llvm::PPUOptimizeExecMaskingID = PPUOptimizeExecMasking::ID;

/// If \p MI is a copy from exec, return the register copied to.
static unsigned isCopyFromExec(const MachineInstr &MI, const PPUSubtarget &ST) {
      /*
  switch (MI.getOpcode()) {
  case PPU::COPY:
  case PPU::S_MOV_B64:
  case PPU::S_MOV_B64_term:
  case PPU::S_MOV_B32:
  case PPU::S_MOV_B32_term: {
    const MachineOperand &Src = MI.getOperand(1);
    /* FIXME
    if (Src.isReg() && Src.getReg() == PPU::TMSK)
      return MI.getOperand(0).getReg();
  }
  }
      */

  return PPU::NoRegister;
}

/// If \p MI is a copy to exec, return the register copied from.
static unsigned isCopyToExec(const MachineInstr &MI, const PPUSubtarget &ST) {
    /* FIXME
  switch (MI.getOpcode()) {
  case PPU::COPY:
  case PPU::S_MOV_B64:
  case PPU::S_MOV_B32: {
    const MachineOperand &Dst = MI.getOperand(0);
    if (Dst.isReg() && Dst.getReg() == PPU::TMSK && MI.getOperand(1).isReg())
      return MI.getOperand(1).getReg();
    break;
  }
  case PPU::S_MOV_B64_term:
  case PPU::S_MOV_B32_term:
    llvm_unreachable("should have been replaced");
  }
  */

  return PPU::NoRegister;
}

/// If \p MI is a logical operation on an exec value,
/// return the register copied to.
static unsigned isLogicalOpOnExec(const MachineInstr &MI) {
    /* FIXME
  switch (MI.getOpcode()) {
  case PPU::S_AND_B64:
  case PPU::S_OR_B64:
  case PPU::S_XOR_B64:
  case PPU::S_ANDN2_B64:
  case PPU::S_ORN2_B64:
  case PPU::S_NAND_B64:
  case PPU::S_NOR_B64:
  case PPU::S_XNOR_B64: {
    const MachineOperand &Src1 = MI.getOperand(1);
    if (Src1.isReg() && Src1.getReg() == PPU::TMSK)
      return MI.getOperand(0).getReg();
    const MachineOperand &Src2 = MI.getOperand(2);
    if (Src2.isReg() && Src2.getReg() == PPU::TMSK)
      return MI.getOperand(0).getReg();
    break;
  }
  case PPU::S_AND_B32:
  case PPU::S_OR_B32:
  case PPU::S_XOR_B32:
  case PPU::S_ANDN2_B32:
  case PPU::S_ORN2_B32:
  case PPU::S_NAND_B32:
  case PPU::S_NOR_B32:
  case PPU::S_XNOR_B32: {
    const MachineOperand &Src1 = MI.getOperand(1);
    if (Src1.isReg() && Src1.getReg() == PPU::TMASK)
      return MI.getOperand(0).getReg();
    const MachineOperand &Src2 = MI.getOperand(2);
    if (Src2.isReg() && Src2.getReg() == PPU::TMASK)
      return MI.getOperand(0).getReg();
    break;
  }
  }
  */

  return PPU::NoRegister;
}

static unsigned getSaveTmskOp(unsigned Opc) {
    /* FIXME
  switch (Opc) {
  case PPU::S_AND_B64:
    return PPU::S_AND_SAVEEXEC_B64;
  case PPU::S_OR_B64:
    return PPU::S_OR_SAVEEXEC_B64;
  case PPU::S_XOR_B64:
    return PPU::S_XOR_SAVEEXEC_B64;
  case PPU::S_ANDN2_B64:
    return PPU::S_ANDN2_SAVEEXEC_B64;
  case PPU::S_ORN2_B64:
    return PPU::S_ORN2_SAVEEXEC_B64;
  case PPU::S_NAND_B64:
    return PPU::S_NAND_SAVEEXEC_B64;
  case PPU::S_NOR_B64:
    return PPU::S_NOR_SAVEEXEC_B64;
  case PPU::S_XNOR_B64:
    return PPU::S_XNOR_SAVEEXEC_B64;
  case PPU::S_AND_B32:
    return PPU::S_AND_SAVEEXEC_B32;
  case PPU::S_OR_B32:
    return PPU::S_OR_SAVEEXEC_B32;
  case PPU::S_XOR_B32:
    return PPU::S_XOR_SAVEEXEC_B32;
  case PPU::S_ANDN2_B32:
    return PPU::S_ANDN2_SAVEEXEC_B32;
  case PPU::S_ORN2_B32:
    return PPU::S_ORN2_SAVEEXEC_B32;
  case PPU::S_NAND_B32:
    return PPU::S_NAND_SAVEEXEC_B32;
  case PPU::S_NOR_B32:
    return PPU::S_NOR_SAVEEXEC_B32;
  case PPU::S_XNOR_B32:
    return PPU::S_XNOR_SAVEEXEC_B32;
  default:
    return PPU::INSTRUCTION_LIST_END;
  }
  */
}

// These are only terminators to get correct spill code placement during
// register allocation, so turn them back into normal instructions. Only one of
// these is expected per block.
static bool removeTerminatorBit(const PPUInstrInfo &TII, MachineInstr &MI) {
  switch (MI.getOpcode()) {
    /*FIXME
  case PPU::S_MOV_B64_term:
  case PPU::S_MOV_B32_term: {
    MI.setDesc(TII.get(PPU::COPY));
    return true;
  }
  case PPU::SL_XOR_B64_term: {
    // This is only a terminator to get the correct spill code placement during
    // register allocation.
    MI.setDesc(TII.get(PPU::SL_XOR_B64));
    return true;
  }
  case PPU::S_XOR_B32_term: {
    // This is only a terminator to get the correct spill code placement during
    // register allocation.
    MI.setDesc(TII.get(PPU::S_XOR_B32));
    return true;
  }
  case PPU::S_OR_B32_term: {
    // This is only a terminator to get the correct spill code placement during
    // register allocation.
    MI.setDesc(TII.get(PPU::S_OR_B32));
    return true;
  }
  case PPU::S_ANDN2_B64_term: {
    // This is only a terminator to get the correct spill code placement during
    // register allocation.
    MI.setDesc(TII.get(PPU::S_ANDN2_B64));
    return true;
  }
  case PPU::S_ANDN2_B32_term: {
    // This is only a terminator to get the correct spill code placement during
    // register allocation.
    MI.setDesc(TII.get(PPU::S_ANDN2_B32));
    return true;
  }
  */
  default:
    return false;
  }
}

static MachineBasicBlock::reverse_iterator fixTerminators(
  const PPUInstrInfo &TII,
  MachineBasicBlock &MBB) {
  MachineBasicBlock::reverse_iterator I = MBB.rbegin(), E = MBB.rend();
  for (; I != E; ++I) {
    if (!I->isTerminator())
      return I;

    if (removeTerminatorBit(TII, *I))
      return I;
  }

  return E;
}

static MachineBasicBlock::reverse_iterator findExecCopy(
  const PPUInstrInfo &TII,
  const PPUSubtarget &ST,
  MachineBasicBlock &MBB,
  MachineBasicBlock::reverse_iterator I,
  unsigned CopyToExec) {
  const unsigned InstLimit = 25;

  auto E = MBB.rend();
  for (unsigned N = 0; N <= InstLimit && I != E; ++I, ++N) {
    unsigned CopyFromExec = isCopyFromExec(*I, ST);
    if (CopyFromExec != PPU::NoRegister)
      return I;
  }

  return E;
}

// XXX - Seems LivePhysRegs doesn't work correctly since it will incorrectly
// report the register as unavailable because a super-register with a lane mask
// is unavailable.
static bool isLiveOut(const MachineBasicBlock &MBB, unsigned Reg) {
  for (MachineBasicBlock *Succ : MBB.successors()) {
    if (Succ->isLiveIn(Reg))
      return true;
  }

  return false;
}

bool PPUOptimizeExecMasking::runOnMachineFunction(MachineFunction &MF) {
  if (skipFunction(MF.getFunction()))
    return false;

  const PPUSubtarget &ST = MF.getSubtarget<PPUSubtarget>();
  const PPURegisterInfo *TRI = ST.getRegisterInfo();
  const PPUInstrInfo *TII = ST.getInstrInfo();

  // Optimize sequences emitted for control flow lowering. They are originally
  // emitted as the separate operations because spill code may need to be
  // inserted for the saved copy of exec.
  //
  //     x = copy exec
  //     z = s_<op>_b64 x, y
  //     exec = copy z
  // =>
  //     x = s_<op>_saveexec_b64 y
  //

  for (MachineBasicBlock &MBB : MF) {
    MachineBasicBlock::reverse_iterator I = fixTerminators(*TII, MBB);
    MachineBasicBlock::reverse_iterator E = MBB.rend();
    if (I == E)
      continue;

    unsigned CopyToExec = isCopyToExec(*I, ST);
    if (CopyToExec == PPU::NoRegister)
      continue;

    // Scan backwards to find the def.
    auto CopyToExecInst = &*I;
    auto CopyFromExecInst = findExecCopy(*TII, ST, MBB, I, CopyToExec);
    if (CopyFromExecInst == E) {
      auto PrepareExecInst = std::next(I);
      if (PrepareExecInst == E)
        continue;
      // Fold exec = COPY (SL_AND_B64 reg, exec) -> exec = SL_AND_B64 reg, exec
      if (CopyToExecInst->getOperand(1).isKill() &&
          isLogicalOpOnExec(*PrepareExecInst) == CopyToExec) {
        LLVM_DEBUG(dbgs() << "Fold exec copy: " << *PrepareExecInst);

        PrepareExecInst->getOperand(0).setReg(PPU::TMSK);

        LLVM_DEBUG(dbgs() << "into: " << *PrepareExecInst << '\n');

        CopyToExecInst->eraseFromParent();
      }

      continue;
    }

    if (isLiveOut(MBB, CopyToExec)) {
      // The copied register is live out and has a second use in another block.
      LLVM_DEBUG(dbgs() << "Exec copy source register is live out\n");
      continue;
    }

    Register CopyFromExec = CopyFromExecInst->getOperand(0).getReg();
    MachineInstr *SaveTmskInst = nullptr;
    SmallVector<MachineInstr *, 4> OtherUseInsts;

    for (MachineBasicBlock::iterator J
           = std::next(CopyFromExecInst->getIterator()), JE = I->getIterator();
         J != JE; ++J) {
      if (SaveTmskInst && J->readsRegister(PPU::TMSK, TRI)) {
        LLVM_DEBUG(dbgs() << "exec read prevents saveexec: " << *J << '\n');
        // Make sure this is inserted after any VALU ops that may have been
        // scheduled in between.
        SaveTmskInst = nullptr;
        break;
      }

      bool ReadsCopyFromExec = J->readsRegister(CopyFromExec, TRI);

      if (J->modifiesRegister(CopyToExec, TRI)) {
        if (SaveTmskInst) {
          LLVM_DEBUG(dbgs() << "Multiple instructions modify "
                            << printReg(CopyToExec, TRI) << '\n');
          SaveTmskInst = nullptr;
          break;
        }

        unsigned SaveTmskOp = getSaveTmskOp(J->getOpcode());
        if (SaveTmskOp == PPU::INSTRUCTION_LIST_END)
          break;

        if (ReadsCopyFromExec) {
          SaveTmskInst = &*J;
          LLVM_DEBUG(dbgs() << "Found save exec op: " << *SaveTmskInst << '\n');
          continue;
        } else {
          LLVM_DEBUG(dbgs()
                     << "Instruction does not read exec copy: " << *J << '\n');
          break;
        }
      } else if (ReadsCopyFromExec && !SaveTmskInst) {
        // Make sure no other instruction is trying to use this copy, before it
        // will be rewritten by the saveexec, i.e. hasOneUse. There may have
        // been another use, such as an inserted spill. For example:
        //
        // %sgpr0_sgpr1 = COPY %exec
        // spill %sgpr0_sgpr1
        // %sgpr2_sgpr3 = SL_AND_B64 %sgpr0_sgpr1
        //
        LLVM_DEBUG(dbgs() << "Found second use of save inst candidate: " << *J
                          << '\n');
        break;
      }

      if (SaveTmskInst && J->readsRegister(CopyToExec, TRI)) {
        assert(SaveTmskInst != &*J);
        OtherUseInsts.push_back(&*J);
      }
    }

    if (!SaveTmskInst)
      continue;

    LLVM_DEBUG(dbgs() << "Insert save exec op: " << *SaveTmskInst << '\n');

    MachineOperand &Src0 = SaveTmskInst->getOperand(1);
    MachineOperand &Src1 = SaveTmskInst->getOperand(2);

    MachineOperand *OtherOp = nullptr;

    if (Src0.isReg() && Src0.getReg() == CopyFromExec) {
      OtherOp = &Src1;
    } else if (Src1.isReg() && Src1.getReg() == CopyFromExec) {
      if (!SaveTmskInst->isCommutable())
        break;

      OtherOp = &Src0;
    } else
      llvm_unreachable("unexpected");

    CopyFromExecInst->eraseFromParent();

    auto InsPt = SaveTmskInst->getIterator();
    const DebugLoc &DL = SaveTmskInst->getDebugLoc();

    BuildMI(MBB, InsPt, DL, TII->get(getSaveTmskOp(SaveTmskInst->getOpcode())),
            CopyFromExec)
      .addReg(OtherOp->getReg())
      .addReg(PPU::TMSK);
    SaveTmskInst->eraseFromParent();

    CopyToExecInst->eraseFromParent();

    for (MachineInstr *OtherInst : OtherUseInsts) {
      OtherInst->substituteRegister(CopyToExec, PPU::TMSK,
                                    PPU::NoSubRegister, *TRI);
    }
  }

  return true;

}

