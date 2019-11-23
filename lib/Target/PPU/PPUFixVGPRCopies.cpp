//===-- PPUFixVGPRCopies.cpp - Fix VGPR Copies after regalloc --------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
/// \file
/// Add implicit use of exec to vector register copies.
///
//===----------------------------------------------------------------------===//

#include "PPU.h"
#include "PPUSubtarget.h"
#include "PPUInstrInfo.h"
#include "MCTargetDesc/PPUMCTargetDesc.h"
#include "llvm/CodeGen/MachineFunctionPass.h"

using namespace llvm;

#define DEBUG_TYPE "ppu-fix-vgpr-copies"

namespace {

class PPUFixVGPRCopies : public MachineFunctionPass {
public:
  static char ID;

public:
  PPUFixVGPRCopies() : MachineFunctionPass(ID) {
    initializePPUFixVGPRCopiesPass(*PassRegistry::getPassRegistry());
  }

  bool runOnMachineFunction(MachineFunction &MF) override;

  StringRef getPassName() const override { return "PPU Fix VGPR copies"; }
};

} // End anonymous namespace.

INITIALIZE_PASS(PPUFixVGPRCopies, DEBUG_TYPE, "PPU Fix VGPR copies", false, false)

char PPUFixVGPRCopies::ID = 0;

char &llvm::PPUFixVGPRCopiesID = PPUFixVGPRCopies::ID;

bool PPUFixVGPRCopies::runOnMachineFunction(MachineFunction &MF) {
  const PPUSubtarget &ST = MF.getSubtarget<PPUSubtarget>();
  const PPURegisterInfo *TRI = ST.getRegisterInfo();
  const PPUInstrInfo *TII = ST.getInstrInfo();
  bool Changed = false;

  for (MachineBasicBlock &MBB : MF) {
    for (MachineInstr &MI : MBB) {
      switch (MI.getOpcode()) {
      case PPU::COPY:
        if (TII->isVGPRCopy(MI) && !MI.readsRegister(PPU::TMSK, TRI)) {
          MI.addOperand(MF,
                        MachineOperand::CreateReg(PPU::TMSK, false, true));
          LLVM_DEBUG(dbgs() << "Add exec use to " << MI);
          Changed = true;
        }
        break;
      default:
        break;
      }
    }
  }

  return Changed;
}
