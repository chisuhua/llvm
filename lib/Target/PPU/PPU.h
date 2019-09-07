//===-- PPU.h - Top-level interface for PPU -----------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the entry points for global functions defined in the LLVM
// PPU back-end.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_PPU_PPU_H
#define LLVM_LIB_TARGET_PPU_PPU_H

#include "Utils/PPUBaseInfo.h"
#include "llvm/Target/TargetMachine.h"

namespace llvm {
class PPURegisterBankInfo;
class PPUSubtarget;
class PPUTargetMachine;
class AsmPrinter;
class FunctionPass;
class InstructionSelector;
class MCInst;
class MCOperand;
class MachineInstr;
class MachineOperand;
class PassRegistry;

void LowerPPUMachineInstrToMCInst(const MachineInstr *MI, MCInst &OutMI,
                                    const AsmPrinter &AP);
bool LowerPPUMachineOperandToMCOperand(const MachineOperand &MO,
                                         MCOperand &MCOp, const AsmPrinter &AP);

FunctionPass *createPPUISelDag(PPUTargetMachine &TM);

FunctionPass *createPPUMergeBaseOffsetOptPass();
void initializePPUMergeBaseOffsetOptPass(PassRegistry &);

FunctionPass *createPPUExpandPseudoPass();
void initializePPUExpandPseudoPass(PassRegistry &);

InstructionSelector *createPPUInstructionSelector(const PPUTargetMachine &,
                                                    PPUSubtarget &,
                                                    PPURegisterBankInfo &);
}

#endif
