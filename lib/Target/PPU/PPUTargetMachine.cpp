//===-- PPUTargetMachine.cpp - Define TargetMachine for PPU -----------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// Implements the info about PPU target spec.
//
//===----------------------------------------------------------------------===//

#include "PPUTargetMachine.h"
#include "PPU.h"
#include "PPUTargetObjectFile.h"
#include "PPUTargetTransformInfo.h"
#include "TargetInfo/PPUTargetInfo.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/Analysis/TargetTransformInfo.h"
#include "llvm/CodeGen/GlobalISel/IRTranslator.h"
#include "llvm/CodeGen/GlobalISel/InstructionSelect.h"
#include "llvm/CodeGen/GlobalISel/Legalizer.h"
#include "llvm/CodeGen/GlobalISel/RegBankSelect.h"
#include "llvm/CodeGen/Passes.h"
#include "llvm/CodeGen/TargetLoweringObjectFileImpl.h"
#include "llvm/CodeGen/TargetPassConfig.h"
#include "llvm/IR/LegacyPassManager.h"
#include "llvm/Support/FormattedStream.h"
#include "llvm/Support/TargetRegistry.h"
#include "llvm/Target/TargetOptions.h"
using namespace llvm;

extern "C" void LLVMInitializePPUTarget() {
  RegisterTargetMachine<PPUTargetMachine> X(getThePPUTarget());
  auto PR = PassRegistry::getPassRegistry();
  initializeGlobalISel(*PR);
  initializePPUExpandPseudoPass(*PR);
}

static StringRef computeDataLayout(const Triple &TT) {
  return "e-m:e-p:32:32-i64:64-n32-S128";
}

static Reloc::Model getEffectiveRelocModel(const Triple &TT,
                                           Optional<Reloc::Model> RM) {
  if (!RM.hasValue())
    return Reloc::Static;
  return *RM;
}

PPUTargetMachine::PPUTargetMachine(const Target &T, const Triple &TT,
                                       StringRef CPU, StringRef FS,
                                       const TargetOptions &Options,
                                       Optional<Reloc::Model> RM,
                                       Optional<CodeModel::Model> CM,
                                       CodeGenOpt::Level OL, bool JIT)
    : LLVMTargetMachine(T, computeDataLayout(TT), TT, CPU, FS, Options,
                        getEffectiveRelocModel(TT, RM),
                        getEffectiveCodeModel(CM, CodeModel::Small), OL),
      TLOF(std::make_unique<PPUELFTargetObjectFile>()),
      Subtarget(TT, CPU, FS, Options.MCOptions.getABIName(), *this) {
  initAsmInfo();
}

TargetTransformInfo
PPUTargetMachine::getTargetTransformInfo(const Function &F) {
  return TargetTransformInfo(PPUTTIImpl(this, F));
}

namespace {
class PPUPassConfig : public TargetPassConfig {
public:
  PPUPassConfig(PPUTargetMachine &TM, PassManagerBase &PM)
      : TargetPassConfig(TM, PM) {}

  PPUTargetMachine &getPPUTargetMachine() const {
    return getTM<PPUTargetMachine>();
  }

  void addIRPasses() override;
  bool addInstSelector() override;
  bool addIRTranslator() override;
  bool addLegalizeMachineIR() override;
  bool addRegBankSelect() override;
  bool addGlobalInstructionSelect() override;
  void addPreEmitPass() override;
  void addPreEmitPass2() override;
  void addPreRegAlloc() override;
  void addMachineSSAOptimization() override;
};
}

TargetPassConfig *PPUTargetMachine::createPassConfig(PassManagerBase &PM) {
  return new PPUPassConfig(*this, PM);
}

void PPUPassConfig::addIRPasses() {
  addPass(createAtomicExpandPass());
  TargetPassConfig::addIRPasses();
}

bool PPUPassConfig::addInstSelector() {
  addPass(createPPUISelDag(getPPUTargetMachine()));

  return false;
}

bool PPUPassConfig::addIRTranslator() {
  addPass(new IRTranslator());
  return false;
}

bool PPUPassConfig::addLegalizeMachineIR() {
  addPass(new Legalizer());
  return false;
}

bool PPUPassConfig::addRegBankSelect() {
  addPass(new RegBankSelect());
  return false;
}

bool PPUPassConfig::addGlobalInstructionSelect() {
  addPass(new InstructionSelect());
  return false;
}

void PPUPassConfig::addPreEmitPass() { addPass(&BranchRelaxationPassID); }

void PPUPassConfig::addPreEmitPass2() {
  // Schedule the expansion of AMOs at the last possible moment, avoiding the
  // possibility for other passes to break the requirements for forward
  // progress in the LR/SC block.
  addPass(createPPUExpandPseudoPass());
}

void PPUPassConfig::addMachineSSAOptimization() {
  TargetPassConfig::addMachineSSAOptimization();
  addPass(createPPUOptimizeVSETVLUsesPass());
}

void PPUPassConfig::addPreRegAlloc() {
  addPass(createPPUMergeBaseOffsetOptPass());
}
