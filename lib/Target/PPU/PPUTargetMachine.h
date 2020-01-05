//===-- PPUTargetMachine.h - Define TargetMachine for PPU ---*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file declares the PPU specific subclass of TargetMachine.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_PPU_PPUTARGETMACHINE_H
#define LLVM_LIB_TARGET_PPU_PPUTARGETMACHINE_H

#include "PPUSubtarget.h"
// #include "PPUIntrinsicInfo.h"
#include "MCTargetDesc/PPUMCTargetDesc.h"
#include "llvm/CodeGen/SelectionDAGTargetInfo.h"
#include "llvm/IR/DataLayout.h"
#include "llvm/Analysis/TargetTransformInfo.h"
#include "llvm/Support/CodeGen.h"
#include "llvm/Target/TargetMachine.h"

namespace llvm {
class PPUTargetMachine : public LLVMTargetMachine {
protected:
  std::unique_ptr<TargetLoweringObjectFile> TLOF;

  StringRef getFeatureString(const Function &F) const;

  // PPUSubtarget Subtarget;
  // PPUIntrinsicInfo IntrinsicInfo;
  mutable StringMap<std::unique_ptr<PPUSubtarget>> SubtargetMap;

public:
  // const TargetOptions Options;
  static bool EnableReconvergeCFG;
  static bool EnableLateStructurizeCFG;
  static bool EnableFunctionCalls;

  PPUTargetMachine(const Target &T, const Triple &TT, StringRef CPU,
                     StringRef FS, const TargetOptions &Options,
                     Optional<Reloc::Model> RM, Optional<CodeModel::Model> CM,
                     CodeGenOpt::Level OL, bool JIT);
/*
  const PPUSubtarget *getSubtargetImpl() const
  {
    return &Subtarget;
  }
*/
  const PPUSubtarget *getSubtargetImpl(const Function &) const override;

  TargetPassConfig *createPassConfig(PassManagerBase &PM) override;


  TargetLoweringObjectFile *getObjFileLowering() const override {
    return TLOF.get();
  }

  TargetTransformInfo getTargetTransformInfo(const Function &F) override;

  void adjustPassManager(PassManagerBuilder &) override;
  /// Get the integer value of a null pointer in the given address space.
  uint64_t getNullPointerValue(unsigned AddrSpace) const {
    return (AddrSpace == AMDGPUAS::LOCAL_ADDRESS ||
            AddrSpace == AMDGPUAS::REGION_ADDRESS) ? -1 : 0;
  }

  bool useIPRA() const override {
    return true;
  }

  // const PPUIntrinsicInfo *getIntrinsicInfo() const override {
  //   return &IntrinsicInfo;
  // }

};
}

#endif
