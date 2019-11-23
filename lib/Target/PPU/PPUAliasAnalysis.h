//===- PPUAliasAnalysis --------------------------------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
/// \file
/// This is the AMGPU address space based alias analysis pass.
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_PPU_ALIASANALYSIS_H
#define LLVM_LIB_TARGET_PPU_ALIASANALYSIS_H

#include "PPU.h"
#include "llvm/ADT/Triple.h"
#include "llvm/Analysis/AliasAnalysis.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/Module.h"
#include "llvm/Pass.h"
#include <algorithm>
#include <memory>

namespace llvm {

class DataLayout;
class MDNode;
class MemoryLocation;

/// A simple AA result that uses TBAA metadata to answer queries.
class PPUAAResult : public AAResultBase<PPUAAResult> {
  friend AAResultBase<PPUAAResult>;

  const DataLayout &DL;

public:
  explicit PPUAAResult(const DataLayout &DL, Triple T) : AAResultBase(),
    DL(DL) {}
  PPUAAResult(PPUAAResult &&Arg)
      : AAResultBase(std::move(Arg)), DL(Arg.DL) {}

  /// Handle invalidation events from the new pass manager.
  ///
  /// By definition, this result is stateless and so remains valid.
  bool invalidate(Function &, const PreservedAnalyses &) { return false; }

  AliasResult alias(const MemoryLocation &LocA, const MemoryLocation &LocB,
                    AAQueryInfo &AAQI);
  bool pointsToConstantMemory(const MemoryLocation &Loc, AAQueryInfo &AAQI,
                              bool OrLocal);

private:
  bool Aliases(const MDNode *A, const MDNode *B) const;
  bool PathAliases(const MDNode *A, const MDNode *B) const;
};

/// Analysis pass providing a never-invalidated alias analysis result.
class PPUAA : public AnalysisInfoMixin<PPUAA> {
  friend AnalysisInfoMixin<PPUAA>;

  static char PassID;

public:
  using Result = PPUAAResult;

  PPUAAResult run(Function &F, AnalysisManager<Function> &AM) {
    return PPUAAResult(F.getParent()->getDataLayout(),
        Triple(F.getParent()->getTargetTriple()));
  }
};

/// Legacy wrapper pass to provide the PPUAAResult object.
class PPUAAWrapperPass : public ImmutablePass {
  std::unique_ptr<PPUAAResult> Result;

public:
  static char ID;

  PPUAAWrapperPass() : ImmutablePass(ID) {
    initializePPUAAWrapperPassPass(*PassRegistry::getPassRegistry());
  }

  PPUAAResult &getResult() { return *Result; }
  const PPUAAResult &getResult() const { return *Result; }

  bool doInitialization(Module &M) override {
    Result.reset(new PPUAAResult(M.getDataLayout(),
        Triple(M.getTargetTriple())));
    return false;
  }

  bool doFinalization(Module &M) override {
    Result.reset();
    return false;
  }

  void getAnalysisUsage(AnalysisUsage &AU) const override;
};

// Wrapper around ExternalAAWrapperPass so that the default constructor gets the
// callback.
class PPUExternalAAWrapper : public ExternalAAWrapperPass {
public:
  static char ID;

  PPUExternalAAWrapper() : ExternalAAWrapperPass(
    [](Pass &P, Function &, AAResults &AAR) {
      if (auto *WrapperPass = P.getAnalysisIfAvailable<PPUAAWrapperPass>())
        AAR.addAAResult(WrapperPass->getResult());
    }) {}
};

} // end namespace llvm

#endif // LLVM_LIB_TARGET_PPU_ALIASANALYSIS_H
