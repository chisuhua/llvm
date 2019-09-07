//===- PPUTargetTransformInfo.h - PPU specific TTI ---------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
/// \file
/// This file defines a TargetTransformInfo::Concept conforming object specific
/// to the PPU target machine. It uses the target's detailed information to
/// provide more precise answers to certain TTI queries, while letting the
/// target independent and default TTI implementations handle the rest.
///
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_PPU_PPUTARGETTRANSFORMINFO_H
#define LLVM_LIB_TARGET_PPU_PPUTARGETTRANSFORMINFO_H

#include "PPUSubtarget.h"
#include "PPUTargetMachine.h"
#include "llvm/Analysis/TargetTransformInfo.h"
#include "llvm/CodeGen/BasicTTIImpl.h"
#include "llvm/IR/Function.h"

namespace llvm {

class PPUTTIImpl : public BasicTTIImplBase<PPUTTIImpl> {
  using BaseT = BasicTTIImplBase<PPUTTIImpl>;
  using TTI = TargetTransformInfo;

  friend BaseT;

  const PPUSubtarget *ST;
  const PPUTargetLowering *TLI;

  const PPUSubtarget *getST() const { return ST; }
  const PPUTargetLowering *getTLI() const { return TLI; }

public:
  explicit PPUTTIImpl(const PPUTargetMachine *TM, const Function &F)
      : BaseT(TM, F.getParent()->getDataLayout()), ST(TM->getSubtargetImpl(F)),
        TLI(ST->getTargetLowering()) {}

  int getIntImmCost(const APInt &Imm, Type *Ty);
  int getIntImmCost(unsigned Opcode, unsigned Idx, const APInt &Imm, Type *Ty);
  int getIntImmCost(Intrinsic::ID IID, unsigned Idx, const APInt &Imm,
                    Type *Ty);
};

} // end namespace llvm

#endif // LLVM_LIB_TARGET_PPU_PPUTARGETTRANSFORMINFO_H
