//===-- PPUTargetInfo.cpp - PPU Target Implementation -----------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "TargetInfo/PPUTargetInfo.h"
#include "llvm/Support/TargetRegistry.h"
using namespace llvm;

Target &llvm::getThePPUTarget() {
  static Target ThePPUTarget;
  return ThePPUTarget;
}

extern "C" void LLVMInitializePPUTargetInfo() {
  RegisterTarget<Triple::ppu> X(getThePPUTarget(), "ppu",
                                    "PPU target", "PPU");
}
