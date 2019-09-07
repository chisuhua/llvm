//===-- PPUSubtarget.cpp - PPU Subtarget Information ------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file implements the PPU specific subclass of TargetSubtargetInfo.
//
//===----------------------------------------------------------------------===//

#include "PPUSubtarget.h"
#include "PPU.h"
#include "PPUCallLowering.h"
#include "PPUFrameLowering.h"
#include "PPULegalizerInfo.h"
#include "PPURegisterBankInfo.h"
#include "PPUTargetMachine.h"
#include "llvm/Support/TargetRegistry.h"

using namespace llvm;

#define DEBUG_TYPE "ppu-subtarget"

#define GET_SUBTARGETINFO_TARGET_DESC
#define GET_SUBTARGETINFO_CTOR
#include "PPUGenSubtargetInfo.inc"

void PPUSubtarget::anchor() {}

PPUSubtarget &PPUSubtarget::initializeSubtargetDependencies(
    const Triple &TT, StringRef CPU, StringRef FS, StringRef ABIName) {
  // Determine default and user-specified characteristics
  bool Is64Bit = TT.isArch64Bit();
  std::string CPUName = CPU;
  if (CPUName.empty())
    CPUName = Is64Bit ? "generic-ppu64" : "generic-ppu";
  ParseSubtargetFeatures(CPUName, FS);
  if (Is64Bit) {
    XLenVT = MVT::i64;
    XLen = 64;
  }

  TargetABI = PPUABI::computeTargetABI(TT, getFeatureBits(), ABIName);
  PPUFeatures::validate(TT, getFeatureBits());
  return *this;
}

PPUSubtarget::PPUSubtarget(const Triple &TT, StringRef CPU, StringRef FS,
                               StringRef ABIName, const TargetMachine &TM)
    : PPUGenSubtargetInfo(TT, CPU, FS),
      FrameLowering(initializeSubtargetDependencies(TT, CPU, FS, ABIName)),
      InstrInfo(), RegInfo(getHwMode()), TLInfo(TM, *this) {
  CallLoweringInfo.reset(new PPUCallLowering(*getTargetLowering()));
  Legalizer.reset(new PPULegalizerInfo(*this));

  auto *RBI = new PPURegisterBankInfo(*getRegisterInfo());
  RegBankInfo.reset(RBI);
  InstSelector.reset(createPPUInstructionSelector(
      *static_cast<const PPUTargetMachine *>(&TM), *this, *RBI));
}

const CallLowering *PPUSubtarget::getCallLowering() const {
  return CallLoweringInfo.get();
}

InstructionSelector *PPUSubtarget::getInstructionSelector() const {
  return InstSelector.get();
}

const LegalizerInfo *PPUSubtarget::getLegalizerInfo() const {
  return Legalizer.get();
}

const RegisterBankInfo *PPUSubtarget::getRegBankInfo() const {
  return RegBankInfo.get();
}
