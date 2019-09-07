//===-- PPUMCTargetDesc.cpp - PPU Target Descriptions -----------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
///
/// This file provides PPU-specific target descriptions.
///
//===----------------------------------------------------------------------===//

#include "PPUMCTargetDesc.h"
#include "PPUELFStreamer.h"
#include "PPUInstPrinter.h"
#include "PPUMCAsmInfo.h"
#include "PPUTargetStreamer.h"
#include "TargetInfo/PPUTargetInfo.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/CodeGen/Register.h"
#include "llvm/MC/MCAsmInfo.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/TargetRegistry.h"

#define GET_INSTRINFO_MC_DESC
#include "PPUGenInstrInfo.inc"

#define GET_REGINFO_MC_DESC
#include "PPUGenRegisterInfo.inc"

#define GET_SUBTARGETINFO_MC_DESC
#include "PPUGenSubtargetInfo.inc"

using namespace llvm;

static MCInstrInfo *createPPUMCInstrInfo() {
  MCInstrInfo *X = new MCInstrInfo();
  InitPPUMCInstrInfo(X);
  return X;
}

static MCRegisterInfo *createPPUMCRegisterInfo(const Triple &TT) {
  MCRegisterInfo *X = new MCRegisterInfo();
  InitPPUMCRegisterInfo(X, PPU::X1);
  return X;
}

static MCAsmInfo *createPPUMCAsmInfo(const MCRegisterInfo &MRI,
                                       const Triple &TT) {
  MCAsmInfo *MAI = new PPUMCAsmInfo(TT);

  Register SP = MRI.getDwarfRegNum(PPU::X2, true);
  MCCFIInstruction Inst = MCCFIInstruction::createDefCfa(nullptr, SP, 0);
  MAI->addInitialFrameState(Inst);

  return MAI;
}

static MCSubtargetInfo *createPPUMCSubtargetInfo(const Triple &TT,
                                                   StringRef CPU, StringRef FS) {
  std::string CPUName = CPU;
  if (CPUName.empty())
    CPUName = "generic-ppu";
  return createPPUMCSubtargetInfoImpl(TT, CPUName, FS);
}

static MCInstPrinter *createPPUMCInstPrinter(const Triple &T,
                                               unsigned SyntaxVariant,
                                               const MCAsmInfo &MAI,
                                               const MCInstrInfo &MII,
                                               const MCRegisterInfo &MRI) {
  return new PPUInstPrinter(MAI, MII, MRI);
}

static MCTargetStreamer *
createPPUObjectTargetStreamer(MCStreamer &S, const MCSubtargetInfo &STI) {
  const Triple &TT = STI.getTargetTriple();
  if (TT.isOSBinFormatELF())
    return new PPUTargetELFStreamer(S, STI);
  return nullptr;
}

static MCTargetStreamer *createPPUAsmTargetStreamer(MCStreamer &S,
                                                      formatted_raw_ostream &OS,
                                                      MCInstPrinter *InstPrint,
                                                      bool isVerboseAsm) {
  return new PPUTargetAsmStreamer(S, OS);
}

extern "C" void LLVMInitializePPUTargetMC() {
  for (Target *T : {&getThePPUTarget()}) {
    TargetRegistry::RegisterMCAsmInfo(*T, createPPUMCAsmInfo);
    TargetRegistry::RegisterMCInstrInfo(*T, createPPUMCInstrInfo);
    TargetRegistry::RegisterMCRegInfo(*T, createPPUMCRegisterInfo);
    TargetRegistry::RegisterMCAsmBackend(*T, createPPUAsmBackend);
    TargetRegistry::RegisterMCCodeEmitter(*T, createPPUMCCodeEmitter);
    TargetRegistry::RegisterMCInstPrinter(*T, createPPUMCInstPrinter);
    TargetRegistry::RegisterMCSubtargetInfo(*T, createPPUMCSubtargetInfo);
    TargetRegistry::RegisterObjectTargetStreamer(
        *T, createPPUObjectTargetStreamer);

    // Register the asm target streamer.
    TargetRegistry::RegisterAsmTargetStreamer(*T, createPPUAsmTargetStreamer);
  }
}
