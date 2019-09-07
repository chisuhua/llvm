//===-- PPUELFStreamer.cpp - PPU ELF Target Streamer Methods ----------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file provides PPU specific target streamer methods.
//
//===----------------------------------------------------------------------===//

#include "PPUELFStreamer.h"
#include "MCTargetDesc/PPUAsmBackend.h"
#include "PPUMCTargetDesc.h"
#include "Utils/PPUBaseInfo.h"
#include "llvm/BinaryFormat/ELF.h"
#include "llvm/MC/MCSubtargetInfo.h"

using namespace llvm;

// This part is for ELF object output.
PPUTargetELFStreamer::PPUTargetELFStreamer(MCStreamer &S,
                                               const MCSubtargetInfo &STI)
    : PPUTargetStreamer(S) {
  MCAssembler &MCA = getStreamer().getAssembler();
  const FeatureBitset &Features = STI.getFeatureBits();
  auto &MAB = static_cast<PPUAsmBackend &>(MCA.getBackend());
  PPUABI::ABI ABI = MAB.getTargetABI();
  assert(ABI != PPUABI::ABI_Unknown && "Improperly initialised target ABI");

  unsigned EFlags = MCA.getELFHeaderEFlags();

  if (Features[PPU::FeatureStdExtC])
    EFlags |= ELF::EF_PPU_RVC;

  switch (ABI) {
  case PPUABI::ABI_ILP32:
  case PPUABI::ABI_LP64:
    break;
  case PPUABI::ABI_ILP32F:
  case PPUABI::ABI_LP64F:
    EFlags |= ELF::EF_PPU_FLOAT_ABI_SINGLE;
    break;
  case PPUABI::ABI_ILP32D:
  case PPUABI::ABI_LP64D:
    EFlags |= ELF::EF_PPU_FLOAT_ABI_DOUBLE;
    break;
  case PPUABI::ABI_ILP32E:
    EFlags |= ELF::EF_PPU_RVE;
    break;
  case PPUABI::ABI_Unknown:
    llvm_unreachable("Improperly initialised target ABI");
  }

  MCA.setELFHeaderEFlags(EFlags);
}

MCELFStreamer &PPUTargetELFStreamer::getStreamer() {
  return static_cast<MCELFStreamer &>(Streamer);
}

void PPUTargetELFStreamer::emitDirectiveOptionPush() {}
void PPUTargetELFStreamer::emitDirectiveOptionPop() {}
void PPUTargetELFStreamer::emitDirectiveOptionRVC() {}
void PPUTargetELFStreamer::emitDirectiveOptionNoRVC() {}
void PPUTargetELFStreamer::emitDirectiveOptionRelax() {}
void PPUTargetELFStreamer::emitDirectiveOptionNoRelax() {}
