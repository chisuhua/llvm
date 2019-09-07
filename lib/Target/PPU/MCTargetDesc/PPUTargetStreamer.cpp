//===-- PPUTargetStreamer.cpp - PPU Target Streamer Methods -----------===//
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

#include "PPUTargetStreamer.h"
#include "llvm/Support/FormattedStream.h"

using namespace llvm;

PPUTargetStreamer::PPUTargetStreamer(MCStreamer &S) : MCTargetStreamer(S) {}

// This part is for ascii assembly output
PPUTargetAsmStreamer::PPUTargetAsmStreamer(MCStreamer &S,
                                               formatted_raw_ostream &OS)
    : PPUTargetStreamer(S), OS(OS) {}

void PPUTargetAsmStreamer::emitDirectiveOptionPush() {
  OS << "\t.option\tpush\n";
}

void PPUTargetAsmStreamer::emitDirectiveOptionPop() {
  OS << "\t.option\tpop\n";
}

void PPUTargetAsmStreamer::emitDirectiveOptionRVC() {
  OS << "\t.option\trvc\n";
}

void PPUTargetAsmStreamer::emitDirectiveOptionNoRVC() {
  OS << "\t.option\tnorvc\n";
}

void PPUTargetAsmStreamer::emitDirectiveOptionRelax() {
  OS << "\t.option\trelax\n";
}

void PPUTargetAsmStreamer::emitDirectiveOptionNoRelax() {
  OS << "\t.option\tnorelax\n";
}
