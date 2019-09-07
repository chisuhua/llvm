//===-- PPUFixupKinds.h - PPU Specific Fixup Entries --------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_PPU_MCTARGETDESC_PPUFIXUPKINDS_H
#define LLVM_LIB_TARGET_PPU_MCTARGETDESC_PPUFIXUPKINDS_H

#include "llvm/MC/MCFixup.h"

#undef PPU

namespace llvm {
namespace PPU {
enum Fixups {
  // fixup_ppu_hi20 - 20-bit fixup corresponding to hi(foo) for
  // instructions like lui
  fixup_ppu_hi20 = FirstTargetFixupKind,
  // fixup_ppu_lo12_i - 12-bit fixup corresponding to lo(foo) for
  // instructions like addi
  fixup_ppu_lo12_i,
  // fixup_ppu_lo12_s - 12-bit fixup corresponding to lo(foo) for
  // the S-type store instructions
  fixup_ppu_lo12_s,
  // fixup_ppu_pcrel_hi20 - 20-bit fixup corresponding to pcrel_hi(foo) for
  // instructions like auipc
  fixup_ppu_pcrel_hi20,
  // fixup_ppu_pcrel_lo12_i - 12-bit fixup corresponding to pcrel_lo(foo) for
  // instructions like addi
  fixup_ppu_pcrel_lo12_i,
  // fixup_ppu_pcrel_lo12_s - 12-bit fixup corresponding to pcrel_lo(foo) for
  // the S-type store instructions
  fixup_ppu_pcrel_lo12_s,
  // fixup_ppu_got_hi20 - 20-bit fixup corresponding to got_pcrel_hi(foo) for
  // instructions like auipc
  fixup_ppu_got_hi20,
  // fixup_ppu_tprel_hi20 - 20-bit fixup corresponding to tprel_hi(foo) for
  // instructions like lui
  fixup_ppu_tprel_hi20,
  // fixup_ppu_tprel_lo12_i - 12-bit fixup corresponding to tprel_lo(foo) for
  // instructions like addi
  fixup_ppu_tprel_lo12_i,
  // fixup_ppu_tprel_lo12_s - 12-bit fixup corresponding to tprel_lo(foo) for
  // the S-type store instructions
  fixup_ppu_tprel_lo12_s,
  // fixup_ppu_tprel_add - A fixup corresponding to %tprel_add(foo) for the
  // add_tls instruction. Used to provide a hint to the linker.
  fixup_ppu_tprel_add,
  // fixup_ppu_tls_got_hi20 - 20-bit fixup corresponding to
  // tls_ie_pcrel_hi(foo) for instructions like auipc
  fixup_ppu_tls_got_hi20,
  // fixup_ppu_tls_gd_hi20 - 20-bit fixup corresponding to
  // tls_gd_pcrel_hi(foo) for instructions like auipc
  fixup_ppu_tls_gd_hi20,
  // fixup_ppu_jal - 20-bit fixup for symbol references in the jal
  // instruction
  fixup_ppu_jal,
  // fixup_ppu_branch - 12-bit fixup for symbol references in the branch
  // instructions
  fixup_ppu_branch,
  // fixup_ppu_rvc_jump - 11-bit fixup for symbol references in the
  // compressed jump instruction
  fixup_ppu_rvc_jump,
  // fixup_ppu_rvc_branch - 8-bit fixup for symbol references in the
  // compressed branch instruction
  fixup_ppu_rvc_branch,
  // fixup_ppu_call - A fixup representing a call attached to the auipc
  // instruction in a pair composed of adjacent auipc+jalr instructions.
  fixup_ppu_call,
  // fixup_ppu_call_plt - A fixup representing a procedure linkage table call
  // attached to the auipc instruction in a pair composed of adjacent auipc+jalr
  // instructions.
  fixup_ppu_call_plt,
  // fixup_ppu_relax - Used to generate an R_PPU_RELAX relocation type,
  // which indicates the linker may relax the instruction pair.
  fixup_ppu_relax,
  // fixup_ppu_align - Used to generate an R_PPU_ALIGN relocation type,
  // which indicates the linker should fixup the alignment after linker
  // relaxation.
  fixup_ppu_align,

  // fixup_ppu_invalid - used as a sentinel and a marker, must be last fixup
  fixup_ppu_invalid,
  NumTargetFixupKinds = fixup_ppu_invalid - FirstTargetFixupKind
};
} // end namespace PPU
} // end namespace llvm

#endif
