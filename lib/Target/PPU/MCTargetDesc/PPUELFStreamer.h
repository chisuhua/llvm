//===-- PPUELFStreamer.h - PPU ELF Target Streamer ---------*- C++ -*--===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_PPU_PPUELFSTREAMER_H
#define LLVM_LIB_TARGET_PPU_PPUELFSTREAMER_H

#include "PPUTargetStreamer.h"
#include "llvm/MC/MCELFStreamer.h"

namespace llvm {

class PPUTargetELFStreamer : public PPUTargetStreamer {
  MCStreamer &Streamer;

  void EmitNote(StringRef Name, const MCExpr *DescSize, unsigned NoteType,
                function_ref<void(MCELFStreamer &)> EmitDesc);

public:
  MCELFStreamer &getStreamer();
  PPUTargetELFStreamer(MCStreamer &S, const MCSubtargetInfo &STI);

  virtual void emitDirectiveOptionPush();
  virtual void emitDirectiveOptionPop();
  virtual void emitDirectiveOptionRVC();
  virtual void emitDirectiveOptionNoRVC();
  virtual void emitDirectiveOptionRelax();
  virtual void emitDirectiveOptionNoRelax();

  // below from AMD
  void finish() override;

  void EmitDirectivePPUTarget(StringRef Target) override;

  // void EmitDirectiveHSACodeObjectVersion(uint32_t Major,
  //                                        uint32_t Minor) override;

  //void EmitDirectiveHSACodeObjectISA(uint32_t Major, uint32_t Minor,
  //                                   uint32_t Stepping, StringRef VendorName,
  //                                   StringRef ArchName) override;

  void EmitAMDKernelCodeT(const amd_kernel_code_t &Header) override;

  void EmitAMDGPUSymbolType(StringRef SymbolName, unsigned Type) override;

  void emitPPULDS(MCSymbol *Sym, unsigned Size, unsigned Align) override;

  /// \returns True on success, false on failure.
  // bool EmitISAVersion(StringRef IsaVersionString) override;

  /// \returns True on success, false on failure.
  bool EmitHSAMetadata(msgpack::Document &HSAMetadata, bool Strict) override;

  /// \returns True on success, false on failure.
  // bool EmitHSAMetadata(const PPU::HSAMD::Metadata &HSAMetadata) override;

  /// \returns True on success, false on failure.
  bool EmitCodeEnd() override;

  void EmitAmdhsaKernelDescriptor(
      const MCSubtargetInfo &STI, StringRef KernelName,
      const amdhsa::kernel_descriptor_t &KernelDescriptor, uint64_t NextVGPR,
      uint64_t NextSGPR, bool ReserveVCC, bool ReserveFlatScr,
      bool ReserveXNACK) override;


};
}
#endif
