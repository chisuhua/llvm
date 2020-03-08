//===-- PPUTargetStreamer.h - PPU Target Streamer ----------*- C++ -*--===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_PPU_PPUTARGETSTREAMER_H
#define LLVM_LIB_TARGET_PPU_PPUTARGETSTREAMER_H

#include "PPUKernelCodeT.h"
#include "llvm/BinaryFormat/MsgPackDocument.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/Support/PPUMetadata.h"
#include "llvm/Support/PPUKernelDescriptor.h"

namespace llvm {
#include "PPUPTNote.h"

class DataLayout;
class Function;
class MCELFStreamer;
class MCSymbol;
class MDNode;
class Module;
class Type;

class PPUTargetStreamer : public MCTargetStreamer {
protected:
  MCContext &getContext() const { return Streamer.getContext(); }
public:
  PPUTargetStreamer(MCStreamer &S);

  virtual void emitDirectiveOptionPush() = 0;
  virtual void emitDirectiveOptionPop() = 0;
  virtual void emitDirectiveOptionRVC() = 0;
  virtual void emitDirectiveOptionNoRVC() = 0;
  virtual void emitDirectiveOptionRelax() = 0;
  virtual void emitDirectiveOptionNoRelax() = 0;

// below is from AMD
  virtual void EmitDirectivePPUTarget(StringRef Target) = 0;

  // virtual void EmitDirectiveHSACodeObjectVersion(uint32_t Major,
  //                                                uint32_t Minor) = 0;

  // virtual void EmitDirectiveHSACodeObjectISA(uint32_t Major, uint32_t Minor,
  //                                            uint32_t Stepping,
  //                                            StringRef VendorName,
  //                                            StringRef ArchName) = 0;

  // virtual void EmitAMDKernelCodeT(const amd_kernel_code_t &Header) = 0;

  // virtual void EmitAMDGPUSymbolType(StringRef SymbolName, unsigned Type) = 0;

  virtual void emitPPULDS(MCSymbol *Symbol, unsigned Size,
                             unsigned Align) = 0;

  /// \returns True on success, false on failure.
  // virtual bool EmitISAVersion(StringRef IsaVersionString) = 0;

  /// \returns True on success, false on failure.
  virtual bool EmitPPSMetadataV3(StringRef PPSMetadataString);

  /// Emit HSA Metadata
  ///
  /// When \p Strict is true, known metadata elements must already be
  /// well-typed. When \p Strict is false, known types are inferred and
  /// the \p PPSMetadata structure is updated with the correct types.
  ///
  /// \returns True on success, false on failure.
  virtual bool EmitPPSMetadata(msgpack::Document &PPSMetadata, bool Strict) = 0;

  /// \returns True on success, false on failure.
  // virtual bool EmitPPSMetadata(const PPU::PPSMD::Metadata &PPSMetadata) = 0;

  /// \returns True on success, false on failure.
  virtual bool EmitCodeEnd() = 0;

  virtual void EmitPpsKernelDescriptor(
      const MCSubtargetInfo &STI, StringRef KernelName,
      const pps::kernel_descriptor_t &KernelDescriptor, uint64_t NextVGPR,
      uint64_t NextSGPR, bool ReserveVCC, bool ReserveFlatScr,
      bool ReserveXNACK) = 0;

  static StringRef getArchNameFromElfMach(unsigned ElfMach);
  static unsigned getElfMach(StringRef GPU);

};

// This part is for ascii assembly output
class PPUTargetAsmStreamer : public PPUTargetStreamer {
  formatted_raw_ostream &OS;

public:
  PPUTargetAsmStreamer(MCStreamer &S, formatted_raw_ostream &OS);

  void emitDirectiveOptionPush() override;
  void emitDirectiveOptionPop() override;
  void emitDirectiveOptionRVC() override;
  void emitDirectiveOptionNoRVC() override;
  void emitDirectiveOptionRelax() override;
  void emitDirectiveOptionNoRelax() override;

// below from AMD
  void finish() override;

  void EmitDirectivePPUTarget(StringRef Target) override;

  // void EmitDirectiveHSACodeObjectVersion(uint32_t Major,
  //                                       uint32_t Minor) override;

  // void EmitDirectiveHSACodeObjectISA(uint32_t Major, uint32_t Minor,
  //                                   uint32_t Stepping, StringRef VendorName,
  //                                   StringRef ArchName) override;

  // void EmitAMDKernelCodeT(const amd_kernel_code_t &Header) override;

  // void EmitAMDGPUSymbolType(StringRef SymbolName, unsigned Type) override;

  void emitPPULDS(MCSymbol *Sym, unsigned Size, unsigned Align) override;

  /// \returns True on success, false on failure.
  // bool EmitISAVersion(StringRef IsaVersionString) override;

  /// \returns True on success, false on failure.
  bool EmitPPSMetadata(msgpack::Document &PPSMetadata, bool Strict) override;

  /// \returns True on success, false on failure.
  // bool EmitPPSMetadata(const PPU::PPSMD::Metadata &PPSMetadata) override;

  /// \returns True on success, false on failure.
  bool EmitCodeEnd() override;

  void EmitPpsKernelDescriptor(
      const MCSubtargetInfo &STI, StringRef KernelName,
      const pps::kernel_descriptor_t &KernelDescriptor, uint64_t NextVGPR,
      uint64_t NextSGPR, bool ReserveVCC, bool ReserveFlatScr,
      bool ReserveXNACK) override;

};

}
#endif
