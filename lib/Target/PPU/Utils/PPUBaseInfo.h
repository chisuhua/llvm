//===-- PPUBaseInfo.h - Top level definitions for PPU MC ----*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains small standalone enum definitions for the PPU target
// useful for the compiler back-end and the MC libraries.
//
//===----------------------------------------------------------------------===//
#ifndef LLVM_LIB_TARGET_PPU_MCTARGETDESC_PPUBASEINFO_H
#define LLVM_LIB_TARGET_PPU_MCTARGETDESC_PPUBASEINFO_H

#include "MCTargetDesc/PPUMCTargetDesc.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/ADT/StringSwitch.h"
#include "llvm/MC/SubtargetFeature.h"

namespace llvm {

// PPUII - This namespace holds all of the target specific flags that
// instruction info tracks. All definitions must match PPUInstrFormats.td.
namespace PPUII {
enum {
  InstFormatPseudo = 0,
  InstFormatR = 1,
  InstFormatR4 = 2,
  InstFormatI = 3,
  InstFormatS = 4,
  InstFormatB = 5,
  InstFormatU = 6,
  InstFormatJ = 7,
  InstFormatCR = 8,
  InstFormatCI = 9,
  InstFormatCSS = 10,
  InstFormatCIW = 11,
  InstFormatCL = 12,
  InstFormatCS = 13,
  InstFormatCA = 14,
  InstFormatCB = 15,
  InstFormatCJ = 16,
  InstFormatOther = 17,

  InstFormatMask = 31
};

enum {
  MO_None,
  MO_CALL,
  MO_PLT,
  MO_LO,
  MO_HI,
  MO_PCREL_LO,
  MO_PCREL_HI,
  MO_GOT_HI,
  MO_TPREL_LO,
  MO_TPREL_HI,
  MO_TPREL_ADD,
  MO_TLS_GOT_HI,
  MO_TLS_GD_HI,
};
} // namespace PPUII

// Describes the predecessor/successor bits used in the FENCE instruction.
namespace PPUFenceField {
enum FenceField {
  I = 8,
  O = 4,
  R = 2,
  W = 1
};
}

// Describes the supported floating point rounding mode encodings.
namespace PPUFPRndMode {
enum RoundingMode {
  RNE = 0,
  RTZ = 1,
  RDN = 2,
  RUP = 3,
  RMM = 4,
  DYN = 7,
  Invalid
};

inline static StringRef roundingModeToString(RoundingMode RndMode) {
  switch (RndMode) {
  default:
    llvm_unreachable("Unknown floating point rounding mode");
  case PPUFPRndMode::RNE:
    return "rne";
  case PPUFPRndMode::RTZ:
    return "rtz";
  case PPUFPRndMode::RDN:
    return "rdn";
  case PPUFPRndMode::RUP:
    return "rup";
  case PPUFPRndMode::RMM:
    return "rmm";
  case PPUFPRndMode::DYN:
    return "dyn";
  }
}

inline static RoundingMode stringToRoundingMode(StringRef Str) {
  return StringSwitch<RoundingMode>(Str)
      .Case("rne", PPUFPRndMode::RNE)
      .Case("rtz", PPUFPRndMode::RTZ)
      .Case("rdn", PPUFPRndMode::RDN)
      .Case("rup", PPUFPRndMode::RUP)
      .Case("rmm", PPUFPRndMode::RMM)
      .Case("dyn", PPUFPRndMode::DYN)
      .Default(PPUFPRndMode::Invalid);
}

inline static bool isValidRoundingMode(unsigned Mode) {
  switch (Mode) {
  default:
    return false;
  case PPUFPRndMode::RNE:
  case PPUFPRndMode::RTZ:
  case PPUFPRndMode::RDN:
  case PPUFPRndMode::RUP:
  case PPUFPRndMode::RMM:
  case PPUFPRndMode::DYN:
    return true;
  }
}
} // namespace PPUFPRndMode

namespace PPUSysReg {
struct SysReg {
  const char *Name;
  unsigned Encoding;
  // FIXME: add these additional fields when needed.
  // Privilege Access: Read, Write, Read-Only.
  // unsigned ReadWrite;
  // Privilege Mode: User, System or Machine.
  // unsigned Mode;
  // Check field name.
  // unsigned Extra;
  // Register number without the privilege bits.
  // unsigned Number;
  FeatureBitset FeaturesRequired;
  bool isRV32Only;

  bool haveRequiredFeatures(FeatureBitset ActiveFeatures) const {
    // Not in 32-bit mode.
    if (isRV32Only && ActiveFeatures[PPU::Feature64Bit])
      return false;
    // No required feature associated with the system register.
    if (FeaturesRequired.none())
      return true;
    return (FeaturesRequired & ActiveFeatures) == FeaturesRequired;
  }
};

#define GET_SysRegsList_DECL
#include "PPUGenSystemOperands.inc"
} // end namespace PPUSysReg

namespace PPUABI {

enum ABI {
  ABI_ILP32,
  ABI_ILP32F,
  ABI_ILP32D,
  ABI_ILP32E,
  ABI_LP64,
  ABI_LP64F,
  ABI_LP64D,
  ABI_Unknown
};

// Returns the target ABI, or else a StringError if the requested ABIName is
// not supported for the given TT and FeatureBits combination.
ABI computeTargetABI(const Triple &TT, FeatureBitset FeatureBits,
                     StringRef ABIName);

} // namespace PPUABI

namespace PPUFeatures {

// Validates if the given combination of features are valid for the target
// triple. Exits with report_fatal_error if not.
void validate(const Triple &TT, const FeatureBitset &FeatureBits);

} // namespace PPUFeatures

} // namespace llvm

#endif
