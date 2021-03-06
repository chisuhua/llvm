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
#include "llvm/IR/CallingConv.h"
#include "llvm/MC/SubtargetFeature.h"
#include "llvm/CodeGen/SelectionDAG.h"
#include "llvm/Support/PPUKernelDescriptor.h"
#include "PPUDefines.h"
#include "PPUKernelCodeT.h"

namespace llvm {

class Argument;
class PPUSubtarget;
class FeatureBitset;
class Function;
class GlobalValue;

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
#include "PPUGenSystemOperands.inc"  // TODO rvv GenSearchableTables.inc
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
  ABI_PPT,
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

namespace PPU {

/* TODO
#define GET_MIMGBaseOpcode_DECL
#define GET_MIMGDim_DECL
#define GET_MIMGEncoding_DECL
#define GET_MIMGLZMapping_DECL
#define GET_MIMGMIPMapping_DECL
#include "AMDGPUGenSearchableTables.inc"
*/

namespace IsaInfo {

/// Instruction set architecture version.
struct IsaVersion {
  unsigned Major;
  unsigned Minor;
  unsigned Stepping;
};

/// \returns Isa version for given subtarget \p Features.
IsaVersion getIsaVersion(const MCSubtargetInfo *STI) ;

IsaVersion getIsaVersion(const llvm::StringRef) ;


/// Streams isa version string for given subtarget \p STI into \p Stream.
void streamIsaVersion(const MCSubtargetInfo *STI, raw_ostream &Stream);

/// \returns True if given subtarget \p STI supports code object version 3,
/// false otherwise.
bool hasCodeObjectV3(const MCSubtargetInfo *STI);

/// \returns Wavefront size for given subtarget \p STI.
unsigned getWavefrontSize(const MCSubtargetInfo *STI);

/// \returns Local memory size in bytes for given subtarget \p STI.
unsigned getLocalMemorySize(const MCSubtargetInfo *STI);

/// \returns Number of execution units per compute unit for given subtarget \p
/// STI.
// unsigned getEUsPerCU(const MCSubtargetInfo *STI);
unsigned getEUsPerCU(const MCSubtargetInfo *STI);

/// \returns Maximum number of work groups per compute unit for given subtarget
/// \p STI and limited by given \p FlatWorkGroupSize.
unsigned getMaxWorkGroupsPerCU(const MCSubtargetInfo *STI,
                               unsigned FlatWorkGroupSize);

/// \returns Maximum number of waves per compute unit for given subtarget \p
/// STI without any kind of limitation.
unsigned getMaxWavesPerCU(const MCSubtargetInfo *STI);

/// \returns Maximum number of waves per compute unit for given subtarget \p
/// STI and limited by given \p FlatWorkGroupSize.
unsigned getMaxWavesPerCU(const MCSubtargetInfo *STI,
                          unsigned FlatWorkGroupSize);

/// \returns Minimum number of waves per execution unit for given subtarget \p
/// STI.
unsigned getMinWavesPerEU(const MCSubtargetInfo *STI);

unsigned getMaxWavesPerEU(const MCSubtargetInfo *STI);
/// \returns Maximum number of waves per execution unit for given subtarget \p
/// STI and limited by given \p FlatWorkGroupSize.
unsigned getMaxWavesPerEU(const MCSubtargetInfo *STI,
                          unsigned FlatWorkGroupSize);

/// \returns Minimum flat work group size for given subtarget \p STI.
unsigned getMinFlatWorkGroupSize(const MCSubtargetInfo *STI);

/// \returns Maximum flat work group size for given subtarget \p STI.
unsigned getMaxFlatWorkGroupSize(const MCSubtargetInfo *STI);

/// \returns Number of waves per work group for given subtarget \p STI and
/// limited by given \p FlatWorkGroupSize.
unsigned getWavesPerWorkGroup(const MCSubtargetInfo *STI,
                              unsigned FlatWorkGroupSize);

/// \returns SGPR allocation granularity for given subtarget \p STI.
unsigned getSGPRAllocGranule(const MCSubtargetInfo *STI);

/// \returns SGPR encoding granularity for given subtarget \p STI.
unsigned getSGPREncodingGranule(const MCSubtargetInfo *STI);

/// \returns Total number of SGPRs for given subtarget \p STI.
unsigned getTotalNumSGPRs(const MCSubtargetInfo *STI);

/// \returns Addressable number of SGPRs for given subtarget \p STI.
unsigned getAddressableNumSGPRs(const MCSubtargetInfo *STI);

/// \returns Minimum number of SGPRs that meets the given number of waves per
/// execution unit requirement for given subtarget \p STI.
unsigned getMinNumSGPRs(const MCSubtargetInfo *STI, unsigned WavesPerEU);

/// \returns Maximum number of SGPRs that meets the given number of waves per
/// execution unit requirement for given subtarget \p STI.
unsigned getMaxNumSGPRs(const MCSubtargetInfo *STI, unsigned WavesPerEU,
                        bool Addressable);

/// \returns Number of extra SGPRs implicitly required by given subtarget \p
/// STI when the given special registers are used.
unsigned getNumExtraSGPRs(const MCSubtargetInfo *STI, bool VCCUsed,
                          bool FlatScrUsed, bool XNACKUsed);

/// \returns Number of extra SGPRs implicitly required by given subtarget \p
/// STI when the given special registers are used. XNACK is inferred from
/// \p STI.
unsigned getNumExtraSGPRs(const MCSubtargetInfo *STI, bool VCCUsed,
                          bool FlatScrUsed);

/// \returns Number of SGPR blocks needed for given subtarget \p STI when
/// \p NumSGPRs are used. \p NumSGPRs should already include any special
/// register counts.
unsigned getNumSGPRBlocks(const MCSubtargetInfo *STI, unsigned NumSGPRs);

/// \returns VGPR allocation granularity for given subtarget \p STI.
///
/// For subtargets which support it, \p EnableWavefrontSize32 should match
/// the ENABLE_WAVEFRONT_SIZE32 kernel descriptor field.
unsigned getVGPRAllocGranule(const MCSubtargetInfo *STI,
                             Optional<bool> EnableWavefrontSize32 = None);

/// \returns VGPR encoding granularity for given subtarget \p STI.
///
/// For subtargets which support it, \p EnableWavefrontSize32 should match
/// the ENABLE_WAVEFRONT_SIZE32 kernel descriptor field.
unsigned getVGPREncodingGranule(const MCSubtargetInfo *STI,
                                Optional<bool> EnableWavefrontSize32 = None);

/// \returns Total number of VGPRs for given subtarget \p STI.
unsigned getTotalNumVGPRs(const MCSubtargetInfo *STI);

/// \returns Addressable number of VGPRs for given subtarget \p STI.
unsigned getAddressableNumVGPRs(const MCSubtargetInfo *STI);

/// \returns Minimum number of VGPRs that meets given number of waves per
/// execution unit requirement for given subtarget \p STI.
unsigned getMinNumVGPRs(const MCSubtargetInfo *STI, unsigned WavesPerEU);

/// \returns Maximum number of VGPRs that meets given number of waves per
/// execution unit requirement for given subtarget \p STI.
unsigned getMaxNumVGPRs(const MCSubtargetInfo *STI, unsigned WavesPerEU);

/// \returns Number of VGPR blocks needed for given subtarget \p STI when
/// \p NumVGPRs are used.
///
/// For subtargets which support it, \p EnableWavefrontSize32 should match the
/// ENABLE_WAVEFRONT_SIZE32 kernel descriptor field.
unsigned getNumVPRBlocks(const MCSubtargetInfo *STI, unsigned NumSGPRs,
                          Optional<bool> EnableWavefrontSize32 = None);

} // end namespace IsaInfo

using namespace IsaInfo;
LLVM_READONLY
int16_t getNamedOperandIdx(uint16_t Opcode, uint16_t NamedIdx);

/* TODO 
LLVM_READONLY
int getSOPPWithRelaxation(uint16_t Opcode);

struct MIMGBaseOpcodeInfo {
  MIMGBaseOpcode BaseOpcode;
  bool Store;
  bool Atomic;
  bool AtomicX2;
  bool Sampler;
  bool Gather4;

  uint8_t NumExtraArgs;
  bool Gradients;
  bool Coordinates;
  bool LodOrClampOrMip;
  bool HasD16;
};

LLVM_READONLY
const MIMGBaseOpcodeInfo *getMIMGBaseOpcodeInfo(unsigned BaseOpcode);

struct MIMGDimInfo {
  MIMGDim Dim;
  uint8_t NumCoords;
  uint8_t NumGradients;
  bool DA;
  uint8_t Encoding;
  const char *AsmSuffix;
};

LLVM_READONLY
const MIMGDimInfo *getMIMGDimInfo(unsigned DimEnum);

LLVM_READONLY
const MIMGDimInfo *getMIMGDimInfoByEncoding(uint8_t DimEnc);

LLVM_READONLY
const MIMGDimInfo *getMIMGDimInfoByAsmSuffix(StringRef AsmSuffix);

struct MIMGLZMappingInfo {
  MIMGBaseOpcode L;
  MIMGBaseOpcode LZ;
};

struct MIMGMIPMappingInfo {
  MIMGBaseOpcode MIP;
  MIMGBaseOpcode NONMIP;
};

LLVM_READONLY
const MIMGLZMappingInfo *getMIMGLZMappingInfo(unsigned L);

LLVM_READONLY
const MIMGMIPMappingInfo *getMIMGMIPMappingInfo(unsigned L);

LLVM_READONLY
int getMIMGOpcode(unsigned BaseOpcode, unsigned MIMGEncoding,
                  unsigned VDataDwords, unsigned VAddrDwords);

LLVM_READONLY
int getMaskedMIMGOp(unsigned Opc, unsigned NewChannels);

struct MIMGInfo {
  uint16_t Opcode;
  uint16_t BaseOpcode;
  uint8_t MIMGEncoding;
  uint8_t VDataDwords;
  uint8_t VAddrDwords;
};

LLVM_READONLY
const MIMGInfo *getMIMGInfo(unsigned Opc);
*/

LLVM_READONLY
int getMUBUFBaseOpcode(unsigned Opc);

LLVM_READONLY
int getMUBUFOpcode(unsigned BaseOpc, unsigned Elements);

LLVM_READONLY
int getMUBUFElements(unsigned Opc);

LLVM_READONLY
bool getMUBUFHasVAddr(unsigned Opc);

LLVM_READONLY
bool getMUBUFHasSrsrc(unsigned Opc);

LLVM_READONLY
bool getMUBUFHasSoffset(unsigned Opc);


LLVM_READONLY
int getMCOpcode(uint16_t Opcode, unsigned Gen);

void initDefaultPPUKernelCodeT(amd_kernel_code_t &Header,
                               const MCSubtargetInfo *STI);

pps::kernel_descriptor_t getDefaultPPUKernelDescriptor(
    const MCSubtargetInfo *STI);

bool isGroupSegment(const GlobalValue *GV);
bool isGlobalSegment(const GlobalValue *GV);
bool isReadOnlySegment(const GlobalValue *GV);

/// \returns True if constants should be emitted to .text section for given
/// target triple \p TT, false otherwise.
bool shouldEmitConstantsToTextSection(const Triple &TT);

/// \returns Integer value requested using \p F's \p Name attribute.
///
/// \returns \p Default if attribute is not present.
///
/// \returns \p Default and emits error if requested value cannot be converted
/// to integer.
int getIntegerAttribute(const Function &F, StringRef Name, int Default);

/// \returns A pair of integer values requested using \p F's \p Name attribute
/// in "first[,second]" format ("second" is optional unless \p OnlyFirstRequired
/// is false).
///
/// \returns \p Default if attribute is not present.
///
/// \returns \p Default and emits error if one of the requested values cannot be
/// converted to integer, or \p OnlyFirstRequired is false and "second" value is
/// not present.
std::pair<int, int> getIntegerPairAttribute(const Function &F,
                                            StringRef Name,
                                            std::pair<int, int> Default,
                                            bool OnlyFirstRequired = false);
/// Represents the counter values to wait for in an s_waitcnt instruction.
///
/// Large values (including the maximum possible integer) can be used to
/// represent "don't care" waits.
struct Waitcnt {
  unsigned VmCnt = ~0u;
  unsigned ExpCnt = ~0u;
  unsigned LgkmCnt = ~0u;
  unsigned VsCnt = ~0u;

  Waitcnt() {}
  Waitcnt(unsigned VmCnt, unsigned ExpCnt, unsigned LgkmCnt, unsigned VsCnt)
      : VmCnt(VmCnt), ExpCnt(ExpCnt), LgkmCnt(LgkmCnt), VsCnt(VsCnt) {}

  static Waitcnt allZero(const IsaVersion &Version) {
    return Waitcnt(0, 0, 0, Version.Major >= 10 ? 0 : ~0u);
  }
  static Waitcnt allZeroExceptVsCnt() { return Waitcnt(0, 0, 0, ~0u); }

  bool hasWait() const {
    return VmCnt != ~0u || ExpCnt != ~0u || LgkmCnt != ~0u || VsCnt != ~0u;
  }

  bool dominates(const Waitcnt &Other) const {
    return VmCnt <= Other.VmCnt && ExpCnt <= Other.ExpCnt &&
           LgkmCnt <= Other.LgkmCnt && VsCnt <= Other.VsCnt;
  }

  Waitcnt combined(const Waitcnt &Other) const {
    return Waitcnt(std::min(VmCnt, Other.VmCnt), std::min(ExpCnt, Other.ExpCnt),
                   std::min(LgkmCnt, Other.LgkmCnt),
                   std::min(VsCnt, Other.VsCnt));
  }
};

/// \returns Vmcnt bit mask for given isa \p Version.
unsigned getVmcntBitMask(const IsaVersion &Version);

/// \returns Expcnt bit mask for given isa \p Version.
unsigned getExpcntBitMask(const IsaVersion &Version);

/// \returns Lgkmcnt bit mask for given isa \p Version.
unsigned getLgkmcntBitMask(const IsaVersion &Version);

/// \returns Waitcnt bit mask for given isa \p Version.
unsigned getWaitcntBitMask(const IsaVersion &Version);

/// \returns Decoded Vmcnt from given \p Waitcnt for given isa \p Version.
unsigned decodeVmcnt(const IsaVersion &Version, unsigned Waitcnt);

/// \returns Decoded Expcnt from given \p Waitcnt for given isa \p Version.
unsigned decodeExpcnt(const IsaVersion &Version, unsigned Waitcnt);

/// \returns Decoded Lgkmcnt from given \p Waitcnt for given isa \p Version.
unsigned decodeLgkmcnt(const IsaVersion &Version, unsigned Waitcnt);

/// Decodes Vmcnt, Expcnt and Lgkmcnt from given \p Waitcnt for given isa
/// \p Version, and writes decoded values into \p Vmcnt, \p Expcnt and
/// \p Lgkmcnt respectively.
///
/// \details \p Vmcnt, \p Expcnt and \p Lgkmcnt are decoded as follows:
///     \p Vmcnt = \p Waitcnt[3:0]                      (pre-gfx9 only)
///     \p Vmcnt = \p Waitcnt[3:0] | \p Waitcnt[15:14]  (gfx9+ only)
///     \p Expcnt = \p Waitcnt[6:4]
///     \p Lgkmcnt = \p Waitcnt[11:8]                   (pre-gfx10 only)
///     \p Lgkmcnt = \p Waitcnt[13:8]                   (gfx10+ only)
void decodeWaitcnt(const IsaVersion &Version, unsigned Waitcnt,
                   unsigned &Vmcnt, unsigned &Expcnt, unsigned &Lgkmcnt);

Waitcnt decodeWaitcnt(const IsaVersion &Version, unsigned Encoded);

/// \returns \p Waitcnt with encoded \p Vmcnt for given isa \p Version.
unsigned encodeVmcnt(const IsaVersion &Version, unsigned Waitcnt,
                     unsigned Vmcnt);

/// \returns \p Waitcnt with encoded \p Expcnt for given isa \p Version.
unsigned encodeExpcnt(const IsaVersion &Version, unsigned Waitcnt,
                      unsigned Expcnt);

/// \returns \p Waitcnt with encoded \p Lgkmcnt for given isa \p Version.
unsigned encodeLgkmcnt(const IsaVersion &Version, unsigned Waitcnt,
                       unsigned Lgkmcnt);

/// Encodes \p Vmcnt, \p Expcnt and \p Lgkmcnt into Waitcnt for given isa
/// \p Version.
///
/// \details \p Vmcnt, \p Expcnt and \p Lgkmcnt are encoded as follows:
///     Waitcnt[3:0]   = \p Vmcnt       (pre-gfx9 only)
///     Waitcnt[3:0]   = \p Vmcnt[3:0]  (gfx9+ only)
///     Waitcnt[6:4]   = \p Expcnt
///     Waitcnt[11:8]  = \p Lgkmcnt     (pre-gfx10 only)
///     Waitcnt[13:8]  = \p Lgkmcnt     (gfx10+ only)
///     Waitcnt[15:14] = \p Vmcnt[5:4]  (gfx9+ only)
///
/// \returns Waitcnt with encoded \p Vmcnt, \p Expcnt and \p Lgkmcnt for given
/// isa \p Version.
unsigned encodeWaitcnt(const IsaVersion &Version,
                       unsigned Vmcnt, unsigned Expcnt, unsigned Lgkmcnt);

unsigned encodeWaitcnt(const IsaVersion &Version, const Waitcnt &Decoded);

namespace Hwreg {

LLVM_READONLY
int64_t getHwregId(const StringRef Name);

LLVM_READNONE
bool isValidHwreg(int64_t Id, const MCSubtargetInfo &STI);

LLVM_READNONE
bool isValidHwreg(int64_t Id);

LLVM_READNONE
bool isValidHwregOffset(int64_t Offset);

LLVM_READNONE
bool isValidHwregWidth(int64_t Width);

LLVM_READNONE
uint64_t encodeHwreg(uint64_t Id, uint64_t Offset, uint64_t Width);

LLVM_READNONE
StringRef getHwreg(unsigned Id, const MCSubtargetInfo &STI);

void decodeHwreg(unsigned Val, unsigned &Id, unsigned &Offset, unsigned &Width);

} // namespace Hwreg

/*
namespace SendMsg {

LLVM_READONLY
int64_t getMsgId(const StringRef Name);

LLVM_READONLY
int64_t getMsgOpId(int64_t MsgId, const StringRef Name);

LLVM_READNONE
StringRef getMsgName(int64_t MsgId);

LLVM_READNONE
StringRef getMsgOpName(int64_t MsgId, int64_t OpId);

LLVM_READNONE
bool isValidMsgId(int64_t MsgId, const MCSubtargetInfo &STI, bool Strict = true);

LLVM_READNONE
bool isValidMsgOp(int64_t MsgId, int64_t OpId, bool Strict = true);

LLVM_READNONE
bool isValidMsgStream(int64_t MsgId, int64_t OpId, int64_t StreamId, bool Strict = true);

LLVM_READNONE
bool msgRequiresOp(int64_t MsgId);

LLVM_READNONE
bool msgSupportsStream(int64_t MsgId, int64_t OpId);

void decodeMsg(unsigned Val,
               uint16_t &MsgId,
               uint16_t &OpId,
               uint16_t &StreamId);

LLVM_READNONE
uint64_t encodeMsg(uint64_t MsgId,
                   uint64_t OpId,
                   uint64_t StreamId);

} // namespace SendMsg


unsigned getInitialPSInputAddr(const Function &F);

*/
LLVM_READNONE
bool isShader(CallingConv::ID CC);

LLVM_READNONE
bool isCompute(CallingConv::ID CC);

LLVM_READNONE
bool isCompute(SelectionDAG *DAG);

LLVM_READNONE
bool isCompute(MachineFunction *MF);

LLVM_READNONE
bool isCompute(const MachineFunction *MF);

LLVM_READNONE
bool isEntryFunctionCC(CallingConv::ID CC);

// FIXME: Remove this when calling conventions cleaned up
// LLVM_READNONE
bool isKernel(CallingConv::ID CC) ;
/*
bool hasXNACK(const MCSubtargetInfo &STI);
bool hasSRAMECC(const MCSubtargetInfo &STI);
bool hasMIMG_R128(const MCSubtargetInfo &STI);
bool hasPackedD16(const MCSubtargetInfo &STI);

bool isSI(const MCSubtargetInfo &STI);
bool isCI(const MCSubtargetInfo &STI);
bool isVI(const MCSubtargetInfo &STI);
bool isGFX9(const MCSubtargetInfo &STI);
bool isGFX10(const MCSubtargetInfo &STI);
*/

/// Is Reg - scalar register
bool isSGPR(unsigned Reg, const MCRegisterInfo* TRI);

/// Is there any intersection between registers
bool isRegIntersect(unsigned Reg0, unsigned Reg1, const MCRegisterInfo* TRI);

/// If \p Reg is a pseudo reg, return the correct hardware register given
/// \p STI otherwise return \p Reg.
unsigned getMCReg(unsigned Reg, const MCSubtargetInfo &STI);

/// Convert hardware register \p Reg to a pseudo register
LLVM_READNONE
unsigned mc2PseudoReg(unsigned Reg);

/// Can this operand also contain immediate values?
bool isSISrcOperand(const MCInstrDesc &Desc, unsigned OpNo);

/// Is this floating-point operand?
bool isSISrcFPOperand(const MCInstrDesc &Desc, unsigned OpNo);

/// Does this opearnd support only inlinable literals?
bool isSISrcInlinableOperand(const MCInstrDesc &Desc, unsigned OpNo);

/// Get the size in bits of a register from the register class \p RC.
unsigned getRegBitWidth(unsigned RCID);

/// Get the size in bits of a register from the register class \p RC.
unsigned getRegBitWidth(const MCRegisterClass &RC);

/// Get size of register operand
unsigned getRegOperandSize(const MCRegisterInfo *MRI, const MCInstrDesc &Desc,
                           unsigned OpNo);

LLVM_READNONE
inline unsigned getOperandSize(const MCOperandInfo &OpInfo) {
  switch (OpInfo.OperandType) {
  case PPU::OPERAND_REG_IMM_INT32:
  case PPU::OPERAND_REG_IMM_FP32:
  case PPU::OPERAND_REG_INLINE_C_INT32:
  case PPU::OPERAND_REG_INLINE_C_FP32:
  case PPU::OPERAND_REG_INLINE_AC_INT32:
  case PPU::OPERAND_REG_INLINE_AC_FP32:
    return 4;

  case PPU::OPERAND_REG_IMM_INT64:
  case PPU::OPERAND_REG_IMM_FP64:
  case PPU::OPERAND_REG_INLINE_C_INT64:
  case PPU::OPERAND_REG_INLINE_C_FP64:
    return 8;

  case PPU::OPERAND_REG_IMM_INT16:
  case PPU::OPERAND_REG_IMM_FP16:
  case PPU::OPERAND_REG_INLINE_C_INT16:
  case PPU::OPERAND_REG_INLINE_C_FP16:
  case PPU::OPERAND_REG_INLINE_C_V2INT16:
  case PPU::OPERAND_REG_INLINE_C_V2FP16:
  case PPU::OPERAND_REG_INLINE_AC_INT16:
  case PPU::OPERAND_REG_INLINE_AC_FP16:
  case PPU::OPERAND_REG_INLINE_AC_V2INT16:
  case PPU::OPERAND_REG_INLINE_AC_V2FP16:
  case PPU::OPERAND_REG_IMM_V2INT16:
  case PPU::OPERAND_REG_IMM_V2FP16:
    return 2;

  default:
    llvm_unreachable("unhandled operand type");
  }
}

LLVM_READNONE
inline unsigned getOperandSize(const MCInstrDesc &Desc, unsigned OpNo) {
  return getOperandSize(Desc.OpInfo[OpNo]);
}

/// Is this literal inlinable
LLVM_READNONE
bool isInlinableLiteral64(int64_t Literal, bool HasInv2Pi);

LLVM_READNONE
bool isInlinableLiteral32(int32_t Literal, bool HasInv2Pi);

LLVM_READNONE
bool isInlinableLiteral16(int16_t Literal, bool HasInv2Pi);

LLVM_READNONE
bool isInlinableLiteralV216(int32_t Literal, bool HasInv2Pi);


bool isArgPassedInSGPR(const Argument *Arg);

/// \returns The encoding that will be used for \p ByteOffset in the SMRD
/// offset field.
int64_t getSMRDEncodedOffset(const MCSubtargetInfo &ST, int64_t ByteOffset);

/// \returns true if this offset is small enough to fit in the SMRD
/// offset field.  \p ByteOffset should be the offset in bytes and
/// not the encoded offset.
bool isLegalSMRDImmOffset(const MCSubtargetInfo &ST, int64_t ByteOffset);

bool splitMUBUFOffset(uint32_t Imm, uint32_t &SOffset, uint32_t &ImmOffset,
                      const PPUSubtarget *Subtarget, uint32_t Align = 4);

/// \returns true if the intrinsic is divergent
bool isIntrinsicSourceOfDivergence(unsigned IntrID);

// Track defaults for fields in the MODE registser.
struct PPUModeRegisterDefaults {
  /// Floating point opcodes that support exception flag gathering quiet and
  /// propagate signaling NaN inputs per IEEE 754-2008. Min_dx10 and max_dx10
  /// become IEEE 754- 2008 compliant due to signaling NaN propagation and
  /// quieting.
  bool IEEE : 1;

  /// Used by the vector ALU to force DX10-style treatment of NaNs: when set,
  /// clamp NaN to zero; otherwise, pass NaN through.
  bool DX10Clamp : 1;

  // TODO: FP mode fields

  PPUModeRegisterDefaults() :
    IEEE(true),
    DX10Clamp(true) {}

  PPUModeRegisterDefaults(const Function &F);

  static PPUModeRegisterDefaults getDefaultForCallingConv(CallingConv::ID CC) {
    PPUModeRegisterDefaults Mode;
    Mode.DX10Clamp = true;
    Mode.IEEE = PPU::isCompute(CC);
    return Mode;
  }

  bool operator ==(const PPUModeRegisterDefaults Other) const {
    return IEEE == Other.IEEE && DX10Clamp == Other.DX10Clamp;
  }

  // FIXME: Inlining should be OK for dx10-clamp, since the caller's mode should
  // be able to override.
  bool isInlineCompatible(PPUModeRegisterDefaults CalleeMode) const {
    return *this == CalleeMode;
  }
};


} // end namespace AMDGPU

} // namespace llvm

#endif
