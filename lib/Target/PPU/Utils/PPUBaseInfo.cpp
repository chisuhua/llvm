#include "PPUBaseInfo.h"
#include "PPUTargetTransformInfo.h"
#include "PPUAsmUtils.h"
#include "llvm/ADT/ArrayRef.h"
#include "llvm/ADT/Triple.h"
#include "llvm/Support/raw_ostream.h"

#include "llvm/BinaryFormat/ELF.h"
#include "llvm/CodeGen/MachineMemOperand.h"
#include "llvm/IR/Attributes.h"
#include "llvm/IR/Constants.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/GlobalValue.h"
#include "llvm/IR/Instruction.h"
#include "llvm/IR/LLVMContext.h"
#include "llvm/IR/Module.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCInstrDesc.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCSectionELF.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/MC/SubtargetFeature.h"
#include "llvm/Support/Casting.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/MathExtras.h"

#define GET_INSTRINFO_NAMED_OPS
#define GET_INSTRMAP_INFO
#include "PPUGenInstrInfo.inc"
#undef GET_INSTRMAP_INFO
#undef GET_INSTRINFO_NAMED_OPS

namespace llvm {
namespace PPUSysReg {
#define GET_SysRegsList_IMPL
#include "PPUGenSystemOperands.inc"  // TODO RISCVGenSearchableTables.inc
} // namespace PPUSysReg

namespace PPUABI {
ABI computeTargetABI(const Triple &TT, FeatureBitset FeatureBits,
                     StringRef ABIName) {
  auto TargetABI = StringSwitch<ABI>(ABIName)
                       .Case("ilp32", ABI_ILP32)
                       .Case("ilp32f", ABI_ILP32F)
                       .Case("ilp32d", ABI_ILP32D)
                       .Case("ilp32e", ABI_ILP32E)
                       .Case("lp64", ABI_LP64)
                       .Case("lp64f", ABI_LP64F)
                       .Case("lp64d", ABI_LP64D)
                       .Case("ppt", ABI_PPT)
                       .Default(ABI_Unknown);

  bool IsRV64 = false; // TT.isArch64Bit();
  bool IsRV32E = false; // FeatureBits[PPU::FeatureRV32E];
  bool IsPPT = false; // FeatureBits[PPU::FeaturePPT];

  if (!ABIName.empty() && TargetABI == ABI_Unknown) {
    errs()
        << "'" << ABIName
        << "' is not a recognized ABI for this target (ignoring target-abi)\n";
  } else if (ABIName.startswith("ilp32") && IsRV64) {
    errs() << "32-bit ABIs are not supported for 64-bit targets (ignoring "
              "target-abi)\n";
    TargetABI = ABI_Unknown;
  } else if (ABIName.startswith("lp64") && !IsRV64) {
    errs() << "64-bit ABIs are not supported for 32-bit targets (ignoring "
              "target-abi)\n";
    TargetABI = ABI_Unknown;
  } else if (ABIName.endswith("f") && !FeatureBits[PPU::FeatureStdExtF]) {
    errs() << "Hard-float 'f' ABI can't be used for a target that "
              "doesn't support the F instruction set extension (ignoring "
              "target-abi)\n";
    TargetABI = ABI_Unknown;
  } else if (ABIName.endswith("d") && !FeatureBits[PPU::FeatureStdExtD]) {
    errs() << "Hard-float 'd' ABI can't be used for a target that "
              "doesn't support the D instruction set extension (ignoring "
              "target-abi)\n";
    TargetABI = ABI_Unknown;
  } else if (IsRV32E && TargetABI != ABI_ILP32E && TargetABI != ABI_Unknown) {
    errs()
        << "Only the ilp32e ABI is supported for RV32E (ignoring target-abi)\n";
    TargetABI = ABI_Unknown;
  }

  if (TargetABI != ABI_Unknown)
    return TargetABI;

  // For now, default to the ilp32/ilp32e/lp64 ABI if no explicit ABI is given
  // or an invalid/unrecognised string is given. In the future, it might be
  // worth changing this to default to ilp32f/lp64f and ilp32d/lp64d when
  // hardware support for floating point is present.
  if (IsRV32E)
    return ABI_ILP32E;
  if (IsRV64)
    return ABI_LP64;
  if (IsPPT)
    return ABI_PPT;
  return ABI_ILP32;
}
} // namespace PPUABI

namespace PPUFeatures {

void validate(const Triple &TT, const FeatureBitset &FeatureBits) {
  // if (TT.isArch64Bit() && FeatureBits[PPU::FeatureRV32E])
  //  report_fatal_error("RV32E can't be enabled for an RV64 target");
}

} // namespace PPUFeatures

namespace {

/// \returns Bit mask for given bit \p Shift and bit \p Width.
unsigned getBitMask(unsigned Shift, unsigned Width) {
  return ((1 << Width) - 1) << Shift;
}

/// Packs \p Src into \p Dst for given bit \p Shift and bit \p Width.
///
/// \returns Packed \p Dst.
unsigned packBits(unsigned Src, unsigned Dst, unsigned Shift, unsigned Width) {
  Dst &= ~(1 << Shift) & ~getBitMask(Shift, Width);
  Dst |= (Src << Shift) & getBitMask(Shift, Width);
  return Dst;
}

/// Unpacks bits from \p Src for given bit \p Shift and bit \p Width.
///
/// \returns Unpacked bits.
unsigned unpackBits(unsigned Src, unsigned Shift, unsigned Width) {
  return (Src & getBitMask(Shift, Width)) >> Shift;
}

/// \returns Vmcnt bit shift (lower bits).
unsigned getVmcntBitShiftLo() { return 0; }

/// \returns Vmcnt bit width (lower bits).
unsigned getVmcntBitWidthLo() { return 4; }

/// \returns Expcnt bit shift.
// unsigned getExpcntBitShift() { return 4; }

/// \returns Expcnt bit width.
// unsigned getExpcntBitWidth() { return 3; }

/// \returns Lgkmcnt bit shift.
unsigned getLgkmcntBitShift() { return 8; }

/// \returns Lgkmcnt bit width.
unsigned getLgkmcntBitWidth(unsigned VersionMajor) {
  // return (VersionMajor >= 10) ? 6 : 4;
  return 4;
}

/// \returns Vmcnt bit shift (higher bits).
unsigned getVmcntBitShiftHi() { return 14; }

/// \returns Vmcnt bit width (higher bits).
unsigned getVmcntBitWidthHi() { return 2; }

} // end namespace anonymous


namespace PPU {


struct MUBUFInfo {
  uint16_t Opcode;
  uint16_t BaseOpcode;
  uint8_t elements;
  bool has_vaddr;
  bool has_srsrc;
  bool has_soffset;
};

#define GET_MUBUFInfoTable_DECL
#define GET_MUBUFInfoTable_IMPL
#include "PPUGenSearchableTables.inc"
/*
int getMUBUFBaseOpcode(unsigned Opc) {
  const MUBUFInfo *Info = getMUBUFInfoFromOpcode(Opc);
  return Info ? Info->BaseOpcode : -1;
}

int getMUBUFOpcode(unsigned BaseOpc, unsigned Elements) {
  const MUBUFInfo *Info = getMUBUFInfoFromBaseOpcodeAndElements(BaseOpc, Elements);
  return Info ? Info->Opcode : -1;
}

int getMUBUFElements(unsigned Opc) {
  const MUBUFInfo *Info = getMUBUFOpcodeHelper(Opc);
  return Info ? Info->elements : 0;
}

bool getMUBUFHasVAddr(unsigned Opc) {
  const MUBUFInfo *Info = getMUBUFOpcodeHelper(Opc);
  return Info ? Info->has_vaddr : false;
}

bool getMUBUFHasSrsrc(unsigned Opc) {
  const MUBUFInfo *Info = getMUBUFOpcodeHelper(Opc);
  return Info ? Info->has_srsrc : false;
}

bool getMUBUFHasSoffset(unsigned Opc) {
  const MUBUFInfo *Info = getMUBUFOpcodeHelper(Opc);
  return Info ? Info->has_soffset : false;
}
*/

// Wrapper for Tablegen'd function.  enum Subtarget is not defined in any
// header files, so we need to wrap it in a function that takes unsigned
// instead.
int getMCOpcode(uint16_t Opcode, unsigned Gen) {
  // return getMCOpcodeGen(Opcode, static_cast<Subtarget>(Gen));
  return getMCOpcodeGen(Opcode);
}

namespace IsaInfo {

IsaVersion getIsaVersion(const MCSubtargetInfo *STI) {
    return {0, 0, 0};
}

IsaVersion getIsaVersion(const llvm::StringRef) {
    return {0, 0, 0};
}

void streamIsaVersion(const MCSubtargetInfo *STI, raw_ostream &Stream) {
  auto TargetTriple = STI->getTargetTriple();
  auto Version = getIsaVersion(STI->getCPU());

  Stream << TargetTriple.getArchName() << '-'
         << TargetTriple.getVendorName() << '-'
         << TargetTriple.getOSName() << '-'
         << TargetTriple.getEnvironmentName() << '-'
         << "gfx"
         << Version.Major
         << Version.Minor
         << Version.Stepping;

  // if (hasXNACK(*STI))
  //   Stream << "+xnack";
  // if (hasSRAMECC(*STI))
  //   Stream << "+sram-ecc";

  Stream.flush();
}

bool hasCodeObjectV3(const MCSubtargetInfo *STI) {
  return STI->getTargetTriple().getOS() == Triple::PPS &&
             STI->getFeatureBits().test(FeatureCodeObjectV3);
}

unsigned getWavefrontSize(const MCSubtargetInfo *STI) {
    return 32;
}

unsigned getLocalMemorySize(const MCSubtargetInfo *STI) {
  if (STI->getFeatureBits().test(FeatureLocalMemorySize32768))
    return 32768;
  if (STI->getFeatureBits().test(FeatureLocalMemorySize65536))
    return 65536;

  return 0;
}


unsigned getEUsPerCU(const MCSubtargetInfo *STI) {
  return 4;
}

unsigned getMaxWorkGroupsPerCU(const MCSubtargetInfo *STI,
                               unsigned FlatWorkGroupSize) {
  assert(FlatWorkGroupSize != 0);
  return 16;
  /*
  if (STI->getTargetTriple().getArch() != Triple::ppu)
    return 8;
  unsigned N = getWavesPerWorkGroup(STI, FlatWorkGroupSize);
  if (N == 1)
    return 40;
  N = 40 / N;
  return std::min(N, 16u);
  */
}

unsigned getMaxWavesPerCU(const MCSubtargetInfo *STI,
                          unsigned FlatWorkGroupSize) {
  return getWavesPerWorkGroup(STI, FlatWorkGroupSize);
}

unsigned getMinWavesPerEU(const MCSubtargetInfo *STI) {
  return 1;
}

unsigned getMaxWavesPerEU(const MCSubtargetInfo *STI) {
  // FIXME: Need to take scratch memory into account.
  /*
  if (!isGFX10(*STI))
    return 10;
    */
  return 20;
}

unsigned getMaxWavesPerEU(const MCSubtargetInfo *STI,
                          unsigned FlatWorkGroupSize) {
  return alignTo(getMaxWavesPerCU(STI, FlatWorkGroupSize),
                 getEUsPerCU(STI)) / getEUsPerCU(STI);
}

unsigned getMaxWavesPerCU(const MCSubtargetInfo *STI) {
  return getMaxWavesPerEU(STI) * getEUsPerCU(STI);
}


unsigned getMinFlatWorkGroupSize(const MCSubtargetInfo *STI) {
  return 1;
}

unsigned getMaxFlatWorkGroupSize(const MCSubtargetInfo *STI) {
  return 2048;
}

unsigned getWavesPerWorkGroup(const MCSubtargetInfo *STI,
                              unsigned FlatWorkGroupSize) {
  return alignTo(FlatWorkGroupSize, getWavefrontSize(STI)) /
                 getWavefrontSize(STI);
}

unsigned getSGPRAllocGranule(const MCSubtargetInfo *STI) {
    /*
  IsaVersion Version = getIsaVersion(STI->getCPU());
  if (Version.Major >= 10)
    return getAddressableNumSGPRs(STI);
  if (Version.Major >= 8)
    return 16;
    */
  return 8;
}

unsigned getSGPREncodingGranule(const MCSubtargetInfo *STI) {
  return 8;
}

unsigned getTotalNumSGPRs(const MCSubtargetInfo *STI) {
    /*
  IsaVersion Version = getIsaVersion(STI->getCPU());
  if (Version.Major >= 8)
    return 800;
  return 512;
  */
    return 32;
}

unsigned getAddressableNumSGPRs(const MCSubtargetInfo *STI) {
/*
  if (STI->getFeatureBits().test(FeatureSGPRInitBug))
    return FIXED_NUM_SGPRS_FOR_INIT_BUG;
  IsaVersion Version = getIsaVersion(STI->getCPU());
  if (Version.Major >= 10)
    return 106;
  if (Version.Major >= 8)
    return 102;
  return 104;
  */
  return 32;
}

unsigned getMinNumSGPRs(const MCSubtargetInfo *STI, unsigned WavesPerEU) {
  assert(WavesPerEU != 0);
/*
  IsaVersion Version = getIsaVersion(STI->getCPU());
  if (Version.Major >= 10)
    return 0;

  unsigned MinNumSGPRs = getTotalNumSGPRs(STI) / (WavesPerEU + 1);
  if (STI->getFeatureBits().test(FeatureTrapHandler))
    MinNumSGPRs -= std::min(MinNumSGPRs, (unsigned)TRAP_NUM_SGPRS);
  MinNumSGPRs = alignDown(MinNumSGPRs, getSGPRAllocGranule(STI)) + 1;
  return std::min(MinNumSGPRs, getAddressableNumSGPRs(STI));
  */
  return 0;
}

unsigned getMaxNumSGPRs(const MCSubtargetInfo *STI, unsigned WavesPerEU,
                        bool Addressable) {
  assert(WavesPerEU != 0);

  return 32;
/*
  unsigned AddressableNumSGPRs = getAddressableNumSGPRs(STI);
  IsaVersion Version = getIsaVersion(STI->getCPU());
  unsigned MaxNumSGPRs = getTotalNumSGPRs(STI) / WavesPerEU;
  if (STI->getFeatureBits().test(FeatureTrapHandler))
    MaxNumSGPRs -= std::min(MaxNumSGPRs, (unsigned)TRAP_NUM_SGPRS);
  MaxNumSGPRs = alignDown(MaxNumSGPRs, getSGPRAllocGranule(STI));
  return std::min(MaxNumSGPRs, AddressableNumSGPRs);
  */
}

unsigned getNumExtraSGPRs(const MCSubtargetInfo *STI, bool VCCUsed,
                          bool FlatScrUsed, bool XNACKUsed) {
  unsigned ExtraSGPRs = 0;
  if (VCCUsed)
    ExtraSGPRs = 2;

//  IsaVersion Version = getIsaVersion(STI->getCPU());
//  if (Version.Major >= 10)
    return ExtraSGPRs;
/*
  if (Version.Major < 8) {
    if (FlatScrUsed)
      ExtraSGPRs = 4;
  } else {
    if (XNACKUsed)
      ExtraSGPRs = 4;

    if (FlatScrUsed)
      ExtraSGPRs = 6;
  }
*/
  return ExtraSGPRs;
}

unsigned getNumExtraSGPRs(const MCSubtargetInfo *STI, bool VCCUsed,
                          bool FlatScrUsed) {
  return getNumExtraSGPRs(STI, VCCUsed, FlatScrUsed, false);
                          // STI->getFeatureBits().test(PPU::FeatureXNACK));
}

unsigned getNumSGPRBlocks(const MCSubtargetInfo *STI, unsigned NumSGPRs) {
  NumSGPRs = alignTo(std::max(1u, NumSGPRs), getSGPREncodingGranule(STI));
  // SGPRBlocks is actual number of SGPR blocks minus 1.
  return NumSGPRs / getSGPREncodingGranule(STI) - 1;
}

unsigned getVGPRAllocGranule(const MCSubtargetInfo *STI,
                             Optional<bool> EnableWavefrontSize32) {
  bool IsWave32 = true;
  return IsWave32 ? 8 : 4;
}

unsigned getVGPREncodingGranule(const MCSubtargetInfo *STI,
                                Optional<bool> EnableWavefrontSize32) {
  return getVGPRAllocGranule(STI, EnableWavefrontSize32);
}


unsigned getTotalNumVGPRs(const MCSubtargetInfo *STI) {
    /*
  if (!isGFX10(*STI))
    return 256;
  return STI->getFeatureBits().test(FeatureWavefrontSize32) ? 1024 : 512;
  */
    return 32;
}

unsigned getAddressableNumVGPRs(const MCSubtargetInfo *STI) {
  return 256;
}

unsigned getMinNumVGPRs(const MCSubtargetInfo *STI, unsigned WavesPerEU) {
    /*
  assert(WavesPerEU != 0);

  if (WavesPerEU >= getMaxWavesPerEU(STI))
    return 0;
  unsigned MinNumVGPRs =
      alignDown(getTotalNumVGPRs(STI) / (WavesPerEU + 1),
                getVGPRAllocGranule(STI)) + 1;
  return std::min(MinNumVGPRs, getAddressableNumVGPRs(STI));
  */
    return 32;
}

unsigned getMaxNumVGPRs(const MCSubtargetInfo *STI, unsigned WavesPerEU) {
  assert(WavesPerEU != 0);
/*
  unsigned MaxNumVGPRs = alignDown(getTotalNumVGPRs(STI) / WavesPerEU,
                                   getVGPRAllocGranule(STI));
  unsigned AddressableNumVGPRs = getAddressableNumVGPRs(STI);
  return std::min(MaxNumVGPRs, AddressableNumVGPRs);
  */
  return 32;
}

unsigned getNumVPRBlocks(const MCSubtargetInfo *STI, unsigned NumVGPRs,
                          Optional<bool> EnableWavefrontSize32) {
  NumVGPRs = alignTo(std::max(1u, NumVGPRs),
                     getVGPREncodingGranule(STI, EnableWavefrontSize32));
  // VGPRBlocks is actual number of VGPR blocks minus 1.
  return NumVGPRs / getVGPREncodingGranule(STI, EnableWavefrontSize32) - 1;
}


} // end namespace IsaInfo

void initDefaultPPUKernelCodeT(amd_kernel_code_t &Header,
                               const MCSubtargetInfo *STI) {
  // IsaVersion Version = getIsaVersion(STI->getCPU());
  IsaVersion Version = getIsaVersion(STI);

  memset(&Header, 0, sizeof(Header));

  Header.amd_kernel_code_version_major = 1;
  Header.amd_kernel_code_version_minor = 2;
  Header.amd_machine_kind = 1; // AMD_MACHINE_KIND_AMDGPU
  Header.amd_machine_version_major = Version.Major;
  Header.amd_machine_version_minor = Version.Minor;
  Header.amd_machine_version_stepping = Version.Stepping;
  Header.kernel_code_entry_byte_offset = sizeof(Header);
  Header.wavefront_size = 6;

  // If the code object does not support indirect functions, then the value must
  // be 0xffffffff.
  Header.call_convention = -1;

  // These alignment values are specified in powers of two, so alignment =
  // 2^n.  The minimum alignment is 2^4 = 16.
  Header.kernarg_segment_alignment = 4;
  Header.group_segment_alignment = 4;
  Header.private_segment_alignment = 4;
/*
  if (Version.Major >= 10) {
    if (STI->getFeatureBits().test(FeatureWavefrontSize32)) {
      Header.wavefront_size = 5;
      Header.code_properties |= AMD_CODE_PROPERTY_ENABLE_WAVEFRONT_SIZE32;
    }
    Header.compute_pgm_resource_registers |=
      S_00B848_WGP_MODE(STI->getFeatureBits().test(FeatureCuMode) ? 0 : 1) |
      S_00B848_MEM_ORDERED(1);
  }
  */
}

amdhsa::kernel_descriptor_t getDefaultPPUKernelDescriptor(
    const MCSubtargetInfo *STI) {
  IsaVersion Version = getIsaVersion(STI->getCPU());

  amdhsa::kernel_descriptor_t KD;
  memset(&KD, 0, sizeof(KD));
/*
  AMDHSA_BITS_SET(KD.compute_pgm_rsrc1,
                  amdhsa::COMPUTE_PGM_RSRC1_FLOAT_DENORM_MODE_16_64,
                  amdhsa::FLOAT_DENORM_MODE_FLUSH_NONE);
  AMDHSA_BITS_SET(KD.compute_pgm_rsrc1,
                  amdhsa::COMPUTE_PGM_RSRC1_ENABLE_DX10_CLAMP, 1);
  AMDHSA_BITS_SET(KD.compute_pgm_rsrc1,
                  amdhsa::COMPUTE_PGM_RSRC1_ENABLE_IEEE_MODE, 1);
  AMDHSA_BITS_SET(KD.compute_pgm_rsrc2,
                  amdhsa::COMPUTE_PGM_RSRC2_ENABLE_SGPR_WORKGROUP_ID_X, 1);
  if (Version.Major >= 10) {
    AMDHSA_BITS_SET(KD.kernel_code_properties,
                    amdhsa::KERNEL_CODE_PROPERTY_ENABLE_WAVEFRONT_SIZE32,
                    STI->getFeatureBits().test(FeatureWavefrontSize32) ? 1 : 0);
    AMDHSA_BITS_SET(KD.compute_pgm_rsrc1,
                    amdhsa::COMPUTE_PGM_RSRC1_WGP_MODE,
                    STI->getFeatureBits().test(FeatureCuMode) ? 0 : 1);
    AMDHSA_BITS_SET(KD.compute_pgm_rsrc1,
                    amdhsa::COMPUTE_PGM_RSRC1_MEM_ORDERED, 1);
  }
  */
  return KD;
}


bool isGroupSegment(const GlobalValue *GV) {
  return GV->getType()->getAddressSpace() == AMDGPUAS::LOCAL_ADDRESS;
}

bool isGlobalSegment(const GlobalValue *GV) {
  return GV->getType()->getAddressSpace() == AMDGPUAS::GLOBAL_ADDRESS;
}

bool isReadOnlySegment(const GlobalValue *GV) {
  return GV->getType()->getAddressSpace() == AMDGPUAS::CONSTANT_ADDRESS ||
         GV->getType()->getAddressSpace() == AMDGPUAS::CONSTANT_ADDRESS_32BIT;
}

bool shouldEmitConstantsToTextSection(const Triple &TT) {
  return TT.getOS() != Triple::PPS;
}

int getIntegerAttribute(const Function &F, StringRef Name, int Default) {
  Attribute A = F.getFnAttribute(Name);
  int Result = Default;

  if (A.isStringAttribute()) {
    StringRef Str = A.getValueAsString();
    if (Str.getAsInteger(0, Result)) {
      LLVMContext &Ctx = F.getContext();
      Ctx.emitError("can't parse integer attribute " + Name);
    }
  }

  return Result;
}

std::pair<int, int> getIntegerPairAttribute(const Function &F,
                                            StringRef Name,
                                            std::pair<int, int> Default,
                                            bool OnlyFirstRequired) {
  Attribute A = F.getFnAttribute(Name);
  if (!A.isStringAttribute())
    return Default;

  LLVMContext &Ctx = F.getContext();
  std::pair<int, int> Ints = Default;
  std::pair<StringRef, StringRef> Strs = A.getValueAsString().split(',');
  if (Strs.first.trim().getAsInteger(0, Ints.first)) {
    Ctx.emitError("can't parse first integer attribute " + Name);
    return Default;
  }
  if (Strs.second.trim().getAsInteger(0, Ints.second)) {
    if (!OnlyFirstRequired || !Strs.second.trim().empty()) {
      Ctx.emitError("can't parse second integer attribute " + Name);
      return Default;
    }
  }

  return Ints;
}

unsigned getVmcntBitMask(const IsaVersion &Version) {
  unsigned VmcntLo = (1 << getVmcntBitWidthLo()) - 1;
  // if (Version.Major < 9)
    return VmcntLo;

  // unsigned VmcntHi = ((1 << getVmcntBitWidthHi()) - 1) << getVmcntBitWidthLo();
  // return VmcntLo | VmcntHi;
}

// unsigned getExpcntBitMask(const IsaVersion &Version) {
//   return (1 << getExpcntBitWidth()) - 1;
// }

unsigned getLgkmcntBitMask(const IsaVersion &Version) {
  return (1 << getLgkmcntBitWidth(Version.Major)) - 1;
}

unsigned getWaitcntBitMask(const IsaVersion &Version) {
  unsigned VmcntLo = getBitMask(getVmcntBitShiftLo(), getVmcntBitWidthLo());
  // unsigned Expcnt = getBitMask(getExpcntBitShift(), getExpcntBitWidth());
  unsigned Lgkmcnt = getBitMask(getLgkmcntBitShift(),
                                getLgkmcntBitWidth(Version.Major));
  // unsigned Waitcnt = VmcntLo | Expcnt | Lgkmcnt;
  unsigned Waitcnt = VmcntLo | Lgkmcnt;
  //if (Version.Major < 9)
    return Waitcnt;

  // unsigned VmcntHi = getBitMask(getVmcntBitShiftHi(), getVmcntBitWidthHi());
  // return Waitcnt | VmcntHi;
}

unsigned decodeVmcnt(const IsaVersion &Version, unsigned Waitcnt) {
  unsigned VmcntLo =
      unpackBits(Waitcnt, getVmcntBitShiftLo(), getVmcntBitWidthLo());
  // if (Version.Major < 9)
    return VmcntLo;
/*
  unsigned VmcntHi =
      unpackBits(Waitcnt, getVmcntBitShiftHi(), getVmcntBitWidthHi());
  VmcntHi <<= getVmcntBitWidthLo();
  return VmcntLo | VmcntHi;
  */
}
/*
unsigned decodeExpcnt(const IsaVersion &Version, unsigned Waitcnt) {
  return unpackBits(Waitcnt, getExpcntBitShift(), getExpcntBitWidth());
}
*/

unsigned decodeLgkmcnt(const IsaVersion &Version, unsigned Waitcnt) {
  return unpackBits(Waitcnt, getLgkmcntBitShift(),
                    getLgkmcntBitWidth(Version.Major));
}

void decodeWaitcnt(const IsaVersion &Version, unsigned Waitcnt,
                   unsigned &Vmcnt, unsigned &Expcnt, unsigned &Lgkmcnt) {
  Vmcnt = decodeVmcnt(Version, Waitcnt);
  // Expcnt = decodeExpcnt(Version, Waitcnt);
  Lgkmcnt = decodeLgkmcnt(Version, Waitcnt);
}

Waitcnt decodeWaitcnt(const IsaVersion &Version, unsigned Encoded) {
  Waitcnt Decoded;
  Decoded.VmCnt = decodeVmcnt(Version, Encoded);
  // Decoded.ExpCnt = decodeExpcnt(Version, Encoded);
  Decoded.LgkmCnt = decodeLgkmcnt(Version, Encoded);
  return Decoded;
}

unsigned encodeVmcnt(const IsaVersion &Version, unsigned Waitcnt,
                     unsigned Vmcnt) {
  Waitcnt =
      packBits(Vmcnt, Waitcnt, getVmcntBitShiftLo(), getVmcntBitWidthLo());
  // if (Version.Major < 9)
    return Waitcnt;
/*
  Vmcnt >>= getVmcntBitWidthLo();
  return packBits(Vmcnt, Waitcnt, getVmcntBitShiftHi(), getVmcntBitWidthHi());
  */
}
/*
unsigned encodeExpcnt(const IsaVersion &Version, unsigned Waitcnt,
                      unsigned Expcnt) {
  return packBits(Expcnt, Waitcnt, getExpcntBitShift(), getExpcntBitWidth());
}
*/

unsigned encodeLgkmcnt(const IsaVersion &Version, unsigned Waitcnt,
                       unsigned Lgkmcnt) {
  return packBits(Lgkmcnt, Waitcnt, getLgkmcntBitShift(),
                                    getLgkmcntBitWidth(Version.Major));
}

unsigned encodeWaitcnt(const IsaVersion &Version,
                       unsigned Vmcnt, unsigned Expcnt, unsigned Lgkmcnt) {
  unsigned Waitcnt = getWaitcntBitMask(Version);
  Waitcnt = encodeVmcnt(Version, Waitcnt, Vmcnt);
  // Waitcnt = encodeExpcnt(Version, Waitcnt, Expcnt);
  Waitcnt = encodeLgkmcnt(Version, Waitcnt, Lgkmcnt);
  return Waitcnt;
}

unsigned encodeWaitcnt(const IsaVersion &Version, const Waitcnt &Decoded) {
  return encodeWaitcnt(Version, Decoded.VmCnt, Decoded.ExpCnt, Decoded.LgkmCnt);
}

//===----------------------------------------------------------------------===//
// hwreg
//===----------------------------------------------------------------------===//
namespace Hwreg {

int64_t getHwregId(const StringRef Name) {
  for (int Id = ID_SYMBOLIC_FIRST_; Id < ID_SYMBOLIC_LAST_; ++Id) {
    if (IdSymbolic[Id] && Name == IdSymbolic[Id])
      return Id;
  }
  return ID_UNKNOWN_;
}

static unsigned getLastSymbolicHwreg(const MCSubtargetInfo &STI) {
    /*
  if (isSI(STI) || isCI(STI) || isVI(STI))
    return ID_SYMBOLIC_FIRST_GFX9_;
  else if (isGFX9(STI))
    return ID_SYMBOLIC_FIRST_GFX10_;
  else
  */
    return ID_SYMBOLIC_LAST_;
}

bool isValidHwreg(int64_t Id, const MCSubtargetInfo &STI) {
  return ID_SYMBOLIC_FIRST_ <= Id && Id < getLastSymbolicHwreg(STI) &&
         IdSymbolic[Id];
}

bool isValidHwreg(int64_t Id) {
  return 0 <= Id && isUInt<ID_WIDTH_>(Id);
}

bool isValidHwregOffset(int64_t Offset) {
  return 0 <= Offset && isUInt<OFFSET_WIDTH_>(Offset);
}

bool isValidHwregWidth(int64_t Width) {
  return 0 <= (Width - 1) && isUInt<WIDTH_M1_WIDTH_>(Width - 1);
}

uint64_t encodeHwreg(uint64_t Id, uint64_t Offset, uint64_t Width) {
  return (Id << ID_SHIFT_) |
         (Offset << OFFSET_SHIFT_) |
         ((Width - 1) << WIDTH_M1_SHIFT_);
}

StringRef getHwreg(unsigned Id, const MCSubtargetInfo &STI) {
   return isValidHwreg(Id, STI) ? IdSymbolic[Id] : "";
}

void decodeHwreg(unsigned Val, unsigned &Id, unsigned &Offset, unsigned &Width) {
  Id = (Val & ID_MASK_) >> ID_SHIFT_;
  Offset = (Val & OFFSET_MASK_) >> OFFSET_SHIFT_;
  Width = ((Val & WIDTH_M1_MASK_) >> WIDTH_M1_SHIFT_) + 1;
}

} // namespace Hwreg

//===----------------------------------------------------------------------===//
// SendMsg
//===----------------------------------------------------------------------===//
/*
namespace SendMsg {

int64_t getMsgId(const StringRef Name) {
  for (int i = ID_GAPS_FIRST_; i < ID_GAPS_LAST_; ++i) {
    if (IdSymbolic[i] && Name == IdSymbolic[i])
      return i;
  }
  return ID_UNKNOWN_;
}

static bool isValidMsgId(int64_t MsgId) {
  return (ID_GAPS_FIRST_ <= MsgId && MsgId < ID_GAPS_LAST_) && IdSymbolic[MsgId];
}

bool isValidMsgId(int64_t MsgId, const MCSubtargetInfo &STI, bool Strict) {
  if (Strict) {
    if (MsgId == ID_GS_ALLOC_REQ || MsgId == ID_GET_DOORBELL)
      return isGFX9(STI) || isGFX10(STI);
    else
      return isValidMsgId(MsgId);
  } else {
    return 0 <= MsgId && isUInt<ID_WIDTH_>(MsgId);
  }
}

StringRef getMsgName(int64_t MsgId) {
  return isValidMsgId(MsgId)? IdSymbolic[MsgId] : "";
}

int64_t getMsgOpId(int64_t MsgId, const StringRef Name) {
  const char* const *S = (MsgId == ID_SYSMSG) ? OpSysSymbolic : OpGsSymbolic;
  const int F = (MsgId == ID_SYSMSG) ? OP_SYS_FIRST_ : OP_GS_FIRST_;
  const int L = (MsgId == ID_SYSMSG) ? OP_SYS_LAST_ : OP_GS_LAST_;
  for (int i = F; i < L; ++i) {
    if (Name == S[i]) {
      return i;
    }
  }
  return OP_UNKNOWN_;
}

bool isValidMsgOp(int64_t MsgId, int64_t OpId, bool Strict) {

  if (!Strict)
    return 0 <= OpId && isUInt<OP_WIDTH_>(OpId);

  switch(MsgId)
  {
  case ID_GS:
    return (OP_GS_FIRST_ <= OpId && OpId < OP_GS_LAST_) && OpId != OP_GS_NOP;
  case ID_GS_DONE:
    return OP_GS_FIRST_ <= OpId && OpId < OP_GS_LAST_;
  case ID_SYSMSG:
    return OP_SYS_FIRST_ <= OpId && OpId < OP_SYS_LAST_;
  default:
    return OpId == OP_NONE_;
  }
}

StringRef getMsgOpName(int64_t MsgId, int64_t OpId) {
  assert(msgRequiresOp(MsgId));
  return (MsgId == ID_SYSMSG)? OpSysSymbolic[OpId] : OpGsSymbolic[OpId];
}

bool isValidMsgStream(int64_t MsgId, int64_t OpId, int64_t StreamId, bool Strict) {

  if (!Strict)
    return 0 <= StreamId && isUInt<STREAM_ID_WIDTH_>(StreamId);

  switch(MsgId)
  {
  case ID_GS:
    return STREAM_ID_FIRST_ <= StreamId && StreamId < STREAM_ID_LAST_;
  case ID_GS_DONE:
    return (OpId == OP_GS_NOP)?
           (StreamId == STREAM_ID_NONE_) :
           (STREAM_ID_FIRST_ <= StreamId && StreamId < STREAM_ID_LAST_);
  default:
    return StreamId == STREAM_ID_NONE_;
  }
}

bool msgRequiresOp(int64_t MsgId) {
  return MsgId == ID_GS || MsgId == ID_GS_DONE || MsgId == ID_SYSMSG;
}

bool msgSupportsStream(int64_t MsgId, int64_t OpId) {
  return (MsgId == ID_GS || MsgId == ID_GS_DONE) && OpId != OP_GS_NOP;
}

void decodeMsg(unsigned Val,
               uint16_t &MsgId,
               uint16_t &OpId,
               uint16_t &StreamId) {
  MsgId = Val & ID_MASK_;
  OpId = (Val & OP_MASK_) >> OP_SHIFT_;
  StreamId = (Val & STREAM_ID_MASK_) >> STREAM_ID_SHIFT_;
}

uint64_t encodeMsg(uint64_t MsgId,
                   uint64_t OpId,
                   uint64_t StreamId) {
  return (MsgId << ID_SHIFT_) |
         (OpId << OP_SHIFT_) |
         (StreamId << STREAM_ID_SHIFT_);
}

} // namespace SendMsg
*/


bool isShader(CallingConv::ID CC) {
  switch (CC) {
   case CallingConv::AMDGPU_CS:
    return true;
  default:
    return false;
  }
}


bool isCompute(CallingConv::ID CC) {
  switch (CC) {
  case CallingConv::AMDGPU_KERNEL:
  case CallingConv::SPIR_KERNEL:
  case CallingConv::AMDGPU_CS:
    return true;
  default:
    return false;
  }
}

bool isCompute(SelectionDAG *DAG) {
  return isCompute(DAG->getMachineFunction().getFunction().getCallingConv());
}

bool isCompute(MachineFunction *MF) {
  return isCompute(MF->getFunction().getCallingConv());
}

bool isCompute(const MachineFunction *MF) {
  return isCompute(const_cast<MachineFunction*>(MF));
}

bool isEntryFunctionCC(CallingConv::ID CC) {
  switch (CC) {
  case CallingConv::AMDGPU_KERNEL:
  case CallingConv::SPIR_KERNEL:
    return true;
  default:
    return false;
  }
}

bool isKernel(CallingConv::ID CC) {
  switch (CC) {
  case CallingConv::AMDGPU_KERNEL:
  case CallingConv::SPIR_KERNEL:
  // case CallingConv::AMDGPU_CS:
    return true;
  default:
    return false;
  }
}

bool isSGPR(unsigned Reg, const MCRegisterInfo* TRI) {
  const MCRegisterClass SGPRClass = TRI->getRegClass(PPU::SReg_32RegClassID);
  const unsigned FirstSubReg = TRI->getSubReg(Reg, 1);
  return SGPRClass.contains(FirstSubReg != 0 ? FirstSubReg : Reg);
    /* FIXME
  const MCRegisterClass SGPRClass = TRI->getRegClass(PPU::Reg_32RegClassID);
  const unsigned FirstSubReg = TRI->getSubReg(Reg, 1);
  return SGPRClass.contains(FirstSubReg != 0 ? FirstSubReg : Reg) ||
    Reg == PPU::SCC;
    */
}

bool isRegIntersect(unsigned Reg0, unsigned Reg1, const MCRegisterInfo* TRI) {
  for (MCRegAliasIterator R(Reg0, TRI, true); R.isValid(); ++R) {
    if (*R == Reg1) return true;
  }
  return false;
}

#define CASE(node) \
  case node: return node;


#define MAP_REG2REG \
  using namespace PPU; \
  switch(Reg) { \
  default: return Reg; \
  CASE(FLAT_SCR) \
  }

unsigned getMCReg(unsigned Reg, const MCSubtargetInfo &STI) {
  MAP_REG2REG
}

unsigned mc2PseudoReg(unsigned Reg) {
  MAP_REG2REG
}

bool isSISrcOperand(const MCInstrDesc &Desc, unsigned OpNo) {
  assert(OpNo < Desc.NumOperands);
  unsigned OpType = Desc.OpInfo[OpNo].OperandType;
  return OpType >= PPU::OPERAND_SRC_FIRST &&
         OpType <= PPU::OPERAND_SRC_LAST;
}

bool isSISrcFPOperand(const MCInstrDesc &Desc, unsigned OpNo) {
  assert(OpNo < Desc.NumOperands);
  unsigned OpType = Desc.OpInfo[OpNo].OperandType;
  switch (OpType) {
  case PPU::OPERAND_REG_IMM_FP32:
  case PPU::OPERAND_REG_IMM_FP64:
  case PPU::OPERAND_REG_IMM_FP16:
  case PPU::OPERAND_REG_IMM_V2FP16:
  case PPU::OPERAND_REG_IMM_V2INT16:
  case PPU::OPERAND_REG_INLINE_C_FP32:
  case PPU::OPERAND_REG_INLINE_C_FP64:
  case PPU::OPERAND_REG_INLINE_C_FP16:
  case PPU::OPERAND_REG_INLINE_C_V2FP16:
  case PPU::OPERAND_REG_INLINE_C_V2INT16:
  case PPU::OPERAND_REG_INLINE_AC_FP32:
  case PPU::OPERAND_REG_INLINE_AC_FP16:
  case PPU::OPERAND_REG_INLINE_AC_V2FP16:
  case PPU::OPERAND_REG_INLINE_AC_V2INT16:
    return true;
  default:
    return false;
  }
}

bool isSISrcInlinableOperand(const MCInstrDesc &Desc, unsigned OpNo) {
  assert(OpNo < Desc.NumOperands);
  unsigned OpType = Desc.OpInfo[OpNo].OperandType;
  return OpType >= PPU::OPERAND_REG_INLINE_C_FIRST &&
         OpType <= PPU::OPERAND_REG_INLINE_C_LAST;
}

// Avoid using MCRegisterClass::getSize, since that function will go away
// (move from MC* level to Target* level). Return size in bits.
unsigned getRegBitWidth(unsigned RCID) {
  switch (RCID) {
  case PPU::SPR_32RegClassID:
  case PPU::VPR_32RegClassID:
  case PPU::VRegOrLds_32RegClassID:
  case PPU::VS_32RegClassID:
  case PPU::SReg_32RegClassID:
  case PPU::SRegOrLds_32RegClassID:
    return 32;
  case PPU::SPR_64RegClassID:
  case PPU::VS_64RegClassID:
  case PPU::SReg_64RegClassID:
  case PPU::VReg_64RegClassID:
  // case PPU::AReg_64RegClassID:
    return 64;
  default:
    llvm_unreachable("Unexpected register class");
  }
}

unsigned getRegBitWidth(const MCRegisterClass &RC) {
  return getRegBitWidth(RC.getID());
}

unsigned getRegOperandSize(const MCRegisterInfo *MRI, const MCInstrDesc &Desc,
                           unsigned OpNo) {
  assert(OpNo < Desc.NumOperands);
  unsigned RCID = Desc.OpInfo[OpNo].RegClass;
  return getRegBitWidth(MRI->getRegClass(RCID)) / 8;
}

bool isInlinableLiteral64(int64_t Literal, bool HasInv2Pi) {
  if (Literal >= -16 && Literal <= 64)
    return true;

  uint64_t Val = static_cast<uint64_t>(Literal);
  return (Val == DoubleToBits(0.0)) ||
         (Val == DoubleToBits(1.0)) ||
         (Val == DoubleToBits(-1.0)) ||
         (Val == DoubleToBits(0.5)) ||
         (Val == DoubleToBits(-0.5)) ||
         (Val == DoubleToBits(2.0)) ||
         (Val == DoubleToBits(-2.0)) ||
         (Val == DoubleToBits(4.0)) ||
         (Val == DoubleToBits(-4.0)) ||
         (Val == 0x3fc45f306dc9c882 && HasInv2Pi);
}

bool isInlinableLiteral32(int32_t Literal, bool HasInv2Pi) {
  if (Literal >= -16 && Literal <= 64)
    return true;

  // The actual type of the operand does not seem to matter as long
  // as the bits match one of the inline immediate values.  For example:
  //
  // -nan has the hexadecimal encoding of 0xfffffffe which is -2 in decimal,
  // so it is a legal inline immediate.
  //
  // 1065353216 has the hexadecimal encoding 0x3f800000 which is 1.0f in
  // floating-point, so it is a legal inline immediate.

  uint32_t Val = static_cast<uint32_t>(Literal);
  return (Val == FloatToBits(0.0f)) ||
         (Val == FloatToBits(1.0f)) ||
         (Val == FloatToBits(-1.0f)) ||
         (Val == FloatToBits(0.5f)) ||
         (Val == FloatToBits(-0.5f)) ||
         (Val == FloatToBits(2.0f)) ||
         (Val == FloatToBits(-2.0f)) ||
         (Val == FloatToBits(4.0f)) ||
         (Val == FloatToBits(-4.0f)) ||
         (Val == 0x3e22f983 && HasInv2Pi);
}

bool isInlinableLiteral16(int16_t Literal, bool HasInv2Pi) {
  if (!HasInv2Pi)
    return false;

  if (Literal >= -16 && Literal <= 64)
    return true;

  uint16_t Val = static_cast<uint16_t>(Literal);
  return Val == 0x3C00 || // 1.0
         Val == 0xBC00 || // -1.0
         Val == 0x3800 || // 0.5
         Val == 0xB800 || // -0.5
         Val == 0x4000 || // 2.0
         Val == 0xC000 || // -2.0
         Val == 0x4400 || // 4.0
         Val == 0xC400 || // -4.0
         Val == 0x3118;   // 1/2pi
}

bool isInlinableLiteralV216(int32_t Literal, bool HasInv2Pi) {
  assert(HasInv2Pi);

  if (isInt<16>(Literal) || isUInt<16>(Literal)) {
    int16_t Trunc = static_cast<int16_t>(Literal);
    return PPU::isInlinableLiteral16(Trunc, HasInv2Pi);
  }
  if (!(Literal & 0xffff))
    return PPU::isInlinableLiteral16(Literal >> 16, HasInv2Pi);

  int16_t Lo16 = static_cast<int16_t>(Literal);
  int16_t Hi16 = static_cast<int16_t>(Literal >> 16);
  return Lo16 == Hi16 && isInlinableLiteral16(Lo16, HasInv2Pi);
}












bool isArgPassedInSGPR(const Argument *A) {
  const Function *F = A->getParent();

  // Arguments to compute shaders are never a source of divergence.
  CallingConv::ID CC = F->getCallingConv();
  switch (CC) {
  case CallingConv::AMDGPU_KERNEL:
  case CallingConv::SPIR_KERNEL:
    return true;
  case CallingConv::AMDGPU_CS:
    // For non-compute shaders, SGPR inputs are marked with either inreg or byval.
    // Everything else is in VGPRs.
    return F->getAttributes().hasParamAttribute(A->getArgNo(), Attribute::InReg) ||
           F->getAttributes().hasParamAttribute(A->getArgNo(), Attribute::ByVal);
  default:
    // TODO: Should calls support inreg for SGPR inputs?
    return false;
  }
}

static bool hasSMEMByteOffset(const MCSubtargetInfo &ST) {
  // return isGCN3Encoding(ST) || isGFX10(ST);
  return true;
}

int64_t getSMRDEncodedOffset(const MCSubtargetInfo &ST, int64_t ByteOffset) {
  if (hasSMEMByteOffset(ST))
    return ByteOffset;
  return ByteOffset >> 2;
}

// FIXME
bool isLegalSMRDImmOffset(const MCSubtargetInfo &ST, int64_t ByteOffset) {
  int64_t EncodedOffset = getSMRDEncodedOffset(ST, ByteOffset);
  return (hasSMEMByteOffset(ST)) ?
    isUInt<20>(EncodedOffset) : isUInt<8>(EncodedOffset);
}

// Given Imm, split it into the values to put into the SOffset and ImmOffset
// fields in an MUBUF instruction. Return false if it is not possible (due to a
// hardware bug needing a workaround).
//
// The required alignment ensures that individual address components remain
// aligned if they are aligned to begin with. It also ensures that additional
// offsets within the given alignment can be added to the resulting ImmOffset.
bool splitMUBUFOffset(uint32_t Imm, uint32_t &SOffset, uint32_t &ImmOffset,
                      const PPUSubtarget *Subtarget, uint32_t Align) {
  const uint32_t MaxImm = alignDown(4095, Align);
  uint32_t Overflow = 0;

  if (Imm > MaxImm) {
    if (Imm <= MaxImm + 64) {
      // Use an SOffset inline constant for 4..64
      Overflow = Imm - MaxImm;
      Imm = MaxImm;
    } else {
      // Try to keep the same value in SOffset for adjacent loads, so that
      // the corresponding register contents can be re-used.
      //
      // Load values with all low-bits (except for alignment bits) set into
      // SOffset, so that a larger range of values can be covered using
      // s_movk_i32.
      //
      // Atomic operations fail to work correctly when individual address
      // components are unaligned, even if their sum is aligned.
      uint32_t High = (Imm + Align) & ~4095;
      uint32_t Low = (Imm + Align) & 4095;
      Imm = Low;
      Overflow = High - Align;
    }
  }

  // There is a hardware bug in SI and CI which prevents address clamping in
  // MUBUF instructions from working correctly with SOffsets. The immediate
  // offset is unaffected.
  /*
  if (Overflow > 0 &&
      Subtarget->getGeneration() <= AMDGPUSubtarget::SEA_ISLANDS)
    return false;
    */

  ImmOffset = Imm;
  SOffset = Overflow;
  return true;
}


PPUModeRegisterDefaults::PPUModeRegisterDefaults(const Function &F) {
  *this = getDefaultForCallingConv(F.getCallingConv());

  StringRef IEEEAttr = F.getFnAttribute("ppu-ieee").getValueAsString();
  if (!IEEEAttr.empty())
    IEEE = IEEEAttr == "true";

  StringRef DX10ClampAttr
    = F.getFnAttribute("ppu-dx10-clamp").getValueAsString();
  if (!DX10ClampAttr.empty())
    DX10Clamp = DX10ClampAttr == "true";
}

namespace {

struct SourceOfDivergence {
  unsigned Intr;
};
const SourceOfDivergence *lookupSourceOfDivergence(unsigned Intr);
/* TODO
#define GET_SourcesOfDivergence_IMPL
#include "PPUGenSearchableTables.inc"
*/
constexpr SourceOfDivergence SourcesOfDivergence[] = {
    /*
  { Intrinsic::ppu_nvvm_laneid },
  { Intrinsic::ppu_nvvm_tid_x },
  { Intrinsic::ppu_nvvm_tid_y },
  { Intrinsic::ppu_nvvm_tid_z },
  */
  { Intrinsic::ppu_workitem_id_x },
  { Intrinsic::ppu_workitem_id_y },
  { Intrinsic::ppu_workitem_id_z },
};

const SourceOfDivergence *lookupSourceOfDivergence(unsigned Intr) {
  struct KeyType {
    unsigned Intr;
  };
  KeyType Key = { Intr };
  auto Table = makeArrayRef(SourcesOfDivergence);
  auto Idx = std::lower_bound(Table.begin(), Table.end(), Key,
       [](const SourceOfDivergence &LHS, const KeyType &RHS) {
         if (LHS.Intr < RHS.Intr)
           return true;
         if (LHS.Intr > RHS.Intr)
           return false;
         return false;
       });

  if (Idx == Table.end() || Key.Intr != Idx->Intr)
    return nullptr;
  return &*Idx;
}

} // end anonymous namespace

bool isIntrinsicSourceOfDivergence(unsigned IntrID) {
  return lookupSourceOfDivergence(IntrID);
}

} // namespace PPU

} // namespace llvm
