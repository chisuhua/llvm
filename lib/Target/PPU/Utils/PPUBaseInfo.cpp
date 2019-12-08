#include "PPUBaseInfo.h"
#include "PPUTargetTransformInfo.h"
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
#include "PPUGenInstrInfo.inc"

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
                       .Default(ABI_Unknown);

  bool IsRV64 = false; // TT.isArch64Bit();
  bool IsRV32E = false; // FeatureBits[PPU::FeatureRV32E];

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
  return ABI_ILP32;
}
} // namespace PPUABI

namespace PPUFeatures {

void validate(const Triple &TT, const FeatureBitset &FeatureBits) {
  // if (TT.isArch64Bit() && FeatureBits[PPU::FeatureRV32E])
  //  report_fatal_error("RV32E can't be enabled for an RV64 target");
}

} // namespace PPUFeatures

namespace PPU {

namespace IsaInfo {
/* FIXME
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

  if (hasXNACK(*STI))
    Stream << "+xnack";
  if (hasSRAMECC(*STI))
    Stream << "+sram-ecc";

  Stream.flush();
}

bool hasCodeObjectV3(const MCSubtargetInfo *STI) {
  return STI->getTargetTriple().getOS() == Triple::AMDHSA &&
             STI->getFeatureBits().test(FeatureCodeObjectV3);
}
*/

unsigned getWavefrontSize(const MCSubtargetInfo *STI) {
    return 32;
}
/*
unsigned getLocalMemorySize(const MCSubtargetInfo *STI) {
  if (STI->getFeatureBits().test(FeatureLocalMemorySize32768))
    return 32768;
  if (STI->getFeatureBits().test(FeatureLocalMemorySize65536))
    return 65536;

  return 0;
}

*/
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

/*
unsigned getNumExtraSGPRs(const MCSubtargetInfo *STI, bool VCCUsed,
                          bool FlatScrUsed, bool XNACKUsed) {
  unsigned ExtraSGPRs = 0;
  if (VCCUsed)
    ExtraSGPRs = 2;

  IsaVersion Version = getIsaVersion(STI->getCPU());
  if (Version.Major >= 10)
    return ExtraSGPRs;

  if (Version.Major < 8) {
    if (FlatScrUsed)
      ExtraSGPRs = 4;
  } else {
    if (XNACKUsed)
      ExtraSGPRs = 4;

    if (FlatScrUsed)
      ExtraSGPRs = 6;
  }

  return ExtraSGPRs;
}

unsigned getNumExtraSGPRs(const MCSubtargetInfo *STI, bool VCCUsed,
                          bool FlatScrUsed) {
  return getNumExtraSGPRs(STI, VCCUsed, FlatScrUsed,
                          STI->getFeatureBits().test(AMDGPU::FeatureXNACK));
}

unsigned getNumSGPRBlocks(const MCSubtargetInfo *STI, unsigned NumSGPRs) {
  NumSGPRs = alignTo(std::max(1u, NumSGPRs), getSGPREncodingGranule(STI));
  // SGPRBlocks is actual number of SGPR blocks minus 1.
  return NumSGPRs / getSGPREncodingGranule(STI) - 1;
}
*/

unsigned getVGPRAllocGranule(const MCSubtargetInfo *STI,
                             Optional<bool> EnableWavefrontSize32) {
  bool IsWave32 = true;
  return IsWave32 ? 8 : 4;
}
/*
unsigned getVGPREncodingGranule(const MCSubtargetInfo *STI,
                                Optional<bool> EnableWavefrontSize32) {
  return getVGPRAllocGranule(STI, EnableWavefrontSize32);
}
*/

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
/*
unsigned getNumVGPRBlocks(const MCSubtargetInfo *STI, unsigned NumVGPRs,
                          Optional<bool> EnableWavefrontSize32) {
  NumVGPRs = alignTo(std::max(1u, NumVGPRs),
                     getVGPREncodingGranule(STI, EnableWavefrontSize32));
  // VGPRBlocks is actual number of VGPR blocks minus 1.
  return NumVGPRs / getVGPREncodingGranule(STI, EnableWavefrontSize32) - 1;
}
*/

} // end namespace IsaInfo



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

bool isCompute(SelectionDAG &DAG) {
  return isCompute(DAG.getMachineFunction().getFunction().getCallingConv());
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

bool isSGPR(unsigned Reg, const MCRegisterInfo* TRI) {
  const MCRegisterClass SGPRClass = TRI->getRegClass(PPU::SReg_32RegClassID);
  const unsigned FirstSubReg = TRI->getSubReg(Reg, 1);
  return SGPRClass.contains(FirstSubReg != 0 ? FirstSubReg : Reg);
    /* FIXME
  const MCRegisterClass SGPRClass = TRI->getRegClass(PPU::Reg_32RegClassID);
  const unsigned FirstSubReg = TRI->getSubReg(Reg, 1);
  return SGPRClass.contains(FirstSubReg != 0 ? FirstSubReg : Reg) ||
    Reg == AMDGPU::SCC;
    */
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
