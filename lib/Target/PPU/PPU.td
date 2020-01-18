//===-- PPU.td - Describe the PPU Target Machine -------*- tablegen -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

include "llvm/Target/Target.td"

include "PPUFeatures.td"

//===----------------------------------------------------------------------===//
// PPU subtarget features and instruction predicates.
//===----------------------------------------------------------------------===//

def FeatureStdExtM
    : SubtargetFeature<"m", "HasStdExtM", "true",
                       "'M' (Integer Multiplication and Division)">;
def HasStdExtM : Predicate<"Subtarget->hasStdExtM()">,
                           AssemblerPredicate<"FeatureStdExtM">;

def FeatureStdExtA
    : SubtargetFeature<"a", "HasStdExtA", "true",
                       "'A' (Atomic Instructions)">;
def HasStdExtA : Predicate<"Subtarget->hasStdExtA()">,
                           AssemblerPredicate<"FeatureStdExtA">;

def FeatureStdExtF
    : SubtargetFeature<"f", "HasStdExtF", "true",
                       "'F' (Single-Precision Floating-Point)">;
def HasStdExtF : Predicate<"Subtarget->hasStdExtF()">,
                           AssemblerPredicate<"FeatureStdExtF">;

def FeatureStdExtD
    : SubtargetFeature<"d", "HasStdExtD", "true",
                       "'D' (Double-Precision Floating-Point)",
                       [FeatureStdExtF]>;
def HasStdExtD : Predicate<"Subtarget->hasStdExtD()">,
                           AssemblerPredicate<"FeatureStdExtD">;

def FeatureStdExtC
    : SubtargetFeature<"c", "HasStdExtC", "true",
                       "'C' (Compressed Instructions)">;
def HasStdExtC : Predicate<"Subtarget->hasStdExtC()">,
                           AssemblerPredicate<"FeatureStdExtC">;

def FeatureRVCHints
    : SubtargetFeature<"rvc-hints", "EnableRVCHintInstrs", "true",
                       "Enable RVC Hint Instructions.">;
def HasRVCHints : Predicate<"Subtarget->enableRVCHintInstrs()">,
                            AssemblerPredicate<"FeatureRVCHints">;

def FeatureStdExtV
    : SubtargetFeature<"v", "HasStdExtV", "true",
                       "'V' (Vector Operations)">;
def HasStdExtV : Predicate<"Subtarget->hasStdExtV()">,
                           AssemblerPredicate<"FeatureStdExtV">;


def Feature64Bit
    : SubtargetFeature<"64bit", "HasRV64", "true", "Implements RV64">;
def IsRV64 : Predicate<"Subtarget->is64Bit()">,
                       AssemblerPredicate<"Feature64Bit">;
def IsRV32 : Predicate<"!Subtarget->is64Bit()">,
                       AssemblerPredicate<"!Feature64Bit">;

def RV64           : HwMode<"+64bit">;
def RV32           : HwMode<"-64bit">;

def FeatureRV32E
    : SubtargetFeature<"e", "IsRV32E", "true",
                       "Implements RV32E (provides 16 rather than 32 GPRs)">;
def IsRV32E : Predicate<"Subtarget->isRV32E()">,
                        AssemblerPredicate<"FeatureRV32E">;

def FeatureRelax
    : SubtargetFeature<"relax", "EnableLinkerRelax", "true",
                       "Enable Linker relaxation.">;



def FeaturePPT
    : SubtargetFeature<"t", "IsPPT", "true",
                       "PPT enable.">;

def IsPPT : Predicate<"Subtarget->isPPT()">,
                       AssemblerPredicate<"FeaturePPT">;
def IsNotPPT : Predicate<"!Subtarget->isPPT()">,
                       AssemblerPredicate<"!FeaturePPT">;

// from AMD
def FeatureGFX10Insts : SubtargetFeature<"gfx10-insts",
  "GFX10Insts",
  "true",
  "Additional instructions for GFX10+"
>;
def Feature16BitInsts : SubtargetFeature<"16-bit-insts",
  "Has16BitInsts",
  "true",
  "Has i16/f16 instructions"
>;
def FeatureAddNoCarryInsts : SubtargetFeature<"add-no-carry-insts",
  "AddNoCarryInsts",
  "true",
  "Have VALU add/sub instructions without carry out"
>;
def FeaturePkFmacF16Inst : SubtargetFeature<"pk-fmac-f16-inst",
  "HasPkFmacF16Inst",
  "true",
  "Has v_pk_fmac_f16 instruction"
>;

def FeatureAtomicFaddInsts : SubtargetFeature<"atomic-fadd-insts",
  "HasAtomicFaddInsts",
  "true",
  "Has buffer_atomic_add_f32, buffer_atomic_pk_add_f16, global_atomic_add_f32, "
  "global_atomic_pk_add_f16 instructions"
>;

def FeatureDLInsts : SubtargetFeature<"dl-insts",
  "HasDLInsts",
  "true",
  "Has v_fmac_f32 and v_xnor_b32 instructions"
>;

def FeatureDot1Insts : SubtargetFeature<"dot1-insts",
  "HasDot1Insts",
  "true",
  "Has v_dot4_i32_i8 and v_dot8_i32_i4 instructions"
>;
def FeatureDot2Insts : SubtargetFeature<"dot2-insts",
  "HasDot2Insts",
  "true",
  "Has v_dot2_f32_f16, v_dot2_i32_i16, v_dot2_u32_u16, v_dot4_u32_u8, v_dot8_u32_u4 instructions"
>;
def FeatureGFX9Insts : SubtargetFeature<"gfx9-insts",
  "GFX9Insts",
  "true",
  "Additional instructions for GFX9+"
>;

def FeatureInv2PiInlineImm : SubtargetFeature<"inv-2pi-inline-imm",
  "HasInv2PiInlineImm",
  "true",
  "Has 1 / (2 * pi) as inline immediate"
>;
def FeatureVOP3Literal : SubtargetFeature<"vop3-literal",
  "HasVOP3Literal",
  "true",
  "Can use one literal in VOP3"
>;

// MCCodeEmiter require
def FeatureVOP3P : SubtargetFeature<"vop3p",
  "HasVOP3PInsts",
  "true",
  "Has VOP3P packed instructions"
>;
def FeatureNoSdstCMPX : SubtargetFeature<"no-sdst-cmpx",
  "HasNoSdstCMPX",
  "true",
  "V_CMPX does not write VCC/SGPR in addition to EXEC"
>;
def FeatureScalarStores : SubtargetFeature<"scalar-stores",
  "HasScalarStores",
  "true",
  "Has store scalar memory instructions"
>;
def FeatureScalarAtomics : SubtargetFeature<"scalar-atomics",
  "HasScalarAtomics",
  "true",
  "Has atomic scalar memory instructions"
>;

def FeatureFlatAddressSpace : SubtargetFeature<"flat-address-space",
  "FlatAddressSpace",
  "true",
  "Support flat address space"
>;

def FeatureFlatInstOffsets : SubtargetFeature<"flat-inst-offsets",
  "FlatInstOffsets",
  "true",
  "Flat instructions have immediate offset addressing mode"
>;

def FeatureFlatGlobalInsts : SubtargetFeature<"flat-global-insts",
  "FlatGlobalInsts",
  "true",
  "Have global_* flat memory instructions"
>;

def FeatureFlatScratchInsts : SubtargetFeature<"flat-scratch-insts",
  "FlatScratchInsts",
  "true",
  "Have scratch_* flat memory instructions"
>;


def FeatureScalarFlatScratchInsts : SubtargetFeature<"scalar-flat-scratch-insts",
  "ScalarFlatScratchInsts",
  "true",
  "Have s_scratch_* flat memory instructions"
>;

def FeatureSRAMECC : SubtargetFeature<"sram-ecc",
  "EnableSRAMECC",
  "true",
  "Enable SRAM ECC"
>;

def FeatureDot3Insts : SubtargetFeature<"dot3-insts",
  "HasDot3Insts",
  "true",
  "Has v_dot8c_i32_i4 instruction"
>;
def FeatureDot4Insts : SubtargetFeature<"dot4-insts",
  "HasDot4Insts",
  "true",
  "Has v_dot2c_i32_i16 instruction"
>;

def FeatureDot5Insts : SubtargetFeature<"dot5-insts",
  "HasDot5Insts",
  "true",
  "Has v_dot2c_f32_f16 instruction"
>;

def FeatureDot6Insts : SubtargetFeature<"dot6-insts",
  "HasDot6Insts",
  "true",
  "Has v_dot4c_i32_i8 instruction"
>;
// Dummy feature used to disable assembler instructions.
def FeatureDisable : SubtargetFeature<"",
  "FeatureDisable","true",
  "Dummy feature to disable assembler instructions"
>;

def FeatureIntClamp : SubtargetFeature<"int-clamp-insts",
  "HasIntClamp",
  "true",
  "Support clamp for integer destination"
>;

//===------------------------------------------------------------===//
// Subtarget Features (options and debugging)
//===------------------------------------------------------------===//

// Denormal handling for fp64 and fp16 is controlled by the same
// config register when fp16 supported.
// TODO: Do we need a separate f16 setting when not legal?
def FeatureFP64FP16Denormals : SubtargetFeature<"fp64-fp16-denormals",
  "FP64FP16Denormals",
  "true",
  "Enable double and half precision denormal handling",
  [FeatureFP64]
>;

def FeatureFP64Denormals : SubtargetFeature<"fp64-denormals",
  "FP64FP16Denormals",
  "true",
  "Enable double and half precision denormal handling",
  [FeatureFP64, FeatureFP64FP16Denormals]
>;

def FeatureFP16Denormals : SubtargetFeature<"fp16-denormals",
  "FP64FP16Denormals",
  "true",
  "Enable half precision denormal handling",
  [FeatureFP64FP16Denormals]
>;

def FeatureFPExceptions : SubtargetFeature<"fp-exceptions",
  "FPExceptions",
  "true",
  "Enable floating point exceptions"
>;

class FeatureMaxPrivateElementSize<int size> : SubtargetFeature<
  "max-private-element-size-"#size,
  "MaxPrivateElementSize",
  !cast<string>(size),
  "Maximum private access size may be "#size
>;

def FeatureMaxPrivateElementSize4 : FeatureMaxPrivateElementSize<4>;
def FeatureMaxPrivateElementSize8 : FeatureMaxPrivateElementSize<8>;
def FeatureMaxPrivateElementSize16 : FeatureMaxPrivateElementSize<16>;

def FeatureDumpCode : SubtargetFeature <"DumpCode",
  "DumpCode",
  "true",
  "Dump MachineInstrs in the CodeEmitter"
>;

def FeatureDumpCodeLower : SubtargetFeature <"dumpcode",
  "DumpCode",
  "true",
  "Dump MachineInstrs in the CodeEmitter"
>;

// XXX - This should probably be removed once enabled by default
def FeatureEnableLoadStoreOpt : SubtargetFeature <"load-store-opt",
  "EnableLoadStoreOpt",
  "true",
  "Enable SI load/store optimizer pass"
>;

// Performance debugging feature. Allow using DS instruction immediate
// offsets even if the base pointer can't be proven to be base. On SI,
// base pointer values that won't give the same result as a 16-bit add
// are not safe to fold, but this will override the conservative test
// for the base pointer.
def FeatureEnableUnsafeDSOffsetFolding : SubtargetFeature <
  "unsafe-ds-offset-folding",
  "EnableUnsafeDSOffsetFolding",
  "true",
  "Force using DS instruction immediate offsets on SI"
>;

def FeatureEnablePPUScheduler : SubtargetFeature<"ppu-scheduler",
  "EnablePPUScheduler",
  "true",
  "Enable PPU Machine Scheduler"
>;

def FeatureCodeObjectV3 : SubtargetFeature <
  "code-object-v3",
  "CodeObjectV3",
  "true",
  "Generate code object version 3"
>;




//===----------------------------------------------------------------------===//
// Predicate helper class
//===----------------------------------------------------------------------===//
// FIXME use cl option or feature?
def EnableReconvergeCFG
    : SubtargetFeature<"r", "EnableReconvergeCFG", "true",
                       "'R' (ReconvergeCFG)">;

// def EnableLateCFGStructurize : Predicate<"EnableLateStructurizeCFG">;
// def EnableNonUniformMIBranchPseudo : Predicate<"EnableReconvergeCFG || EnableLateCFGStructurize">,
//                             AssemblerPredicate<"EnableReconvergeCFG || EnableLateCFGStructurize">;

def EnableNonUniformMIBranchPseudo : Predicate<"EnableReconvergeCFG">,
                             AssemblerPredicate<"EnableReconvergeCFG">;

//===----------------------------------------------------------------------===//
// Named operands for CSR instructions.
//===----------------------------------------------------------------------===//

include "PPUSystemOperands.td"

//===----------------------------------------------------------------------===//
// Registers, calling conventions, instruction descriptions.
//===----------------------------------------------------------------------===//

include "PPUSchedule.td"
//===----------------------------------------------------------------------===//
// Define the PPU target.
//===----------------------------------------------------------------------===//

// TODO i don't know i fix is corrrect
def PPUInstrInfo : InstrInfo {
  let guessInstructionProperties = 1;
  let noNamedPositionallyEncodedOperands = 1;
}

// TODO i don't know i fix is corrrect
def PPUAsmParser : AsmParser {
  let ShouldEmitMatchRegisterAltName = 1;
  let AllowDuplicateRegisterNames = 1;
  // let ShouldEmitMatchRegisterName = 0;
}

def PPUAsmWriter : AsmWriter {
  int PassSubtarget = 1;
}

def PPUAsmVariants {
  string Default = "Default";
  int Default_ID = 0;
  string VOP3 = "VOP3";
  int VOP3_ID = 1;
  string Disable = "Disable";
  int Disable_ID = 2;
}

def DefaultPPUAsmParserVariant : AsmParserVariant {
  let Variant = PPUAsmVariants.Default_ID;
  let Name = PPUAsmVariants.Default;
}

def VOP3AsmParserVariant : AsmParserVariant {
  let Variant = PPUAsmVariants.VOP3_ID;
  let Name = PPUAsmVariants.VOP3;
}

def PPU : Target {
  let InstructionSet = PPUInstrInfo;
  let AssemblyParsers = [PPUAsmParser];
  let AssemblyParserVariants = [DefaultPPUAsmParserVariant,
                                VOP3AsmParserVariant];
  let AssemblyWriters = [PPUAsmWriter];
  let AllowRegisterRenaming = 1;
}
// Dummy Instruction itineraries for pseudo instructions
def ALU_NULL : FuncUnit;
def NullALU : InstrItinClass;



def HasD16LoadStore : Predicate<"subtarget->hasd16loadstore()">,
  AssemblerPredicate<"FeatureGFX9Insts">;

def HasUnpackedD16VMem : Predicate<"Subtarget->hasUnpackedD16VMem()">,
  AssemblerPredicate<"FeatureUnpackedD16VMem">;

def HasPackedD16VMem : Predicate<"!Subtarget->hasUnpackedD16VMem()">,
  AssemblerPredicate<"!FeatureUnpackedD16VMem">;

def D16PreservesUnusedBits : Predicate<"Subtarget->d16PreservesUnusedBits()">,
  AssemblerPredicate<"FeatureGFX9Insts,!FeatureSRAMECC">;



def LDSRequiresM0Init : Predicate<"Subtarget->ldsRequiresM0Init()">;
def NotLDSRequiresM0Init : Predicate<"!Subtarget->ldsRequiresM0Init()">;

def HasDSAddTid : Predicate<"Subtarget->getGeneration() >= PPUSubtarget::PPU">,
  AssemblerPredicate<"FeatureGFX9Insts">;

def HasAddNoCarryInsts : Predicate<"Subtarget->hasAddNoCarry()">,
  AssemblerPredicate<"FeatureAddNoCarryInsts">;

def NotHasAddNoCarryInsts : Predicate<"!Subtarget->hasAddNoCarry()">;

def Has16BitInsts : Predicate<"Subtarget->has16BitInsts()">,
  AssemblerPredicate<"Feature16BitInsts">;

def HasVOP3PInsts : Predicate<"Subtarget->hasVOP3PInsts()">,
  AssemblerPredicate<"FeatureVOP3P">;

def HasSdstCMPX : Predicate<"!Subtarget->hasNoSdstCMPX()">,
  AssemblerPredicate<"!FeatureNoSdstCMPX">;

def HasNoSdstCMPX : Predicate<"Subtarget->hasNoSdstCMPX()">,
  AssemblerPredicate<"FeatureNoSdstCMPX">;

def HasScalarStores : Predicate<"Subtarget->hasScalarStores()">,
  AssemblerPredicate<"FeatureScalarStores">;

def HasScalarAtomics : Predicate<"Subtarget->hasScalarAtomics()">,
  AssemblerPredicate<"FeatureScalarAtomics">;


def HasFlatAddressSpace : Predicate<"Subtarget->hasFlatAddressSpace()">,
  AssemblerPredicate<"FeatureFlatAddressSpace">;

def HasFlatGlobalInsts : Predicate<"Subtarget->hasFlatGlobalInsts()">,
  AssemblerPredicate<"FeatureFlatGlobalInsts">;
def HasFlatScratchInsts : Predicate<"Subtarget->hasFlatScratchInsts()">,
  AssemblerPredicate<"FeatureFlatScratchInsts">;


def HasScalarFlatScratchInsts : Predicate<"Subtarget->hasScalarFlatScratchInsts()">,
  AssemblerPredicate<"FeatureScalarFlatScratchInsts">;


def HasVPRIndexMode : Predicate<"Subtarget->hasVPRIndexMode()">,
  AssemblerPredicate<"FeatureVPRIndexMode">;

def HasMovrel : Predicate<"Subtarget->hasMovrel()">,
                AssemblerPredicate<"FeatureMovrel">;


def isGFX10Plus :
  Predicate<"Subtarget->getGeneration() >= PPUSubtarget::PPU">,
  AssemblerPredicate<"FeatureGFX10Insts">;

def HasPkFmacF16Inst : Predicate<"Subtarget->hasPkFmacF16Inst()">,
  AssemblerPredicate<"FeaturePkFmacF16Inst">;

def HasAtomicFaddInsts : Predicate<"Subtarget->hasAtomicFaddInsts()">,
  AssemblerPredicate<"FeatureAtomicFaddInsts">;

def HasDLInsts : Predicate<"Subtarget->hasDLInsts()">,
  AssemblerPredicate<"FeatureDLInsts">;

def HasDot1Insts : Predicate<"Subtarget->hasDot1Insts()">,
  AssemblerPredicate<"FeatureDot1Insts">;

def HasDot2Insts : Predicate<"Subtarget->hasDot2Insts()">,
  AssemblerPredicate<"FeatureDot2Insts">;

def HasDot3Insts : Predicate<"Subtarget->hasDot3Insts()">,
  AssemblerPredicate<"FeatureDot3Insts">;

def HasDot4Insts : Predicate<"Subtarget->hasDot4Insts()">,
  AssemblerPredicate<"FeatureDot4Insts">;

def HasDot5Insts : Predicate<"Subtarget->hasDot5Insts()">,
  AssemblerPredicate<"FeatureDot5Insts">;

def HasDot6Insts : Predicate<"Subtarget->hasDot6Insts()">,
  AssemblerPredicate<"FeatureDot6Insts">;




include "PPURegisterInfo.td"
include "PPUCallingConv.td"
include "PPUInstructions.td"
include "PPUInstrInfo.td"
include "PPURegisterBanks.td"
//===----------------------------------------------------------------------===//
// PPU processors supported.
//===----------------------------------------------------------------------===//
// The code produced for "generic" is only useful for tests and cannot
// reasonably be expected to execute on any particular target.
def : ProcessorModel<"generic", PPUFullSpeedModel,
  []
>;

def : ProcessorModel<"generic-hsa", NoSchedModel,
  [] // [FeatureFlatAddressSpace]
>;


def : ProcessorModel<"generic-ppu", NoSchedModel, [FeatureRVCHints]>;
def : ProcessorModel<"generic-ppu64", NoSchedModel, [Feature64Bit,
                     FeatureRVCHints]>;

