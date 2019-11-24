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


//===----------------------------------------------------------------------===//
// Predicate helper class
//===----------------------------------------------------------------------===//
// FIXME use cl option or feature?
def EnableReconvergeCFG
    : SubtargetFeature<"r", "EnableReconvergeCFG", "true",
                       "'R' (ReconvergeCFG)">;

def EnableLateCFGStructurize : Predicate<"EnableLateStructurizeCFG">;


def EnableNonUniformMIBranchPseudo : Predicate<"EnableReconvergeCFG || EnableLateCFGStructurize">,
                            AssemblerPredicate<"EnableReconvergeCFG || EnableLateCFGStructurize">;


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

def PPUInstrInfo : InstrInfo {
  let guessInstructionProperties = 0;
}

def PPUAsmParser : AsmParser {
  let ShouldEmitMatchRegisterAltName = 1;
  let AllowDuplicateRegisterNames = 1;
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
  int Disable_ID = 5;
}

def PPU : Target {
  let InstructionSet = PPUInstrInfo;
  let AssemblyParsers = [PPUAsmParser];
  let AssemblyWriters = [PPUAsmWriter];
  let AllowRegisterRenaming = 1;
}
// Dummy Instruction itineraries for pseudo instructions
def ALU_NULL : FuncUnit;
def NullALU : InstrItinClass;



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

def HasScalarFlatScratchInsts : Predicate<"Subtarget->hasScalarFlatScratchInsts()">,
  AssemblerPredicate<"FeatureScalarFlatScratchInsts">;


def HasUnpackedD16VMem : Predicate<"Subtarget->hasUnpackedD16VMem()">,
  AssemblerPredicate<"FeatureUnpackedD16VMem">;

def HasPackedD16VMem : Predicate<"!Subtarget->hasUnpackedD16VMem()">,
  AssemblerPredicate<"!FeatureUnpackedD16VMem">;

def HasVPRIndexMode : Predicate<"Subtarget->hasVPRIndexMode()">,
  AssemblerPredicate<"FeatureVPRIndexMode">;

def HasMovrel : Predicate<"Subtarget->hasMovrel()">,
                AssemblerPredicate<"FeatureMovrel">;

def HasD16LoadStore : Predicate<"subtarget->hasd16loadstore()">,
  AssemblerPredicate<"featuregfx9insts">;

def D16PreservesUnusedBits : Predicate<"Subtarget->d16PreservesUnusedBits()">,
  AssemblerPredicate<"FeatureGFX9Insts,!FeatureSRAMECC">;

def isGFX10Plus :
  Predicate<"Subtarget->getGeneration() >= AMDGPUSubtarget::GFX10">,
  AssemblerPredicate<"FeatureGFX10Insts">;

def HasPkFmacF16Inst : Predicate<"Subtarget->hasPkFmacF16Inst()">,
  AssemblerPredicate<"FeaturePkFmacF16Inst">;


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

