
def FeaturePromoteAlloca : SubtargetFeature <"promote-alloca",
  "EnablePromoteAlloca",
  "true",
  "Enable promote alloca pass"
>;

def FeatureEnablePPUScheduler : SubtargetFeature<"ppu-scheduler",
  "EnablePPUScheduler",
  "true",
  "Enable PPU Machine Scheduler"
>;

def FeatureFlatForGlobal : SubtargetFeature<"flat-for-global",
  "FlatForGlobal",
  "true",
  "Force to generate flat instruction for global"
>;

def FeatureCodeObjectV3 : SubtargetFeature <"code-object-v3",
  "CodeObjectV3",
  "true",
  "Generate code object version 3"
>;

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

//===----------------------------------------------------------------------===//
// feature base on AMDGPU.td
//===----------------------------------------------------------------------===//
def FeatureVPRIndexMode : SubtargetFeature<"vpr-index-mode",
  "HasVPRIndexMode",
  "true",
  "Has VGPR mode register indexing"
>;

def FeatureUnpackedD16VMem : SubtargetFeature<"unpacked-d16-vmem",
  "HasUnpackedD16VMem",
  "true",
  "Has unpacked d16 vmem instructions"
>;

def FeatureMovrel : SubtargetFeature<"movrel",
  "HasMovrel",
  "true",
  "Has v_movrel*_b32 instructions"
>;

