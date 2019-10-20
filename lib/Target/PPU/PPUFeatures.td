
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


