//===-- PPUTargetMachine.cpp - Define TargetMachine for PPU -----------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// Implements the info about PPU target spec.
//
//===----------------------------------------------------------------------===//

#include "PPUTargetMachine.h"
#include "PPU.h"
#include "PPUAliasAnalysis.h"
#include "PPUMacroFusion.h"
#include "PPUTargetObjectFile.h"
#include "PPUTargetTransformInfo.h"
#include "PPUSchedStrategy.h"
#include "PPUIterativeScheduler.h"
#include "PPUMachineScheduler.h"
#include "TargetInfo/PPUTargetInfo.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/Analysis/TargetTransformInfo.h"
#include "llvm/CodeGen/GlobalISel/IRTranslator.h"
#include "llvm/CodeGen/GlobalISel/InstructionSelect.h"
#include "llvm/CodeGen/GlobalISel/Legalizer.h"
#include "llvm/CodeGen/GlobalISel/RegBankSelect.h"
#include "llvm/CodeGen/Passes.h"
#include "llvm/CodeGen/TargetLoweringObjectFileImpl.h"
#include "llvm/CodeGen/TargetPassConfig.h"
#include "llvm/IR/Attributes.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/LegacyPassManager.h"
#include "llvm/Pass.h"
#include "llvm/Support/FormattedStream.h"
#include "llvm/Support/TargetRegistry.h"
#include "llvm/Target/TargetOptions.h"
#include "llvm/Target/TargetLoweringObjectFile.h"
#include "llvm/Transforms/IPO.h"
#include "llvm/Transforms/IPO/AlwaysInliner.h"
#include "llvm/Transforms/IPO/PassManagerBuilder.h"
#include "llvm/Transforms/Scalar.h"
#include "llvm/Transforms/Scalar/GVN.h"
#include "llvm/Transforms/Utils.h"
#include "llvm/Transforms/Vectorize.h"
using namespace llvm;

// Option to use reconverging CFG
///* FIXME schi we use feature instead of option
static cl::opt<bool, true> EnableReconvergeCFG(
  "ppu-EnableReconvergeCFG",
  cl::desc("Use reconverging CFG instead of structurization"),
  cl::location(PPUTargetMachine::EnableReconvergeCFG),
  cl::Hidden);

static cl::opt<bool> EnableSROA(
  "ppu-sroa",
  cl::desc("Run SROA after promote alloca pass"),
  cl::ReallyHidden,
  cl::init(true));

static cl::opt<bool> EnableEarlyIfConversion(
  "ppu-early-ifcvt",
  cl::Hidden,
  cl::desc("Run early if-conversion"),
  cl::init(false));

static cl::opt<bool> OptExecMaskPreRA(
  "ppu-opt-exec-mask-pre-ra", cl::Hidden,
  cl::desc("Run pre-RA exec mask optimizations"),
  cl::init(true));

// Option to disable vectorizer for tests.
static cl::opt<bool> EnableLoadStoreVectorizer(
  "ppu-load-store-vectorizer",
  cl::desc("Enable load store vectorizer"),
  cl::init(true),
  cl::Hidden);

// Option to control global loads scalarization
static cl::opt<bool> ScalarizeGlobal(
  "ppu-scalarize-global-loads",
  cl::desc("Enable global load scalarization"),
  cl::init(true),
  cl::Hidden);

// Option to run internalize pass.
static cl::opt<bool> InternalizeSymbols(
  "ppu-internalize-symbols",
  cl::desc("Enable elimination of non-kernel functions and unused globals"),
  cl::init(false),
  cl::Hidden);

// Option to inline all early.
static cl::opt<bool> EarlyInlineAll(
  "ppu-early-inline-all",
  cl::desc("Inline all functions early"),
  cl::init(false),
  cl::Hidden);

// Enable address space based alias analysis
static cl::opt<bool> EnablePPUAliasAnalysis("enable-ppu-aa", cl::Hidden,
  cl::desc("Enable PPU Alias Analysis"),
  cl::init(true));

// Option to run late CFG structurizer
static cl::opt<bool, true> LateCFGStructurize(
  "ppu-late-structurize",
  cl::desc("Enable late CFG structurization"),
  cl::location(PPUTargetMachine::EnableLateStructurizeCFG),
  cl::Hidden);


static cl::opt<bool, true> EnablePPUFunctionCallsOpt(
  "ppu-function-calls",
  cl::desc("Enable PPU function call support"),
  cl::location(PPUTargetMachine::EnableFunctionCalls),
  cl::init(true),
  cl::Hidden);

// Enable lib calls simplifications
static cl::opt<bool> EnableLibCallSimplify(
  "ppu-simplify-libcall",
  cl::desc("Enable ppu library simplifications"),
  cl::init(true),
  cl::Hidden);

static cl::opt<bool> EnableLowerKernelArguments(
  "ppu-ir-lower-kernel-arguments",
  cl::desc("Lower kernel argument loads in IR pass"),
  cl::init(true),
  cl::Hidden);

// TODO schi change to false
static cl::opt<bool> EnableRegReassign(
  "ppu-reassign-regs",
  cl::desc("Enable register reassign optimizations on gfx10+"),
  cl::init(false),
  cl::Hidden);

// Option is used in lit tests to prevent deadcoding of patterns inspected.
static cl::opt<bool>
EnableDCEInRA("ppu-dce-in-ra",
    cl::init(true), cl::Hidden,
    cl::desc("Enable machine DCE inside regalloc"));

static cl::opt<bool> EnableScalarIRPasses(
  "ppu-scalar-ir-passes",
  cl::desc("Enable scalar IR passes"),
  cl::init(true),
  cl::Hidden);


extern "C" void LLVMInitializePPUTarget() {
  RegisterTargetMachine<PPUTargetMachine> X(getThePPUTarget());

  auto PR = PassRegistry::getPassRegistry();
  initializeGlobalISel(*PR);
  initializePPUDAGToDAGISelPass(*PR);
  initializePPUExpandPseudoPass(*PR);
  initializePPUFoldOperandsPass(*PR);
  initializePPULoadStoreOptimizerPass(*PR);
  initializePPULowerSPRSpillsPass(*PR);
  initializePPULowerI1CopiesPass(*PR);
  initializePPUFixSGPRCopiesPass(*PR);
  initializePPUFixVGPRCopiesPass(*PR);
  initializePPUFixupVectorISelPass(*PR);
  initializePPUAlwaysInlinePass(*PR);
  initializePPUAnnotateKernelFeaturesPass(*PR);
  initializePPUAnnotateUniformValuesPass(*PR);
  initializePPUArgumentUsageInfoPass(*PR);
  initializePPULowerControlFlowPass(*PR);
  initializePPULowerKernelArgumentsPass(*PR);
  initializePPULowerKernelAttributesPass(*PR);
  initializePPULowerIntrinsicsPass(*PR);
  initializePPULowerReconvergingControlFlowPass(*PR);
  initializePPUOptimizeExecMaskingPreRAPass(*PR);
  initializePPUOptimizeExecMaskingPass(*PR);
  initializePPUPromoteAllocaPass(*PR);
  initializePPUPreAllocateWWMRegsPass(*PR);
  initializePPUCodeGenPreparePass(*PR);
  // initializePPUPropagateAttributesEarlyPass(*PR);
  // initializePPUPropagateAttributesLatePass(*PR);
  initializePPURewriteOutArgumentsPass(*PR);
  initializePPUUnifyMetadataPass(*PR);
  initializePPUAnnotateControlFlowPass(*PR);
  // initializePPUInsertWaitcntsPass(*PR);
  initializePPUUnifyDivergentExitNodesPass(*PR);
  initializePPUAAWrapperPassPass(*PR);
  initializePPUExternalAAWrapperPass(*PR);
  initializePPUInlinerPass(*PR);
}


static ScheduleDAGInstrs *createPPUMachineScheduler(MachineSchedContext *C) {
  return new PPUScheduleDAGMI(C);
}

static ScheduleDAGInstrs * createPPUMaxOccupancyMachineScheduler(MachineSchedContext *C) {
  ScheduleDAGMILive *DAG =
    new PPUScheduleDAGMILive(C, std::make_unique<PPUMaxOccupancySchedStrategy>(C));
  DAG->addMutation(createLoadClusterDAGMutation(DAG->TII, DAG->TRI));
  DAG->addMutation(createStoreClusterDAGMutation(DAG->TII, DAG->TRI));
  DAG->addMutation(createPPUMacroFusionDAGMutation());
  return DAG;
}

static ScheduleDAGInstrs *
createIterativePPUMaxOccupancyMachineScheduler(MachineSchedContext *C) {
  auto DAG = new PPUIterativeScheduler(C,
    PPUIterativeScheduler::SCHEDULE_LEGACYMAXOCCUPANCY);
  DAG->addMutation(createLoadClusterDAGMutation(DAG->TII, DAG->TRI));
  DAG->addMutation(createStoreClusterDAGMutation(DAG->TII, DAG->TRI));
  return DAG;
}

static ScheduleDAGInstrs *createMinRegScheduler(MachineSchedContext *C) {
  return new PPUIterativeScheduler(C,
    PPUIterativeScheduler::SCHEDULE_MINREGFORCED);
}

static ScheduleDAGInstrs *
createIterativeILPMachineScheduler(MachineSchedContext *C) {
  auto DAG = new PPUIterativeScheduler(C,
    PPUIterativeScheduler::SCHEDULE_ILP);
  DAG->addMutation(createLoadClusterDAGMutation(DAG->TII, DAG->TRI));
  DAG->addMutation(createStoreClusterDAGMutation(DAG->TII, DAG->TRI));
  DAG->addMutation(createPPUMacroFusionDAGMutation());
  return DAG;
}

static MachineSchedRegistry PPUSchedRegistry("ppu", "Run PPU's custom scheduler",
                createPPUMachineScheduler);

static MachineSchedRegistry PPUMaxOccupancySchedRegistry("ppu-max-occupancy",
                             "Run PPU scheduler to maximize occupancy",
                             createPPUMaxOccupancyMachineScheduler);

// TODO kernel compiled with Max/Min/ILP ,and runtime pps select best one to run
static MachineSchedRegistry IterativePPUMaxOccupancySchedRegistry(
        "ppu-max-occupancy-experimental",
        "Run PPU scheduler to maximize occupancy (experimental)",
        createIterativePPUMaxOccupancyMachineScheduler);

static MachineSchedRegistry PPUMinRegSchedRegistry(
        "ppu-minreg",
        "Run PPU iterative scheduler for minimal register usage (experimental)",
        createMinRegScheduler);

static MachineSchedRegistry PPUILPSchedRegistry(
        "ppu-ilp",
        "Run PPU iterative scheduler for ILP scheduling (experimental)",
        createIterativeILPMachineScheduler);

static StringRef computeDataLayout(const Triple &TT) {
  // odl riscv return "e-m:e-p:32:32-i64:64-n32-S128";
  // TODO PPU
  // 
  // 32-bit private, local, and region pointers. 64-bit global, constant and
  // flat, non-integral buffer fat pointers.
    return "e-p:64:64-p1:64:64-p2:32:32-p3:32:32-p4:64:64-p5:32:32-p6:32:32"
         "-i64:64-v16:16-v24:32-v32:32-v48:64-v96:128"
         "-v192:256-v256:256-v512:512-v1024:1024-v2048:2048-n32:64-S32-A5"
         "-ni:7";
}

LLVM_READNONE
static StringRef getCPUOrDefault(const Triple &TT, StringRef CPU) {
  if (!CPU.empty())
    return CPU;

  // Need to default to a target with flat support for HSA.
  assert(TT.getArch() == Triple::ppu);
  // assert(TT.getOS() == Triple::PPS);
  return "generic-ppu";
}

static Reloc::Model getEffectiveRelocModel(const Triple &TT,
                                           Optional<Reloc::Model> RM) {
  if (!RM.hasValue())
    return Reloc::Static;
  return *RM;
}

PPUTargetMachine::PPUTargetMachine(const Target &T, const Triple &TT,
                                       StringRef CPU, StringRef FS,
                                       const TargetOptions &Options,
                                       Optional<Reloc::Model> RM,
                                       Optional<CodeModel::Model> CM,
                                       CodeGenOpt::Level OL, bool JIT)
    : LLVMTargetMachine(T, computeDataLayout(TT), TT, getCPUOrDefault(TT, CPU),
                        FS, Options, getEffectiveRelocModel(TT, RM),
                        getEffectiveCodeModel(CM, CodeModel::Small), OL),
      TLOF(std::make_unique<PPUELFTargetObjectFile>())
      // Options(Options)
      /*Subtarget(TT, CPU, FS, Options.MCOptions.getABIName(), *this)*/ {
  initAsmInfo();
}

bool PPUTargetMachine::EnableReconvergeCFG = true;
bool PPUTargetMachine::EnableLateStructurizeCFG = false;
bool PPUTargetMachine::EnableFunctionCalls = false;

StringRef PPUTargetMachine::getFeatureString(const Function &F) const {
  Attribute FSAttr = F.getFnAttribute("target-features");

  return FSAttr.hasAttribute(Attribute::None) ?
    getTargetFeatureString() :
    FSAttr.getValueAsString();
}

/// Predicate for Internalize pass.
static bool mustPreserveGV(const GlobalValue &GV) {
  if (const Function *F = dyn_cast<Function>(&GV))
    return F->isDeclaration() || PPU::isEntryFunctionCC(F->getCallingConv());

  return !GV.use_empty();
}

void PPUTargetMachine::adjustPassManager(PassManagerBuilder &Builder) {
  Builder.DivergentTarget = true;

  bool EnableOpt = getOptLevel() > CodeGenOpt::None;
  bool Internalize = InternalizeSymbols;
  bool EarlyInline = EarlyInlineAll && EnableOpt && !EnableFunctionCalls;
  bool PPUAA = EnablePPUAliasAnalysis && EnableOpt;
  bool LibCallSimplify = EnableLibCallSimplify && EnableOpt;

  if (EnableFunctionCalls) {
    delete Builder.Inliner;
    Builder.Inliner = createPPUFunctionInliningPass();
  }

  Builder.addExtension(
    PassManagerBuilder::EP_ModuleOptimizerEarly,
    [Internalize, EarlyInline, PPUAA, this](const PassManagerBuilder &,
                                               legacy::PassManagerBase &PM) {
      if (PPUAA) {
        PM.add(createPPUAAWrapperPass());
        PM.add(createPPUExternalAAWrapperPass());
      }
      PM.add(createPPUUnifyMetadataPass());
      // TODO PM.add(createPPUPrintfRuntimeBinding());
      // TODO PM.add(createPPUPropagateAttributesLatePass(this));
      if (Internalize) {
        PM.add(createInternalizePass(mustPreserveGV));
        PM.add(createGlobalDCEPass());
      }
      if (EarlyInline)
        PM.add(createPPUAlwaysInlinePass(false));
  });

  const auto &Opt = Options;
  Builder.addExtension(
    PassManagerBuilder::EP_EarlyAsPossible,
    [PPUAA, LibCallSimplify, &Opt, this](const PassManagerBuilder &,
                                            legacy::PassManagerBase &PM) {
      if (PPUAA) {
        PM.add(createPPUAAWrapperPass());
        PM.add(createPPUExternalAAWrapperPass());
      }
      // TODO PM.add(llvm::createPPUPropagateAttributesEarlyPass(this));
      // TODO PM.add(llvm::createPPUUseNativeCallsPass());
      // TODO if (LibCallSimplify)
      // TODO   PM.add(llvm::createPPUSimplifyLibCallsPass(Opt, this));
  });

  Builder.addExtension(
    PassManagerBuilder::EP_CGSCCOptimizerLate,
    [](const PassManagerBuilder &, legacy::PassManagerBase &PM) {
      // Add infer address spaces pass to the opt pipeline after inlining
      // but before SROA to increase SROA opportunities.
      PM.add(createInferAddressSpacesPass());

      // This should run after inlining to have any chance of doing anything,
      // and before other cleanup optimizations.
      PM.add(createPPULowerKernelAttributesPass());
  });
}

const PPUSubtarget *PPUTargetMachine::getSubtargetImpl(const Function &F) const {
  StringRef CPU = getTargetCPU(); // "PPU";
  StringRef FS = getFeatureString(F);

  SmallString<128> SubtargetKey(CPU);
  SubtargetKey.append(FS);

  auto &I = SubtargetMap[SubtargetKey];
  if (!I) {
    // This needs to be done before we create a new subtarget since any
    // creation will depend on the TM and the code generation flags on the
    // function that reside in TargetOptions.
    resetTargetOptions(F);
    I = std::make_unique<PPUSubtarget>(TargetTriple, CPU, FS, Options.MCOptions.getABIName(), *this);
  }

  I->setScalarizeGlobalBehavior(ScalarizeGlobal);

  return I.get();
}

TargetTransformInfo
PPUTargetMachine::getTargetTransformInfo(const Function &F) {
  return TargetTransformInfo(PPUTTIImpl(this, F));
}

namespace {
class PPUPassConfig : public TargetPassConfig {
public:
  PPUPassConfig(PPUTargetMachine &TM, PassManagerBase &PM)
      : TargetPassConfig(TM, PM)
  {
    // Exceptions and StackMaps are not supported, so these passes will never do
    // anything.
    disablePass(&StackMapLivenessID);
    disablePass(&FuncletLayoutID);

    // It is necessary to know the register usage of the entire call graph.  We
    // allow calls without EnablePPUFunctionCalls if they are marked
    // noinline, so this is always required.
    setRequiresCodeGenSCCOrder(true);
  }

  PPUTargetMachine &getPPUTargetMachine() const {
    return getTM<PPUTargetMachine>();
  }

  ScheduleDAGInstrs *
  createMachineScheduler(MachineSchedContext *C) const override {
    const PPUSubtarget &ST = C->MF->getSubtarget<PPUSubtarget>();
    if (ST.enablePPUScheduler())
        return createPPUMachineScheduler(C);
    return createPPUMaxOccupancyMachineScheduler(C);
  }

  // GCNBase
  void addEarlyCSEOrGVNPass();
  void addStraightLineScalarOptimizationPasses();
  void addIRPasses() override;
  void addCodeGenPrepare() override;
  bool addPreISel() override;
  bool addInstSelector() override;
  // bool addGCPasses() override;

  // GCN
  // bool addPreISel() override;
  void addMachineSSAOptimization() override;
  bool addILPOpts() override;
  // bool addInstSelector() override;
  bool addIRTranslator() override;
  bool addLegalizeMachineIR() override;
  bool addRegBankSelect() override;
  bool addGlobalInstructionSelect() override;
  void addFastRegAlloc() override;
  void addOptimizedRegAlloc() override;
  void addPreRegAlloc() override;
  bool addPreRewrite() override;
  void addPostRegAlloc() override;
  void addPreSched2() override;
  void addPreEmitPass() override;

  // RISCV
  // void addIRPasses() override;
  // bool addInstSelector() override;
  // bool addIRTranslator() override;
  // bool addLegalizeMachineIR() override;
  // bool addRegBankSelect() override;
  // bool addGlobalInstructionSelect() override;
  // void addPreEmitPass() override;
  void addPreEmitPass2() override;
  // void addPreRegAlloc() override;

  bool EnableReconvergeCFG {false};

  std::unique_ptr<CSEConfigBase> getCSEConfig() const override {
    return getStandardCSEConfigForOpt(TM->getOptLevel());
  }
};
}

void PPUPassConfig::addEarlyCSEOrGVNPass() {
  if (getOptLevel() == CodeGenOpt::Aggressive)
    addPass(createGVNPass());
  else
    addPass(createEarlyCSEPass());
}

void PPUPassConfig::addStraightLineScalarOptimizationPasses() {
  addPass(createLICMPass());
  addPass(createSeparateConstOffsetFromGEPPass());
  addPass(createSpeculativeExecutionPass());
  // ReassociateGEPs exposes more opportunites for SLSR. See
  // the example in reassociate-geps-and-slsr.ll.
  addPass(createStraightLineStrengthReducePass());
  // SeparateConstOffsetFromGEP and SLSR creates common expressions which GVN or
  // EarlyCSE can reuse.
  addEarlyCSEOrGVNPass();
  // Run NaryReassociate after EarlyCSE/GVN to be more effective.
  addPass(createNaryReassociatePass());
  // NaryReassociate on GEPs creates redundant common expressions, so run
  // EarlyCSE after it.
  addPass(createEarlyCSEPass());
}

void PPUPassConfig::addIRPasses() {
  const PPUTargetMachine &TM = getPPUTargetMachine();

  // There is no reason to run these.
  disablePass(&StackMapLivenessID);
  disablePass(&FuncletLayoutID);
  disablePass(&PatchableFunctionID);

  // TODO schi addPass(createPPUPrintfRuntimeBinding());
  // This must occur before inlining, as the inliner will not look through
  // bitcast calls.
  // TODO schi addPass(createPPUFixFunctionBitcastsPass());
  // A call to propagate attributes pass in the backend in case opt was not run.
  // TODO schi addPass(createPPUPropagateAttributesEarlyPass(&TM));
 
  // RISCV
  addPass(createAtomicExpandPass());

  addPass(createPPULowerIntrinsicsPass());

  // TODO schi if (!EnableFunctionCalls) {
  // Function calls are not supported, so make sure we inline everything.
  addPass(createPPUAlwaysInlinePass());
  addPass(createAlwaysInlinerLegacyPass());
  // We need to add the barrier noop pass, otherwise adding the function
  // inlining pass will cause all of the PassConfigs passes to be run
  // one function at a time, which means if we have a nodule with two
  // functions, then we will generate code for the first function
  // without ever running any passes on the second.
  addPass(createBarrierNoopPass());
  // TODO schi end if EnableFunctionCalls

  if (TM.getTargetTriple().getArch() == Triple::ppu) {
    // TODO: May want to move later or split into an early and late one.

    addPass(createPPUCodeGenPreparePass());
  }

  // Replace OpenCL enqueued block function pointers with global variables.
  // TODO addPass(createPPUOpenCLEnqueuedBlockLoweringPass());

  if (TM.getOptLevel() > CodeGenOpt::None) {
    addPass(createInferAddressSpacesPass());
    addPass(createPPUPromoteAlloca());

    if (EnableSROA)
      addPass(createSROAPass());

    if (EnableScalarIRPasses)
      addStraightLineScalarOptimizationPasses();

    if (EnablePPUAliasAnalysis) {
      addPass(createPPUAAWrapperPass());
      addPass(createExternalAAWrapperPass([](Pass &P, Function &,
                                             AAResults &AAR) {
        if (auto *WrapperPass = P.getAnalysisIfAvailable<PPUAAWrapperPass>())
          AAR.addAAResult(WrapperPass->getResult());
        }));
    }
  }

  if (TM.getTargetTriple().getArch() == Triple::ppu) {
    // TODO: May want to move later or split into an early and late one.
    addPass(createPPUCodeGenPreparePass());
  }

  TargetPassConfig::addIRPasses();

  // EarlyCSE is not always strong enough to clean up what LSR produces. For
  // example, GVN can combine
  //
  //   %0 = add %a, %b
  //   %1 = add %b, %a
  //
  // and
  //
  //   %0 = shl nsw %a, 2
  //   %1 = shl %a, 2
  //
  // but EarlyCSE can do neither of them.
  if (getOptLevel() != CodeGenOpt::None && EnableScalarIRPasses)
    addEarlyCSEOrGVNPass();
}

void PPUPassConfig::addCodeGenPrepare() {
  if (TM->getTargetTriple().getArch() == Triple::ppu)
    addPass(createPPUAnnotateKernelFeaturesPass());

  if (TM->getTargetTriple().getArch() == Triple::ppu &&
      EnableLowerKernelArguments)
    addPass(createPPULowerKernelArgumentsPass());

  addPass(&PPUPerfHintAnalysisID);

  TargetPassConfig::addCodeGenPrepare();

  if (EnableLoadStoreVectorizer)
    addPass(createLoadStoreVectorizerPass());
}

bool PPUPassConfig::addPreISel() {
  // GCNBase
  addPass(createLowerSwitchPass());
  addPass(createFlattenCFGPass());

  addPass(createPPUAnnotateUniformValues());

  // Merge divergent exit nodes. StructurizeCFG won't recognize the multi-exit
  // regions formed by them.
  addPass(&PPUUnifyDivergentExitNodesID);

  if (EnableReconvergeCFG) {
    addPass(createReconvergeCFGPass(true)); // true -> SkipUniformBranches
  } else if (!LateCFGStructurize) {
    addPass(createStructurizeCFGPass(true)); // true -> SkipUniformRegions
  }
  addPass(createSinkingPass());
  addPass(createPPUAnnotateUniformValues());
  if (!EnableReconvergeCFG && !LateCFGStructurize) {
    addPass(createPPUAnnotateControlFlowPass());
  }
  addPass(createLCSSAPass());

  return false;
}

bool PPUPassConfig::addInstSelector() {
  // GCNBase
  // Defer the verifier until FinalizeISel.
  // addPass(createPPUISelDag(getPPUTargetMachine()));
  addPass(createPPUISelDag(getPPUTargetMachine(), getOptLevel()), false);

  // GCN
  addPass(&PPUFixSGPRCopiesID);
  addPass(createPPULowerI1CopiesPass());
  // TODO schi addPass(createPPUFixupVectorISelPass());
  /*
  // FIXME: Remove this once the phi on CF_END is cleaned up by either removing
  // LCSSA or other ways.
  addPass(&UnreachableMachineBlockElimID);
  */
  return false;
}

void PPUPassConfig::addMachineSSAOptimization() {
  TargetPassConfig::addMachineSSAOptimization();

  // We want to fold operands after PeepholeOptimizer has run (or as part of
  // it), because it will eliminate extra copies making it easier to fold the
  // real source operand. We want to eliminate dead instructions after, so that
  // we see fewer uses of the copies. We then need to clean up the dead
  // instructions leftover after the operands are folded as well.
  //
  // XXX - Can we get away without running DeadMachineInstructionElim again?
  addPass(&PPUFoldOperandsID);
  /* TODO
  if (EnableDPPCombine)
    addPass(&GCNDPPCombineID);
  */
  addPass(&DeadMachineInstructionElimID);
  addPass(&PPULoadStoreOptimizerID);
  /* TODO
  if (EnableSDWAPeephole) {
    addPass(&SIPeepholeSDWAID);
    addPass(&EarlyMachineLICMID);
    addPass(&MachineCSEID);
    addPass(&SIFoldOperandsID);
    addPass(&DeadMachineInstructionElimID);
  }
  */
  // TODO schi addPass(createPPUShrinkInstructionsPass());
  
  // RISCV-V
  addPass(createPPUOptimizeVSETVLUsesPass());
}

bool PPUPassConfig::addILPOpts() {
  if (EnableEarlyIfConversion)
    addPass(&EarlyIfConverterID);

  TargetPassConfig::addILPOpts();
  return false;
}

bool PPUPassConfig::addIRTranslator() {
  addPass(new IRTranslator());
  return false;
}

bool PPUPassConfig::addLegalizeMachineIR() {
  addPass(new Legalizer());
  return false;
}

bool PPUPassConfig::addRegBankSelect() {
  addPass(new RegBankSelect());
  return false;
}

bool PPUPassConfig::addGlobalInstructionSelect() {
  addPass(new InstructionSelect());
  return false;
}

void PPUPassConfig::addFastRegAlloc() {
  // FIXME: We have to disable the verifier here because of PHIElimination +
  // TwoAddressInstructions disabling it.

  // This must be run immediately after phi elimination and before
  // TwoAddressInstructions, otherwise the processing of the tied operand of
  // SI_ELSE will introduce a copy of the tied operand source after the else.
  insertPass(&PHIEliminationID, &PPULowerControlFlowID, false);

  // This must be run just after RegisterCoalescing.
  insertPass(&RegisterCoalescerID, &PPUPreAllocateWWMRegsID, false);

  TargetPassConfig::addFastRegAlloc();
}

void PPUPassConfig::addOptimizedRegAlloc() {
  // char *PassID = &PHIEliminationID;

  if (OptExecMaskPreRA) {
    if (!EnableReconvergeCFG)
      insertPass(&MachineSchedulerID, &PPUOptimizeExecMaskingPreRAID);

    // insertPass(&PPUOptimizeExecMaskingPreRAID, &PPUFormMemoryClausesID);
  } else {
    // insertPass(&MachineSchedulerID, &PPUFormMemoryClausesID);
  }

  // This must be run immediately after phi elimination and before
  // TwoAddressInstructions, otherwise the processing of the tied operand of
  // SI_ELSE will introduce a copy of the tied operand source after the else.
  insertPass(&PHIEliminationID, &PPULowerControlFlowID, false);

  // This must be run just after RegisterCoalescing.
  insertPass(&RegisterCoalescerID, &PPUPreAllocateWWMRegsID, false);

  if (EnableDCEInRA)
    insertPass(&RenameIndependentSubregsID, &DeadMachineInstructionElimID);

  TargetPassConfig::addOptimizedRegAlloc();
}

void PPUPassConfig::addPreRegAlloc() {
  if (LateCFGStructurize) {
    // addPass(createAMDGPUMachineCFGStructurizerPass());
  }
  if (EnableReconvergeCFG)
    addPass(createPPULowerReconvergingControlFlowPass());

  // TODO addPass(createSIWholeQuadModePass());

  // RISCV
  addPass(createPPUMergeBaseOffsetOptPass());
}

bool PPUPassConfig::addPreRewrite() {
  if (EnableRegReassign) {
    // addPass(&PPUNSAReassignID);
    addPass(&PPURegBankReassignID);
  }
  return true;
}

void PPUPassConfig::addPostRegAlloc() {
  addPass(&PPUFixVGPRCopiesID);
  if (getOptLevel() > CodeGenOpt::None)
    addPass(&PPUOptimizeExecMaskingID);
  TargetPassConfig::addPostRegAlloc();

  // Equivalent of PEI for SGPRs.
  addPass(&PPULowerSPRSpillsID);
}

void PPUPassConfig::addPreSched2() {
}

void PPUPassConfig::addPreEmitPass() {
  addPass(createPPUMemoryLegalizerPass());
  // TODO schi addPass(createPPUInsertWaitcntsPass());
  // TODO schi addPass(createSIShrinkInstructionsPass());
  // TODO schi addPass(createPPUModeRegisterPass());

  // The hazard recognizer that runs as part of the post-ra scheduler does not
  // guarantee to be able handle all hazards correctly. This is because if there
  // are multiple scheduling regions in a basic block, the regions are scheduled
  // bottom up, so when we begin to schedule a region we don't know what
  // instructions were emitted directly before it.
  //
  // Here we add a stand-alone hazard recognizer pass which can handle all
  // cases.
  //
  // FIXME: This stand-alone pass will emit indiv. S_NOP 0, as needed. It would
  // be better for it to emit S_NOP <N> when possible.
  addPass(&PostRAHazardRecognizerID);

  // FIXME addPass(&PPUInsertSkipsPassID);

  // RISCV & GCN
  addPass(&BranchRelaxationPassID);
}


// RISCV
void PPUPassConfig::addPreEmitPass2() {
  // Schedule the expansion of AMOs at the last possible moment, avoiding the
  // possibility for other passes to break the requirements for forward
  // progress in the LR/SC block.
  addPass(createPPUExpandPseudoPass());
}



// FIXME

TargetPassConfig *PPUTargetMachine::createPassConfig(PassManagerBase &PM) {
  return new PPUPassConfig(*this, PM);
}


