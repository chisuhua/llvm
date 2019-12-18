set(LLVM_TARGET_DEFINITIONS PPU.td)

tablegen(LLVM PPUGenAsmMatcher.inc -gen-asm-matcher)
tablegen(LLVM PPUGenAsmWriter.inc -gen-asm-writer)
tablegen(LLVM PPUGenCallingConv.inc -gen-callingconv)
tablegen(LLVM PPUGenCompressInstEmitter.inc -gen-compress-inst-emitter)
tablegen(LLVM PPUGenDAGISel.inc -gen-dag-isel)
tablegen(LLVM PPUGenDisassemblerTables.inc -gen-disassembler)
#tablegen(LLVM PPUGenGlobalISel.inc -gen-global-isel)
tablegen(LLVM PPUGenInstrInfo.inc -gen-instr-info)
tablegen(LLVM PPUGenIntrinsicEnums.inc -gen-tgt-intrinsic-enums)
tablegen(LLVM PPUGenIntrinsicImpl.inc -gen-tgt-intrinsic-impl)
tablegen(LLVM PPUGenMCCodeEmitter.inc -gen-emitter)
tablegen(LLVM PPUGenMCPseudoLowering.inc -gen-pseudo-lowering)
tablegen(LLVM PPUGenRegisterBank.inc -gen-register-bank)
tablegen(LLVM PPUGenRegisterInfo.inc -gen-register-info)
tablegen(LLVM PPUGenSearchableTables.inc -gen-searchable-tables)
tablegen(LLVM PPUGenSubtargetInfo.inc -gen-subtarget)
tablegen(LLVM PPUGenSystemOperands.inc -gen-searchable-tables)

set(LLVM_TARGET_DEFINITIONS PPUGISel.td)
tablegen(LLVM PPUGenGlobalISel.inc -gen-global-isel)


add_public_tablegen_target(PPUCommonTableGen)

# TODO schi PPUOptimizeVSETVLUses.cpp is copied from rvv
add_llvm_target(PPUCodeGen
  PPUAsmPrinter.cpp
  PPUCallLowering.cpp
  PPUExpandPseudoInsts.cpp
  PPUFrameLowering.cpp
  PPUOptimizeVSETVLUses.cpp
  PPUInstrInfo.cpp
  PPUInstructionSelector.cpp
  PPUISelDAGToDAG.cpp
  PPUISelLowering.cpp
  PPUBaseISelLowering.cpp
  PPUILPSched.cpp
  PPULegalizerInfo.cpp
  PPUMCInstLower.cpp
  PPUMergeBaseOffset.cpp
  PPUMachineFunction.cpp
  PPUMachineFunctionInfo.cpp
  PPUMachineModuleInfo.cpp
  PPURegisterBankInfo.cpp
  PPURegisterInfo.cpp
  PPUSubtarget.cpp
  PPUTargetMachine.cpp
  PPUTargetObjectFile.cpp
  PPUTargetTransformInfo.cpp
  PPUPerfHintAnalysis.cpp
  PPUAlwaysInlinePass.cpp
  PPUAliasAnalysis.cpp
  PPUCodeGenPrepare.cpp
  PPUFoldOperands.cpp
  PPUFormMemoryClauses.cpp
  PPUIterativeScheduler.cpp
  PPULoadStoreOptimizer.cpp
  PPULowerControlFlow.cpp
  PPULowerI1Copies.cpp
  PPULowerSPRSpills.cpp
  PPUMacroFusion.cpp
  PPUMemoryLegalizer.cpp
  PPURegBankReassign.cpp
  PPURegPressure.cpp
  PPUOptimizeExecMasking.cpp
  PPUOptimizeExecMaskingPreRA.cpp
  PPUPreAllocateWWMRegs.cpp
  PPUSchedStrategy.cpp
  PPUUnifyDivergentExitNodes.cpp
  PPUUnifyMetadata.cpp
  PPUArgumentUsageInfo.cpp
  PPUAnnotateKernelFeatures.cpp
  PPUAnnotateUniformValues.cpp
  PPULowerIntrinsics.cpp
  PPULowerKernelArguments.cpp
  PPULowerKernelAttributes.cpp
  PPULowerReconvergingControlFlow.cpp
  PPURewriteOutArguments.cpp
  PPUPromoteAlloca.cpp
  PPUMachineScheduler.cpp
  PPUInline.cpp
  PPUFixVGPRCopies.cpp
  PPUAnnotateControlFlow.cpp
  )
#  PPUFixSGPRCopies.cpp
# PPUNSAReassign.cpp
# PPUIntrinsicInfo.cpp

# FIXME
add_subdirectory(AsmParser)
#add_subdirectory(Disassembler)
add_subdirectory(MCTargetDesc)
add_subdirectory(TargetInfo)
add_subdirectory(Utils)
