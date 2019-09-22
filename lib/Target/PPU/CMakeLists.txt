set(LLVM_TARGET_DEFINITIONS PPU.td)

tablegen(LLVM PPUGenAsmMatcher.inc -gen-asm-matcher)
tablegen(LLVM PPUGenAsmWriter.inc -gen-asm-writer)
tablegen(LLVM PPUGenCompressInstEmitter.inc -gen-compress-inst-emitter)
tablegen(LLVM PPUGenDAGISel.inc -gen-dag-isel)
tablegen(LLVM PPUGenDisassemblerTables.inc -gen-disassembler)
tablegen(LLVM PPUGenGlobalISel.inc -gen-global-isel)
tablegen(LLVM PPUGenInstrInfo.inc -gen-instr-info)
tablegen(LLVM PPUGenMCCodeEmitter.inc -gen-emitter)
tablegen(LLVM PPUGenMCPseudoLowering.inc -gen-pseudo-lowering)
tablegen(LLVM PPUGenRegisterBank.inc -gen-register-bank)
tablegen(LLVM PPUGenRegisterInfo.inc -gen-register-info)
tablegen(LLVM PPUGenSubtargetInfo.inc -gen-subtarget)
tablegen(LLVM PPUGenSystemOperands.inc -gen-searchable-tables)

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
  PPULegalizerInfo.cpp
  PPULowerReconvergingControlFlow.cpp
  PPUAnnotateUniformValues.cpp
  PPUMCInstLower.cpp
  PPUMergeBaseOffset.cpp
  PPURegisterBankInfo.cpp
  PPURegisterInfo.cpp
  PPUSubtarget.cpp
  PPUTargetMachine.cpp
  PPUTargetObjectFile.cpp
  PPUTargetTransformInfo.cpp
  )

add_subdirectory(AsmParser)
add_subdirectory(Disassembler)
add_subdirectory(MCTargetDesc)
add_subdirectory(TargetInfo)
add_subdirectory(Utils)
