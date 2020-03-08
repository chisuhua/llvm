//===-- PPUAsmPrinter.h - Print PPU assembly code ---------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
/// \file
/// PPU Assembly printer class.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_PPU_PPUASMPRINTER_H
#define LLVM_LIB_TARGET_PPU_PPUASMPRINTER_H

#include "PPU.h"
#include "PPUKernelCodeT.h"
#include "PPUPPSMetadataStreamer.h"
#include "PPTProgramInfo.h"
#include "MCTargetDesc/PPUTargetStreamer.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/CodeGen/AsmPrinter.h"
#include "llvm/Support/PPUKernelDescriptor.h"
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace llvm {

class PPUMachineFunction;
// class PPUTargetStreamer;
class MCCodeEmitter;
class MCOperand;
class PPUSubtarget;

class PPUAsmPrinter final : public AsmPrinter {
private:
  // Track resource usage for callee functions.
  struct PPTFunctionResourceInfo {
    // Track the number of explicitly used VGPRs. Special registers reserved at
    // the end are tracked separately.
    int32_t NumVGPR = 0;
    int32_t NumExplicitSGPR = 0;
    uint64_t PrivateSegmentSize = 0;
    bool UsesVCC = false;
    bool UsesFlatScratch = false;
    bool HasDynamicallySizedStack = false;
    bool HasRecursion = false;

    int32_t getTotalNumSGPRs(const PPUSubtarget &ST) const;
  };

  PPTProgramInfo CurrentProgramInfo;
  DenseMap<const Function *, PPTFunctionResourceInfo> CallGraphResourceInfo;

  std::unique_ptr<PPU::PPSMD::MetadataStreamer> PPSMetadataStream;

  MCCodeEmitter *DumpCodeInstEmitter = nullptr;

  uint64_t getFunctionCodeSize(const MachineFunction &MF) const;
  PPTFunctionResourceInfo analyzeResourceUsage(const MachineFunction &MF) const;

  void getPPTProgramInfo(PPTProgramInfo &Out, const MachineFunction &MF);
  void getAmdKernelCode(amd_kernel_code_t &Out, const PPTProgramInfo &KernelInfo,
                        const MachineFunction &MF) const;
  void findNumUsedRegistersSI(const MachineFunction &MF,
                              unsigned &NumSGPR,
                              unsigned &NumVGPR) const;

  /// Emit register usage information so that the GPU driver
  /// can correctly setup the GPU state.
  void EmitProgramInfoSI(const MachineFunction &MF,
                         const PPTProgramInfo &KernelInfo);
  void emitCommonFunctionComments(uint32_t NumVGPR,
                                  uint32_t NumSGPR,
                                  uint64_t ScratchSize,
                                  uint64_t CodeSize,
                                  const PPUMachineFunction* MFI);

  uint16_t getPpsKernelCodeProperties(
      const MachineFunction &MF) const;

  pps::kernel_descriptor_t getPpsKernelDescriptor(
      const MachineFunction &MF,
      const PPTProgramInfo &PI) const;

public:
  explicit PPUAsmPrinter(TargetMachine &TM,
                            std::unique_ptr<MCStreamer> Streamer);

  StringRef getPassName() const override;

  const MCSubtargetInfo* getGlobalSTI() const;

  PPUTargetStreamer* getTargetStreamer() const;

  bool doFinalization(Module &M) override;
  bool runOnMachineFunction(MachineFunction &MF) override;

  /// Lower the specified LLVM Constant to an MCExpr.
  /// The AsmPrinter::lowerConstantof does not know how to lower
  /// addrspacecast, therefore they should be lowered by this function.
  const MCExpr *lowerConstant(const Constant *CV) override;

  void EmitFunctionBodyStart() override;

  void EmitFunctionBodyEnd() override;

  // void EmitFunctionEntryLabel() override;

  void EmitBasicBlockStart(const MachineBasicBlock &MBB) override;

  void EmitGlobalVariable(const GlobalVariable *GV) override;

  void EmitStartOfAsmFile(Module &M) override;

  void EmitEndOfAsmFile(Module &M) override;

  bool isBlockOnlyReachableByFallthrough(
    const MachineBasicBlock *MBB) const override;


  /// Implemented in PPUMCInstLower.cpp
  void EmitInstruction(const MachineInstr *MI) override;

  bool PrintAsmOperand(const MachineInstr *MI, unsigned OpNo,
                       const char *ExtraCode, raw_ostream &O) override;

  bool PrintAsmMemoryOperand(const MachineInstr *MI, unsigned OpNo,
                             const char *ExtraCode, raw_ostream &OS) override;

  void EmitToStreamer(MCStreamer &S, const MCInst &Inst);

  /// tblgen'erated driver function for lowering simple MI->MC pseudo
  /// instructions.
  bool emitPseudoExpansionLowering(MCStreamer &OutStreamer,
                                   const MachineInstr *MI);

  /// Wrapper for MCInstLowering.lowerOperand() for the tblgen'erated
  /// pseudo lowering.
  bool lowerOperand(const MachineOperand &MO, MCOperand &MCOp) const;

protected:
  std::vector<std::string> DisasmLines, HexLines;
  size_t DisasmLineMaxLen;
};

} // end namespace llvm

#endif // LLVM_LIB_TARGET_PPU_PPUASMPRINTER_H
