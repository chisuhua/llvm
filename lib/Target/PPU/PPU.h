//===-- PPU.h - Top-level interface for PPU -----------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the entry points for global functions defined in the LLVM
// PPU back-end.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_PPU_PPU_H
#define LLVM_LIB_TARGET_PPU_PPU_H

#include "Utils/PPUBaseInfo.h"
#include "llvm/Target/TargetMachine.h"

namespace llvm {
class PPURegisterBankInfo;
class PPUSubtarget;
class PPUTargetMachine;
class AsmPrinter;
class FunctionPass;
class InstructionSelector;
class MCInst;
class MCOperand;
class MachineInstr;
class MachineOperand;
class PassRegistry;

void LowerPPUMachineInstrToMCInst(const MachineInstr *MI, MCInst &OutMI,
                                    const AsmPrinter &AP);
bool LowerPPUMachineOperandToMCOperand(const MachineOperand &MO,
                                         MCOperand &MCOp, const AsmPrinter &AP);

FunctionPass *createPPUISelDag(PPUTargetMachine &TM);

FunctionPass *createPPUMergeBaseOffsetOptPass();
void initializePPUMergeBaseOffsetOptPass(PassRegistry &);

FunctionPass *createPPUExpandPseudoPass();
void initializePPUExpandPseudoPass(PassRegistry &);

InstructionSelector *createPPUInstructionSelector(const PPUTargetMachine &,
                                                    PPUSubtarget &,
                                                    PPURegisterBankInfo &);
// TODO schi copied from rvv
FunctionPass *createPPUOptimizeVSETVLUsesPass();
void initializePPUOptimizeVSETVLUsesPass(PassRegistry &);

FunctionPass *createPPUAnnotateUniformValues();
void initializePPUAnnotateUniformValuesPass(PassRegistry&);
extern char &PPUAnnotateUniformValuesPassID;


FunctionPass *createPPULowerReconvergingControlFlowPass();
void initializePPULowerReconvergingControlFlowPass(PassRegistry &);
extern char &PPULowerReconvergingControlFlowID;

void initializePPUPerfHintAnalysisPass(PassRegistry &);
extern char &PPUPerfHintAnalysisID;

void initializePPUArgumentUsageInfoPass(PassRegistry &);

}

// FIXME schi below copied from AMDGPU.h
/// OpenCL uses address spaces to differentiate between
/// various memory regions on the hardware. On the CPU
/// all of the address spaces point to the same memory,
/// however on the GPU, each address space points to
/// a separate piece of memory that is unique from other
/// memory locations.
namespace AMDGPUAS {
  enum : unsigned {
    // The maximum value for flat, generic, local, private, constant and region.
    MAX_AMDGPU_ADDRESS = 7,

    FLAT_ADDRESS = 0,     ///< Address space for flat memory.
    GLOBAL_ADDRESS = 1,   ///< Address space for global memory (RAT0, VTX0).
    REGION_ADDRESS = 2,   ///< Address space for region memory. (GDS)

    CONSTANT_ADDRESS = 4, ///< Address space for constant memory (VTX2).
    LOCAL_ADDRESS = 3,    ///< Address space for local memory.
    PRIVATE_ADDRESS = 5,  ///< Address space for private memory.

    CONSTANT_ADDRESS_32BIT = 6, ///< Address space for 32-bit constant memory.

    BUFFER_FAT_POINTER = 7, ///< Address space for 160-bit buffer fat pointers.

    /// Address space for direct addressible parameter memory (CONST0).
    PARAM_D_ADDRESS = 6,
    /// Address space for indirect addressible parameter memory (VTX1).
    PARAM_I_ADDRESS = 7,

    // Do not re-order the CONSTANT_BUFFER_* enums.  Several places depend on
    // this order to be able to dynamically index a constant buffer, for
    // example:
    //
    // ConstantBufferAS = CONSTANT_BUFFER_0 + CBIdx

    CONSTANT_BUFFER_0 = 8,
    CONSTANT_BUFFER_1 = 9,
    CONSTANT_BUFFER_2 = 10,
    CONSTANT_BUFFER_3 = 11,
    CONSTANT_BUFFER_4 = 12,
    CONSTANT_BUFFER_5 = 13,
    CONSTANT_BUFFER_6 = 14,
    CONSTANT_BUFFER_7 = 15,
    CONSTANT_BUFFER_8 = 16,
    CONSTANT_BUFFER_9 = 17,
    CONSTANT_BUFFER_10 = 18,
    CONSTANT_BUFFER_11 = 19,
    CONSTANT_BUFFER_12 = 20,
    CONSTANT_BUFFER_13 = 21,
    CONSTANT_BUFFER_14 = 22,
    CONSTANT_BUFFER_15 = 23,

    // Some places use this if the address space can't be determined.
    UNKNOWN_ADDRESS_SPACE = ~0u,
  };
}

#endif
