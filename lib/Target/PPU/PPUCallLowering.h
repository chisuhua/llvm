//===-- PPUCallLowering.h - Call lowering ---------------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
/// \file
/// This file describes how to lower LLVM calls to machine code calls.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_PPU_PPUCALLLOWERING_H
#define LLVM_LIB_TARGET_PPU_PPUCALLLOWERING_H

#include "llvm/CodeGen/CallingConvLower.h"
#include "llvm/CodeGen/GlobalISel/CallLowering.h"
#include "llvm/CodeGen/ValueTypes.h"

namespace llvm {

class PPUTargetLowering;
class MachineInstrBuilder;


class PPUCallLowering : public CallLowering {

public:
  PPUCallLowering(const PPUTargetLowering &TLI);

  bool lowerReturn(MachineIRBuilder &MIRBuiler, const Value *Val,
                   ArrayRef<Register> VRegs) const override;

  bool lowerFormalArguments(MachineIRBuilder &MIRBuilder, const Function &F,
                            ArrayRef<ArrayRef<Register>> VRegs) const override;

  bool lowerCall(MachineIRBuilder &MIRBuilder,
                 CallLoweringInfo &Info) const override;

// below is from AMDGPU
  Register lowerParameterPtr_compute(MachineIRBuilder &MIRBuilder, Type *ParamTy,
                             uint64_t Offset) const;

  void lowerParameter_compute(MachineIRBuilder &MIRBuilder, Type *ParamTy,
                      uint64_t Offset, unsigned Align,
                      Register DstReg) const;

  /// A function of this type is used to perform value split action.
  using SplitArgTy = std::function<void(ArrayRef<Register>, LLT, LLT, int)>;

  void splitToValueTypes(const ArgInfo &OrigArgInfo,
                         SmallVectorImpl<ArgInfo> &SplitArgs,
                         const DataLayout &DL, MachineRegisterInfo &MRI,
                         CallingConv::ID CallConv,
                         SplitArgTy SplitArg) const;
  bool lowerReturnVal_compute(MachineIRBuilder &MIRBuilder,
                      const Value *Val, ArrayRef<Register> VRegs,
                      MachineInstrBuilder &Ret) const;

public:
  bool lowerReturn_compute(MachineIRBuilder &MIRBuilder, const Value *Val,
                   ArrayRef<Register> VRegs) const ;

  bool lowerFormalArgumentsKernel(MachineIRBuilder &MIRBuilder,
                                  const Function &F,
                                  ArrayRef<ArrayRef<Register>> VRegs) const;

  bool lowerFormalArguments_compute(MachineIRBuilder &MIRBuilder, const Function &F,
                            ArrayRef<ArrayRef<Register>> VRegs) const ;
  static CCAssignFn *CCAssignFnForCall(CallingConv::ID CC, bool IsVarArg);
  static CCAssignFn *CCAssignFnForReturn(CallingConv::ID CC, bool IsVarArg);

};

} // end namespace llvm

#endif // LLVM_LIB_TARGET_PPU_PPUCALLLOWERING_H
