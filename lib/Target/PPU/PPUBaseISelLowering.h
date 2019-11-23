//===-- PPUISelLowering.h - PPU DAG Lowering Interface ------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines the interfaces that PPU uses to lower LLVM code into a
// selection DAG.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_PPU_PPUISELLOWERING_H
#define LLVM_LIB_TARGET_PPU_PPUISELLOWERING_H

#include "PPU.h"
#include "PPUMachineFunction.h"
#include "llvm/CodeGen/SelectionDAG.h"
#include "llvm/CodeGen/CallingConvLower.h"
#include "llvm/CodeGen/TargetLowering.h"

namespace llvm {
class PPUSubtarget;
namespace PPUISD {
enum NodeType : unsigned {
  FIRST_NUMBER = ISD::BUILTIN_OP_END,
  RET_FLAG,
  URET_FLAG,
  SRET_FLAG,
  MRET_FLAG,
  CALL,
  SELECT_CC,
  BuildPairF64,
  SplitF64,
  TAIL,
  // RV64I shifts, directly matching the semantics of the named PPU
  // instructions.
  SLLW,
  SRAW,
  SRLW,
  // 32-bit operations from RV64M that can't be simply matched with a pattern
  // at instruction selection time.
  DIVW,
  DIVUW,
  REMUW,
  // FPR32<->GPR transfer operations for RV64. Needed as an i32<->f32 bitcast
  // is not legal on RV64. FMV_W_X_RV64 matches the semantics of the FMV.W.X.
  // FMV_X_ANYEXTW_RV64 is similar to FMV.X.W but has an any-extended result.
  // This is a more convenient semantic for producing dagcombines that remove
  // unnecessary GPR->FPR->GPR moves.
  FMV_W_X_RV64,
  FMV_X_ANYEXTW_RV64,
  // READ_CYCLE_WIDE - A read of the 64-bit cycle CSR on a 32-bit target
  // (returns (Lo, Hi)). It takes a chain operand.
  READ_CYCLE_WIDE,
  SETVL,
  BROADCAST,

  // AMD
  TC_RETURN,

  // Masked control flow nodes.
  IF,
  ELSE,
  LOOP,

  // A uniform kernel return that terminates the wavefront.
  ENDPGM,

  // Return to a shader part's epilog code.
  RETURN_TO_EPILOG,

  // This is SETCC with the full mask result which is used for a compare with a
  // result bit per item in the wavefront.
  SETCC,
  SETREG,

  // RCP, RSQ - For f32, 1 ULP max error, no denormal handling.
  //            For f64, max error 2^29 ULP, handles denormals.
  RCP,
  BFE_U32, // Extract range of bits with zero extension to 32-bits.
  BFE_I32, // Extract range of bits with sign extension to 32-bits.
  BFI, // (src0 & src1) | (~src0 & src2)
  BFM, // Insert a range of bits into a 32-bit word.

  INIT_TMSK,
  INIT_TMSK_FROM_INPUT,

  KILL,
  DUMMY_CHAIN,


  FIRST_MEM_OPCODE_NUMBER = ISD::FIRST_TARGET_MEMORY_OPCODE,
  LOAD_D16_HI,
  LOAD_D16_LO,
  LOAD_D16_HI_I8,
  LOAD_D16_HI_U8,
  LOAD_D16_LO_I8,
  LOAD_D16_LO_U8,

  CONST_DATA_PTR,
  LOAD_CONSTANT,
  TBUFFER_STORE_FORMAT,
  TBUFFER_STORE_FORMAT_D16,
  TBUFFER_LOAD_FORMAT,
  TBUFFER_LOAD_FORMAT_D16,

  BUFFER_LOAD,
  BUFFER_LOAD_UBYTE,
  BUFFER_LOAD_USHORT,
  BUFFER_LOAD_BYTE,
  BUFFER_LOAD_SHORT,
  BUFFER_LOAD_FORMAT,
  BUFFER_LOAD_FORMAT_D16,
  BUFFER_STORE,
  BUFFER_STORE_BYTE,
  BUFFER_STORE_SHORT,
  BUFFER_STORE_FORMAT,
  BUFFER_STORE_FORMAT_D16,

  LAST_PPU_ISD_NUMBER
};
}

struct ArgDescriptor;

class PPUBaseTargetLowering : public TargetLowering {
  const PPUSubtarget &Subtarget;

public:
  explicit PPUBaseTargetLowering(const TargetMachine &TM,
                               const PPUSubtarget &STI);

  bool getTgtMemIntrinsic(IntrinsicInfo &Info, const CallInst &I,
                          MachineFunction &MF,
                          unsigned Intrinsic) const override;
  bool isLegalAddressingMode(const DataLayout &DL, const AddrMode &AM, Type *Ty,
                             unsigned AS,
                             Instruction *I = nullptr) const override;
  bool isLegalICmpImmediate(int64_t Imm) const override;
  bool isLegalAddImmediate(int64_t Imm) const override;
  bool isTruncateFree(Type *SrcTy, Type *DstTy) const override;
  bool isTruncateFree(EVT SrcVT, EVT DstVT) const override;
  bool isZExtFree(SDValue Val, EVT VT2) const override;
  bool isSExtCheaperThanZExt(EVT SrcVT, EVT DstVT) const override;

  bool hasBitPreservingFPLogic(EVT VT) const override;

  // Provide custom lowering hooks for some operations.
  SDValue LowerOperation(SDValue Op, SelectionDAG &DAG) const override;
  void ReplaceNodeResults(SDNode *N, SmallVectorImpl<SDValue> &Results,
                          SelectionDAG &DAG) const override;

  SDValue PerformDAGCombine(SDNode *N, DAGCombinerInfo &DCI) const override;

  unsigned ComputeNumSignBitsForTargetNode(SDValue Op,
                                           const APInt &DemandedElts,
                                           const SelectionDAG &DAG,
                                           unsigned Depth) const override;

  // This method returns the name of a target specific DAG node.
  const char *getTargetNodeName(unsigned Opcode) const override;

  ConstraintType getConstraintType(StringRef Constraint) const override;

  unsigned getInlineAsmMemConstraint(StringRef ConstraintCode) const override;

  std::pair<unsigned, const TargetRegisterClass *>
  getRegForInlineAsmConstraint(const TargetRegisterInfo *TRI,
                               StringRef Constraint, MVT VT) const override;

  void LowerAsmOperandForConstraint(SDValue Op, std::string &Constraint,
                                    std::vector<SDValue> &Ops,
                                    SelectionDAG &DAG) const override;

  MachineBasicBlock *
  EmitInstrWithCustomInserter(MachineInstr &MI,
                              MachineBasicBlock *BB) const override;

  EVT getSetCCResultType(const DataLayout &DL, LLVMContext &Context,
                         EVT VT) const override;

  bool convertSetCCLogicToBitwiseLogic(EVT VT) const override {
    return VT.isScalarInteger();
  }

  bool shouldInsertFencesForAtomic(const Instruction *I) const override {
    return isa<LoadInst>(I) || isa<StoreInst>(I);
  }
  Instruction *emitLeadingFence(IRBuilder<> &Builder, Instruction *Inst,
                                AtomicOrdering Ord) const override;
  Instruction *emitTrailingFence(IRBuilder<> &Builder, Instruction *Inst,
                                 AtomicOrdering Ord) const override;

  ISD::NodeType getExtendForAtomicOps() const override {
    return ISD::SIGN_EXTEND;
  }

  bool shouldExpandShift(SelectionDAG &DAG, SDNode *N) const override {
    if (DAG.getMachineFunction().getFunction().hasMinSize())
      return false;
    return true;
  }
  bool isDesirableToCommuteWithShift(const SDNode *N,
                                     CombineLevel Level) const override;

  /// If a physical register, this returns the register that receives the
  /// exception address on entry to an EH pad.
  unsigned
  getExceptionPointerRegister(const Constant *PersonalityFn) const override;

  /// If a physical register, this returns the register that receives the
  /// exception typeid on entry to a landing pad.
  unsigned
  getExceptionSelectorRegister(const Constant *PersonalityFn) const override;

  bool shouldExtendTypeInLibCall(EVT Type) const override;

private:
  void analyzeInputArgs(MachineFunction &MF, CCState &CCInfo,
                        const SmallVectorImpl<ISD::InputArg> &Ins,
                        bool IsRet) const;
  void analyzeOutputArgs(MachineFunction &MF, CCState &CCInfo,
                         const SmallVectorImpl<ISD::OutputArg> &Outs,
                         bool IsRet, CallLoweringInfo *CLI) const;
  // Lower incoming arguments, copy physregs into vregs
  SDValue LowerFormalArguments(SDValue Chain, CallingConv::ID CallConv,
                               bool IsVarArg,
                               const SmallVectorImpl<ISD::InputArg> &Ins,
                               const SDLoc &DL, SelectionDAG &DAG,
                               SmallVectorImpl<SDValue> &InVals) const override;
  bool CanLowerReturn(CallingConv::ID CallConv, MachineFunction &MF,
                      bool IsVarArg,
                      const SmallVectorImpl<ISD::OutputArg> &Outs,
                      LLVMContext &Context) const override;
protected:
  SDValue LowerReturn(SDValue Chain, CallingConv::ID CallConv, bool IsVarArg,
                      const SmallVectorImpl<ISD::OutputArg> &Outs,
                      const SmallVectorImpl<SDValue> &OutVals, const SDLoc &DL,
                      SelectionDAG &DAG) const override;
private:
  SDValue LowerCall(TargetLowering::CallLoweringInfo &CLI,
                    SmallVectorImpl<SDValue> &InVals) const override;
  bool shouldConvertConstantLoadToIntImm(const APInt &Imm,
                                         Type *Ty) const override {
    return true;
  }

  template <class NodeTy>
  SDValue getAddr(NodeTy *N, SelectionDAG &DAG, bool IsLocal = true) const;

  SDValue getStaticTLSAddr(GlobalAddressSDNode *N, SelectionDAG &DAG,
                           bool UseGOT) const;
  SDValue getDynamicTLSAddr(GlobalAddressSDNode *N, SelectionDAG &DAG) const;

  bool shouldConsiderGEPOffsetSplit() const override { return true; }
  SDValue lowerGlobalAddress(SDValue Op, SelectionDAG &DAG) const;
  SDValue lowerBlockAddress(SDValue Op, SelectionDAG &DAG) const;
  SDValue lowerConstantPool(SDValue Op, SelectionDAG &DAG) const;
  SDValue lowerGlobalTLSAddress(SDValue Op, SelectionDAG &DAG) const;
  SDValue lowerSELECT(SDValue Op, SelectionDAG &DAG) const;
  SDValue lowerVASTART(SDValue Op, SelectionDAG &DAG) const;
  SDValue lowerFRAMEADDR(SDValue Op, SelectionDAG &DAG) const;
  SDValue lowerRETURNADDR(SDValue Op, SelectionDAG &DAG) const;
  SDValue lowerShiftLeftParts(SDValue Op, SelectionDAG &DAG) const;
  SDValue lowerShiftRightParts(SDValue Op, SelectionDAG &DAG, bool IsSRA) const;

  // TODO copied from rvv
  SDValue lowerINTRINSIC_WO_CHAIN(SDValue Op, SelectionDAG &DAG) const;
  SDValue lowerSETVL(SDValue Op, SelectionDAG &DAG) const;
  SDValue lowerSPLAT_VECTOR(SDValue Op, SelectionDAG &DAG) const;

  bool isEligibleForTailCallOptimization(
      CCState &CCInfo, CallLoweringInfo &CLI, MachineFunction &MF,
      const SmallVector<CCValAssign, 16> &ArgLocs) const;

  TargetLowering::AtomicExpansionKind
  shouldExpandAtomicRMWInIR(AtomicRMWInst *AI) const override;
  virtual Value *emitMaskedAtomicRMWIntrinsic(
      IRBuilder<> &Builder, AtomicRMWInst *AI, Value *AlignedAddr, Value *Incr,
      Value *Mask, Value *ShiftAmt, AtomicOrdering Ord) const override;
  TargetLowering::AtomicExpansionKind
  shouldExpandAtomicCmpXchgInIR(AtomicCmpXchgInst *CI) const override;
  virtual Value *
  emitMaskedAtomicCmpXchgIntrinsic(IRBuilder<> &Builder, AtomicCmpXchgInst *CI,
                                   Value *AlignedAddr, Value *CmpVal,
                                   Value *NewVal, Value *Mask,
                                   AtomicOrdering Ord) const override;

 // from AMDGPUISelLowering
public:
  SDValue LowerReturn_compute(SDValue Chain, CallingConv::ID CallConv, bool isVarArg,
                      const SmallVectorImpl<ISD::OutputArg> &Outs,
                      const SmallVectorImpl<SDValue> &OutVals, const SDLoc &DL,
                      SelectionDAG &DAG) const ;

  SDValue addTokenForArgument(SDValue Chain,
                              SelectionDAG &DAG,
                              MachineFrameInfo &MFI,
                              int ClobberedFI) const;

  SDValue lowerUnhandledCall(CallLoweringInfo &CLI,
                             SmallVectorImpl<SDValue> &InVals,
                             StringRef Reason) const;
/*
  SDValue LowerCall_compute(CallLoweringInfo &CLI,
                    SmallVectorImpl<SDValue> &InVals) const override;
*/
  SDValue LowerOperation_compute(SDValue Op, SelectionDAG &DAG) const ;
  SDValue PerformDAGCombine_compute(SDNode *N, DAGCombinerInfo &DCI) const ;
  /*
  void ReplaceNodeResults_compute(SDNode * N,
                          SmallVectorImpl<SDValue> &Results,
                          SelectionDAG &DAG) const override;
*/


  virtual SDValue LowerGlobalAddress(PPUMachineFunction *MFI, SDValue Op,
                                     SelectionDAG &DAG) const;


  static CCAssignFn *CCAssignFnForCall(CallingConv::ID CC, bool IsVarArg);
  static CCAssignFn *CCAssignFnForReturn(CallingConv::ID CC, bool IsVarArg);
  void analyzeFormalArgumentsCompute(
    CCState &State,
    const SmallVectorImpl<ISD::InputArg> &Ins) const;
  bool mayIgnoreSignedZero(SDValue Op) const {
    if (getTargetMachine().Options.NoSignedZerosFPMath)
      return true;

    const auto Flags = Op.getNode()->getFlags();
    if (Flags.isDefined())
      return Flags.hasNoSignedZeros();

    return false;
  }

  static inline SDValue stripBitcast(SDValue Val) {
    return Val.getOpcode() == ISD::BITCAST ? Val.getOperand(0) : Val;
  }

  virtual SDNode *PostISelFolding(MachineSDNode *N,
                                  SelectionDAG &DAG) const = 0;

  /// Helper function that adds Reg to the LiveIn list of the DAG's
  /// MachineFunction.
  ///
  /// \returns a RegisterSDNode representing Reg if \p RawReg is true, otherwise
  /// a copy from the register.
  SDValue CreateLiveInRegister(SelectionDAG &DAG,
                               const TargetRegisterClass *RC,
                               unsigned Reg, EVT VT,
                               const SDLoc &SL,
                               bool RawReg = false) const;
  SDValue CreateLiveInRegister(SelectionDAG &DAG,
                               const TargetRegisterClass *RC,
                               unsigned Reg, EVT VT) const {
    return CreateLiveInRegister(DAG, RC, Reg, VT, SDLoc(DAG.getEntryNode()));
  }

  // Returns the raw live in register rather than a copy from it.
  SDValue CreateLiveInRegisterRaw(SelectionDAG &DAG,
                                  const TargetRegisterClass *RC,
                                  unsigned Reg, EVT VT) const {
    return CreateLiveInRegister(DAG, RC, Reg, VT, SDLoc(DAG.getEntryNode()), true);
  }

  /// Similar to CreateLiveInRegister, except value maybe loaded from a stack
  /// slot rather than passed in a register.
  SDValue loadStackInputValue(SelectionDAG &DAG,
                              EVT VT,
                              const SDLoc &SL,
                              int64_t Offset) const;

  SDValue storeStackInputValue(SelectionDAG &DAG,
                               const SDLoc &SL,
                               SDValue Chain,
                               SDValue ArgVal,
                               int64_t Offset) const;

  SDValue loadInputValue(SelectionDAG &DAG,
                         const TargetRegisterClass *RC,
                         EVT VT, const SDLoc &SL,
                         const ArgDescriptor &Arg) const;


  enum ImplicitParameter {
    FIRST_IMPLICIT,
    GRID_DIM = FIRST_IMPLICIT,
    GRID_OFFSET,
  };

  /// Helper function that returns the byte offset of the given
  /// type of implicit parameter.
  uint32_t getImplicitParameterOffset(const MachineFunction &MF,
                                      const ImplicitParameter Param) const;

};



}

#endif
