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
  TRAP,

  // Masked control flow nodes.
  IF,
  ELSE,
  LOOP,

  // A uniform kernel return that terminates the wavefront.
  ENDPGM,

  // Return to a shader part's epilog code.
  RETURN_TO_EPILOG,

  DWORDADDR,
  FRACT,
  /// CLAMP value between 0.0 and 1.0. NaN clamped to 0, following clamp output
  /// modifier behavior with dx10_enable.
  CLAMP,

  // This is SETCC with the full mask result which is used for a compare with a
  // result bit per item in the wavefront.
  SETCC,
  SETREG,

  DENORM_MODE,
  // FP ops with input and output chain.
  FMA_W_CHAIN,
  FMUL_W_CHAIN,

  // SIN_HW, COS_HW - f32 for SI, 1 ULP max error, valid from -100 pi to 100 pi.
  // Denormals handled on some parts.
  COS_HW,
  SIN_HW,

  FMAX3,
  SMAX3,
  UMAX3,
  FMIN3,
  SMIN3,
  UMIN3,
  FMED3,
  SMED3,
  UMED3,
  FDOT2,

  URECIP,
  DIV_SCALE,
  DIV_FMAS,
  DIV_FIXUP,
  // For emitting ISD::FMAD when f32 denormals are enabled because mac/mad is
  // treated as an illegal operation.
  FMAD_FTZ,

  // RCP, RSQ - For f32, 1 ULP max error, no denormal handling.
  //            For f64, max error 2^29 ULP, handles denormals.
  RCP,
  RCP_IFLAG,
  RSQ,
  RSQ_CLAMP,
  LDEXP,
  FP_CLASS,
  CARRY,
  BORROW,
  DOT4,
  BFE_U32, // Extract range of bits with zero extension to 32-bits.
  BFE_I32, // Extract range of bits with sign extension to 32-bits.
  BFI, // (src0 & src1) | (~src0 & src2)
  BFM, // Insert a range of bits into a 32-bit word.
  FFBH_U32, // ctlz with -1 if input is zero.
  FFBH_I32,
  FFBL_B32, // cttz with -1 if input is zero.
  MUL_U24,
  MUL_I24,
  MULHI_U24,
  MULHI_I24,
  MAD_U24,
  MAD_I24,
  MAD_U64_U32,
  MAD_I64_I32,
  MUL_LOHI_I24,
  MUL_LOHI_U24,
  PERM,

  // These cvt_f32_ubyte* nodes need to remain consecutive and in order.
  CVT_F32_UBYTE0,
  CVT_F32_UBYTE1,
  CVT_F32_UBYTE2,
  CVT_F32_UBYTE3,

  // Convert two float 32 numbers into a single register holding two packed f16
  // with round to zero.
  CVT_PKRTZ_F16_F32,
  CVT_PKNORM_I16_F32,
  CVT_PKNORM_U16_F32,
  CVT_PK_I16_I32,
  CVT_PK_U16_U32,

  // Same as the standard node, except the high bits of the resulting integer
  // are known 0.
  FP_TO_FP16,

  // Wrapper around fp16 results that are known to zero the high bits.
  FP16_ZEXT,




  CONST_DATA_PTR,
  INIT_TMSK,
  INIT_TMSK_FROM_INPUT,

  PC_ADD_REL_OFFSET,
  LDS,
  KILL,
  DUMMY_CHAIN,


  FIRST_MEM_OPCODE_NUMBER = ISD::FIRST_TARGET_MEMORY_OPCODE,
  LOAD_D16_HI,
  LOAD_D16_LO,
  LOAD_D16_HI_I8,
  LOAD_D16_HI_U8,
  LOAD_D16_LO_I8,
  LOAD_D16_LO_U8,

  STORE_MSKOR,
  LOAD_CONSTANT,
  TBUFFER_STORE_FORMAT,
  TBUFFER_STORE_FORMAT_D16,
  TBUFFER_LOAD_FORMAT,
  TBUFFER_LOAD_FORMAT_D16,

  DS_ORDERED_COUNT,
  ATOMIC_CMP_SWAP,
  ATOMIC_INC,
  ATOMIC_DEC,
  ATOMIC_LOAD_FMIN,
  ATOMIC_LOAD_FMAX,

  BUFFER_LOAD,
  BUFFER_LOAD_UBYTE,
  BUFFER_LOAD_USHORT,
  BUFFER_LOAD_BYTE,
  BUFFER_LOAD_SHORT,
  BUFFER_LOAD_FORMAT,
  BUFFER_LOAD_FORMAT_D16,
  SBUFFER_LOAD,
  BUFFER_STORE,
  BUFFER_STORE_BYTE,
  BUFFER_STORE_SHORT,
  BUFFER_STORE_FORMAT,
  BUFFER_STORE_FORMAT_D16,
  BUFFER_ATOMIC_SWAP,
  BUFFER_ATOMIC_ADD,
  BUFFER_ATOMIC_SUB,
  BUFFER_ATOMIC_SMIN,
  BUFFER_ATOMIC_UMIN,
  BUFFER_ATOMIC_SMAX,
  BUFFER_ATOMIC_UMAX,
  BUFFER_ATOMIC_AND,
  BUFFER_ATOMIC_OR,
  BUFFER_ATOMIC_XOR,
  BUFFER_ATOMIC_INC,
  BUFFER_ATOMIC_DEC,
  BUFFER_ATOMIC_CMPSWAP,
  BUFFER_ATOMIC_FADD,
  BUFFER_ATOMIC_PK_FADD,
  ATOMIC_FADD,
  ATOMIC_PK_FADD,


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
protected:
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
  void PPUBaseTargetLowering_compute();

  /// \returns AMDGPUISD::FFBH_U32 node if the incoming \p Op may have been
  /// legalized from a smaller type VT. Need to match pre-legalized type because
  /// the generic legalization inserts the add/sub between the select and
  /// compare.
  SDValue getFFBX_U32(SelectionDAG &DAG, SDValue Op, const SDLoc &DL, unsigned Opc) const;

public:
  static unsigned numBitsUnsigned(SDValue Op, SelectionDAG &DAG);
  static unsigned numBitsSigned(SDValue Op, SelectionDAG &DAG);

protected:
  SDValue LowerEXTRACT_SUBVECTOR(SDValue Op, SelectionDAG &DAG) const;
  SDValue LowerCONCAT_VECTORS(SDValue Op, SelectionDAG &DAG) const;
  /// Split a vector store into multiple scalar stores.
  /// \returns The resulting chain.

  SDValue LowerFREM(SDValue Op, SelectionDAG &DAG) const;
  SDValue LowerFCEIL(SDValue Op, SelectionDAG &DAG) const;
  SDValue LowerFTRUNC(SDValue Op, SelectionDAG &DAG) const;
  SDValue LowerFRINT(SDValue Op, SelectionDAG &DAG) const;
  SDValue LowerFNEARBYINT(SDValue Op, SelectionDAG &DAG) const;

  SDValue LowerFROUND32_16(SDValue Op, SelectionDAG &DAG) const;
  SDValue LowerFROUND64(SDValue Op, SelectionDAG &DAG) const;
  SDValue LowerFROUND(SDValue Op, SelectionDAG &DAG) const;
  SDValue LowerFFLOOR(SDValue Op, SelectionDAG &DAG) const;
  SDValue LowerFLOG(SDValue Op, SelectionDAG &DAG,
                    double Log2BaseInverted) const;
  SDValue lowerFEXP(SDValue Op, SelectionDAG &DAG) const;

  SDValue LowerCTLZ_CTTZ(SDValue Op, SelectionDAG &DAG) const;

  SDValue LowerINT_TO_FP32(SDValue Op, SelectionDAG &DAG, bool Signed) const;
  SDValue LowerINT_TO_FP64(SDValue Op, SelectionDAG &DAG, bool Signed) const;
  SDValue LowerUINT_TO_FP(SDValue Op, SelectionDAG &DAG) const;
  SDValue LowerSINT_TO_FP(SDValue Op, SelectionDAG &DAG) const;

  SDValue LowerFP64_TO_INT(SDValue Op, SelectionDAG &DAG, bool Signed) const;
  SDValue LowerFP_TO_FP16(SDValue Op, SelectionDAG &DAG) const;
  SDValue LowerFP_TO_UINT(SDValue Op, SelectionDAG &DAG) const;
  SDValue LowerFP_TO_SINT(SDValue Op, SelectionDAG &DAG) const;

  SDValue LowerSIGN_EXTEND_INREG(SDValue Op, SelectionDAG &DAG) const;

protected:
  bool shouldCombineMemoryType(EVT VT) const;
  SDValue performLoadCombine(SDNode *N, DAGCombinerInfo &DCI) const;
  SDValue performStoreCombine(SDNode *N, DAGCombinerInfo &DCI) const;
  SDValue performAssertSZExtCombine(SDNode *N, DAGCombinerInfo &DCI) const;
  SDValue performIntrinsicWOChainCombine(SDNode *N, DAGCombinerInfo &DCI) const;

  SDValue splitBinaryBitConstantOpImpl(DAGCombinerInfo &DCI, const SDLoc &SL,
                                       unsigned Opc, SDValue LHS,
                                       uint32_t ValLo, uint32_t ValHi) const;
  SDValue performShlCombine(SDNode *N, DAGCombinerInfo &DCI) const;
  SDValue performSraCombine(SDNode *N, DAGCombinerInfo &DCI) const;
  SDValue performSrlCombine(SDNode *N, DAGCombinerInfo &DCI) const;
  SDValue performTruncateCombine(SDNode *N, DAGCombinerInfo &DCI) const;
  SDValue performMulCombine(SDNode *N, DAGCombinerInfo &DCI) const;
  SDValue performMulhsCombine(SDNode *N, DAGCombinerInfo &DCI) const;
  SDValue performMulhuCombine(SDNode *N, DAGCombinerInfo &DCI) const;
  SDValue performMulLoHi24Combine(SDNode *N, DAGCombinerInfo &DCI) const;
  SDValue performCtlz_CttzCombine(const SDLoc &SL, SDValue Cond, SDValue LHS,
                             SDValue RHS, DAGCombinerInfo &DCI) const;
  SDValue performSelectCombine(SDNode *N, DAGCombinerInfo &DCI) const;

  bool isConstantCostlierToNegate(SDValue N) const;
  SDValue performFNegCombine(SDNode *N, DAGCombinerInfo &DCI) const;
  SDValue performFAbsCombine(SDNode *N, DAGCombinerInfo &DCI) const;
  SDValue performRcpCombine(SDNode *N, DAGCombinerInfo &DCI) const;

  static EVT getEquivalentMemType(LLVMContext &Context, EVT VT);

  virtual SDValue LowerGlobalAddress(PPUMachineFunction *MFI, SDValue Op,
                                     SelectionDAG &DAG) const;

  /// Return 64-bit value Op as two 32-bit integers.
  std::pair<SDValue, SDValue> split64BitValue(SDValue Op,
                                              SelectionDAG &DAG) const;
  SDValue getLoHalf64(SDValue Op, SelectionDAG &DAG) const;
  SDValue getHiHalf64(SDValue Op, SelectionDAG &DAG) const;

  /// Split a vector type into two parts. The first part is a power of two
  /// vector. The second part is whatever is left over, and is a scalar if it
  /// would otherwise be a 1-vector.
  std::pair<EVT, EVT> getSplitDestVTs(const EVT &VT, SelectionDAG &DAG) const;

  /// Split a vector value into two parts of types LoVT and HiVT. HiVT could be
  /// scalar.
  std::pair<SDValue, SDValue> splitVector(const SDValue &N, const SDLoc &DL,
                                          const EVT &LoVT, const EVT &HighVT,
                                          SelectionDAG &DAG) const;

  /// Split a vector load into 2 loads of half the vector.
  SDValue SplitVectorLoad(SDValue Op, SelectionDAG &DAG) const;

  /// Widen a vector load from vec3 to vec4.
  SDValue WidenVectorLoad(SDValue Op, SelectionDAG &DAG) const;

  /// Split a vector store into 2 stores of half the vector.
  SDValue SplitVectorStore(SDValue Op, SelectionDAG &DAG) const;

  SDValue LowerSTORE(SDValue Op, SelectionDAG &DAG) const;
  SDValue LowerSDIVREM(SDValue Op, SelectionDAG &DAG) const;
  SDValue LowerUDIVREM(SDValue Op, SelectionDAG &DAG) const;
  SDValue LowerDIVREM24(SDValue Op, SelectionDAG &DAG, bool sign) const;
  void LowerUDIVREM64(SDValue Op, SelectionDAG &DAG,
                                    SmallVectorImpl<SDValue> &Results) const;

  void analyzeFormalArgumentsCompute(
    CCState &State,
    const SmallVectorImpl<ISD::InputArg> &Ins) const;

public:


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


  static bool allUsesHaveSourceMods(const SDNode *N,
                                    unsigned CostThreshold = 4);


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

  SDValue LowerDYNAMIC_STACKALLOC(SDValue Op,
                                  SelectionDAG &DAG) const;

  SDValue LowerOperation_compute(SDValue Op, SelectionDAG &DAG) const ;
  SDValue PerformDAGCombine_compute(SDNode *N, DAGCombinerInfo &DCI) const ;
  /*
  void ReplaceNodeResults_compute(SDNode * N,
                          SmallVectorImpl<SDValue> &Results,
                          SelectionDAG &DAG) const override;
*/
  SDValue combineFMinMaxLegacy(const SDLoc &DL, EVT VT, SDValue LHS,
                               SDValue RHS, SDValue True, SDValue False,
                               SDValue CC, DAGCombinerInfo &DCI) const;




  static CCAssignFn *CCAssignFnForCall(CallingConv::ID CC, bool IsVarArg);
  static CCAssignFn *CCAssignFnForReturn(CallingConv::ID CC, bool IsVarArg);




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

  MVT getFenceOperandTy(const DataLayout &DL) const override {
    return MVT::i32;
  }

  // AtomicExpansionKind shouldExpandAtomicRMWInIR(AtomicRMWInst *) const override;

  bool SelectFlatOffset(bool IsSigned, SelectionDAG &DAG, SDNode *N,
                        SDValue Addr, SDValue &VAddr, SDValue &Offset,
                        SDValue &SLC) const;

};



}

#endif
