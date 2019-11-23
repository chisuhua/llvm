//===-- PPUISelDAGToDAG.cpp - A dag to dag inst selector for PPU ------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines an instruction selector for the PPU target.
//
//===----------------------------------------------------------------------===//

#include "PPU.h"
#include "PPUArgumentUsageInfo.h"
#include "PPUISelLowering.h" // For AMDGPUISD
#include "PPUInstrInfo.h"
#include "PPUPerfHintAnalysis.h"
#include "PPUTargetMachine.h"
#include "PPUMachineFunctionInfo.h"
#include "PPURegisterInfo.h"
#include "MCTargetDesc/PPUMCTargetDesc.h"
#include "Utils/PPUMatInt.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/Analysis/LegacyDivergenceAnalysis.h"
#include "llvm/Analysis/ValueTracking.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/SelectionDAG.h"
#include "llvm/CodeGen/SelectionDAGISel.h"
#include "llvm/CodeGen/SelectionDAGNodes.h"
#include "llvm/IR/BasicBlock.h"
#ifdef EXPENSIVE_CHECKS
#include "llvm/IR/Dominators.h"
#endif
#include "llvm/Support/Debug.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/Support/raw_ostream.h"
using namespace llvm;

#define DEBUG_TYPE "ppu-isel"

//===----------------------------------------------------------------------===//
// Instruction Selector Implementation
//===----------------------------------------------------------------------===//

namespace {

static bool isNullConstantOrUndef(SDValue V) {
  if (V.isUndef())
    return true;

  ConstantSDNode *Const = dyn_cast<ConstantSDNode>(V);
  return Const != nullptr && Const->isNullValue();
}

static bool getConstantValue(SDValue N, uint32_t &Out) {
  // This is only used for packed vectors, where ussing 0 for undef should
  // always be good.
  if (N.isUndef()) {
    Out = 0;
    return true;
  }

  if (const ConstantSDNode *C = dyn_cast<ConstantSDNode>(N)) {
    Out = C->getAPIntValue().getSExtValue();
    return true;
  }

  if (const ConstantFPSDNode *C = dyn_cast<ConstantFPSDNode>(N)) {
    Out = C->getValueAPF().bitcastToAPInt().getSExtValue();
    return true;
  }

  return false;
}

// TODO: Handle undef as zero
static SDNode *packConstantV2I16(const SDNode *N, SelectionDAG &DAG,
                                 bool Negate = false) {
  assert(N->getOpcode() == ISD::BUILD_VECTOR && N->getNumOperands() == 2);
  uint32_t LHSVal, RHSVal;
  if (getConstantValue(N->getOperand(0), LHSVal) &&
      getConstantValue(N->getOperand(1), RHSVal)) {
    SDLoc SL(N);
    uint32_t K = Negate ?
      (-LHSVal & 0xffff) | (-RHSVal << 16) :
      (LHSVal & 0xffff) | (RHSVal << 16);
    return DAG.getMachineNode(PPU::SMOV, SL, N->getValueType(0),
                              DAG.getTargetConstant(K, SL, MVT::i32));
  }

  return nullptr;
}

static SDNode *packNegConstantV2I16(const SDNode *N, SelectionDAG &DAG) {
  return packConstantV2I16(N, DAG, true);
}

// PPU-specific code to select PPU machine instructions for
// SelectionDAG operations.
class PPUDAGToDAGISel final : public SelectionDAGISel {
  const PPUSubtarget *Subtarget;
  bool EnableReconvergeCFG;

public:
  explicit PPUDAGToDAGISel(PPUTargetMachine *TM = nullptr,
                              CodeGenOpt::Level OptLevel = CodeGenOpt::Default)
      : SelectionDAGISel(*TM, OptLevel) {
    EnableReconvergeCFG = TM->getSubtargetImpl()->enableReconvergeCFG();
  }

  StringRef getPassName() const override {
    return "PPU DAG->DAG Pattern Instruction Selection";
  }

  ~PPUDAGToDAGISel() override = default;

  void getAnalysisUsage(AnalysisUsage &AU) const override {
    AU.addRequired<PPUArgumentUsageInfo>();
    AU.addRequired<LegacyDivergenceAnalysis>();
#ifdef EXPENSIVE_CHECKS
    AU.addRequired<DominatorTreeWrapperPass>();
    AU.addRequired<LoopInfoWrapperPass>();
#endif
    SelectionDAGISel::getAnalysisUsage(AU);
  }

  static char ID;

  bool matchLoadD16FromBuildVector(SDNode *N) const; // AMD

  bool runOnMachineFunction(MachineFunction &MF) override {
#ifdef EXPENSIVE_CHECKS
  DominatorTree & DT = getAnalysis<DominatorTreeWrapperPass>().getDomTree();
  LoopInfo * LI = &getAnalysis<LoopInfoWrapperPass>().getLoopInfo();
  for (auto &L : LI->getLoopsInPreorder()) {
    assert(L->isLCSSAForm(DT));
  }
#endif
    Subtarget = &MF.getSubtarget<PPUSubtarget>();
    return SelectionDAGISel::runOnMachineFunction(MF);
  }

  void PreprocessISelDAG() override;  // AMD
  void PostprocessISelDAG() override;

  void Select(SDNode *Node) override;

  // AMD begin
protected:
  void SelectBuildVector(SDNode *N, unsigned RegClassID); // AMD

private:
  std::pair<SDValue, SDValue> foldFrameIndex(SDValue N) const;
  bool isNoNanSrc(SDValue N) const;
  /* FIXME
  bool isInlineImmediate(const SDNode *N, bool Negated = false) const;
  bool isNegInlineImmediate(const SDNode *N) const {
    return isInlineImmediate(N, true);
  }
  */
  SDValue getHi16Elt(SDValue In) const;

  bool isUniformBr(const SDNode *N) const;

  // MachineSDNode *buildSMovImm64(SDLoc &DL, uint64_t Val, EVT VT) const;
  const TargetRegisterClass *getOperandRegClass(SDNode *N, unsigned OpNo) const;

  bool isCBranchSCC(const SDNode *N) const;
  void SelectBRCOND(SDNode *N);

  // ... FIXME to port from PPU
  // AMD end

  bool SelectInlineAsmMemoryOperand(const SDValue &Op, unsigned ConstraintID,
                                    std::vector<SDValue> &OutOps) override;

  bool SelectAddrFI(SDValue Addr, SDValue &Base);

protected:

// Include the pieces autogenerated from the target description.
#include "PPUGenDAGISel.inc"

private:
  void doPeepholeLoadStoreADDI();
};

static SDValue stripBitcast(SDValue Val) {
  return Val.getOpcode() == ISD::BITCAST ? Val.getOperand(0) : Val;
}

// Figure out if this is really an extract of the high 16-bits of a dword.
static bool isExtractHiElt(SDValue In, SDValue &Out) {
  In = stripBitcast(In);
  if (In.getOpcode() != ISD::TRUNCATE)
    return false;

  SDValue Srl = In.getOperand(0);
  if (Srl.getOpcode() == ISD::SRL) {
    if (ConstantSDNode *ShiftAmt = dyn_cast<ConstantSDNode>(Srl.getOperand(1))) {
      if (ShiftAmt->getZExtValue() == 16) {
        Out = stripBitcast(Srl.getOperand(0));
        return true;
      }
    }
  }

  return false;
}

// Look through operations that obscure just looking at the low 16-bits of the
// same register.
static SDValue stripExtractLoElt(SDValue In) {
  if (In.getOpcode() == ISD::TRUNCATE) {
    SDValue Src = In.getOperand(0);
    if (Src.getValueType().getSizeInBits() == 32)
      return stripBitcast(Src);
  }

  return In;
}


}  // end anonymous namespace


INITIALIZE_PASS_BEGIN(PPUDAGToDAGISel, "ppu-isel",
                      "PPU DAG->DAG Pattern Instruction Selection", false, false)
INITIALIZE_PASS_DEPENDENCY(PPUArgumentUsageInfo)
INITIALIZE_PASS_DEPENDENCY(PPUPerfHintAnalysis)
INITIALIZE_PASS_DEPENDENCY(LegacyDivergenceAnalysis)
#ifdef EXPENSIVE_CHECKS
INITIALIZE_PASS_DEPENDENCY(DominatorTreeWrapperPass)
INITIALIZE_PASS_DEPENDENCY(LoopInfoWrapperPass)
#endif
INITIALIZE_PASS_END(PPUDAGToDAGISel, "ppu-isel",
                    "PPU DAG->DAG Pattern Instruction Selection", false, false)

char PPUDAGToDAGISel::ID = 0;
char &llvm::PPUDAGToDAGISelID = PPUDAGToDAGISel::ID;


// This pass converts a legalized DAG into a PPU-specific DAG, ready
// for instruction scheduling.
FunctionPass *llvm::createPPUISelDag(PPUTargetMachine &TM,
                                        CodeGenOpt::Level OptLevel) {
  return new PPUDAGToDAGISel(&TM, OptLevel);
}


static SDNode *selectImm(SelectionDAG *CurDAG, const SDLoc &DL, int64_t Imm,
                         MVT XLenVT) {
  PPUMatInt::InstSeq Seq;
  PPUMatInt::generateInstSeq(Imm, XLenVT == MVT::i64, Seq);

  SDNode *Result;
  SDValue SrcReg = CurDAG->getRegister(PPU::X0, XLenVT);
  for (PPUMatInt::Inst &Inst : Seq) {
    SDValue SDImm = CurDAG->getTargetConstant(Inst.Imm, DL, XLenVT);
    if (Inst.Opc == PPU::LUI)
      Result = CurDAG->getMachineNode(PPU::LUI, DL, XLenVT, SDImm);
    else
      Result = CurDAG->getMachineNode(Inst.Opc, DL, XLenVT, SrcReg, SDImm);

    // Only the first instruction has X0 as its source.
    SrcReg = SDValue(Result, 0);
  }

  return Result;
}

// Returns true if the Node is an ISD::AND with a constant argument. If so,
// set Mask to that constant value.
static bool isConstantMask(SDNode *Node, uint64_t &Mask) {
  if (Node->getOpcode() == ISD::AND &&
      Node->getOperand(1).getOpcode() == ISD::Constant) {
    Mask = cast<ConstantSDNode>(Node->getOperand(1))->getZExtValue();
    return true;
  }
  return false;
}

static unsigned selectSGPRVectorRegClassID(unsigned NumVectorElts) {
  switch (NumVectorElts) {
    /* FIXME
  case 1:
    return PPU::SReg_32_XX0RegClassID;
  case 2:
    return PPU::SReg_64RegClassID;
  case 4:
    return PPU::SReg_128RegClassID;
  case 8:
    return PPU::SReg_256RegClassID;
  case 16:
    return PPU::SReg_512RegClassID;
    */
  }

  llvm_unreachable("invalid vector size");
}

static bool getConstantValue(SDValue N, uint32_t &Out) {
  if (const ConstantSDNode *C = dyn_cast<ConstantSDNode>(N)) {
    Out = C->getAPIntValue().getZExtValue();
    return true;
  }

  if (const ConstantFPSDNode *C = dyn_cast<ConstantFPSDNode>(N)) {
    Out = C->getValueAPF().bitcastToAPInt().getZExtValue();
    return true;
  }

  return false;
}

void PPUDAGToDAGISel::Select(SDNode *Node) {
  // If we have a custom node, we have already selected.
  if (Node->isMachineOpcode()) {
    LLVM_DEBUG(dbgs() << "== "; Node->dump(CurDAG); dbgs() << "\n");
    Node->setNodeId(-1);
    return;
  }

  // Instruction Selection not handled by the auto-generated tablegen selection
  // should be handled here.
  unsigned Opcode = Node->getOpcode();

  // be used since AMD code use them
  SDNode *N = Node;
  unsigned int Opc = Opcode;

  MVT XLenVT = Subtarget->getXLenVT();
  SDLoc DL(Node);
  EVT VT = Node->getValueType(0);

  switch (Opcode) {
  case ISD::Constant: {
    auto ConstNode = cast<ConstantSDNode>(Node);
    if (VT == XLenVT && ConstNode->isNullValue()) {
      SDValue New = CurDAG->getCopyFromReg(CurDAG->getEntryNode(), SDLoc(Node),
                                           PPU::X0, XLenVT);
      ReplaceNode(Node, New.getNode());
      return;
    }
    int64_t Imm = ConstNode->getSExtValue();
    if (XLenVT == MVT::i64) {
      ReplaceNode(Node, selectImm(CurDAG, SDLoc(Node), Imm, XLenVT));
      return;
    }
    break;
  }
  case ISD::FrameIndex: {
    SDValue Imm = CurDAG->getTargetConstant(0, DL, XLenVT);
    int FI = cast<FrameIndexSDNode>(Node)->getIndex();
    SDValue TFI = CurDAG->getTargetFrameIndex(FI, VT);
    ReplaceNode(Node, CurDAG->getMachineNode(PPU::ADDI, DL, VT, TFI, Imm));
    return;
  }
  case ISD::SRL: {
    if (!Subtarget->is64Bit())
      break;
    SDValue Op0 = Node->getOperand(0);
    SDValue Op1 = Node->getOperand(1);
    uint64_t Mask;
    // Match (srl (and val, mask), imm) where the result would be a
    // zero-extended 32-bit integer. i.e. the mask is 0xffffffff or the result
    // is equivalent to this (SimplifyDemandedBits may have removed lower bits
    // from the mask that aren't necessary due to the right-shifting).
    if (Op1.getOpcode() == ISD::Constant &&
        isConstantMask(Op0.getNode(), Mask)) {
      uint64_t ShAmt = cast<ConstantSDNode>(Op1.getNode())->getZExtValue();

      if ((Mask | maskTrailingOnes<uint64_t>(ShAmt)) == 0xffffffff) {
        SDValue ShAmtVal =
            CurDAG->getTargetConstant(ShAmt, SDLoc(Node), XLenVT);
        CurDAG->SelectNodeTo(Node, PPU::SRLIW, XLenVT, Op0.getOperand(0),
                             ShAmtVal);
        return;
      }
    }
    break;
  }
  case PPUISD::READ_CYCLE_WIDE:
    assert(!Subtarget->is64Bit() && "READ_CYCLE_WIDE is only used on ppu");

    ReplaceNode(Node, CurDAG->getMachineNode(PPU::ReadCycleWide, DL, MVT::i32,
                                             MVT::i32, MVT::Other,
                                             Node->getOperand(0)));
    return;
  case ISD::SCALAR_TO_VECTOR:
  case ISD::BUILD_VECTOR: {
    EVT VT = N->getValueType(0);
    unsigned NumVectorElts = VT.getVectorNumElements();
    if (VT.getScalarSizeInBits() == 16) {
      if (Opc == ISD::BUILD_VECTOR && NumVectorElts == 2) {
        if (SDNode *Packed = packConstantV2I16(N, *CurDAG)) {
          ReplaceNode(N, Packed);
          return;
        }
      }

      break;
    }

    assert(VT.getVectorElementType().bitsEq(MVT::i32));
    unsigned RegClassID = selectSGPRVectorRegClassID(NumVectorElts);
    SelectBuildVector(N, RegClassID);
    return;
  }
  case ISD::BRCOND:
    SelectBRCOND(Node);
    return;
  }
  // Select the default instruction.
  SelectCode(Node);
}

bool PPUDAGToDAGISel::SelectInlineAsmMemoryOperand(
    const SDValue &Op, unsigned ConstraintID, std::vector<SDValue> &OutOps) {
  switch (ConstraintID) {
  case InlineAsm::Constraint_i:
  case InlineAsm::Constraint_m:
    // We just support simple memory operands that have a single address
    // operand and need no special handling.
    OutOps.push_back(Op);
    return false;
  case InlineAsm::Constraint_A:
    OutOps.push_back(Op);
    return false;
  default:
    break;
  }

  return true;
}

SDValue PPUDAGToDAGISel::getHi16Elt(SDValue In) const {
  if (In.isUndef())
    return CurDAG->getUNDEF(MVT::i32);

  if (ConstantSDNode *C = dyn_cast<ConstantSDNode>(In)) {
    SDLoc SL(In);
    return CurDAG->getConstant(C->getZExtValue() << 16, SL, MVT::i32);
  }

  if (ConstantFPSDNode *C = dyn_cast<ConstantFPSDNode>(In)) {
    SDLoc SL(In);
    return CurDAG->getConstant(
      C->getValueAPF().bitcastToAPInt().getZExtValue() << 16, SL, MVT::i32);
  }

  SDValue Src;
  if (isExtractHiElt(In, Src))
    return Src;

  return SDValue();
}


bool PPUDAGToDAGISel::SelectAddrFI(SDValue Addr, SDValue &Base) {
  if (auto FIN = dyn_cast<FrameIndexSDNode>(Addr)) {
    Base = CurDAG->getTargetFrameIndex(FIN->getIndex(), Subtarget->getXLenVT());
    return true;
  }
  return false;
}

bool PPUDAGToDAGISel::isUniformBr(const SDNode *N) const {
  const BasicBlock *BB = FuncInfo->MBB->getBasicBlock();
  const Instruction *Term = BB->getTerminator();
  return Term->getMetadata("ppu.uniform") ||
         Term->getMetadata("structurizecfg.uniform");
}

bool PPUDAGToDAGISel::isCBranchSCC(const SDNode *N) const {
  assert(N->getOpcode() == ISD::BRCOND);
  if (!N->hasOneUse())
    return false;

  SDValue Cond = N->getOperand(1);
  if (Cond.getOpcode() == ISD::CopyToReg)
    Cond = Cond.getOperand(2);

  if (Cond.getOpcode() != ISD::SETCC || !Cond.hasOneUse())
    return false;

  MVT VT = Cond.getOperand(0).getSimpleValueType();
  if (VT == MVT::i32)
    return true;

  assert(VT != MVT::i64); // TODO i think VT won't be i64, but need verify
/*
  if (VT == MVT::i64) {
    auto ST = static_cast<const PPUSubtarget *>(Subtarget);

    ISD::CondCode CC = cast<CondCodeSDNode>(Cond.getOperand(2))->get();
    return (CC == ISD::SETEQ || CC == ISD::SETNE) && ST->hasScalarCompareEq64();
  }
*/
  return false;
}

void PPUDAGToDAGISel::SelectBRCOND(SDNode *N) {
  SDValue Cond = N->getOperand(1);

  if (Cond.isUndef()) {
    CurDAG->SelectNodeTo(N, PPU::SI_BR_UNDEF, MVT::Other,
                         N->getOperand(2), N->getOperand(0));
    return;
  }

  const PPUSubtarget *ST = static_cast<const PPUSubtarget *>(Subtarget);
  const PPURegisterInfo *TRI = ST->getRegisterInfo();

  bool UseSCCBr = isCBranchSCC(N) && isUniformBr(N);
  if (!UseSCCBr && EnableReconvergeCFG) {
    // Default pattern matching selects SI_NON_UNIFORM_BRCOND_PSEUDO.
    SelectCode(N);
    return;
  }
  // assert("SelectBRCOND UseSCCBr is true");

  // unsigned BrOp = UseSCCBr ? PPU::S_CBRANCH_SCC1 : PPU::S_CBRANCH_VCCNZ;
  // FIXME schi we should use vector version BEQ
  unsigned BrOp = UseSCCBr ? PPU::BEQ : PPU::BEQ;
  // unsigned CondReg = UseSCCBr ? (unsigned)PPU::SCC : TRI->getVCC();
  SDLoc SL(N);
  SDValue VCC;

  if (UseSCCBr) {
    // VCC = CurDAG->getCopyToReg(N->getOperand(0), SL, Cond, Cond, Cond.getValue(1));
    /*
    CurDAG->SelectNodeTo(N, BrOp, MVT::Other,
                       N->getOperand(2), // Basic Block
                       Cond.getValue(0));
                       */
    SelectCode(N);
  } else {
    unsigned CondReg = TRI->getVCC();
    // This is the case that we are selecting to S_CBRANCH_VCCNZ.  We have not
    // analyzed what generates the vcc value, so we do not know whether vcc
    // bits for disabled lanes are 0.  Thus we need to mask out bits for
    // disabled lanes.
    //
    // For the case that we select S_CBRANCH_SCC1 and it gets
    // changed to S_CBRANCH_VCCNZ in SIFixSGPRCopies, SIFixSGPRCopies calls
    // SIInstrInfo::moveToVALU which inserts the S_AND).
    //
    // We could add an analysis of what generates the vcc value here and omit
    // the S_AND when is unnecessary. But it would be better to add a separate
    // pass after SIFixSGPRCopies to do the unnecessary S_AND removal, so it
    // catches both cases.
    Cond = SDValue(CurDAG->getMachineNode(PPU::AND,
                     SL, MVT::i1,
                     CurDAG->getRegister(PPU::TMSK,
                                         MVT::i1),
                    Cond),
                   0);

    VCC = CurDAG->getCopyToReg(N->getOperand(0), SL, CondReg, Cond);
    CurDAG->SelectNodeTo(N, BrOp, MVT::Other,
                       N->getOperand(2), // Basic Block
                       VCC.getValue(0));
  }

}
// Merge an ADDI into the offset of a load/store instruction where possible.
// (load (add base, off), 0) -> (load base, off)
// (store val, (add base, off)) -> (store val, base, off)
void PPUDAGToDAGISel::doPeepholeLoadStoreADDI() {
  SelectionDAG::allnodes_iterator Position(CurDAG->getRoot().getNode());
  ++Position;

  while (Position != CurDAG->allnodes_begin()) {
    SDNode *N = &*--Position;
    // Skip dead nodes and any non-machine opcodes.
    if (N->use_empty() || !N->isMachineOpcode())
      continue;

    int OffsetOpIdx;
    int BaseOpIdx;

    // Only attempt this optimisation for I-type loads and S-type stores.
    switch (N->getMachineOpcode()) {
    default:
      continue;
    case PPU::LB:
    case PPU::LH:
    case PPU::LW:
    case PPU::LBU:
    case PPU::LHU:
    case PPU::LWU:
    case PPU::LD:
    case PPU::FLW:
    case PPU::FLD:
      BaseOpIdx = 0;
      OffsetOpIdx = 1;
      break;
    case PPU::SB:
    case PPU::SH:
    case PPU::SW:
    case PPU::SD:
    case PPU::FSW:
    case PPU::FSD:
      BaseOpIdx = 1;
      OffsetOpIdx = 2;
      break;
    }

    // Currently, the load/store offset must be 0 to be considered for this
    // peephole optimisation.
    if (!isa<ConstantSDNode>(N->getOperand(OffsetOpIdx)) ||
        N->getConstantOperandVal(OffsetOpIdx) != 0)
      continue;

    SDValue Base = N->getOperand(BaseOpIdx);

    // If the base is an ADDI, we can merge it in to the load/store.
    if (!Base.isMachineOpcode() || Base.getMachineOpcode() != PPU::ADDI)
      continue;

    SDValue ImmOperand = Base.getOperand(1);

    if (auto Const = dyn_cast<ConstantSDNode>(ImmOperand)) {
      ImmOperand = CurDAG->getTargetConstant(
          Const->getSExtValue(), SDLoc(ImmOperand), ImmOperand.getValueType());
    } else if (auto GA = dyn_cast<GlobalAddressSDNode>(ImmOperand)) {
      ImmOperand = CurDAG->getTargetGlobalAddress(
          GA->getGlobal(), SDLoc(ImmOperand), ImmOperand.getValueType(),
          GA->getOffset(), GA->getTargetFlags());
    } else {
      continue;
    }

    LLVM_DEBUG(dbgs() << "Folding add-immediate into mem-op:\nBase:    ");
    LLVM_DEBUG(Base->dump(CurDAG));
    LLVM_DEBUG(dbgs() << "\nN: ");
    LLVM_DEBUG(N->dump(CurDAG));
    LLVM_DEBUG(dbgs() << "\n");

    // Modify the offset operand of the load/store.
    if (BaseOpIdx == 0) // Load
      CurDAG->UpdateNodeOperands(N, Base.getOperand(0), ImmOperand,
                                 N->getOperand(2));
    else // Store
      CurDAG->UpdateNodeOperands(N, N->getOperand(0), Base.getOperand(0),
                                 ImmOperand, N->getOperand(3));

    // The add-immediate may now be dead, in which case remove it.
    if (Base.getNode()->use_empty())
      CurDAG->RemoveDeadNode(Base.getNode());
  }
}
/* FIXME
bool PPUDAGToDAGISel::isInlineImmediate(const SDNode *N) const {
  const PPUInstrInfo *TII = Subtarget->getInstrInfo();
  bool IsVALU = TII->isVALU(N->getOpcode());
  unsigned BitSize = N->getValueType(0).getSizeInBits();

  if (BitSize == 64 && IsVALU == false) {
    return false;
  }

  if (const ConstantSDNode *C = dyn_cast<ConstantSDNode>(N))
    return TII->isInlineConstant(C->getAPIntValue());

  if (const ConstantFPSDNode *C = dyn_cast<ConstantFPSDNode>(N))
    return TII->isInlineConstant(C->getValueAPF().bitcastToAPInt());

  return false;
}
*/

/// Determine the register class for \p OpNo
/// \returns The register class of the virtual register that will be used for
/// the given operand number \OpNo or NULL if the register class cannot be
/// determined.
const TargetRegisterClass *
PPUDAGToDAGISel::getOperandRegClass(SDNode *N, unsigned OpNo) const {
  if (!N->isMachineOpcode()) {
    if (N->getOpcode() == ISD::CopyToReg) {
      unsigned Reg = cast<RegisterSDNode>(N->getOperand(1))->getReg();
      if (Register::isVirtualRegister(Reg)) {
        MachineRegisterInfo &MRI = CurDAG->getMachineFunction().getRegInfo();
        return MRI.getRegClass(Reg);
      }

      const PPURegisterInfo *TRI =
          static_cast<const PPUSubtarget *>(Subtarget)->getRegisterInfo();
      return TRI->getPhysRegClass(Reg);
    }

    return nullptr;
  }

  switch (N->getMachineOpcode()) {
  default: {
    const MCInstrDesc &Desc =
        Subtarget->getInstrInfo()->get(N->getMachineOpcode());
    unsigned OpIdx = Desc.getNumDefs() + OpNo;
    if (OpIdx >= Desc.getNumOperands())
      return nullptr;
    int RegClass = Desc.OpInfo[OpIdx].RegClass;
    if (RegClass == -1)
      return nullptr;

    return Subtarget->getRegisterInfo()->getRegClass(RegClass);
  }
  case PPU::REG_SEQUENCE: {
    unsigned RCID = cast<ConstantSDNode>(N->getOperand(0))->getZExtValue();
    const TargetRegisterClass *SuperRC =
        Subtarget->getRegisterInfo()->getRegClass(RCID);

    SDValue SubRegOp = N->getOperand(OpNo + 1);
    unsigned SubRegIdx = cast<ConstantSDNode>(SubRegOp)->getZExtValue();
    return Subtarget->getRegisterInfo()->getSubClassWithSubReg(SuperRC,
                                                               SubRegIdx);
  }
  }
}


void PPUDAGToDAGISel::SelectBuildVector(SDNode *N, unsigned RegClassID) {
  EVT VT = N->getValueType(0);
  unsigned NumVectorElts = VT.getVectorNumElements();
  EVT EltVT = VT.getVectorElementType();
  SDLoc DL(N);
  SDValue RegClass = CurDAG->getTargetConstant(RegClassID, DL, MVT::i32);

  if (NumVectorElts == 1) {
    CurDAG->SelectNodeTo(N, PPU::COPY_TO_REGCLASS, EltVT, N->getOperand(0),
                         RegClass);
    return;
  }

  assert(NumVectorElts <= 16 && "Vectors with more than 16 elements not "
                                "supported yet");
  // 16 = Max Num Vector Elements
  // 2 = 2 REG_SEQUENCE operands per element (value, subreg index)
  // 1 = Vector Register Class
  SmallVector<SDValue, 16 * 2 + 1> RegSeqArgs(NumVectorElts * 2 + 1);

  RegSeqArgs[0] = CurDAG->getTargetConstant(RegClassID, DL, MVT::i32);
  bool IsRegSeq = true;
  unsigned NOps = N->getNumOperands();
  for (unsigned i = 0; i < NOps; i++) {
    // XXX: Why is this here?
    if (isa<RegisterSDNode>(N->getOperand(i))) {
      IsRegSeq = false;
      break;
    }
    unsigned Sub = PPURegisterInfo::getSubRegFromChannel(i);
    RegSeqArgs[1 + (2 * i)] = N->getOperand(i);
    RegSeqArgs[1 + (2 * i) + 1] = CurDAG->getTargetConstant(Sub, DL, MVT::i32);
  }
  if (NOps != NumVectorElts) {
    // Fill in the missing undef elements if this was a scalar_to_vector.
    assert(N->getOpcode() == ISD::SCALAR_TO_VECTOR && NOps < NumVectorElts);
    MachineSDNode *ImpDef =
        CurDAG->getMachineNode(TargetOpcode::IMPLICIT_DEF, DL, EltVT);
    for (unsigned i = NOps; i < NumVectorElts; ++i) {
      unsigned Sub = PPURegisterInfo::getSubRegFromChannel(i);
      RegSeqArgs[1 + (2 * i)] = SDValue(ImpDef, 0);
      RegSeqArgs[1 + (2 * i) + 1] =
          CurDAG->getTargetConstant(Sub, DL, MVT::i32);
    }
  }

  if (!IsRegSeq)
    SelectCode(N);
  CurDAG->SelectNodeTo(N, PPU::REG_SEQUENCE, N->getVTList(), RegSeqArgs);
}

std::pair<SDValue, SDValue>
PPUDAGToDAGISel::foldFrameIndex(SDValue N) const {
  const MachineFunction &MF = CurDAG->getMachineFunction();
  const PPUMachineFunctionInfo *Info = MF.getInfo<PPUMachineFunctionInfo>();

  if (auto FI = dyn_cast<FrameIndexSDNode>(N)) {
    SDValue TFI =
        CurDAG->getTargetFrameIndex(FI->getIndex(), FI->getValueType(0));

    // If we can resolve this to a frame index access, this is relative to the
    // frame pointer SGPR.
    return std::make_pair(
        TFI, CurDAG->getRegister(Info->getFrameOffsetReg(), MVT::i32));
  }

  // If we don't know this private access is a local stack object, it needs to
  // be relative to the entry point's scratch wave offset register.
  return std::make_pair(
      N, CurDAG->getRegister(Info->getScratchWaveOffsetReg(), MVT::i32));
}

bool PPUDAGToDAGISel::matchLoadD16FromBuildVector(SDNode *N) const {
  // assert(Subtarget->d16PreservesUnusedBits()); TODO i think we have LH and LHU
  MVT VT = N->getValueType(0).getSimpleVT();
  if (VT != MVT::v2i16 && VT != MVT::v2f16)
    return false;

  SDValue Lo = N->getOperand(0);
  SDValue Hi = N->getOperand(1);

  LoadSDNode *LdHi = dyn_cast<LoadSDNode>(stripBitcast(Hi));

  // build_vector lo, (load ptr) -> load_d16_hi ptr, lo
  // build_vector lo, (zextload ptr from i8) -> load_d16_hi_u8 ptr, lo
  // build_vector lo, (sextload ptr from i8) -> load_d16_hi_i8 ptr, lo

  // Need to check for possible indirect dependencies on the other half of the
  // vector to avoid introducing a cycle.
  if (LdHi && Hi.hasOneUse() && !LdHi->isPredecessorOf(Lo.getNode())) {
    SDVTList VTList = CurDAG->getVTList(VT, MVT::Other);

    SDValue TiedIn = CurDAG->getNode(ISD::SCALAR_TO_VECTOR, SDLoc(N), VT, Lo);
    SDValue Ops[] = {
      LdHi->getChain(), LdHi->getBasePtr(), TiedIn
    };

    // FIXME
    unsigned LoadOp = PPUISD::LOAD_D16_HI;

    if (LdHi->getMemoryVT() == MVT::i8) {
      // FIXME
      LoadOp = LdHi->getExtensionType() == ISD::SEXTLOAD ?  PPUISD::LOAD_D16_HI_I8 : PPUISD::LOAD_D16_HI_U8;
    } else {
      assert(LdHi->getMemoryVT() == MVT::i16);
    }

    SDValue NewLoadHi =
      CurDAG->getMemIntrinsicNode(LoadOp, SDLoc(LdHi), VTList,
                                  Ops, LdHi->getMemoryVT(),
                                  LdHi->getMemOperand());

    CurDAG->ReplaceAllUsesOfValueWith(SDValue(N, 0), NewLoadHi);
    CurDAG->ReplaceAllUsesOfValueWith(SDValue(LdHi, 1), NewLoadHi.getValue(1));
    return true;
  }

  // build_vector (load ptr), hi -> load_d16_lo ptr, hi
  // build_vector (zextload ptr from i8), hi -> load_d16_lo_u8 ptr, hi
  // build_vector (sextload ptr from i8), hi -> load_d16_lo_i8 ptr, hi
  LoadSDNode *LdLo = dyn_cast<LoadSDNode>(stripBitcast(Lo));
  if (LdLo && Lo.hasOneUse()) {
    SDValue TiedIn = getHi16Elt(Hi);
    if (!TiedIn || LdLo->isPredecessorOf(TiedIn.getNode()))
      return false;

    SDVTList VTList = CurDAG->getVTList(VT, MVT::Other);
    // FIXME
    unsigned LoadOp = PPUISD::LOAD_D16_LO;
    if (LdLo->getMemoryVT() == MVT::i8) {
      // FIXME
      LoadOp = LdLo->getExtensionType() == ISD::SEXTLOAD ?  PPUISD::LOAD_D16_LO_I8 : PPUISD::LOAD_D16_LO_U8;
    } else {
      assert(LdLo->getMemoryVT() == MVT::i16);
    }

    TiedIn = CurDAG->getNode(ISD::BITCAST, SDLoc(N), VT, TiedIn);

    SDValue Ops[] = {
      LdLo->getChain(), LdLo->getBasePtr(), TiedIn
    };

    SDValue NewLoadLo =
      CurDAG->getMemIntrinsicNode(LoadOp, SDLoc(LdLo), VTList,
                                  Ops, LdLo->getMemoryVT(),
                                  LdLo->getMemOperand());

    CurDAG->ReplaceAllUsesOfValueWith(SDValue(N, 0), NewLoadLo);
    CurDAG->ReplaceAllUsesOfValueWith(SDValue(LdLo, 1), NewLoadLo.getValue(1));
    return true;
  }

  return false;
}

void PPUDAGToDAGISel::PreprocessISelDAG() {
    /* RISCV have LH, LHU
  if (!Subtarget->d16PreservesUnusedBits())
    return;
    */

  SelectionDAG::allnodes_iterator Position = CurDAG->allnodes_end();

  bool MadeChange = false;
  while (Position != CurDAG->allnodes_begin()) {
    SDNode *N = &*--Position;
    if (N->use_empty())
      continue;

    switch (N->getOpcode()) {
    case ISD::BUILD_VECTOR:
      MadeChange |= matchLoadD16FromBuildVector(N);
      break;
    default:
      break;
    }
  }

  if (MadeChange) {
    CurDAG->RemoveDeadNodes();
    LLVM_DEBUG(dbgs() << "After PreProcess:\n";
               CurDAG->dump(););
  }
}
/*
void PPUDAGToDAGISel::PostprocessISelDAG() {
  doPeepholeLoadStoreADDI();
}
*/


void PPUDAGToDAGISel::PostprocessISelDAG() {
  const PPUTargetLowering &Lowering =
      *static_cast<const PPUTargetLowering *>(getTargetLowering());
  bool IsModified = false;
  do {
    IsModified = false;

    // Go over all selected nodes and try to fold them a bit more
    SelectionDAG::allnodes_iterator Position = CurDAG->allnodes_begin();
    while (Position != CurDAG->allnodes_end()) {
      SDNode *Node = &*Position++;
      MachineSDNode *MachineNode = dyn_cast<MachineSDNode>(Node);
      if (!MachineNode)
        continue;

      SDNode *ResNode = Lowering.PostISelFolding(MachineNode, *CurDAG);
      if (ResNode != Node) {
        if (ResNode)
          ReplaceUses(Node, ResNode);
        IsModified = true;
      }
    }
    CurDAG->RemoveDeadNodes();
  } while (IsModified);
  doPeepholeLoadStoreADDI();
}



