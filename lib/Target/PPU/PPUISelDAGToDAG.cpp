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
#include "PPUISelLowering.h" // For PPUISD
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
    return DAG.getMachineNode(PPU::S_MOV_B32, SL, N->getValueType(0),
                              DAG.getTargetConstant(K, SL, MVT::i32));
  }

  return nullptr;
}

static SDNode *packNegConstantV2I16(const SDNode *N, SelectionDAG &DAG) {
  return packConstantV2I16(N, DAG, true);
}

// This is for Base
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




// This is for Base
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

class PPUBaseDAGToDAGISel : public SelectionDAGISel {
protected:
  const PPUSubtarget *Subtarget;
  bool EnableReconvergeCFG;

public:
  explicit PPUBaseDAGToDAGISel(PPUTargetMachine &TargetMachine,
                              CodeGenOpt::Level OptLevel = CodeGenOpt::Default)
      : SelectionDAGISel(TargetMachine, OptLevel) {}

  void PostprocessISelDAG() override;
  void Select(SDNode *Node) override;

  bool SelectInlineAsmMemoryOperand(const SDValue &Op, unsigned ConstraintID,
                                    std::vector<SDValue> &OutOps) override;
  bool SelectAddrFI(SDValue Addr, SDValue &Base);

private:
  void doPeepholeLoadStoreADDI();

};

// PPU-specific code to select PPU machine instructions for
// SelectionDAG operations.
class PPUDAGToDAGISel final: public PPUBaseDAGToDAGISel {

public:
  explicit PPUDAGToDAGISel(PPUTargetMachine *TM = nullptr,
                              CodeGenOpt::Level OptLevel = CodeGenOpt::Default)
      : PPUBaseDAGToDAGISel(*TM, OptLevel) {
    // EnableReconvergeCFG = TM->getSubtargetImpl()->enableReconvergeCFG();
    EnableReconvergeCFG = TM->EnableReconvergeCFG;
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
  void PreprocessISelDAG() override;
  void Select(SDNode *N) override;
  void PostprocessISelDAG() override;

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

protected:
  void SelectBuildVector(SDNode *N, unsigned RegClassID); // AMD

private:
  std::pair<SDValue, SDValue> foldFrameIndex(SDValue N) const;
  bool isNoNanSrc(SDValue N) const;

  bool isInlineImmediate(const SDNode *N, bool Negated = false) const;
  bool isNegInlineImmediate(const SDNode *N) const {
    return isInlineImmediate(N, true);
  }

  bool isVGPRImm(const SDNode *N) const;
  bool isUniformLoad(const SDNode *N) const;
  bool isUniformStore(const SDNode *N) const;
  bool isUniformBr(const SDNode *N) const;

  MachineSDNode *buildSMovImm64(SDLoc &DL, uint64_t Val, EVT VT) const;

  SDNode *glueCopyToM0LDSInit(SDNode *N) const;
  SDNode *glueCopyToM0(SDNode *N, SDValue Val) const;

  // MachineSDNode *buildSMovImm64(SDLoc &DL, uint64_t Val, EVT VT) const;
  const TargetRegisterClass *getOperandRegClass(SDNode *N, unsigned OpNo) const;

  virtual bool SelectADDRVTX_READ(SDValue Addr, SDValue &Base, SDValue &Offset);
  virtual bool SelectADDRIndirect(SDValue Addr, SDValue &Base, SDValue &Offset);
  bool isDSOffsetLegal(SDValue Base, unsigned Offset,
                       unsigned OffsetBits) const;
  bool SelectDS1Addr1Offset(SDValue Ptr, SDValue &Base, SDValue &Offset) const;
  bool SelectDS64Bit4ByteAligned(SDValue Ptr, SDValue &Base, SDValue &Offset0,
                                 SDValue &Offset1) const;

  bool SelectMUBUF(SDValue Addr, SDValue &SRsrc, SDValue &VAddr,
                   SDValue &SOffset, SDValue &Offset, SDValue &Offen,
                   SDValue &Idxen, SDValue &Addr64, SDValue &GLC, SDValue &SLC,
                   SDValue &TFE, SDValue &DLC) const;
  bool SelectMUBUFAddr64(SDValue Addr, SDValue &SRsrc, SDValue &VAddr,
                         SDValue &SOffset, SDValue &Offset, SDValue &GLC,
                         SDValue &SLC, SDValue &TFE, SDValue &DLC) const;
  bool SelectMUBUFAddr64(SDValue Addr, SDValue &SRsrc,
                         SDValue &VAddr, SDValue &SOffset, SDValue &Offset,
                         SDValue &SLC) const;
  bool SelectMUBUFScratchOffen(SDNode *Parent,
                               SDValue Addr, SDValue &RSrc, SDValue &VAddr,
                               SDValue &SOffset, SDValue &ImmOffset) const;
  bool SelectMUBUFScratchOffset(SDNode *Parent,
                                SDValue Addr, SDValue &SRsrc, SDValue &Soffset,
                                SDValue &Offset) const;
  bool SelectMUBUFOffset(SDValue Addr, SDValue &SRsrc, SDValue &SOffset,
                         SDValue &Offset, SDValue &GLC, SDValue &SLC,
                         SDValue &TFE, SDValue &DLC) const;
  bool SelectMUBUFOffset(SDValue Addr, SDValue &SRsrc, SDValue &Soffset,
                         SDValue &Offset, SDValue &SLC) const;
  bool SelectMUBUFOffset(SDValue Addr, SDValue &SRsrc, SDValue &Soffset,
                         SDValue &Offset) const;

  bool SelectFlatAtomic(SDNode *N, SDValue Addr, SDValue &VAddr,
                        SDValue &Offset, SDValue &SLC) const;
  bool SelectFlatAtomicSigned(SDNode *N, SDValue Addr, SDValue &VAddr,
                              SDValue &Offset, SDValue &SLC) const;

  template <bool IsSigned>
  bool SelectFlatOffset(SDNode *N, SDValue Addr, SDValue &VAddr,
                        SDValue &Offset, SDValue &SLC) const;

  bool SelectSMRDOffset(SDValue ByteOffsetNode, SDValue &Offset,
                        bool &Imm) const;
  SDValue Expand32BitAddress(SDValue Addr) const;
  bool SelectSMRD(SDValue Addr, SDValue &SBase, SDValue &Offset,
                  bool &Imm) const;
  bool SelectSMRDImm(SDValue Addr, SDValue &SBase, SDValue &Offset) const;
  bool SelectSMRDImm32(SDValue Addr, SDValue &SBase, SDValue &Offset) const;
  bool SelectSMRDSgpr(SDValue Addr, SDValue &SBase, SDValue &Offset) const;
  bool SelectSMRDBufferImm(SDValue Addr, SDValue &Offset) const;
  bool SelectSMRDBufferImm32(SDValue Addr, SDValue &Offset) const;

  bool SelectMOVRELOffset(SDValue Index, SDValue &Base, SDValue &Offset) const;

  bool SelectVOP3Mods_NNaN(SDValue In, SDValue &Src, SDValue &SrcMods) const;
  bool SelectVOP3Mods_f32(SDValue In, SDValue &Src, SDValue &SrcMods) const;
  bool SelectVOP3ModsImpl(SDValue In, SDValue &Src, unsigned &SrcMods) const;
  bool SelectVOP3Mods(SDValue In, SDValue &Src, SDValue &SrcMods) const;
  bool SelectVOP3NoMods(SDValue In, SDValue &Src) const;
  bool SelectVOP3Mods0(SDValue In, SDValue &Src, SDValue &SrcMods,
                       SDValue &Clamp, SDValue &Omod) const;
  bool SelectVOP3NoMods0(SDValue In, SDValue &Src, SDValue &SrcMods,
                         SDValue &Clamp, SDValue &Omod) const;

  bool SelectVOP3Mods0Clamp0OMod(SDValue In, SDValue &Src, SDValue &SrcMods,
                                 SDValue &Clamp,
                                 SDValue &Omod) const;
  bool SelectVOP3OMods(SDValue In, SDValue &Src,
                       SDValue &Clamp, SDValue &Omod) const;

  bool SelectVOP3PMods(SDValue In, SDValue &Src, SDValue &SrcMods) const;
  bool SelectVOP3PMods0(SDValue In, SDValue &Src, SDValue &SrcMods,
                        SDValue &Clamp) const;
  bool SelectVOP3OpSel(SDValue In, SDValue &Src, SDValue &SrcMods) const;
  bool SelectVOP3OpSel0(SDValue In, SDValue &Src, SDValue &SrcMods,
                        SDValue &Clamp) const;

  bool SelectVOP3OpSelMods(SDValue In, SDValue &Src, SDValue &SrcMods) const;
  bool SelectVOP3OpSelMods0(SDValue In, SDValue &Src, SDValue &SrcMods,
                            SDValue &Clamp) const;

  // bool SelectVOP3PMadMixModsImpl(SDValue In, SDValue &Src, unsigned &Mods) const;
  // bool SelectVOP3PMadMixMods(SDValue In, SDValue &Src, SDValue &SrcMods) const;


  SDValue getHi16Elt(SDValue In) const;

  void SelectADD_SUB_I64(SDNode *N);
  void SelectAddcSubb(SDNode *N);
  void SelectUADDO_USUBO(SDNode *N);
  void SelectDIV_SCALE(SDNode *N);
  void SelectDIV_FMAS(SDNode *N);
  void SelectMAD_64_32(SDNode *N);
  void SelectFMA_W_CHAIN(SDNode *N);
  void SelectFMUL_W_CHAIN(SDNode *N);

  SDNode *getS_BFE(unsigned Opcode, const SDLoc &DL, SDValue Val,
                   uint32_t Offset, uint32_t Width);
  void SelectS_BFEFromShifts(SDNode *N);
  void SelectS_BFE(SDNode *N);

  bool isCBranchSCC(const SDNode *N) const;
  void SelectBRCOND(SDNode *N);
  void SelectFMAD_FMA(SDNode *N);
  void SelectATOMIC_CMP_SWAP(SDNode *N);
  void SelectDSAppendConsume(SDNode *N, unsigned IntrID);
  void SelectDS_GWS(SDNode *N, unsigned IntrID);
  void SelectINTRINSIC_W_CHAIN(SDNode *N);
  void SelectINTRINSIC_WO_CHAIN(SDNode *N);
  void SelectINTRINSIC_VOID(SDNode *N);

protected:
// Include the pieces autogenerated from the target description.
#include "PPUGenDAGISel.inc"


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



void PPUBaseDAGToDAGISel::Select(SDNode *Node) {
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
  }

  // Select the default instruction.
  // SelectCode(Node); move to PPUDAGToDAGISel::Select
}

bool PPUBaseDAGToDAGISel::SelectAddrFI(SDValue Addr, SDValue &Base) {
  if (auto FIN = dyn_cast<FrameIndexSDNode>(Addr)) {
    Base = CurDAG->getTargetFrameIndex(FIN->getIndex(), Subtarget->getXLenVT());
    return true;
  }
  return false;
}


bool PPUBaseDAGToDAGISel::SelectInlineAsmMemoryOperand(
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

// Merge an ADDI into the offset of a load/store instruction where possible.
// (load (add base, off), 0) -> (load base, off)
// (store val, (add base, off)) -> (store val, base, off)
void PPUBaseDAGToDAGISel::doPeepholeLoadStoreADDI() {
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
    // case PPU::FLD:
      BaseOpIdx = 0;
      OffsetOpIdx = 1;
      break;
    case PPU::SB:
    case PPU::SH:
    case PPU::SW:
    case PPU::SD:
    case PPU::FSW:
    // case PPU::FSD:
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


void PPUBaseDAGToDAGISel::PostprocessISelDAG() {
  doPeepholeLoadStoreADDI();
}



//// below copied from AMD
//

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
  if (!Subtarget->d16PreservesUnusedBits())
    return;

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

bool PPUDAGToDAGISel::isNoNanSrc(SDValue N) const {
  if (TM.Options.NoNaNsFPMath)
    return true;

  // TODO: Move into isKnownNeverNaN
  if (N->getFlags().isDefined())
    return N->getFlags().hasNoNaNs();

  return CurDAG->isKnownNeverNaN(N);
}

bool PPUDAGToDAGISel::isInlineImmediate(const SDNode *N,
                                           bool Negated) const {
  if (N->isUndef())
    return true;

  const PPUInstrInfo *TII = Subtarget->getInstrInfo();
  if (Negated) {
    if (const ConstantSDNode *C = dyn_cast<ConstantSDNode>(N))
      return TII->isInlineConstant(-C->getAPIntValue());

    if (const ConstantFPSDNode *C = dyn_cast<ConstantFPSDNode>(N))
      return TII->isInlineConstant(-C->getValueAPF().bitcastToAPInt());

  } else {
    if (const ConstantSDNode *C = dyn_cast<ConstantSDNode>(N))
      return TII->isInlineConstant(C->getAPIntValue());

    if (const ConstantFPSDNode *C = dyn_cast<ConstantFPSDNode>(N))
      return TII->isInlineConstant(C->getValueAPF().bitcastToAPInt());
  }

  return false;
}

/// Determine the register class for \p OpNo
/// \returns The register class of the virtual register that will be used for
/// the given operand number \OpNo or NULL if the register class cannot be
/// determined.
const TargetRegisterClass * PPUDAGToDAGISel::getOperandRegClass(SDNode *N, unsigned OpNo) const {
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

SDNode *PPUDAGToDAGISel::glueCopyToM0(SDNode *N, SDValue Val) const {
  const PPUTargetLowering& Lowering =
    *static_cast<const PPUTargetLowering*>(getTargetLowering());

  assert(N->getOperand(0).getValueType() == MVT::Other && "Expected chain");

  SDValue M0 = Lowering.copyToM0(*CurDAG, N->getOperand(0), SDLoc(N),
                                 Val);

  SDValue Glue = M0.getValue(1);

  SmallVector <SDValue, 8> Ops;
  Ops.push_back(M0); // Replace the chain.
  for (unsigned i = 1, e = N->getNumOperands(); i != e; ++i)
    Ops.push_back(N->getOperand(i));

  Ops.push_back(Glue);
  return CurDAG->MorphNodeTo(N, N->getOpcode(), N->getVTList(), Ops);
}

SDNode *PPUDAGToDAGISel::glueCopyToM0LDSInit(SDNode *N) const {
  unsigned AS = cast<MemSDNode>(N)->getAddressSpace();
  if (AS == AMDGPUAS::LOCAL_ADDRESS) {
    if (Subtarget->ldsRequiresM0Init())
      return glueCopyToM0(N, CurDAG->getTargetConstant(-1, SDLoc(N), MVT::i32));
  } else if (AS == AMDGPUAS::REGION_ADDRESS) {
    MachineFunction &MF = CurDAG->getMachineFunction();
    unsigned Value = MF.getInfo<PPUMachineFunctionInfo>()->getGDSSize();
    return
        glueCopyToM0(N, CurDAG->getTargetConstant(Value, SDLoc(N), MVT::i32));
  }
  return N;
}

MachineSDNode *PPUDAGToDAGISel::buildSMovImm64(SDLoc &DL, uint64_t Imm,
                                                  EVT VT) const {
  SDNode *Lo = CurDAG->getMachineNode(PPU::S_MOV_B32, DL, MVT::i32,
      CurDAG->getTargetConstant(Imm & 0xFFFFFFFF, DL, MVT::i32));
  SDNode *Hi =
      CurDAG->getMachineNode(PPU::S_MOV_B32, DL, MVT::i32,
                             CurDAG->getTargetConstant(Imm >> 32, DL, MVT::i32));
  const SDValue Ops[] = {
      CurDAG->getTargetConstant(PPU::SReg_64RegClassID, DL, MVT::i32),
      SDValue(Lo, 0), CurDAG->getTargetConstant(PPU::sub0, DL, MVT::i32),
      SDValue(Hi, 0), CurDAG->getTargetConstant(PPU::sub1, DL, MVT::i32)};

  return CurDAG->getMachineNode(TargetOpcode::REG_SEQUENCE, DL, VT, Ops);
}

static unsigned selectSGPRVectorRegClassID(unsigned NumVectorElts) {
  switch (NumVectorElts) {
  case 1:
    return PPU::SReg_32RegClassID;
  case 2:
    return PPU::SReg_64RegClassID;
  case 4:
    return PPU::SReg_128RegClassID;
    /* FIXME
  case 8:
    return PPU::SReg_256RegClassID;
  case 16:
    return PPU::SReg_512RegClassID;
    */
  }

  llvm_unreachable("invalid vector size");
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

  assert(NumVectorElts <= 32 && "Vectors with more than 16 elements not "
                                "supported yet");
  // 32 = Max Num Vector Elements
  // 2 = 2 REG_SEQUENCE operands per element (value, subreg index)
  // 1 = Vector Register Class
  SmallVector<SDValue, 32 * 2 + 1> RegSeqArgs(NumVectorElts * 2 + 1);

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

void PPUDAGToDAGISel::Select(SDNode *N) {
  if (!PPU::isCompute(CurDAG)) {
      PPUBaseDAGToDAGISel::Select(N);
      return SelectCode(N);
  }
  unsigned int Opc = N->getOpcode();
  if (N->isMachineOpcode()) {
    N->setNodeId(-1);
    return;   // Already selected.
  }

  if (isa<AtomicSDNode>(N) ||
      (Opc == PPUISD::ATOMIC_INC || Opc == PPUISD::ATOMIC_DEC ||
       Opc == ISD::ATOMIC_LOAD_FADD ||
       Opc == PPUISD::ATOMIC_LOAD_FMIN ||
       Opc == PPUISD::ATOMIC_LOAD_FMAX))
    N = glueCopyToM0LDSInit(N);

  switch (Opc) {
  default:
    break;
  // We are selecting i64 ADD here instead of custom lower it during
  // DAG legalization, so we can fold some i64 ADDs used for address
  // calculation into the LOAD and STORE instructions.
  case ISD::ADDC:
  case ISD::ADDE:
  case ISD::SUBC:
  case ISD::SUBE: {
    if (N->getValueType(0) != MVT::i64)
      break;

    SelectADD_SUB_I64(N);
    return;
  }
  case ISD::ADDCARRY:
  case ISD::SUBCARRY:
    if (N->getValueType(0) != MVT::i32)
      break;

    SelectAddcSubb(N);
    return;
  case ISD::UADDO:
  case ISD::USUBO: {
    SelectUADDO_USUBO(N);
    return;
  }
  case PPUISD::FMUL_W_CHAIN: {
    SelectFMUL_W_CHAIN(N);
    return;
  }
  case PPUISD::FMA_W_CHAIN: {
    SelectFMA_W_CHAIN(N);
    return;
  }

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
  case ISD::BUILD_PAIR: {
    SDValue RC, SubReg0, SubReg1;
    SDLoc DL(N);
    if (N->getValueType(0) == MVT::i128) {
      RC = CurDAG->getTargetConstant(PPU::SReg_128RegClassID, DL, MVT::i32);
      SubReg0 = CurDAG->getTargetConstant(PPU::sub0_sub1, DL, MVT::i32);
      SubReg1 = CurDAG->getTargetConstant(PPU::sub2_sub3, DL, MVT::i32);
    } else if (N->getValueType(0) == MVT::i64) {
      RC = CurDAG->getTargetConstant(PPU::SReg_64RegClassID, DL, MVT::i32);
      SubReg0 = CurDAG->getTargetConstant(PPU::sub0, DL, MVT::i32);
      SubReg1 = CurDAG->getTargetConstant(PPU::sub1, DL, MVT::i32);
    } else {
      llvm_unreachable("Unhandled value type for BUILD_PAIR");
    }
    const SDValue Ops[] = { RC, N->getOperand(0), SubReg0,
                            N->getOperand(1), SubReg1 };
    ReplaceNode(N, CurDAG->getMachineNode(TargetOpcode::REG_SEQUENCE, DL,
                                          N->getValueType(0), Ops));
    return;
  }

  case ISD::Constant:
  case ISD::ConstantFP: {
    if (N->getValueType(0).getSizeInBits() != 64 || isInlineImmediate(N))
      break;

    uint64_t Imm;
    if (ConstantFPSDNode *FP = dyn_cast<ConstantFPSDNode>(N))
      Imm = FP->getValueAPF().bitcastToAPInt().getZExtValue();
    else {
      ConstantSDNode *C = cast<ConstantSDNode>(N);
      Imm = C->getZExtValue();
    }

    SDLoc DL(N);
    ReplaceNode(N, buildSMovImm64(DL, Imm, N->getValueType(0)));
    return;
  }
  case ISD::LOAD:
  case ISD::STORE:
  case ISD::ATOMIC_LOAD:
  case ISD::ATOMIC_STORE: {
    N = glueCopyToM0LDSInit(N);
    break;
  }

  case PPUISD::BFE_I32:
  case PPUISD::BFE_U32: {
    // There is a scalar version available, but unlike the vector version which
    // has a separate operand for the offset and width, the scalar version packs
    // the width and offset into a single operand. Try to move to the scalar
    // version if the offsets are constant, so that we can try to keep extended
    // loads of kernel arguments in SGPRs.

    // TODO: Technically we could try to pattern match scalar bitshifts of
    // dynamic values, but it's probably not useful.
    ConstantSDNode *Offset = dyn_cast<ConstantSDNode>(N->getOperand(1));
    if (!Offset)
      break;

    ConstantSDNode *Width = dyn_cast<ConstantSDNode>(N->getOperand(2));
    if (!Width)
      break;

    bool Signed = Opc == PPUISD::BFE_I32;

    uint32_t OffsetVal = Offset->getZExtValue();
    uint32_t WidthVal = Width->getZExtValue();

    ReplaceNode(N, getS_BFE(Signed ? PPU::S_BFE_I32 : PPU::S_BFE_U32,
                            SDLoc(N), N->getOperand(0), OffsetVal, WidthVal));
    return;
  }
  case PPUISD::DIV_SCALE: {
    SelectDIV_SCALE(N);
    return;
  }
  case PPUISD::DIV_FMAS: {
    SelectDIV_FMAS(N);
    return;
  }
  case PPUISD::MAD_I64_I32:
  case PPUISD::MAD_U64_U32: {
    SelectMAD_64_32(N);
    return;
  }
  case ISD::CopyToReg: {
    const PPUTargetLowering& Lowering =
      *static_cast<const PPUTargetLowering*>(getTargetLowering());
    N = Lowering.legalizeTargetIndependentNode(N, *CurDAG);
    break;
  }
  case ISD::AND:
  case ISD::SRL:
  case ISD::SRA:
  case ISD::SIGN_EXTEND_INREG:
    if (N->getValueType(0) != MVT::i32)
      break;

    SelectS_BFE(N);
    return;
  case ISD::BRCOND:
    SelectBRCOND(N);
    return;
  case ISD::FMAD:
  case ISD::FMA:
    SelectFMAD_FMA(N);
    return;
  case PPUISD::ATOMIC_CMP_SWAP:
    SelectATOMIC_CMP_SWAP(N);
    return;
  case PPUISD::CVT_PKRTZ_F16_F32:
  case PPUISD::CVT_PKNORM_I16_F32:
  case PPUISD::CVT_PKNORM_U16_F32:
  case PPUISD::CVT_PK_U16_U32:
  case PPUISD::CVT_PK_I16_I32: {
    // Hack around using a legal type if f16 is illegal.
    if (N->getValueType(0) == MVT::i32) {
      MVT NewVT = Opc == PPUISD::CVT_PKRTZ_F16_F32 ? MVT::v2f16 : MVT::v2i16;
      N = CurDAG->MorphNodeTo(N, N->getOpcode(), CurDAG->getVTList(NewVT),
                              { N->getOperand(0), N->getOperand(1) });
      SelectCode(N);
      return;
    }

    break;
  }
  case ISD::INTRINSIC_W_CHAIN: {
    SelectINTRINSIC_W_CHAIN(N);
    return;
  }
  case ISD::INTRINSIC_WO_CHAIN: {
    SelectINTRINSIC_WO_CHAIN(N);
    return;
  }
  case ISD::INTRINSIC_VOID: {
    SelectINTRINSIC_VOID(N);
    return;
  }
  }

  SelectCode(N);
}

bool PPUDAGToDAGISel::isUniformBr(const SDNode *N) const {
  const BasicBlock *BB = FuncInfo->MBB->getBasicBlock();
  const Instruction *Term = BB->getTerminator();
  return Term->getMetadata("ppu.uniform") ||
         Term->getMetadata("structurizecfg.uniform");
}

//===----------------------------------------------------------------------===//
// Complex Patterns
//===----------------------------------------------------------------------===//

bool PPUDAGToDAGISel::SelectADDRVTX_READ(SDValue Addr, SDValue &Base,
                                            SDValue &Offset) {
  return false;
}

bool PPUDAGToDAGISel::SelectADDRIndirect(SDValue Addr, SDValue &Base,
                                            SDValue &Offset) {
  ConstantSDNode *C;
  SDLoc DL(Addr);
/*
  if ((C = dyn_cast<ConstantSDNode>(Addr))) {
    Base = CurDAG->getRegister(R600::INDIRECT_BASE_ADDR, MVT::i32);
    Offset = CurDAG->getTargetConstant(C->getZExtValue(), DL, MVT::i32);
  } else if ((Addr.getOpcode() == PPUISD::DWORDADDR) &&
             (C = dyn_cast<ConstantSDNode>(Addr.getOperand(0)))) {
    Base = CurDAG->getRegister(R600::INDIRECT_BASE_ADDR, MVT::i32);
    Offset = CurDAG->getTargetConstant(C->getZExtValue(), DL, MVT::i32);
  } else
*/
  if ((Addr.getOpcode() == ISD::ADD || Addr.getOpcode() == ISD::OR) &&
            (C = dyn_cast<ConstantSDNode>(Addr.getOperand(1)))) {
    Base = Addr.getOperand(0);
    Offset = CurDAG->getTargetConstant(C->getZExtValue(), DL, MVT::i32);
  } else {
    Base = Addr;
    Offset = CurDAG->getTargetConstant(0, DL, MVT::i32);
  }

  return true;
}

// FIXME: Should only handle addcarry/subcarry
void PPUDAGToDAGISel::SelectADD_SUB_I64(SDNode *N) {
  SDLoc DL(N);
  SDValue LHS = N->getOperand(0);
  SDValue RHS = N->getOperand(1);

  unsigned Opcode = N->getOpcode();
  bool ConsumeCarry = (Opcode == ISD::ADDE || Opcode == ISD::SUBE);
  bool ProduceCarry =
      ConsumeCarry || Opcode == ISD::ADDC || Opcode == ISD::SUBC;
  bool IsAdd = Opcode == ISD::ADD || Opcode == ISD::ADDC || Opcode == ISD::ADDE;

  SDValue Sub0 = CurDAG->getTargetConstant(PPU::sub0, DL, MVT::i32);
  SDValue Sub1 = CurDAG->getTargetConstant(PPU::sub1, DL, MVT::i32);

  SDNode *Lo0 = CurDAG->getMachineNode(TargetOpcode::EXTRACT_SUBREG,
                                       DL, MVT::i32, LHS, Sub0);
  SDNode *Hi0 = CurDAG->getMachineNode(TargetOpcode::EXTRACT_SUBREG,
                                       DL, MVT::i32, LHS, Sub1);

  SDNode *Lo1 = CurDAG->getMachineNode(TargetOpcode::EXTRACT_SUBREG,
                                       DL, MVT::i32, RHS, Sub0);
  SDNode *Hi1 = CurDAG->getMachineNode(TargetOpcode::EXTRACT_SUBREG,
                                       DL, MVT::i32, RHS, Sub1);

  SDVTList VTList = CurDAG->getVTList(MVT::i32, MVT::Glue);

  unsigned Opc = IsAdd ? PPU::S_ADD_U32 : PPU::S_SUB_U32;
  unsigned CarryOpc = IsAdd ? PPU::S_ADDC_U32 : PPU::S_SUBB_U32;

  SDNode *AddLo;
  if (!ConsumeCarry) {
    SDValue Args[] = { SDValue(Lo0, 0), SDValue(Lo1, 0) };
    AddLo = CurDAG->getMachineNode(Opc, DL, VTList, Args);
  } else {
    SDValue Args[] = { SDValue(Lo0, 0), SDValue(Lo1, 0), N->getOperand(2) };
    AddLo = CurDAG->getMachineNode(CarryOpc, DL, VTList, Args);
  }
  SDValue AddHiArgs[] = {
    SDValue(Hi0, 0),
    SDValue(Hi1, 0),
    SDValue(AddLo, 1)
  };
  SDNode *AddHi = CurDAG->getMachineNode(CarryOpc, DL, VTList, AddHiArgs);

  SDValue RegSequenceArgs[] = {
    CurDAG->getTargetConstant(PPU::SReg_64RegClassID, DL, MVT::i32),
    SDValue(AddLo,0),
    Sub0,
    SDValue(AddHi,0),
    Sub1,
  };
  SDNode *RegSequence = CurDAG->getMachineNode(PPU::REG_SEQUENCE, DL,
                                               MVT::i64, RegSequenceArgs);

  if (ProduceCarry) {
    // Replace the carry-use
    ReplaceUses(SDValue(N, 1), SDValue(AddHi, 1));
  }

  // Replace the remaining uses.
  ReplaceNode(N, RegSequence);
}

void PPUDAGToDAGISel::SelectAddcSubb(SDNode *N) {
  SDLoc DL(N);
  SDValue LHS = N->getOperand(0);
  SDValue RHS = N->getOperand(1);
  SDValue CI = N->getOperand(2);

  unsigned Opc = N->getOpcode() == ISD::ADDCARRY ? PPU::V_ADDC_U32_e64
                                                 : PPU::V_SUBB_U32_e64;
  CurDAG->SelectNodeTo(
      N, Opc, N->getVTList(),
      {LHS, RHS, CI, CurDAG->getTargetConstant(0, {}, MVT::i1) /*clamp bit*/});
}

void PPUDAGToDAGISel::SelectUADDO_USUBO(SDNode *N) {
  // The name of the opcodes are misleading. v_add_i32/v_sub_i32 have unsigned
  // carry out despite the _i32 name. These were renamed in VI to _U32.
  // FIXME: We should probably rename the opcodes here.
  unsigned Opc = N->getOpcode() == ISD::UADDO ?
    PPU::V_ADD_I32_e64 : PPU::V_SUB_I32_e64;

  CurDAG->SelectNodeTo(
      N, Opc, N->getVTList(),
      {N->getOperand(0), N->getOperand(1),
       CurDAG->getTargetConstant(0, {}, MVT::i1) /*clamp bit*/});
}

void PPUDAGToDAGISel::SelectFMA_W_CHAIN(SDNode *N) {
  SDLoc SL(N);
  //  src0_modifiers, src0,  src1_modifiers, src1, src2_modifiers, src2, clamp, omod
  SDValue Ops[10];

  SelectVOP3Mods0(N->getOperand(1), Ops[1], Ops[0], Ops[6], Ops[7]);
  SelectVOP3Mods(N->getOperand(2), Ops[3], Ops[2]);
  SelectVOP3Mods(N->getOperand(3), Ops[5], Ops[4]);
  Ops[8] = N->getOperand(0);
  Ops[9] = N->getOperand(4);

  CurDAG->SelectNodeTo(N, PPU::V_FMA_F32, N->getVTList(), Ops);
}

void PPUDAGToDAGISel::SelectFMUL_W_CHAIN(SDNode *N) {
  SDLoc SL(N);
  //    src0_modifiers, src0,  src1_modifiers, src1, clamp, omod
  SDValue Ops[8];

  SelectVOP3Mods0(N->getOperand(1), Ops[1], Ops[0], Ops[4], Ops[5]);
  SelectVOP3Mods(N->getOperand(2), Ops[3], Ops[2]);
  Ops[6] = N->getOperand(0);
  Ops[7] = N->getOperand(3);

  CurDAG->SelectNodeTo(N, PPU::V_MUL_F32_e64, N->getVTList(), Ops);
}

// We need to handle this here because tablegen doesn't support matching
// instructions with multiple outputs.
void PPUDAGToDAGISel::SelectDIV_SCALE(SDNode *N) {
  SDLoc SL(N);
  EVT VT = N->getValueType(0);

  assert(VT == MVT::f32 || VT == MVT::f64);

  unsigned Opc
    = (VT == MVT::f64) ? PPU::V_DIV_SCALE_F64 : PPU::V_DIV_SCALE_F32;

  SDValue Ops[] = { N->getOperand(0), N->getOperand(1), N->getOperand(2) };
  CurDAG->SelectNodeTo(N, Opc, N->getVTList(), Ops);
}

void PPUDAGToDAGISel::SelectDIV_FMAS(SDNode *N) {
  const PPUSubtarget *ST = static_cast<const PPUSubtarget *>(Subtarget);
  const PPURegisterInfo *TRI = ST->getRegisterInfo();

  SDLoc SL(N);
  EVT VT = N->getValueType(0);

  assert(VT == MVT::f32 || VT == MVT::f64);

  unsigned Opc
    = (VT == MVT::f64) ? PPU::V_DIV_FMAS_F64 : PPU::V_DIV_FMAS_F32;

  SDValue CarryIn = N->getOperand(3);
  // V_DIV_FMAS implicitly reads VCC.
  SDValue VCC = CurDAG->getCopyToReg(CurDAG->getEntryNode(), SL,
                                     TRI->getVCC(), CarryIn, SDValue());

  SDValue Ops[10];

  SelectVOP3Mods0(N->getOperand(0), Ops[1], Ops[0], Ops[6], Ops[7]);
  SelectVOP3Mods(N->getOperand(1), Ops[3], Ops[2]);
  SelectVOP3Mods(N->getOperand(2), Ops[5], Ops[4]);

  Ops[8] = VCC;
  Ops[9] = VCC.getValue(1);

  CurDAG->SelectNodeTo(N, Opc, N->getVTList(), Ops);
}

// We need to handle this here because tablegen doesn't support matching
// instructions with multiple outputs.
void PPUDAGToDAGISel::SelectMAD_64_32(SDNode *N) {
  SDLoc SL(N);
  bool Signed = N->getOpcode() == PPUISD::MAD_I64_I32;
  unsigned Opc = Signed ? PPU::V_MAD_I64_I32 : PPU::V_MAD_U64_U32;

  SDValue Clamp = CurDAG->getTargetConstant(0, SL, MVT::i1);
  SDValue Ops[] = { N->getOperand(0), N->getOperand(1), N->getOperand(2),
                    Clamp };
  CurDAG->SelectNodeTo(N, Opc, N->getVTList(), Ops);
}

bool PPUDAGToDAGISel::isDSOffsetLegal(SDValue Base, unsigned Offset,
                                         unsigned OffsetBits) const {
  if ((OffsetBits == 16 && !isUInt<16>(Offset)) ||
      (OffsetBits == 8 && !isUInt<8>(Offset)))
    return false;

  if (Subtarget->hasUsableDSOffset() ||
      Subtarget->unsafeDSOffsetFoldingEnabled())
    return true;

  // On Southern Islands instruction with a negative base value and an offset
  // don't seem to work.
  return CurDAG->SignBitIsZero(Base);
}

bool PPUDAGToDAGISel::SelectDS1Addr1Offset(SDValue Addr, SDValue &Base,
                                              SDValue &Offset) const {
  SDLoc DL(Addr);
  if (CurDAG->isBaseWithConstantOffset(Addr)) {
    SDValue N0 = Addr.getOperand(0);
    SDValue N1 = Addr.getOperand(1);
    ConstantSDNode *C1 = cast<ConstantSDNode>(N1);
    if (isDSOffsetLegal(N0, C1->getSExtValue(), 16)) {
      // (add n0, c0)
      Base = N0;
      Offset = CurDAG->getTargetConstant(C1->getZExtValue(), DL, MVT::i16);
      return true;
    }
  } else if (Addr.getOpcode() == ISD::SUB) {
    // sub C, x -> add (sub 0, x), C
    if (const ConstantSDNode *C = dyn_cast<ConstantSDNode>(Addr.getOperand(0))) {
      int64_t ByteOffset = C->getSExtValue();
      if (isUInt<16>(ByteOffset)) {
        SDValue Zero = CurDAG->getTargetConstant(0, DL, MVT::i32);

        // XXX - This is kind of hacky. Create a dummy sub node so we can check
        // the known bits in isDSOffsetLegal. We need to emit the selected node
        // here, so this is thrown away.
        SDValue Sub = CurDAG->getNode(ISD::SUB, DL, MVT::i32,
                                      Zero, Addr.getOperand(1));

        if (isDSOffsetLegal(Sub, ByteOffset, 16)) {
          SmallVector<SDValue, 3> Opnds;
          Opnds.push_back(Zero);
          Opnds.push_back(Addr.getOperand(1));

          // FIXME: Select to VOP3 version for with-carry.
          unsigned SubOp = PPU::V_SUB_I32_e32;
          if (Subtarget->hasAddNoCarry()) {
            SubOp = PPU::V_SUB_U32_e64;
            Opnds.push_back(
                CurDAG->getTargetConstant(0, {}, MVT::i1)); // clamp bit
          }

          MachineSDNode *MachineSub =
              CurDAG->getMachineNode(SubOp, DL, MVT::i32, Opnds);

          Base = SDValue(MachineSub, 0);
          Offset = CurDAG->getTargetConstant(ByteOffset, DL, MVT::i16);
          return true;
        }
      }
    }
  } else if (const ConstantSDNode *CAddr = dyn_cast<ConstantSDNode>(Addr)) {
    // If we have a constant address, prefer to put the constant into the
    // offset. This can save moves to load the constant address since multiple
    // operations can share the zero base address register, and enables merging
    // into read2 / write2 instructions.

    SDLoc DL(Addr);

    if (isUInt<16>(CAddr->getZExtValue())) {
      SDValue Zero = CurDAG->getTargetConstant(0, DL, MVT::i32);
      MachineSDNode *MovZero = CurDAG->getMachineNode(PPU::V_MOV_B32_e32,
                                 DL, MVT::i32, Zero);
      Base = SDValue(MovZero, 0);
      Offset = CurDAG->getTargetConstant(CAddr->getZExtValue(), DL, MVT::i16);
      return true;
    }
  }

  // default case
  Base = Addr;
  Offset = CurDAG->getTargetConstant(0, SDLoc(Addr), MVT::i16);
  return true;
}

// TODO: If offset is too big, put low 16-bit into offset.
bool PPUDAGToDAGISel::SelectDS64Bit4ByteAligned(SDValue Addr, SDValue &Base,
                                                   SDValue &Offset0,
                                                   SDValue &Offset1) const {
  SDLoc DL(Addr);

  if (CurDAG->isBaseWithConstantOffset(Addr)) {
    SDValue N0 = Addr.getOperand(0);
    SDValue N1 = Addr.getOperand(1);
    ConstantSDNode *C1 = cast<ConstantSDNode>(N1);
    unsigned DWordOffset0 = C1->getZExtValue() / 4;
    unsigned DWordOffset1 = DWordOffset0 + 1;
    // (add n0, c0)
    if (isDSOffsetLegal(N0, DWordOffset1, 8)) {
      Base = N0;
      Offset0 = CurDAG->getTargetConstant(DWordOffset0, DL, MVT::i8);
      Offset1 = CurDAG->getTargetConstant(DWordOffset1, DL, MVT::i8);
      return true;
    }
  } else if (Addr.getOpcode() == ISD::SUB) {
    // sub C, x -> add (sub 0, x), C
    if (const ConstantSDNode *C = dyn_cast<ConstantSDNode>(Addr.getOperand(0))) {
      unsigned DWordOffset0 = C->getZExtValue() / 4;
      unsigned DWordOffset1 = DWordOffset0 + 1;

      if (isUInt<8>(DWordOffset0)) {
        SDLoc DL(Addr);
        SDValue Zero = CurDAG->getTargetConstant(0, DL, MVT::i32);

        // XXX - This is kind of hacky. Create a dummy sub node so we can check
        // the known bits in isDSOffsetLegal. We need to emit the selected node
        // here, so this is thrown away.
        SDValue Sub = CurDAG->getNode(ISD::SUB, DL, MVT::i32,
                                      Zero, Addr.getOperand(1));

        if (isDSOffsetLegal(Sub, DWordOffset1, 8)) {
          SmallVector<SDValue, 3> Opnds;
          Opnds.push_back(Zero);
          Opnds.push_back(Addr.getOperand(1));
          unsigned SubOp = PPU::V_SUB_I32_e32;
          if (Subtarget->hasAddNoCarry()) {
            SubOp = PPU::V_SUB_U32_e64;
            Opnds.push_back(
                CurDAG->getTargetConstant(0, {}, MVT::i1)); // clamp bit
          }

          MachineSDNode *MachineSub
            = CurDAG->getMachineNode(SubOp, DL, MVT::i32, Opnds);

          Base = SDValue(MachineSub, 0);
          Offset0 = CurDAG->getTargetConstant(DWordOffset0, DL, MVT::i8);
          Offset1 = CurDAG->getTargetConstant(DWordOffset1, DL, MVT::i8);
          return true;
        }
      }
    }
  } else if (const ConstantSDNode *CAddr = dyn_cast<ConstantSDNode>(Addr)) {
    unsigned DWordOffset0 = CAddr->getZExtValue() / 4;
    unsigned DWordOffset1 = DWordOffset0 + 1;
    assert(4 * DWordOffset0 == CAddr->getZExtValue());

    if (isUInt<8>(DWordOffset0) && isUInt<8>(DWordOffset1)) {
      SDValue Zero = CurDAG->getTargetConstant(0, DL, MVT::i32);
      MachineSDNode *MovZero
        = CurDAG->getMachineNode(PPU::V_MOV_B32_e32,
                                 DL, MVT::i32, Zero);
      Base = SDValue(MovZero, 0);
      Offset0 = CurDAG->getTargetConstant(DWordOffset0, DL, MVT::i8);
      Offset1 = CurDAG->getTargetConstant(DWordOffset1, DL, MVT::i8);
      return true;
    }
  }

  // default case

  Base = Addr;
  Offset0 = CurDAG->getTargetConstant(0, DL, MVT::i8);
  Offset1 = CurDAG->getTargetConstant(1, DL, MVT::i8);
  return true;
}

bool PPUDAGToDAGISel::SelectMUBUF(SDValue Addr, SDValue &Ptr,
                                     SDValue &VAddr, SDValue &SOffset,
                                     SDValue &Offset, SDValue &Offen,
                                     SDValue &Idxen, SDValue &Addr64,
                                     SDValue &GLC, SDValue &SLC,
                                     SDValue &TFE, SDValue &DLC) const {
  // Subtarget prefers to use flat instruction
  if (Subtarget->useFlatForGlobal())
    return false;

  SDLoc DL(Addr);

  if (!GLC.getNode())
    GLC = CurDAG->getTargetConstant(0, DL, MVT::i1);
  if (!SLC.getNode())
    SLC = CurDAG->getTargetConstant(0, DL, MVT::i1);
  TFE = CurDAG->getTargetConstant(0, DL, MVT::i1);
  DLC = CurDAG->getTargetConstant(0, DL, MVT::i1);

  Idxen = CurDAG->getTargetConstant(0, DL, MVT::i1);
  Offen = CurDAG->getTargetConstant(0, DL, MVT::i1);
  Addr64 = CurDAG->getTargetConstant(0, DL, MVT::i1);
  SOffset = CurDAG->getTargetConstant(0, DL, MVT::i32);

  ConstantSDNode *C1 = nullptr;
  SDValue N0 = Addr;
  if (CurDAG->isBaseWithConstantOffset(Addr)) {
    C1 = cast<ConstantSDNode>(Addr.getOperand(1));
    if (isUInt<32>(C1->getZExtValue()))
      N0 = Addr.getOperand(0);
    else
      C1 = nullptr;
  }

  if (N0.getOpcode() == ISD::ADD) {
    // (add N2, N3) -> addr64, or
    // (add (add N2, N3), C1) -> addr64
    SDValue N2 = N0.getOperand(0);
    SDValue N3 = N0.getOperand(1);
    Addr64 = CurDAG->getTargetConstant(1, DL, MVT::i1);

    if (N2->isDivergent()) {
      if (N3->isDivergent()) {
        // Both N2 and N3 are divergent. Use N0 (the result of the add) as the
        // addr64, and construct the resource from a 0 address.
        Ptr = SDValue(buildSMovImm64(DL, 0, MVT::v2i32), 0);
        VAddr = N0;
      } else {
        // N2 is divergent, N3 is not.
        Ptr = N3;
        VAddr = N2;
      }
    } else {
      // N2 is not divergent.
      Ptr = N2;
      VAddr = N3;
    }
    Offset = CurDAG->getTargetConstant(0, DL, MVT::i16);
  } else if (N0->isDivergent()) {
    // N0 is divergent. Use it as the addr64, and construct the resource from a
    // 0 address.
    Ptr = SDValue(buildSMovImm64(DL, 0, MVT::v2i32), 0);
    VAddr = N0;
    Addr64 = CurDAG->getTargetConstant(1, DL, MVT::i1);
  } else {
    // N0 -> offset, or
    // (N0 + C1) -> offset
    VAddr = CurDAG->getTargetConstant(0, DL, MVT::i32);
    Ptr = N0;
  }

  if (!C1) {
    // No offset.
    Offset = CurDAG->getTargetConstant(0, DL, MVT::i16);
    return true;
  }

  if (PPUInstrInfo::isLegalMUBUFImmOffset(C1->getZExtValue())) {
    // Legal offset for instruction.
    Offset = CurDAG->getTargetConstant(C1->getZExtValue(), DL, MVT::i16);
    return true;
  }

  // Illegal offset, store it in soffset.
  Offset = CurDAG->getTargetConstant(0, DL, MVT::i16);
  SOffset =
      SDValue(CurDAG->getMachineNode(
                  PPU::S_MOV_B32, DL, MVT::i32,
                  CurDAG->getTargetConstant(C1->getZExtValue(), DL, MVT::i32)),
              0);
  return true;
}

bool PPUDAGToDAGISel::SelectMUBUFAddr64(SDValue Addr, SDValue &SRsrc,
                                           SDValue &VAddr, SDValue &SOffset,
                                           SDValue &Offset, SDValue &GLC,
                                           SDValue &SLC, SDValue &TFE,
                                           SDValue &DLC) const {
  SDValue Ptr, Offen, Idxen, Addr64;

  // addr64 bit was removed for volcanic islands.
  if (!Subtarget->hasAddr64())
    return false;

  if (!SelectMUBUF(Addr, Ptr, VAddr, SOffset, Offset, Offen, Idxen, Addr64,
              GLC, SLC, TFE, DLC))
    return false;

  ConstantSDNode *C = cast<ConstantSDNode>(Addr64);
  if (C->getSExtValue()) {
    SDLoc DL(Addr);

    const PPUTargetLowering& Lowering =
      *static_cast<const PPUTargetLowering*>(getTargetLowering());

    SRsrc = SDValue(Lowering.wrapAddr64Rsrc(*CurDAG, DL, Ptr), 0);
    return true;
  }

  return false;
}

bool PPUDAGToDAGISel::SelectMUBUFAddr64(SDValue Addr, SDValue &SRsrc,
                                           SDValue &VAddr, SDValue &SOffset,
                                           SDValue &Offset,
                                           SDValue &SLC) const {
  SLC = CurDAG->getTargetConstant(0, SDLoc(Addr), MVT::i1);
  SDValue GLC, TFE, DLC;

  return SelectMUBUFAddr64(Addr, SRsrc, VAddr, SOffset, Offset, GLC, SLC, TFE, DLC);
}

static bool isStackPtrRelative(const MachinePointerInfo &PtrInfo) {
  auto PSV = PtrInfo.V.dyn_cast<const PseudoSourceValue *>();
  return PSV && PSV->isStack();
}









std::pair<SDValue, SDValue> PPUDAGToDAGISel::foldFrameIndex(SDValue N) const {
  const MachineFunction &MF = CurDAG->getMachineFunction();
  const PPUMachineFunctionInfo *Info = MF.getInfo<PPUMachineFunctionInfo>();

  if (auto FI = dyn_cast<FrameIndexSDNode>(N)) {
    SDValue TFI = CurDAG->getTargetFrameIndex(FI->getIndex(), FI->getValueType(0));

    // If we can resolve this to a frame index access, this is relative to the
    // frame pointer SGPR.
    // TODO why difference return std::make_pair(TFI, CurDAG->getRegister(Info->getFrameOffsetReg(), MVT::i32));
    return std::make_pair(TFI, CurDAG->getRegister(Info->getStackPtrOffsetReg(), MVT::i32));
  }

  // If we don't know this private access is a local stack object, it needs to
  // be relative to the entry point's scratch wave offset register.
  return std::make_pair(
      N, CurDAG->getRegister(Info->getScratchWaveOffsetReg(), MVT::i32));
}

bool PPUDAGToDAGISel::SelectMUBUFScratchOffen(SDNode *Parent,
                                                 SDValue Addr, SDValue &Rsrc,
                                                 SDValue &VAddr, SDValue &SOffset,
                                                 SDValue &ImmOffset) const {

  SDLoc DL(Addr);
  MachineFunction &MF = CurDAG->getMachineFunction();
  const PPUMachineFunctionInfo *Info = MF.getInfo<PPUMachineFunctionInfo>();

  // FIXME i change to v2i32
  // Rsrc = CurDAG->getRegister(Info->getScratchRSrcReg(), MVT::v4i32);
  Rsrc = CurDAG->getRegister(Info->getScratchRSrcReg(), MVT::v2i32);

  if (ConstantSDNode *CAddr = dyn_cast<ConstantSDNode>(Addr)) {
    unsigned Imm = CAddr->getZExtValue();

    SDValue HighBits = CurDAG->getTargetConstant(Imm & ~4095, DL, MVT::i32);
    MachineSDNode *MovHighBits = CurDAG->getMachineNode(PPU::V_MOV_B32_e32,
                                                        DL, MVT::i32, HighBits);
    VAddr = SDValue(MovHighBits, 0);

    // In a call sequence, stores to the argument stack area are relative to the
    // stack pointer.
    const MachinePointerInfo &PtrInfo = cast<MemSDNode>(Parent)->getPointerInfo();
    unsigned SOffsetReg = isStackPtrRelative(PtrInfo) ?
      Info->getStackPtrOffsetReg() : Info->getScratchWaveOffsetReg();

    SOffset = CurDAG->getRegister(SOffsetReg, MVT::i32);
    ImmOffset = CurDAG->getTargetConstant(Imm & 4095, DL, MVT::i16);
    return true;
  }

  if (CurDAG->isBaseWithConstantOffset(Addr)) {
    // (add n0, c1)

    SDValue N0 = Addr.getOperand(0);
    SDValue N1 = Addr.getOperand(1);

    // Offsets in vaddr must be positive if range checking is enabled.
    //
    // The total computation of vaddr + soffset + offset must not overflow.  If
    // vaddr is negative, even if offset is 0 the sgpr offset add will end up
    // overflowing.
    //
    // Prior to gfx9, MUBUF instructions with the vaddr offset enabled would
    // always perform a range check. If a negative vaddr base index was used,
    // this would fail the range check. The overall address computation would
    // compute a valid address, but this doesn't happen due to the range
    // check. For out-of-bounds MUBUF loads, a 0 is returned.
    //
    // Therefore it should be safe to fold any VGPR offset on gfx9 into the
    // MUBUF vaddr, but not on older subtargets which can only do this if the
    // sign bit is known 0.
    ConstantSDNode *C1 = cast<ConstantSDNode>(N1);
    if (PPUInstrInfo::isLegalMUBUFImmOffset(C1->getZExtValue()) &&
        (!Subtarget->privateMemoryResourceIsRangeChecked() ||
         CurDAG->SignBitIsZero(N0))) {
      std::tie(VAddr, SOffset) = foldFrameIndex(N0);
      ImmOffset = CurDAG->getTargetConstant(C1->getZExtValue(), DL, MVT::i16);
      return true;
    }
  }

  // (node)
  std::tie(VAddr, SOffset) = foldFrameIndex(Addr);
  ImmOffset = CurDAG->getTargetConstant(0, DL, MVT::i16);
  return true;
}

bool PPUDAGToDAGISel::SelectMUBUFScratchOffset(SDNode *Parent,
                                                  SDValue Addr,
                                                  SDValue &SRsrc,
                                                  SDValue &SOffset,
                                                  SDValue &Offset) const {
  ConstantSDNode *CAddr = dyn_cast<ConstantSDNode>(Addr);
  if (!CAddr || !PPUInstrInfo::isLegalMUBUFImmOffset(CAddr->getZExtValue()))
    return false;

  SDLoc DL(Addr);
  MachineFunction &MF = CurDAG->getMachineFunction();
  const PPUMachineFunctionInfo *Info = MF.getInfo<PPUMachineFunctionInfo>();

  // FIXME SRsrc = CurDAG->getRegister(Info->getScratchRSrcReg(), MVT::v4i32);
  SRsrc = CurDAG->getRegister(Info->getScratchRSrcReg(), MVT::v2i32);

  const MachinePointerInfo &PtrInfo = cast<MemSDNode>(Parent)->getPointerInfo();
  unsigned SOffsetReg = isStackPtrRelative(PtrInfo) ?
    Info->getStackPtrOffsetReg() : Info->getScratchWaveOffsetReg();

  // FIXME: Get from MachinePointerInfo? We should only be using the frame
  // offset if we know this is in a call sequence.
  SOffset = CurDAG->getRegister(SOffsetReg, MVT::i32);

  Offset = CurDAG->getTargetConstant(CAddr->getZExtValue(), DL, MVT::i16);
  return true;
}

bool PPUDAGToDAGISel::SelectMUBUFOffset(SDValue Addr, SDValue &SRsrc,
                                           SDValue &SOffset, SDValue &Offset,
                                           SDValue &GLC, SDValue &SLC,
                                           SDValue &TFE, SDValue &DLC) const {
  SDValue Ptr, VAddr, Offen, Idxen, Addr64;
  const PPUInstrInfo *TII =
    static_cast<const PPUInstrInfo *>(Subtarget->getInstrInfo());

  if (!SelectMUBUF(Addr, Ptr, VAddr, SOffset, Offset, Offen, Idxen, Addr64,
              GLC, SLC, TFE, DLC))
    return false;

  if (!cast<ConstantSDNode>(Offen)->getSExtValue() &&
      !cast<ConstantSDNode>(Idxen)->getSExtValue() &&
      !cast<ConstantSDNode>(Addr64)->getSExtValue()) {
    uint64_t Rsrc = TII->getDefaultRsrcDataFormat() |
                    APInt::getAllOnesValue(32).getZExtValue(); // Size
    SDLoc DL(Addr);

    const PPUTargetLowering& Lowering =
      *static_cast<const PPUTargetLowering*>(getTargetLowering());

    SRsrc = SDValue(Lowering.buildRSRC(*CurDAG, DL, Ptr, 0, Rsrc), 0);
    return true;
  }
  return false;
}

bool PPUDAGToDAGISel::SelectMUBUFOffset(SDValue Addr, SDValue &SRsrc,
                                           SDValue &Soffset, SDValue &Offset
                                           ) const {
  SDValue GLC, SLC, TFE, DLC;

  return SelectMUBUFOffset(Addr, SRsrc, Soffset, Offset, GLC, SLC, TFE, DLC);
}
bool PPUDAGToDAGISel::SelectMUBUFOffset(SDValue Addr, SDValue &SRsrc,
                                           SDValue &Soffset, SDValue &Offset,
                                           SDValue &SLC) const {
  SDValue GLC, TFE, DLC;

  return SelectMUBUFOffset(Addr, SRsrc, Soffset, Offset, GLC, SLC, TFE, DLC);
}

template <bool IsSigned>
bool PPUDAGToDAGISel::SelectFlatOffset(SDNode *N,
                                          SDValue Addr,
                                          SDValue &VAddr,
                                          SDValue &Offset,
                                          SDValue &SLC) const {
  return static_cast<const PPUTargetLowering*>(getTargetLowering())->
    SelectFlatOffset(IsSigned, *CurDAG, N, Addr, VAddr, Offset, SLC);
}

bool PPUDAGToDAGISel::SelectFlatAtomic(SDNode *N,
                                          SDValue Addr,
                                          SDValue &VAddr,
                                          SDValue &Offset,
                                          SDValue &SLC) const {
  return SelectFlatOffset<false>(N, Addr, VAddr, Offset, SLC);
}

bool PPUDAGToDAGISel::SelectFlatAtomicSigned(SDNode *N,
                                          SDValue Addr,
                                          SDValue &VAddr,
                                          SDValue &Offset,
                                          SDValue &SLC) const {
  return SelectFlatOffset<true>(N, Addr, VAddr, Offset, SLC);
}

bool PPUDAGToDAGISel::SelectSMRDOffset(SDValue ByteOffsetNode,
                                          SDValue &Offset, bool &Imm) const {

  // FIXME: Handle non-constant offsets.
  ConstantSDNode *C = dyn_cast<ConstantSDNode>(ByteOffsetNode);
  if (!C)
    return false;

  SDLoc SL(ByteOffsetNode);
  PPUSubtarget::Generation Gen = Subtarget->getGeneration();
  int64_t ByteOffset = C->getSExtValue();
  int64_t EncodedOffset = PPU::getSMRDEncodedOffset(*Subtarget, ByteOffset);

  if (PPU::isLegalSMRDImmOffset(*Subtarget, ByteOffset)) {
    Offset = CurDAG->getTargetConstant(EncodedOffset, SL, MVT::i32);
    Imm = true;
    return true;
  }

  if (!isUInt<32>(EncodedOffset) || !isUInt<32>(ByteOffset))
    return false;
/*
  if (Gen == AMDGPUSubtarget::SEA_ISLANDS && isUInt<32>(EncodedOffset)) {
    // 32-bit Immediates are supported on Sea Islands.
    Offset = CurDAG->getTargetConstant(EncodedOffset, SL, MVT::i32);
  } else {
  */
    SDValue C32Bit = CurDAG->getTargetConstant(ByteOffset, SL, MVT::i32);
    Offset = SDValue(CurDAG->getMachineNode(PPU::S_MOV_B32, SL, MVT::i32,
                                            C32Bit), 0);
  // }
  Imm = false;
  return true;
}

SDValue PPUDAGToDAGISel::Expand32BitAddress(SDValue Addr) const {
  if (Addr.getValueType() != MVT::i32)
    return Addr;

  // Zero-extend a 32-bit address.
  SDLoc SL(Addr);

  const MachineFunction &MF = CurDAG->getMachineFunction();
  const PPUMachineFunctionInfo *Info = MF.getInfo<PPUMachineFunctionInfo>();
  unsigned AddrHiVal = Info->get32BitAddressHighBits();
  SDValue AddrHi = CurDAG->getTargetConstant(AddrHiVal, SL, MVT::i32);

  const SDValue Ops[] = {
    CurDAG->getTargetConstant(PPU::SReg_64RegClassID, SL, MVT::i32),
    Addr,
    CurDAG->getTargetConstant(PPU::sub0, SL, MVT::i32),
    SDValue(CurDAG->getMachineNode(PPU::S_MOV_B32, SL, MVT::i32, AddrHi),
            0),
    CurDAG->getTargetConstant(PPU::sub1, SL, MVT::i32),
  };

  return SDValue(CurDAG->getMachineNode(PPU::REG_SEQUENCE, SL, MVT::i64,
                                        Ops), 0);
}

bool PPUDAGToDAGISel::SelectSMRD(SDValue Addr, SDValue &SBase,
                                     SDValue &Offset, bool &Imm) const {
  SDLoc SL(Addr);

  // A 32-bit (address + offset) should not cause unsigned 32-bit integer
  // wraparound, because s_load instructions perform the addition in 64 bits.
  if ((Addr.getValueType() != MVT::i32 ||
       Addr->getFlags().hasNoUnsignedWrap()) &&
      CurDAG->isBaseWithConstantOffset(Addr)) {
    SDValue N0 = Addr.getOperand(0);
    SDValue N1 = Addr.getOperand(1);

    if (SelectSMRDOffset(N1, Offset, Imm)) {
      SBase = Expand32BitAddress(N0);
      return true;
    }
  }
  SBase = Expand32BitAddress(Addr);
  Offset = CurDAG->getTargetConstant(0, SL, MVT::i32);
  Imm = true;
  return true;
}

bool PPUDAGToDAGISel::SelectSMRDImm(SDValue Addr, SDValue &SBase,
                                       SDValue &Offset) const {
  bool Imm;
  return SelectSMRD(Addr, SBase, Offset, Imm) && Imm;
}

bool PPUDAGToDAGISel::SelectSMRDImm32(SDValue Addr, SDValue &SBase,
                                         SDValue &Offset) const {
    llvm_unreachable("FIXME on SelectSMRDImm32");
    return false;
    /*
  if (Subtarget->getGeneration() != AMDGPUSubtarget::SEA_ISLANDS)
    return false;

  bool Imm;
  if (!SelectSMRD(Addr, SBase, Offset, Imm))
    return false;

  return !Imm && isa<ConstantSDNode>(Offset);
  */
}

bool PPUDAGToDAGISel::SelectSMRDSgpr(SDValue Addr, SDValue &SBase,
                                        SDValue &Offset) const {
  bool Imm;
  return SelectSMRD(Addr, SBase, Offset, Imm) && !Imm &&
         !isa<ConstantSDNode>(Offset);
}

bool PPUDAGToDAGISel::SelectSMRDBufferImm(SDValue Addr,
                                             SDValue &Offset) const {
  bool Imm;
  return SelectSMRDOffset(Addr, Offset, Imm) && Imm;
}

bool PPUDAGToDAGISel::SelectSMRDBufferImm32(SDValue Addr,
                                               SDValue &Offset) const {
    return false;
    /*
  if (Subtarget->getGeneration() != AMDGPUSubtarget::SEA_ISLANDS)
    return false;

  bool Imm;
  if (!SelectSMRDOffset(Addr, Offset, Imm))
    return false;

  return !Imm && isa<ConstantSDNode>(Offset);
  */
}

bool PPUDAGToDAGISel::SelectMOVRELOffset(SDValue Index,
                                            SDValue &Base,
                                            SDValue &Offset) const {
  SDLoc DL(Index);

  if (CurDAG->isBaseWithConstantOffset(Index)) {
    SDValue N0 = Index.getOperand(0);
    SDValue N1 = Index.getOperand(1);
    ConstantSDNode *C1 = cast<ConstantSDNode>(N1);

    // (add n0, c0)
    // Don't peel off the offset (c0) if doing so could possibly lead
    // the base (n0) to be negative.
    if (C1->getSExtValue() <= 0 || CurDAG->SignBitIsZero(N0)) {
      Base = N0;
      Offset = CurDAG->getTargetConstant(C1->getZExtValue(), DL, MVT::i32);
      return true;
    }
  }

  if (isa<ConstantSDNode>(Index))
    return false;

  Base = Index;
  Offset = CurDAG->getTargetConstant(0, DL, MVT::i32);
  return true;
}

SDNode *PPUDAGToDAGISel::getS_BFE(unsigned Opcode, const SDLoc &DL,
                                     SDValue Val, uint32_t Offset,
                                     uint32_t Width) {
  // Transformation function, pack the offset and width of a BFE into
  // the format expected by the S_BFE_I32 / S_BFE_U32. In the second
  // source, bits [5:0] contain the offset and bits [22:16] the width.
  uint32_t PackedVal = Offset | (Width << 16);
  SDValue PackedConst = CurDAG->getTargetConstant(PackedVal, DL, MVT::i32);

  return CurDAG->getMachineNode(Opcode, DL, MVT::i32, Val, PackedConst);
}

void PPUDAGToDAGISel::SelectS_BFEFromShifts(SDNode *N) {
  // "(a << b) srl c)" ---> "BFE_U32 a, (c-b), (32-c)
  // "(a << b) sra c)" ---> "BFE_I32 a, (c-b), (32-c)
  // Predicate: 0 < b <= c < 32

  const SDValue &Shl = N->getOperand(0);
  ConstantSDNode *B = dyn_cast<ConstantSDNode>(Shl->getOperand(1));
  ConstantSDNode *C = dyn_cast<ConstantSDNode>(N->getOperand(1));

  if (B && C) {
    uint32_t BVal = B->getZExtValue();
    uint32_t CVal = C->getZExtValue();

    if (0 < BVal && BVal <= CVal && CVal < 32) {
      bool Signed = N->getOpcode() == ISD::SRA;
      unsigned Opcode = Signed ? PPU::S_BFE_I32 : PPU::S_BFE_U32;

      ReplaceNode(N, getS_BFE(Opcode, SDLoc(N), Shl.getOperand(0), CVal - BVal,
                              32 - CVal));
      return;
    }
  }
  SelectCode(N);
}

void PPUDAGToDAGISel::SelectS_BFE(SDNode *N) {
  switch (N->getOpcode()) {
  case ISD::AND:
    if (N->getOperand(0).getOpcode() == ISD::SRL) {
      // "(a srl b) & mask" ---> "BFE_U32 a, b, popcount(mask)"
      // Predicate: isMask(mask)
      const SDValue &Srl = N->getOperand(0);
      ConstantSDNode *Shift = dyn_cast<ConstantSDNode>(Srl.getOperand(1));
      ConstantSDNode *Mask = dyn_cast<ConstantSDNode>(N->getOperand(1));

      if (Shift && Mask) {
        uint32_t ShiftVal = Shift->getZExtValue();
        uint32_t MaskVal = Mask->getZExtValue();

        if (isMask_32(MaskVal)) {
          uint32_t WidthVal = countPopulation(MaskVal);

          ReplaceNode(N, getS_BFE(PPU::S_BFE_U32, SDLoc(N),
                                  Srl.getOperand(0), ShiftVal, WidthVal));
          return;
        }
      }
    }
    break;
  case ISD::SRL:
    if (N->getOperand(0).getOpcode() == ISD::AND) {
      // "(a & mask) srl b)" ---> "BFE_U32 a, b, popcount(mask >> b)"
      // Predicate: isMask(mask >> b)
      const SDValue &And = N->getOperand(0);
      ConstantSDNode *Shift = dyn_cast<ConstantSDNode>(N->getOperand(1));
      ConstantSDNode *Mask = dyn_cast<ConstantSDNode>(And->getOperand(1));

      if (Shift && Mask) {
        uint32_t ShiftVal = Shift->getZExtValue();
        uint32_t MaskVal = Mask->getZExtValue() >> ShiftVal;

        if (isMask_32(MaskVal)) {
          uint32_t WidthVal = countPopulation(MaskVal);

          ReplaceNode(N, getS_BFE(PPU::S_BFE_U32, SDLoc(N),
                                  And.getOperand(0), ShiftVal, WidthVal));
          return;
        }
      }
    } else if (N->getOperand(0).getOpcode() == ISD::SHL) {
      SelectS_BFEFromShifts(N);
      return;
    }
    break;
  case ISD::SRA:
    if (N->getOperand(0).getOpcode() == ISD::SHL) {
      SelectS_BFEFromShifts(N);
      return;
    }
    break;

  case ISD::SIGN_EXTEND_INREG: {
    // sext_inreg (srl x, 16), i8 -> bfe_i32 x, 16, 8
    SDValue Src = N->getOperand(0);
    if (Src.getOpcode() != ISD::SRL)
      break;

    const ConstantSDNode *Amt = dyn_cast<ConstantSDNode>(Src.getOperand(1));
    if (!Amt)
      break;

    unsigned Width = cast<VTSDNode>(N->getOperand(1))->getVT().getSizeInBits();
    ReplaceNode(N, getS_BFE(PPU::S_BFE_I32, SDLoc(N), Src.getOperand(0),
                            Amt->getZExtValue(), Width));
    return;
  }
  }

  SelectCode(N);
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
  // FIXME we have modify from AMD original
  // 
  unsigned BrOp = UseSCCBr ? PPU::S_CBRANCH_SCC1 : PPU::S_CBRANCH_VCCNZ;
  // FIXME schi we should use vector version BEQ
  // unsigned BrOp = UseSCCBr ? PPU::BEQ : PPU::BEQ;

  unsigned CondReg = UseSCCBr ? (unsigned)PPU::SCC : TRI->getVCC();
  SDLoc SL(N);
  SDValue VCC;

  if (UseSCCBr) {
    // VCC = CurDAG->getCopyToReg(N->getOperand(0), SL, Cond, Cond, Cond.getValue(1));
    VCC = CurDAG->getCopyToReg(N->getOperand(0), SL, CondReg, Cond);
    CurDAG->SelectNodeTo(N, BrOp, MVT::Other,
                       N->getOperand(2), // Basic Block
                       VCC.getValue(0));
    // SelectCode(N);
  } else {
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
    Cond = SDValue(CurDAG->getMachineNode(PPU::S_AND_B32, SL, MVT::i1,
                        CurDAG->getRegister(PPU::TMSK, MVT::i1),
                        Cond),
                   0);

    VCC = CurDAG->getCopyToReg(N->getOperand(0), SL, CondReg, Cond);
    CurDAG->SelectNodeTo(N, BrOp, MVT::Other,
                       N->getOperand(2), // Basic Block
                       VCC.getValue(0));
  }

}

void PPUDAGToDAGISel::SelectFMAD_FMA(SDNode *N) {
  MVT VT = N->getSimpleValueType(0);
  bool IsFMA = N->getOpcode() == ISD::FMA;
  if (VT != MVT::f32 || (!Subtarget->hasMadMixInsts() &&
                         !Subtarget->hasFmaMixInsts()) ||
      ((IsFMA && Subtarget->hasMadMixInsts()) ||
       (!IsFMA && Subtarget->hasFmaMixInsts()))) {
    SelectCode(N);
    return;
  }

  SDValue Src0 = N->getOperand(0);
  SDValue Src1 = N->getOperand(1);
  SDValue Src2 = N->getOperand(2);
  unsigned Src0Mods, Src1Mods, Src2Mods;
/*
  // Avoid using v_mad_mix_f32/v_fma_mix_f32 unless there is actually an operand
  // using the conversion from f16.
  bool Sel0 = SelectVOP3PMadMixModsImpl(Src0, Src0, Src0Mods);
  bool Sel1 = SelectVOP3PMadMixModsImpl(Src1, Src1, Src1Mods);
  bool Sel2 = SelectVOP3PMadMixModsImpl(Src2, Src2, Src2Mods);

  assert((IsFMA || !Subtarget->hasFP32Denormals()) &&
         "fmad selected with denormals enabled");
  // TODO: We can select this with f32 denormals enabled if all the sources are
  // converted from f16 (in which case fmad isn't legal).

  if (Sel0 || Sel1 || Sel2) {
    // For dummy operands.
    SDValue Zero = CurDAG->getTargetConstant(0, SDLoc(), MVT::i32);
    SDValue Ops[] = {
      CurDAG->getTargetConstant(Src0Mods, SDLoc(), MVT::i32), Src0,
      CurDAG->getTargetConstant(Src1Mods, SDLoc(), MVT::i32), Src1,
      CurDAG->getTargetConstant(Src2Mods, SDLoc(), MVT::i32), Src2,
      CurDAG->getTargetConstant(0, SDLoc(), MVT::i1),
      Zero, Zero
    };

    CurDAG->SelectNodeTo(N,
                         IsFMA ? PPU::V_FMA_MIX_F32 : PPU::V_MAD_MIX_F32,
                         MVT::f32, Ops);
  } else {
  */
    SelectCode(N);
  // }
}

// This is here because there isn't a way to use the generated sub0_sub1 as the
// subreg index to EXTRACT_SUBREG in tablegen.
void PPUDAGToDAGISel::SelectATOMIC_CMP_SWAP(SDNode *N) {
  MemSDNode *Mem = cast<MemSDNode>(N);
  unsigned AS = Mem->getAddressSpace();
  if (AS == AMDGPUAS::FLAT_ADDRESS) {
    SelectCode(N);
    return;
  }

  MVT VT = N->getSimpleValueType(0);
  bool Is32 = (VT == MVT::i32);
  SDLoc SL(N);

  MachineSDNode *CmpSwap = nullptr;
  if (Subtarget->hasAddr64()) {
    SDValue SRsrc, VAddr, SOffset, Offset, SLC;

    if (SelectMUBUFAddr64(Mem->getBasePtr(), SRsrc, VAddr, SOffset, Offset, SLC)) {
      assert(Is32);
        /*
      unsigned Opcode = Is32 ? PPU::BUFFER_ATOMIC_CMPSWAP_ADDR64_RTN :
        PPU::BUFFER_ATOMIC_CMPSWAP_X2_ADDR64_RTN;
        */
      unsigned Opcode = PPU::BUFFER_ATOMIC_CMPSWAP_ADDR64_RTN;
      SDValue CmpVal = Mem->getOperand(2);

      // XXX - Do we care about glue operands?

      SDValue Ops[] = {
        CmpVal, VAddr, SRsrc, SOffset, Offset, SLC, Mem->getChain()
      };

      CmpSwap = CurDAG->getMachineNode(Opcode, SL, Mem->getVTList(), Ops);
    }
  }

  if (!CmpSwap) {
    SDValue SRsrc, SOffset, Offset, SLC;
    if (SelectMUBUFOffset(Mem->getBasePtr(), SRsrc, SOffset, Offset, SLC)) {
      assert(Is32);
      /*
      unsigned Opcode = Is32 ? PPU::BUFFER_ATOMIC_CMPSWAP_OFFSET_RTN :
        PPU::BUFFER_ATOMIC_CMPSWAP_X2_OFFSET_RTN;
        */
      unsigned Opcode = PPU::BUFFER_ATOMIC_CMPSWAP_OFFSET_RTN;

      SDValue CmpVal = Mem->getOperand(2);
      SDValue Ops[] = {
        CmpVal, SRsrc, SOffset, Offset, SLC, Mem->getChain()
      };

      CmpSwap = CurDAG->getMachineNode(Opcode, SL, Mem->getVTList(), Ops);
    }
  }

  if (!CmpSwap) {
    SelectCode(N);
    return;
  }

  MachineMemOperand *MMO = Mem->getMemOperand();
  CurDAG->setNodeMemRefs(CmpSwap, {MMO});

  unsigned SubReg = Is32 ? PPU::sub0 : PPU::sub0_sub1;

  SDValue Extract
    = CurDAG->getTargetExtractSubreg(SubReg, SL, VT, SDValue(CmpSwap, 0));

  ReplaceUses(SDValue(N, 0), Extract);
  ReplaceUses(SDValue(N, 1), SDValue(CmpSwap, 1));
  CurDAG->RemoveDeadNode(N);
}

/*
void PPUDAGToDAGISel::SelectDSAppendConsume(SDNode *N, unsigned IntrID) {
  // The address is assumed to be uniform, so if it ends up in a VGPR, it will
  // be copied to an SGPR with readfirstlane.
  unsigned Opc = IntrID == Intrinsic::amdgcn_ds_append ?
    PPU::DS_APPEND : PPU::DS_CONSUME;

  SDValue Chain = N->getOperand(0);
  SDValue Ptr = N->getOperand(2);
  MemIntrinsicSDNode *M = cast<MemIntrinsicSDNode>(N);
  MachineMemOperand *MMO = M->getMemOperand();
  bool IsGDS = M->getAddressSpace() == AMDGPUAS::REGION_ADDRESS;

  SDValue Offset;
  if (CurDAG->isBaseWithConstantOffset(Ptr)) {
    SDValue PtrBase = Ptr.getOperand(0);
    SDValue PtrOffset = Ptr.getOperand(1);

    const APInt &OffsetVal = cast<ConstantSDNode>(PtrOffset)->getAPIntValue();
    if (isDSOffsetLegal(PtrBase, OffsetVal.getZExtValue(), 16)) {
      N = glueCopyToM0(N, PtrBase);
      Offset = CurDAG->getTargetConstant(OffsetVal, SDLoc(), MVT::i32);
    }
  }

  if (!Offset) {
    N = glueCopyToM0(N, Ptr);
    Offset = CurDAG->getTargetConstant(0, SDLoc(), MVT::i32);
  }

  SDValue Ops[] = {
    Offset,
    CurDAG->getTargetConstant(IsGDS, SDLoc(), MVT::i32),
    Chain,
    N->getOperand(N->getNumOperands() - 1) // New glue
  };

  SDNode *Selected = CurDAG->SelectNodeTo(N, Opc, N->getVTList(), Ops);
  CurDAG->setNodeMemRefs(cast<MachineSDNode>(Selected), {MMO});
}

static unsigned gwsIntrinToOpcode(unsigned IntrID) {
  switch (IntrID) {
  case Intrinsic::amdgcn_ds_gws_init:
    return PPU::DS_GWS_INIT;
  case Intrinsic::amdgcn_ds_gws_barrier:
    return PPU::DS_GWS_BARRIER;
  case Intrinsic::amdgcn_ds_gws_sema_v:
    return PPU::DS_GWS_SEMA_V;
  case Intrinsic::amdgcn_ds_gws_sema_br:
    return PPU::DS_GWS_SEMA_BR;
  case Intrinsic::amdgcn_ds_gws_sema_p:
    return PPU::DS_GWS_SEMA_P;
  case Intrinsic::amdgcn_ds_gws_sema_release_all:
    return PPU::DS_GWS_SEMA_RELEASE_ALL;
  default:
    llvm_unreachable("not a gws intrinsic");
  }
}

void PPUDAGToDAGISel::SelectDS_GWS(SDNode *N, unsigned IntrID) {
  if (IntrID == Intrinsic::amdgcn_ds_gws_sema_release_all &&
      !Subtarget->hasGWSSemaReleaseAll()) {
    // Let this error.
    SelectCode(N);
    return;
  }

  // Chain, intrinsic ID, vsrc, offset
  const bool HasVSrc = N->getNumOperands() == 4;
  assert(HasVSrc || N->getNumOperands() == 3);

  SDLoc SL(N);
  SDValue BaseOffset = N->getOperand(HasVSrc ? 3 : 2);
  int ImmOffset = 0;
  MemIntrinsicSDNode *M = cast<MemIntrinsicSDNode>(N);
  MachineMemOperand *MMO = M->getMemOperand();

  // Don't worry if the offset ends up in a VGPR. Only one lane will have
  // effect, so SIFixSGPRCopies will validly insert readfirstlane.

  // The resource id offset is computed as (<isa opaque base> + M0[21:16] +
  // offset field) % 64. Some versions of the programming guide omit the m0
  // part, or claim it's from offset 0.
  if (ConstantSDNode *ConstOffset = dyn_cast<ConstantSDNode>(BaseOffset)) {
    // If we have a constant offset, try to use the 0 in m0 as the base.
    // TODO: Look into changing the default m0 initialization value. If the
    // default -1 only set the low 16-bits, we could leave it as-is and add 1 to
    // the immediate offset.
    glueCopyToM0(N, CurDAG->getTargetConstant(0, SL, MVT::i32));
    ImmOffset = ConstOffset->getZExtValue();
  } else {
    if (CurDAG->isBaseWithConstantOffset(BaseOffset)) {
      ImmOffset = BaseOffset.getConstantOperandVal(1);
      BaseOffset = BaseOffset.getOperand(0);
    }

    // Prefer to do the shift in an SGPR since it should be possible to use m0
    // as the result directly. If it's already an SGPR, it will be eliminated
    // later.
    SDNode *SGPROffset
      = CurDAG->getMachineNode(PPU::V_READFIRSTLANE_B32, SL, MVT::i32,
                               BaseOffset);
    // Shift to offset in m0
    SDNode *M0Base
      = CurDAG->getMachineNode(PPU::S_LSHL_B32, SL, MVT::i32,
                               SDValue(SGPROffset, 0),
                               CurDAG->getTargetConstant(16, SL, MVT::i32));
    glueCopyToM0(N, SDValue(M0Base, 0));
  }

  SDValue Chain = N->getOperand(0);
  SDValue OffsetField = CurDAG->getTargetConstant(ImmOffset, SL, MVT::i32);

  // TODO: Can this just be removed from the instruction?
  SDValue GDS = CurDAG->getTargetConstant(1, SL, MVT::i1);

  const unsigned Opc = gwsIntrinToOpcode(IntrID);
  SmallVector<SDValue, 5> Ops;
  if (HasVSrc)
    Ops.push_back(N->getOperand(2));
  Ops.push_back(OffsetField);
  Ops.push_back(GDS);
  Ops.push_back(Chain);

  SDNode *Selected = CurDAG->SelectNodeTo(N, Opc, N->getVTList(), Ops);
  CurDAG->setNodeMemRefs(cast<MachineSDNode>(Selected), {MMO});
}
*/

void PPUDAGToDAGISel::SelectINTRINSIC_W_CHAIN(SDNode *N) {
  unsigned IntrID = cast<ConstantSDNode>(N->getOperand(1))->getZExtValue();
  /*
  switch (IntrID) {
  case Intrinsic::amdgcn_ds_append:
  case Intrinsic::amdgcn_ds_consume: {
    if (N->getValueType(0) != MVT::i32)
      break;
    SelectDSAppendConsume(N, IntrID);
    return;
  }
  }
  */

  SelectCode(N);
}

void PPUDAGToDAGISel::SelectINTRINSIC_WO_CHAIN(SDNode *N) {
  unsigned IntrID = cast<ConstantSDNode>(N->getOperand(0))->getZExtValue();
  unsigned Opcode;
  switch (IntrID) {
  case Intrinsic::ppu_wqm:
    Opcode = PPU::WQM;
    break;
  case Intrinsic::ppu_softwqm:
    Opcode = PPU::SOFT_WQM;
    break;
  case Intrinsic::ppu_wwm:
    Opcode = PPU::WWM;
    break;
  default:
    SelectCode(N);
    return;
  }

  SDValue Src = N->getOperand(1);
  CurDAG->SelectNodeTo(N, Opcode, N->getVTList(), {Src});
}

void PPUDAGToDAGISel::SelectINTRINSIC_VOID(SDNode *N) {
  unsigned IntrID = cast<ConstantSDNode>(N->getOperand(1))->getZExtValue();
  switch (IntrID) {
      /*
  case Intrinsic::amdgcn_ds_gws_init:
  case Intrinsic::amdgcn_ds_gws_barrier:
  case Intrinsic::amdgcn_ds_gws_sema_v:
  case Intrinsic::amdgcn_ds_gws_sema_br:
  case Intrinsic::amdgcn_ds_gws_sema_p:
  case Intrinsic::amdgcn_ds_gws_sema_release_all:
    SelectDS_GWS(N, IntrID);
    return;
    */
  default:
    break;
  }

  SelectCode(N);
}


bool PPUDAGToDAGISel::SelectVOP3ModsImpl(SDValue In, SDValue &Src,
                                            unsigned &Mods) const {
  Mods = 0;
  Src = In;

  if (Src.getOpcode() == ISD::FNEG) {
    Mods |= PPUSrcMods::NEG;
    Src = Src.getOperand(0);
  }

  if (Src.getOpcode() == ISD::FABS) {
    Mods |= PPUSrcMods::ABS;
    Src = Src.getOperand(0);
  }

  return true;
}

bool PPUDAGToDAGISel::SelectVOP3Mods(SDValue In, SDValue &Src,
                                        SDValue &SrcMods) const {
  unsigned Mods;
  if (SelectVOP3ModsImpl(In, Src, Mods)) {
    SrcMods = CurDAG->getTargetConstant(Mods, SDLoc(In), MVT::i32);
    return true;
  }

  return false;
}

bool PPUDAGToDAGISel::SelectVOP3Mods_NNaN(SDValue In, SDValue &Src,
                                             SDValue &SrcMods) const {
  SelectVOP3Mods(In, Src, SrcMods);
  return isNoNanSrc(Src);
}

bool PPUDAGToDAGISel::SelectVOP3Mods_f32(SDValue In, SDValue &Src,
                                            SDValue &SrcMods) const {
  if (In.getValueType() == MVT::f32)
    return SelectVOP3Mods(In, Src, SrcMods);
  Src = In;
  SrcMods = CurDAG->getTargetConstant(0, SDLoc(In), MVT::i32);;
  return true;
}

bool PPUDAGToDAGISel::SelectVOP3NoMods(SDValue In, SDValue &Src) const {
  if (In.getOpcode() == ISD::FABS || In.getOpcode() == ISD::FNEG)
    return false;

  Src = In;
  return true;
}

bool PPUDAGToDAGISel::SelectVOP3Mods0(SDValue In, SDValue &Src,
                                         SDValue &SrcMods, SDValue &Clamp,
                                         SDValue &Omod) const {
  SDLoc DL(In);
  Clamp = CurDAG->getTargetConstant(0, DL, MVT::i1);
  Omod = CurDAG->getTargetConstant(0, DL, MVT::i1);

  return SelectVOP3Mods(In, Src, SrcMods);
}

bool PPUDAGToDAGISel::SelectVOP3Mods0Clamp0OMod(SDValue In, SDValue &Src,
                                                   SDValue &SrcMods,
                                                   SDValue &Clamp,
                                                   SDValue &Omod) const {
  Clamp = Omod = CurDAG->getTargetConstant(0, SDLoc(In), MVT::i32);
  return SelectVOP3Mods(In, Src, SrcMods);
}

bool PPUDAGToDAGISel::SelectVOP3OMods(SDValue In, SDValue &Src,
                                         SDValue &Clamp, SDValue &Omod) const {
  Src = In;

  SDLoc DL(In);
  Clamp = CurDAG->getTargetConstant(0, DL, MVT::i1);
  Omod = CurDAG->getTargetConstant(0, DL, MVT::i1);

  return true;
}

bool PPUDAGToDAGISel::SelectVOP3PMods(SDValue In, SDValue &Src,
                                         SDValue &SrcMods) const {
  unsigned Mods = 0;
  Src = In;

  if (Src.getOpcode() == ISD::FNEG) {
    Mods ^= (PPUSrcMods::NEG | PPUSrcMods::NEG_HI);
    Src = Src.getOperand(0);
  }

  if (Src.getOpcode() == ISD::BUILD_VECTOR) {
    unsigned VecMods = Mods;

    SDValue Lo = stripBitcast(Src.getOperand(0));
    SDValue Hi = stripBitcast(Src.getOperand(1));

    if (Lo.getOpcode() == ISD::FNEG) {
      Lo = stripBitcast(Lo.getOperand(0));
      Mods ^= PPUSrcMods::NEG;
    }

    if (Hi.getOpcode() == ISD::FNEG) {
      Hi = stripBitcast(Hi.getOperand(0));
      Mods ^= PPUSrcMods::NEG_HI;
    }

    if (isExtractHiElt(Lo, Lo))
      Mods |= PPUSrcMods::OP_SEL_0;

    if (isExtractHiElt(Hi, Hi))
      Mods |= PPUSrcMods::OP_SEL_1;

    Lo = stripExtractLoElt(Lo);
    Hi = stripExtractLoElt(Hi);

    if (Lo == Hi && !isInlineImmediate(Lo.getNode())) {
      // Really a scalar input. Just select from the low half of the register to
      // avoid packing.

      Src = Lo;
      SrcMods = CurDAG->getTargetConstant(Mods, SDLoc(In), MVT::i32);
      return true;
    }

    Mods = VecMods;
  }

  // Packed instructions do not have abs modifiers.
  Mods |= PPUSrcMods::OP_SEL_1;

  SrcMods = CurDAG->getTargetConstant(Mods, SDLoc(In), MVT::i32);
  return true;
}

bool PPUDAGToDAGISel::SelectVOP3PMods0(SDValue In, SDValue &Src,
                                          SDValue &SrcMods,
                                          SDValue &Clamp) const {
  SDLoc SL(In);

  // FIXME: Handle clamp and op_sel
  Clamp = CurDAG->getTargetConstant(0, SL, MVT::i32);

  return SelectVOP3PMods(In, Src, SrcMods);
}

bool PPUDAGToDAGISel::SelectVOP3OpSel(SDValue In, SDValue &Src,
                                         SDValue &SrcMods) const {
  Src = In;
  // FIXME: Handle op_sel
  SrcMods = CurDAG->getTargetConstant(0, SDLoc(In), MVT::i32);
  return true;
}

bool PPUDAGToDAGISel::SelectVOP3OpSel0(SDValue In, SDValue &Src,
                                          SDValue &SrcMods,
                                          SDValue &Clamp) const {
  SDLoc SL(In);

  // FIXME: Handle clamp
  Clamp = CurDAG->getTargetConstant(0, SL, MVT::i32);

  return SelectVOP3OpSel(In, Src, SrcMods);
}

bool PPUDAGToDAGISel::SelectVOP3OpSelMods(SDValue In, SDValue &Src,
                                             SDValue &SrcMods) const {
  // FIXME: Handle op_sel
  return SelectVOP3Mods(In, Src, SrcMods);
}

bool PPUDAGToDAGISel::SelectVOP3OpSelMods0(SDValue In, SDValue &Src,
                                              SDValue &SrcMods,
                                              SDValue &Clamp) const {
  SDLoc SL(In);

  // FIXME: Handle clamp
  Clamp = CurDAG->getTargetConstant(0, SL, MVT::i32);

  return SelectVOP3OpSelMods(In, Src, SrcMods);
}
/*
// The return value is not whether the match is possible (which it always is),
// but whether or not it a conversion is really used.
bool PPUDAGToDAGISel::SelectVOP3PMadMixModsImpl(SDValue In, SDValue &Src,
                                                   unsigned &Mods) const {
  Mods = 0;
  SelectVOP3ModsImpl(In, Src, Mods);

  if (Src.getOpcode() == ISD::FP_EXTEND) {
    Src = Src.getOperand(0);
    assert(Src.getValueType() == MVT::f16);
    Src = stripBitcast(Src);

    // Be careful about folding modifiers if we already have an abs. fneg is
    // applied last, so we don't want to apply an earlier fneg.
    if ((Mods & PPUSrcMods::ABS) == 0) {
      unsigned ModsTmp;
      SelectVOP3ModsImpl(Src, Src, ModsTmp);

      if ((ModsTmp & PPUSrcMods::NEG) != 0)
        Mods ^= PPUSrcMods::NEG;

      if ((ModsTmp & PPUSrcMods::ABS) != 0)
        Mods |= PPUSrcMods::ABS;
    }

    // op_sel/op_sel_hi decide the source type and source.
    // If the source's op_sel_hi is set, it indicates to do a conversion from fp16.
    // If the sources's op_sel is set, it picks the high half of the source
    // register.

    Mods |= PPUSrcMods::OP_SEL_1;
    if (isExtractHiElt(Src, Src)) {
      Mods |= PPUSrcMods::OP_SEL_0;

      // TODO: Should we try to look for neg/abs here?
    }

    return true;
  }

  return false;
}

bool PPUDAGToDAGISel::SelectVOP3PMadMixMods(SDValue In, SDValue &Src,
                                               SDValue &SrcMods) const {
  unsigned Mods = 0;
  SelectVOP3PMadMixModsImpl(In, Src, Mods);
  SrcMods = CurDAG->getTargetConstant(Mods, SDLoc(In), MVT::i32);
  return true;
}
*/


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

bool PPUDAGToDAGISel::isVGPRImm(const SDNode * N) const {
  assert(CurDAG->getTarget().getTargetTriple().getArch() == Triple::ppu);

  const PPURegisterInfo *SIRI =
    static_cast<const PPURegisterInfo *>(Subtarget->getRegisterInfo());
  const PPUInstrInfo * SII =
    static_cast<const PPUInstrInfo *>(Subtarget->getInstrInfo());

  unsigned Limit = 0;
  bool AllUsesAcceptSReg = true;
  for (SDNode::use_iterator U = N->use_begin(), E = SDNode::use_end();
    Limit < 10 && U != E; ++U, ++Limit) {
    const TargetRegisterClass *RC = getOperandRegClass(*U, U.getOperandNo());

    // If the register class is unknown, it could be an unknown
    // register class that needs to be an SGPR, e.g. an inline asm
    // constraint
    if (!RC || SIRI->isSGPRClass(RC))
      return false;

    if (RC != &PPU::VS_32RegClass) {
      AllUsesAcceptSReg = false;
      SDNode * User = *U;
      if (User->isMachineOpcode()) {
        unsigned Opc = User->getMachineOpcode();
        MCInstrDesc Desc = SII->get(Opc);
        if (Desc.isCommutable()) {
          unsigned OpIdx = Desc.getNumDefs() + U.getOperandNo();
          unsigned CommuteIdx1 = TargetInstrInfo::CommuteAnyOperandIndex;
          if (SII->findCommutedOpIndices(Desc, OpIdx, CommuteIdx1)) {
            unsigned CommutedOpNo = CommuteIdx1 - Desc.getNumDefs();
            const TargetRegisterClass *CommutedRC = getOperandRegClass(*U, CommutedOpNo);
            if (CommutedRC == &PPU::VS_32RegClass)
              AllUsesAcceptSReg = true;
          }
        }
      }
      // If "AllUsesAcceptSReg == false" so far we haven't suceeded
      // commuting current user. This means have at least one use
      // that strictly require VGPR. Thus, we will not attempt to commute
      // other user instructions.
      if (!AllUsesAcceptSReg)
        break;
    }
  }
  return !AllUsesAcceptSReg && (Limit < 10);
}

bool PPUDAGToDAGISel::isUniformLoad(const SDNode * N) const {
  auto Ld = cast<LoadSDNode>(N);

  return Ld->getAlignment() >= 4 &&
        (
          (
            (
              Ld->getAddressSpace() == AMDGPUAS::CONSTANT_ADDRESS       ||
              Ld->getAddressSpace() == AMDGPUAS::CONSTANT_ADDRESS_32BIT
            )
            &&
            !N->isDivergent()
          )
          ||
          (
            Subtarget->getScalarizeGlobalBehavior() &&
            Ld->getAddressSpace() == AMDGPUAS::GLOBAL_ADDRESS &&
            !Ld->isVolatile() &&
            !N->isDivergent() &&
            static_cast<const PPUTargetLowering *>(
              getTargetLowering())->isMemOpHasNoClobberedMemOperand(N)
          )
        );
}

bool PPUDAGToDAGISel::isUniformStore(const SDNode * N) const {
  auto St = cast<StoreSDNode>(N);

  // FIXME I copied from isUniformLoad, but not sure correct. for collber wheck MemoryDependenceAnalysis
  return St->getAlignment() >= 4 &&
        (
          /*(
            (
              St->getAddressSpace() == AMDGPUAS::CONSTANT_ADDRESS       ||
              St->getAddressSpace() == AMDGPUAS::CONSTANT_ADDRESS_32BIT
            )
            &&
            !N->isDivergent()
          )
          ||*/
          (
            Subtarget->getScalarizeGlobalBehavior() &&
            St->getAddressSpace() == AMDGPUAS::GLOBAL_ADDRESS &&
            !St->isVolatile() &&
            !N->isDivergent() /*&&
            static_cast<const PPUTargetLowering *>(
              getTargetLowering())->isMemOpHasNoClobberedMemOperand(N)*/
          )
        );
}

void PPUDAGToDAGISel::PostprocessISelDAG() {
  if (!PPU::isCompute(CurDAG)) {
      return PPUBaseDAGToDAGISel::PostprocessISelDAG();
  }
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
}






