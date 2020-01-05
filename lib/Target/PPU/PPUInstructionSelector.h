//===- PPUInstructionSelector --------------------------------*- C++ -*-==//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
/// \file
/// This file declares the targeting of the InstructionSelector class for
/// PPU.
//===----------------------------------------------------------------------===//



#ifndef LLVM_LIB_TARGET_PPU_PPUINSTRUCTIONSELECTOR_H
#define LLVM_LIB_TARGET_PPU_PPUINSTRUCTIONSELECTOR_H

#include "PPU.h"
#include "PPUArgumentUsageInfo.h"
#include "llvm/ADT/ArrayRef.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/CodeGen/Register.h"
#include "llvm/CodeGen/GlobalISel/InstructionSelector.h"
#include "llvm/IR/InstrTypes.h"

namespace {
#define GET_GLOBALISEL_PREDICATE_BITSET
// #define PPUSubtarget GCNSubtarget
#include "PPUGenGlobalISel.inc"
#undef GET_GLOBALISEL_PREDICATE_BITSET
// #undef PPUSubtarget
}

namespace llvm {

class PPUInstrInfo;
class PPURegisterBankInfo;
class PPUSubtarget;
class MachineInstr;
class MachineOperand;
class MachineRegisterInfo;
class PPUInstrInfo;
class PPUMachineFunctionInfo;
class PPURegisterInfo;

class PPUInstructionSelector : public InstructionSelector {
public:
  PPUInstructionSelector(const PPUTargetMachine &TM,
                            const PPUSubtarget &STI,
                            const PPURegisterBankInfo &RBI
                            );

  bool select(MachineInstr &I) override;
  static const char *getName();

private:
  struct GEPInfo {
    const MachineInstr &GEP;
    SmallVector<unsigned, 2> SgprParts;
    SmallVector<unsigned, 2> VgprParts;
    int64_t Imm;
    GEPInfo(const MachineInstr &GEP) : GEP(GEP), Imm(0) { }
  };

  bool isInstrUniform(const MachineInstr &MI) const;
  bool isVCC(Register Reg, const MachineRegisterInfo &MRI) const;

  /// tblgen-erated 'select' implementation.
  bool selectImpl(MachineInstr &I, CodeGenCoverage &CoverageInfo) const;

  MachineOperand getSubOperand64(MachineOperand &MO,
                                 const TargetRegisterClass &SubRC,
                                 unsigned SubIdx) const;
  bool selectCOPY(MachineInstr &I) const;
  bool selectPHI(MachineInstr &I) const;
  bool selectG_TRUNC(MachineInstr &I) const;
  bool selectG_SZA_EXT(MachineInstr &I) const;
  bool selectG_CONSTANT(MachineInstr &I) const;
  bool selectG_AND_OR_XOR(MachineInstr &I) const;
  bool selectG_ADD_SUB(MachineInstr &I) const;
  bool selectG_EXTRACT(MachineInstr &I) const;
  bool selectG_MERGE_VALUES(MachineInstr &I) const;
  bool selectG_UNMERGE_VALUES(MachineInstr &I) const;
  bool selectG_GEP(MachineInstr &I) const;
  bool selectG_IMPLICIT_DEF(MachineInstr &I) const;
  bool selectG_INSERT(MachineInstr &I) const;
  bool selectG_INTRINSIC(MachineInstr &I) const;
  bool selectG_INTRINSIC_W_SIDE_EFFECTS(MachineInstr &I) const;
  int getS_CMPOpcode(CmpInst::Predicate P, unsigned Size) const;
  bool selectG_ICMP(MachineInstr &I) const;
  bool hasVgprParts(ArrayRef<GEPInfo> AddrInfo) const;
  void getAddrModeInfo(const MachineInstr &Load, const MachineRegisterInfo &MRI,
                       SmallVectorImpl<GEPInfo> &AddrInfo) const;
  bool selectSMRD(MachineInstr &I, ArrayRef<GEPInfo> AddrInfo) const;

  void initM0(MachineInstr &I) const;
  bool selectG_LOAD_ATOMICRMW(MachineInstr &I) const;
  bool selectG_STORE(MachineInstr &I) const;
  bool selectG_SELECT(MachineInstr &I) const;
  bool selectG_BRCOND(MachineInstr &I) const;
  bool selectG_FRAME_INDEX(MachineInstr &I) const;

  std::pair<Register, unsigned>
  selectVOP3ModsImpl(Register Src, const MachineRegisterInfo &MRI) const;

  InstructionSelector::ComplexRendererFns
  selectVCSRC(MachineOperand &Root) const;

  InstructionSelector::ComplexRendererFns
  selectVSRC0(MachineOperand &Root) const;

  InstructionSelector::ComplexRendererFns
  selectVOP3Mods0(MachineOperand &Root) const;
  InstructionSelector::ComplexRendererFns
  selectVOP3OMods(MachineOperand &Root) const;
  InstructionSelector::ComplexRendererFns
  selectVOP3Mods(MachineOperand &Root) const;

  InstructionSelector::ComplexRendererFns
  selectSmrdImm(MachineOperand &Root) const;
  InstructionSelector::ComplexRendererFns
  selectSmrdImm32(MachineOperand &Root) const;
  InstructionSelector::ComplexRendererFns
  selectSmrdSgpr(MachineOperand &Root) const;

  template <bool Signed>
  InstructionSelector::ComplexRendererFns
  selectFlatOffsetImpl(MachineOperand &Root) const;
  InstructionSelector::ComplexRendererFns
  selectFlatOffset(MachineOperand &Root) const;

  InstructionSelector::ComplexRendererFns
  selectFlatOffsetSigned(MachineOperand &Root) const;

  InstructionSelector::ComplexRendererFns
  selectMUBUFScratchOffen(MachineOperand &Root) const;
  InstructionSelector::ComplexRendererFns
  selectMUBUFScratchOffset(MachineOperand &Root) const;

  bool isDSOffsetLegal(const MachineRegisterInfo &MRI,
                       const MachineOperand &Base,
                       int64_t Offset, unsigned OffsetBits) const;

  InstructionSelector::ComplexRendererFns
  selectDS1Addr1Offset(MachineOperand &Root) const;

  const PPUInstrInfo &TII;
  const PPURegisterInfo &TRI;
  const PPURegisterBankInfo &RBI;
  const PPUTargetMachine &TM;
  const PPUSubtarget &STI;

  // FIXME: This is necessary because DAGISel uses "Subtarget->" and GlobalISel
  // uses "STI." in the code generated by TableGen. We need to unify the name of
  // Subtarget variable.
  const PPUSubtarget *Subtarget = &STI;

  bool EnableLateStructurizeCFG;
  bool EnableReconvergeCFG;
#define GET_GLOBALISEL_PREDICATES_DECL
// #define PPUSubtarget PPUSubtarget
#include "PPUGenGlobalISel.inc"
#undef GET_GLOBALISEL_PREDICATES_DECL
// #undef PPUSubtarget

#define GET_GLOBALISEL_TEMPORARIES_DECL
#include "PPUGenGlobalISel.inc"
#undef GET_GLOBALISEL_TEMPORARIES_DECL
};

} // End llvm namespace.
#endif
