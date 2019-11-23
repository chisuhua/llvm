//===- PPUMemoryLegalizer.cpp ----------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
/// \file
/// Memory legalizer - implements memory model. More information can be
/// found here:
///   http://llvm.org/docs/PPUUsage.html#memory-model
//
//===----------------------------------------------------------------------===//

#include "PPU.h"
#include "PPUMachineModuleInfo.h"
#include "PPUSubtarget.h"
#include "PPUDefines.h"
#include "PPUInstrInfo.h"
#include "MCTargetDesc/PPUMCTargetDesc.h"
#include "Utils/PPUBaseInfo.h"
#include "llvm/ADT/BitmaskEnum.h"
#include "llvm/ADT/None.h"
#include "llvm/ADT/Optional.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineMemOperand.h"
#include "llvm/CodeGen/MachineModuleInfo.h"
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/IR/DebugLoc.h"
#include "llvm/IR/DiagnosticInfo.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/LLVMContext.h"
#include "llvm/MC/MCInstrDesc.h"
#include "llvm/Pass.h"
#include "llvm/Support/AtomicOrdering.h"
#include "llvm/Support/MathExtras.h"
#include <cassert>
#include <list>

using namespace llvm;
using namespace llvm::PPU;

#define DEBUG_TYPE "ppu-memory-legalizer"
#define PASS_NAME "PPU Memory Legalizer"

namespace {

LLVM_ENABLE_BITMASK_ENUMS_IN_NAMESPACE();

/// Memory operation flags. Can be ORed together.
enum class PPUMemOp {
  NONE = 0u,
  LOAD = 1u << 0,
  STORE = 1u << 1,
  LLVM_MARK_AS_BITMASK_ENUM(/* LargestFlag = */ STORE)
};

/// Position to insert a new instruction relative to an existing
/// instruction.
enum class Position {
  BEFORE,
  AFTER
};

/// The atomic synchronization scopes supported by the PPU target.
enum class PPUAtomicScope {
  NONE,
  SINGLETHREAD,
  WAVEFRONT,
  WORKGROUP,
  AGENT,
  SYSTEM
};

/// The distinct address spaces supported by the PPU target for
/// atomic memory operation. Can be ORed toether.
enum class PPUAtomicAddrSpace {
  NONE = 0u,
  GLOBAL = 1u << 0,
  LDS = 1u << 1,
  SCRATCH = 1u << 2,
  GDS = 1u << 3,
  OTHER = 1u << 4,

  /// The address spaces that can be accessed by a FLAT instruction.
  FLAT = GLOBAL | LDS | SCRATCH,

  /// The address spaces that support atomic instructions.
  ATOMIC = GLOBAL | LDS | SCRATCH | GDS,

  /// All address spaces.
  ALL = GLOBAL | LDS | SCRATCH | GDS | OTHER,

  LLVM_MARK_AS_BITMASK_ENUM(/* LargestFlag = */ ALL)
};

/// Sets named bit \p BitName to "true" if present in instruction \p MI.
/// \returns Returns true if \p MI is modified, false otherwise.
template <uint16_t BitName>
bool enableNamedBit(const MachineBasicBlock::iterator &MI) {
  int BitIdx = PPU::getNamedOperandIdx(MI->getOpcode(), BitName);
  if (BitIdx == -1)
    return false;

  MachineOperand &Bit = MI->getOperand(BitIdx);
  if (Bit.getImm() != 0)
    return false;

  Bit.setImm(1);
  return true;
}

class PPUMemOpInfo final {
private:

  friend class PPUMemOpAccess;

  AtomicOrdering Ordering = AtomicOrdering::NotAtomic;
  AtomicOrdering FailureOrdering = AtomicOrdering::NotAtomic;
  PPUAtomicScope Scope = PPUAtomicScope::SYSTEM;
  PPUAtomicAddrSpace OrderingAddrSpace = PPUAtomicAddrSpace::NONE;
  PPUAtomicAddrSpace InstrAddrSpace = PPUAtomicAddrSpace::NONE;
  bool IsCrossAddressSpaceOrdering = false;
  bool IsNonTemporal = false;

  PPUMemOpInfo(AtomicOrdering Ordering = AtomicOrdering::SequentiallyConsistent,
              PPUAtomicScope Scope = PPUAtomicScope::SYSTEM,
              PPUAtomicAddrSpace OrderingAddrSpace = PPUAtomicAddrSpace::ATOMIC,
              PPUAtomicAddrSpace InstrAddrSpace = PPUAtomicAddrSpace::ALL,
              bool IsCrossAddressSpaceOrdering = true,
              AtomicOrdering FailureOrdering =
                AtomicOrdering::SequentiallyConsistent,
              bool IsNonTemporal = false)
    : Ordering(Ordering), FailureOrdering(FailureOrdering),
      Scope(Scope), OrderingAddrSpace(OrderingAddrSpace),
      InstrAddrSpace(InstrAddrSpace),
      IsCrossAddressSpaceOrdering(IsCrossAddressSpaceOrdering),
      IsNonTemporal(IsNonTemporal) {
    // There is also no cross address space ordering if the ordering
    // address space is the same as the instruction address space and
    // only contains a single address space.
    if ((OrderingAddrSpace == InstrAddrSpace) &&
        isPowerOf2_32(uint32_t(InstrAddrSpace)))
      IsCrossAddressSpaceOrdering = false;
  }

public:
  /// \returns Atomic synchronization scope of the machine instruction used to
  /// create this PPUMemOpInfo.
  PPUAtomicScope getScope() const {
    return Scope;
  }

  /// \returns Ordering constraint of the machine instruction used to
  /// create this PPUMemOpInfo.
  AtomicOrdering getOrdering() const {
    return Ordering;
  }

  /// \returns Failure ordering constraint of the machine instruction used to
  /// create this PPUMemOpInfo.
  AtomicOrdering getFailureOrdering() const {
    return FailureOrdering;
  }

  /// \returns The address spaces be accessed by the machine
  /// instruction used to create this SiMemOpInfo.
  PPUAtomicAddrSpace getInstrAddrSpace() const {
    return InstrAddrSpace;
  }

  /// \returns The address spaces that must be ordered by the machine
  /// instruction used to create this SiMemOpInfo.
  PPUAtomicAddrSpace getOrderingAddrSpace() const {
    return OrderingAddrSpace;
  }

  /// \returns Return true iff memory ordering of operations on
  /// different address spaces is required.
  bool getIsCrossAddressSpaceOrdering() const {
    return IsCrossAddressSpaceOrdering;
  }

  /// \returns True if memory access of the machine instruction used to
  /// create this PPUMemOpInfo is non-temporal, false otherwise.
  bool isNonTemporal() const {
    return IsNonTemporal;
  }

  /// \returns True if ordering constraint of the machine instruction used to
  /// create this PPUMemOpInfo is unordered or higher, false otherwise.
  bool isAtomic() const {
    return Ordering != AtomicOrdering::NotAtomic;
  }

};

class PPUMemOpAccess final {
private:
  PPUMachineModuleInfo *MMI = nullptr;

  /// Reports unsupported message \p Msg for \p MI to LLVM context.
  void reportUnsupported(const MachineBasicBlock::iterator &MI,
                         const char *Msg) const;

  /// Inspects the target synchonization scope \p SSID and determines
  /// the PPU atomic scope it corresponds to, the address spaces it
  /// covers, and whether the memory ordering applies between address
  /// spaces.
  Optional<std::tuple<PPUAtomicScope, PPUAtomicAddrSpace, bool>>
  toPPUAtomicScope(SyncScope::ID SSID, PPUAtomicAddrSpace InstrScope) const;

  /// \return Return a bit set of the address spaces accessed by \p AS.
  PPUAtomicAddrSpace toPPUAtomicAddrSpace(unsigned AS) const;

  /// \returns Info constructed from \p MI, which has at least machine memory
  /// operand.
  Optional<PPUMemOpInfo> constructFromMIWithMMO(
      const MachineBasicBlock::iterator &MI) const;

public:
  /// Construct class to support accessing the machine memory operands
  /// of instructions in the machine function \p MF.
  PPUMemOpAccess(MachineFunction &MF);

  /// \returns Load info if \p MI is a load operation, "None" otherwise.
  Optional<PPUMemOpInfo> getLoadInfo(
      const MachineBasicBlock::iterator &MI) const;

  /// \returns Store info if \p MI is a store operation, "None" otherwise.
  Optional<PPUMemOpInfo> getStoreInfo(
      const MachineBasicBlock::iterator &MI) const;

  /// \returns Atomic fence info if \p MI is an atomic fence operation,
  /// "None" otherwise.
  Optional<PPUMemOpInfo> getAtomicFenceInfo(
      const MachineBasicBlock::iterator &MI) const;

  /// \returns Atomic cmpxchg/rmw info if \p MI is an atomic cmpxchg or
  /// rmw operation, "None" otherwise.
  Optional<PPUMemOpInfo> getAtomicCmpxchgOrRmwInfo(
      const MachineBasicBlock::iterator &MI) const;
};

class SICacheControl {
protected:

  /// Instruction info.
  const PPUInstrInfo *TII = nullptr;

  IsaInfo::IsaVersion IV;
  // IsaVersion IV;

  SICacheControl(const PPUSubtarget &ST);

public:

  /// Create a cache control for the subtarget \p ST.
  static std::unique_ptr<SICacheControl> create(const PPUSubtarget &ST);

  /// Update \p MI memory load instruction to bypass any caches up to
  /// the \p Scope memory scope for address spaces \p
  /// AddrSpace. Return true iff the instruction was modified.
  virtual bool enableLoadCacheBypass(const MachineBasicBlock::iterator &MI,
                                     PPUAtomicScope Scope,
                                     PPUAtomicAddrSpace AddrSpace) const = 0;

  /// Update \p MI memory instruction to indicate it is
  /// nontemporal. Return true iff the instruction was modified.
  virtual bool enableNonTemporal(const MachineBasicBlock::iterator &MI)
    const = 0;

  /// Inserts any necessary instructions at position \p Pos relative
  /// to instruction \p MI to ensure any caches associated with
  /// address spaces \p AddrSpace for memory scopes up to memory scope
  /// \p Scope are invalidated. Returns true iff any instructions
  /// inserted.
  virtual bool insertCacheInvalidate(MachineBasicBlock::iterator &MI,
                                     PPUAtomicScope Scope,
                                     PPUAtomicAddrSpace AddrSpace,
                                     Position Pos) const = 0;

  /// Inserts any necessary instructions at position \p Pos relative
  /// to instruction \p MI to ensure memory instructions of kind \p Op
  /// associated with address spaces \p AddrSpace have completed as
  /// observed by other memory instructions executing in memory scope
  /// \p Scope. \p IsCrossAddrSpaceOrdering indicates if the memory
  /// ordering is between address spaces. Returns true iff any
  /// instructions inserted.
  virtual bool insertWait(MachineBasicBlock::iterator &MI,
                          PPUAtomicScope Scope,
                          PPUAtomicAddrSpace AddrSpace,
                          PPUMemOp Op,
                          bool IsCrossAddrSpaceOrdering,
                          Position Pos) const = 0;

  /// Virtual destructor to allow derivations to be deleted.
  virtual ~SICacheControl() = default;

};

class SIGfx6CacheControl : public SICacheControl {
protected:

  /// Sets GLC bit to "true" if present in \p MI. Returns true if \p MI
  /// is modified, false otherwise.
  bool enableGLCBit(const MachineBasicBlock::iterator &MI) const {
    return true;
    //return enableNamedBit<PPU::OpName::glc>(MI);
  }

  /// Sets SLC bit to "true" if present in \p MI. Returns true if \p MI
  /// is modified, false otherwise.
  bool enableSLCBit(const MachineBasicBlock::iterator &MI) const {
    // FIXME return enableNamedBit<PPU::OpName::slc>(MI);
    return false;
  }

  /*
  bool enableKOPBit(const MachineBasicBlock::iterator &MI, unsigned imm) const {
    return enableNamedBit<PPU::OpName::kop>(MI);
  }
  */

public:

  SIGfx6CacheControl(const PPUSubtarget &ST) : SICacheControl(ST) {};

  bool enableLoadCacheBypass(const MachineBasicBlock::iterator &MI,
                             PPUAtomicScope Scope,
                             PPUAtomicAddrSpace AddrSpace) const override;

  bool enableNonTemporal(const MachineBasicBlock::iterator &MI) const override;

  bool insertCacheInvalidate(MachineBasicBlock::iterator &MI,
                             PPUAtomicScope Scope,
                             PPUAtomicAddrSpace AddrSpace,
                             Position Pos) const override;

  bool insertWait(MachineBasicBlock::iterator &MI,
                  PPUAtomicScope Scope,
                  PPUAtomicAddrSpace AddrSpace,
                  PPUMemOp Op,
                  bool IsCrossAddrSpaceOrdering,
                  Position Pos) const override;
};

class SIGfx7CacheControl : public SIGfx6CacheControl {
public:

  SIGfx7CacheControl(const PPUSubtarget &ST) : SIGfx6CacheControl(ST) {};

  bool insertCacheInvalidate(MachineBasicBlock::iterator &MI,
                             PPUAtomicScope Scope,
                             PPUAtomicAddrSpace AddrSpace,
                             Position Pos) const override;

};
/*
class SIGfx10CacheControl : public SIGfx7CacheControl {
protected:
  bool CuMode = false;

  /// Sets DLC bit to "true" if present in \p MI. Returns true if \p MI
  /// is modified, false otherwise.
  bool enableDLCBit(const MachineBasicBlock::iterator &MI) const {
    return enableNamedBit<PPU::OpName::dlc>(MI);
  }

public:

  SIGfx10CacheControl(const PPUSubtarget &ST, bool CuMode) :
    SIGfx7CacheControl(ST), CuMode(CuMode) {};

  bool enableLoadCacheBypass(const MachineBasicBlock::iterator &MI,
                             PPUAtomicScope Scope,
                             PPUAtomicAddrSpace AddrSpace) const override;

  bool enableNonTemporal(const MachineBasicBlock::iterator &MI) const override;

  bool insertCacheInvalidate(MachineBasicBlock::iterator &MI,
                             PPUAtomicScope Scope,
                             PPUAtomicAddrSpace AddrSpace,
                             Position Pos) const override;

  bool insertWait(MachineBasicBlock::iterator &MI,
                  PPUAtomicScope Scope,
                  PPUAtomicAddrSpace AddrSpace,
                  PPUMemOp Op,
                  bool IsCrossAddrSpaceOrdering,
                  Position Pos) const override;
};
*/

class PPUMemoryLegalizer final : public MachineFunctionPass {
private:

  /// Cache Control.
  std::unique_ptr<SICacheControl> CC = nullptr;

  /// List of atomic pseudo instructions.
  std::list<MachineBasicBlock::iterator> AtomicPseudoMIs;

  /// Return true iff instruction \p MI is a atomic instruction that
  /// returns a result.
  bool isAtomicRet(const MachineInstr &MI) const {
    // FIXME return PPU::getAtomicNoRetOp(MI.getOpcode()) != -1;
    return false;
  }

  /// Removes all processed atomic pseudo instructions from the current
  /// function. Returns true if current function is modified, false otherwise.
  bool removeAtomicPseudoMIs();

  /// Expands load operation \p MI. Returns true if instructions are
  /// added/deleted or \p MI is modified, false otherwise.
  bool expandLoad(const PPUMemOpInfo &MOI,
                  MachineBasicBlock::iterator &MI);
  /// Expands store operation \p MI. Returns true if instructions are
  /// added/deleted or \p MI is modified, false otherwise.
  bool expandStore(const PPUMemOpInfo &MOI,
                   MachineBasicBlock::iterator &MI);
  /// Expands atomic fence operation \p MI. Returns true if
  /// instructions are added/deleted or \p MI is modified, false otherwise.
  bool expandAtomicFence(const PPUMemOpInfo &MOI,
                         MachineBasicBlock::iterator &MI);
  /// Expands atomic cmpxchg or rmw operation \p MI. Returns true if
  /// instructions are added/deleted or \p MI is modified, false otherwise.
  bool expandAtomicCmpxchgOrRmw(const PPUMemOpInfo &MOI,
                                MachineBasicBlock::iterator &MI);

public:
  static char ID;

  PPUMemoryLegalizer() : MachineFunctionPass(ID) {}

  void getAnalysisUsage(AnalysisUsage &AU) const override {
    AU.setPreservesCFG();
    MachineFunctionPass::getAnalysisUsage(AU);
  }

  StringRef getPassName() const override {
    return PASS_NAME;
  }

  bool runOnMachineFunction(MachineFunction &MF) override;
};

} // end namespace anonymous

void PPUMemOpAccess::reportUnsupported(const MachineBasicBlock::iterator &MI,
                                      const char *Msg) const {
  const Function &Func = MI->getParent()->getParent()->getFunction();
  DiagnosticInfoUnsupported Diag(Func, Msg, MI->getDebugLoc());
  Func.getContext().diagnose(Diag);
}

Optional<std::tuple<PPUAtomicScope, PPUAtomicAddrSpace, bool>>
PPUMemOpAccess::toPPUAtomicScope(SyncScope::ID SSID,
                               PPUAtomicAddrSpace InstrScope) const {
  if (SSID == SyncScope::System)
    return std::make_tuple(PPUAtomicScope::SYSTEM,
                           PPUAtomicAddrSpace::ATOMIC,
                           true);
  if (SSID == MMI->getAgentSSID())
    return std::make_tuple(PPUAtomicScope::AGENT,
                           PPUAtomicAddrSpace::ATOMIC,
                           true);
  if (SSID == MMI->getWorkgroupSSID())
    return std::make_tuple(PPUAtomicScope::WORKGROUP,
                           PPUAtomicAddrSpace::ATOMIC,
                           true);
  if (SSID == MMI->getWavefrontSSID())
    return std::make_tuple(PPUAtomicScope::WAVEFRONT,
                           PPUAtomicAddrSpace::ATOMIC,
                           true);
  if (SSID == SyncScope::SingleThread)
    return std::make_tuple(PPUAtomicScope::SINGLETHREAD,
                           PPUAtomicAddrSpace::ATOMIC,
                           true);
  if (SSID == MMI->getSystemOneAddressSpaceSSID())
    return std::make_tuple(PPUAtomicScope::SYSTEM,
                           PPUAtomicAddrSpace::ATOMIC & InstrScope,
                           false);
  if (SSID == MMI->getAgentOneAddressSpaceSSID())
    return std::make_tuple(PPUAtomicScope::AGENT,
                           PPUAtomicAddrSpace::ATOMIC & InstrScope,
                           false);
  if (SSID == MMI->getWorkgroupOneAddressSpaceSSID())
    return std::make_tuple(PPUAtomicScope::WORKGROUP,
                           PPUAtomicAddrSpace::ATOMIC & InstrScope,
                           false);
  if (SSID == MMI->getWavefrontOneAddressSpaceSSID())
    return std::make_tuple(PPUAtomicScope::WAVEFRONT,
                           PPUAtomicAddrSpace::ATOMIC & InstrScope,
                           false);
  if (SSID == MMI->getSingleThreadOneAddressSpaceSSID())
    return std::make_tuple(PPUAtomicScope::SINGLETHREAD,
                           PPUAtomicAddrSpace::ATOMIC & InstrScope,
                           false);
  /// TODO: To support HSA Memory Model need to add additional memory
  /// scopes that specify that do require cross address space
  /// ordering.
  return None;
}

PPUAtomicAddrSpace PPUMemOpAccess::toPPUAtomicAddrSpace(unsigned AS) const {
  if (AS == AMDGPUAS::FLAT_ADDRESS)
    return PPUAtomicAddrSpace::FLAT;
  if (AS == AMDGPUAS::GLOBAL_ADDRESS)
    return PPUAtomicAddrSpace::GLOBAL;
  if (AS == AMDGPUAS::LOCAL_ADDRESS)
    return PPUAtomicAddrSpace::LDS;
  if (AS == AMDGPUAS::PRIVATE_ADDRESS)
    return PPUAtomicAddrSpace::SCRATCH;
  if (AS == AMDGPUAS::REGION_ADDRESS)
    return PPUAtomicAddrSpace::GDS;

  return PPUAtomicAddrSpace::OTHER;
}

PPUMemOpAccess::PPUMemOpAccess(MachineFunction &MF) {
  MMI = &MF.getMMI().getObjFileInfo<PPUMachineModuleInfo>();
}

Optional<PPUMemOpInfo> PPUMemOpAccess::constructFromMIWithMMO(
    const MachineBasicBlock::iterator &MI) const {
  assert(MI->getNumMemOperands() > 0);

  SyncScope::ID SSID = SyncScope::SingleThread;
  AtomicOrdering Ordering = AtomicOrdering::NotAtomic;
  AtomicOrdering FailureOrdering = AtomicOrdering::NotAtomic;
  PPUAtomicAddrSpace InstrAddrSpace = PPUAtomicAddrSpace::NONE;
  bool IsNonTemporal = true;

  // Validator should check whether or not MMOs cover the entire set of
  // locations accessed by the memory instruction.
  for (const auto &MMO : MI->memoperands()) {
    IsNonTemporal &= MMO->isNonTemporal();
    InstrAddrSpace |=
      toPPUAtomicAddrSpace(MMO->getPointerInfo().getAddrSpace());
    AtomicOrdering OpOrdering = MMO->getOrdering();
    if (OpOrdering != AtomicOrdering::NotAtomic) {
      const auto &IsSyncScopeInclusion =
          MMI->isSyncScopeInclusion(SSID, MMO->getSyncScopeID());
      if (!IsSyncScopeInclusion) {
        reportUnsupported(MI,
          "Unsupported non-inclusive atomic synchronization scope");
        return None;
      }

      SSID = IsSyncScopeInclusion.getValue() ? SSID : MMO->getSyncScopeID();
      Ordering =
          isStrongerThan(Ordering, OpOrdering) ?
              Ordering : MMO->getOrdering();
      assert(MMO->getFailureOrdering() != AtomicOrdering::Release &&
             MMO->getFailureOrdering() != AtomicOrdering::AcquireRelease);
      FailureOrdering =
          isStrongerThan(FailureOrdering, MMO->getFailureOrdering()) ?
              FailureOrdering : MMO->getFailureOrdering();
    }
  }

  PPUAtomicScope Scope = PPUAtomicScope::NONE;
  PPUAtomicAddrSpace OrderingAddrSpace = PPUAtomicAddrSpace::NONE;
  bool IsCrossAddressSpaceOrdering = false;
  if (Ordering != AtomicOrdering::NotAtomic) {
    auto ScopeOrNone = toPPUAtomicScope(SSID, InstrAddrSpace);
    if (!ScopeOrNone) {
      reportUnsupported(MI, "Unsupported atomic synchronization scope");
      return None;
    }
    std::tie(Scope, OrderingAddrSpace, IsCrossAddressSpaceOrdering) =
      ScopeOrNone.getValue();
    if ((OrderingAddrSpace == PPUAtomicAddrSpace::NONE) ||
        ((OrderingAddrSpace & PPUAtomicAddrSpace::ATOMIC) != OrderingAddrSpace)) {
      reportUnsupported(MI, "Unsupported atomic address space");
      return None;
    }
  }
  return PPUMemOpInfo(Ordering, Scope, OrderingAddrSpace, InstrAddrSpace,
                     IsCrossAddressSpaceOrdering, FailureOrdering, IsNonTemporal);
}

Optional<PPUMemOpInfo> PPUMemOpAccess::getLoadInfo(
    const MachineBasicBlock::iterator &MI) const {
  assert(MI->getDesc().TSFlags & PPUInstrFlags::maybeAtomic);

  if (!(MI->mayLoad() && !MI->mayStore()))
    return None;

  // Be conservative if there are no memory operands.
  if (MI->getNumMemOperands() == 0)
    return PPUMemOpInfo();

  return constructFromMIWithMMO(MI);
}

Optional<PPUMemOpInfo> PPUMemOpAccess::getStoreInfo(
    const MachineBasicBlock::iterator &MI) const {
  assert(MI->getDesc().TSFlags & PPUInstrFlags::maybeAtomic);

  if (!(!MI->mayLoad() && MI->mayStore()))
    return None;

  // Be conservative if there are no memory operands.
  if (MI->getNumMemOperands() == 0)
    return PPUMemOpInfo();

  return constructFromMIWithMMO(MI);
}

Optional<PPUMemOpInfo> PPUMemOpAccess::getAtomicFenceInfo(
    const MachineBasicBlock::iterator &MI) const {
  assert(MI->getDesc().TSFlags & PPUInstrFlags::maybeAtomic);
  /* FIXME

  if (MI->getOpcode() != PPU::ATOMIC_FENCE)
    return None;
    */

  AtomicOrdering Ordering =
    static_cast<AtomicOrdering>(MI->getOperand(0).getImm());

  SyncScope::ID SSID = static_cast<SyncScope::ID>(MI->getOperand(1).getImm());
  auto ScopeOrNone = toPPUAtomicScope(SSID, PPUAtomicAddrSpace::ATOMIC);
  if (!ScopeOrNone) {
    reportUnsupported(MI, "Unsupported atomic synchronization scope");
    return None;
  }

  PPUAtomicScope Scope = PPUAtomicScope::NONE;
  PPUAtomicAddrSpace OrderingAddrSpace = PPUAtomicAddrSpace::NONE;
  bool IsCrossAddressSpaceOrdering = false;
  std::tie(Scope, OrderingAddrSpace, IsCrossAddressSpaceOrdering) =
    ScopeOrNone.getValue();

  if ((OrderingAddrSpace == PPUAtomicAddrSpace::NONE) ||
      ((OrderingAddrSpace & PPUAtomicAddrSpace::ATOMIC) != OrderingAddrSpace)) {
    reportUnsupported(MI, "Unsupported atomic address space");
    return None;
  }

  return PPUMemOpInfo(Ordering, Scope, OrderingAddrSpace, PPUAtomicAddrSpace::ATOMIC,
                     IsCrossAddressSpaceOrdering);
}

Optional<PPUMemOpInfo> PPUMemOpAccess::getAtomicCmpxchgOrRmwInfo(
    const MachineBasicBlock::iterator &MI) const {
  assert(MI->getDesc().TSFlags & PPUInstrFlags::maybeAtomic);

  if (!(MI->mayLoad() && MI->mayStore()))
    return None;

  // Be conservative if there are no memory operands.
  if (MI->getNumMemOperands() == 0)
    return PPUMemOpInfo();

  return constructFromMIWithMMO(MI);
}

SICacheControl::SICacheControl(const PPUSubtarget &ST) {
  TII = ST.getInstrInfo();
  // IV = IsaInfo::getIsaVersion(ST.getFeatureBits());
  // IV = getIsaVersion(ST.getCPU());
}

/* static */
std::unique_ptr<SICacheControl> SICacheControl::create(const PPUSubtarget &ST) {
  /*
  PPUSubtarget::Generation Generation = ST.getGeneration();
  if (Generation <= PPUSubtarget::SOUTHERN_ISLANDS)
    return std::make_unique<SIGfx6CacheControl>(ST);
  if (Generation < PPUSubtarget::GFX10)
    return std::make_unique<SIGfx7CacheControl>(ST);
  return std::make_unique<SIGfx10CacheControl>(ST, ST.isCuModeEnabled());
  */
    return std::make_unique<SIGfx7CacheControl>(ST);
}

bool SIGfx6CacheControl::enableLoadCacheBypass(
    const MachineBasicBlock::iterator &MI,
    PPUAtomicScope Scope,
    PPUAtomicAddrSpace AddrSpace) const {
  assert(MI->mayLoad() && !MI->mayStore());
  bool Changed = false;

  if ((AddrSpace & PPUAtomicAddrSpace::GLOBAL) != PPUAtomicAddrSpace::NONE) {
    /// TODO: Do not set glc for rmw atomic operations as they
    /// implicitly bypass the L1 cache.

    switch (Scope) {
    case PPUAtomicScope::SYSTEM:
    case PPUAtomicScope::AGENT:
      Changed |= enableGLCBit(MI);
      break;
    case PPUAtomicScope::WORKGROUP:
    case PPUAtomicScope::WAVEFRONT:
    case PPUAtomicScope::SINGLETHREAD:
      // No cache to bypass.
      break;
    default:
      llvm_unreachable("Unsupported synchronization scope");
    }
  }

  /// The scratch address space does not need the global memory caches
  /// to be bypassed as all memory operations by the same thread are
  /// sequentially consistent, and no other thread can access scratch
  /// memory.

  /// Other address spaces do not hava a cache.

  return Changed;
}

bool SIGfx6CacheControl::enableNonTemporal(
    const MachineBasicBlock::iterator &MI) const {
  assert(MI->mayLoad() ^ MI->mayStore());
  bool Changed = false;

  /// TODO: Do not enableGLCBit if rmw atomic.
  Changed |= enableGLCBit(MI);
  Changed |= enableSLCBit(MI);

  return Changed;
}

bool SIGfx6CacheControl::insertCacheInvalidate(MachineBasicBlock::iterator &MI,
                                               PPUAtomicScope Scope,
                                               PPUAtomicAddrSpace AddrSpace,
                                               Position Pos) const {
  bool Changed = false;

  MachineBasicBlock &MBB = *MI->getParent();
  DebugLoc DL = MI->getDebugLoc();

  if (Pos == Position::AFTER)
    ++MI;

  if ((AddrSpace & PPUAtomicAddrSpace::GLOBAL) != PPUAtomicAddrSpace::NONE) {
    switch (Scope) {
    case PPUAtomicScope::SYSTEM:
    case PPUAtomicScope::AGENT:
      // BuildMI(MBB, MI, DL, TII->get(PPU::ML_LSA_WPPUNV));
      // FIXME BuildMI(MBB, MI, DL, TII->get(PPU::BUFFER_WBINVL1));
      Changed = true;
      break;
    case PPUAtomicScope::WORKGROUP:
    case PPUAtomicScope::WAVEFRONT:
    case PPUAtomicScope::SINGLETHREAD:
      // No cache to invalidate.
      break;
    default:
      llvm_unreachable("Unsupported synchronization scope");
    }
  }

  /// The scratch address space does not need the global memory cache
  /// to be flushed as all memory operations by the same thread are
  /// sequentially consistent, and no other thread can access scratch
  /// memory.

  /// Other address spaces do not hava a cache.

  if (Pos == Position::AFTER)
    --MI;

  return Changed;
}

bool SIGfx6CacheControl::insertWait(MachineBasicBlock::iterator &MI,
                                    PPUAtomicScope Scope,
                                    PPUAtomicAddrSpace AddrSpace,
                                    PPUMemOp Op,
                                    bool IsCrossAddrSpaceOrdering,
                                    Position Pos) const {
  bool Changed = false;

  MachineBasicBlock &MBB = *MI->getParent();
  DebugLoc DL = MI->getDebugLoc();

  if (Pos == Position::AFTER)
    ++MI;

  bool VMCnt = false;
  bool LGKMCnt = false;
/*
  bool SMCnt = false;
  bool LMCnt = false;
  bool MSGCnt = false;
*/
  if ((AddrSpace & PPUAtomicAddrSpace::GLOBAL) != PPUAtomicAddrSpace::NONE) {
    switch (Scope) {
    case PPUAtomicScope::SYSTEM:
    case PPUAtomicScope::AGENT:
      VMCnt = true;
      break;
    case PPUAtomicScope::WORKGROUP:
    case PPUAtomicScope::WAVEFRONT:
    case PPUAtomicScope::SINGLETHREAD:
      // The L1 cache keeps all memory operations in order for
      // wavefronts in the same work-group.
      break;
    default:
      llvm_unreachable("Unsupported synchronization scope");
    }
  }

  if ((AddrSpace & PPUAtomicAddrSpace::LDS) != PPUAtomicAddrSpace::NONE) {
    switch (Scope) {
    case PPUAtomicScope::SYSTEM:
    case PPUAtomicScope::AGENT:
    case PPUAtomicScope::WORKGROUP:
      // If no cross address space ordering then an LDS waitcnt is not
      // needed as LDS operations for all waves are executed in a
      // total global ordering as observed by all waves. Required if
      // also synchronizing with global/GDS memory as LDS operations
      // could be reordered with respect to later global/GDS memory
      // operations of the same wave.
      LGKMCnt |= IsCrossAddrSpaceOrdering;
      /*
      SMCnt = IsCrossAddrSpaceOrdering;
      LMCnt = IsCrossAddrSpaceOrdering;
      MSGCnt = IsCrossAddrSpaceOrdering;
      */
      break;
    case PPUAtomicScope::WAVEFRONT:
    case PPUAtomicScope::SINGLETHREAD:
      // The LDS keeps all memory operations in order for
      // the same wavesfront.
      break;
    default:
      llvm_unreachable("Unsupported synchronization scope");
    }
  }

#if 0
  if ((AddrSpace & PPUAtomicAddrSpace::GDS) != PPUAtomicAddrSpace::NONE) {
    switch (Scope) {
    case PPUAtomicScope::SYSTEM:
    case PPUAtomicScope::AGENT:
      // If no cross address space ordering then an GDS waitcnt is not
      // needed as GDS operations for all waves are executed in a
      // total global ordering as observed by all waves. Required if
      // also synchronizing with global/LDS memory as GDS operations
      // could be reordered with respect to later global/LDS memory
      // operations of the same wave.
      LGKMCnt |= IsCrossAddrSpaceOrdering;
      // EXPCnt = IsCrossAddrSpaceOrdering;
      break;
    case PPUAtomicScope::WORKGROUP:
    case PPUAtomicScope::WAVEFRONT:
    case PPUAtomicScope::SINGLETHREAD:
      // The GDS keeps all memory operations in order for
      // the same work-group.
      break;
    default:
      llvm_unreachable("Unsupported synchronization scope");
    }
  }
#endif

    /* FIXME
  // if (VMCnt || SMCnt || LMCnt || MSGCnt) {
  if (VMCnt || LGKMCnt) {
    unsigned WaitCntImmediate =
      PPU::encodeWaitcnt(IV,
                            VMCnt ? 0 : getVmcntBitMask(IV),
                            getExpcntBitMask(IV),
                            LGKMCnt ? 0 : getLgkmcntBitMask(IV));
                            SMCnt ? 0 : getSmcntBitMask(IV),
                            LMCnt ? 0 : getLmcntBitMask(IV),
                            MSGCnt ? 0 : getMsgcntBitMask(IV),
                            getPbcntBitMask(IV));
    // BuildMI(MBB, MI, DL, TII->get(PPU::SL_WAIT)).addImm(WaitCntImmediate);
    BuildMI(MBB, MI, DL, TII->get(PPU::S_WAITCNT)).addImm(WaitCntImmediate);
    Changed = true;
  }
                            */

  if (Pos == Position::AFTER)
    --MI;

  return Changed;
}

bool SIGfx7CacheControl::insertCacheInvalidate(MachineBasicBlock::iterator &MI,
                                               PPUAtomicScope Scope,
                                               PPUAtomicAddrSpace AddrSpace,
                                               Position Pos) const {
  bool Changed = false;

  MachineBasicBlock &MBB = *MI->getParent();
  DebugLoc DL = MI->getDebugLoc();

  const PPUSubtarget &STM = MBB.getParent()->getSubtarget<PPUSubtarget>();

 // FIXME  const unsigned Flush = PPU::BUFFER_WBINVL1_VOL;

  if (Pos == Position::AFTER)
    ++MI;

  if ((AddrSpace & PPUAtomicAddrSpace::GLOBAL) != PPUAtomicAddrSpace::NONE) {
    switch (Scope) {
    case PPUAtomicScope::SYSTEM:
    case PPUAtomicScope::AGENT:
      // FIXME BuildMI(MBB, MI, DL, TII->get(Flush));
      // BuildMI(MBB, MI, DL, TII->get(PPU::ML_LSA_WPPUNV));
      Changed = true;
      break;
    case PPUAtomicScope::WORKGROUP:
    case PPUAtomicScope::WAVEFRONT:
    case PPUAtomicScope::SINGLETHREAD:
      // No cache to invalidate.
      break;
    default:
      llvm_unreachable("Unsupported synchronization scope");
    }
  }

  /// The scratch address space does not need the global memory cache
  /// to be flushed as all memory operations by the same thread are
  /// sequentially consistent, and no other thread can access scratch
  /// memory.

  /// Other address spaces do not hava a cache.

  if (Pos == Position::AFTER)
    --MI;

  return Changed;
}
/*
bool SIGfx10CacheControl::enableLoadCacheBypass(
    const MachineBasicBlock::iterator &MI,
    PPUAtomicScope Scope,
    PPUAtomicAddrSpace AddrSpace) const {
  assert(MI->mayLoad() && !MI->mayStore());
  bool Changed = false;

  if ((AddrSpace & PPUAtomicAddrSpace::GLOBAL) != PPUAtomicAddrSpace::NONE) {
    /// TODO Do not set glc for rmw atomic operations as they
    /// implicitly bypass the L0/L1 caches.

    switch (Scope) {
    case PPUAtomicScope::SYSTEM:
    case PPUAtomicScope::AGENT:
      Changed |= enableGLCBit(MI);
      Changed |= enableDLCBit(MI);
      break;
    case PPUAtomicScope::WORKGROUP:
      // In WGP mode the waves of a work-group can be executing on either CU of
      // the WGP. Therefore need to bypass the L0 which is per CU. Otherwise in
      // CU mode and all waves of a work-group are on the same CU, and so the
      // L0 does not need to be bypassed.
      if (!CuMode) Changed |= enableGLCBit(MI);
      break;
    case PPUAtomicScope::WAVEFRONT:
    case PPUAtomicScope::SINGLETHREAD:
      // No cache to bypass.
      break;
    default:
      llvm_unreachable("Unsupported synchronization scope");
    }
  }

  /// The scratch address space does not need the global memory caches
  /// to be bypassed as all memory operations by the same thread are
  /// sequentially consistent, and no other thread can access scratch
  /// memory.

  /// Other address spaces do not hava a cache.

  return Changed;
}

bool SIGfx10CacheControl::enableNonTemporal(
    const MachineBasicBlock::iterator &MI) const {
  assert(MI->mayLoad() ^ MI->mayStore());
  bool Changed = false;

  Changed |= enableSLCBit(MI);
  /// TODO for store (non-rmw atomic) instructions also enableGLCBit(MI)

  return Changed;
}

bool SIGfx10CacheControl::insertCacheInvalidate(MachineBasicBlock::iterator &MI,
                                                PPUAtomicScope Scope,
                                                PPUAtomicAddrSpace AddrSpace,
                                                Position Pos) const {
  bool Changed = false;

  MachineBasicBlock &MBB = *MI->getParent();
  DebugLoc DL = MI->getDebugLoc();

  if (Pos == Position::AFTER)
    ++MI;

  if ((AddrSpace & PPUAtomicAddrSpace::GLOBAL) != PPUAtomicAddrSpace::NONE) {
    switch (Scope) {
    case PPUAtomicScope::SYSTEM:
    case PPUAtomicScope::AGENT:
      BuildMI(MBB, MI, DL, TII->get(PPU::BUFFER_GL0_INV));
      BuildMI(MBB, MI, DL, TII->get(PPU::BUFFER_GL1_INV));
      Changed = true;
      break;
    case PPUAtomicScope::WORKGROUP:
      // In WGP mode the waves of a work-group can be executing on either CU of
      // the WGP. Therefore need to invalidate the L0 which is per CU. Otherwise
      // in CU mode and all waves of a work-group are on the same CU, and so the
      // L0 does not need to be invalidated.
      if (!CuMode) {
        BuildMI(MBB, MI, DL, TII->get(PPU::BUFFER_GL0_INV));
        Changed = true;
      }
      break;
    case PPUAtomicScope::WAVEFRONT:
    case PPUAtomicScope::SINGLETHREAD:
      // No cache to invalidate.
      break;
    default:
      llvm_unreachable("Unsupported synchronization scope");
    }
  }

  /// The scratch address space does not need the global memory cache
  /// to be flushed as all memory operations by the same thread are
  /// sequentially consistent, and no other thread can access scratch
  /// memory.

  /// Other address spaces do not hava a cache.

  if (Pos == Position::AFTER)
    --MI;

  return Changed;
}

bool SIGfx10CacheControl::insertWait(MachineBasicBlock::iterator &MI,
                                     PPUAtomicScope Scope,
                                     PPUAtomicAddrSpace AddrSpace,
                                     PPUMemOp Op,
                                     bool IsCrossAddrSpaceOrdering,
                                     Position Pos) const {
  bool Changed = false;

  MachineBasicBlock &MBB = *MI->getParent();
  DebugLoc DL = MI->getDebugLoc();

  if (Pos == Position::AFTER)
    ++MI;

  bool VMCnt = false;
  bool VSCnt = false;
  bool LGKMCnt = false;

  if ((AddrSpace & PPUAtomicAddrSpace::GLOBAL) != PPUAtomicAddrSpace::NONE) {
    switch (Scope) {
    case PPUAtomicScope::SYSTEM:
    case PPUAtomicScope::AGENT:
      if ((Op & PPUMemOp::LOAD) != PPUMemOp::NONE)
        VMCnt |= true;
      if ((Op & PPUMemOp::STORE) != PPUMemOp::NONE)
        VSCnt |= true;
      break;
    case PPUAtomicScope::WORKGROUP:
      // In WGP mode the waves of a work-group can be executing on either CU of
      // the WGP. Therefore need to wait for operations to complete to ensure
      // they are visible to waves in the other CU as the L0 is per CU.
      // Otherwise in CU mode and all waves of a work-group are on the same CU
      // which shares the same L0.
      if (!CuMode) {
        if ((Op & PPUMemOp::LOAD) != PPUMemOp::NONE)
          VMCnt |= true;
        if ((Op & PPUMemOp::STORE) != PPUMemOp::NONE)
          VSCnt |= true;
      }
      break;
    case PPUAtomicScope::WAVEFRONT:
    case PPUAtomicScope::SINGLETHREAD:
      // The L0 cache keeps all memory operations in order for
      // work-items in the same wavefront.
      break;
    default:
      llvm_unreachable("Unsupported synchronization scope");
    }
  }

  if ((AddrSpace & PPUAtomicAddrSpace::LDS) != PPUAtomicAddrSpace::NONE) {
    switch (Scope) {
    case PPUAtomicScope::SYSTEM:
    case PPUAtomicScope::AGENT:
    case PPUAtomicScope::WORKGROUP:
      // If no cross address space ordering then an LDS waitcnt is not
      // needed as LDS operations for all waves are executed in a
      // total global ordering as observed by all waves. Required if
      // also synchronizing with global/GDS memory as LDS operations
      // could be reordered with respect to later global/GDS memory
      // operations of the same wave.
      LGKMCnt |= IsCrossAddrSpaceOrdering;
      break;
    case PPUAtomicScope::WAVEFRONT:
    case PPUAtomicScope::SINGLETHREAD:
      // The LDS keeps all memory operations in order for
      // the same wavesfront.
      break;
    default:
      llvm_unreachable("Unsupported synchronization scope");
    }
  }

  if ((AddrSpace & PPUAtomicAddrSpace::GDS) != PPUAtomicAddrSpace::NONE) {
    switch (Scope) {
    case PPUAtomicScope::SYSTEM:
    case PPUAtomicScope::AGENT:
      // If no cross address space ordering then an GDS waitcnt is not
      // needed as GDS operations for all waves are executed in a
      // total global ordering as observed by all waves. Required if
      // also synchronizing with global/LDS memory as GDS operations
      // could be reordered with respect to later global/LDS memory
      // operations of the same wave.
      LGKMCnt |= IsCrossAddrSpaceOrdering;
      break;
    case PPUAtomicScope::WORKGROUP:
    case PPUAtomicScope::WAVEFRONT:
    case PPUAtomicScope::SINGLETHREAD:
      // The GDS keeps all memory operations in order for
      // the same work-group.
      break;
    default:
      llvm_unreachable("Unsupported synchronization scope");
    }
  }

  if (VMCnt || LGKMCnt) {
    unsigned WaitCntImmediate =
      PPU::encodeWaitcnt(IV,
                            VMCnt ? 0 : getVmcntBitMask(IV),
                            getExpcntBitMask(IV),
                            LGKMCnt ? 0 : getLgkmcntBitMask(IV));
    BuildMI(MBB, MI, DL, TII->get(PPU::S_WAITCNT)).addImm(WaitCntImmediate);
    Changed = true;
  }

  if (VSCnt) {
    BuildMI(MBB, MI, DL, TII->get(PPU::S_WAITCNT_VSCNT))
      .addReg(PPU::SGPR_NULL, RegState::Undef)
      .addImm(0);
    Changed = true;
  }

  if (Pos == Position::AFTER)
    --MI;

  return Changed;
}
*/

bool PPUMemoryLegalizer::removeAtomicPseudoMIs() {
  if (AtomicPseudoMIs.empty())
    return false;

  for (auto &MI : AtomicPseudoMIs)
    MI->eraseFromParent();

  AtomicPseudoMIs.clear();
  return true;
}

bool PPUMemoryLegalizer::expandLoad(const PPUMemOpInfo &MOI,
                                   MachineBasicBlock::iterator &MI) {
  assert(MI->mayLoad() && !MI->mayStore());

  bool Changed = false;

  if (MOI.isAtomic()) {
    if (MOI.getOrdering() == AtomicOrdering::Monotonic ||
        MOI.getOrdering() == AtomicOrdering::Acquire ||
        MOI.getOrdering() == AtomicOrdering::SequentiallyConsistent) {
      Changed |= CC->enableLoadCacheBypass(MI, MOI.getScope(),
                                           MOI.getOrderingAddrSpace());
    }

    if (MOI.getOrdering() == AtomicOrdering::SequentiallyConsistent)
      Changed |= CC->insertWait(MI, MOI.getScope(),
                                MOI.getOrderingAddrSpace(),
                                PPUMemOp::LOAD | PPUMemOp::STORE,
                                MOI.getIsCrossAddressSpaceOrdering(),
                                Position::BEFORE);

    if (MOI.getOrdering() == AtomicOrdering::Acquire ||
        MOI.getOrdering() == AtomicOrdering::SequentiallyConsistent) {
      Changed |= CC->insertWait(MI, MOI.getScope(),
                                MOI.getInstrAddrSpace(),
                                PPUMemOp::LOAD,
                                MOI.getIsCrossAddressSpaceOrdering(),
                                Position::AFTER);
      Changed |= CC->insertCacheInvalidate(MI, MOI.getScope(),
                                           MOI.getOrderingAddrSpace(),
                                           Position::AFTER);
    }

    return Changed;
  }

  // Atomic instructions do not have the nontemporal attribute.
  if (MOI.isNonTemporal()) {
    Changed |= CC->enableNonTemporal(MI);
    return Changed;
  }

  return Changed;
}

bool PPUMemoryLegalizer::expandStore(const PPUMemOpInfo &MOI,
                                    MachineBasicBlock::iterator &MI) {
  assert(!MI->mayLoad() && MI->mayStore());

  bool Changed = false;

  if (MOI.isAtomic()) {
    if (MOI.getOrdering() == AtomicOrdering::Release ||
        MOI.getOrdering() == AtomicOrdering::SequentiallyConsistent)
      Changed |= CC->insertWait(MI, MOI.getScope(),
                                MOI.getOrderingAddrSpace(),
                                PPUMemOp::LOAD | PPUMemOp::STORE,
                                MOI.getIsCrossAddressSpaceOrdering(),
                                Position::BEFORE);

    return Changed;
  }

  // Atomic instructions do not have the nontemporal attribute.
  if (MOI.isNonTemporal()) {
    Changed |= CC->enableNonTemporal(MI);
    return Changed;
  }

  return Changed;
}

bool PPUMemoryLegalizer::expandAtomicFence(const PPUMemOpInfo &MOI,
                                          MachineBasicBlock::iterator &MI) {
  // FIXME assert(MI->getOpcode() == PPU::ATOMIC_FENCE);

  AtomicPseudoMIs.push_back(MI);
  bool Changed = false;

  if (MOI.isAtomic()) {
    if (MOI.getOrdering() == AtomicOrdering::Acquire ||
        MOI.getOrdering() == AtomicOrdering::Release ||
        MOI.getOrdering() == AtomicOrdering::AcquireRelease ||
        MOI.getOrdering() == AtomicOrdering::SequentiallyConsistent)
      /// TODO: This relies on a barrier always generating a waitcnt
      /// for LDS to ensure it is not reordered with the completion of
      /// the proceeding LDS operations. If barrier had a memory
      /// ordering and memory scope, then library does not need to
      /// generate a fence. Could add support in this file for
      /// barrier. SIInsertWaitcnt.cpp could then stop unconditionally
      /// adding waitcnt before a S_BARRIER.
      Changed |= CC->insertWait(MI, MOI.getScope(),
                                MOI.getOrderingAddrSpace(),
                                PPUMemOp::LOAD | PPUMemOp::STORE,
                                MOI.getIsCrossAddressSpaceOrdering(),
                                Position::BEFORE);

    if (MOI.getOrdering() == AtomicOrdering::Acquire ||
        MOI.getOrdering() == AtomicOrdering::AcquireRelease ||
        MOI.getOrdering() == AtomicOrdering::SequentiallyConsistent)
      Changed |= CC->insertCacheInvalidate(MI, MOI.getScope(),
                                           MOI.getOrderingAddrSpace(),
                                           Position::BEFORE);

    return Changed;
  }

  return Changed;
}

bool PPUMemoryLegalizer::expandAtomicCmpxchgOrRmw(const PPUMemOpInfo &MOI,
  MachineBasicBlock::iterator &MI) {
  assert(MI->mayLoad() && MI->mayStore());

  bool Changed = false;

  if (MOI.isAtomic()) {
    if (MOI.getOrdering() == AtomicOrdering::Release ||
        MOI.getOrdering() == AtomicOrdering::AcquireRelease ||
        MOI.getOrdering() == AtomicOrdering::SequentiallyConsistent ||
        MOI.getFailureOrdering() == AtomicOrdering::SequentiallyConsistent)
      Changed |= CC->insertWait(MI, MOI.getScope(),
                                MOI.getOrderingAddrSpace(),
                                PPUMemOp::LOAD | PPUMemOp::STORE,
                                MOI.getIsCrossAddressSpaceOrdering(),
                                Position::BEFORE);

    if (MOI.getOrdering() == AtomicOrdering::Acquire ||
        MOI.getOrdering() == AtomicOrdering::AcquireRelease ||
        MOI.getOrdering() == AtomicOrdering::SequentiallyConsistent ||
        MOI.getFailureOrdering() == AtomicOrdering::Acquire ||
        MOI.getFailureOrdering() == AtomicOrdering::SequentiallyConsistent) {
      Changed |= CC->insertWait(MI, MOI.getScope(),
                                MOI.getOrderingAddrSpace(),
                                isAtomicRet(*MI) ? PPUMemOp::LOAD :
                                                   PPUMemOp::STORE,
                                MOI.getIsCrossAddressSpaceOrdering(),
                                Position::AFTER);
      Changed |= CC->insertCacheInvalidate(MI, MOI.getScope(),
                                           MOI.getOrderingAddrSpace(),
                                           Position::AFTER);
    }

    return Changed;
  }

  return Changed;
}

bool PPUMemoryLegalizer::runOnMachineFunction(MachineFunction &MF) {
  bool Changed = false;

  PPUMemOpAccess MOA(MF);
  CC = SICacheControl::create(MF.getSubtarget<PPUSubtarget>());

  for (auto &MBB : MF) {
    for (auto MI = MBB.begin(); MI != MBB.end(); ++MI) {
      if (!(MI->getDesc().TSFlags & PPUInstrFlags::maybeAtomic))
        continue;

      if (const auto &MOI = MOA.getLoadInfo(MI))
        Changed |= expandLoad(MOI.getValue(), MI);
      else if (const auto &MOI = MOA.getStoreInfo(MI))
        Changed |= expandStore(MOI.getValue(), MI);
      else if (const auto &MOI = MOA.getAtomicFenceInfo(MI))
        Changed |= expandAtomicFence(MOI.getValue(), MI);
      else if (const auto &MOI = MOA.getAtomicCmpxchgOrRmwInfo(MI))
        Changed |= expandAtomicCmpxchgOrRmw(MOI.getValue(), MI);
    }
  }

  Changed |= removeAtomicPseudoMIs();
  return Changed;
}

INITIALIZE_PASS(PPUMemoryLegalizer, DEBUG_TYPE, PASS_NAME, false, false)

char PPUMemoryLegalizer::ID = 0;
char &llvm::PPUMemoryLegalizerID = PPUMemoryLegalizer::ID;

FunctionPass *llvm::createPPUMemoryLegalizerPass() {
  return new PPUMemoryLegalizer();
}
