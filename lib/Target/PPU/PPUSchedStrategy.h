//===-- PPUSchedStrategy.h - GCN Scheduler Strategy -*- C++ -*-------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
///
//===----------------------------------------------------------------------===//
//
/// \file
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_PPU_SCHEDSTRATEGY_H
#define LLVM_LIB_TARGET_PPU_SCHEDSTRATEGY_H

#include "PPURegPressure.h"
#include "llvm/CodeGen/MachineScheduler.h"

namespace llvm {

class PPUMachineFunctionInfo;
class PPURegisterInfo;
class PPUSubtarget;

/// This is a minimal scheduler strategy.  The main difference between this
/// and the GenericScheduler is that PPUSchedStrategy uses different
/// heuristics to determine excess/critical pressure sets.  Its goal is to
/// maximize kernel occupancy (i.e. maximum number of waves per simd).
class PPUMaxOccupancySchedStrategy : public GenericScheduler {
  friend class PPUScheduleDAGMILive;

  SUnit *pickNodeBidirectional(bool &IsTopNode);

  void pickNodeFromQueue(SchedBoundary &Zone, const CandPolicy &ZonePolicy,
                         const RegPressureTracker &RPTracker,
                         SchedCandidate &Cand);

  void initCandidate(SchedCandidate &Cand, SUnit *SU,
                     bool AtTop, const RegPressureTracker &RPTracker,
                     const PPURegisterInfo *SRI,
                     unsigned SGPRPressure, unsigned VGPRPressure);

  unsigned SGPRExcessLimit;
  unsigned VGPRExcessLimit;
  unsigned SGPRCriticalLimit;
  unsigned VGPRCriticalLimit;

  unsigned TargetOccupancy;

  MachineFunction *MF;

public:
  PPUMaxOccupancySchedStrategy(const MachineSchedContext *C);

  SUnit *pickNode(bool &IsTopNode) override;

  void initialize(ScheduleDAGMI *DAG) override;

  void setTargetOccupancy(unsigned Occ) { TargetOccupancy = Occ; }
};

class PPUScheduleDAGMILive : public ScheduleDAGMILive {

  const PPUSubtarget &ST;

  PPUMachineFunctionInfo &MFI;

  // Occupancy target at the beginning of function scheduling cycle.
  unsigned StartingOccupancy;

  // Minimal real occupancy recorder for the function.
  unsigned MinOccupancy;

  // Scheduling stage number.
  unsigned Stage;

  // Current region index.
  size_t RegionIdx;

  // Vecor of regions recorder for later rescheduling
  SmallVector<
      std::pair<MachineBasicBlock::iterator, MachineBasicBlock::iterator>, 32>
      Regions;

  // Region live-in cache.
  SmallVector<PPURPTracker::LiveRegSet, 32> LiveIns;

  // Region pressure cache.
  SmallVector<PPURegPressure, 32> Pressure;

  // Temporary basic block live-in cache.
  DenseMap<const MachineBasicBlock *, PPURPTracker::LiveRegSet> MBBLiveIns;

  DenseMap<MachineInstr *, PPURPTracker::LiveRegSet> BBLiveInMap;
  DenseMap<MachineInstr *, PPURPTracker::LiveRegSet> getBBLiveInMap() const;

  // Return current region pressure.
  PPURegPressure getRealRegPressure() const;

  // Compute and cache live-ins and pressure for all regions in block.
  void computeBlockPressure(const MachineBasicBlock *MBB);

public:
  PPUScheduleDAGMILive(MachineSchedContext *C,
                      std::unique_ptr<MachineSchedStrategy> S);

  void schedule() override;

  void finalizeSchedule() override;
};

} // End namespace llvm

#endif // PPUSCHEDSTRATEGY_H
