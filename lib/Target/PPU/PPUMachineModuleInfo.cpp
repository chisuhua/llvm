//===--- PPUMachineModuleInfo.cpp ----------------------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
/// \file
/// PPU Machine Module Info.
///
//
//===----------------------------------------------------------------------===//

#include "PPUMachineModuleInfo.h"
#include "llvm/IR/Module.h"

namespace llvm {

PPUMachineModuleInfo::PPUMachineModuleInfo(const MachineModuleInfo &MMI)
    : MachineModuleInfoELF(MMI) {
  LLVMContext &CTX = MMI.getModule()->getContext();
  AgentSSID = CTX.getOrInsertSyncScopeID("agent");
  WorkgroupSSID = CTX.getOrInsertSyncScopeID("workgroup");
  WavefrontSSID = CTX.getOrInsertSyncScopeID("wavefront");
  SystemOneAddressSpaceSSID =
      CTX.getOrInsertSyncScopeID("one-as");
  AgentOneAddressSpaceSSID =
      CTX.getOrInsertSyncScopeID("agent-one-as");
  WorkgroupOneAddressSpaceSSID =
      CTX.getOrInsertSyncScopeID("workgroup-one-as");
  WavefrontOneAddressSpaceSSID =
      CTX.getOrInsertSyncScopeID("wavefront-one-as");
  SingleThreadOneAddressSpaceSSID =
      CTX.getOrInsertSyncScopeID("singlethread-one-as");
}

} // end namespace llvm
