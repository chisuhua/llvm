//===----------------------------------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "PPU.h"
#include "PPUArgumentUsageInfo.h"
#include "PPURegisterInfo.h"
#include "llvm/Support/NativeFormatting.h"
#include "llvm/Support/raw_ostream.h"

using namespace llvm;

#define DEBUG_TYPE "ppu-argument-reg-usage-info"

INITIALIZE_PASS(PPUArgumentUsageInfo, DEBUG_TYPE,
                "Argument Register Usage Information Storage", false, true)
/*
void ArgDescriptor::print(raw_ostream &OS,
                          const TargetRegisterInfo *TRI) const {
  if (!isSet()) {
    OS << "<not set>\n";
    return;
  }

  if (isRegister())
    OS << "Reg " << printReg(getRegister(), TRI);
  else
    OS << "Stack offset " << getStackOffset();

  if (isMasked()) {
    OS << " & ";
    llvm::write_hex(OS, Mask, llvm::HexPrintStyle::PrefixLower);
  }

  OS << '\n';
}
*/

char PPUArgumentUsageInfo::ID = 0;

const PPUFunctionArgInfo PPUArgumentUsageInfo::ExternFunctionInfo{};

bool PPUArgumentUsageInfo::doInitialization(Module &M) {
  return false;
}

bool PPUArgumentUsageInfo::doFinalization(Module &M) {
  ArgInfoMap.clear();
  return false;
}

void PPUArgumentUsageInfo::print(raw_ostream &OS, const Module *M) const {
  for (const auto &FI : ArgInfoMap) {
    OS << "Arguments for " << FI.first->getName() << '\n'
       << "  PrivateSegmentBuffer: " << FI.second.PrivateSegmentBuffer
       << "  DispatchPtr: " << FI.second.DispatchPtr
       << "  QueuePtr: " << FI.second.QueuePtr
       << "  KernargSegmentPtr: " << FI.second.KernargSegmentPtr
       << "  DispatchID: " << FI.second.DispatchID
       << "  FlatScratchInit: " << FI.second.FlatScratchInit
       << "  PrivateSegmentSize: " << FI.second.PrivateSegmentSize
       << "  WorkGroupIDX: " << FI.second.WorkGroupIDX
       << "  WorkGroupIDY: " << FI.second.WorkGroupIDY
       << "  WorkGroupIDZ: " << FI.second.WorkGroupIDZ
       << "  WorkGroupInfo: " << FI.second.WorkGroupInfo
       << "  PrivateSegmentWaveByteOffset: "
          << FI.second.PrivateSegmentWaveByteOffset
       << "  ImplicitBufferPtr: " << FI.second.ImplicitBufferPtr
       << "  ImplicitArgPtr: " << FI.second.ImplicitArgPtr
       << "  WorkItemIDX " << FI.second.WorkItemIDX
       << "  WorkItemIDY " << FI.second.WorkItemIDY
       << "  WorkItemIDZ " << FI.second.WorkItemIDZ
       << '\n';
  }
}

std::pair<const ArgDescriptor *, const TargetRegisterClass *>
PPUFunctionArgInfo::getPreloadedValue(
  PPUFunctionArgInfo::PreloadedValue Value) const {
  switch (Value) {
      /*
  case PPUFunctionArgInfo::PRIVATE_SEGMENT_BUFFER: {
    return std::make_pair(
      PrivateSegmentBuffer ? &PrivateSegmentBuffer : nullptr,
      &PPU::SGPR_128RegClass);
  }
  case PPUFunctionArgInfo::IMPLICIT_BUFFER_PTR:
    return std::make_pair(ImplicitBufferPtr ? &ImplicitBufferPtr : nullptr,
                          &PPU::GPRRegClass);
                          */
  case PPUFunctionArgInfo::WORKGROUP_ID_X:
    return std::make_pair(WorkGroupIDX ? &WorkGroupIDX : nullptr,
                          &PPU::SPR_32RegClass);
  case PPUFunctionArgInfo::WORKGROUP_ID_Y:
    return std::make_pair(WorkGroupIDY ? &WorkGroupIDY : nullptr,
                          &PPU::SPR_32RegClass);
  case PPUFunctionArgInfo::WORKGROUP_ID_Z:
    return std::make_pair(WorkGroupIDZ ? &WorkGroupIDZ : nullptr,
                          &PPU::SPR_32RegClass);
  case PPUFunctionArgInfo::PRIVATE_SEGMENT_WAVE_BYTE_OFFSET:
    return std::make_pair(
      PrivateSegmentWaveByteOffset ? &PrivateSegmentWaveByteOffset : nullptr,
      &PPU::SPR_32RegClass);
  case PPUFunctionArgInfo::KERNARG_SEGMENT_PTR:
    return std::make_pair(KernargSegmentPtr ? &KernargSegmentPtr : nullptr,
                          &PPU::SPR_64RegClass);
    /*
  case PPUFunctionArgInfo::IMPLICIT_ARG_PTR:
    return std::make_pair(ImplicitArgPtr ? &ImplicitArgPtr : nullptr,
                          &PPU::SGPR_64RegClass);
                          */
  case PPUFunctionArgInfo::DISPATCH_ID:
    return std::make_pair(DispatchID ? &DispatchID : nullptr,
                          &PPU::SPR_64RegClass);
    /*
  case PPUFunctionArgInfo::FLAT_SCRATCH_INIT:
    return std::make_pair(FlatScratchInit ? &FlatScratchInit : nullptr,
                          &PPU::SGPR_64RegClass);
                          */
  case PPUFunctionArgInfo::DISPATCH_PTR:
    return std::make_pair(DispatchPtr ? &DispatchPtr : nullptr,
                          &PPU::SPR_64RegClass);
  case PPUFunctionArgInfo::QUEUE_PTR:
    return std::make_pair(QueuePtr ? &QueuePtr : nullptr,
                          &PPU::SPR_64RegClass);
  case PPUFunctionArgInfo::WORKITEM_ID_X:
    return std::make_pair(WorkItemIDX ? &WorkItemIDX : nullptr,
                          &PPU::VPR_32RegClass);
  case PPUFunctionArgInfo::WORKITEM_ID_Y:
    return std::make_pair(WorkItemIDY ? &WorkItemIDY : nullptr,
                          &PPU::VPR_32RegClass);
  case PPUFunctionArgInfo::WORKITEM_ID_Z:
    return std::make_pair(WorkItemIDZ ? &WorkItemIDZ : nullptr,
                          &PPU::VPR_32RegClass);
  }
  llvm_unreachable("unexpected preloaded value type");
}
