//===-- PPUTargetInfo.h - PPU Target Implementation ---------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_PPU_TARGETINFO_PPUTARGETINFO_H
#define LLVM_LIB_TARGET_PPU_TARGETINFO_PPUTARGETINFO_H

namespace llvm {

class Target;

Target &getThePPUTarget();

} // namespace llvm

#endif // LLVM_LIB_TARGET_PPU_TARGETINFO_PPUTARGETINFO_H
