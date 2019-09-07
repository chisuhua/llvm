// REQUIRES: ppu-registered-target
// RUN: not llvm-mc -triple ppu-unknown-linux-gnu < %s 2>&1 | FileCheck %s
"" # CHECK: error: unrecognized instruction mnemonic
