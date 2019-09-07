; RUN: not llc -mtriple=ppu -mattr=+e < %s 2>&1 | FileCheck %s

; CHECK: LLVM ERROR: Codegen not yet implemented for RV32E

define void @nothing() nounwind {
  ret void
}
