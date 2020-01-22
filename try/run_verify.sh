#!/bin/bash
# -*- coding: utf-8 -*-

#cd /work/git/chisuhua/projects/sw/mixlang/tools/toolchain/build/llvm/test && /home/shchi/anaconda2/bin/python /work/git/chisuhua/projects/sw/mixlang/tools/toolchain/build/llvm/./bin/llvm-lit -sv /work/git/chisuhua/projects/sw/mixlang/tools/toolchain/llvm/test/Transforms/SimplifyCFG/PPU
#/work/git/chisuhua/projects/sw/mixlang/tools/toolchain/build/llvm/./bin/llvm-lit -sv /work/git/chisuhua/projects/sw/mixlang/tools/toolchain/llvm/test/Transforms/SimplifyCFG/PPU
#/work/git/chisuhua/projects/sw/mixlang/tools/toolchain/build/llvm/./bin/llvm-lit -sv /work/git/chisuhua/projects/sw/mixlang/tools/toolchain/llvm/test/MC/Disassembler/PPU
#/work/git/chisuhua/projects/sw/mixlang/tools/toolchain/build/llvm/./bin/llvm-lit -sv /work/git/chisuhua/projects/sw/mixlang/tools/toolchain/llvm/test/MC/PPU
#/work/git/chisuhua/projects/sw/mixlang/tools/toolchain/build/llvm/./bin/llvm-lit -sv /work/git/chisuhua/projects/sw/mixlang/tools/toolchain/llvm/test/Transforms/ConstantHoisting/PPU
#/work/git/chisuhua/projects/sw/mixlang/tools/toolchain/build/llvm/./bin/llvm-lit -sv /work/git/chisuhua/projects/sw/mixlang/tools/toolchain/llvm/test/Transforms/AtomicExpand/PPU
#/work/git/chisuhua/projects/sw/mixlang/tools/toolchain/build/llvm/./bin/llvm-lit -sv /work/git/chisuhua/projects/sw/mixlang/tools/toolchain/llvm/test/Analysis/CostModel/PPU
#/work/git/chisuhua/projects/sw/mixlang/tools/toolchain/build/llvm/./bin/llvm-lit -sv /work/git/chisuhua/projects/sw/mixlang/tools/toolchain/llvm/test/Object/PPU
#/work/git/chisuhua/projects/sw/mixlang/tools/toolchain/build/llvm/./bin/llvm-lit -svva /work/git/chisuhua/projects/sw/mixlang/tools/toolchain/llvm/test/CodeGen/PPU
#/work/git/chisuhua/projects/sw/mixlang/tools/toolchain/build/llvm/./bin/llvm-lit -sv /work/git/chisuhua/projects/sw/mixlang/tools/toolchain/llvm/test/DebugInfo/PPU

NAME=setvl
TEST=/work/git/chisuhua/projects/sw/mixlang/tools/toolchain/llvm/test/CodeGen/PPU/$NAME.ll
/work/git/chisuhua/projects/sw/mixlang/tools/toolchain/build/llvm/./bin/llvm-lit -sv $TEST


NAME=testvcopyreg
TEST=/work/git/chisuhua/projects/sw/mixlang/tools/toolchain/llvm/test/CodeGen/PPU/$NAME.ll
/work/git/chisuhua/projects/sw/mixlang/tools/toolchain/build/llvm/./bin/llvm-lit -sv $TEST

NAME=ppuv-reg-imm-op
TEST=/work/git/chisuhua/projects/sw/mixlang/tools/toolchain/llvm/test/CodeGen/PPU/$NAME.ll
/work/git/chisuhua/projects/sw/mixlang/tools/toolchain/build/llvm/./bin/llvm-lit -sv $TEST
exit

NAME=ppuv-shifted-add
TEST=/work/git/chisuhua/projects/sw/mixlang/tools/toolchain/llvm/test/CodeGen/PPU/$NAME.ll
/work/git/chisuhua/projects/sw/mixlang/tools/toolchain/build/llvm/./bin/llvm-lit -sv $TEST

NAME=ppuv-stripmined-binop
TEST=/work/git/chisuhua/projects/sw/mixlang/tools/toolchain/llvm/test/CodeGen/PPU/$NAME.ll
/work/git/chisuhua/projects/sw/mixlang/tools/toolchain/build/llvm/./bin/llvm-lit -sv $TEST

NAME=ppuv-vr-gpr-alu-op
TEST=/work/git/chisuhua/projects/sw/mixlang/tools/toolchain/llvm/test/CodeGen/PPU/$NAME.ll
/work/git/chisuhua/projects/sw/mixlang/tools/toolchain/build/llvm/./bin/llvm-lit -sv $TEST




