#!/bin/bash
# -*- coding: utf-8 -*-

CPU=riscv32
CPU=PPU
#CPU=AMDGPU

if [ $CPU == "AMDGPU" ]; then
OPT_CPU="-mtriple=amdgcn-unknown-unknown"
cpu="amdgpu"
triple="amdgcn"
fi

#OPT_CPU="-mtriple=ppu-unknown-unknown"
if [ $CPU == "PPU" ]; then
OPT_CPU="-mtriple=ppu"
cpu="ppu"
triple="ppu"
fi

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
#/work/git/chisuhua/projects/sw/mixlang/tools/toolchain/build/llvm/./bin/llvm-lit -sv /work/git/chisuhua/projects/sw/mixlang/tools/toolchain/llvm/test/Transforms/ReconvergeCFG
#exit

ROOT=/work/git/chisuhua/projects/sw/mixlang/tools/toolchain/llvm
BIN=/work/git/chisuhua/projects/sw/mixlang/tools/toolchain/build/llvm/bin

NAME=setvl
NAME=ppuv-copyreg
NAME=ppuv-reg-imm-op
NAME=ppuv-shifted-add
NAME=ppuv-stripmined-binop
NAME=ppuv-vr-gpr-alu-op

NAME=alu32
NAME=analyze-branch
NAME=branch

NAME=ppuv-simple_branch


## vop2
NAME=add
SUBD=vop2

TEST=$ROOT/test/CodeGen/PPU/$SUBD/$NAME.ll

## vop2
NAME=vadd
NAME=sign_extend_1
NAME=sign_extend_2
NAME=s_addk_i32
TEST=$ROOT/test/CodeGen/PPU/$NAME.ll


#DEBUG="--print-machineinstrs --print-after-all --stop-after=ppu-isel"

DEBUG=""

#DEBUG="-debug $DEBUG"
#DEBUG="-debug-pass=Details $DEBUG"
#DEBUG="-debug-pass=Executions $DEBUG"

#NAME=ppu_vlw
#TEST=$NAME.ll

#DEBUG="-time-passes $DEBUG"

DEBUG="-debug-only=isel $DEBUG"
#DEBUG="-debug-only=ppu-isel $DEBUG"
#DEBUG="-debug-only=ppu-lower $DEBUG"
#DEBUG="-debug-only=asm-printer $DEBUG"
#DEBUG="-debug $DEBUG"

#DEBUG="--print-machineinstrs $DEBUG"
DEBUG="--print-after-all $DEBUG"
#DEBUG="--view-isel-dags $DEBUG"

#DEBUG="--stop-after=ppu-isel $DEBUG"
#DEBUG="-stop-after=ppu-lower $DEBUG"
#DEBUG="-stop-after=amdgpu-isel $DEBUG"
#DEBUG="--view-sched-dags $DEBUG"
DEBUG="--verify-machineinstrs $DEBUG"

DEBUG="-o $NAME.s"
#DEBUG="$DEBUG -asm-show-inst"
#DEBUG="$DEBUG -asm-verbose"

#LIT="/work/git/chisuhua/projects/sw/mixlang/tools/toolchain/build/llvm/./bin/llvm-lit -s -vv -a $TEST"
#echo $LIT
#$LIT

#DBG_RUN="$BIN/llc -mtriple=ppu -mattr=+v -mattr=+r $DEBUG $TEST"
DBG_RUN="$BIN/llc -mtriple=$triple -mattr=+t,+r $DEBUG $TEST"
#DBG_RUN="$BIN/llc -mtriple=$triple -mattr=-t $DEBUG $TEST"
echo $DBG_RUN
$DBG_RUN
exit


#######################################################
#####################################################

### ReconvergeCFG
OPT_TYPE="-reconvergecfg"
NAME=if-else-phi
#NAME=loop-exit
#NAME=loop-phi
TEST=$ROOT/test/Transforms/ReconvergeCFG/$NAME.ll



### Scalarize
OPT_TYPE="-Scalarizer -Scalarizer-load-store"
NAME=basic
TEST=$ROOT/test/Transforms/Scalarizer/$NAME.ll



### VscaleProp
OPT_TYPE="-vscaleprop"
NAME=if-else-phi
TEST=$ROOT/test/Transforms/VscaleProp/$NAME.ll


### Divergence
OPT_TYPE="-analyze -divergence"
NAME=loads
NAME=llvm.$triple.buffer.atomic
NAME=llvm.$triple.image.atomic
NAME=no-return-blocks
NAME=phi-undef
NAME=unreachable-loop-block
NAME=intrinsics
NAME=kernel-args
NAME=diverge
#NAME=workitem-intrinsics
TEST=$ROOT/test/Analysis/LegacyDivergenceAnalysis/$CPU/$NAME.ll

OPT_TYPE="$OPT_TYPE -use-gpu-divergence-analysis"
NAME=atomics
NAME=hidden_diverge
NAME=hidden_loopdiverge
NAME=irreducible
NAME=kernel-args
NAME=no-return-blocks
NAME=phi-undef
NAME=temporal_diverge
NAME=always_uniform
TEST=$ROOT/test/Analysis/DivergenceAnalysis/$CPU/$NAME.ll

### VscaleProp
OPT_TYPE="-reconvergecfg"
#OPT_TYPE="-analyze -divergence"
NAME=if-else-phi
#NAME=loop-exit
#NAME=loop-phi
TEST=$ROOT/test/Transforms/ReconvergeCFG/$NAME.ll

NAME=vadd_tid_novolatile
TEST=$ROOT/$NAME.ll

#NAME=uniform
#NAME=nonuniform
#NAME=uniform-regions
#TEST=$ROOT/test/Transforms/ReconvergeCFG/PPU/$NAME.ll


OPT_RUN="$BIN/opt $OPT_CPU $DEBUG -S $OPT_TYPE $TEST -o $NAME.$CPU.s"
echo $OPT_RUN
$OPT_RUN
exit



# End
#####################################################


# gdb command /work/git/chisuhua/projects/sw/mixlang/tools/toolchain/build/llvm/bin/llc -mtriple=ppu -verify-machineinstrs
