//===-- SIInstrFormats.td - SI Instruction Encodings ----------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// SI Instruction format definitions.
//
//===----------------------------------------------------------------------===//
/*
class PseudoInst<dag outs, dag ins, list<dag> pattern = [], string asm = "">
  : PPTPseudo<outs, ins, pattern, asm, ""> {
  let isPseudo = 1;
  let isCodeGenOnly = 1;
}
*/

class SPseudoInst<dag outs, dag ins, list<dag> pattern = [], string opcodestr = "", string argstr = "">
  : PPTPseudo<outs, ins, pattern, opcodestr, argstr> {
  let SALU = 1;
}

class VPseudoInst<dag outs, dag ins, list<dag> pattern = [], string opcodestr = "", string argstr = "">
  : PPTPseudo<outs, ins, pattern, opcodestr, argstr> {
  let VALU = 1;
  let Uses = [TMSK];

  // FIXME schi why need this
  // let mayLoad = 0;
  // let mayStore = 0;
  // let hasSideEffects = 0;
}

class CFPseudoInst<dag outs, dag ins, list<dag> pattern = [],
  bit UseTmsk = 0, bit DefTmsk = 0> :
  SPseudoInst<outs, ins, pattern> {

  let Uses = !if(UseTmsk, [TMSK], []);
  let Defs = !if(DefTmsk, [TMSK, SCC], [SCC]);
  let mayLoad = 0;
  let mayStore = 0;
  let hasSideEffects = 0;
}

// MT
/*
class Enc32 {
  field bits<32> Inst;
  int Size = 4;
}

class Enc64 {
  field bits<64> Inst;
  int Size = 8;
}
*/


class EXPe : Enc64 {
  bits<4> en;
  bits<6> tgt;
  bits<1> compr;
  bits<1> done;
  bits<1> vm;
  bits<8> src0;
  bits<8> src1;
  bits<8> src2;
  bits<8> src3;

  let Inst{3-0} = en;
  let Inst{9-4} = tgt;
  let Inst{10} = compr;
  let Inst{11} = done;
  let Inst{12} = vm;
  let Inst{31-26} = 0x3e;
  let Inst{39-32} = src0;
  let Inst{47-40} = src1;
  let Inst{55-48} = src2;
  let Inst{63-56} = src3;
}

// class VOPDstOperand <RegisterClass rc> : RegisterOperand <rc, "printVOPDst">;
/*
let Uses = [TMSK] in {

class EXPCommon<dag outs, dag ins, string asm, list<dag> pattern> :
  PPUInst<outs, ins, asm, "", pattern> {
  let EXP = 1;
  let EXP_CNT = 1;
  let mayLoad = 0; // Set to 1 if done bit is set.
  let mayStore = 1;
  let UseNamedOperandTable = 1;
  let Uses = [TMSK];
  let SchedRW = [WriteExport];
}

} // End Uses = [TMSK]
*/
