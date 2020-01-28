//===-- VOPInstructions.td - Vector Instruction Defintions ----------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

// dummies for outer let
class LetDummies {
  bit isCommutable;
  bit isConvertibleToThreeAddress;
  bit isMoveImm;
  bit isReMaterializable;
  bit isAsCheapAsAMove;
  bit VOPAsmPrefer32Bit;
  Predicate SubtargetPredicate;
  string Constraints;
  string DisableEncoding;
  list<SchedReadWrite> SchedRW;
  list<Register> Uses;
  list<Register> Defs;
}

class VOP <string opName> {
  string OpName = opName;
}

class VOPAnyCommon <dag outs, dag ins, string asm, list<dag> pattern> :
    PPTInst <outs, ins, asm, "", pattern> {

  let mayLoad = 0;
  let mayStore = 0;
  let hasSideEffects = 0;
  let UseNamedOperandTable = 1;
  let VALU = 1;
  let Uses = [TMSK];
}

class VOP_Pseudo <string opName, string suffix, VOPProfile P, dag outs, dag ins,
                  string asmOps, list<dag> pattern> :
  PPTInst <outs, ins, opName#suffix, asmOps, pattern>,
  VOP <opName>,
  PPUMCInstr <opName#suffix, PPUEncodingFamily.NONE>,
  MnemonicAlias<opName#suffix, opName> {

  let isPseudo = 1;
  let isCodeGenOnly = 1;
  let UseNamedOperandTable = 1;

  string Mnemonic = opName;
  VOPProfile Pfl = P;

  string AsmOperands;
}

class VOP3Common <dag outs, dag ins, string asm = "",
                  list<dag> pattern = [], bit HasMods = 0,
                  bit VOP3Only = 0> :
  VOPAnyCommon <outs, ins, asm, pattern> {

  // Using complex patterns gives VOP3 patterns a very high complexity rating,
  // but standalone patterns are almost always preferred, so we need to adjust the
  // priority lower.  The goal is to use a high number to reduce complexity to
  // zero (or less than zero).
  let AddedComplexity = -1000;

  let VOP3 = 1;

  let AsmVariantName = PPUAsmVariants.VOP3;
  let AsmMatchConverter = !if(!eq(HasMods,1), "cvtVOP3", "");

  let isCodeGenOnly = 0;

  int Size = 8;

  // Because SGPRs may be allowed if there are multiple operands, we
  // need a post-isel hook to insert copies in order to avoid
  // violating constant bus requirements.
  let hasPostISelHook = 1;
}

class VOP3_Pseudo <string opName, VOPProfile P, list<dag> pattern = [],
                   bit VOP3Only = 0, bit isVOP3P = 0, bit isVop3OpSel = 0> :
  VOP_Pseudo <opName, "_e64", P, P.Outs64,
              !if(isVop3OpSel,
                  P.InsVOP3OpSel,
                  !if(!and(isVOP3P, P.IsPacked), P.InsVOP3P, P.Ins64)),
              !if(isVop3OpSel,
                  P.AsmVOP3OpSel,
                  !if(!and(isVOP3P, P.IsPacked), P.AsmVOP3P, P.Asm64)),
              pattern> {

  // TODO let VOP3_OPSEL = isVop3OpSel;
  let IsPacked = P.IsPacked;
  let IsMAI = P.IsMAI;

  let AsmOperands = !if(isVop3OpSel,
                        P.AsmVOP3OpSel,
                        !if(!and(isVOP3P, P.IsPacked), P.AsmVOP3P, P.Asm64));

  let Size = 8;
  let mayLoad = 0;
  let mayStore = 0;
  let hasSideEffects = 0;

  // Because SGPRs may be allowed if there are multiple operands, we
  // need a post-isel hook to insert copies in order to avoid
  // violating constant bus requirements.
  let hasPostISelHook = 1;

  // Using complex patterns gives VOP3 patterns a very high complexity rating,
  // but standalone patterns are almost always preferred, so we need to adjust the
  // priority lower.  The goal is to use a high number to reduce complexity to
  // zero (or less than zero).
  let AddedComplexity = -1000;

  let VOP3 = 1;
  let VALU = 1;
  let FPClamp = P.HasFPClamp;
  let IntClamp = P.HasIntClamp;
  let ClampLo = P.HasClampLo;
  let ClampHi = P.HasClampHi;

  let Uses = [TMSK];

  let AsmVariantName = PPUAsmVariants.VOP3;
  let AsmMatchConverter =
    !if(isVOP3P,
        "cvtVOP3P",
        !if(!or(P.HasModifiers, !or(P.HasOMod, P.HasIntClamp)),
            "cvtVOP3",
            ""));
}

class VOP3P_Pseudo <string opName, VOPProfile P, list<dag> pattern = []> :
  VOP3_Pseudo<opName, P, pattern, 1, 1> {
  let VOP3P = 1;
}

class VOP3_Real <VOP_Pseudo ps, int EncodingFamily> :
  PPTInst <ps.OutOperandList, ps.InOperandList, ps.Mnemonic , ps.AsmOperands, []>,
  PPUMCInstr <ps.PseudoInstr, EncodingFamily> {

  let isPseudo = 0;
  let isCodeGenOnly = 0;
  let UseNamedOperandTable = 1;

  let Constraints     = ps.Constraints;
  let DisableEncoding = ps.DisableEncoding;

  // copy relevant pseudo op flags
  let SubtargetPredicate = ps.SubtargetPredicate;
  let AsmMatchConverter  = ps.AsmMatchConverter;
  let AsmVariantName     = ps.AsmVariantName;
  let Constraints        = ps.Constraints;
  let DisableEncoding    = ps.DisableEncoding;
  let TSFlags            = ps.TSFlags;
  let UseNamedOperandTable = ps.UseNamedOperandTable;
  let Uses                 = ps.Uses;
  let Defs                 = ps.Defs;

  VOPProfile Pfl = ps.Pfl;
}

// XXX - Is there any reason to distingusih this from regular VOP3
// here?
class VOP3P_Real<VOP_Pseudo ps, int EncodingFamily> :
  VOP3_Real<ps, EncodingFamily>;

class VOP3a<VOPProfile P> : Enc64 {
  bits<4> src0_modifiers;
  bits<9> src0;
  bits<3> src1_modifiers;
  bits<9> src1;
  bits<3> src2_modifiers;
  bits<9> src2;
  bits<1> clamp;
  bits<2> omod;

  let Inst{8}     = !if(P.HasSrc0Mods, src0_modifiers{1}, 0);
  let Inst{9}     = !if(P.HasSrc1Mods, src1_modifiers{1}, 0);
  let Inst{10}    = !if(P.HasSrc2Mods, src2_modifiers{1}, 0);

  let Inst{31-26} = 0x34; //encoding
  let Inst{40-32} = !if(P.HasSrc0, src0, 0);
  let Inst{49-41} = !if(P.HasSrc1, src1, 0);
  let Inst{58-50} = !if(P.HasSrc2, src2, 0);
  let Inst{60-59} = !if(P.HasOMod, omod, 0);
  let Inst{61}    = !if(P.HasSrc0Mods, src0_modifiers{0}, 0);
  let Inst{62}    = !if(P.HasSrc1Mods, src1_modifiers{0}, 0);
  let Inst{63}    = !if(P.HasSrc2Mods, src2_modifiers{0}, 0);
}


/*
class VOP3a_gfx10<bits<10> op, VOPProfile p> : VOP3a<p> {
  let Inst{15}    = !if(p.HasClamp, clamp{0}, 0);
  let Inst{25-16} = op;
  let Inst{31-26} = 0x35;
}
*/

class VOP3a_ppu <bits<10> op, VOPProfile P> : VOP3a<P> {
  let Inst{25-16} = op;
  let Inst{15}    = !if(P.HasClamp, clamp{0}, 0);
}

class VOP3e_ppu <bits<10> op, VOPProfile P> : VOP3a_ppu <op, P> {
  bits<8> vdst;
  let Inst{7-0} = !if(P.EmitDst, vdst{7-0}, 0);
}

class VOP3OpSel_gfx10<bits<10> op, VOPProfile p> : VOP3e_ppu<op, p> {
  let Inst{11} = !if(p.HasSrc0, src0_modifiers{2}, 0);
  let Inst{12} = !if(p.HasSrc1, src1_modifiers{2}, 0);
  let Inst{13} = !if(p.HasSrc2, src2_modifiers{2}, 0);
  let Inst{14} = !if(p.HasDst,  src0_modifiers{3}, 0);
}

class VOP3be <VOPProfile P> : Enc64 {
  bits<8> vdst;
  bits<2> src0_modifiers;
  bits<9> src0;
  bits<2> src1_modifiers;
  bits<9> src1;
  bits<2> src2_modifiers;
  bits<9> src2;
  bits<7> sdst;
  bits<2> omod;

  let Inst{7-0}   = vdst;
  let Inst{14-8}  = sdst;
  let Inst{31-26} = 0x34; //encoding
  let Inst{40-32} = !if(P.HasSrc0, src0, 0);
  let Inst{49-41} = !if(P.HasSrc1, src1, 0);
  let Inst{58-50} = !if(P.HasSrc2, src2, 0);
  let Inst{60-59} = !if(P.HasOMod, omod, 0);
  let Inst{61}    = !if(P.HasSrc0Mods, src0_modifiers{0}, 0);
  let Inst{62}    = !if(P.HasSrc1Mods, src1_modifiers{0}, 0);
  let Inst{63}    = !if(P.HasSrc2Mods, src2_modifiers{0}, 0);
}

class VOP3Pe <bits<10> op, VOPProfile P> : Enc64 {
  bits<8> vdst;
  // neg, neg_hi, op_sel put in srcN_modifiers
  bits<4> src0_modifiers;
  bits<9> src0;
  bits<4> src1_modifiers;
  bits<9> src1;
  bits<4> src2_modifiers;
  bits<9> src2;
  bits<1> clamp;

  let Inst{7-0} = vdst;
  let Inst{8} = !if(P.HasSrc0Mods, src0_modifiers{1}, 0); // neg_hi src0
  let Inst{9} = !if(P.HasSrc1Mods, src1_modifiers{1}, 0); // neg_hi src1
  let Inst{10} = !if(P.HasSrc2Mods, src2_modifiers{1}, 0); // neg_hi src2

  let Inst{11} = !if(!and(P.HasSrc0, P.HasOpSel), src0_modifiers{2}, 0); // op_sel(0)
  let Inst{12} = !if(!and(P.HasSrc1, P.HasOpSel), src1_modifiers{2}, 0); // op_sel(1)
  let Inst{13} = !if(!and(P.HasSrc2, P.HasOpSel), src2_modifiers{2}, 0); // op_sel(2)

  let Inst{14} = !if(!and(P.HasSrc2, P.HasOpSel), src2_modifiers{3}, 0); // op_sel_hi(2)

  let Inst{15} = !if(P.HasClamp, clamp{0}, 0);

  let Inst{25-16} = op;
  let Inst{31-26} = 0x34; //encoding
  let Inst{40-32} = !if(P.HasSrc0, src0, 0);
  let Inst{49-41} = !if(P.HasSrc1, src1, 0);
  let Inst{58-50} = !if(P.HasSrc2, src2, 0);
  let Inst{59}    = !if(!and(P.HasSrc0, P.HasOpSel), src0_modifiers{3}, 0); // op_sel_hi(0)
  let Inst{60}    = !if(!and(P.HasSrc1, P.HasOpSel), src1_modifiers{3}, 0); // op_sel_hi(1)
  let Inst{61}    = !if(P.HasSrc0Mods, src0_modifiers{0}, 0); // neg (lo)
  let Inst{62}    = !if(P.HasSrc1Mods, src1_modifiers{0}, 0); // neg (lo)
  let Inst{63}    = !if(P.HasSrc2Mods, src2_modifiers{0}, 0); // neg (lo)
}

class VOP3Pe_ppu <bits<10> op, VOPProfile P> : VOP3Pe<op, P> {
  let Inst{31-26} = 0x33; //encoding
}

class VOP3be_ppu<bits<10> op, VOPProfile p> : VOP3be<p> {
  bits<1> clamp;
  let Inst{15}    = !if(p.HasClamp, clamp{0}, 0);
  let Inst{25-16} = op;
  let Inst{31-26} = 0x35;
}


class getNumNodeArgs<SDPatternOperator Op> {
  SDNode N = !cast<SDNode>(Op);
  SDTypeProfile TP = N.TypeProfile;
  int ret = TP.NumOperands;
}


class getDivergentFrag<SDPatternOperator Op> {

  int NumSrcArgs = getNumNodeArgs<Op>.ret;
  PatFrag ret = PatFrag <
    !if(!eq(NumSrcArgs, 1),
             (ops node:$src0),
             !if(!eq(NumSrcArgs, 2),
               (ops node:$src0, node:$src1),
               (ops node:$src0, node:$src1, node:$src2))),
    !if(!eq(NumSrcArgs, 1),
             (Op $src0),
             !if(!eq(NumSrcArgs, 2),
               (Op $src0, $src1),
               (Op $src0, $src1, $src2))),
    [{ return N->isDivergent(); }]
  >;
}

class VOPPatGen<SDPatternOperator Op, VOPProfile P> {

  PatFrag Operator = getDivergentFrag < Op >.ret;

  dag Ins = !foreach(tmp, P.Ins32, !subst(ins, Operator,
                                         !subst(P.Src0RC32, P.Src0VT,
                                               !subst(P.Src1RC32, P.Src1VT, tmp))));


  dag Outs = !foreach(tmp, P.Outs32, !subst(outs, set,
                                           !subst(P.DstRC, P.DstVT, tmp)));

  list<dag> ret =  [!con(Outs, (set Ins))];
}

class VOPPatOrNull<SDPatternOperator Op, VOPProfile P> {
  list<dag> ret = !if(!ne(P.NeedPatGen,PatGenMode.NoPattern), VOPPatGen<Op, P>.ret, []);
}

class DivergentFragOrOp<SDPatternOperator Op, VOPProfile P> {
  SDPatternOperator ret = !if(!eq(P.NeedPatGen,PatGenMode.Pattern),
   !if(!isa<SDNode>(Op), getDivergentFrag<Op>.ret, Op), Op);
}

