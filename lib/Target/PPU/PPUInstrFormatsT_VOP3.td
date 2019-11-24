//===----------------------------------------------------------------------===//
// VOP3 Classes
//===----------------------------------------------------------------------===//

class getVOP3ModPat<VOPProfile P, SDPatternOperator node> {
  dag src0 = !if(P.HasOMod,
    (VOP3Mods0 P.Src0VT:$src0, i32:$src0_modifiers, i1:$clamp, i32:$omod),
    (VOP3Mods0 P.Src0VT:$src0, i32:$src0_modifiers, i1:$clamp));

  list<dag> ret3 = [(set P.DstVT:$vdst,
    (DivergentFragOrOp<node, P>.ret (P.Src0VT src0),
          (P.Src1VT (VOP3Mods P.Src1VT:$src1, i32:$src1_modifiers)),
          (P.Src2VT (VOP3Mods P.Src2VT:$src2, i32:$src2_modifiers))))];

  list<dag> ret2 = [(set P.DstVT:$vdst,
    (DivergentFragOrOp<node, P>.ret (P.Src0VT src0),
          (P.Src1VT (VOP3Mods P.Src1VT:$src1, i32:$src1_modifiers))))];

  list<dag> ret1 = [(set P.DstVT:$vdst,
    (DivergentFragOrOp<node, P>.ret (P.Src0VT src0)))];

  list<dag> ret = !if(!eq(P.NumSrcArgs, 3), ret3,
                  !if(!eq(P.NumSrcArgs, 2), ret2,
                  ret1));
}

class getVOP3PModPat<VOPProfile P, SDPatternOperator node> {
  list<dag> ret3 = [(set P.DstVT:$vdst,
    (DivergentFragOrOp<node, P>.ret (P.Src0VT !if(P.HasClamp, (VOP3PMods0 P.Src0VT:$src0, i32:$src0_modifiers, i1:$clamp),
                                    (VOP3PMods P.Src0VT:$src0, i32:$src0_modifiers))),
          (P.Src1VT (VOP3PMods P.Src1VT:$src1, i32:$src1_modifiers)),
          (P.Src2VT (VOP3PMods P.Src2VT:$src2, i32:$src2_modifiers))))];

  list<dag> ret2 = [(set P.DstVT:$vdst,
    (DivergentFragOrOp<node, P>.ret !if(P.HasClamp, (P.Src0VT (VOP3PMods0 P.Src0VT:$src0, i32:$src0_modifiers, i1:$clamp)),
                          (P.Src0VT (VOP3PMods P.Src0VT:$src0, i32:$src0_modifiers))),
          (P.Src1VT (VOP3PMods P.Src1VT:$src1, i32:$src1_modifiers))))];

  list<dag> ret1 = [(set P.DstVT:$vdst,
    (DivergentFragOrOp<node, P>.ret (P.Src0VT (VOP3PMods0 P.Src0VT:$src0, i32:$src0_modifiers, i1:$clamp))))];

  list<dag> ret = !if(!eq(P.NumSrcArgs, 3), ret3,
                  !if(!eq(P.NumSrcArgs, 2), ret2,
                  ret1));
}

class getVOP3OpSelPat<VOPProfile P, SDPatternOperator node> {
  list<dag> ret3 = [(set P.DstVT:$vdst,
    (DivergentFragOrOp<node, P>.ret (P.Src0VT !if(P.HasClamp, (VOP3OpSel0 P.Src0VT:$src0, i32:$src0_modifiers, i1:$clamp),
                                    (VOP3OpSel P.Src0VT:$src0, i32:$src0_modifiers))),
          (P.Src1VT (VOP3OpSel P.Src1VT:$src1, i32:$src1_modifiers)),
          (P.Src2VT (VOP3OpSel P.Src2VT:$src2, i32:$src2_modifiers))))];

  list<dag> ret2 = [(set P.DstVT:$vdst,
    (DivergentFragOrOp<node, P>.ret !if(P.HasClamp, (P.Src0VT (VOP3OpSel0 P.Src0VT:$src0, i32:$src0_modifiers, i1:$clamp)),
                          (P.Src0VT (VOP3OpSel P.Src0VT:$src0, i32:$src0_modifiers))),
          (P.Src1VT (VOP3OpSel P.Src1VT:$src1, i32:$src1_modifiers))))];

  list<dag> ret1 = [(set P.DstVT:$vdst,
    (DivergentFragOrOp<node, P>.ret (P.Src0VT (VOP3OpSel0 P.Src0VT:$src0, i32:$src0_modifiers, i1:$clamp))))];

  list<dag> ret = !if(!eq(P.NumSrcArgs, 3), ret3,
                  !if(!eq(P.NumSrcArgs, 2), ret2,
                  ret1));
}

class getVOP3OpSelModPat<VOPProfile P, SDPatternOperator node> {
  list<dag> ret3 = [(set P.DstVT:$vdst,
    (DivergentFragOrOp<node, P>.ret (P.Src0VT !if(P.HasClamp, (VOP3OpSelMods0 P.Src0VT:$src0, i32:$src0_modifiers, i1:$clamp),
                                    (VOP3OpSelMods P.Src0VT:$src0, i32:$src0_modifiers))),
          (P.Src1VT (VOP3OpSelMods P.Src1VT:$src1, i32:$src1_modifiers)),
          (P.Src2VT (VOP3OpSelMods P.Src2VT:$src2, i32:$src2_modifiers))))];

  list<dag> ret2 = [(set P.DstVT:$vdst,
    (DivergentFragOrOp<node, P>.ret !if(P.HasClamp, (P.Src0VT (VOP3OpSelMods0 P.Src0VT:$src0, i32:$src0_modifiers, i1:$clamp)),
                          (P.Src0VT (VOP3OpSelMods P.Src0VT:$src0, i32:$src0_modifiers))),
          (P.Src1VT (VOP3OpSelMods P.Src1VT:$src1, i32:$src1_modifiers))))];

  list<dag> ret1 = [(set P.DstVT:$vdst,
    (DivergentFragOrOp<node, P>.ret (P.Src0VT (VOP3OpSelMods0 P.Src0VT:$src0, i32:$src0_modifiers, i1:$clamp))))];

  list<dag> ret = !if(!eq(P.NumSrcArgs, 3), ret3,
                  !if(!eq(P.NumSrcArgs, 2), ret2,
                  ret1));
}

class getVOP3Pat<VOPProfile P, SDPatternOperator node> {
  list<dag> ret3 = [(set P.DstVT:$vdst, (DivergentFragOrOp<node, P>.ret P.Src0VT:$src0, P.Src1VT:$src1, P.Src2VT:$src2))];
  list<dag> ret2 = [(set P.DstVT:$vdst, (DivergentFragOrOp<node, P>.ret P.Src0VT:$src0, P.Src1VT:$src1))];
  list<dag> ret1 = [(set P.DstVT:$vdst, (DivergentFragOrOp<node, P>.ret P.Src0VT:$src0))];
  list<dag> ret = !if(!eq(P.NumSrcArgs, 3), ret3,
                  !if(!eq(P.NumSrcArgs, 2), ret2,
                  ret1));
}

class getVOP3ClampPat<VOPProfile P, SDPatternOperator node> {
  list<dag> ret3 = [(set P.DstVT:$vdst, (node P.Src0VT:$src0, P.Src1VT:$src1, P.Src2VT:$src2, i1:$clamp))];
  list<dag> ret2 = [(set P.DstVT:$vdst, (node P.Src0VT:$src0, P.Src1VT:$src1, i1:$clamp))];
  list<dag> ret1 = [(set P.DstVT:$vdst, (node P.Src0VT:$src0, i1:$clamp))];
  list<dag> ret = !if(!eq(P.NumSrcArgs, 3), ret3,
                  !if(!eq(P.NumSrcArgs, 2), ret2,
                  ret1));
}

class getVOP3MAIPat<VOPProfile P, SDPatternOperator node> {
  list<dag> ret = [(set P.DstVT:$vdst, (node P.Src0VT:$src0, P.Src1VT:$src1, P.Src2VT:$src2,
                                        imm:$cbsz, imm:$abid, imm:$blgp))];
}

class VOP3Inst<string OpName, VOPProfile P, SDPatternOperator node = null_frag, bit VOP3Only = 0> :
  VOP3_Pseudo<OpName, P,
    !if(P.HasOpSel,
        !if(P.HasModifiers,
            getVOP3OpSelModPat<P, node>.ret,
            getVOP3OpSelPat<P, node>.ret),
        !if(P.HasModifiers,
            getVOP3ModPat<P, node>.ret,
            !if(P.HasIntClamp,
                getVOP3ClampPat<P, node>.ret,
                !if (P.IsMAI,
                    getVOP3MAIPat<P, node>.ret,
                    getVOP3Pat<P, node>.ret)))),
    VOP3Only, 0, P.HasOpSel> {

  let IntClamp = P.HasIntClamp;
  let AsmMatchConverter =
    !if(P.HasOpSel,
        "cvtVOP3OpSel",
        !if(!or(P.HasModifiers, !or(P.HasOMod, P.HasIntClamp)),
            "cvtVOP3",
            ""));
}

// Special case for v_div_fmas_{f32|f64}, since it seems to be the
// only VOP instruction that implicitly reads VCC.
let Asm64 = " $vdst, $src0_modifiers, $src1_modifiers, $src2_modifiers$clamp$omod" in {
def VOP_F32_F32_F32_F32_VCC : VOPProfile<[f32, f32, f32, f32]> {
  let Outs64 = (outs DstRC.RegClass:$vdst);
}
def VOP_F64_F64_F64_F64_VCC : VOPProfile<[f64, f64, f64, f64]> {
  let Outs64 = (outs DstRC.RegClass:$vdst);
}
}

class VOP3Features<bit Clamp, bit OpSel, bit Packed, bit MAI> {
  bit HasClamp = Clamp;
  bit HasOpSel = OpSel;
  bit IsPacked = Packed;
  bit IsMAI = MAI;
}

def VOP3_REGULAR : VOP3Features<0, 0, 0, 0>;
def VOP3_CLAMP   : VOP3Features<1, 0, 0, 0>;
def VOP3_OPSEL   : VOP3Features<1, 1, 0, 0>;
def VOP3_PACKED  : VOP3Features<1, 1, 1, 0>;
def VOP3_MAI     : VOP3Features<0, 0, 0, 1>;

class VOP3_Profile<VOPProfile P, VOP3Features Features = VOP3_REGULAR> : VOPProfile<P.ArgVT> {

  let HasClamp = !if(Features.HasClamp, 1, P.HasClamp);
  let HasOpSel = !if(Features.HasOpSel, 1, P.HasOpSel);
  let IsMAI    = !if(Features.IsMAI,    1, P.IsMAI);
  let IsPacked = !if(Features.IsPacked, 1, P.IsPacked);

  let HasModifiers = !if(Features.IsPacked, !if(Features.IsMAI, 0, 1), P.HasModifiers);

  // FIXME: Hack to stop printing _e64
  let Outs64 = (outs DstRC.RegClass:$vdst);
  let Asm64 =
    " " # !if(Features.HasOpSel,
              getAsmVOP3OpSel<NumSrcArgs,
                              HasIntClamp,
                              HasSrc0FloatMods,
                              HasSrc1FloatMods,
                              HasSrc2FloatMods>.ret,
              !if(Features.HasClamp,
                  getAsm64<HasDst, NumSrcArgs, HasIntClamp,
                           HasModifiers, HasOMod, DstVT>.ret,
                  P.Asm64));
  let NeedPatGen = P.NeedPatGen;
}

class VOP3b_Profile<ValueType vt> : VOPProfile<[vt, vt, vt, vt]> {
  // v_div_scale_{f32|f64} do not support input modifiers.
  let HasModifiers = 0;
  let HasClamp = 0;
  let HasOMod = 0;
  let Outs64 = (outs DstRC:$vdst, VOPDstS64orS32:$sdst);
  let Asm64 = " $vdst, $sdst, $src0, $src1, $src2";
}

def VOP3b_F32_I1_F32_F32_F32 : VOP3b_Profile<f32> {
  // FIXME: Hack to stop printing _e64
  let DstRC = RegisterOperand<VPR_32>;
}

def VOP3b_F64_I1_F64_F64_F64 : VOP3b_Profile<f64> {
  // FIXME: Hack to stop printing _e64
  let DstRC = RegisterOperand<VReg_64>;
}

def VOP3b_I64_I1_I32_I32_I64 : VOPProfile<[i64, i32, i32, i64]> {
  let HasClamp = 1;

  // FIXME: Hack to stop printing _e64
  let DstRC = RegisterOperand<VReg_64>;

  let Outs64 = (outs DstRC:$vdst, VOPDstS64orS32:$sdst);
  let Asm64 = " $vdst, $sdst, $src0, $src1, $src2$clamp";
}


