//===----------------------------------------------------------------------===//
// VOP2 Instructions
//===----------------------------------------------------------------------===//

defm V_CNDMASK_B32 : VOP2eInst <"v_cndmask_b32", VOP2e_I32_I32_I32_I1>;
def V_MADMK_F32 : VOP2_Pseudo <"v_madmk_f32", VOP_MADMK_F32, []>;

let isCommutable = 1 in {
defm V_ADD_F32 : VOP2Inst <"v_add_f32", VOP_F32_F32_F32, fadd>;
defm V_SUB_F32 : VOP2Inst <"v_sub_f32", VOP_F32_F32_F32, fsub>;
defm V_SUBREV_F32 : VOP2Inst <"v_subrev_f32", VOP_F32_F32_F32, null_frag, "v_sub_f32">;
// defm V_MUL_LEGACY_F32 : VOP2Inst <"v_mul_legacy_f32", VOP_F32_F32_F32, AMDGPUfmul_legacy>;
defm V_MUL_F32 : VOP2Inst <"v_mul_f32", VOP_F32_F32_F32, fmul>;
defm V_MUL_I32_I24 : VOP2Inst <"v_mul_i32_i24", VOP_PAT_GEN<VOP_I32_I32_I32, 2>, PPUmul_i24>;
defm V_MUL_HI_I32_I24 : VOP2Inst <"v_mul_hi_i32_i24", VOP_PAT_GEN<VOP_I32_I32_I32, 2>, PPUmulhi_i24>;
defm V_MUL_U32_U24 : VOP2Inst <"v_mul_u32_u24", VOP_PAT_GEN<VOP_I32_I32_I32, 2>, PPUmul_u24>;
defm V_MUL_HI_U32_U24 : VOP2Inst <"v_mul_hi_u32_u24", VOP_PAT_GEN<VOP_I32_I32_I32, 2>, PPUmulhi_u24>;
defm V_MIN_F32 : VOP2Inst <"v_min_f32", VOP_F32_F32_F32, fminnum_like>;
defm V_MAX_F32 : VOP2Inst <"v_max_f32", VOP_F32_F32_F32, fmaxnum_like>;
defm V_MIN_I32 : VOP2Inst <"v_min_i32", VOP_PAT_GEN<VOP_I32_I32_I32>, smin>;
defm V_MAX_I32 : VOP2Inst <"v_max_i32", VOP_PAT_GEN<VOP_I32_I32_I32>, smax>;
defm V_MIN_U32 : VOP2Inst <"v_min_u32", VOP_PAT_GEN<VOP_I32_I32_I32>, umin>;
defm V_MAX_U32 : VOP2Inst <"v_max_u32", VOP_PAT_GEN<VOP_I32_I32_I32>, umax>;
defm V_LSHRREV_B32 : VOP2Inst <"v_lshrrev_b32", VOP_I32_I32_I32, lshr_rev, "v_lshr_b32">;
defm V_ASHRREV_I32 : VOP2Inst <"v_ashrrev_i32", VOP_I32_I32_I32, ashr_rev, "v_ashr_i32">;
defm V_LSHLREV_B32 : VOP2Inst <"v_lshlrev_b32", VOP_I32_I32_I32, lshl_rev, "v_lshl_b32">;
defm V_AND_B32 : VOP2Inst <"v_and_b32", VOP_PAT_GEN<VOP_I32_I32_I32>, and>;
defm V_OR_B32 : VOP2Inst <"v_or_b32", VOP_PAT_GEN<VOP_I32_I32_I32>, or>;
defm V_XOR_B32 : VOP2Inst <"v_xor_b32", VOP_PAT_GEN<VOP_I32_I32_I32>, xor>;

let Constraints = "$vdst = $src2", DisableEncoding="$src2",
    isConvertibleToThreeAddress = 1 in {
defm V_MAC_F32 : VOP2Inst <"v_mac_f32", VOP_MAC_F32>;
}

def V_MADAK_F32 : VOP2_Pseudo <"v_madak_f32", VOP_MADAK_F32, []>;

// No patterns so that the scalar instructions are always selected.
// The scalar versions will be replaced with vector when needed later.

// V_ADD_I32, V_SUB_I32, and V_SUBREV_I32 where renamed to *_U32 in VI,
// but the VI instructions behave the same as the SI versions.
defm V_ADD_I32 : VOP2bInst <"v_add_i32", VOP2b_I32_I1_I32_I32, null_frag, "v_add_i32", 1>;
defm V_SUB_I32 : VOP2bInst <"v_sub_i32", VOP2b_I32_I1_I32_I32, null_frag, "v_sub_i32", 1>;
defm V_SUBREV_I32 : VOP2bInst <"v_subrev_i32", VOP2b_I32_I1_I32_I32, null_frag, "v_sub_i32", 1>;
defm V_ADDC_U32 : VOP2bInst <"v_addc_u32", VOP2b_I32_I1_I32_I32_I1, null_frag, "v_addc_u32", 1>;
defm V_SUBB_U32 : VOP2bInst <"v_subb_u32", VOP2b_I32_I1_I32_I32_I1, null_frag, "v_subb_u32", 1>;
defm V_SUBBREV_U32 : VOP2bInst <"v_subbrev_u32", VOP2b_I32_I1_I32_I32_I1, null_frag, "v_subb_u32", 1>;


let SubtargetPredicate = HasAddNoCarryInsts in {
defm V_ADD_U32 : VOP2Inst <"v_add_u32", VOP_I32_I32_I32_ARITH, null_frag, "v_add_u32", 1>;
defm V_SUB_U32 : VOP2Inst <"v_sub_u32", VOP_I32_I32_I32_ARITH, null_frag, "v_sub_u32", 1>;
defm V_SUBREV_U32 : VOP2Inst <"v_subrev_u32", VOP_I32_I32_I32_ARITH, null_frag, "v_sub_u32", 1>;
}

} // End isCommutable = 1


defm V_BFM_B32 : VOP2Inst <"v_bfm_b32", VOP_NO_EXT<VOP_I32_I32_I32>>;
defm V_BCNT_U32_B32 : VOP2Inst <"v_bcnt_u32_b32", VOP_NO_EXT<VOP_I32_I32_I32>>;
// defm V_MBCNT_LO_U32_B32 : VOP2Inst <"v_mbcnt_lo_u32_b32", VOP_NO_EXT<VOP_I32_I32_I32>, int_ppu_mbcnt_lo>;
// defm V_MBCNT_HI_U32_B32 : VOP2Inst <"v_mbcnt_hi_u32_b32", VOP_NO_EXT<VOP_I32_I32_I32>, int_ppu_mbcnt_hi>;
// defm V_LDEXP_F32 : VOP2Inst <"v_ldexp_f32", VOP_NO_EXT<VOP_F32_F32_I32>, AMDGPUldexp>;
defm V_CVT_PKACCUM_U8_F32 : VOP2Inst <"v_cvt_pkaccum_u8_f32", VOP_NO_EXT<VOP_I32_F32_I32>>; // TODO: set "Uses = dst"
defm V_CVT_PKNORM_I16_F32 : VOP2Inst <"v_cvt_pknorm_i16_f32", VOP_NO_EXT<VOP_V2I16_F32_F32>, PPUpknorm_i16_f32>;
defm V_CVT_PKNORM_U16_F32 : VOP2Inst <"v_cvt_pknorm_u16_f32", VOP_NO_EXT<VOP_V2I16_F32_F32>, PPUpknorm_u16_f32>;
defm V_CVT_PKRTZ_F16_F32 : VOP2Inst <"v_cvt_pkrtz_f16_f32", VOP_NO_EXT<VOP_V2F16_F32_F32>, PPUpkrtz_f16_f32>;
defm V_CVT_PK_U16_U32 : VOP2Inst <"v_cvt_pk_u16_u32", VOP_NO_EXT<VOP_V2I16_I32_I32>, PPUpk_u16_u32>;
defm V_CVT_PK_I16_I32 : VOP2Inst <"v_cvt_pk_i16_i32", VOP_NO_EXT<VOP_V2I16_I32_I32>, PPUpk_i16_i32>;

let isCommutable = 1 in {
defm V_LSHR_B32 : VOP2Inst <"v_lshr_b32", VOP_I32_I32_I32>;
defm V_ASHR_I32 : VOP2Inst <"v_ashr_i32", VOP_I32_I32_I32>;
defm V_LSHL_B32 : VOP2Inst <"v_lshl_b32", VOP_I32_I32_I32>;
} // End isCommutable = 1


class DivergentBinOp<SDPatternOperator Op, VOP_Pseudo Inst> :
  PPUPat<
      (getDivergentFrag<Op>.ret Inst.Pfl.Src0VT:$src0, Inst.Pfl.Src1VT:$src1),
      !if(!cast<Commutable_REV>(Inst).IsOrig,
        (Inst $src0, $src1),
        (Inst $src1, $src0)
      )
  >;

class DivergentClampingBinOp<SDPatternOperator Op, VOP_Pseudo Inst> :
  PPUPat<
      (getDivergentFrag<Op>.ret Inst.Pfl.Src0VT:$src0, Inst.Pfl.Src1VT:$src1),
      !if(!cast<Commutable_REV>(Inst).IsOrig,
        (Inst $src0, $src1, 0),
        (Inst $src1, $src0, 0)
      )
  >;

def : DivergentBinOp<srl, V_LSHRREV_B32_e64>;
def : DivergentBinOp<sra, V_ASHRREV_I32_e64>;
def : DivergentBinOp<shl, V_LSHLREV_B32_e64>;

let SubtargetPredicate = HasAddNoCarryInsts in {
  def : DivergentClampingBinOp<add, V_ADD_U32_e64>;
  def : DivergentClampingBinOp<sub, V_SUB_U32_e64>;
}

def : DivergentClampingBinOp<add, V_ADD_I32_e64>;
def : DivergentClampingBinOp<sub, V_SUB_I32_e64>;

def : DivergentBinOp<adde, V_ADDC_U32_e32>;
def : DivergentBinOp<sube, V_SUBB_U32_e32>;

class divergent_i64_BinOp <SDPatternOperator Op, Instruction Inst> :
  PPUPat<
      (getDivergentFrag<Op>.ret i64:$src0, i64:$src1),
      (REG_SEQUENCE VReg_64,
        (Inst
          (i32 (EXTRACT_SUBREG $src0, sub0)),
          (i32 (EXTRACT_SUBREG $src1, sub0))
        ), sub0,
        (Inst
          (i32 (EXTRACT_SUBREG $src0, sub1)),
          (i32 (EXTRACT_SUBREG $src1, sub1))
        ), sub1
      )
  >;

def :  divergent_i64_BinOp <and, V_AND_B32_e32>;
def :  divergent_i64_BinOp <or,  V_OR_B32_e32>;
def :  divergent_i64_BinOp <xor, V_XOR_B32_e32>;

let SubtargetPredicate = Has16BitInsts in {

let FPDPRounding = 1 in {
def V_MADMK_F16 : VOP2_Pseudo <"v_madmk_f16", VOP_MADMK_F16, [], "">;
// defm V_LDEXP_F16 : VOP2Inst <"v_ldexp_f16", VOP_F16_F16_I32, PPUldexp>;
} // End FPDPRounding = 1

defm V_LSHLREV_B16 : VOP2Inst <"v_lshlrev_b16", VOP_I16_I16_I16>;
defm V_LSHRREV_B16 : VOP2Inst <"v_lshrrev_b16", VOP_I16_I16_I16>;
defm V_ASHRREV_I16 : VOP2Inst <"v_ashrrev_i16", VOP_I16_I16_I16>;

let isCommutable = 1 in {
let FPDPRounding = 1 in {
defm V_ADD_F16 : VOP2Inst <"v_add_f16", VOP_F16_F16_F16, fadd>;
defm V_SUB_F16 : VOP2Inst <"v_sub_f16", VOP_F16_F16_F16, fsub>;
defm V_SUBREV_F16 : VOP2Inst <"v_subrev_f16", VOP_F16_F16_F16, null_frag, "v_sub_f16">;
defm V_MUL_F16 : VOP2Inst <"v_mul_f16", VOP_F16_F16_F16, fmul>;
def V_MADAK_F16 : VOP2_Pseudo <"v_madak_f16", VOP_MADAK_F16, [], "">;
} // End FPDPRounding = 1
defm V_ADD_U16 : VOP2Inst <"v_add_u16", VOP_I16_I16_I16>;
defm V_SUB_U16 : VOP2Inst <"v_sub_u16" , VOP_I16_I16_I16>;
defm V_SUBREV_U16 : VOP2Inst <"v_subrev_u16", VOP_I16_I16_I16, null_frag, "v_sub_u16">;
defm V_MUL_LO_U16 : VOP2Inst <"v_mul_lo_u16", VOP_I16_I16_I16>;
defm V_MAX_F16 : VOP2Inst <"v_max_f16", VOP_F16_F16_F16, fmaxnum_like>;
defm V_MIN_F16 : VOP2Inst <"v_min_f16", VOP_F16_F16_F16, fminnum_like>;
defm V_MAX_U16 : VOP2Inst <"v_max_u16", VOP_I16_I16_I16>;
defm V_MAX_I16 : VOP2Inst <"v_max_i16", VOP_I16_I16_I16>;
defm V_MIN_U16 : VOP2Inst <"v_min_u16", VOP_I16_I16_I16>;
defm V_MIN_I16 : VOP2Inst <"v_min_i16", VOP_I16_I16_I16>;

let Constraints = "$vdst = $src2", DisableEncoding="$src2",
    isConvertibleToThreeAddress = 1 in {
defm V_MAC_F16 : VOP2Inst <"v_mac_f16", VOP_MAC_F16>;
}
} // End isCommutable = 1

} // End SubtargetPredicate = Has16BitInsts

let SubtargetPredicate = HasDLInsts in {

defm V_XNOR_B32 : VOP2Inst <"v_xnor_b32", VOP_I32_I32_I32>;

let Constraints = "$vdst = $src2",
    DisableEncoding="$src2",
    isConvertibleToThreeAddress = 1,
    isCommutable = 1 in {
defm V_FMAC_F32 : VOP2Inst <"v_fmac_f32", VOP_MAC_F32>;
}

} // End SubtargetPredicate = HasDLInsts

let Constraints = "$vdst = $src2",
      DisableEncoding="$src2",
      isConvertibleToThreeAddress = 1,
      isCommutable = 1 in {

    defm V_DOT2C_F32_F16 : VOP2Inst_e32<"v_dot2c_f32_f16", VOP_DOT_ACC_F32_V2F16>;
    defm V_DOT4C_I32_I8  : VOP2Inst_e32<"v_dot4c_i32_i8",  VOP_DOT_ACC_I32_I32>;
    defm V_DOT2C_I32_I16 : VOP2Inst_e32<"v_dot2c_i32_i16", VOP_DOT_ACC_I32_I32>;
    defm V_DOT8C_I32_I4  : VOP2Inst_e32<"v_dot8c_i32_i4",  VOP_DOT_ACC_I32_I32>;
}

let AddedComplexity = 30 in {
  def : PPUPat<
    (f32 (PPUfdot2 v2f16:$src0, v2f16:$src1, f32:$src2, (i1 DSTCLAMP.NONE))),
    (f32 (V_DOT2C_F32_F16_e32 $src0, $src1, $src2))
  > {
    // let SubtargetPredicate = HasDot5Insts;
  }

  def : PPUPat<
    (i32 (int_ppu_sdot4 i32:$src0, i32:$src1, i32:$src2, (i1 DSTCLAMP.NONE))),
    (i32 (V_DOT4C_I32_I8_e32 $src0, $src1, $src2))
  > {
    // let SubtargetPredicate = HasDot6Insts;
  }

  def : PPUPat<
    (i32 (int_ppu_sdot2 v2i16:$src0, v2i16:$src1, i32:$src2, (i1 DSTCLAMP.NONE))),
    (i32 (V_DOT2C_I32_I16_e32 $src0, $src1, $src2))
  > {
    // let SubtargetPredicate = HasDot4Insts;
  }

  def : PPUPat<
    (i32 (int_ppu_sdot8 i32:$src0, i32:$src1, i32:$src2, (i1 DSTCLAMP.NONE))),
    (i32 (V_DOT8C_I32_I4_e32 $src0, $src1, $src2))
  > {
    // let SubtargetPredicate = HasDot3Insts;
  }
} // End AddedComplexity = 30

let SubtargetPredicate = isGFX10Plus in {

def V_FMAMK_F32 : VOP2_Pseudo<"v_fmamk_f32", VOP_MADMK_F32, [], "">;
let FPDPRounding = 1 in
def V_FMAMK_F16 : VOP2_Pseudo <"v_fmamk_f16", VOP_MADMK_F16, [], "">;

let isCommutable = 1 in {
def V_FMAAK_F32 : VOP2_Pseudo<"v_fmaak_f32", VOP_MADAK_F32, [], "">;
let FPDPRounding = 1 in
def V_FMAAK_F16 : VOP2_Pseudo <"v_fmaak_f16", VOP_MADAK_F16, [], "">;
} // End isCommutable = 1

let Constraints = "$vdst = $src2",
    DisableEncoding="$src2",
    isConvertibleToThreeAddress = 1,
    isCommutable = 1 in {
defm V_FMAC_F16 : VOP2Inst <"v_fmac_f16", VOP_MAC_F16>;
}

} // End SubtargetPredicate = isGFX10Plus

let SubtargetPredicate = HasPkFmacF16Inst in {
defm V_PK_FMAC_F16 : VOP2Inst<"v_pk_fmac_f16", VOP_V2F16_V2F16_V2F16>;
} // End SubtargetPredicate = HasPkFmacF16Inst

// Note: 16-bit instructions produce a 0 result in the high 16-bits
// on GFX8 and GFX9 and preserve high 16 bits on GFX10+
def ClearHI16 : OutPatFrag<(ops node:$op),
                           (V_AND_B32_e64 $op, (V_MOV_B32_e32 (i32 0xffff)))>;

multiclass Arithmetic_i16_Pats <SDPatternOperator op, Instruction inst,
                                bit PreservesHI16 = 0> {

def : PPUPat<
  (op i16:$src0, i16:$src1),
  !if(!eq(PreservesHI16,1), (ClearHI16 (inst $src0, $src1)), (inst $src0, $src1))
>;

def : PPUPat<
  (i32 (zext (op i16:$src0, i16:$src1))),
  !if(!eq(PreservesHI16,1), (ClearHI16 (inst $src0, $src1)), (inst $src0, $src1))
>;

def : PPUPat<
  (i64 (zext (op i16:$src0, i16:$src1))),
   (REG_SEQUENCE VReg_64,
     !if(!eq(PreservesHI16,1), (ClearHI16 (inst $src0, $src1)), (inst $src0, $src1)),
     sub0,
     (V_MOV_B32_e32 (i32 0)), sub1)
>;
}

multiclass Bits_OpsRev_i16_Pats <SDPatternOperator op, Instruction inst,
                                 bit PreservesHI16 = 0> {

def : PPUPat<
  (op i16:$src0, i16:$src1),
  !if(!eq(PreservesHI16,1), (ClearHI16 (inst $src1, $src0)), (inst $src1, $src0))
>;

def : PPUPat<
  (i32 (zext (op i16:$src0, i16:$src1))),
  !if(!eq(PreservesHI16,1), (ClearHI16 (inst $src1, $src0)), (inst $src1, $src0))
>;


def : PPUPat<
  (i64 (zext (op i16:$src0, i16:$src1))),
   (REG_SEQUENCE VReg_64,
     !if(!eq(PreservesHI16,1), (ClearHI16 (inst $src1, $src0)), (inst $src1, $src0)),
     sub0,
     (V_MOV_B32_e32 (i32 0)), sub1)
>;
}

class ZExt_i16_i1_Pat <SDNode ext> : PPUPat <
  (i16 (ext i1:$src)),
  (V_CNDMASK_B32_e64 (i32 0/*src0mod*/), (i32 0/*src0*/),
                     (i32 0/*src1mod*/), (i32 1/*src1*/),
                     $src)
>;

let Predicates = [Has16BitInsts] in {
defm : Arithmetic_i16_Pats<add, V_ADD_U16_e64,    1>;
defm : Arithmetic_i16_Pats<mul, V_MUL_LO_U16_e64, 1>;
defm : Arithmetic_i16_Pats<sub, V_SUB_U16_e64,    1>;
defm : Arithmetic_i16_Pats<smin, V_MIN_I16_e64,   1>;
defm : Arithmetic_i16_Pats<smax, V_MAX_I16_e64,   1>;
defm : Arithmetic_i16_Pats<umin, V_MIN_U16_e64,   1>;
defm : Arithmetic_i16_Pats<umax, V_MAX_U16_e64,   1>;
}

def : PPUPat <
  (and i16:$src0, i16:$src1),
  (V_AND_B32_e64 $src0, $src1)
>;

def : PPUPat <
  (or i16:$src0, i16:$src1),
  (V_OR_B32_e64 $src0, $src1)
>;

def : PPUPat <
  (xor i16:$src0, i16:$src1),
  (V_XOR_B32_e64 $src0, $src1)
>;

let Predicates = [Has16BitInsts] in {
defm : Bits_OpsRev_i16_Pats<shl, V_LSHLREV_B16_e64, 1>;
defm : Bits_OpsRev_i16_Pats<srl, V_LSHRREV_B16_e64, 1>;
defm : Bits_OpsRev_i16_Pats<sra, V_ASHRREV_I16_e64, 1>;
}

def : ZExt_i16_i1_Pat<zext>;
def : ZExt_i16_i1_Pat<anyext>;

def : PPUPat <
  (i16 (sext i1:$src)),
  (V_CNDMASK_B32_e64 /*src0mod*/(i32 0), /*src0*/(i32 0),
                     /*src1mod*/(i32 0), /*src1*/(i32 -1), $src)
>;

// Undo sub x, c -> add x, -c canonicalization since c is more likely
// an inline immediate than -c.
// TODO: Also do for 64-bit.
def : PPUPat<
  (add i16:$src0, (i16 NegSubInlineConst16:$src1)),
  (V_SUB_U16_e64 $src0, NegSubInlineConst16:$src1)
>;

// } // End Predicates = [Has16BitInsts, isGFX7GFX8GFX9]


//===----------------------------------------------------------------------===//
// Target-specific instruction encodings.
//===----------------------------------------------------------------------===//
/*
class VOP2_DPP<bits<6> op, VOP2_Pseudo ps,
               string opName = ps.OpName, VOPProfile p = ps.Pfl,
               bit IsDPP16 = 0> :
    VOP_DPP<opName, p, IsDPP16> {
  let hasSideEffects = ps.hasSideEffects;
  let Defs = ps.Defs;
  let SchedRW = ps.SchedRW;
  let Uses = ps.Uses;

  bits<8> vdst;
  bits<8> src1;
  let Inst{8-0}   = 0xfa;
  let Inst{16-9}  = !if(p.HasSrc1, src1{7-0}, 0);
  let Inst{24-17} = !if(p.EmitDst, vdst{7-0}, 0);
  let Inst{30-25} = op;
  let Inst{31}    = 0x0;
}

class VOP2_DPP16<bits<6> op, VOP2_Pseudo ps,
                 string opName = ps.OpName, VOPProfile p = ps.Pfl> :
    VOP2_DPP<op, ps, opName, p, 1> {
  let AssemblerPredicate = !if(p.HasExt, HasDPP16, DisableInst);
  let SubtargetPredicate = HasDPP16;
}

class VOP2_DPP8<bits<6> op, VOP2_Pseudo ps,
                string opName = ps.OpName, VOPProfile p = ps.Pfl> :
    VOP_DPP8<ps.OpName, p> {
  let hasSideEffects = ps.hasSideEffects;
  let Defs = ps.Defs;
  let SchedRW = ps.SchedRW;
  let Uses = ps.Uses;

  bits<8> vdst;
  bits<8> src1;

  let Inst{8-0}   = fi;
  let Inst{16-9}  = !if(p.HasSrc1, src1{7-0}, 0);
  let Inst{24-17} = !if(p.EmitDst, vdst{7-0}, 0);
  let Inst{30-25} = op;
  let Inst{31}    = 0x0;

  let AssemblerPredicate = !if(p.HasExt, HasDPP8, DisableInst);
  let SubtargetPredicate = HasDPP8;
}
*/

//===----------------------------------------------------------------------===//
// GFX10.
//===----------------------------------------------------------------------===//

let DecoderNamespace = "PPU" in {
  //===------------------------------- VOP2 -------------------------------===//
  multiclass VOP2Only_Real_MADK_ppu<bits<6> op> {
    def _ppu :
      VOP2_Real<!cast<VOP2_Pseudo>(NAME), PPUEncodingFamily.PPU>,
      VOP2_MADKe<op{5-0}, !cast<VOP2_Pseudo>(NAME).Pfl>;
  }
  multiclass VOP2Only_Real_MADK_ppu_with_name<bits<6> op, string opName,
                                                string asmName> {
    def _ppu :
        VOP2_Real<!cast<VOP2_Pseudo>(opName), PPUEncodingFamily.PPU>,
        VOP2_MADKe<op{5-0}, !cast<VOP2_Pseudo>(opName).Pfl> {
      VOP2_Pseudo ps = !cast<VOP2_Pseudo>(opName);
      let AsmString = asmName # ps.AsmOperands;
    }
  }
  multiclass VOP2_Real_e32_ppu<bits<6> op> {
    def _e32_ppu :
      VOP2_Real<!cast<VOP2_Pseudo>(NAME#"_e32"), PPUEncodingFamily.PPU>,
      VOP2e<op{5-0}, !cast<VOP2_Pseudo>(NAME#"_e32").Pfl>;
  }
  multiclass VOP2_Real_e64_ppu<bits<6> op> {
    def _e64_ppu :
      VOP3_Real<!cast<VOP3_Pseudo>(NAME#"_e64"), PPUEncodingFamily.PPU>,
      VOP3e_ppu<{0, 1, 0, 0, op{5-0}}, !cast<VOP3_Pseudo>(NAME#"_e64").Pfl>;
  }

  //===------------------------- VOP2 (with name) -------------------------===//
  multiclass VOP2_Real_e32_ppu_with_name<bits<6> op, string opName,
                                           string asmName> {
    def _e32_ppu :
      VOP2_Real<!cast<VOP2_Pseudo>(opName#"_e32"), PPUEncodingFamily.PPU>,
      VOP2e<op{5-0}, !cast<VOP2_Pseudo>(opName#"_e32").Pfl> {
        VOP2_Pseudo ps = !cast<VOP2_Pseudo>(opName#"_e32");
        let AsmString = asmName # ps.AsmOperands;
      }
  }
  multiclass VOP2_Real_e64_ppu_with_name<bits<6> op, string opName,
                                           string asmName> {
    def _e64_ppu :
      VOP3_Real<!cast<VOP3_Pseudo>(opName#"_e64"), PPUEncodingFamily.PPU>,
      VOP3e_ppu<{0, 1, 0, 0, op{5-0}},
                  !cast<VOP3_Pseudo>(opName#"_e64").Pfl> {
        VOP3_Pseudo ps = !cast<VOP3_Pseudo>(opName#"_e64");
        let AsmString = asmName # ps.AsmOperands;
      }
  }
  //===------------------------------ VOP2be ------------------------------===//
  multiclass VOP2be_Real_ppu<bits<6> op, string opName, string asmName> {
    def _e32_ppu :
      VOP2_Real<!cast<VOP2_Pseudo>(opName#"_e32"), PPUEncodingFamily.PPU>,
      VOP2e<op{5-0}, !cast<VOP2_Pseudo>(opName#"_e32").Pfl> {
        VOP2_Pseudo Ps = !cast<VOP2_Pseudo>(opName#"_e32");
        let AsmString = asmName # !subst(", vcc", "", Ps.AsmOperands);
      }
    def _e64_ppu :
      VOP3_Real<!cast<VOP3_Pseudo>(opName#"_e64"), PPUEncodingFamily.PPU>,
      VOP3be_ppu<{0, 1, 0, 0, op{5-0}},
                   !cast<VOP3_Pseudo>(opName#"_e64").Pfl> {
        VOP3_Pseudo Ps = !cast<VOP3_Pseudo>(opName#"_e64");
        let AsmString = asmName # Ps.AsmOperands;
      }
  }

  //===----------------------------- VOP3Only -----------------------------===//
  multiclass VOP3Only_Real_ppu<bits<10> op> {
    def _e64_ppu :
      VOP3_Real<!cast<VOP3_Pseudo>(NAME#"_e64"), PPUEncodingFamily.PPU>,
      VOP3e_ppu<op, !cast<VOP3_Pseudo>(NAME#"_e64").Pfl>;
  }

  //===---------------------------- VOP3beOnly ----------------------------===//
  multiclass VOP3beOnly_Real_ppu<bits<10> op, string opName, string asmName> {
    def _e64_ppu :
      VOP3_Real<!cast<VOP3_Pseudo>(opName#"_e64"), PPUEncodingFamily.PPU>,
      VOP3be_ppu<op, !cast<VOP3_Pseudo>(opName#"_e64").Pfl> {
        VOP3_Pseudo Ps = !cast<VOP3_Pseudo>(opName#"_e64");
        let AsmString = asmName # Ps.AsmOperands;
      }
  }
} // End AssemblerPredicate = isGFX10Plus, DecoderNamespace = "GFX10"

multiclass VOP2_Real_ppu<bits<6> op> :
  VOP2_Real_e32_ppu<op>, VOP2_Real_e64_ppu<op>;

multiclass VOP2_Real_ppu_with_name<bits<6> op, string opName,
                                     string asmName> :
  VOP2_Real_e32_ppu_with_name<op, opName, asmName>,
  VOP2_Real_e64_ppu_with_name<op, opName, asmName>;

defm V_CNDMASK_B32   : VOP2_Real_ppu<0x001>;
// defm V_XNOR_B32      : VOP2_Real_ppu<0x01e>;
// defm V_FMAC_F32      : VOP2_Real_ppu<0x02b>;
defm V_FMAMK_F32     : VOP2Only_Real_MADK_ppu<0x02c>;
defm V_FMAAK_F32     : VOP2Only_Real_MADK_ppu<0x02d>;
defm V_ADD_F16       : VOP2_Real_ppu<0x032>;
defm V_SUB_F16       : VOP2_Real_ppu<0x033>;
defm V_SUBREV_F16    : VOP2_Real_ppu<0x034>;
defm V_MUL_F16       : VOP2_Real_ppu<0x035>;
defm V_FMAC_F16      : VOP2_Real_ppu<0x036>;
defm V_FMAMK_F16     : VOP2Only_Real_MADK_ppu<0x037>;
defm V_FMAAK_F16     : VOP2Only_Real_MADK_ppu<0x038>;
defm V_MAX_F16       : VOP2_Real_ppu<0x039>;
defm V_MIN_F16       : VOP2_Real_ppu<0x03a>;
// defm V_LDEXP_F16     : VOP2_Real_ppu<0x03b>;
defm V_PK_FMAC_F16   : VOP2_Real_e32_ppu<0x03c>;

// VOP2 no carry-in, carry-out.
defm V_ADD_NC_U32 :
  VOP2_Real_ppu_with_name<0x025, "V_ADD_U32", "v_add_nc_u32">;
defm V_SUB_NC_U32 :
  VOP2_Real_ppu_with_name<0x026, "V_SUB_U32", "v_sub_nc_u32">;
defm V_SUBREV_NC_U32 :
  VOP2_Real_ppu_with_name<0x027, "V_SUBREV_U32", "v_subrev_nc_u32">;

// VOP2 carry-in, carry-out.
defm V_ADD_CO_CI_U32 :
  VOP2be_Real_ppu<0x028, "V_ADDC_U32", "v_add_co_ci_u32">;
defm V_SUB_CO_CI_U32 :
  VOP2be_Real_ppu<0x029, "V_SUBB_U32", "v_sub_co_ci_u32">;
defm V_SUBREV_CO_CI_U32 :
  VOP2be_Real_ppu<0x02a, "V_SUBBREV_U32", "v_subrev_co_ci_u32">;

// VOP3 only.
defm V_BFM_B32            : VOP3Only_Real_ppu<0x363>;
defm V_BCNT_U32_B32       : VOP3Only_Real_ppu<0x364>;
// defm V_MBCNT_LO_U32_B32   : VOP3Only_Real_ppu<0x365>;
// defm V_MBCNT_HI_U32_B32   : VOP3Only_Real_ppu<0x366>;
// defm V_LDEXP_F32          : VOP3Only_Real_ppu<0x362>;
defm V_CVT_PKNORM_I16_F32 : VOP3Only_Real_ppu<0x368>;
defm V_CVT_PKNORM_U16_F32 : VOP3Only_Real_ppu<0x369>;
defm V_CVT_PK_U16_U32     : VOP3Only_Real_ppu<0x36a>;
defm V_CVT_PK_I16_I32     : VOP3Only_Real_ppu<0x36b>;

// VOP3 carry-in, carry-out.
defm V_ADD_CO_U32 :
  VOP3beOnly_Real_ppu<0x30f, "V_ADD_I32", "v_add_co_u32">;
defm V_SUB_CO_U32 :
  VOP3beOnly_Real_ppu<0x310, "V_SUB_I32", "v_sub_co_u32">;
defm V_SUBREV_CO_U32 :
  VOP3beOnly_Real_ppu<0x319, "V_SUBREV_I32", "v_subrev_co_u32">;

let SubtargetPredicate = isGFX10Plus in {
  defm : VOP2eInstAliases<V_CNDMASK_B32_e32, V_CNDMASK_B32_e32_ppu>;

  defm : VOP2bInstAliases<
    V_ADDC_U32_e32, V_ADD_CO_CI_U32_e32_ppu, "v_add_co_ci_u32">;
  defm : VOP2bInstAliases<
    V_SUBB_U32_e32, V_SUB_CO_CI_U32_e32_ppu, "v_sub_co_ci_u32">;
  defm : VOP2bInstAliases<
    V_SUBBREV_U32_e32, V_SUBREV_CO_CI_U32_e32_ppu, "v_subrev_co_ci_u32">;
} // End SubtargetPredicate = isGFX10Plus


//===----------------------------------------------------------------------===//
// GFX6, GFX7, GFX10.
//===----------------------------------------------------------------------===//
/*
class VOP2_DPPe <bits<6> op, VOP2_DPP_Pseudo ps, VOPProfile P = ps.Pfl> :
  VOP_DPPe <P> {
  bits<8> vdst;
  bits<8> src1;
  let Inst{8-0}   = 0xfa; //dpp
  let Inst{16-9}  = !if(P.HasSrc1, src1{7-0}, 0);
  let Inst{24-17} = !if(P.EmitDst, vdst{7-0}, 0);
  let Inst{30-25} = op;
  let Inst{31}    = 0x0; //encoding
}

let AssemblerPredicate = isGFX6GFX7, DecoderNamespace = "GFX6GFX7" in {
  multiclass VOP2Only_Real_gfx6_gfx7<bits<6> op> {
    def _gfx6_gfx7 :
      VOP2_Real<!cast<VOP2_Pseudo>(NAME), SIEncodingFamily.SI>,
      VOP2e<op{5-0}, !cast<VOP2_Pseudo>(NAME).Pfl>;
  }
  multiclass VOP2Only_Real_MADK_gfx6_gfx7<bits<6> op> {
    def _gfx6_gfx7 :
      VOP2_Real<!cast<VOP2_Pseudo>(NAME), SIEncodingFamily.SI>,
      VOP2_MADKe<op{5-0}, !cast<VOP2_Pseudo>(NAME).Pfl>;
  }
  multiclass VOP2_Real_e32_gfx6_gfx7<bits<6> op> {
    def _e32_gfx6_gfx7 :
      VOP2_Real<!cast<VOP2_Pseudo>(NAME#"_e32"), SIEncodingFamily.SI>,
      VOP2e<op{5-0}, !cast<VOP2_Pseudo>(NAME#"_e32").Pfl>;
  }
  multiclass VOP2_Real_e64_gfx6_gfx7<bits<6> op> {
    def _e64_gfx6_gfx7 :
      VOP3_Real<!cast<VOP3_Pseudo>(NAME#"_e64"), SIEncodingFamily.SI>,
      VOP3e_gfx6_gfx7<{1, 0, 0, op{5-0}}, !cast<VOP3_Pseudo>(NAME#"_e64").Pfl>;
  }
  multiclass VOP2be_Real_e64_gfx6_gfx7<bits<6> op> {
    def _e64_gfx6_gfx7 :
      VOP3_Real<!cast<VOP3_Pseudo>(NAME#"_e64"), SIEncodingFamily.SI>,
      VOP3be_gfx6_gfx7<{1, 0, 0, op{5-0}}, !cast<VOP3_Pseudo>(NAME#"_e64").Pfl>;
  }
} // End AssemblerPredicate = isGFX6GFX7, DecoderNamespace = "GFX6GFX7"

multiclass VOP2Only_Real_MADK_gfx6_gfx7_ppu<bits<6> op> :
  VOP2Only_Real_MADK_gfx6_gfx7<op>, VOP2Only_Real_MADK_ppu<op>;

multiclass VOP2_Real_gfx6_gfx7<bits<6> op> :
  VOP2_Real_e32_gfx6_gfx7<op>, VOP2_Real_e64_gfx6_gfx7<op>;

multiclass VOP2_Real_gfx6_gfx7_ppu<bits<6> op> :
  VOP2_Real_gfx6_gfx7<op>, VOP2_Real_ppu<op>;

multiclass VOP2be_Real_gfx6_gfx7<bits<6> op> :
  VOP2_Real_e32_gfx6_gfx7<op>, VOP2be_Real_e64_gfx6_gfx7<op>;

defm V_CNDMASK_B32        : VOP2_Real_gfx6_gfx7<0x000>;
defm V_MIN_LEGACY_F32     : VOP2_Real_gfx6_gfx7<0x00d>;
defm V_MAX_LEGACY_F32     : VOP2_Real_gfx6_gfx7<0x00e>;
defm V_LSHR_B32           : VOP2_Real_gfx6_gfx7<0x015>;
defm V_ASHR_I32           : VOP2_Real_gfx6_gfx7<0x017>;
defm V_LSHL_B32           : VOP2_Real_gfx6_gfx7<0x019>;
defm V_BFM_B32            : VOP2_Real_gfx6_gfx7<0x01e>;
defm V_BCNT_U32_B32       : VOP2_Real_gfx6_gfx7<0x022>;
defm V_MBCNT_LO_U32_B32   : VOP2_Real_gfx6_gfx7<0x023>;
defm V_MBCNT_HI_U32_B32   : VOP2_Real_gfx6_gfx7<0x024>;
defm V_LDEXP_F32          : VOP2_Real_gfx6_gfx7<0x02b>;
defm V_CVT_PKACCUM_U8_F32 : VOP2_Real_gfx6_gfx7<0x02c>;
defm V_CVT_PKNORM_I16_F32 : VOP2_Real_gfx6_gfx7<0x02d>;
defm V_CVT_PKNORM_U16_F32 : VOP2_Real_gfx6_gfx7<0x02e>;
defm V_CVT_PK_U16_U32     : VOP2_Real_gfx6_gfx7<0x030>;
defm V_CVT_PK_I16_I32     : VOP2_Real_gfx6_gfx7<0x031>;
defm V_ADD_I32            : VOP2be_Real_gfx6_gfx7<0x025>;
defm V_SUB_I32            : VOP2be_Real_gfx6_gfx7<0x026>;
defm V_SUBREV_I32         : VOP2be_Real_gfx6_gfx7<0x027>;
defm V_ADDC_U32           : VOP2be_Real_gfx6_gfx7<0x028>;
defm V_SUBB_U32           : VOP2be_Real_gfx6_gfx7<0x029>;
defm V_SUBBREV_U32        : VOP2be_Real_gfx6_gfx7<0x02a>;
*/
/*
defm V_READLANE_B32 : VOP2Only_Real_gfx6_gfx7<0x001>;

let InOperandList = (ins SSrcOrLds_b32:$src0, SCSrc_b32:$src1, VSrc_b32:$vdst_in) in {
  defm V_WRITELANE_B32 : VOP2Only_Real_gfx6_gfx7<0x002>;
} // End InOperandList = (ins SSrcOrLds_b32:$src0, SCSrc_b32:$src1, VSrc_b32:$vdst_in)

let SubtargetPredicate = isGFX6GFX7 in {
  defm : VOP2eInstAliases<V_CNDMASK_B32_e32, V_CNDMASK_B32_e32_gfx6_gfx7>;
} // End SubtargetPredicate = isGFX6GFX7
*/

defm V_ADD_F32            : VOP2_Real_ppu<0x003>;
defm V_SUB_F32            : VOP2_Real_ppu<0x004>;
defm V_SUBREV_F32         : VOP2_Real_ppu<0x005>;
// defm V_MAC_LEGACY_F32     : VOP2_Real_ppu<0x006>;
// defm V_MUL_LEGACY_F32     : VOP2_Real_ppu<0x007>;
defm V_MUL_F32            : VOP2_Real_ppu<0x008>;
defm V_MUL_I32_I24        : VOP2_Real_ppu<0x009>;
defm V_MUL_HI_I32_I24     : VOP2_Real_ppu<0x00a>;
defm V_MUL_U32_U24        : VOP2_Real_ppu<0x00b>;
defm V_MUL_HI_U32_U24     : VOP2_Real_ppu<0x00c>;
defm V_MIN_F32            : VOP2_Real_ppu<0x00f>;
defm V_MAX_F32            : VOP2_Real_ppu<0x010>;
defm V_MIN_I32            : VOP2_Real_ppu<0x011>;
defm V_MAX_I32            : VOP2_Real_ppu<0x012>;
defm V_MIN_U32            : VOP2_Real_ppu<0x013>;
defm V_MAX_U32            : VOP2_Real_ppu<0x014>;
defm V_LSHRREV_B32        : VOP2_Real_ppu<0x016>;
defm V_ASHRREV_I32        : VOP2_Real_ppu<0x018>;
defm V_LSHLREV_B32        : VOP2_Real_ppu<0x01a>;
defm V_AND_B32            : VOP2_Real_ppu<0x01b>;
defm V_OR_B32             : VOP2_Real_ppu<0x01c>;
defm V_XOR_B32            : VOP2_Real_ppu<0x01d>;
defm V_MAC_F32            : VOP2_Real_ppu<0x01f>;
defm V_CVT_PKRTZ_F16_F32  : VOP2_Real_ppu<0x02f>;
defm V_MADMK_F32          : VOP2Only_Real_MADK_ppu<0x020>;
defm V_MADAK_F32          : VOP2Only_Real_MADK_ppu<0x021>;
