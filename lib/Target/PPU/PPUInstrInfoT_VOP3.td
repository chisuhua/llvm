//===----------------------------------------------------------------------===//
// VOP3 Instructions
//===----------------------------------------------------------------------===//

let isCommutable = 1 in {

// def V_MAD_LEGACY_F32 : VOP3Inst <"v_mad_legacy_f32", VOP3_Profile<VOP_F32_F32_F32_F32>>;
def V_MAD_F32 : VOP3Inst <"v_mad_f32", VOP3_Profile<VOP_F32_F32_F32_F32>, fmad>;
def V_MAD_I32_I24 : VOP3Inst <"v_mad_i32_i24", VOP3_Profile<VOP_I32_I32_I32_I32, VOP3_CLAMP>>;
def V_MAD_U32_U24 : VOP3Inst <"v_mad_u32_u24", VOP3_Profile<VOP_I32_I32_I32_I32, VOP3_CLAMP>>;
def V_FMA_F32 : VOP3Inst <"v_fma_f32", VOP3_Profile<VOP_F32_F32_F32_F32>, fma>;
// def V_LERP_U8 : VOP3Inst <"v_lerp_u8", VOP3_Profile<VOP_I32_I32_I32_I32>, int_ppu_lerp>;
/*
let SchedRW = [WriteDoubleAdd] in {
let FPDPRounding = 1 in {
def V_FMA_F64 : VOP3Inst <"v_fma_f64", VOP3_Profile<VOP_F64_F64_F64_F64>, fma>;
def V_ADD_F64 : VOP3Inst <"v_add_f64", VOP3_Profile<VOP_F64_F64_F64>, fadd, 1>;
def V_MUL_F64 : VOP3Inst <"v_mul_f64", VOP3_Profile<VOP_F64_F64_F64>, fmul, 1>;
} // End FPDPRounding = 1
def V_MIN_F64 : VOP3Inst <"v_min_f64", VOP3_Profile<VOP_F64_F64_F64>, fminnum_like, 1>;
def V_MAX_F64 : VOP3Inst <"v_max_f64", VOP3_Profile<VOP_F64_F64_F64>, fmaxnum_like, 1>;
} // End SchedRW = [WriteDoubleAdd]
*/

let SchedRW = [WriteQuarterRate32] in {
def V_MUL_LO_U32 : VOP3Inst <"v_mul_lo_u32", VOP3_Profile<VOP_I32_I32_I32>, mul>;
def V_MUL_HI_U32 : VOP3Inst <"v_mul_hi_u32", VOP3_Profile<VOP_I32_I32_I32>, mulhu>;
def V_MUL_LO_I32 : VOP3Inst <"v_mul_lo_i32", VOP3_Profile<VOP_I32_I32_I32>>;
def V_MUL_HI_I32 : VOP3Inst <"v_mul_hi_i32", VOP3_Profile<VOP_I32_I32_I32>, mulhs>;
} // End SchedRW = [WriteQuarterRate32]

let Uses = [VCC, TMSK] in {
// v_div_fmas_f32:
//   result = src0 * src1 + src2
//   if (vcc)
//     result *= 2^32
//
def V_DIV_FMAS_F32 : VOP3_Pseudo <"v_div_fmas_f32", VOP_F32_F32_F32_F32_VCC, []> {
  let SchedRW = [WriteFloatFMA];
}
// v_div_fmas_f64:
//   result = src0 * src1 + src2
//   if (vcc)
//     result *= 2^64
//
def V_DIV_FMAS_F64 : VOP3_Pseudo <"v_div_fmas_f64", VOP_F64_F64_F64_F64_VCC, []> {
  let SchedRW = [WriteDouble];
  let FPDPRounding = 1;
}
} // End Uses = [VCC, TMSK]

} // End isCommutable = 1
/*
def V_CUBEID_F32 : VOP3Inst <"v_cubeid_f32", VOP3_Profile<VOP_F32_F32_F32_F32>, int_ppu_cubeid>;
def V_CUBESC_F32 : VOP3Inst <"v_cubesc_f32", VOP3_Profile<VOP_F32_F32_F32_F32>, int_ppu_cubesc>;
def V_CUBETC_F32 : VOP3Inst <"v_cubetc_f32", VOP3_Profile<VOP_F32_F32_F32_F32>, int_ppu_cubetc>;
def V_CUBEMA_F32 : VOP3Inst <"v_cubema_f32", VOP3_Profile<VOP_F32_F32_F32_F32>, int_ppu_cubema>;
*/
def V_BFE_U32 : VOP3Inst <"v_bfe_u32", VOP3_Profile<VOP_I32_I32_I32_I32>, PPUbfe_u32>;
def V_BFE_I32 : VOP3Inst <"v_bfe_i32", VOP3_Profile<VOP_I32_I32_I32_I32>, PPUbfe_i32>;
def V_BFI_B32 : VOP3Inst <"v_bfi_b32", VOP3_Profile<VOP_I32_I32_I32_I32>, PPUbfi>;
def V_ALIGNBIT_B32 : VOP3Inst <"v_alignbit_b32", VOP3_Profile<VOP_I32_I32_I32_I32>, int_ppu_alignbit>;
def V_ALIGNBYTE_B32 : VOP3Inst <"v_alignbyte_b32", VOP3_Profile<VOP_I32_I32_I32_I32>, int_ppu_alignbyte>;
def V_MIN3_F32 : VOP3Inst <"v_min3_f32", VOP3_Profile<VOP_F32_F32_F32_F32>, PPUfmin3>;
def V_MIN3_I32 : VOP3Inst <"v_min3_i32", VOP3_Profile<VOP_I32_I32_I32_I32>, PPUsmin3>;
def V_MIN3_U32 : VOP3Inst <"v_min3_u32", VOP3_Profile<VOP_I32_I32_I32_I32>, PPUumin3>;
def V_MAX3_F32 : VOP3Inst <"v_max3_f32", VOP3_Profile<VOP_F32_F32_F32_F32>, PPUfmax3>;
def V_MAX3_I32 : VOP3Inst <"v_max3_i32", VOP3_Profile<VOP_I32_I32_I32_I32>, PPUsmax3>;
def V_MAX3_U32 : VOP3Inst <"v_max3_u32", VOP3_Profile<VOP_I32_I32_I32_I32>, PPUumax3>;
def V_MED3_F32 : VOP3Inst <"v_med3_f32", VOP3_Profile<VOP_F32_F32_F32_F32>, PPUfmed3>;
def V_MED3_I32 : VOP3Inst <"v_med3_i32", VOP3_Profile<VOP_I32_I32_I32_I32>, PPUsmed3>;
def V_MED3_U32 : VOP3Inst <"v_med3_u32", VOP3_Profile<VOP_I32_I32_I32_I32>, PPUumed3>;
def V_SAD_U8 : VOP3Inst <"v_sad_u8", VOP3_Profile<VOP_I32_I32_I32_I32, VOP3_CLAMP>>;
def V_SAD_HI_U8 : VOP3Inst <"v_sad_hi_u8", VOP3_Profile<VOP_I32_I32_I32_I32, VOP3_CLAMP>>;
def V_SAD_U16 : VOP3Inst <"v_sad_u16", VOP3_Profile<VOP_I32_I32_I32_I32, VOP3_CLAMP>>;
def V_SAD_U32 : VOP3Inst <"v_sad_u32", VOP3_Profile<VOP_I32_I32_I32_I32, VOP3_CLAMP>>;
def V_CVT_PK_U8_F32 : VOP3Inst<"v_cvt_pk_u8_f32", VOP3_Profile<VOP_I32_F32_I32_I32>, int_ppu_cvt_pk_u8_f32>;
def V_DIV_FIXUP_F32 : VOP3Inst <"v_div_fixup_f32", VOP3_Profile<VOP_F32_F32_F32_F32>, PPUdiv_fixup>;

let SchedRW = [WriteDoubleAdd], FPDPRounding = 1 in {
// def V_DIV_FIXUP_F64 : VOP3Inst <"v_div_fixup_f64", VOP3_Profile<VOP_F64_F64_F64_F64>, PPUdiv_fixup>;
// def V_LDEXP_F64 : VOP3Inst <"v_ldexp_f64", VOP3_Profile<VOP_F64_F64_I32>, PPUldexp, 1>;
} // End SchedRW = [WriteDoubleAdd], FPDPRounding = 1

def V_DIV_SCALE_F32 : VOP3_Pseudo <"v_div_scale_f32", VOP3b_F32_I1_F32_F32_F32, [], 1> {
  let SchedRW = [WriteFloatFMA, WriteSALU];
  let AsmMatchConverter = "";
}

// Double precision division pre-scale.
def V_DIV_SCALE_F64 : VOP3_Pseudo <"v_div_scale_f64", VOP3b_F64_I1_F64_F64_F64, [], 1> {
  let SchedRW = [WriteDouble, WriteSALU];
  let AsmMatchConverter = "";
  let FPDPRounding = 1;
}

def V_MSAD_U8 : VOP3Inst <"v_msad_u8", VOP3_Profile<VOP_I32_I32_I32_I32, VOP3_CLAMP>>;

let Constraints = "@earlyclobber $vdst" in {
def V_MQSAD_PK_U16_U8 : VOP3Inst <"v_mqsad_pk_u16_u8", VOP3_Profile<VOP_I64_I64_I32_I64, VOP3_CLAMP>>;
} // End Constraints = "@earlyclobber $vdst"

/*
def V_TRIG_PREOP_F64 : VOP3Inst <"v_trig_preop_f64", VOP3_Profile<VOP_F64_F64_I32>, PPUtrig_preop> {
  let SchedRW = [WriteDouble];
}
*/

let SchedRW = [Write64Bit] in {
let SubtargetPredicate = IsPPT in {
def V_LSHL_B64 : VOP3Inst <"v_lshl_b64", VOP3_Profile<VOP_PAT_GEN<VOP_I64_I64_I32>>, shl>;
def V_LSHR_B64 : VOP3Inst <"v_lshr_b64", VOP3_Profile<VOP_PAT_GEN<VOP_I64_I64_I32>>, srl>;
def V_ASHR_I64 : VOP3Inst <"v_ashr_i64", VOP3_Profile<VOP_PAT_GEN<VOP_I64_I64_I32>>, sra>;
def V_MULLIT_F32 : VOP3Inst <"v_mullit_f32", VOP3_Profile<VOP_F32_F32_F32_F32>>;

def V_LSHLREV_B64 : VOP3Inst <"v_lshlrev_b64", VOP3_Profile<VOP_I64_I32_I64>, lshl_rev>;
def V_LSHRREV_B64 : VOP3Inst <"v_lshrrev_b64", VOP3_Profile<VOP_I64_I32_I64>, lshr_rev>;
def V_ASHRREV_I64 : VOP3Inst <"v_ashrrev_i64", VOP3_Profile<VOP_I64_I32_I64>, ashr_rev>;
} // End SubtargetPredicate = isGFX8Plus
} // End SchedRW = [Write64Bit]

def : PPTPat <
 (getDivergentFrag<shl>.ret i64:$x, i32:$y),
 (V_LSHLREV_B64 $y, $x)
>;
def : PPTPat <
 (getDivergentFrag<srl>.ret i64:$x, i32:$y),
 (V_LSHRREV_B64 $y, $x)
>;
def : PPTPat <
 (getDivergentFrag<sra>.ret i64:$x, i32:$y),
 (V_ASHRREV_I64 $y, $x)
>;


let SchedRW = [Write32Bit] in {
def V_PERM_B32 : VOP3Inst <"v_perm_b32", VOP3_Profile<VOP_I32_I32_I32_I32>, PPUperm>;
} // End SchedRW = [Write32Bit]

let SubtargetPredicate = IsPPT in {

let Constraints = "@earlyclobber $vdst", SchedRW = [WriteQuarterRate32] in {
def V_QSAD_PK_U16_U8 : VOP3Inst <"v_qsad_pk_u16_u8", VOP3_Profile<VOP_I64_I64_I32_I64, VOP3_CLAMP>>;
// def V_MQSAD_U32_U8 : VOP3Inst <"v_mqsad_u32_u8", VOP3_Profile<VOP_V4I32_I64_I32_V4I32, VOP3_CLAMP>>;
} // End Constraints = "@earlyclobber $vdst", SchedRW = [WriteQuarterRate32]

let isCommutable = 1 in {
let SchedRW = [WriteQuarterRate32, WriteSALU] in {
def V_MAD_U64_U32 : VOP3Inst <"v_mad_u64_u32", VOP3b_I64_I1_I32_I32_I64>;
def V_MAD_I64_I32 : VOP3Inst <"v_mad_i64_i32", VOP3b_I64_I1_I32_I32_I64>;
} // End SchedRW = [WriteDouble, WriteSALU]
} // End isCommutable = 1

} // End SubtargetPredicate = isGFX7Plus

/*
def V_DIV_FIXUP_F16 : VOP3Inst <"v_div_fixup_f16", VOP3_Profile<VOP_F16_F16_F16_F16>, PPUdiv_fixup> {
  let Predicates = [Has16BitInsts, isGFX8Only];
  let FPDPRounding = 1;
}
*/
def V_DIV_FIXUP_F16_gfx9 : VOP3Inst <"v_div_fixup_f16_gfx9",
                                      VOP3_Profile<VOP_F16_F16_F16_F16, VOP3_OPSEL>, PPUdiv_fixup> {
  let Predicates = [Has16BitInsts, IsPPT];
  let FPDPRounding = 1;
}

def V_FMA_F16 : VOP3Inst <"v_fma_f16", VOP3_Profile<VOP_F16_F16_F16_F16>, fma> {
  let Predicates = [Has16BitInsts, IsPPT];
  let FPDPRounding = 1;
}

def V_FMA_F16_gfx9 : VOP3Inst <"v_fma_f16_gfx9", VOP3_Profile<VOP_F16_F16_F16_F16, VOP3_OPSEL>, fma> {
  let Predicates = [Has16BitInsts, IsPPT];
  let FPDPRounding = 1;
}

let SubtargetPredicate = Has16BitInsts, isCommutable = 1 in {

def V_MAD_U16 : VOP3Inst <"v_mad_u16", VOP3_Profile<VOP_I16_I16_I16_I16, VOP3_CLAMP>>;
def V_MAD_I16 : VOP3Inst <"v_mad_i16", VOP3_Profile<VOP_I16_I16_I16_I16, VOP3_CLAMP>>;
let FPDPRounding = 1 in {
def V_MAD_F16 : VOP3Inst <"v_mad_f16", VOP3_Profile<VOP_F16_F16_F16_F16>, fmad>;
} // End FPDPRounding = 1
/*
let SubtargetPredicate = isGFX9Only in {
def V_MAD_F16_gfx9   : VOP3Inst <"v_mad_f16_gfx9", VOP3_Profile<VOP_F16_F16_F16_F16, VOP3_OPSEL>> {
  let FPDPRounding = 1;
}
} // End SubtargetPredicate = isGFX9Only
*/

let SubtargetPredicate = IsPPT in {
def V_MAD_U16_gfx9   : VOP3Inst <"v_mad_u16_gfx9", VOP3_Profile<VOP_I16_I16_I16_I16, VOP3_OPSEL>>;
def V_MAD_I16_gfx9   : VOP3Inst <"v_mad_i16_gfx9", VOP3_Profile<VOP_I16_I16_I16_I16, VOP3_OPSEL>>;
} // End SubtargetPredicate = isGFX9Plus

} // End SubtargetPredicate = Has16BitInsts, isCommutable = 1

/*
let Predicates = [Has16BitInsts, isGFX6GFX7GFX8GFX9] in {

multiclass Ternary_i16_Pats <SDPatternOperator op1, SDPatternOperator op2,
                             Instruction inst, SDPatternOperator op3> {
def : PPTPat <
  (op2 (op1 i16:$src0, i16:$src1), i16:$src2),
  (inst i16:$src0, i16:$src1, i16:$src2, (i1 0))
>;

}

defm: Ternary_i16_Pats<mul, add, V_MAD_U16, zext>;
defm: Ternary_i16_Pats<mul, add, V_MAD_I16, sext>;

} // End Predicates = [Has16BitInsts, isGFX6GFX7GFX8GFX9]
*/

let Predicates = [Has16BitInsts] in {

multiclass Ternary_i16_Pats_gfx9<SDPatternOperator op1, SDPatternOperator op2,
                                 Instruction inst, SDPatternOperator op3> {
def : PPTPat <
  (op2 (op1 i16:$src0, i16:$src1), i16:$src2),
  (inst SRCMODS.NONE, $src0, SRCMODS.NONE, $src1, SRCMODS.NONE, $src2, DSTCLAMP.NONE)
>;

}

defm: Ternary_i16_Pats_gfx9<mul, add, V_MAD_U16_gfx9, zext>;
defm: Ternary_i16_Pats_gfx9<mul, add, V_MAD_I16_gfx9, sext>;

} // End Predicates = [Has16BitInsts, isGFX10Plus]

class ThreeOpFrag<SDPatternOperator op1, SDPatternOperator op2> : PatFrag<
  (ops node:$x, node:$y, node:$z),
  // When the inner operation is used multiple times, selecting 3-op
  // instructions may still be beneficial -- if the other users can be
  // combined similarly. Let's be conservative for now.
  (op2 (HasOneUseBinOp<op1> node:$x, node:$y), node:$z),
  [{
    // Only use VALU ops when the result is divergent.
    if (!N->isDivergent())
      return false;

    // Check constant bus limitations.
    //
    // Note: Use !isDivergent as a conservative proxy for whether the value
    //       is in an SGPR (uniform values can end up in VGPRs as well).
    unsigned ConstantBusUses = 0;
    for (unsigned i = 0; i < 3; ++i) {
      if (!Operands[i]->isDivergent() &&
          !isInlineImmediate(Operands[i].getNode())) {
        ConstantBusUses++;
        // This uses PPU::V_ADD3_U32, but all three operand instructions
        // have the same constant bus limit.
        if (ConstantBusUses > Subtarget->getConstantBusLimit(PPU::V_ADD3_U32))
          return false;
      }
    }

    return true;
  }]
> {
  let PredicateCodeUsesOperands = 1;
}

let SubtargetPredicate = IsPPT in {
def V_PACK_B32_F16 : VOP3Inst <"v_pack_b32_f16", VOP3_Profile<VOP_B32_F16_F16, VOP3_OPSEL>>;
def V_LSHL_ADD_U32 : VOP3Inst <"v_lshl_add_u32", VOP3_Profile<VOP_I32_I32_I32_I32>>;
def V_ADD_LSHL_U32 : VOP3Inst <"v_add_lshl_u32", VOP3_Profile<VOP_I32_I32_I32_I32>>;
def V_ADD3_U32 : VOP3Inst <"v_add3_u32", VOP3_Profile<VOP_I32_I32_I32_I32>>;
def V_LSHL_OR_B32 : VOP3Inst <"v_lshl_or_b32", VOP3_Profile<VOP_I32_I32_I32_I32>>;
def V_AND_OR_B32 : VOP3Inst <"v_and_or_b32", VOP3_Profile<VOP_I32_I32_I32_I32>>;
def V_OR3_B32 : VOP3Inst <"v_or3_b32", VOP3_Profile<VOP_I32_I32_I32_I32>>;

def V_XAD_U32 : VOP3Inst <"v_xad_u32", VOP3_Profile<VOP_I32_I32_I32_I32>>;

def V_MED3_F16 : VOP3Inst <"v_med3_f16", VOP3_Profile<VOP_F16_F16_F16_F16, VOP3_OPSEL>, PPUfmed3>;
def V_MED3_I16 : VOP3Inst <"v_med3_i16", VOP3_Profile<VOP_I16_I16_I16_I16, VOP3_OPSEL>, PPUsmed3>;
def V_MED3_U16 : VOP3Inst <"v_med3_u16", VOP3_Profile<VOP_I16_I16_I16_I16, VOP3_OPSEL>, PPUumed3>;

def V_MIN3_F16 : VOP3Inst <"v_min3_f16", VOP3_Profile<VOP_F16_F16_F16_F16, VOP3_OPSEL>, PPUfmin3>;
def V_MIN3_I16 : VOP3Inst <"v_min3_i16", VOP3_Profile<VOP_I16_I16_I16_I16, VOP3_OPSEL>, PPUsmin3>;
def V_MIN3_U16 : VOP3Inst <"v_min3_u16", VOP3_Profile<VOP_I16_I16_I16_I16, VOP3_OPSEL>, PPUumin3>;

def V_MAX3_F16 : VOP3Inst <"v_max3_f16", VOP3_Profile<VOP_F16_F16_F16_F16, VOP3_OPSEL>, PPUfmax3>;
def V_MAX3_I16 : VOP3Inst <"v_max3_i16", VOP3_Profile<VOP_I16_I16_I16_I16, VOP3_OPSEL>, PPUsmax3>;
def V_MAX3_U16 : VOP3Inst <"v_max3_u16", VOP3_Profile<VOP_I16_I16_I16_I16, VOP3_OPSEL>, PPUumax3>;

def V_ADD_I16 : VOP3Inst <"v_add_i16", VOP3_Profile<VOP_I16_I16_I16, VOP3_OPSEL>>;
def V_SUB_I16 : VOP3Inst <"v_sub_i16", VOP3_Profile<VOP_I16_I16_I16, VOP3_OPSEL>>;

def V_MAD_U32_U16 : VOP3Inst <"v_mad_u32_u16", VOP3_Profile<VOP_I32_I16_I16_I32, VOP3_OPSEL>>;
def V_MAD_I32_I16 : VOP3Inst <"v_mad_i32_i16", VOP3_Profile<VOP_I32_I16_I16_I32, VOP3_OPSEL>>;

def V_CVT_PKNORM_I16_F16 : VOP3Inst <"v_cvt_pknorm_i16_f16", VOP3_Profile<VOP_B32_F16_F16, VOP3_OPSEL>>;
def V_CVT_PKNORM_U16_F16 : VOP3Inst <"v_cvt_pknorm_u16_f16", VOP3_Profile<VOP_B32_F16_F16, VOP3_OPSEL>>;

def V_ADD_I32_gfx9 : VOP3Inst <"v_add_i32_gfx9", VOP3_Profile<VOP_I32_I32_I32>>;
def V_SUB_I32_gfx9 : VOP3Inst <"v_sub_i32_gfx9", VOP3_Profile<VOP_I32_I32_I32>>;


class ThreeOp_i32_Pats <SDPatternOperator op1, SDPatternOperator op2, Instruction inst> : PPTPat <
  // This matches (op2 (op1 i32:$src0, i32:$src1), i32:$src2) with conditions.
  (ThreeOpFrag<op1, op2> i32:$src0, i32:$src1, i32:$src2),
  (inst i32:$src0, i32:$src1, i32:$src2)
>;

def : ThreeOp_i32_Pats<shl, add, V_LSHL_ADD_U32>;
def : ThreeOp_i32_Pats<add, shl, V_ADD_LSHL_U32>;
def : ThreeOp_i32_Pats<add, add, V_ADD3_U32>;
def : ThreeOp_i32_Pats<shl, or, V_LSHL_OR_B32>;
def : ThreeOp_i32_Pats<and, or, V_AND_OR_B32>;
def : ThreeOp_i32_Pats<or, or, V_OR3_B32>;
def : ThreeOp_i32_Pats<xor, add, V_XAD_U32>;

} // End SubtargetPredicate = isGFX9Plus

def VOP3_PERMLANE_Profile : VOP3_Profile<VOPProfile <[i32, i32, i32, i32]>, VOP3_OPSEL> {
  let Src0RC64 = VRegSrc_32;
  let Src1RC64 = SCSrc_b32;
  let Src2RC64 = SCSrc_b32;
  let InsVOP3OpSel = (ins IntOpSelMods:$src0_modifiers, VRegSrc_32:$src0,
                          IntOpSelMods:$src1_modifiers, SCSrc_b32:$src1,
                          IntOpSelMods:$src2_modifiers, SCSrc_b32:$src2,
                          VPR_32:$vdst_in, op_sel:$op_sel);
  let HasClamp = 0;
  let HasOMod = 0;
}

let SubtargetPredicate = IsPPT in {
  def V_XOR3_B32 : VOP3Inst <"v_xor3_b32", VOP3_Profile<VOP_I32_I32_I32_I32>>;
  def : ThreeOp_i32_Pats<xor, xor, V_XOR3_B32>;

  let Constraints = "$vdst = $vdst_in", DisableEncoding="$vdst_in" in {
    def V_PERMLANE16_B32 : VOP3Inst <"v_permlane16_b32", VOP3_PERMLANE_Profile>;
    def V_PERMLANEX16_B32 : VOP3Inst <"v_permlanex16_b32", VOP3_PERMLANE_Profile>;
  } // End $vdst = $vdst_in, DisableEncoding $vdst_in

  def : PPTPat<
    (int_ppu_permlane16 i32:$vdst_in, i32:$src0, i32:$src1, i32:$src2, imm:$fi, imm:$bc),
    (V_PERMLANE16_B32 (as_i1imm $fi), $src0, (as_i1imm $bc), $src1, 0, $src2, $vdst_in)
  >;
  def : PPTPat<
    (int_ppu_permlanex16 i32:$vdst_in, i32:$src0, i32:$src1, i32:$src2, imm:$fi, imm:$bc),
    (V_PERMLANEX16_B32 (as_i1imm $fi), $src0, (as_i1imm $bc), $src1, 0, $src2, $vdst_in)
  >;
} // End SubtargetPredicate = isGFX10Plus

//===----------------------------------------------------------------------===//
// Integer Clamp Patterns
//===----------------------------------------------------------------------===//

class getClampPat<VOPProfile P, SDPatternOperator node> {
  dag ret3 = (P.DstVT (node P.Src0VT:$src0, P.Src1VT:$src1, P.Src2VT:$src2));
  dag ret2 = (P.DstVT (node P.Src0VT:$src0, P.Src1VT:$src1));
  dag ret1 = (P.DstVT (node P.Src0VT:$src0));
  dag ret = !if(!eq(P.NumSrcArgs, 3), ret3,
            !if(!eq(P.NumSrcArgs, 2), ret2,
            ret1));
}

class getClampRes<VOPProfile P, Instruction inst> {
  dag ret3 = (inst P.Src0VT:$src0, P.Src1VT:$src1, P.Src2VT:$src2, (i1 0));
  dag ret2 = (inst P.Src0VT:$src0, P.Src1VT:$src1, (i1 0));
  dag ret1 = (inst P.Src0VT:$src0, (i1 0));
  dag ret = !if(!eq(P.NumSrcArgs, 3), ret3,
            !if(!eq(P.NumSrcArgs, 2), ret2,
            ret1));
}

class IntClampPat<VOP3Inst inst, SDPatternOperator node> : PPTPat<
  getClampPat<inst.Pfl, node>.ret,
  getClampRes<inst.Pfl, inst>.ret
>;

def : IntClampPat<V_MAD_I32_I24, PPUmad_i24>;
def : IntClampPat<V_MAD_U32_U24, PPUmad_u24>;

def : IntClampPat<V_SAD_U8, int_ppu_sad_u8>;
def : IntClampPat<V_SAD_HI_U8, int_ppu_sad_hi_u8>;
def : IntClampPat<V_SAD_U16, int_ppu_sad_u16>;

def : IntClampPat<V_MSAD_U8, int_ppu_msad_u8>;
def : IntClampPat<V_MQSAD_PK_U16_U8, int_ppu_mqsad_pk_u16_u8>;

def : IntClampPat<V_QSAD_PK_U16_U8, int_ppu_qsad_pk_u16_u8>;
// def : IntClampPat<V_MQSAD_U32_U8, int_ppu_mqsad_u32_u8>;


//===----------------------------------------------------------------------===//
// Target-specific instruction encodings.
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
// GFX10.
//===----------------------------------------------------------------------===//

let AssemblerPredicate = isGFX10Plus, DecoderNamespace = "GFX10" in {
  multiclass VOP3_Real_gfx10<bits<10> op> {
    def _gfx10 :
      VOP3_Real<!cast<VOP_Pseudo>(NAME), PPUEncodingFamily.PPU>,
      VOP3e_ppu<op, !cast<VOP_Pseudo>(NAME).Pfl>;
  }
  multiclass VOP3_Real_gfx10_with_name<bits<10> op, string opName,
                                       string asmName> {
    def _gfx10 :
      VOP3_Real<!cast<VOP3_Pseudo>(opName), PPUEncodingFamily.PPU>,
      VOP3e_ppu<op, !cast<VOP3_Pseudo>(opName).Pfl> {
        VOP3_Pseudo ps = !cast<VOP3_Pseudo>(opName);
        let AsmString = asmName # ps.AsmOperands;
      }
  }
  multiclass VOP3be_Real_gfx10<bits<10> op> {
    def _gfx10 :
      VOP3_Real<!cast<VOP3_Pseudo>(NAME), PPUEncodingFamily.PPU>,
      VOP3be_ppu<op, !cast<VOP3_Pseudo>(NAME).Pfl>;
  }
  multiclass VOP3OpSel_Real_gfx10<bits<10> op> {
    def _gfx10 :
      VOP3_Real<!cast<VOP3_Pseudo>(NAME), PPUEncodingFamily.PPU>,
      VOP3OpSel_gfx10<op, !cast<VOP3_Pseudo>(NAME).Pfl>;
  }
  multiclass VOP3OpSel_Real_gfx10_with_name<bits<10> op, string opName,
                                            string asmName> {
    def _gfx10 :
      VOP3_Real<!cast<VOP3_Pseudo>(opName), PPUEncodingFamily.PPU>,
      VOP3OpSel_gfx10<op, !cast<VOP3_Pseudo>(opName).Pfl> {
        VOP3_Pseudo ps = !cast<VOP3_Pseudo>(opName);
        let AsmString = asmName # ps.AsmOperands;
      }
  }
} // End AssemblerPredicate = isGFX10Plus, DecoderNamespace = "GFX10"



// These are special and do not read the exec mask.
let isConvergent = 1, Uses = []<Register> in {

def V_READLANE_B32 : VOP2_Pseudo<"v_readlane_b32", VOP_READLANE,
  [(set i32:$vdst, (int_ppu_readlane i32:$src0, i32:$src1))]>;


let Constraints = "$vdst = $vdst_in", DisableEncoding="$vdst_in" in {
def V_WRITELANE_B32 : VOP2_Pseudo<"v_writelane_b32", VOP_WRITELANE,
  [(set i32:$vdst, (int_ppu_writelane i32:$src0, i32:$src1, i32:$vdst_in))]>;
} // End $vdst = $vdst_in, DisableEncoding $vdst_in
} // End isConvergent = 1


defm V_READLANE_B32  : VOP3_Real_gfx10<0x360>;

// FIXME let InOperandList = (ins SSrcOrLds_b32:$src0, SCSrc_b32:$src1, VSrc_b32:$vdst_in) in {
//  defm V_WRITELANE_B32 : VOP3_Real_gfx10<0x361>;
// } // End InOperandList = (ins SSrcOrLds_b32:$src0, SCSrc_b32:$src1, VSrc_b32:$vdst_in)

defm V_XOR3_B32           : VOP3_Real_gfx10<0x178>;
defm V_LSHLREV_B64        : VOP3_Real_gfx10<0x2ff>;
defm V_LSHRREV_B64        : VOP3_Real_gfx10<0x300>;
defm V_ASHRREV_I64        : VOP3_Real_gfx10<0x301>;
defm V_PERM_B32           : VOP3_Real_gfx10<0x344>;
defm V_XAD_U32            : VOP3_Real_gfx10<0x345>;
defm V_LSHL_ADD_U32       : VOP3_Real_gfx10<0x346>;
defm V_ADD_LSHL_U32       : VOP3_Real_gfx10<0x347>;
defm V_ADD3_U32           : VOP3_Real_gfx10<0x36d>;
defm V_LSHL_OR_B32        : VOP3_Real_gfx10<0x36f>;
defm V_AND_OR_B32         : VOP3_Real_gfx10<0x371>;
defm V_OR3_B32            : VOP3_Real_gfx10<0x372>;

// TODO-GFX10: add MC tests for v_add/sub_nc_i16
defm V_ADD_NC_I16 :
  VOP3OpSel_Real_gfx10_with_name<0x30d, "V_ADD_I16", "v_add_nc_i16">;
defm V_SUB_NC_I16 :
  VOP3OpSel_Real_gfx10_with_name<0x30e, "V_SUB_I16", "v_sub_nc_i16">;
defm V_SUB_NC_I32 :
  VOP3_Real_gfx10_with_name<0x376, "V_SUB_I32_gfx9", "v_sub_nc_i32">;
defm V_ADD_NC_I32 :
  VOP3_Real_gfx10_with_name<0x37f, "V_ADD_I32_gfx9", "v_add_nc_i32">;

defm V_PACK_B32_F16       : VOP3OpSel_Real_gfx10<0x311>;
defm V_CVT_PKNORM_I16_F16 : VOP3OpSel_Real_gfx10<0x312>;
defm V_CVT_PKNORM_U16_F16 : VOP3OpSel_Real_gfx10<0x313>;

defm V_MIN3_F16           : VOP3OpSel_Real_gfx10<0x351>;
defm V_MIN3_I16           : VOP3OpSel_Real_gfx10<0x352>;
defm V_MIN3_U16           : VOP3OpSel_Real_gfx10<0x353>;
defm V_MAX3_F16           : VOP3OpSel_Real_gfx10<0x354>;
defm V_MAX3_I16           : VOP3OpSel_Real_gfx10<0x355>;
defm V_MAX3_U16           : VOP3OpSel_Real_gfx10<0x356>;
defm V_MED3_F16           : VOP3OpSel_Real_gfx10<0x357>;
defm V_MED3_I16           : VOP3OpSel_Real_gfx10<0x358>;
defm V_MED3_U16           : VOP3OpSel_Real_gfx10<0x359>;
defm V_MAD_U32_U16        : VOP3OpSel_Real_gfx10<0x373>;
defm V_MAD_I32_I16        : VOP3OpSel_Real_gfx10<0x375>;

defm V_MAD_U16 :
  VOP3OpSel_Real_gfx10_with_name<0x340, "V_MAD_U16_gfx9", "v_mad_u16">;
defm V_FMA_F16 :
  VOP3OpSel_Real_gfx10_with_name<0x34b, "V_FMA_F16_gfx9", "v_fma_f16">;
defm V_MAD_I16 :
  VOP3OpSel_Real_gfx10_with_name<0x35e, "V_MAD_I16_gfx9", "v_mad_i16">;
defm V_DIV_FIXUP_F16 :
  VOP3OpSel_Real_gfx10_with_name<0x35f, "V_DIV_FIXUP_F16_gfx9", "v_div_fixup_f16">;

// FIXME-GFX10-OPSEL: Need to add "selective" opsel support to some of these
// (they do not support SDWA or DPP).
// defm V_ADD_NC_U16      : VOP3_Real_gfx10_with_name<0x303, "V_ADD_U16_e64", "v_add_nc_u16">;
// defm V_SUB_NC_U16      : VOP3_Real_gfx10_with_name<0x304, "V_SUB_U16_e64", "v_sub_nc_u16">;
// defm V_MUL_LO_U16      : VOP3_Real_gfx10_with_name<0x305, "V_MUL_LO_U16_e64", "v_mul_lo_u16">;
defm V_LSHRREV_B16     : VOP3_Real_gfx10_with_name<0x307, "V_LSHRREV_B16_e64", "v_lshrrev_b16">;
defm V_ASHRREV_I16     : VOP3_Real_gfx10_with_name<0x308, "V_ASHRREV_I16_e64", "v_ashrrev_i16">;
defm V_MAX_U16         : VOP3_Real_gfx10_with_name<0x309, "V_MAX_U16_e64", "v_max_u16">;
defm V_MAX_I16         : VOP3_Real_gfx10_with_name<0x30a, "V_MAX_I16_e64", "v_max_i16">;
defm V_MIN_U16         : VOP3_Real_gfx10_with_name<0x30b, "V_MIN_U16_e64", "v_min_u16">;
defm V_MIN_I16         : VOP3_Real_gfx10_with_name<0x30c, "V_MIN_I16_e64", "v_min_i16">;
defm V_LSHLREV_B16     : VOP3_Real_gfx10_with_name<0x314, "V_LSHLREV_B16_e64", "v_lshlrev_b16">;
defm V_PERMLANE16_B32  : VOP3OpSel_Real_gfx10<0x377>;
defm V_PERMLANEX16_B32 : VOP3OpSel_Real_gfx10<0x378>;

//===----------------------------------------------------------------------===//
// GFX7, GFX10.
//===----------------------------------------------------------------------===//

multiclass VOP3_Real_gfx7_gfx10<bits<10> op> :
  VOP3_Real_gfx10<op>;

multiclass VOP3be_Real_gfx7_gfx10<bits<10> op> :
  VOP3be_Real_gfx10<op>;

defm V_QSAD_PK_U16_U8   : VOP3_Real_gfx7_gfx10<0x172>;
// defm V_MQSAD_U32_U8     : VOP3_Real_gfx7_gfx10<0x175>;
defm V_MAD_U64_U32      : VOP3be_Real_gfx7_gfx10<0x176>;
defm V_MAD_I64_I32      : VOP3be_Real_gfx7_gfx10<0x177>;

//===----------------------------------------------------------------------===//
// GFX6, GFX7, GFX10.
//===----------------------------------------------------------------------===//

multiclass VOP3_Real_gfx6_gfx7_gfx10<bits<10> op> :
  VOP3_Real_gfx10<op>;

multiclass VOP3be_Real_gfx6_gfx7_gfx10<bits<10> op> :
  VOP3be_Real_gfx10<op>;

// defm V_MAD_LEGACY_F32  : VOP3_Real_gfx6_gfx7_gfx10<0x140>;
defm V_MAD_F32         : VOP3_Real_gfx6_gfx7_gfx10<0x141>;
defm V_MAD_I32_I24     : VOP3_Real_gfx6_gfx7_gfx10<0x142>;
defm V_MAD_U32_U24     : VOP3_Real_gfx6_gfx7_gfx10<0x143>;
// defm V_CUBEID_F32      : VOP3_Real_gfx6_gfx7_gfx10<0x144>;
// defm V_CUBESC_F32      : VOP3_Real_gfx6_gfx7_gfx10<0x145>;
// defm V_CUBETC_F32      : VOP3_Real_gfx6_gfx7_gfx10<0x146>;
// defm V_CUBEMA_F32      : VOP3_Real_gfx6_gfx7_gfx10<0x147>;
defm V_BFE_U32         : VOP3_Real_gfx6_gfx7_gfx10<0x148>;
defm V_BFE_I32         : VOP3_Real_gfx6_gfx7_gfx10<0x149>;
defm V_BFI_B32         : VOP3_Real_gfx6_gfx7_gfx10<0x14a>;
defm V_FMA_F32         : VOP3_Real_gfx6_gfx7_gfx10<0x14b>;
// defm V_FMA_F64         : VOP3_Real_gfx6_gfx7_gfx10<0x14c>;
// defm V_LERP_U8         : VOP3_Real_gfx6_gfx7_gfx10<0x14d>;
defm V_ALIGNBIT_B32    : VOP3_Real_gfx6_gfx7_gfx10<0x14e>;
defm V_ALIGNBYTE_B32   : VOP3_Real_gfx6_gfx7_gfx10<0x14f>;
defm V_MULLIT_F32      : VOP3_Real_gfx6_gfx7_gfx10<0x150>;
defm V_MIN3_F32        : VOP3_Real_gfx6_gfx7_gfx10<0x151>;
defm V_MIN3_I32        : VOP3_Real_gfx6_gfx7_gfx10<0x152>;
defm V_MIN3_U32        : VOP3_Real_gfx6_gfx7_gfx10<0x153>;
defm V_MAX3_F32        : VOP3_Real_gfx6_gfx7_gfx10<0x154>;
defm V_MAX3_I32        : VOP3_Real_gfx6_gfx7_gfx10<0x155>;
defm V_MAX3_U32        : VOP3_Real_gfx6_gfx7_gfx10<0x156>;
defm V_MED3_F32        : VOP3_Real_gfx6_gfx7_gfx10<0x157>;
defm V_MED3_I32        : VOP3_Real_gfx6_gfx7_gfx10<0x158>;
defm V_MED3_U32        : VOP3_Real_gfx6_gfx7_gfx10<0x159>;
defm V_SAD_U8          : VOP3_Real_gfx6_gfx7_gfx10<0x15a>;
defm V_SAD_HI_U8       : VOP3_Real_gfx6_gfx7_gfx10<0x15b>;
defm V_SAD_U16         : VOP3_Real_gfx6_gfx7_gfx10<0x15c>;
defm V_SAD_U32         : VOP3_Real_gfx6_gfx7_gfx10<0x15d>;
defm V_CVT_PK_U8_F32   : VOP3_Real_gfx6_gfx7_gfx10<0x15e>;
defm V_DIV_FIXUP_F32   : VOP3_Real_gfx6_gfx7_gfx10<0x15f>;
// defm V_DIV_FIXUP_F64   : VOP3_Real_gfx6_gfx7_gfx10<0x160>;
// defm V_ADD_F64         : VOP3_Real_gfx6_gfx7_gfx10<0x164>;
// defm V_MUL_F64         : VOP3_Real_gfx6_gfx7_gfx10<0x165>;
// defm V_MIN_F64         : VOP3_Real_gfx6_gfx7_gfx10<0x166>;
// defm V_MAX_F64         : VOP3_Real_gfx6_gfx7_gfx10<0x167>;
// defm V_LDEXP_F64       : VOP3_Real_gfx6_gfx7_gfx10<0x168>;
defm V_MUL_LO_U32      : VOP3_Real_gfx6_gfx7_gfx10<0x169>;
defm V_MUL_HI_U32      : VOP3_Real_gfx6_gfx7_gfx10<0x16a>;
defm V_MUL_LO_I32      : VOP3_Real_gfx6_gfx7_gfx10<0x16b>;
defm V_MUL_HI_I32      : VOP3_Real_gfx6_gfx7_gfx10<0x16c>;
defm V_DIV_FMAS_F32    : VOP3_Real_gfx6_gfx7_gfx10<0x16f>;
defm V_DIV_FMAS_F64    : VOP3_Real_gfx6_gfx7_gfx10<0x170>;
defm V_MSAD_U8         : VOP3_Real_gfx6_gfx7_gfx10<0x171>;
defm V_MQSAD_PK_U16_U8 : VOP3_Real_gfx6_gfx7_gfx10<0x173>;
// defm V_TRIG_PREOP_F64  : VOP3_Real_gfx6_gfx7_gfx10<0x174>;
defm V_DIV_SCALE_F32   : VOP3be_Real_gfx6_gfx7_gfx10<0x16d>;
defm V_DIV_SCALE_F64   : VOP3be_Real_gfx6_gfx7_gfx10<0x16e>;

