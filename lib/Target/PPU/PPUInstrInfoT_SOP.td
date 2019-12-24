
let isMoveImm = 1 in {
  let isReMaterializable = 1, isAsCheapAsAMove = 1 in {
    def S_MOV_B32 : SOP1_32 <"s_mov_b32">;
    def S_MOV_B64 : SOP1_64 <"s_mov_b64">;
  } // End isRematerializeable = 1

  let Uses = [SCC] in {
    def S_CMOV_B32 : SOP1_32 <"s_cmov_b32">;
    def S_CMOV_B64 : SOP1_64 <"s_cmov_b64">;
  } // End Uses = [SCC]
} // End isMoveImm = 1

let Defs = [SCC] in {
  def S_NOT_B32 : SOP1_32 <"s_not_b32",
    [(set i32:$sdst, (not i32:$src0))]
  >;

  def S_NOT_B64 : SOP1_64 <"s_not_b64",
    [(set i64:$sdst, (not i64:$src0))]
  >;
  def S_WQM_B32 : SOP1_32 <"s_wqm_b32">;
  def S_WQM_B64 : SOP1_64 <"s_wqm_b64">;
} // End Defs = [SCC]

def S_BREV_B32 : SOP1_32 <"s_brev_b32",
  [(set i32:$sdst, (bitreverse i32:$src0))]
>;

let Defs = [SCC] in {
def S_BCNT0_I32_B32 : SOP1_32 <"s_bcnt0_i32_b32">;
// def S_BCNT0_I32_B64 : SOP1_32_64 <"s_bcnt0_i32_b64">;
def S_BCNT1_I32_B32 : SOP1_32 <"s_bcnt1_i32_b32",
  [(set i32:$sdst, (ctpop i32:$src0))]
>;
// def S_BCNT1_I32_B64 : SOP1_32_64 <"s_bcnt1_i32_b64">;
} // End Defs = [SCC]

def S_FF0_I32_B32 : SOP1_32 <"s_ff0_i32_b32">;
// def S_FF0_I32_B64 : SOP1_32_64 <"s_ff0_i32_b64">;
// def S_FF1_I32_B64 : SOP1_32_64 <"s_ff1_i32_b64">;

def S_FF1_I32_B32 : SOP1_32 <"s_ff1_i32_b32",
  [(set i32:$sdst, (PPUffbl_b32 i32:$src0))]
>;

def S_FLBIT_I32_B32 : SOP1_32 <"s_flbit_i32_b32",
  [(set i32:$sdst, (PPUffbh_u32 i32:$src0))]
>;

// def S_FLBIT_I32_B64 : SOP1_32_64 <"s_flbit_i32_b64">;
def S_FLBIT_I32 : SOP1_32 <"s_flbit_i32",
  [(set i32:$sdst, (PPUffbh_i32 i32:$src0))]
>;
// def S_FLBIT_I32_I64 : SOP1_32_64 <"s_flbit_i32_i64">;
def S_SEXT_I32_I8 : SOP1_32 <"s_sext_i32_i8",
  [(set i32:$sdst, (sext_inreg i32:$src0, i8))]
>;
def S_SEXT_I32_I16 : SOP1_32 <"s_sext_i32_i16",
  [(set i32:$sdst, (sext_inreg i32:$src0, i16))]
>;

def S_BITSET0_B32 : SOP1_32    <"s_bitset0_b32", [], 1>;
// def S_BITSET0_B64 : SOP1_64_32 <"s_bitset0_b64", [], 1>;
def S_BITSET1_B32 : SOP1_32    <"s_bitset1_b32", [], 1>;
// def S_BITSET1_B64 : SOP1_64_32 <"s_bitset1_b64", [], 1>;
def S_GETPC_B64 : SOP1_64_0  <"s_getpc_b64",
  [(set i64:$sdst, (int_amdgcn_s_getpc))]
>;


let isTerminator = 1, isBarrier = 1, SchedRW = [WriteBranch] in {

let isBranch = 1, isIndirectBranch = 1 in {
def S_SETPC_B64 : SOP1_0_64 <"s_setpc_b64">;
} // End isBranch = 1, isIndirectBranch = 1

let isReturn = 1 in {
// Define variant marked as return rather than branch.
def S_SETPC_B64_return : SOP1_0_64<"", CCR_SPR_64, [(PPUret_flag i64:$src0)]>;
}
} // End isTerminator = 1, isBarrier = 1

let isCall = 1 in {
def S_SWAPPC_B64 : SOP1_64 <"s_swappc_b64"
>;
}

let hasSideEffects = 1, Uses = [TMSK], Defs = [TMSK, SCC] in {

def S_AND_SAVETMSK_B64 : SOP1_64 <"s_and_savetmsk_b64">;
def S_OR_SAVETMSK_B64 : SOP1_64 <"s_or_savetmsk_b64">;
def S_XOR_SAVETMSK_B64 : SOP1_64 <"s_xor_savetmsk_b64">;
def S_ANDN2_SAVETMSK_B64 : SOP1_64 <"s_andn2_savetmsk_b64">;
def S_ORN2_SAVETMSK_B64 : SOP1_64 <"s_orn2_savetmsk_b64">;
def S_NAND_SAVETMSK_B64 : SOP1_64 <"s_nand_savetmsk_b64">;
def S_NOR_SAVETMSK_B64 : SOP1_64 <"s_nor_savetmsk_b64">;
def S_XNOR_SAVETMSK_B64 : SOP1_64 <"s_xnor_savetmsk_b64">;

} // End hasSideEffects = 1, Uses = [TMSK], Defs = [TMSK, SCC]

//  def S_BITREPLICATE_B64_B32 : SOP1_64_32<"s_bitreplicate_b64_b32">;

  let hasSideEffects = 1, Defs = [TMSK, SCC], Uses = [TMSK] in {
    def S_ANDN1_SAVETMSK_B64 : SOP1_64<"s_andn1_saveexec_b64">;
    def S_ORN1_SAVETMSK_B64  : SOP1_64<"s_orn1_saveexec_b64">;
    def S_ANDN1_WRTMSK_B64   : SOP1_64<"s_andn1_wrexec_b64">;
    def S_ANDN2_WRTMSK_B64   : SOP1_64<"s_andn2_wrexec_b64">;

    def S_AND_SAVETMSK_B32   : SOP1_32<"s_and_saveexec_b32">;
    def S_OR_SAVETMSK_B32    : SOP1_32<"s_or_saveexec_b32">;
    def S_XOR_SAVETMSK_B32   : SOP1_32<"s_xor_saveexec_b32">;
    def S_ANDN2_SAVETMSK_B32 : SOP1_32<"s_andn2_saveexec_b32">;
    def S_ORN2_SAVETMSK_B32  : SOP1_32<"s_orn2_saveexec_b32">;
    def S_NAND_SAVETMSK_B32  : SOP1_32<"s_nand_saveexec_b32">;
    def S_NOR_SAVETMSK_B32   : SOP1_32<"s_nor_saveexec_b32">;
    def S_XNOR_SAVETMSK_B32  : SOP1_32<"s_xnor_saveexec_b32">;
    def S_ANDN1_SAVETMSK_B32 : SOP1_32<"s_andn1_saveexec_b32">;
    def S_ORN1_SAVETMSK_B32  : SOP1_32<"s_orn1_saveexec_b32">;
    def S_ANDN1_WRTMSK_B32   : SOP1_32<"s_andn1_wrexec_b32">;
    def S_ANDN2_WRTMSK_B32   : SOP1_32<"s_andn2_wrexec_b32">;
  } // End hasSideEffects = 1, Defs = [TMSK, SCC], Uses = [TMSK]

  let Uses = [M0] in {
    def S_MOVRELSD_2_B32 : SOP1_32<"s_movrelsd_2_b32">;
  } // End Uses = [M0]


let Defs = [SCC] in {
def S_ABS_I32 : SOP1_32 <"s_abs_i32">;
} // End Defs = [SCC]

let SubtargetPredicate = HasVPRIndexMode in {
def S_SET_GPR_IDX_IDX : SOP1_0_32<"s_set_gpr_idx_idx"> {
  let Uses = [M0];
  let Defs = [M0];
}
}


//===--------------------------------------------------
// SOP2
//===--------------------------------------------------
let Defs = [SCC] in { // Carry out goes to SCC
let isCommutable = 1 in {
def S_ADD_U32 : SOP2_32 <"s_add_u32">;
def S_ADD_I32 : SOP2_32 <"s_add_i32",
  [(set i32:$sdst, (UniformBinFrag<add> SSrc_b32:$src0, SSrc_b32:$src1))]
>;
} // End isCommutable = 1

def S_SUB_U32 : SOP2_32 <"s_sub_u32">;
def S_SUB_I32 : SOP2_32 <"s_sub_i32",
  [(set i32:$sdst, (UniformBinFrag<sub> SSrc_b32:$src0, SSrc_b32:$src1))]
>;

let Uses = [SCC] in { // Carry in comes from SCC
let isCommutable = 1 in {
def S_ADDC_U32 : SOP2_32 <"s_addc_u32",
  [(set i32:$sdst, (UniformBinFrag<adde> (i32 SSrc_b32:$src0), (i32 SSrc_b32:$src1)))]>;
} // End isCommutable = 1

def S_SUBB_U32 : SOP2_32 <"s_subb_u32",
  [(set i32:$sdst, (UniformBinFrag<sube> (i32 SSrc_b32:$src0), (i32 SSrc_b32:$src1)))]>;
} // End Uses = [SCC]


let isCommutable = 1 in {
def S_MIN_I32 : SOP2_32 <"s_min_i32",
  [(set i32:$sdst, (UniformBinFrag<smin> i32:$src0, i32:$src1))]
>;
def S_MIN_U32 : SOP2_32 <"s_min_u32",
  [(set i32:$sdst, (UniformBinFrag<umin> i32:$src0, i32:$src1))]
>;
def S_MAX_I32 : SOP2_32 <"s_max_i32",
  [(set i32:$sdst, (UniformBinFrag<smax> i32:$src0, i32:$src1))]
>;
def S_MAX_U32 : SOP2_32 <"s_max_u32",
  [(set i32:$sdst, (UniformBinFrag<umax> i32:$src0, i32:$src1))]
>;
} // End isCommutable = 1
} // End Defs = [SCC]

let Uses = [SCC] in {
  def S_CSELECT_B32 : SOP2_32 <"s_cselect_b32">;
  def S_CSELECT_B64 : SOP2_64 <"s_cselect_b64">;
} // End Uses = [SCC]

let Defs = [SCC] in {
let isCommutable = 1 in {
def S_AND_B32 : SOP2_32 <"s_and_b32",
  [(set i32:$sdst, (UniformBinFrag<and> i32:$src0, i32:$src1))]
>;

def S_AND_B64 : SOP2_64 <"s_and_b64",
  [(set i64:$sdst, (UniformBinFrag<and> i64:$src0, i64:$src1))]
>;

def S_OR_B32 : SOP2_32 <"s_or_b32",
  [(set i32:$sdst, (UniformBinFrag<or> i32:$src0, i32:$src1))]
>;

def S_OR_B64 : SOP2_64 <"s_or_b64",
  [(set i64:$sdst, (UniformBinFrag<or> i64:$src0, i64:$src1))]
>;

def S_XOR_B32 : SOP2_32 <"s_xor_b32",
  [(set i32:$sdst, (UniformBinFrag<xor> i32:$src0, i32:$src1))]
>;

def S_XOR_B64 : SOP2_64 <"s_xor_b64",
  [(set i64:$sdst, (UniformBinFrag<xor> i64:$src0, i64:$src1))]
>;

def S_XNOR_B32 : SOP2_32 <"s_xnor_b32",
  [(set i32:$sdst, (not (xor_oneuse i32:$src0, i32:$src1)))]
>;

def S_XNOR_B64 : SOP2_64 <"s_xnor_b64",
  [(set i64:$sdst, (not (xor_oneuse i64:$src0, i64:$src1)))]
>;

def S_NAND_B32 : SOP2_32 <"s_nand_b32",
  [(set i32:$sdst, (not (and_oneuse i32:$src0, i32:$src1)))] >;

def S_NAND_B64 : SOP2_64 <"s_nand_b64",
  [(set i64:$sdst, (not (and_oneuse i64:$src0, i64:$src1)))] >;

def S_NOR_B32 : SOP2_32 <"s_nor_b32",
  [(set i32:$sdst, (not (or_oneuse i32:$src0, i32:$src1)))] >;

def S_NOR_B64 : SOP2_64 <"s_nor_b64",
  [(set i64:$sdst, (not (or_oneuse i64:$src0, i64:$src1)))] >;
} // End isCommutable = 1

def S_ANDN2_B32 : SOP2_32 <"s_andn2_b32",
  [(set i32:$sdst, (UniformBinFrag<and> i32:$src0, (UniformUnaryFrag<not> i32:$src1)))] >;

def S_ANDN2_B64 : SOP2_64 <"s_andn2_b64",
  [(set i64:$sdst, (UniformBinFrag<and> i64:$src0, (UniformUnaryFrag<not> i64:$src1)))] >;

def S_ORN2_B32 : SOP2_32 <"s_orn2_b32",
  [(set i32:$sdst, (UniformBinFrag<or> i32:$src0, (UniformUnaryFrag<not> i32:$src1)))] >;

def S_ORN2_B64 : SOP2_64 <"s_orn2_b64",
  [(set i64:$sdst, (UniformBinFrag<or> i64:$src0, (UniformUnaryFrag<not> i64:$src1)))] >;
} // End Defs = [SCC]

let AddedComplexity = 1 in {

let Defs = [SCC] in {
// TODO: b64 versions require VOP3 change since v_lshlrev_b64 is VOP3
def S_LSHL_B32 : SOP2_32 <"s_lshl_b32",
  [(set SReg_32:$sdst, (shl (i32 SSrc_b32:$src0), (i32 SSrc_b32:$src1)))]
>;
def S_LSHL_B64 : SOP2_64_32 <"s_lshl_b64",
  [(set SReg_64:$sdst, (shl (i64 SSrc_b64:$src0), (i32 SSrc_b32:$src1)))]
>;
def S_LSHR_B32 : SOP2_32 <"s_lshr_b32",
  [(set SReg_32:$sdst, (srl (i32 SSrc_b32:$src0), (i32 SSrc_b32:$src1)))]
>;
def S_LSHR_B64 : SOP2_64_32 <"s_lshr_b64",
  [(set SReg_64:$sdst, (srl (i64 SSrc_b64:$src0), (i32 SSrc_b32:$src1)))]
>;
def S_ASHR_I32 : SOP2_32 <"s_ashr_i32",
  [(set SReg_32:$sdst, (sra (i32 SSrc_b32:$src0), (i32 SSrc_b32:$src1)))]
>;
def S_ASHR_I64 : SOP2_64_32 <"s_ashr_i64",
  [(set SReg_64:$sdst, (sra (i64 SSrc_b64:$src0), (i32 SSrc_b32:$src1)))]
>;
} // End Defs = [SCC]

def S_BFM_B32 : SOP2_32 <"s_bfm_b32",
  [(set i32:$sdst, (UniformBinFrag<PPUbfm> i32:$src0, i32:$src1))]>;
def S_BFM_B64 : SOP2_64_32_32 <"s_bfm_b64">;

// TODO: S_MUL_I32 require V_MUL_LO_I32 from VOP3 change
def S_MUL_I32 : SOP2_32 <"s_mul_i32",
  [(set i32:$sdst, (mul i32:$src0, i32:$src1))]> {
  let isCommutable = 1;
}

} // End AddedComplexity = 1

let Defs = [SCC] in {
def S_BFE_U32 : SOP2_32 <"s_bfe_u32">;
def S_BFE_I32 : SOP2_32 <"s_bfe_i32">;
def S_BFE_U64 : SOP2_64_32 <"s_bfe_u64">;
def S_BFE_I64 : SOP2_64_32 <"s_bfe_i64">;
} // End Defs = [SCC]

def S_CBRANCH_G_FORK : SOP2_Pseudo <
  "s_cbranch_g_fork", (outs),
  (ins SCSrc_b64:$src0, SCSrc_b64:$src1),
  "$src0, $src1"
> {
  let has_sdst = 0;
  // let SubtargetPredicate = isGFX6GFX7GFX8GFX9;
}

let Defs = [SCC] in {
def S_ABSDIFF_I32 : SOP2_32 <"s_absdiff_i32">;
} // End Defs = [SCC]

  def S_PACK_LL_B32_B16 : SOP2_32<"s_pack_ll_b32_b16">;
  def S_PACK_LH_B32_B16 : SOP2_32<"s_pack_lh_b32_b16">;
  def S_PACK_HH_B32_B16 : SOP2_32<"s_pack_hh_b32_b16">;

  let Defs = [SCC] in {
    def S_LSHL1_ADD_U32 : SOP2_32<"s_lshl1_add_u32">;
    def S_LSHL2_ADD_U32 : SOP2_32<"s_lshl2_add_u32">;
    def S_LSHL3_ADD_U32 : SOP2_32<"s_lshl3_add_u32">;
    def S_LSHL4_ADD_U32 : SOP2_32<"s_lshl4_add_u32">;
  } // End Defs = [SCC]

  def S_MUL_HI_U32 : SOP2_32<"s_mul_hi_u32">;
  def S_MUL_HI_I32 : SOP2_32<"s_mul_hi_i32">;



//===--------------------------------------------------
// SOPK
//===--------------------------------------------------
let isReMaterializable = 1, isMoveImm = 1 in {
def S_MOVK_I32 : SOPK_32 <"s_movk_i32">;
} // End isReMaterializable = 1
let Uses = [SCC] in {
def S_CMOVK_I32 : SOPK_32 <"s_cmovk_i32">;
}

let isCompare = 1 in {

// This instruction is disabled for now until we can figure out how to teach
// the instruction selector to correctly use the  S_CMP* vs V_CMP*
// instructions.
//
// When this instruction is enabled the code generator sometimes produces this
// invalid sequence:
//
// SCC = S_CMPK_EQ_I32 SGPR0, imm
// VCC = COPY SCC
// VGPR0 = V_CNDMASK VCC, VGPR0, VGPR1
//
// def S_CMPK_EQ_I32 : SOPK_SCC <"s_cmpk_eq_i32",
//   [(set i1:$dst, (setcc i32:$src0, imm:$src1, SETEQ))]
// >;

def S_CMPK_EQ_I32 : SOPK_SCC <"s_cmpk_eq_i32", "s_cmp_eq_i32", 1>;
def S_CMPK_LG_I32 : SOPK_SCC <"s_cmpk_lg_i32", "s_cmp_lg_i32", 1>;
def S_CMPK_GT_I32 : SOPK_SCC <"s_cmpk_gt_i32", "s_cmp_gt_i32", 1>;
def S_CMPK_GE_I32 : SOPK_SCC <"s_cmpk_ge_i32", "s_cmp_ge_i32", 1>;
def S_CMPK_LT_I32 : SOPK_SCC <"s_cmpk_lt_i32", "s_cmp_lt_i32", 1>;
def S_CMPK_LE_I32 : SOPK_SCC <"s_cmpk_le_i32", "s_cmp_le_i32", 1>;

let SOPKZext = 1 in {
def S_CMPK_EQ_U32 : SOPK_SCC <"s_cmpk_eq_u32", "s_cmp_eq_u32", 0>;
def S_CMPK_LG_U32 : SOPK_SCC <"s_cmpk_lg_u32", "s_cmp_lg_u32", 0>;
def S_CMPK_GT_U32 : SOPK_SCC <"s_cmpk_gt_u32", "s_cmp_gt_u32", 0>;
def S_CMPK_GE_U32 : SOPK_SCC <"s_cmpk_ge_u32", "s_cmp_ge_u32", 0>;
def S_CMPK_LT_U32 : SOPK_SCC <"s_cmpk_lt_u32", "s_cmp_lt_u32", 0>;
def S_CMPK_LE_U32 : SOPK_SCC <"s_cmpk_le_u32", "s_cmp_le_u32", 0>;
} // End SOPKZext = 1
} // End isCompare = 1

let Defs = [SCC], isCommutable = 1, DisableEncoding = "$src0",
    Constraints = "$sdst = $src0" in {
  def S_ADDK_I32 : SOPK_32TIE <"s_addk_i32">;
  def S_MULK_I32 : SOPK_32TIE <"s_mulk_i32">;
}
def S_CBRANCH_I_FORK : SOPK_Pseudo <
  "s_cbranch_i_fork",
  (outs), (ins SReg_64:$sdst, sopp_brtarget:$simm16),
  "$sdst, $simm16"
>;

// use CSR instread
/*
let mayLoad = 1 in {
def S_GETREG_B32 : SOPK_Pseudo <
  "s_getreg_b32",
  (outs SReg_32:$sdst), (ins hwreg:$simm16),
  "$sdst, $simm16"
>;
}

let hasSideEffects = 1 in {

def S_SETREG_B32 : SOPK_Pseudo <
  "s_setreg_b32",
  (outs), (ins SReg_32:$sdst, hwreg:$simm16),
  "$simm16, $sdst",
  [(AMDGPUsetreg i32:$sdst, (i16 timm:$simm16))]
>;

// FIXME: Not on SI?
//def S_GETREG_REGRD_B32 : SOPK_32 <sopk<0x14, 0x13>, "s_getreg_regrd_b32">;

def S_SETREG_IMM32_B32 : SOPK_Pseudo <
  "s_setreg_imm32_b32",
  (outs), (ins i32imm:$imm, hwreg:$simm16),
  "$simm16, $imm"> {
  let Size = 8; // Unlike every other SOPK instruction.
  let has_sdst = 0;
}

} // End hasSideEffects = 1
*/

  def S_CALL_B64 : SOPK_Pseudo<
      "s_call_b64",
      (outs SReg_64:$sdst),
      (ins sopp_brtarget:$simm16),
      "$sdst, $simm16"> {
    let isCall = 1;
  }

  def S_WAITCNT_VSCNT   : SOPK_WAITCNT<"s_waitcnt_vscnt">;
  def S_WAITCNT_VMCNT   : SOPK_WAITCNT<"s_waitcnt_vmcnt">;
  def S_WAITCNT_EXPCNT  : SOPK_WAITCNT<"s_waitcnt_expcnt">;
  def S_WAITCNT_LGKMCNT : SOPK_WAITCNT<"s_waitcnt_lgkmcnt">;

//====-------------------------------------------
// SOPC

def S_CMP_EQ_I32 : SOPC_CMP_32 <0x00, "s_cmp_eq_i32">;
def S_CMP_LG_I32 : SOPC_CMP_32 <0x01, "s_cmp_lg_i32">;
def S_CMP_GT_I32 : SOPC_CMP_32 <0x02, "s_cmp_gt_i32", COND_SGT>;
def S_CMP_GE_I32 : SOPC_CMP_32 <0x03, "s_cmp_ge_i32", COND_SGE>;
def S_CMP_LT_I32 : SOPC_CMP_32 <0x04, "s_cmp_lt_i32", COND_SLT, "s_cmp_gt_i32">;
def S_CMP_LE_I32 : SOPC_CMP_32 <0x05, "s_cmp_le_i32", COND_SLE, "s_cmp_ge_i32">;
def S_CMP_EQ_U32 : SOPC_CMP_32 <0x06, "s_cmp_eq_u32", COND_EQ>;
def S_CMP_LG_U32 : SOPC_CMP_32 <0x07, "s_cmp_lg_u32", COND_NE>;
def S_CMP_GT_U32 : SOPC_CMP_32 <0x08, "s_cmp_gt_u32", COND_UGT>;
def S_CMP_GE_U32 : SOPC_CMP_32 <0x09, "s_cmp_ge_u32", COND_UGE>;
def S_CMP_LT_U32 : SOPC_CMP_32 <0x0a, "s_cmp_lt_u32", COND_ULT, "s_cmp_gt_u32">;
def S_CMP_LE_U32 : SOPC_CMP_32 <0x0b, "s_cmp_le_u32", COND_ULE, "s_cmp_ge_u32">;

def S_BITCMP0_B32 : SOPC_32 <0x0c, "s_bitcmp0_b32">;
def S_BITCMP1_B32 : SOPC_32 <0x0d, "s_bitcmp1_b32">;
def S_BITCMP0_B64 : SOPC_64_32 <0x0e, "s_bitcmp0_b64">;
def S_BITCMP1_B64 : SOPC_64_32 <0x0f, "s_bitcmp1_b64">;

def S_SETVSKIP : SOPC_32 <0x10, "s_setvskip">;
def S_CMP_EQ_U64 : SOPC_CMP_64 <0x12, "s_cmp_eq_u64", COND_EQ>;
def S_CMP_LG_U64 : SOPC_CMP_64 <0x13, "s_cmp_lg_u64", COND_NE>;

let SubtargetPredicate = HasVPRIndexMode in {
def S_SET_GPR_IDX_ON : SOPC <0x11,
  (outs),
  (ins SSrc_b32:$src0, VPRIdxMode:$src1),
  "s_set_gpr_idx_on $src0,$src1"> {
  let Defs = [M0]; // No scc def
  let Uses = [M0]; // Other bits of m0 unmodified.
  let hasSideEffects = 1; // Sets mode.gpr_idx_en
  let FixedSize = 1;
}
}

//====-------------------------------------------
// SOPP

def S_NOP : SOPP <0x00000000, (ins i16imm:$simm16), "s_nop $simm16">;

class SOPP_w_nop_e <bits<7> op> : Enc64 {
  bits <16> simm16;

  let Inst{15-0} = simm16;
  let Inst{22-16} = op;
  let Inst{31-23} = 0x17f; // encoding
  let Inst{47-32} = 0x0;
  let Inst{54-48} = S_NOP.Inst{22-16}; // opcode
  let Inst{63-55} = S_NOP.Inst{31-23}; // encoding
}

class SOPP_w_nop <bits<7> op, dag ins, string asm, list<dag> pattern = []> :
  PPTInst <(outs), ins, asm, "", pattern >, SOPP_w_nop_e <op>, Base_SOPP <asm> {

  let mayLoad = 0;
  let mayStore = 0;
  let hasSideEffects = 0;
  let SALU = 1;
  let SOPP = 1;
  let Size = 8;
  let SchedRW = [WriteSALU];

  let UseNamedOperandTable = 1;
}

multiclass SOPP_With_Relaxation <bits<7> op, dag ins, string asm, list<dag> pattern = []> {
  def "" : SOPP <op, ins, asm, pattern>;
  def _pad_s_nop : SOPP_w_nop <op, ins, asm, pattern>;
}


let isTerminator = 1 in {

def S_ENDPGM : SOPP <0x00000001, (ins EndpgmImm:$simm16), "s_endpgm$simm16"> {
  let isBarrier = 1;
  let isReturn = 1;
}

def S_ENDPGM_SAVED : SOPP <0x0000001B, (ins), "s_endpgm_saved"> {
  // let SubtargetPredicate = isGFX8Plus;
  let simm16 = 0;
  let isBarrier = 1;
  let isReturn = 1;
}


let isBranch = 1, SchedRW = [WriteBranch] in {
let isBarrier = 1 in {
defm S_BRANCH : SOPP_With_Relaxation <
  0x00000002, (ins sopp_brtarget:$simm16), "s_branch $simm16",
  [(br bb:$simm16)]>;
}


let Uses = [SCC] in {
defm S_CBRANCH_SCC0 : SOPP_With_Relaxation <
  0x00000004, (ins sopp_brtarget:$simm16),
  "s_cbranch_scc0 $simm16"
>;
defm S_CBRANCH_SCC1 : SOPP_With_Relaxation <
  0x00000005, (ins sopp_brtarget:$simm16),
  "s_cbranch_scc1 $simm16"
>;
} // End Uses = [SCC]

let Uses = [VCC] in {
defm S_CBRANCH_VCCZ : SOPP_With_Relaxation <
  0x00000006, (ins sopp_brtarget:$simm16),
  "s_cbranch_vccz $simm16"
>;
defm S_CBRANCH_VCCNZ : SOPP_With_Relaxation <
  0x00000007, (ins sopp_brtarget:$simm16),
  "s_cbranch_vccnz $simm16"
>;
} // End Uses = [VCC]

let Uses = [TMSK] in {
defm S_CBRANCH_TMSKZ : SOPP_With_Relaxation <
  0x00000008, (ins sopp_brtarget:$simm16),
  "s_cbranch_execz $simm16"
>;
defm S_CBRANCH_TMSKNZ : SOPP_With_Relaxation <
  0x00000009, (ins sopp_brtarget:$simm16),
  "s_cbranch_execnz $simm16"
>;
} // End Uses = [TMSK]

defm S_CBRANCH_CDBGSYS : SOPP_With_Relaxation <
  0x00000017, (ins sopp_brtarget:$simm16),
  "s_cbranch_cdbgsys $simm16"
>;

defm S_CBRANCH_CDBGSYS_AND_USER : SOPP_With_Relaxation <
  0x0000001A, (ins sopp_brtarget:$simm16),
  "s_cbranch_cdbgsys_and_user $simm16"
>;

defm S_CBRANCH_CDBGSYS_OR_USER : SOPP_With_Relaxation <
  0x00000019, (ins sopp_brtarget:$simm16),
  "s_cbranch_cdbgsys_or_user $simm16"
>;

defm S_CBRANCH_CDBGUSER : SOPP_With_Relaxation <
  0x00000018, (ins sopp_brtarget:$simm16),
  "s_cbranch_cdbguser $simm16"
>;

} // End isBranch = 1
} // End isTerminator = 1

let hasSideEffects = 1 in {
def S_BARRIER : SOPP <0x0000000a, (ins), "s_barrier",
  [(int_ppu_barrier)]> {
  let SchedRW = [WriteBarrier];
  let simm16 = 0;
  let mayLoad = 1;
  let mayStore = 1;
  let isConvergent = 1;
}
/*
def S_WAKEUP : SOPP <0x00000003, (ins), "s_wakeup"> {
  let simm16 = 0;
  let mayLoad = 1;
  let mayStore = 1;
}
*/

let mayLoad = 1, mayStore = 1, hasSideEffects = 1 in
def S_WAITCNT : SOPP <0x0000000c, (ins WAIT_FLAG:$simm16), "s_waitcnt $simm16",
    [(int_ppu_waitcnt UIMM16bit:$simm16)]>;
def S_SETHALT : SOPP <0x0000000d, (ins i16imm:$simm16), "s_sethalt $simm16">;
def S_SETKILL : SOPP <0x0000000b, (ins i16imm:$simm16), "s_setkill $simm16">;

// On SI the documentation says sleep for approximately 64 * low 2
// bits, consistent with the reported maximum of 448. On VI the
// maximum reported is 960 cycles, so 960 / 64 = 15 max, so is the
// maximum really 15 on VI?
/*
def S_SLEEP : SOPP <0x0000000e, (ins i32imm:$simm16),
  "s_sleep $simm16", [(int_ppu_s_sleep SIMM16bit:$simm16)]> {
  let hasSideEffects = 1;
  let mayLoad = 1;
  let mayStore = 1;
}
*/

def S_SETPRIO : SOPP <0x0000000f, (ins i16imm:$simm16), "s_setprio $simm16">;

// let Uses = [TMSK, M0], mayLoad = 1, mayStore=1 in {

// FIXME: Should this be mayLoad+mayStore?
// def S_SENDMSG : SOPP <0x00000010, (ins SendMsgImm:$simm16), "s_sendmsg $simm16",
//   [(int_ppu_sendmsg (i32 imm:$simm16), M0)]>;

// def S_SENDMSGHALT : SOPP <0x00000011, (ins SendMsgImm:$simm16), "s_sendmsghalt $simm16",
//   [(int_ppu_sendmsghalt (i32 imm:$simm16), M0)]>;
// 
// } // End Uses = [TMSK, M0]

def S_TRAP : SOPP <0x00000012, (ins i16imm:$simm16), "s_trap $simm16"> {
  let isTrap = 1;
}

def S_ICACHE_INV : SOPP <0x00000013, (ins), "s_icache_inv"> {
  let simm16 = 0;
}

let SubtargetPredicate = HasVPRIndexMode in {
def S_SET_GPR_IDX_OFF : SOPP<0x1c, (ins), "s_set_gpr_idx_off"> {
  let simm16 = 0;
}
}


  def S_ROUND_MODE :
    SOPP<0x024, (ins s16imm:$simm16), "s_round_mode $simm16">;
  def S_DENORM_MODE :
    SOPP<0x025, (ins i32imm:$simm16), "s_denorm_mode $simm16",
    [(SIdenorm_mode (i32 timm:$simm16))]> {
      let hasSideEffects = 1;
    }

} // End hasSideEffects


//===----------------------------------------------------------------------===//
// S_GETREG_B32 Intrinsic Pattern.
//===----------------------------------------------------------------------===//
/*
def : PPUPat <
  (int_ppu_s_getreg imm:$simm16),
  (S_GETREG_B32 (as_i16imm $simm16))
>;
*/

//===----------------------------------------------------------------------===//
// SOP1 Patterns
//===----------------------------------------------------------------------===//

def : PPUPat <
  (PPUendpgm),
    (S_ENDPGM (i16 0))
>;

/*
def : PPUPat <
  (i64 (ctpop i64:$src)),
    (i64 (REG_SEQUENCE SReg_64,
     (i32 (COPY_TO_REGCLASS (S_BCNT1_I32_B64 $src), SReg_32)), sub0,
     (S_MOV_B32 (i32 0)), sub1))
>;
*/

def : PPUPat <
  (i32 (smax i32:$x, (i32 (ineg i32:$x)))),
  (S_ABS_I32 $x)
>;

def : PPUPat <
  (i16 imm:$imm),
  (S_MOV_B32 imm:$imm)
>;

// Same as a 32-bit inreg
def : PPUPat<
  (i32 (sext i16:$src)),
  (S_SEXT_I32_I16 $src)
>;


//===----------------------------------------------------------------------===//
// SOP2 Patterns
//===----------------------------------------------------------------------===//

// V_ADD_I32_e32/S_ADD_U32 produces carry in VCC/SCC. For the vector
// case, the sgpr-copies pass will fix this to use the vector version.
def : PPUPat <
  (i32 (addc i32:$src0, i32:$src1)),
  (S_ADD_U32 $src0, $src1)
>;

// FIXME: We need to use COPY_TO_REGCLASS to work-around the fact that
// REG_SEQUENCE patterns don't support instructions with multiple
// outputs.
def : PPUPat<
  (i64 (zext i16:$src)),
    (REG_SEQUENCE SReg_64,
      (i32 (COPY_TO_REGCLASS (S_AND_B32 $src, (S_MOV_B32 (i32 0xffff))), SPR_32)), sub0,
      (S_MOV_B32 (i32 0)), sub1)
>;

def : PPUPat <
  (i64 (sext i16:$src)),
    (REG_SEQUENCE SReg_64, (i32 (S_SEXT_I32_I16 $src)), sub0,
    (i32 (COPY_TO_REGCLASS (S_ASHR_I32 (i32 (S_SEXT_I32_I16 $src)), (S_MOV_B32 (i32 31))), SPR_32)), sub1)
>;

def : PPUPat<
  (i32 (zext i16:$src)),
  (S_AND_B32 (S_MOV_B32 (i32 0xffff)), $src)
>;


//===----------------------------------------------------------------------===//
// Target-specific instruction encodings.
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
// SOP1 - GFX10.
//===----------------------------------------------------------------------===//

class Select_gfx10<string opName> : PPUMCInstr<opName, PPUEncodingFamily.PPU> {
  Predicate AssemblerPredicate = isPPT;
  string DecoderNamespace      = "PPU";
}

multiclass SOP1_Real_gfx10<bits<8> op> {
  def _gfx10 : SOP1_Real<op, !cast<SOP1_Pseudo>(NAME)>,
               Select_gfx10<!cast<SOP1_Pseudo>(NAME).Mnemonic>;
}

// defm S_BITREPLICATE_B64_B32 : SOP1_Real_gfx10<0x03b>;
defm S_AND_SAVETMSK_B32     : SOP1_Real_gfx10<0x03c>;
defm S_OR_SAVETMSK_B32      : SOP1_Real_gfx10<0x03d>;
defm S_XOR_SAVETMSK_B32     : SOP1_Real_gfx10<0x03e>;
defm S_ANDN2_SAVETMSK_B32   : SOP1_Real_gfx10<0x03f>;
defm S_ORN2_SAVETMSK_B32    : SOP1_Real_gfx10<0x040>;
defm S_NAND_SAVETMSK_B32    : SOP1_Real_gfx10<0x041>;
defm S_NOR_SAVETMSK_B32     : SOP1_Real_gfx10<0x042>;
defm S_XNOR_SAVETMSK_B32    : SOP1_Real_gfx10<0x043>;
defm S_ANDN1_SAVETMSK_B32   : SOP1_Real_gfx10<0x044>;
defm S_ORN1_SAVETMSK_B32    : SOP1_Real_gfx10<0x045>;
defm S_ANDN1_WRTMSK_B32     : SOP1_Real_gfx10<0x046>;
defm S_ANDN2_WRTMSK_B32     : SOP1_Real_gfx10<0x047>;
defm S_MOVRELSD_2_B32       : SOP1_Real_gfx10<0x049>;

defm S_MOV_B32            : SOP1_Real_gfx10<0x003>;
defm S_MOV_B64            : SOP1_Real_gfx10<0x004>;
defm S_CMOV_B32           : SOP1_Real_gfx10<0x005>;
defm S_CMOV_B64           : SOP1_Real_gfx10<0x006>;
defm S_NOT_B32            : SOP1_Real_gfx10<0x007>;
defm S_NOT_B64            : SOP1_Real_gfx10<0x008>;
defm S_WQM_B32            : SOP1_Real_gfx10<0x009>;
defm S_WQM_B64            : SOP1_Real_gfx10<0x00a>;
defm S_BREV_B32           : SOP1_Real_gfx10<0x00b>;
// defm S_BREV_B64           : SOP1_Real_gfx10<0x00c>;
defm S_BCNT0_I32_B32      : SOP1_Real_gfx10<0x00d>;
// defm S_BCNT0_I32_B64      : SOP1_Real_gfx10<0x00e>;
defm S_BCNT1_I32_B32      : SOP1_Real_gfx10<0x00f>;
// defm S_BCNT1_I32_B64      : SOP1_Real_gfx10<0x010>;
defm S_FF0_I32_B32        : SOP1_Real_gfx10<0x011>;
// defm S_FF0_I32_B64        : SOP1_Real_gfx10<0x012>;
defm S_FF1_I32_B32        : SOP1_Real_gfx10<0x013>;
// defm S_FF1_I32_B64        : SOP1_Real_gfx10<0x014>;
defm S_FLBIT_I32_B32      : SOP1_Real_gfx10<0x015>;
// defm S_FLBIT_I32_B64      : SOP1_Real_gfx10<0x016>;
defm S_FLBIT_I32          : SOP1_Real_gfx10<0x017>;
// defm S_FLBIT_I32_I64      : SOP1_Real_gfx10<0x018>;
defm S_SEXT_I32_I8        : SOP1_Real_gfx10<0x019>;
defm S_SEXT_I32_I16       : SOP1_Real_gfx10<0x01a>;
defm S_BITSET0_B32        : SOP1_Real_gfx10<0x01b>;
// defm S_BITSET0_B64        : SOP1_Real_gfx10<0x01c>;
defm S_BITSET1_B32        : SOP1_Real_gfx10<0x01d>;
// defm S_BITSET1_B64        : SOP1_Real_gfx10<0x01e>;
defm S_GETPC_B64          : SOP1_Real_gfx10<0x01f>;
defm S_SETPC_B64          : SOP1_Real_gfx10<0x020>;
defm S_SWAPPC_B64         : SOP1_Real_gfx10<0x021>;
// defm S_RFE_B64            : SOP1_Real_gfx10<0x022>;
defm S_AND_SAVETMSK_B64   : SOP1_Real_gfx10<0x024>;
defm S_OR_SAVETMSK_B64    : SOP1_Real_gfx10<0x025>;
defm S_XOR_SAVETMSK_B64   : SOP1_Real_gfx10<0x026>;
defm S_ANDN2_SAVETMSK_B64 : SOP1_Real_gfx10<0x027>;
defm S_ORN2_SAVETMSK_B64  : SOP1_Real_gfx10<0x028>;
defm S_NAND_SAVETMSK_B64  : SOP1_Real_gfx10<0x029>;
defm S_NOR_SAVETMSK_B64   : SOP1_Real_gfx10<0x02a>;
defm S_XNOR_SAVETMSK_B64  : SOP1_Real_gfx10<0x02b>;
// defm S_QUADMASK_B32       : SOP1_Real_gfx10<0x02c>;
// defm S_QUADMASK_B64       : SOP1_Real_gfx10<0x02d>;
// defm S_MOVRELS_B32        : SOP1_Real_gfx10<0x02e>;
// defm S_MOVRELS_B64        : SOP1_Real_gfx10<0x02f>;
// defm S_MOVRELD_B32        : SOP1_Real_gfx10<0x030>;
// defm S_MOVRELD_B64        : SOP1_Real_gfx10<0x031>;
defm S_ABS_I32            : SOP1_Real_gfx10<0x034>;
// defm S_MOV_FED_B32        : SOP1_Real_gfx10<0x035>;

//===----------------------------------------------------------------------===//
// SOP2 - GFX10.
//===----------------------------------------------------------------------===//

multiclass SOP2_Real_gfx10<bits<7> op> {
  def _gfx10 : SOP2_Real<op, !cast<SOP2_Pseudo>(NAME)>,
               Select_gfx10<!cast<SOP2_Pseudo>(NAME).Mnemonic>;
}

defm S_LSHL1_ADD_U32   : SOP2_Real_gfx10<0x02e>;
defm S_LSHL2_ADD_U32   : SOP2_Real_gfx10<0x02f>;
defm S_LSHL3_ADD_U32   : SOP2_Real_gfx10<0x030>;
defm S_LSHL4_ADD_U32   : SOP2_Real_gfx10<0x031>;
defm S_PACK_LL_B32_B16 : SOP2_Real_gfx10<0x032>;
defm S_PACK_LH_B32_B16 : SOP2_Real_gfx10<0x033>;
defm S_PACK_HH_B32_B16 : SOP2_Real_gfx10<0x034>;
defm S_MUL_HI_U32      : SOP2_Real_gfx10<0x035>;
defm S_MUL_HI_I32      : SOP2_Real_gfx10<0x036>;

defm S_ADD_U32     : SOP2_Real_gfx10<0x000>;
defm S_SUB_U32     : SOP2_Real_gfx10<0x001>;
defm S_ADD_I32     : SOP2_Real_gfx10<0x002>;
defm S_SUB_I32     : SOP2_Real_gfx10<0x003>;
defm S_ADDC_U32    : SOP2_Real_gfx10<0x004>;
defm S_SUBB_U32    : SOP2_Real_gfx10<0x005>;
defm S_MIN_I32     : SOP2_Real_gfx10<0x006>;
defm S_MIN_U32     : SOP2_Real_gfx10<0x007>;
defm S_MAX_I32     : SOP2_Real_gfx10<0x008>;
defm S_MAX_U32     : SOP2_Real_gfx10<0x009>;
defm S_CSELECT_B32 : SOP2_Real_gfx10<0x00a>;
defm S_CSELECT_B64 : SOP2_Real_gfx10<0x00b>;
defm S_AND_B32     : SOP2_Real_gfx10<0x00e>;
defm S_AND_B64     : SOP2_Real_gfx10<0x00f>;
defm S_OR_B32      : SOP2_Real_gfx10<0x010>;
defm S_OR_B64      : SOP2_Real_gfx10<0x011>;
defm S_XOR_B32     : SOP2_Real_gfx10<0x012>;
defm S_XOR_B64     : SOP2_Real_gfx10<0x013>;
defm S_ANDN2_B32   : SOP2_Real_gfx10<0x014>;
defm S_ANDN2_B64   : SOP2_Real_gfx10<0x015>;
defm S_ORN2_B32    : SOP2_Real_gfx10<0x016>;
defm S_ORN2_B64    : SOP2_Real_gfx10<0x017>;
defm S_NAND_B32    : SOP2_Real_gfx10<0x018>;
defm S_NAND_B64    : SOP2_Real_gfx10<0x019>;
defm S_NOR_B32     : SOP2_Real_gfx10<0x01a>;
defm S_NOR_B64     : SOP2_Real_gfx10<0x01b>;
defm S_XNOR_B32    : SOP2_Real_gfx10<0x01c>;
defm S_XNOR_B64    : SOP2_Real_gfx10<0x01d>;
defm S_LSHL_B32    : SOP2_Real_gfx10<0x01e>;
defm S_LSHL_B64    : SOP2_Real_gfx10<0x01f>;
defm S_LSHR_B32    : SOP2_Real_gfx10<0x020>;
defm S_LSHR_B64    : SOP2_Real_gfx10<0x021>;
defm S_ASHR_I32    : SOP2_Real_gfx10<0x022>;
defm S_ASHR_I64    : SOP2_Real_gfx10<0x023>;
defm S_BFM_B32     : SOP2_Real_gfx10<0x024>;
defm S_BFM_B64     : SOP2_Real_gfx10<0x025>;
defm S_MUL_I32     : SOP2_Real_gfx10<0x026>;
defm S_BFE_U32     : SOP2_Real_gfx10<0x027>;
defm S_BFE_I32     : SOP2_Real_gfx10<0x028>;
defm S_BFE_U64     : SOP2_Real_gfx10<0x029>;
defm S_BFE_I64     : SOP2_Real_gfx10<0x02a>;
defm S_ABSDIFF_I32 : SOP2_Real_gfx10<0x02c>;

//===----------------------------------------------------------------------===//
// SOPK - GFX10.
//===----------------------------------------------------------------------===//

multiclass SOPK_Real32_gfx10<bits<5> op> {
  def _gfx10 : SOPK_Real32<op, !cast<SOPK_Pseudo>(NAME)>,
               Select_gfx10<!cast<SOPK_Pseudo>(NAME).Mnemonic>;
}

multiclass SOPK_Real64_gfx10<bits<5> op> {
  def _gfx10 : SOPK_Real64<op, !cast<SOPK_Pseudo>(NAME)>,
               Select_gfx10<!cast<SOPK_Pseudo>(NAME).Mnemonic>;
}

// defm S_VERSION              : SOPK_Real32_gfx10<0x001>;
defm S_CALL_B64             : SOPK_Real32_gfx10<0x016>;
defm S_WAITCNT_VSCNT        : SOPK_Real32_gfx10<0x017>;
defm S_WAITCNT_VMCNT        : SOPK_Real32_gfx10<0x018>;
defm S_WAITCNT_EXPCNT       : SOPK_Real32_gfx10<0x019>;
defm S_WAITCNT_LGKMCNT      : SOPK_Real32_gfx10<0x01a>;
// defm S_SUBVECTOR_LOOP_BEGIN : SOPK_Real32_gfx10<0x01b>;
// defm S_SUBVECTOR_LOOP_END   : SOPK_Real32_gfx10<0x01c>;

defm S_MOVK_I32         : SOPK_Real32_gfx10<0x000>;
defm S_CMOVK_I32        : SOPK_Real32_gfx10<0x002>;
defm S_CMPK_EQ_I32      : SOPK_Real32_gfx10<0x003>;
defm S_CMPK_LG_I32      : SOPK_Real32_gfx10<0x004>;
defm S_CMPK_GT_I32      : SOPK_Real32_gfx10<0x005>;
defm S_CMPK_GE_I32      : SOPK_Real32_gfx10<0x006>;
defm S_CMPK_LT_I32      : SOPK_Real32_gfx10<0x007>;
defm S_CMPK_LE_I32      : SOPK_Real32_gfx10<0x008>;
defm S_CMPK_EQ_U32      : SOPK_Real32_gfx10<0x009>;
defm S_CMPK_LG_U32      : SOPK_Real32_gfx10<0x00a>;
defm S_CMPK_GT_U32      : SOPK_Real32_gfx10<0x00b>;
defm S_CMPK_GE_U32      : SOPK_Real32_gfx10<0x00c>;
defm S_CMPK_LT_U32      : SOPK_Real32_gfx10<0x00d>;
defm S_CMPK_LE_U32      : SOPK_Real32_gfx10<0x00e>;
defm S_ADDK_I32         : SOPK_Real32_gfx10<0x00f>;
defm S_MULK_I32         : SOPK_Real32_gfx10<0x010>;
// defm S_GETREG_B32       : SOPK_Real32_gfx10<0x012>;
// defm S_SETREG_B32       : SOPK_Real32_gfx10<0x013>;
// defm S_SETREG_IMM32_B32 : SOPK_Real64_gfx10<0x015>;


