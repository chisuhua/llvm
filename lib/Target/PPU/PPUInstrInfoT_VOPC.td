
//===----------------------------------------------------------------------===//
// Compare instructions
//===----------------------------------------------------------------------===//

defm V_CMP_F_F32 : VOPC_F32 <"v_cmp_f_f32">;
defm V_CMP_LT_F32 : VOPC_F32 <"v_cmp_lt_f32", COND_OLT, "v_cmp_gt_f32">;
defm V_CMP_EQ_F32 : VOPC_F32 <"v_cmp_eq_f32", COND_OEQ>;
defm V_CMP_LE_F32 : VOPC_F32 <"v_cmp_le_f32", COND_OLE, "v_cmp_ge_f32">;
defm V_CMP_GT_F32 : VOPC_F32 <"v_cmp_gt_f32", COND_OGT>;
defm V_CMP_LG_F32 : VOPC_F32 <"v_cmp_lg_f32", COND_ONE>;
defm V_CMP_GE_F32 : VOPC_F32 <"v_cmp_ge_f32", COND_OGE>;
defm V_CMP_O_F32 : VOPC_F32 <"v_cmp_o_f32", COND_O>;
defm V_CMP_U_F32 : VOPC_F32 <"v_cmp_u_f32", COND_UO>;
defm V_CMP_NGE_F32 : VOPC_F32 <"v_cmp_nge_f32",  COND_ULT, "v_cmp_nle_f32">;
defm V_CMP_NLG_F32 : VOPC_F32 <"v_cmp_nlg_f32", COND_UEQ>;
defm V_CMP_NGT_F32 : VOPC_F32 <"v_cmp_ngt_f32", COND_ULE, "v_cmp_nlt_f32">;
defm V_CMP_NLE_F32 : VOPC_F32 <"v_cmp_nle_f32", COND_UGT>;
defm V_CMP_NEQ_F32 : VOPC_F32 <"v_cmp_neq_f32", COND_UNE>;
defm V_CMP_NLT_F32 : VOPC_F32 <"v_cmp_nlt_f32", COND_UGE>;
defm V_CMP_TRU_F32 : VOPC_F32 <"v_cmp_tru_f32">;

defm V_CMPX_F_F32 : VOPCX_F32 <"v_cmpx_f_f32">;
defm V_CMPX_LT_F32 : VOPCX_F32 <"v_cmpx_lt_f32", "v_cmpx_gt_f32">;
defm V_CMPX_EQ_F32 : VOPCX_F32 <"v_cmpx_eq_f32">;
defm V_CMPX_LE_F32 : VOPCX_F32 <"v_cmpx_le_f32", "v_cmpx_ge_f32">;
defm V_CMPX_GT_F32 : VOPCX_F32 <"v_cmpx_gt_f32">;
defm V_CMPX_LG_F32 : VOPCX_F32 <"v_cmpx_lg_f32">;
defm V_CMPX_GE_F32 : VOPCX_F32 <"v_cmpx_ge_f32">;
defm V_CMPX_O_F32 : VOPCX_F32 <"v_cmpx_o_f32">;
defm V_CMPX_U_F32 : VOPCX_F32 <"v_cmpx_u_f32">;
defm V_CMPX_NGE_F32 : VOPCX_F32 <"v_cmpx_nge_f32", "v_cmpx_nle_f32">;
defm V_CMPX_NLG_F32 : VOPCX_F32 <"v_cmpx_nlg_f32">;
defm V_CMPX_NGT_F32 : VOPCX_F32 <"v_cmpx_ngt_f32", "v_cmpx_nlt_f32">;
defm V_CMPX_NLE_F32 : VOPCX_F32 <"v_cmpx_nle_f32">;
defm V_CMPX_NEQ_F32 : VOPCX_F32 <"v_cmpx_neq_f32">;
defm V_CMPX_NLT_F32 : VOPCX_F32 <"v_cmpx_nlt_f32">;
defm V_CMPX_TRU_F32 : VOPCX_F32 <"v_cmpx_tru_f32">;

defm V_CMP_F_F64 : VOPC_F64 <"v_cmp_f_f64">;
defm V_CMP_LT_F64 : VOPC_F64 <"v_cmp_lt_f64", COND_OLT, "v_cmp_gt_f64">;
defm V_CMP_EQ_F64 : VOPC_F64 <"v_cmp_eq_f64", COND_OEQ>;
defm V_CMP_LE_F64 : VOPC_F64 <"v_cmp_le_f64", COND_OLE, "v_cmp_ge_f64">;
defm V_CMP_GT_F64 : VOPC_F64 <"v_cmp_gt_f64", COND_OGT>;
defm V_CMP_LG_F64 : VOPC_F64 <"v_cmp_lg_f64", COND_ONE>;
defm V_CMP_GE_F64 : VOPC_F64 <"v_cmp_ge_f64", COND_OGE>;
defm V_CMP_O_F64 : VOPC_F64 <"v_cmp_o_f64", COND_O>;
defm V_CMP_U_F64 : VOPC_F64 <"v_cmp_u_f64", COND_UO>;
defm V_CMP_NGE_F64 : VOPC_F64 <"v_cmp_nge_f64", COND_ULT, "v_cmp_nle_f64">;
defm V_CMP_NLG_F64 : VOPC_F64 <"v_cmp_nlg_f64", COND_UEQ>;
defm V_CMP_NGT_F64 : VOPC_F64 <"v_cmp_ngt_f64", COND_ULE, "v_cmp_nlt_f64">;
defm V_CMP_NLE_F64 : VOPC_F64 <"v_cmp_nle_f64", COND_UGT>;
defm V_CMP_NEQ_F64 : VOPC_F64 <"v_cmp_neq_f64", COND_UNE>;
defm V_CMP_NLT_F64 : VOPC_F64 <"v_cmp_nlt_f64", COND_UGE>;
defm V_CMP_TRU_F64 : VOPC_F64 <"v_cmp_tru_f64">;

defm V_CMPX_F_F64 : VOPCX_F64 <"v_cmpx_f_f64">;
defm V_CMPX_LT_F64 : VOPCX_F64 <"v_cmpx_lt_f64", "v_cmpx_gt_f64">;
defm V_CMPX_EQ_F64 : VOPCX_F64 <"v_cmpx_eq_f64">;
defm V_CMPX_LE_F64 : VOPCX_F64 <"v_cmpx_le_f64", "v_cmpx_ge_f64">;
defm V_CMPX_GT_F64 : VOPCX_F64 <"v_cmpx_gt_f64">;
defm V_CMPX_LG_F64 : VOPCX_F64 <"v_cmpx_lg_f64">;
defm V_CMPX_GE_F64 : VOPCX_F64 <"v_cmpx_ge_f64">;
defm V_CMPX_O_F64 : VOPCX_F64 <"v_cmpx_o_f64">;
defm V_CMPX_U_F64 : VOPCX_F64 <"v_cmpx_u_f64">;
defm V_CMPX_NGE_F64 : VOPCX_F64 <"v_cmpx_nge_f64", "v_cmpx_nle_f64">;
defm V_CMPX_NLG_F64 : VOPCX_F64 <"v_cmpx_nlg_f64">;
defm V_CMPX_NGT_F64 : VOPCX_F64 <"v_cmpx_ngt_f64", "v_cmpx_nlt_f64">;
defm V_CMPX_NLE_F64 : VOPCX_F64 <"v_cmpx_nle_f64">;
defm V_CMPX_NEQ_F64 : VOPCX_F64 <"v_cmpx_neq_f64">;
defm V_CMPX_NLT_F64 : VOPCX_F64 <"v_cmpx_nlt_f64">;
defm V_CMPX_TRU_F64 : VOPCX_F64 <"v_cmpx_tru_f64">;

defm V_CMPS_F_F32 : VOPC_F32 <"v_cmps_f_f32">;
defm V_CMPS_LT_F32 : VOPC_F32 <"v_cmps_lt_f32", COND_NULL, "v_cmps_gt_f32">;
defm V_CMPS_EQ_F32 : VOPC_F32 <"v_cmps_eq_f32">;
defm V_CMPS_LE_F32 : VOPC_F32 <"v_cmps_le_f32", COND_NULL, "v_cmps_ge_f32">;
defm V_CMPS_GT_F32 : VOPC_F32 <"v_cmps_gt_f32">;
defm V_CMPS_LG_F32 : VOPC_F32 <"v_cmps_lg_f32">;
defm V_CMPS_GE_F32 : VOPC_F32 <"v_cmps_ge_f32">;
defm V_CMPS_O_F32 : VOPC_F32 <"v_cmps_o_f32">;
defm V_CMPS_U_F32 : VOPC_F32 <"v_cmps_u_f32">;
defm V_CMPS_NGE_F32 : VOPC_F32 <"v_cmps_nge_f32", COND_NULL, "v_cmps_nle_f32">;
defm V_CMPS_NLG_F32 : VOPC_F32 <"v_cmps_nlg_f32">;
defm V_CMPS_NGT_F32 : VOPC_F32 <"v_cmps_ngt_f32", COND_NULL, "v_cmps_nlt_f32">;
defm V_CMPS_NLE_F32 : VOPC_F32 <"v_cmps_nle_f32">;
defm V_CMPS_NEQ_F32 : VOPC_F32 <"v_cmps_neq_f32">;
defm V_CMPS_NLT_F32 : VOPC_F32 <"v_cmps_nlt_f32">;
defm V_CMPS_TRU_F32 : VOPC_F32 <"v_cmps_tru_f32">;

defm V_CMPSX_F_F32 : VOPCX_F32 <"v_cmpsx_f_f32">;
defm V_CMPSX_LT_F32 : VOPCX_F32 <"v_cmpsx_lt_f32", "v_cmpsx_gt_f32">;
defm V_CMPSX_EQ_F32 : VOPCX_F32 <"v_cmpsx_eq_f32">;
defm V_CMPSX_LE_F32 : VOPCX_F32 <"v_cmpsx_le_f32", "v_cmpsx_ge_f32">;
defm V_CMPSX_GT_F32 : VOPCX_F32 <"v_cmpsx_gt_f32">;
defm V_CMPSX_LG_F32 : VOPCX_F32 <"v_cmpsx_lg_f32">;
defm V_CMPSX_GE_F32 : VOPCX_F32 <"v_cmpsx_ge_f32">;
defm V_CMPSX_O_F32 : VOPCX_F32 <"v_cmpsx_o_f32">;
defm V_CMPSX_U_F32 : VOPCX_F32 <"v_cmpsx_u_f32">;
defm V_CMPSX_NGE_F32 : VOPCX_F32 <"v_cmpsx_nge_f32", "v_cmpsx_nle_f32">;
defm V_CMPSX_NLG_F32 : VOPCX_F32 <"v_cmpsx_nlg_f32">;
defm V_CMPSX_NGT_F32 : VOPCX_F32 <"v_cmpsx_ngt_f32", "v_cmpsx_nlt_f32">;
defm V_CMPSX_NLE_F32 : VOPCX_F32 <"v_cmpsx_nle_f32">;
defm V_CMPSX_NEQ_F32 : VOPCX_F32 <"v_cmpsx_neq_f32">;
defm V_CMPSX_NLT_F32 : VOPCX_F32 <"v_cmpsx_nlt_f32">;
defm V_CMPSX_TRU_F32 : VOPCX_F32 <"v_cmpsx_tru_f32">;

defm V_CMPS_F_F64 : VOPC_F64 <"v_cmps_f_f64">;
defm V_CMPS_LT_F64 : VOPC_F64 <"v_cmps_lt_f64", COND_NULL, "v_cmps_gt_f64">;
defm V_CMPS_EQ_F64 : VOPC_F64 <"v_cmps_eq_f64">;
defm V_CMPS_LE_F64 : VOPC_F64 <"v_cmps_le_f64", COND_NULL, "v_cmps_ge_f64">;
defm V_CMPS_GT_F64 : VOPC_F64 <"v_cmps_gt_f64">;
defm V_CMPS_LG_F64 : VOPC_F64 <"v_cmps_lg_f64">;
defm V_CMPS_GE_F64 : VOPC_F64 <"v_cmps_ge_f64">;
defm V_CMPS_O_F64 : VOPC_F64 <"v_cmps_o_f64">;
defm V_CMPS_U_F64 : VOPC_F64 <"v_cmps_u_f64">;
defm V_CMPS_NGE_F64 : VOPC_F64 <"v_cmps_nge_f64", COND_NULL, "v_cmps_nle_f64">;
defm V_CMPS_NLG_F64 : VOPC_F64 <"v_cmps_nlg_f64">;
defm V_CMPS_NGT_F64 : VOPC_F64 <"v_cmps_ngt_f64", COND_NULL, "v_cmps_nlt_f64">;
defm V_CMPS_NLE_F64 : VOPC_F64 <"v_cmps_nle_f64">;
defm V_CMPS_NEQ_F64 : VOPC_F64 <"v_cmps_neq_f64">;
defm V_CMPS_NLT_F64 : VOPC_F64 <"v_cmps_nlt_f64">;
defm V_CMPS_TRU_F64 : VOPC_F64 <"v_cmps_tru_f64">;

defm V_CMPSX_F_F64 : VOPCX_F64 <"v_cmpsx_f_f64">;
defm V_CMPSX_LT_F64 : VOPCX_F64 <"v_cmpsx_lt_f64", "v_cmpsx_gt_f64">;
defm V_CMPSX_EQ_F64 : VOPCX_F64 <"v_cmpsx_eq_f64">;
defm V_CMPSX_LE_F64 : VOPCX_F64 <"v_cmpsx_le_f64", "v_cmpsx_ge_f64">;
defm V_CMPSX_GT_F64 : VOPCX_F64 <"v_cmpsx_gt_f64">;
defm V_CMPSX_LG_F64 : VOPCX_F64 <"v_cmpsx_lg_f64">;
defm V_CMPSX_GE_F64 : VOPCX_F64 <"v_cmpsx_ge_f64">;
defm V_CMPSX_O_F64 : VOPCX_F64 <"v_cmpsx_o_f64">;
defm V_CMPSX_U_F64 : VOPCX_F64 <"v_cmpsx_u_f64">;
defm V_CMPSX_NGE_F64 : VOPCX_F64 <"v_cmpsx_nge_f64", "v_cmpsx_nle_f64">;
defm V_CMPSX_NLG_F64 : VOPCX_F64 <"v_cmpsx_nlg_f64">;
defm V_CMPSX_NGT_F64 : VOPCX_F64 <"v_cmpsx_ngt_f64", "v_cmpsx_nlt_f64">;
defm V_CMPSX_NLE_F64 : VOPCX_F64 <"v_cmpsx_nle_f64">;
defm V_CMPSX_NEQ_F64 : VOPCX_F64 <"v_cmpsx_neq_f64">;
defm V_CMPSX_NLT_F64 : VOPCX_F64 <"v_cmpsx_nlt_f64">;
defm V_CMPSX_TRU_F64 : VOPCX_F64 <"v_cmpsx_tru_f64">;

let SubtargetPredicate = Has16BitInsts in {

defm V_CMP_F_F16    : VOPC_F16 <"v_cmp_f_f16">;
defm V_CMP_LT_F16   : VOPC_F16 <"v_cmp_lt_f16", COND_OLT, "v_cmp_gt_f16">;
defm V_CMP_EQ_F16   : VOPC_F16 <"v_cmp_eq_f16", COND_OEQ>;
defm V_CMP_LE_F16   : VOPC_F16 <"v_cmp_le_f16", COND_OLE, "v_cmp_ge_f16">;
defm V_CMP_GT_F16   : VOPC_F16 <"v_cmp_gt_f16", COND_OGT>;
defm V_CMP_LG_F16   : VOPC_F16 <"v_cmp_lg_f16", COND_ONE>;
defm V_CMP_GE_F16   : VOPC_F16 <"v_cmp_ge_f16", COND_OGE>;
defm V_CMP_O_F16    : VOPC_F16 <"v_cmp_o_f16", COND_O>;
defm V_CMP_U_F16    : VOPC_F16 <"v_cmp_u_f16", COND_UO>;
defm V_CMP_NGE_F16  : VOPC_F16 <"v_cmp_nge_f16", COND_ULT, "v_cmp_nle_f16">;
defm V_CMP_NLG_F16  : VOPC_F16 <"v_cmp_nlg_f16", COND_UEQ>;
defm V_CMP_NGT_F16  : VOPC_F16 <"v_cmp_ngt_f16", COND_ULE, "v_cmp_nlt_f16">;
defm V_CMP_NLE_F16  : VOPC_F16 <"v_cmp_nle_f16", COND_UGT>;
defm V_CMP_NEQ_F16  : VOPC_F16 <"v_cmp_neq_f16", COND_UNE>;
defm V_CMP_NLT_F16  : VOPC_F16 <"v_cmp_nlt_f16", COND_UGE>;
defm V_CMP_TRU_F16  : VOPC_F16 <"v_cmp_tru_f16">;

defm V_CMPX_F_F16   : VOPCX_F16 <"v_cmpx_f_f16">;
defm V_CMPX_LT_F16  : VOPCX_F16 <"v_cmpx_lt_f16", "v_cmpx_gt_f16">;
defm V_CMPX_EQ_F16  : VOPCX_F16 <"v_cmpx_eq_f16">;
defm V_CMPX_LE_F16  : VOPCX_F16 <"v_cmpx_le_f16", "v_cmpx_ge_f16">;
defm V_CMPX_GT_F16  : VOPCX_F16 <"v_cmpx_gt_f16">;
defm V_CMPX_LG_F16  : VOPCX_F16 <"v_cmpx_lg_f16">;
defm V_CMPX_GE_F16  : VOPCX_F16 <"v_cmpx_ge_f16">;
defm V_CMPX_O_F16   : VOPCX_F16 <"v_cmpx_o_f16">;
defm V_CMPX_U_F16   : VOPCX_F16 <"v_cmpx_u_f16">;
defm V_CMPX_NGE_F16 : VOPCX_F16 <"v_cmpx_nge_f16", "v_cmpx_nle_f16">;
defm V_CMPX_NLG_F16 : VOPCX_F16 <"v_cmpx_nlg_f16">;
defm V_CMPX_NGT_F16 : VOPCX_F16 <"v_cmpx_ngt_f16", "v_cmpx_nlt_f16">;
defm V_CMPX_NLE_F16 : VOPCX_F16 <"v_cmpx_nle_f16">;
defm V_CMPX_NEQ_F16 : VOPCX_F16 <"v_cmpx_neq_f16">;
defm V_CMPX_NLT_F16 : VOPCX_F16 <"v_cmpx_nlt_f16">;
defm V_CMPX_TRU_F16 : VOPCX_F16 <"v_cmpx_tru_f16">;

defm V_CMP_F_I16 : VOPC_I16 <"v_cmp_f_i16">;
defm V_CMP_LT_I16 : VOPC_I16 <"v_cmp_lt_i16", COND_SLT, "v_cmp_gt_i16">;
defm V_CMP_EQ_I16 : VOPC_I16 <"v_cmp_eq_i16">;
defm V_CMP_LE_I16 : VOPC_I16 <"v_cmp_le_i16", COND_SLE, "v_cmp_ge_i16">;
defm V_CMP_GT_I16 : VOPC_I16 <"v_cmp_gt_i16", COND_SGT>;
defm V_CMP_NE_I16 : VOPC_I16 <"v_cmp_ne_i16">;
defm V_CMP_GE_I16 : VOPC_I16 <"v_cmp_ge_i16", COND_SGE>;
defm V_CMP_T_I16 : VOPC_I16 <"v_cmp_t_i16">;

defm V_CMP_F_U16 : VOPC_I16 <"v_cmp_f_u16">;
defm V_CMP_LT_U16 : VOPC_I16 <"v_cmp_lt_u16", COND_ULT, "v_cmp_gt_u16">;
defm V_CMP_EQ_U16 : VOPC_I16 <"v_cmp_eq_u16", COND_EQ>;
defm V_CMP_LE_U16 : VOPC_I16 <"v_cmp_le_u16", COND_ULE, "v_cmp_ge_u16">;
defm V_CMP_GT_U16 : VOPC_I16 <"v_cmp_gt_u16", COND_UGT>;
defm V_CMP_NE_U16 : VOPC_I16 <"v_cmp_ne_u16", COND_NE>;
defm V_CMP_GE_U16 : VOPC_I16 <"v_cmp_ge_u16", COND_UGE>;
defm V_CMP_T_U16 : VOPC_I16 <"v_cmp_t_u16">;

defm V_CMPX_F_I16 : VOPCX_I16 <"v_cmpx_f_i16">;
defm V_CMPX_LT_I16 : VOPCX_I16 <"v_cmpx_lt_i16", "v_cmpx_gt_i16">;
defm V_CMPX_EQ_I16 : VOPCX_I16 <"v_cmpx_eq_i16">;
defm V_CMPX_LE_I16 : VOPCX_I16 <"v_cmpx_le_i16", "v_cmpx_ge_i16">;
defm V_CMPX_GT_I16 : VOPCX_I16 <"v_cmpx_gt_i16">;
defm V_CMPX_NE_I16 : VOPCX_I16 <"v_cmpx_ne_i16">;
defm V_CMPX_GE_I16 : VOPCX_I16 <"v_cmpx_ge_i16">;
defm V_CMPX_T_I16 : VOPCX_I16 <"v_cmpx_t_i16">;
defm V_CMPX_F_U16 : VOPCX_I16 <"v_cmpx_f_u16">;

defm V_CMPX_LT_U16 : VOPCX_I16 <"v_cmpx_lt_u16", "v_cmpx_gt_u16">;
defm V_CMPX_EQ_U16 : VOPCX_I16 <"v_cmpx_eq_u16">;
defm V_CMPX_LE_U16 : VOPCX_I16 <"v_cmpx_le_u16", "v_cmpx_ge_u16">;
defm V_CMPX_GT_U16 : VOPCX_I16 <"v_cmpx_gt_u16">;
defm V_CMPX_NE_U16 : VOPCX_I16 <"v_cmpx_ne_u16">;
defm V_CMPX_GE_U16 : VOPCX_I16 <"v_cmpx_ge_u16">;
defm V_CMPX_T_U16 : VOPCX_I16 <"v_cmpx_t_u16">;

} // End SubtargetPredicate = Has16BitInsts

defm V_CMP_F_I32 : VOPC_I32 <"v_cmp_f_i32">;
defm V_CMP_LT_I32 : VOPC_I32 <"v_cmp_lt_i32", COND_SLT, "v_cmp_gt_i32">;
defm V_CMP_EQ_I32 : VOPC_I32 <"v_cmp_eq_i32">;
defm V_CMP_LE_I32 : VOPC_I32 <"v_cmp_le_i32", COND_SLE, "v_cmp_ge_i32">;
defm V_CMP_GT_I32 : VOPC_I32 <"v_cmp_gt_i32", COND_SGT>;
defm V_CMP_NE_I32 : VOPC_I32 <"v_cmp_ne_i32">;
defm V_CMP_GE_I32 : VOPC_I32 <"v_cmp_ge_i32", COND_SGE>;
defm V_CMP_T_I32 : VOPC_I32 <"v_cmp_t_i32">;

defm V_CMPX_F_I32 : VOPCX_I32 <"v_cmpx_f_i32">;
defm V_CMPX_LT_I32 : VOPCX_I32 <"v_cmpx_lt_i32", "v_cmpx_gt_i32">;
defm V_CMPX_EQ_I32 : VOPCX_I32 <"v_cmpx_eq_i32">;
defm V_CMPX_LE_I32 : VOPCX_I32 <"v_cmpx_le_i32", "v_cmpx_ge_i32">;
defm V_CMPX_GT_I32 : VOPCX_I32 <"v_cmpx_gt_i32">;
defm V_CMPX_NE_I32 : VOPCX_I32 <"v_cmpx_ne_i32">;
defm V_CMPX_GE_I32 : VOPCX_I32 <"v_cmpx_ge_i32">;
defm V_CMPX_T_I32 : VOPCX_I32 <"v_cmpx_t_i32">;

defm V_CMP_F_I64 : VOPC_I64 <"v_cmp_f_i64">;
defm V_CMP_LT_I64 : VOPC_I64 <"v_cmp_lt_i64", COND_SLT, "v_cmp_gt_i64">;
defm V_CMP_EQ_I64 : VOPC_I64 <"v_cmp_eq_i64">;
defm V_CMP_LE_I64 : VOPC_I64 <"v_cmp_le_i64", COND_SLE, "v_cmp_ge_i64">;
defm V_CMP_GT_I64 : VOPC_I64 <"v_cmp_gt_i64", COND_SGT>;
defm V_CMP_NE_I64 : VOPC_I64 <"v_cmp_ne_i64">;
defm V_CMP_GE_I64 : VOPC_I64 <"v_cmp_ge_i64", COND_SGE>;
defm V_CMP_T_I64 : VOPC_I64 <"v_cmp_t_i64">;

defm V_CMPX_F_I64 : VOPCX_I64 <"v_cmpx_f_i64">;
defm V_CMPX_LT_I64 : VOPCX_I64 <"v_cmpx_lt_i64", "v_cmpx_gt_i64">;
defm V_CMPX_EQ_I64 : VOPCX_I64 <"v_cmpx_eq_i64">;
defm V_CMPX_LE_I64 : VOPCX_I64 <"v_cmpx_le_i64", "v_cmpx_ge_i64">;
defm V_CMPX_GT_I64 : VOPCX_I64 <"v_cmpx_gt_i64">;
defm V_CMPX_NE_I64 : VOPCX_I64 <"v_cmpx_ne_i64">;
defm V_CMPX_GE_I64 : VOPCX_I64 <"v_cmpx_ge_i64">;
defm V_CMPX_T_I64 : VOPCX_I64 <"v_cmpx_t_i64">;

defm V_CMP_F_U32 : VOPC_I32 <"v_cmp_f_u32">;
defm V_CMP_LT_U32 : VOPC_I32 <"v_cmp_lt_u32", COND_ULT, "v_cmp_gt_u32">;
defm V_CMP_EQ_U32 : VOPC_I32 <"v_cmp_eq_u32", COND_EQ>;
defm V_CMP_LE_U32 : VOPC_I32 <"v_cmp_le_u32", COND_ULE, "v_cmp_ge_u32">;
defm V_CMP_GT_U32 : VOPC_I32 <"v_cmp_gt_u32", COND_UGT>;
defm V_CMP_NE_U32 : VOPC_I32 <"v_cmp_ne_u32", COND_NE>;
defm V_CMP_GE_U32 : VOPC_I32 <"v_cmp_ge_u32", COND_UGE>;
defm V_CMP_T_U32 : VOPC_I32 <"v_cmp_t_u32">;

defm V_CMPX_F_U32 : VOPCX_I32 <"v_cmpx_f_u32">;
defm V_CMPX_LT_U32 : VOPCX_I32 <"v_cmpx_lt_u32", "v_cmpx_gt_u32">;
defm V_CMPX_EQ_U32 : VOPCX_I32 <"v_cmpx_eq_u32">;
defm V_CMPX_LE_U32 : VOPCX_I32 <"v_cmpx_le_u32", "v_cmpx_le_u32">;
defm V_CMPX_GT_U32 : VOPCX_I32 <"v_cmpx_gt_u32">;
defm V_CMPX_NE_U32 : VOPCX_I32 <"v_cmpx_ne_u32">;
defm V_CMPX_GE_U32 : VOPCX_I32 <"v_cmpx_ge_u32">;
defm V_CMPX_T_U32 : VOPCX_I32 <"v_cmpx_t_u32">;

defm V_CMP_F_U64 : VOPC_I64 <"v_cmp_f_u64">;
defm V_CMP_LT_U64 : VOPC_I64 <"v_cmp_lt_u64", COND_ULT, "v_cmp_gt_u64">;
defm V_CMP_EQ_U64 : VOPC_I64 <"v_cmp_eq_u64", COND_EQ>;
defm V_CMP_LE_U64 : VOPC_I64 <"v_cmp_le_u64", COND_ULE, "v_cmp_ge_u64">;
defm V_CMP_GT_U64 : VOPC_I64 <"v_cmp_gt_u64", COND_UGT>;
defm V_CMP_NE_U64 : VOPC_I64 <"v_cmp_ne_u64", COND_NE>;
defm V_CMP_GE_U64 : VOPC_I64 <"v_cmp_ge_u64", COND_UGE>;
defm V_CMP_T_U64 : VOPC_I64 <"v_cmp_t_u64">;

defm V_CMPX_F_U64 : VOPCX_I64 <"v_cmpx_f_u64">;
defm V_CMPX_LT_U64 : VOPCX_I64 <"v_cmpx_lt_u64", "v_cmpx_gt_u64">;
defm V_CMPX_EQ_U64 : VOPCX_I64 <"v_cmpx_eq_u64">;
defm V_CMPX_LE_U64 : VOPCX_I64 <"v_cmpx_le_u64", "v_cmpx_ge_u64">;
defm V_CMPX_GT_U64 : VOPCX_I64 <"v_cmpx_gt_u64">;
defm V_CMPX_NE_U64 : VOPCX_I64 <"v_cmpx_ne_u64">;
defm V_CMPX_GE_U64 : VOPCX_I64 <"v_cmpx_ge_u64">;
defm V_CMPX_T_U64 : VOPCX_I64 <"v_cmpx_t_u64">;

