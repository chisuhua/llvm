//===----------------------------------------------------------------------===//
// MUBUF Instructions
//===----------------------------------------------------------------------===//

defm BUFFER_LOAD_FORMAT_X : MUBUF_Pseudo_Loads_Lds <
  "buffer_load_format_x", f32
>;
defm BUFFER_LOAD_FORMAT_XY : MUBUF_Pseudo_Loads <
  "buffer_load_format_xy", v2f32
>;
// defm BUFFER_LOAD_FORMAT_XYZ : MUBUF_Pseudo_Loads <
//   "buffer_load_format_xyz", v3f32
// >;
// defm BUFFER_LOAD_FORMAT_XYZW : MUBUF_Pseudo_Loads <
//   "buffer_load_format_xyzw", v4f32
// >;
defm BUFFER_STORE_FORMAT_X : MUBUF_Pseudo_Stores <
  "buffer_store_format_x", f32
>;
defm BUFFER_STORE_FORMAT_XY : MUBUF_Pseudo_Stores <
  "buffer_store_format_xy", v2f32
>;
// defm BUFFER_STORE_FORMAT_XYZ : MUBUF_Pseudo_Stores <
//   "buffer_store_format_xyz", v3f32
// >;
// defm BUFFER_STORE_FORMAT_XYZW : MUBUF_Pseudo_Stores <
//   "buffer_store_format_xyzw", v4f32
// >;

let SubtargetPredicate = HasUnpackedD16VMem, D16Buf = 1 in {
  defm BUFFER_LOAD_FORMAT_D16_X_gfx80 : MUBUF_Pseudo_Loads <
    "buffer_load_format_d16_x", i32
  >;
  defm BUFFER_LOAD_FORMAT_D16_XY_gfx80 : MUBUF_Pseudo_Loads <
    "buffer_load_format_d16_xy", v2i32
  >;
  // defm BUFFER_LOAD_FORMAT_D16_XYZ_gfx80 : MUBUF_Pseudo_Loads <
  //   "buffer_load_format_d16_xyz", v3i32
  // >;
  // defm BUFFER_LOAD_FORMAT_D16_XYZW_gfx80 : MUBUF_Pseudo_Loads <
  //  "buffer_load_format_d16_xyzw", v4i32
  // >;
  defm BUFFER_STORE_FORMAT_D16_X_gfx80 : MUBUF_Pseudo_Stores <
    "buffer_store_format_d16_x", i32
  >;
  defm BUFFER_STORE_FORMAT_D16_XY_gfx80 : MUBUF_Pseudo_Stores <
    "buffer_store_format_d16_xy", v2i32
  >;
  // defm BUFFER_STORE_FORMAT_D16_XYZ_gfx80 : MUBUF_Pseudo_Stores <
  //   "buffer_store_format_d16_xyz", v3i32
  // >;
  // defm BUFFER_STORE_FORMAT_D16_XYZW_gfx80 : MUBUF_Pseudo_Stores <
  //   "buffer_store_format_d16_xyzw", v4i32
  // >;
} // End HasUnpackedD16VMem.

let SubtargetPredicate = HasPackedD16VMem, D16Buf = 1 in {
  defm BUFFER_LOAD_FORMAT_D16_X : MUBUF_Pseudo_Loads <
    "buffer_load_format_d16_x", f16
  >;
  defm BUFFER_LOAD_FORMAT_D16_XY : MUBUF_Pseudo_Loads <
    "buffer_load_format_d16_xy", v2f16
  >;
  defm BUFFER_LOAD_FORMAT_D16_XYZ : MUBUF_Pseudo_Loads <
    "buffer_load_format_d16_xyz", v3f16
  >;
  defm BUFFER_LOAD_FORMAT_D16_XYZW : MUBUF_Pseudo_Loads <
    "buffer_load_format_d16_xyzw", v4f16
  >;
  defm BUFFER_STORE_FORMAT_D16_X : MUBUF_Pseudo_Stores <
    "buffer_store_format_d16_x", f16
  >;
  defm BUFFER_STORE_FORMAT_D16_XY : MUBUF_Pseudo_Stores <
    "buffer_store_format_d16_xy", v2f16
  >;
  defm BUFFER_STORE_FORMAT_D16_XYZ : MUBUF_Pseudo_Stores <
    "buffer_store_format_d16_xyz", v3f16
  >;
  defm BUFFER_STORE_FORMAT_D16_XYZW : MUBUF_Pseudo_Stores <
    "buffer_store_format_d16_xyzw", v4f16
  >;
} // End HasPackedD16VMem.

defm BUFFER_LOAD_UBYTE : MUBUF_Pseudo_Loads_Lds <
  "buffer_load_ubyte", i32
>;
defm BUFFER_LOAD_SBYTE : MUBUF_Pseudo_Loads_Lds <
  "buffer_load_sbyte", i32
>;
defm BUFFER_LOAD_USHORT : MUBUF_Pseudo_Loads_Lds <
  "buffer_load_ushort", i32
>;
defm BUFFER_LOAD_SSHORT : MUBUF_Pseudo_Loads_Lds <
  "buffer_load_sshort", i32
>;
defm BUFFER_LOAD_DWORD : MUBUF_Pseudo_Loads_Lds <
  "buffer_load_dword", i32
>;
defm BUFFER_LOAD_DWORDX2 : MUBUF_Pseudo_Loads_Lds <
  "buffer_load_dwordx2", v2i32
>;
// defm BUFFER_LOAD_DWORDX3 : MUBUF_Pseudo_Loads <
//   "buffer_load_dwordx3", v3i32
// >;
// defm BUFFER_LOAD_DWORDX4 : MUBUF_Pseudo_Loads <
//  "buffer_load_dwordx4", v4i32
// >;

defm : MUBUF_Pseudo_Load_Pats<"BUFFER_LOAD_UBYTE", i32, extloadi8_global>;
defm : MUBUF_Pseudo_Load_Pats<"BUFFER_LOAD_UBYTE", i32, zextloadi8_global>;
defm : MUBUF_Pseudo_Load_Pats<"BUFFER_LOAD_SBYTE", i32, sextloadi8_global>;
defm : MUBUF_Pseudo_Load_Pats<"BUFFER_LOAD_USHORT", i32, extloadi16_global>;
defm : MUBUF_Pseudo_Load_Pats<"BUFFER_LOAD_USHORT", i32, zextloadi16_global>;
defm : MUBUF_Pseudo_Load_Pats<"BUFFER_LOAD_SSHORT", i32, sextloadi16_global>;
defm : MUBUF_Pseudo_Load_Pats<"BUFFER_LOAD_DWORD", i32, load_global>;
defm : MUBUF_Pseudo_Load_Pats<"BUFFER_LOAD_DWORDX2", v2i32, load_global>;
//defm : MUBUF_Pseudo_Load_Pats<"BUFFER_LOAD_DWORDX3", v3i32, load_global>;
//defm : MUBUF_Pseudo_Load_Pats<"BUFFER_LOAD_DWORDX4", v4i32, load_global>;

// This is not described in AMD documentation,
// but 'lds' versions of these opcodes are available
// in at least GFX8+ chips. See Bug 37653.
/* TODO
let SubtargetPredicate = isGFX8GFX9 in {
defm BUFFER_LOAD_DWORDX2_LDS : MUBUF_Pseudo_Loads <
  "buffer_load_dwordx2", v2i32, null_frag, 0, 1
>;
defm BUFFER_LOAD_DWORDX3_LDS : MUBUF_Pseudo_Loads <
  "buffer_load_dwordx3", v3i32, null_frag, 0, 1
>;
defm BUFFER_LOAD_DWORDX4_LDS : MUBUF_Pseudo_Loads <
  "buffer_load_dwordx4", v4i32, null_frag, 0, 1
>;
}
*/

defm BUFFER_STORE_BYTE : MUBUF_Pseudo_Stores <
   "buffer_store_byte", i32, truncstorei8_global
>;
defm BUFFER_STORE_SHORT : MUBUF_Pseudo_Stores <
  "buffer_store_short", i32, truncstorei16_global
>;
defm BUFFER_STORE_DWORD : MUBUF_Pseudo_Stores <
  "buffer_store_dword", i32, store_global
>;
defm BUFFER_STORE_DWORDX2 : MUBUF_Pseudo_Stores <
  "buffer_store_dwordx2", v2i32, store_global
>;
// defm BUFFER_STORE_DWORDX3 : MUBUF_Pseudo_Stores <
//   "buffer_store_dwordx3", v3i32, store_global
// >;
// defm BUFFER_STORE_DWORDX4 : MUBUF_Pseudo_Stores <
//   "buffer_store_dwordx4", v4i32, store_global
// >;

defm BUFFER_ATOMIC_SWAP : MUBUF_Pseudo_Atomics <
  "buffer_atomic_swap", VPR_32, i32, atomic_swap_global_32
>;
defm BUFFER_ATOMIC_CMPSWAP : MUBUF_Pseudo_Atomics <
  "buffer_atomic_cmpswap", VReg_64, v2i32, null_frag
>;
defm BUFFER_ATOMIC_ADD : MUBUF_Pseudo_Atomics <
  "buffer_atomic_add", VPR_32, i32, atomic_load_add_global_32
>;
defm BUFFER_ATOMIC_SUB : MUBUF_Pseudo_Atomics <
  "buffer_atomic_sub", VPR_32, i32, atomic_load_sub_global_32
>;
defm BUFFER_ATOMIC_SMIN : MUBUF_Pseudo_Atomics <
  "buffer_atomic_smin", VPR_32, i32, atomic_load_min_global_32
>;
defm BUFFER_ATOMIC_UMIN : MUBUF_Pseudo_Atomics <
  "buffer_atomic_umin", VPR_32, i32, atomic_load_umin_global_32
>;
defm BUFFER_ATOMIC_SMAX : MUBUF_Pseudo_Atomics <
  "buffer_atomic_smax", VPR_32, i32, atomic_load_max_global_32
>;
defm BUFFER_ATOMIC_UMAX : MUBUF_Pseudo_Atomics <
  "buffer_atomic_umax", VPR_32, i32, atomic_load_umax_global_32
>;
defm BUFFER_ATOMIC_AND : MUBUF_Pseudo_Atomics <
  "buffer_atomic_and", VPR_32, i32, atomic_load_and_global_32
>;
defm BUFFER_ATOMIC_OR : MUBUF_Pseudo_Atomics <
  "buffer_atomic_or", VPR_32, i32, atomic_load_or_global_32
>;
defm BUFFER_ATOMIC_XOR : MUBUF_Pseudo_Atomics <
  "buffer_atomic_xor", VPR_32, i32, atomic_load_xor_global_32
>;
defm BUFFER_ATOMIC_INC : MUBUF_Pseudo_Atomics <
  "buffer_atomic_inc", VPR_32, i32, atomic_inc_global_32
>;
defm BUFFER_ATOMIC_DEC : MUBUF_Pseudo_Atomics <
  "buffer_atomic_dec", VPR_32, i32, atomic_dec_global_32
>;
defm BUFFER_ATOMIC_SWAP_X2 : MUBUF_Pseudo_Atomics <
  "buffer_atomic_swap_x2", VReg_64, i64, atomic_swap_global_64
>;
/*
defm BUFFER_ATOMIC_CMPSWAP_X2 : MUBUF_Pseudo_Atomics <
  "buffer_atomic_cmpswap_x2", VReg_128, v2i64, null_frag
>;
*/
defm BUFFER_ATOMIC_ADD_X2 : MUBUF_Pseudo_Atomics <
  "buffer_atomic_add_x2", VReg_64, i64, atomic_load_add_global_64
>;
defm BUFFER_ATOMIC_SUB_X2 : MUBUF_Pseudo_Atomics <
  "buffer_atomic_sub_x2", VReg_64, i64, atomic_load_sub_global_64
>;
defm BUFFER_ATOMIC_SMIN_X2 : MUBUF_Pseudo_Atomics <
  "buffer_atomic_smin_x2", VReg_64, i64, atomic_load_min_global_64
>;
defm BUFFER_ATOMIC_UMIN_X2 : MUBUF_Pseudo_Atomics <
  "buffer_atomic_umin_x2", VReg_64, i64, atomic_load_umin_global_64
>;
defm BUFFER_ATOMIC_SMAX_X2 : MUBUF_Pseudo_Atomics <
  "buffer_atomic_smax_x2", VReg_64, i64, atomic_load_max_global_64
>;
defm BUFFER_ATOMIC_UMAX_X2 : MUBUF_Pseudo_Atomics <
  "buffer_atomic_umax_x2", VReg_64, i64, atomic_load_umax_global_64
>;
defm BUFFER_ATOMIC_AND_X2 : MUBUF_Pseudo_Atomics <
  "buffer_atomic_and_x2", VReg_64, i64, atomic_load_and_global_64
>;
defm BUFFER_ATOMIC_OR_X2 : MUBUF_Pseudo_Atomics <
  "buffer_atomic_or_x2", VReg_64, i64, atomic_load_or_global_64
>;
defm BUFFER_ATOMIC_XOR_X2 : MUBUF_Pseudo_Atomics <
  "buffer_atomic_xor_x2", VReg_64, i64, atomic_load_xor_global_64
>;
defm BUFFER_ATOMIC_INC_X2 : MUBUF_Pseudo_Atomics <
  "buffer_atomic_inc_x2", VReg_64, i64, atomic_inc_global_64
>;
defm BUFFER_ATOMIC_DEC_X2 : MUBUF_Pseudo_Atomics <
  "buffer_atomic_dec_x2", VReg_64, i64, atomic_dec_global_64
>;

/*
// let SubtargetPredicate = isGFX8GFX9 in {
def BUFFER_STORE_LDS_DWORD : MUBUF_Pseudo_Store_Lds <"buffer_store_lds_dword">;
//}

let SubtargetPredicate = isGFX6 in { // isn't on CI & VI
defm BUFFER_ATOMIC_RSUB        : MUBUF_Pseudo_Atomics <"buffer_atomic_rsub">;
defm BUFFER_ATOMIC_FCMPSWAP    : MUBUF_Pseudo_Atomics <"buffer_atomic_fcmpswap">;
defm BUFFER_ATOMIC_FMIN        : MUBUF_Pseudo_Atomics <"buffer_atomic_fmin">;
defm BUFFER_ATOMIC_FMAX        : MUBUF_Pseudo_Atomics <"buffer_atomic_fmax">;
defm BUFFER_ATOMIC_RSUB_X2     : MUBUF_Pseudo_Atomics <"buffer_atomic_rsub_x2">;
defm BUFFER_ATOMIC_FCMPSWAP_X2 : MUBUF_Pseudo_Atomics <"buffer_atomic_fcmpswap_x2">;
defm BUFFER_ATOMIC_FMIN_X2     : MUBUF_Pseudo_Atomics <"buffer_atomic_fmin_x2">;
defm BUFFER_ATOMIC_FMAX_X2     : MUBUF_Pseudo_Atomics <"buffer_atomic_fmax_x2">;

def BUFFER_WBINVL1_SC : MUBUF_Invalidate <"buffer_wbinvl1_sc",
                                          int_amdgcn_buffer_wbinvl1_sc>;
}
*/

let SubtargetPredicate = HasD16LoadStore in {

defm BUFFER_LOAD_UBYTE_D16 : MUBUF_Pseudo_Loads <
  "buffer_load_ubyte_d16", i32, null_frag, 1
>;

defm BUFFER_LOAD_UBYTE_D16_HI : MUBUF_Pseudo_Loads <
  "buffer_load_ubyte_d16_hi", i32, null_frag, 1
>;

defm BUFFER_LOAD_SBYTE_D16 : MUBUF_Pseudo_Loads <
  "buffer_load_sbyte_d16", i32, null_frag, 1
>;

defm BUFFER_LOAD_SBYTE_D16_HI : MUBUF_Pseudo_Loads <
  "buffer_load_sbyte_d16_hi", i32, null_frag, 1
>;

defm BUFFER_LOAD_SHORT_D16 : MUBUF_Pseudo_Loads <
  "buffer_load_short_d16", i32, null_frag, 1
>;

defm BUFFER_LOAD_SHORT_D16_HI : MUBUF_Pseudo_Loads <
  "buffer_load_short_d16_hi", i32, null_frag, 1
>;

defm BUFFER_STORE_BYTE_D16_HI : MUBUF_Pseudo_Stores <
  "buffer_store_byte_d16_hi", i32
>;

defm BUFFER_STORE_SHORT_D16_HI : MUBUF_Pseudo_Stores <
  "buffer_store_short_d16_hi", i32
>;

defm BUFFER_LOAD_FORMAT_D16_HI_X : MUBUF_Pseudo_Loads <
  "buffer_load_format_d16_hi_x", i32
>;
defm BUFFER_STORE_FORMAT_D16_HI_X : MUBUF_Pseudo_Stores <
  "buffer_store_format_d16_hi_x", i32
>;

} // End HasD16LoadStore
/* TODO
def BUFFER_WBINVL1 : MUBUF_Invalidate <"buffer_wbinvl1",
                                       int_amdgcn_buffer_wbinvl1>;

let SubtargetPredicate = HasAtomicFaddInsts in {

defm BUFFER_ATOMIC_ADD_F32 : MUBUF_Pseudo_Atomics_NO_RTN <
  "buffer_atomic_add_f32", VPR_32, f32, atomic_fadd_global_noret
>;
defm BUFFER_ATOMIC_PK_ADD_F16 : MUBUF_Pseudo_Atomics_NO_RTN <
  "buffer_atomic_pk_add_f16", VPR_32, v2f16, atomic_pk_fadd_global_noret
>;

} // End SubtargetPredicate = HasAtomicFaddInsts
*/


//===----------------------------------------------------------------------===//
// MTBUF Instructions
//===----------------------------------------------------------------------===//

defm TBUFFER_LOAD_FORMAT_X     : MTBUF_Pseudo_Loads  <"tbuffer_load_format_x",     VPR_32>;
defm TBUFFER_LOAD_FORMAT_XY    : MTBUF_Pseudo_Loads  <"tbuffer_load_format_xy",    VReg_64>;
// defm TBUFFER_LOAD_FORMAT_XYZ   : MTBUF_Pseudo_Loads  <"tbuffer_load_format_xyz",   VReg_96>;
// defm TBUFFER_LOAD_FORMAT_XYZW  : MTBUF_Pseudo_Loads  <"tbuffer_load_format_xyzw",  VReg_128>;
defm TBUFFER_STORE_FORMAT_X    : MTBUF_Pseudo_Stores <"tbuffer_store_format_x",    VPR_32>;
defm TBUFFER_STORE_FORMAT_XY   : MTBUF_Pseudo_Stores <"tbuffer_store_format_xy",   VReg_64>;
// defm TBUFFER_STORE_FORMAT_XYZ  : MTBUF_Pseudo_Stores <"tbuffer_store_format_xyz",  VReg_96>;
// defm TBUFFER_STORE_FORMAT_XYZW : MTBUF_Pseudo_Stores <"tbuffer_store_format_xyzw", VReg_128>;

let SubtargetPredicate = HasUnpackedD16VMem, D16Buf = 1 in {
  defm TBUFFER_LOAD_FORMAT_D16_X_gfx80     : MTBUF_Pseudo_Loads  <"tbuffer_load_format_d16_x",     VPR_32>;
  defm TBUFFER_LOAD_FORMAT_D16_XY_gfx80    : MTBUF_Pseudo_Loads  <"tbuffer_load_format_d16_xy",    VReg_64>;
  // defm TBUFFER_LOAD_FORMAT_D16_XYZ_gfx80   : MTBUF_Pseudo_Loads  <"tbuffer_load_format_d16_xyz",   VReg_96>;
  // defm TBUFFER_LOAD_FORMAT_D16_XYZW_gfx80  : MTBUF_Pseudo_Loads  <"tbuffer_load_format_d16_xyzw",  VReg_128>;
  defm TBUFFER_STORE_FORMAT_D16_X_gfx80    : MTBUF_Pseudo_Stores <"tbuffer_store_format_d16_x",    VPR_32>;
  defm TBUFFER_STORE_FORMAT_D16_XY_gfx80   : MTBUF_Pseudo_Stores <"tbuffer_store_format_d16_xy",   VReg_64>;
  // defm TBUFFER_STORE_FORMAT_D16_XYZ_gfx80  : MTBUF_Pseudo_Stores <"tbuffer_store_format_d16_xyz",  VReg_96>;
  // defm TBUFFER_STORE_FORMAT_D16_XYZW_gfx80 : MTBUF_Pseudo_Stores <"tbuffer_store_format_d16_xyzw", VReg_128>;
} // End HasUnpackedD16VMem.

let SubtargetPredicate = HasPackedD16VMem, D16Buf = 1 in {
  defm TBUFFER_LOAD_FORMAT_D16_X     : MTBUF_Pseudo_Loads  <"tbuffer_load_format_d16_x",     VPR_32>;
  defm TBUFFER_LOAD_FORMAT_D16_XY    : MTBUF_Pseudo_Loads  <"tbuffer_load_format_d16_xy",    VPR_32>;
  // defm TBUFFER_LOAD_FORMAT_D16_XYZ   : MTBUF_Pseudo_Loads  <"tbuffer_load_format_d16_xyz",   VReg_64>;
  // defm TBUFFER_LOAD_FORMAT_D16_XYZW  : MTBUF_Pseudo_Loads  <"tbuffer_load_format_d16_xyzw",  VReg_64>;
  defm TBUFFER_STORE_FORMAT_D16_X    : MTBUF_Pseudo_Stores <"tbuffer_store_format_d16_x",    VPR_32>;
  defm TBUFFER_STORE_FORMAT_D16_XY   : MTBUF_Pseudo_Stores <"tbuffer_store_format_d16_xy",   VPR_32>;
  // defm TBUFFER_STORE_FORMAT_D16_XYZ  : MTBUF_Pseudo_Stores <"tbuffer_store_format_d16_xyz",  VReg_64>;
  // defm TBUFFER_STORE_FORMAT_D16_XYZW : MTBUF_Pseudo_Stores <"tbuffer_store_format_d16_xyzw", VReg_64>;
} // End HasPackedD16VMem.


// let SubtargetPredicate = isGFX7Plus in {
//===----------------------------------------------------------------------===//
// Instruction definitions for CI and newer.
//===----------------------------------------------------------------------===//
// TODO
// def BUFFER_WBINVL1_VOL : MUBUF_Invalidate <"buffer_wbinvl1_vol",
//                                            int_amdgcn_buffer_wbinvl1_vol>;

// } // End let SubtargetPredicate = isGFX7Plus

// let SubtargetPredicate = isGFX10Plus in {
//   def BUFFER_GL0_INV : MUBUF_Invalidate<"buffer_gl0_inv">;
//   def BUFFER_GL1_INV : MUBUF_Invalidate<"buffer_gl1_inv">;
// } // End SubtargetPredicate = isGFX10Plus

//===----------------------------------------------------------------------===//
// MUBUF Patterns
//===----------------------------------------------------------------------===//

def extract_glc : SDNodeXForm<imm, [{
  return CurDAG->getTargetConstant(N->getZExtValue() & 1, SDLoc(N), MVT::i8);
}]>;

def extract_slc : SDNodeXForm<imm, [{
  return CurDAG->getTargetConstant((N->getZExtValue() >> 1) & 1, SDLoc(N), MVT::i8);
}]>;

def extract_dlc : SDNodeXForm<imm, [{
  return CurDAG->getTargetConstant((N->getZExtValue() >> 2) & 1, SDLoc(N), MVT::i8);
}]>;




//===----------------------------------------------------------------------===//
// buffer_load/store_format patterns
//===----------------------------------------------------------------------===//
// TODO change rsrc from v4i32 to v2i32
multiclass MUBUF_LoadIntrinsicPat<SDPatternOperator name, ValueType vt,
                                  string opcode> {
  def : PPTPat<
    (vt (name v2i32:$rsrc, 0, 0, i32:$soffset, imm:$offset,
              imm:$cachepolicy, 0)),
    (!cast<MUBUF_Pseudo>(opcode # _OFFSET) $rsrc, $soffset, (as_i16imm $offset),
      (extract_glc $cachepolicy), (extract_slc $cachepolicy), 0,  (extract_dlc $cachepolicy))
  >;
  def : PPTPat<
    (vt (name v2i32:$rsrc, 0, i32:$voffset, i32:$soffset, imm:$offset,
              imm:$cachepolicy, 0)),
    (!cast<MUBUF_Pseudo>(opcode # _OFFEN) $voffset, $rsrc, $soffset, (as_i16imm $offset),
      (extract_glc $cachepolicy), (extract_slc $cachepolicy), 0,  (extract_dlc $cachepolicy))
  >;

  def : PPTPat<
    (vt (name v2i32:$rsrc, i32:$vindex, 0, i32:$soffset, imm:$offset,
              imm:$cachepolicy, imm)),
    (!cast<MUBUF_Pseudo>(opcode # _IDXEN) $vindex, $rsrc, $soffset, (as_i16imm $offset),
      (extract_glc $cachepolicy), (extract_slc $cachepolicy), 0,  (extract_dlc $cachepolicy))
  >;

  def : PPTPat<
    (vt (name v2i32:$rsrc, i32:$vindex, i32:$voffset, i32:$soffset, imm:$offset,
              imm:$cachepolicy, imm)),
    (!cast<MUBUF_Pseudo>(opcode # _BOTHEN)
      (REG_SEQUENCE VReg_64, $vindex, sub0, $voffset, sub1),
      $rsrc, $soffset, (as_i16imm $offset),
      (extract_glc $cachepolicy), (extract_slc $cachepolicy), 0,  (extract_dlc $cachepolicy))
  >;
}

defm : MUBUF_LoadIntrinsicPat<SIbuffer_load_format, f32, "BUFFER_LOAD_FORMAT_X">;
defm : MUBUF_LoadIntrinsicPat<SIbuffer_load_format, i32, "BUFFER_LOAD_FORMAT_X">;
defm : MUBUF_LoadIntrinsicPat<SIbuffer_load_format, v2f32, "BUFFER_LOAD_FORMAT_XY">;
defm : MUBUF_LoadIntrinsicPat<SIbuffer_load_format, v2i32, "BUFFER_LOAD_FORMAT_XY">;
// defm : MUBUF_LoadIntrinsicPat<SIbuffer_load_format, v3f32, "BUFFER_LOAD_FORMAT_XYZ">;
// defm : MUBUF_LoadIntrinsicPat<SIbuffer_load_format, v3i32, "BUFFER_LOAD_FORMAT_XYZ">;
// defm : MUBUF_LoadIntrinsicPat<SIbuffer_load_format, v4f32, "BUFFER_LOAD_FORMAT_XYZW">;
// defm : MUBUF_LoadIntrinsicPat<SIbuffer_load_format, v4i32, "BUFFER_LOAD_FORMAT_XYZW">;

let SubtargetPredicate = HasUnpackedD16VMem in {
  defm : MUBUF_LoadIntrinsicPat<SIbuffer_load_format_d16, f16, "BUFFER_LOAD_FORMAT_D16_X_gfx80">;
  defm : MUBUF_LoadIntrinsicPat<SIbuffer_load_format_d16, i16, "BUFFER_LOAD_FORMAT_D16_X_gfx80">;
  defm : MUBUF_LoadIntrinsicPat<SIbuffer_load_format_d16, v2i32, "BUFFER_LOAD_FORMAT_D16_XY_gfx80">;
  // defm : MUBUF_LoadIntrinsicPat<SIbuffer_load_format_d16, v4i32, "BUFFER_LOAD_FORMAT_D16_XYZW_gfx80">;
} // End HasUnpackedD16VMem.

let SubtargetPredicate = HasPackedD16VMem in {
  defm : MUBUF_LoadIntrinsicPat<SIbuffer_load_format_d16, f16, "BUFFER_LOAD_FORMAT_D16_X">;
  defm : MUBUF_LoadIntrinsicPat<SIbuffer_load_format_d16, i16, "BUFFER_LOAD_FORMAT_D16_X">;
  defm : MUBUF_LoadIntrinsicPat<SIbuffer_load_format_d16, v2f16, "BUFFER_LOAD_FORMAT_D16_XY">;
  defm : MUBUF_LoadIntrinsicPat<SIbuffer_load_format_d16, v2i16, "BUFFER_LOAD_FORMAT_D16_XY">;
  // defm : MUBUF_LoadIntrinsicPat<SIbuffer_load_format_d16, v4f16, "BUFFER_LOAD_FORMAT_D16_XYZW">;
  // defm : MUBUF_LoadIntrinsicPat<SIbuffer_load_format_d16, v4i16, "BUFFER_LOAD_FORMAT_D16_XYZW">;
} // End HasPackedD16VMem.

defm : MUBUF_LoadIntrinsicPat<SIbuffer_load, f32, "BUFFER_LOAD_DWORD">;
defm : MUBUF_LoadIntrinsicPat<SIbuffer_load, i32, "BUFFER_LOAD_DWORD">;
defm : MUBUF_LoadIntrinsicPat<SIbuffer_load, v2i16, "BUFFER_LOAD_DWORD">;
defm : MUBUF_LoadIntrinsicPat<SIbuffer_load, v2f16, "BUFFER_LOAD_DWORD">;
defm : MUBUF_LoadIntrinsicPat<SIbuffer_load, v2f32, "BUFFER_LOAD_DWORDX2">;
defm : MUBUF_LoadIntrinsicPat<SIbuffer_load, v2i32, "BUFFER_LOAD_DWORDX2">;
defm : MUBUF_LoadIntrinsicPat<SIbuffer_load, v4i16, "BUFFER_LOAD_DWORDX2">;
defm : MUBUF_LoadIntrinsicPat<SIbuffer_load, v4f16, "BUFFER_LOAD_DWORDX2">;
// defm : MUBUF_LoadIntrinsicPat<SIbuffer_load, v3f32, "BUFFER_LOAD_DWORDX3">;
// defm : MUBUF_LoadIntrinsicPat<SIbuffer_load, v3i32, "BUFFER_LOAD_DWORDX3">;
// defm : MUBUF_LoadIntrinsicPat<SIbuffer_load, v4f32, "BUFFER_LOAD_DWORDX4">;
// defm : MUBUF_LoadIntrinsicPat<SIbuffer_load, v4i32, "BUFFER_LOAD_DWORDX4">;
defm : MUBUF_LoadIntrinsicPat<SIbuffer_load_byte, i32, "BUFFER_LOAD_SBYTE">;
defm : MUBUF_LoadIntrinsicPat<SIbuffer_load_short, i32, "BUFFER_LOAD_SSHORT">;
defm : MUBUF_LoadIntrinsicPat<SIbuffer_load_ubyte, i32, "BUFFER_LOAD_UBYTE">;
defm : MUBUF_LoadIntrinsicPat<SIbuffer_load_ushort,  i32, "BUFFER_LOAD_USHORT">;

multiclass MUBUF_StoreIntrinsicPat<SDPatternOperator name, ValueType vt,
                                   string opcode> {
  def : PPTPat<
    (name vt:$vdata, v2i32:$rsrc, 0, 0, i32:$soffset, imm:$offset,
              imm:$cachepolicy, 0),
    (!cast<MUBUF_Pseudo>(opcode # _OFFSET_exact) $vdata, $rsrc, $soffset, (as_i16imm $offset),
      (extract_glc $cachepolicy), (extract_slc $cachepolicy), 0,  (extract_dlc $cachepolicy))
  >;

  def : PPTPat<
    (name vt:$vdata, v2i32:$rsrc, 0, i32:$voffset, i32:$soffset, imm:$offset,
              imm:$cachepolicy, 0),
    (!cast<MUBUF_Pseudo>(opcode # _OFFEN_exact) $vdata, $voffset, $rsrc, $soffset,
      (as_i16imm $offset), (extract_glc $cachepolicy),
      (extract_slc $cachepolicy), 0,  (extract_dlc $cachepolicy))
  >;

  def : PPTPat<
    (name vt:$vdata, v2i32:$rsrc, i32:$vindex, 0, i32:$soffset, imm:$offset,
              imm:$cachepolicy, imm),
    (!cast<MUBUF_Pseudo>(opcode # _IDXEN_exact) $vdata, $vindex, $rsrc, $soffset,
      (as_i16imm $offset), (extract_glc $cachepolicy),
      (extract_slc $cachepolicy), 0,  (extract_dlc $cachepolicy))
  >;

  def : PPTPat<
    (name vt:$vdata, v2i32:$rsrc, i32:$vindex, i32:$voffset, i32:$soffset, imm:$offset,
              imm:$cachepolicy, imm),
    (!cast<MUBUF_Pseudo>(opcode # _BOTHEN_exact)
      $vdata,
      (REG_SEQUENCE VReg_64, $vindex, sub0, $voffset, sub1),
      $rsrc, $soffset, (as_i16imm $offset), (extract_glc $cachepolicy),
      (extract_slc $cachepolicy), 0,  (extract_dlc $cachepolicy))
  >;
}

defm : MUBUF_StoreIntrinsicPat<SIbuffer_store_format, f32, "BUFFER_STORE_FORMAT_X">;
defm : MUBUF_StoreIntrinsicPat<SIbuffer_store_format, i32, "BUFFER_STORE_FORMAT_X">;
defm : MUBUF_StoreIntrinsicPat<SIbuffer_store_format, v2f32, "BUFFER_STORE_FORMAT_XY">;
defm : MUBUF_StoreIntrinsicPat<SIbuffer_store_format, v2i32, "BUFFER_STORE_FORMAT_XY">;
// defm : MUBUF_StoreIntrinsicPat<SIbuffer_store_format, v3f32, "BUFFER_STORE_FORMAT_XYZ">;
// defm : MUBUF_StoreIntrinsicPat<SIbuffer_store_format, v3i32, "BUFFER_STORE_FORMAT_XYZ">;
// defm : MUBUF_StoreIntrinsicPat<SIbuffer_store_format, v4f32, "BUFFER_STORE_FORMAT_XYZW">;
// defm : MUBUF_StoreIntrinsicPat<SIbuffer_store_format, v4i32, "BUFFER_STORE_FORMAT_XYZW">;

let SubtargetPredicate = HasUnpackedD16VMem in {
  defm : MUBUF_StoreIntrinsicPat<SIbuffer_store_format_d16, f16, "BUFFER_STORE_FORMAT_D16_X_gfx80">;
  defm : MUBUF_StoreIntrinsicPat<SIbuffer_store_format_d16, i16, "BUFFER_STORE_FORMAT_D16_X_gfx80">;
  defm : MUBUF_StoreIntrinsicPat<SIbuffer_store_format_d16, v2i32, "BUFFER_STORE_FORMAT_D16_XY_gfx80">;
  // defm : MUBUF_StoreIntrinsicPat<SIbuffer_store_format_d16, v4i32, "BUFFER_STORE_FORMAT_D16_XYZW_gfx80">;
} // End HasUnpackedD16VMem.

let SubtargetPredicate = HasPackedD16VMem in {
  defm : MUBUF_StoreIntrinsicPat<SIbuffer_store_format_d16, f16, "BUFFER_STORE_FORMAT_D16_X">;
  defm : MUBUF_StoreIntrinsicPat<SIbuffer_store_format_d16, i16, "BUFFER_STORE_FORMAT_D16_X">;
  defm : MUBUF_StoreIntrinsicPat<SIbuffer_store_format_d16, v2f16, "BUFFER_STORE_FORMAT_D16_XY">;
  defm : MUBUF_StoreIntrinsicPat<SIbuffer_store_format_d16, v2i16, "BUFFER_STORE_FORMAT_D16_XY">;
  // defm : MUBUF_StoreIntrinsicPat<SIbuffer_store_format_d16, v4f16, "BUFFER_STORE_FORMAT_D16_XYZW">;
  // defm : MUBUF_StoreIntrinsicPat<SIbuffer_store_format_d16, v4i16, "BUFFER_STORE_FORMAT_D16_XYZW">;
} // End HasPackedD16VMem.

defm : MUBUF_StoreIntrinsicPat<SIbuffer_store, f32, "BUFFER_STORE_DWORD">;
defm : MUBUF_StoreIntrinsicPat<SIbuffer_store, i32, "BUFFER_STORE_DWORD">;
defm : MUBUF_StoreIntrinsicPat<SIbuffer_store, v2i16, "BUFFER_STORE_DWORD">;
defm : MUBUF_StoreIntrinsicPat<SIbuffer_store, v2f16, "BUFFER_STORE_DWORD">;
defm : MUBUF_StoreIntrinsicPat<SIbuffer_store, v2f32, "BUFFER_STORE_DWORDX2">;
defm : MUBUF_StoreIntrinsicPat<SIbuffer_store, v2i32, "BUFFER_STORE_DWORDX2">;
defm : MUBUF_StoreIntrinsicPat<SIbuffer_store, v4i16, "BUFFER_STORE_DWORDX2">;
defm : MUBUF_StoreIntrinsicPat<SIbuffer_store, v4f16, "BUFFER_STORE_DWORDX2">;
defm : MUBUF_StoreIntrinsicPat<SIbuffer_store_byte, i32, "BUFFER_STORE_BYTE">;
defm : MUBUF_StoreIntrinsicPat<SIbuffer_store_short, i32, "BUFFER_STORE_SHORT">;



//===----------------------------------------------------------------------===//
// buffer_atomic patterns
//===----------------------------------------------------------------------===//
/*
multiclass BufferAtomicPatterns<SDPatternOperator name, ValueType vt,
                                string opcode> {
  def : PPTPat<
    (vt (name vt:$vdata_in, v4i32:$rsrc, 0,
          0, i32:$soffset, imm:$offset,
          imm:$cachepolicy, 0)),
    (!cast<MUBUF_Pseudo>(opcode # _OFFSET_RTN) $vdata_in, $rsrc, $soffset,
                                        (as_i16imm $offset), (extract_slc $cachepolicy))
  >;

  def : PPTPat<
    (vt (name vt:$vdata_in, v4i32:$rsrc, i32:$vindex,
          0, i32:$soffset, imm:$offset,
          imm:$cachepolicy, imm)),
    (!cast<MUBUF_Pseudo>(opcode # _IDXEN_RTN) $vdata_in, $vindex, $rsrc, $soffset,
                                       (as_i16imm $offset), (extract_slc $cachepolicy))
  >;

  def : PPTPat<
    (vt (name vt:$vdata_in, v4i32:$rsrc, 0,
          i32:$voffset, i32:$soffset, imm:$offset,
          imm:$cachepolicy, 0)),
    (!cast<MUBUF_Pseudo>(opcode # _OFFEN_RTN) $vdata_in, $voffset, $rsrc, $soffset,
                                       (as_i16imm $offset), (extract_slc $cachepolicy))
  >;

  def : PPTPat<
    (vt (name vt:$vdata_in, v4i32:$rsrc, i32:$vindex,
          i32:$voffset, i32:$soffset, imm:$offset,
          imm:$cachepolicy, imm)),
    (!cast<MUBUF_Pseudo>(opcode # _BOTHEN_RTN)
      $vdata_in,
      (REG_SEQUENCE VReg_64, $vindex, sub0, $voffset, sub1),
      $rsrc, $soffset, (as_i16imm $offset), (extract_slc $cachepolicy))
  >;
}
*/



/*
defm : BufferAtomicPatterns<SIbuffer_atomic_swap, i32, "BUFFER_ATOMIC_SWAP">;
defm : BufferAtomicPatterns<SIbuffer_atomic_add, i32, "BUFFER_ATOMIC_ADD">;
defm : BufferAtomicPatterns<SIbuffer_atomic_sub, i32, "BUFFER_ATOMIC_SUB">;
defm : BufferAtomicPatterns<SIbuffer_atomic_smin, i32, "BUFFER_ATOMIC_SMIN">;
defm : BufferAtomicPatterns<SIbuffer_atomic_umin, i32, "BUFFER_ATOMIC_UMIN">;
defm : BufferAtomicPatterns<SIbuffer_atomic_smax, i32, "BUFFER_ATOMIC_SMAX">;
defm : BufferAtomicPatterns<SIbuffer_atomic_umax, i32, "BUFFER_ATOMIC_UMAX">;
defm : BufferAtomicPatterns<SIbuffer_atomic_and, i32, "BUFFER_ATOMIC_AND">;
defm : BufferAtomicPatterns<SIbuffer_atomic_or, i32, "BUFFER_ATOMIC_OR">;
defm : BufferAtomicPatterns<SIbuffer_atomic_xor, i32, "BUFFER_ATOMIC_XOR">;
defm : BufferAtomicPatterns<SIbuffer_atomic_inc, i32, "BUFFER_ATOMIC_INC">;
defm : BufferAtomicPatterns<SIbuffer_atomic_dec, i32, "BUFFER_ATOMIC_DEC">;
defm : BufferAtomicPatterns<SIbuffer_atomic_swap, i64, "BUFFER_ATOMIC_SWAP_X2">;
defm : BufferAtomicPatterns<SIbuffer_atomic_add, i64,  "BUFFER_ATOMIC_ADD_X2">;
defm : BufferAtomicPatterns<SIbuffer_atomic_sub, i64, "BUFFER_ATOMIC_SUB_X2">;
defm : BufferAtomicPatterns<SIbuffer_atomic_smin, i64, "BUFFER_ATOMIC_SMIN_X2">;
defm : BufferAtomicPatterns<SIbuffer_atomic_umin, i64, "BUFFER_ATOMIC_UMIN_X2">;
defm : BufferAtomicPatterns<SIbuffer_atomic_smax, i64, "BUFFER_ATOMIC_SMAX_X2">;
defm : BufferAtomicPatterns<SIbuffer_atomic_umax, i64, "BUFFER_ATOMIC_UMAX_X2">;
defm : BufferAtomicPatterns<SIbuffer_atomic_and, i64, "BUFFER_ATOMIC_AND_X2">;
defm : BufferAtomicPatterns<SIbuffer_atomic_or, i64, "BUFFER_ATOMIC_OR_X2">;
defm : BufferAtomicPatterns<SIbuffer_atomic_xor, i64, "BUFFER_ATOMIC_XOR_X2">;
defm : BufferAtomicPatterns<SIbuffer_atomic_inc, i64, "BUFFER_ATOMIC_INC_X2">;
defm : BufferAtomicPatterns<SIbuffer_atomic_dec, i64, "BUFFER_ATOMIC_DEC_X2">;

multiclass BufferAtomicPatterns_NO_RTN<SDPatternOperator name, ValueType vt,
                                       string opcode> {
  def : PPTPat<
    (name vt:$vdata_in, v4i32:$rsrc, 0,
          0, i32:$soffset, imm:$offset,
          imm:$cachepolicy, 0),
    (!cast<MUBUF_Pseudo>(opcode # _OFFSET) $vdata_in, $rsrc, $soffset,
                                        (as_i16imm $offset), (extract_slc $cachepolicy))
  >;

  def : PPTPat<
    (name vt:$vdata_in, v4i32:$rsrc, i32:$vindex,
          0, i32:$soffset, imm:$offset,
          imm:$cachepolicy, imm),
    (!cast<MUBUF_Pseudo>(opcode # _IDXEN) $vdata_in, $vindex, $rsrc, $soffset,
                                       (as_i16imm $offset), (extract_slc $cachepolicy))
  >;

  def : PPTPat<
    (name vt:$vdata_in, v4i32:$rsrc, 0,
          i32:$voffset, i32:$soffset, imm:$offset,
          imm:$cachepolicy, 0),
    (!cast<MUBUF_Pseudo>(opcode # _OFFEN) $vdata_in, $voffset, $rsrc, $soffset,
                                       (as_i16imm $offset), (extract_slc $cachepolicy))
  >;

  def : PPTPat<
    (name vt:$vdata_in, v4i32:$rsrc, i32:$vindex,
          i32:$voffset, i32:$soffset, imm:$offset,
          imm:$cachepolicy, imm),
    (!cast<MUBUF_Pseudo>(opcode # _BOTHEN)
      $vdata_in,
      (REG_SEQUENCE VReg_64, $vindex, sub0, $voffset, sub1),
      $rsrc, $soffset, (as_i16imm $offset), (extract_slc $cachepolicy))
  >;
}

defm : BufferAtomicPatterns_NO_RTN<SIbuffer_atomic_fadd, f32, "BUFFER_ATOMIC_ADD_F32">;
defm : BufferAtomicPatterns_NO_RTN<SIbuffer_atomic_pk_fadd, v2f16, "BUFFER_ATOMIC_PK_ADD_F16">;

def : PPTPat<
  (SIbuffer_atomic_cmpswap
      i32:$data, i32:$cmp, v4i32:$rsrc, 0,
      0, i32:$soffset, imm:$offset,
      imm:$cachepolicy, 0),
  (EXTRACT_SUBREG
    (BUFFER_ATOMIC_CMPSWAP_OFFSET_RTN
      (REG_SEQUENCE VReg_64, $data, sub0, $cmp, sub1),
      $rsrc, $soffset, (as_i16imm $offset), (extract_slc $cachepolicy)),
    sub0)
>;

def : PPTPat<
  (SIbuffer_atomic_cmpswap
      i32:$data, i32:$cmp, v4i32:$rsrc, i32:$vindex,
      0, i32:$soffset, imm:$offset,
      imm:$cachepolicy, imm),
  (EXTRACT_SUBREG
    (BUFFER_ATOMIC_CMPSWAP_IDXEN_RTN
      (REG_SEQUENCE VReg_64, $data, sub0, $cmp, sub1),
      $vindex, $rsrc, $soffset, (as_i16imm $offset), (extract_slc $cachepolicy)),
    sub0)
>;

def : PPTPat<
  (SIbuffer_atomic_cmpswap
      i32:$data, i32:$cmp, v4i32:$rsrc, 0,
      i32:$voffset, i32:$soffset, imm:$offset,
      imm:$cachepolicy, 0),
  (EXTRACT_SUBREG
    (BUFFER_ATOMIC_CMPSWAP_OFFEN_RTN
      (REG_SEQUENCE VReg_64, $data, sub0, $cmp, sub1),
      $voffset, $rsrc, $soffset, (as_i16imm $offset), (extract_slc $cachepolicy)),
    sub0)
>;

def : PPTPat<
  (SIbuffer_atomic_cmpswap
      i32:$data, i32:$cmp, v4i32:$rsrc, i32:$vindex,
      i32:$voffset, i32:$soffset, imm:$offset,
      imm:$cachepolicy, imm),
  (EXTRACT_SUBREG
    (BUFFER_ATOMIC_CMPSWAP_BOTHEN_RTN
      (REG_SEQUENCE VReg_64, $data, sub0, $cmp, sub1),
      (REG_SEQUENCE VReg_64, $vindex, sub0, $voffset, sub1),
      $rsrc, $soffset, (as_i16imm $offset), (extract_slc $cachepolicy)),
    sub0)
>;

class MUBUFLoad_PatternADDR64 <MUBUF_Pseudo Instr_ADDR64, ValueType vt,
                              PatFrag constant_ld> : PPTPat <
     (vt (constant_ld (MUBUFAddr64 v4i32:$srsrc, i64:$vaddr, i32:$soffset,
                                   i16:$offset, i1:$glc, i1:$slc, i1:$tfe, i1:$dlc))),
     (Instr_ADDR64 $vaddr, $srsrc, $soffset, $offset, $glc, $slc, $tfe, $dlc)
  >;

multiclass MUBUFLoad_Atomic_Pattern <MUBUF_Pseudo Instr_ADDR64, MUBUF_Pseudo Instr_OFFSET,
                                     ValueType vt, PatFrag atomic_ld> {
  def : PPTPat <
     (vt (atomic_ld (MUBUFAddr64 v4i32:$srsrc, i64:$vaddr, i32:$soffset,
                                   i16:$offset, i1:$slc))),
     (Instr_ADDR64 $vaddr, $srsrc, $soffset, $offset, 0, $slc, 0, 0)
  >;

  def : PPTPat <
    (vt (atomic_ld (MUBUFOffsetNoGLC v4i32:$rsrc, i32:$soffset, i16:$offset))),
    (Instr_OFFSET $rsrc, $soffset, (as_i16imm $offset), 0, 0, 0, 0)
  >;
}
*/

/*
let SubtargetPredicate = isGFX6GFX7 in {
def : MUBUFLoad_PatternADDR64 <BUFFER_LOAD_SBYTE_ADDR64, i32, sextloadi8_constant>;
def : MUBUFLoad_PatternADDR64 <BUFFER_LOAD_UBYTE_ADDR64, i32, extloadi8_constant>;
def : MUBUFLoad_PatternADDR64 <BUFFER_LOAD_UBYTE_ADDR64, i32, zextloadi8_constant>;
def : MUBUFLoad_PatternADDR64 <BUFFER_LOAD_SSHORT_ADDR64, i32, sextloadi16_constant>;
def : MUBUFLoad_PatternADDR64 <BUFFER_LOAD_USHORT_ADDR64, i32, extloadi16_constant>;
def : MUBUFLoad_PatternADDR64 <BUFFER_LOAD_USHORT_ADDR64, i32, zextloadi16_constant>;

defm : MUBUFLoad_Atomic_Pattern <BUFFER_LOAD_DWORD_ADDR64, BUFFER_LOAD_DWORD_OFFSET, i32, atomic_load_32_global>;
defm : MUBUFLoad_Atomic_Pattern <BUFFER_LOAD_DWORDX2_ADDR64, BUFFER_LOAD_DWORDX2_OFFSET, i64, atomic_load_64_global>;
} // End SubtargetPredicate = isGFX6GFX7
*/

multiclass MUBUFLoad_Pattern <MUBUF_Pseudo Instr_OFFSET, ValueType vt,
                               PatFrag ld> {

  def : PPTPat <
    (vt (ld (MUBUFOffset v2i32:$srsrc, i32:$soffset,
                          i16:$offset, i1:$glc, i1:$slc, i1:$tfe, i1:$dlc))),
    (Instr_OFFSET $srsrc, $soffset, $offset, $glc, $slc, $tfe, $dlc)
  >;
}

let OtherPredicates = [Has16BitInsts] in {

defm : MUBUFLoad_Pattern <BUFFER_LOAD_SBYTE_OFFSET, i16, sextloadi8_constant>;
defm : MUBUFLoad_Pattern <BUFFER_LOAD_UBYTE_OFFSET, i16, extloadi8_constant>;
defm : MUBUFLoad_Pattern <BUFFER_LOAD_UBYTE_OFFSET, i16, zextloadi8_constant>;
defm : MUBUFLoad_Pattern <BUFFER_LOAD_SBYTE_OFFSET, i16, sextloadi8_global>;
defm : MUBUFLoad_Pattern <BUFFER_LOAD_UBYTE_OFFSET, i16, extloadi8_global>;
defm : MUBUFLoad_Pattern <BUFFER_LOAD_UBYTE_OFFSET, i16, zextloadi8_global>;

defm : MUBUFLoad_Pattern <BUFFER_LOAD_USHORT_OFFSET, i16, load_global>;

} // End OtherPredicates = [Has16BitInsts]

multiclass MUBUFScratchLoadPat <MUBUF_Pseudo InstrOffen,
                                MUBUF_Pseudo InstrOffset,
                                ValueType vt, PatFrag ld> {
  def : PPTPat <
    (vt (ld (MUBUFScratchOffen v2i32:$srsrc, i32:$vaddr,
                               i32:$soffset, u16imm:$offset))),
    (InstrOffen $vaddr, $srsrc, $soffset, $offset, 0, 0, 0, 0)
  >;

  def : PPTPat <
    (vt (ld (MUBUFScratchOffset v2i32:$srsrc, i32:$soffset, u16imm:$offset))),
    (InstrOffset $srsrc, $soffset, $offset, 0, 0, 0, 0)
  >;
}

// XXX - Is it possible to have a complex pattern in a PatFrag?
multiclass MUBUFScratchLoadPat_D16 <MUBUF_Pseudo InstrOffen,
                                MUBUF_Pseudo InstrOffset,
                                ValueType vt, PatFrag ld_frag> {
  def : PPTPat <
    (ld_frag (MUBUFScratchOffen v2i32:$srsrc, i32:$vaddr, i32:$soffset, u16imm:$offset), vt:$in),
    (InstrOffen $vaddr, $srsrc, $soffset, $offset, 0, 0, 0, 0, $in)
  >;

  def : PPTPat <
    (ld_frag (MUBUFScratchOffset v2i32:$srsrc, i32:$soffset, u16imm:$offset), vt:$in),
    (InstrOffset $srsrc, $soffset, $offset, 0, 0, 0, 0, $in)
  >;
}

defm : MUBUFScratchLoadPat <BUFFER_LOAD_SBYTE_OFFEN, BUFFER_LOAD_SBYTE_OFFSET, i32, sextloadi8_private>;
defm : MUBUFScratchLoadPat <BUFFER_LOAD_UBYTE_OFFEN, BUFFER_LOAD_UBYTE_OFFSET, i32, extloadi8_private>;
defm : MUBUFScratchLoadPat <BUFFER_LOAD_UBYTE_OFFEN, BUFFER_LOAD_UBYTE_OFFSET, i32, zextloadi8_private>;
defm : MUBUFScratchLoadPat <BUFFER_LOAD_SBYTE_OFFEN, BUFFER_LOAD_SBYTE_OFFSET, i16, sextloadi8_private>;
defm : MUBUFScratchLoadPat <BUFFER_LOAD_UBYTE_OFFEN, BUFFER_LOAD_UBYTE_OFFSET, i16, extloadi8_private>;
defm : MUBUFScratchLoadPat <BUFFER_LOAD_UBYTE_OFFEN, BUFFER_LOAD_UBYTE_OFFSET, i16, zextloadi8_private>;
defm : MUBUFScratchLoadPat <BUFFER_LOAD_SSHORT_OFFEN, BUFFER_LOAD_SSHORT_OFFSET, i32, sextloadi16_private>;
defm : MUBUFScratchLoadPat <BUFFER_LOAD_USHORT_OFFEN, BUFFER_LOAD_USHORT_OFFSET, i32, extloadi16_private>;
defm : MUBUFScratchLoadPat <BUFFER_LOAD_USHORT_OFFEN, BUFFER_LOAD_USHORT_OFFSET, i32, zextloadi16_private>;
defm : MUBUFScratchLoadPat <BUFFER_LOAD_USHORT_OFFEN, BUFFER_LOAD_USHORT_OFFSET, i16, load_private>;
defm : MUBUFScratchLoadPat <BUFFER_LOAD_DWORD_OFFEN, BUFFER_LOAD_DWORD_OFFSET, i32, load_private>;
defm : MUBUFScratchLoadPat <BUFFER_LOAD_DWORDX2_OFFEN, BUFFER_LOAD_DWORDX2_OFFSET, v2i32, load_private>;
// defm : MUBUFScratchLoadPat <BUFFER_LOAD_DWORDX3_OFFEN, BUFFER_LOAD_DWORDX3_OFFSET, v3i32, load_private>;
// defm : MUBUFScratchLoadPat <BUFFER_LOAD_DWORDX4_OFFEN, BUFFER_LOAD_DWORDX4_OFFSET, v4i32, load_private>;

let OtherPredicates = [D16PreservesUnusedBits] in {
defm : MUBUFScratchLoadPat_D16<BUFFER_LOAD_SHORT_D16_HI_OFFEN, BUFFER_LOAD_SHORT_D16_HI_OFFSET, v2i16, load_d16_hi_private>;
defm : MUBUFScratchLoadPat_D16<BUFFER_LOAD_UBYTE_D16_HI_OFFEN, BUFFER_LOAD_UBYTE_D16_HI_OFFSET, v2i16, az_extloadi8_d16_hi_private>;
defm : MUBUFScratchLoadPat_D16<BUFFER_LOAD_SBYTE_D16_HI_OFFEN, BUFFER_LOAD_SBYTE_D16_HI_OFFSET, v2i16, sextloadi8_d16_hi_private>;
defm : MUBUFScratchLoadPat_D16<BUFFER_LOAD_SHORT_D16_HI_OFFEN, BUFFER_LOAD_SHORT_D16_HI_OFFSET, v2f16, load_d16_hi_private>;
defm : MUBUFScratchLoadPat_D16<BUFFER_LOAD_UBYTE_D16_HI_OFFEN, BUFFER_LOAD_UBYTE_D16_HI_OFFSET, v2f16, az_extloadi8_d16_hi_private>;
defm : MUBUFScratchLoadPat_D16<BUFFER_LOAD_SBYTE_D16_HI_OFFEN, BUFFER_LOAD_SBYTE_D16_HI_OFFSET, v2f16, sextloadi8_d16_hi_private>;

defm : MUBUFScratchLoadPat_D16<BUFFER_LOAD_SHORT_D16_OFFEN, BUFFER_LOAD_SHORT_D16_OFFSET, v2i16, load_d16_lo_private>;
defm : MUBUFScratchLoadPat_D16<BUFFER_LOAD_UBYTE_D16_OFFEN, BUFFER_LOAD_UBYTE_D16_OFFSET, v2i16, az_extloadi8_d16_lo_private>;
defm : MUBUFScratchLoadPat_D16<BUFFER_LOAD_SBYTE_D16_OFFEN, BUFFER_LOAD_SBYTE_D16_OFFSET, v2i16, sextloadi8_d16_lo_private>;
defm : MUBUFScratchLoadPat_D16<BUFFER_LOAD_SHORT_D16_OFFEN, BUFFER_LOAD_SHORT_D16_OFFSET, v2f16, load_d16_lo_private>;
defm : MUBUFScratchLoadPat_D16<BUFFER_LOAD_UBYTE_D16_OFFEN, BUFFER_LOAD_UBYTE_D16_OFFSET, v2f16, az_extloadi8_d16_lo_private>;
defm : MUBUFScratchLoadPat_D16<BUFFER_LOAD_SBYTE_D16_OFFEN, BUFFER_LOAD_SBYTE_D16_OFFSET, v2f16, sextloadi8_d16_lo_private>;
}

multiclass MUBUFStore_Atomic_Pattern <MUBUF_Pseudo Instr_ADDR64, MUBUF_Pseudo Instr_OFFSET,
                                      ValueType vt, PatFrag atomic_st> {
  // Store follows atomic op convention so address is forst
  def : PPTPat <
     (atomic_st (MUBUFAddr64 v2i32:$srsrc, i64:$vaddr, i32:$soffset,
                                   i16:$offset, i1:$slc), vt:$val),
     (Instr_ADDR64 $val, $vaddr, $srsrc, $soffset, $offset, 0, $slc, 0, 0)
  >;

  def : PPTPat <
    (atomic_st (MUBUFOffsetNoGLC v4i32:$rsrc, i32:$soffset, i16:$offset), vt:$val),
    (Instr_OFFSET $val, $rsrc, $soffset, (as_i16imm $offset), 0, 0, 0, 0)
  >;
}
/*
let SubtargetPredicate = isGFX6GFX7 in {
defm : MUBUFStore_Atomic_Pattern <BUFFER_STORE_DWORD_ADDR64, BUFFER_STORE_DWORD_OFFSET, i32, store_atomic_global>;
defm : MUBUFStore_Atomic_Pattern <BUFFER_STORE_DWORDX2_ADDR64, BUFFER_STORE_DWORDX2_OFFSET, i64, store_atomic_global>;
} // End Predicates = isGFX6GFX7
*/


multiclass MUBUFStore_Pattern <MUBUF_Pseudo Instr_OFFSET, ValueType vt,
                               PatFrag st> {

  def : PPTPat <
    (st vt:$vdata, (MUBUFOffset v2i32:$srsrc, i32:$soffset,
                                      i16:$offset, i1:$glc, i1:$slc, i1:$tfe, i1:$dlc)),
    (Instr_OFFSET $vdata, $srsrc, $soffset, $offset, $glc, $slc, $tfe, $dlc)
  >;
}


defm : MUBUFStore_Pattern <BUFFER_STORE_BYTE_OFFSET, i16, truncstorei8_global>;
defm : MUBUFStore_Pattern <BUFFER_STORE_SHORT_OFFSET, i16, store_global>;
multiclass MUBUFScratchStorePat <MUBUF_Pseudo InstrOffen,
                                 MUBUF_Pseudo InstrOffset,
                                 ValueType vt, PatFrag st,
                                 RegisterClass rc = VPR_32> {
  def : PPTPat <
    (st vt:$value, (MUBUFScratchOffen v2i32:$srsrc, i32:$vaddr,
                                      i32:$soffset, u16imm:$offset)),
    (InstrOffen rc:$value, $vaddr, $srsrc, $soffset, $offset, 0, 0, 0, 0)
  >;

  def : PPTPat <
    (st vt:$value, (MUBUFScratchOffset v2i32:$srsrc, i32:$soffset,
                                       u16imm:$offset)),
    (InstrOffset rc:$value, $srsrc, $soffset, $offset, 0, 0, 0, 0)
  >;
}

defm : MUBUFScratchStorePat <BUFFER_STORE_BYTE_OFFEN, BUFFER_STORE_BYTE_OFFSET, i32, truncstorei8_private>;
defm : MUBUFScratchStorePat <BUFFER_STORE_SHORT_OFFEN, BUFFER_STORE_SHORT_OFFSET, i32, truncstorei16_private>;
defm : MUBUFScratchStorePat <BUFFER_STORE_BYTE_OFFEN, BUFFER_STORE_BYTE_OFFSET, i16, truncstorei8_private>;
defm : MUBUFScratchStorePat <BUFFER_STORE_SHORT_OFFEN, BUFFER_STORE_SHORT_OFFSET, i16, store_private>;
defm : MUBUFScratchStorePat <BUFFER_STORE_DWORD_OFFEN, BUFFER_STORE_DWORD_OFFSET, i32, store_private>;
defm : MUBUFScratchStorePat <BUFFER_STORE_DWORDX2_OFFEN, BUFFER_STORE_DWORDX2_OFFSET, v2i32, store_private, VReg_64>;
// defm : MUBUFScratchStorePat <BUFFER_STORE_DWORDX3_OFFEN, BUFFER_STORE_DWORDX3_OFFSET, v3i32, store_private, VReg_96>;
// defm : MUBUFScratchStorePat <BUFFER_STORE_DWORDX4_OFFEN, BUFFER_STORE_DWORDX4_OFFSET, v4i32, store_private, VReg_128>;


let OtherPredicates = [D16PreservesUnusedBits] in {
 // Hiding the extract high pattern in the PatFrag seems to not
 // automatically increase the complexity.
let AddedComplexity = 1 in {
defm : MUBUFScratchStorePat <BUFFER_STORE_SHORT_D16_HI_OFFEN, BUFFER_STORE_SHORT_D16_HI_OFFSET, i32, store_hi16_private>;
defm : MUBUFScratchStorePat <BUFFER_STORE_BYTE_D16_HI_OFFEN, BUFFER_STORE_BYTE_D16_HI_OFFSET, i32, truncstorei8_hi16_private>;
}
}


//===----------------------------------------------------------------------===//
// MTBUF Patterns
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
// tbuffer_load/store_format patterns
//===----------------------------------------------------------------------===//

multiclass MTBUF_LoadIntrinsicPat<SDPatternOperator name, ValueType vt,
                                  string opcode> {
  def : PPTPat<
    (vt (name v2i32:$rsrc, 0, 0, i32:$soffset, imm:$offset,
              imm:$format, imm:$cachepolicy, 0)),
    (!cast<MTBUF_Pseudo>(opcode # _OFFSET) $rsrc, $soffset, (as_i16imm $offset),
      (as_i8imm $format),
      (extract_glc $cachepolicy), (extract_slc $cachepolicy), 0, (extract_dlc $cachepolicy))
  >;

  def : PPTPat<
    (vt (name v2i32:$rsrc, i32:$vindex, 0, i32:$soffset, imm:$offset,
              imm:$format, imm:$cachepolicy, imm)),
    (!cast<MTBUF_Pseudo>(opcode # _IDXEN) $vindex, $rsrc, $soffset, (as_i16imm $offset),
      (as_i8imm $format),
      (extract_glc $cachepolicy), (extract_slc $cachepolicy), 0, (extract_dlc $cachepolicy))
  >;

  def : PPTPat<
    (vt (name v2i32:$rsrc, 0, i32:$voffset, i32:$soffset, imm:$offset,
              imm:$format, imm:$cachepolicy, 0)),
    (!cast<MTBUF_Pseudo>(opcode # _OFFEN) $voffset, $rsrc, $soffset, (as_i16imm $offset),
      (as_i8imm $format),
      (extract_glc $cachepolicy), (extract_slc $cachepolicy), 0, (extract_dlc $cachepolicy))
  >;

  def : PPTPat<
    (vt (name v2i32:$rsrc, i32:$vindex, i32:$voffset, i32:$soffset, imm:$offset,
              imm:$format, imm:$cachepolicy, imm)),
    (!cast<MTBUF_Pseudo>(opcode # _BOTHEN)
      (REG_SEQUENCE VReg_64, $vindex, sub0, $voffset, sub1),
      $rsrc, $soffset, (as_i16imm $offset),
      (as_i8imm $format),
      (extract_glc $cachepolicy), (extract_slc $cachepolicy), 0, (extract_dlc $cachepolicy))
  >;
}

defm : MTBUF_LoadIntrinsicPat<SItbuffer_load, i32,   "TBUFFER_LOAD_FORMAT_X">;
defm : MTBUF_LoadIntrinsicPat<SItbuffer_load, v2i32, "TBUFFER_LOAD_FORMAT_XY">;
// defm : MTBUF_LoadIntrinsicPat<SItbuffer_load, v3i32, "TBUFFER_LOAD_FORMAT_XYZ">;
// defm : MTBUF_LoadIntrinsicPat<SItbuffer_load, v4i32, "TBUFFER_LOAD_FORMAT_XYZW">;
defm : MTBUF_LoadIntrinsicPat<SItbuffer_load, f32,   "TBUFFER_LOAD_FORMAT_X">;
defm : MTBUF_LoadIntrinsicPat<SItbuffer_load, v2f32, "TBUFFER_LOAD_FORMAT_XY">;
// defm : MTBUF_LoadIntrinsicPat<SItbuffer_load, v3f32, "TBUFFER_LOAD_FORMAT_XYZ">;
// defm : MTBUF_LoadIntrinsicPat<SItbuffer_load, v4f32, "TBUFFER_LOAD_FORMAT_XYZW">;

let SubtargetPredicate = HasUnpackedD16VMem in {
  defm : MTBUF_LoadIntrinsicPat<SItbuffer_load_d16, f16,   "TBUFFER_LOAD_FORMAT_D16_X_gfx80">;
  defm : MTBUF_LoadIntrinsicPat<SItbuffer_load_d16, v2i32, "TBUFFER_LOAD_FORMAT_D16_XY_gfx80">;
  // defm : MTBUF_LoadIntrinsicPat<SItbuffer_load_d16, v4i32, "TBUFFER_LOAD_FORMAT_D16_XYZW_gfx80">;
} // End HasUnpackedD16VMem.

let SubtargetPredicate = HasPackedD16VMem in {
  defm : MTBUF_LoadIntrinsicPat<SItbuffer_load_d16, f16,   "TBUFFER_LOAD_FORMAT_D16_X">;
  defm : MTBUF_LoadIntrinsicPat<SItbuffer_load_d16, v2f16, "TBUFFER_LOAD_FORMAT_D16_XY">;
  // defm : MTBUF_LoadIntrinsicPat<SItbuffer_load_d16, v4f16, "TBUFFER_LOAD_FORMAT_D16_XYZW">;
} // End HasPackedD16VMem.

multiclass MTBUF_StoreIntrinsicPat<SDPatternOperator name, ValueType vt,
                                   string opcode> {
  def : PPTPat<
    (name vt:$vdata, v2i32:$rsrc, 0, 0, i32:$soffset, imm:$offset,
          imm:$format, imm:$cachepolicy, 0),
    (!cast<MTBUF_Pseudo>(opcode # _OFFSET_exact) $vdata, $rsrc, $soffset,
      (as_i16imm $offset), (as_i8imm $format),
      (extract_glc $cachepolicy), (extract_slc $cachepolicy), 0, (extract_dlc $cachepolicy))
  >;

  def : PPTPat<
    (name vt:$vdata, v2i32:$rsrc, i32:$vindex, 0, i32:$soffset, imm:$offset,
          imm:$format, imm:$cachepolicy, imm),
    (!cast<MTBUF_Pseudo>(opcode # _IDXEN_exact) $vdata, $vindex, $rsrc, $soffset,
      (as_i16imm $offset), (as_i8imm $format),
      (extract_glc $cachepolicy), (extract_slc $cachepolicy), 0, (extract_dlc $cachepolicy))
  >;

  def : PPTPat<
    (name vt:$vdata, v2i32:$rsrc, 0, i32:$voffset, i32:$soffset, imm:$offset,
          imm:$format, imm:$cachepolicy, 0),
    (!cast<MTBUF_Pseudo>(opcode # _OFFEN_exact) $vdata, $voffset, $rsrc, $soffset,
      (as_i16imm $offset), (as_i8imm $format),
      (extract_glc $cachepolicy), (extract_slc $cachepolicy), 0, (extract_dlc $cachepolicy))
  >;

  def : PPTPat<
    (name vt:$vdata, v2i32:$rsrc, i32:$vindex, i32:$voffset, i32:$soffset,
          imm:$offset, imm:$format, imm:$cachepolicy, imm),
    (!cast<MTBUF_Pseudo>(opcode # _BOTHEN_exact)
      $vdata,
      (REG_SEQUENCE VReg_64, $vindex, sub0, $voffset, sub1),
      $rsrc, $soffset, (as_i16imm $offset), (as_i8imm $format),
      (extract_glc $cachepolicy), (extract_slc $cachepolicy), 0, (extract_dlc $cachepolicy))
  >;
}

defm : MTBUF_StoreIntrinsicPat<SItbuffer_store, i32,   "TBUFFER_STORE_FORMAT_X">;
defm : MTBUF_StoreIntrinsicPat<SItbuffer_store, v2i32, "TBUFFER_STORE_FORMAT_XY">;
// defm : MTBUF_StoreIntrinsicPat<SItbuffer_store, v3i32, "TBUFFER_STORE_FORMAT_XYZ">;
// defm : MTBUF_StoreIntrinsicPat<SItbuffer_store, v4i32, "TBUFFER_STORE_FORMAT_XYZW">;
defm : MTBUF_StoreIntrinsicPat<SItbuffer_store, f32,   "TBUFFER_STORE_FORMAT_X">;
defm : MTBUF_StoreIntrinsicPat<SItbuffer_store, v2f32, "TBUFFER_STORE_FORMAT_XY">;
// defm : MTBUF_StoreIntrinsicPat<SItbuffer_store, v3f32, "TBUFFER_STORE_FORMAT_XYZ">;
// defm : MTBUF_StoreIntrinsicPat<SItbuffer_store, v4f32, "TBUFFER_STORE_FORMAT_XYZW">;

let SubtargetPredicate = HasUnpackedD16VMem in {
  defm : MTBUF_StoreIntrinsicPat<SItbuffer_store_d16, f16,   "TBUFFER_STORE_FORMAT_D16_X_gfx80">;
  defm : MTBUF_StoreIntrinsicPat<SItbuffer_store_d16, v2i32, "TBUFFER_STORE_FORMAT_D16_XY_gfx80">;
//   defm : MTBUF_StoreIntrinsicPat<SItbuffer_store_d16, v4i32, "TBUFFER_STORE_FORMAT_D16_XYZW_gfx80">;
} // End HasUnpackedD16VMem.

let SubtargetPredicate = HasPackedD16VMem in {
  defm : MTBUF_StoreIntrinsicPat<SItbuffer_store_d16, f16,   "TBUFFER_STORE_FORMAT_D16_X">;
  defm : MTBUF_StoreIntrinsicPat<SItbuffer_store_d16, v2f16, "TBUFFER_STORE_FORMAT_D16_XY">;
  // defm : MTBUF_StoreIntrinsicPat<SItbuffer_store_d16, v4f16, "TBUFFER_STORE_FORMAT_D16_XYZW">;
} // End HasPackedD16VMem.




//===----------------------------------------------------------------------===//
// Target-specific instruction encodings.
//===----------------------------------------------------------------------===//
//===----------------------------------------------------------------------===//
// GFX8, GFX9 (VI).
//===----------------------------------------------------------------------===//

class MUBUF_Real_ppu <bits<7> op, MUBUF_Pseudo ps> :
  MUBUF_Real<ps>,
  Enc64,
  PPUMCInstr<ps.PseudoInstr, PPUEncodingFamily.PPU> {
  let AssemblerPredicate = IsPPT;
  let DecoderNamespace = "PPU";

  // FIXME why we need below but AMD don't have
  let hasSideEffects = 1;
  let mayLoad = 1;
  let mayStore = 1;


  let Inst{11-0}  = !if(ps.has_offset, offset, ?);
  let Inst{12}    = ps.offen;
  let Inst{13}    = ps.idxen;
  let Inst{14}    = !if(ps.has_glc, glc, ps.glc_value);
  let Inst{16}    = !if(ps.lds, 1, 0);
  let Inst{17}    = !if(ps.has_slc, slc, ?);
  let Inst{24-18} = op;
  let Inst{31-26} = 0x38; //encoding
  let Inst{39-32} = !if(ps.has_vaddr, vaddr, ?);
  let Inst{47-40} = !if(ps.has_vdata, vdata, ?);
  let Inst{52-48} = !if(ps.has_srsrc, srsrc{6-2}, ?);
  let Inst{55}    = !if(ps.has_tfe, tfe, ?);
  let Inst{63-56} = !if(ps.has_soffset, soffset, ?);
}

multiclass MUBUF_Real_AllAddr_ppu<bits<7> op> {
  def _OFFSET_ppu : MUBUF_Real_ppu <op, !cast<MUBUF_Pseudo>(NAME#"_OFFSET")>;
  def _OFFEN_ppu  : MUBUF_Real_ppu <op, !cast<MUBUF_Pseudo>(NAME#"_OFFEN")>;
  def _IDXEN_ppu  : MUBUF_Real_ppu <op, !cast<MUBUF_Pseudo>(NAME#"_IDXEN")>;
  def _BOTHEN_ppu : MUBUF_Real_ppu <op, !cast<MUBUF_Pseudo>(NAME#"_BOTHEN")>;
}

multiclass MUBUF_Real_AllAddr_Lds_ppu<bits<7> op> {

  def _OFFSET_ppu : MUBUF_Real_ppu <op, !cast<MUBUF_Pseudo>(NAME#"_OFFSET")>,
                   MUBUFLdsTable<0, NAME # "_OFFSET_ppu">;
  def _OFFEN_ppu  : MUBUF_Real_ppu <op, !cast<MUBUF_Pseudo>(NAME#"_OFFEN")>,
                   MUBUFLdsTable<0, NAME # "_OFFEN_ppu">;
  def _IDXEN_ppu  : MUBUF_Real_ppu <op, !cast<MUBUF_Pseudo>(NAME#"_IDXEN")>,
                   MUBUFLdsTable<0, NAME # "_IDXEN_ppu">;
  def _BOTHEN_ppu : MUBUF_Real_ppu <op, !cast<MUBUF_Pseudo>(NAME#"_BOTHEN")>,
                   MUBUFLdsTable<0, NAME # "_BOTHEN_ppu">;

  def _LDS_OFFSET_ppu : MUBUF_Real_ppu <op, !cast<MUBUF_Pseudo>(NAME#"_LDS_OFFSET")>,
                       MUBUFLdsTable<1, NAME # "_OFFSET_ppu">;
  def _LDS_OFFEN_ppu  : MUBUF_Real_ppu <op, !cast<MUBUF_Pseudo>(NAME#"_LDS_OFFEN")>,
                       MUBUFLdsTable<1, NAME # "_OFFEN_ppu">;
  def _LDS_IDXEN_ppu  : MUBUF_Real_ppu <op, !cast<MUBUF_Pseudo>(NAME#"_LDS_IDXEN")>,
                       MUBUFLdsTable<1, NAME # "_IDXEN_ppu">;
  def _LDS_BOTHEN_ppu : MUBUF_Real_ppu <op, !cast<MUBUF_Pseudo>(NAME#"_LDS_BOTHEN")>,
                       MUBUFLdsTable<1, NAME # "_BOTHEN_ppu">;
}

multiclass MUBUF_Real_Atomic_ppu<bits<7> op> :
  MUBUF_Real_AllAddr_ppu<op> {
  def _OFFSET_RTN_ppu : MUBUF_Real_ppu <op, !cast<MUBUF_Pseudo>(NAME#"_OFFSET_RTN")>;
  def _OFFEN_RTN_ppu  : MUBUF_Real_ppu <op, !cast<MUBUF_Pseudo>(NAME#"_OFFEN_RTN")>;
  def _IDXEN_RTN_ppu  : MUBUF_Real_ppu <op, !cast<MUBUF_Pseudo>(NAME#"_IDXEN_RTN")>;
  def _BOTHEN_RTN_ppu : MUBUF_Real_ppu <op, !cast<MUBUF_Pseudo>(NAME#"_BOTHEN_RTN")>;
}

defm BUFFER_LOAD_FORMAT_X       : MUBUF_Real_AllAddr_Lds_ppu <0x00>;
defm BUFFER_LOAD_FORMAT_XY      : MUBUF_Real_AllAddr_ppu <0x01>;
// defm BUFFER_LOAD_FORMAT_XYZ     : MUBUF_Real_AllAddr_ppu <0x02>;
// defm BUFFER_LOAD_FORMAT_XYZW    : MUBUF_Real_AllAddr_ppu <0x03>;
defm BUFFER_STORE_FORMAT_X      : MUBUF_Real_AllAddr_ppu <0x04>;
defm BUFFER_STORE_FORMAT_XY     : MUBUF_Real_AllAddr_ppu <0x05>;
// defm BUFFER_STORE_FORMAT_XYZ    : MUBUF_Real_AllAddr_ppu <0x06>;
// defm BUFFER_STORE_FORMAT_XYZW   : MUBUF_Real_AllAddr_ppu <0x07>;
/*
let SubtargetPredicate = HasUnpackedD16VMem in {
  defm BUFFER_LOAD_FORMAT_D16_X_gfx80       : MUBUF_Real_AllAddr_ppu <0x08>;
  defm BUFFER_LOAD_FORMAT_D16_XY_gfx80      : MUBUF_Real_AllAddr_ppu <0x09>;
//   defm BUFFER_LOAD_FORMAT_D16_XYZ_gfx80     : MUBUF_Real_AllAddr_gfx80 <0x0a>;
//  defm BUFFER_LOAD_FORMAT_D16_XYZW_gfx80    : MUBUF_Real_AllAddr_gfx80 <0x0b>;
  defm BUFFER_STORE_FORMAT_D16_X_gfx80      : MUBUF_Real_AllAddr_ppu <0x0c>;
  defm BUFFER_STORE_FORMAT_D16_XY_gfx80     : MUBUF_Real_AllAddr_ppu <0x0d>;
//   defm BUFFER_STORE_FORMAT_D16_XYZ_gfx80    : MUBUF_Real_AllAddr_gfx80 <0x0e>;
//  defm BUFFER_STORE_FORMAT_D16_XYZW_gfx80   : MUBUF_Real_AllAddr_gfx80 <0x0f>;
} // End HasUnpackedD16VMem.
*/

let SubtargetPredicate = HasPackedD16VMem in {
  defm BUFFER_LOAD_FORMAT_D16_X       : MUBUF_Real_AllAddr_ppu <0x08>;
  defm BUFFER_LOAD_FORMAT_D16_XY      : MUBUF_Real_AllAddr_ppu <0x09>;
//   defm BUFFER_LOAD_FORMAT_D16_XYZ     : MUBUF_Real_AllAddr_ppu <0x0a>;
//  defm BUFFER_LOAD_FORMAT_D16_XYZW    : MUBUF_Real_AllAddr_ppu <0x0b>;
  defm BUFFER_STORE_FORMAT_D16_X      : MUBUF_Real_AllAddr_ppu <0x0c>;
  defm BUFFER_STORE_FORMAT_D16_XY     : MUBUF_Real_AllAddr_ppu <0x0d>;
//  defm BUFFER_STORE_FORMAT_D16_XYZ    : MUBUF_Real_AllAddr_ppu <0x0e>;
//   defm BUFFER_STORE_FORMAT_D16_XYZW   : MUBUF_Real_AllAddr_ppu <0x0f>;
} // End HasPackedD16VMem.
defm BUFFER_LOAD_UBYTE          : MUBUF_Real_AllAddr_Lds_ppu <0x10>;
defm BUFFER_LOAD_SBYTE          : MUBUF_Real_AllAddr_Lds_ppu <0x11>;
defm BUFFER_LOAD_USHORT         : MUBUF_Real_AllAddr_Lds_ppu <0x12>;
defm BUFFER_LOAD_SSHORT         : MUBUF_Real_AllAddr_Lds_ppu <0x13>;
defm BUFFER_LOAD_DWORD          : MUBUF_Real_AllAddr_Lds_ppu <0x14>;
defm BUFFER_LOAD_DWORDX2        : MUBUF_Real_AllAddr_Lds_ppu <0x15>;
// defm BUFFER_LOAD_DWORDX3        : MUBUF_Real_AllAddr_Lds_ppu <0x16>;
// defm BUFFER_LOAD_DWORDX4        : MUBUF_Real_AllAddr_Lds_ppu <0x17>;
defm BUFFER_STORE_BYTE          : MUBUF_Real_AllAddr_ppu <0x18>;
defm BUFFER_STORE_BYTE_D16_HI   : MUBUF_Real_AllAddr_ppu <0x19>;
defm BUFFER_STORE_SHORT         : MUBUF_Real_AllAddr_ppu <0x1a>;
defm BUFFER_STORE_SHORT_D16_HI  : MUBUF_Real_AllAddr_ppu <0x1b>;
defm BUFFER_STORE_DWORD         : MUBUF_Real_AllAddr_ppu <0x1c>;
defm BUFFER_STORE_DWORDX2       : MUBUF_Real_AllAddr_ppu <0x1d>;

defm BUFFER_LOAD_UBYTE_D16      : MUBUF_Real_AllAddr_ppu <0x20>;
defm BUFFER_LOAD_UBYTE_D16_HI   : MUBUF_Real_AllAddr_ppu <0x21>;
defm BUFFER_LOAD_SBYTE_D16      : MUBUF_Real_AllAddr_ppu <0x22>;
defm BUFFER_LOAD_SBYTE_D16_HI   : MUBUF_Real_AllAddr_ppu <0x23>;
defm BUFFER_LOAD_SHORT_D16      : MUBUF_Real_AllAddr_ppu <0x24>;
defm BUFFER_LOAD_SHORT_D16_HI   : MUBUF_Real_AllAddr_ppu <0x25>;

defm BUFFER_LOAD_FORMAT_D16_HI_X  : MUBUF_Real_AllAddr_ppu <0x26>;
defm BUFFER_STORE_FORMAT_D16_HI_X : MUBUF_Real_AllAddr_ppu <0x27>;

defm BUFFER_ATOMIC_SWAP         : MUBUF_Real_Atomic_ppu <0x40>;
defm BUFFER_ATOMIC_CMPSWAP      : MUBUF_Real_Atomic_ppu <0x41>;
defm BUFFER_ATOMIC_ADD          : MUBUF_Real_Atomic_ppu <0x42>;
defm BUFFER_ATOMIC_SUB          : MUBUF_Real_Atomic_ppu <0x43>;
defm BUFFER_ATOMIC_SMIN         : MUBUF_Real_Atomic_ppu <0x44>;
defm BUFFER_ATOMIC_UMIN         : MUBUF_Real_Atomic_ppu <0x45>;
defm BUFFER_ATOMIC_SMAX         : MUBUF_Real_Atomic_ppu <0x46>;
defm BUFFER_ATOMIC_UMAX         : MUBUF_Real_Atomic_ppu <0x47>;
defm BUFFER_ATOMIC_AND          : MUBUF_Real_Atomic_ppu <0x48>;
defm BUFFER_ATOMIC_OR           : MUBUF_Real_Atomic_ppu <0x49>;
defm BUFFER_ATOMIC_XOR          : MUBUF_Real_Atomic_ppu <0x4a>;
defm BUFFER_ATOMIC_INC          : MUBUF_Real_Atomic_ppu <0x4b>;
defm BUFFER_ATOMIC_DEC          : MUBUF_Real_Atomic_ppu <0x4c>;

defm BUFFER_ATOMIC_SWAP_X2      : MUBUF_Real_Atomic_ppu <0x60>;
// defm BUFFER_ATOMIC_CMPSWAP_X2   : MUBUF_Real_Atomic_ppu <0x61>;
defm BUFFER_ATOMIC_ADD_X2       : MUBUF_Real_Atomic_ppu <0x62>;
defm BUFFER_ATOMIC_SUB_X2       : MUBUF_Real_Atomic_ppu <0x63>;
defm BUFFER_ATOMIC_SMIN_X2      : MUBUF_Real_Atomic_ppu <0x64>;
defm BUFFER_ATOMIC_UMIN_X2      : MUBUF_Real_Atomic_ppu <0x65>;
defm BUFFER_ATOMIC_SMAX_X2      : MUBUF_Real_Atomic_ppu <0x66>;
defm BUFFER_ATOMIC_UMAX_X2      : MUBUF_Real_Atomic_ppu <0x67>;
defm BUFFER_ATOMIC_AND_X2       : MUBUF_Real_Atomic_ppu <0x68>;
defm BUFFER_ATOMIC_OR_X2        : MUBUF_Real_Atomic_ppu <0x69>;
defm BUFFER_ATOMIC_XOR_X2       : MUBUF_Real_Atomic_ppu <0x6a>;
defm BUFFER_ATOMIC_INC_X2       : MUBUF_Real_Atomic_ppu <0x6b>;
defm BUFFER_ATOMIC_DEC_X2       : MUBUF_Real_Atomic_ppu <0x6c>;

// def BUFFER_STORE_LDS_DWORD_ppu   : MUBUF_Real_ppu <0x3d, BUFFER_STORE_LDS_DWORD>;

// def BUFFER_WBINVL1_ppu           : MUBUF_Real_ppu <0x3e, BUFFER_WBINVL1>;
// def BUFFER_WBINVL1_VOL_ppu       : MUBUF_Real_ppu <0x3f, BUFFER_WBINVL1_VOL>;

/*
let SubtargetPredicate = HasAtomicFaddInsts in {

defm BUFFER_ATOMIC_ADD_F32    : MUBUF_Real_AllAddr_vi <0x4d>;
defm BUFFER_ATOMIC_PK_ADD_F16 : MUBUF_Real_AllAddr_vi <0x4e>;

} // End SubtargetPredicate = HasAtomicFaddInsts
*/

class MTBUF_Real_ppu <bits<4> op, MTBUF_Pseudo ps> :
  MTBUF_Real<ps>,
  Enc64,
  PPUMCInstr<ps.PseudoInstr, PPUEncodingFamily.PPU> {
  let AssemblerPredicate = IsPPT;
  let DecoderNamespace = "PPU";

  // FIXME why we need below but AMD don't have
  let hasSideEffects = 1;
  let mayLoad = 1;
  let mayStore = 1;


  let Inst{11-0}  = !if(ps.has_offset, offset, ?);
  let Inst{12}    = ps.offen;
  let Inst{13}    = ps.idxen;
  let Inst{14}    = !if(ps.has_glc, glc, ps.glc_value);
  let Inst{18-15} = op;
  let Inst{22-19} = dfmt;
  let Inst{25-23} = nfmt;
  let Inst{31-26} = 0x3a; //encoding
  let Inst{39-32} = !if(ps.has_vaddr, vaddr, ?);
  let Inst{47-40} = !if(ps.has_vdata, vdata, ?);
  let Inst{52-48} = !if(ps.has_srsrc, srsrc{6-2}, ?);
  let Inst{54}    = !if(ps.has_slc, slc, ?);
  let Inst{55}    = !if(ps.has_tfe, tfe, ?);
  let Inst{63-56} = !if(ps.has_soffset, soffset, ?);
}

multiclass MTBUF_Real_AllAddr_ppu<bits<4> op> {
  def _OFFSET_ppu : MTBUF_Real_ppu <op, !cast<MTBUF_Pseudo>(NAME#"_OFFSET")>;
  def _OFFEN_ppu  : MTBUF_Real_ppu <op, !cast<MTBUF_Pseudo>(NAME#"_OFFEN")>;
  def _IDXEN_ppu  : MTBUF_Real_ppu <op, !cast<MTBUF_Pseudo>(NAME#"_IDXEN")>;
  def _BOTHEN_ppu : MTBUF_Real_ppu <op, !cast<MTBUF_Pseudo>(NAME#"_BOTHEN")>;
}


defm TBUFFER_LOAD_FORMAT_X     : MTBUF_Real_AllAddr_ppu <0x00>;
defm TBUFFER_LOAD_FORMAT_XY    : MTBUF_Real_AllAddr_ppu <0x01>;
// defm TBUFFER_LOAD_FORMAT_XYZ   : MTBUF_Real_AllAddr_ppu <0x02>;
// defm TBUFFER_LOAD_FORMAT_XYZW  : MTBUF_Real_AllAddr_ppu <0x03>;
defm TBUFFER_STORE_FORMAT_X    : MTBUF_Real_AllAddr_ppu <0x04>;
defm TBUFFER_STORE_FORMAT_XY   : MTBUF_Real_AllAddr_ppu <0x05>;
// defm TBUFFER_STORE_FORMAT_XYZ  : MTBUF_Real_AllAddr_ppu <0x06>;
// defm TBUFFER_STORE_FORMAT_XYZW : MTBUF_Real_AllAddr_ppu <0x07>;
/*
let SubtargetPredicate = HasUnpackedD16VMem in {
  defm TBUFFER_LOAD_FORMAT_D16_X_gfx80     : MTBUF_Real_AllAddr_gfx80 <0x08>;
  defm TBUFFER_LOAD_FORMAT_D16_XY_gfx80    : MTBUF_Real_AllAddr_gfx80 <0x09>;
//  defm TBUFFER_LOAD_FORMAT_D16_XYZ_gfx80   : MTBUF_Real_AllAddr_gfx80 <0x0a>;
//  defm TBUFFER_LOAD_FORMAT_D16_XYZW_gfx80  : MTBUF_Real_AllAddr_gfx80 <0x0b>;
  defm TBUFFER_STORE_FORMAT_D16_X_gfx80    : MTBUF_Real_AllAddr_gfx80 <0x0c>;
  defm TBUFFER_STORE_FORMAT_D16_XY_gfx80   : MTBUF_Real_AllAddr_gfx80 <0x0d>;
//  defm TBUFFER_STORE_FORMAT_D16_XYZ_gfx80  : MTBUF_Real_AllAddr_gfx80 <0x0e>;
//  defm TBUFFER_STORE_FORMAT_D16_XYZW_gfx80 : MTBUF_Real_AllAddr_gfx80 <0x0f>;
} // End HasUnpackedD16VMem.
*/
let SubtargetPredicate = HasPackedD16VMem in {
  defm TBUFFER_LOAD_FORMAT_D16_X     : MTBUF_Real_AllAddr_ppu <0x08>;
  defm TBUFFER_LOAD_FORMAT_D16_XY    : MTBUF_Real_AllAddr_ppu <0x09>;
//  defm TBUFFER_LOAD_FORMAT_D16_XYZ   : MTBUF_Real_AllAddr_ppu <0x0a>;
//  defm TBUFFER_LOAD_FORMAT_D16_XYZW  : MTBUF_Real_AllAddr_ppu <0x0b>;
  defm TBUFFER_STORE_FORMAT_D16_X    : MTBUF_Real_AllAddr_ppu <0x0c>;
  defm TBUFFER_STORE_FORMAT_D16_XY   : MTBUF_Real_AllAddr_ppu <0x0d>;
//  defm TBUFFER_STORE_FORMAT_D16_XYZ  : MTBUF_Real_AllAddr_ppu <0x0e>;
//  defm TBUFFER_STORE_FORMAT_D16_XYZW : MTBUF_Real_AllAddr_ppu <0x0f>;
} // End HasUnpackedD16VMem.


def MUBUFInfoTable : GenericTable {
  let FilterClass = "MUBUF_Pseudo";
  let CppTypeName = "MUBUFInfo";
  let Fields = ["Opcode", "BaseOpcode", "elements", "has_vaddr", "has_srsrc", "has_soffset"];

  let PrimaryKey = ["Opcode"];
  let PrimaryKeyName = "getMUBUFOpcodeHelper";
}

def getMUBUFInfoFromOpcode : SearchIndex {
  let Table = MUBUFInfoTable;
  let Key = ["Opcode"];
}

def getMUBUFInfoFromBaseOpcodeAndElements : SearchIndex {
  let Table = MUBUFInfoTable;
  let Key = ["BaseOpcode", "elements"];
}
// include "PPUInstrInfoT_BUF_debug.td"
