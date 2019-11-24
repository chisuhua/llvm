//===----------------------------------------------------------------------===//
// VOP1 Instructions
//===----------------------------------------------------------------------===//

// let VOPAsmPrefer32Bit = 1 in {
defm V_NOP : VOP1Inst <"v_nop", VOP_NONE>;
//}

let isMoveImm = 1, isReMaterializable = 1, isAsCheapAsAMove = 1 in {
defm V_MOV_B32 : VOP1Inst <"v_mov_b32", VOP_I32_I32>;
} // End isMoveImm = 1



// FIXME: Specify SchedRW for READFIRSTLANE_B32
// TODO: Make profile for this, there is VOP3 encoding also
def V_READFIRSTLANE_B32 :
  PPTInst <(outs SReg_32:$vdst),
    (ins VRegOrLds_32:$src0),
    "v_readfirstlane_b32", "$vdst, $src0",
    [(set i32:$vdst, (int_ppu_readfirstlane i32:$src0))]>,
  Enc32 {

  let isCodeGenOnly = 0;
  let UseNamedOperandTable = 1;

  let Size = 4;
  let mayLoad = 0;
  let mayStore = 0;
  let hasSideEffects = 0;

  let VOP1 = 1;
  let VALU = 1;
  let Uses = [TMSK];
  let isConvergent = 1;

  bits<8> vdst;
  bits<9> src0;

  let Inst{8-0}   = src0;
  let Inst{16-9}  = 0x2;
  let Inst{24-17} = vdst;
  let Inst{31-25} = 0x3f; //encoding
}
/*
let SchedRW = [WriteDoubleCvt] in {
defm V_CVT_I32_F64 : VOP1Inst <"v_cvt_i32_f64", VOP_I32_F64,  fp_to_sint>;
defm V_CVT_F64_I32 : VOP1Inst <"v_cvt_f64_i32", VOP1_F64_I32, sint_to_fp>;
defm V_CVT_F32_F64 : VOP1Inst <"v_cvt_f32_f64", VOP_F32_F64,  fpround>;
defm V_CVT_F64_F32 : VOP1Inst <"v_cvt_f64_f32", VOP_F64_F32,  fpextend>;
defm V_CVT_U32_F64 : VOP1Inst <"v_cvt_u32_f64", VOP_I32_F64,  fp_to_uint>;
defm V_CVT_F64_U32 : VOP1Inst <"v_cvt_f64_u32", VOP1_F64_I32, uint_to_fp>;
} // End SchedRW = [WriteDoubleCvt]
*/

let SchedRW = [WriteQuarterRate32] in {
defm V_CVT_F32_I32 : VOP1Inst <"v_cvt_f32_i32", VOP1_F32_I32, sint_to_fp>;
defm V_CVT_F32_U32 : VOP1Inst <"v_cvt_f32_u32", VOP1_F32_I32, uint_to_fp>;
defm V_CVT_U32_F32 : VOP1Inst <"v_cvt_u32_f32", VOP_I32_F32, fp_to_uint>;
defm V_CVT_I32_F32 : VOP1Inst <"v_cvt_i32_f32", VOP_I32_F32, fp_to_sint>;
let FPDPRounding = 1 in {
defm V_CVT_F16_F32 : VOP1Inst <"v_cvt_f16_f32", VOP_F16_F32, fpround>;
} // End FPDPRounding = 1
defm V_CVT_F32_F16 : VOP1Inst <"v_cvt_f32_f16", VOP_F32_F16, fpextend>;
defm V_CVT_RPI_I32_F32 : VOP1Inst <"v_cvt_rpi_i32_f32", VOP_I32_F32, cvt_rpi_i32_f32>;
defm V_CVT_FLR_I32_F32 : VOP1Inst <"v_cvt_flr_i32_f32", VOP_I32_F32, cvt_flr_i32_f32>;
defm V_CVT_OFF_F32_I4 : VOP1Inst  <"v_cvt_off_f32_i4", VOP1_F32_I32>;
} // End SchedRW = [WriteQuarterRate32]

defm V_CVT_F32_UBYTE0 : VOP1Inst <"v_cvt_f32_ubyte0", VOP1_F32_I32, PPUcvt_f32_ubyte0>;
defm V_CVT_F32_UBYTE1 : VOP1Inst <"v_cvt_f32_ubyte1", VOP1_F32_I32, PPUcvt_f32_ubyte1>;
defm V_CVT_F32_UBYTE2 : VOP1Inst <"v_cvt_f32_ubyte2", VOP1_F32_I32, PPUcvt_f32_ubyte2>;
defm V_CVT_F32_UBYTE3 : VOP1Inst <"v_cvt_f32_ubyte3", VOP1_F32_I32, PPUcvt_f32_ubyte3>;

defm V_FRACT_F32 : VOP1Inst <"v_fract_f32", VOP_F32_F32, PPUfract>;
defm V_TRUNC_F32 : VOP1Inst <"v_trunc_f32", VOP_F32_F32, ftrunc>;
defm V_CEIL_F32 : VOP1Inst <"v_ceil_f32", VOP_F32_F32, fceil>;
defm V_RNDNE_F32 : VOP1Inst <"v_rndne_f32", VOP_F32_F32, frint>;
defm V_FLOOR_F32 : VOP1Inst <"v_floor_f32", VOP_F32_F32, ffloor>;

let SchedRW = [WriteQuarterRate32] in {
defm V_EXP_F32 : VOP1Inst <"v_exp_f32", VOP_F32_F32, fexp2>;
defm V_LOG_F32 : VOP1Inst <"v_log_f32", VOP_F32_F32, flog2>;
defm V_RCP_F32 : VOP1Inst <"v_rcp_f32", VOP_F32_F32, PPUrcp>;
defm V_RCP_IFLAG_F32 : VOP1Inst <"v_rcp_iflag_f32", VOP_F32_F32, PPUrcp_iflag>;
defm V_RSQ_F32 : VOP1Inst <"v_rsq_f32", VOP_F32_F32, PPUrsq>;
defm V_SQRT_F32 : VOP1Inst <"v_sqrt_f32", VOP_F32_F32, fsqrt>;
} // End SchedRW = [WriteQuarterRate32]

/*
let SchedRW = [WriteDouble] in {
defm V_RCP_F64 : VOP1Inst <"v_rcp_f64", VOP_F64_F64, PPUrcp>;
defm V_RSQ_F64 : VOP1Inst <"v_rsq_f64", VOP_F64_F64, PPUrsq>;
} // End SchedRW = [WriteDouble];

let SchedRW = [WriteDouble] in {
defm V_SQRT_F64 : VOP1Inst <"v_sqrt_f64", VOP_F64_F64, fsqrt>;
} // End SchedRW = [WriteDouble]
*/

let SchedRW = [WriteQuarterRate32] in {
defm V_SIN_F32 : VOP1Inst <"v_sin_f32", VOP_F32_F32, PPUsin>;
defm V_COS_F32 : VOP1Inst <"v_cos_f32", VOP_F32_F32, PPUcos>;
} // End SchedRW = [WriteQuarterRate32]

defm V_NOT_B32 : VOP1Inst <"v_not_b32", VOP_I32_I32>;
defm V_BFREV_B32 : VOP1Inst <"v_bfrev_b32", VOP_I32_I32>;
defm V_FFBH_U32 : VOP1Inst <"v_ffbh_u32", VOP_I32_I32>;
defm V_FFBL_B32 : VOP1Inst <"v_ffbl_b32", VOP_I32_I32>;
defm V_FFBH_I32 : VOP1Inst <"v_ffbh_i32", VOP_I32_I32>;

/*
let SchedRW = [WriteDoubleAdd] in {
defm V_FREXP_EXP_I32_F64 : VOP1Inst <"v_frexp_exp_i32_f64", VOP_I32_F64, int_amdgcn_frexp_exp>;
defm V_FREXP_MANT_F64 : VOP1Inst <"v_frexp_mant_f64", VOP_F64_F64, int_amdgcn_frexp_mant>;
let FPDPRounding = 1 in {
defm V_FRACT_F64 : VOP1Inst <"v_fract_f64", VOP_F64_F64, AMDGPUfract>;
} // End FPDPRounding = 1
} // End SchedRW = [WriteDoubleAdd]
*/

defm V_FREXP_EXP_I32_F32 : VOP1Inst <"v_frexp_exp_i32_f32", VOP_I32_F32, int_amdgcn_frexp_exp>;
defm V_FREXP_MANT_F32 : VOP1Inst <"v_frexp_mant_f32", VOP_F32_F32, int_amdgcn_frexp_mant>;
/*
let VOPAsmPrefer32Bit = 1 in {
defm V_CLREXCP : VOP1Inst <"v_clrexcp", VOP_NO_EXT<VOP_NONE>>;
}
*/

// Restrict src0 to be VGPR
def VOP_I32_VI32_NO_EXT : VOPProfile<[i32, i32, untyped, untyped]> {
  let Src0RC32 = VRegSrc_32;
  let Src0RC64 = VRegSrc_32;
/*
  let HasExt = 0;
  let HasExtDPP = 0;
  let HasExtSDWA = 0;
  let HasExtSDWA9 = 0;
  */
}
/*
let SubtargetPredicate = HasMovrel, Uses = [M0, TMSK] in {
// v_movreld_b32 is a special case because the destination output
 // register is really a source. It isn't actually read (but may be
 // written), and is only to provide the base register to start
 // indexing from. Tablegen seems to not let you define an implicit
 // virtual register output for the super register being written into,
 // so this must have an implicit def of the register added to it.
defm V_MOVRELD_B32 : VOP1Inst <"v_movreld_b32", VOP_MOVRELD>;
defm V_MOVRELS_B32 : VOP1Inst <"v_movrels_b32", VOP_I32_VI32_NO_EXT>;
defm V_MOVRELSD_B32 : VOP1Inst <"v_movrelsd_b32", VOP_NO_EXT<VOP_I32_I32>>;
} // End Uses = [M0, TMSK]

defm V_MOV_FED_B32 : VOP1Inst <"v_mov_fed_b32", VOP_I32_I32>;
*/
/*
let SubtargetPredicate = isGFX6GFX7 in {
  let SchedRW = [WriteQuarterRate32] in {
    defm V_LOG_CLAMP_F32 :
      VOP1Inst<"v_log_clamp_f32", VOP_F32_F32, int_amdgcn_log_clamp>;
    defm V_RCP_CLAMP_F32 :
      VOP1Inst<"v_rcp_clamp_f32", VOP_F32_F32>;
    defm V_RCP_LEGACY_F32 :
      VOP1Inst<"v_rcp_legacy_f32", VOP_F32_F32, AMDGPUrcp_legacy>;
    defm V_RSQ_CLAMP_F32 :
      VOP1Inst<"v_rsq_clamp_f32", VOP_F32_F32, AMDGPUrsq_clamp>;
    defm V_RSQ_LEGACY_F32 :
      VOP1Inst<"v_rsq_legacy_f32", VOP_F32_F32, AMDGPUrsq_legacy>;
  } // End SchedRW = [WriteQuarterRate32]

} // End SubtargetPredicate = isGFX6GFX7


let SubtargetPredicate = isGFX7GFX8GFX9 in {
  let SchedRW = [WriteQuarterRate32] in {
    defm V_LOG_LEGACY_F32 : VOP1Inst<"v_log_legacy_f32", VOP_F32_F32>;
    defm V_EXP_LEGACY_F32 : VOP1Inst<"v_exp_legacy_f32", VOP_F32_F32>;
  } // End SchedRW = [WriteQuarterRate32]
} // End SubtargetPredicate = isGFX7GFX8GFX9

let SubtargetPredicate = isGFX7Plus in {
  let SchedRW = [WriteDoubleAdd] in {
    defm V_TRUNC_F64 : VOP1Inst<"v_trunc_f64", VOP_F64_F64, ftrunc>;
    defm V_CEIL_F64  : VOP1Inst<"v_ceil_f64", VOP_F64_F64, fceil>;
    defm V_RNDNE_F64 : VOP1Inst<"v_rndne_f64", VOP_F64_F64, frint>;
    defm V_FLOOR_F64 : VOP1Inst<"v_floor_f64", VOP_F64_F64, ffloor>;
  } // End SchedRW = [WriteDoubleAdd]
} // End SubtargetPredicate = isGFX7Plus
*/

let SubtargetPredicate = Has16BitInsts in {

let FPDPRounding = 1 in {
defm V_CVT_F16_U16 : VOP1Inst <"v_cvt_f16_u16", VOP1_F16_I16, uint_to_fp>;
defm V_CVT_F16_I16 : VOP1Inst <"v_cvt_f16_i16", VOP1_F16_I16, sint_to_fp>;
} // End FPDPRounding = 1
defm V_CVT_U16_F16 : VOP1Inst <"v_cvt_u16_f16", VOP_I16_F16, fp_to_uint>;
defm V_CVT_I16_F16 : VOP1Inst <"v_cvt_i16_f16", VOP_I16_F16, fp_to_sint>;
let SchedRW = [WriteQuarterRate32] in {
defm V_RCP_F16 : VOP1Inst <"v_rcp_f16", VOP_F16_F16, PPUrcp>;
defm V_SQRT_F16 : VOP1Inst <"v_sqrt_f16", VOP_F16_F16, fsqrt>;
defm V_RSQ_F16 : VOP1Inst <"v_rsq_f16", VOP_F16_F16, PPUrsq>;
defm V_LOG_F16 : VOP1Inst <"v_log_f16", VOP_F16_F16, flog2>;
defm V_EXP_F16 : VOP1Inst <"v_exp_f16", VOP_F16_F16, fexp2>;
defm V_SIN_F16 : VOP1Inst <"v_sin_f16", VOP_F16_F16, PPUsin>;
defm V_COS_F16 : VOP1Inst <"v_cos_f16", VOP_F16_F16, PPUcos>;
} // End SchedRW = [WriteQuarterRate32]
defm V_FREXP_MANT_F16 : VOP1Inst <"v_frexp_mant_f16", VOP_F16_F16, int_amdgcn_frexp_mant>;
defm V_FREXP_EXP_I16_F16 : VOP1Inst <"v_frexp_exp_i16_f16", VOP_I16_F16, int_amdgcn_frexp_exp>;
defm V_FLOOR_F16 : VOP1Inst <"v_floor_f16", VOP_F16_F16, ffloor>;
defm V_CEIL_F16 : VOP1Inst <"v_ceil_f16", VOP_F16_F16, fceil>;
defm V_TRUNC_F16 : VOP1Inst <"v_trunc_f16", VOP_F16_F16, ftrunc>;
defm V_RNDNE_F16 : VOP1Inst <"v_rndne_f16", VOP_F16_F16, frint>;
let FPDPRounding = 1 in {
defm V_FRACT_F16 : VOP1Inst <"v_fract_f16", VOP_F16_F16, PPUfract>;
} // End FPDPRounding = 1

}

let OtherPredicates = [Has16BitInsts] in {

def : PPUPat<
    (f32 (f16_to_fp i16:$src)),
    (V_CVT_F32_F16_e32 $src)
>;

def : PPUPat<
    (i16 (PPUfp_to_f16 f32:$src)),
    (V_CVT_F16_F32_e32 $src)
>;

}

def VOP_SWAP_I32 : VOPProfile<[i32, i32, i32, untyped]> {
  let Outs32 = (outs VPR_32:$vdst, VPR_32:$vdst1);
  let Ins32 = (ins VPR_32:$src0, VPR_32:$src1);
  let Outs64 = Outs32;
  let Asm32 = " $vdst, $src0";
  let Asm64 = "";
  let Ins64 = (ins);
}

let SubtargetPredicate = isPPT in {
  def V_SWAP_B32 : VOP1_Pseudo<"v_swap_b32", VOP_SWAP_I32, [], 1> {
    let Constraints = "$vdst = $src1, $vdst1 = $src0";
    let DisableEncoding = "$vdst1,$src1";
    let SchedRW = [Write64Bit, Write64Bit];
  }

  defm V_SAT_PK_U8_I16    : VOP1Inst<"v_sat_pk_u8_i16", VOP_I32_I32>;
  defm V_CVT_NORM_I16_F16 : VOP1Inst<"v_cvt_norm_i16_f16", VOP_I16_F16>;
  defm V_CVT_NORM_U16_F16 : VOP1Inst<"v_cvt_norm_u16_f16", VOP_I16_F16>;
} // End SubtargetPredicate = isGFX9Plus

/*
let SubtargetPredicate = isGFX9Only in {
  defm V_SCREEN_PARTITION_4SE_B32 : VOP1Inst <"v_screen_partition_4se_b32", VOP_I32_I32>;
} // End SubtargetPredicate = isGFX9Only

let SubtargetPredicate = isGFX10Plus in {
  defm V_PIPEFLUSH        : VOP1Inst<"v_pipeflush", VOP_NONE>;

  let Uses = [M0] in {
    // FIXME-GFX10: Should V_MOVRELSD_2_B32 be VOP_NO_EXT?
    defm V_MOVRELSD_2_B32 :
      VOP1Inst<"v_movrelsd_2_b32", VOP_NO_EXT<VOP_I32_I32>>;

    def V_SWAPREL_B32 : VOP1_Pseudo<"v_swaprel_b32", VOP_SWAP_I32, [], 1> {
      let Constraints = "$vdst = $src1, $vdst1 = $src0";
      let DisableEncoding = "$vdst1,$src1";
      let SchedRW = [Write64Bit, Write64Bit];
    }
  } // End Uses = [M0]
} // End SubtargetPredicate = isGFX10Plus
*/

defm V_NOP               : VOP1_Real_e32e64_ppu <0x0>;
defm V_MOV_B32           : VOP1_Real_ppu <0x1>;
// defm V_CVT_I32_F64       : VOP1_Real_ppu <0x3>;
// defm V_CVT_F64_I32       : VOP1_Real_ppu <0x4>;
defm V_CVT_F32_I32       : VOP1_Real_ppu <0x5>;
defm V_CVT_F32_U32       : VOP1_Real_ppu <0x6>;
defm V_CVT_U32_F32       : VOP1_Real_ppu <0x7>;
defm V_CVT_I32_F32       : VOP1_Real_ppu <0x8>;
// defm V_MOV_FED_B32       : VOP1_Real_ppu <0x9>;
defm V_CVT_F16_F32       : VOP1_Real_ppu <0xa>;
defm V_CVT_F32_F16       : VOP1_Real_ppu <0xb>;
defm V_CVT_RPI_I32_F32   : VOP1_Real_ppu <0xc>;
defm V_CVT_FLR_I32_F32   : VOP1_Real_ppu <0xd>;
defm V_CVT_OFF_F32_I4    : VOP1_Real_ppu <0xe>;
// defm V_CVT_F32_F64       : VOP1_Real_ppu <0xf>;
// defm V_CVT_F64_F32       : VOP1_Real_ppu <0x10>;
defm V_CVT_F32_UBYTE0    : VOP1_Real_ppu <0x11>;
defm V_CVT_F32_UBYTE1    : VOP1_Real_ppu <0x12>;
defm V_CVT_F32_UBYTE2    : VOP1_Real_ppu <0x13>;
defm V_CVT_F32_UBYTE3    : VOP1_Real_ppu <0x14>;
// defm V_CVT_U32_F64       : VOP1_Real_ppu <0x15>;
//defm V_CVT_F64_U32       : VOP1_Real_ppu <0x16>;
defm V_FRACT_F32         : VOP1_Real_ppu <0x1b>;
defm V_TRUNC_F32         : VOP1_Real_ppu <0x1c>;
defm V_CEIL_F32          : VOP1_Real_ppu <0x1d>;
defm V_RNDNE_F32         : VOP1_Real_ppu <0x1e>;
defm V_FLOOR_F32         : VOP1_Real_ppu <0x1f>;
defm V_EXP_F32           : VOP1_Real_ppu <0x20>;
defm V_LOG_F32           : VOP1_Real_ppu <0x21>;
defm V_RCP_F32           : VOP1_Real_ppu <0x22>;
defm V_RCP_IFLAG_F32     : VOP1_Real_ppu <0x23>;
defm V_RSQ_F32           : VOP1_Real_ppu <0x24>;
// defm V_RCP_F64           : VOP1_Real_ppu <0x25>;
// defm V_RSQ_F64           : VOP1_Real_ppu <0x26>;
defm V_SQRT_F32          : VOP1_Real_ppu <0x27>;
// defm V_SQRT_F64          : VOP1_Real_ppu <0x28>;
defm V_SIN_F32           : VOP1_Real_ppu <0x29>;
defm V_COS_F32           : VOP1_Real_ppu <0x2a>;
defm V_NOT_B32           : VOP1_Real_ppu <0x2b>;
defm V_BFREV_B32         : VOP1_Real_ppu <0x2c>;
defm V_FFBH_U32          : VOP1_Real_ppu <0x2d>;
defm V_FFBL_B32          : VOP1_Real_ppu <0x2e>;
defm V_FFBH_I32          : VOP1_Real_ppu <0x2f>;
// defm V_FREXP_EXP_I32_F64 : VOP1_Real_ppu <0x30>;
// defm V_FREXP_MANT_F64    : VOP1_Real_ppu <0x31>;
// defm V_FRACT_F64         : VOP1_Real_ppu <0x32>;
defm V_FREXP_EXP_I32_F32 : VOP1_Real_ppu <0x33>;
defm V_FREXP_MANT_F32    : VOP1_Real_ppu <0x34>;
// defm V_CLREXCP           : VOP1_Real_ppu <0x35>;
// defm V_MOVRELD_B32       : VOP1_Real_ppu <0x36>;
// defm V_MOVRELS_B32       : VOP1_Real_ppu <0x37>;
// defm V_MOVRELSD_B32      : VOP1_Real_ppu <0x38>;
// defm V_TRUNC_F64         : VOP1_Real_ppu <0x17>;
// defm V_CEIL_F64          : VOP1_Real_ppu <0x18>;
// defm V_FLOOR_F64         : VOP1_Real_ppu <0x1A>;
// defm V_RNDNE_F64         : VOP1_Real_ppu <0x19>;
// defm V_LOG_LEGACY_F32    : VOP1_Real_ppu <0x4c>;
// defm V_EXP_LEGACY_F32    : VOP1_Real_ppu <0x4b>;
defm V_CVT_F16_U16       : VOP1_Real_ppu <0x39>;
defm V_CVT_F16_I16       : VOP1_Real_ppu <0x3a>;
defm V_CVT_U16_F16       : VOP1_Real_ppu <0x3b>;
defm V_CVT_I16_F16       : VOP1_Real_ppu <0x3c>;
defm V_RCP_F16           : VOP1_Real_ppu <0x3d>;
defm V_SQRT_F16          : VOP1_Real_ppu <0x3e>;
defm V_RSQ_F16           : VOP1_Real_ppu <0x3f>;
defm V_LOG_F16           : VOP1_Real_ppu <0x40>;
defm V_EXP_F16           : VOP1_Real_ppu <0x41>;
defm V_FREXP_MANT_F16    : VOP1_Real_ppu <0x42>;
defm V_FREXP_EXP_I16_F16 : VOP1_Real_ppu <0x43>;
defm V_FLOOR_F16         : VOP1_Real_ppu <0x44>;
defm V_CEIL_F16          : VOP1_Real_ppu <0x45>;
defm V_TRUNC_F16         : VOP1_Real_ppu <0x46>;
defm V_RNDNE_F16         : VOP1_Real_ppu <0x47>;
defm V_FRACT_F16         : VOP1_Real_ppu <0x48>;
defm V_SIN_F16           : VOP1_Real_ppu <0x49>;
defm V_COS_F16           : VOP1_Real_ppu <0x4a>;

// defm V_SWAP_B32          : VOP1_Real_ppu <0x51>;
defm V_SAT_PK_U8_I16     : VOP1_Real_ppu<0x4f>;
defm V_CVT_NORM_I16_F16  : VOP1_Real_ppu<0x4d>;
defm V_CVT_NORM_U16_F16  : VOP1_Real_ppu<0x4e>;

// defm V_LOG_CLAMP_F32  : VOP1_Real_ppu<0x026>;
// defm V_RCP_CLAMP_F32  : VOP1_Real_ppu<0x028>;
// defm V_RCP_LEGACY_F32 : VOP1_Real_ppu<0x029>;
// defm V_RSQ_CLAMP_F32  : VOP1_Real_ppu<0x02c>;
// defm V_RSQ_LEGACY_F32 : VOP1_Real_ppu<0x02d>;
// defm V_RCP_CLAMP_F64  : VOP1_Real_ppu<0x030>;
// defm V_RSQ_CLAMP_F64  : VOP1_Real_ppu<0x032>;


// Copy of v_mov_b32 with $vdst as a use operand for use with VGPR
// indexing mode. vdst can't be treated as a def for codegen purposes,
// and an implicit use and def of the super register should be added.
def V_MOV_B32_indirect : VPseudoInst<(outs),
  (ins getVALUDstForVT<i32>.ret:$vdst, getVOPSrc0ForVT<i32>.ret:$src0)>,
  PseudoInstExpansion<(V_MOV_B32_e32_ppu getVALUDstForVT<i32>.ret:$vdst,
                                        getVOPSrc0ForVT<i32>.ret:$src0)> {
  let VOP1 = 1;
  let SubtargetPredicate = isPPT;
}

// This is a pseudo variant of the v_movreld_b32 instruction in which the
// vector operand appears only twice, once as def and once as use. Using this
// pseudo avoids problems with the Two Address instructions pass.
/*
class V_MOVRELD_B32_pseudo<RegisterClass rc> : VPseudoInst <
  (outs rc:$vdst),
  (ins rc:$vsrc, VSrc_b32:$val, i32imm:$offset)> {
  let VOP1 = 1;

  let Constraints = "$vsrc = $vdst";
  let Uses = [M0, TMSK];

  let SubtargetPredicate = HasMovrel;
}
*/

// def V_MOVRELD_B32_V1 : V_MOVRELD_B32_pseudo<VPR_32>;
// def V_MOVRELD_B32_V2 : V_MOVRELD_B32_pseudo<VReg_64>;

let OtherPredicates = [isPPT] in {
def : PPUPat<
  (i32 (anyext i16:$src)),
  (COPY $src)
>;

def : PPUPat<
   (i64 (anyext i16:$src)),
   (REG_SEQUENCE VReg_64,
     (i32 (COPY $src)), sub0,
     (V_MOV_B32_e32 (i32 0)), sub1)
>;

def : PPUPat<
  (i16 (trunc i32:$src)),
  (COPY $src)
>;

def : PPUPat <
  (i16 (trunc i64:$src)),
  (EXTRACT_SUBREG $src, sub0)
>;

} // End OtherPredicates = [isGFX8Plus]


