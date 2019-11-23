//===----------------------------------------------------------------------===//
// Scalar Memory Instructions
//===----------------------------------------------------------------------===//

// We are using the SReg_32_XM0 and not the SReg_32 register class for 32-bit
// SMRD instructions, because the SReg_32_XM0 register class does not include M0
// and writing to M0 from an SMRD instruction will hang the GPU.

// XXX - SMEM instructions do not allow exec for data operand, but
// does sdst for SMRD on SI/CI?
defm S_LOAD_DWORD    : SM_Pseudo_Loads <"s_load_dword", SReg_64, SReg_32>;
defm S_LOAD_DWORDX2  : SM_Pseudo_Loads <"s_load_dwordx2", SReg_64, SReg_64>;


// TODO i change base registerclass from SReg_128 to SReg_64
defm S_BUFFER_LOAD_DWORD : SM_Pseudo_Loads <
  "s_buffer_load_dword", SReg_64, SReg_32
>;

// FIXME: exec_lo/exec_hi appear to be allowed for SMRD loads on
// SI/CI, bit disallowed for SMEM on VI.
defm S_BUFFER_LOAD_DWORDX2 : SM_Pseudo_Loads <
  "s_buffer_load_dwordx2", SReg_64, SReg_64
>;

let SubtargetPredicate = HasScalarStores in {
defm S_STORE_DWORD : SM_Pseudo_Stores <"s_store_dword", SReg_64, SReg_32>;
defm S_STORE_DWORDX2 : SM_Pseudo_Stores <"s_store_dwordx2", SReg_64, SReg_64>;
// defm S_STORE_DWORDX4 : SM_Pseudo_Stores <"s_store_dwordx4", SReg_64, SReg_128>;

defm S_BUFFER_STORE_DWORD : SM_Pseudo_Stores <
  "s_buffer_store_dword", SReg_64, SReg_32
>;

defm S_BUFFER_STORE_DWORDX2 : SM_Pseudo_Stores <
  "s_buffer_store_dwordx2", SReg_64, SReg_64
>;
} // End SubtargetPredicate = HasScalarStores

// def S_MEMTIME : SM_Time_Pseudo <"s_memtime", int_ppu_s_memtime>;
// def S_DCACHE_INV : SM_Inval_Pseudo <"s_dcache_inv", int_ppu_s_dcache_inv>;

// def S_DCACHE_INV_VOL : SM_Inval_Pseudo <"s_dcache_inv_vol", int_ppu_s_dcache_inv_vol>;


let OtherPredicates = [HasScalarStores] in {
// def S_DCACHE_WB     : SM_Inval_Pseudo <"s_dcache_wb", int_ppu_s_dcache_wb>;
// def S_DCACHE_WB_VOL : SM_Inval_Pseudo <"s_dcache_wb_vol", int_ppu_s_dcache_wb_vol>;
} // End OtherPredicates = [HasScalarStores]
// def S_MEMREALTIME   : SM_Time_Pseudo <"s_memrealtime", int_ppu_s_memrealtime>;

/*
let SubtargetPredicate = isGFX10Plus in {
def S_GL1_INV : SM_Inval_Pseudo<"s_gl1_inv">;
def S_GET_WAVEID_IN_WORKGROUP : SM_WaveId_Pseudo <"s_get_waveid_in_workgroup", int_amdgcn_s_get_waveid_in_workgroup>;
} // End SubtargetPredicate = isGFX10Plus
*/

let SubtargetPredicate = HasScalarFlatScratchInsts, Uses = [FLAT_SCR] in {
defm S_SCRATCH_LOAD_DWORD    : SM_Pseudo_Loads <"s_scratch_load_dword",   SReg_64, SReg_32>;
defm S_SCRATCH_LOAD_DWORDX2  : SM_Pseudo_Loads <"s_scratch_load_dwordx2", SReg_64, SReg_64>;
// defm S_SCRATCH_LOAD_DWORDX4  : SM_Pseudo_Loads <"s_scratch_load_dwordx4", SReg_64, SReg_128>;

defm S_SCRATCH_STORE_DWORD   : SM_Pseudo_Stores <"s_scratch_store_dword",   SReg_64, SReg_32>;
defm S_SCRATCH_STORE_DWORDX2 : SM_Pseudo_Stores <"s_scratch_store_dwordx2", SReg_64, SReg_64>;
// defm S_SCRATCH_STORE_DWORDX4 : SM_Pseudo_Stores <"s_scratch_store_dwordx4", SReg_64, SReg_128>;
} // SubtargetPredicate = HasScalarFlatScratchInsts

let SubtargetPredicate = HasScalarAtomics in {

defm S_BUFFER_ATOMIC_SWAP         : SM_Pseudo_Atomics <"s_buffer_atomic_swap", SReg_64, SReg_32>;
defm S_BUFFER_ATOMIC_CMPSWAP      : SM_Pseudo_Atomics <"s_buffer_atomic_cmpswap", SReg_64, SReg_64>;
defm S_BUFFER_ATOMIC_ADD          : SM_Pseudo_Atomics <"s_buffer_atomic_add", SReg_64, SReg_32>;
defm S_BUFFER_ATOMIC_SUB          : SM_Pseudo_Atomics <"s_buffer_atomic_sub", SReg_64, SReg_32>;
defm S_BUFFER_ATOMIC_SMIN         : SM_Pseudo_Atomics <"s_buffer_atomic_smin", SReg_64, SReg_32>;
defm S_BUFFER_ATOMIC_UMIN         : SM_Pseudo_Atomics <"s_buffer_atomic_umin", SReg_64, SReg_32>;
defm S_BUFFER_ATOMIC_SMAX         : SM_Pseudo_Atomics <"s_buffer_atomic_smax", SReg_64, SReg_32>;
defm S_BUFFER_ATOMIC_UMAX         : SM_Pseudo_Atomics <"s_buffer_atomic_umax", SReg_64, SReg_32>;
defm S_BUFFER_ATOMIC_AND          : SM_Pseudo_Atomics <"s_buffer_atomic_and", SReg_64, SReg_32>;
defm S_BUFFER_ATOMIC_OR           : SM_Pseudo_Atomics <"s_buffer_atomic_or", SReg_64, SReg_32>;
defm S_BUFFER_ATOMIC_XOR          : SM_Pseudo_Atomics <"s_buffer_atomic_xor", SReg_64, SReg_32>;
defm S_BUFFER_ATOMIC_INC          : SM_Pseudo_Atomics <"s_buffer_atomic_inc", SReg_64, SReg_32>;
defm S_BUFFER_ATOMIC_DEC          : SM_Pseudo_Atomics <"s_buffer_atomic_dec", SReg_64, SReg_32>;

defm S_BUFFER_ATOMIC_SWAP_X2      : SM_Pseudo_Atomics <"s_buffer_atomic_swap_x2", SReg_64, SReg_64>;
// defm S_BUFFER_ATOMIC_CMPSWAP_X2   : SM_Pseudo_Atomics <"s_buffer_atomic_cmpswap_x2", SReg_64, SReg_64>;
defm S_BUFFER_ATOMIC_ADD_X2       : SM_Pseudo_Atomics <"s_buffer_atomic_add_x2", SReg_64, SReg_64>;
defm S_BUFFER_ATOMIC_SUB_X2       : SM_Pseudo_Atomics <"s_buffer_atomic_sub_x2", SReg_64, SReg_64>;
defm S_BUFFER_ATOMIC_SMIN_X2      : SM_Pseudo_Atomics <"s_buffer_atomic_smin_x2", SReg_64, SReg_64>;
defm S_BUFFER_ATOMIC_UMIN_X2      : SM_Pseudo_Atomics <"s_buffer_atomic_umin_x2", SReg_64, SReg_64>;
defm S_BUFFER_ATOMIC_SMAX_X2      : SM_Pseudo_Atomics <"s_buffer_atomic_smax_x2", SReg_64, SReg_64>;
defm S_BUFFER_ATOMIC_UMAX_X2      : SM_Pseudo_Atomics <"s_buffer_atomic_umax_x2", SReg_64, SReg_64>;
defm S_BUFFER_ATOMIC_AND_X2       : SM_Pseudo_Atomics <"s_buffer_atomic_and_x2", SReg_64, SReg_64>;
defm S_BUFFER_ATOMIC_OR_X2        : SM_Pseudo_Atomics <"s_buffer_atomic_or_x2", SReg_64, SReg_64>;
defm S_BUFFER_ATOMIC_XOR_X2       : SM_Pseudo_Atomics <"s_buffer_atomic_xor_x2", SReg_64, SReg_64>;
defm S_BUFFER_ATOMIC_INC_X2       : SM_Pseudo_Atomics <"s_buffer_atomic_inc_x2", SReg_64, SReg_64>;
defm S_BUFFER_ATOMIC_DEC_X2       : SM_Pseudo_Atomics <"s_buffer_atomic_dec_x2", SReg_64, SReg_64>;

defm S_ATOMIC_SWAP                : SM_Pseudo_Atomics <"s_atomic_swap", SReg_64, SReg_32>;
defm S_ATOMIC_CMPSWAP             : SM_Pseudo_Atomics <"s_atomic_cmpswap", SReg_64, SReg_64>;
defm S_ATOMIC_ADD                 : SM_Pseudo_Atomics <"s_atomic_add", SReg_64, SReg_32>;
defm S_ATOMIC_SUB                 : SM_Pseudo_Atomics <"s_atomic_sub", SReg_64, SReg_32>;
defm S_ATOMIC_SMIN                : SM_Pseudo_Atomics <"s_atomic_smin", SReg_64, SReg_32>;
defm S_ATOMIC_UMIN                : SM_Pseudo_Atomics <"s_atomic_umin", SReg_64, SReg_32>;
defm S_ATOMIC_SMAX                : SM_Pseudo_Atomics <"s_atomic_smax", SReg_64, SReg_32>;
defm S_ATOMIC_UMAX                : SM_Pseudo_Atomics <"s_atomic_umax", SReg_64, SReg_32>;
defm S_ATOMIC_AND                 : SM_Pseudo_Atomics <"s_atomic_and", SReg_64, SReg_32>;
defm S_ATOMIC_OR                  : SM_Pseudo_Atomics <"s_atomic_or", SReg_64, SReg_32>;
defm S_ATOMIC_XOR                 : SM_Pseudo_Atomics <"s_atomic_xor", SReg_64, SReg_32>;
defm S_ATOMIC_INC                 : SM_Pseudo_Atomics <"s_atomic_inc", SReg_64, SReg_32>;
defm S_ATOMIC_DEC                 : SM_Pseudo_Atomics <"s_atomic_dec", SReg_64, SReg_32>;

defm S_ATOMIC_SWAP_X2             : SM_Pseudo_Atomics <"s_atomic_swap_x2", SReg_64, SReg_64>;
// defm S_ATOMIC_CMPSWAP_X2          : SM_Pseudo_Atomics <"s_atomic_cmpswap_x2", SReg_64, SReg_128>;
defm S_ATOMIC_ADD_X2              : SM_Pseudo_Atomics <"s_atomic_add_x2", SReg_64, SReg_64>;
defm S_ATOMIC_SUB_X2              : SM_Pseudo_Atomics <"s_atomic_sub_x2", SReg_64, SReg_64>;
defm S_ATOMIC_SMIN_X2             : SM_Pseudo_Atomics <"s_atomic_smin_x2", SReg_64, SReg_64>;
defm S_ATOMIC_UMIN_X2             : SM_Pseudo_Atomics <"s_atomic_umin_x2", SReg_64, SReg_64>;
defm S_ATOMIC_SMAX_X2             : SM_Pseudo_Atomics <"s_atomic_smax_x2", SReg_64, SReg_64>;
defm S_ATOMIC_UMAX_X2             : SM_Pseudo_Atomics <"s_atomic_umax_x2", SReg_64, SReg_64>;
defm S_ATOMIC_AND_X2              : SM_Pseudo_Atomics <"s_atomic_and_x2", SReg_64, SReg_64>;
defm S_ATOMIC_OR_X2               : SM_Pseudo_Atomics <"s_atomic_or_x2", SReg_64, SReg_64>;
defm S_ATOMIC_XOR_X2              : SM_Pseudo_Atomics <"s_atomic_xor_x2", SReg_64, SReg_64>;
defm S_ATOMIC_INC_X2              : SM_Pseudo_Atomics <"s_atomic_inc_x2", SReg_64, SReg_64>;
defm S_ATOMIC_DEC_X2              : SM_Pseudo_Atomics <"s_atomic_dec_x2", SReg_64, SReg_64>;

} // let SubtargetPredicate = HasScalarAtomics

let SubtargetPredicate = HasScalarAtomics in {
defm S_DCACHE_DISCARD    : SM_Pseudo_Discards <"s_dcache_discard">;
defm S_DCACHE_DISCARD_X2 : SM_Pseudo_Discards <"s_dcache_discard_x2">;
}

//===----------------------------------------------------------------------===//
// Targets
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
// SI
//===----------------------------------------------------------------------===//

class SMRD_Real_ppu <bits<5> op, SM_Pseudo ps>
  : SM_Real<ps>
  , PPUMCInstr<ps.PseudoInstr, PPUEncodingFamily.PPU>
  , Enc32 {

  let AssemblerPredicates = [isPPU];
  let DecoderNamespace = "PPU";

  let Inst{7-0}   = !if(ps.has_offset, offset{7-0}, ?);
  let Inst{8}     = imm;
  let Inst{14-9}  = !if(ps.has_sbase, sbase{6-1}, ?);
  let Inst{21-15} = !if(ps.has_sdst, sdst{6-0}, ?);
  let Inst{26-22} = op;
  let Inst{31-27} = 0x18; //encoding
}

class SMEM_Real_ppu <bits<8> op, SM_Pseudo ps>
  : SM_Real<ps>
  , PPUMCInstr<ps.PseudoInstr, PPUEncodingFamily.PPU>
  , Enc64 {
  bit glc;

  let AssemblerPredicates = [isPPU];
  let DecoderNamespace = "PPU";

  let Inst{5-0}   = !if(ps.has_sbase, sbase{6-1}, ?);
  let Inst{12-6}  = !if(ps.has_sdst, sdst{6-0}, ?);

  let Inst{16} = !if(ps.has_glc, glc, ?);
  let Inst{17} = imm;
  let Inst{25-18} = op;
  let Inst{31-26} = 0x30; //encoding
  let Inst{51-32} = !if(ps.has_offset, offset{19-0}, ?);
}

multiclass SM_Real_Loads_ppu<bits<8> op, string ps,
                            SM_Load_Pseudo immPs = !cast<SM_Load_Pseudo>(ps#_IMM),
                            SM_Load_Pseudo sgprPs = !cast<SM_Load_Pseudo>(ps#_SGPR)> {
  def _IMM_ppu : SMEM_Real_ppu <op, immPs> {
    let InOperandList = (ins immPs.BaseClass:$sbase, smrd_offset_20:$offset, GLC:$glc, DLC:$dlc);
  }
  def _SGPR_ppu : SMEM_Real_ppu <op, sgprPs> {
    let InOperandList = (ins sgprPs.BaseClass:$sbase, SReg_32:$offset, GLC:$glc, DLC:$dlc);
  }
}

/*
defm S_LOAD_DWORD           : SM_Real_Loads_ppu <0x00, "S_LOAD_DWORD">;
defm S_LOAD_DWORDX2         : SM_Real_Loads_ppu <0x01, "S_LOAD_DWORDX2">;
defm S_BUFFER_LOAD_DWORD    : SM_Real_Loads_ppu <0x08, "S_BUFFER_LOAD_DWORD">;
defm S_BUFFER_LOAD_DWORDX2  : SM_Real_Loads_ppu <0x09, "S_BUFFER_LOAD_DWORDX2">;
*/

// def S_MEMTIME_ppu    : SMRD_Real_ppu <0x1e, S_MEMTIME>;
// def S_DCACHE_INV_ppu : SMRD_Real_ppu <0x1f, S_DCACHE_INV>;



class SMEM_Real_Store_ppu <bits<8> op, SM_Pseudo ps> : SMEM_Real_ppu <op, ps> {
  // encoding
  bits<7> sdata;

  let sdst = ?;
  let Inst{12-6}  = !if(ps.has_sdst, sdata{6-0}, ?);
}

multiclass SM_Real_Stores_ppu<bits<8> op, string ps,
                            SM_Store_Pseudo immPs = !cast<SM_Store_Pseudo>(ps#_IMM),
                            SM_Store_Pseudo sgprPs = !cast<SM_Store_Pseudo>(ps#_SGPR)> {
  // FIXME: The operand name $offset is inconsistent with $soff used
  // in the pseudo
  def _IMM_ppu : SMEM_Real_Store_ppu <op, immPs> {
    let InOperandList = (ins immPs.SrcClass:$sdata, immPs.BaseClass:$sbase, smrd_offset_20:$offset, GLC:$glc, DLC:$dlc);
  }

  def _SGPR_ppu : SMEM_Real_Store_ppu <op, sgprPs> {
    let InOperandList = (ins sgprPs.SrcClass:$sdata, sgprPs.BaseClass:$sbase, SReg_32:$offset, GLC:$glc, DLC:$dlc);
  }
}
/*
defm S_STORE_DWORD : SM_Real_Stores_ppu <0x10, "S_STORE_DWORD">;
defm S_STORE_DWORDX2 : SM_Real_Stores_ppu <0x11, "S_STORE_DWORDX2">;

defm S_BUFFER_STORE_DWORD    : SM_Real_Stores_ppu <0x18, "S_BUFFER_STORE_DWORD">;
defm S_BUFFER_STORE_DWORDX2  : SM_Real_Stores_ppu <0x19, "S_BUFFER_STORE_DWORDX2">;
*/

// These instructions use same encoding
// def S_DCACHE_INV_ppu         : SMEM_Real_ppu <0x20, S_DCACHE_INV>;
// def S_DCACHE_WB_ppu          : SMEM_Real_ppu <0x21, S_DCACHE_WB>;
// def S_DCACHE_INV_VOL_ppu     : SMEM_Real_ppu <0x22, S_DCACHE_INV_VOL>;
// def S_DCACHE_WB_VOL_ppu      : SMEM_Real_ppu <0x23, S_DCACHE_WB_VOL>;
// def S_MEMTIME_ppu            : SMEM_Real_ppu <0x24, S_MEMTIME>;
// def S_MEMREALTIME_ppu        : SMEM_Real_ppu <0x25, S_MEMREALTIME>;
/*
defm S_SCRATCH_LOAD_DWORD    : SM_Real_Loads_ppu <0x05, "S_SCRATCH_LOAD_DWORD">;
defm S_SCRATCH_LOAD_DWORDX2  : SM_Real_Loads_ppu <0x06, "S_SCRATCH_LOAD_DWORDX2">;

defm S_SCRATCH_STORE_DWORD   : SM_Real_Stores_ppu <0x15, "S_SCRATCH_STORE_DWORD">;
defm S_SCRATCH_STORE_DWORDX2 : SM_Real_Stores_ppu <0x16, "S_SCRATCH_STORE_DWORDX2">;
*/

//===----------------------------------------------------------------------===//
// GFX9
//===----------------------------------------------------------------------===//

class SMEM_Atomic_Real_ppu <bits<8> op, SM_Atomic_Pseudo ps>
  : SMEM_Real_ppu <op, ps> {

  bits<7> sdata;

  let Constraints = ps.Constraints;
  let DisableEncoding = ps.DisableEncoding;

  let glc = ps.glc;
  let Inst{12-6} = !if(glc, sdst{6-0}, sdata{6-0});
}

multiclass SM_Real_Atomics_ppu<bits<8> op, string ps> {
  def _IMM_ppu       : SMEM_Atomic_Real_ppu <op, !cast<SM_Atomic_Pseudo>(ps#_IMM)>;
  def _SGPR_ppu      : SMEM_Atomic_Real_ppu <op, !cast<SM_Atomic_Pseudo>(ps#_SGPR)>;
  def _IMM_RTN_ppu   : SMEM_Atomic_Real_ppu <op, !cast<SM_Atomic_Pseudo>(ps#_IMM_RTN)>;
  def _SGPR_RTN_ppu  : SMEM_Atomic_Real_ppu <op, !cast<SM_Atomic_Pseudo>(ps#_SGPR_RTN)>;
}
/*
defm S_BUFFER_ATOMIC_SWAP         : SM_Real_Atomics_ppu <0x40, "S_BUFFER_ATOMIC_SWAP">;
defm S_BUFFER_ATOMIC_CMPSWAP      : SM_Real_Atomics_ppu <0x41, "S_BUFFER_ATOMIC_CMPSWAP">;
defm S_BUFFER_ATOMIC_ADD          : SM_Real_Atomics_ppu <0x42, "S_BUFFER_ATOMIC_ADD">;
defm S_BUFFER_ATOMIC_SUB          : SM_Real_Atomics_ppu <0x43, "S_BUFFER_ATOMIC_SUB">;
defm S_BUFFER_ATOMIC_SMIN         : SM_Real_Atomics_ppu <0x44, "S_BUFFER_ATOMIC_SMIN">;
defm S_BUFFER_ATOMIC_UMIN         : SM_Real_Atomics_ppu <0x45, "S_BUFFER_ATOMIC_UMIN">;
defm S_BUFFER_ATOMIC_SMAX         : SM_Real_Atomics_ppu <0x46, "S_BUFFER_ATOMIC_SMAX">;
defm S_BUFFER_ATOMIC_UMAX         : SM_Real_Atomics_ppu <0x47, "S_BUFFER_ATOMIC_UMAX">;
defm S_BUFFER_ATOMIC_AND          : SM_Real_Atomics_ppu <0x48, "S_BUFFER_ATOMIC_AND">;
defm S_BUFFER_ATOMIC_OR           : SM_Real_Atomics_ppu <0x49, "S_BUFFER_ATOMIC_OR">;
defm S_BUFFER_ATOMIC_XOR          : SM_Real_Atomics_ppu <0x4a, "S_BUFFER_ATOMIC_XOR">;
defm S_BUFFER_ATOMIC_INC          : SM_Real_Atomics_ppu <0x4b, "S_BUFFER_ATOMIC_INC">;
defm S_BUFFER_ATOMIC_DEC          : SM_Real_Atomics_ppu <0x4c, "S_BUFFER_ATOMIC_DEC">;

defm S_BUFFER_ATOMIC_SWAP_X2      : SM_Real_Atomics_ppu <0x60, "S_BUFFER_ATOMIC_SWAP_X2">;
// defm S_BUFFER_ATOMIC_CMPSWAP_X2   : SM_Real_Atomics_ppu <0x61, "S_BUFFER_ATOMIC_CMPSWAP_X2">;
defm S_BUFFER_ATOMIC_ADD_X2       : SM_Real_Atomics_ppu <0x62, "S_BUFFER_ATOMIC_ADD_X2">;
defm S_BUFFER_ATOMIC_SUB_X2       : SM_Real_Atomics_ppu <0x63, "S_BUFFER_ATOMIC_SUB_X2">;
defm S_BUFFER_ATOMIC_SMIN_X2      : SM_Real_Atomics_ppu <0x64, "S_BUFFER_ATOMIC_SMIN_X2">;
defm S_BUFFER_ATOMIC_UMIN_X2      : SM_Real_Atomics_ppu <0x65, "S_BUFFER_ATOMIC_UMIN_X2">;
defm S_BUFFER_ATOMIC_SMAX_X2      : SM_Real_Atomics_ppu <0x66, "S_BUFFER_ATOMIC_SMAX_X2">;
defm S_BUFFER_ATOMIC_UMAX_X2      : SM_Real_Atomics_ppu <0x67, "S_BUFFER_ATOMIC_UMAX_X2">;
defm S_BUFFER_ATOMIC_AND_X2       : SM_Real_Atomics_ppu <0x68, "S_BUFFER_ATOMIC_AND_X2">;
defm S_BUFFER_ATOMIC_OR_X2        : SM_Real_Atomics_ppu <0x69, "S_BUFFER_ATOMIC_OR_X2">;
defm S_BUFFER_ATOMIC_XOR_X2       : SM_Real_Atomics_ppu <0x6a, "S_BUFFER_ATOMIC_XOR_X2">;
defm S_BUFFER_ATOMIC_INC_X2       : SM_Real_Atomics_ppu <0x6b, "S_BUFFER_ATOMIC_INC_X2">;
defm S_BUFFER_ATOMIC_DEC_X2       : SM_Real_Atomics_ppu <0x6c, "S_BUFFER_ATOMIC_DEC_X2">;

defm S_ATOMIC_SWAP                : SM_Real_Atomics_ppu <0x80, "S_ATOMIC_SWAP">;
defm S_ATOMIC_CMPSWAP             : SM_Real_Atomics_ppu <0x81, "S_ATOMIC_CMPSWAP">;
defm S_ATOMIC_ADD                 : SM_Real_Atomics_ppu <0x82, "S_ATOMIC_ADD">;
defm S_ATOMIC_SUB                 : SM_Real_Atomics_ppu <0x83, "S_ATOMIC_SUB">;
defm S_ATOMIC_SMIN                : SM_Real_Atomics_ppu <0x84, "S_ATOMIC_SMIN">;
defm S_ATOMIC_UMIN                : SM_Real_Atomics_ppu <0x85, "S_ATOMIC_UMIN">;
defm S_ATOMIC_SMAX                : SM_Real_Atomics_ppu <0x86, "S_ATOMIC_SMAX">;
defm S_ATOMIC_UMAX                : SM_Real_Atomics_ppu <0x87, "S_ATOMIC_UMAX">;
defm S_ATOMIC_AND                 : SM_Real_Atomics_ppu <0x88, "S_ATOMIC_AND">;
defm S_ATOMIC_OR                  : SM_Real_Atomics_ppu <0x89, "S_ATOMIC_OR">;
defm S_ATOMIC_XOR                 : SM_Real_Atomics_ppu <0x8a, "S_ATOMIC_XOR">;
defm S_ATOMIC_INC                 : SM_Real_Atomics_ppu <0x8b, "S_ATOMIC_INC">;
defm S_ATOMIC_DEC                 : SM_Real_Atomics_ppu <0x8c, "S_ATOMIC_DEC">;

defm S_ATOMIC_SWAP_X2             : SM_Real_Atomics_ppu <0xa0, "S_ATOMIC_SWAP_X2">;
// defm S_ATOMIC_CMPSWAP_X2          : SM_Real_Atomics_ppu <0xa1, "S_ATOMIC_CMPSWAP_X2">;
defm S_ATOMIC_ADD_X2              : SM_Real_Atomics_ppu <0xa2, "S_ATOMIC_ADD_X2">;
defm S_ATOMIC_SUB_X2              : SM_Real_Atomics_ppu <0xa3, "S_ATOMIC_SUB_X2">;
defm S_ATOMIC_SMIN_X2             : SM_Real_Atomics_ppu <0xa4, "S_ATOMIC_SMIN_X2">;
defm S_ATOMIC_UMIN_X2             : SM_Real_Atomics_ppu <0xa5, "S_ATOMIC_UMIN_X2">;
defm S_ATOMIC_SMAX_X2             : SM_Real_Atomics_ppu <0xa6, "S_ATOMIC_SMAX_X2">;
defm S_ATOMIC_UMAX_X2             : SM_Real_Atomics_ppu <0xa7, "S_ATOMIC_UMAX_X2">;
defm S_ATOMIC_AND_X2              : SM_Real_Atomics_ppu <0xa8, "S_ATOMIC_AND_X2">;
defm S_ATOMIC_OR_X2               : SM_Real_Atomics_ppu <0xa9, "S_ATOMIC_OR_X2">;
defm S_ATOMIC_XOR_X2              : SM_Real_Atomics_ppu <0xaa, "S_ATOMIC_XOR_X2">;
defm S_ATOMIC_INC_X2              : SM_Real_Atomics_ppu <0xab, "S_ATOMIC_INC_X2">;
defm S_ATOMIC_DEC_X2              : SM_Real_Atomics_ppu <0xac, "S_ATOMIC_DEC_X2">;
*/

/*
multiclass SM_Real_Discard_ppu<bits<8> op, string ps> {
  def _IMM_ppu  : SMEM_Real_ppu <op, !cast<SM_Discard_Pseudo>(ps#_IMM)>;
  def _SGPR_ppu : SMEM_Real_ppu <op, !cast<SM_Discard_Pseudo>(ps#_SGPR)>;
}

defm S_DCACHE_DISCARD    : SM_Real_Discard_ppu <0x28, "S_DCACHE_DISCARD">;
defm S_DCACHE_DISCARD_X2 : SM_Real_Discard_ppu <0x29, "S_DCACHE_DISCARD_X2">;
*/

//===----------------------------------------------------------------------===//
// CI
//===----------------------------------------------------------------------===//
/*
def smrd_literal_offset : NamedOperandU32<"SMRDLiteralOffset",
                                          NamedMatchClass<"SMRDLiteralOffset">> {
  let OperandType = "OPERAND_IMMEDIATE";
}

class SMRD_Real_Load_IMM_ppu <bits<5> op, SM_Load_Pseudo ps> :
  SM_Real<ps>,
  Enc64 {

  let AssemblerPredicates = [isGFX7Only];
  let DecoderNamespace = "GFX7";
  let InOperandList = (ins ps.BaseClass:$sbase, smrd_literal_offset:$offset, GLC:$glc, DLC:$dlc);

  let LGKM_CNT = ps.LGKM_CNT;
  let SMRD = ps.SMRD;
  let mayLoad = ps.mayLoad;
  let mayStore = ps.mayStore;
  let hasSideEffects = ps.hasSideEffects;
  let SchedRW = ps.SchedRW;
  let UseNamedOperandTable = ps.UseNamedOperandTable;

  let Inst{7-0}   = 0xff;
  let Inst{8}     = 0;
  let Inst{14-9}  = sbase{6-1};
  let Inst{21-15} = sdst{6-0};
  let Inst{26-22} = op;
  let Inst{31-27} = 0x18; //encoding
  let Inst{63-32} = offset{31-0};
}

def S_LOAD_DWORD_IMM_ppu           : SMRD_Real_Load_IMM_ppu <0x00, S_LOAD_DWORD_IMM>;
def S_LOAD_DWORDX2_IMM_ppu         : SMRD_Real_Load_IMM_ppu <0x01, S_LOAD_DWORDX2_IMM>;
def S_BUFFER_LOAD_DWORD_IMM_ppu    : SMRD_Real_Load_IMM_ppu <0x08, S_BUFFER_LOAD_DWORD_IMM>;
def S_BUFFER_LOAD_DWORDX2_IMM_ppu  : SMRD_Real_Load_IMM_ppu <0x09, S_BUFFER_LOAD_DWORDX2_IMM>;

class SMRD_Real_ppu <bits<5> op, SM_Pseudo ps>
  : SM_Real<ps>
  , PPUMCInstr<ps.PseudoInstr, PPUEncodingFamily.PPU>
  , Enc32 {

  let AssemblerPredicates = [isPPU];
  let DecoderNamespace = "PPU";

  let Inst{7-0}   = !if(ps.has_offset, offset{7-0}, ?);
  let Inst{8}     = imm;
  let Inst{14-9}  = !if(ps.has_sbase, sbase{6-1}, ?);
  let Inst{21-15} = !if(ps.has_sdst, sdst{6-0}, ?);
  let Inst{26-22} = op;
  let Inst{31-27} = 0x18; //encoding
}

def S_DCACHE_INV_VOL_ppu : SMRD_Real_ppu <0x1d, S_DCACHE_INV_VOL>;
*/

//===----------------------------------------------------------------------===//
// Scalar Memory Patterns
//===----------------------------------------------------------------------===//

def smrd_load : PatFrag <(ops node:$ptr), (load node:$ptr), [{ return isUniformLoad(N);}]> {
  let GISelPredicateCode = [{
    if (!MI.hasOneMemOperand())
      return false;
    if (!isInstrUniform(MI))
      return false;

    // FIXME: We should probably be caching this.
    SmallVector<GEPInfo, 4> AddrInfo;
    getAddrModeInfo(MI, MRI, AddrInfo);

    if (hasVgprParts(AddrInfo))
      return false;
    return true;
  }];
}

def SMRDImm         : ComplexPattern<i64, 2, "SelectSMRDImm">;
def SMRDImm32       : ComplexPattern<i64, 2, "SelectSMRDImm32">;
def SMRDSgpr        : ComplexPattern<i64, 2, "SelectSMRDSgpr">;
def SMRDBufferImm   : ComplexPattern<i32, 1, "SelectSMRDBufferImm">;
def SMRDBufferImm32 : ComplexPattern<i32, 1, "SelectSMRDBufferImm32">;

multiclass SMRD_Pattern <string Instr, ValueType vt> {

  // 1. IMM offset
  def : PPUPat <
    (smrd_load (SMRDImm i64:$sbase, i32:$offset)),
    (vt (!cast<SM_Pseudo>(Instr#"_IMM") $sbase, $offset, 0, 0))
  >;
/*
  // 2. 32-bit IMM offset on CI
  def : PPUPat <
    (smrd_load (SMRDImm32 i64:$sbase, i32:$offset)),
    (vt (!cast<InstSI>(Instr#"_IMM_ppu") $sbase, $offset, 0, 0))> {
    let OtherPredicates = [isGFX7Only];
  }
*/
  // 3. SGPR offset
  def : PPUPat <
    (smrd_load (SMRDSgpr i64:$sbase, i32:$offset)),
    (vt (!cast<SM_Pseudo>(Instr#"_SGPR") $sbase, $offset, 0, 0))
  >;

  // 4. No offset
  def : PPUPat <
    (vt (smrd_load (i64 SReg_64:$sbase))),
    (vt (!cast<SM_Pseudo>(Instr#"_IMM") i64:$sbase, 0, 0, 0))
  >;
}

multiclass SMLoad_Pattern <string Instr, ValueType vt> {
  // 1. Offset as an immediate
  def : PPUPat <
    (SIsbuffer_load v2i32:$sbase, (SMRDBufferImm i32:$offset), i1:$glc, i1:$dlc),
    (vt (!cast<SM_Pseudo>(Instr#"_IMM") $sbase, $offset, (as_i1imm $glc),
                                        (as_i1imm $dlc)))
  >;
/*
  // 2. 32-bit IMM offset on CI
  def : PPUPat <
    (vt (SIsbuffer_load v2i32:$sbase, (SMRDBufferImm32 i32:$offset), i1:$glc, i1:$dlc)),
    (!cast<InstSI>(Instr#"_IMM_ci") $sbase, $offset, (as_i1imm $glc), (as_i1imm $dlc))> {
    let OtherPredicates = [isGFX7Only];
  }
*/
  // 3. Offset loaded in an 32bit SGPR
  def : PPUPat <
    (SIsbuffer_load v2i32:$sbase, i32:$offset, i1:$glc, i1:$dlc),
    (vt (!cast<SM_Pseudo>(Instr#"_SGPR") $sbase, $offset, (as_i1imm $glc),
                                         (as_i1imm $dlc)))
  >;
}

/*
// Global and constant loads can be selected to either MUBUF or SMRD
// instructions, but SMRD instructions are faster so we want the instruction
// selector to prefer those.
let AddedComplexity = 100 in {

defm : SMRD_Pattern <"S_LOAD_DWORD",    i32>;
defm : SMRD_Pattern <"S_LOAD_DWORDX2",  v2i32>;

defm : SMLoad_Pattern <"S_BUFFER_LOAD_DWORD",     i32>;
defm : SMLoad_Pattern <"S_BUFFER_LOAD_DWORDX2",   v2i32>;

defm : SMLoad_Pattern <"S_BUFFER_LOAD_DWORD",     f32>;
defm : SMLoad_Pattern <"S_BUFFER_LOAD_DWORDX2",   v2f32>;
} // End let AddedComplexity = 100

// def : PPUPat <
//   (i64 (readcyclecounter)),
//   (S_MEMTIME)
// >;
*/
