
def atomic_cmp_swap_flat : flat_binary_atomic_op<PPUatomic_cmp_swap>;
def atomic_swap_flat     : flat_binary_atomic_op<atomic_swap>;
def atomic_add_flat      : flat_binary_atomic_op<atomic_load_add>;
def atomic_and_flat      : flat_binary_atomic_op<atomic_load_and>;
def atomic_max_flat      : flat_binary_atomic_op<atomic_load_max>;
def atomic_min_flat      : flat_binary_atomic_op<atomic_load_min>;
def atomic_or_flat       : flat_binary_atomic_op<atomic_load_or>;
def atomic_sub_flat      : flat_binary_atomic_op<atomic_load_sub>;
def atomic_umax_flat     : flat_binary_atomic_op<atomic_load_umax>;
def atomic_umin_flat     : flat_binary_atomic_op<atomic_load_umin>;
def atomic_xor_flat      : flat_binary_atomic_op<atomic_load_xor>;
def atomic_inc_flat      : flat_binary_atomic_op<PPUatomic_inc>;
def atomic_dec_flat      : flat_binary_atomic_op<PPUatomic_dec>;



//===----------------------------------------------------------------------===//
// Flat Instructions
//===----------------------------------------------------------------------===//

def FLAT_LOAD_UBYTE    : FLAT_Load_Pseudo <"flat_load_ubyte", VPR_32>;
def FLAT_LOAD_SBYTE    : FLAT_Load_Pseudo <"flat_load_sbyte", VPR_32>;
def FLAT_LOAD_USHORT   : FLAT_Load_Pseudo <"flat_load_ushort", VPR_32>;
def FLAT_LOAD_SSHORT   : FLAT_Load_Pseudo <"flat_load_sshort", VPR_32>;
def FLAT_LOAD_DWORD    : FLAT_Load_Pseudo <"flat_load_dword", VPR_32>;
def FLAT_LOAD_DWORDX2  : FLAT_Load_Pseudo <"flat_load_dwordx2", VReg_64>;
def FLAT_LOAD_DWORDX4  : FLAT_Load_Pseudo <"flat_load_dwordx4", VReg_128>;
def FLAT_LOAD_DWORDX3  : FLAT_Load_Pseudo <"flat_load_dwordx3", VReg_96>;

def FLAT_STORE_BYTE    : FLAT_Store_Pseudo <"flat_store_byte", VPR_32>;
def FLAT_STORE_SHORT   : FLAT_Store_Pseudo <"flat_store_short", VPR_32>;
def FLAT_STORE_DWORD   : FLAT_Store_Pseudo <"flat_store_dword", VPR_32>;
def FLAT_STORE_DWORDX2 : FLAT_Store_Pseudo <"flat_store_dwordx2", VReg_64>;
def FLAT_STORE_DWORDX4 : FLAT_Store_Pseudo <"flat_store_dwordx4", VReg_128>;
def FLAT_STORE_DWORDX3 : FLAT_Store_Pseudo <"flat_store_dwordx3", VReg_96>;

let SubtargetPredicate = HasD16LoadStore in {
def FLAT_LOAD_UBYTE_D16     : FLAT_Load_Pseudo <"flat_load_ubyte_d16", VPR_32, 1>;
def FLAT_LOAD_UBYTE_D16_HI  : FLAT_Load_Pseudo <"flat_load_ubyte_d16_hi", VPR_32, 1>;
def FLAT_LOAD_SBYTE_D16     : FLAT_Load_Pseudo <"flat_load_sbyte_d16", VPR_32, 1>;
def FLAT_LOAD_SBYTE_D16_HI  : FLAT_Load_Pseudo <"flat_load_sbyte_d16_hi", VPR_32, 1>;
def FLAT_LOAD_SHORT_D16     : FLAT_Load_Pseudo <"flat_load_short_d16", VPR_32, 1>;
def FLAT_LOAD_SHORT_D16_HI  : FLAT_Load_Pseudo <"flat_load_short_d16_hi", VPR_32, 1>;

def FLAT_STORE_BYTE_D16_HI  : FLAT_Store_Pseudo <"flat_store_byte_d16_hi", VPR_32>;
def FLAT_STORE_SHORT_D16_HI : FLAT_Store_Pseudo <"flat_store_short_d16_hi", VPR_32>;
}

defm FLAT_ATOMIC_CMPSWAP    : FLAT_Atomic_Pseudo <"flat_atomic_cmpswap",
                                VPR_32, i32, atomic_cmp_swap_flat,
                                v2i32, VReg_64>;

defm FLAT_ATOMIC_CMPSWAP_X2 : FLAT_Atomic_Pseudo <"flat_atomic_cmpswap_x2",
                                VReg_64, i64, atomic_cmp_swap_flat,
                                v2i64, VReg_128>;

defm FLAT_ATOMIC_SWAP       : FLAT_Atomic_Pseudo <"flat_atomic_swap",
                                VPR_32, i32, atomic_swap_flat>;

defm FLAT_ATOMIC_SWAP_X2    : FLAT_Atomic_Pseudo <"flat_atomic_swap_x2",
                                VReg_64, i64, atomic_swap_flat>;

defm FLAT_ATOMIC_ADD        : FLAT_Atomic_Pseudo <"flat_atomic_add",
                                VPR_32, i32, atomic_add_flat>;

defm FLAT_ATOMIC_SUB        : FLAT_Atomic_Pseudo <"flat_atomic_sub",
                                VPR_32, i32, atomic_sub_flat>;

defm FLAT_ATOMIC_SMIN       : FLAT_Atomic_Pseudo <"flat_atomic_smin",
                                VPR_32, i32, atomic_min_flat>;

defm FLAT_ATOMIC_UMIN       : FLAT_Atomic_Pseudo <"flat_atomic_umin",
                                VPR_32, i32, atomic_umin_flat>;

defm FLAT_ATOMIC_SMAX       : FLAT_Atomic_Pseudo <"flat_atomic_smax",
                                VPR_32, i32, atomic_max_flat>;

defm FLAT_ATOMIC_UMAX       : FLAT_Atomic_Pseudo <"flat_atomic_umax",
                                VPR_32, i32, atomic_umax_flat>;

defm FLAT_ATOMIC_AND        : FLAT_Atomic_Pseudo <"flat_atomic_and",
                                VPR_32, i32, atomic_and_flat>;

defm FLAT_ATOMIC_OR         : FLAT_Atomic_Pseudo <"flat_atomic_or",
                                VPR_32, i32, atomic_or_flat>;

defm FLAT_ATOMIC_XOR        : FLAT_Atomic_Pseudo <"flat_atomic_xor",
                                VPR_32, i32, atomic_xor_flat>;

defm FLAT_ATOMIC_INC        : FLAT_Atomic_Pseudo <"flat_atomic_inc",
                                VPR_32, i32, atomic_inc_flat>;

defm FLAT_ATOMIC_DEC        : FLAT_Atomic_Pseudo <"flat_atomic_dec",
                                VPR_32, i32, atomic_dec_flat>;

defm FLAT_ATOMIC_ADD_X2     : FLAT_Atomic_Pseudo <"flat_atomic_add_x2",
                                VReg_64, i64, atomic_add_flat>;

defm FLAT_ATOMIC_SUB_X2     : FLAT_Atomic_Pseudo <"flat_atomic_sub_x2",
                                VReg_64, i64, atomic_sub_flat>;

defm FLAT_ATOMIC_SMIN_X2    : FLAT_Atomic_Pseudo <"flat_atomic_smin_x2",
                                VReg_64, i64, atomic_min_flat>;

defm FLAT_ATOMIC_UMIN_X2    : FLAT_Atomic_Pseudo <"flat_atomic_umin_x2",
                                VReg_64, i64, atomic_umin_flat>;

defm FLAT_ATOMIC_SMAX_X2    : FLAT_Atomic_Pseudo <"flat_atomic_smax_x2",
                                VReg_64, i64, atomic_max_flat>;

defm FLAT_ATOMIC_UMAX_X2    : FLAT_Atomic_Pseudo <"flat_atomic_umax_x2",
                                VReg_64, i64, atomic_umax_flat>;

defm FLAT_ATOMIC_AND_X2     : FLAT_Atomic_Pseudo <"flat_atomic_and_x2",
                                VReg_64, i64, atomic_and_flat>;

defm FLAT_ATOMIC_OR_X2      : FLAT_Atomic_Pseudo <"flat_atomic_or_x2",
                                VReg_64, i64, atomic_or_flat>;

defm FLAT_ATOMIC_XOR_X2     : FLAT_Atomic_Pseudo <"flat_atomic_xor_x2",
                                VReg_64, i64, atomic_xor_flat>;

defm FLAT_ATOMIC_INC_X2     : FLAT_Atomic_Pseudo <"flat_atomic_inc_x2",
                                VReg_64, i64, atomic_inc_flat>;

defm FLAT_ATOMIC_DEC_X2     : FLAT_Atomic_Pseudo <"flat_atomic_dec_x2",
                                VReg_64, i64, atomic_dec_flat>;

// GFX7-, GFX10-only flat instructions.
// let SubtargetPredicate = isGFX7GFX10 in {

defm FLAT_ATOMIC_FCMPSWAP    : FLAT_Atomic_Pseudo <"flat_atomic_fcmpswap",
                                VPR_32, f32, null_frag, v2f32, VReg_64>;

defm FLAT_ATOMIC_FCMPSWAP_X2 : FLAT_Atomic_Pseudo <"flat_atomic_fcmpswap_x2",
                                VReg_64, f64, null_frag, v2f64, VReg_128>;

defm FLAT_ATOMIC_FMIN        : FLAT_Atomic_Pseudo <"flat_atomic_fmin",
                                VPR_32, f32>;

defm FLAT_ATOMIC_FMAX        : FLAT_Atomic_Pseudo <"flat_atomic_fmax",
                                VPR_32, f32>;

defm FLAT_ATOMIC_FMIN_X2     : FLAT_Atomic_Pseudo <"flat_atomic_fmin_x2",
                                VReg_64, f64>;

defm FLAT_ATOMIC_FMAX_X2     : FLAT_Atomic_Pseudo <"flat_atomic_fmax_x2",
                                VReg_64, f64>;

// } // End SubtargetPredicate = isGFX7GFX10

let SubtargetPredicate = HasFlatGlobalInsts in {
defm GLOBAL_LOAD_UBYTE    : FLAT_Global_Load_Pseudo <"global_load_ubyte", VPR_32>;
defm GLOBAL_LOAD_SBYTE    : FLAT_Global_Load_Pseudo <"global_load_sbyte", VPR_32>;
defm GLOBAL_LOAD_USHORT   : FLAT_Global_Load_Pseudo <"global_load_ushort", VPR_32>;
defm GLOBAL_LOAD_SSHORT   : FLAT_Global_Load_Pseudo <"global_load_sshort", VPR_32>;
defm GLOBAL_LOAD_DWORD    : FLAT_Global_Load_Pseudo <"global_load_dword", VPR_32>;
defm GLOBAL_LOAD_DWORDX2  : FLAT_Global_Load_Pseudo <"global_load_dwordx2", VReg_64>;
defm GLOBAL_LOAD_DWORDX3  : FLAT_Global_Load_Pseudo <"global_load_dwordx3", VReg_96>;
defm GLOBAL_LOAD_DWORDX4  : FLAT_Global_Load_Pseudo <"global_load_dwordx4", VReg_128>;

defm GLOBAL_LOAD_UBYTE_D16    : FLAT_Global_Load_Pseudo <"global_load_ubyte_d16", VPR_32, 1>;
defm GLOBAL_LOAD_UBYTE_D16_HI : FLAT_Global_Load_Pseudo <"global_load_ubyte_d16_hi", VPR_32, 1>;
defm GLOBAL_LOAD_SBYTE_D16    : FLAT_Global_Load_Pseudo <"global_load_sbyte_d16", VPR_32, 1>;
defm GLOBAL_LOAD_SBYTE_D16_HI : FLAT_Global_Load_Pseudo <"global_load_sbyte_d16_hi", VPR_32, 1>;
defm GLOBAL_LOAD_SHORT_D16    : FLAT_Global_Load_Pseudo <"global_load_short_d16", VPR_32, 1>;
defm GLOBAL_LOAD_SHORT_D16_HI : FLAT_Global_Load_Pseudo <"global_load_short_d16_hi", VPR_32, 1>;

defm GLOBAL_STORE_BYTE    : FLAT_Global_Store_Pseudo <"global_store_byte", VPR_32>;
defm GLOBAL_STORE_SHORT   : FLAT_Global_Store_Pseudo <"global_store_short", VPR_32>;
defm GLOBAL_STORE_DWORD   : FLAT_Global_Store_Pseudo <"global_store_dword", VPR_32>;
defm GLOBAL_STORE_DWORDX2 : FLAT_Global_Store_Pseudo <"global_store_dwordx2", VReg_64>;
defm GLOBAL_STORE_DWORDX3 : FLAT_Global_Store_Pseudo <"global_store_dwordx3", VReg_96>;
defm GLOBAL_STORE_DWORDX4 : FLAT_Global_Store_Pseudo <"global_store_dwordx4", VReg_128>;

defm GLOBAL_STORE_BYTE_D16_HI  : FLAT_Global_Store_Pseudo <"global_store_byte_d16_hi", VPR_32>;
defm GLOBAL_STORE_SHORT_D16_HI : FLAT_Global_Store_Pseudo <"global_store_short_d16_hi", VPR_32>;

let is_flat_global = 1 in {
defm GLOBAL_ATOMIC_CMPSWAP : FLAT_Global_Atomic_Pseudo <"global_atomic_cmpswap",
                               VPR_32, i32, PPUatomic_cmp_swap_global,
                               v2i32, VReg_64>;

defm GLOBAL_ATOMIC_CMPSWAP_X2 : FLAT_Global_Atomic_Pseudo <"global_atomic_cmpswap_x2",
                                  VReg_64, i64, PPUatomic_cmp_swap_global,
                                  v2i64, VReg_128>;

defm GLOBAL_ATOMIC_SWAP : FLAT_Global_Atomic_Pseudo <"global_atomic_swap",
                             VPR_32, i32, atomic_swap_global_32>;

defm GLOBAL_ATOMIC_SWAP_X2 : FLAT_Global_Atomic_Pseudo <"global_atomic_swap_x2",
                                VReg_64, i64, atomic_swap_global_64>;

defm GLOBAL_ATOMIC_ADD : FLAT_Global_Atomic_Pseudo <"global_atomic_add",
                           VPR_32, i32, atomic_load_add_global_32>;

defm GLOBAL_ATOMIC_SUB : FLAT_Global_Atomic_Pseudo <"global_atomic_sub",
                           VPR_32, i32, atomic_load_sub_global_32>;

defm GLOBAL_ATOMIC_SMIN : FLAT_Global_Atomic_Pseudo <"global_atomic_smin",
                            VPR_32, i32, atomic_load_min_global_32>;

defm GLOBAL_ATOMIC_UMIN : FLAT_Global_Atomic_Pseudo <"global_atomic_umin",
                            VPR_32, i32, atomic_load_umin_global_32>;

defm GLOBAL_ATOMIC_SMAX : FLAT_Global_Atomic_Pseudo <"global_atomic_smax",
                            VPR_32, i32, atomic_load_max_global_32>;

defm GLOBAL_ATOMIC_UMAX : FLAT_Global_Atomic_Pseudo <"global_atomic_umax",
                            VPR_32, i32, atomic_load_umax_global_32>;

defm GLOBAL_ATOMIC_AND : FLAT_Global_Atomic_Pseudo <"global_atomic_and",
                           VPR_32, i32, atomic_load_and_global_32>;

defm GLOBAL_ATOMIC_OR : FLAT_Global_Atomic_Pseudo <"global_atomic_or",
                          VPR_32, i32, atomic_load_or_global_32>;

defm GLOBAL_ATOMIC_XOR : FLAT_Global_Atomic_Pseudo <"global_atomic_xor",
                           VPR_32, i32, atomic_load_xor_global_32>;

defm GLOBAL_ATOMIC_INC : FLAT_Global_Atomic_Pseudo <"global_atomic_inc",
                           VPR_32, i32, atomic_inc_global_32>;

defm GLOBAL_ATOMIC_DEC : FLAT_Global_Atomic_Pseudo <"global_atomic_dec",
                           VPR_32, i32, atomic_dec_global_32>;

defm GLOBAL_ATOMIC_ADD_X2 : FLAT_Global_Atomic_Pseudo <"global_atomic_add_x2",
                              VReg_64, i64, atomic_load_add_global_64>;

defm GLOBAL_ATOMIC_SUB_X2 : FLAT_Global_Atomic_Pseudo <"global_atomic_sub_x2",
                              VReg_64, i64, atomic_load_sub_global_64>;

defm GLOBAL_ATOMIC_SMIN_X2 : FLAT_Global_Atomic_Pseudo <"global_atomic_smin_x2",
                               VReg_64, i64, atomic_load_min_global_64>;

defm GLOBAL_ATOMIC_UMIN_X2 : FLAT_Global_Atomic_Pseudo <"global_atomic_umin_x2",
                               VReg_64, i64, atomic_load_umin_global_64>;

defm GLOBAL_ATOMIC_SMAX_X2 : FLAT_Global_Atomic_Pseudo <"global_atomic_smax_x2",
                               VReg_64, i64, atomic_load_max_global_64>;

defm GLOBAL_ATOMIC_UMAX_X2 : FLAT_Global_Atomic_Pseudo <"global_atomic_umax_x2",
                               VReg_64, i64, atomic_load_umax_global_64>;

defm GLOBAL_ATOMIC_AND_X2 : FLAT_Global_Atomic_Pseudo <"global_atomic_and_x2",
                              VReg_64, i64, atomic_load_and_global_64>;

defm GLOBAL_ATOMIC_OR_X2 : FLAT_Global_Atomic_Pseudo <"global_atomic_or_x2",
                             VReg_64, i64, atomic_load_or_global_64>;

defm GLOBAL_ATOMIC_XOR_X2 : FLAT_Global_Atomic_Pseudo <"global_atomic_xor_x2",
                              VReg_64, i64, atomic_load_xor_global_64>;

defm GLOBAL_ATOMIC_INC_X2 : FLAT_Global_Atomic_Pseudo <"global_atomic_inc_x2",
                              VReg_64, i64, atomic_inc_global_64>;

defm GLOBAL_ATOMIC_DEC_X2 : FLAT_Global_Atomic_Pseudo <"global_atomic_dec_x2",
                              VReg_64, i64, atomic_dec_global_64>;
} // End is_flat_global = 1

} // End SubtargetPredicate = HasFlatGlobalInsts


let SubtargetPredicate = HasFlatScratchInsts in {
defm SCRATCH_LOAD_UBYTE    : FLAT_Scratch_Load_Pseudo <"scratch_load_ubyte", VPR_32>;
defm SCRATCH_LOAD_SBYTE    : FLAT_Scratch_Load_Pseudo <"scratch_load_sbyte", VPR_32>;
defm SCRATCH_LOAD_USHORT   : FLAT_Scratch_Load_Pseudo <"scratch_load_ushort", VPR_32>;
defm SCRATCH_LOAD_SSHORT   : FLAT_Scratch_Load_Pseudo <"scratch_load_sshort", VPR_32>;
defm SCRATCH_LOAD_DWORD    : FLAT_Scratch_Load_Pseudo <"scratch_load_dword", VPR_32>;
defm SCRATCH_LOAD_DWORDX2  : FLAT_Scratch_Load_Pseudo <"scratch_load_dwordx2", VReg_64>;
defm SCRATCH_LOAD_DWORDX3  : FLAT_Scratch_Load_Pseudo <"scratch_load_dwordx3", VReg_96>;
defm SCRATCH_LOAD_DWORDX4  : FLAT_Scratch_Load_Pseudo <"scratch_load_dwordx4", VReg_128>;

defm SCRATCH_LOAD_UBYTE_D16    : FLAT_Scratch_Load_Pseudo <"scratch_load_ubyte_d16", VPR_32>;
defm SCRATCH_LOAD_UBYTE_D16_HI : FLAT_Scratch_Load_Pseudo <"scratch_load_ubyte_d16_hi", VPR_32>;
defm SCRATCH_LOAD_SBYTE_D16    : FLAT_Scratch_Load_Pseudo <"scratch_load_sbyte_d16", VPR_32>;
defm SCRATCH_LOAD_SBYTE_D16_HI : FLAT_Scratch_Load_Pseudo <"scratch_load_sbyte_d16_hi", VPR_32>;
defm SCRATCH_LOAD_SHORT_D16    : FLAT_Scratch_Load_Pseudo <"scratch_load_short_d16", VPR_32>;
defm SCRATCH_LOAD_SHORT_D16_HI : FLAT_Scratch_Load_Pseudo <"scratch_load_short_d16_hi", VPR_32>;

defm SCRATCH_STORE_BYTE    : FLAT_Scratch_Store_Pseudo <"scratch_store_byte", VPR_32>;
defm SCRATCH_STORE_SHORT   : FLAT_Scratch_Store_Pseudo <"scratch_store_short", VPR_32>;
defm SCRATCH_STORE_DWORD   : FLAT_Scratch_Store_Pseudo <"scratch_store_dword", VPR_32>;
defm SCRATCH_STORE_DWORDX2 : FLAT_Scratch_Store_Pseudo <"scratch_store_dwordx2", VReg_64>;
defm SCRATCH_STORE_DWORDX3 : FLAT_Scratch_Store_Pseudo <"scratch_store_dwordx3", VReg_96>;
defm SCRATCH_STORE_DWORDX4 : FLAT_Scratch_Store_Pseudo <"scratch_store_dwordx4", VReg_128>;

defm SCRATCH_STORE_BYTE_D16_HI : FLAT_Scratch_Store_Pseudo <"scratch_store_byte_d16_hi", VPR_32>;
defm SCRATCH_STORE_SHORT_D16_HI : FLAT_Scratch_Store_Pseudo <"scratch_store_short_d16_hi", VPR_32>;

} // End SubtargetPredicate = HasFlatScratchInsts

let SubtargetPredicate = isGFX10Plus, is_flat_global = 1 in {
  defm GLOBAL_ATOMIC_FCMPSWAP :
    FLAT_Global_Atomic_Pseudo<"global_atomic_fcmpswap", VPR_32, f32>;
  defm GLOBAL_ATOMIC_FMIN :
    FLAT_Global_Atomic_Pseudo<"global_atomic_fmin", VPR_32, f32>;
  defm GLOBAL_ATOMIC_FMAX :
    FLAT_Global_Atomic_Pseudo<"global_atomic_fmax", VPR_32, f32>;
  defm GLOBAL_ATOMIC_FCMPSWAP_X2 :
    FLAT_Global_Atomic_Pseudo<"global_atomic_fcmpswap_x2", VReg_64, f64>;
  defm GLOBAL_ATOMIC_FMIN_X2 :
    FLAT_Global_Atomic_Pseudo<"global_atomic_fmin_x2", VReg_64, f64>;
  defm GLOBAL_ATOMIC_FMAX_X2 :
    FLAT_Global_Atomic_Pseudo<"global_atomic_fmax_x2", VReg_64, f64>;
} // End SubtargetPredicate = isGFX10Plus, is_flat_global = 1

let SubtargetPredicate = HasAtomicFaddInsts, is_flat_global = 1 in {

defm GLOBAL_ATOMIC_ADD_F32 : FLAT_Global_Atomic_Pseudo_NO_RTN <
  "global_atomic_add_f32", VPR_32, f32, atomic_fadd_global_noret
>;
defm GLOBAL_ATOMIC_PK_ADD_F16 : FLAT_Global_Atomic_Pseudo_NO_RTN <
  "global_atomic_pk_add_f16", VPR_32, v2f16, atomic_pk_fadd_global_noret
>;

} // End SubtargetPredicate = HasAtomicFaddInsts

//===----------------------------------------------------------------------===//
// Flat Patterns
//===----------------------------------------------------------------------===//

// Patterns for global loads with no offset.
class FlatLoadPat <FLAT_Pseudo inst, SDPatternOperator node, ValueType vt> : PPTPat <
  (vt (node (FLATOffset i64:$vaddr, i16:$offset, i1:$slc))),
  (inst $vaddr, $offset, 0, 0, $slc)
>;

class FlatLoadPat_D16 <FLAT_Pseudo inst, SDPatternOperator node, ValueType vt> : PPTPat <
  (node (FLATOffset (i64 VReg_64:$vaddr), i16:$offset, i1:$slc), vt:$in),
  (inst $vaddr, $offset, 0, 0, $slc, $in)
>;

class FlatSignedLoadPat_D16 <FLAT_Pseudo inst, SDPatternOperator node, ValueType vt> : PPTPat <
  (node (FLATOffsetSigned (i64 VReg_64:$vaddr), i16:$offset, i1:$slc), vt:$in),
  (inst $vaddr, $offset, 0, 0, $slc, $in)
>;

class FlatLoadAtomicPat <FLAT_Pseudo inst, SDPatternOperator node, ValueType vt> : PPTPat <
  (vt (node (FLATAtomic (i64 VReg_64:$vaddr), i16:$offset, i1:$slc))),
  (inst $vaddr, $offset, 0, 0, $slc)
>;

class FlatLoadSignedPat <FLAT_Pseudo inst, SDPatternOperator node, ValueType vt> : PPTPat <
  (vt (node (FLATOffsetSigned (i64 VReg_64:$vaddr), i16:$offset, i1:$slc))),
  (inst $vaddr, $offset, 0, 0, $slc)
>;

class FlatStorePat <FLAT_Pseudo inst, SDPatternOperator node, ValueType vt, RegisterClass rc = VPR_32> : PPTPat <
  (node vt:$data, (FLATOffset i64:$vaddr, i16:$offset, i1:$slc)),
  (inst $vaddr, rc:$data, $offset, 0, 0, $slc)
>;

class FlatStoreSignedPat <FLAT_Pseudo inst, SDPatternOperator node, ValueType vt, RegisterClass rc = VPR_32> : PPTPat <
  (node vt:$data, (FLATOffsetSigned i64:$vaddr, i16:$offset, i1:$slc)),
  (inst $vaddr, rc:$data, $offset, 0, 0, $slc)
>;

class FlatStoreAtomicPat <FLAT_Pseudo inst, SDPatternOperator node, ValueType vt, RegisterClass rc = VPR_32> : PPTPat <
  // atomic store follows atomic binop convention so the address comes
  // first.
  (node (FLATAtomic i64:$vaddr, i16:$offset, i1:$slc), vt:$data),
  (inst $vaddr, rc:$data, $offset, 0, 0, $slc)
>;

class FlatStoreSignedAtomicPat <FLAT_Pseudo inst, SDPatternOperator node, ValueType vt, RegisterClass rc = VPR_32> : PPTPat <
  // atomic store follows atomic binop convention so the address comes
  // first.
  (node (FLATSignedAtomic i64:$vaddr, i16:$offset, i1:$slc), vt:$data),
  (inst $vaddr, rc:$data, $offset, 0, 0, $slc)
>;

class FlatAtomicPat <FLAT_Pseudo inst, SDPatternOperator node, ValueType vt,
                     ValueType data_vt = vt> : PPTPat <
  (vt (node (FLATAtomic i64:$vaddr, i16:$offset, i1:$slc), data_vt:$data)),
  (inst $vaddr, $data, $offset, $slc)
>;

class FlatAtomicPatNoRtn <FLAT_Pseudo inst, SDPatternOperator node, ValueType vt> : PPTPat <
  (node (FLATAtomic i64:$vaddr, i16:$offset, i1:$slc), vt:$data),
  (inst $vaddr, $data, $offset, $slc)
>;

class FlatSignedAtomicPat <FLAT_Pseudo inst, SDPatternOperator node, ValueType vt,
                     ValueType data_vt = vt> : PPTPat <
  (vt (node (FLATSignedAtomic i64:$vaddr, i16:$offset, i1:$slc), data_vt:$data)),
  (inst $vaddr, $data, $offset, $slc)
>;

let OtherPredicates = [HasFlatAddressSpace] in {

def : FlatLoadPat <FLAT_LOAD_UBYTE, extloadi8_flat, i32>;
def : FlatLoadPat <FLAT_LOAD_UBYTE, zextloadi8_flat, i32>;
def : FlatLoadPat <FLAT_LOAD_SBYTE, sextloadi8_flat, i32>;
def : FlatLoadPat <FLAT_LOAD_UBYTE, extloadi8_flat, i16>;
def : FlatLoadPat <FLAT_LOAD_UBYTE, zextloadi8_flat, i16>;
def : FlatLoadPat <FLAT_LOAD_SBYTE, sextloadi8_flat, i16>;
def : FlatLoadPat <FLAT_LOAD_USHORT, extloadi16_flat, i32>;
def : FlatLoadPat <FLAT_LOAD_USHORT, zextloadi16_flat, i32>;
def : FlatLoadPat <FLAT_LOAD_USHORT, load_flat, i16>;
def : FlatLoadPat <FLAT_LOAD_SSHORT, sextloadi16_flat, i32>;
def : FlatLoadPat <FLAT_LOAD_DWORDX3, load_flat, v3i32>;
def : FlatLoadPat <FLAT_LOAD_DWORDX4, load_flat, v4i32>;

def : FlatLoadAtomicPat <FLAT_LOAD_DWORD, atomic_load_32_flat, i32>;
def : FlatLoadAtomicPat <FLAT_LOAD_DWORDX2, atomic_load_64_flat, i64>;

def : FlatStorePat <FLAT_STORE_BYTE, truncstorei8_flat, i32>;
def : FlatStorePat <FLAT_STORE_SHORT, truncstorei16_flat, i32>;

foreach vt = [i32, f32, v2i16, v2f16, p2, p3, p5, p6] in {
def : FlatLoadPat <FLAT_LOAD_DWORD, load_flat, vt>;
def : FlatStorePat <FLAT_STORE_DWORD, store_flat, vt>;
}

foreach vt = VReg_64.RegTypes in {
def : FlatStorePat <FLAT_STORE_DWORDX2, store_flat, vt, VReg_64>;
def : FlatLoadPat <FLAT_LOAD_DWORDX2, load_flat, vt>;
}

def : FlatStorePat <FLAT_STORE_DWORDX3, store_flat, v3i32, VReg_96>;
def : FlatStorePat <FLAT_STORE_DWORDX4, store_flat, v4i32, VReg_128>;

def : FlatStoreAtomicPat <FLAT_STORE_DWORD, atomic_store_flat_32, i32>;
def : FlatStoreAtomicPat <FLAT_STORE_DWORDX2, atomic_store_flat_64, i64, VReg_64>;

def : FlatAtomicPat <FLAT_ATOMIC_ADD_RTN, atomic_load_add_global_32, i32>;
def : FlatAtomicPat <FLAT_ATOMIC_SUB_RTN, atomic_load_sub_global_32, i32>;
def : FlatAtomicPat <FLAT_ATOMIC_INC_RTN, atomic_inc_global_32, i32>;
def : FlatAtomicPat <FLAT_ATOMIC_DEC_RTN, atomic_dec_global_32, i32>;
def : FlatAtomicPat <FLAT_ATOMIC_AND_RTN, atomic_load_and_global_32, i32>;
def : FlatAtomicPat <FLAT_ATOMIC_SMAX_RTN, atomic_load_max_global_32, i32>;
def : FlatAtomicPat <FLAT_ATOMIC_UMAX_RTN, atomic_load_umax_global_32, i32>;
def : FlatAtomicPat <FLAT_ATOMIC_SMIN_RTN, atomic_load_min_global_32, i32>;
def : FlatAtomicPat <FLAT_ATOMIC_UMIN_RTN, atomic_load_umin_global_32, i32>;
def : FlatAtomicPat <FLAT_ATOMIC_OR_RTN, atomic_load_or_global_32, i32>;
def : FlatAtomicPat <FLAT_ATOMIC_SWAP_RTN, atomic_swap_global_32, i32>;
def : FlatAtomicPat <FLAT_ATOMIC_CMPSWAP_RTN, PPUatomic_cmp_swap_global, i32, v2i32>;
def : FlatAtomicPat <FLAT_ATOMIC_XOR_RTN, atomic_load_xor_global_32, i32>;

def : FlatAtomicPat <FLAT_ATOMIC_ADD_X2_RTN, atomic_load_add_global_64, i64>;
def : FlatAtomicPat <FLAT_ATOMIC_SUB_X2_RTN, atomic_load_sub_global_64, i64>;
def : FlatAtomicPat <FLAT_ATOMIC_INC_X2_RTN, atomic_inc_global_64, i64>;
def : FlatAtomicPat <FLAT_ATOMIC_DEC_X2_RTN, atomic_dec_global_64, i64>;
def : FlatAtomicPat <FLAT_ATOMIC_AND_X2_RTN, atomic_load_and_global_64, i64>;
def : FlatAtomicPat <FLAT_ATOMIC_SMAX_X2_RTN, atomic_load_max_global_64, i64>;
def : FlatAtomicPat <FLAT_ATOMIC_UMAX_X2_RTN, atomic_load_umax_global_64, i64>;
def : FlatAtomicPat <FLAT_ATOMIC_SMIN_X2_RTN, atomic_load_min_global_64, i64>;
def : FlatAtomicPat <FLAT_ATOMIC_UMIN_X2_RTN, atomic_load_umin_global_64, i64>;
def : FlatAtomicPat <FLAT_ATOMIC_OR_X2_RTN, atomic_load_or_global_64, i64>;
def : FlatAtomicPat <FLAT_ATOMIC_SWAP_X2_RTN, atomic_swap_global_64, i64>;
def : FlatAtomicPat <FLAT_ATOMIC_CMPSWAP_X2_RTN, PPUatomic_cmp_swap_global, i64, v2i64>;
def : FlatAtomicPat <FLAT_ATOMIC_XOR_X2_RTN, atomic_load_xor_global_64, i64>;

def : FlatStorePat <FLAT_STORE_BYTE, truncstorei8_flat, i16>;
def : FlatStorePat <FLAT_STORE_SHORT, store_flat, i16>;

let OtherPredicates = [D16PreservesUnusedBits] in {
def : FlatStorePat <FLAT_STORE_SHORT_D16_HI, truncstorei16_hi16_flat, i32>;
def : FlatStorePat <FLAT_STORE_BYTE_D16_HI, truncstorei8_hi16_flat, i32>;

def : FlatLoadPat_D16 <FLAT_LOAD_UBYTE_D16_HI, az_extloadi8_d16_hi_flat, v2i16>;
def : FlatLoadPat_D16 <FLAT_LOAD_UBYTE_D16_HI, az_extloadi8_d16_hi_flat, v2f16>;
def : FlatLoadPat_D16 <FLAT_LOAD_SBYTE_D16_HI, sextloadi8_d16_hi_flat, v2i16>;
def : FlatLoadPat_D16 <FLAT_LOAD_SBYTE_D16_HI, sextloadi8_d16_hi_flat, v2f16>;
def : FlatLoadPat_D16 <FLAT_LOAD_SHORT_D16_HI, load_d16_hi_flat, v2i16>;
def : FlatLoadPat_D16 <FLAT_LOAD_SHORT_D16_HI, load_d16_hi_flat, v2f16>;

def : FlatLoadPat_D16 <FLAT_LOAD_UBYTE_D16, az_extloadi8_d16_lo_flat, v2i16>;
def : FlatLoadPat_D16 <FLAT_LOAD_UBYTE_D16, az_extloadi8_d16_lo_flat, v2f16>;
def : FlatLoadPat_D16 <FLAT_LOAD_SBYTE_D16, sextloadi8_d16_lo_flat, v2i16>;
def : FlatLoadPat_D16 <FLAT_LOAD_SBYTE_D16, sextloadi8_d16_lo_flat, v2f16>;
def : FlatLoadPat_D16 <FLAT_LOAD_SHORT_D16, load_d16_lo_flat, v2i16>;
def : FlatLoadPat_D16 <FLAT_LOAD_SHORT_D16, load_d16_lo_flat, v2f16>;
}

} // End OtherPredicates = [HasFlatAddressSpace]

let OtherPredicates = [HasFlatGlobalInsts], AddedComplexity = 10 in {

def : FlatLoadSignedPat <GLOBAL_LOAD_UBYTE, extloadi8_global, i32>;
def : FlatLoadSignedPat <GLOBAL_LOAD_UBYTE, zextloadi8_global, i32>;
def : FlatLoadSignedPat <GLOBAL_LOAD_SBYTE, sextloadi8_global, i32>;
def : FlatLoadSignedPat <GLOBAL_LOAD_UBYTE, extloadi8_global, i16>;
def : FlatLoadSignedPat <GLOBAL_LOAD_UBYTE, zextloadi8_global, i16>;
def : FlatLoadSignedPat <GLOBAL_LOAD_SBYTE, sextloadi8_global, i16>;
def : FlatLoadSignedPat <GLOBAL_LOAD_USHORT, extloadi16_global, i32>;
def : FlatLoadSignedPat <GLOBAL_LOAD_USHORT, zextloadi16_global, i32>;
def : FlatLoadSignedPat <GLOBAL_LOAD_SSHORT, sextloadi16_global, i32>;
def : FlatLoadSignedPat <GLOBAL_LOAD_USHORT, load_global, i16>;

foreach vt = [i32, f32, v2i16, v2f16, p2, p3, p5, p6] in {
def : FlatLoadSignedPat <GLOBAL_LOAD_DWORD, load_global, vt>;
def : FlatStoreSignedPat <GLOBAL_STORE_DWORD, store_global, vt, VPR_32>;
}

foreach vt = VReg_64.RegTypes in {
def : FlatLoadSignedPat <GLOBAL_LOAD_DWORDX2, load_global, vt>;
def : FlatStoreSignedPat <GLOBAL_STORE_DWORDX2, store_global, vt, VReg_64>;
}

def : FlatLoadSignedPat <GLOBAL_LOAD_DWORDX3, load_global, v3i32>;
def : FlatLoadSignedPat <GLOBAL_LOAD_DWORDX4, load_global, v4i32>;

def : FlatLoadAtomicPat <GLOBAL_LOAD_DWORD, atomic_load_32_global, i32>;
def : FlatLoadAtomicPat <GLOBAL_LOAD_DWORDX2, atomic_load_64_global, i64>;

def : FlatStoreSignedPat <GLOBAL_STORE_BYTE, truncstorei8_global, i32, VPR_32>;
def : FlatStoreSignedPat <GLOBAL_STORE_BYTE, truncstorei8_global, i16, VPR_32>;
def : FlatStoreSignedPat <GLOBAL_STORE_SHORT, truncstorei16_global, i32, VPR_32>;
def : FlatStoreSignedPat <GLOBAL_STORE_SHORT, store_global, i16, VPR_32>;
def : FlatStoreSignedPat <GLOBAL_STORE_DWORDX3, store_global, v3i32, VReg_96>;
def : FlatStoreSignedPat <GLOBAL_STORE_DWORDX4, store_global, v4i32, VReg_128>;

let OtherPredicates = [D16PreservesUnusedBits] in {
def : FlatStoreSignedPat <GLOBAL_STORE_SHORT_D16_HI, truncstorei16_hi16_global, i32>;
def : FlatStoreSignedPat <GLOBAL_STORE_BYTE_D16_HI, truncstorei8_hi16_global, i32>;

def : FlatSignedLoadPat_D16 <GLOBAL_LOAD_UBYTE_D16_HI, az_extloadi8_d16_hi_global, v2i16>;
def : FlatSignedLoadPat_D16 <GLOBAL_LOAD_UBYTE_D16_HI, az_extloadi8_d16_hi_global, v2f16>;
def : FlatSignedLoadPat_D16 <GLOBAL_LOAD_SBYTE_D16_HI, sextloadi8_d16_hi_global, v2i16>;
def : FlatSignedLoadPat_D16 <GLOBAL_LOAD_SBYTE_D16_HI, sextloadi8_d16_hi_global, v2f16>;
def : FlatSignedLoadPat_D16 <GLOBAL_LOAD_SHORT_D16_HI, load_d16_hi_global, v2i16>;
def : FlatSignedLoadPat_D16 <GLOBAL_LOAD_SHORT_D16_HI, load_d16_hi_global, v2f16>;

def : FlatSignedLoadPat_D16 <GLOBAL_LOAD_UBYTE_D16, az_extloadi8_d16_lo_global, v2i16>;
def : FlatSignedLoadPat_D16 <GLOBAL_LOAD_UBYTE_D16, az_extloadi8_d16_lo_global, v2f16>;
def : FlatSignedLoadPat_D16 <GLOBAL_LOAD_SBYTE_D16, sextloadi8_d16_lo_global, v2i16>;
def : FlatSignedLoadPat_D16 <GLOBAL_LOAD_SBYTE_D16, sextloadi8_d16_lo_global, v2f16>;
def : FlatSignedLoadPat_D16 <GLOBAL_LOAD_SHORT_D16, load_d16_lo_global, v2i16>;
def : FlatSignedLoadPat_D16 <GLOBAL_LOAD_SHORT_D16, load_d16_lo_global, v2f16>;
}

def : FlatStoreSignedAtomicPat <GLOBAL_STORE_DWORD, store_atomic_global, i32>;
def : FlatStoreSignedAtomicPat <GLOBAL_STORE_DWORDX2, store_atomic_global, i64, VReg_64>;

def : FlatSignedAtomicPat <GLOBAL_ATOMIC_ADD_RTN, atomic_load_add_global_32, i32>;
def : FlatSignedAtomicPat <GLOBAL_ATOMIC_SUB_RTN, atomic_load_sub_global_32, i32>;
def : FlatSignedAtomicPat <GLOBAL_ATOMIC_INC_RTN, atomic_inc_global_32, i32>;
def : FlatSignedAtomicPat <GLOBAL_ATOMIC_DEC_RTN, atomic_dec_global_32, i32>;
def : FlatSignedAtomicPat <GLOBAL_ATOMIC_AND_RTN, atomic_load_and_global_32, i32>;
def : FlatSignedAtomicPat <GLOBAL_ATOMIC_SMAX_RTN, atomic_load_max_global_32, i32>;
def : FlatSignedAtomicPat <GLOBAL_ATOMIC_UMAX_RTN, atomic_load_umax_global_32, i32>;
def : FlatSignedAtomicPat <GLOBAL_ATOMIC_SMIN_RTN, atomic_load_min_global_32, i32>;
def : FlatSignedAtomicPat <GLOBAL_ATOMIC_UMIN_RTN, atomic_load_umin_global_32, i32>;
def : FlatSignedAtomicPat <GLOBAL_ATOMIC_OR_RTN, atomic_load_or_global_32, i32>;
def : FlatSignedAtomicPat <GLOBAL_ATOMIC_SWAP_RTN, atomic_swap_global_32, i32>;
def : FlatSignedAtomicPat <GLOBAL_ATOMIC_CMPSWAP_RTN, PPUatomic_cmp_swap_global, i32, v2i32>;
def : FlatSignedAtomicPat <GLOBAL_ATOMIC_XOR_RTN, atomic_load_xor_global_32, i32>;

def : FlatSignedAtomicPat <GLOBAL_ATOMIC_ADD_X2_RTN, atomic_load_add_global_64, i64>;
def : FlatSignedAtomicPat <GLOBAL_ATOMIC_SUB_X2_RTN, atomic_load_sub_global_64, i64>;
def : FlatSignedAtomicPat <GLOBAL_ATOMIC_INC_X2_RTN, atomic_inc_global_64, i64>;
def : FlatSignedAtomicPat <GLOBAL_ATOMIC_DEC_X2_RTN, atomic_dec_global_64, i64>;
def : FlatSignedAtomicPat <GLOBAL_ATOMIC_AND_X2_RTN, atomic_load_and_global_64, i64>;
def : FlatSignedAtomicPat <GLOBAL_ATOMIC_SMAX_X2_RTN, atomic_load_max_global_64, i64>;
def : FlatSignedAtomicPat <GLOBAL_ATOMIC_UMAX_X2_RTN, atomic_load_umax_global_64, i64>;
def : FlatSignedAtomicPat <GLOBAL_ATOMIC_SMIN_X2_RTN, atomic_load_min_global_64, i64>;
def : FlatSignedAtomicPat <GLOBAL_ATOMIC_UMIN_X2_RTN, atomic_load_umin_global_64, i64>;
def : FlatSignedAtomicPat <GLOBAL_ATOMIC_OR_X2_RTN, atomic_load_or_global_64, i64>;
def : FlatSignedAtomicPat <GLOBAL_ATOMIC_SWAP_X2_RTN, atomic_swap_global_64, i64>;
def : FlatSignedAtomicPat <GLOBAL_ATOMIC_CMPSWAP_X2_RTN, PPUatomic_cmp_swap_global, i64, v2i64>;
def : FlatSignedAtomicPat <GLOBAL_ATOMIC_XOR_X2_RTN, atomic_load_xor_global_64, i64>;

def : FlatAtomicPatNoRtn <GLOBAL_ATOMIC_ADD_F32,    atomic_fadd_global_noret, f32>;
def : FlatAtomicPatNoRtn <GLOBAL_ATOMIC_PK_ADD_F16, atomic_pk_fadd_global_noret, v2f16>;

} // End OtherPredicates = [HasFlatGlobalInsts], AddedComplexity = 10


//===----------------------------------------------------------------------===//
// Target
//===----------------------------------------------------------------------===//
//===----------------------------------------------------------------------===//
// VI
//===----------------------------------------------------------------------===//

class FLAT_Real_vi <bits<7> op, FLAT_Pseudo ps> :
  FLAT_Real <op, ps>,
  PPUMCInstr <ps.PseudoInstr, PPUEncodingFamily.PPU> {
  // let AssemblerPredicate = isGFX8GFX9;
  // let DecoderNamespace = "GFX8";
  let AssemblerPredicate = IsPPT;
  let DecoderNamespace = "PPU";
}

multiclass FLAT_Real_AllAddr_vi<bits<7> op> {
  def _vi : FLAT_Real_vi<op, !cast<FLAT_Pseudo>(NAME)>;
  def _SADDR_vi : FLAT_Real_vi<op, !cast<FLAT_Pseudo>(NAME#"_SADDR")>;
}

//===----------------------------------------------------------------------===//
// GFX10.
//===----------------------------------------------------------------------===//

class FLAT_Real_ppu<bits<7> op, FLAT_Pseudo ps> :
    FLAT_Real<op, ps>, PPUMCInstr<ps.PseudoInstr, PPUEncodingFamily.PPU> {
  // let AssemblerPredicate = isGFX10Plus;
  // let DecoderNamespace = "GFX10";
  let AssemblerPredicate = IsPPT;
  let DecoderNamespace = "PPU";

  let Inst{11-0}  = {offset{12}, offset{10-0}};
  let Inst{12}    = !if(ps.has_dlc, dlc, ps.dlcValue);
  let Inst{54-48} = !if(ps.has_saddr, !if(ps.enabled_saddr, saddr, 0x7d), 0x7d);
  let Inst{55}    = 0;
}


multiclass FLAT_Real_Base_ppu<bits<7> op> {
  def _ppu :
    FLAT_Real_ppu<op, !cast<FLAT_Pseudo>(NAME)>;
}

multiclass FLAT_Real_RTN_ppu<bits<7> op> {
  def _RTN_ppu :
    FLAT_Real_ppu<op, !cast<FLAT_Pseudo>(NAME#"_RTN")>;
}

multiclass FLAT_Real_SADDR_ppu<bits<7> op> {
  def _SADDR_ppu :
    FLAT_Real_ppu<op, !cast<FLAT_Pseudo>(NAME#"_SADDR")>;
}

multiclass FLAT_Real_SADDR_RTN_ppu<bits<7> op> {
  def _SADDR_RTN_ppu :
    FLAT_Real_ppu<op, !cast<FLAT_Pseudo>(NAME#"_SADDR_RTN")>;
}


multiclass FLAT_Real_AllAddr_ppu<bits<7> op> :
  FLAT_Real_Base_ppu<op>,
  FLAT_Real_SADDR_ppu<op>;

multiclass FLAT_Real_Atomics_ppu<bits<7> op> :
  FLAT_Real_Base_ppu<op>,
  FLAT_Real_RTN_ppu<op>;

multiclass FLAT_Real_GlblAtomics_ppu<bits<7> op> :
  FLAT_Real_AllAddr_ppu<op>,
  FLAT_Real_RTN_ppu<op>,
  FLAT_Real_SADDR_RTN_ppu<op>;


// ENC_FLAT.
defm FLAT_LOAD_UBYTE            : FLAT_Real_Base_ppu<0x008>;
defm FLAT_LOAD_SBYTE            : FLAT_Real_Base_ppu<0x009>;
defm FLAT_LOAD_USHORT           : FLAT_Real_Base_ppu<0x00a>;
defm FLAT_LOAD_SSHORT           : FLAT_Real_Base_ppu<0x00b>;
defm FLAT_LOAD_DWORD            : FLAT_Real_Base_ppu<0x00c>;
defm FLAT_LOAD_DWORDX2          : FLAT_Real_Base_ppu<0x00d>;
defm FLAT_LOAD_DWORDX4          : FLAT_Real_Base_ppu<0x00e>;
defm FLAT_LOAD_DWORDX3          : FLAT_Real_Base_ppu<0x00f>;
defm FLAT_STORE_BYTE            : FLAT_Real_Base_ppu<0x018>;
defm FLAT_STORE_BYTE_D16_HI     : FLAT_Real_Base_ppu<0x019>;
defm FLAT_STORE_SHORT           : FLAT_Real_Base_ppu<0x01a>;
defm FLAT_STORE_SHORT_D16_HI    : FLAT_Real_Base_ppu<0x01b>;
defm FLAT_STORE_DWORD           : FLAT_Real_Base_ppu<0x01c>;
defm FLAT_STORE_DWORDX2         : FLAT_Real_Base_ppu<0x01d>;
defm FLAT_STORE_DWORDX4         : FLAT_Real_Base_ppu<0x01e>;
defm FLAT_STORE_DWORDX3         : FLAT_Real_Base_ppu<0x01f>;
defm FLAT_LOAD_UBYTE_D16        : FLAT_Real_Base_ppu<0x020>;
defm FLAT_LOAD_UBYTE_D16_HI     : FLAT_Real_Base_ppu<0x021>;
defm FLAT_LOAD_SBYTE_D16        : FLAT_Real_Base_ppu<0x022>;
defm FLAT_LOAD_SBYTE_D16_HI     : FLAT_Real_Base_ppu<0x023>;
defm FLAT_LOAD_SHORT_D16        : FLAT_Real_Base_ppu<0x024>;
defm FLAT_LOAD_SHORT_D16_HI     : FLAT_Real_Base_ppu<0x025>;
defm FLAT_ATOMIC_SWAP           : FLAT_Real_Atomics_ppu<0x030>;
defm FLAT_ATOMIC_CMPSWAP        : FLAT_Real_Atomics_ppu<0x031>;
defm FLAT_ATOMIC_ADD            : FLAT_Real_Atomics_ppu<0x032>;
defm FLAT_ATOMIC_SUB            : FLAT_Real_Atomics_ppu<0x033>;
defm FLAT_ATOMIC_SMIN           : FLAT_Real_Atomics_ppu<0x035>;
defm FLAT_ATOMIC_UMIN           : FLAT_Real_Atomics_ppu<0x036>;
defm FLAT_ATOMIC_SMAX           : FLAT_Real_Atomics_ppu<0x037>;
defm FLAT_ATOMIC_UMAX           : FLAT_Real_Atomics_ppu<0x038>;
defm FLAT_ATOMIC_AND            : FLAT_Real_Atomics_ppu<0x039>;
defm FLAT_ATOMIC_OR             : FLAT_Real_Atomics_ppu<0x03a>;
defm FLAT_ATOMIC_XOR            : FLAT_Real_Atomics_ppu<0x03b>;
defm FLAT_ATOMIC_INC            : FLAT_Real_Atomics_ppu<0x03c>;
defm FLAT_ATOMIC_DEC            : FLAT_Real_Atomics_ppu<0x03d>;
defm FLAT_ATOMIC_FCMPSWAP       : FLAT_Real_Atomics_ppu<0x03e>;
defm FLAT_ATOMIC_FMIN           : FLAT_Real_Atomics_ppu<0x03f>;
defm FLAT_ATOMIC_FMAX           : FLAT_Real_Atomics_ppu<0x040>;
defm FLAT_ATOMIC_SWAP_X2        : FLAT_Real_Atomics_ppu<0x050>;
defm FLAT_ATOMIC_CMPSWAP_X2     : FLAT_Real_Atomics_ppu<0x051>;
defm FLAT_ATOMIC_ADD_X2         : FLAT_Real_Atomics_ppu<0x052>;
defm FLAT_ATOMIC_SUB_X2         : FLAT_Real_Atomics_ppu<0x053>;
defm FLAT_ATOMIC_SMIN_X2        : FLAT_Real_Atomics_ppu<0x055>;
defm FLAT_ATOMIC_UMIN_X2        : FLAT_Real_Atomics_ppu<0x056>;
defm FLAT_ATOMIC_SMAX_X2        : FLAT_Real_Atomics_ppu<0x057>;
defm FLAT_ATOMIC_UMAX_X2        : FLAT_Real_Atomics_ppu<0x058>;
defm FLAT_ATOMIC_AND_X2         : FLAT_Real_Atomics_ppu<0x059>;
defm FLAT_ATOMIC_OR_X2          : FLAT_Real_Atomics_ppu<0x05a>;
defm FLAT_ATOMIC_XOR_X2         : FLAT_Real_Atomics_ppu<0x05b>;
defm FLAT_ATOMIC_INC_X2         : FLAT_Real_Atomics_ppu<0x05c>;
defm FLAT_ATOMIC_DEC_X2         : FLAT_Real_Atomics_ppu<0x05d>;
defm FLAT_ATOMIC_FCMPSWAP_X2    : FLAT_Real_Atomics_ppu<0x05e>;
defm FLAT_ATOMIC_FMIN_X2        : FLAT_Real_Atomics_ppu<0x05f>;
defm FLAT_ATOMIC_FMAX_X2        : FLAT_Real_Atomics_ppu<0x060>;


// ENC_FLAT_GLBL.
defm GLOBAL_LOAD_UBYTE          : FLAT_Real_AllAddr_ppu<0x008>;
defm GLOBAL_LOAD_SBYTE          : FLAT_Real_AllAddr_ppu<0x009>;
defm GLOBAL_LOAD_USHORT         : FLAT_Real_AllAddr_ppu<0x00a>;
defm GLOBAL_LOAD_SSHORT         : FLAT_Real_AllAddr_ppu<0x00b>;
defm GLOBAL_LOAD_DWORD          : FLAT_Real_AllAddr_ppu<0x00c>;
defm GLOBAL_LOAD_DWORDX2        : FLAT_Real_AllAddr_ppu<0x00d>;
defm GLOBAL_LOAD_DWORDX4        : FLAT_Real_AllAddr_ppu<0x00e>;
defm GLOBAL_LOAD_DWORDX3        : FLAT_Real_AllAddr_ppu<0x00f>;
defm GLOBAL_STORE_BYTE          : FLAT_Real_AllAddr_ppu<0x018>;
defm GLOBAL_STORE_BYTE_D16_HI   : FLAT_Real_AllAddr_ppu<0x019>;
defm GLOBAL_STORE_SHORT         : FLAT_Real_AllAddr_ppu<0x01a>;
defm GLOBAL_STORE_SHORT_D16_HI  : FLAT_Real_AllAddr_ppu<0x01b>;
defm GLOBAL_STORE_DWORD         : FLAT_Real_AllAddr_ppu<0x01c>;
defm GLOBAL_STORE_DWORDX2       : FLAT_Real_AllAddr_ppu<0x01d>;
defm GLOBAL_STORE_DWORDX4       : FLAT_Real_AllAddr_ppu<0x01e>;
defm GLOBAL_STORE_DWORDX3       : FLAT_Real_AllAddr_ppu<0x01f>;
defm GLOBAL_LOAD_UBYTE_D16      : FLAT_Real_AllAddr_ppu<0x020>;
defm GLOBAL_LOAD_UBYTE_D16_HI   : FLAT_Real_AllAddr_ppu<0x021>;
defm GLOBAL_LOAD_SBYTE_D16      : FLAT_Real_AllAddr_ppu<0x022>;
defm GLOBAL_LOAD_SBYTE_D16_HI   : FLAT_Real_AllAddr_ppu<0x023>;
defm GLOBAL_LOAD_SHORT_D16      : FLAT_Real_AllAddr_ppu<0x024>;
defm GLOBAL_LOAD_SHORT_D16_HI   : FLAT_Real_AllAddr_ppu<0x025>;
defm GLOBAL_ATOMIC_SWAP         : FLAT_Real_GlblAtomics_ppu<0x030>;
defm GLOBAL_ATOMIC_CMPSWAP      : FLAT_Real_GlblAtomics_ppu<0x031>;
defm GLOBAL_ATOMIC_ADD          : FLAT_Real_GlblAtomics_ppu<0x032>;
defm GLOBAL_ATOMIC_SUB          : FLAT_Real_GlblAtomics_ppu<0x033>;
defm GLOBAL_ATOMIC_SMIN         : FLAT_Real_GlblAtomics_ppu<0x035>;
defm GLOBAL_ATOMIC_UMIN         : FLAT_Real_GlblAtomics_ppu<0x036>;
defm GLOBAL_ATOMIC_SMAX         : FLAT_Real_GlblAtomics_ppu<0x037>;
defm GLOBAL_ATOMIC_UMAX         : FLAT_Real_GlblAtomics_ppu<0x038>;
defm GLOBAL_ATOMIC_AND          : FLAT_Real_GlblAtomics_ppu<0x039>;
defm GLOBAL_ATOMIC_OR           : FLAT_Real_GlblAtomics_ppu<0x03a>;
defm GLOBAL_ATOMIC_XOR          : FLAT_Real_GlblAtomics_ppu<0x03b>;
defm GLOBAL_ATOMIC_INC          : FLAT_Real_GlblAtomics_ppu<0x03c>;
defm GLOBAL_ATOMIC_DEC          : FLAT_Real_GlblAtomics_ppu<0x03d>;
defm GLOBAL_ATOMIC_FCMPSWAP     : FLAT_Real_GlblAtomics_ppu<0x03e>;
defm GLOBAL_ATOMIC_FMIN         : FLAT_Real_GlblAtomics_ppu<0x03f>;
defm GLOBAL_ATOMIC_FMAX         : FLAT_Real_GlblAtomics_ppu<0x040>;
defm GLOBAL_ATOMIC_SWAP_X2      : FLAT_Real_GlblAtomics_ppu<0x050>;
defm GLOBAL_ATOMIC_CMPSWAP_X2   : FLAT_Real_GlblAtomics_ppu<0x051>;
defm GLOBAL_ATOMIC_ADD_X2       : FLAT_Real_GlblAtomics_ppu<0x052>;
defm GLOBAL_ATOMIC_SUB_X2       : FLAT_Real_GlblAtomics_ppu<0x053>;
defm GLOBAL_ATOMIC_SMIN_X2      : FLAT_Real_GlblAtomics_ppu<0x055>;
defm GLOBAL_ATOMIC_UMIN_X2      : FLAT_Real_GlblAtomics_ppu<0x056>;
defm GLOBAL_ATOMIC_SMAX_X2      : FLAT_Real_GlblAtomics_ppu<0x057>;
defm GLOBAL_ATOMIC_UMAX_X2      : FLAT_Real_GlblAtomics_ppu<0x058>;
defm GLOBAL_ATOMIC_AND_X2       : FLAT_Real_GlblAtomics_ppu<0x059>;
defm GLOBAL_ATOMIC_OR_X2        : FLAT_Real_GlblAtomics_ppu<0x05a>;
defm GLOBAL_ATOMIC_XOR_X2       : FLAT_Real_GlblAtomics_ppu<0x05b>;
defm GLOBAL_ATOMIC_INC_X2       : FLAT_Real_GlblAtomics_ppu<0x05c>;
defm GLOBAL_ATOMIC_DEC_X2       : FLAT_Real_GlblAtomics_ppu<0x05d>;
defm GLOBAL_ATOMIC_FCMPSWAP_X2  : FLAT_Real_GlblAtomics_ppu<0x05e>;
defm GLOBAL_ATOMIC_FMIN_X2      : FLAT_Real_GlblAtomics_ppu<0x05f>;
defm GLOBAL_ATOMIC_FMAX_X2      : FLAT_Real_GlblAtomics_ppu<0x060>;


// ENC_FLAT_SCRATCH.
defm SCRATCH_LOAD_UBYTE         : FLAT_Real_AllAddr_ppu<0x008>;
defm SCRATCH_LOAD_SBYTE         : FLAT_Real_AllAddr_ppu<0x009>;
defm SCRATCH_LOAD_USHORT        : FLAT_Real_AllAddr_ppu<0x00a>;
defm SCRATCH_LOAD_SSHORT        : FLAT_Real_AllAddr_ppu<0x00b>;
defm SCRATCH_LOAD_DWORD         : FLAT_Real_AllAddr_ppu<0x00c>;
defm SCRATCH_LOAD_DWORDX2       : FLAT_Real_AllAddr_ppu<0x00d>;
defm SCRATCH_LOAD_DWORDX4       : FLAT_Real_AllAddr_ppu<0x00e>;
defm SCRATCH_LOAD_DWORDX3       : FLAT_Real_AllAddr_ppu<0x00f>;
defm SCRATCH_STORE_BYTE         : FLAT_Real_AllAddr_ppu<0x018>;
defm SCRATCH_STORE_BYTE_D16_HI  : FLAT_Real_AllAddr_ppu<0x019>;
defm SCRATCH_STORE_SHORT        : FLAT_Real_AllAddr_ppu<0x01a>;
defm SCRATCH_STORE_SHORT_D16_HI : FLAT_Real_AllAddr_ppu<0x01b>;
defm SCRATCH_STORE_DWORD        : FLAT_Real_AllAddr_ppu<0x01c>;
defm SCRATCH_STORE_DWORDX2      : FLAT_Real_AllAddr_ppu<0x01d>;
defm SCRATCH_STORE_DWORDX4      : FLAT_Real_AllAddr_ppu<0x01e>;
defm SCRATCH_STORE_DWORDX3      : FLAT_Real_AllAddr_ppu<0x01f>;
defm SCRATCH_LOAD_UBYTE_D16     : FLAT_Real_AllAddr_ppu<0x020>;
defm SCRATCH_LOAD_UBYTE_D16_HI  : FLAT_Real_AllAddr_ppu<0x021>;
defm SCRATCH_LOAD_SBYTE_D16     : FLAT_Real_AllAddr_ppu<0x022>;
defm SCRATCH_LOAD_SBYTE_D16_HI  : FLAT_Real_AllAddr_ppu<0x023>;
defm SCRATCH_LOAD_SHORT_D16     : FLAT_Real_AllAddr_ppu<0x024>;
defm SCRATCH_LOAD_SHORT_D16_HI  : FLAT_Real_AllAddr_ppu<0x025>;

let SubtargetPredicate = HasAtomicFaddInsts in {

defm GLOBAL_ATOMIC_ADD_F32    : FLAT_Real_AllAddr_vi <0x04d>;
defm GLOBAL_ATOMIC_PK_ADD_F16 : FLAT_Real_AllAddr_vi <0x04e>;

} // End SubtargetPredicate = HasAtomicFaddInsts
