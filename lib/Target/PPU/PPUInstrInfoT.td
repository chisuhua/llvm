// below is base on SIInstrInfo.td
//    SIIntrInfo.td include the SIIntructions.td/DSInstruction.td/MIMGInstructions.td
//    SIIntructions.td include SOPInstrutions.td/VOPInstructions.td/SMInstructions.td/FLAT.../BUF

def isWave32 : Predicate <"true">;
def DisableInst : Predicate <"false">, AssemblerPredicate<"FeatureDisable">;

// Execpt for the NONE field, this must be kept in sync with the
// PPUEncodingFamily enum in AMDGPUInstrInfo.cpp
def PPUEncodingFamily {
  int NONE = -1;
  int PPU = 0;
}


//===----------------------------------------------------------------------===//
// SI DAG Nodes
//===----------------------------------------------------------------------===//

// def AMDGPUclamp : SDNode<"PPUISD::CLAMP", SDTFPUnaryOp>;

// TODO i change v4i32 to v2i32
def SIsbuffer_load : SDNode<"PPUISD::SBUFFER_LOAD",
  SDTypeProfile<1, 4, [SDTCisVT<1, v2i32>, SDTCisVT<2, i32>, SDTCisVT<3, i1>,
                       SDTCisVT<4, i1>]>,
  [SDNPMayLoad, SDNPMemOperand]
>;
def SIds_ordered_count : SDNode<"PPUISD::DS_ORDERED_COUNT",
  SDTypeProfile<1, 2, [SDTCisVT<0, i32>, SDTCisVT<1, i32>, SDTCisVT<2, i16>]>,
  [SDNPMayLoad, SDNPMayStore, SDNPMemOperand, SDNPHasChain, SDNPInGlue]
>;

def PPUatomic_inc : SDNode<"PPUISD::ATOMIC_INC", SDTAtomic2,
  [SDNPMayLoad, SDNPMayStore, SDNPMemOperand, SDNPHasChain]
>;

def PPUatomic_dec : SDNode<"PPUISD::ATOMIC_DEC", SDTAtomic2,
  [SDNPMayLoad, SDNPMayStore, SDNPMemOperand, SDNPHasChain]
>;

/*
def SDTAtomic2_f32 : SDTypeProfile<1, 2, [
  SDTCisSameAs<0,2>, SDTCisFP<0>, SDTCisPtrTy<1>
]>;

def SIatomic_fmin : SDNode<"PPUISD::ATOMIC_LOAD_FMIN", SDTAtomic2_f32,
  [SDNPMayLoad, SDNPMayStore, SDNPMemOperand, SDNPHasChain]
>;

def SIatomic_fmax : SDNode<"PPUISD::ATOMIC_LOAD_FMAX", SDTAtomic2_f32,
  [SDNPMayLoad, SDNPMayStore, SDNPMemOperand, SDNPHasChain]
>;
*/
// load_d16_{lo|hi} ptr, tied_input
def SIload_d16 : SDTypeProfile<1, 2, [
  SDTCisPtrTy<1>,
  SDTCisSameAs<0, 2>
]>;

// TODO i change rsrc v4i32 to v2i32 in below
def SDTtbuffer_load : SDTypeProfile<1, 8,
  [                     // vdata
   SDTCisVT<1, v2i32>,  // rsrc
   SDTCisVT<2, i32>,    // vindex(VGPR)
   SDTCisVT<3, i32>,    // voffset(VGPR)
   SDTCisVT<4, i32>,    // soffset(SGPR)
   SDTCisVT<5, i32>,    // offset(imm)
   SDTCisVT<6, i32>,    // format(imm)
   SDTCisVT<7, i32>,    // cachecontrol(imm)
   SDTCisVT<8, i1>      // idxen(imm)
  ]>;

def SItbuffer_load :   SDNode<"PPUISD::TBUFFER_LOAD_FORMAT", SDTtbuffer_load,
                              [SDNPMayLoad, SDNPMemOperand, SDNPHasChain]>;
def SItbuffer_load_d16 : SDNode<"PPUISD::TBUFFER_LOAD_FORMAT_D16",
                                SDTtbuffer_load,
                                [SDNPMayLoad, SDNPMemOperand, SDNPHasChain]>;

def SDTtbuffer_store : SDTypeProfile<0, 9,
    [                     // vdata
     SDTCisVT<1, v2i32>,  // rsrc
     SDTCisVT<2, i32>,    // vindex(VGPR)
     SDTCisVT<3, i32>,    // voffset(VGPR)
     SDTCisVT<4, i32>,    // soffset(SGPR)
     SDTCisVT<5, i32>,    // offset(imm)
     SDTCisVT<6, i32>,    // format(imm)
     SDTCisVT<7, i32>,    // cachecontrol(imm)
     SDTCisVT<8, i1>      // idxen(imm)
    ]>;

def SItbuffer_store : SDNode<"PPUISD::TBUFFER_STORE_FORMAT", SDTtbuffer_store,
                             [SDNPMayStore, SDNPMemOperand, SDNPHasChain]>;
def SItbuffer_store_d16 : SDNode<"PPUISD::TBUFFER_STORE_FORMAT_D16",
                                SDTtbuffer_store,
                                [SDNPMayStore, SDNPMemOperand, SDNPHasChain]>;

def SDTBufferLoad : SDTypeProfile<1, 7,
    [                    // vdata
     SDTCisVT<1, v2i32>, // rsrc
     SDTCisVT<2, i32>,   // vindex(VGPR)
     SDTCisVT<3, i32>,   // voffset(VGPR)
     SDTCisVT<4, i32>,   // soffset(SGPR)
     SDTCisVT<5, i32>,   // offset(imm)
     SDTCisVT<6, i32>,   // cachepolicy(imm)
     SDTCisVT<7, i1>]>;  // idxen(imm)

def SIbuffer_load : SDNode <"PPUISD::BUFFER_LOAD", SDTBufferLoad,
                            [SDNPMemOperand, SDNPHasChain, SDNPMayLoad]>;
def SIbuffer_load_ubyte : SDNode <"PPUISD::BUFFER_LOAD_UBYTE", SDTBufferLoad,
                            [SDNPMemOperand, SDNPHasChain, SDNPMayLoad]>;
def SIbuffer_load_ushort : SDNode <"PPUISD::BUFFER_LOAD_USHORT", SDTBufferLoad,
                            [SDNPMemOperand, SDNPHasChain, SDNPMayLoad]>;
def SIbuffer_load_byte : SDNode <"PPUISD::BUFFER_LOAD_BYTE", SDTBufferLoad,
                            [SDNPMemOperand, SDNPHasChain, SDNPMayLoad]>;
def SIbuffer_load_short: SDNode <"PPUISD::BUFFER_LOAD_SHORT", SDTBufferLoad,
                            [SDNPMemOperand, SDNPHasChain, SDNPMayLoad]>;
def SIbuffer_load_format : SDNode <"PPUISD::BUFFER_LOAD_FORMAT", SDTBufferLoad,
                            [SDNPMemOperand, SDNPHasChain, SDNPMayLoad]>;
def SIbuffer_load_format_d16 : SDNode <"PPUISD::BUFFER_LOAD_FORMAT_D16",
                                SDTBufferLoad,
                                [SDNPMemOperand, SDNPHasChain, SDNPMayLoad]>;

def SDTBufferStore : SDTypeProfile<0, 8,
    [                    // vdata
     SDTCisVT<1, v2i32>, // rsrc
     SDTCisVT<2, i32>,   // vindex(VGPR)
     SDTCisVT<3, i32>,   // voffset(VGPR)
     SDTCisVT<4, i32>,   // soffset(SGPR)
     SDTCisVT<5, i32>,   // offset(imm)
     SDTCisVT<6, i32>,   // cachepolicy(imm)
     SDTCisVT<7, i1>]>;  // idxen(imm)

def SIbuffer_store : SDNode <"PPUISD::BUFFER_STORE", SDTBufferStore,
                             [SDNPMayStore, SDNPMemOperand, SDNPHasChain]>;
def SIbuffer_store_byte: SDNode <"PPUISD::BUFFER_STORE_BYTE",
                         SDTBufferStore,
                         [SDNPMayStore, SDNPMemOperand, SDNPHasChain]>;
def SIbuffer_store_short : SDNode <"PPUISD::BUFFER_STORE_SHORT",
                           SDTBufferStore,
                           [SDNPMayStore, SDNPMemOperand, SDNPHasChain]>;
def SIbuffer_store_format : SDNode <"PPUISD::BUFFER_STORE_FORMAT",
                            SDTBufferStore,
                            [SDNPMayStore, SDNPMemOperand, SDNPHasChain]>;
def SIbuffer_store_format_d16 : SDNode <"PPUISD::BUFFER_STORE_FORMAT_D16",
                            SDTBufferStore,
                            [SDNPMayStore, SDNPMemOperand, SDNPHasChain]>;
/*
class SDBufferAtomic<string opcode> : SDNode <opcode,
  SDTypeProfile<1, 8,
       [SDTCisVT<2, v4i32>, // rsrc
       SDTCisVT<3, i32>,   // vindex(VGPR)
       SDTCisVT<4, i32>,   // voffset(VGPR)
       SDTCisVT<5, i32>,   // soffset(SGPR)
       SDTCisVT<6, i32>,   // offset(imm)
       SDTCisVT<7, i32>,   // cachepolicy(imm)
       SDTCisVT<8, i1>]>,  // idxen(imm)
  [SDNPMemOperand, SDNPHasChain, SDNPMayLoad, SDNPMayStore]
>;

class SDBufferAtomicNoRtn<string opcode, ValueType ty> : SDNode <opcode,
  SDTypeProfile<0, 8,
      [SDTCisVT<0, ty>,    // vdata
       SDTCisVT<1, v4i32>, // rsrc
       SDTCisVT<2, i32>,   // vindex(VGPR)
       SDTCisVT<3, i32>,   // voffset(VGPR)
       SDTCisVT<4, i32>,   // soffset(SGPR)
       SDTCisVT<5, i32>,   // offset(imm)
       SDTCisVT<6, i32>,   // cachepolicy(imm)
       SDTCisVT<7, i1>]>,  // idxen(imm)
  [SDNPMemOperand, SDNPHasChain, SDNPMayLoad, SDNPMayStore]
>;

def SIbuffer_atomic_swap : SDBufferAtomic <"PPUISD::BUFFER_ATOMIC_SWAP">;
def SIbuffer_atomic_add : SDBufferAtomic <"PPUISD::BUFFER_ATOMIC_ADD">;
def SIbuffer_atomic_sub : SDBufferAtomic <"PPUISD::BUFFER_ATOMIC_SUB">;
def SIbuffer_atomic_smin : SDBufferAtomic <"PPUISD::BUFFER_ATOMIC_SMIN">;
def SIbuffer_atomic_umin : SDBufferAtomic <"PPUISD::BUFFER_ATOMIC_UMIN">;
def SIbuffer_atomic_smax : SDBufferAtomic <"PPUISD::BUFFER_ATOMIC_SMAX">;
def SIbuffer_atomic_umax : SDBufferAtomic <"PPUISD::BUFFER_ATOMIC_UMAX">;
def SIbuffer_atomic_and : SDBufferAtomic <"PPUISD::BUFFER_ATOMIC_AND">;
def SIbuffer_atomic_or : SDBufferAtomic <"PPUISD::BUFFER_ATOMIC_OR">;
def SIbuffer_atomic_xor : SDBufferAtomic <"PPUISD::BUFFER_ATOMIC_XOR">;
def SIbuffer_atomic_inc : SDBufferAtomic <"PPUISD::BUFFER_ATOMIC_INC">;
def SIbuffer_atomic_dec : SDBufferAtomic <"PPUISD::BUFFER_ATOMIC_DEC">;
def SIbuffer_atomic_fadd : SDBufferAtomicNoRtn <"PPUISD::BUFFER_ATOMIC_FADD", f32>;
def SIbuffer_atomic_pk_fadd : SDBufferAtomicNoRtn <"PPUISD::BUFFER_ATOMIC_PK_FADD", v2f16>;

def SIbuffer_atomic_cmpswap : SDNode <"PPUISD::BUFFER_ATOMIC_CMPSWAP",
  SDTypeProfile<1, 9,
    [SDTCisVT<0, i32>,   // dst
     SDTCisVT<1, i32>,   // src
     SDTCisVT<2, i32>,   // cmp
     SDTCisVT<3, v4i32>, // rsrc
     SDTCisVT<4, i32>,   // vindex(VGPR)
     SDTCisVT<5, i32>,   // voffset(VGPR)
     SDTCisVT<6, i32>,   // soffset(SGPR)
     SDTCisVT<7, i32>,   // offset(imm)
     SDTCisVT<8, i32>,   // cachepolicy(imm)
     SDTCisVT<9, i1>]>,  // idxen(imm)
  [SDNPMemOperand, SDNPHasChain, SDNPMayLoad, SDNPMayStore]
>;

class SDGlobalAtomicNoRtn<string opcode, ValueType ty> : SDNode <opcode,
  SDTypeProfile<0, 2,
      [SDTCisPtrTy<0>,     // vaddr
       SDTCisVT<1, ty>]>,  // vdata
  [SDNPMemOperand, SDNPHasChain, SDNPMayLoad, SDNPMayStore]
>;

def SIglobal_atomic_fadd    : SDGlobalAtomicNoRtn <"PPUISD::ATOMIC_FADD", f32>;
def SIglobal_atomic_pk_fadd : SDGlobalAtomicNoRtn <"PPUISD::ATOMIC_PK_FADD", v2f16>;
*/

def SIpc_add_rel_offset : SDNode<"PPUISD::PC_ADD_REL_OFFSET",
  SDTypeProfile<1, 2, [SDTCisVT<0, iPTR>, SDTCisSameAs<0,1>, SDTCisSameAs<0,2>]>
>;

def SIlds : SDNode<"PPUISD::LDS",
  SDTypeProfile<1, 1, [SDTCisVT<0, iPTR>, SDTCisSameAs<0,1>]>
>;

def SIload_d16_lo : SDNode<"PPUISD::LOAD_D16_LO",
  SIload_d16,
  [SDNPMayLoad, SDNPMemOperand, SDNPHasChain]
>;

def SIload_d16_lo_u8 : SDNode<"PPUISD::LOAD_D16_LO_U8",
  SIload_d16,
  [SDNPMayLoad, SDNPMemOperand, SDNPHasChain]
>;

def SIload_d16_lo_i8 : SDNode<"PPUISD::LOAD_D16_LO_I8",
  SIload_d16,
  [SDNPMayLoad, SDNPMemOperand, SDNPHasChain]
>;

def SIload_d16_hi : SDNode<"PPUISD::LOAD_D16_HI",
  SIload_d16,
  [SDNPMayLoad, SDNPMemOperand, SDNPHasChain]
>;

def SIload_d16_hi_u8 : SDNode<"PPUISD::LOAD_D16_HI_U8",
  SIload_d16,
  [SDNPMayLoad, SDNPMemOperand, SDNPHasChain]
>;

def SIload_d16_hi_i8 : SDNode<"PPUISD::LOAD_D16_HI_I8",
  SIload_d16,
  [SDNPMayLoad, SDNPMemOperand, SDNPHasChain]
>;

def SIdenorm_mode : SDNode<"PPUISD::DENORM_MODE",
  SDTypeProfile<0 ,1, [SDTCisInt<0>]>,
  [SDNPHasChain, SDNPSideEffect, SDNPOptInGlue, SDNPOutGlue]
>;


//===----------------------------------------------------------------------===//
// ValueType helpers
//===----------------------------------------------------------------------===//

// Returns 1 if the source arguments have modifiers, 0 if they do not.
// XXX - do f16 instructions?
class isFloatType<ValueType SrcVT> {
  bit ret =
    !if(!eq(SrcVT.Value, f16.Value), 1,
    !if(!eq(SrcVT.Value, f32.Value), 1,
    !if(!eq(SrcVT.Value, f64.Value), 1,
    !if(!eq(SrcVT.Value, v2f16.Value), 1,
    !if(!eq(SrcVT.Value, v4f16.Value), 1,
    0)))));
}

class isIntType<ValueType SrcVT> {
  bit ret =
    !if(!eq(SrcVT.Value, i16.Value), 1,
    !if(!eq(SrcVT.Value, i32.Value), 1,
    !if(!eq(SrcVT.Value, i64.Value), 1,
    0)));
}

class isPackedType<ValueType SrcVT> {
  bit ret =
    !if(!eq(SrcVT.Value, v2i16.Value), 1,
      !if(!eq(SrcVT.Value, v2f16.Value), 1,
        !if(!eq(SrcVT.Value, v4f16.Value), 1, 0)
    ));
}

//===----------------------------------------------------------------------===//
// PatFrags for global memory operations
//===----------------------------------------------------------------------===//

foreach as = [ "global", "flat", "constant", "local", "private", "region" ] in {
let AddressSpaces = !cast<AddressSpaceList>("LoadAddress_"#as).AddrSpaces in {


defm atomic_inc_#as : binary_atomic_op<PPUatomic_inc>;
defm atomic_dec_#as : binary_atomic_op<PPUatomic_dec>;
/*
defm atomic_load_fmin_#as : binary_atomic_op<SIatomic_fmin, 0>;
defm atomic_load_fmax_#as : binary_atomic_op<SIatomic_fmax, 0>;
*/

} // End let AddressSpaces = ...
} // End foreach AddrSpace


//===----------------------------------------------------------------------===//
// SDNodes PatFrags for loads/stores with a glue input.
// This is for SDNodes and PatFrag for local loads and stores to
// enable s_mov_b32 m0, -1 to be glued to the memory instructions.
//
// These mirror the regular load/store PatFrags and rely on special
// processing during Select() to add the glued copy.
//
//===----------------------------------------------------------------------===//

def si_setcc_uniform : PatFrag <
  (ops node:$lhs, node:$rhs, node:$cond),
  (setcc node:$lhs, node:$rhs, node:$cond), [{
  for (SDNode *Use : N->uses()) {
    if (Use->isMachineOpcode() || Use->getOpcode() != ISD::CopyToReg)
      return false;

    unsigned Reg = cast<RegisterSDNode>(Use->getOperand(1))->getReg();
    if (Reg != AMDGPU::SCC)
      return false;
  }
  return true;
}]>;

//===----------------------------------------------------------------------===//
// SDNodes PatFrags for d16 loads
//===----------------------------------------------------------------------===//

class LoadD16Frag <SDPatternOperator op> : PatFrag<(ops node:$ptr, node:$tied_in), (op node:$ptr, node:$tied_in)>;
class LocalLoadD16 <SDPatternOperator op> : LoadD16Frag <op>, LocalAddress;
class GlobalLoadD16 <SDPatternOperator op> : LoadD16Frag <op>, GlobalLoadAddress;
class PrivateLoadD16 <SDPatternOperator op> : LoadD16Frag <op>, PrivateAddress;
class FlatLoadD16 <SDPatternOperator op> : LoadD16Frag <op>, FlatLoadAddress;

def load_d16_hi_local : LocalLoadD16 <SIload_d16_hi>;
def az_extloadi8_d16_hi_local : LocalLoadD16 <SIload_d16_hi_u8>;
def sextloadi8_d16_hi_local : LocalLoadD16 <SIload_d16_hi_i8>;

def load_d16_hi_global : GlobalLoadD16 <SIload_d16_hi>;
def az_extloadi8_d16_hi_global : GlobalLoadD16 <SIload_d16_hi_u8>;
def sextloadi8_d16_hi_global : GlobalLoadD16 <SIload_d16_hi_i8>;

def load_d16_hi_private : PrivateLoadD16 <SIload_d16_hi>;
def az_extloadi8_d16_hi_private : PrivateLoadD16 <SIload_d16_hi_u8>;
def sextloadi8_d16_hi_private : PrivateLoadD16 <SIload_d16_hi_i8>;

def load_d16_hi_flat : FlatLoadD16 <SIload_d16_hi>;
def az_extloadi8_d16_hi_flat : FlatLoadD16 <SIload_d16_hi_u8>;
def sextloadi8_d16_hi_flat : FlatLoadD16 <SIload_d16_hi_i8>;

def load_d16_lo_local : LocalLoadD16 <SIload_d16_lo>;
def az_extloadi8_d16_lo_local : LocalLoadD16 <SIload_d16_lo_u8>;
def sextloadi8_d16_lo_local : LocalLoadD16 <SIload_d16_lo_i8>;

def load_d16_lo_global : GlobalLoadD16 <SIload_d16_lo>;
def az_extloadi8_d16_lo_global : GlobalLoadD16 <SIload_d16_lo_u8>;
def sextloadi8_d16_lo_global : GlobalLoadD16 <SIload_d16_lo_i8>;

def load_d16_lo_private : PrivateLoadD16 <SIload_d16_lo>;
def az_extloadi8_d16_lo_private : PrivateLoadD16 <SIload_d16_lo_u8>;
def sextloadi8_d16_lo_private : PrivateLoadD16 <SIload_d16_lo_i8>;

def load_d16_lo_flat : FlatLoadD16 <SIload_d16_lo>;
def az_extloadi8_d16_lo_flat : FlatLoadD16 <SIload_d16_lo_u8>;
def sextloadi8_d16_lo_flat : FlatLoadD16 <SIload_d16_lo_i8>;


def lshr_rev : PatFrag <
  (ops node:$src1, node:$src0),
  (srl $src0, $src1)
>;

def ashr_rev : PatFrag <
  (ops node:$src1, node:$src0),
  (sra $src0, $src1)
>;

def lshl_rev : PatFrag <
  (ops node:$src1, node:$src0),
  (shl $src0, $src1)
>;
/*
multiclass SIAtomicM0Glue2 <string op_name, bit is_amdgpu = 0,
                            SDTypeProfile tc = SDTAtomic2,
                            bit IsInt = 1> {

  def _glue : SDNode <
    !if(is_amdgpu, "PPUISD", "ISD")#"::ATOMIC_"#op_name, tc,
    [SDNPHasChain, SDNPMayStore, SDNPMayLoad, SDNPMemOperand, SDNPInGlue]
  >;

  let AddressSpaces = StoreAddress_local.AddrSpaces in {
    defm _local_m0 : binary_atomic_op <!cast<SDNode>(NAME#"_glue"), IsInt>;
  }

  let AddressSpaces = StoreAddress_region.AddrSpaces in {
    defm _region_m0 : binary_atomic_op <!cast<SDNode>(NAME#"_glue"), IsInt>;
  }
}

defm atomic_load_add : SIAtomicM0Glue2 <"LOAD_ADD">;
defm atomic_load_sub : SIAtomicM0Glue2 <"LOAD_SUB">;
defm atomic_inc : SIAtomicM0Glue2 <"INC", 1>;
defm atomic_dec : SIAtomicM0Glue2 <"DEC", 1>;
defm atomic_load_and : SIAtomicM0Glue2 <"LOAD_AND">;
defm atomic_load_min : SIAtomicM0Glue2 <"LOAD_MIN">;
defm atomic_load_max : SIAtomicM0Glue2 <"LOAD_MAX">;
defm atomic_load_or : SIAtomicM0Glue2 <"LOAD_OR">;
defm atomic_load_xor : SIAtomicM0Glue2 <"LOAD_XOR">;
defm atomic_load_umin : SIAtomicM0Glue2 <"LOAD_UMIN">;
defm atomic_load_umax : SIAtomicM0Glue2 <"LOAD_UMAX">;
defm atomic_swap : SIAtomicM0Glue2 <"SWAP">;
defm atomic_load_fadd : SIAtomicM0Glue2 <"LOAD_FADD", 0, SDTAtomic2_f32, 0>;
defm atomic_load_fmin : SIAtomicM0Glue2 <"LOAD_FMIN", 1, SDTAtomic2_f32, 0>;
defm atomic_load_fmax : SIAtomicM0Glue2 <"LOAD_FMAX", 1, SDTAtomic2_f32, 0>;

*/
def as_i1imm : SDNodeXForm<imm, [{
  return CurDAG->getTargetConstant(N->getZExtValue(), SDLoc(N), MVT::i1);
}]>;

def as_i8imm : SDNodeXForm<imm, [{
  return CurDAG->getTargetConstant(N->getZExtValue(), SDLoc(N), MVT::i8);
}]>;

def as_i16imm : SDNodeXForm<imm, [{
  return CurDAG->getTargetConstant(N->getSExtValue(), SDLoc(N), MVT::i16);
}]>;

def as_i32imm: SDNodeXForm<imm, [{
  return CurDAG->getTargetConstant(N->getSExtValue(), SDLoc(N), MVT::i32);
}]>;

def as_i64imm: SDNodeXForm<imm, [{
  return CurDAG->getTargetConstant(N->getSExtValue(), SDLoc(N), MVT::i64);
}]>;

def cond_as_i32imm: SDNodeXForm<cond, [{
  return CurDAG->getTargetConstant(N->get(), SDLoc(N), MVT::i32);
}]>;

// Copied from the AArch64 backend:
def bitcast_fpimm_to_i32 : SDNodeXForm<fpimm, [{
return CurDAG->getTargetConstant(
  N->getValueAPF().bitcastToAPInt().getZExtValue(), SDLoc(N), MVT::i32);
}]>;

def frameindex_to_targetframeindex : SDNodeXForm<frameindex, [{
  auto FI = cast<FrameIndexSDNode>(N);
  return CurDAG->getTargetFrameIndex(FI->getIndex(), MVT::i32);
}]>;

// Copied from the AArch64 backend:
def bitcast_fpimm_to_i64 : SDNodeXForm<fpimm, [{
return CurDAG->getTargetConstant(
  N->getValueAPF().bitcastToAPInt().getZExtValue(), SDLoc(N), MVT::i64);
}]>;

class bitextract_imm<int bitnum> : SDNodeXForm<imm, [{
  uint64_t Imm = N->getZExtValue();
  unsigned Bit = (Imm >> }] # bitnum # [{ ) & 1;
  return CurDAG->getTargetConstant(Bit, SDLoc(N), MVT::i1);
}]>;

// ------------------------------------
def SIMM16bit : ImmLeaf <i32,
  [{return isInt<16>(Imm);}]
>;

def UIMM16bit : ImmLeaf <i32,
  [{return isUInt<16>(Imm);}]
>;

class InlineImm <ValueType vt> : PatLeaf <(vt imm), [{
  return isInlineImmediate(N);
}]>;

class InlineFPImm <ValueType vt> : PatLeaf <(vt fpimm), [{
  return isInlineImmediate(N);
}]>;

class VGPRImm <dag frag> : PatLeaf<frag, [{
  return isVGPRImm(N);
}]>;


//===----------------------------------------------------------------------===//
// Custom Operands
//===----------------------------------------------------------------------===//

def SoppBrTarget : AsmOperandClass {
  let Name = "SoppBrTarget";
  let ParserMethod = "parseSOppBrTarget";
}

def sopp_brtarget : Operand<OtherVT> {
  let EncoderMethod = "getSOPPBrEncoding";
  let DecoderMethod = "decodeSoppBrTarget";
  let OperandType = "OPERAND_PCREL";
  let ParserMatchClass = SoppBrTarget;
}

//===----------------------------------------------------------------------===//
// Vector ALU classes
//===----------------------------------------------------------------------===//


class Commutable_REV <string revOp, bit isOrig> {
  string RevOp = revOp;
  bit IsOrig = isOrig;
}

def EndpgmMatchClass : AsmOperandClass {
  let Name = "EndpgmImm";
  let PredicateMethod = "isEndpgm";
  let ParserMethod = "parseEndpgmOp";
  let RenderMethod = "addImmOperands";
  let IsOptional = 1;
}


def SwizzleMatchClass : AsmOperandClass {
  let Name = "Swizzle";
  let PredicateMethod = "isSwizzle";
  let ParserMethod = "parseSwizzleOp";
  let RenderMethod = "addImmOperands";
  let IsOptional = 1;
}

def SwizzleImm : Operand<i16> {
  let PrintMethod = "printSwizzle";
  let ParserMatchClass = SwizzleMatchClass;
}

def EndpgmImm : Operand<i16> {
  let PrintMethod = "printEndpgm";
  let ParserMatchClass = EndpgmMatchClass;
}

def SWaitMatchClass : AsmOperandClass {
  let Name = "SWaitCnt";
  let RenderMethod = "addImmOperands";
  let ParserMethod = "parseSWaitCntOps";
}

def WAIT_FLAG : Operand <i32> {
  let ParserMatchClass = SWaitMatchClass;
  let PrintMethod = "printWaitFlag";
  let OperandType = "OPERAND_IMMEDIATE";
}

def SendMsgMatchClass : AsmOperandClass {
  let Name = "SendMsg";
  let PredicateMethod = "isSendMsg";
  let ParserMethod = "parseSendMsgOp";
  let RenderMethod = "addImmOperands";
}

def SendMsgImm : Operand<i32> {
  let PrintMethod = "printSendMsg";
  let ParserMatchClass = SendMsgMatchClass;
}


def BoolReg : AsmOperandClass {
  let Name = "BoolReg";
  let ParserMethod = "parseBoolReg";
  let RenderMethod = "addRegOperands";
}

class BoolRC : RegisterOperand<SReg_1> {
  let ParserMatchClass = BoolReg;
  let DecoderMethod = "decodeBoolReg";
}

def SSrc_i1 : RegisterOperand<SReg_1> {
  let ParserMatchClass = BoolReg;
  let DecoderMethod = "decodeBoolReg";
}

def VOPDstS64orS32 : BoolRC {
  let PrintMethod = "printVOPDst";
}

// SCSrc_i1 is the operand for pseudo instructions only.
// Boolean immeadiates shall not be exposed to codegen instructions.
def SCSrc_i1 : RegisterOperand<SReg_1> {
  let OperandNamespace = "PPU";
  let OperandType = "OPERAND_REG_IMM_INT32";
  let ParserMatchClass = BoolReg;
  let DecoderMethod = "decodeBoolReg";
}





class NamedMatchClass<string CName, bit Optional = 1> : AsmOperandClass {
  let Name = "Imm"#CName;
  let PredicateMethod = "is"#CName;
  let ParserMethod = !if(Optional, "parseOptionalOperand", "parse"#CName);
  let RenderMethod = "addImmOperands";
  let IsOptional = Optional;
  let DefaultMethod = !if(Optional, "default"#CName, ?);
}

class NamedOperandBit<string Name, AsmOperandClass MatchClass> : Operand<i1> {
  let PrintMethod = "print"#Name;
  let ParserMatchClass = MatchClass;
}

class NamedOperandU8<string Name, AsmOperandClass MatchClass> : Operand<i8> {
  let PrintMethod = "print"#Name;
  let ParserMatchClass = MatchClass;
}

class NamedOperandU16<string Name, AsmOperandClass MatchClass> : Operand<i16> {
  let PrintMethod = "print"#Name;
  let ParserMatchClass = MatchClass;
}

class NamedOperandU32<string Name, AsmOperandClass MatchClass> : Operand<i32> {
  let PrintMethod = "print"#Name;
  let ParserMatchClass = MatchClass;
}

class NamedOperandU32Default0<string Name, AsmOperandClass MatchClass> :
  OperandWithDefaultOps<i32, (ops (i32 0))> {
  let PrintMethod = "print"#Name;
  let ParserMatchClass = MatchClass;
}

let OperandType = "OPERAND_IMMEDIATE" in {

def offen : NamedOperandBit<"Offen", NamedMatchClass<"Offen">>;
def idxen : NamedOperandBit<"Idxen", NamedMatchClass<"Idxen">>;
def addr64 : NamedOperandBit<"Addr64", NamedMatchClass<"Addr64">>;

def flat_offset : NamedOperandU16<"FlatOffset", NamedMatchClass<"FlatOffset">>;
def offset : NamedOperandU16<"Offset", NamedMatchClass<"Offset">>;
def offset0 : NamedOperandU8<"Offset0", NamedMatchClass<"Offset0">>;
def offset1 : NamedOperandU8<"Offset1", NamedMatchClass<"Offset1">>;

def gds : NamedOperandBit<"GDS", NamedMatchClass<"GDS">>;

def omod : NamedOperandU32<"OModSI", NamedMatchClass<"OModSI">>;
def clampmod : NamedOperandBit<"ClampSI", NamedMatchClass<"ClampSI">>;
def highmod : NamedOperandBit<"High", NamedMatchClass<"High">>;

def DLC : NamedOperandBit<"DLC", NamedMatchClass<"DLC">>;
def GLC : NamedOperandBit<"GLC", NamedMatchClass<"GLC">>;
def SLC : NamedOperandBit<"SLC", NamedMatchClass<"SLC">>;
def TFE : NamedOperandBit<"TFE", NamedMatchClass<"TFE">>;
def UNorm : NamedOperandBit<"UNorm", NamedMatchClass<"UNorm">>;
def DA : NamedOperandBit<"DA", NamedMatchClass<"DA">>;
def R128A16 : NamedOperandBit<"R128A16", NamedMatchClass<"R128A16">>;
def D16 : NamedOperandBit<"D16", NamedMatchClass<"D16">>;
def LWE : NamedOperandBit<"LWE", NamedMatchClass<"LWE">>;
def exp_compr : NamedOperandBit<"ExpCompr", NamedMatchClass<"ExpCompr">>;
def exp_vm : NamedOperandBit<"ExpVM", NamedMatchClass<"ExpVM">>;

def FORMAT : NamedOperandU8<"FORMAT", NamedMatchClass<"FORMAT">>;

def DMask : NamedOperandU16<"DMask", NamedMatchClass<"DMask">>;
def Dim : NamedOperandU8<"Dim", NamedMatchClass<"Dim", 0>>;

def dpp8 : NamedOperandU32<"DPP8", NamedMatchClass<"DPP8", 0>>;

def dpp_ctrl : NamedOperandU32<"DPPCtrl", NamedMatchClass<"DPPCtrl", 0>>;
def row_mask : NamedOperandU32<"RowMask", NamedMatchClass<"RowMask">>;
def bank_mask : NamedOperandU32<"BankMask", NamedMatchClass<"BankMask">>;
def bound_ctrl : NamedOperandBit<"BoundCtrl", NamedMatchClass<"BoundCtrl">>;
def FI : NamedOperandU32<"FI", NamedMatchClass<"FI">>;

def dst_sel : NamedOperandU32<"SDWADstSel", NamedMatchClass<"SDWADstSel">>;
def src0_sel : NamedOperandU32<"SDWASrc0Sel", NamedMatchClass<"SDWASrc0Sel">>;
def src1_sel : NamedOperandU32<"SDWASrc1Sel", NamedMatchClass<"SDWASrc1Sel">>;
def dst_unused : NamedOperandU32<"SDWADstUnused", NamedMatchClass<"SDWADstUnused">>;

def op_sel : NamedOperandU32Default0<"OpSel", NamedMatchClass<"OpSel">>;
def op_sel_hi : NamedOperandU32Default0<"OpSelHi", NamedMatchClass<"OpSelHi">>;
def neg_lo : NamedOperandU32Default0<"NegLo", NamedMatchClass<"NegLo">>;
def neg_hi : NamedOperandU32Default0<"NegHi", NamedMatchClass<"NegHi">>;

def blgp : NamedOperandU32<"BLGP", NamedMatchClass<"BLGP">>;
def cbsz : NamedOperandU32<"CBSZ", NamedMatchClass<"CBSZ">>;
def abid : NamedOperandU32<"ABID", NamedMatchClass<"ABID">>;

def hwreg : NamedOperandU16<"Hwreg", NamedMatchClass<"Hwreg", 0>>;

def exp_tgt : NamedOperandU8<"ExpTgt", NamedMatchClass<"ExpTgt", 0>> {

}

} // End OperandType = "OPERAND_IMMEDIATE"

class FPInputModsMatchClass <int opSize> : AsmOperandClass {
  let Name = "RegOrImmWithFP"#opSize#"InputMods";
  let ParserMethod = "parseRegOrImmWithFPInputMods";
  let PredicateMethod = "isRegOrImmWithFP"#opSize#"InputMods";
}

def FP16InputModsMatchClass : FPInputModsMatchClass<16>;
def FP32InputModsMatchClass : FPInputModsMatchClass<32>;
def FP64InputModsMatchClass : FPInputModsMatchClass<64>;

class InputMods <AsmOperandClass matchClass> : Operand <i32> {
  let OperandNamespace = "PPU";
  let OperandType = "OPERAND_INPUT_MODS";
  let ParserMatchClass = matchClass;
}

class FPInputMods <FPInputModsMatchClass matchClass> : InputMods <matchClass> {
  let PrintMethod = "printOperandAndFPInputMods";
}

def FP16InputMods : FPInputMods<FP16InputModsMatchClass>;
def FP32InputMods : FPInputMods<FP32InputModsMatchClass>;
def FP64InputMods : FPInputMods<FP64InputModsMatchClass>;

class IntInputModsMatchClass <int opSize> : AsmOperandClass {
  let Name = "RegOrImmWithInt"#opSize#"InputMods";
  let ParserMethod = "parseRegOrImmWithIntInputMods";
  let PredicateMethod = "isRegOrImmWithInt"#opSize#"InputMods";
}
def Int32InputModsMatchClass : IntInputModsMatchClass<32>;
def Int64InputModsMatchClass : IntInputModsMatchClass<64>;

class IntInputMods <IntInputModsMatchClass matchClass> : InputMods <matchClass> {
  let PrintMethod = "printOperandAndIntInputMods";
}
def Int32InputMods : IntInputMods<Int32InputModsMatchClass>;
def Int64InputMods : IntInputMods<Int64InputModsMatchClass>;

class OpSelModsMatchClass : AsmOperandClass {
  let Name = "OpSelMods";
  let ParserMethod = "parseRegOrImm";
  let PredicateMethod = "isRegOrImm";
}

def IntOpSelModsMatchClass : OpSelModsMatchClass;
def IntOpSelMods : InputMods<IntOpSelModsMatchClass>;


def FPVRegInputModsMatchClass : AsmOperandClass {
  let Name = "VRegWithFPInputMods";
  let ParserMethod = "parseRegWithFPInputMods";
  let PredicateMethod = "isVReg32";
}

def FPVRegInputMods : InputMods <FPVRegInputModsMatchClass> {
  let PrintMethod = "printOperandAndFPInputMods";
}

def IntVRegInputModsMatchClass : AsmOperandClass {
  let Name = "VRegWithIntInputMods";
  let ParserMethod = "parseRegWithIntInputMods";
  let PredicateMethod = "isVReg32";
}

def IntVRegInputMods : InputMods <IntVRegInputModsMatchClass> {
  let PrintMethod = "printOperandAndIntInputMods";
}

class PackedFPInputModsMatchClass <int opSize> : AsmOperandClass {
  let Name = "PackedFP"#opSize#"InputMods";
  let ParserMethod = "parseRegOrImm";
  let PredicateMethod = "isRegOrImm";
//  let PredicateMethod = "isPackedFP"#opSize#"InputMods";
}

class PackedIntInputModsMatchClass <int opSize> : AsmOperandClass {
  let Name = "PackedInt"#opSize#"InputMods";
  let ParserMethod = "parseRegOrImm";
  let PredicateMethod = "isRegOrImm";
//  let PredicateMethod = "isPackedInt"#opSize#"InputMods";
}

def PackedF16InputModsMatchClass : PackedFPInputModsMatchClass<16>;
def PackedI16InputModsMatchClass : PackedIntInputModsMatchClass<16>;

class PackedFPInputMods <PackedFPInputModsMatchClass matchClass> : InputMods <matchClass> {
//  let PrintMethod = "printPackedFPInputMods";
}

class PackedIntInputMods <PackedIntInputModsMatchClass matchClass> : InputMods <matchClass> {
  //let PrintMethod = "printPackedIntInputMods";
}

def PackedF16InputMods : PackedFPInputMods<PackedF16InputModsMatchClass>;
def PackedI16InputMods : PackedIntInputMods<PackedI16InputModsMatchClass>;

//===----------------------------------------------------------------------===//
// Complex patterns
//===----------------------------------------------------------------------===//




//===----------------------------------------------------------------------===//
// SI assembler operands
//===----------------------------------------------------------------------===//

def SIOperand {
  int ZERO = 0x80;
  int VCC = 0x6A;
  int FLAT_SCR = 0x68;
}

// This should be kept in sync with SISrcMods enum
def SRCMODS {
  int NONE = 0;
  int NEG = 1;
  int ABS = 2;
  int NEG_ABS = 3;

  int NEG_HI = ABS;
  int OP_SEL_0 = 4;
  int OP_SEL_1 = 8;
  int DST_OP_SEL = 8;
}

def DSTCLAMP {
  int NONE = 0;
  int ENABLE = 1;
}

def DSTOMOD {
  int NONE = 0;
}

def TRAPID{
  int LLVM_TRAP = 2;
  int LLVM_DEBUG_TRAP = 3;
}

def HWREG {
  int MODE = 1;
  int STATUS = 2;
  int TRAPSTS = 3;
  int HW_ID = 4;
  int GPR_ALLOC = 5;
  int LDS_ALLOC = 6;
  int IB_STS = 7;
  int MEM_BASES = 15;
  int TBA_LO = 16;
  int TBA_HI = 17;
  int TMA_LO = 18;
  int TMA_HI = 19;
  int FLAT_SCR_LO = 20;
  int FLAT_SCR_HI = 21;
  int XNACK_MASK = 22;
  int POPS_PACKER = 25;
}

class getHwRegImm<int Reg, int Offset = 0, int Size = 32> {
  int ret = !or(Reg,
                !or(!shl(Offset, 6),
                    !shl(!add(Size, -1), 11)));
}

//===----------------------------------------------------------------------===//
//
// PPU Instruction multiclass helpers.
//
// Instructions with _32 take 32-bit operands.
// Instructions with _64 take 64-bit operands.
//
// VOP_* instructions can use either a 32-bit or 64-bit encoding.  The 32-bit
// encoding is the standard encoding, but instruction that make use of
// any of the instruction modifiers must use the 64-bit encoding.
//
// Instructions with _e32 use the 32-bit encoding.
// Instructions with _e64 use the 64-bit encoding.
//
//===----------------------------------------------------------------------===//

class PPUMCInstr <string pseudo, int subtarget> {
  string PseudoInstr = pseudo;
  int Subtarget = subtarget;
}





//===----------------------------------------------------------------------===//
// Vector ALU classes
//===----------------------------------------------------------------------===//

class getNumSrcArgs<ValueType Src0, ValueType Src1, ValueType Src2> {
  int ret =
    !if (!eq(Src0.Value, untyped.Value),      0,
      !if (!eq(Src1.Value, untyped.Value),    1,   // VOP1
         !if (!eq(Src2.Value, untyped.Value), 2,   // VOP2
                                              3))); // VOP3
}

// Returns the register class to use for the destination of VOP[123C]
// instructions for the given VT.
class getVALUDstForVT<ValueType VT> {
  RegisterOperand ret = !if(!eq(VT.Size, 32), VOPDstOperand<VPR_32>,
                            !if(!eq(VT.Size, 64), VOPDstOperand<VReg_64>,
                              !if(!eq(VT.Size, 16), VOPDstOperand<VPR_32>,
                              VOPDstS64orS32))); // else VT == i1
//                          !if(!eq(VT.Size, 128), VOPDstOperand<VReg_128>,
}
// Returns true if VT is floating point.
class getIsFP<ValueType VT> {
  bit ret = !if(!eq(VT.Value, f16.Value), 1,
            !if(!eq(VT.Value, v2f16.Value), 1,
            !if(!eq(VT.Value, v4f16.Value), 1,
            !if(!eq(VT.Value, f32.Value), 1,
            !if(!eq(VT.Value, v2f32.Value), 1,
            !if(!eq(VT.Value, f64.Value), 1,
            !if(!eq(VT.Value, v2f64.Value), 1,
            0)))))));
}
// Returns the register class to use for source 0 of VOP[12C]
// instructions for the given VT.
class getVOPSrc0ForVT<ValueType VT> {
  bit isFP = getIsFP<VT>.ret;

  RegisterOperand ret =
    !if(isFP,
      !if(!eq(VT.Size, 64),
         VSrc_f64,
         !if(!eq(VT.Value, f16.Value),
            VSrc_f16,
            !if(!eq(VT.Value, v2f16.Value),
               VSrc_v2f16,
               !if(!eq(VT.Value, v4f16.Value),
                 VSrc_v4f16,
                 VSrc_f32
               )
            )
         )
       ),
       !if(!eq(VT.Size, 64),
          VSrc_b64,
          !if(!eq(VT.Value, i16.Value),
             VSrc_b16,
             !if(!eq(VT.Value, v2i16.Value),
                VSrc_v2b16,
                VSrc_b32
             )
          )
       )
    );
}

// Returns the vreg register class to use for source operand given VT
class getVregSrcForVT<ValueType VT> {
  RegisterClass ret = !if(!eq(VT.Size, 64), VReg_64,
                            !if(!eq(VT.Size, 48), VReg_64,
                              VPR_32));
}

// Returns the register class to use for sources of VOP3 instructions for the
// given VT.
class getVOP3SrcForVT<ValueType VT> {
  bit isFP = getIsFP<VT>.ret;
  RegisterOperand ret =
     !if(!eq(VT.Size, 64),
        !if(isFP,
           VSrc_f64,
           VSrc_b64),
        !if(!eq(VT.Value, i1.Value),
           SSrc_i1,
           !if(isFP,
              !if(!eq(VT.Value, f16.Value),
                 VSrc_f16,
                 !if(!eq(VT.Value, v2f16.Value),
                    VSrc_v2f16,
                    !if(!eq(VT.Value, v4f16.Value),
                      VSrc_v4f16,
                      VSrc_f32
                    )
                 )
              ),
              !if(!eq(VT.Value, i16.Value),
                 VSrc_b16,
                 !if(!eq(VT.Value, v2i16.Value),
                    VSrc_v2b16,
                    VSrc_b32
                 )
              )
           )
        )
     );
}

// Float or packed int
class isModifierType<ValueType SrcVT> {
  bit ret =
    !if(!eq(SrcVT.Value, f16.Value), 1,
    !if(!eq(SrcVT.Value, f32.Value), 1,
    !if(!eq(SrcVT.Value, f64.Value), 1,
    !if(!eq(SrcVT.Value, v2f16.Value), 1,
    !if(!eq(SrcVT.Value, v2i16.Value), 1,
    0)))));
}

// Return type of input modifiers operand for specified input operand
class getSrcMod <ValueType VT, bit EnableF32SrcMods> {
  bit isFP = getIsFP<VT>.ret;
  bit isPacked = isPackedType<VT>.ret;
  Operand ret =  !if(!eq(VT.Size, 64),
                     !if(isFP, FP64InputMods, Int64InputMods),
                       !if(isFP,
                         !if(!eq(VT.Value, f16.Value),
                            FP16InputMods,
                            FP32InputMods
                          ),
                         !if(EnableF32SrcMods, FP32InputMods, Int32InputMods))
                     );
}

class getOpSelMod <ValueType VT> {
  Operand ret = !if(!eq(VT.Value, f16.Value), FP16InputMods, IntOpSelMods);
}

// Return type of input modifiers operand specified input operand for DPP
class getSrcModExt <ValueType VT> {
  bit isFP = getIsFP<VT>.ret;
  Operand ret = !if(isFP, FPVRegInputMods, IntVRegInputMods);
}

// Returns the input arguments for VOP[12C] instructions for the given SrcVT.
class getIns32 <RegisterOperand Src0RC, RegisterClass Src1RC, int NumSrcArgs> {
  dag ret = !if(!eq(NumSrcArgs, 1), (ins Src0RC:$src0),               // VOP1
            !if(!eq(NumSrcArgs, 2), (ins Src0RC:$src0, Src1RC:$src1), // VOP2
                                    (ins)));
}

// Returns the input arguments for VOP3 instructions for the given SrcVT.
class getIns64 <RegisterOperand Src0RC, RegisterOperand Src1RC,
                RegisterOperand Src2RC, int NumSrcArgs,
                bit HasIntClamp, bit HasModifiers, bit HasSrc2Mods, bit HasOMod,
                Operand Src0Mod, Operand Src1Mod, Operand Src2Mod> {

  dag ret =
    !if (!eq(NumSrcArgs, 0),
      // VOP1 without input operands (V_NOP, V_CLREXCP)
      (ins),
      /* else */
    !if (!eq(NumSrcArgs, 1),
      !if (!eq(HasModifiers, 1),
        // VOP1 with modifiers
        (ins Src0Mod:$src0_modifiers, Src0RC:$src0,
             clampmod:$clamp, omod:$omod)
      /* else */,
        // VOP1 without modifiers
        !if (!eq(HasIntClamp, 1),
          (ins Src0RC:$src0, clampmod:$clamp),
          (ins Src0RC:$src0))
      /* endif */ ),
    !if (!eq(NumSrcArgs, 2),
      !if (!eq(HasModifiers, 1),
        // VOP 2 with modifiers
        !if( !eq(HasOMod, 1),
          (ins Src0Mod:$src0_modifiers, Src0RC:$src0,
               Src1Mod:$src1_modifiers, Src1RC:$src1,
               clampmod:$clamp, omod:$omod),
           (ins Src0Mod:$src0_modifiers, Src0RC:$src0,
               Src1Mod:$src1_modifiers, Src1RC:$src1,
               clampmod:$clamp))
      /* else */,
        // VOP2 without modifiers
        !if (!eq(HasIntClamp, 1),
          (ins Src0RC:$src0, Src1RC:$src1, clampmod:$clamp),
          (ins Src0RC:$src0, Src1RC:$src1))

      /* endif */ )
    /* NumSrcArgs == 3 */,
      !if (!eq(HasModifiers, 1),
        !if (!eq(HasSrc2Mods, 1),
          // VOP3 with modifiers
          !if (!eq(HasOMod, 1),
            (ins Src0Mod:$src0_modifiers, Src0RC:$src0,
                 Src1Mod:$src1_modifiers, Src1RC:$src1,
                 Src2Mod:$src2_modifiers, Src2RC:$src2,
                 clampmod:$clamp, omod:$omod),
            !if (!eq(HasIntClamp, 1),
              (ins Src0Mod:$src0_modifiers, Src0RC:$src0,
                   Src1Mod:$src1_modifiers, Src1RC:$src1,
                   Src2Mod:$src2_modifiers, Src2RC:$src2,
                   clampmod:$clamp),
              (ins Src0Mod:$src0_modifiers, Src0RC:$src0,
                   Src1Mod:$src1_modifiers, Src1RC:$src1,
                   Src2Mod:$src2_modifiers, Src2RC:$src2))),
          // VOP3 with modifiers except src2
          !if (!eq(HasOMod, 1),
            (ins Src0Mod:$src0_modifiers, Src0RC:$src0,
                 Src1Mod:$src1_modifiers, Src1RC:$src1,
                 Src2RC:$src2, clampmod:$clamp, omod:$omod),
            !if (!eq(HasIntClamp, 1),
              (ins Src0Mod:$src0_modifiers, Src0RC:$src0,
                   Src1Mod:$src1_modifiers, Src1RC:$src1,
                   Src2RC:$src2, clampmod:$clamp),
              (ins Src0Mod:$src0_modifiers, Src0RC:$src0,
                   Src1Mod:$src1_modifiers, Src1RC:$src1,
                   Src2RC:$src2))))
      /* else */,
        // VOP3 without modifiers
        !if (!eq(HasIntClamp, 1),
          (ins Src0RC:$src0, Src1RC:$src1, Src2RC:$src2, clampmod:$clamp),
          (ins Src0RC:$src0, Src1RC:$src1, Src2RC:$src2))
      /* endif */ ))));
}

/// XXX - src1 may only allow VGPRs?

// The modifiers (except clamp) are dummy operands for the benefit of
// printing and parsing. They defer their values to looking at the
// srcN_modifiers for what to print.
class getInsVOP3P <RegisterOperand Src0RC, RegisterOperand Src1RC,
                   RegisterOperand Src2RC, int NumSrcArgs,
                   bit HasClamp,
                   Operand Src0Mod, Operand Src1Mod, Operand Src2Mod> {
  dag ret = !if (!eq(NumSrcArgs, 2),
    !if (HasClamp,
      (ins Src0Mod:$src0_modifiers, Src0RC:$src0,
           Src1Mod:$src1_modifiers, Src1RC:$src1,
           clampmod:$clamp,
           op_sel:$op_sel, op_sel_hi:$op_sel_hi,
           neg_lo:$neg_lo, neg_hi:$neg_hi),
      (ins Src0Mod:$src0_modifiers, Src0RC:$src0,
           Src1Mod:$src1_modifiers, Src1RC:$src1,
           op_sel:$op_sel, op_sel_hi:$op_sel_hi,
           neg_lo:$neg_lo, neg_hi:$neg_hi)),
    // else NumSrcArgs == 3
    !if (HasClamp,
      (ins Src0Mod:$src0_modifiers, Src0RC:$src0,
           Src1Mod:$src1_modifiers, Src1RC:$src1,
           Src2Mod:$src2_modifiers, Src2RC:$src2,
           clampmod:$clamp,
           op_sel:$op_sel, op_sel_hi:$op_sel_hi,
           neg_lo:$neg_lo, neg_hi:$neg_hi),
      (ins Src0Mod:$src0_modifiers, Src0RC:$src0,
           Src1Mod:$src1_modifiers, Src1RC:$src1,
           Src2Mod:$src2_modifiers, Src2RC:$src2,
           op_sel:$op_sel, op_sel_hi:$op_sel_hi,
           neg_lo:$neg_lo, neg_hi:$neg_hi))
  );
}

class getInsVOP3OpSel <RegisterOperand Src0RC,
                       RegisterOperand Src1RC,
                       RegisterOperand Src2RC,
                       int NumSrcArgs,
                       bit HasClamp,
                       Operand Src0Mod,
                       Operand Src1Mod,
                       Operand Src2Mod> {
  dag ret = !if (!eq(NumSrcArgs, 2),
    !if (HasClamp,
      (ins Src0Mod:$src0_modifiers, Src0RC:$src0,
           Src1Mod:$src1_modifiers, Src1RC:$src1,
           clampmod:$clamp,
           op_sel:$op_sel),
      (ins Src0Mod:$src0_modifiers, Src0RC:$src0,
           Src1Mod:$src1_modifiers, Src1RC:$src1,
           op_sel:$op_sel)),
    // else NumSrcArgs == 3
    !if (HasClamp,
      (ins Src0Mod:$src0_modifiers, Src0RC:$src0,
           Src1Mod:$src1_modifiers, Src1RC:$src1,
           Src2Mod:$src2_modifiers, Src2RC:$src2,
           clampmod:$clamp,
           op_sel:$op_sel),
      (ins Src0Mod:$src0_modifiers, Src0RC:$src0,
           Src1Mod:$src1_modifiers, Src1RC:$src1,
           Src2Mod:$src2_modifiers, Src2RC:$src2,
           op_sel:$op_sel))
  );
}

// Returns the assembly string for the inputs and outputs of a VOP[12C]
// instruction.  This does not add the _e32 suffix, so it can be reused
// by getAsm64.
class getAsm32 <bit HasDst, int NumSrcArgs, ValueType DstVT = i32> {
  string dst = !if(!eq(DstVT.Size, 1), "$sdst", "$vdst"); // use $sdst for VOPC
  string src0 = ", $src0";
  string src1 = ", $src1";
  string src2 = ", $src2";
  string ret = !if(HasDst, dst, "") #
               !if(!eq(NumSrcArgs, 1), src0, "") #
               !if(!eq(NumSrcArgs, 2), src0#src1, "") #
               !if(!eq(NumSrcArgs, 3), src0#src1#src2, "");
}


// Returns the assembly string for the inputs and outputs of a VOP3
// instruction.
class getAsm64 <bit HasDst, int NumSrcArgs, bit HasIntClamp, bit HasModifiers,
                bit HasOMod, ValueType DstVT = i32> {
  string dst = !if(!eq(DstVT.Size, 1), "$sdst", "$vdst"); // use $sdst for VOPC
  string src0 = !if(!eq(NumSrcArgs, 1), "$src0_modifiers", "$src0_modifiers,");
  string src1 = !if(!eq(NumSrcArgs, 1), "",
                   !if(!eq(NumSrcArgs, 2), " $src1_modifiers",
                                           " $src1_modifiers,"));
  string src2 = !if(!eq(NumSrcArgs, 3), " $src2_modifiers", "");
  string iclamp = !if(HasIntClamp, "$clamp", "");
  string ret =
  !if(!eq(HasModifiers, 0),
      getAsm32<HasDst, NumSrcArgs, DstVT>.ret # iclamp,
      dst#", "#src0#src1#src2#"$clamp"#!if(HasOMod, "$omod", ""));
}

// Returns the assembly string for the inputs and outputs of a VOP3P
// instruction.
class getAsmVOP3P <bit HasDst, int NumSrcArgs, bit HasModifiers,
                   bit HasClamp, ValueType DstVT = i32> {
  string dst = " $vdst";
  string src0 = !if(!eq(NumSrcArgs, 1), "$src0", "$src0,");
  string src1 = !if(!eq(NumSrcArgs, 1), "",
                   !if(!eq(NumSrcArgs, 2), " $src1",
                                           " $src1,"));
  string src2 = !if(!eq(NumSrcArgs, 3), " $src2", "");

  string mods = !if(HasModifiers, "$neg_lo$neg_hi", "");
  string clamp = !if(HasClamp, "$clamp", "");

  // Each modifier is printed as an array of bits for each operand, so
  // all operands are printed as part of src0_modifiers.
  string ret = dst#", "#src0#src1#src2#"$op_sel$op_sel_hi"#mods#clamp;
}

class getAsmVOP3OpSel <int NumSrcArgs,
                       bit HasClamp,
                       bit Src0HasMods,
                       bit Src1HasMods,
                       bit Src2HasMods> {
  string dst = " $vdst";

  string isrc0 = !if(!eq(NumSrcArgs, 1), "$src0", "$src0,");
  string isrc1 = !if(!eq(NumSrcArgs, 1), "",
                     !if(!eq(NumSrcArgs, 2), " $src1",
                                             " $src1,"));
  string isrc2 = !if(!eq(NumSrcArgs, 3), " $src2", "");

  string fsrc0 = !if(!eq(NumSrcArgs, 1), "$src0_modifiers", "$src0_modifiers,");
  string fsrc1 = !if(!eq(NumSrcArgs, 1), "",
                     !if(!eq(NumSrcArgs, 2), " $src1_modifiers",
                                             " $src1_modifiers,"));
  string fsrc2 = !if(!eq(NumSrcArgs, 3), " $src2_modifiers", "");

  string src0 = !if(Src0HasMods, fsrc0, isrc0);
  string src1 = !if(Src1HasMods, fsrc1, isrc1);
  string src2 = !if(Src2HasMods, fsrc2, isrc2);

  string clamp = !if(HasClamp, "$clamp", "");

  string ret = dst#", "#src0#src1#src2#"$op_sel"#clamp;
}

class BitOr<bit a, bit b> {
  bit ret = !if(a, 1, !if(b, 1, 0));
}

class BitAnd<bit a, bit b> {
  bit ret = !if(a, !if(b, 1, 0), 0);
}

def PatGenMode {
  int NoPattern = 0;
  int Pattern   = 1;
}


class VOPProfile <list<ValueType> _ArgVT, bit _EnableF32SrcMods = 0,
                  bit _EnableClamp = 0> {

  field list<ValueType> ArgVT = _ArgVT;
  field bit EnableF32SrcMods = _EnableF32SrcMods;
  field bit EnableClamp = _EnableClamp;

  field ValueType DstVT = ArgVT[0];
  field ValueType Src0VT = ArgVT[1];
  field ValueType Src1VT = ArgVT[2];
  field ValueType Src2VT = ArgVT[3];
  field RegisterOperand DstRC = getVALUDstForVT<DstVT>.ret;
  // field RegisterOperand DstRCDPP = getVALUDstForVT<DstVT>.ret;
  // field RegisterOperand DstRCSDWA = getSDWADstForVT<DstVT>.ret;
  field RegisterOperand Src0RC32 = getVOPSrc0ForVT<Src0VT>.ret;
  field RegisterClass Src1RC32 = getVregSrcForVT<Src1VT>.ret;
  field RegisterOperand Src0RC64 = getVOP3SrcForVT<Src0VT>.ret;
  field RegisterOperand Src1RC64 = getVOP3SrcForVT<Src1VT>.ret;
  field RegisterOperand Src2RC64 = getVOP3SrcForVT<Src2VT>.ret;
  // field RegisterClass Src0DPP = getVregSrcForVT<Src0VT>.ret;
  // field RegisterClass Src1DPP = getVregSrcForVT<Src1VT>.ret;
  // field RegisterOperand Src0SDWA = getSDWASrcForVT<Src0VT>.ret;
  // field RegisterOperand Src1SDWA = getSDWASrcForVT<Src0VT>.ret;
  field Operand Src0Mod = getSrcMod<Src0VT, EnableF32SrcMods>.ret;
  field Operand Src1Mod = getSrcMod<Src1VT, EnableF32SrcMods>.ret;
  field Operand Src2Mod = getSrcMod<Src2VT, EnableF32SrcMods>.ret;
  // field Operand Src0ModDPP = getSrcModExt<Src0VT>.ret;
  // field Operand Src1ModDPP = getSrcModExt<Src1VT>.ret;
  // field Operand Src0ModSDWA = getSrcModSDWA<Src0VT>.ret;
  // field Operand Src1ModSDWA = getSrcModSDWA<Src1VT>.ret;


  field bit HasDst = !if(!eq(DstVT.Value, untyped.Value), 0, 1);
  field bit HasDst32 = HasDst;
  field bit EmitDst = HasDst; // force dst encoding, see v_movreld_b32 special case
  field int NumSrcArgs = getNumSrcArgs<Src0VT, Src1VT, Src2VT>.ret;
  field bit HasSrc0 = !if(!eq(Src0VT.Value, untyped.Value), 0, 1);
  field bit HasSrc1 = !if(!eq(Src1VT.Value, untyped.Value), 0, 1);
  field bit HasSrc2 = !if(!eq(Src2VT.Value, untyped.Value), 0, 1);

  // TODO: Modifiers logic is somewhat adhoc here, to be refined later
  // HasModifiers affects the normal and DPP encodings. We take note of EnableF32SrcMods, which
  // enables modifiers for i32 type.
  field bit HasModifiers = BitOr<isModifierType<Src0VT>.ret, EnableF32SrcMods>.ret;

  // HasSrc*FloatMods affects the SDWA encoding. We ignore EnableF32SrcMods.
  field bit HasSrc0FloatMods = isFloatType<Src0VT>.ret;
  field bit HasSrc1FloatMods = isFloatType<Src1VT>.ret;
  field bit HasSrc2FloatMods = isFloatType<Src2VT>.ret;

  // HasSrc*IntMods affects the SDWA encoding. We ignore EnableF32SrcMods.
  field bit HasSrc0IntMods = isIntType<Src0VT>.ret;
  field bit HasSrc1IntMods = isIntType<Src1VT>.ret;
  field bit HasSrc2IntMods = isIntType<Src2VT>.ret;

  field bit HasSrc0Mods = HasModifiers;
  field bit HasSrc1Mods = !if(HasModifiers, BitOr<HasSrc1FloatMods, HasSrc1IntMods>.ret, 0);
  field bit HasSrc2Mods = !if(HasModifiers, BitOr<HasSrc2FloatMods, HasSrc2IntMods>.ret, 0);

  field bit HasClamp = BitOr<isModifierType<Src0VT>.ret, EnableClamp>.ret;
  // field bit HasSDWAClamp = EmitDst;
  field bit HasFPClamp = BitAnd<isFloatType<DstVT>.ret, HasClamp>.ret;
  field bit HasIntClamp = !if(isFloatType<DstVT>.ret, 0, HasClamp);
  field bit HasClampLo = HasClamp;
  field bit HasClampHi = BitAnd<isPackedType<DstVT>.ret, HasClamp>.ret;
  field bit HasHigh = 0;

  field bit IsPacked = isPackedType<Src0VT>.ret;
  field bit HasOpSel = IsPacked;
  field bit HasOMod = !if(HasOpSel, 0, isFloatType<DstVT>.ret);
  // field bit HasSDWAOMod = isFloatType<DstVT>.ret;

  // field bit HasExt = getHasExt<NumSrcArgs, DstVT, Src0VT, Src1VT>.ret;
  // field bit HasExtDPP = getHasDPP<NumSrcArgs, DstVT, Src0VT, Src1VT>.ret;
  // field bit HasExtSDWA = HasExt;
  // field bit HasExtSDWA9 = HasExt;
  field int NeedPatGen = PatGenMode.NoPattern;

  field bit IsMAI = 0;

  field Operand Src0PackedMod = !if(HasSrc0FloatMods, PackedF16InputMods, PackedI16InputMods);
  field Operand Src1PackedMod = !if(HasSrc1FloatMods, PackedF16InputMods, PackedI16InputMods);
  field Operand Src2PackedMod = !if(HasSrc2FloatMods, PackedF16InputMods, PackedI16InputMods);

  field dag Outs = !if(HasDst,(outs DstRC:$vdst),(outs));

  // VOP3b instructions are a special case with a second explicit
  // output. This is manually overridden for them.
  field dag Outs32 = Outs;
  field dag Outs64 = Outs;
  // field dag OutsDPP = getOutsExt<HasDst, DstVT, DstRCDPP>.ret;
  // field dag OutsDPP8 = getOutsExt<HasDst, DstVT, DstRCDPP>.ret;
  // field dag OutsSDWA = getOutsSDWA<HasDst, DstVT, DstRCSDWA>.ret;

  field dag Ins32 = getIns32<Src0RC32, Src1RC32, NumSrcArgs>.ret;
  field dag Ins64 = getIns64<Src0RC64, Src1RC64, Src2RC64, NumSrcArgs,
                             HasIntClamp, HasModifiers, HasSrc2Mods,
                             HasOMod, Src0Mod, Src1Mod, Src2Mod>.ret;
  field dag InsVOP3P = getInsVOP3P<Src0RC64, Src1RC64, Src2RC64,
                                   NumSrcArgs, HasClamp,
                                   Src0PackedMod, Src1PackedMod, Src2PackedMod>.ret;
  field dag InsVOP3OpSel = getInsVOP3OpSel<Src0RC64, Src1RC64, Src2RC64,
                                           NumSrcArgs,
                                           HasClamp,
                                           getOpSelMod<Src0VT>.ret,
                                           getOpSelMod<Src1VT>.ret,
                                           getOpSelMod<Src2VT>.ret>.ret;
  // field dag InsDPP = !if(HasExtDPP,
  //                        getInsDPP<DstRCDPP, Src0DPP, Src1DPP, NumSrcArgs,
  //                                  HasModifiers, Src0ModDPP, Src1ModDPP>.ret,
  //                        (ins));
  // field dag InsDPP16 = getInsDPP16<DstRCDPP, Src0DPP, Src1DPP, NumSrcArgs,
  //                                  HasModifiers, Src0ModDPP, Src1ModDPP>.ret;
  // field dag InsDPP8 = getInsDPP8<DstRCDPP, Src0DPP, Src1DPP, NumSrcArgs, 0,
  //                                Src0ModDPP, Src1ModDPP>.ret;
  // field dag InsSDWA = getInsSDWA<Src0SDWA, Src1SDWA, NumSrcArgs,
  //                                HasSDWAOMod, Src0ModSDWA, Src1ModSDWA,
  //                                DstVT>.ret;


  field string Asm32 = getAsm32<HasDst, NumSrcArgs, DstVT>.ret;
  field string Asm64 = getAsm64<HasDst, NumSrcArgs, HasIntClamp, HasModifiers, HasOMod, DstVT>.ret;
  field string AsmVOP3P = getAsmVOP3P<HasDst, NumSrcArgs, HasModifiers, HasClamp, DstVT>.ret;
  field string AsmVOP3OpSel = getAsmVOP3OpSel<NumSrcArgs,
                                              HasClamp,
                                              HasSrc0FloatMods,
                                              HasSrc1FloatMods,
                                              HasSrc2FloatMods>.ret;
  // field string AsmDPP = !if(HasExtDPP,
  //                           getAsmDPP<HasDst, NumSrcArgs, HasModifiers, DstVT>.ret, "");
  // field string AsmDPP16 = getAsmDPP16<HasDst, NumSrcArgs, HasModifiers, DstVT>.ret;
  // field string AsmDPP8 = getAsmDPP8<HasDst, NumSrcArgs, 0, DstVT>.ret;
  // field string AsmSDWA = getAsmSDWA<HasDst, NumSrcArgs, DstVT>.ret;
  // field string AsmSDWA9 = getAsmSDWA9<HasDst, HasSDWAOMod, NumSrcArgs, DstVT>.ret;

  field string TieRegDPP = "$old";
}

def VOP_F16_F16 : VOPProfile <[f16, f16, untyped, untyped]>;
def VOP_F16_I16 : VOPProfile <[f16, i16, untyped, untyped]>;
def VOP_I16_F16 : VOPProfile <[i16, f16, untyped, untyped]>;

def VOP_F16_F16_F16 : VOPProfile <[f16, f16, f16, untyped]>;
def VOP_F16_F16_I16 : VOPProfile <[f16, f16, i16, untyped]>;
def VOP_F16_F16_I32 : VOPProfile <[f16, f16, i32, untyped]>;
def VOP_I16_I16_I16 : VOPProfile <[i16, i16, i16, untyped]>;

def VOP_I16_I16_I16_I16 : VOPProfile <[i16, i16, i16, i16, untyped]>;
def VOP_F16_F16_F16_F16 : VOPProfile <[f16, f16, f16, f16, untyped]>;

def VOP_I32_I16_I16_I32 : VOPProfile <[i32, i16, i16, i32, untyped]>;

def VOP_V2F16_V2F16_V2F16 : VOPProfile <[v2f16, v2f16, v2f16, untyped]>;
def VOP_V2I16_V2I16_V2I16 : VOPProfile <[v2i16, v2i16, v2i16, untyped]>;
def VOP_B32_F16_F16 : VOPProfile <[i32, f16, f16, untyped]>;

def VOP_V2F16_V2F16_V2F16_V2F16 : VOPProfile <[v2f16, v2f16, v2f16, v2f16]>;
def VOP_V2I16_V2I16_V2I16_V2I16 : VOPProfile <[v2i16, v2i16, v2i16, v2i16]>;
def VOP_V2I16_F32_F32 : VOPProfile <[v2i16, f32, f32, untyped]>;
def VOP_V2I16_I32_I32 : VOPProfile <[v2i16, i32, i32, untyped]>;

def VOP_F32_V2F16_V2F16_V2F16 : VOPProfile <[f32, v2f16, v2f16, v2f16]>;

def VOP_NONE : VOPProfile <[untyped, untyped, untyped, untyped]>;

def VOP_F32_F32 : VOPProfile <[f32, f32, untyped, untyped]>;
def VOP_F32_F64 : VOPProfile <[f32, f64, untyped, untyped]>;
def VOP_F32_I32 : VOPProfile <[f32, i32, untyped, untyped]>;
def VOP_F64_F32 : VOPProfile <[f64, f32, untyped, untyped]>;
def VOP_F64_F64 : VOPProfile <[f64, f64, untyped, untyped]>;
def VOP_F64_I32 : VOPProfile <[f64, i32, untyped, untyped]>;
def VOP_I32_F32 : VOPProfile <[i32, f32, untyped, untyped]>;
def VOP_I32_F64 : VOPProfile <[i32, f64, untyped, untyped]>;
def VOP_I32_I32 : VOPProfile <[i32, i32, untyped, untyped]>;
def VOP_F16_F32 : VOPProfile <[f16, f32, untyped, untyped]>;
def VOP_F32_F16 : VOPProfile <[f32, f16, untyped, untyped]>;

def VOP_F32_F32_F16 : VOPProfile <[f32, f32, f16, untyped]>;
def VOP_F32_F32_F32 : VOPProfile <[f32, f32, f32, untyped]>;
def VOP_F32_F32_I32 : VOPProfile <[f32, f32, i32, untyped]>;
def VOP_F64_F64_F64 : VOPProfile <[f64, f64, f64, untyped]>;
def VOP_F64_F64_I32 : VOPProfile <[f64, f64, i32, untyped]>;
def VOP_I32_F32_F32 : VOPProfile <[i32, f32, f32, untyped]>;
def VOP_I32_F32_I32 : VOPProfile <[i32, f32, i32, untyped]>;
def VOP_I32_I32_I32 : VOPProfile <[i32, i32, i32, untyped]>;
def VOP_I32_I32_I32_ARITH : VOPProfile <[i32, i32, i32, untyped], 0, /*EnableClamp=*/1>;
def VOP_V2F16_F32_F32 : VOPProfile <[v2f16, f32, f32, untyped]>;
def VOP_F32_F16_F16_F16 : VOPProfile <[f32, f16, f16, f16]>;

def VOP_I64_I64_I32 : VOPProfile <[i64, i64, i32, untyped]>;
def VOP_I64_I32_I64 : VOPProfile <[i64, i32, i64, untyped]>;
def VOP_I64_I64_I64 : VOPProfile <[i64, i64, i64, untyped]>;

def VOP_F16_F32_F16_F32 : VOPProfile <[f16, f32, f16, f32]>;
def VOP_F32_F32_F16_F16 : VOPProfile <[f32, f32, f16, f16]>;
def VOP_F32_F32_F32_F32 : VOPProfile <[f32, f32, f32, f32]>;
def VOP_F64_F64_F64_F64 : VOPProfile <[f64, f64, f64, f64]>;
def VOP_I32_I32_I32_I32 : VOPProfile <[i32, i32, i32, i32]>;
def VOP_I64_I32_I32_I64 : VOPProfile <[i64, i32, i32, i64]>;
def VOP_I32_F32_I32_I32 : VOPProfile <[i32, f32, i32, i32]>;
def VOP_I64_I64_I32_I64 : VOPProfile <[i64, i64, i32, i64]>;
def VOP_V4I32_I64_I32_V4I32 : VOPProfile <[v4i32, i64, i32, v4i32]>;

def VOP_F32_V2F16_V2F16_F32 : VOPProfile <[f32, v2f16, v2f16, f32]>;
def VOP_I32_V2I16_V2I16_I32 : VOPProfile <[i32, v2i16, v2i16, i32]>;

def VOP_V4F32_F32_F32_V4F32       : VOPProfile <[v4f32,  f32,   f32,   v4f32]>;
def VOP_V16F32_F32_F32_V16F32     : VOPProfile <[v16f32, f32,   f32,   v16f32]>;
def VOP_V32F32_F32_F32_V32F32     : VOPProfile <[v32f32, f32,   f32,   v32f32]>;
def VOP_V4F32_V4F16_V4F16_V4F32   : VOPProfile <[v4f32,  v4f16, v4f16, v4f32]>;
def VOP_V16F32_V4F16_V4F16_V16F32 : VOPProfile <[v16f32, v4f16, v4f16, v16f32]>;
def VOP_V32F32_V4F16_V4F16_V32F32 : VOPProfile <[v32f32, v4f16, v4f16, v32f32]>;
def VOP_V4F32_V2I16_V2I16_V4F32   : VOPProfile <[v4f32,  v2i16, v2i16, v4f32]>;
def VOP_V16F32_V2I16_V2I16_V16F32 : VOPProfile <[v16f32, v2i16, v2i16, v16f32]>;
def VOP_V32F32_V2I16_V2I16_V32F32 : VOPProfile <[v32f32, v2i16, v2i16, v32f32]>;
def VOP_V4I32_I32_I32_V4I32       : VOPProfile <[v4i32,  i32,   i32,   v4i32]>;
def VOP_V16I32_I32_I32_V16I32     : VOPProfile <[v16i32, i32,   i32,   v16i32]>;
def VOP_V32I32_I32_I32_V32I32     : VOPProfile <[v32i32, i32,   i32,   v32i32]>;


class AtomicNoRet <string noRetOp, bit isRet> {
  string NoRetOp = noRetOp;
  bit IsRet = isRet;
}


include "PPUInstrFormatsT_SOP.td"
// VOP
include "PPUInstrFormatsT_VOP.td"
include "PPUInstrFormatsT_VOP1.td"
include "PPUInstrFormatsT_VOPC.td"

include "PPUInstrFormatsT_SM.td"
// include "PPUInstrFormatsT_FLAT.td"
include "PPUInstrFormatsT_BUF.td"



include "PPUInstrInfoT_SOP.td"
// VOP
include "PPUInstrInfoT_VOP1.td"
include "PPUInstrInfoT_VOPC.td"

include "PPUInstrInfoT_SM.td"
// include "PPUInstrInfoT_FLAT.td"
include "PPUInstrInfoT_BUF.td"

