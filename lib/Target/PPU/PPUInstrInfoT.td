// below is base on SIInstrInfo.td
// note that:
//    in original, SIIntrInfo.td include the SIIntructions.td/DSInstruction.td/MIMGInstructions.td
//    and SIIntructions.td include SOPInstrutions.td/VOPInstructions.td/SMInstructions.td/FLAT.../BUF
// in this file:
//   it firstly define most of SIInstrInfo.td defintion, and the include OPtype instructions td file , the OP instructions is rename to PPUInstrInfoT_*.td and PPUInstrFormatsT_*.td
//  after include all Op type instruction.td , we add SIInstruction.td's definition in the end
// 
// line 945: include "PPUInstrFormatsT.td"

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

def PPUclamp : SDNode<"PPUISD::CLAMP", SDTFPUnaryOp>;

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

def NegateImm : SDNodeXForm<imm, [{
  return CurDAG->getConstant(-N->getSExtValue(), SDLoc(N), MVT::i32);
}]>;

// TODO: When FP inline imm values work?
def NegSubInlineConst32 : ImmLeaf<i32, [{
  return Imm < -16 && Imm >= -64;
}], NegateImm>;

def NegSubInlineConst16 : ImmLeaf<i16, [{
  return Imm < -16 && Imm >= -64;
}], NegateImm>;

def ShiftAmt32Imm : PatLeaf <(imm), [{
  return N->getZExtValue() < 32;
}]>;

def getNegV2I16Imm : SDNodeXForm<build_vector, [{
  return SDValue(packNegConstantV2I16(N, *CurDAG), 0);
}]>;

def NegSubInlineConstV216 : PatLeaf<(build_vector), [{
  assert(N->getNumOperands() == 2);
  assert(N->getOperand(0).getValueType().getSizeInBits() == 16);
  SDValue Src0 = N->getOperand(0);
  SDValue Src1 = N->getOperand(1);
  if (Src0 == Src1)
    return isNegInlineImmediate(Src0.getNode());

  return (isNullConstantOrUndef(Src0) && isNegInlineImmediate(Src1.getNode())) ||
         (isNullConstantOrUndef(Src1) && isNegInlineImmediate(Src0.getNode()));
}], getNegV2I16Imm>;


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

def si_ga : Operand<iPTR>;

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

class KImmMatchClass<int size> : AsmOperandClass {
  let Name = "KImmFP"#size;
  let PredicateMethod = "isKImmFP"#size;
  let ParserMethod = "parseImm";
  let RenderMethod = "addKImmFP"#size#"Operands";
}

class kimmOperand<ValueType vt> : Operand<vt> {
  let OperandNamespace = "PPU";
  let OperandType = "OPERAND_KIMM"#vt.Size;
  let PrintMethod = "printU"#vt.Size#"ImmOperand";
  let ParserMatchClass = !cast<AsmOperandClass>("KImmFP"#vt.Size#"MatchClass");
}

// 32-bit VALU immediate operand that uses the constant bus.
def KImmFP32MatchClass : KImmMatchClass<32>;
def f32kimm : kimmOperand<i32>;

// 32-bit VALU immediate operand with a 16-bit value that uses the
// constant bus.
def KImmFP16MatchClass : KImmMatchClass<16>;
def f16kimm : kimmOperand<i16>;

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

def DS1Addr1Offset : ComplexPattern<i32, 2, "SelectDS1Addr1Offset">;
def DS64Bit4ByteAligned : ComplexPattern<i32, 3, "SelectDS64Bit4ByteAligned">;

def MOVRELOffset : ComplexPattern<i32, 2, "SelectMOVRELOffset">;

def VOP3Mods0 : ComplexPattern<untyped, 4, "SelectVOP3Mods0">;
def VOP3Mods0Clamp : ComplexPattern<untyped, 3, "SelectVOP3Mods0Clamp">;
def VOP3Mods0Clamp0OMod : ComplexPattern<untyped, 4, "SelectVOP3Mods0Clamp0OMod">;
def VOP3Mods  : ComplexPattern<untyped, 2, "SelectVOP3Mods">;
def VOP3NoMods : ComplexPattern<untyped, 1, "SelectVOP3NoMods">;
// VOP3Mods, but the input source is known to never be NaN.
def VOP3Mods_nnan : ComplexPattern<fAny, 2, "SelectVOP3Mods_NNaN">;
// VOP3Mods, but only allowed for f32 operands.
def VOP3Mods_f32 : ComplexPattern<fAny, 2, "SelectVOP3Mods_f32">;

def VOP3OMods : ComplexPattern<untyped, 3, "SelectVOP3OMods">;

def VOP3PMods  : ComplexPattern<untyped, 2, "SelectVOP3PMods">;
def VOP3PMods0 : ComplexPattern<untyped, 3, "SelectVOP3PMods0">;

def VOP3OpSel  : ComplexPattern<untyped, 2, "SelectVOP3OpSel">;
def VOP3OpSel0 : ComplexPattern<untyped, 3, "SelectVOP3OpSel0">;

def VOP3OpSelMods  : ComplexPattern<untyped, 2, "SelectVOP3OpSelMods">;
def VOP3OpSelMods0 : ComplexPattern<untyped, 3, "SelectVOP3OpSelMods0">;

def VOP3PMadMixMods  : ComplexPattern<untyped, 2, "SelectVOP3PMadMixMods">;


def Hi16Elt  : ComplexPattern<untyped, 1, "SelectHi16Elt">;




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

include "PPUInstrFormatsT.td"

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
// EXP classes
//===----------------------------------------------------------------------===//
/*
class EXP_Helper<bit done, SDPatternOperator node = null_frag> : EXPCommon<
  (outs),
  (ins exp_tgt:$tgt,
       ExpSrc0:$src0, ExpSrc1:$src1, ExpSrc2:$src2, ExpSrc3:$src3,
       exp_vm:$vm, exp_compr:$compr, i8imm:$en),
  "exp$tgt $src0, $src1, $src2, $src3"#!if(done, " done", "")#"$compr$vm",
  [(node (i8 timm:$tgt), (i8 timm:$en),
         f32:$src0, f32:$src1, f32:$src2, f32:$src3,
         (i1 timm:$compr), (i1 timm:$vm))]> {
  let AsmMatchConverter = "cvtExp";
}

// Split EXP instruction into EXP and EXP_DONE so we can set
// mayLoad for done=1.
multiclass EXP_m<bit done, SDPatternOperator node> {
  let mayLoad = done, DisableWQM = 1 in {
    let isPseudo = 1, isCodeGenOnly = 1 in {
      def "" : EXP_Helper<done, node>,
               SIMCInstr <"exp"#!if(done, "_done", ""), PPUEncodingFamily.NONE>;
    }

    let done = done in {
      def _si : EXP_Helper<done>,
                SIMCInstr <"exp"#!if(done, "_done", ""), PPUEncodingFamily.PPU>,
                EXPe {
        let AssemblerPredicates = [isPPT];
        let DecoderNamespace = "PPU";
        let DisableDecoder = DisableSIDecoder;
      }
    }
  }
}
*/



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
  field Operand Src0ModDPP = getSrcModExt<Src0VT>.ret;
  field Operand Src1ModDPP = getSrcModExt<Src1VT>.ret;
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

class VOP_NO_EXT <VOPProfile p> : VOPProfile <p.ArgVT> {
  /*
  let HasExt = 0;
  let HasExtDPP = 0;
  let HasExtSDWA = 0;
  let HasExtSDWA9 = 0;
  */
}

class VOP_PAT_GEN <VOPProfile p, int mode=PatGenMode.Pattern> : VOPProfile <p.ArgVT> {
  let NeedPatGen = mode;
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
include "PPUInstrFormatsT_VOP2.td"
include "PPUInstrFormatsT_VOP3.td"
include "PPUInstrFormatsT_VOP3P.td"
include "PPUInstrFormatsT_VOPC.td"

include "PPUInstrFormatsT_SM.td"
// include "PPUInstrFormatsT_FLAT.td"
include "PPUInstrFormatsT_BUF.td"



include "PPUInstrInfoT_SOP.td"
// VOP
include "PPUInstrInfoT_VOP1.td"
include "PPUInstrInfoT_VOP2.td"
include "PPUInstrInfoT_VOP3.td"
include "PPUInstrInfoT_VOP3P.td"
include "PPUInstrInfoT_VOPC.td"

include "PPUInstrInfoT_SM.td"
// include "PPUInstrInfoT_FLAT.td"
include "PPUInstrInfoT_BUF.td"

//===----------------------------------------------------------------------===//
// EXP Instructions
//===----------------------------------------------------------------------===//

// defm EXP : EXP_m<0, PPUexport>;
// defm EXP_DONE : EXP_m<1, PPUexport_done>;


//===----------------------------------------------------------------------===//
// Pseudo Instructions
//===----------------------------------------------------------------------===//
def ATOMIC_FENCE : SPseudoInst<
  (outs), (ins i32imm:$ordering, i32imm:$scope),
  [(atomic_fence (i32 imm:$ordering), (i32 imm:$scope))],
  "ATOMIC_FENCE $ordering, $scope"> {
  let hasSideEffects = 1;
  let maybeAtomic = 1;
}



let hasSideEffects = 0, mayLoad = 0, mayStore = 0, Uses = [TMSK] in {

// For use in patterns
def V_CNDMASK_B64_PSEUDO : VOP3Common <(outs VReg_64:$vdst),
  (ins VSrc_b64:$src0, VSrc_b64:$src1, SSrc_b64:$src2), "", []> {
  let isPseudo = 1;
  let isCodeGenOnly = 1;
  let usesCustomInserter = 1;
}

// 64-bit vector move instruction. This is mainly used by the
// SIFoldOperands pass to enable folding of inline immediates.
def V_MOV_B64_PSEUDO : VPseudoInst <(outs VReg_64:$vdst),
                                      (ins VSrc_b64:$src0)>;

// Pseudoinstruction for @llvm.amdgcn.wqm. It is turned into a copy after the
// WQM pass processes it.
def WQM : PPTPseudo <(outs unknown:$vdst), (ins unknown:$src0)>;

// Pseudoinstruction for @llvm.amdgcn.softwqm. Like @llvm.amdgcn.wqm it is
// turned into a copy by WQM pass, but does not seed WQM requirements.
def SOFT_WQM : PPTPseudo <(outs unknown:$vdst), (ins unknown:$src0)>;

// Pseudoinstruction for @llvm.amdgcn.wwm. It is turned into a copy post-RA, so
// that the @earlyclobber is respected. The @earlyclobber is to make sure that
// the instruction that defines $src0 (which is run in WWM) doesn't
// accidentally clobber inactive channels of $vdst.
let Constraints = "@earlyclobber $vdst" in {
def WWM : PPTPseudo <(outs unknown:$vdst), (ins unknown:$src0)>;
}

} // End let hasSideEffects = 0, mayLoad = 0, mayStore = 0, Uses = [TMSK]

def ENTER_WWM : SPseudoInst <(outs SReg_1:$sdst), (ins i64imm:$src0)> {
  let Defs = [TMSK];
  let hasSideEffects = 0;
  let mayLoad = 0;
  let mayStore = 0;
}

def EXIT_WWM : SPseudoInst <(outs SReg_1:$sdst), (ins SReg_1:$src0)> {
  let hasSideEffects = 0;
  let mayLoad = 0;
  let mayStore = 0;
}

// Invert the exec mask and overwrite the inactive lanes of dst with inactive,
// restoring it after we're done.
def V_SET_INACTIVE_B32 : VPseudoInst <(outs VPR_32:$vdst),
  (ins VPR_32: $src, VSrc_b32:$inactive),
  [(set i32:$vdst, (int_ppu_set_inactive i32:$src, i32:$inactive))]> {
  let Constraints = "$src = $vdst";
}

let usesCustomInserter = 1, Defs = [SCC] in {
def S_ADD_U64_PSEUDO : SPseudoInst <
  (outs SReg_64:$vdst), (ins SSrc_b64:$src0, SSrc_b64:$src1),
  [(set SReg_64:$vdst, (add i64:$src0, i64:$src1))]
>;

def S_SUB_U64_PSEUDO : SPseudoInst <
  (outs SReg_64:$vdst), (ins SSrc_b64:$src0, SSrc_b64:$src1),
  [(set SReg_64:$vdst, (sub i64:$src0, i64:$src1))]
>;

def S_ADD_U64_CO_PSEUDO : SPseudoInst <
  (outs SReg_64:$vdst, VOPDstS64orS32:$sdst), (ins SSrc_b64:$src0, SSrc_b64:$src1)
>;

def S_SUB_U64_CO_PSEUDO : SPseudoInst <
  (outs SReg_64:$vdst, VOPDstS64orS32:$sdst), (ins SSrc_b64:$src0, SSrc_b64:$src1)
>;
} // End usesCustomInserter = 1, Defs = [SCC]

/*
let usesCustomInserter = 1 in {
def GET_GROUPSTATICSIZE : SPseudoInst <(outs SReg_32:$sdst), (ins),
  [(set SReg_32:$sdst, (int_ppu_groupstaticsize))]>;
} // End let usesCustomInserter = 1, SALU = 1
*/

// Wrap an instruction by duplicating it, except for setting isTerminator.
class WrapTerminatorInst<SOP_Pseudo base_inst> : SPseudoInst<
      base_inst.OutOperandList,
      base_inst.InOperandList> {
  let Uses = base_inst.Uses;
  let Defs = base_inst.Defs;
  let isTerminator = 1;
  let isAsCheapAsAMove = base_inst.isAsCheapAsAMove;
  let hasSideEffects = base_inst.hasSideEffects;
  let UseNamedOperandTable = base_inst.UseNamedOperandTable;
  let CodeSize = base_inst.CodeSize;
}

// let WaveSizePredicate = isWave32 in {
def S_MOV_B32_term : WrapTerminatorInst<S_MOV_B32>;
def S_XOR_B32_term : WrapTerminatorInst<S_XOR_B32>;
def S_OR_B32_term : WrapTerminatorInst<S_OR_B32>;
def S_ANDN2_B32_term : WrapTerminatorInst<S_ANDN2_B32>;
// }

def WAVE_BARRIER : SPseudoInst<(outs), (ins),
  [(int_ppu_wave_barrier)]> {
  let SchedRW = [];
  let hasNoSchedulingInfo = 1;
  let hasSideEffects = 1;
  let mayLoad = 1;
  let mayStore = 1;
  let isConvergent = 1;
  let FixedSize = 1;
  let Size = 0;
}

// SI pseudo instructions. These are used by the CFG structurizer pass
// and should be lowered to ISA instructions prior to codegen.

// Dummy terminator instruction to use after control flow instructions
// replaced with exec mask operations.
def SI_MASK_BRANCH : VPseudoInst <
  (outs), (ins brtarget:$target)> {
  let isBranch = 0;
  let isTerminator = 1;
  let isBarrier = 0;
  let SchedRW = [];
  let hasNoSchedulingInfo = 1;
  let FixedSize = 1;
  let Size = 0;
}

// TODO vcc is SReg_32 or SReg_1?
let isTerminator = 1 in {
let Uses = [TMSK], Defs = [TMSK],
    OtherPredicates = [EnableNonUniformMIBranchPseudo] in {
 def SI_NON_UNIFORM_BRCOND_PSEUDO : CFPseudoInst <
  (outs),
  (ins SReg_32:$vcc, brtarget:$target),
  [(brcond i1:$vcc, bb:$target)]> {
    // let Size = 12;
}
}

def SI_IF: CFPseudoInst <
  (outs SReg_1:$dst), (ins SReg_1:$vcc, brtarget:$target),
  [(set i1:$dst, (AMDGPUif i1:$vcc, bb:$target))], 1, 1> {
  let Constraints = "";
  let Size = 12;
  let hasSideEffects = 1;
}

def SI_ELSE : CFPseudoInst <
  (outs SReg_1:$dst),
  (ins SReg_1:$src, brtarget:$target, i1imm:$execfix), [], 1, 1> {
  let Size = 12;
  let hasSideEffects = 1;
}

def SI_LOOP : CFPseudoInst <
  (outs), (ins SReg_1:$saved, brtarget:$target),
  [(AMDGPUloop i1:$saved, bb:$target)], 1, 1> {
  let Size = 8;
  let isBranch = 1;
  let hasSideEffects = 1;
}

} // End isTerminator = 1

def SI_END_CF : CFPseudoInst <
  (outs), (ins SReg_1:$saved), [], 1, 1> {
  let Size = 4;
  let isAsCheapAsAMove = 1;
  let isReMaterializable = 1;
  let hasSideEffects = 1;
  let mayLoad = 1; // FIXME: Should not need memory flags
  let mayStore = 1;
}

def SI_IF_BREAK : CFPseudoInst <
  (outs SReg_1:$dst), (ins SReg_1:$vcc, SReg_1:$src), []> {
  let Size = 4;
  let isAsCheapAsAMove = 1;
  let isReMaterializable = 1;
}

let Uses = [TMSK] in {

multiclass PseudoInstKill <dag ins> {
  // Even though this pseudo can usually be expanded without an SCC def, we
  // conservatively assume that it has an SCC def, both because it is sometimes
  // required in degenerate cases (when V_CMPX cannot be used due to constant
  // bus limitations) and because it allows us to avoid having to track SCC
  // liveness across basic blocks.
  let Defs = [TMSK,VCC,SCC] in
  def _PSEUDO : PPTPseudo <(outs), ins> {
    let isConvergent = 1;
    let usesCustomInserter = 1;
  }

  let Defs = [TMSK,VCC,SCC] in
  def _TERMINATOR : SPseudoInst <(outs), ins> {
    let isTerminator = 1;
  }
}

defm SI_KILL_I1 : PseudoInstKill <(ins SCSrc_i1:$src, i1imm:$killvalue)>;
defm SI_KILL_F32_COND_IMM : PseudoInstKill <(ins VSrc_b32:$src0, i32imm:$src1, i32imm:$cond)>;

let Defs = [TMSK,VCC] in
def SI_ILLEGAL_COPY : SPseudoInst <
  (outs unknown:$dst), (ins unknown:$src),
  [], " ; illegal copy $src to $dst">;

} // End Uses = [TMSK], Defs = [TMSK,VCC]

// Branch on undef scc. Used to avoid intermediate copy from
// IMPLICIT_DEF to SCC.
def SI_BR_UNDEF : SPseudoInst <(outs), (ins sopp_brtarget:$simm16)> {
  let isTerminator = 1;
  let usesCustomInserter = 1;
  let isBranch = 1;
}

/*
def SI_MASKED_UNREACHABLE : SPseudoInst <(outs), (ins),
  [(int_ppu_unreachable)],
  "; divergent unreachable"> {
  let Size = 0;
  let hasNoSchedulingInfo = 1;
  let FixedSize = 1;
}
*/

// Used as an isel pseudo to directly emit initialization with an
// s_mov_b32 rather than a copy of another initialized
// register. MachineCSE skips copies, and we don't want to have to
// fold operands before it runs.
def SI_INIT_M0 : SPseudoInst <(outs), (ins SSrc_b32:$src)> {
  let Defs = [M0];
  let usesCustomInserter = 1;
  let isAsCheapAsAMove = 1;
  let isReMaterializable = 1;
}

def SI_INIT_TMSK : SPseudoInst <
  (outs), (ins i32imm:$src), []> {
  let Defs = [TMSK];
  let usesCustomInserter = 1;
  let isAsCheapAsAMove = 1;
  // let WaveSizePredicate = isWave64;
}

def SI_INIT_TMSK_FROM_INPUT : SPseudoInst <
  (outs), (ins SSrc_b32:$input, i32imm:$shift), []> {
  let Defs = [TMSK];
  let usesCustomInserter = 1;
}

// Return for returning shaders to a shader variant epilog.
def SI_RETURN_TO_EPILOG : SPseudoInst <
  (outs), (ins variable_ops), [(AMDGPUreturn_to_epilog)]> {
  let isTerminator = 1;
  let isBarrier = 1;
  let isReturn = 1;
  let hasNoSchedulingInfo = 1;
  let DisableWQM = 1;
  let FixedSize = 1;
}


// Return for returning function calls.
def SI_RETURN : SPseudoInst <
  (outs), (ins), [],
  "; return"> {
  let isTerminator = 1;
  let isBarrier = 1;
  let isReturn = 1;
  let SchedRW = [WriteBranch];
}

// Return for returning function calls without output register.
//
// This version is only needed so we can fill in the output regiter in
// the custom inserter.
/* FIXME
def SI_CALL_ISEL : SPseudoInst <
  (outs), (ins SSrc_b64:$src0, unknown:$callee),
  [(AMDGPUcall i64:$src0, tglobaladdr:$callee)]> {
  let Size = 4;
  let isCall = 1;
  let SchedRW = [WriteBranch];
  let usesCustomInserter = 1;
  // TODO: Should really base this on the call target
  let isConvergent = 1;
}
*/

// Wrapper around s_swappc_b64 with extra $callee parameter to track
// the called function after regalloc.
def SI_CALL : SPseudoInst <
  (outs SReg_64:$dst), (ins SSrc_b64:$src0, unknown:$callee)> {
  let Size = 4;
  let isCall = 1;
  let UseNamedOperandTable = 1;
  let SchedRW = [WriteBranch];
  // TODO: Should really base this on the call target
  let isConvergent = 1;
}

/*
// Tail call handling pseudo
def SI_TCRETURN : SPseudoInst <(outs),
  (ins SSrc_b64:$src0, unknown:$callee, i32imm:$fpdiff),
  [(AMDGPUtc_return i64:$src0, tglobaladdr:$callee, i32:$fpdiff)]> {
  let Size = 4;
  let isCall = 1;
  let isTerminator = 1;
  let isReturn = 1;
  let isBarrier = 1;
  let UseNamedOperandTable = 1;
  let SchedRW = [WriteBranch];
  // TODO: Should really base this on the call target
  let isConvergent = 1;
}
*/

def PPT_ADJCALLSTACKUP : SPseudoInst<
  (outs), (ins i32imm:$amt0, i32imm:$amt1),
  [(callseq_start timm:$amt0, timm:$amt1)],
  "; ppt_adjcallstackup $amt0 $amt1"> {
  let Size = 8; // Worst case. (s_add_u32 + constant)
  let FixedSize = 1;
  let hasSideEffects = 1;
  let usesCustomInserter = 1;
  let SchedRW = [WriteSALU];
  let Defs = [SCC];
}

def PPT_ADJCALLSTACKDOWN : SPseudoInst<
  (outs), (ins i32imm:$amt1, i32imm:$amt2),
  [(callseq_end timm:$amt1, timm:$amt2)],
  "; ppt_adjcallstackdown $amt1"> {
  let Size = 8; // Worst case. (s_add_u32 + constant)
  let hasSideEffects = 1;
  let usesCustomInserter = 1;
  let SchedRW = [WriteSALU];
  let Defs = [SCC];
}

let Defs = [M0, TMSK, SCC],
  UseNamedOperandTable = 1 in {

class SI_INDIRECT_SRC<RegisterClass rc> : VPseudoInst <
  (outs VPR_32:$vdst),
  (ins rc:$src, VS_32:$idx, i32imm:$offset)> {
  let usesCustomInserter = 1;
}

class SI_INDIRECT_DST<RegisterClass rc> : VPseudoInst <
  (outs rc:$vdst),
  (ins rc:$src, VS_32:$idx, i32imm:$offset, VPR_32:$val)> {
  let Constraints = "$src = $vdst";
  let usesCustomInserter = 1;
}

// TODO: We can support indirect SGPR access.
def SI_INDIRECT_SRC_V1 : SI_INDIRECT_SRC<VPR_32>;
def SI_INDIRECT_SRC_V2 : SI_INDIRECT_SRC<VReg_64>;
/*
def SI_INDIRECT_SRC_V4 : SI_INDIRECT_SRC<VReg_128>;
def SI_INDIRECT_SRC_V8 : SI_INDIRECT_SRC<VReg_256>;
def SI_INDIRECT_SRC_V16 : SI_INDIRECT_SRC<VReg_512>;
*/

def SI_INDIRECT_DST_V1 : SI_INDIRECT_DST<VPR_32>;
def SI_INDIRECT_DST_V2 : SI_INDIRECT_DST<VReg_64>;
/*
def SI_INDIRECT_DST_V4 : SI_INDIRECT_DST<VReg_128>;
def SI_INDIRECT_DST_V8 : SI_INDIRECT_DST<VReg_256>;
def SI_INDIRECT_DST_V16 : SI_INDIRECT_DST<VReg_512>;
*/

} // End Uses = [TMSK], Defs = [M0, TMSK]


multiclass SI_SPILL_SGPR <RegisterClass sgpr_class> {
  let UseNamedOperandTable = 1, SPRSpill = 1, Uses = [TMSK] in {
    def _SAVE : PPTPseudo <
      (outs),
      (ins sgpr_class:$data, i32imm:$addr)> {
      let mayStore = 1;
      let mayLoad = 0;
    }

    def _RESTORE : PPTPseudo <
      (outs sgpr_class:$data),
      (ins i32imm:$addr)> {
      let mayStore = 0;
      let mayLoad = 1;
    }
  } // End UseNamedOperandTable = 1
}

// You cannot use M0 as the output of v_readlane_b32 instructions or
// use it in the sdata operand of SMEM instructions. We still need to
// be able to spill the physical register m0, so allow it for
// SI_SPILL_32_* instructions.
defm SI_SPILL_S32  : SI_SPILL_SGPR <SReg_32>;
defm SI_SPILL_S64  : SI_SPILL_SGPR <SReg_64>;
/*
defm SI_SPILL_S96  : SI_SPILL_SGPR <SReg_96>;
defm SI_SPILL_S128 : SI_SPILL_SGPR <SReg_128>;
defm SI_SPILL_S160 : SI_SPILL_SGPR <SReg_160>;
defm SI_SPILL_S256 : SI_SPILL_SGPR <SReg_256>;
defm SI_SPILL_S512 : SI_SPILL_SGPR <SReg_512>;
defm SI_SPILL_S1024 : SI_SPILL_SGPR <SReg_1024>;
*/

multiclass SI_SPILL_VGPR <RegisterClass vgpr_class> {
  let UseNamedOperandTable = 1, VPRSpill = 1,
       SchedRW = [WriteVMEM] in {
    def _SAVE : VPseudoInst <
      (outs),
      (ins vgpr_class:$vdata, i32imm:$vaddr, SReg_64:$srsrc,
           SReg_32:$soffset, i32imm:$offset)> {
      let mayStore = 1;
      let mayLoad = 0;
      // (2 * 4) + (8 * num_subregs) bytes maximum
      int MaxSize = !add(!shl(!srl(vgpr_class.Size, 5), 3), 8);
      // Size field is unsigned char and cannot fit more.
      let Size = !if(!le(MaxSize, 256), MaxSize, 252);
    }

    def _RESTORE : VPseudoInst <
      (outs vgpr_class:$vdata),
      (ins i32imm:$vaddr, SReg_64:$srsrc, SReg_32:$soffset,
           i32imm:$offset)> {
      let mayStore = 0;
      let mayLoad = 1;

      // (2 * 4) + (8 * num_subregs) bytes maximum
      int MaxSize = !add(!shl(!srl(vgpr_class.Size, 5), 3), 8);
      // Size field is unsigned char and cannot fit more.
      let Size = !if(!le(MaxSize, 256), MaxSize, 252);
    }
  } // End UseNamedOperandTable = 1, VPRSpill = 1, SchedRW = [WriteVMEM]
}

defm SI_SPILL_V32  : SI_SPILL_VGPR <VPR_32>;
defm SI_SPILL_V64  : SI_SPILL_VGPR <VReg_64>;
/*
defm SI_SPILL_V96  : SI_SPILL_VGPR <VReg_96>;
defm SI_SPILL_V128 : SI_SPILL_VGPR <VReg_128>;
defm SI_SPILL_V160 : SI_SPILL_VGPR <VReg_160>;
defm SI_SPILL_V256 : SI_SPILL_VGPR <VReg_256>;
defm SI_SPILL_V512 : SI_SPILL_VGPR <VReg_512>;
defm SI_SPILL_V1024 : SI_SPILL_VGPR <VReg_1024>;
*/
/*
multiclass SI_SPILL_AGPR <RegisterClass vgpr_class> {
  let UseNamedOperandTable = 1, VGPRSpill = 1,
      Constraints = "@earlyclobber $tmp",
      SchedRW = [WriteVMEM] in {
    def _SAVE : VPseudoInst <
      (outs VPR_32:$tmp),
      (ins vgpr_class:$vdata, i32imm:$vaddr, SReg_128:$srsrc,
           SReg_32:$soffset, i32imm:$offset)> {
      let mayStore = 1;
      let mayLoad = 0;
      // (2 * 4) + (16 * num_subregs) bytes maximum
      int MaxSize = !add(!shl(!srl(vgpr_class.Size, 5), 4), 8);
      // Size field is unsigned char and cannot fit more.
      let Size = !if(!le(MaxSize, 256), MaxSize, 252);
    }

    def _RESTORE : VPseudoInst <
      (outs vgpr_class:$vdata, VPR_32:$tmp),
      (ins i32imm:$vaddr, SReg_128:$srsrc, SReg_32:$soffset,
           i32imm:$offset)> {
      let mayStore = 0;
      let mayLoad = 1;

      // (2 * 4) + (16 * num_subregs) bytes maximum
      int MaxSize = !add(!shl(!srl(vgpr_class.Size, 5), 4), 8);
      // Size field is unsigned char and cannot fit more.
      let Size = !if(!le(MaxSize, 256), MaxSize, 252);
    }
  } // End UseNamedOperandTable = 1, VPRSpill = 1, SchedRW = [WriteVMEM]
}

defm SI_SPILL_A32  : SI_SPILL_AGPR <AGPR_32>;
defm SI_SPILL_A64  : SI_SPILL_AGPR <AReg_64>;
defm SI_SPILL_A128 : SI_SPILL_AGPR <AReg_128>;
defm SI_SPILL_A512 : SI_SPILL_AGPR <AReg_512>;
defm SI_SPILL_A1024 : SI_SPILL_AGPR <AReg_1024>;
*/

def SI_PC_ADD_REL_OFFSET : SPseudoInst <
  (outs SReg_64:$dst),
  (ins si_ga:$ptr_lo, si_ga:$ptr_hi),
  [(set SReg_64:$dst,
      (i64 (SIpc_add_rel_offset tglobaladdr:$ptr_lo, tglobaladdr:$ptr_hi)))]> {
  let Defs = [SCC];
}

def : PPUPat <
  (SIpc_add_rel_offset tglobaladdr:$ptr_lo, 0),
  (SI_PC_ADD_REL_OFFSET $ptr_lo, (i32 0))
>;

def : PPUPat <
  (PPUinit_exec i32:$src),
  (SI_INIT_TMSK (as_i32imm $src))
>;

def : PPUPat <
  (PPUinit_exec_from_input i32:$input, i32:$shift),
  (SI_INIT_TMSK_FROM_INPUT (i32 $input), (as_i32imm $shift))
>;

def : PPUPat<
  (PPUtrap timm:$trapid),
  (S_TRAP $trapid)
>;

def : PPUPat<
  (PPUelse i1:$src, bb:$target),
  (SI_ELSE $src, $target, 0)
>;

/* FIXME confict with mayLoad/Store
def : Pat <
  // -1.0 as i32 (LowerINTRINSIC_VOID converts all other constants to -1.0)
  (PPUkill (i32 -1082130432)),
  (SI_KILL_I1_PSEUDO (i1 0), 0)
>;

def : Pat <
  (int_ppu_kill i1:$src),
  (SI_KILL_I1_PSEUDO $src, 0)
>;


def : Pat <
  (int_ppu_kill (i1 (not i1:$src))),
  (SI_KILL_I1_PSEUDO $src, -1)
>;

def : Pat <
  (PPUkill i32:$src),
  (SI_KILL_F32_COND_IMM_PSEUDO $src, 0, 3) // 3 means SETOGE
>;


def : Pat <
  (int_ppu_kill (i1 (setcc f32:$src, InlineFPImm<f32>:$imm, cond:$cond))),
  (SI_KILL_F32_COND_IMM_PSEUDO $src, (bitcast_fpimm_to_i32 $imm), (cond_as_i32imm $cond))
>;
*/

// TODO: we could add more variants for other types of conditionals
/*
def : Pat <
  (i64 (int_ppu_icmp i1:$src, (i1 0), (i32 33))),
  (COPY $src) // Return the SGPRs representing i1 src
>;
*/

def : Pat <
  (i32 (int_ppu_icmp i1:$src, (i1 0), (i32 33))),
  (COPY $src) // Return the SGPRs representing i1 src
>;


//===----------------------------------------------------------------------===//
// VOP1 Patterns
//===----------------------------------------------------------------------===//

let OtherPredicates = [UnsafeFPMath] in {

//def : RcpPat<V_RCP_F64_e32, f64>;
//defm : RsqPat<V_RSQ_F64_e32, f64>;
//defm : RsqPat<V_RSQ_F32_e32, f32>;

def : RsqPat<V_RSQ_F32_e32, f32>;
// def : RsqPat<V_RSQ_F64_e32, f64>;

// Convert (x - floor(x)) to fract(x)
def : PPUPat <
  (f32 (fsub (f32 (VOP3Mods f32:$x, i32:$mods)),
             (f32 (ffloor (f32 (VOP3Mods f32:$x, i32:$mods)))))),
  (V_FRACT_F32_e64 $mods, $x, DSTCLAMP.NONE, DSTOMOD.NONE)
>;

// Convert (x + (-floor(x))) to fract(x)
/*
def : PPUPat <
  (f64 (fadd (f64 (VOP3Mods f64:$x, i32:$mods)),
             (f64 (fneg (f64 (ffloor (f64 (VOP3Mods f64:$x, i32:$mods)))))))),
  (V_FRACT_F64_e64 $mods, $x, DSTCLAMP.NONE, DSTOMOD.NONE)
>;
*/

} // End OtherPredicates = [UnsafeFPMath]


// f16_to_fp patterns
def : PPUPat <
  (f32 (f16_to_fp i32:$src0)),
  (V_CVT_F32_F16_e64 SRCMODS.NONE, $src0, DSTCLAMP.NONE, DSTOMOD.NONE)
>;

def : PPUPat <
  (f32 (f16_to_fp (and_oneuse i32:$src0, 0x7fff))),
  (V_CVT_F32_F16_e64 SRCMODS.ABS, $src0, DSTCLAMP.NONE, DSTOMOD.NONE)
>;

def : PPUPat <
  (f32 (f16_to_fp (i32 (srl_oneuse (and_oneuse i32:$src0, 0x7fff0000), (i32 16))))),
  (V_CVT_F32_F16_e64 SRCMODS.ABS, (i32 (V_LSHRREV_B32_e64 (i32 16), i32:$src0)), DSTCLAMP.NONE, DSTOMOD.NONE)
>;

def : PPUPat <
  (f32 (f16_to_fp (or_oneuse i32:$src0, 0x8000))),
  (V_CVT_F32_F16_e64 SRCMODS.NEG_ABS, $src0, DSTCLAMP.NONE, DSTOMOD.NONE)
>;

def : PPUPat <
  (f32 (f16_to_fp (xor_oneuse i32:$src0, 0x8000))),
  (V_CVT_F32_F16_e64 SRCMODS.NEG, $src0, DSTCLAMP.NONE, DSTOMOD.NONE)
>;

/*
def : PPUPat <
  (f64 (fpextend f16:$src)),
  (V_CVT_F64_F32_e32 (V_CVT_F32_F16_e32 $src))
>;
*/

// fp_to_fp16 patterns
def : PPUPat <
  (i32 (PPUfp_to_f16 (f32 (VOP3Mods f32:$src0, i32:$src0_modifiers)))),
  (V_CVT_F16_F32_e64 $src0_modifiers, f32:$src0, DSTCLAMP.NONE, DSTOMOD.NONE)
>;

def : PPUPat <
  (i32 (fp_to_sint f16:$src)),
  (V_CVT_I32_F32_e32 (V_CVT_F32_F16_e32 $src))
>;

def : PPUPat <
  (i32 (fp_to_uint f16:$src)),
  (V_CVT_U32_F32_e32 (V_CVT_F32_F16_e32 $src))
>;

def : PPUPat <
  (f16 (sint_to_fp i32:$src)),
  (V_CVT_F16_F32_e32 (V_CVT_F32_I32_e32 $src))
>;

def : PPUPat <
  (f16 (uint_to_fp i32:$src)),
  (V_CVT_F16_F32_e32 (V_CVT_F32_U32_e32 $src))
>;

//===----------------------------------------------------------------------===//
// VOP2 Patterns
//===----------------------------------------------------------------------===//

multiclass FMADPat <ValueType vt, Instruction inst> {
  def : PPUPat <
    (vt (fmad (VOP3NoMods vt:$src0),
              (VOP3NoMods vt:$src1),
              (VOP3NoMods vt:$src2))),
    (inst SRCMODS.NONE, $src0, SRCMODS.NONE, $src1,
          SRCMODS.NONE, $src2, DSTCLAMP.NONE, DSTOMOD.NONE)
  >;
}

defm : FMADPat <f16, V_MAC_F16_e64>;
defm : FMADPat <f32, V_MAC_F32_e64>;

class FMADModsPat<Instruction inst, SDPatternOperator mad_opr, ValueType Ty>
  : PPUPat<
  (Ty (mad_opr (VOP3Mods Ty:$src0, i32:$src0_mod),
  (VOP3Mods Ty:$src1, i32:$src1_mod),
  (VOP3Mods Ty:$src2, i32:$src2_mod))),
  (inst $src0_mod, $src0, $src1_mod, $src1,
  $src2_mod, $src2, DSTCLAMP.NONE, DSTOMOD.NONE)
>;

/* TODO 
def : FMADModsPat<V_MAD_F32, PPUfmad_ftz, f32>;
def : FMADModsPat<V_MAD_F16, PPUfmad_ftz, f16> {
  let SubtargetPredicate = Has16BitInsts;
}
*/

multiclass SelectPat <ValueType vt> {
  def : PPUPat <
    (vt (select i1:$src0, (VOP3Mods_f32 vt:$src1, i32:$src1_mods),
                          (VOP3Mods_f32 vt:$src2, i32:$src2_mods))),
    (V_CNDMASK_B32_e64 $src2_mods, $src2, $src1_mods, $src1, $src0)
  >;
}

defm : SelectPat <i16>;
defm : SelectPat <i32>;
defm : SelectPat <f16>;
defm : SelectPat <f32>;

let AddedComplexity = 1 in {
def : PPUPat <
  (i32 (add (i32 (getDivergentFrag<ctpop>.ret i32:$popcnt)), i32:$val)),
  (V_BCNT_U32_B32_e64 $popcnt, $val)
>;
}
def : PPUPat <
  (i16 (add (i16 (trunc (getDivergentFrag<ctpop>.ret i32:$popcnt))), i16:$val)),
  (V_BCNT_U32_B32_e64 $popcnt, $val)
>;

/********** ============================================ **********/
/********** Extraction, Insertion, Building and Casting  **********/
/********** ============================================ **********/

foreach Index = 0-2 in {
  def Extract_Element_v2i32_#Index : Extract_Element <
    i32, v2i32, Index, !cast<SubRegIndex>(sub#Index)
  >;
  def Insert_Element_v2i32_#Index : Insert_Element <
    i32, v2i32, Index, !cast<SubRegIndex>(sub#Index)
  >;

  def Extract_Element_v2f32_#Index : Extract_Element <
    f32, v2f32, Index, !cast<SubRegIndex>(sub#Index)
  >;
  def Insert_Element_v2f32_#Index : Insert_Element <
    f32, v2f32, Index, !cast<SubRegIndex>(sub#Index)
  >;
}

/*
foreach Index = 0-2 in {
  def Extract_Element_v3i32_#Index : Extract_Element <
    i32, v3i32, Index, !cast<SubRegIndex>(sub#Index)
  >;
  def Insert_Element_v3i32_#Index : Insert_Element <
    i32, v3i32, Index, !cast<SubRegIndex>(sub#Index)
  >;

  def Extract_Element_v3f32_#Index : Extract_Element <
    f32, v3f32, Index, !cast<SubRegIndex>(sub#Index)
  >;
  def Insert_Element_v3f32_#Index : Insert_Element <
    f32, v3f32, Index, !cast<SubRegIndex>(sub#Index)
  >;
}

foreach Index = 0-3 in {
  def Extract_Element_v4i32_#Index : Extract_Element <
    i32, v4i32, Index, !cast<SubRegIndex>(sub#Index)
  >;
  def Insert_Element_v4i32_#Index : Insert_Element <
    i32, v4i32, Index, !cast<SubRegIndex>(sub#Index)
  >;

  def Extract_Element_v4f32_#Index : Extract_Element <
    f32, v4f32, Index, !cast<SubRegIndex>(sub#Index)
  >;
  def Insert_Element_v4f32_#Index : Insert_Element <
    f32, v4f32, Index, !cast<SubRegIndex>(sub#Index)
  >;
}

foreach Index = 0-4 in {
  def Extract_Element_v5i32_#Index : Extract_Element <
    i32, v5i32, Index, !cast<SubRegIndex>(sub#Index)
  >;
  def Insert_Element_v5i32_#Index : Insert_Element <
    i32, v5i32, Index, !cast<SubRegIndex>(sub#Index)
  >;

  def Extract_Element_v5f32_#Index : Extract_Element <
    f32, v5f32, Index, !cast<SubRegIndex>(sub#Index)
  >;
  def Insert_Element_v5f32_#Index : Insert_Element <
    f32, v5f32, Index, !cast<SubRegIndex>(sub#Index)
  >;
}

foreach Index = 0-7 in {
  def Extract_Element_v8i32_#Index : Extract_Element <
    i32, v8i32, Index, !cast<SubRegIndex>(sub#Index)
  >;
  def Insert_Element_v8i32_#Index : Insert_Element <
    i32, v8i32, Index, !cast<SubRegIndex>(sub#Index)
  >;

  def Extract_Element_v8f32_#Index : Extract_Element <
    f32, v8f32, Index, !cast<SubRegIndex>(sub#Index)
  >;
  def Insert_Element_v8f32_#Index : Insert_Element <
    f32, v8f32, Index, !cast<SubRegIndex>(sub#Index)
  >;
}

foreach Index = 0-15 in {
  def Extract_Element_v16i32_#Index : Extract_Element <
    i32, v16i32, Index, !cast<SubRegIndex>(sub#Index)
  >;
  def Insert_Element_v16i32_#Index : Insert_Element <
    i32, v16i32, Index, !cast<SubRegIndex>(sub#Index)
  >;

  def Extract_Element_v16f32_#Index : Extract_Element <
    f32, v16f32, Index, !cast<SubRegIndex>(sub#Index)
  >;
  def Insert_Element_v16f32_#Index : Insert_Element <
    f32, v16f32, Index, !cast<SubRegIndex>(sub#Index)
  >;
}

*/

def : Pat <
  (extract_subvector v4i16:$vec, (i32 0)),
  (v2i16 (EXTRACT_SUBREG v4i16:$vec, sub0))
>;

def : Pat <
  (extract_subvector v4i16:$vec, (i32 2)),
  (v2i16 (EXTRACT_SUBREG v4i16:$vec, sub1))
>;

def : Pat <
  (extract_subvector v4f16:$vec, (i32 0)),
  (v2f16 (EXTRACT_SUBREG v4f16:$vec, sub0))
>;

def : Pat <
  (extract_subvector v4f16:$vec, (i32 2)),
  (v2f16 (EXTRACT_SUBREG v4f16:$vec, sub1))
>;

/*
foreach Index = 0-31 in {
  def Extract_Element_v32i32_#Index : Extract_Element <
    i32, v32i32, Index, !cast<SubRegIndex>(sub#Index)
  >;

  def Insert_Element_v32i32_#Index : Insert_Element <
    i32, v32i32, Index, !cast<SubRegIndex>(sub#Index)
  >;

  def Extract_Element_v32f32_#Index : Extract_Element <
    f32, v32f32, Index, !cast<SubRegIndex>(sub#Index)
  >;

  def Insert_Element_v32f32_#Index : Insert_Element <
    f32, v32f32, Index, !cast<SubRegIndex>(sub#Index)
  >;
}
*/

// FIXME: Why do only some of these type combinations for SReg and
// VReg?
// 16-bit bitcast
def : BitConvert <i16, f16, VPR_32>;
def : BitConvert <f16, i16, VPR_32>;
def : BitConvert <i16, f16, SReg_32>;
def : BitConvert <f16, i16, SReg_32>;

// 32-bit bitcast
def : BitConvert <i32, f32, VPR_32>;
def : BitConvert <f32, i32, VPR_32>;

def : BitConvert <i32, f32, SReg_32>;
def : BitConvert <f32, i32, SReg_32>;
def : BitConvert <v2i16, i32, SReg_32>;
def : BitConvert <i32, v2i16, SReg_32>;
def : BitConvert <v2f16, i32, SReg_32>;
def : BitConvert <i32, v2f16, SReg_32>;
def : BitConvert <v2i16, v2f16, SReg_32>;
def : BitConvert <v2f16, v2i16, SReg_32>;
def : BitConvert <v2f16, f32, SReg_32>;
def : BitConvert <f32, v2f16, SReg_32>;
def : BitConvert <v2i16, f32, SReg_32>;
def : BitConvert <f32, v2i16, SReg_32>;

// 64-bit bitcast
def : BitConvert <i64, f64, VReg_64>;
def : BitConvert <f64, i64, VReg_64>;
def : BitConvert <v2i32, v2f32, VReg_64>;
def : BitConvert <v2f32, v2i32, VReg_64>;
def : BitConvert <i64, v2i32, VReg_64>;
def : BitConvert <v2i32, i64, VReg_64>;
def : BitConvert <i64, v2f32, VReg_64>;
def : BitConvert <v2f32, i64, VReg_64>;
def : BitConvert <f64, v2f32, VReg_64>;
def : BitConvert <v2f32, f64, VReg_64>;
def : BitConvert <f64, v2i32, VReg_64>;
def : BitConvert <v2i32, f64, VReg_64>;
def : BitConvert <v4i16, v4f16, VReg_64>;
def : BitConvert <v4f16, v4i16, VReg_64>;

// FIXME: Make SGPR
def : BitConvert <v2i32, v4f16, VReg_64>;
def : BitConvert <v4f16, v2i32, VReg_64>;
def : BitConvert <v2i32, v4f16, VReg_64>;
def : BitConvert <v2i32, v4i16, VReg_64>;
def : BitConvert <v4i16, v2i32, VReg_64>;
def : BitConvert <v2f32, v4f16, VReg_64>;
def : BitConvert <v4f16, v2f32, VReg_64>;
def : BitConvert <v2f32, v4i16, VReg_64>;
def : BitConvert <v4i16, v2f32, VReg_64>;
def : BitConvert <v4i16, f64, VReg_64>;
def : BitConvert <v4f16, f64, VReg_64>;
def : BitConvert <f64, v4i16, VReg_64>;
def : BitConvert <f64, v4f16, VReg_64>;
def : BitConvert <v4i16, i64, VReg_64>;
def : BitConvert <v4f16, i64, VReg_64>;
def : BitConvert <i64, v4i16, VReg_64>;
def : BitConvert <i64, v4f16, VReg_64>;

/********** =================== **********/
/********** Src & Dst modifiers **********/
/********** =================== **********/


// If denormals are not enabled, it only impacts the compare of the
// inputs. The output result is not flushed.
class ClampPat<Instruction inst, ValueType vt> : PPUPat <
  (vt (PPUclamp (VOP3Mods vt:$src0, i32:$src0_modifiers))),
  (inst i32:$src0_modifiers, vt:$src0,
        i32:$src0_modifiers, vt:$src0, DSTCLAMP.ENABLE, DSTOMOD.NONE)
>;

def : ClampPat<V_MAX_F32_e64, f32>;
// def : ClampPat<V_MAX_F64, f64>;
def : ClampPat<V_MAX_F16_e64, f16>;

let SubtargetPredicate = HasVOP3PInsts in {
def : PPUPat <
  (v2f16 (PPUclamp (VOP3PMods v2f16:$src0, i32:$src0_modifiers))),
  (V_PK_MAX_F16 $src0_modifiers, $src0,
                $src0_modifiers, $src0, DSTCLAMP.ENABLE)
>;
}

/********** ================================ **********/
/********** Floating point absolute/negative **********/
/********** ================================ **********/

// Prevent expanding both fneg and fabs.

def : PPUPat <
  (fneg (fabs f32:$src)),
  (S_OR_B32 $src, (S_MOV_B32(i32 0x80000000))) // Set sign bit
>;

// FIXME: Should use S_OR_B32
def : PPUPat <
  (fneg (fabs f64:$src)),
  (REG_SEQUENCE VReg_64,
    (i32 (EXTRACT_SUBREG f64:$src, sub0)),
    sub0,
    (V_OR_B32_e32 (i32 (EXTRACT_SUBREG f64:$src, sub1)),
                  (V_MOV_B32_e32 (i32 0x80000000))), // Set sign bit.
    sub1)
>;

def : PPUPat <
  (fabs f32:$src),
  (S_AND_B32 $src, (S_MOV_B32 (i32 0x7fffffff)))
>;

def : PPUPat <
  (fneg f32:$src),
  (V_XOR_B32_e32 $src, (V_MOV_B32_e32 (i32 0x80000000)))
>;

def : PPUPat <
  (fabs f64:$src),
  (REG_SEQUENCE VReg_64,
    (i32 (EXTRACT_SUBREG f64:$src, sub0)),
    sub0,
    (V_AND_B32_e64 (i32 (EXTRACT_SUBREG f64:$src, sub1)),
                   (V_MOV_B32_e32 (i32 0x7fffffff))), // Set sign bit.
     sub1)
>;

def : PPUPat <
  (fneg f64:$src),
  (REG_SEQUENCE VReg_64,
    (i32 (EXTRACT_SUBREG f64:$src, sub0)),
    sub0,
    (V_XOR_B32_e32 (i32 (EXTRACT_SUBREG f64:$src, sub1)),
                   (i32 (V_MOV_B32_e32 (i32 0x80000000)))),
    sub1)
>;

def : PPUPat <
  (fcopysign f16:$src0, f16:$src1),
  (V_BFI_B32 (S_MOV_B32 (i32 0x00007fff)), $src0, $src1)
>;

def : PPUPat <
  (fcopysign f32:$src0, f16:$src1),
  (V_BFI_B32 (S_MOV_B32 (i32 0x7fffffff)), $src0,
             (V_LSHLREV_B32_e64 (i32 16), $src1))
>;

def : PPUPat <
  (fcopysign f64:$src0, f16:$src1),
  (REG_SEQUENCE SReg_64,
    (i32 (EXTRACT_SUBREG $src0, sub0)), sub0,
    (V_BFI_B32 (S_MOV_B32 (i32 0x7fffffff)), (i32 (EXTRACT_SUBREG $src0, sub1)),
               (V_LSHLREV_B32_e64 (i32 16), $src1)), sub1)
>;

def : PPUPat <
  (fcopysign f16:$src0, f32:$src1),
  (V_BFI_B32 (S_MOV_B32 (i32 0x00007fff)), $src0,
             (V_LSHRREV_B32_e64 (i32 16), $src1))
>;

def : PPUPat <
  (fcopysign f16:$src0, f64:$src1),
  (V_BFI_B32 (S_MOV_B32 (i32 0x00007fff)), $src0,
             (V_LSHRREV_B32_e64 (i32 16), (EXTRACT_SUBREG $src1, sub1)))
>;

def : PPUPat <
  (fneg f16:$src),
  (S_XOR_B32 $src, (S_MOV_B32 (i32 0x00008000)))
>;

def : PPUPat <
  (fabs f16:$src),
  (S_AND_B32 $src, (S_MOV_B32 (i32 0x00007fff)))
>;

def : PPUPat <
  (fneg (fabs f16:$src)),
  (S_OR_B32 $src, (S_MOV_B32 (i32 0x00008000))) // Set sign bit
>;

def : PPUPat <
  (fneg v2f16:$src),
  (S_XOR_B32 $src, (S_MOV_B32 (i32 0x80008000)))
>;

def : PPUPat <
  (fabs v2f16:$src),
  (S_AND_B32 $src, (S_MOV_B32 (i32 0x7fff7fff)))
>;

// This is really (fneg (fabs v2f16:$src))
//
// fabs is not reported as free because there is modifier for it in
// VOP3P instructions, so it is turned into the bit op.
def : PPUPat <
  (fneg (v2f16 (bitconvert (and_oneuse i32:$src, 0x7fff7fff)))),
  (S_OR_B32 $src, (S_MOV_B32 (i32 0x80008000))) // Set sign bit
>;

def : PPUPat <
  (fneg (v2f16 (fabs v2f16:$src))),
  (S_OR_B32 $src, (S_MOV_B32 (i32 0x80008000))) // Set sign bit
>;

/********** ================== **********/
/********** Immediate Patterns **********/
/********** ================== **********/

def : PPUPat <
  (VGPRImm<(i32 imm)>:$imm),
  (V_MOV_B32_e32 imm:$imm)
>;

def : PPUPat <
  (VGPRImm<(f32 fpimm)>:$imm),
  (V_MOV_B32_e32 (f32 (bitcast_fpimm_to_i32 $imm)))
>;

def : PPUPat <
  (i32 imm:$imm),
  (S_MOV_B32 imm:$imm)
>;

def : PPUPat <
  (VGPRImm<(SIlds tglobaladdr:$ga)>),
  (V_MOV_B32_e32 $ga)
>;

def : PPUPat <
  (SIlds tglobaladdr:$ga),
  (S_MOV_B32 $ga)
>;

// FIXME: Workaround for ordering issue with peephole optimizer where
// a register class copy interferes with immediate folding.  Should
// use s_mov_b32, which can be shrunk to s_movk_i32
def : PPUPat <
  (VGPRImm<(f16 fpimm)>:$imm),
  (V_MOV_B32_e32 (f16 (bitcast_fpimm_to_i32 $imm)))
>;

def : PPUPat <
  (f32 fpimm:$imm),
  (S_MOV_B32 (f32 (bitcast_fpimm_to_i32 $imm)))
>;

def : PPUPat <
  (f16 fpimm:$imm),
  (S_MOV_B32 (i32 (bitcast_fpimm_to_i32 $imm)))
>;

def : PPUPat <
 (i32 frameindex:$fi),
 (V_MOV_B32_e32 (i32 (frameindex_to_targetframeindex $fi)))
>;

def : PPUPat <
  (i64 InlineImm<i64>:$imm),
  (S_MOV_B64 InlineImm<i64>:$imm)
>;

// XXX - Should this use a s_cmp to set SCC?

def : PPUPat <
  (i1 imm:$imm),
  (S_MOV_B32 (i32 (as_i32imm $imm)))
>;

def : PPUPat <
  (f64 InlineFPImm<f64>:$imm),
  (S_MOV_B64 (f64 (bitcast_fpimm_to_i64 InlineFPImm<f64>:$imm)))
>;

/********** ================== **********/
/********** Intrinsic Patterns **********/
/********** ================== **********/

// def : POW_Common <V_LOG_F32_e32, V_EXP_F32_e32, V_MUL_LEGACY_F32_e32>;

def : PPUPat <
  (i32 (sext i1:$src0)),
  (V_CNDMASK_B32_e64 /*src0mod*/(i32 0), /*src0*/(i32 0),
                     /*src1mod*/(i32 0), /*src1*/(i32 -1), $src0)
>;

class Ext32Pat <SDNode ext> : PPUPat <
  (i32 (ext i1:$src0)),
  (V_CNDMASK_B32_e64 /*src0mod*/(i32 0), /*src0*/(i32 0),
                     /*src1mod*/(i32 0), /*src1*/(i32 1), $src0)
>;

def : Ext32Pat <zext>;
def : Ext32Pat <anyext>;

// The multiplication scales from [0,1] to the unsigned integer range
def : PPUPat <
  (PPUurecip i32:$src0),
  (V_CVT_U32_F32_e32
    (V_MUL_F32_e32 (i32 CONST.FP_UINT_MAX_PLUS_1),
                   (V_RCP_IFLAG_F32_e32 (V_CVT_F32_U32_e32 $src0))))
>;

//===----------------------------------------------------------------------===//
// VOP3 Patterns
//===----------------------------------------------------------------------===//

def : IMad24Pat<V_MAD_I32_I24, 1>;
def : UMad24Pat<V_MAD_U32_U24, 1>;

// FIXME: This should only be done for VALU inputs
defm : BFIPatterns <V_BFI_B32, S_MOV_B32, SReg_64>;
def : ROTRPattern <V_ALIGNBIT_B32>;

def : PPUPat<(i32 (trunc (srl i64:$src0, (and i32:$src1, (i32 31))))),
          (V_ALIGNBIT_B32 (i32 (EXTRACT_SUBREG (i64 $src0), sub1)),
                          (i32 (EXTRACT_SUBREG (i64 $src0), sub0)), $src1)>;

def : PPUPat<(i32 (trunc (srl i64:$src0, (i32 ShiftAmt32Imm:$src1)))),
          (V_ALIGNBIT_B32 (i32 (EXTRACT_SUBREG (i64 $src0), sub1)),
                          (i32 (EXTRACT_SUBREG (i64 $src0), sub0)), $src1)>;

/********** ====================== **********/
/**********   Indirect addressing  **********/
/********** ====================== **********/

multiclass SI_INDIRECT_Pattern <ValueType vt, ValueType eltvt, string VecSize> {
  // Extract with offset
  def : PPUPat<
    (eltvt (extractelt vt:$src, (MOVRELOffset i32:$idx, (i32 imm:$offset)))),
    (!cast<Instruction>("SI_INDIRECT_SRC_"#VecSize) $src, $idx, imm:$offset)
  >;

  // Insert with offset
  def : PPUPat<
    (insertelt vt:$src, eltvt:$val, (MOVRELOffset i32:$idx, (i32 imm:$offset))),
    (!cast<Instruction>("SI_INDIRECT_DST_"#VecSize) $src, $idx, imm:$offset, $val)
  >;
}

defm : SI_INDIRECT_Pattern <v2f32, f32, "V2">;
// defm : SI_INDIRECT_Pattern <v4f32, f32, "V4">;
// defm : SI_INDIRECT_Pattern <v8f32, f32, "V8">;
// defm : SI_INDIRECT_Pattern <v16f32, f32, "V16">;

defm : SI_INDIRECT_Pattern <v2i32, i32, "V2">;
//defm : SI_INDIRECT_Pattern <v4i32, i32, "V4">;
// defm : SI_INDIRECT_Pattern <v8i32, i32, "V8">;
// defm : SI_INDIRECT_Pattern <v16i32, i32, "V16">;

//===----------------------------------------------------------------------===//
// SAD Patterns
//===----------------------------------------------------------------------===//

def : PPUPat <
  (add (sub_oneuse (umax i32:$src0, i32:$src1),
                   (umin i32:$src0, i32:$src1)),
       i32:$src2),
  (V_SAD_U32 $src0, $src1, $src2, (i1 0))
>;

def : PPUPat <
  (add (select_oneuse (i1 (setugt i32:$src0, i32:$src1)),
                      (sub i32:$src0, i32:$src1),
                      (sub i32:$src1, i32:$src0)),
       i32:$src2),
  (V_SAD_U32 $src0, $src1, $src2, (i1 0))
>;

//===----------------------------------------------------------------------===//
// Conversion Patterns
//===----------------------------------------------------------------------===//

def : PPUPat<(i32 (sext_inreg i32:$src, i1)),
  (S_BFE_I32 i32:$src, (i32 65536))>; // 0 | 1 << 16

// Handle sext_inreg in i64
def : PPUPat <
  (i64 (sext_inreg i64:$src, i1)),
  (S_BFE_I64 i64:$src, (i32 0x10000)) // 0 | 1 << 16
>;

def : PPUPat <
  (i16 (sext_inreg i16:$src, i1)),
  (S_BFE_I32 $src, (i32 0x00010000)) // 0 | 1 << 16
>;

def : PPUPat <
  (i16 (sext_inreg i16:$src, i8)),
  (S_BFE_I32 $src, (i32 0x80000)) // 0 | 8 << 16
>;

def : PPUPat <
  (i64 (sext_inreg i64:$src, i8)),
  (S_BFE_I64 i64:$src, (i32 0x80000)) // 0 | 8 << 16
>;

def : PPUPat <
  (i64 (sext_inreg i64:$src, i16)),
  (S_BFE_I64 i64:$src, (i32 0x100000)) // 0 | 16 << 16
>;

def : PPUPat <
  (i64 (sext_inreg i64:$src, i32)),
  (S_BFE_I64 i64:$src, (i32 0x200000)) // 0 | 32 << 16
>;

def : PPUPat <
  (i64 (zext i32:$src)),
  (REG_SEQUENCE SReg_64, $src, sub0, (S_MOV_B32 (i32 0)), sub1)
>;

def : PPUPat <
  (i64 (anyext i32:$src)),
  (REG_SEQUENCE SReg_64, $src, sub0, (i32 (IMPLICIT_DEF)), sub1)
>;

class ZExt_i64_i1_Pat <SDNode ext> : PPUPat <
  (i64 (ext i1:$src)),
    (REG_SEQUENCE VReg_64,
      (V_CNDMASK_B32_e64 /*src0mod*/(i32 0), /*src0*/(i32 0),
                         /*src1mod*/(i32 0), /*src1*/(i32 1), $src),
      sub0, (S_MOV_B32 (i32 0)), sub1)
>;


def : ZExt_i64_i1_Pat<zext>;
def : ZExt_i64_i1_Pat<anyext>;

// FIXME: We need to use COPY_TO_REGCLASS to work-around the fact that
// REG_SEQUENCE patterns don't support instructions with multiple outputs.
def : PPUPat <
  (i64 (sext i32:$src)),
    (REG_SEQUENCE SReg_64, $src, sub0,
    (i32 (COPY_TO_REGCLASS (S_ASHR_I32 $src, (i32 31)), SReg_32)), sub1)
>;

def : PPUPat <
  (i64 (sext i1:$src)),
  (REG_SEQUENCE VReg_64,
    (V_CNDMASK_B32_e64 /*src0mod*/(i32 0), /*src0*/(i32 0),
                       /*src1mod*/(i32 0), /*src1*/(i32 -1), $src), sub0,
    (V_CNDMASK_B32_e64 /*src0mod*/(i32 0), /*src0*/(i32 0),
                       /*src1mod*/(i32 0), /*src1*/(i32 -1), $src), sub1)
>;

class FPToI1Pat<Instruction Inst, int KOne, ValueType kone_type, ValueType vt, SDPatternOperator fp_to_int> : PPUPat <
  (i1 (fp_to_int (vt (VOP3Mods vt:$src0, i32:$src0_modifiers)))),
  (i1 (Inst 0, (kone_type KOne), $src0_modifiers, $src0, DSTCLAMP.NONE))
>;

def : FPToI1Pat<V_CMP_EQ_F32_e64, CONST.FP32_ONE, i32, f32, fp_to_uint>;
def : FPToI1Pat<V_CMP_EQ_F32_e64, CONST.FP32_NEG_ONE, i32, f32, fp_to_sint>;
def : FPToI1Pat<V_CMP_EQ_F64_e64, CONST.FP64_ONE, i64, f64, fp_to_uint>;
def : FPToI1Pat<V_CMP_EQ_F64_e64, CONST.FP64_NEG_ONE, i64, f64, fp_to_sint>;

// If we need to perform a logical operation on i1 values, we need to
// use vector comparisons since there is only one SCC register. Vector
// comparisons may write to a pair of SGPRs or a single SGPR, so treat
// these as 32 or 64-bit comparisons. When legalizing SGPR copies,
// instructions resulting in the copies from SCC to these instructions
// will be moved to the VALU.

def : PPUPat <
  (i1 (and i1:$src0, i1:$src1)),
  (S_AND_B32 $src0, $src1)
>;

def : PPUPat <
  (i1 (or i1:$src0, i1:$src1)),
  (S_OR_B32 $src0, $src1)
>;

def : PPUPat <
  (i1 (xor i1:$src0, i1:$src1)),
  (S_XOR_B32 $src0, $src1)
>;

def : PPUPat <
  (i1 (add i1:$src0, i1:$src1)),
  (S_XOR_B32 $src0, $src1)
>;

def : PPUPat <
  (i1 (sub i1:$src0, i1:$src1)),
  (S_XOR_B32 $src0, $src1)
>;

let AddedComplexity = 1 in {
def : PPUPat <
  (i1 (add i1:$src0, (i1 -1))),
  (S_NOT_B32 $src0)
>;

def : PPUPat <
  (i1 (sub i1:$src0, (i1 -1))),
  (S_NOT_B32 $src0)
>;
}

def : PPUPat <
  (f16 (sint_to_fp i1:$src)),
  (V_CVT_F16_F32_e32 (
      V_CNDMASK_B32_e64 /*src0mod*/(i32 0), /*src0*/(i32 0),
                        /*src1mod*/(i32 0), /*src1*/(i32 CONST.FP32_NEG_ONE),
                        $src))
>;

def : PPUPat <
  (f16 (uint_to_fp i1:$src)),
  (V_CVT_F16_F32_e32 (
      V_CNDMASK_B32_e64 /*src0mod*/(i32 0), /*src0*/(i32 0),
                        /*src1mod*/(i32 0), /*src1*/(i32 CONST.FP32_ONE),
                        $src))
>;

def : PPUPat <
  (f32 (sint_to_fp i1:$src)),
  (V_CNDMASK_B32_e64 /*src0mod*/(i32 0), /*src0*/(i32 0),
                        /*src1mod*/(i32 0), /*src1*/(i32 CONST.FP32_NEG_ONE),
                        $src)
>;

def : PPUPat <
  (f32 (uint_to_fp i1:$src)),
  (V_CNDMASK_B32_e64 /*src0mod*/(i32 0), /*src0*/(i32 0),
                        /*src1mod*/(i32 0), /*src1*/(i32 CONST.FP32_ONE),
                        $src)
>;
/*
def : PPUPat <
  (f64 (sint_to_fp i1:$src)),
  (V_CVT_F64_I32_e32 (V_CNDMASK_B32_e64 /*src0mod*/(i32 0), /*src0*/(i32 0),
                                        /*src1mod*/(i32 0), /*src1*/(i32 -1),
                                        $src))
>;

def : PPUPat <
  (f64 (uint_to_fp i1:$src)),
  (V_CVT_F64_U32_e32 (V_CNDMASK_B32_e64 /*src0mod*/(i32 0), /*src0*/(i32 0),
                                        /*src1mod*/(i32 0), /*src1*/(i32 1),
                                        $src))
>;
*/

//===----------------------------------------------------------------------===//
// Miscellaneous Patterns
//===----------------------------------------------------------------------===//
def : PPUPat <
  (i32 (PPUfp16_zext f16:$src)),
  (COPY $src)
>;


def : PPUPat <
  (i32 (trunc i64:$a)),
  (EXTRACT_SUBREG $a, sub0)
>;

def : PPUPat <
  (i1 (trunc i32:$a)),
  (V_CMP_EQ_U32_e64 (S_AND_B32 (i32 1), $a), (i32 1))
>;

def : PPUPat <
  (i1 (trunc i16:$a)),
  (V_CMP_EQ_U32_e64 (S_AND_B32 (i32 1), $a), (i32 1))
>;

def : PPUPat <
  (i1 (trunc i64:$a)),
  (V_CMP_EQ_U32_e64 (S_AND_B32 (i32 1),
                    (i32 (EXTRACT_SUBREG $a, sub0))), (i32 1))
>;

def : PPUPat <
  (i32 (bswap i32:$a)),
  (V_BFI_B32 (S_MOV_B32 (i32 0x00ff00ff)),
             (V_ALIGNBIT_B32 $a, $a, (i32 24)),
             (V_ALIGNBIT_B32 $a, $a, (i32 8)))
>;

let OtherPredicates = [NoFP16Denormals] in {
def : PPUPat<
  (fcanonicalize (f16 (VOP3Mods f16:$src, i32:$src_mods))),
  (V_MUL_F16_e64 0, (i32 CONST.FP16_ONE), $src_mods, $src, 0, 0)
>;

def : PPUPat<
  (fcanonicalize (f16 (fneg (VOP3Mods f16:$src, i32:$src_mods)))),
  (V_MUL_F16_e64 0, (i32 CONST.FP16_NEG_ONE), $src_mods, $src, 0, 0)
>;

def : PPUPat<
  (fcanonicalize (v2f16 (VOP3PMods v2f16:$src, i32:$src_mods))),
  (V_PK_MUL_F16 0, (i32 CONST.FP16_ONE), $src_mods, $src, DSTCLAMP.NONE)
>;
}

let OtherPredicates = [FP16Denormals] in {
def : PPUPat<
  (fcanonicalize (f16 (VOP3Mods f16:$src, i32:$src_mods))),
  (V_MAX_F16_e64 $src_mods, $src, $src_mods, $src, 0, 0)
>;

let SubtargetPredicate = HasVOP3PInsts in {
def : PPUPat<
  (fcanonicalize (v2f16 (VOP3PMods v2f16:$src, i32:$src_mods))),
  (V_PK_MAX_F16 $src_mods, $src, $src_mods, $src, DSTCLAMP.NONE)
>;
}
}

let OtherPredicates = [NoFP32Denormals] in {
def : PPUPat<
  (fcanonicalize (f32 (VOP3Mods f32:$src, i32:$src_mods))),
  (V_MUL_F32_e64 0, (i32 CONST.FP32_ONE), $src_mods, $src, 0, 0)
>;

def : PPUPat<
  (fcanonicalize (f32 (fneg (VOP3Mods f32:$src, i32:$src_mods)))),
  (V_MUL_F32_e64 0, (i32 CONST.FP32_NEG_ONE), $src_mods, $src, 0, 0)
>;
}

let OtherPredicates = [FP32Denormals] in {
def : PPUPat<
  (fcanonicalize (f32 (VOP3Mods f32:$src, i32:$src_mods))),
  (V_MAX_F32_e64 $src_mods, $src, $src_mods, $src, 0, 0)
>;
}

/*
let OtherPredicates = [NoFP64Denormals] in {
def : PPUPat<
  (fcanonicalize (f64 (VOP3Mods f64:$src, i32:$src_mods))),
  (V_MUL_F64 0, CONST.FP64_ONE, $src_mods, $src, 0, 0)
>;
}

let OtherPredicates = [FP64Denormals] in {
def : PPUPat<
  (fcanonicalize (f64 (VOP3Mods f64:$src, i32:$src_mods))),
  (V_MAX_F64 $src_mods, $src, $src_mods, $src, 0, 0)
>;
}
*/
/*
let OtherPredicates = [HasDLInsts] in {
def : PPUPat <
  (fma (f32 (VOP3Mods0 f32:$src0, i32:$src0_modifiers, i1:$clamp, i32:$omod)),
       (f32 (VOP3Mods f32:$src1, i32:$src1_modifiers)),
       (f32 (VOP3NoMods f32:$src2))),
  (V_FMAC_F32_e64 $src0_modifiers, $src0, $src1_modifiers, $src1,
                  SRCMODS.NONE, $src2, $clamp, $omod)
>;
} // End OtherPredicates = [HasDLInsts]
*/

let SubtargetPredicate = isPPT in
def : PPUPat <
  (fma (f16 (VOP3Mods0 f32:$src0, i32:$src0_modifiers, i1:$clamp, i32:$omod)),
       (f16 (VOP3Mods f32:$src1, i32:$src1_modifiers)),
       (f16 (VOP3NoMods f32:$src2))),
  (V_FMAC_F16_e64 $src0_modifiers, $src0, $src1_modifiers, $src1,
                  SRCMODS.NONE, $src2, $clamp, $omod)
>;

// Allow integer inputs
/*
class ExpPattern<SDPatternOperator node, ValueType vt, Instruction Inst> : PPUPat<
  (node (i8 timm:$tgt), (i8 timm:$en), vt:$src0, vt:$src1, vt:$src2, vt:$src3, (i1 timm:$compr), (i1 timm:$vm)),
  (Inst i8:$tgt, vt:$src0, vt:$src1, vt:$src2, vt:$src3, i1:$vm, i1:$compr, i8:$en)
>;

def : ExpPattern<PPUexport, i32, EXP>;
def : ExpPattern<PPUexport_done, i32, EXP_DONE>;
*/

// COPY is workaround tablegen bug from multiple outputs
// from S_LSHL_B32's multiple outputs from implicit scc def.
def : PPUPat <
  (v2i16 (build_vector (i16 0), i16:$src1)),
  (v2i16 (COPY (S_LSHL_B32 i16:$src1, (i16 16))))
>;

def : PPUPat <
  (v2i16 (build_vector i16:$src0, (i16 undef))),
  (v2i16 (COPY $src0))
>;

def : PPUPat <
  (v2f16 (build_vector f16:$src0, (f16 undef))),
  (v2f16 (COPY $src0))
>;

def : PPUPat <
  (v2i16 (build_vector (i16 undef), i16:$src1)),
  (v2i16 (COPY (S_LSHL_B32 $src1, (i32 16))))
>;

def : PPUPat <
  (v2f16 (build_vector (f16 undef), f16:$src1)),
  (v2f16 (COPY (S_LSHL_B32 $src1, (i32 16))))
>;

let SubtargetPredicate = HasVOP3PInsts in {
def : PPUPat <
  (v2i16 (build_vector i16:$src0, i16:$src1)),
  (v2i16 (S_PACK_LL_B32_B16 $src0, $src1))
>;

// With multiple uses of the shift, this will duplicate the shift and
// increase register pressure.
def : PPUPat <
  (v2i16 (build_vector i16:$src0, (i16 (trunc (srl_oneuse i32:$src1, (i32 16)))))),
  (v2i16 (S_PACK_LH_B32_B16 i16:$src0, i32:$src1))
>;


def : PPUPat <
  (v2i16 (build_vector (i16 (trunc (srl_oneuse i32:$src0, (i32 16)))),
                       (i16 (trunc (srl_oneuse i32:$src1, (i32 16)))))),
  (v2i16 (S_PACK_HH_B32_B16 $src0, $src1))
>;

// TODO: Should source modifiers be matched to v_pack_b32_f16?
def : PPUPat <
  (v2f16 (build_vector f16:$src0, f16:$src1)),
  (v2f16 (S_PACK_LL_B32_B16 $src0, $src1))
>;

} // End SubtargetPredicate = HasVOP3PInsts


def : PPUPat <
  (v2f16 (scalar_to_vector f16:$src0)),
  (COPY $src0)
>;

def : PPUPat <
  (v2i16 (scalar_to_vector i16:$src0)),
  (COPY $src0)
>;

def : PPUPat <
  (v4i16 (scalar_to_vector i16:$src0)),
  (INSERT_SUBREG (IMPLICIT_DEF), $src0, sub0)
>;

def : PPUPat <
  (v4f16 (scalar_to_vector f16:$src0)),
  (INSERT_SUBREG (IMPLICIT_DEF), $src0, sub0)
>;

//============================================================================//
// Miscellaneous Optimization Patterns
//============================================================================//

// Undo sub x, c -> add x, -c canonicalization since c is more likely
// an inline immediate than -c.
// TODO: Also do for 64-bit.
def : PPUPat<
  (add i32:$src0, (i32 NegSubInlineConst32:$src1)),
  (S_SUB_I32 $src0, NegSubInlineConst32:$src1)
>;

// Avoid pointlessly materializing a constant in VGPR.
// FIXME: Should also do this for readlane, but tablegen crashes on
// the ignored src1.
def : PPUPat<
  (int_ppu_readfirstlane (i32 imm:$src)),
  (S_MOV_B32 $src)
>;

multiclass BFMPatterns <ValueType vt, PPTInst BFM, PPTInst MOV> {
  def : PPUPat <
    (vt (shl (vt (add (vt (shl 1, vt:$a)), -1)), vt:$b)),
    (BFM $a, $b)
  >;

  def : PPUPat <
    (vt (add (vt (shl 1, vt:$a)), -1)),
    (BFM $a, (MOV (i32 0)))
  >;
}

defm : BFMPatterns <i32, S_BFM_B32, S_MOV_B32>;
// FIXME: defm : BFMPatterns <i64, S_BFM_B64, S_MOV_B64>;

defm : BFEPattern <V_BFE_U32, V_BFE_I32, S_MOV_B32>;
defm : SHA256MaPattern <V_BFI_B32, V_XOR_B32_e64, SReg_64>;

defm : IntMed3Pat<V_MED3_I32, smin, smax, smin_oneuse, smax_oneuse>;
defm : IntMed3Pat<V_MED3_U32, umin, umax, umin_oneuse, umax_oneuse>;

// This matches 16 permutations of
// max(min(x, y), min(max(x, y), z))
class FPMed3Pat<ValueType vt,
                //SDPatternOperator max, SDPatternOperator min,
                Instruction med3Inst> : PPUPat<
  (fmaxnum_like (fminnum_like_oneuse (VOP3Mods_nnan vt:$src0, i32:$src0_mods),
                           (VOP3Mods_nnan vt:$src1, i32:$src1_mods)),
           (fminnum_like_oneuse (fmaxnum_like_oneuse (VOP3Mods_nnan vt:$src0, i32:$src0_mods),
                                           (VOP3Mods_nnan vt:$src1, i32:$src1_mods)),
                           (vt (VOP3Mods_nnan vt:$src2, i32:$src2_mods)))),
  (med3Inst $src0_mods, $src0, $src1_mods, $src1, $src2_mods, $src2, DSTCLAMP.NONE, DSTOMOD.NONE)
>;

class FP16Med3Pat<ValueType vt,
                Instruction med3Inst> : PPUPat<
  (fmaxnum_like (fminnum_like_oneuse (VOP3Mods_nnan vt:$src0, i32:$src0_mods),
                                     (VOP3Mods_nnan vt:$src1, i32:$src1_mods)),
           (fminnum_like_oneuse (fmaxnum_like_oneuse (VOP3Mods_nnan vt:$src0, i32:$src0_mods),
                                                     (VOP3Mods_nnan vt:$src1, i32:$src1_mods)),
                           (vt (VOP3Mods_nnan vt:$src2, i32:$src2_mods)))),
  (med3Inst $src0_mods, $src0, $src1_mods, $src1, $src2_mods, $src2, DSTCLAMP.NONE)
>;

multiclass Int16Med3Pat<Instruction med3Inst,
                   SDPatternOperator min,
                   SDPatternOperator max,
                   SDPatternOperator max_oneuse,
                   SDPatternOperator min_oneuse,
                   ValueType vt = i16> {
  // This matches 16 permutations of
  // max(min(x, y), min(max(x, y), z))
  def : PPUPat <
  (max (min_oneuse vt:$src0, vt:$src1),
       (min_oneuse (max_oneuse vt:$src0, vt:$src1), vt:$src2)),
  (med3Inst SRCMODS.NONE, $src0, SRCMODS.NONE, $src1, SRCMODS.NONE, $src2, DSTCLAMP.NONE)
>;

  // This matches 16 permutations of
  // min(max(a, b), max(min(a, b), c))
  def : PPUPat <
  (min (max_oneuse vt:$src0, vt:$src1),
      (max_oneuse (min_oneuse vt:$src0, vt:$src1), vt:$src2)),
  (med3Inst SRCMODS.NONE, $src0, SRCMODS.NONE, $src1, SRCMODS.NONE, $src2, DSTCLAMP.NONE)
>;
}
/*
def : FPMed3Pat<f32, V_MED3_F32>;

let OtherPredicates = [isGFX9Plus] in {
def : FP16Med3Pat<f16, V_MED3_F16>;
defm : Int16Med3Pat<V_MED3_I16, smin, smax, smax_oneuse, smin_oneuse>;
defm : Int16Med3Pat<V_MED3_U16, umin, umax, umax_oneuse, umin_oneuse>;
} // End Predicates = [isGFX9Plus]
*/

