//===- IntrinsicsPPU.td - Defines PPU intrinsics -------*- tablegen -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines all of the PPU-specific intrinsics.
//
//===----------------------------------------------------------------------===//

class PPUReadPreloadRegisterIntrinsic
  : Intrinsic<[llvm_i32_ty], [], [IntrNoMem, IntrSpeculatable]>;

class PPUReadPreloadRegisterIntrinsicNamed<string name>
  : Intrinsic<[llvm_i32_ty], [], [IntrNoMem, IntrSpeculatable]>, GCCBuiltin<name>;

// Used to tag image and resource intrinsics with information used to generate
// mem operands.
class PPURsrcIntrinsic<int rsrcarg, bit isimage = 0> {
  int RsrcArg = rsrcarg;
  bit IsImage = isimage;
}


multiclass PPUReadPreloadRegisterIntrinsic_xyz {
  def _x : PPUReadPreloadRegisterIntrinsic;
  def _y : PPUReadPreloadRegisterIntrinsic;
  def _z : PPUReadPreloadRegisterIntrinsic;
}

multiclass PPUReadPreloadRegisterIntrinsic_xyz_named<string prefix> {
  def _x : PPUReadPreloadRegisterIntrinsicNamed<!strconcat(prefix, "_x")>;
  def _y : PPUReadPreloadRegisterIntrinsicNamed<!strconcat(prefix, "_y")>;
  def _z : PPUReadPreloadRegisterIntrinsicNamed<!strconcat(prefix, "_z")>;
}

let TargetPrefix = "ppu" in {

//===----------------------------------------------------------------------===//
// Atomics

class PPUMaskedAtomicRMW32Intrinsic
    : Intrinsic<[llvm_i32_ty],
                [llvm_anyptr_ty, llvm_i32_ty, llvm_i32_ty, llvm_i32_ty],
                [IntrArgMemOnly, NoCapture<0>, ImmArg<3>]>;

class PPUMaskedAtomicRMW32WithSextIntrinsic
    : Intrinsic<[llvm_i32_ty],
                [llvm_anyptr_ty, llvm_i32_ty, llvm_i32_ty, llvm_i32_ty,
                 llvm_i32_ty],
                [IntrArgMemOnly, NoCapture<0>, ImmArg<4>]>;

def int_ppu_masked_atomicrmw_xchg_i32 : PPUMaskedAtomicRMW32Intrinsic;
def int_ppu_masked_atomicrmw_add_i32  : PPUMaskedAtomicRMW32Intrinsic;
def int_ppu_masked_atomicrmw_sub_i32  : PPUMaskedAtomicRMW32Intrinsic;
def int_ppu_masked_atomicrmw_nand_i32 : PPUMaskedAtomicRMW32Intrinsic;
def int_ppu_masked_atomicrmw_max_i32  : PPUMaskedAtomicRMW32WithSextIntrinsic;
def int_ppu_masked_atomicrmw_min_i32  : PPUMaskedAtomicRMW32WithSextIntrinsic;
def int_ppu_masked_atomicrmw_umax_i32 : PPUMaskedAtomicRMW32Intrinsic;
def int_ppu_masked_atomicrmw_umin_i32 : PPUMaskedAtomicRMW32Intrinsic;

def int_ppu_masked_cmpxchg_i32
    : Intrinsic<[llvm_i32_ty], [llvm_anyptr_ty, llvm_i32_ty, llvm_i32_ty,
                                llvm_i32_ty, llvm_i32_ty],
                [IntrArgMemOnly, NoCapture<0>, ImmArg<4>]>;

class PPUMaskedAtomicRMW64Intrinsic
    : Intrinsic<[llvm_i64_ty],
                [llvm_anyptr_ty, llvm_i64_ty, llvm_i64_ty, llvm_i64_ty],
                [IntrArgMemOnly, NoCapture<0>, ImmArg<3>]>;

class PPUMaskedAtomicRMW64WithSextIntrinsic
    : Intrinsic<[llvm_i64_ty],
                [llvm_anyptr_ty, llvm_i64_ty, llvm_i64_ty, llvm_i64_ty,
                 llvm_i64_ty],
                [IntrArgMemOnly, NoCapture<0>, ImmArg<4>]>;

def int_ppu_masked_atomicrmw_xchg_i64 : PPUMaskedAtomicRMW64Intrinsic;
def int_ppu_masked_atomicrmw_add_i64  : PPUMaskedAtomicRMW64Intrinsic;
def int_ppu_masked_atomicrmw_sub_i64  : PPUMaskedAtomicRMW64Intrinsic;
def int_ppu_masked_atomicrmw_nand_i64 : PPUMaskedAtomicRMW64Intrinsic;
def int_ppu_masked_atomicrmw_max_i64  : PPUMaskedAtomicRMW64WithSextIntrinsic;
def int_ppu_masked_atomicrmw_min_i64  : PPUMaskedAtomicRMW64WithSextIntrinsic;
def int_ppu_masked_atomicrmw_umax_i64 : PPUMaskedAtomicRMW64Intrinsic;
def int_ppu_masked_atomicrmw_umin_i64 : PPUMaskedAtomicRMW64Intrinsic;

def int_ppu_masked_cmpxchg_i64
    : Intrinsic<[llvm_i64_ty], [llvm_anyptr_ty, llvm_i64_ty, llvm_i64_ty,
                                llvm_i64_ty, llvm_i64_ty],
                [IntrArgMemOnly, NoCapture<0>, ImmArg<4>]>;

//===----------------------------------------------------------------------===//
// Vector extension

def int_ppu_setvl : Intrinsic<[llvm_i32_ty], [llvm_i32_ty], [IntrNoMem]>;

def int_ppu_vadd : Intrinsic<[llvm_nxv1i32_ty],
                               [llvm_nxv1i32_ty, llvm_nxv1i32_ty, llvm_i32_ty],
                               [IntrNoMem]>;

def int_ppu_vaddi : Intrinsic<[llvm_nxv1i32_ty],
                               [llvm_nxv1i32_ty, llvm_nxv1i32_ty, llvm_i32_ty],
                               [IntrNoMem]>;

def int_ppu_vsub : Intrinsic<[llvm_nxv1i32_ty],
                               [llvm_nxv1i32_ty, llvm_nxv1i32_ty, llvm_i32_ty],
                               [IntrNoMem]>;


def int_ppu_vmul : Intrinsic<[llvm_nxv1i32_ty],
                               [llvm_nxv1i32_ty, llvm_nxv1i32_ty, llvm_i32_ty],
                               [IntrNoMem]>;


def int_ppu_vand : Intrinsic<[llvm_nxv1i32_ty],
                               [llvm_nxv1i32_ty, llvm_nxv1i32_ty, llvm_i32_ty],
                               [IntrNoMem]>;


def int_ppu_vor : Intrinsic<[llvm_nxv1i32_ty],
                               [llvm_nxv1i32_ty, llvm_nxv1i32_ty, llvm_i32_ty],
                               [IntrNoMem]>;

def int_ppu_vxor : Intrinsic<[llvm_nxv1i32_ty],
                               [llvm_nxv1i32_ty, llvm_nxv1i32_ty, llvm_i32_ty],
                               [IntrNoMem]>;

def int_ppu_vlw : Intrinsic<[llvm_nxv1i32_ty],
                              [llvm_ptr32_ty, llvm_i32_ty],
                              [IntrReadMem]>;
def int_ppu_vsw : Intrinsic<[],
                              [llvm_ptr32_ty, llvm_nxv1i32_ty, llvm_i32_ty],
                              [IntrWriteMem]>;

def int_ppu_vmpopcnt : Intrinsic<[llvm_i32_ty],
                               [llvm_nxv1i32_ty, llvm_i32_ty],
                               [IntrNoMem]>;

def int_ppu_vmfirst : Intrinsic<[llvm_i32_ty],
                               [llvm_nxv1i32_ty, llvm_i32_ty],
                               [IntrNoMem]>;


//===----------------------------------------------------------------------===//
// ABI Special Intrinsics
//===----------------------------------------------------------------------===//

defm int_ppu_workitem_id : PPUReadPreloadRegisterIntrinsic_xyz;
defm int_ppu_workgroup_id : PPUReadPreloadRegisterIntrinsic_xyz_named<"__builtin_ppu_workgroup_id">;
defm int_ppu_workgroup_size : PPUReadPreloadRegisterIntrinsic_xyz_named<"__builtin_ppu_workgroup_size">;
defm int_ppu_workitem_size : PPUReadPreloadRegisterIntrinsic_xyz_named<"__builtin_ppu_workitem_size">;
defm int_ppu_global_size : PPUReadPreloadRegisterIntrinsic_xyz_named<"__builtin_ppu_global_size">;
defm int_ppu_local_size : PPUReadPreloadRegisterIntrinsic_xyz_named<"__builtin_ppu_local_size">;

def int_ppu_dispatch_ptr :
  GCCBuiltin<"__builtin_ppu_dispatch_ptr">,
  Intrinsic<[LLVMQualPointerType<llvm_i8_ty, 4>], [],
  [IntrNoMem, IntrSpeculatable]>;

def int_ppu_queue_ptr :
  GCCBuiltin<"__builtin_ppu_queue_ptr">,
  Intrinsic<[LLVMQualPointerType<llvm_i8_ty, 4>], [],
  [IntrNoMem, IntrSpeculatable]>;

def int_ppu_kernarg_segment_ptr :
  GCCBuiltin<"__builtin_ppu_kernarg_segment_ptr">,
  Intrinsic<[LLVMQualPointerType<llvm_i8_ty, 4>], [],
  [IntrNoMem, IntrSpeculatable]>;

def int_ppu_implicitarg_ptr :
  GCCBuiltin<"__builtin_ppu_implicitarg_ptr">,
  Intrinsic<[LLVMQualPointerType<llvm_i8_ty, 4>], [],
  [IntrNoMem, IntrSpeculatable]>;

def int_ppu_groupstaticsize :
  GCCBuiltin<"__builtin_ppu_groupstaticsize">,
  Intrinsic<[llvm_i32_ty], [], [IntrNoMem, IntrSpeculatable]>;

def int_ppu_dispatch_id :
  GCCBuiltin<"__builtin_ppu_dispatch_id">,
  Intrinsic<[llvm_i64_ty], [], [IntrNoMem, IntrSpeculatable]>;

// Fields should mirror atomicrmw
class PPUAtomicIncIntrin : Intrinsic<[llvm_anyint_ty],
  [llvm_anyptr_ty,
  LLVMMatchType<0>,
  llvm_i32_ty, // ordering
  llvm_i32_ty, // scope
  llvm_i1_ty], // isVolatile
  [IntrArgMemOnly, NoCapture<0>, ImmArg<2>, ImmArg<3>, ImmArg<4>], "",
  [SDNPMemOperand]
>;

def int_ppu_atomic_inc : PPUAtomicIncIntrin;
def int_ppu_atomic_dec : PPUAtomicIncIntrin;

class PPULDSF32Intrin<string clang_builtin> :
  GCCBuiltin<clang_builtin>,
  Intrinsic<[llvm_float_ty],
    [LLVMQualPointerType<llvm_float_ty, 3>,
    llvm_float_ty,
    llvm_i32_ty, // ordering
    llvm_i32_ty, // scope
    llvm_i1_ty], // isVolatile
    [IntrArgMemOnly, NoCapture<0>, ImmArg<2>, ImmArg<3>, ImmArg<4>]
>;

// FIXME: The m0 argument should be moved after the normal arguments
class PPUDSOrderedIntrinsic : Intrinsic<
  [llvm_i32_ty],
  // M0 = {hi16:address, lo16:waveID}. Allow passing M0 as a pointer, so that
  // the bit packing can be optimized at the IR level.
  [LLVMQualPointerType<llvm_i32_ty, 2>, // IntToPtr(M0)
   llvm_i32_ty, // value to add or swap
   llvm_i32_ty, // ordering
   llvm_i32_ty, // scope
   llvm_i1_ty,  // isVolatile
   llvm_i32_ty, // ordered count index (OA index), also added to the address
                // gfx10: bits 24-27 indicate the number of active threads/dwords
   llvm_i1_ty,  // wave release, usually set to 1
   llvm_i1_ty], // wave done, set to 1 for the last ordered instruction
  [NoCapture<0>,
   ImmArg<2>, ImmArg<3>, ImmArg<4>,
   ImmArg<5>, ImmArg<6>, ImmArg<7>
  ]
>;

class PPUDSAppendConsumedIntrinsic : Intrinsic<
  [llvm_i32_ty],
  [llvm_anyptr_ty, // LDS or GDS ptr
   llvm_i1_ty], // isVolatile
   [IntrConvergent, IntrArgMemOnly, NoCapture<0>, ImmArg<1>],
   "",
   [SDNPMemOperand]
>;

def int_ppu_ds_ordered_add : PPUDSOrderedIntrinsic;
def int_ppu_ds_ordered_swap : PPUDSOrderedIntrinsic;

// The pointer argument is assumed to be dynamically uniform if a VGPR.
def int_ppu_ds_append : PPUDSAppendConsumedIntrinsic;
def int_ppu_ds_consume : PPUDSAppendConsumedIntrinsic;

def int_ppu_ds_fadd : PPULDSF32Intrin<"__builtin_ppu_ds_faddf">;
def int_ppu_ds_fmin : PPULDSF32Intrin<"__builtin_ppu_ds_fminf">;
def int_ppu_ds_fmax : PPULDSF32Intrin<"__builtin_ppu_ds_fmaxf">;


// Set EXEC to the 64-bit value given.
// This is always moved to the beginning of the basic block.
def int_ppu_init_exec : Intrinsic<[],
  [llvm_i32_ty],      // 32-bit literal constant
  [IntrConvergent]>;

// Set EXEC according to a thread count packed in an SGPR input:
//    thread_count = (input >> bitoffset) & 0x7f;
// This is always moved to the beginning of the basic block.
def int_ppu_init_exec_from_input : Intrinsic<[],
  [llvm_i32_ty,       // 32-bit SGPR input
   llvm_i32_ty],      // bit offset of the thread count
  [IntrConvergent]>;

//===----------------------------------------------------------------------===//
// Instruction Intrinsics
//===----------------------------------------------------------------------===//
// The first parameter is s_sendmsg immediate (i16),
// the second one is copied to m0
def int_ppu_sendmsg : GCCBuiltin<"__builtin_ppu_sendmsg">,
  Intrinsic <[], [llvm_i32_ty, llvm_i32_ty], []>;

def int_ppu_sendmsghalt : GCCBuiltin<"__builtin_ppu_sendmsghalt">,
  Intrinsic <[], [llvm_i32_ty, llvm_i32_ty], []>;

def int_ppu_barrier : GCCBuiltin<"__syncthreads">,
  Intrinsic<[], [], [IntrConvergent]>;

def int_ppu_barrier_popc : GCCBuiltin<"__builtin_ppu_barrier_popc">,
  Intrinsic<[llvm_i32_ty], [llvm_i32_ty], [IntrConvergent]>;

def int_ppu_barrier_sync_cnt : GCCBuiltin<"__builtin_ppu_barrier_sync_cnt">,
    Intrinsic<[], [llvm_i32_ty, llvm_i32_ty], [IntrConvergent]>;

def int_ppu_wave_barrier : GCCBuiltin<"__builtin_ppu_wave_barrier">,
  Intrinsic<[], [], [IntrConvergent]>;

def int_ppu_waitcnt : GCCBuiltin<"__builtin_ppu_waitcnt">,
  Intrinsic<[], [llvm_i32_ty], []>;



def int_ppu_rcp : Intrinsic<
  [llvm_anyfloat_ty], [LLVMMatchType<0>], [IntrNoMem, IntrSpeculatable]
>;

def int_ppu_rsq :  Intrinsic<
  [llvm_anyfloat_ty], [LLVMMatchType<0>], [IntrNoMem, IntrSpeculatable]
>;




def int_ppu_cvt_pkrtz : GCCBuiltin<"__builtin_ppu_cvt_pkrtz">,
  Intrinsic<[llvm_v2f16_ty], [llvm_float_ty, llvm_float_ty],
            [IntrNoMem, IntrSpeculatable]
>;

def int_ppu_cvt_pknorm_i16 :
  GCCBuiltin<"__builtin_ppu_cvt_pknorm_i16">,
  Intrinsic<[llvm_v2i16_ty], [llvm_float_ty, llvm_float_ty],
            [IntrNoMem, IntrSpeculatable]
>;

def int_ppu_cvt_pknorm_u16 :
  GCCBuiltin<"__builtin_ppu_cvt_pknorm_u16">,
  Intrinsic<[llvm_v2i16_ty], [llvm_float_ty, llvm_float_ty],
            [IntrNoMem, IntrSpeculatable]
>;

def int_ppu_cvt_pk_i16 :
    GCCBuiltin<"__builtin_ppu_cvt_pk_i16">,
    Intrinsic<
  [llvm_v2i16_ty], [llvm_i32_ty, llvm_i32_ty],
  [IntrNoMem, IntrSpeculatable]
>;

def int_ppu_cvt_pk_u16 : GCCBuiltin<"__builtin_ppu_cvt_pk_u16">,
  Intrinsic<[llvm_v2i16_ty], [llvm_i32_ty, llvm_i32_ty],
    [IntrNoMem, IntrSpeculatable]
>;






//////////////////////////////////////////////////////////////////////////
// Buffer intrinsics
//////////////////////////////////////////////////////////////////////////

// NOTE ppu use rsrc to v2i32

defset list<PPURsrcIntrinsic> PPUBufferIntrinsics = {

class PPUBufferLoad<LLVMType data_ty = llvm_any_ty> : Intrinsic <
  [data_ty],
  [llvm_v2i32_ty,     // rsrc(SGPR)
   llvm_i32_ty,       // vindex(VGPR)
   llvm_i32_ty,       // offset(SGPR/VGPR/imm)
   llvm_i1_ty,        // glc(imm)
   llvm_i1_ty],       // slc(imm)
  [IntrReadMem, ImmArg<3>, ImmArg<4>], "", [SDNPMemOperand]>,
  PPURsrcIntrinsic<0>;
def int_ppu_buffer_load_format : PPUBufferLoad<llvm_anyfloat_ty>;
def int_ppu_buffer_load : PPUBufferLoad;

def int_ppu_s_buffer_load : Intrinsic <
  [llvm_any_ty],
  [llvm_v2i32_ty,     // rsrc(SGPR)
   llvm_i32_ty,       // byte offset(SGPR/imm)
   llvm_i32_ty],      // cachepolicy(imm; bit 0 = glc, bit 2 = dlc)
  [IntrNoMem, ImmArg<2>]>,
  PPURsrcIntrinsic<0>;

class PPUBufferStore<LLVMType data_ty = llvm_any_ty> : Intrinsic <
  [],
  [data_ty,          // vdata(VGPR)
   llvm_v2i32_ty,     // rsrc(SGPR)
   llvm_i32_ty,       // vindex(VGPR)
   llvm_i32_ty,       // offset(SGPR/VGPR/imm)
   llvm_i1_ty,        // glc(imm)
   llvm_i1_ty],       // slc(imm)
  [IntrWriteMem, ImmArg<4>, ImmArg<5>], "", [SDNPMemOperand]>,
  PPURsrcIntrinsic<1>;
def int_ppu_buffer_store_format : PPUBufferStore<llvm_anyfloat_ty>;
def int_ppu_buffer_store : PPUBufferStore;

// New buffer intrinsics with separate raw and struct variants.  The raw
// variant never has an index. The struct variant always has an index, even if
// it is const 0. A struct intrinsic with constant 0 index is different to the
// corresponding raw intrinsic on gfx9+ because the behavior of bound checking
// and swizzling changes depending on whether idxen is set in the instruction.
// These new instrinsics also keep the offset and soffset arguments separate as
// they behave differently in bounds checking and swizzling.
class PPURawBufferLoad<LLVMType data_ty = llvm_any_ty> : Intrinsic <
  [data_ty],
  [llvm_v2i32_ty,     // rsrc(SGPR)
   llvm_i32_ty,       // offset(VGPR/imm, included in bounds checking and swizzling)
   llvm_i32_ty,       // soffset(SGPR/imm, excluded from bounds checking and swizzling)
   llvm_i32_ty],      // cachepolicy(imm; bit 0 = glc, bit 1 = slc, bit 2 = dlc on gfx10+)
  [IntrReadMem, ImmArg<3>], "", [SDNPMemOperand]>,
  PPURsrcIntrinsic<0>;
def int_ppu_raw_buffer_load_format : PPURawBufferLoad<llvm_anyfloat_ty>;
def int_ppu_raw_buffer_load : PPURawBufferLoad;

class PPUStructBufferLoad<LLVMType data_ty = llvm_any_ty> : Intrinsic <
  [data_ty],
  [llvm_v2i32_ty,     // rsrc(SGPR)
   llvm_i32_ty,       // vindex(VGPR)
   llvm_i32_ty,       // offset(VGPR/imm, included in bounds checking and swizzling)
   llvm_i32_ty,       // soffset(SGPR/imm, excluded from bounds checking and swizzling)
   llvm_i32_ty],      // cachepolicy(imm; bit 0 = glc, bit 1 = slc, bit 2 = dlc on gfx10+)
  [IntrReadMem, ImmArg<4>], "", [SDNPMemOperand]>,
  PPURsrcIntrinsic<0>;
def int_ppu_struct_buffer_load_format : PPUStructBufferLoad<llvm_anyfloat_ty>;
def int_ppu_struct_buffer_load : PPUStructBufferLoad;

class PPURawBufferStore<LLVMType data_ty = llvm_any_ty> : Intrinsic <
  [],
  [data_ty,           // vdata(VGPR)
   llvm_v2i32_ty,     // rsrc(SGPR)
   llvm_i32_ty,       // offset(VGPR/imm, included in bounds checking and swizzling)
   llvm_i32_ty,       // soffset(SGPR/imm, excluded from bounds checking and swizzling)
   llvm_i32_ty],      // cachepolicy(imm; bit 0 = glc, bit 1 = slc, bit 2 = dlc on gfx10+)
  [IntrWriteMem, ImmArg<4>], "", [SDNPMemOperand]>,
  PPURsrcIntrinsic<1>;
def int_ppu_raw_buffer_store_format : PPURawBufferStore<llvm_anyfloat_ty>;
def int_ppu_raw_buffer_store : PPURawBufferStore;

class PPUStructBufferStore<LLVMType data_ty = llvm_any_ty> : Intrinsic <
  [],
  [data_ty,           // vdata(VGPR)
   llvm_v2i32_ty,     // rsrc(SGPR)
   llvm_i32_ty,       // vindex(VGPR)
   llvm_i32_ty,       // offset(VGPR/imm, included in bounds checking and swizzling)
   llvm_i32_ty,       // soffset(SGPR/imm, excluded from bounds checking and swizzling)
   llvm_i32_ty],      // cachepolicy(imm; bit 0 = glc, bit 1 = slc, bit 2 = dlc on gfx10+)
  [IntrWriteMem, ImmArg<5>], "", [SDNPMemOperand]>,
  PPURsrcIntrinsic<1>;
def int_ppu_struct_buffer_store_format : PPUStructBufferStore<llvm_anyfloat_ty>;
def int_ppu_struct_buffer_store : PPUStructBufferStore;

class PPURawBufferAtomic<LLVMType data_ty = llvm_any_ty> : Intrinsic <
  [data_ty],
  [LLVMMatchType<0>,  // vdata(VGPR)
   llvm_v2i32_ty,     // rsrc(SGPR)
   llvm_i32_ty,       // offset(VGPR/imm, included in bounds checking and swizzling)
   llvm_i32_ty,       // soffset(SGPR/imm, excluded from bounds checking and swizzling)
   llvm_i32_ty],      // cachepolicy(imm; bit 1 = slc)
  [ImmArg<4>], "", [SDNPMemOperand]>,
  PPURsrcIntrinsic<1, 0>;
def int_ppu_raw_buffer_atomic_swap : PPURawBufferAtomic;
def int_ppu_raw_buffer_atomic_add : PPURawBufferAtomic;
def int_ppu_raw_buffer_atomic_sub : PPURawBufferAtomic;
def int_ppu_raw_buffer_atomic_smin : PPURawBufferAtomic;
def int_ppu_raw_buffer_atomic_umin : PPURawBufferAtomic;
def int_ppu_raw_buffer_atomic_smax : PPURawBufferAtomic;
def int_ppu_raw_buffer_atomic_umax : PPURawBufferAtomic;
def int_ppu_raw_buffer_atomic_and : PPURawBufferAtomic;
def int_ppu_raw_buffer_atomic_or : PPURawBufferAtomic;
def int_ppu_raw_buffer_atomic_xor : PPURawBufferAtomic;
def int_ppu_raw_buffer_atomic_inc : PPURawBufferAtomic;
def int_ppu_raw_buffer_atomic_dec : PPURawBufferAtomic;
def int_ppu_raw_buffer_atomic_cmpswap : Intrinsic<
  [llvm_anyint_ty],
  [LLVMMatchType<0>,  // src(VGPR)
   LLVMMatchType<0>,  // cmp(VGPR)
   llvm_v2i32_ty,     // rsrc(SGPR)
   llvm_i32_ty,       // offset(VGPR/imm, included in bounds checking and swizzling)
   llvm_i32_ty,       // soffset(SGPR/imm, excluded from bounds checking and swizzling)
   llvm_i32_ty],      // cachepolicy(imm; bit 1 = slc)
  [ImmArg<5>], "", [SDNPMemOperand]>,
  PPURsrcIntrinsic<2, 0>;

class PPUStructBufferAtomic<LLVMType data_ty = llvm_any_ty> : Intrinsic <
  [data_ty],
  [LLVMMatchType<0>,  // vdata(VGPR)
   llvm_v2i32_ty,     // rsrc(SGPR)
   llvm_i32_ty,       // vindex(VGPR)
   llvm_i32_ty,       // offset(VGPR/imm, included in bounds checking and swizzling)
   llvm_i32_ty,       // soffset(SGPR/imm, excluded from bounds checking and swizzling)
   llvm_i32_ty],      // cachepolicy(imm; bit 1 = slc)
  [ImmArg<5>], "", [SDNPMemOperand]>,
  PPURsrcIntrinsic<1, 0>;
def int_ppu_struct_buffer_atomic_swap : PPUStructBufferAtomic;
def int_ppu_struct_buffer_atomic_add : PPUStructBufferAtomic;
def int_ppu_struct_buffer_atomic_sub : PPUStructBufferAtomic;
def int_ppu_struct_buffer_atomic_smin : PPUStructBufferAtomic;
def int_ppu_struct_buffer_atomic_umin : PPUStructBufferAtomic;
def int_ppu_struct_buffer_atomic_smax : PPUStructBufferAtomic;
def int_ppu_struct_buffer_atomic_umax : PPUStructBufferAtomic;
def int_ppu_struct_buffer_atomic_and : PPUStructBufferAtomic;
def int_ppu_struct_buffer_atomic_or : PPUStructBufferAtomic;
def int_ppu_struct_buffer_atomic_xor : PPUStructBufferAtomic;
def int_ppu_struct_buffer_atomic_inc : PPUStructBufferAtomic;
def int_ppu_struct_buffer_atomic_dec : PPUStructBufferAtomic;
def int_ppu_struct_buffer_atomic_cmpswap : Intrinsic<
  [llvm_anyint_ty],
  [LLVMMatchType<0>,  // src(VGPR)
   LLVMMatchType<0>,  // cmp(VGPR)
   llvm_v2i32_ty,     // rsrc(SGPR)
   llvm_i32_ty,       // vindex(VGPR)
   llvm_i32_ty,       // offset(VGPR/imm, included in bounds checking and swizzling)
   llvm_i32_ty,       // soffset(SGPR/imm, excluded from bounds checking and swizzling)
   llvm_i32_ty],      // cachepolicy(imm; bit 1 = slc)
  [ImmArg<6>], "", [SDNPMemOperand]>,
  PPURsrcIntrinsic<2, 0>;

// Obsolescent tbuffer intrinsics.
def int_ppu_tbuffer_load : Intrinsic <
    [llvm_any_ty],    // overloaded for types f32/i32, v2f32/v2i32, v4f32/v4i32
    [llvm_v2i32_ty,   // rsrc(SGPR)
     llvm_i32_ty,     // vindex(VGPR)
     llvm_i32_ty,     // voffset(VGPR)
     llvm_i32_ty,     // soffset(SGPR)
     llvm_i32_ty,     // offset(imm)
     llvm_i32_ty,     // dfmt(imm)
     llvm_i32_ty,     // nfmt(imm)
     llvm_i1_ty,     // glc(imm)
     llvm_i1_ty],    // slc(imm)
    [IntrReadMem, ImmArg<4>, ImmArg<5>, ImmArg<6>,
     ImmArg<7>, ImmArg<8>], "", [SDNPMemOperand]>,
  PPURsrcIntrinsic<0>;

def int_ppu_tbuffer_store : Intrinsic <
    [],
    [llvm_any_ty,    // vdata(VGPR), overloaded for types f32/i32, v2f32/v2i32, v4f32/v4i32
     llvm_v2i32_ty,  // rsrc(SGPR)
     llvm_i32_ty,    // vindex(VGPR)
     llvm_i32_ty,    // voffset(VGPR)
     llvm_i32_ty,    // soffset(SGPR)
     llvm_i32_ty,    // offset(imm)
     llvm_i32_ty,    // dfmt(imm)
     llvm_i32_ty,    // nfmt(imm)
     llvm_i1_ty,     // glc(imm)
     llvm_i1_ty],    // slc(imm)
    [IntrWriteMem, ImmArg<5>, ImmArg<6>, ImmArg<7>,
     ImmArg<8>, ImmArg<9>], "", [SDNPMemOperand]>,
  PPURsrcIntrinsic<1>;

// New tbuffer intrinsics, with:
// - raw and struct variants
// - joint format field
// - joint cachepolicy field
def int_ppu_raw_tbuffer_load : Intrinsic <
    [llvm_any_ty],    // overloaded for types f32/i32, v2f32/v2i32, v4f32/v4i32
    [llvm_v2i32_ty,   // rsrc(SGPR)
     llvm_i32_ty,     // offset(VGPR/imm, included in bounds checking and swizzling)
     llvm_i32_ty,     // soffset(SGPR/imm, excluded from bounds checking and swizzling)
     llvm_i32_ty,     // format(imm; bits 3..0 = dfmt, bits 6..4 = nfmt)
     llvm_i32_ty],    // cachepolicy(imm; bit 0 = glc, bit 1 = slc, bit 2 = dlc on gfx10+)
    [IntrReadMem, ImmArg<3>, ImmArg<4>], "", [SDNPMemOperand]>,
  PPURsrcIntrinsic<0>;

def int_ppu_raw_tbuffer_store : Intrinsic <
    [],
    [llvm_any_ty,    // vdata(VGPR), overloaded for types f32/i32, v2f32/v2i32, v4f32/v4i32
     llvm_v2i32_ty,  // rsrc(SGPR)
     llvm_i32_ty,    // offset(VGPR/imm, included in bounds checking and swizzling)
     llvm_i32_ty,    // soffset(SGPR/imm, excluded from bounds checking and swizzling)
     llvm_i32_ty,    // format(imm; bits 3..0 = dfmt, bits 6..4 = nfmt)
     llvm_i32_ty],   // cachepolicy(imm; bit 0 = glc, bit 1 = slc, bit 2 = dlc on gfx10+)
    [IntrWriteMem, ImmArg<4>, ImmArg<5>], "", [SDNPMemOperand]>,
  PPURsrcIntrinsic<1>;

def int_ppu_struct_tbuffer_load : Intrinsic <
    [llvm_any_ty],    // overloaded for types f32/i32, v2f32/v2i32, v4f32/v4i32
    [llvm_v2i32_ty,   // rsrc(SGPR)
     llvm_i32_ty,     // vindex(VGPR)
     llvm_i32_ty,     // offset(VGPR/imm, included in bounds checking and swizzling)
     llvm_i32_ty,     // soffset(SGPR/imm, excluded from bounds checking and swizzling)
     llvm_i32_ty,     // format(imm; bits 3..0 = dfmt, bits 6..4 = nfmt)
     llvm_i32_ty],    // cachepolicy(imm; bit 0 = glc, bit 1 = slc, bit 2 = dlc on gfx10+)
    [IntrReadMem, ImmArg<4>, ImmArg<5>], "", [SDNPMemOperand]>,
  PPURsrcIntrinsic<0>;

def int_ppu_struct_tbuffer_store : Intrinsic <
    [],
    [llvm_any_ty,    // vdata(VGPR), overloaded for types f32/i32, v2f32/v2i32, v4f32/v4i32
     llvm_v2i32_ty,  // rsrc(SGPR)
     llvm_i32_ty,    // vindex(VGPR)
     llvm_i32_ty,    // offset(VGPR/imm, included in bounds checking and swizzling)
     llvm_i32_ty,    // soffset(SGPR/imm, excluded from bounds checking and swizzling)
     llvm_i32_ty,    // format(imm; bits 3..0 = dfmt, bits 6..4 = nfmt)
     llvm_i32_ty],   // cachepolicy(imm; bit 0 = glc, bit 1 = slc, bit 2 = dlc on gfx10+)
    [IntrWriteMem, ImmArg<5>, ImmArg<6>], "", [SDNPMemOperand]>,
  PPURsrcIntrinsic<1>;

class PPUBufferAtomic : Intrinsic <
  [llvm_anyint_ty],
  [LLVMMatchType<0>,       // vdata(VGPR)
   llvm_v2i32_ty,     // rsrc(SGPR)
   llvm_i32_ty,       // vindex(VGPR)
   llvm_i32_ty,       // offset(SGPR/VGPR/imm)
   llvm_i1_ty],       // slc(imm)
  [ImmArg<4>], "", [SDNPMemOperand]>,
  PPURsrcIntrinsic<1, 0>;
def int_ppu_buffer_atomic_swap : PPUBufferAtomic;
def int_ppu_buffer_atomic_add : PPUBufferAtomic;
def int_ppu_buffer_atomic_sub : PPUBufferAtomic;
def int_ppu_buffer_atomic_smin : PPUBufferAtomic;
def int_ppu_buffer_atomic_umin : PPUBufferAtomic;
def int_ppu_buffer_atomic_smax : PPUBufferAtomic;
def int_ppu_buffer_atomic_umax : PPUBufferAtomic;
def int_ppu_buffer_atomic_and : PPUBufferAtomic;
def int_ppu_buffer_atomic_or : PPUBufferAtomic;
def int_ppu_buffer_atomic_xor : PPUBufferAtomic;
def int_ppu_buffer_atomic_cmpswap : Intrinsic<
  [llvm_i32_ty],
  [llvm_i32_ty,       // src(VGPR)
   llvm_i32_ty,       // cmp(VGPR)
   llvm_v2i32_ty,     // rsrc(SGPR)
   llvm_i32_ty,       // vindex(VGPR)
   llvm_i32_ty,       // offset(SGPR/VGPR/imm)
   llvm_i1_ty],       // slc(imm)
  [ImmArg<5>], "", [SDNPMemOperand]>,
  PPURsrcIntrinsic<2, 0>;

} // defset PPUBufferIntrinsics
/*
// Uses that do not set the done bit should set IntrWriteMem on the
// call site.
def int_ppu_exp : Intrinsic <[], [
  llvm_i32_ty,       // tgt,
  llvm_i32_ty,       // en
  llvm_any_ty,       // src0 (f32 or i32)
  LLVMMatchType<0>,  // src1
  LLVMMatchType<0>,  // src2
  LLVMMatchType<0>,  // src3
  llvm_i1_ty,        // done
  llvm_i1_ty         // vm
  ],
  [ImmArg<0>, ImmArg<1>, ImmArg<6>, ImmArg<7>, IntrInaccessibleMemOnly]
>;

// exp with compr bit set.
def int_ppu_exp_compr : Intrinsic <[], [
  llvm_i32_ty,       // tgt,
  llvm_i32_ty,       // en
  llvm_anyvector_ty, // src0 (v2f16 or v2i16)
  LLVMMatchType<0>,  // src1
  llvm_i1_ty,        // done
  llvm_i1_ty],       // vm
  [ImmArg<0>, ImmArg<1>, ImmArg<4>, ImmArg<5>, IntrInaccessibleMemOnly]
>;
*/
def int_ppu_buffer_wbinvl1_sc :
  GCCBuiltin<"__builtin_ppu_buffer_wbinvl1_sc">,
  Intrinsic<[], [], []>;

def int_ppu_buffer_wbinvl1 :
  GCCBuiltin<"__builtin_ppu_buffer_wbinvl1">,
  Intrinsic<[], [], []>;

def int_ppu_s_dcache_inv :
  GCCBuiltin<"__builtin_ppu_s_dcache_inv">,
  Intrinsic<[], [], []>;

def int_ppu_s_memtime :
  GCCBuiltin<"__builtin_ppu_s_memtime">,
  Intrinsic<[llvm_i64_ty], []>;

def int_ppu_s_sleep :
  GCCBuiltin<"__builtin_ppu_s_sleep">,
  Intrinsic<[], [llvm_i32_ty], [ImmArg<0>]> {
}

def int_ppu_s_getreg :
  GCCBuiltin<"__builtin_ppu_s_getreg">,
  Intrinsic<[llvm_i32_ty], [llvm_i32_ty],
  [IntrInaccessibleMemOnly, IntrReadMem, IntrSpeculatable, ImmArg<0>]
>;

// int_ppu_s_getpc is provided to allow a specific style of position
// independent code to determine the high part of its address when it is
// known (through convention) that the code and any data of interest does
// not cross a 4Gb address boundary. Use for any other purpose may not
// produce the desired results as optimizations may cause code movement,
// especially as we explicitly use IntrNoMem to allow optimizations.
def int_ppu_s_getpc :
  GCCBuiltin<"__builtin_ppu_s_getpc">,
  Intrinsic<[llvm_i64_ty], [], [IntrNoMem, IntrSpeculatable]>;










def int_ppu_lerp :
  GCCBuiltin<"__builtin_ppu_lerp">,
  Intrinsic<[llvm_i32_ty], [llvm_i32_ty, llvm_i32_ty, llvm_i32_ty],
  [IntrNoMem, IntrSpeculatable]
>;

def int_ppu_sad_u8 :
  GCCBuiltin<"__builtin_ppu_sad_u8">,
  Intrinsic<[llvm_i32_ty], [llvm_i32_ty, llvm_i32_ty, llvm_i32_ty],
  [IntrNoMem, IntrSpeculatable]
>;

def int_ppu_msad_u8 :
  GCCBuiltin<"__builtin_ppu_msad_u8">,
  Intrinsic<[llvm_i32_ty], [llvm_i32_ty, llvm_i32_ty, llvm_i32_ty],
  [IntrNoMem, IntrSpeculatable]
>;

def int_ppu_sad_hi_u8 :
  GCCBuiltin<"__builtin_ppu_sad_hi_u8">,
  Intrinsic<[llvm_i32_ty], [llvm_i32_ty, llvm_i32_ty, llvm_i32_ty],
  [IntrNoMem, IntrSpeculatable]
>;

def int_ppu_sad_u16 :
  GCCBuiltin<"__builtin_ppu_sad_u16">,
  Intrinsic<[llvm_i32_ty], [llvm_i32_ty, llvm_i32_ty, llvm_i32_ty],
  [IntrNoMem, IntrSpeculatable]
>;

def int_ppu_qsad_pk_u16_u8 :
  GCCBuiltin<"__builtin_ppu_qsad_pk_u16_u8">,
  Intrinsic<[llvm_i64_ty], [llvm_i64_ty, llvm_i32_ty, llvm_i64_ty],
  [IntrNoMem, IntrSpeculatable]
>;

def int_ppu_mqsad_pk_u16_u8 :
  GCCBuiltin<"__builtin_ppu_mqsad_pk_u16_u8">,
  Intrinsic<[llvm_i64_ty], [llvm_i64_ty, llvm_i32_ty, llvm_i64_ty],
  [IntrNoMem, IntrSpeculatable]
>;

def int_ppu_mqsad_u32_u8 :
  GCCBuiltin<"__builtin_ppu_mqsad_u32_u8">,
  Intrinsic<[llvm_v4i32_ty], [llvm_i64_ty, llvm_i32_ty, llvm_v4i32_ty],
  [IntrNoMem, IntrSpeculatable]
>;

def int_ppu_cvt_pk_u8_f32 :
  GCCBuiltin<"__builtin_ppu_cvt_pk_u8_f32">,
  Intrinsic<[llvm_i32_ty], [llvm_float_ty, llvm_i32_ty, llvm_i32_ty],
  [IntrNoMem, IntrSpeculatable]
>;

def int_ppu_icmp :
  Intrinsic<[llvm_anyint_ty], [llvm_anyint_ty, LLVMMatchType<1>, llvm_i32_ty],
            [IntrNoMem, IntrConvergent, ImmArg<2>]>;

def int_ppu_fcmp :
  Intrinsic<[llvm_anyint_ty], [llvm_anyfloat_ty, LLVMMatchType<1>, llvm_i32_ty],
            [IntrNoMem, IntrConvergent, ImmArg<2>]>;

def int_ppu_readfirstlane :
  GCCBuiltin<"__builtin_ppu_readfirstlane">,
  Intrinsic<[llvm_i32_ty], [llvm_i32_ty], [IntrNoMem, IntrConvergent]>;

// The lane argument must be uniform across the currently active threads of the
// current wave. Otherwise, the result is undefined.
def int_ppu_readlane :
  GCCBuiltin<"__builtin_ppu_readlane">,
  Intrinsic<[llvm_i32_ty], [llvm_i32_ty, llvm_i32_ty], [IntrNoMem, IntrConvergent]>;

// The value to write and lane select arguments must be uniform across the
// currently active threads of the current wave. Otherwise, the result is
// undefined.
def int_ppu_writelane :
  GCCBuiltin<"__builtin_ppu_writelane">,
  Intrinsic<[llvm_i32_ty], [
    llvm_i32_ty,    // uniform value to write: returned by the selected lane
    llvm_i32_ty,    // uniform lane select
    llvm_i32_ty     // returned by all lanes other than the selected one
  ],
  [IntrNoMem, IntrConvergent]
>;

def int_ppu_alignbit :
  GCCBuiltin<"__builtin_ppu_alignbit">, Intrinsic<[llvm_i32_ty],
  [llvm_i32_ty, llvm_i32_ty, llvm_i32_ty],
  [IntrNoMem, IntrSpeculatable]
>;

def int_ppu_alignbyte : GCCBuiltin<"__builtin_ppu_alignbyte">,
  Intrinsic<[llvm_i32_ty], [llvm_i32_ty, llvm_i32_ty, llvm_i32_ty],
  [IntrNoMem, IntrSpeculatable]
>;

def int_ppu_mul_i24 : Intrinsic<[llvm_i32_ty],
  [llvm_i32_ty, llvm_i32_ty],
  [IntrNoMem, IntrSpeculatable]
>;

def int_ppu_mul_u24 : Intrinsic<[llvm_i32_ty],
  [llvm_i32_ty, llvm_i32_ty],
  [IntrNoMem, IntrSpeculatable]
>;

// Copies the source value to the destination value, with the guarantee that
// the source value is computed as if the entire program were executed in WQM.
def int_ppu_wqm : Intrinsic<[llvm_any_ty],
  [LLVMMatchType<0>], [IntrNoMem, IntrSpeculatable]
>;

// Copies the source value to the destination value, such that the source
// is computed as if the entire program were executed in WQM if any other
// program code executes in WQM.
def int_ppu_softwqm : Intrinsic<[llvm_any_ty],
  [LLVMMatchType<0>], [IntrNoMem, IntrSpeculatable]
>;

// Return true if at least one thread within the pixel quad passes true into
// the function.
def int_ppu_wqm_vote : Intrinsic<[llvm_i1_ty],
  [llvm_i1_ty], [IntrNoMem, IntrConvergent]
>;

// If false, set TMSK=0 for the current thread until the end of program.
def int_ppu_kill : Intrinsic<[], [llvm_i1_ty], []>;

// Copies the active channels of the source value to the destination value,
// with the guarantee that the source value is computed as if the entire
// program were executed in Whole Wavefront Mode, i.e. with all channels
// enabled, with a few exceptions: - Phi nodes with require WWM return an
// undefined value.
def int_ppu_wwm : Intrinsic<[llvm_any_ty],
  [LLVMMatchType<0>], [IntrNoMem, IntrSpeculatable, IntrConvergent]
>;

// Given a value, copies it while setting all the inactive lanes to a given
// value. Note that OpenGL helper lanes are considered active, so if the
// program ever uses WQM, then the instruction and the first source will be
// computed in WQM.
def int_ppu_set_inactive :
  Intrinsic<[llvm_anyint_ty],
            [LLVMMatchType<0>, // value to be copied
             LLVMMatchType<0>], // value for the inactive lanes to take
            [IntrNoMem, IntrConvergent]>;

def int_ppu_s_dcache_inv_vol :
  GCCBuiltin<"__builtin_ppu_s_dcache_inv_vol">,
  Intrinsic<[], [], []>;

def int_ppu_buffer_wbinvl1_vol :
  GCCBuiltin<"__builtin_ppu_buffer_wbinvl1_vol">,
  Intrinsic<[], [], []>;

def int_ppu_s_dcache_wb :
  GCCBuiltin<"__builtin_ppu_s_dcache_wb">,
  Intrinsic<[], [], []>;

def int_ppu_s_dcache_wb_vol :
  GCCBuiltin<"__builtin_ppu_s_dcache_wb_vol">,
  Intrinsic<[], [], []>;

def int_ppu_s_memrealtime :
  GCCBuiltin<"__builtin_ppu_s_memrealtime">,
  Intrinsic<[llvm_i64_ty]>;

// llvm.ppu.ds.permute <index> <src>
def int_ppu_ds_permute :
  GCCBuiltin<"__builtin_ppu_ds_permute">,
  Intrinsic<[llvm_i32_ty], [llvm_i32_ty, llvm_i32_ty], [IntrNoMem, IntrConvergent]>;

// llvm.ppu.ds.bpermute <index> <src>
def int_ppu_ds_bpermute :
  GCCBuiltin<"__builtin_ppu_ds_bpermute">,
  Intrinsic<[llvm_i32_ty], [llvm_i32_ty, llvm_i32_ty], [IntrNoMem, IntrConvergent]>;

// llvm.ppu.permlane16 <old> <src0> <src1> <src2> <fi> <bound_control>
def int_ppu_permlane16 : GCCBuiltin<"__builtin_ppu_permlane16">,
  Intrinsic<[llvm_i32_ty],
            [llvm_i32_ty, llvm_i32_ty, llvm_i32_ty, llvm_i32_ty, llvm_i1_ty, llvm_i1_ty],
            [IntrNoMem, IntrConvergent, ImmArg<4>, ImmArg<5>]>;

// llvm.ppu.permlanex16 <old> <src0> <src1> <src2> <fi> <bound_control>
def int_ppu_permlanex16 : GCCBuiltin<"__builtin_ppu_permlanex16">,
  Intrinsic<[llvm_i32_ty],
            [llvm_i32_ty, llvm_i32_ty, llvm_i32_ty, llvm_i32_ty, llvm_i1_ty, llvm_i1_ty],
            [IntrNoMem, IntrConvergent, ImmArg<4>, ImmArg<5>]>;


def int_ppu_s_get_waveid_in_workgroup :
  GCCBuiltin<"__builtin_ppu_s_get_waveid_in_workgroup">,
  Intrinsic<[llvm_i32_ty], [], [IntrReadMem, IntrInaccessibleMemOnly]>;

//===----------------------------------------------------------------------===//
// Deep learning intrinsics.
//===----------------------------------------------------------------------===//

// f32 %r = llvm.ppu.fdot2(v2f16 %a, v2f16 %b, f32 %c, i1 %clamp)
//   %r = %a[0] * %b[0] + %a[1] * %b[1] + %c
def int_ppu_fdot2 :
  GCCBuiltin<"__builtin_ppu_fdot2">,
  Intrinsic<
    [llvm_float_ty], // %r
    [
      llvm_v2f16_ty, // %a
      llvm_v2f16_ty, // %b
      llvm_float_ty, // %c
      llvm_i1_ty     // %clamp
    ],
    [IntrNoMem, IntrSpeculatable, ImmArg<3>]
  >;

// i32 %r = llvm.ppu.sdot2(v2i16 %a, v2i16 %b, i32 %c, i1 %clamp)
//   %r = %a[0] * %b[0] + %a[1] * %b[1] + %c
def int_ppu_sdot2 :
  GCCBuiltin<"__builtin_ppu_sdot2">,
  Intrinsic<
    [llvm_i32_ty], // %r
    [
      llvm_v2i16_ty, // %a
      llvm_v2i16_ty, // %b
      llvm_i32_ty,   // %c
      llvm_i1_ty     // %clamp
    ],
    [IntrNoMem, IntrSpeculatable, ImmArg<3>]
  >;

// u32 %r = llvm.ppu.udot2(v2u16 %a, v2u16 %b, u32 %c, i1 %clamp)
//   %r = %a[0] * %b[0] + %a[1] * %b[1] + %c
def int_ppu_udot2 :
  GCCBuiltin<"__builtin_ppu_udot2">,
  Intrinsic<
    [llvm_i32_ty], // %r
    [
      llvm_v2i16_ty, // %a
      llvm_v2i16_ty, // %b
      llvm_i32_ty,   // %c
      llvm_i1_ty     // %clamp
    ],
    [IntrNoMem, IntrSpeculatable, ImmArg<3>]
  >;

// i32 %r = llvm.ppu.sdot4(v4i8 (as i32) %a, v4i8 (as i32) %b, i32 %c, i1 %clamp)
//   %r = %a[0] * %b[0] + %a[1] * %b[1] + %a[2] * %b[2] + %a[3] * %b[3] + %c
def int_ppu_sdot4 :
  GCCBuiltin<"__builtin_ppu_sdot4">,
  Intrinsic<
    [llvm_i32_ty], // %r
    [
      llvm_i32_ty, // %a
      llvm_i32_ty, // %b
      llvm_i32_ty, // %c
      llvm_i1_ty   // %clamp
    ],
    [IntrNoMem, IntrSpeculatable, ImmArg<3>]
  >;

// u32 %r = llvm.ppu.udot4(v4u8 (as u32) %a, v4u8 (as u32) %b, u32 %c, i1 %clamp)
//   %r = %a[0] * %b[0] + %a[1] * %b[1] + %a[2] * %b[2] + %a[3] * %b[3] + %c
def int_ppu_udot4 :
  GCCBuiltin<"__builtin_ppu_udot4">,
  Intrinsic<
    [llvm_i32_ty], // %r
    [
      llvm_i32_ty, // %a
      llvm_i32_ty, // %b
      llvm_i32_ty, // %c
      llvm_i1_ty   // %clamp
    ],
    [IntrNoMem, IntrSpeculatable, ImmArg<3>]
  >;

// i32 %r = llvm.ppu.sdot8(v8i4 (as i32) %a, v8i4 (as i32) %b, i32 %c, i1 %clamp)
//   %r = %a[0] * %b[0] + %a[1] * %b[1] + %a[2] * %b[2] + %a[3] * %b[3] +
//        %a[4] * %b[4] + %a[5] * %b[5] + %a[6] * %b[6] + %a[7] * %b[7] + %c
def int_ppu_sdot8 :
  GCCBuiltin<"__builtin_ppu_sdot8">,
  Intrinsic<
    [llvm_i32_ty], // %r
    [
      llvm_i32_ty, // %a
      llvm_i32_ty, // %b
      llvm_i32_ty, // %c
      llvm_i1_ty   // %clamp
    ],
    [IntrNoMem, IntrSpeculatable, ImmArg<3>]
  >;

// u32 %r = llvm.ppu.udot8(v8u4 (as u32) %a, v8u4 (as u32) %b, u32 %c, i1 %clamp)
//   %r = %a[0] * %b[0] + %a[1] * %b[1] + %a[2] * %b[2] + %a[3] * %b[3] +
//        %a[4] * %b[4] + %a[5] * %b[5] + %a[6] * %b[6] + %a[7] * %b[7] + %c
def int_ppu_udot8 :
  GCCBuiltin<"__builtin_ppu_udot8">,
  Intrinsic<
    [llvm_i32_ty], // %r
    [
      llvm_i32_ty, // %a
      llvm_i32_ty, // %b
      llvm_i32_ty, // %c
      llvm_i1_ty   // %clamp
    ],
    [IntrNoMem, IntrSpeculatable, ImmArg<3>]
  >;


class PPUBufferAtomicNoRtn : Intrinsic <
  [],
  [llvm_anyfloat_ty,  // vdata(VGPR)
   llvm_v2i32_ty,     // rsrc(SGPR)
   llvm_i32_ty,       // vindex(VGPR)
   llvm_i32_ty,       // offset(SGPR/VGPR/imm)
   llvm_i1_ty],       // slc(imm)
  [], "", [SDNPMemOperand]>,
  PPURsrcIntrinsic<1, 0>;

class PPUGlobalAtomicNoRtn : Intrinsic <
  [],
  [llvm_anyptr_ty,    // vaddr
   llvm_anyfloat_ty],               // vdata(VGPR)
  [IntrArgMemOnly, NoCapture<0>], "", [SDNPMemOperand]>;

def int_ppu_buffer_atomic_fadd    : PPUBufferAtomicNoRtn;
def int_ppu_global_atomic_fadd    : PPUGlobalAtomicNoRtn;



//===----------------------------------------------------------------------===//
// Special Intrinsics for backend internal use only. No frontend
// should emit calls to these.
// ===----------------------------------------------------------------------===//
def int_ppu_if : Intrinsic<[llvm_i1_ty, llvm_i32_ty],
  [llvm_i1_ty], [IntrConvergent]
>;

def int_ppu_else : Intrinsic<[llvm_i1_ty, llvm_i32_ty],
  [llvm_i32_ty], [IntrConvergent]
>;

def int_ppu_break : Intrinsic<[llvm_i32_ty],
  [llvm_i32_ty], [IntrNoMem, IntrConvergent]
>;

def int_ppu_if_break : Intrinsic<[llvm_i32_ty],
  [llvm_i1_ty, llvm_i32_ty], [IntrNoMem, IntrConvergent]
>;

def int_ppu_else_break : Intrinsic<[llvm_i32_ty],
  [llvm_i32_ty, llvm_i32_ty], [IntrNoMem, IntrConvergent]
>;

def int_ppu_loop : Intrinsic<[llvm_i1_ty],
  [llvm_i32_ty], [IntrConvergent]
>;

def int_ppu_end_cf : Intrinsic<[], [llvm_i32_ty], [IntrConvergent]>;

// Represent unreachable in a divergent region.
def int_ppu_unreachable : Intrinsic<[], [], [IntrConvergent]>;

// Emit 2.5 ulp, no denormal division. Should only be inserted by
// pass based on !fpmath metadata.
def int_ppu_fdiv_fast : Intrinsic<
  [llvm_float_ty], [llvm_float_ty, llvm_float_ty],
  [IntrNoMem, IntrSpeculatable]
>;


} // TargetPrefix = "ppu"

// Below is from Intrinsicsnvvm.td

// The following intrinsics were once defined here, but are now auto-upgraded
// to target-generic LLVM intrinsics.
//
//   * llvm.nvvm.brev32  --> llvm.bitreverse.i32
//   * llvm.nvvm.brev64  --> llvm.bitreverse.i64
//   * llvm.nvvm.clz.i   --> llvm.ctlz.i32
//   * llvm.nvvm.clz.ll  --> trunc i64 llvm.ctlz.i64(x) to i32
//   * llvm.nvvm.popc.i  --> llvm.ctpop.i32
//   * llvm.nvvm.popc.ll --> trunc i64 llvm.ctpop.i64 to i32
//   * llvm.nvvm.abs.i   --> select(x >= -x, x, -x)
//   * llvm.nvvm.abs.ll  --> ibid.
//   * llvm.nvvm.max.i   --> select(x sge y, x, y)
//   * llvm.nvvm.max.ll  --> ibid.
//   * llvm.nvvm.max.ui  --> select(x uge y, x, y)
//   * llvm.nvvm.max.ull --> ibid.
//   * llvm.nvvm.max.i   --> select(x sle y, x, y)
//   * llvm.nvvm.max.ll  --> ibid.
//   * llvm.nvvm.max.ui  --> select(x ule y, x, y)
//   * llvm.nvvm.max.ull --> ibid.
//   * llvm.nvvm.h2f     --> llvm.convert.to.fp16.f32

// def llvm_anyi64ptr_ty     : LLVMAnyPointerType<llvm_i64_ty>;     // (space)i64*

// Helper class for construction of n-element list<LLVMtype> [t,t,...,t]
// class RepLLVMType<int N, LLVMType T> {
//   list<LLVMType> ret = !if(N, !listconcat(RepLLVMType<!add(N,-1), T>.ret, [T]), []);
// }


//let TargetPrefix = "ppu" in {
//defm int_ppu_nvvm_tid : PPUThreadRegisterIntrinsic_xyz_named
//                               <"__builtin_ppu_nvvm_tid">;
//
//defm int_ppu_nvvm_ntid : PPUWarpRegisterIntrinsic_xyz_named
//                               <"__builtin_ppu_nvvm_ntid">;
//
//def int_ppu_nvvm_laneid : PPUThreadRegisterIntrinsic;
//
//defm int_ppu_nvvm_warpid : PPUWarpRegisterIntrinsic_xyz_named
//                               <"__builtin_ppu_nvvm_warpid">;
//
//defm int_ppu_nvvm_nwarpid : PPUWarpRegisterIntrinsic_xyz_named
//                               <"__builtin_ppu_nvvm_nwarpid">;
//
//defm int_ppu_nvvm_ctaid : PPUWarpRegisterIntrinsic_xyz_named
//                               <"__builtin_ppu_nvvm_ctaid">;
//
//defm int_ppu_nvvm_nctaid : PPUWarpRegisterIntrinsic_xyz_named
//                               <"__builtin_ppu_nvvm_nctaid">;
//}
