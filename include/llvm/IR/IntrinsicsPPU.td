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
