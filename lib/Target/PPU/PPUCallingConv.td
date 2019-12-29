//===-- PPUCallingConv.td - Calling Conventions PPU ----*- tablegen -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This describes the calling conventions for the PPU architecture.
//
//===----------------------------------------------------------------------===//

// The PPU calling convention is handled with custom code in
// PPUISelLowering.cpp (CC_PPT).

def CSR_ILP32_LP64
    : CalleeSavedRegs<(add X1, X3, X4, X8, X9, (sequence "X%u", 18, 27))>;

/* TODO schi remove rv float
def CSR_ILP32F_LP64F
    : CalleeSavedRegs<(add CSR_ILP32_LP64,
                       F8_32, F9_32, (sequence "F%u_32", 18, 27))>;

def CSR_ILP32D_LP64D
    : CalleeSavedRegs<(add CSR_ILP32_LP64,
                       F8_64, F9_64, (sequence "F%u_64", 18, 27))>;
                       */

// Needed for implementation of PPURegisterInfo::getNoPreservedMask()
def CSR_NoRegs : CalleeSavedRegs<(add)>;

// Interrupt handler needs to save/restore all registers that are used,
// both Caller and Callee saved registers.
def CSR_Interrupt : CalleeSavedRegs<(add X1,
    (sequence "X%u", 3, 9),
    (sequence "X%u", 10, 11),
    (sequence "X%u", 12, 17),
    (sequence "X%u", 18, 27),
    (sequence "X%u", 28, 31))>;

// Same as CSR_Interrupt, but including all 32-bit FP registers.
def CSR_XLEN_F32_Interrupt: CalleeSavedRegs<(add X1,
    (sequence "X%u", 3, 9),
    (sequence "X%u", 10, 11),
    (sequence "X%u", 12, 17),
    (sequence "X%u", 18, 27),
    (sequence "X%u", 28, 31),
    (sequence "F%u_32", 0, 7),
    (sequence "F%u_32", 10, 11),
    (sequence "F%u_32", 12, 17),
    (sequence "F%u_32", 28, 31),
    (sequence "F%u_32", 8, 9),
    (sequence "F%u_32", 18, 27))>;

// Same as CSR_Interrupt, but including all 64-bit FP registers.
/*
def CSR_XLEN_F64_Interrupt: CalleeSavedRegs<(add X1,
    (sequence "X%u", 3, 9),
    (sequence "X%u", 10, 11),
    (sequence "X%u", 12, 17),
    (sequence "X%u", 18, 27),
    (sequence "X%u", 28, 31),
    (sequence "F%u_64", 0, 7),
    (sequence "F%u_64", 10, 11),
    (sequence "F%u_64", 12, 17),
    (sequence "F%u_64", 28, 31),
    (sequence "F%u_64", 8, 9),
    (sequence "F%u_64", 18, 27))>;
*/
// AMDGPUCallingConv

// Node: for CallingConv::AMDGPU_KERNEL/SPIR_KERNEL it don't have Call/Ret 
//       for CallingConv::AMDGPU_CS , use CC_PPT_Compute
//       for CallingConv::C/Fast/COld , use CC_PPT_Func

// Inversion of CCIfInReg
class CCIfNotInReg<CCAction A> : CCIf<"!ArgFlags.isInReg()", A> {}
class CCIfExtend<CCAction A>
  : CCIf<"ArgFlags.isSExt() || ArgFlags.isZExt()", A>;

// for inreg attr, it pass by SPR, other pass by VPR
// Calling convention for Compute
def CC_PPT_Compute : CallingConv<[

  CCIfInReg<CCIfType<[f32, i32, f16, v2i16, v2f16] , CCAssignToReg<[
    SPR0, SPR1, SPR2, SPR3, SPR4, SPR5, SPR6, SPR7,
    SPR8, SPR9, SPR10, SPR11, SPR12, SPR13, SPR14, SPR15,
    SPR16, SPR17, SPR18, SPR19, SPR20, SPR21, SPR22, SPR23,
    SPR24, SPR25, SPR26, SPR27, SPR28, SPR29, SPR30, SPR31,
    SPR32, SPR33, SPR34, SPR35, SPR36, SPR37, SPR38, SPR39,
    SPR40, SPR41, SPR42, SPR43
  ]>>>,

  // 32*4 + 4 is the minimum for a fetch shader consumer with 32 inputs.
  CCIfNotInReg<CCIfType<[f32, i32, f16, v2i16, v2f16] , CCAssignToReg<[
    VPR0, VPR1, VPR2, VPR3, VPR4, VPR5, VPR6, VPR7,
    VPR8, VPR9, VPR10, VPR11, VPR12, VPR13, VPR14, VPR15,
    VPR16, VPR17, VPR18, VPR19, VPR20, VPR21, VPR22, VPR23,
    VPR24, VPR25, VPR26, VPR27, VPR28, VPR29, VPR30, VPR31,
    VPR32, VPR33, VPR34, VPR35, VPR36, VPR37, VPR38, VPR39,
    VPR40, VPR41, VPR42, VPR43, VPR44, VPR45, VPR46, VPR47,
    VPR48, VPR49, VPR50, VPR51, VPR52, VPR53, VPR54, VPR55,
    VPR56, VPR57, VPR58, VPR59, VPR60, VPR61, VPR62, VPR63,
    VPR64, VPR65, VPR66, VPR67, VPR68, VPR69, VPR70, VPR71,
    VPR72, VPR73, VPR74, VPR75, VPR76, VPR77, VPR78, VPR79,
    VPR80, VPR81, VPR82, VPR83, VPR84, VPR85, VPR86, VPR87,
    VPR88, VPR89, VPR90, VPR91, VPR92, VPR93, VPR94, VPR95,
    VPR96, VPR97, VPR98, VPR99, VPR100, VPR101, VPR102, VPR103,
    VPR104, VPR105, VPR106, VPR107, VPR108, VPR109, VPR110, VPR111,
    VPR112, VPR113, VPR114, VPR115, VPR116, VPR117, VPR118, VPR119,
    VPR120, VPR121, VPR122, VPR123, VPR124, VPR125, VPR126, VPR127,
    VPR128, VPR129, VPR130, VPR131, VPR132, VPR133, VPR134, VPR135
  ]>>>
]>;

def RetCC_PPT_Compute : CallingConv<[
  CCIfType<[i32] , CCAssignToReg<[
    SPR0, SPR1, SPR2, SPR3, SPR4, SPR5, SPR6, SPR7,
    SPR8, SPR9, SPR10, SPR11, SPR12, SPR13, SPR14, SPR15,
    SPR16, SPR17, SPR18, SPR19, SPR20, SPR21, SPR22, SPR23,
    SPR24, SPR25, SPR26, SPR27, SPR28, SPR29, SPR30, SPR31,
    SPR32, SPR33, SPR34, SPR35, SPR36, SPR37, SPR38, SPR39,
    SPR40, SPR41, SPR42, SPR43
  ]>>,

  // 32*4 + 4 is the minimum for a fetch shader with 32 outputs.
  CCIfType<[f32, f16, v2f16] , CCAssignToReg<[
    VPR0, VPR1, VPR2, VPR3, VPR4, VPR5, VPR6, VPR7,
    VPR8, VPR9, VPR10, VPR11, VPR12, VPR13, VPR14, VPR15,
    VPR16, VPR17, VPR18, VPR19, VPR20, VPR21, VPR22, VPR23,
    VPR24, VPR25, VPR26, VPR27, VPR28, VPR29, VPR30, VPR31,
    VPR32, VPR33, VPR34, VPR35, VPR36, VPR37, VPR38, VPR39,
    VPR40, VPR41, VPR42, VPR43, VPR44, VPR45, VPR46, VPR47,
    VPR48, VPR49, VPR50, VPR51, VPR52, VPR53, VPR54, VPR55,
    VPR56, VPR57, VPR58, VPR59, VPR60, VPR61, VPR62, VPR63,
    VPR64, VPR65, VPR66, VPR67, VPR68, VPR69, VPR70, VPR71,
    VPR72, VPR73, VPR74, VPR75, VPR76, VPR77, VPR78, VPR79,
    VPR80, VPR81, VPR82, VPR83, VPR84, VPR85, VPR86, VPR87,
    VPR88, VPR89, VPR90, VPR91, VPR92, VPR93, VPR94, VPR95,
    VPR96, VPR97, VPR98, VPR99, VPR100, VPR101, VPR102, VPR103,
    VPR104, VPR105, VPR106, VPR107, VPR108, VPR109, VPR110, VPR111,
    VPR112, VPR113, VPR114, VPR115, VPR116, VPR117, VPR118, VPR119,
    VPR120, VPR121, VPR122, VPR123, VPR124, VPR125, VPR126, VPR127,
    VPR128, VPR129, VPR130, VPR131, VPR132, VPR133, VPR134, VPR135
  ]>>
]>;

def CSR_PPU_VPRs_24_255 : CalleeSavedRegs<
  (sequence "VPR%u", 24, 255)
>;

def CSR_PPU_VPRs_32_255 : CalleeSavedRegs<
  (sequence "VPR%u", 32, 255)
>;

def CSR_PPU_SPRs_32_105 : CalleeSavedRegs<
  (sequence "SPR%u", 32, 105)
>;

// Just to get the regmask, not for calling convention purposes.
def CSR_PPU_AllVPRs : CalleeSavedRegs<
  (sequence "VPR%u", 0, 255)
>;

// Just to get the regmask, not for calling convention purposes.
def CSR_PPU_AllAllocatableSRegs : CalleeSavedRegs<
  (add (sequence "SPR%u", 0, 105), VCC)
>;

def CSR_PPU_HighRegs : CalleeSavedRegs<
  (add CSR_PPU_VPRs_32_255, CSR_PPU_SPRs_32_105)
>;

// Calling convention for leaf functions
def CC_PPT_Func : CallingConv<[
  CCIfByVal<CCPassByVal<4, 4>>,
  CCIfType<[i1], CCPromoteToType<i32>>,
  CCIfType<[i1, i8, i16], CCIfExtend<CCPromoteToType<i32>>>,
  CCIfType<[i32, f32, i16, f16, v2i16, v2f16, i1], CCAssignToReg<[
    VPR0, VPR1, VPR2, VPR3, VPR4, VPR5, VPR6, VPR7,
    VPR8, VPR9, VPR10, VPR11, VPR12, VPR13, VPR14, VPR15,
    VPR16, VPR17, VPR18, VPR19, VPR20, VPR21, VPR22, VPR23,
    VPR24, VPR25, VPR26, VPR27, VPR28, VPR29, VPR30, VPR31]>>,
  CCIfType<[i32, f32, v2i16, v2f16, i16, f16, i1], CCAssignToStack<4, 4>>,
  CCIfType<[i64, f64, v2i32, v2f32], CCAssignToStack<8, 4>>,
  CCIfType<[v3i32, v3f32], CCAssignToStack<12, 4>>,
  CCIfType<[v4i32, v4f32, v2i64, v2f64], CCAssignToStack<16, 4>>,
  CCIfType<[v5i32, v5f32], CCAssignToStack<20, 4>>,
  CCIfType<[v8i32, v8f32], CCAssignToStack<32, 4>>,
  CCIfType<[v16i32, v16f32], CCAssignToStack<64, 4>>
]>;

// Calling convention for leaf functions
def RetCC_PPT_Func : CallingConv<[
  CCIfType<[i1], CCPromoteToType<i32>>,
  CCIfType<[i1, i16], CCIfExtend<CCPromoteToType<i32>>>,
  CCIfType<[i32, f32, i16, f16, v2i16, v2f16], CCAssignToReg<[
    VPR0, VPR1, VPR2, VPR3, VPR4, VPR5, VPR6, VPR7,
    VPR8, VPR9, VPR10, VPR11, VPR12, VPR13, VPR14, VPR15,
    VPR16, VPR17, VPR18, VPR19, VPR20, VPR21, VPR22, VPR23,
    VPR24, VPR25, VPR26, VPR27, VPR28, VPR29, VPR30, VPR31]>>,
]>;

def CC_PPT : CallingConv<[
   CCIf<"State.getCallingConv() == CallingConv::AMDGPU_CS",
        CCDelegateTo<CC_PPT_Compute>>,
   CCIf<"State.getCallingConv() == CallingConv::C",
        CCDelegateTo<CC_PPT_Func>>
]>;
