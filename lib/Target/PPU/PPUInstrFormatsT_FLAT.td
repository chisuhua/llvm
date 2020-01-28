//===-- FLATInstructions.td - FLAT Instruction Defintions -----------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

def FLATAtomic : ComplexPattern<i64, 3, "SelectFlatAtomic", [], [SDNPWantRoot], -10>;
def FLATOffset : ComplexPattern<i64, 3, "SelectFlatOffset<false>", [], [SDNPWantRoot], -10>;

def FLATOffsetSigned : ComplexPattern<i64, 3, "SelectFlatOffset<true>", [], [SDNPWantRoot], -10>;
def FLATSignedAtomic : ComplexPattern<i64, 3, "SelectFlatAtomicSigned", [], [SDNPWantRoot], -10>;

//===----------------------------------------------------------------------===//
// FLAT classes
//===----------------------------------------------------------------------===//

class FLAT_Pseudo<string opName, dag outs, dag ins,
                  string asmOps, list<dag> pattern=[]> :
  PPTInst<outs, ins, opName, asmOps, pattern>,
  PPUMCInstr<opName, PPUEncodingFamily.NONE> {

  let isPseudo = 1;
  let isCodeGenOnly = 1;

  let FLAT = 1;

  let UseNamedOperandTable = 1;
  let hasSideEffects = 0;
  let SchedRW = [WriteVMEM];

  string Mnemonic = opName;
  string AsmOperands = asmOps;

  bits<1> is_flat_global = 0;
  bits<1> is_flat_scratch = 0;

  bits<1> has_vdst = 1;

  // We need to distinguish having saddr and enabling saddr because
  // saddr is only valid for scratch and global instructions. Pre-gfx9
  // these bits were reserved, so we also don't necessarily want to
  // set these bits to the disabled value for the original flat
  // segment instructions.
  bits<1> has_saddr = 0;
  bits<1> enabled_saddr = 0;
  bits<7> saddr_value = 0;
  bits<1> has_vaddr = 1;

  bits<1> has_data = 1;
  bits<1> has_glc  = 1;
  bits<1> glcValue = 0;
  bits<1> has_dlc  = 1;
  bits<1> dlcValue = 0;

  let SubtargetPredicate = !if(is_flat_global, HasFlatGlobalInsts,
    !if(is_flat_scratch, HasFlatScratchInsts, HasFlatAddressSpace));

  // TODO: M0 if it could possibly access LDS (before gfx9? only)?
  let Uses = !if(is_flat_global, [TMSK], [TMSK, FLAT_SCR]);

  // Internally, FLAT instruction are executed as both an LDS and a
  // Buffer instruction; so, they increment both VM_CNT and LGKM_CNT
  // and are not considered done until both have been decremented.
  let VM_CNT = 1;
  let LGKM_CNT = !if(!or(is_flat_global, is_flat_scratch), 0, 1);

  let IsNonFlatSeg = !if(!or(is_flat_global, is_flat_scratch), 1, 0);
}

class FLAT_Real <bits<7> op, FLAT_Pseudo ps> :
  PPTInst <ps.OutOperandList, ps.InOperandList, ps.Mnemonic , ps.AsmOperands, []>,
  Enc64 {

  let isPseudo = 0;
  let isCodeGenOnly = 0;

  // copy relevant pseudo op flags
  let SubtargetPredicate = ps.SubtargetPredicate;
  let AsmMatchConverter  = ps.AsmMatchConverter;
  let TSFlags = ps.TSFlags;
  let UseNamedOperandTable = ps.UseNamedOperandTable;

  // encoding fields
  bits<8> vaddr;
  bits<8> vdata;
  bits<7> saddr;
  bits<8> vdst;

  bits<1> slc;
  bits<1> glc;
  bits<1> dlc;

  // Only valid on gfx9
  bits<1> lds = 0; // XXX - What does this actually do?

  // Segment, 00=flat, 01=scratch, 10=global, 11=reserved
  bits<2> seg = !if(ps.is_flat_global, 0b10,
                  !if(ps.is_flat_scratch, 0b01, 0));

  // Signed offset. Highest bit ignored for flat and treated as 12-bit
  // unsigned for flat acceses.
  bits<13> offset;
  bits<1> nv = 0; // XXX - What does this actually do?

  // We don't use tfe right now, and it was removed in gfx9.
  bits<1> tfe = 0;

  // Only valid on GFX9+
  let Inst{12-0} = offset;
  let Inst{13} = lds;
  let Inst{15-14} = seg;

  let Inst{16}    = !if(ps.has_glc, glc, ps.glcValue);
  let Inst{17}    = slc;
  let Inst{24-18} = op;
  let Inst{31-26} = 0x37; // Encoding.
  let Inst{39-32} = !if(ps.has_vaddr, vaddr, ?);
  let Inst{47-40} = !if(ps.has_data, vdata, ?);
  let Inst{54-48} = !if(ps.has_saddr, !if(ps.enabled_saddr, saddr, 0x7f), 0);

  // 54-48 is reserved.
  let Inst{55}    = nv; // nv on GFX9+, TFE before.
  let Inst{63-56} = !if(ps.has_vdst, vdst, ?);
}

class GlobalSaddrTable <bit is_saddr, string Name = ""> {
  bit IsSaddr = is_saddr;
  string SaddrOp = Name;
}

// TODO: Is exec allowed for saddr? The disabled value 0x7f is the
// same encoding value as exec_hi, so it isn't possible to use that if
// saddr is 32-bit (which isn't handled here yet).
class FLAT_Load_Pseudo <string opName, RegisterClass regClass,
  bit HasTiedOutput = 0,
  bit HasSaddr = 0, bit EnableSaddr = 0> : FLAT_Pseudo<
  opName,
  (outs regClass:$vdst),
  !con(
    !con(
      !con((ins VReg_64:$vaddr),
        !if(EnableSaddr, (ins SReg_64:$saddr), (ins))),
          (ins flat_offset:$offset, GLC:$glc, SLC:$slc, DLC:$dlc)),
          !if(HasTiedOutput, (ins regClass:$vdst_in), (ins))),
  " $vdst, $vaddr"#!if(HasSaddr, !if(EnableSaddr, ", $saddr", ", off"), "")#"$offset$glc$slc$dlc"> {
  let has_data = 0;
  let mayLoad = 1;
  let has_saddr = HasSaddr;
  let enabled_saddr = EnableSaddr;
  let PseudoInstr = opName#!if(!and(HasSaddr, EnableSaddr), "_SADDR", "");
  let maybeAtomic = 1;

  let Constraints = !if(HasTiedOutput, "$vdst = $vdst_in", "");
  let DisableEncoding = !if(HasTiedOutput, "$vdst_in", "");
}

class FLAT_Store_Pseudo <string opName, RegisterClass vdataClass,
  bit HasSaddr = 0, bit EnableSaddr = 0> : FLAT_Pseudo<
  opName,
  (outs),
  !con(
    !con((ins VReg_64:$vaddr, vdataClass:$vdata),
      !if(EnableSaddr, (ins SReg_64:$saddr), (ins))),
        (ins flat_offset:$offset, GLC:$glc, SLC:$slc, DLC:$dlc)),
  " $vaddr, $vdata"#!if(HasSaddr, !if(EnableSaddr, ", $saddr", ", off"), "")#"$offset$glc$slc$dlc"> {
  let mayLoad  = 0;
  let mayStore = 1;
  let has_vdst = 0;
  let has_saddr = HasSaddr;
  let enabled_saddr = EnableSaddr;
  let PseudoInstr = opName#!if(!and(HasSaddr, EnableSaddr), "_SADDR", "");
  let maybeAtomic = 1;
}

multiclass FLAT_Global_Load_Pseudo<string opName, RegisterClass regClass, bit HasTiedInput = 0> {
  let is_flat_global = 1 in {
    def "" : FLAT_Load_Pseudo<opName, regClass, HasTiedInput, 1>,
      GlobalSaddrTable<0, opName>;
    def _SADDR : FLAT_Load_Pseudo<opName, regClass, HasTiedInput, 1, 1>,
      GlobalSaddrTable<1, opName>;
  }
}

multiclass FLAT_Global_Store_Pseudo<string opName, RegisterClass regClass> {
  let is_flat_global = 1 in {
    def "" : FLAT_Store_Pseudo<opName, regClass, 1>,
      GlobalSaddrTable<0, opName>;
    def _SADDR : FLAT_Store_Pseudo<opName, regClass, 1, 1>,
      GlobalSaddrTable<1, opName>;
  }
}
// FIXM I change saddr/vaddr from 32 to 64
class FLAT_Scratch_Load_Pseudo <string opName, RegisterClass regClass,
  bit EnableSaddr = 0>: FLAT_Pseudo<
  opName,
  (outs regClass:$vdst),
  !if(EnableSaddr,
      (ins SReg_64:$saddr, flat_offset:$offset, GLC:$glc, SLC:$slc, DLC:$dlc),
      (ins VReg_64:$vaddr, flat_offset:$offset, GLC:$glc, SLC:$slc, DLC:$dlc)),
  " $vdst, "#!if(EnableSaddr, "off", "$vaddr")#!if(EnableSaddr, ", $saddr", ", off")#"$offset$glc$slc$dlc"> {
  let has_data = 0;
  let mayLoad = 1;
  let has_saddr = 1;
  let enabled_saddr = EnableSaddr;
  let has_vaddr = !if(EnableSaddr, 0, 1);
  let PseudoInstr = opName#!if(EnableSaddr, "_SADDR", "");
  let maybeAtomic = 1;
}

// FIXM I change saddr/vaddr from 32 to 64
class FLAT_Scratch_Store_Pseudo <string opName, RegisterClass vdataClass, bit EnableSaddr = 0> : FLAT_Pseudo<
  opName,
  (outs),
  !if(EnableSaddr,
    (ins SReg_64:$saddr, vdataClass:$vdata, flat_offset:$offset, GLC:$glc, SLC:$slc, DLC:$dlc),
    (ins VReg_64:$vaddr, vdataClass:$vdata, flat_offset:$offset, GLC:$glc, SLC:$slc, DLC:$dlc)),
  /*
  !if(EnableSaddr,
    (ins vdataClass:$vdata, SReg_64:$saddr, flat_offset:$offset, GLC:$glc, SLC:$slc, DLC:$dlc),
    (ins vdataClass:$vdata, VReg_64:$vaddr, flat_offset:$offset, GLC:$glc, SLC:$slc, DLC:$dlc)),
    */
  " "#!if(EnableSaddr, "off", "$vaddr")#", $vdata, "#!if(EnableSaddr, "$saddr", "off")#"$offset$glc$slc$dlc"> {
  let mayLoad  = 0;
  let mayStore = 1;
  let has_vdst = 0;
  let has_saddr = 1;
  let enabled_saddr = EnableSaddr;
  let has_vaddr = !if(EnableSaddr, 0, 1);
  let PseudoInstr = opName#!if(EnableSaddr, "_SADDR", "");
  let maybeAtomic = 1;
}

multiclass FLAT_Scratch_Load_Pseudo<string opName, RegisterClass regClass> {
  let is_flat_scratch = 1 in {
    def "" : FLAT_Scratch_Load_Pseudo<opName, regClass>,
      GlobalSaddrTable<0, opName>;
    def _SADDR : FLAT_Scratch_Load_Pseudo<opName, regClass, 1>,
      GlobalSaddrTable<1, opName>;
  }
}

multiclass FLAT_Scratch_Store_Pseudo<string opName, RegisterClass regClass> {
  let is_flat_scratch = 1 in {
    def "" : FLAT_Scratch_Store_Pseudo<opName, regClass>,
      GlobalSaddrTable<0, opName>;
    def _SADDR : FLAT_Scratch_Store_Pseudo<opName, regClass, 1>,
      GlobalSaddrTable<1, opName>;
  }
}

class FLAT_AtomicNoRet_Pseudo<string opName, dag outs, dag ins,
                               string asm, list<dag> pattern = []> :
  FLAT_Pseudo<opName, outs, ins, asm, pattern> {
    let mayLoad = 1;
    let mayStore = 1;
    let has_glc  = 0;
    let glcValue = 0;
    let has_dlc  = 0;
    let dlcValue = 0;
    let has_vdst = 0;
    let maybeAtomic = 1;
}

class FLAT_AtomicRet_Pseudo<string opName, dag outs, dag ins,
                            string asm, list<dag> pattern = []>
  : FLAT_AtomicNoRet_Pseudo<opName, outs, ins, asm, pattern> {
  let hasPostISelHook = 1;
  let has_vdst = 1;
  let glcValue = 1;
  let dlcValue = 0;
  let PseudoInstr = NAME # "_RTN";
}

multiclass FLAT_Atomic_Pseudo<
  string opName,
  RegisterClass vdst_rc,
  ValueType vt,
  SDPatternOperator atomic = null_frag,
  ValueType data_vt = vt,
  RegisterClass data_rc = vdst_rc,
  bit isFP = getIsFP<data_vt>.ret> {
  def "" : FLAT_AtomicNoRet_Pseudo <opName,
    (outs),
    (ins VReg_64:$vaddr, data_rc:$vdata, flat_offset:$offset, SLC:$slc),
    " $vaddr, $vdata$offset$slc">,
    GlobalSaddrTable<0, opName>,
    AtomicNoRet <opName, 0> {
    let PseudoInstr = NAME;
    let FPAtomic = isFP;
  }

  def _RTN : FLAT_AtomicRet_Pseudo <opName,
    (outs vdst_rc:$vdst),
    (ins VReg_64:$vaddr, data_rc:$vdata, flat_offset:$offset, SLC:$slc),
    " $vdst, $vaddr, $vdata$offset glc$slc",
    [(set vt:$vdst,
      (atomic (FLATAtomic i64:$vaddr, i16:$offset, i1:$slc), data_vt:$vdata))]>,
       GlobalSaddrTable<0, opName#"_rtn">,
       AtomicNoRet <opName, 1>{
    let FPAtomic = isFP;
  }
}

multiclass FLAT_Global_Atomic_Pseudo_NO_RTN<
  string opName,
  RegisterClass vdst_rc,
  ValueType vt,
  SDPatternOperator atomic = null_frag,
  ValueType data_vt = vt,
  RegisterClass data_rc = vdst_rc,
  bit isFP = getIsFP<data_vt>.ret> {

  def "" : FLAT_AtomicNoRet_Pseudo <opName,
    (outs),
    (ins VReg_64:$vaddr, data_rc:$vdata, flat_offset:$offset, SLC:$slc),
    " $vaddr, $vdata, off$offset$slc">,
    GlobalSaddrTable<0, opName>,
    AtomicNoRet <opName, 0> {
    let has_saddr = 1;
    let PseudoInstr = NAME;
    let FPAtomic = isFP;
  }

  def _SADDR : FLAT_AtomicNoRet_Pseudo <opName,
    (outs),
    (ins VReg_64:$vaddr, data_rc:$vdata, SReg_64:$saddr, flat_offset:$offset, SLC:$slc),
    " $vaddr, $vdata, $saddr$offset$slc">,
    GlobalSaddrTable<1, opName>,
    AtomicNoRet <opName#"_saddr", 0> {
    let has_saddr = 1;
    let enabled_saddr = 1;
    let PseudoInstr = NAME#"_SADDR";
    let FPAtomic = isFP;
  }
}

multiclass FLAT_Global_Atomic_Pseudo_RTN<
  string opName,
  RegisterClass vdst_rc,
  ValueType vt,
  SDPatternOperator atomic = null_frag,
  ValueType data_vt = vt,
  RegisterClass data_rc = vdst_rc,
  bit isFP = getIsFP<data_vt>.ret> {

  def _RTN : FLAT_AtomicRet_Pseudo <opName,
    (outs vdst_rc:$vdst),
      (ins VReg_64:$vaddr, data_rc:$vdata, flat_offset:$offset, SLC:$slc),
    " $vdst, $vaddr, $vdata, off$offset glc$slc",
    [(set vt:$vdst,
      (atomic (FLATSignedAtomic i64:$vaddr, i16:$offset, i1:$slc), data_vt:$vdata))]>,
      GlobalSaddrTable<0, opName#"_rtn">,
      AtomicNoRet <opName, 1> {
    let has_saddr = 1;
    let FPAtomic = isFP;
  }

  def _SADDR_RTN : FLAT_AtomicRet_Pseudo <opName,
    (outs vdst_rc:$vdst),
      (ins VReg_64:$vaddr, data_rc:$vdata, SReg_64:$saddr, flat_offset:$offset, SLC:$slc),
    " $vdst, $vaddr, $vdata, $saddr$offset glc$slc">,
    GlobalSaddrTable<1, opName#"_rtn">,
    AtomicNoRet <opName#"_saddr", 1> {
     let has_saddr = 1;
     let enabled_saddr = 1;
     let PseudoInstr = NAME#"_SADDR_RTN";
     let FPAtomic = isFP;
  }
}

multiclass FLAT_Global_Atomic_Pseudo<
  string opName,
  RegisterClass vdst_rc,
  ValueType vt,
  SDPatternOperator atomic = null_frag,
  ValueType data_vt = vt,
  RegisterClass data_rc = vdst_rc> :
    FLAT_Global_Atomic_Pseudo_NO_RTN<opName, vdst_rc, vt, atomic, data_vt, data_rc>,
    FLAT_Global_Atomic_Pseudo_RTN<opName, vdst_rc, vt, atomic, data_vt, data_rc>;

class flat_binary_atomic_op<SDNode atomic_op> : PatFrag<
  (ops node:$ptr, node:$value),
  (atomic_op node:$ptr, node:$value),
  [{return cast<MemSDNode>(N)->getAddressSpace() == AMDGPUAS::FLAT_ADDRESS;}]
>;

