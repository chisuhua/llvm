//===----  Scalar Memory Instruction Defintions --------===//

def smrd_offset_8 : NamedOperandU32<"SMRDOffset8",
                                  NamedMatchClass<"SMRDOffset8">> {
  let OperandType = "OPERAND_IMMEDIATE";
}

def smrd_offset_20 : NamedOperandU32<"SMRDOffset20",
                                  NamedMatchClass<"SMRDOffset20">> {
  let OperandType = "OPERAND_IMMEDIATE";
}

//===----------------------------------------------------------------------===//
// Scalar Memory classes
//===----------------------------------------------------------------------===//

class SM_Pseudo <string opName, dag outs, dag ins, string asmOps, list<dag> pattern=[]> :
  PPUInst <outs, ins, "", "", pattern>,
  PPUMCInstr<opName, PPUEncodingFamily.NONE> {
  let isPseudo = 1;
  let isCodeGenOnly = 1;

  let LGKM_CNT = 1;
  let SMRD = 1;
  let mayStore = 0;
  let mayLoad = 1;
  let hasSideEffects = 0;
  let UseNamedOperandTable = 1;
  let SchedRW = [WriteSMEM];

  string Mnemonic = opName;
  string AsmOperands = asmOps;

  bits<1> has_sbase = 1;
  bits<1> has_sdst = 1;
  bit has_glc = 0;
  bit has_dlc = 0;
  bits<1> has_offset = 1;
  bits<1> offset_is_imm = 0;
}

class SM_Real <SM_Pseudo ps>
  : PPUInst<ps.OutOperandList, ps.InOperandList, ps.Mnemonic, ps.AsmOperands, []> {

  let isPseudo = 0;
  let isCodeGenOnly = 0;

  // copy relevant pseudo op flags
  let SubtargetPredicate = ps.SubtargetPredicate;
  let AsmMatchConverter  = ps.AsmMatchConverter;

  // encoding
  bits<7>  sbase;
  bits<7>  sdst;
  bits<32> offset;
  bits<1> imm = !if(ps.has_offset, ps.offset_is_imm, 0);
}

class SM_Probe_Pseudo <string opName, dag ins, bit isImm>
  : SM_Pseudo<opName, (outs), ins, " $sdata, $sbase, $offset"> {
  let mayLoad = 0;
  let mayStore = 0;
  let has_glc = 0;
  let LGKM_CNT = 0;
  let ScalarStore = 0;
  let hasSideEffects = 1;
  let offset_is_imm = isImm;
  let PseudoInstr = opName # !if(isImm, "_IMM", "_SGPR");
}

class SM_Load_Pseudo <string opName, dag outs, dag ins, string asmOps, list<dag> pattern=[]>
  : SM_Pseudo<opName, outs, ins, asmOps, pattern> {
  RegisterClass BaseClass;
  let mayLoad = 1;
  let mayStore = 0;
  let has_glc = 1;
  let has_dlc = 1;
}

class SM_Store_Pseudo <string opName, dag ins, string asmOps, list<dag> pattern = []>
  : SM_Pseudo<opName, (outs), ins, asmOps, pattern> {
  RegisterClass BaseClass;
  RegisterClass SrcClass;
  let mayLoad = 0;
  let mayStore = 1;
  let has_glc = 1;
  let has_dlc = 1;
  let ScalarStore = 1;
}

class SM_Discard_Pseudo <string opName, dag ins, bit isImm>
  : SM_Pseudo<opName, (outs), ins, " $sbase, $offset"> {
  let mayLoad = 0;
  let mayStore = 0;
  let has_glc = 0;
  let has_sdst = 0;
  let ScalarStore = 0;
  let hasSideEffects = 1;
  let offset_is_imm = isImm;
  let PseudoInstr = opName # !if(isImm, "_IMM", "_SGPR");
}


multiclass SM_Pseudo_Loads<string opName,
                           RegisterClass baseClass,
                           RegisterClass dstClass> {
  def _IMM  : SM_Load_Pseudo <opName,
                              (outs dstClass:$sdst),
                              (ins baseClass:$sbase, i32imm:$offset, i1imm:$glc, i1imm:$dlc),
                              " $sdst, $sbase, $offset$glc$dlc", []> {
    let offset_is_imm = 1;
    let BaseClass = baseClass;
    let PseudoInstr = opName # "_IMM";
    let has_glc = 1;
    let has_dlc = 1;
  }

  def _SGPR  : SM_Load_Pseudo <opName,
                              (outs dstClass:$sdst),
                              (ins baseClass:$sbase, SReg_32:$soff, i1imm:$glc, i1imm:$dlc),
                              " $sdst, $sbase, $offset$glc$dlc", []> {
    let BaseClass = baseClass;
    let PseudoInstr = opName # "_SGPR";
    let has_glc = 1;
    let has_dlc = 1;
  }
}

multiclass SM_Pseudo_Stores<string opName,
                           RegisterClass baseClass,
                           RegisterClass srcClass> {
  def _IMM  : SM_Store_Pseudo <opName,
    (ins srcClass:$sdata, baseClass:$sbase, i32imm:$offset, i1imm:$glc, i1imm:$dlc),
    " $sdata, $sbase, $offset$glc$dlc", []> {
    let offset_is_imm = 1;
    let BaseClass = baseClass;
    let SrcClass = srcClass;
    let PseudoInstr = opName # "_IMM";
  }

  def _SGPR  : SM_Store_Pseudo <opName,
    (ins srcClass:$sdata, baseClass:$sbase, SReg_32:$soff, i1imm:$glc, i1imm:$dlc),
    " $sdata, $sbase, $offset$glc$dlc", []> {
    let BaseClass = baseClass;
    let SrcClass = srcClass;
    let PseudoInstr = opName # "_SGPR";
  }
}

multiclass SM_Pseudo_Discards<string opName> {
  def _IMM  : SM_Discard_Pseudo <opName, (ins SReg_64:$sbase, smrd_offset_20:$offset), 1>;
  def _SGPR : SM_Discard_Pseudo <opName, (ins SReg_64:$sbase, SReg_32:$offset), 0>;
}

class SM_Time_Pseudo<string opName, SDPatternOperator node = null_frag> : SM_Pseudo<
  opName, (outs SReg_64:$sdst), (ins),
  " $sdst", [(set i64:$sdst, (node))]> {
  let hasSideEffects = 1;

  // FIXME: This should be definitively mayStore = 0. TableGen
  // brokenly tries to infer these based on the intrinsic properties
  // corresponding to the IR attributes. The target intrinsics are
  // considered as writing to memory for IR dependency purposes, but
  // those can be modeled with hasSideEffects here. These also end up
  // inferring differently for llvm.readcyclecounter and the amdgcn
  // intrinsics.
  let mayStore = ?;
  let mayLoad = 1;
  let has_sbase = 0;
  let has_offset = 0;
}

class SM_Inval_Pseudo <string opName, SDPatternOperator node = null_frag> : SM_Pseudo<
  opName, (outs), (ins), "", [(node)]> {
  let hasSideEffects = 1;
  let mayStore = 1;
  let has_sdst = 0;
  let has_sbase = 0;
  let has_offset = 0;
}


multiclass SM_Pseudo_Probe<string opName, RegisterClass baseClass> {
  def _IMM  : SM_Probe_Pseudo <opName, (ins i8imm:$sdata, baseClass:$sbase, smrd_offset_20:$offset), 1>;
  def _SGPR : SM_Probe_Pseudo <opName, (ins i8imm:$sdata, baseClass:$sbase, SReg_32:$offset), 0>;
}

class SM_WaveId_Pseudo<string opName, SDPatternOperator node> : SM_Pseudo<
  opName, (outs SReg_32:$sdst), (ins),
  " $sdst", [(set i32:$sdst, (node))]> {
  let hasSideEffects = 1;
  let mayStore = 0;
  let mayLoad = 1;
  let has_sbase = 0;
  let has_offset = 0;
}
//===----------------------------------------------------------------------===//
// Scalar Atomic Memory Classes
//===----------------------------------------------------------------------===//

class SM_Atomic_Pseudo <string opName,
                        dag outs, dag ins, string asmOps, bit isRet>
  : SM_Pseudo<opName, outs, ins, asmOps, []> {

  bit glc = isRet;

  let mayLoad = 1;
  let mayStore = 1;
  let has_glc = 1;
  let has_dlc = 1;

  // Should these be set?
  let ScalarStore = 1;
  let hasSideEffects = 1;
  let maybeAtomic = 1;
}

class SM_Pseudo_Atomic<string opName,
                       RegisterClass baseClass,
                       RegisterClass dataClass,
                       bit isImm,
                       bit isRet> :
  SM_Atomic_Pseudo<opName,
                   !if(isRet, (outs dataClass:$sdst), (outs)),
                   !if(isImm,
                       (ins dataClass:$sdata, baseClass:$sbase, smrd_offset_20:$offset, DLC:$dlc),
                       (ins dataClass:$sdata, baseClass:$sbase, SReg_32:$offset, DLC:$dlc)),
                   !if(isRet, " $sdst", " $sdata") # ", $sbase, $offset" # !if(isRet, " glc", "") # "$dlc",
                   isRet> {
  let offset_is_imm = isImm;
  let PseudoInstr = opName # !if(isImm,
                                 !if(isRet, "_IMM_RTN", "_IMM"),
                                 !if(isRet, "_SGPR_RTN", "_SGPR"));

  let Constraints = !if(isRet, "$sdst = $sdata", "");
  let DisableEncoding = !if(isRet, "$sdata", "");
}

multiclass SM_Pseudo_Atomics<string opName,
                             RegisterClass baseClass,
                             RegisterClass dataClass> {
  def _IMM      : SM_Pseudo_Atomic <opName, baseClass, dataClass, 1, 0>;
  def _SGPR     : SM_Pseudo_Atomic <opName, baseClass, dataClass, 0, 0>;
  def _IMM_RTN  : SM_Pseudo_Atomic <opName, baseClass, dataClass, 1, 1>;
  def _SGPR_RTN : SM_Pseudo_Atomic <opName, baseClass, dataClass, 0, 1>;
}


