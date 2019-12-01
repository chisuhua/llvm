//===----------------------------------------------------------------------===//
// SOP Instructions
//===----------------------------------------------------------------------===//

def VPRIdxModeMatchClass : AsmOperandClass {
  let Name = "VPRIdxMode";
  let PredicateMethod = "isVPRIdxMode";
  let ParserMethod = "parseVPRIdxMode";
  let RenderMethod = "addImmOperands";
}

def VPRIdxMode : Operand<i32> {
  let PrintMethod = "printVPRIndexMode";
  let ParserMatchClass = VPRIdxModeMatchClass;
  let OperandType = "OPERAND_IMMEDIATE";
}

// Base pseudo class
class SOP_Pseudo<string opName, dag outs, dag ins, string asmOps,
            list<dag> pattern=[]> :
    PPTInst<outs, ins, "", "", pattern> ,
    PPUMCInstr<opName, PPUEncodingFamily.NONE> {
  let isPseudo           = 1;
  let isCodeGenOnly      = 1;

  string Mnemonic        = opName;
  string AsmOperands     = asmOps;
  string PseudoInstr = opName;

  bits<1> has_sdst = 0;
}

//===----------------------------------------------------------------------===//
// SOP1 Instructions
//===----------------------------------------------------------------------===//
class SOP1_Pseudo <string opName, dag outs, dag ins,
                   string asmOps, list<dag> pattern=[]> :
  SOP_Pseudo<opName, outs, ins, asmOps, pattern> {

  let mayLoad = 0;
  let mayStore = 0;
  let hasSideEffects = 0;
  let SALU = 1;
  let SOP1 = 1;
  let SchedRW = [WriteSALU];
  let Size = 4;
  let UseNamedOperandTable = 1;

  bits<1> has_src0 = 1;
  bits<1> has_sdst = 1;
}

class SOP1<bits<8> op, SOP1_Pseudo ps> :
  PPTInst <ps.OutOperandList, ps.InOperandList,
          ps.Mnemonic,  ps.AsmOperands, []>,
  Enc32 {

  let isPseudo = 0;
  let isCodeGenOnly = 0;
  let Size = 4;

  // copy relevant pseudo op flags
  // let SubtargetPredicate = ps.SubtargetPredicate;
  // let AsmMatchConverter  = ps.AsmMatchConverter;

  // encoding
  bits<7> sdst;
  bits<8> src0;
/* TODO
  let Inst{7-0} = !if(ps.has_src0, src0, ?);
  let Inst{15-8} = op;
  let Inst{22-16} = !if(ps.has_sdst, sdst, ?);
  let Inst{31-23} = 0x17d; //encoding;
  */
}

//===----------------------------------------------------------------------===//
// SOP2 Instructions
//===----------------------------------------------------------------------===//

class SOP2_Pseudo<string opName, dag outs, dag ins,
                  string asmOps, list<dag> pattern=[]> :
  SOP_Pseudo<opName, outs, ins, asmOps, pattern> {

  let mayLoad = 0;
  let mayStore = 0;
  let hasSideEffects = 0;
  let SALU = 1;
  let SOP2 = 1;
  let SchedRW = [WriteSALU];
  let UseNamedOperandTable = 1;

  let has_sdst = 1;

  // Pseudo instructions have no encodings, but adding this field here allows
  // us to do:
  // let sdst = xxx in {
  // for multiclasses that include both real and pseudo instructions.
  // field bits<7> sdst = 0;
  // let Size = 4; // Do we need size here?
}

class SOP2_Real<bits<7> op, SOP_Pseudo ps> :
  PPTInst <ps.OutOperandList, ps.InOperandList,
          ps.Mnemonic, ps.AsmOperands, []>,
  Enc32 {
  let isPseudo = 0;
  let isCodeGenOnly = 0;

  // copy relevant pseudo op flags
  // let SubtargetPredicate = ps.SubtargetPredicate;
  // let AsmMatchConverter  = ps.AsmMatchConverter;
  let UseNamedOperandTable = ps.UseNamedOperandTable;
  let TSFlags = ps.TSFlags;

  // encoding
  bits<7> sdst;
  bits<8> src0;
  bits<8> src1;
/*
  let Inst{7-0}   = src0;
  let Inst{15-8}  = src1;
  let Inst{22-16} = !if(ps.has_sdst, sdst, ?);
  let Inst{29-23} = op;
  let Inst{31-30} = 0x2; // encoding
  */
}

//===----------------------------------------------------------------------===//
// SOPK Instructions
//===----------------------------------------------------------------------===//

class SOPK_Pseudo <string opName, dag outs, dag ins,
                   string asmOps, list<dag> pattern=[]> :
  PPTInst <outs, ins, "", "", pattern> ,
  PPUMCInstr<opName, PPUEncodingFamily.NONE> {
  let isPseudo = 1;
  let isCodeGenOnly = 1;
  let mayLoad = 0;
  let mayStore = 0;
  let hasSideEffects = 0;
  let SALU = 1;
  let SOPK = 1;
  let SchedRW = [WriteSALU];
  let UseNamedOperandTable = 1;
  string Mnemonic = opName;
  string AsmOperands = asmOps;

  bits<1> has_sdst = 1;
}

class SOPK_Real<bits<5> op, SOPK_Pseudo ps> :
  PPTInst <ps.OutOperandList, ps.InOperandList,
          ps.Mnemonic , ps.AsmOperands, []> {
  let isPseudo = 0;
  let isCodeGenOnly = 0;

  // copy relevant pseudo op flags
  // let SubtargetPredicate = ps.SubtargetPredicate;
  let AsmMatchConverter  = ps.AsmMatchConverter;
  let DisableEncoding    = ps.DisableEncoding;
  let Constraints        = ps.Constraints;

  // encoding
  bits<7>  sdst;
  bits<16> simm16;
  bits<32> imm;
}

class SOPK_Real32<bits<5> op, SOPK_Pseudo ps> :
  SOPK_Real <op, ps>,
  Enc32 {
  let Inst{15-0}  = simm16;
  let Inst{22-16} = !if(ps.has_sdst, sdst, ?);
  let Inst{27-23} = op;
  let Inst{31-28} = 0xb; //encoding
}

class SOPK_Real64<bits<5> op, SOPK_Pseudo ps> :
  SOPK_Real<op, ps>,
  Enc64 {
  let Inst{15-0}  = simm16;
  let Inst{22-16} = !if(ps.has_sdst, sdst, ?);
  let Inst{27-23} = op;
  let Inst{31-28} = 0xb; //encoding
  let Inst{63-32} = imm;
}

class SOPKInstTable <bit is_sopk, string cmpOp = ""> {
  bit IsSOPK = is_sopk;
  string BaseCmpOp = cmpOp;
}

//===----------------------------------------------------------------------===//
// SOPC Instructions
//===----------------------------------------------------------------------===//

class SOPCe <bits<7> op> : Enc32 {
  bits<8> src0;
  bits<8> src1;

  let Inst{7-0} = src0;
  let Inst{15-8} = src1;
  let Inst{22-16} = op;
  let Inst{31-23} = 0x17e;
}

class SOPC <bits<7> op, dag outs, dag ins, string asm,
            list<dag> pattern = []> :
  PPTInst<outs, ins, asm, "", pattern>, SOPCe <op> {
  let mayLoad = 0;
  let mayStore = 0;
  let hasSideEffects = 0;
  let SALU = 1;
  let SOPC = 1;
  let isCodeGenOnly = 0;
  let Defs = [SCC];
  let SchedRW = [WriteSALU];
  let UseNamedOperandTable = 1;
}

//===----------------------------------------------------------------------===//
// SOPP Instructions
//===----------------------------------------------------------------------===//

class Base_SOPP <string asm> {
  string AsmString = asm;
}

class SOPPe <bits<7> op> : Enc32 {
  bits <16> simm16;

  let Inst{15-0} = simm16;
  let Inst{22-16} = op;
  let Inst{31-23} = 0x17f; // encoding
}

class SOPP <bits<7> op, dag ins, string asm, list<dag> pattern = []> :
  PPTInst <(outs), ins, asm, "", pattern >, SOPPe <op>, Base_SOPP <asm> {

  let mayLoad = 0;
  let mayStore = 0;
  let hasSideEffects = 0;
  let SALU = 1;
  let SOPP = 1;
  let Size = 4;
  let SchedRW = [WriteSALU];

  let UseNamedOperandTable = 1;
}


//===----------------------------------------------------------------------===//
// SOP1
class SOP1_Real<bits<8> op, SOP1_Pseudo ps> :
  PPTInst <ps.OutOperandList, ps.InOperandList,
          ps.Mnemonic , ps.AsmOperands, []>,
  Enc32 {

  let isPseudo = 0;
  let isCodeGenOnly = 0;
  let Size = 4;

  // copy relevant pseudo op flags
  let SubtargetPredicate = ps.SubtargetPredicate;
  let AsmMatchConverter  = ps.AsmMatchConverter;

  // encoding
  bits<7> sdst;
  bits<8> src0;

  let Inst{7-0} = !if(ps.has_src0, src0, ?);
  let Inst{15-8} = op;
  let Inst{22-16} = !if(ps.has_sdst, sdst, ?);
  let Inst{31-23} = 0x17d; //encoding;
}


// 32-bit output, 32-bit input
class SOP1_32 <string opName, list<dag> pattern=[], bit tied_in = 0> : SOP1_Pseudo <
  opName, (outs SReg_32:$sdst),
  !if(tied_in, (ins SSrc_b32:$src0, SReg_32:$sdst_in),
               (ins SSrc_b32:$src0)),
  "$sdst, $src0", pattern> {
  let Constraints = !if(tied_in, "$sdst = $sdst_in", "");
}

// 32-bit input, no output.
class SOP1_0_32 <string opName, list<dag> pattern = []> : SOP1_Pseudo <
  opName, (outs), (ins SSrc_b32:$src0),
  "$src0", pattern> {
  let has_sdst = 0;
}

// no output, 32-bit scalar reg input
class SOP1_0_32R <string opName, list<dag> pattern = []> : SOP1_Pseudo <
  opName, (outs), (ins SReg_32:$src0),
  "$src0", pattern> {
  let has_sdst = 0;
}

// 64-bit output, 64-bit input
class SOP1_64 <string opName, list<dag> pattern=[]> : SOP1_Pseudo <
  opName, (outs SReg_64:$sdst), (ins SSrc_b64:$src0),
  "$sdst, $src0", pattern
>;

// 64-bit input, 32-bit output.
class SOP1_32_64 <string opName, list<dag> pattern=[]> : SOP1_Pseudo <
  opName, (outs SReg_32:$sdst), (ins SSrc_b64:$src0),
  "$sdst, $src0", pattern
>;

// 32-bit input, 64-bit output.
class SOP1_64_32 <string opName, list<dag> pattern=[], bit tied_in = 0> : SOP1_Pseudo <
  opName, (outs SReg_64:$sdst),
  !if(tied_in, (ins SSrc_b32:$src0, SReg_64:$sdst_in),
               (ins SSrc_b32:$src0)),
  "$sdst, $src0", pattern> {
  let Constraints = !if(tied_in, "$sdst = $sdst_in", "");
}

// no input, 64-bit output.
class SOP1_64_0 <string opName, list<dag> pattern=[]> : SOP1_Pseudo <
  opName, (outs SReg_64:$sdst), (ins), "$sdst", pattern> {
  let has_src0 = 0;
}

// 64-bit input, no output
class SOP1_0_64 <string opName, RegisterClass rc = SReg_64, list<dag> pattern=[]> : SOP1_Pseudo <
  opName, (outs), (ins rc:$src0), "$src0", pattern> {
  let has_sdst = 0;
}

//===----------------------------------------------------------------------===//
// SOP2

class SOP2_32 <string opName, list<dag> pattern=[]> : SOP2_Pseudo <
  opName, (outs SReg_32:$sdst), (ins SSrc_b32:$src0, SSrc_b32:$src1),
  "$sdst, $src0, $src1", pattern
>;

class SOP2_64 <string opName, list<dag> pattern=[]> : SOP2_Pseudo <
  opName, (outs SReg_64:$sdst), (ins SSrc_b64:$src0, SSrc_b64:$src1),
  "$sdst, $src0, $src1", pattern
>;

class SOP2_64_32 <string opName, list<dag> pattern=[]> : SOP2_Pseudo <
  opName, (outs SReg_64:$sdst), (ins SSrc_b64:$src0, SSrc_b32:$src1),
  "$sdst, $src0, $src1", pattern
>;

class SOP2_64_32_32 <string opName, list<dag> pattern=[]> : SOP2_Pseudo <
  opName, (outs SReg_64:$sdst), (ins SSrc_b32:$src0, SSrc_b32:$src1),
  "$sdst, $src0, $src1", pattern
>;

class UniformUnaryFrag<SDPatternOperator Op> : PatFrag <
  (ops node:$src0),
  (Op $src0),
  [{ return !N->isDivergent(); }]
>;

class UniformBinFrag<SDPatternOperator Op> : PatFrag <
  (ops node:$src0, node:$src1),
  (Op $src0, $src1),
  [{ return !N->isDivergent(); }]
>;

//===----------------------------------------------------------------------===//
// SOP2

class SOPK_32 <string opName, list<dag> pattern=[]> : SOPK_Pseudo <
  opName,
  (outs SReg_32:$sdst),
  (ins s16imm:$simm16),
  "$sdst, $simm16",
  pattern>;

class SOPK_32_BR <string opName, list<dag> pattern=[]> : SOPK_Pseudo <
  opName,
  (outs),
  (ins sopp_brtarget:$simm16, SReg_32:$sdst),
  "$sdst, $simm16",
  pattern> {
  let Defs = [TMSK];
  let Uses = [TMSK];
  let isBranch = 1;
  let isTerminator = 1;
  let SchedRW = [WriteBranch];
}

class SOPK_SCC <string opName, string base_op, bit isSignExt> : SOPK_Pseudo <
  opName,
  (outs),
  !if(isSignExt,
      (ins SReg_32:$sdst, s16imm:$simm16),
      (ins SReg_32:$sdst, u16imm:$simm16)),
  "$sdst, $simm16", []>,
  SOPKInstTable<1, base_op>{
  let Defs = [SCC];
}

class SOPK_32TIE <string opName, list<dag> pattern=[]> : SOPK_Pseudo <
  opName,
  (outs SReg_32:$sdst),
  (ins SReg_32:$src0, s16imm:$simm16),
  "$sdst, $simm16",
  pattern
>;

class SOPK_WAITCNT<string opName, list<dag> pat=[]> :
    SOPK_Pseudo<
        opName,
        (outs),
        (ins SReg_32:$sdst, s16imm:$simm16),
        "$sdst, $simm16",
        pat> {
  let hasSideEffects = 1;
  let mayLoad = 1;
  let mayStore = 1;
  let has_sdst = 1; // First source takes place of sdst in encoding
}

//===----------------------------------------------------------------------===//
// SOPC
class SOPC_Base <bits<7> op, RegisterOperand rc0, RegisterOperand rc1,
                 string opName, list<dag> pattern = []> : SOPC <
  op, (outs), (ins rc0:$src0, rc1:$src1),
  opName#" $src0, $src1", pattern > {
  let Defs = [SCC];
}
class SOPC_Helper <bits<7> op, RegisterOperand rc, ValueType vt,
                    string opName, SDPatternOperator cond> : SOPC_Base <
  op, rc, rc, opName,
  [(set SCC, (si_setcc_uniform vt:$src0, vt:$src1, cond))] > {
}

class SOPC_CMP_32<bits<7> op, string opName,
                  SDPatternOperator cond = COND_NULL, string revOp = opName>
  : SOPC_Helper<op, SSrc_b32, i32, opName, cond>,
    Commutable_REV<revOp, !eq(revOp, opName)>,
    SOPKInstTable<0, opName> {
  let isCompare = 1;
  let isCommutable = 1;
}

class SOPC_CMP_64<bits<7> op, string opName,
                  SDPatternOperator cond = COND_NULL, string revOp = opName>
  : SOPC_Helper<op, SSrc_b64, i64, opName, cond>,
    Commutable_REV<revOp, !eq(revOp, opName)> {
  let isCompare = 1;
  let isCommutable = 1;
}

class SOPC_32<bits<7> op, string opName, list<dag> pattern = []>
  : SOPC_Base<op, SSrc_b32, SSrc_b32, opName, pattern>;

class SOPC_64_32<bits<7> op, string opName, list<dag> pattern = []>
  : SOPC_Base<op, SSrc_b64, SSrc_b32, opName, pattern>;

