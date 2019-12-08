//===-- BUFInstructions.td - Buffer Instruction Defintions ----------------===//
def MUBUFAddr32 : ComplexPattern<i64, 9, "SelectMUBUFAddr32">;
def MUBUFAddr64 : ComplexPattern<i64, 8, "SelectMUBUFAddr64">;
def MUBUFAddr64Atomic : ComplexPattern<i64, 5, "SelectMUBUFAddr64">;

def MUBUFScratchOffen : ComplexPattern<i64, 4, "SelectMUBUFScratchOffen", [], [SDNPWantParent]>;
def MUBUFScratchOffset : ComplexPattern<i64, 3, "SelectMUBUFScratchOffset", [], [SDNPWantParent], 20>;

def MUBUFOffset : ComplexPattern<i64, 7, "SelectMUBUFOffset">;
def MUBUFOffsetNoGLC : ComplexPattern<i64, 3, "SelectMUBUFOffset">;
def MUBUFOffsetAtomic : ComplexPattern<i64, 4, "SelectMUBUFOffset">;

def BUFAddrKind {
  int Offset = 0;
  int OffEn  = 1;
  int IdxEn  = 2;
  int BothEn = 3;
  int Addr64 = 4;
}
class getAddrName<int addrKind> {
  string ret =
    !if(!eq(addrKind, BUFAddrKind.Offset), "offset",
    !if(!eq(addrKind, BUFAddrKind.OffEn),  "offen",
    !if(!eq(addrKind, BUFAddrKind.IdxEn),  "idxen",
    !if(!eq(addrKind, BUFAddrKind.BothEn), "bothen",
    !if(!eq(addrKind, BUFAddrKind.Addr64), "addr64",
    "")))));
}

class MUBUFAddr64Table <bit is_addr64, string Name> {
  bit IsAddr64 = is_addr64;
  string OpName = Name;
}

class MUBUFLdsTable <bit is_lds, string Name> {
  bit IsLds = is_lds;
  string OpName = Name;
}

class MTBUFAddr64Table <bit is_addr64, string Name> {
  bit IsAddr64 = is_addr64;
  string OpName = Name;
}

//===----------------------------------------------------------------------===//
// MTBUF classes
//===----------------------------------------------------------------------===//

class MTBUF_Pseudo <string opName, dag outs, dag ins,
                    string asmOps, list<dag> pattern=[]> :
  PPTInst<outs, ins, "", "", pattern>,
  PPUMCInstr<opName, PPUEncodingFamily.NONE> {

  let isPseudo = 1;
  let isCodeGenOnly = 1;
  let Size = 8;
  let UseNamedOperandTable = 1;

  string Mnemonic = opName;
  string AsmOperands = asmOps;

  let VM_CNT = 1;
  let EXP_CNT = 1;
  let MTBUF = 1;
  let Uses = [TMSK];
  let hasSideEffects = 0;
  let SchedRW = [WriteVMEM];

  let AsmMatchConverter = "cvtMtbuf";

  bits<1> offen       = 0;
  bits<1> idxen       = 0;
  bits<1> addr64      = 0;
  bits<1> has_vdata   = 1;
  bits<1> has_vaddr   = 1;
  bits<1> has_glc     = 1;
  bits<1> has_dlc     = 1;
  bits<1> glc_value   = 0; // the value for glc if no such operand
  bits<1> dlc_value   = 0; // the value for dlc if no such operand
  bits<1> has_srsrc   = 1;
  bits<1> has_soffset = 1;
  bits<1> has_offset  = 1;
  bits<1> has_slc     = 1;
  bits<1> has_tfe     = 1;
}
class MTBUF_Real <MTBUF_Pseudo ps> :
  PPTInst <ps.OutOperandList, ps.InOperandList, ps.Mnemonic , ps.AsmOperands, []> {

  let isPseudo = 0;
  let isCodeGenOnly = 0;

  // copy relevant pseudo op flags
  let SubtargetPredicate = ps.SubtargetPredicate;
  let AsmMatchConverter  = ps.AsmMatchConverter;
  let Constraints        = ps.Constraints;
  let DisableEncoding    = ps.DisableEncoding;
  let TSFlags            = ps.TSFlags;

  bits<12> offset;
  bits<1>  glc;
  bits<1>  dlc;
  bits<7>  format;
  bits<8>  vaddr;
  bits<8>  vdata;
  bits<7>  srsrc;
  bits<1>  slc;
  bits<1>  tfe;
  bits<8>  soffset;

  bits<4> dfmt = format{3-0};
  bits<3> nfmt = format{6-4};
}


// FIXME I change all SReg_128 to SReg_64, compare with BUFInstructions.td to find out
class getMTBUFInsDA<list<RegisterClass> vdataList,
                    list<RegisterClass> vaddrList=[]> {
  RegisterClass vdataClass = !if(!empty(vdataList), ?, !head(vdataList));
  RegisterClass vaddrClass = !if(!empty(vaddrList), ?, !head(vaddrList));
  dag InsNoData = !if(!empty(vaddrList),
    (ins                    SReg_64:$srsrc, SCSrc_b32:$soffset,
         offset:$offset, FORMAT:$format, GLC:$glc, SLC:$slc, TFE:$tfe, DLC:$dlc),
    (ins vaddrClass:$vaddr, SReg_64:$srsrc, SCSrc_b32:$soffset,
         offset:$offset, FORMAT:$format, GLC:$glc, SLC:$slc, TFE:$tfe, DLC:$dlc)
  );
  dag InsData = !if(!empty(vaddrList),
    (ins vdataClass:$vdata,                    SReg_64:$srsrc,
         SCSrc_b32:$soffset, offset:$offset, FORMAT:$format, GLC:$glc,
         SLC:$slc, TFE:$tfe, DLC:$dlc),
    (ins vdataClass:$vdata, vaddrClass:$vaddr, SReg_64:$srsrc,
         SCSrc_b32:$soffset, offset:$offset, FORMAT:$format, GLC:$glc,
         SLC:$slc, TFE:$tfe, DLC:$dlc)
  );
  dag ret = !if(!empty(vdataList), InsNoData, InsData);
}

class getMTBUFIns<int addrKind, list<RegisterClass> vdataList=[]> {
  dag ret =
    !if(!eq(addrKind, BUFAddrKind.Offset), getMTBUFInsDA<vdataList>.ret,
    !if(!eq(addrKind, BUFAddrKind.OffEn),  getMTBUFInsDA<vdataList, [VPR_32]>.ret,
    !if(!eq(addrKind, BUFAddrKind.IdxEn),  getMTBUFInsDA<vdataList, [VPR_32]>.ret,
    !if(!eq(addrKind, BUFAddrKind.BothEn), getMTBUFInsDA<vdataList, [VReg_64]>.ret,
    !if(!eq(addrKind, BUFAddrKind.Addr64), getMTBUFInsDA<vdataList, [VReg_64]>.ret,
    (ins))))));
}

class getMTBUFAsmOps<int addrKind> {
  string Pfx =
    !if(!eq(addrKind, BUFAddrKind.Offset), "off, $srsrc, $format, $soffset",
    !if(!eq(addrKind, BUFAddrKind.OffEn),
            "$vaddr, $srsrc, $format, $soffset offen",
    !if(!eq(addrKind, BUFAddrKind.IdxEn),
            "$vaddr, $srsrc, $format, $soffset idxen",
    !if(!eq(addrKind, BUFAddrKind.BothEn),
            "$vaddr, $srsrc, $format, $soffset idxen offen",
    !if(!eq(addrKind, BUFAddrKind.Addr64),
            "$vaddr, $srsrc, $format, $soffset addr64",
    "")))));
  string ret = Pfx # "$offset";
}

class MTBUF_SetupAddr<int addrKind> {
  bits<1> offen  = !if(!eq(addrKind, BUFAddrKind.OffEn), 1,
                   !if(!eq(addrKind, BUFAddrKind.BothEn), 1 , 0));

  bits<1> idxen  = !if(!eq(addrKind, BUFAddrKind.IdxEn), 1,
                   !if(!eq(addrKind, BUFAddrKind.BothEn), 1 , 0));

  bits<1> addr64 = !if(!eq(addrKind, BUFAddrKind.Addr64), 1, 0);

  bits<1> has_vaddr = !if(!eq(addrKind, BUFAddrKind.Offset), 0, 1);
}

class MTBUF_Load_Pseudo <string opName,
                         int addrKind,
                         RegisterClass vdataClass,
                         list<dag> pattern=[],
                         // Workaround bug bz30254
                         int addrKindCopy = addrKind>
  : MTBUF_Pseudo<opName,
                 (outs vdataClass:$vdata),
                 getMTBUFIns<addrKindCopy>.ret,
                 " $vdata, " # getMTBUFAsmOps<addrKindCopy>.ret # "$glc$slc$tfe$dlc",
                 pattern>,
    MTBUF_SetupAddr<addrKindCopy> {
  let PseudoInstr = opName # "_" # getAddrName<addrKindCopy>.ret;
  let mayLoad = 1;
  let mayStore = 0;
}

// TODO change all srsrc from v4i32 to v2i32
multiclass MTBUF_Pseudo_Loads<string opName, RegisterClass vdataClass,
                              ValueType load_vt = i32,
                              SDPatternOperator ld = null_frag> {

  def _OFFSET : MTBUF_Load_Pseudo <opName, BUFAddrKind.Offset, vdataClass,
    [(set load_vt:$vdata,
     (ld (MUBUFOffset v2i32:$srsrc, i32:$soffset, i16:$offset, i8:$format,
                      i1:$glc, i1:$slc, i1:$tfe, i1:$dlc)))]>,
    MTBUFAddr64Table<0, NAME>;

  def _ADDR64 : MTBUF_Load_Pseudo <opName, BUFAddrKind.Addr64, vdataClass,
    [(set load_vt:$vdata,
     (ld (MUBUFAddr64 v2i32:$srsrc, i64:$vaddr, i32:$soffset, i16:$offset,
                      i8:$format, i1:$glc, i1:$slc, i1:$tfe, i1:$dlc)))]>,
    MTBUFAddr64Table<1, NAME>;

  def _OFFEN  : MTBUF_Load_Pseudo <opName, BUFAddrKind.OffEn, vdataClass>;
  def _IDXEN  : MTBUF_Load_Pseudo <opName, BUFAddrKind.IdxEn, vdataClass>;
  def _BOTHEN : MTBUF_Load_Pseudo <opName, BUFAddrKind.BothEn, vdataClass>;

  let DisableWQM = 1 in {
    def _OFFSET_exact : MTBUF_Load_Pseudo <opName, BUFAddrKind.Offset, vdataClass>;
    def _OFFEN_exact  : MTBUF_Load_Pseudo <opName, BUFAddrKind.OffEn, vdataClass>;
    def _IDXEN_exact  : MTBUF_Load_Pseudo <opName, BUFAddrKind.IdxEn, vdataClass>;
    def _BOTHEN_exact : MTBUF_Load_Pseudo <opName, BUFAddrKind.BothEn, vdataClass>;
  }
}

class MTBUF_Store_Pseudo <string opName,
                          int addrKind,
                          RegisterClass vdataClass,
                          list<dag> pattern=[],
                          // Workaround bug bz30254
                          int addrKindCopy = addrKind,
                          RegisterClass vdataClassCopy = vdataClass>
  : MTBUF_Pseudo<opName,
                 (outs),
                 getMTBUFIns<addrKindCopy, [vdataClassCopy]>.ret,
                 " $vdata, " # getMTBUFAsmOps<addrKindCopy>.ret # "$glc$slc$tfe$dlc",
                 pattern>,
    MTBUF_SetupAddr<addrKindCopy> {
  let PseudoInstr = opName # "_" # getAddrName<addrKindCopy>.ret;
  let mayLoad = 0;
  let mayStore = 1;
}

multiclass MTBUF_Pseudo_Stores<string opName, RegisterClass vdataClass,
                               ValueType store_vt = i32,
                               SDPatternOperator st = null_frag> {

  def _OFFSET : MTBUF_Store_Pseudo <opName, BUFAddrKind.Offset, vdataClass,
    [(st store_vt:$vdata, (MUBUFOffset v2i32:$srsrc, i32:$soffset,
                                       i16:$offset, i8:$format, i1:$glc,
                                       i1:$slc, i1:$tfe, i1:$dlc))]>,
    MTBUFAddr64Table<0, NAME>;

  def _ADDR64 : MTBUF_Store_Pseudo <opName, BUFAddrKind.Addr64, vdataClass,
    [(st store_vt:$vdata, (MUBUFAddr64 v2i32:$srsrc, i64:$vaddr, i32:$soffset,
                                       i16:$offset, i8:$format, i1:$glc,
                                       i1:$slc, i1:$tfe, i1:$dlc))]>,
    MTBUFAddr64Table<1, NAME>;

  def _OFFEN  : MTBUF_Store_Pseudo <opName, BUFAddrKind.OffEn, vdataClass>;
  def _IDXEN  : MTBUF_Store_Pseudo <opName, BUFAddrKind.IdxEn, vdataClass>;
  def _BOTHEN : MTBUF_Store_Pseudo <opName, BUFAddrKind.BothEn, vdataClass>;

  let DisableWQM = 1 in {
    def _OFFSET_exact : MTBUF_Store_Pseudo <opName, BUFAddrKind.Offset, vdataClass>;
    def _OFFEN_exact  : MTBUF_Store_Pseudo <opName, BUFAddrKind.OffEn, vdataClass>;
    def _IDXEN_exact  : MTBUF_Store_Pseudo <opName, BUFAddrKind.IdxEn, vdataClass>;
    def _BOTHEN_exact : MTBUF_Store_Pseudo <opName, BUFAddrKind.BothEn, vdataClass>;
  }
}

//===----------------------------------------------------------------------===//
// MUBUF classes
//===----------------------------------------------------------------------===//

class MUBUFGetBaseOpcode<string Op> {
  string ret = !subst("DWORDX2", "DWORD",
    !subst("DWORDX3", "DWORD",
    !subst("DWORDX4", "DWORD", Op)));
}

class MUBUF_Pseudo <string opName, dag outs, dag ins,
                    string asmOps, list<dag> pattern=[]> :
  PPTInst<outs, ins, "", "", pattern>,
  PPUMCInstr<opName, PPUEncodingFamily.NONE> {

  let isPseudo = 1;
  let isCodeGenOnly = 1;
  let Size = 8;
  let UseNamedOperandTable = 1;

  string Mnemonic = opName;
  string AsmOperands = asmOps;

  Instruction Opcode = !cast<Instruction>(NAME);
  Instruction BaseOpcode = !cast<Instruction>(MUBUFGetBaseOpcode<NAME>.ret);

  let VM_CNT = 1;
  let EXP_CNT = 1;
  let MUBUF = 1;
  let Uses = [TMSK];
  let hasSideEffects = 0;
  let SchedRW = [WriteVMEM];

  let AsmMatchConverter = "cvtMubuf";

  bits<1> offen       = 0;
  bits<1> idxen       = 0;
  bits<1> addr64      = 0;
  bits<1> lds         = 0;
  bits<1> has_vdata   = 1;
  bits<1> has_vaddr   = 1;
  bits<1> has_glc     = 1;
  bits<1> has_dlc     = 1;
  bits<1> glc_value   = 0; // the value for glc if no such operand
  bits<1> dlc_value   = 0; // the value for dlc if no such operand
  bits<1> has_srsrc   = 1;
  bits<1> has_soffset = 1;
  bits<1> has_offset  = 1;
  bits<1> has_slc     = 1;
  bits<1> has_tfe     = 1;
  bits<4> elements    = 0;
}

class MUBUF_Real <MUBUF_Pseudo ps> :
  PPTInst <ps.OutOperandList, ps.InOperandList, ps.Mnemonic , ps.AsmOperands, []> {

  let isPseudo = 0;
  let isCodeGenOnly = 0;

  // copy relevant pseudo op flags
  let SubtargetPredicate = ps.SubtargetPredicate;
  let AsmMatchConverter  = ps.AsmMatchConverter;
  let Constraints        = ps.Constraints;
  let DisableEncoding    = ps.DisableEncoding;
  let TSFlags            = ps.TSFlags;

  bits<12> offset;
  bits<1>  glc;
  bits<1>  dlc;
  bits<8>  vaddr;
  bits<8>  vdata;
  bits<7>  srsrc;
  bits<1>  slc;
  bits<1>  tfe;
  bits<8>  soffset;
}

// For cache invalidation instructions.
class MUBUF_Invalidate <string opName, SDPatternOperator node = null_frag> :
  MUBUF_Pseudo<opName, (outs), (ins), "", [(node)]> {

  let AsmMatchConverter = "";

  let hasSideEffects = 1;
  let mayStore = 1;

  // Set everything to 0.
  let offen       = 0;
  let idxen       = 0;
  let addr64      = 0;
  let has_vdata   = 0;
  let has_vaddr   = 0;
  let has_glc     = 0;
  let has_dlc     = 0;
  let glc_value   = 0;
  let dlc_value   = 0;
  let has_srsrc   = 0;
  let has_soffset = 0;
  let has_offset  = 0;
  let has_slc     = 0;
  let has_tfe     = 0;
}

class getMUBUFInsDA<list<RegisterClass> vdataList,
                    list<RegisterClass> vaddrList=[],
                    bit isLds = 0> {
  RegisterClass vdataClass = !if(!empty(vdataList), ?, !head(vdataList));
  RegisterClass vaddrClass = !if(!empty(vaddrList), ?, !head(vaddrList));
  dag InsNoData = !if(!empty(vaddrList),
    (ins                    SReg_64:$srsrc, SCSrc_b32:$soffset,
         offset:$offset, GLC:$glc, SLC:$slc),
    (ins vaddrClass:$vaddr, SReg_64:$srsrc, SCSrc_b32:$soffset,
         offset:$offset, GLC:$glc, SLC:$slc)
  );
  dag InsData = !if(!empty(vaddrList),
    (ins vdataClass:$vdata,                    SReg_64:$srsrc,
         SCSrc_b32:$soffset, offset:$offset, GLC:$glc, SLC:$slc),
    (ins vdataClass:$vdata, vaddrClass:$vaddr, SReg_64:$srsrc,
         SCSrc_b32:$soffset, offset:$offset, GLC:$glc, SLC:$slc)
  );
  dag ret = !con(
              !if(!empty(vdataList), InsNoData, InsData),
              !if(isLds, (ins DLC:$dlc), (ins TFE:$tfe, DLC:$dlc))
             );
}

class getMUBUFElements<ValueType vt> {
  // eq does not support ValueType for some reason.
  string vtAsStr = !cast<string>(vt);

  int ret =
    !if(!eq(vtAsStr, "f16"), 1,
      !if(!eq(vtAsStr, "v2f16"), 2,
        !if(!eq(vtAsStr, "v3f16"), 3,
          !if(!eq(vtAsStr, "v4f16"), 4,
            !if(!eq(vt.Size, 32), 1,
              !if(!eq(vt.Size, 64), 2,
                !if(!eq(vt.Size, 96), 3,
                  !if(!eq(vt.Size, 128), 4, 0)
                )
              )
            )
          )
        )
      )
    );
}

class getMUBUFIns<int addrKind, list<RegisterClass> vdataList=[], bit isLds = 0> {
  dag ret =
    !if(!eq(addrKind, BUFAddrKind.Offset), getMUBUFInsDA<vdataList, [], isLds>.ret,
    !if(!eq(addrKind, BUFAddrKind.OffEn),  getMUBUFInsDA<vdataList, [VPR_32], isLds>.ret,
    !if(!eq(addrKind, BUFAddrKind.IdxEn),  getMUBUFInsDA<vdataList, [VPR_32], isLds>.ret,
    !if(!eq(addrKind, BUFAddrKind.BothEn), getMUBUFInsDA<vdataList, [VReg_64], isLds>.ret,
    !if(!eq(addrKind, BUFAddrKind.Addr64), getMUBUFInsDA<vdataList, [VReg_64], isLds>.ret,
    (ins))))));
}

class getMUBUFAsmOps<int addrKind> {
  string Pfx =
    !if(!eq(addrKind, BUFAddrKind.Offset), "off, $srsrc, $soffset",
    !if(!eq(addrKind, BUFAddrKind.OffEn),  "$vaddr, $srsrc, $soffset offen",
    !if(!eq(addrKind, BUFAddrKind.IdxEn),  "$vaddr, $srsrc, $soffset idxen",
    !if(!eq(addrKind, BUFAddrKind.BothEn), "$vaddr, $srsrc, $soffset idxen offen",
    !if(!eq(addrKind, BUFAddrKind.Addr64), "$vaddr, $srsrc, $soffset addr64",
    "")))));
  string ret = Pfx # "$offset";
}

class MUBUF_SetupAddr<int addrKind> {
  bits<1> offen  = !if(!eq(addrKind, BUFAddrKind.OffEn), 1,
                   !if(!eq(addrKind, BUFAddrKind.BothEn), 1 , 0));

  bits<1> idxen  = !if(!eq(addrKind, BUFAddrKind.IdxEn), 1,
                   !if(!eq(addrKind, BUFAddrKind.BothEn), 1 , 0));

  bits<1> addr64 = !if(!eq(addrKind, BUFAddrKind.Addr64), 1, 0);

  bits<1> has_vaddr = !if(!eq(addrKind, BUFAddrKind.Offset), 0, 1);
}

class MUBUF_Load_Pseudo <string opName,
                         int addrKind,
                         ValueType vdata_vt,
                         bit HasTiedDest = 0,
                         bit isLds = 0,
                         list<dag> pattern=[],
                         // Workaround bug bz30254
                         int addrKindCopy = addrKind>
  : MUBUF_Pseudo<opName,
                 (outs getVregSrcForVT<vdata_vt>.ret:$vdata),
                 !con(getMUBUFIns<addrKindCopy, [], isLds>.ret,
                      !if(HasTiedDest, (ins getVregSrcForVT<vdata_vt>.ret:$vdata_in), (ins))),
                 " $vdata, " # getMUBUFAsmOps<addrKindCopy>.ret # "$glc$slc" #
                   !if(isLds, " lds", "$tfe") # "$dlc",
                 pattern>,
    MUBUF_SetupAddr<addrKindCopy> {
  let PseudoInstr = opName # !if(isLds, "_lds", "") #
                    "_" # getAddrName<addrKindCopy>.ret;
  let AsmMatchConverter = !if(isLds, "cvtMubufLds", "cvtMubuf");

  let Constraints = !if(HasTiedDest, "$vdata = $vdata_in", "");
  let mayLoad = 1;
  let mayStore = 0;
  let maybeAtomic = 1;
  let Uses = !if(isLds, [TMSK, M0], [TMSK]);
  let has_tfe = !if(isLds, 0, 1);
  let lds = isLds;
  let elements = getMUBUFElements<vdata_vt>.ret;
}


class MUBUF_Offset_Load_Pat <Instruction inst, ValueType load_vt = i32, SDPatternOperator ld = null_frag> : Pat <
  (load_vt (ld (MUBUFOffset v2i32:$srsrc, i32:$soffset, i16:$offset, i1:$glc, i1:$slc, i1:$tfe, i1:$dlc))),
  (load_vt (inst v2i32:$srsrc, i32:$soffset, i16:$offset, i1:$glc, i1:$slc, i1:$tfe, i1:$dlc))
>;

class MUBUF_Addr64_Load_Pat <Instruction inst,
                            ValueType load_vt = i32,
                            SDPatternOperator ld = null_frag> : Pat <
  (load_vt (ld (MUBUFAddr64 v2i32:$srsrc, i64:$vaddr, i32:$soffset, i16:$offset, i1:$glc, i1:$slc, i1:$tfe, i1:$dlc))),
  (load_vt (inst i64:$vaddr, v2i32:$srsrc, i32:$soffset, i16:$offset, i1:$glc, i1:$slc, i1:$tfe, i1:$dlc))
>;

multiclass MUBUF_Pseudo_Load_Pats<string BaseInst, ValueType load_vt = i32, SDPatternOperator ld = null_frag> {
  def : MUBUF_Offset_Load_Pat<!cast<Instruction>(BaseInst#"_OFFSET"), load_vt, ld>;
  def : MUBUF_Addr64_Load_Pat<!cast<Instruction>(BaseInst#"_ADDR64"), load_vt, ld>;
}


// FIXME: tfe can't be an operand because it requires a separate
// opcode because it needs an N+1 register class dest register.
multiclass MUBUF_Pseudo_Loads<string opName,
                              ValueType load_vt = i32,
                              SDPatternOperator ld = null_frag,
                              bit TiedDest = 0,
                              bit isLds = 0> {

  def _OFFSET : MUBUF_Load_Pseudo <opName, BUFAddrKind.Offset, load_vt, TiedDest, isLds>,
    MUBUFAddr64Table<0, NAME # !if(isLds, "_LDS", "")>;

  def _ADDR64 : MUBUF_Load_Pseudo <opName, BUFAddrKind.Addr64, load_vt, TiedDest, isLds>,
    MUBUFAddr64Table<1, NAME # !if(isLds, "_LDS", "")>;

  def _OFFEN  : MUBUF_Load_Pseudo <opName, BUFAddrKind.OffEn, load_vt, TiedDest, isLds>;
  def _IDXEN  : MUBUF_Load_Pseudo <opName, BUFAddrKind.IdxEn, load_vt, TiedDest, isLds>;
  def _BOTHEN : MUBUF_Load_Pseudo <opName, BUFAddrKind.BothEn, load_vt, TiedDest, isLds>;

  let DisableWQM = 1 in {
    def _OFFSET_exact : MUBUF_Load_Pseudo <opName, BUFAddrKind.Offset, load_vt, TiedDest, isLds>;
    def _OFFEN_exact  : MUBUF_Load_Pseudo <opName, BUFAddrKind.OffEn, load_vt, TiedDest, isLds>;
    def _IDXEN_exact  : MUBUF_Load_Pseudo <opName, BUFAddrKind.IdxEn, load_vt, TiedDest, isLds>;
    def _BOTHEN_exact : MUBUF_Load_Pseudo <opName, BUFAddrKind.BothEn, load_vt, TiedDest, isLds>;
  }
}

multiclass MUBUF_Pseudo_Loads_Lds<string opName, ValueType load_vt = i32,
                                  SDPatternOperator ld_nolds = null_frag,
                                  SDPatternOperator ld_lds = null_frag> {
  defm NAME : MUBUF_Pseudo_Loads<opName, load_vt, ld_nolds>;
  defm _LDS : MUBUF_Pseudo_Loads<opName, load_vt, ld_lds, 0, 1>;
}

class MUBUF_Store_Pseudo <string opName,
                          int addrKind,
                          ValueType store_vt,
                          list<dag> pattern=[],
                          // Workaround bug bz30254
                          int addrKindCopy = addrKind>
  : MUBUF_Pseudo<opName,
                 (outs),
                 getMUBUFIns<addrKindCopy, [getVregSrcForVT<store_vt>.ret]>.ret,
                 " $vdata, " # getMUBUFAsmOps<addrKindCopy>.ret # "$glc$slc$tfe$dlc",
                 pattern>,
    MUBUF_SetupAddr<addrKindCopy> {
  let PseudoInstr = opName # "_" # getAddrName<addrKindCopy>.ret;
  let mayLoad = 0;
  let mayStore = 1;
  let maybeAtomic = 1;
  let elements = getMUBUFElements<store_vt>.ret;
}

multiclass MUBUF_Pseudo_Stores<string opName,
                               ValueType store_vt = i32,
                               SDPatternOperator st = null_frag> {

  def _OFFSET : MUBUF_Store_Pseudo <opName, BUFAddrKind.Offset, store_vt,
    [(st store_vt:$vdata, (MUBUFOffset v2i32:$srsrc, i32:$soffset,
                                       i16:$offset, i1:$glc, i1:$slc, i1:$tfe, i1:$dlc))]>,
    MUBUFAddr64Table<0, NAME>;

  def _ADDR64 : MUBUF_Store_Pseudo <opName, BUFAddrKind.Addr64, store_vt,
    [(st store_vt:$vdata, (MUBUFAddr64 v2i32:$srsrc, i64:$vaddr, i32:$soffset,
                                       i16:$offset, i1:$glc, i1:$slc, i1:$tfe, i1:$dlc))]>,
    MUBUFAddr64Table<1, NAME>;

  def _OFFEN  : MUBUF_Store_Pseudo <opName, BUFAddrKind.OffEn, store_vt>;
  def _IDXEN  : MUBUF_Store_Pseudo <opName, BUFAddrKind.IdxEn, store_vt>;
  def _BOTHEN : MUBUF_Store_Pseudo <opName, BUFAddrKind.BothEn, store_vt>;

  let DisableWQM = 1 in {
    def _OFFSET_exact : MUBUF_Store_Pseudo <opName, BUFAddrKind.Offset, store_vt>;
    def _OFFEN_exact  : MUBUF_Store_Pseudo <opName, BUFAddrKind.OffEn, store_vt>;
    def _IDXEN_exact  : MUBUF_Store_Pseudo <opName, BUFAddrKind.IdxEn, store_vt>;
    def _BOTHEN_exact : MUBUF_Store_Pseudo <opName, BUFAddrKind.BothEn, store_vt>;
  }
}

class MUBUF_Pseudo_Store_Lds<string opName>
  : MUBUF_Pseudo<opName,
                 (outs),
                 (ins SReg_64:$srsrc, SCSrc_b32:$soffset, offset:$offset, GLC:$glc, SLC:$slc),
                 " $srsrc, $soffset$offset lds$glc$slc"> {
  let mayLoad = 0;
  let mayStore = 1;
  let maybeAtomic = 1;

  let has_vdata = 0;
  let has_vaddr = 0;
  let has_tfe = 0;
  let lds = 1;

  let Uses = [TMSK, M0];
  let AsmMatchConverter = "cvtMubufLds";
}

class getMUBUFAtomicInsDA<RegisterClass vdataClass, bit vdata_in,
                          list<RegisterClass> vaddrList=[]> {
  RegisterClass vaddrClass = !if(!empty(vaddrList), ?, !head(vaddrList));
  dag ret = !if(vdata_in,
    !if(!empty(vaddrList),
      (ins vdataClass:$vdata_in,
           SReg_64:$srsrc, SCSrc_b32:$soffset, offset:$offset, SLC:$slc),
      (ins vdataClass:$vdata_in, vaddrClass:$vaddr,
           SReg_64:$srsrc, SCSrc_b32:$soffset, offset:$offset, SLC:$slc)
    ),
    !if(!empty(vaddrList),
      (ins vdataClass:$vdata,
           SReg_64:$srsrc, SCSrc_b32:$soffset, offset:$offset, SLC:$slc),
      (ins vdataClass:$vdata, vaddrClass:$vaddr,
           SReg_64:$srsrc, SCSrc_b32:$soffset, offset:$offset, SLC:$slc)
  ));
}

class getMUBUFAtomicIns<int addrKind,
                        RegisterClass vdataClass,
                        bit vdata_in,
                        // Workaround bug bz30254
                        RegisterClass vdataClassCopy=vdataClass> {
  dag ret =
    !if(!eq(addrKind, BUFAddrKind.Offset),
            getMUBUFAtomicInsDA<vdataClassCopy, vdata_in>.ret,
    !if(!eq(addrKind, BUFAddrKind.OffEn),
            getMUBUFAtomicInsDA<vdataClassCopy, vdata_in, [VPR_32]>.ret,
    !if(!eq(addrKind, BUFAddrKind.IdxEn),
            getMUBUFAtomicInsDA<vdataClassCopy, vdata_in, [VPR_32]>.ret,
    !if(!eq(addrKind, BUFAddrKind.BothEn),
            getMUBUFAtomicInsDA<vdataClassCopy, vdata_in, [VReg_64]>.ret,
    !if(!eq(addrKind, BUFAddrKind.Addr64),
            getMUBUFAtomicInsDA<vdataClassCopy, vdata_in, [VReg_64]>.ret,
    (ins))))));
}

class MUBUF_Atomic_Pseudo<string opName,
                          int addrKind,
                          dag outs,
                          dag ins,
                          string asmOps,
                          list<dag> pattern=[],
                          // Workaround bug bz30254
                          int addrKindCopy = addrKind>
  : MUBUF_Pseudo<opName, outs, ins, asmOps, pattern>,
    MUBUF_SetupAddr<addrKindCopy> {
  let mayStore = 1;
  let mayLoad = 1;
  let hasPostISelHook = 1;
  let hasSideEffects = 1;
  let DisableWQM = 1;
  let has_glc = 0;
  let has_dlc = 0;
  let has_tfe = 0;
  let maybeAtomic = 1;
}

class MUBUF_AtomicNoRet_Pseudo<string opName, int addrKind,
                               RegisterClass vdataClass,
                               list<dag> pattern=[],
                               // Workaround bug bz30254
                               int addrKindCopy = addrKind,
                               RegisterClass vdataClassCopy = vdataClass>
  : MUBUF_Atomic_Pseudo<opName, addrKindCopy,
                        (outs),
                        getMUBUFAtomicIns<addrKindCopy, vdataClassCopy, 0>.ret,
                        " $vdata, " # getMUBUFAsmOps<addrKindCopy>.ret # "$slc",
                        pattern>,
    AtomicNoRet<opName # "_" # getAddrName<addrKindCopy>.ret, 0> {
  let PseudoInstr = opName # "_" # getAddrName<addrKindCopy>.ret;
  let glc_value = 0;
  let dlc_value = 0;
  let AsmMatchConverter = "cvtMubufAtomic";
}

class MUBUF_AtomicRet_Pseudo<string opName, int addrKind,
                             RegisterClass vdataClass,
                             list<dag> pattern=[],
                             // Workaround bug bz30254
                             int addrKindCopy = addrKind,
                             RegisterClass vdataClassCopy = vdataClass>
  : MUBUF_Atomic_Pseudo<opName, addrKindCopy,
                        (outs vdataClassCopy:$vdata),
                        getMUBUFAtomicIns<addrKindCopy, vdataClassCopy, 1>.ret,
                        " $vdata, " # getMUBUFAsmOps<addrKindCopy>.ret # " glc$slc",
                        pattern>,
    AtomicNoRet<opName # "_" # getAddrName<addrKindCopy>.ret, 1> {
  let PseudoInstr = opName # "_rtn_" # getAddrName<addrKindCopy>.ret;
  let glc_value = 1;
  let dlc_value = 0;
  let Constraints = "$vdata = $vdata_in";
  let DisableEncoding = "$vdata_in";
  let AsmMatchConverter = "cvtMubufAtomicReturn";
}

multiclass MUBUF_Pseudo_Atomics_NO_RTN <string opName,
                                        RegisterClass vdataClass,
                                        ValueType vdataType,
                                        SDPatternOperator atomic,
                                        bit isFP = getIsFP<vdataType>.ret> {
  let FPAtomic = isFP in
  def _OFFSET : MUBUF_AtomicNoRet_Pseudo <opName, BUFAddrKind.Offset, vdataClass>,
                MUBUFAddr64Table <0, NAME>;

  let FPAtomic = isFP in
  def _ADDR64 : MUBUF_AtomicNoRet_Pseudo <opName, BUFAddrKind.Addr64, vdataClass>,
                MUBUFAddr64Table <1, NAME>;

  let FPAtomic = isFP in
  def _OFFEN  : MUBUF_AtomicNoRet_Pseudo <opName, BUFAddrKind.OffEn,  vdataClass>;

  let FPAtomic = isFP in

  def _IDXEN  : MUBUF_AtomicNoRet_Pseudo <opName, BUFAddrKind.IdxEn,  vdataClass>;

  let FPAtomic = isFP in
  def _BOTHEN : MUBUF_AtomicNoRet_Pseudo <opName, BUFAddrKind.BothEn, vdataClass>;
}

multiclass MUBUF_Pseudo_Atomics_RTN <string opName,
                                     RegisterClass vdataClass,
                                     ValueType vdataType,
                                     SDPatternOperator atomic,
                                     bit isFP = getIsFP<vdataType>.ret> {
  let FPAtomic = isFP in
  def _OFFSET_RTN : MUBUF_AtomicRet_Pseudo <opName, BUFAddrKind.Offset, vdataClass,
    [(set vdataType:$vdata,
     (atomic (MUBUFOffsetAtomic v2i32:$srsrc, i32:$soffset, i16:$offset, i1:$slc),
             vdataType:$vdata_in))]>,
    MUBUFAddr64Table <0, NAME # "_RTN">;

  let FPAtomic = isFP in
  def _ADDR64_RTN : MUBUF_AtomicRet_Pseudo <opName, BUFAddrKind.Addr64, vdataClass,
    [(set vdataType:$vdata,
     (atomic (MUBUFAddr64Atomic v2i32:$srsrc, i64:$vaddr, i32:$soffset, i16:$offset, i1:$slc),
             vdataType:$vdata_in))]>,
    MUBUFAddr64Table <1, NAME # "_RTN">;

  let FPAtomic = isFP in
  def _OFFEN_RTN  : MUBUF_AtomicRet_Pseudo <opName, BUFAddrKind.OffEn,  vdataClass>;

  let FPAtomic = isFP in
  def _IDXEN_RTN  : MUBUF_AtomicRet_Pseudo <opName, BUFAddrKind.IdxEn,  vdataClass>;

  let FPAtomic = isFP in
  def _BOTHEN_RTN : MUBUF_AtomicRet_Pseudo <opName, BUFAddrKind.BothEn, vdataClass>;
}

multiclass MUBUF_Pseudo_Atomics <string opName,
                                 RegisterClass vdataClass,
                                 ValueType vdataType,
                                 SDPatternOperator atomic> :
  MUBUF_Pseudo_Atomics_NO_RTN<opName, vdataClass, vdataType, atomic>,
  MUBUF_Pseudo_Atomics_RTN<opName, vdataClass, vdataType, atomic>;

