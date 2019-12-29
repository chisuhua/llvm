class DS_Pseudo <string opName, dag outs, dag ins, string asmOps, list<dag> pattern=[]> :
  PPTInst <outs, ins, "", "", pattern>,
  PPUMCInstr <opName, PPUEncodingFamily.NONE> {

  let LGKM_CNT = 1;
  let DS = 1;
  let Size = 8;
  let UseNamedOperandTable = 1;

  // Most instruction load and store data, so set this as the default.
  let mayLoad = 1;
  let mayStore = 1;
  let maybeAtomic = 1;

  let hasSideEffects = 0;
  let SchedRW = [WriteLDS];

  let isPseudo = 1;
  let isCodeGenOnly = 1;

  let AsmMatchConverter = "cvtDS";

  string Mnemonic = opName;
  string AsmOperands = asmOps;

  // Well these bits a kind of hack because it would be more natural
  // to test "outs" and "ins" dags for the presence of particular operands
  bits<1> has_vdst = 1;
  bits<1> has_addr = 1;
  bits<1> has_data0 = 1;
  bits<1> has_data1 = 1;

  bits<1> has_gws_data0 = 0; // data0 is encoded as addr

  bits<1> has_offset  = 1; // has "offset" that should be split to offset0,1
  bits<1> has_offset0 = 1;
  bits<1> has_offset1 = 1;

  bits<1> has_gds = 1;
  bits<1> gdsValue = 0; // if has_gds == 0 set gds to this value

  bits<1> has_m0_read = 1;

  let Uses = !if(has_m0_read, [M0, TMSK], [TMSK]);
}

class DS_Real <DS_Pseudo ds> :
  PPTInst <ds.OutOperandList, ds.InOperandList, ds.Mnemonic, ds.AsmOperands, []>,
  Enc64 {

  let isPseudo = 0;
  let isCodeGenOnly = 0;

  // copy relevant pseudo op flags
  let SubtargetPredicate = ds.SubtargetPredicate;
  let OtherPredicates = ds.OtherPredicates;
  let AsmMatchConverter  = ds.AsmMatchConverter;

  // encoding fields
  bits<8> vdst;
  bits<1> gds;
  bits<8> addr;
  bits<8> data0;
  bits<8> data1;
  bits<8> offset0;
  bits<8> offset1;

  bits<16> offset;
  let offset0 = !if(ds.has_offset, offset{7-0}, ?);
  let offset1 = !if(ds.has_offset, offset{15-8}, ?);
}

// DS Pseudo instructions

class DS_1A1D_NORET<string opName, RegisterClass rc = VPR_32>
: DS_Pseudo<opName,
  (outs),
  (ins VPR_32:$addr, rc:$data0, offset:$offset, gds:$gds),
  "$addr, $data0$offset$gds"> {

  let has_data1 = 0;
  let has_vdst = 0;
}

multiclass DS_1A1D_NORET_mc<string opName, RegisterClass rc = VPR_32> {
  def "" : DS_1A1D_NORET<opName, rc>,
           AtomicNoRet<opName, 0>;

  let has_m0_read = 0 in {
    def _ppu : DS_1A1D_NORET<opName, rc>,
                AtomicNoRet<opName#"_ppu", 0>;
  }
}

class DS_1A2D_NORET<string opName, RegisterClass rc = VPR_32>
: DS_Pseudo<opName,
  (outs),
  (ins VPR_32:$addr, rc:$data0, rc:$data1, offset:$offset, gds:$gds),
  "$addr, $data0, $data1"#"$offset"#"$gds"> {

  let has_vdst = 0;
}

multiclass DS_1A2D_NORET_mc<string opName, RegisterClass rc = VPR_32> {
  def "" : DS_1A2D_NORET<opName, rc>,
           AtomicNoRet<opName, 0>;

  let has_m0_read = 0 in {
    def _ppu : DS_1A2D_NORET<opName, rc>,
                AtomicNoRet<opName#"_ppu", 0>;
  }
}

class DS_1A2D_Off8_NORET <string opName, RegisterClass rc = VPR_32>
: DS_Pseudo<opName,
  (outs),
  (ins VPR_32:$addr, rc:$data0, rc:$data1,
       offset0:$offset0, offset1:$offset1, gds:$gds),
  "$addr, $data0, $data1$offset0$offset1$gds"> {

  let has_vdst = 0;
  let has_offset = 0;
  let AsmMatchConverter = "cvtDSOffset01";
}

multiclass DS_1A2D_Off8_NORET_mc <string opName, RegisterClass rc = VPR_32> {
  def "" : DS_1A2D_Off8_NORET<opName, rc>;

  let has_m0_read = 0 in {
    def _ppu : DS_1A2D_Off8_NORET<opName, rc>;
  }
}

class DS_1A1D_RET <string opName, RegisterClass rc = VPR_32>
: DS_Pseudo<opName,
  (outs rc:$vdst),
  (ins VPR_32:$addr, rc:$data0, offset:$offset, gds:$gds),
  "$vdst, $addr, $data0$offset$gds"> {

  let hasPostISelHook = 1;
  let has_data1 = 0;
}

multiclass DS_1A1D_RET_mc <string opName, RegisterClass rc = VPR_32,
                           string NoRetOp = ""> {
  def "" : DS_1A1D_RET<opName, rc>,
    AtomicNoRet<NoRetOp, !if(!eq(NoRetOp, ""), 0, 1)>;

  let has_m0_read = 0 in {
    def _ppu : DS_1A1D_RET<opName, rc>,
      AtomicNoRet<!if(!eq(NoRetOp, ""), "", NoRetOp#"_ppu"),
                  !if(!eq(NoRetOp, ""), 0, 1)>;
  }
}

class DS_1A2D_RET<string opName,
                  RegisterClass rc = VPR_32,
                  RegisterClass src = rc>
: DS_Pseudo<opName,
  (outs rc:$vdst),
  (ins VPR_32:$addr, src:$data0, src:$data1, offset:$offset, gds:$gds),
  "$vdst, $addr, $data0, $data1$offset$gds"> {

  let hasPostISelHook = 1;
}

multiclass DS_1A2D_RET_mc<string opName,
                          RegisterClass rc = VPR_32,
                          string NoRetOp = "",
                          RegisterClass src = rc> {
  def "" : DS_1A2D_RET<opName, rc, src>,
    AtomicNoRet<NoRetOp, !if(!eq(NoRetOp, ""), 0, 1)>;

  let has_m0_read = 0 in {
    def _ppu : DS_1A2D_RET<opName, rc, src>,
      AtomicNoRet<NoRetOp#"_ppu", !if(!eq(NoRetOp, ""), 0, 1)>;
  }
}

class DS_1A2D_Off8_RET<string opName,
                       RegisterClass rc = VPR_32,
                       RegisterClass src = rc>
: DS_Pseudo<opName,
  (outs rc:$vdst),
  (ins VPR_32:$addr, src:$data0, src:$data1, offset0:$offset0, offset1:$offset1, gds:$gds),
  "$vdst, $addr, $data0, $data1$offset0$offset1$gds"> {

  let has_offset = 0;
  let AsmMatchConverter = "cvtDSOffset01";

  let hasPostISelHook = 1;
}

multiclass DS_1A2D_Off8_RET_mc<string opName,
                               RegisterClass rc = VPR_32,
                               RegisterClass src = rc> {
  def "" : DS_1A2D_Off8_RET<opName, rc, src>;

  let has_m0_read = 0 in {
    def _ppu : DS_1A2D_Off8_RET<opName, rc, src>;
  }
}


class DS_1A_RET<string opName, RegisterClass rc = VPR_32, bit HasTiedOutput = 0, Operand ofs = offset>
: DS_Pseudo<opName,
  (outs rc:$vdst),
  !if(HasTiedOutput,
    (ins VPR_32:$addr, ofs:$offset, gds:$gds, rc:$vdst_in),
    (ins VPR_32:$addr, ofs:$offset, gds:$gds)),
  "$vdst, $addr$offset$gds"> {
  let Constraints = !if(HasTiedOutput, "$vdst = $vdst_in", "");
  let DisableEncoding = !if(HasTiedOutput, "$vdst_in", "");
  let has_data0 = 0;
  let has_data1 = 0;
}

multiclass DS_1A_RET_mc<string opName, RegisterClass rc = VPR_32, bit HasTiedOutput = 0, Operand ofs = offset> {
  def "" : DS_1A_RET<opName, rc, HasTiedOutput, ofs>;

  let has_m0_read = 0 in {
    def _ppu : DS_1A_RET<opName, rc, HasTiedOutput, ofs>;
  }
}

class DS_1A_RET_Tied<string opName, RegisterClass rc = VPR_32> :
  DS_1A_RET<opName, rc, 1>;

class DS_1A_Off8_RET <string opName, RegisterClass rc = VPR_32>
: DS_Pseudo<opName,
  (outs rc:$vdst),
  (ins VPR_32:$addr, offset0:$offset0, offset1:$offset1, gds:$gds),
  "$vdst, $addr$offset0$offset1$gds"> {

  let has_offset = 0;
  let has_data0 = 0;
  let has_data1 = 0;
  let AsmMatchConverter = "cvtDSOffset01";
}

multiclass DS_1A_Off8_RET_mc <string opName, RegisterClass rc = VPR_32> {
  def "" : DS_1A_Off8_RET<opName, rc>;

  let has_m0_read = 0 in {
    def _ppu : DS_1A_Off8_RET<opName, rc>;
  }
}

class DS_1A_RET_GDS <string opName> : DS_Pseudo<opName,
  (outs VPR_32:$vdst),
  (ins VPR_32:$addr, offset:$offset),
  "$vdst, $addr$offset gds"> {

  let has_data0 = 0;
  let has_data1 = 0;
  let has_gds = 0;
  let gdsValue = 1;
  let AsmMatchConverter = "cvtDSGds";
}

class DS_0A_RET <string opName> : DS_Pseudo<opName,
  (outs VPR_32:$vdst),
  (ins offset:$offset, gds:$gds),
  "$vdst$offset$gds"> {

  let mayLoad = 1;
  let mayStore = 1;

  let has_addr = 0;
  let has_data0 = 0;
  let has_data1 = 0;
}

class DS_1A <string opName> : DS_Pseudo<opName,
  (outs),
  (ins VPR_32:$addr, offset:$offset, gds:$gds),
  "$addr$offset$gds"> {

  let mayLoad = 1;
  let mayStore = 1;

  let has_vdst = 0;
  let has_data0 = 0;
  let has_data1 = 0;
}

multiclass DS_1A_mc <string opName> {
  def "" : DS_1A<opName>;

  let has_m0_read = 0 in {
    def _ppu : DS_1A<opName>;
  }
}


class DS_GWS <string opName, dag ins, string asmOps>
: DS_Pseudo<opName, (outs), ins, asmOps> {

  let has_vdst  = 0;
  let has_addr  = 0;
  let has_data0 = 0;
  let has_data1 = 0;

  let has_gds   = 0;
  let gdsValue  = 1;
  let AsmMatchConverter = "cvtDSGds";
}

class DS_GWS_0D <string opName>
: DS_GWS<opName,
  (ins offset:$offset, gds:$gds), "$offset gds"> {
  let hasSideEffects = 1;
}

class DS_GWS_1D <string opName>
: DS_GWS<opName,
  (ins VPR_32:$data0, offset:$offset, gds:$gds), "$data0$offset gds"> {

  let has_gws_data0 = 1;
  let hasSideEffects = 1;
}

class DS_VOID <string opName> : DS_Pseudo<opName,
  (outs), (ins), ""> {
  let mayLoad = 0;
  let mayStore = 0;
  let hasSideEffects = 1;
  let UseNamedOperandTable = 0;
  let AsmMatchConverter = "";

  let has_vdst = 0;
  let has_addr = 0;
  let has_data0 = 0;
  let has_data1 = 0;
  let has_offset = 0;
  let has_offset0 = 0;
  let has_offset1 = 0;
  let has_gds = 0;
}

class DS_1A1D_PERMUTE <string opName, SDPatternOperator node = null_frag>
: DS_Pseudo<opName,
  (outs VPR_32:$vdst),
  (ins VPR_32:$addr, VPR_32:$data0, offset:$offset),
  "$vdst, $addr, $data0$offset",
  [(set i32:$vdst,
   (node (DS1Addr1Offset i32:$addr, i16:$offset), i32:$data0))] > {

  let mayLoad = 0;
  let mayStore = 0;
  let isConvergent = 1;

  let has_data1 = 0;
  let has_gds = 0;
}

