

// FIXME: Specify SchedRW for READFIRSTLANE_B32
// TODO: Make profile for this, there is VOP3 encoding also
def V_READFIRSTLANE_B32 :
  PPUInst <(outs SReg_32:$vdst),
    (ins VRegOrLds_32:$src0),
    "v_readfirstlane_b32", "$vdst, $src0",
    [(set i32:$vdst, (int_ppu_readfirstlane i32:$src0))]>,
  Enc32 {

  let isCodeGenOnly = 0;
  let UseNamedOperandTable = 1;

  let Size = 4;
  let mayLoad = 0;
  let mayStore = 0;
  let hasSideEffects = 0;

  let VOP1 = 1;
  let VALU = 1;
  let Uses = [TMSK];
  let isConvergent = 1;

  bits<8> vdst;
  bits<9> src0;

  let Inst{8-0}   = src0;
  let Inst{16-9}  = 0x2;
  let Inst{24-17} = vdst;
  let Inst{31-25} = 0x3f; //encoding
}

