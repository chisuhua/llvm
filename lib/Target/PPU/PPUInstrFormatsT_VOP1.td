class VOP1e <bits<8> op, VOPProfile P> : Enc32 {
  bits<8> vdst;
  bits<9> src0;

  let Inst{8-0}   = !if(P.HasSrc0, src0{8-0}, ?);
  let Inst{16-9}  = op;
  let Inst{24-17} = !if(P.EmitDst, vdst{7-0}, 0);
  let Inst{31-25} = 0x3f; //encoding
}

class VOP1_Pseudo <string opName, VOPProfile P, list<dag> pattern=[], bit VOP1Only = 0> :
  VOP_Pseudo <opName, !if(VOP1Only, "", "_e32"), P, P.Outs32, P.Ins32, P.Asm32, pattern> {

  let AsmOperands = P.Asm32;

  let Size = 4;
  let mayLoad = 0;
  let mayStore = 0;
  let hasSideEffects = 0;

  let VOP1 = 1;
  let VALU = 1;
  let Uses = [TMSK];

  let AsmVariantName = PPUAsmVariants.Default;
}

class VOP1_Real <VOP1_Pseudo ps, int EncodingFamily> :
  PPTInst <ps.OutOperandList, ps.InOperandList, ps.Mnemonic , ps.AsmOperands, []>,
  PPUMCInstr <ps.PseudoInstr, EncodingFamily> {

  let isPseudo = 0;
  let isCodeGenOnly = 0;

  let Constraints     = ps.Constraints;
  let DisableEncoding = ps.DisableEncoding;

  // copy relevant pseudo op flags
  let SubtargetPredicate = ps.SubtargetPredicate;
  let AsmMatchConverter  = ps.AsmMatchConverter;
  let AsmVariantName     = ps.AsmVariantName;
  let Constraints        = ps.Constraints;
  let DisableEncoding    = ps.DisableEncoding;
  let TSFlags            = ps.TSFlags;
  let UseNamedOperandTable = ps.UseNamedOperandTable;
  let Uses                 = ps.Uses;
  let Defs                 = ps.Defs;

  // FIXME schi why need this
  // let mayLoad = 0;
  // let mayStore = 0;
  // let hasSideEffects = 0;
}

class getVOP1Pat64 <SDPatternOperator node, VOPProfile P> : LetDummies {
  list<dag> ret =
    !if(P.HasModifiers,
        [(set P.DstVT:$vdst, (node (P.Src0VT (VOP3Mods0 P.Src0VT:$src0,
                                              i32:$src0_modifiers,
                                              i1:$clamp, i32:$omod))))],
        !if(P.HasOMod,
            [(set P.DstVT:$vdst, (node (P.Src0VT (VOP3OMods P.Src0VT:$src0,
                                                  i1:$clamp, i32:$omod))))],
            [(set P.DstVT:$vdst, (node P.Src0VT:$src0))]
        )
    );
}

multiclass VOP1Inst <string opName, VOPProfile P,
                     SDPatternOperator node = null_frag> {
  def _e32 : VOP1_Pseudo <opName, P>;
  def _e64 : VOP3_Pseudo <opName, P, getVOP1Pat64<node, P>.ret>;
}

// Special profile for instructions which have clamp
// and output modifiers (but have no input modifiers)
class VOPProfileI2F<ValueType dstVt, ValueType srcVt> :
  VOPProfile<[dstVt, srcVt, untyped, untyped]> {

  let Ins64 = (ins Src0RC64:$src0, clampmod:$clamp, omod:$omod);
  let Asm64 = "$vdst, $src0$clamp$omod";

  let HasModifiers = 0;
  let HasClamp = 1;
  let HasOMod = 1;
}

def VOP1_F64_I32 : VOPProfileI2F <f64, i32>;
def VOP1_F32_I32 : VOPProfileI2F <f32, i32>;
def VOP1_F16_I16 : VOPProfileI2F <f16, i16>;



multiclass VOP1Only_Real_ppu <bits<10> op> {
  let DecoderNamespace = "PPU" in {
    def _ppu :
      VOP1_Real<!cast<VOP1_Pseudo>(NAME), PPUEncodingFamily.PPU>,
      VOP1e<op{7-0}, !cast<VOP1_Pseudo>(NAME).Pfl>;
  }
}

multiclass VOP1_Real_e32e64_ppu <bits<10> op> {
  let DecoderNamespace = "PPU" in {
    def _e32_ppu :
      VOP1_Real<!cast<VOP1_Pseudo>(NAME#"_e32"), PPUEncodingFamily.PPU>,
      VOP1e<op{7-0}, !cast<VOP1_Pseudo>(NAME#"_e32").Pfl>;
    def _e64_ppu :
      VOP3_Real<!cast<VOP3_Pseudo>(NAME#"_e64"), PPUEncodingFamily.PPU>,
      VOP3e_ppu <!add(0x140, op), !cast<VOP3_Pseudo>(NAME#"_e64").Pfl>;
  }
}

multiclass VOP1_Real_ppu <bits<10> op> {
  defm NAME : VOP1_Real_e32e64_ppu <op>;
}

