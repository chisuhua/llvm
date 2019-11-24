//===----------------------------------------------------------------------===//
// VOP3P Classes
//===----------------------------------------------------------------------===//

class VOP3PInst<string OpName, VOPProfile P, SDPatternOperator node = null_frag> :
  VOP3P_Pseudo<OpName, P,
    !if(P.HasModifiers, getVOP3PModPat<P, node>.ret, getVOP3Pat<P, node>.ret)
>;

// Non-packed instructions that use the VOP3P encoding.
// VOP3 neg/abs and VOP3P opsel/opsel_hi modifiers are allowed.
class VOP3_VOP3PInst<string OpName, VOPProfile P, bit UseTiedOutput = 0,
                     SDPatternOperator node = null_frag> :
  VOP3P_Pseudo<OpName, P> {
  // These operands are only sort of f16 operands. Depending on
  // op_sel_hi, these may be interpreted as f32. The inline immediate
  // values are really f16 converted to f32, so we treat these as f16
  // operands.
  let InOperandList =
    !con(
      !con(
        (ins FP16InputMods:$src0_modifiers, VCSrc_f16:$src0,
             FP16InputMods:$src1_modifiers, VCSrc_f16:$src1,
             FP16InputMods:$src2_modifiers, VCSrc_f16:$src2,
             clampmod:$clamp),
         !if(UseTiedOutput, (ins VPR_32:$vdst_in), (ins))),
         (ins op_sel:$op_sel, op_sel_hi:$op_sel_hi));

  let Constraints = !if(UseTiedOutput, "$vdst = $vdst_in", "");
  let DisableEncoding = !if(UseTiedOutput, "$vdst_in", "");
  let AsmOperands =
    " $vdst, $src0_modifiers, $src1_modifiers, $src2_modifiers$op_sel$op_sel_hi$clamp";
}

