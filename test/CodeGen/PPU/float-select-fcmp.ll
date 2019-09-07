; NOTE: Assertions have been autogenerated by utils/update_llc_test_checks.py
; RUN: llc -mtriple=ppu -mattr=+f -verify-machineinstrs < %s \
; RUN:   | FileCheck -check-prefix=RV32IF %s

define float @select_fcmp_false(float %a, float %b) nounwind {
; RV32IF-LABEL: select_fcmp_false:
; RV32IF:       # %bb.0:
; RV32IF-NEXT:    mv a0, a1
; RV32IF-NEXT:    ret
;
  %1 = fcmp false float %a, %b
  %2 = select i1 %1, float %a, float %b
  ret float %2
}

define float @select_fcmp_oeq(float %a, float %b) nounwind {
; RV32IF-LABEL: select_fcmp_oeq:
; RV32IF:       # %bb.0:
; RV32IF-NEXT:    fmv.w.x ft1, a1
; RV32IF-NEXT:    fmv.w.x ft0, a0
; RV32IF-NEXT:    feq.s a0, ft0, ft1
; RV32IF-NEXT:    bnez a0, .LBB1_2
; RV32IF-NEXT:  # %bb.1:
; RV32IF-NEXT:    fmv.s ft0, ft1
; RV32IF-NEXT:  .LBB1_2:
; RV32IF-NEXT:    fmv.x.w a0, ft0
; RV32IF-NEXT:    ret
;
  %1 = fcmp oeq float %a, %b
  %2 = select i1 %1, float %a, float %b
  ret float %2
}

define float @select_fcmp_ogt(float %a, float %b) nounwind {
; RV32IF-LABEL: select_fcmp_ogt:
; RV32IF:       # %bb.0:
; RV32IF-NEXT:    fmv.w.x ft0, a0
; RV32IF-NEXT:    fmv.w.x ft1, a1
; RV32IF-NEXT:    flt.s a0, ft1, ft0
; RV32IF-NEXT:    bnez a0, .LBB2_2
; RV32IF-NEXT:  # %bb.1:
; RV32IF-NEXT:    fmv.s ft0, ft1
; RV32IF-NEXT:  .LBB2_2:
; RV32IF-NEXT:    fmv.x.w a0, ft0
; RV32IF-NEXT:    ret
;
  %1 = fcmp ogt float %a, %b
  %2 = select i1 %1, float %a, float %b
  ret float %2
}

define float @select_fcmp_oge(float %a, float %b) nounwind {
; RV32IF-LABEL: select_fcmp_oge:
; RV32IF:       # %bb.0:
; RV32IF-NEXT:    fmv.w.x ft0, a0
; RV32IF-NEXT:    fmv.w.x ft1, a1
; RV32IF-NEXT:    fle.s a0, ft1, ft0
; RV32IF-NEXT:    bnez a0, .LBB3_2
; RV32IF-NEXT:  # %bb.1:
; RV32IF-NEXT:    fmv.s ft0, ft1
; RV32IF-NEXT:  .LBB3_2:
; RV32IF-NEXT:    fmv.x.w a0, ft0
; RV32IF-NEXT:    ret
;
  %1 = fcmp oge float %a, %b
  %2 = select i1 %1, float %a, float %b
  ret float %2
}

define float @select_fcmp_olt(float %a, float %b) nounwind {
; RV32IF-LABEL: select_fcmp_olt:
; RV32IF:       # %bb.0:
; RV32IF-NEXT:    fmv.w.x ft1, a1
; RV32IF-NEXT:    fmv.w.x ft0, a0
; RV32IF-NEXT:    flt.s a0, ft0, ft1
; RV32IF-NEXT:    bnez a0, .LBB4_2
; RV32IF-NEXT:  # %bb.1:
; RV32IF-NEXT:    fmv.s ft0, ft1
; RV32IF-NEXT:  .LBB4_2:
; RV32IF-NEXT:    fmv.x.w a0, ft0
; RV32IF-NEXT:    ret
;
  %1 = fcmp olt float %a, %b
  %2 = select i1 %1, float %a, float %b
  ret float %2
}

define float @select_fcmp_ole(float %a, float %b) nounwind {
; RV32IF-LABEL: select_fcmp_ole:
; RV32IF:       # %bb.0:
; RV32IF-NEXT:    fmv.w.x ft1, a1
; RV32IF-NEXT:    fmv.w.x ft0, a0
; RV32IF-NEXT:    fle.s a0, ft0, ft1
; RV32IF-NEXT:    bnez a0, .LBB5_2
; RV32IF-NEXT:  # %bb.1:
; RV32IF-NEXT:    fmv.s ft0, ft1
; RV32IF-NEXT:  .LBB5_2:
; RV32IF-NEXT:    fmv.x.w a0, ft0
; RV32IF-NEXT:    ret
;
  %1 = fcmp ole float %a, %b
  %2 = select i1 %1, float %a, float %b
  ret float %2
}

define float @select_fcmp_one(float %a, float %b) nounwind {
; TODO: feq.s+sltiu+bne sequence could be optimised
; RV32IF-LABEL: select_fcmp_one:
; RV32IF:       # %bb.0:
; RV32IF-NEXT:    fmv.w.x ft0, a0
; RV32IF-NEXT:    fmv.w.x ft1, a1
; RV32IF-NEXT:    feq.s a0, ft1, ft1
; RV32IF-NEXT:    feq.s a1, ft0, ft0
; RV32IF-NEXT:    and a0, a1, a0
; RV32IF-NEXT:    feq.s a1, ft0, ft1
; RV32IF-NEXT:    not a1, a1
; RV32IF-NEXT:    and a0, a1, a0
; RV32IF-NEXT:    bnez a0, .LBB6_2
; RV32IF-NEXT:  # %bb.1:
; RV32IF-NEXT:    fmv.s ft0, ft1
; RV32IF-NEXT:  .LBB6_2:
; RV32IF-NEXT:    fmv.x.w a0, ft0
; RV32IF-NEXT:    ret
;
  %1 = fcmp one float %a, %b
  %2 = select i1 %1, float %a, float %b
  ret float %2
}

define float @select_fcmp_ord(float %a, float %b) nounwind {
; RV32IF-LABEL: select_fcmp_ord:
; RV32IF:       # %bb.0:
; RV32IF-NEXT:    fmv.w.x ft0, a0
; RV32IF-NEXT:    fmv.w.x ft1, a1
; RV32IF-NEXT:    feq.s a0, ft1, ft1
; RV32IF-NEXT:    feq.s a1, ft0, ft0
; RV32IF-NEXT:    and a0, a1, a0
; RV32IF-NEXT:    bnez a0, .LBB7_2
; RV32IF-NEXT:  # %bb.1:
; RV32IF-NEXT:    fmv.s ft0, ft1
; RV32IF-NEXT:  .LBB7_2:
; RV32IF-NEXT:    fmv.x.w a0, ft0
; RV32IF-NEXT:    ret
;
  %1 = fcmp ord float %a, %b
  %2 = select i1 %1, float %a, float %b
  ret float %2
}

define float @select_fcmp_ueq(float %a, float %b) nounwind {
; RV32IF-LABEL: select_fcmp_ueq:
; RV32IF:       # %bb.0:
; RV32IF-NEXT:    fmv.w.x ft0, a0
; RV32IF-NEXT:    fmv.w.x ft1, a1
; RV32IF-NEXT:    feq.s a0, ft1, ft1
; RV32IF-NEXT:    feq.s a1, ft0, ft0
; RV32IF-NEXT:    and a0, a1, a0
; RV32IF-NEXT:    seqz a0, a0
; RV32IF-NEXT:    feq.s a1, ft0, ft1
; RV32IF-NEXT:    or a0, a1, a0
; RV32IF-NEXT:    bnez a0, .LBB8_2
; RV32IF-NEXT:  # %bb.1:
; RV32IF-NEXT:    fmv.s ft0, ft1
; RV32IF-NEXT:  .LBB8_2:
; RV32IF-NEXT:    fmv.x.w a0, ft0
; RV32IF-NEXT:    ret
;
  %1 = fcmp ueq float %a, %b
  %2 = select i1 %1, float %a, float %b
  ret float %2
}

define float @select_fcmp_ugt(float %a, float %b) nounwind {
; RV32IF-LABEL: select_fcmp_ugt:
; RV32IF:       # %bb.0:
; RV32IF-NEXT:    fmv.w.x ft1, a1
; RV32IF-NEXT:    fmv.w.x ft0, a0
; RV32IF-NEXT:    fle.s a0, ft0, ft1
; RV32IF-NEXT:    xori a0, a0, 1
; RV32IF-NEXT:    bnez a0, .LBB9_2
; RV32IF-NEXT:  # %bb.1:
; RV32IF-NEXT:    fmv.s ft0, ft1
; RV32IF-NEXT:  .LBB9_2:
; RV32IF-NEXT:    fmv.x.w a0, ft0
; RV32IF-NEXT:    ret
;
  %1 = fcmp ugt float %a, %b
  %2 = select i1 %1, float %a, float %b
  ret float %2
}

define float @select_fcmp_uge(float %a, float %b) nounwind {
; RV32IF-LABEL: select_fcmp_uge:
; RV32IF:       # %bb.0:
; RV32IF-NEXT:    fmv.w.x ft1, a1
; RV32IF-NEXT:    fmv.w.x ft0, a0
; RV32IF-NEXT:    flt.s a0, ft0, ft1
; RV32IF-NEXT:    xori a0, a0, 1
; RV32IF-NEXT:    bnez a0, .LBB10_2
; RV32IF-NEXT:  # %bb.1:
; RV32IF-NEXT:    fmv.s ft0, ft1
; RV32IF-NEXT:  .LBB10_2:
; RV32IF-NEXT:    fmv.x.w a0, ft0
; RV32IF-NEXT:    ret
;
  %1 = fcmp uge float %a, %b
  %2 = select i1 %1, float %a, float %b
  ret float %2
}

define float @select_fcmp_ult(float %a, float %b) nounwind {
; RV32IF-LABEL: select_fcmp_ult:
; RV32IF:       # %bb.0:
; RV32IF-NEXT:    fmv.w.x ft0, a0
; RV32IF-NEXT:    fmv.w.x ft1, a1
; RV32IF-NEXT:    fle.s a0, ft1, ft0
; RV32IF-NEXT:    xori a0, a0, 1
; RV32IF-NEXT:    bnez a0, .LBB11_2
; RV32IF-NEXT:  # %bb.1:
; RV32IF-NEXT:    fmv.s ft0, ft1
; RV32IF-NEXT:  .LBB11_2:
; RV32IF-NEXT:    fmv.x.w a0, ft0
; RV32IF-NEXT:    ret
;
  %1 = fcmp ult float %a, %b
  %2 = select i1 %1, float %a, float %b
  ret float %2
}

define float @select_fcmp_ule(float %a, float %b) nounwind {
; RV32IF-LABEL: select_fcmp_ule:
; RV32IF:       # %bb.0:
; RV32IF-NEXT:    fmv.w.x ft0, a0
; RV32IF-NEXT:    fmv.w.x ft1, a1
; RV32IF-NEXT:    flt.s a0, ft1, ft0
; RV32IF-NEXT:    xori a0, a0, 1
; RV32IF-NEXT:    bnez a0, .LBB12_2
; RV32IF-NEXT:  # %bb.1:
; RV32IF-NEXT:    fmv.s ft0, ft1
; RV32IF-NEXT:  .LBB12_2:
; RV32IF-NEXT:    fmv.x.w a0, ft0
; RV32IF-NEXT:    ret
;
  %1 = fcmp ule float %a, %b
  %2 = select i1 %1, float %a, float %b
  ret float %2
}

define float @select_fcmp_une(float %a, float %b) nounwind {
; RV32IF-LABEL: select_fcmp_une:
; RV32IF:       # %bb.0:
; RV32IF-NEXT:    fmv.w.x ft1, a1
; RV32IF-NEXT:    fmv.w.x ft0, a0
; RV32IF-NEXT:    feq.s a0, ft0, ft1
; RV32IF-NEXT:    xori a0, a0, 1
; RV32IF-NEXT:    bnez a0, .LBB13_2
; RV32IF-NEXT:  # %bb.1:
; RV32IF-NEXT:    fmv.s ft0, ft1
; RV32IF-NEXT:  .LBB13_2:
; RV32IF-NEXT:    fmv.x.w a0, ft0
; RV32IF-NEXT:    ret
;
  %1 = fcmp une float %a, %b
  %2 = select i1 %1, float %a, float %b
  ret float %2
}

define float @select_fcmp_uno(float %a, float %b) nounwind {
; TODO: sltiu+bne could be optimized
; RV32IF-LABEL: select_fcmp_uno:
; RV32IF:       # %bb.0:
; RV32IF-NEXT:    fmv.w.x ft0, a0
; RV32IF-NEXT:    fmv.w.x ft1, a1
; RV32IF-NEXT:    feq.s a0, ft1, ft1
; RV32IF-NEXT:    feq.s a1, ft0, ft0
; RV32IF-NEXT:    and a0, a1, a0
; RV32IF-NEXT:    seqz a0, a0
; RV32IF-NEXT:    bnez a0, .LBB14_2
; RV32IF-NEXT:  # %bb.1:
; RV32IF-NEXT:    fmv.s ft0, ft1
; RV32IF-NEXT:  .LBB14_2:
; RV32IF-NEXT:    fmv.x.w a0, ft0
; RV32IF-NEXT:    ret
;
  %1 = fcmp uno float %a, %b
  %2 = select i1 %1, float %a, float %b
  ret float %2
}

define float @select_fcmp_true(float %a, float %b) nounwind {
; RV32IF-LABEL: select_fcmp_true:
; RV32IF:       # %bb.0:
; RV32IF-NEXT:    ret
;
  %1 = fcmp true float %a, %b
  %2 = select i1 %1, float %a, float %b
  ret float %2
}

; Ensure that ISel succeeds for a select+fcmp that has an i32 result type.
define i32 @i32_select_fcmp_oeq(float %a, float %b, i32 %c, i32 %d) nounwind {
; RV32IF-LABEL: i32_select_fcmp_oeq:
; RV32IF:       # %bb.0:
; RV32IF-NEXT:    fmv.w.x ft0, a1
; RV32IF-NEXT:    fmv.w.x ft1, a0
; RV32IF-NEXT:    feq.s a0, ft1, ft0
; RV32IF-NEXT:    bnez a0, .LBB16_2
; RV32IF-NEXT:  # %bb.1:
; RV32IF-NEXT:    mv a2, a3
; RV32IF-NEXT:  .LBB16_2:
; RV32IF-NEXT:    mv a0, a2
; RV32IF-NEXT:    ret
;
  %1 = fcmp oeq float %a, %b
  %2 = select i1 %1, i32 %c, i32 %d
  ret i32 %2
}
