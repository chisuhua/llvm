; NOTE: Assertions have been autogenerated by utils/update_llc_test_checks.py
; RUN: llc -mtriple=ppu -mattr=+f,+d -target-abi ilp32d -verify-machineinstrs < %s \
; RUN:   | FileCheck -check-prefix=RV32IFD %s

; These test that we can use both the architectural names (x*) and the ABI names
; (a*, s*, t* etc) to refer to registers in inline asm constraint lists. In each
; case, the named register should be used for the source register of the `addi`.
; It is very likely that `a0` will be chosen as the designation register, but
; this is left to the compiler to choose.
;
; The inline assembly will, by default, contain the ABI names for the registers.
;
; Parenthesised registers in comments are the other aliases for this register.


define i32 @explicit_register_f0(double %a) nounwind {
; RV32IFD-LABEL: explicit_register_f0:
; RV32IFD:       # %bb.0:
; RV32IFD-NEXT:    fmv.d ft0, fa0
; RV32IFD-NEXT:    #APP
; RV32IFD-NEXT:    fcvt.w.d a0, ft0
; RV32IFD-NEXT:    #NO_APP
; RV32IFD-NEXT:    ret
;
  %1 = tail call i32 asm "fcvt.w.d $0, $1", "=r,{f0}"(double %a)
  ret i32 %1
}

define i32 @explicit_register_ft0(double %a) nounwind {
; RV32IFD-LABEL: explicit_register_ft0:
; RV32IFD:       # %bb.0:
; RV32IFD-NEXT:    fmv.d ft0, fa0
; RV32IFD-NEXT:    #APP
; RV32IFD-NEXT:    fcvt.w.d a0, ft0
; RV32IFD-NEXT:    #NO_APP
; RV32IFD-NEXT:    ret
;
  %1 = tail call i32 asm "fcvt.w.d $0, $1", "=r,{ft0}"(double %a)
  ret i32 %1
}

define i32 @explicit_register_f1(double %a) nounwind {
; RV32IFD-LABEL: explicit_register_f1:
; RV32IFD:       # %bb.0:
; RV32IFD-NEXT:    fmv.d ft1, fa0
; RV32IFD-NEXT:    #APP
; RV32IFD-NEXT:    fcvt.w.d a0, ft1
; RV32IFD-NEXT:    #NO_APP
; RV32IFD-NEXT:    ret
;
  %1 = tail call i32 asm "fcvt.w.d $0, $1", "=r,{f1}"(double %a)
  ret i32 %1
}

define i32 @explicit_register_ft1(double %a) nounwind {
; RV32IFD-LABEL: explicit_register_ft1:
; RV32IFD:       # %bb.0:
; RV32IFD-NEXT:    fmv.d ft1, fa0
; RV32IFD-NEXT:    #APP
; RV32IFD-NEXT:    fcvt.w.d a0, ft1
; RV32IFD-NEXT:    #NO_APP
; RV32IFD-NEXT:    ret
;
  %1 = tail call i32 asm "fcvt.w.d $0, $1", "=r,{ft1}"(double %a)
  ret i32 %1
}

define i32 @explicit_register_f2(double %a) nounwind {
; RV32IFD-LABEL: explicit_register_f2:
; RV32IFD:       # %bb.0:
; RV32IFD-NEXT:    fmv.d ft2, fa0
; RV32IFD-NEXT:    #APP
; RV32IFD-NEXT:    fcvt.w.d a0, ft2
; RV32IFD-NEXT:    #NO_APP
; RV32IFD-NEXT:    ret
;
  %1 = tail call i32 asm "fcvt.w.d $0, $1", "=r,{f2}"(double %a)
  ret i32 %1
}

define i32 @explicit_register_ft2(double %a) nounwind {
; RV32IFD-LABEL: explicit_register_ft2:
; RV32IFD:       # %bb.0:
; RV32IFD-NEXT:    fmv.d ft2, fa0
; RV32IFD-NEXT:    #APP
; RV32IFD-NEXT:    fcvt.w.d a0, ft2
; RV32IFD-NEXT:    #NO_APP
; RV32IFD-NEXT:    ret
;
  %1 = tail call i32 asm "fcvt.w.d $0, $1", "=r,{ft2}"(double %a)
  ret i32 %1
}

define i32 @explicit_register_f3(double %a) nounwind {
; RV32IFD-LABEL: explicit_register_f3:
; RV32IFD:       # %bb.0:
; RV32IFD-NEXT:    fmv.d ft3, fa0
; RV32IFD-NEXT:    #APP
; RV32IFD-NEXT:    fcvt.w.d a0, ft3
; RV32IFD-NEXT:    #NO_APP
; RV32IFD-NEXT:    ret
;
  %1 = tail call i32 asm "fcvt.w.d $0, $1", "=r,{f3}"(double %a)
  ret i32 %1
}

define i32 @explicit_register_ft3(double %a) nounwind {
; RV32IFD-LABEL: explicit_register_ft3:
; RV32IFD:       # %bb.0:
; RV32IFD-NEXT:    fmv.d ft3, fa0
; RV32IFD-NEXT:    #APP
; RV32IFD-NEXT:    fcvt.w.d a0, ft3
; RV32IFD-NEXT:    #NO_APP
; RV32IFD-NEXT:    ret
;
  %1 = tail call i32 asm "fcvt.w.d $0, $1", "=r,{ft3}"(double %a)
  ret i32 %1
}

define i32 @explicit_register_f4(double %a) nounwind {
; RV32IFD-LABEL: explicit_register_f4:
; RV32IFD:       # %bb.0:
; RV32IFD-NEXT:    fmv.d ft4, fa0
; RV32IFD-NEXT:    #APP
; RV32IFD-NEXT:    fcvt.w.d a0, ft4
; RV32IFD-NEXT:    #NO_APP
; RV32IFD-NEXT:    ret
;
  %1 = tail call i32 asm "fcvt.w.d $0, $1", "=r,{f4}"(double %a)
  ret i32 %1
}

define i32 @explicit_register_ft4(double %a) nounwind {
; RV32IFD-LABEL: explicit_register_ft4:
; RV32IFD:       # %bb.0:
; RV32IFD-NEXT:    fmv.d ft4, fa0
; RV32IFD-NEXT:    #APP
; RV32IFD-NEXT:    fcvt.w.d a0, ft4
; RV32IFD-NEXT:    #NO_APP
; RV32IFD-NEXT:    ret
;
  %1 = tail call i32 asm "fcvt.w.d $0, $1", "=r,{ft4}"(double %a)
  ret i32 %1
}

define i32 @explicit_register_f5(double %a) nounwind {
; RV32IFD-LABEL: explicit_register_f5:
; RV32IFD:       # %bb.0:
; RV32IFD-NEXT:    fmv.d ft5, fa0
; RV32IFD-NEXT:    #APP
; RV32IFD-NEXT:    fcvt.w.d a0, ft5
; RV32IFD-NEXT:    #NO_APP
; RV32IFD-NEXT:    ret
;
  %1 = tail call i32 asm "fcvt.w.d $0, $1", "=r,{f5}"(double %a)
  ret i32 %1
}

define i32 @explicit_register_ft5(double %a) nounwind {
; RV32IFD-LABEL: explicit_register_ft5:
; RV32IFD:       # %bb.0:
; RV32IFD-NEXT:    fmv.d ft5, fa0
; RV32IFD-NEXT:    #APP
; RV32IFD-NEXT:    fcvt.w.d a0, ft5
; RV32IFD-NEXT:    #NO_APP
; RV32IFD-NEXT:    ret
;
  %1 = tail call i32 asm "fcvt.w.d $0, $1", "=r,{ft5}"(double %a)
  ret i32 %1
}

define i32 @explicit_register_f6(double %a) nounwind {
; RV32IFD-LABEL: explicit_register_f6:
; RV32IFD:       # %bb.0:
; RV32IFD-NEXT:    fmv.d ft6, fa0
; RV32IFD-NEXT:    #APP
; RV32IFD-NEXT:    fcvt.w.d a0, ft6
; RV32IFD-NEXT:    #NO_APP
; RV32IFD-NEXT:    ret
;
  %1 = tail call i32 asm "fcvt.w.d $0, $1", "=r,{f6}"(double %a)
  ret i32 %1
}

define i32 @explicit_register_ft6(double %a) nounwind {
; RV32IFD-LABEL: explicit_register_ft6:
; RV32IFD:       # %bb.0:
; RV32IFD-NEXT:    fmv.d ft6, fa0
; RV32IFD-NEXT:    #APP
; RV32IFD-NEXT:    fcvt.w.d a0, ft6
; RV32IFD-NEXT:    #NO_APP
; RV32IFD-NEXT:    ret
;
  %1 = tail call i32 asm "fcvt.w.d $0, $1", "=r,{ft6}"(double %a)
  ret i32 %1
}

define i32 @explicit_register_f7(double %a) nounwind {
; RV32IFD-LABEL: explicit_register_f7:
; RV32IFD:       # %bb.0:
; RV32IFD-NEXT:    fmv.d ft7, fa0
; RV32IFD-NEXT:    #APP
; RV32IFD-NEXT:    fcvt.w.d a0, ft7
; RV32IFD-NEXT:    #NO_APP
; RV32IFD-NEXT:    ret
;
  %1 = tail call i32 asm "fcvt.w.d $0, $1", "=r,{f7}"(double %a)
  ret i32 %1
}

define i32 @explicit_register_ft7(double %a) nounwind {
; RV32IFD-LABEL: explicit_register_ft7:
; RV32IFD:       # %bb.0:
; RV32IFD-NEXT:    fmv.d ft7, fa0
; RV32IFD-NEXT:    #APP
; RV32IFD-NEXT:    fcvt.w.d a0, ft7
; RV32IFD-NEXT:    #NO_APP
; RV32IFD-NEXT:    ret
;
  %1 = tail call i32 asm "fcvt.w.d $0, $1", "=r,{ft7}"(double %a)
  ret i32 %1
}


; NOTE: This test uses `f8` (`fs0`) as an input, so it should be saved.
define i32 @explicit_register_f8(double %a) nounwind {
; RV32IFD-LABEL: explicit_register_f8:
; RV32IFD:       # %bb.0:
; RV32IFD-NEXT:    addi sp, sp, -16
; RV32IFD-NEXT:    fsd fs0, 8(sp)
; RV32IFD-NEXT:    fmv.d fs0, fa0
; RV32IFD-NEXT:    #APP
; RV32IFD-NEXT:    fcvt.w.d a0, fs0
; RV32IFD-NEXT:    #NO_APP
; RV32IFD-NEXT:    fld fs0, 8(sp)
; RV32IFD-NEXT:    addi sp, sp, 16
; RV32IFD-NEXT:    ret
;
  %1 = tail call i32 asm "fcvt.w.d $0, $1", "=r,{f8}"(double %a)
  ret i32 %1
}

; NOTE: This test uses `fs0` (`f8`) as an input, so it should be saved.
define i32 @explicit_register_fs0(double %a) nounwind {
; RV32IFD-LABEL: explicit_register_fs0:
; RV32IFD:       # %bb.0:
; RV32IFD-NEXT:    addi sp, sp, -16
; RV32IFD-NEXT:    fsd fs0, 8(sp)
; RV32IFD-NEXT:    fmv.d fs0, fa0
; RV32IFD-NEXT:    #APP
; RV32IFD-NEXT:    fcvt.w.d a0, fs0
; RV32IFD-NEXT:    #NO_APP
; RV32IFD-NEXT:    fld fs0, 8(sp)
; RV32IFD-NEXT:    addi sp, sp, 16
; RV32IFD-NEXT:    ret
;
  %1 = tail call i32 asm "fcvt.w.d $0, $1", "=r,{fs0}"(double %a)
  ret i32 %1
}

; NOTE: This test uses `f9` (`fs1`) as an input, so it should be saved.
define i32 @explicit_register_f9(double %a) nounwind {
; RV32IFD-LABEL: explicit_register_f9:
; RV32IFD:       # %bb.0:
; RV32IFD-NEXT:    addi sp, sp, -16
; RV32IFD-NEXT:    fsd fs1, 8(sp)
; RV32IFD-NEXT:    fmv.d fs1, fa0
; RV32IFD-NEXT:    #APP
; RV32IFD-NEXT:    fcvt.w.d a0, fs1
; RV32IFD-NEXT:    #NO_APP
; RV32IFD-NEXT:    fld fs1, 8(sp)
; RV32IFD-NEXT:    addi sp, sp, 16
; RV32IFD-NEXT:    ret
;
  %1 = tail call i32 asm "fcvt.w.d $0, $1", "=r,{f9}"(double %a)
  ret i32 %1
}

; NOTE: This test uses `fs1` (`f9`) as an input, so it should be saved.
define i32 @explicit_register_fs1(double %a) nounwind {
; RV32IFD-LABEL: explicit_register_fs1:
; RV32IFD:       # %bb.0:
; RV32IFD-NEXT:    addi sp, sp, -16
; RV32IFD-NEXT:    fsd fs1, 8(sp)
; RV32IFD-NEXT:    fmv.d fs1, fa0
; RV32IFD-NEXT:    #APP
; RV32IFD-NEXT:    fcvt.w.d a0, fs1
; RV32IFD-NEXT:    #NO_APP
; RV32IFD-NEXT:    fld fs1, 8(sp)
; RV32IFD-NEXT:    addi sp, sp, 16
; RV32IFD-NEXT:    ret
;
  %1 = tail call i32 asm "fcvt.w.d $0, $1", "=r,{fs1}"(double %a)
  ret i32 %1
}

define i32 @explicit_register_f10(double %a) nounwind {
; RV32IFD-LABEL: explicit_register_f10:
; RV32IFD:       # %bb.0:
; RV32IFD-NEXT:    #APP
; RV32IFD-NEXT:    fcvt.w.d a0, fa0
; RV32IFD-NEXT:    #NO_APP
; RV32IFD-NEXT:    ret
;
  %1 = tail call i32 asm "fcvt.w.d $0, $1", "=r,{f10}"(double %a)
  ret i32 %1
}

define i32 @explicit_register_fa0(double %a) nounwind {
; RV32IFD-LABEL: explicit_register_fa0:
; RV32IFD:       # %bb.0:
; RV32IFD-NEXT:    #APP
; RV32IFD-NEXT:    fcvt.w.d a0, fa0
; RV32IFD-NEXT:    #NO_APP
; RV32IFD-NEXT:    ret
;
  %1 = tail call i32 asm "fcvt.w.d $0, $1", "=r,{fa0}"(double %a)
  ret i32 %1
}

define i32 @explicit_register_f11(double %a) nounwind {
; RV32IFD-LABEL: explicit_register_f11:
; RV32IFD:       # %bb.0:
; RV32IFD-NEXT:    fmv.d fa1, fa0
; RV32IFD-NEXT:    #APP
; RV32IFD-NEXT:    fcvt.w.d a0, fa1
; RV32IFD-NEXT:    #NO_APP
; RV32IFD-NEXT:    ret
;
  %1 = tail call i32 asm "fcvt.w.d $0, $1", "=r,{f11}"(double %a)
  ret i32 %1
}

define i32 @explicit_register_fa1(double %a) nounwind {
; RV32IFD-LABEL: explicit_register_fa1:
; RV32IFD:       # %bb.0:
; RV32IFD-NEXT:    fmv.d fa1, fa0
; RV32IFD-NEXT:    #APP
; RV32IFD-NEXT:    fcvt.w.d a0, fa1
; RV32IFD-NEXT:    #NO_APP
; RV32IFD-NEXT:    ret
;
  %1 = tail call i32 asm "fcvt.w.d $0, $1", "=r,{fa1}"(double %a)
  ret i32 %1
}

define i32 @explicit_register_f12(double %a) nounwind {
; RV32IFD-LABEL: explicit_register_f12:
; RV32IFD:       # %bb.0:
; RV32IFD-NEXT:    fmv.d fa2, fa0
; RV32IFD-NEXT:    #APP
; RV32IFD-NEXT:    fcvt.w.d a0, fa2
; RV32IFD-NEXT:    #NO_APP
; RV32IFD-NEXT:    ret
;
  %1 = tail call i32 asm "fcvt.w.d $0, $1", "=r,{f12}"(double %a)
  ret i32 %1
}

define i32 @explicit_register_fa2(double %a) nounwind {
; RV32IFD-LABEL: explicit_register_fa2:
; RV32IFD:       # %bb.0:
; RV32IFD-NEXT:    fmv.d fa2, fa0
; RV32IFD-NEXT:    #APP
; RV32IFD-NEXT:    fcvt.w.d a0, fa2
; RV32IFD-NEXT:    #NO_APP
; RV32IFD-NEXT:    ret
;
  %1 = tail call i32 asm "fcvt.w.d $0, $1", "=r,{fa2}"(double %a)
  ret i32 %1
}

define i32 @explicit_register_f13(double %a) nounwind {
; RV32IFD-LABEL: explicit_register_f13:
; RV32IFD:       # %bb.0:
; RV32IFD-NEXT:    fmv.d fa3, fa0
; RV32IFD-NEXT:    #APP
; RV32IFD-NEXT:    fcvt.w.d a0, fa3
; RV32IFD-NEXT:    #NO_APP
; RV32IFD-NEXT:    ret
;
  %1 = tail call i32 asm "fcvt.w.d $0, $1", "=r,{f13}"(double %a)
  ret i32 %1
}

define i32 @explicit_register_fa3(double %a) nounwind {
; RV32IFD-LABEL: explicit_register_fa3:
; RV32IFD:       # %bb.0:
; RV32IFD-NEXT:    fmv.d fa3, fa0
; RV32IFD-NEXT:    #APP
; RV32IFD-NEXT:    fcvt.w.d a0, fa3
; RV32IFD-NEXT:    #NO_APP
; RV32IFD-NEXT:    ret
;
  %1 = tail call i32 asm "fcvt.w.d $0, $1", "=r,{fa3}"(double %a)
  ret i32 %1
}

define i32 @explicit_register_f14(double %a) nounwind {
; RV32IFD-LABEL: explicit_register_f14:
; RV32IFD:       # %bb.0:
; RV32IFD-NEXT:    fmv.d fa4, fa0
; RV32IFD-NEXT:    #APP
; RV32IFD-NEXT:    fcvt.w.d a0, fa4
; RV32IFD-NEXT:    #NO_APP
; RV32IFD-NEXT:    ret
;
  %1 = tail call i32 asm "fcvt.w.d $0, $1", "=r,{f14}"(double %a)
  ret i32 %1
}

define i32 @explicit_register_fa4(double %a) nounwind {
; RV32IFD-LABEL: explicit_register_fa4:
; RV32IFD:       # %bb.0:
; RV32IFD-NEXT:    fmv.d fa4, fa0
; RV32IFD-NEXT:    #APP
; RV32IFD-NEXT:    fcvt.w.d a0, fa4
; RV32IFD-NEXT:    #NO_APP
; RV32IFD-NEXT:    ret
;
  %1 = tail call i32 asm "fcvt.w.d $0, $1", "=r,{fa4}"(double %a)
  ret i32 %1
}

define i32 @explicit_register_f15(double %a) nounwind {
; RV32IFD-LABEL: explicit_register_f15:
; RV32IFD:       # %bb.0:
; RV32IFD-NEXT:    fmv.d fa5, fa0
; RV32IFD-NEXT:    #APP
; RV32IFD-NEXT:    fcvt.w.d a0, fa5
; RV32IFD-NEXT:    #NO_APP
; RV32IFD-NEXT:    ret
;
  %1 = tail call i32 asm "fcvt.w.d $0, $1", "=r,{f15}"(double %a)
  ret i32 %1
}

define i32 @explicit_register_fa5(double %a) nounwind {
; RV32IFD-LABEL: explicit_register_fa5:
; RV32IFD:       # %bb.0:
; RV32IFD-NEXT:    fmv.d fa5, fa0
; RV32IFD-NEXT:    #APP
; RV32IFD-NEXT:    fcvt.w.d a0, fa5
; RV32IFD-NEXT:    #NO_APP
; RV32IFD-NEXT:    ret
;
  %1 = tail call i32 asm "fcvt.w.d $0, $1", "=r,{fa5}"(double %a)
  ret i32 %1
}

define i32 @explicit_register_f16(double %a) nounwind {
; RV32IFD-LABEL: explicit_register_f16:
; RV32IFD:       # %bb.0:
; RV32IFD-NEXT:    fmv.d fa6, fa0
; RV32IFD-NEXT:    #APP
; RV32IFD-NEXT:    fcvt.w.d a0, fa6
; RV32IFD-NEXT:    #NO_APP
; RV32IFD-NEXT:    ret
;
  %1 = tail call i32 asm "fcvt.w.d $0, $1", "=r,{f16}"(double %a)
  ret i32 %1
}

define i32 @explicit_register_fa6(double %a) nounwind {
; RV32IFD-LABEL: explicit_register_fa6:
; RV32IFD:       # %bb.0:
; RV32IFD-NEXT:    fmv.d fa6, fa0
; RV32IFD-NEXT:    #APP
; RV32IFD-NEXT:    fcvt.w.d a0, fa6
; RV32IFD-NEXT:    #NO_APP
; RV32IFD-NEXT:    ret
;
  %1 = tail call i32 asm "fcvt.w.d $0, $1", "=r,{fa6}"(double %a)
  ret i32 %1
}

define i32 @explicit_register_f17(double %a) nounwind {
; RV32IFD-LABEL: explicit_register_f17:
; RV32IFD:       # %bb.0:
; RV32IFD-NEXT:    fmv.d fa7, fa0
; RV32IFD-NEXT:    #APP
; RV32IFD-NEXT:    fcvt.w.d a0, fa7
; RV32IFD-NEXT:    #NO_APP
; RV32IFD-NEXT:    ret
;
  %1 = tail call i32 asm "fcvt.w.d $0, $1", "=r,{f17}"(double %a)
  ret i32 %1
}

define i32 @explicit_register_fa7(double %a) nounwind {
; RV32IFD-LABEL: explicit_register_fa7:
; RV32IFD:       # %bb.0:
; RV32IFD-NEXT:    fmv.d fa7, fa0
; RV32IFD-NEXT:    #APP
; RV32IFD-NEXT:    fcvt.w.d a0, fa7
; RV32IFD-NEXT:    #NO_APP
; RV32IFD-NEXT:    ret
;
  %1 = tail call i32 asm "fcvt.w.d $0, $1", "=r,{fa7}"(double %a)
  ret i32 %1
}

; NOTE: This test uses `f18` (`fs2`) as an input, so it should be saved.
define i32 @explicit_register_f18(double %a) nounwind {
; RV32IFD-LABEL: explicit_register_f18:
; RV32IFD:       # %bb.0:
; RV32IFD-NEXT:    addi sp, sp, -16
; RV32IFD-NEXT:    fsd fs2, 8(sp)
; RV32IFD-NEXT:    fmv.d fs2, fa0
; RV32IFD-NEXT:    #APP
; RV32IFD-NEXT:    fcvt.w.d a0, fs2
; RV32IFD-NEXT:    #NO_APP
; RV32IFD-NEXT:    fld fs2, 8(sp)
; RV32IFD-NEXT:    addi sp, sp, 16
; RV32IFD-NEXT:    ret
;
  %1 = tail call i32 asm "fcvt.w.d $0, $1", "=r,{f18}"(double %a)
  ret i32 %1
}

; NOTE: This test uses `fs2` (`f18`) as an input, so it should be saved.
define i32 @explicit_register_fs2(double %a) nounwind {
; RV32IFD-LABEL: explicit_register_fs2:
; RV32IFD:       # %bb.0:
; RV32IFD-NEXT:    addi sp, sp, -16
; RV32IFD-NEXT:    fsd fs2, 8(sp)
; RV32IFD-NEXT:    fmv.d fs2, fa0
; RV32IFD-NEXT:    #APP
; RV32IFD-NEXT:    fcvt.w.d a0, fs2
; RV32IFD-NEXT:    #NO_APP
; RV32IFD-NEXT:    fld fs2, 8(sp)
; RV32IFD-NEXT:    addi sp, sp, 16
; RV32IFD-NEXT:    ret
;
  %1 = tail call i32 asm "fcvt.w.d $0, $1", "=r,{fs2}"(double %a)
  ret i32 %1
}

; NOTE: This test uses `f19` (`fs3`) as an input, so it should be saved.
define i32 @explicit_register_f19(double %a) nounwind {
; RV32IFD-LABEL: explicit_register_f19:
; RV32IFD:       # %bb.0:
; RV32IFD-NEXT:    addi sp, sp, -16
; RV32IFD-NEXT:    fsd fs3, 8(sp)
; RV32IFD-NEXT:    fmv.d fs3, fa0
; RV32IFD-NEXT:    #APP
; RV32IFD-NEXT:    fcvt.w.d a0, fs3
; RV32IFD-NEXT:    #NO_APP
; RV32IFD-NEXT:    fld fs3, 8(sp)
; RV32IFD-NEXT:    addi sp, sp, 16
; RV32IFD-NEXT:    ret
;
  %1 = tail call i32 asm "fcvt.w.d $0, $1", "=r,{f19}"(double %a)
  ret i32 %1
}

; NOTE: This test uses `fs3` (`f19`) as an input, so it should be saved.
define i32 @explicit_register_fs3(double %a) nounwind {
; RV32IFD-LABEL: explicit_register_fs3:
; RV32IFD:       # %bb.0:
; RV32IFD-NEXT:    addi sp, sp, -16
; RV32IFD-NEXT:    fsd fs3, 8(sp)
; RV32IFD-NEXT:    fmv.d fs3, fa0
; RV32IFD-NEXT:    #APP
; RV32IFD-NEXT:    fcvt.w.d a0, fs3
; RV32IFD-NEXT:    #NO_APP
; RV32IFD-NEXT:    fld fs3, 8(sp)
; RV32IFD-NEXT:    addi sp, sp, 16
; RV32IFD-NEXT:    ret
;
  %1 = tail call i32 asm "fcvt.w.d $0, $1", "=r,{fs3}"(double %a)
  ret i32 %1
}

; NOTE: This test uses `f20` (`fs4`) as an input, so it should be saved.
define i32 @explicit_register_f20(double %a) nounwind {
; RV32IFD-LABEL: explicit_register_f20:
; RV32IFD:       # %bb.0:
; RV32IFD-NEXT:    addi sp, sp, -16
; RV32IFD-NEXT:    fsd fs4, 8(sp)
; RV32IFD-NEXT:    fmv.d fs4, fa0
; RV32IFD-NEXT:    #APP
; RV32IFD-NEXT:    fcvt.w.d a0, fs4
; RV32IFD-NEXT:    #NO_APP
; RV32IFD-NEXT:    fld fs4, 8(sp)
; RV32IFD-NEXT:    addi sp, sp, 16
; RV32IFD-NEXT:    ret
;
  %1 = tail call i32 asm "fcvt.w.d $0, $1", "=r,{f20}"(double %a)
  ret i32 %1
}

; NOTE: This test uses `fs4` (`f20`) as an input, so it should be saved.
define i32 @explicit_register_fs4(double %a) nounwind {
; RV32IFD-LABEL: explicit_register_fs4:
; RV32IFD:       # %bb.0:
; RV32IFD-NEXT:    addi sp, sp, -16
; RV32IFD-NEXT:    fsd fs4, 8(sp)
; RV32IFD-NEXT:    fmv.d fs4, fa0
; RV32IFD-NEXT:    #APP
; RV32IFD-NEXT:    fcvt.w.d a0, fs4
; RV32IFD-NEXT:    #NO_APP
; RV32IFD-NEXT:    fld fs4, 8(sp)
; RV32IFD-NEXT:    addi sp, sp, 16
; RV32IFD-NEXT:    ret
;
  %1 = tail call i32 asm "fcvt.w.d $0, $1", "=r,{fs4}"(double %a)
  ret i32 %1
}

; NOTE: This test uses `f21` (`fs5`) as an input, so it should be saved.
define i32 @explicit_register_f21(double %a) nounwind {
; RV32IFD-LABEL: explicit_register_f21:
; RV32IFD:       # %bb.0:
; RV32IFD-NEXT:    addi sp, sp, -16
; RV32IFD-NEXT:    fsd fs5, 8(sp)
; RV32IFD-NEXT:    fmv.d fs5, fa0
; RV32IFD-NEXT:    #APP
; RV32IFD-NEXT:    fcvt.w.d a0, fs5
; RV32IFD-NEXT:    #NO_APP
; RV32IFD-NEXT:    fld fs5, 8(sp)
; RV32IFD-NEXT:    addi sp, sp, 16
; RV32IFD-NEXT:    ret
;
  %1 = tail call i32 asm "fcvt.w.d $0, $1", "=r,{f21}"(double %a)
  ret i32 %1
}

; NOTE: This test uses `fs5` (`f21`) as an input, so it should be saved.
define i32 @explicit_register_fs5(double %a) nounwind {
; RV32IFD-LABEL: explicit_register_fs5:
; RV32IFD:       # %bb.0:
; RV32IFD-NEXT:    addi sp, sp, -16
; RV32IFD-NEXT:    fsd fs5, 8(sp)
; RV32IFD-NEXT:    fmv.d fs5, fa0
; RV32IFD-NEXT:    #APP
; RV32IFD-NEXT:    fcvt.w.d a0, fs5
; RV32IFD-NEXT:    #NO_APP
; RV32IFD-NEXT:    fld fs5, 8(sp)
; RV32IFD-NEXT:    addi sp, sp, 16
; RV32IFD-NEXT:    ret
;
  %1 = tail call i32 asm "fcvt.w.d $0, $1", "=r,{fs5}"(double %a)
  ret i32 %1
}

; NOTE: This test uses `f22` (`fs6`) as an input, so it should be saved.
define i32 @explicit_register_f22(double %a) nounwind {
; RV32IFD-LABEL: explicit_register_f22:
; RV32IFD:       # %bb.0:
; RV32IFD-NEXT:    addi sp, sp, -16
; RV32IFD-NEXT:    fsd fs6, 8(sp)
; RV32IFD-NEXT:    fmv.d fs6, fa0
; RV32IFD-NEXT:    #APP
; RV32IFD-NEXT:    fcvt.w.d a0, fs6
; RV32IFD-NEXT:    #NO_APP
; RV32IFD-NEXT:    fld fs6, 8(sp)
; RV32IFD-NEXT:    addi sp, sp, 16
; RV32IFD-NEXT:    ret
;
  %1 = tail call i32 asm "fcvt.w.d $0, $1", "=r,{f22}"(double %a)
  ret i32 %1
}

; NOTE: This test uses `fs6` (`f22`) as an input, so it should be saved.
define i32 @explicit_register_fs6(double %a) nounwind {
; RV32IFD-LABEL: explicit_register_fs6:
; RV32IFD:       # %bb.0:
; RV32IFD-NEXT:    addi sp, sp, -16
; RV32IFD-NEXT:    fsd fs6, 8(sp)
; RV32IFD-NEXT:    fmv.d fs6, fa0
; RV32IFD-NEXT:    #APP
; RV32IFD-NEXT:    fcvt.w.d a0, fs6
; RV32IFD-NEXT:    #NO_APP
; RV32IFD-NEXT:    fld fs6, 8(sp)
; RV32IFD-NEXT:    addi sp, sp, 16
; RV32IFD-NEXT:    ret
;
  %1 = tail call i32 asm "fcvt.w.d $0, $1", "=r,{fs6}"(double %a)
  ret i32 %1
}

; NOTE: This test uses `f23` (`fs7`) as an input, so it should be saved.
define i32 @explicit_register_f23(double %a) nounwind {
; RV32IFD-LABEL: explicit_register_f23:
; RV32IFD:       # %bb.0:
; RV32IFD-NEXT:    addi sp, sp, -16
; RV32IFD-NEXT:    fsd fs7, 8(sp)
; RV32IFD-NEXT:    fmv.d fs7, fa0
; RV32IFD-NEXT:    #APP
; RV32IFD-NEXT:    fcvt.w.d a0, fs7
; RV32IFD-NEXT:    #NO_APP
; RV32IFD-NEXT:    fld fs7, 8(sp)
; RV32IFD-NEXT:    addi sp, sp, 16
; RV32IFD-NEXT:    ret
;
  %1 = tail call i32 asm "fcvt.w.d $0, $1", "=r,{f23}"(double %a)
  ret i32 %1
}

; NOTE: This test uses `fs7` (`f23`) as an input, so it should be saved.
define i32 @explicit_register_fs7(double %a) nounwind {
; RV32IFD-LABEL: explicit_register_fs7:
; RV32IFD:       # %bb.0:
; RV32IFD-NEXT:    addi sp, sp, -16
; RV32IFD-NEXT:    fsd fs7, 8(sp)
; RV32IFD-NEXT:    fmv.d fs7, fa0
; RV32IFD-NEXT:    #APP
; RV32IFD-NEXT:    fcvt.w.d a0, fs7
; RV32IFD-NEXT:    #NO_APP
; RV32IFD-NEXT:    fld fs7, 8(sp)
; RV32IFD-NEXT:    addi sp, sp, 16
; RV32IFD-NEXT:    ret
;
  %1 = tail call i32 asm "fcvt.w.d $0, $1", "=r,{fs7}"(double %a)
  ret i32 %1
}

; NOTE: This test uses `f24` (`fs8`) as an input, so it should be saved.
define i32 @explicit_register_f24(double %a) nounwind {
; RV32IFD-LABEL: explicit_register_f24:
; RV32IFD:       # %bb.0:
; RV32IFD-NEXT:    addi sp, sp, -16
; RV32IFD-NEXT:    fsd fs8, 8(sp)
; RV32IFD-NEXT:    fmv.d fs8, fa0
; RV32IFD-NEXT:    #APP
; RV32IFD-NEXT:    fcvt.w.d a0, fs8
; RV32IFD-NEXT:    #NO_APP
; RV32IFD-NEXT:    fld fs8, 8(sp)
; RV32IFD-NEXT:    addi sp, sp, 16
; RV32IFD-NEXT:    ret
;
  %1 = tail call i32 asm "fcvt.w.d $0, $1", "=r,{f24}"(double %a)
  ret i32 %1
}

; NOTE: This test uses `fs8` (`f24`) as an input, so it should be saved.
define i32 @explicit_register_fs8(double %a) nounwind {
; RV32IFD-LABEL: explicit_register_fs8:
; RV32IFD:       # %bb.0:
; RV32IFD-NEXT:    addi sp, sp, -16
; RV32IFD-NEXT:    fsd fs8, 8(sp)
; RV32IFD-NEXT:    fmv.d fs8, fa0
; RV32IFD-NEXT:    #APP
; RV32IFD-NEXT:    fcvt.w.d a0, fs8
; RV32IFD-NEXT:    #NO_APP
; RV32IFD-NEXT:    fld fs8, 8(sp)
; RV32IFD-NEXT:    addi sp, sp, 16
; RV32IFD-NEXT:    ret
;
  %1 = tail call i32 asm "fcvt.w.d $0, $1", "=r,{fs8}"(double %a)
  ret i32 %1
}

; NOTE: This test uses `f25` (`fs9`) as an input, so it should be saved.
define i32 @explicit_register_f25(double %a) nounwind {
; RV32IFD-LABEL: explicit_register_f25:
; RV32IFD:       # %bb.0:
; RV32IFD-NEXT:    addi sp, sp, -16
; RV32IFD-NEXT:    fsd fs9, 8(sp)
; RV32IFD-NEXT:    fmv.d fs9, fa0
; RV32IFD-NEXT:    #APP
; RV32IFD-NEXT:    fcvt.w.d a0, fs9
; RV32IFD-NEXT:    #NO_APP
; RV32IFD-NEXT:    fld fs9, 8(sp)
; RV32IFD-NEXT:    addi sp, sp, 16
; RV32IFD-NEXT:    ret
;
  %1 = tail call i32 asm "fcvt.w.d $0, $1", "=r,{f25}"(double %a)
  ret i32 %1
}

; NOTE: This test uses `fs9` (`f25`) as an input, so it should be saved.
define i32 @explicit_register_fs9(double %a) nounwind {
; RV32IFD-LABEL: explicit_register_fs9:
; RV32IFD:       # %bb.0:
; RV32IFD-NEXT:    addi sp, sp, -16
; RV32IFD-NEXT:    fsd fs9, 8(sp)
; RV32IFD-NEXT:    fmv.d fs9, fa0
; RV32IFD-NEXT:    #APP
; RV32IFD-NEXT:    fcvt.w.d a0, fs9
; RV32IFD-NEXT:    #NO_APP
; RV32IFD-NEXT:    fld fs9, 8(sp)
; RV32IFD-NEXT:    addi sp, sp, 16
; RV32IFD-NEXT:    ret
;
  %1 = tail call i32 asm "fcvt.w.d $0, $1", "=r,{fs9}"(double %a)
  ret i32 %1
}

; NOTE: This test uses `f26` (`fs10`) as an input, so it should be saved.
define i32 @explicit_register_f26(double %a) nounwind {
; RV32IFD-LABEL: explicit_register_f26:
; RV32IFD:       # %bb.0:
; RV32IFD-NEXT:    addi sp, sp, -16
; RV32IFD-NEXT:    fsd fs10, 8(sp)
; RV32IFD-NEXT:    fmv.d fs10, fa0
; RV32IFD-NEXT:    #APP
; RV32IFD-NEXT:    fcvt.w.d a0, fs10
; RV32IFD-NEXT:    #NO_APP
; RV32IFD-NEXT:    fld fs10, 8(sp)
; RV32IFD-NEXT:    addi sp, sp, 16
; RV32IFD-NEXT:    ret
;
  %1 = tail call i32 asm "fcvt.w.d $0, $1", "=r,{f26}"(double %a)
  ret i32 %1
}

; NOTE: This test uses `fs10` (`f26`) as an input, so it should be saved.
define i32 @explicit_register_fs10(double %a) nounwind {
; RV32IFD-LABEL: explicit_register_fs10:
; RV32IFD:       # %bb.0:
; RV32IFD-NEXT:    addi sp, sp, -16
; RV32IFD-NEXT:    fsd fs10, 8(sp)
; RV32IFD-NEXT:    fmv.d fs10, fa0
; RV32IFD-NEXT:    #APP
; RV32IFD-NEXT:    fcvt.w.d a0, fs10
; RV32IFD-NEXT:    #NO_APP
; RV32IFD-NEXT:    fld fs10, 8(sp)
; RV32IFD-NEXT:    addi sp, sp, 16
; RV32IFD-NEXT:    ret
;
  %1 = tail call i32 asm "fcvt.w.d $0, $1", "=r,{fs10}"(double %a)
  ret i32 %1
}

; NOTE: This test uses `f27` (`fs11`) as an input, so it should be saved.
define i32 @explicit_register_f27(double %a) nounwind {
; RV32IFD-LABEL: explicit_register_f27:
; RV32IFD:       # %bb.0:
; RV32IFD-NEXT:    addi sp, sp, -16
; RV32IFD-NEXT:    fsd fs11, 8(sp)
; RV32IFD-NEXT:    fmv.d fs11, fa0
; RV32IFD-NEXT:    #APP
; RV32IFD-NEXT:    fcvt.w.d a0, fs11
; RV32IFD-NEXT:    #NO_APP
; RV32IFD-NEXT:    fld fs11, 8(sp)
; RV32IFD-NEXT:    addi sp, sp, 16
; RV32IFD-NEXT:    ret
;
  %1 = tail call i32 asm "fcvt.w.d $0, $1", "=r,{f27}"(double %a)
  ret i32 %1
}

; NOTE: This test uses `fs11` (`f27`) as an input, so it should be saved.
define i32 @explicit_register_fs11(double %a) nounwind {
; RV32IFD-LABEL: explicit_register_fs11:
; RV32IFD:       # %bb.0:
; RV32IFD-NEXT:    addi sp, sp, -16
; RV32IFD-NEXT:    fsd fs11, 8(sp)
; RV32IFD-NEXT:    fmv.d fs11, fa0
; RV32IFD-NEXT:    #APP
; RV32IFD-NEXT:    fcvt.w.d a0, fs11
; RV32IFD-NEXT:    #NO_APP
; RV32IFD-NEXT:    fld fs11, 8(sp)
; RV32IFD-NEXT:    addi sp, sp, 16
; RV32IFD-NEXT:    ret
;
  %1 = tail call i32 asm "fcvt.w.d $0, $1", "=r,{fs11}"(double %a)
  ret i32 %1
}

define i32 @explicit_register_f28(double %a) nounwind {
; RV32IFD-LABEL: explicit_register_f28:
; RV32IFD:       # %bb.0:
; RV32IFD-NEXT:    fmv.d ft8, fa0
; RV32IFD-NEXT:    #APP
; RV32IFD-NEXT:    fcvt.w.d a0, ft8
; RV32IFD-NEXT:    #NO_APP
; RV32IFD-NEXT:    ret
;
  %1 = tail call i32 asm "fcvt.w.d $0, $1", "=r,{f28}"(double %a)
  ret i32 %1
}

define i32 @explicit_register_ft8(double %a) nounwind {
; RV32IFD-LABEL: explicit_register_ft8:
; RV32IFD:       # %bb.0:
; RV32IFD-NEXT:    fmv.d ft8, fa0
; RV32IFD-NEXT:    #APP
; RV32IFD-NEXT:    fcvt.w.d a0, ft8
; RV32IFD-NEXT:    #NO_APP
; RV32IFD-NEXT:    ret
;
  %1 = tail call i32 asm "fcvt.w.d $0, $1", "=r,{ft8}"(double %a)
  ret i32 %1
}

define i32 @explicit_register_f29(double %a) nounwind {
; RV32IFD-LABEL: explicit_register_f29:
; RV32IFD:       # %bb.0:
; RV32IFD-NEXT:    fmv.d ft9, fa0
; RV32IFD-NEXT:    #APP
; RV32IFD-NEXT:    fcvt.w.d a0, ft9
; RV32IFD-NEXT:    #NO_APP
; RV32IFD-NEXT:    ret
;
  %1 = tail call i32 asm "fcvt.w.d $0, $1", "=r,{f29}"(double %a)
  ret i32 %1
}

define i32 @explicit_register_ft9(double %a) nounwind {
; RV32IFD-LABEL: explicit_register_ft9:
; RV32IFD:       # %bb.0:
; RV32IFD-NEXT:    fmv.d ft9, fa0
; RV32IFD-NEXT:    #APP
; RV32IFD-NEXT:    fcvt.w.d a0, ft9
; RV32IFD-NEXT:    #NO_APP
; RV32IFD-NEXT:    ret
;
  %1 = tail call i32 asm "fcvt.w.d $0, $1", "=r,{ft9}"(double %a)
  ret i32 %1
}

define i32 @explicit_register_f30(double %a) nounwind {
; RV32IFD-LABEL: explicit_register_f30:
; RV32IFD:       # %bb.0:
; RV32IFD-NEXT:    fmv.d ft10, fa0
; RV32IFD-NEXT:    #APP
; RV32IFD-NEXT:    fcvt.w.d a0, ft10
; RV32IFD-NEXT:    #NO_APP
; RV32IFD-NEXT:    ret
;
  %1 = tail call i32 asm "fcvt.w.d $0, $1", "=r,{f30}"(double %a)
  ret i32 %1
}

define i32 @explicit_register_ft10(double %a) nounwind {
; RV32IFD-LABEL: explicit_register_ft10:
; RV32IFD:       # %bb.0:
; RV32IFD-NEXT:    fmv.d ft10, fa0
; RV32IFD-NEXT:    #APP
; RV32IFD-NEXT:    fcvt.w.d a0, ft10
; RV32IFD-NEXT:    #NO_APP
; RV32IFD-NEXT:    ret
;
  %1 = tail call i32 asm "fcvt.w.d $0, $1", "=r,{ft10}"(double %a)
  ret i32 %1
}

define i32 @explicit_register_f31(double %a) nounwind {
; RV32IFD-LABEL: explicit_register_f31:
; RV32IFD:       # %bb.0:
; RV32IFD-NEXT:    fmv.d ft11, fa0
; RV32IFD-NEXT:    #APP
; RV32IFD-NEXT:    fcvt.w.d a0, ft11
; RV32IFD-NEXT:    #NO_APP
; RV32IFD-NEXT:    ret
;
  %1 = tail call i32 asm "fcvt.w.d $0, $1", "=r,{f31}"(double %a)
  ret i32 %1
}

define i32 @explicit_register_ft11(double %a) nounwind {
; RV32IFD-LABEL: explicit_register_ft11:
; RV32IFD:       # %bb.0:
; RV32IFD-NEXT:    fmv.d ft11, fa0
; RV32IFD-NEXT:    #APP
; RV32IFD-NEXT:    fcvt.w.d a0, ft11
; RV32IFD-NEXT:    #NO_APP
; RV32IFD-NEXT:    ret
;
  %1 = tail call i32 asm "fcvt.w.d $0, $1", "=r,{ft11}"(double %a)
  ret i32 %1
}
