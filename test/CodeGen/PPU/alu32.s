	.text
	.file	"/work/git/chisuhua/projects/sw/mixlang/tools/toolchain/llvm/test/CodeGen/PPU/alu32.ll"
	.globl	addi
	.align	2
	.type	addi,@function
	.section	.opd,"aw",@progbits
addi:                                   # @addi
	.align	3
	.quad	.Lfunc_begin0
	.quad	.TOC.@tocbase
	.quad	0
	.text
.Lfunc_begin0:
# BB#0:
	addi 3, 3, 1
	blr
	.long	0
	.quad	0
.Lfunc_end0:
	.size	addi, .Lfunc_end0-.Lfunc_begin0

	.globl	slti
	.align	2
	.type	slti,@function
	.section	.opd,"aw",@progbits
slti:                                   # @slti
	.align	3
	.quad	.Lfunc_begin1
	.quad	.TOC.@tocbase
	.quad	0
	.text
.Lfunc_begin1:
# BB#0:
	cmpwi 0, 3, 2
	li 3, 1
	bclr 12, 0, 0
# BB#1:
	li 3, 0
	blr
	.long	0
	.quad	0
.Lfunc_end1:
	.size	slti, .Lfunc_end1-.Lfunc_begin1

	.globl	sltiu
	.align	2
	.type	sltiu,@function
	.section	.opd,"aw",@progbits
sltiu:                                  # @sltiu
	.align	3
	.quad	.Lfunc_begin2
	.quad	.TOC.@tocbase
	.quad	0
	.text
.Lfunc_begin2:
# BB#0:
	cmplwi 0, 3, 3
	li 3, 1
	bclr 12, 0, 0
# BB#1:
	li 3, 0
	blr
	.long	0
	.quad	0
.Lfunc_end2:
	.size	sltiu, .Lfunc_end2-.Lfunc_begin2

	.globl	xori
	.align	2
	.type	xori,@function
	.section	.opd,"aw",@progbits
xori:                                   # @xori
	.align	3
	.quad	.Lfunc_begin3
	.quad	.TOC.@tocbase
	.quad	0
	.text
.Lfunc_begin3:
# BB#0:
	xori 3, 3, 4
	blr
	.long	0
	.quad	0
.Lfunc_end3:
	.size	xori, .Lfunc_end3-.Lfunc_begin3

	.globl	ori
	.align	2
	.type	ori,@function
	.section	.opd,"aw",@progbits
ori:                                    # @ori
	.align	3
	.quad	.Lfunc_begin4
	.quad	.TOC.@tocbase
	.quad	0
	.text
.Lfunc_begin4:
# BB#0:
	ori 3, 3, 5
	blr
	.long	0
	.quad	0
.Lfunc_end4:
	.size	ori, .Lfunc_end4-.Lfunc_begin4

	.globl	andi
	.align	2
	.type	andi,@function
	.section	.opd,"aw",@progbits
andi:                                   # @andi
	.align	3
	.quad	.Lfunc_begin5
	.quad	.TOC.@tocbase
	.quad	0
	.text
.Lfunc_begin5:
# BB#0:
	andi. 3, 3, 6
	blr
	.long	0
	.quad	0
.Lfunc_end5:
	.size	andi, .Lfunc_end5-.Lfunc_begin5

	.globl	slli
	.align	2
	.type	slli,@function
	.section	.opd,"aw",@progbits
slli:                                   # @slli
	.align	3
	.quad	.Lfunc_begin6
	.quad	.TOC.@tocbase
	.quad	0
	.text
.Lfunc_begin6:
# BB#0:
	slwi 3, 3, 7
	blr
	.long	0
	.quad	0
.Lfunc_end6:
	.size	slli, .Lfunc_end6-.Lfunc_begin6

	.globl	srli
	.align	2
	.type	srli,@function
	.section	.opd,"aw",@progbits
srli:                                   # @srli
	.align	3
	.quad	.Lfunc_begin7
	.quad	.TOC.@tocbase
	.quad	0
	.text
.Lfunc_begin7:
# BB#0:
	srwi 3, 3, 8
	blr
	.long	0
	.quad	0
.Lfunc_end7:
	.size	srli, .Lfunc_end7-.Lfunc_begin7

	.globl	srai
	.align	2
	.type	srai,@function
	.section	.opd,"aw",@progbits
srai:                                   # @srai
	.align	3
	.quad	.Lfunc_begin8
	.quad	.TOC.@tocbase
	.quad	0
	.text
.Lfunc_begin8:
# BB#0:
	srawi 3, 3, 9
	blr
	.long	0
	.quad	0
.Lfunc_end8:
	.size	srai, .Lfunc_end8-.Lfunc_begin8

	.globl	add
	.align	2
	.type	add,@function
	.section	.opd,"aw",@progbits
add:                                    # @add
	.align	3
	.quad	.Lfunc_begin9
	.quad	.TOC.@tocbase
	.quad	0
	.text
.Lfunc_begin9:
# BB#0:
	add 3, 3, 4
	blr
	.long	0
	.quad	0
.Lfunc_end9:
	.size	add, .Lfunc_end9-.Lfunc_begin9

	.globl	sub
	.align	2
	.type	sub,@function
	.section	.opd,"aw",@progbits
sub:                                    # @sub
	.align	3
	.quad	.Lfunc_begin10
	.quad	.TOC.@tocbase
	.quad	0
	.text
.Lfunc_begin10:
# BB#0:
	subf 3, 4, 3
	blr
	.long	0
	.quad	0
.Lfunc_end10:
	.size	sub, .Lfunc_end10-.Lfunc_begin10

	.globl	sll
	.align	2
	.type	sll,@function
	.section	.opd,"aw",@progbits
sll:                                    # @sll
	.align	3
	.quad	.Lfunc_begin11
	.quad	.TOC.@tocbase
	.quad	0
	.text
.Lfunc_begin11:
# BB#0:
	slw 3, 3, 4
	blr
	.long	0
	.quad	0
.Lfunc_end11:
	.size	sll, .Lfunc_end11-.Lfunc_begin11

	.globl	slt
	.align	2
	.type	slt,@function
	.section	.opd,"aw",@progbits
slt:                                    # @slt
	.align	3
	.quad	.Lfunc_begin12
	.quad	.TOC.@tocbase
	.quad	0
	.text
.Lfunc_begin12:
# BB#0:
	cmpw 0, 3, 4
	li 3, 1
	bclr 12, 0, 0
# BB#1:
	li 3, 0
	blr
	.long	0
	.quad	0
.Lfunc_end12:
	.size	slt, .Lfunc_end12-.Lfunc_begin12

	.globl	sltu
	.align	2
	.type	sltu,@function
	.section	.opd,"aw",@progbits
sltu:                                   # @sltu
	.align	3
	.quad	.Lfunc_begin13
	.quad	.TOC.@tocbase
	.quad	0
	.text
.Lfunc_begin13:
# BB#0:
	cmplw 0, 3, 4
	li 3, 1
	bclr 12, 0, 0
# BB#1:
	li 3, 0
	blr
	.long	0
	.quad	0
.Lfunc_end13:
	.size	sltu, .Lfunc_end13-.Lfunc_begin13

	.globl	xor
	.align	2
	.type	xor,@function
	.section	.opd,"aw",@progbits
xor:                                    # @xor
	.align	3
	.quad	.Lfunc_begin14
	.quad	.TOC.@tocbase
	.quad	0
	.text
.Lfunc_begin14:
# BB#0:
	xor 3, 3, 4
	blr
	.long	0
	.quad	0
.Lfunc_end14:
	.size	xor, .Lfunc_end14-.Lfunc_begin14

	.globl	srl
	.align	2
	.type	srl,@function
	.section	.opd,"aw",@progbits
srl:                                    # @srl
	.align	3
	.quad	.Lfunc_begin15
	.quad	.TOC.@tocbase
	.quad	0
	.text
.Lfunc_begin15:
# BB#0:
	srw 3, 3, 4
	blr
	.long	0
	.quad	0
.Lfunc_end15:
	.size	srl, .Lfunc_end15-.Lfunc_begin15

	.globl	sra
	.align	2
	.type	sra,@function
	.section	.opd,"aw",@progbits
sra:                                    # @sra
	.align	3
	.quad	.Lfunc_begin16
	.quad	.TOC.@tocbase
	.quad	0
	.text
.Lfunc_begin16:
# BB#0:
	sraw 3, 3, 4
	blr
	.long	0
	.quad	0
.Lfunc_end16:
	.size	sra, .Lfunc_end16-.Lfunc_begin16

	.globl	or
	.align	2
	.type	or,@function
	.section	.opd,"aw",@progbits
or:                                     # @or
	.align	3
	.quad	.Lfunc_begin17
	.quad	.TOC.@tocbase
	.quad	0
	.text
.Lfunc_begin17:
# BB#0:
	or 3, 3, 4
	blr
	.long	0
	.quad	0
.Lfunc_end17:
	.size	or, .Lfunc_end17-.Lfunc_begin17

	.globl	and
	.align	2
	.type	and,@function
	.section	.opd,"aw",@progbits
and:                                    # @and
	.align	3
	.quad	.Lfunc_begin18
	.quad	.TOC.@tocbase
	.quad	0
	.text
.Lfunc_begin18:
# BB#0:
	and 3, 3, 4
	blr
	.long	0
	.quad	0
.Lfunc_end18:
	.size	and, .Lfunc_end18-.Lfunc_begin18


	.section	".note.GNU-stack","",@progbits
