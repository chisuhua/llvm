	.text
	.file	"alu64.ll"
	.globl	addi                    # -- Begin function addi
	.p2align	2
	.type	addi,@function
addi:                                   # @addi
# %bb.0:
	addi	a2, a0, 1
	sltu	a0, a2, a0
	add	a1, a1, a0
	mv	a0, a2
	ret
.Lfunc_end0:
	.size	addi, .Lfunc_end0-addi
                                        # -- End function
	.globl	slti                    # -- Begin function slti
	.p2align	2
	.type	slti,@function
slti:                                   # @slti
# %bb.0:
	beqz	a1, .LBB1_2
# %bb.1:
	slti	a0, a1, 0
	mv	a1, zero
	ret
.LBB1_2:
	sltiu	a0, a0, 2
	mv	a1, zero
	ret
.Lfunc_end1:
	.size	slti, .Lfunc_end1-slti
                                        # -- End function
	.globl	sltiu                   # -- Begin function sltiu
	.p2align	2
	.type	sltiu,@function
sltiu:                                  # @sltiu
# %bb.0:
	beqz	a1, .LBB2_2
# %bb.1:
	mv	a0, zero
	mv	a1, zero
	ret
.LBB2_2:
	sltiu	a0, a0, 3
	mv	a1, zero
	ret
.Lfunc_end2:
	.size	sltiu, .Lfunc_end2-sltiu
                                        # -- End function
	.globl	xori                    # -- Begin function xori
	.p2align	2
	.type	xori,@function
xori:                                   # @xori
# %bb.0:
	xori	a0, a0, 4
	ret
.Lfunc_end3:
	.size	xori, .Lfunc_end3-xori
                                        # -- End function
	.globl	ori                     # -- Begin function ori
	.p2align	2
	.type	ori,@function
ori:                                    # @ori
# %bb.0:
	ori	a0, a0, 5
	ret
.Lfunc_end4:
	.size	ori, .Lfunc_end4-ori
                                        # -- End function
	.globl	andi                    # -- Begin function andi
	.p2align	2
	.type	andi,@function
andi:                                   # @andi
# %bb.0:
	andi	a0, a0, 6
	mv	a1, zero
	ret
.Lfunc_end5:
	.size	andi, .Lfunc_end5-andi
                                        # -- End function
	.globl	slli                    # -- Begin function slli
	.p2align	2
	.type	slli,@function
slli:                                   # @slli
# %bb.0:
	slli	a1, a1, 7
	srli	a2, a0, 25
	or	a1, a1, a2
	slli	a0, a0, 7
	ret
.Lfunc_end6:
	.size	slli, .Lfunc_end6-slli
                                        # -- End function
	.globl	srli                    # -- Begin function srli
	.p2align	2
	.type	srli,@function
srli:                                   # @srli
# %bb.0:
	srli	a0, a0, 8
	slli	a2, a1, 24
	or	a0, a0, a2
	srli	a1, a1, 8
	ret
.Lfunc_end7:
	.size	srli, .Lfunc_end7-srli
                                        # -- End function
	.globl	srai                    # -- Begin function srai
	.p2align	2
	.type	srai,@function
srai:                                   # @srai
# %bb.0:
	srli	a0, a0, 9
	slli	a2, a1, 23
	or	a0, a0, a2
	srai	a1, a1, 9
	ret
.Lfunc_end8:
	.size	srai, .Lfunc_end8-srai
                                        # -- End function
	.globl	add                     # -- Begin function add
	.p2align	2
	.type	add,@function
add:                                    # @add
# %bb.0:
	add	a1, a1, a3
	add	a2, a0, a2
	sltu	a0, a2, a0
	add	a1, a1, a0
	mv	a0, a2
	ret
.Lfunc_end9:
	.size	add, .Lfunc_end9-add
                                        # -- End function
	.globl	sub                     # -- Begin function sub
	.p2align	2
	.type	sub,@function
sub:                                    # @sub
# %bb.0:
	sub	a1, a1, a3
	sltu	a3, a0, a2
	sub	a1, a1, a3
	sub	a0, a0, a2
	ret
.Lfunc_end10:
	.size	sub, .Lfunc_end10-sub
                                        # -- End function
	.globl	sll                     # -- Begin function sll
	.p2align	2
	.type	sll,@function
sll:                                    # @sll
# %bb.0:
	addi	a3, a2, -32
	bltz	a3, .LBB11_2
# %bb.1:
	sll	a1, a0, a3
	mv	a0, zero
	ret
.LBB11_2:
	addi	a3, zero, 31
	sub	a3, a3, a2
	srli	a4, a0, 1
	srl	a3, a4, a3
	sll	a1, a1, a2
	or	a1, a1, a3
	sll	a0, a0, a2
	ret
.Lfunc_end11:
	.size	sll, .Lfunc_end11-sll
                                        # -- End function
	.globl	slt                     # -- Begin function slt
	.p2align	2
	.type	slt,@function
slt:                                    # @slt
# %bb.0:
	beq	a1, a3, .LBB12_2
# %bb.1:
	slt	a0, a1, a3
	mv	a1, zero
	ret
.LBB12_2:
	sltu	a0, a0, a2
	mv	a1, zero
	ret
.Lfunc_end12:
	.size	slt, .Lfunc_end12-slt
                                        # -- End function
	.globl	sltu                    # -- Begin function sltu
	.p2align	2
	.type	sltu,@function
sltu:                                   # @sltu
# %bb.0:
	beq	a1, a3, .LBB13_2
# %bb.1:
	sltu	a0, a1, a3
	mv	a1, zero
	ret
.LBB13_2:
	sltu	a0, a0, a2
	mv	a1, zero
	ret
.Lfunc_end13:
	.size	sltu, .Lfunc_end13-sltu
                                        # -- End function
	.globl	xor                     # -- Begin function xor
	.p2align	2
	.type	xor,@function
xor:                                    # @xor
# %bb.0:
	xor	a0, a0, a2
	xor	a1, a1, a3
	ret
.Lfunc_end14:
	.size	xor, .Lfunc_end14-xor
                                        # -- End function
	.globl	srl                     # -- Begin function srl
	.p2align	2
	.type	srl,@function
srl:                                    # @srl
# %bb.0:
	addi	a3, a2, -32
	bltz	a3, .LBB15_2
# %bb.1:
	srl	a0, a1, a3
	mv	a1, zero
	ret
.LBB15_2:
	addi	a3, zero, 31
	sub	a3, a3, a2
	slli	a4, a1, 1
	sll	a3, a4, a3
	srl	a0, a0, a2
	or	a0, a0, a3
	srl	a1, a1, a2
	ret
.Lfunc_end15:
	.size	srl, .Lfunc_end15-srl
                                        # -- End function
	.globl	sra                     # -- Begin function sra
	.p2align	2
	.type	sra,@function
sra:                                    # @sra
# %bb.0:
	addi	a3, a2, -32
	bltz	a3, .LBB16_2
# %bb.1:
	sra	a0, a1, a3
	srai	a1, a1, 31
	ret
.LBB16_2:
	addi	a3, zero, 31
	sub	a3, a3, a2
	slli	a4, a1, 1
	sll	a3, a4, a3
	srl	a0, a0, a2
	or	a0, a0, a3
	sra	a1, a1, a2
	ret
.Lfunc_end16:
	.size	sra, .Lfunc_end16-sra
                                        # -- End function
	.globl	or                      # -- Begin function or
	.p2align	2
	.type	or,@function
or:                                     # @or
# %bb.0:
	or	a0, a0, a2
	or	a1, a1, a3
	ret
.Lfunc_end17:
	.size	or, .Lfunc_end17-or
                                        # -- End function
	.globl	and                     # -- Begin function and
	.p2align	2
	.type	and,@function
and:                                    # @and
# %bb.0:
	and	a0, a0, a2
	and	a1, a1, a3
	ret
.Lfunc_end18:
	.size	and, .Lfunc_end18-and
                                        # -- End function
	.globl	addiw                   # -- Begin function addiw
	.p2align	2
	.type	addiw,@function
addiw:                                  # @addiw
# %bb.0:
	addi	a0, a0, 123
	ret
.Lfunc_end19:
	.size	addiw, .Lfunc_end19-addiw
                                        # -- End function
	.globl	slliw                   # -- Begin function slliw
	.p2align	2
	.type	slliw,@function
slliw:                                  # @slliw
# %bb.0:
	slli	a0, a0, 17
	ret
.Lfunc_end20:
	.size	slliw, .Lfunc_end20-slliw
                                        # -- End function
	.globl	srliw                   # -- Begin function srliw
	.p2align	2
	.type	srliw,@function
srliw:                                  # @srliw
# %bb.0:
	srli	a0, a0, 8
	ret
.Lfunc_end21:
	.size	srliw, .Lfunc_end21-srliw
                                        # -- End function
	.globl	sraiw                   # -- Begin function sraiw
	.p2align	2
	.type	sraiw,@function
sraiw:                                  # @sraiw
# %bb.0:
	srai	a0, a0, 9
	ret
.Lfunc_end22:
	.size	sraiw, .Lfunc_end22-sraiw
                                        # -- End function
	.globl	sextw                   # -- Begin function sextw
	.p2align	2
	.type	sextw,@function
sextw:                                  # @sextw
# %bb.0:
	ret
.Lfunc_end23:
	.size	sextw, .Lfunc_end23-sextw
                                        # -- End function
	.globl	addw                    # -- Begin function addw
	.p2align	2
	.type	addw,@function
addw:                                   # @addw
# %bb.0:
	add	a0, a0, a1
	ret
.Lfunc_end24:
	.size	addw, .Lfunc_end24-addw
                                        # -- End function
	.globl	subw                    # -- Begin function subw
	.p2align	2
	.type	subw,@function
subw:                                   # @subw
# %bb.0:
	sub	a0, a0, a1
	ret
.Lfunc_end25:
	.size	subw, .Lfunc_end25-subw
                                        # -- End function
	.globl	sllw                    # -- Begin function sllw
	.p2align	2
	.type	sllw,@function
sllw:                                   # @sllw
# %bb.0:
	sll	a0, a0, a1
	ret
.Lfunc_end26:
	.size	sllw, .Lfunc_end26-sllw
                                        # -- End function
	.globl	srlw                    # -- Begin function srlw
	.p2align	2
	.type	srlw,@function
srlw:                                   # @srlw
# %bb.0:
	srl	a0, a0, a1
	ret
.Lfunc_end27:
	.size	srlw, .Lfunc_end27-srlw
                                        # -- End function
	.globl	sraw                    # -- Begin function sraw
	.p2align	2
	.type	sraw,@function
sraw:                                   # @sraw
# %bb.0:
	sra	a0, a0, a2
	ret
.Lfunc_end28:
	.size	sraw, .Lfunc_end28-sraw
                                        # -- End function

	.section	".note.GNU-stack","",@progbits
