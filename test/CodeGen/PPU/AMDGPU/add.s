	.text
	.section	.AMDGPU.config
	.long	47176
	.long	11272192
	.long	47180
	.long	132
	.long	47200
	.long	0
	.long	4
	.long	0
	.long	8
	.long	0
	.text
	.globl	s_add_i32               ; -- Begin function s_add_i32
	.p2align	8
	.type	s_add_i32,@function
s_add_i32:                              ; @s_add_i32
; %bb.0:
	s_load_dwordx4 s[4:7], s[0:1], 0x24
	s_mov_b32 s3, 0xf000
	s_mov_b32 s2, -1
	s_waitcnt lgkmcnt(0)
	s_mov_b32 s0, s4
	s_mov_b32 s1, s5
	s_load_dwordx2 s[4:5], s[6:7], 0x0
	s_waitcnt lgkmcnt(0)
	s_add_i32 s4, s4, s5
	v_mov_b32_e32 v0, s4
	buffer_store_dword v0, off, s[0:3], 0
	s_endpgm
.Lfunc_end0:
	.size	s_add_i32, .Lfunc_end0-s_add_i32
                                        ; -- End function
	.section	.AMDGPU.csdata
; Kernel info:
; codeLenInByte = 64
; NumSgprs: 8
; NumVgprs: 1
; ScratchSize: 0
; MemoryBound: 0
; FloatMode: 192
; IeeeMode: 1
; LDSByteSize: 0 bytes/workgroup (compile time only)
; SGPRBlocks: 0
; VGPRBlocks: 0
; NumSGPRsForWavesPerEU: 8
; NumVGPRsForWavesPerEU: 1
; Occupancy: 10
; WaveLimiterHint : 1
; COMPUTE_PGM_RSRC2:USER_SGPR: 2
; COMPUTE_PGM_RSRC2:TRAP_HANDLER: 0
; COMPUTE_PGM_RSRC2:TGID_X_EN: 1
; COMPUTE_PGM_RSRC2:TGID_Y_EN: 0
; COMPUTE_PGM_RSRC2:TGID_Z_EN: 0
; COMPUTE_PGM_RSRC2:TIDIG_COMP_CNT: 0
	.section	.AMDGPU.config
	.long	47176
	.long	11272192
	.long	47180
	.long	132
	.long	47200
	.long	0
	.long	4
	.long	0
	.long	8
	.long	0
	.text
	.globl	s_add_v2i32             ; -- Begin function s_add_v2i32
	.p2align	8
	.type	s_add_v2i32,@function
s_add_v2i32:                            ; @s_add_v2i32
; %bb.0:
	s_load_dwordx4 s[4:7], s[0:1], 0x24
	s_mov_b32 s3, 0xf000
	s_mov_b32 s2, -1
	s_waitcnt lgkmcnt(0)
	s_mov_b32 s0, s4
	s_mov_b32 s1, s5
	s_load_dwordx2 s[4:5], s[6:7], 0x0
	s_load_dwordx2 s[6:7], s[6:7], 0x8
	s_waitcnt lgkmcnt(0)
	s_add_i32 s5, s5, s7
	s_add_i32 s4, s4, s6
	v_mov_b32_e32 v0, s4
	v_mov_b32_e32 v1, s5
	buffer_store_dwordx2 v[0:1], off, s[0:3], 0
	s_endpgm
.Lfunc_end1:
	.size	s_add_v2i32, .Lfunc_end1-s_add_v2i32
                                        ; -- End function
	.section	.AMDGPU.csdata
; Kernel info:
; codeLenInByte = 80
; NumSgprs: 8
; NumVgprs: 2
; ScratchSize: 0
; MemoryBound: 0
; FloatMode: 192
; IeeeMode: 1
; LDSByteSize: 0 bytes/workgroup (compile time only)
; SGPRBlocks: 0
; VGPRBlocks: 0
; NumSGPRsForWavesPerEU: 8
; NumVGPRsForWavesPerEU: 2
; Occupancy: 10
; WaveLimiterHint : 1
; COMPUTE_PGM_RSRC2:USER_SGPR: 2
; COMPUTE_PGM_RSRC2:TRAP_HANDLER: 0
; COMPUTE_PGM_RSRC2:TGID_X_EN: 1
; COMPUTE_PGM_RSRC2:TGID_Y_EN: 0
; COMPUTE_PGM_RSRC2:TGID_Z_EN: 0
; COMPUTE_PGM_RSRC2:TIDIG_COMP_CNT: 0
	.section	.AMDGPU.config
	.long	47176
	.long	11272256
	.long	47180
	.long	132
	.long	47200
	.long	0
	.long	4
	.long	0
	.long	8
	.long	0
	.text
	.globl	s_add_v4i32             ; -- Begin function s_add_v4i32
	.p2align	8
	.type	s_add_v4i32,@function
s_add_v4i32:                            ; @s_add_v4i32
; %bb.0:
	s_load_dwordx4 s[4:7], s[0:1], 0x24
	s_mov_b32 s3, 0xf000
	s_mov_b32 s2, -1
	s_waitcnt lgkmcnt(0)
	s_mov_b32 s0, s4
	s_mov_b32 s1, s5
	s_load_dwordx4 s[8:11], s[6:7], 0x0
	s_load_dwordx4 s[4:7], s[6:7], 0x10
	s_waitcnt lgkmcnt(0)
	s_add_i32 s7, s11, s7
	s_add_i32 s6, s10, s6
	s_add_i32 s5, s9, s5
	s_add_i32 s4, s8, s4
	v_mov_b32_e32 v0, s4
	v_mov_b32_e32 v1, s5
	v_mov_b32_e32 v2, s6
	v_mov_b32_e32 v3, s7
	buffer_store_dwordx4 v[0:3], off, s[0:3], 0
	s_endpgm
.Lfunc_end2:
	.size	s_add_v4i32, .Lfunc_end2-s_add_v4i32
                                        ; -- End function
	.section	.AMDGPU.csdata
; Kernel info:
; codeLenInByte = 96
; NumSgprs: 12
; NumVgprs: 4
; ScratchSize: 0
; MemoryBound: 0
; FloatMode: 192
; IeeeMode: 1
; LDSByteSize: 0 bytes/workgroup (compile time only)
; SGPRBlocks: 1
; VGPRBlocks: 0
; NumSGPRsForWavesPerEU: 12
; NumVGPRsForWavesPerEU: 4
; Occupancy: 10
; WaveLimiterHint : 1
; COMPUTE_PGM_RSRC2:USER_SGPR: 2
; COMPUTE_PGM_RSRC2:TRAP_HANDLER: 0
; COMPUTE_PGM_RSRC2:TGID_X_EN: 1
; COMPUTE_PGM_RSRC2:TGID_Y_EN: 0
; COMPUTE_PGM_RSRC2:TGID_Z_EN: 0
; COMPUTE_PGM_RSRC2:TIDIG_COMP_CNT: 0
	.section	.AMDGPU.config
	.long	47176
	.long	11272323
	.long	47180
	.long	132
	.long	47200
	.long	0
	.long	4
	.long	0
	.long	8
	.long	0
	.text
	.globl	s_add_v8i32             ; -- Begin function s_add_v8i32
	.p2align	8
	.type	s_add_v8i32,@function
s_add_v8i32:                            ; @s_add_v8i32
; %bb.0:                                ; %entry
	s_load_dwordx2 s[20:21], s[0:1], 0x24
	s_load_dwordx8 s[4:11], s[0:1], 0x44
	s_load_dwordx8 s[12:19], s[0:1], 0x64
	s_mov_b32 s23, 0xf000
	s_mov_b32 s22, -1
	s_waitcnt lgkmcnt(0)
	s_add_i32 s0, s7, s15
	s_add_i32 s1, s6, s14
	s_add_i32 s2, s5, s13
	s_add_i32 s3, s4, s12
	s_add_i32 s4, s11, s19
	s_add_i32 s5, s10, s18
	s_add_i32 s6, s9, s17
	s_add_i32 s7, s8, s16
	v_mov_b32_e32 v0, s7
	v_mov_b32_e32 v1, s6
	v_mov_b32_e32 v2, s5
	v_mov_b32_e32 v3, s4
	buffer_store_dwordx4 v[0:3], off, s[20:23], 0 offset:16
	s_nop 0
	v_mov_b32_e32 v0, s3
	v_mov_b32_e32 v1, s2
	v_mov_b32_e32 v2, s1
	v_mov_b32_e32 v3, s0
	buffer_store_dwordx4 v[0:3], off, s[20:23], 0
	s_endpgm
.Lfunc_end3:
	.size	s_add_v8i32, .Lfunc_end3-s_add_v8i32
                                        ; -- End function
	.section	.AMDGPU.csdata
; Kernel info:
; codeLenInByte = 128
; NumSgprs: 24
; NumVgprs: 16
; ScratchSize: 0
; MemoryBound: 0
; FloatMode: 192
; IeeeMode: 1
; LDSByteSize: 0 bytes/workgroup (compile time only)
; SGPRBlocks: 2
; VGPRBlocks: 3
; NumSGPRsForWavesPerEU: 24
; NumVGPRsForWavesPerEU: 16
; Occupancy: 10
; WaveLimiterHint : 1
; COMPUTE_PGM_RSRC2:USER_SGPR: 2
; COMPUTE_PGM_RSRC2:TRAP_HANDLER: 0
; COMPUTE_PGM_RSRC2:TGID_X_EN: 1
; COMPUTE_PGM_RSRC2:TGID_Y_EN: 0
; COMPUTE_PGM_RSRC2:TGID_Z_EN: 0
; COMPUTE_PGM_RSRC2:TIDIG_COMP_CNT: 0
	.section	.AMDGPU.config
	.long	47176
	.long	11272455
	.long	47180
	.long	132
	.long	47200
	.long	0
	.long	4
	.long	0
	.long	8
	.long	0
	.text
	.globl	s_add_v16i32            ; -- Begin function s_add_v16i32
	.p2align	8
	.type	s_add_v16i32,@function
s_add_v16i32:                           ; @s_add_v16i32
; %bb.0:                                ; %entry
	s_load_dwordx2 s[36:37], s[0:1], 0x24
	s_load_dwordx16 s[4:19], s[0:1], 0x64
	s_load_dwordx16 s[20:35], s[0:1], 0xa4
	s_mov_b32 s39, 0xf000
	s_mov_b32 s38, -1
	s_waitcnt lgkmcnt(0)
	s_add_i32 s0, s7, s23
	s_add_i32 s1, s6, s22
	s_add_i32 s2, s5, s21
	s_add_i32 s3, s4, s20
	s_add_i32 s4, s11, s27
	s_add_i32 s5, s10, s26
	s_add_i32 s6, s9, s25
	s_add_i32 s7, s8, s24
	s_add_i32 s8, s15, s31
	s_add_i32 s9, s14, s30
	s_add_i32 s10, s13, s29
	s_add_i32 s11, s12, s28
	s_add_i32 s12, s19, s35
	s_add_i32 s13, s18, s34
	s_add_i32 s14, s17, s33
	s_add_i32 s15, s16, s32
	v_mov_b32_e32 v0, s15
	v_mov_b32_e32 v1, s14
	v_mov_b32_e32 v2, s13
	v_mov_b32_e32 v3, s12
	buffer_store_dwordx4 v[0:3], off, s[36:39], 0 offset:48
	s_nop 0
	v_mov_b32_e32 v0, s11
	v_mov_b32_e32 v1, s10
	v_mov_b32_e32 v2, s9
	v_mov_b32_e32 v3, s8
	buffer_store_dwordx4 v[0:3], off, s[36:39], 0 offset:32
	s_nop 0
	v_mov_b32_e32 v0, s7
	v_mov_b32_e32 v1, s6
	v_mov_b32_e32 v2, s5
	v_mov_b32_e32 v3, s4
	buffer_store_dwordx4 v[0:3], off, s[36:39], 0 offset:16
	s_nop 0
	v_mov_b32_e32 v0, s3
	v_mov_b32_e32 v1, s2
	v_mov_b32_e32 v2, s1
	v_mov_b32_e32 v3, s0
	buffer_store_dwordx4 v[0:3], off, s[36:39], 0
	s_endpgm
.Lfunc_end4:
	.size	s_add_v16i32, .Lfunc_end4-s_add_v16i32
                                        ; -- End function
	.section	.AMDGPU.csdata
; Kernel info:
; codeLenInByte = 216
; NumSgprs: 40
; NumVgprs: 32
; ScratchSize: 0
; MemoryBound: 0
; FloatMode: 192
; IeeeMode: 1
; LDSByteSize: 0 bytes/workgroup (compile time only)
; SGPRBlocks: 4
; VGPRBlocks: 7
; NumSGPRsForWavesPerEU: 40
; NumVGPRsForWavesPerEU: 32
; Occupancy: 8
; WaveLimiterHint : 1
; COMPUTE_PGM_RSRC2:USER_SGPR: 2
; COMPUTE_PGM_RSRC2:TRAP_HANDLER: 0
; COMPUTE_PGM_RSRC2:TGID_X_EN: 1
; COMPUTE_PGM_RSRC2:TGID_Y_EN: 0
; COMPUTE_PGM_RSRC2:TGID_Z_EN: 0
; COMPUTE_PGM_RSRC2:TIDIG_COMP_CNT: 0
	.section	.AMDGPU.config
	.long	47176
	.long	11272256
	.long	47180
	.long	132
	.long	47200
	.long	0
	.long	4
	.long	0
	.long	8
	.long	0
	.text
	.globl	v_add_i32               ; -- Begin function v_add_i32
	.p2align	8
	.type	v_add_i32,@function
v_add_i32:                              ; @v_add_i32
; %bb.0:
	s_load_dwordx4 s[0:3], s[0:1], 0x24
	v_lshlrev_b32_e32 v0, 2, v0
	s_mov_b32 s7, 0xf000
	s_mov_b32 s6, -1
	s_waitcnt lgkmcnt(0)
	v_mov_b32_e32 v1, s3
	v_add_co_u32_e32 v0, vcc, s2, v0
	v_addc_co_u32_e32 v1, vcc, 0, v1, vcc
	global_load_dword v2, v[0:1], off
	global_load_dword v0, v[0:1], off offset:4
	s_mov_b32 s4, s0
	s_mov_b32 s5, s1
	s_waitcnt vmcnt(0)
	v_add_u32_e32 v0, v2, v0
	buffer_store_dword v0, off, s[4:7], 0
	s_endpgm
.Lfunc_end5:
	.size	v_add_i32, .Lfunc_end5-v_add_i32
                                        ; -- End function
	.section	.AMDGPU.csdata
; Kernel info:
; codeLenInByte = 84
; NumSgprs: 10
; NumVgprs: 3
; ScratchSize: 0
; MemoryBound: 0
; FloatMode: 192
; IeeeMode: 1
; LDSByteSize: 0 bytes/workgroup (compile time only)
; SGPRBlocks: 1
; VGPRBlocks: 0
; NumSGPRsForWavesPerEU: 10
; NumVGPRsForWavesPerEU: 3
; Occupancy: 10
; WaveLimiterHint : 1
; COMPUTE_PGM_RSRC2:USER_SGPR: 2
; COMPUTE_PGM_RSRC2:TRAP_HANDLER: 0
; COMPUTE_PGM_RSRC2:TGID_X_EN: 1
; COMPUTE_PGM_RSRC2:TGID_Y_EN: 0
; COMPUTE_PGM_RSRC2:TGID_Z_EN: 0
; COMPUTE_PGM_RSRC2:TIDIG_COMP_CNT: 0
	.section	.AMDGPU.config
	.long	47176
	.long	11272256
	.long	47180
	.long	132
	.long	47200
	.long	0
	.long	4
	.long	0
	.long	8
	.long	0
	.text
	.globl	v_add_imm_i32           ; -- Begin function v_add_imm_i32
	.p2align	8
	.type	v_add_imm_i32,@function
v_add_imm_i32:                          ; @v_add_imm_i32
; %bb.0:
	s_load_dwordx4 s[0:3], s[0:1], 0x24
	v_lshlrev_b32_e32 v0, 2, v0
	s_mov_b32 s7, 0xf000
	s_mov_b32 s6, -1
	s_waitcnt lgkmcnt(0)
	v_mov_b32_e32 v1, s3
	v_add_co_u32_e32 v0, vcc, s2, v0
	v_addc_co_u32_e32 v1, vcc, 0, v1, vcc
	global_load_dword v0, v[0:1], off
	s_mov_b32 s4, s0
	s_mov_b32 s5, s1
	s_waitcnt vmcnt(0)
	v_add_u32_e32 v0, 0x7b, v0
	buffer_store_dword v0, off, s[4:7], 0
	s_endpgm
.Lfunc_end6:
	.size	v_add_imm_i32, .Lfunc_end6-v_add_imm_i32
                                        ; -- End function
	.section	.AMDGPU.csdata
; Kernel info:
; codeLenInByte = 80
; NumSgprs: 10
; NumVgprs: 2
; ScratchSize: 0
; MemoryBound: 0
; FloatMode: 192
; IeeeMode: 1
; LDSByteSize: 0 bytes/workgroup (compile time only)
; SGPRBlocks: 1
; VGPRBlocks: 0
; NumSGPRsForWavesPerEU: 10
; NumVGPRsForWavesPerEU: 2
; Occupancy: 10
; WaveLimiterHint : 1
; COMPUTE_PGM_RSRC2:USER_SGPR: 2
; COMPUTE_PGM_RSRC2:TRAP_HANDLER: 0
; COMPUTE_PGM_RSRC2:TGID_X_EN: 1
; COMPUTE_PGM_RSRC2:TGID_Y_EN: 0
; COMPUTE_PGM_RSRC2:TGID_Z_EN: 0
; COMPUTE_PGM_RSRC2:TIDIG_COMP_CNT: 0
	.section	.AMDGPU.config
	.long	47176
	.long	11272256
	.long	47180
	.long	132
	.long	47200
	.long	0
	.long	4
	.long	0
	.long	8
	.long	0
	.text
	.globl	add64                   ; -- Begin function add64
	.p2align	8
	.type	add64,@function
add64:                                  ; @add64
; %bb.0:                                ; %entry
	s_load_dwordx4 s[4:7], s[0:1], 0x24
	s_load_dwordx2 s[8:9], s[0:1], 0x34
	s_mov_b32 s3, 0xf000
	s_mov_b32 s2, -1
	s_waitcnt lgkmcnt(0)
	s_mov_b32 s0, s4
	s_add_u32 s4, s6, s8
	s_mov_b32 s1, s5
	s_addc_u32 s5, s7, s9
	v_mov_b32_e32 v0, s4
	v_mov_b32_e32 v1, s5
	buffer_store_dwordx2 v[0:1], off, s[0:3], 0
	s_endpgm
.Lfunc_end7:
	.size	add64, .Lfunc_end7-add64
                                        ; -- End function
	.section	.AMDGPU.csdata
; Kernel info:
; codeLenInByte = 68
; NumSgprs: 10
; NumVgprs: 4
; ScratchSize: 0
; MemoryBound: 0
; FloatMode: 192
; IeeeMode: 1
; LDSByteSize: 0 bytes/workgroup (compile time only)
; SGPRBlocks: 1
; VGPRBlocks: 0
; NumSGPRsForWavesPerEU: 10
; NumVGPRsForWavesPerEU: 4
; Occupancy: 10
; WaveLimiterHint : 1
; COMPUTE_PGM_RSRC2:USER_SGPR: 2
; COMPUTE_PGM_RSRC2:TRAP_HANDLER: 0
; COMPUTE_PGM_RSRC2:TGID_X_EN: 1
; COMPUTE_PGM_RSRC2:TGID_Y_EN: 0
; COMPUTE_PGM_RSRC2:TGID_Z_EN: 0
; COMPUTE_PGM_RSRC2:TIDIG_COMP_CNT: 0
	.section	.AMDGPU.config
	.long	47176
	.long	11272256
	.long	47180
	.long	132
	.long	47200
	.long	0
	.long	4
	.long	0
	.long	8
	.long	0
	.text
	.globl	add64_sgpr_vgpr         ; -- Begin function add64_sgpr_vgpr
	.p2align	8
	.type	add64_sgpr_vgpr,@function
add64_sgpr_vgpr:                        ; @add64_sgpr_vgpr
; %bb.0:                                ; %entry
	s_load_dwordx4 s[4:7], s[0:1], 0x24
	s_load_dwordx2 s[8:9], s[0:1], 0x34
	s_mov_b32 s3, 0xf000
	s_mov_b32 s2, -1
	s_waitcnt lgkmcnt(0)
	s_mov_b32 s0, s4
	s_mov_b32 s1, s5
	s_load_dwordx2 s[4:5], s[8:9], 0x0
	s_waitcnt lgkmcnt(0)
	s_add_u32 s4, s6, s4
	s_addc_u32 s5, s7, s5
	v_mov_b32_e32 v0, s4
	v_mov_b32_e32 v1, s5
	buffer_store_dwordx2 v[0:1], off, s[0:3], 0
	s_endpgm
.Lfunc_end8:
	.size	add64_sgpr_vgpr, .Lfunc_end8-add64_sgpr_vgpr
                                        ; -- End function
	.section	.AMDGPU.csdata
; Kernel info:
; codeLenInByte = 80
; NumSgprs: 10
; NumVgprs: 2
; ScratchSize: 0
; MemoryBound: 0
; FloatMode: 192
; IeeeMode: 1
; LDSByteSize: 0 bytes/workgroup (compile time only)
; SGPRBlocks: 1
; VGPRBlocks: 0
; NumSGPRsForWavesPerEU: 10
; NumVGPRsForWavesPerEU: 2
; Occupancy: 10
; WaveLimiterHint : 1
; COMPUTE_PGM_RSRC2:USER_SGPR: 2
; COMPUTE_PGM_RSRC2:TRAP_HANDLER: 0
; COMPUTE_PGM_RSRC2:TGID_X_EN: 1
; COMPUTE_PGM_RSRC2:TGID_Y_EN: 0
; COMPUTE_PGM_RSRC2:TGID_Z_EN: 0
; COMPUTE_PGM_RSRC2:TIDIG_COMP_CNT: 0
	.section	.AMDGPU.config
	.long	47176
	.long	11272193
	.long	47180
	.long	132
	.long	47200
	.long	0
	.long	4
	.long	0
	.long	8
	.long	0
	.text
	.globl	add64_in_branch         ; -- Begin function add64_in_branch
	.p2align	8
	.type	add64_in_branch,@function
add64_in_branch:                        ; @add64_in_branch
; %bb.0:                                ; %entry
	s_load_dwordx8 s[0:7], s[0:1], 0x24
	s_waitcnt lgkmcnt(0)
	s_cmp_lg_u64 s[4:5], 0
	s_cbranch_scc0 BB9_2
; %bb.1:                                ; %else
	s_add_u32 s4, s4, s6
	s_addc_u32 s5, s5, s7
	s_branch BB9_3
BB9_2:                                  ; %if
	s_load_dwordx2 s[4:5], s[2:3], 0x0
BB9_3:                                  ; %endif
	s_waitcnt lgkmcnt(0)
	v_mov_b32_e32 v0, s4
	s_mov_b32 s3, 0xf000
	s_mov_b32 s2, -1
	v_mov_b32_e32 v1, s5
	buffer_store_dwordx2 v[0:1], off, s[0:3], 0
	s_endpgm
.Lfunc_end9:
	.size	add64_in_branch, .Lfunc_end9-add64_in_branch
                                        ; -- End function
	.section	.AMDGPU.csdata
; Kernel info:
; codeLenInByte = 76
; NumSgprs: 8
; NumVgprs: 6
; ScratchSize: 0
; MemoryBound: 0
; FloatMode: 192
; IeeeMode: 1
; LDSByteSize: 0 bytes/workgroup (compile time only)
; SGPRBlocks: 0
; VGPRBlocks: 1
; NumSGPRsForWavesPerEU: 8
; NumVGPRsForWavesPerEU: 6
; Occupancy: 10
; WaveLimiterHint : 1
; COMPUTE_PGM_RSRC2:USER_SGPR: 2
; COMPUTE_PGM_RSRC2:TRAP_HANDLER: 0
; COMPUTE_PGM_RSRC2:TGID_X_EN: 1
; COMPUTE_PGM_RSRC2:TGID_Y_EN: 0
; COMPUTE_PGM_RSRC2:TGID_Z_EN: 0
; COMPUTE_PGM_RSRC2:TIDIG_COMP_CNT: 0
	.section	.AMDGPU.config
	.long	45096
	.long	0
	.long	165608
	.long	0
	.long	45100
	.long	0
	.long	165580
	.long	1
	.long	165584
	.long	1
	.long	4
	.long	0
	.long	8
	.long	0
	.text
	.globl	add_select_vop3         ; -- Begin function add_select_vop3
	.p2align	8
	.type	add_select_vop3,@function
add_select_vop3:                        ; @add_select_vop3
; %bb.0:
	v_add_u32_e32 v0, s0, v0
	;;#ASMSTART
	; def vcc
	;;#ASMEND
	ds_write_b32 v0, v0
	;;#ASMSTART
	; use vcc
	;;#ASMEND
	s_endpgm
.Lfunc_end10:
	.size	add_select_vop3, .Lfunc_end10-add_select_vop3
                                        ; -- End function
	.section	.AMDGPU.csdata
; Kernel info:
; codeLenInByte = 16
; NumSgprs: 3
; NumVgprs: 1
; ScratchSize: 0
; MemoryBound: 0
; FloatMode: 192
; IeeeMode: 0
; LDSByteSize: 0 bytes/workgroup (compile time only)
; SGPRBlocks: 0
; VGPRBlocks: 0
; NumSGPRsForWavesPerEU: 3
; NumVGPRsForWavesPerEU: 1
; Occupancy: 10
; WaveLimiterHint : 0
; COMPUTE_PGM_RSRC2:USER_SGPR: 0
; COMPUTE_PGM_RSRC2:TRAP_HANDLER: 0
; COMPUTE_PGM_RSRC2:TGID_X_EN: 0
; COMPUTE_PGM_RSRC2:TGID_Y_EN: 0
; COMPUTE_PGM_RSRC2:TGID_Z_EN: 0
; COMPUTE_PGM_RSRC2:TIDIG_COMP_CNT: 0

	.section	".note.GNU-stack"
	.amd_amdgpu_isa "amdgcn-unknown-linux-gnu-gfx900"
