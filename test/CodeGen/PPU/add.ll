; RUN: llc -mtriple=ppu -mattr=+t,+r -mattr=-flat-for-global -verify-machineinstrs < %s | FileCheck -enable-var-scope -check-prefixes=GCN,GFX9,FUNC %s

; FUNC-LABEL: {{^}}s_add_i32:
; PPU: s_add_i32 s[[REG:[0-9]+]], {{s[0-9]+, s[0-9]+}}
; PPU: v_mov_b32_e32 v[[V_REG:[0-9]+]], s[[REG]]
; PPU: buffer_store_dword v[[V_REG]],
define amdgpu_kernel void @s_add_i32(i32 addrspace(1)* %out, i32 addrspace(1)* %in) #0 {
  %b_ptr = getelementptr i32, i32 addrspace(1)* %in, i32 1
  %a = load i32, i32 addrspace(1)* %in
  %b = load i32, i32 addrspace(1)* %b_ptr
  %result = add i32 %a, %b
  store i32 %result, i32 addrspace(1)* %out
  ret void
}

; FUNC-LABEL: {{^}}s_add_v2i32:
; GCN: s_add_i32 s{{[0-9]+, s[0-9]+, s[0-9]+}}
; GCN: s_add_i32 s{{[0-9]+, s[0-9]+, s[0-9]+}}
define amdgpu_kernel void @s_add_v2i32(<2 x i32> addrspace(1)* %out, <2 x i32> addrspace(1)* %in) {
  %b_ptr = getelementptr <2 x i32>, <2 x i32> addrspace(1)* %in, i32 1
  %a = load <2 x i32>, <2 x i32> addrspace(1)* %in
  %b = load <2 x i32>, <2 x i32> addrspace(1)* %b_ptr
  %result = add <2 x i32> %a, %b
  store <2 x i32> %result, <2 x i32> addrspace(1)* %out
  ret void
}

; FUNC-LABEL: {{^}}s_add_v4i32:
; GCN: s_add_i32 s{{[0-9]+, s[0-9]+, s[0-9]+}}
; GCN: s_add_i32 s{{[0-9]+, s[0-9]+, s[0-9]+}}
; GCN: s_add_i32 s{{[0-9]+, s[0-9]+, s[0-9]+}}
; GCN: s_add_i32 s{{[0-9]+, s[0-9]+, s[0-9]+}}
define amdgpu_kernel void @s_add_v4i32(<4 x i32> addrspace(1)* %out, <4 x i32> addrspace(1)* %in) {
  %b_ptr = getelementptr <4 x i32>, <4 x i32> addrspace(1)* %in, i32 1
  %a = load <4 x i32>, <4 x i32> addrspace(1)* %in
  %b = load <4 x i32>, <4 x i32> addrspace(1)* %b_ptr
  %result = add <4 x i32> %a, %b
  store <4 x i32> %result, <4 x i32> addrspace(1)* %out
  ret void
}

; FUNC-LABEL: {{^}}s_add_v8i32:
; GCN: s_add_i32
; GCN: s_add_i32
; GCN: s_add_i32
; GCN: s_add_i32
; GCN: s_add_i32
; GCN: s_add_i32
; GCN: s_add_i32
; GCN: s_add_i32
define amdgpu_kernel void @s_add_v8i32(<8 x i32> addrspace(1)* %out, <8 x i32> %a, <8 x i32> %b) {
entry:
  %0 = add <8 x i32> %a, %b
  store <8 x i32> %0, <8 x i32> addrspace(1)* %out
  ret void
}

; FUNC-LABEL: {{^}}s_add_v16i32:
; GCN: s_add_i32
; GCN: s_add_i32
; GCN: s_add_i32
; GCN: s_add_i32
; GCN: s_add_i32
; GCN: s_add_i32
; GCN: s_add_i32
; GCN: s_add_i32
; GCN: s_add_i32
; GCN: s_add_i32
; GCN: s_add_i32
; GCN: s_add_i32
; GCN: s_add_i32
; GCN: s_add_i32
; GCN: s_add_i32
; GCN: s_add_i32
define amdgpu_kernel void @s_add_v16i32(<16 x i32> addrspace(1)* %out, <16 x i32> %a, <16 x i32> %b) {
entry:
  %0 = add <16 x i32> %a, %b
  store <16 x i32> %0, <16 x i32> addrspace(1)* %out
  ret void
}


attributes #0 = { nounwind }
attributes #1 = { nounwind readnone speculatable }
