; RUN: llc -mtriple=amdgcn--amdpal -mcpu=tahiti -verify-machineinstrs < %s | FileCheck -check-prefix=SI %s
; RUN: llc -mtriple=amdgcn--amdpal -mcpu=tonga -mattr=-flat-for-global -verify-machineinstrs < %s | FileCheck -check-prefix=SI %s

; TODO: Some of those tests fail with OS == amdhsa due to unreasonable register
;       allocation differences.

; SI-LABEL: {{^}}s_addk_i32_k0:
; SI: s_load_dword [[VAL:s[0-9]+]]
; SI: s_addk_i32 [[VAL]], 0x41
; SI: v_mov_b32_e32 [[VRESULT:v[0-9]+]], [[VAL]]
; SI: buffer_store_dword [[VRESULT]]
; SI: s_endpgm
define amdgpu_kernel void @s_addk_i32_k0(i32 addrspace(1)* %out, i32 %b) {
  %add = add i32 %b, 65
  store i32 %add, i32 addrspace(1)* %out
  ret void
}
