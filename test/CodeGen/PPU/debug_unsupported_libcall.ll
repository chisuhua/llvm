; FUNC-LABEL: {{^}}mul_32bit_ptr:
; SI: s_mul_i32
; SI-NEXT: s_add_i32
; SI: ds_read_b32
define amdgpu_kernel void @mul_32bit_ptr(float addrspace(1)* %out, [3 x float] addrspace(3)* %lds, i32 %tid) {
  %ptr = getelementptr [3 x float], [3 x float] addrspace(3)* %lds, i32 %tid, i32 0
  %val = load float, float addrspace(3)* %ptr
  store float %val, float addrspace(1)* %out
  ret void
}

