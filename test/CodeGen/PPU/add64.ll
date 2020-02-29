define amdgpu_kernel void @add32(i64 addrspace(1)* %out, i32 %a, i32 %b) {
entry:
  %add = add i64 %a, %b
  ;%add64 = bitcast i32 %add to i64
  store i64 %add, i64 addrspace(1)* %out
  ret void
}

; FUNC-LABEL: {{^}}add64:
; GCN: s_add_u32
; GCN: s_addc_u32
;define amdgpu_kernel void @add64(i64 addrspace(1)* %out, i64 %a, i64 %b) {
;entry:
;  %add = add i64 %a, %b
;  store i64 %add, i64 addrspace(1)* %out
;  ret void
;}

attributes #0 = { nounwind }
attributes #1 = { nounwind readnone speculatable }
