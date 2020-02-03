

define amdgpu_cs float @add3_bitcast(i32 %a, i32 %b) {
  %x = add i32 %a, %b
  %bc = bitcast i32 %x to float
  ret float %bc
}


define amdgpu_cs void @vadd_inreg(i32 addrspace(1)* %out, i32 %a, i32 %b) {
  %x = add i32 %a, %b
  store i32 %x, i32 addrspace(1)* %out
  ret void
}

define amdgpu_cs void @add_inreg(i32 addrspace(1)* inreg %out, i32 inreg %a, i32 inreg %b) {
  %x = add i32 %a, %b
  store i32 %x, i32 addrspace(1)* %out
  ret void
}

define amdgpu_cs i32 @add_ret(i32 inreg %a, i32 inreg %b) {
  %x = add i32 %a, %b
  ret i32 %x
}
