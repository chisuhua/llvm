define amdgpu_cs void @vadd_inreg(i16 addrspace(1)* %out, i16 %a, i16 %b) {
  %x = add i16 %a, %b
  store i16 %x, i16 addrspace(1)* %out
  ret void
}

