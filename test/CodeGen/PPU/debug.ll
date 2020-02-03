
define amdgpu_kernel void @add(i32 addrspace(1)* %out, i32 %in0, i32 %in1, i32 %in2) {
  ;%tmp2 = sext i8 %tmp1 to i32
  ;%b = load volatile i32, i32 addrspace(1)* %in1
  %add1 = add i32 %in0, %in1
  %add = add i32 %add1, %in2
  store i32 %add, i32 addrspace(1)* %out
  ret void
}


attributes #0 = { nounwind readnone }
attributes #1 = { nounwind }
