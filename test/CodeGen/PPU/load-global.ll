

define amdgpu_kernel void @global_load_i32(i32 addrspace(1)* %out, i32 addrspace(1)* %in) #0 {
entry:
  %ld = load i32, i32 addrspace(1)* %in
  store i32 %ld, i32 addrspace(1)* %out
  ret void
}

define amdgpu_kernel void @global_load_v2i32(<2 x i32> addrspace(1)* %out, <2 x i32> addrspace(1)* %in) #0 {
entry:
  %ld = load <2 x i32>, <2 x i32> addrspace(1)* %in
  store <2 x i32> %ld, <2 x i32> addrspace(1)* %out
  ret void
}

attributes #0 = { nounwind }
