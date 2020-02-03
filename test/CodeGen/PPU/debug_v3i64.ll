define amdgpu_kernel void @v_test_add_i32(i32 addrspace(1)* %out, i32 addrspace(1)* %in0, i32 addrspace(1)* %in1) #1 {
  %tid = call i32 @llvm.ppu.workitem.id.x()
  %gep.out = getelementptr inbounds i32, i32 addrspace(1)* %out, i32 %tid
  %gep.in0 = getelementptr inbounds i32, i32 addrspace(1)* %in0, i32 %tid
  %gep.in1 = getelementptr inbounds i32, i32 addrspace(1)* %in1, i32 %tid
  %a = load volatile i32, i32 addrspace(1)* %gep.in0
  %b = load volatile i32, i32 addrspace(1)* %gep.in1
  %add = add i32 %a, %b
  store i32 %add, i32 addrspace(1)* %out
  ret void
}

