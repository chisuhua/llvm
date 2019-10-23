; RUN: llc -mcpu=ppu -mtriple=ppu-- -mattr=+v  -show-mc-encoding --verify-machineinstrs < %s | FileCheck -enable-var-scope -check-prefixes=BI,FUNC %s

; FUNC-LABEL: {{^}}v_add_i32:
; BI: ml_add_u32
define amdgpu_kernel void @v_add_i32(i32 addrspace(1)* %out, i32 addrspace(1)* %in) #0 {
  %tid = call i32 @llvm.ppu.nvvm.tid.x() #3
  %gep = getelementptr inbounds i32, i32 addrspace(1)* %in, i32 %tid
  %b_ptr = getelementptr i32, i32 addrspace(1)* %gep, i32 1
  %a = load volatile i32, i32 addrspace(1)* %gep
  %b = load volatile i32, i32 addrspace(1)* %b_ptr
  %result = add i32 %a, %b
  store i32 %result, i32 addrspace(1)* %out
  ret void
}

; Function Attrs: nounwind readnone
declare i32 @llvm.ppu.nvvm.tid.x() #2

attributes #0 = { nounwind }
attributes #1 = { nounwind readnone speculatable }
