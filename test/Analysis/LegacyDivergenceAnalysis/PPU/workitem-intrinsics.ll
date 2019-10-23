; RUN: opt  -mtriple ppu-- -analyze -divergence %s | FileCheck %s

declare i32 @llvm.ppu.workitem.id.x() #0
declare i32 @llvm.ppu.workitem.id.y() #0
declare i32 @llvm.ppu.workitem.id.z() #0
declare i32 @llvm.ppu.workgroup.id.x() #0
declare i32 @llvm.ppu.workgroup.id.y() #0
declare i32 @llvm.ppu.workgroup.id.z() #0
declare i32 @llvm.ppu.nvvm.tid.x() #0
declare i32 @llvm.ppu.nvvm.tid.y() #0
declare i32 @llvm.ppu.nvvm.tid.z() #0
declare i32 @llvm.ppu.nvvm.laneid() #0
declare i32 @llvm.ppu.nvvm.warpid.x() #0
declare i32 @llvm.ppu.nvvm.warpid.y() #0
declare i32 @llvm.ppu.nvvm.warpid.z() #0
;declare i32 @llvm.ppu.mbcnt.lo(i32, i32) #0
;declare i32 @llvm.ppu.mbcnt.hi(i32, i32) #0

; CHECK: DIVERGENT:  %id.x = call i32 @llvm.ppu.workitem.id.x()
define amdgpu_kernel void @workitem_id_x() #1 {
  %id.x = call i32 @llvm.ppu.workitem.id.x()
  store volatile i32 %id.x, i32 addrspace(1)* undef
  ret void
}

; CHECK: DIVERGENT:  %id.y = call i32 @llvm.ppu.workitem.id.y()
define amdgpu_kernel void @workitem_id_y() #1 {
  %id.y = call i32 @llvm.ppu.workitem.id.y()
  store volatile i32 %id.y, i32 addrspace(1)* undef
  ret void
}

; CHECK: DIVERGENT:  %id.z = call i32 @llvm.ppu.workitem.id.z()
define amdgpu_kernel void @workitem_id_z() #1 {
  %id.z = call i32 @llvm.ppu.workitem.id.z()
  store volatile i32 %id.z, i32 addrspace(1)* undef
  ret void
}

; CHECK: DIVERGENT:  %id.x = call i32 @llvm.ppu.workgroup.id.x()
define amdgpu_kernel void @workgroup_id_x() #1 {
  %id.x = call i32 @llvm.ppu.workgroup.id.x()
  store volatile i32 %id.x, i32 addrspace(1)* undef
  ret void
}

; CHECK: DIVERGENT:  %id.y = call i32 @llvm.ppu.workgroup.id.y()
define amdgpu_kernel void @workgroup_id_y() #1 {
  %id.y = call i32 @llvm.ppu.workgroup.id.y()
  store volatile i32 %id.y, i32 addrspace(1)* undef
  ret void
}

; CHECK: DIVERGENT:  %id.z = call i32 @llvm.ppu.workgroup.id.z()
define amdgpu_kernel void @workgroup_id_z() #1 {
  %id.z = call i32 @llvm.ppu.workgroup.id.z()
  store volatile i32 %id.z, i32 addrspace(1)* undef
  ret void
}

; CHECK: DIVERGENT:  %id.x = call i32 @llvm.ppu.nvvm.tid.x()
define amdgpu_kernel void @nvvm_tid_x() #1 {
  %id.x = call i32 @llvm.ppu.nvvm.tid.x()
  store volatile i32 %id.x, i32 addrspace(1)* undef
  ret void
}

; CHECK: DIVERGENT:  %id.y = call i32 @llvm.ppu.nvvm.tid.y()
define amdgpu_kernel void @nvvm_tid_y() #1 {
  %id.y = call i32 @llvm.ppu.nvvm.tid.y()
  store volatile i32 %id.y, i32 addrspace(1)* undef
  ret void
}

; CHECK: DIVERGENT:  %id.z = call i32 @llvm.ppu.nvvm.tid.z()
define amdgpu_kernel void @nvvm_tid_z() #1 {
  %id.z = call i32 @llvm.ppu.nvvm.tid.z()
  store volatile i32 %id.z, i32 addrspace(1)* undef
  ret void
}

; CHECK: DIVERGENT:  %id.x = call i32 @llvm.ppu.nvvm.laneid()
define amdgpu_kernel void @nvvm_laneid() #1 {
  %id.x = call i32 @llvm.ppu.nvvm.laneid()
  store volatile i32 %id.x, i32 addrspace(1)* undef
  ret void
}

; CHECK: DIVERGENT:  %id.x = call i32 @llvm.ppu.nvvm.warpid.x()
define amdgpu_kernel void @nvvm_warpid_x() #1 {
  %id.x = call i32 @llvm.ppu.nvvm.warpid.x()
  store volatile i32 %id.x, i32 addrspace(1)* undef
  ret void
}

; CHECK: DIVERGENT:  %id.y = call i32 @llvm.ppu.nvvm.warpid.y()
define amdgpu_kernel void @nvvm_warpid_y() #1 {
  %id.y = call i32 @llvm.ppu.nvvm.warpid.y()
  store volatile i32 %id.y, i32 addrspace(1)* undef
  ret void
}

; CHECK: DIVERGENT:  %id.z = call i32 @llvm.ppu.nvvm.warpid.z()
define amdgpu_kernel void @nvvm_warpid_z() #1 {
  %id.z = call i32 @llvm.ppu.nvvm.warpid.z()
  store volatile i32 %id.z, i32 addrspace(1)* undef
  ret void
}

;; CHECK: DIVERGENT:  %mbcnt.lo = call i32 @llvm.ppu.mbcnt.lo(i32 0, i32 0)
;define amdgpu_kernel void @mbcnt_lo() #1 {
;  %mbcnt.lo = call i32 @llvm.ppu.mbcnt.lo(i32 0, i32 0)
;  store volatile i32 %mbcnt.lo, i32 addrspace(1)* undef
;  ret void
;}

;; CHECK: DIVERGENT:  %mbcnt.hi = call i32 @llvm.ppu.mbcnt.hi(i32 0, i32 0)
;define amdgpu_kernel void @mbcnt_hi() #1 {
;  %mbcnt.hi = call i32 @llvm.ppu.mbcnt.hi(i32 0, i32 0)
;  store volatile i32 %mbcnt.hi, i32 addrspace(1)* undef
;  ret void
;}

attributes #0 = { nounwind readnone }
attributes #1 = { nounwind }
