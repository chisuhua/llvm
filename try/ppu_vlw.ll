
declare i32 @llvm.ppu.setvl(i32)
;declare <vscale x 1 x i32> @llvm.ppu.vlw(i32*, i32)
declare <vscale x 1 x i32> @llvm.ppu.vlw(i32*)
;declare void @llvm.ppu.vsw(i32*, <vscale x 1 x i32>, i32)
declare void @llvm.ppu.vsw(i32*, <vscale x 1 x i32>)

define void @foo(i32* %A, i32* %R) {
	%vl = call i32 @llvm.ppu.setvl(i32 1)
	; %v1 = call <vscale x 1 x i32> @llvm.ppu.vlw(i32* %A, i32 %vl)
	%v1 = call <vscale x 1 x i32> @llvm.ppu.vlw(i32* %A)
	; call void @llvm.ppu.vsw(i32* %R, <vscale x 1 x i32> %v1, i32 %vl)
	call void @llvm.ppu.vsw(i32* %R, <vscale x 1 x i32> %v1)
	ret void
}
