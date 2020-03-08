//===-- PPUTargetStreamer.cpp - PPU Target Streamer Methods -----------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file provides PPU specific target streamer methods.
//
//===----------------------------------------------------------------------===//

#include "PPUTargetStreamer.h"
#include "PPU.h"
#include "PPUDefines.h"
#include "Utils/PPUBaseInfo.h"
#include "Utils/PPUKernelCodeTUtils.h"
#include "llvm/ADT/Twine.h"
#include "llvm/BinaryFormat/PPUMetadataVerifier.h"
#include "llvm/BinaryFormat/ELF.h"
#include "llvm/IR/Constants.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/Metadata.h"
#include "llvm/IR/Module.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCELFStreamer.h"
#include "llvm/MC/MCObjectFileInfo.h"
#include "llvm/MC/MCSectionELF.h"
#include "llvm/Support/TargetParser.h"
#include "llvm/Support/FormattedStream.h"
#include "llvm/Support/PPUMetadata.h"

using namespace llvm;
using namespace llvm::PPU;
using namespace llvm::PPU::PPSMD;


PPUTargetStreamer::PPUTargetStreamer(MCStreamer &S) : MCTargetStreamer(S) {}

// This part is for ascii assembly output
PPUTargetAsmStreamer::PPUTargetAsmStreamer(MCStreamer &S,
                                               formatted_raw_ostream &OS)
    : PPUTargetStreamer(S), OS(OS) {}

void PPUTargetAsmStreamer::emitDirectiveOptionPush() {
  OS << "\t.option\tpush\n";
}

void PPUTargetAsmStreamer::emitDirectiveOptionPop() {
  OS << "\t.option\tpop\n";
}

void PPUTargetAsmStreamer::emitDirectiveOptionRVC() {
  OS << "\t.option\trvc\n";
}

void PPUTargetAsmStreamer::emitDirectiveOptionNoRVC() {
  OS << "\t.option\tnorvc\n";
}

void PPUTargetAsmStreamer::emitDirectiveOptionRelax() {
  OS << "\t.option\trelax\n";
}

void PPUTargetAsmStreamer::emitDirectiveOptionNoRelax() {
  OS << "\t.option\tnorelax\n";
}

bool PPUTargetStreamer::EmitHSAMetadataV3(StringRef HSAMetadataString) {
  msgpack::Document HSAMetadataDoc;

  if (!HSAMetadataDoc.fromYAML(HSAMetadataString))
    return false;
  return EmitHSAMetadata(HSAMetadataDoc, false);
}

StringRef PPUTargetStreamer::getArchNameFromElfMach(unsigned ElfMach) {
  PPU::GPUKind AK;

  switch (ElfMach) {
  case ELF::EF_PPU_PPT:      AK = GK_PPU;    break;
  case ELF::EF_PPU_NONE:           AK = GK_NONE;    break;
  }

  StringRef GPUName = "PPU"; // getArchNamePPU(AK);
  if (GPUName != "")
    return GPUName;
}

unsigned PPUTargetStreamer::getElfMach(StringRef GPU) {
  PPU::GPUKind AK = GK_PPU; // parseArchPPU(GPU);

  switch (AK) {
  case GK_PPU:    return ELF::EF_PPU_PPT;
  case GK_NONE:    return ELF::EF_PPU_NONE;
  }

  llvm_unreachable("unknown GPU");
}

// A hook for emitting stuff at the end.
// We use it for emitting the accumulated PAL metadata as directives.
void PPUTargetAsmStreamer::finish() {
    /*
  std::string S;
  getPALMetadata()->toString(S);
  OS << S;
  */
}

void PPUTargetAsmStreamer::EmitDirectivePPUTarget(StringRef Target) {
  OS << "\t.ppu_target \"" << Target << "\"\n";
}
/*
void PPUTargetAsmStreamer::EmitDirectiveHSACodeObjectVersion(
    uint32_t Major, uint32_t Minor) {
  OS << "\t.hsa_code_object_version " <<
        Twine(Major) << "," << Twine(Minor) << '\n';
}

void
PPUTargetAsmStreamer::EmitDirectiveHSACodeObjectISA(uint32_t Major,
                                                       uint32_t Minor,
                                                       uint32_t Stepping,
                                                       StringRef VendorName,
                                                       StringRef ArchName) {
  OS << "\t.hsa_code_object_isa " <<
        Twine(Major) << "," << Twine(Minor) << "," << Twine(Stepping) <<
        ",\"" << VendorName << "\",\"" << ArchName << "\"\n";

}
*/

void
PPUTargetAsmStreamer::EmitAMDKernelCodeT(const amd_kernel_code_t &Header) {
  OS << "\t.amd_kernel_code_t\n";
  dumpAmdKernelCode(&Header, OS, "\t\t");
  OS << "\t.end_amd_kernel_code_t\n";
}

void PPUTargetAsmStreamer::EmitAMDGPUSymbolType(StringRef SymbolName,
                                                   unsigned Type) {
  switch (Type) {
    default: llvm_unreachable("Invalid AMDGPU symbol type");
    case ELF::STT_AMDGPU_HSA_KERNEL:
      OS << "\t.amdgpu_hsa_kernel " << SymbolName << '\n' ;
      break;
  }
}

void PPUTargetAsmStreamer::emitPPULDS(MCSymbol *Symbol, unsigned Size,
                                            unsigned Align) {
  OS << "\t.amdgpu_lds " << Symbol->getName() << ", " << Size << ", " << Align
     << '\n';
}

/*
bool PPUTargetAsmStreamer::EmitISAVersion(StringRef IsaVersionString) {
  OS << "\t.amd_amdgpu_isa \"" << IsaVersionString << "\"\n";
  return true;
}

bool PPUTargetAsmStreamer::EmitHSAMetadata(
    const PPU::PPSMD::Metadata &HSAMetadata) {
  std::string HSAMetadataString;
  if (PPSMD::toString(HSAMetadata, HSAMetadataString))
    return false;

  OS << '\t' << AssemblerDirectiveBegin << '\n';
  OS << HSAMetadataString << '\n';
  OS << '\t' << AssemblerDirectiveEnd << '\n';
  return true;
}
*/

bool PPUTargetAsmStreamer::EmitHSAMetadata(
    msgpack::Document &HSAMetadataDoc, bool Strict) {
  V3::MetadataVerifier Verifier(Strict);
  if (!Verifier.verify(HSAMetadataDoc.getRoot()))
    return false;

  std::string HSAMetadataString;
  raw_string_ostream StrOS(HSAMetadataString);
  HSAMetadataDoc.toYAML(StrOS);

  OS << '\t' << V3::AssemblerDirectiveBegin << '\n';
  OS << StrOS.str() << '\n';
  OS << '\t' << V3::AssemblerDirectiveEnd << '\n';
  return true;
}

bool PPUTargetAsmStreamer::EmitCodeEnd() {
  const uint32_t Encoded_s_code_end = 0xbf9f0000;
  OS << "\t.p2alignl 6, " << Encoded_s_code_end << '\n';
  OS << "\t.fill 48, 4, " << Encoded_s_code_end << '\n';
  return true;
}

void PPUTargetAsmStreamer::EmitPpsKernelDescriptor(
    const MCSubtargetInfo &STI, StringRef KernelName,
    const pps::kernel_descriptor_t &KD, uint64_t NextVGPR, uint64_t NextSGPR,
    bool ReserveVCC, bool ReserveFlatScr, bool ReserveXNACK) {
  // IsaVersion IVersion = getIsaVersion(STI.getCPU());

  OS << "\t.pps_kernel " << KernelName << '\n';

#define PRINT_FIELD(STREAM, DIRECTIVE, KERNEL_DESC, MEMBER_NAME, FIELD_NAME)   \
  STREAM << "\t\t" << DIRECTIVE << " "                                         \
         << PPS_BITS_GET(KERNEL_DESC.MEMBER_NAME, FIELD_NAME) << '\n';

  OS << "\t\t.pps_group_segment_fixed_size " << KD.group_segment_fixed_size
     << '\n';
  OS << "\t\t.pps_private_segment_fixed_size "
     << KD.private_segment_fixed_size << '\n';

  PRINT_FIELD(OS, ".pps_user_sgpr_private_segment_buffer", KD,
              kernel_code_properties,
              pps::KERNEL_CODE_PROPERTY_ENABLE_SGPR_PRIVATE_SEGMENT_BUFFER);
  PRINT_FIELD(OS, ".pps_user_sgpr_dispatch_ptr", KD,
              kernel_code_properties,
              pps::KERNEL_CODE_PROPERTY_ENABLE_SGPR_DISPATCH_PTR);
  PRINT_FIELD(OS, ".pps_user_sgpr_queue_ptr", KD,
              kernel_code_properties,
              pps::KERNEL_CODE_PROPERTY_ENABLE_SGPR_QUEUE_PTR);
  PRINT_FIELD(OS, ".pps_user_sgpr_kernarg_segment_ptr", KD,
              kernel_code_properties,
              pps::KERNEL_CODE_PROPERTY_ENABLE_SGPR_KERNARG_SEGMENT_PTR);
  PRINT_FIELD(OS, ".pps_user_sgpr_dispatch_id", KD,
              kernel_code_properties,
              pps::KERNEL_CODE_PROPERTY_ENABLE_SGPR_DISPATCH_ID);
  PRINT_FIELD(OS, ".pps_user_sgpr_flat_scratch_init", KD,
              kernel_code_properties,
              pps::KERNEL_CODE_PROPERTY_ENABLE_SGPR_FLAT_SCRATCH_INIT);
  PRINT_FIELD(OS, ".pps_user_sgpr_private_segment_size", KD,
              kernel_code_properties,
              pps::KERNEL_CODE_PROPERTY_ENABLE_SGPR_PRIVATE_SEGMENT_SIZE);
  // FIXME schi on below field
#if 0
  /* FIXME
  if (IVersion.Major >= 10)
    PRINT_FIELD(OS, ".pps_wavefront_size32", KD,
                kernel_code_properties,
                pps::KERNEL_CODE_PROPERTY_ENABLE_WAVEFRONT_SIZE32);
                */
  PRINT_FIELD(
      OS, ".pps_system_sgpr_private_segment_wavefront_offset", KD,
      compute_pgm_rsrc2,
      pps::COMPUTE_PGM_RSRC2_ENABLE_SGPR_PRIVATE_SEGMENT_WAVEFRONT_OFFSET);
  PRINT_FIELD(OS, ".pps_system_sgpr_workgroup_id_x", KD,
              compute_pgm_rsrc2,
              pps::COMPUTE_PGM_RSRC2_ENABLE_SGPR_WORKGROUP_ID_X);
  PRINT_FIELD(OS, ".pps_system_sgpr_workgroup_id_y", KD,
              compute_pgm_rsrc2,
              pps::COMPUTE_PGM_RSRC2_ENABLE_SGPR_WORKGROUP_ID_Y);
  PRINT_FIELD(OS, ".pps_system_sgpr_workgroup_id_z", KD,
              compute_pgm_rsrc2,
              pps::COMPUTE_PGM_RSRC2_ENABLE_SGPR_WORKGROUP_ID_Z);
  PRINT_FIELD(OS, ".pps_system_sgpr_workgroup_info", KD,
              compute_pgm_rsrc2,
              pps::COMPUTE_PGM_RSRC2_ENABLE_SGPR_WORKGROUP_INFO);
  PRINT_FIELD(OS, ".pps_system_vgpr_workitem_id", KD,
              compute_pgm_rsrc2,
              pps::COMPUTE_PGM_RSRC2_ENABLE_VGPR_WORKITEM_ID);
#endif
  // These directives are required.
  OS << "\t\t.pps_next_free_vgpr " << NextVGPR << '\n';
  OS << "\t\t.pps_next_free_sgpr " << NextSGPR << '\n';
/*
  if (!ReserveVCC)
    OS << "\t\t.pps_reserve_vcc " << ReserveVCC << '\n';
  if (IVersion.Major >= 7 && !ReserveFlatScr)
    OS << "\t\t.pps_reserve_flat_scratch " << ReserveFlatScr << '\n';
  if (IVersion.Major >= 8 && ReserveXNACK != hasXNACK(STI))
    OS << "\t\t.pps_reserve_xnack_mask " << ReserveXNACK << '\n';
    */
#if 0
  PRINT_FIELD(OS, ".pps_float_round_mode_32", KD,
              compute_pgm_rsrc1,
              pps::COMPUTE_PGM_RSRC1_FLOAT_ROUND_MODE_32);
  PRINT_FIELD(OS, ".pps_float_round_mode_16_64", KD,
              compute_pgm_rsrc1,
              pps::COMPUTE_PGM_RSRC1_FLOAT_ROUND_MODE_16_64);
  PRINT_FIELD(OS, ".pps_float_denorm_mode_32", KD,
              compute_pgm_rsrc1,
              pps::COMPUTE_PGM_RSRC1_FLOAT_DENORM_MODE_32);
  PRINT_FIELD(OS, ".pps_float_denorm_mode_16_64", KD,
              compute_pgm_rsrc1,
              pps::COMPUTE_PGM_RSRC1_FLOAT_DENORM_MODE_16_64);
  PRINT_FIELD(OS, ".pps_dx10_clamp", KD,
              compute_pgm_rsrc1,
              pps::COMPUTE_PGM_RSRC1_ENABLE_DX10_CLAMP);
  PRINT_FIELD(OS, ".pps_ieee_mode", KD,
              compute_pgm_rsrc1,
              pps::COMPUTE_PGM_RSRC1_ENABLE_IEEE_MODE);
  // if (IVersion.Major >= 9)
    PRINT_FIELD(OS, ".pps_fp16_overflow", KD,
                compute_pgm_rsrc1,
                pps::COMPUTE_PGM_RSRC1_FP16_OVFL);
  // if (IVersion.Major >= 10) {
    PRINT_FIELD(OS, ".pps_workgroup_processor_mode", KD,
                compute_pgm_rsrc1,
                pps::COMPUTE_PGM_RSRC1_WGP_MODE);
    PRINT_FIELD(OS, ".pps_memory_ordered", KD,
                compute_pgm_rsrc1,
                pps::COMPUTE_PGM_RSRC1_MEM_ORDERED);
    PRINT_FIELD(OS, ".pps_forward_progress", KD,
                compute_pgm_rsrc1,
                pps::COMPUTE_PGM_RSRC1_FWD_PROGRESS);
  // }
  PRINT_FIELD(
      OS, ".pps_exception_fp_ieee_invalid_op", KD,
      compute_pgm_rsrc2,
      pps::COMPUTE_PGM_RSRC2_ENABLE_EXCEPTION_IEEE_754_FP_INVALID_OPERATION);
  PRINT_FIELD(OS, ".pps_exception_fp_denorm_src", KD,
              compute_pgm_rsrc2,
              pps::COMPUTE_PGM_RSRC2_ENABLE_EXCEPTION_FP_DENORMAL_SOURCE);
  PRINT_FIELD(
      OS, ".pps_exception_fp_ieee_div_zero", KD,
      compute_pgm_rsrc2,
      pps::COMPUTE_PGM_RSRC2_ENABLE_EXCEPTION_IEEE_754_FP_DIVISION_BY_ZERO);
  PRINT_FIELD(OS, ".pps_exception_fp_ieee_overflow", KD,
              compute_pgm_rsrc2,
              pps::COMPUTE_PGM_RSRC2_ENABLE_EXCEPTION_IEEE_754_FP_OVERFLOW);
  PRINT_FIELD(OS, ".pps_exception_fp_ieee_underflow", KD,
              compute_pgm_rsrc2,
              pps::COMPUTE_PGM_RSRC2_ENABLE_EXCEPTION_IEEE_754_FP_UNDERFLOW);
  PRINT_FIELD(OS, ".pps_exception_fp_ieee_inexact", KD,
              compute_pgm_rsrc2,
              pps::COMPUTE_PGM_RSRC2_ENABLE_EXCEPTION_IEEE_754_FP_INEXACT);
  PRINT_FIELD(OS, ".pps_exception_int_div_zero", KD,
              compute_pgm_rsrc2,
              pps::COMPUTE_PGM_RSRC2_ENABLE_EXCEPTION_INT_DIVIDE_BY_ZERO);
#undef PRINT_FIELD
#endif

  OS << "\t.end_pps_kernel\n";
}
