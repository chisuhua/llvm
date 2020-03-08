//===-- PPUAsmPrinter.cpp - PPU LLVM assembly writer ------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains a printer that converts from our internal representation
// of machine-dependent LLVM code to the PPU assembly language.
//
//===----------------------------------------------------------------------===//

#include "PPU.h"
#include "PPUDefines.h"
#include "PPUInstrInfo.h"
#include "PPUAsmPrinter.h"
#include "PPUMCInstLower.h"
#include "MCTargetDesc/PPUInstPrinter.h"
#include "MCTargetDesc/PPUTargetStreamer.h"
#include "MCTargetDesc/PPUMCExpr.h"
#include "PPUTargetMachine.h"
#include "PPUMachineFunctionInfo.h"
#include "TargetInfo/PPUTargetInfo.h"
#include "llvm/CodeGen/AsmPrinter.h"
#include "llvm/CodeGen/MachineConstantPool.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineModuleInfo.h"
#include "llvm/MC/MCAssembler.h"
#include "llvm/MC/MCAsmInfo.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/MC/MCSymbol.h"
#include "llvm/MC/MCCodeEmitter.h"
#include "llvm/Support/PPUMetadata.h"
#include "llvm/Support/TargetRegistry.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Target/TargetLoweringObjectFile.h"
#include "llvm/BinaryFormat/ELF.h"
#include "llvm/IR/DiagnosticInfo.h"

using namespace llvm;

using namespace PPU::PPSMD;

#define DEBUG_TYPE "asm-printer"

// TODO: This should get the default rounding mode from the kernel. We just set
// the default here, but this could change if the OpenCL rounding mode pragmas
// are used.
//
// The denormal mode here should match what is reported by the OpenCL runtime
// for the CL_FP_DENORM bit from CL_DEVICE_{HALF|SINGLE|DOUBLE}_FP_CONFIG, but
// can also be override to flush with the -cl-denorms-are-zero compiler flag.
//
// AMD OpenCL only sets flush none and reports CL_FP_DENORM for double
// precision, and leaves single precision to flush all and does not report
// CL_FP_DENORM for CL_DEVICE_SINGLE_FP_CONFIG. Mesa's OpenCL currently reports
// CL_FP_DENORM for both.
//
// FIXME: It seems some instructions do not support single precision denormals
// regardless of the mode (exp_*_f32, rcp_*_f32, rsq_*_f32, rsq_*f32, sqrt_f32,
// and sin_f32, cos_f32 on most parts).

// We want to use these instructions, and using fp32 denormals also causes
// instructions to run at the double precision rate for the device so it's
// probably best to just report no single precision denormals.
static uint32_t getFPMode(const MachineFunction &F) {
  const PPUSubtarget& ST = F.getSubtarget<PPUSubtarget>();
  // TODO: Is there any real use for the flush in only / flush out only modes?

  uint32_t FP32Denormals =
    ST.hasFP32Denormals() ? FP_DENORM_FLUSH_NONE : FP_DENORM_FLUSH_IN_FLUSH_OUT;

  uint32_t FP64Denormals =
    ST.hasFP64Denormals() ? FP_DENORM_FLUSH_NONE : FP_DENORM_FLUSH_IN_FLUSH_OUT;

  return FP_ROUND_MODE_SP(FP_ROUND_ROUND_TO_NEAREST) |
         FP_ROUND_MODE_DP(FP_ROUND_ROUND_TO_NEAREST) |
         FP_DENORM_MODE_SP(FP32Denormals) |
         FP_DENORM_MODE_DP(FP64Denormals);
}

namespace {
/*
class PPUAsmPrinter : public AsmPrinter {
public:
  explicit PPUAsmPrinter(TargetMachine &TM,
                           std::unique_ptr<MCStreamer> Streamer)
      : AsmPrinter(TM, std::move(Streamer)) {}

  StringRef getPassName() const override { return "PPU Assembly Printer"; }

  void EmitInstruction(const MachineInstr *MI) override;

  bool PrintAsmOperand(const MachineInstr *MI, unsigned OpNo,
                       const char *ExtraCode, raw_ostream &OS) override;
  bool PrintAsmMemoryOperand(const MachineInstr *MI, unsigned OpNo,
                             const char *ExtraCode, raw_ostream &OS) override;

  void EmitToStreamer(MCStreamer &S, const MCInst &Inst);
  bool emitPseudoExpansionLowering(MCStreamer &OutStreamer,
                                   const MachineInstr *MI);

  // Wrapper needed for tblgenned pseudo lowering.
  bool lowerOperand(const MachineOperand &MO, MCOperand &MCOp) const {
    return LowerPPUMachineOperandToMCOperand(MO, MCOp, *this);
  }
};
*/
}

// #define GEN_COMPRESS_INSTR
// #include "PPUGenCompressInstEmitter.inc"
void PPUAsmPrinter::EmitToStreamer(MCStreamer &S, const MCInst &Inst) {
  MCInst CInst;
  /* FIXME on e64 to e32 to e16
  bool Res = compressInst(CInst, Inst, *TM.getMCSubtargetInfo(),
                          OutStreamer->getContext());
  AsmPrinter::EmitToStreamer(*OutStreamer, Res ? CInst : Inst);
                          */
  AsmPrinter::EmitToStreamer(*OutStreamer, Inst);
}

// Simple pseudo-instructions have their lowering (with expansion to real
// instructions) auto-generated.
#include "PPUGenMCPseudoLowering.inc"

/* TODO use AMD instead
void PPUAsmPrinter::EmitInstruction(const MachineInstr *MI) {
  // Do any auto-generated pseudo lowerings.
  if (emitPseudoExpansionLowering(*OutStreamer, MI))
    return;

  MCInst TmpInst;
  LowerPPUMachineInstrToMCInst(MI, TmpInst, *this);
  EmitToStreamer(*OutStreamer, TmpInst);
}
*/

bool PPUAsmPrinter::PrintAsmOperand(const MachineInstr *MI, unsigned OpNo,
                                      const char *ExtraCode, raw_ostream &OS) {
  // First try the generic code, which knows about modifiers like 'c' and 'n'.
  if (!AsmPrinter::PrintAsmOperand(MI, OpNo, ExtraCode, OS))
    return false;

  const MachineOperand &MO = MI->getOperand(OpNo);
  if (ExtraCode && ExtraCode[0]) {
    if (ExtraCode[1] != 0)
      return true; // Unknown modifier.

    switch (ExtraCode[0]) {
    default:
      return true; // Unknown modifier.
    case 'z':      // Print zero register if zero, regular printing otherwise.
      if (MO.isImm() && MO.getImm() == 0) {
        OS << PPUInstPrinter::getRegisterName(PPU::X0);
        return false;
      }
      break;
    case 'i': // Literal 'i' if operand is not a register.
      if (!MO.isReg())
        OS << 'i';
      return false;
    }
  }

  switch (MO.getType()) {
  case MachineOperand::MO_Immediate:
    OS << MO.getImm();
    return false;
  case MachineOperand::MO_Register:
    OS << PPUInstPrinter::getRegisterName(MO.getReg());
    return false;
  default:
    break;
  }

  return true;
}

bool PPUAsmPrinter::PrintAsmMemoryOperand(const MachineInstr *MI,
                                            unsigned OpNo,
                                            const char *ExtraCode,
                                            raw_ostream &OS) {
  if (!ExtraCode) {
    const MachineOperand &MO = MI->getOperand(OpNo);
    // For now, we only support register memory operands in registers and
    // assume there is no addend
    if (!MO.isReg())
      return true;

    OS << "0(" << PPUInstPrinter::getRegisterName(MO.getReg()) << ")";
    return false;
  }

  return AsmPrinter::PrintAsmMemoryOperand(MI, OpNo, ExtraCode, OS);
}

const MCSubtargetInfo *PPUAsmPrinter::getGlobalSTI() const {
  return TM.getMCSubtargetInfo();
}

PPUTargetStreamer* PPUAsmPrinter::getTargetStreamer() const {
  if (!OutStreamer)
    return nullptr;
  return static_cast<PPUTargetStreamer*>(OutStreamer->getTargetStreamer());
}

PPUAsmPrinter::PPUAsmPrinter(TargetMachine &TM,
                                   std::unique_ptr<MCStreamer> Streamer)
  : AsmPrinter(TM, std::move(Streamer)) {
    if (PPU::hasCodeObjectV3(getGlobalSTI()))
      HSAMetadataStream.reset(new MetadataStreamerV3());
}

StringRef PPUAsmPrinter::getPassName() const {
  return "PPU Assembly Printer";
}

// Force static initialization.
extern "C" void LLVMInitializePPUAsmPrinter() {
  RegisterAsmPrinter<PPUAsmPrinter> X(getThePPUTarget());
}


bool PPUAsmPrinter::lowerOperand(const MachineOperand &MO,
                                    MCOperand &MCOp) const {
  const PPUSubtarget &STI = MF->getSubtarget<PPUSubtarget>();
  PPUMCInstLower MCInstLowering(OutContext, STI, *this);
  return MCInstLowering.lowerOperand(MO, MCOp);
}

static const MCExpr *lowerAddrSpaceCast(const TargetMachine &TM,
                                        const Constant *CV,
                                        MCContext &OutContext) {
  // TargetMachine does not support llvm-style cast. Use C++-style cast.
  // This is safe since TM is always of type AMDGPUTargetMachine or its
  // derived class.
  auto &AT = static_cast<const PPUTargetMachine&>(TM);
  auto *CE = dyn_cast<ConstantExpr>(CV);

  // Lower null pointers in private and local address space.
  // Clang generates addrspacecast for null pointers in private and local
  // address space, which needs to be lowered.
  if (CE && CE->getOpcode() == Instruction::AddrSpaceCast) {
    auto Op = CE->getOperand(0);
    auto SrcAddr = Op->getType()->getPointerAddressSpace();
    if (Op->isNullValue() && AT.getNullPointerValue(SrcAddr) == 0) {
      auto DstAddr = CE->getType()->getPointerAddressSpace();
      return MCConstantExpr::create(AT.getNullPointerValue(DstAddr),
        OutContext);
    }
  }
  return nullptr;
}

const MCExpr *PPUAsmPrinter::lowerConstant(const Constant *CV) {
  if (const MCExpr *E = lowerAddrSpaceCast(TM, CV, OutContext))
    return E;
  return AsmPrinter::lowerConstant(CV);
}

void PPUAsmPrinter::EmitInstruction(const MachineInstr *MI) {
  if (emitPseudoExpansionLowering(*OutStreamer, MI))
    return;

  const PPUSubtarget &STI = MF->getSubtarget<PPUSubtarget>();
  PPUMCInstLower MCInstLowering(OutContext, STI, *this);

  StringRef Err;
  if (!STI.getInstrInfo()->verifyInstruction(*MI, Err)) {
    LLVMContext &C = MI->getParent()->getParent()->getFunction().getContext();
    C.emitError("Illegal instruction detected: " + Err);
    MI->print(errs());
  }

  if (MI->isBundle()) {
    const MachineBasicBlock *MBB = MI->getParent();
    MachineBasicBlock::const_instr_iterator I = ++MI->getIterator();
    while (I != MBB->instr_end() && I->isInsideBundle()) {
      EmitInstruction(&*I);
      ++I;
    }
  } else {
    // We don't want SI_MASK_BRANCH/SI_RETURN_TO_EPILOG encoded. They are
    // placeholder terminator instructions and should only be printed as
    // comments.
    if (MI->getOpcode() == PPU::SI_MASK_BRANCH) {
      if (isVerbose()) {
        SmallVector<char, 16> BBStr;
        raw_svector_ostream Str(BBStr);

        const MachineBasicBlock *MBB = MI->getOperand(0).getMBB();
        const MCSymbolRefExpr *Expr
          = MCSymbolRefExpr::create(MBB->getSymbol(), OutContext);
        Expr->print(Str, MAI);
        OutStreamer->emitRawComment(Twine(" mask branch ") + BBStr);
      }

      return;
    }

    if (MI->getOpcode() == PPU::SI_RETURN_TO_EPILOG) {
      if (isVerbose())
        OutStreamer->emitRawComment(" return to shader part epilog");
      return;
    }

    if (MI->getOpcode() == PPU::WAVE_BARRIER) {
      if (isVerbose())
        OutStreamer->emitRawComment(" wave barrier");
      return;
    }
/* FIXME PPU need to enable SI_MASKED_UNREACHABLE at PPUInstrInfoT.td
    if (MI->getOpcode() == PPU::SI_MASKED_UNREACHABLE) {
      if (isVerbose())
        OutStreamer->emitRawComment(" divergent unreachable");
      return;
    }
    */

    MCInst TmpInst;
    MCInstLowering.lower(MI, TmpInst);
    EmitToStreamer(*OutStreamer, TmpInst);

#ifdef EXPENSIVE_CHECKS
    // Sanity-check getInstSizeInBytes on explicitly specified CPUs (it cannot
    // work correctly for the generic CPU).
    //
    // The isPseudo check really shouldn't be here, but unfortunately there are
    // some negative lit tests that depend on being able to continue through
    // here even when pseudo instructions haven't been lowered.
    if (!MI->isPseudo() && STI.isCPUStringValid(STI.getCPU())) {
      SmallVector<MCFixup, 4> Fixups;
      SmallVector<char, 16> CodeBytes;
      raw_svector_ostream CodeStream(CodeBytes);

      std::unique_ptr<MCCodeEmitter> InstEmitter(createSIMCCodeEmitter(
          *STI.getInstrInfo(), *OutContext.getRegisterInfo(), OutContext));
      InstEmitter->encodeInstruction(TmpInst, CodeStream, Fixups, STI);

      assert(CodeBytes.size() == STI.getInstrInfo()->getInstSizeInBytes(*MI));
    }
#endif

    if (DumpCodeInstEmitter) {
      // Disassemble instruction/operands to text
      DisasmLines.resize(DisasmLines.size() + 1);
      std::string &DisasmLine = DisasmLines.back();
      raw_string_ostream DisasmStream(DisasmLine);

      PPUInstPrinter InstPrinter(*TM.getMCAsmInfo(), *STI.getInstrInfo(),
                                    *STI.getRegisterInfo());
      InstPrinter.printInst(&TmpInst, DisasmStream, StringRef(), STI);

      // Disassemble instruction/operands to hex representation.
      SmallVector<MCFixup, 4> Fixups;
      SmallVector<char, 16> CodeBytes;
      raw_svector_ostream CodeStream(CodeBytes);

      DumpCodeInstEmitter->encodeInstruction(
          TmpInst, CodeStream, Fixups, MF->getSubtarget<MCSubtargetInfo>());
      HexLines.resize(HexLines.size() + 1);
      std::string &HexLine = HexLines.back();
      raw_string_ostream HexStream(HexLine);

      for (size_t i = 0; i < CodeBytes.size(); i += 4) {
        unsigned int CodeDWord = *(unsigned int *)&CodeBytes[i];
        HexStream << format("%s%08X", (i > 0 ? " " : ""), CodeDWord);
      }

      DisasmStream.flush();
      DisasmLineMaxLen = std::max(DisasmLineMaxLen, DisasmLine.size());
    }
  }
}

// below hook the EmitStreamer 
void PPUAsmPrinter::EmitFunctionBodyStart() {
  const PPUMachineFunctionInfo &MFI = *MF->getInfo<PPUMachineFunctionInfo>();
  if (!MFI.isEntryFunction())
    return;

  const PPUSubtarget &STM = MF->getSubtarget<PPUSubtarget>();
  const Function &F = MF->getFunction();
  if (!PPU::isCompute(MF)) return;
  /*
  if (!STM.hasCodeObjectV3() && STM.isPPSOS(F) &&
      (F.getCallingConv() == CallingConv::AMDGPU_KERNEL ||
       F.getCallingConv() == CallingConv::SPIR_KERNEL)) {
    amd_kernel_code_t KernelCode;
    getAmdKernelCode(KernelCode, CurrentProgramInfo, *MF);
    getTargetStreamer()->EmitAMDKernelCodeT(KernelCode);
  }
  */
  // I assueme device should have V3
  // if (!PPU::hasCodeObjectV3(getGlobalSTI()))
  //  return;

  // FIXME hwo to enitm deive only not by PPSOS
  // if (STM.isPPSOS())
  if (PPU::hasCodeObjectV3(getGlobalSTI())) {
    HSAMetadataStream->emitKernel(*MF, CurrentProgramInfo);
  }
}

void PPUAsmPrinter::EmitFunctionBodyEnd() {
  const PPUMachineFunctionInfo &MFI = *MF->getInfo<PPUMachineFunctionInfo>();
  if (!PPU::isCompute(MF)) return;
  if (!MFI.isEntryFunction())
    return;
/*
 I assume If V3 is not enabled , then it is not device code
  if (!IsaInfo::hasCodeObjectV3(getGlobalSTI()) ||
      TM.getTargetTriple().getOS() != Triple::AMDHSA)
    return;
*/
  if (!PPU::hasCodeObjectV3(getGlobalSTI()))
    return;

  auto &Streamer = getTargetStreamer()->getStreamer();
  auto &Context = Streamer.getContext();
  auto &ObjectFileInfo = *Context.getObjectFileInfo();
  auto &ReadOnlySection = *ObjectFileInfo.getReadOnlySection();

  Streamer.PushSection();
  Streamer.SwitchSection(&ReadOnlySection);

  // CP microcode requires the kernel descriptor to be allocated on 64 byte
  // alignment.
  Streamer.EmitValueToAlignment(64, 0, 1, 0);
  if (ReadOnlySection.getAlignment() < 64)
    ReadOnlySection.setAlignment(64);

  const MCSubtargetInfo &STI = MF->getSubtarget();

  SmallString<128> KernelName;
  getNameWithPrefix(KernelName, &MF->getFunction());
  getTargetStreamer()->EmitPpsKernelDescriptor(
      STI, KernelName, getPpsKernelDescriptor(*MF, CurrentProgramInfo),
      CurrentProgramInfo.NumVGPRsForWavesPerEU,
      CurrentProgramInfo.NumSGPRsForWavesPerEU -
          PPU::getNumExtraSGPRs(&STI,
                                    CurrentProgramInfo.VCCUsed,
                                    CurrentProgramInfo.FlatUsed),
      CurrentProgramInfo.VCCUsed, CurrentProgramInfo.FlatUsed,
      false/*hasXNACK(STI)*/);

  Streamer.PopSection();
}
/*
void PPUAsmPrinter::EmitFunctionEntryLabel() {
  if (IsaInfo::hasCodeObjectV3(getGlobalSTI()) &&
      TM.getTargetTriple().getOS() == Triple::AMDHSA) {
    AsmPrinter::EmitFunctionEntryLabel();
    return;
  }

  const SIMachineFunctionInfo *MFI = MF->getInfo<SIMachineFunctionInfo>();
  const PPUSubtarget &STM = MF->getSubtarget<PPUSubtarget>();
  if (MFI->isEntryFunction() && STM.isAmdHsaOrMesa(MF->getFunction())) {
    SmallString<128> SymbolName;
    getNameWithPrefix(SymbolName, &MF->getFunction()),
    getTargetStreamer()->EmitAMDGPUSymbolType(
        SymbolName, ELF::STT_AMDGPU_HSA_KERNEL);
  }
  if (DumpCodeInstEmitter) {
    // Disassemble function name label to text.
    DisasmLines.push_back(MF->getName().str() + ":");
    DisasmLineMaxLen = std::max(DisasmLineMaxLen, DisasmLines.back().size());
    HexLines.push_back("");
  }

  AsmPrinter::EmitFunctionEntryLabel();
}
*/

void PPUAsmPrinter::EmitBasicBlockStart(const MachineBasicBlock &MBB) {
  if (DumpCodeInstEmitter && !isBlockOnlyReachableByFallthrough(&MBB)) {
    // Write a line for the basic block label if it is not only fallthrough.
    DisasmLines.push_back(
        (Twine("BB") + Twine(getFunctionNumber())
         + "_" + Twine(MBB.getNumber()) + ":").str());
    DisasmLineMaxLen = std::max(DisasmLineMaxLen, DisasmLines.back().size());
    HexLines.push_back("");
  }
  AsmPrinter::EmitBasicBlockStart(MBB);
}

void PPUAsmPrinter::EmitGlobalVariable(const GlobalVariable *GV) {
  if (GV->getAddressSpace() == AMDGPUAS::LOCAL_ADDRESS) {
    if (GV->hasInitializer() && !isa<UndefValue>(GV->getInitializer())) {
      OutContext.reportError({},
                             Twine(GV->getName()) +
                                 ": unsupported initializer for address space");
      return;
    }

    // LDS variables aren't emitted in HSA or PAL yet.
    const Triple::OSType OS = TM.getTargetTriple().getOS();
    if (OS == Triple::PPS)
      return;

    MCSymbol *GVSym = getSymbol(GV);

    GVSym->redefineIfPossible();
    if (GVSym->isDefined() || GVSym->isVariable())
      report_fatal_error("symbol '" + Twine(GVSym->getName()) +
                         "' is already defined");

    const DataLayout &DL = GV->getParent()->getDataLayout();
    uint64_t Size = DL.getTypeAllocSize(GV->getValueType());
    unsigned Align = GV->getAlignment();
    if (!Align)
      Align = 4;

    EmitVisibility(GVSym, GV->getVisibility(), !GV->isDeclaration());
    EmitLinkage(GV, GVSym);
    if (auto TS = getTargetStreamer())
      TS->emitPPULDS(GVSym, Size, Align);
    return;
  }

  AsmPrinter::EmitGlobalVariable(GV);
}

void PPUAsmPrinter::EmitStartOfAsmFile(Module &M) {
  // if (!PPU::isCompute(MF)) return;
  if (PPU::hasCodeObjectV3(getGlobalSTI())) {
    std::string ExpectedTarget;
    raw_string_ostream ExpectedTargetOS(ExpectedTarget);
    PPU::streamIsaVersion(getGlobalSTI(), ExpectedTargetOS);

    getTargetStreamer()->EmitDirectivePPUTarget(ExpectedTarget);
  }

  // if (TM.getTargetTriple().getOS() == Triple::PPS)
  if (PPU::hasCodeObjectV3(getGlobalSTI()))
    HSAMetadataStream->begin(M);

}

void PPUAsmPrinter::EmitEndOfAsmFile(Module &M) {
  // if (!PPU::isCompute(MF)) return;
  // Following code requires TargetStreamer to be present.
  if (!getTargetStreamer())
    return;

  // Emit HSA Metadata (NT_AMD_AMDGPU_HSA_METADATA).
  // if (TM.getTargetTriple().getOS() == Triple::PPS) {
  if (PPU::hasCodeObjectV3(getGlobalSTI())) {
    HSAMetadataStream->end();
    bool Success = HSAMetadataStream->emitTo(*getTargetStreamer());
    (void)Success;
    assert(Success && "Malformed HSA Metadata");
  }
}

bool PPUAsmPrinter::isBlockOnlyReachableByFallthrough(
  const MachineBasicBlock *MBB) const {
  if (!AsmPrinter::isBlockOnlyReachableByFallthrough(MBB))
    return false;

  if (MBB->empty())
    return true;

  // If this is a block implementing a long branch, an expression relative to
  // the start of the block is needed.  to the start of the block.
  // XXX - Is there a smarter way to check this?
  return (MBB->back().getOpcode() != PPU::S_SETPC_B64);
}

bool PPUAsmPrinter::doFinalization(Module &M) {
  CallGraphResourceInfo.clear();

  // Pad with s_code_end to help tools and guard against instruction prefetch
  // causing stale data in caches. Arguably this should be done by the linker,
  // which is why this isn't done for Mesa.
  const MCSubtargetInfo &STI = *getGlobalSTI();
  if (/*AMDGPU::isGFX10(STI) &&*/
      STI.getTargetTriple().getOS() == Triple::PPS) {
    OutStreamer->SwitchSection(getObjFileLowering().getTextSection());
    getTargetStreamer()->EmitCodeEnd();
  }

  return AsmPrinter::doFinalization(M);
}

// Print comments that apply to both callable functions and entry points.
void PPUAsmPrinter::emitCommonFunctionComments(
  uint32_t NumVGPR,
  uint32_t NumSGPR,
  uint64_t ScratchSize,
  uint64_t CodeSize,
  const PPUMachineFunction *MFI) {
  OutStreamer->emitRawComment(" codeLenInByte = " + Twine(CodeSize), false);
  OutStreamer->emitRawComment(" NumSgprs: " + Twine(NumSGPR), false);
  OutStreamer->emitRawComment(" NumVgprs: " + Twine(NumVGPR), false);
  OutStreamer->emitRawComment(" ScratchSize: " + Twine(ScratchSize), false);
  OutStreamer->emitRawComment(" MemoryBound: " + Twine(MFI->isMemoryBound()),
                              false);
}

uint16_t PPUAsmPrinter::getPpsKernelCodeProperties(
    const MachineFunction &MF) const {
  const PPUMachineFunctionInfo &MFI = *MF.getInfo<PPUMachineFunctionInfo>();
  uint16_t KernelCodeProperties = 0;

  if (MFI.hasPrivateSegmentBuffer()) {
    KernelCodeProperties |=
        pps::KERNEL_CODE_PROPERTY_ENABLE_SGPR_PRIVATE_SEGMENT_BUFFER;
  }
  if (MFI.hasDispatchPtr()) {
    KernelCodeProperties |=
        pps::KERNEL_CODE_PROPERTY_ENABLE_SGPR_DISPATCH_PTR;
  }
  if (MFI.hasQueuePtr()) {
    KernelCodeProperties |=
        pps::KERNEL_CODE_PROPERTY_ENABLE_SGPR_QUEUE_PTR;
  }
  if (MFI.hasKernargSegmentPtr()) {
    KernelCodeProperties |=
        pps::KERNEL_CODE_PROPERTY_ENABLE_SGPR_KERNARG_SEGMENT_PTR;
  }
  if (MFI.hasDispatchID()) {
    KernelCodeProperties |=
        pps::KERNEL_CODE_PROPERTY_ENABLE_SGPR_DISPATCH_ID;
  }
  if (MFI.hasFlatScratchInit()) {
    KernelCodeProperties |=
        pps::KERNEL_CODE_PROPERTY_ENABLE_SGPR_FLAT_SCRATCH_INIT;
  }
  if (MF.getSubtarget<PPUSubtarget>().isWave32()) {
    KernelCodeProperties |=
        pps::KERNEL_CODE_PROPERTY_ENABLE_WAVEFRONT_SIZE32;
  }

  return KernelCodeProperties;
}

pps::kernel_descriptor_t PPUAsmPrinter::getPpsKernelDescriptor(
    const MachineFunction &MF,
    const PPTProgramInfo &PI) const {
  pps::kernel_descriptor_t KernelDescriptor;
  memset(&KernelDescriptor, 0x0, sizeof(KernelDescriptor));

  assert(isUInt<32>(PI.ScratchSize));
  assert(isUInt<32>(PI.ComputePGMRSrc1));
  assert(isUInt<32>(PI.ComputePGMRSrc2));

  KernelDescriptor.group_segment_fixed_size = PI.LDSSize;
  KernelDescriptor.private_segment_fixed_size = PI.ScratchSize;
  KernelDescriptor.compute_pgm_rsrc1 = PI.ComputePGMRSrc1;
  KernelDescriptor.compute_pgm_rsrc2 = PI.ComputePGMRSrc2;
  KernelDescriptor.kernel_code_properties = getPpsKernelCodeProperties(MF);

  return KernelDescriptor;
}

bool PPUAsmPrinter::runOnMachineFunction(MachineFunction &MF) {

  if (!PPU::isCompute(&MF)) {
    SetupMachineFunction(MF);
    EmitFunctionBody();
    return false;
  }
  CurrentProgramInfo = PPTProgramInfo();

  const PPUMachineFunction *MFI = MF.getInfo<PPUMachineFunction>();

  // The starting address of all shader programs must be 256 bytes aligned.
  // Regular functions just need the basic required instruction alignment.
  MF.setAlignment(MFI->isEntryFunction() ? 8 : 2);

  SetupMachineFunction(MF);

  const PPUSubtarget &STM = MF.getSubtarget<PPUSubtarget>();
  MCContext &Context = getObjFileLowering().getContext();
  // FIXME: This should be an explicit check for Mesa.
  /*
  if (!STM.isPPSOS()) {
    MCSectionELF *ConfigSection =
        Context.getELFSection(".AMDGPU.config", ELF::SHT_PROGBITS, 0);
    OutStreamer->SwitchSection(ConfigSection);
  }
  */

  if (MFI->isEntryFunction()) {
    getPPTProgramInfo(CurrentProgramInfo, MF);
  } else {
    auto I = CallGraphResourceInfo.insert(
      std::make_pair(&MF.getFunction(), PPTFunctionResourceInfo()));
    PPTFunctionResourceInfo &Info = I.first->second;
    assert(I.second && "should only be called once per function");
    Info = analyzeResourceUsage(MF);
  }
/*
  if (STM.isAmdPalOS())
    EmitPALMetadata(MF, CurrentProgramInfo);
  else if (!STM.isAmdHsaOS()) {
    EmitProgramInfoSI(MF, CurrentProgramInfo);
  }
*/
  DumpCodeInstEmitter = nullptr;
  if (STM.dumpCode()) {
    // For -dumpcode, get the assembler out of the streamer, even if it does
    // not really want to let us have it. This only works with -filetype=obj.
    bool SaveFlag = OutStreamer->getUseAssemblerInfoForParsing();
    OutStreamer->setUseAssemblerInfoForParsing(true);
    MCAssembler *Assembler = OutStreamer->getAssemblerPtr();
    OutStreamer->setUseAssemblerInfoForParsing(SaveFlag);
    if (Assembler)
      DumpCodeInstEmitter = Assembler->getEmitterPtr();
  }

  DisasmLines.clear();
  HexLines.clear();
  DisasmLineMaxLen = 0;

  EmitFunctionBody();

  if (isVerbose()) {
      /*
    MCSectionELF *CommentSection =
        Context.getELFSection(".AMDGPU.csdata", ELF::SHT_PROGBITS, 0);
    OutStreamer->SwitchSection(CommentSection);
    */

    if (!MFI->isEntryFunction()) {
      OutStreamer->emitRawComment(" Function info:", false);
      PPTFunctionResourceInfo &Info = CallGraphResourceInfo[&MF.getFunction()];
      emitCommonFunctionComments(
        Info.NumVGPR,
        Info.getTotalNumSGPRs(MF.getSubtarget<PPUSubtarget>()),
        Info.PrivateSegmentSize,
        getFunctionCodeSize(MF), MFI);
      return false;
    }

    OutStreamer->emitRawComment(" Kernel info:", false);
    emitCommonFunctionComments(CurrentProgramInfo.NumVGPR,
                               CurrentProgramInfo.NumSGPR,
                               CurrentProgramInfo.ScratchSize,
                               getFunctionCodeSize(MF), MFI);

    OutStreamer->emitRawComment(
      " FloatMode: " + Twine(CurrentProgramInfo.FloatMode), false);
    OutStreamer->emitRawComment(
      " IeeeMode: " + Twine(CurrentProgramInfo.IEEEMode), false);
    OutStreamer->emitRawComment(
      " LDSByteSize: " + Twine(CurrentProgramInfo.LDSSize) +
      " bytes/workgroup (compile time only)", false);

    OutStreamer->emitRawComment(
      " SGPRBlocks: " + Twine(CurrentProgramInfo.SGPRBlocks), false);
    OutStreamer->emitRawComment(
      " VGPRBlocks: " + Twine(CurrentProgramInfo.VGPRBlocks), false);

    OutStreamer->emitRawComment(
      " NumSGPRsForWavesPerEU: " +
      Twine(CurrentProgramInfo.NumSGPRsForWavesPerEU), false);
    OutStreamer->emitRawComment(
      " NumVGPRsForWavesPerEU: " +
      Twine(CurrentProgramInfo.NumVGPRsForWavesPerEU), false);

    OutStreamer->emitRawComment(
      " Occupancy: " +
      Twine(CurrentProgramInfo.Occupancy), false);

    OutStreamer->emitRawComment(
      " WaveLimiterHint : " + Twine(MFI->needsWaveLimiter()), false);
/* FIXME PPU
    OutStreamer->emitRawComment(
      " COMPUTE_PGM_RSRC2:USER_SGPR: " +
      Twine(G_00B84C_USER_SGPR(CurrentProgramInfo.ComputePGMRSrc2)), false);
    OutStreamer->emitRawComment(
      " COMPUTE_PGM_RSRC2:TRAP_HANDLER: " +
      Twine(G_00B84C_TRAP_HANDLER(CurrentProgramInfo.ComputePGMRSrc2)), false);
    OutStreamer->emitRawComment(
      " COMPUTE_PGM_RSRC2:TGID_X_EN: " +
      Twine(G_00B84C_TGID_X_EN(CurrentProgramInfo.ComputePGMRSrc2)), false);
    OutStreamer->emitRawComment(
      " COMPUTE_PGM_RSRC2:TGID_Y_EN: " +
      Twine(G_00B84C_TGID_Y_EN(CurrentProgramInfo.ComputePGMRSrc2)), false);
    OutStreamer->emitRawComment(
      " COMPUTE_PGM_RSRC2:TGID_Z_EN: " +
      Twine(G_00B84C_TGID_Z_EN(CurrentProgramInfo.ComputePGMRSrc2)), false);
    OutStreamer->emitRawComment(
      " COMPUTE_PGM_RSRC2:TIDIG_COMP_CNT: " +
      Twine(G_00B84C_TIDIG_COMP_CNT(CurrentProgramInfo.ComputePGMRSrc2)),
      false);
*/
  }

  if (DumpCodeInstEmitter) {
/* FIXME
    OutStreamer->SwitchSection(
        Context.getELFSection(".AMDGPU.disasm", ELF::SHT_NOTE, 0));
*/
    for (size_t i = 0; i < DisasmLines.size(); ++i) {
      std::string Comment = "\n";
      if (!HexLines[i].empty()) {
        Comment = std::string(DisasmLineMaxLen - DisasmLines[i].size(), ' ');
        Comment += " ; " + HexLines[i] + "\n";
      }

      OutStreamer->EmitBytes(StringRef(DisasmLines[i]));
      OutStreamer->EmitBytes(StringRef(Comment));
    }
  }

  return false;
}

uint64_t PPUAsmPrinter::getFunctionCodeSize(const MachineFunction &MF) const {
  const PPUSubtarget &STM = MF.getSubtarget<PPUSubtarget>();
  const PPUInstrInfo *TII = STM.getInstrInfo();

  uint64_t CodeSize = 0;

  for (const MachineBasicBlock &MBB : MF) {
    for (const MachineInstr &MI : MBB) {
      // TODO: CodeSize should account for multiple functions.

      // TODO: Should we count size of debug info?
      if (MI.isDebugInstr())
        continue;

      CodeSize += TII->getInstSizeInBytes(MI);
    }
  }

  return CodeSize;
}

static bool hasAnyNonFlatUseOfReg(const MachineRegisterInfo &MRI,
                                  const PPUInstrInfo &TII,
                                  unsigned Reg) {
  for (const MachineOperand &UseOp : MRI.reg_operands(Reg)) {
    if (!UseOp.isImplicit() || !TII.isFLAT(*UseOp.getParent()))
      return true;
  }

  return false;
}

int32_t PPUAsmPrinter::PPTFunctionResourceInfo::getTotalNumSGPRs(
  const PPUSubtarget &ST) const {
  return NumExplicitSGPR + PPU::getNumExtraSGPRs(&ST,
                                                     UsesVCC, UsesFlatScratch);
}

PPUAsmPrinter::PPTFunctionResourceInfo PPUAsmPrinter::analyzeResourceUsage(
  const MachineFunction &MF) const {
  PPTFunctionResourceInfo Info;

  const PPUMachineFunctionInfo *MFI = MF.getInfo<PPUMachineFunctionInfo>();
  const PPUSubtarget &ST = MF.getSubtarget<PPUSubtarget>();
  const MachineFrameInfo &FrameInfo = MF.getFrameInfo();
  const MachineRegisterInfo &MRI = MF.getRegInfo();
  const PPUInstrInfo *TII = ST.getInstrInfo();
  const PPURegisterInfo &TRI = TII->getRegisterInfo();

  Info.UsesFlatScratch = MRI.isPhysRegUsed(PPU::FLAT_SCR_LO) ||
                         MRI.isPhysRegUsed(PPU::FLAT_SCR_HI);

  // Even if FLAT_SCRATCH is implicitly used, it has no effect if flat
  // instructions aren't used to access the scratch buffer. Inline assembly may
  // need it though.
  //
  // If we only have implicit uses of flat_scr on flat instructions, it is not
  // really needed.
  if (Info.UsesFlatScratch && !MFI->hasFlatScratchInit() &&
      (!hasAnyNonFlatUseOfReg(MRI, *TII, PPU::FLAT_SCR) &&
       !hasAnyNonFlatUseOfReg(MRI, *TII, PPU::FLAT_SCR_LO) &&
       !hasAnyNonFlatUseOfReg(MRI, *TII, PPU::FLAT_SCR_HI))) {
    Info.UsesFlatScratch = false;
  }

  Info.HasDynamicallySizedStack = FrameInfo.hasVarSizedObjects();
  Info.PrivateSegmentSize = FrameInfo.getStackSize();
  if (MFI->isStackRealigned())
    Info.PrivateSegmentSize += FrameInfo.getMaxAlignment();


  Info.UsesVCC = MRI.isPhysRegUsed(PPU::VCC);

  // If there are no calls, MachineRegisterInfo can tell us the used register
  // count easily.
  // A tail call isn't considered a call for MachineFrameInfo's purposes.
  if (!FrameInfo.hasCalls() && !FrameInfo.hasTailCall()) {
    MCPhysReg HighestVGPRReg = PPU::NoRegister;
    for (MCPhysReg Reg : reverse(PPU::VPR_32RegClass.getRegisters())) {
      if (MRI.isPhysRegUsed(Reg)) {
        HighestVGPRReg = Reg;
        break;
      }
    }

    MCPhysReg HighestSGPRReg = PPU::NoRegister;
    for (MCPhysReg Reg : reverse(PPU::SPR_32RegClass.getRegisters())) {
      if (MRI.isPhysRegUsed(Reg)) {
        HighestSGPRReg = Reg;
        break;
      }
    }

    // We found the maximum register index. They start at 0, so add one to get the
    // number of registers.
    Info.NumVGPR = HighestVGPRReg == PPU::NoRegister ? 0 :
      TRI.getHWRegIndex(HighestVGPRReg) + 1;
    Info.NumExplicitSGPR = HighestSGPRReg == PPU::NoRegister ? 0 :
      TRI.getHWRegIndex(HighestSGPRReg) + 1;

    return Info;
  }

  int32_t MaxVGPR = -1;
  int32_t MaxSGPR = -1;
  uint64_t CalleeFrameSize = 0;

  for (const MachineBasicBlock &MBB : MF) {
    for (const MachineInstr &MI : MBB) {
      // TODO: Check regmasks? Do they occur anywhere except calls?
      for (const MachineOperand &MO : MI.operands()) {
        unsigned Width = 0;
        bool IsSGPR = false;

        if (!MO.isReg())
          continue;

        Register Reg = MO.getReg();
        switch (Reg) {
        case PPU::TMSK:
        case PPU::SCC:
        case PPU::M0:
        case PPU::SRC_SHARED_BASE:
        case PPU::SRC_SHARED_LIMIT:
        case PPU::SRC_PRIVATE_BASE:
        case PPU::SRC_PRIVATE_LIMIT:
        case PPU::SPR_NULL:
          continue;
/*
        case PPU::SRC_POPS_EXITING_WAVE_ID:
          llvm_unreachable("src_pops_exiting_wave_id should not be used");
*/
        case PPU::NoRegister:
          assert(MI.isDebugInstr());
          continue;

        case PPU::VCC:
          Info.UsesVCC = true;
          continue;

        case PPU::FLAT_SCR:
        case PPU::FLAT_SCR_LO:
        case PPU::FLAT_SCR_HI:
          continue;
/*
        case PPU::XNACK_MASK:
        case PPU::XNACK_MASK_LO:
        case PPU::XNACK_MASK_HI:
          llvm_unreachable("xnack_mask registers should not be used");
*/
        case PPU::LDS_DIRECT:
          llvm_unreachable("lds_direct register should not be used");
/*
        case PPU::TBA:
        case PPU::TBA_LO:
        case PPU::TBA_HI:
        case PPU::TMA:
        case PPU::TMA_LO:
        case PPU::TMA_HI:
          llvm_unreachable("trap handler registers should not be used");
*/
        case PPU::SRC_VCCZ:
          llvm_unreachable("src_vccz register should not be used");

        case PPU::SRC_TMSKZ:
          llvm_unreachable("src_execz register should not be used");

        case PPU::SRC_SCC:
          llvm_unreachable("src_scc register should not be used");

        default:
          break;
        }

        if (PPU::SReg_32RegClass.contains(Reg)) {
            /* TODO
          assert(!PPU::TTMP_32RegClass.contains(Reg) &&
                 "trap handler registers should not be used");
                 */
          IsSGPR = true;
          Width = 1;
        } else if (PPU::VPR_32RegClass.contains(Reg)) {
          IsSGPR = false;
          Width = 1;
        } else if (PPU::SReg_64RegClass.contains(Reg)) {
            /* TODO
          assert(!PPU::TTMP_64RegClass.contains(Reg) &&
                 "trap handler registers should not be used");
                 */
          IsSGPR = true;
          Width = 2;
        } else if (PPU::VReg_64RegClass.contains(Reg)) {
          IsSGPR = false;
          Width = 2;
        } else if (PPU::VReg_96RegClass.contains(Reg)) {
          IsSGPR = false;
          Width = 3;
        } else if (PPU::SReg_96RegClass.contains(Reg)) {
          Width = 3;
        } else if (PPU::SReg_128RegClass.contains(Reg)) {
            /* TODO
          assert(!PPU::TTMP_128RegClass.contains(Reg) &&
            "trap handler registers should not be used");
            */
          IsSGPR = true;
          Width = 4;
        } else if (PPU::VReg_128RegClass.contains(Reg)) {
          IsSGPR = false;
          Width = 4;
          /*
        } else if (PPU::SReg_256RegClass.contains(Reg)) {
          assert(!PPU::TTMP_256RegClass.contains(Reg) &&
            "trap handler registers should not be used");
          IsSGPR = true;
          Width = 8;
        } else if (PPU::VReg_256RegClass.contains(Reg)) {
          IsSGPR = false;
          Width = 8;
        } else if (PPU::SReg_512RegClass.contains(Reg)) {
          assert(!PPU::TTMP_512RegClass.contains(Reg) &&
            "trap handler registers should not be used");
          IsSGPR = true;
          Width = 16;
        } else if (PPU::VReg_512RegClass.contains(Reg)) {
          IsSGPR = false;
          Width = 16;
        } else if (PPU::SReg_1024RegClass.contains(Reg)) {
          IsSGPR = true;
          Width = 32;
        } else if (PPU::VReg_1024RegClass.contains(Reg)) {
          IsSGPR = false;
          Width = 32;
          */
        } else {
          llvm_unreachable("Unknown register class");
        }
        unsigned HWReg = TRI.getHWRegIndex(Reg);
        int MaxUsed = HWReg + Width - 1;
        if (IsSGPR) {
          MaxSGPR = MaxUsed > MaxSGPR ? MaxUsed : MaxSGPR;
        } else {
          MaxVGPR = MaxUsed > MaxVGPR ? MaxUsed : MaxVGPR;
        }
      }

      if (MI.isCall()) {
        // Pseudo used just to encode the underlying global. Is there a better
        // way to track this?

        const MachineOperand *CalleeOp
          = TII->getNamedOperand(MI, PPU::OpName::callee);
        const Function *Callee = cast<Function>(CalleeOp->getGlobal());
        if (Callee->isDeclaration()) {
          // If this is a call to an external function, we can't do much. Make
          // conservative guesses.

          // 48 SGPRs - vcc, - flat_scr, -xnack
          int MaxSGPRGuess =
            47 - PPU::getNumExtraSGPRs(&ST, true, ST.hasFlatAddressSpace());
          MaxSGPR = std::max(MaxSGPR, MaxSGPRGuess);
          MaxVGPR = std::max(MaxVGPR, 23);

          CalleeFrameSize = std::max(CalleeFrameSize, UINT64_C(16384));
          Info.UsesVCC = true;
          Info.UsesFlatScratch = ST.hasFlatAddressSpace();
          Info.HasDynamicallySizedStack = true;
        } else {
          // We force CodeGen to run in SCC order, so the callee's register
          // usage etc. should be the cumulative usage of all callees.

          auto I = CallGraphResourceInfo.find(Callee);
          if (I == CallGraphResourceInfo.end()) {
            // Avoid crashing on undefined behavior with an illegal call to a
            // kernel. If a callsite's calling convention doesn't match the
            // function's, it's undefined behavior. If the callsite calling
            // convention does match, that would have errored earlier.
            // FIXME: The verifier shouldn't allow this.
            if (PPU::isEntryFunctionCC(Callee->getCallingConv()))
              report_fatal_error("invalid call to entry function");

            llvm_unreachable("callee should have been handled before caller");
          }

          MaxSGPR = std::max(I->second.NumExplicitSGPR - 1, MaxSGPR);
          MaxVGPR = std::max(I->second.NumVGPR - 1, MaxVGPR);
          CalleeFrameSize
            = std::max(I->second.PrivateSegmentSize, CalleeFrameSize);
          Info.UsesVCC |= I->second.UsesVCC;
          Info.UsesFlatScratch |= I->second.UsesFlatScratch;
          Info.HasDynamicallySizedStack |= I->second.HasDynamicallySizedStack;
          Info.HasRecursion |= I->second.HasRecursion;
        }

        if (!Callee->doesNotRecurse())
          Info.HasRecursion = true;
      }
    }
  }

  Info.NumExplicitSGPR = MaxSGPR + 1;
  Info.NumVGPR = MaxVGPR + 1;
  Info.PrivateSegmentSize += CalleeFrameSize;

  return Info;
}

void PPUAsmPrinter::getPPTProgramInfo(PPTProgramInfo &ProgInfo,
                                        const MachineFunction &MF) {
  PPTFunctionResourceInfo Info = analyzeResourceUsage(MF);

  ProgInfo.NumVGPR = Info.NumVGPR;
  ProgInfo.NumSGPR = Info.NumExplicitSGPR;
  ProgInfo.ScratchSize = Info.PrivateSegmentSize;
  ProgInfo.VCCUsed = Info.UsesVCC;
  ProgInfo.FlatUsed = Info.UsesFlatScratch;
  ProgInfo.DynamicCallStack = Info.HasDynamicallySizedStack || Info.HasRecursion;

  if (!isUInt<32>(ProgInfo.ScratchSize)) {
    DiagnosticInfoStackSize DiagStackSize(MF.getFunction(),
                                          ProgInfo.ScratchSize, DS_Error);
    MF.getFunction().getContext().diagnose(DiagStackSize);
  }

  const PPUSubtarget &STM = MF.getSubtarget<PPUSubtarget>();
  const PPUMachineFunctionInfo *MFI = MF.getInfo<PPUMachineFunctionInfo>();

  // TODO(scott.linder): The calculations related to SGPR/VGPR blocks are
  // duplicated in part in AMDGPUAsmParser::calculateGPRBlocks, and could be
  // unified.
  unsigned ExtraSGPRs = PPU::getNumExtraSGPRs(
      &STM, ProgInfo.VCCUsed, ProgInfo.FlatUsed);

  // Check the addressable register limit before we add ExtraSGPRs.
  /* TODO
  if (STM.getGeneration() >= AMDGPUSubtarget::VOLCANIC_ISLANDS &&
      !STM.hasSGPRInitBug()) {
    unsigned MaxAddressableNumSGPRs = STM.getAddressableNumSGPRs();
    if (ProgInfo.NumSGPR > MaxAddressableNumSGPRs) {
      // This can happen due to a compiler bug or when using inline asm.
      LLVMContext &Ctx = MF.getFunction().getContext();
      DiagnosticInfoResourceLimit Diag(MF.getFunction(),
                                       "addressable scalar registers",
                                       ProgInfo.NumSGPR, DS_Error,
                                       DK_ResourceLimit,
                                       MaxAddressableNumSGPRs);
      Ctx.diagnose(Diag);
      ProgInfo.NumSGPR = MaxAddressableNumSGPRs - 1;
    }
  }
  */

  // Account for extra SGPRs and VGPRs reserved for debugger use.
  ProgInfo.NumSGPR += ExtraSGPRs;

  // Ensure there are enough SGPRs and VGPRs for wave dispatch, where wave
  // dispatch registers are function args.
  unsigned WaveDispatchNumSGPR = 0, WaveDispatchNumVGPR = 0;
  for (auto &Arg : MF.getFunction().args()) {
    unsigned NumRegs = (Arg.getType()->getPrimitiveSizeInBits() + 31) / 32;
    if (Arg.hasAttribute(Attribute::InReg))
      WaveDispatchNumSGPR += NumRegs;
    else
      WaveDispatchNumVGPR += NumRegs;
  }
  ProgInfo.NumSGPR = std::max(ProgInfo.NumSGPR, WaveDispatchNumSGPR);
  ProgInfo.NumVGPR = std::max(ProgInfo.NumVGPR, WaveDispatchNumVGPR);

  // Adjust number of registers used to meet default/requested minimum/maximum
  // number of waves per execution unit request.
  ProgInfo.NumSGPRsForWavesPerEU = std::max(
    std::max(ProgInfo.NumSGPR, 1u), STM.getMinNumSGPRs(MFI->getMaxWavesPerEU()));
  ProgInfo.NumVGPRsForWavesPerEU = std::max(
    std::max(ProgInfo.NumVGPR, 1u), STM.getMinNumVGPRs(MFI->getMaxWavesPerEU()));

  if (MFI->getNumUserSGPRs() > STM.getMaxNumUserSGPRs()) {
    LLVMContext &Ctx = MF.getFunction().getContext();
    DiagnosticInfoResourceLimit Diag(MF.getFunction(), "user SGPRs",
                                     MFI->getNumUserSGPRs(), DS_Error);
    Ctx.diagnose(Diag);
  }

  if (MFI->getLDSSize() > static_cast<unsigned>(STM.getLocalMemorySize())) {
    LLVMContext &Ctx = MF.getFunction().getContext();
    DiagnosticInfoResourceLimit Diag(MF.getFunction(), "local memory",
                                     MFI->getLDSSize(), DS_Error);
    Ctx.diagnose(Diag);
  }

  ProgInfo.SGPRBlocks = PPU::getNumSGPRBlocks( &STM, ProgInfo.NumSGPRsForWavesPerEU);
  ProgInfo.VGPRBlocks = PPU::getNumVPRBlocks( &STM, ProgInfo.NumVGPRsForWavesPerEU);

  // Set the value to initialize FP_ROUND and FP_DENORM parts of the mode
  // register.
  ProgInfo.FloatMode = getFPMode(MF);

  const PPU::PPUModeRegisterDefaults Mode = MFI->getMode();
  ProgInfo.IEEEMode = Mode.IEEE;

  // Make clamp modifier on NaN input returns 0.
  ProgInfo.DX10Clamp = Mode.DX10Clamp;

  unsigned LDSAlignShift;
  /*
  if (STM.getGeneration() < AMDGPUSubtarget::SEA_ISLANDS) {
    // LDS is allocated in 64 dword blocks.
    LDSAlignShift = 8;
  } else {
  */
    // LDS is allocated in 128 dword blocks.
    LDSAlignShift = 9;
  //}

  unsigned LDSSpillSize =
    MFI->getLDSWaveSpillSize() * MFI->getMaxFlatWorkGroupSize();

  ProgInfo.LDSSize = MFI->getLDSSize() + LDSSpillSize;
  ProgInfo.LDSBlocks =
      alignTo(ProgInfo.LDSSize, 1ULL << LDSAlignShift) >> LDSAlignShift;

  // Scratch is allocated in 256 dword blocks.
  unsigned ScratchAlignShift = 10;
  // We need to program the hardware with the amount of scratch memory that
  // is used by the entire wave.  ProgInfo.ScratchSize is the amount of
  // scratch memory used per thread.
  ProgInfo.ScratchBlocks =
      alignTo(ProgInfo.ScratchSize * STM.getWavefrontSize(),
              1ULL << ScratchAlignShift) >>
      ScratchAlignShift;
/*
  if (getIsaVersion(getGlobalSTI()->getCPU()).Major >= 10) {
    ProgInfo.WgpMode = STM.isCuModeEnabled() ? 0 : 1;
    ProgInfo.MemOrdered = 1;
  }
  */

/* ProgInfo.ComputePGMRSrc1 =
      S_00B848_VGPRS(ProgInfo.VGPRBlocks) |
      S_00B848_SGPRS(ProgInfo.SGPRBlocks) |
      S_00B848_PRIORITY(ProgInfo.Priority) |
      S_00B848_FLOAT_MODE(ProgInfo.FloatMode) |
      S_00B848_PRIV(ProgInfo.Priv) |
      S_00B848_DX10_CLAMP(ProgInfo.DX10Clamp) |
      S_00B848_DEBUG_MODE(ProgInfo.DebugMode) |
      S_00B848_IEEE_MODE(ProgInfo.IEEEMode) |
      S_00B848_WGP_MODE(ProgInfo.WgpMode) |
      S_00B848_MEM_ORDERED(ProgInfo.MemOrdered);*/

  // 0 = X, 1 = XY, 2 = XYZ
  unsigned TIDIGCompCnt = 0;
  if (MFI->hasWorkItemIDZ())
    TIDIGCompCnt = 2;
  else if (MFI->hasWorkItemIDY())
    TIDIGCompCnt = 1;

/*  ProgInfo.ComputePGMRSrc2 =
      S_00B84C_SCRATCH_EN(ProgInfo.ScratchBlocks > 0) |
      S_00B84C_USER_SGPR(MFI->getNumUserSGPRs()) |
      // For AMDHSA, TRAP_HANDLER must be zero, as it is populated by the CP.
      S_00B84C_TRAP_HANDLER(STM.isAmdHsaOS() ? 0 : STM.isTrapHandlerEnabled()) |
      S_00B84C_TGID_X_EN(MFI->hasWorkGroupIDX()) |
      S_00B84C_TGID_Y_EN(MFI->hasWorkGroupIDY()) |
      S_00B84C_TGID_Z_EN(MFI->hasWorkGroupIDZ()) |
      S_00B84C_TG_SIZE_EN(MFI->hasWorkGroupInfo()) |
      S_00B84C_TIDIG_COMP_CNT(TIDIGCompCnt) |
      S_00B84C_EXCP_EN_MSB(0) |
      // For AMDHSA, LDS_SIZE must be zero, as it is populated by the CP.
      S_00B84C_LDS_SIZE(STM.isAmdHsaOS() ? 0 : ProgInfo.LDSBlocks) |
      S_00B84C_EXCP_EN(0);*/

  ProgInfo.Occupancy = STM.computeOccupancy(MF, ProgInfo.LDSSize,
                                            ProgInfo.NumSGPRsForWavesPerEU,
                                            ProgInfo.NumVGPRsForWavesPerEU);
}

static unsigned getRsrcReg(CallingConv::ID CallConv) {
  /*
  switch (CallConv) {
  default: LLVM_FALLTHROUGH;
  case CallingConv::AMDGPU_CS: return R_00B848_COMPUTE_PGM_RSRC1;
  }
  */
}


