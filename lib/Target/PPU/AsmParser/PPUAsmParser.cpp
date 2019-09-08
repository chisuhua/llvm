//===-- PPUAsmParser.cpp - Parse PPU assembly to MCInst instructions --===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/PPUAsmBackend.h"
#include "MCTargetDesc/PPUMCExpr.h"
#include "MCTargetDesc/PPUMCTargetDesc.h"
#include "MCTargetDesc/PPUTargetStreamer.h"
#include "TargetInfo/PPUTargetInfo.h"
#include "Utils/PPUBaseInfo.h"
#include "Utils/PPUMatInt.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/StringSwitch.h"
#include "llvm/CodeGen/Register.h"
#include "llvm/MC/MCAssembler.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCInstBuilder.h"
#include "llvm/MC/MCObjectFileInfo.h"
#include "llvm/MC/MCParser/MCAsmLexer.h"
#include "llvm/MC/MCParser/MCParsedAsmOperand.h"
#include "llvm/MC/MCParser/MCTargetAsmParser.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/Support/Casting.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/Support/TargetRegistry.h"

#include <limits>

using namespace llvm;

// Include the auto-generated portion of the compress emitter.
#define GEN_COMPRESS_INSTR
#include "PPUGenCompressInstEmitter.inc"

namespace {
struct PPUOperand;

class PPUAsmParser : public MCTargetAsmParser {
  SmallVector<FeatureBitset, 4> FeatureBitStack;

  SMLoc getLoc() const { return getParser().getTok().getLoc(); }
  bool isRV64() const { return getSTI().hasFeature(PPU::Feature64Bit); }
  bool isRV32E() const { return getSTI().hasFeature(PPU::FeatureRV32E); }

  PPUTargetStreamer &getTargetStreamer() {
    MCTargetStreamer &TS = *getParser().getStreamer().getTargetStreamer();
    return static_cast<PPUTargetStreamer &>(TS);
  }

  unsigned validateTargetOperandClass(MCParsedAsmOperand &Op,
                                      unsigned Kind) override;

  bool generateImmOutOfRangeError(OperandVector &Operands, uint64_t ErrorInfo,
                                  int64_t Lower, int64_t Upper, Twine Msg);

  bool MatchAndEmitInstruction(SMLoc IDLoc, unsigned &Opcode,
                               OperandVector &Operands, MCStreamer &Out,
                               uint64_t &ErrorInfo,
                               bool MatchingInlineAsm) override;

  bool ParseRegister(unsigned &RegNo, SMLoc &StartLoc, SMLoc &EndLoc) override;

  bool ParseInstruction(ParseInstructionInfo &Info, StringRef Name,
                        SMLoc NameLoc, OperandVector &Operands) override;

  bool ParseDirective(AsmToken DirectiveID) override;

  // Helper to actually emit an instruction to the MCStreamer. Also, when
  // possible, compression of the instruction is performed.
  void emitToStreamer(MCStreamer &S, const MCInst &Inst);

  // Helper to emit a combination of LUI, ADDI(W), and SLLI instructions that
  // synthesize the desired immedate value into the destination register.
  void emitLoadImm(Register DestReg, int64_t Value, MCStreamer &Out);

  // Helper to emit a combination of AUIPC and SecondOpcode. Used to implement
  // helpers such as emitLoadLocalAddress and emitLoadAddress.
  void emitAuipcInstPair(MCOperand DestReg, MCOperand TmpReg,
                         const MCExpr *Symbol, PPUMCExpr::VariantKind VKHi,
                         unsigned SecondOpcode, SMLoc IDLoc, MCStreamer &Out);

  // Helper to emit pseudo instruction "lla" used in PC-rel addressing.
  void emitLoadLocalAddress(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out);

  // Helper to emit pseudo instruction "la" used in GOT/PC-rel addressing.
  void emitLoadAddress(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out);

  // Helper to emit pseudo instruction "la.tls.ie" used in initial-exec TLS
  // addressing.
  void emitLoadTLSIEAddress(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out);

  // Helper to emit pseudo instruction "la.tls.gd" used in global-dynamic TLS
  // addressing.
  void emitLoadTLSGDAddress(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out);

  // Helper to emit pseudo load/store instruction with a symbol.
  void emitLoadStoreSymbol(MCInst &Inst, unsigned Opcode, SMLoc IDLoc,
                           MCStreamer &Out, bool HasTmpReg);

  // Checks that a PseudoAddTPRel is using x4/tp in its second input operand.
  // Enforcing this using a restricted register class for the second input
  // operand of PseudoAddTPRel results in a poor diagnostic due to the fact
  // 'add' is an overloaded mnemonic.
  bool checkPseudoAddTPRel(MCInst &Inst, OperandVector &Operands);

  /// Helper for processing MC instructions that have been successfully matched
  /// by MatchAndEmitInstruction. Modifications to the emitted instructions,
  /// like the expansion of pseudo instructions (e.g., "li"), can be performed
  /// in this method.
  bool processInstruction(MCInst &Inst, SMLoc IDLoc, OperandVector &Operands,
                          MCStreamer &Out);

// Auto-generated instruction matching functions
#define GET_ASSEMBLER_HEADER
#include "PPUGenAsmMatcher.inc"

  OperandMatchResultTy parseCSRSystemRegister(OperandVector &Operands);
  OperandMatchResultTy parseImmediate(OperandVector &Operands);
  OperandMatchResultTy parseRegister(OperandVector &Operands,
                                     bool AllowParens = false);
  OperandMatchResultTy parseMemOpBaseReg(OperandVector &Operands);
  OperandMatchResultTy parseAtomicMemOp(OperandVector &Operands);
  OperandMatchResultTy parseOperandWithModifier(OperandVector &Operands);
  OperandMatchResultTy parseBareSymbol(OperandVector &Operands);
  OperandMatchResultTy parseCallSymbol(OperandVector &Operands);
  OperandMatchResultTy parseJALOffset(OperandVector &Operands);

  bool parseOperand(OperandVector &Operands, StringRef Mnemonic);

  bool parseDirectiveOption();

  void setFeatureBits(uint64_t Feature, StringRef FeatureString) {
    if (!(getSTI().getFeatureBits()[Feature])) {
      MCSubtargetInfo &STI = copySTI();
      setAvailableFeatures(
          ComputeAvailableFeatures(STI.ToggleFeature(FeatureString)));
    }
  }

  void clearFeatureBits(uint64_t Feature, StringRef FeatureString) {
    if (getSTI().getFeatureBits()[Feature]) {
      MCSubtargetInfo &STI = copySTI();
      setAvailableFeatures(
          ComputeAvailableFeatures(STI.ToggleFeature(FeatureString)));
    }
  }

  void pushFeatureBits() {
    FeatureBitStack.push_back(getSTI().getFeatureBits());
  }

  bool popFeatureBits() {
    if (FeatureBitStack.empty())
      return true;

    FeatureBitset FeatureBits = FeatureBitStack.pop_back_val();
    copySTI().setFeatureBits(FeatureBits);
    setAvailableFeatures(ComputeAvailableFeatures(FeatureBits));

    return false;
  }
public:
  enum PPUMatchResultTy {
    Match_Dummy = FIRST_TARGET_MATCH_RESULT_TY,
#define GET_OPERAND_DIAGNOSTIC_TYPES
#include "PPUGenAsmMatcher.inc"
#undef GET_OPERAND_DIAGNOSTIC_TYPES
  };

  static bool classifySymbolRef(const MCExpr *Expr,
                                PPUMCExpr::VariantKind &Kind,
                                int64_t &Addend);

  PPUAsmParser(const MCSubtargetInfo &STI, MCAsmParser &Parser,
                 const MCInstrInfo &MII, const MCTargetOptions &Options)
      : MCTargetAsmParser(Options, STI, MII) {
    Parser.addAliasForDirective(".half", ".2byte");
    Parser.addAliasForDirective(".hword", ".2byte");
    Parser.addAliasForDirective(".word", ".4byte");
    Parser.addAliasForDirective(".dword", ".8byte");
    setAvailableFeatures(ComputeAvailableFeatures(STI.getFeatureBits()));
  }
};

/// PPUOperand - Instances of this class represent a parsed machine
/// instruction
struct PPUOperand : public MCParsedAsmOperand {

  enum class KindTy {
    Token,
    Register,
    Immediate,
    SystemRegister
  } Kind;

  bool IsRV64;

  struct RegOp {
    Register RegNum;
  };

  struct ImmOp {
    const MCExpr *Val;
  };

  struct SysRegOp {
    const char *Data;
    unsigned Length;
    unsigned Encoding;
    // FIXME: Add the Encoding parsed fields as needed for checks,
    // e.g.: read/write or user/supervisor/machine privileges.
  };

  SMLoc StartLoc, EndLoc;
  union {
    StringRef Tok;
    RegOp Reg;
    ImmOp Imm;
    struct SysRegOp SysReg;
  };

  PPUOperand(KindTy K) : MCParsedAsmOperand(), Kind(K) {}

public:
  PPUOperand(const PPUOperand &o) : MCParsedAsmOperand() {
    Kind = o.Kind;
    IsRV64 = o.IsRV64;
    StartLoc = o.StartLoc;
    EndLoc = o.EndLoc;
    switch (Kind) {
    case KindTy::Register:
      Reg = o.Reg;
      break;
    case KindTy::Immediate:
      Imm = o.Imm;
      break;
    case KindTy::Token:
      Tok = o.Tok;
      break;
    case KindTy::SystemRegister:
      SysReg = o.SysReg;
      break;
    }
  }

  bool isToken() const override { return Kind == KindTy::Token; }
  bool isReg() const override { return Kind == KindTy::Register; }
  bool isImm() const override { return Kind == KindTy::Immediate; }
  bool isMem() const override { return false; }
  bool isSystemRegister() const { return Kind == KindTy::SystemRegister; }

  static bool evaluateConstantImm(const MCExpr *Expr, int64_t &Imm,
                                  PPUMCExpr::VariantKind &VK) {
    if (auto *RE = dyn_cast<PPUMCExpr>(Expr)) {
      VK = RE->getKind();
      return RE->evaluateAsConstant(Imm);
    }

    if (auto CE = dyn_cast<MCConstantExpr>(Expr)) {
      VK = PPUMCExpr::VK_PPU_None;
      Imm = CE->getValue();
      return true;
    }

    return false;
  }

  // True if operand is a symbol with no modifiers, or a constant with no
  // modifiers and isShiftedInt<N-1, 1>(Op).
  template <int N> bool isBareSimmNLsb0() const {
    int64_t Imm;
    PPUMCExpr::VariantKind VK = PPUMCExpr::VK_PPU_None;
    if (!isImm())
      return false;
    bool IsConstantImm = evaluateConstantImm(getImm(), Imm, VK);
    bool IsValid;
    if (!IsConstantImm)
      IsValid = PPUAsmParser::classifySymbolRef(getImm(), VK, Imm);
    else
      IsValid = isShiftedInt<N - 1, 1>(Imm);
    return IsValid && VK == PPUMCExpr::VK_PPU_None;
  }

  // Predicate methods for AsmOperands defined in PPUInstrInfo.td

  bool isBareSymbol() const {
    int64_t Imm;
    PPUMCExpr::VariantKind VK = PPUMCExpr::VK_PPU_None;
    // Must be of 'immediate' type but not a constant.
    if (!isImm() || evaluateConstantImm(getImm(), Imm, VK))
      return false;
    return PPUAsmParser::classifySymbolRef(getImm(), VK, Imm) &&
           VK == PPUMCExpr::VK_PPU_None;
  }

  bool isCallSymbol() const {
    int64_t Imm;
    PPUMCExpr::VariantKind VK = PPUMCExpr::VK_PPU_None;
    // Must be of 'immediate' type but not a constant.
    if (!isImm() || evaluateConstantImm(getImm(), Imm, VK))
      return false;
    return PPUAsmParser::classifySymbolRef(getImm(), VK, Imm) &&
           (VK == PPUMCExpr::VK_PPU_CALL ||
            VK == PPUMCExpr::VK_PPU_CALL_PLT);
  }

  bool isTPRelAddSymbol() const {
    int64_t Imm;
    PPUMCExpr::VariantKind VK = PPUMCExpr::VK_PPU_None;
    // Must be of 'immediate' type but not a constant.
    if (!isImm() || evaluateConstantImm(getImm(), Imm, VK))
      return false;
    return PPUAsmParser::classifySymbolRef(getImm(), VK, Imm) &&
           VK == PPUMCExpr::VK_PPU_TPREL_ADD;
  }

  bool isCSRSystemRegister() const { return isSystemRegister(); }

  /// Return true if the operand is a valid for the fence instruction e.g.
  /// ('iorw').
  bool isFenceArg() const {
    if (!isImm())
      return false;
    const MCExpr *Val = getImm();
    auto *SVal = dyn_cast<MCSymbolRefExpr>(Val);
    if (!SVal || SVal->getKind() != MCSymbolRefExpr::VK_None)
      return false;

    StringRef Str = SVal->getSymbol().getName();
    // Letters must be unique, taken from 'iorw', and in ascending order. This
    // holds as long as each individual character is one of 'iorw' and is
    // greater than the previous character.
    char Prev = '\0';
    for (char c : Str) {
      if (c != 'i' && c != 'o' && c != 'r' && c != 'w')
        return false;
      if (c <= Prev)
        return false;
      Prev = c;
    }
    return true;
  }

  /// Return true if the operand is a valid floating point rounding mode.
  bool isFRMArg() const {
    if (!isImm())
      return false;
    const MCExpr *Val = getImm();
    auto *SVal = dyn_cast<MCSymbolRefExpr>(Val);
    if (!SVal || SVal->getKind() != MCSymbolRefExpr::VK_None)
      return false;

    StringRef Str = SVal->getSymbol().getName();

    return PPUFPRndMode::stringToRoundingMode(Str) != PPUFPRndMode::Invalid;
  }

  bool isImmXLenLI() const {
    int64_t Imm;
    PPUMCExpr::VariantKind VK = PPUMCExpr::VK_PPU_None;
    if (!isImm())
      return false;
    bool IsConstantImm = evaluateConstantImm(getImm(), Imm, VK);
    if (VK == PPUMCExpr::VK_PPU_LO || VK == PPUMCExpr::VK_PPU_PCREL_LO)
      return true;
    // Given only Imm, ensuring that the actually specified constant is either
    // a signed or unsigned 64-bit number is unfortunately impossible.
    bool IsInRange = isRV64() ? true : isInt<32>(Imm) || isUInt<32>(Imm);
    return IsConstantImm && IsInRange && VK == PPUMCExpr::VK_PPU_None;
  }

  // TODO schi copy from rvv
  bool isSImm3() const {
    if (!isImm())
      return false;
    PPUMCExpr::VariantKind VK;
    int64_t Imm;
    bool IsConstantImm = evaluateConstantImm(getImm(), Imm, VK);
    return IsConstantImm && isInt<3>(Imm) &&
           VK == PPUMCExpr::VK_PPU_None;
  }

  bool isSImm8() const {
    if (!isImm())
      return false;
    PPUMCExpr::VariantKind VK;
    int64_t Imm;
    bool IsConstantImm = evaluateConstantImm(getImm(), Imm, VK);
    return IsConstantImm && isInt<8>(Imm) &&
           VK == PPUMCExpr::VK_PPU_None;
  }


  bool isUImmLog2XLen() const {
    int64_t Imm;
    PPUMCExpr::VariantKind VK = PPUMCExpr::VK_PPU_None;
    if (!isImm())
      return false;
    if (!evaluateConstantImm(getImm(), Imm, VK) ||
        VK != PPUMCExpr::VK_PPU_None)
      return false;
    return (isRV64() && isUInt<6>(Imm)) || isUInt<5>(Imm);
  }

  bool isUImmLog2XLenNonZero() const {
    int64_t Imm;
    PPUMCExpr::VariantKind VK = PPUMCExpr::VK_PPU_None;
    if (!isImm())
      return false;
    if (!evaluateConstantImm(getImm(), Imm, VK) ||
        VK != PPUMCExpr::VK_PPU_None)
      return false;
    if (Imm == 0)
      return false;
    return (isRV64() && isUInt<6>(Imm)) || isUInt<5>(Imm);
  }

  bool isUImm5() const {
    int64_t Imm;
    PPUMCExpr::VariantKind VK = PPUMCExpr::VK_PPU_None;
    if (!isImm())
      return false;
    bool IsConstantImm = evaluateConstantImm(getImm(), Imm, VK);
    return IsConstantImm && isUInt<5>(Imm) && VK == PPUMCExpr::VK_PPU_None;
  }

  bool isUImm5NonZero() const {
    int64_t Imm;
    PPUMCExpr::VariantKind VK = PPUMCExpr::VK_PPU_None;
    if (!isImm())
      return false;
    bool IsConstantImm = evaluateConstantImm(getImm(), Imm, VK);
    return IsConstantImm && isUInt<5>(Imm) && (Imm != 0) &&
           VK == PPUMCExpr::VK_PPU_None;
  }

  bool isSImm6() const {
    if (!isImm())
      return false;
    PPUMCExpr::VariantKind VK = PPUMCExpr::VK_PPU_None;
    int64_t Imm;
    bool IsConstantImm = evaluateConstantImm(getImm(), Imm, VK);
    return IsConstantImm && isInt<6>(Imm) &&
           VK == PPUMCExpr::VK_PPU_None;
  }

  bool isSImm6NonZero() const {
    if (!isImm())
      return false;
    PPUMCExpr::VariantKind VK = PPUMCExpr::VK_PPU_None;
    int64_t Imm;
    bool IsConstantImm = evaluateConstantImm(getImm(), Imm, VK);
    return IsConstantImm && isInt<6>(Imm) && (Imm != 0) &&
           VK == PPUMCExpr::VK_PPU_None;
  }

  bool isCLUIImm() const {
    if (!isImm())
      return false;
    int64_t Imm;
    PPUMCExpr::VariantKind VK = PPUMCExpr::VK_PPU_None;
    bool IsConstantImm = evaluateConstantImm(getImm(), Imm, VK);
    return IsConstantImm && (Imm != 0) &&
           (isUInt<5>(Imm) || (Imm >= 0xfffe0 && Imm <= 0xfffff)) &&
           VK == PPUMCExpr::VK_PPU_None;
  }

  bool isUImm7Lsb00() const {
    if (!isImm())
      return false;
    int64_t Imm;
    PPUMCExpr::VariantKind VK = PPUMCExpr::VK_PPU_None;
    bool IsConstantImm = evaluateConstantImm(getImm(), Imm, VK);
    return IsConstantImm && isShiftedUInt<5, 2>(Imm) &&
           VK == PPUMCExpr::VK_PPU_None;
  }


  // TODO schi copy from rvv
  bool isUImm8() const {
    int64_t Imm;
    PPUMCExpr::VariantKind VK;
    if (!isImm())
      return false;
    bool IsConstantImm = evaluateConstantImm(getImm(), Imm, VK);
    return IsConstantImm && isUInt<8>(Imm) && VK == PPUMCExpr::VK_PPU_None;
  }

  bool isUImm8Lsb00() const {
    if (!isImm())
      return false;
    int64_t Imm;
    PPUMCExpr::VariantKind VK = PPUMCExpr::VK_PPU_None;
    bool IsConstantImm = evaluateConstantImm(getImm(), Imm, VK);
    return IsConstantImm && isShiftedUInt<6, 2>(Imm) &&
           VK == PPUMCExpr::VK_PPU_None;
  }

  bool isUImm8Lsb000() const {
    if (!isImm())
      return false;
    int64_t Imm;
    PPUMCExpr::VariantKind VK = PPUMCExpr::VK_PPU_None;
    bool IsConstantImm = evaluateConstantImm(getImm(), Imm, VK);
    return IsConstantImm && isShiftedUInt<5, 3>(Imm) &&
           VK == PPUMCExpr::VK_PPU_None;
  }

  bool isSImm9Lsb0() const { return isBareSimmNLsb0<9>(); }

  bool isUImm9Lsb000() const {
    if (!isImm())
      return false;
    int64_t Imm;
    PPUMCExpr::VariantKind VK = PPUMCExpr::VK_PPU_None;
    bool IsConstantImm = evaluateConstantImm(getImm(), Imm, VK);
    return IsConstantImm && isShiftedUInt<6, 3>(Imm) &&
           VK == PPUMCExpr::VK_PPU_None;
  }

  bool isUImm10Lsb00NonZero() const {
    if (!isImm())
      return false;
    int64_t Imm;
    PPUMCExpr::VariantKind VK = PPUMCExpr::VK_PPU_None;
    bool IsConstantImm = evaluateConstantImm(getImm(), Imm, VK);
    return IsConstantImm && isShiftedUInt<8, 2>(Imm) && (Imm != 0) &&
           VK == PPUMCExpr::VK_PPU_None;
  }

  bool isSImm12() const {
    PPUMCExpr::VariantKind VK = PPUMCExpr::VK_PPU_None;
    int64_t Imm;
    bool IsValid;
    if (!isImm())
      return false;
    bool IsConstantImm = evaluateConstantImm(getImm(), Imm, VK);
    if (!IsConstantImm)
      IsValid = PPUAsmParser::classifySymbolRef(getImm(), VK, Imm);
    else
      IsValid = isInt<12>(Imm);
    return IsValid && ((IsConstantImm && VK == PPUMCExpr::VK_PPU_None) ||
                       VK == PPUMCExpr::VK_PPU_LO ||
                       VK == PPUMCExpr::VK_PPU_PCREL_LO ||
                       VK == PPUMCExpr::VK_PPU_TPREL_LO);
  }

  bool isSImm12Lsb0() const { return isBareSimmNLsb0<12>(); }

  bool isSImm13Lsb0() const { return isBareSimmNLsb0<13>(); }

  bool isSImm10Lsb0000NonZero() const {
    if (!isImm())
      return false;
    int64_t Imm;
    PPUMCExpr::VariantKind VK = PPUMCExpr::VK_PPU_None;
    bool IsConstantImm = evaluateConstantImm(getImm(), Imm, VK);
    return IsConstantImm && (Imm != 0) && isShiftedInt<6, 4>(Imm) &&
           VK == PPUMCExpr::VK_PPU_None;
  }

  bool isUImm20LUI() const {
    PPUMCExpr::VariantKind VK = PPUMCExpr::VK_PPU_None;
    int64_t Imm;
    bool IsValid;
    if (!isImm())
      return false;
    bool IsConstantImm = evaluateConstantImm(getImm(), Imm, VK);
    if (!IsConstantImm) {
      IsValid = PPUAsmParser::classifySymbolRef(getImm(), VK, Imm);
      return IsValid && (VK == PPUMCExpr::VK_PPU_HI ||
                         VK == PPUMCExpr::VK_PPU_TPREL_HI);
    } else {
      return isUInt<20>(Imm) && (VK == PPUMCExpr::VK_PPU_None ||
                                 VK == PPUMCExpr::VK_PPU_HI ||
                                 VK == PPUMCExpr::VK_PPU_TPREL_HI);
    }
  }

  bool isUImm20AUIPC() const {
    PPUMCExpr::VariantKind VK = PPUMCExpr::VK_PPU_None;
    int64_t Imm;
    bool IsValid;
    if (!isImm())
      return false;
    bool IsConstantImm = evaluateConstantImm(getImm(), Imm, VK);
    if (!IsConstantImm) {
      IsValid = PPUAsmParser::classifySymbolRef(getImm(), VK, Imm);
      return IsValid && (VK == PPUMCExpr::VK_PPU_PCREL_HI ||
                         VK == PPUMCExpr::VK_PPU_GOT_HI ||
                         VK == PPUMCExpr::VK_PPU_TLS_GOT_HI ||
                         VK == PPUMCExpr::VK_PPU_TLS_GD_HI);
    } else {
      return isUInt<20>(Imm) && (VK == PPUMCExpr::VK_PPU_None ||
                                 VK == PPUMCExpr::VK_PPU_PCREL_HI ||
                                 VK == PPUMCExpr::VK_PPU_GOT_HI ||
                                 VK == PPUMCExpr::VK_PPU_TLS_GOT_HI ||
                                 VK == PPUMCExpr::VK_PPU_TLS_GD_HI);
    }
  }

  bool isSImm21Lsb0JAL() const { return isBareSimmNLsb0<21>(); }

  bool isImmZero() const {
    if (!isImm())
      return false;
    int64_t Imm;
    PPUMCExpr::VariantKind VK = PPUMCExpr::VK_PPU_None;
    bool IsConstantImm = evaluateConstantImm(getImm(), Imm, VK);
    return IsConstantImm && (Imm == 0) && VK == PPUMCExpr::VK_PPU_None;
  }

  /// getStartLoc - Gets location of the first token of this operand
  SMLoc getStartLoc() const override { return StartLoc; }
  /// getEndLoc - Gets location of the last token of this operand
  SMLoc getEndLoc() const override { return EndLoc; }
  /// True if this operand is for an RV64 instruction
  bool isRV64() const { return IsRV64; }

  unsigned getReg() const override {
    assert(Kind == KindTy::Register && "Invalid type access!");
    return Reg.RegNum.id();
  }

  StringRef getSysReg() const {
    assert(Kind == KindTy::SystemRegister && "Invalid access!");
    return StringRef(SysReg.Data, SysReg.Length);
  }

  const MCExpr *getImm() const {
    assert(Kind == KindTy::Immediate && "Invalid type access!");
    return Imm.Val;
  }

  StringRef getToken() const {
    assert(Kind == KindTy::Token && "Invalid type access!");
    return Tok;
  }

  void print(raw_ostream &OS) const override {
    switch (Kind) {
    case KindTy::Immediate:
      OS << *getImm();
      break;
    case KindTy::Register:
      OS << "<register x";
      OS << getReg() << ">";
      break;
    case KindTy::Token:
      OS << "'" << getToken() << "'";
      break;
    case KindTy::SystemRegister:
      OS << "<sysreg: " << getSysReg() << '>';
      break;
    }
  }

  static std::unique_ptr<PPUOperand> createToken(StringRef Str, SMLoc S,
                                                   bool IsRV64) {
    auto Op = std::make_unique<PPUOperand>(KindTy::Token);
    Op->Tok = Str;
    Op->StartLoc = S;
    Op->EndLoc = S;
    Op->IsRV64 = IsRV64;
    return Op;
  }

  static std::unique_ptr<PPUOperand> createReg(unsigned RegNo, SMLoc S,
                                                 SMLoc E, bool IsRV64) {
    auto Op = std::make_unique<PPUOperand>(KindTy::Register);
    Op->Reg.RegNum = RegNo;
    Op->StartLoc = S;
    Op->EndLoc = E;
    Op->IsRV64 = IsRV64;
    return Op;
  }

  static std::unique_ptr<PPUOperand> createImm(const MCExpr *Val, SMLoc S,
                                                 SMLoc E, bool IsRV64) {
    auto Op = std::make_unique<PPUOperand>(KindTy::Immediate);
    Op->Imm.Val = Val;
    Op->StartLoc = S;
    Op->EndLoc = E;
    Op->IsRV64 = IsRV64;
    return Op;
  }

  static std::unique_ptr<PPUOperand>
  createSysReg(StringRef Str, SMLoc S, unsigned Encoding, bool IsRV64) {
    auto Op = std::make_unique<PPUOperand>(KindTy::SystemRegister);
    Op->SysReg.Data = Str.data();
    Op->SysReg.Length = Str.size();
    Op->SysReg.Encoding = Encoding;
    Op->StartLoc = S;
    Op->IsRV64 = IsRV64;
    return Op;
  }

  void addExpr(MCInst &Inst, const MCExpr *Expr) const {
    assert(Expr && "Expr shouldn't be null!");
    int64_t Imm = 0;
    PPUMCExpr::VariantKind VK = PPUMCExpr::VK_PPU_None;
    bool IsConstant = evaluateConstantImm(Expr, Imm, VK);

    if (IsConstant)
      Inst.addOperand(MCOperand::createImm(Imm));
    else
      Inst.addOperand(MCOperand::createExpr(Expr));
  }

  // Used by the TableGen Code
  void addRegOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    Inst.addOperand(MCOperand::createReg(getReg()));
  }

  void addImmOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    addExpr(Inst, getImm());
  }

  void addFenceArgOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    // isFenceArg has validated the operand, meaning this cast is safe
    auto SE = cast<MCSymbolRefExpr>(getImm());

    unsigned Imm = 0;
    for (char c : SE->getSymbol().getName()) {
      switch (c) {
      default:
        llvm_unreachable("FenceArg must contain only [iorw]");
      case 'i': Imm |= PPUFenceField::I; break;
      case 'o': Imm |= PPUFenceField::O; break;
      case 'r': Imm |= PPUFenceField::R; break;
      case 'w': Imm |= PPUFenceField::W; break;
      }
    }
    Inst.addOperand(MCOperand::createImm(Imm));
  }

  void addCSRSystemRegisterOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    Inst.addOperand(MCOperand::createImm(SysReg.Encoding));
  }

  // Returns the rounding mode represented by this PPUOperand. Should only
  // be called after checking isFRMArg.
  PPUFPRndMode::RoundingMode getRoundingMode() const {
    // isFRMArg has validated the operand, meaning this cast is safe.
    auto SE = cast<MCSymbolRefExpr>(getImm());
    PPUFPRndMode::RoundingMode FRM =
        PPUFPRndMode::stringToRoundingMode(SE->getSymbol().getName());
    assert(FRM != PPUFPRndMode::Invalid && "Invalid rounding mode");
    return FRM;
  }

  void addFRMArgOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    Inst.addOperand(MCOperand::createImm(getRoundingMode()));
  }
};
} // end anonymous namespace.

#define GET_REGISTER_MATCHER
#define GET_MATCHER_IMPLEMENTATION
#include "PPUGenAsmMatcher.inc"

// Return the matching FPR64 register for the given FPR32.
// FIXME: Ideally this function could be removed in favour of using
// information from TableGen.
static Register convertFPR32ToFPR64(Register Reg) {
  switch (Reg) {
  default:
    llvm_unreachable("Not a recognised FPR32 register");
  case PPU::F0_32: return PPU::F0_64;
  case PPU::F1_32: return PPU::F1_64;
  case PPU::F2_32: return PPU::F2_64;
  case PPU::F3_32: return PPU::F3_64;
  case PPU::F4_32: return PPU::F4_64;
  case PPU::F5_32: return PPU::F5_64;
  case PPU::F6_32: return PPU::F6_64;
  case PPU::F7_32: return PPU::F7_64;
  case PPU::F8_32: return PPU::F8_64;
  case PPU::F9_32: return PPU::F9_64;
  case PPU::F10_32: return PPU::F10_64;
  case PPU::F11_32: return PPU::F11_64;
  case PPU::F12_32: return PPU::F12_64;
  case PPU::F13_32: return PPU::F13_64;
  case PPU::F14_32: return PPU::F14_64;
  case PPU::F15_32: return PPU::F15_64;
  case PPU::F16_32: return PPU::F16_64;
  case PPU::F17_32: return PPU::F17_64;
  case PPU::F18_32: return PPU::F18_64;
  case PPU::F19_32: return PPU::F19_64;
  case PPU::F20_32: return PPU::F20_64;
  case PPU::F21_32: return PPU::F21_64;
  case PPU::F22_32: return PPU::F22_64;
  case PPU::F23_32: return PPU::F23_64;
  case PPU::F24_32: return PPU::F24_64;
  case PPU::F25_32: return PPU::F25_64;
  case PPU::F26_32: return PPU::F26_64;
  case PPU::F27_32: return PPU::F27_64;
  case PPU::F28_32: return PPU::F28_64;
  case PPU::F29_32: return PPU::F29_64;
  case PPU::F30_32: return PPU::F30_64;
  case PPU::F31_32: return PPU::F31_64;
  }
}

unsigned PPUAsmParser::validateTargetOperandClass(MCParsedAsmOperand &AsmOp,
                                                    unsigned Kind) {
  PPUOperand &Op = static_cast<PPUOperand &>(AsmOp);
  if (!Op.isReg())
    return Match_InvalidOperand;

  Register Reg = Op.getReg();
  bool IsRegFPR32 =
      PPUMCRegisterClasses[PPU::FPR32RegClassID].contains(Reg);
  bool IsRegFPR32C =
      PPUMCRegisterClasses[PPU::FPR32CRegClassID].contains(Reg);

  // As the parser couldn't differentiate an FPR32 from an FPR64, coerce the
  // register from FPR32 to FPR64 or FPR32C to FPR64C if necessary.
  if ((IsRegFPR32 && Kind == MCK_FPR64) ||
      (IsRegFPR32C && Kind == MCK_FPR64C)) {
    Op.Reg.RegNum = convertFPR32ToFPR64(Reg);
    return Match_Success;
  }
  return Match_InvalidOperand;
}

bool PPUAsmParser::generateImmOutOfRangeError(
    OperandVector &Operands, uint64_t ErrorInfo, int64_t Lower, int64_t Upper,
    Twine Msg = "immediate must be an integer in the range") {
  SMLoc ErrorLoc = ((PPUOperand &)*Operands[ErrorInfo]).getStartLoc();
  return Error(ErrorLoc, Msg + " [" + Twine(Lower) + ", " + Twine(Upper) + "]");
}

bool PPUAsmParser::MatchAndEmitInstruction(SMLoc IDLoc, unsigned &Opcode,
                                             OperandVector &Operands,
                                             MCStreamer &Out,
                                             uint64_t &ErrorInfo,
                                             bool MatchingInlineAsm) {
  MCInst Inst;

  auto Result =
    MatchInstructionImpl(Operands, Inst, ErrorInfo, MatchingInlineAsm);
  switch (Result) {
  default:
    break;
  case Match_Success:
    return processInstruction(Inst, IDLoc, Operands, Out);
  case Match_MissingFeature:
    return Error(IDLoc, "instruction use requires an option to be enabled");
  case Match_MnemonicFail:
    return Error(IDLoc, "unrecognized instruction mnemonic");
  case Match_InvalidOperand: {
    SMLoc ErrorLoc = IDLoc;
    if (ErrorInfo != ~0U) {
      if (ErrorInfo >= Operands.size())
        return Error(ErrorLoc, "too few operands for instruction");

      ErrorLoc = ((PPUOperand &)*Operands[ErrorInfo]).getStartLoc();
      if (ErrorLoc == SMLoc())
        ErrorLoc = IDLoc;
    }
    return Error(ErrorLoc, "invalid operand for instruction");
  }
  }

  // Handle the case when the error message is of specific type
  // other than the generic Match_InvalidOperand, and the
  // corresponding operand is missing.
  if (Result > FIRST_TARGET_MATCH_RESULT_TY) {
    SMLoc ErrorLoc = IDLoc;
    if (ErrorInfo != ~0U && ErrorInfo >= Operands.size())
        return Error(ErrorLoc, "too few operands for instruction");
  }

  switch(Result) {
  default:
    break;
  case Match_InvalidImmXLenLI:
    if (isRV64()) {
      SMLoc ErrorLoc = ((PPUOperand &)*Operands[ErrorInfo]).getStartLoc();
      return Error(ErrorLoc, "operand must be a constant 64-bit integer");
    }
    return generateImmOutOfRangeError(Operands, ErrorInfo,
                                      std::numeric_limits<int32_t>::min(),
                                      std::numeric_limits<uint32_t>::max());
  case Match_InvalidImmZero: {
    SMLoc ErrorLoc = ((PPUOperand &)*Operands[ErrorInfo]).getStartLoc();
    return Error(ErrorLoc, "immediate must be zero");
  }
  case Match_InvalidUImmLog2XLen:
    if (isRV64())
      return generateImmOutOfRangeError(Operands, ErrorInfo, 0, (1 << 6) - 1);
    return generateImmOutOfRangeError(Operands, ErrorInfo, 0, (1 << 5) - 1);
  case Match_InvalidUImmLog2XLenNonZero:
    if (isRV64())
      return generateImmOutOfRangeError(Operands, ErrorInfo, 1, (1 << 6) - 1);
    return generateImmOutOfRangeError(Operands, ErrorInfo, 1, (1 << 5) - 1);
  case Match_InvalidUImm5:
    return generateImmOutOfRangeError(Operands, ErrorInfo, 0, (1 << 5) - 1);
  case Match_InvalidSImm6:
    return generateImmOutOfRangeError(Operands, ErrorInfo, -(1 << 5),
                                      (1 << 5) - 1);
  case Match_InvalidSImm6NonZero:
    return generateImmOutOfRangeError(
        Operands, ErrorInfo, -(1 << 5), (1 << 5) - 1,
        "immediate must be non-zero in the range");
  case Match_InvalidCLUIImm:
    return generateImmOutOfRangeError(
        Operands, ErrorInfo, 1, (1 << 5) - 1,
        "immediate must be in [0xfffe0, 0xfffff] or");
  case Match_InvalidUImm7Lsb00:
    return generateImmOutOfRangeError(
        Operands, ErrorInfo, 0, (1 << 7) - 4,
        "immediate must be a multiple of 4 bytes in the range");
  case Match_InvalidUImm8Lsb00:
    return generateImmOutOfRangeError(
        Operands, ErrorInfo, 0, (1 << 8) - 4,
        "immediate must be a multiple of 4 bytes in the range");
  case Match_InvalidUImm8Lsb000:
    return generateImmOutOfRangeError(
        Operands, ErrorInfo, 0, (1 << 8) - 8,
        "immediate must be a multiple of 8 bytes in the range");
  case Match_InvalidSImm9Lsb0:
    return generateImmOutOfRangeError(
        Operands, ErrorInfo, -(1 << 8), (1 << 8) - 2,
        "immediate must be a multiple of 2 bytes in the range");
  case Match_InvalidUImm9Lsb000:
    return generateImmOutOfRangeError(
        Operands, ErrorInfo, 0, (1 << 9) - 8,
        "immediate must be a multiple of 8 bytes in the range");
  case Match_InvalidUImm10Lsb00NonZero:
    return generateImmOutOfRangeError(
        Operands, ErrorInfo, 4, (1 << 10) - 4,
        "immediate must be a multiple of 4 bytes in the range");
  case Match_InvalidSImm10Lsb0000NonZero:
    return generateImmOutOfRangeError(
        Operands, ErrorInfo, -(1 << 9), (1 << 9) - 16,
        "immediate must be a multiple of 16 bytes and non-zero in the range");
  case Match_InvalidSImm12:
    return generateImmOutOfRangeError(
        Operands, ErrorInfo, -(1 << 11), (1 << 11) - 1,
        "operand must be a symbol with %lo/%pcrel_lo/%tprel_lo modifier or an "
        "integer in the range");
  case Match_InvalidSImm12Lsb0:
    return generateImmOutOfRangeError(
        Operands, ErrorInfo, -(1 << 11), (1 << 11) - 2,
        "immediate must be a multiple of 2 bytes in the range");
  case Match_InvalidSImm13Lsb0:
    return generateImmOutOfRangeError(
        Operands, ErrorInfo, -(1 << 12), (1 << 12) - 2,
        "immediate must be a multiple of 2 bytes in the range");
  case Match_InvalidUImm20LUI:
    return generateImmOutOfRangeError(Operands, ErrorInfo, 0, (1 << 20) - 1,
                                      "operand must be a symbol with "
                                      "%hi/%tprel_hi modifier or an integer in "
                                      "the range");
  case Match_InvalidUImm20AUIPC:
    return generateImmOutOfRangeError(
        Operands, ErrorInfo, 0, (1 << 20) - 1,
        "operand must be a symbol with a "
        "%pcrel_hi/%got_pcrel_hi/%tls_ie_pcrel_hi/%tls_gd_pcrel_hi modifier or "
        "an integer in the range");
  case Match_InvalidSImm21Lsb0JAL:
    return generateImmOutOfRangeError(
        Operands, ErrorInfo, -(1 << 20), (1 << 20) - 2,
        "immediate must be a multiple of 2 bytes in the range");
  case Match_InvalidCSRSystemRegister: {
    return generateImmOutOfRangeError(Operands, ErrorInfo, 0, (1 << 12) - 1,
                                      "operand must be a valid system register "
                                      "name or an integer in the range");
  }
  case Match_InvalidFenceArg: {
    SMLoc ErrorLoc = ((PPUOperand &)*Operands[ErrorInfo]).getStartLoc();
    return Error(
        ErrorLoc,
        "operand must be formed of letters selected in-order from 'iorw'");
  }
  case Match_InvalidFRMArg: {
    SMLoc ErrorLoc = ((PPUOperand &)*Operands[ErrorInfo]).getStartLoc();
    return Error(
        ErrorLoc,
        "operand must be a valid floating point rounding mode mnemonic");
  }
  case Match_InvalidBareSymbol: {
    SMLoc ErrorLoc = ((PPUOperand &)*Operands[ErrorInfo]).getStartLoc();
    return Error(ErrorLoc, "operand must be a bare symbol name");
  }
  case Match_InvalidCallSymbol: {
    SMLoc ErrorLoc = ((PPUOperand &)*Operands[ErrorInfo]).getStartLoc();
    return Error(ErrorLoc, "operand must be a bare symbol name");
  }
  case Match_InvalidTPRelAddSymbol: {
    SMLoc ErrorLoc = ((PPUOperand &)*Operands[ErrorInfo]).getStartLoc();
    return Error(ErrorLoc, "operand must be a symbol with %tprel_add modifier");
  }
  }

  llvm_unreachable("Unknown match type detected!");
}

// Attempts to match Name as a register (either using the default name or
// alternative ABI names), setting RegNo to the matching register. Upon
// failure, returns true and sets RegNo to 0. If IsRV32E then registers
// x16-x31 will be rejected.
static bool matchRegisterNameHelper(bool IsRV32E, Register &RegNo,
                                    StringRef Name) {
  RegNo = MatchRegisterName(Name);
  if (RegNo == 0)
    RegNo = MatchRegisterAltName(Name);
  if (IsRV32E && RegNo >= PPU::X16 && RegNo <= PPU::X31)
    RegNo = 0;
  return RegNo == 0;
}

bool PPUAsmParser::ParseRegister(unsigned &RegNo, SMLoc &StartLoc,
                                   SMLoc &EndLoc) {
  const AsmToken &Tok = getParser().getTok();
  StartLoc = Tok.getLoc();
  EndLoc = Tok.getEndLoc();
  RegNo = 0;
  StringRef Name = getLexer().getTok().getIdentifier();

  if (matchRegisterNameHelper(isRV32E(), (Register&)RegNo, Name))
    return Error(StartLoc, "invalid register name");

  getParser().Lex(); // Eat identifier token.
  return false;
}

OperandMatchResultTy PPUAsmParser::parseRegister(OperandVector &Operands,
                                                   bool AllowParens) {
  SMLoc FirstS = getLoc();
  bool HadParens = false;
  AsmToken LParen;

  // If this is an LParen and a parenthesised register name is allowed, parse it
  // atomically.
  if (AllowParens && getLexer().is(AsmToken::LParen)) {
    AsmToken Buf[2];
    size_t ReadCount = getLexer().peekTokens(Buf);
    if (ReadCount == 2 && Buf[1].getKind() == AsmToken::RParen) {
      HadParens = true;
      LParen = getParser().getTok();
      getParser().Lex(); // Eat '('
    }
  }

  switch (getLexer().getKind()) {
  default:
    if (HadParens)
      getLexer().UnLex(LParen);
    return MatchOperand_NoMatch;
  case AsmToken::Identifier:
    StringRef Name = getLexer().getTok().getIdentifier();
    Register RegNo;
    matchRegisterNameHelper(isRV32E(), RegNo, Name);

    if (RegNo == 0) {
      if (HadParens)
        getLexer().UnLex(LParen);
      return MatchOperand_NoMatch;
    }
    if (HadParens)
      Operands.push_back(PPUOperand::createToken("(", FirstS, isRV64()));
    SMLoc S = getLoc();
    SMLoc E = SMLoc::getFromPointer(S.getPointer() - 1);
    getLexer().Lex();
    Operands.push_back(PPUOperand::createReg(RegNo, S, E, isRV64()));
  }

  if (HadParens) {
    getParser().Lex(); // Eat ')'
    Operands.push_back(PPUOperand::createToken(")", getLoc(), isRV64()));
  }

  return MatchOperand_Success;
}

OperandMatchResultTy
PPUAsmParser::parseCSRSystemRegister(OperandVector &Operands) {
  SMLoc S = getLoc();
  const MCExpr *Res;

  switch (getLexer().getKind()) {
  default:
    return MatchOperand_NoMatch;
  case AsmToken::LParen:
  case AsmToken::Minus:
  case AsmToken::Plus:
  case AsmToken::Exclaim:
  case AsmToken::Tilde:
  case AsmToken::Integer:
  case AsmToken::String: {
    if (getParser().parseExpression(Res))
      return MatchOperand_ParseFail;

    auto *CE = dyn_cast<MCConstantExpr>(Res);
    if (CE) {
      int64_t Imm = CE->getValue();
      if (isUInt<12>(Imm)) {
        auto SysReg = PPUSysReg::lookupSysRegByEncoding(Imm);
        // Accept an immediate representing a named or un-named Sys Reg
        // if the range is valid, regardless of the required features.
        Operands.push_back(PPUOperand::createSysReg(
            SysReg ? SysReg->Name : "", S, Imm, isRV64()));
        return MatchOperand_Success;
      }
    }

    Twine Msg = "immediate must be an integer in the range";
    Error(S, Msg + " [" + Twine(0) + ", " + Twine((1 << 12) - 1) + "]");
    return MatchOperand_ParseFail;
  }
  case AsmToken::Identifier: {
    StringRef Identifier;
    if (getParser().parseIdentifier(Identifier))
      return MatchOperand_ParseFail;

    auto SysReg = PPUSysReg::lookupSysRegByName(Identifier);
    // Accept a named Sys Reg if the required features are present.
    if (SysReg) {
      if (!SysReg->haveRequiredFeatures(getSTI().getFeatureBits())) {
        Error(S, "system register use requires an option to be enabled");
        return MatchOperand_ParseFail;
      }
      Operands.push_back(PPUOperand::createSysReg(
          Identifier, S, SysReg->Encoding, isRV64()));
      return MatchOperand_Success;
    }

    Twine Msg = "operand must be a valid system register name "
                "or an integer in the range";
    Error(S, Msg + " [" + Twine(0) + ", " + Twine((1 << 12) - 1) + "]");
    return MatchOperand_ParseFail;
  }
  case AsmToken::Percent: {
    // Discard operand with modifier.
    Twine Msg = "immediate must be an integer in the range";
    Error(S, Msg + " [" + Twine(0) + ", " + Twine((1 << 12) - 1) + "]");
    return MatchOperand_ParseFail;
  }
  }

  return MatchOperand_NoMatch;
}

OperandMatchResultTy PPUAsmParser::parseImmediate(OperandVector &Operands) {
  SMLoc S = getLoc();
  SMLoc E = SMLoc::getFromPointer(S.getPointer() - 1);
  const MCExpr *Res;

  switch (getLexer().getKind()) {
  default:
    return MatchOperand_NoMatch;
  case AsmToken::LParen:
  case AsmToken::Dot:
  case AsmToken::Minus:
  case AsmToken::Plus:
  case AsmToken::Exclaim:
  case AsmToken::Tilde:
  case AsmToken::Integer:
  case AsmToken::String:
  case AsmToken::Identifier:
    if (getParser().parseExpression(Res))
      return MatchOperand_ParseFail;
    break;
  case AsmToken::Percent:
    return parseOperandWithModifier(Operands);
  }

  Operands.push_back(PPUOperand::createImm(Res, S, E, isRV64()));
  return MatchOperand_Success;
}

OperandMatchResultTy
PPUAsmParser::parseOperandWithModifier(OperandVector &Operands) {
  SMLoc S = getLoc();
  SMLoc E = SMLoc::getFromPointer(S.getPointer() - 1);

  if (getLexer().getKind() != AsmToken::Percent) {
    Error(getLoc(), "expected '%' for operand modifier");
    return MatchOperand_ParseFail;
  }

  getParser().Lex(); // Eat '%'

  if (getLexer().getKind() != AsmToken::Identifier) {
    Error(getLoc(), "expected valid identifier for operand modifier");
    return MatchOperand_ParseFail;
  }
  StringRef Identifier = getParser().getTok().getIdentifier();
  PPUMCExpr::VariantKind VK = PPUMCExpr::getVariantKindForName(Identifier);
  if (VK == PPUMCExpr::VK_PPU_Invalid) {
    Error(getLoc(), "unrecognized operand modifier");
    return MatchOperand_ParseFail;
  }

  getParser().Lex(); // Eat the identifier
  if (getLexer().getKind() != AsmToken::LParen) {
    Error(getLoc(), "expected '('");
    return MatchOperand_ParseFail;
  }
  getParser().Lex(); // Eat '('

  const MCExpr *SubExpr;
  if (getParser().parseParenExpression(SubExpr, E)) {
    return MatchOperand_ParseFail;
  }

  const MCExpr *ModExpr = PPUMCExpr::create(SubExpr, VK, getContext());
  Operands.push_back(PPUOperand::createImm(ModExpr, S, E, isRV64()));
  return MatchOperand_Success;
}

OperandMatchResultTy PPUAsmParser::parseBareSymbol(OperandVector &Operands) {
  SMLoc S = getLoc();
  SMLoc E = SMLoc::getFromPointer(S.getPointer() - 1);
  const MCExpr *Res;

  if (getLexer().getKind() != AsmToken::Identifier)
    return MatchOperand_NoMatch;

  StringRef Identifier;
  AsmToken Tok = getLexer().getTok();

  if (getParser().parseIdentifier(Identifier))
    return MatchOperand_ParseFail;

  if (Identifier.consume_back("@plt")) {
    Error(getLoc(), "'@plt' operand not valid for instruction");
    return MatchOperand_ParseFail;
  }

  MCSymbol *Sym = getContext().getOrCreateSymbol(Identifier);

  if (Sym->isVariable()) {
    const MCExpr *V = Sym->getVariableValue(/*SetUsed=*/false);
    if (!isa<MCSymbolRefExpr>(V)) {
      getLexer().UnLex(Tok); // Put back if it's not a bare symbol.
      return MatchOperand_NoMatch;
    }
    Res = V;
  } else
    Res = MCSymbolRefExpr::create(Sym, MCSymbolRefExpr::VK_None, getContext());

  MCBinaryExpr::Opcode Opcode;
  switch (getLexer().getKind()) {
  default:
    Operands.push_back(PPUOperand::createImm(Res, S, E, isRV64()));
    return MatchOperand_Success;
  case AsmToken::Plus:
    Opcode = MCBinaryExpr::Add;
    break;
  case AsmToken::Minus:
    Opcode = MCBinaryExpr::Sub;
    break;
  }

  const MCExpr *Expr;
  if (getParser().parseExpression(Expr))
    return MatchOperand_ParseFail;
  Res = MCBinaryExpr::create(Opcode, Res, Expr, getContext());
  Operands.push_back(PPUOperand::createImm(Res, S, E, isRV64()));
  return MatchOperand_Success;
}

OperandMatchResultTy PPUAsmParser::parseCallSymbol(OperandVector &Operands) {
  SMLoc S = getLoc();
  SMLoc E = SMLoc::getFromPointer(S.getPointer() - 1);
  const MCExpr *Res;

  if (getLexer().getKind() != AsmToken::Identifier)
    return MatchOperand_NoMatch;

  // Avoid parsing the register in `call rd, foo` as a call symbol.
  if (getLexer().peekTok().getKind() != AsmToken::EndOfStatement)
    return MatchOperand_NoMatch;

  StringRef Identifier;
  if (getParser().parseIdentifier(Identifier))
    return MatchOperand_ParseFail;

  PPUMCExpr::VariantKind Kind = PPUMCExpr::VK_PPU_CALL;
  if (Identifier.consume_back("@plt"))
    Kind = PPUMCExpr::VK_PPU_CALL_PLT;

  MCSymbol *Sym = getContext().getOrCreateSymbol(Identifier);
  Res = MCSymbolRefExpr::create(Sym, MCSymbolRefExpr::VK_None, getContext());
  Res = PPUMCExpr::create(Res, Kind, getContext());
  Operands.push_back(PPUOperand::createImm(Res, S, E, isRV64()));
  return MatchOperand_Success;
}

OperandMatchResultTy PPUAsmParser::parseJALOffset(OperandVector &Operands) {
  // Parsing jal operands is fiddly due to the `jal foo` and `jal ra, foo`
  // both being acceptable forms. When parsing `jal ra, foo` this function
  // will be called for the `ra` register operand in an attempt to match the
  // single-operand alias. parseJALOffset must fail for this case. It would
  // seem logical to try parse the operand using parseImmediate and return
  // NoMatch if the next token is a comma (meaning we must be parsing a jal in
  // the second form rather than the first). We can't do this as there's no
  // way of rewinding the lexer state. Instead, return NoMatch if this operand
  // is an identifier and is followed by a comma.
  if (getLexer().is(AsmToken::Identifier) &&
      getLexer().peekTok().is(AsmToken::Comma))
    return MatchOperand_NoMatch;

  return parseImmediate(Operands);
}

OperandMatchResultTy
PPUAsmParser::parseMemOpBaseReg(OperandVector &Operands) {
  if (getLexer().isNot(AsmToken::LParen)) {
    Error(getLoc(), "expected '('");
    return MatchOperand_ParseFail;
  }

  getParser().Lex(); // Eat '('
  Operands.push_back(PPUOperand::createToken("(", getLoc(), isRV64()));

  if (parseRegister(Operands) != MatchOperand_Success) {
    Error(getLoc(), "expected register");
    return MatchOperand_ParseFail;
  }

  if (getLexer().isNot(AsmToken::RParen)) {
    Error(getLoc(), "expected ')'");
    return MatchOperand_ParseFail;
  }

  getParser().Lex(); // Eat ')'
  Operands.push_back(PPUOperand::createToken(")", getLoc(), isRV64()));

  return MatchOperand_Success;
}

OperandMatchResultTy PPUAsmParser::parseAtomicMemOp(OperandVector &Operands) {
  // Atomic operations such as lr.w, sc.w, and amo*.w accept a "memory operand"
  // as one of their register operands, such as `(a0)`. This just denotes that
  // the register (in this case `a0`) contains a memory address.
  //
  // Normally, we would be able to parse these by putting the parens into the
  // instruction string. However, GNU as also accepts a zero-offset memory
  // operand (such as `0(a0)`), and ignores the 0. Normally this would be parsed
  // with parseImmediate followed by parseMemOpBaseReg, but these instructions
  // do not accept an immediate operand, and we do not want to add a "dummy"
  // operand that is silently dropped.
  //
  // Instead, we use this custom parser. This will: allow (and discard) an
  // offset if it is zero; require (and discard) parentheses; and add only the
  // parsed register operand to `Operands`.
  //
  // These operands are printed with PPUInstPrinter::printAtomicMemOp, which
  // will only print the register surrounded by parentheses (which GNU as also
  // uses as its canonical representation for these operands).
  std::unique_ptr<PPUOperand> OptionalImmOp;

  if (getLexer().isNot(AsmToken::LParen)) {
    // Parse an Integer token. We do not accept arbritrary constant expressions
    // in the offset field (because they may include parens, which complicates
    // parsing a lot).
    int64_t ImmVal;
    SMLoc ImmStart = getLoc();
    if (getParser().parseIntToken(ImmVal,
                                  "expected '(' or optional integer offset"))
      return MatchOperand_ParseFail;

    // Create a PPUOperand for checking later (so the error messages are
    // nicer), but we don't add it to Operands.
    SMLoc ImmEnd = getLoc();
    OptionalImmOp =
        PPUOperand::createImm(MCConstantExpr::create(ImmVal, getContext()),
                                ImmStart, ImmEnd, isRV64());
  }

  if (getLexer().isNot(AsmToken::LParen)) {
    Error(getLoc(), OptionalImmOp ? "expected '(' after optional integer offset"
                                  : "expected '(' or optional integer offset");
    return MatchOperand_ParseFail;
  }
  getParser().Lex(); // Eat '('

  if (parseRegister(Operands) != MatchOperand_Success) {
    Error(getLoc(), "expected register");
    return MatchOperand_ParseFail;
  }

  if (getLexer().isNot(AsmToken::RParen)) {
    Error(getLoc(), "expected ')'");
    return MatchOperand_ParseFail;
  }
  getParser().Lex(); // Eat ')'

  // Deferred Handling of non-zero offsets. This makes the error messages nicer.
  if (OptionalImmOp && !OptionalImmOp->isImmZero()) {
    Error(OptionalImmOp->getStartLoc(), "optional integer offset must be 0",
          SMRange(OptionalImmOp->getStartLoc(), OptionalImmOp->getEndLoc()));
    return MatchOperand_ParseFail;
  }

  return MatchOperand_Success;
}

/// Looks at a token type and creates the relevant operand from this
/// information, adding to Operands. If operand was parsed, returns false, else
/// true.
bool PPUAsmParser::parseOperand(OperandVector &Operands, StringRef Mnemonic) {
  // Check if the current operand has a custom associated parser, if so, try to
  // custom parse the operand, or fallback to the general approach.
  OperandMatchResultTy Result =
      MatchOperandParserImpl(Operands, Mnemonic, /*ParseForAllFeatures=*/true);
  if (Result == MatchOperand_Success)
    return false;
  if (Result == MatchOperand_ParseFail)
    return true;

  // Attempt to parse token as a register.
  if (parseRegister(Operands, true) == MatchOperand_Success)
    return false;

  // Attempt to parse token as an immediate
  if (parseImmediate(Operands) == MatchOperand_Success) {
    // Parse memory base register if present
    if (getLexer().is(AsmToken::LParen))
      return parseMemOpBaseReg(Operands) != MatchOperand_Success;
    return false;
  }

  // Finally we have exhausted all options and must declare defeat.
  Error(getLoc(), "unknown operand");
  return true;
}

bool PPUAsmParser::ParseInstruction(ParseInstructionInfo &Info,
                                      StringRef Name, SMLoc NameLoc,
                                      OperandVector &Operands) {
  // Ensure that if the instruction occurs when relaxation is enabled,
  // relocations are forced for the file. Ideally this would be done when there
  // is enough information to reliably determine if the instruction itself may
  // cause relaxations. Unfortunately instruction processing stage occurs in the
  // same pass as relocation emission, so it's too late to set a 'sticky bit'
  // for the entire file.
  if (getSTI().getFeatureBits()[PPU::FeatureRelax]) {
    auto *Assembler = getTargetStreamer().getStreamer().getAssemblerPtr();
    if (Assembler != nullptr) {
      PPUAsmBackend &MAB =
          static_cast<PPUAsmBackend &>(Assembler->getBackend());
      MAB.setForceRelocs();
    }
  }

  // First operand is token for instruction
  Operands.push_back(PPUOperand::createToken(Name, NameLoc, isRV64()));

  // If there are no more operands, then finish
  if (getLexer().is(AsmToken::EndOfStatement))
    return false;

  // Parse first operand
  if (parseOperand(Operands, Name))
    return true;

  // Parse until end of statement, consuming commas between operands
  unsigned OperandIdx = 1;
  while (getLexer().is(AsmToken::Comma)) {
    // Consume comma token
    getLexer().Lex();

    // Parse next operand
    if (parseOperand(Operands, Name))
      return true;

    ++OperandIdx;
  }

  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    SMLoc Loc = getLexer().getLoc();
    getParser().eatToEndOfStatement();
    return Error(Loc, "unexpected token");
  }

  getParser().Lex(); // Consume the EndOfStatement.
  return false;
}

bool PPUAsmParser::classifySymbolRef(const MCExpr *Expr,
                                       PPUMCExpr::VariantKind &Kind,
                                       int64_t &Addend) {
  Kind = PPUMCExpr::VK_PPU_None;
  Addend = 0;

  if (const PPUMCExpr *RE = dyn_cast<PPUMCExpr>(Expr)) {
    Kind = RE->getKind();
    Expr = RE->getSubExpr();
  }

  // It's a simple symbol reference or constant with no addend.
  if (isa<MCConstantExpr>(Expr) || isa<MCSymbolRefExpr>(Expr))
    return true;

  const MCBinaryExpr *BE = dyn_cast<MCBinaryExpr>(Expr);
  if (!BE)
    return false;

  if (!isa<MCSymbolRefExpr>(BE->getLHS()))
    return false;

  if (BE->getOpcode() != MCBinaryExpr::Add &&
      BE->getOpcode() != MCBinaryExpr::Sub)
    return false;

  // We are able to support the subtraction of two symbol references
  if (BE->getOpcode() == MCBinaryExpr::Sub &&
      isa<MCSymbolRefExpr>(BE->getRHS()))
    return true;

  // See if the addend is a constant, otherwise there's more going
  // on here than we can deal with.
  auto AddendExpr = dyn_cast<MCConstantExpr>(BE->getRHS());
  if (!AddendExpr)
    return false;

  Addend = AddendExpr->getValue();
  if (BE->getOpcode() == MCBinaryExpr::Sub)
    Addend = -Addend;

  // It's some symbol reference + a constant addend
  return Kind != PPUMCExpr::VK_PPU_Invalid;
}

bool PPUAsmParser::ParseDirective(AsmToken DirectiveID) {
  // This returns false if this function recognizes the directive
  // regardless of whether it is successfully handles or reports an
  // error. Otherwise it returns true to give the generic parser a
  // chance at recognizing it.
  StringRef IDVal = DirectiveID.getString();

  if (IDVal == ".option")
    return parseDirectiveOption();

  return true;
}

bool PPUAsmParser::parseDirectiveOption() {
  MCAsmParser &Parser = getParser();
  // Get the option token.
  AsmToken Tok = Parser.getTok();
  // At the moment only identifiers are supported.
  if (Tok.isNot(AsmToken::Identifier))
    return Error(Parser.getTok().getLoc(),
                 "unexpected token, expected identifier");

  StringRef Option = Tok.getIdentifier();

  if (Option == "push") {
    getTargetStreamer().emitDirectiveOptionPush();

    Parser.Lex();
    if (Parser.getTok().isNot(AsmToken::EndOfStatement))
      return Error(Parser.getTok().getLoc(),
                   "unexpected token, expected end of statement");

    pushFeatureBits();
    return false;
  }

  if (Option == "pop") {
    SMLoc StartLoc = Parser.getTok().getLoc();
    getTargetStreamer().emitDirectiveOptionPop();

    Parser.Lex();
    if (Parser.getTok().isNot(AsmToken::EndOfStatement))
      return Error(Parser.getTok().getLoc(),
                   "unexpected token, expected end of statement");

    if (popFeatureBits())
      return Error(StartLoc, ".option pop with no .option push");

    return false;
  }

  if (Option == "rvc") {
    getTargetStreamer().emitDirectiveOptionRVC();

    Parser.Lex();
    if (Parser.getTok().isNot(AsmToken::EndOfStatement))
      return Error(Parser.getTok().getLoc(),
                   "unexpected token, expected end of statement");

    setFeatureBits(PPU::FeatureStdExtC, "c");
    return false;
  }

  if (Option == "norvc") {
    getTargetStreamer().emitDirectiveOptionNoRVC();

    Parser.Lex();
    if (Parser.getTok().isNot(AsmToken::EndOfStatement))
      return Error(Parser.getTok().getLoc(),
                   "unexpected token, expected end of statement");

    clearFeatureBits(PPU::FeatureStdExtC, "c");
    return false;
  }

  if (Option == "relax") {
    getTargetStreamer().emitDirectiveOptionRelax();

    Parser.Lex();
    if (Parser.getTok().isNot(AsmToken::EndOfStatement))
      return Error(Parser.getTok().getLoc(),
                   "unexpected token, expected end of statement");

    setFeatureBits(PPU::FeatureRelax, "relax");
    return false;
  }

  if (Option == "norelax") {
    getTargetStreamer().emitDirectiveOptionNoRelax();

    Parser.Lex();
    if (Parser.getTok().isNot(AsmToken::EndOfStatement))
      return Error(Parser.getTok().getLoc(),
                   "unexpected token, expected end of statement");

    clearFeatureBits(PPU::FeatureRelax, "relax");
    return false;
  }

  // Unknown option.
  Warning(Parser.getTok().getLoc(),
          "unknown option, expected 'push', 'pop', 'rvc', 'norvc', 'relax' or "
          "'norelax'");
  Parser.eatToEndOfStatement();
  return false;
}

void PPUAsmParser::emitToStreamer(MCStreamer &S, const MCInst &Inst) {
  MCInst CInst;
  bool Res = compressInst(CInst, Inst, getSTI(), S.getContext());
  CInst.setLoc(Inst.getLoc());
  S.EmitInstruction((Res ? CInst : Inst), getSTI());
}

void PPUAsmParser::emitLoadImm(Register DestReg, int64_t Value,
                                 MCStreamer &Out) {
  PPUMatInt::InstSeq Seq;
  PPUMatInt::generateInstSeq(Value, isRV64(), Seq);

  Register SrcReg = PPU::X0;
  for (PPUMatInt::Inst &Inst : Seq) {
    if (Inst.Opc == PPU::LUI) {
      emitToStreamer(
          Out, MCInstBuilder(PPU::LUI).addReg(DestReg).addImm(Inst.Imm));
    } else {
      emitToStreamer(
          Out, MCInstBuilder(Inst.Opc).addReg(DestReg).addReg(SrcReg).addImm(
                   Inst.Imm));
    }

    // Only the first instruction has X0 as its source.
    SrcReg = DestReg;
  }
}

void PPUAsmParser::emitAuipcInstPair(MCOperand DestReg, MCOperand TmpReg,
                                       const MCExpr *Symbol,
                                       PPUMCExpr::VariantKind VKHi,
                                       unsigned SecondOpcode, SMLoc IDLoc,
                                       MCStreamer &Out) {
  // A pair of instructions for PC-relative addressing; expands to
  //   TmpLabel: AUIPC TmpReg, VKHi(symbol)
  //             OP DestReg, TmpReg, %pcrel_lo(TmpLabel)
  MCContext &Ctx = getContext();

  MCSymbol *TmpLabel = Ctx.createTempSymbol(
      "pcrel_hi", /* AlwaysAddSuffix */ true, /* CanBeUnnamed */ false);
  Out.EmitLabel(TmpLabel);

  const PPUMCExpr *SymbolHi = PPUMCExpr::create(Symbol, VKHi, Ctx);
  emitToStreamer(
      Out, MCInstBuilder(PPU::AUIPC).addOperand(TmpReg).addExpr(SymbolHi));

  const MCExpr *RefToLinkTmpLabel =
      PPUMCExpr::create(MCSymbolRefExpr::create(TmpLabel, Ctx),
                          PPUMCExpr::VK_PPU_PCREL_LO, Ctx);

  emitToStreamer(Out, MCInstBuilder(SecondOpcode)
                          .addOperand(DestReg)
                          .addOperand(TmpReg)
                          .addExpr(RefToLinkTmpLabel));
}

void PPUAsmParser::emitLoadLocalAddress(MCInst &Inst, SMLoc IDLoc,
                                          MCStreamer &Out) {
  // The load local address pseudo-instruction "lla" is used in PC-relative
  // addressing of local symbols:
  //   lla rdest, symbol
  // expands to
  //   TmpLabel: AUIPC rdest, %pcrel_hi(symbol)
  //             ADDI rdest, rdest, %pcrel_lo(TmpLabel)
  MCOperand DestReg = Inst.getOperand(0);
  const MCExpr *Symbol = Inst.getOperand(1).getExpr();
  emitAuipcInstPair(DestReg, DestReg, Symbol, PPUMCExpr::VK_PPU_PCREL_HI,
                    PPU::ADDI, IDLoc, Out);
}

void PPUAsmParser::emitLoadAddress(MCInst &Inst, SMLoc IDLoc,
                                     MCStreamer &Out) {
  // The load address pseudo-instruction "la" is used in PC-relative and
  // GOT-indirect addressing of global symbols:
  //   la rdest, symbol
  // expands to either (for non-PIC)
  //   TmpLabel: AUIPC rdest, %pcrel_hi(symbol)
  //             ADDI rdest, rdest, %pcrel_lo(TmpLabel)
  // or (for PIC)
  //   TmpLabel: AUIPC rdest, %got_pcrel_hi(symbol)
  //             Lx rdest, %pcrel_lo(TmpLabel)(rdest)
  MCOperand DestReg = Inst.getOperand(0);
  const MCExpr *Symbol = Inst.getOperand(1).getExpr();
  unsigned SecondOpcode;
  PPUMCExpr::VariantKind VKHi;
  // FIXME: Should check .option (no)pic when implemented
  if (getContext().getObjectFileInfo()->isPositionIndependent()) {
    SecondOpcode = isRV64() ? PPU::LD : PPU::LW;
    VKHi = PPUMCExpr::VK_PPU_GOT_HI;
  } else {
    SecondOpcode = PPU::ADDI;
    VKHi = PPUMCExpr::VK_PPU_PCREL_HI;
  }
  emitAuipcInstPair(DestReg, DestReg, Symbol, VKHi, SecondOpcode, IDLoc, Out);
}

void PPUAsmParser::emitLoadTLSIEAddress(MCInst &Inst, SMLoc IDLoc,
                                          MCStreamer &Out) {
  // The load TLS IE address pseudo-instruction "la.tls.ie" is used in
  // initial-exec TLS model addressing of global symbols:
  //   la.tls.ie rdest, symbol
  // expands to
  //   TmpLabel: AUIPC rdest, %tls_ie_pcrel_hi(symbol)
  //             Lx rdest, %pcrel_lo(TmpLabel)(rdest)
  MCOperand DestReg = Inst.getOperand(0);
  const MCExpr *Symbol = Inst.getOperand(1).getExpr();
  unsigned SecondOpcode = isRV64() ? PPU::LD : PPU::LW;
  emitAuipcInstPair(DestReg, DestReg, Symbol, PPUMCExpr::VK_PPU_TLS_GOT_HI,
                    SecondOpcode, IDLoc, Out);
}

void PPUAsmParser::emitLoadTLSGDAddress(MCInst &Inst, SMLoc IDLoc,
                                          MCStreamer &Out) {
  // The load TLS GD address pseudo-instruction "la.tls.gd" is used in
  // global-dynamic TLS model addressing of global symbols:
  //   la.tls.gd rdest, symbol
  // expands to
  //   TmpLabel: AUIPC rdest, %tls_gd_pcrel_hi(symbol)
  //             ADDI rdest, rdest, %pcrel_lo(TmpLabel)
  MCOperand DestReg = Inst.getOperand(0);
  const MCExpr *Symbol = Inst.getOperand(1).getExpr();
  emitAuipcInstPair(DestReg, DestReg, Symbol, PPUMCExpr::VK_PPU_TLS_GD_HI,
                    PPU::ADDI, IDLoc, Out);
}

void PPUAsmParser::emitLoadStoreSymbol(MCInst &Inst, unsigned Opcode,
                                         SMLoc IDLoc, MCStreamer &Out,
                                         bool HasTmpReg) {
  // The load/store pseudo-instruction does a pc-relative load with
  // a symbol.
  //
  // The expansion looks like this
  //
  //   TmpLabel: AUIPC tmp, %pcrel_hi(symbol)
  //             [S|L]X    rd, %pcrel_lo(TmpLabel)(tmp)
  MCOperand DestReg = Inst.getOperand(0);
  unsigned SymbolOpIdx = HasTmpReg ? 2 : 1;
  unsigned TmpRegOpIdx = HasTmpReg ? 1 : 0;
  MCOperand TmpReg = Inst.getOperand(TmpRegOpIdx);
  const MCExpr *Symbol = Inst.getOperand(SymbolOpIdx).getExpr();
  emitAuipcInstPair(DestReg, TmpReg, Symbol, PPUMCExpr::VK_PPU_PCREL_HI,
                    Opcode, IDLoc, Out);
}

bool PPUAsmParser::checkPseudoAddTPRel(MCInst &Inst,
                                         OperandVector &Operands) {
  assert(Inst.getOpcode() == PPU::PseudoAddTPRel && "Invalid instruction");
  assert(Inst.getOperand(2).isReg() && "Unexpected second operand kind");
  if (Inst.getOperand(2).getReg() != PPU::X4) {
    SMLoc ErrorLoc = ((PPUOperand &)*Operands[3]).getStartLoc();
    return Error(ErrorLoc, "the second input operand must be tp/x4 when using "
                           "%tprel_add modifier");
  }

  return false;
}

bool PPUAsmParser::processInstruction(MCInst &Inst, SMLoc IDLoc,
                                        OperandVector &Operands,
                                        MCStreamer &Out) {
  Inst.setLoc(IDLoc);

  switch (Inst.getOpcode()) {
  default:
    break;
  case PPU::PseudoLI: {
    Register Reg = Inst.getOperand(0).getReg();
    const MCOperand &Op1 = Inst.getOperand(1);
    if (Op1.isExpr()) {
      // We must have li reg, %lo(sym) or li reg, %pcrel_lo(sym) or similar.
      // Just convert to an addi. This allows compatibility with gas.
      emitToStreamer(Out, MCInstBuilder(PPU::ADDI)
                              .addReg(Reg)
                              .addReg(PPU::X0)
                              .addExpr(Op1.getExpr()));
      return false;
    }
    int64_t Imm = Inst.getOperand(1).getImm();
    // On RV32 the immediate here can either be a signed or an unsigned
    // 32-bit number. Sign extension has to be performed to ensure that Imm
    // represents the expected signed 64-bit number.
    if (!isRV64())
      Imm = SignExtend64<32>(Imm);
    emitLoadImm(Reg, Imm, Out);
    return false;
  }
  case PPU::PseudoLLA:
    emitLoadLocalAddress(Inst, IDLoc, Out);
    return false;
  case PPU::PseudoLA:
    emitLoadAddress(Inst, IDLoc, Out);
    return false;
  case PPU::PseudoLA_TLS_IE:
    emitLoadTLSIEAddress(Inst, IDLoc, Out);
    return false;
  case PPU::PseudoLA_TLS_GD:
    emitLoadTLSGDAddress(Inst, IDLoc, Out);
    return false;
  case PPU::PseudoLB:
    emitLoadStoreSymbol(Inst, PPU::LB, IDLoc, Out, /*HasTmpReg=*/false);
    return false;
  case PPU::PseudoLBU:
    emitLoadStoreSymbol(Inst, PPU::LBU, IDLoc, Out, /*HasTmpReg=*/false);
    return false;
  case PPU::PseudoLH:
    emitLoadStoreSymbol(Inst, PPU::LH, IDLoc, Out, /*HasTmpReg=*/false);
    return false;
  case PPU::PseudoLHU:
    emitLoadStoreSymbol(Inst, PPU::LHU, IDLoc, Out, /*HasTmpReg=*/false);
    return false;
  case PPU::PseudoLW:
    emitLoadStoreSymbol(Inst, PPU::LW, IDLoc, Out, /*HasTmpReg=*/false);
    return false;
  case PPU::PseudoLWU:
    emitLoadStoreSymbol(Inst, PPU::LWU, IDLoc, Out, /*HasTmpReg=*/false);
    return false;
  case PPU::PseudoLD:
    emitLoadStoreSymbol(Inst, PPU::LD, IDLoc, Out, /*HasTmpReg=*/false);
    return false;
  case PPU::PseudoFLW:
    emitLoadStoreSymbol(Inst, PPU::FLW, IDLoc, Out, /*HasTmpReg=*/true);
    return false;
  case PPU::PseudoFLD:
    emitLoadStoreSymbol(Inst, PPU::FLD, IDLoc, Out, /*HasTmpReg=*/true);
    return false;
  case PPU::PseudoSB:
    emitLoadStoreSymbol(Inst, PPU::SB, IDLoc, Out, /*HasTmpReg=*/true);
    return false;
  case PPU::PseudoSH:
    emitLoadStoreSymbol(Inst, PPU::SH, IDLoc, Out, /*HasTmpReg=*/true);
    return false;
  case PPU::PseudoSW:
    emitLoadStoreSymbol(Inst, PPU::SW, IDLoc, Out, /*HasTmpReg=*/true);
    return false;
  case PPU::PseudoSD:
    emitLoadStoreSymbol(Inst, PPU::SD, IDLoc, Out, /*HasTmpReg=*/true);
    return false;
  case PPU::PseudoFSW:
    emitLoadStoreSymbol(Inst, PPU::FSW, IDLoc, Out, /*HasTmpReg=*/true);
    return false;
  case PPU::PseudoFSD:
    emitLoadStoreSymbol(Inst, PPU::FSD, IDLoc, Out, /*HasTmpReg=*/true);
    return false;
  case PPU::PseudoAddTPRel:
    if (checkPseudoAddTPRel(Inst, Operands))
      return true;
    break;
  }

  emitToStreamer(Out, Inst);
  return false;
}

extern "C" void LLVMInitializePPUAsmParser() {
  RegisterMCAsmParser<PPUAsmParser> X(getThePPUTarget());
}
