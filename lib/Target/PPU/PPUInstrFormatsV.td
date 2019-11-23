//===-- PPUInstrFormatsV.td - PPU V Instruction Formats --*- tablegen -*-=//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
//  This file describes the PPU V extension instruction formats.
//
//  NOTE: these formats are not frozen and not even officially named yet
//
//===----------------------------------------------------------------------===//

class RVVMaskCond<bits<2> m> {
  bits<2> Value = m;
}

def RVV_Unmasked : RVVMaskCond<0b01>;

//Vector, Vector -> Vector
class RVInstVrVr<bits<7> funct7, bits<1> funct1, RVVMaskCond m, PPUOpcode opcode,
               dag outs, dag ins, string opcodestr, string argstr>
    : RVInst<outs, ins, opcodestr, argstr, [], InstFormatR> {
  bits<5> vs2;
  bits<5> vs1;
  bits<5> vd;

  let Inst{31-25} = funct7;
  let Inst{24-20} = vs2;
  let Inst{19-15} = vs1;
  let Inst{14}    = funct1;
  let Inst{13-12} = m.Value;
  let Inst{11-7}  = vd;
  let Opcode = opcode.Value;
}

//Vector -> GPR 
class RVInstVrGpr<bits<12> funct12, bits<1> funct1, RVVMaskCond m, PPUOpcode opcode,
               dag outs, dag ins, string opcodestr, string argstr>
    : RVInst<outs, ins, opcodestr, argstr, [], InstFormatOther> {
  bits<5> vs1;
  bits<5> rd;

  let Inst{31-20} = funct12;
  let Inst{19-15} = vs1;
  let Inst{14}    = funct1;
  let Inst{13-12} = m.Value;
  let Inst{11-7}  = rd;
  let Opcode = opcode.Value;
}

//Vector, Immediate -> Vector
class RVInstVI<bits<4> funct4, bits<1> funct1, RVVMaskCond m, PPUOpcode opcode,
               dag outs, dag ins, string opcodestr, string argstr>
    : RVInst<outs, ins, opcodestr, argstr, [], InstFormatOther> {
  bits<8> simm8;
  bits<5> vs1;
  bits<5> vd;

  let Inst{31-28} = funct4;
  let Inst{27-20} = simm8;
  let Inst{19-15} = vs1;
  let Inst{14}    = funct1;
  let Inst{13-12} = m.Value;
  let Inst{11-7}  = vd;
  let Opcode = opcode.Value;
}


// TODO parametrize over rs2/vs2
class RVInstVLoad<bits<2> funct2, bits<3> funct3, RVVMaskCond m,
                  PPUOpcode opcode, dag outs, dag ins,
                  string opcodestr, string argstr>
    : RVInst<outs, ins, opcodestr, argstr, [], InstFormatOther /* TODO */> {
  bits<5> rs1;
  bits<5> vd;
  bits<3> simm3;

  let Inst{31-29} = simm3;
  let Inst{28-27} = funct2;
  let Inst{26-25} = m.Value;
  let Inst{24-20} = 0b00000; // TODO rs2 (stride) / vs2 (index vector)
  let Inst{19-15} = rs1;
  let Inst{14-12} = funct3;
  let Inst{11-7} = vd;
  let Opcode = opcode.Value;
}

// TODO parametrize over rs2/vs2
class RVInstVStore<bits<3> funct3, bits<2> funct2, RVVMaskCond m,
                   PPUOpcode opcode, dag outs, dag ins,
                   string opcodestr, string argstr>
    : RVInst<outs, ins, opcodestr, argstr, [], InstFormatOther /* TODO */> {
  bits<5> rs1;
  bits<5> vs3;
  bits<3> simm3;

  let Inst{31-27} = vs3;
  let Inst{26-25} = m.Value;
  let Inst{24-20} = 0b00000; // TODO rs2 (stride) / vs2 (index vector)
  let Inst{19-15} = rs1;
  let Inst{14-12} = funct3;
  let Inst{11-9} = simm3;
  let Inst{8-7} = m.Value;
  let Opcode = opcode.Value;
}

//Vector, Immediate -> Vector
class RVInstTSI<bits<4> funct4, bits<1> funct1, RVVMaskCond m, PPUOpcode opcode,
               dag outs, dag ins, string opcodestr, string argstr>
    : RVInst<outs, ins, opcodestr, argstr, [], InstFormatOther> {
  bits<8> simm8;
  bits<5> rs1;
  bits<5> vd;

  let Inst{31-28} = funct4;
  let Inst{27-20} = simm8;
  let Inst{19-15} = rs1;
  let Inst{14}    = funct1;
  let Inst{13-12} = m.Value;
  let Inst{11-7}  = vd;
  let Opcode = opcode.Value;
}

// TODO orginize code to better place, PPUInst

class SPseudoInst<dag outs, dag ins, list<dag> pattern = [], string opcodestr = "", string argstr = "">
  : PPUPseudo<outs, ins, pattern, opcodestr, argstr> {
  let SALU = 1;
}

class VPseudoInst<dag outs, dag ins, list<dag> pattern = [], string opcodestr = "", string argstr = "">
  : PPUPseudo<outs, ins, pattern, opcodestr, argstr> {
  let VALU = 1;
  let Uses = [TMSK];
}

class CFPseudoInst<dag outs, dag ins, list<dag> pattern = [],
  bit UseTmsk = 0, bit DefTmsk = 0> :
  SPseudoInst<outs, ins, pattern> {

  let Uses = !if(UseTmsk, [TMSK], []);
  let Defs = !if(DefTmsk, [TMSK, SCC], [SCC]);
  let mayLoad = 0;
  let mayStore = 0;
  let hasSideEffects = 0;
}

class EXPe : Enc64 {
  bits<4> en;
  bits<6> tgt;
  bits<1> compr;
  bits<1> done;
  bits<1> vm;
  bits<8> src0;
  bits<8> src1;
  bits<8> src2;
  bits<8> src3;

  let Inst{3-0} = en;
  let Inst{9-4} = tgt;
  let Inst{10} = compr;
  let Inst{11} = done;
  let Inst{12} = vm;
  let Inst{31-26} = 0x3e;
  let Inst{39-32} = src0;
  let Inst{47-40} = src1;
  let Inst{55-48} = src2;
  let Inst{63-56} = src3;
}

