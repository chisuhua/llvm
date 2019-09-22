//===-- AMDGPUInstructions.td - Common instruction defs ---*- tablegen -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains instruction defs that are common to all hw codegen
// targets.
//
//===----------------------------------------------------------------------===//

class AddressSpacesImpl {
  int Flat = 0;
  int Global = 1;
  int Region = 2;
  int Local = 3;
  int Constant = 4;
  int Private = 5;
}

def AddrSpaces : AddressSpacesImpl;


class PPUInst <dag outs, dag ins, string asm = "",
  list<dag> pattern = []> : Instruction {
  // field bit isRegisterLoad = 0;
  // field bit isRegisterStore = 0;

  let Namespace = "PPU";
  let OutOperandList = outs;
  let InOperandList = ins;
  let AsmString = asm;
  let Pattern = pattern;
  // let Itinerary = NullALU;

  // SoftFail is a field the disassembler can use to provide a way for
  // instructions to not match without killing the whole decode process. It is
  // mainly used for ARM, but Tablegen expects this field to exist or it fails
  // to build the decode table.
  field bits<32> SoftFail = 0;

  let DecoderNamespace = Namespace;

  // let TSFlags{63} = isRegisterLoad;
  // let TSFlags{62} = isRegisterStore;
}

def TruePredicate : Predicate<"">;

class PPUPredicateControl {
  list<Predicate> AssemblerPredicates = [];
//  Predicate AssemblerPredicate = TruePredicate;
//  Predicate WaveSizePredicate = TruePredicate;
  list<Predicate> OtherPredicates = [];
  list<Predicate> Predicates = !listconcat(AssemblerPredicates,
                                          OtherPredicates);
//  list<Predicate> Predicates = !listconcat([AssemblerPredicate,
//                                            WaveSizePredicate],
//                                            AssemblerPredicates,
//                                            OtherPredicates);
}

class PPUPat<dag pattern, dag result> : Pat<pattern, result>,
      PPUPredicateControl;


