//===- Hello.cpp - Example code from "Writing an LLVM Pass" ---------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements two versions of the LLVM "Hello World" pass described
// in docs/WritingAnLLVMPass.html
//
//===----------------------------------------------------------------------===//
#include <iostream>
#include "llvm/IR/Argument.h"
#include "llvm/IR/Attributes.h"
#include "llvm/IR/BasicBlock.h"
#include "llvm/IR/CallSite.h"
#include "llvm/IR/Constant.h"
#include "llvm/IR/Constants.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/InstIterator.h"
#include "llvm/IR/Instructions.h"
#include "llvm/IR/IRBuilder.h"
#include "llvm/IR/LLVMContext.h"
#include "llvm/IR/MDBuilder.h"
#include "llvm/IR/Module.h"
#include "llvm/Pass.h"
#include "llvm/Transforms/Utils/BasicBlockUtils.h"
#include "llvm/Support/raw_ostream.h"
using namespace llvm;

#define DEBUG_TYPE "FuncRep"

namespace {

/* this class defines the rules played when try to replace cuda api with new one */
class API_replace_rules {
public :
    /* api replace action */
    enum {
        API_RR_DO = 0, /* a cuda api, go on replacing it with new one */
        API_RR_IGNORE, /* the api already is a new api, no need to replace */
        API_RR_UKNOWN, /* unknown api */
    };
private :
    std::string name;
public  :
    API_replace_rules(std::string rules_name)
    {
        name = rules_name;  
    }

    virtual ~API_replace_rules()
    {
    }

    virtual int load_rules(const char *config_file_name) = 0;
    virtual int match(const char *api_name) = 0;
    virtual int get_replaceto_api_name(const char *api_name, char *replaceto_api_name) = 0;
};

/* simple repalce rules currently used:
 *   replaces all api starting with "cu" or "cuda" with "ix" or "ixrt". For example:
 *     cudaChooseDevice  --> ixrtChooseDevice
 *     cuMemoryMalloc    --> ixMemoryMalloc 
 */
class simple_API_replace_rules : public API_replace_rules {
public :
    simple_API_replace_rules() : API_replace_rules("simple API replace rules")
    {
    }

    virtual int load_rules(const char *config_file_name)
    {
        return 0;
    } 

    virtual int match(const char *api_name)
    {
        if (memcmp("cuda", api_name, 4) == 0) {
            return API_RR_DO;
        }
        if (memcmp("cu", api_name, 2) == 0) {
            return API_RR_DO;
        }

        if (memcmp("ixrt", api_name, 4) == 0 ) {
            return API_RR_IGNORE;
        }
        if (memcmp("ix", api_name, 2) == 0) {
            return API_RR_IGNORE;
        }
        return API_RR_UKNOWN;
    }
 
    virtual int get_replaceto_api_name(const char *api_name, char *replaceto_api_name)
    {
        std::string new_api_name(api_name);

        std::cout << "try to get new api: " << new_api_name << std::endl;
        if (memcmp("cuda", api_name, 4) == 0) {
            new_api_name.replace(0, 4, "ixrt");
        } else {
            new_api_name.replace(0, 2, "ix");
        }
        strcpy(replaceto_api_name, new_api_name.c_str());
        return 0;
    }
};

  /* 
   * Module pass to replace cuda api with another one. Some rules that you need to know:
   *    1. the two API must have the same number of input parameters;
   */
  struct FuncRep : public ModulePass {
    static char ID;
    API_replace_rules *rpl_rules; 

    FuncRep() : ModulePass(ID) 
    {
        /* set the api replace rules here */
        rpl_rules = new simple_API_replace_rules();    
    }

    bool replace(CallInst *CI)
    {
        Function *FC = CI->getCalledFunction();
        if (FC) {
            if (rpl_rules->match(FC->getName().begin()) != API_replace_rules::API_RR_DO ) {
                return false;
            }

            errs() << "find called function : " << FC->getName() << " argument : " << CI->getNumArgOperands() <<"\n";
            char new_api_name[200] = {0};
            rpl_rules->get_replaceto_api_name(FC->getName().begin(), new_api_name);     

            //get the new api defitions
            Module *M = CI->getModule();
            FunctionType *ft = FC->getFunctionType();
            Constant *FCache = M->getOrInsertFunction(new_api_name, ft);
          
            //get the input paramters
            CallSite CS(CI);
            SmallVector<Value *, 8> Args(CS.arg_begin(), CS.arg_end());

            //create instruction to call the new api
            IRBuilder<> Builder(CI->getParent(), CI->getIterator());
            CallInst *NewCI = Builder.CreateCall(FCache, Args);
            NewCI->setName(CI->getName());
 
            //replace the cuda api with new one
            CI->replaceAllUsesWith(NewCI);
            CI->eraseFromParent();
        }

        return true; 
    }

    bool runOnFunction(Function &F) //override 
    {
      if (rpl_rules->match(F.getName().begin()) == API_replace_rules::API_RR_IGNORE) {
          return false;
      }

      for (Function::iterator BB = F.begin(); BB != F.end(); BB++) {
          for (BasicBlock::iterator Inst = BB->begin(); Inst != BB->end();/* Inst++ */) {
              if (!isa<CallInst> (*Inst)) {
                  Inst++;
                  continue;
              }
              CallInst *CI = (CallInst *)(&*Inst);
              Inst++;
              replace(CI);
          } 
      }

      return false;
    }

    bool runOnModule(Module &M) override {
        errs() << "entry module pass\n";
        for (auto &F : M) {
            runOnFunction(F);
        }
        return false;
    }
  };
}

char FuncRep::ID = 0;
static RegisterPass<FuncRep> X("FuncRep", "FuncRep");

