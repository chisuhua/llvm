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

#include "llvm/IR/Argument.h"
#include "llvm/IR/Attributes.h"
#include "llvm/IR/BasicBlock.h"
#include "llvm/IR/CallSite.h"
#include "llvm/IR/Constant.h"
#include "llvm/IR/Constants.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/IRBuilder.h"
#include "llvm/IR/LLVMContext.h"
#include "llvm/IR/MDBuilder.h"
#include "llvm/IR/Module.h"
#include "llvm/Pass.h"
#include "llvm/Support/raw_ostream.h"
using namespace llvm;

#define DEBUG_TYPE "FuncWapper"

namespace {
    /* supported input param type */
    enum {
        FPT_INT8 = 0,
        FPT_INT16,
        FPT_INT32,
        FPT_INT64,
        FPT_PINT8,
        FPT_PINT16,
        FPT_PINT32,
        FPT_PINT64,
        FPT_UNSP
    };

    typedef struct func_input_param_info_ {
        uint8_t tid;
        uint8_t size;
        uint8_t offset;
        uint8_t index;
	    llvm::Type *type;
    }func_input_param_info_t;

  //Module pass to add wapper function to call real kernel.
  struct FuncWapper : public ModulePass {
    static char ID;
    FuncWapper() : ModulePass(ID) {}

    uint8_t get_FPT_id(llvm::LLVMContext &context, Type *ipt)
    {
        if (!ipt) {
            return FPT_UNSP;
        }
    
        if (ipt->isPointerTy()) {
            if (ipt == llvm::Type::getInt8PtrTy(context))  return FPT_PINT8;
            if (ipt == llvm::Type::getInt16PtrTy(context)) return FPT_PINT16;
            if (ipt == llvm::Type::getInt32PtrTy(context)) return FPT_PINT32;
            if (ipt == llvm::Type::getInt64PtrTy(context)) return FPT_PINT64;
        } else {
            if (ipt->isIntegerTy(8))  return FPT_INT8;
            if (ipt->isIntegerTy(16)) return FPT_INT16;
            if (ipt->isIntegerTy(32)) return FPT_INT32;
            if (ipt->isIntegerTy(64)) return FPT_INT64;
        }
        return FPT_UNSP;
    }

    uint8_t get_FPT_size(uint8_t fpt)
    {
        uint8_t sz_lst[] = {
            1, 2, 4, 8, /* sizeof(int8), sizeof(int16) ... */
            8, 8, 8, 8, /* sizeof(pointer) */
            0 /* sizeof(unsupported type) */
        };
        if (fpt > FPT_UNSP) return 0;

        return sz_lst[fpt];
    }  

    int get_func_input_param_list(Function &F, std::vector<func_input_param_info_t> &param_lst)
    {
        llvm::Module *module = F.getParent();
        llvm::LLVMContext &context = module->getContext();
        FunctionType *ft = F.getFunctionType();
        uint8_t fun_in_param_num = ft->getNumParams();
       
        if (fun_in_param_num < 1) {
            return -1;
        }
        param_lst.clear();

        int offset = 0;
        int param_idx;
        for (param_idx = 0; param_idx < fun_in_param_num; param_idx++) {
            func_input_param_info_t param = {0};
            param.type = ft->getParamType(param_idx);
            param.index = param_idx;
            param.tid = get_FPT_id(context, param.type);
            param.size = get_FPT_size(param.tid);
            param.offset =  offset;
            offset += param.size;
            if (param.tid == FPT_UNSP) {
                return -1;
            }
            param_lst.push_back(param);
        }
        
        return 0;
    }

    IntegerType *get_param_type_by_tid(uint8_t tid, llvm::LLVMContext &context)
    {
        switch(tid) {
        case FPT_INT8 :
        case FPT_PINT8 :
            return llvm::Type::getInt8Ty(context);
        case FPT_INT16:
        case FPT_PINT16:
            return llvm::Type::getInt16Ty(context);
        case FPT_INT32:
        case FPT_PINT32:
            return llvm::Type::getInt32Ty(context);
        case FPT_INT64:
        case FPT_PINT64:
            return llvm::Type::getInt64Ty(context);
        default:
            return llvm::Type::getInt8Ty(context);
        }
    }

    bool is_kernel_func(Function &F) {
        if (F.getCallingConv() == CallingConv::SPIR_KERNEL ||
            F.getCallingConv() == CallingConv::AMDGPU_KERNEL) {
            return true;
        }
        return false;
    }

    bool runOnFunction(Function &F) {//override {
        llvm::Module *module = F.getParent();
        llvm::LLVMContext &context = module->getContext();
        IRBuilder<> builder(context);
      
        /* create wapper function, with function name "runKernel" and the 
         * the definition of it is below:
         * void runKernel(uint8_t *kernel_addr, uint8_t *kernel_args); 
         */
        /* wapper function name */
        char rkf_name[] = "runKernel";
  
        if (!is_kernel_func(F)) {
            return false;
        } 
        /* wapper function input param type */
        llvm::Type *rkf_param_tp[] = {
            builder.getInt8Ty()->getPointerTo(),
            builder.getInt8Ty()->getPointerTo()
        };

        /* wapper functon return type */
        llvm::Type *rkf_ret_tp = builder.getVoidTy();
  
        llvm::FunctionType *rkf_def = llvm::FunctionType::get(rkf_ret_tp, rkf_param_tp, false);
        llvm::Function *rkf_fn = llvm::Function::Create(rkf_def, llvm::Function::ExternalLinkage,
                                                        rkf_name, module);
        llvm::Function::arg_iterator rkf_arg_it = rkf_fn->arg_begin();
        llvm::Value *rkf_arg0 = rkf_arg_it++;
        rkf_arg0->setName("kernel_addr");
        llvm::Value *rkf_arg1 = rkf_arg_it++;
        rkf_arg1->setName("kernel_args");
        
        llvm::BasicBlock *rkf_blk = llvm::BasicBlock::Create(context, "entry", rkf_fn);
        llvm::IRBuilder<> rkf_bld(rkf_blk);
  
        /* get kernel function input param list */
        std::vector<func_input_param_info_t> fn_in_params;
        get_func_input_param_list(F, fn_in_params);
  
        /* prepare input params before calling kernel function */
        std::vector<llvm::Value *> k_args;
        for (auto it = fn_in_params.begin(); it != fn_in_params.end(); it++) {
            if (it->type->isPointerTy()) {
    	        llvm::Value *buf_p = rkf_bld.CreateGEP(builder.getInt8Ty(), rkf_arg1, rkf_bld.getInt32(it->offset));
                llvm::Value *buf_p64 = rkf_bld.CreatePointerCast(buf_p, builder.getInt64Ty()->getPointerTo());
                llvm::Value *buf_d = rkf_bld.CreateLoad(buf_p64);
                llvm::Value *k_arg_p = rkf_bld.CreateIntToPtr(buf_d, it->type);
                k_args.push_back(k_arg_p);
            } else {
                /* 
                 * CUDA/Opencl kernel don't support reference by value, so 
                 * shouldn't get here.
                 */
                llvm::Value *buf_p = rkf_bld.CreateGEP(builder.getInt8Ty() ,rkf_arg1, rkf_bld.getInt32(it->offset));
                llvm::Value *k_arg_p = rkf_bld.CreatePointerCast(buf_p, it->type->getPointerTo());
                llvm::Value *k_arg_v = rkf_bld.CreateLoad(k_arg_p);
                k_args.push_back(k_arg_v);
            }
        }
       
        printf("try to create function call here!\n"); 
        /* call kernel function here */ 
        ArrayRef<llvm::Value *> k_arg_arr_ref(k_args);
        rkf_bld.CreateCall(&F, k_arg_arr_ref);
  
        /* end of calling kernel */
        llvm::ReturnInst::Create(context, rkf_blk);
        
        return false;
    }
    bool runOnModule(Module &M) override {
        errs() << "entry module pass : FuncWapper\n";

        for (auto &F : M) {
            runOnFunction(F);
        }
        return false;
    }
  };
}

char FuncWapper::ID = 0;
static RegisterPass<FuncWapper> X("FuncWapper", "FuncWapper");

//ReplaceInstWithInst checkSanitizerInterfaceFunction
