//===- ReconvergeCFG.cpp --------------------------------------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// Ensure that the CFG is reconverging. A CFG is reconverging if every
// non-uniform branch terminator has exactly two successors, one of which
// post-dominates the terminator. This property is useful for generating
// single-program multiple-data control flow as required by GPUs.
//
//===----------------------------------------------------------------------===//

#include "llvm/Analysis/LegacyDivergenceAnalysis.h"
#include "llvm/Analysis/InstructionSimplify.h"
#include "llvm/Analysis/LoopInfo.h"
#include "llvm/Analysis/PostDominators.h"
#include "llvm/IR/Constants.h"
#include "llvm/IR/Dominators.h"
#include "llvm/IR/Instructions.h"
#include "llvm/Pass.h"
#include "llvm/Transforms/Scalar.h"
#include "llvm/Transforms/Utils.h"
#include "llvm/Transforms/Utils/SSAUpdater.h"
#include "llvm/ADT/PostOrderIterator.h"

#include <fstream>
#include <deque>

using namespace llvm;

#define DEBUG_TYPE "reconvergecfg"

// The name for newly created blocks.
static const char *const FlowBlockName = "flow";

namespace {

static cl::opt<bool> ForceSkipUniformBranches(
    "reconvergecfg-skip-uniform-branches", cl::Hidden,
    cl::desc("Force the ReconvergeCFG pass to skip uniform branches"),
    cl::init(false));

struct BBNode;

using BBVector = SmallVector<BasicBlock *, 8>;
using BBNodeValueVec = SmallVector<std::pair<BBNode *, Value *>, 2>;
using BBNodeMap = DenseMap<BasicBlock *, std::unique_ptr<BBNode>>;
using BBNodeList = simple_ilist<BBNode>;
using BBNodeVec = SmallVector<BBNode *, 8>;

// Node corresponding to a basic block in the ordering.
struct BBNode : public ilist_node<BBNode> {
  std::string sName;
  BasicBlock *BB;

  // Open tree links.
  BBNode *Parent = nullptr;
  simple_ilist<BBNode> Children;

  // The virtual successor that was created for this node to fix backward edges,
  // if any.
  std::unique_ptr<BBNode> Virtual;

  bool Visited = false;
  bool IsArmed = false;

  // If this is a virtual node created for the basic block.
  bool IsVirtual = false;

  // This is a flow node created for rerouting.
  bool IsFlow = false;

  bool IsUniform = false;
  unsigned NumOpenIncoming = 0;
  unsigned NumOpenOutgoing = 0;

  // For any kind of block, this records all pending incoming edges from flow
  // blocks.
  BBNodeVec FlowIncoming;

  // For unvisited flow blocks, this records all outgoing edges and their
  // conditions.
  BBNodeValueVec FlowOutgoing;

  // For flow blocks, the first closed successor and its condition.
  BBNode *FlowFirstSuccessor = nullptr;

  explicit BBNode(BasicBlock *BB) : BB(BB) { if (BB && BB->hasName())sName = BB->getName().str(); }

  // Iterate over all current predecessors / incoming edges of a node.
  class pred_iterator
      : public std::iterator<std::forward_iterator_tag, BBNode> {
    using super = std::iterator<std::forward_iterator_tag, BBNode>;

    BBNodeMap *NodeForBB;
    BBNode *Node;
    BBNodeVec::iterator FlowIt;
    llvm::pred_iterator CFGIt;
    bool AtEnd; // for virtual blocks

  public:
    using pointer = typename super::pointer;
    using reference = typename super::reference;

    pred_iterator()
        : NodeForBB(nullptr), Node(nullptr), CFGIt(nullptr), AtEnd(true) {}

    explicit pred_iterator(BBNodeMap &NodeForBB, BBNode *Node)
        : NodeForBB(&NodeForBB), Node(Node), FlowIt(Node->FlowIncoming.begin()),
          CFGIt(pred_begin(Node->BB)), AtEnd(false) {}

    explicit pred_iterator(BBNodeMap &NodeForBB, BBNode *Node, bool End)
        : NodeForBB(&NodeForBB), Node(Node), FlowIt(Node->FlowIncoming.end()),
          CFGIt(pred_end(Node->BB)), AtEnd(true) {
      assert(End);
    }
    bool operator==(const pred_iterator &x) const {
      if (Node != x.Node)
        return false;

      if (Node->IsVirtual)
        return AtEnd != x.AtEnd;

      return FlowIt == x.FlowIt && CFGIt == x.CFGIt;
    }
    bool operator!=(const pred_iterator &x) const { return !operator==(x); }

    reference operator*() const {
      if (Node->IsVirtual)
        return *(*NodeForBB)[Node->BB];
      if (FlowIt != Node->FlowIncoming.end())
        return **FlowIt;
      BBNode *PredNode = (*NodeForBB)[*CFGIt].get();
      return PredNode->Virtual ? *PredNode->Virtual : *PredNode;
    }

    pred_iterator &operator++() { // Preincrement
      if (Node->IsVirtual)
        AtEnd = true;
      else if (FlowIt != Node->FlowIncoming.end())
        ++FlowIt;
      else
        ++CFGIt;
      return *this;
    }

    pred_iterator operator++(int) { // Postincrement
      pred_iterator tmp = *this;
      ++*this;
      return tmp;
    }
  };

  iterator_range<pred_iterator> predecessors(BBNodeMap &NodeForBB) {
    return make_range(pred_iterator(NodeForBB, this),
                      pred_iterator(NodeForBB, this, true));
  }

  // Iterate over all current successors / outgoing edges of a node.
  class succ_iterator
      : public std::iterator<std::forward_iterator_tag, BBNode> {
    using super = std::iterator<std::forward_iterator_tag, BBNode>;

    BBNode *Node;
    BBNodeMap *NodeForBB;
    BBNodeValueVec::iterator FlowIt;
    llvm::succ_iterator CFGIt;
    bool AtEnd; // for blocks that have a virtual

  public:
    using pointer = typename super::pointer;
    using reference = typename super::reference;

    succ_iterator()
        : Node(nullptr), NodeForBB(nullptr), CFGIt(nullptr), AtEnd(true) {}

    explicit succ_iterator(BBNodeMap &NodeForBB, BBNode *Node)
        : Node(Node), NodeForBB(&NodeForBB), FlowIt(Node->FlowOutgoing.begin()),
          CFGIt(succ_begin(Node->BB)), AtEnd(false) {}

    succ_iterator(BBNodeMap &NodeForBB, BBNode *Node, bool End)
        : Node(Node), NodeForBB(&NodeForBB), FlowIt(Node->FlowOutgoing.end()),
          CFGIt(succ_end(Node->BB)), AtEnd(true) {
      assert(End);
    }

    bool operator==(const succ_iterator &x) const {
      if (Node != x.Node)
        return false;

      if (Node->IsFlow)
        return FlowIt == x.FlowIt;
      if (Node->Virtual)
        return AtEnd == x.AtEnd;
      return CFGIt == x.CFGIt;
    }
    bool operator!=(const succ_iterator &x) const { return !operator==(x); }

    reference operator*() const {
      assert(!AtEnd);
      if (Node->IsFlow)
        return *FlowIt->first;
      if (Node->Virtual)
        return *Node->Virtual;
      return *(*NodeForBB)[*CFGIt];
    }

    succ_iterator &operator++() { // Preincrement
      assert(!AtEnd);
      if (Node->IsFlow) {
        ++FlowIt;
      } else if (Node->Virtual) {
        AtEnd = true;
      } else {
        ++CFGIt;
      }
      return *this;
    }

    succ_iterator operator++(int) { // Postincrement
      succ_iterator tmp = *this;
      ++*this;
      return tmp;
    }
  };

  iterator_range<succ_iterator> successors(BBNodeMap &NodeForBB) {
    return make_range(succ_iterator(NodeForBB, this),
                      succ_iterator(NodeForBB, this, true));
  }
};

// Maintain a collection of nodes that arises as the union of subtrees.
//
// Use a vector to provide iteration in a deterministic order.
class BBNodeSubtrees {
public:
  BBNodeSubtrees() {}

  const BBNodeVec nodes() const
  {
      if (Nodes.empty() == false)
      {
          LLVM_DEBUG(dbgs() << "Subtree:");
          for (const auto& Node : Nodes)
          {
              LLVM_DEBUG(dbgs() << ' ' << Node->sName);
          }
          LLVM_DEBUG(dbgs() << '\n');
      }
      return Nodes;
  }
  unsigned numRoots() const { return Roots.size(); }

  void addSubtree(BBNode *Root) {
    if (Set.count(Root) != 0)
      return;

    BBNodeVec Stack;

    Roots.insert(Root);
    Stack.push_back(Root);

    while (!Stack.empty()) {
      BBNode *Node = Stack.back();
      Stack.pop_back();

      Nodes.push_back(Node);
      Set.insert(Node);

      for (BBNode &Child : Node->Children) {
        if (Set.count(&Child) == 0)
          Stack.push_back(&Child);
        else
          Roots.erase(&Child);
      }
    }
  }

private:
  BBNodeVec Nodes;
  DenseSet<BBNode *> Set;
  DenseSet<BBNode *> Roots;
};

struct PHIInfo {
  SmallVector<std::pair<BasicBlock *, Value *>, 4> OriginalIncoming;
  Value *New = nullptr;
};

class ReconvergeCFG : public FunctionPass {
  const bool SkipUniformBranches;

  bool Changed;
  LegacyDivergenceAnalysis *DA;
  LoopInfo *LI;
  DominatorTree *DT;
  PostDominatorTree *PDT;

  // Map basic blocks to (non-virtual) nodes. Holds ownership of all BBNode
  // objects.
  BBNodeMap NodeForBB;
  BBNode Root;

  DenseSet<BasicBlock *> ReroutedSuccessors;
  SmallVector<BasicBlock *, 8> ReroutedSuccessorsOrdered;

  SmallVector<Instruction *, 8> InsertedConditions;

  void computeOrderingDepthFirst(Function &F, BBVector &Ordering, const bool _bForward = true, const bool _bExitLast = false) const;
  void computeOrdering2(Function &F, BBVector &Ordering) const;

  void computeOrderingBreadthFirst(Function &F, BBVector &Ordering) const;

  void computeOrdering(Function &F, BBVector &Ordering) const;
  void computeOrderingDomRegion(Function &F, BBVector &Ordering) const;
  void computeOrderingCustom(Function& F, BBVector& Ordering,
                             ArrayRef<std::string> BBNames) const;

  void computeOrderingRPOT(Function &F, BBVector &Ordering) const;

  void buildNodes(const BBVector &BBOrdering, BBNodeList &NodeOrdering, const bool _bPreparePass = true);
  void processNode(BBNode &Node);
  void reroute(const BBNodeSubtrees &Subtrees);
  void addNode(BBNode &Node);
  void removeNode(BBNode &Node);
  void recreatePhis();
  void printTree();

public:
  static char ID;

  explicit ReconvergeCFG(bool SkipUniformBranches = true)
      : FunctionPass(ID),
        SkipUniformBranches(SkipUniformBranches || ForceSkipUniformBranches),
        Root(nullptr) {
    initializeReconvergeCFGPass(*PassRegistry::getPassRegistry());
  }

  bool runOnFunction(Function &F) override;

  StringRef getPassName() const override { return "Reconverge control flow"; }

  void getAnalysisUsage(AnalysisUsage &AU) const override {
    AU.addRequiredID(LowerSwitchID);
    if (SkipUniformBranches)
      AU.addRequired<LegacyDivergenceAnalysis>();
    AU.addRequired<LoopInfoWrapperPass>();
    AU.addRequired<DominatorTreeWrapperPass>();
    AU.addRequired<PostDominatorTreeWrapperPass>();

    FunctionPass::getAnalysisUsage(AU);
  }
};

} // anonymous namespace

char ReconvergeCFG::ID = 0;

INITIALIZE_PASS_BEGIN(ReconvergeCFG, "reconvergecfg", "Reconverge the CFG",
                      false, false)
INITIALIZE_PASS_DEPENDENCY(LegacyDivergenceAnalysis)
INITIALIZE_PASS_DEPENDENCY(LoopInfoWrapperPass)
INITIALIZE_PASS_DEPENDENCY(LowerSwitch)
INITIALIZE_PASS_DEPENDENCY(PostDominatorTreeWrapperPass)
INITIALIZE_PASS_END(ReconvergeCFG, "reconvergecfg", "Reconverge the CFG", false,
                    false)

#pragma region Fabians Helper
    void PrintSuccessors(BasicBlock *BB)
{
    unsigned int uSuccessors = BB->getTerminator()->getNumSuccessors();

    LLVM_DEBUG(dbgs() << "  " << BB->getName());

    if (uSuccessors != 0) {
        LLVM_DEBUG(dbgs() << " -> ");

        for (BasicBlock *Succ : successors(BB)) {
            LLVM_DEBUG(dbgs() << Succ->getName() << " ");
        }
    }
    LLVM_DEBUG(dbgs() << "\n");
}

void GenerateDotGraph(DenseSet<BasicBlock *> &Set, BasicBlock *BB, std::string& DotGraph, LegacyDivergenceAnalysis *DA)
{
    Set.insert(BB);

    std::string sStyle;

    if (DA != nullptr) {
        if (BranchInst *BI = dyn_cast<BranchInst>(BB->getTerminator())) {
            if (BI->isUnconditional() || (BI->isConditional() && DA->isUniform(BI->getCondition()))) {
                sStyle = " [style = dotted]";
            }
        }
    }

    for (BasicBlock *Succ : successors(BB)) {
        DotGraph += "\t" + BB->getName().str() + " -> " + Succ->getName().str() + sStyle + ";\n";
        if (Set.count(Succ) == 0) {
            GenerateDotGraph(Set, Succ, DotGraph, DA);
        }
    }
}

void GenerateDotGraph(Function &F, const std::string& _sFilePath, LegacyDivergenceAnalysis *DA)
{
    std::string sGraph;
    DenseSet<BasicBlock *> Set;

    GenerateDotGraph(Set, &F.getEntryBlock(), sGraph, DA);

    std::ofstream GraphOut(_sFilePath);
    if (GraphOut.is_open())
    {
        static const char* pPreamble = "\tnode[shape = box]\n";

        GraphOut << "digraph " << F.getName().str() << " {\n" << pPreamble << sGraph << "}\n";
        GraphOut.close();
    }
}

void printOTGraph(DenseSet<BBNode*> &Set, BBNode* Node, std::string& DotGraph)
{
    Set.insert(Node);

    std::string sStyle;

    for (BBNode &Child : Node->Children)
    {
        DotGraph += "\t" + (Node->BB == nullptr ? "root" : Node->BB->getName().str()) + " -> " + Child.BB->getName().str() + sStyle + ";\n";

        if (Set.count(&Child) == 0) {
            printOTGraph(Set, &Child, DotGraph);
        }
    }
}

void printOTGraph(BBNode* Node, const std::string& sOutPath)
{
    DenseSet<BBNode*> Set;
    std::string sGraph;

    printOTGraph(Set, Node, sGraph);

    std::ofstream GraphOut(sOutPath);

    if (GraphOut.is_open())
    {
        static const char* pPreamble = "\tnode[shape = box]\n";

        GraphOut << "digraph G {\n" << pPreamble << sGraph << "}\n";
        GraphOut.close();
    }
}

void PrintPDT(const DomTreeNodeBase<BasicBlock>* Node, std::string& PDTGraph)
{
    const std::string sName = Node->getBlock() ? Node->getBlock()->getName().str() : "exit";

    for (auto child : Node->getChildren())
    {
        PDTGraph += "\t" + sName + " -> " + child->getBlock()->getName().str() + ";\n";
        PrintPDT(child, PDTGraph);
    }
}

void PrintPDT(PostDominatorTree* PDT, const std::string& sOutPath)
{
    std::string sGraph;
    PrintPDT(PDT->getRootNode(), sGraph);

    std::ofstream GraphOut(sOutPath);

    if (GraphOut.is_open())
    {
        static const char* pPreamble = "\tnode[shape = box]\n";

        GraphOut << "digraph G {\n" << pPreamble << sGraph << "}\n";
        GraphOut.close();
    }
}

bool IsReconverging(Function& Func, PostDominatorTree* PDT, LegacyDivergenceAnalysis* DA)
{
    if (PDT == nullptr || DA == nullptr)
        return true;

    for (const BasicBlock& BB : Func)
    {
        if (const BranchInst* BI = dyn_cast<const BranchInst>(BB.getTerminator()))
        {
            if (BI->isConditional() && DA->isDivergent(BI->getCondition()))
            {
                const BasicBlock* pTrueBlock = BI->getSuccessor(0);
                const BasicBlock* pFalseBlock = BI->getSuccessor(1);

                if((!PDT->dominates(pTrueBlock, &BB) || !PDT->dominates(pTrueBlock, pFalseBlock)) &&
                    (!PDT->dominates(pFalseBlock, &BB) || !PDT->dominates(pFalseBlock, pTrueBlock)))
                {
                    LLVM_DEBUG(dbgs() << BB.getName() << " not reconverging \n");
                    return false;
                }
            }
        }
    }

    return true;
}

#pragma endregion

bool ReconvergeCFG::runOnFunction(Function &F) {
  Changed = false;
  DA = SkipUniformBranches ? &getAnalysis<LegacyDivergenceAnalysis>() : nullptr;
  LI = &getAnalysis<LoopInfoWrapperPass>().getLoopInfo();
  DT = &getAnalysis<DominatorTreeWrapperPass>().getDomTree();
  PDT = &getAnalysis<PostDominatorTreeWrapperPass>().getPostDomTree();

  const bool bInputReconverging = IsReconverging(F, PDT, DA);

  if (bInputReconverging) return Changed;

  PrintPDT(PDT, F.getName().str() + "_pdt.txt");
  
  GenerateDotGraph(F, F.getName().str() + ".dot", DA);

  BBVector BBOrdering;
  //computeOrderingDepthFirst(F, BBOrdering);
  computeOrderingRPOT(F, BBOrdering);
  //computeOrdering(F, BBOrdering);
  //computeOrderingDomRegion(F, BBOrdering);
  //computeOrderingCustom(F, BBOrdering, { "A", "Entry", "B", "D", "C" });

  assert(F.getBasicBlockList().size() == BBOrdering.size());

  BBNodeList NodeOrdering;
  buildNodes(BBOrdering, NodeOrdering);

  while (!NodeOrdering.empty()) {
    // Remove the node from the list, since the list pointers are also used for
    // the open tree.
    BBNode &Node = NodeOrdering.front();
    NodeOrdering.pop_front();
    processNode(Node);
  }

  assert(Root.Children.empty());
  for (const auto &NodeE : NodeForBB) {
    assert(NodeE.second->Visited);
    assert(!NodeE.second->NumOpenIncoming);
    assert(!NodeE.second->NumOpenOutgoing);
    assert(NodeE.second->FlowIncoming.empty());
    assert(NodeE.second->FlowOutgoing.empty());
    if (NodeE.second->Virtual) {
      assert(NodeE.second->Virtual->Visited);
      assert(!NodeE.second->Virtual->NumOpenIncoming);
      assert(!NodeE.second->Virtual->NumOpenOutgoing);
      assert(NodeE.second->Virtual->FlowIncoming.empty());
      assert(NodeE.second->Virtual->FlowOutgoing.empty());
    }
  }

  recreatePhis();

  NodeForBB.clear();

  // Cleanup redundant instructions that were inserted for flow block
  // conditions. Process instructions in the reverse order of their creation,
  // since they may depend on each other.
  while (!InsertedConditions.empty()) {
    Instruction *I = InsertedConditions.back();
    InsertedConditions.pop_back();

    if (!I->getNumUses())
      I->eraseFromParent();
  }
 
  PDT->recalculate(F);
  if (DA != nullptr)
  {
      DA->runOnFunction(F);
  }

  GenerateDotGraph(F, F.getName().str() + "_reconv.dot", DA);

  const bool bOutputReconverging = IsReconverging(F, PDT, DA);
  assert(bOutputReconverging);

  // rerouting should only occure when the input is not reconverging
  //assert(bInputReconverging ? !Changed : Changed);

  return Changed;
}

// naive depth first traversal of BBs
void AddBBNaive(DenseSet<BasicBlock *> &Set, BBVector &Ordering, BasicBlock *BB, const bool _bForward, const bool _bExitLast) {
  if (!_bExitLast || BB->getTerminator()->getNumSuccessors() > 0) // assume theres only one unique exit
  {
    PrintSuccessors(BB);
    Ordering.push_back(BB);
  }

  Set.insert(BB);
  
  if (_bForward)
  {
      for (BasicBlock *Succ : successors(BB)) {
          if (Succ != BB && Set.count(Succ) == 0) {
              AddBBNaive(Set, Ordering, Succ, _bForward, _bExitLast);
          }
      }
  }
  else
  {
      for (BasicBlock *Succ : predecessors(BB)) {
          if (Succ != BB && Set.count(Succ) == 0) {
              AddBBNaive(Set, Ordering, Succ, _bForward, _bExitLast);
          }
      }
  }
}

void ReconvergeCFG::computeOrderingDepthFirst(Function &F, BBVector &Ordering, const bool _bForward, const bool _bExitLast) const {

  LLVM_DEBUG(dbgs() << "computeOrderingDepthFirst()\n");

  DenseSet<BasicBlock *> Set;
  BasicBlock* Exit = nullptr;

  for (BasicBlock& BB : F)
  {
      if (BB.getTerminator()->getNumSuccessors() == 0)
      {
          Exit = &BB;
          break;
      }
  }

  assert(Exit);

  AddBBNaive(Set, Ordering, _bForward ? &F.getEntryBlock() : Exit, _bForward, _bExitLast);

  if (_bExitLast)
  {
      Ordering.push_back(Exit);
      PrintSuccessors(Exit);
  }
}

void ReconvergeCFG::computeOrderingRPOT(Function &F, BBVector &Ordering) const
{
   ReversePostOrderTraversal<Function*> RPOT(&F);
   for (auto I = RPOT.begin(); I != RPOT.end(); ++I) {
       Ordering.push_back(*I);
   }
}

void ReconvergeCFG::computeOrdering2(Function &F, BBVector &Ordering) const
{
    LLVM_DEBUG(dbgs() << "computeOrdering2()\n");

    DenseSet<BasicBlock*> Visited;

    BasicBlock* A = &F.getEntryBlock();
    std::deque<BasicBlock*> ToVisit = { A };

    while (ToVisit.empty() == false)
    {
        ToVisit.erase(std::remove(ToVisit.begin(), ToVisit.end(), A));
        Ordering.push_back(A);
        Visited.insert(A);

        LLVM_DEBUG(dbgs() << "Traversing " << A->getName());

        for (BasicBlock* B : llvm::successors(A))
        {
            if (Visited.count(B) == 0u)
            {
                // TODO: put them in in Dom order on the visit list
                if (std::find(ToVisit.begin(), ToVisit.end(), B) == ToVisit.end())
                {
                    ToVisit.push_back(B);
                }
            }
        }

        if (ToVisit.empty())
        {
            LLVM_DEBUG(dbgs() << "\n");
            break;
        }

        LLVM_DEBUG(dbgs() << " open:");
        for (BasicBlock* BB : ToVisit)
        {
            LLVM_DEBUG(dbgs()<< ' '<< BB->getName());
        }
        LLVM_DEBUG(dbgs() << "\n");

        BasicBlock* pNext = nullptr;

        for (BasicBlock* B : llvm::successors(A))
        {
            if (Visited.count(B) != 0u)
                continue;

            pNext = B;

            for (BasicBlock* pAncestorOfA : llvm::predecessors(A))
            {
                if (pAncestorOfA == A) // need to skip loops otherwise pB == pSucc triggers
                    continue;

                for (BasicBlock* pSucc : llvm::successors(pAncestorOfA))
                {
                    if (Visited.count(pSucc) == 0) // if pB == pSucc => unvisited
                    {
                        // pSucc is an unvisited successor of an ancestor of a

                        // Do not traverse an edge E = (A, B) if B is an unvisited successor or
                        // a post-dominator of an unvisited successor of an ancestor of A (in the traversal tree?)
                        if (B == pSucc || PDT->dominates(B, pSucc))
                        {
                            LLVM_DEBUG(dbgs() << "Rejected: " << B->getName() << '\n');
                            pNext = nullptr;
                            break;
                        }
                    }
                }

                if (pNext == nullptr)
                    break;
            }

            if (pNext != nullptr)
            {
                break;
            }
        }

        if (pNext == nullptr)
        {
            if (ToVisit.size() > 1)
            {
                // TODO: pick successor of A which is not the IPDOM
                for (BasicBlock* BB : ToVisit)
                {
                    if (BB->getTerminator() && BB->getTerminator()->getNumSuccessors() != 0)
                    {
                        pNext = BB;
                        break;
                    }
                }
            }
            else
            {
                pNext = ToVisit.front();
            }
        }

        A = pNext;
    }
}

void ReconvergeCFG::computeOrderingBreadthFirst(Function &F, BBVector &Ordering) const
{
    LLVM_DEBUG(dbgs() << "computeOrderingBreadthFirst()\n");

    DenseSet<BasicBlock*> traversed;

    struct Front
    {
        BasicBlock* pBB = nullptr;
        uint32_t uDistFromRoot = UINT32_MAX;
        bool operator<(const Front& r) { return uDistFromRoot < r.uDistFromRoot; }
    };

    std::list<Front> frontier = { {&F.getEntryBlock(), 0u} };

    const auto AncestorsTraversed = [&traversed](BasicBlock* _pBB) -> bool
    {
        for (BasicBlock* pAncestor : llvm::predecessors(_pBB))
        {
            if (pAncestor != _pBB) // ignore loops / backward edges to self
            {
                if (traversed.count(pAncestor) == 0) // not traversed yet
                    return false;
            }
        }

        return true;
    };

    const auto Traverse = [&](std::list<Front>::iterator it) -> std::list<Front>::iterator
    {
        traversed.insert(it->pBB);
        Ordering.push_back(it->pBB);
        LLVM_DEBUG(dbgs() << "\t" << it->pBB->getName() << "\n");

        const auto InFrontier = [&](auto pSuccessor) {for (const auto& f : frontier) { if (f.pBB == pSuccessor) return true; } return false; };

        for (BasicBlock* pSuccessor : llvm::successors(it->pBB))
        {
            if (traversed.count(pSuccessor) == 0u && InFrontier(pSuccessor) == false) // ignore backward eges / loops
            {
                frontier.push_back({ pSuccessor, it->uDistFromRoot + 1u });
            }
        }

        return frontier.erase(it);
    };

    while (frontier.empty() == false)
    {
        const size_t size = frontier.size();

        for (auto it = frontier.begin(); it != frontier.end();)
        {
            if (AncestorsTraversed(it->pBB))
            {
                it = Traverse(it);
            }
            else
            {
                ++it;
            }
        }

        // size didnt change, break tie
        if (frontier.size() == size)
        {
            // frontier is sorted already, take the first node that is not the sink!
            for (auto it = frontier.begin(), end = frontier.end(); it != end; ++it)
            {
                if (it->pBB->getTerminator()->getNumSuccessors() != 0)
                {
                    Traverse(it);
                    break;
                }
            }
        }
    }
}

// Compute the ordering of the basic blocks using a modified depth-first
// search.
void ReconvergeCFG::computeOrdering(Function &F, BBVector &Ordering) const {
  DenseSet<BasicBlock *> Visited;
  BBVector Stack;
  Stack.push_back(&F.getEntryBlock());

  LLVM_DEBUG(dbgs() << "computeOrdering()\n");

  while (!Stack.empty()) {
    BasicBlock *BB = Stack.back();
    Stack.pop_back();

    LLVM_DEBUG(dbgs() << "Pop " << BB->getName() << " [");    
    for (auto AncestorI = Stack.rbegin(); AncestorI != Stack.rend(); ++AncestorI) {
        LLVM_DEBUG(dbgs() << (*AncestorI)->getName() << " ");
    }
    LLVM_DEBUG(dbgs() << "]\n");

    bool Skip = false;
    for (auto AncestorI = Stack.rbegin(); AncestorI != Stack.rend();
         ++AncestorI) {
      BasicBlock *Ancestor = *AncestorI;

      // check if Ancestor from stack is an actuall ancestor and not a duplicate (we want do descent the cfg towards the unique exit)
      if (Ancestor == BB || PDT->dominates(BB, Ancestor)) {
        Skip = true;
        break;
      }
    }
    if (Skip)
      continue;

    if (Visited.count(BB))
      continue;
    Visited.insert(BB);

    Ordering.push_back(BB);
    PrintSuccessors(BB);

    //LLVM_DEBUG(dbgs() << "  " << BB->getName() << "\n");

    // If one successor is already a post-dominator (there can be only one),
    // it should come first on the stack, so that it occurs last in the
    // ordering.
    // Furthermore, if one successor is in a different loop, it should occur
    // later in the ordering as well, but no later than the post-dominator.
    Loop *L = LI->getLoopFor(BB);
    BasicBlock *PostDom = nullptr;
    BasicBlock *DifferentLoop = nullptr;
    for (BasicBlock *Succ : successors(BB)) {
      if (Succ == BB) // backward edge
        continue;

      if (PDT->dominates(Succ, BB)) {
        PostDom = Succ;
        break;
      }

      Loop *SL = LI->getLoopFor(Succ);
      if (L != SL)
        DifferentLoop = Succ;
    }

    if (PostDom) {
      LLVM_DEBUG(dbgs() << "Push PostDom " << PostDom->getName() << "\n");
      Stack.push_back(PostDom);
      DifferentLoop = nullptr;
    } else if (DifferentLoop) {
      LLVM_DEBUG(dbgs() << "Push DifLoop " << DifferentLoop->getName() << "\n");
      Stack.push_back(DifferentLoop);
    }

    for (BasicBlock *Succ : successors(BB)) {
      if (Visited.count(Succ))
        continue;
      if (Succ != PostDom && Succ != DifferentLoop)
      {
        Stack.push_back(Succ);
        LLVM_DEBUG(dbgs() << "Push Succ " << Succ->getName() << "\n");
      }
    }
  }
}

// Experimental ordering:
//  - keep dominance regions contiguous
//  - as a secondary criterion, hold off on blocks with pending predecessors
void ReconvergeCFG::computeOrderingDomRegion(Function &F, BBVector &Ordering) const {
  DenseSet<BasicBlock *> Visited;
  std::vector<std::unique_ptr<std::vector<DomTreeNode *>>> Stack;

  LLVM_DEBUG(dbgs() << "computeOrderingDomRegion()\n");

  Stack.emplace_back(std::make_unique<std::vector<DomTreeNode *>>());
  Stack.back()->push_back(DT->getRootNode());

  while (!Stack.empty()) {
    std::vector<DomTreeNode *> &Nodes = *Stack.back();

    if (Nodes.empty()) {
      Stack.pop_back();
      continue;
    }

    // TODO: Iterating over predecessors is expensive, and there's a weakly
    //       quadratic behavior hidden here.
    unsigned NonExitIdx = Nodes.size();
    bool FoundOne = false;
    for (unsigned i = 0; i < Nodes.size(); ++i) {
      DomTreeNode *Node = Nodes[i];
      BasicBlock *BB = Node->getBlock();

      LLVM_DEBUG(dbgs() << "  check: " << BB->getName() << "\n");

      if (succ_begin(BB) != succ_end(BB))
        NonExitIdx = i;

      bool HaveUnvisitedPredecessor = false;
      for (BasicBlock *Pred : predecessors(BB)) {
        if (!Visited.count(Pred)) {
          HaveUnvisitedPredecessor = true;
          break;
        }
      }

      if (HaveUnvisitedPredecessor)
        continue;

      LLVM_DEBUG(dbgs() << "    no predecessors\n");

      FoundOne = true;
      Visited.insert(BB);
      Ordering.push_back(BB);
      Stack.emplace_back(std::make_unique<std::vector<DomTreeNode *>>(Node->begin(), Node->end()));

      Nodes[i] = Nodes.back();
      Nodes.pop_back();
      i--;
      break;
    }

    if (!FoundOne) {
      assert(NonExitIdx < Nodes.size() && "exit has unreachable predecessors");
      DomTreeNode *Node = Nodes[NonExitIdx];
      Nodes[NonExitIdx] = Nodes.back();
      Nodes.pop_back();

      LLVM_DEBUG(dbgs() << "  push " << Node->getBlock()->getName() << " despite open predecessors\n");

      Visited.insert(Node->getBlock());
      Ordering.push_back(Node->getBlock());
      Stack.emplace_back(std::make_unique<std::vector<DomTreeNode *>>(Node->begin(), Node->end()));
    }
  }
}

void ReconvergeCFG::computeOrderingCustom(Function& F, BBVector& Ordering,
                                          ArrayRef<std::string> BBNames) const
{
  for (const std::string& Name : BBNames) {
    for (auto& BB : F) {
      if (BB.hasName() && BB.getName().str() == Name)
          Ordering.push_back(&BB);
    }
  }
}

// Prepare the main phase of the algorithm.
void ReconvergeCFG::buildNodes(const BBVector &BBOrdering, BBNodeList &NodeOrdering, const bool _bPreparePass) {  
  size_t nodePos = 0u;
  for (BasicBlock *BB : BBOrdering) {
    auto Node = std::make_unique<BBNode>(BB);
    //bool HasForwardOutgoing = false;
    unsigned int numBackwards = 0u;

    Node->NumOpenIncoming = std::distance(pred_begin(BB), pred_end(BB));

    for (BasicBlock *Succ : successors(BB)) {
      Node->NumOpenOutgoing++;
      
      auto succPos = std::distance(BBOrdering.begin(), std::find(BBOrdering.begin(), BBOrdering.end(), Succ));
      if (succPos <= nodePos)
          numBackwards++;
    }

    if (Node->NumOpenOutgoing <= 1) {
      Node->IsUniform = true;
    } else if (DA) {
      if (BranchInst *BI = dyn_cast<BranchInst>(BB->getTerminator())) {
        if (DA->isUniform(BI->getCondition()))
          Node->IsUniform = true;
      }
    }

    if (!Node->IsUniform && numBackwards == 2 && _bPreparePass) {
      Node->Virtual = std::make_unique<BBNode>(BB);
      Node->Virtual->IsVirtual = true;
      Node->Virtual->NumOpenIncoming = 1;
      Node->Virtual->NumOpenOutgoing = Node->NumOpenOutgoing;

      Node->IsUniform = true;
      Node->NumOpenOutgoing = 1;

      //LLVM_DEBUG(dbgs() << BB->getName() << ": virtual node @ "
      //                  << Node->Virtual.get() << "\n");
      NodeOrdering.push_front(*Node->Virtual);
    }

    NodeOrdering.push_back(*Node);

    NodeForBB[BB] = std::move(Node);
    nodePos++;
  }

  for (const auto& Node : NodeOrdering) {
      LLVM_DEBUG(dbgs() << Node.sName);
      if (Node.IsVirtual) LLVM_DEBUG(dbgs() << " Virtual");
      if (Node.IsUniform) LLVM_DEBUG(dbgs() << " Uniform");

      LLVM_DEBUG(dbgs() << "\n");
  }
}

void ReconvergeCFG::processNode(BBNode &Node) {
  LLVM_DEBUG(dbgs() << "processNode for " << Node.BB->getName()
                    << ", IsVirtual = " << Node.IsVirtual << "\n");

  if (!Node.IsVirtual) {
    // Defuse armed predecessors.
    BBNodeSubtrees Subtrees;

    for (BBNode &PredNode : Node.predecessors(NodeForBB)) {
      if (PredNode.IsArmed) {
        LLVM_DEBUG(dbgs() << "  armed predecessor: " << PredNode.BB->getName()
                          << "\n");
        Subtrees.addSubtree(&PredNode);
      }
    }

    // If S contains open outgoing edges that do not lead to B, reroute S Through a newly created basic block. FLOW
    for (BBNode *PredNode : Subtrees.nodes()) {
      for (BBNode &SuccNode : PredNode->successors(NodeForBB)) {
        if (&SuccNode != &Node && !SuccNode.Visited) { //!Visited = open edges
          reroute(Subtrees);
          goto rerouted_preds;
        }
      }
    }
  rerouted_preds:;
  }

  addNode(Node);

  // Handle outgoing backward edges.
  BBNodeSubtrees Subtrees;
  for (BBNode &SuccNode : Node.successors(NodeForBB)) {
    // since SuccNode has already been visited it must be a predecessor of Node => this is a backward edge
    if (SuccNode.Visited) {
      LLVM_DEBUG(dbgs() << "  outgoing backward edge to "
                        << SuccNode.BB->getName() << "\n");
      Subtrees.addSubtree(&SuccNode);

      Node.NumOpenOutgoing--;
      if (!Node.NumOpenIncoming && !Node.NumOpenOutgoing)
        removeNode(Node);

      SuccNode.NumOpenIncoming--;
      if (!SuccNode.NumOpenIncoming && !SuccNode.NumOpenOutgoing)
        removeNode(SuccNode);
      //else
      //  Subtrees.addSubtree(&SuccNode); // DONT add closed nodes to subtree roots 
    }
  }

  if (Subtrees.numRoots() > 1) {
    reroute(Subtrees); // does not work if non of the roots as any open outgoing edges for the new flow node
  } else {
    BBNode *FirstOpenSubtreeSucc = nullptr;
    for (BBNode *N : Subtrees.nodes()) {
      for (BBNode &SuccNode : N->successors(NodeForBB)) {
        if (!SuccNode.Visited) {
          if (!FirstOpenSubtreeSucc) {
            FirstOpenSubtreeSucc = &SuccNode;
          } else if (&SuccNode != FirstOpenSubtreeSucc) {
            reroute(Subtrees);
            goto rerouted_backward;
          }
        }
      }
    }
  rerouted_backward:;
  }
}

namespace {

struct Successors {
  // Keep a vector of successor nodes for deterministic ordering
  BBNodeVec SuccessorsVec;

  // Conditions[To][From] is the condition under which we should proceed to
  // To when coming from From.
  DenseMap<BBNode *, DenseMap<BBNode *, Value *>> Conditions;

  void add(BBNode *From, BBNode *To, Value *Condition) {
    if (!Conditions.count(To))
      SuccessorsVec.push_back(To);
    Conditions[To][From] = Condition;
  }
};

} // anonymous namespace

void ReconvergeCFG::reroute(const BBNodeSubtrees &Subtrees) {
  Function *F = Subtrees.nodes().back()->BB->getParent();
  BasicBlock *Insert = Subtrees.nodes().back()->BB->getNextNode();
  BasicBlock *Flow =
      BasicBlock::Create(F->getContext(), FlowBlockName, F, Insert);
  auto Node = std::make_unique<BBNode>(Flow);
  Successors S;
  BBNodeVec Predecessors;

  LLVM_DEBUG(dbgs() << "  rerouting, flow node @ " << Node.get() << "\n");

  Node->IsFlow = true;

  // Update predecessors' terminators and collect conditions for successors
  // of the flow block.
  for (BBNode *PredNode : Subtrees.nodes()) {
    if (!PredNode->NumOpenOutgoing)
      continue;

    Predecessors.push_back(PredNode);
    Node->NumOpenIncoming++;

    LLVM_DEBUG(dbgs() << "    pred: " << PredNode->BB->getName() << "\n");

    if (PredNode->IsFlow) {
      Value *FirstCondition = nullptr;
      Value *RemainderCondition = nullptr;

      for (const auto &OutgoingE : PredNode->FlowOutgoing) {
        if (OutgoingE.first == PredNode->FlowFirstSuccessor) {
          FirstCondition = OutgoingE.second;
          Instruction *Not =
              BinaryOperator::CreateNot(FirstCondition, "", PredNode->BB);
          RemainderCondition = Not;
          InsertedConditions.push_back(Not);
        } else {
          S.add(PredNode, OutgoingE.first, OutgoingE.second);
          OutgoingE.first->FlowIncoming.erase(
              std::find(OutgoingE.first->FlowIncoming.begin(),
                        OutgoingE.first->FlowIncoming.end(),
                        PredNode));
          OutgoingE.first->NumOpenIncoming--;
          PredNode->NumOpenOutgoing--;
        }
      }

      assert(PredNode->NumOpenOutgoing == 0);

      if (!RemainderCondition)
        RemainderCondition = ConstantInt::getTrue(F->getContext());

      Node->FlowIncoming.push_back(PredNode);

      PredNode->FlowOutgoing.clear();
      if (FirstCondition)
        PredNode->FlowOutgoing.emplace_back(PredNode->FlowFirstSuccessor,
                                            FirstCondition);
      PredNode->FlowOutgoing.emplace_back(Node.get(), RemainderCondition);
      PredNode->NumOpenOutgoing = 1;
    } else {
      BranchInst *Branch = dyn_cast<BranchInst>(PredNode->BB->getTerminator());
      assert(Branch);

      BBNode &TrueNode = *NodeForBB[Branch->getSuccessor(0)];

      if (Branch->isUnconditional()) {
        assert(PredNode->NumOpenOutgoing == 1);

        S.add(PredNode, &TrueNode, ConstantInt::getTrue(F->getContext()));
        TrueNode.NumOpenIncoming--;

        Branch->setSuccessor(0, Flow);
      } else {
        BBNode &FalseNode = *NodeForBB[Branch->getSuccessor(1)];
        unsigned Rerouted = 0;

        if (!TrueNode.Visited) {
          Value *Cond = PredNode->NumOpenOutgoing == 2
                            ? Branch->getCondition()
                            : ConstantInt::getTrue(F->getContext());
          S.add(PredNode, &TrueNode, Cond);
          TrueNode.NumOpenIncoming--;

          Branch->setSuccessor(0, Flow);
          Rerouted++;
        }

        if (!FalseNode.Visited) {
          Value *Cond;
          if (PredNode->NumOpenOutgoing == 2) {
            Instruction *Not =
                BinaryOperator::CreateNot(Branch->getCondition(), "", Branch);
            Cond = Not;
            InsertedConditions.push_back(Not);
          } else {
            Cond = ConstantInt::getTrue(F->getContext());
          }

          S.add(PredNode, &FalseNode, Cond);
          FalseNode.NumOpenIncoming--;

          Branch->setSuccessor(1, Flow);
          Rerouted++;
        }

        assert(Rerouted == PredNode->NumOpenOutgoing);

        if (Rerouted == 2) {
          Branch->eraseFromParent();
          BranchInst::Create(Flow, PredNode->BB);
          PredNode->NumOpenOutgoing = 1;
        }
      }
    }
  }

  // Setup the new flow node's outgoing edges
  Type *Boolean = Type::getInt1Ty(F->getContext());

  for (BBNode *SuccNode : S.SuccessorsVec) {
    const DenseMap<BBNode *, Value *>& Conditions = S.Conditions[SuccNode];
    PHINode *Phi = PHINode::Create(Boolean, Predecessors.size(), "", Flow);

    for (BBNode *PredNode : Predecessors) {
      Value *Condition = Conditions.lookup(PredNode);
      if (!Condition)
        Condition = ConstantInt::getFalse(F->getContext());
      Phi->addIncoming(Condition, PredNode->BB);
    }

    Value *Cond =
        SimplifyInstruction(Phi, SuccNode->BB->getModule()->getDataLayout());
    if (Cond)
      Phi->eraseFromParent();
    else
    {
      Cond = Phi;
      InsertedConditions.push_back(Phi);
    }

    Phi = nullptr;

    SuccNode->FlowIncoming.push_back(Node.get());
    SuccNode->NumOpenIncoming++;

    Node->FlowOutgoing.emplace_back(SuccNode, Cond);
    Node->NumOpenOutgoing++;

    LLVM_DEBUG(dbgs() << "    succ: " << SuccNode->BB->getName() << "; "
                      << *Cond << "\n");

    if (!ReroutedSuccessors.count(SuccNode->BB)) { // only used for Phi recreation
      ReroutedSuccessors.insert(SuccNode->BB);
      ReroutedSuccessorsOrdered.push_back(SuccNode->BB);
    }
  }

  addNode(*Node);

  NodeForBB[Flow] = std::move(Node);
  Changed = true;
}

void ReconvergeCFG::addNode(BBNode &Node) {
  BBNodeVec VisitedPreds;

  LLVM_DEBUG(dbgs() << "  addNode " << Node.BB->getName()
                    << ", IsVirtual = " << Node.IsVirtual << "\n");

  for (BBNode &PredNode : Node.predecessors(NodeForBB)) {
    if (PredNode.Visited)
      VisitedPreds.push_back(&PredNode);
  }

  // Update the open tree structure.
  assert(Node.NumOpenIncoming || Node.NumOpenOutgoing);

  if (!VisitedPreds.empty()) {
    // Find a common ancestor of all visited predecessors.
    BBNodeVec Heads;
    DenseSet<BBNode *> Subtree;

    for (BBNode *Node : VisitedPreds) {
      assert(Node->Parent);
      Heads.push_back(Node);
    }

    unsigned i = 0;
    bool HitRoot = false;
    while (!Heads.empty() && (Heads.size() > 1 || HitRoot)) {
      if (i >= Heads.size())
        i = 0;

      if (Subtree.count(Heads[i])) {
        Heads.erase(Heads.begin() + i);
        continue;
      }

      Subtree.insert(Heads[i]);

      if (!Heads[i]->Parent) {
        assert(Heads[i] == &Root);
        HitRoot = true;
        Heads.erase(Heads.begin() + i);
        continue;
      }

      Heads[i] = Heads[i]->Parent;
      i++;
    }

    // Walk back down from the common ancestor, flattening all nodes in the
    // subtree to a single path. Leaf nodes are kept at the bottom.
    BBNode *Head = HitRoot ? &Root : Heads[0];
    BBNodeVec Branches;
    BBNodeVec Leaves;

    for (;;) {
      simple_ilist<BBNode>::iterator ChildI = Head->Children.begin();
      while (ChildI != Head->Children.end()) {
        BBNode &Child = *ChildI;
        auto NextI = ChildI;
        ++NextI;

        if (Subtree.count(&Child)) {
          Head->Children.erase(ChildI);
          Child.Parent = nullptr;

          if (Child.Children.empty())
            Leaves.push_back(&Child);
          else
            Branches.push_back(&Child);
        }

        ChildI = NextI;
      }

      BBNode *Next = nullptr;
      if (!Branches.empty()) {
        Next = Branches.back();
        Branches.pop_back();
      } else if (!Leaves.empty()) {
        Next = Leaves.back();
        Leaves.pop_back();
      } else
        break;

      Head->Children.push_back(*Next);
      Next->Parent = Head;
      Head = Next;
    }

    Head->Children.push_back(Node);
    Node.Parent = Head;
  } else {
    Root.Children.push_back(Node);
    Node.Parent = &Root;
  }

  static int order = 0;
  LLVM_DEBUG(printOTGraph(&Root, std::to_string(order++) + "_" + Node.BB->getName().str() + ".txt"));

  LLVM_DEBUG(printTree());

  // Update open edge counts, remove closed nodes, and fixup flow blocks
  for (BBNode *PredNode : VisitedPreds) {
    assert(PredNode->NumOpenOutgoing > 0);
    assert(Node.NumOpenIncoming > 0);

    LLVM_DEBUG(dbgs() << "    predecessor: " << PredNode->BB->getName()
                      << "\n");

    PredNode->NumOpenOutgoing--;
    if (!PredNode->IsUniform)
      PredNode->IsArmed = true;
    Node.NumOpenIncoming--;

    if (PredNode->IsFlow && !PredNode->FlowFirstSuccessor)
      PredNode->FlowFirstSuccessor = &Node;

    if (!PredNode->NumOpenOutgoing && !PredNode->NumOpenIncoming)
      removeNode(*PredNode);
  }

  if (!Node.NumOpenIncoming && !Node.NumOpenOutgoing)
    removeNode(Node);

  Node.Visited = true;
}

void ReconvergeCFG::removeNode(BBNode &Node) {
  LLVM_DEBUG(dbgs() << "      remove from open tree: " << Node.BB->getName()
                    << "\n");

  while (!Node.Children.empty()) {
    auto ChildI = Node.Children.begin();
    BBNode &Child = *ChildI;
    Node.Children.erase(ChildI);
    Child.Parent = Node.Parent;
    if (Child.Parent)
      Child.Parent->Children.push_back(Child);
  }

  Node.Parent->Children.erase(simple_ilist<BBNode>::iterator(Node));
  Node.Parent = nullptr;

  // Create the final terminator for flow blocks
  if (Node.IsFlow) {
    assert(Node.FlowOutgoing.size() <= 2);

    if (Node.FlowOutgoing.size() == 2) {
      BranchInst::Create(Node.FlowOutgoing[0].first->BB,
                         Node.FlowOutgoing[1].first->BB,
                         Node.FlowOutgoing[0].second, Node.BB);
    } else {
      BranchInst::Create(Node.FlowOutgoing[0].first->BB, Node.BB);
    }

    for (const auto &OutgoingE : Node.FlowOutgoing) {
      OutgoingE.first->FlowIncoming.erase(
          std::find(OutgoingE.first->FlowIncoming.begin(),
                    OutgoingE.first->FlowIncoming.end(), &Node));
    }

    Node.FlowOutgoing.clear();
  }
}

void ReconvergeCFG::recreatePhis() {
  SSAUpdater Updater;

  // The SSAUpdater gets confused when there are phi nodes with incorrect
  // predecessor lists. Since we cannot just remove phi nodes either, as they
  // may be used by other phi nodes, we just save the incoming values
  // information and reset the phi's incoming blocks.
  DenseMap<PHINode *, PHIInfo> PhiInfos;
  SmallVector<PHINode *, 32> Phis;

  for (BasicBlock *BB : ReroutedSuccessorsOrdered) {
    for (Instruction &I : *BB) {
      PHINode *Phi = dyn_cast<PHINode>(&I);
      if (!Phi)
        break;

      PHIInfo &Info = PhiInfos[Phi];

      while (Phi->getNumIncomingValues()) {
        unsigned i = Phi->getNumIncomingValues() - 1;
        Info.OriginalIncoming.emplace_back(Phi->getIncomingBlock(i),
                                           Phi->getIncomingValue(i));
        Phi->removeIncomingValue(i, false);
      }

      Value *Undef = UndefValue::get(Phi->getType());
      for (BasicBlock *Pred : predecessors(BB))
        Phi->addIncoming(Undef, Pred);

      Phis.push_back(Phi);
    }
  }

  // Rebuild SSA form
  for (PHINode *Phi : Phis) {
    PHIInfo &Info = PhiInfos[Phi];

    Updater.Initialize(Phi->getType(), Phi->getName());

    for (const auto &Incoming : Info.OriginalIncoming)
      Updater.AddAvailableValue(Incoming.first, Incoming.second);

    Info.New = Updater.GetValueInMiddleOfBlock(Phi->getParent());
  }

  for (const auto &PhiE : PhiInfos) {
    if (PhiE.second.New != PhiE.first) {
      PhiE.first->replaceAllUsesWith(PhiE.second.New);
      PhiE.first->eraseFromParent();
    }
  }
}

void ReconvergeCFG::printTree() {
  BBNodeVec Stack;
  BBNode *Parent = nullptr;
  unsigned Depth = 0;

  Stack.push_back(&Root);
  while (!Stack.empty()) {
    BBNode &Node = *Stack.back();
    Stack.pop_back();

    while (Node.Parent != Parent) {
      Depth--;
      Parent = Node.Parent;
    }

    for (unsigned i = 0; i < Depth; ++i)
      dbgs() << "  ";
    if (!Node.Parent) {
      dbgs() << "(root)";
    } else {
      dbgs() << Node.BB->getName();
      if (Node.IsVirtual)
        dbgs() << " (virtual)";
    }
    dbgs() << "\n";

    if (!Node.Children.empty()) {
      Depth++;
      Parent = &Node;

      for (BBNode &Child : Node.Children)
        Stack.push_back(&Child);
    }
  }
}

Pass *llvm::createReconvergeCFGPass(bool SkipUniformBranches) {
  return new ReconvergeCFG(SkipUniformBranches);
}
