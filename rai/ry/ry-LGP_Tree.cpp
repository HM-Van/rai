#ifdef RAI_PYBIND

#include "ry-LGP_Tree.h"
#include "ry-KOMO.h"
#include "types.h"

#include "../LGP/LGP_tree.h"

void init_LGP_Tree(pybind11::module &m) {
pybind11::class_<ry::RyLGP_Tree>(m, "LGP_Tree")
.def("walkToNode", [](ry::RyLGP_Tree& self, const char* seq, int verbose) {
  self.lgp->walkToNode(seq, verbose);
}, "", pybind11::arg("seq"),
pybind11::arg("verbose")=true)

.def("walkToRoot", [](ry::RyLGP_Tree& self) {
  self.lgp->focusNode = self.lgp->root;
})

.def("walkToParent", [](ry::RyLGP_Tree& self) {
  self.lgp->focusNode = self.lgp->focusNode->parent;
})

.def("walkToDecision", [](ry::RyLGP_Tree& self, uint decision) {
  LGP_Node* focusNode = self.lgp->focusNode;
  if(!focusNode->isExpanded) focusNode->expand();
  self.lgp->focusNode = focusNode->children(decision);
})

.def("getDecisions", [](ry::RyLGP_Tree& self) {
  LGP_Node* focusNode = self.lgp->focusNode;
  if(!focusNode->isExpanded) focusNode->expand();
  StringA decisions(focusNode->children.N);
  uint c=0;
  for(LGP_Node* a:focusNode->children) {
    decisions(c++) <<*a->decision;
  }
  return I_conv(decisions);
})

.def("nodeInfo", [](ry::RyLGP_Tree& self) {
  Graph G = self.lgp->focusNode->getInfo();
  LOG(0) <<G;
  return graph2dict(G);
})

.def("nodeState", [](ry::RyLGP_Tree& self){
    Graph G = self.lgp->focusNode->getState();
    return graph2list(G);
  } )

.def("isTerminal", [](ry::RyLGP_Tree& self){
  return self.lgp->focusNode->isTerminal;
} )

.def("isInfeasible", [](ry::RyLGP_Tree& self){
  return self.lgp->focusNode->isInfeasible;
} )

.def("returnFeasible", [](ry::RyLGP_Tree& self, BoundType bound){
  return self.lgp->focusNode->feasible(bound);
} )

.def("returnConstraint", [](ry::RyLGP_Tree& self, BoundType bound){
  return self.lgp->focusNode->constraints(bound);
} )

.def("reset", [](ry::RyLGP_Tree& self){
  return self.lgp->focusNode->resetData();
} )

.def("isExpanded", [](ry::RyLGP_Tree& self){
  return self.lgp->focusNode->isExpanded;
} )

.def("viewTree", [](ry::RyLGP_Tree& self) {
  self.lgp->displayTreeUsingDot();
})

.def("optBound", [](ry::RyLGP_Tree& self, BoundType bound, bool collisions, bool view, double initnoise, double delay) {
  self.lgp->focusNode->optBound(bound, collisions, -1, initnoise);
  if(view){
    if(bound == BD_seqPath) {
      self.lgp->focusNode->komoProblem(bound)->displayTrajectory(delay, false, false);
    } else {
      self.lgp->focusNode->komoProblem(bound)->displayTrajectory(.1, false, false);
    }
  }
}, "", pybind11::arg("bound"),
pybind11::arg("collisions"),
pybind11::arg("view")=true,
pybind11::arg("initnoise")=0.01,
pybind11::arg("delay")=0.02)

.def("getKOMOforBound", [](ry::RyLGP_Tree& self, BoundType bound) {
  return ry::RyKOMO(self.lgp->focusNode->komoProblem(bound));
})

.def("addTerminalRule", [](ry::RyLGP_Tree& self, const char* precondition, int verbose) {
  self.lgp->fol.addTerminalRule(precondition, verbose);
}, "", pybind11::arg("precondition"),
pybind11::arg("verbose")=1)

.def("run", [](ry::RyLGP_Tree& self, int verbose) {
  self.lgp->displayBound = BD_seqPath;
  self.lgp->LGP_Tree::verbose=verbose;
  self.lgp->threadLoop();
})

.def("stop", [](ry::RyLGP_Tree& self) {
  self.lgp->threadStop();
})

.def("numSolutions", [](ry::RyLGP_Tree& self) {
  return self.lgp->numSolutions();
})

.def("getReport", [](ry::RyLGP_Tree& self, uint solution, BoundType bound) {
  Graph G = self.lgp->getReport(solution, bound);
  return graph2list(G);
})

.def("getKOMO", [](ry::RyLGP_Tree& self, uint solution, BoundType bound) {
  const auto& komo = self.lgp->getKOMO(solution, bound);
  return ry::RyKOMO(komo);
})

.def("resetFrameState", [](ry::RyLGP_Tree& self){
  if(&self.lgp->kin != &self.lgp->root->startKinematics)
    self.lgp->kin = self.lgp->root->startKinematics;
} )

.def("getFrameState", [](ry::RyLGP_Tree& self){
  arr X = self.lgp->kin.getFrameState();
  return pybind11::array(X.dim(), X.p);
} )

.def("getFrameState", [](ry::RyLGP_Tree& self, const char* frame){
  arr X;
  auto Kget = self.lgp->kin;
  rai::Frame *f = Kget.getFrameByName(frame, true);
  if(f) X = f->ensure_X().getArr7d();
  return pybind11::array(X.dim(), X.p);
} )
;
}

#endif
