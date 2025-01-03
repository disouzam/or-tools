// Copyright 2010-2024 Google LLC
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// An implementation of a push-relabel algorithm for the max flow problem.
//
// In the following, we consider a graph G = (V,E,s,t) where V denotes the set
// of nodes (vertices) in the graph, E denotes the set of arcs (edges). s and t
// denote distinguished nodes in G called source and target. n = |V| denotes the
// number of nodes in the graph, and m = |E| denotes the number of arcs in the
// graph.
//
// Each arc (v,w) is associated a capacity c(v,w).
//
// A flow is a function from E to R such that:
//
//  a) f(v,w) <= c(v,w) for all (v,w) in E (capacity constraint.)
//
//  b) f(v,w) = -f(w,v) for all (v,w) in E (flow antisymmetry constraint.)
//
//  c) sum on v f(v,w) = 0  (flow conservation.)
//
// The goal of this algorithm is to find the maximum flow from s to t, i.e.
// for example to maximize sum v f(s,v).
//
// The starting reference for this class of algorithms is:
// A.V. Goldberg and R.E. Tarjan. A new approach to the maximum flow problem.
// ACM Symposium on Theory of Computing, pp. 136-146.
// http://portal.acm.org/citation.cfm?id=12144.
//
// The basic idea of the algorithm is to handle preflows instead of flows,
// and to refine preflows until a maximum flow is obtained.
// A preflow is like a flow, except that the inflow can be larger than the
// outflow. If it is the case at a given node v, it is said that there is an
// excess at node v, and inflow = outflow + excess.
//
// More formally, a preflow is a function f such that:
//
// 1) f(v,w) <= c(v,w) for all (v,w) in E  (capacity constraint). c(v,w)  is a
//    value representing the maximum capacity for arc (v,w).
//
// 2) f(v,w) = -f(w,v) for all (v,w) in E (flow antisymmetry constraint)
//
// 3) excess(v) = sum on u f(u,v) >= 0 is the excess at node v, the
//    algebraic sum of all the incoming preflows at this node.
//
// Each node has an associated "height", in addition to its excess. The
// height of the source is defined to be equal to n, and cannot change. The
// height of the target is defined to be zero, and cannot change either. The
// height of all the other nodes is initialized at zero and is updated during
// the algorithm (see below). For those who want to know the details, the height
// of a node, corresponds to a reduced cost, and this enables one to prove that
// the algorithm actually computes the max flow. Note that the height of a node
// can be initialized to the distance to the target node in terms of number of
// nodes. This has not been tried in this implementation.
//
// A node v is said to be *active* if excess(v) > 0.
//
// In this case the following operations can be applied to it:
//
// - if there are *admissible* incident arcs, i.e. arcs which are not saturated,
//   and whose head's height is lower than the height of the active node
//   considered, a PushFlow operation can be applied. It consists in sending as
//   much flow as both the excess at the node and the capacity of the arc
//   permit.
// - if there are no admissible arcs, the active node considered is relabeled,
//   i.e. its height is increased to 1 + the minimum height of its neighboring
//   nodes on admissible arcs.
// This is implemented in Discharge, which itself calls PushFlow and Relabel.
//
// Before running Discharge, it is necessary to initialize the algorithm with a
// preflow. This is done in InitializePreflow, which saturates all the arcs
// leaving the source node, and sets the excess at the heads of those arcs
// accordingly.
//
// The algorithm terminates when there are no remaining active nodes, i.e. all
// the excesses at all nodes are equal to zero. In this case, a maximum flow is
// obtained.
//
// The complexity of this algorithm depends amongst other things on the choice
// of the next active node. It has been shown, for example in:
// L. Tuncel, "On the Complexity of Preflow-Push Algorithms for Maximum-Flow
// Problems", Algorithmica 11(4): 353-359 (1994).
// and
// J. Cheriyan and K. Mehlhorn, "An analysis of the highest-level selection rule
// in the preflow-push max-flow algorithm", Information processing letters,
// 69(5):239-242 (1999).
// http://www.math.uwaterloo.ca/~jcheriya/PS_files/me3.0.ps
//
// ...that choosing the active node with the highest level yields a
// complexity of O(n^2 * sqrt(m)).
//
// TODO(user): implement the above active node choice rule.
//
// This has been validated experimentally in:
// R.K. Ahuja, M. Kodialam, A.K. Mishra, and J.B. Orlin, "Computational
// Investigations of Maximum Flow Algorithms", EJOR 97:509-542(1997).
// http://jorlin.scripts.mit.edu/docs/publications/58-comput%20investigations%20of.pdf.
//
//
// TODO(user): an alternative would be to evaluate:
// A.V. Goldberg, "The Partial Augment-Relabel Algorithm for the Maximum Flow
// Problem.” In Proceedings of Algorithms ESA, LNCS 5193:466-477, Springer 2008.
// http://www.springerlink.com/index/5535k2j1mt646338.pdf
//
// An interesting general reference on network flows is:
// R. K. Ahuja, T. L. Magnanti, J. B. Orlin, "Network Flows: Theory, Algorithms,
// and Applications," Prentice Hall, 1993, ISBN: 978-0136175490,
// http://www.amazon.com/dp/013617549X
//
// Keywords: Push-relabel, max-flow, network, graph, Goldberg, Tarjan, Dinic,
//           Dinitz.

#ifndef OR_TOOLS_GRAPH_GENERIC_MAX_FLOW_H_
#define OR_TOOLS_GRAPH_GENERIC_MAX_FLOW_H_

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/memory/memory.h"
#include "absl/strings/string_view.h"
#include "ortools/base/logging.h"
#include "ortools/graph/ebert_graph.h"
#include "ortools/graph/flow_problem.pb.h"
#include "ortools/graph/graphs.h"
#include "ortools/util/stats.h"
#include "ortools/util/zvector.h"

namespace operations_research {

// Specific but efficient priority queue implementation. The priority type must
// be an integer. The queue allows to retrieve the element with highest priority
// but only allows pushes with a priority greater or equal to the highest
// priority in the queue minus one. All operations are in O(1) and the memory is
// in O(num elements in the queue). Elements with the same priority are
// retrieved with LIFO order.
//
// Note(user): As far as I know, this is an original idea and is the only code
// that use this in the Maximum Flow context. Papers usually refer to an
// height-indexed array of simple linked lists of active node with the same
// height. Even worse, sometimes they use double-linked list to allow arbitrary
// height update in order to detect missing height (used for the Gap heuristic).
// But this can actually be implemented a lot more efficiently by just
// maintaining the height distribution of all the node in the graph.
template <typename Element, typename IntegerPriority>
class PriorityQueueWithRestrictedPush {
 public:
  PriorityQueueWithRestrictedPush() : even_queue_(), odd_queue_() {}

#ifndef SWIG
  // This type is neither copyable nor movable.
  PriorityQueueWithRestrictedPush(const PriorityQueueWithRestrictedPush&) =
      delete;
  PriorityQueueWithRestrictedPush& operator=(
      const PriorityQueueWithRestrictedPush&) = delete;
#endif

  // Is the queue empty?
  bool IsEmpty() const;

  // Clears the queue.
  void Clear();

  // Push a new element in the queue. Its priority must be greater or equal to
  // the highest priority present in the queue, minus one. This condition is
  // DCHECKed, and violating it yields erroneous queue behavior in NDEBUG mode.
  void Push(Element element, IntegerPriority priority);

  // Returns the element with highest priority and remove it from the queue.
  // IsEmpty() must be false, this condition is DCHECKed.
  Element Pop();

 private:
  // Helper function to get the last element of a vector and pop it.
  Element PopBack(std::vector<std::pair<Element, IntegerPriority> >* queue);

  // This is the heart of the algorithm. basically we split the elements by
  // parity of their priority and the precondition on the Push() ensures that
  // both vectors are always sorted by increasing priority.
  std::vector<std::pair<Element, IntegerPriority> > even_queue_;
  std::vector<std::pair<Element, IntegerPriority> > odd_queue_;
};

// We want an enum for the Status of a max flow run, and we want this
// enum to be scoped under GenericMaxFlow<>. Unfortunately, swig
// doesn't handle templated enums very well, so we need a base,
// untemplated class to hold it.
class MaxFlowStatusClass {
 public:
  enum Status {
    NOT_SOLVED,    // The problem was not solved, or its data were edited.
    OPTIMAL,       // Solve() was called and found an optimal solution.
    INT_OVERFLOW,  // There is a feasible flow > max possible flow.

    // TODO(user): These are no longer used. Remove.
    BAD_INPUT,
    BAD_RESULT
  };
};

// Generic MaxFlow (there is a default MaxFlow specialization defined below)
// that works with StarGraph and all the reverse arc graphs from graph.h, see
// the end of max_flow.cc for the exact types this class is compiled for.
template <typename Graph>
class GenericMaxFlow : public MaxFlowStatusClass {
 public:
  typedef typename Graph::NodeIndex NodeIndex;
  typedef typename Graph::ArcIndex ArcIndex;
  typedef typename Graph::OutgoingArcIterator OutgoingArcIterator;
  typedef typename Graph::OutgoingOrOppositeIncomingArcIterator
      OutgoingOrOppositeIncomingArcIterator;
  typedef typename Graph::IncomingArcIterator IncomingArcIterator;

  // The height of a node never excess 2 times the number of node, so we
  // use the same type as a Node index.
  typedef NodeIndex NodeHeight;

  // Initialize a MaxFlow instance on the given graph. The graph does not need
  // to be fully built yet, but its capacity reservation are used to initialize
  // the memory of this class. source and sink must also be valid node of
  // graph.
  GenericMaxFlow(const Graph* graph, NodeIndex source, NodeIndex sink);

#ifndef SWIG
  // This type is neither copyable nor movable.
  GenericMaxFlow(const GenericMaxFlow&) = delete;
  GenericMaxFlow& operator=(const GenericMaxFlow&) = delete;
#endif

  virtual ~GenericMaxFlow() {}

  // Returns the graph associated to the current object.
  const Graph* graph() const { return graph_; }

  // Returns the status of last call to Solve(). NOT_SOLVED is returned if
  // Solve() has never been called or if the problem has been modified in such a
  // way that the previous solution becomes invalid.
  Status status() const { return status_; }

  // Returns the index of the node corresponding to the source of the network.
  NodeIndex GetSourceNodeIndex() const { return source_; }

  // Returns the index of the node corresponding to the sink of the network.
  NodeIndex GetSinkNodeIndex() const { return sink_; }

  // Sets the capacity for arc to new_capacity.
  void SetArcCapacity(ArcIndex arc, FlowQuantity new_capacity);

  // Returns true if a maximum flow was solved.
  bool Solve();

  // Returns the total flow found by the algorithm.
  FlowQuantity GetOptimalFlow() const { return node_excess_[sink_]; }

  // Returns the flow on arc using the equations given in the comment on
  // residual_arc_capacity_.
  FlowQuantity Flow(ArcIndex arc) const {
    if (IsArcDirect(arc)) {
      return residual_arc_capacity_[Opposite(arc)];
    } else {
      return -residual_arc_capacity_[arc];
    }
  }

  // Returns the capacity of arc using the equations given in the comment on
  // residual_arc_capacity_.
  FlowQuantity Capacity(ArcIndex arc) const {
    if (IsArcDirect(arc)) {
      return residual_arc_capacity_[arc] +
             residual_arc_capacity_[Opposite(arc)];
    } else {
      return 0;
    }
  }

  // Returns the nodes reachable from the source in the residual graph, the
  // outgoing arcs of this set form a minimum cut.
  void GetSourceSideMinCut(std::vector<NodeIndex>* result);

  // Returns the nodes that can reach the sink in the residual graph, the
  // outgoing arcs of this set form a minimum cut. Note that if this is the
  // complement of GetNodeReachableFromSource(), then the min-cut is unique.
  //
  // TODO(user): In the two-phases algorithm, we can get this minimum cut
  // without doing the second phase. Add an option for this if there is a need
  // to, note that the second phase is pretty fast so the gain will be small.
  void GetSinkSideMinCut(std::vector<NodeIndex>* result);

  // Returns true if there exists a path from the source to the sink with
  // remaining capacity. This allows us to easily check at the end that the flow
  // we computed is indeed optimal (provided that all the conditions tested by
  // CheckResult() also hold).
  bool AugmentingPathExists() const;

  // Returns the protocol buffer representation of the current problem.
  FlowModelProto CreateFlowModel();

 protected:
  // Checks whether the result is valid, i.e. that node excesses are all equal
  // to zero (we have a flow) and that residual capacities are all non-negative
  // or zero.
  bool CheckResult() const;

  // Returns true if arc is admissible.
  bool IsAdmissible(NodeIndex tail, ArcIndex arc,
                    const NodeHeight* potentials) const {
    DCHECK_EQ(tail, Tail(arc));
    return residual_arc_capacity_[arc] > 0 &&
           potentials[tail] == potentials[Head(arc)] + 1;
  }

  // Returns true if node is active, i.e. if its excess is positive and it
  // is neither the source or the sink of the graph.
  bool IsActive(NodeIndex node) const {
    return (node != source_) && (node != sink_) && (node_excess_[node] > 0);
  }

  // Sets the capacity of arc to 'capacity' and clears the flow on arc.
  void SetCapacityAndClearFlow(ArcIndex arc, FlowQuantity capacity) {
    residual_arc_capacity_.Set(arc, capacity);
    residual_arc_capacity_.Set(Opposite(arc), 0);
  }

  // Returns true if a precondition for Relabel is met, i.e. the outgoing arcs
  // of node are all either saturated or the heights of their heads are greater
  // or equal to the height of node.
  bool CheckRelabelPrecondition(NodeIndex node) const;

  // Returns context concatenated with information about arc
  // in a human-friendly way.
  std::string DebugString(absl::string_view context, ArcIndex arc) const;

  // Initializes the container active_nodes_.
  void InitializeActiveNodeContainer();

  // Get the first element from the active node container.
  NodeIndex GetAndRemoveFirstActiveNode() {
    return active_node_by_height_.Pop();
  }

  // Push element to the active node container.
  void PushActiveNode(const NodeIndex& node) {
    active_node_by_height_.Push(node, node_potential_[node]);
  }

  // Check the emptiness of the container.
  bool IsEmptyActiveNodeContainer() { return active_node_by_height_.IsEmpty(); }

  // Performs optimization step.
  void Refine();
  void RefineWithGlobalUpdate();

  // Discharges an active node node by saturating its admissible adjacent arcs,
  // if any, and by relabelling it when it becomes inactive.
  void Discharge(NodeIndex node);

  // Initializes the preflow to a state that enables to run Refine.
  void InitializePreflow();

  // Clears the flow excess at each node by pushing the flow back to the source:
  // - Do a depth-first search from the source in the direct graph to cancel
  //   flow cycles.
  // - Then, return flow excess along the depth-first search tree (by pushing
  //   the flow in the reverse dfs topological order).
  // The theoretical complexity is O(mn), but it is a lot faster in practice.
  void PushFlowExcessBackToSource();

  // Computes the best possible node potential given the current flow using a
  // reverse breadth-first search from the sink in the reverse residual graph.
  // This is an implementation of the global update heuristic mentioned in many
  // max-flow papers. See for instance: B.V. Cherkassky, A.V. Goldberg, "On
  // implementing push-relabel methods for the maximum flow problem",
  // Algorithmica, 19:390-410, 1997.
  // ftp://reports.stanford.edu/pub/cstr/reports/cs/tr/94/1523/CS-TR-94-1523.pdf
  void GlobalUpdate();

  // Tries to saturate all the outgoing arcs from the source that can reach the
  // sink. Most of the time, we can do that in one go, except when more flow
  // than kMaxFlowQuantity can be pushed out of the source in which case we
  // have to be careful. Returns true if some flow was pushed.
  bool SaturateOutgoingArcsFromSource();

  // Pushes flow on arc,  i.e. consumes flow on residual_arc_capacity_[arc],
  // and consumes -flow on residual_arc_capacity_[Opposite(arc)]. Updates
  // node_excess_ at the tail and head of arc accordingly.
  void PushFlow(FlowQuantity flow, NodeIndex tail, ArcIndex arc);

  // Same as PushFlow() but with a cached node_excess_.data(), this is faster
  // in hot loop as we remove bound checking and the pointer is constant.
  void FastPushFlow(FlowQuantity flow, NodeIndex tail, ArcIndex arc,
                    FlowQuantity* node_excess);

  // Relabels a node, i.e. increases its height by the minimum necessary amount.
  // This version of Relabel is relaxed in a way such that if an admissible arc
  // exists at the current node height, then the node is not relabeled. This
  // enables us to deal with wrong values of first_admissible_arc_[node] when
  // updating it is too costly.
  void Relabel(NodeIndex node);

  // Handy member functions to make the code more compact.
  NodeIndex Head(ArcIndex arc) const { return graph_->Head(arc); }
  NodeIndex Tail(ArcIndex arc) const { return graph_->Tail(arc); }
  ArcIndex Opposite(ArcIndex arc) const;
  bool IsArcDirect(ArcIndex arc) const;
  bool IsArcValid(ArcIndex arc) const;

  // Returns the set of nodes reachable from start in the residual graph or in
  // the reverse residual graph (if reverse is true).
  template <bool reverse>
  void ComputeReachableNodes(NodeIndex start, std::vector<NodeIndex>* result);

  // Maximum manageable flow.
  static constexpr FlowQuantity kMaxFlowQuantity =
      std::numeric_limits<FlowQuantity>::max();

  // A pointer to the graph passed as argument.
  const Graph* graph_;

  // An array representing the excess for each node in graph_.
  std::unique_ptr<FlowQuantity[]> node_excess_;

  // An array representing the height function for each node in graph_. For a
  // given node, this is a lower bound on the shortest path length from this
  // node to the sink in the residual network. The height of a node always goes
  // up during the course of a Solve().
  //
  // Since initially we saturate all the outgoing arcs of the source, we can
  // never reach the sink from the source in the residual graph. Initially we
  // set the height of the source to n (the number of node of the graph) and it
  // never changes. If a node as an height >= n, then this node can't reach the
  // sink and its height minus n is a lower bound on the shortest path length
  // from this node to the source in the residual graph.
  std::unique_ptr<NodeHeight[]> node_potential_;

  // An array representing the residual_capacity for each arc in graph_.
  // Residual capacities enable one to represent the capacity and flow for all
  // arcs in the graph in the following manner.
  // For all arc, residual_arc_capacity_[arc] = capacity[arc] - flow[arc]
  // Moreover, for reverse arcs, capacity[arc] = 0 by definition,
  // Also flow[Opposite(arc)] = -flow[arc] by definition.
  // Therefore:
  // - for a direct arc:
  //    flow[arc] = 0 - flow[Opposite(arc)]
  //              = capacity[Opposite(arc)] - flow[Opposite(arc)]
  //              = residual_arc_capacity_[Opposite(arc)]
  // - for a reverse arc:
  //    flow[arc] = -residual_arc_capacity_[arc]
  // Using these facts enables one to only maintain residual_arc_capacity_,
  // instead of both capacity and flow, for each direct and indirect arc. This
  // reduces the amount of memory for this information by a factor 2.
  ZVector<FlowQuantity> residual_arc_capacity_;

  // An array representing the first admissible arc for each node in graph_.
  std::unique_ptr<ArcIndex[]> first_admissible_arc_;

  // A priority queue used for managing active nodes in the algorithm. It allows
  // to select the active node with highest height before each Discharge().
  // Moreover, since all pushes from this node will be to nodes with height
  // greater or equal to the initial discharged node height minus one, the
  // PriorityQueueWithRestrictedPush is a perfect fit.
  PriorityQueueWithRestrictedPush<NodeIndex, NodeHeight> active_node_by_height_;

  // The index of the source node in graph_.
  NodeIndex source_;

  // The index of the sink node in graph_.
  NodeIndex sink_;

  // The status of the problem.
  Status status_;

  // BFS queue used by the GlobalUpdate() function. We do not use a C++ queue
  // because we need access to the vector for different optimizations.
  std::vector<bool> node_in_bfs_queue_;
  std::vector<NodeIndex> bfs_queue_;

  // Statistics about this class.
  mutable StatsGroup stats_;
};

#if !SWIG

// Default instance MaxFlow that uses StarGraph. Note that we cannot just use a
// typedef because of dependent code expecting MaxFlow to be a real class.
// TODO(user): Modify this code and remove it.
class MaxFlow : public GenericMaxFlow<StarGraph> {
 public:
  MaxFlow(const StarGraph* graph, NodeIndex source, NodeIndex target)
      : GenericMaxFlow(graph, source, target) {}
};

#endif  // SWIG

template <typename Element, typename IntegerPriority>
bool PriorityQueueWithRestrictedPush<Element, IntegerPriority>::IsEmpty()
    const {
  return even_queue_.empty() && odd_queue_.empty();
}

template <typename Element, typename IntegerPriority>
void PriorityQueueWithRestrictedPush<Element, IntegerPriority>::Clear() {
  even_queue_.clear();
  odd_queue_.clear();
}

template <typename Element, typename IntegerPriority>
void PriorityQueueWithRestrictedPush<Element, IntegerPriority>::Push(
    Element element, IntegerPriority priority) {
  // Since users may rely on it, we DCHECK the exact condition.
  DCHECK(even_queue_.empty() || priority >= even_queue_.back().second - 1);
  DCHECK(odd_queue_.empty() || priority >= odd_queue_.back().second - 1);

  // Note that the DCHECK() below are less restrictive than the ones above but
  // check a necessary and sufficient condition for the priority queue to behave
  // as expected.
  if (priority & 1) {
    DCHECK(odd_queue_.empty() || priority >= odd_queue_.back().second);
    odd_queue_.push_back(std::make_pair(element, priority));
  } else {
    DCHECK(even_queue_.empty() || priority >= even_queue_.back().second);
    even_queue_.push_back(std::make_pair(element, priority));
  }
}

template <typename Element, typename IntegerPriority>
Element PriorityQueueWithRestrictedPush<Element, IntegerPriority>::Pop() {
  DCHECK(!IsEmpty());
  if (even_queue_.empty()) return PopBack(&odd_queue_);
  if (odd_queue_.empty()) return PopBack(&even_queue_);
  if (odd_queue_.back().second > even_queue_.back().second) {
    return PopBack(&odd_queue_);
  } else {
    return PopBack(&even_queue_);
  }
}

template <typename Element, typename IntegerPriority>
Element PriorityQueueWithRestrictedPush<Element, IntegerPriority>::PopBack(
    std::vector<std::pair<Element, IntegerPriority> >* queue) {
  DCHECK(!queue->empty());
  Element element = queue->back().first;
  queue->pop_back();
  return element;
}

template <typename Graph>
GenericMaxFlow<Graph>::GenericMaxFlow(const Graph* graph, NodeIndex source,
                                      NodeIndex sink)
    : graph_(graph),
      residual_arc_capacity_(),
      source_(source),
      sink_(sink),
      stats_("MaxFlow") {
  SCOPED_TIME_STAT(&stats_);
  DCHECK(graph->IsNodeValid(source));
  DCHECK(graph->IsNodeValid(sink));
  const NodeIndex max_num_nodes = Graphs<Graph>::NodeReservation(*graph_);
  if (max_num_nodes > 0) {
    node_excess_ = std::make_unique<FlowQuantity[]>(max_num_nodes);
    node_potential_ = std::make_unique<NodeHeight[]>(max_num_nodes);
    first_admissible_arc_ =
        absl::make_unique<ArcIndex[]>(max_num_nodes);
    std::fill(first_admissible_arc_.get(),
              first_admissible_arc_.get() + max_num_nodes, Graph::kNilArc);
    bfs_queue_.reserve(max_num_nodes);
  }
  const ArcIndex max_num_arcs = Graphs<Graph>::ArcReservation(*graph_);
  if (max_num_arcs > 0) {
    residual_arc_capacity_.Reserve(-max_num_arcs, max_num_arcs - 1);
    residual_arc_capacity_.SetAll(0);
  }
}

template <typename Graph>
void GenericMaxFlow<Graph>::SetArcCapacity(ArcIndex arc,
                                           FlowQuantity new_capacity) {
  SCOPED_TIME_STAT(&stats_);
  DCHECK_LE(0, new_capacity);
  DCHECK(IsArcDirect(arc));
  const FlowQuantity free_capacity = residual_arc_capacity_[arc];
  const FlowQuantity capacity_delta = new_capacity - Capacity(arc);
  if (capacity_delta == 0) {
    return;  // Nothing to do.
  }
  status_ = NOT_SOLVED;
  if (free_capacity + capacity_delta >= 0) {
    // The above condition is true if one of the two conditions is true:
    // 1/ (capacity_delta > 0), meaning we are increasing the capacity
    // 2/ (capacity_delta < 0 && free_capacity + capacity_delta >= 0)
    //    meaning we are reducing the capacity, but that the capacity
    //    reduction is not larger than the free capacity.
    DCHECK((capacity_delta > 0) ||
           (capacity_delta < 0 && free_capacity + capacity_delta >= 0));
    residual_arc_capacity_.Set(arc, free_capacity + capacity_delta);
    DCHECK_LE(0, residual_arc_capacity_[arc]);
  } else {
    // Note that this breaks the preflow invariants but it is currently not an
    // issue since we restart from scratch on each Solve() and we set the status
    // to NOT_SOLVED.
    //
    // TODO(user): The easiest is probably to allow negative node excess in
    // other places than the source, but the current implementation does not
    // deal with this.
    SetCapacityAndClearFlow(arc, new_capacity);
  }
}

template <typename Graph>
void GenericMaxFlow<Graph>::GetSourceSideMinCut(
    std::vector<NodeIndex>* result) {
  ComputeReachableNodes<false>(source_, result);
}

template <typename Graph>
void GenericMaxFlow<Graph>::GetSinkSideMinCut(std::vector<NodeIndex>* result) {
  ComputeReachableNodes<true>(sink_, result);
}

template <typename Graph>
bool GenericMaxFlow<Graph>::CheckResult() const {
  SCOPED_TIME_STAT(&stats_);
  if (node_excess_[source_] != -node_excess_[sink_]) {
    LOG(DFATAL) << "-node_excess_[source_] = " << -node_excess_[source_]
                << " != node_excess_[sink_] = " << node_excess_[sink_];
    return false;
  }
  for (NodeIndex node = 0; node < graph_->num_nodes(); ++node) {
    if (node != source_ && node != sink_) {
      if (node_excess_[node] != 0) {
        LOG(DFATAL) << "node_excess_[" << node << "] = " << node_excess_[node]
                    << " != 0";
        return false;
      }
    }
  }
  for (ArcIndex arc = 0; arc < graph_->num_arcs(); ++arc) {
    const ArcIndex opposite = Opposite(arc);
    const FlowQuantity direct_capacity = residual_arc_capacity_[arc];
    const FlowQuantity opposite_capacity = residual_arc_capacity_[opposite];
    if (direct_capacity < 0) {
      LOG(DFATAL) << "residual_arc_capacity_[" << arc
                  << "] = " << direct_capacity << " < 0";
      return false;
    }
    if (opposite_capacity < 0) {
      LOG(DFATAL) << "residual_arc_capacity_[" << opposite
                  << "] = " << opposite_capacity << " < 0";
      return false;
    }
    // The initial capacity of the direct arcs is non-negative.
    if (direct_capacity + opposite_capacity < 0) {
      LOG(DFATAL) << "initial capacity [" << arc
                  << "] = " << direct_capacity + opposite_capacity << " < 0";
      return false;
    }
  }

  if (GetOptimalFlow() < kMaxFlowQuantity && AugmentingPathExists()) {
    LOG(DFATAL) << "The algorithm terminated, but the flow is not maximal!";
    return false;
  }

  return true;
}

template <typename Graph>
bool GenericMaxFlow<Graph>::AugmentingPathExists() const {
  SCOPED_TIME_STAT(&stats_);

  // We simply compute the reachability from the source in the residual graph.
  const NodeIndex num_nodes = graph_->num_nodes();
  std::vector<bool> is_reached(num_nodes, false);
  std::vector<NodeIndex> to_process;

  to_process.push_back(source_);
  is_reached[source_] = true;
  while (!to_process.empty()) {
    const NodeIndex node = to_process.back();
    to_process.pop_back();
    for (OutgoingOrOppositeIncomingArcIterator it(*graph_, node); it.Ok();
         it.Next()) {
      const ArcIndex arc = it.Index();
      if (residual_arc_capacity_[arc] > 0) {
        const NodeIndex head = graph_->Head(arc);
        if (!is_reached[head]) {
          is_reached[head] = true;
          to_process.push_back(head);
        }
      }
    }
  }
  return is_reached[sink_];
}

template <typename Graph>
bool GenericMaxFlow<Graph>::CheckRelabelPrecondition(NodeIndex node) const {
  DCHECK(IsActive(node));
  for (OutgoingOrOppositeIncomingArcIterator it(*graph_, node); it.Ok();
       it.Next()) {
    const ArcIndex arc = it.Index();
    DCHECK(!IsAdmissible(node, arc, node_potential_.data()))
        << DebugString("CheckRelabelPrecondition:", arc);
  }
  return true;
}

template <typename Graph>
std::string GenericMaxFlow<Graph>::DebugString(absl::string_view context,
                                               ArcIndex arc) const {
  const NodeIndex tail = Tail(arc);
  const NodeIndex head = Head(arc);
  return absl::StrFormat(
      "%s Arc %d, from %d to %d, "
      "Capacity = %d, Residual capacity = %d, "
      "Flow = residual capacity for reverse arc = %d, "
      "Height(tail) = %d, Height(head) = %d, "
      "Excess(tail) = %d, Excess(head) = %d",
      context, arc, tail, head, Capacity(arc), residual_arc_capacity_[arc],
      Flow(arc), node_potential_[tail], node_potential_[head],
      node_excess_[tail], node_excess_[head]);
}

template <typename Graph>
bool GenericMaxFlow<Graph>::Solve() {
  status_ = NOT_SOLVED;
  InitializePreflow();

  // Deal with the case when source_ or sink_ is not inside graph_.
  // Since they are both specified independently of the graph, we do need to
  // take care of this corner case.
  const NodeIndex num_nodes = graph_->num_nodes();
  if (sink_ >= num_nodes || source_ >= num_nodes) {
    // Behave like a normal graph where source_ and sink_ are disconnected.
    // Note that the arc flow is set to 0 by InitializePreflow().
    status_ = OPTIMAL;
    return true;
  }

  RefineWithGlobalUpdate();

  status_ = OPTIMAL;
  DCHECK(CheckResult());

  if (GetOptimalFlow() == kMaxFlowQuantity && AugmentingPathExists()) {
    // In this case, we are sure that the flow is > kMaxFlowQuantity.
    status_ = INT_OVERFLOW;
  }
  IF_STATS_ENABLED(VLOG(1) << stats_.StatString());
  return true;
}

template <typename Graph>
void GenericMaxFlow<Graph>::InitializePreflow() {
  SCOPED_TIME_STAT(&stats_);

  // TODO(user): Ebert graph has an issue with nodes with no arcs, so we
  // use max_num_nodes here to resize vectors.
  const NodeIndex num_nodes = graph_->num_nodes();
  const NodeIndex max_num_nodes = Graphs<Graph>::NodeReservation(*graph_);
  const ArcIndex num_arcs = graph_->num_arcs();

  // InitializePreflow() clears the whole flow that could have been computed
  // by a previous Solve(). This is not optimal in terms of complexity.
  //
  // TODO(user): find a way to make the re-solving incremental (not an obvious
  // task, and there has not been a lot of literature on the subject.)
  std::fill(node_excess_.get(), node_excess_.get() + max_num_nodes, 0);
  for (ArcIndex arc = 0; arc < num_arcs; ++arc) {
    SetCapacityAndClearFlow(arc, Capacity(arc));
  }

  // All the initial heights are zero except for the source whose height is
  // equal to the number of nodes and will never change during the algorithm.
  std::fill(node_potential_.get(), node_potential_.get() + max_num_nodes, 0);
  node_potential_[source_] = num_nodes;

  // Initially no arcs are admissible except maybe the one leaving the source,
  // but we treat the source in a special way, see
  // SaturateOutgoingArcsFromSource().
  std::fill(first_admissible_arc_.get(),
            first_admissible_arc_.get() + max_num_nodes, Graph::kNilArc);
}

// Note(user): Calling this function will break the property on the node
// potentials because of the way we cancel flow on cycle. However, we only call
// that at the end of the algorithm, or just before a GlobalUpdate() that will
// restore the precondition on the node potentials.
template <typename Graph>
void GenericMaxFlow<Graph>::PushFlowExcessBackToSource() {
  SCOPED_TIME_STAT(&stats_);
  const NodeIndex num_nodes = graph_->num_nodes();

  // We implement a variation of Tarjan's strongly connected component algorithm
  // to detect cycles published in: Tarjan, R. E. (1972), "Depth-first search
  // and linear graph algorithms", SIAM Journal on Computing. A description can
  // also be found in wikipedia.

  // Stored nodes are settled nodes already stored in the
  // reverse_topological_order (except the sink_ that we do not actually store).
  std::vector<bool> stored(num_nodes, false);
  stored[sink_] = true;

  // The visited nodes that are not yet stored are all the nodes from the
  // source_ to the current node in the current dfs branch.
  std::vector<bool> visited(num_nodes, false);
  visited[sink_] = true;

  // Stack of arcs to explore in the dfs search.
  // The current node is Head(arc_stack.back()).
  std::vector<ArcIndex> arc_stack;

  // Increasing list of indices into the arc_stack that correspond to the list
  // of arcs in the current dfs branch from the source_ to the current node.
  std::vector<int> index_branch;

  // Node in reverse_topological_order in the final dfs tree.
  std::vector<NodeIndex> reverse_topological_order;

  // We start by pushing all the outgoing arcs from the source on the stack to
  // avoid special conditions in the code. As a result, source_ will not be
  // stored in reverse_topological_order, and this is what we want.
  for (OutgoingArcIterator it(*graph_, source_); it.Ok(); it.Next()) {
    const ArcIndex arc = it.Index();
    const FlowQuantity flow = Flow(arc);
    if (flow > 0) {
      arc_stack.push_back(arc);
    }
  }
  visited[source_] = true;

  // Start the dfs on the subgraph formed by the direct arcs with positive flow.
  while (!arc_stack.empty()) {
    const NodeIndex node = Head(arc_stack.back());

    // If the node is visited, it means we have explored all its arcs and we
    // have just backtracked in the dfs. Store it if it is not already stored
    // and process the next arc on the stack.
    if (visited[node]) {
      if (!stored[node]) {
        stored[node] = true;
        reverse_topological_order.push_back(node);
        DCHECK(!index_branch.empty());
        index_branch.pop_back();
      }
      arc_stack.pop_back();
      continue;
    }

    // The node is a new unexplored node, add all its outgoing arcs with
    // positive flow to the stack and go deeper in the dfs.
    DCHECK(!stored[node]);
    DCHECK(index_branch.empty() ||
           (arc_stack.size() - 1 > index_branch.back()));
    visited[node] = true;
    index_branch.push_back(arc_stack.size() - 1);

    for (OutgoingArcIterator it(*graph_, node); it.Ok(); it.Next()) {
      const ArcIndex arc = it.Index();
      const FlowQuantity flow = Flow(arc);
      const NodeIndex head = Head(arc);
      if (flow > 0 && !stored[head]) {
        if (!visited[head]) {
          arc_stack.push_back(arc);
        } else {
          // There is a cycle.
          // Find the first index to consider,
          // arc_stack[index_branch[cycle_begin]] will be the first arc on the
          // cycle.
          int cycle_begin = index_branch.size();
          while (cycle_begin > 0 &&
                 Head(arc_stack[index_branch[cycle_begin - 1]]) != head) {
            --cycle_begin;
          }

          // Compute the maximum flow that can be canceled on the cycle and the
          // min index such that arc_stack[index_branch[i]] will be saturated.
          FlowQuantity max_flow = flow;
          int first_saturated_index = index_branch.size();
          for (int i = index_branch.size() - 1; i >= cycle_begin; --i) {
            const ArcIndex arc_on_cycle = arc_stack[index_branch[i]];
            if (Flow(arc_on_cycle) <= max_flow) {
              max_flow = Flow(arc_on_cycle);
              first_saturated_index = i;
            }
          }

          // This is just here for a DCHECK() below.
          const FlowQuantity excess = node_excess_[head];

          // Cancel the flow on the cycle, and set visited[node] = false for
          // the node that will be backtracked over.
          PushFlow(-max_flow, node, arc);
          for (int i = index_branch.size() - 1; i >= cycle_begin; --i) {
            const ArcIndex arc_on_cycle = arc_stack[index_branch[i]];
            PushFlow(-max_flow, Tail(arc_on_cycle), arc_on_cycle);
            if (i >= first_saturated_index) {
              DCHECK(visited[Head(arc_on_cycle)]);
              visited[Head(arc_on_cycle)] = false;
            } else {
              DCHECK_GT(Flow(arc_on_cycle), 0);
            }
          }

          // This is a simple check that the flow was pushed properly.
          DCHECK_EQ(excess, node_excess_[head]);

          // Backtrack the dfs just before index_branch[first_saturated_index].
          // If the current node is still active, there is nothing to do.
          if (first_saturated_index < index_branch.size()) {
            arc_stack.resize(index_branch[first_saturated_index]);
            index_branch.resize(first_saturated_index);

            // We backtracked over the current node, so there is no need to
            // continue looping over its arcs.
            break;
          }
        }
      }
    }
  }
  DCHECK(arc_stack.empty());
  DCHECK(index_branch.empty());

  // Return the flow to the sink. Note that the sink_ and the source_ are not
  // stored in reverse_topological_order.
  for (int i = 0; i < reverse_topological_order.size(); i++) {
    const NodeIndex node = reverse_topological_order[i];
    if (node_excess_[node] == 0) continue;
    for (IncomingArcIterator it(*graph_, node); it.Ok(); it.Next()) {
      const ArcIndex opposite_arc = Opposite(it.Index());
      if (residual_arc_capacity_[opposite_arc] > 0) {
        const FlowQuantity flow =
            std::min(node_excess_[node], residual_arc_capacity_[opposite_arc]);
        PushFlow(flow, node, opposite_arc);
        if (node_excess_[node] == 0) break;
      }
    }
    DCHECK_EQ(0, node_excess_[node]);
  }
  DCHECK_EQ(-node_excess_[source_], node_excess_[sink_]);
}

template <typename Graph>
void GenericMaxFlow<Graph>::GlobalUpdate() {
  SCOPED_TIME_STAT(&stats_);
  bfs_queue_.clear();
  int queue_index = 0;
  const NodeIndex num_nodes = graph_->num_nodes();
  node_in_bfs_queue_.assign(num_nodes, false);
  node_in_bfs_queue_[sink_] = true;

  // We cache this as this is a hot-loop and we don't want bound checking.
  FlowQuantity* const node_excess = node_excess_.get();

  // We do a BFS in the reverse residual graph, starting from the sink.
  // Because all the arcs from the source are saturated (except in
  // presence of integer overflow), the source cannot reach the sink in the
  // residual graph. If there is possible overflow and the source is reachable,
  // we still do not want to relabel it, so we start with the source marked.
  node_in_bfs_queue_[source_] = true;
  bfs_queue_.push_back(sink_);

  while (queue_index != bfs_queue_.size()) {
    const NodeIndex node = bfs_queue_[queue_index];
    ++queue_index;
    const NodeIndex candidate_distance = node_potential_[node] + 1;
    for (OutgoingOrOppositeIncomingArcIterator it(*graph_, node); it.Ok();
         it.Next()) {
      const ArcIndex arc = it.Index();
      const NodeIndex head = Head(arc);

      // Skip the arc if the height of head was already set to the correct
      // value (Remember we are doing reverse BFS).
      if (node_in_bfs_queue_[head]) continue;

      // TODO(user): By using more memory we can speed this up quite a bit by
      // avoiding to take the opposite arc here, too options:
      // - if (residual_arc_capacity_[arc] != arc_capacity_[arc])
      // - if (opposite_arc_is_admissible_[arc])  // need updates.
      // Experiment with the first option shows more than 10% gain on this
      // function running time, which is the bottleneck on many instances.
      const ArcIndex opposite_arc = Opposite(arc);
      if (residual_arc_capacity_[opposite_arc] > 0) {
        // Note(user): We used to have a DCHECK_GE(candidate_distance,
        // node_potential_[head]); which is always true except in the case
        // where we can push more than kMaxFlowQuantity out of the source. The
        // problem comes from the fact that in this case, we call
        // PushFlowExcessBackToSource() in the middle of the algorithm. The
        // later call will break the properties of the node potential. Note
        // however, that this function will recompute a good node potential
        // for all the nodes and thus fix the issue.

        // If head is active, we can steal some or all of its excess.
        // This brings a huge gain on some problems.
        // Note(user): I haven't seen this anywhere in the literature.
        // TODO(user): Investigate more and maybe write a publication :)
        if (node_excess[head] > 0) {
          const FlowQuantity flow =
              std::min(node_excess[head], residual_arc_capacity_[opposite_arc]);
          FastPushFlow(flow, head, opposite_arc, node_excess);

          // If the arc became saturated, it is no longer in the residual
          // graph, so we do not need to consider head at this time.
          if (residual_arc_capacity_[opposite_arc] == 0) continue;
        }

        // Note that there is no need to touch first_admissible_arc_[node]
        // because of the relaxed Relabel() we use.
        node_potential_[head] = candidate_distance;
        node_in_bfs_queue_[head] = true;
        bfs_queue_.push_back(head);
      }
    }
  }

  // At the end of the search, some nodes may not be in the bfs_queue_. Such
  // nodes cannot reach the sink_ or source_ in the residual graph, so there is
  // no point trying to push flow toward them. We obtain this effect by setting
  // their height to something unreachable.
  //
  // Note that this also prevents cycling due to our anti-overflow procedure.
  // For instance, suppose there is an edge s -> n outgoing from the source. If
  // node n has no other connection and some excess, we will push the flow back
  // to the source, but if we don't update the height of n
  // SaturateOutgoingArcsFromSource() will push the flow to n again.
  // TODO(user): This is another argument for another anti-overflow algorithm.
  for (NodeIndex node = 0; node < num_nodes; ++node) {
    if (!node_in_bfs_queue_[node]) {
      node_potential_[node] = 2 * num_nodes - 1;
    }
  }

  // Reset the active nodes. Doing it like this pushes the nodes in increasing
  // order of height. Note that bfs_queue_[0] is the sink_ so we skip it.
  DCHECK(IsEmptyActiveNodeContainer());
  for (int i = 1; i < bfs_queue_.size(); ++i) {
    const NodeIndex node = bfs_queue_[i];
    if (node_excess_[node] > 0) {
      DCHECK(IsActive(node));
      PushActiveNode(node);
    }
  }
}

template <typename Graph>
bool GenericMaxFlow<Graph>::SaturateOutgoingArcsFromSource() {
  SCOPED_TIME_STAT(&stats_);
  const NodeIndex num_nodes = graph_->num_nodes();

  // If sink_ or source_ already have kMaxFlowQuantity, then there is no
  // point pushing more flow since it will cause an integer overflow.
  if (node_excess_[sink_] == kMaxFlowQuantity) return false;
  if (node_excess_[source_] == -kMaxFlowQuantity) return false;

  bool flow_pushed = false;
  for (OutgoingArcIterator it(*graph_, source_); it.Ok(); it.Next()) {
    const ArcIndex arc = it.Index();
    const FlowQuantity flow = residual_arc_capacity_[arc];

    // This is a special IsAdmissible() condition for the source.
    if (flow == 0 || node_potential_[Head(arc)] >= num_nodes) continue;

    // We are careful in case the sum of the flow out of the source is greater
    // than kMaxFlowQuantity to avoid overflow.
    const FlowQuantity current_flow_out_of_source = -node_excess_[source_];
    DCHECK_GE(flow, 0) << flow;
    DCHECK_GE(current_flow_out_of_source, 0) << current_flow_out_of_source;
    const FlowQuantity capped_flow =
        kMaxFlowQuantity - current_flow_out_of_source;
    if (capped_flow < flow) {
      // We push as much flow as we can so the current flow on the network will
      // be kMaxFlowQuantity.

      // Since at the beginning of the function, current_flow_out_of_source
      // was different from kMaxFlowQuantity, we are sure to have pushed some
      // flow before if capped_flow is 0.
      if (capped_flow == 0) return true;
      PushFlow(capped_flow, source_, arc);
      return true;
    }
    PushFlow(flow, source_, arc);
    flow_pushed = true;
  }
  DCHECK_LE(node_excess_[source_], 0);
  return flow_pushed;
}

template <typename Graph>
void GenericMaxFlow<Graph>::PushFlow(FlowQuantity flow, NodeIndex tail,
                                     ArcIndex arc) {
  SCOPED_TIME_STAT(&stats_);

  // Update the residual capacity of the arc and its opposite arc.
  DCHECK_NE(flow, 0);
  residual_arc_capacity_[arc] -= flow;
  residual_arc_capacity_[Opposite(arc)] += flow;
  DCHECK_GE(residual_arc_capacity_[arc], 0);
  DCHECK_GE(residual_arc_capacity_[Opposite(arc)], 0);

  // Update the excesses at the tail and head of the arc.
  //
  // Note(user): node_excess_ should be always greater than or equal to 0 except
  // for the source where it should always be smaller than or equal to 0. Note
  // however that we cannot check this because when we cancel the flow on a
  // cycle in PushFlowExcessBackToSource(), we may break this invariant during
  // the operation even if it is still valid at the end.
  node_excess_[tail] -= flow;
  node_excess_[Head(arc)] += flow;
}

template <typename Graph>
void GenericMaxFlow<Graph>::FastPushFlow(FlowQuantity flow, NodeIndex tail,
                                         ArcIndex arc,
                                         FlowQuantity* node_excess) {
  SCOPED_TIME_STAT(&stats_);

  DCHECK_NE(flow, 0);
  residual_arc_capacity_[arc] -= flow;
  residual_arc_capacity_[Opposite(arc)] += flow;
  DCHECK_GE(residual_arc_capacity_[arc], 0);
  DCHECK_GE(residual_arc_capacity_[Opposite(arc)], 0);

  node_excess[tail] -= flow;
  node_excess[Head(arc)] += flow;
}

template <typename Graph>
void GenericMaxFlow<Graph>::InitializeActiveNodeContainer() {
  SCOPED_TIME_STAT(&stats_);
  DCHECK(IsEmptyActiveNodeContainer());
  const NodeIndex num_nodes = graph_->num_nodes();
  for (NodeIndex node = 0; node < num_nodes; ++node) {
    if (IsActive(node)) {
      // This node cannot reach the sink in the residual graph, we don't need to
      // consider it anymore, and we will just push its potential excess back to
      // the source in PushFlowExcessBackToSource() at the end.
      if (node_potential_[node] >= num_nodes) continue;
      PushActiveNode(node);
    }
  }
}

template <typename Graph>
void GenericMaxFlow<Graph>::RefineWithGlobalUpdate() {
  SCOPED_TIME_STAT(&stats_);

  // TODO(user): This should be graph_->num_nodes(), but ebert graph does not
  // have a correct size if the highest index nodes have no arcs.
  const NodeIndex num_nodes = Graphs<Graph>::NodeReservation(*graph_);
  std::vector<int> skip_active_node;

  // Usually SaturateOutgoingArcsFromSource() will saturate all the arcs from
  // the source in one go, and we will loop just once. But in case we can push
  // more than kMaxFlowQuantity out of the source the loop is as follow:
  // - Push up to kMaxFlowQuantity out of the source on the admissible outgoing
  //   arcs. Stop if no flow was pushed.
  // - Compute the current max-flow. This will push some flow back to the
  //   source and render more outgoing arcs from the source not admissible.
  //
  // TODO(user): This may not be the most efficient algorithm if we need to loop
  // many times. An alternative may be to handle the source like the other nodes
  // in the algorithm, initially putting an excess of kMaxFlowQuantity on it,
  // and making the source active like any other node with positive excess. To
  // investigate.
  while (SaturateOutgoingArcsFromSource()) {
    int num_skipped;
    do {
      num_skipped = 0;
      skip_active_node.assign(num_nodes, 0);
      skip_active_node[sink_] = 2;
      skip_active_node[source_] = 2;
      GlobalUpdate();
      while (!IsEmptyActiveNodeContainer()) {
        const NodeIndex node = GetAndRemoveFirstActiveNode();
        if (skip_active_node[node] > 1) {
          if (node != sink_ && node != source_) ++num_skipped;
          continue;
        }
        const NodeIndex old_height = node_potential_[node];
        Discharge(node);

        // The idea behind this is that if a node height augments by more than
        // one, then it is likely to push flow back the way it came. This can
        // lead to very costly loops. A bad case is: source -> n1 -> n2 and n2
        // just recently isolated from the sink. Then n2 will push flow back to
        // n1, and n1 to n2 and so on. The height of each node will increase by
        // steps of two until the height of the source is reached, which can
        // take a long time. If the chain is longer, the situation is even
        // worse. The behavior of this heuristic is related to the Gap
        // heuristic.
        //
        // Note that the global update will fix all such cases efficiently. So
        // the idea is to discharge the active node as much as possible, and
        // then do a global update.
        //
        // We skip a node when this condition was true 2 times to avoid doing a
        // global update too frequently.
        if (node_potential_[node] > old_height + 1) {
          ++skip_active_node[node];
        }
      }
    } while (num_skipped > 0);

    // We use a two-phase algorithm:
    // 1/ Only deal with nodes that can reach the sink. At the end we know the
    //    value of the maximum flow and we have a min-cut.
    // 2/ Call PushFlowExcessBackToSource() to obtain a max-flow. This is
    //    usually a lot faster than the first phase.
    PushFlowExcessBackToSource();
  }
}

template <typename Graph>
void GenericMaxFlow<Graph>::Discharge(const NodeIndex node) {
  SCOPED_TIME_STAT(&stats_);
  const NodeIndex num_nodes = graph_->num_nodes();

  // We cache this as this is a hot-loop and we don't want bound checking.
  FlowQuantity* const node_excess = node_excess_.get();
  NodeHeight* const node_potentials = node_potential_.get();
  ArcIndex* const first_admissible_arc = first_admissible_arc_.get();

  while (true) {
    DCHECK(IsActive(node));
    for (OutgoingOrOppositeIncomingArcIterator it(*graph_, node,
                                                  first_admissible_arc[node]);
         it.Ok(); it.Next()) {
      const ArcIndex arc = it.Index();
      if (IsAdmissible(node, arc, node_potentials)) {
        DCHECK(IsActive(node));
        const NodeIndex head = Head(arc);
        if (node_excess[head] == 0) {
          // The push below will make the node active for sure. Note that we may
          // push the sink_, but that is handled properly in Refine().
          PushActiveNode(head);
        }
        const FlowQuantity delta =
            std::min(node_excess[node], residual_arc_capacity_[arc]);
        FastPushFlow(delta, node, arc, node_excess);
        if (node_excess[node] == 0) {
          first_admissible_arc[node] = arc;  // arc may still be admissible.
          return;
        }
      }
    }
    Relabel(node);

    // This node can no longer reach the sink, skip until
    // PushFlowExcessBackToSource().
    if (node_potentials[node] >= num_nodes) break;
  }
}

template <typename Graph>
void GenericMaxFlow<Graph>::Relabel(NodeIndex node) {
  SCOPED_TIME_STAT(&stats_);
  // Because we use a relaxed version, this is no longer true if the
  // first_admissible_arc_[node] was not actually the first arc!
  // DCHECK(CheckRelabelPrecondition(node));
  NodeHeight min_height = std::numeric_limits<NodeHeight>::max();
  ArcIndex first_admissible_arc = Graph::kNilArc;
  for (OutgoingOrOppositeIncomingArcIterator it(*graph_, node); it.Ok();
       it.Next()) {
    const ArcIndex arc = it.Index();
    if (residual_arc_capacity_[arc] > 0) {
      // Update min_height only for arcs with available capacity.
      NodeHeight head_height = node_potential_[Head(arc)];
      if (head_height < min_height) {
        min_height = head_height;
        first_admissible_arc = arc;

        // We found an admissible arc at the current height, just stop there.
        // This is the true first_admissible_arc_[node].
        if (min_height + 1 == node_potential_[node]) break;
      }
    }
  }
  DCHECK_NE(first_admissible_arc, Graph::kNilArc);
  node_potential_[node] = min_height + 1;

  // Note that after a Relabel(), the loop will continue in Discharge(), and
  // we are sure that all the arcs before first_admissible_arc are not
  // admissible since their height is > min_height.
  first_admissible_arc_[node] = first_admissible_arc;
}

template <typename Graph>
typename Graph::ArcIndex GenericMaxFlow<Graph>::Opposite(ArcIndex arc) const {
  return Graphs<Graph>::OppositeArc(*graph_, arc);
}

template <typename Graph>
bool GenericMaxFlow<Graph>::IsArcDirect(ArcIndex arc) const {
  return IsArcValid(arc) && arc >= 0;
}

template <typename Graph>
bool GenericMaxFlow<Graph>::IsArcValid(ArcIndex arc) const {
  return Graphs<Graph>::IsArcValid(*graph_, arc);
}

template <typename Graph>
template <bool reverse>
void GenericMaxFlow<Graph>::ComputeReachableNodes(
    NodeIndex start, std::vector<NodeIndex>* result) {
  // If start is not a valid node index, it can reach only itself.
  // Note(user): This is needed because source and sink are given independently
  // of the graph and sometimes before it is even constructed.
  const NodeIndex num_nodes = graph_->num_nodes();
  if (start >= num_nodes) {
    result->clear();
    result->push_back(start);
    return;
  }
  bfs_queue_.clear();
  node_in_bfs_queue_.assign(num_nodes, false);

  int queue_index = 0;
  bfs_queue_.push_back(start);
  node_in_bfs_queue_[start] = true;
  while (queue_index != bfs_queue_.size()) {
    const NodeIndex node = bfs_queue_[queue_index];
    ++queue_index;
    for (OutgoingOrOppositeIncomingArcIterator it(*graph_, node); it.Ok();
         it.Next()) {
      const ArcIndex arc = it.Index();
      const NodeIndex head = Head(arc);
      if (node_in_bfs_queue_[head]) continue;
      if (residual_arc_capacity_[reverse ? Opposite(arc) : arc] == 0) continue;
      node_in_bfs_queue_[head] = true;
      bfs_queue_.push_back(head);
    }
  }
  *result = bfs_queue_;
}

template <typename Graph>
FlowModelProto GenericMaxFlow<Graph>::CreateFlowModel() {
  FlowModelProto model;
  model.set_problem_type(FlowModelProto::MAX_FLOW);
  for (int n = 0; n < graph_->num_nodes(); ++n) {
    FlowNodeProto* node = model.add_nodes();
    node->set_id(n);
    if (n == source_) node->set_supply(1);
    if (n == sink_) node->set_supply(-1);
  }
  for (int a = 0; a < graph_->num_arcs(); ++a) {
    FlowArcProto* arc = model.add_arcs();
    arc->set_tail(graph_->Tail(a));
    arc->set_head(graph_->Head(a));
    arc->set_capacity(Capacity(a));
  }
  return model;
}

}  // namespace operations_research

#endif  // OR_TOOLS_GRAPH_GENERIC_MAX_FLOW_H_
