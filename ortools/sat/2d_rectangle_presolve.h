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

#ifndef OR_TOOLS_SAT_2D_RECTANGLE_PRESOLVE_H_
#define OR_TOOLS_SAT_2D_RECTANGLE_PRESOLVE_H_

#include <tuple>
#include <utility>
#include <vector>

#include "absl/algorithm/container.h"
#include "absl/container/flat_hash_map.h"
#include "absl/container/inlined_vector.h"
#include "absl/types/span.h"
#include "ortools/sat/diffn_util.h"
#include "ortools/sat/integer.h"

namespace operations_research {
namespace sat {

// Given a set of fixed boxes and a set of boxes that are not yet
// fixed (but attributed a range), look for a more optimal set of fixed
// boxes that are equivalent to the initial set of fixed boxes. This
// uses "equivalent" in the sense that a placement of the non-fixed boxes will
// be non-overlapping with all other boxes if and only if it was with the
// original set of fixed boxes too.
bool PresolveFixed2dRectangles(
    absl::Span<const RectangleInRange> non_fixed_boxes,
    std::vector<Rectangle>* fixed_boxes);

// Given a set of non-overlapping rectangles split in two groups, mandatory and
// optional, try to build a set of as few non-overlapping rectangles as
// possible defining a region R that satisfy:
//   - R \subset (mandatory \union optional);
//   - mandatory \subset R.
//
// The function updates the set of `mandatory_rectangles` with `R` and
// `optional_rectangles` with  `optional_rectangles \setdiff R`. It returns
// true if the `mandatory_rectangles` was updated.
bool ReduceNumberofBoxes(std::vector<Rectangle>* mandatory_rectangles,
                         std::vector<Rectangle>* optional_rectangles);

enum EdgePosition { TOP = 0, RIGHT = 1, BOTTOM = 2, LEFT = 3 };

template <typename Sink>
void AbslStringify(Sink& sink, EdgePosition e) {
  switch (e) {
    case EdgePosition::TOP:
      sink.Append("TOP");
      break;
    case EdgePosition::RIGHT:
      sink.Append("RIGHT");
      break;
    case EdgePosition::BOTTOM:
      sink.Append("BOTTOM");
      break;
    case EdgePosition::LEFT:
      sink.Append("LEFT");
      break;
  }
}

// Given a set of non-overlapping rectangles, precompute a data-structure that
// allow for each rectangle to find the adjacent rectangle along an edge.
//
// Note that it only consider adjacent rectangles whose segments has a
// intersection of non-zero size. In particular, rectangles as following are not
// considered adjacent:
//
// ********
// ********
// ********
// ********
//         +++++++++
//         +++++++++
//         +++++++++
//         +++++++++
//
// Precondition: All rectangles must be disjoint.
class Neighbours {
 public:
  class CompareClockwise {
   public:
    explicit CompareClockwise(EdgePosition edge) : edge_(edge) {}

    bool operator()(const Rectangle& a, const Rectangle& b) const {
      switch (edge_) {
        case EdgePosition::BOTTOM:
          return std::tie(a.x_min, a.x_max) > std::tie(b.x_min, b.x_max);
        case EdgePosition::TOP:
          return std::tie(a.x_min, a.x_max) < std::tie(b.x_min, b.x_max);
        case EdgePosition::LEFT:
          return std::tie(a.y_min, a.y_max) < std::tie(b.y_min, b.y_max);
        case EdgePosition::RIGHT:
          return std::tie(a.y_min, a.y_max) > std::tie(b.y_min, b.y_max);
      }
    }
    EdgePosition edge_;
  };

  explicit Neighbours(
      absl::Span<const Rectangle> rectangles,
      absl::Span<const std::tuple<int, EdgePosition, int>> neighbors)
      : size_(rectangles.size()) {
    for (const auto& [box_index, edge, neighbor] : neighbors) {
      neighbors_[edge][box_index].push_back(neighbor);
    }
    for (int edge = 0; edge < 4; ++edge) {
      for (auto& [box_index, neighbors] : neighbors_[edge]) {
        absl::c_sort(neighbors, [&rectangles, edge](int a, int b) {
          return CompareClockwise(static_cast<EdgePosition>(edge))(
              rectangles[a], rectangles[b]);
        });
      }
    }
  }

  int NumRectangles() const { return size_; }

  // Neighbors are sorted in the clockwise order.
  absl::Span<const int> GetSortedNeighbors(int rectangle_index,
                                           EdgePosition edge) const {
    if (auto it = neighbors_[edge].find(rectangle_index);
        it != neighbors_[edge].end()) {
      return it->second;
    } else {
      return {};
    }
  }

 private:
  absl::flat_hash_map<int, absl::InlinedVector<int, 3>> neighbors_[4];
  int size_;
};

Neighbours BuildNeighboursGraph(absl::Span<const Rectangle> rectangles);

std::vector<std::vector<int>> SplitInConnectedComponents(
    const Neighbours& neighbours);

// Generally, given a set of non-overlapping rectangles and a path that doesn't
// cross itself, the path can be cut into segments that touch only one single
// rectangle in the interior of the region delimited by the path. This struct
// holds a path cut into such segments. In particular, for the contour of an
// union of rectangles, the path is a subset of the union of all the rectangle's
// edges.
struct ShapePath {
  // The two vectors should have exactly the same size.
  std::vector<std::pair<IntegerValue, IntegerValue>> step_points;
  // touching_box_index[i] contains the index of the unique interior rectangle
  // touching the segment step_points[i]->step_points[(i+1)%size].
  std::vector<int> touching_box_index;
};

// Returns a path delimiting a boundary of the union of a set of rectangles. It
// should work for both the exterior boundary and the boundaries of the holes
// inside the union. The path will start on `starting_point` and follow the
// boundary on clockwise order.
//
// `starting_point` should be a point in the boundary and `starting_box_index`
// the index of a rectangle with one edge containing `starting_point`.
//
// The resulting `path` satisfy:
// - path.step_points.front() == path.step_points.back() == starting_point
// - path.touching_box_index.front() == path.touching_box_index.back() ==
//                                   == starting_box_index
//
ShapePath TraceBoundary(
    const std::pair<IntegerValue, IntegerValue>& starting_step_point,
    int starting_box_index, absl::Span<const Rectangle> rectangles,
    const Neighbours& neighbours);

}  // namespace sat
}  // namespace operations_research

#endif  // OR_TOOLS_SAT_2D_RECTANGLE_PRESOLVE_H_
