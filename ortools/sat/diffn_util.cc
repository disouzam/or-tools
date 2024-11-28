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

#include "ortools/sat/diffn_util.h"

#include <stddef.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numeric>
#include <optional>
#include <ostream>
#include <set>
#include <sstream>
#include <string>
#include <string_view>
#include <tuple>
#include <utility>
#include <vector>

#include "absl/container/btree_map.h"
#include "absl/container/btree_set.h"
#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/container/inlined_vector.h"
#include "absl/log/check.h"
#include "absl/random/bit_gen_ref.h"
#include "absl/strings/str_cat.h"
#include "absl/types/span.h"
#include "ortools/base/logging.h"
#include "ortools/base/stl_util.h"
#include "ortools/graph/graph.h"
#include "ortools/graph/minimum_spanning_tree.h"
#include "ortools/graph/strongly_connected_components.h"
#include "ortools/sat/integer.h"
#include "ortools/sat/intervals.h"
#include "ortools/sat/util.h"
#include "ortools/util/integer_pq.h"
#include "ortools/util/strong_integers.h"

namespace operations_research {
namespace sat {

bool Rectangle::IsDisjoint(const Rectangle& other) const {
  return x_min >= other.x_max || other.x_min >= x_max || y_min >= other.y_max ||
         other.y_min >= y_max;
}

absl::InlinedVector<Rectangle, 4> Rectangle::RegionDifference(
    const Rectangle& other) const {
  const Rectangle intersect = Intersect(other);
  if (intersect.SizeX() == 0) {
    return {*this};
  }

  //-------------------
  //|   |    4    |   |
  //|   |---------|   |
  //| 1 |  other  | 2 |
  //|   |---------|   |
  //|   |    3    |   |
  //-------------------
  absl::InlinedVector<Rectangle, 4> result;
  if (x_min < intersect.x_min) {
    // Piece 1
    result.push_back({.x_min = x_min,
                      .x_max = intersect.x_min,
                      .y_min = y_min,
                      .y_max = y_max});
  }
  if (x_max > intersect.x_max) {
    // Piece 2
    result.push_back({.x_min = intersect.x_max,
                      .x_max = x_max,
                      .y_min = y_min,
                      .y_max = y_max});
  }
  if (y_min < intersect.y_min) {
    // Piece 3
    result.push_back({.x_min = intersect.x_min,
                      .x_max = intersect.x_max,
                      .y_min = y_min,
                      .y_max = intersect.y_min});
  }
  if (y_max > intersect.y_max) {
    // Piece 4
    result.push_back({.x_min = intersect.x_min,
                      .x_max = intersect.x_max,
                      .y_min = intersect.y_max,
                      .y_max = y_max});
  }

  return result;
}

CompactVectorVector<int> GetOverlappingRectangleComponents(
    absl::Span<const Rectangle> rectangles,
    absl::Span<const int> active_rectangles) {
  if (active_rectangles.empty()) return {};

  std::vector<Rectangle> rectangles_to_process;
  std::vector<int> rectangles_index;
  rectangles_to_process.reserve(active_rectangles.size());
  rectangles_index.reserve(active_rectangles.size());
  for (const int r : active_rectangles) {
    rectangles_to_process.push_back(rectangles[r]);
    rectangles_index.push_back(r);
  }

  std::vector<std::pair<int, int>> intersections =
      FindPartialRectangleIntersectionsAlsoEmpty(rectangles_to_process);
  const int num_intersections = intersections.size();
  intersections.reserve(num_intersections * 2 + 1);
  for (int i = 0; i < num_intersections; ++i) {
    intersections.push_back({intersections[i].second, intersections[i].first});
  }

  CompactVectorVector<int> view;
  view.ResetFromPairs(intersections, /*minimum_num_nodes=*/rectangles.size());
  CompactVectorVector<int> components;
  FindStronglyConnectedComponents(static_cast<int>(rectangles.size()), view,
                                  &components);
  CompactVectorVector<int> result;
  for (int i = 0; i < components.size(); ++i) {
    absl::Span<const int> component = components[i];
    if (component.size() == 1) continue;
    result.Add({});
    for (const int r : component) {
      result.AppendToLastVector(rectangles_index[r]);
    }
  }
  return result;
}

bool ReportEnergyConflict(Rectangle bounding_box, absl::Span<const int> boxes,
                          SchedulingConstraintHelper* x,
                          SchedulingConstraintHelper* y) {
  x->ClearReason();
  y->ClearReason();
  IntegerValue total_energy(0);
  for (const int b : boxes) {
    const IntegerValue x_min = x->ShiftedStartMin(b);
    const IntegerValue x_max = x->ShiftedEndMax(b);
    if (x_min < bounding_box.x_min || x_max > bounding_box.x_max) continue;
    const IntegerValue y_min = y->ShiftedStartMin(b);
    const IntegerValue y_max = y->ShiftedEndMax(b);
    if (y_min < bounding_box.y_min || y_max > bounding_box.y_max) continue;

    x->AddEnergyMinInIntervalReason(b, bounding_box.x_min, bounding_box.x_max);
    y->AddEnergyMinInIntervalReason(b, bounding_box.y_min, bounding_box.y_max);

    x->AddPresenceReason(b);
    y->AddPresenceReason(b);

    total_energy += x->SizeMin(b) * y->SizeMin(b);

    // We abort early if a subset of boxes is enough.
    // TODO(user): Also relax the box if possible.
    if (total_energy > bounding_box.Area()) break;
  }

  CHECK_GT(total_energy, bounding_box.Area());
  x->ImportOtherReasons(*y);
  return x->ReportConflict();
}

bool BoxesAreInEnergyConflict(absl::Span<const Rectangle> rectangles,
                              absl::Span<const IntegerValue> energies,
                              absl::Span<const int> boxes,
                              Rectangle* conflict) {
  // First consider all relevant intervals along the x axis.
  std::vector<IntegerValue> x_starts;
  std::vector<TaskTime> boxes_by_increasing_x_max;
  for (const int b : boxes) {
    x_starts.push_back(rectangles[b].x_min);
    boxes_by_increasing_x_max.push_back({b, rectangles[b].x_max});
  }
  gtl::STLSortAndRemoveDuplicates(&x_starts);
  std::sort(boxes_by_increasing_x_max.begin(), boxes_by_increasing_x_max.end());

  std::vector<IntegerValue> y_starts;
  std::vector<IntegerValue> energy_sum;
  std::vector<TaskTime> boxes_by_increasing_y_max;

  std::vector<std::vector<int>> stripes(x_starts.size());
  for (int i = 0; i < boxes_by_increasing_x_max.size(); ++i) {
    const int b = boxes_by_increasing_x_max[i].task_index;
    const IntegerValue x_min = rectangles[b].x_min;
    const IntegerValue x_max = rectangles[b].x_max;
    for (int j = 0; j < x_starts.size(); ++j) {
      if (x_starts[j] > x_min) break;
      stripes[j].push_back(b);

      // Redo the same on the y coordinate for the current x interval
      // which is [starts[j], x_max].
      y_starts.clear();
      boxes_by_increasing_y_max.clear();
      for (const int b : stripes[j]) {
        y_starts.push_back(rectangles[b].y_min);
        boxes_by_increasing_y_max.push_back({b, rectangles[b].y_max});
      }
      gtl::STLSortAndRemoveDuplicates(&y_starts);
      std::sort(boxes_by_increasing_y_max.begin(),
                boxes_by_increasing_y_max.end());

      const IntegerValue x_size = x_max - x_starts[j];
      energy_sum.assign(y_starts.size(), IntegerValue(0));
      for (int i = 0; i < boxes_by_increasing_y_max.size(); ++i) {
        const int b = boxes_by_increasing_y_max[i].task_index;
        const IntegerValue y_min = rectangles[b].y_min;
        const IntegerValue y_max = rectangles[b].y_max;
        for (int j = 0; j < y_starts.size(); ++j) {
          if (y_starts[j] > y_min) break;
          energy_sum[j] += energies[b];
          if (energy_sum[j] > x_size * (y_max - y_starts[j])) {
            if (conflict != nullptr) {
              *conflict = rectangles[b];
              for (int k = 0; k < i; ++k) {
                const int task_index = boxes_by_increasing_y_max[k].task_index;
                if (rectangles[task_index].y_min >= y_starts[j]) {
                  conflict->GrowToInclude(rectangles[task_index]);
                }
              }
            }
            return true;
          }
        }
      }
    }
  }
  return false;
}

bool AnalyzeIntervals(bool transpose, absl::Span<const int> local_boxes,
                      absl::Span<const Rectangle> rectangles,
                      absl::Span<const IntegerValue> rectangle_energies,
                      IntegerValue* x_threshold, IntegerValue* y_threshold,
                      Rectangle* conflict) {
  // First, we compute the possible x_min values (removing duplicates).
  // We also sort the relevant tasks by their x_max.
  //
  // TODO(user): If the number of unique x_max is smaller than the number of
  // unique x_min, it is better to do it the other way around.
  std::vector<IntegerValue> starts;
  std::vector<TaskTime> task_by_increasing_x_max;
  for (const int t : local_boxes) {
    const IntegerValue x_min =
        transpose ? rectangles[t].y_min : rectangles[t].x_min;
    const IntegerValue x_max =
        transpose ? rectangles[t].y_max : rectangles[t].x_max;
    starts.push_back(x_min);
    task_by_increasing_x_max.push_back({t, x_max});
  }
  gtl::STLSortAndRemoveDuplicates(&starts);

  // Note that for the same end_max, the order change our heuristic to
  // evaluate the max_conflict_height.
  std::sort(task_by_increasing_x_max.begin(), task_by_increasing_x_max.end());

  // The maximum y dimension of a bounding area for which there is a potential
  // conflict.
  IntegerValue max_conflict_height(0);

  // This is currently only used for logging.
  absl::flat_hash_set<std::pair<IntegerValue, IntegerValue>> stripes;

  // All quantities at index j correspond to the interval [starts[j], x_max].
  std::vector<IntegerValue> energies(starts.size(), IntegerValue(0));
  std::vector<IntegerValue> y_mins(starts.size(), kMaxIntegerValue);
  std::vector<IntegerValue> y_maxs(starts.size(), -kMaxIntegerValue);
  std::vector<IntegerValue> energy_at_max_y(starts.size(), IntegerValue(0));
  std::vector<IntegerValue> energy_at_min_y(starts.size(), IntegerValue(0));

  // Sentinel.
  starts.push_back(kMaxIntegerValue);

  // Iterate over all boxes by increasing x_max values.
  int first_j = 0;
  const IntegerValue threshold = transpose ? *y_threshold : *x_threshold;
  for (int i = 0; i < task_by_increasing_x_max.size(); ++i) {
    const int t = task_by_increasing_x_max[i].task_index;

    const IntegerValue energy = rectangle_energies[t];
    IntegerValue x_min = rectangles[t].x_min;
    IntegerValue x_max = rectangles[t].x_max;
    IntegerValue y_min = rectangles[t].y_min;
    IntegerValue y_max = rectangles[t].y_max;
    if (transpose) {
      std::swap(x_min, y_min);
      std::swap(x_max, y_max);
    }

    // Add this box contribution to all the [starts[j], x_max] intervals.
    while (first_j + 1 < starts.size() && x_max - starts[first_j] > threshold) {
      ++first_j;
    }
    for (int j = first_j; starts[j] <= x_min; ++j) {
      const IntegerValue old_energy_at_max = energy_at_max_y[j];
      const IntegerValue old_energy_at_min = energy_at_min_y[j];

      energies[j] += energy;

      const bool is_disjoint = y_min >= y_maxs[j] || y_max <= y_mins[j];

      if (y_min <= y_mins[j]) {
        if (y_min < y_mins[j]) {
          y_mins[j] = y_min;
          energy_at_min_y[j] = energy;
        } else {
          energy_at_min_y[j] += energy;
        }
      }

      if (y_max >= y_maxs[j]) {
        if (y_max > y_maxs[j]) {
          y_maxs[j] = y_max;
          energy_at_max_y[j] = energy;
        } else {
          energy_at_max_y[j] += energy;
        }
      }

      // If the new box is disjoint in y from the ones added so far, there
      // cannot be a new conflict involving this box, so we skip until we add
      // new boxes.
      if (is_disjoint) continue;

      const IntegerValue width = x_max - starts[j];
      IntegerValue conflict_height = CeilRatio(energies[j], width) - 1;
      if (y_max - y_min > conflict_height) continue;
      if (conflict_height >= y_maxs[j] - y_mins[j]) {
        // We have a conflict.
        if (conflict != nullptr) {
          *conflict = rectangles[t];
          for (int k = 0; k < i; ++k) {
            const int task_index = task_by_increasing_x_max[k].task_index;
            const IntegerValue task_x_min = transpose
                                                ? rectangles[task_index].y_min
                                                : rectangles[task_index].x_min;
            if (task_x_min < starts[j]) continue;
            conflict->GrowToInclude(rectangles[task_index]);
          }
        }
        return false;
      }

      // Because we currently do not have a conflict involving the new box, the
      // only way to have one is to remove enough energy to reduce the y domain.
      IntegerValue can_remove = std::min(old_energy_at_min, old_energy_at_max);
      if (old_energy_at_min < old_energy_at_max) {
        if (y_maxs[j] - y_min >=
            CeilRatio(energies[j] - old_energy_at_min, width)) {
          // In this case, we need to remove at least old_energy_at_max to have
          // a conflict.
          can_remove = old_energy_at_max;
        }
      } else if (old_energy_at_max < old_energy_at_min) {
        if (y_max - y_mins[j] >=
            CeilRatio(energies[j] - old_energy_at_max, width)) {
          can_remove = old_energy_at_min;
        }
      }
      conflict_height = CeilRatio(energies[j] - can_remove, width) - 1;

      // If the new box height is above the conflict_height, do not count
      // it now. We only need to consider conflict involving the new box.
      if (y_max - y_min > conflict_height) continue;

      if (VLOG_IS_ON(2)) stripes.insert({starts[j], x_max});
      max_conflict_height = std::max(max_conflict_height, conflict_height);
    }
  }

  VLOG(2) << " num_starts: " << starts.size() - 1 << "/" << local_boxes.size()
          << " conflict_height: " << max_conflict_height
          << " num_stripes:" << stripes.size() << " (<= " << threshold << ")";

  if (transpose) {
    *x_threshold = std::min(*x_threshold, max_conflict_height);
  } else {
    *y_threshold = std::min(*y_threshold, max_conflict_height);
  }
  return true;
}

absl::Span<int> FilterBoxesAndRandomize(
    absl::Span<const Rectangle> cached_rectangles, absl::Span<int> boxes,
    IntegerValue threshold_x, IntegerValue threshold_y,
    absl::BitGenRef random) {
  size_t new_size = 0;
  for (const int b : boxes) {
    const Rectangle& dim = cached_rectangles[b];
    if (dim.x_max - dim.x_min > threshold_x) continue;
    if (dim.y_max - dim.y_min > threshold_y) continue;
    boxes[new_size++] = b;
  }
  if (new_size == 0) return {};
  std::shuffle(&boxes[0], &boxes[0] + new_size, random);
  return {&boxes[0], new_size};
}

absl::Span<int> FilterBoxesThatAreTooLarge(
    absl::Span<const Rectangle> cached_rectangles,
    absl::Span<const IntegerValue> energies, absl::Span<int> boxes) {
  // Sort the boxes by increasing area.
  std::sort(boxes.begin(), boxes.end(), [cached_rectangles](int a, int b) {
    return cached_rectangles[a].Area() < cached_rectangles[b].Area();
  });

  IntegerValue total_energy(0);
  for (const int box : boxes) total_energy += energies[box];

  // Remove all the large boxes until we have one with area smaller than the
  // energy of the boxes below.
  int new_size = boxes.size();
  while (new_size > 0 &&
         cached_rectangles[boxes[new_size - 1]].Area() >= total_energy) {
    --new_size;
    total_energy -= energies[boxes[new_size]];
  }
  return boxes.subspan(0, new_size);
}

void ConstructOverlappingSets(bool already_sorted,
                              std::vector<IndexedInterval>* intervals,
                              std::vector<std::vector<int>>* result) {
  result->clear();
  if (already_sorted) {
    DCHECK(std::is_sorted(intervals->begin(), intervals->end(),
                          IndexedInterval::ComparatorByStart()));
  } else {
    std::sort(intervals->begin(), intervals->end(),
              IndexedInterval::ComparatorByStart());
  }
  IntegerValue min_end_in_set = kMaxIntegerValue;
  intervals->push_back({-1, kMaxIntegerValue, kMaxIntegerValue});  // Sentinel.
  const int size = intervals->size();

  // We do a line sweep. The "current" subset crossing the "line" at
  // (time, time + 1) will be in (*intervals)[start_index, end_index) at the end
  // of the loop block.
  int start_index = 0;
  for (int end_index = 0; end_index < size;) {
    const IntegerValue time = (*intervals)[end_index].start;

    // First, if there is some deletion, we will push the "old" set to the
    // result before updating it. Otherwise, we will have a superset later, so
    // we just continue for now.
    if (min_end_in_set <= time) {
      result->push_back({});
      min_end_in_set = kMaxIntegerValue;
      for (int i = start_index; i < end_index; ++i) {
        result->back().push_back((*intervals)[i].index);
        if ((*intervals)[i].end <= time) {
          std::swap((*intervals)[start_index++], (*intervals)[i]);
        } else {
          min_end_in_set = std::min(min_end_in_set, (*intervals)[i].end);
        }
      }

      // Do not output subset of size one.
      if (result->back().size() == 1) result->pop_back();
    }

    // Add all the new intervals starting exactly at "time".
    do {
      min_end_in_set = std::min(min_end_in_set, (*intervals)[end_index].end);
      ++end_index;
    } while (end_index < size && (*intervals)[end_index].start == time);
  }
}

void GetOverlappingIntervalComponents(
    std::vector<IndexedInterval>* intervals,
    std::vector<std::vector<int>>* components) {
  components->clear();
  if (intervals->empty()) return;
  if (intervals->size() == 1) {
    components->push_back({intervals->front().index});
    return;
  }

  // For correctness, ComparatorByStart is enough, but in unit tests we want to
  // verify this function against another implementation, and fully defined
  // sorting with tie-breaking makes that much easier.
  // If that becomes a performance bottleneck:
  // - One may want to sort the list outside of this function, and simply
  //   have this function DCHECK that it's sorted by start.
  // - One may use stable_sort() with ComparatorByStart().
  std::sort(intervals->begin(), intervals->end(),
            IndexedInterval::ComparatorByStartThenEndThenIndex());

  IntegerValue end_max_so_far = (*intervals)[0].end;
  components->push_back({(*intervals)[0].index});
  for (int i = 1; i < intervals->size(); ++i) {
    const IndexedInterval& interval = (*intervals)[i];
    if (interval.start >= end_max_so_far) {
      components->push_back({interval.index});
    } else {
      components->back().push_back(interval.index);
    }
    end_max_so_far = std::max(end_max_so_far, interval.end);
  }
}

std::vector<int> GetIntervalArticulationPoints(
    std::vector<IndexedInterval>* intervals) {
  std::vector<int> articulation_points;
  if (intervals->size() < 3) return articulation_points;  // Empty.
  if (DEBUG_MODE) {
    for (const IndexedInterval& interval : *intervals) {
      DCHECK_LT(interval.start, interval.end);
    }
  }

  std::sort(intervals->begin(), intervals->end(),
            IndexedInterval::ComparatorByStart());

  IntegerValue end_max_so_far = (*intervals)[0].end;
  int index_of_max = 0;
  IntegerValue prev_end_max = kMinIntegerValue;  // Initialized as a sentinel.
  for (int i = 1; i < intervals->size(); ++i) {
    const IndexedInterval& interval = (*intervals)[i];
    if (interval.start >= end_max_so_far) {
      // New connected component.
      end_max_so_far = interval.end;
      index_of_max = i;
      prev_end_max = kMinIntegerValue;
      continue;
    }
    // Still the same connected component. Was the previous "max" an
    // articulation point ?
    if (prev_end_max != kMinIntegerValue && interval.start >= prev_end_max) {
      // We might be re-inserting the same articulation point: guard against it.
      if (articulation_points.empty() ||
          articulation_points.back() != index_of_max) {
        articulation_points.push_back(index_of_max);
      }
    }
    // Update the max end.
    if (interval.end > end_max_so_far) {
      prev_end_max = end_max_so_far;
      end_max_so_far = interval.end;
      index_of_max = i;
    } else if (interval.end > prev_end_max) {
      prev_end_max = interval.end;
    }
  }
  // Convert articulation point indices to IndexedInterval.index.
  for (int& index : articulation_points) index = (*intervals)[index].index;
  return articulation_points;
}

namespace {
bool IsZeroOrPowerOfTwo(int value) { return (value & (value - 1)) == 0; }

void AppendPairwiseRestriction(const ItemForPairwiseRestriction& item1,
                               const ItemForPairwiseRestriction& item2,
                               std::vector<PairwiseRestriction>* result) {
  const int state =
      // box1 can be left of box2.
      (item1.x.end_min <= item2.x.start_max) +
      // box1 can be right of box2.
      2 * (item2.x.end_min <= item1.x.start_max) +
      // box1 can be below box2.
      4 * (item1.y.end_min <= item2.y.start_max) +
      // box1 can be up of box2.
      8 * (item2.y.end_min <= item1.y.start_max);

  if (!IsZeroOrPowerOfTwo(state)) {
    return;
  }

  switch (state) {
    case 0:
      // Conflict. The two boxes must overlap in both dimensions.
      result->push_back(
          {.first_index = item1.index,
           .second_index = item2.index,
           .type = PairwiseRestriction::PairwiseRestrictionType::CONFLICT});
      break;
    case 1:
      // box2 can only be after box1 on x.
      if (item1.x.end_min > item2.x.start_min ||
          item2.x.start_max < item1.x.end_max) {
        result->push_back({.first_index = item1.index,
                           .second_index = item2.index,
                           .type = PairwiseRestriction::
                               PairwiseRestrictionType::FIRST_LEFT_OF_SECOND});
      }
      break;
    case 2:  // box1 an only be after box2 on x.
      if (item2.x.end_min > item1.x.start_min ||
          item1.x.start_max < item2.x.end_max) {
        result->push_back({.first_index = item1.index,
                           .second_index = item2.index,
                           .type = PairwiseRestriction::
                               PairwiseRestrictionType::FIRST_RIGHT_OF_SECOND});
      }
      break;
    case 4:
      // box2 an only be after box1 on y.
      if (item1.y.end_min > item2.y.start_min ||
          item2.y.start_max < item1.y.end_max) {
        result->push_back({.first_index = item1.index,
                           .second_index = item2.index,
                           .type = PairwiseRestriction::
                               PairwiseRestrictionType::FIRST_BELOW_SECOND});
      }
      break;
    case 8:  // box1 an only be after box2 on y.
      if (item2.y.end_min > item1.y.start_min ||
          item1.y.start_max < item2.y.end_max) {
        result->push_back({.first_index = item1.index,
                           .second_index = item2.index,
                           .type = PairwiseRestriction::
                               PairwiseRestrictionType::FIRST_ABOVE_SECOND});
      }
      break;
    default:
      break;
  }
}
}  // namespace

void AppendPairwiseRestrictions(
    absl::Span<const ItemForPairwiseRestriction> items,
    std::vector<PairwiseRestriction>* result) {
  for (int i1 = 0; i1 + 1 < items.size(); ++i1) {
    for (int i2 = i1 + 1; i2 < items.size(); ++i2) {
      AppendPairwiseRestriction(items[i1], items[i2], result);
    }
  }
}

void AppendPairwiseRestrictions(
    absl::Span<const ItemForPairwiseRestriction> items,
    absl::Span<const ItemForPairwiseRestriction> other_items,
    std::vector<PairwiseRestriction>* result) {
  for (int i1 = 0; i1 < items.size(); ++i1) {
    for (int i2 = 0; i2 < other_items.size(); ++i2) {
      AppendPairwiseRestriction(items[i1], other_items[i2], result);
    }
  }
}

void CapacityProfile::Clear() {
  events_.clear();
  num_rectangles_added_ = 0;
}

void CapacityProfile::AddRectangle(IntegerValue x_min, IntegerValue x_max,
                                   IntegerValue y_min, IntegerValue y_max) {
  DCHECK_LE(x_min, x_max);
  if (x_min == x_max) return;

  events_.push_back(
      StartRectangleEvent(num_rectangles_added_, x_min, y_min, y_max));
  events_.push_back(EndRectangleEvent(num_rectangles_added_, x_max));
  ++num_rectangles_added_;
}

void CapacityProfile::AddMandatoryConsumption(IntegerValue x_min,
                                              IntegerValue x_max,
                                              IntegerValue y_height) {
  DCHECK_LE(x_min, x_max);
  if (x_min == x_max) return;

  events_.push_back(ChangeMandatoryProfileEvent(x_min, y_height));
  events_.push_back(ChangeMandatoryProfileEvent(x_max, -y_height));
}

void CapacityProfile::BuildResidualCapacityProfile(
    std::vector<CapacityProfile::Rectangle>* result) {
  std::sort(events_.begin(), events_.end());
  IntegerPriorityQueue<QueueElement> min_pq(num_rectangles_added_);
  IntegerPriorityQueue<QueueElement> max_pq(num_rectangles_added_);
  IntegerValue mandatory_capacity(0);

  result->clear();

  result->push_back({kMinIntegerValue, IntegerValue(0)});

  for (int i = 0; i < events_.size();) {
    const IntegerValue current_time = events_[i].time;
    for (; i < events_.size(); ++i) {
      const Event& event = events_[i];
      if (event.time != current_time) break;

      switch (events_[i].type) {
        case START_RECTANGLE: {
          min_pq.Add({event.index, -event.y_min});
          max_pq.Add({event.index, event.y_max});
          break;
        }
        case END_RECTANGLE: {
          min_pq.Remove(event.index);
          max_pq.Remove(event.index);
          break;
        }
        case CHANGE_MANDATORY_PROFILE: {
          mandatory_capacity += event.y_min;
          break;
        }
      }
    }

    DCHECK(!max_pq.IsEmpty() || mandatory_capacity == 0);
    const IntegerValue new_height =
        max_pq.IsEmpty()
            ? IntegerValue(0)
            : max_pq.Top().value + min_pq.Top().value - mandatory_capacity;
    if (new_height != result->back().height) {
      result->push_back({current_time, new_height});
    }
  }
}

IntegerValue CapacityProfile::GetBoundingArea() {
  std::sort(events_.begin(), events_.end());
  IntegerPriorityQueue<QueueElement> min_pq(num_rectangles_added_);
  IntegerPriorityQueue<QueueElement> max_pq(num_rectangles_added_);

  IntegerValue area(0);
  IntegerValue previous_time = kMinIntegerValue;
  IntegerValue previous_height(0);

  for (int i = 0; i < events_.size();) {
    const IntegerValue current_time = events_[i].time;
    for (; i < events_.size(); ++i) {
      const Event& event = events_[i];
      if (event.time != current_time) break;

      switch (event.type) {
        case START_RECTANGLE: {
          min_pq.Add({event.index, -event.y_min});
          max_pq.Add({event.index, event.y_max});
          break;
        }
        case END_RECTANGLE: {
          min_pq.Remove(event.index);
          max_pq.Remove(event.index);
          break;
        }
        case CHANGE_MANDATORY_PROFILE: {
          break;
        }
      }
    }
    const IntegerValue new_height =
        max_pq.IsEmpty() ? IntegerValue(0)
                         : max_pq.Top().value + min_pq.Top().value;
    if (previous_height != 0) {
      area += previous_height * (current_time - previous_time);
    }
    previous_time = current_time;
    previous_height = new_height;
  }
  return area;
}

IntegerValue Smallest1DIntersection(IntegerValue range_min,
                                    IntegerValue range_max, IntegerValue size,
                                    IntegerValue interval_min,
                                    IntegerValue interval_max) {
  // If the item is on the left of the range, we get the intersection between
  // [range_min, range_min + size] and [interval_min, interval_max].
  const IntegerValue overlap_on_left =
      std::min(range_min + size, interval_max) -
      std::max(range_min, interval_min);

  // If the item is on the right of the range, we get the intersection between
  // [range_max - size, range_max] and [interval_min, interval_max].
  const IntegerValue overlap_on_right =
      std::min(range_max, interval_max) -
      std::max(range_max - size, interval_min);

  return std::max(IntegerValue(0), std::min(overlap_on_left, overlap_on_right));
}

ProbingRectangle::ProbingRectangle(
    const std::vector<RectangleInRange>& intervals)
    : intervals_(intervals) {
  minimum_energy_ = 0;
  if (intervals_.empty()) {
    return;
  }
  interval_points_sorted_by_x_.reserve(intervals_.size() * 4 + 2);
  interval_points_sorted_by_y_.reserve(intervals_.size() * 4 + 2);

  Rectangle bounding_box = {.x_min = std::numeric_limits<IntegerValue>::max(),
                            .x_max = std::numeric_limits<IntegerValue>::min(),
                            .y_min = std::numeric_limits<IntegerValue>::max(),
                            .y_max = std::numeric_limits<IntegerValue>::min()};

  for (int i = 0; i < intervals_.size(); ++i) {
    const RectangleInRange& interval = intervals_[i];
    minimum_energy_ += interval.x_size * interval.y_size;

    bounding_box.x_min =
        std::min(bounding_box.x_min, interval.bounding_area.x_min);
    bounding_box.x_max =
        std::max(bounding_box.x_max, interval.bounding_area.x_max);
    bounding_box.y_min =
        std::min(bounding_box.y_min, interval.bounding_area.y_min);
    bounding_box.y_max =
        std::max(bounding_box.y_max, interval.bounding_area.y_max);

    interval_points_sorted_by_x_.push_back({interval.bounding_area.x_min, i});
    interval_points_sorted_by_x_.push_back(
        {interval.bounding_area.x_min + interval.x_size, i});
    interval_points_sorted_by_x_.push_back(
        {interval.bounding_area.x_max - interval.x_size, i});
    interval_points_sorted_by_x_.push_back({interval.bounding_area.x_max, i});

    interval_points_sorted_by_y_.push_back({interval.bounding_area.y_min, i});
    interval_points_sorted_by_y_.push_back(
        {interval.bounding_area.y_min + interval.y_size, i});
    interval_points_sorted_by_y_.push_back(
        {interval.bounding_area.y_max - interval.y_size, i});
    interval_points_sorted_by_y_.push_back({interval.bounding_area.y_max, i});
  }

  full_energy_ = minimum_energy_;
  // Add four bogus points in the extremities so we can delegate setting up all
  // internal state to Shrink().
  interval_points_sorted_by_x_.push_back({bounding_box.x_min - 1, -1});
  interval_points_sorted_by_x_.push_back({bounding_box.x_max + 1, -1});
  interval_points_sorted_by_y_.push_back({bounding_box.y_min - 1, -1});
  interval_points_sorted_by_y_.push_back({bounding_box.y_max + 1, -1});

  auto comparator = [](const IntervalPoint& a, const IntervalPoint& b) {
    return std::tie(a.value, a.index) < std::tie(b.value, b.index);
  };
  gtl::STLSortAndRemoveDuplicates(&interval_points_sorted_by_x_, comparator);
  gtl::STLSortAndRemoveDuplicates(&interval_points_sorted_by_y_, comparator);

  grouped_intervals_sorted_by_x_.reserve(interval_points_sorted_by_x_.size());
  grouped_intervals_sorted_by_y_.reserve(interval_points_sorted_by_y_.size());
  int i = 0;
  while (i < interval_points_sorted_by_x_.size()) {
    int idx_begin = i;
    while (i < interval_points_sorted_by_x_.size() &&
           interval_points_sorted_by_x_[i].value ==
               interval_points_sorted_by_x_[idx_begin].value) {
      i++;
    }
    grouped_intervals_sorted_by_x_.push_back(
        {interval_points_sorted_by_x_[idx_begin].value,
         absl::Span<IntervalPoint>(interval_points_sorted_by_x_)
             .subspan(idx_begin, i - idx_begin)});
  }

  i = 0;
  while (i < interval_points_sorted_by_y_.size()) {
    int idx_begin = i;
    while (i < interval_points_sorted_by_y_.size() &&
           interval_points_sorted_by_y_[i].value ==
               interval_points_sorted_by_y_[idx_begin].value) {
      i++;
    }
    grouped_intervals_sorted_by_y_.push_back(
        {interval_points_sorted_by_y_[idx_begin].value,
         absl::Span<IntervalPoint>(interval_points_sorted_by_y_)
             .subspan(idx_begin, i - idx_begin)});
  }

  Reset();
}

void ProbingRectangle::Reset() {
  indexes_[Edge::LEFT] = 0;
  indexes_[Edge::RIGHT] = grouped_intervals_sorted_by_x_.size() - 1;
  indexes_[Edge::BOTTOM] = 0;
  indexes_[Edge::TOP] = grouped_intervals_sorted_by_y_.size() - 1;

  next_indexes_[Edge::LEFT] = 1;
  next_indexes_[Edge::RIGHT] = grouped_intervals_sorted_by_x_.size() - 2;
  next_indexes_[Edge::BOTTOM] = 1;
  next_indexes_[Edge::TOP] = grouped_intervals_sorted_by_y_.size() - 2;

  minimum_energy_ = full_energy_;
  ranges_touching_both_boundaries_[0].clear();
  ranges_touching_both_boundaries_[1].clear();

  for (int i = 0; i < 4; ++i) {
    corner_count_[i] = 0;
    intersect_length_[i] = 0;
    cached_delta_energy_[i] = 0;
  }

  // Remove the four bogus points we added.
  Shrink(Edge::LEFT);
  Shrink(Edge::BOTTOM);
  Shrink(Edge::RIGHT);
  Shrink(Edge::TOP);
}

Rectangle ProbingRectangle::GetCurrentRectangle() const {
  return {
      .x_min = grouped_intervals_sorted_by_x_[indexes_[Edge::LEFT]].coordinate,
      .x_max = grouped_intervals_sorted_by_x_[indexes_[Edge::RIGHT]].coordinate,
      .y_min =
          grouped_intervals_sorted_by_y_[indexes_[Edge::BOTTOM]].coordinate,
      .y_max = grouped_intervals_sorted_by_y_[indexes_[Edge::TOP]].coordinate};
}

namespace {
// Intersects `rectangle` with the largest rectangle that must intersect with
// the range in some way. To visualize this largest rectangle, imagine the four
// possible extreme positions for the item in range (the four corners). This
// rectangle is the one defined by the interior points of each position. This
// don't use IsDisjoint() because it also works when the rectangle would be
// malformed (it's bounding box less than twice the size).
bool CanConsumeEnergy(const Rectangle& rectangle,
                      const RectangleInRange& item) {
  return (rectangle.x_max > item.bounding_area.x_max - item.x_size) &&
         (rectangle.y_max > item.bounding_area.y_max - item.y_size) &&
         (rectangle.x_min < item.bounding_area.x_min + item.x_size) &&
         (rectangle.y_min < item.bounding_area.y_min + item.y_size);
}

std::array<bool, 4> GetPossibleEdgeIntersection(const Rectangle& rectangle,
                                                const RectangleInRange& range) {
  std::array<bool, 4> result;
  using Edge = ProbingRectangle::Edge;
  result[Edge::LEFT] = rectangle.x_min >= range.bounding_area.x_min;
  result[Edge::BOTTOM] = rectangle.y_min >= range.bounding_area.y_min;
  result[Edge::RIGHT] = rectangle.x_max <= range.bounding_area.x_max;
  result[Edge::TOP] = rectangle.y_max <= range.bounding_area.y_max;
  return result;
}

}  // namespace

// NOMUTANTS -- This is a sanity check, it is hard to corrupt the data in an
// unit test to check it will fail.
void ProbingRectangle::ValidateInvariants() const {
  const Rectangle current_rectangle = GetCurrentRectangle();

  IntegerValue intersect_length[4] = {0, 0, 0, 0};
  IntegerValue corner_count[4] = {0, 0, 0, 0};
  IntegerValue energy = 0;
  CHECK_LE(next_indexes_[Edge::LEFT], indexes_[Edge::RIGHT]);
  CHECK_LE(next_indexes_[Edge::BOTTOM], indexes_[Edge::TOP]);
  CHECK_GE(next_indexes_[Edge::TOP], indexes_[Edge::BOTTOM]);
  CHECK_GE(next_indexes_[Edge::RIGHT], indexes_[Edge::LEFT]);

  for (int interval_idx = 0; interval_idx < intervals_.size(); interval_idx++) {
    const RectangleInRange& range = intervals_[interval_idx];

    const Rectangle min_intersect =
        range.GetMinimumIntersection(current_rectangle);
    CHECK_LE(min_intersect.SizeX(), range.x_size);
    CHECK_LE(min_intersect.SizeY(), range.y_size);
    energy += min_intersect.Area();

    std::array<bool, 4> touching_boundary = {false, false, false, false};
    CHECK_EQ(CanConsumeEnergy(current_rectangle, range) &&
                 current_rectangle.Area() != 0,
             range.GetMinimumIntersectionArea(current_rectangle) != 0);
    if (CanConsumeEnergy(current_rectangle, range)) {
      touching_boundary = GetPossibleEdgeIntersection(current_rectangle, range);
    }

    CHECK_EQ(
        touching_boundary[Edge::LEFT] && touching_boundary[Edge::RIGHT],
        ranges_touching_both_boundaries_[Direction::LEFT_AND_RIGHT].contains(
            interval_idx));
    CHECK_EQ(
        touching_boundary[Edge::TOP] && touching_boundary[Edge::BOTTOM],
        ranges_touching_both_boundaries_[Direction::TOP_AND_BOTTOM].contains(
            interval_idx));

    if (touching_boundary[Edge::LEFT] && !touching_boundary[Edge::RIGHT]) {
      intersect_length[Edge::LEFT] += Smallest1DIntersection(
          range.bounding_area.y_min, range.bounding_area.y_max, range.y_size,
          current_rectangle.y_min, current_rectangle.y_max);
    }

    if (touching_boundary[Edge::RIGHT] && !touching_boundary[Edge::LEFT]) {
      intersect_length[Edge::RIGHT] += Smallest1DIntersection(
          range.bounding_area.y_min, range.bounding_area.y_max, range.y_size,
          current_rectangle.y_min, current_rectangle.y_max);
    }

    if (touching_boundary[Edge::TOP] && !touching_boundary[Edge::BOTTOM]) {
      intersect_length[Edge::TOP] += Smallest1DIntersection(
          range.bounding_area.x_min, range.bounding_area.x_max, range.x_size,
          current_rectangle.x_min, current_rectangle.x_max);
    }

    if (touching_boundary[Edge::BOTTOM] && !touching_boundary[Edge::TOP]) {
      intersect_length[Edge::BOTTOM] += Smallest1DIntersection(
          range.bounding_area.x_min, range.bounding_area.x_max, range.x_size,
          current_rectangle.x_min, current_rectangle.x_max);
    }

    if ((touching_boundary[Edge::LEFT] && touching_boundary[Edge::RIGHT]) ||
        (touching_boundary[Edge::TOP] && touching_boundary[Edge::BOTTOM])) {
      // We account separately for the problematic items that touches both
      // sides.
      continue;
    }
    if (touching_boundary[Edge::BOTTOM] && touching_boundary[Edge::LEFT]) {
      corner_count[RectangleInRange::Corner::BOTTOM_LEFT]++;
    }
    if (touching_boundary[Edge::BOTTOM] && touching_boundary[Edge::RIGHT]) {
      corner_count[RectangleInRange::Corner::BOTTOM_RIGHT]++;
    }
    if (touching_boundary[Edge::TOP] && touching_boundary[Edge::LEFT]) {
      corner_count[RectangleInRange::Corner::TOP_LEFT]++;
    }
    if (touching_boundary[Edge::TOP] && touching_boundary[Edge::RIGHT]) {
      corner_count[RectangleInRange::Corner::TOP_RIGHT]++;
    }
  }

  CHECK_EQ(energy, minimum_energy_);
  for (int i = 0; i < 4; i++) {
    CHECK_EQ(intersect_length[i], intersect_length_[i]);
    CHECK_EQ(corner_count[i], corner_count_[i]);
  }
}

namespace {

struct EdgeInfo {
  using Edge = ProbingRectangle::Edge;
  using Direction = ProbingRectangle::Direction;
  using Corner = RectangleInRange::Corner;

  Edge opposite_edge;

  struct OrthogonalInfo {
    Edge edge;
    Corner adjacent_corner;
  };

  Direction shrink_direction;
  Direction orthogonal_shrink_direction;
  // Lower coordinate one first (ie., BOTTOM before TOP, LEFT before RIGHT).
  OrthogonalInfo orthogonal_edges[2];
};

struct EdgeInfoHolder {
  using Edge = ProbingRectangle::Edge;
  using Direction = ProbingRectangle::Direction;
  using Corner = RectangleInRange::Corner;

  static constexpr EdgeInfo kLeft = {
      .opposite_edge = Edge::RIGHT,
      .shrink_direction = Direction::LEFT_AND_RIGHT,
      .orthogonal_shrink_direction = Direction::TOP_AND_BOTTOM,
      .orthogonal_edges = {
          {.edge = Edge::BOTTOM, .adjacent_corner = Corner::BOTTOM_LEFT},
          {.edge = Edge::TOP, .adjacent_corner = Corner::TOP_LEFT}}};

  static constexpr EdgeInfo kRight = {
      .opposite_edge = Edge::LEFT,
      .shrink_direction = Direction::LEFT_AND_RIGHT,
      .orthogonal_shrink_direction = Direction::TOP_AND_BOTTOM,
      .orthogonal_edges = {
          {.edge = Edge::BOTTOM, .adjacent_corner = Corner::BOTTOM_RIGHT},
          {.edge = Edge::TOP, .adjacent_corner = Corner::TOP_RIGHT}}};
  static constexpr EdgeInfo kBottom = {
      .opposite_edge = Edge::TOP,
      .shrink_direction = Direction::TOP_AND_BOTTOM,
      .orthogonal_shrink_direction = Direction::LEFT_AND_RIGHT,
      .orthogonal_edges = {
          {.edge = Edge::LEFT, .adjacent_corner = Corner::BOTTOM_LEFT},
          {.edge = Edge::RIGHT, .adjacent_corner = Corner::BOTTOM_RIGHT}}};
  static constexpr EdgeInfo kTop = {
      .opposite_edge = Edge::BOTTOM,
      .shrink_direction = Direction::TOP_AND_BOTTOM,
      .orthogonal_shrink_direction = Direction::LEFT_AND_RIGHT,
      .orthogonal_edges = {
          {.edge = Edge::LEFT, .adjacent_corner = Corner::TOP_LEFT},
          {.edge = Edge::RIGHT, .adjacent_corner = Corner::TOP_RIGHT}}};
};

constexpr const EdgeInfo& GetEdgeInfo(ProbingRectangle::Edge edge) {
  using Edge = ProbingRectangle::Edge;
  switch (edge) {
    case Edge::LEFT:
      return EdgeInfoHolder::kLeft;
    case Edge::RIGHT:
      return EdgeInfoHolder::kRight;
    case Edge::BOTTOM:
      return EdgeInfoHolder::kBottom;
    case Edge::TOP:
      return EdgeInfoHolder::kTop;
  }
}

IntegerValue GetSmallest1DIntersection(ProbingRectangle::Direction direction,
                                       const RectangleInRange& range,
                                       const Rectangle& rectangle) {
  switch (direction) {
    case ProbingRectangle::Direction::LEFT_AND_RIGHT:
      return Smallest1DIntersection(range.bounding_area.x_min,
                                    range.bounding_area.x_max, range.x_size,
                                    rectangle.x_min, rectangle.x_max);
    case ProbingRectangle::Direction::TOP_AND_BOTTOM:
      return Smallest1DIntersection(range.bounding_area.y_min,
                                    range.bounding_area.y_max, range.y_size,
                                    rectangle.y_min, rectangle.y_max);
  }
}

}  // namespace

template <ProbingRectangle::Edge edge>
void ProbingRectangle::ShrinkImpl() {
  constexpr EdgeInfo e = GetEdgeInfo(edge);

  bool update_next_index[4] = {false, false, false, false};
  update_next_index[edge] = true;

  IntegerValue step_1d_size;
  minimum_energy_ -= GetShrinkDeltaEnergy(edge);
  const std::vector<PointsForCoordinate>& sorted_intervals =
      e.shrink_direction == Direction::LEFT_AND_RIGHT
          ? grouped_intervals_sorted_by_x_
          : grouped_intervals_sorted_by_y_;

  const Rectangle prev_rectangle = GetCurrentRectangle();
  indexes_[edge] = next_indexes_[edge];
  const Rectangle current_rectangle = GetCurrentRectangle();

  switch (edge) {
    case Edge::LEFT:
      step_1d_size = current_rectangle.x_min - prev_rectangle.x_min;
      next_indexes_[edge] =
          std::min(indexes_[edge] + 1, indexes_[e.opposite_edge]);
      next_indexes_[e.opposite_edge] =
          std::max(indexes_[edge], next_indexes_[e.opposite_edge]);
      break;
    case Edge::BOTTOM:
      step_1d_size = current_rectangle.y_min - prev_rectangle.y_min;
      next_indexes_[edge] =
          std::min(indexes_[edge] + 1, indexes_[e.opposite_edge]);
      next_indexes_[e.opposite_edge] =
          std::max(indexes_[edge], next_indexes_[e.opposite_edge]);
      break;
    case Edge::RIGHT:
      step_1d_size = prev_rectangle.x_max - current_rectangle.x_max;
      next_indexes_[edge] =
          std::max(indexes_[edge] - 1, indexes_[e.opposite_edge]);
      next_indexes_[e.opposite_edge] =
          std::min(indexes_[edge], next_indexes_[e.opposite_edge]);
      break;
    case Edge::TOP:
      step_1d_size = prev_rectangle.y_max - current_rectangle.y_max;
      next_indexes_[edge] =
          std::max(indexes_[edge] - 1, indexes_[e.opposite_edge]);
      next_indexes_[e.opposite_edge] =
          std::min(indexes_[edge], next_indexes_[e.opposite_edge]);
      break;
  }

  absl::Span<ProbingRectangle::IntervalPoint> items_touching_coordinate =
      sorted_intervals[indexes_[edge]].items_touching_coordinate;

  IntegerValue delta_corner_count[4] = {0, 0, 0, 0};
  for (const auto& item : items_touching_coordinate) {
    const RectangleInRange& range = intervals_[item.index];
    if (!CanConsumeEnergy(prev_rectangle, range)) {
      // This item is out of our area of interest, skip.
      continue;
    }

    const std::array<bool, 4> touching_boundary_before =
        GetPossibleEdgeIntersection(prev_rectangle, range);
    const std::array<bool, 4> touching_boundary_after =
        CanConsumeEnergy(current_rectangle, range)
            ? GetPossibleEdgeIntersection(current_rectangle, range)
            : std::array<bool, 4>({false, false, false, false});

    bool remove_corner[4] = {false, false, false, false};
    auto erase_item = [this, &prev_rectangle, &range, &touching_boundary_before,
                       &remove_corner](Edge edge_to_erase) {
      const EdgeInfo& erase_info = GetEdgeInfo(edge_to_erase);
      intersect_length_[edge_to_erase] -= GetSmallest1DIntersection(
          erase_info.orthogonal_shrink_direction, range, prev_rectangle);

      if (touching_boundary_before[erase_info.orthogonal_edges[0].edge] &&
          touching_boundary_before[erase_info.orthogonal_edges[1].edge]) {
        // Ignore touching both corners
        return;
      }
      for (const auto [orthogonal_edge, corner] : erase_info.orthogonal_edges) {
        if (touching_boundary_before[orthogonal_edge]) {
          remove_corner[corner] = true;
        }
      }
    };

    if (touching_boundary_after[edge] && !touching_boundary_before[edge]) {
      if (touching_boundary_before[e.opposite_edge]) {
        ranges_touching_both_boundaries_[e.shrink_direction].insert(item.index);
        erase_item(e.opposite_edge);
      } else {
        // Do the opposite of remove_item().
        intersect_length_[edge] += GetSmallest1DIntersection(
            e.orthogonal_shrink_direction, range, prev_rectangle);
        // Update the corner count unless it is touching both.
        if (!touching_boundary_before[e.orthogonal_edges[0].edge] ||
            !touching_boundary_before[e.orthogonal_edges[1].edge]) {
          for (const auto [orthogonal_edge, corner] : e.orthogonal_edges) {
            if (touching_boundary_before[orthogonal_edge]) {
              delta_corner_count[corner]++;
            }
          }
        }
      }
    }

    for (int i = 0; i < 4; i++) {
      const Edge edge_to_update = (Edge)i;
      const EdgeInfo& info = GetEdgeInfo(edge_to_update);
      const bool remove_edge = touching_boundary_before[edge_to_update] &&
                               !touching_boundary_after[edge_to_update];
      if (!remove_edge) {
        continue;
      }

      update_next_index[edge_to_update] = true;

      if (touching_boundary_before[info.opposite_edge]) {
        ranges_touching_both_boundaries_[info.shrink_direction].erase(
            item.index);
      } else {
        erase_item(edge_to_update);
      }
    }

    for (int i = 0; i < 4; i++) {
      corner_count_[i] -= remove_corner[i];
    }
  }

  // Update the intersection length for items touching both sides.
  for (const int idx : ranges_touching_both_boundaries_[e.shrink_direction]) {
    const RectangleInRange& range = intervals_[idx];
    const std::array<bool, 2> touching_corner =
        (e.shrink_direction == Direction::LEFT_AND_RIGHT)
            ? std::array<bool, 2>(
                  {current_rectangle.y_min >= range.bounding_area.y_min,
                   current_rectangle.y_max <= range.bounding_area.y_max})
            : std::array<bool, 2>(
                  {current_rectangle.x_min >= range.bounding_area.x_min,
                   current_rectangle.x_max <= range.bounding_area.x_max});
    if (touching_corner[0] == touching_corner[1]) {
      // Either it is not touching neither corners (so no length to update) or
      // it is touching both corners, which will be handled by the "both
      // sides" code and should not contribute to intersect_length_.
      continue;
    }

    const IntegerValue incr =
        GetSmallest1DIntersection(e.shrink_direction, range, prev_rectangle) -
        GetSmallest1DIntersection(e.shrink_direction, range, current_rectangle);
    for (int i = 0; i < 2; i++) {
      if (touching_corner[i]) {
        intersect_length_[e.orthogonal_edges[i].edge] -= incr;
      }
    }
  }

  for (const auto [orthogonal_edge, corner] : e.orthogonal_edges) {
    intersect_length_[orthogonal_edge] -= corner_count_[corner] * step_1d_size;
  }

  for (int i = 0; i < 4; i++) {
    corner_count_[i] += delta_corner_count[i];
  }

  auto points_consume_energy =
      [this,
       &current_rectangle](absl::Span<ProbingRectangle::IntervalPoint> points) {
        for (const auto& item : points) {
          const RectangleInRange& range = intervals_[item.index];
          if (CanConsumeEnergy(current_rectangle, range)) {
            return true;
          }
        }
        return false;
      };
  if (update_next_index[Edge::LEFT]) {
    for (; next_indexes_[Edge::LEFT] < indexes_[Edge::RIGHT];
         ++next_indexes_[Edge::LEFT]) {
      if (points_consume_energy(
              grouped_intervals_sorted_by_x_[next_indexes_[Edge::LEFT]]
                  .items_touching_coordinate)) {
        break;
      }
    }
  }
  if (update_next_index[Edge::BOTTOM]) {
    for (; next_indexes_[Edge::BOTTOM] < indexes_[Edge::TOP];
         ++next_indexes_[Edge::BOTTOM]) {
      if (points_consume_energy(
              grouped_intervals_sorted_by_y_[next_indexes_[Edge::BOTTOM]]
                  .items_touching_coordinate)) {
        break;
      }
    }
  }
  if (update_next_index[Edge::RIGHT]) {
    for (; next_indexes_[Edge::RIGHT] > indexes_[Edge::LEFT];
         --next_indexes_[Edge::RIGHT]) {
      if (points_consume_energy(
              grouped_intervals_sorted_by_x_[next_indexes_[Edge::RIGHT]]
                  .items_touching_coordinate)) {
        break;
      }
    }
  }
  if (update_next_index[Edge::TOP]) {
    for (; next_indexes_[Edge::TOP] > indexes_[Edge::BOTTOM];
         --next_indexes_[Edge::TOP]) {
      if (points_consume_energy(
              grouped_intervals_sorted_by_y_[next_indexes_[Edge::TOP]]
                  .items_touching_coordinate)) {
        break;
      }
    }
  }

  probe_area_ = current_rectangle.Area();
  CacheShrinkDeltaEnergy(0);
  CacheShrinkDeltaEnergy(1);
}

void ProbingRectangle::Shrink(Edge edge) {
  switch (edge) {
    case Edge::LEFT:
      ShrinkImpl<Edge::LEFT>();
      return;
    case Edge::BOTTOM:
      ShrinkImpl<Edge::BOTTOM>();
      return;
    case Edge::RIGHT:
      ShrinkImpl<Edge::RIGHT>();
      return;
    case Edge::TOP:
      ShrinkImpl<Edge::TOP>();
      return;
  }
}

IntegerValue ProbingRectangle::GetShrinkDeltaArea(Edge edge) const {
  const Rectangle current_rectangle = GetCurrentRectangle();
  const std::vector<PointsForCoordinate>& sorted_intervals =
      (edge == Edge::LEFT || edge == Edge::RIGHT)
          ? grouped_intervals_sorted_by_x_
          : grouped_intervals_sorted_by_y_;
  const IntegerValue coordinate =
      sorted_intervals[next_indexes_[edge]].coordinate;
  switch (edge) {
    case Edge::LEFT:
      return (coordinate - current_rectangle.x_min) * current_rectangle.SizeY();
    case Edge::BOTTOM:
      return (coordinate - current_rectangle.y_min) * current_rectangle.SizeX();
    case Edge::RIGHT:
      return (current_rectangle.x_max - coordinate) * current_rectangle.SizeY();
    case Edge::TOP:
      return (current_rectangle.y_max - coordinate) * current_rectangle.SizeX();
  }
}

void ProbingRectangle::CacheShrinkDeltaEnergy(int dimension) {
  const Rectangle current_rectangle = GetCurrentRectangle();
  Rectangle next_rectangle_up = current_rectangle;
  Rectangle next_rectangle_down = current_rectangle;
  IntegerValue step_1d_size_up, step_1d_size_down;
  IntegerValue units_crossed_up, units_crossed_down;
  IntegerValue* delta_energy_up_ptr;
  IntegerValue* delta_energy_down_ptr;

  if (dimension == 0) {
    // CanShrink(Edge::RIGHT) and CanShrink(Edge::LEFT) are equivalent
    if (!CanShrink(Edge::LEFT)) {
      cached_delta_energy_[Edge::LEFT] = 0;
      cached_delta_energy_[Edge::RIGHT] = 0;
      return;
    }

    next_rectangle_up.x_min =
        grouped_intervals_sorted_by_x_[next_indexes_[Edge::LEFT]].coordinate;
    next_rectangle_down.x_max =
        grouped_intervals_sorted_by_x_[next_indexes_[Edge::RIGHT]].coordinate;

    step_1d_size_up = next_rectangle_up.x_min - current_rectangle.x_min;
    step_1d_size_down = current_rectangle.x_max - next_rectangle_down.x_max;
    units_crossed_up = intersect_length_[Edge::LEFT];
    units_crossed_down = intersect_length_[Edge::RIGHT];
    delta_energy_up_ptr = &cached_delta_energy_[Edge::LEFT];
    delta_energy_down_ptr = &cached_delta_energy_[Edge::RIGHT];
  } else {
    if (!CanShrink(Edge::TOP)) {
      cached_delta_energy_[Edge::BOTTOM] = 0;
      cached_delta_energy_[Edge::TOP] = 0;
      return;
    }

    next_rectangle_up.y_min =
        grouped_intervals_sorted_by_y_[next_indexes_[Edge::BOTTOM]].coordinate;
    next_rectangle_down.y_max =
        grouped_intervals_sorted_by_y_[next_indexes_[Edge::TOP]].coordinate;

    step_1d_size_up = next_rectangle_up.y_min - current_rectangle.y_min;
    step_1d_size_down = current_rectangle.y_max - next_rectangle_down.y_max;
    units_crossed_up = intersect_length_[Edge::BOTTOM];
    units_crossed_down = intersect_length_[Edge::TOP];
    delta_energy_up_ptr = &cached_delta_energy_[Edge::BOTTOM];
    delta_energy_down_ptr = &cached_delta_energy_[Edge::TOP];
  }
  IntegerValue delta_energy_up = 0;
  IntegerValue delta_energy_down = 0;

  // Note that the non-deterministic iteration order is fine here.
  for (const int idx : ranges_touching_both_boundaries_[dimension]) {
    const RectangleInRange& range = intervals_[idx];
    const IntegerValue curr_x = Smallest1DIntersection(
        range.bounding_area.x_min, range.bounding_area.x_max, range.x_size,
        current_rectangle.x_min, current_rectangle.x_max);
    const IntegerValue curr_y = Smallest1DIntersection(
        range.bounding_area.y_min, range.bounding_area.y_max, range.y_size,
        current_rectangle.y_min, current_rectangle.y_max);
    const IntegerValue curr = curr_x * curr_y;
    delta_energy_up += curr;
    delta_energy_down += curr;

    if (dimension == 0) {
      const IntegerValue up_x = Smallest1DIntersection(
          range.bounding_area.x_min, range.bounding_area.x_max, range.x_size,
          next_rectangle_up.x_min, next_rectangle_up.x_max);
      const IntegerValue down_x = Smallest1DIntersection(
          range.bounding_area.x_min, range.bounding_area.x_max, range.x_size,
          next_rectangle_down.x_min, next_rectangle_down.x_max);

      delta_energy_up -= curr_y * up_x;
      delta_energy_down -= curr_y * down_x;
    } else {
      const IntegerValue up_y = Smallest1DIntersection(
          range.bounding_area.y_min, range.bounding_area.y_max, range.y_size,
          next_rectangle_up.y_min, next_rectangle_up.y_max);
      const IntegerValue down_y = Smallest1DIntersection(
          range.bounding_area.y_min, range.bounding_area.y_max, range.y_size,
          next_rectangle_down.y_min, next_rectangle_down.y_max);

      delta_energy_up -= curr_x * up_y;
      delta_energy_down -= curr_x * down_y;
    }
  }
  delta_energy_up += units_crossed_up * step_1d_size_up;
  delta_energy_down += units_crossed_down * step_1d_size_down;
  *delta_energy_up_ptr = delta_energy_up;
  *delta_energy_down_ptr = delta_energy_down;
}

bool ProbingRectangle::CanShrink(Edge edge) const {
  switch (edge) {
    case Edge::LEFT:
    case Edge::RIGHT:
      return (next_indexes_[Edge::RIGHT] > indexes_[Edge::LEFT]);
    case Edge::BOTTOM:
    case Edge::TOP:
      return (indexes_[Edge::TOP] > next_indexes_[Edge::BOTTOM]);
  }
}

namespace {
std::vector<double> GetExpTable() {
  std::vector<double> table(101);
  for (int i = 0; i <= 100; ++i) {
    table[i] = std::exp(-(i - 50) / 5.0);
  }
  return table;
}

}  // namespace

FindRectanglesResult FindRectanglesWithEnergyConflictMC(
    const std::vector<RectangleInRange>& intervals, absl::BitGenRef random,
    double temperature, double candidate_energy_usage_factor) {
  FindRectanglesResult result;
  ProbingRectangle ranges(intervals);

  static const std::vector<double>* cached_probabilities =
      new std::vector<double>(GetExpTable());

  const double inv_temp = 1.0 / temperature;
  absl::InlinedVector<ProbingRectangle::Edge, 4> candidates;
  absl::InlinedVector<IntegerValue, 4> energy_deltas;
  absl::InlinedVector<double, 4> weights;
  while (!ranges.IsMinimal()) {
    const IntegerValue rect_area = ranges.GetCurrentRectangleArea();
    const IntegerValue min_energy = ranges.GetMinimumEnergy();
    if (min_energy > rect_area) {
      result.conflicts.push_back(ranges.GetCurrentRectangle());
    } else if (min_energy.value() >
               candidate_energy_usage_factor * rect_area.value()) {
      result.candidates.push_back(ranges.GetCurrentRectangle());
    }
    if (min_energy == 0) {
      break;
    }
    candidates.clear();
    energy_deltas.clear();

    for (int border_idx = 0; border_idx < 4; ++border_idx) {
      const ProbingRectangle::Edge border =
          static_cast<ProbingRectangle::Edge>(border_idx);
      if (!ranges.CanShrink(border)) {
        continue;
      }
      candidates.push_back(border);
      const IntegerValue delta_area = ranges.GetShrinkDeltaArea(border);
      const IntegerValue delta_energy = ranges.GetShrinkDeltaEnergy(border);
      energy_deltas.push_back(delta_energy - delta_area);
    }
    const IntegerValue min_energy_delta =
        *std::min_element(energy_deltas.begin(), energy_deltas.end());
    weights.clear();
    for (const IntegerValue delta_slack : energy_deltas) {
      const int64_t table_lookup =
          std::max((int64_t)0,
                   std::min((int64_t)((delta_slack - min_energy_delta).value() *
                                          5 * inv_temp +
                                      50),
                            (int64_t)100));
      weights.push_back((*cached_probabilities)[table_lookup]);
    }
    // Pick a change with a probability proportional to exp(- delta_E / Temp)
    ranges.Shrink(candidates[WeightedPick(weights, random)]);
  }
  if (ranges.GetMinimumEnergy() > ranges.GetCurrentRectangleArea()) {
    result.conflicts.push_back(ranges.GetCurrentRectangle());
  }
  return result;
}

std::string RenderDot(std::optional<Rectangle> bb,
                      absl::Span<const Rectangle> solution,
                      std::string_view extra_dot_payload) {
  const std::vector<std::string> colors = {"#0000ff80", "#ee00ee80",
                                           "#ff000080", "#eeee0080",
                                           "#00ff0080", "#00eeee80"};
  std::stringstream ss;
  ss << "digraph {\n";
  ss << "  graph [ bgcolor=lightgray ]\n";
  ss << "  node [style=filled shape=box]\n";
  if (bb.has_value()) {
    ss << "  bb [fillcolor=\"grey\" pos=\"" << 2 * bb->x_min + bb->SizeX()
       << "," << 2 * bb->y_min + bb->SizeY() << "!\" width=" << 2 * bb->SizeX()
       << " height=" << 2 * bb->SizeY() << "]\n";
  }
  for (int i = 0; i < solution.size(); ++i) {
    ss << "  " << i << " [fillcolor=\"" << colors[i % colors.size()]
       << "\" pos=\"" << 2 * solution[i].x_min + solution[i].SizeX() << ","
       << 2 * solution[i].y_min + solution[i].SizeY()
       << "!\" width=" << 2 * solution[i].SizeX()
       << " height=" << 2 * solution[i].SizeY() << "]\n";
  }
  ss << extra_dot_payload;
  ss << "}\n";
  return ss.str();
}

std::vector<Rectangle> FindEmptySpaces(
    const Rectangle& bounding_box, std::vector<Rectangle> ocupied_rectangles) {
  // Sorting is not necessary for correctness but makes it faster.
  std::sort(ocupied_rectangles.begin(), ocupied_rectangles.end(),
            [](const Rectangle& a, const Rectangle& b) {
              return std::tuple(a.x_min, -a.x_max, a.y_min) <
                     std::tuple(b.x_min, -b.x_max, b.y_min);
            });
  return PavedRegionDifference({bounding_box}, ocupied_rectangles);
}

std::vector<Rectangle> PavedRegionDifference(
    std::vector<Rectangle> original_region,
    absl::Span<const Rectangle> area_to_remove) {
  std::vector<Rectangle> new_area_to_cover;
  for (const Rectangle& rectangle : area_to_remove) {
    new_area_to_cover.clear();
    for (const Rectangle& r : original_region) {
      const auto& new_rectangles = r.RegionDifference(rectangle);
      new_area_to_cover.insert(new_area_to_cover.end(), new_rectangles.begin(),
                               new_rectangles.end());
    }
    original_region.swap(new_area_to_cover);
    if (original_region.empty()) break;
  }
  return original_region;
}

std::vector<std::pair<int, int>> FindPartialRectangleIntersections(
    absl::Span<const Rectangle> rectangles) {
  // We are going to use a sweep line algorithm to find the intersections.
  // First, we sort the rectangles by their x coordinates, then consider a sweep
  // line that goes from the left to the right.
  enum class Type {
    kEnd,
    kBegin,
  };
  struct RectangleHorizontalEdge {
    IntegerValue x_coordinate;
    int index;
    Type type;
  };
  std::vector<RectangleHorizontalEdge> edges;
  edges.reserve(rectangles.size() * 2);
  for (int i = 0; i < rectangles.size(); ++i) {
    DCHECK_GT(rectangles[i].SizeX(), 0);
    DCHECK_GT(rectangles[i].SizeY(), 0);
    edges.push_back({.x_coordinate = rectangles[i].x_min,
                     .index = i,
                     .type = Type::kBegin});
    edges.push_back(
        {.x_coordinate = rectangles[i].x_max, .index = i, .type = Type::kEnd});
  }

  std::sort(
      edges.begin(), edges.end(),
      [](const RectangleHorizontalEdge& a, const RectangleHorizontalEdge& b) {
        return std::tuple(a.x_coordinate, a.type, a.index) <
               std::tuple(b.x_coordinate, b.type, b.index);
      });

  // Current y-coordinate intervals that are intersecting the sweep line.
  // Note that the interval_set only contains disjoint intervals.
  struct Interval {
    IntegerValue start;
    mutable IntegerValue end;

    // This interval will always be a "subpart" of the full y-span of the
    // initial rectangle with given index.
    mutable int index;

    bool operator<(const Interval& other) const { return start < other.start; }

    std::string to_string() const {
      return absl::StrCat("[", start.value(), ",", end.value(), "](", index,
                          ")");
    }
  };
  std::set<Interval> interval_set;

  // The finer point of this algorithm is deciding what to keep on our interval
  // list when we find two rectangles that intersect. Since we don't want to
  // lose track of any rectangle that might intersect another one, we keep in
  // the intervals the pieces that has the largest x_max: ie., we make sure that
  // we will not remove an interval that is still intersecting some rectangles.
  absl::btree_set<std::pair<int, int>> arcs;
  int current_index = 0;
  while (current_index < edges.size()) {
    const IntegerValue x_pos = edges[current_index].x_coordinate;
    // First check all the rectangles that end at this x_pos and remove them
    // from the interval list.
    for (; current_index < edges.size() &&
           edges[current_index].x_coordinate == x_pos &&
           edges[current_index].type == Type::kEnd;
         ++current_index) {
      const int index_to_remove = edges[current_index].index;
      const Rectangle r = rectangles[index_to_remove];

      // Loop over all intervals intersecting [r.y_min, r.y_max] and remove the
      // one with the given index.
      const Interval for_lookup = {r.y_min, r.y_max, index_to_remove};
      auto it = interval_set.lower_bound(for_lookup);
      while (it != interval_set.end()) {
        // Subtle: we need to find the next interval here because the current
        // one might be invalidated by the erase.
        auto current = it++;
        if (current->start >= r.y_max) break;
        if (current->index == index_to_remove) {
          interval_set.erase(current);
        }
      }
    }
    // Now add the rectangles that start at this x_pos and potentially detect
    // any overlap.
    for (; current_index < edges.size() &&
           edges[current_index].x_coordinate == x_pos &&
           edges[current_index].type == Type::kBegin;
         ++current_index) {
      const int cur_index = edges[current_index].index;
      const Rectangle r = rectangles[cur_index];

      // We need to insert a new interval, we will split it according to the
      // interval that it overlaps with.
      Interval to_insert = {r.y_min, r.y_max, cur_index};
      auto it = interval_set.lower_bound(to_insert);
      if (it != interval_set.begin()) --it;
      while (it != interval_set.end()) {
        auto current = it++;
        if (to_insert.start == to_insert.end) break;
        if (current->end <= to_insert.start) continue;
        if (current->start >= to_insert.end) break;

        // We have an intersection.
        if (to_insert.index == cur_index) {
          arcs.insert({std::min(to_insert.index, current->index),
                       std::max(to_insert.index, current->index)});
        }

        // It can be composed of up to 3 part:
        // Where part1 and/or part2 could be empty.
        //   current     [                  ]
        //   to_insert            [                   ]
        //   current     [                            ]
        //   to_insert            [         ]
        //   current              [         ]
        //   to_insert   [                            ]
        //   current              [                   ]
        //   to_insert   [                  ]
        //               | part1  |  part2  | part 3  |
        const int first_index =
            current->start < to_insert.start ? current->index : to_insert.index;
        const int middle_index =
            rectangles[to_insert.index].x_max < rectangles[current->index].x_max
                ? current->index
                : to_insert.index;
        const int last_index =
            current->end > to_insert.end ? current->index : to_insert.index;

        IntegerValue points[4];
        points[0] = current->start;
        points[1] = current->end;
        points[2] = to_insert.start;
        points[3] = to_insert.end;
        std::sort(points, points + 4);

        // There is always a middle part because the intersection is not empty.
        CHECK_LT(points[1], points[2]);

        if (points[0] == points[1]) {
          // Rewrite current as part 2.
          current->end = points[2];
          current->index = middle_index;
        } else {
          if (points[0] == current->start) {
            // Rewrite current as part 1.
            current->end = points[1];
            current->index = first_index;

            // Insert a new part2 interval.
            // As an optimization, if middle_index == last_index, we just
            // merge part2 and 3 in the next to_insert.
            if (middle_index == last_index) {
              to_insert.start = points[1];
              to_insert.end = points[3];
              to_insert.index = last_index;
              continue;
            }
            interval_set.insert({points[1], points[2], middle_index});
          } else {
            CHECK_EQ(points[1], current->start);

            // Rewrite current as part 2.
            current->end = points[2];
            current->index = middle_index;

            // Insert a new part1 interval.
            interval_set.insert({points[0], points[1], first_index});
          }
        }

        // To_insert is the final part (it can be empty).
        to_insert.start = points[2];
        to_insert.end = points[3];
        to_insert.index = last_index;
      }

      if (to_insert.start < to_insert.end) {
        interval_set.insert(to_insert);
      }
    }
  }

  // At this point we have a set of arcs that doesn't contain all of the
  // pairwise intersections, but is not either a minimal spanning tree either
  // since it has cycles. The good property is that it does have enough arcs to
  // cover all the rectangles, so we can use it to build a proper MST.
  ::util::ReverseArcListGraph<> graph;
  std::vector<int> arc_indexes;
  absl::flat_hash_map<int, std::pair<int, int>> pair_by_arc_index;
  for (const auto& [a, b] : arcs) {
    pair_by_arc_index[arc_indexes.size()] = {a, b};
    arc_indexes.push_back(graph.AddArc(a, b));
  }
  const std::vector<int> mst_arc_indices =
      BuildKruskalMinimumSpanningTreeFromSortedArcs(graph, arc_indexes);
  std::vector<std::pair<int, int>> result;
  for (const int arc_index : mst_arc_indices) {
    const auto& [a, b] = pair_by_arc_index[arc_index];
    result.push_back({a, b});
  }
  return result;
}

std::vector<std::pair<int, int>> FindPartialRectangleIntersectionsAlsoEmpty(
    absl::Span<const Rectangle> rectangles) {
  auto first_index_no_area_it = std::find_if(
      rectangles.begin(), rectangles.end(), [](const Rectangle& r) {
        DCHECK_GE(r.SizeX(), 0);
        DCHECK_GE(r.SizeY(), 0);
        return r.SizeX() == 0 || r.SizeY() == 0;
      });
  if (first_index_no_area_it == rectangles.end()) {
    // Avoid copying, all rectangles have non-zero area.
    return FindPartialRectangleIntersections(rectangles);
  }

  // Now we need to do the boring code of special-casing all the different cases
  // of rectangles with zero area. We still want to use the N log N algorithm
  // for the subset of the input with non-zero area.
  std::vector<Rectangle> rectangles_with_area, horizontal_lines, vertical_lines,
      points;
  std::vector<int> rectangles_with_area_indexes, horizontal_lines_indexes,
      vertical_lines_indexes, points_indexes;
  rectangles_with_area.reserve(rectangles.size());
  rectangles_with_area_indexes.reserve(rectangles.size());
  rectangles_with_area.insert(rectangles_with_area.end(), rectangles.begin(),
                              first_index_no_area_it);
  rectangles_with_area_indexes.resize(rectangles_with_area.size());
  std::iota(rectangles_with_area_indexes.begin(),
            rectangles_with_area_indexes.end(), 0);

  for (int i = first_index_no_area_it - rectangles.begin();
       i < rectangles.size(); ++i) {
    if (rectangles[i].SizeX() > 0 && rectangles[i].SizeY() > 0) {
      rectangles_with_area.push_back(rectangles[i]);
      rectangles_with_area_indexes.push_back(i);
    } else if (rectangles[i].SizeX() > 0) {
      horizontal_lines.push_back(rectangles[i]);
      horizontal_lines_indexes.push_back(i);
    } else if (rectangles[i].SizeY() > 0) {
      vertical_lines.push_back(rectangles[i]);
      vertical_lines_indexes.push_back(i);
    } else {
      points.push_back(rectangles[i]);
      points_indexes.push_back(i);
    }
  }

  // Handle rectangles intersecting rectangles using the sweep line algorithm.
  std::vector<std::pair<int, int>> arcs =
      FindPartialRectangleIntersections(rectangles_with_area);
  for (std::pair<int, int>& arc : arcs) {
    arc.first = rectangles_with_area_indexes[arc.first];
    arc.second = rectangles_with_area_indexes[arc.second];
  }

  // Handle rectangles intersecting non-rectangles.
  for (int i = 0; i < rectangles_with_area.size(); ++i) {
    const int index = rectangles_with_area_indexes[i];
    const Rectangle& r = rectangles_with_area[i];
    for (int j = 0; j < vertical_lines.size(); ++j) {
      const int vertical_line_index = vertical_lines_indexes[j];
      const Rectangle& vertical_line = vertical_lines[j];
      if (!r.IsDisjoint(vertical_line)) {
        arcs.push_back({index, vertical_line_index});
      }
    }
    for (int j = 0; j < horizontal_lines.size(); ++j) {
      const int horizontal_line_index = horizontal_lines_indexes[j];
      const Rectangle& horizontal_line = horizontal_lines[j];
      if (!r.IsDisjoint(horizontal_line)) {
        arcs.push_back({index, horizontal_line_index});
      }
    }
    for (int j = 0; j < points.size(); ++j) {
      const int point_index = points_indexes[j];
      const Rectangle& point = points[j];
      if (!r.IsDisjoint(point)) {
        arcs.push_back({index, point_index});
      }
    }
  }

  // Finally handle vertical lines intersecting horizontal lines.
  for (int i = 0; i < horizontal_lines.size(); ++i) {
    const int index = horizontal_lines_indexes[i];
    const Rectangle& r = horizontal_lines[i];
    for (int j = 0; j < vertical_lines.size(); ++j) {
      const int vertical_line_index = vertical_lines_indexes[j];
      const Rectangle& vertical_line = vertical_lines[j];
      if (!r.IsDisjoint(vertical_line)) {
        arcs.push_back({index, vertical_line_index});
      }
    }
  }

  // Now make our graph a minimal spanning tree again.
  ::util::ReverseArcListGraph<> graph;
  std::vector<int> arc_indexes;
  absl::flat_hash_map<int, std::pair<int, int>> pair_by_arc_index;
  for (const auto& [a, b] : arcs) {
    pair_by_arc_index[arc_indexes.size()] = {a, b};
    arc_indexes.push_back(graph.AddArc(a, b));
  }
  const std::vector<int> mst_arc_indices =
      BuildKruskalMinimumSpanningTreeFromSortedArcs(graph, arc_indexes);
  std::vector<std::pair<int, int>> result;
  for (const int arc_index : mst_arc_indices) {
    const auto& [a, b] = pair_by_arc_index[arc_index];
    result.push_back({a, b});
  }
  return result;
}

}  // namespace sat
}  // namespace operations_research
