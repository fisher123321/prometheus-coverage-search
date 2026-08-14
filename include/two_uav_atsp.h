#ifndef TWO_UAV_ATSP_H
#define TWO_UAV_ATSP_H

#include <algorithm>
#include <cmath>
#include <vector>

// Solve the open asymmetric TSP starting at matrix node 0.  Returned node
// indices are zero-based task nodes (matrix nodes 1..N).
bool solveOpenAtsp(const std::vector<std::vector<double>> &cost_matrix,
                   std::vector<int> &order);

inline bool shouldSwitchAtspGoal(double incumbent_length, double proposed_length,
                                 double min_distance, double min_ratio) {
    if (!std::isfinite(incumbent_length) || !std::isfinite(proposed_length)) return false;
    const double improvement = incumbent_length - proposed_length;
    return improvement >= min_distance &&
           improvement / std::max(0.10, incumbent_length) >= min_ratio;
}

#endif  // TWO_UAV_ATSP_H
