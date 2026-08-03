#ifndef TWO_UAV_ATSP_H
#define TWO_UAV_ATSP_H

#include <vector>

// Solve the open asymmetric TSP starting at matrix node 0.  Returned node
// indices are zero-based task nodes (matrix nodes 1..N).
bool solveOpenAtsp(const std::vector<std::vector<double>> &cost_matrix,
                   std::vector<int> &order);

#endif  // TWO_UAV_ATSP_H
