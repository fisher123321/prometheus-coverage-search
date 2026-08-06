#include "two_uav_atsp.h"

#include <algorithm>
#include <cmath>
#include <limits>

bool solveOpenAtsp(const std::vector<std::vector<double>> &cost_matrix,
                   std::vector<int> &order) {
    order.clear();
    if (cost_matrix.empty()) return true;
    const int dimension = static_cast<int>(cost_matrix.size());
    if (dimension == 1) return true;
    if (dimension > 6) return false;  // node 0 plus at most five leased clusters
    for (const auto &row : cost_matrix) {
        if (static_cast<int>(row.size()) != dimension) return false;
    }

    const int task_count = dimension - 1;
    const int masks = 1 << task_count;
    const double inf = std::numeric_limits<double>::infinity();
    std::vector<std::vector<double>> cost(masks, std::vector<double>(task_count, inf));
    std::vector<std::vector<int>> parent(masks, std::vector<int>(task_count, -1));
    for (int task = 0; task < task_count; ++task) {
        const double edge = cost_matrix[0][task + 1];
        if (std::isfinite(edge) && edge >= 0.0) cost[1 << task][task] = edge;
    }
    for (int mask = 1; mask < masks; ++mask) {
        for (int last = 0; last < task_count; ++last) {
            if (!(mask & (1 << last)) || !std::isfinite(cost[mask][last])) continue;
            for (int next = 0; next < task_count; ++next) {
                if (mask & (1 << next)) continue;
                const double edge = cost_matrix[last + 1][next + 1];
                if (!std::isfinite(edge) || edge < 0.0) continue;
                const int next_mask = mask | (1 << next);
                const double candidate = cost[mask][last] + edge;
                if (candidate < cost[next_mask][next]) {
                    cost[next_mask][next] = candidate;
                    parent[next_mask][next] = last;
                }
            }
        }
    }

    const int full_mask = masks - 1;
    int last = -1;
    for (int task = 0; task < task_count; ++task) {
        if (last < 0 || cost[full_mask][task] < cost[full_mask][last]) last = task;
    }
    if (last < 0 || !std::isfinite(cost[full_mask][last])) return false;
    for (int mask = full_mask; last >= 0;) {
        order.push_back(last);
        const int previous = parent[mask][last];
        mask &= ~(1 << last);
        last = previous;
    }
    std::reverse(order.begin(), order.end());
    return static_cast<int>(order.size()) == task_count;
}
