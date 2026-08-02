#include "two_uav_coverage_search.h"

// ============================================================
// Astar2D 实现 - 2D A*路径搜索（借鉴FALCON pathfinding）
// ============================================================
void Astar2D::init(CoverageMap *map) {
    map_ = map;
    search_timeout_ms_ = 250.0;
    last_failure_reason_ = ASTAR_OK;
    cout << GREEN << "[Astar2D] init. Grid: " << map_->grid_size_(0)
         << "x" << map_->grid_size_(1) << TAIL << endl;
}

int Astar2D::toAddress2D(int x, int y) {
    return x * map_->grid_size_(1) + y;
}

double Astar2D::heuristic(const Eigen::Vector2i &from, const Eigen::Vector2i &to) {
    return sqrt((double)((from(0) - to(0)) * (from(0) - to(0)) +
                         (from(1) - to(1)) * (from(1) - to(1))));
}

bool Astar2D::isWalkable(int x, int y) {
    if (x < 0 || x >= map_->grid_size_(0) || y < 0 || y >= map_->grid_size_(1))
        return false;
    if (!map_->isFree2D(x, y) || map_->isOccupied2D(x, y)) return false;
    const double hard_clearance = std::max(0.45, map_->esdf_safe_distance_);
    return map_->getDistance2D(x, y) >= hard_clearance;
}

bool Astar2D::search(const Eigen::Vector3d &start, const Eigen::Vector3d &goal,
                     std::vector<Eigen::Vector3d> &path,
                     double *path_len,
                     int *unknown_cells,
                     double *unknown_ratio) {
    path.clear();
    last_failure_reason_ = ASTAR_OK;
    map_->updateDistanceFields();

    Eigen::Vector3i start_idx, goal_idx;
    map_->posToIndex(start, start_idx);
    map_->posToIndex(goal, goal_idx);

    // 2D索引
    Eigen::Vector2i si(start_idx(0), start_idx(1));
    Eigen::Vector2i gi(goal_idx(0), goal_idx(1));

    const int nx = map_->grid_size_(0), ny = map_->grid_size_(1);
    const int total = nx * ny;
    std::vector<int8_t> walkable_cache(total, -1);
    std::vector<double> distance_cache(total, -1.0);
    auto walkableAt = [&](int x, int y) {
        if (x < 0 || x >= nx || y < 0 || y >= ny) return false;
        const int key = toAddress2D(x, y);
        if (walkable_cache[key] < 0) {
            distance_cache[key] = map_->getDistance2D(x, y);
            walkable_cache[key] =
                map_->isFree2D(x, y) &&
                !map_->isOccupied2D(x, y) &&
                distance_cache[key] >= std::max(0.45, map_->esdf_safe_distance_);
        }
        return walkable_cache[key] != 0;
    };

    const Eigen::Vector2i requested_si = si;
    const double hard_clearance = std::max(0.45, map_->esdf_safe_distance_);
    const double bridge_step = std::max(0.05, 0.5 * map_->resolution_);
    auto hardBridgeClear = [&](const Eigen::Vector3d &from,
                               const Eigen::Vector3d &to) {
        Eigen::Vector3d seg = to - from;
        seg(2) = 0.0;
        const int steps = std::max(1, (int)std::ceil(seg.norm() / bridge_step));
        for (int i = 0; i <= steps; ++i) {
            Eigen::Vector3d p = from + ((double)i / steps) * seg;
            p(2) = map_->fly_height_;
            Eigen::Vector3i idx;
            map_->posToIndex(p, idx);
            if (!map_->isInMap2D(idx(0), idx(1)) ||
                !map_->isFree2D(idx(0), idx(1)) ||
                map_->isOccupied2D(idx(0), idx(1)) ||
                map_->getDistance2D(idx(0), idx(1)) < hard_clearance) {
                return false;
            }
        }
        return true;
    };
    auto escapeBridgeClear = [&](const Eigen::Vector3d &from,
                                 const Eigen::Vector3d &to) {
        Eigen::Vector3d seg = to - from;
        seg(2) = 0.0;
        const int steps = std::max(1, (int)std::ceil(seg.norm() / bridge_step));
        double prev_clearance = map_->getDistance2D(requested_si(0), requested_si(1));
        for (int i = 1; i <= steps; ++i) {
            Eigen::Vector3d p = from + ((double)i / steps) * seg;
            p(2) = map_->fly_height_;
            Eigen::Vector3i idx;
            map_->posToIndex(p, idx);
            if (!map_->isInMap2D(idx(0), idx(1))) return false;
            if (idx(0) == requested_si(0) && idx(1) == requested_si(1)) continue;
            if (!map_->isFree2D(idx(0), idx(1))) return false;
            double clearance = map_->getDistance2D(idx(0), idx(1));
            if (clearance + 1e-3 < prev_clearance) return false;
            prev_clearance = clearance;
        }
        return prev_clearance >= hard_clearance;
    };

    // 当前机体可能因地图噪声或控制超调落在安全膨胀区内。
    // 只允许沿已知自由且ESDF不减的直线连接到最近安全栅格，禁止跨墙“跳起点”。
    bool start_relocated = false;
    if (!walkableAt(si(0), si(1))) {
        struct EscapeCell { int x, y, dist2; };
        std::vector<EscapeCell> escape_cells;
        for (int dx = -5; dx <= 5; ++dx) {
            for (int dy = -5; dy <= 5; ++dy) {
                if (dx == 0 && dy == 0) continue;
                int x = requested_si(0) + dx;
                int y = requested_si(1) + dy;
                if (walkableAt(x, y)) escape_cells.push_back({x, y, dx * dx + dy * dy});
            }
        }
        std::sort(escape_cells.begin(), escape_cells.end(),
                  [](const EscapeCell &a, const EscapeCell &b) {
                      return a.dist2 < b.dist2;
                  });

        bool found_bridge = false;
        for (const auto &cell : escape_cells) {
            Eigen::Vector3d bridge_goal;
            map_->indexToPos(Eigen::Vector3i(cell.x, cell.y, map_->fly_z_idx_), bridge_goal);
            bridge_goal(2) = map_->fly_height_;
            if (!escapeBridgeClear(start, bridge_goal)) continue;
            si = Eigen::Vector2i(cell.x, cell.y);
            found_bridge = true;
            start_relocated = true;
            break;
        }
        if (!found_bridge) {
            last_failure_reason_ = ASTAR_ENDPOINT_BLOCKED;
            ROS_WARN_THROTTLE(1.0,
                "[Astar2D] Exact start has no monotonic-clearance bridge to a safe cell: "
                "start=(%.2f, %.2f), clearance=%.2f",
                start(0), start(1), map_->getDistance2D(requested_si(0), requested_si(1)));
            return false;
        }
        ROS_WARN_THROTTLE(1.0,
            "[Astar2D] Start is inside hard clearance; using validated escape bridge "
            "(%d,%d)->(%d,%d).",
            requested_si(0), requested_si(1), si(0), si(1));
    }

    // 前沿视点必须精确可达，不能静默换成附近另一个点。
    if (!walkableAt(gi(0), gi(1))) {
        last_failure_reason_ = ASTAR_ENDPOINT_BLOCKED;
        ROS_WARN_THROTTLE(1.0,
            "[Astar2D] Exact goal is not walkable: goal=(%.2f, %.2f), free=%d, clearance=%.2f",
            goal(0), goal(1), map_->isFree2D(gi(0), gi(1)),
            map_->getDistance2D(gi(0), gi(1)));
        return false;
    }

    // A*搜索
    std::vector<double> g_score(total, 1e10);
    std::vector<double> f_score(total, 1e10);
    std::vector<int> parent(total, -1);
    std::vector<bool> closed(total, false);

    int si_key = toAddress2D(si(0), si(1));
    int gi_key = toAddress2D(gi(0), gi(1));

    g_score[si_key] = 0;
    f_score[si_key] = heuristic(si, gi);

    // 优先队列: (f_score, key)
    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>,
                        std::greater<std::pair<double, int>>> open;
    open.push({f_score[si_key], si_key});

    auto t_start = std::chrono::high_resolution_clock::now();
    int expand_count = 0;
    const int MAX_EXPAND = 90000;
    auto clearancePenalty = [&](int x, int y) -> double {
        int key = toAddress2D(x, y);
        const double hard_clearance = std::max(0.45, map_->esdf_safe_distance_);
        const double desired_clearance = std::max(0.80, hard_clearance);
        const double dist = distance_cache[key] >= 0.0
                                ? distance_cache[key]
                                : map_->getDistance2D(x, y);
        if (dist < hard_clearance) return 1e6;
        if (dist >= desired_clearance) return 0.0;
        double lack = (desired_clearance - dist) /
                      std::max(1e-3, desired_clearance - hard_clearance);
        return 12.0 * lack * lack;
    };

    while (!open.empty()) {
        // 超时检查
        if (expand_count++ % 200 == 0) {
            auto t_now = std::chrono::high_resolution_clock::now();
            double elapsed = std::chrono::duration<double, std::milli>(t_now - t_start).count();
            if (elapsed > search_timeout_ms_ || expand_count > MAX_EXPAND) {
                last_failure_reason_ = ASTAR_TIMEOUT;
                return false;
            }
        }

        auto top = open.top();
        open.pop();
        int cur_key = top.second;

        if (closed[cur_key]) continue;
        closed[cur_key] = true;

        if (cur_key == gi_key) {
            // 回溯路径
            std::vector<Eigen::Vector2i> idx_path;
            int key = gi_key;
            while (key != -1) {
                int x = key / ny;
                int y = key % ny;
                idx_path.push_back(Eigen::Vector2i(x, y));
                key = parent[key];
            }
            std::reverse(idx_path.begin(), idx_path.end());

            double raw_path_len = 0.0;
            int raw_unknown_cells = 0;
            double raw_unknown_ratio = 0.0;
            if (!analyzePathUnknown(idx_path, raw_path_len, raw_unknown_cells, raw_unknown_ratio)) {
                last_failure_reason_ = ASTAR_UNKNOWN_REJECTED;
                return false;
            }

            // 转换为世界坐标并简化路径
            for (auto &idx : idx_path) {
                Eigen::Vector3i idx3(idx(0), idx(1), start_idx(2));
                Eigen::Vector3d pos;
                map_->indexToPos(idx3, pos);
                pos(2) = map_->fly_height_;
                path.push_back(pos);
            }

            // 路径简化：移除共线点（使用距离阈值避免浮点比较问题）
            if (path.size() > 2) {
                std::vector<Eigen::Vector3d> simplified;
                simplified.push_back(path[0]);
                for (int i = 1; i < (int)path.size() - 1; i++) {
                    Eigen::Vector3d dir_prev = path[i] - path[i-1];
                    Eigen::Vector3d dir_next = path[i+1] - path[i];
                    dir_prev(2) = 0; dir_next(2) = 0;
                    double len_prev = dir_prev.norm();
                    double len_next = dir_next.norm();
                    // 如果方向不同（非共线），保留该点
                    bool collinear = false;
                    if (len_prev > 1e-6 && len_next > 1e-6) {
                        double dot = dir_prev.normalized().dot(dir_next.normalized());
                        collinear = (fabs(dot - 1.0) < 1e-3);  // 方向几乎相同
                    }
                    if (!collinear) {
                        simplified.push_back(path[i]);
                    }
                }
                simplified.push_back(path.back());
                path = simplified;
            }

            Eigen::Vector3d exact_start = start;
            Eigen::Vector3d exact_goal = goal;
            exact_start(2) = exact_goal(2) = map_->fly_height_;
            const Eigen::Vector3d grid_start = path.front();
            const Eigen::Vector3d grid_goal = path.back();
            bool start_bridge_ok = start_relocated
                ? escapeBridgeClear(exact_start, grid_start)
                : hardBridgeClear(exact_start, grid_start);
            bool goal_bridge_ok = hardBridgeClear(grid_goal, exact_goal);
            if (!start_bridge_ok || !goal_bridge_ok) {
                path.clear();
                last_failure_reason_ = ASTAR_ENDPOINT_BLOCKED;
                ROS_WARN_THROTTLE(1.0,
                    "[Astar2D] Grid path found, but exact endpoint bridge is unsafe: "
                    "start_bridge=%d goal_bridge=%d",
                    start_bridge_ok, goal_bridge_ok);
                return false;
            }

            double start_bridge_len = (grid_start.head<2>() - exact_start.head<2>()).norm();
            double goal_bridge_len = (exact_goal.head<2>() - grid_goal.head<2>()).norm();
            if (start_bridge_len > 1e-3) path.insert(path.begin(), exact_start);
            else path.front() = exact_start;
            if (goal_bridge_len > 1e-3) path.push_back(exact_goal);
            else path.back() = exact_goal;
            raw_path_len += start_bridge_len + goal_bridge_len;

            if (path_len) *path_len = raw_path_len;
            if (unknown_cells) *unknown_cells = raw_unknown_cells;
            if (unknown_ratio) *unknown_ratio = raw_unknown_ratio;
            return true;
        }

        int cx = cur_key / ny;
        int cy = cur_key % ny;

        for (int d = 0; d < 8; d++) {
            int nx_ = cx + dx_[d];
            int ny_ = cy + dy_[d];

            if (!walkableAt(nx_, ny_)) continue;

            // 对角线移动时检查两个相邻格子是否都可通行（防止穿墙角）
            if (d == 0 || d == 2 || d == 5 || d == 7) {
                if (!walkableAt(cx + dx_[d], cy) || !walkableAt(cx, cy + dy_[d]))
                    continue;
            }

            int n_key = toAddress2D(nx_, ny_);
            if (closed[n_key]) continue;

            double cell_cost = move_cost_[d] + clearancePenalty(nx_, ny_);
            double new_g = g_score[cur_key] + cell_cost;
            if (new_g < g_score[n_key]) {
                g_score[n_key] = new_g;
                f_score[n_key] = new_g + heuristic(Eigen::Vector2i(nx_, ny_), gi);
                parent[n_key] = cur_key;
                open.push({f_score[n_key], n_key});
            }
        }
    }

    last_failure_reason_ = ASTAR_NO_PATH;
    return false;  // 搜索失败
}

bool Astar2D::analyzePathUnknown(const std::vector<Eigen::Vector2i> &idx_path,
                                 double &path_len,
                                 int &unknown_cells,
                                 double &unknown_ratio) {
    path_len = 0.0;
    unknown_cells = 0;
    int checked_cells = 0;
    for (int i = 0; i < (int)idx_path.size(); ++i) {
        int x = idx_path[i](0);
        int y = idx_path[i](1);
        if (!map_->isInMap2D(x, y) || !map_->isFree2D(x, y)) return false;
        if (map_->isUnknown2D(x, y)) unknown_cells++;
        checked_cells++;
        if (i > 0) {
            Eigen::Vector2i d = idx_path[i] - idx_path[i - 1];
            path_len += std::sqrt((double)(d(0) * d(0) + d(1) * d(1))) * map_->resolution_;
        }
    }
    unknown_ratio = checked_cells > 0 ? (double)unknown_cells / (double)checked_cells : 0.0;
    return unknown_cells == 0;
}

bool Astar2D::isPathClear(const Eigen::Vector3d &start, const Eigen::Vector3d &goal) {
    map_->updateDistanceFields();
    Eigen::Vector3d dir = goal - start;
    dir(2) = 0;
    double dist = dir.norm();
    if (dist < 0.01) return true;
    dir.normalize();

    const double hard_clearance = std::max(0.45, map_->esdf_safe_distance_);
    double step = std::max(0.05, 0.5 * map_->resolution_);
    for (double d = step; d <= dist; d += step) {
        Eigen::Vector3d check_pos = start + d * dir;
        check_pos(2) = map_->fly_height_;
        Eigen::Vector3i idx;
        map_->posToIndex(check_pos, idx);
        if (!map_->isFree2D(idx(0), idx(1))) return false;
        if (map_->getDistance2D(idx(0), idx(1)) < hard_clearance) return false;
    }
    return true;
}

bool Astar2D::isPathTraversable(const Eigen::Vector3d &start,
                                const Eigen::Vector3d &goal,
                                double *path_len,
                                int *unknown_cells,
                                double *unknown_ratio) {
    map_->updateDistanceFields();
    Eigen::Vector3d dir = goal - start;
    dir(2) = 0;
    double dist = dir.norm();
    if (path_len) *path_len = dist;
    if (unknown_cells) *unknown_cells = 0;
    if (unknown_ratio) *unknown_ratio = 0.0;
    if (dist < 0.01) return true;
    dir.normalize();

    int unknown = 0;
    int checked = 0;
    const double hard_clearance = std::max(0.45, map_->esdf_safe_distance_);
    double step = std::max(0.05, 0.5 * map_->resolution_);
    for (double d = step; d <= dist; d += step) {
        Eigen::Vector3d check_pos = start + d * dir;
        check_pos(2) = map_->fly_height_;
        if (!map_->isInMap(check_pos)) return false;
        Eigen::Vector3i idx;
        map_->posToIndex(check_pos, idx);
        if (!map_->isInMap2D(idx(0), idx(1)) || !map_->isFree2D(idx(0), idx(1))) {
            return false;
        }
        if (map_->getDistance2D(idx(0), idx(1)) < hard_clearance) return false;
        if (map_->isUnknown2D(idx(0), idx(1))) unknown++;
        checked++;
    }

    double ratio = checked > 0 ? (double)unknown / (double)checked : 0.0;
    if (path_len) *path_len = dist;
    if (unknown_cells) *unknown_cells = unknown;
    if (unknown_ratio) *unknown_ratio = ratio;
    return unknown == 0;
}

bool Astar2D::findReachableApproach(const Eigen::Vector3d &start,
                                    const Eigen::Vector3d &target,
                                    Eigen::Vector3d &approach,
                                    std::vector<Eigen::Vector3d> &path) {
    path.clear();
    map_->updateDistanceFields();
    Eigen::Vector3i start_idx, target_idx;
    map_->posToIndex(start, start_idx);
    map_->posToIndex(target, target_idx);
    const int nx = map_->grid_size_(0), ny = map_->grid_size_(1);
    const int total = nx * ny;
    const double hard_clearance = std::max(0.45, map_->esdf_safe_distance_);
    auto walkable = [&](int x, int y) {
        return x >= 0 && x < nx && y >= 0 && y < ny &&
               map_->isFree2D(x, y) && !map_->isOccupied2D(x, y) &&
               map_->getDistance2D(x, y) >= hard_clearance;
    };

    int sx = start_idx(0), sy = start_idx(1);
    if (!walkable(sx, sy)) {
        int best_dist2 = std::numeric_limits<int>::max();
        for (int dx = -5; dx <= 5; ++dx) for (int dy = -5; dy <= 5; ++dy) {
            if (!walkable(sx + dx, sy + dy)) continue;
            const int dist2 = dx * dx + dy * dy;
            if (dist2 < best_dist2) {
                best_dist2 = dist2;
                sx = start_idx(0) + dx;
                sy = start_idx(1) + dy;
            }
        }
        if (best_dist2 == std::numeric_limits<int>::max()) return false;
    }

    const auto key = [ny](int x, int y) { return x * ny + y; };
    std::vector<int> steps(total, -1);
    std::queue<Eigen::Vector2i> open;
    steps[key(sx, sy)] = 0;
    open.emplace(sx, sy);
    const double start_distance = std::hypot((sx - target_idx(0)) * map_->resolution_,
                                             (sy - target_idx(1)) * map_->resolution_);
    int best_key = -1;
    double best_distance = start_distance;
    int best_steps = std::numeric_limits<int>::max();

    while (!open.empty()) {
        const Eigen::Vector2i cell = open.front();
        open.pop();
        const int current_steps = steps[key(cell(0), cell(1))];
        const double target_distance = std::hypot((cell(0) - target_idx(0)) * map_->resolution_,
                                                  (cell(1) - target_idx(1)) * map_->resolution_);
        if (current_steps >= 2 && target_distance + map_->resolution_ <= best_distance + 1e-6) {
            best_key = key(cell(0), cell(1));
            best_distance = target_distance;
            best_steps = current_steps;
        }
        for (int d = 0; d < 8; ++d) {
            const int nx_ = cell(0) + dx_[d], ny_ = cell(1) + dy_[d];
            const int next_key = key(nx_, ny_);
            if (!walkable(nx_, ny_) || steps[next_key] >= 0) continue;
            if ((d == 0 || d == 2 || d == 5 || d == 7) &&
                (!walkable(cell(0) + dx_[d], cell(1)) ||
                 !walkable(cell(0), cell(1) + dy_[d]))) continue;
            steps[next_key] = current_steps + 1;
            open.emplace(nx_, ny_);
        }
    }
    if (best_key < 0 || best_distance + map_->resolution_ > start_distance ||
        best_steps == std::numeric_limits<int>::max()) return false;

    const int bx = best_key / ny, by = best_key % ny;
    map_->indexToPos(Eigen::Vector3i(bx, by, map_->fly_z_idx_), approach);
    approach(2) = map_->fly_height_;
    return search(start, approach, path);
}
