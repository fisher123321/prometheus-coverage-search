#include "two_uav_coverage_search.h"

// ============================================================
// Astar2D 实现 - 2D A*路径搜索（借鉴FALCON pathfinding）
// ============================================================
void Astar2D::init(CoverageMap *map) {
    map_ = map;
    search_timeout_ms_ = 250.0;
    last_failure_reason_ = ASTAR_OK;
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
    const double hard_clearance = std::max(planning_clearance_, map_->esdf_safe_distance_);
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
                distance_cache[key] >= std::max(planning_clearance_, map_->esdf_safe_distance_);
        }
        return walkable_cache[key] != 0;
    };

    const Eigen::Vector2i requested_si = si;
    const double hard_clearance = std::max(planning_clearance_, map_->esdf_safe_distance_);
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
        const double hard_clearance = std::max(planning_clearance_, map_->esdf_safe_distance_);
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

    const double hard_clearance = std::max(planning_clearance_, map_->esdf_safe_distance_);
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
    const double hard_clearance = std::max(planning_clearance_, map_->esdf_safe_distance_);
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
    const double hard_clearance = std::max(planning_clearance_, map_->esdf_safe_distance_);
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

// ============================================================
// 2D Kinodynamic A* implementation
// ============================================================
bool KinodynamicAstar2D::search(
    const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel,
    const Eigen::Vector3d &start_acc, const Eigen::Vector3d &goal_pos,
    const Eigen::Vector3d &goal_vel, double max_vel, double max_acc,
    double timeout_ms, std::vector<Eigen::Vector3d> &path,
    const std::vector<Eigen::Vector3d> *guide_path, double guide_radius) {
    path.clear();
    last_failure_reason_ = KINO_OK;
    if (!map_ || max_vel <= 1e-3 || max_acc <= 1e-3) {
        last_failure_reason_ = KINO_NO_PATH;
        return false;
    }

    map_->updateDistanceFields();
    const double plan_clearance = std::max(planning_clearance_, map_->esdf_safe_distance_);
    Eigen::Vector3i goal_idx;
    map_->posToIndex(goal_pos, goal_idx);
    if (!map_->isInMap2D(goal_idx(0), goal_idx(1)) ||
        !map_->isFree2D(goal_idx(0), goal_idx(1)) ||
        map_->getDistance2D(goal_idx(0), goal_idx(1)) + 1e-3 < plan_clearance) {
        last_failure_reason_ = KINO_ENDPOINT_BLOCKED;
        return false;
    }

    const int ny = map_->grid_size_(1);
    std::vector<uint8_t> guide_mask;
    if (guide_path && guide_path->size() >= 2) {
        guide_radius = std::max(0.4, guide_radius);
        guide_mask.assign(map_->grid_size_(0) * ny, 0);
        const int radius_cells = static_cast<int>(
            std::ceil(guide_radius / map_->resolution_)) + 1;
        for (size_t segment = 1; segment < guide_path->size(); ++segment) {
            const Eigen::Vector2d a = (*guide_path)[segment - 1].head<2>();
            const Eigen::Vector2d b = (*guide_path)[segment].head<2>();
            Eigen::Vector3i ai, bi;
            map_->posToIndex((*guide_path)[segment - 1], ai);
            map_->posToIndex((*guide_path)[segment], bi);
            const int min_x = std::max(0, std::min(ai(0), bi(0)) - radius_cells);
            const int max_x = std::min(map_->grid_size_(0) - 1,
                std::max(ai(0), bi(0)) + radius_cells);
            const int min_y = std::max(0, std::min(ai(1), bi(1)) - radius_cells);
            const int max_y = std::min(map_->grid_size_(1) - 1,
                std::max(ai(1), bi(1)) + radius_cells);
            const Eigen::Vector2d ab = b - a;
            const double denominator = ab.squaredNorm();
            for (int x = min_x; x <= max_x; ++x) {
                for (int y = min_y; y <= max_y; ++y) {
                    Eigen::Vector3d center;
                    map_->indexToPos(Eigen::Vector3i(x, y, map_->fly_z_idx_), center);
                    const double t = denominator > 1e-9
                        ? std::max(0.0, std::min(1.0,
                            (center.head<2>() - a).dot(ab) / denominator)) : 0.0;
                    if ((center.head<2>() - (a + t * ab)).norm() <=
                        guide_radius + 0.5 * map_->resolution_) {
                        guide_mask[x * ny + y] = 1;
                    }
                }
            }
        }
    }
    const auto insideGuideCorridor = [&](const Eigen::Vector2d &position) {
        if (guide_mask.empty()) return true;
        Eigen::Vector3i idx;
        map_->posToIndex(Eigen::Vector3d(position(0), position(1), map_->fly_height_), idx);
        return map_->isInMap2D(idx(0), idx(1)) && guide_mask[idx(0) * ny + idx(1)] != 0;
    };

    struct Node {
        Eigen::Vector2d pos = Eigen::Vector2d::Zero();
        Eigen::Vector2d vel = Eigen::Vector2d::Zero();
        Eigen::Vector2d input = Eigen::Vector2d::Zero();
        double duration = 0.0;
        double g = 0.0;
        int parent = -1;
        int64_t key = 0;
    };

    const double velocity_resolution = std::max(0.35, 0.35 * max_vel);
    const int velocity_bins = std::max(3,
        static_cast<int>(std::ceil(2.0 * max_vel / velocity_resolution)) + 1);
    const auto velocityBin = [&](double velocity) {
        const double clipped = std::max(-max_vel, std::min(max_vel, velocity));
        return std::max(0, std::min(velocity_bins - 1,
            static_cast<int>(std::lround((clipped + max_vel) / velocity_resolution))));
    };
    const auto stateKey = [&](const Eigen::Vector2d &position,
                              const Eigen::Vector2d &velocity) {
        Eigen::Vector3d p(position(0), position(1), map_->fly_height_);
        Eigen::Vector3i idx;
        map_->posToIndex(p, idx);
        if (!map_->isInMap2D(idx(0), idx(1))) return int64_t(-1);
        int64_t key = static_cast<int64_t>(idx(0) * ny + idx(1));
        key = key * velocity_bins + velocityBin(velocity(0));
        key = key * velocity_bins + velocityBin(velocity(1));
        return key;
    };
    const auto safePrimitive = [&](const Eigen::Vector2d &position,
                                   const Eigen::Vector2d &velocity,
                                   const Eigen::Vector2d &acceleration,
                                   double duration) {
        Eigen::Vector3d start3(position(0), position(1), map_->fly_height_);
        Eigen::Vector3i start_index;
        map_->posToIndex(start3, start_index);
        double previous_clearance = map_->isInMap2D(start_index(0), start_index(1))
            ? map_->getDistance2D(start_index(0), start_index(1)) : 0.0;
        bool escaping = previous_clearance + 1e-3 < plan_clearance;
        const int samples = std::max(5, static_cast<int>(std::ceil(duration / 0.08)));
        for (int sample = 1; sample <= samples; ++sample) {
            const double t = duration * sample / samples;
            const Eigen::Vector2d p = position + t * velocity +
                                      0.5 * t * t * acceleration;
            const Eigen::Vector2d v = velocity + t * acceleration;
            if (v.norm() > 1.001 * max_vel || !insideGuideCorridor(p)) return false;
            Eigen::Vector3d p3(p(0), p(1), map_->fly_height_);
            Eigen::Vector3i idx;
            map_->posToIndex(p3, idx);
            if (!map_->isInMap2D(idx(0), idx(1)) ||
                !map_->isFree2D(idx(0), idx(1))) return false;
            const double clearance = map_->getDistance2D(idx(0), idx(1));
            if (escaping) {
                if (clearance + 1e-3 < previous_clearance) return false;
                if (clearance + 1e-3 >= plan_clearance) escaping = false;
            } else if (clearance + 1e-3 < plan_clearance) {
                return false;
            }
            previous_clearance = clearance;
        }
        return true;
    };
    const auto connectorSafe = [&](const Eigen::Vector2d &from,
                                   const Eigen::Vector2d &to) {
        const Eigen::Vector2d segment = to - from;
        const int samples = std::max(1, static_cast<int>(std::ceil(
            segment.norm() / std::max(0.05, 0.5 * map_->resolution_))));
        for (int i = 1; i <= samples; ++i) {
            const Eigen::Vector2d p = from + static_cast<double>(i) / samples * segment;
            if (!insideGuideCorridor(p)) return false;
            Eigen::Vector3d p3(p(0), p(1), map_->fly_height_);
            Eigen::Vector3i idx;
            map_->posToIndex(p3, idx);
            if (!map_->isInMap2D(idx(0), idx(1)) ||
                !map_->isFree2D(idx(0), idx(1)) ||
                map_->getDistance2D(idx(0), idx(1)) + 1e-3 < plan_clearance) return false;
        }
        return true;
    };

    std::vector<Eigen::Vector2d> accelerations{Eigen::Vector2d::Zero()};
    for (int direction = 0; direction < 8; ++direction) {
        const double angle = direction * M_PI / 4.0;
        accelerations.emplace_back(max_acc * std::cos(angle), max_acc * std::sin(angle));
    }
    Eigen::Vector2d inherited_acc = start_acc.head<2>();
    if (inherited_acc.norm() > max_acc) inherited_acc *= max_acc / inherited_acc.norm();
    if (std::none_of(accelerations.begin(), accelerations.end(),
        [&](const Eigen::Vector2d &candidate) {
            return (candidate - inherited_acc).norm() < 1e-3;
        })) accelerations.push_back(inherited_acc);

    const Eigen::Vector2d goal2 = goal_pos.head<2>();
    Eigen::Vector2d bounded_start_vel = start_vel.head<2>();
    if (bounded_start_vel.norm() > max_vel)
        bounded_start_vel *= max_vel / bounded_start_vel.norm();
    Eigen::Vector2d bounded_goal_vel = goal_vel.head<2>();
    if (bounded_goal_vel.norm() > max_vel)
        bounded_goal_vel *= max_vel / bounded_goal_vel.norm();

    std::vector<Node> nodes;
    nodes.reserve(24000);
    Node start;
    start.pos = start_pos.head<2>();
    start.vel = bounded_start_vel;
    start.key = stateKey(start.pos, start.vel);
    if (start.key < 0) {
        last_failure_reason_ = KINO_ENDPOINT_BLOCKED;
        return false;
    }
    nodes.push_back(start);
    std::unordered_map<int64_t, int> best_node;
    best_node[start.key] = 0;
    using QueueEntry = std::pair<double, int>;
    std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> open;
    const auto heuristic = [&](const Eigen::Vector2d &position,
                               const Eigen::Vector2d &velocity) {
        return (goal2 - position).norm() / std::max(0.1, max_vel) +
               0.20 * (bounded_goal_vel - velocity).norm() / std::max(0.1, max_acc);
    };
    open.push({heuristic(start.pos, start.vel), 0});

    const auto search_start = std::chrono::steady_clock::now();
    int goal_node = -1;
    // The geometric corridor already supplies the route; this state search
    // only needs a moving-start-compatible guide. One medium duration halves
    // branching versus the former two-duration lattice.
    const double durations[] = {0.55};
    while (!open.empty() && nodes.size() < 24000) {
        if (std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - search_start).count() > timeout_ms) {
            last_failure_reason_ = KINO_TIMEOUT;
            return false;
        }
        const int current_index = open.top().second;
        open.pop();
        const Node current = nodes[current_index];
        const auto best = best_node.find(current.key);
        if (best == best_node.end() || best->second != current_index) continue;

        const double goal_distance = (current.pos - goal2).norm();
        const double goal_velocity_error = (current.vel - bounded_goal_vel).norm();
        if (goal_distance <= std::max(0.30, 1.5 * map_->resolution_) &&
            goal_velocity_error <= std::max(0.50, 0.55 * max_vel) &&
            connectorSafe(current.pos, goal2)) {
            goal_node = current_index;
            break;
        }

        for (const Eigen::Vector2d &acceleration : accelerations) {
            for (double duration : durations) {
                if (!safePrimitive(current.pos, current.vel, acceleration, duration)) continue;
                Node next;
                next.pos = current.pos + duration * current.vel +
                           0.5 * duration * duration * acceleration;
                next.vel = current.vel + duration * acceleration;
                next.input = acceleration;
                next.duration = duration;
                next.parent = current_index;
                next.key = stateKey(next.pos, next.vel);
                if (next.key < 0) continue;
                Eigen::Vector3d next3(next.pos(0), next.pos(1), map_->fly_height_);
                Eigen::Vector3i next_idx;
                map_->posToIndex(next3, next_idx);
                const double clearance = map_->getDistance2D(next_idx(0), next_idx(1));
                const double clearance_lack = std::max(0.0, 0.80 - clearance);
                next.g = current.g + duration + 0.03 * acceleration.squaredNorm() * duration +
                         0.5 * clearance_lack * clearance_lack;
                const auto previous = best_node.find(next.key);
                if (previous != best_node.end() &&
                    nodes[previous->second].g <= next.g + 1e-6) continue;
                const int next_index = static_cast<int>(nodes.size());
                nodes.push_back(next);
                best_node[next.key] = next_index;
                open.push({next.g + 1.5 * heuristic(next.pos, next.vel), next_index});
            }
        }
    }

    if (goal_node < 0) {
        last_failure_reason_ = KINO_NO_PATH;
        return false;
    }

    std::vector<int> chain;
    for (int index = goal_node; index >= 0; index = nodes[index].parent)
        chain.push_back(index);
    std::reverse(chain.begin(), chain.end());
    path.emplace_back(start_pos(0), start_pos(1), map_->fly_height_);
    for (size_t i = 1; i < chain.size(); ++i) {
        const Node &parent = nodes[chain[i - 1]];
        const Node &child = nodes[chain[i]];
        const int samples = std::max(2, static_cast<int>(std::ceil(child.duration / 0.12)));
        for (int sample = 1; sample <= samples; ++sample) {
            const double t = child.duration * sample / samples;
            const Eigen::Vector2d p = parent.pos + t * parent.vel +
                                      0.5 * t * t * child.input;
            path.emplace_back(p(0), p(1), map_->fly_height_);
        }
    }
    const Eigen::Vector2d last = path.back().head<2>();
    const int connector_samples = std::max(1, static_cast<int>(std::ceil(
        (goal2 - last).norm() / std::max(0.05, map_->resolution_))));
    for (int sample = 1; sample <= connector_samples; ++sample) {
        const Eigen::Vector2d p = last + static_cast<double>(sample) /
            connector_samples * (goal2 - last);
        path.emplace_back(p(0), p(1), map_->fly_height_);
    }
    last_failure_reason_ = KINO_OK;
    return path.size() >= 2;
}
