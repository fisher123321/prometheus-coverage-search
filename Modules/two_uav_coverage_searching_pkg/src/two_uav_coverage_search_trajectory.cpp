#include "two_uav_coverage_search.h"

#include <nlopt.hpp>

namespace {

double quinticProgress(double x) {
    x = std::max(0.0, std::min(1.0, x));
    return x * x * x * (10.0 + x * (-15.0 + 6.0 * x));
}

struct LocalBsplineOptContext {
    CoverageMap *map = nullptr;
    std::vector<Eigen::Vector3d> initial_ctrl;
    int first_free = 0;
    int last_free = -1;
    double knot_span = 0.0;
    Eigen::Vector3d start_pos, start_vel, start_acc, end_pos, end_vel, end_acc;
    double start_yaw = 0.0, start_yaw_rate = 0.0, start_yaw_acc = 0.0, end_yaw = 0.0;
    double desired_clearance = 0.0;
    double max_vel = 0.0;
    double max_acc = 0.0;
    std::vector<double> best;
    double best_cost = std::numeric_limits<double>::infinity();
};

double localBsplineDistance(LocalBsplineOptContext *ctx, const Eigen::Vector3d &p) {
    Eigen::Vector3i idx;
    ctx->map->posToIndex(p, idx);
    if (!ctx->map->isInMap2D(idx(0), idx(1))) return 0.0;
    return ctx->map->getDistance2D(idx(0), idx(1));
}

double localBsplineCost(const std::vector<double> &x, std::vector<double> &grad,
                        void *data) {
    auto *ctx = static_cast<LocalBsplineOptContext *>(data);
    std::vector<Eigen::Vector3d> q = ctx->initial_ctrl;
    const int free_count = ctx->last_free - ctx->first_free + 1;
    const int time_index = 2 * free_count;
    const double duration = std::max(0.05, x[time_index]);
    const double knot_span = duration / std::max(1, (int)q.size() - 3);
    grad.assign(x.size(), 0.0);
    for (int k = 0; k < free_count; ++k) {
        q[ctx->first_free + k](0) = x[2 * k];
        q[ctx->first_free + k](1) = x[2 * k + 1];
    }
    q[0] = ctx->start_pos;
    q[1] = ctx->start_pos + knot_span / 3.0 * ctx->start_vel;
    q[2] = q[1] + 2.0 * knot_span / 3.0 *
        (ctx->start_vel + 0.5 * knot_span * ctx->start_acc);
    const int last = (int)q.size() - 1;
    q[last] = ctx->end_pos;
    q[last - 1] = ctx->end_pos - knot_span / 3.0 * ctx->end_vel;
    q[last - 2] = q[last - 1] + 2.0 * knot_span / 3.0 *
        (-ctx->end_vel + 0.5 * knot_span * ctx->end_acc);
    auto addGrad = [&](int i, const Eigen::Vector2d &g) {
        if (i < ctx->first_free || i > ctx->last_free) return;
        const int k = 2 * (i - ctx->first_free);
        grad[k] += g(0);
        grad[k + 1] += g(1);
    };

    double cost = 0.0;
    constexpr double kJerkWeight = 4.0;
    constexpr double kGuideWeight = 0.8;
    constexpr double kDistanceWeight = 30.0;
    constexpr double kFeasibilityWeight = 2.0;

    for (int i = 0; i + 3 < (int)q.size(); ++i) {
        const Eigen::Vector2d jerk = (q[i + 3] - 3.0 * q[i + 2] +
                                      3.0 * q[i + 1] - q[i]).head<2>();
        cost += kJerkWeight * jerk.squaredNorm();
        const Eigen::Vector2d g = 2.0 * kJerkWeight * jerk;
        addGrad(i, -g);
        addGrad(i + 1, 3.0 * g);
        addGrad(i + 2, -3.0 * g);
        addGrad(i + 3, g);
    }
    for (int i = ctx->first_free; i <= ctx->last_free; ++i) {
        const Eigen::Vector2d deviation =
            (q[i] - ctx->initial_ctrl[i]).head<2>();
        cost += kGuideWeight * deviation.squaredNorm();
        addGrad(i, 2.0 * kGuideWeight * deviation);

        const double dist = localBsplineDistance(ctx, q[i]);
        if (dist <= 0.45) return 1e12;  // numerical infinity: hard B-spline clearance
        if (dist < ctx->desired_clearance) {
            const double lack = ctx->desired_clearance - dist;
            cost += kDistanceWeight * lack * lack;
            const double eps = std::max(0.05, 0.5 * ctx->map->resolution_);
            Eigen::Vector3d px = q[i], py = q[i];
            px(0) += eps;
            py(1) += eps;
            Eigen::Vector3d mx = q[i], my = q[i];
            mx(0) -= eps;
            my(1) -= eps;
            Eigen::Vector2d distance_grad(
                (localBsplineDistance(ctx, px) - localBsplineDistance(ctx, mx)) / (2.0 * eps),
                (localBsplineDistance(ctx, py) - localBsplineDistance(ctx, my)) / (2.0 * eps));
            if (distance_grad.norm() > 1e-5) {
                distance_grad.normalize();
                addGrad(i, -2.0 * kDistanceWeight * lack * distance_grad);
            }
        }
    }
    const double inv_dt = 1.0 / std::max(1e-3, knot_span);
    const double vel_scale = 3.0 * inv_dt;
    const double acc_scale = 6.0 * inv_dt * inv_dt;
    for (int i = 0; i + 1 < (int)q.size(); ++i) {
        const Eigen::Vector2d velocity = vel_scale * (q[i + 1] - q[i]).head<2>();
        const double speed = velocity.norm();
        if (speed > ctx->max_vel) {
            const double excess = speed - ctx->max_vel;
            const Eigen::Vector2d g = 2.0 * kFeasibilityWeight * excess *
                                      velocity.normalized() * vel_scale;
            cost += kFeasibilityWeight * excess * excess;
            grad[time_index] -= 2.0 * kFeasibilityWeight * excess * speed / duration;
            addGrad(i, -g);
            addGrad(i + 1, g);
        }
    }
    constexpr double kTimeWeight = 8.0;
    cost += kTimeWeight * duration;
    grad[time_index] = kTimeWeight;
    for (int i = 0; i + 2 < (int)q.size(); ++i) {
        const Eigen::Vector2d acceleration = acc_scale *
            (q[i + 2] - 2.0 * q[i + 1] + q[i]).head<2>();
        const double acc = acceleration.norm();
        if (acc > ctx->max_acc) {
            const double excess = acc - ctx->max_acc;
            const Eigen::Vector2d g = 2.0 * kFeasibilityWeight * excess *
                                      acceleration.normalized() * acc_scale;
            cost += kFeasibilityWeight * excess * excess;
            grad[time_index] -= 4.0 * kFeasibilityWeight * excess * acc / duration;
            addGrad(i, g);
            addGrad(i + 1, -2.0 * g);
            addGrad(i + 2, g);
        }
    }
    if (std::isfinite(cost) && cost < ctx->best_cost) {
        ctx->best_cost = cost;
        ctx->best = x;
    }
    return std::isfinite(cost) ? cost : 1e12;
}

}  // namespace

// ============================================================
// ★ 三次均匀B样条评估
// ============================================================
Eigen::Vector3d CoverageSearchManager::evaluateBspline(
    const std::vector<Eigen::Vector3d> &ctrl_pts, double t) {
    // 三次均匀B样条基函数（Cox-de Boor简化版）
    // N0(t) = (1-t)^3 / 6
    // N1(t) = (3t^3 - 6t^2 + 4) / 6
    // N2(t) = (-3t^3 + 3t^2 + 3t + 1) / 6
    // N3(t) = t^3 / 6
    int n = ctrl_pts.size();
    int n_segments = n - 3;
    if (n_segments <= 0) return ctrl_pts[0];

    double scaled_t = t * n_segments;
    int seg = min((int)floor(scaled_t), n_segments - 1);
    seg = max(seg, 0);
    double local_t = scaled_t - seg;
    local_t = min(max(local_t, 0.0), 1.0);

    double lt = local_t, lt2 = lt * lt, lt3 = lt2 * lt;
    double lb0 = (1 - lt) * (1 - lt) * (1 - lt) / 6.0;
    double lb1 = (3 * lt3 - 6 * lt2 + 4) / 6.0;
    double lb2 = (-3 * lt3 + 3 * lt2 + 3 * lt + 1) / 6.0;
    double lb3 = lt3 / 6.0;

    return lb0 * ctrl_pts[seg] + lb1 * ctrl_pts[seg + 1]
         + lb2 * ctrl_pts[seg + 2] + lb3 * ctrl_pts[seg + 3];
}

Eigen::Vector3d CoverageSearchManager::deBoor(
    const std::vector<Eigen::Vector3d> &ctrl_pts,
    const std::vector<double> &knots, int degree, double time) const {
    if (ctrl_pts.empty() || degree < 0 ||
        knots.size() != ctrl_pts.size() + degree + 1) {
        return Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    }
    if (degree == 0) return ctrl_pts.front();

    const int n = (int)ctrl_pts.size() - 1;
    const double begin = knots[degree];
    const double end = knots[n + 1];
    const double t = std::max(begin, std::min(end, time));
    int span = n;
    if (t < end) {
        span = (int)(std::upper_bound(knots.begin(), knots.end(), t) - knots.begin()) - 1;
        span = std::max(degree, std::min(n, span));
    }

    std::vector<Eigen::Vector3d> values(degree + 1);
    for (int j = 0; j <= degree; ++j) values[j] = ctrl_pts[span - degree + j];
    for (int r = 1; r <= degree; ++r) {
        for (int j = degree; j >= r; --j) {
            const int i = span - degree + j;
            const double denominator = knots[i + degree - r + 1] - knots[i];
            const double alpha = denominator > 1e-9 ? (t - knots[i]) / denominator : 0.0;
            values[j] = (1.0 - alpha) * values[j - 1] + alpha * values[j];
        }
    }
    return values[degree];
}

bool CoverageSearchManager::evaluateTimeBspline(
    const TimeBspline &spline, double time,
    Eigen::Vector3d &position, Eigen::Vector3d &velocity,
    Eigen::Vector3d &acceleration, double &yaw, double &yaw_rate,
    double &yaw_acceleration) const {
    if (!spline.valid || spline.degree != 3 || spline.position_ctrl.size() < 4 ||
        spline.yaw_ctrl.size() < 4 || spline.position_duration <= 0.0 ||
        spline.yaw_duration <= 0.0) return false;

    auto derivative = [](const std::vector<Eigen::Vector3d> &ctrl,
                         const std::vector<double> &knots, int degree,
                         std::vector<Eigen::Vector3d> &out_ctrl,
                         std::vector<double> &out_knots) {
        if (degree <= 0 || ctrl.size() < 2) return false;
        out_ctrl.resize(ctrl.size() - 1);
        for (int i = 0; i + 1 < (int)ctrl.size(); ++i) {
            const double denominator = knots[i + degree + 1] - knots[i + 1];
            if (denominator <= 1e-9) return false;
            out_ctrl[i] = degree * (ctrl[i + 1] - ctrl[i]) / denominator;
        }
        out_knots.assign(knots.begin() + 1, knots.end() - 1);
        return true;
    };

    std::vector<Eigen::Vector3d> velocity_ctrl, acceleration_ctrl;
    std::vector<Eigen::Vector3d> yaw_rate_ctrl, yaw_acc_ctrl;
    std::vector<double> velocity_knots, acceleration_knots;
    std::vector<double> yaw_rate_knots, yaw_acc_knots;
    if (!derivative(spline.position_ctrl, spline.knots, 3,
                    velocity_ctrl, velocity_knots) ||
        !derivative(velocity_ctrl, velocity_knots, 2,
                    acceleration_ctrl, acceleration_knots) ||
        !derivative(spline.yaw_ctrl, spline.yaw_knots, 3,
                    yaw_rate_ctrl, yaw_rate_knots) ||
        !derivative(yaw_rate_ctrl, yaw_rate_knots, 2,
                    yaw_acc_ctrl, yaw_acc_knots)) return false;

    const double position_time = std::max(0.0, std::min(spline.position_duration, time));
    const double yaw_time = std::max(0.0, std::min(spline.yaw_duration, time));
    position = deBoor(spline.position_ctrl, spline.knots, 3, position_time);
    velocity = deBoor(velocity_ctrl, velocity_knots, 2, position_time);
    acceleration = deBoor(acceleration_ctrl, acceleration_knots, 1, position_time);
    const Eigen::Vector3d yaw_value = deBoor(spline.yaw_ctrl, spline.yaw_knots, 3, yaw_time);
    const Eigen::Vector3d yaw_rate_value = deBoor(yaw_rate_ctrl, yaw_rate_knots, 2, yaw_time);
    const Eigen::Vector3d yaw_acc_value = deBoor(yaw_acc_ctrl, yaw_acc_knots, 1, yaw_time);
    yaw = std::atan2(std::sin(yaw_value(0)), std::cos(yaw_value(0)));
    yaw_rate = yaw_rate_value(0);
    yaw_acceleration = yaw_acc_value(0);
    return position.allFinite() && velocity.allFinite() && acceleration.allFinite() &&
           std::isfinite(yaw) && std::isfinite(yaw_rate) &&
           std::isfinite(yaw_acceleration);
}

bool CoverageSearchManager::optimizeTimeBspline2D(TimeBspline &spline, bool rolling,
                                                   std::string &reason) {
    const int first_free = 3;
    const int last_free = (int)spline.position_ctrl.size() - 4;
    if (last_free < first_free) return true;

    LocalBsplineOptContext ctx;
    ctx.map = &coverage_map_;
    ctx.initial_ctrl = spline.position_ctrl;
    ctx.first_free = first_free;
    ctx.last_free = last_free;
    ctx.knot_span = spline.position_duration /
        std::max(1, (int)spline.position_ctrl.size() - spline.degree);
    ctx.desired_clearance = 0.80;
    ctx.max_vel = max_vel_;
    ctx.max_acc = max_acc_;
    Eigen::Vector3d ignored_p, ignored_v, ignored_a;
    double ignored_yaw_rate = 0.0, ignored_yaw_acc = 0.0;
    if (!evaluateTimeBspline(spline, 0.0, ctx.start_pos, ctx.start_vel, ctx.start_acc,
                             ctx.start_yaw, ctx.start_yaw_rate, ctx.start_yaw_acc) ||
        !evaluateTimeBspline(spline, spline.position_duration,
                             ctx.end_pos, ctx.end_vel, ctx.end_acc,
                             ctx.end_yaw, ignored_yaw_rate, ignored_yaw_acc)) {
        reason = "cannot recover B-spline boundary states";
        return false;
    }

    const int free_count = last_free - first_free + 1;
    std::vector<double> variables(2 * free_count + 1);
    std::vector<double> lower(variables.size()), upper(variables.size());
    for (int k = 0; k < free_count; ++k) {
        const Eigen::Vector3d &p = spline.position_ctrl[first_free + k];
        variables[2 * k] = p(0);
        variables[2 * k + 1] = p(1);
        lower[2 * k] = std::max(coverage_map_.origin_(0) + 0.1, p(0) - 1.0);
        upper[2 * k] = std::min(coverage_map_.origin_(0) + coverage_map_.map_size_3d_(0) - 0.1,
                                p(0) + 1.0);
        lower[2 * k + 1] = std::max(coverage_map_.origin_(1) + 0.1, p(1) - 1.0);
        upper[2 * k + 1] = std::min(coverage_map_.origin_(1) + coverage_map_.map_size_3d_(1) - 0.1,
                                    p(1) + 1.0);
    }
    variables.back() = spline.position_duration;
    // A failed sampled trajectory asks the outer loop for more time.  Keeping
    // that as the lower bound lets the outer retry/refinement find the shortest
    // duration that actually satisfies the hard P/V/A/yaw limits.
    lower.back() = spline.position_duration;
    upper.back() = 1.50 * spline.position_duration;
    std::vector<double> initial_grad;
    localBsplineCost(variables, initial_grad, &ctx);

    try {
        nlopt::opt opt(nlopt::LD_LBFGS, variables.size());
        opt.set_min_objective(localBsplineCost, &ctx);
        opt.set_lower_bounds(lower);
        opt.set_upper_bounds(upper);
        opt.set_maxeval(rolling ? 24 : 80);
        opt.set_maxtime(rolling ? 0.04 : 0.12);
        opt.set_xtol_rel(1e-4);
        double ignored_cost = 0.0;
        opt.optimize(variables, ignored_cost);
    } catch (const std::exception &e) {
        reason = std::string("NLOPT error: ") + e.what();
        return false;
    }
    if (ctx.best.size() != variables.size()) {
        reason = "NLOPT produced no finite control-point solution";
        return false;
    }
    const double optimized_duration = ctx.best.back();
    spline.position_duration = optimized_duration;
    const int spans = (int)spline.position_ctrl.size() - spline.degree;
    const double knot_span = optimized_duration / spans;
    for (int i = 0; i < (int)spline.knots.size(); ++i) {
        spline.knots[i] = i <= spline.degree ? 0.0 :
            (i >= (int)spline.position_ctrl.size() ? optimized_duration :
             (i - spline.degree) * knot_span);
    }
    for (int k = 0; k < free_count; ++k) {
        spline.position_ctrl[first_free + k](0) = ctx.best[2 * k];
        spline.position_ctrl[first_free + k](1) = ctx.best[2 * k + 1];
        spline.position_ctrl[first_free + k](2) = fly_height_;
    }
    const int last = (int)spline.position_ctrl.size() - 1;
    spline.position_ctrl[0] = ctx.start_pos;
    spline.position_ctrl[1] = ctx.start_pos + knot_span / 3.0 * ctx.start_vel;
    spline.position_ctrl[2] = spline.position_ctrl[1] + 2.0 * knot_span / 3.0 *
        (ctx.start_vel + 0.5 * knot_span * ctx.start_acc);
    spline.position_ctrl[last] = ctx.end_pos;
    spline.position_ctrl[last - 1] = ctx.end_pos - knot_span / 3.0 * ctx.end_vel;
    spline.position_ctrl[last - 2] = spline.position_ctrl[last - 1] + 2.0 * knot_span / 3.0 *
        (-ctx.end_vel + 0.5 * knot_span * ctx.end_acc);
    return true;
}

bool CoverageSearchManager::buildTimeParameterizedSpline(
    const Eigen::Vector3d &start_acc, double start_yaw_rate,
    double start_yaw_acceleration) {
    active_time_spline_ = TimeBspline();
    if (traj_points_.size() < 2) return false;

    std::vector<double> guide_arc(traj_points_.size(), 0.0);
    for (int i = 1; i < (int)traj_points_.size(); ++i) {
        guide_arc[i] = guide_arc[i - 1] +
            (traj_points_[i] - traj_points_[i - 1]).head<2>().norm();
    }
    const double total_length = guide_arc.back();
    if (total_length < 0.05) return false;

    const int control_count = std::max(7, std::min(160,
        (int)std::ceil(total_length / 0.15) + 4));
    auto sampleGuide = [&](double distance) {
        if (distance <= 0.0) return traj_points_.front();
        if (distance >= total_length) return traj_points_.back();
        auto upper = std::upper_bound(guide_arc.begin(), guide_arc.end(), distance);
        int next = std::max(1, (int)(upper - guide_arc.begin()));
        int prev = next - 1;
        const double span = std::max(1e-6, guide_arc[next] - guide_arc[prev]);
        Eigen::Vector3d point = traj_points_[prev] +
            (distance - guide_arc[prev]) / span * (traj_points_[next] - traj_points_[prev]);
        point(2) = fly_height_;
        return point;
    };

    const Eigen::Vector3d start_pos = traj_points_.front();
    Eigen::Vector3d start_vel = uav_vel_;
    Eigen::Vector3d desired_start_acc = start_acc;
    // Only a true trajectory endpoint is rest-to-rest.  Rolling handoffs use
    // their sampled in-trajectory state below and therefore retain P/V/A/yaw
    // derivatives instead of inheriting this zero terminal state.
    const Eigen::Vector3d terminal_vel = Eigen::Vector3d::Zero();
    const Eigen::Vector3d terminal_acc = Eigen::Vector3d::Zero();
    start_vel(2) = 0.0;
    desired_start_acc(2) = 0.0;
    if (!planning_start_state_valid_) {
        Eigen::Vector3d tangent = sampleGuide(std::min(0.50, total_length)) - start_pos;
        tangent(2) = 0.0;
        if (tangent.norm() > 1e-3) {
            tangent.normalize();
            const double forward_speed = std::max(0.0, std::min(
                0.8 * max_vel_, start_vel.head<2>().dot(tangent.head<2>())));
            start_vel = forward_speed * tangent;
        } else {
            start_vel.setZero();
        }
        desired_start_acc.setZero();
    }
    const double yaw_delta = std::atan2(std::sin(current_goal_yaw_ - uav_yaw_),
                                        std::cos(current_goal_yaw_ - uav_yaw_));
    const double goal_yaw_unwrapped = uav_yaw_ + yaw_delta;
    const auto minTravelTime = [](double distance, double initial_speed,
                                  double final_speed, double speed_limit,
                                  double acc_limit) {
        speed_limit = std::max(1e-3, speed_limit);
        acc_limit = std::max(1e-3, acc_limit);
        initial_speed = std::max(0.0, std::min(speed_limit, initial_speed));
        final_speed = std::max(0.0, std::min(speed_limit, final_speed));
        const double accel_distance = std::max(
            0.0, (speed_limit * speed_limit - initial_speed * initial_speed) /
                     (2.0 * acc_limit));
        const double decel_distance = std::max(
            0.0, (speed_limit * speed_limit - final_speed * final_speed) /
                     (2.0 * acc_limit));
        if (accel_distance + decel_distance <= distance) {
            return (speed_limit - initial_speed + speed_limit - final_speed) /
                       acc_limit +
                   (distance - accel_distance - decel_distance) / speed_limit;
        }
        const double peak_sq = std::max(
            0.0, acc_limit * distance +
                     0.5 * (initial_speed * initial_speed + final_speed * final_speed));
        const double peak_speed = std::sqrt(peak_sq);
        return (std::fabs(peak_speed - initial_speed) +
                std::fabs(peak_speed - final_speed)) / acc_limit;
    };
    const auto minRestToRestTurnTime = [](double angle, double rate_limit,
                                          double acc_limit) {
        angle = std::fabs(angle);
        rate_limit = std::max(1e-3, rate_limit);
        acc_limit = std::max(1e-3, acc_limit);
        const double triangular_angle = rate_limit * rate_limit / acc_limit;
        return angle <= triangular_angle ? 2.0 * std::sqrt(angle / acc_limit)
                                         : angle / rate_limit + rate_limit / acc_limit;
    };
    // Position and yaw use independent time axes, but yaw must complete before
    // the terminal P/V/A=0 position boundary.  A slow yaw therefore extends
    // the moving trajectory instead of creating a terminal hover.
    double position_duration = std::max(1.0,
        minTravelTime(total_length, start_vel.head<2>().norm(),
                      terminal_vel.head<2>().norm(), max_vel_, max_acc_));
    const double hard_clearance = std::max(
        std::max(0.35, coverage_map_.esdf_safe_distance_), traj_cut_clearance_);
    const Eigen::Vector3d terminal_pos = traj_points_.back();
    std::string last_rejection = "unknown";

    const auto setUniformKnots = [](std::vector<double> &knots, int count,
                                    int degree, double duration) {
        knots.resize(count + degree + 1);
        const double span = duration / std::max(1, count - degree);
        for (int i = 0; i < (int)knots.size(); ++i) {
            knots[i] = i <= degree ? 0.0 :
                (i >= count ? duration : (i - degree) * span);
        }
    };
    const auto configureYaw = [&](TimeBspline &spline, double yaw_duration) {
        spline.yaw_duration = yaw_duration;
        spline.yaw_ctrl.resize(control_count);
        for (int i = 0; i < control_count; ++i) {
            const double fraction = (double)i / (control_count - 1);
            spline.yaw_ctrl[i] = Eigen::Vector3d(
                uav_yaw_ + quinticProgress(fraction) * yaw_delta, 0.0, 0.0);
        }
        setUniformKnots(spline.yaw_knots, control_count, spline.degree, yaw_duration);
        const double span = yaw_duration / (control_count - spline.degree);
        const double next_yaw_rate = start_yaw_rate +
            0.5 * span * start_yaw_acceleration;
        spline.yaw_ctrl[0](0) = uav_yaw_;
        spline.yaw_ctrl[1](0) = uav_yaw_ + span / 3.0 * start_yaw_rate;
        spline.yaw_ctrl[2](0) = spline.yaw_ctrl[1](0) + 2.0 * span / 3.0 * next_yaw_rate;
        spline.yaw_ctrl[control_count - 1](0) = goal_yaw_unwrapped;
        spline.yaw_ctrl[control_count - 2](0) = goal_yaw_unwrapped;
        spline.yaw_ctrl[control_count - 3](0) = goal_yaw_unwrapped;
    };
    const auto yawFitsLimits = [&](TimeBspline &spline, double yaw_duration,
                                   double &peak_rate, double &peak_acceleration) {
        configureYaw(spline, yaw_duration);
        peak_rate = 0.0;
        peak_acceleration = 0.0;
        const int samples = std::max(8, (int)std::ceil(yaw_duration / 0.01));
        for (int i = 0; i <= samples; ++i) {
            Eigen::Vector3d p, v, a;
            double yaw = 0.0, yaw_rate = 0.0, yaw_acceleration = 0.0;
            if (!evaluateTimeBspline(spline, yaw_duration * i / samples,
                                     p, v, a, yaw, yaw_rate, yaw_acceleration)) {
                return false;
            }
            peak_rate = std::max(peak_rate, std::fabs(yaw_rate));
            peak_acceleration = std::max(peak_acceleration,
                                         std::fabs(yaw_acceleration));
        }
        return peak_rate <= 1.001 * max_yaw_rate_ &&
               peak_acceleration <= 1.001 * max_yaw_acc_;
    };

    for (int position_attempt = 0; position_attempt < 8; ++position_attempt) {
        TimeBspline candidate;
        candidate.degree = 3;
        candidate.position_duration = position_duration;
        candidate.duration = position_duration;
        candidate.position_ctrl.resize(control_count);
        for (int i = 0; i < control_count; ++i) {
            candidate.position_ctrl[i] = sampleGuide(
                (double)i / (control_count - 1) * total_length);
        }
        setUniformKnots(candidate.knots, control_count, candidate.degree, position_duration);
        const double position_span = position_duration / (control_count - candidate.degree);
        candidate.position_ctrl[0] = start_pos;
        candidate.position_ctrl[1] = start_pos + position_span / 3.0 * start_vel;
        candidate.position_ctrl[2] = candidate.position_ctrl[1] +
            2.0 * position_span / 3.0 * (start_vel + 0.5 * position_span * desired_start_acc);
        candidate.position_ctrl[control_count - 1] = terminal_pos;
        candidate.position_ctrl[control_count - 2] =
            terminal_pos - position_span / 3.0 * terminal_vel;
        candidate.position_ctrl[control_count - 3] = candidate.position_ctrl[control_count - 2] +
            2.0 * position_span / 3.0 * (-terminal_vel + 0.5 * position_span * terminal_acc);
        candidate.valid = true;
        configureYaw(candidate, position_duration);  // optimizer needs a complete spline state

        if (!optimizeTimeBspline2D(candidate, rolling_prepare_in_progress_, last_rejection)) {
            position_duration *= 1.15;
            continue;
        }

        // Search the independent yaw axis directly.  There is no yaw-progress
        // monotonicity condition: inherited opposite yaw velocity naturally
        // produces a braking segment, while rate and acceleration stay hard limits.
        double yaw_lower = 0.02;
        double yaw_upper = std::max({0.10,
            minRestToRestTurnTime(yaw_delta, max_yaw_rate_, max_yaw_acc_),
            std::fabs(start_yaw_rate) / std::max(1e-3, max_yaw_acc_)});
        double peak_yaw_rate = 0.0, peak_yaw_acc = 0.0;
        bool yaw_feasible = false;
        for (int i = 0; i < 24; ++i) {
            if (yawFitsLimits(candidate, yaw_upper, peak_yaw_rate, peak_yaw_acc)) {
                yaw_feasible = true;
                break;
            }
            yaw_lower = yaw_upper;
            yaw_upper *= 1.35;
        }
        if (!yaw_feasible) {
            last_rejection = "cannot find bounded independent yaw duration";
            break;
        }
        for (int i = 0; i < 12; ++i) {
            const double midpoint = 0.5 * (yaw_lower + yaw_upper);
            if (yawFitsLimits(candidate, midpoint, peak_yaw_rate, peak_yaw_acc)) {
                yaw_upper = midpoint;
            } else {
                yaw_lower = midpoint;
            }
        }
        yawFitsLimits(candidate, yaw_upper, peak_yaw_rate, peak_yaw_acc);
        constexpr double kYawFinishRatio = 0.90;
        if (candidate.yaw_duration > kYawFinishRatio * candidate.position_duration) {
            position_duration = candidate.yaw_duration / kYawFinishRatio;
            continue;
        }
        candidate.duration = candidate.position_duration;

        const int sample_count = std::max(2, (int)std::ceil(
            candidate.duration / std::min(0.10, 0.05 / std::max(0.20, max_vel_))));
        const double sample_dt = candidate.duration / sample_count;
        std::vector<Eigen::Vector3d> points;
        std::vector<Eigen::Vector3d> velocities;
        std::vector<Eigen::Vector3d> accelerations;
        std::vector<double> yaws;
        points.reserve(sample_count + 1);
        velocities.reserve(sample_count + 1);
        accelerations.reserve(sample_count + 1);
        yaws.reserve(sample_count + 1);
        bool safe = true;
        double peak_speed = 0.0, peak_acc = 0.0;
        Eigen::Vector3d previous = start_pos;
        for (int i = 0; i <= sample_count; ++i) {
            Eigen::Vector3d p, v, a;
            double yaw = 0.0, yaw_rate = 0.0, yaw_acceleration = 0.0;
            if (!evaluateTimeBspline(candidate, i * sample_dt, p, v, a,
                                     yaw, yaw_rate, yaw_acceleration)) {
                last_rejection = "De Boor evaluation failed";
                safe = false;
                break;
            }
            p(2) = fly_height_;
            v(2) = 0.0;
            a(2) = 0.0;
            Eigen::Vector3i idx;
            coverage_map_.posToIndex(p, idx);
            const bool point_safe = coverage_map_.isInMap2D(idx(0), idx(1)) &&
                coverage_map_.isFree2D(idx(0), idx(1)) &&
                coverage_map_.getDistance2D(idx(0), idx(1)) + 1e-3 >=
                    requiredTrajectoryClearance(p, hard_clearance);
            peak_speed = std::max(peak_speed, v.head<2>().norm());
            peak_acc = std::max(peak_acc, a.head<2>().norm());
            const bool segment_blocked = point_safe && i > 0 &&
                pathToTargetBlocked(p, previous, traj_cut_clearance_);
            if (!point_safe || segment_blocked || peak_speed > 1.03 * max_vel_ ||
                peak_acc > 1.03 * max_acc_) {
                last_rejection = !point_safe ? "ESDF/free-space point check" :
                    (segment_blocked ? "continuous segment clearance" :
                     (peak_speed > 1.03 * max_vel_ ? "speed limit" : "acceleration limit"));
                safe = false;
                break;
            }
            points.push_back(p);
            velocities.push_back(v);
            accelerations.push_back(a);
            yaws.push_back(yaw);
            previous = p;
        }
        if (!safe) {
            position_duration = std::max(position_duration * 1.15,
                                         candidate.position_duration * 1.05);
            continue;
        }

        Eigen::Vector3d p0, v0, a0, p_end, v_end, a_end, ignored_p, ignored_v, ignored_a;
        double yaw0 = 0.0, yaw_rate0 = 0.0, yaw_acc0 = 0.0;
        double yaw_end = 0.0, yaw_rate_end = 0.0, yaw_acc_end = 0.0;
        double ignored_yaw = 0.0, ignored_yaw_rate = 0.0, ignored_yaw_acc = 0.0;
        if (!evaluateTimeBspline(candidate, 0.0, p0, v0, a0,
                                 yaw0, yaw_rate0, yaw_acc0) ||
            !evaluateTimeBspline(candidate, candidate.position_duration,
                                 p_end, v_end, a_end,
                                 ignored_yaw, ignored_yaw_rate, ignored_yaw_acc) ||
            !evaluateTimeBspline(candidate, candidate.yaw_duration,
                                 ignored_p, ignored_v, ignored_a,
                                 yaw_end, yaw_rate_end, yaw_acc_end) ||
            (p0 - start_pos).norm() > 1e-6 || (v0 - start_vel).norm() > 1e-6 ||
            (a0 - desired_start_acc).norm() > 1e-5 ||
            std::fabs(std::atan2(std::sin(yaw0 - uav_yaw_),
                                 std::cos(yaw0 - uav_yaw_))) > 1e-6 ||
            std::fabs(yaw_rate0 - start_yaw_rate) > 1e-6 ||
            std::fabs(yaw_acc0 - start_yaw_acceleration) > 1e-5 ||
            (p_end - terminal_pos).norm() > 1e-6 ||
            (v_end - terminal_vel).norm() > 1e-6 || (a_end - terminal_acc).norm() > 1e-5 ||
            std::fabs(std::atan2(std::sin(yaw_end - goal_yaw_unwrapped),
                                 std::cos(yaw_end - goal_yaw_unwrapped))) > 1e-6 ||
            std::fabs(yaw_rate_end) > 1e-6 || std::fabs(yaw_acc_end) > 1e-5) {
            ROS_ERROR("[CoverageSearch] Reject time B-spline: De Boor endpoint "
                      "P/V/A/yaw invariant failed.");
            return false;
        }

        active_time_spline_ = candidate;
        traj_points_.swap(points);
        traj_vels_.swap(velocities);
        traj_accs_.swap(accelerations);
        traj_yaws_.swap(yaws);
        traj_dt_ = sample_dt;
        ROS_INFO("[CoverageSearch] Time B-spline accepted: position=%.2fs, yaw=%.2fs, "
                 "total=%.2fs, ctrl=%d, samples=%zu, peak_v=%.2f, peak_a=%.2f, "
                 "peak_yaw_rate=%.2f, peak_yaw_acc=%.2f.",
                 candidate.position_duration, candidate.yaw_duration, candidate.duration,
                 control_count, traj_points_.size(), peak_speed, peak_acc,
                 peak_yaw_rate, peak_yaw_acc);
        return true;
    }

    ROS_WARN("[CoverageSearch] Time B-spline rejected after position-duration expansion: "
             "last_reason=%s.", last_rejection.c_str());
    return false;
}

// ============================================================
// ★ 核心方法3：从A*路径生成B样条轨迹
// 使用三次均匀B样条对A*路径点进行平滑，替代线性插值
// ============================================================
void CoverageSearchManager::generateBsplineTraj() {
    if (!rolling_prepare_in_progress_) {
        pending_traj_ = PendingTrajectory();
        rolling_last_attempt_time_ = ros::Time::now();
        ++rolling_generation_;
    }
    active_time_spline_ = TimeBspline();
    traj_points_.clear();
    traj_vels_.clear();
    traj_accs_.clear();
    traj_yaws_.clear();

    if (astar_path_.empty()) { has_traj_ = false; return; }
    coverage_map_.updateDistanceFields();

    std::string traj_type = "Bspline";
    const double traj_min_clearance = std::max(
        std::max(0.35, coverage_map_.esdf_safe_distance_), traj_cut_clearance_);
    const double preferred_clearance = std::max(0.80, traj_min_clearance);
    const std::vector<Eigen::Vector3d> raw_astar_path = astar_path_;
    Eigen::Vector3i start_idx;
    coverage_map_.posToIndex(uav_pos_, start_idx);
    const bool start_requires_escape =
        !coverage_map_.isFree2D(start_idx(0), start_idx(1)) ||
        coverage_map_.getDistance2D(start_idx(0), start_idx(1)) <
            requiredTrajectoryClearance(uav_pos_, traj_min_clearance);
    const bool startup_bootstrap =
        !start_time_.isZero() && (ros::Time::now() - start_time_).toSec() < 6.0 &&
        (current_goal_ - uav_pos_).head<2>().norm() < 1.8;
    auto segmentHasKnownObstacle = [&](const Eigen::Vector3d &to,
                                       const Eigen::Vector3d &from) -> bool {
        Eigen::Vector3d seg = to - from;
        seg(2) = 0.0;
        double dist = seg.norm();
        if (dist < 0.05) return false;
        Eigen::Vector3d dir = seg / dist;
        double step = std::max(0.05, 0.5 * coverage_map_.resolution_);
        for (double d = 0.0; d <= dist + 1e-3; d += step) {
            Eigen::Vector3d p = from + d * dir;
            p(2) = fly_height_;
            if (!coverage_map_.isInMap(p)) return true;
            Eigen::Vector3i idx;
            coverage_map_.posToIndex(p, idx);
            if (coverage_map_.isOccupied2D(idx(0), idx(1)) ||
                coverage_map_.getDistance(p) <
                    requiredTrajectoryClearance(p, traj_min_clearance)) {
                return true;
            }
        }
        return false;
    };
    auto optimizeAstarPathByESDF = [&]() {
        if ((int)astar_path_.size() < 3) return;

        std::vector<Eigen::Vector3d> original = astar_path_;
        const double desired_clearance = preferred_clearance;
        const double eps = std::max(0.05, 0.5 * coverage_map_.resolution_);
        const double max_step = std::max(0.05, 0.5 * coverage_map_.resolution_);
        const double max_total_move = 0.5;

        auto pointSafe = [&](const Eigen::Vector3d &p) {
            if (!coverage_map_.isInMap(p)) return false;
            Eigen::Vector3i idx;
            coverage_map_.posToIndex(p, idx);
            return coverage_map_.isFree2D(idx(0), idx(1)) &&
                   coverage_map_.getDistance(p) >=
                       requiredTrajectoryClearance(p, traj_min_clearance);
        };

        for (int iter = 0; iter < 18; ++iter) {
            for (int i = 1; i < (int)astar_path_.size() - 1; ++i) {
                Eigen::Vector3d p = astar_path_[i];
                p(2) = fly_height_;
                Eigen::Vector3d prev = astar_path_[i - 1];
                Eigen::Vector3d next = astar_path_[i + 1];
                prev(2) = fly_height_;
                next(2) = fly_height_;

                Eigen::Vector3d smooth_delta = 0.25 * (0.5 * (prev + next) - p);
                Eigen::Vector3d ref_delta = 0.12 * (original[i] - p);

                Eigen::Vector3d obstacle_delta(0.0, 0.0, 0.0);
                double dist = coverage_map_.getDistance(p);
                if (dist < desired_clearance) {
                    Eigen::Vector3d px1 = p, px0 = p, py1 = p, py0 = p;
                    px1(0) += eps; px0(0) -= eps;
                    py1(1) += eps; py0(1) -= eps;
                    Eigen::Vector3d grad(
                        coverage_map_.getDistance(px1) - coverage_map_.getDistance(px0),
                        coverage_map_.getDistance(py1) - coverage_map_.getDistance(py0),
                        0.0);
                    if (grad.norm() > 1e-4) {
                        double lack = (desired_clearance - dist) /
                                      std::max(1e-3, desired_clearance - traj_min_clearance);
                        obstacle_delta = 0.70 * lack * lack * grad.normalized();
                    }
                }

                Eigen::Vector3d delta =
                    obstacle_delta + smooth_delta + ref_delta;
                delta(2) = 0.0;
                if (delta.norm() > max_step) delta = delta.normalized() * max_step;
                Eigen::Vector3d cand = p + delta;
                cand(2) = fly_height_;
                Eigen::Vector3d total_offset = cand - original[i];
                total_offset(2) = 0.0;
                if (total_offset.norm() > max_total_move) {
                    cand = original[i] + total_offset.normalized() * max_total_move;
                    cand(2) = fly_height_;
                }

                if (!pointSafe(cand)) continue;
                if (pathToTargetBlocked(cand, astar_path_[i - 1], traj_min_clearance)) continue;
                if (pathToTargetBlocked(astar_path_[i + 1], cand, traj_min_clearance)) continue;
                astar_path_[i] = cand;
            }
        }
    };
    auto buildLinearTraj = [&]() -> bool {
        std::vector<Eigen::Vector3d> pts;
        std::vector<Eigen::Vector3d> vels;
        std::vector<double> yaws;

        const double step_size = max(0.05, traj_step_size_);
        Eigen::Vector3d last_pt = uav_pos_;
        last_pt(2) = fly_height_;
        Eigen::Vector3i escape_start_idx;
        coverage_map_.posToIndex(last_pt, escape_start_idx);
        double escape_prev_clearance =
            coverage_map_.getDistance2D(escape_start_idx(0), escape_start_idx(1));
        bool escaping_start = start_requires_escape;
        auto rejectLinear = [&](const char *reason, int waypoint, int sample,
                                const Eigen::Vector3d &p, double clearance) {
            ROS_WARN_THROTTLE(1.0,
                "[CoverageSearch] Linear trajectory rejected: reason=%s waypoint=%d "
                "sample=%d pos=(%.2f,%.2f) clearance=%.2f goal=(%.2f,%.2f)",
                reason, waypoint, sample, p(0), p(1), clearance,
                current_goal_(0), current_goal_(1));
            return false;
        };

        for (int i = 0; i < (int)astar_path_.size(); i++) {
            Eigen::Vector3d pt = astar_path_[i];
            pt(2) = fly_height_;
            Eigen::Vector3d seg = pt - last_pt; seg(2) = 0;
            double seg_len = seg.norm();
            if (seg_len < 0.01) continue;
            int n_steps = max(1, (int)ceil(seg_len / step_size));

            Eigen::Vector3d prev_interp = last_pt;
            for (int s = 1; s <= n_steps; s++) {
                double t = (double)s / n_steps;
                Eigen::Vector3d interp = last_pt + t * seg;
                interp(2) = fly_height_;
                if (!coverage_map_.isInMap(interp)) {
                    return rejectLinear("outside_map", i, s, interp, -1.0);
                }
                Eigen::Vector3i interp_idx;
                coverage_map_.posToIndex(interp, interp_idx);
                bool free = coverage_map_.isFree2D(interp_idx(0), interp_idx(1));
                double clearance = coverage_map_.getDistance2D(interp_idx(0), interp_idx(1));
                bool blocked = false;
                const char *blocked_reason = "segment_blocked";
                if (escaping_start) {
                    bool in_start_cell = interp_idx(0) == escape_start_idx(0) &&
                                         interp_idx(1) == escape_start_idx(1);
                    if (!in_start_cell && !free) {
                        blocked = true;
                        blocked_reason = "escape_not_free";
                    } else if (!in_start_cell &&
                               clearance + 1e-3 < escape_prev_clearance) {
                        blocked = true;
                        blocked_reason = "escape_clearance_decreased";
                    } else if (!in_start_cell) {
                        escape_prev_clearance = clearance;
                        if (clearance >= requiredTrajectoryClearance(
                                interp, traj_min_clearance)) escaping_start = false;
                    }
                } else if (!free) {
                    blocked = true;
                    blocked_reason = "not_free";
                } else if (clearance < requiredTrajectoryClearance(
                               interp, traj_min_clearance)) {
                    blocked = true;
                    blocked_reason = "below_hard_clearance";
                } else if (pathToTargetBlocked(interp, prev_interp, traj_min_clearance)) {
                    blocked = true;
                    blocked_reason = "segment_blocked";
                }
                if (blocked && startup_bootstrap) {
                    blocked = segmentHasKnownObstacle(interp, prev_interp);
                    blocked_reason = "startup_known_obstacle";
                }
                if (blocked) {
                    return rejectLinear(blocked_reason, i, s, interp, clearance);
                }

                double yaw;
                if (s < n_steps) {
                    Eigen::Vector3d next_interp = last_pt + ((double)(s+1) / n_steps) * seg;
                    next_interp(2) = fly_height_;
                    Eigen::Vector3d dir = next_interp - interp; dir(2) = 0;
                    yaw = (dir.norm() > 1e-3) ? atan2(dir(1), dir(0)) : uav_yaw_;
                } else if (i < (int)astar_path_.size() - 1) {
                    Eigen::Vector3d next_wp = astar_path_[i + 1]; next_wp(2) = fly_height_;
                    Eigen::Vector3d dir = next_wp - interp; dir(2) = 0;
                    yaw = (dir.norm() > 1e-3) ? atan2(dir(1), dir(0)) : uav_yaw_;
                } else {
                    yaw = current_goal_yaw_;
                }
                pts.push_back(interp);
                yaws.push_back(yaw);
                vels.push_back(Eigen::Vector3d(0, 0, 0));
                prev_interp = interp;
            }
            last_pt = pt;
        }

        if (escaping_start) {
            return rejectLinear("escape_never_reached_hard_clearance",
                                (int)astar_path_.size() - 1, 0,
                                astar_path_.back(), escape_prev_clearance);
        }

	        if (pts.empty()) return false;
	        if (pts.size() > 4) {
	            std::vector<Eigen::Vector3d> smoothed = pts;
	            for (int iter = 0; iter < 2; ++iter) {
	                std::vector<Eigen::Vector3d> next;
	                next.push_back(smoothed.front());
	                for (int i = 0; i < (int)smoothed.size() - 1; ++i) {
	                    Eigen::Vector3d q = 0.75 * smoothed[i] + 0.25 * smoothed[i + 1];
	                    Eigen::Vector3d r = 0.25 * smoothed[i] + 0.75 * smoothed[i + 1];
	                    q(2) = fly_height_;
	                    r(2) = fly_height_;
	                    next.push_back(q);
	                    next.push_back(r);
	                }
	                next.push_back(smoothed.back());
	                smoothed.swap(next);
	            }

	            bool smooth_safe = true;
	            for (int i = 0; i < (int)smoothed.size(); ++i) {
	                Eigen::Vector3i sidx;
	                coverage_map_.posToIndex(smoothed[i], sidx);
		                if (!coverage_map_.isFree2D(sidx(0), sidx(1)) ||
	                        coverage_map_.getDistance(smoothed[i]) <
	                            requiredTrajectoryClearance(smoothed[i], traj_min_clearance)) {
		                    smooth_safe = false;
		                    break;
		                }
	                if (i > 0 && pathToTargetBlocked(
	                        smoothed[i], smoothed[i - 1], traj_min_clearance)) {
	                    smooth_safe = false;
	                    break;
	                }
	            }

	            if (smooth_safe) {
	                pts.swap(smoothed);
	                yaws.clear();
	                vels.clear();
	                for (int i = 0; i < (int)pts.size(); ++i) {
	                    Eigen::Vector3d dir;
	                    if (i + 1 < (int)pts.size()) dir = pts[i + 1] - pts[i];
	                    else dir = pts[i] - pts[i - 1];
	                    dir(2) = 0.0;
	                    yaws.push_back(dir.norm() > 1e-3 ? atan2(dir(1), dir(0)) : current_goal_yaw_);
	                    vels.push_back(Eigen::Vector3d(0, 0, 0));
	                }
	                if (!yaws.empty()) yaws.back() = current_goal_yaw_;
	            }
	        }
	        traj_points_.swap(pts);
	        traj_yaws_.swap(yaws);
	        traj_vels_.swap(vels);
	        return true;
	    };

    auto trajectoryPointSafe = [&](const Eigen::Vector3d &p) {
        if (!coverage_map_.isInMap(p)) return false;
        Eigen::Vector3i idx;
        coverage_map_.posToIndex(p, idx);
        return coverage_map_.isFree2D(idx(0), idx(1)) &&
               coverage_map_.getDistance2D(idx(0), idx(1)) >=
                   requiredTrajectoryClearance(p, traj_min_clearance);
    };
    auto trajectorySequenceSafe = [&](const std::vector<Eigen::Vector3d> &pts) {
        for (int i = 0; i < (int)pts.size(); ++i) {
            if (!trajectoryPointSafe(pts[i])) return false;
            if (i > 0 && pathToTargetBlocked(
                    pts[i], pts[i - 1], traj_min_clearance)) return false;
        }
        return !pts.empty();
    };
    auto buildPiecewiseBsplineTraj = [&]() -> bool {
        std::vector<Eigen::Vector3d> stitched;
        int spline_pieces = 0;
        int linear_pieces = 0;

        auto appendSamples = [&](const std::vector<Eigen::Vector3d> &samples) {
            if (!trajectorySequenceSafe(samples)) return false;
            int begin = !stitched.empty() &&
                        (stitched.back() - samples.front()).norm() < 1e-3 ? 1 : 0;
            stitched.insert(stitched.end(), samples.begin() + begin, samples.end());
            return true;
        };

        std::function<bool(int, int)> fitPiece = [&](int begin, int end) -> bool {
            if (end <= begin) return true;
            const int point_count = end - begin + 1;

            if (point_count >= 3) {
                std::vector<Eigen::Vector3d> ctrl;
                Eigen::Vector3d first = raw_astar_path[begin];
                Eigen::Vector3d last = raw_astar_path[end];
                first(2) = last(2) = fly_height_;
                for (int i = 0; i < 3; ++i) ctrl.push_back(first);
                for (int i = begin; i <= end; ++i) {
                    Eigen::Vector3d p = raw_astar_path[i];
                    p(2) = fly_height_;
                    ctrl.push_back(p);
                }
                for (int i = 0; i < 3; ++i) ctrl.push_back(last);

                std::vector<Eigen::Vector3d> samples;
                const int sample_count = std::max(12, point_count * 8);
                samples.reserve(sample_count);
                for (int i = 0; i < sample_count; ++i) {
                    Eigen::Vector3d p = evaluateBspline(
                        ctrl, (double)i / (double)(sample_count - 1));
                    p(2) = fly_height_;
                    samples.push_back(p);
                }
                if (appendSamples(samples)) {
                    ++spline_pieces;
                    return true;
                }
            }

            if (point_count == 2) {
                Eigen::Vector3d from = raw_astar_path[begin];
                Eigen::Vector3d to = raw_astar_path[end];
                from(2) = to(2) = fly_height_;
                const double length = (to - from).norm();
                const int sample_count = std::max(2, (int)ceil(
                    length / std::max(0.05, std::min(traj_step_size_, max_vel_ * traj_dt_))) + 1);
                std::vector<Eigen::Vector3d> samples;
                samples.reserve(sample_count);
                for (int i = 0; i < sample_count; ++i) {
                    Eigen::Vector3d p = from + (double)i / (sample_count - 1) * (to - from);
                    p(2) = fly_height_;
                    samples.push_back(p);
                }
                if (!appendSamples(samples)) return false;
                ++linear_pieces;
                return true;
            }

            const int middle = (begin + end) / 2;
            return fitPiece(begin, middle) && fitPiece(middle, end);
        };

        if (!fitPiece(0, (int)raw_astar_path.size() - 1) || stitched.empty()) return false;

        bool join_smoothing_applied = false;
        for (int iter = 0; iter < 2 && stitched.size() > 3; ++iter) {
            std::vector<Eigen::Vector3d> smoothed;
            smoothed.reserve(2 * stitched.size());
            smoothed.push_back(stitched.front());
            for (int i = 0; i + 1 < (int)stitched.size(); ++i) {
                Eigen::Vector3d q = 0.75 * stitched[i] + 0.25 * stitched[i + 1];
                Eigen::Vector3d r = 0.25 * stitched[i] + 0.75 * stitched[i + 1];
                q(2) = r(2) = fly_height_;
                smoothed.push_back(q);
                smoothed.push_back(r);
            }
            smoothed.push_back(stitched.back());
            if (!trajectorySequenceSafe(smoothed)) break;
            stitched.swap(smoothed);
            join_smoothing_applied = true;
        }

        traj_points_.swap(stitched);
        traj_yaws_.assign(traj_points_.size(), uav_yaw_);
        traj_vels_.assign(traj_points_.size(), Eigen::Vector3d::Zero());
        ROS_WARN("[CoverageSearch] Piecewise B-spline recovery succeeded: spline_pieces=%d, "
                 "straight_safety_pieces=%d, join_smoothing=%s, points=%zu.",
                 spline_pieces, linear_pieces,
                 join_smoothing_applied ? "yes" : "no", traj_points_.size());
        return true;
    };

    if (!start_requires_escape) optimizeAstarPathByESDF();

    if (start_requires_escape) {
        astar_path_ = raw_astar_path;
        traj_type = "Linear escape";
        if (!buildLinearTraj()) {
            has_traj_ = false;
            cout << RED << "[CoverageSearch] Failed to generate safe linear trajectory." << TAIL << endl;
            return;
        }
    } else if ((int)astar_path_.size() < 4) {
        traj_type = "Piecewise Bspline";
        if (!buildPiecewiseBsplineTraj()) {
            astar_path_ = raw_astar_path;
            traj_type = "Linear safety fallback";
            if (!buildLinearTraj()) {
                has_traj_ = false;
                cout << RED << "[CoverageSearch] Failed to generate safe short trajectory."
                     << TAIL << endl;
                return;
            }
        }
    } else {
        // ★ 三次B样条平滑
        // 1. 准备钳制控制点，保证曲线经过起点和目标视点
        std::vector<Eigen::Vector3d> ctrl_pts;
        Eigen::Vector3d first = astar_path_.front();
        first(2) = fly_height_;
        for (int i = 0; i < 3; ++i) ctrl_pts.push_back(first);
        for (auto &pt : astar_path_) {
            Eigen::Vector3d cp = pt; cp(2) = fly_height_;
            ctrl_pts.push_back(cp);
        }
        Eigen::Vector3d last = astar_path_.back();
        last(2) = fly_height_;
        for (int i = 0; i < 3; ++i) ctrl_pts.push_back(last);

        auto optimizeBsplineControlPoints = [&]() {
            if ((int)ctrl_pts.size() < 9) return;

            std::vector<Eigen::Vector3d> original = ctrl_pts;
            const double desired_clearance = preferred_clearance;
            const double eps = std::max(0.05, 0.5 * coverage_map_.resolution_);
            const double max_step = std::max(0.05, 0.5 * coverage_map_.resolution_);
            const double max_total_move = 0.5;

            auto pointSafe = [&](const Eigen::Vector3d &p) {
                if (!coverage_map_.isInMap(p)) return false;
                Eigen::Vector3i idx;
                coverage_map_.posToIndex(p, idx);
                return coverage_map_.isFree2D(idx(0), idx(1)) &&
                       coverage_map_.getDistance(p) >=
                           requiredTrajectoryClearance(p, traj_min_clearance);
            };

            for (int iter = 0; iter < 14; ++iter) {
                for (int i = 4; i < (int)ctrl_pts.size() - 4; ++i) {
                    Eigen::Vector3d p = ctrl_pts[i];
                    Eigen::Vector3d prev = ctrl_pts[i - 1];
                    Eigen::Vector3d next = ctrl_pts[i + 1];
                    p(2) = prev(2) = next(2) = fly_height_;

                    Eigen::Vector3d smooth_delta = 0.18 * (0.5 * (prev + next) - p);
                    Eigen::Vector3d ref_delta = 0.10 * (original[i] - p);

                    Eigen::Vector3d length_grad(0.0, 0.0, 0.0);
                    Eigen::Vector3d from_prev = p - prev;
                    Eigen::Vector3d from_next = p - next;
                    from_prev(2) = 0.0;
                    from_next(2) = 0.0;
                    if (from_prev.norm() > 1e-3) length_grad += from_prev.normalized();
                    if (from_next.norm() > 1e-3) length_grad += from_next.normalized();
                    Eigen::Vector3d length_delta = -0.04 * length_grad;

                    Eigen::Vector3d dyn_delta = 0.08 * (prev - 2.0 * p + next);
                    dyn_delta(2) = 0.0;

                    Eigen::Vector3d obstacle_delta(0.0, 0.0, 0.0);
                    double dist = coverage_map_.getDistance(p);
                    if (dist < desired_clearance) {
                        Eigen::Vector3d px1 = p, px0 = p, py1 = p, py0 = p;
                        px1(0) += eps; px0(0) -= eps;
                        py1(1) += eps; py0(1) -= eps;
                        Eigen::Vector3d grad(
                            coverage_map_.getDistance(px1) - coverage_map_.getDistance(px0),
                            coverage_map_.getDistance(py1) - coverage_map_.getDistance(py0),
                            0.0);
                        if (grad.norm() > 1e-4) {
                            double lack = (desired_clearance - dist) /
                                          std::max(1e-3, desired_clearance - traj_min_clearance);
                            obstacle_delta = 0.70 * lack * lack * grad.normalized();
                        }
                    }

                    Eigen::Vector3d delta = obstacle_delta + smooth_delta + length_delta +
                                            ref_delta + dyn_delta;
                    delta(2) = 0.0;
                    if (delta.norm() > max_step) delta = delta.normalized() * max_step;

                    Eigen::Vector3d cand = p + delta;
                    cand(2) = fly_height_;
                    Eigen::Vector3d total_offset = cand - original[i];
                    total_offset(2) = 0.0;
                    if (total_offset.norm() > max_total_move) {
                        cand = original[i] + total_offset.normalized() * max_total_move;
                        cand(2) = fly_height_;
                    }
                    if (!pointSafe(cand)) continue;
                    if (pathToTargetBlocked(
                            cand, ctrl_pts[i - 1], traj_min_clearance)) continue;
                    if (pathToTargetBlocked(
                            ctrl_pts[i + 1], cand, traj_min_clearance)) continue;
                    ctrl_pts[i] = cand;
                }
            }
        };
        optimizeBsplineControlPoints();

        // 2. 在B样条曲线上均匀采样
        int n_samples = max(20, (int)(ctrl_pts.size() * 8));
        bool bspline_safe = true;
        std::string bspline_fail_reason;
        int bspline_fail_sample = -1;
        Eigen::Vector3d bspline_fail_pos = Eigen::Vector3d::Zero();
        double bspline_fail_clearance = -1.0;
        std::vector<Eigen::Vector3d> pts;
        std::vector<Eigen::Vector3d> vels;
        std::vector<double> yaws;

        for (int i = 0; i < n_samples; i++) {
            double t = (double)i / (n_samples - 1);
            Eigen::Vector3d pt = evaluateBspline(ctrl_pts, t);
            pt(2) = fly_height_;

            // 3. 安全检查：轨迹点必须在已知free区域
            Eigen::Vector3i pt_idx;
            coverage_map_.posToIndex(pt, pt_idx);
	            bool pt_free = coverage_map_.isFree2D(pt_idx(0), pt_idx(1));
	            double pt_clearance = coverage_map_.getDistance2D(pt_idx(0), pt_idx(1));
	            if (!pt_free || pt_clearance <
	                    requiredTrajectoryClearance(pt, traj_min_clearance)) {
	                bspline_safe = false;
	                bspline_fail_reason = pt_free ? "below_hard_clearance" : "not_free";
	                bspline_fail_sample = i;
	                bspline_fail_pos = pt;
	                bspline_fail_clearance = pt_clearance;
	                break;
	            }
	            if (!pts.empty() && pathToTargetBlocked(
	                    pt, pts.back(), traj_min_clearance)) {
	                bspline_safe = false;
	                bspline_fail_reason = "segment_blocked";
	                bspline_fail_sample = i;
	                bspline_fail_pos = pt;
	                bspline_fail_clearance = pt_clearance;
	                break;
	            }

            // 4. 动力学约束：相邻点间距不超过max_vel_ * dt
            if (!pts.empty()) {
                Eigen::Vector3d last = pts.back();
                double dist = (pt - last).norm();
                double max_dist = max(0.05, max_vel_ * traj_dt_);
                if (dist > max_dist) {
                    // 插入中间点
                    int n_fill = max(1, (int)ceil(dist / max_dist));
                    for (int f = 1; f <= n_fill; f++) {
                        double ft = (double)f / n_fill;
                        Eigen::Vector3d fill_pt = last + ft * (pt - last);
                        fill_pt(2) = fly_height_;
                        Eigen::Vector3i fill_idx;
                        coverage_map_.posToIndex(fill_pt, fill_idx);
	                        bool fill_free = coverage_map_.isFree2D(fill_idx(0), fill_idx(1));
	                        double fill_clearance =
	                            coverage_map_.getDistance2D(fill_idx(0), fill_idx(1));
	                        if (!fill_free || fill_clearance <
	                                requiredTrajectoryClearance(fill_pt, traj_min_clearance)) {
	                            bspline_safe = false;
	                            bspline_fail_reason = fill_free ? "fill_below_hard_clearance"
	                                                              : "fill_not_free";
	                            bspline_fail_sample = i;
	                            bspline_fail_pos = fill_pt;
	                            bspline_fail_clearance = fill_clearance;
	                            break;
	                        }
                        pts.push_back(fill_pt);
                        Eigen::Vector3d dir = fill_pt - last; dir(2) = 0;
                        double yaw = (dir.norm() > 1e-3) ? atan2(dir(1), dir(0)) : uav_yaw_;
                        yaws.push_back(yaw);
                        vels.push_back(Eigen::Vector3d(0, 0, 0));
                        last = fill_pt;
                    }
                    if (!bspline_safe) break;
                } else if (dist < 0.01) {
                    continue;  // 跳过太近的点
                } else {
                    pts.push_back(pt);
                    Eigen::Vector3d dir = pt - last; dir(2) = 0;
                    double yaw = (dir.norm() > 1e-3) ? atan2(dir(1), dir(0)) : uav_yaw_;
                    yaws.push_back(yaw);
                    vels.push_back(Eigen::Vector3d(0, 0, 0));
                }
            } else {
                pts.push_back(pt);
                yaws.push_back(uav_yaw_);
                vels.push_back(Eigen::Vector3d(0, 0, 0));
            }
        }

        if (bspline_safe && !pts.empty()) {
            traj_points_.swap(pts);
            traj_yaws_.swap(yaws);
            traj_vels_.swap(vels);
        } else {
            traj_type = "Piecewise Bspline";
            ROS_WARN_THROTTLE(1.0,
                "[CoverageSearch] B-spline rejected: reason=%s sample=%d "
                "pos=(%.2f,%.2f) clearance=%.2f; retry as safe pieces.",
                bspline_fail_reason.empty() ? "empty_trajectory" : bspline_fail_reason.c_str(),
                bspline_fail_sample, bspline_fail_pos(0), bspline_fail_pos(1),
                bspline_fail_clearance);
            astar_path_ = raw_astar_path;
            if (!buildPiecewiseBsplineTraj()) {
                traj_type = "Linear safety fallback";
                if (!buildLinearTraj()) {
                    has_traj_ = false;
                    cout << RED << "[CoverageSearch] Failed to generate safe piecewise or linear "
                         << "trajectory from raw A* path." << TAIL << endl;
                    return;
                }
            }
        }
    }

    traj_idx_ = 0;
    has_traj_ = !traj_points_.empty();
    if (has_traj_) {
        const int n = (int)traj_points_.size();
        const double yaw_delta = atan2(sin(current_goal_yaw_ - uav_yaw_),
                                       cos(current_goal_yaw_ - uav_yaw_));
        const double yaw_goal_unwrapped = uav_yaw_ + yaw_delta;
        std::vector<double> raw_arc(raw_astar_path.size(), 0.0);
        for (int i = 1; i < (int)raw_astar_path.size(); ++i) {
            raw_arc[i] = raw_arc[i - 1] +
                         (raw_astar_path[i] - raw_astar_path[i - 1]).head<2>().norm();
        }
        const double raw_total = raw_arc.empty() ? 0.0 : raw_arc.back();
        std::vector<Eigen::Vector3d> yaw_ctrl;
        for (int i = 0; i < 3; ++i) yaw_ctrl.emplace_back(uav_yaw_, 0.0, 0.0);
        for (int i = 0; i < (int)raw_astar_path.size(); ++i) {
            double s = raw_total > 1e-3 ? raw_arc[i] / raw_total
                                        : (double)i / std::max(1, (int)raw_astar_path.size() - 1);
            yaw_ctrl.emplace_back(uav_yaw_ + s * yaw_delta, 0.0, 0.0);
        }
        for (int i = 0; i < 3; ++i) yaw_ctrl.emplace_back(yaw_goal_unwrapped, 0.0, 0.0);

        std::vector<double> traj_arc(n, 0.0);
        for (int i = 1; i < n; ++i) {
            traj_arc[i] = traj_arc[i - 1] +
                          (traj_points_[i] - traj_points_[i - 1]).head<2>().norm();
        }
        const double traj_total = traj_arc.back();
        traj_yaws_.clear();
        traj_yaws_.reserve(n);
        for (int i = 0; i < n; ++i) {
            double s = traj_total > 1e-3 ? traj_arc[i] / traj_total : 1.0;
            double yaw = evaluateBspline(yaw_ctrl, s)(0);
            traj_yaws_.push_back(atan2(sin(yaw), cos(yaw)));
        }
        traj_yaws_.front() = uav_yaw_;
        traj_yaws_.back() = current_goal_yaw_;
        ROS_INFO("[CoverageSearch] Coupled yaw B-spline: start=%.2f, goal=%.2f, delta=%.2f rad.",
                 uav_yaw_, current_goal_yaw_, yaw_delta);

        // 对几何轨迹做一次前后向速度规划，供原生 TRAJECTORY 模式同时跟踪 P/V/A。
        traj_vels_.assign(n, Eigen::Vector3d::Zero());
        traj_accs_.assign(n, Eigen::Vector3d::Zero());
        if (n >= 2) {
            std::vector<double> segment_lengths(n - 1, 0.0);
            std::vector<Eigen::Vector3d> tangents(n, Eigen::Vector3d::Zero());
            for (int i = 0; i + 1 < n; ++i) {
                Eigen::Vector3d segment = traj_points_[i + 1] - traj_points_[i];
                segment(2) = 0.0;
                segment_lengths[i] = segment.norm();
            }
            for (int i = 0; i < n; ++i) {
                Eigen::Vector3d tangent;
                if (i == 0) tangent = traj_points_[1] - traj_points_[0];
                else if (i == n - 1) tangent = traj_points_[n - 1] - traj_points_[n - 2];
                else tangent = traj_points_[i + 1] - traj_points_[i - 1];
                tangent(2) = 0.0;
                if (tangent.norm() > 1e-4) tangents[i] = tangent.normalized();
                else if (i > 0) tangents[i] = tangents[i - 1];
            }

            std::vector<double> speeds(n, max_vel_);
            for (int i = 1; i + 1 < n; ++i) {
                double turn = acos(std::max(-1.0, std::min(1.0,
                    tangents[i - 1].dot(tangents[i + 1]))));
                double arc = std::max(0.05,
                    0.5 * (segment_lengths[i - 1] + segment_lengths[i]));
                double curvature = turn / arc;
                if (curvature > 1e-3) {
                    speeds[i] = std::min(speeds[i], sqrt(max_acc_ / curvature));
                }

                double clearance = coverage_map_.getDistance(traj_points_[i]);
                if (clearance < preferred_clearance) {
                    const double local_min_clearance =
                        requiredTrajectoryClearance(traj_points_[i], traj_min_clearance);
                    double ratio = (clearance - local_min_clearance) /
                                   std::max(1e-3, preferred_clearance - local_min_clearance);
                    ratio = std::max(0.0, std::min(1.0, ratio));
                    speeds[i] = std::min(speeds[i], max_vel_ * (0.25 + 0.75 * ratio));
                }
            }

            double initial_speed = std::max(0.0, uav_vel_.dot(tangents.front()));
            speeds.front() = std::min(speeds.front(), initial_speed);
            for (int i = 1; i < n; ++i) {
                speeds[i] = std::min(speeds[i], sqrt(
                    speeds[i - 1] * speeds[i - 1] +
                    2.0 * max_acc_ * segment_lengths[i - 1]));
            }
            speeds.back() = 0.0;
            for (int i = n - 2; i >= 0; --i) {
                speeds[i] = std::min(speeds[i], sqrt(
                    speeds[i + 1] * speeds[i + 1] +
                    2.0 * max_acc_ * segment_lengths[i]));
            }
            for (int i = 0; i < n; ++i) traj_vels_[i] = speeds[i] * tangents[i];

            std::vector<double> segment_times(n - 1, traj_dt_);
            for (int i = 0; i + 1 < n; ++i) {
                double avg_speed = 0.5 * (speeds[i] + speeds[i + 1]);
                if (segment_lengths[i] > 1e-4 && avg_speed > 1e-3) {
                    segment_times[i] = std::max(0.05, segment_lengths[i] / avg_speed);
                }
            }
            traj_accs_[0] = (traj_vels_[1] - traj_vels_[0]) / segment_times[0];
            for (int i = 1; i + 1 < n; ++i) {
                traj_accs_[i] = (traj_vels_[i + 1] - traj_vels_[i - 1]) /
                                (segment_times[i - 1] + segment_times[i]);
            }
            traj_accs_[n - 1] =
                (traj_vels_[n - 1] - traj_vels_[n - 2]) / segment_times[n - 2];
            for (auto &acc : traj_accs_) {
                if (acc.norm() > max_acc_) acc *= max_acc_ / acc.norm();
            }
            traj_vels_.back().setZero();
            traj_accs_.back().setZero();
        }

        bool profile_valid = traj_vels_.size() == traj_points_.size() &&
                             traj_accs_.size() == traj_points_.size();
        for (int i = 0; profile_valid && i < n; ++i) {
            profile_valid = traj_points_[i].allFinite() && traj_vels_[i].allFinite() &&
                            traj_accs_[i].allFinite() && std::isfinite(traj_yaws_[i]);
        }
        ROS_ASSERT_MSG(profile_valid, "Coverage trajectory P/V/A profile is inconsistent");
        if (!profile_valid) {
            has_traj_ = false;
            return;
        }

        const Eigen::Vector3d start_acc = planning_start_state_valid_
            ? planning_start_acc_ : Eigen::Vector3d::Zero();
        const double start_yaw_rate = planning_start_state_valid_
            ? planning_start_yaw_rate_ : 0.0;
        const double start_yaw_acc = planning_start_state_valid_
            ? planning_start_yaw_acc_ : 0.0;
        const bool time_spline_ready = buildTimeParameterizedSpline(
            start_acc, start_yaw_rate, start_yaw_acc);
        planning_start_state_valid_ = false;
        if (!time_spline_ready && rolling_prepare_in_progress_) {
            has_traj_ = false;
            ROS_WARN("[CoverageSearch] Reject rolling successor: it cannot preserve "
                     "the De Boor handoff P/V/A/yaw state safely.");
            return;
        }
        if (!time_spline_ready) {
            ROS_WARN("[CoverageSearch] Keep safety-checked legacy samples for this trajectory; "
                     "rolling handoff is disabled until the next time B-spline succeeds.");
        }
    }
    traj_start_time_ = ros::Time::now();
    traj_point_reach_time_ = ros::Time::now();
    cmd_yaw_inited_ = false;
    if (!rolling_snapshot_mode_) armRealtimeTrajectory();

    cout << GREEN << "[CoverageSearch] Trajectory generated: "
         << traj_points_.size() << " points from "
         << astar_path_.size() << " waypoints (" << traj_type << ")." << TAIL << endl;
}

bool CoverageSearchManager::buildContinuousBridge(
    const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel,
    const Eigen::Vector3d &start_acc, double start_yaw, double start_yaw_rate,
    double start_yaw_acceleration) {
    if (traj_points_.size() < 3 || traj_vels_.size() != traj_points_.size() ||
        traj_accs_.size() != traj_points_.size() ||
        traj_yaws_.size() != traj_points_.size()) {
        return false;
    }

    // 跳过新轨迹紧邻起点的一小段，用五次多项式同时匹配交接点和新轨迹的 P/V/A。
    int join_idx = 1;
    double join_arc = 0.0;
    const double desired_join_arc = std::max(0.65, 0.8 * max_vel_);
    while (join_idx < (int)traj_points_.size() - 1 &&
           join_arc < desired_join_arc) {
        join_arc += (traj_points_[join_idx] - traj_points_[join_idx - 1]).head<2>().norm();
        ++join_idx;
    }
    join_idx = std::min(join_idx, (int)traj_points_.size() - 1);

    const Eigen::Vector3d end_pos = traj_points_[join_idx];
    const Eigen::Vector3d end_vel = traj_vels_[join_idx];
    const Eigen::Vector3d end_acc = traj_accs_[join_idx];
    const double end_yaw_delta = std::atan2(
        std::sin(traj_yaws_[join_idx] - start_yaw),
        std::cos(traj_yaws_[join_idx] - start_yaw));
    const double end_yaw = start_yaw + end_yaw_delta;

    double end_yaw_rate = 0.0;
    if (join_idx + 1 < (int)traj_yaws_.size()) {
        const double dt = std::max(1e-3, traj_dt_);
        end_yaw_rate = std::atan2(
            std::sin(traj_yaws_[join_idx + 1] - traj_yaws_[join_idx]),
            std::cos(traj_yaws_[join_idx + 1] - traj_yaws_[join_idx])) / dt;
    }
    double end_yaw_acceleration = 0.0;
    if (join_idx > 0 && join_idx + 1 < (int)traj_yaws_.size()) {
        const double dt = std::max(1e-3, traj_dt_);
        const double yaw_rate_before = std::atan2(
            std::sin(traj_yaws_[join_idx] - traj_yaws_[join_idx - 1]),
            std::cos(traj_yaws_[join_idx] - traj_yaws_[join_idx - 1])) / dt;
        const double yaw_rate_after = std::atan2(
            std::sin(traj_yaws_[join_idx + 1] - traj_yaws_[join_idx]),
            std::cos(traj_yaws_[join_idx + 1] - traj_yaws_[join_idx])) / dt;
        end_yaw_acceleration = (yaw_rate_after - yaw_rate_before) / dt;
    }
    start_yaw_rate = std::max(-max_yaw_rate_, std::min(max_yaw_rate_, start_yaw_rate));
    end_yaw_rate = std::max(-max_yaw_rate_, std::min(max_yaw_rate_, end_yaw_rate));
    start_yaw_acceleration = std::max(-max_yaw_acc_,
                                      std::min(max_yaw_acc_, start_yaw_acceleration));
    end_yaw_acceleration = std::max(-max_yaw_acc_,
                                    std::min(max_yaw_acc_, end_yaw_acceleration));

    const double distance = (end_pos - start_pos).head<2>().norm();
    const double initial_duration = std::max(
        0.9, std::min(2.0, 1.35 * distance / std::max(0.35, max_vel_)));

    std::vector<Eigen::Vector3d> bridge_points;
    std::vector<Eigen::Vector3d> bridge_vels;
    std::vector<Eigen::Vector3d> bridge_accs;
    std::vector<double> bridge_yaws;
    double accepted_duration = 0.0;

    for (int duration_try = 0; duration_try < 9; ++duration_try) {
        const double T = initial_duration + 0.25 * duration_try;
        const double T2 = T * T;
        const double T3 = T2 * T;
        const double T4 = T3 * T;
        const double T5 = T4 * T;

        Eigen::Vector3d c0 = start_pos;
        Eigen::Vector3d c1 = start_vel;
        Eigen::Vector3d c2 = 0.5 * start_acc;
        Eigen::Vector3d c3 = (20.0 * (end_pos - start_pos) -
            (8.0 * end_vel + 12.0 * start_vel) * T -
            (3.0 * start_acc - end_acc) * T2) / (2.0 * T3);
        Eigen::Vector3d c4 = (30.0 * (start_pos - end_pos) +
            (14.0 * end_vel + 16.0 * start_vel) * T +
            (3.0 * start_acc - 2.0 * end_acc) * T2) / (2.0 * T4);
        Eigen::Vector3d c5 = (12.0 * (end_pos - start_pos) -
            (6.0 * end_vel + 6.0 * start_vel) * T -
            (start_acc - end_acc) * T2) / (2.0 * T5);

        const double y0 = start_yaw;
        const double y1 = start_yaw_rate;
        const double y2 = 0.5 * start_yaw_acceleration;
        const double y3 = (20.0 * (end_yaw - start_yaw) -
            (8.0 * end_yaw_rate + 12.0 * start_yaw_rate) * T -
            (3.0 * start_yaw_acceleration - end_yaw_acceleration) * T2) /
            (2.0 * T3);
        const double y4 = (30.0 * (start_yaw - end_yaw) +
            (14.0 * end_yaw_rate + 16.0 * start_yaw_rate) * T +
            (3.0 * start_yaw_acceleration - 2.0 * end_yaw_acceleration) * T2) /
            (2.0 * T4);
        const double y5 = (12.0 * (end_yaw - start_yaw) -
            (6.0 * end_yaw_rate + 6.0 * start_yaw_rate) * T -
            (start_yaw_acceleration - end_yaw_acceleration) * T2) /
            (2.0 * T5);

        const int samples = std::max(4, (int)std::ceil(T / traj_dt_));
        const double sample_dt = T / samples;
        bridge_points.clear();
        bridge_vels.clear();
        bridge_accs.clear();
        bridge_yaws.clear();
        bool safe = true;
        Eigen::Vector3d previous = start_pos;
        double previous_unwrapped_yaw = start_yaw;

        for (int i = 0; i <= samples; ++i) {
            const double t = sample_dt * i;
            const double t2 = t * t;
            const double t3 = t2 * t;
            const double t4 = t3 * t;
            const double t5 = t4 * t;
            Eigen::Vector3d p = c0 + c1 * t + c2 * t2 + c3 * t3 + c4 * t4 + c5 * t5;
            Eigen::Vector3d v = c1 + 2.0 * c2 * t + 3.0 * c3 * t2 +
                                4.0 * c4 * t3 + 5.0 * c5 * t4;
            Eigen::Vector3d a = 2.0 * c2 + 6.0 * c3 * t +
                                12.0 * c4 * t2 + 20.0 * c5 * t3;
            const double yaw_unwrapped = y0 + y1 * t + y2 * t2 +
                y3 * t3 + y4 * t4 + y5 * t5;
            const double yaw_rate = y1 + 2.0 * y2 * t + 3.0 * y3 * t2 +
                4.0 * y4 * t3 + 5.0 * y5 * t4;
            const double yaw_acceleration = 2.0 * y2 + 6.0 * y3 * t +
                12.0 * y4 * t2 + 20.0 * y5 * t3;
            p(2) = fly_height_;
            v(2) = 0.0;
            a(2) = 0.0;

            Eigen::Vector3i idx;
            coverage_map_.posToIndex(p, idx);
            const bool point_safe = coverage_map_.isInMap2D(idx(0), idx(1)) &&
                coverage_map_.isFree2D(idx(0), idx(1)) &&
                coverage_map_.getDistance2D(idx(0), idx(1)) + 1e-3 >=
                    requiredTrajectoryClearance(p, traj_cut_clearance_);
            if (!point_safe || v.head<2>().norm() > 1.05 * max_vel_ ||
                a.head<2>().norm() > 1.05 * max_acc_ ||
                std::fabs(yaw_rate) > 1.05 * max_yaw_rate_ ||
                std::fabs(yaw_acceleration) > 1.05 * max_yaw_acc_ ||
                (i > 0 && pathToTargetBlocked(p, previous, traj_cut_clearance_))) {
                safe = false;
                break;
            }
            if (i > 0) {
                const double sampled_rate = std::fabs(std::atan2(
                    std::sin(yaw_unwrapped - previous_unwrapped_yaw),
                    std::cos(yaw_unwrapped - previous_unwrapped_yaw))) / sample_dt;
                if (sampled_rate > 1.05 * max_yaw_rate_) {
                    safe = false;
                    break;
                }
            }

            bridge_points.push_back(p);
            bridge_vels.push_back(v);
            bridge_accs.push_back(a);
            bridge_yaws.push_back(std::atan2(std::sin(yaw_unwrapped),
                                             std::cos(yaw_unwrapped)));
            previous = p;
            previous_unwrapped_yaw = yaw_unwrapped;
        }

        if (safe) {
            accepted_duration = T;
            break;
        }
    }

    if (bridge_points.empty() || accepted_duration <= 0.0) {
        ROS_WARN("[CoverageSearch] Rolling C2 bridge rejected by dynamics or ESDF safety.");
        return false;
    }

    for (int i = join_idx + 1; i < (int)traj_points_.size(); ++i) {
        bridge_points.push_back(traj_points_[i]);
        bridge_vels.push_back(traj_vels_[i]);
        bridge_accs.push_back(traj_accs_[i]);
        bridge_yaws.push_back(traj_yaws_[i]);
    }
    traj_points_.swap(bridge_points);
    traj_vels_.swap(bridge_vels);
    traj_accs_.swap(bridge_accs);
    traj_yaws_.swap(bridge_yaws);
    traj_idx_ = 0;

    ROS_INFO("[CoverageSearch] C2 bridge accepted: duration=%.2fs, join=%d, "
             "P/V/A/yaw continuous, output=%zu points.",
             accepted_duration, join_idx, traj_points_.size());
    return true;
}

bool CoverageSearchManager::activatePendingTrajectory() {
    if (!pending_traj_.ready || pending_traj_.points.empty()) return false;

    const ros::Time now = ros::Time::now();
    const bool timed_handoff = active_time_spline_.valid &&
        pending_traj_.time_spline.valid && pending_traj_.handoff_time >= 0.0;
    ros::Time handoff_stamp;
    double successor_elapsed = 0.0;
    Eigen::Vector3d successor_pos = pending_traj_.points.front();
    Eigen::Vector3d successor_vel = pending_traj_.vels.front();
    Eigen::Vector3d successor_acc = Eigen::Vector3d::Zero();
    double successor_yaw = pending_traj_.yaws.front();
    double successor_yaw_rate = 0.0, successor_yaw_acc = 0.0;
    bool missed_handoff = false;
    bool successor_state_valid = true;
    if (timed_handoff) {
        handoff_stamp = traj_start_time_ + ros::Duration(pending_traj_.handoff_time);
        successor_elapsed = std::max(0.0, (now - handoff_stamp).toSec());
        missed_handoff = !pending_traj_.result_ready_time.isZero() &&
            pending_traj_.result_ready_time > handoff_stamp;
        if (successor_elapsed > pending_traj_.time_spline.duration ||
            !evaluateTimeBspline(pending_traj_.time_spline, successor_elapsed,
                                 successor_pos, successor_vel, successor_acc,
                                 successor_yaw, successor_yaw_rate,
                                 successor_yaw_acc)) {
            successor_state_valid = false;
            missed_handoff = true;
        }
    } else {
        missed_handoff = traj_idx_ > pending_traj_.handoff_idx + 1;
    }

    const double tracking_error =
        (uav_pos_.head<2>() - successor_pos.head<2>()).norm();
    const double velocity_error =
        (uav_vel_.head<2>() - successor_vel.head<2>()).norm();
    const double yaw_error = std::fabs(std::atan2(
        std::sin(successor_yaw - uav_yaw_), std::cos(successor_yaw - uav_yaw_)));
    const double max_switch_error = std::max(0.45, 1.5 * traj_advance_dist_);
    bool frontier_still_present = false;
    bool peer_owns_goal = false;
    for (const auto &frontier : frontier_finder_.frontiers_) {
        if (frontierTaskId(frontier) == pending_traj_.task_id) {
            for (const auto &viewpoint : frontier.viewpoints) {
                if ((viewpoint.head<2>() - pending_traj_.goal.head<2>()).norm() < 0.10) {
                    frontier_still_present = true;
                    break;
                }
            }
        }
        for (const auto &viewpoint : frontier.viewpoints) {
            if ((viewpoint.head<2>() - pending_traj_.goal.head<2>()).norm() < 0.30 &&
                isFrontierLeasedToPeer(frontier)) {
                peer_owns_goal = true;
                break;
            }
        }
        if (peer_owns_goal) break;
    }
    Eigen::Vector3i goal_idx;
    coverage_map_.posToIndex(pending_traj_.goal, goal_idx);
    const bool goal_still_free = coverage_map_.isInMap(pending_traj_.goal) &&
        coverage_map_.isInMap2D(goal_idx(0), goal_idx(1)) &&
        coverage_map_.isFree2D(goal_idx(0), goal_idx(1)) &&
        coverage_map_.getDistance2D(goal_idx(0), goal_idx(1)) + 1e-3 >=
            requiredTrajectoryClearance(pending_traj_.goal, traj_cut_clearance_);
    // A frontier can be re-clustered between planning and handoff.  Its exact
    // task ID is then stale, but the already planned route remains reusable if
    // the target is still free and no peer owns the nearby frontier.
    bool atsp_tour_valid = true;
    if (pending_traj_.atsp_tour_active) {
        for (int index = pending_traj_.frontier_target_idx;
             index < static_cast<int>(pending_traj_.frontier_targets.size()); ++index) {
            if (!isAtspTargetValid(pending_traj_.frontier_targets[index],
                                   pending_traj_.frontier_target_yaws[index],
                                   pending_traj_.frontier_target_task_ids[index])) {
                atsp_tour_valid = false;
                break;
            }
        }
    }
    const bool goal_reusable = pending_traj_.atsp_tour_active ? atsp_tour_valid :
        !peer_owns_goal && (frontier_still_present || goal_still_free);
    if (!successor_state_valid || !goal_reusable) {
        ROS_WARN("[CoverageSearch] Rolling successor discarded: valid=%s, frontier=%s, "
                 "goal_free=%s, peer_owned=%s, atsp_tour=%s.",
                 successor_state_valid ? "yes" : "no",
                 frontier_still_present ? "fresh" : "stale",
                 goal_still_free ? "yes" : "no",
                 peer_owns_goal ? "yes" : "no",
                 atsp_tour_valid ? "valid" : "invalid");
        pending_traj_ = PendingTrajectory();
        return false;
    }

    const bool direct_handoff = timed_handoff && !missed_handoff &&
        tracking_error <= max_switch_error && velocity_error <= 0.30 &&
        yaw_error <= 0.20;

    const Eigen::Vector3d old_goal = current_goal_;
    const double old_goal_yaw = current_goal_yaw_;
    const uint64_t old_goal_task_id = current_goal_task_id_;
    const std::vector<Eigen::Vector3d> old_astar_path = astar_path_;
    const std::vector<Eigen::Vector3d> old_points = traj_points_;
    const std::vector<Eigen::Vector3d> old_vels = traj_vels_;
    const std::vector<Eigen::Vector3d> old_accs = traj_accs_;
    const std::vector<double> old_yaws = traj_yaws_;
    const TimeBspline old_spline = active_time_spline_;
    const double old_traj_dt = traj_dt_;
    const int old_traj_idx = traj_idx_;
    const ros::Time old_traj_start = traj_start_time_;

    current_goal_ = pending_traj_.goal;
    current_goal_yaw_ = pending_traj_.goal_yaw;
    current_goal_task_id_ = pending_traj_.task_id;
    frontier_targets_ = pending_traj_.frontier_targets;
    frontier_target_yaws_ = pending_traj_.frontier_target_yaws;
    frontier_target_task_ids_ = pending_traj_.frontier_target_task_ids;
    frontier_target_idx_ = pending_traj_.frontier_target_idx;
    atsp_tour_active_ = pending_traj_.atsp_tour_active;
    astar_path_ = pending_traj_.astar_path;
    traj_points_ = pending_traj_.points;
    traj_vels_ = pending_traj_.vels;
    traj_accs_ = pending_traj_.accs;
    traj_yaws_ = pending_traj_.yaws;
    active_time_spline_ = pending_traj_.time_spline;
    if (active_time_spline_.valid && traj_points_.size() > 1) {
        traj_dt_ = active_time_spline_.duration / (traj_points_.size() - 1);
    }
    bool bridge_used = false;
    if (!direct_handoff) {
        // Skip the already elapsed successor prefix and bridge from the
        // measured state to a future suffix.  This is the recovery path for a
        // late worker result or a tracking mismatch, not a direct command jump.
        const int first_future_idx = timed_handoff ? std::min(
            (int)traj_points_.size() - 1, std::max(0, (int)std::ceil(
                successor_elapsed / std::max(1e-3, traj_dt_)))) : 0;
        const bool suffix_available = first_future_idx + 2 < (int)traj_points_.size();
        if (suffix_available && first_future_idx > 0) {
            traj_points_.erase(traj_points_.begin(), traj_points_.begin() + first_future_idx);
            traj_vels_.erase(traj_vels_.begin(), traj_vels_.begin() + first_future_idx);
            traj_accs_.erase(traj_accs_.begin(), traj_accs_.begin() + first_future_idx);
            traj_yaws_.erase(traj_yaws_.begin(), traj_yaws_.begin() + first_future_idx);
        }
        active_time_spline_ = TimeBspline();
        const double measured_yaw_rate = std::max(-max_yaw_rate_,
            std::min(max_yaw_rate_, static_cast<double>(realtime_yaw_rate_ref_.load())));
        if (suffix_available && buildContinuousBridge(
                uav_pos_, uav_vel_, Eigen::Vector3d::Zero(),
                uav_yaw_, measured_yaw_rate, 0.0)) {
            planning_start_state_valid_ = true;
            planning_start_acc_.setZero();
            planning_start_yaw_rate_ = measured_yaw_rate;
            planning_start_yaw_acc_ = 0.0;
            rolling_prepare_in_progress_ = true;
            bridge_used = buildTimeParameterizedSpline(
                Eigen::Vector3d::Zero(), measured_yaw_rate, 0.0);
            rolling_prepare_in_progress_ = false;
            planning_start_state_valid_ = false;
        }
        if (!bridge_used) {
            current_goal_ = old_goal;
            current_goal_yaw_ = old_goal_yaw;
            current_goal_task_id_ = old_goal_task_id;
            astar_path_ = old_astar_path;
            traj_points_ = old_points;
            traj_vels_ = old_vels;
            traj_accs_ = old_accs;
            traj_yaws_ = old_yaws;
            active_time_spline_ = old_spline;
            traj_dt_ = old_traj_dt;
            traj_idx_ = old_traj_idx;
            traj_start_time_ = old_traj_start;
            pending_traj_ = PendingTrajectory();
            ROS_WARN("[CoverageSearch] Rolling recovery failed: no safe C2 bridge to "
                     "the future successor suffix; retain old trajectory.");
            return false;
        }
    }
    traj_idx_ = bridge_used ? 0 : (timed_handoff && traj_points_.size() > 1
        ? std::min((int)traj_points_.size() - 1, std::max(0, (int)std::lround(
            successor_elapsed / std::max(1e-3, active_time_spline_.duration) *
            (traj_points_.size() - 1))))
        : 0);
    astar_path_idx_ = 0;
    has_goal_ = true;
    has_traj_ = true;
    traj_start_time_ = bridge_used ? now : (timed_handoff ? handoff_stamp : now);
    traj_point_reach_time_ = now;
    armRealtimeTrajectory();
    pending_traj_ = PendingTrajectory();
    rolling_last_attempt_time_ = traj_start_time_;
    ++rolling_generation_;
    if (!captureActiveGoalFrontier()) {
        ROS_WARN("[CoverageSearch] Rolling handoff target has no fresh frontier signature.");
    }
    markLocalReservationActive(current_goal_task_id_);

    ROS_INFO("[CoverageSearch] Rolling handoff activated: next_goal=(%.2f,%.2f), "
             "phase=%.2fs, bridge=%s, missed=%s, pos_err=%.2fm, vel_err=%.2fm/s, "
             "yaw_err=%.2frad.",
             current_goal_(0), current_goal_(1), successor_elapsed,
             bridge_used ? "C2" : "direct", missed_handoff ? "yes" : "no",
             tracking_error, velocity_error, yaw_error);
    return true;
}

// ============================================================
// ★ 核心方法4：执行轨迹（按位置推进 + 偏航速率限制）
// traj_idx_ 只有在无人机接近当前轨迹点时才推进，避免指令点跑得过快
// 偏航指令以 max_yaw_rate_ 限速变化，避免来回摆头
// ============================================================
double CoverageSearchManager::terminalBrakingDistance() const {
    const double speed = uav_vel_.head<2>().norm();
    return std::max(goal_reach_dist_,
                    speed * speed / (2.0 * std::max(0.3, max_acc_)) + 0.30);
}

void CoverageSearchManager::armRealtimeTrajectory() {
    if (!has_traj_ || traj_points_.empty()) {
        disarmRealtimeTrajectory();
        return;
    }

    std::shared_ptr<RealtimeTrajectory> snapshot(new RealtimeTrajectory());
    snapshot->generation = realtime_trajectory_generation_.fetch_add(1) + 1;
    snapshot->time_spline = active_time_spline_;
    snapshot->points = traj_points_;
    snapshot->velocities = traj_vels_;
    snapshot->accelerations = traj_accs_;
    snapshot->yaws = traj_yaws_;
    snapshot->start_time = traj_start_time_;
    snapshot->goal = current_goal_;
    snapshot->goal_yaw = current_goal_yaw_;
    snapshot->sample_dt = std::max(0.01, traj_dt_);
    realtime_command_id_.store(std::max(realtime_command_id_.load(), uav_command_.Command_ID));
    realtime_completed_generation_.store(0);
    {
        std::lock_guard<std::mutex> lock(realtime_trajectory_mutex_);
        realtime_trajectory_ = snapshot;
    }
}

void CoverageSearchManager::disarmRealtimeTrajectory() {
    {
        std::lock_guard<std::mutex> lock(realtime_trajectory_mutex_);
        realtime_trajectory_.reset();
    }
    realtime_completed_generation_.store(0);
    realtime_yaw_rate_ref_.store(0.0f);
}

bool CoverageSearchManager::consumeRealtimeTrajectoryCompletion() {
    const uint64_t completed = realtime_completed_generation_.exchange(0);
    if (completed == 0) return false;
    std::lock_guard<std::mutex> lock(realtime_trajectory_mutex_);
    return realtime_trajectory_ && realtime_trajectory_->generation == completed;
}

void CoverageSearchManager::executeRealtimeTrajectory() {
    std::shared_ptr<const RealtimeTrajectory> snapshot;
    {
        std::lock_guard<std::mutex> lock(realtime_trajectory_mutex_);
        snapshot = realtime_trajectory_;
    }
    if (!snapshot || snapshot->points.empty()) return;

    Eigen::Vector3d measured_pos, measured_vel;
    double measured_yaw = 0.0;
    {
        std::lock_guard<std::mutex> lock(vehicle_state_mutex_);
        measured_pos = uav_pos_;
        measured_vel = uav_vel_;
        measured_yaw = uav_yaw_;
    }

    const double fallback_duration = snapshot->sample_dt *
        std::max(0, static_cast<int>(snapshot->points.size()) - 1);
    const double duration = snapshot->time_spline.valid
        ? snapshot->time_spline.duration : fallback_duration;
    if (duration <= 1e-3) return;
    const double elapsed = std::max(0.0, (ros::Time::now() - snapshot->start_time).toSec());
    const double command_time = std::min(elapsed, duration);

    Eigen::Vector3d target, velocity, acceleration;
    double yaw = snapshot->goal_yaw;
    double yaw_rate = 0.0, yaw_acceleration = 0.0;
    bool evaluated = false;
    if (snapshot->time_spline.valid) {
        evaluated = evaluateTimeBspline(snapshot->time_spline, command_time,
                                        target, velocity, acceleration,
                                        yaw, yaw_rate, yaw_acceleration);
    } else {
        const int idx = std::min(static_cast<int>(snapshot->points.size()) - 1,
            std::max(0, static_cast<int>(std::floor(
                command_time / std::max(1e-3, snapshot->sample_dt)))));
        target = snapshot->points[idx];
        velocity = idx < static_cast<int>(snapshot->velocities.size())
            ? snapshot->velocities[idx] : Eigen::Vector3d::Zero();
        acceleration = idx < static_cast<int>(snapshot->accelerations.size())
            ? snapshot->accelerations[idx] : Eigen::Vector3d::Zero();
        yaw = idx < static_cast<int>(snapshot->yaws.size()) ? snapshot->yaws[idx]
                                                             : snapshot->goal_yaw;
        evaluated = true;
    }
    if (!evaluated || !target.allFinite() || !velocity.allFinite() ||
        !acceleration.allFinite() || !std::isfinite(yaw) || !std::isfinite(yaw_rate)) {
        ROS_ERROR_THROTTLE(1.0, "[CoverageSearch] Realtime trajectory evaluation failed; keep last safe command.");
        return;
    }

    const bool terminal_hold = elapsed >= duration;
    if (terminal_hold) {
        // Do not replace the analytic endpoint with XYZ_POS.  Keep publishing
        // the exact P/V/A=0 terminal sample until measured motion has settled.
        target = snapshot->goal;
        velocity.setZero();
        acceleration.setZero();
        yaw = snapshot->goal_yaw;
        yaw_rate = 0.0;
    }
    target(2) = fly_height_;
    velocity(2) = 0.0;
    acceleration(2) = 0.0;

    prometheus_msgs::UAVCommand command;
    command.header.stamp = ros::Time::now();
    command.Agent_CMD = prometheus_msgs::UAVCommand::Move;
    command.Control_Level = prometheus_msgs::UAVCommand::DEFAULT_CONTROL;
    command.Move_mode = prometheus_msgs::UAVCommand::TRAJECTORY;
    command.Yaw_Rate_Mode = true;
    for (int axis = 0; axis < 3; ++axis) {
        command.position_ref[axis] = target(axis);
        command.velocity_ref[axis] = velocity(axis);
        command.acceleration_ref[axis] = acceleration(axis);
    }
    command.yaw_ref = yaw;
    command.yaw_rate_ref = yaw_rate;
    command.Command_ID = realtime_command_id_.fetch_add(1) + 1;
    realtime_yaw_rate_ref_.store(yaw_rate);
    uav_cmd_pub_.publish(command);

    if (terminal_hold) {
        const double goal_dist = (measured_pos.head<2>() - snapshot->goal.head<2>()).norm();
        const double yaw_error = std::fabs(std::atan2(
            std::sin(snapshot->goal_yaw - measured_yaw),
            std::cos(snapshot->goal_yaw - measured_yaw)));
        if (goal_dist <= goal_reach_dist_ && measured_vel.head<2>().norm() <= 0.15 &&
            yaw_error <= 0.15) {
            uint64_t expected = 0;
            if (realtime_completed_generation_.compare_exchange_strong(
                    expected, snapshot->generation)) {
                ROS_INFO("[CoverageSearch] Terminal P/V/A=0 hold settled: dist=%.2fm, speed=%.2fm/s, yaw_error=%.2frad.",
                         goal_dist, measured_vel.head<2>().norm(), yaw_error);
            }
        }
    }
}

void CoverageSearchManager::maintainActiveTrajectory() {
    if (!has_traj_ || traj_points_.empty()) return;
    if (!currentGoalLeaseValid()) {
        abortCurrentGoalForSafety("task lease expired or reassigned");
        return;
    }

    if (active_time_spline_.valid) {
        const double elapsed = std::max(0.0, (ros::Time::now() - traj_start_time_).toSec());
        const bool successor_due = active_time_spline_.duration - elapsed <= 0.50;
        const bool final_atsp_target = atsp_tour_active_ && !frontier_targets_.empty() &&
            frontier_target_idx_ == static_cast<int>(frontier_targets_.size()) - 1;
        if (final_atsp_target) {
            tryPrepareRollingHandoff(true, true);
        } else if (currentGoalFrontierCovered() || successor_due) {
            tryPrepareRollingHandoff(true);
        }
        if (pending_traj_.ready && pending_traj_.handoff_time >= 0.0 &&
            elapsed >= pending_traj_.handoff_time) {
            activatePendingTrajectory();
            return;
        }
    }

    std::string cut_reason;
    if (currentGoalUnsafe(cut_reason)) {
        abortCurrentGoalForSafety(cut_reason);
        return;
    }

    const double speed = uav_vel_.head<2>().norm();
    if (speed <= 0.15) return;
    Eigen::Vector3d stop_probe = uav_pos_;
    stop_probe.head<2>() += terminalBrakingDistance() * uav_vel_.head<2>().normalized();
    stop_probe(2) = fly_height_;
    if (!pathToTargetBlocked(stop_probe, uav_pos_, traj_cut_clearance_)) return;

    ROS_WARN_THROTTLE(0.5, "[CoverageSearch] Dynamic braking guard: speed=%.2fm/s, stop_distance=%.2fm. Stop and replan before obstacle.",
                      speed, terminalBrakingDistance());
    uav_command_.header.stamp = ros::Time::now();
    uav_command_.Agent_CMD = prometheus_msgs::UAVCommand::Move;
    uav_command_.Move_mode = prometheus_msgs::UAVCommand::XYZ_POS;
    uav_command_.position_ref[0] = uav_pos_(0);
    uav_command_.position_ref[1] = uav_pos_(1);
    uav_command_.position_ref[2] = fly_height_;
    uav_command_.yaw_ref = uav_yaw_;
    uav_command_.Command_ID++;
    uav_cmd_pub_.publish(uav_command_);
    disarmRealtimeTrajectory();
    has_traj_ = false;
    has_goal_ = true;
    ++replan_count_;
}

void CoverageSearchManager::executeTrajectory() {
    // Kept as the compatibility entry point for any out-of-tree caller.  The
    // old map-coupled executor below is intentionally unreachable: all live
    // commands use the dedicated realtime snapshot path above.
    executeRealtimeTrajectory();
    return;

    if (!has_traj_ || traj_points_.empty()) {
        has_traj_ = false;
        return;
    }

    const ros::Time execution_now = ros::Time::now();
    if (!currentGoalLeaseValid()) {
        abortCurrentGoalForSafety("task lease expired or reassigned");
        return;
    }
    const double execution_elapsed = (execution_now - traj_start_time_).toSec();
    const double remaining_time = active_time_spline_.valid
        ? active_time_spline_.duration - execution_elapsed
        : std::numeric_limits<double>::infinity();
    // Start a successor search in the final 0.50 s; its handoff state is
    // sampled exactly 0.25 s ahead by tryPrepareRollingHandoff().
    const bool successor_due = remaining_time <= 0.50;
    if (currentGoalFrontierCovered() || successor_due) {
        // A new frontier inherits the future rolling P/V/A/yaw state.
        tryPrepareRollingHandoff(true);
    }
    const bool handoff_due = pending_traj_.ready &&
        (active_time_spline_.valid && pending_traj_.handoff_time >= 0.0
             ? execution_elapsed >= pending_traj_.handoff_time
             : traj_idx_ >= pending_traj_.handoff_idx);
    if (handoff_due) {
        activatePendingTrajectory();
    }

    std::string cut_reason;
    if (currentGoalUnsafe(cut_reason)) {
        abortCurrentGoalForSafety(cut_reason);
        return;
    }

    // ★ 初始化平滑偏航为当前真实偏航，避免第一条指令产生大跳变
    if (!cmd_yaw_inited_) {
        cmd_yaw_smoothed_ = uav_yaw_;
        cmd_yaw_inited_ = true;
    }

    auto stepYawToward = [&](double desired_yaw) {
        double yaw_err = atan2(sin(desired_yaw - cmd_yaw_smoothed_),
                               cos(desired_yaw - cmd_yaw_smoothed_));
        double max_delta = max_yaw_rate_ * 0.1;
        if (fabs(yaw_err) <= max_delta) cmd_yaw_smoothed_ = desired_yaw;
        else cmd_yaw_smoothed_ += copysign(max_delta, yaw_err);
        cmd_yaw_smoothed_ = atan2(sin(cmd_yaw_smoothed_), cos(cmd_yaw_smoothed_));
    };

    // 按实际速度对应的制动距离提前切到零速度/零加速度位置控制。
    // 只有位置、偏航和实际速度同时收敛，才允许把视点标记为完成。
    if (has_goal_ && !pending_traj_.ready && !active_time_spline_.valid) {
        double goal_dist = (uav_pos_.head<2>() - current_goal_.head<2>()).norm();
        double actual_speed = uav_vel_.head<2>().norm();
        double terminal_control_dist = terminalBrakingDistance();
        if (goal_dist <= terminal_control_dist &&
            !pathToTargetBlocked(current_goal_, uav_pos_, traj_cut_clearance_)) {
            stepYawToward(current_goal_yaw_);
            double actual_yaw_err = fabs(atan2(sin(current_goal_yaw_ - uav_yaw_),
                                                cos(current_goal_yaw_ - uav_yaw_)));

            uav_command_.header.stamp = ros::Time::now();
            uav_command_.Agent_CMD = prometheus_msgs::UAVCommand::Move;
            uav_command_.Move_mode = prometheus_msgs::UAVCommand::XYZ_POS;
            uav_command_.position_ref[0] = current_goal_(0);
            uav_command_.position_ref[1] = current_goal_(1);
            uav_command_.position_ref[2] = fly_height_;
            uav_command_.yaw_ref = cmd_yaw_smoothed_;
            uav_command_.Command_ID++;
            uav_cmd_pub_.publish(uav_command_);

            ROS_INFO_THROTTLE(1.0,
                "[CoverageSearch] Terminal braking: dist=%.2fm, speed=%.2fm/s, "
                "brake_entry=%.2fm, yaw_error=%.2frad.",
                goal_dist, actual_speed, terminal_control_dist, actual_yaw_err);

            if (goal_dist <= goal_reach_dist_ && actual_yaw_err <= 0.15 &&
                actual_speed <= 0.15) {
                cout << GREEN << "[CoverageSearch] Viewpoint stopped and yaw aligned. dist="
                     << goal_dist << "m, speed=" << actual_speed
                     << "m/s, yaw_error=" << actual_yaw_err << TAIL << endl;
                has_traj_ = false;
                has_goal_ = false;
            }
            return;
        }
    }

    // 位置曲线即使安全，真实机体也可能因速度和跟踪误差沿当前速度方向冲出曲线。
    // 用实际制动距离检查惯性延长线；发现墙体时先停车，再从实测位置重规划。
    double actual_speed = uav_vel_.head<2>().norm();
    if (actual_speed > 0.15) {
        Eigen::Vector3d stop_probe = uav_pos_;
        stop_probe.head<2>() += terminalBrakingDistance() *
                                uav_vel_.head<2>().normalized();
        stop_probe(2) = fly_height_;
        if (pathToTargetBlocked(stop_probe, uav_pos_, traj_cut_clearance_)) {
            ROS_WARN_THROTTLE(0.5,
                "[CoverageSearch] Dynamic braking guard: speed=%.2fm/s, "
                "stop_distance=%.2fm. Stop and replan before obstacle.",
                actual_speed, terminalBrakingDistance());
            uav_command_.header.stamp = ros::Time::now();
            uav_command_.Agent_CMD = prometheus_msgs::UAVCommand::Move;
            uav_command_.Move_mode = prometheus_msgs::UAVCommand::XYZ_POS;
            uav_command_.position_ref[0] = uav_pos_(0);
            uav_command_.position_ref[1] = uav_pos_(1);
            uav_command_.position_ref[2] = fly_height_;
            uav_command_.yaw_ref = uav_yaw_;
            uav_command_.Command_ID++;
            uav_cmd_pub_.publish(uav_command_);
            has_traj_ = false;
            has_goal_ = true;
            replan_count_++;
            return;
        }
    }

    // 时间B样条必须按时钟用De Boor解析执行。离散点只保留给安全预检和RViz，
    // 不再用“靠近一个点才推进”的空间索引驱动，否则轨迹会走完后停顿再突加速。
    if (active_time_spline_.valid) {
        const double elapsed = std::max(
            0.0, (ros::Time::now() - traj_start_time_).toSec());
        if (elapsed >= active_time_spline_.duration && !pending_traj_.ready) {
            if (pathToTargetBlocked(current_goal_, uav_pos_, traj_cut_clearance_)) {
                ROS_WARN("[CoverageSearch] Terminal hold path is blocked; replan same goal.");
                uav_command_.header.stamp = ros::Time::now();
                uav_command_.Agent_CMD = prometheus_msgs::UAVCommand::Move;
                uav_command_.Move_mode = prometheus_msgs::UAVCommand::XYZ_POS;
                uav_command_.position_ref[0] = uav_pos_(0);
                uav_command_.position_ref[1] = uav_pos_(1);
                uav_command_.position_ref[2] = fly_height_;
                uav_command_.yaw_ref = uav_yaw_;
                uav_command_.Command_ID++;
                uav_cmd_pub_.publish(uav_command_);
                has_traj_ = false;
                has_goal_ = true;
                replan_count_++;
                return;
            }
            uav_command_.header.stamp = ros::Time::now();
            uav_command_.Agent_CMD = prometheus_msgs::UAVCommand::Move;
            uav_command_.Move_mode = prometheus_msgs::UAVCommand::XYZ_POS;
            uav_command_.position_ref[0] = current_goal_(0);
            uav_command_.position_ref[1] = current_goal_(1);
            uav_command_.position_ref[2] = fly_height_;
            uav_command_.yaw_ref = current_goal_yaw_;
            uav_command_.Command_ID++;
            uav_cmd_pub_.publish(uav_command_);
            const double goal_dist = (uav_pos_.head<2>() - current_goal_.head<2>()).norm();
            const double yaw_error = std::fabs(std::atan2(
                std::sin(current_goal_yaw_ - uav_yaw_),
                std::cos(current_goal_yaw_ - uav_yaw_)));
            if (goal_dist <= goal_reach_dist_ && yaw_error <= 0.15 &&
                uav_vel_.head<2>().norm() <= 0.15) {
                has_traj_ = false;
                has_goal_ = false;
            }
            return;
        }
        const double command_time = std::min(active_time_spline_.duration, elapsed);
        Eigen::Vector3d target, command_vel, command_acc;
        double desired_yaw = 0.0, desired_yaw_rate = 0.0, desired_yaw_acc = 0.0;
        if (!evaluateTimeBspline(active_time_spline_, command_time,
                                 target, command_vel, command_acc,
                                 desired_yaw, desired_yaw_rate, desired_yaw_acc)) {
            ROS_ERROR("[CoverageSearch] De Boor execution failed at t=%.3fs; "
                      "stop and replan the same goal.", command_time);
            has_traj_ = false;
            has_goal_ = true;
            return;
        }
        target(2) = fly_height_;
        command_vel(2) = 0.0;
        command_acc(2) = 0.0;
        const double progress = command_time /
            std::max(1e-3, active_time_spline_.duration);
        traj_idx_ = std::min((int)traj_points_.size() - 1,
            std::max(0, (int)std::lround(
                progress * std::max(0, (int)traj_points_.size() - 1))));

        if (pathToTargetBlocked(target, uav_pos_, traj_cut_clearance_)) {
            ROS_WARN("[CoverageSearch] Timed De Boor command became unsafe at "
                     "t=%.2f/%.2fs; stop and replan the same goal.",
                     command_time, active_time_spline_.duration);
            uav_command_.header.stamp = ros::Time::now();
            uav_command_.Agent_CMD = prometheus_msgs::UAVCommand::Move;
            uav_command_.Move_mode = prometheus_msgs::UAVCommand::XYZ_POS;
            uav_command_.position_ref[0] = uav_pos_(0);
            uav_command_.position_ref[1] = uav_pos_(1);
            uav_command_.position_ref[2] = fly_height_;
            uav_command_.yaw_ref = uav_yaw_;
            uav_command_.Command_ID++;
            uav_cmd_pub_.publish(uav_command_);
            has_traj_ = false;
            has_goal_ = true;
            replan_count_++;
            return;
        }

        uav_command_.header.stamp = ros::Time::now();
        uav_command_.Agent_CMD = prometheus_msgs::UAVCommand::Move;
        uav_command_.Move_mode = prometheus_msgs::UAVCommand::TRAJECTORY;
        for (int axis = 0; axis < 3; ++axis) {
            uav_command_.position_ref[axis] = target(axis);
            uav_command_.velocity_ref[axis] = command_vel(axis);
            uav_command_.acceleration_ref[axis] = command_acc(axis);
        }
        // The time B-spline already bounds yaw rate/acceleration and distributes
        // the turn across the full flight. A second rate limiter here can lag
        // the reference and force the remaining turn into the terminal hold.
        uav_command_.yaw_ref = desired_yaw;
        uav_command_.yaw_rate_ref = desired_yaw_rate;
        uav_command_.Command_ID++;
        uav_cmd_pub_.publish(uav_command_);

        if (elapsed >= active_time_spline_.duration) {
            const double goal_dist =
                (uav_pos_.head<2>() - current_goal_.head<2>()).norm();
            const double yaw_error = std::fabs(std::atan2(
                std::sin(current_goal_yaw_ - uav_yaw_),
                std::cos(current_goal_yaw_ - uav_yaw_)));
            if (goal_dist <= goal_reach_dist_ && yaw_error <= 0.15 &&
                uav_vel_.head<2>().norm() <= 0.15) {
                ROS_INFO("[CoverageSearch] Timed trajectory completed at rest: "
                         "dist=%.2fm, speed=%.2fm/s, yaw_error=%.2frad.",
                         goal_dist, uav_vel_.head<2>().norm(), yaw_error);
                has_traj_ = false;
                has_goal_ = false;
            }
        }
        if (elapsed > 180.0) {
            ROS_WARN("[CoverageSearch] Timed trajectory timeout after %.1fs.", elapsed);
            has_traj_ = false;
            has_goal_ = false;
            replan_count_++;
        }
        return;
    }

    if (traj_idx_ >= (int)traj_points_.size()) {
        has_traj_ = false;
        return;
    }

    // ★ 按位置推进：先把 traj_idx_ 对齐到当前位置附近的轨迹点，避免前视控制绕过
    // 密集中间点后连续 dwell timeout。
    bool advanced = false;
    {
        const int active_end = pending_traj_.ready
            ? std::min((int)traj_points_.size() - 1, pending_traj_.handoff_idx)
            : (int)traj_points_.size() - 1;
        int search_end = std::min(active_end, traj_idx_ + 20);
        int nearest_idx = traj_idx_;
        double nearest_dist = 1e9;
        for (int i = traj_idx_; i <= search_end; ++i) {
            double dist = (uav_pos_.head<2>() - traj_points_[i].head<2>()).norm();
            if (dist < nearest_dist) {
                nearest_dist = dist;
                nearest_idx = i;
            }
        }
        const double snap_dist = std::max(0.25, std::min(traj_advance_dist_, 0.45));
        if (nearest_idx > traj_idx_ && nearest_dist < snap_dist) {
            traj_idx_ = nearest_idx;
            traj_point_reach_time_ = ros::Time::now();
            advanced = true;
        }
    }

    const double dynamic_advance_dist = std::max(0.25, std::min(traj_advance_dist_, 0.45));
    const int active_end = pending_traj_.ready
        ? std::min((int)traj_points_.size() - 1, pending_traj_.handoff_idx)
        : (int)traj_points_.size() - 1;
    while (traj_idx_ <= active_end) {
        double dist = (uav_pos_.head<2>() - traj_points_[traj_idx_].head<2>()).norm();
        if (dist < dynamic_advance_dist) {
            if (traj_idx_ == active_end) {
                break;
            }
            traj_idx_++;
            traj_point_reach_time_ = ros::Time::now();
            advanced = true;
        } else {
            break;
        }
    }
    // ★ 前进即清零同目标重规划计数：只要在推进就不算卡死，不会被放弃
    if (advanced) same_goal_replan_count_ = 0;

    // 正常完成只允许走上面的“位置 + 实际偏航”统一入口。
    // 若索引意外越过末尾，保留目标交给主循环重新规划。
    if (traj_idx_ >= (int)traj_points_.size()) {
        has_traj_ = false;
        return;
    }

    Eigen::Vector3i current_idx;
    coverage_map_.posToIndex(uav_pos_, current_idx);
    const double requested_clearance = std::max(
        std::max(0.35, coverage_map_.esdf_safe_distance_), traj_cut_clearance_);
    const double hard_clearance =
        requiredTrajectoryClearance(uav_pos_, requested_clearance);
    const double current_clearance =
        coverage_map_.getDistance2D(current_idx(0), current_idx(1));
    const bool current_requires_escape =
        !coverage_map_.isFree2D(current_idx(0), current_idx(1)) ||
        current_clearance < hard_clearance;
    auto unsafePathPoint = [&](const Eigen::Vector3d &p, double min_clearance,
                               bool &escaping, double &prev_clearance) {
        Eigen::Vector3i idx;
        coverage_map_.posToIndex(p, idx);
        if (!coverage_map_.isInMap2D(idx(0), idx(1))) return true;
        bool free = coverage_map_.isFree2D(idx(0), idx(1));
        double clearance = coverage_map_.getDistance2D(idx(0), idx(1));
        if (escaping) {
            bool in_start_cell = idx(0) == current_idx(0) && idx(1) == current_idx(1);
            if (!in_start_cell) {
                if (!free || clearance + 1e-3 < prev_clearance) return true;
                prev_clearance = clearance;
                if (clearance >= requiredTrajectoryClearance(p, min_clearance))
                    escaping = false;
            }
            return false;
        }
        return !free || clearance < requiredTrajectoryClearance(p, min_clearance);
    };

    if (traj_cut_clearance_ > 0.0) {
        Eigen::Vector3i goal_idx;
        coverage_map_.posToIndex(current_goal_, goal_idx);
        const double goal_cutoff =
            requiredTrajectoryClearance(current_goal_, requested_clearance);
        if (has_goal_ && coverage_map_.getDistance2D(goal_idx(0), goal_idx(1)) + 1e-3 <
                             goal_cutoff) {
            abortCurrentGoalForSafety("goal ESDF clearance below trajectory cut threshold");
            return;
        }

        int check_end = std::min(active_end, traj_idx_ + 15);
        double acc_dist = 0.0;
        Eigen::Vector3d last_check = uav_pos_;
        last_check(2) = fly_height_;
        bool escaping_check = current_requires_escape;
        double prev_clearance = current_clearance;
        for (int i = traj_idx_; i <= check_end; ++i) {
            Eigen::Vector3d p = traj_points_[i];
            p(2) = fly_height_;
            Eigen::Vector3d seg = p - last_check;
            seg(2) = 0.0;
            acc_dist += seg.norm();
            Eigen::Vector3i p_idx;
            coverage_map_.posToIndex(p, p_idx);
            double clearance = coverage_map_.getDistance2D(p_idx(0), p_idx(1));
            if (unsafePathPoint(p, traj_cut_clearance_, escaping_check, prev_clearance) ||
                pathToTargetBlocked(p, last_check, traj_cut_clearance_)) {
                cout << YELLOW << "[CoverageSearch] Cut unsafe trajectory ahead: traj point "
                     << i << " ESDF clearance " << clearance << " < "
                     << requiredTrajectoryClearance(p, requested_clearance)
                     << ". Replan to SAME frontier viewpoint." << TAIL << endl;
                uav_command_.header.stamp = ros::Time::now();
                uav_command_.Agent_CMD = prometheus_msgs::UAVCommand::Move;
                uav_command_.Move_mode = prometheus_msgs::UAVCommand::XYZ_POS;
                uav_command_.position_ref[0] = uav_pos_(0);
                uav_command_.position_ref[1] = uav_pos_(1);
                uav_command_.position_ref[2] = fly_height_;
                uav_command_.yaw_ref = cmd_yaw_smoothed_;
                uav_command_.Command_ID++;
                uav_cmd_pub_.publish(uav_command_);
                has_traj_ = false;
                has_goal_ = true;
                replan_count_++;
                return;
            }
            last_check = p;
            if (acc_dist >= 2.0) break;
        }
    }

    // 轨迹点迟迟到不了说明执行轨迹已与真实位置脱节；禁止继续跳索引，
    // 立即停车并从真实位置重规划同一目标。
    double dwell = (ros::Time::now() - traj_point_reach_time_).toSec();
    if (!advanced && dwell > traj_point_dwell_timeout_) {
        double point_dist =
            (uav_pos_.head<2>() - traj_points_[traj_idx_].head<2>()).norm();
        ROS_WARN("[CoverageSearch] Trajectory tracking stalled: idx=%d/%zu, "
                 "dwell=%.2fs, point_dist=%.2fm, speed=%.2fm/s. Replan from measured pose.",
                 traj_idx_, traj_points_.size(), dwell, point_dist,
                 uav_vel_.head<2>().norm());
        uav_command_.header.stamp = ros::Time::now();
        uav_command_.Agent_CMD = prometheus_msgs::UAVCommand::Move;
        uav_command_.Move_mode = prometheus_msgs::UAVCommand::XYZ_POS;
        uav_command_.position_ref[0] = uav_pos_(0);
        uav_command_.position_ref[1] = uav_pos_(1);
        uav_command_.position_ref[2] = fly_height_;
        uav_command_.yaw_ref = uav_yaw_;
        uav_command_.Command_ID++;
        uav_cmd_pub_.publish(uav_command_);
        has_traj_ = false;
        has_goal_ = true;
        replan_count_++;
        return;
    }

    int cmd_idx = traj_idx_;
    const double command_min_clearance = requested_clearance;
    double lookahead_dist = std::max(0.45, std::min(0.90, max_vel_ * 0.25));
    if (current_clearance < 2.0 * command_min_clearance) {
        lookahead_dist = std::min(lookahead_dist, 0.45);
    }
    double acc_dist = 0.0;
    Eigen::Vector3d last_look = uav_pos_;
    last_look(2) = fly_height_;
    int last_safe_idx = -1;
    for (int i = traj_idx_; i <= active_end; ++i) {
        Eigen::Vector3d p = traj_points_[i];
        p(2) = fly_height_;
        Eigen::Vector3d seg = p - last_look;
        seg(2) = 0.0;
        acc_dist += seg.norm();
        if (pathToTargetBlocked(p, uav_pos_, traj_cut_clearance_)) {
            break;
        }
        last_safe_idx = i;
        cmd_idx = i;
        last_look = p;
        if (acc_dist >= lookahead_dist) break;
    }
    if (last_safe_idx < 0) {
        ROS_WARN("[CoverageSearch] No safe controller chord from measured pose to trajectory "
                 "idx=%d/%zu; stop and replan same goal.",
                 traj_idx_, traj_points_.size());
        uav_command_.header.stamp = ros::Time::now();
        uav_command_.Agent_CMD = prometheus_msgs::UAVCommand::Move;
        uav_command_.Move_mode = prometheus_msgs::UAVCommand::XYZ_POS;
        uav_command_.position_ref[0] = uav_pos_(0);
        uav_command_.position_ref[1] = uav_pos_(1);
        uav_command_.position_ref[2] = fly_height_;
        uav_command_.yaw_ref = uav_yaw_;
        uav_command_.Command_ID++;
        uav_cmd_pub_.publish(uav_command_);
        has_traj_ = false;
        has_goal_ = true;
        replan_count_++;
        return;
    }
    cmd_idx = last_safe_idx;

    Eigen::Vector3d target = traj_points_[cmd_idx];

    // ★ 方向承诺跟随实际运动方向(低通滤波)，不再锁死
    //   这样无人机往哪走，承诺就往哪；被迫绕行/回头后承诺自然更新，不再反向振荡
    {
        Eigen::Vector3d motion_dir = target - uav_pos_;
        motion_dir(2) = 0;
        if (motion_dir.norm() > 0.15) {
            double motion_heading = atan2(motion_dir(1), motion_dir(0));
            if (!has_committed_heading_) {
                committed_heading_ = motion_heading;
                has_committed_heading_ = true;
            } else {
                double diff = atan2(sin(motion_heading - committed_heading_),
                                     cos(motion_heading - committed_heading_));
                committed_heading_ = atan2(sin(committed_heading_ + 0.2 * diff),
                                            cos(committed_heading_ + 0.2 * diff));
            }
        }
    }

    // ★ 偏航速率限制：cmd_yaw_smoothed_ 以 max_yaw_rate_*dt 速率向目标偏航靠近
    // 避免每个周期 yaw_ref 大幅跳变导致飞控持续追偏航、机身来回摆头
    double desired_yaw = traj_yaws_[cmd_idx];
    stepYawToward(desired_yaw);

    // ★ 避障：检查前方轨迹点是否安全
    // 原来用 pathToTargetBlocked 检查"无人机→轨迹点"的直线，这是错的！
    // 因为A*是绕行的，从无人机到轨迹点0的直线可能穿过障碍物，导致刚规划就被废弃
    // 正确做法：检查轨迹点本身是否在occupied区域，以及相邻轨迹点之间的连线是否被挡
    bool path_blocked = false;
    int lookahead = min(10, active_end - traj_idx_ + 1);
    bool escaping_command = current_requires_escape;
    double command_prev_clearance = current_clearance;
    for (int k = 0; k < lookahead; k++) {
        int check_idx = traj_idx_ + k;
        Eigen::Vector3d check_pt = traj_points_[check_idx];
        bool escape_segment = escaping_command;
        if (unsafePathPoint(check_pt, command_min_clearance,
                            escaping_command, command_prev_clearance)) {
            path_blocked = true;
            break;
        }
        // 检查相邻轨迹点之间的连线是否被挡（这才是合理的避障检查）
        if (k > 0 && !escape_segment) {
            int prev_idx = traj_idx_ + k - 1;
            if (pathToTargetBlocked(traj_points_[check_idx], traj_points_[prev_idx],
                                    traj_cut_clearance_)) {
                path_blocked = true;
                break;
            }
        }
    }

    if (path_blocked) {
        if (currentGoalUnsafe(cut_reason)) {
            abortCurrentGoalForSafety(cut_reason);
            return;
        }
        cout << YELLOW << "[CoverageSearch] Path to traj point " << traj_idx_
             << " blocked. Abandon trajectory, replan to SAME goal." << TAIL << endl;
        uav_command_.header.stamp = ros::Time::now();
        uav_command_.Agent_CMD = prometheus_msgs::UAVCommand::Move;
        uav_command_.Move_mode = prometheus_msgs::UAVCommand::XYZ_POS;
        uav_command_.position_ref[0] = uav_pos_(0);
        uav_command_.position_ref[1] = uav_pos_(1);
        uav_command_.position_ref[2] = fly_height_;
        uav_command_.yaw_ref = cmd_yaw_smoothed_;
        uav_command_.Command_ID++;
        uav_cmd_pub_.publish(uav_command_);
        has_traj_ = false;       // 触发重规划轨迹
        has_goal_ = true;        // ★ 保留目标，让 mainloop 对同一目标重新规划
        return;
    }

    // 原生轨迹模式：位置闭环，同时提供曲线切向速度和加速度前馈。
    uav_command_.header.stamp = ros::Time::now();
    uav_command_.Agent_CMD = prometheus_msgs::UAVCommand::Move;
    uav_command_.Move_mode = prometheus_msgs::UAVCommand::TRAJECTORY;
    uav_command_.position_ref[0] = target(0);
    uav_command_.position_ref[1] = target(1);
    uav_command_.position_ref[2] = fly_height_;
    for (int axis = 0; axis < 3; ++axis) {
        uav_command_.velocity_ref[axis] = traj_vels_[cmd_idx](axis);
        uav_command_.acceleration_ref[axis] = traj_accs_[cmd_idx](axis);
    }
    uav_command_.yaw_ref = cmd_yaw_smoothed_;
    uav_command_.Command_ID++;
    uav_cmd_pub_.publish(uav_command_);

    // ★ 轨迹整体超时保护
    double traj_elapsed = (ros::Time::now() - traj_start_time_).toSec();
    if (traj_elapsed > 180.0) {
        double dist_to_target = (uav_pos_.head<2>() - target.head<2>()).norm();
        has_traj_ = false;
        has_goal_ = false;
        cout << YELLOW << "[CoverageSearch] Trajectory timeout (" << traj_elapsed
             << "s). dist=" << dist_to_target << TAIL << endl;
        replan_count_++;
    }
}

bool CoverageSearchManager::reachedGoal() {
    // ★ 仅检查无人机与目标视点的2D距离，移除宽松的"感知范围到达"判定
    double dist2d = (uav_pos_.head<2>() - current_goal_.head<2>()).norm();
    return dist2d < goal_reach_dist_;
}
