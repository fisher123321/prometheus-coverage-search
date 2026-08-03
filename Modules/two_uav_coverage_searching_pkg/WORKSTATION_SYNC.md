# 工位电脑同步说明

本版本提交说明：`重规划机制效果很差`。

远程同步分支：`coverage-replanning-poor-20260802`。

在工位电脑的 Prometheus 工作区执行：

```bash
cd /home/wjy/Prometheus
git remote get-url coverage-source || \
  git remote add coverage-source git@github.com:fisher123321/prometheus-coverage-search.git
git fetch coverage-source coverage-replanning-poor-20260802:refs/remotes/coverage-source/coverage-replanning-poor-20260802
git switch --track -c coverage-replanning-poor-20260802 coverage-source/coverage-replanning-poor-20260802
```

后续更新使用：

```bash
cd /home/wjy/Prometheus
git switch coverage-replanning-poor-20260802
git pull --ff-only coverage-source coverage-replanning-poor-20260802
```

拉取前若 `git status --short` 显示本地改动，请先自行提交或暂存，避免覆盖工位上的工作。不要强推或把该分支硬合并进工位的 `main`。

本次同步同时包含：

- `Modules/two_uav_coverage_search/`：覆盖搜索、实时轨迹发布和滚动重规划代码；
- `Modules/uav_control/src/uav_controller.cpp`：轨迹 `yaw_rate_ref` 转发修复。

拉取后由 Codex 执行编译；手动编译命令为：

```bash
source /opt/ros/noetic/setup.bash
source /home/wjy/prometheus_mavros/devel/setup.bash
cd /home/wjy/Prometheus
catkin_make --source Modules/two_uav_coverage_search --build build/two_uav_coverage_search --make-args two_uav_coverage_search_node -j1
catkin_make --source Modules/uav_control --build build/uav_control --make-args uav_control_main -j1
```
