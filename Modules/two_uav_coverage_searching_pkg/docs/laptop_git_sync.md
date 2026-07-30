# 笔记本 Codex 拉取双机协同覆盖功能包

远程仓库的 `two-uav-coverage` 分支根目录就是 ROS 功能包
`prometheus_two_uav_coverage_search`。只操作该目录，不修改笔记本 Prometheus
工作空间根仓库的 Git 分支。

## 本版关键说明：Nlopt 时间最小化求解器

本版已加入 **Nlopt 时间最小化求解器**。轨迹代码使用
`nlopt::LD_LBFGS` 同时优化二维 B 样条的自由控制点和总飞行时间；目标函数包含
时间、平滑度、参考路径偏差、障碍物间隙以及速度/加速度可行性约束。

该功能不是可选项：`two_uav_coverage_search_trajectory.cpp` 包含 `nlopt.hpp`，
`CMakeLists.txt` 会查找 `nlopt.hpp` 和 `libnlopt.so`。拉取后、编译前必须先确认
笔记本已将 Nlopt 安装到 `/usr/local`：

```bash
test -f /usr/local/include/nlopt.hpp
test -f /usr/local/lib/libnlopt.so
```

任一检查失败时，先请求用户授权安装 Nlopt 的开发文件到 `/usr/local`，再构建；
不要删除 Nlopt 链接、不要用关闭优化来绕过 CMake 错误。

## 首次拉取

```bash
package_dir="$HOME/Prometheus/Modules/two_uav_coverage_searching_pkg"
test ! -e "$package_dir" || { echo "请先备份或移除已有目录：$package_dir"; exit 1; }
git clone --branch two-uav-coverage --single-branch --depth 1 \
  git@github.com:fisher123321/prometheus-coverage-search.git "$package_dir"
cd "$package_dir"
test -f package.xml && git status --short && git log -1 --oneline
```

然后使用笔记本既有的 `catkin_make` 工作空间配置构建该包：

```bash
source /opt/ros/noetic/setup.bash
cd "$HOME/Prometheus"
catkin_make --pkg prometheus_two_uav_coverage_search
```

## 后续更新

```bash
cd "$HOME/Prometheus/Modules/two_uav_coverage_searching_pkg"
git status --short
git pull --ff-only origin two-uav-coverage
git log -1 --oneline
```

有未提交修改时先停止并由用户决定如何处理，不能自动暂存、丢弃或覆盖。不要在
`$HOME/Prometheus` 根目录执行 `git add .`，也不要执行 `git reset --hard`、
`git push --force` 或修改其他功能包。

## 给笔记本 Codex 的指令

```text
只操作 $HOME/Prometheus/Modules/two_uav_coverage_searching_pkg。
目录不存在时，从 git@github.com:fisher123321/prometheus-coverage-search.git 的
two-uav-coverage 分支单分支浅克隆；目录存在时先输出 git status --short，
只在工作区干净时执行 git pull --ff-only origin two-uav-coverage。

本包使用 Nlopt 时间最小化求解器：必须保留 nlopt.hpp 和 libnlopt.so 依赖。先检查
/usr/local/include/nlopt.hpp 与 /usr/local/lib/libnlopt.so；缺少时停止并请求用户授权
安装 Nlopt 开发文件，不能删除优化或改成占位实现。完成后确认 package.xml 存在、
输出 git log -1 --oneline，并用笔记本现有 catkin_make 配置构建
prometheus_two_uav_coverage_search。不要修改 Prometheus 根仓库或其他功能包；
不要执行 force、reset --hard、自动 stash 或自动丢弃改动。
```
