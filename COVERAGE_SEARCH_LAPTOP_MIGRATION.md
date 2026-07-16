# 覆盖搜索功能迁移到另一台 Prometheus 电脑

本文给笔记本上的 Codex 使用。目标不是用本仓库覆盖笔记本已有的官方 Prometheus，而是只迁移覆盖搜索相关文件，并对官方控制器应用一处最小补丁。

## 1. 迁移原则

- 保留笔记本现有的官方 Prometheus 仓库及其 Git 历史。
- 不复制整个工作空间，不复制 `.git/`、`build/`、`devel/`、日志或编译产物。
- 新增内容可以直接复制；官方文件只按第 4 节做语义补丁，不应整文件盲目覆盖。
- 导入前先执行 `git status --short`，笔记本已有修改必须先提交或备份。

## 2. 本项目实际新增的内容

需要迁移以下 5 组路径：

```text
Modules/coverage_searching_pkg/
Scripts/simulation/searching_pkg/
Simulator/gazebo_simulator/gazebo_worlds/planning_worlds/samplemaze.world
Simulator/gazebo_simulator/launch_basic/maze_mapping.rviz
Simulator/gazebo_simulator/launch_basic/sitl_coverage_search_1uav_P450.launch
```

说明：

- `Modules/coverage_searching_pkg/` 是完整覆盖搜索 ROS 功能包，ROS 包名为 `prometheus_coverage_search`。
- `coverage_search_P450.sh` 启动 roscore、PX4/Gazebo/MAVROS 和官方无人机控制器。
- `coverage_search_node.sh` 启动覆盖搜索节点。
- `samplemaze.world`、`maze_mapping.rviz` 和专用 SITL launch 只服务于本实验，不替换官方通用文件。

唯一需要修改的官方源码是：

```text
Modules/uav_control/include/uav_controller.h
Modules/uav_control/src/uav_controller.cpp
```

该补丁只让 `TRAJECTORY` 模式向 PX4 同时发送位置、速度、加速度和偏航角；其他控制模式不变。

## 3. 推荐迁移方式：从 GitHub 只取指定路径

远程私有仓库：

```text
git@github.com:fisher123321/prometheus-coverage-search.git
```

笔记本必须先配置能访问该私有仓库的 GitHub SSH 密钥；也可以把远程地址换成带有正确认证方式的 HTTPS 地址。

在笔记本已有的 `~/Prometheus` 中执行：

```bash
cd ~/Prometheus
git status --short
git remote add coverage-source git@github.com:fisher123321/prometheus-coverage-search.git
git fetch coverage-source main
git checkout coverage-source/main -- \
  COVERAGE_SEARCH_LAPTOP_MIGRATION.md \
  Modules/coverage_searching_pkg \
  Scripts/simulation/searching_pkg \
  Simulator/gazebo_simulator/gazebo_worlds/planning_worlds/samplemaze.world \
  Simulator/gazebo_simulator/launch_basic/maze_mapping.rviz \
  Simulator/gazebo_simulator/launch_basic/sitl_coverage_search_1uav_P450.launch
```

如果 `coverage-source` 已存在，不要重复添加，先检查：

```bash
git remote get-url coverage-source
```

不要执行 `git reset --hard coverage-source/main`，也不要把整个远程分支合并到笔记本官方仓库。上面的 `git checkout ... -- 指定路径` 只导入列出的自研文件，而且会把它们放入暂存区；这是预期行为。

随后由笔记本 Codex 按下一节修改控制器，并检查：

```bash
git status --short
git diff --cached --stat
git diff -- Modules/uav_control/include/uav_controller.h \
             Modules/uav_control/src/uav_controller.cpp
```

确认后在笔记本自己的分支提交即可。

## 4. 官方无人机控制器的最小补丁

不要直接覆盖笔记本上的两个官方控制器文件，应按其当前版本寻找同名函数并应用以下三处改动。

### 4.1 添加函数声明

在 `Modules/uav_control/include/uav_controller.h` 的 `send_pos_vel_xyz_setpoint(...)` 附近添加：

```cpp
void send_pos_vel_acc_xyz_setpoint(const Eigen::Vector3d &pos_sp,
                                   const Eigen::Vector3d &vel_sp,
                                   const Eigen::Vector3d &acc_sp,
                                   float yaw_sp);
```

### 4.2 让 TRAJECTORY 模式调用新函数

在 `Modules/uav_control/src/uav_controller.cpp` 的
`send_pos_cmd_to_px4_original_controller()` 内，把 `TRAJECTORY` 分支设置为：

```cpp
else if (uav_command.Move_mode == prometheus_msgs::UAVCommand::TRAJECTORY)
{
    send_pos_vel_acc_xyz_setpoint(pos_des, vel_des, acc_des, yaw_des);
    vel_control = false;
    yaw_control = false;
}
```

不要修改 `set_command_des()` 中读取 `position_ref`、`velocity_ref`、`acceleration_ref` 和 `yaw_ref` 的现有逻辑。

### 4.3 添加发送函数实现

在 `send_pos_vel_xyz_setpoint(...)` 实现之后添加：

```cpp
// Coverage trajectories use position feedback with velocity and acceleration feed-forward.
void UAV_controller::send_pos_vel_acc_xyz_setpoint(const Eigen::Vector3d &pos_sp,
                                                   const Eigen::Vector3d &vel_sp,
                                                   const Eigen::Vector3d &acc_sp,
                                                   float yaw_sp)
{
    mavros_msgs::PositionTarget pos_setpoint;
    pos_setpoint.type_mask = 0b100000000000; // P/V/A + yaw; ignore yaw_rate only
    pos_setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    pos_setpoint.position.x = pos_sp[0];
    pos_setpoint.position.y = pos_sp[1];
    pos_setpoint.position.z = pos_sp[2];
    pos_setpoint.velocity.x = vel_sp[0];
    pos_setpoint.velocity.y = vel_sp[1];
    pos_setpoint.velocity.z = vel_sp[2];
    pos_setpoint.acceleration_or_force.x = acc_sp[0];
    pos_setpoint.acceleration_or_force.y = acc_sp[1];
    pos_setpoint.acceleration_or_force.z = acc_sp[2];
    pos_setpoint.yaw = yaw_sp;
    px4_setpoint_raw_local_pub.publish(pos_setpoint);
}
```

检查补丁没有影响其他模式：

```bash
rg -n -C 4 'send_pos_vel_acc_xyz_setpoint|UAVCommand::TRAJECTORY' \
  Modules/uav_control/include/uav_controller.h \
  Modules/uav_control/src/uav_controller.cpp
```

## 5. 外部环境和固定目录

当前脚本默认以下目录存在：

```text
${HOME}/Prometheus
${HOME}/prometheus_mavros/devel/setup.bash
${HOME}/prometheus_px4/Tools/setup_gazebo.bash
${HOME}/prometheus_px4/build/amovlab_sitl_default/bin/px4
```

注意：`prometheus_mavros` 和 `prometheus_px4` 是外部工作空间，不在本 Git 仓库中，也不应通过本仓库迁移。笔记本应按原 Prometheus 安装方式准备相同版本并完成编译。如果笔记本使用不同路径，可通过环境变量覆盖：

```bash
export PROMETHEUS_MAVROS=/实际路径/prometheus_mavros
export PROMETHEUS_PX4=/实际路径/prometheus_px4
```

功能包使用 ROS Noetic、Eigen、PCL 以及 Prometheus 消息。若笔记本官方环境缺少运行依赖，再安装：

```bash
sudo apt install libeigen3-dev libpcl-dev \
  ros-noetic-laser-geometry ros-noetic-pcl-ros \
  ros-noetic-octomap-server ros-noetic-octomap-rviz-plugins
```

## 6. 建议的 `load_prometheus` 环境函数

可将以下函数追加到笔记本的 `~/.bashrc`。如果笔记本已有同名函数，应由 Codex 合并内容，不要重复定义：

```bash
load_prometheus() {
  source /opt/ros/noetic/setup.bash
  source "${HOME}/prometheus_mavros/devel/setup.bash"
  source "${HOME}/Prometheus/devel/setup.bash" --extend
  source "${HOME}/prometheus_px4/Tools/setup_gazebo.bash" \
         "${HOME}/prometheus_px4" \
         "${HOME}/prometheus_px4/build/amovlab_sitl_default" >/dev/null 2>&1

  export ROS_PACKAGE_PATH="${ROS_PACKAGE_PATH}:${HOME}/prometheus_px4:${HOME}/prometheus_px4/Tools/sitl_gazebo"
  export GAZEBO_PLUGIN_PATH="${HOME}/Prometheus/devel/lib${GAZEBO_PLUGIN_PATH:+:${GAZEBO_PLUGIN_PATH}}"
  export GAZEBO_MODEL_PATH="${HOME}/Prometheus/Simulator/gazebo_simulator/gazebo_models/uav_models:${HOME}/Prometheus/Simulator/gazebo_simulator/gazebo_models/sensor_models:${HOME}/Prometheus/Simulator/gazebo_simulator/gazebo_models/scene_models:${HOME}/Prometheus/Simulator/gazebo_simulator/gazebo_models/texture${GAZEBO_MODEL_PATH:+:${GAZEBO_MODEL_PATH}}"
}
```

保存后运行：

```bash
source ~/.bashrc
load_prometheus
```

专用仿真脚本已经自行加载 ROS、MAVROS、Prometheus 和 PX4 环境，但原有的 `arm_and_command.sh` 仍依赖当前终端环境，因此启动整套实验前执行一次 `load_prometheus` 最稳妥。

## 7. 编译

如果笔记本的官方 Prometheus 从未完整编译，先按官方方式完成一次全工作空间编译。之后只编译受影响的两个模块：

```bash
cd ~/Prometheus
source /opt/ros/noetic/setup.bash
source ~/prometheus_mavros/devel/setup.bash
source devel/setup.bash --extend

catkin_make --source Modules/uav_control \
  --build build/uav_control -j4
catkin_make --source Modules/coverage_searching_pkg \
  --build build/coverage_searching_pkg -j4
```

重新打开终端或执行：

```bash
load_prometheus
```

## 8. 编译后检查

```bash
rospack find mavros
rospack find px4
rospack find prometheus_coverage_search

bash -n Scripts/simulation/searching_pkg/coverage_search_P450.sh
bash -n Scripts/simulation/searching_pkg/coverage_search_node.sh

roslaunch --files prometheus_gazebo sitl_coverage_search_1uav_P450.launch
roslaunch --nodes prometheus_uav_control \
  uav_control_main_outdoor.launch joy_enable:=false
roslaunch --files prometheus_coverage_search coverage_search.launch
```

三个 `rospack find` 都必须返回有效目录。控制器节点检查应出现 `/uav_control_main_1`；`joy_enable:=false` 用于无遥控器仿真，避免 `/dev/input/js0` 警告干扰诊断。

## 9. 启动顺序

终端 1：

```bash
load_prometheus
~/Prometheus/Scripts/simulation/searching_pkg/coverage_search_P450.sh
```

等待 PX4/MAVROS 建立连接，可检查：

```bash
rostopic echo -n1 /uav1/mavros/state
```

只有看到 `connected: True` 后才在终端 2 解锁并切换命令模式：

```bash
load_prometheus
~/Prometheus/Scripts/simulation/NO_RC/arm_and_command.sh
```

终端 3 启动覆盖搜索：

```bash
load_prometheus
~/Prometheus/Scripts/simulation/searching_pkg/coverage_search_node.sh
```

若出现 `Waiting for connect PX4!`，先检查第一个终端是否有
`cannot launch node of type [mavros/mavros_node]`，以及 `rospack find mavros` 是否成功；不要反复执行解锁脚本掩盖连接根因。

## 10. 无网络时的 U 盘方案

U 盘方案同样可靠。主机上生成只包含必要文件的压缩包：

```bash
cd ~/Prometheus
tar -czf coverage-search-transfer.tar.gz \
  COVERAGE_SEARCH_LAPTOP_MIGRATION.md \
  Modules/coverage_searching_pkg \
  Scripts/simulation/searching_pkg \
  Simulator/gazebo_simulator/gazebo_worlds/planning_worlds/samplemaze.world \
  Simulator/gazebo_simulator/launch_basic/maze_mapping.rviz \
  Simulator/gazebo_simulator/launch_basic/sitl_coverage_search_1uav_P450.launch
```

将压缩包复制到 U 盘。笔记本上先确认 `~/Prometheus` 工作区干净，再解压：

```bash
tar -xzf /U盘实际路径/coverage-search-transfer.tar.gz -C ~/Prometheus
```

然后仍需让笔记本 Codex 执行第 4 节的控制器语义补丁，并完成第 7、8 节。不要把主机的 `.git/`、`build/`、`devel/` 或整个 `~/Prometheus` 复制到笔记本。

## 11. 给笔记本 Codex 的完成清单

- [ ] 先保护笔记本现有未提交修改。
- [ ] 只导入第 2 节列出的 5 组新增路径。
- [ ] 对两个 `uav_controller` 文件应用第 4 节最小补丁。
- [ ] 确认 MAVROS、PX4 和 Prometheus 路径存在。
- [ ] 合并或添加 `load_prometheus`，不要重复定义。
- [ ] 编译 `uav_control` 和 `coverage_searching_pkg`。
- [ ] 三个 `rospack find` 和三个 `roslaunch` 静态检查通过。
- [ ] MAVROS 显示 `connected: True` 后再解锁。
- [ ] 最后在笔记本自己的 Git 分支提交迁移结果。
