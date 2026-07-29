# 双无人机协同覆盖：设计决策与实施记录

> 本文是持续更新的唯一设计记录。每次确认需求、改变接口、完成代码或验证，
> 都追加到“决策记录”和对应章节；未决问题不得从文档中删除，只能标为已解决。

## 目标与范围

- 首先完成 Gazebo 双机仿真；仿真架构必须保持可移植到两台独立工控机。
- 每台无人机有独立 `roscore`，实机使用 `swarm_ros_bridge` 通过局域网 TCP
  转发指定 ROS 话题。
- 每台机独立维护体素地图、前沿和本机轨迹；两机共享经过版本验证的地图块、
  状态、未来轨迹和任务分配信息。
- 首版为固定飞行高度的二维覆盖路径与三维体素感知。默认 `fly_height=1.5 m`，
  必须可由 launch 覆盖。
- 避障是硬约束：静态障碍物和对机预测轨迹任一不安全时，不得继续执行轨迹。
- 当前版本已加入轨迹重规划机制。
- 当前版本已加入Nlopt求解器。

## 已确认的仿真基线

- 首个协同仿真使用已有的 `P450 + D435i`，世界使用 `samplemaze.world`。
- Gazebo world 只加载迷宫、地面等静态环境；两架 P450 由 launch 分别
  `spawn_model`，不复制同内容的 `2uav_samplemaze.world`。
- 两机朝向相同、在与朝向垂直的方向并排生成。初始中心、航向、横向间距均为
  launch 参数；位置由中心和航向推导，不能把 UAV ID 或出生点写死在源码。
- RViz 只需一个实例，同时显示两组 `/uav<ID>/...` 话题；可保存专用 RViz
  配置，但不为所有节点机械添加 `2uav` 前缀。
- 当前单机覆盖包已生成并实时执行时间参数化三次 B 样条：De Boor 求得
  P/V/A 后，以 `UAVCommand::TRAJECTORY` 发布。它不是 EGO-Swarm 的
  `traj_utils/Bspline` 消息，但可直接作为本项目对机轨迹预测的来源。

## 身份与部署

- 代码必须在两台机完全相同；差异仅来自 launch/YAML 参数：`self_id`、
  `peer_ids`、本机/对端 IP、端口、传感器话题与标定。
- 参与编队的真实无人机不固定，例如可为 UAV1 和 UAV3，而不是假定 UAV1/UAV2。
- 新 ROS 包名不能以数字开头。候选名为 `prometheus_two_uav_coverage_search`；
  在确认创建前，不复制现有单机包。
- 已检查的 UAV1/UAV5 工程结构基本相同，但也有真实硬件差异：ID、MAVROS
  system ID、MID360 host/lidar IP，以及 Fast-LIO 外参估计开关。因此绝不复制
  某一机的硬编码配置到另一机。
- 不修改 Prometheus 原有 PX4、MAVROS、`uav_control`、FAST-LIO、Gazebo、
  通信桥等功能包的源码。双机逻辑、消息、launch、RViz、bridge 配置和部署脚本
  均属于新功能包；原有节点仅作为话题接口使用。

## 坐标系与实机启动校验

### 已确认操作

两台机将依次放到同一物理参考点、同一朝向启动各自的 MID360/FAST-LIO；
随后移至各自并排起飞点。该过程定义共同的名义原点和名义航向；不要求操作者
精确量取两机相对位置，只要求初始水平间距大于可配置下限（首版 1.0 m）。

### 必须实现的起飞前检查

两套独立 FAST-LIO 仍可能有不同的初始误差和累计漂移，因此“同点启动”不是
自动的严格坐标对齐保证。bridge 应在解锁前运行；协同覆盖节点可启动但必须保持
暂停。两台机都完成定位初始化、移至并排起飞点后，先解锁并起飞至目标高度稳定
悬停，再由 bridge 交换状态并执行协同自检：

1. 双方 `odom_valid`、bridge 双向心跳、控制状态和目标高度稳定均为真；
2. 通过 bridge 交换位姿和航向，检查估计的水平间距不小于
   `min_start_separation`；
3. 检查双方航向差不超过 `max_start_yaw_difference`；
4. 通过后才将协同状态从 `WAIT_SWARM_READY` 切换为竞价和覆盖，而不是立即在
   节点启动时开始探索。两台机不要求同一时刻启动 launch：先启动的一方持续等待。

最小间距、航向容差和等待时间均为 launch 参数。FAST-LIO 本身只能估计本机在
共同名义坐标系中的位姿；它不会自动识别“哪一个激光点是另一架无人机”。因此
对机相对位置来自 bridge 交换的两机估计，不能声称只靠激光就完成对机识别。
飞行中若共享地图重合质量失效，远端体素不得参与任务分配；本机静态避障和紧急
悬停保持有效。

## 通信与消息

- 使用已安装的 `swarm_ros_bridge`。其底层为 ZeroMQ TCP PUB/SUB，适合两个
  独立 ROS master；需要扩展其默认仅支持的 `Imu`、`Twist`、`String` 消息类型。
- 协同包和扩展后的 bridge 必须在两台工控机以相同版本编译。
- 需要的逻辑消息：本机状态、时间参数化未来轨迹、地图 chunk 摘要/快照/增量、
  竞价表、任务承诺、紧急悬停与链路确认。
- 不依赖 ROS 的跨机 master；IP 和端口只在每台机自己的 bridge YAML 中配置。
- 对机 B 样条是按绝对时间预测的；实机两台工控机必须先以 chrony/NTP/PTP 同步
  系统时钟，并在起飞前检查偏差。该项不通过时不能启用对机时空防撞。
- chrony 是操作系统服务，不能由普通 ROS 节点安全地永久修改系统时钟。新包只
  提供一次性 sudo 部署脚本、配置模板和起飞前偏差检查；协同节点负责测量/门控，
  不负责偷偷修改系统时间。

### 实机 chrony 部署流程

1. 在两台工控机的 Ubuntu 系统中安装并启用 chrony；这一步独立于 ROS。
2. 每次选择参与机后，最低 UAV ID 的工控机使用新包提供的 `--master` 配置，
   其余工控机使用 `--server <最低ID机IP>` 配置。
3. 两台机联网后执行 `chronyc tracking`，确认 chrony 已同步且 Last offset 小于
   `max_clock_offset`（首版 20 ms）；未通过时只允许悬停，不允许协同覆盖。
4. 协同节点还会以 bridge 的时间戳消息检查时钟偏差和抖动，防止 chrony 服务
   虽启动但链路/配置异常。

## Chunk 地图同步

### 识别与一致性

- 预定义覆盖地图按固定世界坐标切分。使用 `chunk_id`（不是网络 IP）作为唯一
  标识；推荐由 `(cx, cy, cz)` 按字典序编码，避免跨机碰撞。
- 所有参与机必须使用同一 `map_epoch`、原点、尺寸、分辨率和 chunk 尺寸；
  几何哈希不一致的包直接拒绝。
- 每个发送者对每个 chunk 维护单调递增 revision。接收方仅应用连续 revision；
  发现缺失 revision 或刚接入时，先请求最新完整快照，快照成功后才接收增量。
- “版本一致”定义为：不会静默应用错图或乱序数据，且重连后收敛到最新已同步
  chunk；两个独立传感器的本地地图不可能在每一时刻逐比特相同。

### 传输策略

- 所有 chunk 以低频摘要公布：`chunk_id`、已知比例、最新 revision、前沿摘要。
  接收方仅在缺少或落后时请求详细内容。
- 已知比例到达 25%、50%、75%、100% 时发送完整快照，作为断链/漏包恢复检查点。
- 首次达到 25% 后，每 `0.5 s` 发送本周期改变的体素增量；增量为空则不发送。
- 未达到 25% 时不发送完整体素，但可发布摘要供任务分配判断区域活跃度。

### Log-Odds 体素融合

- 不采用“任一机曾观测占据就永久占据”的规则。每次扫描都可改变体素的占据证据。
- 现有 `CoverageMap` 已有饱和的量化证据缓存：命中使证据增加、自由射线使证据
  减少，当前范围为 `[-3, 4]`，再由阈值导出 unknown/free/occupied 状态。
- 远端每个 chunk 维护独立的 `peer_evidence` 层，不能直接覆写本机
  `local_evidence`。全量快照替换该来源在该 chunk 的证据层；增量发送
  `(voxel_index, new_quantized_log_odds)`，而不是不可重放的“加一/减一”操作。
- 融合状态由本机和全部远端来源的当前 Log-Odds 证据求和、饱和并阈值化得到。
  因此后续自由观测可以抵消旧占据观测，重传或乱序包不会将证据重复累计。
- 每个来源的 revision 仍独立校验；地图对齐质量差时可将对应来源权重降为零，
  而不是把其旧占据点永久留在地图中。

## 分布式共识竞价任务分配

### 已确认方向

首版直接采用分布式共识竞价，而不是由低 ID 无人机集中计算。每台机运行相同
代码和相同算法，通过 bridge 交换竞价状态。

这属于拍卖算法的一个分布式共识版本；具体采用“共识式 bundle auction”思想：
每台机都维护任务赢家表与自己的任务 bundle，收到对方消息后按确定性的赢家
规则合并，最终收敛到同一份分配。

### 任务与 bundle

- 任务不是单一视点。任务单位是预定义 chunk 内的前沿簇；一个簇含多个候选视点。
- 每机维护未来 `task_bundle_size` 个任务，首版默认 3；每机在已分到的前沿簇内
  选择当前安全、代价最低的视点。
- 每个任务有稳定任务 ID，例如 `map_epoch/chunk_id/frontier_id`。大前沿沿用已有
  的 split 逻辑，避免单个任务过大。
- 出价使用预计增量路径时间、偏航代价、信息增益、已承诺任务和时空冲突惩罚。
- 同一任务的冲突按更优代价获胜；完全相等时按更低 UAV ID 打破平局。赢家、
  代价、任务版本、消息序号共同广播。
- 每 2 秒以及任务失效/完成/安全事件时触发重竞价；不能永久分配整张地图。

## 对机安全约束

- 两机持续广播本机位姿、速度和未来时间参数化 B 样条。
- 每次任务提交、轨迹生成和轨迹执行期间均采样未来时间窗，检查两机时空距离。
- 预测冲突、远机轨迹过期、对机失踪或本机感知到动态障碍时，立即发布锁存
  `EMERGENCY_HOLD` 并发送当前位悬停命令；任务承诺失效，重新竞价前必须通过
  双方安全检查。
- MID360/D435i 看到的对机不能永久写入静态体素地图；后续实现时要按远机预测
  位姿过滤该动态点，并把对机作为独立时空障碍物。

## 通信失联：待最终确认的安全状态机

用户提出：低 ID 无人机继续探索并寻找高 ID 无人机，高 ID 无人机悬停。

这个策略只有在“高 ID 已被可靠确认进入悬停”后才安全。单向链路故障时，低 ID
可能收不到高 ID，而高 ID 仍收得到低 ID；若仅凭单向心跳判定，双方可能一个继续
飞、一个继续飞，违反硬防撞要求。

待实现的安全前提候选：

1. 心跳必须双向确认（每包确认所见的对端序号）；任何一方无法证明双向连通时，
   两机先同时进入 `LINK_LOST_HOLD`。
2. 低 ID 只有通过本机传感器确认高 ID 已静止、位置可追踪且其周围已作为动态禁区
   后，才可从悬停恢复单机探索。
3. 若 D435i 的有限 FOV 无法确认高 ID 位置，则不得恢复；保留到 MID360 版本再
   评估是否允许该降级模式。

在以上前提确认前，仿真首版的安全行为为双方悬停。

## 实施顺序

1. 参数化现有单机 launch，并建立一次加载 world、两次 spawn P450+D435i 的仿真。
2. 在同一 ROS master 下验证两套单机覆盖节点、独立话题和初始安全距离。
3. 添加协同消息、chunk 状态机、共识竞价、任务 bundle 与对机时空安全检查。
4. 用 loopback/两 master 验证扩展后的 `swarm_ros_bridge`；再将仿真协同话题改由
   bridge 转发，验证断连、重连、快照恢复和冲突悬停。
5. 转接 MID360 定位 + D435i 感知，完成实机起飞前对齐自检、地面测试和系留测试。

## 人工启动方式

- 笔记本通过两个 NoMachine 窗口分别操作两台工控机，不能以一条命令同时远程
  启动两机。
- 每台工控机各自启动一次相同的协同 launch。先启动的节点只执行 bridge 连接、
  状态发布和 `WAIT_PEER`，绝不发布覆盖轨迹。
- 第二台启动后，双方完成双向握手；两机已由遥控器起飞并稳定悬停时，状态机自动
  从 `WAIT_SWARM_READY` 同步进入竞价与覆盖。启动先后相差数秒或数分钟均可。

## 实机任务前检查与起飞流程（已确认）

1. 两台工控机启动 chrony；检查 `chronyc tracking`，两机相对时钟偏差小于 20 ms。
2. 两机依次在同一物理参考点、同一朝向启动各自 FAST-LIO，再移动到各自起飞点。
3. 两机保持相同朝向，并在朝向的横向相隔大于 `min_start_separation`（首版 1.0 m）。共同前向为全局 x 轴时，该横向方向就是全局 y 轴；一般情况以“垂直于共同朝向”为准，而不是固定全局 y 轴。
4. 两端分别启动 `swarm_ros_bridge` 和协同节点。先启动的一端只能停在 `WAIT_PEER`，不能开始搜索。双方通过心跳、双向确认、有效里程计、时钟检查和最小间距检查后，进入 `WAIT_TAKEOFF`。
5. 飞手用遥控器分别解锁、起飞，并让两机在同一目标高度稳定悬停。两机还必须进入 Prometheus 的 `COMMAND_CONTROL` 状态，协同节点才允许发送轨迹命令。
6. 协同节点确认双方均已就绪（在线、里程计有效、间距安全、命令控制、悬停稳定）后，才同步切换至协同覆盖、开始竞价和轨迹执行。两端 launch 可以先后启动，不要求同时操作。

首版不能直接以 `auto_start:=true` 单独启动原单机覆盖节点；新功能包必须在上述协同门禁通过前屏蔽其覆盖轨迹输出，避免某一架先自行搜索。

## 决策记录

### 2026-07-27：贴墙前沿簇反复生成修正

- 问题表现：完整墙面旁的一小片前沿簇已经进入相机 FOV 并短暂消失后，飞机转向又会
  出现；无人机可能持续十余秒重复观察同一处。非贴墙前沿在被观测一次后能正常消失。
- 根因是三态地图把“融合证据冲突”也表示为 `unknown`。两机从不同视角观测同一面墙时，
  0.2 m 体素量化可能让一台将某格作为命中端点、另一台将其作为端点前的 free 射线。
  两层证据相加后落在 `0` 或 `+1`，旧逻辑会把已经观测过的格重新写成 `unknown`；其
  邻接的 free 格随即再次满足“free 的 6 邻域含 unknown”的前沿定义。
- 证据饱和范围统一扩大为 `[-5, +6]`：本机 hit/miss、远端证据接收、融合饱和和机体/
  对机体积清除均使用同一范围；状态阈值仍为 `>= +2` occupied、`<= -1` free。
- 新增 `observed_buffer`。仅从未观测的体素保持 `unknown`；一旦本机 hit、miss、明确
  体积清除或非零远端证据使其成为已观测体素，融合值处于冲突区间 `0/+1` 时保持其上一
  个 `free` 或 `occupied` 状态，不再退回 `unknown`。只有融合证据达到上述明确阈值时，
  已知体素才在 free 与 occupied 之间转换。
- 前沿算法无需改变：`occupancy == 0` 现在只代表从未观测，因而已观测墙体不会因双机
  融合短暂冲突再次作为贴墙前沿的 unknown 邻域。`ROS_ASSERT` 保证已观测体素不会被
  状态转换逻辑写回 unknown。已执行 `git diff --check` 和
  `make -C build/two_uav_coverage_searching_pkg -j2`，编译通过。

### 2026-07-24：RViz 地面体素显示过滤

- 地面体素的当前处理仅在 RViz：两个 `octomap_rviz_plugin/OccupancyGrid`
  （`/uav1/octomap_full`、`/uav2/octomap_full`）均设置
  `Min. Height Display=0.5`。`samplemaze.world` 地面为 `z=0`，该余量同时
  裁掉深度 15 合并后仍可能露出的低层块。
- 此设置只影响显示，不修改 `octomap_full`、`octomap_binary` 或覆盖规划地图；
  不启用 `octomap_server` 的 RANSAC `filter_ground`。修改后仅需重新加载 RViz
  配置，无需重启 Gazebo 或算法节点。

### 2026-07-24：协同飞行中周期性颠簸修正

- 症状是两架飞机均会在前进中反复“抖一下、回正、再抖”，并非仅在两机接近时发生。
  根因位于 `two_uav_coordinator`：原 `stableAtHeight()` 同时要求目标高度误差不超过
  0.30 m 和水平速度不超过 0.35 m/s；20 Hz 主循环在已经进入 `ACTIVE` 的飞行阶段仍调用
  它。覆盖轨迹允许 `max_vel=1.0 m/s`，所以飞机每次加速越过 0.35 m/s 都会被改发
  当前位姿的 `XYZ_POS` 悬停命令；减速后又恢复 `TRAJECTORY`，形成周期性刹车与再加速。
- 修复将两个语义分开：`stableAtHeight()` 保留为起飞门禁，仅在 `WAIT_PEER` 或
  `WAIT_TAKEOFF` 阶段要求低速稳定；新增 `altitudeAtHeight()` 只检查高度，在 `ACTIVE`
  飞行阶段继续检查两机高度，但不把正常飞行速度当作悬停条件。
- 此修改不改变 `max_vel=1.0`、`max_acc=0.8`、最小起飞间距、状态/通信有效性或真实的
  双机安全距离判断。编译目标 `two_uav_coordinator` 和 `git diff --check` 已通过。验收时，
  起飞稳定后应连续输出轨迹；若仍发生悬停，协调器终端的 `UAV ... holds:` 原因可用于区分
  双机安全判断与地图安全重规划，而不再是 0.35 m/s 的误触发。

### 2026-07-24：位置与视点偏航同步轨迹

- 需求是无人机抵达视点位置时同时抵达该视点的指定偏航角；偏航不能在轨迹末段才集中
  完成。原时间 B 样条把偏航控制点按 `fraction / 0.80` 提前压缩到前 80%，而执行端又对
  已规划偏航施加第二个逐帧限速器，可能造成参考滞后并把剩余转向留给终点悬停。
- 时间 B 样条现在使用与位置相同的完整时间域线性控制点，并保留起止处的零速/零角加速度
  边界；有效轨迹的偏航连续、单调地从当前朝向变化到视点朝向。执行端直接下发经 De Boor
  验证的 `desired_yaw`，不再二次限速。
- 新增 `coverage_search/max_yaw_acc=1.8 rad/s²`（`max_yaw_rate=1.8 rad/s` 不变）。轨迹
  时长同时受路径长度、偏航速率和偏航角加速度约束；短距离而初末朝向差大时会拉长整段
  轨迹，不以末段急转补偿。
- 候选轨迹采样校验峰值偏航速率、峰值偏航角加速度及单调性；另外要求偏航进度在 25% 时
  已开始、50% 位于 35%--65%、75% 至少完成 65%，否则拒绝该候选。`two_uav_coverage_search_node`
  已重新编译通过，`git diff --check` 无格式错误；运行日志会输出 `peak_yaw_rate` 与
  `peak_yaw_acc` 供验收。

### 2026-07-19：地面显示、前沿实时性与路径代价竞价修正

- 对比单机 `coverage_searching_pkg` 后确认：内部覆盖地图的 D435i 地面处理逻辑
  已一致——低于 `depth_ground_ignore_height` 的回波只用于 free 射线、不写成内部
  occupied。双机仿真中遗漏的是 **OctoMap 输入链路**：现已逐项恢复为实际单机仿真
  `sitl_coverage_search_1uav_P450.launch` 的 `pointcloud_min_z=-0.05`、
  `pointcloud_max_z=2.8`、`filter_ground=true`、`ground_filter/distance=0.10` 与
  `ground_filter/plane_distance=0.12`。RViz 同时保留 OctoMap 的 `Min. Height Display=0.2`，因此不显示
  低高度地面体素；这只影响显示，不影响覆盖规划的二值/Log-Odds 地图。
- `CoverageMap::setRemoteEvidence()` 现返回“证据是否实际改变”。重复到达的 chunk
  快照或相同增量不再使 `remote_frontier_refresh_pending` 置位，避免每 3 秒无意义地
  重做一次完整前沿扫描。
- 前沿逻辑保持：本机深度更新只扫描更新 AABB 及相交旧簇；远端体素真的改变时才
  请求一次完整扫描，且完整扫描频率被 `remote_full_frontier_period=3.0 s` 限制。
  全局扫描仍可能较慢，但它不应再周期性由重复数据触发。
- 原竞价的 `costTo()` 是欧氏直线距离，不能反映隔墙绕行，正是出现“大绕路、两机
  聚集后等待”的根因之一。现增加 `SwarmBid/SwarmBidArray`：每架机每 2 秒在其本地
  融合地图中对至多 16 个候选计算安全直达或 A* 路径长度（最多 6 次完整 A*），并
  通过 bridge 的新 `/bid` 通道广播。只有两架机都给某任务提供可达 bid 时，协调器
  才以两条真实路径代价做成对任务分配；不可达/尚未评估任务不分配。这个 bounded
  预算避免把路径评估重新变成主循环卡顿来源。
- 任务承诺上限从 30 秒降至 15 秒：若轨迹/目标在安全约束下没有推进，及时进入
  冷却并重新竞价，而不是两机长时间悬停等待旧任务。
- 修正 bid 启动死锁：本机暂时没有前沿时也必须发布空 bid，并且仍对 bridge 收到的
  对方前沿计算自己的 A* 测地 bid；不能因 `local_frontiers.empty()` 直接返回，否则
  一方先发现前沿时两架机会永远停在 “waits for fresh A* bids”。
- 空 bid 也是链路心跳：即使本机地图尚未 ready，也按 `swarm_bid_period` 发送空数组。
  因而 coordinator 可区分“对方在线但暂时无可分配任务”和“bridge/bid 链路失效”。

### 2026-07-18

- 确认首个双机仿真使用 P450+D435i；后续实机使用 MID360 定位、D435i 感知。
- 确认首版采用固定高度；飞行高度 launch 可调，默认 1.5 m。
- 确认采用 `swarm_ros_bridge`，两台机各自运行 `roscore`。
- 确认采用分布式共识竞价，并分配多任务 bundle，不是每机单个视点。
- 确认 chunk 用预定义 ID 和按需请求；25/50/75/100 全量快照与 25% 后 0.5 秒
  体素增量结合使用。
- 确认预测碰撞时以悬停和重新规划为硬规则。
- 确认首版 `min_start_separation=1.0 m`；操作者只需将两机放开，不要求量取
  精确起飞相对位姿。
- 确认起飞由遥控器完成；通过两个 NoMachine 窗口分别启动协同 launch。先启动
  的节点等待对机，双方达到稳定悬停并握手完成后才自动开始协同任务。
- 确认首版复用单机地图量化 Log-Odds 范围 `[-3, 4]` 与阈值
  （`>=2` 占据、`<=-1` 自由）。
- 确认不修改 Prometheus 原有控制、定位、仿真与通信功能包源码；新功能包通过
  既有 ROS 话题与它们协作。
- 确认新建 `prometheus_two_uav_coverage_search`，并允许对本项目的
  `coverage_searching_pkg` 增加极少量只读接口，以复用其地图、前沿与 B 样条，
  不改变其原有单机运行行为。
- 确认 chrony 在工控机操作系统中配置；先完成系统同步，再用 `chronyc tracking`
  检查时间偏差。
- 确认实机流程：共同参考点启动定位、横向分开至少 1 m、bridge/协同节点握手、
  遥控器起飞稳定悬停并进入 `COMMAND_CONTROL` 后，才自动开始协同覆盖。
- 新问题：低 ID 在通信失联后继续探索的恢复条件尚未确认，见“通信失联”。
- 新问题：需确认 P450+D435i 双机仿真后，MID360 接入使用何个 Gazebo 模型和话题。
- 新问题：实机时钟同步方案与允许最大时钟偏差尚未确认；它是带时间戳轨迹防撞的
  前置条件。
- 新问题：需确定 Log-Odds 融合的远端证据权重、阈值和 chunk 快照编码；不采用
  永久占据优先规则。
- 已解决：允许新双机包以只读接口复用/小幅扩展本项目已有的单机覆盖包；禁止
  修改 Prometheus 原有控制、定位、仿真与通信包。

## 代码与验证记录

### 2026-07-18：首个可编译实现

- 新建 `Modules/two_uav_coverage_search`（ROS 包名
  `prometheus_two_uav_coverage_search`）。它包含协同消息、基于已下载
  `swarm_ros_bridge` 实现改写的专用 ZMQ bridge、双机协调器、双机仿真 launch 和部署脚本。
- bridge 只转发协同实际需要的六类消息：状态、前沿、任务 bundle、轨迹、地图 chunk 和
  chunk 请求；不修改用户下载的原始 bridge 包，也不使用 Prometheus 原通信包。
- 单机 `prometheus_coverage_search` 仅增加默认关闭的 `cooperative_mode` 接口：
  外部目标模式、原始命令输出、前沿/B 样条采样发布、2 m chunk 的 25/50/75/100 快照、
  25% 后 0.5 s Log-Odds 增量、远端证据层融合和缺块快照请求。默认单机 launch 的行为不变。
- 协调器在两机均在线、里程计有效、`COMMAND_CONTROL`、目标高度稳定、横向距离至少
  1 m 前只悬停；协同模式下按前沿候选的确定性成本和 UAV ID 平局规则给每机最多 3 个任务，
  并在将原始覆盖指令转发给 Prometheus 前检查对机状态和未来轨迹采样。状态/轨迹过期或
  预测距离不足时发布悬停命令。
- 时钟配置与检查已分成 `configure_chrony_peer.sh` 与 `check_clock_offset.sh`；后者是只读
  飞前检查，要求 Last/RMS offset 均小于给定阈值（默认 20 ms）。bridge 与协同覆盖也各有
  独立启动脚本。
- 已执行：新包 `catkin_make` 成功；改动后的 `coverage_searching_pkg` 成功；所有 launch
  文件 XML 语法检查、所有脚本 `bash -n` 和 `git diff --check` 均通过。双机 Gazebo
  smoke launch 已成功拉起两个协调器、两个 bridge、两个覆盖节点、两个深度降采样节点和
  两个控制节点；第一轮发现并修复了三维体素增量缓存按二维大小分配导致的内存越界。
- smoke 中仍发现现有多 PX4 SITL 启动链的一个环境级问题：首架 PX4 在 25 秒启动窗口内
  发生 simulator poll timeout；另有 Prometheus 既有的 `map_base_link` 重复 TF 警告。
  它们发生在原有 PX4/控制仿真链，协同节点和覆盖节点未再崩溃。必须在下一轮完成
  双机解锁、起飞、`COMMAND_CONTROL` 和实际覆盖验证后，才能称为仿真完成。
- 尚未执行：Gazebo/PX4 全流程起飞与覆盖、双独立 ROS master 的端到端网络测试、实机
  chrony 配置、FAST-LIO 对齐验收、系留测试。它们是实机部署前的强制验证步骤，不能由
  “可编译”替代。

### 2026-07-18：独立功能包范围修正

- 已确认先前“协同外挂层”范围不足。`prometheus_two_uav_coverage_search` 必须在包内
  包含完整的单机覆盖实现，而不能在运行时启动或依赖另一个
  `prometheus_coverage_search` 节点。
- 已将单机包的头文件、地图 Log-Odds、前沿簇/视点、二维 A*、分层网格、覆盖规划、
  B 样条轨迹、覆盖节点和 D435i 点云降采样节点复制并命名隔离到新包。新包由 7 个
  覆盖核心 CPP、2 个本地节点 CPP，以及 bridge 与协同调度 CPP 组成；协同能力在同一
  本地覆盖实现之上运行。
- 新的 `two_uav_onboard.launch` 与 `two_uav_coverage_sim.launch` 已改为只启动本包的
  覆盖节点和点云节点，不再在运行时引用 `prometheus_coverage_search`。原单机包保留，
  供既有单机实验兼容使用。
- 此文件从此作为双机功能包的主设计记录，随
  `Modules/two_uav_coverage_search` 一起部署；旧包内同名文件只保留历史副本。

### 2026-07-18：独立包启动验证

- 新包独立 `catkin_make` 已成功，生成 `two_uav_coverage_search_node`、
  `two_uav_depth_cloud_downsample_node`、`two_uav_swarm_bridge` 和
  `two_uav_coordinator` 四个可执行文件；`two_uav_onboard.launch` 的展开结果只含本包
  的覆盖节点，不含旧单机包节点。
- 加载既有 PX4 Gazebo 环境后，双机 Gazebo launch 能展开并实际启动两个新覆盖节点；
  两者均完成 96x96x15 体素地图、前沿、A* 和分层网格初始化，无进程崩溃。
- 本次完整飞行 smoke 被已存在的 Gazebo/PX4 实例阻断：模型名已存在，导致新实例的
  MAVROS 无法连接。未停止该已存在实例；在干净仿真会话中仍需继续完成解锁、起飞和覆盖
  验证。

### 2026-07-18：启动脚本与高度一致性

- 仿真总入口是 `scripts/start_two_uav_sim.sh [uav1-id] [uav2-id] [fly-height-m]`。
  它加载 ROS、定制 MAVROS、PX4 Gazebo 环境，然后启动唯一的
  `two_uav_coverage_sim.launch`；该 launch 依次加载迷宫 world、两套 PX4、两套 MAVROS、
  两套 `uav_control`、两套 D435i 处理、bridge 和协同覆盖节点。
- 仿真解锁入口是 `scripts/arm_two_uav_sim.sh [uav1-id] [uav2-id] [settle-seconds]`。它只在
  两个 `/uav<ID>/prometheus/setup` 话题存在时，先解锁两机，再切换二者到
  `COMMAND_CONTROL`。原生 `uav_control` 的初始位置悬停命令会使其爬升到配置高度；双机
  协调器确认二者高度稳定后才开始覆盖。
- 修正：总 launch 的 `fly_height` 现在同时写入两套 `uav_control` 的
  `control/Takeoff_height`，不再只改变覆盖节点高度。仿真中这三个高度参数始终一致。
- 实机不使用解锁脚本：飞手遥控器完成起飞和悬停；两台工控机分别运行
  `start_swarm_bridge.sh <local-ip> <peer-ip>` 与
  `start_two_uav_coverage.sh <local-id> <peer-id> [height]`。后一个脚本明确要求双方 ID，
  启动先后无关；高度必须与飞手实际悬停高度/本机 `uav_control` 参数一致。
- 修复：`arm_two_uav_sim.sh` 的话题存在检查不再使用带 `pipefail` 的
  `rostopic list | grep -q` 管道；该组合会在匹配时触发无害的 BrokenPipe，却被错误当作
  “话题不存在”。现改为先完整读取话题列表再匹配。
- 解锁脚本在切换 `COMMAND_CONTROL` 前等待两机 `/prometheus/state` 都报告 `armed=true`
  （最长 15 s），避免 MAVROS/PX4 尚未完成解锁服务时固定 3 s 等待造成的竞态。
- 故障复盘：若已有 Gazebo/PX4 实例未退出，又启动新的双机 launch，Gazebo 保留旧模型和
  instance 0，而新 launch 可能只产生部分 PX4/MAVROS 进程；表现为一机连接到旧飞控甚至
  进入 `AUTO.RTL`，另一机 `connected=false`。`start_two_uav_sim.sh` 现在先检测
  `/gazebo/get_world_properties`；发现已有 world 就拒绝启动，必须先清空旧仿真。
- 清理：P450+D435i Gazebo 模型自身已发布 `camera_link -> d435i_link`；双机 launch 不再
  额外启动同一变换的两个 `static_transform_publisher`，消除 `TF_REPEATED_DATA` 警告。
- 解锁门禁补充：`arm_two_uav_sim.sh` 先等待两机都上报 `connected=true` 与
  `odom_valid=true`（最长 30 s），任何一机未连接就不向任一飞控发布解锁，避免单机起飞。

### 2026-07-18：仿真任务分阶段启动

- 仿真不再在起飞前启动协同覆盖。`two_uav_sim_world.launch` 只加载 world、两套
  PX4/MAVROS、控制节点和一个 RViz；`two_uav_coverage_sim_algorithm.launch` 只加载
  D435i 降采样、两套本地覆盖、bridge 与协调器。保留 `two_uav_coverage_sim.launch` 作为
  调试用一体化入口。
- 操作顺序固定为：`start_two_uav_sim.sh` -> `arm_two_uav_sim.sh` ->
  `start_two_uav_coverage_sim.sh`。最后一个脚本才是“开始协同覆盖”动作，且三个脚本都接受
  `uav1-id uav2-id fly-height` 参数。
- `arm_two_uav_sim.sh` 将两个 `rostopic pub -1` 请求并行执行，并把两机已解锁后的额外
  等待默认由 3.0 s 降至 0.5 s；两个飞控仍必须先实际报告 `armed=true`，不能为速度跳过
  该安全确认。
- RViz 使用 `rviz/two_uav_coverage_search.rviz`，其基于 Prometheus 的
  `maze_mapping.rviz`，包含两机模型、路径、前沿簇/视点、FOV、两套 octomap，以及两机
  各自融合远端 chunk 后的 `/uav<ID>/prometheus/coverage_search/map_vis`。这两份地图是
  分布式协同地图的本地融合副本，不伪造一个未实际计算的中心“共同地图”话题。
- 启动保护补充：除已有 Gazebo world 外，若检测到旧单机
  `coverage_search_P450.sh` 遗留的 coverage、深度降采样、octomap 或 D435 TF 节点，
  `start_two_uav_sim.sh` 也会拒绝启动。旧单机 `roscore` 中持续的仿真 `/clock` 会导致
  新节点显示数百秒的 `Duration`，并且会与 `/uav1` 话题冲突。

### 当前编译与启动顺序

### 2026-07-18：双机共同坐标系修正

- 首次双机运行的症状是：两台 PX4 均已在 `OFFBOARD` 悬停，但状态消息中的横向
  距离只有约 5 cm，RViz 中两个模型重合，协调器处于 `WAIT_TAKEOFF`（phase 1），
  因而不会发布任何协同目标。这不是 RViz 的单纯显示重复。
- 原因是新仿真 launch 误用了 outdoor/GPS SITL。其每架机的 `local_position`
  都以该机起飞点为原点；该行为在 Prometheus 的 `uav_estimator.cpp` 中已有注释。
  单机没有问题，双机的相对距离和地图融合则不能使用此坐标。
- `two_uav_sim_world.launch` 现改用 indoor/Gazebo estimator：两个
  `/uav*/prometheus/ground_truth` 都处在 Gazebo 的同一个 `world` 坐标系。仅改本包
  仿真 launch，不修改 Prometheus 原始代码。两机出生参数也改为 launch 参数，默认
  `(-6,-1,0)` 与 `(-6,+1,0)`，同朝向、间距 2 m。
- 实机仍采用既定的“共同参考点、共同朝向初始化 FAST-LIO 后再横向分开”的流程；本次
  改动仅使 Gazebo 仿真严格复现这一共同坐标系前提。
- `scripts/start_two_uav_coverage_sim.sh` 在启动 bridge、覆盖与协调器之前会读取两台
  `prometheus/state`，计算二维间距。少于 1 m 时脚本拒绝继续，而不是启动后无声地停在
  `WAIT_TAKEOFF`。该检查同样适用于未来实机的协同节点启动前检查。

### 2026-07-18：首段轨迹后重复悬停修正

- 运行日志显示轨迹正常到达终点，地图仍有 4--6 个前沿、竞价话题仍有多个任务，但
  `/prometheus/coverage_search/raw_command` 变为当前位姿悬停。根因是协调器把已到达的
  首个任务再次作为代价最低的任务；外部目标模式的覆盖节点已清除该目标，双方于是等待
  一个新的目标却从未收到。
- 竞价现在过滤与本机距离小于 `task_reach_dist`（默认 0.35 m）的视点，自动选择任务
  队列中的下一个视点并发布新目标。该参数在 `two_uav_onboard.launch` 中可修改。

### 2026-07-18：融合障碍物体素可视化

- 双机仿真恢复使用原单机验证过的 OctoMap 可视化链：每架机的 D435i 下采样节点发布
  `points_octomap`，各自的 `octomap_server` 输出 `/uav*/octomap_full` 与
  `/uav*/projected_map`。RViz 默认以 `Voxel Coloring: Z-Axis` 显示前者，后者显示
  free（白色）和 unknown（深绿色）的二维地面栅格。
- 原先仅用于内部 Log-Odds 调试的 `map_vis` MarkerArray 已在 RViz 默认关闭，避免和
  OctoMap 体素、地面栅格混淆。

### 2026-07-19：任务承诺与前沿可视化

- 协调器不会在飞向一个尚未到达的已分配视点时，因 2 秒一次的前沿代价重排而重新
  指派另一视点；仅在到达 0.35 m 内或 30 秒任务承诺超时后重新竞价。这防止 A/B
  视点之间往返。前沿仍依据接收的 Log-Odds chunk 重算，不能因为“对方在飞”而盲删。
- 此条已被下文“统一前沿与双机航向”的显示策略取代：默认显示一套融合地图的完整
  前沿簇，另一套仅作为调试开关保留。

### 2026-07-19：统一前沿与双机航向

- 此条已被下文 RViz 原生 Odometry 箭头方案取代。
- RViz 显示 UAV1 的完整 `frontier_vis` 作为“统一前沿簇（UAV1 融合地图）”，UAV2
  的同类显示保持关闭以避免重复。两机仍独立更新与交换 Log-Odds 地图，并非删除
  UAV2 的前沿数据。
- `start_two_uav_coverage_sim.sh` 在启动算法前强制检查
  `/uav<ID>/prometheus/command` 已有对应 `/uav_control_main_<ID>` 订阅者；没有控制器
  时直接退出并报错，不能再出现“算法正在发运动指令、飞行器却悬停”的假成功状态。
- 仿真的 `uav1_id`、`uav2_id` 会同时派生全部 bridge 的 `tx/rx` 话题前缀；不再隐含
  写死为 UAV1/UAV2，因此可直接用 `... 1 3` 验证不同 ID 组合。

### 2026-07-19：RViz、融合前沿与任务保底

- UAV2 的 RViz 配置直接复用 UAV1 的原生 `Odometry` Arrow 显示，仅把话题换为
  `/uav2/prometheus/odom`；不再发布或订阅额外航向话题。
- bridge 使用 ZeroMQ PUB/SUB 的 TCP 回环只验证消息格式、时序和丢包恢复逻辑，不等同于
  实机网络无丢包测试：状态/轨迹按频率限流，断连或慢订阅者可能丢弃旧消息；地图 chunk
  依靠 revision + request/snapshot 补齐。
- 任务表采用确定性的双机成对分配。若有至少两个可达前沿，两个 UAV 各得到一个不同的
  初始任务，剩余前沿再按代价补入任务包，避免所有前沿恰好更靠近 UAV1 时 UAV2 长时间
  无任务。若因硬防撞进入悬停，协调器会输出具体原因（对方状态过期、目标进入安全半径、
  对方轨迹过期或预测轨迹冲突）。
- 每次本地或远端体素更新后都完整重算前沿簇，不能再以“一个旧簇只有不足 15% 单元
  改变”为由继续显示它；统一前沿显示只取 UAV1 的融合地图。占据状态在融合证据回到
  未知区间时会正确清除为未知，不再保留旧的 free/occupied 状态。
- 覆盖率使用融合体素地图的已知体素并集，不能相加两台机的百分比；同一时刻两机日志的
  细小差异只反映 chunk 同步延迟。仿真 OctoMap 过滤 `z < 0.15m` 的地面回波，并关闭会
  反复报错的 RANSAC ground filter；地面显示仅保留 2D projected map 的 free/unknown，
  避免低高度紫色占据体素。
- 协同日志只由低 UAV ID 的融合地图输出 `Global fused Known`，避免把两台机的百分比
  相加或误读为两个独立覆盖率；启动脚本自动选择低 ID 作为该唯一统计源。
- 前沿更新遵循：本机深度更新时仅重算更新 AABB 与受影响簇的 AABB；收到远端 chunk 时
  至多每 3 秒做一次完整重算以校正融合地图。两个首任务优先要求相距至少 3 m，避免在
  同一前沿簇附近聚集；没有满足间距的可达任务对时才回退到普通总代价最小分配。
- RViz 的 OctoMap OccupancyGrid 设置最小显示高度为 0.2 m，地面低高度占据点不再可视化。
- samplemaze 仿真默认出生点为 `(-8,-6)` 与 `(-8,6)`，航向均为 `+x`；横向间距 12 m，
  避免旧的 `(-6,-1)`/`(-6,1)` 同一区域初始观测和任务聚集。

### 2026-07-19：协同启动悬停根因修复

- 实机式通信桥中的 `/uav<N>/two_uav/tx/bid` 曾经没有任何发布者：覆盖节点只发布本机
  `/prometheus/coverage_search/swarm_bids`，协调器订阅后却没有像前沿信息那样转发给 bridge。
  因此两端永远收不到对方的 A* 竞价，协调器持续输出 `waits for fresh A* bids`，不会下发
  首个协同目标。协调器现已把每一帧本机 bid 转发到 `tx/bid`。
- 竞价器曾错误地丢弃“仅一架机在当前已知自由空间连通分量内可达”的前沿，代码条件等价于
  要求两架机都具有有限 A* 代价。样例迷宫开始时两机所在区域之间往往尚未探索出连通走廊，
  所有候选便被过滤，表现为双方一直悬停。现在只在两机都不可达时丢弃；只有一架可达时该机
  获得任务，两个不同区域各自可达时则各自获得一个首任务。
- 此外 `Wall_9` 的最近边界为 `y=6.59 m`，旧的 UAV2 默认出生点 `(-8,6)` 只有约
  0.6 m 净距。默认改为 `(-8,5)`，两机同朝向，且距该墙超过 1.5 m。改出生点后必须重启
  基础仿真（世界/PX4），不能只重启算法。

### 2026-07-19：地图同步与 RViz 精简

- OctoMap 的 D435i RANSAC 地面拟合已关闭；该拟合在看不到地面时反复输出
  `No ground plane found in scan`，却不参与覆盖规划。二进制 OctoMap 仍保留高度显示裁剪。
- 删除内部 `map_vis` 调试发布器、参数和 RViz 项；删除两张 `/projected_map` 地面栅格。
  UAV1、UAV2 均只显示各自的 `octomap_binary`。
- chunk 未达到 25% 前也发送实际改变的体素增量；25/50/75/100% 的完整快照和缺 revision
  请求机制保持不变。这样 UAV2 扫过小块区域后，UAV1 融合地图能在下一次增量同步和前沿
  AABB 更新后清除对应前沿，而不会等到该 chunk 达到 25%。
- 竞价仍以 A* 测地路径长度为主体；信息增益权重由 0.05 降为 launch 参数 0.005，防止
  数百个可见体素的增益压过实际绕障飞行距离，从而让两机无必要地汇聚到同一片远处区域。

### 2026-07-19：OctoMap 地面占据体素输出过滤

- 不修改用户保存的 RViz 配置。`occupancy_min_z` 只影响 Marker/2D 投影，不会过滤
  `octomap_binary`/`octomap_full` 的原始序列化树；两个 `octomap_server` 因此改用原生
  `pointcloud_min_z=0.20`。该筛选发生在点云转换至 `world` 后、插入 OctoMap 前，地板点
  不会进入两种地图消息；墙体从 0.20 m 以上仍按 Z 轴渐变显示。
- `points_octomap` 的虚拟 FOV clear-point 端点仅在其距离**严格大于**
  `sensor_model/max_range=3.0 m` 时启用，当前为 `3.15 m`。Noetic `octomap_server` 会将
  超量程点截断到 3.0 m 并只写入沿途和截断端点的 free/miss 证据，不把虚拟端点写为
  occupied；这为无有效回波的视线提供清空证据，能消除移动对机离开后残留的 OctoMap
  幽灵障碍物。
- 真实深度回波及其相邻角度桶会阻挡虚拟射线，避免穿透墙体。虚拟点只进入
  `points_octomap`；覆盖搜索继续只接收真实 `points_downsampled`，因此不会把虚拟端点
  误作为规划地图的障碍物。

### 2026-07-19：地图同步审计（待实现可靠补齐）

- 当前 bridge 的状态通道可双向工作，但 `map_chunk` 仅在本机体素变化时发布；PUB/SUB 若
  丢失启动时的首个 chunk，且无人机随后悬停，对端不会再见到 revision 跳变，也就不会请求
  快照。因此当前实现不能宣称地图已经可靠融合。
- 正确流程应为：每秒广播一个小型 chunk revision 摘要；接收端比较摘要，落后即请求该
  chunk；发送端以 snapshot 响应；正常扫描期间仍以 0.5 秒限频发送变化体素 Log-Odds delta。
  摘要/请求重复直到 revision 对齐，不能依赖一次 PUB/SUB 成功。
- 新地图到达后先更新本机融合图并对当前任务做安全/路径重规划；不应每次体素变化都重新
  拍卖。仅在任务完成、目标已无信息价值/变为不可达、路径代价发生显著变化、或另一机空闲时
  才开新一轮任务分配；已承诺任务保持不变。

```bash
cd ~/Prometheus
source /opt/ros/noetic/setup.bash
catkin_make --source Modules/two_uav_coverage_search --build build/two_uav_coverage_search
source devel/setup.bash
```

实机每台机各自运行 `roscore` 和原有定位/控制后，先运行
`start_swarm_bridge.sh <本机IP> <对机IP>`，再运行
`start_two_uav_coverage.sh <本机UAV_ID> <对机UAV_ID> [高度]`。低 ID 机的 chrony
使用 `configure_chrony_peer.sh master`，高 ID 机使用
`configure_chrony_peer.sh client <低ID机IP>`；两端均须用 `check_clock_offset.sh 20`
通过后才进入任务前检查。

### 2026-07-22：拍卖 V2 方案 — Top-3 视点、纯成本竞价、增量分配

用户（wjy）与 Codex 经过对现有代码的逐行审查，确定以下改进方案。原有 V1
行为通过 `auction_mode` 参数保留，默认仍为 `"v1"` 以保持向后兼容。

#### 需求来源

1. 任务应是前沿簇，拍卖时应考虑该簇的多个候选视点而非仅最优一个。
2. 纯路径执行代价拍卖：不考虑信息增益收益项，谁飞得近（A* 测地距离短）谁拿任务。
3. 增量分配：已分配且未超时的任务不再重复进入拍卖，只有新出现的前沿才参与。
4. 确定性共识：两台机对同一 task 计算出相同 task_id，交换代价后按确定规则判定
   赢家，不依赖 frame_id 重命名。

#### 消息改动（向后兼容）

**SwarmFrontier.msg** — 增加 top-3 视点字段（追加在末尾，ROS 消息向后兼容）：

```
uint64 task_id
uint32 chunk_id
geometry_msgs/Pose viewpoint        # 最佳视点（V1 兼容）
float32 information_gain
# === V2 新增 ===
geometry_msgs/Pose viewpoint_2      # 第 2 视点
geometry_msgs/Pose viewpoint_3      # 第 3 视点
float32 yaw_2
float32 yaw_3
uint8 num_viewpoints                # 实际视点数 (1-3)
```

V1 订阅者忽略新字段，V2 订阅者读取 `num_viewpoints` 确定有效视点数。

**SwarmBid.msg** — 不变。`cost` 字段在 V2 下含义变为
   `min(path_len_i) / max_vel`（三个视点中 A* 路径最短的时间代价），
   不含信息增益项。

**SwarmTask.msg** — 增加增量分配状态：

```
uint64 task_id
uint32 task_version
uint32 winner_uav_id
float32 cost
geometry_msgs/Pose goal
# === V2 新增 ===
bool   incremental_assigned        # 已被分配过
time   assigned_at                 # 分配时刻
```

#### 新增参数

全部通过 launch/YAML 可配，默认保持 V1 行为：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `coverage_search/auction_mode` | `"v1"` | `"v1"` 原始 / `"v2"` 新拍卖 |
| `coverage_search/top_viewpoints_count` | `3` | 每个前沿簇共享的视点数 |
| `coverage_search/incremental_task_timeout` | `30.0` | 已分配任务超时后复位为未分配（s） |

#### 改动点 1：publishSwarmFrontiers() — 发布 top-N 视点

文件：[src/two_uav_coverage_search_manager.cpp](src/two_uav_coverage_search_manager.cpp)

- V2 模式：对 `frontier_finder_.frontiers_[i]` 的 `viewpoints` / `viewpoint_yaws`
  取 min(N, viewpoints.size()) 个，填充到 SwarmFrontier 的新增字段
- V1 模式：仅填充 `viewpoint`（保持原行为）

`FrontierCluster::viewpoints` 已在 `sampleViewpoints()` 中按可见性分数降序排列，
无需额外排序。

#### 改动点 2：publishSwarmBids() — 纯成本，取 min of N 个 A* 代价

文件：[src/two_uav_coverage_search_manager.cpp](src/two_uav_coverage_search_manager.cpp)

V2 模式：
```cpp
double best_cost = INFINITY;
for (int v = 0; v < frontier.num_viewpoints; ++v) {
    Vector3d vp = getViewpoint(frontier, v);
    double path_len = (vp - uav_pos).norm();
    bool reachable = astar.isPathTraversable(uav_pos, vp, &path_len);
    if (!reachable) { /* try full A* up to swarm_bid_max_astar_ times */ }
    if (reachable) best_cost = min(best_cost, path_len / max_vel);
}
bid.cost = best_cost;        // 纯路径时间，不含 info_gain
bid.reachable = isfinite(best_cost);
```

V1 模式：保持原 `path_len/max_vel - 0.005*info_gain` 计算。

#### 改动点 3：runAuction() — 增量分配 + 超时复位

文件：[src/two_uav_coordinator.cpp](src/two_uav_coordinator.cpp)

新增成员：
```cpp
struct TaskAlloc {
    uint64_t task_id;
    ros::Time assigned_at;
    uint32_t winner;
};
std::map<uint64_t, TaskAlloc> task_allocations_;  // 全局已分配任务表
```

V2 模式竞价流程：
```
1. 超时清理：遍历 task_allocations_，超过 incremental_task_timeout 且
   winner 已失联/状态过期 → 移除（该 task 重新进入候选池）

2. 候选过滤：
   - 已在 task_allocations_ 中且未超时 → 跳过（增量分配）
   - 已被任一方到达（< task_reach_dist_） → 跳过
   - 双方都不可达 → 跳过

3. 赢家判定（同 V1）：
   - 对每个候选 task，比较 self_cost vs peer_cost
   - self_cost < peer_cost - 1e-3 → self wins
   - 相等 → 低 UAV ID 赢

4. 新分配的任务写入 task_allocations_[task_id] = {now, winner_id}

5. 发布 SwarmTaskArray（显式共识，防止数据不对齐时的脑裂）
```

V1 模式：保持原 `rejected_until_` + `task_commit_timeout` + `task_retry_cooldown` 机制。

#### 改动点 4：FrontierFinder 接口不变

`FrontierCluster::viewpoints` 和 `viewpoint_yaws` 已是按 `visib_nums` 降序排列的
数组。V2 仅改变消费端（publishSwarmFrontiers 取前 N 个），生产端无需修改。

#### 不变的部分

- `task_id` 编码公式不变：`(chunk_id << 40) | (x << 20) | y`，确定性，跨机一致
- `SwarmTaskArray` 显式交换保留，作为共识安全冗余
- deploy/launch/scripts 不变
- OctoMap / 地图同步 / bridge 不变
- 低 UAV ID 打破平局规则不变

#### 实施顺序

1. 修改 `SwarmFrontier.msg`、`SwarmTask.msg`，重新编译消息
2. 在 `two_uav_coverage_search.h` 中声明 V2 相关参数和辅助方法
3. 修改 `publishSwarmFrontiers()` → V2 填充 top-N 视点
4. 修改 `publishSwarmBids()` → V2 纯成本 + min-of-N
5. 修改 `runAuction()` → V2 增量分配 + 超时复位
6. 更新 `two_uav_coverage_search.yaml` 和 launch 文件参数
7. 编译验证

### 2026-07-20：前沿簇发布者、融合地图前沿与 AABB 增量更新机制分析

用户（wjy）询问 RViz 中显示的融合地图前沿簇由哪个无人机、哪个话题、什么频率发布，
以及前沿簇的本地更新是否为 AABB 轴对称包围盒下的增量更新。

#### 前沿簇可视化发布者

- **发布者**：仅 UAV1。默认 RViz 配置中 UAV1 的 `frontier_vis` 启用，UAV2 的同类
  显示标记为“调试”且 `Enabled: false`。
- **话题**：`/uav1/prometheus/coverage_search/frontier_vis`
- **消息类型**：`visualization_msgs::MarkerArray`，包含三个命名空间：
  - `frontier_cells` — 前沿体素（按簇着色，HSV 色相）
  - `frontier_best_viewpoint` — 每簇最佳视点（球体标记）
  - `frontier_best_view_yaw` — 最佳视点推荐偏航角（箭头标记）
- **频率**：约 **0.83 Hz**（周期 1.2 s），由 `frontier_timer_` 驱动
  （[two_uav_coverage_search_manager.cpp:138](src/two_uav_coverage_search_manager.cpp#L138)）。
- 两架无人机各自独立运行 `FrontierFinder`，分别在自己的融合地图上搜索前沿。
  UAV2 的前沿可视化话题 `/uav2/prometheus/coverage_search/frontier_vis` 同样存在并
  发布数据，但 RViz 默认不显示。

#### 协同模式下的 swarm 前沿交换频率

| 话题 | 周期 | 频率 |
|------|------|------|
| `frontier_vis`（RViz MarkerArray） | 1.2 s | ~0.83 Hz |
| `swarm_frontiers`（本机，供协调器出价） | `swarm_chunk_delta_period_`（默认 0.5 s） | 2 Hz |
| bridge 转发 relay | `data_hz`（默认 5 Hz） | 5 Hz |
| 协调器竞价 | 0.05 s | 20 Hz |

#### UAV1 显示的融合地图前沿是否包含 UAV2 的数据

**是的，包含。** 在协同模式下，UAV1 的前沿簇基于本地融合地图搜索，该地图已融合了
UAV2 的观测数据。流程：

1. UAV2 通过 ZMQ bridge 发送地图 chunk 到 `/two_uav/rx/map_chunk`
2. UAV1 的 `remoteChunkCb`（[two_uav_coverage_search_manager.cpp:527](src/two_uav_coverage_search_manager.cpp#L527)）
   调用 `coverage_map_.setRemoteEvidence()` 将 UAV2 的 Log-Odds 证据写入 UAV1 本地
   地图的远端证据层
3. 远端证据导致占据状态变化时，触发 `markUpdatedIndex` 并置位
   `remote_frontier_refresh_pending_ = true`
4. 下一次 `updateFrontiers()` 调用触发 `refreshFrontiersFull()`（全量重扫描），在
   **已融合的地图**上重新搜索前沿
5. 全量刷新受 `remote_full_frontier_period_`（默认 3.0 s）限频，避免每收到一个
   chunk 就重扫

因此 `/uav1/prometheus/coverage_search/frontier_vis` 上显示的前沿簇，是在融合了两架
无人机观测数据的同一张地图上搜索出来的，自然包含了 UAV2 发现的前沿。两机独立维护
各自的融合地图，不是删除 UAV2 的数据。

#### AABB 增量更新机制

**是的，前沿簇本地更新是严格的 AABB 轴对称包围盒增量更新。** 机制分两阶段：

**累积阶段** — 每次体素占据状态变化时，`markUpdatedIndex(idx, margin=1)`
（[two_uav_coverage_map.cpp:557](src/two_uav_coverage_map.cpp#L557)）不断扩张累积 AABB：

```cpp
// 首次标记：直接用当前体素初始化
updated_min_idx_ = min_idx;
updated_max_idx_ = max_idx;

// 后续标记：做 union 扩张
updated_min_idx_(k) = std::min(updated_min_idx_(k), min_idx(k));
updated_max_idx_(k) = std::max(updated_max_idx_(k), max_idx(k));
```

只有占据状态**实际改变**时（`occupancy_buffer_[adr]` 值从 unknown→free/occupied 或
反之）才调用 `markUpdatedIndex`；证据数值变化但不改变状态分类时不触发。

**消费阶段** — `FrontierFinder::searchFrontiers()` 调用
`map_->getUpdatedBox(update_min, update_max, true)`（`reset=true` 立即清零）：

1. 取出累积 AABB 并加 margin（`2*resolution` 与 `esdf_safe_distance_+resolution` 的
   较大值），得到搜索范围
2. **保留**不与搜索范围重叠的旧前沿簇（直接放入 `kept` 列表，不做任何重算）
3. **移除**与搜索范围重叠的旧前沿簇，并将其 AABB 也并入搜索范围（确保整个受影响
   簇都被重新评估）
4. **仅**在搜索范围内做 BFS 膨胀聚类找新前沿
5. 新前沿做 PCA 分裂（`splitLargeFrontiers`）和视点采样（`computeViewpoints`）

日志输出直接印证（[two_uav_coverage_search_manager.cpp:306](src/two_uav_coverage_search_manager.cpp#L306)）：
```cpp
ROS_WARN_THROTTLE(2.0, "[CoverageSearch] frontier %s update took %.1f ms, frontiers=%d",
                  do_remote_full_refresh ? "remote-full" : "local-AABB", ...);
```
- `"local-AABB"` = 增量路径（本机深度更新，仅扫描 AABB）
- `"remote-full"` = 全量路径（收到远端 chunk 后触发，受 3.0 s 限频）

#### 关键代码路径

- `markUpdatedIndex`：[two_uav_coverage_map.cpp:557-576](src/two_uav_coverage_map.cpp#L557)
- `getUpdatedBox`：[two_uav_coverage_map.cpp:601-616](src/two_uav_coverage_map.cpp#L601)
- `searchFrontiers`（增量）：[two_uav_frontier_finder.cpp:120-185](src/two_uav_frontier_finder.cpp#L120)
- `searchFrontiersFull`（全量）：[two_uav_frontier_finder.cpp:74-106](src/two_uav_frontier_finder.cpp#L74)
- `refreshFrontiersFull`：[two_uav_frontier_finder.cpp:115-118](src/two_uav_frontier_finder.cpp#L115)
- `updateFrontiers`（调度逻辑）：[two_uav_coverage_search_manager.cpp:279-310](src/two_uav_coverage_search_manager.cpp#L279)
- `remoteChunkCb`（远端地图融合）：[two_uav_coverage_search_manager.cpp:527-555](src/two_uav_coverage_search_manager.cpp#L527)

### 2026-07-20：RViz 无人机 ID 标签实现

用户（wjy）要求在 RViz 中显示跟随无人机移动的 ID 标签，以便时刻区分两架飞机。

#### 实现方案

在 `CoverageSearchManager` 中新增 `TEXT_VIEW_FACING` 类型的 Marker 发布：

- **话题**：`<uav_name>/prometheus/coverage_search/uav_label`
  （即 `/uav1/prometheus/coverage_search/uav_label` 和
  `/uav2/prometheus/coverage_search/uav_label`）
- **消息类型**：`visualization_msgs::Marker`（`TEXT_VIEW_FACING`）
- **更新频率**：10 Hz（在 `publishVisualization()` 中随 `mainloopCb` 发布）
- **显示效果**：绿色文字（`#00FF00`），位于无人机实际位置上方 0.6 m，字号 0.45 m，
  文字始终面向观察者
- **标签内容**：`uav_name_`（即 `/uav1` 或 `/uav2`）
- **生命周期**：0.5 s（持续刷新，确保断连后自动消失）

#### 修改的文件

1. [include/two_uav_coverage_search.h](include/two_uav_coverage_search.h#L386) —
   新增 `uav_label_pub_` 发布器成员
2. [src/two_uav_coverage_search_manager.cpp](src/two_uav_coverage_search_manager.cpp) —
   `init()` 初始化发布器；`publishVisualization()` 发布文字 Marker
3. [rviz/two_uav_coverage_search.rviz](rviz/two_uav_coverage_search.rviz) —
   新增 `UAV1 标签` 和 `UAV2 标签` 两个 Marker Display，均默认启用

### 2026-07-20：移除冗余的 remote-full 全量前沿扫描路径

用户（wjy）质疑 `remote-full` 全量扫描的必要性，经代码审查确认该路径是冗余的，
已删除。

#### 冗余分析

远端 chunk 到达时的调用链：

```
remoteChunkCb
  → setRemoteEvidence(address, evidence)
    → remote_evidence_buffer_[address] = evidence
    → updateOccupancyFromEvidence(idx)
      → fused_evidence = local + remote
      → if (next != occupancy_buffer_[adr])
          markUpdatedIndex(idx, margin=1)   // ← 远端变化已加入 AABB
```

关键发现：远端证据导致的占据状态变化，**已经通过 `markUpdatedIndex` 把对应体素指
标加入了累积 AABB**。下一次 `searchFrontiers()` 调用 `getUpdatedBox(reset=true)` 时，
自然会包含这些远端变化区域。

`searchFrontiers()` 的 BFS 膨胀（`expandFrontier`）不受 AABB 边界限制 — 只要种子
在 AABB 范围内，BFS 会沿 26 邻域一直膨胀到整个前沿簇的边界。配合 `markUpdatedIndex`
的 margin=1 和搜索侧的额外 margin（`max(2*resolution, esdf_safe_distance+resolution)`），
远端变化不可能漏掉。

因此 `refreshFrontiersFull()` 触发的全地图扫描对该场景没有任何额外效果 — 它只是把
已经由 AABB 增量覆盖的区域重新扫描了一遍。

#### 已删除的代码

1. **头文件** (`two_uav_coverage_search.h`)：
   - 删除 `refreshFrontiersFull()` 声明
   - 删除 `remote_frontier_refresh_pending_`、`last_remote_frontier_full_refresh_` 成员
   - 删除 `remote_full_frontier_period_` 参数成员

2. **覆盖管理器** (`two_uav_coverage_search_manager.cpp`)：
   - `updateFrontiers()` 简化为始终调用 `searchFrontiers()`（AABB 增量），
     移除 `do_remote_full_refresh` 条件分支和限频逻辑
   - 删除 `nh.param("coverage_search/remote_full_frontier_period", ...)` 参数读取
   - `remoteChunkCb` 中删除 `remote_frontier_refresh_pending_ = ...` 赋值

3. **前沿查找器** (`two_uav_frontier_finder.cpp`)：
   - 删除 `refreshFrontiersFull()` 实现（仅 3 行包装函数）
   - `searchFrontiersFull()` 保留，供 `searchResidualFrontiers()` 使用

4. **launch 文件** (`two_uav_coverage_search.launch`)：
   - 删除 `<param name="coverage_search/remote_full_frontier_period" value="3.0"/>`

#### 不变的部分

- `searchResidualFrontiers()` 继续使用 `searchFrontiersFull(cur_pos, true)` —
  这是长时间无前沿时的宽松兜底重扫，与 remote-full 无关
- AABB 增量机制的累积/消费逻辑完全不变
- `remoteChunkCb` → `setRemoteEvidence` → `markUpdatedIndex` 的调用链完全不变

### 2026-07-20：三个待解决问题分析（仅分析，不做代码修改）

用户（wjy）报告了三个仿真中观察到的问题，以下逐项分析。

#### 问题 1：UAV2 前沿簇显示未及时更新 — UAV1 已探索区域的前沿在 UAV2 上仍然可见

**现象**：UAV1 通过拍卖竞价拿到了原本属于 UAV2 的前沿簇任务并完成探索，但
UAV2 的 RViz 前沿簇显示中，该区域的前沿簇仍然存在。

**分析**：

远端地图同步的完整链路是：

```
UAV1 探索 → 本机体素状态变化 → publishSwarmMapDelta() 发送 chunk 增量
  → bridge 转发 → UAV2 remoteChunkCb → setRemoteEvidence
  → updateOccupancyFromEvidence → 若 fused 状态改变 → markUpdatedIndex
  → 下一次 frontierCb (1.2s) → searchFrontiers() → 移除/更新受影响前沿簇
```

链路本身逻辑是正确的，但存在以下可能导致延迟或遗漏的因素：

1. **chunk 同步延迟**：delta 周期为 `swarm_chunk_delta_period_` = 0.5 s，加上
   bridge ZMQ 转发延迟和 TCP 传输，实际端到端延迟可能达到 1-2 秒。

2. **AABB 增量可能遗漏**：`searchFrontiers()` 通过 `getUpdatedBox` 获取变化体素
   的 AABB。如果远端证据导致前沿簇内部的 free 体素附近出现了新的 free 体素
   （原本 unknown），变化体素在 AABB 内，但现有前沿簇的 `isFrontierChanged()`
   检查的是"旧前沿簇的 cells 是否仍是前沿体素"——如果旧前沿簇的每个 cell 恰好
   仍是前沿（因为新 free 体素旁边还有其他 unknown 体素），则该簇会被直接保留
   （`kept.push_back(ftr)`），不会重新 BFS 聚类。这意味着旧前沿簇的边界可能
   不会收缩，即使其中部分区域已被探索。

3. **关键场景**：远端证据标记了旧前沿簇附近的一些 unknown 体素为 free。这些
   新 free 体素本身变成了前沿（free + 有其他 unknown 邻居），但旧前沿簇的
   原有 cells 仍然是前沿（仍然有各自的其他 unknown 邻居）。此时：
   - `isFrontierChanged` 对旧簇返回 `false`（所有旧 cells 仍是前沿）
   - 旧簇被保留
   - AABB 内的新前沿体素被 BFS 找到，形成新簇
   - 结果：同一区域出现两个重叠的前沿簇 —— 旧簇和新簇各占一部分
   - 表现：UAV2 上该区域的前沿"似乎没有消失"

4. **根本原因**：增量更新逻辑的设计假设是"前沿簇要么整体消失，要么保持不变"。
   但实际上远端地图更新可能导致前沿簇**部分收缩**——部分区域不再是前沿但
   其他部分仍然是。增量路径无法检测这种"部分收缩"。

5. **为什么之前 remote-full 存在**：原 `refreshFrontiersFull()` 全量扫描正好
   弥补了这个问题——定期全图重搜会清除这种残留。但全量扫描又太重。正确的修复
   方向可能是：对与 AABB 重叠的旧簇，不仅检查 `isFrontierChanged`，而是
   **无条件移除并重搜**（即只要重叠就重搜，不管是否 changed）。这比当前条件
   更激进但能避免残留，且远轻于全量扫描。

**结论**：旧前沿簇的保留条件过于宽松（要求同时满足重叠且 changed 才移除）。
改为"只要与 AABB 重叠就重搜"可以解决此问题，开销可控（通常只有少数簇与
AABB 重叠）。

#### 问题 2：前沿簇缺少任务归属标签

**需求**：在 RViz 中每个前沿簇旁边显示它是分配给 UAV1 还是 UAV2 的任务。

**分析**：

当前 RViz 只显示 UAV1 融合地图的前沿簇（`/uav1/prometheus/coverage_search/frontier_vis`），
不显示任务归属信息。任务分配由协调器的拍卖逻辑完成，分配结果通过外部目标模式
(`external_goal_only`) 下发给覆盖节点。

要实现此需求，可能的方案：

1. **在协调器中新增发布**：协调器维护任务分配表，为每个已分配任务发布一个
   `TEXT_VIEW_FACING` Marker，文字为 `UAV1` 或 `UAV2`，位置为任务视点上方。
   话题：`/two_uav/task_labels`（或类似）。

2. **在覆盖节点中扩展前沿发布**：`publishFrontiers()` 知道本机前沿，但不知道
   远端 UAV 的任务归属。需要协调器下发任务归属信息。

3. **方案 1 最简单**：协调器已经有任务分配信息（`task_allocations_` 或等价
   结构），直接从协调器发布归属标签。RViz 新增一个 Marker Display 订阅即可。

**注意**：此问题不做代码修改，仅记录分析供后续实施参考。

#### 问题 3：虚假占据体素 — 无人机相互观测导致对机被写入静态障碍地图

**现象**：
- RViz 中出现了 Gazebo 世界不存在的"障碍物体素"
- 无人机在这些虚假障碍周围徘徊
- 前沿簇被嵌入到虚假障碍中
- 持续观测未能清除这些占据体素

**分析**：

**3a. 根因确认：D435i 深度相机观测到了对机**

P450 是物理 3D 模型，在 Gazebo 中 D435i 深度相机发射的红外光线打到对机机身
上会反射，产生真实的深度点。这些点进入 `updateFromDepthPcl`：

```cpp
// 命中点 → integrateHit → occupancy_evidence_buffer += 1~2
// 射线路径 → integrateMiss → occupancy_evidence_buffer -= 1
```

对机机身上的点被当作"命中"（hit），证据 +1~+2。如果对机悬停在某位置，每帧
深度点云都会产生新的命中，证据迅速累积到上限 +4，体素变为 occupied。

**3b. 为什么持续观测不能清除**

`clearRobotVolume` 只清除**本机**机体体积内的体素（[two_uav_coverage_search_manager.cpp:577](src/two_uav_coverage_search_manager.cpp#L577)）：

```cpp
coverage_map_.clearRobotVolume(Eigen::Vector3d(
    uav_state_.position[0], uav_state_.position[1], uav_state_.position[2]));
```

它**不清除对机**的体积。因此对机占据的体素只能通过正常的射线投射（miss/free
射线穿过对机原来位置）来逐渐衰减证据。但问题是：

1. 对机本身是不透明的，深度相机**看不到对机背后的体素**。对机背后的区域被
   遮挡，永远不会收到 free 射线。
2. 当对机移动后，原来位置的体素可以被后续的深度射线穿过了，此时 `integrateMiss`
   会逐步减少证据。但这个衰减需要多次观测（每次 -1，从 +4 降到 <= -1 需要至少
   5 帧）。
3. 如果对机在附近徘徊，不断产生新的命中，新旧占据交替出现，地图中始终有
   对机体积的占据体素。

**3c. 为什么前沿簇嵌入虚假障碍**

前沿检测依赖 `isFrontierCell`，它要求体素是 free 且有 unknown 邻居。如果对机
产生了一片 occupied 体素，那么：
- 障碍后面的区域变为 unknown（被遮挡）
- 障碍边缘的 free 体素紧邻 unknown → 形成前沿
- 前沿簇就在虚假障碍周围形成

A* 规划器避开这些 occupied 体素，导致无人机路径绕行、徘徊。

**3d. 解决方案方向**（仅分析，不做修改）

1. **对机点云过滤**：在深度点云进入 `updateFromDepthPcl` 之前，根据对机位姿
   （bridge 交换得到），过滤掉对机机体包围盒内的点。这是最直接的修复。
2. **扩展 `clearRobotVolume`**：同时清除本机和对机机体体积内的体素。这只能
   清除当前对机位置的体素，不能清除历史积累。
3. **对机占用动态层**：将对机作为一个独立的动态障碍层，不写入静态 Log-Odds
   地图。MID360/D435i 看到的对机点永远不计入 `occupancy_evidence_buffer_`。
4. **联合方案**：方案 1（点云过滤）+ 方案 3（动态障碍层）是最完整的解法。
   方案 1 防止新假点进入，方案 3 清理历史假点。

**结论**：这是多机器人系统中经典的自/对机感知干扰问题。短期的仿真 workaround
包括增大两机初始距离、让两机在不同区域探索以减少相互观测。真正的修复需要
点云过滤或动态障碍层。

#### 同步修正：2026-07-20 前沿 AABB 更新日志措辞

上一条记录中 `updateFrontiers()` 改为始终调用 `searchFrontiers()` 后，日志文本
由 `"remote-full"/"local-AABB"` 改为 `"AABB update"`。此修改在
[src/two_uav_coverage_search_manager.cpp](src/two_uav_coverage_search_manager.cpp)
中已生效。

### 2026-07-20：前沿簇跨机不一致问题深度分析（根因分析与最小修复）

用户（wjy）报告了一个更精确的问题场景：
- A 区域 = UAV1 探索区域，B 区域 = UAV2 探索区域
- UAV1 显示 A 区域的前沿簇：正确
- UAV2 显示 B 区域的前沿簇：正确
- UAV1 显示 B 区域的前沿簇（通过 chunk 同步融合）：**与 UAV2 本地显示不一致**

#### 根因分析

这个问题的根因与 [问题 1](#问题-1uav2-前沿簇显示未及时更新) 完全相同，是 AABB 增量
更新的保留条件过于保守。

**具体过程：**

1. UAV2 在 B 区域探索，产生前沿簇 F_B（包含一组 frontier cells）
2. UAV2 通过 chunk 同步将 B 区域的 Log-Odds 证据发送给 UAV1
3. UAV1 接收并融合：`remote_evidence_buffer_` 更新 → `updateOccupancyFromEvidence`
   → 若 fused 状态变化 → `markUpdatedIndex` → 累积 AABB
4. UAV1 的下一次 `searchFrontiers()` 取出 AABB，检查 UAV1 上 B 区域已有的旧前沿簇
5. **关键判断**：旧前沿簇的 AABB 是否与更新 AABB 重叠？重叠的前沿 cell 是否不再是
   前沿（`isFrontierChanged`）？
6. 如果旧前沿簇的每个 cell 碰巧仍然是前沿（因为旁边还有 unknown 体素），
   `isFrontierChanged` 返回 false → 旧簇被**直接保留**
7. AABB 内的新前沿体素被 BFS 找到，形成新簇
8. 结果：UAV1 上 B 区域的旧前沿簇和新前沿簇并存，边界重叠、形状不一致

这不是远端 chunk 特有的算法分支：本地与远端体素最终都经
`updateOccupancyFromEvidence()` → `markUpdatedIndex()` → `searchFrontiers()`。UAV2 本地
未复现，只是它的连续相机更新更常让旧簇中至少一个 cell 直接失去 frontier 属性，
从而偶然通过旧的 `isFrontierChanged()` 门槛；远端批量更新更容易触发“簇拓扑变了、
但旧 cell 仍全部是 frontier”的反例。换言之，根因是**缓存簇的拓扑失效判断错误**，
不是 chunk 没有发送某类边界 free cell，也不是 UAV1/UAV2 使用了不同的前沿算法。

#### 为什么"前沿簇标志位"方案不合理

用户提出：给每个栅格加一个 `is_frontier` 标志位，传输时携带。

**架构上不合理的原因：**

1. **前沿是导出量，不是原始数据**。前沿由"free cell + 相邻 unknown cell"这个
   拓扑关系决定。如果占据状态正确同步，前沿一定可以通过本地重算得到。直接传输
   前沿标志相当于在"传输正确占据"之外引入了第二条可能不一致的数据通道。

2. **标志位会过期**。UAV2 标记某 cell 为前沿的时刻，到 UAV1 收到这个标志的
   时刻之间，UAV2 可能已经探索了该 cell 旁边区域，该 cell 不再是前沿。标志位
   需要"撤销"机制 → 复杂度远超当前方案。

3. **标志位不能替代前沿检测**。即使收到了标志位，UAV1 仍然需要在自己的融合
   地图上做前沿检测（因为 UAV1 也需要知道本机区域的前沿）。两条路径两次计算
   → 不如把一条路径修好。

4. **真正的修复只需一行条件改动**（见下方"修复方向"）。

#### chunk 同步协议回顾

用户问到当前的地图传输机制。当前协议如下：

**发送端（每 0.5 s，`publishSwarmMapDelta`）：**

```
for each chunk:
    for each voxel in chunk:
        evidence = getLocalEvidence(address)  // 本机 Log-Odds 证据 [-3, 4]
        if snapshot OR evidence != last_sent[address]:
            加入 delta 消息
            last_sent[address] = evidence
    if chunk.known% 达到 25/50/75/100%:
        发送完整快照（snapshot=true）
```

- 只发送**与本机上次发送相比有变化**的体素
- 每达到 25% 阈值发一次完整快照，用于断链恢复
- 本机证据值**直接覆写**对端的远端证据层（不是增量加减）

**接收端（`remoteChunkCb`）：**

```
收到 chunk 消息:
    if revision 跳号 → 发送 SwarmMapRequest 请求完整快照
    对消息中的每个 (address, evidence):
        setRemoteEvidence(address, evidence)
          → remote_evidence_buffer_[address] = evidence  // 直接覆写
          → updateOccupancyFromEvidence(idx)
            → fused = local_evidence + remote_evidence  // 融合
            → 阈值判定 → 若状态真的改变 → markUpdatedIndex
```

- 远端证据直接覆写，不是加/减操作（避免重传/乱序导致重复计数）
- 每个来源独立维护自己的证据层
- 融合后再阈值化，不依赖单个来源

**这个协议本身是正确的。** 问题不在传输，而在 UAV1 收到正确证据后，前沿增量
更新没有完全反映新的融合地图状态。

#### 修复方向

将 `searchFrontiers()` 中对旧前沿簇的处理从：

```cpp
// 当前：同时要求重叠 AND changed 才移除重搜
if (haveOverlap(ftr.box_min_, ftr.box_max_, search_min, search_max) &&
    isFrontierChanged(ftr)) {
    // remove and re-search
} else {
    kept.push_back(ftr);  // keep unchanged
}
```

改为：

```cpp
// 修复后：只要与 AABB 重叠就无条件移除重搜
if (haveOverlap(ftr.box_min_, ftr.box_max_, search_min, search_max)) {
    // always remove and re-search, regardless of isFrontierChanged
    search_min = search_min.cwiseMin(ftr.box_min_ - margin);
    search_max = search_max.cwiseMax(ftr.box_max_ + margin);
    resetFrontierFlag(ftr);
} else {
    kept.push_back(ftr);
}
```

这样做的好处：
- 任何与更新区域有重叠的前沿簇都会被完整重搜
- BFS 会从 AABB 内的种子出发，膨胀到正确的簇边界
- 不重叠的簇完全跳过，不影响性能
- 开销：通常只有少数簇与 AABB 重叠（更新区域有限），额外开销可忽略
- 不需要增加任何消息字段、标志位或传输带宽

此修复同时解决了"问题 1（前沿残留）"和"前沿跨机不一致"两个问题。

#### 复核结论与验收边界（2026-07-20）

已逐段复核发送、bridge、接收、地图融合、AABB 消费和 RViz 发布路径，结论如下：

```
UAV2 local evidence
  -> publishSwarmMapDelta()                 # 每 0.5 s
  -> two_uav_swarm_bridge /map_chunk        # 每个 chunk 均转发；源端每 0.5 s 批处理
  -> UAV1 remoteChunkCb()
  -> setRemoteEvidence()                    # 直接覆写远端证据层
  -> updateOccupancyFromEvidence()
  -> markUpdatedIndex()
  -> FrontierFinder::searchFrontiers()      # 每 1.2 s
  -> /uav1/.../frontier_vis
```

- chunk 带的是 UAV2 的本机证据值，不是前沿簇；接收端没有“远端前沿显示”专用缓存或
  第二套聚类逻辑。因此不应增加 `is_frontier` 网络字段。
- 原始 `isFrontierChanged()` 只问“旧簇中是否有 cell 不再是 frontier”，不能检测
  “新 free frontier 与旧簇连通/旧簇需要重分裂”的拓扑改变。旧 `frontier_flag_` 保持为
  true 后，BFS 会跳过旧 cell，产生过期簇或分裂簇。
- 当时采用的最小修复是：只要旧簇 AABB 与更新搜索区相交，就移除该簇、清除其
  `frontier_flag_` 并在扩展后的范围重新 BFS；`isFrontierChanged()` 的声明和定义被删除。
  该路径同时覆盖本地和远端更新。后续发现该策略会使搜索 AABB 沿旧簇连锁扩张并导致秒级耗时，
  已由本文 2026-07-26“前沿更新中的旧簇 AABB 连锁扩张”章节取代。
- 不存在静态地图更新时，两机显示的正常传播上界约为 `0.5 + 1.2 = 1.7 s`（另加少量
  bridge/ROS 调度时间）。在这个窗口内的暂时不一致是设计上的最终一致性，不是本缺陷。
  若最后一次 B 区域变化超过约 2 s 后仍不一致，再检查 bridge 的
  `drop ... message`、revision 跳号和两个 `frontier_vis` 话题是否都已启用。

构建复核：已在现有 package build 目录成功构建 `two_uav_coverage_lib` 与
`two_uav_coordinator`。未添加消息字段、同步线程、全图重扫或新依赖。

### 2026-07-21：块状直线前沿的真正根因——bridge 丢弃同批 map chunk（已修复）

用户提供的 RViz 截图显示：UAV1 的融合地图中，代表 B 区域的前沿呈绿色/红色的直线、
直角和块状边界；UAV2 本机在同一区域显示的紫色/橙色前沿则随真实观测边界弯曲。
这说明问题位于**前沿计算之前的地图证据传输**，而不是前沿簇 AABB 重搜。

> RViz 的颜色仅按本机 `frontiers_` 容器下标生成，并不编码“来自哪架无人机”。这里的
> 绿色/红色应理解为 UAV1 融合图 B 区域中的簇，而非网络传输了绿色/红色前沿簇。

#### 精确故障链

`publishSwarmMapDelta()` 每 0.5 s 依次发布本轮所有发生变化的 2 m × 2 m chunk；地图
分辨率为 0.2 m，因此一个 chunk 是 10 × 10 × 15 个体素。旧 bridge 对整个
`SwarmMapChunk` 通道共用一个 `last_send_`，并套用 `data_hz=5` 的节流：

```cpp
if ((now - last_send_).toSec() < 1.0 / 5.0) return;
```

同一个 0.5 s 定时回调中产生的第一条 chunk 消息通过，后续所有 chunk 都在 0.2 s
窗口内被该 `return` **静默丢弃**。与此同时，发送端在构造 ROS 消息时已经更新了
`swarm_last_sent_evidence_`，以后认为这些体素“已发送”，因而不会重发未真正穿过
bridge 的证据。接收端没收到该 chunk 的任何后续 revision 时也无法发出补快照请求。

这正好解释了图像特征：融合图只拥有部分 2 m 网格块的观测，已知/未知边界被 chunk
边界切断，前沿自然退化成直线、直角和矩形，而不是传感器几何产生的弯曲边界。

#### 已实施的最小修复

在 [src/two_uav_swarm_bridge.cpp](../src/two_uav_swarm_bridge.cpp) 中允许 `max_hz=0` 表示
不节流，并只给 `SwarmMapChunk` 传入 0。状态、轨迹、前沿、出价和请求通道仍保持原来的
`data_hz` / `state_hz` 限速；地图发送仍由 `swarm_chunk_delta_period=0.5 s` 的增量批处理
控制，单包仍受 `max_packet_kib` 限制。没有添加前沿字段、全图重扫或新依赖。

`two_uav_swarm_bridge` 已重新编译通过，且 `git diff --check` 无格式错误。

#### 部署与验收

1. 将新 bridge 二进制部署到**两台**机载计算机，并重启两端 bridge。
2. 为消除旧 bridge 已静默丢失的历史 evidence，首次验证应重启两侧 coverage-search
   节点/重新开始任务；否则要等待每个旧 chunk 再发生一次变化并由 revision 快照恢复。
3. 启用两台飞机的 `frontier_vis` 后，等待超过一次 0.5 s 同步和一次 1.2 s 前沿刷新；
   UAV1 的 B 区域应不再出现按 2 m 规则切割的直线/直角前沿，形状应与 UAV2 的融合图
   在正常时延内一致。

当前协议仍没有逐包 ACK；断链后完全静止的历史 chunk 不能立即自愈。若真实链路中需要
支持这种场景，下一步才增加低频轮询快照/连接后同步请求；这不是本次块状前沿的必要修复。

#### 现场验证（2026-07-21）

用户在重新部署并运行后确认：本修复**直接消除了问题**。UAV1 融合图中的 B 区域前沿不再
呈 chunk 对齐的直线/直角块状，说明根因确为 `SwarmMapChunk` 在 bridge 的全通道 5 Hz
节流下被静默丢弃，而非 AABB 聚类或前沿可视化。

### 2026-07-26：RViz FOV 实时跟随无人机朝向

- 症状：`/uav<ID>/prometheus/state` 实测稳定为 50 Hz，但原先由
  `coverage_search_node` 发布的 `/uav<ID>/prometheus/coverage_search/fov_vis`
  约为 1 Hz。深度建图和规划与状态回调共用该节点的单线程 `ros::spin()`；重计算阻塞
  状态回调，导致 RViz 中的 world-FOV 明显落后于无人机姿态。
- 最终方案：新增独立的 `two_uav_fov_visualizer` 进程。每架机各启动一个实例，直接订阅
  `/uav<ID>/prometheus/state`，以每条状态消息重建并发布原有的
  `/uav<ID>/prometheus/coverage_search/fov_vis` Marker。FOV 保持 `frame_id=world`，
  不采用会被当前 RViz/TF 链路丢弃的 frame-locked Marker，因此原有 RViz 的两个 FOV
  Display 和话题均不变。
- FOV 几何、D435i 安装偏移、俯仰和水平/垂直视场角均与覆盖搜索原实现相同；仅将发布者
  从会被阻塞的覆盖节点移出。`two_uav_coverage_search_manager` 不再向该话题发布，避免
  低频旧 Marker 覆盖实时 Marker。
- `two_uav_coverage_sim_algorithm.launch` 在两套 onboard 覆盖节点后自动启动
  `fov_visualizer_uav1` 和 `fov_visualizer_uav2`。RViz 的两个 FOV Display 订阅队列为 1，
  根配置帧率为 60，旧消息不会积压。编译已验证
  `two_uav_fov_visualizer` 与 `two_uav_coverage_search_node`。

### 2026-07-26：前沿更新中的旧簇 AABB 连锁扩张（已优化）

#### 问题

此前为避免旧前沿残留，双机版采用“旧簇 AABB 与更新搜索区相交即删除”的策略，并在删除时将
`search_min/search_max` 扩展到该簇完整边界及 margin。由于扩展后的搜索区会继续命中更多旧簇，
形成连锁失效：

```text
局部地图更新 AABB
  -> 删除相交簇并扩大搜索 AABB
  -> 命中更多簇并继续扩大
  -> 多簇重新 BFS、切分、视点及遮挡射线计算
```

因此日志中的 `frontier AABB update took 1.8~3 s` 并不只是 AABB 扫描时间；其计时还包含
ESDF、前沿重搜、视点/可见性计算、RViz Marker 发布和层级网格输入。该行为与单机及 FUEL
的增量更新机制不同，会在前沿簇较多时退化为近似大范围重建。

#### 解决方案

`FrontierFinder::searchFrontiers()` 改为 FUEL/单机风格的固定增量范围：

```text
搜索范围：原始 updated AABB + 固定 margin
旧簇删除：仅当“与原始 updated AABB 相交”且“抽样确认簇内前沿已失效”时执行
```

实现复用单机的 `isFrontierChanged()` 轻量抽样判定：最多约 180 个采样点，若失效样本达到
簇大小的 15% 即重建；未变化的簇保留其已有 cells、viewpoints 和 `frontier_flag_`。移除了
按旧簇动态执行 `cwiseMin/cwiseMax` 扩大 `search_min/search_max` 的代码。

区域生长 `expandFrontier()` 从固定搜索区内的新种子出发时仍会遍历整块连通前沿，因此不需要
为重建完整簇而扩大扫描 AABB。

#### 边界与后续观察

此修改解决的是“**旧前沿簇导致搜索 AABB 连锁扩大**”这一主矛盾，不修改地图同步协议。两机各自
的本地地图变化和远端 chunk 融合仍会合并到单一 `updated_bbox`；两机相距较远时，这个原始
AABB 仍可能较大。若实测仍出现超过 500 ms 的更新，下一项应将单一 dirty AABB 拆为多个独立
脏区域，而不是恢复旧簇驱动的动态扩大策略。

本优化已编译验证：`two_uav_coverage_search_node` 构建成功。

### 2026-07-26：远端地图 chunk 不再扩大本机前沿 AABB（已优化）

两机相距较远时，旧实现将本机深度更新和远端 `SwarmMapChunk` 的融合变化都写入同一
`updated_min_idx_/updated_max_idx_`。一次前沿定时更新会把相距很远的两个区域取 min/max
并合成一块大 AABB，可能横跨两机之间整张地图。

现实现保留本机 `updated_min_idx_/updated_max_idx_` 机制不变，并将远端融合变化单独记录为
`remote_updated_boxes_`：

```text
本机占据变化       -> 本机 AABB
远端一个 chunk 融合 -> 该消息实际发生占据变化的 remote box
frontier timer      -> 本机 AABB 与每个 remote box 分别增量搜索
```

远端 evidence 值变化但未改变融合后的占据状态时，不产生前沿/ESDF 更新。相邻或重叠的 remote
box 才会合并；相距较远的 chunk 始终保持独立，因而不会再经由远端地图传输把本机 AABB 拉长。
同一轮中一个旧前沿簇即使与多个区域相交，也只会失效和重建一次；所有区域处理完后再统一计算
新簇视点、发布 RViz Marker。

ESDF 使用独立的 `remote_df_boxes_` 处理远端变化，避免前沿搜索已分区但 ESDF 仍因远端/本机
min/max 合并而退化为大范围更新。前沿定时器周期保持 1.2 s，未在本次修改中调整。

`two_uav_coverage_search_node` 已重新编译通过。

#### 本机更新与两机融合的最终实现流程

本机和远端仍融合到同一个 `occupancy_buffer_`，因此 A*、碰撞检测、前沿判定和视点可见性始终
使用同一份融合地图；本次改变的是“地图变化如何触发前沿/ESDF 增量更新”，不是将地图拆成两份。

```text
本机深度/点云回调
  -> occupancy_evidence_buffer_ 更新
  -> updateOccupancyFromEvidence(idx, true)
  -> markUpdatedIndex(idx, 1) + markDistanceFieldDirtyIndex(idx, 1)
  -> updated_min_idx_/updated_max_idx_：保持原有单一 local AABB

对方 UAV 每 0.5 s 发送发生变化的 SwarmMapChunk
  -> remoteChunkCb()
  -> remote_evidence_buffer_ 更新
  -> updateOccupancyFromEvidence(idx, false)
  -> 仅当融合后的 occupancy_buffer_ 状态实际改变时，累计该消息的 changed_min/max
  -> markRemoteUpdatedBox(changed_min, changed_max)
  -> remote_updated_boxes_ + remote_df_boxes_：不写入 local AABB

frontier timer（保持 1.2 s）
  -> updateDistanceFields()：本机 df AABB 与各 remote_df_box 分别局部更新
  -> searchFrontiers()：消费一个 local AABB 和多个 remote_updated_box
  -> 仅把重叠/相邻区域合并；远距离区域独立扫描
  -> 一轮结束后统一重建新簇视点并发布 Marker
```

`updateOccupancyFromEvidence()` 由原来的无返回 `void` 改为返回“融合占据状态是否改变”。这避免了
远端 evidence 虽变化、但本机 evidence 抵消后占据状态不变时仍触发无效的前沿与 ESDF 更新。

对每一轮待处理区域，旧前沿簇只要与任一原始区域相交且经 `isFrontierChanged()` 确认失效，就
清除一次 `frontier_flag_` 并重建一次。随后对各区域执行固定 margin 的增量扫描；同一簇即使跨
多个 remote chunk，也不会被重复删除和重算。空间连续的 chunk 可以合并以避免边界重复扫描，
相距较远的本机/远端更新不会被 min/max 合成为横跨地图的大包围盒。

### 2026-07-26：前沿更新时间日志未显示（正常阈值行为）

优化后终端不再持续出现如下警告：

```text
[CoverageSearch] frontier AABB update took ... ms
```

这不是前沿更新停止，也不是日志被删除。`CoverageSearchManager::updateFrontiers()` 仍在每次
前沿定时器触发时执行 ESDF、前沿簇增量更新、Marker 发布和层级网格输入；只是该日志只在总耗时
超过 120 ms 时输出，并使用 `ROS_WARN_THROTTLE(2.0)` 限制为同类警告最多每 2 s 一条。

因此没有该日志通常意味着：本轮更新低于 120 ms、处于 2 s 节流窗口内，或没有新的本机/远端
地图脏区域而前沿搜索快速返回。该阈值与节流保持不变，用于避免正常快速更新刷屏。

### 2026-07-26：双机互相观测产生的假占据体素（内部地图与 RViz OctoMap 同步消除）

#### 问题

两架无人机进入彼此 D435i 视场时，对机机身会成为真实的深度回波。若直接按静态环境处理，
该回波不断累积为 occupied 体素；对机悬停或相互观察时，普通 miss 射线无法穿透机身，因而
假占据与其遮挡边缘的假前沿簇不会自行稳定消失。

此前已在覆盖规划内部的 `CoverageMap` 中加入动态对机排除：接收 `/uav<ID>/two_uav/rx/state`
中的对机位置，以半径 `0.40 m`、半高 `0.30 m` 的圆柱体屏蔽新 hit，周期性清除当前及上一
位置的历史占据，并让 `isFrontierCell()` 拒绝该体积内的前沿体素。这保证 A*、前沿和视点选择
不会把对机当作静态障碍。

但 RViz 的占据显示来自独立的 `octomap_server`，其输入是
`/uav<ID>/camera/depth/color/points_octomap`，不读取 `CoverageMap`。因此内部规划已正确时，
RViz 仍可能显示对机造成的假占据，形成两张地图不一致。

#### 最小完整修复

在 `two_uav_depth_cloud_downsample_node` 的 **OctoMap 输出支路**加入同一动态对机圆柱过滤：

```text
真实深度点云
  -> 下采样
  -> points_downsampled（不改，继续供 CoverageMap 使用）
  -> 以本机 UAVState 外参将点变换到 world
  -> 对方 SwarmState 新鲜且点落入对机圆柱：剔除
  -> points_octomap + FOV free-only clear points
  -> octomap_server -> RViz
```

- 本机位姿使用 `/uav<ID>/prometheus/state`；对机位姿使用 bridge 转发的
  `/uav<ID>/two_uav/rx/state`。点坐标变换复用覆盖搜索的 D435i 外参：安装偏移
  `(0.095, 0, 0) m`、俯仰 `0.35 rad` 与 optical-to-body 旋转，保证两处判定的空间范围一致。
- 仅过滤 `points_octomap`，不改变 `points_downsampled`。规划内部仍由原有动态排除层统一处理，
  从而避免在两个地图分支重复耦合。
- 被剔除的对机回波不再占用其 FOV 角度桶；原有 `3.15 m` 的虚拟 clear-point 会向该方向发送
  free-only 射线。OctoMap 后续帧因此会清除历史的对机假占据，而不会把虚拟端点写为 occupied。
- 对方状态超过 `0.60 s` 未刷新，或本机里程计无效时，OctoMap 过滤自动停用；此时保留全部真实
  回波，避免通信异常下误删静态障碍。

相关参数在 `two_uav_coverage_sim_algorithm.launch` 的两个 `depth_downsample_uav*` 节点中设置：
`peer_clear_radius=0.40`、`peer_clear_half_height=0.30`、`peer_state_timeout=0.60`。节点会节流输出
`suppressed ... peer-body points from OctoMap input`，可作为运行时确认。`two_uav_depth_cloud_downsample_node`
已编译通过，用户实测确认本修复有效。

### 2026-07-26：前沿簇区域租约、执行进展续租与近距仿真解除悬停

#### 问题背景

双机从“协调器直接下发最终视点”改为“协调器分配前沿簇任务、本机自行选择最终视点”后，任务的
语义必须与前沿簇而非某一次生成的视点绑定。否则地图融合、AABB 增量更新或视点评分变化会造成：

```text
同一未知边界区域
  -> 最佳视点位置变化
  -> 若 task_id 由视点生成：被误认为新任务
  -> 原 owner 丢失、对机可能重新认领，或远任务被长期预占
```

此前的任务消息仅以“最近一次收到任务消息的时间”近似租约有效期；协调器每 2 s 重发任务，因而
未执行的 bundle 任务会被不断刷新，实际可无限期占用。若简单改为固定 8 s 硬过期，旧 owner 已在
飞行、但新 owner 已重新竞价获胜时，又可能出现两机同时执行同一任务。

另外，协调器原本会在两机距离过近、命令目标进入对机安全半径、或预测轨迹相交时强制发布 hold。
这对正式安全飞行有意义，但在当前租约与选点机制的仿真验证阶段会遮蔽任务分配本身的效果。

#### 任务、区域与消息数据结构

任务仍定义为“一个前沿簇所代表的待观测未知边界区域”，而不是飞行最终目标。一个任务的当前
最佳视点只是该区域在本轮地图上的执行代表：

```text
前沿簇区域任务 T
  -> 当前唯一最佳视点 V(T)
  -> 竞价：A*路径长度(UAV_i -> V(T))，单位 m
  -> 获胜 owner 在本机用当前地图中的 V(T) 规划最终轨迹
```

竞价仍只比较 A* 路径长度；不引入信息增益、速度、姿态或偏航项。信息增益仅保留在**簇内唯一最佳
视点**的可见性评分中，不参与双机任务归属。

为使租约可跨一次普通地图更新保持，扩展了协同消息：

| 消息 | 新字段 | 用途 |
|---|---|---|
| `SwarmFrontier` | `cluster_version`、`frontier_cell_count`、`centroid`、`box_min`、`box_max` | 发布前沿簇区域描述，而不是只发布视点。`cluster_version` 来自簇变化 generation。 |
| `SwarmTask` | 同一组区域字段、`lease_expire_time` | 将拍卖结果明确表示为“哪个 UAV 在何时前拥有哪片前沿区域”。 |
| `SwarmTrajectory` | `active_task_id`、`active_task_distance` | 规划节点向本机协调器报告当前正在执行的任务以及到其目标的二维距离。 |

`task_id` 保留为前沿簇中心的 0.5 m 量化区域键，用于分布式消息去重和确定性平局；它不再由最佳
视点位置构造。为应对中心越过量化边界或局部簇形状变化，本机不会只以 `task_id` 判定同一任务。

#### 更新后前沿簇与租约的匹配

本机每次增量更新前沿后，对新簇和有效租约做以下最小区域匹配：

```text
有效租约任务 T
  ├─ task_id 相同，且簇中心距 <= 1.0 m        -> 匹配
  └─ 或 AABB（带 0.5 m margin）重叠，且中心距 <= 1.0 m
                                                 -> 匹配
```

匹配成功表示仍是同一任务区域：owner 与原租约不变，但本机可使用该簇刚刚重新计算出的唯一最佳
视点。匹配失败则该簇不受旧租约保护，重新作为候选前沿发布和竞价。

这是在现有消息和计算预算内加入的最小区域描述方案。当前未发送低采样 frontier cell 集合或其哈希；
若未来遇到相邻小簇 AABB 重叠造成误匹配，再增加 cell 签名/重叠率作为第三层判据，而不改变本次
租约主流程。

#### 8 秒租约、执行校验与进展续租

`task_lease_duration` 设为 8.0 s。租约的行为区分“已执行任务”和“仅预留 bundle 任务”：

```text
拍卖得到任务
  -> 首次分配：lease_expire_time = now + 8 s

bundle 中但未被本机选为最终目标
  -> 没有 active_task_id / 距离进展
  -> 到 8 s 必定释放，重新进入双方竞价

当前执行任务
  -> CoverageSearch 每 0.5 s 发布 task_id 与距目标距离
  -> 距离相对上次减少 >= 0.05 m：记录“有进展”
  -> 租约临近到期且最近 2.5 s 内有进展：续 8 s
  -> 无进展：不续租，按到期规则重新竞价
```

执行端还在每次 `executeTrajectory()` 前检查：当前 `current_goal_task_id` 是否仍由本机拥有且
`lease_expire_time` 尚未到期。失效、过期或被重新分配时，立即中止当前轨迹、发布当前位置 hold，
再从有效的本机租约前沿簇重新选择目标。这样避免“旧 owner 继续飞行，而新 owner 已接手”的重叠
执行风险。

协调器收到对方未过期任务后不再认领；规划节点同时合并本机/对机任务，冲突时按更低 UAV ID
确定 owner。这是拍卖后的第二层任务一致性保护。

#### 本机最终选点边界

本机候选池只包含“区域匹配有效租约且 owner 为本机”的前沿簇；对机租约簇仍可融合到地图、生成
前沿和显示 Marker，但不会成为本机最终飞行目标。没有租约的簇只参与发布与竞价，不能被直接执行。
每个簇只保留可见性评分最优的一个视点，最终目标由本机在自己当前位姿、速度方向和偏航条件下从
有效租约簇的这些代表视点中选择。

#### 仿真阶段关闭近距协调器悬停

为避免近距离双机时协调器 hold 掩盖租约/选点实验结果，暂时关闭了以下**协调器命令转发层**拦截：

- 起飞后要求两机满足 `min_start_separation` 的等待条件；
- 命令参考点进入对机 `safe_separation` 半径时的 hold；
- 与对机预测轨迹点接近时的 hold。

对方状态过期时仍会 hold；`CoverageMap` 和 OctoMap 分支中的动态对机机体过滤也保持启用。因此本项
仅用于移除“距离近即停止转发覆盖命令”的仿真干扰，并不等同于删除对机观测假占据的处理。

#### 构建与运行注意

本次修改了 `SwarmFrontier.msg`、`SwarmTask.msg` 和 `SwarmTrajectory.msg`，已重新生成 ROS
消息并编译验证：`two_uav_coverage_search_node`、`two_uav_coordinator`、`two_uav_swarm_bridge`
均构建成功。测试前必须同时重启两侧覆盖节点、协调器和 bridge，不能让新旧消息定义混用。

### 2026-07-27：覆盖完成共识、任务 lease 防抢占与滚动交接说明

本节覆盖上文“执行进展续租”和“区域匹配”的旧细节；当前代码以本节为准。

#### 使用范围与启动入口

本轮修改只面向 `two_uav_coverage_searching_pkg` 的双机仿真流程。常用启动顺序为：

```bash
~/Prometheus/Modules/two_uav_coverage_searching_pkg/scripts/start_two_uav_sim.sh 1 2 1.5
~/Prometheus/Modules/two_uav_coverage_searching_pkg/scripts/arm_two_uav_sim.sh 1 2
~/Prometheus/Modules/two_uav_coverage_searching_pkg/scripts/start_two_uav_coverage_sim.sh 1 2 1.5
```

两台无人机运行相同的覆盖搜索与协调算法；不能把 UAV1 视为中心控制器或唯一完成裁决者。

#### 2D 覆盖率、阶段统计与双机完成共识

覆盖率继续使用 `CoverageMap::getKnownSpaceRatio()` 的二维体素统计。任务的**本地完成就绪**条件为：

```text
二维已知比例 >= 99%
且
连续无前沿 >= completion_no_frontier_dwell（默认 8 s）
```

达到条件的 UAV 不会单独结束，而是发布 latched 主题：

```text
/uav<N>/prometheus/coverage_search/completion_ready = true
```

本机 coordinator 将该标志随 `SwarmState.completion_ready` 经现有 bridge 转发给对机。只有同时满足
“本机 ready + 对机状态新鲜且 ready”时，两台无人机才各自进入 `FINISH`，悬停后走既有降落流程。
单机模式不等待对机。在达成双机共识并进入 `FINISH` 前，若前沿再次出现、已知比例跌破阈值，
或对机状态超时，则 ready 被撤销。

`publishCoverageStatus()` 在覆盖开始后锁存首次达到 25%、50%、75% 的时刻，并持续打印：

```text
Duration, T25, T50, T75, Known, Frontiers, Ready, Peer ready
```

进入 `FINISH` 后 `Duration` 锁定为 `finish_time - start_time`，不把悬停和降落时间继续累加。
为了避免两台终端重复刷统计，协同模式仍按 `global_coverage_source` 选择一个日志输出源；这不改变
完成共识本身，两个 UAV 都必须 ready 才会结束。

实现位置：

- `msg/SwarmState.msg`：新增 `bool completion_ready`；
- `two_uav_coverage_search_manager.cpp`：本地 ready 判定、阶段时间锁存与 FINISH 时间锁定；
- `two_uav_coordinator.cpp`：本地 Bool 到 `SwarmState` 的转发。

#### 任务归属的唯一事实来源：有效 lease

RViz 视点上的 `UAV1/UAV2` 不再由“这一轮最新 bid 的理论赢家”重算，而是由已发布且未过期的
`SwarmTask` lease 决定。标签格式为：

```text
UAV1_7s
```

其中 `7s` 是 lease 的剩余秒数向上取整。两侧任务表对同一 `task_id` 的 owner 不一致时显示红色
`CONFLICT`，而不是伪造一个 owner。

任务状态遵循：

```text
FREE -> LEASED(owner, expiry) -> ACTIVE(owner, heartbeat renew) -> natural expiry
```

- coordinator 维护 `own_leases_`，不会因一次前沿更新或候选列表缺席而删除未过期任务；
- `SwarmTrajectory.active_task_id` 在新鲜（<= 2.5 s）时持续为 ACTIVE 任务续约 8 s；
- 未 ACTIVE 的 bundle 任务仅保留到自然到期，不无限续租；
- 对机任务表不新鲜（> 5 s）或任一方 bid 不新鲜时，协调器**只重发已有 lease，不做新竞价**；
- 对机任务表中任意未过期的同 ID 条目都视为占用，即使消息的 owner 字段异常，也绝不抢占；
- 执行端在每次轨迹循环检查当前 `task_id` 是否仍是本机有效 lease。过期或被重分配时执行安全中止。

因此空闲机只能得到真正 FREE 的任务。当前没有“提前释放/主动接管”协议：ACTIVE 不再上报后，旧
lease 仅等待自然过期。这是有意选择的保守最小实现，优先避免双机同时飞向同一簇；若后续需要故障
接管，应增加带单调 revision 的明确 release/active-absence 协议，而不是用两次本地选点作确认。

“连续两次选择到对机任务才允许执行”的去抖机制没有加入。它不能解决任务表陈旧，且会把一次可靠
lease 的问题变成依赖 10 Hz 调用次数的偶然行为。上述 lease 锁定已在根源排除对机未释放任务。

#### 密集前沿的任务匹配与冲突策略

`task_id` 仍是 0.5 m 区域键，单独使用可能让相邻簇碰撞。执行端当前将前沿簇与任务认定为同一
任务，必须同时满足：

```text
task_id 相同
AND 中心距离 <= 1.0 m
AND 2D AABB 在 0.5 m margin 下重叠
```

如果本机/对机新鲜任务表对同一前沿匹配出不同 owner，执行端返回“不属于本机”，而不是按 UAV ID
选一个赢家；协调器也不会竞价对机表中未过期的同 ID 条目。这一策略会在碰撞时暂时少分配任务，
但不会造成抢占。

这不是跨地图更新的完整簇跟踪器：在极密集前沿中，0.5 m ID 仍可能发生真实碰撞。若日志或 RViz
持续出现 `CONFLICT`，下一步应维护前沿簇轨迹，并以 IoU/中心匹配生成稳定 ID；不要继续放宽当前
三重匹配条件。

#### 滚动轨迹预规划：固定在旧轨迹 100% 终端交接

当前已有滚动预规划，目的是避免每个视点到达后悬停。其严格时序如下：

```text
旧轨迹执行进度 >= 60%
  -> 以该时刻的地图、前沿和旧 B 样条创建后台快照
  -> 固定 handoff_time = 旧 B 样条总时长 T（100%）
  -> 用 De Boor 求唯一的终端 P(T)/V(T)/A(T)/yaw(T)
  -> 以该未来终端状态选择新视点并规划新轨迹
  -> 到 T 时用真实状态复核；成功才切换
```

不再存在 85%..95% 枚举窗口或候选评分。旧轨迹始终执行到规划终点；终端状态取路径末端切向量，
并写入：

```text
|V(T)| = rolling_terminal_speed_ratio * max_vel
|A(T)| = rolling_terminal_acc_ratio * max_acc
yaw_rate(T) = yaw_acc(T) = 0
```

两个 ratio 默认均为 `0.10`，并在代码中限制为不大于 `0.10`；可针对真实飞控向下标定，但不能放宽
本轮约定的 10% 上限。

新时间 B 样条把该终端 P/V/A 写为自己的起始边界，并在生成时以 De Boor 复核首、末端状态不变量；
因此成功交接保持位置、速度、加速度和偏航连续。真实交接仍容许既有的位置、速度、偏航误差阈值；
超限或新前沿消失时取消 successor，不切换到新任务。

终端交接前除新轨迹全程 ESDF/连线、速度/加速度限制检查外，还会检查终端速度方向的制动走廊。若
制动距离内有障碍，则拒绝 successor；若无 successor、交接复核失败或终点路径被最新地图阻塞，则
立即下发当前位置/终点 hold，再安全制动或重规划。安全切断（lease 失效、目标/路径碰撞、ESDF
间隙不足、跟踪卡住等）同样中止轨迹；大部分安全问题重规划同一目标，只有目标本身不安全时才放弃并
重新选点。

当前滚动交接的已知限制是：它不验证**旧**前沿簇是否已经被看清或在最新地图中消失，只验证几何
接近旧视点。因此无人机可能在距旧视点约 0.60 m 时切往新目标，留下尚未完成的旧簇。这与“必须
完成旧簇观测后才离开”的任务语义不一致，本轮按需求只记录原因、未改动轨迹策略。

若要修复，应在未来单独加入“旧任务观测完成屏障”：到达视点并保持足够观测后，用最新地图确认
与旧 `task_id` 匹配的前沿簇已消失，才允许准备或激活 successor。该功能会引入任务完成状态与
跨机同步，不能以简单的二次选点确认替代。

#### 验证与重启要求

已执行：

```bash
make -C build/two_uav_coverage_searching_pkg -j2
git diff --check
```

构建成功。由于 `SwarmState.msg` 的消息定义已变更，测试时必须同时重启两台 UAV 的覆盖节点、
coordinator 和 bridge，避免一侧仍使用旧消息定义。
