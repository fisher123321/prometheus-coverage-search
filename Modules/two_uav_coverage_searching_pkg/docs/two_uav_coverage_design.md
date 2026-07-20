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
- `points_octomap` 不再加入虚拟 FOV clear-point 端点。OctoMap 的 `PointCloud2` 输入无法
  区分“清空射线终点”和真实深度命中，前者会被错误积分为 occupied，形成大块紫色伪体素；
  覆盖搜索仍使用独立的真实 `points_downsampled` 输入，故该改动只影响 OctoMap 可视化。

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
