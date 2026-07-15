# FALCON Exploration Notes

This note tracks the FALCON code paths used to guide `prometheus_coverage_search`.

## Frontier Finder

Source:
`/home/wjy/falcon_ws/src/FALCON/falcon_planner/exploration_preprocessing/src/frontier_finder.cpp`

Important logic:
- Frontier seed: known free voxel with a 6-neighbor unknown voxel (`isNeighborUnknown()` uses `sixNeighbors()`).
- Region growing: 26-neighbor expansion (`allNeighbors()`) over valid frontier voxels. This keeps curved/frontier surfaces connected without making the frontier qualification itself thick.
- Large frontier split: PCA on filtered frontier cells in XY, then recursive split by the first principal component.
- Viewpoint sampling: sample rings around the frontier average, keep viewpoints that are free, not near unknown, and preferably not near occupied.
- Viewpoint score: visible frontier cells plus unknown voxels visible in the camera FOV.

Mapped changes:
- `FrontierFinder::searchFrontiers()`: first run is full, later runs are incremental. It removes changed/overlapping old clusters, scans only the updated AABB for new seeds, and keeps unchanged clusters.
- `FrontierFinder::searchFrontiers()`: scans a configurable 3D height band around the flight layer (`frontier_z_half_layers`) and clamps it by `min_frontier_height`/`max_frontier_height`. The lower bound stays above the floor so ground cells are not frontiers, but the frontier surface is thick enough in Z for the depth camera.
- `frontier_finder/max_frontier_height` is the ceiling-frontier filter. It removes floating upper/ceiling-like frontier cells that should not guide a low-altitude indoor exploration run.
- Floating one/two-layer fragments are not useful exploration frontiers. Keep `min_frontier_z_layers >= 3` and require enough XY cells plus neighbor support before accepting a cluster.
- Ceiling/top frontiers are filtered twice: absolute `max_frontier_height` and relative `max_frontier_above_flight`. A cluster also needs enough Z extent (`min_frontier_z_extent`); a set of cells lying in only one thin Z layer is not a valid exploration frontier cluster.
- `FrontierFinder::isFrontierCell()`: now uses FALCON-style 6-neighbor qualification and requires a horizontal unknown neighbor, so diagonal/vertical unknown remnants do not make the XY frontier several cells thick.
- `FrontierFinder::expandFrontier()`: uses FALCON-style 26-neighbor region growing. The frontier qualification remains direct-neighbor free/unknown, but the cluster growth must connect diagonal/curved frontier surface cells so one wall-like frontier is one dense cluster.
- `FrontierFinder::mergeCloseFrontiers()` was removed. Frontier clusters now come from region growing plus PCA splitting only; do not re-add close-cluster merging without explicit approval.
- `FrontierFinder::splitLargeFrontiers()`: PCA split is kept with moderate thresholds. Do not make it too conservative: large frontiers should split around `cluster_size_xy`, while child clusters must still satisfy the tight-cluster validity checks. Also do not split very high-aspect-ratio thin continuous frontier strips; those should remain one cluster instead of a row of colors.
- `FrontierFinder::sampleViewpoints()`: scores viewpoint yaw by frontier visibility and visible unknown gain.
- A frontier viewpoint is a free pose sampled on a ring around the frontier cluster. It must not be re-filtered by "near frontier/near unknown" in target selection; FALCON only uses distance fallback in `getTopViewpointsInfo()` and still returns the best viewpoint when all viewpoints are close.
- `FrontierFinder::publishFrontiers()`: RViz must show both frontier cluster cells and each cluster's current best viewpoint/yaw, using the same cluster color. This makes it visible whether a failure is viewpoint generation or later reachability/selection.
- Low visibility must not delete an otherwise free candidate viewpoint. Keep such candidates as fallback with low score, otherwise the planner can enter `Frontiers exist but no usable viewpoint` and stop moving.
- `FrontierFinder::publishFrontiers()`: RViz visualization is downsampled and capped per cluster and globally; cluster colors are generated from golden-ratio HSV so the fixed palette does not repeat.

## Perception Utils

Source:
`/home/wjy/falcon_ws/src/FALCON/falcon_planner/exploration_utils/src/perception_utils/perception_utils.cpp`

Important logic:
- Camera FOV uses a shorter visualization distance (`vis_dist`) than sensing distance.
- FOV is drawn as a LINE_LIST frustum.
- Unknown visibility is evaluated by ray casting through camera FOV samples.

Mapped changes:
- `CoverageSearchManager::publishCameraFov()` refreshes the depth camera FOV from the UAV-state callback with a persistent marker, so RViz does not blink between updates. The dedicated FOV marker publisher uses queue size 1 and is not latched, so RViz receives the newest frustum instead of delayed queued markers.
- FOV visualization must use a stable marker id and a dedicated `fov_vis` topic; publishing it only on the busy path marker topic can flicker in RViz.
- `CoverageMap::raycastCameraFovFree()` clears only no-return camera directions out to the trusted depth range; angular bins containing real returns and their neighbors are excluded so clearing cannot pass behind observed obstacles.
- Depth points below `sensor/depth_ground_ignore_height` clear free space but are not inserted as occupied obstacles in the internal coverage map. OctoMap still receives the downsampled depth cloud and keeps `filter_ground=true`.
- `coverage_search/camera_pitch` must match the Gazebo D435i model pose. A mismatch corrupts the transformed depth cloud and creates false frontiers/no-viewpoint failures.
- `FrontierFinder::countVisibleUnknown()` estimates unknown gain by ray casting through FOV samples.
- `depth_cloud_downsample_node`: filters D435i point clouds by range and voxel-grid downsamples them before octomap/RViz/coverage search, reducing `occupied_cells_vis_array` and frontier-update load.

## Trajectory Server

Source:
`/home/wjy/falcon_ws/src/FALCON/falcon_planner/fast_planner/src/traj_server.cpp`

Important logic:
- FOV marker is refreshed with a stable marker id and namespace.
- Trajectory execution follows a time/preview target rather than getting stuck on dense points.

Mapped changes:
- `executeTrajectory()` uses lookahead target selection.
- Dense trajectory point dwell time is reduced.
- The former immediate `tryPreemptWithForwardFrontier()` path and its parameters
  were removed. Immediate replacement started a new curve
  from a measured mid-flight state and could not preserve P/V/A continuity.
  Rolling preparation and a validated handoff buffer replace it.

## Local Viewpoint Selection

Source:
`/home/wjy/falcon_ws/src/FALCON/falcon_planner/exploration_manager/src/exploration_manager.cpp`

Important logic:
- `refineLocalTourHGrid()` builds a local cost matrix from current pose/velocity/yaw to candidate viewpoints and then to the next grid target.
- `PathCostEvaluator::computeCost()` and `computeCostUnknown()` make path distance and yaw/velocity transition cost the main cost terms; gain is useful, but it should not make the robot jump across the whole map.

Mapped changes:
- `CoverageSearchManager::computeFrontierCost()` now keeps travel distance as the main term. Forward direction is a bounded preference, not an unbounded reward for far-away frontiers.
- `coverage_search/fly_height` must match `/uav_control_main_1/control/Takeoff_height`. If coverage uses a different fly layer from the real depth-camera height, depth rays update one Z layer while planning/coverage/frontier selection query another, causing `Coverage: 0%` and `Frontiers exist but no usable viewpoint`.
- `selectNextFrontier()` first sorts candidates by approximate local cost, then runs A* reachability only for the top few candidates. This avoids several-second pauses when many frontiers exist.

## 2026-07-14 Tiered Viewpoint Clearance

- Viewpoints are reviewed per frontier cluster in ordered occupied-clearance
  tiers: 0.55 m, 0.50 m, 0.45 m, and 0.35 m. The first tier containing any
  candidates wins; lower tiers are ignored for that cluster. No viewpoint below
  the 0.35 m ESDF hard limit is admitted.
- Runtime path validation retains the normal 0.55 m clearance. Only near a
  selected lower-tier endpoint does the required clearance taper toward that
  endpoint's actual tier, never below 0.35 m. This prevents a 0.45 m endpoint
  fallback from weakening the safety margin along the whole path.
- Logs expose `Viewpoint clearance fallback` and the selected target's
  `clearance_tier`.

## 2026-07-14 RViz OctoMap Crash

- This was a native plugin crash, not exhausted RAM. The ROS launch log recorded
  RViz exit code `-6` (SIGABRT). The saved core backtrace ended in
  `octomap::OcTreeBaseImpl::~OcTreeBaseImpl()` called by
  `octomap_rviz_plugin::TemplatedOccupancyMapDisplay::handleOctomapBinaryMessage()`
  while processing repeated `/uav1/octomap_binary` updates; glibc reported heap
  corruption (`corrupted size vs. prev_size in fastbins`).
- `maze_mapping.rviz`, which is the default config of
  `sitl_outdoor_1uav_P450.launch`, no longer loads the crashing OctoMap display
  classes. It uses the standard `rviz/Map` display on `/projected_map` instead.
  The robot model, coverage markers, depth cloud, and other displays remain.
- The edited RViz config passes YAML parsing. This avoids that known plugin crash
  path; a different future RViz failure must be diagnosed from its new exit code
  and backtrace rather than assumed to have the same cause.

## 2026-07-14 Continuous Rolling Trajectory Handoff

### Root cause

- A geometrically smooth B-spline was still followed by terminal
  `XYZ_POS` braking. Only after the UAV stopped did the exploration loop select
  and generate the next goal. Therefore each individual path looked smooth, but
  the reference sequence was discontinuous between viewpoints.
- Planning in the ordinary single-threaded ROS callback would also freeze
  command publication during target A* checks. The rolling planner therefore
  works on a deep snapshot of the map buffers, frontier list, vehicle state,
  and active trajectory in a worker thread. The main callback continues sending
  the old trajectory at 10 Hz.

### Implemented handoff

- At 60% active-trajectory progress, one worker predicts a future state from
  the old trajectory's synchronized position, velocity, acceleration, and yaw
  samples. A handoff state is allowed only near the old viewpoint, with old-goal
  yaw substantially aligned and nonzero forward speed. Thus the old viewpoint
  is still observed; the algorithm does not cut the corner merely to avoid a
  stop.
- The snapshot selects another reachable frontier (excluding the old viewpoint
  neighborhood), runs the existing A* and B-spline pipeline, and constructs a
  closed-form quintic bridge. The quintic exactly matches P/V/A at the old
  handoff state and at an early state of the new trajectory. Yaw is unwrapped
  over the shortest arc and bridged with matched endpoint yaw rates.
- Every bridge sample and chord is checked against free space, the tier-aware
  ESDF clearance, maximum velocity, maximum acceleration, and maximum yaw rate.
  At activation, measured position, velocity, and yaw errors are checked again,
  the new frontier must still exist, and late results carry a trajectory
  generation number so they cannot overwrite a newer plan.
- No terminal hover command is published for an accepted handoff. The old and
  new P/V/A/yaw references meet at the same state. Distinguishing logs are
  `Rolling preplan worker started`, `Rolling trajectory ready`, `C2 bridge
  accepted`, `Rolling handoff activated`, and explicit cancellation reasons.
- Continuous handoff is conditional: if no other frontier is known early
  enough, the old viewpoint requires a zero-speed endpoint, the bridge violates
  dynamics/ESDF, tracking error is too large, or the next frontier becomes
  stale, the existing measured-state terminal braking remains active. This is a
  deliberate safety fallback, not a forced unsafe splice.

### Why NLOpt was not added

- NLOpt is unnecessary for the equality constraints in this change. A quintic
  polynomial solves the two-end P/V/A boundary conditions directly and is then
  safety-validated. Adding a numerical optimizer would increase latency and add
  failure modes without making those boundary conditions more exact. NLOpt
  remains appropriate only if a later requirement adds a genuine optimization
  objective such as minimum snap over free segment times.

### Verification

- `catkin_make --source Modules/coverage_searching_pkg --build build/coverage_searching_pkg -j4`
  completed successfully after the worker, C2 bridge, activation guards, and
  launch parameters were added.
- `roslaunch prometheus_coverage_search coverage_search.launch --nodes` resolves
  `/coverage_search_node`, `maze_mapping.rviz` passes YAML parsing, and
  `git diff --check` reports no whitespace errors.
- If the UAV has current velocity, `selectNextFrontier()` first uses the velocity direction as the forward direction. If forward candidates exist, only forward reachable targets are allowed. Backtracking is allowed only when the forward side has no frontier viewpoint candidates at all.
- Do not keep expanding A* checks when many forward candidates are temporarily unreachable. Use direct-path checks first, a small A* time budget second, then a reachable forward relay toward the frontier instead of hovering or choosing a backward target.
- Recovery target selection also sorts cheap candidates before A*, instead of running A* for every ring sample.
- Forward preemption only accepts candidates farther along the current goal direction and only near the end of the current trajectory, reducing left-right goal switching while still allowing the UAV to continue toward newly discovered frontiers ahead.
- `generateBsplineTraj()` rebuilds yaw references along the trajectory by sampling yaw candidates and scoring visible unknown gain. The physical D435i pitch is fixed by the Gazebo model, so active exploration attitude control here is primarily yaw planning.
- `executeTrajectory()` advances by nearest future trajectory point plus a dynamic radius. Do not require the UAV to hit every dense sampled point exactly when position commands use a lookahead target.
- A* adds ESDF clearance cost near raw occupied cells; fallback trajectories use safe Chaikin corner smoothing before falling back to raw free line segments.

## 2026-07-02 Maze ROI, A*, Yaw/Path Notes

Reference:
`/home/wjy/catkin_tp/yaw_and_path_planning_analysis.md`

Mapped changes:
- `maze.world` outer walls are at approximately `x/y = +/-20m`. `coverage_search.launch` must not set the internal coverage map to `[-22, 22]`, otherwise unknown space outside the sealed maze wall becomes a valid frontier source. Keep the ROI inside the wall, currently `origin=(-19.6,-19.6)`, `size=(39.2,39.2)`.
- Keep the same ROI in `coverage_search.yaml` even if launch overrides it, so future runs do not silently reintroduce outside-wall frontiers.
- Do not use an inflated occupied map as the hard safety boundary in this coverage node. A* should search raw free cells, but must also require hard ESDF clearance. Making inflated cells hard obstacles makes narrow maze corridors appear unreachable and causes `Forward frontier viewpoints exist but are not reachable yet`.
- A* timeout checks must run frequently. Checking only every 1000 expansions lets a nominal 18-35ms reachability query consume hundreds of ms and stall command publishing.
- When forward frontier viewpoints exist but are temporarily unreachable, do not choose a backward target. First try a reachable forward relay in the current velocity/committed-heading corridor so the UAV keeps opening the map with the depth camera.
- `HierarchicalGrid::updateFromMap()` is only needed when frontier fallback is used. Running it every frontier update adds a full-map scan to an already expensive frontier pass and causes visible pauses.
- FUEL yaw planning uses a forward lookahead along the position trajectory plus yaw continuity (`calcNextYaw`). In this lightweight node, `generateBsplineTraj()` approximates that by looking 0.9-3.0m ahead on the generated path, then blends toward the frontier/viewpoint yaw near the end.
- `planYawForPoint()` should balance unknown gain with path yaw and previous yaw. A pure information-gain yaw selector can make the UAV pause and swing the camera instead of moving.
- A* paths should be shortcut before B-spline sampling, matching the FUEL `shortenPath` idea. This reduces grid-zigzag waypoints and gives the B-spline a cleaner route.

## 2026-07-02 Forward-Blocked Relay And Yaw DP Fix

Mapped changes:
- A forward frontier candidate is not automatically a useful forward goal. If it is behind a wall, forcing forward-only selection produces repeated short relays such as `dist=0.72, gain=0`, which does not reveal new unknown space and looks like the UAV is stuck.
- Relay targets must either move a meaningful distance forward or provide positive unknown gain. Tiny `gain=0` relays are ignored.
- If forward candidates exist but all reachable tests and useful relays fail, treat the current local direction as blocked/dead-end and allow a large turn to any reachable frontier. This matches the intended rule: keep moving with the velocity direction until a local dead-end/no-gain condition is reached, then allow turning back or sideways.
- The main loop must release `committed_heading_` after repeated selected-target failures. Permanent strict heading lock is wrong in a maze dead-end.
- Yaw planning is now sequence-level dynamic programming over candidate yaw layers, inspired by FUEL `planYawActMap/searchPathOfYaw`: local unknown gain is rewarded, but adjacent yaw changes are penalized by yaw-rate cost. This is more stable than greedy per-point yaw selection.
- Simulation parameters are slightly faster: `max_vel=2.8`, `max_acc=4.0`, `max_yaw_rate=1.8`.

## 2026-07-02 Actual Path Cost And Unknown-Limited Traversal

Mapped changes:
- Next-viewpoint selection must not use only Euclidean distance. First keep the nearest fraction of candidate viewpoints as a fast prefilter, then run A* on that pool and use actual path length as the dominant cost.
- Final exploration targets must be real frontier-cluster viewpoints (`ftr.viewpoints`). Relay/probe points may be used only as local recovery commands when no viewpoint is selectable; they must not be reported as `Heading to frontier`.
- A* path search now follows raw free cells only. Unknown cells are not part of planned traversal; raw occupied cells are the only hard obstacle.
- Trajectory generation and execution checks follow the same hard rule: raw occupied is blocked. ESDF is used to reject unsafe smoothed curves and to bias A* away from walls.
- Very close viewpoints create one-point trajectories and long yaw-only stalls; skip viewpoint targets closer than about 0.55m and let local observation handle such cases.
- Frontier-cluster detection remains unchanged, but viewpoint scoring is cheaper: visible-cell scoring samples large clusters by stride and unknown-gain ray sampling is reduced. This speeds frontier updates/RViz refresh without changing what qualifies as a frontier cell.

## 2026-07-02 Viewpoint Selection Stall Fix

Reference re-read:
- `/home/wjy/catkin_tp/viewpoint_sampling_selection_analysis.md`
- FUEL `active_perception/src/frontier_finder.cpp::sampleViewpoints/getTopViewpointsInfo/getViewpointsInfo`
- FUEL `active_perception/src/graph_node.cpp::ViewNode::computeCost`

Mapped changes:
- `Frontiers exist but no usable viewpoint` with `no_view=0, too_close=0` means the code had valid viewpoint candidates but the target selector only tested a small, approximate-cost-sorted subset. If those happened to be behind walls or timed out in A*, the UAV hovered even though other real viewpoints were reachable.
- Candidate selection is now staged by real reachability: first test the nearest Euclidean pool, then, if no reachable candidate is found, release the forward lock and test all real frontier viewpoints. Relay/probe points are still not reported as frontier targets.
- Final ranking is dominated by obstacle-aware A* path length. A same-distance band is defined as `max(0.9m, 25% of nearest path length)`. Only inside that band can yaw and unknown gain decide the winner; outside the band, extra path length is strongly penalized.
- Recovery selection follows the same rule and checks more candidates by Euclidean order, instead of sorting by an approximate gain score first.
- Viewpoint sampling is closer to FUEL: concentric ring sampling uses `1.0-2.6m` radii and `15deg` angular steps. Each sample first evaluates average frontier yaw and center yaw; only poor samples evaluate small yaw offsets, keeping frontier update time bounded.
- A* reachability is now raw-free based. Unknown traversal limits were removed from the actual walkability rule because viewpoint reachability should be determined by known free connectivity.

## 2026-07-02 RViz Best Viewpoint And Linear Fallback Fix

Mapped changes:
- Be precise about RViz semantics: `frontier_best_viewpoint` was the highest local visibility viewpoint in each cluster, while `selectNextFrontier()` could pick another candidate viewpoint from the same cluster after path-cost sorting. That made the displayed best viewpoint and the actual next target appear inconsistent.
- The 2026-07-03 reachability fix changed this: RViz still shows the best local-visibility representative for each cluster, but normal target selection may inspect several high-score viewpoints from the same cluster and choose the reachable one after cheap filters. Reachability stays the final expensive filter instead of being baked into frontier sampling.
- The repeated `Failed to generate safe linear trajectory` came from `buildLinearTraj()` rejecting shortcuted/smoothed interpolated points after A* had already accepted the grid path. The later safety fix keeps the A* path un-shortcuted for fallback and checks raw occupied blockage; B-spline/Chaikin smoothing also requires enough ESDF clearance.

## 2026-07-02 Incremental Frontier Update And Selection Budget

Reference re-read:
- FALCON `/home/wjy/falcon_ws/src/FALCON/falcon_planner/exploration_preprocessing/src/frontier_finder.cpp:53-134`

Mapped changes:
- Previous `FrontierFinder::searchFrontiers()` was full-map every time: clear all frontiers, clear all flags, scan all x/y/z cells, then resample viewpoints for every cluster. This explains frontier update growing to 1-2s as the explored map and frontier count grew.
- `CoverageMap` now records an updated AABB whenever raw occupancy/free cells actually change. Raycasting marks newly freed cells; obstacle writes mark changed cells only.
- Frontier search is now FALCON-style incremental: first search is full; later updates get the changed AABB, remove only overlapping clusters that are no longer valid, scan only the ESDF-safe-distance padded AABB for new seeds, and keep unchanged clusters.
- Viewpoints are no longer resampled for unchanged clusters. `computeFrontierInfo()` clears viewpoint data only for new/split clusters, and `computeViewpoints()` samples only clusters with empty viewpoint lists.
- Target selection keeps a bounded A* budget while still checking several cached viewpoints per cluster. Do not collapse selection to one displayed best viewpoint per cluster.

## 2026-07-02 Viewpoint Clearance And Safe Fallback Trajectory

Mapped changes:
- A sampled viewpoint must not sit inside its own frontier cluster. Ring samples are now rejected when their XY distance to the cluster's frontier cells is below `frontier_finder/min_viewpoint_frontier_dist` (default `0.55m`). This specifically prevents RViz best-viewpoint spheres/arrows from appearing embedded in the colored frontier voxels.
- `coverage_search/visited_goal_clearance` is a parameter and defaults to `0.8m`, not a hard-coded `2.0m`. A 2m visited-goal radius in maze corridors filtered out many valid nearby viewpoints and produced misleading `Frontiers exist but no usable viewpoint` logs.
- The no-usable-viewpoint log now separates sampled-view absence, not-free filtering, visited-goal filtering, and too-close filtering. This makes future terminal logs actionable instead of treating every rejection as `no_view`.
- Recovery target selection has a hard time budget and fewer A* checks. The old recovery path could spend about a second in repeated reachability checks after normal selection failed, causing visible hover gaps.
- Do not shortcut A* paths across long line segments before fallback trajectory generation. The raw A* corner path is safer near maze walls, while B-spline/Chaikin smoothing may still be used only when its sampled curve stays collision-free.
- Linear fallback trajectories check raw occupied cells and segment blockage. Smoothed trajectories additionally require ESDF clearance; occupied cells are never allowed in the command trajectory.

## 2026-07-03 Raw-Free A* And ESDF Distance Field

Reference re-read:
- `/home/wjy/catkin_tp/tsdf_esdf_map_analysis.md`
- FUEL `plan_env/src/sdf_map.cpp::fillESDF/updateESDF3d`
- FALCON `exploration_preprocessing/src/frontier_finder.cpp`

Mapped changes:
- Removed the inflated-map hard obstacle path from `CoverageMap`. `occupancy_buffer_` is now the raw hard map: `0=unknown`, `1=free`, `2=occupied`. `isOccupied/isFree/isOccupied2D/isFree2D` no longer check an inflated buffer.
- Added ESDF buffers and TSDF-style truncated signed-distance queries derived from raw occupied cells. This follows the practical FUEL pattern for using distance fields, while avoiding a full log-odds TSDF front-end rewrite in this lightweight node.
- ESDF is updated only when the raw map is dirty. The update uses the three-pass 1D Felzenszwalb-Huttenlocher distance transform from the FUEL notes.
- A* walkability is raw-free only. Unknown is no longer accepted as traversable, and `max_unknown_ratio` no longer determines reachability.
- ESDF distance is used as a hard clearance gate in A*, direct path checks, viewpoint filtering, and trajectory execution. It is not an inflated occupancy map.
- Viewpoint target selection now considers up to six high-score viewpoints per frontier cluster, then applies reachability as the final expensive filter during target selection. This keeps viewpoint sampling cacheable while avoiding the old single-representative viewpoint failure.
- D435i trusted depth range is now `0.3-3.0m` in launch/config and default map parameters.

## 2026-07-03 ESDF Runtime And Forward Preempt Fix

Mapped issue:
- Full 3D ESDF recomputation was put in the frontier update/target-selection hot path. With the maze map size this made `frontier update took 1.1-1.4s` and could make `target selection` inherit the same stall whenever a depth update dirtied the map.
- `Forward preempt` gave an unbounded positive reward to large forward projection, so a far viewpoint with high frontier gain could interrupt a nearby valid goal. This caused the UAV to jump from a local target to a long path across the map.

Mapped changes:
- ESDF maintenance is now 3D again, but uses a dirty-AABB local update for ordinary depth changes and falls back to full 3D EDT only for the first run or large dirty regions.
- A* reachability is raw-free plus hard ESDF clearance.
- Forward preemption is now local. A new target must stay within a bounded forward lookahead distance, its measured reachable path length must be close to the current remaining goal distance, and the final score subtracts path length before it is allowed to replace the current goal.

## 2026-07-03 Wall Collision Fix

Mapped issue:
- Having TSDF/ESDF buffers did not by itself prevent collision. The previous code used ESDF mostly as a soft A* penalty and as a smoothing filter. The current code treats ESDF clearance as a hard gate for A*, direct segments, trajectory generation, and command lookahead.
- `executeTrajectory()` also sent a far lookahead position target (`max_vel * 0.6`, about 1.7m at the current speed). Around maze corners this allowed the low-level position controller to cut a straight line toward a far safe waypoint, even if the safe discrete trajectory curved around the wall.
- `traj_advance_dist=0.85` let the executor skip trajectory points before the UAV was actually close to them, increasing corner cutting.

Mapped changes:
- A* walkability now requires raw-free plus ESDF clearance. Direct reachability checks also sample at half-resolution and reject low-clearance cells.
- Linear fallback, Chaikin smoothing, B-spline sampling, and inserted fill points all require `free && ESDF >= traj_min_clearance`; linear fallback is no longer just a raw-occupied check.
- `pathToTargetBlocked()` is now a hard execution safety check over the whole segment from current position to command point: out-of-map, not-free, or low-ESDF-clearance all block.
- Command lookahead is capped below 0.9m and reduced near walls. The executor selects only the farthest lookahead point whose direct segment from the UAV is ESDF-safe.
- `traj_step_size` and `traj_advance_dist` are both set to `0.35m`; `map/esdf_safe_distance` and viewpoint occupied clearance are set to `0.40m`.

## 2026-07-07 Target Cleanup And Hard 3D ESDF

Mapped changes:
- Deleted `mergeCloseFrontiers()` completely: declaration, definition, calls, launch params, and yaml params are gone.
- Exploration targets are only frontier-cluster viewpoints from `ftr.viewpoints`. Normal selection, recovery selection, and forward preemption may still inspect several cached viewpoints per cluster; do not restrict selection to only `viewpoints[0]`, because that can skip an otherwise reachable nearby cluster.
- Removed hidden non-frontier movement targets: hierarchical-grid fallback, A* local direction fallback, and no-reachable-viewpoint probe movement. If no frontier viewpoint is selectable, the UAV observes in place and waits for updated frontiers.
- `goal_point` in RViz is the actual `current_goal_`. It may be a non-displayed cached viewpoint from the selected frontier cluster, while `frontier_best_viewpoint` shows the cluster's representative best viewpoint.
- During trajectory execution, if the current goal becomes non-free or its ESDF clearance drops below the hard cutoff, the trajectory is cut immediately and the goal is short-term filtered so the next cycle selects another frontier viewpoint.
- ESDF/TSDF update is 3D. Normal updates use the dirty AABB expanded by the ESDF radius; full 3D EDT is used only on first update or when the dirty region is too large.

## 2026-07-07 OctoMap Vs CoverageMap Debug

Mapped issue:
- RViz `OccupancyMap` is OctoMap, while frontier clusters come from `CoverageMap`. Both subscribe to the downsampled D435i cloud, but OctoMap does not know about coverage_search's extra FOV free-space clearing for no-return depth directions.
- Therefore an OctoMap dark-cyan patch can be an OctoMap projection/free-clearing artifact, not necessarily a real internal frontier.

Mapped changes:
- `depth_to_octomap.launch` now uses the same trusted D435i range as coverage_search: `0.3-3.0m`.
- `CoverageMap::publishMap()` now also publishes a debug marker on `/uav1/prometheus/coverage_search/map_vis`, namespace `internal_unknown_free_boundary_2d`. It shows only internal-map unknown cells that touch internal free cells. If OctoMap is dark but this marker is absent there, the mismatch is OctoMap-side display/clearing. If this marker is present but no colored frontier cluster is present, then the frontier height/cluster filters are the next thing to inspect.

Follow-up fix:
- `depth_cloud_downsample_node` keeps `/camera/depth/color/points_downsampled` as real measured points for coverage_search.
- It can also publish `/camera/depth/color/points_octomap` with sparse FOV clear points only for OctoMap. Real depth returns block nearby angular bins, so clear points are added only in directions with no measured return. This gives OctoMap rays for open space without feeding fake obstacle endpoints into frontier logic.

Observed result:
- When the depth camera looks into open space, the raw depth cloud has few/no endpoints. OctoMap cannot clear those no-return rays by itself, so RViz `OccupancyMap` showed mixed unknown/free patches. With the dedicated OctoMap cloud, open FOV directions now get sparse clearing endpoints while coverage_search still receives only real measured points.

## 2026-07-07 Startup Linear Trajectory Bootstrap

Mapped issue:
- In the first few seconds after starting coverage search, logs repeatedly showed `Failed to generate safe linear trajectory` even for nearby frontier viewpoints. The selected viewpoint was reasonable, but `buildLinearTraj()` required every interpolated cell to already be `free` and also called `pathToTargetBlocked()`, which has the same raw-free hard check.
- At startup the depth map and ESDF are still catching up. Nearby cells can be not-yet-free for a short moment even when no known obstacle blocks the segment, so all early candidates were rejected and the UAV hovered.

Mapped change:
- Only during the first 6 seconds, and only for goals within 1.8m, linear trajectory generation may pass through not-yet-free cells if the segment has no known occupied cell and ESDF clearance is still above the normal trajectory cutoff. Normal exploration keeps the original hard `free && ESDF` checks.

## 2026-07-08 Trajectory Cut Rules

> Superseded on 2026-07-14 for execution clearance and completion: the current
> hard runtime clearance is 0.55 m, and a goal is completed only by the measured
> position+yaw+speed condition documented in the latest section below.

Mapped changes:
- Added execution-time trajectory cutting in `executeTrajectory()`. This runs while the UAV is executing the trajectory, not during frontier selection.
- `coverage_search/traj_cut_clearance = 0.5m`: if a future trajectory guide point within the short lookahead falls below this ESDF clearance, the current trajectory is cut and replanned to the same frontier viewpoint.
- If the final selected frontier viewpoint falls below `traj_cut_clearance`, the current goal is aborted and the next frontier viewpoint is selected.
- `coverage_search/early_switch_progress = 0.90`: once the UAV reaches 90% of the current trajectory, the current goal is marked visited and the next frontier viewpoint is selected, instead of forcing the UAV to fly all the way to the final trajectory point.
- `map/esdf_safe_distance` and `traj_cut_clearance` are both set to `0.35m` to keep planning and execution cut thresholds consistent.
- Safety cutting runs before early switching. Early switching only marks a goal visited if the UAV is already within sensing range of the selected frontier viewpoint.
- `coverage_search/early_switch_goal_dist = 0.5m`: early switching now requires the UAV to be close to the selected viewpoint itself, not merely within the camera sensing range.
- When no usable viewpoint is found, the log now reports direct traversability failures, A* attempts/failures, A* skips, and budget breaks to diagnose whether far frontiers are failing due to reachability or search budget.
- A* failure logging is split further: `astar_timeout` means the per-search time/expand budget stopped the search; `astar_no_path` means the current internal map search exhausted with no path; `astar_endpoint_blocked` means start/goal could not be moved to a walkable cell; `astar_unknown_rejected` means A* found a candidate path but it still crossed unknown cells and was rejected by the current free-only policy.
- Added completed-goal cutting: during trajectory execution, after a short delay, the current goal is checked every `0.5s`. If no current frontier viewpoint remains near the selected goal, or the best nearby viewpoint gain is very low for two consecutive checks, the goal is treated as completed/visited and the next frontier viewpoint is selected. This is not a failed-goal path.

## 2026-07-08 ESDF Elastic-Band And B-spline Control-Point Adjustment

> Superseded on 2026-07-14 for the hard execution/controller-chord threshold:
> current trajectory execution uses 0.55 m; 0.35 m below records historical
> behavior.

Mapped change:
- Before B-spline generation, the sparse A* path gets a conservative three-term ESDF elastic-band adjustment: obstacle push, local smoothness, and weak pullback to the original A* path. Only intermediate A* waypoints move; start and goal stay fixed.
- After B-spline control points are built, intermediate control points get a small five-term refinement: obstacle push, smoothness, path length, pullback to original control points, and second-difference dynamics smoothing.
- Any waypoint/control-point move is accepted only if the new point is free, has at least `traj_min_clearance`, and both adjacent line segments pass the existing hard safety check. If no safe move exists, the original point is kept.

## 2026-07-09 Long-Range A-star Timeout Recovery

Mapped issue:
- Late in exploration, distant frontier viewpoints were reported unusable with `astar_timeout` equal to every A* attempt and `astar_no_path=0`. The selector allowed only `25ms/35ms` per quick A* search, then skipped most remaining viewpoints. The exhaustive pass also repeated candidates already attempted by the primary pass.

Mapped change:
- Primary and exhaustive quick A* passes no longer repeat the same viewpoint.
- If all quick checks fail and at least one failure is a timeout, the selector retries up to three nearest real frontier viewpoints with the normal `250ms` A* allowance and a bounded `700ms` rescue budget. This changes only reachability evaluation; frontier generation and target scoring are unchanged.
## 2026-07-09 A-star Per-Search Grid Query Cache

- Symptom: distant frontier viewpoints repeatedly ended in `astar_timeout`, while
  `astar_no_path` stayed zero and the UAV hovered.
- Root cause: every expanded edge repeatedly scanned the same Z layers through
  `isFree2D()` and `getDistance2D()`.
- Fix: cache each 2D cell's free/ESDF result for one A-star search. Safety
  thresholds, frontier logic, target ordering, and timeout limits are unchanged.
## 2026-07-09 Three-Layer 2D Planning Projection

- A-star occupancy and ESDF projection now checks the flight-height layer plus
  one grid layer above and below.
- At `0.2 m` resolution, the vertical planning band is `fly_height +/- 0.2 m`;
  safety distance and frontier logic are unchanged.

## 2026-07-09 Source File Split

- Split the former monolithic `coverage_search.cpp` into seven implementation
  files: A-star, coverage map, frontier finder, hierarchical grid, manager,
  planning, and trajectory execution.
- This was a mechanical move only. Class declarations, function bodies,
  parameters, topics, logs, and algorithms were not changed.
- `CMakeLists.txt` still builds the implementations into the same
  `coverage_search_lib` and links the same `coverage_search_node`.
- Splitting exposed `Print::s_object_name` as a header-level multiple
  definition. `printf_utils.h` now keeps the same process-wide state in a
  function-local static, avoiding duplicate linker symbols without changing
  print behavior.
- Verified with `cmake --build build/coverage_searching_pkg --parallel 2`; both
  `coverage_search_lib` and `coverage_search_node` built successfully.

## Hierarchical Grid Status

- `HierarchicalGrid` is a coarse XY partition over the fine occupancy map, not
  a stack of vertical altitude layers. With the launch value
  `hgrid/cell_size=5.0`, each coarse cell covers a 5 m by 5 m XY area and its
  bounding box spans the map's full Z range.
- A coarse cell is marked active when it contains at least
  `hgrid/min_unknown_num` unknown 2D map cells or at least one frontier.
  Frontier averages are assigned by
  `cx=floor((x-origin_x)/cell_size)` and the equivalent Y expression.
- The class contains a coarse cost matrix and greedy coverage-order
  implementation, but the current manager does not call either method.
  Runtime use is limited to initialization, map statistics when no frontiers
  exist, and assigning frontier averages to coarse cells. It currently does
  not select targets, alter A-star paths, or publish the grid visualization.

## 2026-07-09 Viewpoint Target Cost Rewrite

- Changed frontier-ring viewpoint sampling from `1.0-2.6m` to `1.6-2.4m` in
  launch, YAML, and code defaults.
- Kept the `0.8m` visited-goal exclusion to prevent immediate reselection and
  oscillation, and kept the `0.55m` current-position exclusion to avoid
  degenerate position/yaw-only trajectories.
- Removed the hard forward-half-plane candidate filter. Ordinary target
  selection now evaluates nearby, side, and rear candidates through the same
  cost function.
- Removed `same_dist_band`, the unknown-ratio/cell terms, approximate-cost
  ranking, and the separate recovery score. Euclidean distance is now used
  only to bound and order expensive reachability checks; it is not a target
  cost.
- Candidate reachability now uses one `150ms` quick A-star timeout with a
  bounded `1.8s` selection budget. If none is reachable, up to three nearest
  candidates are retried at the normal `250ms` timeout.
- Final target cost uses the obstacle-aware path length returned by direct
  traversal/A-star:
  `final_cost = 1.25 * path_length / max_vel`
  `+ vel_align_weight * direction_change`
  `+ shortest_yaw_change / max_yaw_rate`.
- `vel_align_weight` is now active and defaults to `1.5`.
  Direction cost is applied only when horizontal UAV speed exceeds `0.1m/s`,
  avoiding unstable direction penalties from near-hover velocity noise.
  Removed unused `strict_no_backtracking`, `yaw_align_weight`,
  `goal_yaw_hysteresis`, `filterByCommittedHeading()`, and
  `headingDiffTo()`.
- The selected actual target now appears as a magenta sphere plus a matching
  yaw arrow (`goal_point` and `goal_view_yaw`). Per-cluster best viewpoints
  retain their existing colors and display.

## 2026-07-10 Final-yaw, Frontier Freshness, And Clearance Fixes

- A frontier viewpoint is now completed only when the UAV is within
  `goal_reach_dist=0.5m` and its measured yaw is within `0.15rad` of
  `current_goal_yaw_`. While finishing the yaw, the position command stays at
  `current_goal_` and the yaw command remains limited by `max_yaw_rate`.
- Removed the old 90%-trajectory early completion and low-frontier-gain
  completion paths because both could clear the goal before final yaw
  alignment. A final-point dwell timeout now preserves the goal and replans
  instead of marking the viewpoint visited. Forward preemption is disabled
  once the UAV is within the goal-reaching radius.
- Removed the extra frontier-update throttle that stretched the nominal
  `1.2s` timer to about `2.4s` during most of a trajectory. Immediately before
  selecting a new target, the manager now consumes the latest dirty-map AABB
  and performs the existing incremental frontier update, so target selection
  no longer starts from an avoidably stale frontier cache. If the map has no
  dirty AABB, frontier search now returns immediately instead of repeatedly
  running a full-map search when the frontier list is empty.
- A-star now has a quadratic clearance penalty from `0.70m` down to the hard
  `0.35m` boundary, zero penalty at and beyond `0.70m`, and an effectively
  infinite penalty below `0.35m`; the existing walkability test still treats
  the hard boundary as non-traversable.
- Both sparse A-star waypoint adjustment and B-spline control-point adjustment
  use the same `0.70m` preferred / `0.35m` hard clearance model. Each optimizer
  step remains bounded to `0.10m`, while total displacement from the original
  point is capped at `0.50m`; all moved points and adjacent segments must still
  pass the hard free-space and ESDF checks.
- Code defaults, launch, and YAML now agree on `0.35m` for both
  `map/esdf_safe_distance` and `coverage_search/traj_cut_clearance`.
- Verified with `cmake --build build/coverage_searching_pkg --parallel 2`; both
  `coverage_search_lib` and `coverage_search_node` build successfully.

## 2026-07-10 Exact A-star Endpoints And Raw-path Fallback

- Root cause of the late-exploration hover was a planner/trajectory mismatch:
  A-star silently moved an unsafe start or goal to the first walkable cell in
  a five-cell neighborhood, while trajectory generation restarted from the
  exact UAV position. A-star could therefore report a long reachable path even
  though the unvalidated exact-start-to-grid-start segment crossed an obstacle
  or the hard-clearance region.
- An unsafe A-star start may now connect only to the nearest candidate that has
  a sampled, known-free bridge whose 2D ESDF clearance never decreases and
  reaches the normal `0.35m` hard boundary. The exact UAV position is inserted
  at the start of the returned path and the bridge length is included in the
  geodesic length. If no such bridge exists, A-star returns
  `ASTAR_ENDPOINT_BLOCKED` instead of jumping across the map.
- A-star no longer relocates an unsafe frontier goal. The exact viewpoint must
  be walkable, and the grid-center-to-exact-goal bridge must pass the normal
  hard check. The returned path now ends at the exact `current_goal_`.
- If the current UAV position is already inside the hard-clearance band, both
  linear trajectory generation and execution allow only the validated
  monotonic-clearance escape prefix. As soon as the prefix reaches `0.35m`, all
  ordinary hard point and segment checks resume.
- The original A-star polyline is preserved before ESDF/B-spline optimization.
  A rejected B-spline now falls back to that raw validated polyline rather than
  to the optimizer-mutated copy.
- Added throttled diagnostics for endpoint-bridge rejection, B-spline sample
  rejection, and linear waypoint/sample rejection, including position and
  clearance. Manager logs now distinguish `no safe A* path` from `A* path, but
  trajectory generation rejected it`; the final summary no longer calls every
  conversion failure `unreachable`.
- Verified with `cmake --build build/coverage_searching_pkg --parallel 2`; both
  `coverage_search_lib` and `coverage_search_node` build successfully.

## 2026-07-10 Residual-frontier Revisit Fallback

> Superseded on 2026-07-13: the completed-goal 0.8 m pool and its state were
> removed completely; current frontier viewpoints now enter one common pool.

- Root cause of the `96.5%` hover was not A-star range or obstacle complexity.
  The diagnostic line showed `near_visited=12` with zero direct/A-star checks:
  every current viewpoint was discarded before reachability evaluation because
  it lay within `visited_goal_clearance=0.8m` of a previously completed goal.
- The `0.8m` rule is now a priority rule instead of a permanent hard filter.
  Current-frontier viewpoints outside completed-goal neighborhoods are checked
  first. If that pool has no reachable viewpoint, viewpoints near completed
  goals are retried through the same direct traversal, A-star, and final cost
  evaluation. This permits late residual frontiers to be revisited without
  changing the target cost or the `0.35m` safety boundary.
- Removed the `selectNextFrontier()` entry path that recorded an arbitrary
  still-active/abandoned `current_goal_` as visited. A goal enters
  `visited_goals_` only after the UAV reaches its position and aligns its
  measured yaw.
- The no-viewpoint diagnostic field is now named `near_visited` rather than
  `visited`, making clear that it counts candidate viewpoints in the revisit
  pool, not the number of completed goals.
- Added a throttled `No usable fresh viewpoint; retry ... viewpoints near
  completed goals` message so the fallback is visible in runtime logs.
- Verified with `cmake --build build/coverage_searching_pkg --parallel 2`; both
  `coverage_search_lib` and `coverage_search_node` build successfully.

## 2026-07-10 Reversible Occupancy And Robot-volume Consistency

- The repeated `astar_endpoint_blocked` failures were confirmed to share the
  exact start, not the frontier goals. Runtime logs reported the UAV start cell
  with `clearance=0.00`; once the UAV drifted into a cell with a validated
  bridge, A-star immediately succeeded. The target selector and long-range
  search were therefore not the cause.
- Root map defect: every depth endpoint previously changed a voxel directly to
  occupied, while ray traversal changed only unknown voxels to free and stopped
  at occupied voxels. A single noisy endpoint was consequently permanent and
  could poison the UAV's current three-layer XY planning column after that
  position had originally passed planning.
- Added a compact signed evidence value per voxel. Ray misses decrement it,
  depth/scan/local-cloud hits increment it, free is committed at `<=-1`, and
  occupied is committed at `>=2`, with bounded hysteresis. This makes stale
  occupied voxels reversible while requiring repeat support for a new
  obstacle. One-time global static clouds still use direct occupied writes.
- Measured rays now exclude their depth endpoint from free updates. They may
  provide miss evidence through stale occupied cells because a farther measured
  endpoint proves the ray is open. Synthetic full-FOV clearing rays may extend
  known free space but still stop at occupied and cannot erode it.
- Added an odometry-backed P450 body-volume invariant. Only the measured robot
  volume is directly marked free (`robot_clear_radius=0.22m`,
  `robot_clear_half_height=0.20m`); the external `0.35m` ESDF safety region is
  not cleared. A throttled warning reports any occupied voxels removed from the
  body volume, exposing persistent self-reflection or transform errors.
- `currentGoalUnsafe()` now uses the same three-layer `getDistance2D()` as
  A-star. Sub-`0.05m` segment checks no longer return safe without inspecting
  both endpoint cells and their hard clearance.
- Verified with `cmake --build build/coverage_searching_pkg --parallel 2`; both
  `coverage_search_lib` and `coverage_search_node` build successfully.

## 2026-07-13 Occlusion-aware No-return Clearing

- The dense `CoverageMap::raycastCameraFovFree()` pass previously emitted a
  maximum-range clearing ray in every sampled camera direction. A newly or
  sparsely observed wall voxel could therefore fail to stop a neighboring
  synthetic ray, creating false free space and frontier clusters behind it.
- No-return clearing remains necessary because Gazebo depth clouds contain no
  endpoint for open directions; deleting it would leave internal unknown holes
  and stale frontiers. OctoMap handles the same sensor limitation separately on
  its dedicated point-cloud topic.
- CoverageMap now follows the existing OctoMap angular-bin rule: every trusted
  real depth return blocks its bin and the eight neighboring bins, measured
  rays handle those directions, and maximum-range clearing rays are emitted
  only for unblocked/no-return bins. Synthetic rays still stop at occupied
  voxels and never weaken occupied evidence.
- The camera origin and mount parameters were intentionally left unchanged.
- Verified with `cmake --build build/coverage_searching_pkg --parallel 2`; both
  `coverage_search_lib` and `coverage_search_node` build successfully.

### Follow-up: horizontal nearest-return clipping

- The first angular-bin guard still treated yaw and pitch independently. A
  maximum-range clearing ray could therefore pass above, below, or between the
  sampled wall returns and create an XY frontier behind the wall.
- The Gazebo P450 model confirms the existing camera transform is correct:
  offset `(0.095, 0, 0)` and pitch `0.35 rad`; it was not changed.
- For the 2D coverage planner, every synthetic ray in one horizontal bearing
  is now clipped one map cell before that bearing's nearest non-ground depth
  return. Open bearings still clear to maximum range, and occupied voxels are
  never weakened.
- Verified with `cmake --build build/coverage_searching_pkg --parallel 2`; both
  `coverage_search_lib` and `coverage_search_node` build successfully.

### Follow-up: one evidence update per voxel per sensor frame

- The remaining wall penetration came from measured rays, not the camera
  transform or the synthetic FOV rays. Every point ray independently applied a
  miss to every traversed voxel, so hundreds of rays in one cloud could exhaust
  the occupancy hysteresis and turn a confirmed wall voxel free in one frame.
- Added a frame epoch stamp for occupancy evidence. Each voxel now accepts at
  most one miss and one hit per sensor frame; a hit cancels an earlier same-frame
  miss and takes priority over later misses. A wall therefore needs consistent
  misses over multiple frames to be removed, while stale occupancy remains
  reversible.
- Verified with `cmake --build build/coverage_searching_pkg --parallel 2`; both
  `coverage_search_lib` and `coverage_search_node` build successfully.

## 2026-07-13 Gazebo Target Sphere Removal

- `sitl_outdoor_1uav_P450.launch` loads `planning_test3.world`, not the open
  `swarm_planner.world`. The pink sphere was the explicit
  `target_point_1 -> model://target_point` include at the world origin.
- Removed `target_point_1` from `planning_test3.world`. The source target model
  has only a visual sphere and no collision geometry; it was unrelated to the
  RViz UAV marker publisher.

## 2026-07-13 MAVROS Environment Fix

- The simulator log reported `cannot launch node of type
  [mavros/mavros_node]: mavros`; `uav_control` consequently remained at
  `Waiting for connect PX4`, so arming and the RViz UAV marker both failed.
- Changed `load_prometheus()` in `~/.bashrc` to source
  `~/Prometheus/devel/setup.bash --extend`, preserving the previously loaded
  `~/prometheus_mavros` workspace.
- Verified that one `load_prometheus` call resolves both `mavros` and
  `prometheus_gazebo`, with both devel spaces in `CMAKE_PREFIX_PATH`.

## 2026-07-13 Fixed Known-space Completion And P/V/A Trajectory Control

### Fixed known-space ratio and strict completion

- Removed the free-space visitation metric and all `visited_buffer_` /
  `updateSensorCoverage()` state. The displayed progress is now the 2D known
  space ratio
  `1 - unknown_xy_cells / (grid_size_x * grid_size_y)`. A cell is known when
  the flight-height projection is free or occupied. The denominator is the
  configured exploration rectangle and never grows with observations.
- The old permissive completion rule (`coverage > 80%` or 15 s without a
  frontier) was removed. Completion now requires both
  `known_ratio >= completion_known_ratio` (default 0.99) and no frontier for a
  continuous 8 s. Exceeding the replan counter no longer forces a false
  finish.
- If no normal frontier exists below the threshold, a full residual scan runs
  every 2 s with only the cluster shape relaxed (at least two connected XY
  frontier cells); the free/unknown definition, occlusion rules, viewpoint
  safety, and A* checks stay unchanged. If even that finds no usable target,
  the UAV holds position and scans yaw at 0.6 rad/s. Logs distinguish
  `Relaxed residual-frontier full scan` from `Residual recovery scan`.
- `coverage_search.launch` now exposes map origin and size as launch arguments.
  Its defaults match `samplemaze.world`: origin `(-9.6,-9.6,-0.5)` and size
  `(19.2,19.2,3.0)`, kept inside the outer walls. A different world requires
  correct ROI metadata, but no C++ change or rebuild; for the original 40 m
  maze use `map_origin_x:=-19.6 map_origin_y:=-19.6 map_size_x:=39.2
  map_size_y:=39.2` on the roslaunch command.

### Target selection

- Removed `visited_goal_clearance`, `visited_goals_`, and the completed-goal
  revisit pool. A viewpoint is no longer demoted merely because it is within
  0.8 m of an earlier completed target.
- The final target cost remains a direct sum, not a maximum:
  `1.25 * geodesic_path / max_vel + vel_align_weight * direction_angle +
  yaw_angle / max_yaw_rate`. The direction term is enabled only above 0.1 m/s;
  `vel_align_weight` remains 1.5 as previously requested.
- Each frontier contributes at most six viewpoints. Candidates are sorted by
  Euclidean distance only to schedule expensive reachability checks. The
  primary pass inspects at most 48 candidates, runs at most 16 blocked-candidate
  A* searches (150 ms each, 1.8 s pool budget), and computes the final summed
  cost with geodesic length for every successful candidate. Only when this
  produces no reachable candidate, the rescue pass rechecks the nearest 12
  with at most three 250 ms A* searches and an 800 ms pool budget.

### Smooth geometry and native trajectory control

- Full-path cubic B-spline remains the first choice. If it violates free-space,
  segment, or 0.35 m ESDF hard-clearance checks, the raw validated A* path is
  recursively split. Safe cubic pieces are stitched and their joins receive up
  to two validated Chaikin smoothing passes. A straight sub-piece is used only
  for the final two-point safety case; logs report spline piece count, straight
  safety piece count, join smoothing, and point count.
- The final geometry receives a forward/backward speed profile constrained by
  `max_vel`, `max_acc`, local curvature, stopping at the goal, and reduced speed
  inside the 0.7 m preferred-clearance band. Finite-difference acceleration is
  clamped to `max_acc`, and a runtime consistency assertion checks equal,
  finite position/velocity/acceleration/yaw arrays.
- Normal execution now publishes `UAVCommand::TRAJECTORY` with position,
  velocity, and acceleration together. Goal yaw alignment and emergency holds
  intentionally remain zero-feedforward position control.
- Verified twice with
  `catkin_make --source Modules/coverage_searching_pkg --build build/coverage_searching_pkg -j4`;
  `coverage_search_lib` and `coverage_search_node` both build successfully.

## 2026-07-14 Terminal Braking Safety

- Root cause of target overshoot was the terminal state transition, not the
  B-spline geometry alone. The trajectory allowed 2.8 m/s and assumed an ideal
  4.0 m/s^2 stop, then declared a viewpoint complete from position and yaw
  without checking measured velocity. If yaw was already aligned, the goal
  could be released while the UAV was still moving quickly.
- Reduced the existing trajectory limits to 1.0 m/s and 0.8 m/s^2 and tightened
  the position tolerance from 0.5 m to 0.25 m. No second controller or duplicate
  speed-limit parameter was added.
- Execution now enters zero-feedforward `XYZ_POS` braking at
  `v_actual^2 / (2 * max(0.3, max_acc)) + 0.30 m`. A viewpoint is complete only
  when distance is at most 0.25 m, yaw error at most 0.15 rad, and measured XY
  speed at most 0.15 m/s.
- The final P/V/A sample explicitly has zero velocity and zero acceleration.
  The `Terminal braking` log exposes distance, measured speed, braking-entry
  distance, and yaw error for simulation tuning.
- Verified with
  `catkin_make --source Modules/coverage_searching_pkg --build build/coverage_searching_pkg -j4`;
  `coverage_search_lib` and `coverage_search_node` build successfully.

## 2026-07-14 Coupled Yaw Trajectory, Stall Recovery, And Runtime Clearance

### Why the UAV could stop while the planner still looked busy

- The old dwell-timeout branch advanced `traj_idx_` even when the measured UAV
  pose had not reached that trajectory point. Repeated timeouts could therefore
  consume the displayed trajectory while the physical UAV stayed behind; the
  program and vehicle were no longer executing the same state.
- Timeout no longer skips any point. It sends a zero-feedforward position hold,
  keeps the same frontier goal, clears only the stale trajectory, and replans
  from the latest measured pose. The distinguishing log is `Trajectory tracking
  stalled ... Replan from measured pose`.
- The final trajectory sample is never advanced by ordinary index logic. Goal
  completion remains the single measured-state condition: position within
  0.25 m, yaw error within 0.15 rad, and XY speed no more than 0.15 m/s.
- Controller lookahead now starts with no assumed-safe point. If no direct safe
  chord exists from the measured UAV pose to a future trajectory sample, the UAV
  holds and replans instead of publishing a point across a wall. The log is `No
  safe controller chord ... stop and replan same goal`.

### Collision margin and real stopping motion

- A 0.35 m centre-to-obstacle threshold left too little margin for the P450 body,
  map discretization, command latency, and tracking error. Trajectory samples,
  sample-to-sample segments, measured-pose-to-command chords, and frontier
  viewpoint occupied clearance now use 0.55 m. The preferred ESDF repulsion band
  remains at least 0.70 m and is automatically larger when required by the hard
  clearance.
- Execution additionally projects the measured horizontal velocity over
  `v^2 / (2 * max(0.3, max_acc)) + 0.30 m`. If this physical stopping corridor
  violates free space or the 0.55 m clearance, it commands a hold and replans.
  The distinguishing log is `Dynamic braking guard`.
- The controller chord is checked as a continuous sampled segment, not only as
  safe discrete B-spline points. This addresses the corner-cutting mechanism of
  a position controller targeting a future sample.

### Position and viewpoint yaw are now one trajectory

- Removed the independent candidate/DP yaw planner. It could select headings
  unrelated to the final viewpoint and was not the requested coupled pose
  trajectory.
- The shortest unwrapped yaw change from measured start yaw to
  `current_goal_yaw_` is now fitted by a clamped scalar cubic B-spline. Its sample
  parameter is the same normalized path progress used by the position
  trajectory. Thus every published trajectory sample contains synchronized
  position, velocity, acceleration, and yaw; the first yaw equals the measured
  start yaw and the last equals the selected viewpoint yaw exactly. The log is
  `Coupled yaw B-spline`.
- `UAVCommand::TRAJECTORY` already passes `yaw_ref` in both the PX4-origin and
  built-in-controller branches. The PX4-origin forwarding path previously
  masked acceleration, however. Its existing setpoint sender now forwards
  P/V/A+yaw in one `mavros_msgs::PositionTarget`; no controller selection was
  changed.

### Verification

- `catkin_make --source Modules/coverage_searching_pkg --build build/coverage_searching_pkg -j4`
  completed successfully.

## 2026-07-14 Hard Occlusion Invariant For All Map Rays

- This section supersedes the earlier reversible-occupancy rule that allowed a
  farther measured endpoint to weaken and pass through an occupied voxel.
- The remaining wall penetration was in the shared `CoverageMap::raycastFree()`
  policy, not the camera transform or frontier classifier. Its
  `clear_occupied` argument defaulted to `true`, and both measured depth rays
  and scan rays used that default. On meeting an occupied voxel they therefore
  applied miss evidence and continued clearing voxels behind it. Only the
  synthetic camera-FOV rays had previously passed `false`.
- Removed that split policy. Confirmed occupied voxels are now hard occluders
  for measured depth rays, scan rays, and supplementary no-return FOV rays:
  the blocking voxel is not weakened and ray traversal returns immediately, so
  no voxel behind it can receive free evidence from that ray.
- This intentionally prefers false-occupied retention over false-free space
  behind a real obstacle. The measured P450 body-volume clearing remains the
  only direct exception and is spatially limited to the robot body itself.
- Added the throttled runtime log `Occlusion guard stopped ray at occupied
  voxel ... cells behind it were not cleared` so a blocked ray is distinguishable
  from missing depth data or frontier filtering.
- `catkin_make --source Modules/coverage_searching_pkg --build build/coverage_searching_pkg -j4`
  completed successfully.
- `catkin_make --source Modules/uav_control --build build/uav_control -j4`
  completed successfully. Only pre-existing MAVLink packed-member and utility
  warnings were emitted.

## 2026-07-14 前沿版本优先与 De Boor 连续滚动规划

> 本节替代上一节“进度 80% 或剩余 2 秒触发”和离散轨迹点预测交接状态的描述。

### 前沿簇版本与分级选择

- `FrontierFinder` 每次实际消费地图更新 AABB 时递增 `update_generation_`；新生成、
  被局部地图改变后重新生成以及 PCA 拆分出的簇都携带 `changed_generation`。
- 目标选择先只评估最近两个更新周期（当前代和前一代）内新增或变化的簇。
  这一级没有任何可达视点时，才回退到其余旧前沿簇；两级内部仍执行原有的
  直线可达检查、A* 测地距离和三项时间瓶颈代价，不用欧氏距离替代最终代价。
- 新日志 `Frontier generation priority` 会打印当前版本、两级视点数量以及实际采用
  `recent-two` 还是 `global-fallback`，用于判断远处旧簇是否抢占了近处新簇。

### 持续监听而非一次性预规划

- 删除固定“剩余时间不超过 2 秒”触发；滚动监听从旧轨迹进度 60% 开始。
- 每个前沿版本最多启动一次后台规划。失败后继续执行旧轨迹，不忙等、不重复使用
  同一快照；下一次前沿增量更新到来时自动使用最新版本重试。若线程运行期间又产生
  多个版本，线程结束后直接尝试当时最新版本。
- 成功得到安全后续轨迹后停止重试并等待交接；直到 95% 仍没有满足条件的后续轨迹，
  保留原有终点制动和停车逻辑。

### 时间参数 B 样条与解析交接状态

- 位置和展开后的偏航共同使用带真实秒制节点向量的三次钳制 B 样条。
  De Boor 算法直接求位置/偏航；对控制点和节点向量解析求导后，再由 De Boor 求
  速度、加速度、偏航角速度和偏航角加速度，不再用相邻离散点差分预测交接状态。
- 新轨迹首三个控制点编码交接处的 P/V/A 与偏航/角速度/角加速度，末三个控制点
  重合于最终视点。因此安全后续轨迹在数学上与旧轨迹保持 C2 状态连续；没有后续
  轨迹时，新轨迹自身以零速度、零加速度到达最终视点。
- 交接候选只在旧轨迹 85%～95% 内选取，并同时要求：距旧视点不超过 0.60 m
  （已进入旧视点观测邻域）、偏航误差不超过 0.15 rad、水平速度不低于 0.10 m/s。
  偏航 B 样条在约 80% 前完成目标朝向，以便观测和连续飞越条件能够同时成立。
- 每条时间 B 样条按约 0.1 秒用解析 P/V/A/yaw 采样，逐段经过 free、ESDF 净空、
  连线碰撞、最大速度、最大加速度和最大偏航角速度校验。普通轨迹失败时保留已通过
  安全检查的旧离散轨迹，但禁用该条轨迹的滚动交接；滚动后续轨迹失败则直接拒绝，
  不允许牺牲连续性或安全性。

### 关键日志

- `Time B-spline accepted`：输出时长、控制点数、采样数和速度/加速度/偏航角速度峰值。
- `Rolling monitor`：输出启动规划时的轨迹进度、前沿版本和活动轨迹版本。
- `Rolling trajectory ready: DeBoor handoff`：输出解析交接时刻、离散执行索引和新目标。
- `No safe successor in frontier generation`：本版失败，旧轨迹继续并等待更新。

### Verification

- `catkin_make --source Modules/coverage_searching_pkg --build build/coverage_searching_pkg -j4`
  completed successfully after the frontier-generation and time-B-spline changes.

## 2026-07-14 Bottleneck Target Cost And Short-Trajectory Rolling Update（历史）

> This section supersedes the earlier `1.25 + 1.5 + yaw` summed-cost and fixed
> 60% rolling-trigger descriptions. Its 80%/2-second rolling trigger was later
> superseded by the De Boor rolling-planning section immediately above.

### RViz OctoMap display

- The OctoMap RViz plugin group is restored; OctoMap mapping and all server
  topics remain unchanged.
- The binary `OccupancyMap` display is retained but disabled because the saved
  RViz core crashed inside its binary-message handler. The plugin's full-map
  `OccupancyGrid` display is enabled on `/uav1/octomap_full`. The standard
  `/projected_map` display remains enabled as an independent 2D view.

### Equal-weight bottleneck target cost

- The final cost is now
  `max(geodesic_path/max_vel, direction_angle/max_yaw_rate,
  yaw_angle/max_yaw_rate)`. All three terms have weight 1 and units of seconds;
  they are no longer summed.
- For an A* route, motion-direction change uses the first nonzero segment of the
  geodesic path instead of the straight line from UAV to viewpoint. Directly
  traversable routes still use the direct segment.
- Every sampled viewpoint is admitted to the global Euclidean scheduling pool;
  the former per-cluster top-six visibility truncation is removed. The existing
  global limits (48 ordinary, 24 rolling) still bound expensive reachability
  checks.
- The best-target log prints all three components as
  `Best viewpoint bottleneck=[path, direction, yaw]`.

### Viewpoint distance and rolling trigger

- Frontier-ring sampling is `1.2-1.6 m`. Normal ring candidates now enforce
  `min_candidate_dist` from the UAV; previously only the emergency linear
  fallback enforced it.
- Rolling preparation starts when either trajectory progress reaches 80% or
  estimated remaining execution time is at most 2.0 s. Remaining time is
  estimated from the existing segment lengths and speed profile, so trajectories
  with at least three samples can start preparing immediately when short.
- The old 0.8 m rolling exclusion is removed. Only a viewpoint within 0.10 m of
  the exact old goal is treated as the same numerical target.
- A continuous handoff still requires a safe successor, a nonzero-speed state
  near the old viewpoint, planned old-view yaw error no more than 0.15 rad, and
  acceptable measured P/V/yaw error. With an accepted successor, terminal
  position braking is skipped; without one, the existing safe stop remains.

### Verification

- `catkin_make --source Modules/coverage_searching_pkg --build build/coverage_searching_pkg -j4`
  completed successfully.

## 2026-07-14 视点硬遮挡收益与真正的时间轨迹交接

### 视点收益不再穿墙

- 根因不是前沿分类器，而是收益计算和地图更新使用了三套不同的射线逻辑。
  `countVisibleCells()` 按一个栅格步长采样，`countVisibleUnknown()` 甚至按
  `max(1.5 * resolution, 0.30 m)` 跳跃；薄墙可能落在相邻采样点之间而被跳过。
- `countVisibleUnknown()` 原来每条射线遇到第一个未知格就停止，因此统计值实质上是
  “包含未知的采样方向数”，并非该位置和朝向真正可见的未知栅格数量。
- 现在地图清空、前沿簇可见数和未知收益共用 `CoverageMap::traceRay()` 的体素 DDA
  遍历。射线按实际经过的栅格逐格访问，遇到第一个 occupied 立即硬终止：阻挡格不被
  削弱，墙后 free/unknown 既不更新地图，也不进入视点收益。
- 未知收益会收集每条视锥采样射线上、首个障碍物之前的全部唯一 unknown 地址，
  再去重计数；不会再把墙后的未知区域作为高收益目标。

### De Boor 从“仅用于生成”改为“实际控制时钟”

- 根因是旧实现虽然生成了时间 B 样条并用 De Boor 计算交接状态，但执行器仍以
  “实测位置靠近离散点后才推进 `traj_idx_`”驱动，交接也由离散索引触发。因此数学上
  连续的曲线没有成为控制器真正执行的参考，容易表现为旧轨迹结束、惯性滑行、再突然
  执行新轨迹。
- 时间 B 样条有效时，10 Hz 控制循环现在直接以
  `now - traj_start_time_` 为参数，通过 De Boor 解析计算同一时刻的位置、速度、
  加速度和偏航并发布 `TRAJECTORY` 指令。离散采样点只用于 RViz 和安全预检。
- 滚动规划进度、85%～95% 交接窗口和交接触发全部改用真实轨迹时间；到达
  `handoff_time` 后，新轨迹从预先编码的同一 P/V/A/yaw 状态的 `t=0` 开始执行。
  超过交接时刻 0.20 s（或两个采样周期）仍未成功切换会取消该结果，不允许迟到后
  突然切轨迹。
- 普通（非滚动）轨迹建立时间样条时，起始速度只保留沿首段安全路径的前向分量，
  避免横向实测速度把首段控制点拉向障碍物而导致时间样条反复拒绝；滚动交接仍严格
  保留 De Boor 预测的完整 P/V/A 状态。
- 新日志 `Timed De Boor execution` 可确认当前确实由时间曲线执行，并显示时间、进度、
  速度、加速度和待交接状态；拒绝日志现在附带 `last_reason`，可区分 ESDF、连续段
  净空、速度、加速度和偏航角速度限制。

### Verification

- `cmake --build build/coverage_searching_pkg -j4` completed successfully;
  `coverage_search_lib` and `coverage_search_node` both linked successfully.

## 2026-07-14 隔墙视点候选根因修正

- RViz 中洋红色球是实际目标、洋红色箭头是实际目标偏航；橙色轨迹绕墙到达该点，
  说明错误发生在视点候选生成，而不是 A* 不会绕路。
- 根因一：旧代码在 `visib` 低于门槛时只增加 `reject_vis` 计数，却没有退出候选生成；
  所以看不到所属前沿簇的视点仍被保存，`unknown_gain` 还可能继续提高其排序分数。
  现在所属前沿的直接可见数量低于 `min_visib_num` 会立即淘汰，未知收益不能救回
  隔墙视点；保存候选时另有运行期断言守住这个不变量。
- 根因二：三维占据点云在某一个高度层可能有小孔。传感器建图仍保持真实三维射线，
  但用于二维覆盖搜索的前沿可见性和未知收益改为保守的占据列遮挡：只要该 XY 栅格列
  在飞行高度范围存在 occupied，收益射线便立即终止。
- 视点朝向只用无障碍直视的前沿单元计算；被墙挡住的簇中心不再生成中心偏航候选。
- `cmake --build build/coverage_searching_pkg -j4` completed successfully.

## 2026-07-15 官方基线恢复与覆盖搜索隔离

- 清理前完整状态保存在 Git 分支 `backup/pre-cleanup-20260715`，指向提交
  `9f2d6b4e`。当前工作分支 `coverage-search-clean` 从官方
  `origin/main` 的 `ce71be9e` 建立，因此可以随时对照或回退。
- 官方 `Modules/searching_pkg` 子模块及 `.gitmodules` 已恢复。自研覆盖搜索包迁移到
  `Modules/coverage_searching_pkg`，ROS 包名仍为 `prometheus_coverage_search`，
  现有 launch、话题和节点名称不变。
- 官方 `takeoff_land_P450.sh`、`sitl_outdoor_1uav_P450.launch`、通用深度建图 launch、
  P450 模型和控制参数均保持官方版本。覆盖搜索场景使用独立的
  `sitl_coverage_search_1uav_P450.launch`，只组合 `samplemaze.world`、D435i、
  点云预处理、OctoMap 和 `maze_mapping.rviz`。
- 新脚本 `Scripts/simulation/searching_pkg/coverage_search_P450.sh` 自行加载 ROS、当前
  工作区和 `${HOME}/prometheus_px4` 环境；不再依赖修改官方起降脚本。覆盖搜索节点由
  `Scripts/simulation/searching_pkg/coverage_search_node.sh` 单独启动。
- 对官方源码唯一保留的功能修改位于 `prometheus_uav_control`：仅在
  `UAVCommand::TRAJECTORY` 分支把位置、速度、加速度和偏航一并转发给 PX4。起降、
  定点、速度及其他控制分支保持官方逻辑。
- 覆盖搜索包使用本地 `coverage_console_colors.h`，不再为了拆分源码而修改官方
  `Modules/common/include/printf_utils.h`。

### 启动顺序

1. `Scripts/simulation/searching_pkg/coverage_search_P450.sh`
2. `Scripts/simulation/NO_RC/arm_and_command.sh`
3. `Scripts/simulation/searching_pkg/coverage_search_node.sh`

### 验证

- `catkin_make --source Modules/coverage_searching_pkg --build build/coverage_searching_pkg -j4`
  编译成功，`coverage_search_lib`、`coverage_search_node` 和
  `depth_cloud_downsample_node` 均已链接。
- `cmake --build build/uav_control -j4` 编译成功，`uav_control_main` 已链接。
- 两个新脚本通过 `bash -n`；自定义 launch、覆盖搜索 launch 和 `samplemaze.world`
  通过 XML 校验；`rospack` 能把 `prometheus_coverage_search` 定位到隔离后的目录。

## 2026-07-15 专用启动脚本 MAVROS 环境修正

- 现象：Gazebo、PX4 和 P450 模型正常启动，但控制终端报告
  `Waiting for connect PX4`，随后解锁失败。
- 根因：专用脚本重新加载 ROS 环境时遗漏了
  `${HOME}/prometheus_mavros/devel/setup.bash`，日志中的直接错误是
  `cannot launch node of type [mavros/mavros_node]`；这与 PX4 端口和模型无关。
- 修正：脚本按 ROS、定制 MAVROS、Prometheus `--extend`、PX4 的顺序加载环境，并在
  启动前检查 MAVROS setup 是否存在。NO_RC 场景同时通过 `joy_enable:=false` 关闭未使用
  的手柄节点，避免 `/dev/input/js0` 警告。
