# Immutable frontier UID used as the task key; it is not regenerated on handoff.
uint64 task_id
uint32 frontier_source_uav_id
uint32 task_version
uint32 cluster_version
uint32 frontier_cell_count
uint32 winner_uav_id
# Assignment provenance is visualised in RViz.  Only DISTRIBUTED is the
# confirmed result of an expired-frontier allocation round.
uint8 ALLOCATION_LOCAL_RESERVATION=1
uint8 ALLOCATION_DISTRIBUTED=2
uint8 allocation_state
float32 cost
time lease_expire_time
geometry_msgs/Point centroid
geometry_msgs/Point box_min
geometry_msgs/Point box_max
# Copied from the source frontier; the committed bid viewpoint is index 0.
geometry_msgs/Pose[] candidate_viewpoints
