std_msgs/Header header
uint32 source_uav_id
uint32 map_epoch
uint64 task_set_hash
uint64 uav1_state_sequence
uint64 uav2_state_sequence
# Coordinators exchange the complete UID list and a single best representative
# viewpoint per task. Full cells and all viewpoints are sent only after the
# remote owner is committed.
prometheus_two_uav_coverage_search/SwarmFrontier[] frontiers
