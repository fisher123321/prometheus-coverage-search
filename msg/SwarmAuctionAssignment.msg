std_msgs/Header header
uint32 source_uav_id
uint32 map_epoch
uint64 task_set_hash
uint64 uav1_state_sequence
uint64 uav2_state_sequence
uint64 assignment_hash
# false: locally calculated proposal. true: canonical result after both
# proposals were compared. A committed result must be echoed by both peers.
bool final_result
# Per-task assignment result. The selected viewpoint is moved to index 0 when
# the winning lease is committed, so planning and RViz use the bid viewpoint.
uint8 MODE_ASTAR=1
uint8 MODE_EUCLIDEAN_TIMEOUT=2
uint8 MODE_UNREACHABLE=3
uint64[] task_ids
uint32[] frontier_source_uav_ids
uint32[] cluster_versions
uint32[] owner_uav_ids
uint32[] viewpoint_indices
uint8[] assignment_modes
