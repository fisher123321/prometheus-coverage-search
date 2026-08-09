# Immutable frontier UID; task ownership may change but this value never does.
uint64 task_id
uint32 frontier_source_uav_id
uint32 chunk_id
uint32 cluster_version
uint32 frontier_cell_count
geometry_msgs/Point centroid
geometry_msgs/Point box_min
geometry_msgs/Point box_max
# Complete frontier geometry. It is transferred with the task so the new
# owner can maintain the same cluster instead of reconstructing a new one.
geometry_msgs/Point[] frontier_cells
# Before assignment this contains only the best representative viewpoint.
# A source sends all viewpoints only with the complete post-assignment transfer.
geometry_msgs/Pose[] candidate_viewpoints
float32 information_gain
uint32 reservation_owner_uav_id
time reservation_expire_time
