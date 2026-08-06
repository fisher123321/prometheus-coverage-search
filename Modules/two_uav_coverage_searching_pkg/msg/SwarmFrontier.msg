uint64 task_id
uint32 frontier_source_uav_id
uint32 chunk_id
uint32 cluster_version
uint32 frontier_cell_count
geometry_msgs/Point centroid
geometry_msgs/Point box_min
geometry_msgs/Point box_max
# Ordered best-first.  Index 0 alone is used for the A* bid.
geometry_msgs/Pose[] candidate_viewpoints
float32 information_gain
uint32 reservation_owner_uav_id
time reservation_expire_time
