uint64 task_id
uint32 frontier_source_uav_id
uint32 task_version
uint32 cluster_version
uint32 frontier_cell_count
uint32 winner_uav_id
float32 cost
time lease_expire_time
geometry_msgs/Point centroid
geometry_msgs/Point box_min
geometry_msgs/Point box_max
# Copied from the source frontier; index 0 is the A* bidding representative.
geometry_msgs/Pose[] candidate_viewpoints
