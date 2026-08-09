std_msgs/Header header
# Sender of this task-set digest. Both coordinators publish the same digest
# once their local and peer frontier advertisements describe the same set.
uint32 source_uav_id
uint32 map_epoch
uint64 task_set_hash
uint32 task_count
# Diagnostic state cut used by the local A* evaluator. It is recorded with a
# round but does not make unrelated state packets invalidate the task set.
uint64 uav1_state_sequence
uint64 uav2_state_sequence
