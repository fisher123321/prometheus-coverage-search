std_msgs/Header header
uint32 source_uav_id
uint32 map_epoch
# A candidate offer is immutable for this sequence. The acknowledgement binds
# both coordinators to the same pair of offers before they form the union.
uint64 offer_sequence
uint64 peer_offer_sequence_ack
uint64 task_set_hash
uint64 uav1_state_sequence
uint64 uav2_state_sequence
# The positions matching the state sequences are frozen with the round.
# Both evaluators must use these exact A* starts, not their current local view.
geometry_msgs/Point uav1_position
geometry_msgs/Point uav2_position
# Coordinators exchange the complete UID list and a single best representative
# viewpoint per task. Full cells and all viewpoints are sent only after the
# remote owner is committed.
prometheus_two_uav_coverage_search/SwarmFrontier[] frontiers
