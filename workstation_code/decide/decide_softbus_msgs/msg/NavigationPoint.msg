# A navigation point with planned pose, expected arrival time,
# as well as assigned formation type and parameters (for a 
# group). Task allocation, such as sensor tasking, could also 
# be added in the future.
#
# The header
std_msgs/Header header
# Planned pose
geometry_msgs/Pose pose
# Expected arrival time
time arrival_time
# Assigned formation type (could be extended in the future)
# 0: Single agent (not a group)
# 1: Linear formation
# 2: V formation
# 3: Swarm
int32 formation
# Formation-related parameters
int32[] parameters

