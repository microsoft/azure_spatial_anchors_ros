# Azure Spatial Anchors ID (64-character UUID string) that was found.
string anchor_id
# The pose of the ASA anchor expressed in the world (odometry) coordinate frame.
# The "world" frame is defined as whatever the global/world frame that the
# camera poses are expressed in.
geometry_msgs/TransformStamped anchor_in_world_frame
