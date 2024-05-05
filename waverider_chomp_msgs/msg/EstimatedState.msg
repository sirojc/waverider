std_msgs/Header header

# Position
geometry_msgs/Vector3 pos_unfiltered
geometry_msgs/Vector3 pos_filtered

# Yaw
float32 yaw_unfiltered
float32 yaw_filtered

# Velocity
waverider_chomp_msgs/Vector2 lin_vel_unfiltered
waverider_chomp_msgs/Vector2 lin_vel_filtered

# Acceleration
waverider_chomp_msgs/Vector2 lin_acc_unfiltered
waverider_chomp_msgs/Vector2 lin_acc_filtered