# Command manager send msg to locomotion unit.
std_msgs/Header header
# robot availible fsm: transform_up，transform_down，loco，joint_pd，car，rl_*(1, 2, 3)
string fsm_mode 

# Robot body pose
geometry_msgs/Pose pose

# Speed control of robot chassis 
geometry_msgs/Twist twist