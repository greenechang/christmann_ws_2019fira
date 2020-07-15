import rospy
from op3_ros_utils import Robot

rospy.init_node("set_ctrl")

op3 = Robot()

upper_joints = ["r_sho_pitch", "l_sho_pitch", "r_sho_roll", "l_sho_roll", "r_el", "l_el",
    "head_pan", "head_tilt"]
lower_joints = ["r_hip_yaw", "l_hip_yaw", "r_hip_roll", "l_hip_roll", "r_hip_pitch",
    "l_hip_pitch", "r_knee", "l_knee", "r_ank_pitch", "l_ank_pitch", "r_ank_roll",
    "l_ank_roll"]

op3.setJointsControlModule(upper_joints, ["base_module"])
op3.setJointsControlModule(lower_joints, ["online_walking_module"])
