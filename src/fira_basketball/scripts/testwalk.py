import rospy
from op3_ros_utils import Robot

rospy.init_node("test_node")
rospy.sleep(1)

op3 = Robot()
op3.onlineWalkSetup(x=0.025, foot_dist=0.08)
op3.onlineWalkCommand(direction="right", start_leg="right", step_num=4, side_length=0.05)

rospy.spin()
