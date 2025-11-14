#!/usr/bin/env python3
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

rospy.init_node('panda_fixed_position_loop')
pub = rospy.Publisher('/panda_arm_controller/command', JointTrajectory, queue_size=1)
rate = rospy.Rate(50)  # 50 Hz

joint_names = [
    "panda_joint1", "panda_joint2", "panda_joint3",
    "panda_joint4", "panda_joint5", "panda_joint6",
    "panda_joint7"
]

traj = JointTrajectory()
traj.joint_names = joint_names

point = JointTrajectoryPoint()
point.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
point.time_from_start = rospy.Duration(0.1)  # short duration for controller
traj.points.append(point)

rospy.sleep(1.0)  # wait for publisher

while not rospy.is_shutdown():
    pub.publish(traj)
    rate.sleep()
