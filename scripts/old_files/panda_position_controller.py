#!/usr/bin/env python3

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def send_trajectory():
    # Initialize ROS node
    rospy.init_node('send_franka_trajectory', anonymous=True)

    # Publisher to the joint trajectory controller topic
    pub = rospy.Publisher(
        '/position_joint_trajectory_controller/command',
        JointTrajectory,
        queue_size=10
    )

    # Wait until the publisher is ready
    rospy.sleep(1)

    # Create JointTrajectory message
    traj = JointTrajectory()
    traj.joint_names = [
        'panda_joint1',
        'panda_joint2',
        'panda_joint3',
        'panda_joint4',
        'panda_joint5',
        'panda_joint6',
        'panda_joint7'
    ]

    # Create one trajectory point
    point = JointTrajectoryPoint()
    point.positions = [0.0, 0.3, 0.0, -2.1, 0.0, 2.0, 0.8]
    point.velocities = [0.0] * 7
    point.time_from_start = rospy.Duration(3.0)

    # Add the point to the trajectory
    traj.points.append(point)

    # Publish at 10 Hz (like -r 10 in rostopic pub)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub.publish(traj)
        rate.sleep()

if __name__ == '__main__':
    try:
        send_trajectory()
    except rospy.ROSInterruptException:
        pass
