#!/usr/bin/env python3
import rospy
from pose_publisher import PosePublisher
from data_listener import FrankaListener

import time
import numpy as np
if __name__ == "__main__":
    rospy.init_node("trajectory_replay_node", anonymous=True)
    listener = FrankaListener()
    rospy.sleep(2)  # give nodes time to start
    pp = PosePublisher()
    q = listener.get_joint_positions()
    print("Joint positions:", q)
    # Pose 1
    pp.set_pose([0.3, 0.0, 0.4], [np.pi, 0, 0])
    time.sleep(5)
    q = listener.get_joint_positions()
    print("Joint positions:", q)

    # Pose 2
    pp.set_pose([0.3, 0.1, 0.4], [np.pi, 0, 0])
    time.sleep(5)
    q = listener.get_joint_positions()
    print("Joint positions:", q)

    # Pose 3
    pp.set_pose([0.35, 0.05, 0.45], [np.pi, 0, 0])
    time.sleep(5)
    q = listener.get_joint_positions()
    print("Joint positions:", q)
