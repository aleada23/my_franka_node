#!/usr/bin/env python3
import rospy
from pose_publisher import PosePublisher
from data_listener import FrankaListener
from joints_pos_publisher import JointTrajectoryPublisher


import time
import numpy as np

from franka_actions.bt_manager import BehaviorTreeManager


if __name__ == "__main__":
    #object1_pose = [0.3, 0.0, 0.4, np.pi, 0, 0]
    rospy.init_node("trajectory_replay_node", anonymous=True)
    listener = FrankaListener()
    rospy.sleep(2)  # give nodes time to start
    pp = PosePublisher()
    jj = JointTrajectoryPublisher()
    q = listener.get_joint_positions()
    bt_builder = BehaviorTreeManager()
    #bt_definition = ["Sequence", ["OpenGripper", {}], ["MovePose", {"target_pose": object1_pose}], ["CloseGripper", {}]]
    #bt_definition = ["Sequence",["MoveJoints", {"target_pos": [0.0, -0.5, 0.0, -2.5, 0.0, 2.0, 0.0]}]]
    home = [0, 0, 0, -1.57079, 0, 1.57079, -0.7853]
    object1_pose = [0.7, 0.0, 0.35, 0, np.pi/2, 0]
    offset_pose = [0.6, 0.0, 0.35, 0, np.pi/2, 0]
    offset_lift_pose = [0.6, 0.0, 0.7, 0, np.pi/2, 0]
    above_table = [0.4, 0.2, 0.5, -np.pi, 0, 0]
    #bt_definition = ["Sequence", ["Sequence", ["MovePose", {"target_pose": offset_pose, "pose_controller" : pp}],["OpenGripper", {}],["MovePose", {"target_pose": object1_pose, "pose_controller" : pp}],["CloseGripper", {}],["MeasureGripperOpnening", {}],["MovePose", {"target_pose": offset_lift_pose, "pose_controller" : pp}],["MeasureAppliedHE", {}],["MovePose", {"target_pose": object1_pose, "pose_controller" : pp}],["OpenGripper", {}]],["Sequence",["MovePose", {"target_pose": above_table, "pose_controller" : pp}],["MoveDownUntillContact", {"target_pose": above_table, "pose_controller" : pp}],["MeasureGripperSites", {}]]]
    #bt_definition = ["Sequence", ["Sequence", ["MovePose", {"target_pose": offset_pose, "pose_controller" : pp}], ["MoveJoints", {"target_pos": home, "joints_controller" : pp}]]]
    bt_definition = ["Sequence", ["Sequence", ["MoveJoints", {"target_pos": home, "joints_controller" : jj}], ["MovePose", {"target_pose": offset_pose, "pose_controller" : pp}]]]
    #bt_definition =["Sequence",["MovePose", {"target_pose": object1_pose, "pose_controller" : pp}]]
    tree = bt_builder.build_tree(bt_definition)
    bt_builder.print_tree()
    
    while True:
        q = listener.get_joint_positions()
        bt_builder.tick(display_tree = False)
