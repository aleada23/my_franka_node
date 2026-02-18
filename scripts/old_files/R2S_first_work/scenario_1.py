#!/usr/bin/env python3
import rospy
from pose_publisher import PosePublisher
from data_listener import FrankaListener
from joints_pos_publisher import JointTrajectoryPublisher

import numpy as np

from franka_actions.bt_manager import BehaviorTreeManager


if __name__ == "__main__":
    rospy.init_node("trajectory_replay_node", anonymous=True)
    listener = FrankaListener()
    rospy.sleep(2)
    pp = PosePublisher()
    jj = JointTrajectoryPublisher()

    q = listener.get_joint_positions()
    bt_builder = BehaviorTreeManager()

    #rospy.sleep(2) 

    #define poses
    home_pose = np.array([0.0, -0.785, -0.0, -2.356, 0.0, 1.571, 0.785])
    table_top_approach_pose = np.array([0.4, 0.0, 0.35, np.pi, 0, 0])
    #blue_bottle_pose = np.array([0.4, 0.0, 0.35, np.pi, 0, 0])
    blue_bottle_pose = np.array([0.8, 0.0, 0.35, 0, np.pi/2, 0])
    green_bottle_pose = np.array([0.4, 0.2, 0.35, np.pi, 0, 0])
    pink_bottle_pose = np.array([0.4, -0.2, 0.35, np.pi, 0, 0])
    offset_pose = np.array([0.0, 0.0, -0.1, 0.0, 0.0, 0.0])
    offset_lift_pose = np.array([0.0, 0.0, 0.1, 0.0, 0.0, 0.0])
    offset_slide_x = np.array([-0.1, 0.0, 0.0, 0.0, 0.0, 0.0])
    blue_block = [0.7, 0.0, 0.1, np.pi, 0, 0]
    red_block = [0.7, 0.0, 0.2, np.pi, 0, 0]
    temp_pose = [0.7, -0.2, 0.1, np.pi, 0, 0]


    #scenario-1, 1 borraccia sul tavolo
    bt_1 = ["Sequence",["MoveJoints", {"target_pos": home_pose, "joints_controller" : jj}], ["Sequence", ["MovePose", {"target_pose": table_top_approach_pose, "pose_controller" : pp}], ["MoveDownUntillContact", {"target_pose": table_top_approach_pose, "pose_controller" : pp}], ["MeasureGripperSites", {}], ["MoveJoints", {"target_pos": home_pose, "joints_controller" : jj}]], ["Sequence",["MovePose", {"target_pose": blue_bottle_pose-offset_pose, "pose_controller" : pp}], ["OpenGripper", {}], ["MovePose", {"target_pose": blue_bottle_pose, "pose_controller" : pp}], ["CloseGripper", {}], ["MovePose", {"target_pose": blue_bottle_pose+offset_lift_pose, "pose_controller" : pp}], ["MeasureMassWithTorque", {}], ["MovePose", {"target_pose": blue_bottle_pose, "pose_controller" : pp}], ["OpenGripper", {}], ["MoveJoints", {"target_pos": home_pose, "joints_controller" : jj}]]]
	
    #scenario-1, 1 boraccia e altezza del tavolo nota[["MoveJoints"
    bt_2 = ["Sequence",["MoveJoints", {"target_pos": home_pose, "joints_controller" : jj}], ["Sequence",["MovePose", {"target_pose": blue_bottle_pose-offset_pose, "pose_controller" : pp}], ["OpenGripper", {}], ["MovePose", {"target_pose": blue_bottle_pose, "pose_controller" : pp}], ["CloseGripper", {}], ["MovePose", {"target_pose": blue_bottle_pose+offset_lift_pose, "pose_controller" : pp}], ["MeasureMassWithTorque", {}], ["MovePose", {"target_pose": blue_bottle_pose, "pose_controller" : pp}], ["OpenGripper", {}], ["MoveJoints", {"target_pos": home_pose, "joints_controller" : jj}]]]
		
    #scenario-1, 1 borraccia sul tavolo foto simulazione
    #bt_3 = [["Sequence",["MoveJoints", {"target_pos": home_pose, "joints_controller" : jj}]],["Sequence",["MovePose", {"target_pose": table_top_approach_pose, "pose_controller" : pp}],["MoveDownUntillContact", {"target_pose": table_top_approach_pose, "pose_controller" : pp}],["MeasureGripperSites", {}],["MoveJoints", {"target_pos": home_pose, "joints_controller" : jj}]],["Sequence",["MovePose", {"target_pose": blue_bottle_pose-offset_pose, "pose_controller" : pp}],["OpenGripper", {}]],["MovePose", {"target_pose": blue_bottle_pose, "pose_controller" : pp}],["CloseGripper", {}]],["MovePose", {"target_pose": blue_bottle_pose+offset_lift_pose, "pose_controller" : pp}],["MeasureMassWithTorque", {}],["MovePose", {"target_pose": blue_bottle_pose, "pose_controller" : pp}],["OpenGripper", {}]],["MoveJoints", {"target_pos": home_pose, "joints_controller" : jj}]],["MoveJoints", {"target_pos": home_pose, "joints_controller" : jj}]]]]
		
    #scenario-1, 1 boraccia e altezza del tavolo nota foto simulazione
    #bt_4 = [["Sequence",["MoveJoints", {"target_pos": home_pose, "joints_controller" : jj}]],["Sequence",["MovePose", {"target_pose": blue_bottle_pose-offset_pose, "pose_controller" : pp}],["OpenGripper", {}]],["MovePose", {"target_pose": blue_bottle_pose, "pose_controller" : pp}],["CloseGripper", {}]],["MovePose", {"target_pose": blue_bottle_pose+offset_lift_pose, "pose_controller" : pp}],["MeasureMassWithTorque", {}],["MovePose", {"target_pose": blue_bottle_pose, "pose_controller" : pp}],["OpenGripper", {}]],["MoveJoints", {"target_pos": home_pose, "joints_controller" : jj}]],["MoveJoints", {"target_pos": home_pose, "joints_controller" : jj}]]]]

    #scenario-2, 3 borracce sul tavolo
    bt_5 = ["Sequence",["MoveJoints", {"target_pos": home_pose, "joints_controller" : jj}], ["Sequence",["MovePose", {"target_pose": table_top_approach_pose, "pose_controller" : pp}], ["MoveDownUntillContact", {"target_pose": table_top_approach_pose, "pose_controller" : pp}], ["MeasureGripperSites", {}], ["MoveJoints", {"target_pos": home_pose, "joints_controller" : jj}], ["Sequence",["MovePose", {"target_pose": blue_bottle_pose-offset_pose, "pose_controller" : pp}], ["OpenGripper", {}], ["MovePose", {"target_pose": blue_bottle_pose, "pose_controller" : pp}], ["CloseGripper", {}], ["MovePose", {"target_pose": blue_bottle_pose+offset_lift_pose, "pose_controller" : pp}], ["MeasureMassWithTorque", {}], ["MovePose", {"target_pose": blue_bottle_pose, "pose_controller" : pp}], ["OpenGripper", {}], ["MoveJoints", {"target_pos": home_pose, "joints_controller" : jj}]], ["Sequence",["MovePose", {"target_pose": green_bottle_pose-offset_pose, "pose_controller" : pp}], ["OpenGripper", {}], ["MovePose", {"target_pose": green_bottle_pose, "pose_controller" : pp}], ["CloseGripper", {}], ["MovePose", {"target_pose": green_bottle_pose+offset_lift_pose, "pose_controller" : pp}], ["MeasureMassWithTorque", {}], ["MovePose", {"target_pose": green_bottle_pose, "pose_controller" : pp}], ["OpenGripper", {}], ["MoveJoints", {"target_pos": home_pose, "joints_controller" : jj}]], ["Sequence",["MovePose", {"target_pose": pink_bottle_pose-offset_pose, "pose_controller" : pp}], ["OpenGripper", {}], ["MovePose", {"target_pose": pink_bottle_pose, "pose_controller" : pp}], ["CloseGripper", {}], ["MovePose", {"target_pose": pink_bottle_pose+offset_lift_pose, "pose_controller" : pp}], ["MeasureMassWithTorque", {}], ["MovePose", {"target_pose": pink_bottle_pose, "pose_controller" : pp}], ["OpenGripper", {}], ["MoveJoints", {"target_pos": home_pose, "joints_controller" : jj}]], ["MoveJoints", {"target_pos": home_pose, "joints_controller" : jj}]]]
	
    #scenario-2, 3 borracce sul tavolo stima un solo colore (verde)
    bt_6 = ["Sequence",["MoveJoints", {"target_pos": home_pose, "joints_controller" : jj}], ["Sequence",["MovePose", {"target_pose": table_top_approach_pose, "pose_controller" : pp}], ["MoveDownUntillContact", {"target_pose": table_top_approach_pose, "pose_controller" : pp}], ["MeasureGripperSites", {}], ["MoveJoints", {"target_pos": home_pose, "joints_controller" : jj}], ["Sequence",["MovePose", {"target_pose": green_bottle_pose-offset_pose, "pose_controller" : pp}], ["OpenGripper", {}], ["MovePose", {"target_pose": green_bottle_pose, "pose_controller" : pp}], ["CloseGripper", {}], ["MovePose", {"target_pose": green_bottle_pose+offset_lift_pose, "pose_controller" : pp}], ["MeasureMassWithTorque", {}], ["MovePose", {"target_pose": green_bottle_pose, "pose_controller" : pp}], ["OpenGripper", {}], ["MoveJoints", {"target_pos": home_pose, "joints_controller" : jj}]], ["MoveJoints", {"target_pos": home_pose, "joints_controller" : jj}]]]

    #scenario-2, 3 borracce sul tavolo foto simulazione
    #bt_7 = None

    #scenario-2, 3 borracce sul tavolo stima un solo colore foto simulazione
    #bt_8 = None

    #scenario-3, friction param with unknown action
    bt_9 = ["Sequence",["MoveJoints", {"target_pos": home_pose, "joints_controller" : jj}],["Sequence",["MovePose", {"target_pose": blue_bottle_pose+offset_lift_pose, "pose_controller" : pp}],["OpenGripper", {}],["MovePose", {"target_pose": blue_bottle_pose, "pose_controller" : pp}],["CloseGripper", {}],["MovePose", {"target_pose": blue_bottle_pose+offset_lift_pose, "pose_controller" : pp}],["MeasureMassWithTorque", {}],["MovePose", {"target_pose": blue_bottle_pose, "pose_controller" : pp}],["Sequence",["Parallel",["MeasureAppliedHE", {}],["MovePose", {"target_pose": blue_bottle_pose+offset_slide_x, "pose_controller" : pp}]],["MovePose", {"target_pose": blue_bottle_pose, "pose_controller" : pp}]],["OpenGripper", {}]]]

    #esempio cubi
    bt_10= ["Sequence",["MoveJoints", {"target_pos": home_pose, "joints_controller" : jj}],["Sequence",["MovePose", {"target_pose":red_block-offset_pose, "pose_controller" : pp}],["OpenGripper", {}],["MovePose", {"target_pose":red_block, "pose_controller" : pp}],["CloseGripper", {}],["MovePose", {"target_pose":temp_pose, "pose_controller" : pp}],["OpenGripper", {}]],["Sequence", ["MovePose", {"target_pose":blue_block+offset_pose, "pose_controller" : pp}],["OpenGripper", {}],["MovePose", {"target_pose":blue_block, "pose_controller" : pp}],["CloseGripper", {}], ["MovePose", {"target_pose":blue_block+offset_lift_pose, "pose_controller" : pp}],["MeasureMassWithTorque", {}],["MovePose", {"target_pose":blue_block, "pose_controller" : pp}],["OpenGripper", {}]], ["Sequence", ["MovePose", {"target_pose":temp_pose-offset_pose, "pose_controller" : pp}],["OpenGripper", {}],["MovePose", {"target_pose":temp_pose, "pose_controller" : pp}],["CloseGripper", {}],["MovePose", {"target_pose":red_block, "pose_controller" : pp}],["OpenGripper", {}]],["MoveJoints", {"target_pos": home_pose, "joints_controller" : jj}]]

    tree = bt_builder.build_tree(bt_2)
    bt_builder.print_tree()

    while True:
        q = listener.get_joint_positions()
        bt_builder.tick(display_tree = False)
