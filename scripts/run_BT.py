#!/usr/bin/env python3
import rospy
import numpy as np
from pose_publisher import PosePublisher
from data_listener import FrankaListener
from joints_pos_publisher import JointTrajectoryPublisher
from franka_actions.bt_manager import BehaviorTreeManager
import py_trees


if __name__ == "__main__":
    rospy.init_node("panda_bt_control_node", anonymous=True)
    
    listener = FrankaListener()
    pp = PosePublisher()            
    jj = JointTrajectoryPublisher()  
    rospy.sleep(1.0)                

    z_offset = np.array([0, 0, 0.1, 0, 0, 0])
    poses = {
        "home": [0, 0, 0, -1.57079, 0, 1.57079, -0.7853],
        "green_cube_0": [0.4, 0.0, 0.3, 0, np.pi, 0],
        "green_cube_0 + z_offset": ([0.5, 0.0, 0.3, 0, np.pi, 0] + z_offset),
        "temp_pose_1 + z_offset": [0.4, 0.0, 0.4, 0, np.pi, 0],
    }

    bt_json = {
        "type": "Selector",
        "children": [
            {"type": "Condition", "name": "is_at_home", "args": [poses["home"]]},
            {
                "type": "Sequence",
                "children": [
                    {
                        "type": "Selector",
                        "children": [
                            {"type": "Condition", "name": "is_grasped", "args": []},
                            {
                                "type": "Sequence",
                                "children": [
                                    {
                                        "type": "Selector",
                                        "children": [
                                            {"type": "Condition", "name": "is_gripper_open", "args": []},
                                            {"type": "Action", "name": "OpenGripper", "args": []}
                                        ]
                                    },
                                    {
                                        "type": "Parallel",
                                        "children": [
                                            {"type": "Condition", "name": "is_at_pose", "args": [poses["green_cube_0 + z_offset"]]},
                                            {"type": "Action", "name": "MovePose", "args": [poses["green_cube_0 + z_offset"]]}
                                        ]
                                    },
                                    {"type": "Action", "name": "CloseGripper", "args": []}
                                ]
                            }
                        ]
                    },
                    {
                        "type": "Parallel",
                        "children": [
                            {"type": "Condition", "name": "is_at_home", "args": [poses["home"]]},
                            {"type": "Action", "name": "MoveJoints", "args": [poses["home"]]}
                        ]
                    }
                ]
            }
        ]
    }

    bt_manager = BehaviorTreeManager(pose_controller=pp, joints_controller=jj, data_listener=listener)
    
    tree = bt_manager.build_tree(bt_json)
    
    rospy.loginfo("Behavior Tree initialized. Starting tick loop...")

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        status = bt_manager.tick(display_tree=True)
        if status == py_trees.common.Status.FAILURE:
            for node in tree.root.iterate():
                if node.status == py_trees.common.Status.FAILURE:
                    print(f"Node {node.name} failed!")
        
        if status == py_trees.common.Status.SUCCESS:
            rospy.loginfo("Tree reached SUCCESS state.")
            break
        elif status == py_trees.common.Status.FAILURE:
            rospy.logwarn("Tree reached FAILURE state.")
            break
            
        rate.sleep()