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
    
    # 1. Initialize Hardware Interfaces
    listener = FrankaListener()
    pp = PosePublisher()             # Cartesian Controller
    jj = JointTrajectoryPublisher()  # Joint Controller
    rospy.sleep(1.0)                 # Warmup time for ROS publishers/subscribers

    # 2. Define Pose/Config Constants (The "Knowledge Base")
    # This maps the string names in your JSON to actual robot coordinates.
    z_offset = np.array([0, 0, 0.1, 0, 0, 0])
    poses = {
        "home": [0, 0, 0, -1.57079, 0, 1.57079, -0.7853],
        "green_cube_0": [0.4, 0.0, 0.3, 0, np.pi, 0],
        "green_cube_0 + z_offset": ([0.5, 0.0, 0.3, 0, np.pi, 0] + z_offset),
        "temp_pose_1 + z_offset": [0.4, 0.0, 0.4, 0, np.pi, 0],
    }

    # 3. Define the Behavior Tree using the new JSON structure
    # This structure matches the "O_BT" dictionary you provided
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

    # 4. Initialize Manager and Build Tree
    # We pass the hardware interfaces once to the manager
    bt_manager = BehaviorTreeManager(
        pose_controller=pp, 
        joints_controller=jj, 
        data_listener=listener
    )
    
    tree = bt_manager.build_tree(bt_json)
    
    # Optional: Render the tree to a file to verify structure
    # bt_manager.print_tree()

    rospy.loginfo("Behavior Tree initialized. Starting tick loop...")

    # 5. Execution Loop
    rate = rospy.Rate(10) # 10Hz
    while not rospy.is_shutdown():
        status = bt_manager.tick(display_tree=True)
        if status == py_trees.common.Status.FAILURE:
            # Find the tip (the node that actually failed)
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