#!/usr/bin/env python3
import rospy
import numpy as np
from pose_publisher import PosePublisher
from data_listener import FrankaListener
from joints_pos_publisher import JointTrajectoryPublisher
from franka_actions.bt_manager import BehaviorTreeManager
import py_trees

def resolve_bt_args(bt_node, poses):
    if "args" in bt_node and len(bt_node["args"]) == 1:
        arg = bt_node["args"][0]

        if isinstance(arg, str) and arg in poses:
            bt_node["args"] = [poses[arg]]

    if "children" in bt_node:
        for child in bt_node["children"]:
            resolve_bt_args(child, poses)

def fix_bt(node):
    if "name" in node and node["name"] in ["Parallel","Sequence","Selector"]:
        node["type"] = node["name"]
        del node["name"]
    elif "name" in node and "type" not in node:
        if node["name"].startswith("is_"):
            node["type"] = "Condition"
        else:
            node["type"] = "Action"
    if "children" in node:
        for c in node["children"]:
            fix_bt(c)


if __name__ == "__main__":
    rospy.init_node("panda_bt_control_node", anonymous=True)
    
    listener = FrankaListener()
    pp = PosePublisher()            
    jj = JointTrajectoryPublisher()  
    rospy.sleep(1.0)                

    z_offset = np.array([0, 0, 0.1, 0, 0, 0])
    poses = {
        "home": [0, 0, 0, -1.57079, 0, 1.57079, -0.7853],
<<<<<<< HEAD
        "yellow_cube_2": [0.4, 0.0, 0.3, 0, np.pi, 0],
        "yellow_cube_2 + z_offset": ([0.5, 0.0, 0.3, 0, np.pi, 0] + z_offset),
        "blue_cube_3": [0.4, 0.0, 0.3, 0, np.pi, 0],
        "blue_cube_3 + z_offset": ([0.5, 0.0, 0.3, 0, np.pi, 0] + z_offset),
        "temp_pose_2 + z_offset": [0.4, 0.0, 0.4, 0, np.pi, 0],
=======
        "green_parallelepiped_0": [0.4, 0.0, 0.3, 0, np.pi, 0],
        "green_parallelepiped_0 + z_offset": ([0.5, 0.0, 0.3, 0, np.pi, 0] + z_offset),
        "blue_parallelepiped_2": [0.4, 0.0, 0.3, 0, np.pi, 0],
        "blue_parallelepiped_2 + z_offset": ([0.5, 0.0, 0.3, 0, np.pi, 0] + z_offset),
        "temp_pose_0 + z_offset": [0.4, 0.0, 0.4, 0, np.pi, 0],
        
>>>>>>> updated functions with BTs
    }

  
    bt_json = {
<<<<<<< HEAD
        "type": "Selector",
        "children": [
          {
            "type": "Condition",
            "name": "is_at_home",
            "args": [poses["home"]]
          },
          {
            "type": "Sequence",
            "children": [
              {
                "type": "Selector",
                "children": [
                  { "type": "Condition", "name": "is_gripper_open", "args": [] },
                  { "type": "Action", "name": "OpenGripper", "args": [] }
                ]
              },
              {
                "type": "Parallel",
                "children": [
                  { "type": "Condition", "name": "is_at_pose", "args": [poses["yellow_cube_2 + z_offset"]] },
                  { "type": "Action", "name": "MovePose", "args": [poses["yellow_cube_2 + z_offset"]] }
                ]
              },
              {
                "type": "Parallel",
                "children": [
                  { "type": "Condition", "name": "is_at_pose", "args": [poses["yellow_cube_2"]] },
                  { "type": "Action", "name": "MovePose", "args": [poses["yellow_cube_2"]] }
                ]
              },
              { "type": "Action", "name": "CloseGripper", "args": [] },
              {
                "type": "Parallel",
                "children": [
                  { "type": "Condition", "name": "is_at_pose", "args": [poses["yellow_cube_2 + z_offset"]] },
                  { "type": "Action", "name": "MovePose", "args": [poses["yellow_cube_2 + z_offset"]] }
                ]
              },
              {
                "type": "Parallel",
                "children": [
                  { "type": "Condition", "name": "is_at_pose", "args": [poses["temp_pose_2 + z_offset"]] },
                  { "type": "Action", "name": "MovePose", "args": [poses["temp_pose_2 + z_offset"]] }
                ]
              },
              {
                "type": "Parallel",
                "children": [
                  { "type": "Condition", "name": "is_contact_detected", "args": [] },
                  { "type": "Action", "name": "MoveDownUntillContact", "args": [] }
                ]
              },
              { "type": "Action", "name": "OpenGripper", "args": [] },
              {
                "type": "Parallel",
                "children": [
                  { "type": "Condition", "name": "is_at_pose", "args": [poses["temp_pose_2 + z_offset"]] },
                  { "type": "Action", "name": "MovePose", "args": [poses["temp_pose_2 + z_offset"]] }
                ]
              },
              {
                "type": "Parallel",
                "children": [
                  { "type": "Condition", "name": "is_at_pose", "args": [poses["blue_cube_3 + z_offset"]] },
                  { "type": "Action", "name": "MovePose", "args": [poses["blue_cube_3 + z_offset"]] }
                ]
              },
              {
                "type": "Parallel",
                "children": [
                  { "type": "Condition", "name": "is_at_pose", "args": [poses["blue_cube_3"]] },
                  { "type": "Action", "name": "MovePose", "args": [poses["blue_cube_3"]] }
                ]
              },
              { "type": "Action", "name": "CloseGripper", "args": [] },
              {
                "type": "Parallel",
                "children": [
                  { "type": "Condition", "name": "is_at_pose", "args": [poses["blue_cube_3 + z_offset"]] },
                  { "type": "Action", "name": "MovePose", "args": [poses["blue_cube_3 + z_offset"]] }
                ]
              },
              {
                "type": "Parallel",
                "children": [
                  { "type": "Condition", "name": "is_at_pose", "args": [poses["temp_pose_2 + z_offset"]] },
                  { "type": "Action", "name": "MovePose", "args": [poses["temp_pose_2 + z_offset"]] }
                ]
              },
              {
                "type": "Parallel",
                "children": [
                  { "type": "Condition", "name": "is_contact_detected", "args": [] },
                  { "type": "Action", "name": "MoveDownUntillContact", "args": [] }
                ]
              },
              { "type": "Action", "name": "OpenGripper", "args": [] },
              {
                "type": "Parallel",
                "children": [
                  { "type": "Condition", "name": "is_at_pose", "args": [poses["temp_pose_2 + z_offset"]] },
                  { "type": "Action", "name": "MovePose", "args": [poses["temp_pose_2 + z_offset"]] }
                ]
              },
            
                  { "type": "Action", "name": "MoveJoints", "args": [poses["home"]] }
            
            ]
          }
        ]
      }

=======
 "children": [
  {
   "args": [],
   "name": "is_at_home",
   "type": "Condition"
  },
  {
   "children": [
    {
     "children": [
      {
       "args": [],
       "name": "is_gripper_open",
       "type": "Condition"
      },
      {
       "args": [],
       "name": "OpenGripper",
       "type": "Action"
      }
     ],
     "type": "Selector"
    },
    {
     "children": [
      {
       "args": [
        "green_parallelepiped_0 + z_offset"
       ],
       "name": "is_at_pose",
       "type": "Condition"
      },
      {
       "args": [
        "green_parallelepiped_0 + z_offset"
       ],
       "name": "MovePose",
       "type": "Action"
      }
     ],
     "type": "Parallel"
    },
    {
     "children": [
      {
       "args": [
        "green_parallelepiped_0"
       ],
       "name": "is_at_pose",
       "type": "Condition"
      },
      {
       "args": [
        "green_parallelepiped_0"
       ],
       "name": "MovePose",
       "type": "Action"
      }
     ],
     "type": "Parallel"
    },
    {
     "args": [],
     "name": "CloseGripper",
     "type": "Action"
    },
    {
     "children": [
      {
       "args": [
        "green_parallelepiped_0 + z_offset"
       ],
       "name": "is_at_pose",
       "type": "Condition"
      },
      {
       "args": [
        "green_parallelepiped_0 + z_offset"
       ],
       "name": "MovePose",
       "type": "Action"
      }
     ],
     "type": "Parallel"
    },
    {
     "children": [
      {
       "args": [
        "temp_pose_0 + z_offset"
       ],
       "name": "is_at_pose",
       "type": "Condition"
      },
      {
       "args": [
        "temp_pose_0 + z_offset"
       ],
       "name": "MovePose",
       "type": "Action"
      }
     ],
     "type": "Parallel"
    },
    {
     "children": [
      {
       "args": [],
       "name": "is_contact_detected",
       "type": "Condition"
      },
      {
       "args": [],
       "name": "MoveDownUntillContact",
       "type": "Action"
      }
     ],
     "type": "Parallel"
    },
    {
     "args": [],
     "name": "OpenGripper",
     "type": "Action"
    },
    {
     "children": [
      {
       "args": [
        "temp_pose_0 + z_offset"
       ],
       "name": "is_at_pose",
       "type": "Condition"
      },
      {
       "args": [
        "temp_pose_0 + z_offset"
       ],
       "name": "MovePose",
       "type": "Action"
      }
     ],
     "type": "Parallel"
    },
    {
     "children": [
      {
       "args": [
        "blue_parallelepiped_2 + z_offset"
       ],
       "name": "is_at_pose",
       "type": "Condition"
      },
      {
       "args": [
        "blue_parallelepiped_2 + z_offset"
       ],
       "name": "MovePose",
       "type": "Action"
      }
     ],
     "type": "Parallel"
    },
    {
     "children": [
      {
       "args": [
        "blue_parallelepiped_2"
       ],
       "name": "is_at_pose",
       "type": "Condition"
      },
      {
       "args": [
        "blue_parallelepiped_2"
       ],
       "name": "MovePose",
       "type": "Action"
      }
     ],
     "type": "Parallel"
    },
    {
     "args": [],
     "name": "CloseGripper",
     "type": "Action"
    },
    {
     "children": [
      {
       "args": [
        "blue_parallelepiped_2 + z_offset"
       ],
       "name": "is_at_pose",
       "type": "Condition"
      },
      {
       "args": [
        "blue_parallelepiped_2 + z_offset"
       ],
       "name": "MovePose",
       "type": "Action"
      }
     ],
     "type": "Parallel"
    },
    {
     "children": [
      {
       "args": [
        "temp_pose_0 + z_offset"
       ],
       "name": "is_at_pose",
       "type": "Condition"
      },
      {
       "args": [
        "temp_pose_0 + z_offset"
       ],
       "name": "MovePose",
       "type": "Action"
      }
     ],
     "type": "Parallel"
    },
    {
     "children": [
      {
       "args": [],
       "name": "is_contact_detected",
       "type": "Condition"
      },
      {
       "args": [],
       "name": "MoveDownUntillContact",
       "type": "Action"
      }
     ],
     "type": "Parallel"
    },
    {
     "args": [],
     "name": "OpenGripper",
     "type": "Action"
    },
    {
     "children": [
      {
       "args": [
        "temp_pose_0 + z_offset"
       ],
       "name": "is_at_pose",
       "type": "Condition"
      },
      {
       "args": [
        "temp_pose_0 + z_offset"
       ],
       "name": "MovePose",
       "type": "Action"
      }
     ],
     "type": "Parallel"
    },
    {
     "args": [
      "home"
     ],
     "name": "MoveJoints",
     "type": "Action"
    }
   ],
   "type": "Sequence"
  }
 ],
 "type": "Selector"
}
    
>>>>>>> updated functions with BTs
    bt_manager = BehaviorTreeManager(pose_controller=pp, joints_controller=jj, data_listener=listener)
    
    fix_bt(bt_json)
    resolve_bt_args(bt_json, poses)
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