#!/usr/bin/env python3
import rospy
from franka_msgs.msg import FrankaState
from sensor_msgs.msg import JointState
#from franka_gripper.msg import JointState
import numpy as np
import threading

class FrankaListener:
    def __init__(self):
        #rospy.init_node("franka_listener_node", anonymous=True)
        
        # Latest data storage
        self.lock = threading.Lock()
        self.latest_franka_state = None
        self.latest_joint_state = None

        # Subscribers
        rospy.Subscriber("/franka_state_controller/franka_states", FrankaState, self.franka_state_callback)
        rospy.Subscriber("/franka_state_controller/joint_states", JointState, self.joint_state_callback)
        rospy.Subscriber("/franka_gripper/joint_states", JointState, self.gripper_joint_state_callback)

        rospy.loginfo("FrankaListener initialized.")

        # Run ROS spin in a separate thread
        self.thread = threading.Thread(target=rospy.spin)
        self.thread.start()

    def franka_state_callback(self, msg):
        with self.lock:
            self.latest_franka_state = msg

    def joint_state_callback(self, msg):
        with self.lock:
            self.latest_joint_state = msg
    
    def gripper_joint_state_callback(self, msg):
        with self.lock:
            self.gripper_latest_joint_state = msg


    # ----------------------------
    # Methods to access data easily
    # ----------------------------
    def get_joint_positions(self):
        with self.lock:
            if self.latest_joint_state is None:
                return None
            joint_order = [
                "panda_joint1", "panda_joint2", "panda_joint3",
                "panda_joint4", "panda_joint5", "panda_joint6",
                "panda_joint7"
            ]
            joint_map = {name: i for i, name in enumerate(self.latest_joint_state.name)}
            return np.array([self.latest_joint_state.position[joint_map[j]] for j in joint_order])

    def get_gripper_joint_positions(self):
        with self.lock:
            if self.gripper_latest_joint_state is None:
                return None
            joint_order = [
                "panda_finger_joint1", "panda_finger_joint1"
            ]
            joint_map = {name: i for i, name in enumerate(self.gripper_latest_joint_state.name)}
            return np.array([self.gripper_latest_joint_state.position[joint_map[j]] for j in joint_order])

    def get_joint_velocities(self):
        with self.lock:
            if self.latest_joint_state is None:
                return None
            joint_order = [
                "panda_joint1", "panda_joint2", "panda_joint3",
                "panda_joint4", "panda_joint5", "panda_joint6",
                "panda_joint7"
            ]
            joint_map = {name: i for i, name in enumerate(self.latest_joint_state.name)}
            return np.array([self.latest_joint_state.velocity[joint_map[j]] for j in joint_order])

    def get_end_effector_pose(self):
        with self.lock:
            if self.latest_franka_state is None:
                return None
            return np.array(self.latest_franka_state.O_T_EE).reshape(4,4)

    def get_joint_torques(self):
        with self.lock:
            if self.latest_joint_state is None:
                return None
            joint_order = [
                "panda_joint1", "panda_joint2", "panda_joint3",
                "panda_joint4", "panda_joint5", "panda_joint6",
                "panda_joint7"
            ]
            joint_map = {name: i for i, name in enumerate(self.latest_joint_state.name)}
            return np.array([self.latest_joint_state.effort[joint_map[j]] for j in joint_order])

    def get_robot_mode(self):
        with self.lock:
            if self.latest_franka_state is None:
                return None
            return self.latest_franka_state.robot_mode

    def get_last_joint_time(self):
        with self.lock:
            if self.latest_joint_state is None:
                return None
            return self.latest_joint_state.header.stamp.to_sec()

