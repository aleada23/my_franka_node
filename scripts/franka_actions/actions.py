#!/usr/bin/env python3
import rospy
import tf
import actionlib
from franka_gripper.msg import MoveAction, MoveGoal, GraspAction, GraspGoal, HomingAction
from pose_publisher import PosePublisher
from joints_pos_publisher import JointTrajectoryPublisher
from data_listener import FrankaListener
import franka_actions.panda_simb_jac as jac 
import switch_controller as ctrl_switch

import py_trees
import numpy as np
import time



class MovePose(py_trees.behaviour.Behaviour): 
    def __init__(self, name, pose_controller, target_pose, kp=1.0, tolerance=1e-2):
        super().__init__(name)
        self.target_pose = np.array(target_pose)
        self.kp = kp
        self.tolerance = tolerance
        self.start_time = None
        self.pose_controller = pose_controller
        
    def setup(self, **kwargs):
        self.logger.debug(f"{self.name} [MovePose::setup()]")

    def initialise(self):
        
        self.logger.debug(f"{self.name} [MovePose::initialise()]")
        ctrl_switch.start_controller_exclusive("cartesian_impedance_example_controller")
        #CONTROLLER INIT
        self.pose_controller.set_stiffness(200, 200, 1)
        self.pose_controller.set_pose(self.target_pose[:3], self.target_pose[3:])
        self.listener = tf.TransformListener()
        
    def update(self):      
        try:
            (trans, rot) = self.listener.lookupTransform("panda_link0", "panda_EE", rospy.Time(0))
            roll, pitch, yaw = tf.transformations.euler_from_quaternion(rot)
            current_pose = np.hstack((trans, np.array([roll, pitch, yaw])))
            error = self.target_pose - current_pose
            if np.allclose(error[:3], 0, atol=self.tolerance):
                self.logger.debug(f"{self.name}: Reached  pose.")
                print("reached pose: ", current_pose)
                return py_trees.common.Status.SUCCESS
        except:
            pass
        
        return py_trees.common.Status.RUNNING
    def terminate(self, new_status):
        self.logger.debug(f"{self.name} [MovePose::terminate()] -> {new_status}")

class MoveJoints(py_trees.behaviour.Behaviour):
    def __init__(self, name, joints_controller, target_pos, kp=1.0, tolerance=1e-2):
        super().__init__(name)
        self.target_pos = np.array(target_pos)
        self.kp = kp
        self.tolerance = tolerance
        self.start_time = None
        self.joints_controller = joints_controller
        
    def setup(self, **kwargs):
        self.logger.debug(f"{self.name} [MoveJoints::setup()]")

    def initialise(self):
        self.logger.debug(f"{self.name} [MoveJoints::initialise()]")
        ctrl_switch.start_controller_exclusive("position_joint_trajectory_controller")
        #CONTROLLER INIT
        self.joints_controller.set_joints(self.target_pos)
        self.data_listener = FrankaListener()
        
    def update(self):      
        try:
                   
            q = self.data_listener.get_joint_positions()
            error = self.target_pos - q
            if np.allclose(error[:], 0, atol=self.tolerance):
                self.logger.debug(f"{self.name}: Reached joints position.")
                print("rached configuration: ", q)
                return py_trees.common.Status.SUCCESS
        except:
            pass
        return py_trees.common.Status.RUNNING

class OpenGripper(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.client = actionlib.SimpleActionClient('/franka_gripper/move', MoveAction)
        self.client.wait_for_server()
        

    def setup(self, **kwargs):
        self.logger.debug(f"{self.name} [OpenGripper::setup()]")

    def initialise(self):
        self.logger.debug(f"{self.name} [OpenGripper::initialise()]")
        self.opening = 0.04
        goal = MoveGoal(width=self.opening, speed=0.1)
        self.client.send_goal(goal)
        self.listener = FrankaListener()
        self.t_start = rospy.get_time()

    def update(self):
        try:
            t = rospy.get_time()
            q = self.listener.get_gripper_joint_positions()
            if q[0] > 0.0014 and q[1] > 0.0014 and (t - self.t_start) >3:
                self.logger.debug(f"{self.name}: Gripper is open.")
                print("gripper is open")
                return py_trees.common.Status.SUCCESS
        except:
            pass
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self.logger.debug(f"{self.name} [OpenGripper::terminate()] -> {new_status}")

class CloseGripper(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.client = actionlib.SimpleActionClient('/franka_gripper/move', MoveAction)
        self.client.wait_for_server()
        

    def setup(self, **kwargs):
        self.logger.debug(f"{self.name} [CloseGripper::setup()]")

    def initialise(self):
        self.logger.debug(f"{self.name} [CloseGripper::initialise()]")
        self.opening = 0.00
        self.t_start = rospy.get_time()

        goal = MoveGoal(width=self.opening, speed=0.1)
        self.client.send_goal(goal)
        self.listener = FrankaListener()

    def update(self):
        try:
            t = rospy.get_time()  
            q = self.listener.get_gripper_joint_positions()
            if q[0] <= 0.0013 and q[1] <= 0.0013 and (t - self.t_start) >3:
                self.logger.debug(f"{self.name}: Gripper is close.")
                print("gripper is closed", q)
                return py_trees.common.Status.SUCCESS
        except:
            pass
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self.logger.debug(f"{self.name} [CloseGripper::terminate()] -> {new_status}")

class MoveDownUntillContact(py_trees.behaviour.Behaviour):
    def __init__(self, name, target_pose, pose_controller, kp=1.0, tolerance=1e-2):
        super().__init__(name)
        self.target_pose = np.array(target_pose)
        self.kp = kp
        self.tolerance = tolerance
        self.start_time = None
        self.pose_controller = pose_controller
        
    def setup(self, **kwargs):
        self.logger.debug(f"{self.name} [MoveDownUntillContact::setup()]")

    def initialise(self):
        self.logger.debug(f"{self.name} [MoveDownUntillContact::initialise()]")
        #CONTROLLER INIT
        ctrl_switch.start_controller_exclusive("cartesian_impedance_example_controller")
        self.pose_controller.set_stiffness(100, 100, 5)
        self.pose_controller.set_pose(self.target_pose[:3], self.target_pose[3:])
        self.listener = tf.TransformListener()
        self.t_start = rospy.get_time()
        self.data_listener = FrankaListener()


    def update(self):      
        try:      
            (trans, rot) = self.listener.lookupTransform("panda_link0", "panda_EE", rospy.Time(0))
            roll, pitch, yaw = tf.transformations.euler_from_quaternion(rot)
            current_pose = np.hstack((trans, np.array([roll, pitch, yaw])))
            t = rospy.get_time()
            #get tau and joint position
            tau = self.data_listener.get_joint_torques()
            q = self.data_listener.get_joint_positions()
            #compute he
            Jac = jac.panda_jac(q)
            J_inv_T = np.linalg.pinv(Jac.T)
            h_e = J_inv_T @ tau
            
            self.target_pose = self.target_pose + np.array([0.0, 0.0, -0.001, 0.0, 0.0, 0.0])
            self.pose_controller.set_pose(self.target_pose[:3], self.target_pose[3:])
            if (t - self.t_start) >5 and np.linalg.norm(h_e[3:])>2:
                self.logger.debug(f"{self.name}: Reached  pose.")
                (trans, rot) = self.listener.lookupTransform("panda_link0", "panda_EE", rospy.Time(0))
                roll, pitch, yaw = tf.transformations.euler_from_quaternion(rot)
                current_pose = np.hstack((trans, np.array([roll, pitch, yaw])))
                print("table touched, e-e pose: ", current_pose)
                return py_trees.common.Status.SUCCESS
        except:
            pass
        
        return py_trees.common.Status.RUNNING

class MeasureGripperSites(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        
    def setup(self, **kwargs):
        self.logger.debug(f"{self.name} [MeasureGripperSites::setup()]")

    def initialise(self):
        self.logger.debug(f"{self.name} [MeasureGripperSites::initialise()]")
        self.t_start = rospy.get_time()
        self.pose_buffer = []
        self.listener = tf.TransformListener()

    def update(self):
        try:
            t = rospy.get_time()
            (trans, rot) = self.listener.lookupTransform("panda_link0", "panda_EE", rospy.Time(0))
            roll, pitch, yaw = tf.transformations.euler_from_quaternion(rot)
            current_pose = np.hstack((trans, np.array([roll, pitch, yaw])))
            if (t - self.t_start)<1:
                self.pose_buffer.append(current_pose)
            else:
                mean_pose = np.mean(np.array(self.pose_buffer), axis=0)
                print("MeasureGripperSites position", mean_pose)
                return py_trees.common.Status.SUCCESS
        except:
            pass

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self.logger.debug(f"{self.name} [MeasureGripperSites::terminate()] -> {new_status}")

class MeasureAppliedHE(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)

    def setup(self, **kwargs):
        self.logger.debug(f"{self.name} [MeasureGripperSites::setup()]")

    def initialise(self):
        self.logger.debug(f"{self.name} [MeasureGripperSites::initialise()]")
        self.t_start = rospy.get_time()
        self.data_listener = FrankaListener()
        self.mass_buffer = []
    
    def update(self):
        try:
            t = rospy.get_time()
            #get tau and joint position
            tau = self.data_listener.get_joint_torques()
            q = self.data_listener.get_joint_positions()
            #compute he
            Jac = jac.panda_jac(q)
            J_inv_T = np.linalg.pinv(Jac.T)
            h_e = J_inv_T @ tau
            
            print("mean h_e", h_e)
            #return py_trees.common.Status.SUCCESS
        except:
            pass
        return py_trees.common.Status.RUNNING
    def terminate(self, new_status):
        self.logger.debug(f"{self.name} [MeasureGripperSites::terminate()] -> {new_status}")

class MeasureMassWithTorque(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)

    def setup(self, **kwargs):
        self.logger.debug(f"{self.name} [MeasureGripperSites::setup()]")

    def initialise(self):
        self.logger.debug(f"{self.name} [MeasureGripperSites::initialise()]")
        self.t_start = rospy.get_time()
        self.data_listener = FrankaListener()
        self.mass_buffer = []
    
    def update(self):
        try:
            t = rospy.get_time()
            #get tau and joint position
            tau = self.data_listener.get_joint_torques()
            q = self.data_listener.get_joint_positions()
            #compute he
            Jac = jac.panda_jac(q)
            J_inv_T = np.linalg.pinv(Jac.T)
            h_e = J_inv_T @ tau
            if (t - self.t_start)<5:
                if (t - self.t_start)>4:
                    self.mass_buffer.append(h_e)
            elif (t - self.t_start)>=5:
                mean_mass = np.mean(np.array(self.mass_buffer), axis=0)
                print("Mass meas. h_e", mean_mass)
                return py_trees.common.Status.SUCCESS
        except:
            pass
        return py_trees.common.Status.RUNNING
    def terminate(self, new_status):
        self.logger.debug(f"{self.name} [MeasureGripperSites::terminate()] -> {new_status}")

class MeasureGripperOpnening(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)

    def setup(self, **kwargs):
        self.logger.debug(f"{self.name} [MeasureGripperSites::setup()]")

    def initialise(self):
        self.logger.debug(f"{self.name} [MeasureGripperSites::initialise()]")
        self.t_start = rospy.get_time()
        self.positions_buffer = []
        self.data_listener = FrankaListener()
        
    def update(self):
        try:
            t = rospy.get_time()
            q = self.data_listener.get_gripper_joint_positions()
            if (t - self.t_start)<3:
                if (t - self.t_start)>2:
                    self.positions_buffer.append(q)
            elif (t - self.t_start)>=3:
                q_mean = np.mean(np.array(self.positions_buffer), axis=0)
                print("gripper opening", q_mean)
                return py_trees.common.Status.SUCCESS
        except:
            pass
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self.logger.debug(f"{self.name} [MeasureGripperSites::terminate()] -> {new_status}")

