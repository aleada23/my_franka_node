import rospy
import tf
import actionlib
import numpy as np
import py_trees
from franka_gripper.msg import MoveAction, MoveGoal
import franka_actions.panda_simb_jac as jac 
import switch_controller as ctrl_switch

class MovePose(py_trees.behaviour.Behaviour):
    def __init__(self, name, pose_controller, target_pose):
        super().__init__(name)
        self.target_pose = np.array(target_pose)
        self.pose_controller = pose_controller

    def initialise(self):
        ctrl_switch.start_controller_exclusive("cartesian_impedance_example_controller")
        self.pose_controller.set_stiffness(200, 200, 1)
        self.pose_controller.set_pose(self.target_pose[:3], self.target_pose[3:])

    def update(self):
        return py_trees.common.Status.RUNNING

class OpenGripper(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.client = actionlib.SimpleActionClient('/franka_gripper/move', MoveAction)

    def initialise(self):
        self.client.wait_for_server()
        self.client.send_goal(MoveGoal(width=0.08, speed=0.1))

    def update(self):
        state = self.client.get_state()
        if state == 1:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING

class CloseGripper(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.client = actionlib.SimpleActionClient('/franka_gripper/move', MoveAction)

    def initialise(self):
        self.client.wait_for_server()
        self.client.send_goal(MoveGoal(width=0.0, speed=0.1))

    def update(self):
        state = self.client.get_state()
        if state == actionlib.GoalStatus.SUCCEEDED:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING

class MoveDownUntillContact(py_trees.behaviour.Behaviour):
    def __init__(self, name, pose_controller, current_pose):
        super().__init__(name)
        self.pose_controller = pose_controller
        self.cmd_pose = np.array(current_pose)

    def initialise(self):
        ctrl_switch.start_controller_exclusive("cartesian_impedance_example_controller")
        self.pose_controller.set_stiffness(100, 100, 5)

    def update(self):
        self.cmd_pose[2] -= 0.001
        self.pose_controller.set_pose(self.cmd_pose[:3], self.cmd_pose[3:])
        return py_trees.common.Status.RUNNING

class MoveJoints(py_trees.behaviour.Behaviour):
    def __init__(self, name, joints_controller, target_pos):
        super().__init__(name)
        self.target_pos = np.array(target_pos)
        self.joints_controller = joints_controller

    def initialise(self):
        ctrl_switch.start_controller_exclusive("position_joint_trajectory_controller")
        self.joints_controller.set_joints(self.target_pos)

    def update(self):
        return py_trees.common.Status.RUNNING

class MeasureGripperSites(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.listener = tf.TransformListener()
    def update(self):
        try:
            (trans, rot) = self.listener.lookupTransform("panda_link0", "panda_EE", rospy.Time(0))
            self.logger.info(f"Gripper Sites Measured at: {trans}")
            return py_trees.common.Status.SUCCESS
        except:
            return py_trees.common.Status.RUNNING

class MeasureMassWithTorque(py_trees.behaviour.Behaviour):
    def __init__(self, name, data_listener):
        super().__init__(name)
        self.data_listener = data_listener
        self.samples = []

    def initialise(self):
        self.samples = []
        self.start_time = rospy.get_time()

    def update(self):
        if rospy.get_time() - self.start_time < 1.0:
            tau = self.data_listener.get_joint_torques()
            q = self.data_listener.get_joint_positions()
            h_e = np.linalg.pinv(jac.panda_jac(q).T) @ tau
            self.samples.append(h_e)
            return py_trees.common.Status.RUNNING
        mass_estimate = np.mean(self.samples, axis=0)
        self.logger.info(f"Object mass estimate (wrench): {mass_estimate}")
        return py_trees.common.Status.SUCCESS