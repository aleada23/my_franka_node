import rospy
import tf
import numpy as np
import py_trees
import franka_actions.panda_simb_jac as jac 

class IsGripperOpen(py_trees.behaviour.Behaviour):
    def __init__(self, name, data_listener, threshold=0.03):
        super().__init__(name)
        self.data_listener = data_listener
        self.threshold = threshold

    def update(self):
        q = self.data_listener.get_gripper_joint_positions()
        if q[0] > self.threshold:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

class IsAtPose(py_trees.behaviour.Behaviour):
    def __init__(self, name, data_listener, target_pose, atol=0.01):
        super().__init__(name)
        self.target_pose = np.array(target_pose)
        self.atol = atol
        self.listener = tf.TransformListener()

    def update(self):
        try:
            # trans = [x, y, z], rot = [qx, qy, qz, qw]
            (trans, rot) = self.listener.lookupTransform("panda_link0", "panda_EE", rospy.Time(0))
            # Calculate actual distance in meters
            dist = np.linalg.norm(np.array(trans) - self.target_pose[:3])
            
            if dist < self.atol:
                return py_trees.common.Status.SUCCESS
                
            # Context-aware logic we discussed earlier
            if isinstance(self.parent, py_trees.composites.Parallel):
                return py_trees.common.Status.RUNNING
            return py_trees.common.Status.FAILURE

        except Exception as e:
            # If TF fails, we stay RUNNING to wait for the buffer
            return py_trees.common.Status.RUNNING

class IsGrasped(py_trees.behaviour.Behaviour):
    def __init__(self, name, data_listener, effort_threshold=20.0):
        super().__init__(name)
        self.data_listener = data_listener
        self.effort_threshold = effort_threshold # Adjust based on object stiffness

    def update(self):
        # 1. Get current joint positions and efforts (torques)
        q = self.data_listener.get_gripper_joint_positions()
        effort = self.data_listener.get_gripper_joint_efforts()
        
        # 2. Logic: If effort is high and fingers aren't fully closed
        # Panda gripper efforts are usually negative during closure
        is_applying_force = any(abs(e) > self.effort_threshold for e in effort)
        not_fully_closed = q[0] > 0.001 

        if is_applying_force and not_fully_closed:
            return py_trees.common.Status.SUCCESS
            
        # Context-aware return for Parallel/Selector
        if isinstance(self.parent, py_trees.composites.Parallel):
            return py_trees.common.Status.RUNNING
        return py_trees.common.Status.FAILURE

class IsContactDetected(py_trees.behaviour.Behaviour):
    def __init__(self, name, data_listener, force_threshold=3.0):
        super().__init__(name)
        self.data_listener = data_listener
        self.force_threshold = force_threshold

    def update(self):
        tau = self.data_listener.get_joint_torques()
        q = self.data_listener.get_joint_positions()
        h_e = np.linalg.pinv(jac.panda_jac(q).T) @ tau
        if np.linalg.norm(h_e[:3]) > self.force_threshold:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

class IsAtHome(py_trees.behaviour.Behaviour):
    def __init__(self, name, data_listener, home_config, atol=0.05):
        super().__init__(name)
        self.data_listener = data_listener
        self.home_config = np.array(home_config)
        self.atol = atol

    def initialise(self):
        self.t_start = rospy.get_time()
        self.q_start = self.data_listener.get_joint_positions()

    def update(self):
        # Check if the parent is a Selector
        is_in_selector = isinstance(self.parent, py_trees.composites.Selector)
        
        # Check if the parent is a Parallel
        is_in_parallel = isinstance(self.parent, py_trees.composites.Parallel)
        q = self.data_listener.get_joint_positions()    
        # Example stall detection
        if np.allclose(self.home_config, q, atol=1e-2): 
            return py_trees.common.Status.SUCCESS
        if is_in_selector:
            # If the condition isn't met, return FAILURE so the Selector 
            # moves to the next child (the Action)
            return py_trees.common.Status.FAILURE
        else:
            # If in a Parallel, return RUNNING so we don't kill the 
            # simultaneous Move action
            return py_trees.common.Status.RUNNING