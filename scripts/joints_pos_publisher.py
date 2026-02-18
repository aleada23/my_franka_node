#!/usr/bin/env python3
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import threading
from dynamic_reconfigure.client import Client


class JointTrajectoryPublisher:
    def __init__(self):
        # Publisher to the trajectory controller
        self.pub = rospy.Publisher(
            "/position_joint_trajectory_controller/command",
            JointTrajectory,
            queue_size=10
        )

        # Optional subscriber for topic-based updates
        self.sub = rospy.Subscriber("/desired_joint_positions", JointTrajectoryPoint, self.update_target)

        # Joint names for Panda
        self.joint_names = [
            "panda_joint1", "panda_joint2", "panda_joint3",
            "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"
        ]

        # Current target joint positions
        self.current_point = JointTrajectoryPoint()
        self.current_point.positions = [0.0] * 7
        self.current_point.time_from_start = rospy.Duration(1.0)

        self.lock = threading.Lock()  # thread-safe updates

        #rospy.loginfo("JointTrajectoryPublisher initialized.")

        self.joint_position_clients = [
            Client(f"/motion_generators/position/gains/panda_joint{i+1}", timeout=5)
            for i in range(7)
        ]
        self.joint_velocity_clients = [
            Client(f"/motion_generators/velocity/gains/panda_joint{i+1}", timeout=5)
            for i in range(7)
        ]

        # Start publishing thread
        self.thread = threading.Thread(target=self.run)
        self.thread.daemon = True  # allow clean exit
        self.thread.start()

    def update_target(self, msg):
        """Callback for topic updates"""
        if len(msg.positions) != 7:
            rospy.logwarn("Received joint command does not have 7 elements. Ignoring.")
            return
        with self.lock:
            self.current_point = msg
        #rospy.loginfo(f"Updated target joints via topic: {msg.positions}")

    def set_joints(self, joint_list, duration=5.0):
        """Update joint targets directly from code"""
        if len(joint_list) != 7:
            rospy.logwarn("Joint list must have 7 elements. Ignoring.")
            return
        point = JointTrajectoryPoint()
        point.positions = joint_list
        point.time_from_start = rospy.Duration(duration)
        with self.lock:
            self.current_point = point
        #rospy.loginfo(f"Updated target joints via code: {point.positions}")

    def set_joint_position_gains(self, p_gains, d_gains):
        """Update joint position gains (7-element lists)"""
        for i, client in enumerate(self.joint_position_clients):
            try:
                cfg = {"p": p_gains[i], "d": d_gains[i]}
                client.update_configuration(cfg)
            except Exception as e:
                rospy.logerr(f"Failed to update position gains for joint {i+1}: {e}")

    def set_joint_velocity_gains(self, p_gains, d_gains):
        """Update joint velocity gains (7-element lists)"""
        for i, client in enumerate(self.joint_velocity_clients):
            try:
                cfg = {"p": p_gains[i], "d": d_gains[i]}
                client.update_configuration(cfg)
            except Exception as e:
                rospy.logerr(f"Failed to update velocity gains for joint {i+1}: {e}")


    def run(self):
        """Publish joint trajectory continuously"""
        rate = rospy.Rate(100)
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names

        while not rospy.is_shutdown():
            with self.lock:
                try:
                    traj_msg.points = [self.current_point]
                    self.pub.publish(traj_msg)
                except:
                    pass
            rate.sleep()


