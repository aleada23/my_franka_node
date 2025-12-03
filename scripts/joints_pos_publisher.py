#!/usr/bin/env python3
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import threading

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

        rospy.loginfo("JointTrajectoryPublisher initialized.")

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
        rospy.loginfo(f"Updated target joints via topic: {msg.positions}")

    def set_joints(self, joint_list, duration=1.0):
        """Update joint targets directly from code"""
        if len(joint_list) != 7:
            rospy.logwarn("Joint list must have 7 elements. Ignoring.")
            return
        point = JointTrajectoryPoint()
        point.positions = joint_list
        point.time_from_start = rospy.Duration(duration)
        with self.lock:
            self.current_point = point
        rospy.loginfo(f"Updated target joints via code: {point.positions}")

    def run(self):
        """Publish joint trajectory continuously"""
        rate = rospy.Rate(50)  # 50 Hz is enough for trajectory controller
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names

        while not rospy.is_shutdown():
            with self.lock:
                traj_msg.points = [self.current_point]
                self.pub.publish(traj_msg)
            rate.sleep()


