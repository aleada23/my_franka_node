#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
import tf.transformations as tft
import threading

class PosePublisher:
    def __init__(self):
        #rospy.init_node("trajectory_replay_node", anonymous=True)
        self.pub = rospy.Publisher(
            "/cartesian_impedance_example_controller/equilibrium_pose",
            PoseStamped,
            queue_size=10,
        )
        self.sub = rospy.Subscriber("/desired_pose", PoseStamped, self.update_target)
        
        self.current_pose = PoseStamped()
        self.current_pose.header.frame_id = "panda_link0"
        
        self.lock = threading.Lock()  # thread-safe updates
        rospy.loginfo("PosePublisher initialized.")

        # Start publishing in a separate thread so we can update poses from main
        self.thread = threading.Thread(target=self.run)
        self.thread.start()

    def update_target(self, msg):
        """Callback for ROS topic updates"""
        with self.lock:
            self.current_pose = msg
            rospy.loginfo(f"Updated target pose via topic: {msg.pose.position}")

    def set_pose(self, xyz, euler):
        """Update pose directly from code"""
        quat = tft.quaternion_from_euler(*euler)
        msg = PoseStamped()
        msg.header.frame_id = "panda_link0"
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = xyz
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = quat
        with self.lock:
            self.current_pose = msg
        rospy.loginfo(f"Updated target pose via code: {xyz}")

    def run(self):
        rate = rospy.Rate(100)  # 100 Hz
        while not rospy.is_shutdown():
            with self.lock:
                self.current_pose.header.stamp = rospy.Time.now()
                self.pub.publish(self.current_pose)
            rate.sleep()
