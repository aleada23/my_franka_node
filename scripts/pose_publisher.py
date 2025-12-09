#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
import tf.transformations as tft
import threading
from dynamic_reconfigure.client import Client

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
        print("ok")
        self.stiffness_client = Client("/cartesian_impedance_example_controller/dynamic_reconfigure_compliance_param_node",timeout=5)
        print("ok 2")
        #self.joint_position_clients = [Client(f"/motion_generators/position/gains/panda_joint{i+1}", timeout=5) for i in range(7)]
        #self.joint_velocity_clients = [Client(f"/motion_generators/velocity/gains/panda_joint{i+1}", timeout=5) for i in range(7)]

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

    def set_stiffness(self, translational, rotational, nullspace=5.0):
        """Update Cartesian stiffness using dynamic_reconfigure."""
        try:
            cfg = {
                "translational_stiffness": translational,
                "rotational_stiffness": rotational,
                "nullspace_stiffness": nullspace
            }
            self.stiffness_client.update_configuration(cfg)
            rospy.loginfo(f"Updated stiffness: translational={translational}, rotational={rotational}, nullspace={nullspace}")
        except Exception as e:
            rospy.logerr(f"Failed to update stiffness: {e}")


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

