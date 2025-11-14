#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
import tf.transformations as tft
import math

def main():
    rospy.init_node("cartesian_impedance_publisher")

    pub = rospy.Publisher(
        "/cartesian_impedance_example_controller/equilibrium_pose",
        PoseStamped,
        queue_size=10
    )

    rate = rospy.Rate(100)  # 100 Hz publish rate

    # Initial pose parameters
    x, y, z = 0.3, 0.0, 0.5

    while not rospy.is_shutdown():
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "panda_link0"

        # Simple circular motion in Y-Z plane
        t = rospy.Time.now().to_sec()
        msg.pose.position.x = x
        msg.pose.position.y = 0.2 * math.sin(0.2 * t)
        msg.pose.position.z = z + 0.05 * math.cos(0.2 * t)

        # Keep orientation constant (45° about Y-axis)
        quat = tft.quaternion_from_euler(0, math.pi/4, 0)
        msg.pose.orientation.x = quat[0]
        msg.pose.orientation.y = quat[1]
        msg.pose.orientation.z = quat[2]
        msg.pose.orientation.w = quat[3]

        pub.publish(msg)
        rate.sleep()

if __name__ == "__main__":
    main()
