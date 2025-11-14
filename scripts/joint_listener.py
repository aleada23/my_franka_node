#!/usr/bin/env python3
import rospy
from franka_msgs.msg import FrankaState
from sensor_msgs.msg import JointState
import numpy as np

def franka_state_callback(msg):
    #print("\n=== Franka State ===")
    #print("Joint positions (q):", msg.q)
    #print("Joint velocities (dq):", msg.dq)
    #print("Joint torques (tau_J):", msg.tau_J)
    #print("End-effector pose (O_T_EE):", msg.O_T_EE)
    #print("Robot mode:", msg.robot_mode)

    delta = 1e-6
    jacobian = np.zeros((6, 7))
    q = np.array(msg.q)
    T0 = np.array(msg.O_T_EE).reshape(4, 4)
    print()
    for i in range(7):
        dq = np.zeros(7)
        dq[i] = delta
        T_new = T0.copy()  # In real application, call FK here
        jacobian[0:3, i] = (T_new[0:3, 3] - T0[0:3, 3]) / delta
        jacobian[3:6, i] = np.zeros(3)

    print("\nJacobian (6x7, approximate):\n", jacobian)
    

def joint_state_callback(msg):
    #print("\n=== Joint States ===")
    #print("Joint names:", msg.name)
    #print("Positions:", msg.position)
    #print("Velocities:", msg.velocity)
    #print("Effort (torques):", msg.effort)

    joint_order = [
        "panda_joint1",
        "panda_joint2",
        "panda_joint3",
        "panda_joint4",
        "panda_joint5",
        "panda_joint6",
        "panda_joint7"
    ]

    # Map joint states to ordered vector
    joint_map = {name: i for i, name in enumerate(msg.name)}
    joint_angles = np.array([msg.position[joint_map[j]] for j in joint_order])
    
def listener():
    rospy.init_node('read_franka_topics_node')

    # Subscribe to FrankaState
    rospy.Subscriber("/franka_state_controller/franka_states", FrankaState, franka_state_callback)
    
    # Subscribe to JointState
    rospy.Subscriber("/franka_state_controller/joint_states", JointState, joint_state_callback)

    rospy.spin()

if __name__ == "__main__":
    listener()

