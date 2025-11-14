#!/usr/bin/env python3
import rospy
from dynamic_reconfigure.client import Client

def franka_impedance_control():
    rospy.init_node("franka_impedance_control", anonymous=True)
    
    # Connect to the dynamic reconfigure node of the controller
    client = Client(
        "/cartesian_impedance_example_controller/dynamic_reconfigure_compliance_param_node",
        timeout=5
    )
    
    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        # Get current parameters
        params = client.get_configuration()
        translational_stiffness = params.get("translational_stiffness", None)
        rotational_stiffness = params.get("rotational_stiffness", None)
        nullspace_stiffness = params.get("nullspace_stiffness", None)
        
        print("\n=== Current Franka Impedance Parameters ===")
        print("1) Translational Stiffness:", translational_stiffness)
        print("2) Rotational Stiffness:", rotational_stiffness)
        print("3) Nullspace Stiffness:", nullspace_stiffness)
        
        # Ask user if they want to update parameters
        try:
            user_input = input("\nDo you want to update parameters? (y/n): ").strip().lower()
            if user_input == "y":
                trans_new = float(input("Enter new translational stiffness: "))
                rot_new = float(input("Enter new rotational stiffness: "))
                null_new = float(input("Enter new nullspace stiffness: "))
                
                # Update controller parameters
                client.update_configuration({
                    "translational_stiffness": trans_new,
                    "rotational_stiffness": rot_new,
                    "nullspace_stiffness": null_new
                })
                print("Parameters updated successfully!")
        except KeyboardInterrupt:
            break
        except Exception as e:
            print("Error:", e)
        
        rate.sleep()

if __name__ == "__main__":
    try:
        franka_impedance_control()
    except rospy.ROSInterruptException:
        pass
