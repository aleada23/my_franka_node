#!/usr/bin/env python3
import rospy
from controller_manager_msgs.srv import ListControllers, SwitchController

def start_controller_exclusive(controller_name, strictness=2):
    """
    Start ONLY the specified controller and stop all others.
    If it's already the only running controller → no-op.
    Safe for repeated calls (idempotent).
    """

    rospy.wait_for_service('/controller_manager/list_controllers')
    rospy.wait_for_service('/controller_manager/switch_controller')

    list_srv   = rospy.ServiceProxy('/controller_manager/list_controllers', ListControllers)
    switch_srv = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)

    # ---- Get active controllers ----
    controllers = list_srv().controller
    running = {c.name for c in controllers if (c.state == "running" and ("position_joint_trajectory_controller" in c.name or "cartesian_impedance_example_controller" in c.name))}

    # ---- If our controller is already the ONLY running one, do nothing ----
    #if running == {controller_name}:
    #    rospy.loginfo(f"[start_controller_exclusive] '{controller_name}' already exclusively running.")
    #    return True

    # ---- Determine which controllers must be stopped ----
    stop_list = [c for c in running if c != controller_name]

    # ---- Determine if we need to start our controller ----
    start_list = []
    if controller_name not in running:
        start_list = [controller_name]

    rospy.loginfo(f"[start_controller_exclusive] Starting: {start_list}, Stopping: {stop_list}")

    # ---- Perform the switch ----
    try:
        resp = switch_srv(
            start_controllers=start_list,
            stop_controllers=stop_list,
            strictness=strictness
        )
        if resp.ok:
            rospy.loginfo("[start_controller_exclusive] Switch OK.")
            return True
        else:
            rospy.logerr("[start_controller_exclusive] Switch FAILED.")
            return False

    except rospy.ServiceException as e:
        rospy.logerr(f"[start_controller_exclusive] Service call failed: {e}")
        return False

def wait_until_running(controller_name, timeout=5.0, rate_hz=20):
    """
    Wait until a controller reaches the 'running' state.

    Args:
        controller_name: name of the controller
        timeout: max time (seconds) to wait
        rate_hz: polling rate for checking controller state
    """
    rospy.wait_for_service('/controller_manager/list_controllers')
    list_srv = rospy.ServiceProxy('/controller_manager/list_controllers', ListControllers)

    rate = rospy.Rate(rate_hz)
    start_time = rospy.get_time()

    while not rospy.is_shutdown():
        # timeout reached?
        if rospy.get_time() - start_time > timeout:
            rospy.logwarn(f"[wait_until_running] Timeout: {controller_name} not running after {timeout}s.")
            return False

        try:
            controllers = list_srv().controller
            for c in controllers:
                if c.name == controller_name and c.state == "running":
                    rospy.loginfo(f"[wait_until_running] Controller '{controller_name}' is running.")
                    return True
        except rospy.ServiceException as e:
            rospy.logerr(f"[wait_until_running] Service error: {e}")
            return False

        rate.sleep()