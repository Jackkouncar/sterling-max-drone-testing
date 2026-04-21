"""Shared settings for the ModalAI Starling 2 indoor flight tests."""

import os


TARGET_DRONE = "ModalAI Starling 2"
TARGET_ROS_DISTRO = "foxy"

TAKEOFF_HEIGHT_M = 1.0
TAKEOFF_Z_NED = -TAKEOFF_HEIGHT_M

# How long each horizontal transit leg takes. Longer is slower and safer indoors.
TRANSIT_DISTANCE_M = 1.5
TRANSIT_DURATION_S = 6.0

# How long to wait for the drone to stabilize at a waypoint before moving on.
HOVER_SETTLE_S = 3.0

# Landing is handed to PX4. Do not command custom near-ground offboard descent.
LAND_COMMAND_REPEAT_SECONDS = 2.0
LAND_COMPLETE_WAIT_SECONDS = 10.0


def smooth_transit_xy(elapsed_s, x_start, y_start, x_end, y_end,
                      duration_s=TRANSIT_DURATION_S):
    """
    Return an (x, y) setpoint that ramps smoothly between two points.

    This uses smoothstep so the drone starts and stops each movement leg more
    gently instead of jumping straight to the next target.
    """
    t = min(max(elapsed_s / duration_s, 0.0), 1.0)
    s = t * t * (3.0 - 2.0 * t)
    x = x_start + s * (x_end - x_start)
    y = y_start + s * (y_end - y_start)
    return x, y


def log_environment_check(node):
    """Log the ROS 2 distro so the team can catch Humble/Foxy mixups early."""
    ros_distro = os.environ.get("ROS_DISTRO")

    if ros_distro is None:
        node.get_logger().warn(
            f"ROS_DISTRO is not set. This repo is currently targeted for ROS 2 {TARGET_ROS_DISTRO}."
        )
        return

    if ros_distro != TARGET_ROS_DISTRO:
        node.get_logger().warn(
            f"ROS_DISTRO is '{ros_distro}', but these tests are targeted for ROS 2 {TARGET_ROS_DISTRO}."
        )
        return

    node.get_logger().info(f"ROS 2 distro check passed: {ros_distro}")
