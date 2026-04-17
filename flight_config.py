"""Shared settings for the ModalAI Starling 2 indoor flight tests."""

import os


TARGET_DRONE = "ModalAI Starling 2"
TARGET_ROS_DISTRO = "foxy"

TAKEOFF_HEIGHT_M = 1.0
TAKEOFF_Z_NED = -TAKEOFF_HEIGHT_M

TRANSIT_DISTANCE_M = 1.5

SOFT_LAND_FINAL_HEIGHT_M = 0.15
SOFT_LAND_FINAL_Z_NED = -SOFT_LAND_FINAL_HEIGHT_M
SOFT_LAND_DESCENT_SECONDS = 5.0
LAND_COMMAND_SECONDS = 4.0


def soft_landing_z(elapsed_seconds):
    """Ramp from takeoff height down close to the floor before PX4 land mode."""
    progress = min(max(elapsed_seconds / SOFT_LAND_DESCENT_SECONDS, 0.0), 1.0)
    return TAKEOFF_Z_NED + progress * (SOFT_LAND_FINAL_Z_NED - TAKEOFF_Z_NED)


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
