"""Verify that SCAN starts with the native rolling exploration mode."""

import os
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


@pytest.mark.launch_test
def generate_test_description():
    config = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..", "config", "planner.yaml")
    )
    planner = launch_ros.actions.Node(
        package="scan_planner",
        executable="scan_planner_node",
        name="scan_planner_node",
        parameters=[config, {"fsm.navi_mode": 4}],
        output="screen",
    )
    return (
        launch.LaunchDescription([planner, launch_testing.actions.ReadyToTest()]),
        {"planner": planner},
    )


class TestExplorationMode(unittest.TestCase):
    def test_mode_is_ready(self, proc_output, planner):
        proc_output.assertWaitFor(
            "Exploration waypoint mode is ready", process=planner, timeout=15
        )

    def test_fsm_state_is_latched(self):
        rclpy.init()
        node = rclpy.create_node("scan_fsm_state_test")
        states = []
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        node.create_subscription(
            String, "/planning/fsm_state", lambda message: states.append(message.data), qos
        )
        try:
            for _ in range(50):
                rclpy.spin_once(node, timeout_sec=0.1)
                if states:
                    break
            self.assertEqual(states[-1], "INIT")
        finally:
            node.destroy_node()
            rclpy.shutdown()
