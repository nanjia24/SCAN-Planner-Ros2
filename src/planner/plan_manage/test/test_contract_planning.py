"""Actual ROS boundary rejects goals without initial odometry; no fake planning."""
import time
from pathlib import Path
import unittest
import launch
import launch_ros.actions
import launch_testing.actions
import pytest
import rclpy
from rclpy.duration import Duration
from gbplanner3_interfaces.msg import LocalGoal, PlanningResult


@pytest.mark.launch_test
def generate_test_description():
    planner = launch_ros.actions.Node(
        package='scan_planner', executable='scan_planner_node',
        parameters=[str(Path(__file__).resolve().parents[1] / 'config' / 'planner.yaml'),
                    {'fsm.navi_mode': 1, 'fsm.contract_planning_only': True}],
        output='screen')
    return launch.LaunchDescription([planner, launch_testing.actions.ReadyToTest()])


class ContractBoundary(unittest.TestCase):
    def test_missing_pose_and_duplicate_identity(self):
        rclpy.init()
        node = rclpy.create_node('contract_boundary_test')
        results = []
        sub = node.create_subscription(PlanningResult, '/planning/contract_result', results.append, 10)
        pub = node.create_publisher(LocalGoal, '/planning/local_goal', 10)
        deadline = time.monotonic() + 10
        while pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        self.assertGreater(pub.get_subscription_count(), 0)
        goal = LocalGoal()
        goal.header.frame_id = 'camera_init'
        goal.header.stamp = node.get_clock().now().to_msg()
        goal.valid_until = (node.get_clock().now() + Duration(seconds=10)).to_msg()
        goal.exploration_session_id = 123
        goal.goal_id = 4
        goal.source_path_id = 9
        goal.planning_attempt_id = 11
        goal.pose.orientation.w = 1.
        goal.planning_radius = 3.
        pub.publish(goal)
        deadline = time.monotonic() + 5
        while not results and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        self.assertEqual(len(results), 1)
        self.assertEqual(results[0].status, PlanningResult.ABORTED)
        self.assertEqual(results[0].trajectory_id, 0)
        self.assertEqual(results[0].planning_attempt_id, 11)
        self.assertEqual(results[0].exploration_session_id, 123)
        self.assertEqual(results[0].goal_id, 4)
        self.assertFalse(results[0].effective_goal_valid)
        self.assertFalse(results[0].goal_adjusted)
        self.assertEqual(results[0].requested_goal, goal.pose)
        self.assertEqual(results[0].trajectory_start_time.sec, 0)
        pub.publish(goal)
        deadline = time.monotonic() + .5
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        self.assertEqual(len(results), 1)
        node.destroy_subscription(sub)
        node.destroy_node()
        rclpy.shutdown()
