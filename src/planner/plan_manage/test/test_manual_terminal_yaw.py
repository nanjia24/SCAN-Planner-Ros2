"""Opt-in manual goal yaw on the actual controller with synthetic odometry.

No planner/map/physical-motion acceptance is claimed by this interface test.
"""
import math
import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import pytest
import rclpy
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from gbplanner3_interfaces.msg import ScanTrajectory, PlanningResult

PREFIX = '/manual_terminal_yaw_test'


@pytest.mark.launch_test
def generate_test_description():
    controller = launch_ros.actions.Node(
        package='scan_planner', executable='closed_loop_controller',
        namespace='manual_terminal_yaw_test',
        parameters=[{'terminal_goal_yaw': True, 'contract_mode': False}],
        remappings=[('cmd_vel', PREFIX+'/isolated_cmd_vel')], output='screen')
    return launch.LaunchDescription([controller, launch_testing.actions.ReadyToTest()])


class ManualTerminalYaw(unittest.TestCase):
    def test_final_segment_replacement_emergency_and_stale_pose(self):
        rclpy.init()
        node = rclpy.create_node('manual_terminal_yaw_client')
        commands = []
        node.create_subscription(Twist, PREFIX+'/isolated_cmd_vel', commands.append, 20)
        envelope_pub = node.create_publisher(ScanTrajectory, PREFIX+'/planning/goal_trajectory', 10)
        odom_pub = node.create_publisher(Odometry, PREFIX+'/body_pose', 10)
        measured_yaw = 0.
        trajectory_id = 0

        def pump(duration=.25, publish_odom=True):
            deadline = time.monotonic()+duration
            while time.monotonic() < deadline:
                if publish_odom:
                    msg = Odometry()
                    msg.header.frame_id = 'camera_init'
                    msg.header.stamp = node.get_clock().now().to_msg()
                    msg.child_frame_id = 'body_center'
                    msg.pose.pose.orientation.z = math.sin(measured_yaw/2.)
                    msg.pose.pose.orientation.w = math.cos(measured_yaw/2.)
                    odom_pub.publish(msg)
                rclpy.spin_once(node, timeout_sec=.01)

        def send(yaw, final_x=0., emergency=False):
            nonlocal trajectory_id
            trajectory_id += 1
            msg = ScanTrajectory()
            msg.result.header.frame_id = 'camera_init'
            msg.result.header.stamp = node.get_clock().now().to_msg()
            msg.result.status = PlanningResult.ABORTED if emergency else PlanningResult.READY
            msg.result.effective_goal_valid = not emergency
            msg.result.effective_goal.position.x = final_x
            msg.result.effective_goal.orientation.w = 1.
            msg.result.requested_goal.position.x = final_x
            msg.result.requested_goal.orientation.z = math.sin(yaw/2.)
            msg.result.requested_goal.orientation.w = math.cos(yaw/2.)
            msg.result.trajectory_id = trajectory_id
            msg.result.trajectory_start_time = node.get_clock().now().to_msg()
            msg.result.trajectory_end = Point()
            msg.trajectory.traj_id = trajectory_id
            msg.trajectory.start_time = msg.result.trajectory_start_time
            msg.trajectory.order = 3
            msg.trajectory.knots = [float(i)*.1 for i in range(10)]
            # A stationary local endpoint isolates terminal-yaw handling from
            # path-tangent steering. It is explicitly a synthetic test spline.
            msg.trajectory.pos_pts = [Point() for _ in range(6)]
            start = len(commands)
            envelope_pub.publish(msg)
            pump()
            self.assertGreater(len(commands), start)
            return commands[start:]

        def assert_zero(msg):
            self.assertEqual((msg.linear.x, msg.linear.y, msg.linear.z,
                              msg.angular.x, msg.angular.y, msg.angular.z), (0.,)*6)

        try:
            pump(1.)
            self.assertGreater(envelope_pub.get_subscription_count(), 0)
            intermediate = send(math.pi, final_x=1.)
            # Robot is at the local segment end but NOT the final goal. The
            # final observation yaw must not be applied at this waypoint.
            self.assertTrue(all(abs(x.angular.z) < 1e-9 for x in intermediate))
            final = send(math.pi)
            self.assertTrue(any(abs(x.angular.z) > .1 for x in final))
            pump(.2)
            self.assertGreater(abs(commands[-1].angular.z), .1)
            measured_yaw = math.pi
            pump()
            assert_zero(commands[-1])

            replacement = send(math.pi/2.)
            self.assertTrue(any(x.angular.z < -.1 for x in replacement))
            send(math.pi)  # New desired yaw equals current measured yaw.
            assert_zero(commands[-1])

            send(0.)
            self.assertGreater(abs(commands[-1].angular.z), .1)
            send(0., emergency=True)
            assert_zero(commands[-1])
            pump(.2)
            assert_zero(commands[-1])  # Old terminal-yaw state must stay cleared.

            send(0.)
            self.assertGreater(abs(commands[-1].angular.z), .1)
            pump(.8, publish_odom=False)
            assert_zero(commands[-1])
        finally:
            node.destroy_node()
            rclpy.shutdown()
