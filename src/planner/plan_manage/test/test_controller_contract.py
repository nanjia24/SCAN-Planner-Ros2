"""Isolated synthetic controller contract test; no physical execution claim."""
import math
import time
import unittest
import launch
import launch_ros.actions
import launch_testing.actions
import pytest
import rclpy
from rclpy.duration import Duration
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from gbplanner3_interfaces.msg import ControllerCommand as Command, ControllerAck as Ack


@pytest.mark.launch_test
def generate_test_description():
    controller = launch_ros.actions.Node(
        package='scan_planner', executable='closed_loop_controller',
        namespace='controller_contract_test', parameters=[{'contract_mode': True, 'allow_goal_handoff': True, 'terminal_speed_limit': True}], output='screen')
    return launch.LaunchDescription([controller, launch_testing.actions.ReadyToTest()])


class ControllerContract(unittest.TestCase):
    def test_cancel_replay_pose_and_expiry(self):
        rclpy.init()
        node = rclpy.create_node('controller_contract_test_client')
        prefix = '/controller_contract_test/'
        results, commands = [], []
        node.create_subscription(Ack, prefix+'planning/controller_ack', results.append, 10)
        node.create_subscription(Twist, prefix+'planning/controller_cmd_vel', commands.append, 10)
        pub = node.create_publisher(Command, prefix+'planning/controller_command', 10)
        odom_pub = node.create_publisher(Odometry, prefix+'body_pose', 10)
        measured_yaw = 0.
        measured_x = 0.
        stamp_offset = -5.0
        def pump(duration=.15, odom=True):
            end = time.monotonic()+duration
            while time.monotonic() < end:
                if odom:
                    msg = Odometry()
                    msg.header.frame_id = 'camera_init'
                    msg.header.stamp = (node.get_clock().now()+Duration(seconds=stamp_offset)).to_msg()
                    msg.pose.pose.position.x = measured_x
                    msg.pose.pose.orientation.z = math.sin(measured_yaw / 2.)
                    msg.pose.pose.orientation.w = math.cos(measured_yaw / 2.)
                    odom_pub.publish(msg)
                rclpy.spin_once(node, timeout_sec=.01)
        def command(goal, action=Command.START, target=1., yaw=False):
            msg = Command()
            msg.header.frame_id = 'camera_init'
            msg.header.stamp = (node.get_clock().now()+Duration(seconds=stamp_offset)).to_msg()
            msg.exploration_session_id, msg.goal_id = 123, goal
            msg.planning_attempt_id, msg.trajectory_id = 1, goal
            msg.action = action
            msg.valid_until = (node.get_clock().now()+Duration(seconds=10)).to_msg()
            msg.target_pose.position.x = target
            msg.target_pose.orientation.w = 1.
            if yaw:
                msg.target_pose.orientation.z = 1.
                msg.target_pose.orientation.w = 0.
            msg.require_terminal_yaw = yaw
            msg.trajectory.order = 3
            msg.trajectory.traj_id = goal
            msg.trajectory.pos_pts = [Point(x=0. if target == 0. else float(i)*.1) for i in range(6)]
            msg.trajectory.knots = [float(i)*.1 for i in range(10)]
            return msg
        def send(msg, status):
            n = len(results)
            pub.publish(msg)
            pump()
            matches = [x for x in results[n:] if x.goal_id == msg.goal_id and x.status == status]
            self.assertTrue(matches, (msg.goal_id, status, [(x.status,x.reason) for x in results[n:]]))
        def assert_zero(msg):
            self.assertEqual((msg.linear.x, msg.linear.y, msg.linear.z,
                              msg.angular.x, msg.angular.y, msg.angular.z), (0.,)*6)
        try:
            pump(1.)
            self.assertGreater(pub.get_subscription_count(), 0)
            # Header clock offset is allowed while messages keep arriving.
            stamp_offset = 5.0
            pump(.2)
            send(command(100), Ack.ACCEPTED)
            pump(.6)
            self.assertFalse(any(x.goal_id == 100 and x.status == Ack.ABORTED for x in results))
            send(command(100, Command.CANCEL), Ack.CANCELLED)
            expired = command(101)
            expired.valid_until = (node.get_clock().now()-Duration(seconds=1)).to_msg()
            send(expired, Ack.REJECTED)
            stamp_offset = -5.0
            pump(.2)
            send(command(1, Command.CANCEL), Ack.CANCELLED)
            send(command(1), Ack.REJECTED)  # CANCEL before delayed START is a tombstone.
            send(command(2), Ack.ACCEPTED)
            send(command(2), Ack.ACCEPTED)  # Idempotent; implementation leaves exec_time untouched.
            self.assertEqual(results[-1].reason, "ALREADY_ACCEPTED_NO_RESTART")
            send(command(99), Ack.REJECTED)
            old_cancel_command_index = len(commands)
            send(command(1, Command.CANCEL), Ack.CANCELLED)
            # Only commands observed after old CANCEL count; old buffered movement
            # must not conceal an accidental stop of the current identity.
            after_old_cancel = commands[old_cancel_command_index:]
            self.assertTrue(after_old_cancel)
            self.assertTrue(all(abs(x.linear.x) > 0 for x in after_old_cancel))
            current_cancel_command_index = len(commands)
            send(command(2, Command.CANCEL), Ack.CANCELLED)
            self.assertGreater(len(commands), current_cancel_command_index)
            assert_zero(commands[-1])
            settled_index = len(commands)
            pump(.2)
            for msg in commands[settled_index:]:
                assert_zero(msg)
            send(command(2), Ack.REJECTED)
            send(command(99), Ack.REJECTED)  # A rejected busy identity cannot activate later.
            send(command(3, target=0., yaw=True), Ack.ACCEPTED)
            # Inside tolerance from the start: wait for spline completion.
            self.assertFalse(any(x.goal_id == 3 and x.status in
                                 (Ack.POSITION_REACHED, Ack.POSE_REACHED) for x in results))
            pump(.25)
            self.assertTrue(any(x.goal_id == 3 and x.status == Ack.POSITION_REACHED for x in results))
            self.assertFalse(any(x.goal_id == 3 and x.status == Ack.POSE_REACHED for x in results))
            self.assertGreater(abs(commands[-1].angular.z), 0.)
            self.assertLessEqual(abs(commands[-1].angular.z), .5)
            # Reaching target yaw is a measured input change, not elapsed time.
            pump(.25)
            self.assertFalse(any(x.goal_id == 3 and x.status == Ack.POSE_REACHED for x in results))
            measured_yaw = math.pi
            pump(.2)
            self.assertTrue(any(x.goal_id == 3 and x.status == Ack.POSE_REACHED for x in results))
            assert_zero(commands[-1])
            send(command(3, Command.CANCEL), Ack.CANCELLED)
            measured_yaw = 0.
            pump()
            send(command(5, target=0., yaw=True), Ack.ACCEPTED)
            pump(.25)
            self.assertGreater(abs(commands[-1].angular.z), 0.)
            self.assertLessEqual(abs(commands[-1].angular.z), .5)
            pump(.7, odom=False)
            self.assertTrue(any(x.goal_id == 5 and x.status == Ack.ABORTED for x in results))
            assert_zero(commands[-1])
            pump()
            send(command(4, target=0.), Ack.ACCEPTED)
            pump(.25)
            self.assertTrue(any(x.goal_id == 4 and x.status == Ack.POSE_REACHED for x in results))
            send(command(4, Command.CANCEL), Ack.CANCELLED)
            # Native continuous replan replaces only the same attempt. Old
            # CANCEL and malformed/newer candidates cannot stop or corrupt it.
            pump()
            first = command(6)
            send(first, Ack.ACCEPTED)
            pump(.05)
            update = command(6)
            update.trajectory_id = update.trajectory.traj_id = 60
            send(update, Ack.ACCEPTED)
            self.assertTrue(any(x.trajectory_id == 60 and x.reason == 'LIVE_TRAJECTORY_REPLACED' for x in results))
            old_cancel = command(6, Command.CANCEL)
            index = len(commands)
            send(old_cancel, Ack.CANCELLED)
            self.assertTrue(all(abs(x.linear.x) > 0 for x in commands[index:]))
            bad = command(6)
            bad.trajectory_id = bad.trajectory.traj_id = 61
            bad.trajectory.pos_pts[0].x = float('nan')
            send(bad, Ack.REJECTED)
            send(first, Ack.REJECTED)
            cancel = command(6, Command.CANCEL)
            cancel.trajectory_id = 60
            send(cancel, Ack.CANCELLED)
            assert_zero(commands[-1])
            # Native spline has finished, robot overshot by 25 cm: back up,
            # without rotating 180 degrees. Front-side error still moves forward.
            measured_yaw = 0.
            measured_x = .25
            pump(.1)
            send(command(110, target=0.), Ack.ACCEPTED)
            pump(.4)
            self.assertLess(commands[-1].linear.x, 0.)
            self.assertLessEqual(abs(commands[-1].linear.x), .2)
            self.assertAlmostEqual(commands[-1].angular.z, 0., places=5)
            self.assertEqual(commands[-1].linear.y, 0.)
            measured_x = -.25
            pump(.2)
            self.assertGreater(commands[-1].linear.x, 0.)
            self.assertAlmostEqual(commands[-1].angular.z, 0., places=5)
            measured_x = 0.
            pump(.2)
            self.assertTrue(any(x.goal_id == 110 and x.status == Ack.POSE_REACHED for x in results))
            send(command(110, Command.CANCEL), Ack.CANCELLED)
            measured_x = 0.
            pump(.1)
            send(command(120), Ack.ACCEPTED)
            replacement = command(121)
            replacement.planning_attempt_id = 2
            index = len(commands)
            send(replacement, Ack.ACCEPTED)
            self.assertTrue(all(abs(x.linear.x) > 0 for x in commands[index:]))
            index = len(commands)
            send(command(120, Command.CANCEL), Ack.CANCELLED)
            self.assertTrue(all(abs(x.linear.x) > 0 for x in commands[index:]))
            replacement.action = Command.CANCEL
            send(replacement, Ack.CANCELLED)
            assert_zero(commands[-1])
        finally:
            node.destroy_node()
            rclpy.shutdown()
