"""Synthetic final-goal braking checks; isolated ROS domain, no hardware commands."""
import time
import unittest
import launch
import launch_ros.actions
import launch_testing.actions
import pytest
import rclpy
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from gbplanner3_interfaces.msg import ScanTrajectory

@pytest.mark.launch_test
def generate_test_description():
    node = launch_ros.actions.Node(package='scan_planner', executable='closed_loop_controller',
        namespace='terminal_brake_test', parameters=[{
            'terminal_goal_yaw': True, 'terminal_speed_limit': True,
            'max_vx': 1., 'max_vy': 0., 'kp_pos': .8}], output='screen')
    return launch.LaunchDescription([node, launch_testing.actions.ReadyToTest()])

class TerminalController(unittest.TestCase):
    def test_final_only_replan_and_new_goal(self):
        rclpy.init()
        node = rclpy.create_node('terminal_brake_probe')
        commands = []
        sub = node.create_subscription(Twist, '/terminal_brake_test/cmd_vel', commands.append, 20)
        odom_pub = node.create_publisher(Odometry, '/terminal_brake_test/body_pose', 10)
        pub = node.create_publisher(ScanTrajectory, '/terminal_brake_test/planning/goal_trajectory', 10)
        x = .595
        def pump(duration):
            end = time.monotonic()+duration
            while time.monotonic()<end:
                odom = Odometry()
                odom.header.frame_id='camera_init'
                odom.header.stamp=node.get_clock().now().to_msg()
                odom.pose.pose.position.x=x
                odom.pose.pose.orientation.w=1.
                odom_pub.publish(odom)
                rclpy.spin_once(node,timeout_sec=.005)
        def send(goal, identity):
            msg=ScanTrajectory()
            msg.result.header.frame_id='camera_init'
            msg.result.header.stamp=node.get_clock().now().to_msg()
            msg.result.status=msg.result.READY
            msg.result.effective_goal_valid=True
            msg.result.effective_goal.position.x=goal
            msg.result.requested_goal.orientation.w=1.
            msg.trajectory.order=3
            msg.trajectory.traj_id=identity
            msg.trajectory.pos_pts=[Point(x=i*.2) for i in range(6)]
            msg.trajectory.knots=[i*.1 for i in range(10)]
            pub.publish(msg)
            pump(.08)
            commands.clear()
            pump(.08)
            self.assertTrue(commands)
            return [abs(c.linear.x) for c in commands]
        try:
            pump(1.)
            self.assertGreater(pub.get_subscription_count(),0)
            # Spline endpoint .8 is only a local horizon for goal 2.0.
            self.assertGreater(max(send(2.,1)),.7)
            # Same spline now terminates at final goal .8: feedforward is capped.
            speeds=send(.8,2)
            self.assertGreater(max(speeds),.1)
            self.assertLessEqual(max(speeds),.313)
            self.assertLessEqual(max(send(.8,3)),.313) # Replanning cannot reset cap.
            x=.78
            self.assertLess(max(send(.8,4)),1e-6)
            x=.595
            self.assertGreater(max(send(2.,5)),.7) # New distant goal is not latched stopped.
        finally:
            node.destroy_node()
            rclpy.shutdown()
