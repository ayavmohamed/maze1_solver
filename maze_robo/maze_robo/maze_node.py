import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from maze_interfaces.action import MoveRobotX, RotateRobotYaw


class MazeNode(Node):

    def __init__(self):
        super().__init__('maze_node')

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_odom_time = self.get_clock().now()
        self.odom_timeout=1.0

        # Allow odometry callback and action callbacks
        # to run at the same time.
        self.callback_group = ReentrantCallbackGroup()

        # Velocity publisher
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Odometry subscriber
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_cb,
            10,
            callback_group=self.callback_group
        )

        # Action Servers
        self.move_x_server = ActionServer(
            self,
            MoveRobotX,
            'movement_x',
            self.execute_x,
            callback_group=self.callback_group
        )

        self.rotate_yaw_server = ActionServer(
            self,
            RotateRobotYaw,
            'movement_yaw',
            self.execute_yaw,
            callback_group=self.callback_group
        )

        self.get_logger().info(
            'maze_node action servers '
            '(movement_x & movement_yaw) are ready.'
        )

    def odom_cb(self, msg):
        """Update robot position and yaw from odometry."""

        self.last_odom_time = self.get_clock().now()
        
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        self.yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    def odom_is_alive(self):
        elapsed = (
        self.get_clock().now() - self.last_odom_time).nanoseconds / 1e9
        return elapsed <= self.odom_timeout

    def execute_x(self, goal_handle):
        """Move the robot forward or backward."""

        target = goal_handle.request.distance

        if target == 0.0:
            goal_handle.succeed()
            return MoveRobotX.Result(
                success=True,
                message='No movement required.'
            )

        start_x = self.x
        start_y = self.y

        cmd = Twist()
        cmd.linear.x = math.copysign(0.5, target)


        while rclpy.ok():

            if not self.odom_is_alive():
                self.cmd_pub.publish(Twist())
                goal_handle.abort()
                return MoveRobotX.Result(
                    success=False,
                    message='Odometry timeout. Movement aborted.')

            traveled = math.hypot(
                self.x - start_x,
                self.y - start_y
            )

            feedback = MoveRobotX.Feedback()
            feedback.current_distance = traveled
            goal_handle.publish_feedback(feedback)

            if traveled >= abs(target):
                self.cmd_pub.publish(Twist())

                goal_handle.succeed()

                return MoveRobotX.Result(
                    success=True,
                    message='Movement completed.'
                )

            self.cmd_pub.publish(cmd)

            time.sleep(0.01)

        self.cmd_pub.publish(Twist())

        return MoveRobotX.Result(
            success=False,
            message='Movement interrupted.'
        )

    def normalize_angle(self, angle):
        return math.atan2(
            math.sin(angle),
            math.cos(angle)
        )

    def execute_yaw(self, goal_handle):
        """Rotate the robot around the Z-axis."""

        target = goal_handle.request.yaw

        if target == 0.0:
            goal_handle.succeed()
            return RotateRobotYaw.Result(
                success=True,
                message='No rotation required.'
            )

        start_yaw = self.yaw

        cmd = Twist()
        cmd.angular.z = math.copysign(0.4, target)


        while rclpy.ok():

            if not self.odom_is_alive():
                self.cmd_pub.publish(Twist())
                goal_handle.abort()
                return RotateRobotYaw.Result(
                    success=False,
                    message='Odometry timeout. Rotation aborted.')

            turned = abs(
                self.normalize_angle(
                    self.yaw - start_yaw
                )
            )

            feedback = RotateRobotYaw.Feedback()
            feedback.current_yaw = turned
            goal_handle.publish_feedback(feedback)

            if turned >= abs(target):
                self.cmd_pub.publish(Twist())

                goal_handle.succeed()

                return RotateRobotYaw.Result(
                    success=True,
                    message='Rotation completed.'
                )

            self.cmd_pub.publish(cmd)

            time.sleep(0.01)

        self.cmd_pub.publish(Twist())

        return RotateRobotYaw.Result(
            success=False,
            message='Rotation interrupted.'
        )





def main(args=None):
    rclpy.init(args=args)

    node = MazeNode()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Publish zero velocity BEFORE shutting down the executor or rclpy context
        if rclpy.ok():
            node.cmd_pub.publish(Twist())

        executor.shutdown()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()