import asyncio
import math
import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from maze_interfaces.action import MoveRobotX, RotateRobotYaw


def get_yaw(odom):
    q = odom.pose.pose.orientation
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class MovementServer(Node):

    def __init__(self):
        super().__init__("movement_server")

        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)

        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.odom = None
        cb_group = ReentrantCallbackGroup()

        self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10, callback_group=cb_group
        )

        self.move_x_server = ActionServer(
            self,
            MoveRobotX,
            "movement_x",
            self.execute_move_x,
            callback_group=cb_group,
        )
        self.move_yaw_server = ActionServer(
            self,
            RotateRobotYaw,
            "movement_yaw",
            self.execute_move_yaw,
            callback_group=cb_group,
        )

        self.get_logger().info("movement_x and movement_yaw servers ready")

    def get_current_time_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def odom_callback(self, msg):
        self.odom = msg

    def stop(self):
        self.cmd_vel_pub.publish(Twist())

    async def execute_move_x(self, goal_handle):
        self.get_logger().info("move_x goal received")
        result = MoveRobotX.Result()

        start_wait = self.get_current_time_sec()
        while self.odom is None:
            if self.get_current_time_sec() - start_wait > 5.0:
                self.get_logger().error("no odom, aborting")
                goal_handle.abort()
                result.success = False
                return result
            await asyncio.sleep(0.05)

        start_x = self.odom.pose.pose.position.x
        start_y = self.odom.pose.pose.position.y
        target = goal_handle.request.distance

        twist = Twist()
        twist.linear.x = 0.2
        feedback = MoveRobotX.Feedback()

        start_time = self.get_current_time_sec()
        last_progress = 0.0
        last_progress_time = self.get_current_time_sec()

        while rclpy.ok():
            now = self.get_current_time_sec()
            distance = math.sqrt(
                (self.odom.pose.pose.position.x - start_x) ** 2
                + (self.odom.pose.pose.position.y - start_y) ** 2
            )

            if distance - last_progress > 0.005:
                last_progress = distance
                last_progress_time = now
            elif now - last_progress_time > 5.0:
                self.get_logger().error(
                    "stalled: no progress despite active odom"
                )
                self.stop()
                goal_handle.abort()
                result.success = False
                return result

            if distance >= target:
                break

            if now - start_time > 30.0:
                self.get_logger().error("move_x timed out")
                self.stop()
                goal_handle.abort()
                result.success = False
                return result

            feedback.current_distance = distance
            goal_handle.publish_feedback(feedback)
            self.cmd_vel_pub.publish(twist)

            await asyncio.sleep(0.05)

        self.stop()
        goal_handle.succeed()
        result.success = True
        return result

    async def execute_move_yaw(self, goal_handle):
        self.get_logger().info("move_yaw goal received")
        result = RotateRobotYaw.Result()

        start_wait = self.get_current_time_sec()
        while self.odom is None:
            if self.get_current_time_sec() - start_wait > 5.0:
                self.get_logger().error("no odom, aborting")
                goal_handle.abort()
                result.success = False
                return result
            await asyncio.sleep(0.05)

        start_yaw = get_yaw(self.odom)
        target = goal_handle.request.yaw

        twist = Twist()
        twist.angular.z = 0.5
        feedback = RotateRobotYaw.Feedback()

        start_time = self.get_current_time_sec()
        last_progress = 0.0
        last_progress_time = self.get_current_time_sec()

        while rclpy.ok():
            now = self.get_current_time_sec()
            rotated = get_yaw(self.odom) - start_yaw
            if rotated > math.pi:
                rotated -= 2 * math.pi
            if rotated < -math.pi:
                rotated += 2 * math.pi
            rotated = abs(rotated)

            if rotated - last_progress > 0.005:
                last_progress = rotated
                last_progress_time = now
            elif now - last_progress_time > 5.0:
                self.get_logger().error(
                    "stalled: no progress despite active odom"
                )
                self.stop()
                goal_handle.abort()
                result.success = False
                return result

            if rotated >= abs(target):
                break

            if now - start_time > 30.0:
                self.get_logger().error("move_yaw timed out")
                self.stop()
                goal_handle.abort()
                result.success = False
                return result

            feedback.current_yaw = rotated
            goal_handle.publish_feedback(feedback)
            self.cmd_vel_pub.publish(twist)

            await asyncio.sleep(0.05)

        self.stop()
        goal_handle.succeed()
        result.success = True
        return result


def main():
    rclpy.init()
    node = MovementServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()