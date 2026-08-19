import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# Updated to use your teammate's action interface package
from maze_interfaces.action import MoveRobotX, RotateRobotYaw


class MazeNode(Node):
    def __init__(self):
        super().__init__('maze_node')
        self.x = self.y = self.yaw = 0.0

        # Velocity publisher and Odometry subscriber
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

        # Action Servers initialized using MoveRobotX and RotateRobotYaw
        ActionServer(self, MoveRobotX, 'movement_x', self.execute_x)
        ActionServer(self, RotateRobotYaw, 'movement_yaw', self.execute_yaw)

        self.get_logger().info('maze_node action servers (movement_x & movement_yaw) are ready.')

    def odom_cb(self, msg):
        """Callback to store modern coordinates and extract yaw heading."""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        
        # Convert quaternion orientation into 2D yaw angle
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def execute_x(self, goal_handle):
        """Handles linear forward/backward movement along the X-axis."""
        target = goal_handle.request.distance
        start_x, start_y = self.x, self.y
        
        cmd = Twist()
        cmd.linear.x = math.copysign(0.5, target)

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            traveled = math.hypot(self.x - start_x, self.y - start_y)

            # Reached target distance
            if traveled >= abs(target):
                self.cmd_pub.publish(Twist())  # Stop motor movement
                goal_handle.succeed()
                return MoveRobotX.Result(success=True)

            self.cmd_pub.publish(cmd)

    def execute_yaw(self, goal_handle):
        """Handles rotational turning around the Z-axis."""
        target = goal_handle.request.angle
        start_yaw = self.yaw
        
        cmd = Twist()
        cmd.angular.z = math.copysign(0.4, target)

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            turned = abs(self.yaw - start_yaw)

            # Reached target angle
            if turned >= abs(target):
                self.cmd_pub.publish(Twist())  # Stop rotation
                goal_handle.succeed()
                return RotateRobotYaw.Result(success=True)

            self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = MazeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())  # Safety stop on shutdown
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()