import time
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

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            10
        )

        self.odom = None

        cb_group = ReentrantCallbackGroup()

        self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10,
            callback_group=cb_group
        )

        self.move_x_server = ActionServer(
            self,
            MoveRobotX,
            "movement_x",
            self.execute_move_x,
            callback_group=cb_group
        )

        self.move_yaw_server = ActionServer(
            self,
            RotateRobotYaw,
            "movement_yaw",
            self.execute_move_yaw,
            callback_group=cb_group
        )

        self.get_logger().info(
            "movement_x and movement_yaw servers ready"
        )

    def get_current_time_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def odom_callback(self, msg):
        self.odom = msg

    def stop(self):
        self.cmd_vel_pub.publish(Twist())

    # ============================================================
    # MEMBER 2 - LINEAR DISTANCE PID
    # ============================================================

    async def execute_move_x(self, goal_handle):

        self.get_logger().info("move_x goal received")

        result = MoveRobotX.Result()

        # --------------------------------------------------------
        # Wait for odometry
        # --------------------------------------------------------

        start_wait = self.get_current_time_sec()

        while self.odom is None:

            if self.get_current_time_sec() - start_wait > 5.0:

                self.get_logger().error(
                    "No odometry received, aborting move_x"
                )

                goal_handle.abort()

                result.success = False
                result.message = "Odometry not received."

                return result

            time.sleep(0.05)

        # --------------------------------------------------------
        # Starting position
        # --------------------------------------------------------

        start_x = self.odom.pose.pose.position.x
        start_y = self.odom.pose.pose.position.y

        target = goal_handle.request.distance

        # --------------------------------------------------------
        # No movement required
        # --------------------------------------------------------

        if target == 0.0:

            goal_handle.succeed()

            result.success = True
            result.message = "No movement required."

            return result

        # --------------------------------------------------------
        # Target information
        # --------------------------------------------------------

        target_distance = abs(target)

        # +1 for forward
        # -1 for backward
        direction = math.copysign(1.0, target)

        # ========================================================
        # MEMBER 5 WILL MODIFY THIS SECTION
        # ========================================================
        #
        # These are currently fixed values.
        #
        # Member 5 should replace them with ROS2 parameters:
        #
        #   linear_kp
        #   linear_ki
        #   linear_kd
        #
        # and add/update the required Watchdog Timer.
        #
        # ========================================================

        Kp = 1.0
        Ki = 0.0
        Kd = 0.1

        deadzone = 0.02
        max_output = 0.5
        integral_limit = 1.0

        # ========================================================
        # END OF MEMBER 5 SECTION
        # ========================================================

        # --------------------------------------------------------
        # PID state variables
        # --------------------------------------------------------

        integral = 0.0

        previous_error = target_distance

        previous_time = self.get_current_time_sec()

        feedback = MoveRobotX.Feedback()

        start_time = self.get_current_time_sec()

        last_progress = 0.0
        last_progress_time = self.get_current_time_sec()

        # --------------------------------------------------------
        # PID CONTROL LOOP
        # --------------------------------------------------------

        while rclpy.ok():

            now = self.get_current_time_sec()

            # ====================================================
            # 1. Get current position from /odom
            # ====================================================

            current_x = self.odom.pose.pose.position.x
            current_y = self.odom.pose.pose.position.y

            # ====================================================
            # 2. Calculate traveled distance
            # ====================================================

            distance = math.sqrt(
                (current_x - start_x) ** 2
                + (current_y - start_y) ** 2
            )

            # ====================================================
            # 3. Calculate error
            # ====================================================

            error = target_distance - distance

            # ====================================================
            # 4. TARGET DEADZONE
            # ====================================================

            if abs(error) < deadzone:

                self.get_logger().info(
                    f"Target reached. Error = {error:.4f} m"
                )

                self.stop()

                break

            # ====================================================
            # 5. Calculate dt
            # ====================================================

            dt = now - previous_time

            if dt <= 0.0:
                dt = 0.001

            # ====================================================
            # 6. ZERO-CROSSING RESET
            # ====================================================

            if previous_error * error < 0.0:

                integral = 0.0

            # ====================================================
            # 7. DERIVATIVE
            # ====================================================

            derivative = (error - previous_error) / dt

            # ====================================================
            # 8. CONDITIONAL INTEGRATION
            # ====================================================

            #
            # First calculate the PID output WITHOUT
            # adding the new integral.
            #

            proportional = Kp * error

            derivative_term = Kd * derivative

            output_without_integral = (
                proportional
                + derivative_term
                + Ki * integral
            )

            #
            # Calculate what the integral WOULD become.
            #

            candidate_integral = integral + error * dt

            #
            # Integral Anti-Windup:
            # limit the integral itself.
            #

            candidate_integral = max(
                -integral_limit,
                min(candidate_integral, integral_limit)
            )

            #
            # Calculate the output if we allow integration.
            #

            candidate_output = (
                proportional
                + Ki * candidate_integral
                + derivative_term
            )

            #
            # Conditional Integration:
            #
            # If output is already saturated at the upper limit
            # AND the error would make the saturation worse,
            # DO NOT integrate.
            #

            if (
                candidate_output > max_output
                and error > 0.0
            ):

                # Hold the previous integral value
                integral = integral

            else:

                # Accept the new integral
                integral = candidate_integral

            # ====================================================
            # 9. FINAL PID OUTPUT
            # ====================================================

            output = (
                Kp * error
                + Ki * integral
                + Kd * derivative
            )

            # ====================================================
            # 10. OUTPUT CLAMPING
            # ====================================================

            output = max(
                0.0,
                min(output, max_output)
            )

            # ====================================================
            # 11. APPLY DIRECTION
            # ====================================================

            linear_velocity = direction * output

            # ====================================================
            # 12. Publish feedback
            # ====================================================

            feedback.current_distance = distance

            goal_handle.publish_feedback(feedback)

            # ====================================================
            # 13. Publish /cmd_vel
            # ====================================================

            twist = Twist()

            twist.linear.x = linear_velocity

            self.cmd_vel_pub.publish(twist)

            # ====================================================
            # 14. Progress Watch
            # ====================================================

            if distance - last_progress > 0.005:

                last_progress = distance
                last_progress_time = now

            elif now - last_progress_time > 5.0:

                self.get_logger().error(
                    "move_x stalled: no progress"
                )

                self.stop()

                goal_handle.abort()

                result.success = False
                result.message = "Robot stalled."

                return result

            # ====================================================
            # 15. Overall Timeout
            # ====================================================

            if now - start_time > 30.0:

                self.get_logger().error(
                    "move_x timed out"
                )

                self.stop()

                goal_handle.abort()

                result.success = False
                result.message = "Movement timed out."

                return result

            # ====================================================
            # 16. Update PID state
            # ====================================================

            previous_error = error
            previous_time = now

            time.sleep(0.05)

        # --------------------------------------------------------
        # Stop robot
        # --------------------------------------------------------

        self.stop()

        goal_handle.succeed()

        result.success = True
        result.message = "Movement completed successfully."

        return result

    # ============================================================
    # MEMBER 3 - ROTATIONAL YAW
    # ============================================================

    async def execute_move_yaw(self, goal_handle):

        self.get_logger().info("move_yaw goal received")

        result = RotateRobotYaw.Result()

        # Wait for odometry
        start_wait = self.get_current_time_sec()

        while self.odom is None:

            if self.get_current_time_sec() - start_wait > 5.0:

                self.get_logger().error(
                    "No odometry received, aborting move_yaw"
                )

                goal_handle.abort()

                result.success = False
                result.message = "Odometry not received."

                return result

            time.sleep(0.05)

        start_yaw = get_yaw(self.odom)

        target = goal_handle.request.yaw

        # No rotation required
        if target == 0.0:

            goal_handle.succeed()

            result.success = True
            result.message = "No rotation required."

            return result

        # Direction depends on target sign
        twist = Twist()
        twist.angular.z = math.copysign(0.7, target)

        feedback = RotateRobotYaw.Feedback()

        start_time = self.get_current_time_sec()

        last_progress = 0.0
        last_progress_time = self.get_current_time_sec()

        while rclpy.ok():

            now = self.get_current_time_sec()

            # Calculate rotation
            rotated = get_yaw(self.odom) - start_yaw

            if rotated > math.pi:
                rotated -= 2.0 * math.pi

            if rotated < -math.pi:
                rotated += 2.0 * math.pi

            rotated = abs(rotated)

            # Check progress
            if rotated - last_progress > 0.005:

                last_progress = rotated
                last_progress_time = now

            elif now - last_progress_time > 5.0:

                self.get_logger().error(
                    "move_yaw stalled: no progress"
                )

                self.stop()

                goal_handle.abort()

                result.success = False
                result.message = "Robot rotation stalled."

                return result

            # Check target reached
            if rotated >= abs(target):

                break

            # Overall timeout
            if now - start_time > 30.0:

                self.get_logger().error(
                    "move_yaw timed out"
                )

                self.stop()

                goal_handle.abort()

                result.success = False
                result.message = "Rotation timed out."

                return result

            # Publish feedback
            feedback.current_yaw = rotated
            goal_handle.publish_feedback(feedback)

            # Rotate robot
            self.cmd_vel_pub.publish(twist)

            time.sleep(0.05)

        # Stop robot
        self.stop()

        goal_handle.succeed()

        result.success = True
        result.message = "Rotation completed successfully."

        return result


def main():

    rclpy.init()

    node = MovementServer()

    executor = MultiThreadedExecutor()

    executor.add_node(node)

    try:
        executor.spin()

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()