import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import SetBool

from maze_interfaces.action import MoveRobotX, RotateRobotYaw


class MazeActionClient(Node):

    def __init__(self):
        super().__init__("maze_action_client")

        # Wall service
        self.wall_client = self.create_client(
            SetBool,
            "/toggle_walls_1_2"
        )

        # Action clients
        self.move_x_client = ActionClient(
            self,
            MoveRobotX,
            "movement_x"
        )

        self.move_yaw_client = ActionClient(
            self,
            RotateRobotYaw,
            "movement_yaw"
        )

        self.get_logger().info("Maze Action Client is ready.")

    def open_wall(self, open_wall=True):

        if not self.wall_client.wait_for_service(timeout_sec=5.0):

            self.get_logger().error(
                "/toggle_walls_1_2 service is not available."
            )

            return False

        request = SetBool.Request()
        request.data = open_wall

        self.get_logger().info(
            f"Sending wall command: {open_wall}"
        )

        future = self.wall_client.call_async(request)

        rclpy.spin_until_future_complete(
            self,
            future
        )

        response = future.result()

        if response is None:

            self.get_logger().error(
                "No response from wall service."
            )

            return False

        if response.success:

            self.get_logger().info(
                f"Wall service succeeded: {response.message}"
            )

            return True

        self.get_logger().error(
            f"Wall service failed: {response.message}"
        )

        return False

    def move_x(self, distance):

        self.get_logger().info(
            f"Sending move_x goal: {distance} m"
        )

        if not self.move_x_client.wait_for_server(
            timeout_sec=5.0
        ):

            self.get_logger().error(
                "movement_x action server not available."
            )

            return False

        goal_msg = MoveRobotX.Goal()

        # MoveRobotX.action contains: float64 distance
        goal_msg.distance = distance

        send_goal_future = self.move_x_client.send_goal_async(
            goal_msg
        )

        rclpy.spin_until_future_complete(
            self,
            send_goal_future
        )

        goal_handle = send_goal_future.result()

        if goal_handle is None or not goal_handle.accepted:

            self.get_logger().error(
                "move_x goal rejected."
            )

            return False

        self.get_logger().info(
            "move_x goal accepted."
        )

        result_future = goal_handle.get_result_async()

        rclpy.spin_until_future_complete(
            self,
            result_future
        )

        result = result_future.result().result

        if result.success:

            self.get_logger().info(
                f"move_x succeeded: {result.message}"
            )

        else:

            self.get_logger().error(
                f"move_x failed: {result.message}"
            )

        return result.success

    def rotate_yaw(self, angle):

        self.get_logger().info(
            f"Sending rotate_yaw goal: {angle} rad"
        )

        if not self.move_yaw_client.wait_for_server(
            timeout_sec=5.0
        ):

            self.get_logger().error(
                "movement_yaw action server not available."
            )

            return False

        goal_msg = RotateRobotYaw.Goal()

        # RotateRobotYaw.action contains: float64 yaw
        goal_msg.yaw = angle

        send_goal_future = self.move_yaw_client.send_goal_async(
            goal_msg
        )

        rclpy.spin_until_future_complete(
            self,
            send_goal_future
        )

        goal_handle = send_goal_future.result()

        if goal_handle is None or not goal_handle.accepted:

            self.get_logger().error(
                "rotate_yaw goal rejected."
            )

            return False

        self.get_logger().info(
            "rotate_yaw goal accepted."
        )

        result_future = goal_handle.get_result_async()

        rclpy.spin_until_future_complete(
            self,
            result_future
        )

        result = result_future.result().result

        if result.success:

            self.get_logger().info(
                f"rotate_yaw succeeded: {result.message}"
            )

        else:

            self.get_logger().error(
                f"rotate_yaw failed: {result.message}"
            )

        return result.success

    def solve_maze(self):
        self.get_logger().info("Starting maze solving...")

      

    # 1. Move a small distance
        if not self.move_x(0.2):
             self.get_logger().error("First movement failed.")
             return False

    # 2. Turn LEFT 90 degrees
        if not self.rotate_yaw(1.5708):
          self.get_logger().error("First rotation failed.")
          return False
        
    # 3 . Open the first gate
        if not self.open_wall(True):
            self.get_logger().error("Could not open Gate 1.")
            return False
        
    # 4. Move toward the second gate
        if not self.move_x(1.2):
          self.get_logger().error("Movement toward Gate 2 failed.")
          return False

    # 5. Open the second gate
        if not self.open_wall(False):
            self.get_logger().error("Could not open Gate 2.")
            return False

    # 6. Move a very small distance
        if not self.move_x(0.9):
             self.get_logger().error("Small movement failed.")
             return False

    # 7. Turn RIGHT 90 degrees
        if not self.rotate_yaw(-1.5708):
             self.get_logger().error("Second rotation failed.")
             return False

    # 8. Move straight toward the finish
        if not self.move_x(4.3):
              self.get_logger().error("Final movement failed.")
              return False

        self.get_logger().info("Maze completed successfully!")

        return True

def main(args=None):

    rclpy.init(args=args)

    client = MazeActionClient()

    try:
        client.solve_maze()

    except KeyboardInterrupt:
        pass

    finally:
        client.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
