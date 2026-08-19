import math
import time # import this to use (sleep and timeout)
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist #Twist is for sending velocity commands, and Odometry is for reading the robot's position and orientation.
from nav_msgs.msg import Odometry
from maze_interfaces.action import MoveRobot, MoveRobot_yaw


def get_yaw(odom): #helper function that takes an Odometry message as input.
    q = odom.pose.pose.orientation  #xtracts the robot's orientation, which ROS 2 stores as a quaternion (x, y, z, w).
    siny = 2.0 * (q.w * q.z + q.x * q.y)   #Converts the complex 3D quaternion math into a simple 2D Euler angle (Yaw/heading) in radians. It calculates the rotation around the Z-axis.
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class MovementServer(Node):
    def __init__(self):
        super().__init__("movement_server")

        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10) #Creates a publisher to send velocity commands to the /cmd_vel topic with a queue size of 10.
        self.odom = None # I set at as none to prevent any errors in the start of the code 
        cb_group = ReentrantCallbackGroup() # it makes parael call backs togeher to not make the odometry stop or get blocks in any time 

        self.create_subscription(Odometry, "/odom", self.odom_callback, 10, callback_group=cb_group)

        self.move_x_server = ActionServer(self, MoveRobot, "movement_x", self.execute_move_x, callback_group=ReentrantCallbackGroup())
        self.move_yaw_server = ActionServer(self, MoveRobot_yaw, "movement_yaw", self.execute_move_yaw, callback_group=ReentrantCallbackGroup())

        self.get_logger().info("movement_x and movement_yaw servers ready") #Prints a message to the console confirming the node is ready.

    def odom_callback(self, msg):#Every time the robot publishes to /odom, this saves the latest message to self.odom.
        self.odom = msg

    def wait_for_odom(self):
        start = time.time() #It records the current time
        while self.odom is None:
            if time.time() - start > 2.0:
                return False # it return false that means that will send a message (no odom ,aborting)
            time.sleep(0.1)
        return True

    def stop(self): #Creates an empty Twist message (which defaults to 0.0 for all velocities) and publishes it, forcing the robot to stop.
        self.cmd_vel_pub.publish(Twist())

    def execute_move_x(self, goal_handle):
        self.get_logger().info("move_x goal received")
        result = MoveRobot.Result()#Prepares an empty result object to send back when finished.

        if not self.wait_for_odom():
            self.get_logger().error("no odom, aborting")
            goal_handle.abort()
            result.success = False
            result.message = "no odom received"
            return result

        start_x = self.odom.pose.pose.position.x
        start_y = self.odom.pose.pose.position.y
        target = goal_handle.request.distance  #Extracts the requested travel distance from the client's action request.

        twist = Twist()
        twist.linear.x = 0.15
        feedback = MoveRobot.Feedback()

        start_time = time.time()
        last_progress = 0.0         
        last_progress_time = time.time() 

        while True:
            distance = math.sqrt((self.odom.pose.pose.position.x - start_x) ** 2 +
                                  (self.odom.pose.pose.position.y - start_y) ** 2) #Uses the Pythagorean theorem ($a^2 + b^2 = c^2$) to calculate the straight-line distance between the starting coordinates and the current coordinates.

            if distance - last_progress > 0.01:
               last_progress = distance
               last_progress_time = time.time()
            elif time.time() - last_progress_time > 2.0:
                self.stop()
                goal_handle.abort()
                result.success = False
                result.message = "stalled: no progress despite active odom"
                return result
           
            if distance >= target:
                break

            if time.time() - start_time > 15.0:
                self.get_logger().error("move_x timed out")
                self.stop()
                goal_handle.abort()
                result.success = False
                result.message = "timed out"
                return result

            feedback.distance_travelled = distance #  Updates the feedback message with the current distance travelled.
            goal_handle.publish_feedback(feedback) #Publishes the feedback to the client.
            self.cmd_vel_pub.publish(twist) #Publishes the 0.15 m/s velocity command to keep the robot moving.
            time.sleep(0.1) #Sleeps for 0.1 seconds before checking again.

        self.stop()
        goal_handle.succeed()
        result.success = True
        result.message = "reached target distance"
        return result

    def execute_move_yaw(self, goal_handle):
        self.get_logger().info("move_yaw goal received")
        result = MoveRobot_yaw.Result()

        if not self.wait_for_odom():
            self.get_logger().error("no odom, aborting")
            goal_handle.abort()
            result.success = False
            result.message = "no odom received"
            return result

        start_yaw = get_yaw(self.odom)
        target = goal_handle.request.angle

        twist = Twist()
        twist.angular.z = 0.5
        feedback = MoveRobot_yaw.Feedback()

        start_time = time.time()
        last_progress = 0.0          
        last_progress_time = time.time() 

        while True:
            rotated = get_yaw(self.odom) - start_yaw
            if rotated > math.pi:
                rotated -= 2 * math.pi
            if rotated < -math.pi:
                rotated += 2 * math.pi
            rotated = abs(rotated)

            if rotated - last_progress > 0.01:
              last_progress = rotated
              last_progress_time = time.time()
            elif time.time() - last_progress_time > 2.0:
                self.stop()
                goal_handle.abort()
                result.success = False
                result.message = "stalled: no progress despite active odom"
                return result

            if rotated >= abs(target):
                break

            if time.time() - start_time > 15.0:
                self.get_logger().error("move_yaw timed out")
                self.stop()
                goal_handle.abort()
                result.success = False
                result.message = "timed out"
                return result

            feedback.angle_travelled = rotated
            goal_handle.publish_feedback(feedback)
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)

        self.stop()
        goal_handle.succeed()
        result.success = True
        result.message = "reached target angle"
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