#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

class WallController(Node):
    def __init__(self):
        super().__init__('wall_controller')
        
        
        self.examiner_client = self.create_client(SetBool, '/toggle_walls_1_2')
        
        # Client to update our own server (/set_door_state)
        self.update_server_client = self.create_client(SetBool, '/set_door_state')
        
        # Wait for services to be available
        while not self.examiner_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for /toggle_walls_1_2...')
        while not self.update_server_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for /set_door_state...')
        
        self.get_logger().info('Wall Controller is ready!')

    def toggle_wall(self, open_wall: bool) -> bool:
        """
        Change the wall state (open/close) in the simulation and update our server.

        Args:
            open_wall (bool): True to open, False to close.

        Returns:
            bool: True if success, False if failed.
        """
        #step1:call the  examiner service to move the wall in gazebo
        req_examiner = SetBool.Request()   #create an empty request
        req_examiner.data = open_wall      # put the value true or false into the request
        #send the request to /toggle_walls_1_2
        future_examiner = self.examiner_client.call_async(req_examiner)
        rclpy.spin_until_future_complete(self, future_examiner)
        res_examiner = future_examiner.result()
        
        if not res_examiner.success:
            self.get_logger().error(f'Examiner failed: {res_examiner.message}')
            return False
        
        self.get_logger().info(f'Examiner executed: {res_examiner.message}')
        
        # step2: update our own server /check_door with the new state
        req_update = SetBool.Request()
        req_update.data = open_wall  #same value (True for open, False for close)
        
        future_update = self.update_server_client.call_async(req_update)
        rclpy.spin_until_future_complete(self, future_update)
        res_update = future_update.result()
        
        if res_update.success:
            self.get_logger().info(f'Server updated: {res_update.message}')
        else:
            self.get_logger().error(f'Server update failed: {res_update.message}')
        
        return res_update.success

def main(args=None):
    rclpy.init(args=args)
    node = WallController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()