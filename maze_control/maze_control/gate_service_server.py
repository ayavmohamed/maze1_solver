#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from maze_interfaces.srv import CheckDoor
from std_srvs.srv import SetBool 

class GateServiceServer(Node):
    def __init__(self):
        super().__init__('gate_service_server')
        
        
        self.door_states = {
            "red_wall_1": False,
            "red_wall_2": False,
        }
        
        
        self.srv = self.create_service(
            CheckDoor, 
            'check_door', 
            self.check_door_callback
        )
        
        
        self.update_srv = self.create_service(
            SetBool,
            '/set_door_state',   
            self.update_door_callback
        )
        
        self.get_logger().info('Gate Service Server is ready!')
        self.get_logger().info(f'Initial doors: {self.door_states}')

   
    def check_door_callback(self, request, response):
        door_name = request.door_name
        if door_name in self.door_states:
            is_open = self.door_states[door_name]
            response.is_open = is_open
            response.message = f"Door '{door_name}' is {'OPEN' if is_open else 'CLOSED'}"
        else:
            response.is_open = False
            response.message = f"Door '{door_name}' not found!"
        self.get_logger().info(f'Check request: {door_name} -> {response.message}')
        return response

 
    def update_door_callback(self, request, response):
        
        new_state = request.data
        
        self.door_states["red_wall_1"] = new_state
        self.door_states["red_wall_2"] = new_state
        
        self.get_logger().info(f'🔄 State updated: Both walls set to {new_state}')
        
        response.success = True
        response.message = f"Door states updated to {'OPEN' if new_state else 'CLOSED'}"
        return response

def main():
    rclpy.init()
    node = GateServiceServer()
    rclpy.spin(node)

if __name__== '__main__':
    main()