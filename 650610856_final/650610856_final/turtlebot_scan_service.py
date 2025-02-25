import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from myword_650610856.srv import SetState
from rclpy.qos import qos_profile_sensor_data 
import numpy as np

service_name = "turtlebot_service"

class Service(Node):
    def __init__(self):
        super().__init__(service_name)
        self.srv = self.create_service(
            SetState, 
            'obstacle_state', 
            self.obstacle_callback)
        self.turtle_lidar = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.scan_callback,
            qos_profile_sensor_data
            )
        
        self.state_stop = False
        self.obstacle_dist = -1.0
        self.obstacle_angle = -1.0

    def scan_callback(self, msg):
        angle_min = msg.angle_min 
        angle_max = msg.angle_max  
        ranges = msg.ranges 
        num_readings = len(ranges)
        angles = np.linspace(angle_min, angle_max, num_readings)
        angles_deg = np.degrees(angles)
        
        obstacle_detected = False

        for i in range(num_readings):
            distance = ranges[i]
            angle = angles_deg[i]
            if distance <= 0.20:
                self.obstacle_dist = distance
                self.obstacle_angle = angle
                self.state_stop = True
                obstacle_detected = True
                break

        if not obstacle_detected:
            self.state_stop = False
            self.obstacle_dist = -1.0
            self.obstacle_angle = -1.0

    def obstacle_callback(self, request, response):
        self.get_logger().info(f"angle: {self.obstacle_angle} dist: {self.obstacle_dist} state: {self.state_stop}")
        if request.data:
            response.state = self.state_stop
            response.dist = self.obstacle_dist
            response.angle = self.obstacle_angle
        return response

def main():
    rclpy.init()
    service = Service()
    rclpy.spin(service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
