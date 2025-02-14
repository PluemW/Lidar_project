import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from myword_650610856.srv import SetState
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
            self.scan_callback)
        self.turtle_lidar
        
        self.state_stop = False
        self.obstacle_dist = -1.0
        self.obstacle_angle = -1.0

    def scan_callback(self, msg):
        angle_min = msg.angle_min  # Minimum angle of the scan [rad]
        angle_max = msg.angle_max  # Maximum angle of the scan [rad]
        ranges = msg.ranges  # Distance readings
        num_readings = len(ranges)
        angles = np.linspace(angle_min, angle_max, num_readings)  # Generate angles
        angles_deg = np.degrees(angles)  # Convert to degrees
        
        for i in range(num_readings):
            distance = ranges[i]
            angle = angles_deg[i]
            if distance<=0.15:
                self.obstacle_dist = distance
                self.obstacle_angle = angle
                self.state_stop = True
                return
    
    def obstacle_callback(self, request, response):
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
    