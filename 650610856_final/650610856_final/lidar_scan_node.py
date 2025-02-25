import rclpy
from rclpy import qos
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int64MultiArray
from myword_650610856.srv import SetState
import numpy as np

class LidarReader(Node):
    def __init__(self):
        super().__init__('lidar_reader')
        self.cli = self.create_client(SetState, 'obstacle_state')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = SetState.Request()
        
        self.move = self.create_publisher(Int64MultiArray, "/pub_move", qos_profile=qos.qos_profile_system_default)
        self.subscription = self.create_subscription(LaserScan, '/lidar_scan', self.scan_callback, 10)

        self.sum_dist = 0.0
        self.check_angle = [0, 0, 0, 0]
        self.check_stop = False
        self.stop_angle = -1.0
        self.future = None  # Track the future response
        self.create_timer(0.05, self.timer_callback)

    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)
        angles_deg = np.degrees(np.linspace(msg.angle_min, msg.angle_max, len(ranges)))
        valid_indices = np.where(ranges <= 0.25)[0]
        
        if len(valid_indices) == 0:
            self.sum_dist = 0.0
            self.check_angle = [0, 0, 0, 0]
            return

        self.sum_dist = np.mean(ranges[valid_indices])
        angles = angles_deg[valid_indices]
        self.check_angle = [0, 0, 0, 0]
        
        forward_mask = np.abs(angles) < 15
        backward_mask = np.abs(angles) > 165
        left_mask = (-105 < angles) & (angles < -75)
        right_mask = (75 < angles) & (angles < 105)
        
        if np.any(forward_mask):
            self.check_angle = [1, 0, 0, 0]  # Full-Forward
        elif np.any(backward_mask):
            self.check_angle = [0, 1, 0, 0]  # Full-Backward
        elif np.any(left_mask):
            self.check_angle = [0, 0, 1, 0]  # Full-Left
        elif np.any(right_mask):
            self.check_angle = [0, 0, 0, 1]  # Full-Right
        else:
            self.check_angle[0] = np.sum(angles <= 90)
            self.check_angle[1] = np.sum(angles > 90)
            self.check_angle[2] = np.sum(angles < 0)
            self.check_angle[3] = np.sum(angles > 0)

    def timer_callback(self):
        msg = Int64MultiArray()
        sum_angle = sum(self.check_angle)
        normalized_check_angle = [int(x) for x in self.check_angle] if sum_angle > 0 else [0, 0, 0, 0]
        
        if self.check_stop:
            if 0 < self.stop_angle < 20 or self.stop_angle > 340:
                normalized_check_angle[0] = 0  # Stop Forward
            if 160 < self.stop_angle < 200:
                normalized_check_angle[1] = 0  # Stop Backward

        if self.future is None or self.future.done():
            self.send_request(True)

        msg.data = [int(self.sum_dist * 100)] + normalized_check_angle
        self.move.publish(msg)

    def send_request(self, data):
        try:
            self.req.data = data
            self.future = self.cli.call_async(self.req) 
            self.future.add_done_callback(self.response_callback)
        except Exception as e:
            self.get_logger().error(f"Service request failed: {str(e)}")

    def response_callback(self, future):
        try:
            response = future.result()
            self.check_stop = response.state
            self.stop_angle = response.angle
        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    lidar_reader = LidarReader()
    rclpy.spin(lidar_reader)
    lidar_reader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
