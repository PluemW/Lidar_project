import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64MultiArray
from geometry_msgs.msg import Twist

class MovementController(Node):
    def __init__(self):
        super().__init__('movement_controller')
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.subscription = self.create_subscription(Int64MultiArray, "/pub_move", self.move_callback, 10)
    
    def move_callback(self, msg):
        cmd_msg = Twist() 
        # [Dist, Forward, Backward, Left, Right]
        dist_speed = msg.data[0]
        ratio_speed = (30.0 - dist_speed) * 0.05
        check_angle = msg.data[1:]  # [Forward, Backward, Left, Right]
        
        if check_angle[0] != 0 or check_angle[1] != 0:
            if check_angle[0] > check_angle[1]:
                cmd_msg.linear.x = 0.5 * ratio_speed  # Move forward
            elif check_angle[1] > check_angle[0]:
                cmd_msg.linear.x = -0.5 * ratio_speed  # Move backward
        else:
            cmd_msg.linear.x = 0.0  # Stop moving forward/backward

        if check_angle[2] != 0 or check_angle[3] != 0:
            if check_angle[2] > check_angle[3]:
                cmd_msg.angular.z = -0.5 * ratio_speed  # Turn left
            elif check_angle[3] > check_angle[2]:
                cmd_msg.angular.z = 0.5 * ratio_speed  # Turn right
        else:
            cmd_msg.angular.z = 0.0  # Stop turning

        # self.get_logger().info(f"dis: {dist_speed} ratio: {ratio_speed} ang: {check_angle}")
        # self.get_logger().info(f"dis: {dist_speed} ratio: {ratio_speed:.2f} x: {cmd_msg.linear.x:.2f} z: {cmd_msg.angular.z:.2f}")
        self.cmd_vel_pub.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MovementController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
