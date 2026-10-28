import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
import math

class GoToGoalNode(Node):
    def __init__(self):
        super().__init__('go_to_goal_node')
        
        # Publisher changed to TwistStamped to match Jazzy Gazebo Bridge
        self.publisher_ = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        
        # Subscriber with correct QoS profile
        self.subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile_sensor_data)
        
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.goal_x = 2.0
        self.goal_y = 2.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        self.goal_reached = False
        self.get_logger().info(f"Target set to X: {self.goal_x}, Y: {self.goal_y}")

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        self.current_yaw = self.euler_from_quaternion(q.x, q.y, q.z, q.w)

    def euler_from_quaternion(self, x, y, z, w):
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        return math.atan2(t3, t4)

    def control_loop(self):
        if self.goal_reached:
            return

        distance_error = math.sqrt((self.goal_x - self.current_x)**2 + (self.goal_y - self.current_y)**2)
        angle_to_goal = math.atan2(self.goal_y - self.current_y, self.goal_x - self.current_x)
        angle_error = angle_to_goal - self.current_yaw
        
        while angle_error > math.pi: angle_error -= 2.0 * math.pi
        while angle_error < -math.pi: angle_error += 2.0 * math.pi

        # Create a TwistStamped message and attach the current ROS time
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        if distance_error > 0.1:
            msg.twist.angular.z = 1.5 * angle_error 
            
            if abs(angle_error) < 0.2:
                msg.twist.linear.x = 0.3 * distance_error 
            else:
                msg.twist.linear.x = 0.0
        else:
            msg.twist.linear.x = 0.0
            msg.twist.angular.z = 0.0
            self.get_logger().info("Target reached! Velocity set to 0.")
            self.goal_reached = True

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GoToGoalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()