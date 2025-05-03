import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from rclpy.qos import qos_profile_sensor_data

class OdomSubscriber(Node):
    def __init__(self):
        super().__init__('odom_listener')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback,
            qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # Position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Orientation (convert quaternion to Euler angles)
        q = msg.pose.pose.orientation
        q_vec = [q.x, q.y, q.z, q.w]
       	raw, pitch, yawl = euler_from_quaternion(q_vec)

        # Velocity
        linear_vel = msg.twist.twist.linear.x
        angular_vel = msg.twist.twist.angular.z
 	
        self.get_logger().info(f"Position: x={x:.2f}, y={y:.2f}, yawl={yawl:.2f}")
        self.get_logger().info(f"Velocity: linear={linear_vel:.2f}, angular={angular_vel:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = OdomSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

