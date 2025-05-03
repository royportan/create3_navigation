import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class Create3Mover(Node):
    def __init__(self,name):
    	# name the Node
        super().__init__(name)

        # Publisher for linear velocity (forward/backward)
        self.linear_velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Publisher for angular velocity (rotation)
        self.angular_velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        time.sleep(2)

    def move_forward(self, duration, speed=0.2):
        twist = Twist()
        twist.linear.x = speed  # Move forward by setting linear.x
        rate = 10  # Publish at 10 Hz
        end_time = time.time() + duration
        while time.time() < end_time:
            self.linear_velocity_publisher.publish(twist)
            time.sleep(1.0 / rate)
        self.stop_robot()

    def turn(self, duration, angular_speed=0.5):
        twist = Twist()
        twist.angular.z = angular_speed  # Rotate by setting angular.z
        rate = 10  # Publish at 10 Hz
        end_time = time.time() + duration
        while time.time() < end_time:
            self.angular_velocity_publisher.publish(twist)
            time.sleep(1.0 / rate)
        self.stop_robot()

    def stop_robot(self):
        # Stop both forward motion and rotation by publishing zeros to both topics
        twist = Twist()
        self.linear_velocity_publisher.publish(twist)
        self.angular_velocity_publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    mover = Create3Mover("Motion")

    try:
        mover.move_forward(3)  # Move forward for 3 seconds
        time.sleep(1)  # Pause before turning
        mover.turn(2)  # Turn for 2 seconds
    except KeyboardInterrupt:
        mover.stop_robot()
        mover.get_logger().info('Motion interrupted by user.')
    finally:
        mover.stop_robot()
        mover.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
