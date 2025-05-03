import rclpy
from rclpy.node import Node
from irobot_create_msgs.msg import IrIntensityVector
import tf2_ros, tf2_geometry_msgs
from geometry_msgs.msg import Point
from rclpy.qos import qos_profile_sensor_data
class IRMapper(Node):
	def __init__(self):
		super().__init__('ir_mapper')
		self.tf_buffer	 = tf2_ros.Buffer()
		self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
		self.ir_sub		 = self.create_subscription(IrIntensityVector, '/ir_intensity', self.ir_callback, qos_profile_sensor_data)

	def ir_callback(self, msg):
		# 1) Read the distance
		#dist = msg.range
		
		# 2) Construct a point in the IR sensor’s frame
		#pt = Point(x=dist, y=0.0, z=0.0)
		
		# 3) Lookup transform from sensor frame → base_link or odom
		try:
			for reading in msg.readings:
				#print(reading.value)
				xf = self.tf_buffer.lookup_transform(
					'base_link',	  # target frame
					reading.header.frame_id,  # source frame (e.g. “ir_intensity_left”)
					rclpy.time.Time())
				print(xf)
		except Exception as e:
			self.get_logger().warn(f"TF lookup failed: {e}")
			return
		xb =  self.tf_buffer.lookup_transform(
					'odom',	  # target frame
					'base_link',  #original frame
					rclpy.time.Time())
		print(xb)
		# 4) Transform the point
		#pt_base = tf2_geometry_msgs.do_transform_point(pt, xf)
		
		# 5) Now pt_base.point.x/y/z is the obstacle’s location in base_link
		#self.get_logger().info(f"Obstacle at x={pt_base.point.x:.2f}, y={pt_base.point.y:.2f}")

def main(args=None):
	rclpy.init(args=args)
	node = IRMapper()
	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == '__main__':
	main()

