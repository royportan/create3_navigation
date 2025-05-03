

import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from irobot_create_msgs.msg import IrIntensityVector
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import tf2_ros, tf2_geometry_msgs
from geometry_msgs.msg import Point
import numpy as np
from geometry_msgs.msg import Twist
import time
# Input your namespace here
namespace = ''

class IRSubscriber(Node):
	def __init__(self):
		super().__init__('IR_subscriber')
		print('Creating subscription to the IrIntensityVector type over the /ir_intensity topic')
		self.subscription = self.create_subscription(
			IrIntensityVector, namespace + '/ir_intensity', self.listener_callback,
			qos_profile_sensor_data)
		self.IR = np.zeros((7, 1))

	def getIR(self, msg):
		IR = np.zeros((7, 1))
		count = 0
		for reading in msg.readings:
			IR[count] = reading.value
			count += 1
		#print(IR)
		return IR
	def getDistance(self,IR):
		D = np.zeros((7,1))
		D = 2.7306*(IR**(-0.567))
		print(D)
		return D
	def listener_callback(self, msg: IrIntensityVector):
		print('Now listening to IR sensor readings it hears...')
		self.IR = self.getIR(msg)
		self.D = self.getDistance(self.IR)
		
		# self.printIR(msg)

	def printIR(self, msg):
		print('Printing IR sensor readings:')
		for reading in msg.readings: 
			val = reading.value
			print("IR Sensor:", val)


def main(args=None):
	rclpy.init(args=args)
	IR_subscriber = IRSubscriber()
	print('Callbacks are called.')
	try:
		rclpy.spin(IR_subscriber)
	except KeyboardInterrupt:
		print('\nCaught keyboard interrupt')
	finally:
		print("Done")
		IR_subscriber.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main()

