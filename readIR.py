'''
sub_ir.py
Tufts Create®3 Educational Robot Example
by Kate Wujciak 

This file shows how to subscribe to a topic in ROS2 using the Create®3. It subscribes
to the IR sensor and displays the relevant information in your terminal. 
'''

import sys
import rclpy

'''
Statements that import messages, actions, interfaces, and variable types.
'''
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from irobot_create_msgs.msg import IrIntensityVector
import numpy as np
'''
Input your namespace here as a global variable. 
'''
namespace = ''
np.set_printoptions(suppress=True)
class IRSubscriber(Node):
	def __init__(self):
		super().__init__('IR_subscriber')
		print('Creating subscription to to the IrIntensityVector type over the /ir_intensity topic')
		self.subscription = self.create_subscription(
			IrIntensityVector, namespace + '/ir_intensity', self.listener_callback,
			qos_profile_sensor_data)
		self.IR = np.zeros((7,1))
		self.D = np.zeros((7,1))
	def getIR(self, msg):
		IR = np.zeros((7,1))
		count = 0
		for reading in msg.readings:
			IR[count] = reading.value
			count = count + 1
		#print(IR)
		return IR
	def getDistance(self,IR):
		D = np.zeros((7,1))
		D = 2.7306*(IR**(-0.567))
		print(D)
		return D
	def listener_callback(self, msg:IrIntensityVector):
		'''
		The subscriber's callback listens and as soon as it receives the message,
		this function runs. 
		This callback function is basically printing what it hears. It runs the data
		it receives in your terminal (msg).	 
		'''
		print('Now listening to IR sensor readings it hears...')
		self.IR = self.getIR(msg)
		self.D = self.getDistance(self.IR)
		#self.printIR(msg)
		
	def printIR(self, msg):
		'''
		This function is used in the above function. Its purpose is to determine 
		which parts of the info are worth showing.
		:type msg: IrIntensity
		:rtype: None
		The msg is returned from our topic '/ir_intensity.'
		To get components of a message, use the '.' dot operator. 
		'''
		print('Printing IR sensor readings:')
		for reading in msg.readings: 
			val = reading.value
			print("IR Sensor:", val)

	
def main(args=None):
	'''
	This line initializes the rclpy library. 
	'''
	rclpy.init(args=args)
	'''
	This creates the node.
	'''
	IR_subscriber = IRSubscriber()
	'''
	The node is then "spun" so its callbacks are called.
	'''
	print('Callbacks are called.')
	try:
		rclpy.spin(IR_subscriber)
	except KeyboardInterrupt:
		print('\nCaught keyboard interrupt')
	finally:
		'''
		Destroying the node acts as a "reset" so we don't run into 
		any problems when we try and run again.
		'''
		print("Done")
		IR_subscriber.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main()
