
from builtin_interfaces.msg import Time
from rclpy.duration import Duration
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from irobot_create_msgs.msg import IrIntensityVector
import tf_transformations
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import tf2_ros, tf2_geometry_msgs
from geometry_msgs.msg import Point
import numpy as np
from geometry_msgs.msg import Twist
import time
import matplotlib.pyplot as plt

# Input your namespace here
namespace = ''

class Follow_Wall(Node):
	def __init__(self):
		super().__init__('Follow_Wall')
		self.state = "wandering"
		self.prev_side = "S";
		self.prev_state = self.state
		plt.ion()  # Interactive mode ON
		self.fig, self.ax = plt.subplots()
		self.ax.set_xlim(-1000, 1000)
		self.ax.set_ylim(-1000, 1000)
		self.sc = self.ax.scatter([], [])
		self.x_data = []
		self.y_data = []
		self.T = np.identity(3)
		self.linear_velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
		#C3twist = Twist()
		#twist.linear.x  = 0.05 
		self.angular_velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
		self.ODOM = self.create_subscription(Odometry,'/odom',self.updateOdom,qos_profile_sensor_data) 
		print('Creating subscription to the IrIntensityVector type over the /ir_intensity topic')
		self.sensors_name  = [
			'ir_intensity_side_left',
			'ir_intensity_left',
			'ir_intensity_front_left',
		   	'ir_intensity_front_center_left',
		   	'ir_intensity_front_center_right',
		   	'ir_intensity_front_right',
		   	'ir_intensity_right',
		]
		self.order = np.array([0,1,2,3,4,5,6])
		self.sensors_order = np.array([0,1,2,3,4,5,6])
		self.order_ind = np.array([0,1,2,3,4,5,6])
		self.IR = np.zeros((7, 1))
		self.tf_buffer	 = tf2_ros.Buffer()
		self.T0 = np.identity(3)
		self.tf_listener = tf2_ros.TransformListener(self.tf_buffer,self,spin_thread=True)
		while not self.tf_buffer.can_transform('odom', 'base_link', rclpy.time.Time()):
			self.get_logger().info("Waiting for odom -> base_link transform...")

			time.sleep(0.01)
		self.IRreading = self.create_subscription(
			IrIntensityVector, namespace + '/ir_intensity', self.listener_callback,
			qos_profile_sensor_data)	
		q0 =  self.tf_buffer.lookup_transform(
					'odom',	  # target frame
					'base_link',  #original frame
					rclpy.time.Time())
		self.get_logger().info("Waiting 3 second for TF tree to populate...")
		# time.sleep(3)
		# latest_time = Time()
		# timeout = Duration(seconds=1.0)
		# count = 0
		# for ir_name in self.sensors_name:
		# 	while not self.tf_buffer.can_transform('base_link', ir_name, latest_time, timeout):
		# 		self.get_logger().info("Waiting for ir -> base_link transform...")
		# 		time.sleep(0.01)
		# 		print(self.tf_buffer.can_transform('base_link','ir_intensity_right',latest_time, timeout))	
		# 	irT = self.tf_buffer.lookup_transform(
		# 				'base_link',	  # target frame
		# 				ir_name,  #original frameads
		# 				rclpy.time.Time())
		# 	self.ir[count,:,:] = self.getT(irT)
		# 	count = count + 1

		# Initialize the array to store the 2D transformation matrices
		self.ir = np.zeros((7, 3, 3))

		# Assign each 3x3 transformation matrix to self.ir[i,:,:]
		# side left
		self.ir[0,:,:] = np.array([
			[0.540, -0.841,  0.095],
			[0.841,  0.540,  0.147],
			[0.000,  0.000,  1.000]
		])
 		# left
		self.ir[1,:,:] = np.array([
			[0.809, -0.588,  0.141],
			[0.588,  0.809,  0.103],
			[0.000,  0.000,  1.000]
		])
		# front_left
		self.ir[2,:,:] = np.array([
			[0.945, -0.327,  0.165],
			[0.327,  0.945,  0.057],
			[0.000,  0.000,  1.000]
		])
		#center_front_left
		self.ir[3,:,:] = np.array([
			[0.999, -0.046,  0.175],
			[0.046,  0.999,  0.008],
			[0.000,  0.000,  1.000]
		])
		#cent_right
		self.ir[4,:,:] = np.array([
			[0.971,  0.238,  0.170],
			[-0.238,  0.971, -0.042],
			[0.000,  0.000,  1.000]
		])
		#front_right
		self.ir[5,:,:] = np.array([
			[0.845,  0.535,  0.148],
			[-0.535,  0.845, -0.094],
			[0.000,  0.000,  1.000]
		])
		#right
		self.ir[6,:,:] = np.array([
			[0.461,  0.888,  0.081],
			[-0.888,  0.461, -0.155],
			[0.000,  0.000,  1.000]
		])

		
		#print(self.T @ self.ir)		
		self.T0 = self.getT(q0)
		self.invT0 = self.InvT(self.T0)
		self.tf_listener = tf2_ros.TransformListener(self.tf_buffer,self)
		0
	def InvT(self,R):
		invR = np.identity(3)
		rot = R[:2,:2]
		t = R[:2,2]
		rotT = rot.T
		tT = -rotT@ t
		invR[:2,:2] = rotT
		invR[:2,2] = tT
		return invR
				
	def getT(self,qi):
		#print("hello")
		q = qi.transform.rotation
		quat = [q.x, q.y, q.z, q.w]
		rot_matrix = tf_transformations.quaternion_matrix(quat)
		rot2d = rot_matrix[:2, :2] 
		# handle translation
		t = qi.transform.translation
		translate2d = np.array([t.x, t.y]) 
		T0 = np.identity(3)
		T0[:2,:2] = rot2d
		T0[:2,2] = translate2d
		#print(T0)
		return T0
			
	def getIR(self, msg):
		IR = np.zeros((7, 1))
		count = 0
		for reading in msg.readings:
			IR[count] = reading.value
			count += 1
		IR[2] -= 11
		IR[3] -= 11
		IR[4] -= 11
		IR = np.clip(IR,0,4000)
		print(IR)
		return IR
	def getDistance(self,IR):
		D = np.zeros((7,1))
		#D = 2.7306*(IR**(-0.567))
		D = 0.6862*(IR**(-0.33))
		# D2 = np.zeros((7,1))
		# D2[0] = D[3]
		# D2[1] = D[4]
		# D2[2] = D[2]
		# D2[3] = D[5]
		# D2[4] = D[1]
		# D2[5] = D[0]
		# D2[6] = D[6] 
		#print(D2)
		return D
	def listener_callback(self, msg: IrIntensityVector):
		#print('Now listening to IR sensor readings it hears...')
		self.IR = self.getIR(msg)
		self.D = self.getDistance(self.IR)
		self.order_ind = np.argsort(self.D)
		#self.sensors_order = self.order[order_ind]
		#print("T0")
		#print(self.T0)
		#self.printIR(msg)
		self.processIR()
		#print(self.ir[1])
		#time.sleep(1)
	def setStates(self):
		if min(self.D) <= 0.07:
			self.state = 'OA'
			return
		if min(self.D) > 0.29:
			self.state = 'Random'
			return
		self.state = "FW"
		return
	def states(self):
		if self.state == 'FW':
			self.processIR()
		else:
			if self.state == 'OA':
				self.avoid_obs()
			else: # wander randomly
				self.wander()
	def printIR(self, msg):
		print('Printing IR sensor readings:')
		for reading in msg.readings: 
			val = reading.value
			print("IR Sensor:", val)
	def processIR(self):
		print(self.state)
		print(self.prev_side)
		i = 0
		count = 0
		u_fw = np.array([0,0])
		p = np.zeros((2,3))
		chosed = np.zeros((2,1))
		print("Self D")
		d1 = self.D[self.order_ind[0]]
		d2 = self.D[self.order_ind[1]]
		T1 = self.ir[self.order_ind[0],:,:]
		T2 = self.ir[self.order_ind[1],:,:] 
		if d1 > 0.07 and d1 < 0.285:
			ir_vec = np.ones((3))
			ir_vec[0] = d1
			ir_vec[1] = 0
			chosed[count] = d1;
			print(type(self.order_ind))
			print(self.order_ind.shape)
			print(self.order_ind)
			print(self.sensors_name[self.order_ind[0]]);
			print("chose d :" + str(d1))
			ir_vec = self.T @ T1 @ ir_vec
			#print(self.ir[i])
			coords = self.toCoords(ir_vec[0],ir_vec[1])
			self.update_plot(coords[0],coords[1])
			p[count,:2] = ir_vec[:2]
			p[count,2] = self.order_ind[0]
			count += 1
		if d2 > 0.07 and d2 < 0.285:
			ir_vec = np.ones((3))
			ir_vec[0] = d2
			ir_vec[1] = 0
			chosed[count] = d2;
			print(self.sensors_name[self.order_ind[1]]);
			print("chose d :" + str(d2))
			ir_vec = self.T @ T2 @ ir_vec
			#print(self.ir[i])
			coords = self.toCoords(ir_vec[0],ir_vec[1])
			self.update_plot(coords[0],coords[1])
			p[count,:2] = ir_vec[:2]
			p[count,2] = self.order_ind[1]
			count +=1
		#print(self.D)
		'''
		for d in self.D:
			if count == 2 :
				break
			if d >= 0.03 and d <= 0.285:
				ir_vec = np.ones((3))
				ir_vec[0] = d
				ir_vec[1] = 0
				chosed[count] = d;
				print("chose i:" + str(i))
				print("chose d :" + str(d))
				ir_vec = self.T @ self.ir[i,:,:] @ ir_vec
				#print(self.ir[i])
				coords = self.toCoords(ir_vec[0],ir_vec[1])
				self.update_plot(coords[0],coords[1])
				p[count,:2] = ir_vec[:2]
				p[count,2] = self.sensors_order[i]
				
				print(self.sensors_name[i])
				count = count+ 1
			else:
				#print(self.sensors_name[i] + " is not detecting wall")	
				pass
			#print("i:" +str(i))
			i = i + 1
		'''
			
		print("count" + str(count)) 
		if count == 1: 
			#if p[0,0]**2 + p[0,1]**2 <= 0.15**2:
			#	self.move(0,-0.25)
			#else:
			order = p[0,2]
			print("order" + str(order))
			print("d1:" + str(chosed[0]))
			if chosed[0] >= 0.07 and chosed[0] <= 0.29:

				if order == 0:
					#self.move(0.01,-0.15)
					self.prev_side = "S"
					self.state = "wandering"
				if order == 1:
					#self.move(0.01,0.15)
					self.prev_side = "S"
					self.state = "wandering"
				if order == 2:
					#self.move(0.01,-0.15)
					self.prev_side = "S"
					self.state = "wandering"
				if order == 3:
					#self.move(0.01,0.15)
					self.prev_side = "S"
					self.state = "wandering"
				if order == 4:
					#self.move(0.5,0.075)
					self.prev_side = "L"
					self.state = "FW"
				if order == 5:
					#self.move(0.5,0.075)
					self.prev_side = "L"
					self.state = "FW"
				if order == 6:
					print(" in order 6")
					#self.move(0.5,-0.075)
					self.prev_side = "R"
					self.state = "FW"	
			else:
				self.prev_side = "S"
				twist = Twist()
				#twist.linear.x = 0.3
				self.linear_velocity_publisher.publish(twist)
		if count < 1:

			if self.state == "FW":
				if self.prev_side == "L":
					twist = Twist()
					#twist.angular.z = 0.1
					#twist.linear.x = 0.05
					self.linear_velocity_publisher.publish(twist)
				else:
					if self.prev_side == "R":
						twist = Twist()
						#twist.linear.x = 0.05
						#twist.angular.z = -0.25
						self.linear_velocity_publisher.publish(twist)
					else: 
						twist = Twist()
						#twist.linear.x = 0.25
						self.linear_velocity_publisher.publish(twist)
						self.state = "wandering"
						
			else:			
				twist = Twist()
				#twist.linear.x = 0.5
				self.linear_velocity_publisher.publish(twist)
				self.state = "wandering"
		if count == 2:
			print("Detect Wall")
			self.state = "FW"
			print(p[0,2])
			print(p[1,2])
			head_vec = self.T[:2,0];
			u_fw = p[1,:2] - p[0,:2];
			dot = np.dot(head_vec,u_fw)
			if dot < 0:
				u_fw = -u_fw
			u_a = p[0,:2]
			'''
			if p[0,2] < p[1,2]:
				u_fw = p[0,:2] - p[1,:2]
				u_a = p[1,:2]	
			else:	
				u_fw = p[1,:2] - p[0,:2]
				u_a = p[0,:2]
			'''
			u_p = np.array([self.x,self.y])
			uptoa = u_a - u_p
			uf_norm = np.sqrt(u_fw[1]**2 +u_fw[0]**2)
			u_fw = u_fw/uf_norm 
			u_fwp = uptoa - (uptoa.T @ u_fw)*u_fw
			u_fwp_norm = np.sqrt(u_fwp[0]**2 + u_fwp[1]**2)
			dkeep = 0.1
			u_fwpK = u_fwp -  (dkeep/u_fwp_norm)*u_fwp
			u_fw = u_fw + u_fwpK
			#print(u_fw)
			ang_curr = self.heading
			#print(u_fw)
			ang_targ = np.arctan2(u_fw[1],u_fw[0])
			angleDiff = self.round_angle(ang_targ-ang_curr)
			self.turnCompare(angleDiff)
			#print(angleDiff)
		elif count == 1:
			pass
		else:	
			pass
			#wanderrandom
		#Ltwist = Twist()
		#Ltwist.linear.x  = 0.2
		#self.linear_velocity_publisher.publish(Ltwist)
	def move_forward(self, duration, speed=0.2):
		twist = Twist()
		twist.linear.x = speed  # Move forward by setting linear.x
		rate = 10  # Publish at 10 Hz
		end_time = time.time() + duration
		while time.time() < end_time:
		    self.linear_velocity_publisher.publish(twist)
		    time.sleep(1.0 / rate)
		self.stop_robot() 
	def turnAngle(self,angle, angular_speed=0.1):
		curr_angle = self.round_angle(self.heading)
		target_angle =self.round_angle(self.heading + angle )
		if angle < 0:
			direction = - 1
		else:
			direction =  1
		
		twist = Twist()
		twist.angular.z = direction * angular_speed  # Rotate by setting angular.z
		Prate = 10  # Publish at 10 Hz
		#end_time = time.time() + duration
		print("target")
		print(target_angle)
		print("heading")
		print(self.heading)
		while abs(self.heading - angle) >  0.01:
					
			self.angular_velocity_publisher.publish(twist)
			#time.sleep(1.0 / rate)
			time.sleep(1/Prate)
			print("target")
			print(target_angle)
			print("heading")
			print(self.heading)
		self.stop_robot()
	def turnCompare(self,angleDiff, angular_speed=0.8):
		#curr_angle = self.round_angle(self.heading)
		#target_angle =self.round_angle(self.heading + angle )
		if angleDiff < 0:
			direction = - 1
		else:
			direction =  1
		
		twist = Twist()
		twist.angular.z = direction * angular_speed  # Rotate by setting angular.z
		Prate = 10  # Publish at 10 Hz
		#end_time = time.time() + duration
		
		#print("target")
		#print(target_angle)
		#print("heading")
		#print(self.heading)
		if abs(angleDiff) >  0.1:
			twist.linear.x = 0.1	
			self.angular_velocity_publisher.publish(twist)
			#time.sleep(1.0 / rate)
			time.sleep(1/Prate)
			#print("target")
			#print(target_angle)
		#	print("heading")
		#	print(self.heading)
		else:	
			twist.linear.x = 0.25
			twist.angular.z = 0.0
			self.angular_velocity_publisher.publish(twist)
			
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
	def updateOdom(self,msg):
		#print("HELLO")
		self.x = msg.pose.pose.position.x
		self.y = msg.pose.pose.position.y
		# Orientation (convert quaternion to Euler angles)
		q = msg.pose.pose.orientation
		q_vec = [q.x, q.y, q.z, q.w]
		raw, pitch, yawl = euler_from_quaternion(q_vec)
		print("yawl")
		print(yawl)
		self.heading = self.round_angle(yawl)
		# Velocity
		linear_vel = msg.twist.twist.linear.x
		angular_vel = msg.twist.twist.angular.z
		self.T[0,2] = self.x
		self.T[1,2] = self.y
		rot_matrix = tf_transformations.quaternion_matrix(q_vec)
		rot2d = rot_matrix[:2,:2]
		self.T[:2,:2] = rot2d
		#print("heading")
		#
	def round_angle(self,angle):
		return (angle + np.pi) % (2 * np.pi) - np.pi
	#def check_sensor(order):
	#	if 
	def toCoords(self,x,y):
		x = round(100*x)
		y = round(100*y)
		return np.array([x,y])
	def update_plot(self,x, y):
		self.x_data.append(x)
		self.y_data.append(y)
		#print(self.x_data)
    		#print(self.y_data)
		self.sc.set_offsets(np.c_[self.x_data, self.y_data])  
		plt.draw()                              
		plt.pause(0.001)  
	def move(self,ang_sp,lin_sp):
		twist = Twist()
		twist.angular.z = ang_sp
		twist.linear.x = lin_sp
		self.linear_velocity_publisher.publish(twist)
		                        
def main(args=None):
	# Setup
	rclpy.init(args=args)
	FW = Follow_Wall()
	print('Callbacks are called.')
	try:	
		#FW.move_forward(4)
		#FW.turnAngle(np.pi/2)
		#FW.turnAngle(-np.pi/2)
		rclpy.spin(FW)
	except KeyboardInterrupt:
		print('\nCaught keyboard interrupt')
	finally:
		print("Done")
		FW.destroy_node()
		rclpy.shutdown()
		plt.ioff()


if __name__ == '__main__':
	main()

