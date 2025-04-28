import math

import rclpy
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from serial import Serial
from std_srvs.srv import Trigger


class PoseNode(Node):
	def __init__(self):
		# Initialize the ROS2 node
		super().__init__('pose_node')

		# Initialize the serial port
		self.serial = Serial(
			port='/dev/ttyAMA2',
			baudrate=38400,
			bytesize=8,
			parity='N',
			stopbits=1,
			timeout=1
		)

		# Subscribe to the Lidar topics
		self.laser_subscriber = self.create_subscription(
			LaserScan,
			'/scan',
			self.ladar_callback,
			10
		)

		# Subscribe to the IMU topic
		self.imu_angle_subscriber = self.create_subscription(
			Vector3,
			'/imu/angles',
			self.angles_callback,
			10
		)

		self.action_service = self.create_service(
			Trigger,
			'/action',
			self.action_callback
		)

		# Add class variables to store distances
		self.distance_0 = 0.0  # 0 degrees
		self.distance_90 = 0.0  # 90 degrees
		self.distance_180 = 0.0  # 180 degrees (left side)
		self.distance_270 = 0.0  # 270 degrees (back side)

		self.roll = 0.0
		self.pitch = 0.0
		self.yaw = 0.0

		self.send_pose_data = False

		# Publisher
		# Create a publisher
		self.timer = self.create_timer(0.1, self.timer_callback)  # Message Frequency: 10Hz

	def action_callback(self, request, response):
		"""
		Callback function for the action service
		:param request: Request object
		:param response: Response object
		:return:
		"""
		self.get_logger().info('Action service called')
		action_msg = f'!Action?'
		self.serial.write(action_msg.encode('utf-8'))  # Send action message to STM32

		# 设置标志位，开始发送pose数据
		self.send_pose_data = True
		self.get_logger().info('开始以10Hz频率发送pose数据')

		response.success = True
		response.message = 'Action executed successfully'
		return response

	def timer_callback(self):
		"""
		Timer callback to send position data at 10Hz
		"""
		# Use the send_pose_data flag to determine if data should be sent
		if self.send_pose_data:
			# Use distance_180 as Y coordinate and distance_270 as X coordinate
			y_coord = self.distance_180 if hasattr(self, 'distance_180') else 0.0
			x_coord = self.distance_270 if hasattr(self, 'distance_270') else 0.0

			# Build the formatted message
			formatted_data = self.pose_msg_builder(
				self.pitch, self.roll, self.yaw, x_coord, y_coord)

			# Send the data
			self.pose_sender(formatted_data)

	def angles_callback(self, msg):
		"""
		Callback function for IMU angles
		:param msg: IMU angles message
		:return:
		"""
		# Extract angles from the message
		self.roll = msg.x
		self.pitch = msg.y
		self.yaw = msg.z

	# # Most important: Yaw Axis
	# angles_data = f'Angles: {self.roll:.2f},{self.pitch:.2f},{self.yaw:.2f}\n'
	# self.serial.write(angles_data.encode('utf-8'))  # Send angles to STM32
	# self.get_logger().info(angles_data)

	def ladar_callback(self, msg):
		"""
		Callback function for Lidar messages
		:param msg: Lidar message
		:return:
		"""
		# Calculate how many points are in the scan
		num_points = len(msg.ranges)

		# Define angles to check (in degrees)
		angles = {
			0: 'distance_0',
			90: 'distance_90',
			180: 'distance_180',
			270: 'distance_270'
		}

		# Calculate the index and obtain the safe distance for each angle
		for angle_deg, attr_name in angles.items():
			# Convert degrees to radians
			angle_rad = math.radians(angle_deg)

			# Calculate the index in the ranges array
			idx = int((angle_rad - msg.angle_min) / msg.angle_increment)

			# Make sure index is within bounds
			if idx >= 0 and idx < num_points:
				distance = msg.ranges[idx]

				# Store current value or keep previous if invalid
				if not math.isnan(distance) and distance != float('inf'):
					setattr(self, attr_name, distance)
				# If no previous valid value exists, set to 0
				elif not hasattr(self, attr_name) or getattr(self, attr_name) in (float('inf'), float('nan')):
					setattr(self, attr_name, 0.0)

	# Otherwise keep previous value (already in the attribute)

	# self.get_logger().info(
	# 	f'Distances - 0°: {self.distance_0:.2f}, 90°: {self.distance_90:.2f}, 180°: {self.distance_180:.2f}, 270°: {self.distance_270:.2f}')

	def pose_msg_builder(self, pitch, roll, yaw, x_coord, y_coord):
		"""
		Build the message to be sent
		:param pitch: Pitch angle
		:param roll: Roll angle
		:param yaw: Yaw angle
		:param x_coord: x_coord
		:param y_coord: y_coord
		:return: Formatted message string
		"""
		# Initialize last valid coordinates if not already set
		if not hasattr(self, 'last_valid_x'):
			self.last_valid_x = 0
		if not hasattr(self, 'last_valid_y'):
			self.last_valid_y = 0

		# First convert to centimeters for raw coordinates
		x_cm = x_coord * 100
		y_cm = y_coord * 100

		# Safety check for coordinates
		if math.isnan(x_coord) or x_coord == float('inf') or x_coord == 0.0:
			self.get_logger().warn(f'Invalid X value detected: {x_coord}, using previous value')
			x_cm = self.last_valid_x
		else:
			# Store the valid raw value
			self.last_valid_x = x_cm

		if math.isnan(y_coord) or y_coord == float('inf') or y_coord == 0.0:
			self.get_logger().warn(f'Invalid Y value detected: {y_coord}, using previous value')
			y_cm = self.last_valid_y
		else:
			# Store the valid raw value
			self.last_valid_y = y_cm

		# Limit to 4 digits
		x_cm = min(int(x_cm), 9999)
		y_cm = min(int(y_cm), 9999)

		# Safety check for angles
		if math.isnan(pitch):
			pitch = 0.0
		if math.isnan(roll):
			roll = 0.0
		if math.isnan(yaw):
			yaw = 0.0

		# Format angles as: sign + 3 digit integer + decimal point + 2 decimals
		pitch_str = f"{'+' if pitch >= 0 else '-'}{int(abs(pitch)):03d}.{int(abs(pitch * 100) % 100):02d}"
		roll_str = f"{'+' if roll >= 0 else '-'}{int(abs(roll)):03d}.{int(abs(roll * 100) % 100):02d}"
		yaw_str = f"{'+' if yaw >= 0 else '-'}{int(abs(yaw)):03d}.{int(abs(yaw * 100) % 100):02d}"

		# Construct the formatted message string
		data = f"@:{pitch_str},{roll_str},{yaw_str},{x_cm:04d},{y_cm:04d}#"

		return data

	def pose_sender(self, data):
		"""
		Send data to the serial port
		:param data: Data to be sent
		"""
		# Send data to the serial port
		self.serial.write(data.encode('utf-8'))
		# Log the sent data
		self.get_logger().info(f'Sent data: {data.strip()}')



def main():
	rclpy.init()
	node = PoseNode()
	rclpy.spin(node)
	node.serial.close()
	node.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
