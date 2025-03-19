import rclpy
from rclpy.node import Node

from autoware_auto_perception_msgs.msg import PredictedObjects
from autoware_auto_perception_msgs.msg import PredictedObject

from std_msgs.msg import Header
from autoware_auto_perception_msgs.msg import DetectedObjects
from autoware_auto_perception_msgs.msg import DetectedObject
from autoware_auto_perception_msgs.msg import ObjectClassification

from std_msgs.msg import Float32

import time

class PredictedObjectsSubscriber(Node):
	def __init__(self):
		super().__init__('predicted_objects_subscriber')

		self.predicted_objects_subscription = self.create_subscription(PredictedObjects, "/perception/object_recognition/prediction/path/carom/objects", \
			self.process_predicted_object, 10)
		self.predicted_objects_subscription

		self.predicted_infrastructure_objects_publisher = self.create_publisher(DetectedObjects, \
                '/perception/object_recognition/prediction/point/carom/objects', 10)
		
		self.carom_process_time_subscriber = self.create_subscription(Float32, '/perception/object_recognition/detection/carom/objects/delay', \
												self.carom_process_time_updater, 10)
		self.carom_process_time_subscriber
		self.carom_process_time = 100
		
		self.publish_objects_msg = DetectedObjects()
		
	def carom_process_time_updater(self, process_time_msg):
		self.carom_process_time = process_time_msg.data
		#print("carom process time: ", self.carom_process_time)

	def path_point_extraction(self, prediction_msg, path_point_no):
		objects = [0 for o in range(len(prediction_msg.objects))]
		classification = [0,0,0]
		for i in range(len(prediction_msg.objects)):
			# path[9] - no. 9 here means prediction at 100th ms. The window size in the prediction algorithm set to 10ms
			predicted_px = prediction_msg.objects[i].kinematics.predicted_paths[0].path[path_point_no].position.x
			predicted_py = prediction_msg.objects[i].kinematics.predicted_paths[0].path[path_point_no].position.y
			predicted_pz = prediction_msg.objects[i].kinematics.predicted_paths[0].path[path_point_no].position.z
			predicted_qx = prediction_msg.objects[i].kinematics.predicted_paths[0].path[path_point_no].orientation.x
			predicted_qy = prediction_msg.objects[i].kinematics.predicted_paths[0].path[path_point_no].orientation.y
			predicted_qz = prediction_msg.objects[i].kinematics.predicted_paths[0].path[path_point_no].orientation.z
			predicted_qw = prediction_msg.objects[i].kinematics.predicted_paths[0].path[path_point_no].orientation.w
			object_classification = prediction_msg.objects[i].classification[0].label
			object_classification_probability = prediction_msg.objects[i].classification[0].probability
			dim_x = prediction_msg.objects[i].shape.dimensions.x
			dim_y = prediction_msg.objects[i].shape.dimensions.y
			dim_z = prediction_msg.objects[i].shape.dimensions.z

			objects[i] = DetectedObject()
			classification[0] = ObjectClassification()
			classification[1] = ObjectClassification()
			classification[2] = ObjectClassification()
			objects[i].existence_probability = 1.0
			classification[0].label = object_classification
			classification[0].probability = object_classification_probability
			classification[1].label = 1
			classification[1].probability = 0.0
			classification[2].label = 1
			classification[2].probability = 0.0
			objects[i].classification = prediction_msg.objects[i].classification
			objects[i].kinematics.pose_with_covariance.pose.position.x = predicted_px
			objects[i].kinematics.pose_with_covariance.pose.position.y = predicted_py
			objects[i].kinematics.pose_with_covariance.pose.position.z = predicted_pz
			objects[i].kinematics.pose_with_covariance.pose.orientation.x = predicted_qx
			objects[i].kinematics.pose_with_covariance.pose.orientation.y = predicted_qy
			objects[i].kinematics.pose_with_covariance.pose.orientation.z = predicted_qz
			objects[i].kinematics.pose_with_covariance.pose.orientation.w = predicted_qw
			objects[i].kinematics.orientation_availability = 1
			objects[i].shape.dimensions.x = dim_x
			objects[i].shape.dimensions.y = dim_y
			objects[i].shape.dimensions.z = dim_z

		self.publish_objects_msg.objects = objects
		#self.publish_objects_msg.header = prediction_msg.header
	
	def process_predicted_object(self, prediction_msg):
		#self.get_logger().info('prediction_msg {}'.format(prediction_msg.objects[0].kinematics.predicted_paths[0].path[9]))
		self.publish_objects_msg = DetectedObjects()

		# self.carom_process_time is in ms. ms/10.0 provides a number (E.g. 110.0/10.0 = 11.0)
		# As array starts from zero 11.0 should be subtracted with 1 to get the point at 110th ms.
		path_point_no = round(self.carom_process_time/10.0)-1   
		path_point_no = path_point_no + 1 # This addition of one is made to compensate and simulate the delay caused by transmission.
		#print("FIRST path point no.: ", path_point_no)

		# This if condition helps prevent out of index range error. Because for now the path array consists of max of 20 paths.
		if path_point_no>20:
			path_point_no = 20
		self.path_point_extraction(prediction_msg, path_point_no=path_point_no)
		
		#seconds = int((prediction_msg.header.stamp.nanosec + int((path_point_no+1)*1e8))*1e-9)
		#self.publish_objects_msg.header.stamp.sec = int(prediction_msg.header.stamp.sec + seconds)
		#self.publish_objects_msg.header.stamp.nanosec = int(prediction_msg.header.stamp.nanosec + int((path_point_no+1)*1e8) - seconds*1e9)
		self.publish_objects_msg.header.stamp.sec = prediction_msg.header.stamp.sec
		self.publish_objects_msg.header.stamp.nanosec = prediction_msg.header.stamp.nanosec
	
		self.publish_predicted_object()
		
		self.start_time = time.time()
		while True:
			# This if condition breaks the while loop if the max point has been reached already
			# This if condition prevents the downstream if condition to not malfunction if the max point has been reached already
			# Of course, not publishing a message from here will result in drop in FPS just for that instance (May be for the second alone)
			# Cuz the increase in process time more than 190 or 200 ms is rare and is not continuous I guess
			if path_point_no >= 20:
				break
			wait_time = (200.0 - round(self.carom_process_time))/1000.0
			if time.time() - self.start_time > wait_time:
				#self.publish_objects_msg = DetectedObjects()
				path_point_no = 20
				#print("SECOND path point no.: ", path_point_no)
				self.path_point_extraction(prediction_msg, path_point_no=path_point_no)
				seconds = int((self.publish_objects_msg.header.stamp.nanosec + int(wait_time*1e9))*1e-9)
				self.publish_objects_msg.header.stamp.sec = int(self.publish_objects_msg.header.stamp.sec + seconds)
				self.publish_objects_msg.header.stamp.nanosec = int(self.publish_objects_msg.header.stamp.nanosec + int(wait_time*1e9) - seconds*1e9)
				self.publish_predicted_object()
				break
				

	def publish_predicted_object(self):
		self.publish_objects_msg.header.frame_id = "map"
		#time.sleep(0.01)
		self.predicted_infrastructure_objects_publisher.publish(self.publish_objects_msg)	
	

def main(args=None):
	rclpy.init(args=args)
	predicted_objects_subscriber = PredictedObjectsSubscriber()
	rclpy.spin(predicted_objects_subscriber)
	predicted_objects_subscriber.destroy_node()
	rclpy.shutdown()


