import rclpy
from rclpy.node import Node

from autoware_auto_perception_msgs.msg import PredictedObjects
from autoware_auto_perception_msgs.msg import PredictedObject

from std_msgs.msg import Header
from autoware_auto_perception_msgs.msg import DetectedObjects
from autoware_auto_perception_msgs.msg import DetectedObject
from autoware_auto_perception_msgs.msg import ObjectClassification

from std_msgs.msg import Float32

class PredictedObjectsSubscriber(Node):
	def __init__(self):
		super().__init__('predicted_objects_subscriber')

		self.predicted_objects_subscription = self.create_subscription(PredictedObjects, "/perception/object_recognition/prediction/path/carom/objects", \
			self.publish_predicted_object, 10)
		
		self.carom_process_time_subscriber = self.create_subscription(Float32, '/perception/object_recognition/detection/carom/objects/delay', \
												self.carom_process_time_updater, 10)

		self.predicted_infrastructure_objects_publisher = self.create_publisher(DetectedObjects, \
                '/perception/object_recognition/prediction/point/carom/objects', 10)
		
	def carom_process_time_updater(self, process_time_msg):
		self.carom_process_time = process_time_msg.data
		#print("carom process time: ", self.carom_process_time)
		
	def publish_predicted_object(self, prediction_msg):
		#self.get_logger().info('prediction_msg {}'.format(prediction_msg.objects[0].kinematics.predicted_paths[0].path[9]))
		publish_objects_msg = DetectedObjects()
		objects = [0 for o in range(len(prediction_msg.objects))]
		classification = [0]
		path_point_no = round(self.carom_process_time/10.0)-1 
		for i in range(len(prediction_msg.objects)):
			# path[9] - no. 9 here means prediction at 100th ms. The window size in the prediction algorithm set to 10ms
			
			objects[i] = DetectedObject()
			classification[0] = ObjectClassification()
			objects[i].existence_probability = 1.0
			classification[0].label = 1
			objects[i].classification = list(classification)
			objects[i].kinematics.pose_with_covariance.pose.position.x = prediction_msg.objects[i].kinematics.predicted_paths[0].path[path_point_no].position.x
			objects[i].kinematics.pose_with_covariance.pose.position.y = prediction_msg.objects[i].kinematics.predicted_paths[0].path[path_point_no].position.y
			objects[i].kinematics.pose_with_covariance.pose.position.z = prediction_msg.objects[i].kinematics.predicted_paths[0].path[path_point_no].position.z
			objects[i].kinematics.pose_with_covariance.pose.orientation.x = prediction_msg.objects[i].kinematics.predicted_paths[0].path[path_point_no].orientation.x
			objects[i].kinematics.pose_with_covariance.pose.orientation.y = prediction_msg.objects[i].kinematics.predicted_paths[0].path[path_point_no].orientation.y
			objects[i].kinematics.pose_with_covariance.pose.orientation.z = prediction_msg.objects[i].kinematics.predicted_paths[0].path[path_point_no].orientation.z
			objects[i].kinematics.pose_with_covariance.pose.orientation.w = prediction_msg.objects[i].kinematics.predicted_paths[0].path[path_point_no].orientation.w
			objects[i].kinematics.orientation_availability = 1
			objects[i].shape.dimensions.x = prediction_msg.objects[i].shape.dimensions.x
			objects[i].shape.dimensions.y = prediction_msg.objects[i].shape.dimensions.y
			objects[i].shape.dimensions.z = prediction_msg.objects[i].shape.dimensions.z
			
		seconds = int((prediction_msg.header.stamp.nanosec + int((path_point_no+1)*1e7))*1e-9)
		
		publish_objects_msg.header.stamp.sec = prediction_msg.header.stamp.sec + seconds
		publish_objects_msg.header.stamp.nanosec = prediction_msg.header.stamp.nanosec + int((path_point_no+1)*1e7) - seconds
		publish_objects_msg.objects = objects
		publish_objects_msg.header.frame_id = "map"
		
		self.predicted_infrastructure_objects_publisher.publish(publish_objects_msg)


def main(args=None):
	rclpy.init(args=args)
	predicted_objects_subscriber = PredictedObjectsSubscriber()
	rclpy.spin(predicted_objects_subscriber)
	predicted_objects_subscriber.destroy_node()
	rclpy.shutdown()


