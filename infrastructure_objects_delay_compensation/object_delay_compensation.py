import rclpy
from rclpy.node import Node

from autoware_auto_perception_msgs.msg import PredictedObjects
from autoware_auto_perception_msgs.msg import PredictedObject

from std_msgs.msg import Header
from autoware_auto_perception_msgs.msg import DetectedObjects
from autoware_auto_perception_msgs.msg import DetectedObject
from autoware_auto_perception_msgs.msg import ObjectClassification

class PredictedObjectsSubscriber(Node):
	def __init__(self):
		super().__init__('predicted_objects_subscriber')

		self.predicted_objects_subscription = self.create_subscription(PredictedObjects, "/perception/object_recognition/prediction/path/carom/objects", \
			self.publish_predicted_object, 10)
		self.predicted_objects_subscription

		self.predicted_infrastructure_objects_publisher = self.create_publisher(DetectedObjects, \
                '/perception/object_recognition/prediction/point/carom/objects', 10)

	def publish_predicted_object(self, prediction_msg):
		#self.get_logger().info('prediction_msg {}'.format(prediction_msg.objects[0].kinematics.predicted_paths[0].path[9]))
		publish_objects_msg = DetectedObjects()
		objects = [0 for o in range(len(prediction_msg.objects))]
		classification = [0]
		for i in range(len(prediction_msg.objects)):
			# path[9] - no. 9 here means prediction at 100th ms. The window size in the prediction algorithm set to 10ms
			predicted_px = prediction_msg.objects[i].kinematics.predicted_paths[0].path[14].position.x
			predicted_py = prediction_msg.objects[i].kinematics.predicted_paths[0].path[14].position.y
			predicted_pz = prediction_msg.objects[i].kinematics.predicted_paths[0].path[14].position.z
			predicted_qx = prediction_msg.objects[i].kinematics.predicted_paths[0].path[14].orientation.x
			predicted_qy = prediction_msg.objects[i].kinematics.predicted_paths[0].path[14].orientation.y
			predicted_qz = prediction_msg.objects[i].kinematics.predicted_paths[0].path[14].orientation.z
			predicted_qw = prediction_msg.objects[i].kinematics.predicted_paths[0].path[14].orientation.w
			object_classification = prediction_msg.objects[i].classification[0]
			dim_x = prediction_msg.objects[i].shape.dimensions.x
			dim_y = prediction_msg.objects[i].shape.dimensions.y
			dim_z = prediction_msg.objects[i].shape.dimensions.z

			objects[i] = DetectedObject()
			classification[0] = ObjectClassification()
			objects[i].existence_probability = 1.0
			classification[0].label = 1
			objects[i].classification = list(classification)
			objects[i].kinematics.pose_with_covariance.pose.position.x = prediction_msg.objects[i].kinematics.predicted_paths[0].path[14].position.x
			objects[i].kinematics.pose_with_covariance.pose.position.y = prediction_msg.objects[i].kinematics.predicted_paths[0].path[14].position.y
			objects[i].kinematics.pose_with_covariance.pose.position.z = prediction_msg.objects[i].kinematics.predicted_paths[0].path[14].position.z
			objects[i].kinematics.pose_with_covariance.pose.orientation.x = prediction_msg.objects[i].kinematics.predicted_paths[0].path[14].orientation.x
			objects[i].kinematics.pose_with_covariance.pose.orientation.y = prediction_msg.objects[i].kinematics.predicted_paths[0].path[14].orientation.y
			objects[i].kinematics.pose_with_covariance.pose.orientation.z = prediction_msg.objects[i].kinematics.predicted_paths[0].path[14].orientation.z
			objects[i].kinematics.pose_with_covariance.pose.orientation.w = prediction_msg.objects[i].kinematics.predicted_paths[0].path[14].orientation.w
			objects[i].kinematics.orientation_availability = 1
			objects[i].shape.dimensions.x = dim_x
			objects[i].shape.dimensions.y = dim_y
			objects[i].shape.dimensions.z = dim_z

		publish_objects_msg.objects = objects
		publish_objects_msg.header = prediction_msg.header
		self.predicted_infrastructure_objects_publisher.publish(publish_objects_msg)


def main(args=None):
	rclpy.init(args=args)
	predicted_objects_subscriber = PredictedObjectsSubscriber()
	rclpy.spin(predicted_objects_subscriber)
	predicted_objects_subscriber.destroy_node()
	rclpy.shutdown()


