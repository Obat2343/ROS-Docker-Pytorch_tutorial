#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import baxter_interface
import tf
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

class GripperProjection:
    def __init__(self):
        rospy.init_node("gripper_projection")

        # Baxter's head camera topic
        self.image_topic = "/cameras/head_camera/image"
        self.bridge = CvBridge()
        self.latest_image = None

        # Camera intrinsic parameters
        self.camera_matrix = None  # Camera matrix from /camera_info topic
        self.dist_coeffs = None  # Distortion coefficients from /camera_info topic

        # TF listener for coordinate transformation
        self.tf_listener = tf.TransformListener()

        # ROS publishers
        self.image_pub = rospy.Publisher("/cameras/head_camera/gripper_marked_image", Image, queue_size=1)

        # Subscribe to the camera image and camera info topics
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback)
        self.camera_info_sub = rospy.Subscriber("/cameras/head_camera/camera_info", CameraInfo, self.camera_info_callback)

    def camera_info_callback(self, msg):
        """ Callback to receive camera intrinsic parameters """
        self.camera_matrix = np.array(msg.K).reshape(3, 3)
        self.dist_coeffs = np.array(msg.D)

        # Adjust camera matrix for image flipping if necessary
        self.camera_matrix[0, 0] *= -1
        self.camera_matrix[1, 1] *= -1

    def image_callback(self, msg):
        """ Callback to receive head camera image """
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.process_and_publish()
        except Exception as e:
            rospy.logerr("Failed to convert image: {}".format(e))

    def get_gripper_pose(self, limb):
        """ Retrieve the current pose of the specified arm's gripper """
        arm = baxter_interface.Limb(limb)
        pose = arm.endpoint_pose()
        return pose["position"], pose["orientation"]

    def transform_pose(self, position, orientation, from_frame, to_frame):
        """ Transform coordinates from `from_frame` to `to_frame` """
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = from_frame
        pose_stamped.header.stamp = rospy.Time(0)
        pose_stamped.pose.position = Point(*position)
        pose_stamped.pose.orientation = Quaternion(*orientation)

        try:
            self.tf_listener.waitForTransform(to_frame, from_frame, rospy.Time(0), rospy.Duration(1.0))
            transformed_pose = self.tf_listener.transformPose(to_frame, pose_stamped)
            return transformed_pose.pose.position
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("TF Transformation failed: {}".format(e))
            return None

    def project_to_image_plane(self, position):
        """ Project 3D coordinates to 2D image pixel coordinates """
        point_3D = np.array([position.x, position.y, position.z, 1])  # Homogeneous coordinates
        point_2D = np.dot(self.camera_matrix, point_3D[:3] / point_3D[2])
        return int(point_2D[0]), int(point_2D[1])

    def process_and_publish(self):
        """ Process image and publish with gripper markers """
        if self.latest_image is None:
            return

        # Get the current gripper pose
        left_pos, left_ori = self.get_gripper_pose("left")
        right_pos, right_ori = self.get_gripper_pose("right")

        # Transform gripper position to head camera frame
        left_transformed = self.transform_pose(left_pos, left_ori, "base", "head_camera")
        right_transformed = self.transform_pose(right_pos, right_ori, "base", "head_camera")

        if left_transformed and right_transformed:
            # Project onto image plane
            left_px = self.project_to_image_plane(left_transformed)
            right_px = self.project_to_image_plane(right_transformed)

            # Draw markers on the image
            marked_image = self.latest_image.copy()
            cv2.circle(marked_image, left_px, 10, (0, 0, 255), -1)  # Left gripper in red
            cv2.circle(marked_image, right_px, 10, (255, 0, 0), -1)  # Right gripper in blue

            # Convert and publish the processed image
            ros_image = self.bridge.cv2_to_imgmsg(marked_image, encoding="bgr8")
            print("pub to /cameras/head_camera/gripper_marked_image. time: {}".format(rospy.get_time()))
            self.image_pub.publish(ros_image)

if __name__ == "__main__":
    gp = GripperProjection()
    rospy.spin()
