#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import os
import argparse
import baxter_interface
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

class HeadCameraCapture:
    def __init__(self, save_dir):
        """
        Initializes the head camera capture node and resets Baxter's arms to initial positions.
        """
        rospy.init_node("head_camera_capture")
        self.save_dir = save_dir
        self.bridge = CvBridge()
        self.latest_image = None
        self.camera_matrix = None
        self.dist_coeffs = None

        # Initialize Baxter's arms
        self._limb_left = baxter_interface.Limb("left")
        self._limb_right = baxter_interface.Limb("right")

        # Reset Baxter's arms to initial positions
        self.reset_initial_pose()

        # Create directory if it does not exist
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)

        # Subscribe to head camera image and camera info
        rospy.Subscriber("/cameras/head_camera/image", Image, self.image_callback)
        rospy.Subscriber("/cameras/head_camera/camera_info", CameraInfo, self.camera_info_callback)

    def reset_initial_pose(self):
        """ Move Baxter's arms to a predefined initial pose before recording. """
        rospy.loginfo("Resetting Baxter's arms to initial pose...")

        # Define the initial joint positions
        initial_positions_left = {
            'left_s0': 0.0,
            'left_s1': -1.0,
            'left_e0': 0.0,
            'left_e1': 1.5,
            'left_w0': 0.0,
            'left_w1': 1.0,
            'left_w2': 0.0
        }

        initial_positions_right = {
            'right_s0': 0.0,
            'right_s1': -1.0,
            'right_e0': 0.0,
            'right_e1': 1.5,
            'right_w0': 0.0,
            'right_w1': 1.0,
            'right_w2': 0.0
        }

        # Move both arms to their initial positions
        self._limb_left.move_to_joint_positions(initial_positions_left)
        self._limb_right.move_to_joint_positions(initial_positions_right)

        rospy.loginfo("Initial pose set.")

    def image_callback(self, msg):
        """ Callback function to store the latest head camera image. """
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr("Failed to convert image: {}".format(e))

    def camera_info_callback(self, msg):
        """ Callback function to store the latest camera intrinsic parameters. """
        self.camera_matrix = np.array(msg.K).reshape(3, 3)
        self.dist_coeffs = np.array(msg.D)

    def save_data(self):
        """ Save the captured image and camera parameters in the specified directory. """
        if self.latest_image is None:
            rospy.logwarn("No image available!")
            return

        # Save image
        image_path = os.path.join(self.save_dir, "image.png")
        cv2.imwrite(image_path, self.latest_image)
        rospy.loginfo("Saved image to {}".format(image_path))

        # Save camera parameters
        if self.camera_matrix is not None and self.dist_coeffs is not None:
            np.save(os.path.join(self.save_dir, "camera_matrix.npy"), self.camera_matrix)
            np.save(os.path.join(self.save_dir, "dist_coeffs.npy"), self.dist_coeffs)

            rospy.loginfo("Saved camera parameters to {}".format(self.save_dir))
        else:
            rospy.logwarn("Camera parameters not available!")

    def run(self):
        """ Main loop to capture images and save data when Enter is pressed. """
        rospy.loginfo("Press Enter to capture an image. Press Ctrl+C to exit.")
        try:
            while not rospy.is_shutdown():
                raw_input("Press Enter to capture an image...")
                self.save_data()
        except KeyboardInterrupt:
            rospy.loginfo("Exiting...")
            rospy.signal_shutdown("User interrupted")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Capture images from Baxter's head camera and save them.")
    parser.add_argument("save_dir", help="Directory to save images and camera parameters")
    
    args = parser.parse_args(rospy.myargv()[1:])  # ROS-safe argument parsing
    
    camera_capture = HeadCameraCapture(args.save_dir)
    camera_capture.run()
