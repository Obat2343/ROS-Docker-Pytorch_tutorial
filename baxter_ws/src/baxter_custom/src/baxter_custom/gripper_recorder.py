from numpy.lib.npyio import save
import rospy
import cv2
import numpy as np
import baxter_interface
import tf
import os
import time
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

class GripperRecorder(object):
    def __init__(self, base_dir, rate):
        """
        Records joint data to a file at a specified rate.
        """
        self.base_dir = base_dir
        self._raw_rate = rate
        self._rate = rospy.Rate(rate)
        self._start_time = None
        self._done = False
        self.bridge = CvBridge()
        self.latest_image = None

        self._limb_left = baxter_interface.Limb("left")
        self._limb_right = baxter_interface.Limb("right")
        self._gripper_left = baxter_interface.Gripper("left", baxter_interface.CHECK_VERSION)
        self._gripper_right = baxter_interface.Gripper("right", baxter_interface.CHECK_VERSION)
        self.tf_listener = tf.TransformListener()

        # ROS Subscribers
        rospy.Subscriber("/cameras/head_camera/image", Image, self.image_callback)
        rospy.Subscriber("/cameras/head_camera/camera_info", CameraInfo, self.camera_info_callback)

    def image_callback(self, msg):
        """ Save the latest camera image """
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def camera_info_callback(self, msg):
        """ Callback to receive camera intrinsic parameters """
        self.camera_matrix = np.array(msg.K).reshape(3, 3)
        self.dist_coeffs = np.array(msg.D)

        # Adjust camera matrix for image flipping if necessary
        self.camera_matrix[0, 0] *= -1
        self.camera_matrix[1, 1] *= -1
    
    def _time_stamp(self):
        return rospy.get_time() - self._start_time

    def stop(self):
        """ Stop recording. """
        self._done = True

    def done(self):
        """ Return whether or not recording is done. """
        if rospy.is_shutdown():
            self.stop()
        return self._done

    def transform_pose(self, position, orientation, from_frame, to_frame):
        """ Transform both position and orientation from `from_frame` to `to_frame` """
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = from_frame
        pose_stamped.header.stamp = rospy.Time(0)
        pose_stamped.pose.position = Point(*position)
        pose_stamped.pose.orientation = Quaternion(*orientation)

        try:
            self.tf_listener.waitForTransform(to_frame, from_frame, rospy.Time(0), rospy.Duration(1.0))
            transformed_pose = self.tf_listener.transformPose(to_frame, pose_stamped)
            return transformed_pose.pose.position, transformed_pose.pose.orientation
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("TF Transformation failed: {}".format(e))
            return None, None
        
    def create_directory(self):
        """ Create a new directory with a five-digit number """
        existing_folders = sorted([int(f) for f in os.listdir(self.base_dir) if f.isdigit()])
        new_folder_num = (existing_folders[-1] + 1) if existing_folders else 1
        folder_name = "{:05d}".format(new_folder_num)
        save_path = os.path.join(self.base_dir, folder_name)
        os.makedirs(save_path)
        return save_path

    def reset_initial_pose(self):
        """ Move Baxter's arms to a predefined initial pose before recording. """
        rospy.loginfo("Resetting to initial pose...")

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

        # Move the arms to the initial positions
        self._limb_left.move_to_joint_positions(initial_positions_left)
        self._limb_right.move_to_joint_positions(initial_positions_right)

        rospy.loginfo("Initial pose set.")

    def record(self):
        """ Record gripper data after capturing an image """
        # Move Baxter to the initial pose before starting the recording
        self.reset_initial_pose()

        for i in range(5, -1, -1):
            rospy.loginfo("Start recording after {} seconds...".format(i))
            time.sleep(1)

        rospy.loginfo("Recording start")
        save_dir = self.create_directory()

        # Save camera image
        image_path = os.path.join(save_dir, "image.png")
        if self.latest_image is not None:
            cv2.imwrite(image_path, self.latest_image)
            rospy.loginfo("Saved image to {}".format(image_path))
        else:
            rospy.logwarn("No image available!")

        # Wait 5 seconds before recording trajectories
        for i in range(5, -1, -1):
            rospy.loginfo("Waiting {} seconds before recording trajectory...".format(i))
            time.sleep(1)

        # save_camera_params
        if self.camera_matrix is None or self.dist_coeffs is None:
            rospy.logwarn("Camera parameters not available.")
        
        np.save(os.path.join(save_dir, "camera_matrix.npy"), self.camera_matrix)
        np.save(os.path.join(save_dir, "dist_coeffs.npy"), self.dist_coeffs)
        rospy.loginfo("Saved camera parameters as .npy files in {}".format(save_dir))


        # Open CSV files for recording
        original_csv = os.path.join(save_dir, "original_trajectory.csv")
        transformed_csv = os.path.join(save_dir, "transformed_trajectory.csv")

        with open(original_csv, 'w') as f_orig, open(transformed_csv, 'w') as f_trans:
            f_orig.write("time,posx_left,posy_left,posz_left,qx_left,qy_left,qz_left,qw_left,left_gripper," 
                         "posx_right,posy_right,posz_right,qx_right,qy_right,qz_right,qw_right,right_gripper\n")

            f_trans.write("time,posx_left,posy_left,posz_left,qx_left,qy_left,qz_left,qw_left,left_gripper," 
                          "posx_right,posy_right,posz_right,qx_right,qy_right,qz_right,qw_right,right_gripper\n")

            self._start_time = rospy.get_time()
            while not self.done():
                pose_left = self._limb_left.endpoint_pose()
                pose_right = self._limb_right.endpoint_pose()

                left_transformed, left_rot_transformed = self.transform_pose(pose_left['position'], pose_left['orientation'], "base", "head_camera")
                right_transformed, right_rot_transformed = self.transform_pose(pose_right['position'], pose_right['orientation'], "base", "head_camera")

                timestamp = self._time_stamp()

                f_orig.write("{},{},{},{},{},{},{},{},{},".format(
                    timestamp, pose_left['position'][0], pose_left['position'][1], pose_left['position'][2],
                    pose_left['orientation'][0], pose_left['orientation'][1], pose_left['orientation'][2], pose_left['orientation'][3],
                    self._gripper_left.position()
                ))
                f_orig.write("{},{},{},{},{},{},{},{}\n".format(
                    pose_right['position'][0], pose_right['position'][1], pose_right['position'][2],
                    pose_right['orientation'][0], pose_right['orientation'][1], pose_right['orientation'][2], pose_right['orientation'][3],
                    self._gripper_right.position()
                ))

                if left_transformed and left_rot_transformed and right_transformed and right_rot_transformed:
                    f_trans.write("{},{},{},{},{},{},{},{},{},".format(
                        timestamp, left_transformed.x, left_transformed.y, left_transformed.z,
                        left_rot_transformed.x, left_rot_transformed.y, left_rot_transformed.z, left_rot_transformed.w,
                        self._gripper_left.position()
                    ))
                    f_trans.write("{},{},{},{},{},{},{},{}\n".format(
                        right_transformed.x, right_transformed.y, right_transformed.z,
                        right_rot_transformed.x, right_rot_transformed.y, right_rot_transformed.z, right_rot_transformed.w,
                        self._gripper_right.position()
                    ))
                
                self._rate.sleep()
        rospy.loginfo("Trajectory recording completed.")
