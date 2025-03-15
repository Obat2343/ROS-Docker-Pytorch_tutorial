#!/usr/bin/env python

import rospy
import csv
import copy
import struct
import tf
import numpy as np
import argparse
import os
from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
from sensor_msgs.msg import CameraInfo

class IKService:
    def __init__(self):
        self.tf_listener = tf.TransformListener()
        self.camera_matrix = None

        # Subscribe to camera info to get intrinsic parameters
        rospy.Subscriber("/cameras/head_camera/camera_info", CameraInfo, self.camera_info_callback)
    
    def camera_info_callback(self, msg):
        """ Callback to receive camera intrinsic parameters """
        self.camera_matrix = np.array(msg.K).reshape(3, 3)
    
    def call_ik_service(self, limb, position, orientation):
        """
        Calls Baxter's IK service to get joint angles for a given end-effector pose.

        Args:
            limb (str): "left" or "right" arm.
            position (list): [x, y, z] coordinates.
            orientation (list): [qx, qy, qz, qw] quaternion.

        Returns:
            dict: Dictionary of joint names and angles if successful, else None.
        """
        print("Start IK")
        ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        ik_service = rospy.ServiceProxy(ns, SolvePositionIK)
        
        ik_request = SolvePositionIKRequest()
        header = Header(stamp=rospy.Time.now(), frame_id="base")
        
        pose_stamped = PoseStamped(
            header=header,
            pose=Pose(
                position=Point(x=position[0], y=position[1], z=position[2]),
                orientation=Quaternion(x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3])
            )
        )
        
        ik_request.pose_stamp.append(pose_stamped)

        try:
            rospy.wait_for_service(ns, 5)
            response = ik_service(ik_request)
        except rospy.ServiceException as e:
            rospy.logerr("IK Service call failed: {}".format(e))
            return None

        # Check if IK solution is valid
        result_type = struct.unpack('<%dB' % len(response.result_type), response.result_type)
        if result_type[0] != response.RESULT_INVALID:
            joint_angles = dict(zip(response.joints[0].name, response.joints[0].position))
            return joint_angles
        else:
            rospy.logwarn("No valid IK solution found for the given pose.")
            return None

    def transform_pose(self, position, orientation, from_frame, to_frame):
        """
        Transform a pose from one frame to another.
        """
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = from_frame
        pose_stamped.header.stamp = rospy.Time(0)
        pose_stamped.pose.position = Point(*position)
        pose_stamped.pose.orientation = Quaternion(*orientation)

        try:
            self.tf_listener.waitForTransform(to_frame, from_frame, rospy.Time(0), rospy.Duration(1.0))
            transformed_pose = self.tf_listener.transformPose(to_frame, pose_stamped)

            transformed_position = [transformed_pose.pose.position.x, transformed_pose.pose.position.y, transformed_pose.pose.position.z]
            transformed_orientation = [
                transformed_pose.pose.orientation.x,
                transformed_pose.pose.orientation.y,
                transformed_pose.pose.orientation.z,
                transformed_pose.pose.orientation.w
            ]

            return transformed_position, transformed_orientation
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("TF Transformation failed: {}".format(e))
            return None, None

    def uvz_to_xyz(self, uvz):
        """
        Convert UVZ (image coordinates + depth) to XYZ (camera coordinates).
        """
        print("convert uvz to xyz.")
        if self.camera_matrix is None:
            rospy.logerr("Camera matrix not available. Cannot convert UVZ to XYZ.")
            return None

        u, v, depth = uvz
        fx, fy = -self.camera_matrix[0, 0], -self.camera_matrix[1, 1]
        cx, cy = self.camera_matrix[0, 2], self.camera_matrix[1, 2]

        # Convert pixel coordinates to camera coordinates
        x = (u - cx) * depth / fx
        y = (v - cy) * depth / fy
        z = depth  # Depth remains the same

        return [x, y, z]

    def transform_ik(self, limb, position, orientation, input_frame="base"):
        """
        Process the gripper pose based on the input coordinate system.
        """
        print("transform to base coordinate")
        if input_frame == "head_camera":
            # Transform from head_camera to base before solving IK
            transformed_position, transformed_orientation = self.transform_pose(position, orientation, "head_camera", "base")
        else:
            # Already in base frame
            transformed_position, transformed_orientation = position, orientation

        if transformed_position and transformed_orientation:
            return self.call_ik_service(limb, transformed_position, transformed_orientation)
        return None

    def uvz_transform_ik(self, limb, uvz, orientation, input_frame="head_camera"):
        """
        Process the UVZ input (image coordinates + depth) and find IK solution.
        """
        if input_frame != "head_camera":
            raise ValueError("Invalid input frame: input frame should be head_camera.")

        xyz_position = self.uvz_to_xyz(uvz)
        if xyz_position is None:
            return None, None

        # Convert to base frame
        transformed_position, transformed_orientation = self.transform_pose(xyz_position, orientation, "head_camera", "base")

        if transformed_position:
            return xyz_position, self.call_ik_service(limb, transformed_position, transformed_orientation)
        return None, None

def convert_gripper_data(input_filename, output_filename, coordinate_origin, coordinate_system):
    """
    Reads the gripper position data from a CSV file, calculates corresponding joint angles using IK,
    and writes the results to a new CSV file.

    Args:
        input_filename (str): Path to the input CSV file with gripper pose data.
        output_filename (str): Path to the output CSV file to store joint angles.
    """
    ik_service = IKService()

    camera_info_msg = rospy.wait_for_message("/cameras/head_camera/camera_info", CameraInfo, timeout=5)
    if camera_info_msg:
        rospy.loginfo("Received camera_info message!")
    else:
        rospy.logwarn("No camera_info message received within timeout!")

    # Define joint names
    left_joint_names = ["left_s0", "left_s1", "left_e0", "left_e1", "left_w0", "left_w1", "left_w2"]
    right_joint_names = ["right_s0", "right_s1", "right_e0", "right_e1", "right_w0", "right_w1", "right_w2"]

    output_base, ext = os.path.splitext(output_filename)
    xyz_filename = output_base + "_xyz" + ext

    with open(input_filename, 'r') as infile, open(output_filename, 'w') as outfile, open(xyz_filename, 'w') as xyzfile:
        reader = csv.reader(infile)
        writer = csv.writer(outfile)
        xyz_writer = csv.writer(xyzfile)

        # Read and modify header
        header = next(reader)
        writer.writerow(["time"] + left_joint_names + ["left_gripper"] + right_joint_names + ["right_gripper"])
        xyzfile.write("time,posx_left,posy_left,posz_left,qx_left,qy_left,qz_left,qw_left,left_gripper," 
                          "posx_right,posy_right,posz_right,qx_right,qy_right,qz_right,qw_right,right_gripper\n")

        for row in reader:
            row = copy.deepcopy(row)
            timestamp = row[0]
            left_position = list(map(float, row[1:4]))  # posx, posy, posz
            left_orientation = list(map(float, row[4:8]))  # qx, qy, qz, qw
            left_grip = float(row[8])  # Gripper position
            right_position = list(map(float, row[9:12]))  # posx, posy, posz
            right_orientation = list(map(float, row[12:16]))  # qx, qy, qz, qw
            right_grip = float(row[16])  # Gripper position

            if coordinate_origin == "base" and coordinate_system == "xyz":
                left_joint_angles = ik_service.call_ik_service("left", left_position, left_orientation)
                right_joint_angles = ik_service.call_ik_service("right", right_position, right_orientation)
            elif coordinate_origin == "head_camera" and coordinate_system == "xyz":
                left_joint_angles = ik_service.transform_ik("left", left_position, left_orientation, input_frame=coordinate_origin)
                right_joint_angles = ik_service.transform_ik("right", right_position, right_orientation, input_frame=coordinate_origin)
            elif coordinate_origin == "head_camera" and coordinate_system == "uvz":
                left_xyz_position, left_joint_angles = ik_service.uvz_transform_ik("left", left_position, left_orientation, input_frame=coordinate_origin)
                right_xyz_position, right_joint_angles = ik_service.uvz_transform_ik("right", right_position, right_orientation, input_frame=coordinate_origin)
            else:
                raise ValueError("invalid options")

            left_joint_values = [left_joint_angles.get(joint, "N/A") if left_joint_angles else "N/A" for joint in left_joint_names]
            right_joint_values = [right_joint_angles.get(joint, "N/A") if right_joint_angles else "N/A" for joint in right_joint_names]

            if "N/A" in left_joint_values or "N/A" in right_joint_values:
                rospy.logwarn("Skipping row due to N/A values: Time {}".format(timestamp))
                continue

            xyz_writer.writerow([timestamp] + left_xyz_position + left_orientation + [left_grip] + right_xyz_position + right_orientation + [right_grip])
            writer.writerow([timestamp] + left_joint_values + [left_grip] + right_joint_values + [right_grip])

    print("Conversion completed. Output saved to {}".format(output_filename))

if __name__ == "__main__":
    rospy.init_node("convert_gripper_to_joint") 

    parser = argparse.ArgumentParser(description="Convert gripper pose data to joint angles using IK.")
    parser.add_argument("input_file", help="Path to input CSV file with gripper pose data")
    parser.add_argument("output_file", help="Path to output CSV file for joint angles")
    parser.add_argument("coordinate_origin", choices=["base", "head_camera"], help="Choose coordinate system")
    parser.add_argument("coordinate_system", choices=["xyz", "uvz"], help="Choose coordinate representation")

    args = parser.parse_args(rospy.myargv()[1:])  # Avoid ROS internal args
    convert_gripper_data(args.input_file, args.output_file, args.coordinate_origin, args.coordinate_system)