#!/usr/bin/env python

import rospy
import csv
import struct
import baxter_interface
from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header

def call_ik_service(limb, position, orientation):
    """
    Calls Baxter's IK service to get joint angles for a given end-effector pose.
    
    Args:
        limb (str): "left" or "right" arm.
        position (list): [x, y, z] coordinates.
        orientation (list): [qx, qy, qz, qw] quaternion.

    Returns:
        dict: Dictionary of joint names and angles if successful, else None.
    """
    rospy.wait_for_service("ExternalTools/{}//PositionKinematicsNode/IKService".format(limb))
    ik_service = rospy.ServiceProxy("ExternalTools/{}/PositionKinematicsNode/IKService".format(limb), SolvePositionIK)
    
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

def convert_gripper_data(input_filename, output_filename):
    """
    Reads the gripper position data from a CSV file, calculates corresponding joint angles using IK,
    and writes the results to a new CSV file.

    Args:
        input_filename (str): Path to the input CSV file with gripper pose data.
        output_filename (str): Path to the output CSV file to store joint angles.
    """
    rospy.init_node("convert_gripper_to_joint")

    # Define joint names
    left_joint_names = ["left_s0", "left_s1", "left_e0", "left_e1", "left_w0", "left_w1", "left_w2"]
    right_joint_names = ["right_s0", "right_s1", "right_e0", "right_e1", "right_w0", "right_w1", "right_w2"]

    with open(input_filename, 'r') as infile, open(output_filename, 'w') as outfile:
        reader = csv.reader(infile)
        writer = csv.writer(outfile)

        # Read and modify header
        header = next(reader)
        writer.writerow(["time"] + left_joint_names + ["left_gripper"] + right_joint_names + ["right_gripper"])

        for row in reader:
            timestamp = row[0]
            left_position = list(map(float, row[1:4]))  # posx, posy, posz
            left_orientation = list(map(float, row[4:8]))  # qx, qy, qz, qw
            left_grip = float(row[8])  # Gripper position
            right_position = list(map(float, row[9:12]))  # posx, posy, posz
            right_orientation = list(map(float, row[12:16]))  # qx, qy, qz, qw
            right_grip = float(row[16])  # Gripper position

            left_joint_angles = call_ik_service("left", left_position, left_orientation)
            right_joint_angles = call_ik_service("right", right_position, right_orientation)

            left_joint_values = [left_joint_angles.get(joint, "N/A") if left_joint_angles else "N/A" for joint in left_joint_names]
            right_joint_values = [right_joint_angles.get(joint, "N/A") if right_joint_angles else "N/A" for joint in right_joint_names]

            writer.writerow([timestamp] + left_joint_values + [left_grip] + right_joint_values + [right_grip])

    print("Conversion completed. Output saved to {}".format(output_filename))

if __name__ == "__main__":
    input_file = "gripper_positions.csv"  # Change to the actual filename
    output_file = "joint_angles.csv"

    convert_gripper_data(input_file, output_file)
