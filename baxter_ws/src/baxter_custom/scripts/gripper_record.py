#!/usr/bin/env python

import argparse
import rospy
import baxter_interface
from baxter_custom import GripperRecorder

def main():
    """Start gripper recording process"""
    parser = argparse.ArgumentParser(description="Record Baxter's gripper movements.")
    parser.add_argument('-d', '--directory', required=True, help='Base directory to save recordings')
    parser.add_argument('-r', '--record-rate', type=int, default=100, help='Recording rate (default: 100)')
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node("baxter_gripper_recorder")
    rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
    rs.enable()

    recorder = GripperRecorder(args.directory, args.record_rate)
    rospy.on_shutdown(recorder.stop)
    
    rospy.loginfo("Recording started. Press Ctrl-C to stop.")
    recorder.record()
    rospy.loginfo("Recording complete.")

if __name__ == '__main__':
    main()
