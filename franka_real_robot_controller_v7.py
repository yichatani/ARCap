#!/usr/bin/env python3
"""
python franka_real_robot_controller_v7.py --handedness left
python franka_real_robot_controller_v7.py --handedness right
"""

import numpy as np
import time
import signal
import sys
import socket
import os
import select
from argparse import ArgumentParser
from scipy.spatial.transform import Rotation

import rospy
from std_msgs.msg import Float64MultiArray

from ip_config import *
from quest_robot_module import QuestRightArmLeapModule, QuestLeftArmGripperModule
import pybullet as pb


def transform_ar_to_real(position, quaternion):
    """
    Transform AR frame to real robot frame
    Rotate 90 degrees clockwise around Z axis
    AR Y axis -> Real X axis
    """
    # Position transformation
    pos = np.array(position)
    R_z = np.array([
        [0, -1, 0],
        [1, 0, 0],
        [0, 0, 1]
    ])
    real_pos = R_z @ pos

    # Quaternion transformation (rotate 90 deg clockwise around Z)
    rot_quat = np.array([0.7071068, 0, 0, 0.7071068])  # [w, x, y, z]

    ar_rotation = Rotation.from_quat(quaternion)
    rot_rotation = Rotation.from_quat(rot_quat)

    real_rotation = rot_rotation * ar_rotation
    real_quat = real_rotation.as_quat()

    return real_pos, real_quat


class GracefulKiller:
    def __init__(self):
        self.kill_now = False
        signal.signal(signal.SIGINT, self.exit_gracefully)
        signal.signal(signal.SIGTERM, self.exit_gracefully)

    def exit_gracefully(self, signum, frame):
        print("\nStopping...")
        self.kill_now = True


def main():
    parser = ArgumentParser(description="ARCap + Franka")
    parser.add_argument("--frequency", type=int, default=30,
                       help="Quest data update frequency (Hz)")
    parser.add_argument("--handedness", type=str, default="right",
                       choices=["right", "left"])
    parser.add_argument("--verbose", action="store_true",
                       help="Enable verbose logging")
    parser.add_argument("--max_joint_change", type=float, default=0.1,
                       help="Max joint angle change threshold (radians)")
    args = parser.parse_args()

    killer = GracefulKiller()
    handedness = args.handedness
    robot_type = "Gripper" if handedness == "left" else "Leap Hand"

    print(f"\n{'='*70}")
    print(f"ARCap + Franka")
    print(f"Robot type: {robot_type}")
    print(f"Quest IP: {VR_HOST}")
    print(f"Local IP: {LOCAL_HOST}")
    print(f"{'='*70}\n")

    # Create data folder
    if not os.path.isdir("data"):
        os.mkdir("data")

    # Initialize PyBullet
    pb_c = pb.connect(pb.DIRECT)
    if pb_c < 0:
        print("PyBullet connection failed")
        sys.exit(1)

    # Initialize ROS node
    rospy.init_node("arcap_franka_control_v7", anonymous=True)

    # Create ROS publisher for Franka action
    action_pub = rospy.Publisher("/fr3_action", Float64MultiArray, queue_size=1)


    # Initialize Quest module
    print("\nInitializing Quest connection...")
    try:
        if handedness == "right":
            quest = QuestRightArmLeapModule(VR_HOST, LOCAL_HOST, POSE_CMD_PORT, IK_RESULT_PORT, vis_sp=None)
        else:
            quest = QuestLeftArmGripperModule(VR_HOST, LOCAL_HOST, POSE_CMD_PORT, IK_RESULT_PORT, vis_sp=None)
        print("Quest module loaded")
    except Exception as e:
        print(f"Quest init failed: {e}")
        sys.exit(1)

    # Default finger positions
    default_finger_positions = np.array([
        [0.09, 0.02, -0.1],
        [0.09, -0.03, -0.1],
        [0.09, -0.08, -0.1],
        [0.01, 0.02, -0.14]
    ])

    # Control variables
    control_enabled = False
    following_mode = False
    last_arm_q = None
    data_count = 0
    control_count = 0

    print("\n" + "="*70)
    print("  Enter 'start'  - Move to initial position")
    print("  Enter 'follow' - Start following")
    print("  Enter 'stop'   - Stop")
    print("  Ctrl+C        - Exit")
    print("="*70 + "\n")

    dt = 1.0 / args.frequency

    try:
        while not rospy.is_shutdown() and not killer.kill_now:
            loop_start = time.time()

            # Receive Quest data
            try:
                wrist, head_pose = quest.receive()

                if wrist is not None:
                    wrist_pos = wrist[0]
                    wrist_orn = Rotation.from_quat(wrist[1])
                    real_pos, real_quat = transform_ar_to_real(wrist[0], wrist[1])
                    #print(f"real pose:{real_pos}")
                    head_pos = head_pose[0]
                    head_orn = Rotation.from_quat(head_pose[1])
                    hand_tip_pose = wrist_orn.apply(default_finger_positions) + wrist_pos

                    if handedness == "right":
                        hand_tip_pose[[0,1,2,3]] = hand_tip_pose[[1,2,3,0]]

                    arm_q, hand_q, wrist_pos_out, wrist_orn_out = quest.solve_system_world(
                        wrist_pos, wrist_orn, hand_tip_pose
                    )

                    action = quest.send_ik_result(arm_q, hand_q)

                    if len(arm_q) >= 7:
                        new_arm_q = np.array(arm_q[:7])
                        last_arm_q = new_arm_q
                        data_count += 1

            except socket.error:
                pass
            except Exception as e:
                if args.verbose:
                    print(f"Quest error: {e}")

            # Control logic - send Cartesian pose to ROS topic
            if control_enabled and wrist is not None:
                msg = Float64MultiArray()

                if following_mode:
                    # Following mode: send current wrist pose
                    tx, ty, tz = real_pos
                    qx, qy, qz, qw = real_quat
                    width = 0.04

                    msg.data = [tx, ty, tz, qx, qy, qz, qw, width]
                    action_pub.publish(msg)
                    control_count += 1

                else:
                    # Initial positioning mode: send first received pose
                    width = 0.04

                    msg.data = [itx, ity, itz, iqx, iqy, iqz, iqw, width]
                    action_pub.publish(msg)
                    control_count += 1

            # Check user input
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                user_input = sys.stdin.readline().strip().lower()

                if user_input == 'start':
                    if last_arm_q is not None:
                        itx,ity,itz = real_pos
                        iqx, iqy, iqz, iqw = real_quat
                        control_enabled = True
                        following_mode = False
                        print("\nMoving to initial position")
                    else:
                        print("\nNo Quest data yet")

                elif user_input == 'follow':
                    if control_enabled:
                        following_mode = True
                        print("\nFollowing started")
                    else:
                        print("\nEnter 'start' first")

                elif user_input == 'stop':
                    control_enabled = False
                    following_mode = False
                    print("\nStopped")

            # Maintain fixed rate
            elapsed = time.time() - loop_start
            sleep_time = max(0, dt - elapsed)
            time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n\nShutting down...")

    finally:
        quest.close()
        pb.disconnect()

        print(f"\nCompleted")
        print(f"  Data packets: {data_count}")
        print(f"  Control updates: {control_count}")


if __name__ == "__main__":
    main()