#!/usr/bin/env python3
"""
ARCap + Franka 实机控制版 (1000Hz版本) - 完整功能保留

用法:
    # 右臂Leap Hand + Franka实机
    python franka_real_robot_controller_v2_fixed.py --robot_ip 192.168.1.100

    # 左臂Gripper + Franka实机
    python franka_real_robot_controller_v2_fixed.py --robot_ip 192.168.1.100 --handedness left
"""

import argparse
import numpy as np
import time
import threading
import select
import sys
import signal
import socket
import os
import pybullet as pb

from scipy.spatial.transform import Rotation
from argparse import ArgumentParser

from quest_robot_module import (
    VR_HOST, LOCAL_HOST, POSE_CMD_PORT, IK_RESULT_PORT,
    QuestRightArmLeapModule, QuestLeftArmGripperModule
)
from franka_real_robot_controller import FrankaRealRobotController, FRANKA_AVAILABLE


def generate_default_hand_positions(handedness="right"):
    """
    生成默认的手指位置(相对于手腕坐标系)
    这些位置对应手指自然伸展的姿态

    返回: 4个指尖位置 [拇指, 食指, 中指, 无名指]
    """
    if handedness == "right":
        # Leap Hand右手默认指尖位置(单位:米)
        tip_positions = np.array([
            [0.09, 0.02, -0.1],   # 拇指
            [0.09, -0.03, -0.1],  # 食指
            [0.09, -0.08, -0.1],  # 中指
            [0.01, 0.02, -0.14]   # 无名指
        ])
    else:
        # Gripper左手默认位置(夹爪中心)
        tip_positions = np.array([
            [0.09, 0.02, -0.1],   # 拇指侧
            [0.09, -0.03, -0.1],  # 食指侧
            [0.09, -0.08, -0.1],  # 中指侧(gripper不用)
            [0.01, 0.02, -0.14]   # 无名指侧(gripper不用)
        ])

    return tip_positions


def franka_control_loop(franka_controller, shared_data, lock):
    """
    Franka控制线程（1000Hz）

    Args:
        franka_controller: Franka控制器实例
        shared_data: 共享数据字典
        lock: 线程锁
    """
    print("Franka控制线程启动 (1000Hz)")

    while True:
        try:
            # 检查是否启用控制
            with lock:
                if not shared_data['control_enabled']:
                    # 控制未启用，只读取状态维持连接
                    if franka_controller.active_control is not None:
                        try:
                            franka_controller.active_control.readOnce()
                        except:
                            pass
                    time.sleep(0.001)
                    continue

                if shared_data['transition_in_progress'] and shared_data['first_frame_for_franka'] is not None:
                    # 处理初始过渡
                    if not hasattr(franka_controller, 'transition_generator'):
                        # 使用MotionGenerator进行平滑过渡
                        import sys
                        sys.path.append('/home/rmx/libfranka/pylibfranka/examples')
                        from example_common import MotionGenerator

                        real_q = franka_controller.last_joint_positions
                        delta_deg = np.rad2deg(np.abs(shared_data['first_frame_for_franka'] - real_q))
                        max_delta = np.max(delta_deg)
                        speed = 0.05 if max_delta > 60 else (0.1 if max_delta > 30 else 0.15)

                        franka_controller.transition_generator = MotionGenerator(
                            speed,
                            shared_data['first_frame_for_franka'].tolist()
                        )

                    # 执行过渡（1000Hz）
                    robot_state, duration = franka_controller.active_control.readOnce()
                    cmd = franka_controller.transition_generator(robot_state, duration.to_sec())
                    franka_controller.active_control.writeOnce(cmd)
                    franka_controller.last_joint_positions = np.array(robot_state.q)

                    if cmd.motion_finished:
                        shared_data['transition_in_progress'] = False
                        # 初始化速度监控变量（跟随模式开始）
                        franka_controller.following_mode_started = True
                        franka_controller.prev_joint_positions = np.array(robot_state.q)  # 实际位置
                        franka_controller.last_velocity_check_time = time.time()
                        print("开始跟随模式")

                else:
                    # 正常跟随模式
                    if shared_data['last_arm_q'] is not None:
                        success = franka_controller.send_joint_positions(shared_data['last_arm_q'])
                        if success:
                            shared_data['franka_cmd_counter'] += 1

        except Exception as e:
            print(f"Franka控制错误: {e}")
            time.sleep(0.01)


def main():
    # 参数解析
    parser = ArgumentParser(description="ARCap + Franka 实机控制 (1000Hz版本)")
    parser.add_argument("--robot_ip", type=str, required=True,
                        help="Franka机器人IP地址")
    parser.add_argument("--frequency", type=int, default=30,
                        help="Quest数据更新频率(Hz),默认30")
    parser.add_argument("--handedness", type=str, default="right",
                        choices=["right", "left"],
                        help="选择机器人类型: right=Leap Hand, left=Gripper")
    args = parser.parse_args()

    # 确定机器人类型
    handedness = args.handedness
    robot_type = "Gripper(平行夹爪)" if handedness == "left" else "Leap Hand(灵巧手)"

    # 创建数据文件夹
    if not os.path.isdir("data"):
        os.mkdir("data")

    # 初始化PyBullet
    c = pb.connect(pb.DIRECT)
    if c < 0:
        print("PyBullet连接失败")
        sys.exit(1)
    print("PyBullet连接成功")

    # 初始化Quest机器人模块
    print(f"\n正在初始化Quest机器人模块 (handedness={handedness})...")
    if handedness == "right":
        quest = QuestRightArmLeapModule(VR_HOST, LOCAL_HOST, POSE_CMD_PORT, IK_RESULT_PORT, vis_sp=None)
        print("右臂Leap Hand模块已加载")
    else:
        quest = QuestLeftArmGripperModule(VR_HOST, LOCAL_HOST, POSE_CMD_PORT, IK_RESULT_PORT, vis_sp=None)
        print("左臂Gripper模块已加载")

    # 初始化 Franka 实机控制器
    if not FRANKA_AVAILABLE:
        print("错误: pylibfranka 未安装")
        sys.exit(1)

    franka_controller = FrankaRealRobotController(
        robot_ip=args.robot_ip,
        control_mode="position",
        frequency=1000,  # 1000Hz
        max_velocity_deg=10.0
    )

    if not franka_controller.is_connected:
        print("错误: 无法连接Franka")
        sys.exit(1)

    if not franka_controller.start_control():
        print("错误: 无法启动控制循环")
        sys.exit(1)

    # 生成默认手指位置
    default_finger_positions = generate_default_hand_positions(handedness)

    # 统计变量
    start_time = time.time()
    fps_counter = 0
    packet_counter = 0
    franka_cmd_counter = 0

    print("\n" + "="*70)
    print(f"\n机器人类型: {robot_type}")
    print(f"Quest IP: {VR_HOST}")
    print(f"本机 IP: {LOCAL_HOST}")
    print(f"Quest数据更新频率: {args.frequency} Hz")
    print(f"Franka控制频率: 1000 Hz")
    print(f"已连接到 {args.robot_ip}")
    print("="*70 + "\n")

    # Franka控制开关
    franka_control_enabled = False
    first_frame_for_franka = None
    transition_in_progress = False

    # 共享数据（线程间通信）
    shared_data = {
        'control_enabled': False,
        'transition_in_progress': False,
        'first_frame_for_franka': None,
        'last_arm_q': None,
        'franka_cmd_counter': 0,
        'packet_counter': 0,
        'fps_counter': 0
    }

    # 创建线程锁
    lock = threading.Lock()

    # 启动Franka控制线程（1000Hz）
    franka_thread = threading.Thread(
        target=franka_control_loop,
        args=(franka_controller, shared_data, lock),
        daemon=True
    )
    franka_thread.start()

    print("操作+enter: s=启动, x=停止, r=重置急停, q=退出")
    print("")

    try:
        quest_update_frequency = args.frequency  # Quest数据更新频率
        quest_update_period = 1.0 / quest_update_frequency
        last_quest_update = 0
        last_arm_q = None  # 保存最后一次的IK结果

        while True:
            now = time.time()

            # 检查用户输入（非阻塞）
            if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
                user_input = sys.stdin.readline().strip().lower()
                if user_input == 's':
                    # 启动Franka控制
                    if not franka_control_enabled:
                        with lock:
                            if last_arm_q is not None:
                                franka_control_enabled = True
                                shared_data['control_enabled'] = True
                                first_frame_for_franka = last_arm_q.copy()
                                shared_data['first_frame_for_franka'] = last_arm_q.copy()
                                transition_in_progress = True
                                shared_data['transition_in_progress'] = True
                                print("启动Franka控制...")
                            else:
                                print("错误: 还没有收到Quest数据")
                    else:
                        pass

                elif user_input == 'x':
                    # 停止Franka控制
                    if franka_control_enabled:
                        with lock:
                            franka_control_enabled = False
                            shared_data['control_enabled'] = False
                            transition_in_progress = False
                            shared_data['transition_in_progress'] = False
                            franka_controller.following_mode_started = False
                            franka_controller.prev_joint_positions = None
                            franka_controller.last_velocity_check_time = None
                            print("停止Franka控制")
                    else:
                        pass

                elif user_input == 'r':
                    # 重置紧急停止
                    franka_controller.reset_emergency_stop()

                elif user_input == 'q':
                    print("退出程序...")
                    break

            # Quest数据更新（保持原频率）
            if now - last_quest_update >= quest_update_period:
                last_quest_update = now

                try:
                    wrist, head_pose = quest.receive()

                    if wrist is not None:
                        wrist_pos = wrist[0]
                        wrist_orn = Rotation.from_quat(wrist[1])
                        head_pos = head_pose[0]
                        head_orn = Rotation.from_quat(head_pose[1])

                        hand_tip_pose = wrist_orn.apply(default_finger_positions) + wrist_pos

                        if handedness == "right":
                            hand_tip_pose[[0,1,2,3]] = hand_tip_pose[[1,2,3,0]]

                        arm_q, hand_q, wrist_pos_out, wrist_orn_out = quest.solve_system_world(
                            wrist_pos, wrist_orn, hand_tip_pose
                        )

                        action = quest.send_ik_result(arm_q, hand_q)

                        # 保存IK结果供高频控制循环使用
                        with lock:
                            if len(arm_q) >= 7:
                                last_arm_q = np.array(arm_q[:7])
                                shared_data['last_arm_q'] = np.array(arm_q[:7])
                            else:
                                print("IK结果长度不足跳过此帧")

                            # 如果正在录制,保存数据
                            if quest.data_dir is not None:
                                point_cloud = np.zeros((1000, 3))  # dummy点云(无相机模式)

                                if handedness == "right":
                                    np.savez(
                                        f"{quest.data_dir}/right_data_{time.time()}.npz",
                                        right_wrist_pos=wrist_pos_out,
                                        right_wrist_orn=wrist_orn_out,
                                        head_pos=head_pos,
                                        head_orn=head_orn.as_quat(),
                                        right_arm_q=arm_q,
                                        right_hand_q=action,
                                        raw_hand_q=hand_q,
                                        right_tip_poses=hand_tip_pose,
                                        point_cloud=point_cloud
                                    )
                                else:
                                    np.savez(
                                        f"{quest.data_dir}/left_data_{time.time()}.npz",
                                        left_wrist_pos=wrist_pos_out,
                                        left_wrist_orn=wrist_orn_out,
                                        head_pos=head_pos,
                                        head_orn=head_orn.as_quat(),
                                        left_arm_q=arm_q,
                                        left_hand_q=action,
                                        raw_hand_q=hand_q,
                                        left_tip_poses=hand_tip_pose,
                                        point_cloud=point_cloud
                                    )

                            # 统计数据接收
                            packet_counter += 1
                            shared_data['packet_counter'] += 1
                            fps_counter += 1
                            shared_data['fps_counter'] += 1

                except socket.error as e:
                    # 网络错误,继续等待
                    pass

    except KeyboardInterrupt:
        print("\n\n正在关闭...")
        franka_controller.stop()
        quest.close()
        pb.disconnect()

        # 获取最终统计
        with lock:
            final_packet_counter = shared_data['packet_counter'] + packet_counter
            final_franka_cmd_counter = shared_data['franka_cmd_counter'] + franka_cmd_counter

        print(f"\n已关闭 | Quest: {final_packet_counter} pkt | Franka: {final_franka_cmd_counter} cmd")


if __name__ == "__main__":
    main()