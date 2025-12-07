#!/usr/bin/env python3
"""
ARCap + Franka 实机控制版

用法:
    # 右臂Leap Hand + Franka实机
    python franka_real_robot_controller.py --robot_ip 172.16.0.2

    # 左臂Gripper + Franka实机
    python franka_real_robot_controller.py --robot_ip 172.16.0.2 --handedness left
"""

import numpy as np
import time
import threading
import select
import sys
import socket
import os
import pybullet as pb

from scipy.spatial.transform import Rotation
from argparse import ArgumentParser
from ip_config import *
from quest_robot_module import QuestRightArmLeapModule, QuestLeftArmGripperModule
from franka_real_robot_controller import FrankaRealRobotController, FRANKA_AVAILABLE

# 添加线程安全的终止标志
GLOBAL_SHUTDOWN = False


def generate_default_hand_positions(handedness="right"):
    """
    生成默认的手指位置(相对���手腕坐标系)
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


def franka_control_loop(franka_controller, control_vars):
    """
    Franka控制线程

    Args:
        franka_controller: Franka控制器实例
        control_vars: 控制变量字典
    """
    print("Franka控制线程启动 (1000Hz)")

    while not GLOBAL_SHUTDOWN:
        try:
            # 检查全局终止标志
            if GLOBAL_SHUTDOWN:
                print("Franka控制线程收到终止信号")
                break

            # 检查是否启用控制
            if not control_vars.get('control_enabled', False):
                # 控制未启用，等待
                time.sleep(0.01)
                continue

            # 过渡阶段
            if control_vars.get('transition_in_progress', False):
                first_frame = control_vars.get('first_frame_for_franka')
                if first_frame is not None:
                    # 发送目标位置
                    success = franka_controller.send_joint_positions(first_frame)

                    if success and franka_controller.last_joint_positions is not None:
                        # 检查是否接近目标
                        position_error = np.linalg.norm(
                            first_frame - franka_controller.last_joint_positions
                        )
                        # 如果误差小于阈值，认为过渡完成
                        if position_error < 0.05:  # 约3度误差
                            control_vars['transition_in_progress'] = False
                            # 初始化速度监控变量（跟随模式开始）
                            franka_controller.following_mode_started = True
                            franka_controller.prev_joint_positions = franka_controller.last_joint_positions.copy()
                            franka_controller.last_velocity_check_time = time.time()
                            print(f"过渡完成，误差: {np.rad2deg(position_error):.2f}°")
                            print("开始跟随模式")

            else:
                # 正常跟随模式
                last_arm_q = control_vars.get('last_arm_q')
                if last_arm_q is not None:
                    franka_controller.send_joint_positions(last_arm_q)

        except Exception as e:
            print(f"Franka控制错误: {e}")
            if not GLOBAL_SHUTDOWN:
                time.sleep(0.01)

    print("Franka控制线程已安全退出")


def main():
    # 参数解析
    parser = ArgumentParser(description="ARCap + Franka 实机控制")
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
        max_velocity_deg=10.0  # 每秒最大10度，确保安全
    )

    if not franka_controller.is_connected:
        print("错误: 无法连接Franka")
        sys.exit(1)

    if not franka_controller.start_control():
        print("错误: 无法启动控制循环")
        sys.exit(1)

    # 生成默认手指位置
    default_finger_positions = generate_default_hand_positions(handedness)

    
    print("\n" + "="*70)
    print(f"\n机器人类型: {robot_type}")
    print(f"Quest IP: {VR_HOST}")
    print(f"本机 IP: {LOCAL_HOST}")
    print(f"Quest数据更新频率: {args.frequency} Hz")
    print(f"Franka控制频率: 1000 Hz")
    print(f"最大关节速度: 10°/s (安全限制)")
    print(f"已连接到 {args.robot_ip}")
    print("="*70 + "\n")

    
    # 控制变量
    control_vars = {
        'control_enabled': False,
        'transition_in_progress': False,
        'first_frame_for_franka': None,
        'last_arm_q': None
    }

    # 启动Franka控制线程（1000Hz）
    franka_thread = threading.Thread(
        target=franka_control_loop,
        args=(franka_controller, control_vars),
        daemon=True
    )
    franka_thread.start()

    print("操作+enter: s=启动, x=停止, r=重置急停, q=退出")
    print("")

    try:
        quest_update_frequency = args.frequency  # Quest数据更新频率
        quest_update_period = 1.0 / quest_update_frequency
        last_quest_update = 0

        while True:
            now = time.time()

            # 检查用户输入（非阻塞）
            if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
                user_input = sys.stdin.readline().strip().lower()
                if user_input == 's':
                    # 启动Franka控制
                    if not control_vars['control_enabled']:
                        if control_vars['last_arm_q'] is not None:
                            # 安全检查目标位置
                            is_safe, reason = franka_controller.check_safety(control_vars['last_arm_q'])
                            if not is_safe:
                                print(f"错误：目标位置不安全 - {reason}")
                                continue

                            control_vars['control_enabled'] = True
                            control_vars['first_frame_for_franka'] = control_vars['last_arm_q'].copy()
                            control_vars['transition_in_progress'] = True
                            print("启动Franka控制...")
                            print(f"目标位置: {np.round(np.rad2deg(control_vars['last_arm_q']), 1)}°")
                        else:
                            print("错误: 还没有收到Quest数据")

                elif user_input == 'x':
                    # 停止Franka控制
                    if control_vars['control_enabled']:
                        control_vars['control_enabled'] = False
                        control_vars['transition_in_progress'] = False
                        franka_controller.following_mode_started = False
                        franka_controller.prev_joint_positions = None
                        franka_controller.last_velocity_check_time = None
                        print("停止Franka控制")

                elif user_input == 'r':
                    # 重置紧急停止
                    franka_controller.reset_emergency_stop()

                elif user_input == 'q':
                    print("退出程序...")
                    # 设置全局终止标志
                    global GLOBAL_SHUTDOWN
                    GLOBAL_SHUTDOWN = True
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
                        # 只在非过渡阶段更新，避免目标位置变化
                        if len(arm_q) >= 7:
                            if not control_vars['transition_in_progress']:
                                control_vars['last_arm_q'] = np.array(arm_q[:7])
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

                            
                except socket.error as e:
                    # 网络错误,继续等待
                    pass

    except KeyboardInterrupt:
        print("\n\n正在关闭...")
        # 设置全局终止标志
        global GLOBAL_SHUTDOWN
        GLOBAL_SHUTDOWN = True

        # 等待Franka线程退出
        if franka_thread.is_alive():
            print("等待Franka控制线程退出...")
            franka_thread.join(timeout=2.0)
            if franka_thread.is_alive():
                print("警告：Franka线程未能及时退出")

        # 安全停止机器人
        franka_controller.stop()
        quest.close()
        pb.disconnect()
        print("\n已安全关闭")


if __name__ == "__main__":
    main()