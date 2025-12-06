#!/usr/bin/env python3
"""
ARCap + Franka 实机控制版 (1000Hz版本)

用法:
    # 右臂Leap Hand + Franka实机
    python franka_real_robot_controller_v2.py --robot_ip 192.168.1.100

    # 左臂Gripper + Franka实机
    python franka_real_robot_controller_v2.py --robot_ip 192.168.1.100 --handedness left
"""

import argparse
import numpy as np
import time
import threading
import select
import sys
import signal
import socket
import pybullet as pb

from scipy.spatial.transform import Rotation

from quest_robot_module import QuestAR
from franka_real_robot_controller import FrankaRealRobotController
from data_recording import DataRecorder


def signal_handler(sig, frame):
    """Ctrl+C处理"""
    print('\n正在关闭...')
    sys.exit(0)


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
            if not shared_data['control_enabled']:
                # 控制未启用，等待
                time.sleep(0.01)
                continue

            # 读取机器人状态（1000Hz）
            robot_state, duration = franka_controller.active_control.readOnce()

            with lock:
                if shared_data['transition_in_progress']:
                    # 过渡阶段
                    if not hasattr(franka_controller, 'transition_generator'):
                        # 创建MotionGenerator
                        from example_common import MotionGenerator

                        real_q = franka_controller.last_joint_positions
                        delta_deg = np.rad2deg(np.abs(shared_data['first_frame_target'] - real_q))
                        max_delta = np.max(delta_deg)
                        speed = 0.05 if max_delta > 60 else (0.1 if max_delta > 30 else 0.15)

                        franka_controller.transition_generator = MotionGenerator(
                            speed,
                            shared_data['first_frame_target'].tolist()
                        )

                    # 执行过渡轨迹
                    cmd = franka_controller.transition_generator(robot_state, duration.to_sec())
                    franka_controller.active_control.writeOnce(cmd)
                    franka_controller.last_joint_positions = np.array(robot_state.q)

                    # 检查过渡是否完成
                    if cmd.motion_finished:
                        shared_data['transition_in_progress'] = False
                        # 初始化速度监控
                        franka_controller.following_mode_started = True
                        franka_controller.prev_joint_positions = np.array(robot_state.q)
                        franka_controller.last_velocity_check_time = time.time()
                        print("跟随模式开始")

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
    # 信号处理
    signal.signal(signal.SIGINT, signal_handler)

    # 参数解析
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot_ip", type=str, default="192.168.100.2", help="Franka机器人IP地址")
    parser.add_argument("--handedness", type=str, default="right", choices=["right", "left"])
    parser.add_argument("--record", action="store_true", help="录制数据")
    parser.add_argument("--frequency", type=float, default=30, help="Quest数据更新频率")
    parser.add_argument("--max_velocity_deg", type=float, default=10.0, help="最大关节角速度(度/秒)")
    args = parser.parse_args()

    # 初始化组件
    print("\n=== ARCap + Franka 实机控制 v2 (1000Hz) ===\n")

    # 1. 初始化PyBullet（静默模式）
    pb.connect(pb.DIRECT)
    pb.setGravity(0, 0, -9.81)
    pb.setAdditionalSearchPath(pb.getDataPath())

    # 2. 初始化Franka
    print("初始化Franka...")
    franka_controller = FrankaRealRobotController(
        args.robot_ip,
        control_mode="position",
        frequency=1000,  # 内部使用1000Hz
        max_velocity_deg=args.max_velocity_deg
    )

    # 3. 初始化Quest
    print("\n初始化Quest AR...")
    handedness = args.handedness
    quest = QuestAR(handedness=handedness, panda=True)
    quest.connect()

    # 4. 初始化数据录制
    if args.record:
        data_recorder = DataRecorder(handedness=handedness, output_dir="recorded_data")
        data_recorder.start_recording()
        print("\n开始录制数据...")

    # 5. 生成默认手指位置
    default_finger_positions = quest.generate_default_hand_positions(handedness)
    print(f"手指默认位置: {default_finger_positions}")

    # 6. 共享数据（线程间通信）
    shared_data = {
        'control_enabled': False,
        'transition_in_progress': False,
        'first_frame_target': None,
        'last_arm_q': None,
        'franka_cmd_counter': 0,
        'packet_counter': 0,
        'fps_counter': 0
    }

    # 7. 创建线程锁
    lock = threading.Lock()

    # 8. 启动Franka控制线程（1000Hz）
    franka_thread = threading.Thread(
        target=franka_control_loop,
        args=(franka_controller, shared_data, lock),
        daemon=True
    )
    franka_thread.start()

    # 9. 主线程处理Quest数据（30Hz）
    print("\n控制启动成功！")
    print("操作+enter: s=启动, x=停止, r=重置急停, q=退出")
    print("")

    # 统计变量
    start_time = time.time()
    last_fps_print = start_time
    last_quest_update = 0
    quest_update_period = 1.0 / args.frequency

    try:
        while True:
            now = time.time()

            # 检查用户输入（非阻塞）
            if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
                user_input = sys.stdin.readline().strip().lower()

                if user_input == 's':
                    # 启动Franka控制
                    with lock:
                        if not shared_data['control_enabled']:
                            if shared_data['last_arm_q'] is not None:
                                shared_data['control_enabled'] = True
                                shared_data['first_frame_target'] = shared_data['last_arm_q'].copy()
                                shared_data['transition_in_progress'] = True
                                print("启动Franka控制...")
                            else:
                                print("错误: 还没有收到Quest数据")

                elif user_input == 'x':
                    # 停止Franka控制
                    with lock:
                        if shared_data['control_enabled']:
                            shared_data['control_enabled'] = False
                            shared_data['transition_in_progress'] = False
                            franka_controller.following_mode_started = False
                            print("停止Franka控制")

                elif user_input == 'r':
                    # 重置紧急停止
                    franka_controller.reset_emergency_stop()

                elif user_input == 'q':
                    print("退出程序...")
                    break

            # Quest数据更新（30Hz）
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

                        # 保存IK结果到共享数据
                        with lock:
                            if len(arm_q) >= 7:
                                shared_data['last_arm_q'] = np.array(arm_q[:7])
                            else:
                                print("⚠️ IK结果长度不足，跳过此帧")

                            shared_data['packet_counter'] += 1
                            shared_data['fps_counter'] += 1

                        # 录制数据
                        if args.record:
                            point_cloud = np.zeros((1000, 3))
                            data_recorder.record_frame(
                                wrist_pos_out, wrist_orn_out.as_quat(),
                                head_pos, head_orn.as_quat(),
                                arm_q, action, hand_q,
                                hand_tip_pose, point_cloud
                            )

                except socket.error as e:
                    # 网络错误，继续等待
                    pass

            # 打印统计信息（每5秒）
            if now - last_fps_print >= 5.0:
                with lock:
                    elapsed = now - last_fps_print
                    quest_fps = shared_data['fps_counter'] / elapsed
                    franka_cps = shared_data['franka_cmd_counter'] / elapsed

                    print(f"\rQuest: {quest_fps:.1f} fps | Franka: {franka_cps:.1f} cmd/s", end="")

                    shared_data['fps_counter'] = 0
                    shared_data['franka_cmd_counter'] = 0
                    last_fps_print = now

    except KeyboardInterrupt:
        print("\n\n正在关闭...")

    finally:
        # 清理资源
        franka_controller.stop()
        quest.close()
        pb.disconnect()

        if args.record:
            data_recorder.stop_recording()

        print(f"\n已关闭")


if __name__ == "__main__":
    main()