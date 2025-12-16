#!/usr/bin/env python3
"""
ARCap + Franka 实机控制版 V6

用法:
  python franka_real_robot_controller_v6.py --robot_ip 172.16.0.2

  # 启用调试
  python franka_real_robot_controller_v6.py --robot_ip 172.16.0.2 --verbose

  # 指定频率
  python franka_real_robot_controller_v6.py --robot_ip 172.16.0.2 --frequency 30
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

sys.path.insert(0, '/home/ani/ExDex/data_collection/franka_ws/async_pos_ctrl_cpp/franka_async_controller/build')
import franka_controller as fc
from ip_config import *
from quest_robot_module import QuestRightArmLeapModule, QuestLeftArmGripperModule
import pybullet as pb


def check_workspace_safety(ee_pose_matrix):
    """检查工作空间安全性"""
    position = ee_pose_matrix[:3, 3]

    # 定义安全工作空间
    x_min, x_max = 0.4, 0.8    # 前方范围
    y_limit = 0.3              # 左右范围
    z_min, z_max = 0.3, 0.8    # 高度范围

    # 检查边界
    if position[0] < x_min or position[0] > x_max:
        return False, f"X位置超出范围: {position[0]:.3f}m (范围: {x_min}-{x_max}m)"

    if abs(position[1]) > y_limit:
        return False, f"Y位置超出范围: {position[1]:.3f}m (范围: ±{y_limit}m)"

    if position[2] < z_min or position[2] > z_max:
        return False, f"Z位置超出范围: {position[2]:.3f}m (范围: {z_min}-{z_max}m)"

    return True, ""


class GracefulKiller:
    """处理Ctrl+C退出"""
    def __init__(self):
        self.kill_now = False
        signal.signal(signal.SIGINT, self.exit_gracefully)
        signal.signal(signal.SIGTERM, self.exit_gracefully)

    def exit_gracefully(self, signum, frame):
        print("\n正在停止...")
        self.kill_now = True


def main():
    # 参数解析
    parser = ArgumentParser(description="ARCap + Franka")
    parser.add_argument("--robot_ip", type=str, required=True,
                       help="Franka机器人IP地址")
    parser.add_argument("--frequency", type=int, default=30,
                       help="Quest数据更新频率(Hz), 默认30")
    parser.add_argument("--handedness", type=str, default="right",
                       choices=["right", "left"],
                       help="选择机器人类型")
    parser.add_argument("--verbose", action="store_true",
                       help="启用详细日志")
    args = parser.parse_args()

    robot_ip = args.robot_ip
    killer = GracefulKiller()

    # 机器人类型
    handedness = args.handedness
    robot_type = "Gripper(平行夹爪)" if handedness == "left" else "Leap Hand(灵巧手)"

    print(f"\n{'='*70}")
    print(f"ARCap + Franka")
    print(f"机器人类型: {robot_type}")
    print(f"机器人IP: {robot_ip}")
    print(f"Quest IP: {VR_HOST}")
    print(f"本地IP: {LOCAL_HOST}")
    print(f"更新频率: {args.frequency} Hz")
    print(f"{'='*70}\n")

    # 创建数据文件夹
    if not os.path.isdir("data"):
        os.mkdir("data")

    pb_c = pb.connect(pb.DIRECT)
    if pb_c < 0:
        print("PyBullet连接失败")
        sys.exit(1)

    # 配置Franka控制器
    print("正在初始化Franka控制器...")
    config = fc.ControllerConfig()
    config.verbose = args.verbose
    config.control_period_ms = 20
    config.goal_tolerance = 10.0
    config.maximum_joint_velocities = [0.3, 0.3, 0.3, 0.3, 0.6, 0.6, 0.6] #减半速度
    print(f"连接Franka机器人 {robot_ip}...")
    controller = fc.FrankaAsyncController(robot_ip, config)
    if not controller.start():
        print(f"控制器启动失败: {controller.get_error_message()}")
        sys.exit(1)

    time.sleep(0.5)

    # 获取初始状态
    initial_state = controller.get_robot_state()
    if initial_state is None:
        print("无法获取机器人状态")
        controller.stop()
        sys.exit(1)

    initial_q = initial_state.q.copy()
    if args.verbose:
        print(f"初始关节位置: {initial_q}")

    # 初始化Quest模块
    print("\n初始化Quest连接...")
    try:
        if handedness == "right":
            quest = QuestRightArmLeapModule(VR_HOST, LOCAL_HOST, POSE_CMD_PORT, IK_RESULT_PORT, vis_sp=None)
        else:
            quest = QuestLeftArmGripperModule(VR_HOST, LOCAL_HOST, POSE_CMD_PORT, IK_RESULT_PORT, vis_sp=None)
    except Exception as e:
        print(f"Quest模块初始化失败: {e}")
        controller.stop()
        sys.exit(1)

    default_finger_positions = np.array([
        [0.09, 0.02, -0.1],   # 拇指
        [0.09, -0.03, -0.1],  # 食指
        [0.09, -0.08, -0.1],  # 中指
        [0.01, 0.02, -0.14]   # 无名指
    ])

    # 控制变量
    control_enabled = False
    following_mode = False
    initial_target = None
    last_arm_q = None
    data_count = 0
    control_count = 0

    print("\n" + "="*70)
    print("\n本地控制:")
    print("  输入 'start'  - 移动到初始位置")
    print("  输入 'follow' - 开始跟随模式")
    print("  输入 'stop'   - 停止机器人")
    print("  按 Ctrl+C    - 退出程序")
    print("="*70 + "\n")

    control_rate = args.frequency
    dt = 1.0 / control_rate
    start_time = time.time()

    try:
        while not killer.kill_now and not controller.has_error():
            loop_start = time.time()
            elapsed = loop_start - start_time

            # 尝试接收Quest数据
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

                    if len(arm_q) >= 7:
                        last_arm_q = np.array(arm_q[:7])
                        data_count += 1

                    # 统计
                    # if data_count % 30 == 0:
                    #     print(f"✓ 收到Quest数据: {data_count} 包 | 控制更新: {control_count} 次", end="\r")

            except socket.error:
                pass
            except Exception as e:
                if args.verbose:
                    print(f"Quest处理错误: {e}")

            # 控制逻辑
            if control_enabled and initial_target is not None:
                state = controller.get_robot_state()
                # 安全检查
                if state:
                    is_safe, warning = check_workspace_safety(state.O_T_EE)
                    if not is_safe:
                        print(f"\安全警告: {warning}")
                        control_enabled = False
                        following_mode = False
                        initial_target = None
                        continue

                # 发送目标
                if following_mode and last_arm_q is not None:
                    success = controller.set_joint_position_target(last_arm_q)
                else:
                    success = controller.set_joint_position_target(initial_target)

                if success:
                    control_count += 1

                    if args.verbose and control_count % 10 == 0 and state:
                        position = state.O_T_EE[:3, 3]
                        print(f"\n关节0: {state.q[0]:.4f} rad | 位置: [{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}] m")

            # 检查用户输入
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                user_input = sys.stdin.readline().strip().lower()

                if user_input == 'start':
                    if last_arm_q is not None:
                        initial_target = last_arm_q.copy()
                        control_enabled = True
                        following_mode = False
                        print(f"\n移动到初始位置: {initial_target}")
                    else:
                        print("\n错误: 还没有收到Quest数据")

                elif user_input == 'follow':
                    if control_enabled and initial_target is not None:
                        following_mode = True
                        print("\n开始跟随模式")
                    else:
                        print("\n请先输入 'start' 移动到初始位置")

                elif user_input == 'stop':
                    control_enabled = False
                    following_mode = False
                    initial_target = None
                    print("\n机器人控制已停止")

            # 维持固定频率
            elapsed_loop = time.time() - loop_start
            sleep_time = max(0, dt - elapsed_loop)
            time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n\n正在关闭...")

    finally:
        # 停止控制器
        print("\n停止Franka控制器...")
        if controller.is_running():
            controller.stop()
        try:
            quest.close()
        except:
            pass
        try:
            pb.disconnect()
        except:
            pass

        # 最终状态
        if controller.has_error():
            print(f"控制器错误: {controller.get_error_message()}")
        else:
            print(f"\n程序完成")
            print(f"  收到数据包: {data_count}")
            print(f"  控制更新次数: {control_count}")

            # 显示最终状态
            final_state = controller.get_robot_state()
            if final_state:
                print(f"  最终关节位置: {final_state.q}")


if __name__ == "__main__":
    main()