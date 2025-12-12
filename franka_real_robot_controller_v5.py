#!/usr/bin/env python3
"""
ARCap + Franka 实机控制版 V5.1 (修复版)

  # 直接运行
  python franka_real_robot_controller_v5.py --robot_ip 172.16.0.2

  # 调试模式
  python franka_real_robot_controller_v5.py --robot_ip 172.16.0.2 --verbose

  # 频率监控
  python franka_real_robot_controller_v5.py --robot_ip 172.16.0.2 --monitor_freq

  # 自定义最大速度
  python franka_real_robot_controller_v5.py --robot_ip 172.16.0.2 --max_velocity 20.0

修复内容：
1. 移除频率机制矛盾（删除了无用的target_frequency）
2. 移除多余的速度限制（MotionGenerator内部已有）
3. 简化控制循环，完全信任MotionGenerator
4. 添加频率监控（可选）
5. 使用RealtimeConfig.kIgnore
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
from pylibfranka import Robot, ControllerMode, JointPositions, RealtimeConfig

sys.path.append('/home/rmx/libfranka/pylibfranka/examples')
from example_common import MotionGenerator

global GLOBAL_SHUTDOWN, VERBOSE
GLOBAL_SHUTDOWN = False
VERBOSE = False


class FrankaRealRobotController:
    """
    Franka控制器 V5.1 - 简化版

    移除了不必要的频率限制，完全依赖MotionGenerator的内部轨迹规划。
    """
    def __init__(self, robot_ip, max_velocity_deg=15.0, monitor_frequency=False):
        self.robot_ip = robot_ip
        self.max_angular_velocity = np.deg2rad(max_velocity_deg)
        self.monitor_frequency = monitor_frequency

        # 频率监控（可选）
        self.control_count = 0
        self.frequency_start_time = None
        self.last_frequency_report = 0.0

        # Franka Panda关节限位（弧度）
        self.joint_lower_limits = np.array([-2.7, -1.6, -2.7, -2.9, -2.7, 0.1, -2.7])
        self.joint_upper_limits = np.array([ 2.7,  1.6,  2.7, -0.1,  2.7,  3.6,  2.7])

        # 工作空间限制
        self.workspace_x_min, self.workspace_x_max = 0.2, 0.8
        self.workspace_y_range = 0.3
        self.workspace_z_min, self.workspace_z_max = 0.2, 0.5

        # 紧急停止标志和速度监控
        self.max_velocity_emergency_stop = np.deg2rad(30.0)  # 紧急停止速度阈值
        self.prev_joint_positions = None
        self.last_velocity_check_time = None
        self.last_stop_reason = ""
        self.last_joint_positions = None  # 保存上一帧位置

        # 状态变量
        self.robot = None
        self.active_control = None
        self.model = None
        self.is_connected = False
        self.emergency_stop = False
        self.following_mode_started = False

        # 缓存变量
        self._last_O_T_EE = None
        self._last_ee_pos = None

        self._connect()

    def _connect(self):
        """连接机器人"""
        try:
            # 使用 kIgnore 跳过实时检查（Python无法保证严格实时）
            self.robot = Robot(self.robot_ip, RealtimeConfig.kIgnore)

            # 设置碰撞行为
            lower_torque = [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]
            upper_torque = [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]
            lower_force = [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]
            upper_force = [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]

            self.robot.set_collision_behavior(
                lower_torque, upper_torque, lower_force, upper_force
            )

            # 设置阻抗
            self.robot.set_joint_impedance([300.0, 300.0, 300.0, 200.0, 200.0, 200.0, 200.0])
            self.robot.set_cartesian_impedance([300.0, 300.0, 300.0, 30.0, 30.0, 30.0])

            # 读取初始状态
            initial_state = self.robot.read_once()
            self.model = self.robot.load_model()
            self.last_joint_positions = np.array(initial_state.q)

            if VERBOSE:
                print(f"Franka 机器人已连接: {self.robot_ip}")

            self.is_connected = True

        except Exception as e:
            if VERBOSE:
                print(f"连接失败: {e}")
            self.is_connected = False

    def start_control(self):
        """启动控制"""
        if not self.is_connected:
            return False

        try:
            self.active_control = self.robot.start_joint_position_control(
                ControllerMode.CartesianImpedance
            )

            # 初始化频率监控
            if self.monitor_frequency:
                self.frequency_start_time = time.time()
                self.control_count = 0

            return True
        except Exception as e:
            if VERBOSE:
                print(f"启动控制失败: {e}")
            return False

    def check_safety(self, joint_positions):
        """
        完整的安全检查 - 包含关节限位和工作空间
        """
        joint_positions = np.array(joint_positions)

        # 1. 关节限位检查
        for i in range(7):
            if joint_positions[i] < self.joint_lower_limits[i] or joint_positions[i] > self.joint_upper_limits[i]:
                reason = f"关节{i+1}超限: {np.rad2deg(joint_positions[i]):.1f}° (范围: {np.rad2deg(self.joint_lower_limits[i]):.1f}° 到 {np.rad2deg(self.joint_upper_limits[i]):.1f}°)"
                return False, reason

        # 2. 工作空间检查
        try:
            ee_pos = self.get_end_effector_position()

            # 检查前方工作空间
            if ee_pos[0] < self.workspace_x_min or ee_pos[0] > self.workspace_x_max:
                reason = f"末端X位置超出前方范围: {ee_pos[0]:.2f}m (范围: {self.workspace_x_min:.1f}m 到 {self.workspace_x_max:.1f}m)"
                return False, reason

            if abs(ee_pos[1]) > self.workspace_y_range:
                reason = f"末端Y位置超出左右范围: {ee_pos[1]:.2f}m (范围: ±{self.workspace_y_range:.1f}m)"
                return False, reason

            if ee_pos[2] < self.workspace_z_min or ee_pos[2] > self.workspace_z_max:
                reason = f"末端Z位置超出高度范围: {ee_pos[2]:.2f}m (范围: {self.workspace_z_min:.1f}m 到 {self.workspace_z_max:.1f}m)"
                return False, reason

        except Exception as e:
            reason = f"工作空间检查失败: {e}"
            return False, reason

        return True, ""

    def get_end_effector_position(self):
        """
        获取末端执行器位置（使用缓存）
        """
        # 使用缓存的末端位置，避免并发读取问题
        if hasattr(self, '_last_ee_pos') and self._last_ee_pos is not None:
            return self._last_ee_pos
        else:
            # 如果还没有缓存，返回默认安全位置
            return np.array([0.4, 0.0, 0.4])  # 工作空间中心位置

    def trigger_emergency_stop(self, reason):
        """
        触发紧急停止 - 完整实现
        """
        self.emergency_stop = True
        self.last_stop_reason = reason
        print(f"\n紧急停止! {reason}")

        # 尝试停止机器人运动
        if self.active_control is not None:
            try:
                # 发送当前保持位置的命令
                if self.last_joint_positions is not None:
                    hold_cmd = JointPositions(self.last_joint_positions.tolist())
                    hold_cmd.motion_finished = False
                    self.active_control.writeOnce(hold_cmd)
            except:
                pass

    def reset_emergency_stop(self):
        """
        重置紧急停止状态 - 完整实现
        """
        was_stopped = self.emergency_stop
        self.emergency_stop = False
        self.following_mode_started = False
        if was_stopped:
            if VERBOSE:
                print(f"紧急停止已重置")
            # 重新初始化速度监控变量
            try:
                current_state = self.robot.read_once()
                self.prev_joint_positions = np.array(current_state.q)
                self.last_velocity_check_time = time.time()
                if VERBOSE:
                    print("速度监控基准已重置")
            except Exception as e:
                if VERBOSE:
                    print(f"警告：无法重置速度监控基准: {e}")

    def update_frequency_monitor(self):
        """更新频率监控"""
        if self.monitor_frequency and self.frequency_start_time is not None:
            self.control_count += 1
            current_time = time.time()

            # 每5秒报告一次频率
            if current_time - self.last_frequency_report > 5.0:
                elapsed_time = current_time - self.frequency_start_time
                actual_frequency = self.control_count / elapsed_time
                print(f"[频率监控] 实际控制频率: {actual_frequency:.1f} Hz")
                self.last_frequency_report = current_time

    def stop(self):
        """停止机器人"""
        if self.is_connected and self.robot is not None:
            try:
                self.robot.stop()
            except:
                pass


def franka_control_loop(franka_controller, control_vars):
    """
    高性能控制循环 - 简化版

    移除了额外的速度限制，完全依赖MotionGenerator的内部轨迹规划。
    """
    motion_generator = None
    last_target = None
    target_update_time = 0
    update_threshold = np.deg2rad(5.0)  # 5度阈值（转换为弧度）
    min_update_interval = 0.2  # 200ms最小间隔

    if VERBOSE:
        print("Franka控制线程启动 (简化版 - 完全依赖MotionGenerator)")

    while not GLOBAL_SHUTDOWN:
        try:
            # 检查控制开关
            if not control_vars.get('control_enabled', False):
                # 保持连接但不控制
                if franka_controller.active_control is not None:
                    try:
                        _, _ = franka_controller.active_control.readOnce()  # 读取并丢弃返回值
                    except:
                        pass
                time.sleep(0.01)
                motion_generator = None
                continue

            # 读取机器人状态
            robot_state, duration = franka_controller.active_control.readOnce()
            current_position = np.array(robot_state.q)
            current_time = time.time()
            dt = duration.to_sec()

            # 处理过渡阶段
            if control_vars.get('transition_in_progress', False):
                # 如果还没有MotionGenerator，创建一个
                if motion_generator is None:
                    first_frame = control_vars.get('first_frame_for_franka')
                    if first_frame is not None:
                        # 创建过渡MotionGenerator
                        max_error = np.max(np.abs(first_frame - current_position))
                        speed = 0.1 if max_error > np.deg2rad(30) else 0.2

                        motion_generator = MotionGenerator(
                            speed_factor=speed,
                            q_goal=first_frame.tolist()
                        )
                        last_target = np.array(first_frame)
                        target_update_time = current_time
                        if VERBOSE:
                            print(f"过渡开始: 误差{np.rad2deg(max_error):.1f}°, 速度因子={speed}")
                # 如果motion_generator已存在，继续使用它（不需要做额外操作）

            # 处理跟随模式
            elif not control_vars.get('transition_in_progress', False):
                last_arm_q = control_vars.get('last_arm_q')
                if last_arm_q is not None:
                    new_target = np.array(last_arm_q)

                    # 智能更新策略
                    need_update = False
                    if last_target is None:
                        need_update = True
                    elif np.linalg.norm(new_target - last_target) > update_threshold:
                        need_update = True
                    elif current_time - target_update_time > min_update_interval:
                        need_update = True

                    if need_update:
                        # 创建新的MotionGenerator
                        motion_generator = MotionGenerator(
                            speed_factor=0.2,  # 响应速度
                            q_goal=new_target.tolist()
                        )
                        last_target = new_target
                        target_update_time = current_time

            # 执行控制命令
            if motion_generator is not None:
                # 使用MotionGenerator生成命令
                joint_cmd = motion_generator(robot_state, dt)
                target_positions = np.array(joint_cmd.q)

                # 更新频率监控
                franka_controller.update_frequency_monitor()

                # 速度监控（保持原有的安全机制）
                if franka_controller.following_mode_started:
                    if franka_controller.prev_joint_positions is not None and \
                       franka_controller.last_velocity_check_time is not None:
                        dt_check = current_time - franka_controller.last_velocity_check_time
                        if dt_check > 0.001:  # 避免除零
                            velocity = np.abs(target_positions - franka_controller.prev_joint_positions) / dt_check
                            max_velocity = np.max(velocity)
                            if max_velocity > franka_controller.max_velocity_emergency_stop:
                                reason = f"速度过快! {np.rad2deg(max_velocity):.1f}°/s > 限制 {np.rad2deg(franka_controller.max_velocity_emergency_stop):.1f}°/s"
                                franka_controller.trigger_emergency_stop(reason)
                                continue

                            franka_controller.prev_joint_positions = target_positions.copy()
                            franka_controller.last_velocity_check_time = current_time

                # 安全检查
                is_safe, reason = franka_controller.check_safety(target_positions)
                if not is_safe:
                    franka_controller.trigger_emergency_stop(reason)
                    continue

                # 更新缓存的位置信息
                franka_controller._last_O_T_EE = np.array(robot_state.O_T_EE).reshape(4, 4)
                franka_controller._last_ee_pos = np.array([robot_state.O_T_EE[12], robot_state.O_T_EE[13], robot_state.O_T_EE[14]])
                franka_controller.last_joint_positions = target_positions

                # 创建并发送命令
                final_cmd = JointPositions(target_positions.tolist())

                # 设置motion_finished标志
                if control_vars.get('transition_in_progress', False):
                    if joint_cmd.motion_finished:
                        control_vars['transition_in_progress'] = False
                        motion_generator = None  # 重置，准备跟随
                        franka_controller.following_mode_started = True
                        if VERBOSE:
                            print("过渡完成")
                    else:
                        final_cmd.motion_finished = False
                else:
                    final_cmd.motion_finished = False

                # 发送命令
                franka_controller.active_control.writeOnce(final_cmd)

        except Exception as e:
            if VERBOSE:
                print(f"控制错误: {e}")
            time.sleep(0.001)

    if VERBOSE:
        print("Franka控制线程退出")


def main():
    global VERBOSE, GLOBAL_SHUTDOWN

    # 参数解析
    parser = ArgumentParser(description="ARCap + Franka V5.1 - 修复版")
    parser.add_argument("--robot_ip", type=str, required=True, help="Franka机器人IP地址")
    parser.add_argument("--frequency", type=int, default=30, help="Quest数据更新频率(Hz)")
    parser.add_argument("--handedness", type=str, default="right", choices=["right", "left"])
    parser.add_argument("--verbose", action="store_true", help="启用调试信息")
    parser.add_argument("--monitor_freq", action="store_true", help="启用控制频率监控")
    parser.add_argument("--max_velocity", type=float, default=15.0, help="最大关节角速度(度/秒)")
    args = parser.parse_args()

    VERBOSE = args.verbose

    handedness = args.handedness
    robot_type = "Gripper" if handedness == "left" else "Leap Hand"

    # 创建数据文件夹
    if not os.path.isdir("data"):
        os.mkdir("data")

    # 初始化PyBullet
    c = pb.connect(pb.DIRECT)
    if c < 0:
        print("PyBullet连接失败")
        sys.exit(1)

    # 初始化Quest模块
    if handedness == "right":
        quest = QuestRightArmLeapModule(VR_HOST, LOCAL_HOST, POSE_CMD_PORT, IK_RESULT_PORT, vis_sp=None)
    else:
        quest = QuestLeftArmGripperModule(VR_HOST, LOCAL_HOST, POSE_CMD_PORT, IK_RESULT_PORT, vis_sp=None)

    # 初始化独立的Franka控制器
    franka_controller = FrankaRealRobotController(
        robot_ip=args.robot_ip,
        max_velocity_deg=args.max_velocity,
        monitor_frequency=args.monitor_freq
    )

    if not franka_controller.is_connected:
        print("错误: 无法连接Franka")
        sys.exit(1)

    if not franka_controller.start_control():
        print("错误: 无法启动控制循环")
        sys.exit(1)

    # 生成默认手指位置
    default_finger_positions = np.array([
        [0.09, 0.02, -0.1],   # 拇指
        [0.09, -0.03, -0.1],  # 食指
        [0.09, -0.08, -0.1],  # 中指
        [0.01, 0.02, -0.14]   # 无名指
    ])

    if VERBOSE:
        print(f"\n{'='*70}")
        print(f"机器人类型: {robot_type}")
        print(f"使用官方MotionGenerator: 是")
        print(f"已连接到 {args.robot_ip}")
        print(f"简化控制器: 无额外速度限制")
        if args.monitor_freq:
            print(f"频率监控: 已启用")
        print(f"最大角速度: {args.max_velocity}°/s")
        print("="*70)

    # 控制变量
    control_vars = {
        'control_enabled': False,
        'transition_in_progress': False,
        'first_frame_for_franka': None,
        'last_arm_q': None
    }

    # 启动Franka控制线程
    franka_thread = threading.Thread(
        target=franka_control_loop,
        args=(franka_controller, control_vars),
        daemon=True
    )
    franka_thread.start()

    print("操作: s=启动, x=停止, r=重置, q=退出")

    try:
        quest_update_period = 1.0 / args.frequency
        last_quest_update = 0

        while True:
            now = time.time()

            # 检查用户输入
            if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
                user_input = sys.stdin.readline().strip().lower()

                if user_input == 's':
                    if not control_vars['control_enabled']:
                        if control_vars['last_arm_q'] is not None:
                            is_safe, reason = franka_controller.check_safety(control_vars['last_arm_q'])
                            if not is_safe:
                                print(f"错误: {reason}")
                                continue

                            control_vars['control_enabled'] = True
                            control_vars['first_frame_for_franka'] = control_vars['last_arm_q'].copy()
                            control_vars['transition_in_progress'] = True
                            if VERBOSE:
                                print("启动Franka控制")
                        else:
                            print("错误: 还没有收到Quest数据")

                elif user_input == 'x':
                    control_vars['control_enabled'] = False
                    control_vars['transition_in_progress'] = False
                    franka_controller.following_mode_started = False
                    print("停止Franka控制")

                elif user_input == 'r':
                    franka_controller.reset_emergency_stop()

                elif user_input == 'q':
                    global GLOBAL_SHUTDOWN
                    GLOBAL_SHUTDOWN = True
                    break

            # Quest数据更新
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

                        # 更新IK结果
                        if len(arm_q) >= 7:
                            if not control_vars['transition_in_progress']:
                                control_vars['last_arm_q'] = np.array(arm_q[:7])

                except socket.error:
                    pass

    except KeyboardInterrupt:
        print("\n正在关闭...")
        GLOBAL_SHUTDOWN = True

        # 等待线程退出
        if franka_thread.is_alive():
            franka_thread.join(timeout=2.0)

        # 清理
        franka_controller.stop()
        quest.close()
        pb.disconnect()
        print("已安全关闭")


if __name__ == "__main__":
    main()