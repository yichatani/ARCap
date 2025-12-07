#!/usr/bin/env python3
"""
ARCap + Franka 实机控制版

用法:
    # 右臂Leap Hand + Franka实机
    python franka_real_robot_controller.py --robot_ip 172.16.0.2

    # 左臂Gripper + Franka实机
    python franka_real_robot_controller.py --robot_ip 172.16.0.2 --handedness left
"""

import os
import socket
import time
import sys
import select
from argparse import ArgumentParser
import numpy as np
from scipy.spatial.transform import Rotation
import pybullet as pb
from rigidbodySento import create_primitive_shape
from ip_config import *
from quest_robot_module import QuestRightArmLeapModule, QuestLeftArmGripperModule

try:
    from pylibfranka import Robot, ControllerMode, JointPositions
    FRANKA_AVAILABLE = True
except ImportError:
    FRANKA_AVAILABLE = False



class FrankaRealRobotController:
    """
    Franka 实机控制器 - 慢速安全追踪版本
    机械臂会以限制的速度缓慢追踪目标位置，确保运动安全
    """
    def __init__(self, robot_ip, control_mode="position", frequency=1000, max_velocity_deg=20.0):
        """
        初始化 Franka 控制器

        Args:
            robot_ip: Franka 机器人IP地址
            control_mode: 控制模式 ("position" 或 "velocity")
            frequency: 控制频率 (Hz)
            max_velocity_deg: 最大关节角速度 (度/秒)
        """
        self.robot_ip = robot_ip
        self.control_mode = control_mode
        self.frequency = frequency
        self.robot = None
        self.active_control = None
        self.last_joint_positions = None
        self.is_connected = False

        # 速度限制参数
        self.max_angular_velocity = np.deg2rad(max_velocity_deg)  # 转换为弧度/秒
        self.max_position_per_frame = self.max_angular_velocity / frequency  # 每帧最大移动量

        # Franka Panda关节限位（弧度）
        self.joint_lower_limits = np.array([-2.7, -1.6, -2.7, -2.9, -2.7, 0.1, -2.7])
        self.joint_upper_limits = np.array([ 2.7,  1.6,  2.7, -0.1,  2.7,  3.6,  2.7])

        self.workspace_x_min = 0.2 
        self.workspace_x_max = 0.8  
        self.workspace_y_range = 0.3 
        self.workspace_z_min = 0.2  
        self.workspace_z_max = 0.5  

        # 紧急停止标志和速度监控
        self.emergency_stop = False
        self.last_stop_reason = ""
        self.max_velocity_emergency_stop = np.deg2rad(30.0)  # 紧急停止速度阈值 30度/秒
        self.prev_joint_positions = None  # 跟随模式开始时的实际位置
        self.last_velocity_check_time = None  # 跟随模式开始时间
        self.following_mode_started = False  # 是否已进入跟随模式

        print(f"\n正在连接 Franka 机器人 ")

        try:
            # 连接机器人
            self.robot = Robot(robot_ip)

            # 设置碰撞行为(安全阈值)
            lower_torque_thresholds = [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]
            upper_torque_thresholds = [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]
            lower_force_thresholds = [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]
            upper_force_thresholds = [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]

            self.robot.set_collision_behavior(
                lower_torque_thresholds,
                upper_torque_thresholds,
                lower_force_thresholds,
                upper_force_thresholds,
            )

            # 设置关节阻抗
            self.robot.set_joint_impedance([300.0, 300.0, 300.0, 200.0, 200.0, 200.0, 200.0])

            # 设置笛卡尔阻抗
            self.robot.set_cartesian_impedance([300.0, 300.0, 300.0, 30.0, 30.0, 30.0])

            # 读取初始状态
            initial_state = self.robot.read_once()
            self.model = self.robot.load_model()
            self.last_joint_positions = np.array(initial_state.q)

            print(f"Franka 机器人已连接!")
            print(f"初始关节位置: {np.round(self.last_joint_positions, 3)}")
            self.is_connected = True

        except Exception as e:
            print(f"连接 Franka 失败: {e}")
            self.is_connected = False

    def start_control(self):
        """启动实时控制循环"""
        if not self.is_connected:
            return False

        try:
            print(f"\n启动 Franka 关节位置控制...")
            self.active_control = self.robot.start_joint_position_control(
                ControllerMode.CartesianImpedance
            )
            print("控制循环已启动!")
            return True
        except Exception as e:
            print(f"启动控制失败: {e}")
            return False

    def send_joint_positions(self, joint_positions, motion_finished=False):
        """
        发送关节位置命令到 Franka

        Args:
            joint_positions: 7个关节角度 (弧度)
            motion_finished: 是否结束运动

        Returns:
            success: 是否成功发送
        """
        if not self.is_connected or self.active_control is None:
            return False

        try:
            joint_positions = np.array(joint_positions)

            if self.following_mode_started:
                # 安全检查：确保变量已初始化
                if self.prev_joint_positions is None or self.last_velocity_check_time is None:
                    print("警告：速度监控变量未初始化，跳过速度检查")
                    self.prev_joint_positions = joint_positions.copy()
                    self.last_velocity_check_time = time.time()
                    return False if self.emergency_stop else True

                current_time = time.time()
                dt = current_time - self.last_velocity_check_time
                if dt > 0.001:  # 避免除零
                    velocity = np.abs(joint_positions - self.prev_joint_positions) / dt
                    max_velocity = np.max(velocity)
                    if max_velocity > self.max_velocity_emergency_stop:
                        reason = f"速度过快! {np.rad2deg(max_velocity):.1f}°/s > 限制 {np.rad2deg(self.max_velocity_emergency_stop):.1f}°/s"
                        self.trigger_emergency_stop(reason)
                        return False

                    self.prev_joint_positions = joint_positions.copy()
                    self.last_velocity_check_time = current_time

            if self.emergency_stop:
                return False

            is_safe, reason = self.check_safety(joint_positions)
            if not is_safe:
                self.trigger_emergency_stop(reason)
                return False

            if self.last_joint_positions is not None:
                # 计算需要移动的距离
                position_error = joint_positions - self.last_joint_positions
                abs_position_error = np.abs(position_error)

                # 计算每帧允许的最大移动量
                max_step = self.max_position_per_frame
                print(f"上一帧位置: {np.round(np.rad2deg(self.last_joint_positions), 1)}°")
                # 对每个关节进行速度限制
                limited_positions = self.last_joint_positions.copy()
                for i in range(7):
                    if abs_position_error[i] > max_step:
                        # 限制移动速度
                        step_direction = np.sign(position_error[i])
                        limited_positions[i] += step_direction * max_step
                    else:
                        limited_positions[i] = joint_positions[i]

                max_error = np.max(abs_position_error)
                estimated_time = max_error / self.max_angular_velocity
                if estimated_time > 0.5:  # 如果需要超过0.5秒，打印提示
                    print(f"慢速移动中: 需要 {estimated_time:.1f}秒 到达目标位置", end='\r')
                print(f"规划帧位置: {np.round(np.rad2deg(limited_positions), 1)}°")
                joint_positions = limited_positions

            # 读取机器人状态(同步控制循环)
            robot_state, _ = self.active_control.readOnce()

            # 缓存O_T_EE和末端位置（供工作空间检查使用）
            self._last_O_T_EE = np.array(robot_state.O_T_EE).reshape(4, 4)
            # 使用原始数据直接获取末端位置（索引12,13,14）
            self._last_ee_pos = np.array([robot_state.O_T_EE[12], robot_state.O_T_EE[13], robot_state.O_T_EE[14]])
            
            print(f"dangqian位置: {np.round(np.rad2deg(np.array(robot_state.q)), 1)}°")
            # 创建关节位置命令
            joint_cmd = JointPositions(joint_positions.tolist())
            joint_cmd.motion_finished = motion_finished

            # 发送命令
            self.active_control.writeOnce(joint_cmd)

            # 更新上一次的关节位置代理
            self.last_joint_positions = np.array(joint_positions)

            return True

        except Exception as e:
            print(f"发送命令失败: {e}")
            return False

    def stop(self):
        """停止控制并断开连接"""
        if self.is_connected and self.robot is not None:
            try:
                print("\n正在停止 Franka 机器人...")
                self.robot.stop()
                print("Franka 已停止")
            except Exception as e:
                print(f"停止机器人时出错: {e}")

    def check_safety(self, joint_positions):
        """
        综合安全检查

        Args:
            joint_positions: 7个关节角度 (弧度)

        Returns:
            (is_safe, reason): 是否安全，不安全的原因
        """
        # 1. 关节限位检查
        for i in range(7):
            if joint_positions[i] < self.joint_lower_limits[i] or joint_positions[i] > self.joint_upper_limits[i]:
                reason = f"关节{i+1}超限: {np.rad2deg(joint_positions[i]):.1f}° (范围: {np.rad2deg(self.joint_lower_limits[i]):.1f}° 到 {np.rad2deg(self.joint_upper_limits[i]):.1f}°)"
                return False, reason

        # 2. 工作空间检查
        try:
            ee_pos = self.get_end_effector_position(joint_positions)

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
    
    def get_end_effector_position(self, joint_positions):
        """
        获取末端执行器位置（使用缓存的O_T_EE，避免并发冲突）

        Args:
            joint_positions: 7个关节角度 (弧度)
        Returns:
            ee_pos: 末端执行器位置 [x, y, z]
        """
        # 使用缓存的末端位置，避免并发读取问题
        if hasattr(self, '_last_ee_pos'):
            return self._last_ee_pos
        else:
            # 如果还没有缓存，返回默认安全位置
            print("警告: 尚无末端位置缓存，返回默认位置")
            return np.array([0.4, 0.0, 0.4])  # 工作空间中心位置

    def trigger_emergency_stop(self, reason):
        """
        触发紧急停止
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
        重置紧急停止状态
        """
        was_stopped = self.emergency_stop
        self.emergency_stop = False
        if was_stopped:
            print(f"紧急停止已重置")
            # 重新初始化速度监控变量（需要新的基准位置）
            try:
                current_state = self.robot.read_once()
                self.prev_joint_positions = np.array(current_state.q)
                self.last_velocity_check_time = time.time()
                print("速度监控基准已重置")
            except Exception as e:
                print(f"警告：无法重置速度监控基准: {e}")


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


if __name__ == "__main__":
    parser = ArgumentParser(description="ARCap + Franka 实机控制")
    parser.add_argument("--robot_ip", type=str, required=True,
                        help="Franka机器人IP地址")
    parser.add_argument("--frequency", type=int, default=30,
                        help="控制频率(Hz),默认30")
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
        frequency=30,
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
    fps_counter = 0
    packet_counter = 0
    franka_cmd_counter = 0
    print("\n" + "="*70)
    print(f"\n机器人类型: {robot_type}")
    print(f"Quest IP: {VR_HOST}")
    print(f"本机 IP: {LOCAL_HOST}")
    print(f"更新频率: {args.frequency} Hz")
    print(f"已连接到 {args.robot_ip}")
    print("="*70 + "\n")

    franka_control_enabled = False  # Franka机械臂控制开关
    first_frame_for_franka = None   # 启用时的第一帧位置
    transition_in_progress = False  # 是否正在进行初始过渡

    print("操作+enter: s=启动, x=停止, r=重置急停, q=退出")
    print("")

    try:
        quest_update_frequency = args.frequency  # Quest数据更新频率
        franka_control_frequency = 30  # Franka控制频率
        quest_update_period = 1.0 / quest_update_frequency
        franka_control_period = 1.0 / franka_control_frequency

        last_quest_update = 0
        last_arm_q = None  # 保存最后一次的IK结果
        current_ts = time.time()  # 初始化当前时间戳

        while True:
            now = time.time()

            # 检查用户输入（非阻塞）
            if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
                user_input = sys.stdin.readline().strip().lower()
                if user_input == 's':
                    # 启动Franka控制
                    if not franka_control_enabled:
                        if last_arm_q is not None:
                            franka_control_enabled = True
                            first_frame_for_franka = last_arm_q.copy()
                            transition_in_progress = True
                            print("启动Franka控制...")
                        else:
                            print("错误: 还没有收到Quest数据")
                    else:
                        pass

                elif user_input == 'x':
                    # 停止Franka控制
                    if franka_control_enabled:
                        franka_control_enabled = False
                        transition_in_progress = False
                        # 重置跟随模式标志
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

            # Franka控制循环
            if now - current_ts >= franka_control_period:
                current_ts = now

                # 处理初始过渡
                if transition_in_progress and first_frame_for_franka is not None:
                    # 使用MotionGenerator进行平滑过渡
                    if not hasattr(franka_controller, 'transition_generator'):
                        import sys
                        sys.path.append('/home/rmx/libfranka/pylibfranka/examples')
                        from example_common import MotionGenerator

                        real_q = franka_controller.last_joint_positions
                        delta_deg = np.rad2deg(np.abs(first_frame_for_franka - real_q))
                        max_delta = np.max(delta_deg)
                        speed = 0.05 if max_delta > 60 else (0.1 if max_delta > 30 else 0.15)

                        franka_controller.transition_generator = MotionGenerator(speed, first_frame_for_franka.tolist())

                    # 执行过渡
                    robot_state, duration = franka_controller.active_control.readOnce()
                    cmd = franka_controller.transition_generator(robot_state, duration.to_sec())
                    franka_controller.active_control.writeOnce(cmd)
                    franka_controller.last_joint_positions = np.array(robot_state.q)

                    if cmd.motion_finished:
                        transition_in_progress = False
                        # 初始化速度监控变量（跟随模式开始）
                        franka_controller.following_mode_started = True
                        franka_controller.prev_joint_positions = np.array(robot_state.q)  # 实际位置
                        franka_controller.last_velocity_check_time = time.time()
                        print("开始跟随模式")

                # 正常跟随模式
                elif franka_control_enabled and last_arm_q is not None:
                    success = franka_controller.send_joint_positions(last_arm_q)
                    if success:
                        franka_cmd_counter += 1

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

                        # 保存IK结果
                        if len(arm_q) >= 7:
                            last_arm_q = np.array(arm_q[:7])
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
                    fps_counter += 1

                except socket.error as e:
                    # 网络错误,继续等待
                    pass

    except KeyboardInterrupt:
        print("\n\n正在关闭...")
        franka_controller.stop()
        quest.close()
        pb.disconnect()
        print(f"\n已关闭 | Quest: {packet_counter} pkt | Franka: {franka_cmd_counter} cmd")