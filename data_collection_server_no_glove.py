#!/usr/bin/env python3
"""
ARCap极简测试版 - 只需Quest 3头显

✅ 可以测试:
   - 手臂跟踪 (手腕位置+朝向)
   - IK求解 (实时计算关节角度)
   - AR反馈 (黄色警告、蓝色碰撞、震动)
   - 虚拟机器人可视化

❌ 不需要:
   - Rokoko手套
   - RealSense相机
   - 任何其他硬件

用法:
    # 右臂Leap Hand
    python data_collection_server_no_glove.py

    # 左臂Gripper
    python data_collection_server_no_glove.py --handedness left
"""
import os
import socket
import time
from argparse import ArgumentParser
import numpy as np
from scipy.spatial.transform import Rotation
import pybullet as pb
from rigidbodySento import create_primitive_shape
from ip_config import *
from quest_robot_module import QuestRightArmLeapModule, QuestLeftArmGripperModule

def generate_default_hand_positions(handedness="right"):
    """
    生成默认的手指位置(相对于手腕坐标系)
    这些位置对应手指自然伸展的姿态

    返回: 4个指尖位置 [拇指, 食指, 中指, 无名指]
    """
    if handedness == "right":
        # Leap Hand右手默认指尖位置(单位:米)
        # 这些值来自quest_robot_module.py:89行的right_hand_dest
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
    parser = ArgumentParser(description="ARCap极简测试 - 只需Quest 3头显")
    parser.add_argument("--frequency", type=int, default=30,
                        help="IK求解频率(Hz),默认30")
    parser.add_argument("--handedness", type=str, default="right",
                        choices=["right", "left"],
                        help="选择机器人类型: right=Leap Hand, left=Gripper")
    args = parser.parse_args()

    # 注意: 此版本不需要摄像头和手套!

    # 确定机器人类型
    handedness = args.handedness
    robot_type = "Gripper(平行夹爪)" if handedness == "left" else "Leap Hand(灵巧手)"

    # 创建数据文件夹
    if not os.path.isdir("data"):
        os.mkdir("data")

    # 初始化PyBullet(用于IK求解和碰撞检测)
    print("正在初始化PyBullet...")
    c = pb.connect(pb.DIRECT)

    # 创建4个可视化球体(用于显示指尖位置)
    vis_sp = []
    colors = [[1,0,0,1], [0,1,0,1], [0,0,1,1], [1,1,0,1]]  # 红绿蓝黄
    for i in range(4):
        vis_sp.append(create_primitive_shape(pb, 0.1, pb.GEOM_SPHERE, [0.02], color=colors[i]))

    # 初始化Quest机器人模块
    print(f"正在初始化Quest机器人模块 (handedness={handedness})...")
    if handedness == "right":
        quest = QuestRightArmLeapModule(VR_HOST, LOCAL_HOST, POSE_CMD_PORT, IK_RESULT_PORT, vis_sp=None)
        print("✓ 右臂Leap Hand模块已加载")
    else:
        quest = QuestLeftArmGripperModule(VR_HOST, LOCAL_HOST, POSE_CMD_PORT, IK_RESULT_PORT, vis_sp=vis_sp)
        print("✓ 左臂Gripper模块已加载")

    # 生成默认手指位置(无手套模式)
    default_finger_positions = generate_default_hand_positions(handedness)
    print(f"✓ 使用默认手指姿态(无手套模式)")
    print(f"  指尖位置(相对手腕): \n{default_finger_positions}")

    start_time = time.time()
    fps_counter = 0
    packet_counter = 0
    current_ts = time.time()

    print("\n" + "="*70)
    print("  ARCap 极简测试模式 - 只需Quest 3头显")
    print("="*70)
    print(f"\n🤖 机器人类型: {robot_type}")
    print(f"📡 Quest IP: {VR_HOST}")
    print(f"💻 本机 IP: {LOCAL_HOST}")
    print(f"🔄 更新频率: {args.frequency} Hz")
    print("\n⚠️  极简模式(无需手套和相机):")
    print("   ✅ 手臂会完美跟踪你的手腕")
    print("   ✅ AR反馈正常工作(黄色/蓝色警告)")
    print("   ⚠️ 手指保持默认姿态(因为没有手套)")
    print("\n" + "="*70)
    print("\n📋 Quest 3操作步骤:")
    print("  1️⃣  启动ARCap应用")
    print(f"  2️⃣  输入此电脑IP: {LOCAL_HOST}")
    print("  3️⃣  选择机器人类型:")
    if handedness == "right":
        print("       👉 按Y键选择Leap Hand")
    else:
        print("       👉 按X键选择Gripper")
    print("  4️⃣  放置虚拟机器人基座(用摇杆)")
    print("  5️⃣  按X键保存WorldFrame")
    print("  6️⃣  移动你的手!")
    print("\n🎯 测试项目:")
    print("   ✅ 慢速移动 → 虚拟机器人跟随")
    print("   ✅ 快速移动 → 黄色警告")
    print("   ✅ 靠近障碍 → 蓝色+震动")
    print("   ✅ 按A录制  → 红色边框")
    print("\n按Ctrl+C停止服务器")
    print("="*70 + "\n")

    try:
        while True:
            now = time.time()

            # 频率控制
            if now - current_ts < 1 / args.frequency:
                continue
            else:
                current_ts = now

            try:
                # 接收Quest发来的手腕姿态
                wrist, head_pose = quest.receive()

                if wrist is not None:
                    # 解析手腕和头部姿态
                    wrist_pos = wrist[0]
                    wrist_orn = Rotation.from_quat(wrist[1])
                    head_pos = head_pose[0]
                    head_orn = Rotation.from_quat(head_pose[1])

                    # 使用默认手指位置(无手套数据)
                    # 将相对位置转换到世界坐标系
                    hand_tip_pose = wrist_orn.apply(default_finger_positions) + wrist_pos

                    # Leap Hand需要调整指尖顺序 (拇指移到最后)
                    if handedness == "right":
                        hand_tip_pose[[0,1,2,3]] = hand_tip_pose[[1,2,3,0]]

                    # 求解IK(机器人关节角度)
                    arm_q, hand_q, wrist_pos_out, wrist_orn_out = quest.solve_system_world(
                        wrist_pos, wrist_orn, hand_tip_pose
                    )

                    # 发送IK结果回Quest(用于显示虚拟机器人)
                    action = quest.send_ik_result(arm_q, hand_q)

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

                    # 统计FPS
                    fps_counter += 1
                    packet_counter += 1
                    packet_time = time.time()

                    if (packet_time - start_time) > 1.0:
                        print(f"✓ 接收到 {fps_counter} 个数据包/秒 | 总数: {packet_counter}", end="\r")
                        start_time += 1.0
                        fps_counter = 0

            except socket.error as e:
                # 网络错误,继续等待
                pass

    except KeyboardInterrupt:
        print("\n\n正在关闭服务器...")
        quest.close()
        print("✓ 服务器已关闭")
