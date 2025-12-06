#!/usr/bin/env python3

import argparse
import numpy as np

from pylibfranka import Robot


def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="读���Franka机器人末端坐标")
    parser.add_argument("--ip", type=str, default="localhost", help="机器人IP地址")
    args = parser.parse_args()

    try:
        # 连接机器人
        print(f"正在连接机器人 {args.ip}...")
        robot = Robot(args.ip)

        # 读取机器人状态
        print("读取机器人状态...")
        state = robot.read_once()

        # 获取末端位置（从O_T_EE矩阵）
        # O_T_EE是16个元素的列表，按列优先存储
        # 位置在索引12, 13, 14（第4列的前3个元素）
        position = [state.O_T_EE[12], state.O_T_EE[13], state.O_T_EE[14]]

        print("\n末端执行器位置（相对于基座）：")
        print(f"X = {position[0]:.4f} m")
        print(f"Y = {position[1]:.4f} m")
        print(f"Z = {position[2]:.4f} m")

        # 打印完整的O_T_EE矩阵（4x4变换矩阵）
        print("\n完整的O_T_EE变换矩阵（4x4）：")
        O_T_EE_matrix = np.array(state.O_T_EE).reshape(4, 4)
        print(O_T_EE_matrix)

        # 打印当前关节角度
        print("\n当前关节角度（弧度）：")
        print(np.round(state.q, 4).tolist())
        print("\n当前关节角度（度）：")
        print(np.round(np.rad2deg(state.q), 2).tolist())

    except Exception as e:
        print(f"错误：{e}")
        return -1

    return 0


if __name__ == "__main__":
    exit(main())