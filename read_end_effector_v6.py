#!/usr/bin/env python3

import sys
sys.path.insert(0, '/home/ani/ExDex/data_collection/franka_ws/async_pos_ctrl_cpp/franka_async_controller/build')
import franka_controller as fc
import numpy as np


def main():
    ip = "172.16.0.2"
    if len(sys.argv) > 1:
        ip = sys.argv[1]

    config = fc.ControllerConfig()
    controller = fc.FrankaAsyncController(ip, config)

    if controller.start():
        state = controller.get_robot_state()
        if state:
            # state.O_T_EE 是 4x4 numpy 数组
            position = state.O_T_EE[:3, 3]  # 提取位置
            print(f"{position[0]:.4f} {position[1]:.4f} {position[2]:.4f}")
        controller.stop()


if __name__ == "__main__":
    main()