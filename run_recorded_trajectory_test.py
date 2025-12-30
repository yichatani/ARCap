import argparse
import pickle
import numpy as np
import time

from autolab_core import RigidTransform
from frankapy import FrankaArm, SensorDataMessageType
from frankapy import FrankaConstants as FC
from frankapy.proto_utils import sensor_proto2ros_msg, make_sensor_group_msg
from frankapy.proto import PosePositionSensorMessage, ShouldTerminateSensorMessage
from franka_interface_msgs.msg import SensorDataGroup

from frankapy.utils import convert_array_to_rigid_transform

import rospy
from std_msgs.msg import Float64MultiArray
from franka_interface_msgs.msg import RobotState
import threading
from scipy.spatial.transform import Rotation as R


# class FrankaStreamingPoseController:
#     """
#     通过 frankapy 的 dynamic goto_pose 技能 + 发布 PosePositionSensorMessage，
#     实现“边推理边执行”的在线末端位姿控制。

#     使用方式：
#       ctrl = FrankaStreamingPoseController(fa)
#       ctrl.start(first_pose_rt, duration=60.0)
#       ctrl.send_target(next_pose_rt, dt=0.02)
#       ...
#       ctrl.stop()
#     """
#     def __init__(self, fa: FrankaArm, topic: str = FC.DEFAULT_SENSOR_PUBLISHER_TOPIC, queue_size: int = 10):
#         self.fa = fa
#         self.topic = topic
#         self.pub = rospy.Publisher(self.topic, SensorDataGroup, queue_size=queue_size)

#         self._started = False
#         self._init_time = None
#         self._msg_id = 0
#         fr3_action

#     def start(self,
#               first_pose,
#               duration: float = 30.0,
#               buffer_time: float = 10.0,
#               cartesian_impedances=(600.0, 600.0, 600.0, 50.0, 50.0, 50.0),
#               goto_start_duration: float = 3.0):
#         """
#         first_pose: RigidTransform (建议) 或 4x4/16/7 等可以被 convert_array_to_rigid_transform 解析的数组
#         duration: 这次 dynamic skill 的最长运行时间
#         buffer_time: 防止 skill 提前结束的缓冲
#         goto_start_duration: 启动 skill 前先走到 first_pose 的时间（避免大跳）
#         """
#         if not rospy.core.is_initialized():
#             rospy.init_node("franka_streaming_pose_controller", anonymous=True, disable_signals=True)

#         # 统一 pose 类型
#         if not isinstance(first_pose, RigidTransform):
#             first_pose = convert_array_to_rigid_transform(first_pose)

#         # 先把机器人移动到起点（阻塞）
#         self.fa.goto_pose(first_pose,
#                           duration=goto_start_duration,
#                           cartesian_impedances=list(cartesian_impedances))

#         # 启动 dynamic skill（只需要给一个初始目标，之后靠 send_target 更新）
#         self._init_time = rospy.Time.now().to_time()
#         self._msg_id = 0

#         # 启动后立刻进入在线更新模式
#         self.fa.goto_pose(first_pose,
#                           duration=duration,
#                           dynamic=True,
#                           buffer_time=buffer_time,
#                           cartesian_impedances=list(cartesian_impedances))

#         self._started = True

#     def send_target(self, target_pose, dt: float = None):
#         """
#         target_pose: RigidTransform 或可被 convert_array_to_rigid_transform 解析的数组
#         dt: 你希望这条目标保持/间隔多久（用于 sleep 节奏）。如果你的推理线程自己控制频率，可传 None。
#         """
#         if not self._started:
#             raise RuntimeError("Controller not started. Call start() first.")

#         # 统一 pose 类型
#         if not isinstance(target_pose, RigidTransform):
#             target_pose = convert_array_to_rigid_transform(target_pose)

#         timestamp = rospy.Time.now().to_time() - self._init_time
#         self._msg_id += 1

#         msg = PosePositionSensorMessage(
#             id=self._msg_id,
#             timestamp=timestamp,
#             position=target_pose.translation,
#             quaternion=target_pose.quaternion
#         )

#         ros_msg = make_sensor_group_msg(
#             trajectory_generator_sensor_msg=sensor_proto2ros_msg(msg, SensorDataMessageType.POSE_POSITION)
#         )
#         self.pub.publish(ros_msg)

#         if dt is not None and dt > 0:
#             time.sleep(dt)

#     def stop(self):
#         """发送 terminate 消息终止 skill"""
#         if not self._started:
#             return

#         timestamp = rospy.Time.now().to_time() - self._init_time
#         term_msg = ShouldTerminateSensorMessage(timestamp=timestamp, should_terminate=True)
#         ros_msg = make_sensor_group_msg(
#             termination_handler_sensor_msg=sensor_proto2ros_msg(term_msg, SensorDataMessageType.SHOULD_TERMINATE)
#         )
#         self.pub.publish(ros_msg)

#         self._started = False
#         self._init_time = None


# def test_franka_straming_controller(fa, pose_traj, dt):
#     controller = FrankaStreamingPoseController(
#         fa=fa,
#         topic=FC.DEFAULT_SENSOR_PUBLISHER_TOPIC,
#         queue_size=10)
#     controller.start(pose_traj[0], duration=4.0)

#     for pos_idx in range(len(pose_traj)-1):
#         controller.send_target(pose_traj[pos_idx+1], dt)

#     controller.stop()


from std_msgs.msg import Float64MultiArray


class FrankaStreamingPoseController:
    """
    Online absolute-pose streaming controller for Franka.

    Subscribe:
      /fr3_action (Float64MultiArray, len=8)
        [tx, ty, tz, qx, qy, qz, qw, width]
    """

    def __init__(self,
                 fa: FrankaArm,
                 topic: str = FC.DEFAULT_SENSOR_PUBLISHER_TOPIC,
                 action_topic: str = "/fr3_action",
                 queue_size: int = 10):
        self.fa = fa
        self.topic = topic
        self.pub = rospy.Publisher(self.topic, SensorDataGroup, queue_size=queue_size)

        # ---- action subscriber ----
        self._action_sub = rospy.Subscriber(
            action_topic,
            Float64MultiArray,
            self._action_callback,
            queue_size=1
        )

        self._started = False
        self._init_time = None
        self._msg_id = 0

        # cache last action if needed
        self._last_action = None
        # for test
        self.predefined_traj = None
        self.cnt = 0
                # ---- robot state subscriber ----
        self._robot_state_sub = rospy.Subscriber(
            "/robot_state_publisher_node_1/robot_state",
            RobotState,
            self._robot_state_callback,
            queue_size=1
        )

        # ---- ee pose publisher ----
        self._ee_pose_pub = rospy.Publisher(
            "/fr3_ee_pose",
            Float64MultiArray,
            queue_size=10
        )

        self._latest_robot_state = None
        self._state_lock = threading.Lock()

        # timer (started later)
        self._ee_pub_timer = None

    def _publish_ee_pose_timer_cb(self, event):
        with self._state_lock:
            if self._latest_robot_state is None:
                return
            robot_state = self._latest_robot_state

        # O_T_EE is column-major
        T = np.array(robot_state.O_T_EE).reshape(4, 4).T

        translation = T[:3, 3]
        rotation = T[:3, :3]

        quat_xyzw = R.from_matrix(rotation).as_quat()

        msg = Float64MultiArray()
        msg.data = [
            translation[0],
            translation[1],
            translation[2],
            quat_xyzw[0],
            quat_xyzw[1],  # qx
            quat_xyzw[2],  # qy
            quat_xyzw[3],  # qz
        ]

        self._ee_pose_pub.publish(msg)

        # print("[FEEDBACK] Published EE pose:", msg.data)
        # action_msg = rospy.wait_for_message("/fr3_action",Float64MultiArray,timeout=10)
        # rospy.loginfo(f"received message:{action_msg.data=}")
        # self.send_target(self.predefined_traj[self.cnt],dt=0.01)
        # self.cnt += 1




    # ------------------------------------------------------------------
    # lifecycle
    # ------------------------------------------------------------------
    def start(self,
              first_pose,
              duration: float = 30.0,
              buffer_time: float = 10.0,
              joint_impedances=(600.0, 600.0, 600.0, 50.0, 50.0, 50.0),
              cartesian_impedances=(600.0, 600.0, 600.0, 50.0, 50.0, 50.0),
              goto_start_duration: float = 3.0):

        if not rospy.core.is_initialized():
            rospy.init_node(
                "franka_streaming_pose_controller",
                anonymous=True,
                disable_signals=True
            )
        print(first_pose)
        if not isinstance(first_pose, RigidTransform):
            first_pose = convert_array_to_rigid_transform(first_pose)

        # move to initial pose (blocking)
        self.fa.goto_pose(
            first_pose,
            duration=goto_start_duration,
            cartesian_impedances=list(cartesian_impedances)
        )

        self._init_time = rospy.Time.now().to_time()
        self._msg_id = 0

        # start dynamic skill
        self.fa.goto_pose(
            first_pose,
            duration=duration,
            dynamic=True,
            buffer_time=buffer_time,
            cartesian_impedances=list(cartesian_impedances)
        )

        self._started = True
                # publish ee pose at 50 Hz
        self._ee_pub_timer = rospy.Timer(
            rospy.Duration(0.02),
            self._publish_ee_pose_timer_cb
        )

        self.fa.home_gripper()

        rospy.loginfo("[FrankaStreamingPoseController] Started")

    def stop(self):
        if not self._started:
            return

        timestamp = rospy.Time.now().to_time() - self._init_time
        term_msg = ShouldTerminateSensorMessage(
            timestamp=timestamp,
            should_terminate=True
        )
        ros_msg = make_sensor_group_msg(
            termination_handler_sensor_msg=sensor_proto2ros_msg(
                term_msg, SensorDataMessageType.SHOULD_TERMINATE
            )
        )
        self.pub.publish(ros_msg)

        self._started = False
        self._init_time = None
        if self._ee_pub_timer is not None:
            self._ee_pub_timer.shutdown()
            self._ee_pub_timer = None

        rospy.loginfo("[FrankaStreamingPoseController] Stopped")


    def send_target(self, target_pose, dt: float = 0.01):
        """
        target_pose: RigidTransform 或可被 convert_array_to_rigid_transform 解析的数组
        dt: 你希望这条目标保持/间隔多久（用于 sleep 节奏）。如果你的推理线程自己控制频率，可传 None。
        """
        if not self._started:
            raise RuntimeError("Controller not started. Call start() first.")

        # 统一 pose 类型
        if not isinstance(target_pose, RigidTransform):
            target_pose = convert_array_to_rigid_transform(target_pose)

        timestamp = rospy.Time.now().to_time() - self._init_time
        self._msg_id += 1

        msg = PosePositionSensorMessage(
            id=self._msg_id,
            timestamp=timestamp,
            position=target_pose.translation,
            quaternion=target_pose.quaternion
        )

        ros_msg = make_sensor_group_msg(
            trajectory_generator_sensor_msg=sensor_proto2ros_msg(msg, SensorDataMessageType.POSE_POSITION)
        )
        self.pub.publish(ros_msg)

        if dt is not None and dt > 0:
            time.sleep(dt)


    # ------------------------------------------------------------------
    # ROS callback
    # ------------------------------------------------------------------
    def _action_callback(self, msg: Float64MultiArray):
        """
        msg.data = [tx, ty, tz, qx, qy, qz, qw, width]
        """
        rospy.loginfo(f"receiced command from inference pc {msg.data}")
        if not self._started:
            return

        if len(msg.data) != 8:
            rospy.logwarn(
                f"[FrankaStreamingPoseController] "
                f"Expected action dim 8, got {len(msg.data)}"
            )
            return

        self._last_action = np.array(msg.data, dtype=np.float64)

        # pose_rt = self._action_to_rigid_transform(self._last_action)
        pose_rt = self._action_to_transform_matrix(self._last_action)

        # online execution
        # pose_tar = self.predefined_traj[self.cnt]
        # pose_tar[2] += 0.02
        # self.send_target(self.predefined_traj[self.cnt],dt=0.01)
        # rospy.loginfo(f"execute: {self.predefined_traj[self.cnt]} cnt:{self.cnt}")

        self.send_target(pose_rt, dt=0.1)
        rospy.loginfo(f"execute: {pose_rt} cnt:{self.cnt}")

        # gripper control
        gripper_width = self._last_action[7]
        print("received gripper width:", gripper_width)
        if gripper_width >= 0.08:
            gripper_width = 0.08
        if gripper_width <= 0.0:
            gripper_width = 0.0
        self.fa.goto_gripper(gripper_width, speed=0.1)
        # save pose_rt to piclkle file
        
        # with open("./pose_rt.pkl", "wb") as f:
        #     pickle.dump(pose_rt, f)

        self.cnt += 1

    def _robot_state_callback(self, msg: RobotState):
        with self._state_lock:
            self._latest_robot_state = msg


    # ------------------------------------------------------------------
    # utils
    # ------------------------------------------------------------------
    @staticmethod
    def _action_to_rigid_transform(action: np.ndarray) -> RigidTransform:
        """
        Convert [tx, ty, tz, qx, qy, qz, qw, width] to RigidTransform
        """
        tx, ty, tz, qx, qy, qz, qw, _ = action

        rot_matrix = R.from_quat([qx,qy,qz,qw]).as_matrix()

        return RigidTransform(
            translation=[tx, ty, tz],
            rotation=rot_matrix,
            from_frame="franka_hand",
            to_frame="base"
        )
    
    @staticmethod
    def _action_to_transform_matrix(action: np.ndarray) -> np.ndarray:
        """
        Convert action [tx, ty, tz, qx, qy, qz, qw, width] to 4x4 transform matrix.
        Quaternion uses xyzw order.

        Returns:
        T: np.ndarray, shape (4, 4)
            [[R(3x3), t(3x1)],
            [0 0 0,   1   ]]
        """
        a = np.asarray(action, dtype=np.float64).reshape(-1)
        if a.size < 7:
            raise ValueError(f"action must have at least 7 elements (tx,ty,tz,qx,qy,qz,qw), got {a.size}")

        tx, ty, tz, qx, qy, qz, qw = a[:7]

        rot_matrix = R.from_quat([qx, qy, qz, qw]).as_matrix()

        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = rot_matrix
        T[:3,  3] = [tx, ty, tz]
        if T.shape == (4, 4):
            T = T.reshape(-1)
        return T
    

    # for test
    def set_predefined_traj(self,traj):
        self.predefined_traj = traj



def test_franka_straming_controller(fa, pose_traj, dt):
    controller = FrankaStreamingPoseController(
        fa=fa,
        topic=FC.DEFAULT_SENSOR_PUBLISHER_TOPIC,
        queue_size=10)
    controller.set_predefined_traj(pose_traj)
    initial_pose = controller._action_to_transform_matrix(
        np.array([0.62092876, 0.07294732, 0.32858068,   
                  0.99969971, -0.00416662, -0.01742757,  0.01671541,
                  0.08])
    )

    controller.start(initial_pose, duration=400.0)
    # controller.start(pose_traj[0], duration=400.0,buffer_time=400.0)

# execute data from inference
    # rospy.loginfo("Franka streaming controller running...")
    rospy.spin()   #

# execute predefined traj and record feedback trajectory
    # robot_pose_list = []

    # for pos_idx in range(len(pose_traj) - 1):
    #     controller.send_target(pose_traj[pos_idx + 1], dt)

    #     robot_state = controller.fa.get_robot_state()
    #     robot_pose = robot_state['pose']

    #     robot_pose_list.append({
    #         "translation": np.array(robot_pose.translation),
    #         "quaternion": np.array(robot_pose.quaternion),
    #     })

    # with open("./robot_pose_feedback.pkl", "wb") as f:
    #     pickle.dump(robot_pose_list, f)

        

    controller.stop()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--trajectory_pickle', '-t', type=str, required=True,
                        help='Path to trajectory (in pickle format) to replay.')
    parser.add_argument('--open_gripper', '-o', action='store_true')
    args = parser.parse_args()

    print('Starting robot')
    fa = FrankaArm()
    # fa.reset_joints()

    rospy.loginfo('Loading Trajectory')

    with open(args.trajectory_pickle, 'rb') as pkl_f:
        skill_data = pickle.load(pkl_f)
    
    assert skill_data[0]['skill_description'] == 'GuideMode', \
        "Trajectory not collected in guide mode"
    skill_state_dict = skill_data[0]['skill_state_dict']

    T = float(skill_state_dict['time_since_skill_started'][-1])
    dt = 0.1
    ts = np.arange(0, T, dt)

    pose_traj = skill_state_dict['O_T_EE']

    print(pose_traj[0])
    # Goto the first position in the trajectory.
    print(convert_array_to_rigid_transform(pose_traj[0]).matrix)

    test_franka_straming_controller(fa, pose_traj, dt)


    rospy.loginfo('Done')

