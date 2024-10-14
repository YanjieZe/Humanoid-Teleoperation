"""
Author: WANG Wenhao
Date: 2024-06-29
Version: 1.0.0
copyright (c) 2024 All Rights Reserved
"""

import time
from typing import Literal
import numpy as np
import scipy.optimize as opt
import quaternionic
from scipy.spatial.transform import Rotation

# os.environ["PYTHONPATH"] = (
#     os.path.dirname(__file__) + "/arm_control/:" + os.environ["PYTHONPATH"]
# )
from arm_control.FK.fkpytest import GR1_ArmFk
from utils import *


class ArmRetarget:
    def __init__(self, platform: Literal["AVP", "PICO"] = "AVP"):
        # 前七个是left，后七个是right
        self.fk = GR1_ArmFk()
        self.platform = platform

        # 左右第一四个关节,向前弯是负
        # 第一关节,向后最多30度=0.6,向前150度
        # 第二关节, 前后总共180
        # 第三关节, 前后各90度略多=1.57略多
        # 第四关节, 后0度，前120度
        # 第五关节, 前后各90度略多=1.57略多
        self.lower_bound = [
            -2.79, -0.30, -2.00, -2.05, -2.00,
            -2.79, -2.90, -2.00, -2.05, -2.00,
        ]
        self.upper_bound = [
            0.60, 2.90, 2.00, 0.0, 2.00,
            0.60, 0.30, 2.00, 0.0, 2.00
        ]
        self.bounds = [(l, u)
                       for l, u in zip(self.lower_bound, self.upper_bound)]

        self.last_optimized_q_10d = np.zeros(10)
        self.last_optimized_q_10d[3] = -np.pi / 2
        self.last_optimized_q_10d[3 + 5] = -np.pi / 2

        self.T_robot_vr = np.array(
            [[0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 1, -1.2568], [0, 0, 0, 1]]
        )

        self.T_left_arm = np.array(
            [[0, 0, 1, 0], [0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 0, 1]]
        )
        self.T_right_arm = np.array(
            [[0, 0, -1, 0], [0, -1, 0, 0], [-1, 0, 0, 0], [0, 0, 0, 1]]
        )

        # cache
        self.last_valid_r = None

    def _fi(self, n, s, c, r, x):
        return (-1) ** n * np.exp(-((x - s) ** 2) / (2 * c**2)) + r * (x - s) ** 4

    def _position_cost(self, target_pos, actual_pos):
        n, s, c, r = 1, 0, 0.2, 5.0
        xl = np.linalg.norm(target_pos[0] - actual_pos[0])
        xr = np.linalg.norm(target_pos[1] - actual_pos[1])

        return self._fi(n, s, c, r, xl) + self._fi(n, s, c, r, xr)

    def _orientation_cost(self, target_quat, actual_quat):
        n, s, c, r = 1, 0, 0.2, 5.0
        q1 = quaternionic.array(target_quat[0])
        q2 = quaternionic.array(actual_quat[0])
        # xl = np.min(np.log(q1.inverse * q2), np.log(q1.inverse * -q2))
        xl = quaternionic.distance.rotation.intrinsic(q1, q2)
        q1 = quaternionic.array(target_quat[1])
        q2 = quaternionic.array(actual_quat[1])
        # xr = np.min(np.log(q1.inverse * q2), np.log(q1.inverse * -q2))
        xr = quaternionic.distance.rotation.intrinsic(q1, q2)

        return self._fi(n, s, c, r, xl) + self._fi(n, s, c, r, xr)

    def _joint_velocity_cost(self, q0_10d, q_10d):
        n, s, c, r = 1, 0, 0.2, 5.0
        xll = q0_10d[0:5] - q_10d[0:5]
        xll[0:3] = xll[0:3] * 2
        xl = np.linalg.norm(xll)
        xrr = q0_10d[5:10] - q_10d[5:10]
        xrr[0:3] = xrr[0:3] * 2
        xr = np.linalg.norm(xrr)
        return self._fi(n, s, c, r, xl) + self._fi(n, s, c, r, xr)

    
    def cur_eef(self, q_14d):
        r = self.fk.calculateFK(q_14d)
        actual_pos = np.vstack((r.left_fk.pos[:], r.right_fk.pos[:]))
        actual_quat = np.vstack(
            (r.left_fk_elb.quat[:], r.right_fk_elb.quat[:]))
        return actual_pos, actual_quat
        
    def _objective_function(self, q_10d):
        # 生成的当前的
        q0_14d = np.zeros(14)
        q0_14d[0:5] = q_10d[0:5]
        q0_14d[7:12] = q_10d[5:10]
        r = self.fk.calculateFK(q0_14d)
        actual_pos = np.vstack((r.left_fk.pos[:], r.right_fk.pos[:]))
        actual_quat = np.vstack(
            (r.left_fk_elb.quat[:], r.right_fk_elb.quat[:]))

        # self里面取得target和现在的
        pos_cost = self._position_cost(self.target_pos, actual_pos)
        ori_cost = self._orientation_cost(self.target_quat, actual_quat)
        vel_cost = self._joint_velocity_cost(self.last_optimized_q_10d, q_10d)

        return 60 * pos_cost + 6 * ori_cost + 2 * vel_cost

    def _solve_wrist_angles(self, T_wrist, T_forearmWrist, chirality=None):
        # self.T_wrist, self.T_forearmWrist 算最后两个关节的角度

        # wrist y 和 arm y 的夹角 是招手自由度的角度，朝着arm x的方向为向内招手
        wrist_y = T_wrist[:3, 1]
        arm_y = T_forearmWrist[:3, 1]
        angle6 = calculate_angle_between_vectors(wrist_y, arm_y)
        arm_x = T_forearmWrist[:3, 0]
        if np.dot(arm_x, wrist_y) > 0:
            # 向内招手 右手+
            if chirality == "left":
                q6 = -angle6 - np.pi / 12
            elif chirality == "right":
                q6 = angle6 + np.pi / 12
        else:
            # 向外招手 左手+
            if chirality == "left":
                q6 = angle6 - np.pi / 12
            elif chirality == "right":
                q6 = -angle6 + np.pi / 12

        # wrist z 和 -arm z 的夹角 是摆手自由度的角度，朝着arm x的方向为向外摆手
        wrist_z = T_wrist[:3, 2]
        arm_z = T_forearmWrist[:3, 2]
        angle7 = calculate_angle_between_vectors(wrist_z, -arm_z)
        arm_x = T_forearmWrist[:3, 0]
        if np.dot(arm_x, wrist_z) > 0:
            # 向外摆手 左手右手+
            if chirality == "left":
                q7 = angle7
            elif chirality == "right":
                q7 = angle7
        else:
            # 向内
            if chirality == "left":
                q7 = -angle7
            elif chirality == "right":
                q7 = -angle7

        # in robot
        # lefthand up 5 negative
        # lefthand inner 6 negative
        # righthand up 12 negative
        # righthand out 13 negative
        q7 = np.clip(q7, -np.pi * 35 / 180, np.pi * 35 / 180)
        q6 = np.clip(q6, -np.pi * 50 / 180, np.pi * 55 / 180)
        # print("++++++++++",q7 / np.pi * 180, q6 / np.pi * 180 )

        return q7, q6  # reverse 6 and 7 because rcs interface error

    def _solve_uparm_angles(self, q0, target_pos, target_quat):
        self.target_pos = target_pos
        self.target_quat = target_quat

        # 应该输入10维，然后在objecive function里补成14维
        q0_10d = np.concatenate((q0[0:5], q0[7:12]))
        result = opt.minimize(
            self._objective_function,
            q0_10d,
            method="SLSQP",
            tol=1e-3,
            bounds=self.bounds,
            options={"maxiter": 100},
        )

        if result.success:
            optimized_q = result.x
            # print("optimized_q: ", optimized_q)
            # print("Optimization successful! Joint angles: ", optimized_q)

            self.last_optimized_q_10d = optimized_q
            return optimized_q
        else:
            raise Exception(
                "Optimization failed! After "
                + result.nit
                + " iterations."
                + result.message
            )

    def solve_arm_angles(self, q0, r, pitch=0.0, use_wrist=False):

        if self.last_valid_r is None:
            self.last_valid_r = r

        is_left_missing = r["left_fingers"][1][0, 0] == 0 and r["left_fingers"][1][0,
                                                                                   1] == 0 and r["left_fingers"][1][0, 2] == 0
        is_right_missing = r["right_fingers"][1][0, 0] == 0 and r["right_fingers"][1][0,
                                                                                      1] == 0 and r["right_fingers"][1][0, 2] == 0
        if is_left_missing or is_right_missing:
            r = self.last_valid_r
        else:
            self.last_valid_r = r

        if self.platform == "AVP":
            left_wrist = r["left_wrist"][0]
            right_wrist = r["right_wrist"][0]
            left_forearmWrist = left_wrist @ r["left_fingers"][-2]
            right_forearmWrist = right_wrist @ r["right_fingers"][-2]
        elif self.platform == "PICO":
            left_wrist = r["left_wrist"]
            right_wrist = r["right_wrist"]
            left_forearmWrist = r["left_forearmWrist"]
            right_forearmWrist = r["right_forearmWrist"]
        else:
            raise ValueError("Invalid platform")

        q_2d_l = self._solve_wrist_angles(
            left_wrist, left_forearmWrist, chirality="left"
        )
        q_2d_r = self._solve_wrist_angles(
            right_wrist, right_forearmWrist, chirality="right"
        )

        # rotate y axis by pitch
        pitch_rotate = np.array([[np.cos(pitch), 0, np.sin(pitch), 0],
             [0, 1, 0, 0],
             [-np.sin(pitch), 0, np.cos(pitch), 0],
             [0, 0, 0, 1]])

        left_forearmWrist = pitch_rotate.T @ self.T_robot_vr @ left_forearmWrist @ self.T_left_arm
        right_forearmWrist = pitch_rotate.T @ self.T_robot_vr @ right_forearmWrist @ self.T_right_arm
        pos = np.vstack((left_forearmWrist[:3, 3], right_forearmWrist[:3, 3]))
        quat1 = quaternionic.array.from_rotation_matrix(left_forearmWrist)
        quat2 = quaternionic.array.from_rotation_matrix(right_forearmWrist)
        quat = np.vstack((quat1, quat2))

        # print("pos:", pos)
        # print("quat:", quat)
        # print("-----")
        
        q_10d = self._solve_uparm_angles(q0, pos, quat)
        
        
        if use_wrist:
            # the last two dim of arm is wrist
            q_14d = np.concatenate((q_10d[0:5], q_2d_l, q_10d[5:10], q_2d_r))
        else:
            q_14d = np.concatenate((q_10d[0:5], [0,0], q_10d[5:10], [0,0]))
            
        return q_14d, pos, quat
    
    def ik(self, q0, pos, quat):
        q_10d = self._solve_uparm_angles(q0, pos, quat)
        q_14d = np.concatenate((q_10d[0:5], [0,0], q_10d[5:10], [0,0]))
        return q_14d


class HandRetarget:
    def __init__(self):
        # gripper parameters
        # 完全闭合的时候pinch_distance大概是8mm=0.008m
        self.pinching_threshold = 0.02
        self.gripper_limits = (0.0, 90.0)

        # fingers parameters
        self.four_fingers_limits = (40.0, 170.0)
        self.thumb_bending_limits = (15.0, 30.0)
        self.thumb_rotation_limits = (80.0, 150.0)

        # cache
        self.last_valid_left = None
        self.last_valid_right = None
        self.last_valid_left_pinch = None
        self.last_valid_right_pinch = None

    def _get_point_angle(self, finger_frames, origin, point1, point2):
        vector1 = finger_frames[point1, :3, 3] - \
            finger_frames[origin, :3, 3]
        vector2 = finger_frames[point2, :3, 3] - \
            finger_frames[origin, :3, 3]
        angle = calculate_angle_between_vectors(
            vector1, vector2)/np.pi*180
        return angle

    def _solve_four_fingers(self, finger_frames):
        # (little, ring, middle, index)
        four_angles = np.zeros(4)
        for i in range(4):
            # 6 to 9, 6 to 5 is index finger
            # plus 5 per finger
            angle = self._get_point_angle(
                finger_frames, 6+5*i, 5+5*i, 9+5*i)
            four_angles[3-i] = angle  # 倒着排

        # 这里两个值应该是人手打开和握拳的角度，映射到0到1000之间
        # 机械手的角度在19到176.7之间，但这个值和参数无关
        four_angles = np.clip(four_angles, *self.four_fingers_limits)
        four_angles = (four_angles - self.four_fingers_limits[0]) / (
            self.four_fingers_limits[1] - self.four_fingers_limits[0]) * 1000
        return four_angles

    def _solve_thumb(self, finger_frames, pinch_distance):
        # 在大多数情况下都是直接映射两个自由度
        bending_angle = self._get_point_angle(
            finger_frames, 1, 4, 6)
        rotation_angle = self._get_point_angle(
            finger_frames, 6, 3, 21)

        # bending
        # 人手角度在什么和什么之间，映射到0到1000之间，
        # 机械手值 -13.0deg 到 53.6deg
        bending_angle = np.clip(bending_angle, *self.thumb_bending_limits)
        bending_angle = (bending_angle - self.thumb_bending_limits[0]) / (
            self.thumb_bending_limits[1] - self.thumb_bending_limits[0]) * 1000

        # rotation
        # 机械手值 90deg 到 165deg
        rotation_angle = np.clip(rotation_angle, *self.thumb_rotation_limits)
        rotation_angle = (rotation_angle - self.thumb_rotation_limits[0]) / (
            self.thumb_rotation_limits[1] - self.thumb_rotation_limits[0]) * 1000

        # 在pinch模式下例外
        # distance 0.01 到 0.04 之间，线性变换。
        # bending_angle 400 到 1000 之间
        # pinch_distance = 0.0
        is_pinch_mode = pinch_distance < 0.04
        if is_pinch_mode:
            rotation_angle = 150
            pinch_distance = np.clip(pinch_distance, 0.01, 0.04)
            bending_angle = 800 * (pinch_distance - 0.01) / (0.04 - 0.01) + 400

        return bending_angle, rotation_angle

    def solve_fingers_angles(self, r):
        if self.last_valid_left is None:
            self.last_valid_left = r["left_fingers"]
            self.last_valid_left_pinch = r["left_pinch_distance"]
        if self.last_valid_right is None:
            self.last_valid_right = r["right_fingers"]
            self.last_valid_right_pinch = r["right_pinch_distance"]

        finger_frames = r["left_fingers"]
        pinch_distance = r["left_pinch_distance"]

        if finger_frames[1][0, 0] == 0 and finger_frames[1][0, 1] == 0 and finger_frames[1][0, 2] == 0:
            # 说明是空的
            finger_frames = self.last_valid_left
            pinch_distance = self.last_valid_left_pinch
        else:
            self.last_valid_left = finger_frames
            self.last_valid_left_pinch = pinch_distance

        left_four_fingers_angles = self._solve_four_fingers(finger_frames)
        left_thumb_angles = self._solve_thumb(
            finger_frames, pinch_distance)
        left_angles = np.concatenate(
            (left_four_fingers_angles, left_thumb_angles))

        finger_frames = r["right_fingers"]
        pinch_distance = r["right_pinch_distance"]

        if finger_frames[1][0, 0] == 0 and finger_frames[1][0, 1] == 0 and finger_frames[1][0, 2] == 0:
            # 说明是空的
            finger_frames = self.last_valid_right
            pinch_distance = self.last_valid_right_pinch
        else:
            self.last_valid_right = finger_frames
            self.last_valid_right_pinch = pinch_distance

        right_four_fingers_angles = self._solve_four_fingers(finger_frames)
        right_thumb_angles = self._solve_thumb(
            finger_frames, pinch_distance)
        right_angles = np.concatenate(
            (right_four_fingers_angles, right_thumb_angles))

        return left_angles, right_angles

    def solve_gripper_angles(self, r):
        is_left_pinching = r["left_pinch_distance"] < self.pinching_threshold
        is_right_pinching = r["right_pinch_distance"] < self.pinching_threshold

        left_gripper_angle = self._get_point_angle(
            r["left_fingers"], 1, 4, 9)
        right_gripper_angle = self._get_point_angle(
            r["right_fingers"], 1, 4, 9)

        left_gripper_angle = np.clip(
            left_gripper_angle, *self.gripper_limits)
        right_gripper_angle = np.clip(
            right_gripper_angle, *self.gripper_limits)

        return (is_left_pinching, left_gripper_angle), (is_right_pinching, right_gripper_angle)


class UpperBodyRetarget:
    def __init__(self) -> None:
        self.height = 1.70
        self.waist_height_ratio = 0.530
        self.waist_height = self.height * self.waist_height_ratio

    def solve_upper_body_angles(self, head):

        head_height = head[2, 3]
        stoop_distance = head[1, 3]

        R = Rotation.from_matrix(head[:3, :3])
        euler = R.as_euler('xyz')
        # 小写代表intrinsic rotation，轴和刚体绑定，一般都是intrinsic
        # 这个顺序和机器人硬件上的顺序应该一样

        pitch, roll, yaw = euler[0], euler[1], euler[2]
        # print(roll, -pitch, yaw)

        total_pitch = -pitch
        head_yaw = yaw

        waist_pitch = np.arctan2(
            stoop_distance, (head_height - self.waist_height))
        head_pitch = total_pitch - waist_pitch

        # print(waist_pitch, head_pitch, head_yaw)

        q_upper_body = [0.0, waist_pitch, 0.0, head_pitch, 0.0, head_yaw]

        q_upper_body = np.clip(q_upper_body, -np.pi/6, np.pi/6)

        return q_upper_body
