"""
Author: WANG Wenhao
Date: 2024-06-29
Version: 1.0.0
copyright (c) 2024 All Rights Reserved
"""

from scipy.spatial.transform import Rotation
import numpy as np
import time


class SnapMonitorOneSide:
    def __init__(self, chirality=None):
        self.hand_side = chirality

        self.snap_detected = False
        self.last_snap_time = 0  # 记录上次打响指的时间
        self.prev_distance13 = None
        self.prev_distance03 = None
        self.prev_movement1 = None
        self.last_update_time = 0  # 记录上次更新的时间

        # 设置阈值
        self.cooldown = 1.1  # 冷却时间，单位秒
        # 0 1 2 3 4 5 分别指的是wrist，拇指，食指，中指，无名指，小指
        # 13就是指拇指和中指的相对速度
        self.threshold_speed13 = 0.20  # 根据实际需要调整
        self.threshold_speed03 = 0.20  # 根据实际需要调整
        self.threshold_speed1 = 0.30  # 根据实际需要调整

    def update(self, fingers):
        current_time = time.time()
        time_interval = current_time - self.last_update_time
        if time_interval < 0.1:
            self.snap_detected = False
            return

        self.last_update_time = current_time

        thumbTipPos = fingers[4, :3, 3]
        indexTipPos = fingers[9, :3, 3]
        middleTipPos = fingers[14, :3, 3]
        distance13 = np.linalg.norm(thumbTipPos - middleTipPos)
        distance03 = np.linalg.norm(middleTipPos)
        if self.hand_side == 'left':
            movement1 = thumbTipPos[0] - thumbTipPos[1] - thumbTipPos[2]
        if self.hand_side == 'right':
            movement1 = -thumbTipPos[0] + thumbTipPos[1] + thumbTipPos[2]

        # 初始化 previous 值
        if self.prev_distance13 is None:
            self.prev_distance13 = distance13
            self.prev_distance03 = distance03
            self.prev_movement1 = movement1
            return

        # 计算速度
        speed13 = (distance13 - self.prev_distance13)/time_interval
        speed03 = (self.prev_distance03 - distance03)/time_interval
        speed1 = (movement1 - self.prev_movement1)/time_interval
        distance23 = np.linalg.norm(indexTipPos - middleTipPos)

        # 判断是否打响指
        # distance23 > 0.02
        if (speed13 > self.threshold_speed13 and
            speed03 > self.threshold_speed03 and
                speed1 > self.threshold_speed1):
            if current_time - self.last_snap_time > self.cooldown:
                self.snap_detected = True
                self.last_snap_time = current_time
        else:
            self.snap_detected = False

        # 更新 previous 值
        self.prev_distance13 = distance13
        self.prev_distance03 = distance03
        self.prev_movement1 = movement1


# call: snap_monitor = SnapMonitor()
# flag: snap_monitor.left.snap_detected or snap_monitor.right.snap_detected
class SnapMonitor:
    def __init__(self):
        self.left = SnapMonitorOneSide(chirality='left')
        self.right = SnapMonitorOneSide(chirality='right')

    def update(self, r):
        self.left.update(r['left_fingers'])
        self.right.update(r['right_fingers'])


# undebug
class SteeringMonitor:
    def __init__(self):
        pass
    # 可能需要非线性映射、根据人体型调整

    def euler2control(self, roll, pitch, yaw):
        max_velocity_x = 1.0
        max_velocity_y = 1.0

        # Normalize the angles (assuming roll and pitch are in degrees)
        norm_roll = roll / 90.0  # Normalize to range [-1, 1]
        norm_pitch = pitch / 90.0  # Normalize to range [-1, 1]
        norm_yaw = yaw / 90.0  # Normalize to range [-1, 1]

        # Calculate velocity components
        vx = max_velocity_x * - norm_pitch  # Forward/backward movement
        vy = max_velocity_y * - norm_roll  # Side-to-side movement

        heading_angle = norm_yaw

        return (vx, vy), heading_angle

    def control_from_wrist(self, wrist, chirality=None):
        R = wrist[:3, :3]

        if chirality == 'left':
            R_default = np.array([
                [0, 0, -1],
                [1, 0, 0],
                [0, -1, 0]
            ])
        elif chirality == 'right':
            R_default = np.array([
                [0, 0, -1],
                [-1, 0, 0],
                [0, 1, 0]
            ])

        R = R @ R_default.T
        R = Rotation.from_matrix(R)
        roll, pitch, yaw = R.as_euler('yxz', degrees=True)

        v, heading = self.euler2control(roll, pitch, yaw)

        return v, heading
