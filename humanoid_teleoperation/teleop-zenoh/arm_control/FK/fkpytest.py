import ctypes
from ctypes import POINTER, Structure, c_double, c_void_p
import os

class PositionQuaternion(Structure):
    _fields_ = [
        ("pos", c_double * 3),    # x, y, z
        ("quat", c_double * 4)    # w, x, y, z
    ]


class FKResult(Structure):
    _fields_ = [
        ("left_fk", PositionQuaternion),
        ("right_fk", PositionQuaternion),
        ("left_fk_elb", PositionQuaternion),
        ("right_fk_elb", PositionQuaternion)
    ]


class GR1_ArmFk:
    def __init__(self):
        so_path = os.path.dirname(__file__) + "/libGR1_ArmFk.so"
        
        self.lib = ctypes.CDLL(so_path)

        self.lib.GR1_ArmFk_new.restype = c_void_p
        self.lib.GR1_ArmFk_delete.argtypes = [c_void_p]
        self.lib.GR1_ArmFk_calculateFK.argtypes = [
            c_void_p, POINTER(c_double), POINTER(FKResult)]
        self.obj = self.lib.GR1_ArmFk_new()

    def __del__(self):
        self.lib.GR1_ArmFk_delete(self.obj)

    def calculateFK(self, motor_positions):
        if len(motor_positions) != 14:
            raise ValueError("motor_positions must be of length 14")

        motor_positions_array = (c_double * 14)(*motor_positions)
        result = FKResult()

        self.lib.GR1_ArmFk_calculateFK(
            self.obj, motor_positions_array, ctypes.byref(result))

        return result


if __name__ == "__main__":
    arm_fk = GR1_ArmFk()

    motor_positions = [0.0] * 14
    result = arm_fk.calculateFK(motor_positions)

    print(
        f"Left FK Position: {result.left_fk.pos[:]}, Quaternion: {result.left_fk.quat[:]}")
    print(
        f"Right FK Position: {result.right_fk.pos[:]}, Quaternion: {result.right_fk.quat[:]}")
    print(
        f"Left Elbow FK Position: {result.left_fk_elb.pos[:]}, Quaternion: {result.left_fk_elb.quat[:]}")
    print(
        f"Right Elbow FK Position: {result.right_fk_elb.pos[:]}, Quaternion: {result.right_fk_elb.quat[:]}")
