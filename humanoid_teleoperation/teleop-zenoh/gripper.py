# Copyright 2024 Fourier Intelligence
# Author: Yuxiang Gao <yuxiang.gao@fftai.com>
from __future__ import annotations

import json
import socket
import struct
from dataclasses import dataclass, field
from enum import IntEnum, StrEnum
import time
import traceback
from typing import Literal, TypeAlias

RequestMethodType: TypeAlias = Literal["GET", "SET"]


def pascal_to_snake(name: str) -> str:
    return "".join(["_" + i.lower() if i.isupper() else i for i in name]).lstrip("_")


def is_pascal_case(name: str) -> bool:
    return name[0].isupper() and name[1].islower() and "_" not in name


def is_snake_case(name: str) -> bool:
    return name.islower() and "_" in name


class FourierActuatorUDPPort(IntEnum):
    """Fourier UDP ports"""

    COMM = 2333
    CTRL = 2334
    FAST = 2335


class FourierEncoderUDPPort(IntEnum):
    """Fourier UDP ports"""

    COMM = 2334
    CTRL = 2333
    FAST = 2335


FourierUDPPort: TypeAlias = FourierEncoderUDPPort | FourierActuatorUDPPort


@dataclass
class FIProtocol:
    """Helper class for building requests to FSA servers

    Args:
        method (RequestMethodType): GET or SET
        path (str): request target path. This key will be renamed to `reqTarget` during serialization.
        prop (str | None): Optional prop arg. This key will be renamed to `property` during serialization. Maybe move this to data because limited usage
        data (dict | None): Additional data for the request. During serialization, the content will be attached to the parent object.
    """

    method: RequestMethodType
    path: str = field(default_factory=str)
    prop: str = field(default_factory=str)
    data: dict = field(default_factory=dict)

    def serialize_model(self):
        res = {"method": self.method, "reqTarget": self.path, "property": ""}
        # if self.prop is not None:
        #     res["property"] = self.prop
        if self.data is not None:
            res.update(self.data)
        return res

    def encode(self):
        return json.dumps(self.serialize_model()).encode()


class FIFastIdentifier(IntEnum):
    # WATCHDOG = 0xFF
    ENABLE = 0x01
    DISABLE = 0x02
    CLEAR_FAULT = 0x03
    MODE_POSITION = 0x04
    MODE_VELOCITY = 0x05
    MODE_TORQUE = 0x06
    MODE_CURRENT = 0x07
    MODE_PD = 0x09
    SET_POSITION = 0x0A  # ">Bfff"
    SET_VELOCITY = 0x0B  # ">Bff"
    SET_TORQUE = 0x0C  # ">Bf"
    SET_CURRENT = 0x0D  # ">Bf"
    SET_PD = 0x0E  # ">Bf"
    GET_PVC = 0x1A  # ">Bfff"
    # GET_PVCT = 0x1D  # ">Bffff"
    GET_ERROR = 0x1B  # ">Bi"


@dataclass
class FIFastProtocol:
    ident: FIFastIdentifier
    payload: list[int | float] = field(default_factory=list)
    timestamp: float | None = None

    def encode(self):
        format_ = ">B"
        for i in self.payload:
            if isinstance(i, int):
                format_ += "i"
            elif isinstance(i, float):
                format_ += "f"
            else:
                raise ValueError(f"Invalid type {type(i)} for payload")
        return struct.pack(format_, self.ident, *self.payload)

    @classmethod
    def from_bytes(cls, data: bytes, ts: float | None = None) -> FIFastProtocol:
        try:
            ident = FIFastIdentifier(data[0])
        except ValueError as err:
            raise ValueError(f"Invalid identifier {data[0]} for payload") from err

        format_ = ">B"
        if ident == FIFastIdentifier.GET_PVC:
            format_ += "fff"
        # elif ident == FIFastIdentifier.GET_PVCT:
        #     format_ += "ffff"
        elif ident == FIFastIdentifier.GET_ERROR:
            format_ += "i"
        else:
            raise ValueError(f"Invalid identifier {ident} for payload")

        payload = struct.unpack(format_, data[: 1 + (len(format_) - 2) * 4])
        # for i in payload:
        #     if not isinstance(i, int | float):
        #         raise ValueError(f"Invalid type {type(i)} for payload")
        return cls(ident=ident, payload=list(payload[1:]), timestamp=ts)



class Gripper:
    def __init__(self, side: Literal["left", "right"]):
        self.side = side
        self.enabled = False
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(1/30)
        match self.side:
            case "left":
                self.addr = ("192.168.137.17", FourierActuatorUDPPort.FAST)
            case "right":
                self.addr = ("192.168.137.37", FourierActuatorUDPPort.FAST)
            case _:
                raise ValueError(f"Invalid side {side}")
        
        
    def init(self):
        self.sock.sendto(
            FIFastProtocol(FIFastIdentifier.ENABLE).encode(), self.addr)
        self.enabled = True
        self.close()
        time.sleep(1.0)
        self.reboot()
        self.enabled = False
        time.sleep(1)
        print("gripper init done.")
        
    def disable(self):
        self.sock.sendto(
            FIFastProtocol(FIFastIdentifier.DISABLE).encode(), self.addr)
        self.enabled = False

    def read(self):
        self.sock.sendto(
            FIFastProtocol(FIFastIdentifier.GET_PVC).encode(), self.addr)
        try:
            data, _ = self.sock.recvfrom(1024)
            decoded_data = FIFastProtocol.from_bytes(data)
            position, velocity, current = decoded_data.payload
            return position, velocity, current
        except TimeoutError:  # fail after 1 second of no activity
            return None

        except Exception as ex:
            traceback.print_exc()

    def open(self, pos=50.0):
        if not self.enabled:
            self.sock.sendto(
            FIFastProtocol(FIFastIdentifier.ENABLE).encode(), self.addr)
            self.enabled = True
        self.sock.sendto(
            FIFastProtocol(FIFastIdentifier.MODE_POSITION).encode(), self.addr)
        self.sock.sendto(
            FIFastProtocol(FIFastIdentifier.SET_POSITION, [float(pos), 0.0, 0.0]).encode(), self.addr)
        
    def close(self, torque=-1.0):
        if not self.enabled:
            self.sock.sendto(
            FIFastProtocol(FIFastIdentifier.ENABLE).encode(), self.addr)
            self.enabled = True
        self.sock.sendto(
            FIFastProtocol(FIFastIdentifier.MODE_TORQUE).encode(), self.addr)
        self.sock.sendto(
            FIFastProtocol(FIFastIdentifier.SET_TORQUE, [float(torque)]).encode(), self.addr)
        

    def reboot(self):
        data = FIProtocol(method="SET", path="/reboot")
        self.sock.sendto(data.encode(), (self.addr[0], FourierActuatorUDPPort.CTRL)) 