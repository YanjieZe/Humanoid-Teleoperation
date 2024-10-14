"""
Author: WANG Wenhao
Date: 2024-06-29
Version: 1.0.0
copyright (c) 2024 All Rights Reserved
"""

import socket
import json
from threading import Thread
import numpy as np
import quaternionic


YUP2ZUP = np.array([[1, 0, 0, 0],
                    [0, 0, 1, 0],
                    [0, 1, 0, 1.70],
                    [0, 0, 0, 1]], dtype=np.float64)

T_leftWrist_pico = np.array([
    [-1, 0, 0, 0],
    [0, -1, 0, 0],
    [0, 0, -1, 0],
    [0, 0, 0, 1]
])
T_leftElbow_pico = np.array([
    [1, 0, 0, 0],
    [0, -1, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
])
T_rightWrist_pico = np.array([
    [-1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
])
T_rightElbow_pico = np.array([
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, -1, 0],
    [0, 0, 0, 1]
])


def msg2transform(msg):
    T = np.eye(4)
    T[0:3, 3] = np.array(msg[0:3])
    T[0:3, 0:3] = quaternionic.array(msg[3:7]).to_rotation_matrix
    return T


class PicoStreamer:

    def __init__(self, ip, record=False):

        # Vision Pro IP
        self.ip = ip
        self.record = record
        self.recording = []
        self.latest = None
        self.axis_transform = YUP2ZUP
        self.start_streaming()

    def start_streaming(self):

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.ip, 8005))
        stream_thread = Thread(target=self.stream)
        stream_thread.start()
        while self.latest is None:
            pass
        print(' == DATA IS FLOWING IN! ==')
        print('Ready to start streaming.')

    def stream(self):
        while True:
            try:
                print("\nWaiting to receive message...")
                data, address = self.sock.recvfrom(4096)
                data_str = data.decode()
                t = json.loads(data_str)
                r = {
                    'left_wrist': self.axis_transform @ msg2transform(t['left_wrist']) @ T_leftWrist_pico,
                    'left_elbow': self.axis_transform @ msg2transform(t['left_elbow']) @ T_leftElbow_pico,
                    'right_wrist': self.axis_transform @ msg2transform(t['right_wrist']) @ T_rightWrist_pico,
                    'right_elbow': self.axis_transform @ msg2transform(t['right_elbow']) @ T_rightElbow_pico,
                }
                # json.loads(data_str)['left_wrist'] = (x y z) (w x y z)
                r['left_forearmWrist'] = np.eye(4)
                r['left_forearmWrist'][0:3, 0:3] = r["left_elbow"][0:3, 0:3]
                r['left_forearmWrist'][0:3, 3] = r['left_wrist'][0:3, 3]
                r['right_forearmWrist'] = np.eye(4)
                r['right_forearmWrist'][0:3, 0:3] = r["right_elbow"][0:3, 0:3]
                r['right_forearmWrist'][0:3, 3] = r['right_wrist'][0:3, 3]
                print(r)
                if self.record:
                    self.recording.append(r)
                self.latest = r

            except Exception as e:
                print(f"An error occurred: {e}")
            pass

    def get_latest(self):
        return self.latest

    def get_recording(self):
        return self.recording


if __name__ == "__main__":

    streamer = PicoStreamer(ip='0.0.0.0')
    while True:
        latest = streamer.get_latest()
        print(latest)
