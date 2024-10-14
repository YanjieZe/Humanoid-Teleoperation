import asyncio
import time
from enum import Enum
from collections import deque 

import cv2
import numpy as np
from vuer import Vuer
from vuer.schemas import Hands, ImageBackground

# from multi_realsense_async import MultiRealSenseAsync
from multi_realsense import MultiRealSense

import action_util
import sys
# sys.path.append("..")
import os
import numpy as np
import time
import argparse
from termcolor import cprint
import pickle
import h5py
from tqdm import tqdm
import cv2
import quaternionic
import torch
from scipy.ndimage import zoom

sys.path.append("../teleop-zenoh")
from communication import *
from gesture import SnapMonitor
from retarget import ArmRetarget, HandRetarget, UpperBodyRetarget
from filter import OneEuroFilter

async def ainput(prompt: str) -> str:
    return await asyncio.to_thread(input, prompt)


class MovingAverage:
    def __init__(self, window_size):
        self.window_size = window_size
        self.window = []

    def next(self, val):
        if len(self.window) < self.window_size:
            self.window.append(val)
            return sum(self.window) / len(self.window)
        else:
            self.window.pop(0)
            self.window.append(val)
            return sum(self.window) / self.window_size

    def get(self):
        return sum(self.window) / len(self.window)



class Silverscreen:
    def __init__(self, fps=50, use_cert=True):    
        
        if use_cert:
            self.app = Vuer(
                host="0.0.0.0",
                cert="./cert.pem",
                key="./key.pem",
                queries=dict(grid=False),
                # set window size to 1920x1080
                # window_size=(2920, 1080),
            )
        else:
            self.app = Vuer(host="0.0.0.0", queries=dict(grid=False))
        self.app.add_handler("HAND_MOVE", self.on_hand_move)
        self.app.add_handler("CAMERA_MOVE", self.on_cam_move)
        self.app.spawn(self.main, start=False)

        self.fps = fps

        self.left_hand = np.zeros((4, 4))
        self.right_hand = np.zeros((4, 4))
        self.left_landmarks = np.zeros((25, 3))
        self.right_landmarks = np.zeros((25, 3))
        self.head_matrix = np.zeros((4, 4))
        self.aspect = 1.0
        
        self.color_array = []
        self.depth_array = []
        self.cloud_array = []
        self.vision_pro_queue = deque(maxlen=1000)
        self.lock = asyncio.Lock()
        
        self.camera_context = MultiRealSense(use_front_cam=True, 
                                            use_right_cam=False,
                                            front_num_points=10000)

    async def on_cam_move(self, event, session):
        self.head_matrix = np.asarray(event.value["camera"]["matrix"]).reshape(
            (4, 4), order="F"
        )
        self.aspect = event.value["camera"]["aspect"]

    async def on_hand_move(self, event, session):
        self.left_hand = np.asarray(event.value["leftHand"]).reshape((4, 4), order="F")
        self.right_hand = np.asarray(event.value["rightHand"]).reshape(
            (4, 4), order="F"
        )
        self.left_landmarks = np.asarray(event.value["leftLandmarks"]).reshape((25, 3))
        self.right_landmarks = np.asarray(event.value["rightLandmarks"]).reshape(
            (25, 3)
        )

    async def main(self, session):
        
        session.upsert @ Hands(fps=self.fps, stream=True, key="hands")
        measured_duration = MovingAverage(10)

        if True:
            
            ##### some parameters #####
            ROBOT_IP = '192.168.137.252'
            PLATFORM = "AVP"

            
            self.save_img = True
            self.save_depth = True
            # length of demo
            length = 2000
                        
            # disable wrist would be eaiser
            use_wrist = False
            
            demo_dir = "demo_dir"
            demo_name = "demo"
            
            
            control_fps = self.fps
            STEP_TIME = 1 / control_fps
            th = control_fps * np.pi/60
            # 3 degrees = angle threshold in one iteration
            
            
            qpos_body = np.array([-np.pi / 12, 0, 0, -1.6, 0, 0, 0, 
                    -np.pi / 12, 0, 0, -1.6, 0, 0, 0])
            qpos_init2 = np.array([-np.pi / 12, 0, 1.5, -1.6, 0, 0, 0, 
                        -np.pi / 12, 0, -1.5, -1.6, 0, 0, 0])
            hand_init = np.ones(12)
            hand_angles = hand_init.copy()
        
            
            ###########################
            
            
            ##### initialize teleop #####
            snap_monitor = SnapMonitor()
            arm_solver = ArmRetarget(PLATFORM)
            upbody_solver = UpperBodyRetarget()
            upbody_comm = UpperBodyCommunication()
            hand_solver = HandRetarget()
            hand_comm = HandCommunication()
            
            ##### initialize filter #####
            # filter = AverageFilter(10, 14)
            oef_min_cutoff = 0.05
            oef_beta = 1.5
            oef = None


            ##### initialize demo saving #####
            os.makedirs(demo_dir, exist_ok=True)
            record_file_name = os.path.join(demo_dir, demo_name+".h5")
            env_qpos_array = []
            action_array = []
            eef_pos_array = []
            eef_quat_array = []
                        
            
            ##### initialize robot #####
            upbody_initpos = np.concatenate([qpos_init2])
            upbody_comm.init_set_pos(upbody_initpos)
            hand_comm.send_hand_cmd(hand_init[:6], hand_init[6:])
            q_14d = qpos_body.copy()
            q_upbody = np.zeros(6)
        
            # cprint("ready to start streaming vision. please enter the app. and then press enter", "cyan")
            # await ainput("enter")
            time.sleep(2)
            cprint("ready to start streaming vision!", "cyan")
            
            ##### initialize VR #####
            vr_comm = VRCommunication(record=False, latency=0)
        
                
            #### detect teleop start by snap ####
            while True:
                r = vr_comm.get_data()
                snap_monitor.update(r)
                cam_dict = self.camera_context()
                
                img_show_vr = cam_dict['color']
                    
                # stream vision to vision pro
                session.upsert(
                    [ImageBackground(
                        img_show_vr,format="jpeg",quality=40,key="left-image",interpolate=False,aspect=960./540,distanceToCamera=2,
                        position=[0, -0.3, -1],rotation=[0, 0, 0],layers=1,),],
                    to="bgChildren",)
                if snap_monitor.left.snap_detected:
                    print("snap detected! start teleop.")
                    break
                # time.sleep(0.02)
                await asyncio.sleep(1 / control_fps)
            
                
            upbody_comm.init_set_pos(qpos_body)
            
            
            eef_pos, eef_quat = action_util.init_arm_pos, action_util.init_arm_quat
            q_14d = arm_solver.ik(q_14d, eef_pos, eef_quat)
            if not use_wrist:
                q_14d[5] = 0
                q_14d[6] = 0
                q_14d[5+7] = 0
                q_14d[6+7] = 0

            upbody_comm.init_set_pos(q_14d)
            
            eef_pos_array.append(eef_pos)
            eef_quat_array.append(eef_quat)
            
            
            time.sleep(3)

            ### teleop loop ###
            for i in range(length):
                start = time.time()
                cam_dict = self.camera_context()
                
                # stream visi
                # on to vision pro
                img_show_vr = cam_dict['color']
                    
                # stream vision to vision pro
                session.upsert(
                    [ImageBackground(
                        img_show_vr,format="jpeg",quality=40,key="left-image",interpolate=False,aspect=960./540,distanceToCamera=2,
                        position=[0, -0.3, -1],rotation=[0, 0, 0],layers=1,),],
                    to="bgChildren",)
                
                # resize
                target_size = (224, 224)  # Replace with your desired dimensions
                color_resized = cv2.resize(cam_dict['color'], target_size, interpolation=cv2.INTER_LINEAR)
                depth_resized = cv2.resize(cam_dict['depth'], target_size, interpolation=cv2.INTER_LINEAR)
                
                await self.write_data(color_resized, depth_resized, cam_dict['point_cloud'])
            
                robot_pos = upbody_comm.get_pos()
                hand_pos = hand_comm.get_qpos()
                env_qpos = np.concatenate([robot_pos, hand_pos])
                env_qpos_array.append(env_qpos)
                
                r = vr_comm.get_data()
                snap_monitor.update(r)
                if snap_monitor.left.snap_detected:
                    break
            

                q_upbody = upbody_solver.solve_upper_body_angles(r["head"][0]) # 6 dim, wasit yaw picth roll, head 
                q_14d, eef_pos, eef_quat = arm_solver.solve_arm_angles(q_14d, r, q_upbody[1], use_wrist=use_wrist)
                if not use_wrist:
                    q_14d[5] = 0
                    q_14d[6] = 0
                    q_14d[5+7] = 0
                    q_14d[6+7] = 0
                q_total = np.concatenate([q_upbody, q_14d])
                left_hand_angles, right_hand_angles = hand_solver.solve_fingers_angles(r)
                

                # ======================== filter ========================
                if oef is None:
                    oef = OneEuroFilter(time.time(), q_total, min_cutoff=oef_min_cutoff, beta=oef_beta)
                    filtered_pos = q_total
                else:
                    filtered_pos = oef(time.time(), q_total)
                  
                hand_angles = np.concatenate((left_hand_angles, right_hand_angles)) / 1000.           
            
                
                upbody_comm.set_pos(filtered_pos)
                hand_comm.send_hand_cmd(hand_angles[6:], hand_angles[:6])

                action = np.concatenate([filtered_pos, hand_angles])
                # left arm, right arm, right hand, left hand
                action = action_util.joint32_to_joint25(action)
                action_array.append(action)
                
                
                duration = time.time() - start
                measured_duration.next(duration)
                fps = 1 / duration
                text = f"time (ms): {measured_duration.get() * 1000.0: >#8.3f} | step: {i} / {length} | fps: {fps:.3f}"
                print(text, end="\r")
                await asyncio.sleep(1 / self.fps)

            self.camera_context.finalize()
            # save the data
            discard_end_length = 10 # discard the end
            
            with h5py.File(record_file_name, "w") as f:
                seq_length = len(action_array)
                color_array = np.array(self.color_array)[:seq_length]
                depth_array = np.array(self.depth_array)[:seq_length]
                cloud_array = np.array(self.cloud_array)[:seq_length]
                env_qpos_array = np.array(env_qpos_array)[:seq_length]
                action_array = np.array(action_array)
                
            
                    
                f.create_dataset("color", data=color_array[:-discard_end_length])
                f.create_dataset("depth", data=depth_array[:-discard_end_length])
                f.create_dataset("cloud", data=cloud_array[:-discard_end_length])
                f.create_dataset("env_qpos_proprioception", data=env_qpos_array[:-discard_end_length])
                f.create_dataset("action", data=action_array[:-discard_end_length])
    
                
            cprint(f"color shape: {color_array.shape}", "yellow")
            cprint(f"depth shape: {depth_array.shape}", "yellow")
            cprint(f"cloud shape: {cloud_array.shape}", "yellow")
            cprint(f"action shape: {action_array.shape}", "yellow")
            cprint(f"env_qpos shape: {env_qpos_array.shape}", "yellow")
            cprint(f"save data at step: {seq_length} in {record_file_name}", "yellow")

            choice = input("whether to rename: y/n")
            if choice == "y":
                renamed = input("file rename:")
                os.rename(src=record_file_name, dst=record_file_name.replace("demo.h5", renamed+'.h5'))
            cprint("Program finished.", "green")
            exit()
            
    async def write_data(self, color, depth, point_cloud):
        async with self.lock:
            if self.save_img:
                self.color_array.append(color)
            if self.save_depth:
                self.depth_array.append(depth)
            self.cloud_array.append(point_cloud)
     
            
if __name__ == "__main__":
    ss = Silverscreen(use_cert=True)
    ss.app.run()
