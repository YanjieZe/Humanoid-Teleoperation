import pyrealsense2 as rs
import time
import numpy as np
import cv2
import os

np.set_printoptions(threshold=np.inf)


def get_realsense_id():
    ctx = rs.context()
    devices = ctx.query_devices()
    devices = [devices[i].get_info(rs.camera_info.serial_number) for i in range(len(devices))]
    devices.sort() # Make sure the order is correct
    print("Found {} devices: {}".format(len(devices), devices))
    return devices

def init_given_realsense(
    device,
    cfg=dict(
        width=640,
        height=360,
        depth_only=False,
        rgb_only=False,
    )
):
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(device)

    print("Initializing camera {}".format(device))

    # Get RGBD streams
    print(cfg.width, cfg.height)
    if not cfg.depth_only:
        print("enable depth", cfg.width, cfg.height)
        config.enable_stream(rs.stream.depth, int(cfg.width), int(cfg.height), rs.format.z16, 30)
        # config.enable_stream(rs.stream.depth, int(cfg.width), int(cfg.height), rs.format.z16, 15)
    if not cfg.rgb_only:
        print("enable color", cfg.width, cfg.height)
        config.enable_stream(rs.stream.color, int(cfg.width), int(cfg.height), rs.format.rgb8, 30)

    profile = pipeline.start(config)
    depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
    
    sensor_dep = profile.get_device().first_depth_sensor()

    print("Trying to set Exposure")
    exp = sensor_dep.get_option(rs.option.exposure)
    print("exposure =  ", exp)
    print("Setting exposure to new value")
    exp = sensor_dep.set_option(rs.option.exposure, 10000)
    exp = sensor_dep.get_option(rs.option.exposure)
    print("New exposure = ", exp)

    print("Aligning")
    align_to = rs.stream.color
    align = rs.align(align_to)
    print("Aligned")

    return pipeline, align, depth_scale

def init_realsense(cfg=dict(
    width=640,
    height=360,
    depth_only=False,
    rgb_only=False,
    cam_num=1,
)):
    # Create a list to store the pipeline objects
    pipelines = []
    # Create configurations for each camera
    configs = []
    depth_scales = []

    ctx = rs.context()
    devices = ctx.query_devices()
    devices = [devices[i].get_info(rs.camera_info.serial_number) for i in range(len(devices))]
    devices.sort() # Make sure the order is correct
    print("Found {} devices: {}".format(len(devices), devices))

    for i in range(cfg["cam_num"]):
        print("Initializing camera {}".format(i))
        # Create a context object. This object owns the handles to all connected realsense devices
        pipeline = rs.pipeline()
        pipelines.append(pipeline)
        config = rs.config()
        config.enable_device(devices[i])
        configs.append(config)

        # Get RGBD streams
        if not cfg.depth_only:
            print("enable depth")
            config.enable_stream(rs.stream.depth, cfg.width, cfg.height, rs.format.z16, 30)
        if not cfg.rgb_only:
            print("enable color")
            config.enable_stream(rs.stream.color, cfg.width, cfg.height, rs.format.rgb8, 30)

        profile = pipeline.start(config)
        depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        depth_scales.append(float(depth_scale))
      
        sensor_dep = profile.get_device().first_depth_sensor()
        print("Trying to set Exposure")
        exp = sensor_dep.get_option(rs.option.exposure)
        print("exposure =  ", exp)
        print("Setting exposure to new value")
        exp = sensor_dep.set_option(rs.option.exposure, 10000)
        exp = sensor_dep.get_option(rs.option.exposure)
        print("New exposure = ", exp)

    # Create an align object
    # rs.align allows us to perform alignment of depth frames to others frames
    # The "align_to" is the stream type to which we plan to align depth frames.
    align_to = rs.stream.color
    align = rs.align(align_to)

    return pipelines, align, depth_scales

def finalize_realsense(pipeline):
    pipeline.stop()

def get_depth_filter():
    # filter stuff
    depth_to_disparity = rs.disparity_transform(True)
    disparity_to_depth = rs.disparity_transform(False)
    decimation = rs.decimation_filter()
    decimation.set_option(rs.option.filter_magnitude, 4)
    spatial = rs.spatial_filter()
    spatial.set_option(rs.option.filter_magnitude, 5)
    spatial.set_option(rs.option.filter_smooth_alpha, 1)
    spatial.set_option(rs.option.filter_smooth_delta, 50)
    spatial.set_option(rs.option.holes_fill, 3)
    temporal = rs.temporal_filter()
    hole_filling = rs.hole_filling_filter(mode=1)

    return depth_to_disparity, disparity_to_depth, decimation, spatial, temporal, hole_filling

def depth_process(frame, depth_to_disparity=None, disparity_to_depth=None, decimation=None, spatial=None, temporal=None, hole_filling=None):
    # if decimation is not None:
    #     frame = decimation.process(frame)
    # if depth_to_disparity is not None:
    #     frame = depth_to_disparity.process(frame)

    # if disparity_to_depth is not None:
    #     frame = disparity_to_depth.process(frame)
    # if hole_filling is not None:
    #     frame = hole_filling.process(frame)
    # if spatial is not None:
    #     frame = spatial.process(frame)
    # if temporal is not None:
    #     frame = temporal.process(frame)

    return frame


if __name__=='__main__':
    # Create a context object. This object owns the handles to all connected realsense devices
    ctx = rs.context()
    devices = ctx.query_devices()
    devices = [devices[i].get_info(rs.camera_info.serial_number) for i in range(len(devices))]
    devices.sort() # Make sure the order is correct

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(devices[0])
    # filter stuff
    depth_to_disparity = rs.disparity_transform(True)
    disparity_to_depth = rs.disparity_transform(False)
    decimation = rs.decimation_filter()
    decimation.set_option(rs.option.filter_magnitude, 4)
    spatial = rs.spatial_filter()
    spatial.set_option(rs.option.filter_magnitude, 5)
    spatial.set_option(rs.option.filter_smooth_alpha, 1)
    spatial.set_option(rs.option.filter_smooth_delta, 50)
    spatial.set_option(rs.option.holes_fill, 3)
    temporal = rs.temporal_filter()
    hole_filling = rs.hole_filling_filter(mode=2)
    # start
    config.enable_stream(rs.stream.depth, 640, 360, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 360, rs.format.rgb8, 30)

    # config.enable_stream(rs.stream.depth, 640, 360, rs.format.z16, 30)
    # config.enable_stream(rs.stream.color, 640, 360, rs.format.rgb8, 30)

    print(dir(rs.stream))
    profile = pipeline.start(config)
    depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()

    # Create an align object
    # rs.align allows us to perform alignment of depth frames to others frames
    # The "align_to" is the stream type to which we plan to align depth frames.
    align_to = rs.stream.color
    # align_to = rs.stream.depth
    align = rs.align(align_to)

    sensor_dep = profile.get_device().first_depth_sensor()
    print("Trying to set Exposure")
    exp = sensor_dep.get_option(rs.option.exposure)
    print("exposure =  ", exp)
    print("Setting exposure to new value")
    exp = sensor_dep.set_option(rs.option.exposure, 10000)
    exp = sensor_dep.get_option(rs.option.exposure)
    print("New exposure = ", exp)

    # plt.figure(figsize=(24, 12), dpi=80)

    # ax1 = plt.subplot(1,2,1)
    # ax2 = plt.subplot(1,2,2)
    # ax3 = plt.subplot(1,3,3)

    # im1 = None
    # im2 = None
    # im3 = None

    # plt.ion()
    os.system("rm -rf frames")
    os.system("rm -rf frames_npy")
    # os.system("rm -rf depth_output")

    os.mkdir("frames")
    os.mkdir("frames_npy")
    # os.mkdir("depth_output")

    for x in range(20):
        pipeline.wait_for_frames()

    start = time.time()
    # for gitr in range(100):
    while True:
        frame = pipeline.wait_for_frames(timeout_ms = 50)
        aligned_frames = align.process(frame)

        # Get aligned frames
        depth_frame = frame.get_depth_frame() 
        color_frame = aligned_frames.get_color_frame()

        # color_frame = frame.get_color_frame()
        # depth_frame = frame.get_depth_frame()

        # print("color shape", np.asanyarray(color_frame.get_data()).shape)
        # print("depth shape", np.asanyarray(depth_frame.get_data()).shape)
        
        # depth_frame = decimation.process(depth_frame)
        raw_depth = np.asanyarray(depth_frame.get_data())[80:440,:]*depth_scale
        cv2.imshow("raw depth frame", raw_depth)
        a = np.asarray(depth_frame.get_data())
        # print(a.shape)
        # plt.imsave("depth_frame_{}.png".format(gitr), np.asanyarray(depth_frame.get_data())*depth_scale, vmin=-2, vmax=2)
        # depth_frame = depth_to_disparity.process(depth_frame)
        # depth_frame = spatial.process(depth_frame)
        # depth_frame = disparity_to_depth.process(depth_frame)
        # plt.imsave("spatial_frame_{}.png".format(gitr), np.asanyarray(depth_frame.get_data())*depth_scale, vmin=-2, vmax=2)
        # depth_frame = hole_filling.process(depth_frame)
        # plt.imsave("hole_frame_{}.png".format(gitr), np.asanyarray(depth_frame.get_data())*depth_scale, vmin=-2, vmax=2)
        # depth_frame = temporal.process(depth_frame)
        # plt.imsave("temporal_frame_{}.png".format(gitr), np.asanyarray(depth_frame.get_data())*depth_scale, vmin=-2, vmax=2)
        # np_frame = (np.asanyarray(frame.get_data()) * depth_scale)
        cv2.imshow("color frame", np.asanyarray(color_frame.get_data()))
        processed_color = np.asanyarray(color_frame.get_data())
        cv2.imshow("processed color frame", processed_color)
        # print(depth_frame.get_data()) 
        # print(1./depth_scale)
        # processed_frame = np.asanyarray(depth_frame.get_data())*depth_scale
        # processed_frame = np.clip(processed_frame, 0.1, 2)
        # processed_frame = processed_frame[:,1:]
        # processed_frame = processed_frame[:-54,:]
        # processed_frame = processed_frame.clip(0.2, 2)
        # processed_frame[processed_frame < 0.2] = 0.
        # processed_frame /= 2
        # cv2.imshow("processed depth frame", processed_frame)
        depth_frame = depth_process(depth_frame, depth_to_disparity, disparity_to_depth, decimation, spatial, temporal, hole_filling)
        depth_frame = np.asanyarray(depth_frame.get_data())
        # print(depth_frame[366,371]*depth_scale)
        processed_depth = process_and_resize(depth_frame, depth_scale=depth_scale, resize=False)
        resized_depth = process_and_resize(depth_frame, depth_scale=depth_scale, dsize=(400,225))
        print(resized_depth[165,225])
        cv2.imshow("processed depth frame", processed_depth)
        cv2.imshow("resized depth frame", resized_depth)
       # cv2.imwrite("real_depth.png", processed_depth*255)
       # cv2.imwrite("real_depth_resize.png", resized_depth*255)
       # exit(0)
        cv2.waitKey(1)
        # low_res = process_and_resize(np_frame)
        # print(gitr, low_res.min(), low_res.max())
        # np.save("frames_npy/np_frame_{}.npy".format(gitr), low_res)
        # plt.imsave("frames/np_frame_{}.png".format(gitr), np_frame, vmin=-2, vmax=2)
        # plt.imsave("frames/np_frame_{}.png".format(gitr), low_res)
        # cv2.imwrite("frames/np_frame_{}.png".format(gitr), low_res)

    print("FPS: ", 100 / (time.time()-start), (time.time()-start))

    pipeline.stop()