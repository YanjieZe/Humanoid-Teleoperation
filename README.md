# [Generalizable Humanoid Manipulation with Improved 3D Diffusion Policies](https://humanoid-manipulation.github.io/)

Our project is **fully open-sourced**. We separate them into two repos: [Learning & Deployment of iDP3](https://github.com/YanjieZe/Improved-3D-Diffusion-Policy) and [Humanoid Teleoperation](https://github.com/YanjieZe/Humanoid-Teleoperation). This repo is for humanoid teleoperation.


https://github.com/user-attachments/assets/9c013594-2181-47f7-a190-bb754c1fd934


# Humanoid Teleoperation

This repo is for humanoid teleoperation using Apple Vision Pro. 

Hardware requirements:
- [Apple Vision Pro](https://www.apple.com/apple-vision-pro/). 
- Humanoid robot. We use [Fourier GR1](https://www.fourierintelligence.com/gr1) as the robot platform, with [Inspire Hands](https://inspire-robots.store/collections/the-dexterous-hands) as the end-effector.
- RealSense camera, to stream the robot vision back to Apple Vision Pro. [RealSense L515](https://www.intelrealsense.com/lidar-camera-l515/) is used in our project.

This repo contains code for two parts:
-  [The Vision Pro APP](vision_pro_app/README.md) is installed on Apple Vision Pro.
-  [The teleoperation code](humanoid_teleoperation/README.md)  is installed on the onboard computer of Fourier GR1. 

The training data is collected and the converted to the training format to train [iDP3](https://github.com/YanjieZe/Improved-3D-Diffusion-Policy).


We provide the raw data example in [Google Drive](https://drive.google.com/file/d/1JOaOYugZDtkrz3aYpQq3w8zQPdy4AudD/view?usp=sharing). You could explore the data and use our provided script to convert the data to the training format. We have also provided the training data example in [Google Drive](https://drive.google.com/file/d/1c-rDOe1CcJM8iUuT1ecXKjDYAn-afy2e/view?usp=sharing).

## Installation

For APP installation, please see [Vision Pro APP README](vision_pro_app/README.md).

For teleoperation installation on the robot, please see [Teleoperation INSTALL](humanoid_teleoperation/README.md).

## Usage


1. In the terminal of the robot, start the robot server:

        cd humanoid_teleoperation
        bash start_robo.sh

2. In another terminal of the robot, start the teleop:

        cd humanoid_teleoperation
        bash collect_data.sh

3. In Apple Visiob Pro, open the app and enter. 
    - You should see the vision stream now. Snap your left hand to start the teleop.
    - To end the teleop, snap your left hand again.

4. The data is saved to `scripts/demo_dir` by default. 


5. Convert data to the training format using our script. Please specify the data path in the script first. For example, I set the path in `humanoid_teleoperation/convert_data.sh` as follow:

        # the path to our provided raw data
        demo_path=/home/ze/projects/Humanoid-Teleoperation/humanoid_teleoperation/scripts/demo_dir/raw_data_example
        # the path to save the converted data
        save_path=/home/ze/projects/Humanoid-Teleoperation/humanoid_teleoperation/scripts/demo_dir/training_data_example

    Then, run the script in the terminal:

        cd humanoid_teleoperation
        bash convert_data.sh
    
    You would have the converted training data. To use the training data, see [our iDP3 repo](https://github.com/YanjieZe/Improved-3D-Diffusion-Policy).

## Use it for your own robot

We also provide the instructions for adapting the teleoperation to your own robot.

By providing basic configuration and forward kinematics, you can teleoperate your humanoid robot arms and hands.

Please see [Humanoid Arm Retarget](https://github.com/Hao-Starrr/humanoid-arm-retarget) for more details.

## BibTeX

Please consider citing our paper if you find this repo useful:
```
@article{ze2024humanoid_manipulation,
  title   = {Generalizable Humanoid Manipulation with Improved 3D Diffusion Policies},
  author  = {Yanjie Ze and Zixuan Chen and Wenhao Wang and Tianyi Chen and Xialin He and Ying Yuan and Xue Bin Peng and Jiajun Wu},
  year    = {2024},
  journal = {arXiv preprint arXiv:2410.10803}
}
```

## Acknowledgement

We thank the authors of the following repos for their great work: [3D Diffusion Policy](https://github.com/YanjieZe/3D-Diffusion-Policy), [Diffusion Policy](https://github.com/columbia-ai-robotics/diffusion_policy), [VisionProTeleop](https://github.com/Improbable-AI/VisionProTeleop), [Open-TeleVision](https://github.com/OpenTeleVision/TeleVision). 
