# bash convert_data.sh


save_img=1
save_depth=0


demo_path=/home/ze/projects/Humanoid-Teleoperation/humanoid_teleoperation/scripts/demo_dir/raw_data_example
save_path=/home/ze/projects/Humanoid-Teleoperation/humanoid_teleoperation/scripts/demo_dir/training_data_example

cd scripts
python convert_demos.py --demo_dir ${demo_path} \
                                --save_dir ${save_path} \
                                --save_img ${save_img} \
                                --save_depth ${save_depth} \
