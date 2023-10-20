# _ Robust Autonomous Welding algorithm based on LbD and CAD model _

This ROS workspace provides all packages for autonomous welding based on Learning by Demonstration paradigm and CAD model. 
The overall aim of the project is to increase the roboustness of a welding routine allowing the human operatro teach to the robot the desired trajectory by means an hand-tool pointer which pose is detected by an optical system situated on top of the cell. 

## Requirements 

- Ubuntu 20.04
- ROS noetic

<!-- ### Launch camera
```
ros2 launch realsense2_camera rs_launch.py device_type:=d455 serial_no:=_213522253798 align_depth.enable:=true camera_name:=front_camera pointcloud.enable:=true unite_imu_method:=2 enable_gyro:=true enable_accel:=true
Tool to convert camera point cloud to laser scan for slam without a lidar https://github.com/ros-perception/depthimage_to_laserscan/tree/foxy-devel
ros2 launch depthimage_to_laserscan depthimage_to_laserscan_composed-launch.py
```

### Launch navigation
```
ros2 launch agribot_prototype_description display.launch.py
ros2 launch slam_toolbox online_async_launch.py params_file:=/home/agri/agribot/colcon_ws/src/robot_controller/config/SLAM_param.yaml 
//sim
//ros2 launch nav2_bringup navigation_launch.py params_file:=/home/agri/agribot/colcon_ws/src/agribot_prototype_description/config/nav2_params.yaml
ros2 launch nav2_bringup navigation_launch.py params_file:=/home/agri/agribot/colcon_ws/src/robot_controller/config/nav2_params.yaml
ros2 run nav2_costmap_2d nav2_costmap_2d_markers --ros-args --remap voxel_grid:=/local_costmap/voxel_grid --remap visualization_marker:=/my_marker
//ros2 run robot_localization navsat_transform_node
```

### Launch ROVER/Hoverboard bringup
```bash
# on board pc run
ros2 launch robot_controller bringup.launch.py 
# on personal pc run
ros2 launch robot_controller display.launch.py 
```

### Workflow for simulation (old, probably no longer working)
```bash
# gazebo, rviz, ekf node, state publisher
ros2 launch agribot_prototype_description display.launch.py

# slam 
ros2 launch slam_toolbox online_async_launch.py

ros2 launch nav2_bringup navigation_launch.py params_file:=/home/agri/agribot/colcon_ws/src/agribot_prototype_description/config/nav2_params.yaml
``` -->
