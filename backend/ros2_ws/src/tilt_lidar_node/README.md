Takes LaserScan messages from topic `/scan` and servo angle (Int32) from topic `/lidar_servo_angle` and publishes a 3D point cloud (PointCloud2) in topic `/point_cloud`.

Assumes that the servo angle is in degrees and that the horizontal position is 90 degrees (range 0–180).

# TLDR: one command to run

```bash
ros2 launch tilt_lidar_node view_tilt_lidar_launch.py
```

or if using ros bag recording:

```bash
ros2 launch tilt_lidar_node view_tilt_lidar_sim_launch.py
```

# Manual way

## Run on recorded bag file

In directory `backend/bag_files`, run:

```bash
ros2 bag play -l lidar_servo_imu_pcl_slow_10hz_5hz_10hz --clock
```

in another terminal:

```bash
ros2 run tilt_lidar_node tilt_lidar_node \    
  --ros-args -p use_sim_time:=true
```

## Run on live data

```bash
ros2 run tilt_lidar_node tilt_lidar_node
```

## Visualize the point cloud

```bash
ros2 run rviz2 rviz2
```

Add a new display (CTRL + N) of type `PointCloud2` and set the topic to `/point_cloud`.

Set Global Options -> Fixed Frame to `base_link`.