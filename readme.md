# UR5e Robot Vision Control System

**Language**: [English](readme.md) | [中文](readme.zh.md)

## Quick Start

### 1. Install Dependencies
```bash
sudo apt install ros-humble-ur-simulation-gazebo ros-humble-cv-bridge python3-opencv ros-humble-gazebo-ros-pkgs
```

### 2. Build Workspace
```bash
cd /home/xichen/robot_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 3. Launch System

#### Step 1: Start UR Robot Simulation
```bash
# Terminal 1: Launch UR simulation with MoveIt2
ros2 launch ur_simulation_gazebo ur_sim_moveit.launch.py
```

#### Step 2: Add Camera
```bash
# Terminal 2: Add world camera
ros2 launch ur_vision_system simple_camera_spawn.launch.py
```

#### Step 3: Add Objects
```bash
# Terminal 3: Generate test objects (red cylinder, green box, blue prism)
ros2 launch ur_vision_system spawn_random_objects.launch.py
```

#### Step 4: Open Camera View
```bash
# Terminal 4: Open camera image viewer
ros2 run rqt_image_view rqt_image_view /world_camera/world_camera/image_raw
```

#### Step 5: Run Controller
```bash
# Terminal 5: Start main controller
ros2 run main_controller main_controller
```

### 4. Use Controller Menu
```
=== Main Controller Menu ===
1. Initialize robot      # Move to initial position
2. Object detection      # Detect objects and show positions (run before home pose)
3. Move to home pose     # Move to home position  
4. Move to objects       # Visit objects from near to far
5. Exit                  # Exit program
```

**Usage Flow**: Run options 1 → 2 → 3 → 4 in sequence.

## System Components

- **Main Controller**: Integrates vision and robot control
- **Vision System**: Detects red/green/blue objects using camera
- **Robot Control**: UR5e arm movement with MoveIt2
- **Camera Config**: Unified camera parameters in `robot_common/camera_config.py`

## Troubleshooting

### Build Issues
```bash
rm -rf build/ install/ log/
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

### No Camera Image
```bash
ros2 topic list | grep world_camera
ros2 topic echo /world_camera/world_camera/image_raw --once
```

### Robot Not Moving
```bash
ros2 control list_controllers
ros2 service list | grep compute
```

---

**Tech Stack**: ROS2 Humble, Gazebo, MoveIt2, OpenCV, Python 3.10