# UR5e Intelligent Robot Vision Control System

**Language**: [English](readme.md) | [中文](readme.zh.md)

## Project Overview
This is a UR5e robotic arm intelligent control system based on ROS2, integrating MoveIt2 motion planning, Gazebo physics simulation, computer vision, and custom controllers. The system adopts a modular design, supporting precise control in both joint space and Cartesian space, with real-time object detection and automated grasping capabilities.

## 🏗️ System Architecture

```
robot_ws/src/
├── my_robot_controller/              # Core control system
│   ├── main_controller/             # Main controller node - integrates vision and motion
│   ├── move_node/                   # Motion control node - MoveIt2 interface
│   ├── robot_common/                # Common libraries - geometry processing, state management, camera config
│   └── vision_node/                 # Vision control node - object detection and position estimation
├── ur_vision_system/                # Environment configuration package - Gazebo camera models
├── Universal_Robots_ROS2_Gazebo_Simulation/  # Official UR simulation
└── README.md                        # This documentation
```

## 🚀 Quick Start

### 1. Environment Setup
```bash
# Install dependencies
sudo apt install ros-humble-ur-simulation-gazebo
sudo apt install ros-humble-cv-bridge python3-opencv
sudo apt install ros-humble-gazebo-ros-pkgs
```

### 2. Build Workspace
```bash
cd /home/xichen/robot_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 3. Launch Complete System

#### Option A: Basic Simulation Test
```bash
# Terminal 1: Launch empty Gazebo environment
ros2 launch ur_vision_system empty_gazebo.launch.py

# Terminal 2: Add camera
ros2 launch ur_vision_system simple_camera_spawn.launch.py

# Terminal 3: Generate random test objects
ros2 launch ur_vision_system spawn_random_objects.launch.py

# Terminal 4: Run position detection test
ros2 run vision_node position_detector_test
```

#### Option B: Complete Robot System
```bash
# Terminal 1: Launch UR simulation environment
ros2 launch ur_simulation_gazebo ur_sim_moveit.launch.py

# Terminal 2: Add vision camera
ros2 launch ur_vision_system simple_camera_spawn.launch.py

# Terminal 3: Run main controller
ros2 run main_controller main_controller
```

## 🎯 Core Features

### 1. Intelligent Geometry Processing (`robot_common/arm_geometry.py`)
- **Smart Shape Detection**: Detects "concave down" configuration based on Link2 and Link3 ground angle comparison
- **Precise Geometric Transforms**: Calculates alpha angle of J4-J2-J3 triangle at J2 point for elbow-up/elbow-down conversion
- **Link4 Angle Preservation**: Maintains Link4 ground angle during transformation
- **360° Joint Handling**: Automatically adjusts J4 to equivalent angle range
- **Joint Limit Management**: Complete UR5 joint limit processing
- **Multi-seed Strategy**: Improves inverse kinematics solution success rate

### 2. Computer Vision System (`vision_node/`)
- **Real-time Object Detection**: Multi-object detection based on HSV color space
- **Precise Position Estimation**: Accurate conversion from pixel coordinates to world coordinates
- **Supported Object Types**: Red cylinders, green boxes, blue prisms
- **Automatic Camera Configuration**: Unified camera parameter management system
- **High-precision Detection**: Vertical downward camera, covering ±0.4m work area

### 3. Motion Controller (`move_node/`)
- **RobotArmController**: Simplified API interface supporting joint and Cartesian motion
- **PositionController**: MoveIt2 Cartesian space controller
- **JointController**: Joint space trajectory controller
- **Intelligent Obstacle Avoidance**: Safety checks based on workspace boundaries

### 4. Environment Configuration (`ur_vision_system/`)
- **Camera Model Management**: SDF format depth camera models
- **Test Object Generation**: Various shapes and colors of test objects
- **Unified Configuration Architecture**: `camera_config.py` implements global parameter management

## 📊 Vision Detection System

### Current Configuration ✅ WORKING
- **Camera Position**: (0, 0, 1.5) - 1.5 meters above origin
- **Orientation**: Vertically downward (pitch=90 degrees)
- **Resolution**: 640x480 pixels
- **Field of View**: 60 degrees, covering 2.6m x 2.6m ground area
- **Working Range**: ±0.4 meters (within robotic arm workspace)

### Real-time Detection Output
```bash
# Typical output example
Detected 3 objects: red: (0.20, -0.17) conf:0.85 | green: (-0.02, -0.35) conf:0.92 | blue: (-0.32, -0.11) conf:0.78
```

### Supported Object Colors
- 🔴 **Red** (red) - Cylinder
- 🟢 **Green** (green) - Box  
- 🔵 **Blue** (blue) - Prism

## 🔧 API Usage Examples

### Basic Motion Control
```python
from move_node.robot_arm_controller import RobotArmController

# Create controller
robot = RobotArmController()

# Joint space motion
robot.move_to_joint_positions(0.0, -1.57, 0.0, -1.57, 0.0, 0.0)

# Cartesian space motion
robot.move_to_position(0.3, 0.2, 0.5, 0.0, 0.0, 0.0, 1.0)

# Relative motion
robot.move_relative(0.1, 0.0, 0.0)  # Move 10cm in X direction
```

### Vision Detection API
```python
from vision_node.vision_api import VisionAPI

# Create vision API
vision = VisionAPI()
vision.initialize()

# Get all detected object positions
positions = vision.get_object_positions()

# Get largest object position
largest = vision.get_largest_object_position()

# Get specific color objects
red_objects = vision.get_largest_object_position(color_filter='red')
```

### Integrated Control Example
```python
# Vision-guided grasping
vision = VisionAPI()
robot = RobotArmController()

# Detect objects
target = vision.get_largest_object_position(color_filter='red')

if target:
    # Move above object
    robot.move_to_position(target.x, target.y, target.z + 0.1, 0, 0, 0, 1)
    
    # Descend to grasp
    robot.move_relative(0, 0, -0.05)
```

## 🎮 Interactive Control

Main controller provides interactive menu:
```
=== Main Controller Menu ===
1. Test basic robot functions      # Test basic robot functions
2. Test vision-robot integration   # Test vision+robot integration
3. Quick vision test (check objects) # Quick object detection
4. Exit                           # Exit
```

## 🛠️ Technical Details

### Geometric Algorithms
System implements precise transformations based on actual geometric relationships:

1. **Shape Detection**: `Link2 angle = min(|j2|, |j2+π|)`, `Link3 angle = min(|j2+j3|, |j2+j3+π|)`
2. **Alpha Angle Calculation**: Angle of J4-J2-J3 triangle at J2 point, using cosine law and UR5 parameters
3. **Configuration Conversion Formula**:
   - J3 flip: `new_j3 = -j3`
   - J2 adjustment: `new_j2 = j2 ± 2*alpha`
   - J4 preservation: `new_j4 = (j2+j3+j4) - new_j2 - new_j3`

### Vision Coordinate Transformation
Direct mapping designed specifically for vertical downward camera:
```python
# Calculate ground coverage
ground_coverage = 2 * camera_height * tan(fov/2)
pixel_to_meter_ratio = ground_coverage / image_width

# Pixel to world coordinate conversion
world_x = (pixel_x - center_x) * pixel_to_meter_ratio
world_y = (pixel_y - center_y) * pixel_to_meter_ratio
```

## 📖 Testing Guide

### Phase 1: Basic Depth Camera Function Verification
```bash
# Launch empty Gazebo
ros2 launch ur_vision_system empty_gazebo.launch.py

# Add camera
ros2 launch ur_vision_system simple_camera_spawn.launch.py

# Add test objects
ros2 launch ur_vision_system spawn_random_objects.launch.py

# Verify detection
ros2 run vision_node position_detector_test
```

### Phase 2: View Camera Data
```bash
# View RGB image
ros2 run rqt_image_view rqt_image_view /world_camera/world_camera/image_raw

# View depth image
ros2 run rqt_image_view rqt_image_view /world_camera/world_camera/depth/image_raw

# Check topics
ros2 topic list | grep world_camera
```

### Phase 3: Robot Integration Test
```bash
# Launch UR simulation
ros2 launch ur_simulation_gazebo ur_sim_control.launch.py

# Add camera and objects
ros2 launch ur_vision_system simple_camera_spawn.launch.py
ros2 launch ur_vision_system spawn_random_objects.launch.py

# Run integrated controller
ros2 run main_controller main_controller
```

## 🔧 Troubleshooting

### Common Issues

#### 1. Build Failure
```bash
# Clean rebuild
rm -rf build/ install/ log/
colcon build --symlink-install
```

#### 2. Camera No Image Output
```bash
# Check camera topics
ros2 topic list | grep world_camera

# Check Gazebo plugin
ros2 topic echo /world_camera/world_camera/image_raw --once
```

#### 3. Vision Detection Failure
- Ensure objects are within camera field of view (±0.4m range)
- Check if object colors are supported red/green/blue
- Verify lighting conditions

#### 4. Robot Not Moving
```bash
# Check controller status
ros2 control list_controllers

# Check MoveIt2 services
ros2 service list | grep compute
```

### System Status Check
```bash
# Check node status
ros2 node list

# Monitor joint status
ros2 topic echo /joint_states

# View camera info
ros2 topic echo /world_camera/world_camera/camera_info --once
```

## 🎯 Performance Optimization

### Vision Detection Optimization
- Use recommended work area to improve efficiency
- Adjust color thresholds for different lighting conditions
- Optimize detection frequency to balance accuracy and performance

### Motion Control Optimization
- Set reasonable motion speed percentages
- Configure appropriate motion interval times
- Use multi-seed strategy to improve IK success rate

## 📋 Development Workflow

### Post Code Modification Process
1. Modify code
2. `colcon build --symlink-install`
3. `source install/setup.bash`
4. Restart nodes

### New Feature Development
1. **Add new object detection**: Modify color ranges in `object_detector.py`
2. **Adjust camera parameters**: Modify `robot_common/camera_config.py`
3. **Extend motion functions**: Add new methods in `robot_arm_controller.py`

## 📝 Changelog

### v2.0 - Vision System Integration (Current Version)
- ✅ Complete vision detection system
- ✅ Unified camera configuration management
- ✅ Real-time object position detection
- ✅ Vision-guided motion control
- ✅ Multi-object type support

### v1.0 - Basic Motion Control
- ✅ MoveIt2 integration
- ✅ Intelligent geometry processing
- ✅ Joint space control
- ✅ Workspace boundary checking

## 🤝 Contribution and Support

### Technology Stack
- **ROS2 Humble**
- **Gazebo Classic 11**
- **MoveIt2**
- **OpenCV 4.x**
- **Python 3.10**

### Compatibility
- Ubuntu 22.04 LTS
- UR5e robotic arm
- Robotiq 85 gripper

---

**Note**: This system is designed specifically for UR5e robotic arms, including complete vision detection and intelligent motion control capabilities. If you encounter issues, please check the troubleshooting guide above.