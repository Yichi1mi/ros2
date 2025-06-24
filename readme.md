# UR5e + Robotiq 85 Robot Control System

## Project Overview
This is a ROS2-based UR5e robotic arm control system that integrates MoveIt2 motion planning, Gazebo physics simulation, and custom controllers. The system features modular design and supports precise control in both joint space and Cartesian space.

## Project Structure
```
robot_ws/src/
├── my_robot_controller/           # Main controller package
│   ├── main_controller/          # Main controller node
│   ├── move_node/               # Motion control node
│   ├── robot_common/            # Common library (geometry processing, state management)
│   └── vision_node/             # Vision control node
├── ur_robotiq_moveit_config/     # MoveIt2 configuration
├── ur_description_custom/        # Custom URDF description
└── readme.md                    # This document
```

## Quick Start

### 1. Build Workspace
```bash
cd /home/xichen/robot_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 2. Launch Simulation Environment
```bash
# Launch complete simulation environment (Gazebo + MoveIt2)
ros2 launch ur_simulation_gazebo ur_sim_moveit.launch.py
```

### 3. Run Controller (New Terminal)
```bash
cd /home/xichen/robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# Launch main controller
ros2 run main_controller main_controller
```

## System Architecture

### Core Components

#### 1. **Robotic Arm Geometry Processing Module** (`robot_common/arm_geometry.py`)
- **Intelligent Shape Detection**: Based on Link2 and Link3 ground angle comparison to detect "concave down" configuration
- **Precise Geometric Transformation**: Calculate alpha angle of J4-J2-J3 at J2 point for elbow-up/elbow-down conversion
- **Link4 Angle Preservation**: Maintain Link4 ground angle during transformation
- **360° Joint Handling**: Automatically adjust J4 to equivalent angle range
- **Joint Limit Management**: Complete UR5 joint limits (J2: -180°~0°, J4: -270°~90°, etc.)
- **Multi-seed Strategy**: Improve inverse kinematics solution success rate

#### 2. **Motion Controllers** (`move_node/`)
- **RobotArmController**: Simplified API interface, supports joint and Cartesian motion
- **PositionController**: MoveIt2 Cartesian space controller
- **JointController**: Joint space trajectory controller

#### 3. **State Management** (`robot_common/robot_state.py`)
- Real-time joint state monitoring
- Forward kinematics calculation
- Trajectory displacement calculation

### Key Features

#### Intelligent Angle Correction
- **Geometric Shape Detection**: Detect "concave down" configuration by comparing Link2 and Link3 ground angles
- **Precise Configuration Conversion**: Geometric transformation based on J4-J2-J3 triangle angle for elbow-up ↔ elbow-down conversion
- **Link4 Angle Preservation**: Maintain Link4 absolute ground angle during transformation
- **360° Joint Optimization**: Automatically adjust J4 to reasonable equivalent angle range
- **Detailed Debug Information**: Complete geometric calculation process and angle transformation logs

#### Safe Workspace
- **Cylindrical Workspace**: Safe boundary definition based on UR5e specifications
- **Real-time Boundary Check**: Validate target position safety before movement
- **Recommended Work Zone**: Distinguish between safe zone and recommended efficient zone

## Usage Examples

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

### Predefined Positions
```python
# Move to safe initial position
robot.move_to_initial_position()

# Move to work home position
robot.move_to_home()
```

## Technical Details

### Geometric Algorithms
The system implements precise transformations based on actual geometric relationships:

1. **Shape Detection**: `Link2 angle = min(|j2|, |j2+π|)`, `Link3 angle = min(|j2+j3|, |j2+j3+π|)`
2. **Alpha Angle Calculation**: Angle of J4-J2-J3 triangle at J2 point, using cosine law and UR5 parameters (L2=0.425m, L3=0.39225m)
3. **Configuration Conversion Formula**:
   - J3 flip: `new_j3 = -j3`  
   - J2 adjustment: `new_j2 = j2 ± 2*alpha`
   - J4 preservation: `new_j4 = (j2+j3+j4) - new_j2 - new_j3`
   - J4 range adjustment: Automatic 360° equivalent angle handling

### Debug Features
- Link2 and Link3 ground angle calculation logs
- Detailed alpha angle calculation process for J4-J2-J3 triangle
- Link4 angle preservation verification information
- J4 equivalent angle adjustment process
- Geometric shape detection process tracking
- Joint limit violation warnings and automatic correction
- Workspace boundary check notifications

## Troubleshooting

### Common Issues

#### 1. Build Failures
```bash
# Clean and rebuild
rm -rf build/ install/ log/
colcon build --symlink-install
```

#### 2. MoveIt2 Service Unavailable
```bash
# Check MoveIt2 node status
ros2 node list | grep move
ros2 service list | grep compute
```

#### 3. Joint Angle Limits Exceeded
- Check if J2 angle is within -180° to 0° range
- Verify geometric shape detection logic triggers correctly

#### 4. IK Solution Failures
- Target position may exceed workspace
- Try different initial joint configurations
- Check end-effector orientation constraints

### Verify System Status
```bash
# Check controller status
ros2 control list_controllers

# View active topics
ros2 topic list

# Monitor joint states
ros2 topic echo /joint_states
```

## Development Notes

### Workflow After Code Changes
1. Modify code
2. `colcon build --symlink-install`
3. `source install/setup.bash`
4. Restart nodes

### Geometric Algorithm Debugging
- Review Link2 and Link3 ground angle calculation process
- Verify correctness of J4-J2-J3 triangle alpha angle calculation
- Check if Link4 angle preservation constraint is satisfied
- Confirm reasonableness of J4 equivalent angle adjustment
- Test geometric transformation effects with different seed configurations

### Performance Optimization
- Use recommended work zone for improved efficiency
- Set reasonable motion speed percentage
- Configure appropriate pause time between movements