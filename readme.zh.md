# UR5e智能机器人视觉控制系统

**语言**: [English](readme.md) | [中文](readme.zh.md)

## 项目概述
这是一个基于ROS2的UR5e机械臂智能控制系统，集成了MoveIt2运动规划、Gazebo物理仿真、计算机视觉和自定义控制器。系统采用模块化设计，支持关节空间和笛卡尔空间的精确控制，具备实时物体检测和自动抓取功能。

## 🏗️ 系统架构

```
robot_ws/src/
├── my_robot_controller/              # 核心控制系统
│   ├── main_controller/             # 主控制器节点 - 集成视觉和运动
│   ├── move_node/                   # 运动控制节点 - MoveIt2接口
│   ├── robot_common/                # 通用库 - 几何处理、状态管理、相机配置
│   └── vision_node/                 # 视觉控制节点 - 物体检测和位置估算
├── ur_vision_system/                # 环境配置包 - Gazebo相机模型
├── Universal_Robots_ROS2_Gazebo_Simulation/  # 官方UR仿真
└── README.md                        # 本文档
```

## 🚀 快速开始

### 1. 环境准备
```bash
# 安装依赖
sudo apt install ros-humble-ur-simulation-gazebo
sudo apt install ros-humble-cv-bridge python3-opencv
sudo apt install ros-humble-gazebo-ros-pkgs
```

### 2. 构建工作空间
```bash
cd /home/xichen/robot_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 3. 启动完整系统

#### 选项A：基础仿真测试
```bash
# 终端1：启动空Gazebo环境
ros2 launch ur_vision_system empty_gazebo.launch.py

# 终端2：添加相机
ros2 launch ur_vision_system simple_camera_spawn.launch.py

# 终端3：生成随机测试物体
ros2 launch ur_vision_system spawn_random_objects.launch.py

# 终端4：运行位置检测测试
ros2 run vision_node position_detector_test
```

#### 选项B：完整机器人系统
```bash
# 终端1：启动UR仿真环境
ros2 launch ur_simulation_gazebo ur_sim_moveit.launch.py

# 终端2：添加视觉相机
ros2 launch ur_vision_system simple_camera_spawn.launch.py

# 终端3：运行主控制器
ros2 run main_controller main_controller
```

## 🎯 核心功能特性

### 1. 智能几何处理 (`robot_common/arm_geometry.py`)
- **智能形状检测**：基于Link2和Link3地面角度比较检测"凹向下"配置
- **精确几何变换**：计算J4-J2-J3三角形在J2点的alpha角进行肘上/肘下转换
- **Link4角度保持**：变换过程中保持Link4地面角度
- **360°关节处理**：自动调整J4到等效角度范围
- **关节限制管理**：完整UR5关节限制处理
- **多种子策略**：提高逆运动学解算成功率

### 2. 计算机视觉系统 (`vision_node/`)
- **实时物体检测**：基于HSV颜色空间的多物体检测
- **精确位置估算**：像素坐标到世界坐标的精确转换
- **支持物体类型**：红色圆柱、绿色长方体、蓝色三棱柱
- **自动相机配置**：统一的相机参数管理系统
- **高精度检测**：垂直向下相机，覆盖±0.4m工作区域

### 3. 运动控制器 (`move_node/`)
- **RobotArmController**：简化API接口，支持关节和笛卡尔运动
- **PositionController**：MoveIt2笛卡尔空间控制器
- **JointController**：关节空间轨迹控制器
- **智能避障**：基于工作空间边界的安全检查

### 4. 环境配置 (`ur_vision_system/`)
- **相机模型管理**：SDF格式的深度相机模型
- **测试物体生成**：多种形状和颜色的测试物体
- **统一配置架构**：`camera_config.py`实现全局参数管理

## 📊 视觉检测系统

### 当前配置 ✅ WORKING
- **相机位置**: (0, 0, 1.5) - 原点正上方1.5米
- **朝向**: 垂直向下 (pitch=90度)
- **分辨率**: 640x480像素
- **视野**: 60度，覆盖2.6m x 2.6m地面区域
- **工作范围**: ±0.4米（机械臂工作空间内）

### 实时检测输出
```bash
# 典型输出示例
Detected 3 objects: red: (0.20, -0.17) conf:0.85 | green: (-0.02, -0.35) conf:0.92 | blue: (-0.32, -0.11) conf:0.78
```

### 支持的物体颜色
- 🔴 **红色** (red) - 圆柱体
- 🟢 **绿色** (green) - 长方体  
- 🔵 **蓝色** (blue) - 三棱柱

## 🔧 使用API示例

### 基础运动控制
```python
from move_node.robot_arm_controller import RobotArmController

# 创建控制器
robot = RobotArmController()

# 关节空间运动
robot.move_to_joint_positions(0.0, -1.57, 0.0, -1.57, 0.0, 0.0)

# 笛卡尔空间运动
robot.move_to_position(0.3, 0.2, 0.5, 0.0, 0.0, 0.0, 1.0)

# 相对运动
robot.move_relative(0.1, 0.0, 0.0)  # X方向移动10cm
```

### 视觉检测API
```python
from vision_node.vision_api import VisionAPI

# 创建视觉API
vision = VisionAPI()
vision.initialize()

# 获取所有检测到的物体位置
positions = vision.get_object_positions()

# 获取最大物体位置
largest = vision.get_largest_object_position()

# 获取特定颜色物体
red_objects = vision.get_largest_object_position(color_filter='red')
```

### 集成控制示例
```python
# 视觉引导的抓取
vision = VisionAPI()
robot = RobotArmController()

# 检测物体
target = vision.get_largest_object_position(color_filter='red')

if target:
    # 移动到物体上方
    robot.move_to_position(target.x, target.y, target.z + 0.1, 0, 0, 0, 1)
    
    # 下降抓取
    robot.move_relative(0, 0, -0.05)
```

## 🎮 交互式控制

主控制器提供交互式菜单：
```
=== Main Controller Menu ===
1. Test basic robot functions      # 测试基本机器人功能
2. Test vision-robot integration   # 测试视觉+机器人集成
3. Quick vision test (check objects) # 快速检测物体
4. Exit                           # 退出
```

## 🛠️ 技术细节

### 几何算法
系统基于实际几何关系实现精确变换：

1. **形状检测**: `Link2角度 = min(|j2|, |j2+π|)`，`Link3角度 = min(|j2+j3|, |j2+j3+π|)`
2. **Alpha角计算**: J4-J2-J3三角形在J2点的角度，使用余弦定理和UR5参数
3. **配置转换公式**:
   - J3翻转: `new_j3 = -j3`
   - J2调整: `new_j2 = j2 ± 2*alpha`
   - J4保持: `new_j4 = (j2+j3+j4) - new_j2 - new_j3`

### 视觉坐标转换
专为垂直向下相机设计的直接映射：
```python
# 计算地面覆盖范围
ground_coverage = 2 * camera_height * tan(fov/2)
pixel_to_meter_ratio = ground_coverage / image_width

# 像素到世界坐标转换
world_x = (pixel_x - center_x) * pixel_to_meter_ratio
world_y = (pixel_y - center_y) * pixel_to_meter_ratio
```

## 📖 测试指南

### 阶段1：基础深度相机功能验证
```bash
# 启动空Gazebo
ros2 launch ur_vision_system empty_gazebo.launch.py

# 添加相机
ros2 launch ur_vision_system simple_camera_spawn.launch.py

# 添加测试物体
ros2 launch ur_vision_system spawn_random_objects.launch.py

# 验证检测
ros2 run vision_node position_detector_test
```

### 阶段2：查看相机数据
```bash
# 查看RGB图像
ros2 run rqt_image_view rqt_image_view /world_camera/world_camera/image_raw

# 查看深度图像
ros2 run rqt_image_view rqt_image_view /world_camera/world_camera/depth/image_raw

# 检查话题
ros2 topic list | grep world_camera
```

### 阶段3：机器人集成测试
```bash
# 启动UR仿真
ros2 launch ur_simulation_gazebo ur_sim_control.launch.py

# 添加相机和物体
ros2 launch ur_vision_system simple_camera_spawn.launch.py
ros2 launch ur_vision_system spawn_random_objects.launch.py

# 运行集成控制器
ros2 run main_controller main_controller
```

## 🔧 故障排除

### 常见问题

#### 1. 构建失败
```bash
# 清理重建
rm -rf build/ install/ log/
colcon build --symlink-install
```

#### 2. 相机无图像输出
```bash
# 检查相机话题
ros2 topic list | grep world_camera

# 检查Gazebo插件
ros2 topic echo /world_camera/world_camera/image_raw --once
```

#### 3. 视觉检测失败
- 确保物体在相机视野内（±0.4m范围）
- 检查物体颜色是否为支持的红/绿/蓝
- 验证光照条件

#### 4. 机器人不移动
```bash
# 检查控制器状态
ros2 control list_controllers

# 检查MoveIt2服务
ros2 service list | grep compute
```

### 系统状态检查
```bash
# 检查节点状态
ros2 node list

# 监控关节状态
ros2 topic echo /joint_states

# 查看相机信息
ros2 topic echo /world_camera/world_camera/camera_info --once
```

## 🎯 性能优化

### 视觉检测优化
- 使用推荐工作区域提高效率
- 调整颜色阈值以适应不同光照
- 优化检测频率平衡精度和性能

### 运动控制优化
- 设置合理运动速度百分比
- 配置适当运动间隔时间
- 使用多种子策略提高IK成功率

## 📋 开发工作流

### 代码修改后的流程
1. 修改代码
2. `colcon build --symlink-install`
3. `source install/setup.bash`
4. 重启节点

### 新功能开发
1. **添加新物体检测**：修改`object_detector.py`中的颜色范围
2. **调整相机参数**：修改`robot_common/camera_config.py`
3. **扩展运动功能**：在`robot_arm_controller.py`中添加新方法

## 📝 更新日志

### v2.0 - 视觉系统集成 (当前版本)
- ✅ 完整的视觉检测系统
- ✅ 统一相机配置管理
- ✅ 实时物体位置检测
- ✅ 视觉引导运动控制
- ✅ 多物体类型支持

### v1.0 - 基础运动控制
- ✅ MoveIt2集成
- ✅ 智能几何处理
- ✅ 关节空间控制
- ✅ 工作空间边界检查

## 🤝 贡献和支持

### 技术栈
- **ROS2 Humble**
- **Gazebo Classic 11**
- **MoveIt2**
- **OpenCV 4.x**
- **Python 3.10**

### 兼容性
- Ubuntu 22.04 LTS
- UR5e机械臂
- Robotiq 85夹爪

---

**注意**: 本系统专为UR5e机械臂设计，包含完整的视觉检测和智能运动控制功能。如有问题，请检查上述故障排除指南。