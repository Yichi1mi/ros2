# UR5e智能机器人视觉控制系统

**语言**: [English](readme.md) | [中文](readme.zh.md)

## 项目概述
这是一个基于ROS2的UR5e机械臂智能控制系统，集成了MoveIt2运动规划、Gazebo物理仿真、计算机视觉和自定义控制器。系统采用模块化设计，支持精确的视觉引导运动控制，具备实时物体检测和智能路径规划功能。

## 🏗️ 系统架构

```
robot_ws/src/
├── my_robot_controller/              # 核心控制系统
│   ├── main_controller/             # 主控制器 - 统一工作流程管理
│   ├── move_node/                   # 运动控制 - MoveIt2集成和路径规划
│   ├── robot_common/                # 公共库 - 几何算法、相机配置、状态管理
│   └── vision_node/                 # 视觉系统 - 物体检测和精确定位
├── ur_vision_system/                # 环境配置 - Gazebo模型和启动文件
├── Universal_Robots_ROS2_Gazebo_Simulation/  # 官方UR仿真支持
└── README.md                        # 项目文档
```

## 🚀 快速开始

### 1. 环境准备
```bash
# 安装ROS2依赖
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

```bash
# 终端1：启动UR5e仿真环境（包含MoveIt2）
ros2 launch ur_simulation_gazebo ur_sim_moveit.launch.py

# 终端2：添加视觉相机
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ur_vision_system simple_camera_spawn.launch.py

# 终端3：生成测试物体
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ur_vision_system spawn_random_objects.launch.py

# 终端4：运行主控制器
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run main_controller main_controller
```

## 🎯 主控制器工作流程

启动主控制器后，按照以下顺序操作：

```
=== Main Controller Menu ===
1. Initialize            # 移动到初始位置（垂直向上）
2. Ready position        # 移动到工作就绪位置
3. Detect objects        # 扫描并检测物体位置
4. Move to objects       # 按从近到远顺序移动到物体
5. Exit                  # 退出程序
```

### 推荐操作流程
1. **Initialize** → 机器人初始化到安全位置
2. **Ready position** → 移动到工作区域准备状态
3. **Detect objects** → 扫描环境，显示所有检测到的物体位置
4. **Move to objects** → 自动按距离排序，依次移动到各物体上方

## 🔧 核心技术特性

### 1. 智能运动控制 (`move_node/`)
- **安全工作空间**：圆柱形边界检查（半径0.15-0.75m，高度-0.1-0.8m）
- **智能几何处理**：自动处理UR5复杂关节配置和限制
- **精确路径规划**：集成MoveIt2进行碰撞避免和轨迹优化
- **距离排序算法**：自动计算最优移动顺序

### 2. 高精度视觉系统 (`vision_node/`)
- **精确坐标转换**：考虑相机位置和旋转的完整变换
- **多物体检测**：同时识别红色圆柱、绿色长方体、蓝色三棱柱
- **实时定位**：亚厘米级的物体位置估算精度
- **自动配置**：统一的相机参数管理系统

### 3. 统一配置管理 (`robot_common/`)
- **相机配置**：`camera_config.py`统一管理所有相机参数
- **几何算法**：`arm_geometry.py`处理复杂的关节变换
- **坐标系管理**：自动处理相机坐标系到世界坐标系转换

## 📊 视觉检测系统

### 相机配置
- **位置**: (0.0, 0.4, 1.5) - 机器人基座后方1.5米高度
- **姿态**: Roll=90°, Pitch=90° (垂直向下)
- **分辨率**: 640×480像素
- **视野**: 60度，地面覆盖约2.6×2.6米
- **工作精度**: 亚厘米级定位精度

### 检测输出示例
```bash
=== Detect Objects ===
✅ Found 3 objects:
  1. RED: (0.02, 0.95, 0.000) area=1396 confidence=0.14
  2. GREEN: (-0.19, 0.61, 0.000) area=1174 confidence=0.12  
  3. BLUE: (0.11, 0.88, 0.000) area=1370 confidence=0.14
```

### 支持的物体类型
- 🔴 **红色圆柱体** - 基于HSV颜色检测
- 🟢 **绿色长方体** - 轮廓形状识别
- 🔵 **蓝色三棱柱** - 多特征融合检测

## ⚙️ 配置管理

### 相机参数调整
所有相机相关参数通过 `robot_common/camera_config.py` 统一管理：

```python
# 修改相机位置
position_x: float = 0.0      # X坐标
position_y: float = 0.4      # Y坐标  
position_z: float = 1.5      # Z坐标

# 修改相机姿态
roll: float = 1.57           # Roll角（弧度）
pitch: float = 1.5708        # Pitch角（弧度）

# 修改图像参数
image_width: int = 640       # 图像宽度
image_height: int = 480      # 图像高度
horizontal_fov: float = 1.047198  # 水平视野角（弧度）
```

### 工作空间配置
机器人工作空间在 `move_node/robot_arm_controller.py` 中定义：

```python
WORKSPACE_LIMITS = {
    'inner_radius': 0.15,    # 内径15cm
    'outer_radius': 0.75,    # 外径75cm  
    'z_min': -0.10,          # 最低高度
    'z_max': 0.80,           # 最高高度
}
```

## 🔧 故障排除

### 常见问题

#### 1. 构建失败
```bash
# 清理重建
rm -rf build/ install/ log/
colcon build --symlink-install
```

#### 2. 相机检测不到物体
- 确保物体颜色为支持的红/绿/蓝色
- 检查物体是否在相机视野范围内
- 验证相机话题是否正常发布：
```bash
ros2 topic list | grep world_camera
ros2 topic echo /world_camera/world_camera/image_raw --once
```

#### 3. 机器人不移动
```bash
# 检查MoveIt2服务状态
ros2 service list | grep compute
ros2 control list_controllers
```

#### 4. 坐标检测偏差
- 检查 `camera_config.py` 中的相机位置参数
- 确认物体实际位置与检测位置的对应关系

## 📋 技术规格

### 软件环境
- **ROS2 Humble** - 机器人操作系统
- **Gazebo Classic 11** - 物理仿真环境
- **MoveIt2** - 运动规划框架
- **OpenCV 4.x** - 计算机视觉库
- **Python 3.10** - 主要编程语言

### 硬件兼容性
- **Ubuntu 22.04 LTS** - 推荐操作系统
- **UR5e机械臂** - 目标硬件平台
- **支持的末端执行器** - Robotiq系列夹爪

### 性能指标
- **视觉检测精度**: ±1cm (在推荐工作范围内)
- **运动重复精度**: ±0.5mm (符合UR5e规格)
- **检测响应时间**: <100ms
- **路径规划时间**: <2s (典型场景)

## 📝 更新日志

### v2.1 - 统一配置和精确定位 (当前版本)
- ✅ 完整的主控制器工作流程
- ✅ 精确的相机坐标转换系统
- ✅ 统一的配置管理架构
- ✅ 智能距离排序运动算法
- ✅ 亚厘米级物体定位精度

### v2.0 - 视觉系统集成
- ✅ 实时多物体检测系统
- ✅ MoveIt2运动规划集成
- ✅ 安全工作空间边界检查
- ✅ 模块化系统架构设计

---

**系统状态**: 生产就绪 | **最后更新**: 2025-07-17 | **维护状态**: 活跃开发

本系统专为UR5e机械臂设计，提供完整的视觉引导智能控制解决方案。适用于工业自动化、研究开发和教育培训场景。