# UR5e 机器人视觉控制系统

**语言**: [English](readme.md) | [中文](readme.zh.md)

## 快速启动

### 1. 安装依赖
```bash
sudo apt install ros-humble-ur-simulation-gazebo ros-humble-cv-bridge python3-opencv ros-humble-gazebo-ros-pkgs
```

### 2. 构建工作空间
```bash
cd /home/xichen/robot_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 3. 启动系统

#### 步骤1：启动UR机器人仿真
```bash
# 终端1：启动UR仿真和MoveIt2
ros2 launch ur_simulation_gazebo ur_sim_moveit.launch.py
```

#### 步骤2：添加相机
```bash
# 终端2：添加世界相机
ros2 launch ur_vision_system simple_camera_spawn.launch.py
```

#### 步骤3：添加物体
```bash
# 终端3：生成测试物体（红色圆柱体、绿色盒子、蓝色三棱柱）
ros2 launch ur_vision_system spawn_random_objects.launch.py
```

#### 步骤4：打开相机视图
```bash
# 终端4：打开相机图像查看器
ros2 run rqt_image_view rqt_image_view /world_camera/world_camera/image_raw
```

#### 步骤5：运行控制器
```bash
# 终端5：启动主控制器
ros2 run main_controller main_controller
```

### 4. 使用控制器菜单
```
=== 主控制器菜单 ===
1. 初始化机器人       # 移动到初始位置
2. 物体检测           # 检测物体并显示位置（在home位置之前运行）
3. 移动到home位置      # 移动到home位置  
4. 移动到物体         # 按从近到远顺序访问物体
5. 退出              # 退出程序
```

**使用流程**：按顺序运行选项 1 → 2 → 3 → 4。

## 系统组件

- **主控制器**：集成视觉和机器人控制
- **视觉系统**：使用相机检测红/绿/蓝色物体
- **机器人控制**：UR5e机械臂运动控制（MoveIt2）
- **相机配置**：`robot_common/camera_config.py`中的统一相机参数

## 故障排除

### 构建问题
```bash
rm -rf build/ install/ log/
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

### 无相机图像
```bash
ros2 topic list | grep world_camera
ros2 topic echo /world_camera/world_camera/image_raw --once
```

### 机器人不运动
```bash
ros2 control list_controllers
ros2 service list | grep compute
```

---

**技术栈**：ROS2 Humble, Gazebo, MoveIt2, OpenCV, Python 3.10