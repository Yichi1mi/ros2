# UR5e + Robotiq 85 仿真启动指南

## 目录说明
- `ur5_docker/` - Docker环境配置
- `robot_ws/` - ROS2工作空间（映射到Docker内部）

## 启动步骤

### 1. 启动Docker容器（在host bash中执行）
```bash
cd /home/xichen/ur5_docker
./run_sim.sh
```

### 2. 构建工作空间（在Docker容器内执行，仅首次或修改后需要）
```bash
# 在Docker容器内
source /root/ros2_ws/install/setup.bash
colcon build --symlink-install
```

### 3. 选择仿真模式（在Docker容器内执行）

#### 选项A: 仅MoveIt仿真（推荐用于开发测试）
```bash
# 在Docker容器内
source install/setup.bash
ros2 launch ur_robotiq_moveit_config demo.launch.py
```

#### 选项B: Gazebo + MoveIt完整仿真
```bash
# 在Docker容器内
source install/setup.bash
ros2 launch ur_robotiq_moveit_config ur5e_robotiq_gazebo_moveit.launch.py
```

#### 选项C: 仅查看机器人模型
```bash
# 在Docker容器内
source install/setup.bash
ros2 launch ur_description_custom test_ur5e_robotiq.launch.py
```

### 4. 运行控制器（可选，在新终端）
```bash
# 在host bash中打开新终端
docker exec -it ur_simulation_env bash

# 在新Docker终端内
cd /home/xichen/robot_ws
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
source install/setup.bash
ros2 run main_controller main_controller
```

## 推荐使用流程
1. **开发调试**: 使用选项A（仅MoveIt）- 启动快，资源占用少
2. **物理仿真**: 使用选项B（仅Gazebo）- 单纯的物理仿真环境
3. **完整测试**: 使用选项C（Gazebo+MoveIt）- 包含物理仿真和运动规划
4. **模型检查**: 使用选项D（仅显示）- 快速查看机器人模型

## 注意事项

### 构建工作空间
- **首次使用** 或 **修改launch文件/配置后** 必须执行 `colcon build`
- 构建成功后需要重新 `source install/setup.bash`
- 如果遇到问题，可以尝试 `colcon build --packages-select ur_robotiq_moveit_config`

### 环境设置
- 每次进入Docker容器都需要执行三个source命令：
  ```bash
  source /opt/ros/humble/setup.bash
  source /root/ros2_ws/install/setup.bash  
  source install/setup.bash
  ```

### 重新启动流程
1. 如果修改了代码/配置：退出容器 → 重启Docker → 构建 → 启动
2. 如果只是重启：退出容器 → 重启Docker → 直接启动

### 验证启动成功
- **MoveIt demo**: 应该看到RViz界面，机械臂上有交互式标记（小圆球）
- **控制器状态**: 新终端执行 `ros2 control list_controllers` 应显示active状态