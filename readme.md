# UR5e + Robotiq 85 机器人控制系统

## 项目概述
这是一个基于ROS2的UR5e机械臂控制系统，集成了MoveIt2运动规划、Gazebo物理仿真和自定义控制器。系统采用模块化设计，支持关节空间和笛卡尔空间的精确控制。

## 项目结构
```
robot_ws/src/
├── my_robot_controller/           # 主控制器包
│   ├── main_controller/          # 主控制器节点
│   ├── move_node/               # 运动控制节点
│   ├── robot_common/            # 公共库（几何处理、状态管理）
│   └── vision_node/             # 视觉控制节点
├── ur_robotiq_moveit_config/     # MoveIt2配置
├── ur_description_custom/        # 自定义URDF描述
└── readme.md                    # 本文档
```

## 快速启动

### 1. 构建工作空间
```bash
cd /home/xichen/robot_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 2. 启动仿真环境
```bash
# 启动完整仿真环境（Gazebo + MoveIt2）
ros2 launch ur_simulation_gazebo ur_sim_moveit.launch.py
```

### 3. 运行控制器（新终端）
```bash
cd /home/xichen/robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# 启动主控制器
ros2 run main_controller main_controller
```

## 系统架构

### 核心组件

#### 1. **机械臂几何处理模块** (`robot_common/arm_geometry.py`)
- **智能形状判断**：基于Link2和Link3相对地面角度比较，判断机械臂是否"向下凹"
- **精确几何变换**：计算J4-J2-J3在J2点的alpha角，实现肘上/肘下配置转换
- **Link4角度保持**：变换过程中保持Link4相对地面角度不变
- **360°关节处理**：自动调整J4到等效角度范围内
- **关节限制管理**：UR5完整的关节限制 (J2: -180°~0°, J4: -270°~90°等)
- **多种子值策略**：提高逆运动学求解成功率

#### 2. **运动控制器** (`move_node/`)
- **RobotArmController**: 简化的API接口，支持关节和笛卡尔运动
- **PositionController**: MoveIt2笛卡尔空间控制器
- **JointController**: 关节空间轨迹控制器

#### 3. **状态管理** (`robot_common/robot_state.py`)
- 实时关节状态监控
- 前向运动学计算
- 轨迹位移计算

### 关键特性

#### 智能角度修正
- **几何形状判断**: 通过比较Link2和Link3相对地面角度，检测"向下凹"配置
- **精确配置转换**: 基于J4-J2-J3三点角度的几何变换，实现肘上↔肘下转换  
- **Link4角度保持**: 变换过程中保持Link4相对地面的绝对角度不变
- **360°关节优化**: 自动调整J4到合理的等效角度范围
- **详细调试信息**: 完整的几何计算过程和角度变换日志

#### 安全工作空间
- **圆柱形工作空间**: 基于UR5e规格的安全边界定义
- **实时边界检查**: 运动前验证目标位置安全性
- **推荐工作区域**: 区分安全区域和推荐高效区域

## 使用示例

### 基本运动控制
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

### 预定义位置
```python
# 移动到安全的初始位置
robot.move_to_initial_position()

# 移动到工作Home位置
robot.move_to_home()
```

## 技术细节

### 几何算法
系统实现了基于实际几何关系的精确变换：

1. **形状判断**: `Link2角度 = min(|j2|, |j2+π|)`, `Link3角度 = min(|j2+j3|, |j2+j3+π|)`
2. **Alpha角计算**: J4-J2-J3三点在J2处的角度，使用余弦定理和UR5参数(L2=0.425m, L3=0.39225m)
3. **配置转换公式**:
   - J3翻转: `new_j3 = -j3`  
   - J2调整: `new_j2 = j2 ± 2*alpha`
   - J4保持: `new_j4 = (j2+j3+j4) - new_j2 - new_j3`
   - J4范围调整: 自动处理360°等效角度

### 调试功能
- Link2和Link3相对地面角度计算日志
- J4-J2-J3三点alpha角的详细计算过程
- Link4角度保持验证信息
- J4等效角度调整过程
- 几何形状判断过程追踪
- 关节限制违规警告和自动修正
- 工作空间边界检查提示

## 故障排除

### 常见问题

#### 1. 构建失败
```bash
# 清理并重新构建
rm -rf build/ install/ log/
colcon build --symlink-install
```

#### 2. MoveIt2服务不可用
```bash
# 检查MoveIt2节点状态
ros2 node list | grep move
ros2 service list | grep compute
```

#### 3. 关节角度超限
- 检查J2角度是否在-180°到0°范围内
- 确认几何形状判断逻辑是否正确触发

#### 4. IK求解失败
- 目标位置可能超出工作空间
- 尝试不同的初始关节配置
- 检查末端执行器方向约束

### 验证系统状态
```bash
# 检查控制器状态
ros2 control list_controllers

# 查看活跃话题
ros2 topic list

# 监控关节状态
ros2 topic echo /joint_states
```

## 开发注意事项

### 代码修改后的流程
1. 修改代码
2. `colcon build --symlink-install`
3. `source install/setup.bash`
4. 重启节点

### 几何算法调试
- 查看Link2和Link3相对地面角度的计算过程
- 验证J4-J2-J3三点alpha角计算的正确性
- 检查Link4角度保持约束是否满足
- 确认J4等效角度调整的合理性
- 测试不同种子配置的几何变换效果

### 性能优化
- 使用推荐工作区域提高效率
- 合理设置运动速度百分比
- 配置适当的运动间暂停时间