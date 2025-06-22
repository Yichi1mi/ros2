# Franka Emika Panda Docker化仿真项目指南 (README)

## 1. 项目概述

本项目提供了一个基于Docker的、完整的ROS 2 Humble环境，用于对Franka Emika Panda机械臂（包含其官方夹爪）进行物理仿真和运动规划。

- **核心技术**:
- **Docker**: 提供可移植、可复现的隔离开发环境。
- **ROS 2 Humble**: 机器人操作系统。
- **Gazebo**: 用于进行物理仿真。
- **MoveIt 2**: 用于进行运动规划。
- **工作空间挂载**:
- **本地主机**的 `~/robot_ws` 目录被实时挂载到**Docker容器**内部的 `/ros2_ws` 目录。这意味着两边的文件是实时同步的。

## 2. 首次环境设置 (One-Time Environment Setup)

如果您是第一次使用这个项目，或者您刚刚修改了`Dockerfile`，请遵循以下步骤。

### 2.1 构建Docker镜像 (在Host主机上执行)

此步骤会根据`Dockerfile`创建一个名为`ros2_panda_sim:latest`的镜像，其中包含了所有系统级和ROS基础依赖。

```bash
# 导航到包含Dockerfile的项目根目录 (例如 ~/panda_docker)
docker build -t ros2_panda_sim:latest .
```


# 导航到包含run_sim.sh的目录 (例如 ~/panda_docker)
./run_sim.sh


# 确认您在容器内的 /ros2_ws 目录下
cd /ros2_ws
colcon build --symlink-install 


source install/setup.bash
ros2 launch my_franka_project fr3_moveit_only.launch.py

# 2. 测试MoveIt+Gazebo协同仿真：

source install/setup.bash
ros2 launch my_franka_project fr3_moveit_gazebo.launch.py


第三步：运行你的代码

# 新开第二个终端连接到容器
docker exec -it panda_simulation_env bash

cd /ros2_ws
colcon build --symlink-install



# 运行主控制器
source install/setup.bash
ros2 run main_controller main_controller