# SD05 OCS2 交接记录

更新时间：2026-07-15

## 目标

将 SD05 默认控制链切换为仓库现有的 SQP-NMPC + WeightedWbc：

```text
SqpMpc -> MPC policy -> WeightedWbc -> joint torque/position/velocity/kp/kd
```

平地站立、直行和转向阶段不启用感知地形模块。感知模块仅用于高程地图、可落脚区域和障碍物约束。

## 用户约束

- ROS 2 运行在宿主机，不使用 Docker。
- 默认不启动 RViz。
- 修改方案必须先说明，再执行修改。
- 只提交当前任务相关的源文件；不要提交 `build/`、`install/`、`log/`。
- 发生新的编译、依赖或行为问题时，先报告原因，不直接盲调控制参数。
- 如果连续 10 分钟没有实质进展，必须主动说明正在等待什么、卡在哪里和下一步选项。

## 仓库状态

### `quadruped_ros2_control`

- 路径：`/home/wl/workspace/legged_control_sim/quadruped_ros2_control`
- 分支：`humble`
- HEAD：`578ddb7 fix(gazebo): 使用本地模型资源并关闭阴影`
- 工作区中与本任务相关、尚未提交的改动：
  - `controllers/ocs2_quadruped_controller/CMakeLists.txt`
  - `controllers/ocs2_quadruped_controller/include/ocs2_quadruped_controller/control/CtrlComponent.h`
  - `controllers/ocs2_quadruped_controller/src/control/CtrlComponent.cpp`

上述改动已获得用户确认：增加 `BUILD_PERCEPTIVE` 选项，默认 `OFF`。关闭时不编译 `convex_plane_decomposition`、`grid_map_sdf` 等感知地形依赖；SQP-NMPC、状态估计、WBC 不受影响。

`build/`、`install/`、`log/`、`scripts/__pycache__/` 均为生成目录，不提交。

### `ocs2`

- 路径：`/home/wl/workspace/legged_control_sim/ocs2`
- 分支：`ros2`
- 本地提交：

```text
8d9621440 fix(ocs2_core): 修复梯形积分的类型命名空间
```

该提交只修改：

```text
ocs2_core/include/ocs2_core/integration/TrapezoidalIntegration.h
```

改动：引入 `<cstddef>`，并将循环变量改为 `std::size_t`。验证后 `ocs2_oc` 及后续多个 OCS2 包可继续构建。

## 已安装的宿主机依赖

### Gazebo Sim / ROS-GZ

已安装并验证：

```text
ros-humble-ros-gz-sim
ros-humble-ros-gz-bridge
libignition-gazebo6
libignition-gazebo6-dev
libignition-gazebo6-plugins
```

当前使用 Gazebo Fortress。`gz_quadruped_hardware` 的 CMake 已支持 `ignition-gazebo6` 回退路径。

### OCS2 数值与动力学依赖

本地下载的固定版本：

```text
/home/wl/workspace/legged_control_sim/.deps/src/blasfeo
ae6e2d1dea015862a09990b95905038a756ffc7d

/home/wl/workspace/legged_control_sim/.deps/src/hpipm
255ffdf38d3a5e2c3285b29568ce65ae286e5faf
```

宿主机 Robotpkg 安装：

```text
robotpkg-pinocchio 3.9.0
robotpkg-coal 3.0.2
```

安装位置：`/opt/openrobots`。

构建 OCS2 时必须将 `/opt/openrobots` 放在旧的私有 Pinocchio 2.5.1 前面：

```bash
source /opt/ros/humble/setup.bash
export PATH=/opt/openrobots/bin:$PATH
export CMAKE_PREFIX_PATH=/opt/openrobots:/home/wl/workspace/legged_control_sim/.deps/install:$CMAKE_PREFIX_PATH
export LD_LIBRARY_PATH=/opt/openrobots/lib:/home/wl/workspace/legged_control_sim/.deps/install/lib:$LD_LIBRARY_PATH
export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:/home/wl/workspace/legged_control_sim/.deps/install/lib/pkgconfig:/opt/ros/humble/lib/x86_64-linux-gnu/pkgconfig:$PKG_CONFIG_PATH
```

验证命令：

```bash
pkg-config --modversion pinocchio
pkg-config --modversion coal
```

预期输出：`3.9.0` 和 `3.0.2`。

## 当前编译状态

构建命令使用本地 BLASFEO/HPIPM，避免 `FetchContent` 再次联网：

```bash
source /opt/ros/humble/setup.bash
export PATH=/opt/openrobots/bin:$PATH
export CMAKE_PREFIX_PATH=/opt/openrobots:/home/wl/workspace/legged_control_sim/.deps/install:$CMAKE_PREFIX_PATH
export LD_LIBRARY_PATH=/opt/openrobots/lib:/home/wl/workspace/legged_control_sim/.deps/install/lib:$LD_LIBRARY_PATH
export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:/home/wl/workspace/legged_control_sim/.deps/install/lib/pkgconfig:/opt/ros/humble/lib/x86_64-linux-gnu/pkgconfig:$PKG_CONFIG_PATH
export MAKEFLAGS=-j2

colcon --log-base /home/wl/workspace/legged_control_sim/quadruped_ros2_control/log build \
  --base-paths /home/wl/workspace/legged_control_sim/ocs2 \
               /home/wl/workspace/legged_control_sim/ocs2_robotic_assets \
               /home/wl/workspace/legged_control_sim/quadruped_ros2_control \
  --build-base /home/wl/workspace/legged_control_sim/quadruped_ros2_control/build \
  --install-base /home/wl/workspace/legged_control_sim/quadruped_ros2_control/install \
  --packages-up-to ocs2_quadruped_controller \
  --symlink-install \
  --executor sequential \
  --event-handlers console_cohesion+ \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DBUILD_PERCEPTIVE=OFF \
    -DFETCHCONTENT_SOURCE_DIR_BLASFEODOWNLOAD=/home/wl/workspace/legged_control_sim/.deps/src/blasfeo \
    -DFETCHCONTENT_SOURCE_DIR_HPIPMDOWNLOAD=/home/wl/workspace/legged_control_sim/.deps/src/hpipm
```

已通过的关键节点：

```text
blasfeo_catkin
hpipm_catkin
ocs2_core
ocs2_oc
ocs2_mpc
ocs2_ddp
ocs2_qp_solver
ocs2_robotic_tools
ocs2_msgs
```

使用 Pinocchio 3.9 后，先前 `FrameTpl::parentJoint` 缺失的问题已消失。

## 当前阻塞点

失败包：`ocs2_pinocchio_interface`。

错误：

```text
fatal error: urdfdom/urdf_parser/urdf_parser.h: No such file or directory
```

原因：OCS2 源码仍使用 ROS 1 风格路径：

```cpp
#include <urdfdom/urdf_parser/urdf_parser.h>
```

当前宿主机 Debian/Ubuntu 开发包的真实路径为：

```text
/usr/include/urdf_parser/urdf_parser.h
```

ROS Humble 还提供兼容路径：

```text
/opt/ros/humble/include/urdfdom/urdf_parser/urdf_parser.h
```

但是 `ocs2_pinocchio_interface/CMakeLists.txt` 未调用 `find_package(urdfdom REQUIRED)`，因此不会正确声明这项依赖。

## 建议的下一步修复

执行前需要再次获得用户确认。修改范围仅在 `ocs2` 仓库：

1. 在 `ocs2_pinocchio/ocs2_pinocchio_interface/src/urdf.cpp` 使用 `__has_include` 兼容两种头文件路径：

```cpp
#if __has_include(<urdfdom/urdf_parser/urdf_parser.h>)
#include <urdfdom/urdf_parser/urdf_parser.h>
#else
#include <urdf_parser/urdf_parser.h>
#endif
```

2. 在 `ocs2_pinocchio/ocs2_pinocchio_interface/CMakeLists.txt` 中：

```cmake
find_package(urdfdom REQUIRED)
```

并使用 `${urdfdom_LIBRARIES}` 代替当前硬编码的 `urdfdom_model`、`urdfdom_world`、`urdfdom_sensor`、`urdfdom_model_state` 链接项。

这个修复不改变 SQP、WBC、URDF 内容或机器人参数，只处理 ROS 1/ROS 2 的 URDFDOM 头文件与链接发现兼容性。

修复后，删除旧缓存并重跑构建：

```bash
rm -rf build/ocs2_pinocchio_interface
rm -rf build/ocs2_centroidal_model
rm -rf build/ocs2_self_collision
rm -rf build/ocs2_sphere_approximation
rm -rf build/ocs2_legged_robot
rm -rf build/ocs2_legged_robot_ros
rm -rf build/ocs2_quadruped_controller
```

之后使用上面的 `colcon build` 命令继续。该修复应单独提交到 `ocs2` 仓库。

## SD05 尚未完成的迁移工作

OCS2 本体可编译后，再进行以下步骤：

1. 为 `sd05_description` 添加 Gazebo Sim 专用 `gazebo.xacro`。
2. 接入 `gz_quadruped_hardware/GazeboSimSystem`。
3. 配置 12 个关节的 `effort/position/velocity/kp/kd` 命令接口。
4. 配置 IMU 和四个足端力传感器。
5. 生成 OCS2 固定读取的 `urdf/robot.urdf`。
6. 新增 `config/ocs2/task.info`、`reference.info`、`gait.info`。
7. 使用 SD05 参数：力矩上限 `34.92 Nm`，初始关节角 `hip=0`、`thigh=0.67`、`calf=-1.3`，初始高度约 `0.355 m`。
8. 新增 SD05 OCS2 启动脚本：清理旧仿真、默认启动 OCS2、默认不启动 RViz。
9. 按顺序验证：控制器加载、原地站立、直行 5 秒、旋转 360 度、旋转 720 度。

测试失败时先记录姿态、接触状态、控制器状态和具体日志，不通过降低速度上限掩盖问题。

## 系统配置说明

为提升 ROS 2 包下载速度，系统 ROS 2 apt 源已切换到清华镜像：

```text
/etc/apt/sources.list.d/ros2.list
```

原文件备份为：

```text
/etc/apt/sources.list.d/ros2.list.codex-backup
```

Robotpkg 源和密钥：

```text
/etc/apt/sources.list.d/robotpkg.list
/etc/apt/keyrings/robotpkg.gpg
```

该文档不包含任何 sudo 密码或其他凭据。
