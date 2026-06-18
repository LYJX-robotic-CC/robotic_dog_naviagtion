# FAST_LIO_LOCALIZATION（scan context） 使用手册

## 1. 项目当前实现了什么

这个仓库当前不是一个“纯 FAST-LIO 官方镜像”，而是一套围绕 FAST-LIO 里程计结果做地图重定位的 ROS 工程。

按现有代码实现，核心链路是：

1. `fastlio_mapping1` 输出实时点云与里程计。
2. `global_localization` 订阅实时扫描和里程计，在先验点云地图上做匹配。
3. `transform_fusion` 将 `/map_to_odom` 和 FAST-LIO 里程计融合，发布最终定位结果 `/localization`。
4. `base_link` 再根据 `/localization` 补发 `map -> base_link` 的 TF。

也就是说，这个仓库的重点不是“建图算法说明”，而是“如何把 FAST-LIO 的局部里程计接到一个点云地图重定位系统上”。

## 2. 代码里真实使用的话题

下面这些话题是按源码实际写死或默认使用的，不是泛化推测。

### 2.1 `global_localization` 输入

- `/map`
  先验地图点云，由 `pcl_ros/pcd_to_pointcloud` 发布

- `/cloud_registered`
  当前配准后的扫描点云

- `/Odometry`
  FAST-LIO 当前里程计

- `/initialpose`
  手动初始化位姿，消息类型为 `geometry_msgs/PoseWithCovarianceStamped`

### 2.2 `global_localization` 输出

- `/cur_scan_in_map`
  当前扫描点云消息

- `/submap`
  当前参与匹配的局部子图

- `/map_to_odom`
  全局定位估计出的 `map -> odom`

- `/localization_valid`
  当前定位结果是否可信

- `/fast_lio_reset`
  当定位失效且里程计仍明显运动时触发的重置信号

### 2.3 `transform_fusion` 输入输出

输入：

- `/Odometry`
- `/map_to_odom`
- `/localization_valid`

输出：

- `/localization`

同时发布 TF：

- `map -> camera_init`

### 2.4 `base_link` 额外处理

`base_link.cpp` 会订阅：

- `/localization`

然后发布：

- `map -> base_link`

注意：这里代码里写死了一个雷达到 `base_link` 的偏移：

```cpp
constexpr double LIDAR_OFFSET_X = 0.45;
constexpr double LIDAR_OFFSET_Y = 0.0;
constexpr double LIDAR_OFFSET_Z = 0.0;
```

如果你的车体中心和雷达安装位置不是这个关系，需要改这里。

## 3. 当前工程的节点和可执行文件

按 `CMakeLists.txt`，当前实际会生成这些可执行文件：

- `fastlio_mapping1`
- `global_localization`
- `transform_fusion`
- `publish_initial_pose`
- `base_forward_arrow`
- `base_link`

这里要特别注意一个现实问题：

部分 launch 文件里写的是：

- `type="fastlio_mapping"`

但 `CMakeLists.txt` 实际编译出来的是：

- `fastlio_mapping1`

所以不是所有 launch 文件都能直接运行。就当前仓库状态来说，`localization_airy96.launch` 里的写法和代码是匹配的，其他一些 launch 文件需要你自己改成 `fastlio_mapping1`。

## 4. 哪个 launch 文件最接近当前可用状态

从代码与 launch 的一致性来看，当前最值得优先参考的是：

- `launch/localization_airy96.launch`

原因：

- 它调用的是 `fastlio_mapping1`
- 它显式启动了 `global_localization`
- 它显式启动了 `transform_fusion`
- 它还启动了 `base_link` 和 `base_forward_arrow`
- 它把先验地图通过 `pcd_to_pointcloud` 发布到 `/map`

相比之下，像 `localization_avia.launch`、`localization_horizon.launch`、`localization_ouster64.launch`、`localization_velodyne.launch` 这几份文件，虽然结构类似，但当前文件里很多还写着 `fastlio_mapping`，需要先修。

## 5. 配置文件里真实使用的传感器输入

配置文件决定了 FAST-LIO 里程计部分订阅哪个 LiDAR 话题、哪个 IMU 话题，以及外参。

### 5.1 `config/airy96.yaml`

当前写的是：

- LiDAR 话题：`/front_rslidar_points`
- IMU 话题：`/front_rslidar_imu_data2`
- `lidar_type: 2`
- `scan_line: 96`

外参：

- `extrinsic_T: [ -0.015106, 0.003812, 0.010996 ]`

这说明你这套 Airy96 配置实际上是按 96 线雷达和对应 IMU 接口写的。

### 5.2 `config/mid360.yaml`

当前写的是：

- LiDAR 话题：`/livox/lidar`
- IMU 话题：`/livox/imu`
- `lidar_type: 1`

这套是典型 Livox/MID360 输入风格。

## 6. `global_localization` 真实做了什么

这部分不能只写概念，必须按代码说。

当前 `src/global_localization.cpp` 里实现了下面几层逻辑：

- 启动后等待 `/map`，最多等 30 秒
- 收到 `/map` 后对全局地图做体素降采样
- 订阅 `/cloud_registered` 和 `/Odometry`
- 支持订阅 `/initialpose` 做手动初始化
- 自动线程周期运行定位
- 单独线程处理手动初始位姿请求

匹配策略上，代码不是简单单次 ICP，而是分情况：

- 未初始化时可自动初始化
- 已初始化但连续低置信度时，会进入更强的局部召回
- 连续失败后会进入全局召回模式
- 手动初始位姿会走专门的邻域搜索

代码里还实现了这些机制：

- 局部子图裁剪
- Scan Context 相关候选检索
- 多候选 coarse-to-fine ICP
- 低置信度连续帧失效判定
- 定位失败时触发 `/fast_lio_reset`
- 对 `map_to_odom` 结果做一定稳定化插值

### 6.1 关键参数

以下参数都在 `global_localization.cpp` 里实际存在：

- `map_voxel_size`
- `scan_voxel_size`
- `freq_localization`
- `localization_th`
- `global_recall_success_th`
- `failures_before_recovery`
- `enable_global_recall`
- `enable_auto_init`
- `local_crop_radius_xy`
- `local_crop_radius_z`
- `global_search_grid`
- `global_search_submap_radius`
- `scan_context_num_rings`
- `scan_context_num_sectors`
- `scan_context_max_radius`
- `manual_local_search_xy_step`
- `manual_local_search_xy_radius`
- `manual_local_search_yaw_step_deg`
- `manual_local_search_yaw_radius_deg`

`launch/localization_airy96.launch` 已经覆盖了其中一部分参数，说明它就是围绕这套新 `global_localization.cpp` 调出来的。

## 7. 坐标系关系

按当前代码，几个主要坐标系关系如下：

- `map -> odom`
  由 `global_localization` 输出到 `/map_to_odom`

- `map -> camera_init`
  由 `transform_fusion` 直接发 TF

- `map -> body`
  `transform_fusion` 计算后的 `/localization` 中使用

- `map -> base_link`
  由 `base_link.cpp` 根据 `/localization` 再次换算发布

这意味着你当前工程里至少同时涉及：

- `map`
- `camera_init`
- `body`
- `base_link`

如果你后续接导航或控制，一定要先把这几个 frame 的定义统一清楚。

## 8. 推荐的实际运行方式

### 8.1 优先从 `localization_airy96.launch` 入手

如果你现在要跑起来，最现实的入口是：

```bash
roslaunch fast_lio_localization localization_airy96.launch
```

但在运行前，需要先检查这几个点：

- `config/airy96.yaml` 里的雷达和 IMU 话题是否和你的系统一致
- `launch/localization_airy96.launch` 里的地图路径是否存在
- `base_link_to_body` 静态变换是否符合你的车体结构
- `base_link.cpp` 里的 `LIDAR_OFFSET_X` 是否正确

### 8.2 如果你要跑其他 launch 文件

先检查是否存在下面这个问题：

- launch 写的是 `fastlio_mapping`
- 但当前仓库只编译了 `fastlio_mapping1`

如果是，就先把 launch 改掉再运行。

## 9. 手动初始化怎么用

当前仓库提供了 `publish_initial_pose`，实际代码行为如下：

- 向 `/initialpose` 发布 `PoseWithCovarianceStamped`
- `frame_id` 固定为 `map`
- 命令行参数顺序是：

```text
x y z yaw pitch roll
```

命令：

```bash
rosrun fast_lio_localization publish_initial_pose 14.5 -7.5 0 -0.25 0 0
```

注意：

- 代码内部是 `q.setRPY(roll, pitch, yaw)`
- 所以命令行输入顺序虽然是 `yaw pitch roll`，最终内部会按 `roll pitch yaw` 填到四元数接口里
- 这一点最好不要改着猜，直接保持现有顺序使用

## 10. 这个仓库里目前值得你优先检查的实际问题

这些不是泛泛建议，而是从代码直接看出来的：

- 多个 launch 文件的 `type` 和实际可执行文件名不一致
- 多个 launch 文件仍保留旧机器绝对路径
- `base_link.cpp` 里雷达到底盘偏移写死
- `transform_fusion` 发布的是 `map -> camera_init`，但最终 `/localization` 的 child frame 用的是 `body`
- 工程中同时存在较多历史备份 launch 文件，容易误用

## 11. 编译

当前工程是标准 catkin 包，编译方式：

```bash
cd ~/catkin_ws/src
git clone <repo-url>
cd ..
catkin_make
source devel/setup.bash
```

依赖以 `package.xml` 和 `CMakeLists.txt` 为准，当前明确包含：

- `roscpp`
- `rospy`
- `sensor_msgs`
- `geometry_msgs`
- `nav_msgs`
- `tf`
- `pcl_ros`
- `pcl_conversions`
- `visualization_msgs`
- `message_generation`
- `livox_ros_driver2`
- `Eigen3`
- `PCL`

## 12. 一个最小理解模型

如果你只想先抓住主线，可以把当前代码理解成下面这个流程：

```text
LiDAR/IMU -> fastlio_mapping1 -> /cloud_registered + /Odometry
                           -> global_localization + /map
                           -> /map_to_odom + /localization_valid
                           -> transform_fusion
                           -> /localization
                           -> base_link TF
```

## 13. 后续整理建议

如果你准备继续维护这个仓库，建议下一步优先做这些：

- 统一所有 launch 文件的可执行文件名
- 把地图路径全部改成参数，不保留旧绝对路径
- 写清楚 `body`、`base_link`、`camera_init` 的关系
- 把 Airy96、MID360、Velodyne、Ouster 各自的输入话题整理成表格
- 确认 `publish_initial_pose` 的参数文档和实际坐标约定完全一致

## 14. 相关文件

建议你重点看这些文件：

- `src/global_localization.cpp`
- `src/transform_fusion.cpp`
- `src/base_link.cpp`
- `src/publish_initial_pose.cpp`
- `launch/localization_airy96.launch`
- `config/airy96.yaml`
- `config/mid360.yaml`
