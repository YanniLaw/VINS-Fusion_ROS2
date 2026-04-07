# ROS2 Humble Migration Design

## Goal

将当前仓库整体从 ROS1 `catkin` 迁移到 ROS2 Humble `ament_cmake`，同时统一日志体系到 `glog`，并尽量保持现有外部接口兼容，包括包名、主要可执行文件名、topic 名、消息类型语义和现有算法配置 YAML 字段。

## Scope

本次迁移覆盖以下包：

- `camera_models`
- `vins_estimator`
- `loop_fusion`
- `global_fusion`

本次迁移同时覆盖在线节点与工具程序：

- `vins_node`
- `loop_fusion_node`
- `global_fusion_node`
- `kitti_odom_test`
- `kitti_gps_test`
- `Calibrations`

## Non-Goals

- 不重写 VIO、回环检测、全局优化等核心算法逻辑
- 不引入新的自定义消息体系
- 不对现有算法配置 YAML 做大规模字段重命名
- 不把现有节点重构成 ROS2 components
- 不对数据流、topic 语义和外部联调方式做不必要的行为变更

## Constraints

- ROS 运行时迁移目标为 ROS2 Humble
- 构建系统迁移目标为 `ament_cmake`
- 日志统一使用 `glog`
- 运行接口兼容优先于内部代码风格现代化
- launch 文件迁移到 ROS2 Python launch
- 资源文件必须通过安装规则进入 share 目录，运行期不依赖源码相对路径

## Compatibility Strategy

以下内容优先保持兼容：

- 包名：`camera_models`、`vins_estimator`、`loop_fusion`、`global_fusion`
- 主要可执行文件名：`vins_node`、`loop_fusion_node`、`global_fusion_node`、`kitti_odom_test`、`kitti_gps_test`、`Calibrations`
- 现有 topic 名与消息类型语义
- 现有 OpenCV `FileStorage` 配置 YAML 字段
- 现有结果文件和模型文件的用途

以下内容允许内部改造：

- `roscpp` / `ros::NodeHandle` / `ros::Publisher` / `ros::Subscriber` 改为 `rclcpp`
- `tf` 改为 `tf2_ros`
- `catkin` 改为 `ament_cmake`
- ROS1 日志宏、断言、`printf/cout/cerr` 改为 `glog`
- ROS1 XML launch 改为 ROS2 Python launch

## Shared Infrastructure

为避免四个包各自实现一套 ROS2 兼容写法，本次迁移增加一层仓库内共享基础设施，职责限定如下：

### Logging

提供统一 `glog` 初始化入口，用于：

- `InitGoogleLogging`
- 设置 `FLAGS_logtostderr`
- 设置 `FLAGS_v`
- 设置日志目录
- 统一程序退出时 `ShutdownGoogleLogging`

日志映射规则：

- 原 `ROS_DEBUG`、`ROS_DEBUG_STREAM` 改为 `VLOG(1)` 或等价低级别调试日志
- 原 `ROS_INFO`、`ROS_INFO_STREAM` 改为 `LOG(INFO)`
- 原 `ROS_WARN`、`ROS_WARN_STREAM` 改为 `LOG(WARNING)`
- 原 `ROS_ERROR`、`ROS_ERROR_STREAM` 改为 `LOG(ERROR)`
- 原 `ROS_ASSERT`、`ROS_BREAK`、关键运行时 `assert` 改为 `CHECK` / `LOG(FATAL)`

### Time and Header Helpers

提供 `double` 时间戳与 ROS2 时间类型之间的转换辅助，避免各节点重复实现：

- `double -> rclcpp::Time`
- `rclcpp::Time -> double`
- 标准 `std_msgs::msg::Header` stamp 写入

### Resource Path Helpers

提供通过 `ament_index_cpp` 查询 share 目录资源路径的能力，替换：

- `ros::package::getPath()`

主要用于：

- rviz 配置
- 词袋文件
- 模型文件
- 可能的默认配置文件

### TF Helpers

统一 `tf2_ros::TransformBroadcaster` 的使用方式，替换原 `tf::TransformBroadcaster` 和 `tf::StampedTransform`。

## Package Design

### camera_models

定位保持为基础库加工具程序：

- 迁移 `CMakeLists.txt` 和 `package.xml` 到 `ament_cmake`
- 导出 `include` 和 `camera_models` 库
- 保持 `Calibrations` 可执行程序存在
- 不引入 ROS2 节点通信逻辑
- 把库和工具中的 `cout/cerr` 迁到 `glog`

依赖：

- `ament_cmake`
- `OpenCV`
- `Ceres`
- `Boost`

### vins_estimator

这是主迁移包，改造内容包括：

- `rosNodeTest.cpp` 改成 ROS2 节点类或等价 ROS2 入口
- 订阅、发布、spin、时钟、线程模型切换到 `rclcpp`
- `visualization.*` 中所有 publisher、Header、Time、TF 广播切到 ROS2
- `parameters.cpp` 去除对 `ros::NodeHandle` 的依赖，保留 YAML 读取逻辑
- `KITTIOdomTest.cpp`、`KITTIGPSTest.cpp` 迁移到 ROS2 入口
- 算法层所有 `ROS_*` / `ROS_ASSERT` / `printf/cout/cerr` 迁到 `glog`

保持兼容的接口：

- 主要 topic 名
- 消息类型语义
- 配置文件字段
- 输出文件语义

### loop_fusion

保留现有单节点加后台处理线程结构，不做算法级重构：

- `pose_graph_node.cpp` 改为 ROS2 节点入口
- 订阅和发布切换到 ROS2
- 图像、点云、位姿同步仍使用现有队列方式
- `ros::package::getPath()` 改为 `ament_index_cpp`
- `pose_graph.*` 和 `keyframe.*` 只做 ROS2 API 与日志替换
- 可视化 publisher 与图像发布改为 ROS2 风格

### global_fusion

保持为轻量在线融合节点：

- `globalOptNode.cpp` 改为 ROS2 节点入口
- 订阅 `/gps` 和 `/vins_estimator/odometry` 的逻辑保持一致
- 发布全局里程计、路径和车模 marker
- 输出结果路径从硬编码改为参数或 share/config 相关逻辑
- 日志统一迁移到 `glog`

## Parameter Strategy

参数分两层：

### Algorithm Config Files

继续保留现有 OpenCV `FileStorage` YAML 读取方式，原因：

- 现有配置字段多
- 这些字段已经被多个数据集和启动流程依赖
- 强制改成大量 ROS2 declared parameters 会引入高风险和高迁移成本

配置方式保持为：

- 启动节点时传入 `config_file`
- 节点内部读取原始 YAML

### ROS2 Runtime Parameters

仅用于少量运行时入口配置，例如：

- `config_file`
- `use_sim_time`
- `log_dir`
- `log_to_stderr`
- `verbosity`
- 某些结果输出路径覆盖项

## Launch Strategy

ROS1 XML launch 全部迁移为 ROS2 Python launch。

原则：

- 每个在线包至少提供一个基础 launch
- launch 通过参数传入 `config_file`
- 保持现有 rviz 配置的使用方式
- 所有被运行时使用的配置、模型、rviz 文件通过 `install()` 安装到 share

特殊项：

- `vins_estimator/launch/vins_rviz.launch` 迁为 ROS2 launch，并通过 share 目录定位 `config/vins_rviz_config.rviz`

## Build System Strategy

四个包统一迁移到 `ament_cmake`。

要求：

- 更新所有 `CMakeLists.txt`
- 更新所有 `package.xml`
- 用 `ament_target_dependencies()` 和现代 CMake 连接 ROS2 依赖
- 安装二进制、launch、配置、模型、rviz 资源
- 支持 `colcon build`

## Verification Strategy

验收标准如下：

1. 四个包都能作为 ROS2 Humble `ament_cmake` 包参与 `colcon build`
2. `vins_node`、`loop_fusion_node`、`global_fusion_node` 可以在 ROS2 下启动
3. `kitti_odom_test`、`kitti_gps_test`、`Calibrations` 可编译可运行
4. 仓库中不再残留 ROS1 核心依赖：
   - `catkin`
   - `roscpp`
   - `ros::NodeHandle`
   - `ros::Publisher`
   - `ros::Subscriber`
   - `ros::init`
   - `ROS_*` 日志宏
   - `tf::TransformBroadcaster`
5. 主要 topic 和消息语义保持兼容
6. launch 可以通过 ROS2 方式启动，资源路径解析正常
7. 日志统一输出到 `glog`

## Risks

### ROS1 Message API Differences

ROS2 消息命名空间和智能指针类型与 ROS1 不同，节点入口和回调签名改动面较大。缓解方式是优先修改节点和可视化边界文件，算法层尽量保持不感知 ROS。

### TF Migration

`tf` 到 `tf2_ros` 的转换集中在可视化和姿态广播模块，需要避免时间戳和 frame id 行为偏差。缓解方式是把 TF 广播封装在共享 helper 内。

### Resource Lookup

原有部分逻辑依赖 `ros::package::getPath()` 和源码相对路径。迁移后必须通过安装目录和 `ament_index_cpp` 定位资源，否则 launch 成功但运行失败。

### Logging Volume

把所有 `printf/cout` 直接替换为 `LOG(INFO)` 可能导致日志量过大。缓解方式是调试类输出优先使用 `VLOG(1)`，仅保留关键状态为 `LOG(INFO)`。

## Implementation Order

建议实现顺序如下：

1. 迁移 `camera_models`
2. 增加共享 `glog` / ROS2 helper
3. 迁移 `vins_estimator`
4. 迁移 `loop_fusion`
5. 迁移 `global_fusion`
6. 迁移 launch 与资源安装
7. 全仓库清理剩余 ROS1 API 和旧日志接口
8. 统一构建与运行验证
