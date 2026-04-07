# ROS2 Humble Workspace Migration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Migrate the full workspace from ROS1 `catkin` to ROS2 Humble `ament_cmake` while preserving the current external interfaces and unifying all runtime logging on `glog`.

**Architecture:** Keep the existing package boundaries and node names, add a small shared header-only compatibility layer under the repository root for `glog` initialization, time conversion, and package resource lookup, then migrate each package in dependency order. Move ROS-facing code to `rclcpp` and `tf2_ros`, keep algorithm configuration loading on OpenCV `FileStorage`, and verify the migration with package-by-package `colcon build` plus residual ROS1 API scans.

**Tech Stack:** ROS2 Humble, `ament_cmake`, `rclcpp`, `tf2_ros`, `cv_bridge`, OpenCV, Ceres, Boost, `glog`, `ament_index_cpp`, Python launch

---

## File Structure Map

- Shared compatibility headers live in `common/include/vins_fusion/common/`.
- Package build metadata remains in each package’s `CMakeLists.txt` and `package.xml`.
- ROS2 node entrypoints stay in the current source files:
  - `vins_estimator/src/rosNodeTest.cpp`
  - `vins_estimator/src/KITTIOdomTest.cpp`
  - `vins_estimator/src/KITTIGPSTest.cpp`
  - `loop_fusion/src/pose_graph_node.cpp`
  - `global_fusion/src/globalOptNode.cpp`
- ROS-facing visualization helpers stay package-local:
  - `vins_estimator/src/utility/visualization.*`
  - `vins_estimator/src/utility/CameraPoseVisualization.*`
  - `loop_fusion/src/utility/CameraPoseVisualization.*`
  - `loop_fusion/src/pose_graph.*`
- Launch files are added under each package’s `launch/` directory as `.launch.py`.

### Task 1: Migrate Build Metadata to `ament_cmake`

**Files:**
- Create: `vins_estimator/launch/vins_estimator.launch.py`
- Create: `loop_fusion/launch/loop_fusion.launch.py`
- Create: `global_fusion/launch/global_fusion.launch.py`
- Create: `vins_estimator/launch/vins_rviz.launch.py`
- Modify: `camera_models/CMakeLists.txt`
- Modify: `camera_models/package.xml`
- Modify: `vins_estimator/CMakeLists.txt`
- Modify: `vins_estimator/package.xml`
- Modify: `loop_fusion/CMakeLists.txt`
- Modify: `loop_fusion/package.xml`
- Modify: `global_fusion/CMakeLists.txt`
- Modify: `global_fusion/package.xml`

- [ ] **Step 1: Run the current package build to capture the ROS1 baseline failure mode under ROS2 tooling**

Run: `colcon build --packages-select camera_models vins_estimator loop_fusion global_fusion`
Expected: FAIL with `catkin_package` / `find_package(catkin ...)` / `roscpp`-style configuration errors.

- [ ] **Step 2: Rewrite `camera_models/CMakeLists.txt` to export an `ament_cmake` library and executable**

```cmake
cmake_minimum_required(VERSION 3.16)
project(camera_models)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem program_options system)

add_library(camera_models
  src/chessboard/Chessboard.cc
  src/calib/CameraCalibration.cc
  src/camera_models/Camera.cc
  src/camera_models/CameraFactory.cc
  src/camera_models/CostFunctionFactory.cc
  src/camera_models/PinholeCamera.cc
  src/camera_models/PinholeFullCamera.cc
  src/camera_models/CataCamera.cc
  src/camera_models/EquidistantCamera.cc
  src/camera_models/ScaramuzzaCamera.cc
  src/sparse_graph/Transform.cc
  src/gpl/gpl.cc
  src/gpl/EigenQuaternionParameterization.cc)

target_include_directories(camera_models PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../common/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(camera_models ${Boost_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})

add_executable(Calibrations src/intrinsic_calib.cc)
target_link_libraries(Calibrations camera_models)

install(TARGETS camera_models Calibrations
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../common/include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(camera_models)
ament_package()
```

- [ ] **Step 3: Rewrite the four `package.xml` files to ROS2 format with `ament_cmake` and runtime dependencies**

```xml
<?xml version="1.0"?>
<package format="3">
  <name>vins_estimator</name>
  <version>0.0.0</version>
  <description>ROS2 Humble port of the VINS estimator package.</description>
  <maintainer email="qintonguav@gmail.com">qintong</maintainer>
  <license>GPLv3</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>camera_models</depend>
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>visualization_msgs</depend>
  <depend>cv_bridge</depend>
  <depend>tf2</depend>
  <depend>tf2_ros</depend>
  <depend>tf2_geometry_msgs</depend>
  <depend>ament_index_cpp</depend>

  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 4: Convert the ROS packages’ `CMakeLists.txt` files to ROS2 targets and install rules**

```cmake
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

ament_target_dependencies(vins_lib
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  std_msgs
  cv_bridge
  tf2
  tf2_ros
  tf2_geometry_msgs
  ament_index_cpp)

install(TARGETS vins_lib vins_node kitti_odom_test kitti_gps_test
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY ../config/ DESTINATION share/${PROJECT_NAME}/config)
```

- [ ] **Step 5: Run the metadata-only build to confirm CMake now configures under ROS2**

Run: `colcon build --packages-select camera_models vins_estimator loop_fusion global_fusion --cmake-args -DBUILD_TESTING=OFF`
Expected: FAIL in source compilation, not in `catkin` / package manifest parsing.

- [ ] **Step 6: Commit the build-system migration checkpoint**

```bash
git add camera_models/CMakeLists.txt camera_models/package.xml global_fusion/CMakeLists.txt global_fusion/package.xml loop_fusion/CMakeLists.txt loop_fusion/package.xml vins_estimator/CMakeLists.txt vins_estimator/package.xml vins_estimator/launch/vins_estimator.launch.py vins_estimator/launch/vins_rviz.launch.py loop_fusion/launch/loop_fusion.launch.py global_fusion/launch/global_fusion.launch.py
git commit -m "build: migrate package metadata to ament"
```

### Task 2: Add the Shared `glog` and ROS2 Compatibility Layer

**Files:**
- Create: `common/include/vins_fusion/common/glog_init.h`
- Create: `common/include/vins_fusion/common/ros2_time.h`
- Create: `common/include/vins_fusion/common/resource_path.h`
- Create: `common/include/vins_fusion/common/visibility.h`
- Modify: `camera_models/CMakeLists.txt`
- Modify: `vins_estimator/CMakeLists.txt`
- Modify: `loop_fusion/CMakeLists.txt`
- Modify: `global_fusion/CMakeLists.txt`

- [ ] **Step 1: Run a source scan to confirm the repository still lacks the shared helper headers**

Run: `rg -n "glog_init|ros2_time|resource_path" common include camera_models vins_estimator loop_fusion global_fusion`
Expected: no matches.

- [ ] **Step 2: Create `common/include/vins_fusion/common/glog_init.h` with process-level `glog` initialization**

```cpp
#pragma once

#include <glog/logging.h>
#include <string>

namespace vins_fusion::common {

inline void InitGlog(const char *program_name, const std::string &log_dir,
                     bool log_to_stderr, int verbosity) {
  static bool initialized = false;
  if (!initialized) {
    google::InitGoogleLogging(program_name);
    initialized = true;
  }
  FLAGS_logtostderr = log_to_stderr ? 1 : 0;
  FLAGS_v = verbosity;
  if (!log_dir.empty()) {
    google::SetLogDestination(google::GLOG_INFO, (log_dir + "/").c_str());
    google::SetLogDestination(google::GLOG_WARNING, (log_dir + "/").c_str());
    google::SetLogDestination(google::GLOG_ERROR, (log_dir + "/").c_str());
  }
}

}  // namespace vins_fusion::common
```

- [ ] **Step 3: Create `common/include/vins_fusion/common/ros2_time.h` and `resource_path.h`**

```cpp
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace vins_fusion::common {

inline rclcpp::Time FromSec(double seconds) {
  return rclcpp::Time(static_cast<int64_t>(seconds * 1e9));
}

inline double ToSec(const rclcpp::Time &stamp) {
  return static_cast<double>(stamp.nanoseconds()) * 1e-9;
}

inline std::string SharePath(const std::string &package_name) {
  return ament_index_cpp::get_package_share_directory(package_name);
}

}  // namespace vins_fusion::common
```

- [ ] **Step 4: Add the shared include directory and `glog` linkage in each package CMake target**

```cmake
find_package(glog REQUIRED)

target_include_directories(loop_fusion_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../common/include)

target_link_libraries(loop_fusion_node
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES}
  glog::glog)
```

- [ ] **Step 5: Build one target that consumes the helper headers to confirm include propagation**

Run: `colcon build --packages-select camera_models --cmake-args -DBUILD_TESTING=OFF`
Expected: PASS for `camera_models`, or fail only on package-local source issues unrelated to missing shared headers.

- [ ] **Step 6: Commit the compatibility-layer checkpoint**

```bash
git add common/include/vins_fusion/common/glog_init.h common/include/vins_fusion/common/ros2_time.h common/include/vins_fusion/common/resource_path.h common/include/vins_fusion/common/visibility.h camera_models/CMakeLists.txt vins_estimator/CMakeLists.txt loop_fusion/CMakeLists.txt global_fusion/CMakeLists.txt
git commit -m "feat: add shared ros2 and glog helpers"
```

### Task 3: Migrate `camera_models` to ROS2-Compatible Build and `glog`

**Files:**
- Modify: `camera_models/src/intrinsic_calib.cc`
- Modify: `camera_models/src/camera_models/CameraFactory.cc`
- Modify: `camera_models/src/camera_models/EquidistantCamera.cc`
- Modify: `camera_models/src/camera_models/CataCamera.cc`
- Modify: `camera_models/src/calib/CameraCalibration.cc`

- [ ] **Step 1: Scan `camera_models` for runtime logging still using iostreams**

Run: `rg -n "std::cout|std::cerr|printf\\(" camera_models/src camera_models/include`
Expected: matches in `intrinsic_calib.cc`, `CameraFactory.cc`, camera model implementations, and calibration code.

- [ ] **Step 2: Replace the CLI entrypoint logging in `camera_models/src/intrinsic_calib.cc`**

```cpp
#include "vins_fusion/common/glog_init.h"
#include <glog/logging.h>

int main(int argc, char **argv) {
  vins_fusion::common::InitGlog(argv[0], "", true, 0);
  CHECK(argc >= 2) << "Usage: Calibrations <config.yaml>";
  LOG(INFO) << "Running camera calibration with config: " << argv[1];
  // existing calibration flow
}
```

- [ ] **Step 3: Replace library-side stream logging with `glog` macros**

```cpp
if (unknown_model) {
  LOG(ERROR) << "Unknown camera model: " << sModelType;
  return CameraPtr();
}

LOG(INFO) << "[" << params.cameraName() << "] reprojection error: "
          << reprojectionError;
```

- [ ] **Step 4: Build the package to verify `camera_models` is a stable ROS2-era dependency**

Run: `colcon build --packages-select camera_models --cmake-args -DBUILD_TESTING=OFF`
Expected: PASS.

- [ ] **Step 5: Commit the `camera_models` checkpoint**

```bash
git add camera_models/src/intrinsic_calib.cc camera_models/src/camera_models/CameraFactory.cc camera_models/src/camera_models/EquidistantCamera.cc camera_models/src/camera_models/CataCamera.cc camera_models/src/calib/CameraCalibration.cc
git commit -m "refactor: migrate camera_models logging to glog"
```

### Task 4: Port `vins_estimator` Core Library and Logging

**Files:**
- Modify: `vins_estimator/src/estimator/parameters.h`
- Modify: `vins_estimator/src/estimator/parameters.cpp`
- Modify: `vins_estimator/src/estimator/estimator.cpp`
- Modify: `vins_estimator/src/estimator/feature_manager.h`
- Modify: `vins_estimator/src/estimator/feature_manager.cpp`
- Modify: `vins_estimator/src/featureTracker/feature_tracker.cpp`
- Modify: `vins_estimator/src/initial/initial_alignment.h`
- Modify: `vins_estimator/src/initial/initial_aligment.cpp`
- Modify: `vins_estimator/src/initial/initial_ex_rotation.h`
- Modify: `vins_estimator/src/initial/initial_ex_rotation.cpp`
- Modify: `vins_estimator/src/initial/solve_5pts.h`
- Modify: `vins_estimator/src/factor/*.h`
- Modify: `vins_estimator/src/factor/marginalization_factor.cpp`

- [ ] **Step 1: Capture the current ROS1-only include and macro usage in the core library**

Run: `rg -n "#include <ros/|ROS_[A-Z]+|ros/assert|ros/console|printf\\(|std::cout|std::cerr" vins_estimator/src/estimator vins_estimator/src/featureTracker vins_estimator/src/initial vins_estimator/src/factor`
Expected: matches in parameters, estimator, feature manager, factor headers, and initialization helpers.

- [ ] **Step 2: Remove ROS headers from parameter and algorithm headers, and replace assertions with `CHECK`**

```cpp
#include <glog/logging.h>

template <typename T>
T ReadRequiredYaml(const cv::FileStorage &fs, const std::string &name) {
  T value{};
  fs[name] >> value;
  CHECK(!fs[name].empty()) << "Missing required config key: " << name;
  return value;
}
```

- [ ] **Step 3: Replace ROS logging macros in the estimator pipeline with `glog`**

```cpp
LOG(INFO) << "init begins";
VLOG(1) << "new image coming ------------------------------------------";
VLOG(1) << "Adding feature points " << image.size();
LOG(WARNING) << "failure detection!";
CHECK(last_marginalization_parameter_blocks[i] != para_SpeedBias[WINDOW_SIZE - 1]);
```

- [ ] **Step 4: Replace ROS-only failure paths in config loading and marginalization**

```cpp
if (fh == nullptr) {
  LOG(FATAL) << "config_file doesn't exist: " << config_file;
}

if (ret != 0) {
  LOG(FATAL) << "pthread_create error";
}
```

- [ ] **Step 5: Build only `vins_estimator` library targets to confirm the core code no longer depends on ROS1**

Run: `colcon build --packages-select vins_estimator --cmake-args -DBUILD_TESTING=OFF`
Expected: FAIL only in ROS node entry files or visualization files, not in the algorithm library due to `ros/ros.h` or `ROS_*` usage.

- [ ] **Step 6: Commit the core-library checkpoint**

```bash
git add vins_estimator/src/estimator/parameters.h vins_estimator/src/estimator/parameters.cpp vins_estimator/src/estimator/estimator.cpp vins_estimator/src/estimator/feature_manager.h vins_estimator/src/estimator/feature_manager.cpp vins_estimator/src/featureTracker/feature_tracker.cpp vins_estimator/src/initial/initial_alignment.h vins_estimator/src/initial/initial_aligment.cpp vins_estimator/src/initial/initial_ex_rotation.h vins_estimator/src/initial/initial_ex_rotation.cpp vins_estimator/src/initial/solve_5pts.h vins_estimator/src/factor vins_estimator/src/factor/marginalization_factor.cpp
git commit -m "refactor: decouple vins_estimator core from ros1"
```

### Task 5: Port `vins_estimator` ROS Nodes, Visualization, TF, and Tool Programs

**Files:**
- Modify: `vins_estimator/src/rosNodeTest.cpp`
- Modify: `vins_estimator/src/KITTIOdomTest.cpp`
- Modify: `vins_estimator/src/KITTIGPSTest.cpp`
- Modify: `vins_estimator/src/utility/visualization.h`
- Modify: `vins_estimator/src/utility/visualization.cpp`
- Modify: `vins_estimator/src/utility/CameraPoseVisualization.h`
- Modify: `vins_estimator/src/utility/CameraPoseVisualization.cpp`

- [ ] **Step 1: Verify the current node entrypoints still depend on ROS1 node handles and publishers**

Run: `rg -n "ros::NodeHandle|ros::Publisher|ros::Subscriber|ros::spin|tf::TransformBroadcaster|sensor_msgs::ImagePtr|cv_bridge::CvImage" vins_estimator/src/rosNodeTest.cpp vins_estimator/src/KITTIOdomTest.cpp vins_estimator/src/KITTIGPSTest.cpp vins_estimator/src/utility`
Expected: matches across all listed files.

- [ ] **Step 2: Rewrite `rosNodeTest.cpp` as a ROS2 node wrapper around the existing estimator object**

```cpp
class VinsEstimatorNode : public rclcpp::Node {
 public:
  VinsEstimatorNode() : Node("vins_estimator") {
    this->declare_parameter<std::string>("config_file", "");
    const auto config_file = this->get_parameter("config_file").as_string();
    vins_fusion::common::InitGlog("vins_node", "", true, 0);
    readParameters(config_file);
    estimator_.setParameter();
    registerPub(shared_from_this(), tf_broadcaster_);
    sub_feature_ = create_subscription<sensor_msgs::msg::PointCloud>(
        "/feature_tracker/feature", rclcpp::SensorDataQoS(),
        std::bind(&VinsEstimatorNode::FeatureCallback, this, std::placeholders::_1));
  }
};
```

- [ ] **Step 3: Rewrite `visualization.*` to use ROS2 publishers and `tf2_ros::TransformBroadcaster`**

```cpp
using OdometryPub = rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr;

void registerPub(const rclcpp::Node::SharedPtr &node,
                 const std::shared_ptr<tf2_ros::TransformBroadcaster> &tf_broadcaster);

geometry_msgs::msg::TransformStamped tf_msg;
tf_msg.header = header;
tf_msg.child_frame_id = "body";
tf_msg.transform.translation.x = correct_t.x();
tf_msg.transform.rotation.w = correct_q.w();
tf_broadcaster->sendTransform(tf_msg);
```

- [ ] **Step 4: Port `KITTIOdomTest.cpp` and `KITTIGPSTest.cpp` to ROS2-compatible publishers**

```cpp
auto node = std::make_shared<rclcpp::Node>("vins_estimator");
auto pub_left = node->create_publisher<sensor_msgs::msg::Image>("/leftImage", 1000);
auto pub_gps = node->create_publisher<sensor_msgs::msg::NavSatFix>("/gps", 1000);

auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", image).toImageMsg();
msg->header.stamp = vins_fusion::common::FromSec(image_time);
pub_left->publish(*msg);
```

- [ ] **Step 5: Build `vins_estimator` end-to-end**

Run: `colcon build --packages-select vins_estimator --cmake-args -DBUILD_TESTING=OFF`
Expected: PASS.

- [ ] **Step 6: Commit the node-port checkpoint**

```bash
git add vins_estimator/src/rosNodeTest.cpp vins_estimator/src/KITTIOdomTest.cpp vins_estimator/src/KITTIGPSTest.cpp vins_estimator/src/utility/visualization.h vins_estimator/src/utility/visualization.cpp vins_estimator/src/utility/CameraPoseVisualization.h vins_estimator/src/utility/CameraPoseVisualization.cpp
git commit -m "feat: port vins_estimator nodes to ros2"
```

### Task 6: Port `loop_fusion` Node, Pose Graph Publishing, and Resource Lookup

**Files:**
- Modify: `loop_fusion/src/pose_graph_node.cpp`
- Modify: `loop_fusion/src/pose_graph.h`
- Modify: `loop_fusion/src/pose_graph.cpp`
- Modify: `loop_fusion/src/parameters.h`
- Modify: `loop_fusion/src/utility/CameraPoseVisualization.h`
- Modify: `loop_fusion/src/utility/CameraPoseVisualization.cpp`
- Modify: `loop_fusion/src/keyframe.cpp`

- [ ] **Step 1: Run a targeted scan of ROS1 APIs in `loop_fusion`**

Run: `rg -n "#include <ros/|ros::|ROS_[A-Z]+|ros::package::getPath|cv_bridge::CvImage|printf\\(|std::cout|std::cerr" loop_fusion/src`
Expected: matches in `pose_graph_node.cpp`, `pose_graph.*`, `parameters.h`, `keyframe.cpp`, and visualization helpers.

- [ ] **Step 2: Convert `PoseGraph::registerPub` and stored publisher types to ROS2**

```cpp
void PoseGraph::registerPub(const rclcpp::Node::SharedPtr &node) {
  pub_pg_path = node->create_publisher<nav_msgs::msg::Path>("pose_graph_path", 1000);
  pub_base_path = node->create_publisher<nav_msgs::msg::Path>("base_path", 1000);
  pub_pose_graph =
      node->create_publisher<visualization_msgs::msg::MarkerArray>("pose_graph", 1000);
}
```

- [ ] **Step 3: Rewrite `pose_graph_node.cpp` to use ROS2 subscriptions and `ament_index_cpp`**

```cpp
const std::string pkg_path = vins_fusion::common::SharePath("loop_fusion");
BRIEF_PATTERN_FILE = pkg_path + "/support_files/brief_pattern.yml";

sub_vio_ = create_subscription<nav_msgs::msg::Odometry>(
    "/vins_estimator/odometry", rclcpp::SensorDataQoS(),
    std::bind(&LoopFusionNode::VioCallback, this, std::placeholders::_1));
```

- [ ] **Step 4: Replace runtime prints and ROS macros with `glog` in the loop-fusion pipeline**

```cpp
LOG(WARNING) << "image discontinue! detect a new sequence!";
CHECK(sequence <= 5) << "only support 5 sequences";
LOG(INFO) << "load pose graph finish";
VLOG(1) << "throw point at beginning";
```

- [ ] **Step 5: Build the package against the already-migrated dependencies**

Run: `colcon build --packages-select loop_fusion --cmake-args -DBUILD_TESTING=OFF`
Expected: PASS.

- [ ] **Step 6: Commit the `loop_fusion` checkpoint**

```bash
git add loop_fusion/src/pose_graph_node.cpp loop_fusion/src/pose_graph.h loop_fusion/src/pose_graph.cpp loop_fusion/src/parameters.h loop_fusion/src/utility/CameraPoseVisualization.h loop_fusion/src/utility/CameraPoseVisualization.cpp loop_fusion/src/keyframe.cpp
git commit -m "feat: port loop_fusion to ros2"
```

### Task 7: Port `global_fusion` Node and Parameterize Outputs

**Files:**
- Modify: `global_fusion/src/globalOptNode.cpp`
- Modify: `global_fusion/src/globalOpt.cpp`
- Modify: `global_fusion/src/globalOpt.h`

- [ ] **Step 1: Confirm the remaining ROS1 APIs in `global_fusion`**

Run: `rg -n "#include <ros/|ros::|ROS_[A-Z]+|printf\\(|std::cout|std::cerr|ros::Time" global_fusion/src`
Expected: matches in `globalOptNode.cpp` and `globalOpt.cpp`.

- [ ] **Step 2: Rewrite the node entrypoint as a ROS2 node with declared output parameters**

```cpp
class GlobalFusionNode : public rclcpp::Node {
 public:
  GlobalFusionNode() : Node("globalEstimator") {
    this->declare_parameter<std::string>("output_path", "");
    output_path_ = this->get_parameter("output_path").as_string();
    sub_gps_ = create_subscription<sensor_msgs::msg::NavSatFix>(
        "/gps", 100, std::bind(&GlobalFusionNode::GpsCallback, this, std::placeholders::_1));
    sub_vio_ = create_subscription<nav_msgs::msg::Odometry>(
        "/vins_estimator/odometry", 100,
        std::bind(&GlobalFusionNode::VioCallback, this, std::placeholders::_1));
  }
};
```

- [ ] **Step 3: Replace time conversions, logging, and the hardcoded results path**

```cpp
car_mesh.header.stamp = vins_fusion::common::FromSec(t);
LOG(INFO) << "vio t: " << t << ", gps t: " << gps_t;

std::ofstream foutC(output_path_, std::ios::app);
CHECK(foutC.is_open()) << "Failed to open output file: " << output_path_;
```

- [ ] **Step 4: Build `global_fusion`**

Run: `colcon build --packages-select global_fusion --cmake-args -DBUILD_TESTING=OFF`
Expected: PASS.

- [ ] **Step 5: Commit the `global_fusion` checkpoint**

```bash
git add global_fusion/src/globalOptNode.cpp global_fusion/src/globalOpt.cpp global_fusion/src/globalOpt.h
git commit -m "feat: port global_fusion to ros2"
```

### Task 8: Install Launch Files, Models, Configs, and Run Workspace Verification

**Files:**
- Modify: `vins_estimator/CMakeLists.txt`
- Modify: `loop_fusion/CMakeLists.txt`
- Modify: `global_fusion/CMakeLists.txt`
- Modify: `camera_models/CMakeLists.txt`
- Create: `vins_estimator/launch/vins_estimator.launch.py`
- Create: `vins_estimator/launch/vins_rviz.launch.py`
- Create: `loop_fusion/launch/loop_fusion.launch.py`
- Create: `global_fusion/launch/global_fusion.launch.py`

- [ ] **Step 1: Add install rules for runtime assets and launch files**

```cmake
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY models/ DESTINATION share/${PROJECT_NAME}/models)
install(DIRECTORY support_files/ DESTINATION share/${PROJECT_NAME}/support_files)
```

- [ ] **Step 2: Add ROS2 Python launch files that preserve the current entrypoint semantics**

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("config_file"),
        Node(
            package="vins_estimator",
            executable="vins_node",
            name="vins_estimator",
            output="screen",
            parameters=[{"config_file": LaunchConfiguration("config_file")}],
        ),
    ])
```

- [ ] **Step 3: Run the full workspace build**

Run: `colcon build --packages-select camera_models vins_estimator loop_fusion global_fusion --cmake-args -DBUILD_TESTING=OFF`
Expected: PASS for all four packages.

- [ ] **Step 4: Run residual ROS1 API and logging scans**

Run: `rg -n "find_package\\(catkin|catkin_package\\(|#include <ros/|ros::NodeHandle|ros::Publisher|ros::Subscriber|ros::init|ROS_[A-Z]+|tf::TransformBroadcaster" camera_models vins_estimator loop_fusion global_fusion`
Expected: no matches.

Run: `rg -n "printf\\(|std::cout|std::cerr" camera_models vins_estimator loop_fusion global_fusion`
Expected: either no matches or matches only inside deliberate numeric debug helper code that is immediately replaced in the same commit.

- [ ] **Step 5: Smoke-test the launch files**

Run: `source install/setup.bash && ros2 launch vins_estimator vins_estimator.launch.py config_file:=/absolute/path/to/config.yaml`
Expected: node starts, declares `config_file`, initializes `glog`, and creates publishers/subscribers without `rclcpp` or package share lookup errors.

- [ ] **Step 6: Commit the final verification checkpoint**

```bash
git add camera_models/CMakeLists.txt vins_estimator/CMakeLists.txt loop_fusion/CMakeLists.txt global_fusion/CMakeLists.txt vins_estimator/launch/vins_estimator.launch.py vins_estimator/launch/vins_rviz.launch.py loop_fusion/launch/loop_fusion.launch.py global_fusion/launch/global_fusion.launch.py
git commit -m "feat: finish ros2 humble workspace migration"
```
