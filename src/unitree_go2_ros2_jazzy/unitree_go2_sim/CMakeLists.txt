cmake_minimum_required(VERSION 3.8)
project(unitree_go2_sim)

# 该包只安装 launch/config/rviz 等运行期资源，不编译 C++ 节点。
# 编译选项保留为模板默认值，便于以后新增源码时自动启用常见警告。
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ament_cmake 负责生成 ROS 2 包索引和安装规则。
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# 安装启动文件，launch 时会从 share/unitree_go2_sim/launch 中查找。
install(
  DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}
)

# 安装 CHAMP、ros2_control 等运行参数。
install(
  DIRECTORY config 
  DESTINATION share/${PROJECT_NAME}
)

# 安装 RViz 预设视图，方便一键打开机器人模型、点云和相机话题。
install(
  DIRECTORY rviz 
  DESTINATION share/${PROJECT_NAME}
)

# 生成 ament 包元数据，使 ros2 launch 和 FindPackageShare 可以定位本包资源。
ament_package()
