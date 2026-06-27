cmake_minimum_required(VERSION 3.8)
project(unitree_go2_description)

# 该包是机器人描述资源包，主要安装 xacro、mesh 和 SDF world。
# 目前没有需要编译的 C++ 目标，保留编译警告模板方便未来扩展。
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ament_cmake 负责安装资源并把包注册到 ROS 2 索引中。
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Gazebo 世界文件和世界专用 mesh 资源。
install(DIRECTORY
  worlds
  DESTINATION
  share/${PROJECT_NAME})

# 机器人本体和传感器使用的 COLLADA 网格。
install(DIRECTORY
  meshes
  DESTINATION
  share/${PROJECT_NAME})

# URDF/xacro 描述文件，robot_state_publisher 和 Gazebo spawn 都会读取。
install(DIRECTORY
  urdf
  DESTINATION
  share/${PROJECT_NAME})
  

# 生成 ament 包元数据，使 $(find unitree_go2_description) 和 FindPackageShare 可用。
ament_package()
