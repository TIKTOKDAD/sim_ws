cmake_minimum_required(VERSION 3.20)
project(clearpath_generator_gz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# 安装可执行脚本到 lib/${PROJECT_NAME}，这样 ROS 2 可以通过
# `ros2 run clearpath_generator_gz <executable>` 或 launch Node 找到它们。
install(PROGRAMS
  ${PROJECT_NAME}/launch/generate_launch
  ${PROJECT_NAME}/nodes/ptz_controller_node
  ${PROJECT_NAME}/nodes/ptu_sim_relay_node
  ${PROJECT_NAME}/param/generate_param
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 Python package 本体，供上面的可执行脚本 import generator/sensors 模块。
ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
