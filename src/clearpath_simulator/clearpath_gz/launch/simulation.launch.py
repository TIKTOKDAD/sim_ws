# Copyright 2023 Clearpath Robotics, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# @author Roni Kreinin (rkreinin@clearpathrobotics.com)

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution


ARGUMENTS = [
    # 是否同时启动 RViz。默认只跑仿真，避免无显示环境或 CI 中额外打开图形界面。
    DeclareLaunchArgument('rviz', default_value='false',
                          choices=['true', 'false'], description='Start rviz.'),
    # world 只填写名字；下层 gz_sim.launch.py 会把它转换为 `<world>.sdf`。
    DeclareLaunchArgument('world', default_value='warehouse',
                          choices=[
                              'citysim',
                              'construction',
                              'office',
                              'orchard',
                              'pipeline',
                              'solar_farm',
                              'warehouse',
                          ],
                          description='Gazebo World'),
    # Clearpath 工具链约定从 setup_path/robot.yaml 读取机器人平台、传感器和命名空间配置。
    DeclareLaunchArgument('setup_path',
                          default_value=[EnvironmentVariable('HOME'), '/clearpath/'],
                          description='Clearpath setup path'),
    # 仿真中必须让下游节点使用 Gazebo 发布的 /clock，否则 TF、传感器时间戳会与仿真时间错开。
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='use_sim_time'),
]

# 机器人 spawn 到世界中的初始位姿；yaw 使用 Gazebo/ROS 常用的弧度制。
for pose_element in ['x', 'y', 'yaw']:
    ARGUMENTS.append(DeclareLaunchArgument(pose_element, default_value='0.0',
                     description=f'{pose_element} component of the robot pose.'))

ARGUMENTS.append(DeclareLaunchArgument('z', default_value='0.3',
                 description='z component of the robot pose.'))


def generate_launch_description():
    # 入口 launch 只负责组合两个子流程：
    # 1. 启动 Gazebo 和指定 world；
    # 2. 生成/加载机器人配置，并把机器人实体 spawn 进 world。
    pkg_clearpath_gz = get_package_share_directory(
        'clearpath_gz')

    # 这里使用 PathJoinSubstitution，使路径解析延迟到 launch 执行期，兼容 install space。
    gz_sim_launch = PathJoinSubstitution(
        [pkg_clearpath_gz, 'launch', 'gz_sim.launch.py'])
    robot_spawn_launch = PathJoinSubstitution(
        [pkg_clearpath_gz, 'launch', 'robot_spawn.launch.py'])

    # Gazebo 子 launch 只需要知道加载哪个 world；具体资源路径和 /clock bridge 在其中处理。
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gz_sim_launch]),
        launch_arguments=[
            ('world', LaunchConfiguration('world'))
        ]
    )

    # spawn 子 launch 消费机器人配置、初始位姿、RViz 开关，并负责按需生成仿真参数。
    robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_spawn_launch]),
        launch_arguments=[
            ('use_sim_time', LaunchConfiguration('use_sim_time')),
            ('setup_path', LaunchConfiguration('setup_path')),
            ('world', LaunchConfiguration('world')),
            ('rviz', LaunchConfiguration('rviz')),
            ('x', LaunchConfiguration('x')),
            ('y', LaunchConfiguration('y')),
            ('z', LaunchConfiguration('z')),
            ('yaw', LaunchConfiguration('yaw'))]
    )

    # 先注册所有顶层参数，再依次追加 Gazebo 和机器人 spawn 动作。
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(gz_sim)
    ld.add_action(robot_spawn)
    return ld
