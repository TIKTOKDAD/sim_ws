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

import os

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


ARGUMENTS = [
    # Gazebo 运行时也需要 use_sim_time 参数，保持接口与上层 simulation.launch.py 一致。
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='use_sim_time'),
    # 这里只接收世界名，例如 `warehouse`；真正传给 Gazebo 时会拼成 `warehouse.sdf`。
    DeclareLaunchArgument('world', default_value='warehouse',
                          description='Gazebo World'),
    # true 时给 Gazebo gz_args 加 `-r`，加载完成后自动开始仿真步进。
    DeclareLaunchArgument('auto_start', default_value='true',
                          choices=['true', 'false'],
                          description='Auto-start Gazebo simulation'),
]


def gz_launch(context, *args, **kwargs):

    # OpaqueFunction 可以在运行期读取 LaunchConfiguration 的真实值；
    # 这里需要判断 auto_start，所以不能只用静态 substitutions。
    pkg_clearpath_gz = get_package_share_directory(
        'clearpath_gz')
    pkg_ros_gz_sim = get_package_share_directory(
        'ros_gz_sim')

    # 复用 ros_gz_sim 提供的官方 Gazebo 启动文件，当前文件只负责组装 gz_args。
    gz_sim_launch = PathJoinSubstitution(
        [pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])

    # 固定 GUI 配置可以让用户每次进入相同的 Gazebo 面板布局。
    gui_config = PathJoinSubstitution(
        [pkg_clearpath_gz, 'config', 'gui.config'])

    auto_start_option = ''
    auto_start = LaunchConfiguration('auto_start').perform(context)
    if (auto_start == 'true'):
        auto_start_option = ' -r'

    # Gazebo 参数：
    # - `<world>.sdf`：从 GZ_SIM_RESOURCE_PATH 中查找；
    # - `-r`：可选自动运行；
    # - `-v 4`：输出较详细日志；
    # - `--gui-config`：加载本包的 GUI 布局。
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gz_sim_launch]),
        launch_arguments=[
            ('gz_args', [LaunchConfiguration('world'),
                         '.sdf',
                         auto_start_option,
                         ' -v 4',
                         ' --gui-config ',
                         gui_config])
        ]
    )

    return [gz_sim]


def generate_launch_description():

    # 资源路径必须在 Gazebo 启动前设置好，否则 world 中的 model:// URI 会解析失败。
    pkg_clearpath_gz = get_package_share_directory(
        'clearpath_gz')

    # 把所有已 source 的 ROS package share 目录追加进去，允许 world 引用其他包安装的资源。
    packages_paths = [os.path.join(p, 'share') for p in os.getenv('AMENT_PREFIX_PATH').split(':')]

    # Gazebo Harmonic 使用 GZ_SIM_RESOURCE_PATH。前两个路径是本包 world/mesh，
    # 后续路径来自 AMENT_PREFIX_PATH，覆盖 overlay workspace 的常见资源查找场景。
    unitree_resource_paths = []
    try:
        pkg_unitree_go2_description = get_package_share_directory(
            'unitree_go2_description')
        unitree_worlds = os.path.join(pkg_unitree_go2_description, 'worlds')
        unitree_resource_paths = [
            unitree_worlds + ':',
            os.path.join(unitree_worlds, 'citysim_models') + ':',
        ]
    except PackageNotFoundError:
        pass

    gz_sim_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(pkg_clearpath_gz, 'worlds') + ':',
            os.path.join(pkg_clearpath_gz, 'meshes') + ':',
            *unitree_resource_paths,
            ':' + ':'.join(packages_paths)])

    # 把 Gazebo /clock 桥接到 ROS。所有 use_sim_time=true 的节点都会依赖这个话题。
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ]
    )

    # 动作顺序有意义：先设置资源路径，再启动 Gazebo，最后启动 /clock bridge。
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(gz_sim_resource_path)
    ld.add_action(OpaqueFunction(function=gz_launch))
    ld.add_action(clock_bridge)
    return ld
