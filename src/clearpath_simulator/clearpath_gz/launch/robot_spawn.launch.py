# Copyright 2021 Clearpath Robotics, Inc.
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

from clearpath_config.clearpath_config import ClearpathConfig

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


ARGUMENTS = [
    # RViz 默认关闭，用户显式传 rviz:=true 时才打开机器人可视化。
    DeclareLaunchArgument('rviz', default_value='false',
                          choices=['true', 'false'],
                          description='Start rviz.'),
    # 所有由本文件启动的节点都应使用仿真时间，以 Gazebo /clock 为统一时间源。
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='use_sim_time'),
    # 传给 Gazebo topic prefix 的 world 名；必须与已经加载的 world 保持一致。
    DeclareLaunchArgument('world', default_value='warehouse',
                          description='Gazebo World'),
    # setup_path 目录中必须存在 robot.yaml；生成出的 launch/param 也会落在这个目录下。
    DeclareLaunchArgument('setup_path',
                          default_value=[EnvironmentVariable('HOME'), '/clearpath/'],
                          description='Clearpath setup path'),
    # true 时先重新生成 robot_description、语义描述、launch 和 bridge 参数，再 spawn。
    # false 时直接复用 setup_path 下已经生成好的文件，适合调试生成结果。
    DeclareLaunchArgument('generate',
                          default_value='true',
                          choices=['true', 'false'],
                          description='Generate parameters and launch files')
]

# spawn 位姿传给 `ros_gz_sim create`；z 默认略高于地面，避免初始化时与地面碰撞穿插。
for pose_element in ['x', 'y', 'yaw']:
    ARGUMENTS.append(DeclareLaunchArgument(pose_element, default_value='0.0',
                     description=f'{pose_element} component of the robot pose.'))

ARGUMENTS.append(DeclareLaunchArgument('z', default_value='0.15',
                 description='z component of the robot pose.'))


def launch_setup(context, *args, **kwargs):
    # 在 OpaqueFunction 中读取 launch 参数，才能把 setup_path 解析成真实字符串并读取 YAML。
    setup_path = LaunchConfiguration('setup_path')
    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')
    x, y, z = LaunchConfiguration('x'), LaunchConfiguration('y'), LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    generate = LaunchConfiguration('generate')

    # robot.yaml 是整个 Clearpath 生成链路的输入源：平台型号、namespace、传感器都从这里来。
    clearpath_config = ClearpathConfig(os.path.join(
        str(setup_path.perform(context)), 'robot.yaml'))

    # Gazebo model 名需要与 ros_gz topic prefix 对齐。空 namespace 时沿用 `robot`，
    # 非空 namespace 时使用 `<namespace>/robot`，让多机器人场景的 topic 不互相冲突。
    namespace = clearpath_config.system.namespace
    if namespace in ('', '/'):
        robot_name = 'robot'
    else:
        robot_name = namespace + '/robot'

    # RViz launch 在 clearpath_viz 包中，spawn 逻辑只在用户开启 rviz 时包含它。
    pkg_clearpath_viz = FindPackageShare('clearpath_viz')

    # 下列 service launch 文件由 clearpath_generator_gz/generate_launch 生成。
    rviz_launch = PathJoinSubstitution(
        [pkg_clearpath_viz, 'launch', 'view_robot.launch.py'])
    launch_file_platform_service = PathJoinSubstitution([
        setup_path, 'platform/launch', 'platform-service.launch.py'])
    launch_file_sensors_service = PathJoinSubstitution([
        setup_path, 'sensors/launch', 'sensors-service.launch.py'])

    # platform/sensors service launch 必须先包含，因为它们会启动 bridge 和 TF 节点；
    # prefix 指向 Gazebo 中机器人 base_link 下的 sensor 作用域，供传感器 bridge 拼接 gz topic。
    group_action_spawn_robot = GroupAction([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_platform_service]),
            launch_arguments=[
              ('prefix', ['/world/', world, '/model/', robot_name, '/link/base_link/sensor/'])]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_sensors_service]),
            launch_arguments=[
              ('prefix', ['/world/', world, '/model/', robot_name, '/link/base_link/sensor/'])]
        ),

        # ros_gz_sim create 从 robot_description 话题读取 URDF/SDF 描述，并在 Gazebo 中创建实体。
        Node(
            package='ros_gz_sim',
            executable='create',
            namespace=namespace,
            arguments=['-name', robot_name,
                       '-x', x,
                       '-y', y,
                       '-z', z,
                       '-Y', yaw,
                       '-topic', 'robot_description'],
            output='screen'
        ),
    ])

    # 生成顺序很重要：
    # description -> semantic_description -> gz launch -> gz param -> spawn。
    # 后一步通常依赖前一步生成的文件，所以用 OnProcessExit 串起来。
    node_generate_description = Node(
        package='clearpath_generator_common',
        executable='generate_description',
        name='generate_description',
        output='screen',
        condition=IfCondition(generate),
        arguments=['-s', setup_path]
    )

    node_generate_semantic_description = Node(
        package='clearpath_generator_common',
        executable='generate_semantic_description',
        name='generate_semantic_description',
        output='screen',
        condition=IfCondition(generate),
        arguments=['-s', setup_path]
    )

    node_generate_launch = Node(
        package='clearpath_generator_gz',
        executable='generate_launch',
        name='generate_launch',
        output='screen',
        condition=IfCondition(generate),
        arguments=['-s', setup_path]
    )

    node_generate_param = Node(
        package='clearpath_generator_gz',
        executable='generate_param',
        name='generate_param',
        output='screen',
        condition=IfCondition(generate),
        arguments=['-s', setup_path]
    )

    # 这些事件处理器把上面的生成节点串成线性流水线，避免并发写同一 setup_path。
    event_generate_description = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=node_generate_description,
            on_exit=[node_generate_semantic_description]
        )
    )

    event_generate_semantic_description = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=node_generate_semantic_description,
            on_exit=[node_generate_launch]
        )
    )

    event_generate_launch = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=node_generate_launch,
            on_exit=[node_generate_param]
        )
    )

    event_generate_param = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=node_generate_param,
            on_exit=[group_action_spawn_robot]
        )
    )

    # RViz 只依赖 namespace 和 use_sim_time；它不参与 Gazebo spawn，只做可视化。
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rviz_launch]),
        launch_arguments=[
            ('namespace', namespace),
            ('use_sim_time', use_sim_time)],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    # generate:=true 时完整再生成，适合 robot.yaml 改动后启动；generate:=false
    # 直接复用现有生成文件，适合检查已经落盘的 launch/param。
    do_generate = GroupAction(
        actions=[
            node_generate_description,
            event_generate_description,
            event_generate_semantic_description,
            event_generate_launch,
            event_generate_param,
            rviz
        ],
        condition=IfCondition(LaunchConfiguration('generate'))
    )

    do_not_generate = GroupAction(actions=[group_action_spawn_robot],
                                  condition=UnlessCondition(LaunchConfiguration('generate')))

    return [
        do_generate,
        do_not_generate
    ]


def generate_launch_description():
    # 顶层只注册参数并延迟到 launch_setup 中执行动态逻辑。
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
