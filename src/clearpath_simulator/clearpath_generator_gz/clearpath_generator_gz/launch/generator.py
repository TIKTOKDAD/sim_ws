#!/usr/bin/env python3

# Software License Agreement (BSD)
#
# @author    Roni Kreinin <rkreinin@clearpathrobotics.com>
# @copyright (c) 2023, Clearpath Robotics, Inc., All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice,
#   this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of Clearpath Robotics nor the names of its contributors
#   may be used to endorse or promote products derived from this software
#   without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Redistribution and use in source and binary forms, with or without
# modification, is not permitted without the express permission
# of Clearpath Robotics.
import os

from clearpath_config.common.types.platform import Platform
from clearpath_generator_common.common import LaunchFile
from clearpath_generator_common.launch.generator import LaunchGenerator
from clearpath_generator_common.launch.writer import LaunchWriter
from clearpath_generator_gz.launch.sensors import SensorLaunch


class GzLaunchGenerator(LaunchGenerator):
    """生成 Gazebo 仿真需要的 ROS 2 launch 文件。

    基类 `LaunchGenerator` 已经完成 robot.yaml 解析、输出目录组织和通用
    platform/sensors/manipulators launch 文件对象创建；本类只补充 Gazebo
    仿真特有的 bridge、IMU filter、GPS bridge 等节点。
    """

    # ros_gz_bridge 的方向语法：
    #   ROS_MSG[gz.msg：Gazebo -> ROS
    #   ROS_MSG]gz.msg：ROS -> Gazebo
    # 这里保留为常量，避免在多个 bridge argument 中重复硬编码。
    GZ_TO_ROS_TWIST = '@geometry_msgs/msg/TwistStamped[gz.msgs.Twist'
    ROS_TO_GZ_TWIST = '@geometry_msgs/msg/TwistStamped]gz.msgs.Twist'
    GZ_TO_ROS_TF = '@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'

    def __init__(self, setup_path: str = '/etc/clearpath/') -> None:
        super().__init__(setup_path)

        # 仿真生成出的 platform launch 一律启用 use_sim_time；
        # 如果基类已经放了该参数，这里就把默认值改成 true。
        for i, arg in enumerate(self.platform_launch_file.args):
            if arg[0] == 'use_sim_time':
                self.platform_launch_file.args[i] = ('use_sim_time', 'true')

        # 仿真中默认也加载 manipulation controllers，便于带机械臂的平台直接运行。
        self.platform_launch_file.args.append(
            ('use_manipulation_controllers', 'true')
        )

        # Gazebo model 名和 ROS namespace 需要保持一致：
        # - 无 namespace：/model/robot/...
        # - 有 namespace：/model/<namespace>/robot/...
        if self.namespace in ('', '/'):
            self.robot_name = 'robot'
        else:
            self.robot_name = self.namespace + '/robot'

        # cmd_vel 有两条 bridge：
        # 1. Gazebo world 中的 /cmd_vel -> ROS cmd_vel，兼容外部仿真控制输入；
        # 2. ROS platform/cmd_vel -> Gazebo /model/<robot>/cmd_vel，驱动 Gazebo 模型。
        if self.namespace in ('', '/'):
            cmd_vel_bridge_arg = '/cmd_vel' + self.GZ_TO_ROS_TWIST
            cmd_vel_bridge_remap = ('/cmd_vel', 'cmd_vel')
        else:
            cmd_vel_bridge_arg = self.namespace + '/cmd_vel' + self.GZ_TO_ROS_TWIST
            cmd_vel_bridge_remap = (self.namespace + '/cmd_vel', 'cmd_vel')

        cmd_vel_robot_bridge_arg = '/model/' + self.robot_name + '/cmd_vel' + self.ROS_TO_GZ_TWIST
        cmd_vel_robot_bridge_remap = (
            '/model/' + self.robot_name + '/cmd_vel',
            'platform/cmd_vel'
          )

        self.cmd_vel_node = LaunchFile.Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='cmd_vel_bridge',
            namespace=self.namespace,
            parameters=[{'use_sim_time': True}],
            arguments=[
                cmd_vel_bridge_arg,
                cmd_vel_robot_bridge_arg
            ],
            remappings=[
                cmd_vel_bridge_remap,
                cmd_vel_robot_bridge_remap
            ])

        # Gazebo 会发布模型内部的 odom/base_link pose；这里桥接到 ROS /tf，
        # 让 robot_state_publisher、导航和可视化能看到仿真中的底盘位姿。
        self.odom_base_node = LaunchFile.Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='odom_base_tf_bridge',
            namespace=self.namespace,
            parameters=[{'use_sim_time': True}],
            arguments=[
                '/model/' + self.robot_name + '/tf' + self.GZ_TO_ROS_TF
            ],
            remappings=[
                ('/model/' + self.robot_name + '/tf', 'tf')
            ])

        # 某些平台在 Gazebo 模型中自带 IMU。bridge 参数文件由 param/generator.py 生成。
        self.imu_0_bridge_node = LaunchFile.Node(
          name='imu_0_gz_bridge',
          package='ros_gz_bridge',
          executable='parameter_bridge',
          namespace=self.namespace,
          parameters=[{
              'use_sim_time': True,
              'config_file': os.path.join(
                  self.sensors_params_path, 'imu_0.yaml')
          }]
        )

        # IMU filter 将 raw IMU 和磁力计数据融合成 orientation。这里的参数文件来自平台参数目录，
        # remapping 到 sensors/imu_0 下，保持与真实机器人 topic 结构一致。
        self.imu_filter_arg = LaunchFile.LaunchArg(
            'imu_filter',
            default_value=os.path.join(self.platform_params_path, 'imu_filter.yaml')
        )
        imu_filter_variable = LaunchFile.Variable('imu_filter')

        self.imu_filter_node = LaunchFile.Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter_node',
            namespace=self.namespace,
            parameters=[imu_filter_variable],
            remappings=[
              ('imu/data_raw', 'sensors/imu_0/data_raw'),
              ('imu/mag', 'sensors/imu_0/magnetic_field'),
              ('imu/data', 'sensors/imu_0/data'),
              ('/tf', 'tf'),
            ],
        )

        # J100 等平台带内置 GPS；仿真中通过 NavSat bridge 输出 NavSatFix。
        self.gps_0_bridge_node = LaunchFile.Node(
          name='gps_0_gz_bridge',
          package='ros_gz_bridge',
          executable='parameter_bridge',
          namespace=self.namespace,
          parameters=[{
              'use_sim_time': True,
              'config_file': os.path.join(
                  self.sensors_params_path, 'gps_0.yaml')
          }]
        )

        # 所有平台都需要速度命令和 odom/base_link TF bridge。
        self.common_platform_components = [
            self.cmd_vel_node,
            self.odom_base_node
        ]

        # 平台型号决定是否额外启动内置 IMU/GPS bridge。这里的枚举与 clearpath_config
        # 中的 Platform 保持一致，新增平台时通常也要在这里登记仿真组件。
        self.platform_components = {
            Platform.J100: self.common_platform_components + [
                self.imu_0_bridge_node,
                self.imu_filter_arg,
                self.imu_filter_node,
                self.gps_0_bridge_node,
            ],
            Platform.A200: self.common_platform_components,
            Platform.A300: self.common_platform_components,
            Platform.DD100: self.common_platform_components + [
                self.imu_0_bridge_node,
                self.imu_filter_arg,
                self.imu_filter_node,
            ],
            Platform.DD150:  self.common_platform_components + [
                self.imu_0_bridge_node,
                self.imu_filter_arg,
                self.imu_filter_node,
            ],
            Platform.DO100: self.common_platform_components + [
                self.imu_0_bridge_node,
                self.imu_filter_arg,
                self.imu_filter_node,
            ],
            Platform.DO150: self.common_platform_components + [
                self.imu_0_bridge_node,
                self.imu_filter_arg,
                self.imu_filter_node,
            ],
            Platform.GENERIC: self.common_platform_components,
            Platform.R100: self.common_platform_components + [
                self.imu_0_bridge_node,
                self.imu_filter_arg,
                self.imu_filter_node,
            ],
            Platform.W200: self.common_platform_components + [
                self.imu_0_bridge_node,
                self.imu_filter_arg,
                self.imu_filter_node,
            ],
        }

    def generate_sensors(self) -> None:
        """为 robot.yaml 中启用 launch 的每个传感器生成单独 launch，并汇总到 service launch。"""
        sensors_service_launch_writer = LaunchWriter(self.sensors_service_launch_file)
        sensors = self.clearpath_config.sensors.get_all_sensors()

        for sensor in sensors:
            if sensor.get_launch_enabled():
                # 每个传感器都有自己的 bridge launch，便于单独调试 topic 和 remapping。
                sensor_launch = SensorLaunch(
                        sensor,
                        self.namespace,
                        self.sensors_launch_path,
                        self.sensors_params_path)
                sensor_launch.generate()
                # Add sensor to top level sensors launch file
                sensors_service_launch_writer.add_launch_file(sensor_launch.launch_file)

        sensors_service_launch_writer.generate_file()

    def generate_platform(self) -> None:
        """生成平台 service launch：包含通用平台 launch 和 Gazebo 专用 bridge 节点。"""
        platform_service_launch_writer = LaunchWriter(self.platform_service_launch_file)
        platform_service_launch_writer.add_launch_file(self.platform_launch_file)

        # Platform components
        for component in self.platform_components[self.platform_model]:
            platform_service_launch_writer.add(component)

        platform_service_launch_writer.generate_file()

    def generate_manipulators(self) -> None:
        """生成 manipulator service launch。

        当前 Gazebo 侧没有额外动作要追加，但仍生成空/占位 service launch，
        让上层启动流程可以稳定引用该文件。
        """
        manipulators_service_launch_writer = LaunchWriter(self.manipulators_service_launch_file)
        manipulators_service_launch_writer.generate_file()
