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
from clearpath_config.sensors.types.gps import Garmin18x
from clearpath_config.sensors.types.imu import BaseIMU
from clearpath_generator_common.param.generator import ParamGenerator
from clearpath_generator_common.param.manipulators import ManipulatorParam
from clearpath_generator_common.param.platform import PlatformParam
from clearpath_generator_gz.param.sensors import SensorParam

# 平台内置传感器登记表。robot.yaml 中未显式配置这些传感器时，仿真仍需要为
# Gazebo 模型自带的 IMU/GPS 生成 bridge 参数文件。
PLATFORMS = {
    Platform.A200: {'imu': False, 'gps': False},
    Platform.A300: {'imu': False, 'gps': False},
    Platform.J100: {'imu': True, 'gps': True},
    Platform.DD100: {'imu': True, 'gps': False},
    Platform.DD150: {'imu': True, 'gps': False},
    Platform.DO100: {'imu': True, 'gps': False},
    Platform.DO150: {'imu': True, 'gps': False},
    Platform.R100: {'imu': True, 'gps': False},
    Platform.W200: {'imu': True, 'gps': False},
    Platform.GENERIC: {'imu': False, 'gps': False},
}


class GzParamGenerator(ParamGenerator):
    """生成 Gazebo 仿真专用参数文件。

    通用平台参数由 clearpath_generator_common 负责，本类追加 Gazebo bridge
    所需的 YAML 配置，让 ros_gz_bridge 知道每个 ROS topic 与 Gazebo topic 的映射关系。
    """

    def generate_sensors(self) -> None:
        """为 robot.yaml 中声明的外接传感器生成 bridge remapping YAML。"""
        sensors = self.clearpath_config.sensors.get_all_sensors()
        if sensors:
            os.makedirs(self.sensors_params_path, exist_ok=True)
            for sensor in sensors:
                # SensorParam 会根据 sensor.SENSOR_MODEL 自动选择对应的参数生成类。
                sensor_param = SensorParam(
                    sensor,
                    self.clearpath_config.get_namespace(),
                    self.sensors_params_path)
                sensor_param.generate_config()

    def generate_platform(self) -> None:
        """生成平台参数，以及平台内置 IMU/GPS 的仿真 bridge 参数。"""
        # 先复用通用平台参数生成器，并强制 use_sim_time=true。
        for param in PlatformParam.PARAMETERS:
            platform_param = PlatformParam(
                param,
                self.clearpath_config,
                self.platform_params_path)
            platform_param.generate_parameters(use_sim_time=True)
            platform_param.generate_parameter_file()

        # 如果平台模型在 Gazebo 中自带 IMU，则生成 sensors/imu_0.yaml 供 launch/generator.py 使用。
        if PLATFORMS[self.clearpath_config.get_platform_model()]['imu']:
            os.makedirs(self.sensors_params_path, exist_ok=True)
            sensor_param = SensorParam(
                BaseIMU(idx=0),
                self.clearpath_config.get_namespace(),
                self.sensors_params_path,
                'sensors'
            )
            sensor_param.generate_config()

        # 目前只有部分平台自带 GPS，例如 J100；生成出的 gps_0.yaml 会被 gps bridge 节点加载。
        if PLATFORMS[self.clearpath_config.get_platform_model()]['gps']:
            os.makedirs(self.sensors_params_path, exist_ok=True)
            sensor_param = SensorParam(
                Garmin18x(idx=0),
                self.clearpath_config.get_namespace(),
                self.sensors_params_path,
                'sensors',
            )
            sensor_param.generate_config()

    def generate_manipulators(self) -> None:
        """生成机械臂相关参数。仿真中同样需要 MoveIt 使用仿真时间。"""
        moveit = ManipulatorParam(
            ManipulatorParam.MOVEIT,
            self.clearpath_config,
            self.manipulators_params_path)
        moveit.generate_parameters(use_sim_time=True)
        moveit.generate_parameter_file()
