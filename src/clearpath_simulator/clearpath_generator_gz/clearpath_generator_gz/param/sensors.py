# Software License Agreement (BSD)
#
# @author    Luis Cameor <lcamero@clearpathrobotics.com>
# @copyright (c) 2024, Clearpath Robotics, Inc., All rights reserved.
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

from clearpath_config.common.utils.yaml import write_yaml
from clearpath_config.sensors.types.cameras import (
    BaseCamera,
    FlirBlackfly,
    IntelRealsense,
    StereolabsZed,
)
from clearpath_config.sensors.types.gps import (
    BaseGPS,
    Garmin18x,
    NovatelSmart6,
    NovatelSmart7,
    SwiftNavDuro,
)
from clearpath_config.sensors.types.imu import BaseIMU, CHRoboticsUM6, Microstrain, RedshiftUM7
from clearpath_config.sensors.types.lidars_2d import BaseLidar2D, HokuyoUST, SickLMS1XX
from clearpath_config.sensors.types.lidars_3d import (
    BaseLidar3D,
    OusterOS1,
    SeyondLidar,
    VelodyneLidar,
)
from clearpath_config.sensors.types.sensor import BaseSensor


class MessageType():
    """集中定义 ROS 与 Gazebo 消息类型名。

    ros_gz_bridge 的 YAML 配置要求写完整类型字符串。集中维护可以降低拼写错误风险，
    也让新增传感器时更容易复用已有类型。
    """

    class ROS():
        """ROS 2 topic 类型。"""
        CAMERA_INFO = 'sensor_msgs/msg/CameraInfo'
        IMAGE = 'sensor_msgs/msg/Image'
        IMU = 'sensor_msgs/msg/Imu'
        LASER_SCAN = 'sensor_msgs/msg/LaserScan'
        NAVSAT = 'sensor_msgs/msg/NavSatFix'
        POINT_CLOUD = 'sensor_msgs/msg/PointCloud2'

    class GZ():
        """Gazebo Transport topic 类型。"""
        CAMERA_INFO = 'gz.msgs.CameraInfo'
        IMAGE = 'gz.msgs.Image'
        IMU = 'gz.msgs.IMU'
        LASER_SCAN = 'gz.msgs.LaserScan'
        NAVSAT = 'gz.msgs.NavSat'
        POINT_CLOUD = 'gz.msgs.PointCloudPacked'


class RemapFile():
    """封装 ros_gz_bridge 参数文件的写入。

    ros_gz_bridge 的 parameter_bridge 可以读取一个 YAML 列表，每一项描述：
    ROS topic、Gazebo topic、两侧消息类型以及桥接方向。
    """

    def __init__(
            self,
            name: str,
            path: str = 'config'
            ) -> None:
        self.path = os.path.join(path, name)
        # remappings 中的每个 dict 会原样写入 YAML。
        self.remappings = []

    def add(
            self,
            ros_topic,
            gz_topic,
            ros_type,
            gz_type,
            direction='GZ_TO_ROS',
            ) -> dict:
        """追加一条 bridge 映射。默认方向是 Gazebo -> ROS，符合大多数传感器输出。"""
        self.remappings.append({
            'ros_topic_name': ros_topic,
            'gz_topic_name': gz_topic,
            'ros_type_name': ros_type,
            'gz_type_name': gz_type,
            'direction': direction
        })

    def write(self) -> None:
        """把 remappings 写成 YAML 文件，供 ros_gz_bridge parameter_bridge 加载。"""
        write_yaml(self.path, self.remappings)


class SensorParam():
    """按传感器型号选择对应的 bridge 参数生成器。"""

    class BaseParam():
        """所有传感器参数生成类的基类。

        子类只需要调用 `self.param_file.add(...)` 登记 topic 映射；
        路径、命名空间和 YAML 写入由基类统一处理。
        """

        TOPIC_NAMESPACE = 'sensors'

        def __init__(
                self,
                sensor: BaseSensor,
                namespace: str,
                param_path: str,
                namespace_prefix: str = None,
                ) -> None:
            self.sensor = sensor
            self.namespace = namespace
            self.param_path = param_path
            self.ros_ns_prefix = namespace_prefix
            # 每个传感器生成一个同名 YAML，例如 camera_0.yaml、lidar_0.yaml。
            self.param_file = RemapFile(
                name=self.sensor.name + '.yaml',
                path=self.param_path
            )

        def generate_config(self):
            """写出 YAML，并在生成器日志中打印路径，便于用户定位生成结果。"""
            self.param_file.write()
            print('Generated config: {0}'.format(self.param_file.path))

        def get_ros_topic(self, topic: str) -> str:
            """生成 ROS 侧 topic。

            namespace_prefix 用于平台内置传感器，例如 `sensors/imu_0/data`；
            外接传感器则直接使用 `<sensor>/<topic>`，因为 launch namespace 已经包含 sensors/。
            """
            if self.ros_ns_prefix:
                return os.path.join(
                    self.ros_ns_prefix,
                    self.sensor.name,
                    topic
                )
            else:
                return os.path.join(
                    self.sensor.name,
                    topic
                )

        def get_gz_topic(self, topic: str = None) -> str:
            """生成 Gazebo 侧 topic。

            Gazebo topic 必须是绝对路径，并包含机器人 namespace、sensors 命名层级和传感器名。
            """
            namespace = os.path.join(
                '/',
                self.namespace,
                self.TOPIC_NAMESPACE,
                self.sensor.name,
            )
            if topic:
                return os.path.join(namespace, topic)
            else:
                return namespace

    class Lidar2dParam(BaseParam):
        """2D 激光雷达：只桥接 LaserScan。"""

        def __init__(
                self,
                sensor: BaseLidar2D,
                namespace: str,
                param_path: str,
                namespace_prefix: str = None,
                ) -> None:
            super().__init__(sensor, namespace, param_path, namespace_prefix)

            self.param_file.add(
                ros_topic=self.get_ros_topic('scan'),
                gz_topic=self.get_gz_topic('scan'),
                ros_type=MessageType.ROS.LASER_SCAN,
                gz_type=MessageType.GZ.LASER_SCAN,
            )

    class Lidar3dParam(BaseParam):
        """3D 激光雷达：同时桥接 scan 和点云。"""

        def __init__(
                self,
                sensor: BaseLidar3D,
                namespace: str,
                param_path: str,
                namespace_prefix: str = None,
                ) -> None:
            super().__init__(sensor, namespace, param_path, namespace_prefix)

            self.param_file.add(
                ros_topic=self.get_ros_topic('scan'),
                gz_topic=self.get_gz_topic('scan'),
                ros_type=MessageType.ROS.LASER_SCAN,
                gz_type=MessageType.GZ.LASER_SCAN,
            )

            self.param_file.add(
                ros_topic=self.get_ros_topic('points'),
                gz_topic=self.get_gz_topic('scan/points'),
                ros_type=MessageType.ROS.POINT_CLOUD,
                gz_type=MessageType.GZ.POINT_CLOUD,
            )

    class ImuParam(BaseParam):
        """IMU：桥接滤波前后的 data/data_raw，保持真实平台常见 topic 结构。"""

        def __init__(
                self,
                sensor: BaseIMU,
                namespace: str,
                param_path: str,
                namespace_prefix: str = None,
                ) -> None:
            super().__init__(sensor, namespace, param_path, namespace_prefix)

            self.param_file.add(
                ros_topic=self.get_ros_topic('data'),
                gz_topic=self.get_gz_topic('data'),
                ros_type=MessageType.ROS.IMU,
                gz_type=MessageType.GZ.IMU,
            )
            self.param_file.add(
                ros_topic=self.get_ros_topic('data_raw'),
                gz_topic=self.get_gz_topic('data_raw'),
                ros_type=MessageType.ROS.IMU,
                gz_type=MessageType.GZ.IMU,
            )

    class CameraParam(BaseParam):
        """普通相机：parameter_bridge 只负责 camera_info，图像由 ros_gz_image 处理。"""

        def __init__(
                self,
                sensor: BaseCamera,
                namespace: str,
                param_path: str,
                namespace_prefix: str = None,
                ) -> None:
            super().__init__(sensor, namespace, param_path, namespace_prefix)

            self.param_file.add(
                ros_topic=self.get_ros_topic('color/camera_info'),
                gz_topic=self.get_gz_topic('camera_info'),
                ros_type=MessageType.ROS.CAMERA_INFO,
                gz_type=MessageType.GZ.CAMERA_INFO,
            )

    class RGBDCameraParam(CameraParam):
        """RGB-D 相机：在普通相机基础上增加点云和深度 camera_info。"""

        def __init__(
                self,
                sensor: BaseCamera,
                namespace: str,
                param_path: str,
                namespace_prefix: str = None,
                ) -> None:
            super().__init__(sensor, namespace, param_path, namespace_prefix)

            self.param_file.add(
                ros_topic=self.get_ros_topic('points'),
                gz_topic=self.get_gz_topic('points'),
                ros_type=MessageType.ROS.POINT_CLOUD,
                gz_type=MessageType.GZ.POINT_CLOUD,
            )

            self.param_file.add(
                ros_topic=self.get_ros_topic('depth/camera_info'),
                gz_topic=self.get_gz_topic('camera_info'),
                ros_type=MessageType.ROS.CAMERA_INFO,
                gz_type=MessageType.GZ.CAMERA_INFO,
            )

    class GPSParam(BaseParam):
        """GPS：把 Gazebo NavSat 消息桥接成 ROS NavSatFix。"""

        def __init__(
                self,
                sensor: BaseGPS,
                namespace: str,
                param_path: str,
                namespace_prefix: str = None,
                ) -> None:
            super().__init__(sensor, namespace, param_path, namespace_prefix)

            self.param_file.add(
                ros_topic=self.get_ros_topic('fix'),
                gz_topic=self.get_gz_topic('navsat'),
                ros_type=MessageType.ROS.NAVSAT,
                gz_type=MessageType.GZ.NAVSAT,
            )

    # 传感器型号到参数生成类的分发表。新增传感器模型时，如果 topic 结构与现有类型一致，
    # 只需把 SENSOR_MODEL 映射到对应 Param 类；如果 topic 不同，再新增一个子类。
    MODEL = {
        HokuyoUST.SENSOR_MODEL: Lidar2dParam,
        SickLMS1XX.SENSOR_MODEL: Lidar2dParam,
        FlirBlackfly.SENSOR_MODEL: CameraParam,
        IntelRealsense.SENSOR_MODEL: RGBDCameraParam,
        StereolabsZed.SENSOR_MODEL: RGBDCameraParam,
        BaseIMU.SENSOR_MODEL: ImuParam,
        CHRoboticsUM6.SENSOR_MODEL: ImuParam,
        Microstrain.SENSOR_MODEL: ImuParam,
        RedshiftUM7.SENSOR_MODEL: ImuParam,
        VelodyneLidar.SENSOR_MODEL: Lidar3dParam,
        OusterOS1.SENSOR_MODEL: Lidar3dParam,
        SeyondLidar.SENSOR_MODEL: Lidar3dParam,
        Garmin18x.SENSOR_MODEL: GPSParam,
        NovatelSmart6.SENSOR_MODEL: GPSParam,
        NovatelSmart7.SENSOR_MODEL: GPSParam,
        SwiftNavDuro.SENSOR_MODEL: GPSParam,
    }

    def __new__(cls,
                sensor: BaseSensor,
                namespace: str,
                param_path: str,
                namespace_prefix: str = None) -> BaseParam:
        # 使用工厂模式：调用 SensorParam(...) 时实际返回具体的 BaseParam 子类实例。
        # 未登记型号回退到 BaseParam，仅生成空 YAML，避免未知传感器直接让生成流程失败。
        return SensorParam.MODEL.setdefault(sensor.SENSOR_MODEL, SensorParam.BaseParam)(
            sensor, namespace, param_path, namespace_prefix)
