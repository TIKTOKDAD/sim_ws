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

from clearpath_config.sensors.types.cameras import (
    AxisCamera,
    BaseCamera,
    IntelRealsense,
    StereolabsZed
)
from clearpath_config.sensors.types.ptu import BasePTU
from clearpath_config.sensors.types.sensor import BaseSensor
from clearpath_generator_common.common import LaunchFile, ParamFile
from clearpath_generator_common.launch.writer import LaunchWriter


class SensorLaunch():
    """为单个传感器生成 Gazebo bridge launch 文件。

    输入来自 clearpath_config 的传感器对象；输出是一个 launch 文件，里面通常包含：
    - ros_gz_bridge parameter_bridge，用参数文件描述 topic 映射；
    - sensor link 到 Gazebo sensor frame 的静态 TF；
    - 相机图像、深度图、PTZ/PTU 等特殊传感器的辅助节点。
    """

    # 传感器 ROS topic 都放在 sensors/ 下，模拟真实 Clearpath 平台的命名层级。
    TOPIC_NAMESPACE = 'sensors/'

    # Launch arguments
    # 这两个字符串保留给生成 launch 文件时复用，避免调用方手写参数名。
    PARAMETERS = 'parameters'
    NAMESPACE = 'namespace'

    # ros_gz_bridge 方向常量。`[` 表示 Gazebo -> ROS，`]` 表示 ROS -> Gazebo。
    GZ_TO_ROS_LASERSCAN = '@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
    GZ_TO_ROS_POINTCLOUD = '@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'
    GZ_TO_ROS_IMAGE = '@sensor_msgs/msg/Image[gz.msgs.Image'
    GZ_TO_ROS_CAMERA_INFO = '@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
    GZ_TO_ROS_IMU = '@sensor_msgs/msg/Imu[gz.msgs.IMU'
    GZ_TO_ROS_NAVSAT = '@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat'
    GZ_TO_ROS_JOINTSTATE = '@sensor_msgs/msg/JointState[gz.msgs.Model'

    ROS_TO_GZ_FLOAT = '@std_msgs/msg/Float64]gz.msgs.Double'

    # RGB-D 相机除了 color image，还需要深度图和点云 bridge。
    RGBD_CAMERAS = [
        IntelRealsense.SENSOR_MODEL,
        StereolabsZed.SENSOR_MODEL
    ]

    # PTZ 相机需要 pan/tilt joint controller 和 action server 的额外仿真适配。
    PTZ_CAMERAS = [
        AxisCamera.SENSOR_MODEL
    ]

    def __init__(
          self,
          sensor: BaseSensor,
          robot_namespace: str,
          launch_path: str,
          param_path: str) -> None:
        self.sensor = sensor
        self._robot_namespace = robot_namespace
        self.parameters = ParamFile(self.name, path=param_path)
        # prefix 由 robot_spawn.launch.py 传入，指向 Gazebo world/model/link/sensor 路径。
        self.prefix_launch_arg = LaunchFile.LaunchArg('prefix')

        # 单个传感器独立生成一个 launch，最后由 sensors-service.launch.py 汇总包含。
        self.launch_file = LaunchFile(
            self.name,
            path=launch_path)

        # PTUs don't have a single `<name>_link` frame (they expose
        # base/pan/tilt/mount links driven by revolute joints), so the
        # sensor-to-gz static transform is neither valid nor required.
        if self.sensor.SENSOR_TYPE == BasePTU.SENSOR_TYPE:
            self.static_tf_node = None
        else:
            # 传感器在 ROS 描述里通常有 `<sensor>_link`，Gazebo sensor topic 则带
            # `<robot>/base_link/<sensor>` 前缀；静态 TF 把两套 frame 对齐。
            self.static_tf_node = LaunchFile.get_static_tf_node(
                name=self.name,
                namespace=self._robot_namespace,
                parent_link=self.name + '_link',
                child_link=self.robot_name + '/base_link/' + self.name,
                use_sim_time=True
            )

        # 通用 bridge 节点。具体 topic/type/remap 不直接写在 launch 中，而是读取
        # param/sensors.py 生成的 YAML，便于每种传感器复用同一个节点模板。
        self.gz_bridge_node = LaunchFile.Node(
            name=self.name + '_gz_bridge',
            namespace=self.namespace,
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{
                'use_sim_time': True,
                'config_file': self.parameters.full_path
            }]
        )

        # 某些传感器需要通用 parameter_bridge 之外的节点，例如 ros_gz_image 或 PTZ 控制器。
        self.extra_gz_nodes = []

        # 普通相机图像使用 ros_gz_image/image_bridge，比 parameter_bridge 更适合 image transport
        # 相关的 compressed、compressedDepth、theora topic。
        if (
            self.sensor.SENSOR_TYPE == BaseCamera.SENSOR_TYPE and
            self.sensor.SENSOR_MODEL not in self.PTZ_CAMERAS
        ):
            image_ns = '/' + self.namespace + self.name
            image_topic = image_ns + '/image'

            image_bridge_node = LaunchFile.Node(
                name=self.name + '_gz_image_bridge',
                namespace=self.namespace,
                package='ros_gz_image',
                executable='image_bridge',
                parameters=[{
                    'use_sim_time': True,
                }],
                arguments=[image_topic],
                remappings=[
                    (image_topic, image_ns + '/color/image'),
                    (image_topic + '/compressed', image_ns + '/color/compressed'),
                    (image_topic + '/compressedDepth', image_ns + '/color/compressedDepth'),
                    (image_topic + '/theora', image_ns + '/color/theora'),
                ]
            )
            self.extra_gz_nodes.append(image_bridge_node)

        # RGB-D 相机额外桥接 depth_image，并 remap 到 ROS 侧常见的 depth/* topic。
        if self.sensor.SENSOR_MODEL in self.RGBD_CAMERAS:
            depth_ns = '/' + self.namespace + self.name
            depth_topic = depth_ns + '/depth_image'

            depth_bridge_node = LaunchFile.Node(
                name=self.name + '_gz_depth_bridge',
                namespace=self.namespace,
                package='ros_gz_image',
                executable='image_bridge',
                parameters=[{
                    'use_sim_time': True,
                }],
                arguments=[depth_topic],
                remappings=[
                    (depth_topic, depth_ns + '/depth/image'),
                    (depth_topic + '/compressed', depth_ns + '/depth/compressed'),
                    (depth_topic + '/compressedDepth', depth_ns + '/depth/compressedDepth'),
                    (depth_topic + '/theora', depth_ns + '/depth/theora'),
                ]
            )
            self.extra_gz_nodes.append(depth_bridge_node)

        # PTZ 相机的图像先进入内部 `_/` topic，由 ptz_controller_node 做数字变焦后再输出到
        # color/image；pan/tilt 速度命令则通过 Float64 bridge 驱动 Gazebo joint controller。
        if self.sensor.SENSOR_MODEL in self.PTZ_CAMERAS:
            image_ns = '/' + self.namespace + self.name
            image_topic = image_ns + '/image'

            image_bridge_node = LaunchFile.Node(
                name=self.name + '_gz_image_bridge',
                namespace=self.namespace,
                package='ros_gz_image',
                executable='image_bridge',
                parameters=[{
                    'use_sim_time': True,
                }],
                arguments=[image_topic],
                remappings=[
                    (image_topic, image_ns + '/_/image_raw'),
                    (image_topic + '/compressed', image_ns + '/_/compressed'),
                    (image_topic + '/compressedDepth', image_ns + '/_/compressedDepth'),
                    (image_topic + '/theora', image_ns + '/_/theora'),
                ]
            )
            self.extra_gz_nodes.append(image_bridge_node)

            cmd_ns = '/' + self.namespace + self.name
            cmd_bridge_node = LaunchFile.Node(
                name=self.name + '_gz_cmd_bridge',
                namespace=self.namespace,
                package='ros_gz_bridge',
                executable='parameter_bridge',
                parameters=[{'use_sim_time': True}],
                arguments=[
                    cmd_ns + '/cmd_pan_vel' + self.ROS_TO_GZ_FLOAT,
                    cmd_ns + '/cmd_tilt_vel' + self.ROS_TO_GZ_FLOAT,
                    cmd_ns + '/pan_joint_state' + self.GZ_TO_ROS_JOINTSTATE,
                    cmd_ns + '/tilt_joint_state' + self.GZ_TO_ROS_JOINTSTATE,
                ],
                remappings=[
                    (cmd_ns + '/pan_joint_state', '/' + self._robot_namespace + '/platform/joint_states'),  # noqa:E501
                    (cmd_ns + '/tilt_joint_state', '/' + self._robot_namespace + '/platform/joint_states'),  # noqa:E501
                    (cmd_ns + '/cmd_pan_vel', cmd_ns + '/_/cmd_pan_vel'),
                    (cmd_ns + '/cmd_tilt_vel', cmd_ns + '/_/cmd_tilt_vel'),
                ],
            )
            self.extra_gz_nodes.append(cmd_bridge_node)

            # ptz_controller_node 实现 PTZ action server，负责：
            # - 把 action/velocity 命令转换为 pan/tilt 速度 topic；
            # - 订阅 joint_states 估算当前姿态；
            # - 对输入图像做中心裁剪模拟 zoom。
            ptz_node = LaunchFile.Node(
                name='ptz_action_server_node',
                namespace=image_ns,
                package='clearpath_generator_gz',
                executable='ptz_controller_node',
                parameters=[
                    {'use_sim_time': True},
                    {'camera_name': self.name},
                ],
                remappings=[
                    ('image_in', image_ns + '/_/image_raw'),
                    ('image_out', image_ns + '/color/image'),
                    ('cmd/velocity', image_ns + '/cmd/velocity'),
                    ('joint_states', '/' + self._robot_namespace + '/platform/joint_states'),
                    ('cmd_pan_vel', cmd_ns + '/_/cmd_pan_vel'),
                    ('cmd_tilt_vel', cmd_ns + '/_/cmd_tilt_vel'),
                ],
            )
            self.extra_gz_nodes.append(ptz_node)

        # FLIR PTU: bridge joint-state from gz and relay /ptu/cmd positions
        # to the Gazebo JointPositionController topics.
        if self.sensor.SENSOR_TYPE == BasePTU.SENSOR_TYPE:
            # PTU 使用独立的 Gazebo joint position controller。这里把 ROS /ptu/cmd
            # 中的关节目标拆成 pan/tilt 两个 Float64 topic，并把 Gazebo state 回灌到 joint_states。
            cmd_ns = '/' + self.namespace + self.name
            gz_prefix = '/' + self.name
            ptu_bridge_node = LaunchFile.Node(
                name=self.name + '_gz_ptu_bridge',
                namespace=self.namespace,
                package='ros_gz_bridge',
                executable='parameter_bridge',
                parameters=[{'use_sim_time': True}],
                arguments=[
                    gz_prefix + '/cmd_pan' + self.ROS_TO_GZ_FLOAT,
                    gz_prefix + '/cmd_tilt' + self.ROS_TO_GZ_FLOAT,
                    gz_prefix + '/joint_states' + self.GZ_TO_ROS_JOINTSTATE,
                ],
                remappings=[
                    (gz_prefix + '/cmd_pan', cmd_ns + '/cmd_pan'),
                    (gz_prefix + '/cmd_tilt', cmd_ns + '/cmd_tilt'),
                    (gz_prefix + '/joint_states', cmd_ns + '/state'),
                ],
            )
            self.extra_gz_nodes.append(ptu_bridge_node)

            ptu_relay_node = LaunchFile.Node(
                name=self.name + '_sim_relay',
                namespace=self.namespace,
                package='clearpath_generator_gz',
                executable='ptu_sim_relay_node',
                parameters=[
                    {'use_sim_time': True},
                    {'pan_joint': self.name + '_pan'},
                    {'tilt_joint': self.name + '_tilt'},
                ],
                remappings=[
                    ('cmd', '/ptu/cmd'),
                    ('cmd_pan', cmd_ns + '/cmd_pan'),
                    ('cmd_tilt', cmd_ns + '/cmd_tilt'),
                    ('state', cmd_ns + '/state'),
                    ('joint_states', '/' + self._robot_namespace + '/platform/joint_states'),
                ],
            )
            self.extra_gz_nodes.append(ptu_relay_node)

    def generate(self):
        """落盘当前传感器的 launch 文件。"""
        sensor_writer = LaunchWriter(self.launch_file)
        # bridge 是必需项；static TF 对 PTU 例外，因为 PTU 由多个运动关节 frame 组成。
        sensor_writer.add(self.gz_bridge_node)
        if self.static_tf_node is not None:
            sensor_writer.add(self.static_tf_node)
        sensor_writer.add(self.prefix_launch_arg)
        for node in self.extra_gz_nodes:
            sensor_writer.add(node)
        # 最终文件会被上层 sensors-service.launch.py include。
        sensor_writer.generate_file()

    @property
    def namespace(self) -> str:
        """Return sensor namespace."""
        if self._robot_namespace in ('', '/'):
            return f'{self.TOPIC_NAMESPACE}'
        else:
            return f'{self._robot_namespace}/{self.TOPIC_NAMESPACE}'

    @property
    def name(self) -> str:
        """Return sensor name."""
        return self.sensor.name

    @property
    def robot_name(self) -> str:
        """Return robot name."""
        # 与 GzLaunchGenerator 保持同一套 model 命名逻辑，保证 topic prefix 一致。
        if self._robot_namespace in ('', '/'):
            return 'robot'
        else:
            return self._robot_namespace + '/robot'

    @property
    def model(self) -> str:
        """Return sensor model."""
        return self.sensor.SENSOR_MODEL

    def get_gz_bridge_arg(self, suffix: str, gz_to_ros: str) -> list:
        """构造 ros_gz_bridge 的 argument 片段，供旧式按参数传 bridge 的代码复用。"""
        return [
          LaunchFile.Variable('prefix'),
          self.sensor.get_name() + '/' + suffix + gz_to_ros
        ]

    def get_gz_bridge_remap(self, suffix: str, topic: str) -> tuple:
        """构造从 Gazebo sensor topic 到 ROS topic 的 remap 规则。"""
        return (
          [
            LaunchFile.Variable('prefix'),
            self.sensor.get_name() + '/' + suffix
          ],
          topic
        )
