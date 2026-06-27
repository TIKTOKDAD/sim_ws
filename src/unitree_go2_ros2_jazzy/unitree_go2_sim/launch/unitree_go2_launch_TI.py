import os

import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    GroupAction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution


# TIbuilding 仿真启动文件：加载室内建筑场景 TIbuilding.sdf，生成 Go2 机器人，
# 同时启动 robot_state_publisher、CHAMP 控制器、EKF、Gazebo 桥接和 RViz。
def generate_launch_description():
    # use_sim_time 贯穿所有节点，确保控制器和可视化都使用 Gazebo 发布的 /clock。
    use_sim_time = LaunchConfiguration("use_sim_time")
    # CHAMP 和 robot_localization 以 base_link 作为主体坐标系。
    base_frame = "base_link"

    # 通过 ROS 2 包索引查找安装后的资源路径，避免写死工作空间绝对路径。
    unitree_go2_sim = launch_ros.substitutions.FindPackageShare(
        package="unitree_go2_sim").find("unitree_go2_sim")
    unitree_go2_description = launch_ros.substitutions.FindPackageShare(
        package="unitree_go2_description").find("unitree_go2_description")

    # CHAMP 使用 joints/links/gait 三类配置描述机器人拓扑和步态参数。
    joints_config = os.path.join(unitree_go2_sim, "config/joints/joints.yaml")
    ros_control_config = os.path.join(
        unitree_go2_sim, "config/ros_control/ros_control.yaml"
    )
    gait_config = os.path.join(unitree_go2_sim, "config/gait/gait.yaml")
    links_config = os.path.join(unitree_go2_sim, "config/links/links.yaml")

    # 默认机器人模型和 TIbuilding 世界文件，可通过 launch 参数覆盖。
    default_model_path = os.path.join(unitree_go2_description, "urdf/unitree_go2_robot.xacro")
    default_world_path = os.path.join(unitree_go2_description, "worlds/TIbuilding.sdf")

    # launch 参数集中声明在前面，便于 ros2 launch ... 参数:=值 覆盖。
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )
    declare_rviz = DeclareLaunchArgument(
        "rviz", default_value="true", description="Launch rviz"
    )
    declare_robot_name = DeclareLaunchArgument(
        "robot_name", default_value="go2", description="Robot name"
    )
    declare_lite = DeclareLaunchArgument(
        "lite", default_value="false", description="Lite"
    )
    declare_ros_control_file = DeclareLaunchArgument(
        "ros_control_file",
        default_value=ros_control_config,
        description="Ros control config path",
    )
    declare_gazebo_world = DeclareLaunchArgument(
        "world", default_value=default_world_path, description="Gazebo world name"
    )

    # gui 参数目前保留给上层调用兼容，实际 Gazebo 启动参数在 gz_args 中指定。
    declare_gui = DeclareLaunchArgument(
        "gui", default_value="true", description="Use gui"
    )
    # TIbuilding 的主要地面模型位于 z=4 附近，因此默认生成高度高于平地场景。
    declare_world_init_x = DeclareLaunchArgument("world_init_x", default_value="0.0")
    declare_world_init_y = DeclareLaunchArgument("world_init_y", default_value="0.0")
    declare_world_init_z = DeclareLaunchArgument("world_init_z", default_value="4.375")
    declare_world_init_heading = DeclareLaunchArgument(
        "world_init_heading", default_value="0.0"
    )
    declare_description_path = DeclareLaunchArgument(
        "unitree_go2_description_path",
        default_value=default_model_path,
        description="Path to the robot description xacro file",
    )

    # robot_description 由 xacro 命令动态生成，后续 spawn、RViz 和控制节点共享同一份模型。
    robot_description = {"robot_description": Command(["xacro ", LaunchConfiguration("unitree_go2_description_path")])}

    # 发布 URDF 中的 link/joint TF，Gazebo 生成实体前后都需要该节点提供 robot_description。
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {"use_sim_time": use_sim_time}
        ],
    )

    # CHAMP 四足控制器：订阅 /cmd_vel，依据 gait/joints/links 配置输出关节轨迹。
    quadruped_controller_node = Node(
        package="champ_base",
        executable="quadruped_controller_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"gazebo": True},
            {"publish_joint_states": True},
            {"publish_joint_control": True},
            {"publish_foot_contacts": False},
            # 与 ros_control.yaml 中的 joint_group_effort_controller 对应。
            {"joint_controller_topic": "joint_group_effort_controller/joint_trajectory"},
            {"urdf": Command(['xacro ', LaunchConfiguration('unitree_go2_description_path')])},
            joints_config,
            links_config,
            gait_config,
            {"hardware_connected": False},
            {"publish_foot_contacts": False},
            {"close_loop_odom": True},
        ],
        remappings=[("/cmd_vel/smooth", "/cmd_vel")],
    )

    # CHAMP 状态估计节点：结合 URDF、IMU 和关节状态估计机体状态。
    state_estimator_node = Node(
        package="champ_base",
        executable="state_estimation_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"orientation_from_imu": True},
            {"urdf": Command(['xacro ', LaunchConfiguration('unitree_go2_description_path')])},
            joints_config,
            links_config,
            gait_config,
        ],
    )

    # 把 Gazebo 四个足端 contact sensor 转换成 CHAMP 状态估计需要的 /foot_contacts。
    foot_contact_converter_node = Node(
        package="champ_base",
        executable="foot_contact_converter_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
        ],
    )

    # 第一层 EKF：把 base_link 相关估计整理成 base_footprint 下的局部里程计。
    base_to_footprint_ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="base_to_footprint_ekf",
        output="screen",
        parameters=[
            {"base_link_frame": base_frame},
            {"use_sim_time": use_sim_time},
            os.path.join(
                get_package_share_directory("champ_base"),
                "config",
                "ekf",
                "base_to_footprint.yaml",
            ),
            {"publish_tf": True},
        ],
        remappings=[("odometry/filtered", "odom/local")],
    )

    # 第二层 EKF：融合 CHAMP 足端里程计和 IMU，发布 odom 坐标系下的滤波里程计。
    footprint_to_odom_ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="footprint_to_odom_ekf",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"base_link_frame": "base_footprint"},
            {"odom_frame": "odom"},
            {"world_frame": "odom"},
            {"publish_tf": True},
            {"frequency": 50.0},
            {"two_d_mode": True},
            {"odom0": "odom/raw"},
            # 与旧版保持一致：第二层 EKF 使用 CHAMP state_estimation 发布的足端里程计速度。
            {"odom0_config": [False, False, False, False, False, False, True, True, False, False, False, True, False, False, False]},
            {"imu0": "imu/data"},
            # 只从 IMU 取 yaw 角速度相关信息，与 two_d_mode 的平面运动假设匹配。
            {"imu0_config": [False, False, False, False, False, True, False, False, False, False, False, True, False, False, False]},
        ],
        remappings=[("odometry/filtered", "odom")],
    )

    # 静态 map -> odom：没有外部定位系统时，先把全局地图和里程计原点重合。
    map_to_odom_tf_node = Node(
        package='tf2_ros',
        name='map_to_odom_tf_node',
        executable='static_transform_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'map', '--child-frame-id', 'odom'
        ],
    )

    # 静态 base_footprint -> base_link：为导航/里程计工具提供常见的平面足迹坐标系。
    base_footprint_to_base_link_tf_node = Node(
        package='tf2_ros',
        name='base_footprint_to_base_link_tf_node',
        executable='static_transform_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_footprint', '--child-frame-id', 'base_link'
        ],
    )

    # RViz 只在 rviz:=true 时启动，默认加载本包保存的显示配置。
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(unitree_go2_sim, "rviz/rviz.rviz")],
        condition=IfCondition(LaunchConfiguration("rviz")),
        # parameters=[{"use_sim_time": use_sim_time}]
    )

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # 启动 Gazebo Sim 并加载 TIbuilding.sdf；-r 表示启动后立即运行，不暂停等待。
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': [PathJoinSubstitution([
                unitree_go2_description,
                'worlds',
                'TIbuilding.sdf'
            ]), ' -r']  # Add -r flag to start unpaused
        }.items(),
    )

    # 根据 /robot_description 在 Gazebo 中创建机器人实体，并使用 launch 参数设置初始位姿。
    gazebo_spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', LaunchConfiguration('robot_name'),
            '-topic', 'robot_description',
            '-x', LaunchConfiguration('world_init_x'),
            '-y', LaunchConfiguration('world_init_y'),
            '-z', LaunchConfiguration('world_init_z'),
            '-Y', LaunchConfiguration('world_init_heading')
        ],
    )

    # ROS 2 与 Gazebo Sim 的话题桥。方括号方向表示单向桥，@ 两侧表示双向桥。
    gazebo_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gazebo_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            # Gazebo -> ROS：仿真时钟、传感器、TF、关节状态和里程计。
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU',
            # TF 和 joint_states 由 robot_state_publisher / joint_state_broadcaster 发布，
            # 不从 Gazebo PosePublisher 桥接，避免同一 frame 或话题出现多个源。
            # '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            # '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
            '/velodyne_points/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            '/unitree_lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            # '/velodyne_points@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/rgb_image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/left_rgb_image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/right_rgb_image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/world/TIbuilding/model/go2/link/lf_lower_leg_link/sensor/lf_foot_contact/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
            '/world/TIbuilding/model/go2/link/rf_lower_leg_link/sensor/rf_foot_contact/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
            '/world/TIbuilding/model/go2/link/lh_lower_leg_link/sensor/lh_foot_contact/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
            '/world/TIbuilding/model/go2/link/rh_lower_leg_link/sensor/rh_foot_contact/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',

            # ROS -> Gazebo：速度命令和关节轨迹命令进入仿真。
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/joint_group_effort_controller/joint_trajectory@trajectory_msgs/msg/JointTrajectory]gz.msgs.JointTrajectory',
        ],
        remappings=[
            ('/world/TIbuilding/model/go2/link/lf_lower_leg_link/sensor/lf_foot_contact/contact', '/lf_foot_contact'),
            ('/world/TIbuilding/model/go2/link/rf_lower_leg_link/sensor/rf_foot_contact/contact', '/rf_foot_contact'),
            ('/world/TIbuilding/model/go2/link/lh_lower_leg_link/sensor/lh_foot_contact/contact', '/lh_foot_contact'),
            ('/world/TIbuilding/model/go2/link/rh_lower_leg_link/sensor/rh_foot_contact/contact', '/rh_foot_contact'),
            ('/odom', '/odom/gazebo'),
        ],
    )

    # spawner 会完成 load、configure、activate 生命周期。延迟启动可等待 Gazebo/控制插件准备好。
    controller_spawner_js = TimerAction(
        period=20.0,  # Wait for Gazebo to fully initialize
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                output="screen",
                arguments=[
                    "--controller-manager-timeout", "120",  # Longer timeout
                    "joint_states_controller",  # No --inactive flag to ensure full activation
                ],
                parameters=[{"use_sim_time": use_sim_time}],
            )
        ]
    )

    # effort 控制器依赖关节状态和 ros2_control 硬件接口，因此比 joint_states_controller 更晚启动。
    controller_spawner_effort = TimerAction(
        period=30.0,  # Wait 5 seconds after joint_states_controller
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                output="screen",
                arguments=[
                    "--controller-manager-timeout", "120",  # Longer timeout
                    "joint_group_effort_controller",  # No --inactive flag to ensure full activation
                ],
                parameters=[{"use_sim_time": use_sim_time}],
            )
        ]
    )

    # 启动后打印控制器状态，便于确认控制器是否已 active。
    controller_status_check = TimerAction(
        period=25.0,  # Check status after controllers should be loaded
        actions=[
            ExecuteProcess(
                cmd=["bash", "-c", "echo 'Checking controller status:' && ros2 control list_controllers"],
                output='screen',
            )
        ]
    )

    return LaunchDescription(
        [
            # 启动参数。
            declare_use_sim_time,
            declare_rviz,
            declare_robot_name,
            declare_lite,
            declare_ros_control_file,
            declare_gazebo_world,
            declare_gui,
            declare_world_init_x,
            declare_world_init_y,
            declare_world_init_z,
            declare_world_init_heading,
            declare_description_path, 

            # Gazebo 和机器人描述先启动，后续控制器才能连接到仿真实体。
            gz_sim,
            robot_state_publisher_node,
            gazebo_spawn_robot,
            gazebo_bridge,

            # CHAMP 控制和状态估计。
            quadruped_controller_node,
            state_estimator_node,
            foot_contact_converter_node,

            # 里程计滤波。
            base_to_footprint_ekf,
            footprint_to_odom_ekf,

            # 静态 TF 连接。
            map_to_odom_tf_node,
            # base_footprint_to_base_link_tf_node,

            # 控制器生命周期管理。
            controller_spawner_js,
            controller_spawner_effort,
            controller_status_check,

            # 可视化。
            rviz2,
        ]
    )
