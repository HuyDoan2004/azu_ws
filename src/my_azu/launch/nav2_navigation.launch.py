    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
    from launch.conditions import IfCondition
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
    from launch_ros.actions import Node
    from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
    from launch_ros.substitutions import FindPackageShare
    from ament_index_python.packages import get_package_share_directory


    def generate_launch_description():

        pkg_share = get_package_share_directory('my_azu')

        # ================= FIX MAP =================
        default_map = '/home/azusa/azu_ws/maps/home_map.yaml'

        map_yaml = LaunchConfiguration('map')
        use_sim_time = LaunchConfiguration('use_sim_time')
        autostart = LaunchConfiguration('autostart')
        launch_rviz = LaunchConfiguration('launch_rviz')

        # ================= RVIZ =================
        rviz_cfg = PathJoinSubstitution([
            FindPackageShare('nav2_bringup'),
            'rviz',
            'nav2_default_view.rviz'
        ])

        # ================= NAV2 =================
        nav2_launch = PathJoinSubstitution([
            FindPackageShare('nav2_bringup'),
            'launch',
            'bringup_launch.py'
        ])

        # ================= ROBOT =================
        urdf_file = PathJoinSubstitution([pkg_share, 'urdf', 'azu.urdf.xacro'])
        xacro_exe = FindExecutable(name='xacro')

        robot_description = ParameterValue(
            Command([xacro_exe, ' ', urdf_file]),
            value_type=str,
        )

        # ================= CONFIG =================
        rtab_cfg = PathJoinSubstitution([pkg_share, 'configs', 'rtabmap.yaml'])
        rplidar_cfg = PathJoinSubstitution([pkg_share, 'configs', 'rplidar.yaml'])
        nav2_cfg = PathJoinSubstitution([pkg_share, 'configs', 'nav2_params.yaml'])

        # ================= ROBOT STATE =================
        joint_state_pub = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        )

        robot_state_pub = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        )

        # ================= LIDAR =================
        lidar = Node(
            package='my_azu',
            executable='rplidar_node',
            name='rplidar_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                ParameterFile(rplidar_cfg, allow_substs=True),
                {'frame_id': 'lidar_link'},
            ],
        )

        # ================= ICP ODOM =================
        icp_odom = Node(
            package='rtabmap_odom',
            executable='icp_odometry',
            name='icp_odometry',
            output='screen',
            emulate_tty=True,
            parameters=[
                ParameterFile(rtab_cfg, allow_substs=True),
                {
                    'frame_id': 'base_link',
                    'odom_frame_id': 'odom',
                    'publish_tf': True,
                    'subscribe_scan': True,
                    'wait_for_transform': 0.5,
                },
            ],
            remappings=[
                ('scan', '/scan'),
            ],
        )

        # ================= MOTOR =================
        motor = Node(
            package='my_azu',
            executable='nav_bridge',
            name='nav_bridge',
            output='screen',
            emulate_tty=True,
            arguments=['--cmd-vel-topic', '/cmd_vel', '--min-speed', '1500'],
        )

        # ================= NAV2 (FIXED MAP HANDLING) =================
        nav2_bringup = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                'slam': 'False',

                # ✔ FIX 100%: KHÔNG dùng LaunchConfiguration default trong include
                'map': map_yaml,

                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'use_composition': 'False',
                'use_respawn': 'False',
                'params_file': nav2_cfg,
                'log_level': 'info',
            }.items(),
        )

        # ================= RVIZ =================
        rviz = Node(
            condition=IfCondition(launch_rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_cfg],
        )

        # ================= LAUNCH =================
        return LaunchDescription([
            DeclareLaunchArgument(
                'map',
                default_value=default_map
            ),

            DeclareLaunchArgument('use_sim_time', default_value='false'),
            DeclareLaunchArgument('autostart', default_value='true'),
            DeclareLaunchArgument('launch_rviz', default_value='true'),

            joint_state_pub,
            robot_state_pub,
            lidar,

            TimerAction(period=2.0, actions=[icp_odom]),
            TimerAction(period=2.0, actions=[motor]),
            TimerAction(period=4.0, actions=[nav2_bringup]),
            TimerAction(period=6.0, actions=[rviz]),
        ])
