from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable

def generate_launch_description():
    pkg_share = FindPackageShare('my_azu')
    
    # Load xacro file
    urdf_file = PathJoinSubstitution([pkg_share, 'urdf', 'azu.urdf.xacro'])
    xacro_exe = FindExecutable(name='xacro')
    
    robot_description = ParameterValue(
        Command([xacro_exe, ' ', urdf_file]),
        value_type=str
    )

    # Joint State Publisher GUI (để xoay Head bằng slider)
    joint_state_pub = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # Robot State Publisher (để publish TF tree)
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    # RViz2 để visualize TF
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        joint_state_pub,
        robot_state_pub,
        rviz,
    ])
