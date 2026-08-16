import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    sim_launch = os.path.join(
        get_package_share_directory('spaceship_sim'),
        'launch',
        'spaceship_sim.launch.py'
    )

    args = [
        DeclareLaunchArgument('target_x', default_value='10.0'),
        DeclareLaunchArgument('target_y', default_value='8.0'),
        DeclareLaunchArgument('start_x', default_value='0.0'),
        DeclareLaunchArgument('start_y', default_value='0.0'),
        DeclareLaunchArgument('start_heading', default_value='1.5708'),
        DeclareLaunchArgument('wind_seed', default_value='42'),
        DeclareLaunchArgument('wind_strength', default_value='0.0'),
        DeclareLaunchArgument('wind_frequency', default_value='0.0'),
        DeclareLaunchArgument('kp_pos', default_value='0.8'),
        DeclareLaunchArgument('kd_vel', default_value='2.2'),
        DeclareLaunchArgument('max_power', default_value='100.0'),
        DeclareLaunchArgument('min_thrust_power', default_value='12.0'),
        DeclareLaunchArgument('turn_gain', default_value='42.0'),
        DeclareLaunchArgument('omega_damping', default_value='18.0'),
        DeclareLaunchArgument('angle_tolerance', default_value='0.18'),
        DeclareLaunchArgument('hard_angle_tolerance', default_value='0.85'),
        DeclareLaunchArgument('vertical_guidance_offset', default_value='1.15'),
        DeclareLaunchArgument('guidance_release_distance', default_value='4.0'),
        DeclareLaunchArgument('service_period', default_value='0.08'),
        DeclareLaunchArgument('turn_sign', default_value='1.0'),
    ]

    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch),
        launch_arguments={
            'target_x': LaunchConfiguration('target_x'),
            'target_y': LaunchConfiguration('target_y'),
            'start_x': LaunchConfiguration('start_x'),
            'start_y': LaunchConfiguration('start_y'),
            'start_heading': LaunchConfiguration('start_heading'),
            'wind_seed': LaunchConfiguration('wind_seed'),
            'wind_strength': LaunchConfiguration('wind_strength'),
            'wind_frequency': LaunchConfiguration('wind_frequency'),
        }.items()
    )

    controller = Node(
        package='spaceship_student_h',
        executable='controller_node',
        name='spaceship_controller',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'target_x': LaunchConfiguration('target_x'),
            'target_y': LaunchConfiguration('target_y'),
            'kp_pos': LaunchConfiguration('kp_pos'),
            'kd_vel': LaunchConfiguration('kd_vel'),
            'max_power': LaunchConfiguration('max_power'),
            'min_thrust_power': LaunchConfiguration('min_thrust_power'),
            'turn_gain': LaunchConfiguration('turn_gain'),
            'omega_damping': LaunchConfiguration('omega_damping'),
            'angle_tolerance': LaunchConfiguration('angle_tolerance'),
            'hard_angle_tolerance': LaunchConfiguration('hard_angle_tolerance'),
            'vertical_guidance_offset': LaunchConfiguration('vertical_guidance_offset'),
            'guidance_release_distance': LaunchConfiguration('guidance_release_distance'),
            'service_period': LaunchConfiguration('service_period'),
            'turn_sign': LaunchConfiguration('turn_sign'),
        }]
    )

    return LaunchDescription(args + [simulator, controller])
