"""
Launch principal del simulador de la nave espacial.

USO
───
  # Sin target (usar click en RViz):
  ros2 launch spaceship_sim spaceship_sim.launch.py

  # Con target inicial:
  ros2 launch spaceship_sim spaceship_sim.launch.py target_x:=10.0 target_y:=8.0

  # Configurar viento (para evaluación: todos los grupos con la misma semilla):
  ros2 launch spaceship_sim spaceship_sim.launch.py \\
      target_x:=10.0 target_y:=8.0 \\
      wind_seed:=42 wind_strength:=1.2 wind_frequency:=0.15

  # Sin viento (para desarrollo inicial):
  ros2 launch spaceship_sim spaceship_sim.launch.py \\
      target_x:=10.0 target_y:=8.0 wind_strength:=0.0
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

_NO_TARGET = '-999.0'


def generate_launch_description():
    pkg_share = get_package_share_directory('spaceship_sim')

    args = [
        # Target
        DeclareLaunchArgument('target_x', default_value=_NO_TARGET,
            description='Target X (m). Dejar vacío para usar click RViz.'),
        DeclareLaunchArgument('target_y', default_value=_NO_TARGET,
            description='Target Y (m). Dejar vacío para usar click RViz.'),
        # Posición inicial
        DeclareLaunchArgument('start_x',       default_value='0.0'),
        DeclareLaunchArgument('start_y',       default_value='0.0'),
        DeclareLaunchArgument('start_heading', default_value='1.5708',
            description='Heading inicial (rad). Por defecto π/2 = norte.'),
        # Viento
        DeclareLaunchArgument('wind_seed',      default_value='42',
            description='Semilla del generador de viento. '
                        'Misma semilla = misma secuencia de viento para todos los grupos.'),
        DeclareLaunchArgument('wind_strength',  default_value='0.0',
            description='Fuerza máxima del viento (N/kg). 0.0 = sin viento.'),
        DeclareLaunchArgument('wind_frequency', default_value='0.0',
            description='Frecuencia de cambio de dirección del viento (Hz).'),
    ]

    simulator = Node(
        package='spaceship_sim',
        executable='ship_simulator',
        name='ship_simulator',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'target_x':       LaunchConfiguration('target_x'),
            'target_y':       LaunchConfiguration('target_y'),
            'start_x':        LaunchConfiguration('start_x'),
            'start_y':        LaunchConfiguration('start_y'),
            'start_heading':  LaunchConfiguration('start_heading'),
            'wind_seed':      LaunchConfiguration('wind_seed'),
            'wind_strength':  LaunchConfiguration('wind_strength'),
            'wind_frequency': LaunchConfiguration('wind_frequency'),
        }]
    )

    rviz_pub = Node(
        package='spaceship_sim',
        executable='rviz_publisher',
        name='rviz_publisher',
        output='screen',
        emulate_tty=True,
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'spaceship.rviz')],
        output='screen',
    )

    tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map']
    )

    return LaunchDescription(args + [simulator, rviz_pub, rviz, tf])
