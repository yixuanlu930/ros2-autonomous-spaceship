# Práctica 3 — Controlador Autónomo Nave Espacial 2D (ROS2)

## Compilación
```bash
source /opt/ros/jazzy/setup.bash
cd ~/spaceship_ws
colcon build --symlink-install --packages-select spaceship_msgs spaceship_sim spaceship_student_h
source install/setup.bash
```

## Ejecución
```bash
# Sin viento:
ros2 launch spaceship_student_h full_sim.launch.py target_x:=10.0 target_y:=8.0

# Con viento máximo:
ros2 launch spaceship_student_h full_sim.launch.py target_x:=10.0 target_y:=8.0 wind_strength:=1.0 wind_frequency:=0.15
```

## Dependencias
rclpy, geometry_msgs, visualization_msgs, tf2_ros, rviz2. Sin librerías externas adicionales.
