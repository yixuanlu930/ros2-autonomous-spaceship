# ROS 2 Autonomous Spaceship

An autonomous **2D spacecraft navigation and control system built with ROS 2**, featuring differential-thrust control, PD guidance, wind-disturbance compensation, custom ROS interfaces, interactive target selection, and RViz visualization.

The project includes both a physics-based spacecraft simulator and an autonomous controller capable of navigating the spacecraft toward a target while controlling position, velocity, heading, and motor power.

---

## Overview

The objective of this project is to autonomously control a simulated two-dimensional spacecraft using two independently controlled thrusters.

The spacecraft must:

* Orient itself toward a target
* Accelerate toward the destination
* Manage its velocity during the approach
* Brake before overshooting
* Compensate for wind disturbances
* Reach the target with low residual speed

The complete system is implemented as a ROS 2 workspace and includes:

* A spacecraft physics simulator
* A custom autonomous controller
* Custom ROS messages
* A motor-control service
* RViz visualization
* Configurable control parameters
* Interactive target selection
* Wind simulation

---

# System Architecture

The project is organized into three ROS 2 packages:

```text
spaceship_msgs
spaceship_sim
spaceship_student_h
```

Their responsibilities are:

```text
┌──────────────────────────────┐
│        spaceship_sim         │
│                              │
│  2D physics simulation       │
│  wind disturbances           │
│  spacecraft state            │
│  RViz visualization          │
└──────────────┬───────────────┘
               │
               │ /ship_state
               ▼
┌──────────────────────────────┐
│    spaceship_student_h       │
│                              │
│ autonomous PD controller     │
│ heading control              │
│ wind estimation              │
│ navigation state machine     │
└──────────────┬───────────────┘
               │
               │ /set_motor_power
               ▼
┌──────────────────────────────┐
│        spaceship_sim         │
│                              │
│ left and right thrusters     │
└──────────────────────────────┘
```

The packages communicate using the custom interfaces defined in:

```text
spaceship_msgs
```

---

# Autonomous Controller

The autonomous controller is implemented in:

```text
spaceship_student_h/spaceship_student_h/controller_node.py
```

The controller receives the spacecraft state through:

```text
/ship_state
```

and controls both thrusters using the asynchronous service:

```text
/set_motor_power
```

---

# PD Position and Velocity Control

The main translational control law follows a proportional-derivative structure:

```text
a_des =
kp · position_error
-
kd · velocity
-
estimated_wind
```

Conceptually:

```text
Target position
      │
      ▼
Position error
      │
      ├────────────┐
      │            │
      ▼            ▼
     KP          Velocity
      │            │
      │           KD
      │            │
      └──────┬─────┘
             │
             ▼
     Desired acceleration
             │
             ▼
      Desired heading
             │
             ▼
  Differential thrust control
```

The default gains are:

```text
kp_pos = 0.8
kd_vel = 2.2
```

These parameters can be modified from the ROS 2 launch file.

---

# Differential-Thrust Steering

The spacecraft has two independent motors:

```text
M1 = left thruster
M2 = right thruster
```

Each motor receives a power command between:

```text
0–100%
```

The difference between motor powers produces rotational motion.

For example:

```text
M1 > M2
```

and:

```text
M2 > M1
```

produce turns in opposite directions.

When both motors apply similar power, the spacecraft primarily accelerates forward.

---

# Navigation State Machine

The controller uses several navigation phases:

```text
ORIENT
THRUST
BRAKE
SETTLE
ARRIVED
```

These phases allow the control strategy to change depending on the current situation.

---

## ORIENT

When the heading error is too large, the spacecraft prioritizes rotation before applying significant forward thrust.

This prevents the spacecraft from accelerating strongly in the wrong direction.

---

## THRUST

Once the spacecraft is sufficiently aligned with the desired direction, thrust is applied toward the target.

The requested thrust depends on the desired acceleration produced by the PD controller.

---

## BRAKE

As the spacecraft approaches the target, the controller detects when its current velocity could cause it to overshoot.

It then transitions into a braking strategy.

---

## SETTLE

Near the target, the controller focuses on reducing residual motion and improving final positioning.

---

## ARRIVED

The spacecraft is considered to have reached the destination when:

```text
distance < 2.0 m
```

and:

```text
speed < 0.1 m/s
```

These criteria are also represented in the custom `ShipState` message.

---

# Wind Disturbances

The simulator can generate external wind forces.

The wind:

* Has configurable strength
* Changes direction over time
* Uses smooth interpolation
* Can use a deterministic random seed
* Starts when the spacecraft first applies thrust

Parameters include:

```text
wind_seed
wind_strength
wind_frequency
```

Example:

```bash
ros2 launch spaceship_student_h full_sim.launch.py \
    target_x:=10.0 \
    target_y:=8.0 \
    wind_strength:=1.0 \
    wind_frequency:=0.15
```

---

# Wind Estimation

The controller also maintains an internal estimate of external acceleration.

The estimate is obtained by comparing:

```text
measured acceleration
```

against:

```text
expected acceleration from the spacecraft model
```

A low-pass filter is then used to estimate the disturbance.

The estimated wind components are incorporated directly into the translational controller:

```text
desired_ax =
kp_pos * dx
-
kd_vel * vx
-
wind_est_x
```

```text
desired_ay =
kp_pos * dy
-
kd_vel * vy
-
wind_est_y
```

This improves robustness when external forces disturb the trajectory.

---

# Heading Control

Rotational control is implemented using both heading error and angular velocity.

Main parameters include:

```text
turn_gain
omega_damping
angle_tolerance
hard_angle_tolerance
```

Default values include:

```text
turn_gain = 42.0
omega_damping = 18.0

angle_tolerance = 0.18 rad
hard_angle_tolerance = 0.85 rad
```

The angular damping term reduces oscillation while the spacecraft rotates toward the desired heading.

---

# Vertical Guidance Offset

The controller includes an additional guidance mechanism designed to improve the final approach.

During part of the trajectory, the controller can aim slightly above the actual target using:

```text
vertical_guidance_offset
```

The effect gradually disappears as the spacecraft approaches the target.

Default values:

```text
vertical_guidance_offset = 1.15 m
guidance_release_distance = 4.0 m
```

This provides a smoother final approach in the simulated dynamics.

---

# Interactive Target Selection

The controller can receive new destinations through RViz.

When:

```text
use_clicked_point = true
```

the controller subscribes to:

```text
/clicked_point
```

A new target can therefore be selected interactively using the RViz **Publish Point** tool.

When a new target is received:

* The target coordinates are updated
* The trajectory history is cleared
* A new navigation run begins

---

# Custom ROS 2 Interfaces

The project defines its own messages and service in:

```text
spaceship_msgs
```

---

## ShipState

```text
spaceship_msgs/msg/ShipState.msg
```

Contains the complete state of the spacecraft:

```text
x
y
heading

vx
vy
omega

power_m1
power_m2

elapsed_time
arrived

target_x
target_y
```

The simulator publishes this state at approximately:

```text
20 Hz
```

---

## MotorCommand

```text
spaceship_msgs/msg/MotorCommand.msg
```

Defines direct motor commands.

Motor identifiers are:

```text
0 = both motors
1 = left motor
2 = right motor
```

Power is expressed as:

```text
0–100%
```

---

## SetMotorPower

```text
spaceship_msgs/srv/SetMotorPower.srv
```

The autonomous controller primarily uses this service to change motor power.

Request:

```text
motor_id
power
```

Response:

```text
success
message
```

The controller uses asynchronous service calls.

---

## ControlDebug

```text
spaceship_msgs/msg/ControlDebug.msg
```

The controller publishes diagnostic information including:

```text
phase
distance_to_target
angle_error
speed
desired_heading
desired_accel_x
desired_accel_y
power_m1
power_m2
```

This makes it easier to inspect controller behavior while the simulation is running.

---

# RViz Visualization

The project includes an RViz configuration:

```text
spaceship_sim/rviz/spaceship.rviz
```

The simulator and controller publish markers for visual inspection.

The visualization can represent information such as:

* Spacecraft position
* Orientation
* Target
* Trajectory
* Controller information

This allows the autonomous behavior to be observed interactively.

---

# ROS 2 Workspace Structure

```text
ros2-autonomous-spaceship/
│
├── spaceship_ws/
│   └── src/
│       │
│       ├── spaceship_msgs/
│       │   ├── msg/
│       │   │   ├── ShipState.msg
│       │   │   ├── MotorCommand.msg
│       │   │   └── ControlDebug.msg
│       │   │
│       │   ├── srv/
│       │   │   └── SetMotorPower.srv
│       │   │
│       │   ├── CMakeLists.txt
│       │   └── package.xml
│       │
│       ├── spaceship_sim/
│       │   ├── spaceship_sim/
│       │   │   ├── ship_simulator.py
│       │   │   └── rviz_publisher.py
│       │   │
│       │   ├── launch/
│       │   │   └── spaceship_sim.launch.py
│       │   │
│       │   ├── rviz/
│       │   │   └── spaceship.rviz
│       │   │
│       │   ├── setup.py
│       │   └── package.xml
│       │
│       └── spaceship_student_h/
│           ├── spaceship_student_h/
│           │   └── controller_node.py
│           │
│           ├── config/
│           │   └── controller_params.yaml
│           │
│           ├── launch/
│           │   ├── full_sim.launch.py
│           │   └── student_controller.launch.py
│           │
│           ├── setup.py
│           └── package.xml
│
├── Video_muestra.mp4
├── memoria_ROS_grupo_h.pdf
├── LICENSE
└── .gitignore
```

---

# Requirements

The project targets:

```text
ROS 2 Jazzy
```

Main ROS dependencies include:

```text
rclpy
geometry_msgs
std_msgs
visualization_msgs
tf2_ros
rviz2
```

The controller itself does not require additional external Python libraries.

---

# Build

First source ROS 2:

```bash
source /opt/ros/jazzy/setup.bash
```

Navigate to the workspace:

```bash
cd spaceship_ws
```

Build the packages:

```bash
colcon build --symlink-install \
    --packages-select \
    spaceship_msgs \
    spaceship_sim \
    spaceship_student_h
```

Then source the workspace:

```bash
source install/setup.bash
```

---

# Running the Simulation

## Without Wind

```bash
ros2 launch spaceship_student_h full_sim.launch.py \
    target_x:=10.0 \
    target_y:=8.0
```

---

## With Wind

Example with stronger disturbances:

```bash
ros2 launch spaceship_student_h full_sim.launch.py \
    target_x:=10.0 \
    target_y:=8.0 \
    wind_strength:=1.0 \
    wind_frequency:=0.15
```

---

# Custom Starting Position

The launch file also supports configuring the initial spacecraft state.

Example:

```bash
ros2 launch spaceship_student_h full_sim.launch.py \
    start_x:=0.0 \
    start_y:=0.0 \
    start_heading:=1.5708 \
    target_x:=10.0 \
    target_y:=8.0
```

---

# Controller Parameters

Important launch parameters include:

| Parameter                   | Default | Description                      |
| --------------------------- | ------: | -------------------------------- |
| `kp_pos`                    |   `0.8` | Position proportional gain       |
| `kd_vel`                    |   `2.2` | Velocity damping gain            |
| `max_power`                 |   `100` | Maximum motor power              |
| `min_thrust_power`          |    `12` | Minimum active thrust            |
| `turn_gain`                 |    `42` | Heading proportional gain        |
| `omega_damping`             |    `18` | Angular velocity damping         |
| `angle_tolerance`           |  `0.18` | Normal heading tolerance         |
| `hard_angle_tolerance`      |  `0.85` | Rotation-only threshold          |
| `vertical_guidance_offset`  |  `1.15` | Virtual vertical target offset   |
| `guidance_release_distance` |   `4.0` | Guidance offset release distance |
| `service_period`            |  `0.08` | Motor service update interval    |

Parameters can also be configured in:

```text
spaceship_student_h/config/controller_params.yaml
```

---

# Example Control Loop

The complete navigation loop can be summarized as:

```text
Receive ShipState
      │
      ▼
Compute target error
      │
      ▼
Estimate wind disturbance
      │
      ▼
PD position/velocity control
      │
      ▼
Desired acceleration
      │
      ▼
Desired heading
      │
      ▼
Heading PD controller
      │
      ▼
Navigation phase
      │
 ┌────┼─────────┬────────┬────────┐
 │    │         │        │        │
 ▼    ▼         ▼        ▼        ▼
ORIENT THRUST  BRAKE   SETTLE  ARRIVED
      │
      ▼
Compute M1 / M2 power
      │
      ▼
/set_motor_power
      │
      ▼
Physics simulator
```

---

# Demo

The repository includes a demonstration video:

```text
Video_muestra.mp4
```

showing the spacecraft simulation and autonomous controller behavior.

---

# Technical Report

A complete project report is included in:

```text
memoria_ROS_grupo_h.pdf
```

It contains additional information about the implementation, control strategy, experiments, and results.

---

# Technologies

* ROS 2 Jazzy
* Python
* rclpy
* RViz2
* ROS 2 custom messages
* ROS 2 services
* ROS 2 launch system
* Differential thrust control
* PD control
* Physics simulation

---

# Key Concepts

This project demonstrates:

* Robotics
* Autonomous navigation
* Closed-loop control
* Proportional-derivative control
* Position control
* Velocity damping
* Heading control
* Differential thrust
* Disturbance estimation
* Wind compensation
* State machines
* ROS 2 nodes
* ROS 2 topics
* ROS 2 services
* Custom ROS interfaces
* Simulation
* RViz visualization

---

# Possible Extensions

Future improvements could include:

* PID control
* Model Predictive Control
* Kalman filtering
* More advanced disturbance observers
* Path planning through multiple waypoints
* Obstacle avoidance
* Fuel-aware optimization
* Trajectory optimization
* Automatic controller tuning
* Monte Carlo disturbance testing
* Reinforcement-learning-based control
* 3D spacecraft dynamics

---

# Academic Context

This project was developed as an educational robotics assignment focused on **ROS 2 and autonomous spacecraft control**.

The goal was to design a controller capable of autonomously navigating a simulated spacecraft using differential motor thrust while handling physical dynamics and environmental disturbances.

---

# Disclaimer

This is an educational simulation.

The control strategy and physical model are simplified and are not intended for real spacecraft guidance or flight-control applications.

---

# License

This project is distributed under the MIT License.

