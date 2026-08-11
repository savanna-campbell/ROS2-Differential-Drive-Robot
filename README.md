# Ros2-Differential-Drive-Robot

A ROS2-based mobile robot that executes open-loop motion primitives (straight, circular, figure-8) while separately logging its true trajectory via sensor fusion, for offline motion analysis.

<!-- Add a photo or short clip of the robot here once you have one -->
![demo](resources/robot_movement.gif)

## Overview

This project splits compute across a Jetson Nano and an Arduino: the Jetson runs ROS2 and decides what the robot should do, the Arduino turns those decisions into motor signals and reports sensor data back. A separate estimation pipeline logs where the robot actually went, independent of what it was commanded to do which is useful for comparing intended vs. actual motion after the fact.

## System Architecture

- **Jetson Nano** — runs ROS2 nodes; selects and executes motion primitives; receives fused state estimates
- **Arduino** — reads wheel encoders and IMU; converts incoming commands to PWM motor signals
- **Communication** — serial link between Jetson and Arduino
- **State estimation** — encoder + IMU data fused via `robot_localization` (EKF) on the Jetson; fused pose trajectory logged to CSV

```
[Encoders + IMU] → Arduino → (serial) → Jetson (ROS2)
                                            ├─→ robot_localization (EKF) → trajectory CSV (offline analysis)
                                            └─→ motion primitive commands → (serial) → Arduino → PWM → motors
```

## Features

- Parameterized open-loop motion primitives: straight-line, circular, and figure-8 paths, configurable velocity and turn radius, fixed-duration execution
- Threshold-triggered behavior switching (e.g. wheel-direction reversal after a tracked 360° rotation)
- Sensor fusion (EKF via `robot_localization`) combining wheel encoder and IMU data for pose estimation
- Trajectory logging to CSV for offline analysis of actual vs. commanded motion

## Limitations / Future Work

- **Open-loop control** — motion primitives run for a fixed duration without feedback correction; there's no closed-loop controller (e.g. PID) adjusting for error while the robot moves. The state estimation pipeline exists but currently only logs data — it isn't fed back into the motor commands. Closing that loop is the natural next step.

## Requirements

- ROS2 Humble
- Jetson Nano (or similar) running Jetpack 6.0
- Arduino Mega 2560
- Python 3.13.1
- `robot_localization` ROS2 package


## Installation

```bash
# Clone the repo
git clone https://github.com/savanna-campbell/Ros2-Differential-Drive-Robot.git
cd Ros2-Differential-Drive-Robot

# Install ROS2 dependencies
# TODO: rosdep install 

```

<!-- ## Usage

```bash
# Launch the ROS2 stack on the Jetson
# TODO: e.g. ros2 launch [package] [launch_file]

# Run a motion primitive
# TODO: e.g. ros2 run [package] [node] --primitive circle --velocity 0.3 --radius 0.5 --duration 10
``` -->

Trajectory logs are written to individual csvs for offline analysis.

## Project Structure

```
.
├── src/            # ROS2 packages (motion primitives, state logic, localization)
├── robot_code/     # Arduino firmware for motor + sensor interface
└── README.md
```

<!-- Adjust/expand as your repo grows — e.g. if src/ splits into multiple packages
     like my_robot_controller and my_robot_interfaces, list those explicitly -->

