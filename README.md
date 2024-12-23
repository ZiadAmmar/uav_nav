# UAV Navigation Integration Assignment

## Overview
This package implements UAV navigation control using PID position control and FastPlanner HKUST for trajectory planning in ROS Noetic.

## Candidate
- **Name:** Ziad Ammar
- **Email:** ZiadRoboticist@outlook.com

## Package Components

### Core Components
1. **PIDController**
   - Implements PID Position Control in x, y, and z axes
   - Features enhanced functionality for precise position control

2. **FlightController** (Under Development)
   - Manages different mission states
   - Integrates FastPlanner HKUST with PID Controller
   - Implements logic for assignment objectives

### ROS Nodes
1. **offboard_mode_node**
   - MAVROS Offboard control example
   - Used for testing configuration and takeoff procedures
   - Based on official MAVROS documentation

2. **simple_hover_test**
   - Test node for PID Position Controller
   - Used for:
     - Controller tuning
     - Debugging
     - Development experiments

3. **pid_test_node**
   - Sets test position control points
   - Configured for aws small_house.world chandelier positions

4. **uav_navigation_node** (Under Development)
   - Implements FlightController functionality
   - Manages mission state transitions
   - Integrates FastPlanner and PIDController

### Configuration Files
Located in `config/`:
1. `controller.yaml`
   - PID controller parameters
   - Tuning configurations

2. `navigation.yaml`
   - Navigation parameters
   - Mission configurations

### Maps
Located in `maps/`:
- Utilizes aws small_house world map
- Map publishing handled by map_server package

### Launch Files
All launch files should be executed after the main simulation launch:

1. `controller_test.launch`
   - Loads controller parameters to ROS parameter server
   - Launches pid_test_control node

2. `fast_planner_test.launch`
   - Tests FastPlanner Trajectory Planner

3. `uav_nav.launch`
   - Main launch file
   - Loads all parameters to ROS parameter server
   - Sets FastPlanner arguments and topic remapping
   - Launches FastPlanner and uav_navigation_node

### Running the Package

1. Launch the main simulation:
```bash
cd /workspace/assignment/scripts
./run.sh
```

2. Choose one of the following test scenarios:

For controller testing:
```bash
./run_controller_test
```

For trajectory planner testing:
```bash
./run_traj_planner
```

## Thanks
I would like to express my gratitude to the hiring team for providing this challenging and interesting assignment.

