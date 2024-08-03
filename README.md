# Final Project - ENPM702 Summer 2024 
## By: Soroush Etemad and Harsh Senjaliya  
## Overview
Presented with an environment containing one TurtleBot, five cameras, and five floating parts (batteries), the objective is to maneuver the robot sequentially under each part in the order specified by the 'waypoint_param.yaml'.

## Objectives
- The sequence for robot movement is specified in the parameter file `waypoint_params.yaml`. Feel free to modify the order of the batteries to test the code!
- Subscribe to the camera topics to determine the position of each part.
- Send an action request to an active action server to initiate robot movement.

## Setup and Running the Code

1. **Build the Workspace:**
    ```bash
    cd <workspace>
    colcon build
    ```

2. **Source the Workspace:**
    ```bash
    source <workspace>
    ```

3. **Run the Gazebo Simulation:**
    ```bash
    ros2 launch final_project final_project.launch.py
    ```

4. **Open aother Terminal, Launch the Target Reacher:**
    ```bash
    cd <workspace>
    source <workspace>
    ros2 launch final_project target_reacher.launch.py
    ```

## Files

### reach_target_action.hpp
This file defines the action server responsible for handling the robot's navigation requests.

### reach_target_client.cpp
This file contains the client code that sends navigation requests to the action server.

## Additional Information
- Ensure that all dependencies are installed and properly configured.
- The parameter file `waypoint_params.yaml` should be placed in the appropriate directory as specified in your ROS2 project structure.
