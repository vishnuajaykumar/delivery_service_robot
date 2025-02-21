# Home Service Robot Project

This project simulates a home service robot that autonomously navigates in a mapped environment, picks up a virtual object, and drops it off at a designated location. It uses SLAM, AMCL, Move Base, and custom nodes to handle mapping, localization, and navigation.

## Overview

The robot performs the following tasks:

- **SLAM Mapping**: Uses `gmapping` to map the environment and save it as a `.pgm` file and yaml file in a map package for AMCL use. 
- **Localization**: Implements Adaptive Monte Carlo Localization (`AMCL`) to track the robot’s position using the saved map files.
- **Navigation**: Utilizes the `move_base` package with global and local costmaps to navigate around obstacles.
- **Auto Goal**: The `pick_objects` node sends the robot to a predefied pick up and drop off locations.
- **Virtual Object**: The `add_marker` node drops virual green marker in pick up area for 5 secs then reappears in the drop off area. 
- **Home service**: The `start_home_service` node interacts with the robot’s odometry to simulate picking up and dropping off a virtual object.

## Packages Used

### 1. **Mapping - `gmapping`**
- The robot explores the environment using `SLAM Gmapping`controlled using teleop package. 
- The map is generated and saved as a `.pgm` file with the yaml config. 
- `roslaunch turtlebot_gazebo gmapping_demo.launch` is used to launch mappin and 'turtlebot_gazebo keybaord_teleop.launch' for tele operation of robot.

### 2. **Localization - `AMCL`**
- Once the map is available, `AMCL` is used for localization.
- The robot’s position is estimated using particle filters.

### 3. **Navigation - `move_base`**
- The `move_base` node plans and executes paths from the robot’s current location to the goal.
- Uses `Dijkstra’s Algorithm` for global path planning.
- Local and global costmaps are called in using params to avoid collisions.

### 4. **Virtual Object  - `add_markers` & 'start-home_service'**
- The `add_markers` node visualizes the object in `Rviz`.
- 'start_home_service' node subscribes to odometry to detect when the robot reaches the pickup and drop-off zones with a tolrance. 
- The marker disappears at pickup and reappears at the drop-off location as the robt reaches the positon

### 5. ** Auto Goal - `pick_objects`**
- The `pick_objects` node commands the robot to move between predefined pickup and drop-off locations.
- It interacts with `move_base` to send navigation goals.

## Launch 

Clone the package into your catkin workspace and install the dependeces;

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### **5. Send Robot to Pickup & Drop-off locations (`pick_objects`)**

Navuage to scripts direcotry and 
```bash
./start_home_service.sh
```
