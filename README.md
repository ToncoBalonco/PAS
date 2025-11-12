Fanuc CRX10iA ROS2 Project

This repository contains a ROS2 project for simulating and controlling the Fanuc CRX10iA collaborative robot.
The setup includes robot visualization in RViz, controller management, and trajectory/position publishing scripts.

Prerequisites

Make sure you have:

ROS2 (Humble or later)

colcon build tools

rviz2

ros2_control and related controller packages installed

1. Setting up the workspace

Open a terminal and create a new workspace:

```bash
mkdir test_1
cd test_1
```

Clone the repository:
```bash
git clone <repository_url>
```

Build the workspace:
```bash
colcon build
```

Source the setup file:
```bash
source install/setup.bash
```
2. Launching the robot visualization and GUI

Launch the robot model display:
```bash
ros2 launch fanuc_crx10ia_support display.launch.py
```

Then, in RViz:

Set the Fixed Frame to:

base_link


Add the RobotModel display.

For the Description Topic, select:

/robot_description


At this point, the robot should be visible in RViz.

3. Launching the controllers

Open a new terminal, then:
```bash
cd test_1
source install/setup.bash
ros2 launch fanuc_crx10ia_support ros2_control.launch.py
```
4. Running trajectory tracking

To run trajectory tracking, open another terminal and execute:
```bash
ros2 launch fanuc_crx10ia_support publish_joint_trajectory.launch.py
```

If you encounter errors, you can alternatively run the Python script directly:
```bash
python3 ~/fanuc_cuk_ws/fanuc_crx10ia_support/scripts/publish_joint_trajectory.py

```
(Replace the path with the correct one to your Python script.)

5. Publishing joint positions only

If you prefer to publish only joint positions (not full trajectories), follow these steps:

Ensure that the forward_position_controller exists:
```bash
ros2 run controller_manager spawner forward_position_controller --inactive
```

Deactivate the trajectory controller:
```bash
ros2 control set_controller_state joint_trajectory_controller inactive
```

Activate the forward position controller:
```bash
ros2 control set_controller_state forward_position_controller active
```

Then run:
```bash
ros2 launch fanuc_crx10ia_support publish_joint_positions.launch.py
```

If there are issues, you can manually run the Python script:
```bash
python3 ~/fanuc_cuk_ws/fanuc_crx10ia_support/scripts/publish_joint_positions.py
```

(Again, update the path as needed.)

Notes

Always source your workspace before launching any node:
```bash
source install/setup.bash
```

Make sure the joint_state_broadcaster and relevant controllers are active.

You can monitor controller states with:

ros2 control list_controllers
