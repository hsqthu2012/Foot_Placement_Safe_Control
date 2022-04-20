# Foot_Placement_Safe_Control
Quadruped robot foot placement safe control using OCS2.

The simulator is based on [OCS2](https://github.com/leggedrobotics/ocs2), the doc can be found in [OCS2 Doc](https://leggedrobotics.github.io/ocs2/overview.html).
The realization of foot placement safe control has referenced the work by [matheecs](https://github.com/matheecs/ocs2_legged_robot_annotated).

## Build

Build OCS2 following the [installation guide](https://leggedrobotics.github.io/ocs2/installation.html). Then build ocs2_legged_robot_annotated:

```Bash
catkin build ocs2_legged_robot_annotated
source devel/setup.bash
```

## Run the simulation

```Bash
roslaunch ocs2_legged_robot_annotated legged_robot.launch
```

In case the python node `draw_polygon.py` is not launched successfully, one can run the python script manually.

```Bash
python3 src/ocs2_legged_robot_annotated/scripts/draw_polygon.py
```

There are three new terminal will be launched, 1) Showing the current time, 2) Enter the gait type, 3) Enter the target pose (X Y Z yaw, relatively).
The gait type can be chosen within

```Bash
[0]: stance
[1]: trot
[2]: standing_trot
[3]: flying_trot
[4]: pace
[5]: standing_pace
[6]: dynamic_walk
[7]: static_walk
[8]: amble
[9]: lindyhop
[10]: skipping
[11]: pawup
```
