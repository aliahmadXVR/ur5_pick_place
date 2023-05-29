# Run Pick & Place

## Load Gazebo & RVIZ

```roslaunch ur5_gripper_moveit_config demo_gazebo.launch```

## Run the Pick & Place Node

```rosrun ur5_trajectory_gen pick_and_place.py```

![](https://github.com/aliahmadXVR/ur5_pick_place/blob/master/pick_place.gif)


# Run Straight Line Motion & Show Collisions

## Load Gazebo & RVIZ

```roslaunch ur5_gripper_moveit_config demo_gazebo.launch```

## Run the Node

```rosrun ur5_trajectory_gen line_move.py```


# Run Straight Line With Fixed Force

## Load Gazebo & RVIZ

```roslaunch ur5_gripper_moveit_config demo_gazebo.launch```

## Run the Node

```rosrun ur5_trajectory_gen line_move_force.py```
