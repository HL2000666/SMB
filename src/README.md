## Instructions

This repo is slightly modified version of the part of the code provided in [ROS workshop Excercises](https://rsl.ethz.ch/education-students/lectures/ros.html). For additional information one can follow some of the lecture input provided there. However, everything required is already provided or referenced in this README.

Use this code within your `catkin_ws`.

### Useful commands

Launch the simulation in Gazebo:
```bash
roslaunch smb_gazebo smb_gazebo.launch
```

Open in RVIZ:
```bash
rosrun rviz rviz -d src/smb_common_v2/smb_config.rviz
```

Useful stuff:

```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py # Control robot with the keyboard
rosrun rqt_tf_tree rqt_tf_tree # check TF tree
```


### Task

Write your code in `smb_highlevel_controller`. Following is required (please respect the order):
- Do the subtasks 4. to 7. from [Excercise 2](https://ethz.ch/content/dam/ethz/special-interest/mavt/robotics-n-intelligent-systems/rsl-dam/ROS2021/lec2/Exercise%20Session%202.pdf)
- Do the subtasks 3. to 8. from [Excercise 3](https://ethz.ch/content/dam/ethz/special-interest/mavt/robotics-n-intelligent-systems/rsl-dam/ROS2021/lec3/Exercise%20Session%203.pdf)


### General guidelines

- Please try to use modular approach to developement (object oriented approach)
- All parameters should be defined and read from `.yaml` files for easy reconfiguration
- Use `.launch` files with parameters to automate running the code 