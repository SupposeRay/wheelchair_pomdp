# wheelchair_pomdp package

This package is the POMDP model compatible with HyP-DESPOT-ALPHA solver for wheelchair navigation in the simulation world. It works with ubuntu 18, cuda 10.2 and RTX 2080Ti GPU.
For compatibility with cuda 10.2, make sure [eigen3](https://eigen.tuxfamily.org/dox/GettingStarted.html) version is atleast 3.3.9.

To use this package, clone this package in your catkin workspace. Name the folder wheelchair_pomdp_GPU_version 

Then clone [hyp-despot-alpha package static_scene_shared_control_wheelchair branch](https://github.com/ntu-rris/hyp-despot-alpha-dev/tree/static_scene_shared_control_wheelchair) in catkin workspace

Assuming both the packages are in the same folder, symlink with name wheelchair_pomdp_GPU_version inside hyp-despot-alpha-dev/src/HyP_examples should be correctly pointing to wheelchair_pomdp_GPU_version directory.


Then compile the package from catkin workspace using command:
```
catkin build hyp_despot

```

Then package can be run with the command:
```

rosrun hyp_despot hyp_despot_wheelchair_pomdp <additional arguments>

```
## To test the hyp_despot_wheelchair_pomdp in the simulation environment

First, clone [wheelchair_pomdp repo](https://github.com/ntu-rris/wheelchair_pomdp/tree/intermediate_goals), [scwheelchair-simulation repo](https://github.com/ntu-rris/scwheelchair-simulation/tree/w_goal_recalculation), and [scat_autonomous_packages repo](https://github.com/ntu-rris/scat_autonomous_packages/tree/wheelchair_pomdp) to your workspace and compile them under different branches repsectively.

> wheelchair_pomdp repo: **intermediate_goals branch**
> scwheelchair-simulation repo: **w_goal_recalculation branch**
> scat_autonomous_packages repo: **wheelchair_pomdp branch**

1. Run Gazebo simulation environment (in scwheelchair-simulation repo)

    `roslaunch mbot_gazebo view_nuric_gazebo_pomdp_test.launch`

2. Run the Rviz visualization script (in scwheelchair-simulation repo)

    `roslaunch mbot_navigation navigation_pomdp.launch`

3. Run the Voronoi path planner (in scat_autonomous_packages repo)

    `roslaunch scat_move_base move_base.launch`

4. Pin a point on the map to set the destination by clicking the "2D Nav Goal" in Rviz

5. Select the control method: joystick or keyboard

* Joystick: plug the cable connected to the arudino joystick into the USB port and run the arduino joystick (in scwheelchair-simulation repo)

  Change [line 49 in wheelchair_gazebo.cpp](https://github.com/ntu-rris/wheelchair_pomdp/blob/intermediate_goals/src/wheelchair_gazebo.cpp#L49) to 
  
  `joystick_subscriber_ = node_handle_->subscribe("/arduino/joystick", 1, &GazeboWheelchairWorld::joystickCallback, this);`

  Then re-build the repo and run

  `sudo chmod 666 /dev/ttyUSB0`

  `rosrun rosserial_python serial_node.py /dev/ttyUSB0`

* Keyboard: run the keyboard control command in a detached terminal (in scwheelchair-simulation repo)

  Change [line 49 in wheelchair_gazebo.cpp](https://github.com/ntu-rris/wheelchair_pomdp/blob/intermediate_goals/src/wheelchair_gazebo.cpp#L49) to 
  
  `joystick_subscriber_ = node_handle_->subscribe("/keyboard", 1, &GazeboWheelchairWorld::joystickCallback, this);`

  Then re-build the repo and run

  `roslaunch control_keyboard control_keyboard.launch`

6. Run the hyp_despot_wheelchair_pomdp code

* Run in multi-threaded CPU mode

  `rosrun hyp_despot hyp_despot_wheelchair_pomdp -n 100 --max-policy-simlen 5 -s 800 -t 0.15 --CPU 1 --num_thread 20`

* Run in multi-threaded GPU mode

  `rosrun hyp_despot hyp_despot_wheelchair_pomdp -n 100 --max-policy-simlen 5 -s 800 -t 0.15 --CPU 1 --num_thread 20 --GPU 1 --GPUID 0`

7. To manually control the agent:

* Joystick:
  
  Change [line 5 in input_converter_config.yaml](https://github.com/ntu-rris/scwheelchair-simulation/blob/w_goal_recalculation/input_converter/config/input_converter_config.yaml#L5) to 
  
  `input_source: "Joystick"`

* Keyboard:
  
  Change [line 5 in input_converter_config.yaml](https://github.com/ntu-rris/scwheelchair-simulation/blob/w_goal_recalculation/input_converter/config/input_converter_config.yaml#L5) to 
  
  `input_source: "Keyboard"`

  Run in terminal 

  `roslaunch input_converter input_converter.launch`

  Then use the joystick or the detached terminal in Step 5 to control the agent



## To test the goal-based shared-DWA in the simulation environment

1. Run Gazebo simulation environment (in scwheelchair-simulation repo)

    `roslaunch mbot_gazebo view_nuric_gazebo_pomdp_test.launch`

2. Run the Rviz visualization script (in scwheelchair-simulation repo)

    `roslaunch mbot_navigation navigation_pomdp.launch`

3. Run the Voronoi path planner (in scat_autonomous_packages repo)

    Uncomment [line 72 to line 76 in move_base.launch](https://github.com/ntu-rris/scat_autonomous_packages/blob/wheelchair_pomdp/scat_move_base/launch/move_base.launch#L72)

    Then

    `roslaunch scat_move_base move_base.launch`

4. Run the belief update package

    `roslaunch path_belief_update belief_update.launch`

5. Select the control method: joystick or keyboard

* Joystick: plug the cable connected to the arudino joystick into the USB port and run the arduino joystick (in scwheelchair-simulation repo)

  `sudo chmod 666 /dev/ttyUSB0`

  `rosrun rosserial_python serial_node.py /dev/ttyUSB0`

* Keyboard: run the keyboard control command in a detached terminal (in scwheelchair-simulation repo)

  `roslaunch control_keyboard control_keyboard.launch`

6. Control the agent

    Comment line 7 and uncomment line 8 in [input_converter.launch](https://github.com/ntu-rris/scwheelchair-simulation/blob/w_goal_recalculation/input_converter/launch/input_converter.launch#L7)

* Joystick:
  
  Change [line 5 in input_converter_config.yaml](https://github.com/ntu-rris/scwheelchair-simulation/blob/w_goal_recalculation/input_converter/config/input_converter_config.yaml#L5) to 
  
  `input_source: "Joystick"`

* Keyboard:
  
  Change [line 5 in input_converter_config.yaml](https://github.com/ntu-rris/scwheelchair-simulation/blob/w_goal_recalculation/input_converter/config/input_converter_config.yaml#L5) to 
  
  `input_source: "Keyboard"`

  Run in terminal 

  `roslaunch input_converter input_converter.launch`

7. Pin a point on the map to set the destination by clicking the "2D Nav Goal" in Rviz and use the joystick or the detached terminal in Step 5 to control the agent
  
## Necessary changes for GPU model to work
* Specify Obs_type as OBS_INT_ARRAY and num_Obs_element_in_GPU in InitializeDefaultParametes function. See this [commit](https://github.com/ntu-rris/wheelchair_pomdp/commit/a957e769120b32f54953b7324ce19bd483aaf1a6)
* Make sure Lower and Upper Bounds are updated in CPU functions. See this [commit](https://github.com/ntu-rris/wheelchair_pomdp/commit/0d0fa130f45c3d90bcbba1f35a4d17deb367d025)
* The Dvc_Step function should work even when the observation passed is NULL as default policy passes obs as null in GPU. See this [commit](https://github.com/ntu-rris/wheelchair_pomdp/commit/a901b994d3d95bbba6b058a80a8f86ef6abb58c4)
  
