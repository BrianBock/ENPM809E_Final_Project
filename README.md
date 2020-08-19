# ENPM809E Final Project
## Summer 2020
## Brian Bock

## How to Run
To change the start position of the robot, edit 
`<arg name="x_pos" default="-11.5"/>`

`<arg name="y_pos" default="-5.0"/>` in `~/catkin_ws/src/final_project/launch/maze.launch`
Or, more simply, add the desired start position to the roslaunch statement (example below)

Launch the maze file: `roslaunch final_project maze.launch`. To change the start position of the robot, enter `roslaunch final_project maze.launch [x_pos:=12, y_pos:=3]`

When Gazebo finishes opening, run the program:
`rosrun final_project final_project.py`


## Dependencies
This program requires `numpy`, which is not installed by default with Python or ROS. 