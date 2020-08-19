# ENPM809E Final Project
## Summer 2020
## Brian Bock

## How to Run
To change the start position of the robot, edit 
`<arg name="x_pos" default="-11.5"/>`

`<arg name="y_pos" default="-5.0"/>` in `~/catkin_ws/src/final_project/launch/maze.launch`
Or, more simply, add the desired start position to the roslaunch statement (example below)

Launch the maze file: `roslaunch final_project maze.launch`. To change the start position of the robot, enter `roslaunch final_project maze.launch [x_pos:=12, y_pos:=3]`. For reasons that are not immediately clear, the x position when entered this way is taken as a negative value. This is not a problem within the launch file. 

When Gazebo finishes opening, run the program:
`rosrun final_project final_project.py`


If the robot has trouble picking the right wall, change the value of `inc` in `find_wall()`. For maze 1, 5 degrees works well. For maze 2, 60 degrees seems to work better (for the default start position).
The robot often struggles to get through the upper portion of the narrow angled section at the bottom edge of room 7. Sometimes the robot will recover, sometimes it just gets stuck. If it gets stuck, respawn the robot at (14,1) and restart the program. It will find the wall, get to it, and then finish the maze out. 

## Dependencies
This program requires `numpy`, which is not installed by default with Python or ROS. 