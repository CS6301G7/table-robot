Steps to Run the project

Getting started
 - clone the repo into the src of your ROS workspace
 - Build the ROS workspace and source your workspace
 - "cd src/table-robot"
 - copy the models folder under table-robot to the ~/.gazebo folder "cp -r models ~/.gazebo"

Launching Gazebo + MoveIt
 - "cd table-robot/launch", then run "roslaunch project_grasp.launch"
 - Launch moveit with "fetch_moveit_config move_group.launch"

Running the code
 - "cd table-robot/scripts"
 - python grasp_tableware.py
 