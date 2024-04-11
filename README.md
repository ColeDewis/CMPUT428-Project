# CMPUT 428 Project
By Cole Dewis and Anya Hanatuke

# Running the Project
This project uses a Kinova Gen3 arm, and thus will need one to run visual servoing. However, the GUI can still be run without robot connection.

First, clone the repo. This project uses Docker and VSCode Dev Containers to containerize the development process. Opening the project in VSCode with the Dev Containers extension should allow you to open the project in the container, which will install all necessary dependencies and open an environment to work with the code. From there, `cameras_start` in terminal will run the necessary ROS commands to start the GUI and 2 cameras, and `robot_start` can be used to run everything that cameras_start would have alongside running necessary code for robot communication as well.

# Libraries
This makes use of the kinova ros_kortex package at https://github.com/Kinovarobotics/ros_kortex as well as the kortex_bringup package from https://github.com/cjiang2/kortex_bringup
