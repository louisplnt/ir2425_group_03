#!/bin/bash

# Function to source the setup file and open a new terminal for a command
run_in_new_terminal() {
  gnome-terminal -- bash -c "echo 'Sourcing setup.bash...'; source ~/catkin_ws/devel/setup.bash; echo 'Running: $1'; eval $1; exec bash"
}

# Start the simulation and MoveIt
run_in_new_terminal "roslaunch tiago_iaslab_simulation start_simulation.launch world_name:=iaslab_assignment2"

sleep 15

# Launch AprilTag
run_in_new_terminal "roslaunch tiago_iaslab_simulation apriltag2.launch"

# Launch Navigation stack
run_in_new_terminal "roslaunch tiago_iaslab_simulation navigation.launch"

# Run get_straightline_node
run_in_new_terminal "rosrun tiago_iaslab_simulation get_straightline_node"

