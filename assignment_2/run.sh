#!/bin/bash



# Function to source the setup file and open a new terminal for a command
run_in_new_terminal() {
  gnome-terminal -- bash -c "echo 'Sourcing: source ~/catkin_ws/devel/setup.bash'; source ~/catkin_ws/devel/setup.bash; echo 'Running : $1'; eval $1; exec bash"
}

# Run node_a 
run_in_new_terminal "rosrun assignment_2 node_a.py"

# Run node_b
run_in_new_terminal "rosrun assignment_2 node_b_camera.py"
run_in_new_terminal "rosrun assignment_2 node_b_main.py"


# Run node_c
run_in_new_terminal "rosrun assignment_2 node_c.py"

