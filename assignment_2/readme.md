# GROUP 03
* Nathan Mbock, nathanjeanemmanueljolly.mbock@studenti.unipd.it
* Damien Brocheton, damien.brocheton@studenti.unipd.it
* Louis Plantey, louis.plantey@studenti.unipd.it

# IR2425 Group 03 Assignment 2

## Build Instructions

To set up and build the project, follow these steps:

1. **Clone the repository**:  
   Clone the repository into the `src` folder of your Catkin workspace:
   ```bash
   cd ~/catkin_ws/src
   git clone https://nathan_mbock-admin@bitbucket.org/nathan_mbock/ir2425_group_03.git
   ```

2. **Build the project**:  
   From the root of your Catkin workspace, run:
   ```bash
   cd ~/catkin_ws
   catkin build
   ```

## Launch Instructions

### Launching Simulation

To launch the simulation, use the file `launch.sh` (launch: apriltag2, navigation, get_straightline_node and the simulation iaslab_assignment2)

```bash
 ~/catkin_ws/src/ir2425_group_03/assignment_2/launch.sh 
```

### Launching Nodes

To run the simulation, use the file `run.sh` to run all the nodes details in the next part.

```bash
 ~/catkin_ws/src/ir2425_group_03/assignment_2/run.sh 
```


### Available Nodes

The project consists of the following nodes:

1. **`node_a.py`**:  
   - Retrive the m and q coefficients from `/straight_line_srv.`<br> 
   - Send the coefficients to node_c.

2. **`node_b_main.py`**:  
   - Subscribes to `/apriltags_ids` to receive IDs.<br>
   - Creates a collection of positions for the IDs.<br>
   - TF transformations of these positions.<br>
   - Send the positions to node_c.

3. **`node_b_camera.py`**:  
   - Displays the camera feed.<br>
   - Adjusts the camera angle towards the ground for detecting AprilTags.

5. **`node_c.py`**:  
   - Move the robot and his arm.<br>
   - Request the coefficients and the positions of the AprilTags.<br>
   - Do the Pick and Place routine.





