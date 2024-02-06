# motion_planner_3dof
### Forward and inverse kinematics:
 Since the target and goal poses are given eef poses, we implemented ik/fk to convert the given arm poses to joint angles.
 We verified ik and fk by checking the output of inverse kinematics inputting the output of forward kinematics (input the initial arm joint angle).
 If the output of solutions is the same as the initial arm joint angle, we can verify forward kinematics and inverse kinematics are correct.
 
 NOTE: IK has redundant solutions for a single target eef pose. we chose a single solution of IK (due to the time limit), but we can try a parallel algorithm
 to find a solution faster by running multiple RRTs.

### RRT:
 We implemented RRT in the configuration space to generate the continuous robot motion in the action space.
 For the improvement, we would try a biased sampling or bidirectional RRT. 

### Clearance (and collision checking):
 For the collision checking we implemented the Separating Axis Theorem (SAT) since the problem assumes that robot shapes and objects are polygons.
 We check the intersection for the collision checking and we also check the gap between them and use the information for the clearance guarantee.

### Smoothing: 
 We implemented a short-cut algorithm to minimize jerky motions.

### Efficiency: 
Since we applied the short-cut smoothing, it could have a shorter distance than the original RRT's distance but it doesn't guarantee the optimal.

## Environment setup
1. OpenCV for visualization:
	
	```
        sudo apt install libopencv-dev
    ```

2. How to run it in "motion_planner_3dof" folder:
        
	```
    mkdir build && cd build
    cmake ..
    make -j4
    
    ./test_rrt    
    ./test_ik 
    ```


