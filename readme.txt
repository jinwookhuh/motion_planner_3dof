1. How to run it in "motion_planner_3dof" folder:
mkdir build && cd build
cmake ..
make -j4

./test_rrt    
./test_ik 

2. summary
=============
- forward and inverse kinematics:
 Since the target and goal poses are given eef poses, I implemented ik/fk to convert the given arm poses to joint angles.
 I verified ik and fk by checking the output of inverse kinematics inputting the output of forward kinematics (input the initial arm joint angle).
 If the output of solutions is the same as the initial arm joint angle, we can verify forward kinematics and inverse kinematics are correct.
 NOTE: IK has redundant solutions for a single target eef pose. I chose a single solution of IK (due to the time limit), but we can try a parallel algorithm
 to find a solution faster by running multiple RRTs.

- RRT:
 I implemented RRT in the configuration space to generate the continuous robot motion in the action space.
 If I had more time, I would try a biased sampling or bidirectional RRT, but I implemented a basic RRT now due to the time limit. 

- Clearance (and collision checking):
 For the collision checking I implemented the Separating Axis Theorem (SAT) since the problem assumes that robot shapes and objects are polygons.
 I check the intersection for the collision checking and I also check the gap between them and use the information for the clearance guarantee.

- Smoothing: 
 I implemented a short-cut algorithm to minimize jerky motions.

- efficient: 
Since it applied the short-cut smoothing, it could have a shorter distance than the original RRT's distance but it doesn't guarantee the optimal.

I appreciate your consideration! 

