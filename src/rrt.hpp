#ifndef RRT__H_
#define RRT__H_

#include <tuple>
#include <unordered_set>
#include "ik.hpp"

using namespace kinematics;
// Node of RRT tree
struct Node
{
    JointAngles pos;
    Node *parent;

    Node(JointAngles pos_, Node *parent_ = nullptr);
    float distance(const Node &p);
};

using NodeSet = std::vector<Node *>;
using ConfigList = std::vector<JointAngles>;
using VerticeList = std::vector<Position>;
using ObstacleList = std::vector<VerticeList>;
using FlatVector = Position;

class RRT
{
public:
    RRT(JointAngles start_, JointAngles goal_);
    // function to compute Eucledian distance between two configurations.
    float angle_distance(JointAngles a1, JointAngles a2);
    // find the nearest node from a new random configuration.
    Node *FindNearestNode(NodeSet &nodes_, JointAngles new_ang);
    // extend a branch from nearest node toward the random configuration.
    JointAngles step_from_to(JointAngles nn_config_, JointAngles rand_config_);
    // input the dimension (width and height) of the target box.
    void setTargetBoxDim(float w, float h);
    // Compute all shapes of the robot given a configuration.
    ObstacleList findRobotBodies(JointAngles config);
    // Given a pose of the object, width, and height, compute all vertices.
    VerticeList findVertice(ArmPose obs_pose, float w, float h);
    // Add an obstacle defined with the pose, width, and height.
    // It adds a new obstacle to the list whenever this function is called.
    void addObstacle(ArmPose obs_pose, float w, float h);
    // Collision checking function. Separating Axis Theorem (SAT).
    bool collision_check(JointAngles config);
    // project onto the axis. 
    Vec1d ProjectVertices(VerticeList vertices, FlatVector axis);
    // Check the overlapping between two polygons
    bool IntersectPolygons(VerticeList verticesA, VerticeList verticesB, float clearance);
    // find a new node.
    Node *find_newnode(JointAngles rand_config, NodeSet &nodes_);
    // Random generation.
    JointAngles random_sampling(int it);
    // Release the memory .
    void releaseNodes(NodeSet &nodes_);
    // Trajectory generation.
    ConfigList generate_traj();
    // Path smoothing using short-cut algorithm.
    ConfigList smoothing_path(ConfigList path);

private:
    // parameters.
    JointAngles start, goal;
    // maximum iteration to find a solution.
    int max_total_episodes = 100000;
    // tree extension size.
    float step_size = 0.1;
    // the threshold to decide whether the tree reaches the goal
    float arrival_threshold = 0.3;
    ObstacleList obs_set;
    // target object's dimension
    float target_width;
    float target_height;
    // the robot width.
    float robot_width = 0.1;
    // Clearance :minimum clearance guarantee with the obstacle.
    float clearance = 0.03;
};

#endif // RRT__H_
