#include <iostream>
#include <math.h>
#include <cstdlib>
#include <stdlib.h>
#include <ctime>
#include <time.h>
#include <float.h>
#include "rrt.hpp"

// Node of RRT tree
Node::Node(JointAngles pos_, Node *parent_)
{
    parent = parent_;
    pos = pos_;
}

float Node::distance(const Node &p)
{
    return sqrt(sq(pos.arr[0] - p.pos.arr[0]) + sq(pos.arr[1] - p.pos.arr[1]) + sq(pos.arr[2] - p.pos.arr[2]));
}

RRT::RRT(JointAngles start_, JointAngles goal_)
{
    start = start_;
    goal = goal_;
}

// function to compute Eucledian distance between two configurations.
float RRT::angle_distance(JointAngles a1, JointAngles a2)
{
    return sqrt(sq(a1.arr[0] - a2.arr[0]) + sq(a1.arr[1] - a2.arr[1]) + sq(a1.arr[2] - a2.arr[2]));
}

// find the nearest node from a new random configuration.
Node *RRT::FindNearestNode(NodeSet &nodes_, JointAngles new_ang)
{

    float nearest_dist = 10000; //maximum distance in C-space
    Node *nearest_node = nullptr;

    for (auto node : nodes_)
    {
        if (angle_distance(node->pos, new_ang) < nearest_dist)
        {
            nearest_dist = angle_distance(node->pos, new_ang);
            nearest_node = node;
        }
    }
    return nearest_node;
}

// extend a branch from nearest node toward the random configuration.
JointAngles RRT::step_from_to(JointAngles nn_config_, JointAngles rand_config_)
{
    float abs_distance = angle_distance(nn_config_, rand_config_);
    if (abs_distance <= step_size)
    {
        return rand_config_;
    }
    else
    {
        JointAngles new_config_ = JointAngles({0, 0, 0});
        for (int i = 0; i < 3; ++i)
        {
            new_config_.arr[i] = nn_config_.arr[i] + (rand_config_.arr[i] - nn_config_.arr[i]) * step_size / abs_distance;
        }
        // std::cout << new_config_.arr[0] << " " << new_config_.arr[1] <<" " << new_config_.arr[2] << std::endl;

        return new_config_;
    }
}

// project vertices onto the axis and check overlapping in the projected space.
Vec1d RRT::ProjectVertices(VerticeList vertices, FlatVector axis)
{
    float min = FLT_MAX;
    float max = FLT_MIN;

    for (int i = 0; i < vertices.size(); i++)
    {
        FlatVector v = vertices[i];
        float proj = v.x * axis.x + v.y * axis.y;

        if (proj < min)
        {
            min = proj;
        }
        if (proj > max)
        {
            max = proj;
        }
    }
    Vec1d min_max;
    min_max.push_back(min);
    min_max.push_back(max);

    return min_max;
}

// Check the overlapping between two polygons. 
// Polygon is represented by the ordered (clockwised) vertices.
// Clearance :minimum clearance guarantee with the obstacle.
bool RRT::IntersectPolygons(VerticeList verticesA, VerticeList verticesB, float clearance)
{
    // project vertices vertical to the edges of a polygon! 
    for (int i = 0; i < verticesA.size(); i++)
    {
        FlatVector va = verticesA[i];
        // vertices should be clockwise order!
        FlatVector vb = verticesA[(i + 1) % verticesA.size()];

        FlatVector edge = vb - va;
        FlatVector axis = FlatVector(-edge.y, edge.x);
        float axis_len = sqrt(sq(axis.x) + sq(axis.y));
        axis.x = axis.x / axis_len;
        axis.y = axis.y / axis_len;

        Vec1d MinMaxA = ProjectVertices(verticesA, axis);
        Vec1d MinMaxB = ProjectVertices(verticesB, axis);

        if (MinMaxA[0] >= (MinMaxB[1] + clearance) || MinMaxB[0] >= (MinMaxA[1] + clearance))
        {
            return false;
        }
    }

    // need to check edges of another polygon! 
    for (int i = 0; i < verticesB.size(); i++)
    {
        FlatVector va = verticesB[i];
        // vertices should be clockwise order!
        FlatVector vb = verticesB[(i + 1) % verticesB.size()];

        FlatVector edge = vb - va;
        FlatVector axis = FlatVector(-edge.y, edge.x);
        float axis_len = sqrt(sq(axis.x) + sq(axis.y));
        axis.x = axis.x / axis_len;
        axis.y = axis.y / axis_len;

        Vec1d MinMaxA = ProjectVertices(verticesA, axis);
        Vec1d MinMaxB = ProjectVertices(verticesB, axis);

        if (MinMaxA[0] >= (MinMaxB[1] + clearance) || MinMaxB[0] >= (MinMaxA[1] + clearance))
        {
            return false;
        }
    }

    return true;
}

// Find the clockwise vertices using homogeneous transformation
VerticeList RRT::findVertice(ArmPose obs_pose, float w, float h)
{
    Transform t;
    float cwz = cos(obs_pose.pos[2]);
    float swz = sin(obs_pose.pos[2]);
    t(0, 0) = cwz;
    t(0, 1) = -swz;
    t(0, 2) = obs_pose.pos[0];
    t(1, 0) = swz;
    t(1, 1) = cwz;
    t(1, 2) = obs_pose.pos[1];

    VerticeList obs_vertice;
    // first vertice
    t.translateY(h / 2.0);
    Position new_vertice(t(0, 2), t(1, 2));
    obs_vertice.push_back(new_vertice);
    // second vertice
    t.translateX(w);
    new_vertice.x = t(0, 2);
    new_vertice.y = t(1, 2);
    obs_vertice.push_back(new_vertice);
    // third vertice
    t.translateY(-h);
    new_vertice.x = t(0, 2);
    new_vertice.y = t(1, 2);
    obs_vertice.push_back(new_vertice);
    // forth vertice
    t.translateX(-w);
    new_vertice.x = t(0, 2);
    new_vertice.y = t(1, 2);
    obs_vertice.push_back(new_vertice);

    return obs_vertice;
}
// add the obstacle in the obstacle set.
void RRT::addObstacle(ArmPose obs_pose, float w, float h)
{
    obs_set.push_back(findVertice(obs_pose, w, h));
}
// setting the target box dimension.
void RRT::setTargetBoxDim(float w, float h)
{
    target_width = w;
    target_height = h;
}

// Compute all shapes of the robot given a configuration.
// The shape is represented by clockwise vertices. 
ObstacleList RRT::findRobotBodies(JointAngles config)
{
    ObstacleList RobotBodyList;

    Transform t;
    t.translateY(DeviceConstants::len0);
    t.rotateZ(config.arr[0]);
    RobotBodyList.push_back(findVertice(getpose3D(t), DeviceConstants::len1, robot_width));
    t.translateX(DeviceConstants::len1);
    t.rotateZ(config.arr[1]);
    RobotBodyList.push_back(findVertice(getpose3D(t), DeviceConstants::len2, robot_width));
    t.translateX(DeviceConstants::len2);
    t.rotateZ(config.arr[2]);
    RobotBodyList.push_back(findVertice(getpose3D(t), DeviceConstants::len3, robot_width));
    t.translateX(DeviceConstants::len3);
    RobotBodyList.push_back(findVertice(getpose3D(t), target_width, target_height));

    return RobotBodyList;
}

// Collision checking function. Separating Axis Theorem (SAT).
bool RRT::collision_check(JointAngles config)
{
    ObstacleList body_set = findRobotBodies(config);
    for (int i = 0; i < body_set.size(); i++)
    {
        for (int j = 0; j < obs_set.size(); j++)
        {
            if (IntersectPolygons(body_set[i], obs_set[j], clearance))
            {
                return true;
            }
        }
    }

    return false;
}
// find a new node.
Node *RRT::find_newnode(JointAngles rand_config_, NodeSet &nodes_)
{

    // newnode
    Node *nn_node = FindNearestNode(nodes_, rand_config_);
    JointAngles new_config = step_from_to(nn_node->pos, rand_config_);
    if (collision_check(new_config))
    {
        return nullptr;
    }
    else
    {
        Node *newnode = new Node(new_config, nn_node);
        return newnode;
    }
}

// Random configuration generation.
JointAngles RRT::random_sampling(int it)
{
    srand(time(NULL) * it);

    float j1 = ((float)rand() / RAND_MAX) * M_PI;
    float j2 = ((float)rand() / RAND_MAX - 0.5) * M_PI;
    float j3 = ((float)rand() / RAND_MAX - 0.5) * M_PI;

    JointAngles rand_angles({j1, j2, j3});

    return rand_angles;
}

// Release the memory.
void RRT::releaseNodes(NodeSet &nodes_)
{
    for (auto it = nodes_.begin(); it != nodes_.end();)
    {
        delete *it;
        it = nodes_.erase(it);
    }
}

// Trajectory generation.
ConfigList RRT::generate_traj()
{
    NodeSet TreeNode;
    TreeNode.reserve(max_total_episodes);
    TreeNode.push_back(new Node(start));

    bool is_success = false;
    int episode = 0;
    Node *current = nullptr;

    // until finding a solution or reach the max iteration.
    while (episode < max_total_episodes)
    {
        episode = episode + 1;
        
        // random configuration.
        JointAngles rand_config = random_sampling(episode);
        // find a new node.
        Node *newnode = find_newnode(rand_config, TreeNode);

        if (newnode != nullptr)
        {
            // valid newnode
            TreeNode.push_back(newnode);
        }
        else
        {
            continue;
        }
        // check it reaches the roal.
        if (angle_distance(newnode->pos, goal) < arrival_threshold)
        {
            // close to the goal
            is_success = true;
            Node *goal_node = new Node(goal, newnode);
            TreeNode.push_back(goal_node);
            current = goal_node;
            break;
        }
    }

    ConfigList path;
    ConfigList final_path;

    // it reaches the goal!
    if (is_success)
    {
        while (current != nullptr)
        {
            path.push_back(current->pos);
            current = current->parent;
        }

        // reverse trajectory to make it from start configuration.
        for (auto it = path.rbegin(); it != path.rend(); ++it)
        {
            final_path.push_back(*it);
        }
    }
    releaseNodes(TreeNode);
    return final_path;
}

// Path smoothing. Shortcut algorithm.
ConfigList RRT::smoothing_path(ConfigList path)
{

    int smmthiters = 30;
    int iter = 1;

    while (iter <= smmthiters)
    {
        int m = path.size();
        std::vector<float> dis_cost(m);

        dis_cost[0] = 0;
        for (int k = 1; k < dis_cost.size(); k++)
        {
            dis_cost[k] = angle_distance(path[k], path[k - 1]) + dis_cost[k - 1];
        }
        float cost_init = dis_cost[dis_cost.size() - 1];


        srand(time(NULL) * iter);

        float s1 = ((float)rand() / RAND_MAX) * cost_init;
        float s2 = ((float)rand() / RAND_MAX) * cost_init;

        if (s2 < s1)
        {
            float temps = s1;
            s1 = s2;
            s2 = temps;
        }

        int ipx = 0;
        int jpx = 0;
        for (int k = 1; k < dis_cost.size(); k++)
        {
            if (dis_cost[k] > s1)
            {
                ipx = k - 1;
                break;
            }
        }

        for (int k = 0; k < dis_cost.size(); k++)
        {
            if (dis_cost[k] >= s2)
            {
                jpx = k;
                break;
            }
        }

        if ((jpx - ipx) < 3)
        {
            iter = iter + 1;
            continue;
        }

        JointAngles gamma1 = path[ipx];
        JointAngles gamma2 = path[jpx];

        int num_newset = int(angle_distance(gamma1, gamma2) / step_size);
        if (num_newset < 1)
        {
            num_newset = 1;
        }

        ConfigList gamma;
        gamma.push_back(gamma1);
        bool col = false;
        for (int idx = 1; idx < num_newset; idx++)
        {
            JointAngles new_gamma = gamma[idx - 1];
            new_gamma.arr[0] = new_gamma.arr[0] + (gamma2.arr[0] - gamma1.arr[0]) / num_newset;
            new_gamma.arr[1] = new_gamma.arr[1] + (gamma2.arr[1] - gamma1.arr[1]) / num_newset;
            new_gamma.arr[2] = new_gamma.arr[2] + (gamma2.arr[2] - gamma1.arr[2]) / num_newset;
            if (collision_check(new_gamma))
            {
                col = true;
                break;
            }
            else
            {
                gamma.push_back(new_gamma);
            }
        }
        if (col == true)
        {
            iter = iter + 1;
            continue;
        }

        // No collision between gamma1 and gamm2
        ConfigList new_path_points;
        for (int i = 0; i < ipx; i++)
        {
            new_path_points.push_back(path[i]);
        }
        for (int i = 0; i < gamma.size(); i++)
        {
            new_path_points.push_back(gamma[i]);
        }
        for (int i = jpx; i < path.size(); i++)
        {
            new_path_points.push_back(path[i]);
        }

        path = new_path_points;

        // m = path.size();

        // dis_cost[0] = 0;
        // for (int k = 1; k < dis_cost.size(); k++)
        // {
        //     dis_cost[k] = angle_distance(path[k], path[k - 1]) + dis_cost[k - 1];
        // }
        // cost_init = dis_cost[dis_cost.size() - 1];

        iter = iter + 1;
    }
    return path;
}