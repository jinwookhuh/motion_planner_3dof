#include <iostream>
#include "src/rrt.hpp"
#include "src/visualization.hpp"

void test_rrt()
{
    ArmPose start_eef_pose({2.5, 1.2, 0.0});
    ArmPose goal_eef_pose({1.8, 0.3, -M_PI / 2.0});

    std::vector<JointAngles> start_solutions = get_eef_pose_solutions(start_eef_pose);
    std::vector<JointAngles> goal_solutions = get_eef_pose_solutions(goal_eef_pose);
    // JointAngles start({1.11353, -1.32803, -1.3563}); //0,0,0
    // JointAngles goal({1.21367, -1.38707, 0.173397}); //M_PI / 3.0,- M_PI / 6.0, M_PI / 4.0

    if (start_solutions.size() == 0 || goal_solutions.size() == 0)
    {
        std::cout << "infeasible target or goal pose" << std::endl;
    }
    // I choose the first solution (due to time limit),
    // but we can try a parallel algorithm to find
    // the solution faster by running multiple RRTs.
    RRT new_rrt = RRT(start_solutions[0], goal_solutions[0]);

    // first obstacle
    // new_rrt.addObstacle(ArmPose({2.5, 1.7, 0}), 0.4, 0.3);
    ArmPose obstacle1({2.1, 1.6, 0}); //ArmPose({2.0, 1.7, 0})  //{2.1, 1.6, 0}
    float obs1_w = 0.4;
    float obs1_h = 0.3;
    new_rrt.addObstacle(obstacle1, obs1_w, obs1_h);
    // second obstacle
    // new_rrt.addObstacle(ArmPose({2.8, 0.7, 0}), 0.4, 0.3);
    ArmPose obstacle2({2.1, 0.8, 0}); //ArmPose({2.8, 0.7, 0})  //{2.1, 0.8, 0}
    float obs2_w = 0.4;
    float obs2_h = 0.3;
    new_rrt.addObstacle(obstacle2, obs2_w, obs2_h);
    //  if you want, you can add more obstacles.

    // set the dimension of the taget object. width and height.
    // The pose of the target box is decided by the goal eef pose.
    new_rrt.setTargetBoxDim(0.4, 0.3);
    auto path = new_rrt.generate_traj();
    // check
    if (path.size() == 0)
    {
        std::cout << "No solution" << std::endl;
    }
    // print the solution
    else
    {
        std::cout << "done"
                  << "\n";
        for (auto &angles : path)
        {
            std::cout << angles.arr[0] << " " << angles.arr[1] << " " << angles.arr[2] << "\n";
        }
    }

    // Smoothing!!!!
    if (path.size() != 0)
    {   
        auto smooth_path = new_rrt.smoothing_path(path);
        std::cout << "smooth_path"
                  << "\n";
        for (auto &angles : smooth_path)
        {
            std::cout << angles.arr[0] << " " << angles.arr[1] << " " << angles.arr[2] << "\n";
        }
        // visualizer in workspace
        Visualization new_visualizer = Visualization();
        // List of obstalce
        std::vector<VerticeList> list_vertices;
        list_vertices.emplace_back(new_rrt.findVertice(obstacle1, obs1_w, obs1_h));
        list_vertices.emplace_back(new_rrt.findVertice(obstacle2, obs2_w, obs2_h));
        new_visualizer.showWorkspace(new_rrt, list_vertices, smooth_path);
    }
}

    int main()
    {
        test_rrt();
        return 0;
    }
