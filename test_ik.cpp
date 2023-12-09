#include <iostream>
#include "src/ik.hpp"

using namespace kinematics;

void test_ik()
{

    // Test example. Joint angles
    JointAngles test_arm_angles({M_PI / 3.0,
                                 -M_PI / 6.0,
                                 M_PI / 4.0});

    // foward kinematics.
    ArmPose target_eef_pose = fk_solver(test_arm_angles);

    // Solve inverse kinematics with the output of foward kinematics.
    // If the output of solutions is the same as the inital arm joint angle,
    // we can verify forward kinematics and inverse kinematics are correct.
    // NOTE: IK has redundant solution for a single target eef pose.
    std::vector<JointAngles> solutions = get_eef_pose_solutions(target_eef_pose);

    // Check!!
    std::cout << " test_arm_angles" << std::endl;
    std::cout << test_arm_angles.arr[0] << " " << test_arm_angles.arr[1] << " " << test_arm_angles.arr[2] << std::endl;
    std::cout << " IK_solution" << std::endl;
    for (int i = 0; i < solutions.size(); ++i)
    {
        std::cout << solutions[i].arr[0] << " " << solutions[i].arr[1] << " " << solutions[i].arr[2] << std::endl;
    }
    std::cout << " target_eef_pose" << std::endl;
    std::cout << target_eef_pose.pos[0] << " " << target_eef_pose.pos[1] << " " << target_eef_pose.pos[2] << std::endl;
    std::cout << " FK_check of IK solutions" << std::endl;
    for (int i = 0; i < solutions.size(); ++i)
    {
        ArmPose fk_check = fk_solver(solutions[i]);
        std::cout << fk_check.pos[0] << " " << fk_check.pos[1] << " " << fk_check.pos[2] << std::endl;
    }
    /* Output:
    test_arm_angles
    1.0472 -0.523599 0.785398
    IK_solution
    0.502164 0.523598 0.283235
    1.0472 -0.523598 0.785398
    target_eef_pose
    1.93289 2.76197 1.309
    FK_check of IK solutions
    1.93289 2.76197 1.309
    1.93289 2.76197 1.309
    */

    // Case 2
    target_eef_pose.pos[0] = 1.8;
    target_eef_pose.pos[1] = 0.3;
    target_eef_pose.pos[2] = -M_PI / 2.0;

    solutions = get_eef_pose_solutions(target_eef_pose);

    std::cout << " IK_solution" << std::endl;
    for (int i = 0; i < solutions.size(); ++i)
    {
        std::cout << solutions[i].arr[0] << " " << solutions[i].arr[1] << " " << solutions[i].arr[2] << std::endl;
    }
    std::cout << " target_pose" << std::endl;
    std::cout << target_eef_pose.pos[0] << " " << target_eef_pose.pos[1] << " " << target_eef_pose.pos[2] << std::endl;
    std::cout << " FK_check of IK solutions" << std::endl;
    for (int i = 0; i < solutions.size(); ++i)
    {
        ArmPose fk_check = fk_solver(solutions[i]);
        std::cout << fk_check.pos[0] << " " << fk_check.pos[1] << " " << fk_check.pos[2] << std::endl;
    }

    /* Output:
    IK_solution
    1.11353 -1.32803 -1.3563
    target_pose
    1.8 0.3 -1.5708
    FK_check of IK solutions
    1.8 0.3 -1.5708
    */

    // Case 3
    target_eef_pose.pos[0] = 2.5;
    target_eef_pose.pos[1] = 1.2;
    target_eef_pose.pos[2] = 0.0;

    solutions = get_eef_pose_solutions(target_eef_pose);

    std::cout << " IK_solution" << std::endl;
    for (int i = 0; i < solutions.size(); ++i)
    {
        std::cout << solutions[i].arr[0] << " " << solutions[i].arr[1] << " " << solutions[i].arr[2] << std::endl;
    }
    std::cout << " start_pose" << std::endl;
    std::cout << target_eef_pose.pos[0] << " " << target_eef_pose.pos[1] << " " << target_eef_pose.pos[2] << std::endl;
    std::cout << " FK_check of IK solutions" << std::endl;
    for (int i = 0; i < solutions.size(); ++i)
    {
        ArmPose fk_check = fk_solver(solutions[i]);
        std::cout << fk_check.pos[0] << " " << fk_check.pos[1] << " " << fk_check.pos[2] << std::endl;
    }
    /* Output:
        IK_solution
        1.21367 -1.38707 0.173397
        start_pose
        2.5 1.2 0
        FK_check of IK solutions
        2.5 1.2 -1.49012e-08
    */
}

int main()
{
    test_ik();
    return 0;
}
