#include <vector>
#include <math.h>
#include <iostream>
#include "ik.hpp"

namespace kinematics
{

  float sq(const float v)
  {
    return v * v;
  }

  Position::Position() : x(0), y(0)
  {
  }

  Position::Position(float x, float y) : x(x), y(y)
  {
  }

  float Position::distance(const Position &p) const
  {
    return sqrt(sq(x - p.x) + sq(y - p.y));
  }

  // Transform class
  Transform::Transform()
  {
    clear();
  }

  void Transform::clear()
  {
    // Initialize to identity matrix:
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++)
        t[i][j] = 0;

    t[0][0] = 1;
    t[1][1] = 1;
    t[2][2] = 1;
  }

  float const Transform::operator()(int i, int j) const
  {
    return t[i][j];
  }

  float &Transform::operator()(int i, int j)
  {
    return t[i][j];
  }

  Transform &Transform::translateX(float x)
  {
    t[0][2] += t[0][0] * x;
    t[1][2] += t[1][0] * x;
    return *this;
  }

  Transform &Transform::translateY(float y)
  {
    t[0][2] += t[0][1] * y;
    t[1][2] += t[1][1] * y;
    return *this;
  }

  Transform &Transform::rotateZ(float a)
  {
    float ca = cos(a);
    float sa = sin(a);
    for (int i = 0; i < 2; i++)
    {
      float tx = t[i][0];
      float ty = t[i][1];
      t[i][0] = ca * tx + sa * ty;
      t[i][1] = -sa * tx + ca * ty;
    }
    return *this;
  }

  Transform transform6D(const float p[3])
  {
    Transform t;

    float cwz = cos(p[3]);
    float swz = sin(p[3]);
    t(0, 0) = cwz;
    t(0, 1) = -swz;
    t(0, 2) = p[0];
    t(1, 0) = swz;
    t(1, 1) = cwz;
    t(1, 2) = p[1];
    return t;
  }

  ArmPose getpose3D(const Transform &t1)
  {
    return ArmPose({t1(0, 2), t1(1, 2), atan2(t1(1, 0), t1(0, 0))});
  }

  // compute the position of tip of the second link.
  Position get_l2(const ArmPose &goal_pose)
  {
    float ex = goal_pose.pos[0];
    float ey = goal_pose.pos[1] - DeviceConstants::len0;
    float eef_alpha = goal_pose.pos[2];

    auto wx = ex - DeviceConstants::len3 * cos(eef_alpha);
    auto wy = ey - DeviceConstants::len3 * sin(eef_alpha);

    return Position(wx, wy);
  }

  // find the second joint angle using law of cosines.
  Vec1d get_j2_values(const Position &l2)
  {

    constexpr float len1 = DeviceConstants::len1;
    constexpr float len2 = DeviceConstants::len2;
    float delta = sqrt(sq(l2.x) + sq(l2.y));
    float c2 = -(sq(len1) + sq(len2) - sq(delta)) / (2 * len1 * len2); // minus for the outer angle

    // preventing an numerical error
    float abs_limit = 1.0 - 1.0e-8;

    if (c2 > abs_limit)
    {
      c2 = abs_limit;
    }

    if (c2 < -abs_limit)
    {
      c2 = -abs_limit;
    }

    return {acos(c2), -1 * acos(c2)};
  }

  // find the first joint angle in two cases.
  float get_j1(float j2, const Position &l2, float delta)
  {

    constexpr float len1 = DeviceConstants::len1;
    constexpr float len2 = DeviceConstants::len2;

    const float c1 = ((len1 + len2 * cos(j2)) * l2.x - len2 * sin(j2) * l2.y) / sq(delta);
    const float s1 = ((len1 + len2 * cos(j2)) * l2.y + len2 * sin(j2) * l2.x) / sq(delta);
    return atan2(s1, c1);
  }

  float get_j1_2(float j2, const Position &l2, float delta)
  {

    constexpr float len1 = DeviceConstants::len1;
    constexpr float len2 = DeviceConstants::len2;

    const float c1 = ((len1 + len2 * cos(j2)) * l2.x + len2 * sin(j2) * l2.y) / sq(delta);
    const float s1 = ((len1 + len2 * cos(j2)) * l2.y - len2 * sin(j2) * l2.x) / sq(delta);
    return atan2(s1, c1);
  }

  // Check the solution.
  bool check_solution(const JointAngles &solution, const ArmPose &goal_pose)
  {

    //  check joint angle ranges
    for (int j = 0; j < 3; j++)
    {
      float angle = solution.arr[j];
      if (angle < DeviceConstants::joint_limit_radians[j][0])
      {
        return false;
      }
      else if (angle > DeviceConstants::joint_limit_radians[j][1])
      {
        return false;
      }
    }

    //  check forward kinematics
    ArmPose fk_check = fk_solver(solution);
    float err_chk = sq(fk_check.pos[0] - goal_pose.pos[0]) + sq(fk_check.pos[1] - goal_pose.pos[1]) + sq(fk_check.pos[2] - goal_pose.pos[2]);

    if (fabs(err_chk) > 0.001)
    {
      return false;
    }

    return true;
  }

  // Compute the angle of the third joint and check the feasibility of solution.
  std::vector<JointAngles> get_all_solutions(
      const Vec1d &j1_values,
      const Vec1d &j2_values,
      const ArmPose &goal_pose)
  {
    std::vector<JointAngles> r;
    for (int i = 0; i < j2_values.size(); ++i)
    {
      float j1 = j1_values[i];
      float j2 = j2_values[i];
      float j3 = goal_pose.pos[2] - j1 - j2;
      JointAngles candidate({j1, j2, j3});

      // Check the feasibility of the solution.
      if (check_solution(candidate, goal_pose))
      {
        r.push_back(candidate);
      }
    }
    return r;
  }

  /* solve inverse kinematics */
  std::vector<JointAngles> get_eef_pose_solutions(const ArmPose &goal_pose)
  {

    // calculate unique l2 from the goal pose
    Position l2 = get_l2(goal_pose);

    // Calculate multiple possible values for j2
    constexpr float len1 = DeviceConstants::len1;
    constexpr float len2 = DeviceConstants::len2;
    float delta = sqrt(sq(l2.x) + sq(l2.y));
    float c2 = -(sq(len1) + sq(len2) - sq(delta)) / (2 * len1 * len2); // minus for the outer angle

    // preventing an numerical error
    float abs_limit = 1.0 - 1.0e-8;

    if (c2 > abs_limit)
    {
      c2 = abs_limit;
    }

    if (c2 < -abs_limit)
    {
      c2 = -abs_limit;
    }

    // elbow up / down
    Vec1d j2_candi = {acos(c2), -1 * acos(c2)};

    // Calculate multiple possible values for j1
    Vec1d j1_values;
    Vec1d j2_values;
    for (auto j2 : j2_candi)
    {
      j1_values.push_back(get_j1(j2, l2, delta));
      j2_values.push_back(j2);
      j1_values.push_back(get_j1_2(j2, l2, delta));
      j2_values.push_back(j2);
    }

    // find all feasible solutions.
    std::vector<JointAngles> solutions = get_all_solutions(j1_values, j2_values, goal_pose);

    return solutions;
  }

  /* solve forward kinematics */
  ArmPose fk_solver(JointAngles current)
  {

    float q[3];

    for (int i = 0; i < 3; i++)
    {
      q[i] = current.arr[i];
    }
    // Transformation.
    Transform t;
    t.translateY(DeviceConstants::len0);
    t.rotateZ(q[0]);
    t.translateX(DeviceConstants::len1);
    t.rotateZ(q[1]);
    t.translateX(DeviceConstants::len2);
    t.rotateZ(q[2]);
    t.translateX(DeviceConstants::len3);

    return getpose3D(t);
  }

} // namespace kinematics
