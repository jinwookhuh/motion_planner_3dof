#ifndef IK__H_
#define IK__H_

#include <vector>
#include <tuple>
#include "device_constants.hpp"

namespace kinematics
{

	using Vec1d = std::vector<float>;

	// Position in 2D space.
	struct Position
	{
		float x, y; // meter

		Position();
		Position(float x, float y);

		friend Position operator+(const Position &left_, const Position &right_)
		{
			return {left_.x + right_.x, left_.y + right_.y};
		}
		friend Position operator-(const Position &left_, const Position &right_)
		{
			return {left_.x - right_.x, left_.y - right_.y};
		}

		float distance(const Position &p) const;
	};

	// Pose of the arm.
	struct ArmPose
	{
		// Coordinates in the robot frame.
		float pos[3];

		ArmPose(){};

		ArmPose(float x, float y, float theta)
		{
			pos[0] = x;
			pos[1] = y;
			pos[2] = theta;
		}

		ArmPose(float *pose)
		{
			for (int idx = 0; idx < 3; idx++)
			{
				pos[idx] = pose[idx];
			}
		}
	};

	// Joint angles of the arm.
	struct JointAngles
	{
		float arr[3];

		JointAngles() {}

		JointAngles(float act1, float act2, float act3)
		{
			arr[0] = act1;
			arr[1] = act2;
			arr[2] = act3;
		}

		JointAngles(float *angles)
		{
			for (int idx = 0; idx < 3; idx++)
			{
				arr[idx] = angles[idx];
			}
		}
	};

	// Homogeneous transformation
	class Transform
	{
	public:
		Transform();

		void clear();
		Transform &translateX(float x = 0);
		Transform &translateY(float y = 0);
		Transform &rotateZ(float a = 0);
		Transform transform6D(const float p[3]);

		float &operator()(int i, int j);
		const float operator()(int i, int j) const;

	private:
		float t[4][4];
	};

	float sq(const float v);
	// Check if the ik solution using forward kinematics.
	bool check_solution(const JointAngles &solution);
	// Compute arm pose from Transform.
	ArmPose getpose3D(const Transform &t1);
	// forward kinematics.
	ArmPose fk_solver(JointAngles current);
	// solutions of inverse kinematics.
	std::vector<JointAngles> get_eef_pose_solutions(const ArmPose &goal_pose);
	std::vector<JointAngles> get_all_solutions(const Vec1d &j1j2_values, float goal_angle);
	ArmPose fk_solver(JointAngles current);
} //end name space

#endif // IK__H_
