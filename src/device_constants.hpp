#ifndef CONSTANT__H_
#define CONSTANT__H_

#define M_PI 3.14159265358979323846

namespace kinematics
{

	// paramters
	namespace DeviceConstants
	{
		constexpr float len0 = 0.3;
		constexpr float len1 = 1.2;
		constexpr float len2 = 1.3;
		constexpr float len3 = 0.8;

		constexpr float joint_limit_radians[3][2] = {
			{0, M_PI},
			{-M_PI / 2.0, M_PI / 2.0},
			{-M_PI / 2.0, M_PI / 2.0},
		};
	};

} // namespace kinematics

#endif // CONSTANT__H_
