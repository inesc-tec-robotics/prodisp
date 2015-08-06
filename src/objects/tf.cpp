#include "objects/tf.h"

#include "opencv2/core/core.hpp"
//#include <sns.h>
//#include <amino.hpp>

using namespace std;

namespace sg
{
	Tf::Tf()
	{
	}

	Tf::~Tf()
	{
	}


	void Tf::setTranslation(float x, float y, float z)
	{
		translation_ = Vector3(x, y, z);
	}

	void Tf::setRotation(float x, float y, float z)
	{
		rotation_deg_ = Vector3(x, y, z);
	}

	void Tf::incTranslation(float x, float y, float z)
	{
		translation_.c[0] += x;
		translation_.c[1] += y;
		translation_.c[2] += z;
	}

	void Tf::incRotation(float x, float y, float z)
	{
		rotation_deg_.c[0] += x;
		rotation_deg_.c[1] += y;
		rotation_deg_.c[2] += z;
	}

	void Tf::reset(void)
	{
		translation_.c[0] = 0;
		translation_.c[1] = 0;
		translation_.c[2] = 0;
		rotation_deg_.c[0] = 0;
		rotation_deg_.c[1] = 0;
		rotation_deg_.c[2] = 0;
	}

	void Tf::set(const MathOp::Transform& pose)
	{
		cv::Vec3d t = pose.getTranslation();
		cv::Vec3d r = pose.getEuler(MathOp::ZYX_EULER);
		for (int i = 0; i < 3; i++)
		{
			translation_.c[i] = t[i];
		}
		rotation_deg_.c[2] = r[0] * RAD2DEG;	// Z
		rotation_deg_.c[1] = r[1] * RAD2DEG;	// Y
		rotation_deg_.c[0] = r[2] * RAD2DEG;	// X - Euler vinkler
	}

	MathOp::Transform Tf::getMathOp(void) const
	{
		return MathOp::Transform(
					rotation_deg_.c[2]*DEG2RAD, rotation_deg_.c[1]*DEG2RAD, rotation_deg_.c[0]*DEG2RAD,
					translation_.c[0],  translation_.c[1],  translation_.c[2]);
	}
}





