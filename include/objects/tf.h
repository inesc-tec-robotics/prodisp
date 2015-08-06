#ifndef TRANSFORM_H
#define TRANSFORM_H

//#include "node.h"
#include "common.h"

static const float RAD2DEG = 180.0/M_PI;
static const float DEG2RAD = M_PI/180.0;

namespace sg
{
	class Tf
	{
	public:
		Tf();
		virtual ~Tf();

	public:
		void setTranslation(float x, float y, float z);
		void setTranslation(const Vector3& translation) { translation_ = translation; }
		void setRotation(float x, float y, float z);
		void setRotation(const Vector3& rotation_deg) { rotation_deg_ = rotation_deg; }
		void incTranslation(float x, float y, float z);
		void incRotation(float x, float y, float z);

		void reset(void);
		void set(const MathOp::Transform& pose);
		MathOp::Transform getMathOp(void) const;

	public:
		Vector3 translation_;
		Vector3 rotation_deg_;
	};
}


#endif // TRANSFORM_H

