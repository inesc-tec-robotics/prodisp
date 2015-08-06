#ifndef PLANE_H__
#define PLANE_H__

#include "objects/primitive_surface.h"

class Plane : public PrimitiveSurface
{
public:
	Plane(float width, float height, int divisions);
	virtual void createPrimitive();
	void setTextureAutoAspect(const std::string& filename);

private:
	float width_;
	float height_;
	int divisions_;
};

#endif // PLANE_H__
