#ifndef PRIMITIVE_POINTS_H
#define PRIMITIVE_POINTS_H

#include "objects/object.h"

#include <vector>

class PointCloud : public Object
{
public:
	PointCloud(Colour colour, float point_size);
	PointCloud(const PointCloud& src);	// Copy constructor
	virtual ~PointCloud() {}

	void		resetPoints(CvPoint3D32f* points, int num_points);
	void		setPointSize(float point_size) { point_size_ = point_size; }
	void		setColour(const Colour& colour) { colour_ = colour; }

private:
	bool		drawVertices();

protected:
	Colour	colour_;

private:
	float		point_size_;
};

#endif // PRIMITIVE_POINTS_H
