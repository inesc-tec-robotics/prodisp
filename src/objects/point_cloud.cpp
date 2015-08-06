#include "objects/point_cloud.h"

using namespace std;

PointCloud::PointCloud(Colour colour, float point_size)
	: Object()
	, colour_(colour)
	, point_size_(point_size)
{ }

PointCloud::PointCloud(const PointCloud& src)
	: Object(src)
	, colour_(src.colour_)
	, point_size_(src.point_size_)
{ }

void PointCloud::resetPoints(CvPoint3D32f* points, int num_points)
{
	// Remove old points:
	for (vector<Vector3*>::iterator v = vertices_.begin(); v != vertices_.end(); ++v)
	{
		delete (*v);
	}
	vertices_.clear();

	// Insert new points:
	vertices_.reserve(num_points);
	for(int i = 0; i < num_points; i++)
	{
		 vertices_.push_back( new Vector3(points[i].x, points[i].y, points[i].z) );
	}
}

bool PointCloud::drawVertices()
{
	glDisable(GL_LIGHTING);
	glPointSize(point_size_);
	glColor3fv(colour_.c);

	glBegin(GL_POINTS);
	for (vector<Vector3*>::iterator v = vertices_.begin(); v != vertices_.end(); ++v)
	{
		glVertex3fv((*v)->c);
	}
	glEnd();

	glEnable(GL_LIGHTING);

	return true;
}

