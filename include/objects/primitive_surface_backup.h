#ifndef PRIMITIVE_H__
#define PRIMITIVE_H__

#include "objects/object.h"

#include <vector>

struct Face
{
public:
	Vector3* vertices_[3];
	Vector3* normals_[3];

	Face(Vector3* v0, Vector3* v1, Vector3* v2,
		  Vector3* n0, Vector3* n1, Vector3* n2)
	{
		vertices_[0] = v0;
		vertices_[1] = v1;
		vertices_[2] = v2;

		normals_[0] = n0;
		normals_[1] = n1;
		normals_[2] = n2;
	}
};

class PrimitiveSurface : public Object
{
public:
	PrimitiveSurface();
	PrimitiveSurface(const PrimitiveSurface& src);	// Copy constructor
	virtual ~PrimitiveSurface();

	virtual void createPrimitive(void) = 0;
	void setColour(const Colour& colour);
	void setColour(const Colour& ambient, const Colour& diffuse,
						const Colour& specular = Colour::BLACK,
						float shininess = 0.0);

protected:
	int addNormal(float x, float y, float z);
	void addFace(int v0, int v1, int v2,
					 int n0, int n1, int n2);


private:
	bool drawVertices();
	void drawFace(Face* face);

private:
	Colour c_ambient_;
	Colour c_diffuse_;
	Colour c_specular_;
	float c_shininess_;

	std::vector<Vector3*> normals_;
	std::vector<Face*> faces_;
};


#endif // PRIMITIVE_H__
