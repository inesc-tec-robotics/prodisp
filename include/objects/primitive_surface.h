#ifndef PRIMITIVE_H__
#define PRIMITIVE_H__

#include "objects/object.h"

#include <vector>

struct Face
{
public:
	Vector3* vertices_[3];
	Vector3* normals_[3];
	Vector2* uvs_[3];

	Face(Vector3* v0,  Vector3* v1,  Vector3* v2,
		  Vector3* n0,  Vector3* n1,  Vector3* n2,
		  Vector2* uv0 = NULL, Vector2* uv1 = NULL, Vector2* uv2 = NULL)
	{
		vertices_[0] = v0;
		vertices_[1] = v1;
		vertices_[2] = v2;

		normals_[0] = n0;
		normals_[1] = n1;
		normals_[2] = n2;

		uvs_[0] = uv0;
		uvs_[1] = uv1;
		uvs_[2] = uv2;
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
	void setTexture(const std::string& filename);

protected:
	int addNormal(float x, float y, float z);
	int addUV(float x, float y);
	void addFace(int v0,  int v1,  int v2,
					 int n0,  int n1,  int n2);
	void addFace(int v0,  int v1,  int v2,
					 int n0,  int n1,  int n2,
					 int uv0, int uv1, int uv2);


private:
	bool drawVertices();
	void drawFace(Face* face);

protected:
	boost::scoped_ptr<Texture> texture_;	// if false, use colours instead

private:
	Colour c_ambient_;
	Colour c_diffuse_;
	Colour c_specular_;
	float c_shininess_;

	std::vector<Vector3*> normals_;
	std::vector<Vector2*> uvs_;
	std::vector<Face*> faces_;
};


#endif // PRIMITIVE_H__
