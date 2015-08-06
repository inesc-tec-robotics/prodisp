#include "objects/primitive.h"

using namespace std;

Primitive::Primitive() : Object(),
	c_ambient_(Colour::WHITE),
	c_diffuse_(Colour::WHITE),
	c_specular_(Colour::BLACK),
	c_shininess_(0.0)
{

}

Primitive::Primitive(const Primitive& src) : Object(src),
	c_ambient_(src.c_ambient_),
	c_diffuse_(src.c_diffuse_),
	c_specular_(src.c_specular_),
	c_shininess_(src.c_shininess_)
{
	// Copy vertices:
	for (int i = 0; i < (int)src.vertices_.size(); ++i)
		addVertex(src.vertices_[i]->x,
					 src.vertices_[i]->y,
					 src.vertices_[i]->z);

	// Copy normals:
	for (int i = 0; i < (int)src.normals_.size(); ++i)
		addNormal(src.normals_[i]->x,
					 src.normals_[i]->y,
					 src.normals_[i]->z);

	// Copy faces:
	for (int f = 0; f < (int)src.faces_.size(); ++f)
	{
		int dest_v_i[3];
		int dest_n_i[3];

		for (int p = 0; p < 3; ++p)
		{
			Vector3* src_v = src.faces_[f]->vertices_[p];
			Vector3* src_n = src.faces_[f]->normals_[p];

			// Find the same vertex in this:
			for (int i = 0; i < (int)vertices_.size(); ++i)
			{
				Vector3* dest_v = vertices_[i];
				if (*src_v == *dest_v)
				{
					// Found!
					dest_v_i[p] = i;
					break;
				}
			}

			// Find the same normal in this:
			for (int i = 0; i < (int)normals_.size(); ++i)
			{
				Vector3* dest_n = normals_[i];
				if (*src_n == *dest_n)
				{
					// Found!
					dest_n_i[p] = i;
					break;
				}
			}

		}
		// Add new face:
		addFace(dest_v_i[0], dest_v_i[1], dest_v_i[2],
				  dest_n_i[0], dest_n_i[1], dest_n_i[2]);
	}
}

Primitive::~Primitive()
{
	for (int i = 0; i < (int)vertices_.size(); ++i)
		delete vertices_[i];

	for (int i = 0; i < (int)normals_.size(); ++i)
		delete normals_[i];

	for (int i = 0; i < (int)faces_.size(); ++i)
		delete faces_[i];
}

int Primitive::addVertex(float x, float y, float z)
{
	vertices_.push_back( new Vector3(x, y, z) );

	return ((int)vertices_.size()-1);
}

int Primitive::addNormal(float x, float y, float z)
{
	normals_.push_back( new Vector3(x, y, z) );

	return ((int)normals_.size()-1);
}

void Primitive::addFace(int v0, int v1, int v2,
								int n0, int n1, int n2)
{
	faces_.push_back( new Face(vertices_[v0], vertices_[v1], vertices_[v2],
										normals_[n0], normals_[n1], normals_[n2]) );
}

bool Primitive::drawVertices()
{
	glMaterialfv(GL_FRONT, GL_AMBIENT, c_ambient_.c);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, c_diffuse_.c);
	glMaterialfv(GL_FRONT, GL_SPECULAR, c_specular_.c);
	glMaterialfv(GL_FRONT, GL_SHININESS, &c_shininess_);

	glMaterialfv(GL_BACK, GL_AMBIENT, c_ambient_.c);
	glMaterialfv(GL_BACK, GL_DIFFUSE, c_diffuse_.c);
	glMaterialfv(GL_BACK, GL_SPECULAR, c_specular_.c);
	glMaterialfv(GL_BACK, GL_SHININESS, &c_shininess_);

	for (int i = 0; i < (int)faces_.size(); i++)
	{
		drawFace( faces_[i] );
	}
	return true;
}

void Primitive::drawFace(Face* face)
{
	glBegin(GL_TRIANGLES);
		for (int i = 0; i < 3; i++)
		{
			glNormal3fv(face->normals_[i]->c);
			glVertex3fv(face->vertices_[i]->c);
		}
	glEnd();
}

void Primitive::scaleVerticesAlong(float x, float y, float z)
{
	for (int i = 0; i < (int)vertices_.size(); ++i)
	{
		vertices_[i]->x *= x;
		vertices_[i]->y *= y;
		vertices_[i]->z *= z;
	}
}

void Primitive::setColour(const Colour &colour)
{
	setColour(colour, colour);
}

void Primitive::setColour(const Colour& ambient, const Colour& diffuse,
								  const Colour& specular, float shininess)
{
	c_ambient_ = ambient;
	c_diffuse_ = diffuse;
	c_specular_ = specular;
	c_shininess_ = shininess;
}







