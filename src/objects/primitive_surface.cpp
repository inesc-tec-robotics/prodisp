#include "objects/primitive_surface.h"
#include "texture.h"

#include <ros/this_node.h>
#include <ros/package.h>

using namespace std;

PrimitiveSurface::PrimitiveSurface()
	: c_ambient_(Colour::WHITE)
	, c_diffuse_(Colour::WHITE)
	, c_specular_(Colour::BLACK)
	, c_shininess_(0.0)
{

}

PrimitiveSurface::PrimitiveSurface(const PrimitiveSurface& src)
	: Object(src)
	, c_ambient_(src.c_ambient_)
	, c_diffuse_(src.c_diffuse_)
	, c_specular_(src.c_specular_)
	, c_shininess_(src.c_shininess_)
{
	// Copy normals:
	for (int i = 0; i < (int)src.normals_.size(); ++i)
		addNormal(src.normals_[i]->c[0],
					 src.normals_[i]->c[1],
					 src.normals_[i]->c[2]);

	// Copy uvs:
	for (int i = 0; i < (int)src.uvs_.size(); ++i)
		addUV(src.uvs_[i]->c[0],
				src.uvs_[i]->c[1]);

	// Copy texture:
	if (src.texture_)
		texture_ = src.texture_;

	// Copy faces:
	for (int f = 0; f < (int)src.faces_.size(); ++f)
	{
		int dest_v_i[3];
		int dest_n_i[3];
		int dest_uv_i[3];

		for (int p = 0; p < 3; ++p)
		{
			Vector3* src_v = src.faces_[f]->vertices_[p];
			Vector2* src_uv= src.faces_[f]->uvs_[p];
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

			// Find the same uv in this:
			if (src.texture_)
			{
				for (int i = 0; i < (int)uvs_.size(); ++i)
				{
					Vector2* dest_uv = uvs_[i];
					if (*src_uv == *dest_uv)
					{
						// Found!
						dest_uv_i[p] = i;
						break;
					}
				}
			}

		}

		// Add new face:
		if (src.texture_)
		{
			addFace(dest_v_i[0], dest_v_i[1], dest_v_i[2],
					  dest_n_i[0], dest_n_i[1], dest_n_i[2]);
		}
		else
		{
			addFace(dest_v_i[0], dest_v_i[1], dest_v_i[2],
					  dest_n_i[0], dest_n_i[1], dest_n_i[2],
					  dest_uv_i[0], dest_uv_i[1], dest_uv_i[2]);
		}
	}
}

PrimitiveSurface::~PrimitiveSurface()
{
	for (int i = 0; i < (int)normals_.size(); ++i)
		delete normals_[i];

	for (int i = 0; i < (int)uvs_.size(); ++i)
		delete uvs_[i];

	for (int i = 0; i < (int)faces_.size(); ++i)
		delete faces_[i];
}

int PrimitiveSurface::addNormal(float x, float y, float z)
{
	normals_.push_back( new Vector3(x, y, z) );

	return ((int)normals_.size()-1);
}

int PrimitiveSurface::addUV(float x, float y)
{
	uvs_.push_back( new Vector2(x, y) );

	return ((int)uvs_.size()-1);
}

void PrimitiveSurface::addFace(int v0, int v1, int v2,
										 int n0, int n1, int n2)
{
	faces_.push_back( new Face(vertices_[v0], vertices_[v1], vertices_[v2],
										normals_[n0], normals_[n1], normals_[n2]) );
}

void PrimitiveSurface::addFace(int v0,  int v1,  int v2,
										 int n0,  int n1,  int n2,
										 int uv0, int uv1, int uv2)
{
	faces_.push_back( new Face(vertices_[v0], vertices_[v1], vertices_[v2],
										normals_[n0],  normals_[n1],  normals_[n2],
										uvs_[uv0],     uvs_[uv1],     uvs_[uv2]     ) );
}

bool PrimitiveSurface::drawVertices()
{
	glMaterialfv(GL_FRONT, GL_AMBIENT, c_ambient_.c);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, c_diffuse_.c);
	glMaterialfv(GL_FRONT, GL_SPECULAR, c_specular_.c);
	glMaterialfv(GL_FRONT, GL_SHININESS, &c_shininess_);

	glMaterialfv(GL_BACK, GL_AMBIENT, c_ambient_.c);
	glMaterialfv(GL_BACK, GL_DIFFUSE, c_diffuse_.c);
	glMaterialfv(GL_BACK, GL_SPECULAR, c_specular_.c);
	glMaterialfv(GL_BACK, GL_SHININESS, &c_shininess_);

	if (texture_)
	{
		// Enable textures:
		glEnable(GL_TEXTURE_2D);
		texture_->bind();
	}

	for (int i = 0; i < (int)faces_.size(); i++)
	{
		drawFace( faces_[i] );
	}

	if (texture_)
	{
		glDisable(GL_TEXTURE_2D);
	}

	return true;
}

void PrimitiveSurface::drawFace(Face* face)
{
	glBegin(GL_TRIANGLES);
		if (texture_)
		{
			for (int i = 0; i < 3; i++)
			{
				glTexCoord2fv(face->uvs_[i]->c);
				glNormal3fv(face->normals_[i]->c);
				glVertex3fv(face->vertices_[i]->c);
			}
		}
		else
		{
			for (int i = 0; i < 3; i++)
			{
				glNormal3fv(face->normals_[i]->c);
				glVertex3fv(face->vertices_[i]->c);
			}
		}
	glEnd();
}

void PrimitiveSurface::setColour(const Colour &colour)
{
	setColour(colour, colour);
}

void PrimitiveSurface::setColour(const Colour& ambient, const Colour& diffuse,
								  const Colour& specular, float shininess)
{
	c_ambient_ = ambient;
	c_diffuse_ = diffuse;
	c_specular_ = specular;
	c_shininess_ = shininess;
}

void PrimitiveSurface::setTexture(const string& filename)
{
	// Get path:
	string name = ros::this_node::getName();
	std::string path = ros::package::getPath(name.substr(1)) + "/models/" + filename;

	// Load image and set texture:
	texture_.reset( new Texture(path) );
}







