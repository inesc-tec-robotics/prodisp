#include "configuration.h"
#include "objects/cad_model.h"

#include "objects/glm.h"

using namespace std;

CadModel::CadModel(std::string obj_name)
	: wireframe_mode_(false)
	, meshmodel_(0)
	, dl_(0)
{
	// Build object path:
	stringstream obj_path;
	obj_path << Configuration::getInstance().getPath() << "/models/" << obj_name;
	if (obj_name.find(".obj") == string::npos)
		obj_path << ".obj";

	readModel(obj_path.str());
}

CadModel::~CadModel()
{
	if (meshmodel_)
		glmDelete(meshmodel_);

	if(dl_)
	  glDeleteLists(dl_, 1);
}

bool CadModel::readModel(std::string obj_path)
{
	if(meshmodel_ != NULL)
	  free(meshmodel_);

	// Read object:
	meshmodel_ = glmReadOBJ((char *)obj_path.c_str());

	if(!meshmodel_)
	{
	  std::cerr << "Error in reading OBJ file: '" << obj_path << "'" << std::endl;
	  return false;
	}
	glmFacetNormals(meshmodel_);          // (re)calculate face normals
	glmVertexNormals(meshmodel_, 90.0);   // (re)calculate vertex normals

	// generate a display list
//	dl_ = glGenLists(1);
	dl_ = glmList(meshmodel_, GLM_MATERIAL | GLM_SMOOTH);
	if(!dl_)
	{
	  std::cerr << "Fail to create a display list." << std::endl;
	  return false;
	}


	// store drawing function in the display list
//	glNewList(dl_, GL_COMPILE);
//		for(unsigned int i=0; i<meshmodel_->numtriangles; i++)
//		{
//			int v1 = meshmodel_->triangles[i].vindices[0];
//			int v2 = meshmodel_->triangles[i].vindices[1];
//			int v3 = meshmodel_->triangles[i].vindices[2];

//			float x1 = meshmodel_->vertices[3*v1+0];
//			float y1 = meshmodel_->vertices[3*v1+1];
//			float z1 = meshmodel_->vertices[3*v1+2];

//			float x2 = meshmodel_->vertices[3*v2+0];
//			float y2 = meshmodel_->vertices[3*v2+1];
//			float z2 = meshmodel_->vertices[3*v2+2];

//			float x3 = meshmodel_->vertices[3*v3+0];
//			float y3 = meshmodel_->vertices[3*v3+1];
//			float z3 = meshmodel_->vertices[3*v3+2];

//			float nx = meshmodel_->facetnorms[i+0];
//			float ny = meshmodel_->facetnorms[i+1];
//			float nz = meshmodel_->facetnorms[i+2];

//			float nx1 = meshmodel_->normals[3*v1+0];
//			float ny1 = meshmodel_->normals[3*v1+1];
//			float nz1 = meshmodel_->normals[3*v1+2];

//			float nx2 = meshmodel_->normals[3*v2+0];
//			float ny2 = meshmodel_->normals[3*v2+1];
//			float nz2 = meshmodel_->normals[3*v2+2];

//			float nx3 = meshmodel_->normals[3*v3+0];
//			float ny3 = meshmodel_->normals[3*v3+1];
//			float nz3 = meshmodel_->normals[3*v3+2];

//			glBegin(GL_POLYGON);
//				glNormal3f(nx, ny, nz);
////				glNormal3f(nx1, ny1, nz1);
//				glVertex3f(x1, y1, z1);
////				glNormal3f(nx2, ny2, nz2);
//				glVertex3f(x2, y2, z2);
////				glNormal3f(nx3, ny3, nz3);
//				glVertex3f(x3, y3, z3);
//			glEnd();
//		}
//	glEndList();

	return true;
}

bool CadModel::drawVertices()
{
	// Wire frame mode:
	if (wireframe_mode_)
	{
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glDepthFunc(GL_LEQUAL);
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		glLineWidth(1);
	}
	else
	{
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		glEnable(GL_LIGHTING);
		glEnable(GL_DEPTH_TEST);
	}
	glCallList(dl_);
	return true;
}
