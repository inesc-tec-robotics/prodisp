//// inspireed by http://donkerdump.nl/node/207
//#ifndef Model3ds_H__
//#define Model3ds_H__

//#include "prerequisites.h"
//#include "renderable.h"
//#include "vertex_buffer_object.h"
//#include <lib3ds/file.h>
//#include <GL/glut.h>
//#include <string>

//#include <boost/scoped_ptr.hpp>
//#include <boost/shared_ptr.hpp>


//// 3DS model class
//class Model3ds : public Renderable
//{
//public:
//	Model3ds(std::string filename, Colour ambient, Colour diffuse,
//				 Colour specular=Colour::BLACK, GLfloat shineness=0.0);
//	virtual void bind() const;
//	virtual void setAppearance() const;
//	~Model3ds();

//private:
//	Colour mAmbient, mDiffuse, mSpecular;
//	GLfloat mShineness;
//	boost::scoped_ptr<VertexBufferObject> mVertexVBO;
//	boost::scoped_ptr<VertexBufferObject> mNormalVBO;

//	void InitialiseVBO(Lib3dsFile * model);
//	void CountVertices(Lib3dsFile * model);
//};

//typedef boost::shared_ptr<Model3ds> Model3ds_ptr;

//inline void Model3ds::bind() const	// Bind the vbo with the normals
//{
//	mVertexVBO->bind();
//	mNormalVBO->bind();
//}

//inline void Model3ds::setAppearance() const
//{
//	glMaterialfv(GL_FRONT,GL_AMBIENT,mAmbient.c);
//	glMaterialfv(GL_FRONT,GL_DIFFUSE,mDiffuse.c);
//	glMaterialfv(GL_FRONT,GL_SPECULAR,mSpecular.c);
//	glMaterialf(GL_FRONT,GL_SHININESS,mShineness);
//}

//#endif // Model3ds_H__
