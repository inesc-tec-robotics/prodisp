#ifndef Renderable_H__
#define Renderable_H__

#include "prerequisites.h"

#include <boost/noncopyable.hpp>


#include <GL/gl.h>

class Renderable : boost::noncopyable
{
public:
	Renderable(GLenum primitiveMode, GLsizei numVertices=0);
	virtual ~Renderable();

	virtual void bind() const =0;
	virtual void setAppearance() const =0;

	GLenum getPrimitiveMode() const { return mPrimitiveMode; }
	GLsizei getNumVertices() const { return mNumVertices; }

protected:
	GLsizei mNumVertices;

private:
	GLenum mPrimitiveMode;
};

inline Renderable::Renderable(GLenum primitiveMode, GLsizei numVertices)
	: mNumVertices(numVertices), mPrimitiveMode(primitiveMode)
{
	assert( primitiveMode==GL_LINES || primitiveMode==GL_TRIANGLES );
	assert( numVertices >= 0 );
}

inline Renderable::~Renderable()
{
}

#endif // Renderable_H__
