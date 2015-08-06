#ifndef LinesRenderable_H__
#define LinesRenderable_H__

#include "prerequisites.h"
#include "renderable.h"
#include "vertex_buffer_object.h"

#include <boost/noncopyable.hpp>


#include <GL/gl.h>
#include <GL/glext.h>

class LinesRenderable : public Renderable
{
public:
	LinesRenderable(Colour colour, GLfloat lineWidth, unsigned int maxNumLines,
		unsigned int dimension, GLenum vboType=GL_INT);

	virtual void bind() const;
	virtual void setAppearance() const;

	void update(const void* array, unsigned int numLines);
private:
	Colour mColour;
	GLfloat mLineWidth;
	VertexBufferObject mVBO;
};

inline LinesRenderable::LinesRenderable(Colour colour, GLfloat lineWidth, unsigned int maxNumLines,
		unsigned int dimension, GLenum vboType)
	: Renderable(GL_LINES), mColour(colour), mLineWidth(lineWidth),
	  mVBO(dimension, vboType, 2*maxNumLines) //NOTE: 2 Vertices per line
{
}

inline void LinesRenderable::update(const void* array, unsigned int numLines)
{
	mNumVertices = 2*numLines;//NOTE: 2 Verteses per line
	mVBO.update(array, mNumVertices );
}

inline void LinesRenderable::bind() const
{
	mVBO.bind();
}

inline void LinesRenderable::setAppearance() const
{
	glColor4fv(mColour.c);
	glLineWidth(mLineWidth);
}

#endif // LinesRenderable_H__
