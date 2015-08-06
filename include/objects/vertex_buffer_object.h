#ifndef VertexBufferObject_H__
#define VertexBufferObject_H__

#include "prerequisites.h"

#include <boost/noncopyable.hpp>


#include <GL/gl.h>
#include <GL/glext.h>

enum VboEnum
{
	VERTEX,
	NORMAL
};

class VertexBufferObject : boost::noncopyable
{
public:
	VertexBufferObject(GLenum size, GLenum vboType, unsigned int maxNumVertices,
		GLenum usage=GL_DYNAMIC_DRAW, const GLvoid *pointer=0, VboEnum vboEnum=VERTEX);
	~VertexBufferObject();

	void bind() const;
	void update(const void* dataArray, unsigned int numVertices);

	int getLength() const { return mLength; }

private:
	GLuint mVertexBufferObject;
	unsigned mLength;
	GLenum mSize;	// Specifies number of coord/vertex. Must be 2, 3, or 4. Initial value is 4.
	GLenum mVboType;
	VboEnum mVboEnum;
};


inline void VertexBufferObject::bind() const
{

	glBindBuffer(GL_ARRAY_BUFFER, mVertexBufferObject);
	
	switch (mVboEnum)
	{
		case VERTEX:
			// The pointer for the vertexs is NULL which means that OpenGL will use the currently bound vbo
			glVertexPointer(mSize,mVboType,0,NULL);
			break;
		case NORMAL:
			glNormalPointer(mVboType,0,NULL);
			break;
	}
			
}

#endif // VertexBufferObject_H__
