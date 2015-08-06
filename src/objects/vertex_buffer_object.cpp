#include "objects/vertex_buffer_object.h"

#include <iostream>
#include <boost/scoped_array.hpp>
#include <cassert>


using namespace std;

VertexBufferObject::VertexBufferObject(GLenum size, GLenum vboType, unsigned int maxNumVertices,
	GLenum usage, const GLvoid *pointer, VboEnum vboEnum)
	: mSize(size), mVboType(vboType), mVboEnum(vboEnum)
{

	assert( mSize==2 || mSize==3 || mSize==4);
	assert( mVboType==GL_SHORT ||mVboType==GL_INT ||mVboType==GL_FLOAT ||mVboType==GL_DOUBLE );

	// Create and bind texture
	glEnableClientState(GL_VERTEX_ARRAY);
	glGenBuffers(1, &mVertexBufferObject);
	bind();
	
	// FIXME: Most probably the sizeof(mVboType) is plain wrong!!!
	glBufferData(GL_ARRAY_BUFFER,mSize*maxNumVertices*sizeof(mVboType),pointer,usage);
}

VertexBufferObject::~VertexBufferObject()
{
	glDeleteBuffers(1,&mVertexBufferObject);
}

void VertexBufferObject::update(const void* dataArray, unsigned int numVertices)
{
	assert(dataArray);
	bind();

	// FIXME: Most probably the sizeof(mVboType) is plain wrong!!!
	glBufferSubData(GL_ARRAY_BUFFER,0,mSize*numVertices*sizeof(mVboType),dataArray);
}
