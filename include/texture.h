#ifndef Texture_H__
#define Texture_H__

#include "prerequisites.h"

#include <boost/noncopyable.hpp>
#include <GL/gl.h>
//#include <cv.h>
#include <opencv2/highgui/highgui.hpp>

class Texture : boost::noncopyable
{
public:
//	Texture(const IplImage* image, GLint internalFormat = GL_RGB, GLenum inputFormat = GL_BGR_EXT);
	Texture(const std::string& path);
	~Texture();

	void bind() const;
//	void update(const IplImage* image);

	unsigned getWidth() const { return width_; }
	unsigned getHeight() const { return height_; }

private:
	GLuint mTexture;
	unsigned width_;
	unsigned height_;

	void create(const cv::Mat &mat);
};

inline void Texture::bind() const
{
	glBindTexture(GL_TEXTURE_2D, mTexture);
}

#endif // Texture_H__
