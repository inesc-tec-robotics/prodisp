#include "texture.h"

#include <iostream>
#include <boost/scoped_array.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "LoggerSys.hpp"

using namespace std;

//Texture::Texture(const IplImage* image, GLint internalFormat, GLenum inputFormat)
//	: mInternalFormat(internalFormat), mInputFormat(inputFormat)
//{
//	create(image);
//}

Texture::Texture(const std::string& path)
{
	FDEBUG("Loading texture from: '" << path << "'");

	cv::Mat mat = cv::imread(path, CV_LOAD_IMAGE_UNCHANGED);
//	IplImage* image = cvLoadImage(filename, CV_LOAD_IMAGE_UNCHANGED);
	if (!mat.data)
	{
		FERROR("Failed to load texture from: '" << path << "'");
		throw runtime_error("could not load image");
	}
	else if (!mat.isContinuous())
	{
		FWARN("Image data is not continuous for texture: '" << path << "'! May cause failure.");
	}

	create(mat);
}

Texture::~Texture()
{
	glDeleteTextures(1, &mTexture);
}

void Texture::create(const cv::Mat &mat)
{
//	assert(image->origin == 0 && "origin must be top-left");

	width_  = mat.cols;
	height_ = mat.rows;

//	cout << "Creating texture with dimensions " << mWidth << 'x' << mHeight <<
//		", " << image->nChannels << " channels." << endl;

	// Create and bind texture
	glGenTextures(1, &mTexture);
	bind();

	// Set texture interpolation methods for minification and magnification:
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

	//	Set pixel alignment requirements for the start of each pixel row in memory:
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

	// Set incoming texture format:
	GLenum input_colour_format = GL_BGR;
	if (mat.channels() == 1)
	{
		input_colour_format = GL_LUMINANCE;
	}

	// Upload texture
	glTexImage2D(GL_TEXTURE_2D,
			0,							// Pyramid level (for mip-mapping) - 0 is the top level
			GL_RGB,					// Internal colour format to convert to
			width_,					// Image width
			height_,					// Image height
			0,							// Border width in pixels (can either be 1 or 0)
			input_colour_format, // Input image format (i.e. GL_RGB, GL_RGBA, GL_BGR etc.)
			GL_UNSIGNED_BYTE,	   // Image data type
			mat.ptr());				// The actual image data itself
}

//void Texture::update(const IplImage* image)
//{
//	assert(image->origin == 0 && "origin must be top-left");

//	bind();

//	glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, mWidth, mHeight,
//			mInputFormat, GL_UNSIGNED_BYTE, image->imageData);
//}
