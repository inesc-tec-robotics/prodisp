#include "objects/plane.h"
#include "texture.h"

#include "LoggerSys.hpp"

using namespace std;

Plane::Plane(float width, float height, int divisions)
	: width_(width)
	, height_(height)
	, divisions_(divisions)
{

}

void Plane::createPrimitive()
{
	float incX = width_ / (float)divisions_;
	float incY = height_ / (float)divisions_;

	addNormal(0, 0, -1);

	for (float x = 0; x < divisions_; ++x)
	{
		for (float y = 0; y < divisions_; ++y)
		{
			// Create vertices:
			int v0 = addVertex((x+1)*incX-width_/2, (y+1)*incY-height_/2,  0);
			int v1 = addVertex(  x  *incX-width_/2, (y+1)*incY-height_/2,  0);
			int v2 = addVertex((x+1)*incX-width_/2,	y  *incY-height_/2,	0);
			int v3 = addVertex(  x  *incX-width_/2,	y  *incY-height_/2,	0);

			// Create uvs (only used if texture is set):
			int uv0 = addUV((x+1)/(float)divisions_, (y+1)/(float)divisions_);
			int uv1 = addUV(  x  /(float)divisions_, (y+1)/(float)divisions_);
			int uv2 = addUV((x+1)/(float)divisions_,	 y  /(float)divisions_);
			int uv3 = addUV(  x  /(float)divisions_,	 y  /(float)divisions_);

			// Create faces:
			addFace(v0, v2, v3, 0, 0, 0, uv0, uv2, uv3);
			addFace(v3, v1, v0, 0, 0, 0, uv3, uv1, uv0);
		}
	}
}

void Plane::setTextureAutoAspect(const string& filename)
{
	// Load image and set texture:
	setTexture(filename);

	// Adjust aspect ratio of shape:
	float shape_aspect = width_/height_;
	float texture_aspect = (float)texture_->getWidth() / (float)texture_->getHeight();

//	cout << "shape_aspect: [: " << width_ << ", " << height_
//		  << "]" << endl;
//	cout << "texture_aspect: [: " << texture_->getWidth() << ", " << texture_->getHeight()
//		  << "]" << endl;

	if (shape_aspect != texture_aspect)
	{
		FWARN("Aspect ratio of texture '" << filename << "' does not match. "
				<< "shape_aspect: " << width_ << "/" << height_ << " = " << shape_aspect
				<< ", texture_aspect: " << texture_->getWidth() << "/" << texture_->getHeight() << " = " << texture_aspect
				<< ". Adjusting shape.");

		float scale_width  = 1.0;
		float scale_height = 1.0;

		if (shape_aspect > texture_aspect)
		{
			// Shape too wide
			float new_width = height_ * texture_aspect;
//			cout << "height_: " << height_
//				  << ", texture_aspect: " << texture_aspect
//				  << ", new_width: " << new_width << endl;
			scale_width = new_width/width_;
			width_ = new_width;
		}
		else
		{
			// Shape too high
			float new_height = (float)width_ / texture_aspect;
			scale_height = new_height/height_;
			height_ = new_height;
		}

//		cout << "Scale width: " << scale_width << ", height = " << scale_height << endl;

		// Adjust vertices:
		for (vector<Vector3*>::iterator v_it = vertices_.begin(); v_it != vertices_.end(); ++v_it)
		{
			(*v_it)->c[0] *= scale_width;
			(*v_it)->c[1] *= scale_height;
		}
	}
}












