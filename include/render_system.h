#ifndef RenderSystem_H__
#define RenderSystem_H__

#include "prerequisites.h"
#include "singleton.h"
#include "objects/renderable.h"

#include <string>
#include <GL/gl.h>
#include <opencv2/highgui/highgui.hpp>
//#include <cv.h>

class RenderSystem : public Singleton<RenderSystem>
{
public:
	RenderSystem();
	~RenderSystem();

private:
	void initGLUT();
	void initGL();

public:
	void run();

	// Projections
	void setupProjection2D();
	void setupProjection3D(bool reset_projection = true);
	void resetViewport(void);

	// Enable/disable full screen:
	void updateWindowMode(void);

	// Primitives
	void drawQuad(const Rectf& coords = Rectf::UNIT, const Rectf& texCoords = Rectf::UNIT);
	void drawPoint(const Pointf &p,  const Colour& colour, int size = 2);
	void drawCircle(const cv::Point2f &p,  const Colour& colour, float size = 0.08);
	void drawLine(const cv::Point2f &start, const cv::Point2f &stop, const Colour& colour, int width = 4);
	void blendQuad(const Rectf& coords = Rectf::UNIT, const Colour& colour = Colour::WHITE, const Rectf& texCoords = Rectf::UNIT);

	void drawTexturedQuad(const Texture& texture,
			const Rectf& coords = Rectf::UNIT, const Colour& colour = Colour::WHITE);
	
	void drawTexturedQuad(const Texture& texture, const CvRect& roi,
			const Rectf& coords = Rectf::UNIT, const Colour& colour = Colour::WHITE);

	void blendTexturedQuad(const Texture& texture,
			const Rectf& coords = Rectf::UNIT, const Colour& colour = Colour::WHITE);

	void blendTexturedQuad(const Texture& texture, const CvRect& roi,
			const Rectf& coords = Rectf::UNIT, const Colour& colour = Colour::WHITE);

	void DrawRenderable(const Renderable& renderable);
	void DrawRenderable(const Renderable& renderable, Colour colour);

	// Coordinate transformations
	Pointf pixel2viewport(const Pointi& p) const;
	cv::Point2f ndc2viewport(const cv::Point2f &p) const;
	cv::Point2f ndc2eyespaceOrtho(const cv::Point2f &p) const;

	void pushTransform(const MathOp::Transform& transform);

	int getWidth() const { return viewport_width_; }
	int getHeight() const { return viewport_height_; }
	int getStartX() const { return viewport_startx_; }
	int getStartY() const { return viewport_starty_; }

	void onDisplay();
	void onReshape(int width, int height);

	void drawTextQuad(const Pointi& topLetft, const Pointi& botRight, void* font, const std::string& s);
	void renderBitmapString(float x, float y, void* font, const std::string& s);

private:
	int viewport_width_;
	int viewport_height_;
	int viewport_startx_;
	int viewport_starty_;

	GLdouble proj_matrix_[16];
};

inline Pointf RenderSystem::pixel2viewport(const Pointi& p) const
{
	return Pointf(
			static_cast<float>(p.x)/viewport_width_,
			static_cast<float>(p.y)/viewport_height_);
}

inline cv::Point2f RenderSystem::ndc2viewport(const cv::Point2f& p) const
{
	return cv::Point2f(
			(p.x+1.0)/2.0 * viewport_width_ + viewport_startx_,
			(p.y+1.0)/2.0 * viewport_height_ + viewport_starty_ );
}



#endif // RenderSystem_H__
