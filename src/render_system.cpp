#include "render_system.h"
#include "configuration.h"
#include "renderer.h"
#include "application.h"
#include "texture.h"

#include "LoggerSys.hpp"
#include <iostream>
#include <GL/gl.h>
#include <GL/freeglut.h>
#include <ros/ros.h>

using namespace std;

// Forward declarations
static void glutDisplay();
static void glutIdle();
static void glutReshape(int width, int height);
static void glutKeyboard(unsigned char key, int x, int y);
static void glutSpecial(int key, int x, int y);
static void glutMouse(int button, int state, int x, int y);

// Singleton instance
template<> RenderSystem* Singleton<RenderSystem>::msInstance = 0;

RenderSystem::RenderSystem()
{
	FDEBUG("RenderSystem::RenderSystem");

	Configuration& config = Configuration::getInstance();

	// Initialize GLUT and set initial OpenGL state
	initGLUT();
	initGL();

	// Init 3D projection matrix:
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glFrustum(config.getProjLeft(),
				 config.getProjRight(),
				 config.getProjBottom(),
				 config.getProjTop(),
				 config.getProjNear(),
				 config.getProjFar());
	GLdouble projection_matrix[16];
	glGetDoublev(GL_PROJECTION_MATRIX, projection_matrix);
	config.setProjMatrix(projection_matrix);
}

RenderSystem::~RenderSystem()
{
//	FDEBUG("RenderSystem::~RenderSystem");		// Causes log4cxx error
}

void RenderSystem::initGLUT()
{
	// NOTE: glutInit() has already been called by Application ctor
	FDEBUG("RenderSystem::initGLUT");

	const Configuration& config = Configuration::getInstance();

	viewport_width_ = config.getInitWindowWidth();
	viewport_height_ = config.getInitWindowHeight();
	FDEBUG("Window dimensions: " << viewport_width_ << "x" << viewport_height_);

	// Initialize window size and display mode:
	glutInitWindowSize(viewport_width_, viewport_height_);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
	
	// Create rendering window:
	glutCreateWindow("ProDisp: CARLoS Pojector Display System");

	// Set full screen:
	updateWindowMode();
	
	// Register callbacks:
	glutDisplayFunc(glutDisplay);	// Can be forced by glutPostRedisplay
	glutIdleFunc(glutIdle);
	glutReshapeFunc(glutReshape);
	glutKeyboardFunc(glutKeyboard);
	glutSpecialFunc(glutSpecial);
	glutMouseFunc(glutMouse);
}

void RenderSystem::initGL()
{
	FDEBUG("RenderSystem::initGL");
	glShadeModel(GL_SMOOTH);


//	GLfloat light_diffuse[] = {1.0, 0.0, 0.0, 1.0};  /* Red diffuse light. */
//	GLfloat light_position[] = {1.0, 1.0, 1.0, 0.0};  /* Infinite light location. */

//	/* Enable a single OpenGL light. */
//	glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
//	glLightfv(GL_LIGHT0, GL_POSITION, light_position);
//	glEnable(GL_LIGHT0);

	// Two sided lightning:
//	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);

	/********* Global ambient *********/
//	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, (Colour::WHITE).c);
	/********* Global ambient *********/

	/********* Nice 3D lights *********/
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, (Colour::WHITE*0.05).c);

	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHT1);
	
	GLfloat dir0[] = {-.1, 1, 0.3, 0};	// {0, 0, 1, 0} = directional: negative-z
	glLightfv(GL_LIGHT0, GL_POSITION, dir0);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, (Colour::WHITE*0.7).c);
	glLightfv(GL_LIGHT0, GL_SPECULAR, (Colour::WHITE).c);

	GLfloat dir1[] = {0.4, -0.1, 1, 0};	// {0, 0, 1, 0} = directional: negative-z
	glLightfv(GL_LIGHT1, GL_POSITION, dir1);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, (Colour::WHITE*0.4).c);
	glLightfv(GL_LIGHT1, GL_SPECULAR, (Colour::WHITE).c);
	/********* Nice 3D lights *********/



//	glEnableClientState(GL_VERTEX_ARRAY);
//	glEnableClientState(GL_NORMAL_ARRAY);
}

void RenderSystem::run()
{
	FDEBUG("RenderSystem::run");

	// Enable return on window-close:
	glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_CONTINUE_EXECUTION);

	// Let GLUT run the main loop (never returns)
	glutMainLoop();
}

void RenderSystem::setupProjection2D()
{
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_LIGHTING);

	// Set projection matrix
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	static const Rectf proj2d = Configuration::getInstance().getProj2dEyeArea();
	gluOrtho2D(proj2d.left, proj2d.right, proj2d.bottom, proj2d.top);

	// Set modelview matrix
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void RenderSystem::setupProjection3D(bool reset_projection)
{
	// Set projection matrix
	glMatrixMode(GL_PROJECTION);
	if (reset_projection)
		glLoadIdentity();
	Configuration& config = Configuration::getInstance();

	glFrustum(config.getProjLeft(),
				 config.getProjRight(),
				 config.getProjBottom(),
				 config.getProjTop(),
				 config.getProjNear(),
				 config.getProjFar());

//	glLoadMatrixd(config.getProjM());
	GLdouble projection[16];
	glGetDoublev(GL_PROJECTION_MATRIX, projection);

	// Enable lighting:
	glEnable(GL_LIGHTING);
	glEnable(GL_DEPTH_TEST);
//	glEnable(GL_NORMALIZE);

	// Set modelview matrix:
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void RenderSystem::resetViewport(void)
{
	glViewport(viewport_startx_, viewport_starty_, viewport_width_, viewport_height_);
}

void RenderSystem::updateWindowMode(void)
{
	const Configuration& config = Configuration::getInstance();

	if (config.getFullScreen())
	{
		glutFullScreen();
	}
	else
	{
//		glutPositionWindow(0,0);
		glutReshapeWindow(config.getInitWindowWidth(), config.getInitWindowHeight());
	}
}

void RenderSystem::drawQuad(const Rectf& coords, const Rectf& texCoords)
{
	glBegin(GL_QUADS);

		glTexCoord2f(texCoords.left, texCoords.top);
		glVertex2f(coords.left, coords.top);

		glTexCoord2f(texCoords.left, texCoords.bottom);
		glVertex2f(coords.left, coords.bottom);

		glTexCoord2f(texCoords.right, texCoords.bottom);
		glVertex2f(coords.right, coords.bottom);

		glTexCoord2f(texCoords.right, texCoords.top);
		glVertex2f(coords.right, coords.top);

	glEnd();
}

void RenderSystem::drawPoint(const Pointf& p,  const Colour& colour, int size)
{
	glColor4fv(colour.c);
	glPointSize(size);
	
	glBegin(GL_POINTS);
		glVertex2f(p.x, p.y);
	glEnd();
}

void RenderSystem::drawCircle(const cv::Point2f &p,  const Colour& colour, float size)
{
	int triangles = 20; // number of triangles
	float step = 2*M_PI/triangles;
//	float radius_y = radius_x * Configuration::getInstance().getProjAspectRatio();

	glColor4fv(colour.c);
	glBegin(GL_TRIANGLE_FAN);
		glVertex2f(p.x, p.y);	// center
		for (int i = 0; i <= triangles; ++i)
		{
			glVertex2f( p.x + size*cos(i*step),
							p.y + size*sin(i*step) );
		}
	glEnd();
}

void RenderSystem::drawLine(const cv::Point2f &start, const cv::Point2f &stop, const Colour& colour, int width)
{
	glLineWidth(width);
	glColor4fv(colour.c);
	glBegin(GL_LINES);
		glVertex2f(start.x, start.y);
		glVertex2f(stop.x, stop.y);
	glEnd();
}

void RenderSystem::blendQuad(const Rectf& coords, const Colour& colour,
									  const Rectf& texCoords)
{
	glColor4fv(colour.c);

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	
	drawQuad(coords,texCoords);
	
	glDisable(GL_BLEND);
}

void RenderSystem::drawTexturedQuad(const Texture& texture,
		const Rectf& coords, const Colour& colour)
{
	glColor4fv(colour.c);

	glEnable(GL_TEXTURE_2D);
	texture.bind();

	drawQuad(coords, Rectf::UNIT);

	glDisable(GL_TEXTURE_2D);
}

void RenderSystem::drawTexturedQuad(const Texture& texture, const CvRect& roi,
		const Rectf& coords, const Colour& colour)
{
	Rectf texCoords, croppedCoords;

	// Calculate texture coordinates cropped to RoI
	texCoords.left   = static_cast<float>(roi.x)/texture.getWidth();
	texCoords.top    = static_cast<float>(roi.y)/texture.getHeight();
	texCoords.right  = static_cast<float>(roi.width)/texture.getWidth() + texCoords.left;
	texCoords.bottom = static_cast<float>(roi.height)/texture.getHeight() + texCoords.top;

	float w = coords.right  - coords.left;
	float h = coords.bottom - coords.top;

	// Calculate coordinates cropped to RoI
	croppedCoords.left   = coords.left + w*texCoords.left;
	croppedCoords.right  = coords.left + w*texCoords.right;
	croppedCoords.top    = coords.top  + h*texCoords.top;
	croppedCoords.bottom = coords.top  + h*texCoords.bottom;

	glColor4fv(colour.c);

	glEnable(GL_TEXTURE_2D);
	texture.bind();
	
	drawQuad(croppedCoords, texCoords);

	glDisable(GL_TEXTURE_2D);
}

void RenderSystem::blendTexturedQuad(const Texture& texture,
		const Rectf& coords, const Colour& colour)
{
	//glDisable(GL_DEPTH_TEST); // not required

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	drawTexturedQuad(texture, coords, colour);

	glDisable(GL_BLEND);
}

void RenderSystem::blendTexturedQuad(const Texture& texture, const CvRect& roi,
		const Rectf& coords, const Colour& colour)
{
	//glDisable(GL_DEPTH_TEST); // not required

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	drawTexturedQuad(texture, roi, coords, colour);

	glDisable(GL_BLEND);
}

void RenderSystem::DrawRenderable(const Renderable& renderable)
{
	renderable.bind();
	renderable.setAppearance();
	glDrawArrays(renderable.getPrimitiveMode(),0,renderable.getNumVertices());
}

void RenderSystem::DrawRenderable(const Renderable& renderable, Colour colour)
{
	renderable.bind();
	glMaterialfv(GL_FRONT,GL_AMBIENT,colour.c);
	glMaterialfv(GL_FRONT,GL_DIFFUSE,colour.c);
	glMaterialfv(GL_FRONT,GL_SPECULAR,Colour::BLACK.c);
	glMaterialf(GL_FRONT,GL_SHININESS,0.0f);
	glMaterialfv(GL_FRONT,GL_EMISSION,(colour*0.1).c);
	glDrawArrays(renderable.getPrimitiveMode(),0,renderable.getNumVertices());
	glMaterialfv(GL_FRONT,GL_EMISSION,Colour::BLACK.c);
}

void RenderSystem::pushTransform(const MathOp::Transform& transform)
{
	// Note: Must be followed by glPopMatrix()
	double c[16] = {
		transform(0,0), transform(1,0), transform(2,0), 0.0f,
		transform(0,1), transform(1,1), transform(2,1), 0.0f,
		transform(0,2), transform(1,2), transform(2,2), 0.0f,
		transform(0,3), transform(1,3), transform(2,3), 1.0f
	};
	glPushMatrix();
	glMultMatrixd(c);
}

void RenderSystem::onDisplay()
{
	//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // video covers it all

	// Delegate to Renderer
	Renderer::getInstance().onDisplay();

	glutSwapBuffers();
}

void RenderSystem::onReshape(int width, int height)
{
	// Clear entire window
	glViewport(0, 0, width, height);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	float windowAspect = static_cast<float>(width)/height;
	float projAspect = Configuration::getInstance().getProjAspectRatio();

	// Calculate fixed aspect ratio viewport
	if (windowAspect < projAspect)
	{
		viewport_width_  = width;
		viewport_height_ = static_cast<int>(roundf(width/projAspect));

		viewport_startx_ = 0;
		viewport_starty_ = (height - viewport_height_)/2;
	}
	else
	{
		viewport_width_  = static_cast<int>(roundf(height*projAspect));
		viewport_height_ = height;

		viewport_startx_ = (width - viewport_width_)/2;
		viewport_starty_ = 0;
	}

	// Set the viewport
	glViewport(viewport_startx_, viewport_starty_, viewport_width_, viewport_height_);
}

void RenderSystem::drawTextQuad(const Pointi& topLeft, const Pointi& botRight, void* font, const string& s)
{
	// Character sizes:
	int CHARPIX_Y;
	if (font == GLUT_BITMAP_HELVETICA_18)
		CHARPIX_Y = 19;
	else if (font == GLUT_BITMAP_HELVETICA_12)
		CHARPIX_Y = 13;
	else if (font == GLUT_BITMAP_HELVETICA_10)
		CHARPIX_Y = 11;
	else if (font == GLUT_BITMAP_8_BY_13)
		CHARPIX_Y = 14;
	else if (font == GLUT_BITMAP_9_BY_15)
		CHARPIX_Y = 16;
	else
		CHARPIX_Y = 19;
	const float LINEHEIGHT = (float)CHARPIX_Y/viewport_height_;
	
	glColor3f(0.0,0.0,0.0);
	
	unsigned int boxWidth = botRight.x - topLeft.x;
	float imgPlaceX = (float)topLeft.x/viewport_width_;
	float imgPlaceY = (float)(topLeft.y+CHARPIX_Y)/viewport_height_;
	glRasterPos2f(imgPlaceX, imgPlaceY);
	
	unsigned int strPos = 0;
	while (strPos < s.length())
	{
		// Find line with maximum possible length:
		unsigned int lineWidth = 0;
		unsigned int linePos = 0;
		string line;
		while (strPos+linePos < s.length())
		{
			lineWidth += glutBitmapWidth(font, s.at(strPos+linePos));
			if (lineWidth > boxWidth)
				break;
			line.push_back(s.at(strPos+linePos));
			linePos++;
		}
		
		// Search for line change:
		size_t charPos = line.find_first_of("\n",0);
		if (charPos != string::npos)
		{
			line = line.substr(0,charPos);
			strPos = strPos+charPos+1;
		} else 
		{
			// Search for spaces (except for the last line):
			if (linePos+strPos >= s.length())	// Last line
			{
				strPos = s.length();
			} else
			{
				charPos = line.find_last_of(" ",line.length());
				if (charPos != string::npos)	// Space found
				{
					line = line.substr(0,charPos);
					strPos = strPos+charPos+1;
				} else
				{
					strPos = strPos+linePos;
				}
			}
		}
		renderBitmapString(imgPlaceX,imgPlaceY,font,line);
		imgPlaceY = imgPlaceY + LINEHEIGHT;
	}
}

void RenderSystem::renderBitmapString(float x, float y, void* font, const string& s)
{
	glRasterPos2f(x, y);
	for (unsigned int i=0; i < s.length(); i++) {
		glutBitmapCharacter(font, s.at(i));
	}
}

cv::Point2f RenderSystem::ndc2eyespaceOrtho(const cv::Point2f &p) const
{
	static const Rectf proj2d = Configuration::getInstance().getProj2dEyeArea();
	static const float scalex = proj2d.width()/2.0;
	static const float scaley = proj2d.height()/2.0;
	return cv::Point2f(
			p.x * scalex,
			p.y * scaley );
}

static void glutDisplay()
{
	RenderSystem::getInstance().onDisplay();
}

static void glutIdle()
{
	RenderSystem::getInstance().onDisplay();

	// Update transforms from other nodes:
	ros::spinOnce();
}

static void glutReshape(int width, int height)
{
	RenderSystem::getInstance().onReshape(width, height);
}

static void glutKeyboard(unsigned char key, int x, int y)
{
	FDEBUG("glutKeyboard: char='" << key << "', int='" << (int)key << "'");
	Application::getInstance().onKey(key, x, y);
}

static void glutSpecial(int key, int x, int y)
{
	FDEBUG("glutSpecial: '" << key << "'");
	Application::getInstance().onArrow(key, x, y);
}

static void glutMouse(int button, int state, int x, int y)
{
	Application::getInstance().onMouse(button, state, x, y);
}
