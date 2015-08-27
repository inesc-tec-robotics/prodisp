#include "renderer.h"
#include "render_system.h"
#include "interface_handler.h"
#include "configuration.h"
#include "wii_handler.h"
#include "scene.h"
#include "objects/cad_model.h"
#include "objects/model3ds.h"
#include "objects/object_composer.h"
#include "objects/cube.h"
#include "objects/plane.h"
#include "objects/position.h"
#include "objects/line_coordsys.h"
#include "objects/line_circle.h"
#include "objects/point_cloud.h"
#include "objects/cad_model.h"
#include "objects/fire_extinguisher.h"

#include "LoggerSys.hpp"
#include <GL/glut.h>
#include <iostream>
#include <boost/lexical_cast.hpp>

using namespace std;

// Singleton instance
template<> Renderer* Singleton<Renderer>::msInstance = 0;

Renderer::Renderer()
	: current_scene_num_(0)
	, active_(false)
	, held_object_(NULL)
	, mark_start_ndc_(0,0)
	, mark_stop_ndc_(0,0)
{
	FDEBUG("Renderer::Renderer");

	// Prepare for selectable objects:
//	selectable_objects_.reset( new map<GLuint, Object*> );

	// Setup scene graph:
	graph_origin_ = new Position;
	setupBaseScene();
	constructScenes();

	// Setup 2D overlayed graphics:
	setupGraphics2D();

//	ros::spinOnce();

	if (Configuration::getInstance().getEnableAtStart())
		active_ = true;
}

Renderer::~Renderer()
{
	FDEBUG("Renderer::~Renderer");

	// Make sure no scene is enabled to avoid double deletions:
	for (vector<ScenePtr>::iterator scene = scenes_.begin(); scene != scenes_.end(); scene++)
		(*scene)->disable();

	delete graph_origin_;
}

void Renderer::start(const vector<Object*>& objects)
{
	FDEBUG("Renderer::start");

	for (vector<Object*>::const_iterator obj = objects.begin(); obj != objects.end(); obj++)
	{
		(*obj)->setParent(wall_);
	}

	active_ = true;
}

void Renderer::stop(void)
{
	FDEBUG("Renderer::stop");
	if (active_)
	{
		active_ = false;

		// Clear screen:
		onDisplay();

		// Retrieve all studs:
		vector<Stud*> studs;
		set<Object*>& selectable = Object::getSelectable();
		for (set<Object*>::iterator obj_it = selectable.begin();
			  obj_it != selectable.end(); obj_it++)
		{
			if (Stud* stud = dynamic_cast<Stud*>(*obj_it))
				studs.push_back(stud);
		}
		interface_handler_.wiiTeachingComplete(studs);

		// Clean up selectable objects:
		for( set<Object*>::iterator it = selectable.begin(),
			  e = selectable.end(); it != e;  )
		{
			 Object* p = *it;
			 selectable.erase(it++);
			 delete p;
		}
//		while(!selectable.empty())
//		{
//			delete selectable.back();
//			studs.pop_back();
//		}
	}
}

/** Controls rendering of everything in the display. */
void Renderer::onDisplay()
{
	RenderSystem& rs = RenderSystem::getInstance();
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	 if (active_)
    {
        interface_handler_.refreshRosTf();

		  rs.setupProjection2D();
		  renderGraphics2D();

        rs.setupProjection3D();
        renderGraphics3D();
    }

	if (Configuration::getInstance().getGuiHelp())
		renderHelp();
}

sg::Tf& Renderer::getCamera()
{
	return scene_origin_->getTf();
}

bool Renderer::select()
{
	FDEBUG("Renderer::select");

	// Select stud:
	Object* stud_selected = chooseWallObj(getCursorOnWall());

	// Remove:
	if (stud_selected != NULL)
	{
		stud_selected->setParent(cursor_hit_);
		stud_selected->getTf().setTranslation(0,0,0);
		held_object_ = stud_selected;
		return true;
	}

	return false;
}

bool Renderer::release()
{
	FDEBUG("Renderer::release");

	if (held_object_ == NULL)
	{
		FERROR("Trying to release object, but none is held - should not happen!");
		return false;
	}

	// Release the object:
	held_object_->setParent(wall_);
	held_object_->getTf().setTranslation(cursor_hit_->getTf().translation_);
	held_object_ = NULL;

	return true;
}

bool Renderer::markStart()
{
	FDEBUG("Renderer::markStart");

	if (mark_start_ndc_ == mark_stop_ndc_)
	{
		// Start marking
		mark_start_ndc_ = cursor_->getPosNDC();
		mark_stop_ndc_  = cursor_->getPosNDC();
		return true;
	}
	else
	{
		// Clear old marking:
		mark_start_ndc_ = cv::Point2f(0,0);
		mark_stop_ndc_  = cv::Point2f(0,0);
		return false;
	}
}

bool Renderer::markStop()
{
	FDEBUG("Renderer::markStop");

	selected_objects_ = selectInside(mark_start_ndc_, mark_stop_ndc_);

	if (selected_objects_.size() == 0)
	{
		// Clear old marking:
		mark_start_ndc_ = cv::Point2f(0,0);
		mark_stop_ndc_  = cv::Point2f(0,0);
	}

	return true;
}

bool Renderer::add()
{
	FDEBUG("Renderer::add");

	// Add stud:
	Stud* stud = new Stud(Vector3(getCursorOnWall()));
	stud->setParent(wall_);

	return false;
}

bool Renderer::remove(void)
{
	FDEBUG("Renderer::remove");

	if (mark_start_ndc_ != mark_stop_ndc_)
	{
		cv::Point2f cursor = cursor_->getPosNDC();

		// Determine if cursor is inside:
		if (((cursor.x - mark_start_ndc_.x) > 0) != ((cursor.x - mark_stop_ndc_.x) > 0))
		{
			if (((cursor.y - mark_start_ndc_.y) > 0) != ((cursor.y - mark_stop_ndc_.y) > 0))
			{
				vector<Object*> objects = selectInside(mark_start_ndc_, mark_stop_ndc_);

				// Delete:
				while(!objects.empty())
				{
					delete objects.back();
					objects.pop_back();
				}

				mark_start_ndc_ = mark_stop_ndc_;
			}
		}
	}

	// Select stud:
	Object* stud_selected = chooseWallObj(getCursorOnWall());

	// Remove:
	if (stud_selected != NULL)
	{
		delete stud_selected;
		return true;
	}

	return false;
}

void Renderer::setSceneNum(int scene_num)
{
	assert(scenes_.size() > 0);

	if (scene_num < 0)
		scene_num = scenes_.size()-1;
	if (scene_num >= (int)scenes_.size())
		scene_num = 0;

	scenes_[current_scene_num_]->disable();
	scenes_[scene_num]->enable();
	current_scene_num_ = scene_num;
}

void Renderer::setupBaseScene()
{
	FDEBUG("Renderer::setupBaseScene");

	ObjectComposer* obj_comp = ObjectComposer::getInstancePtr();
	Configuration* config = Configuration::getInstancePtr();

	// Init camera:
	scene_origin_ = new Position;
	scene_origin_->setParent(graph_origin_);
	if (config->initCamAtDistance())
	  getCamera().setTranslation(0,0,-3);

	// Init projector:
	LineCoordsys* proj = new LineCoordsys;
	proj->createPrimitive();
	proj->scaleVertices(0.3);
	proj->getTf().setRotation(180, 0, 0);
	proj->setParent(scene_origin_);

	Object* wall_coordsys;
	if (config->dispWallCoordsys())
		wall_coordsys = obj_comp->getCoordBlocks(0.2);
	else
		wall_coordsys = new Position;
	wall_coordsys->getTf().setTranslation(-0.4, 0.4, 3);
	wall_coordsys->getTf().setRotation(180, 30, 0);
	wall_coordsys->setParent(proj);
	wall_ = wall_coordsys;
	interface_handler_.addRosTf("projector_link", "wall", &wall_->getTf());

	cursor_hit_ = new Position;
	cursor_hit_->setParent(wall_);

//	FireExtinguisher* fe = new FireExtinguisher(Vector3(0.3,0.3,0));
//	fe->setParent(wall_);

//	Plane* wall_plane = new Plane(0.8, 0.6, 10);
//	wall_plane->setTextureAutoAspect("fire_extinguisher.bmp");
//	wall_plane->getTf().setTranslation(0.4, 0.3, 0);
//	wall_plane->setColour(Colour(0.7, 0.7, 0.7, 0.5));
//	wall_plane->createPrimitive();
//	wall_plane->setParent(wall_);


	Plane* rotator = new Plane(.2, .2, 10);
	rotator->setColour(Colour::WHITE, Colour::WHITE, Colour::RED, 50);
	rotator->setParent(wall_);
	rotator->getTf().setTranslation(0.00, 1.00, 4.00);
	rotator->createPrimitive();
	rotator_ = rotator;

//	 Cube* c = new Cube;
//	 c->setColour(Colour::RED);
//	 c->setParent(proj);
//	 c->getTf().setTranslation(0,0,2);
//	 c->createPrimitive();
//	 c->scaleVertices(0.2);
}

void Renderer::constructScenes()
{
	FDEBUG("Renderer::constructScenes");

	scenes_.push_back(ScenePtr( new Scene(wall_) ));
//    scenes_.back()->addObject(new CadModel("hole2"));
}

void Renderer::setupGraphics2D()
{
	FDEBUG("Renderer::setupGraphics2D");

//	InterfaceHandler& ih = InterfaceHandler::getInstance();
	cursor_		 = boost::shared_ptr<Cursor>( new Cursor(this) );
	interface_handler_.bindWii(cursor_);
}

Object* Renderer::chooseWallObj(cv::Point3d cursor)
{
	FDEBUG("Renderer::chooseWallObj");

	// Find nearest stud:
	double dist_min = 0.05;	// 5 cm
	Object* obj_selected = NULL;
	const set<Object*>& selectable = Object::getSelectable();
	for (set<Object*>::iterator obj_it = selectable.begin();
		  obj_it != selectable.end(); obj_it++)
	{
		Vector3 pos = (*obj_it)->getTf().translation_;
		double dist = sqrt(
					(cursor.x-pos.c[0])*(cursor.x-pos.c[0]) +
					(cursor.y-pos.c[1])*(cursor.y-pos.c[1]) +
					(cursor.z-pos.c[2])*(cursor.z-pos.c[2]) );
		if (dist < dist_min)
		{
			dist_min = dist;
			obj_selected = (*obj_it);
		}
	}

	// Print:
	if (obj_selected != NULL)
		FDEBUG("Stud was selected " << dist_min << "m from cursor.");

	return obj_selected;
}

cv::Point3d Renderer::getCursorOnWall(void)
{
	RenderSystem& rs = RenderSystem::getInstance();

	// Get cursor pos:
	cv::Point2f cursor_ndc = cursor_->getPosNDC();

	// Get viewport coordinates:
	cv::Point2f viewport_coords = rs.ndc2viewport(cursor_ndc);

	// Get modelview matrix:
	MathOp::Transform wall_tf_mo = wall_->getTfFromOrigin();
	GLdouble wall_tf[16];
	wall_tf_mo.getGlArray(wall_tf);

	// Get projection matrix:
	GLdouble projection[16];
	Configuration::getInstance().getProjMatrixGl(projection);

	// Get viewport:
	GLint viewport[4];
	glGetIntegerv(GL_VIEWPORT, viewport);

	// Get vector:
	GLdouble vec_start[3], vec_end[3];
	gluUnProject(viewport_coords.x, viewport_coords.y, 0.0,
					 wall_tf, projection, viewport,
					 &vec_start[0], &vec_start[1], &vec_start[2]);
	gluUnProject(viewport_coords.x, viewport_coords.y, 1.0,
					 wall_tf, projection, viewport,
					 &vec_end[0], &vec_end[1], &vec_end[2]);
	cv::Vec3d vec(vec_end[0]-vec_start[0],
			vec_end[1]-vec_start[1],
			vec_end[2]-vec_start[2]);


	// Calc hit point in local space:
	double t = -vec_start[2]/vec[2];
	cv::Point3d hit_wall_coords(vec_start[0] + t*vec[0],vec_start[1] + t*vec[1], 0.0);

//	// Print:
//	cout << "cursor_ndc = " << cursor_ndc << endl
//			<< "viewport_coords = " << viewport_coords << endl
////			<< "wall_tf = " << wall_tf_mo.print()
//			<< "viewport = [" << viewport[0] << ", " << viewport[1] << ", " << viewport[2] << ", " << viewport[3] << "]" << endl
//			<< "vec_start = [" << vec_start[0] << ", " << vec_start[1] << ", " << vec_start[2] << "]" << endl
//			<< "vec_end = [" << vec_end[0] << ", " << vec_end[1] << ", " << vec_end[2] << "]" << endl
//			<< "vec = " << vec << endl
//			<< "   hit = " << hit_wall_coords << ", t = " << t << endl;

	return hit_wall_coords;
}

void Renderer::renderGraphics3D()
{
//	RenderSystem& rs = RenderSystem::getInstance();
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	if (cursor_)
	{
		cv::Point3d p = getCursorOnWall();
		cursor_hit_->getTf().setTranslation(p.x, p.y, p.z);
	}

//	rotator_->getTf().incRotation(0.15,0.3,0.5);
	rotator_->getTf().incRotation(0.0,0.5,0.0);
	graph_origin_->draw();

}

void Renderer::renderGraphics2D()
{
	RenderSystem& rs = RenderSystem::getInstance();

	if (cursor_)
	{
		cv::Point2f p = cursor_->getPos();

		// Draw fix lines:
		if (cursor_->lockedX())
			rs.drawLine(cv::Point2f(p.x, p.y+0.05), cv::Point2f(p.x, p.y-0.05), Colour::GREEN);
		if (cursor_->lockedY())
			rs.drawLine(cv::Point2f(p.x+0.05, p.y), cv::Point2f(p.x-0.05, p.y), Colour::GREEN);

		// Draw cursor dot:
		rs.drawCircle(p, cursor_->getColour(), 0.016);

		// Update marking area:
		if (cursor_->getState() == CursorStates::Marking)
			mark_stop_ndc_ = cursor_->getPosNDC();

		// Draw marking area:
		if (mark_start_ndc_ != mark_stop_ndc_)
		{
			Rectf coords(rs.ndc2eyespaceOrtho(mark_start_ndc_),
							 rs.ndc2eyespaceOrtho(mark_stop_ndc_) );
			rs.blendQuad(coords, Colour(1.0, 1.0, 1.0, 0.6));
		}
	}
}

vector<Object*> Renderer::selectInside(cv::Point2f p1, cv::Point2f p2)
{
	FDEBUG("Renderer::selectInside");

	vector<Object*> ret_val;

	// Get projection matrix:
	GLdouble projection[16];
	Configuration::getInstance().getProjMatrixGl(projection);

	// Get viewport:
	GLint ndc[4];
	ndc[0] = -1;
	ndc[1] = -1;
	ndc[2] = 2;
	ndc[3] = 2;

//	cout << "ndc = ["
//		  << ndc[0] << ", "
//		  << ndc[1] << ", "
//		  << ndc[2] << ", "
//		  << ndc[3] << "]" << endl;
//	cout << "p1= " << p1 << endl;
//	cout << "p2= " << p2 << endl;

	// Temporary variables:
	GLdouble viewport_loc[3];
	GLdouble modelview[16];

	const set<Object*>& selectable = Object::getSelectable();
	for (set<Object*>::iterator obj_it = selectable.begin(); obj_it != selectable.end(); ++obj_it)
	{
		// Get modelview matrix:
		(*obj_it)->getTfFromOrigin().getGlArray(modelview);

		// Project:
		gluProject(0, 0, 0, modelview, projection, ndc,
					  &viewport_loc[0], &viewport_loc[1], &viewport_loc[2]);

//		cout << "viewport_loc = ["
//			  << viewport_loc[0] << ", "
//			  << viewport_loc[1] << ", "
//			  << viewport_loc[2] << "]" << endl;

		// Determine if inside:
		if (((viewport_loc[0] - p1.x) > 0) != ((viewport_loc[0] - p2.x) > 0))
		{
			if (((viewport_loc[1] - p1.y) > 0) != ((viewport_loc[1] - p2.y) > 0))
			{
				cout << "SELECTED!!!!" << endl;
				ret_val.push_back(*obj_it);
			}
		}
	}

	return ret_val;
}

void Renderer::renderHelp()
{
//	const Configuration& config = Configuration::getInstance();

	RenderSystem& rs = RenderSystem::getInstance();

	// Draw large white box
	static GLfloat transparency = 0.8;
	static const int mainWidth = 500;
	static const int mainHeight = 410;
	static const int border = 5;
	Pointi mainA((rs.getWidth() - mainWidth) / 2, (rs.getHeight() - mainHeight)/2); // upper left
	Pointi mainB(mainA.x + mainWidth, mainA.y + mainHeight); // lower right
	Rectf coords(rs.pixel2viewport(mainA), rs.pixel2viewport(mainB));
	rs.blendQuad(coords, Colour(0.8,0.8,0.8,transparency));
	
	// Build help text
	stringstream str;
	str<< "ProDisp\n"
		<< "    by alborg University for CARLoS\n\n"

		<< "    Key     Description (current state)\n"
		<< "    ---     ---------------------------\n"

		<< "\nGeneral:\n"
		<< "    F1, H   [H]elp screen\n"
		<< "    ESC     Quit\n"

		<< "\nWindow Control:\n"
		<< "    F       [F]ull screen (toggle)\n"

		<< "\nNavigation:\n"
		<< "    P       Move to [P]rojector view\n"
		<< "    PgUp    Move forward\n"
		<< "    PgDown  Move backwards\n"
		<< "    ARROWS  Navigate in the X/Y plane\n";


	// Render help text
	Pointi a(mainA.x + border, mainA.y + border); // upper left
	Pointi b(mainB.x - border, mainB.y - border); // lower right
	rs.drawTextQuad(a, b, GLUT_BITMAP_8_BY_13, str.str());
}




//// Select using selection buffer:
//bool Renderer::selectInside(cv::Point2f location)
//{
//	FDEBUG("Renderer::graspObject");

//	RenderSystem& rs = RenderSystem::getInstance();

////	cout << "NDC: " << location.x << ", " << location.y << endl;

//	cv::Point2f window_coords(location.x*rs.getWidth(), location.y*rs.getHeight());
////	cout << "window_coords: " << window_coords << endl;

//	// Array for the returned hit records:
//	static const GLsizei MAXHITS = 512;
//	GLuint selectBuf[MAXHITS];
//	glSelectBuffer(MAXHITS, selectBuf);

//	// Enter selection mode:
//	glRenderMode(GL_SELECT);

//	// Initialize name stack:
//	glInitNames();
//	glPushName(0);

//	// Modify projection matrix to restrict drawing to a small region:
//	glMatrixMode (GL_PROJECTION);
//	glLoadIdentity ();
//	GLint viewport[4];
//	glGetIntegerv(GL_VIEWPORT, viewport);
//	gluPickMatrix(window_coords.x, viewport[3] - window_coords.y,
//					  5.0, 5.0, viewport);

//	// Find selections:
//	rs.setupProjection3D(false);
//	renderGraphics3D();

//	GLint hits = glRenderMode(GL_RENDER);

//	// Print hits:
//	GLuint names, *ptr;
//	cout << "hits: " << hits << endl;
//	ptr = (GLuint *) selectBuf;
//	for (unsigned int i = 0; i < (GLuint)hits; i++)
//	{
//		names = *ptr;
//		if (names == 1 && (*(ptr+3)) == 0)
//		{
//			ptr += 4;
//			continue;
//		}
//		cout << "  n_names="	<< names; ptr++;
//		cout << ", z=["	<< (float)*ptr/0x7fffffff; ptr++;
//		cout << ", "		<< (float)*ptr/0x7fffffff << "]"; ptr++;
//		cout << ", hit_names=[";
//		for (unsigned int j = 0; j < names; j++)
//		{
//			cout << *ptr;
//			if (j != names-1)
//				cout << ", ";
//			ptr++;
//		}
//		cout << "]" << endl;
//	}


//	return false;
//}
